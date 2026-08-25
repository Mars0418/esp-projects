#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define DRIVER_STBY GPIO_NUM_5

#define LINE_LEFT_OUTER_GPIO  GPIO_NUM_13
#define LINE_LEFT_INNER_GPIO  GPIO_NUM_14
#define LINE_RIGHT_INNER_GPIO GPIO_NUM_21
#define LINE_RIGHT_OUTER_GPIO GPIO_NUM_39

#define ENCODER_A_PHASE_A_GPIO GPIO_NUM_16
#define ENCODER_A_PHASE_B_GPIO GPIO_NUM_17
#define ENCODER_C_PHASE_A_GPIO GPIO_NUM_2
#define ENCODER_C_PHASE_B_GPIO GPIO_NUM_1

#define PWM_FREQUENCY_HZ 20000
#define PWM_DUTY          260
#define COMMAND_TIMEOUT_MS 600
#define CONTROL_PERIOD_MS   10
#define LINE_STABLE_SAMPLES  2
#define LINE_REPORT_INTERVAL_MS 200
#define LINE_BASE_DUTY        180
#define LINE_MAX_DUTY         240
#define LINE_KP_DUTY           14
#define LINE_D_TRIM_DUTY        0
#define LINE_PULSE_ON_MS        70
#define LINE_PULSE_CYCLE_MS     70
#define LINE_LOST_TIMEOUT_MS 2000
#define LINE_TURN_DUTY         200
#define LINE_TURN_INNER_DUTY   110
#define LINE_TURN_MIN_MS       100
#define LINE_TURN_TIMEOUT_MS  1200
#define ENCODER_REPORT_MS      500
#define BALANCE_UPDATE_MS      100
#define BALANCE_NORMALIZE      256
#define BALANCE_KP               2
#define BALANCE_KI_DIV           8
#define BALANCE_INTEGRAL_MAX    80
#define BALANCE_TRIM_MAX        25

typedef struct {
    gpio_num_t pwm_pin;
    gpio_num_t in1_pin;
    gpio_num_t in2_pin;
    ledc_channel_t pwm_channel;
} motor_t;

typedef enum {
    MOTION_STOP,
    MOTION_FORWARD,
    MOTION_REVERSE,
    MOTION_LEFT,
    MOTION_RIGHT,
} motion_t;

typedef enum {
    SHARP_TURN_NONE,
    SHARP_TURN_LEFT,
    SHARP_TURN_RIGHT,
} sharp_turn_t;

typedef struct {
    const char *name;
    gpio_num_t phase_a;
    gpio_num_t phase_b;
    volatile int32_t count;
    volatile uint8_t previous_state;
} encoder_t;

enum {
    MOTOR_A = 0,
    MOTOR_B = 1,
    MOTOR_D = 2,
};

static const char *TAG = "keyboard_remote";

static const motor_t motors[] = {
    {GPIO_NUM_6,  GPIO_NUM_15, GPIO_NUM_7,  LEDC_CHANNEL_0},
    {GPIO_NUM_11, GPIO_NUM_9,  GPIO_NUM_10, LEDC_CHANNEL_1},
    {GPIO_NUM_40, GPIO_NUM_42, GPIO_NUM_41, LEDC_CHANNEL_2},
};

static motion_t current_motion = MOTION_STOP;
static int64_t command_deadline_us;
static bool line_follow_enabled;
static int last_line_error;
static int64_t line_lost_since_us;
static uint8_t last_line_control_state = 0xff;
static bool line_lost_stop_reported;
static bool line_has_been_seen;
static sharp_turn_t sharp_turn;
static int64_t sharp_turn_since_us;
static bool line_lost_search_active;
static sharp_turn_t line_lost_search_turn;
static int64_t line_pulse_epoch_us;
static encoder_t encoders[] = {
    {"A-right", ENCODER_A_PHASE_A_GPIO, ENCODER_A_PHASE_B_GPIO, 0, 0},
    {"C-left",  ENCODER_C_PHASE_A_GPIO, ENCODER_C_PHASE_B_GPIO, 0, 0},
};
static int64_t balance_last_update_us;
static int32_t balance_previous_a;
static int32_t balance_previous_c;
static int balance_integral;
static int balance_trim;

static const int8_t quadrature_delta[16] = {
     0,  1, -1,  0,
    -1,  0,  0,  1,
     1,  0,  0, -1,
     0, -1,  1,  0,
};

static uint8_t encoder_read_state(const encoder_t *encoder)
{
    return ((uint8_t)gpio_get_level(encoder->phase_a) << 1) |
           (uint8_t)gpio_get_level(encoder->phase_b);
}

static void encoder_gpio_isr(void *arg)
{
    encoder_t *encoder = (encoder_t *)arg;
    const uint8_t current_state = encoder_read_state(encoder);
    const uint8_t transition =
        (encoder->previous_state << 2) | current_state;
    encoder->count += quadrature_delta[transition];
    encoder->previous_state = current_state;
}

static void configure_encoders(void)
{
    const gpio_config_t config = {
        .pin_bit_mask = (1ULL << ENCODER_A_PHASE_A_GPIO) |
                        (1ULL << ENCODER_A_PHASE_B_GPIO) |
                        (1ULL << ENCODER_C_PHASE_A_GPIO) |
                        (1ULL << ENCODER_C_PHASE_B_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&config));
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    for (size_t i = 0; i < sizeof(encoders) / sizeof(encoders[0]); ++i) {
        encoders[i].count = 0;
        encoders[i].previous_state = encoder_read_state(&encoders[i]);
        ESP_ERROR_CHECK(gpio_isr_handler_add(encoders[i].phase_a,
                                              encoder_gpio_isr,
                                              &encoders[i]));
        ESP_ERROR_CHECK(gpio_isr_handler_add(encoders[i].phase_b,
                                              encoder_gpio_isr,
                                              &encoders[i]));
    }
}

static int clamp_int(int value, int minimum, int maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static void reset_wheel_balance(int64_t now_us)
{
    balance_last_update_us = now_us;
    balance_previous_a = encoders[0].count;
    balance_previous_c = encoders[1].count;
    balance_integral = 0;
    balance_trim = 0;
}

/* Limited PI synchronizer. It compares encoder movement normalized by each
 * wheel's requested PWM, so the outer line controller can still request
 * different wheel speeds on a curve. Positive trim slows A and speeds C. */
static void update_wheel_balance(uint32_t a_request, uint32_t c_request,
                                 int64_t now_us)
{
    if (a_request == 0 || c_request == 0) {
        reset_wheel_balance(now_us);
        return;
    }
    if (now_us - balance_last_update_us <
        (int64_t)BALANCE_UPDATE_MS * 1000) {
        return;
    }

    const int32_t count_a = encoders[0].count;
    const int32_t count_c = encoders[1].count;
    const int32_t delta_a = abs(count_a - balance_previous_a);
    const int32_t delta_c = abs(count_c - balance_previous_c);
    balance_previous_a = count_a;
    balance_previous_c = count_c;
    balance_last_update_us = now_us;

    const int normalized_a =
        (int)(delta_a * BALANCE_NORMALIZE / (int32_t)a_request);
    const int normalized_c =
        (int)(delta_c * BALANCE_NORMALIZE / (int32_t)c_request);
    const int error = normalized_a - normalized_c;

    balance_integral = clamp_int(balance_integral + error,
                                 -BALANCE_INTEGRAL_MAX,
                                 BALANCE_INTEGRAL_MAX);
    balance_trim = clamp_int(BALANCE_KP * error +
                                 balance_integral / BALANCE_KI_DIV,
                             -BALANCE_TRIM_MAX,
                             BALANCE_TRIM_MAX);
}

static void configure_output_low(gpio_num_t pin)
{
    ESP_ERROR_CHECK(gpio_reset_pin(pin));
    ESP_ERROR_CHECK(gpio_set_level(pin, 0));
    ESP_ERROR_CHECK(gpio_set_direction(pin, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_level(pin, 0));
}

static void configure_line_sensors(void)
{
    const gpio_config_t config = {
        .pin_bit_mask = (1ULL << LINE_LEFT_OUTER_GPIO) |
                        (1ULL << LINE_LEFT_INNER_GPIO) |
                        (1ULL << LINE_RIGHT_INNER_GPIO) |
                        (1ULL << LINE_RIGHT_OUTER_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&config));
}

static uint8_t read_line_sensors(void)
{
    return (gpio_get_level(LINE_LEFT_OUTER_GPIO) << 3) |
           (gpio_get_level(LINE_LEFT_INNER_GPIO) << 2) |
           (gpio_get_level(LINE_RIGHT_INNER_GPIO) << 1) |
           gpio_get_level(LINE_RIGHT_OUTER_GPIO);
}

static void log_line_sensors(uint8_t state)
{
    ESP_LOGI(TAG, "IR OUT1..OUT4=%d%d%d%d (left outer, left inner, right inner, right outer; black=0 white=1)",
             (state >> 3) & 1, (state >> 2) & 1,
             (state >> 1) & 1, state & 1);
}

static void all_motors_stop(void)
{
    for (size_t i = 0; i < sizeof(motors) / sizeof(motors[0]); ++i) {
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE,
                                      motors[i].pwm_channel, 0));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE,
                                         motors[i].pwm_channel));
    }

    ESP_ERROR_CHECK(gpio_set_level(DRIVER_STBY, 0));
    for (size_t i = 0; i < sizeof(motors) / sizeof(motors[0]); ++i) {
        ESP_ERROR_CHECK(gpio_set_level(motors[i].in1_pin, 0));
        ESP_ERROR_CHECK(gpio_set_level(motors[i].in2_pin, 0));
    }
}

static void motor_prepare(size_t index, int direction, uint32_t duty)
{
    const motor_t *motor = &motors[index];
    ESP_ERROR_CHECK(gpio_set_level(motor->in1_pin, direction > 0));
    ESP_ERROR_CHECK(gpio_set_level(motor->in2_pin, direction < 0));
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE,
                                  motor->pwm_channel, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE,
                                     motor->pwm_channel));
}

static uint32_t clamp_line_duty(int duty)
{
    if (duty < 0) {
        return 0;
    }
    if (duty > LINE_MAX_DUTY) {
        return LINE_MAX_DUTY;
    }
    return (uint32_t)duty;
}

static void force_stop(void)
{
    all_motors_stop();
    current_motion = MOTION_STOP;
}

/* Motor A is the right-side drive wheel; the C-channel motor is the left-side
 * wheel. The C connector has the opposite forward polarity from the old D
 * connector, so its forward direction is +1. */
static void drive_line_duties(uint32_t a_duty, uint32_t d_duty)
{
    motor_prepare(MOTOR_A, -1, a_duty);
    motor_prepare(MOTOR_B, 0, 0);
    motor_prepare(MOTOR_D, 1, d_duty);
    ESP_ERROR_CHECK(gpio_set_level(DRIVER_STBY, 1));
    current_motion = MOTION_FORWARD;
}

static void drive_sharp_turn(sharp_turn_t turn)
{
    if (turn == SHARP_TURN_LEFT) {
        /* Smooth left arc: outer wheel fast, inner wheel crawling forward. */
        motor_prepare(MOTOR_A, -1, LINE_TURN_DUTY);
        motor_prepare(MOTOR_D, 1, LINE_TURN_INNER_DUTY);
        current_motion = MOTION_LEFT;
    } else {
        /* Smooth right arc: outer wheel fast, inner wheel crawling forward. */
        motor_prepare(MOTOR_A, -1, LINE_TURN_INNER_DUTY);
        motor_prepare(MOTOR_D, 1,
                      clamp_line_duty(LINE_TURN_DUTY + LINE_D_TRIM_DUTY));
        current_motion = MOTION_RIGHT;
    }
    motor_prepare(MOTOR_B, 0, 0);
    ESP_ERROR_CHECK(gpio_set_level(DRIVER_STBY, 1));
}

static bool line_pulse_is_on(int64_t now_us)
{
    const int64_t elapsed_ms = (now_us - line_pulse_epoch_us) / 1000;
    return elapsed_ms % LINE_PULSE_CYCLE_MS < LINE_PULSE_ON_MS;
}

/* Keep each active pulse above the motors' starting-torque threshold, then
 * coast briefly. This is slower than using a continuously tiny PWM value and
 * avoids one motor stalling earlier than the other. */
static void drive_line_duties_pulsed(uint32_t a_duty, uint32_t d_duty,
                                     int64_t now_us)
{
    update_wheel_balance(a_duty, d_duty, now_us);
    const uint32_t adjusted_a =
        clamp_line_duty((int)a_duty - balance_trim);
    const uint32_t adjusted_c =
        clamp_line_duty((int)d_duty + balance_trim);
    if (line_pulse_is_on(now_us)) {
        drive_line_duties(adjusted_a, adjusted_c);
    } else {
        force_stop();
    }
}

static void drive_sharp_turn_pulsed(sharp_turn_t turn, int64_t now_us)
{
    reset_wheel_balance(now_us);
    if (line_pulse_is_on(now_us)) {
        drive_sharp_turn(turn);
    } else {
        force_stop();
    }
}

static void set_line_follow_enabled(bool enabled)
{
    force_stop();
    command_deadline_us = 0;
    line_follow_enabled = enabled;
    last_line_error = 0;
    line_lost_since_us = 0;
    last_line_control_state = 0xff;
    line_lost_stop_reported = false;
    line_has_been_seen = false;
    sharp_turn = SHARP_TURN_NONE;
    sharp_turn_since_us = 0;
    line_lost_search_active = false;
    line_lost_search_turn = SHARP_TURN_NONE;
    line_pulse_epoch_us = esp_timer_get_time();
    reset_wheel_balance(line_pulse_epoch_us);
    ESP_LOGI(TAG, "LINE FOLLOW %s", enabled ? "ENABLED" : "DISABLED / STOP");
}

static void update_line_follow(uint8_t sensor_state, int64_t now_us)
{
    /* Sensor bits are 1 on white and 0 on black. */
    static const int weights[] = {-3, -1, 1, 3};
    int black_count = 0;
    int weighted_sum = 0;
    for (int sensor = 0; sensor < 4; ++sensor) {
        const int bit = 3 - sensor;
        if ((sensor_state & (1U << bit)) == 0) {
            ++black_count;
            weighted_sum += weights[sensor];
        }
    }

    const bool left_outer_black = (sensor_state & 0x8) == 0;
    const bool left_inner_black = (sensor_state & 0x4) == 0;
    const bool right_inner_black = (sensor_state & 0x2) == 0;
    const bool right_outer_black = (sensor_state & 0x1) == 0;
    const bool inner_black = (sensor_state & 0x6) != 0x6;

    /* Lost-line recovery is deliberately sticky. A single black channel can
     * be edge noise, so keep turning toward the last known line side until at
     * least two channels agree that the line has been found again. */
    if (line_lost_search_active) {
        const int64_t search_us = now_us - line_lost_since_us;
        if (black_count >= 2) {
            ESP_LOGI(TAG, "LINE FOUND: %d black sensors; resume tracking",
                     black_count);
            line_lost_search_active = false;
            line_lost_search_turn = SHARP_TURN_NONE;
            line_lost_since_us = 0;
            line_lost_stop_reported = false;
        } else if (search_us >= (int64_t)LINE_LOST_TIMEOUT_MS * 1000) {
            force_stop();
            if (!line_lost_stop_reported) {
                ESP_LOGW(TAG, "LINE LOST: search timeout; STOP");
                line_lost_stop_reported = true;
            }
            line_lost_search_active = false;
            line_lost_search_turn = SHARP_TURN_NONE;
            line_has_been_seen = false;
            last_line_control_state = sensor_state;
            return;
        } else {
            drive_sharp_turn_pulsed(line_lost_search_turn, now_us);
            last_line_control_state = sensor_state;
            return;
        }
    }

    /* Once a sharp turn starts, keep pivoting through the temporary lost-line
     * interval until the new branch reaches an inner sensor. */
    if (sharp_turn != SHARP_TURN_NONE) {
        const int64_t turn_us = now_us - sharp_turn_since_us;
        const bool outer_released =
            sharp_turn == SHARP_TURN_LEFT ? !left_outer_black
                                          : !right_outer_black;
        if (turn_us >= (int64_t)LINE_TURN_MIN_MS * 1000 &&
            outer_released && inner_black && black_count >= 1 &&
            black_count < 4) {
            ESP_LOGI(TAG, "RIGHT-ANGLE %s: line reacquired",
                     sharp_turn == SHARP_TURN_LEFT ? "LEFT" : "RIGHT");
            sharp_turn = SHARP_TURN_NONE;
            sharp_turn_since_us = 0;
            line_lost_since_us = 0;
        } else if (turn_us >= (int64_t)LINE_TURN_TIMEOUT_MS * 1000) {
            force_stop();
            ESP_LOGW(TAG, "RIGHT-ANGLE timeout; STOP");
            sharp_turn = SHARP_TURN_NONE;
            sharp_turn_since_us = 0;
            line_has_been_seen = false;
            line_lost_stop_reported = true;
            last_line_control_state = sensor_state;
            return;
        } else {
            drive_sharp_turn_pulsed(sharp_turn, now_us);
            last_line_control_state = sensor_state;
            return;
        }
    }

    if (black_count == 4) {
        force_stop();
        line_lost_since_us = 0;
        if (sensor_state != last_line_control_state) {
            ESP_LOGW(TAG, "LINE: all sensors black / intersection; STOP");
        }
        last_line_control_state = sensor_state;
        return;
    }

    if (black_count > 0) {
        const int error = weighted_sum / black_count;

        /* Require two adjacent sensors on the same side before declaring a
         * right-angle corner. A single outer sensor is only a normal curve. */
        if (left_outer_black && left_inner_black &&
            !right_outer_black && error < 0) {
            sharp_turn = SHARP_TURN_LEFT;
        } else if (right_outer_black && right_inner_black &&
                   !left_outer_black && error > 0) {
            sharp_turn = SHARP_TURN_RIGHT;
        }
        if (sharp_turn != SHARP_TURN_NONE) {
            sharp_turn_since_us = now_us;
            last_line_error = sharp_turn == SHARP_TURN_LEFT ? -3 : 3;
            line_has_been_seen = true;
            line_lost_since_us = 0;
            ESP_LOGI(TAG, "RIGHT-ANGLE %s: gentle turn start",
                     sharp_turn == SHARP_TURN_LEFT ? "LEFT" : "RIGHT");
            drive_sharp_turn_pulsed(sharp_turn, now_us);
            last_line_control_state = sensor_state;
            return;
        }

        line_has_been_seen = true;
        line_lost_since_us = 0;
        line_lost_stop_reported = false;
        if (error != 0) {
            last_line_error = error;
        }

        const int correction = error * LINE_KP_DUTY;
        const uint32_t a_duty =
            clamp_line_duty(LINE_BASE_DUTY - correction);
        const uint32_t d_duty =
            clamp_line_duty(LINE_BASE_DUTY + correction + LINE_D_TRIM_DUTY);
        drive_line_duties_pulsed(a_duty, d_duty, now_us);

        if (sensor_state != last_line_control_state) {
            ESP_LOGI(TAG, "LINE TRACK state=%x error=%d PWM(A-right)=%lu PWM(D-left)=%lu",
                     sensor_state, error,
                     (unsigned long)a_duty, (unsigned long)d_duty);
        }
        last_line_control_state = sensor_state;
        return;
    }

    /* No sensor sees black. Never move blindly if F was pressed before the
     * car had acquired a line. */
    if (!line_has_been_seen) {
        force_stop();
        if (!line_lost_stop_reported) {
            ESP_LOGW(TAG, "NO LINE AT START; STOP");
            line_lost_stop_reported = true;
        }
        last_line_control_state = sensor_state;
        return;
    }

    if (last_line_error == 0) {
        force_stop();
        if (!line_lost_stop_reported) {
            ESP_LOGW(TAG, "LINE LOST: no previous side direction; STOP");
            line_lost_stop_reported = true;
        }
        last_line_control_state = sensor_state;
        return;
    }

    line_lost_since_us = now_us;
    line_lost_search_turn = last_line_error < 0
                                ? SHARP_TURN_LEFT
                                : SHARP_TURN_RIGHT;
    line_lost_search_active = true;
    line_lost_stop_reported = false;
    ESP_LOGW(TAG, "LINE LOST: slow search %s until >=2 black sensors",
             line_lost_search_turn == SHARP_TURN_LEFT ? "LEFT" : "RIGHT");
    drive_sharp_turn_pulsed(line_lost_search_turn, now_us);
    last_line_control_state = sensor_state;
}

static const char *motion_name(motion_t motion)
{
    switch (motion) {
    case MOTION_FORWARD: return "FORWARD";
    case MOTION_REVERSE: return "REVERSE";
    case MOTION_LEFT: return "LEFT";
    case MOTION_RIGHT: return "RIGHT";
    default: return "STOP";
    }
}

static void apply_motion(motion_t motion)
{
    if (motion == current_motion) {
        return;
    }

    all_motors_stop();
    if (motion == MOTION_STOP) {
        current_motion = motion;
        ESP_LOGI(TAG, "STOP");
        return;
    }

    int a_direction = 0;
    int d_direction = 0;
    switch (motion) {
    case MOTION_FORWARD:
        a_direction = -1;
        d_direction = 1;
        break;
    case MOTION_REVERSE:
        a_direction = 1;
        d_direction = -1;
        break;
    case MOTION_LEFT:
        a_direction = -1;
        break;
    case MOTION_RIGHT:
        d_direction = 1;
        break;
    default:
        return;
    }

    motor_prepare(MOTOR_A, a_direction, a_direction ? PWM_DUTY : 0);
    motor_prepare(MOTOR_B, 0, 0);
    motor_prepare(MOTOR_D, d_direction, d_direction ? PWM_DUTY : 0);
    ESP_ERROR_CHECK(gpio_set_level(DRIVER_STBY, 1));
    current_motion = motion;
    ESP_LOGI(TAG, "%s: A=%d B=0 D=%d", motion_name(motion),
             a_direction, d_direction);
}

static bool decode_key(char key, motion_t *motion)
{
    switch (tolower((unsigned char)key)) {
    case 'w': *motion = MOTION_FORWARD; return true;
    case 's': *motion = MOTION_REVERSE; return true;
    case 'a': *motion = MOTION_LEFT; return true;
    case 'd': *motion = MOTION_RIGHT; return true;
    case 'x':
    case ' ': *motion = MOTION_STOP; return true;
    default: return false;
    }
}

static void hardware_init(void)
{
    configure_output_low(DRIVER_STBY);
    for (size_t i = 0; i < sizeof(motors) / sizeof(motors[0]); ++i) {
        configure_output_low(motors[i].in1_pin);
        configure_output_low(motors[i].in2_pin);
    }

    const ledc_timer_config_t pwm_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = PWM_FREQUENCY_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&pwm_timer));

    for (size_t i = 0; i < sizeof(motors) / sizeof(motors[0]); ++i) {
        const ledc_channel_config_t pwm_channel = {
            .gpio_num = motors[i].pwm_pin,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = motors[i].pwm_channel,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = LEDC_TIMER_0,
            .duty = 0,
            .hpoint = 0,
            .flags.output_invert = 0,
        };
        ESP_ERROR_CHECK(ledc_channel_config(&pwm_channel));
    }
    all_motors_stop();
}

void app_main(void)
{
    hardware_init();
    configure_line_sensors();
    configure_encoders();

    /*
     * The startup console uses a polling USB Serial/JTAG VFS.  That is enough
     * for boot logs, but it does not continuously drain host-to-device data.
     * Install the interrupt-driven driver before accepting held movement keys.
     */
    usb_serial_jtag_driver_config_t usb_config =
        USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    usb_config.rx_buffer_size = 1024;
    usb_config.tx_buffer_size = 1024;
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_config));
    usb_serial_jtag_vfs_use_driver();

    const int stdin_flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    if (stdin_flags < 0 ||
        fcntl(STDIN_FILENO, F_SETFL, stdin_flags | O_NONBLOCK) < 0) {
        ESP_LOGE(TAG, "Failed to make USB console input non-blocking");
        while (1) {
            all_motors_stop();
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    ESP_LOGI(TAG, "USB KEYBOARD REMOTE READY");
    ESP_LOGI(TAG, "Hold W/S/A/D to move; X or SPACE to stop");
    ESP_LOGI(TAG, "Press F once to start line following; X or SPACE is emergency stop");
    ESP_LOGI(TAG, "Manual PWM=%d/1023; input timeout=%d ms; Motor B remains stopped",
             PWM_DUTY, COMMAND_TIMEOUT_MS);
    ESP_LOGI(TAG, "IR test pins: OUT1=GPIO13 OUT2=GPIO14 OUT3=GPIO21 OUT4=GPIO39");
    ESP_LOGI(TAG, "Encoder PI: A(E1)=GPIO16/17 C(E3)=GPIO2/1; trim limit=+/- %d",
             BALANCE_TRIM_MAX);
    ESP_LOGI(TAG, "Line follow PWM: base=%d D-trim=%d max=%d; pulse=%d/%d ms; lost=%d ms; right-angle=%d ms",
             LINE_BASE_DUTY, LINE_D_TRIM_DUTY, LINE_MAX_DUTY,
             LINE_PULSE_ON_MS, LINE_PULSE_CYCLE_MS,
             LINE_LOST_TIMEOUT_MS, LINE_TURN_TIMEOUT_MS);

    uint8_t line_candidate = 0xff;
    uint8_t line_reported = 0xff;
    unsigned line_stable_count = 0;
    int64_t line_next_report_us = 0;
    int64_t encoder_next_report_us = 0;
    int32_t encoder_previous_a = 0;
    int32_t encoder_previous_c = 0;

    while (1) {
        char input[16];
        const ssize_t count = read(STDIN_FILENO, input, sizeof(input));
        if (count > 0) {
            for (ssize_t i = 0; i < count; ++i) {
                const char key = tolower((unsigned char)input[i]);
                if (key == 'f') {
                    if (!line_follow_enabled) {
                        set_line_follow_enabled(true);
                    }
                    continue;
                }

                motion_t requested;
                if (decode_key(input[i], &requested)) {
                    if (line_follow_enabled) {
                        set_line_follow_enabled(false);
                    }
                    apply_motion(requested);
                    if (requested == MOTION_STOP) {
                        command_deadline_us = 0;
                    } else {
                        command_deadline_us = esp_timer_get_time() +
                            (int64_t)COMMAND_TIMEOUT_MS * 1000;
                    }
                }
            }
        } else if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            ESP_LOGE(TAG, "USB console read failed: errno=%d", errno);
            apply_motion(MOTION_STOP);
        }

        if (!line_follow_enabled && current_motion != MOTION_STOP &&
            esp_timer_get_time() >= command_deadline_us) {
            apply_motion(MOTION_STOP);
        }

        const uint8_t line_now = read_line_sensors();
        if (line_now != line_candidate) {
            line_candidate = line_now;
            line_stable_count = 1;
        } else if (line_stable_count < LINE_STABLE_SAMPLES) {
            ++line_stable_count;
        }
        if (line_stable_count >= LINE_STABLE_SAMPLES) {
            line_reported = line_candidate;
        }
        const int64_t now_us = esp_timer_get_time();
        if (line_reported != 0xff && now_us >= line_next_report_us) {
            log_line_sensors(line_reported);
            line_next_report_us = now_us +
                (int64_t)LINE_REPORT_INTERVAL_MS * 1000;
        }
        if (now_us >= encoder_next_report_us) {
            const int32_t encoder_a = encoders[0].count;
            const int32_t encoder_c = encoders[1].count;
            ESP_LOGI(TAG, "ENC count A=%" PRId32 " C=%" PRId32
                          " delta/%dms A=%" PRId32 " C=%" PRId32
                          " PI-trim=%d",
                     encoder_a, encoder_c, ENCODER_REPORT_MS,
                     encoder_a - encoder_previous_a,
                     encoder_c - encoder_previous_c, balance_trim);
            encoder_previous_a = encoder_a;
            encoder_previous_c = encoder_c;
            encoder_next_report_us = now_us +
                (int64_t)ENCODER_REPORT_MS * 1000;
        }
        if (line_follow_enabled && line_reported != 0xff) {
            update_line_follow(line_reported, now_us);
        }
        vTaskDelay(pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }
}
