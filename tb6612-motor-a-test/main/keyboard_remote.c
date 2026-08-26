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
#define LINE_RIGHT_OUTER_GPIO GPIO_NUM_47

#define ENCODER_A_PHASE_A_GPIO GPIO_NUM_16
#define ENCODER_A_PHASE_B_GPIO GPIO_NUM_17
#define ENCODER_LEFT_PHASE_A_GPIO GPIO_NUM_8
#define ENCODER_LEFT_PHASE_B_GPIO GPIO_NUM_18

#define PWM_FREQUENCY_HZ 20000
#define PWM_DUTY          260
#define COMMAND_TIMEOUT_MS 600
#define CONTROL_PERIOD_MS   10
#define LINE_STABLE_SAMPLES  2
#define LINE_REPORT_INTERVAL_MS 200
#define LINE_BASE_DUTY        130
#define LINE_MIN_DUTY         105
#define LINE_MAX_DUTY         175
#define LINE_ERROR_SCALE        8
#define LINE_PID_KP              6
#define LINE_PID_KI_DIV        400
#define LINE_PID_KD              6
#define LINE_PID_DEADBAND        3
#define LINE_PID_INTEGRAL_MAX 3200
#define LINE_PID_OUTPUT_MAX     30
#define LINE_D_TRIM_DUTY        0
#define LINE_PULSE_ON_MS        80
#define LINE_PULSE_CYCLE_MS     80
#define LINE_LOST_TIMEOUT_MS 2000
#define LINE_TURN_DUTY         170
#define LINE_TURN_INNER_DUTY   135
#define LINE_TURN_MIN_MS       100
#define LINE_TURN_TIMEOUT_MS  1200
#define ENCODER_REPORT_MS      500
#define SPEED_PI_UPDATE_MS     150
#define SPEED_TARGET_NUM         3
#define SPEED_TARGET_DEN         2
#define SPEED_PI_KP_DIV          4
#define SPEED_PI_KI_DIV        100
#define SPEED_PI_INTEGRAL_MAX 1500
#define SPEED_PI_OUTPUT_MAX     35

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
    MOTOR_C = 2,
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
    {"C-left-on-B", ENCODER_LEFT_PHASE_A_GPIO,
                    ENCODER_LEFT_PHASE_B_GPIO, 0, 0},
};
static int64_t speed_pi_last_update_us;
static int32_t speed_pi_previous_a;
static int32_t speed_pi_previous_b;
static int speed_pi_integral_a;
static int speed_pi_integral_b;
static int speed_pi_output_a;
static int speed_pi_output_b;
static bool line_pid_initialized;
static int line_pid_filtered_error;
static int line_pid_integral;
static int line_pid_p;
static int line_pid_i;
static int line_pid_d;

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
                        (1ULL << ENCODER_LEFT_PHASE_A_GPIO) |
                        (1ULL << ENCODER_LEFT_PHASE_B_GPIO),
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

static void reset_line_pid(void)
{
    line_pid_initialized = false;
    line_pid_filtered_error = 0;
    line_pid_integral = 0;
    line_pid_p = 0;
    line_pid_i = 0;
    line_pid_d = 0;
}

/* The four line sensors are digital, so their position estimate changes in
 * steps. A one-pole filter before the PID keeps those steps from becoming
 * abrupt wheel commands. The small I term removes persistent bias, while
 * sign-change reset and explicit limits prevent wind-up on corners. */
static int calculate_line_pid(int measured_error)
{
    int derivative = 0;
    if (!line_pid_initialized) {
        line_pid_filtered_error = measured_error;
        line_pid_initialized = true;
    } else {
        const int previous_error = line_pid_filtered_error;
        line_pid_filtered_error =
            (3 * line_pid_filtered_error + measured_error) / 4;
        derivative = line_pid_filtered_error - previous_error;
    }

    int control_error = line_pid_filtered_error;
    if (abs(control_error) <= LINE_PID_DEADBAND) {
        control_error = 0;
    }

    if (control_error == 0) {
        line_pid_integral = line_pid_integral * 7 / 8;
    } else {
        const bool changed_side =
            (line_pid_integral > 0 && control_error < 0) ||
            (line_pid_integral < 0 && control_error > 0);
        if (changed_side) {
            line_pid_integral = 0;
        }
        line_pid_integral = clamp_int(line_pid_integral + control_error,
                                      -LINE_PID_INTEGRAL_MAX,
                                      LINE_PID_INTEGRAL_MAX);
    }

    line_pid_p = LINE_PID_KP * control_error / LINE_ERROR_SCALE;
    line_pid_i = line_pid_integral / LINE_PID_KI_DIV;
    line_pid_d = LINE_PID_KD * derivative / LINE_ERROR_SCALE;
    return clamp_int(line_pid_p + line_pid_i + line_pid_d,
                     -LINE_PID_OUTPUT_MAX, LINE_PID_OUTPUT_MAX);
}

static void reset_wheel_speed_pi(int64_t now_us)
{
    speed_pi_last_update_us = now_us;
    speed_pi_previous_a = encoders[0].count;
    speed_pi_previous_b = encoders[1].count;
    speed_pi_integral_a = 0;
    speed_pi_integral_b = 0;
    speed_pi_output_a = 0;
    speed_pi_output_b = 0;
}

/* Independent wheel-speed PI controllers. Each nominal PWM request from the
 * outer line PID is converted to an encoder-count target. Unlike a controller
 * that only compares the two wheels, these controllers produce a strong
 * positive correction when both wheels stall at the same time. */
static void update_wheel_speed_pi(uint32_t a_request, uint32_t b_request,
                                  int64_t now_us)
{
    if (a_request == 0 || b_request == 0) {
        reset_wheel_speed_pi(now_us);
        return;
    }
    if (now_us - speed_pi_last_update_us <
        (int64_t)SPEED_PI_UPDATE_MS * 1000) {
        return;
    }

    const int32_t count_a = encoders[0].count;
    const int32_t count_b = encoders[1].count;
    const int32_t delta_a = abs(count_a - speed_pi_previous_a);
    const int32_t delta_b = abs(count_b - speed_pi_previous_b);
    speed_pi_previous_a = count_a;
    speed_pi_previous_b = count_b;
    speed_pi_last_update_us = now_us;

    const int target_a =
        (int)a_request * SPEED_TARGET_NUM / SPEED_TARGET_DEN;
    const int target_b =
        (int)b_request * SPEED_TARGET_NUM / SPEED_TARGET_DEN;
    const int error_a = target_a - (int)delta_a;
    const int error_b = target_b - (int)delta_b;

    speed_pi_integral_a =
        clamp_int(speed_pi_integral_a + error_a,
                  -SPEED_PI_INTEGRAL_MAX, SPEED_PI_INTEGRAL_MAX);
    speed_pi_integral_b =
        clamp_int(speed_pi_integral_b + error_b,
                  -SPEED_PI_INTEGRAL_MAX, SPEED_PI_INTEGRAL_MAX);
    speed_pi_output_a =
        clamp_int(error_a / SPEED_PI_KP_DIV +
                      speed_pi_integral_a / SPEED_PI_KI_DIV,
                  -SPEED_PI_OUTPUT_MAX, SPEED_PI_OUTPUT_MAX);
    speed_pi_output_b =
        clamp_int(error_b / SPEED_PI_KP_DIV +
                      speed_pi_integral_b / SPEED_PI_KI_DIV,
                  -SPEED_PI_OUTPUT_MAX, SPEED_PI_OUTPUT_MAX);
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
    if (duty <= 0) {
        return 0;
    }
    if (duty < LINE_MIN_DUTY) {
        return LINE_MIN_DUTY;
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

/* After swapping the B/C plugs, Motor A remains the right drive wheel and the
 * physical left wheel is driven by channel B. Channel C is intentionally
 * stopped. The B channel reverses this wheel's control polarity, so its
 * forward software direction is -1. */
static void drive_line_duties(uint32_t a_duty, uint32_t left_duty)
{
    motor_prepare(MOTOR_A, -1, a_duty);
    motor_prepare(MOTOR_B, -1, left_duty);
    motor_prepare(MOTOR_C, 0, 0);
    ESP_ERROR_CHECK(gpio_set_level(DRIVER_STBY, 1));
    current_motion = MOTION_FORWARD;
}

static void drive_sharp_turn(sharp_turn_t turn)
{
    if (turn == SHARP_TURN_LEFT) {
        /* Differential pivot: right wheel forward, left wheel reverse. */
        motor_prepare(MOTOR_A, -1, LINE_TURN_DUTY);
        motor_prepare(MOTOR_B, 1, LINE_TURN_INNER_DUTY);
        current_motion = MOTION_LEFT;
    } else {
        /* Differential pivot: right wheel reverse, left wheel forward. */
        motor_prepare(MOTOR_A, 1, LINE_TURN_INNER_DUTY);
        motor_prepare(MOTOR_B, -1,
                      clamp_line_duty(LINE_TURN_DUTY + LINE_D_TRIM_DUTY));
        current_motion = MOTION_RIGHT;
    }
    motor_prepare(MOTOR_C, 0, 0);
    ESP_ERROR_CHECK(gpio_set_level(DRIVER_STBY, 1));
}

static bool line_pulse_is_on(int64_t now_us)
{
    const int64_t elapsed_ms = (now_us - line_pulse_epoch_us) / 1000;
    return elapsed_ms % LINE_PULSE_CYCLE_MS < LINE_PULSE_ON_MS;
}

/* Keep this wrapper so pulse modulation can be re-enabled if needed. With
 * ON_MS equal to CYCLE_MS the motors receive continuous PWM, which avoids the
 * stop-start caster-wheel shake seen on the three-wheel chassis. */
static void drive_line_duties_pulsed(uint32_t a_duty, uint32_t left_duty,
                                     int64_t now_us)
{
    update_wheel_speed_pi(a_duty, left_duty, now_us);
    const uint32_t adjusted_a =
        clamp_line_duty((int)a_duty + speed_pi_output_a);
    const uint32_t adjusted_b =
        clamp_line_duty((int)left_duty + speed_pi_output_b);
    if (line_pulse_is_on(now_us)) {
        drive_line_duties(adjusted_a, adjusted_b);
    } else {
        force_stop();
    }
}

static void drive_sharp_turn_pulsed(sharp_turn_t turn, int64_t now_us)
{
    reset_wheel_speed_pi(now_us);
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
    reset_line_pid();
    reset_wheel_speed_pi(line_pulse_epoch_us);
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
            reset_line_pid();
        } else if (search_us >= (int64_t)LINE_LOST_TIMEOUT_MS * 1000) {
            force_stop();
            if (!line_lost_stop_reported) {
                ESP_LOGW(TAG, "LINE LOST: search timeout; STOP");
                line_lost_stop_reported = true;
            }
            line_lost_search_active = false;
            line_lost_search_turn = SHARP_TURN_NONE;
            line_has_been_seen = false;
            reset_line_pid();
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
            reset_line_pid();
        } else if (turn_us >= (int64_t)LINE_TURN_TIMEOUT_MS * 1000) {
            force_stop();
            ESP_LOGW(TAG, "RIGHT-ANGLE timeout; STOP");
            sharp_turn = SHARP_TURN_NONE;
            sharp_turn_since_us = 0;
            line_has_been_seen = false;
            line_lost_stop_reported = true;
            reset_line_pid();
            last_line_control_state = sensor_state;
            return;
        } else {
            drive_sharp_turn_pulsed(sharp_turn, now_us);
            last_line_control_state = sensor_state;
            return;
        }
    }

    /* Finish detection is intentionally disabled. Treat a transverse black
     * region as track and continue forward so corner handling remains in
     * charge of the following sensor transition. */
    if (black_count == 4) {
        drive_line_duties_pulsed(LINE_BASE_DUTY,
                                 LINE_BASE_DUTY, now_us);
        last_line_control_state = sensor_state;
        return;
    }

    if (black_count > 0) {
        const int error = weighted_sum * LINE_ERROR_SCALE / black_count;

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
            last_line_error = sharp_turn == SHARP_TURN_LEFT
                                  ? -3 * LINE_ERROR_SCALE
                                  : 3 * LINE_ERROR_SCALE;
            line_has_been_seen = true;
            line_lost_since_us = 0;
            reset_line_pid();
            ESP_LOGI(TAG, "RIGHT-ANGLE %s: differential pivot start",
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

        const int correction = calculate_line_pid(error);
        const uint32_t a_duty =
            clamp_line_duty(LINE_BASE_DUTY - correction);
        const uint32_t left_duty =
            clamp_line_duty(LINE_BASE_DUTY + correction + LINE_D_TRIM_DUTY);
        drive_line_duties_pulsed(a_duty, left_duty, now_us);

        if (sensor_state != last_line_control_state) {
            ESP_LOGI(TAG, "LINE PID state=%x error=%d/8 P=%d I=%d D=%d corr=%d PWM(A-right)=%lu PWM(C-left)=%lu",
                     sensor_state, error, line_pid_p, line_pid_i, line_pid_d,
                     correction,
                     (unsigned long)a_duty, (unsigned long)left_duty);
        }
        last_line_control_state = sensor_state;
        return;
    }

    /* No sensor sees black. Never move blindly if F was pressed before the
     * car had acquired a line. */
    if (!line_has_been_seen) {
        force_stop();
        reset_line_pid();
        if (!line_lost_stop_reported) {
            ESP_LOGW(TAG, "NO LINE AT START; STOP");
            line_lost_stop_reported = true;
        }
        last_line_control_state = sensor_state;
        return;
    }

    if (last_line_error == 0) {
        force_stop();
        reset_line_pid();
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
    reset_line_pid();
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
    int left_direction = 0;
    switch (motion) {
    case MOTION_FORWARD:
        a_direction = -1;
        left_direction = -1;
        break;
    case MOTION_REVERSE:
        a_direction = 1;
        left_direction = 1;
        break;
    case MOTION_LEFT:
        a_direction = -1;
        break;
    case MOTION_RIGHT:
        left_direction = -1;
        break;
    default:
        return;
    }

    motor_prepare(MOTOR_A, a_direction, a_direction ? PWM_DUTY : 0);
    motor_prepare(MOTOR_B, left_direction,
                  left_direction ? PWM_DUTY : 0);
    motor_prepare(MOTOR_C, 0, 0);
    ESP_ERROR_CHECK(gpio_set_level(DRIVER_STBY, 1));
    current_motion = motion;
    ESP_LOGI(TAG, "%s: A-right=%d B-left=%d C-channel=0",
             motion_name(motion), a_direction, left_direction);
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
    ESP_LOGI(TAG, "Manual PWM=%d/1023; input timeout=%d ms; channel C remains stopped",
             PWM_DUTY, COMMAND_TIMEOUT_MS);
    ESP_LOGI(TAG, "IR test pins: OUT1=GPIO13 OUT2=GPIO14 OUT3=GPIO21 OUT4=GPIO47");
    ESP_LOGI(TAG, "Dual speed PI: A(E1)=GPIO16/17 B-left(E2)=GPIO8/18; boost limit=+/- %d",
             SPEED_PI_OUTPUT_MAX);
    ESP_LOGI(TAG, "Line PID: Kp=%d KiDiv=%d Kd=%d output=+/-%d",
             LINE_PID_KP, LINE_PID_KI_DIV, LINE_PID_KD,
             LINE_PID_OUTPUT_MAX);
    ESP_LOGI(TAG, "Sharp turns: pivot outer=%d inner=%d; finish detection disabled",
             LINE_TURN_DUTY, LINE_TURN_INNER_DUTY);

    uint8_t line_candidate = 0xff;
    uint8_t line_reported = 0xff;
    unsigned line_stable_count = 0;
    int64_t line_next_report_us = 0;
    int64_t encoder_next_report_us = 0;
    int32_t encoder_previous_a = 0;
    int32_t encoder_previous_b = 0;

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
            const int32_t encoder_b = encoders[1].count;
            ESP_LOGI(TAG, "ENC count A=%" PRId32 " B=%" PRId32
                          " delta/%dms A=%" PRId32 " B=%" PRId32
                          " speed-PI A=%d B=%d",
                     encoder_a, encoder_b, ENCODER_REPORT_MS,
                     encoder_a - encoder_previous_a,
                     encoder_b - encoder_previous_b,
                     speed_pi_output_a, speed_pi_output_b);
            encoder_previous_a = encoder_a;
            encoder_previous_b = encoder_b;
            encoder_next_report_us = now_us +
                (int64_t)ENCODER_REPORT_MS * 1000;
        }
        if (line_follow_enabled && line_reported != 0xff) {
            update_line_follow(line_reported, now_us);
        }
        vTaskDelay(pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }
}
