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
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define DRIVER_STBY GPIO_NUM_5

#define LINE_LEFT_OUTER_GPIO  GPIO_NUM_13
#define LINE_LEFT_INNER_GPIO  GPIO_NUM_14
#define LINE_RIGHT_INNER_GPIO GPIO_NUM_21
#define LINE_RIGHT_OUTER_GPIO GPIO_NUM_47

#define ULTRASONIC_TRIG_GPIO GPIO_NUM_48
#define ULTRASONIC_ECHO_GPIO GPIO_NUM_39

#define ENCODER_A_PHASE_A_GPIO GPIO_NUM_16
#define ENCODER_A_PHASE_B_GPIO GPIO_NUM_17
#define ENCODER_B_PHASE_A_GPIO GPIO_NUM_8
#define ENCODER_B_PHASE_B_GPIO GPIO_NUM_18
#define ENCODER_D_PHASE_A_GPIO GPIO_NUM_2
#define ENCODER_D_PHASE_B_GPIO GPIO_NUM_1

#define PWM_FREQUENCY_HZ 20000
#define PWM_DUTY          130
#define COMMAND_TIMEOUT_MS 180
#define CONTROL_PERIOD_MS   10
#define LINE_STABLE_SAMPLES  2
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
#define SPEED_PI_UPDATE_MS     150
#define SPEED_TARGET_NUM         3
#define SPEED_TARGET_DEN         2
#define SPEED_PI_KP_DIV          4
#define SPEED_PI_KI_DIV        100
#define SPEED_PI_INTEGRAL_MAX 1500
#define SPEED_PI_OUTPUT_MAX     35

/* HC-SR04 obstacle trigger and one-shot bypass manoeuvre. ECHO must reach
 * GPIO39 through a 5 V -> 3.3 V divider. */
#define ULTRASONIC_SAMPLE_INTERVAL_MS       80
#define ULTRASONIC_ECHO_TIMEOUT_US       25000
#define ULTRASONIC_MOTOR_QUIET_US          2000
#define OBSTACLE_TRIGGER_MIN_MM              80
#define OBSTACLE_TRIGGER_MM                 150
#define OBSTACLE_CONFIRM_SAMPLES              3
#define OBSTACLE_STOP_BEFORE_ARC_MS          300
#define OBSTACLE_SETTLE_MS                   300
#define OBSTACLE_STRAFE_DIAGONAL_DUTY        90
#define OBSTACLE_STRAFE_REAR_DUTY           180
#define OBSTACLE_ARC_MIN_MS                  800
#define OBSTACLE_ARC_TIMEOUT_MS            12000
#define OBSTACLE_U_TURN_DUTY                 170
/* Initial 180-degree A/D pivot estimate. The rear B omni wheel is released
 * because its encoder showed a persistent stall during three-wheel rotation. */
#define OBSTACLE_U_TURN_MIN_COUNTS            550
#define OBSTACLE_U_TURN_MAX_COUNTS            950
#define OBSTACLE_U_TURN_TIMEOUT_MS            3500

/* 120-degree kiwi-wheel polarity calibrated on the assembled car. */
#define OBSTACLE_LEFT_A_DIRECTION            -1
#define OBSTACLE_LEFT_B_DIRECTION             1
#define OBSTACLE_LEFT_D_DIRECTION             1

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
    MOTION_STRAFE_LEFT,
    MOTION_STRAFE_RIGHT,
} motion_t;

typedef enum {
    SHARP_TURN_NONE,
    SHARP_TURN_LEFT,
    SHARP_TURN_RIGHT,
} sharp_turn_t;

typedef enum {
    OBSTACLE_IDLE,
    OBSTACLE_PAUSE_BEFORE_ARC,
    OBSTACLE_ARC_FIND_LINE,
    OBSTACLE_PAUSE_BEFORE_U_TURN,
    OBSTACLE_U_TURN_FIND_LINE,
} obstacle_state_t;

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
static int8_t motor_command_direction[3];
static uint32_t motor_command_duty[3];
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
    {"A-right-front", ENCODER_A_PHASE_A_GPIO, ENCODER_A_PHASE_B_GPIO, 0, 0},
    {"B-rear", ENCODER_B_PHASE_A_GPIO,
               ENCODER_B_PHASE_B_GPIO, 0, 0},
    {"D-left-front", ENCODER_D_PHASE_A_GPIO, ENCODER_D_PHASE_B_GPIO, 0, 0},
};
static int64_t speed_pi_last_update_us;
static int32_t speed_pi_previous_a;
static int32_t speed_pi_previous_d;
static int speed_pi_integral_a;
static int speed_pi_integral_d;
static int speed_pi_output_a;
static int speed_pi_output_d;
static bool line_pid_initialized;
static int line_pid_filtered_error;
static int line_pid_integral;
static int line_pid_p;
static int line_pid_i;
static int line_pid_d;
static obstacle_state_t obstacle_state = OBSTACLE_IDLE;
static bool obstacle_avoidance_done;
static bool obstacle_search_saw_white;
static int obstacle_near_samples;
static int64_t ultrasonic_next_sample_us;
static int latest_ultrasonic_distance_mm = -1;
static int64_t obstacle_state_started_us;
static int32_t obstacle_encoder_start_a;
static int32_t obstacle_encoder_start_d;
static uint8_t finish_previous_sensor_state = 0xff;

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
                        (1ULL << ENCODER_B_PHASE_A_GPIO) |
                        (1ULL << ENCODER_B_PHASE_B_GPIO) |
                        (1ULL << ENCODER_D_PHASE_A_GPIO) |
                        (1ULL << ENCODER_D_PHASE_B_GPIO),
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
    speed_pi_previous_a = encoders[MOTOR_A].count;
    speed_pi_previous_d = encoders[MOTOR_D].count;
    speed_pi_integral_a = 0;
    speed_pi_integral_d = 0;
    speed_pi_output_a = 0;
    speed_pi_output_d = 0;
}

/* Independent wheel-speed PI controllers. Each nominal PWM request from the
 * outer line PID is converted to an encoder-count target. Unlike a controller
 * that only compares the two wheels, these controllers produce a strong
 * positive correction when both wheels stall at the same time. */
static void update_wheel_speed_pi(uint32_t a_request, uint32_t d_request,
                                  int64_t now_us)
{
    if (a_request == 0 || d_request == 0) {
        reset_wheel_speed_pi(now_us);
        return;
    }
    if (now_us - speed_pi_last_update_us <
        (int64_t)SPEED_PI_UPDATE_MS * 1000) {
        return;
    }

    const int32_t count_a = encoders[MOTOR_A].count;
    const int32_t count_d = encoders[MOTOR_D].count;
    const int32_t delta_a = abs(count_a - speed_pi_previous_a);
    const int32_t delta_d = abs(count_d - speed_pi_previous_d);
    speed_pi_previous_a = count_a;
    speed_pi_previous_d = count_d;
    speed_pi_last_update_us = now_us;

    const int target_a =
        (int)a_request * SPEED_TARGET_NUM / SPEED_TARGET_DEN;
    const int target_d =
        (int)d_request * SPEED_TARGET_NUM / SPEED_TARGET_DEN;
    const int error_a = target_a - (int)delta_a;
    const int error_d = target_d - (int)delta_d;

    speed_pi_integral_a =
        clamp_int(speed_pi_integral_a + error_a,
                  -SPEED_PI_INTEGRAL_MAX, SPEED_PI_INTEGRAL_MAX);
    speed_pi_integral_d =
        clamp_int(speed_pi_integral_d + error_d,
                  -SPEED_PI_INTEGRAL_MAX, SPEED_PI_INTEGRAL_MAX);
    speed_pi_output_a =
        clamp_int(error_a / SPEED_PI_KP_DIV +
                      speed_pi_integral_a / SPEED_PI_KI_DIV,
                  -SPEED_PI_OUTPUT_MAX, SPEED_PI_OUTPUT_MAX);
    speed_pi_output_d =
        clamp_int(error_d / SPEED_PI_KP_DIV +
                      speed_pi_integral_d / SPEED_PI_KI_DIV,
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

static void configure_ultrasonic(void)
{
    configure_output_low(ULTRASONIC_TRIG_GPIO);
    const gpio_config_t echo_config = {
        .pin_bit_mask = 1ULL << ULTRASONIC_ECHO_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&echo_config));
}

/* Return millimetres, or -1 if the echo never completes. The bounded polling
 * interval is at most 25 ms and only runs every 80 ms while normal line
 * following is active. */
static int read_ultrasonic_distance_mm(void)
{
    int64_t deadline_us =
        esp_timer_get_time() + ULTRASONIC_ECHO_TIMEOUT_US;
    while (gpio_get_level(ULTRASONIC_ECHO_GPIO)) {
        if (esp_timer_get_time() >= deadline_us) {
            return -1;
        }
    }

    ESP_ERROR_CHECK(gpio_set_level(ULTRASONIC_TRIG_GPIO, 0));
    esp_rom_delay_us(2);
    ESP_ERROR_CHECK(gpio_set_level(ULTRASONIC_TRIG_GPIO, 1));
    esp_rom_delay_us(10);
    ESP_ERROR_CHECK(gpio_set_level(ULTRASONIC_TRIG_GPIO, 0));

    deadline_us = esp_timer_get_time() + ULTRASONIC_ECHO_TIMEOUT_US;
    while (!gpio_get_level(ULTRASONIC_ECHO_GPIO)) {
        if (esp_timer_get_time() >= deadline_us) {
            return -1;
        }
    }
    const int64_t echo_start_us = esp_timer_get_time();

    deadline_us = echo_start_us + ULTRASONIC_ECHO_TIMEOUT_US;
    while (gpio_get_level(ULTRASONIC_ECHO_GPIO)) {
        if (esp_timer_get_time() >= deadline_us) {
            return -1;
        }
    }

    const int64_t echo_width_us = esp_timer_get_time() - echo_start_us;
    return (int)((echo_width_us * 10 + 29) / 58);
}

/* Sample continuously in every operating mode. HC-SR04 modules need a quiet
 * interval between trigger pulses, so 80 ms is the real-time telemetry rate. */
static bool update_ultrasonic(int64_t now_us)
{
    if (now_us < ultrasonic_next_sample_us) {
        return false;
    }

    ultrasonic_next_sample_us = now_us +
        (int64_t)ULTRASONIC_SAMPLE_INTERVAL_MS * 1000;

    /* Motor PWM noise produced persistent false echoes around 20--28 mm in
     * captured runs. Briefly disable the bridges without changing their PWM
     * or direction registers, then restore STBY immediately after ranging. */
    const bool driver_was_enabled = gpio_get_level(DRIVER_STBY) != 0;
    if (driver_was_enabled) {
        ESP_ERROR_CHECK(gpio_set_level(DRIVER_STBY, 0));
        esp_rom_delay_us(ULTRASONIC_MOTOR_QUIET_US);
    }
    latest_ultrasonic_distance_mm = read_ultrasonic_distance_mm();
    if (driver_was_enabled) {
        ESP_ERROR_CHECK(gpio_set_level(DRIVER_STBY, 1));
    }
    return true;
}

static uint8_t read_line_sensors(void)
{
    return (gpio_get_level(LINE_LEFT_OUTER_GPIO) << 3) |
           (gpio_get_level(LINE_LEFT_INNER_GPIO) << 2) |
           (gpio_get_level(LINE_RIGHT_INNER_GPIO) << 1) |
           gpio_get_level(LINE_RIGHT_OUTER_GPIO);
}

static int count_black_sensors(uint8_t state)
{
    int count = 0;
    for (int bit = 0; bit < 4; ++bit) {
        if ((state & (1U << bit)) == 0) {
            ++count;
        }
    }
    return count;
}

static void all_motors_stop(void)
{
    for (size_t i = 0; i < sizeof(motors) / sizeof(motors[0]); ++i) {
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE,
                                      motors[i].pwm_channel, 0));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE,
                                         motors[i].pwm_channel));
        motor_command_direction[i] = 0;
        motor_command_duty[i] = 0;
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
    motor_command_direction[index] = direction > 0 ? 1 :
                                     direction < 0 ? -1 : 0;
    motor_command_duty[index] = direction == 0 ? 0 : duty;
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

/* Physical layout: D is left-front, A is right-front and B is rear. Straight
 * line following therefore uses A/D with equal software polarity; B stops. */
static void drive_line_duties(uint32_t a_duty, uint32_t d_duty)
{
    motor_prepare(MOTOR_A, -1, a_duty);
    motor_prepare(MOTOR_B, 0, 0);
    motor_prepare(MOTOR_D, -1, d_duty);
    ESP_ERROR_CHECK(gpio_set_level(DRIVER_STBY, 1));
    current_motion = MOTION_FORWARD;
}

static void drive_sharp_turn(sharp_turn_t turn)
{
    if (turn == SHARP_TURN_LEFT) {
        /* Keep the inner wheel reversing so the chassis can negotiate a tight
         * corner, but run it much slower than the forward outer wheel. The
         * unequal speeds move the rotation centre away from the chassis
         * centre and avoid a sustained zero-radius spin. */
        motor_prepare(MOTOR_A, -1, LINE_TURN_DUTY);
        motor_prepare(MOTOR_D, 1, LINE_TURN_INNER_DUTY);
        current_motion = MOTION_LEFT;
    } else {
        /* Mirror the left turn: right inner wheel reverses slowly while the
         * left outer wheel moves forward. */
        motor_prepare(MOTOR_A, 1, LINE_TURN_INNER_DUTY);
        motor_prepare(MOTOR_D, -1,
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

/* Keep this wrapper so pulse modulation can be re-enabled if needed. With
 * ON_MS equal to CYCLE_MS the motors receive continuous PWM, which avoids the
 * stop-start caster-wheel shake seen on the three-wheel chassis. */
static void drive_line_duties_pulsed(uint32_t a_duty, uint32_t d_duty,
                                     int64_t now_us)
{
    update_wheel_speed_pi(a_duty, d_duty, now_us);
    const uint32_t adjusted_a =
        clamp_line_duty((int)a_duty + speed_pi_output_a);
    const uint32_t adjusted_d =
        clamp_line_duty((int)d_duty + speed_pi_output_d);
    if (line_pulse_is_on(now_us)) {
        drive_line_duties(adjusted_a, adjusted_d);
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

static void snapshot_obstacle_encoders(void)
{
    obstacle_encoder_start_a = encoders[MOTOR_A].count;
    obstacle_encoder_start_d = encoders[MOTOR_D].count;
}

static int32_t obstacle_encoder_progress(void)
{
    int32_t delta_a = encoders[MOTOR_A].count - obstacle_encoder_start_a;
    int32_t delta_d = encoders[MOTOR_D].count - obstacle_encoder_start_d;
    if (delta_a < 0) {
        delta_a = -delta_a;
    }
    if (delta_d < 0) {
        delta_d = -delta_d;
    }
    /* Both front wheels must reach the target; B is deliberately released. */
    return delta_a < delta_d ? delta_a : delta_d;
}

static void drive_kiwi_strafe(bool left)
{
    const int direction = left ? 1 : -1;
    motor_prepare(MOTOR_A,
                  direction * OBSTACLE_LEFT_A_DIRECTION,
                  OBSTACLE_STRAFE_DIAGONAL_DUTY);
    motor_prepare(MOTOR_B,
                  direction * OBSTACLE_LEFT_B_DIRECTION,
                  OBSTACLE_STRAFE_REAR_DUTY);
    motor_prepare(MOTOR_D,
                  direction * OBSTACLE_LEFT_D_DIRECTION,
                  OBSTACLE_STRAFE_DIAGONAL_DUTY);
    ESP_ERROR_CHECK(gpio_set_level(DRIVER_STBY, 1));
    current_motion = left ? MOTION_STRAFE_LEFT : MOTION_STRAFE_RIGHT;
}

static void drive_obstacle_u_turn(void)
{
    motor_prepare(MOTOR_A, -1, OBSTACLE_U_TURN_DUTY);
    motor_prepare(MOTOR_B, 0, 0);
    motor_prepare(MOTOR_D, 1, OBSTACLE_U_TURN_DUTY);
    ESP_ERROR_CHECK(gpio_set_level(DRIVER_STBY, 1));
    current_motion = MOTION_LEFT;
}

static void cancel_obstacle_avoidance(void)
{
    if (obstacle_state != OBSTACLE_IDLE) {
        ESP_LOGW(TAG, "OBSTACLE bypass cancelled");
    }
    obstacle_state = OBSTACLE_IDLE;
    obstacle_near_samples = 0;
    obstacle_search_saw_white = false;
    finish_previous_sensor_state = 0xff;
}

static void fail_obstacle_avoidance(const char *reason)
{
    force_stop();
    obstacle_state = OBSTACLE_IDLE;
    obstacle_near_samples = 0;
    obstacle_search_saw_white = false;
    finish_previous_sensor_state = 0xff;
    line_follow_enabled = false;
    ESP_LOGE(TAG, "OBSTACLE bypass failed: %s; LINE FOLLOW DISABLED / STOP",
             reason);
}

static void start_obstacle_avoidance(int distance_mm, int64_t now_us)
{
    force_stop();
    obstacle_state = OBSTACLE_PAUSE_BEFORE_ARC;
    obstacle_state_started_us = now_us;
    obstacle_near_samples = 0;
    obstacle_search_saw_white = false;
    reset_line_pid();
    reset_wheel_speed_pi(now_us);
    ESP_LOGW(TAG,
             "OBSTACLE %d mm: STOP %d ms, arc left until line, then U-turn",
             distance_mm, OBSTACLE_STOP_BEFORE_ARC_MS);
}

static void finish_obstacle_avoidance(int black_count, int64_t now_us)
{
    force_stop();
    obstacle_state = OBSTACLE_IDLE;
    obstacle_avoidance_done = true;
    obstacle_search_saw_white = false;
    obstacle_near_samples = 0;
    finish_previous_sensor_state = 0xff;
    line_has_been_seen = true;
    last_line_error = 0;
    line_lost_since_us = 0;
    line_lost_stop_reported = false;
    line_lost_search_active = false;
    line_lost_search_turn = SHARP_TURN_NONE;
    sharp_turn = SHARP_TURN_NONE;
    sharp_turn_since_us = 0;
    last_line_control_state = 0xff;
    line_pulse_epoch_us = now_us;
    reset_line_pid();
    reset_wheel_speed_pi(now_us);
    ESP_LOGW(TAG,
             "OBSTACLE U-turn complete (%d black); resume line follow",
             black_count);
}

/* Return true while the obstacle state machine owns the motors. */
static bool update_obstacle_avoidance(uint8_t sensor_state, int64_t now_us,
                                      bool ultrasonic_sample_ready)
{
    if (obstacle_state == OBSTACLE_IDLE) {
        if (obstacle_avoidance_done || !ultrasonic_sample_ready) {
            return false;
        }

        const int distance_mm = latest_ultrasonic_distance_mm;

        if (distance_mm >= OBSTACLE_TRIGGER_MIN_MM &&
            distance_mm <= OBSTACLE_TRIGGER_MM) {
            ++obstacle_near_samples;
        } else {
            obstacle_near_samples = 0;
        }

        if (obstacle_near_samples >= OBSTACLE_CONFIRM_SAMPLES) {
            start_obstacle_avoidance(distance_mm, now_us);
            return true;
        }
        return false;
    }

    const int64_t elapsed_ms =
        (now_us - obstacle_state_started_us) / 1000;
    const int32_t progress = obstacle_encoder_progress();
    const int black_count = count_black_sensors(sensor_state);

    switch (obstacle_state) {
    case OBSTACLE_PAUSE_BEFORE_ARC:
        if (elapsed_ms >= OBSTACLE_STOP_BEFORE_ARC_MS) {
            obstacle_state = OBSTACLE_ARC_FIND_LINE;
            obstacle_state_started_us = now_us;
            obstacle_search_saw_white = black_count == 0;
            snapshot_obstacle_encoders();
            ESP_LOGI(TAG,
                     "OBSTACLE start hard-coded left arc; wait for next line");
            drive_kiwi_strafe(true);
        }
        return true;

    case OBSTACLE_ARC_FIND_LINE:
        if (black_count == 0) {
            obstacle_search_saw_white = true;
        }
        if (elapsed_ms >= OBSTACLE_ARC_MIN_MS &&
            obstacle_search_saw_white && black_count >= 2) {
            force_stop();
            obstacle_state = OBSTACLE_PAUSE_BEFORE_U_TURN;
            obstacle_state_started_us = now_us;
            ESP_LOGW(TAG,
                     "OBSTACLE arc reached line (%d black); stop before U-turn",
                     black_count);
        } else if (elapsed_ms >= OBSTACLE_ARC_TIMEOUT_MS) {
            fail_obstacle_avoidance("arc line-search timeout");
        } else {
            drive_kiwi_strafe(true);
        }
        return true;

    case OBSTACLE_PAUSE_BEFORE_U_TURN:
        if (elapsed_ms >= OBSTACLE_SETTLE_MS) {
            obstacle_state = OBSTACLE_U_TURN_FIND_LINE;
            obstacle_state_started_us = now_us;
            obstacle_search_saw_white = false;
            snapshot_obstacle_encoders();
            ESP_LOGI(TAG,
                     "OBSTACLE start A/D pivot U-turn (B released); target >=%d counts",
                     OBSTACLE_U_TURN_MIN_COUNTS);
            drive_obstacle_u_turn();
        }
        return true;

    case OBSTACLE_U_TURN_FIND_LINE:
        if (black_count == 0) {
            obstacle_search_saw_white = true;
        }
        if (progress >= OBSTACLE_U_TURN_MIN_COUNTS &&
            obstacle_search_saw_white && black_count >= 2) {
            finish_obstacle_avoidance(black_count, now_us);
        } else if (progress >= OBSTACLE_U_TURN_MAX_COUNTS) {
            fail_obstacle_avoidance("U-turn passed encoder limit without line");
        } else if (elapsed_ms >= OBSTACLE_U_TURN_TIMEOUT_MS) {
            fail_obstacle_avoidance("U-turn timeout");
        } else {
            drive_obstacle_u_turn();
        }
        return true;

    default:
        fail_obstacle_avoidance("invalid state");
        return true;
    }
}

/* The finish marker is valid only after the one-shot obstacle manoeuvre. The
 * sensor convention is black=0 and white=1, so 0000 -> 1111 means leaving a
 * full-width black finish bar onto the white floor. */
static bool update_finish_detection(uint8_t sensor_state)
{
    if (!obstacle_avoidance_done) {
        return false;
    }

    const uint8_t previous = finish_previous_sensor_state;
    finish_previous_sensor_state = sensor_state;
    if (previous != 0x0 || sensor_state != 0xf) {
        return false;
    }

    force_stop();
    line_follow_enabled = false;
    command_deadline_us = 0;
    ESP_LOGW(TAG,
             "FINISH detected: stable IR transition 0000 -> 1111; STOP LATCHED");
    return true;
}

static void set_line_follow_enabled(bool enabled)
{
    force_stop();
    cancel_obstacle_avoidance();
    command_deadline_us = 0;
    line_follow_enabled = enabled;
    obstacle_avoidance_done = false;
    finish_previous_sensor_state = 0xff;
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

    /* Once a sharp turn starts, keep the unequal-speed differential turn
     * through the temporary lost-line interval until the new branch reaches
     * an inner sensor. */
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
     * region as track and continue forward. */
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
            ESP_LOGI(TAG, "RIGHT-ANGLE %s: biased differential turn start",
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
        const uint32_t d_duty =
            clamp_line_duty(LINE_BASE_DUTY + correction + LINE_D_TRIM_DUTY);
        drive_line_duties_pulsed(a_duty, d_duty, now_us);

        if (sensor_state != last_line_control_state) {
            ESP_LOGI(TAG, "LINE PID state=%x error=%d/8 P=%d I=%d D=%d corr=%d PWM(A-right-front)=%lu PWM(D-left-front)=%lu",
                     sensor_state, error, line_pid_p, line_pid_i, line_pid_d,
                     correction,
                     (unsigned long)a_duty, (unsigned long)d_duty);
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
    case MOTION_STRAFE_LEFT: return "STRAFE_LEFT";
    case MOTION_STRAFE_RIGHT: return "STRAFE_RIGHT";
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

    if (motion == MOTION_STRAFE_LEFT || motion == MOTION_STRAFE_RIGHT) {
        drive_kiwi_strafe(motion == MOTION_STRAFE_LEFT);
        ESP_LOGI(TAG, "%s: slow three-wheel translation",
                 motion_name(motion));
        return;
    }

    int a_direction = 0;
    int d_direction = 0;
    switch (motion) {
    case MOTION_FORWARD:
        a_direction = -1;
        d_direction = -1;
        break;
    case MOTION_REVERSE:
        a_direction = 1;
        d_direction = 1;
        break;
    case MOTION_LEFT:
        a_direction = -1;
        break;
    case MOTION_RIGHT:
        d_direction = -1;
        break;
    default:
        return;
    }

    motor_prepare(MOTOR_A, a_direction, a_direction ? PWM_DUTY : 0);
    motor_prepare(MOTOR_B, 0, 0);
    motor_prepare(MOTOR_D, d_direction,
                  d_direction ? PWM_DUTY : 0);
    ESP_ERROR_CHECK(gpio_set_level(DRIVER_STBY, 1));
    current_motion = motion;
    ESP_LOGI(TAG, "%s: A-right-front=%d D-left-front=%d B-rear=0",
             motion_name(motion), a_direction, d_direction);
}

static bool decode_key(char key, motion_t *motion)
{
    switch (tolower((unsigned char)key)) {
    case 'w': *motion = MOTION_FORWARD; return true;
    case 's': *motion = MOTION_REVERSE; return true;
    case 'a': *motion = MOTION_LEFT; return true;
    case 'd': *motion = MOTION_RIGHT; return true;
    case 'q': *motion = MOTION_STRAFE_LEFT; return true;
    case 'e': *motion = MOTION_STRAFE_RIGHT; return true;
    case 'x':
    case ' ': *motion = MOTION_STOP; return true;
    default: return false;
    }
}

static const char *obstacle_phase_name(void)
{
    switch (obstacle_state) {
    case OBSTACLE_PAUSE_BEFORE_ARC: return "STOP_BEFORE_ARC";
    case OBSTACLE_ARC_FIND_LINE: return "ARC_FIND_LINE";
    case OBSTACLE_PAUSE_BEFORE_U_TURN: return "STOP_BEFORE_U_TURN";
    case OBSTACLE_U_TURN_FIND_LINE: return "U_TURN_FIND_LINE";
    default: return "IDLE";
    }
}

static const char *control_mode_name(void)
{
    if (obstacle_state != OBSTACLE_IDLE) {
        return "OBSTACLE";
    }
    return line_follow_enabled ? "LINE" : "MANUAL";
}

/* One parse-friendly record per ultrasonic sample. Signed encoder deltas and
 * counts/s preserve all three wheel directions. */
static void log_realtime_telemetry(uint8_t sensor_state, int64_t now_us)
{
    static int64_t previous_us;
    static int32_t previous_a;
    static int32_t previous_b;
    static int32_t previous_d;

    const int32_t encoder_a = encoders[MOTOR_A].count;
    const int32_t encoder_b = encoders[MOTOR_B].count;
    const int32_t encoder_d = encoders[MOTOR_D].count;
    int32_t delta_a = 0;
    int32_t delta_b = 0;
    int32_t delta_d = 0;
    int32_t counts_per_second_a = 0;
    int32_t counts_per_second_b = 0;
    int32_t counts_per_second_d = 0;

    if (previous_us != 0) {
        const int64_t elapsed_us = now_us - previous_us;
        delta_a = encoder_a - previous_a;
        delta_b = encoder_b - previous_b;
        delta_d = encoder_d - previous_d;
        if (elapsed_us > 0) {
            counts_per_second_a =
                (int32_t)((int64_t)delta_a * 1000000 / elapsed_us);
            counts_per_second_b =
                (int32_t)((int64_t)delta_b * 1000000 / elapsed_us);
            counts_per_second_d =
                (int32_t)((int64_t)delta_d * 1000000 / elapsed_us);
        }
    }

    ESP_LOGI(TAG,
             "TELEM,%" PRId64 ",%s,%s,%s,%d,%d%d%d%d,%" PRId32
             ",%" PRId32 ",%" PRId32 ",%" PRId32 ",%" PRId32
             ",%" PRId32 ",%" PRId32 ",%" PRId32 ",%" PRId32
             ",%d,%lu,%d,%lu,%d,%lu,%d,%d",
             now_us / 1000,
             control_mode_name(), motion_name(current_motion),
             obstacle_phase_name(), latest_ultrasonic_distance_mm,
             (sensor_state >> 3) & 1, (sensor_state >> 2) & 1,
             (sensor_state >> 1) & 1, sensor_state & 1,
             encoder_a, encoder_b, encoder_d,
             delta_a, delta_b, delta_d,
             counts_per_second_a, counts_per_second_b, counts_per_second_d,
             motor_command_direction[MOTOR_A],
             (unsigned long)motor_command_duty[MOTOR_A],
             motor_command_direction[MOTOR_B],
             (unsigned long)motor_command_duty[MOTOR_B],
             motor_command_direction[MOTOR_D],
             (unsigned long)motor_command_duty[MOTOR_D],
             speed_pi_output_a, speed_pi_output_d);

    previous_us = now_us;
    previous_a = encoder_a;
    previous_b = encoder_b;
    previous_d = encoder_d;
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
    configure_ultrasonic();
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
    ESP_LOGI(TAG, "Hold W/S to drive, A/D to turn, Q/E to strafe; X or SPACE to stop");
    ESP_LOGI(TAG, "Press F once to start line following; X or SPACE is emergency stop");
    ESP_LOGI(TAG, "Manual PWM=%d/1023; input timeout=%d ms; layout D=left-front A=right-front B=rear",
             PWM_DUTY, COMMAND_TIMEOUT_MS);
    ESP_LOGI(TAG, "IR test pins: OUT1=GPIO13 OUT2=GPIO14 OUT3=GPIO21 OUT4=GPIO47");
    ESP_LOGI(TAG, "Hall encoders: A-right(E1)=GPIO16/17 B-rear(E2)=GPIO8/18 D-left(E4)=GPIO2/1; A/D speed-PI limit=+/- %d",
             SPEED_PI_OUTPUT_MAX);
    ESP_LOGI(TAG, "Line PID: Kp=%d Ki=1/%d Kd=%d deadband=%d/%d output=+/-%d",
             LINE_PID_KP, LINE_PID_KI_DIV, LINE_PID_KD,
             LINE_PID_DEADBAND, LINE_ERROR_SCALE, LINE_PID_OUTPUT_MAX);
    ESP_LOGI(TAG, "Line follow PWM: base=%d min=%d max=%d D-trim=%d; turn outer/inner=%d/%d; pulse=%d/%d ms; lost=%d ms; right-angle=%d ms",
             LINE_BASE_DUTY, LINE_MIN_DUTY, LINE_MAX_DUTY,
             LINE_D_TRIM_DUTY, LINE_TURN_DUTY, LINE_TURN_INNER_DUTY,
             LINE_PULSE_ON_MS, LINE_PULSE_CYCLE_MS,
             LINE_LOST_TIMEOUT_MS, LINE_TURN_TIMEOUT_MS);
    ESP_LOGI(TAG,
             "HC-SR04 TRIG=GPIO48 ECHO=GPIO39(divided); trigger=%d..%d mm x%d",
             OBSTACLE_TRIGGER_MIN_MM, OBSTACLE_TRIGGER_MM,
             OBSTACLE_CONFIRM_SAMPLES);
    ESP_LOGI(TAG,
             "Hard-coded route: left arc to line, U-turn >=%d counts, resume; finish=0000->1111",
             OBSTACLE_U_TURN_MIN_COUNTS);
    ESP_LOGI(TAG, "Ultrasonic + Hall telemetry runs continuously every %d ms",
             ULTRASONIC_SAMPLE_INTERVAL_MS);
    ESP_LOGI(TAG, "TELEM fields 1-9: t_ms mode motion phase sonar_mm ir encA encB encD");
    ESP_LOGI(TAG, "TELEM fields 10-16: dA dB dD cpsA cpsB cpsD dirA");
    ESP_LOGI(TAG, "TELEM fields 17-23: pwmA dirB pwmB dirD pwmD speedPiA speedPiD");

    uint8_t line_candidate = 0xff;
    uint8_t line_reported = 0xff;
    unsigned line_stable_count = 0;
    ultrasonic_next_sample_us = esp_timer_get_time();

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
            if (line_follow_enabled) {
                set_line_follow_enabled(false);
            }
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
        const bool ultrasonic_sample_ready = update_ultrasonic(now_us);
        bool finish_stopped = false;
        bool obstacle_owns_motors = false;
        if (line_follow_enabled && line_reported != 0xff) {
            finish_stopped = update_finish_detection(line_reported);
        }
        if (line_follow_enabled && !finish_stopped &&
            line_reported != 0xff) {
            obstacle_owns_motors =
                update_obstacle_avoidance(line_reported, now_us,
                                          ultrasonic_sample_ready);
        }
        if (line_follow_enabled && !finish_stopped &&
            !obstacle_owns_motors &&
            line_reported != 0xff) {
            update_line_follow(line_reported, now_us);
        }
        if (ultrasonic_sample_ready) {
            const uint8_t telemetry_sensor_state =
                line_reported == 0xff ? line_now : line_reported;
            log_realtime_telemetry(telemetry_sensor_state, now_us);
        }
        vTaskDelay(pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }
}
