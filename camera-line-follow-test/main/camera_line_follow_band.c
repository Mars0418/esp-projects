#include "camera_line_follow.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define DRIVER_STBY GPIO_NUM_5

#define MOTOR_A_PWM GPIO_NUM_6
#define MOTOR_A_IN1 GPIO_NUM_15
#define MOTOR_A_IN2 GPIO_NUM_7
#define MOTOR_A_CHANNEL LEDC_CHANNEL_0

#define MOTOR_B_PWM GPIO_NUM_11
#define MOTOR_B_IN1 GPIO_NUM_9
#define MOTOR_B_IN2 GPIO_NUM_10
#define MOTOR_B_CHANNEL LEDC_CHANNEL_1

#define MOTOR_D_PWM GPIO_NUM_40
#define MOTOR_D_IN1 GPIO_NUM_42
#define MOTOR_D_IN2 GPIO_NUM_41
#define MOTOR_D_CHANNEL LEDC_CHANNEL_2

#define ULTRASONIC_TRIG_GPIO GPIO_NUM_48
#define ULTRASONIC_ECHO_GPIO GPIO_NUM_39

#define ENCODER_A_PHASE_A_GPIO GPIO_NUM_16
#define ENCODER_A_PHASE_B_GPIO GPIO_NUM_17
#define ENCODER_B_PHASE_A_GPIO GPIO_NUM_8
#define ENCODER_B_PHASE_B_GPIO GPIO_NUM_18
#define ENCODER_D_PHASE_A_GPIO GPIO_NUM_2
#define ENCODER_D_PHASE_B_GPIO GPIO_NUM_1

/* The chassis needs a short high-duty launch to overcome static friction. */
#define FOLLOW_BASE_DUTY 170
#define FOLLOW_LAUNCH_DUTY 220
#define FOLLOW_LAUNCH_FRAMES 4
#define FOLLOW_MIN_FORWARD_DUTY 150
#define FOLLOW_MAX_FORWARD_DUTY 270
#define FOLLOW_PROPORTIONAL_GAIN 180
#define FOLLOW_CORRECTION_MAX 120
#define FOLLOW_CORRECTION_STEP 24
#define FOLLOW_ERROR_DEADBAND 20
#define FOLLOW_INTEGRAL_ENABLE_ERROR 45
#define FOLLOW_INTEGRAL_LIMIT 9600
#define FOLLOW_INTEGRAL_DIVISOR 800
#define FOLLOW_MIN_CONFIDENCE 20
#define FOLLOW_FRAME_TIMEOUT_MS 600
#define TURN_HINT_ERROR 650
#define TURN_HINT_FAR_WEIGHT 40
#define TURN_HINT_CONFIRM_FRAMES 2
#define TURN_HINT_MAX_AGE_MS 1200
#define TURN_GEOMETRY_MIN_CONFIDENCE 50
#define TURN_GEOMETRY_STRONG_CONFIDENCE 75
#define TURN_GEOMETRY_MIN_ANGLE 45
#define TURN_REARM_COOLDOWN_MS 500
#define TURN_CLEAN_STRAIGHT_FRAMES 3
#define TURN_CLEAN_NEAR_DELTA_MAX 12
#define FOOT_LOST_CONFIRM_FRAMES 2
#define FOOT_LOST_STOP_FRAMES 3
#define PIVOT_INNER_REVERSE_DUTY 135
#define PIVOT_OUTER_FORWARD_DUTY 145
#define PIVOT_SLOW_INNER_REVERSE_DUTY 130
#define PIVOT_SLOW_OUTER_FORWARD_DUTY 140
#define PIVOT_FAST_PHASE_MS 160
#define PIVOT_MAX_MS 1800
#define PIVOT_REACQUIRE_FRAMES 2
#define FOLLOW_STATUS_INTERVAL_MS 2000
#define STEERING_SIGN -1

/* Reuse the calibrated infrared-version obstacle bypass. Camera following
 * holds the chassis on the incoming line before an obstacle is detected. */
#define ULTRASONIC_SAMPLE_INTERVAL_MS 80
#define ULTRASONIC_ECHO_TIMEOUT_US 25000
#define ULTRASONIC_MOTOR_QUIET_US 2000
#define OBSTACLE_TRIGGER_MM 120
#define OBSTACLE_CONFIRM_SAMPLES 3
#define OBSTACLE_PRE_STRAFE_WAIT_MS 500
#define OBSTACLE_PRE_STRAFE_BRAKE_MS 80
#define OBSTACLE_ALIGN_HEADING_DEADBAND 120
#define OBSTACLE_ALIGN_PRIMARY_MS 80
#define OBSTACLE_ALIGN_COUNTER_MS 40
#define OBSTACLE_ALIGN_SETTLE_MS 200
#define OBSTACLE_ALIGN_BRAKE_MS 80
#define OBSTACLE_ALIGN_PRIMARY_DUTY 145
#define OBSTACLE_ALIGN_COUNTER_DUTY 140
#define OBSTACLE_ALIGN_MAX_ATTEMPTS 3
#define ENCODER_COUNTS_PER_REV 406
#define OBSTACLE_LEFT_REVS_NUM 12
#define OBSTACLE_LEFT_REVS_DEN 5
#define OBSTACLE_LEFT_REAR_TARGET_COUNTS \
    ((ENCODER_COUNTS_PER_REV * OBSTACLE_LEFT_REVS_NUM + \
      OBSTACLE_LEFT_REVS_DEN / 2) / OBSTACLE_LEFT_REVS_DEN)
#define OBSTACLE_LEFT_STRAFE_TIMEOUT_MS 5000
#define OBSTACLE_LEFT_SLOWDOWN_COUNTS 300
#define OBSTACLE_LEFT_POSITION_TOLERANCE 12
#define OBSTACLE_LEFT_SYNC_CORRECTION_MAX 35
#define OBSTACLE_STRAFE_BRAKE_MS 80
#define OBSTACLE_SETTLE_SAMPLE_MS 50
#define OBSTACLE_SETTLE_SAMPLES 3
#define OBSTACLE_SETTLE_MAX_DELTA 2
#define OBSTACLE_SETTLE_TIMEOUT_MS 1000
#define OBSTACLE_FORWARD_DUTY 180
#define OBSTACLE_FORWARD_TARGET_COUNTS \
    ((ENCODER_COUNTS_PER_REV * 9 + 2) / 5)
#define OBSTACLE_FORWARD_SLOWDOWN_COUNTS 200
#define OBSTACLE_FORWARD_POSITION_TOLERANCE 8
#define OBSTACLE_FORWARD_MIN_DUTY 140
#define OBSTACLE_FORWARD_MAX_DUTY 230
#define OBSTACLE_FORWARD_SYNC_CORRECTION_MAX 25
#define OBSTACLE_FORWARD_IMBALANCE_ABORT 120
#define OBSTACLE_FORWARD_BRAKE_MS 80
#define OBSTACLE_FORWARD_TIMEOUT_MS 5000
#define OBSTACLE_RIGHT_SEARCH_TIMEOUT_MS 8000
#define OBSTACLE_REACQUIRE_CONFIRM_FRAMES 2
#define OBSTACLE_CAMERA_CENTER_ERROR_MAX 260
#define OBSTACLE_RIGHT_SYNC_CORRECTION_MAX 35
#define OBSTACLE_FINAL_LOST_FRAMES 3

#define KIWI_STRAFE_DIAGONAL_DUTY 150
#define KIWI_STRAFE_REAR_DUTY 300
#define KIWI_RIGHT_STRAFE_DIAGONAL_DUTY 100
#define KIWI_RIGHT_STRAFE_REAR_DUTY 200
#define KIWI_LEFT_A_DIRECTION -1
#define KIWI_LEFT_B_DIRECTION -1
#define KIWI_LEFT_D_DIRECTION 1
#define STRAFE_SPEED_PI_UPDATE_MS 150
#define STRAFE_SPEED_TARGET_NUM_LEFT 3
#define STRAFE_SPEED_TARGET_DEN_LEFT 2
#define STRAFE_SPEED_TARGET_NUM_RIGHT 3
#define STRAFE_SPEED_TARGET_DEN_RIGHT 4
#define STRAFE_SPEED_PI_KP_DIV 4
#define STRAFE_SPEED_PI_KI_DIV 100
#define STRAFE_SPEED_PI_INTEGRAL_MAX 4000
#define STRAFE_SPEED_PI_OUTPUT_MAX 80
#define STRAFE_DIAGONAL_MIN_DUTY 120
#define STRAFE_DIAGONAL_MAX_DUTY 230
#define STRAFE_REAR_MIN_DUTY 220
#define STRAFE_REAR_MAX_DUTY 420
#define STRAFE_LEFT_SLOW_DIAGONAL_MIN_DUTY 90
#define STRAFE_LEFT_SLOW_REAR_MIN_DUTY 180
#define STRAFE_RIGHT_DIAGONAL_MIN_DUTY 90
#define STRAFE_RIGHT_DIAGONAL_MAX_DUTY 180
#define STRAFE_RIGHT_REAR_MIN_DUTY 170
#define STRAFE_RIGHT_REAR_MAX_DUTY 320

#define FORWARD_SPEED_PI_UPDATE_MS 150
#define FORWARD_SPEED_TARGET_NUM 3
#define FORWARD_SPEED_TARGET_DEN 2
#define FORWARD_SPEED_PI_KP_DIV 4
#define FORWARD_SPEED_PI_KI_DIV 100
#define FORWARD_SPEED_PI_INTEGRAL_MAX 1500
#define FORWARD_SPEED_PI_OUTPUT_MAX 35

#define NORMAL_UART_BAUD 115200
#define TUNER_UART_BAUD 921600
#define TUNER_TX_DRAIN_TIMEOUT_MS 700

typedef struct {
    gpio_num_t pwm;
    gpio_num_t in1;
    gpio_num_t in2;
    ledc_channel_t channel;
} motor_t;

typedef struct {
    int filtered_error;
    int integral_error;
    int correction;
    int a_command;
    int d_command;
    int tracking_frames;
    int hint_candidate_direction;
    int hint_candidate_frames;
    int clean_straight_frames;
    int foot_lost_frames;
    int turn_direction;
    int reacquire_frames;
    int64_t turn_hint_us;
    int64_t turn_armed_us;
    int64_t turn_arm_ignore_until_us;
    int64_t pivot_started_us;
    bool initialized;
    bool turn_armed;
    bool pivot_active;
} pursuit_controller_t;

typedef struct {
    gpio_num_t phase_a;
    gpio_num_t phase_b;
    volatile int32_t count;
    volatile uint8_t previous_state;
} encoder_t;

enum {
    WHEEL_A = 0,
    WHEEL_B = 1,
    WHEEL_D = 2,
    WHEEL_COUNT = 3,
};

typedef enum {
    OBSTACLE_IDLE,
    OBSTACLE_WAIT_BEFORE_LEFT,
    OBSTACLE_ALIGN_PRIMARY,
    OBSTACLE_ALIGN_COUNTER,
    OBSTACLE_ALIGN_SETTLE,
    OBSTACLE_STRAFE_LEFT,
    OBSTACLE_STRAFE_LEFT_BRAKE,
    OBSTACLE_STRAFE_LEFT_SETTLE,
    OBSTACLE_FORWARD,
    OBSTACLE_FORWARD_BRAKE,
    OBSTACLE_FORWARD_SETTLE,
    OBSTACLE_STRAFE_RIGHT_FIND_CENTER,
    OBSTACLE_STRAFE_RIGHT_BRAKE,
    OBSTACLE_STRAFE_RIGHT_SETTLE,
    OBSTACLE_FINAL_FORWARD,
} obstacle_state_t;

typedef struct {
    obstacle_state_t state;
    int near_samples;
    int align_direction;
    int align_attempts;
    int reacquire_frames;
    int final_lost_frames;
    int latest_distance_mm;
    int64_t next_ultrasonic_sample_us;
    int64_t state_started_us;
    int32_t strafe_start_count[WHEEL_COUNT];
    int32_t forward_start_a_count;
    int32_t forward_start_d_count;
    int32_t settle_previous_count[WHEEL_COUNT];
    int64_t settle_next_sample_us;
    int settle_stable_samples;
} obstacle_controller_t;

static const char *TAG = "CAMERA_PURSUIT";
static const motor_t s_motor_a = {
    MOTOR_A_PWM, MOTOR_A_IN1, MOTOR_A_IN2, MOTOR_A_CHANNEL};
static const motor_t s_motor_b = {
    MOTOR_B_PWM, MOTOR_B_IN1, MOTOR_B_IN2, MOTOR_B_CHANNEL};
static const motor_t s_motor_d = {
    MOTOR_D_PWM, MOTOR_D_IN1, MOTOR_D_IN2, MOTOR_D_CHANNEL};
static const motor_t *const s_motors[WHEEL_COUNT] = {
    &s_motor_a, &s_motor_b, &s_motor_d};
static encoder_t s_encoders[WHEEL_COUNT] = {
    {ENCODER_A_PHASE_A_GPIO, ENCODER_A_PHASE_B_GPIO, 0, 0},
    {ENCODER_B_PHASE_A_GPIO, ENCODER_B_PHASE_B_GPIO, 0, 0},
    {ENCODER_D_PHASE_A_GPIO, ENCODER_D_PHASE_B_GPIO, 0, 0},
};

static int64_t s_strafe_pi_last_update_us;
static int32_t s_strafe_pi_previous[WHEEL_COUNT];
static int s_strafe_pi_integral[WHEEL_COUNT];
static int s_strafe_pi_output[WHEEL_COUNT];
static int s_strafe_balance_output[WHEEL_COUNT];
static int64_t s_forward_pi_last_update_us;
static int32_t s_forward_pi_previous_a;
static int32_t s_forward_pi_previous_d;
static int s_forward_pi_integral_a;
static int s_forward_pi_integral_d;
static int s_forward_pi_output_a;
static int s_forward_pi_output_d;

static portMUX_TYPE s_result_lock = portMUX_INITIALIZER_UNLOCKED;
static line_vision_result_t s_latest_result;
static int64_t s_latest_frame_us;
static uint32_t s_result_sequence;
static volatile bool s_enabled;
static volatile bool s_debug_enabled;
static volatile bool s_tuner_enabled;
static volatile bool s_calibration_enabled;
static char s_uart_line[64];
static size_t s_uart_line_length;

static int clamp_int(int value, int minimum, int maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

static void set_output_low(gpio_num_t pin)
{
    const gpio_config_t config = {
        .pin_bit_mask = 1ULL << pin,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&config));
    ESP_ERROR_CHECK(gpio_set_level(pin, 0));
}

static void motor_prepare(const motor_t *motor, int direction, int duty)
{
    duty = clamp_int(duty, 0, 1023);
    ESP_ERROR_CHECK(gpio_set_level(motor->in1, direction > 0));
    ESP_ERROR_CHECK(gpio_set_level(motor->in2, direction < 0));
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, motor->channel, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, motor->channel));
}

static void stop_motors(void)
{
    motor_prepare(&s_motor_a, 0, 0);
    motor_prepare(&s_motor_b, 0, 0);
    motor_prepare(&s_motor_d, 0, 0);
    ESP_ERROR_CHECK(gpio_set_level(DRIVER_STBY, 0));
}

/* Positive command means physical forward. The board's A/D forward polarity
 * is -1, as verified by the infrared line-following program. */
static void drive_wheels(int a_command, int d_command)
{
    const int a_direction = a_command > 0 ? -1 : (a_command < 0 ? 1 : 0);
    const int d_direction = d_command > 0 ? -1 : (d_command < 0 ? 1 : 0);
    motor_prepare(&s_motor_a, a_direction, abs(a_command));
    motor_prepare(&s_motor_b, 0, 0);
    motor_prepare(&s_motor_d, d_direction, abs(d_command));
    ESP_ERROR_CHECK(gpio_set_level(DRIVER_STBY, 1));
}

static const int8_t s_quadrature_delta[16] = {
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

static void encoder_gpio_isr(void *argument)
{
    encoder_t *encoder = (encoder_t *)argument;
    const uint8_t current_state = encoder_read_state(encoder);
    const uint8_t transition =
        (uint8_t)((encoder->previous_state << 2) | current_state);
    encoder->count += s_quadrature_delta[transition];
    encoder->previous_state = current_state;
}

static esp_err_t configure_encoders(void)
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
    ESP_RETURN_ON_ERROR(gpio_config(&config), TAG,
                        "encoder GPIO config failed");
    const esp_err_t service_error = gpio_install_isr_service(0);
    if (service_error != ESP_OK && service_error != ESP_ERR_INVALID_STATE) {
        return service_error;
    }
    for (size_t wheel = 0; wheel < WHEEL_COUNT; ++wheel) {
        s_encoders[wheel].count = 0;
        s_encoders[wheel].previous_state =
            encoder_read_state(&s_encoders[wheel]);
        ESP_RETURN_ON_ERROR(
            gpio_isr_handler_add(s_encoders[wheel].phase_a,
                                 encoder_gpio_isr, &s_encoders[wheel]),
            TAG, "encoder phase A handler failed");
        ESP_RETURN_ON_ERROR(
            gpio_isr_handler_add(s_encoders[wheel].phase_b,
                                 encoder_gpio_isr, &s_encoders[wheel]),
            TAG, "encoder phase B handler failed");
    }
    return ESP_OK;
}

static void configure_ultrasonic(void)
{
    set_output_low(ULTRASONIC_TRIG_GPIO);
    const gpio_config_t echo_config = {
        .pin_bit_mask = 1ULL << ULTRASONIC_ECHO_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&echo_config));
}

static int read_ultrasonic_distance_mm(void)
{
    int64_t deadline_us =
        esp_timer_get_time() + ULTRASONIC_ECHO_TIMEOUT_US;
    while (gpio_get_level(ULTRASONIC_ECHO_GPIO)) {
        if (esp_timer_get_time() >= deadline_us) return -1;
    }

    ESP_ERROR_CHECK(gpio_set_level(ULTRASONIC_TRIG_GPIO, 0));
    esp_rom_delay_us(2);
    ESP_ERROR_CHECK(gpio_set_level(ULTRASONIC_TRIG_GPIO, 1));
    esp_rom_delay_us(10);
    ESP_ERROR_CHECK(gpio_set_level(ULTRASONIC_TRIG_GPIO, 0));

    deadline_us = esp_timer_get_time() + ULTRASONIC_ECHO_TIMEOUT_US;
    while (!gpio_get_level(ULTRASONIC_ECHO_GPIO)) {
        if (esp_timer_get_time() >= deadline_us) return -1;
    }
    const int64_t echo_start_us = esp_timer_get_time();
    deadline_us = echo_start_us + ULTRASONIC_ECHO_TIMEOUT_US;
    while (gpio_get_level(ULTRASONIC_ECHO_GPIO)) {
        if (esp_timer_get_time() >= deadline_us) return -1;
    }
    const int64_t echo_width_us = esp_timer_get_time() - echo_start_us;
    return (int)((echo_width_us * 10 + 29) / 58);
}

static bool update_ultrasonic(obstacle_controller_t *obstacle,
                              int64_t now_us)
{
    if (now_us < obstacle->next_ultrasonic_sample_us) return false;
    obstacle->next_ultrasonic_sample_us = now_us +
        (int64_t)ULTRASONIC_SAMPLE_INTERVAL_MS * 1000;

    const bool driver_was_enabled =
        gpio_get_level(DRIVER_STBY) != 0;
    if (driver_was_enabled) {
        ESP_ERROR_CHECK(gpio_set_level(DRIVER_STBY, 0));
        esp_rom_delay_us(ULTRASONIC_MOTOR_QUIET_US);
    }
    obstacle->latest_distance_mm = read_ultrasonic_distance_mm();
    if (driver_was_enabled) {
        ESP_ERROR_CHECK(gpio_set_level(DRIVER_STBY, 1));
    }
    return true;
}

static void brake_motors(void)
{
    for (size_t wheel = 0; wheel < WHEEL_COUNT; ++wheel) {
        ESP_ERROR_CHECK(gpio_set_level(s_motors[wheel]->in1, 1));
        ESP_ERROR_CHECK(gpio_set_level(s_motors[wheel]->in2, 1));
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE,
                                      s_motors[wheel]->channel, 1023));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE,
                                         s_motors[wheel]->channel));
    }
    ESP_ERROR_CHECK(gpio_set_level(DRIVER_STBY, 1));
}

static void reset_strafe_pi(int64_t now_us)
{
    s_strafe_pi_last_update_us = now_us;
    for (size_t wheel = 0; wheel < WHEEL_COUNT; ++wheel) {
        s_strafe_pi_previous[wheel] = s_encoders[wheel].count;
        s_strafe_pi_integral[wheel] = 0;
        s_strafe_pi_output[wheel] = 0;
        s_strafe_balance_output[wheel] = 0;
    }
}

static void update_strafe_pi(bool left, int speed_percent,
                             int64_t now_us)
{
    if (now_us - s_strafe_pi_last_update_us <
        (int64_t)STRAFE_SPEED_PI_UPDATE_MS * 1000) return;

    const int diagonal_request = left ? KIWI_STRAFE_DIAGONAL_DUTY
                                      : KIWI_RIGHT_STRAFE_DIAGONAL_DUTY;
    const int rear_request = left ? KIWI_STRAFE_REAR_DUTY
                                  : KIWI_RIGHT_STRAFE_REAR_DUTY;
    const int target_num = left ? STRAFE_SPEED_TARGET_NUM_LEFT
                                : STRAFE_SPEED_TARGET_NUM_RIGHT;
    const int target_den = left ? STRAFE_SPEED_TARGET_DEN_LEFT
                                : STRAFE_SPEED_TARGET_DEN_RIGHT;
    const int request[WHEEL_COUNT] = {
        diagonal_request, rear_request, diagonal_request};
    for (size_t wheel = 0; wheel < WHEEL_COUNT; ++wheel) {
        const int32_t count = s_encoders[wheel].count;
        const int delta = abs(count - s_strafe_pi_previous[wheel]);
        s_strafe_pi_previous[wheel] = count;
        const int target = request[wheel] * speed_percent / 100 *
                           target_num / target_den;
        const int error = target - delta;
        s_strafe_pi_integral[wheel] = clamp_int(
            s_strafe_pi_integral[wheel] + error,
            -STRAFE_SPEED_PI_INTEGRAL_MAX,
            STRAFE_SPEED_PI_INTEGRAL_MAX);
        s_strafe_pi_output[wheel] = clamp_int(
            error / STRAFE_SPEED_PI_KP_DIV +
                s_strafe_pi_integral[wheel] /
                    STRAFE_SPEED_PI_KI_DIV,
            -STRAFE_SPEED_PI_OUTPUT_MAX,
            STRAFE_SPEED_PI_OUTPUT_MAX);
    }
    s_strafe_pi_last_update_us = now_us;
}

static void apply_strafe(bool left, int speed_percent)
{
    const int direction = left ? 1 : -1;
    const int diagonal_request =
        (left ? KIWI_STRAFE_DIAGONAL_DUTY
              : KIWI_RIGHT_STRAFE_DIAGONAL_DUTY) * speed_percent / 100;
    const int rear_request =
        (left ? KIWI_STRAFE_REAR_DUTY
              : KIWI_RIGHT_STRAFE_REAR_DUTY) * speed_percent / 100;
    const int diagonal_min = left
        ? (speed_percent < 100 ? STRAFE_LEFT_SLOW_DIAGONAL_MIN_DUTY
                               : STRAFE_DIAGONAL_MIN_DUTY)
        : STRAFE_RIGHT_DIAGONAL_MIN_DUTY;
    const int rear_min = left
        ? (speed_percent < 100 ? STRAFE_LEFT_SLOW_REAR_MIN_DUTY
                               : STRAFE_REAR_MIN_DUTY)
        : STRAFE_RIGHT_REAR_MIN_DUTY;
    const int diagonal_max = left ? STRAFE_DIAGONAL_MAX_DUTY
                                  : STRAFE_RIGHT_DIAGONAL_MAX_DUTY;
    const int rear_max = left ? STRAFE_REAR_MAX_DUTY
                              : STRAFE_RIGHT_REAR_MAX_DUTY;
    const int a_duty = clamp_int(
        diagonal_request + s_strafe_pi_output[WHEEL_A] +
            s_strafe_balance_output[WHEEL_A],
        diagonal_min, diagonal_max);
    const int b_duty = clamp_int(
        rear_request + s_strafe_pi_output[WHEEL_B] +
            s_strafe_balance_output[WHEEL_B],
        rear_min, rear_max);
    const int d_duty = clamp_int(
        diagonal_request + s_strafe_pi_output[WHEEL_D] +
            s_strafe_balance_output[WHEEL_D],
        diagonal_min, diagonal_max);
    motor_prepare(&s_motor_a, direction * KIWI_LEFT_A_DIRECTION, a_duty);
    motor_prepare(&s_motor_b, direction * KIWI_LEFT_B_DIRECTION, b_duty);
    motor_prepare(&s_motor_d, direction * KIWI_LEFT_D_DIRECTION, d_duty);
    ESP_ERROR_CHECK(gpio_set_level(DRIVER_STBY, 1));
}

static void reset_forward_pi(int64_t now_us)
{
    s_forward_pi_last_update_us = now_us;
    s_forward_pi_previous_a = s_encoders[WHEEL_A].count;
    s_forward_pi_previous_d = s_encoders[WHEEL_D].count;
    s_forward_pi_integral_a = 0;
    s_forward_pi_integral_d = 0;
    s_forward_pi_output_a = 0;
    s_forward_pi_output_d = 0;
}

static void update_forward_pi(int a_request, int d_request,
                              int64_t now_us)
{
    if (now_us - s_forward_pi_last_update_us <
        (int64_t)FORWARD_SPEED_PI_UPDATE_MS * 1000) return;
    const int32_t count_a = s_encoders[WHEEL_A].count;
    const int32_t count_d = s_encoders[WHEEL_D].count;
    const int delta_a = abs(count_a - s_forward_pi_previous_a);
    const int delta_d = abs(count_d - s_forward_pi_previous_d);
    s_forward_pi_previous_a = count_a;
    s_forward_pi_previous_d = count_d;
    s_forward_pi_last_update_us = now_us;
    const int error_a = a_request * FORWARD_SPEED_TARGET_NUM /
                            FORWARD_SPEED_TARGET_DEN - delta_a;
    const int error_d = d_request * FORWARD_SPEED_TARGET_NUM /
                            FORWARD_SPEED_TARGET_DEN - delta_d;
    s_forward_pi_integral_a = clamp_int(
        s_forward_pi_integral_a + error_a,
        -FORWARD_SPEED_PI_INTEGRAL_MAX,
        FORWARD_SPEED_PI_INTEGRAL_MAX);
    s_forward_pi_integral_d = clamp_int(
        s_forward_pi_integral_d + error_d,
        -FORWARD_SPEED_PI_INTEGRAL_MAX,
        FORWARD_SPEED_PI_INTEGRAL_MAX);
    s_forward_pi_output_a = clamp_int(
        error_a / FORWARD_SPEED_PI_KP_DIV +
            s_forward_pi_integral_a / FORWARD_SPEED_PI_KI_DIV,
        -FORWARD_SPEED_PI_OUTPUT_MAX, FORWARD_SPEED_PI_OUTPUT_MAX);
    s_forward_pi_output_d = clamp_int(
        error_d / FORWARD_SPEED_PI_KP_DIV +
            s_forward_pi_integral_d / FORWARD_SPEED_PI_KI_DIV,
        -FORWARD_SPEED_PI_OUTPUT_MAX, FORWARD_SPEED_PI_OUTPUT_MAX);
}

static bool result_ready(const line_vision_result_t *result,
                         int64_t frame_us, int64_t now_us)
{
    return result->found && result->path_point_count >= 2 &&
           result->path_length_pixels >= 9 &&
           result->confidence >= FOLLOW_MIN_CONFIDENCE && frame_us > 0 &&
           now_us - frame_us <= (int64_t)FOLLOW_FRAME_TIMEOUT_MS * 1000;
}

static void reset_controller(pursuit_controller_t *controller);
static void apply_tracking_control(int unsigned_error, int base_duty,
                                   pursuit_controller_t *controller);

static const char *obstacle_state_name(obstacle_state_t state)
{
    switch (state) {
    case OBSTACLE_IDLE: return "IDLE";
    case OBSTACLE_WAIT_BEFORE_LEFT: return "WAIT_LEFT";
    case OBSTACLE_ALIGN_PRIMARY: return "ALIGN_OUT";
    case OBSTACLE_ALIGN_COUNTER: return "ALIGN_BACK";
    case OBSTACLE_ALIGN_SETTLE: return "ALIGN_SETTLE";
    case OBSTACLE_STRAFE_LEFT: return "LEFT";
    case OBSTACLE_STRAFE_LEFT_BRAKE: return "LEFT_BRAKE";
    case OBSTACLE_STRAFE_LEFT_SETTLE: return "LEFT_SETTLE";
    case OBSTACLE_FORWARD: return "FORWARD";
    case OBSTACLE_FORWARD_BRAKE: return "FORWARD_BRAKE";
    case OBSTACLE_FORWARD_SETTLE: return "FORWARD_SETTLE";
    case OBSTACLE_STRAFE_RIGHT_FIND_CENTER: return "RIGHT_FIND";
    case OBSTACLE_STRAFE_RIGHT_BRAKE: return "RIGHT_BRAKE";
    case OBSTACLE_STRAFE_RIGHT_SETTLE: return "RIGHT_SETTLE";
    case OBSTACLE_FINAL_FORWARD: return "FINAL";
    default: return "INVALID";
    }
}

static void reset_obstacle_controller(obstacle_controller_t *obstacle,
                                      int64_t now_us)
{
    memset(obstacle, 0, sizeof(*obstacle));
    obstacle->state = OBSTACLE_IDLE;
    obstacle->latest_distance_mm = -1;
    obstacle->next_ultrasonic_sample_us = now_us;
    reset_strafe_pi(now_us);
    reset_forward_pi(now_us);
}

static void get_strafe_progress(const obstacle_controller_t *obstacle,
                                int progress[WHEEL_COUNT])
{
    progress[WHEEL_A] =
        2 * abs(s_encoders[WHEEL_A].count -
                obstacle->strafe_start_count[WHEEL_A]);
    progress[WHEEL_B] =
        abs(s_encoders[WHEEL_B].count -
            obstacle->strafe_start_count[WHEEL_B]);
    progress[WHEEL_D] =
        2 * abs(s_encoders[WHEEL_D].count -
                obstacle->strafe_start_count[WHEEL_D]);
}

static int progress_average(const int progress[WHEEL_COUNT])
{
    return (progress[WHEEL_A] + progress[WHEEL_B] +
            progress[WHEEL_D]) / WHEEL_COUNT;
}

static void fail_obstacle_action(obstacle_controller_t *obstacle,
                                 const char *reason)
{
    stop_motors();
    s_enabled = false;
    obstacle->state = OBSTACLE_IDLE;
    ESP_LOGE(TAG, "OBSTACLE failed: %s; stopped", reason);
}

static void start_obstacle_left(obstacle_controller_t *obstacle,
                                int64_t now_us)
{
    stop_motors();
    obstacle->state = OBSTACLE_STRAFE_LEFT;
    obstacle->state_started_us = now_us;
    obstacle->near_samples = 0;
    for (size_t wheel = 0; wheel < WHEEL_COUNT; ++wheel) {
        obstacle->strafe_start_count[wheel] = s_encoders[wheel].count;
    }
    reset_strafe_pi(now_us);
    apply_strafe(true, 100);
    ESP_LOGW(TAG,
             "OBSTACLE %dmm confirmed: STRAFE LEFT target=%d counts",
             obstacle->latest_distance_mm,
             OBSTACLE_LEFT_REAR_TARGET_COUNTS);
}

static void start_obstacle_wait(obstacle_controller_t *obstacle,
                                int64_t now_us)
{
    obstacle->state = OBSTACLE_WAIT_BEFORE_LEFT;
    obstacle->state_started_us = now_us;
    obstacle->near_samples = 0;
    brake_motors();
    ESP_LOGW(TAG,
             "OBSTACLE %dmm confirmed: STOP %dms before strafe left",
             obstacle->latest_distance_mm,
             OBSTACLE_PRE_STRAFE_WAIT_MS);
}

static void drive_obstacle_alignment_pivot(int direction, int duty)
{
    if (direction > 0) {
        drive_wheels(-duty, duty);
    } else {
        drive_wheels(duty, -duty);
    }
}

static void start_obstacle_alignment_pulse(
    obstacle_controller_t *obstacle, int heading_error, int64_t now_us)
{
    obstacle->state = OBSTACLE_ALIGN_PRIMARY;
    obstacle->state_started_us = now_us;
    obstacle->align_direction =
        STEERING_SIGN * heading_error > 0 ? 1 : -1;
    obstacle->align_attempts++;
    drive_obstacle_alignment_pivot(obstacle->align_direction,
                                   OBSTACLE_ALIGN_PRIMARY_DUTY);
    ESP_LOGW(TAG,
             "OBSTACLE ALIGN attempt=%d heading=%d direction=%d pulse=%d/%dms",
             obstacle->align_attempts, heading_error,
             obstacle->align_direction, OBSTACLE_ALIGN_PRIMARY_MS,
             OBSTACLE_ALIGN_COUNTER_MS);
}

static bool check_obstacle_trigger(obstacle_controller_t *obstacle,
                                   bool ultrasonic_sample_ready,
                                   int64_t now_us)
{
    if (obstacle->state != OBSTACLE_IDLE ||
        !ultrasonic_sample_ready) return false;
    const int distance_mm = obstacle->latest_distance_mm;
    if (distance_mm > 0 && distance_mm <= OBSTACLE_TRIGGER_MM) {
        obstacle->near_samples++;
    } else {
        obstacle->near_samples = 0;
    }
    if (obstacle->near_samples < OBSTACLE_CONFIRM_SAMPLES) return false;
    start_obstacle_wait(obstacle, now_us);
    return true;
}

static void start_settle(obstacle_controller_t *obstacle,
                         obstacle_state_t state, int64_t now_us)
{
    stop_motors();
    obstacle->state = state;
    obstacle->state_started_us = now_us;
    obstacle->settle_next_sample_us = now_us +
        (int64_t)OBSTACLE_SETTLE_SAMPLE_MS * 1000;
    obstacle->settle_stable_samples = 0;
    for (size_t wheel = 0; wheel < WHEEL_COUNT; ++wheel) {
        obstacle->settle_previous_count[wheel] =
            s_encoders[wheel].count;
    }
}

static bool wheels_have_settled(obstacle_controller_t *obstacle,
                                int64_t now_us)
{
    stop_motors();
    if (now_us < obstacle->settle_next_sample_us) return false;
    bool stopped = true;
    for (size_t wheel = 0; wheel < WHEEL_COUNT; ++wheel) {
        const int32_t count = s_encoders[wheel].count;
        if (abs(count - obstacle->settle_previous_count[wheel]) >
            OBSTACLE_SETTLE_MAX_DELTA) {
            stopped = false;
        }
        obstacle->settle_previous_count[wheel] = count;
    }
    obstacle->settle_stable_samples = stopped
        ? obstacle->settle_stable_samples + 1 : 0;
    obstacle->settle_next_sample_us = now_us +
        (int64_t)OBSTACLE_SETTLE_SAMPLE_MS * 1000;
    return obstacle->settle_stable_samples >= OBSTACLE_SETTLE_SAMPLES;
}

static void update_left_strafe(obstacle_controller_t *obstacle,
                               int64_t now_us)
{
    int progress[WHEEL_COUNT];
    get_strafe_progress(obstacle, progress);
    const int average = progress_average(progress);
    const int remaining = OBSTACLE_LEFT_REAR_TARGET_COUNTS - average;
    const int elapsed_ms =
        (int)((now_us - obstacle->state_started_us) / 1000);
    if (average >= OBSTACLE_LEFT_REAR_TARGET_COUNTS -
                       OBSTACLE_LEFT_POSITION_TOLERANCE) {
        obstacle->state = OBSTACLE_STRAFE_LEFT_BRAKE;
        obstacle->state_started_us = now_us;
        brake_motors();
        ESP_LOGW(TAG, "STRAFE LEFT reached Aeq=%d B=%d Deq=%d",
                 progress[WHEEL_A], progress[WHEEL_B], progress[WHEEL_D]);
        return;
    }
    if (elapsed_ms >= OBSTACLE_LEFT_STRAFE_TIMEOUT_MS) {
        fail_obstacle_action(obstacle, "left strafe timeout");
        return;
    }
    const int speed_percent = remaining >= OBSTACLE_LEFT_SLOWDOWN_COUNTS
        ? 100
        : clamp_int(45 + 55 * (remaining > 0 ? remaining : 0) /
                              OBSTACLE_LEFT_SLOWDOWN_COUNTS,
                    45, 100);
    for (size_t wheel = 0; wheel < WHEEL_COUNT; ++wheel) {
        s_strafe_balance_output[wheel] = clamp_int(
            (average - progress[wheel]) / 6,
            -OBSTACLE_LEFT_SYNC_CORRECTION_MAX,
            OBSTACLE_LEFT_SYNC_CORRECTION_MAX);
    }
    update_strafe_pi(true, speed_percent, now_us);
    apply_strafe(true, speed_percent);
}

static void start_obstacle_forward(obstacle_controller_t *obstacle,
                                   int64_t now_us)
{
    stop_motors();
    obstacle->state = OBSTACLE_FORWARD;
    obstacle->state_started_us = now_us;
    obstacle->forward_start_a_count = s_encoders[WHEEL_A].count;
    obstacle->forward_start_d_count = s_encoders[WHEEL_D].count;
    reset_forward_pi(now_us);
    drive_wheels(OBSTACLE_FORWARD_DUTY, OBSTACLE_FORWARD_DUTY);
    ESP_LOGW(TAG, "OBSTACLE FORWARD target=%d counts",
             OBSTACLE_FORWARD_TARGET_COUNTS);
}

static void update_obstacle_forward(obstacle_controller_t *obstacle,
                                    int64_t now_us)
{
    const int a_progress =
        abs(s_encoders[WHEEL_A].count -
            obstacle->forward_start_a_count);
    const int d_progress =
        abs(s_encoders[WHEEL_D].count -
            obstacle->forward_start_d_count);
    const int average = (a_progress + d_progress) / 2;
    const int remaining = OBSTACLE_FORWARD_TARGET_COUNTS - average;
    const int elapsed_ms =
        (int)((now_us - obstacle->state_started_us) / 1000);
    if (average >= OBSTACLE_FORWARD_TARGET_COUNTS -
                       OBSTACLE_FORWARD_POSITION_TOLERANCE) {
        if (abs(a_progress - d_progress) >
            OBSTACLE_FORWARD_IMBALANCE_ABORT) {
            fail_obstacle_action(obstacle,
                                 "forward encoder imbalance");
            return;
        }
        obstacle->state = OBSTACLE_FORWARD_BRAKE;
        obstacle->state_started_us = now_us;
        brake_motors();
        ESP_LOGW(TAG, "OBSTACLE FORWARD reached A=%d D=%d",
                 a_progress, d_progress);
        return;
    }
    if (elapsed_ms >= OBSTACLE_FORWARD_TIMEOUT_MS) {
        fail_obstacle_action(obstacle, "forward timeout");
        return;
    }
    const int speed_percent = remaining >=
            OBSTACLE_FORWARD_SLOWDOWN_COUNTS
        ? 100
        : clamp_int(55 + 45 * (remaining > 0 ? remaining : 0) /
                              OBSTACLE_FORWARD_SLOWDOWN_COUNTS,
                    55, 100);
    const int base_duty = OBSTACLE_FORWARD_DUTY * speed_percent / 100;
    const int a_sync = clamp_int((average - a_progress) / 4,
        -OBSTACLE_FORWARD_SYNC_CORRECTION_MAX,
        OBSTACLE_FORWARD_SYNC_CORRECTION_MAX);
    const int d_sync = clamp_int((average - d_progress) / 4,
        -OBSTACLE_FORWARD_SYNC_CORRECTION_MAX,
        OBSTACLE_FORWARD_SYNC_CORRECTION_MAX);
    update_forward_pi(base_duty, base_duty, now_us);
    const int a_duty = clamp_int(
        base_duty + s_forward_pi_output_a + a_sync,
        OBSTACLE_FORWARD_MIN_DUTY, OBSTACLE_FORWARD_MAX_DUTY);
    const int d_duty = clamp_int(
        base_duty + s_forward_pi_output_d + d_sync,
        OBSTACLE_FORWARD_MIN_DUTY, OBSTACLE_FORWARD_MAX_DUTY);
    drive_wheels(a_duty, d_duty);
}

static void start_right_search(obstacle_controller_t *obstacle,
                               int64_t now_us)
{
    stop_motors();
    obstacle->state = OBSTACLE_STRAFE_RIGHT_FIND_CENTER;
    obstacle->state_started_us = now_us;
    obstacle->reacquire_frames = 0;
    for (size_t wheel = 0; wheel < WHEEL_COUNT; ++wheel) {
        obstacle->strafe_start_count[wheel] = s_encoders[wheel].count;
    }
    reset_strafe_pi(now_us);
    apply_strafe(false, 100);
    ESP_LOGW(TAG, "OBSTACLE STRAFE RIGHT until camera line centered");
}

static void update_right_search(obstacle_controller_t *obstacle,
                                const line_vision_result_t *result,
                                bool line_valid, bool new_frame,
                                int64_t now_us)
{
    int progress[WHEEL_COUNT];
    get_strafe_progress(obstacle, progress);
    const int elapsed_ms =
        (int)((now_us - obstacle->state_started_us) / 1000);
    if (elapsed_ms >= OBSTACLE_RIGHT_SEARCH_TIMEOUT_MS) {
        fail_obstacle_action(obstacle, "right strafe line timeout");
        return;
    }
    if (new_frame) {
        const bool centered = line_valid && result->foot_track_valid &&
            abs(result->foot_lateral_error) <=
                OBSTACLE_CAMERA_CENTER_ERROR_MAX;
        obstacle->reacquire_frames = centered
            ? obstacle->reacquire_frames + 1 : 0;
        if (obstacle->reacquire_frames >=
            OBSTACLE_REACQUIRE_CONFIRM_FRAMES) {
            obstacle->state = OBSTACLE_STRAFE_RIGHT_BRAKE;
            obstacle->state_started_us = now_us;
            brake_motors();
            ESP_LOGW(TAG,
                     "CAMERA LINE CENTERED err=%d frames=%d Aeq=%d B=%d Deq=%d",
                     result->foot_lateral_error,
                     obstacle->reacquire_frames,
                     progress[WHEEL_A], progress[WHEEL_B],
                     progress[WHEEL_D]);
            return;
        }
        if (obstacle->reacquire_frames > 0) {
            /* Pause immediately on the first centered frame so the 15 fps
             * confirmation interval does not carry the chassis past center. */
            brake_motors();
            return;
        }
    } else if (obstacle->reacquire_frames > 0) {
        brake_motors();
        return;
    }

    const int average = progress_average(progress);
    for (size_t wheel = 0; wheel < WHEEL_COUNT; ++wheel) {
        s_strafe_balance_output[wheel] = clamp_int(
            (average - progress[wheel]) / 6,
            -OBSTACLE_RIGHT_SYNC_CORRECTION_MAX,
            OBSTACLE_RIGHT_SYNC_CORRECTION_MAX);
    }
    update_strafe_pi(false, 100, now_us);
    apply_strafe(false, 100);
}

static void start_final_forward(obstacle_controller_t *obstacle,
                                pursuit_controller_t *controller,
                                int64_t now_us)
{
    obstacle->state = OBSTACLE_FINAL_FORWARD;
    obstacle->state_started_us = now_us;
    obstacle->final_lost_frames = 0;
    reset_controller(controller);
    controller->a_command = FOLLOW_BASE_DUTY;
    controller->d_command = FOLLOW_BASE_DUTY;
    drive_wheels(controller->a_command, controller->d_command);
    ESP_LOGW(TAG,
             "OBSTACLE BYPASS COMPLETE: FINAL LINE FOLLOW, turns disabled");
}

/* Return true while the obstacle/final-course state machine owns all motors. */
static bool update_obstacle_action(obstacle_controller_t *obstacle,
                                   pursuit_controller_t *controller,
                                   const line_vision_result_t *result,
                                   bool line_valid, bool new_frame,
                                   int64_t now_us)
{
    const int elapsed_ms =
        (int)((now_us - obstacle->state_started_us) / 1000);
    switch (obstacle->state) {
    case OBSTACLE_IDLE:
        return false;
    case OBSTACLE_WAIT_BEFORE_LEFT:
        if (elapsed_ms >= OBSTACLE_PRE_STRAFE_WAIT_MS) {
            if (line_valid && result->foot_track_valid &&
                abs(result->heading_error) >
                    OBSTACLE_ALIGN_HEADING_DEADBAND) {
                start_obstacle_alignment_pulse(
                    obstacle, result->heading_error, now_us);
            } else {
                ESP_LOGW(TAG,
                         "OBSTACLE ALIGN skipped heading=%d line=%d foot=%d",
                         result->heading_error, line_valid,
                         result->foot_track_valid);
                start_obstacle_left(obstacle, now_us);
            }
        } else if (elapsed_ms < OBSTACLE_PRE_STRAFE_BRAKE_MS) {
            brake_motors();
        } else {
            stop_motors();
        }
        return true;
    case OBSTACLE_ALIGN_PRIMARY:
        if (elapsed_ms >= OBSTACLE_ALIGN_PRIMARY_MS) {
            obstacle->state = OBSTACLE_ALIGN_COUNTER;
            obstacle->state_started_us = now_us;
            drive_obstacle_alignment_pivot(
                -obstacle->align_direction,
                OBSTACLE_ALIGN_COUNTER_DUTY);
        } else {
            drive_obstacle_alignment_pivot(
                obstacle->align_direction,
                OBSTACLE_ALIGN_PRIMARY_DUTY);
        }
        return true;
    case OBSTACLE_ALIGN_COUNTER:
        if (elapsed_ms >= OBSTACLE_ALIGN_COUNTER_MS) {
            obstacle->state = OBSTACLE_ALIGN_SETTLE;
            obstacle->state_started_us = now_us;
            brake_motors();
        } else {
            drive_obstacle_alignment_pivot(
                -obstacle->align_direction,
                OBSTACLE_ALIGN_COUNTER_DUTY);
        }
        return true;
    case OBSTACLE_ALIGN_SETTLE:
        if (elapsed_ms < OBSTACLE_ALIGN_BRAKE_MS) {
            brake_motors();
            return true;
        }
        stop_motors();
        if (elapsed_ms < OBSTACLE_ALIGN_SETTLE_MS) return true;
        if (!line_valid || !result->foot_track_valid) {
            ESP_LOGW(TAG,
                     "OBSTACLE ALIGN line unavailable after attempt=%d; continue",
                     obstacle->align_attempts);
            start_obstacle_left(obstacle, now_us);
        } else if (abs(result->heading_error) <=
                       OBSTACLE_ALIGN_HEADING_DEADBAND) {
            ESP_LOGW(TAG,
                     "OBSTACLE ALIGN complete attempt=%d heading=%d",
                     obstacle->align_attempts, result->heading_error);
            start_obstacle_left(obstacle, now_us);
        } else if (obstacle->align_attempts >=
                       OBSTACLE_ALIGN_MAX_ATTEMPTS) {
            ESP_LOGW(TAG,
                     "OBSTACLE ALIGN limit heading=%d attempts=%d; continue",
                     result->heading_error,
                     obstacle->align_attempts);
            start_obstacle_left(obstacle, now_us);
        } else {
            start_obstacle_alignment_pulse(
                obstacle, result->heading_error, now_us);
        }
        return true;
    case OBSTACLE_STRAFE_LEFT:
        update_left_strafe(obstacle, now_us);
        return true;
    case OBSTACLE_STRAFE_LEFT_BRAKE:
        if (elapsed_ms >= OBSTACLE_STRAFE_BRAKE_MS) {
            start_settle(obstacle, OBSTACLE_STRAFE_LEFT_SETTLE,
                         now_us);
        } else {
            brake_motors();
        }
        return true;
    case OBSTACLE_STRAFE_LEFT_SETTLE:
        if (elapsed_ms >= OBSTACLE_SETTLE_TIMEOUT_MS) {
            fail_obstacle_action(obstacle, "left wheels did not settle");
        } else if (wheels_have_settled(obstacle, now_us)) {
            start_obstacle_forward(obstacle, now_us);
        }
        return true;
    case OBSTACLE_FORWARD:
        update_obstacle_forward(obstacle, now_us);
        return true;
    case OBSTACLE_FORWARD_BRAKE:
        if (elapsed_ms >= OBSTACLE_FORWARD_BRAKE_MS) {
            start_settle(obstacle, OBSTACLE_FORWARD_SETTLE, now_us);
        } else {
            brake_motors();
        }
        return true;
    case OBSTACLE_FORWARD_SETTLE:
        if (elapsed_ms >= OBSTACLE_SETTLE_TIMEOUT_MS) {
            fail_obstacle_action(obstacle,
                                 "forward wheels did not settle");
        } else if (wheels_have_settled(obstacle, now_us)) {
            start_right_search(obstacle, now_us);
        }
        return true;
    case OBSTACLE_STRAFE_RIGHT_FIND_CENTER:
        update_right_search(obstacle, result, line_valid, new_frame,
                            now_us);
        return true;
    case OBSTACLE_STRAFE_RIGHT_BRAKE:
        if (elapsed_ms >= OBSTACLE_STRAFE_BRAKE_MS) {
            start_settle(obstacle, OBSTACLE_STRAFE_RIGHT_SETTLE,
                         now_us);
        } else {
            brake_motors();
        }
        return true;
    case OBSTACLE_STRAFE_RIGHT_SETTLE:
        if (elapsed_ms >= OBSTACLE_SETTLE_TIMEOUT_MS) {
            fail_obstacle_action(obstacle,
                                 "right wheels did not settle");
        } else if (wheels_have_settled(obstacle, now_us)) {
            start_final_forward(obstacle, controller, now_us);
        }
        return true;
    case OBSTACLE_FINAL_FORWARD:
        if (new_frame) {
            const bool foot_valid =
                line_valid && result->foot_track_valid;
            obstacle->final_lost_frames = foot_valid
                ? 0 : obstacle->final_lost_frames + 1;
            if (obstacle->final_lost_frames >=
                OBSTACLE_FINAL_LOST_FRAMES) {
                stop_motors();
                s_enabled = false;
                ESP_LOGW(TAG,
                         "FINAL black line lost for %d frames: stopped",
                         obstacle->final_lost_frames);
                return true;
            }
            if (foot_valid) {
                /* Reuse ordinary foot-position correction, but deliberately
                 * skip update_turn_hint(), arm_turn() and every pivot state. */
                apply_tracking_control(result->foot_lateral_error,
                                       FOLLOW_BASE_DUTY, controller);
                return true;
            }
        }
        drive_wheels(controller->a_command, controller->d_command);
        return true;
    default:
        fail_obstacle_action(obstacle, "invalid obstacle state");
        return true;
    }
}

static void reset_controller(pursuit_controller_t *controller)
{
    memset(controller, 0, sizeof(*controller));
}

static bool turn_hint_recent(const pursuit_controller_t *controller,
                             int64_t now_us)
{
    return controller->turn_direction != 0 &&
           controller->turn_hint_us > 0 &&
           now_us - controller->turn_hint_us <=
               (int64_t)TURN_HINT_MAX_AGE_MS * 1000;
}

static void update_turn_hint(const line_vision_result_t *result,
                             pursuit_controller_t *controller,
                             int64_t now_us)
{
    const int signed_turn_error = STEERING_SIGN * result->steering_error;
    const bool preview_hint = abs(signed_turn_error) >= TURN_HINT_ERROR &&
                              result->far_preview_weight >=
                                  TURN_HINT_FAR_WEIGHT;
    const int preview_direction = signed_turn_error > 0 ? 1 : -1;
    const bool geometry_hint = result->big_turn &&
        result->turn_angle_deg >= TURN_GEOMETRY_MIN_ANGLE &&
        result->turn_confidence >= TURN_GEOMETRY_MIN_CONFIDENCE;
    const int geometry_direction = STEERING_SIGN * result->turn_direction;
    int direction = 0;
    if (geometry_hint && preview_hint &&
        geometry_direction != preview_direction) {
        if (result->turn_confidence >=
            TURN_GEOMETRY_STRONG_CONFIDENCE) {
            direction = geometry_direction;
        }
    } else if (geometry_hint) {
        direction = geometry_direction;
    } else if (preview_hint) {
        direction = preview_direction;
    }
    const bool strong_hint = direction != 0;
    if (!strong_hint) {
        if (controller->hint_candidate_frames > 0) {
            controller->hint_candidate_frames--;
        }
        if (controller->hint_candidate_frames == 0) {
            controller->hint_candidate_direction = 0;
        }
        if (!turn_hint_recent(controller, now_us)) {
            controller->turn_direction = 0;
            controller->turn_hint_us = 0;
        }
        return;
    }

    if (direction == controller->hint_candidate_direction) {
        controller->hint_candidate_frames++;
    } else {
        controller->hint_candidate_direction = direction;
        controller->hint_candidate_frames = 1;
    }
    if (controller->hint_candidate_frames >= TURN_HINT_CONFIRM_FRAMES ||
        (geometry_hint && result->turn_confidence >=
             TURN_GEOMETRY_STRONG_CONFIDENCE)) {
        controller->turn_direction = direction;
        controller->turn_hint_us = now_us;
    }
}

static void apply_tracking_control(int unsigned_error, int base_duty,
                                   pursuit_controller_t *controller)
{
    const int effective_base_duty =
        controller->tracking_frames < FOLLOW_LAUNCH_FRAMES
            ? FOLLOW_LAUNCH_DUTY
            : base_duty;
    if (controller->tracking_frames < FOLLOW_LAUNCH_FRAMES) {
        controller->tracking_frames++;
    }
    const int signed_error = STEERING_SIGN * unsigned_error;
    if (!controller->initialized) {
        controller->filtered_error = signed_error;
        controller->initialized = true;
    } else {
        /* The foot point is already constrained to the under-car component,
         * so favor prompt recentering over the older, heavily damped filter. */
        controller->filtered_error =
            (controller->filtered_error + 3 * signed_error) / 4;
    }

    int control_error = controller->filtered_error;
    if (abs(control_error) <= FOLLOW_ERROR_DEADBAND) control_error = 0;
    if (abs(control_error) > FOLLOW_INTEGRAL_ENABLE_ERROR) {
        controller->integral_error = clamp_int(
            controller->integral_error + control_error,
            -FOLLOW_INTEGRAL_LIMIT, FOLLOW_INTEGRAL_LIMIT);
    } else {
        controller->integral_error = controller->integral_error * 7 / 8;
    }

    const int target_correction = clamp_int(
        control_error * FOLLOW_PROPORTIONAL_GAIN / 1000 +
            controller->integral_error / FOLLOW_INTEGRAL_DIVISOR,
        -FOLLOW_CORRECTION_MAX, FOLLOW_CORRECTION_MAX);
    controller->correction = clamp_int(
        target_correction,
        controller->correction - FOLLOW_CORRECTION_STEP,
        controller->correction + FOLLOW_CORRECTION_STEP);

    controller->a_command = clamp_int(
        effective_base_duty - controller->correction,
        FOLLOW_MIN_FORWARD_DUTY, FOLLOW_MAX_FORWARD_DUTY);
    controller->d_command = clamp_int(
        effective_base_duty + controller->correction,
        FOLLOW_MIN_FORWARD_DUTY, FOLLOW_MAX_FORWARD_DUTY);
    drive_wheels(controller->a_command, controller->d_command);
}

static void apply_follow_control(const line_vision_result_t *result,
                                 pursuit_controller_t *controller)
{
    /* Only the component connected to the vehicle-facing foot gate steers.
     * Far pixels record turn direction but never become the steering target. */
    apply_tracking_control(result->foot_lateral_error, FOLLOW_BASE_DUTY,
                           controller);
}

static void update_clean_straight(const line_vision_result_t *result,
                                  pursuit_controller_t *controller)
{
    const bool near_direction_clean =
        abs(result->near_lookahead_x - result->near_x) <=
            TURN_CLEAN_NEAR_DELTA_MAX;
    if (result->foot_track_valid && near_direction_clean) {
        controller->clean_straight_frames = clamp_int(
            controller->clean_straight_frames + 1, 0, 100);
    } else {
        controller->clean_straight_frames = 0;
    }
}

static void clear_pending_turn_hint(pursuit_controller_t *controller)
{
    controller->turn_direction = 0;
    controller->turn_hint_us = 0;
    controller->hint_candidate_direction = 0;
    controller->hint_candidate_frames = 0;
}

static bool arm_turn(const line_vision_result_t *result,
                     pursuit_controller_t *controller,
                     int64_t now_us)
{
    if (now_us < controller->turn_arm_ignore_until_us ||
        controller->clean_straight_frames <
            TURN_CLEAN_STRAIGHT_FRAMES ||
        !turn_hint_recent(controller, now_us) ||
        !result->foot_track_valid) {
        return false;
    }
    controller->turn_armed = true;
    controller->turn_armed_us = now_us;
    ESP_LOGW(TAG,
             "TURN READY direction=%d clean=%d foot=%d center=%d path=%d angle=%d tconf=%d steer=%d weight=%d",
             controller->turn_direction,
             controller->clean_straight_frames,
             result->foot_pixel_count,
             result->foot_center_pixel_count,
             result->foot_path_length_pixels,
             result->turn_angle_deg,
             result->turn_confidence,
             result->steering_error,
             result->far_preview_weight);
    return true;
}

static void drive_pivot(pursuit_controller_t *controller, int64_t now_us)
{
    const bool slow_search = controller->pivot_started_us > 0 &&
        now_us - controller->pivot_started_us >=
            (int64_t)PIVOT_FAST_PHASE_MS * 1000;
    const int inner_duty = slow_search
                               ? PIVOT_SLOW_INNER_REVERSE_DUTY
                               : PIVOT_INNER_REVERSE_DUTY;
    const int outer_duty = slow_search
                               ? PIVOT_SLOW_OUTER_FORWARD_DUTY
                               : PIVOT_OUTER_FORWARD_DUTY;
    if (controller->turn_direction > 0) {
        controller->a_command = -inner_duty;
        controller->d_command = outer_duty;
    } else {
        controller->a_command = outer_duty;
        controller->d_command = -inner_duty;
    }
    drive_wheels(controller->a_command, controller->d_command);
}

static void start_pivot(pursuit_controller_t *controller, int64_t now_us)
{
    controller->turn_armed = false;
    controller->pivot_active = true;
    controller->pivot_started_us = now_us;
    controller->reacquire_frames = 0;
    controller->integral_error = 0;
    controller->correction = 0;
    drive_pivot(controller, now_us);
    ESP_LOGW(TAG,
             "PIVOT START direction=%d fast=%d/%d slow=%d/%d after=%dms",
             controller->turn_direction, PIVOT_INNER_REVERSE_DUTY,
             PIVOT_OUTER_FORWARD_DUTY,
             PIVOT_SLOW_INNER_REVERSE_DUTY,
             PIVOT_SLOW_OUTER_FORWARD_DUTY,
             PIVOT_FAST_PHASE_MS);
}

static void set_normal_uart_mode(void)
{
    uart_wait_tx_done(UART_NUM_0,
                      pdMS_TO_TICKS(TUNER_TX_DRAIN_TIMEOUT_MS));
    ESP_ERROR_CHECK(uart_set_baudrate(UART_NUM_0, NORMAL_UART_BAUD));
    esp_log_level_set("*", ESP_LOG_INFO);
}

static void enter_binary_uart_mode(bool calibration)
{
    s_enabled = false;
    stop_motors();
    s_debug_enabled = false;
    s_tuner_enabled = false;
    s_calibration_enabled = false;
    esp_log_level_set("*", ESP_LOG_NONE);
    uart_wait_tx_done(UART_NUM_0,
                      pdMS_TO_TICKS(TUNER_TX_DRAIN_TIMEOUT_MS));
    ESP_ERROR_CHECK(uart_set_baudrate(UART_NUM_0, TUNER_UART_BAUD));
    s_tuner_enabled = !calibration;
    s_calibration_enabled = calibration;
}

static void process_uart_line(void)
{
    s_uart_line[s_uart_line_length] = '\0';
    int red;
    int green;
    int blue;
    int enabled;
    if (sscanf(s_uart_line, "RGB,%d,%d,%d", &red, &green, &blue) == 3) {
        red = clamp_int(red, 0, 255);
        green = clamp_int(green, 0, 255);
        blue = clamp_int(blue, 0, 255);
        line_vision_set_rgb_thresholds((uint8_t)red, (uint8_t)green,
                                       (uint8_t)blue);
        ESP_LOGI(TAG, "RGB_THRESHOLDS r=%d g=%d b=%d", red, green, blue);
    } else if (sscanf(s_uart_line, "TUNER,%d", &enabled) == 1) {
        if (enabled != 0) {
            enter_binary_uart_mode(false);
        } else {
            s_tuner_enabled = false;
            set_normal_uart_mode();
            ESP_LOGI(TAG, "TUNER disabled; UART restored to %d",
                     NORMAL_UART_BAUD);
        }
    } else if (sscanf(s_uart_line, "CALIB,%d", &enabled) == 1) {
        if (enabled != 0) {
            enter_binary_uart_mode(true);
        } else {
            s_calibration_enabled = false;
            set_normal_uart_mode();
            ESP_LOGI(TAG, "CALIB disabled; UART restored to %d",
                     NORMAL_UART_BAUD);
        }
    } else if (sscanf(s_uart_line, "DEBUG,%d", &enabled) == 1) {
        if (s_tuner_enabled || s_calibration_enabled) {
            s_tuner_enabled = false;
            s_calibration_enabled = false;
            set_normal_uart_mode();
        }
        s_debug_enabled = enabled != 0;
        if (s_debug_enabled) {
            ESP_LOGI(TAG, "RGB_DEBUG enabled=1; normal logs paused");
            esp_log_level_set("*", ESP_LOG_NONE);
        } else {
            esp_log_level_set("*", ESP_LOG_INFO);
            ESP_LOGI(TAG, "RGB_DEBUG enabled=0; normal logs resumed");
        }
    } else if (strcmp(s_uart_line, "STATUS") == 0) {
        const line_vision_rgb_thresholds_t thresholds =
            line_vision_get_rgb_thresholds();
        ESP_LOGI(TAG,
                 "RGB_THRESHOLDS r=%u g=%u b=%u debug=%d tuner=%d calib=%d",
                 thresholds.red, thresholds.green, thresholds.blue,
                 s_debug_enabled, s_tuner_enabled, s_calibration_enabled);
    }
    s_uart_line_length = 0;
}

static void handle_uart_command(void)
{
    uint8_t input[64];
    const int count = uart_read_bytes(UART_NUM_0, input, sizeof(input), 0);
    for (int index = 0; index < count; ++index) {
        const uint8_t value = input[index];
        if (value == 'f' || value == 'F') {
            if (s_tuner_enabled || s_calibration_enabled) {
                s_enabled = false;
                stop_motors();
                continue;
            }
            line_vision_result_t result;
            int64_t frame_us;
            portENTER_CRITICAL(&s_result_lock);
            result = s_latest_result;
            frame_us = s_latest_frame_us;
            portEXIT_CRITICAL(&s_result_lock);
            if (result_ready(&result, frame_us, esp_timer_get_time()) &&
                result.foot_track_valid) {
                s_enabled = true;
                ESP_LOGW(TAG, "PATH PURSUIT ENABLED");
            } else {
                s_enabled = false;
                stop_motors();
                ESP_LOGW(TAG,
                         "START REFUSED: no fresh track under foot gate");
            }
        } else if (value == 'x' || value == 'X' || value == ' ') {
            s_enabled = false;
            stop_motors();
            ESP_LOGW(TAG, "PATH PURSUIT STOPPED");
        } else if (value == '\r' || value == '\n') {
            if (s_uart_line_length > 0) process_uart_line();
        } else if (value >= 32 && value <= 126) {
            if (s_uart_line_length + 1 < sizeof(s_uart_line)) {
                s_uart_line[s_uart_line_length++] = (char)value;
            } else {
                s_uart_line_length = 0;
            }
        }
    }
}

static void control_task(void *argument)
{
    (void)argument;
    pursuit_controller_t controller;
    reset_controller(&controller);
    obstacle_controller_t obstacle;
    reset_obstacle_controller(&obstacle, esp_timer_get_time());
    uint32_t processed_sequence = 0;
    int64_t last_report_us = 0;
    bool previous_enabled = false;

    while (true) {
        handle_uart_command();
        line_vision_result_t result;
        int64_t frame_us;
        uint32_t sequence;
        portENTER_CRITICAL(&s_result_lock);
        result = s_latest_result;
        frame_us = s_latest_frame_us;
        sequence = s_result_sequence;
        portEXIT_CRITICAL(&s_result_lock);

        const int64_t now_us = esp_timer_get_time();
        const bool frame_fresh = frame_us > 0 &&
            now_us - frame_us <= (int64_t)FOLLOW_FRAME_TIMEOUT_MS * 1000;
        const bool line_valid = result_ready(&result, frame_us, now_us);
        const bool foot_valid = line_valid && result.foot_track_valid;
        const bool new_frame = sequence != processed_sequence;

        if (s_enabled && !previous_enabled) {
            reset_controller(&controller);
            reset_obstacle_controller(&obstacle, now_us);
        } else if (!s_enabled && previous_enabled) {
            reset_obstacle_controller(&obstacle, now_us);
        }

        bool ultrasonic_sample_ready = false;
        if (obstacle.state == OBSTACLE_IDLE &&
            !s_tuner_enabled && !s_calibration_enabled &&
            !s_debug_enabled) {
            ultrasonic_sample_ready =
                update_ultrasonic(&obstacle, now_us);
        }

        if (!s_enabled) {
            reset_controller(&controller);
        } else if (!frame_fresh) {
            s_enabled = false;
            stop_motors();
            reset_controller(&controller);
            reset_obstacle_controller(&obstacle, now_us);
            ESP_LOGE(TAG, "STALE CAMERA FRAME: motors stopped");
        } else {
            if (check_obstacle_trigger(&obstacle,
                                       ultrasonic_sample_ready,
                                       now_us)) {
                reset_controller(&controller);
            }

            const bool obstacle_owns_motors =
                update_obstacle_action(&obstacle, &controller, &result,
                                       line_valid, new_frame, now_us);
            if (obstacle_owns_motors) {
                if (new_frame) processed_sequence = sequence;
            } else if (new_frame) {
            if (controller.pivot_active) {
                const int64_t pivot_ms =
                    (now_us - controller.pivot_started_us) / 1000;
                if (pivot_ms >= PIVOT_MAX_MS) {
                    s_enabled = false;
                    stop_motors();
                    ESP_LOGW(TAG,
                             "PIVOT TIMEOUT after %lld ms: stopped",
                             (long long)pivot_ms);
                    reset_controller(&controller);
                } else {
                    /* Pivoting starts only after the previous track has left
                     * the foot gate. Stop on the first valid component that
                     * re-enters the wider foot gate; waiting for it to reach
                     * the center or become nearly aligned can rotate past the
                     * outgoing track and reacquire the incoming track. */
                    const bool reacquire_candidate = foot_valid;
                    if (reacquire_candidate) {
                        if (controller.reacquire_frames == 0) {
                            ESP_LOGI(TAG,
                                     "PIVOT CANDIDATE foot=%d center=%d flat=%d heading=%d: paused",
                                     result.foot_pixel_count,
                                     result.foot_center_pixel_count,
                                     result.foot_lateral_error,
                                     result.heading_error);
                        }
                        controller.reacquire_frames++;
                    } else {
                        controller.reacquire_frames = 0;
                    }
                    if (controller.reacquire_frames >=
                        PIVOT_REACQUIRE_FRAMES) {
                        const int completed_direction =
                            controller.turn_direction;
                        reset_controller(&controller);
                        controller.tracking_frames = FOLLOW_LAUNCH_FRAMES;
                        controller.turn_arm_ignore_until_us = now_us +
                            (int64_t)TURN_REARM_COOLDOWN_MS * 1000;
                        apply_follow_control(&result, &controller);
                        ESP_LOGW(TAG,
                                 "PIVOT COMPLETE direction=%d after %lldms foot=%d center=%d heading=%d",
                                 completed_direction, (long long)pivot_ms,
                                 result.foot_pixel_count,
                                 result.foot_center_pixel_count,
                                 result.heading_error);
                    } else if (controller.reacquire_frames > 0) {
                        /* Pause on the first aligned frame. At 15 FPS, still
                         * rotating during the confirmation frame is enough to
                         * overshoot a narrow outgoing line. */
                        controller.a_command = 0;
                        controller.d_command = 0;
                        drive_wheels(0, 0);
                    } else {
                        drive_pivot(&controller, now_us);
                    }
                }
            } else if (foot_valid) {
                controller.foot_lost_frames = 0;
                if (controller.turn_armed) {
                    /* Once a direction is confirmed, retain it until the
                     * under-vehicle track actually ends. Distance to the
                     * corner varies, so a time-based expiry can discard the
                     * correct direction immediately before arrival. */
                    apply_tracking_control(result.foot_lateral_error,
                                           FOLLOW_BASE_DUTY,
                                           &controller);
                } else {
                    update_clean_straight(&result, &controller);
                    if (now_us < controller.turn_arm_ignore_until_us) {
                        clear_pending_turn_hint(&controller);
                        apply_follow_control(&result, &controller);
                    } else {
                        update_turn_hint(&result, &controller, now_us);
                    }
                    if (now_us >= controller.turn_arm_ignore_until_us &&
                        arm_turn(&result, &controller, now_us)) {
                        apply_tracking_control(result.foot_lateral_error,
                                               FOLLOW_BASE_DUTY,
                                               &controller);
                    } else if (now_us >=
                               controller.turn_arm_ignore_until_us) {
                        apply_follow_control(&result, &controller);
                    }
                }
            } else {
                controller.foot_lost_frames++;
                if (controller.foot_lost_frames >=
                        FOOT_LOST_CONFIRM_FRAMES &&
                    controller.turn_armed) {
                    ESP_LOGW(TAG,
                             "FOOT TRACK ENDED direction=%d frames=%d vision=%d foot=%d center=%d path=%d",
                             controller.turn_direction,
                             controller.foot_lost_frames,
                             line_valid, result.foot_pixel_count,
                             result.foot_center_pixel_count,
                             result.foot_path_length_pixels);
                    start_pivot(&controller, now_us);
                } else if (controller.turn_armed) {
                    controller.a_command = 0;
                    controller.d_command = 0;
                    brake_motors();
                    ESP_LOGI(TAG,
                             "FOOT EDGE candidate direction=%d: braking for confirmation",
                             controller.turn_direction);
                } else if (controller.foot_lost_frames <
                               FOOT_LOST_STOP_FRAMES &&
                           controller.initialized) {
                    drive_wheels(controller.a_command,
                                 controller.d_command);
                } else if (controller.foot_lost_frames >=
                           FOOT_LOST_STOP_FRAMES) {
                    s_enabled = false;
                    stop_motors();
                    ESP_LOGW(TAG,
                             "FOOT TRACK LOST for %d frames without ready turn: stopped",
                             controller.foot_lost_frames);
                }
            }
            processed_sequence = sequence;
            }
        }

        if (now_us - last_report_us >=
            (int64_t)FOLLOW_STATUS_INTERVAL_MS * 1000) {
            const int64_t age_ms = frame_us > 0 ? (now_us - frame_us) / 1000
                                                : -1;
            const char *state = obstacle.state != OBSTACLE_IDLE
                ? obstacle_state_name(obstacle.state)
                : (controller.pivot_active ? "PIVOT" :
                    (controller.turn_armed ? "READY" : "FOLLOW"));
            ESP_LOGI(TAG,
                     "RUN en=%d state=%s line=%d foot=%d err=%d corr=%d pwm=%d/%d dist=%dmm age=%lldms",
                     s_enabled,
                     state,
                     line_valid, result.foot_track_valid,
                     controller.filtered_error,
                     controller.correction,
                     controller.a_command, controller.d_command,
                     obstacle.latest_distance_mm,
                     (long long)age_ms);
            last_report_us = now_us;
        }
        previous_enabled = s_enabled;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

esp_err_t camera_line_follow_init(void)
{
    set_output_low(DRIVER_STBY);
    const motor_t *motors[] = {&s_motor_a, &s_motor_b, &s_motor_d};
    for (size_t index = 0; index < 3; ++index) {
        set_output_low(motors[index]->in1);
        set_output_low(motors[index]->in2);
    }

    const ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 20000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_RETURN_ON_ERROR(ledc_timer_config(&timer), TAG, "PWM timer failed");
    for (size_t index = 0; index < 3; ++index) {
        const ledc_channel_config_t channel = {
            .gpio_num = motors[index]->pwm,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = motors[index]->channel,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = LEDC_TIMER_0,
            .duty = 0,
            .hpoint = 0,
        };
        ESP_RETURN_ON_ERROR(ledc_channel_config(&channel), TAG,
                            "PWM channel failed");
    }
    stop_motors();
    ESP_RETURN_ON_ERROR(configure_encoders(), TAG,
                        "encoder initialization failed");
    configure_ultrasonic();

    const esp_err_t uart_error = uart_driver_install(UART_NUM_0, 1024, 0, 0,
                                                      NULL, 0);
    if (uart_error != ESP_OK && uart_error != ESP_ERR_INVALID_STATE) {
        return uart_error;
    }
    if (xTaskCreatePinnedToCore(control_task, "camera_pursuit", 4096, NULL,
                                6, NULL, 0) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGW(TAG,
             "SAFE STOP. F=start X/SPACE=stop RGB,r,g,b DEBUG/TUNER/CALIB,0/1");
    ESP_LOGI(TAG,
             "Foot-track controller: gate=center-half/12rows follow=%d correction<=%d pivot=%d/%d reacquire=%dframes hint=%d@weight%d foot-lost=%dframes timeout=%dms",
             FOLLOW_BASE_DUTY,
             FOLLOW_CORRECTION_MAX, PIVOT_INNER_REVERSE_DUTY,
             PIVOT_OUTER_FORWARD_DUTY, PIVOT_REACQUIRE_FRAMES,
             TURN_HINT_ERROR,
             TURN_HINT_FAR_WEIGHT, FOOT_LOST_CONFIRM_FRAMES,
             PIVOT_MAX_MS);
    ESP_LOGI(TAG,
             "Obstacle bypass: trigger<=%dmm x%d wait=%dms align<=%d pulse=%d/%dms x%d left=%dcounts forward=%dcounts right-center<=%d final-lost=%dframes",
             OBSTACLE_TRIGGER_MM, OBSTACLE_CONFIRM_SAMPLES,
             OBSTACLE_PRE_STRAFE_WAIT_MS,
             OBSTACLE_ALIGN_HEADING_DEADBAND,
             OBSTACLE_ALIGN_PRIMARY_MS,
             OBSTACLE_ALIGN_COUNTER_MS,
             OBSTACLE_ALIGN_MAX_ATTEMPTS,
             OBSTACLE_LEFT_REAR_TARGET_COUNTS,
             OBSTACLE_FORWARD_TARGET_COUNTS,
             OBSTACLE_CAMERA_CENTER_ERROR_MAX,
             OBSTACLE_FINAL_LOST_FRAMES);
    return ESP_OK;
}

void camera_line_follow_submit(const line_vision_result_t *result,
                               int64_t captured_at_us)
{
    portENTER_CRITICAL(&s_result_lock);
    s_latest_result = *result;
    s_latest_frame_us = captured_at_us;
    s_result_sequence++;
    portEXIT_CRITICAL(&s_result_lock);
}

void camera_line_follow_camera_disconnected(void)
{
    portENTER_CRITICAL(&s_result_lock);
    s_latest_frame_us = 0;
    portEXIT_CRITICAL(&s_result_lock);
    s_enabled = false;
    stop_motors();
}

bool camera_line_follow_debug_enabled(void)
{
    return s_debug_enabled;
}

bool camera_line_follow_tuner_enabled(void)
{
    return s_tuner_enabled;
}

bool camera_line_follow_calibration_enabled(void)
{
    return s_calibration_enabled;
}
