#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
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
#include "tft_status_display.h"

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
#define PWM_DUTY          180
#define PWM_MAX_DUTY     1023
#define COMMAND_TIMEOUT_MS 180
#define CONTROL_PERIOD_MS   10
#define LINE_STABLE_SAMPLES  2
#define STATUS_REPORT_INTERVAL_MS 250
#define DISPLAY_SPEED_SAMPLE_MS   500
#define LINE_BASE_DUTY        180
#define LINE_MIN_DUTY         140
#define LINE_MAX_DUTY         230
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
#define FINISH_CONFIRM_SAMPLES   3
#define FINISH_ARM_DELAY_COUNTS (ENCODER_COUNTS_PER_REV / 2)
#define SPEED_PI_UPDATE_MS     150
#define SPEED_TARGET_NUM         3
#define SPEED_TARGET_DEN         2
#define SPEED_PI_KP_DIV          4
#define SPEED_PI_KI_DIV        100
#define SPEED_PI_INTEGRAL_MAX 1500
#define SPEED_PI_OUTPUT_MAX     35

/* HC-SR04 obstacle trigger and line-alignment stop action. ECHO must reach
 * GPIO39 through a 5 V -> 3.3 V divider. */
#define ULTRASONIC_SAMPLE_INTERVAL_MS       80
#define ULTRASONIC_ECHO_TIMEOUT_US       25000
#define ULTRASONIC_MOTOR_QUIET_US          2000
#define OBSTACLE_TRIGGER_MM                 100
#define OBSTACLE_CONFIRM_SAMPLES              3
#define OBSTACLE_ALIGN_TARGET_STATE          0x9
#define OBSTACLE_ALIGN_STABLE_SAMPLES          3
#define OBSTACLE_ALIGN_DUTY                  100
#define OBSTACLE_ALIGN_TIMEOUT_MS             6000
#define ENCODER_COUNTS_PER_REV                406
#define OBSTACLE_LEFT_REVS_NUM                  9
#define OBSTACLE_LEFT_REVS_DEN                  4
#define OBSTACLE_LEFT_REAR_TARGET_COUNTS       \
    ((ENCODER_COUNTS_PER_REV * OBSTACLE_LEFT_REVS_NUM + \
      OBSTACLE_LEFT_REVS_DEN / 2) / \
     OBSTACLE_LEFT_REVS_DEN)
#define OBSTACLE_LEFT_STRAFE_TIMEOUT_MS       5000
#define OBSTACLE_LEFT_SLOWDOWN_COUNTS          300
#define OBSTACLE_LEFT_POSITION_TOLERANCE         12
#define OBSTACLE_LEFT_SYNC_CORRECTION_MAX        35
#define OBSTACLE_LEFT_IMBALANCE_ABORT            180
#define OBSTACLE_STRAFE_BRAKE_MS                 80
#define OBSTACLE_STRAFE_SETTLE_SAMPLE_MS        50
#define OBSTACLE_STRAFE_SETTLE_SAMPLES           3
#define OBSTACLE_STRAFE_SETTLE_MAX_DELTA          2
#define OBSTACLE_STRAFE_SETTLE_TIMEOUT_MS      1000
#define OBSTACLE_FORWARD_DUTY                  180
#define OBSTACLE_FORWARD_TARGET_COUNTS         \
    (ENCODER_COUNTS_PER_REV * 3 / 2)
#define OBSTACLE_FORWARD_SLOWDOWN_COUNTS        200
#define OBSTACLE_FORWARD_POSITION_TOLERANCE       8
#define OBSTACLE_FORWARD_MIN_DUTY               140
#define OBSTACLE_FORWARD_MAX_DUTY               230
#define OBSTACLE_FORWARD_SYNC_CORRECTION_MAX     25
#define OBSTACLE_FORWARD_IMBALANCE_ABORT        120
#define OBSTACLE_FORWARD_BRAKE_MS                80
#define OBSTACLE_FORWARD_TIMEOUT_MS            5000
#define OBSTACLE_RIGHT_SEARCH_TIMEOUT_MS       8000
#define OBSTACLE_REACQUIRE_CONFIRM_SAMPLES        3
#define OBSTACLE_RIGHT_SYNC_CORRECTION_MAX        35
#define OBSTACLE_RIGHT_IMBALANCE_ABORT           180
#define OBSTACLE_RIGHT_BRAKE_MS                   80
#define KIWI_STRAFE_DIAGONAL_DUTY             150
#define KIWI_STRAFE_REAR_DUTY                 300
#define KIWI_RIGHT_STRAFE_DIAGONAL_DUTY       100
#define KIWI_RIGHT_STRAFE_REAR_DUTY           200
#define STRAFE_RIGHT_TARGET_NUM                  3
#define STRAFE_RIGHT_TARGET_DEN                  4
#define STRAFE_SPEED_PI_UPDATE_MS              150
#define STRAFE_SPEED_PI_KP_DIV                   4
#define STRAFE_SPEED_PI_KI_DIV                 100
#define STRAFE_SPEED_PI_INTEGRAL_MAX          4000
#define STRAFE_SPEED_PI_OUTPUT_MAX              80
#define STRAFE_DIAGONAL_MIN_DUTY               120
#define STRAFE_DIAGONAL_MAX_DUTY               230
#define STRAFE_REAR_MIN_DUTY                   220
#define STRAFE_REAR_MAX_DUTY                   420
#define STRAFE_LEFT_SLOW_DIAGONAL_MIN_DUTY      90
#define STRAFE_LEFT_SLOW_REAR_MIN_DUTY          180
#define STRAFE_RIGHT_DIAGONAL_MIN_DUTY          90
#define STRAFE_RIGHT_DIAGONAL_MAX_DUTY         180
#define STRAFE_RIGHT_REAR_MIN_DUTY             170
#define STRAFE_RIGHT_REAR_MAX_DUTY             320

/* 120-degree kiwi-wheel strafe test. A/D counter-rotate and the rear wheel
 * cancels their yaw component; the rear wheel therefore runs at about twice
 * the diagonal-wheel duty. */
#define KIWI_LEFT_A_DIRECTION                -1
#define KIWI_LEFT_B_DIRECTION                -1
#define KIWI_LEFT_D_DIRECTION                 1

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
    OBSTACLE_ALIGN_TO_LINE,
    OBSTACLE_STRAFE_LEFT,
    OBSTACLE_STRAFE_BRAKE,
    OBSTACLE_STRAFE_SETTLE,
    OBSTACLE_FORWARD_POSITION,
    OBSTACLE_FORWARD_BRAKE,
    OBSTACLE_FORWARD_SETTLE,
    OBSTACLE_STRAFE_RIGHT_FIND_LINE,
    OBSTACLE_STRAFE_RIGHT_BRAKE,
    OBSTACLE_STRAFE_RIGHT_SETTLE,
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
static int64_t command_deadline_us;
static bool line_follow_enabled;
static int last_line_error;
static int64_t line_lost_since_us;
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
static int64_t strafe_speed_pi_last_update_us;
static int32_t strafe_speed_pi_previous[3];
static int strafe_speed_pi_integral[3];
static int strafe_speed_pi_output[3];
static bool line_pid_initialized;
static int line_pid_filtered_error;
static int line_pid_integral;
static int line_pid_p;
static int line_pid_i;
static int line_pid_d;
static obstacle_state_t obstacle_state = OBSTACLE_IDLE;
static int obstacle_near_samples;
static int obstacle_align_stable_samples;
static int64_t ultrasonic_next_sample_us;
static int latest_ultrasonic_distance_mm = -1;
static int64_t obstacle_state_started_us;
static int32_t obstacle_strafe_start_count[3];
static int32_t obstacle_settle_previous_count[3];
static int64_t obstacle_settle_next_sample_us;
static int obstacle_settle_stable_samples;
static int strafe_balance_output[3];
static int32_t obstacle_forward_start_a_count;
static int32_t obstacle_forward_start_d_count;
static int32_t obstacle_right_start_count[3];
static int obstacle_reacquire_samples;
static bool finish_detection_wait_for_center;
static bool finish_detection_delay_active;
static bool finish_detection_armed;
static int finish_marker_samples;
static bool course_finished;
static int32_t finish_delay_start_a_count;
static int32_t finish_delay_start_d_count;

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

static void reset_strafe_speed_pi(int64_t now_us)
{
    strafe_speed_pi_last_update_us = now_us;
    for (size_t wheel = 0; wheel < 3; ++wheel) {
        strafe_speed_pi_previous[wheel] = encoders[wheel].count;
        strafe_speed_pi_integral[wheel] = 0;
        strafe_speed_pi_output[wheel] = 0;
        strafe_balance_output[wheel] = 0;
    }
}

/* During a kiwi strafe, A and D should accumulate equal encoder magnitudes
 * while B accumulates about twice as many. Independent PI loops hold those
 * targets even when floor friction loads one wheel more than the others. */
static bool update_strafe_speed_pi(bool left, int speed_percent,
                                   int64_t now_us)
{
    if (now_us - strafe_speed_pi_last_update_us <
        (int64_t)STRAFE_SPEED_PI_UPDATE_MS * 1000) {
        return false;
    }

    const int diagonal_request = left ? KIWI_STRAFE_DIAGONAL_DUTY
                                      : KIWI_RIGHT_STRAFE_DIAGONAL_DUTY;
    const int rear_request = left ? KIWI_STRAFE_REAR_DUTY
                                  : KIWI_RIGHT_STRAFE_REAR_DUTY;
    const int target_num = left ? SPEED_TARGET_NUM
                                : STRAFE_RIGHT_TARGET_NUM;
    const int target_den = left ? SPEED_TARGET_DEN
                                : STRAFE_RIGHT_TARGET_DEN;
    const int request[3] = {
        diagonal_request,
        rear_request,
        diagonal_request,
    };
    for (size_t wheel = 0; wheel < 3; ++wheel) {
        const int32_t count = encoders[wheel].count;
        const int delta = abs(count - strafe_speed_pi_previous[wheel]);
        strafe_speed_pi_previous[wheel] = count;

        const int target = request[wheel] * speed_percent / 100 *
                           target_num / target_den;
        const int error = target - delta;
        strafe_speed_pi_integral[wheel] =
            clamp_int(strafe_speed_pi_integral[wheel] + error,
                      -STRAFE_SPEED_PI_INTEGRAL_MAX,
                      STRAFE_SPEED_PI_INTEGRAL_MAX);
        strafe_speed_pi_output[wheel] =
            clamp_int(error / STRAFE_SPEED_PI_KP_DIV +
                          strafe_speed_pi_integral[wheel] /
                              STRAFE_SPEED_PI_KI_DIV,
                      -STRAFE_SPEED_PI_OUTPUT_MAX,
                      STRAFE_SPEED_PI_OUTPUT_MAX);
    }
    strafe_speed_pi_last_update_us = now_us;
    return true;
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

/* TB6612 short-brake mode: STBY high, both direction inputs high and PWM
 * fully on. Use it only for a brief state-machine interval; normal STOP keeps
 * STBY low so the car remains electrically quiet while parked. */
static void all_motors_brake(void)
{
    for (size_t i = 0; i < sizeof(motors) / sizeof(motors[0]); ++i) {
        ESP_ERROR_CHECK(gpio_set_level(motors[i].in1_pin, 1));
        ESP_ERROR_CHECK(gpio_set_level(motors[i].in2_pin, 1));
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE,
                                      motors[i].pwm_channel,
                                      PWM_MAX_DUTY));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE,
                                         motors[i].pwm_channel));
    }
    ESP_ERROR_CHECK(gpio_set_level(DRIVER_STBY, 1));
    current_motion = MOTION_STOP;
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

static void apply_kiwi_strafe_duties_scaled(bool left, int speed_percent)
{
    const int direction = left ? 1 : -1;
    const int diagonal_request =
        (left ? KIWI_STRAFE_DIAGONAL_DUTY
              : KIWI_RIGHT_STRAFE_DIAGONAL_DUTY) * speed_percent / 100;
    const int rear_request =
        (left ? KIWI_STRAFE_REAR_DUTY
              : KIWI_RIGHT_STRAFE_REAR_DUTY) * speed_percent / 100;
    const int diagonal_min =
        left ? (speed_percent < 100
                    ? STRAFE_LEFT_SLOW_DIAGONAL_MIN_DUTY
                    : STRAFE_DIAGONAL_MIN_DUTY)
             : STRAFE_RIGHT_DIAGONAL_MIN_DUTY;
    const int rear_min =
        left ? (speed_percent < 100
                    ? STRAFE_LEFT_SLOW_REAR_MIN_DUTY
                    : STRAFE_REAR_MIN_DUTY)
             : STRAFE_RIGHT_REAR_MIN_DUTY;
    const int diagonal_max = left ? STRAFE_DIAGONAL_MAX_DUTY
                                  : STRAFE_RIGHT_DIAGONAL_MAX_DUTY;
    const int rear_max = left ? STRAFE_REAR_MAX_DUTY
                              : STRAFE_RIGHT_REAR_MAX_DUTY;
    const uint32_t a_duty = (uint32_t)clamp_int(
        diagonal_request + strafe_speed_pi_output[MOTOR_A] +
            strafe_balance_output[MOTOR_A],
        diagonal_min, diagonal_max);
    const uint32_t b_duty = (uint32_t)clamp_int(
        rear_request + strafe_speed_pi_output[MOTOR_B] +
            strafe_balance_output[MOTOR_B],
        rear_min, rear_max);
    const uint32_t d_duty = (uint32_t)clamp_int(
        diagonal_request + strafe_speed_pi_output[MOTOR_D] +
            strafe_balance_output[MOTOR_D],
        diagonal_min, diagonal_max);
    motor_prepare(MOTOR_A,
                  direction * KIWI_LEFT_A_DIRECTION,
                  a_duty);
    motor_prepare(MOTOR_B,
                  direction * KIWI_LEFT_B_DIRECTION,
                  b_duty);
    motor_prepare(MOTOR_D,
                  direction * KIWI_LEFT_D_DIRECTION,
                  d_duty);
    ESP_ERROR_CHECK(gpio_set_level(DRIVER_STBY, 1));
    current_motion = left ? MOTION_STRAFE_LEFT : MOTION_STRAFE_RIGHT;
}

static void apply_kiwi_strafe_duties(bool left)
{
    apply_kiwi_strafe_duties_scaled(left, 100);
}

static void drive_kiwi_strafe(bool left)
{
    reset_strafe_speed_pi(esp_timer_get_time());
    apply_kiwi_strafe_duties(left);
}

static void update_kiwi_strafe(int64_t now_us)
{
    if (current_motion != MOTION_STRAFE_LEFT &&
        current_motion != MOTION_STRAFE_RIGHT) {
        return;
    }
    const bool left = current_motion == MOTION_STRAFE_LEFT;
    if (update_strafe_speed_pi(left, 100, now_us)) {
        apply_kiwi_strafe_duties(left);
    }
}

/* Rotate around the chassis centre until the two inner sensors are on black
 * and the two outer sensors are on white (1001). A negative weighted error
 * means the line is left of centre, matching the normal line-follow steering
 * convention. The rear omni wheel is released during alignment. */
static void drive_obstacle_line_alignment(uint8_t sensor_state)
{
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

    int error = black_count > 0
                    ? weighted_sum * LINE_ERROR_SCALE / black_count
                    : last_line_error;
    if (error == 0 && sensor_state != OBSTACLE_ALIGN_TARGET_STATE) {
        error = last_line_error;
    }
    if (error == 0) {
        force_stop();
        return;
    }

    if (error < 0) {
        motor_prepare(MOTOR_A, -1, OBSTACLE_ALIGN_DUTY);
        motor_prepare(MOTOR_D, 1, OBSTACLE_ALIGN_DUTY);
        current_motion = MOTION_LEFT;
    } else {
        motor_prepare(MOTOR_A, 1, OBSTACLE_ALIGN_DUTY);
        motor_prepare(MOTOR_D, -1, OBSTACLE_ALIGN_DUTY);
        current_motion = MOTION_RIGHT;
    }
    motor_prepare(MOTOR_B, 0, 0);
    ESP_ERROR_CHECK(gpio_set_level(DRIVER_STBY, 1));
    last_line_error = error;
}

static void cancel_obstacle_action(void)
{
    if (obstacle_state != OBSTACLE_IDLE) {
        ESP_LOGW(TAG, "OBSTACLE action cancelled");
    }
    obstacle_state = OBSTACLE_IDLE;
    obstacle_near_samples = 0;
    obstacle_align_stable_samples = 0;
}

static void fail_obstacle_action(const char *reason)
{
    force_stop();
    obstacle_state = OBSTACLE_IDLE;
    obstacle_near_samples = 0;
    obstacle_align_stable_samples = 0;
    line_follow_enabled = false;
    ESP_LOGE(TAG, "OBSTACLE action failed: %s; LINE FOLLOW DISABLED / STOP",
             reason);
}

static void start_obstacle_alignment(int distance_mm, int64_t now_us)
{
    force_stop();
    obstacle_state = OBSTACLE_ALIGN_TO_LINE;
    obstacle_state_started_us = now_us;
    obstacle_near_samples = 0;
    obstacle_align_stable_samples = 0;
    reset_line_pid();
    reset_wheel_speed_pi(now_us);
    ESP_LOGW(TAG, "OBSTACLE %d mm: align IR=1001", distance_mm);
}

static void start_obstacle_left_strafe(int64_t now_us)
{
    force_stop();
    obstacle_state = OBSTACLE_STRAFE_LEFT;
    obstacle_state_started_us = now_us;
    obstacle_align_stable_samples = 0;
    for (size_t wheel = 0; wheel < 3; ++wheel) {
        obstacle_strafe_start_count[wheel] = encoders[wheel].count;
    }
    reset_strafe_speed_pi(now_us);
    apply_kiwi_strafe_duties(true);
    ESP_LOGW(TAG,
             "OBSTACLE aligned IR=1001; closed-loop STRAFE LEFT target=%d rear-equivalent counts",
             OBSTACLE_LEFT_REAR_TARGET_COUNTS);
}

/* Convert all three encoder displacements to the rear-wheel-equivalent scale
 * used by the calibrated kiwi strafe (A:B:D is approximately 1:2:1). */
static void get_normalized_strafe_progress(const int32_t start_count[3],
                                           int progress[3])
{
    progress[MOTOR_A] =
        2 * abs(encoders[MOTOR_A].count -
                start_count[MOTOR_A]);
    progress[MOTOR_B] =
        abs(encoders[MOTOR_B].count -
            start_count[MOTOR_B]);
    progress[MOTOR_D] =
        2 * abs(encoders[MOTOR_D].count -
                start_count[MOTOR_D]);
}

/* Cascaded left-strafe controller. The outer position loop creates one common
 * speed profile, so A/B/D keep moving together and preserve chassis heading.
 * The inner speed PI plus cumulative normalized-position correction maintains
 * the calibrated A:B:D ratio of 1:2:1. No wheel stops independently. */
static void apply_obstacle_left_coordinated_control(const int progress[3],
                                                     int64_t now_us)
{
    const int progress_average =
        (progress[MOTOR_A] + progress[MOTOR_B] + progress[MOTOR_D]) / 3;
    const int remaining =
        OBSTACLE_LEFT_REAR_TARGET_COUNTS - progress_average;
    const int speed_percent =
        remaining >= OBSTACLE_LEFT_SLOWDOWN_COUNTS
            ? 100
            : clamp_int(45 + 55 * (remaining > 0 ? remaining : 0) /
                                  OBSTACLE_LEFT_SLOWDOWN_COUNTS,
                        45, 100);

    for (size_t wheel = 0; wheel < 3; ++wheel) {
        strafe_balance_output[wheel] = clamp_int(
            (progress_average - progress[wheel]) / 6,
            -OBSTACLE_LEFT_SYNC_CORRECTION_MAX,
            OBSTACLE_LEFT_SYNC_CORRECTION_MAX);
    }
    update_strafe_speed_pi(true, speed_percent, now_us);
    apply_kiwi_strafe_duties_scaled(true, speed_percent);
}

static void start_obstacle_strafe_brake(int64_t now_us,
                                        const int progress[3])
{
    obstacle_state = OBSTACLE_STRAFE_BRAKE;
    obstacle_state_started_us = now_us;
    all_motors_brake();
    ESP_LOGW(TAG,
             "STRAFE LEFT position reached Aeq=%d B=%d Deq=%d; active brake",
             progress[MOTOR_A], progress[MOTOR_B], progress[MOTOR_D]);
}

static void begin_obstacle_settle(obstacle_state_t settle_state,
                                  int64_t now_us)
{
    force_stop();
    obstacle_state = settle_state;
    obstacle_state_started_us = now_us;
    obstacle_settle_next_sample_us = now_us +
        (int64_t)OBSTACLE_STRAFE_SETTLE_SAMPLE_MS * 1000;
    obstacle_settle_stable_samples = 0;
    for (size_t wheel = 0; wheel < 3; ++wheel) {
        obstacle_settle_previous_count[wheel] = encoders[wheel].count;
        strafe_balance_output[wheel] = 0;
    }
}

static bool update_obstacle_settle(int64_t now_us)
{
    force_stop();
    if (now_us < obstacle_settle_next_sample_us) {
        return false;
    }

    bool wheels_stopped = true;
    for (size_t wheel = 0; wheel < 3; ++wheel) {
        const int32_t count = encoders[wheel].count;
        const int delta =
            abs(count - obstacle_settle_previous_count[wheel]);
        obstacle_settle_previous_count[wheel] = count;
        if (delta > OBSTACLE_STRAFE_SETTLE_MAX_DELTA) {
            wheels_stopped = false;
        }
    }
    obstacle_settle_stable_samples =
        wheels_stopped ? obstacle_settle_stable_samples + 1 : 0;
    obstacle_settle_next_sample_us = now_us +
        (int64_t)OBSTACLE_STRAFE_SETTLE_SAMPLE_MS * 1000;
    return obstacle_settle_stable_samples >=
        OBSTACLE_STRAFE_SETTLE_SAMPLES;
}

static void start_obstacle_strafe_settle(int64_t now_us,
                                         const int progress[3])
{
    begin_obstacle_settle(OBSTACLE_STRAFE_SETTLE, now_us);
    ESP_LOGW(TAG,
             "STRAFE LEFT target reached Aeq=%d B=%d Deq=%d; wait for wheels to stop",
             progress[MOTOR_A], progress[MOTOR_B], progress[MOTOR_D]);
}

static void start_obstacle_forward_position(int64_t now_us)
{
    force_stop();
    obstacle_state = OBSTACLE_FORWARD_POSITION;
    obstacle_state_started_us = now_us;
    obstacle_forward_start_a_count = encoders[MOTOR_A].count;
    obstacle_forward_start_d_count = encoders[MOTOR_D].count;
    line_pulse_epoch_us = now_us;
    reset_wheel_speed_pi(now_us);
    drive_line_duties(OBSTACLE_FORWARD_DUTY, OBSTACLE_FORWARD_DUTY);
    ESP_LOGW(TAG,
             "STRAFE settled; closed-loop FORWARD A/D for 1.5 revolutions (%d counts)",
             OBSTACLE_FORWARD_TARGET_COUNTS);
}

static void apply_obstacle_forward_position_control(int a_progress,
                                                    int d_progress,
                                                    int64_t now_us)
{
    const int progress_average = (a_progress + d_progress) / 2;
    const int remaining =
        OBSTACLE_FORWARD_TARGET_COUNTS - progress_average;
    const int speed_percent =
        remaining >= OBSTACLE_FORWARD_SLOWDOWN_COUNTS
            ? 100
            : clamp_int(55 + 45 * (remaining > 0 ? remaining : 0) /
                                  OBSTACLE_FORWARD_SLOWDOWN_COUNTS,
                        55, 100);
    const int base_duty =
        OBSTACLE_FORWARD_DUTY * speed_percent / 100;
    const int a_sync = clamp_int(
        (progress_average - a_progress) / 4,
        -OBSTACLE_FORWARD_SYNC_CORRECTION_MAX,
        OBSTACLE_FORWARD_SYNC_CORRECTION_MAX);
    const int d_sync = clamp_int(
        (progress_average - d_progress) / 4,
        -OBSTACLE_FORWARD_SYNC_CORRECTION_MAX,
        OBSTACLE_FORWARD_SYNC_CORRECTION_MAX);

    update_wheel_speed_pi((uint32_t)base_duty,
                          (uint32_t)base_duty, now_us);
    const uint32_t a_duty = (uint32_t)clamp_int(
        base_duty + speed_pi_output_a + a_sync,
        OBSTACLE_FORWARD_MIN_DUTY, OBSTACLE_FORWARD_MAX_DUTY);
    const uint32_t d_duty = (uint32_t)clamp_int(
        base_duty + speed_pi_output_d + d_sync,
        OBSTACLE_FORWARD_MIN_DUTY, OBSTACLE_FORWARD_MAX_DUTY);
    drive_line_duties(a_duty, d_duty);
}

static void start_obstacle_forward_brake(int64_t now_us,
                                         int a_counts, int d_counts)
{
    obstacle_state = OBSTACLE_FORWARD_BRAKE;
    obstacle_state_started_us = now_us;
    all_motors_brake();
    ESP_LOGW(TAG,
             "FORWARD position reached A=%d D=%d; active brake",
             a_counts, d_counts);
}

static void start_obstacle_forward_settle(int64_t now_us)
{
    begin_obstacle_settle(OBSTACLE_FORWARD_SETTLE, now_us);
    ESP_LOGW(TAG, "FORWARD brake complete; wait for wheels to stop");
}

static void start_obstacle_right_search(int64_t now_us,
                                        int a_counts, int d_counts)
{
    force_stop();
    obstacle_state = OBSTACLE_STRAFE_RIGHT_FIND_LINE;
    obstacle_state_started_us = now_us;
    obstacle_reacquire_samples = 0;
    for (size_t wheel = 0; wheel < 3; ++wheel) {
        obstacle_right_start_count[wheel] = encoders[wheel].count;
    }
    reset_strafe_speed_pi(now_us);
    apply_kiwi_strafe_duties(false);
    ESP_LOGW(TAG,
             "FORWARD complete A=%d D=%d; slow STRAFE RIGHT until line",
             a_counts, d_counts);
}

static void apply_obstacle_right_coordinated_control(const int progress[3],
                                                      int64_t now_us)
{
    const int progress_average =
        (progress[MOTOR_A] + progress[MOTOR_B] + progress[MOTOR_D]) / 3;
    for (size_t wheel = 0; wheel < 3; ++wheel) {
        strafe_balance_output[wheel] = clamp_int(
            (progress_average - progress[wheel]) / 6,
            -OBSTACLE_RIGHT_SYNC_CORRECTION_MAX,
            OBSTACLE_RIGHT_SYNC_CORRECTION_MAX);
    }
    update_strafe_speed_pi(false, 100, now_us);
    apply_kiwi_strafe_duties(false);
}

static void start_obstacle_right_brake(int64_t now_us,
                                       const int progress[3])
{
    obstacle_state = OBSTACLE_STRAFE_RIGHT_BRAKE;
    obstacle_state_started_us = now_us;
    all_motors_brake();
    ESP_LOGW(TAG,
             "RIGHT line found Aeq=%d B=%d Deq=%d; active brake",
             progress[MOTOR_A], progress[MOTOR_B], progress[MOTOR_D]);
}

static void start_obstacle_right_settle(int64_t now_us)
{
    begin_obstacle_settle(OBSTACLE_STRAFE_RIGHT_SETTLE, now_us);
    ESP_LOGW(TAG, "RIGHT brake complete; wait for wheels to stop");
}

static void resume_line_follow_after_obstacle(int64_t now_us)
{
    force_stop();
    obstacle_state = OBSTACLE_IDLE;
    obstacle_near_samples = 0;
    obstacle_align_stable_samples = 0;
    command_deadline_us = 0;
    line_follow_enabled = true;
    line_has_been_seen = true;
    line_lost_since_us = 0;
    line_lost_stop_reported = false;
    sharp_turn = SHARP_TURN_NONE;
    sharp_turn_since_us = 0;
    line_lost_search_active = false;
    line_lost_search_turn = SHARP_TURN_NONE;
    line_pulse_epoch_us = now_us;
    finish_detection_wait_for_center = true;
    finish_detection_delay_active = false;
    finish_detection_armed = false;
    finish_marker_samples = 0;
    course_finished = false;
    reset_line_pid();
    reset_wheel_speed_pi(now_us);
    reset_strafe_speed_pi(now_us);
    ESP_LOGW(TAG, "OBSTACLE line reacquired; RESUME LINE FOLLOW");
}

/* Return true while the obstacle state machine owns the motors. */
static bool update_obstacle_alignment(uint8_t sensor_state, int64_t now_us,
                                      bool ultrasonic_sample_ready)
{
    if (obstacle_state == OBSTACLE_IDLE) {
        if (finish_detection_wait_for_center ||
            finish_detection_delay_active || finish_detection_armed) {
            return false;
        }
        if (!ultrasonic_sample_ready) {
            return false;
        }

        const int distance_mm = latest_ultrasonic_distance_mm;

        if (distance_mm > 0 && distance_mm <= OBSTACLE_TRIGGER_MM) {
            ++obstacle_near_samples;
        } else {
            obstacle_near_samples = 0;
        }

        if (obstacle_near_samples >= OBSTACLE_CONFIRM_SAMPLES) {
            start_obstacle_alignment(distance_mm, now_us);
            return true;
        }
        return false;
    }

    const int64_t elapsed_ms =
        (now_us - obstacle_state_started_us) / 1000;

    switch (obstacle_state) {
    case OBSTACLE_ALIGN_TO_LINE:
        if (elapsed_ms >= OBSTACLE_ALIGN_TIMEOUT_MS) {
            fail_obstacle_action("line/distance alignment timeout");
            return true;
        }
        if (sensor_state != OBSTACLE_ALIGN_TARGET_STATE) {
            obstacle_align_stable_samples = 0;
            drive_obstacle_line_alignment(sensor_state);
            return true;
        }

        force_stop();
        if (ultrasonic_sample_ready &&
            obstacle_align_stable_samples < OBSTACLE_ALIGN_STABLE_SAMPLES) {
            ++obstacle_align_stable_samples;
        }
        if (obstacle_align_stable_samples >=
            OBSTACLE_ALIGN_STABLE_SAMPLES) {
            start_obstacle_left_strafe(now_us);
        }
        return true;

    case OBSTACLE_STRAFE_LEFT: {
        int progress[3];
        get_normalized_strafe_progress(obstacle_strafe_start_count,
                                       progress);
        const int progress_min =
            progress[MOTOR_A] < progress[MOTOR_B]
                ? (progress[MOTOR_A] < progress[MOTOR_D]
                       ? progress[MOTOR_A] : progress[MOTOR_D])
                : (progress[MOTOR_B] < progress[MOTOR_D]
                       ? progress[MOTOR_B] : progress[MOTOR_D]);
        const int progress_max =
            progress[MOTOR_A] > progress[MOTOR_B]
                ? (progress[MOTOR_A] > progress[MOTOR_D]
                       ? progress[MOTOR_A] : progress[MOTOR_D])
                : (progress[MOTOR_B] > progress[MOTOR_D]
                       ? progress[MOTOR_B] : progress[MOTOR_D]);
        const int progress_average =
            (progress[MOTOR_A] + progress[MOTOR_B] +
             progress[MOTOR_D]) / 3;
        const bool position_reached =
            progress_average >= OBSTACLE_LEFT_REAR_TARGET_COUNTS -
                                    OBSTACLE_LEFT_POSITION_TOLERANCE;
        const int progress_spread = progress_max - progress_min;

        if (position_reached) {
            if (progress_spread > OBSTACLE_LEFT_IMBALANCE_ABORT) {
                fail_obstacle_action("left strafe encoder imbalance");
                return true;
            }
            start_obstacle_strafe_brake(now_us, progress);
        } else if (elapsed_ms >= OBSTACLE_LEFT_STRAFE_TIMEOUT_MS) {
            fail_obstacle_action("left strafe timeout");
        } else {
            apply_obstacle_left_coordinated_control(progress, now_us);
        }
        return true;
    }

    case OBSTACLE_STRAFE_BRAKE:
        if (elapsed_ms >= OBSTACLE_STRAFE_BRAKE_MS) {
            int progress[3];
            get_normalized_strafe_progress(obstacle_strafe_start_count,
                                           progress);
            start_obstacle_strafe_settle(now_us, progress);
        } else {
            all_motors_brake();
        }
        return true;

    case OBSTACLE_STRAFE_SETTLE:
        if (elapsed_ms >= OBSTACLE_STRAFE_SETTLE_TIMEOUT_MS) {
            fail_obstacle_action("wheels did not settle after left strafe");
            return true;
        }
        if (update_obstacle_settle(now_us)) {
            start_obstacle_forward_position(now_us);
        }
        return true;

    case OBSTACLE_FORWARD_POSITION: {
        const int a_counts =
            abs(encoders[MOTOR_A].count - obstacle_forward_start_a_count);
        const int d_counts =
            abs(encoders[MOTOR_D].count - obstacle_forward_start_d_count);
        const int progress_average = (a_counts + d_counts) / 2;
        const int progress_spread = abs(a_counts - d_counts);
        if (progress_average >= OBSTACLE_FORWARD_TARGET_COUNTS -
                                    OBSTACLE_FORWARD_POSITION_TOLERANCE) {
            if (progress_spread > OBSTACLE_FORWARD_IMBALANCE_ABORT) {
                fail_obstacle_action("forward encoder imbalance");
                return true;
            }
            start_obstacle_forward_brake(now_us, a_counts, d_counts);
        } else if (elapsed_ms >= OBSTACLE_FORWARD_TIMEOUT_MS) {
            fail_obstacle_action("1.5-revolution forward timeout");
        } else {
            apply_obstacle_forward_position_control(a_counts, d_counts,
                                                    now_us);
        }
        return true;
    }

    case OBSTACLE_FORWARD_BRAKE:
        if (elapsed_ms >= OBSTACLE_FORWARD_BRAKE_MS) {
            start_obstacle_forward_settle(now_us);
        } else {
            all_motors_brake();
        }
        return true;

    case OBSTACLE_FORWARD_SETTLE:
        if (elapsed_ms >= OBSTACLE_STRAFE_SETTLE_TIMEOUT_MS) {
            fail_obstacle_action("wheels did not settle after forward");
            return true;
        }
        if (update_obstacle_settle(now_us)) {
            const int a_counts =
                abs(encoders[MOTOR_A].count - obstacle_forward_start_a_count);
            const int d_counts =
                abs(encoders[MOTOR_D].count - obstacle_forward_start_d_count);
            start_obstacle_right_search(now_us, a_counts, d_counts);
        }
        return true;

    case OBSTACLE_STRAFE_RIGHT_FIND_LINE: {
        int progress[3];
        get_normalized_strafe_progress(obstacle_right_start_count, progress);
        const int progress_min =
            progress[MOTOR_A] < progress[MOTOR_B]
                ? (progress[MOTOR_A] < progress[MOTOR_D]
                       ? progress[MOTOR_A] : progress[MOTOR_D])
                : (progress[MOTOR_B] < progress[MOTOR_D]
                       ? progress[MOTOR_B] : progress[MOTOR_D]);
        const int progress_max =
            progress[MOTOR_A] > progress[MOTOR_B]
                ? (progress[MOTOR_A] > progress[MOTOR_D]
                       ? progress[MOTOR_A] : progress[MOTOR_D])
                : (progress[MOTOR_B] > progress[MOTOR_D]
                       ? progress[MOTOR_B] : progress[MOTOR_D]);
        if (sensor_state != 0xF) {
            ++obstacle_reacquire_samples;
        } else {
            obstacle_reacquire_samples = 0;
        }
        if (obstacle_reacquire_samples >=
            OBSTACLE_REACQUIRE_CONFIRM_SAMPLES) {
            start_obstacle_right_brake(now_us, progress);
            return true;
        }
        if (elapsed_ms >= OBSTACLE_RIGHT_SEARCH_TIMEOUT_MS) {
            fail_obstacle_action("right strafe line-search timeout");
        } else if (progress_max - progress_min >
                       OBSTACLE_RIGHT_IMBALANCE_ABORT) {
            fail_obstacle_action("right strafe encoder imbalance");
        } else {
            apply_obstacle_right_coordinated_control(progress, now_us);
        }
        return true;
    }

    case OBSTACLE_STRAFE_RIGHT_BRAKE:
        if (elapsed_ms >= OBSTACLE_RIGHT_BRAKE_MS) {
            start_obstacle_right_settle(now_us);
        } else {
            all_motors_brake();
        }
        return true;

    case OBSTACLE_STRAFE_RIGHT_SETTLE:
        if (elapsed_ms >= OBSTACLE_STRAFE_SETTLE_TIMEOUT_MS) {
            fail_obstacle_action("wheels did not settle after right strafe");
            return true;
        }
        if (update_obstacle_settle(now_us)) {
            resume_line_follow_after_obstacle(now_us);
        }
        return true;

    default:
        fail_obstacle_action("invalid state");
        return true;
    }
}

static void set_line_follow_enabled(bool enabled)
{
    force_stop();
    cancel_obstacle_action();
    command_deadline_us = 0;
    line_follow_enabled = enabled;
    last_line_error = 0;
    line_lost_since_us = 0;
    line_lost_stop_reported = false;
    line_has_been_seen = false;
    sharp_turn = SHARP_TURN_NONE;
    sharp_turn_since_us = 0;
    line_lost_search_active = false;
    line_lost_search_turn = SHARP_TURN_NONE;
    line_pulse_epoch_us = esp_timer_get_time();
    finish_detection_wait_for_center = false;
    finish_detection_delay_active = false;
    finish_detection_armed = false;
    finish_marker_samples = 0;
    course_finished = false;
    reset_line_pid();
    reset_wheel_speed_pi(line_pulse_epoch_us);
}

static void finish_course(uint8_t sensor_state, int64_t now_us)
{
    force_stop();
    line_follow_enabled = false;
    command_deadline_us = 0;
    finish_detection_wait_for_center = false;
    finish_detection_delay_active = false;
    finish_detection_armed = false;
    finish_marker_samples = 0;
    course_finished = true;
    sharp_turn = SHARP_TURN_NONE;
    line_lost_search_active = false;
    reset_line_pid();
    reset_wheel_speed_pi(now_us);
    ESP_LOGW(TAG, "FINISH marker detected: IR=%d%d%d%d; STOP LATCHED",
             (sensor_state >> 3) & 1, (sensor_state >> 2) & 1,
             (sensor_state >> 1) & 1, sensor_state & 1);
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

    /* After returning from the obstacle bypass, first reacquire a centred
     * 1001 line, then follow it for another half wheel revolution. Only after
     * that grace distance may a black bar or explicit corner finish the run. */
    if (finish_detection_wait_for_center &&
        sensor_state == OBSTACLE_ALIGN_TARGET_STATE) {
        finish_detection_wait_for_center = false;
        finish_detection_delay_active = true;
        finish_delay_start_a_count = encoders[MOTOR_A].count;
        finish_delay_start_d_count = encoders[MOTOR_D].count;
        finish_marker_samples = 0;
        ESP_LOGW(TAG,
                 "FINISH grace distance started: A/D=%d counts",
                 FINISH_ARM_DELAY_COUNTS);
    }
    if (finish_detection_delay_active) {
        const int a_counts =
            abs(encoders[MOTOR_A].count - finish_delay_start_a_count);
        const int d_counts =
            abs(encoders[MOTOR_D].count - finish_delay_start_d_count);
        if (a_counts >= FINISH_ARM_DELAY_COUNTS &&
            d_counts >= FINISH_ARM_DELAY_COUNTS && black_count > 0) {
            finish_detection_delay_active = false;
            finish_detection_armed = true;
            finish_marker_samples = 0;
            ESP_LOGW(TAG,
                     "FINISH detection armed after A=%d D=%d counts",
                     a_counts, d_counts);
        }
    }
    if (finish_detection_armed) {
        const bool full_width_marker = sensor_state == 0x0;
        const bool turn_marker =
            (left_outer_black && left_inner_black) ||
            (right_inner_black && right_outer_black);
        if (full_width_marker || turn_marker) {
            if (finish_marker_samples < FINISH_CONFIRM_SAMPLES) {
                ++finish_marker_samples;
            }
            if (finish_marker_samples >= FINISH_CONFIRM_SAMPLES) {
                finish_course(sensor_state, now_us);
                return;
            }
        } else {
            finish_marker_samples = 0;
        }
    }

    /* Lost-line recovery is deliberately sticky. A single black channel can
     * be edge noise, so keep turning toward the last known line side until at
     * least two channels agree that the line has been found again. */
    if (line_lost_search_active) {
        const int64_t search_us = now_us - line_lost_since_us;
        if (black_count >= 2) {
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
            return;
        } else {
            drive_sharp_turn_pulsed(line_lost_search_turn, now_us);
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
            return;
        } else {
            drive_sharp_turn_pulsed(sharp_turn, now_us);
            return;
        }
    }

    /* Finish detection is intentionally disabled. Treat a transverse black
     * region as track and continue forward. */
    if (black_count == 4) {
        drive_line_duties_pulsed(LINE_BASE_DUTY,
                                 LINE_BASE_DUTY, now_us);
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
            drive_sharp_turn_pulsed(sharp_turn, now_us);
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
        return;
    }

    if (last_line_error == 0) {
        force_stop();
        reset_line_pid();
        if (!line_lost_stop_reported) {
            ESP_LOGW(TAG, "LINE LOST: no previous side direction; STOP");
            line_lost_stop_reported = true;
        }
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
        return;
    }

    if (motion == MOTION_STRAFE_LEFT || motion == MOTION_STRAFE_RIGHT) {
        drive_kiwi_strafe(motion == MOTION_STRAFE_LEFT);
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

static const char *control_state_name(void)
{
    if (obstacle_state == OBSTACLE_ALIGN_TO_LINE) {
        return "ALIGN_POSITION";
    }
    if (obstacle_state == OBSTACLE_STRAFE_LEFT) {
        return "OBSTACLE_STRAFE_LEFT";
    }
    if (obstacle_state == OBSTACLE_STRAFE_BRAKE) {
        return "OBSTACLE_STRAFE_BRAKE";
    }
    if (obstacle_state == OBSTACLE_STRAFE_SETTLE) {
        return "OBSTACLE_STRAFE_SETTLE";
    }
    if (obstacle_state == OBSTACLE_FORWARD_POSITION) {
        return "OBSTACLE_FORWARD_1_5_REVS";
    }
    if (obstacle_state == OBSTACLE_FORWARD_BRAKE) {
        return "OBSTACLE_FORWARD_BRAKE";
    }
    if (obstacle_state == OBSTACLE_FORWARD_SETTLE) {
        return "OBSTACLE_FORWARD_SETTLE";
    }
    if (obstacle_state == OBSTACLE_STRAFE_RIGHT_FIND_LINE) {
        return "OBSTACLE_STRAFE_RIGHT_FIND_LINE";
    }
    if (obstacle_state == OBSTACLE_STRAFE_RIGHT_BRAKE) {
        return "OBSTACLE_STRAFE_RIGHT_BRAKE";
    }
    if (obstacle_state == OBSTACLE_STRAFE_RIGHT_SETTLE) {
        return "OBSTACLE_STRAFE_RIGHT_SETTLE";
    }
    if (course_finished) {
        return "FINISHED";
    }
    if (finish_detection_wait_for_center) {
        return "REACQUIRE_LINE";
    }
    if (finish_detection_delay_active) {
        return "FINISH_DELAY";
    }
    if (finish_detection_armed) {
        return "FINISH_ARMED";
    }
    if (!line_follow_enabled) {
        return "MANUAL";
    }
    if (line_lost_search_active) {
        return line_lost_search_turn == SHARP_TURN_LEFT
                   ? "SEARCH_LEFT" : "SEARCH_RIGHT";
    }
    if (sharp_turn != SHARP_TURN_NONE) {
        return sharp_turn == SHARP_TURN_LEFT
                   ? "TURN_LEFT" : "TURN_RIGHT";
    }
    if (!line_has_been_seen && line_lost_stop_reported) {
        return "WAIT_LINE";
    }
    return "LINE_FOLLOW";
}

static void print_compact_status(uint8_t sensor_state)
{
    if (latest_ultrasonic_distance_mm >= 0) {
        printf("track=%d%d%d%d action=%s state=%s distance=%dmm\n",
               (sensor_state >> 3) & 1, (sensor_state >> 2) & 1,
               (sensor_state >> 1) & 1, sensor_state & 1,
               motion_name(current_motion), control_state_name(),
               latest_ultrasonic_distance_mm);
    } else {
        printf("track=%d%d%d%d action=%s state=%s distance=NA\n",
               (sensor_state >> 3) & 1, (sensor_state >> 2) & 1,
               (sensor_state >> 1) & 1, sensor_state & 1,
               motion_name(current_motion), control_state_name());
    }
    fflush(stdout);
}

static void update_tft_status(int64_t now_us)
{
    static int64_t previous_us;
    static int32_t previous_count[3];

    if (previous_us == 0) {
        previous_us = now_us;
        for (size_t wheel = 0; wheel < 3; ++wheel) {
            previous_count[wheel] = encoders[wheel].count;
        }
        tft_status_display_set(0, 0, 0, latest_ultrasonic_distance_mm);
        return;
    }

    const int64_t elapsed_us = now_us - previous_us;
    if (elapsed_us < (int64_t)DISPLAY_SPEED_SAMPLE_MS * 1000) {
        return;
    }

    int rpm[3];
    for (size_t wheel = 0; wheel < 3; ++wheel) {
        const int32_t count = encoders[wheel].count;
        int32_t delta = count - previous_count[wheel];
        if (delta < 0) {
            delta = -delta;
        }
        rpm[wheel] = (int)((int64_t)delta * 60 * 1000000 /
                           ((int64_t)ENCODER_COUNTS_PER_REV * elapsed_us));
        previous_count[wheel] = count;
    }
    previous_us = now_us;
    tft_status_display_set(rpm[MOTOR_A], rpm[MOTOR_B], rpm[MOTOR_D],
                           latest_ultrasonic_distance_mm);
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

    const esp_err_t display_error = tft_status_display_start();
    if (display_error != ESP_OK) {
        ESP_LOGE(TAG, "TFT unavailable; controls remain active: %s",
                 esp_err_to_name(display_error));
    }

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

    printf("READY: Q/E=strafe, X/SPACE=stop, F=line follow; status every %dms\n",
           STATUS_REPORT_INTERVAL_MS);
    fflush(stdout);

    uint8_t line_candidate = 0xff;
    uint8_t line_reported = 0xff;
    unsigned line_stable_count = 0;
    ultrasonic_next_sample_us = esp_timer_get_time();
    int64_t status_next_report_us = ultrasonic_next_sample_us;

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

        const int64_t now_us = esp_timer_get_time();
        if (!line_follow_enabled && current_motion != MOTION_STOP &&
            now_us >= command_deadline_us) {
            apply_motion(MOTION_STOP);
        }
        if (!line_follow_enabled) {
            update_kiwi_strafe(now_us);
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
        const bool ultrasonic_sample_ready = update_ultrasonic(now_us);
        update_tft_status(now_us);
        bool obstacle_owns_motors = false;
        if (line_follow_enabled && line_reported != 0xff) {
            obstacle_owns_motors =
                update_obstacle_alignment(line_reported, now_us,
                                          ultrasonic_sample_ready);
        }
        if (line_follow_enabled && !obstacle_owns_motors &&
            line_reported != 0xff) {
            update_line_follow(line_reported, now_us);
        }
        if (now_us >= status_next_report_us) {
            const uint8_t status_sensor_state =
                line_reported == 0xff ? line_now : line_reported;
            print_compact_status(status_sensor_state);
            status_next_report_us = now_us +
                (int64_t)STATUS_REPORT_INTERVAL_MS * 1000;
        }
        vTaskDelay(pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }
}
