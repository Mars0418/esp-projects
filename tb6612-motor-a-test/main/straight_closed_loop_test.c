#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <limits.h>
#include <math.h>
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

#include "straight_closed_loop_config.h"

#define DRIVER_STBY GPIO_NUM_5
#define PWM_FREQUENCY_HZ 20000
#define PWM_MAX_DUTY 1023

typedef struct {
    const char *name;
    gpio_num_t pwm_pin;
    gpio_num_t in1_pin;
    gpio_num_t in2_pin;
    gpio_num_t phase_a_pin;
    gpio_num_t phase_b_pin;
    ledc_channel_t pwm_channel;
    volatile int32_t encoder_count;
    volatile uint8_t previous_state;
    int floor_pwm[2];
    bool floor_valid[2];
    int positive_motor_encoder_sign;
} wheel_t;

enum {
    WHEEL_A = 0,
    WHEEL_B = 1,
    WHEEL_D = 2,
    WHEEL_COUNT = 3,
};

static const char *TAG = "straight_closed_loop";

static wheel_t s_wheels[WHEEL_COUNT] = {
    {"A-right-front", GPIO_NUM_6, GPIO_NUM_15, GPIO_NUM_7,
     GPIO_NUM_16, GPIO_NUM_17, LEDC_CHANNEL_0, 0, 0, {0}, {false}, 0},
    {"B-rear", GPIO_NUM_11, GPIO_NUM_9, GPIO_NUM_10,
     GPIO_NUM_8, GPIO_NUM_18, LEDC_CHANNEL_1, 0, 0, {0}, {false}, 0},
    {"D-left-front", GPIO_NUM_40, GPIO_NUM_42, GPIO_NUM_41,
     GPIO_NUM_2, GPIO_NUM_1, LEDC_CHANNEL_2, 0, 0, {0}, {false}, 0},
};

static const int8_t s_quadrature_delta[16] = {
     0,  1, -1,  0,
    -1,  0,  0,  1,
     1,  0,  0, -1,
     0, -1,  1,  0,
};

static bool s_test_running;
static bool s_calibration_running;
static bool s_calibration_has_missing_limit;
static int64_t s_calibration_armed_until_us;
static int64_t s_test_started_us;
static int64_t s_next_control_us;
static int32_t s_previous_counts[WHEEL_COUNT];
static int32_t s_test_start_counts[WHEEL_COUNT];
static int s_speed_integral[WHEEL_COUNT];
static int s_applied_pwm[WHEEL_COUNT];
static int s_applied_direction[WHEEL_COUNT];
static int s_stall_intervals[2];
static int s_breakaway_pwm[WHEEL_COUNT];
static int s_min_effective_pwm[WHEEL_COUNT];

static int clamp_int(int value, int minimum, int maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

static int sign_int(int value)
{
    return (value > 0) - (value < 0);
}

static size_t direction_slot(int direction)
{
    return direction > 0 ? 1U : 0U;
}

static int fallback_floor(size_t wheel)
{
    static const int floors[WHEEL_COUNT] = {
        STRAIGHT_FALLBACK_FLOOR_A,
        STRAIGHT_FALLBACK_FLOOR_B,
        STRAIGHT_FALLBACK_FLOOR_D,
    };
    return floors[wheel];
}

static int wheel_floor(size_t wheel, int direction)
{
    const size_t slot = direction_slot(direction);
    if (s_wheels[wheel].floor_valid[slot]) {
        return s_wheels[wheel].floor_pwm[slot];
    }
    return fallback_floor(wheel);
}

static uint8_t encoder_state(const wheel_t *wheel)
{
    return ((uint8_t)gpio_get_level(wheel->phase_a_pin) << 1) |
           (uint8_t)gpio_get_level(wheel->phase_b_pin);
}

static void IRAM_ATTR encoder_isr(void *argument)
{
    wheel_t *wheel = (wheel_t *)argument;
    const uint8_t current = encoder_state(wheel);
    const uint8_t transition = (uint8_t)((wheel->previous_state << 2) |
                                         current);
    wheel->encoder_count += s_quadrature_delta[transition];
    wheel->previous_state = current;
}

static void snapshot_counts(int32_t counts[WHEEL_COUNT])
{
    for (size_t wheel = 0; wheel < WHEEL_COUNT; ++wheel) {
        counts[wheel] = s_wheels[wheel].encoder_count;
    }
}

static void configure_output_low(gpio_num_t pin)
{
    ESP_ERROR_CHECK(gpio_reset_pin(pin));
    ESP_ERROR_CHECK(gpio_set_direction(pin, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_level(pin, 0));
}

static void set_motor(size_t wheel, int direction, int pwm)
{
    pwm = clamp_int(pwm, 0, PWM_MAX_DUTY);
    if (direction == 0 || pwm == 0) {
        direction = 0;
        pwm = 0;
    }
    ESP_ERROR_CHECK(gpio_set_level(s_wheels[wheel].in1_pin,
                                   direction > 0));
    ESP_ERROR_CHECK(gpio_set_level(s_wheels[wheel].in2_pin,
                                   direction < 0));
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE,
                                  s_wheels[wheel].pwm_channel,
                                  (uint32_t)pwm));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE,
                                     s_wheels[wheel].pwm_channel));
    s_applied_direction[wheel] = direction;
    s_applied_pwm[wheel] = pwm;
}

static void safe_stop(void)
{
    ESP_ERROR_CHECK(gpio_set_level(DRIVER_STBY, 0));
    for (size_t wheel = 0; wheel < WHEEL_COUNT; ++wheel) {
        set_motor(wheel, 0, 0);
    }
    s_test_running = false;
}

/* TB6612 short brake. Keep this brief; normal stopped state uses STBY low. */
static void active_brake(void)
{
    for (size_t wheel = 0; wheel < WHEEL_COUNT; ++wheel) {
        ESP_ERROR_CHECK(gpio_set_level(s_wheels[wheel].in1_pin, 1));
        ESP_ERROR_CHECK(gpio_set_level(s_wheels[wheel].in2_pin, 1));
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE,
                                      s_wheels[wheel].pwm_channel,
                                      PWM_MAX_DUTY));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE,
                                         s_wheels[wheel].pwm_channel));
        s_applied_direction[wheel] = 0;
        s_applied_pwm[wheel] = PWM_MAX_DUTY;
    }
    ESP_ERROR_CHECK(gpio_set_level(DRIVER_STBY, 1));
    s_test_running = false;
}

static bool consume_stop_request(void)
{
    char input[16];
    const ssize_t count = read(STDIN_FILENO, input, sizeof(input));
    if (count <= 0) return false;
    for (ssize_t index = 0; index < count; ++index) {
        if (tolower((unsigned char)input[index]) == 'x' ||
            input[index] == ' ') {
            return true;
        }
    }
    return false;
}

static bool wait_or_abort(int duration_ms)
{
    const int64_t deadline_us = esp_timer_get_time() +
        (int64_t)duration_ms * 1000;
    while (esp_timer_get_time() < deadline_us) {
        if (consume_stop_request()) {
            safe_stop();
            ESP_LOGE(TAG, "EMERGENCY STOP");
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return true;
}

static bool calibration_trial(size_t wheel, int direction, int pwm,
                              int32_t *delta_out)
{
    safe_stop();
    if (!wait_or_abort(PWM_CALIBRATION_SETTLE_MS)) return false;

    const int32_t start = s_wheels[wheel].encoder_count;
    set_motor(wheel, direction, pwm);
    ESP_ERROR_CHECK(gpio_set_level(DRIVER_STBY, 1));
    if (!wait_or_abort(PWM_CALIBRATION_DRIVE_MS)) return false;
    const int32_t end = s_wheels[wheel].encoder_count;
    safe_stop();
    *delta_out = end - start;
    return true;
}

static bool calibrate_direction(size_t wheel, int direction)
{
    ESP_LOGW(TAG, "CALIBRATE wheel=%s direction=%+d",
             s_wheels[wheel].name, direction);
    for (int pwm = PWM_CALIBRATION_START;
         pwm <= PWM_CALIBRATION_MAX;
         pwm += PWM_CALIBRATION_STEP) {
        bool confirmed = true;
        int32_t signed_delta = 0;
        for (int trial = 0; trial < PWM_CALIBRATION_CONFIRM_TRIALS; ++trial) {
            int32_t delta = 0;
            if (!calibration_trial(wheel, direction, pwm, &delta)) {
                return false;
            }
            ESP_LOGI(TAG,
                     "CAL_DATA,wheel=%s,dir=%+d,pwm=%d,trial=%d,delta=%" PRId32,
                     s_wheels[wheel].name, direction, pwm, trial + 1, delta);
            if (abs(delta) < PWM_CALIBRATION_MIN_COUNTS) {
                confirmed = false;
                break;
            }
            signed_delta += delta;
        }
        if (!confirmed) continue;

        const size_t slot = direction_slot(direction);
        s_wheels[wheel].floor_pwm[slot] = pwm;
        s_wheels[wheel].floor_valid[slot] = true;
        if (direction > 0) {
            s_wheels[wheel].positive_motor_encoder_sign =
                sign_int((int)signed_delta);
        }
        ESP_LOGW(TAG,
                 "CAL_RESULT,wheel=%s,dir=%+d,floor=%d,encoder_delta=%" PRId32,
                 s_wheels[wheel].name, direction, pwm, signed_delta);
        return true;
    }

    ESP_LOGE(TAG, "CAL_RESULT,wheel=%s,dir=%+d,NOT_FOUND,max=%d",
             s_wheels[wheel].name, direction, PWM_CALIBRATION_MAX);
    s_calibration_has_missing_limit = true;
    return true;
}

static void print_calibration(void)
{
    for (size_t wheel = 0; wheel < WHEEL_COUNT; ++wheel) {
        ESP_LOGW(TAG,
                 "PWM_FLOOR,%s,negative=%d%s,positive=%d%s,pos_encoder_sign=%+d",
                 s_wheels[wheel].name,
                 wheel_floor(wheel, -1),
                 s_wheels[wheel].floor_valid[0] ? "" : "(fallback)",
                 wheel_floor(wheel, +1),
                 s_wheels[wheel].floor_valid[1] ? "" : "(fallback)",
                 s_wheels[wheel].positive_motor_encoder_sign);
    }
}

static void run_calibration(void)
{
    s_calibration_running = true;
    s_calibration_has_missing_limit = false;
    safe_stop();
    ESP_LOGW(TAG,
             "LOADED CALIBRATION START: vehicle must be on clear level floor");
    ESP_LOGW(TAG,
             "Each wheel will pulse separately and the chassis will move; X/SPACE=STOP");

    bool completed = true;
    for (size_t wheel = 0; wheel < WHEEL_COUNT && completed; ++wheel) {
        completed = calibrate_direction(wheel, -1) &&
                    calibrate_direction(wheel, +1);
    }
    safe_stop();
    s_calibration_running = false;
    print_calibration();
    ESP_LOGW(TAG, "CALIBRATION %s; SAFE STOP",
             !completed ? "CANCELLED" :
             (s_calibration_has_missing_limit ? "MISSING_LIMITS" :
                                                "COMPLETE"));
}

static int calculate_front_pwm(size_t wheel, int error,
                               int synchronization)
{
    s_speed_integral[wheel] = clamp_int(
        s_speed_integral[wheel] + error,
        -STRAIGHT_SPEED_INTEGRAL_LIMIT,
        STRAIGHT_SPEED_INTEGRAL_LIMIT);
    const int base = wheel_floor(wheel, -1) + STRAIGHT_PWM_ABOVE_FLOOR;
    const int requested = base + STRAIGHT_SPEED_KP * error +
        s_speed_integral[wheel] / STRAIGHT_SPEED_KI_DIV + synchronization;
    return clamp_int(requested, wheel_floor(wheel, -1), STRAIGHT_PWM_MAX);
}

static int calculate_motion_pwm(size_t wheel, int direction,
                                int target_speed, int measured_speed,
                                int synchronization)
{
    const int error = target_speed - measured_speed;
    s_speed_integral[wheel] = clamp_int(
        s_speed_integral[wheel] + error,
        -STRAIGHT_SPEED_INTEGRAL_LIMIT,
        STRAIGHT_SPEED_INTEGRAL_LIMIT);
    const int floor = wheel_floor(wheel, direction);
    const int requested = floor + STRAIGHT_PWM_ABOVE_FLOOR +
        STRAIGHT_SPEED_KP * error +
        s_speed_integral[wheel] / STRAIGHT_SPEED_KI_DIV + synchronization;
    return clamp_int(requested, floor, STRAIGHT_PWM_MAX);
}

static void calculate_b_hold_command(int encoder_delta,
                                     int *direction_out, int *pwm_out)
{
    *direction_out = 0;
    *pwm_out = 0;
    const int positive_sign = s_wheels[WHEEL_B].positive_motor_encoder_sign;
    if (positive_sign == 0 ||
        abs(encoder_delta) <= STRAIGHT_B_HOLD_DEADBAND_COUNTS) {
        return;
    }

    const int desired_encoder_sign = -sign_int(encoder_delta);
    const int direction = desired_encoder_sign * positive_sign;
    int pwm = wheel_floor(WHEEL_B, direction) +
        STRAIGHT_B_HOLD_PWM_MARGIN +
        (abs(encoder_delta) - STRAIGHT_B_HOLD_DEADBAND_COUNTS) *
            STRAIGHT_B_HOLD_KP;
    *direction_out = direction;
    *pwm_out = clamp_int(pwm, wheel_floor(WHEEL_B, direction),
                         STRAIGHT_B_HOLD_PWM_MAX);
}

static void finish_straight_test(const char *reason, int64_t now_us)
{
    int32_t counts[WHEEL_COUNT];
    snapshot_counts(counts);
    const int32_t total_a = abs(counts[WHEEL_A] -
                                s_test_start_counts[WHEEL_A]);
    const int32_t total_b = counts[WHEEL_B] - s_test_start_counts[WHEEL_B];
    const int32_t total_d = abs(counts[WHEEL_D] -
                                s_test_start_counts[WHEEL_D]);
    safe_stop();
    ESP_LOGW(TAG,
             "RESULT,%s,t_ms=%" PRId64 ",A=%" PRId32 ",B=%" PRId32
             ",D=%" PRId32 ",AD_difference=%" PRId32,
             reason, (now_us - s_test_started_us) / 1000,
             total_a, total_b, total_d, total_a - total_d);
    ESP_LOGW(TAG, "EFFECTIVE_PWM,A=%d,B=%d,D=%d",
             s_min_effective_pwm[WHEEL_A] == INT_MAX ? -1 :
                 s_min_effective_pwm[WHEEL_A],
             s_min_effective_pwm[WHEEL_B] == INT_MAX ? -1 :
                 s_min_effective_pwm[WHEEL_B],
             s_min_effective_pwm[WHEEL_D] == INT_MAX ? -1 :
                 s_min_effective_pwm[WHEEL_D]);
}

static void start_straight_test(void)
{
    safe_stop();
    snapshot_counts(s_previous_counts);
    snapshot_counts(s_test_start_counts);
    for (size_t wheel = 0; wheel < WHEEL_COUNT; ++wheel) {
        s_speed_integral[wheel] = 0;
        s_breakaway_pwm[wheel] = 0;
        s_min_effective_pwm[wheel] = INT_MAX;
    }
    s_stall_intervals[0] = 0;
    s_stall_intervals[1] = 0;
    s_test_started_us = esp_timer_get_time();
    s_next_control_us = s_test_started_us +
        (int64_t)STRAIGHT_CONTROL_INTERVAL_MS * 1000;
    s_test_running = true;

    set_motor(WHEEL_A, -1,
              wheel_floor(WHEEL_A, -1) + STRAIGHT_PWM_ABOVE_FLOOR);
    set_motor(WHEEL_B, 0, 0);
    set_motor(WHEEL_D, -1,
              wheel_floor(WHEEL_D, -1) + STRAIGHT_PWM_ABOVE_FLOOR);
    ESP_ERROR_CHECK(gpio_set_level(DRIVER_STBY, 1));
    ESP_LOGW(TAG,
             "STRAIGHT START,target=%d counts/%dms,duration=%dms; X/SPACE=STOP",
             STRAIGHT_TARGET_COUNTS_PER_INTERVAL,
             STRAIGHT_CONTROL_INTERVAL_MS,
             STRAIGHT_RUN_DURATION_MS);
    if (s_wheels[WHEEL_B].positive_motor_encoder_sign == 0) {
        ESP_LOGW(TAG,
                 "B lateral hold disabled until successful C calibration");
    }
}

static void update_effective_minimum(size_t wheel, int movement)
{
    if (movement < STRAIGHT_MOVING_COUNTS_MIN ||
        s_applied_pwm[wheel] <= 0) {
        return;
    }
    if (s_breakaway_pwm[wheel] == 0) {
        s_breakaway_pwm[wheel] = s_applied_pwm[wheel];
    }
    if (s_applied_pwm[wheel] < s_min_effective_pwm[wheel]) {
        s_min_effective_pwm[wheel] = s_applied_pwm[wheel];
    }
}

static void update_straight_control(int64_t now_us)
{
    int32_t counts[WHEEL_COUNT];
    int delta[WHEEL_COUNT];
    snapshot_counts(counts);
    for (size_t wheel = 0; wheel < WHEEL_COUNT; ++wheel) {
        delta[wheel] = (int)(counts[wheel] - s_previous_counts[wheel]);
        s_previous_counts[wheel] = counts[wheel];
    }

    const int speed_a = abs(delta[WHEEL_A]);
    const int speed_d = abs(delta[WHEEL_D]);
    update_effective_minimum(WHEEL_A, speed_a);
    update_effective_minimum(WHEEL_B, abs(delta[WHEEL_B]));
    update_effective_minimum(WHEEL_D, speed_d);

    s_stall_intervals[0] = speed_a < STRAIGHT_MOVING_COUNTS_MIN
        ? s_stall_intervals[0] + 1 : 0;
    s_stall_intervals[1] = speed_d < STRAIGHT_MOVING_COUNTS_MIN
        ? s_stall_intervals[1] + 1 : 0;
    if (s_stall_intervals[0] >= STRAIGHT_STALL_INTERVAL_LIMIT ||
        s_stall_intervals[1] >= STRAIGHT_STALL_INTERVAL_LIMIT) {
        finish_straight_test("STALL", now_us);
        return;
    }

    const int sync = clamp_int(
        (speed_d - speed_a) * STRAIGHT_SYNC_KP,
        -STRAIGHT_SYNC_CORRECTION_MAX,
        STRAIGHT_SYNC_CORRECTION_MAX);
    const int pwm_a = calculate_front_pwm(
        WHEEL_A, STRAIGHT_TARGET_COUNTS_PER_INTERVAL - speed_a, sync);
    const int pwm_d = calculate_front_pwm(
        WHEEL_D, STRAIGHT_TARGET_COUNTS_PER_INTERVAL - speed_d, -sync);

    int b_direction;
    int b_pwm;
    calculate_b_hold_command(delta[WHEEL_B], &b_direction, &b_pwm);

    set_motor(WHEEL_A, -1, pwm_a);
    set_motor(WHEEL_B, b_direction, b_pwm);
    set_motor(WHEEL_D, -1, pwm_d);
    ESP_ERROR_CHECK(gpio_set_level(DRIVER_STBY, 1));

    ESP_LOGI(TAG,
             "DATA,t=%" PRId64 ",A=%" PRId32 ",dA=%d,pwmA=%d"
             ",B=%" PRId32 ",dB=%d,dirB=%+d,pwmB=%d"
             ",D=%" PRId32 ",dD=%d,pwmD=%d,sync=%d",
             (now_us - s_test_started_us) / 1000,
             counts[WHEEL_A] - s_test_start_counts[WHEEL_A], speed_a, pwm_a,
             counts[WHEEL_B] - s_test_start_counts[WHEEL_B], delta[WHEEL_B],
             b_direction, b_pwm,
             counts[WHEEL_D] - s_test_start_counts[WHEEL_D], speed_d, pwm_d,
             sync);

    if (now_us - s_test_started_us >=
        (int64_t)STRAIGHT_RUN_DURATION_MS * 1000) {
        finish_straight_test("COMPLETE", now_us);
    }
}

static int forward_distance_target_counts(void)
{
    const float wheel_circumference =
        (float)M_PI * MOTION_WHEEL_DIAMETER_MM;
    return (int)lroundf(MOTION_FORWARD_DISTANCE_MM *
                        MOTION_FORWARD_WHEEL_PROJECTION /
                        wheel_circumference *
                        MOTION_ENCODER_COUNTS_PER_REV);
}

static int right_turn_target_counts(void)
{
    const float wheel_circumference =
        (float)M_PI * MOTION_WHEEL_DIAMETER_MM;
    const float turn_radians = MOTION_RIGHT_TURN_DEG * (float)M_PI / 180.0f;
    return (int)lroundf(MOTION_CENTER_TO_WHEEL_MM * turn_radians /
                        wheel_circumference *
                        MOTION_ENCODER_COUNTS_PER_REV);
}

static int position_target_speed(int normal_speed, int remaining,
                                 int slowdown_counts)
{
    if (remaining >= slowdown_counts) return normal_speed;
    const int scaled = normal_speed * remaining / slowdown_counts;
    return clamp_int(scaled, MOTION_MIN_SPEED_COUNTS_PER_INTERVAL,
                     normal_speed);
}

static bool brake_and_settle(void)
{
    active_brake();
    if (!wait_or_abort(MOTION_BRAKE_MS)) return false;
    safe_stop();
    return wait_or_abort(MOTION_SETTLE_MS);
}

static bool run_sequence_forward(int target_counts)
{
    int32_t start[WHEEL_COUNT];
    int32_t previous[WHEEL_COUNT];
    int stall_a = 0;
    int stall_d = 0;
    snapshot_counts(start);
    snapshot_counts(previous);
    for (size_t wheel = 0; wheel < WHEEL_COUNT; ++wheel) {
        s_speed_integral[wheel] = 0;
    }

    set_motor(WHEEL_A, -1,
              wheel_floor(WHEEL_A, -1) + STRAIGHT_PWM_ABOVE_FLOOR);
    set_motor(WHEEL_B, 0, 0);
    set_motor(WHEEL_D, -1,
              wheel_floor(WHEEL_D, -1) + STRAIGHT_PWM_ABOVE_FLOOR);
    ESP_ERROR_CHECK(gpio_set_level(DRIVER_STBY, 1));

    const int64_t started_us = esp_timer_get_time();
    int64_t next_update_us = started_us +
        (int64_t)STRAIGHT_CONTROL_INTERVAL_MS * 1000;
    ESP_LOGW(TAG, "SEQUENCE FORWARD start target=%d counts (%.0f mm)",
             target_counts, (double)MOTION_FORWARD_DISTANCE_MM);

    while (true) {
        if (consume_stop_request()) {
            safe_stop();
            ESP_LOGE(TAG, "SEQUENCE FORWARD cancelled by user");
            return false;
        }
        const int64_t now_us = esp_timer_get_time();
        if (now_us - started_us >=
            (int64_t)MOTION_FORWARD_TIMEOUT_MS * 1000) {
            safe_stop();
            ESP_LOGE(TAG, "SEQUENCE FORWARD timeout");
            return false;
        }
        if (now_us < next_update_us) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        next_update_us = now_us +
            (int64_t)STRAIGHT_CONTROL_INTERVAL_MS * 1000;

        int32_t counts[WHEEL_COUNT];
        snapshot_counts(counts);
        const int delta_a = abs((int)(counts[WHEEL_A] - previous[WHEEL_A]));
        const int delta_b = (int)(counts[WHEEL_B] - previous[WHEEL_B]);
        const int delta_d = abs((int)(counts[WHEEL_D] - previous[WHEEL_D]));
        for (size_t wheel = 0; wheel < WHEEL_COUNT; ++wheel) {
            previous[wheel] = counts[wheel];
        }
        const int progress_a = abs((int)(counts[WHEEL_A] - start[WHEEL_A]));
        const int progress_d = abs((int)(counts[WHEEL_D] - start[WHEEL_D]));
        const int progress_average = (progress_a + progress_d) / 2;
        const int progress_spread = abs(progress_a - progress_d);

        if (progress_average >= target_counts -
                                    MOTION_POSITION_TOLERANCE_COUNTS) {
            ESP_LOGW(TAG,
                     "SEQUENCE FORWARD reached A=%d D=%d average=%d; brake",
                     progress_a, progress_d, progress_average);
            return brake_and_settle();
        }
        if (progress_average > 100 &&
            progress_spread > MOTION_PROGRESS_SPREAD_ABORT) {
            safe_stop();
            ESP_LOGE(TAG, "SEQUENCE FORWARD imbalance A=%d D=%d",
                     progress_a, progress_d);
            return false;
        }

        stall_a = delta_a < STRAIGHT_MOVING_COUNTS_MIN ? stall_a + 1 : 0;
        stall_d = delta_d < STRAIGHT_MOVING_COUNTS_MIN ? stall_d + 1 : 0;
        if (stall_a >= STRAIGHT_STALL_INTERVAL_LIMIT ||
            stall_d >= STRAIGHT_STALL_INTERVAL_LIMIT) {
            safe_stop();
            ESP_LOGE(TAG, "SEQUENCE FORWARD stall dA=%d dD=%d", delta_a,
                     delta_d);
            return false;
        }

        const int remaining = target_counts - progress_average;
        const int target_speed = position_target_speed(
            STRAIGHT_TARGET_COUNTS_PER_INTERVAL, remaining,
            MOTION_FORWARD_SLOWDOWN_COUNTS);
        const int synchronization = clamp_int(
            (delta_d - delta_a) * STRAIGHT_SYNC_KP +
                (progress_d - progress_a) / MOTION_POSITION_SYNC_DIV,
            -MOTION_SYNC_CORRECTION_MAX, MOTION_SYNC_CORRECTION_MAX);
        const int pwm_a = calculate_motion_pwm(
            WHEEL_A, -1, target_speed, delta_a, synchronization);
        const int pwm_d = calculate_motion_pwm(
            WHEEL_D, -1, target_speed, delta_d, -synchronization);
        int b_direction;
        int b_pwm;
        calculate_b_hold_command(delta_b, &b_direction, &b_pwm);

        set_motor(WHEEL_A, -1, pwm_a);
        set_motor(WHEEL_B, b_direction, b_pwm);
        set_motor(WHEEL_D, -1, pwm_d);
        ESP_ERROR_CHECK(gpio_set_level(DRIVER_STBY, 1));
        ESP_LOGI(TAG,
                 "SEQ_FORWARD,t=%" PRId64 ",A=%d,dA=%d,pwmA=%d,B=%d,pwmB=%d,D=%d,dD=%d,pwmD=%d,targetSpeed=%d",
                 (now_us - started_us) / 1000, progress_a, delta_a, pwm_a,
                 delta_b, b_pwm, progress_d, delta_d, pwm_d, target_speed);
    }
}

static bool run_sequence_right_turn(int target_counts)
{
    static const int directions[WHEEL_COUNT] = {+1, -1, -1};
    int32_t start[WHEEL_COUNT];
    int32_t previous[WHEEL_COUNT];
    int stall[WHEEL_COUNT] = {0};
    snapshot_counts(start);
    snapshot_counts(previous);
    for (size_t wheel = 0; wheel < WHEEL_COUNT; ++wheel) {
        s_speed_integral[wheel] = 0;
        set_motor(wheel, directions[wheel],
                  wheel_floor(wheel, directions[wheel]) +
                      STRAIGHT_PWM_ABOVE_FLOOR);
    }
    ESP_ERROR_CHECK(gpio_set_level(DRIVER_STBY, 1));

    const int64_t started_us = esp_timer_get_time();
    int64_t next_update_us = started_us +
        (int64_t)STRAIGHT_CONTROL_INTERVAL_MS * 1000;
    ESP_LOGW(TAG,
             "SEQUENCE RIGHT TURN start target=%d counts (%.0f deg, radius=%.0f mm)",
             target_counts, (double)MOTION_RIGHT_TURN_DEG,
             (double)MOTION_CENTER_TO_WHEEL_MM);

    while (true) {
        if (consume_stop_request()) {
            safe_stop();
            ESP_LOGE(TAG, "SEQUENCE RIGHT TURN cancelled by user");
            return false;
        }
        const int64_t now_us = esp_timer_get_time();
        if (now_us - started_us >=
            (int64_t)MOTION_TURN_TIMEOUT_MS * 1000) {
            safe_stop();
            ESP_LOGE(TAG, "SEQUENCE RIGHT TURN timeout");
            return false;
        }
        if (now_us < next_update_us) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        next_update_us = now_us +
            (int64_t)STRAIGHT_CONTROL_INTERVAL_MS * 1000;

        int32_t counts[WHEEL_COUNT];
        int delta[WHEEL_COUNT];
        int progress[WHEEL_COUNT];
        int progress_min = INT_MAX;
        int progress_max = 0;
        int progress_sum = 0;
        snapshot_counts(counts);
        for (size_t wheel = 0; wheel < WHEEL_COUNT; ++wheel) {
            delta[wheel] = abs((int)(counts[wheel] - previous[wheel]));
            progress[wheel] = abs((int)(counts[wheel] - start[wheel]));
            previous[wheel] = counts[wheel];
            if (progress[wheel] < progress_min) progress_min = progress[wheel];
            if (progress[wheel] > progress_max) progress_max = progress[wheel];
            progress_sum += progress[wheel];
            stall[wheel] = delta[wheel] < STRAIGHT_MOVING_COUNTS_MIN
                ? stall[wheel] + 1 : 0;
        }
        const int progress_average = progress_sum / WHEEL_COUNT;
        if (progress_average >= target_counts -
                                    MOTION_POSITION_TOLERANCE_COUNTS) {
            ESP_LOGW(TAG,
                     "SEQUENCE RIGHT TURN reached A=%d B=%d D=%d average=%d; brake",
                     progress[WHEEL_A], progress[WHEEL_B], progress[WHEEL_D],
                     progress_average);
            return brake_and_settle();
        }
        if (progress_average > 80 &&
            progress_max - progress_min > MOTION_PROGRESS_SPREAD_ABORT) {
            safe_stop();
            ESP_LOGE(TAG, "SEQUENCE RIGHT TURN imbalance A=%d B=%d D=%d",
                     progress[WHEEL_A], progress[WHEEL_B], progress[WHEEL_D]);
            return false;
        }
        for (size_t wheel = 0; wheel < WHEEL_COUNT; ++wheel) {
            if (stall[wheel] >= STRAIGHT_STALL_INTERVAL_LIMIT) {
                safe_stop();
                ESP_LOGE(TAG, "SEQUENCE RIGHT TURN stall wheel=%s",
                         s_wheels[wheel].name);
                return false;
            }
        }

        const int remaining = target_counts - progress_average;
        const int target_speed = position_target_speed(
            MOTION_TURN_SPEED_COUNTS_PER_INTERVAL, remaining,
            MOTION_TURN_SLOWDOWN_COUNTS);
        int pwm[WHEEL_COUNT];
        for (size_t wheel = 0; wheel < WHEEL_COUNT; ++wheel) {
            const int synchronization = clamp_int(
                (progress_average - progress[wheel]) /
                    MOTION_POSITION_SYNC_DIV,
                -MOTION_SYNC_CORRECTION_MAX,
                MOTION_SYNC_CORRECTION_MAX);
            pwm[wheel] = calculate_motion_pwm(
                wheel, directions[wheel], target_speed, delta[wheel],
                synchronization);
            set_motor(wheel, directions[wheel], pwm[wheel]);
        }
        ESP_ERROR_CHECK(gpio_set_level(DRIVER_STBY, 1));
        ESP_LOGI(TAG,
                 "SEQ_TURN,t=%" PRId64 ",A=%d,dA=%d,pwmA=%d,B=%d,dB=%d,pwmB=%d,D=%d,dD=%d,pwmD=%d,targetSpeed=%d",
                 (now_us - started_us) / 1000,
                 progress[WHEEL_A], delta[WHEEL_A], pwm[WHEEL_A],
                 progress[WHEEL_B], delta[WHEEL_B], pwm[WHEEL_B],
                 progress[WHEEL_D], delta[WHEEL_D], pwm[WHEEL_D],
                 target_speed);
    }
}

static void run_distance_turn_sequence(void)
{
    safe_stop();
    const int forward_target = forward_distance_target_counts();
    const int turn_target = right_turn_target_counts();
    ESP_LOGW(TAG,
             "SEQUENCE READY: forward target=%d, right-turn target=%d; X/SPACE=STOP",
             forward_target, turn_target);
    const bool forward_ok = run_sequence_forward(forward_target);
    const bool turn_ok = forward_ok && run_sequence_right_turn(turn_target);
    safe_stop();
    ESP_LOGW(TAG, "SEQUENCE %s; SAFE STOP",
             forward_ok && turn_ok ? "COMPLETE" : "ABORTED");
}

static void hardware_init(void)
{
    configure_output_low(DRIVER_STBY);
    for (size_t wheel = 0; wheel < WHEEL_COUNT; ++wheel) {
        configure_output_low(s_wheels[wheel].in1_pin);
        configure_output_low(s_wheels[wheel].in2_pin);
    }

    const ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = PWM_FREQUENCY_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    for (size_t wheel = 0; wheel < WHEEL_COUNT; ++wheel) {
        const ledc_channel_config_t channel = {
            .gpio_num = s_wheels[wheel].pwm_pin,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = s_wheels[wheel].pwm_channel,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = LEDC_TIMER_0,
            .duty = 0,
            .hpoint = 0,
            .flags.output_invert = 0,
        };
        ESP_ERROR_CHECK(ledc_channel_config(&channel));
    }

    uint64_t encoder_mask = 0;
    for (size_t wheel = 0; wheel < WHEEL_COUNT; ++wheel) {
        encoder_mask |= 1ULL << s_wheels[wheel].phase_a_pin;
        encoder_mask |= 1ULL << s_wheels[wheel].phase_b_pin;
    }
    const gpio_config_t encoder_config = {
        .pin_bit_mask = encoder_mask,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&encoder_config));
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    for (size_t wheel = 0; wheel < WHEEL_COUNT; ++wheel) {
        s_wheels[wheel].previous_state = encoder_state(&s_wheels[wheel]);
        ESP_ERROR_CHECK(gpio_isr_handler_add(
            s_wheels[wheel].phase_a_pin, encoder_isr, &s_wheels[wheel]));
        ESP_ERROR_CHECK(gpio_isr_handler_add(
            s_wheels[wheel].phase_b_pin, encoder_isr, &s_wheels[wheel]));
    }
    safe_stop();
}

static void handle_idle_input(void)
{
    char input[16];
    const ssize_t count = read(STDIN_FILENO, input, sizeof(input));
    if (count < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
            safe_stop();
            ESP_LOGE(TAG, "USB input error=%d; SAFE STOP", errno);
        }
        return;
    }
    for (ssize_t index = 0; index < count; ++index) {
        const char key = (char)tolower((unsigned char)input[index]);
        if (key == 'x' || key == ' ') {
            safe_stop();
            ESP_LOGW(TAG, "SAFE STOP");
        } else if (key == 'p') {
            print_calibration();
        } else if (key == 'c') {
            const int64_t now_us = esp_timer_get_time();
            if (now_us <= s_calibration_armed_until_us) {
                s_calibration_armed_until_us = 0;
                run_calibration();
            } else {
                s_calibration_armed_until_us = now_us +
                    (int64_t)PWM_CALIBRATION_ARM_TIMEOUT_MS * 1000;
                ESP_LOGW(TAG,
                         "CALIBRATION ARMED: clear floor area, then press C again within %d ms",
                         PWM_CALIBRATION_ARM_TIMEOUT_MS);
            }
        } else if (key == 'w' && !s_calibration_running) {
            start_straight_test();
        } else if (key == 'g' && !s_calibration_running) {
            run_distance_turn_sequence();
        }
    }
}

void app_main(void)
{
    hardware_init();

    usb_serial_jtag_driver_config_t usb_config =
        USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    usb_config.rx_buffer_size = 256;
    usb_config.tx_buffer_size = 2048;
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_config));
    usb_serial_jtag_vfs_use_driver();

    const int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    ESP_ERROR_CHECK(flags < 0 ? ESP_FAIL : ESP_OK);
    ESP_ERROR_CHECK(fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK) < 0
                        ? ESP_FAIL : ESP_OK);

    ESP_LOGW(TAG, "STRAIGHT CLOSED-LOOP TEST READY; motors are stopped");
    ESP_LOGI(TAG, "C,C=loaded calibration; W=8s forward; G=38cm+right120; P=limits; X/SPACE=stop");
    ESP_LOGI(TAG, "Forward kinematics: A/D track equal speed; B target is zero lateral speed");
    print_calibration();

    while (true) {
        if (s_test_running) {
            if (consume_stop_request()) {
                finish_straight_test("USER_STOP", esp_timer_get_time());
            } else {
                const int64_t now_us = esp_timer_get_time();
                if (now_us >= s_next_control_us) {
                    update_straight_control(now_us);
                    s_next_control_us = now_us +
                        (int64_t)STRAIGHT_CONTROL_INTERVAL_MS * 1000;
                }
            }
        } else if (!s_calibration_running) {
            handle_idle_input();
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
