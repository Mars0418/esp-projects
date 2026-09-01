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

/* The chassis needs a short high-duty launch to overcome static friction. */
#define FOLLOW_BASE_DUTY 195
#define FOLLOW_TURN_APPROACH_DUTY 175
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
#define TURN_HINT_ERROR 280
#define TURN_HINT_FAR_WEIGHT 40
#define TURN_HINT_CONFIRM_FRAMES 2
#define TURN_HINT_MAX_AGE_MS 1200
#define TURN_GEOMETRY_MIN_CONFIDENCE 50
#define TURN_GEOMETRY_STRONG_CONFIDENCE 75
#define TURN_GEOMETRY_MIN_ANGLE 25
#define TURN_REARM_COOLDOWN_MS 500
#define TURN_CLEAN_STRAIGHT_FRAMES 3
#define TURN_CLEAN_NEAR_DELTA_MAX 12
#define FOOT_LOST_CONFIRM_FRAMES 2
#define FOOT_LOST_STOP_FRAMES 3
#define TURN_COMMIT_FORWARD_MS 40
#define TURN_COMMIT_DUTY 165
#define PIVOT_INNER_REVERSE_DUTY 135
#define PIVOT_OUTER_FORWARD_DUTY 145
#define PIVOT_SLOW_INNER_REVERSE_DUTY 130
#define PIVOT_SLOW_OUTER_FORWARD_DUTY 140
#define PIVOT_FAST_PHASE_MS 160
#define PIVOT_MAX_MS 1800
#define PIVOT_REACQUIRE_FRAMES 2
#define FOLLOW_STATUS_INTERVAL_MS 2000
#define STEERING_SIGN -1

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
    int64_t commit_started_us;
    int64_t pivot_started_us;
    bool initialized;
    bool turn_armed;
    bool commit_active;
    bool pivot_active;
} pursuit_controller_t;

static const char *TAG = "CAMERA_PURSUIT";
static const motor_t s_motor_a = {
    MOTOR_A_PWM, MOTOR_A_IN1, MOTOR_A_IN2, MOTOR_A_CHANNEL};
static const motor_t s_motor_b = {
    MOTOR_B_PWM, MOTOR_B_IN1, MOTOR_B_IN2, MOTOR_B_CHANNEL};
static const motor_t s_motor_d = {
    MOTOR_D_PWM, MOTOR_D_IN1, MOTOR_D_IN2, MOTOR_D_CHANNEL};

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

static bool result_ready(const line_vision_result_t *result,
                         int64_t frame_us, int64_t now_us)
{
    return result->found && result->path_point_count >= 2 &&
           result->path_length_pixels >= 9 &&
           result->confidence >= FOLLOW_MIN_CONFIDENCE && frame_us > 0 &&
           now_us - frame_us <= (int64_t)FOLLOW_FRAME_TIMEOUT_MS * 1000;
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
                                 pursuit_controller_t *controller,
                                 int64_t now_us)
{
    /* Only the component connected to the vehicle-facing foot gate steers.
     * Far pixels record turn direction but never become the steering target. */
    const int base_duty = turn_hint_recent(controller, now_us)
                              ? FOLLOW_TURN_APPROACH_DUTY
                              : FOLLOW_BASE_DUTY;
    apply_tracking_control(result->foot_lateral_error, base_duty,
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
             "TURN READY direction=%d clean=%d foot=%d center=%d path=%d",
             controller->turn_direction,
             controller->clean_straight_frames,
             result->foot_pixel_count,
             result->foot_center_pixel_count,
             result->foot_path_length_pixels);
    return true;
}

static void start_turn_commit(pursuit_controller_t *controller,
                              int64_t now_us)
{
    controller->turn_armed = false;
    controller->commit_active = true;
    controller->commit_started_us = now_us;
    controller->a_command = TURN_COMMIT_DUTY;
    controller->d_command = TURN_COMMIT_DUTY;
    drive_wheels(controller->a_command, controller->d_command);
    ESP_LOGW(TAG,
             "TURN COMMIT direction=%d foot-exit straight=%dms duty=%d",
             controller->turn_direction, TURN_COMMIT_FORWARD_MS,
             TURN_COMMIT_DUTY);
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
    controller->commit_active = false;
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
    uint32_t processed_sequence = 0;
    int64_t last_report_us = 0;

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

        if (!s_enabled) {
            reset_controller(&controller);
        } else if (!frame_fresh) {
            s_enabled = false;
            stop_motors();
            reset_controller(&controller);
            ESP_LOGE(TAG, "STALE CAMERA FRAME: motors stopped");
        } else if (sequence != processed_sequence) {
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
                        apply_follow_control(&result, &controller, now_us);
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
            } else if (controller.commit_active) {
                const int64_t commit_ms =
                    (now_us - controller.commit_started_us) / 1000;
                if (commit_ms >= TURN_COMMIT_FORWARD_MS) {
                    start_pivot(&controller, now_us);
                } else {
                    controller.a_command = TURN_COMMIT_DUTY;
                    controller.d_command = TURN_COMMIT_DUTY;
                    drive_wheels(controller.a_command,
                                 controller.d_command);
                }
            } else if (foot_valid) {
                controller.foot_lost_frames = 0;
                if (controller.turn_armed) {
                    /* Once a direction is confirmed, retain it until the
                     * under-vehicle track actually ends. Distance to the
                     * corner varies, so a time-based expiry can discard the
                     * correct direction immediately before arrival. */
                    apply_tracking_control(result.foot_lateral_error,
                                           FOLLOW_TURN_APPROACH_DUTY,
                                           &controller);
                } else {
                    update_clean_straight(&result, &controller);
                    if (now_us < controller.turn_arm_ignore_until_us) {
                        clear_pending_turn_hint(&controller);
                        apply_follow_control(&result, &controller, now_us);
                    } else {
                        update_turn_hint(&result, &controller, now_us);
                    }
                    if (now_us >= controller.turn_arm_ignore_until_us &&
                        arm_turn(&result, &controller, now_us)) {
                        apply_tracking_control(result.foot_lateral_error,
                                               FOLLOW_TURN_APPROACH_DUTY,
                                               &controller);
                    } else if (now_us >=
                               controller.turn_arm_ignore_until_us) {
                        apply_follow_control(&result, &controller, now_us);
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
                    start_turn_commit(&controller, now_us);
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

        if (now_us - last_report_us >=
            (int64_t)FOLLOW_STATUS_INTERVAL_MS * 1000) {
            const int64_t age_ms = frame_us > 0 ? (now_us - frame_us) / 1000
                                                : -1;
            ESP_LOGI(TAG,
                     "PURSUIT en=%d state=%s vision=%d foot=%d centered=%d conf=%d fpix=%d cpix=%d fpath=%d flat=%d near=%d nlook=%d,%d farlook=%d,%d weight=%d far=%d path=%d lat=%d head=%d steer=%d clean=%d lost=%d hint=%d/%d filt=%d integ=%d corr=%d reacq=%d A=%d D=%d age=%lldms",
                     s_enabled,
                     controller.pivot_active ? "PIVOT" :
                         (controller.commit_active ? "COMMIT" :
                          (controller.turn_armed ? "READY" : "FOLLOW")),
                     line_valid, result.foot_track_valid,
                     result.foot_track_centered, result.confidence,
                     result.foot_pixel_count,
                     result.foot_center_pixel_count,
                     result.foot_path_length_pixels,
                     result.foot_lateral_error,
                     result.near_x, result.near_lookahead_x,
                     result.near_lookahead_y, result.lookahead_x,
                     result.lookahead_y, result.far_preview_weight,
                     result.far_x,
                     result.path_length_pixels, result.lateral_error,
                     result.heading_error, result.steering_error,
                     controller.clean_straight_frames,
                     controller.foot_lost_frames,
                     controller.turn_direction,
                     controller.hint_candidate_frames,
                     controller.filtered_error,
                     controller.integral_error / FOLLOW_INTEGRAL_DIVISOR,
                     controller.correction,
                     controller.reacquire_frames,
                     controller.a_command, controller.d_command,
                     (long long)age_ms);
            last_report_us = now_us;
        }
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
             "Foot-track controller: gate=center-half/12rows follow=%d approach=%d correction<=%d pivot=%d/%d reacquire=%dframes hint=%d@weight%d foot-lost=%dframes timeout=%dms",
             FOLLOW_BASE_DUTY, FOLLOW_TURN_APPROACH_DUTY,
             FOLLOW_CORRECTION_MAX, PIVOT_INNER_REVERSE_DUTY,
             PIVOT_OUTER_FORWARD_DUTY, PIVOT_REACQUIRE_FRAMES,
             TURN_HINT_ERROR,
             TURN_HINT_FAR_WEIGHT, FOOT_LOST_CONFIRM_FRAMES,
             PIVOT_MAX_MS);
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
