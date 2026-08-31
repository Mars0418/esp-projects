#include "camera_line_follow.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
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

#define ENCODER_A_PHASE_A GPIO_NUM_16
#define ENCODER_A_PHASE_B GPIO_NUM_17
#define ENCODER_D_PHASE_A GPIO_NUM_2
#define ENCODER_D_PHASE_B GPIO_NUM_1
#define ENCODER_COUNTS_PER_REV 406

#define PWM_MAX_DUTY 1023
#define FOLLOW_BASE_DUTY 160
#define FOLLOW_LOW_CONFIDENCE_DUTY 150
#define FOLLOW_INNER_MIN_DUTY 140
#define FOLLOW_MAX_DUTY 190
#define FOLLOW_CORRECTION_MAX 20
#define FOLLOW_CORRECTION_STEP 5
#define FOLLOW_ERROR_DEADBAND 45
#define FOLLOW_FRAME_TIMEOUT_MS 600
#define FOLLOW_MIN_CONFIDENCE 30
#define FOLLOW_STATUS_INTERVAL_MS 2000
#define LINE_EVENT_CONFIRM_FRAMES 3
#define STEERING_SIGN -1

#define TURN_CONFIRM_FRAMES 3
#define TURN_CANDIDATE_X_TOLERANCE 24
#define TURN_CANDIDATE_Y_TOLERANCE 20
#define TURN_TRACK_Y_TOLERANCE 18
#define TURN_LOST_TRIGGER_MARGIN 18
#define TURN_LOST_GRACE_MS 300
/* Raw ROI y=12..57 runs from near to far; lower y is closer to the car. */
#define TURN_TRIGGER_Y LINE_VISION_TURN_TRIGGER_Y
#define TURN_APPROACH_DUTY 160
#define TURN_APPROACH_TIMEOUT_MS 2500
#define TURN_ADVANCE_DUTY 150
#define TURN_ADVANCE_COUNTS (ENCODER_COUNTS_PER_REV * 4)
#define TURN_ADVANCE_TIMEOUT_MS 10000
#define TURN_SPIN_DUTY 95
#define PIVOT_MIN_TIME_MS 250
#define PIVOT_MAX_TIME_MS 1200
#define REACQUIRE_SPIN_DUTY 95
#define REACQUIRE_STABLE_FRAMES 3
#define REACQUIRE_STEERING_LIMIT 250

typedef enum {
    FOLLOW_STATE_TRACK,
    FOLLOW_STATE_APPROACH_TURN,
    FOLLOW_STATE_ADVANCE_TO_TURN,
    FOLLOW_STATE_PIVOT,
    FOLLOW_STATE_REACQUIRE,
} follow_state_t;

typedef struct {
    gpio_num_t pwm;
    gpio_num_t in1;
    gpio_num_t in2;
    ledc_channel_t channel;
} motor_t;

typedef struct {
    gpio_num_t phase_a;
    gpio_num_t phase_b;
    volatile int32_t count;
    volatile uint8_t previous_state;
} encoder_t;

static const char *TAG = "CAMERA_FOLLOW";
static const motor_t s_motor_a = {
    MOTOR_A_PWM, MOTOR_A_IN1, MOTOR_A_IN2, MOTOR_A_CHANNEL};
static const motor_t s_motor_b = {
    MOTOR_B_PWM, MOTOR_B_IN1, MOTOR_B_IN2, MOTOR_B_CHANNEL};
static const motor_t s_motor_d = {
    MOTOR_D_PWM, MOTOR_D_IN1, MOTOR_D_IN2, MOTOR_D_CHANNEL};
static encoder_t s_encoder_a = {
    ENCODER_A_PHASE_A, ENCODER_A_PHASE_B, 0, 0};
static encoder_t s_encoder_d = {
    ENCODER_D_PHASE_A, ENCODER_D_PHASE_B, 0, 0};
static portMUX_TYPE s_result_lock = portMUX_INITIALIZER_UNLOCKED;
static line_vision_result_t s_latest_result;
static int64_t s_latest_frame_us;
static uint32_t s_result_sequence;
static bool s_enabled;
static volatile bool s_debug_enabled;
static char s_uart_line[64];
static size_t s_uart_line_length;

static const int8_t s_quadrature_delta[16] = {
     0,  1, -1,  0,
    -1,  0,  0,  1,
     1,  0,  0, -1,
     0, -1,  1,  0,
};

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

static void drive_forward(int a_duty, int d_duty)
{
    motor_prepare(&s_motor_a, -1, a_duty);
    motor_prepare(&s_motor_b, 0, 0);
    motor_prepare(&s_motor_d, -1, d_duty);
    ESP_ERROR_CHECK(gpio_set_level(DRIVER_STBY, 1));
}

static void drive_three_wheel_spin(int physical_direction, int duty)
{
    const int a_direction = physical_direction > 0 ? 1 : -1;
    const int d_direction = -a_direction;
    const int b_direction = d_direction;

    /* A/D provide differential yaw; B reinforces that yaw on the kiwi base. */
    motor_prepare(&s_motor_a, a_direction, duty);
    motor_prepare(&s_motor_b, b_direction, duty);
    motor_prepare(&s_motor_d, d_direction, duty);
    ESP_ERROR_CHECK(gpio_set_level(DRIVER_STBY, 1));
}

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
        .pin_bit_mask = (1ULL << ENCODER_A_PHASE_A) |
                        (1ULL << ENCODER_A_PHASE_B) |
                        (1ULL << ENCODER_D_PHASE_A) |
                        (1ULL << ENCODER_D_PHASE_B),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&config), TAG,
                        "encoder GPIO config failed");

    const esp_err_t isr_error = gpio_install_isr_service(0);
    if (isr_error != ESP_OK && isr_error != ESP_ERR_INVALID_STATE) {
        return isr_error;
    }

    encoder_t *encoders[] = {&s_encoder_a, &s_encoder_d};
    for (size_t index = 0; index < 2; ++index) {
        encoders[index]->count = 0;
        encoders[index]->previous_state = encoder_read_state(encoders[index]);
        ESP_RETURN_ON_ERROR(
            gpio_isr_handler_add(encoders[index]->phase_a,
                                 encoder_gpio_isr, encoders[index]),
            TAG, "encoder phase A ISR failed");
        ESP_RETURN_ON_ERROR(
            gpio_isr_handler_add(encoders[index]->phase_b,
                                 encoder_gpio_isr, encoders[index]),
            TAG, "encoder phase B ISR failed");
    }
    return ESP_OK;
}

static int32_t encoder_distance(int32_t current, int32_t start)
{
    int32_t distance = current - start;
    if (distance < 0) {
        distance = -distance;
    }
    return distance;
}

static void signed_spin_duties(int physical_direction, int duty,
                               int *a_duty, int *b_duty, int *d_duty)
{
    if (physical_direction > 0) {
        *a_duty = duty;
        *b_duty = -duty;
        *d_duty = -duty;
    } else {
        *a_duty = -duty;
        *b_duty = duty;
        *d_duty = duty;
    }
}

static const char *state_name(follow_state_t state)
{
    switch (state) {
    case FOLLOW_STATE_TRACK: return "TRACK";
    case FOLLOW_STATE_APPROACH_TURN: return "APPROACH";
    case FOLLOW_STATE_ADVANCE_TO_TURN: return "ADVANCE";
    case FOLLOW_STATE_PIVOT: return "PIVOT";
    case FOLLOW_STATE_REACQUIRE: return "REACQUIRE";
    default: return "UNKNOWN";
    }
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
    } else if (sscanf(s_uart_line, "DEBUG,%d", &enabled) == 1) {
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
        ESP_LOGI(TAG, "RGB_THRESHOLDS r=%u g=%u b=%u debug=%d",
                 thresholds.red, thresholds.green, thresholds.blue,
                 s_debug_enabled);
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
            line_vision_result_t result;
            int64_t frame_us;
            portENTER_CRITICAL(&s_result_lock);
            result = s_latest_result;
            frame_us = s_latest_frame_us;
            portEXIT_CRITICAL(&s_result_lock);
            const bool ready = result.found &&
                result.confidence >= FOLLOW_MIN_CONFIDENCE &&
                frame_us > 0 &&
                esp_timer_get_time() - frame_us <=
                    (int64_t)FOLLOW_FRAME_TIMEOUT_MS * 1000;
            if (ready) {
                s_enabled = true;
                ESP_LOGW(TAG, "CAMERA LINE FOLLOW ENABLED");
            } else {
                s_enabled = false;
                stop_motors();
                ESP_LOGW(TAG, "START REFUSED: no fresh, confident line");
            }
        } else if (value == 'x' || value == 'X' || value == ' ') {
            s_enabled = false;
            stop_motors();
            ESP_LOGW(TAG, "CAMERA LINE FOLLOW STOPPED");
        } else if (value == '\r' || value == '\n') {
            if (s_uart_line_length > 0) {
                process_uart_line();
            }
        } else if (value >= 32 && value <= 126) {
            if (s_uart_line_length + 1 < sizeof(s_uart_line)) {
                s_uart_line[s_uart_line_length++] = (char)value;
            } else {
                s_uart_line_length = 0;
            }
        }
    }
}

static void start_advance(follow_state_t *state, int64_t *state_started_us,
                          int32_t *start_a_count, int32_t *start_d_count,
                          int32_t *a_counts, int32_t *d_counts,
                          int corner_y, int instant_direction,
                          int locked_direction, const char *reason)
{
    *state = FOLLOW_STATE_ADVANCE_TO_TURN;
    *state_started_us = esp_timer_get_time();
    *start_a_count = s_encoder_a.count;
    *start_d_count = s_encoder_d.count;
    *a_counts = 0;
    *d_counts = 0;
    ESP_LOGW(TAG,
             "TURN TRIGGER reason=%s y=%d instant=%d locked=%d; ADVANCE target=%d",
             reason, corner_y, instant_direction, locked_direction,
             TURN_ADVANCE_COUNTS);
}

static void control_task(void *argument)
{
    (void)argument;
    uint32_t processed_sequence = 0;
    int filtered_error = 0;
    bool filter_initialized = false;
    int current_base = 0;
    int current_correction = 0;
    int current_a_duty = 0;
    int current_b_duty = 0;
    int current_d_duty = 0;
    follow_state_t state = FOLLOW_STATE_TRACK;
    int turn_candidate_direction = 0;
    int turn_candidate_frames = 0;
    int turn_candidate_x = -1;
    int turn_candidate_y = -1;
    int latched_turn_direction = 0;
    int last_corner_x = -1;
    int last_corner_y = -1;
    int64_t last_corner_seen_us = 0;
    int stable_frames = 0;
    int64_t state_started_us = 0;
    int32_t advance_start_a_count = 0;
    int32_t advance_start_d_count = 0;
    int32_t advance_a_counts = 0;
    int32_t advance_d_counts = 0;
    bool was_enabled = false;
    bool line_state_known = false;
    bool last_line_valid = false;
    bool line_candidate_valid = false;
    int line_candidate_frames = 0;
    int64_t last_report_us = 0;
    stop_motors();

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
        const bool line_valid = result.found &&
                                 result.confidence >= FOLLOW_MIN_CONFIDENCE;
        const bool result_updated = sequence != processed_sequence;

        if (s_enabled && !was_enabled) {
            state = FOLLOW_STATE_TRACK;
            state_started_us = now_us;
            turn_candidate_direction = 0;
            turn_candidate_frames = 0;
            turn_candidate_x = -1;
            turn_candidate_y = -1;
            latched_turn_direction = 0;
            last_corner_x = -1;
            last_corner_y = -1;
            last_corner_seen_us = 0;
            stable_frames = 0;
            filtered_error = 0;
            filter_initialized = false;
            advance_start_a_count = s_encoder_a.count;
            advance_start_d_count = s_encoder_d.count;
            advance_a_counts = 0;
            advance_d_counts = 0;
        }
        was_enabled = s_enabled;

        if (!s_enabled || !frame_fresh) {
            stop_motors();
            if (!frame_fresh && s_enabled) {
                s_enabled = false;
                was_enabled = false;
                ESP_LOGE(TAG, "CAMERA FRAME STALE: safety stop");
            }
            filtered_error = 0;
            filter_initialized = false;
            current_base = 0;
            current_correction = 0;
            current_a_duty = 0;
            current_b_duty = 0;
            current_d_duty = 0;
            state = FOLLOW_STATE_TRACK;
        } else if (result_updated) {
            const bool corner_valid = result.big_turn &&
                                      result.turn_confidence >= 50 &&
                                      result.turn_direction != 0;
            const bool corner_position_valid =
                result.corner_x >= 0 && result.corner_y >= 0;
            if (state == FOLLOW_STATE_TRACK) {
                if (corner_valid) {
                    const bool same_candidate =
                        turn_candidate_frames > 0 &&
                        turn_candidate_direction == result.turn_direction &&
                        abs(turn_candidate_x - result.corner_x) <=
                            TURN_CANDIDATE_X_TOLERANCE &&
                        abs(turn_candidate_y - result.corner_y) <=
                            TURN_CANDIDATE_Y_TOLERANCE;
                    if (same_candidate) {
                        turn_candidate_frames++;
                    } else {
                        turn_candidate_direction = result.turn_direction;
                        turn_candidate_frames = 1;
                    }
                    turn_candidate_x = result.corner_x;
                    turn_candidate_y = result.corner_y;
                    if (turn_candidate_frames >= TURN_CONFIRM_FRAMES) {
                        latched_turn_direction =
                            STEERING_SIGN * turn_candidate_direction;
                        state_started_us = now_us;
                        last_corner_x = result.corner_x;
                        last_corner_y = result.corner_y;
                        last_corner_seen_us = now_us;
                        ESP_LOGW(TAG,
                                 "TURN CONFIRMED visual=%d physical=%d frames=%d angle=%d corner=%d,%d",
                                 turn_candidate_direction,
                                 latched_turn_direction,
                                 turn_candidate_frames,
                                 result.turn_angle_deg,
                                 result.corner_x, result.corner_y);
                        if (result.corner_y <= TURN_TRIGGER_Y) {
                            start_advance(
                                &state, &state_started_us,
                                &advance_start_a_count,
                                &advance_start_d_count,
                                &advance_a_counts, &advance_d_counts,
                                result.corner_y,
                                STEERING_SIGN * result.turn_direction,
                                latched_turn_direction, "visible");
                        } else {
                            state = FOLLOW_STATE_APPROACH_TURN;
                        }
                    }
                } else {
                    turn_candidate_frames = 0;
                    turn_candidate_direction = 0;
                    turn_candidate_x = -1;
                    turn_candidate_y = -1;
                }
            } else if (state == FOLLOW_STATE_APPROACH_TURN) {
                /* Follow the already confirmed near corner. A later corner
                 * is farther down the path and must not replace this one. */
                const bool tracked_corner =
                    corner_valid && corner_position_valid &&
                    (last_corner_y < 0 ||
                     result.corner_y <=
                         last_corner_y + TURN_TRACK_Y_TOLERANCE);
                if (tracked_corner) {
                    last_corner_x = result.corner_x;
                    last_corner_y = result.corner_y;
                    last_corner_seen_us = now_us;
                }

                if (tracked_corner && result.corner_y <= TURN_TRIGGER_Y) {
                    start_advance(
                        &state, &state_started_us,
                        &advance_start_a_count, &advance_start_d_count,
                        &advance_a_counts, &advance_d_counts,
                        result.corner_y,
                        STEERING_SIGN * result.turn_direction,
                        latched_turn_direction, "visible");
                } else if (last_corner_seen_us > 0 &&
                           last_corner_y <=
                               TURN_TRIGGER_Y + TURN_LOST_TRIGGER_MARGIN &&
                           now_us - last_corner_seen_us >=
                               (int64_t)TURN_LOST_GRACE_MS * 1000) {
                    start_advance(
                        &state, &state_started_us,
                        &advance_start_a_count, &advance_start_d_count,
                        &advance_a_counts, &advance_d_counts,
                        last_corner_y, 0, latched_turn_direction,
                        "lost-near-line");
                } else if (now_us - state_started_us >=
                           (int64_t)TURN_APPROACH_TIMEOUT_MS * 1000) {
                    state = FOLLOW_STATE_TRACK;
                    state_started_us = now_us;
                    turn_candidate_direction = 0;
                    turn_candidate_frames = 0;
                    turn_candidate_x = -1;
                    turn_candidate_y = -1;
                    latched_turn_direction = 0;
                    last_corner_x = -1;
                    last_corner_y = -1;
                    last_corner_seen_us = 0;
                    ESP_LOGW(TAG,
                             "APPROACH TIMEOUT: corner did not reach y<=%d; resume TRACK",
                             TURN_TRIGGER_Y);
                }
            } else if (state == FOLLOW_STATE_REACQUIRE) {
                const int previous_stable_frames = stable_frames;
                const bool aligned =
                    line_valid &&
                    abs(result.steering_error) <= REACQUIRE_STEERING_LIMIT;
                stable_frames = aligned ? stable_frames + 1 : 0;
                if (aligned && previous_stable_frames == 0) {
                    ESP_LOGW(TAG,
                             "REACQUIRE CANDIDATE: pause to confirm steer=%d big=%d",
                             result.steering_error, result.big_turn);
                } else if (!aligned && previous_stable_frames > 0) {
                    ESP_LOGW(TAG,
                             "REACQUIRE CANDIDATE LOST: resume spin steer=%d line=%d",
                             result.steering_error, line_valid);
                }
            }

            if ((state == FOLLOW_STATE_TRACK ||
                 state == FOLLOW_STATE_APPROACH_TURN) && line_valid) {
                const int signed_error =
                    STEERING_SIGN * result.steering_error;
                if (!filter_initialized) {
                    filtered_error = signed_error;
                    filter_initialized = true;
                } else {
                    filtered_error =
                        (filtered_error + 3 * signed_error) / 4;
                }
                const int control_error =
                    abs(filtered_error) <= FOLLOW_ERROR_DEADBAND
                        ? 0
                        : filtered_error;
                const int target_correction = clamp_int(
                    control_error * FOLLOW_CORRECTION_MAX / 1000,
                    -FOLLOW_CORRECTION_MAX, FOLLOW_CORRECTION_MAX);
                const int correction = clamp_int(
                    target_correction,
                    current_correction - FOLLOW_CORRECTION_STEP,
                    current_correction + FOLLOW_CORRECTION_STEP);
                const int confidence_base =
                    state == FOLLOW_STATE_APPROACH_TURN
                        ? TURN_APPROACH_DUTY
                        : (result.confidence < 50
                               ? FOLLOW_LOW_CONFIDENCE_DUTY
                               : FOLLOW_BASE_DUTY);
                const int turn_slowdown =
                    clamp_int(abs(control_error) * 20 / 1000, 0, 20);
                const int base = confidence_base - turn_slowdown;
                const int inner_min = FOLLOW_INNER_MIN_DUTY;
                const int a_duty = clamp_int(base - correction,
                                             inner_min,
                                             FOLLOW_MAX_DUTY);
                const int d_duty = clamp_int(base + correction,
                                             inner_min,
                                             FOLLOW_MAX_DUTY);
                current_base = base;
                current_correction = correction;
                current_a_duty = a_duty;
                current_b_duty = 0;
                current_d_duty = d_duty;
                drive_forward(a_duty, d_duty);
            }
            processed_sequence = sequence;
        }

        if (s_enabled && frame_fresh &&
            state == FOLLOW_STATE_ADVANCE_TO_TURN) {
            advance_a_counts = encoder_distance(s_encoder_a.count,
                                                advance_start_a_count);
            advance_d_counts = encoder_distance(s_encoder_d.count,
                                                advance_start_d_count);
            drive_forward(TURN_ADVANCE_DUTY, TURN_ADVANCE_DUTY);
            current_base = TURN_ADVANCE_DUTY;
            current_correction = 0;
            current_a_duty = TURN_ADVANCE_DUTY;
            current_b_duty = 0;
            current_d_duty = TURN_ADVANCE_DUTY;

            const bool advance_complete =
                advance_a_counts >= TURN_ADVANCE_COUNTS &&
                advance_d_counts >= TURN_ADVANCE_COUNTS;
            const bool advance_timed_out =
                now_us - state_started_us >=
                    (int64_t)TURN_ADVANCE_TIMEOUT_MS * 1000;
            if (advance_complete || advance_timed_out) {
                if (advance_timed_out && !advance_complete) {
                    ESP_LOGW(TAG,
                             "ADVANCE TIMEOUT A=%ld D=%ld target=%d; start spin",
                             (long)advance_a_counts,
                             (long)advance_d_counts,
                             TURN_ADVANCE_COUNTS);
                }
                state = FOLLOW_STATE_PIVOT;
                state_started_us = now_us;
                stable_frames = 0;
                ESP_LOGW(TAG,
                         "SPIN START direction=%d duty=%d advance=A%ld/D%ld",
                         latched_turn_direction, TURN_SPIN_DUTY,
                         (long)advance_a_counts, (long)advance_d_counts);
            }
        } else if (s_enabled && frame_fresh &&
                   state == FOLLOW_STATE_PIVOT) {
            drive_three_wheel_spin(latched_turn_direction, TURN_SPIN_DUTY);
            current_base = 0;
            current_correction = latched_turn_direction;
            signed_spin_duties(latched_turn_direction, TURN_SPIN_DUTY,
                               &current_a_duty, &current_b_duty,
                               &current_d_duty);
            const int64_t pivot_age_us = now_us - state_started_us;
            if ((pivot_age_us >= (int64_t)PIVOT_MIN_TIME_MS * 1000 &&
                 stable_frames >= REACQUIRE_STABLE_FRAMES) ||
                pivot_age_us >= (int64_t)PIVOT_MAX_TIME_MS * 1000) {
                state = FOLLOW_STATE_REACQUIRE;
                state_started_us = now_us;
                stable_frames = 0;
                ESP_LOGW(TAG, "PIVOT -> REACQUIRE");
            }
        } else if (s_enabled && frame_fresh &&
                   state == FOLLOW_STATE_REACQUIRE) {
            if (stable_frames > 0) {
                /* Stop on the first plausible line so rotation cannot sweep
                 * past it while waiting for three confirmation frames. */
                stop_motors();
                current_base = 0;
                current_correction = 0;
                current_a_duty = 0;
                current_b_duty = 0;
                current_d_duty = 0;
            } else {
                drive_three_wheel_spin(latched_turn_direction,
                                       REACQUIRE_SPIN_DUTY);
                signed_spin_duties(latched_turn_direction,
                                   REACQUIRE_SPIN_DUTY,
                                   &current_a_duty, &current_b_duty,
                                   &current_d_duty);
            }
            if (stable_frames >= REACQUIRE_STABLE_FRAMES) {
                state = FOLLOW_STATE_TRACK;
                state_started_us = now_us;
                turn_candidate_frames = 0;
                turn_candidate_direction = 0;
                turn_candidate_x = -1;
                turn_candidate_y = -1;
                latched_turn_direction = 0;
                last_corner_x = -1;
                last_corner_y = -1;
                last_corner_seen_us = 0;
                filtered_error = 0;
                filter_initialized = false;
                ESP_LOGW(TAG, "LINE REACQUIRED: normal tracking resumed");
            }
        }

        if (!s_enabled) {
            line_state_known = false;
            line_candidate_frames = 0;
        } else if (result_updated) {
            if (!line_state_known) {
                line_state_known = true;
                last_line_valid = line_valid;
                line_candidate_valid = line_valid;
                line_candidate_frames = 0;
            } else if (line_valid == last_line_valid) {
                line_candidate_valid = line_valid;
                line_candidate_frames = 0;
            } else {
                if (line_valid == line_candidate_valid) {
                    line_candidate_frames++;
                } else {
                    line_candidate_valid = line_valid;
                    line_candidate_frames = 1;
                }
                if (line_candidate_frames >= LINE_EVENT_CONFIRM_FRAMES) {
                    last_line_valid = line_valid;
                    line_candidate_frames = 0;
                    if (line_valid) {
                        ESP_LOGW(TAG, "LINE FOUND conf=%d", result.confidence);
                    } else {
                        ESP_LOGW(
                            TAG,
                            "LINE LOST: state=%s holding A=%d B=%d D=%d; X=stop",
                            state_name(state), current_a_duty,
                            current_b_duty, current_d_duty);
                    }
                }
            }
        }

        if (now_us - last_report_us >=
            (int64_t)FOLLOW_STATUS_INTERVAL_MS * 1000) {
            ESP_LOGI(TAG,
                     "FOLLOW en=%d state=%s line=%d conf=%d band=%d/%d band_err=%d steer=%d corner=%d,%d angle=%d dir=%d locked=%d last=%d,%d stable=%d base=%d A=%d B=%d D=%d advance=%ld/%ld fresh=%d",
                     s_enabled, state_name(state), line_valid,
                     result.confidence,
                     result.steering_band_left_percent,
                     result.steering_band_right_percent,
                     result.steering_band_error,
                     result.steering_error,
                     result.corner_x, result.corner_y,
                     result.turn_angle_deg, result.turn_direction,
                     latched_turn_direction, last_corner_x, last_corner_y,
                     stable_frames,
                     current_base, current_a_duty, current_b_duty,
                     current_d_duty, (long)advance_a_counts,
                     (long)advance_d_counts, frame_fresh);
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
    ESP_RETURN_ON_ERROR(configure_encoders(), TAG,
                        "encoder initialization failed");

    const esp_err_t uart_error = uart_driver_install(UART_NUM_0, 1024, 0, 0,
                                                      NULL, 0);
    if (uart_error != ESP_OK && uart_error != ESP_ERR_INVALID_STATE) {
        return uart_error;
    }
    if (xTaskCreatePinnedToCore(control_task, "camera_follow", 4096, NULL,
                                6, NULL, 0) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGW(TAG, "SAFE STOP. F=start X/SPACE=stop RGB,r,g,b DEBUG,0/1");
    ESP_LOGW(TAG,
             "TEST MODE: line loss holds the last command; X/SPACE to stop");
    ESP_LOGI(TAG,
             "Profile: track=%d min=%d max=%d trigger_y=%d advance=%d@%d spin=%d reacquire=%d",
             FOLLOW_BASE_DUTY, FOLLOW_INNER_MIN_DUTY, FOLLOW_MAX_DUTY,
             TURN_TRIGGER_Y, TURN_ADVANCE_COUNTS, TURN_ADVANCE_DUTY,
             TURN_SPIN_DUTY, REACQUIRE_SPIN_DUTY);
    return ESP_OK;
}

void camera_line_follow_submit(const line_vision_result_t *result)
{
    portENTER_CRITICAL(&s_result_lock);
    s_latest_result = *result;
    s_latest_frame_us = esp_timer_get_time();
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
