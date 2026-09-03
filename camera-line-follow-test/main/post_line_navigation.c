#include "post_line_navigation.h"

#include <math.h>
#include <stdint.h>
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
#include "post_line_odometry.h"

#define NAV_PI 3.14159265358979323846f
#define NAV_WHEEL_COUNT 3

#define DRIVER_STBY GPIO_NUM_5
#define MOTOR_A_PWM GPIO_NUM_6
#define MOTOR_A_IN1 GPIO_NUM_15
#define MOTOR_A_IN2 GPIO_NUM_7
#define MOTOR_B_PWM GPIO_NUM_11
#define MOTOR_B_IN1 GPIO_NUM_9
#define MOTOR_B_IN2 GPIO_NUM_10
#define MOTOR_D_PWM GPIO_NUM_40
#define MOTOR_D_IN1 GPIO_NUM_42
#define MOTOR_D_IN2 GPIO_NUM_41

#define NAV_CONTROL_INTERVAL_MS 20
#define NAV_SPEED_PI_INTERVAL_MS 150
#define NAV_STATUS_INTERVAL_MS 500
#define NAV_MOVE_TIMEOUT_MS 20000
#define NAV_ROTATE_TIMEOUT_MS 8000
#define NAV_SLOWDOWN_DISTANCE_MM 260.0f
#define NAV_MIN_TRANSLATION_SCALE 0.42f
#define NAV_COMMAND_SCALE 0.60f
#define NAV_HEADING_KP 0.30f
#define NAV_MAX_TURN_COMPONENT 0.28f
#define NAV_DRIVE_HEADING_DEADBAND_RAD 0.035f
#define NAV_ALIGN_ENTER_RAD 0.3490659f
#define NAV_ALIGN_EXIT_RAD 0.1221730f
#define NAV_ALIGN_MIN_TURN_COMPONENT 0.09f
#define NAV_ALIGN_SETTLE_MS 120
#define NAV_BRAKE_MS 140
#define NAV_WAYPOINT_SETTLE_MS 300
#define NAV_COMPONENT_DEADBAND 0.08f
#define NAV_PATH_WAYPOINT_RADIUS_MM 55.0f
#define NAV_PATH_HEADING_TOLERANCE_RAD 0.2094395f

/* The inner speed loop follows the camera-line controller's 150 ms PI and
 * anti-stall policy. These are floor-test parameters, intentionally grouped
 * here for later tuning. */
#define NAV_SPEED_TARGET_COUNTS 48
#define NAV_SPEED_KP_DIV 4
#define NAV_SPEED_KI_DIV 120
#define NAV_SPEED_INTEGRAL_LIMIT 3000
#define NAV_SPEED_OUTPUT_LIMIT 60
#define NAV_STALL_DELTA_MAX 6
#define NAV_STALL_KICK 30

#define NAV_A_MIN_DUTY 130
#define NAV_A_MAX_DUTY 235
#define NAV_B_MIN_DUTY 185
#define NAV_B_MAX_DUTY 380
#define NAV_D_MIN_DUTY 130
#define NAV_D_MAX_DUTY 235

enum {
    WHEEL_A = 0,
    WHEEL_B = 1,
    WHEEL_D = 2,
};

typedef struct {
    gpio_num_t pwm;
    gpio_num_t in1;
    gpio_num_t in2;
    ledc_channel_t channel;
    int polarity;
    int minimum_duty;
    int maximum_duty;
} navigation_motor_t;

typedef struct {
    float field_x_mm;
    float field_y_mm;
    float field_heading_rad;
    post_line_odometry_pose_t odometry;
} navigation_anchor_t;

static const navigation_motor_t s_motors[NAV_WHEEL_COUNT] = {
    {MOTOR_A_PWM, MOTOR_A_IN1, MOTOR_A_IN2, LEDC_CHANNEL_0,
     -1, NAV_A_MIN_DUTY, NAV_A_MAX_DUTY},
    {MOTOR_B_PWM, MOTOR_B_IN1, MOTOR_B_IN2, LEDC_CHANNEL_1,
     1, NAV_B_MIN_DUTY, NAV_B_MAX_DUTY},
    {MOTOR_D_PWM, MOTOR_D_IN1, MOTOR_D_IN2, LEDC_CHANNEL_2,
     1, NAV_D_MIN_DUTY, NAV_D_MAX_DUTY},
};

static const char *TAG = "post_nav";
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static navigation_anchor_t s_anchor;
static post_line_navigation_pose_t s_pose;
static bool s_started;
static bool s_paused;
static bool s_stopped;
static bool s_settling;
static post_line_navigation_command_t s_command;
static uint32_t s_command_id;
static float s_target_x_mm;
static float s_target_y_mm;
static float s_target_heading_rad;
static float s_target_tolerance_mm;
static float s_command_scale;
static int64_t s_command_timeout_us;
static int64_t s_phase_started_us;
static int64_t s_brake_until_us;
static int64_t s_last_status_us;
static int64_t s_last_pi_us;
static int32_t s_previous_counts[NAV_WHEEL_COUNT];
static int s_previous_sign[NAV_WHEEL_COUNT];
static int s_integral[NAV_WHEEL_COUNT];
static int s_pi_output[NAV_WHEEL_COUNT];
static bool s_reset_pi_requested;
static uint32_t s_anchor_revision;
static bool s_aligning;
static int64_t s_align_settle_until_us;
static post_line_navigation_waypoint_t
    s_path_points[POST_LINE_NAVIGATION_MAX_PATH_POINTS];
static size_t s_path_point_count;
static size_t s_path_point_index;

static bool navigation_motion_allowed(uint32_t command_id);

static int clamp_int(int value, int minimum, int maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

static float clamp_float(float value, float minimum, float maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

static float normalize_radians(float value)
{
    while (value > NAV_PI) value -= 2.0f * NAV_PI;
    while (value <= -NAV_PI) value += 2.0f * NAV_PI;
    return value;
}

static int32_t count_delta(int32_t current, int32_t previous)
{
    return (int32_t)((uint32_t)current - (uint32_t)previous);
}

const char *post_line_navigation_state_name(post_line_navigation_state_t state)
{
    switch (state) {
    case POST_NAV_WAITING: return "WAITING";
    case POST_NAV_RUNNING: return "RUNNING";
    case POST_NAV_PAUSED: return "PAUSED";
    case POST_NAV_SETTLING: return "SETTLING";
    case POST_NAV_COMPLETE: return "COMPLETE";
    case POST_NAV_STOPPED: return "STOPPED";
    default: return "UNKNOWN";
    }
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

static void prepare_motor(const navigation_motor_t *motor, int command)
{
    const int command_sign = command > 0 ? 1 : (command < 0 ? -1 : 0);
    const int direction = command_sign * motor->polarity;
    const int duty = clamp_int(abs(command), 0, 1023);
    ESP_ERROR_CHECK(gpio_set_level(motor->in1, direction > 0));
    ESP_ERROR_CHECK(gpio_set_level(motor->in2, direction < 0));
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, motor->channel, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, motor->channel));
}

static void coast_motors(void)
{
    for (size_t wheel = 0; wheel < NAV_WHEEL_COUNT; ++wheel) {
        prepare_motor(&s_motors[wheel], 0);
    }
    ESP_ERROR_CHECK(gpio_set_level(DRIVER_STBY, 0));
}

static void brake_motors(void)
{
    for (size_t wheel = 0; wheel < NAV_WHEEL_COUNT; ++wheel) {
        ESP_ERROR_CHECK(gpio_set_level(s_motors[wheel].in1, 1));
        ESP_ERROR_CHECK(gpio_set_level(s_motors[wheel].in2, 1));
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE,
                                      s_motors[wheel].channel, 1023));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE,
                                         s_motors[wheel].channel));
    }
    ESP_ERROR_CHECK(gpio_set_level(DRIVER_STBY, 1));
}

static void reset_speed_pi(const post_line_odometry_pose_t *odometry,
                           int64_t now_us)
{
    const int32_t counts[NAV_WHEEL_COUNT] = {
        odometry->count_a, odometry->count_b, odometry->count_d};
    for (size_t wheel = 0; wheel < NAV_WHEEL_COUNT; ++wheel) {
        s_previous_counts[wheel] = counts[wheel];
        s_previous_sign[wheel] = 0;
        s_integral[wheel] = 0;
        s_pi_output[wheel] = 0;
    }
    s_last_pi_us = now_us;
}

static void update_speed_pi(const float wheel_request[NAV_WHEEL_COUNT],
                            const post_line_odometry_pose_t *odometry,
                            int64_t now_us)
{
    if (now_us - s_last_pi_us <
        (int64_t)NAV_SPEED_PI_INTERVAL_MS * 1000) return;

    const int32_t counts[NAV_WHEEL_COUNT] = {
        odometry->count_a, odometry->count_b, odometry->count_d};
    for (size_t wheel = 0; wheel < NAV_WHEEL_COUNT; ++wheel) {
        const float magnitude = fabsf(wheel_request[wheel]);
        const int sign = wheel_request[wheel] > 0.0f
                             ? 1 : (wheel_request[wheel] < 0.0f ? -1 : 0);
        const int delta = abs(count_delta(counts[wheel],
                                          s_previous_counts[wheel]));
        s_previous_counts[wheel] = counts[wheel];
        if (sign == 0 || sign != s_previous_sign[wheel]) {
            s_integral[wheel] = 0;
            s_pi_output[wheel] = 0;
            s_previous_sign[wheel] = sign;
            continue;
        }
        const int target = (int)lroundf(
            magnitude * NAV_SPEED_TARGET_COUNTS);
        const int error = target - delta;
        s_integral[wheel] = clamp_int(
            s_integral[wheel] + error,
            -NAV_SPEED_INTEGRAL_LIMIT, NAV_SPEED_INTEGRAL_LIMIT);
        int output = error / NAV_SPEED_KP_DIV +
                     s_integral[wheel] / NAV_SPEED_KI_DIV;
        output = clamp_int(output, -NAV_SPEED_OUTPUT_LIMIT,
                           NAV_SPEED_OUTPUT_LIMIT);
        if (target > NAV_STALL_DELTA_MAX && delta <= NAV_STALL_DELTA_MAX &&
            output < NAV_STALL_KICK) {
            output = NAV_STALL_KICK;
        }
        s_pi_output[wheel] = output;
    }
    s_last_pi_us = now_us;
}

static void apply_wheel_requests(
    const float wheel_request[NAV_WHEEL_COUNT],
    const post_line_odometry_pose_t *odometry, int64_t now_us)
{
    update_speed_pi(wheel_request, odometry, now_us);
    bool active = false;
    for (size_t wheel = 0; wheel < NAV_WHEEL_COUNT; ++wheel) {
        const float magnitude = fabsf(wheel_request[wheel]);
        if (magnitude < NAV_COMPONENT_DEADBAND) {
            prepare_motor(&s_motors[wheel], 0);
            continue;
        }
        const navigation_motor_t *motor = &s_motors[wheel];
        const int feedforward = motor->minimum_duty + (int)lroundf(
            magnitude * (motor->maximum_duty - motor->minimum_duty));
        const int duty = clamp_int(feedforward + s_pi_output[wheel],
                                   motor->minimum_duty,
                                   motor->maximum_duty);
        const int signed_duty = wheel_request[wheel] > 0.0f ? duty : -duty;
        prepare_motor(motor, signed_duty);
        active = true;
    }
    ESP_ERROR_CHECK(gpio_set_level(DRIVER_STBY, active ? 1 : 0));
}

static void apply_scaled_wheel_requests(
    float wheel_request[NAV_WHEEL_COUNT],
    const post_line_odometry_pose_t *odometry, int64_t now_us,
    float command_scale)
{
    float maximum = 1.0f;
    for (size_t wheel = 0; wheel < NAV_WHEEL_COUNT; ++wheel) {
        if (fabsf(wheel_request[wheel]) > maximum) {
            maximum = fabsf(wheel_request[wheel]);
        }
    }
    for (size_t wheel = 0; wheel < NAV_WHEEL_COUNT; ++wheel) {
        wheel_request[wheel] =
            wheel_request[wheel] / maximum * command_scale;
    }
    apply_wheel_requests(wheel_request, odometry, now_us);
}

static void apply_rotation_request(
    float heading_error, const post_line_odometry_pose_t *odometry,
    int64_t now_us, float command_scale, uint32_t command_id)
{
    float turn = clamp_float(
        NAV_HEADING_KP * heading_error,
        -NAV_MAX_TURN_COMPONENT, NAV_MAX_TURN_COMPONENT) * command_scale;
    if (fabsf(turn) < NAV_ALIGN_MIN_TURN_COMPONENT) {
        turn = turn < 0.0f
            ? -NAV_ALIGN_MIN_TURN_COMPONENT
            : NAV_ALIGN_MIN_TURN_COMPONENT;
    }
    float wheel_request[NAV_WHEEL_COUNT] = {turn, turn, turn};
    if (navigation_motion_allowed(command_id)) {
        apply_wheel_requests(wheel_request, odometry, now_us);
    } else {
        brake_motors();
    }
}

static void apply_path_request(
    float field_x, float field_y, float heading_rad, float heading_error,
    float translation_scale, const post_line_odometry_pose_t *odometry,
    int64_t now_us, float command_scale, uint32_t command_id)
{
    const float sine = sinf(heading_rad);
    const float cosine = cosf(heading_rad);
    const float body_right = sine * field_x - cosine * field_y;
    const float body_forward = cosine * field_x + sine * field_y;
    const float turn = clamp_float(
        NAV_HEADING_KP * heading_error,
        -NAV_MAX_TURN_COMPONENT, NAV_MAX_TURN_COMPONENT);
    float wheel_request[NAV_WHEEL_COUNT] = {
        -0.5f * body_right * translation_scale +
            0.8660254f * body_forward * translation_scale + turn,
        body_right * translation_scale + turn,
        -0.5f * body_right * translation_scale -
            0.8660254f * body_forward * translation_scale + turn,
    };
    if (navigation_motion_allowed(command_id)) {
        apply_scaled_wheel_requests(wheel_request, odometry, now_us,
                                    command_scale);
    } else {
        brake_motors();
    }
}

static post_line_navigation_pose_t pose_from_odometry(
    const navigation_anchor_t *anchor,
    const post_line_odometry_pose_t *odometry)
{
    const float delta_x = odometry->x_mm - anchor->odometry.x_mm;
    const float delta_y = odometry->y_mm - anchor->odometry.y_mm;
    const float boot_to_field = anchor->field_heading_rad -
        (NAV_PI * 0.5f + anchor->odometry.heading_rad);
    const float cosine = cosf(boot_to_field);
    const float sine = sinf(boot_to_field);
    post_line_navigation_pose_t pose = {
        .valid = true,
        .x_mm = anchor->field_x_mm + cosine * delta_x - sine * delta_y,
        .y_mm = anchor->field_y_mm + sine * delta_x + cosine * delta_y,
        .heading_rad = normalize_radians(
            anchor->field_heading_rad + odometry->heading_rad -
            anchor->odometry.heading_rad),
    };
    pose.heading_deg = pose.heading_rad * 180.0f / NAV_PI;
    return pose;
}

/* Caller holds s_lock. */
static post_line_navigation_state_t current_state_locked(void)
{
    if (s_stopped) return POST_NAV_STOPPED;
    if (!s_started) return POST_NAV_WAITING;
    if (s_paused) return POST_NAV_PAUSED;
    if (s_settling) return POST_NAV_SETTLING;
    if (s_command == POST_NAV_COMMAND_NONE) return POST_NAV_COMPLETE;
    return POST_NAV_RUNNING;
}

static bool navigation_motion_allowed(uint32_t command_id)
{
    portENTER_CRITICAL(&s_lock);
    const bool allowed = s_started && !s_paused && !s_stopped &&
        !s_settling && s_command != POST_NAV_COMMAND_NONE &&
        s_command_id == command_id;
    portEXIT_CRITICAL(&s_lock);
    return allowed;
}

static void publish_pose(post_line_navigation_pose_t pose,
                         uint32_t anchor_revision)
{
    portENTER_CRITICAL(&s_lock);
    if (anchor_revision != s_anchor_revision) {
        portEXIT_CRITICAL(&s_lock);
        return;
    }
    pose.state = current_state_locked();
    pose.command = s_command;
    pose.command_id = s_command_id;
    pose.waypoint_index = (int)s_command_id;
    pose.target_x_mm = s_target_x_mm;
    pose.target_y_mm = s_target_y_mm;
    pose.target_heading_deg = s_target_heading_rad * 180.0f / NAV_PI;
    if (s_command == POST_NAV_COMMAND_MOVE ||
        s_command == POST_NAV_COMMAND_PUSH ||
        s_command == POST_NAV_COMMAND_REVERSE ||
        s_command == POST_NAV_COMMAND_PATH) {
        pose.distance_to_target_mm = hypotf(
            pose.target_x_mm - pose.x_mm,
            pose.target_y_mm - pose.y_mm);
    } else {
        pose.distance_to_target_mm = 0.0f;
    }
    s_pose = pose;
    portEXIT_CRITICAL(&s_lock);
}

static void navigation_task(void *argument)
{
    (void)argument;
    TickType_t last_wake = xTaskGetTickCount();
    while (true) {
        const int64_t now_us = esp_timer_get_time();
        post_line_odometry_pose_t odometry;
        if (!post_line_odometry_get_pose(&odometry)) {
            coast_motors();
            vTaskDelayUntil(&last_wake,
                            pdMS_TO_TICKS(NAV_CONTROL_INTERVAL_MS));
            continue;
        }

        navigation_anchor_t anchor;
        uint32_t anchor_revision;
        bool started;
        bool paused;
        bool stopped;
        bool settling;
        bool reset_pi;
        post_line_navigation_command_t command;
        uint32_t command_id;
        float target_x;
        float target_y;
        float target_heading;
        float target_tolerance;
        float command_scale;
        int64_t command_timeout_us;
        int64_t phase_started_us;
        int64_t brake_until_us;
        size_t path_point_count;
        size_t path_point_index;
        portENTER_CRITICAL(&s_lock);
        anchor = s_anchor;
        anchor_revision = s_anchor_revision;
        started = s_started;
        paused = s_paused;
        stopped = s_stopped;
        settling = s_settling;
        command = s_command;
        command_id = s_command_id;
        target_x = s_target_x_mm;
        target_y = s_target_y_mm;
        target_heading = s_target_heading_rad;
        target_tolerance = s_target_tolerance_mm;
        command_scale = s_command_scale;
        command_timeout_us = s_command_timeout_us;
        phase_started_us = s_phase_started_us;
        brake_until_us = s_brake_until_us;
        path_point_count = s_path_point_count;
        path_point_index = s_path_point_index;
        reset_pi = s_reset_pi_requested;
        s_reset_pi_requested = false;
        portEXIT_CRITICAL(&s_lock);
        if (reset_pi) reset_speed_pi(&odometry, now_us);
        post_line_navigation_pose_t pose =
            pose_from_odometry(&anchor, &odometry);

        if (stopped || !started || command == POST_NAV_COMMAND_NONE) {
            coast_motors();
            publish_pose(pose, anchor_revision);
        } else if (paused) {
            if (now_us < brake_until_us) brake_motors();
            else coast_motors();
            publish_pose(pose, anchor_revision);
        } else if (settling) {
            if (now_us < brake_until_us) {
                brake_motors();
            } else if (now_us - phase_started_us >=
                           (int64_t)NAV_WAYPOINT_SETTLE_MS * 1000) {
                coast_motors();
                portENTER_CRITICAL(&s_lock);
                if (s_settling && s_command_id == command_id) {
                    s_settling = false;
                    s_command = POST_NAV_COMMAND_NONE;
                    s_phase_started_us = now_us;
                    s_aligning = true;
                    s_align_settle_until_us = 0;
                }
                portEXIT_CRITICAL(&s_lock);
                reset_speed_pi(&odometry, now_us);
                ESP_LOGW(TAG,
                         "COMMAND_COMPLETE id=%lu pose=(%d,%d,%ddeg)",
                         (unsigned long)command_id,
                         (int)lroundf(pose.x_mm),
                         (int)lroundf(pose.y_mm),
                         (int)lroundf(pose.heading_deg));
            } else {
                coast_motors();
            }
            publish_pose(pose, anchor_revision);
        } else if (command == POST_NAV_COMMAND_ROTATE) {
            const float heading_error = normalize_radians(
                target_heading - pose.heading_rad);
            if (fabsf(heading_error) <= NAV_ALIGN_EXIT_RAD) {
                portENTER_CRITICAL(&s_lock);
                s_settling = true;
                s_phase_started_us = now_us;
                s_brake_until_us = now_us + (int64_t)NAV_BRAKE_MS * 1000;
                portEXIT_CRITICAL(&s_lock);
                brake_motors();
                ESP_LOGW(TAG,
                         "HEADING_REACHED id=%lu heading=%ddeg target=%ddeg",
                         (unsigned long)command_id,
                         (int)lroundf(pose.heading_deg),
                         (int)lroundf(target_heading * 180.0f / NAV_PI));
            } else if (now_us - phase_started_us >= command_timeout_us) {
                brake_motors();
                portENTER_CRITICAL(&s_lock);
                s_stopped = true;
                portEXIT_CRITICAL(&s_lock);
                ESP_LOGE(TAG,
                         "ROTATE_TIMEOUT id=%lu heading=%ddeg target=%ddeg",
                         (unsigned long)command_id,
                         (int)lroundf(pose.heading_deg),
                         (int)lroundf(target_heading * 180.0f / NAV_PI));
            } else {
                apply_rotation_request(heading_error, &odometry, now_us,
                                       command_scale, command_id);
            }
            publish_pose(pose, anchor_revision);
        } else {
            const float error_x = target_x - pose.x_mm;
            const float error_y = target_y - pose.y_mm;
            const float distance = hypotf(error_x, error_y);
            const bool path_command = command == POST_NAV_COMMAND_PATH;
            const bool final_path_point = !path_command ||
                path_point_index + 1 >= path_point_count;
            const float path_heading_error = normalize_radians(
                target_heading - pose.heading_rad);
            const float forward_remaining =
                cosf(target_heading) * error_x +
                sinf(target_heading) * error_y;
            const bool reverse_drive =
                command == POST_NAV_COMMAND_REVERSE;
            const bool fixed_heading_drive =
                command == POST_NAV_COMMAND_PUSH || reverse_drive;
            const float drive_direction = reverse_drive ? -1.0f : 1.0f;
            const float directional_remaining =
                drive_direction * forward_remaining;
            const bool position_reached = path_command
                ? final_path_point && distance <= target_tolerance &&
                    fabsf(path_heading_error) <=
                        NAV_PATH_HEADING_TOLERANCE_RAD
                : fixed_heading_drive
                ? directional_remaining <= target_tolerance
                : distance <= target_tolerance;
            if (path_command && !final_path_point &&
                distance <= NAV_PATH_WAYPOINT_RADIUS_MM) {
                portENTER_CRITICAL(&s_lock);
                if (s_command == POST_NAV_COMMAND_PATH &&
                    s_command_id == command_id &&
                    s_path_point_index + 1 < s_path_point_count) {
                    s_path_point_index++;
                    s_target_x_mm = s_path_points[s_path_point_index].x_mm;
                    s_target_y_mm = s_path_points[s_path_point_index].y_mm;
                    s_phase_started_us = now_us;
                }
                const size_t next_index = s_path_point_index;
                const float next_x = s_target_x_mm;
                const float next_y = s_target_y_mm;
                portEXIT_CRITICAL(&s_lock);
                ESP_LOGW(TAG,
                         "PATH_WAYPOINT id=%lu reached=%u/%u next=(%d,%d)",
                         (unsigned long)command_id,
                         (unsigned)(next_index),
                         (unsigned)path_point_count,
                         (int)lroundf(next_x), (int)lroundf(next_y));
            } else if (position_reached) {
                portENTER_CRITICAL(&s_lock);
                s_settling = true;
                s_phase_started_us = now_us;
                s_brake_until_us = now_us + (int64_t)NAV_BRAKE_MS * 1000;
                portEXIT_CRITICAL(&s_lock);
                brake_motors();
                ESP_LOGW(TAG,
                         "POSITION_REACHED id=%lu pose=(%d,%d,%ddeg) error=%dmm",
                         (unsigned long)command_id,
                         (int)lroundf(pose.x_mm),
                         (int)lroundf(pose.y_mm),
                         (int)lroundf(pose.heading_deg),
                         (int)lroundf(distance));
            } else if (now_us - phase_started_us >= command_timeout_us) {
                brake_motors();
                portENTER_CRITICAL(&s_lock);
                s_stopped = true;
                portEXIT_CRITICAL(&s_lock);
                ESP_LOGE(TAG,
                         "MOVE_TIMEOUT id=%lu pose=(%d,%d) remaining=%dmm",
                         (unsigned long)command_id,
                         (int)lroundf(pose.x_mm),
                         (int)lroundf(pose.y_mm),
                         (int)lroundf(distance));
            } else {
                const float translation_scale = distance >=
                        NAV_SLOWDOWN_DISTANCE_MM
                    ? 1.0f
                    : clamp_float(distance / NAV_SLOWDOWN_DISTANCE_MM,
                                  NAV_MIN_TRANSLATION_SCALE, 1.0f);
                const float desired_heading = path_command ||
                        fixed_heading_drive
                    ? target_heading : atan2f(error_y, error_x);
                float heading_error = normalize_radians(
                    desired_heading - pose.heading_rad);
                if (path_command) {
                    if (distance <= target_tolerance) {
                        apply_rotation_request(
                            heading_error, &odometry, now_us,
                            command_scale, command_id);
                    } else {
                        const float inverse_distance = 1.0f / distance;
                        apply_path_request(
                            error_x * inverse_distance,
                            error_y * inverse_distance,
                            pose.heading_rad, heading_error,
                            translation_scale, &odometry, now_us,
                            command_scale, command_id);
                    }
                } else if (now_us < s_align_settle_until_us) {
                    brake_motors();
                } else if (!fixed_heading_drive && !s_aligning &&
                    fabsf(heading_error) > NAV_ALIGN_ENTER_RAD) {
                    s_aligning = true;
                    s_align_settle_until_us = 0;
                    reset_speed_pi(&odometry, now_us);
                    brake_motors();
                    ESP_LOGW(TAG,
                             "REALIGN heading=%ddeg target=%ddeg error=%ddeg",
                             (int)lroundf(pose.heading_deg),
                             (int)lroundf(desired_heading * 180.0f / NAV_PI),
                             (int)lroundf(heading_error * 180.0f / NAV_PI));
                } else if (s_aligning &&
                           fabsf(heading_error) <= NAV_ALIGN_EXIT_RAD) {
                    s_aligning = false;
                    s_align_settle_until_us = now_us +
                        (int64_t)NAV_ALIGN_SETTLE_MS * 1000;
                    reset_speed_pi(&odometry, now_us);
                    brake_motors();
                    ESP_LOGW(TAG,
                             "ALIGNED heading=%ddeg target=%ddeg; drive forward",
                             (int)lroundf(pose.heading_deg),
                             (int)lroundf(desired_heading * 180.0f / NAV_PI));
                } else if (s_aligning) {
                    apply_rotation_request(heading_error, &odometry, now_us,
                                           command_scale, command_id);
                } else {
                    if (fabsf(heading_error) <
                        NAV_DRIVE_HEADING_DEADBAND_RAD) {
                        heading_error = 0.0f;
                    }
                    const float turn = clamp_float(
                        NAV_HEADING_KP * heading_error,
                        -NAV_MAX_TURN_COMPONENT, NAV_MAX_TURN_COMPONENT);
                    float wheel_request[NAV_WHEEL_COUNT] = {
                        0.8660254f * translation_scale * drive_direction + turn,
                        0.0f,
                        -0.8660254f * translation_scale * drive_direction + turn,
                    };
                    if (navigation_motion_allowed(command_id)) {
                        apply_scaled_wheel_requests(
                            wheel_request, &odometry, now_us,
                            command_scale);
                    } else {
                        brake_motors();
                    }
                }
            }
            publish_pose(pose, anchor_revision);
        }

        if (now_us - s_last_status_us >=
                (int64_t)NAV_STATUS_INTERVAL_MS * 1000) {
            post_line_navigation_pose_t status;
            post_line_navigation_get_pose(&status);
            const char *mode = status.command == POST_NAV_COMMAND_PATH
                ? "CURVE" : (s_aligning ? "ALIGN" : "DRIVE");
            ESP_LOGI(TAG,
                     "NAV state=%s command=%d id=%lu mode=%s pose=(%d,%d,%ddeg) target=(%d,%d,%ddeg) remaining=%dmm",
                     post_line_navigation_state_name(status.state),
                     status.command, (unsigned long)status.command_id,
                     mode,
                     (int)lroundf(status.x_mm),
                     (int)lroundf(status.y_mm),
                     (int)lroundf(status.heading_deg),
                     (int)lroundf(status.target_x_mm),
                     (int)lroundf(status.target_y_mm),
                     (int)lroundf(status.target_heading_deg),
                     (int)lroundf(status.distance_to_target_mm));
            s_last_status_us = now_us;
        }
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(NAV_CONTROL_INTERVAL_MS));
    }
}

static void navigation_safety_uart_task(void *argument)
{
    (void)argument;
    uint8_t input[32];
    while (true) {
        const int count = uart_read_bytes(UART_NUM_0, input, sizeof(input),
                                          pdMS_TO_TICKS(20));
        for (int index = 0; index < count; ++index) {
            if (input[index] == 'x' || input[index] == 'X' ||
                input[index] == ' ') {
                post_line_navigation_stop();
                ESP_LOGE(TAG,
                         "EMERGENCY_STOP received; reset required to restart");
                break;
            }
        }
    }
}

esp_err_t post_line_navigation_init(float initial_x_mm, float initial_y_mm,
                                    float initial_heading_deg)
{
    post_line_odometry_pose_t odometry;
    if (!post_line_odometry_get_pose(&odometry)) return ESP_ERR_INVALID_STATE;

    set_output_low(DRIVER_STBY);
    for (size_t wheel = 0; wheel < NAV_WHEEL_COUNT; ++wheel) {
        set_output_low(s_motors[wheel].in1);
        set_output_low(s_motors[wheel].in2);
    }
    const ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 20000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_RETURN_ON_ERROR(ledc_timer_config(&timer), TAG,
                        "navigation PWM timer failed");
    for (size_t wheel = 0; wheel < NAV_WHEEL_COUNT; ++wheel) {
        const ledc_channel_config_t channel = {
            .gpio_num = s_motors[wheel].pwm,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = s_motors[wheel].channel,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = LEDC_TIMER_0,
            .duty = 0,
            .hpoint = 0,
        };
        ESP_RETURN_ON_ERROR(ledc_channel_config(&channel), TAG,
                            "navigation PWM channel failed");
    }
    coast_motors();

    memset(&s_pose, 0, sizeof(s_pose));
    s_anchor = (navigation_anchor_t) {
        .field_x_mm = initial_x_mm,
        .field_y_mm = initial_y_mm,
        .field_heading_rad = initial_heading_deg * NAV_PI / 180.0f,
        .odometry = odometry,
    };
    s_pose = pose_from_odometry(&s_anchor, &odometry);
    s_pose.state = POST_NAV_WAITING;
    s_pose.command = POST_NAV_COMMAND_NONE;
    s_pose.command_id = 0;
    s_pose.waypoint_index = 0;
    s_pose.target_x_mm = initial_x_mm;
    s_pose.target_y_mm = initial_y_mm;
    s_pose.target_heading_deg = initial_heading_deg;
    s_pose.distance_to_target_mm = 0.0f;
    s_started = false;
    s_paused = false;
    s_stopped = false;
    s_settling = false;
    s_reset_pi_requested = false;
    s_anchor_revision = 1;
    s_aligning = true;
    s_align_settle_until_us = 0;
    s_command = POST_NAV_COMMAND_NONE;
    s_command_id = 0;
    s_path_point_count = 0;
    s_path_point_index = 0;
    s_target_x_mm = initial_x_mm;
    s_target_y_mm = initial_y_mm;
    s_target_heading_rad = initial_heading_deg * NAV_PI / 180.0f;
    s_target_tolerance_mm = 45.0f;
    s_command_scale = NAV_COMMAND_SCALE;
    s_command_timeout_us = (int64_t)NAV_MOVE_TIMEOUT_MS * 1000;
    const int64_t now_us = esp_timer_get_time();
    s_phase_started_us = now_us;
    reset_speed_pi(&odometry, now_us);
    const esp_err_t uart_error = uart_driver_install(
        UART_NUM_0, 1024, 0, 0, NULL, 0);
    if (uart_error != ESP_OK && uart_error != ESP_ERR_INVALID_STATE) {
        return uart_error;
    }
    if (xTaskCreatePinnedToCore(navigation_task, "post_navigation", 6144,
                                NULL, 6, NULL, 0) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    if (xTaskCreatePinnedToCore(navigation_safety_uart_task, "nav_safety",
                                3072, NULL, 7, NULL, 0) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGW(TAG,
             "NAV_READY initial=(%d,%d,%ddeg) dynamic_commands=1 scale=60%%",
             (int)lroundf(initial_x_mm), (int)lroundf(initial_y_mm),
             (int)lroundf(initial_heading_deg));
    return ESP_OK;
}

void post_line_navigation_start(void)
{
    portENTER_CRITICAL(&s_lock);
    if (!s_stopped) {
        s_started = true;
        s_paused = false;
    }
    portEXIT_CRITICAL(&s_lock);
}

void post_line_navigation_pause(void)
{
    portENTER_CRITICAL(&s_lock);
    s_paused = true;
    s_brake_until_us = esp_timer_get_time() + (int64_t)NAV_BRAKE_MS * 1000;
    portEXIT_CRITICAL(&s_lock);
}

void post_line_navigation_resume(void)
{
    const int64_t now_us = esp_timer_get_time();
    portENTER_CRITICAL(&s_lock);
    if (s_started && !s_stopped) {
        s_paused = false;
        s_phase_started_us = now_us;
        s_reset_pi_requested = true;
    }
    portEXIT_CRITICAL(&s_lock);
}

void post_line_navigation_stop(void)
{
    portENTER_CRITICAL(&s_lock);
    s_stopped = true;
    portEXIT_CRITICAL(&s_lock);
}

bool post_line_navigation_move_to(float x_mm, float y_mm,
                                  float tolerance_mm, float speed_scale,
                                  uint32_t *command_id)
{
    if (!isfinite(x_mm) || !isfinite(y_mm) || tolerance_mm <= 0.0f ||
        speed_scale <= 0.0f || speed_scale > 1.0f) {
        return false;
    }
    const int64_t now_us = esp_timer_get_time();
    portENTER_CRITICAL(&s_lock);
    if (!s_started || s_stopped) {
        portEXIT_CRITICAL(&s_lock);
        return false;
    }
    s_command_id++;
    const uint32_t issued_id = s_command_id;
    s_command = POST_NAV_COMMAND_MOVE;
    s_path_point_count = 0;
    s_path_point_index = 0;
    s_target_x_mm = x_mm;
    s_target_y_mm = y_mm;
    s_target_heading_rad = atan2f(y_mm - s_pose.y_mm,
                                  x_mm - s_pose.x_mm);
    s_target_tolerance_mm = tolerance_mm;
    s_command_scale = speed_scale;
    s_command_timeout_us = (int64_t)NAV_MOVE_TIMEOUT_MS * 1000;
    s_paused = false;
    s_settling = false;
    s_aligning = true;
    s_align_settle_until_us = 0;
    s_phase_started_us = now_us;
    s_reset_pi_requested = true;
    s_pose.command = s_command;
    s_pose.command_id = issued_id;
    s_pose.state = POST_NAV_RUNNING;
    if (command_id) *command_id = issued_id;
    portEXIT_CRITICAL(&s_lock);
    ESP_LOGW(TAG, "MOVE_COMMAND id=%lu target=(%d,%d) tolerance=%d scale=%d%%",
             (unsigned long)issued_id, (int)lroundf(x_mm),
             (int)lroundf(y_mm), (int)lroundf(tolerance_mm),
             (int)lroundf(speed_scale * 100.0f));
    return true;
}

bool post_line_navigation_rotate_to(float heading_deg, float speed_scale,
                                    uint32_t *command_id)
{
    if (!isfinite(heading_deg) || speed_scale <= 0.0f ||
        speed_scale > 1.0f) {
        return false;
    }
    const int64_t now_us = esp_timer_get_time();
    portENTER_CRITICAL(&s_lock);
    if (!s_started || s_stopped) {
        portEXIT_CRITICAL(&s_lock);
        return false;
    }
    s_command_id++;
    const uint32_t issued_id = s_command_id;
    s_command = POST_NAV_COMMAND_ROTATE;
    s_path_point_count = 0;
    s_path_point_index = 0;
    s_target_x_mm = s_pose.x_mm;
    s_target_y_mm = s_pose.y_mm;
    s_target_heading_rad = normalize_radians(heading_deg * NAV_PI / 180.0f);
    s_target_tolerance_mm = 0.0f;
    s_command_scale = speed_scale;
    s_command_timeout_us = (int64_t)NAV_ROTATE_TIMEOUT_MS * 1000;
    s_paused = false;
    s_settling = false;
    s_aligning = true;
    s_align_settle_until_us = 0;
    s_phase_started_us = now_us;
    s_reset_pi_requested = true;
    s_pose.command = s_command;
    s_pose.command_id = issued_id;
    s_pose.state = POST_NAV_RUNNING;
    if (command_id) *command_id = issued_id;
    portEXIT_CRITICAL(&s_lock);
    ESP_LOGW(TAG, "ROTATE_COMMAND id=%lu heading=%ddeg scale=%d%%",
             (unsigned long)issued_id, (int)lroundf(heading_deg),
             (int)lroundf(speed_scale * 100.0f));
    return true;
}

bool post_line_navigation_push_to(float x_mm, float y_mm,
                                  float heading_deg, float tolerance_mm,
                                  float speed_scale, uint32_t *command_id)
{
    if (!isfinite(x_mm) || !isfinite(y_mm) || !isfinite(heading_deg) ||
        tolerance_mm <= 0.0f || speed_scale <= 0.0f ||
        speed_scale > 1.0f) {
        return false;
    }
    const int64_t now_us = esp_timer_get_time();
    portENTER_CRITICAL(&s_lock);
    if (!s_started || s_stopped) {
        portEXIT_CRITICAL(&s_lock);
        return false;
    }
    s_command_id++;
    const uint32_t issued_id = s_command_id;
    s_command = POST_NAV_COMMAND_PUSH;
    s_path_point_count = 0;
    s_path_point_index = 0;
    s_target_x_mm = x_mm;
    s_target_y_mm = y_mm;
    s_target_heading_rad = normalize_radians(heading_deg * NAV_PI / 180.0f);
    s_target_tolerance_mm = tolerance_mm;
    s_command_scale = speed_scale;
    s_command_timeout_us = (int64_t)NAV_MOVE_TIMEOUT_MS * 1000;
    s_paused = false;
    s_settling = false;
    s_aligning = true;
    s_align_settle_until_us = 0;
    s_phase_started_us = now_us;
    s_reset_pi_requested = true;
    s_pose.command = s_command;
    s_pose.command_id = issued_id;
    s_pose.state = POST_NAV_RUNNING;
    if (command_id) *command_id = issued_id;
    portEXIT_CRITICAL(&s_lock);
    ESP_LOGW(TAG,
             "PUSH_COMMAND id=%lu target=(%d,%d) heading=%ddeg tolerance=%d scale=%d%%",
             (unsigned long)issued_id, (int)lroundf(x_mm),
             (int)lroundf(y_mm), (int)lroundf(heading_deg),
             (int)lroundf(tolerance_mm),
             (int)lroundf(speed_scale * 100.0f));
    return true;
}

bool post_line_navigation_reverse_by(float distance_mm, float speed_scale,
                                     uint32_t *command_id)
{
    if (!isfinite(distance_mm) || distance_mm <= 0.0f ||
        !isfinite(speed_scale) || speed_scale <= 0.0f ||
        speed_scale > 1.0f) {
        return false;
    }
    const int64_t now_us = esp_timer_get_time();
    portENTER_CRITICAL(&s_lock);
    if (!s_started || s_stopped) {
        portEXIT_CRITICAL(&s_lock);
        return false;
    }
    s_command_id++;
    const uint32_t issued_id = s_command_id;
    s_command = POST_NAV_COMMAND_REVERSE;
    s_path_point_count = 0;
    s_path_point_index = 0;
    s_target_heading_rad = s_pose.heading_rad;
    s_target_x_mm = s_pose.x_mm - cosf(s_target_heading_rad) * distance_mm;
    s_target_y_mm = s_pose.y_mm - sinf(s_target_heading_rad) * distance_mm;
    s_target_tolerance_mm = 8.0f;
    s_command_scale = speed_scale;
    s_command_timeout_us = (int64_t)NAV_MOVE_TIMEOUT_MS * 1000;
    s_paused = false;
    s_settling = false;
    s_aligning = true;
    s_align_settle_until_us = 0;
    s_phase_started_us = now_us;
    s_reset_pi_requested = true;
    s_pose.command = s_command;
    s_pose.command_id = issued_id;
    s_pose.state = POST_NAV_RUNNING;
    if (command_id) *command_id = issued_id;
    const float target_x = s_target_x_mm;
    const float target_y = s_target_y_mm;
    const float heading_deg = s_target_heading_rad * 180.0f / NAV_PI;
    portEXIT_CRITICAL(&s_lock);
    ESP_LOGW(TAG,
             "REVERSE_COMMAND id=%lu distance=%dmm target=(%d,%d) heading=%ddeg scale=%d%%",
             (unsigned long)issued_id, (int)lroundf(distance_mm),
             (int)lroundf(target_x), (int)lroundf(target_y),
             (int)lroundf(heading_deg),
             (int)lroundf(speed_scale * 100.0f));
    return true;
}

bool post_line_navigation_follow_path(
    const post_line_navigation_waypoint_t *points, size_t point_count,
    float final_heading_deg, float final_tolerance_mm, float speed_scale,
    uint32_t *command_id)
{
    if (!points || point_count == 0 ||
        point_count > POST_LINE_NAVIGATION_MAX_PATH_POINTS ||
        !isfinite(final_heading_deg) || final_tolerance_mm <= 0.0f ||
        speed_scale <= 0.0f || speed_scale > 1.0f) {
        return false;
    }
    for (size_t index = 0; index < point_count; ++index) {
        if (!isfinite(points[index].x_mm) ||
            !isfinite(points[index].y_mm)) {
            return false;
        }
    }

    const int64_t now_us = esp_timer_get_time();
    portENTER_CRITICAL(&s_lock);
    if (!s_started || s_stopped) {
        portEXIT_CRITICAL(&s_lock);
        return false;
    }
    memcpy(s_path_points, points,
           point_count * sizeof(s_path_points[0]));
    s_path_point_count = point_count;
    s_path_point_index = 0;
    s_command_id++;
    const uint32_t issued_id = s_command_id;
    s_command = POST_NAV_COMMAND_PATH;
    s_target_x_mm = s_path_points[0].x_mm;
    s_target_y_mm = s_path_points[0].y_mm;
    s_target_heading_rad = normalize_radians(
        final_heading_deg * NAV_PI / 180.0f);
    s_target_tolerance_mm = final_tolerance_mm;
    s_command_scale = speed_scale;
    s_command_timeout_us = (int64_t)NAV_MOVE_TIMEOUT_MS * 1000;
    s_paused = false;
    s_settling = false;
    s_aligning = false;
    s_align_settle_until_us = 0;
    s_phase_started_us = now_us;
    s_reset_pi_requested = true;
    s_pose.command = s_command;
    s_pose.command_id = issued_id;
    s_pose.state = POST_NAV_RUNNING;
    if (command_id) *command_id = issued_id;
    portEXIT_CRITICAL(&s_lock);
    ESP_LOGW(TAG,
             "PATH_COMMAND id=%lu points=%u first=(%d,%d) final=(%d,%d,%ddeg) tolerance=%d scale=%d%%",
             (unsigned long)issued_id, (unsigned)point_count,
             (int)lroundf(points[0].x_mm),
             (int)lroundf(points[0].y_mm),
             (int)lroundf(points[point_count - 1].x_mm),
             (int)lroundf(points[point_count - 1].y_mm),
             (int)lroundf(final_heading_deg),
             (int)lroundf(final_tolerance_mm),
             (int)lroundf(speed_scale * 100.0f));
    return true;
}

bool post_line_navigation_correct_pose(float x_mm, float y_mm,
                                       float heading_deg)
{
    post_line_odometry_pose_t odometry;
    if (!post_line_odometry_get_pose(&odometry)) return false;
    navigation_anchor_t anchor = {
        .field_x_mm = x_mm,
        .field_y_mm = y_mm,
        .field_heading_rad = heading_deg * NAV_PI / 180.0f,
        .odometry = odometry,
    };
    post_line_navigation_pose_t pose = pose_from_odometry(&anchor, &odometry);
    portENTER_CRITICAL(&s_lock);
    s_anchor = anchor;
    s_anchor_revision++;
    pose.state = current_state_locked();
    pose.command = s_command;
    pose.command_id = s_command_id;
    pose.waypoint_index = (int)s_command_id;
    pose.target_x_mm = s_target_x_mm;
    pose.target_y_mm = s_target_y_mm;
    pose.target_heading_deg = s_target_heading_rad * 180.0f / NAV_PI;
    if (s_command == POST_NAV_COMMAND_MOVE ||
        s_command == POST_NAV_COMMAND_PUSH ||
        s_command == POST_NAV_COMMAND_REVERSE ||
        s_command == POST_NAV_COMMAND_PATH) {
        pose.distance_to_target_mm = hypotf(
            pose.target_x_mm - pose.x_mm,
            pose.target_y_mm - pose.y_mm);
    }
    s_pose = pose;
    portEXIT_CRITICAL(&s_lock);
    ESP_LOGW(TAG, "POSE_CORRECTED x=%d y=%d heading=%ddeg",
             (int)lroundf(x_mm), (int)lroundf(y_mm),
             (int)lroundf(heading_deg));
    return true;
}

bool post_line_navigation_get_pose(post_line_navigation_pose_t *pose)
{
    if (!pose) return false;
    portENTER_CRITICAL(&s_lock);
    *pose = s_pose;
    portEXIT_CRITICAL(&s_lock);
    return pose->valid;
}
