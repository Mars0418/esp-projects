#include <stdbool.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ball_vision.h"
#include "black_marker_vision.h"
#include "camera_display.h"
#include "camera_line_follow.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_intr_alloc.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "jpeg_decoder.h"
#include "libuvc/libuvc.h"
#include "libuvc_adapter.h"
#include "libuvc_helper.h"
#include "line_vision.h"
#include "mbedtls/base64.h"
#include "post_line_odometry.h"
#include "post_line_navigation.h"
#include "quarter_goal_pose.h"
#include "usb/usb_host.h"

#define MJPEG_SLOT_COUNT      2
#define FRAME_QUEUE_LENGTH    1
#define DISPLAY_QUEUE_LENGTH  1
#define MJPEG_SLOT_CAPACITY   (256 * 1024)
#define DECODED_WIDTH         160
#define DECODED_HEIGHT        120
#define DECODED_BUFFER_BYTES  (DECODED_WIDTH * DECODED_HEIGHT * 2)
#define JPEG_WORK_BUFFER_BYTES 8192
#define RGB_DEBUG_WIDTH       32
#define RGB_DEBUG_HEIGHT      24
#define RGB_DEBUG_INTERVAL_US 500000
#define TFT_PREVIEW_INTERVAL_US 60000
#define TUNER_INTERVAL_US     500000
#define CALIBRATION_INTERVAL_US 1000000
#define RGB_DEBUG_PIXELS      (RGB_DEBUG_WIDTH * RGB_DEBUG_HEIGHT)
#define RGB_DEBUG_BYTES       (RGB_DEBUG_PIXELS * 2)
#define RGB_DEBUG_MASK_BYTES  ((RGB_DEBUG_PIXELS + 7) / 8)
#define RGB_DEBUG_B64_BYTES   (((RGB_DEBUG_BYTES + 2) / 3) * 4 + 1)
#define MASK_DEBUG_B64_BYTES  (((RGB_DEBUG_MASK_BYTES + 2) / 3) * 4 + 1)
#define TUNER_MASK_BYTES      ((DECODED_WIDTH * DECODED_HEIGHT + 7) / 8)
#define PRECISE_WIDTH         640
#define PRECISE_HEIGHT        480
#define PRECISE_BUFFER_BYTES  (PRECISE_WIDTH * PRECISE_HEIGHT * 2)
#define PRECISE_TRIGGER_CONFIDENCE 70
#define PRECISE_TRIGGER_FRAMES 3
#define GOAL_DETECTION_WINDOW_FRAMES 10
#define GOAL_DETECTION_REQUIRED_FRAMES 8
#define OBJECT_STATUS_INTERVAL_US 500000
#define INITIAL_FIELD_X_MM 470.0f
#define INITIAL_FIELD_Y_MM 650.0f
#define INITIAL_FIELD_HEADING_DEG (-90.0f)
#define PI_F 3.14159265359f

#define MISSION_TRAVEL_SPEED 0.60f
#define MISSION_PUSH_SPEED 0.35f
#define MISSION_BALL_CANDIDATE_FRAMES 2
#define MISSION_BALL_SETTLE_US 250000
#define MISSION_BALL_STABLE_FRAMES 3
#define MISSION_BALL_STABLE_MAX_MM 30.0f
#define MISSION_PREPUSH_STANDOFF_MM 180.0f
#define MISSION_PREPUSH_TOLERANCE_MM 20.0f
#define MISSION_BALL_KEEP_OUT_MM 150.0f
#define MISSION_BALL_ORBIT_RADIUS_MM 240.0f
#define MISSION_BALL_ARC_STEP_RAD 0.5235988f
#define MISSION_APPROACH_REALIGN_RAD 0.3490659f
#define MISSION_MOVE_TOLERANCE_MM 45.0f
#define MISSION_PUSH_TOLERANCE_MM 35.0f
#define MISSION_PRECISE_TIMEOUT_US 6000000
#define MISSION_EXPLORATION_START_FRAMES 10
#define MISSION_RETURN_HEADING_DEG (-45.0f)
#define MISSION_BALL_CENTER_TOLERANCE_PX 10
#define MISSION_PUSH_HEADING_TOLERANCE_DEG 20.0f
#define MISSION_BALL_CENTER_CONFIRM_FRAMES 3
#define MISSION_BALL_CENTER_CONFIRM_TIMEOUT_US 3000000
#define MISSION_RECOVERY_REVERSE_MM 50.0f
#define MISSION_GOAL_CAPTURE_RADIUS_MM 100.0f
#define MISSION_LOST_BALL_FORWARD_MM 80.0f
#define MISSION_VISUAL_PUSH_CENTER_X (DECODED_WIDTH / 2)
#define UPPER_GOAL_X_MM 0.0f
#define UPPER_GOAL_Y_MM 0.0f
#define LOWER_GOAL_X_MM 900.0f
#define LOWER_GOAL_Y_MM 0.0f

/* Temporary high-resolution calibration firmware. Set to 0 after exporting
 * the new camera calibration and copying its parameters into runtime code. */
#define CAMERA_CALIBRATION_ONLY 0
#define CALIBRATION_UART_BAUD 921600

static const gpio_num_t s_motor_safe_stop_pins[] = {
    GPIO_NUM_5,
    GPIO_NUM_6, GPIO_NUM_15, GPIO_NUM_7,
    GPIO_NUM_11, GPIO_NUM_9, GPIO_NUM_10,
    GPIO_NUM_40, GPIO_NUM_42, GPIO_NUM_41,
};

typedef struct {
    uint8_t *data;
    size_t length;
    size_t step;
    uint16_t width;
    uint16_t height;
    enum uvc_frame_format format;
    int64_t captured_at_us;
    bool busy;
} mjpeg_slot_t;

typedef struct {
    uint8_t detected[GOAL_DETECTION_WINDOW_FRAMES];
    int index;
    int count;
    int detected_count;
} goal_detection_window_t;

typedef enum {
    MISSION_BOOT = 0,
    MISSION_SELECT_FIRST_BALL,
    MISSION_BALL_SETTLE,
    MISSION_BALL_CONFIRM,
    MISSION_BALL_STABLE,
    MISSION_MOVE_BEHIND,
    MISSION_FINAL_BALL_SETTLE,
    MISSION_PUSH_PRECISE_LOCALIZE,
    MISSION_PUSH_CENTER_CONFIRM,
    MISSION_PUSH_RECOVERY_REVERSE,
    MISSION_PUSH_RECOVERY_SEARCH,
    MISSION_PUSH_RECOVERY_SETTLE,
    MISSION_PUSH,
    MISSION_RETURN_INITIAL,
    MISSION_RETURN_ALIGN,
    MISSION_SELECT_SECOND_BALL,
    MISSION_DONE,
    MISSION_ERROR,
} ball_capture_mission_state_t;

typedef enum {
    MISSION_BALL_NONE = 0,
    MISSION_BALL_RED,
    MISSION_BALL_WHITE,
} mission_ball_kind_t;

typedef struct {
    ball_capture_mission_state_t state;
    mission_ball_kind_t selected_ball;
    mission_ball_kind_t first_ball;
    int target_goal_index;
    int candidate_frames;
    int missing_ball_frames;
    int ball_center_samples;
    int ball_center_x_sum;
    bool exploring;
    bool push_precise_locked;
    goal_detection_window_t ball_window;
    int stable_ball_frames;
    float last_ball_field_x_mm;
    float last_ball_field_y_mm;
    float ball_field_x_mm;
    float ball_field_y_mm;
    uint32_t command_id;
    int stable_goal_frames;
    int64_t settle_until_us;
    int64_t deadline_us;
} ball_capture_mission_t;

static const char *TAG = "CAMERA_VIEW";
static EventGroupHandle_t s_uvc_events;
static QueueHandle_t s_frame_queue;
static QueueHandle_t s_display_queue;
static mjpeg_slot_t s_slots[MJPEG_SLOT_COUNT];
static uint8_t *s_decoded_frame;
static uint8_t *s_display_frame;
static uint8_t *s_debug_raw_frame;
static uint8_t *s_precise_frame;
static uint8_t *s_jpeg_work_buffer;
static portMUX_TYPE s_slot_lock = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE s_display_lock = portMUX_INITIALIZER_UNLOCKED;
static bool s_display_busy;
static volatile uint32_t s_received_frames;
static volatile uint32_t s_displayed_frames;
static volatile uint32_t s_dropped_frames;
static bool s_capture_core_reported;
static uint8_t s_rgb_debug_pixels[RGB_DEBUG_BYTES];
static uint8_t s_rgb_debug_mask[RGB_DEBUG_MASK_BYTES];
static unsigned char s_rgb_debug_b64[RGB_DEBUG_B64_BYTES];
static unsigned char s_mask_debug_b64[MASK_DEBUG_B64_BYTES];
static char s_rgb_debug_line[2560];
static uint32_t s_rgb_debug_sequence;
static uint8_t *s_tuner_mask;
static char s_tuner_header[256];
static uint32_t s_tuner_sequence;
static char s_calibration_header[96];
static uint32_t s_calibration_sequence;
static int s_stream_width = 640;
static int s_stream_height = 480;
static int s_stream_fps = 15;
static enum uvc_frame_format s_stream_format = UVC_FRAME_FORMAT_MJPEG;
static esp_jpeg_image_scale_t s_decode_scale = JPEG_IMAGE_SCALE_1_4;

static esp_err_t downsample_yuyv_luma(const mjpeg_slot_t *slot);

static esp_err_t initialize_motor_safe_stop(void)
{
    for (size_t index = 0;
         index < sizeof(s_motor_safe_stop_pins) /
                     sizeof(s_motor_safe_stop_pins[0]);
         ++index) {
        const gpio_num_t pin = s_motor_safe_stop_pins[index];
        ESP_RETURN_ON_ERROR(gpio_reset_pin(pin), TAG,
                            "Failed to reset motor GPIO");
        ESP_RETURN_ON_ERROR(gpio_set_level(pin, 0), TAG,
                            "Failed to clear motor GPIO");
        ESP_RETURN_ON_ERROR(gpio_set_direction(pin, GPIO_MODE_OUTPUT), TAG,
                            "Failed to configure motor GPIO");
        ESP_RETURN_ON_ERROR(gpio_set_level(pin, 0), TAG,
                            "Failed to hold motor GPIO low");
    }
    return ESP_OK;
}

static esp_err_t initialize_calibration_uart(void)
{
    const esp_err_t install_error = uart_driver_install(
        UART_NUM_0, 1024, 0, 0, NULL, 0);
    if (install_error != ESP_OK && install_error != ESP_ERR_INVALID_STATE) {
        return install_error;
    }
    uart_wait_tx_done(UART_NUM_0, pdMS_TO_TICKS(200));
    esp_log_level_set("*", ESP_LOG_NONE);
    return uart_set_baudrate(UART_NUM_0, CALIBRATION_UART_BAUD);
}

static float normalize_radians(float radians)
{
    while (radians > PI_F) radians -= 2.0f * PI_F;
    while (radians <= -PI_F) radians += 2.0f * PI_F;
    return radians;
}

static bool update_goal_detection_window(goal_detection_window_t *window,
                                         bool detected)
{
    if (window->count == GOAL_DETECTION_WINDOW_FRAMES) {
        window->detected_count -= window->detected[window->index];
    } else {
        window->count++;
    }
    window->detected[window->index] = detected ? 1 : 0;
    window->detected_count += detected ? 1 : 0;
    window->index = (window->index + 1) % GOAL_DETECTION_WINDOW_FRAMES;
    return window->count == GOAL_DETECTION_WINDOW_FRAMES &&
           window->detected_count >= GOAL_DETECTION_REQUIRED_FRAMES;
}

static float pose_prior_score(const quarter_goal_pose_result_t *candidate,
                              const post_line_navigation_pose_t *prior)
{
    const float position_error = hypotf(
        candidate->robot_field_x_mm - prior->x_mm,
        candidate->robot_field_y_mm - prior->y_mm);
    const float heading_error = fabsf(normalize_radians(
        candidate->robot_heading_deg * PI_F / 180.0f - prior->heading_rad));
    return position_error + heading_error * 150.0f;
}

static bool choose_precise_goal_pose(
    const quarter_goal_pose_result_t *raw_pose,
    quarter_goal_pose_result_t *selected)
{
    if (!raw_pose || !selected || !raw_pose->field_pose_valid) return false;
    quarter_goal_pose_result_t upper = *raw_pose;
    quarter_goal_pose_result_t lower = *raw_pose;
    if (!quarter_goal_pose_assign_goal(&upper, QUARTER_GOAL_UPPER) ||
        !quarter_goal_pose_assign_goal(&lower, QUARTER_GOAL_LOWER)) {
        return false;
    }

    post_line_navigation_pose_t prior;
    if (!post_line_navigation_get_pose(&prior)) {
        *selected = upper;
        return true;
    }
    const float upper_score = pose_prior_score(&upper, &prior);
    const float lower_score = pose_prior_score(&lower, &prior);
    *selected = lower_score < upper_score ? lower : upper;
    ESP_LOGW(TAG,
             "GOAL_SELECT upper_score=%d lower_score=%d selected=%s prior=(%d,%d,%ddeg)",
             (int)lroundf(upper_score), (int)lroundf(lower_score),
             selected->position_goal == QUARTER_GOAL_LOWER
                 ? "LOWER" : "UPPER",
             (int)lroundf(prior.x_mm), (int)lroundf(prior.y_mm),
             (int)lroundf(prior.heading_deg));
    return true;
}

static int scale_precise_coordinate(int coordinate, int precise_size,
                                    int display_size)
{
    if (coordinate < 0) return -1;
    return (coordinate * (display_size - 1) +
            (precise_size - 1) / 2) / (precise_size - 1);
}

static quarter_goal_pose_result_t precise_pose_for_display(
    const quarter_goal_pose_result_t *precise)
{
    quarter_goal_pose_result_t display = *precise;
    display.origin_x_raw = scale_precise_coordinate(
        precise->origin_x_raw, PRECISE_WIDTH, DECODED_WIDTH);
    display.origin_y_raw = scale_precise_coordinate(
        precise->origin_y_raw, PRECISE_HEIGHT, DECODED_HEIGHT);
    display.x_axis_end_x_raw = scale_precise_coordinate(
        precise->x_axis_end_x_raw, PRECISE_WIDTH, DECODED_WIDTH);
    display.x_axis_end_y_raw = scale_precise_coordinate(
        precise->x_axis_end_y_raw, PRECISE_HEIGHT, DECODED_HEIGHT);
    display.y_axis_end_x_raw = scale_precise_coordinate(
        precise->y_axis_end_x_raw, PRECISE_WIDTH, DECODED_WIDTH);
    display.y_axis_end_y_raw = scale_precise_coordinate(
        precise->y_axis_end_y_raw, PRECISE_HEIGHT, DECODED_HEIGHT);
    return display;
}

static const char *mission_state_name(ball_capture_mission_state_t state)
{
    switch (state) {
    case MISSION_BOOT: return "BOOT";
    case MISSION_SELECT_FIRST_BALL: return "SELECT_FIRST_BALL";
    case MISSION_BALL_SETTLE: return "BALL_SETTLE";
    case MISSION_BALL_CONFIRM: return "BALL_CONFIRM";
    case MISSION_BALL_STABLE: return "BALL_STABLE";
    case MISSION_MOVE_BEHIND: return "MOVE_BEHIND";
    case MISSION_FINAL_BALL_SETTLE: return "FINAL_BALL_SETTLE";
    case MISSION_PUSH_PRECISE_LOCALIZE: return "PUSH_PRECISE_LOCALIZE";
    case MISSION_PUSH_CENTER_CONFIRM: return "PUSH_CENTER_CONFIRM";
    case MISSION_PUSH_RECOVERY_REVERSE: return "PUSH_RECOVERY_REVERSE";
    case MISSION_PUSH_RECOVERY_SEARCH: return "PUSH_RECOVERY_SEARCH";
    case MISSION_PUSH_RECOVERY_SETTLE: return "PUSH_RECOVERY_SETTLE";
    case MISSION_PUSH: return "PUSH";
    case MISSION_RETURN_INITIAL: return "RETURN_INITIAL";
    case MISSION_RETURN_ALIGN: return "RETURN_ALIGN";
    case MISSION_SELECT_SECOND_BALL: return "SELECT_SECOND_BALL";
    case MISSION_DONE: return "DONE";
    case MISSION_ERROR: return "ERROR";
    default: return "UNKNOWN";
    }
}

static const ball_vision_result_t *selected_ball_result(
    mission_ball_kind_t kind, const ball_vision_result_t *red,
    const ball_vision_result_t *white)
{
    if (kind == MISSION_BALL_RED) return red;
    if (kind == MISSION_BALL_WHITE) return white;
    return NULL;
}

static bool ball_kind_is_real(
    mission_ball_kind_t kind, const ball_vision_result_t *red,
    const ball_vision_result_t *white)
{
    const ball_vision_result_t *ball =
        selected_ball_result(kind, red, white);
    return ball && ball->found && !ball->predicted;
}

static bool project_ball_to_field(const ball_vision_result_t *ball,
                                  float *field_x_mm, float *field_y_mm)
{
    if (!ball || !ball->found || ball->predicted ||
        !field_x_mm || !field_y_mm) {
        return false;
    }
    float vehicle_right_mm;
    float vehicle_forward_mm;
    const float contact_x = 0.5f * (ball->left + ball->right);
    const float contact_y = (float)ball->bottom;
    if (!quarter_goal_pose_project_ground_pixel(
            contact_x, contact_y, &vehicle_right_mm,
            &vehicle_forward_mm)) {
        return false;
    }
    post_line_navigation_pose_t pose;
    if (!post_line_navigation_get_pose(&pose)) return false;
    const float sine = sinf(pose.heading_rad);
    const float cosine = cosf(pose.heading_rad);
    *field_x_mm = pose.x_mm + sine * vehicle_right_mm +
                  cosine * vehicle_forward_mm;
    *field_y_mm = pose.y_mm - cosine * vehicle_right_mm +
                  sine * vehicle_forward_mm;
    return isfinite(*field_x_mm) && isfinite(*field_y_mm);
}

static mission_ball_kind_t choose_ball_nearest_goal(
    const ball_vision_result_t *red, const ball_vision_result_t *white,
    float goal_x_mm, float goal_y_mm)
{
    float red_x = 0.0f;
    float red_y = 0.0f;
    float white_x = 0.0f;
    float white_y = 0.0f;
    const bool red_valid = red->found && !red->predicted &&
        project_ball_to_field(red, &red_x, &red_y);
    const bool white_valid = white->found && !white->predicted &&
        project_ball_to_field(white, &white_x, &white_y);
    if (red_valid && !white_valid) return MISSION_BALL_RED;
    if (white_valid && !red_valid) return MISSION_BALL_WHITE;
    if (!red_valid && !white_valid) return MISSION_BALL_NONE;

    const float red_distance = hypotf(red_x - goal_x_mm,
                                      red_y - goal_y_mm);
    const float white_distance = hypotf(white_x - goal_x_mm,
                                        white_y - goal_y_mm);
    if (fabsf(red_distance - white_distance) < 20.0f) {
        return red->confidence >= white->confidence
            ? MISSION_BALL_RED : MISSION_BALL_WHITE;
    }
    return red_distance < white_distance
        ? MISSION_BALL_RED : MISSION_BALL_WHITE;
}

static bool mission_command_complete(const ball_capture_mission_t *mission,
                                     post_line_navigation_pose_t *pose)
{
    if (!post_line_navigation_get_pose(pose)) return false;
    return pose->command_id == mission->command_id &&
           pose->state == POST_NAV_COMPLETE;
}

static void mission_fail(ball_capture_mission_t *mission,
                         const char *reason)
{
    if (mission->state == MISSION_ERROR) return;
    mission->state = MISSION_ERROR;
    post_line_navigation_stop();
    ESP_LOGE(TAG, "MISSION_ERROR reason=%s", reason);
}

static void mission_goal_geometry(const ball_capture_mission_t *mission,
                                  float *goal_x_mm, float *goal_y_mm)
{
    const bool lower = mission->target_goal_index == 1;
    *goal_x_mm = lower ? LOWER_GOAL_X_MM : UPPER_GOAL_X_MM;
    *goal_y_mm = lower ? LOWER_GOAL_Y_MM : UPPER_GOAL_Y_MM;
}

static float point_to_segment_distance(
    float point_x, float point_y, float start_x, float start_y,
    float end_x, float end_y)
{
    const float segment_x = end_x - start_x;
    const float segment_y = end_y - start_y;
    const float length_squared =
        segment_x * segment_x + segment_y * segment_y;
    if (length_squared < 1.0f) {
        return hypotf(point_x - start_x, point_y - start_y);
    }
    const float projection = fmaxf(0.0f, fminf(1.0f,
        ((point_x - start_x) * segment_x +
         (point_y - start_y) * segment_y) / length_squared));
    const float closest_x = start_x + projection * segment_x;
    const float closest_y = start_y + projection * segment_y;
    return hypotf(point_x - closest_x, point_y - closest_y);
}

static bool append_approach_point(
    post_line_navigation_waypoint_t *points, size_t *count,
    float x_mm, float y_mm)
{
    if (*count > 0 &&
        hypotf(points[*count - 1].x_mm - x_mm,
               points[*count - 1].y_mm - y_mm) < 10.0f) {
        points[*count - 1].x_mm = x_mm;
        points[*count - 1].y_mm = y_mm;
        return true;
    }
    if (*count >= POST_LINE_NAVIGATION_MAX_PATH_POINTS) return false;
    points[*count] = (post_line_navigation_waypoint_t) {
        .x_mm = x_mm,
        .y_mm = y_mm,
    };
    (*count)++;
    return true;
}

static bool mission_move_behind_ball(ball_capture_mission_t *mission,
                                     bool full_avoidance_route)
{
    float goal_x, goal_y;
    mission_goal_geometry(mission, &goal_x, &goal_y);
    const float dx = goal_x - mission->ball_field_x_mm;
    const float dy = goal_y - mission->ball_field_y_mm;
    const float distance = hypotf(dx, dy);
    if (distance < 50.0f) return false;
    const float push_x = dx / distance;
    const float push_y = dy / distance;
    const float prepush_x = mission->ball_field_x_mm -
        push_x * MISSION_PREPUSH_STANDOFF_MM;
    const float prepush_y = mission->ball_field_y_mm -
        push_y * MISSION_PREPUSH_STANDOFF_MM;
    const float push_heading = atan2f(push_y, push_x);

    post_line_navigation_pose_t pose;
    if (!post_line_navigation_get_pose(&pose)) return false;
    post_line_navigation_waypoint_t
        points[POST_LINE_NAVIGATION_MAX_PATH_POINTS];
    size_t point_count = 0;
    const float current_from_ball_x =
        pose.x_mm - mission->ball_field_x_mm;
    const float current_from_ball_y =
        pose.y_mm - mission->ball_field_y_mm;
    const float current_radius = hypotf(current_from_ball_x,
                                        current_from_ball_y);
    const float direct_clearance = point_to_segment_distance(
        mission->ball_field_x_mm, mission->ball_field_y_mm,
        pose.x_mm, pose.y_mm, prepush_x, prepush_y);
    const float heading_error = normalize_radians(
        push_heading - pose.heading_rad);
    const bool needs_orbit = full_avoidance_route &&
        direct_clearance < MISSION_BALL_KEEP_OUT_MM;
    const bool needs_backoff = !full_avoidance_route &&
        fabsf(heading_error) > MISSION_APPROACH_REALIGN_RAD;

    if (needs_orbit) {
        float current_angle;
        if (current_radius >= 1.0f) {
            current_angle = atan2f(current_from_ball_y,
                                   current_from_ball_x);
            if (fabsf(current_radius - MISSION_BALL_ORBIT_RADIUS_MM) >
                    MISSION_MOVE_TOLERANCE_MM &&
                !append_approach_point(
                    points, &point_count,
                    mission->ball_field_x_mm +
                        cosf(current_angle) * MISSION_BALL_ORBIT_RADIUS_MM,
                    mission->ball_field_y_mm +
                        sinf(current_angle) * MISSION_BALL_ORBIT_RADIUS_MM)) {
                return false;
            }
        } else {
            current_angle = normalize_radians(
                push_heading + PI_F * 0.5f);
        }
        const float behind_angle = normalize_radians(push_heading + PI_F);
        const float arc_angle = normalize_radians(
            behind_angle - current_angle);
        int arc_segments = (int)ceilf(
            fabsf(arc_angle) / MISSION_BALL_ARC_STEP_RAD);
        if (arc_segments < 1) arc_segments = 1;
        if (arc_segments > 6) arc_segments = 6;
        for (int segment = 1; segment <= arc_segments; ++segment) {
            const float angle = current_angle +
                arc_angle * segment / arc_segments;
            if (!append_approach_point(
                    points, &point_count,
                    mission->ball_field_x_mm +
                        cosf(angle) * MISSION_BALL_ORBIT_RADIUS_MM,
                    mission->ball_field_y_mm +
                        sinf(angle) * MISSION_BALL_ORBIT_RADIUS_MM)) {
                return false;
            }
        }
    } else if (full_avoidance_route || needs_backoff) {
        if (!append_approach_point(
                points, &point_count,
                mission->ball_field_x_mm -
                    push_x * MISSION_BALL_ORBIT_RADIUS_MM,
                mission->ball_field_y_mm -
                    push_y * MISSION_BALL_ORBIT_RADIUS_MM)) {
            return false;
        }
    }
    if (!append_approach_point(points, &point_count,
                               prepush_x, prepush_y)) {
        return false;
    }

    ESP_LOGW(TAG,
             "MISSION_BALL_LOCK goal=%s ball=(%d,%d) prepush=(%d,%d) route=%u orbit=%d clearance=%dmm",
             mission->target_goal_index == 0 ? "UPPER" : "LOWER",
             (int)lroundf(mission->ball_field_x_mm),
             (int)lroundf(mission->ball_field_y_mm),
             (int)lroundf(prepush_x), (int)lroundf(prepush_y),
             (unsigned)point_count, needs_orbit,
             (int)lroundf(direct_clearance));
    return post_line_navigation_follow_path(
        points, point_count, push_heading * 180.0f / PI_F,
        MISSION_PREPUSH_TOLERANCE_MM, MISSION_TRAVEL_SPEED,
        &mission->command_id);
}

static bool mission_start_exploration(ball_capture_mission_t *mission)
{
    float goal_x, goal_y;
    mission_goal_geometry(mission, &goal_x, &goal_y);
    post_line_navigation_pose_t pose;
    if (!post_line_navigation_get_pose(&pose)) return false;
    const float heading = atan2f(goal_y - pose.y_mm,
                                 goal_x - pose.x_mm) * 180.0f / PI_F;
    const post_line_navigation_waypoint_t target = {
        .x_mm = goal_x,
        .y_mm = goal_y,
    };
    if (!post_line_navigation_follow_path(
            &target, 1, heading, MISSION_MOVE_TOLERANCE_MM,
            MISSION_PUSH_SPEED, &mission->command_id)) return false;
    ESP_LOGW(TAG,
             "MISSION_EXPLORE goal=%s target=(%d,%d) heading=%ddeg speed=%d%%",
             mission->target_goal_index == 0 ? "UPPER" : "LOWER",
             (int)lroundf(goal_x), (int)lroundf(goal_y),
             (int)lroundf(heading),
             (int)lroundf(MISSION_PUSH_SPEED * 100.0f));
    return true;
}

static bool mission_start_final_push(ball_capture_mission_t *mission)
{
    float goal_x, goal_y;
    mission_goal_geometry(mission, &goal_x, &goal_y);
    const float push_heading = atan2f(
        goal_y - mission->ball_field_y_mm,
        goal_x - mission->ball_field_x_mm) * 180.0f / PI_F;
    if (!post_line_navigation_push_to(
            goal_x, goal_y, push_heading,
            MISSION_PUSH_TOLERANCE_MM, MISSION_PUSH_SPEED,
            &mission->command_id)) {
        return false;
    }
    ESP_LOGW(TAG,
             "MISSION_PUSH goal=%s vertex=(%d,%d) fixed_heading=%ddeg",
             mission->target_goal_index == 0 ? "UPPER" : "LOWER",
             (int)lroundf(goal_x), (int)lroundf(goal_y),
             (int)lroundf(push_heading));
    return true;
}

static bool mission_update_stable_ball(
    ball_capture_mission_t *mission, const ball_vision_result_t *ball,
    bool real_detection)
{
    float ball_x;
    float ball_y;
    if (!real_detection ||
        !project_ball_to_field(ball, &ball_x, &ball_y)) {
        mission->stable_ball_frames = 0;
        return false;
    }

    if (mission->stable_ball_frames == 0 ||
        hypotf(ball_x - mission->last_ball_field_x_mm,
               ball_y - mission->last_ball_field_y_mm) <=
            MISSION_BALL_STABLE_MAX_MM) {
        mission->stable_ball_frames++;
    } else {
        mission->stable_ball_frames = 1;
    }
    mission->last_ball_field_x_mm = ball_x;
    mission->last_ball_field_y_mm = ball_y;
    if (mission->stable_ball_frames < MISSION_BALL_STABLE_FRAMES) {
        return false;
    }
    mission->ball_field_x_mm = ball_x;
    mission->ball_field_y_mm = ball_y;
    return true;
}

static bool queue_tft_preview(const uint8_t *pixels)
{
    bool reserved = false;
    portENTER_CRITICAL(&s_display_lock);
    if (!s_display_busy) {
        s_display_busy = true;
        reserved = true;
    }
    portEXIT_CRITICAL(&s_display_lock);
    if (!reserved) return false;

    memcpy(s_display_frame, pixels, DECODED_BUFFER_BYTES);
    const uint8_t signal = 1;
    if (xQueueSend(s_display_queue, &signal, 0) != pdPASS) {
        portENTER_CRITICAL(&s_display_lock);
        s_display_busy = false;
        portEXIT_CRITICAL(&s_display_lock);
        return false;
    }
    return true;
}

static void tft_display_task(void *argument)
{
    (void)argument;
    uint8_t signal;
    while (true) {
        if (xQueueReceive(s_display_queue, &signal, portMAX_DELAY) != pdPASS) {
            continue;
        }
        const esp_err_t error = camera_display_show_rotated_rgb565(
            s_display_frame, DECODED_WIDTH, DECODED_HEIGHT);
        if (error == ESP_OK) {
            s_displayed_frames++;
        } else {
            ESP_LOGW(TAG, "TFT preview failed: %s", esp_err_to_name(error));
        }
        portENTER_CRITICAL(&s_display_lock);
        s_display_busy = false;
        portEXIT_CRITICAL(&s_display_lock);
    }
}

static void emit_rgb_debug_frame(const uint8_t *raw_pixels,
                                 size_t width, size_t height,
                                 const line_vision_result_t *result)
{
    memset(s_rgb_debug_mask, 0, sizeof(s_rgb_debug_mask));
    for (size_t sample_y = 0; sample_y < RGB_DEBUG_HEIGHT; ++sample_y) {
        const size_t source_y = (sample_y * height + height / 2) /
                                RGB_DEBUG_HEIGHT;
        for (size_t sample_x = 0; sample_x < RGB_DEBUG_WIDTH; ++sample_x) {
            const size_t source_x = (sample_x * width + width / 2) /
                                    RGB_DEBUG_WIDTH;
            const size_t source_index = source_y * width + source_x;
            const size_t sample_index = sample_y * RGB_DEBUG_WIDTH + sample_x;
            s_rgb_debug_pixels[sample_index * 2] =
                raw_pixels[source_index * 2];
            s_rgb_debug_pixels[sample_index * 2 + 1] =
                raw_pixels[source_index * 2 + 1];
            if (line_vision_pixel_selected(source_index)) {
                s_rgb_debug_mask[sample_index / 8] |=
                    (uint8_t)(1U << (sample_index % 8));
            }
        }
    }

    size_t rgb_length = 0;
    size_t mask_length = 0;
    if (mbedtls_base64_encode(s_rgb_debug_b64,
                              sizeof(s_rgb_debug_b64),
                              &rgb_length, s_rgb_debug_pixels,
                              sizeof(s_rgb_debug_pixels)) != 0 ||
        mbedtls_base64_encode(s_mask_debug_b64,
                              sizeof(s_mask_debug_b64),
                              &mask_length, s_rgb_debug_mask,
                              sizeof(s_rgb_debug_mask)) != 0) {
        return;
    }
    s_rgb_debug_b64[rgb_length] = '\0';
    s_mask_debug_b64[mask_length] = '\0';
    const line_vision_rgb_thresholds_t thresholds =
        line_vision_get_rgb_thresholds();
    const int line_length = snprintf(
        s_rgb_debug_line, sizeof(s_rgb_debug_line),
        "@RGB,%lu,%d,%d,%u,%u,%u,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%s,%s\n",
        (unsigned long)++s_rgb_debug_sequence,
        RGB_DEBUG_WIDTH, RGB_DEBUG_HEIGHT,
        thresholds.red, thresholds.green, thresholds.blue,
        result->found, result->confidence, result->steering_error,
        result->near_x, result->far_x, result->big_turn,
        result->turn_direction, result->turn_angle_deg,
        result->turn_confidence, result->corner_y,
        s_rgb_debug_b64, s_mask_debug_b64);
    if (line_length > 0 && line_length < (int)sizeof(s_rgb_debug_line)) {
        uart_write_bytes(UART_NUM_0, s_rgb_debug_line, line_length);
    }
}

static void emit_tuner_frame(const uint8_t *raw_pixels,
                             size_t width, size_t height,
                             const line_vision_result_t *result)
{
    const size_t pixel_count = width * height;
    const size_t mask_bytes = (pixel_count + 7) / 8;
    if (width != DECODED_WIDTH || height != DECODED_HEIGHT ||
        mask_bytes > TUNER_MASK_BYTES) {
        return;
    }

    memset(s_tuner_mask, 0, mask_bytes);
    for (size_t index = 0; index < pixel_count; ++index) {
        if (line_vision_pixel_selected(index)) {
            s_tuner_mask[index / 8] |= (uint8_t)(1U << (index % 8));
        }
    }

    const line_vision_rgb_thresholds_t thresholds =
        line_vision_get_rgb_thresholds();
    const int header_length = snprintf(
        s_tuner_header, sizeof(s_tuner_header),
        "@RGB565,%lu,%u,%u,%u,%u,%u,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%u,%u\n",
        (unsigned long)++s_tuner_sequence,
        (unsigned)width, (unsigned)height,
        thresholds.red, thresholds.green, thresholds.blue,
        result->found, result->confidence, result->steering_error,
        result->near_x, result->far_x, result->big_turn,
        result->turn_direction, result->turn_angle_deg,
        result->turn_confidence, result->corner_y,
        (unsigned)(pixel_count * 2), (unsigned)mask_bytes);
    if (header_length <= 0 || header_length >= (int)sizeof(s_tuner_header)) {
        return;
    }

    uart_write_bytes(UART_NUM_0, s_tuner_header, header_length);
    uart_write_bytes(UART_NUM_0, raw_pixels, pixel_count * 2);
    uart_write_bytes(UART_NUM_0, s_tuner_mask, mask_bytes);
}

static void emit_calibration_jpeg(const mjpeg_slot_t *slot)
{
    if (slot->format != UVC_FRAME_FORMAT_MJPEG || slot->length < 4) {
        return;
    }
    const int header_length = snprintf(
        s_calibration_header, sizeof(s_calibration_header),
        "@CALJPEG,%lu,%u,%u,%u\n",
        (unsigned long)++s_calibration_sequence,
        (unsigned)slot->width, (unsigned)slot->height,
        (unsigned)slot->length);
    if (header_length <= 0 ||
        header_length >= (int)sizeof(s_calibration_header)) {
        return;
    }
    uart_write_bytes(UART_NUM_0, s_calibration_header, header_length);
    uart_write_bytes(UART_NUM_0, slot->data, slot->length);
}

static void usb_library_task(void *argument)
{
    (void)argument;
    while (true) {
        uint32_t event_flags = 0;
        const esp_err_t error = usb_host_lib_handle_events(portMAX_DELAY,
                                                           &event_flags);
        if (error != ESP_OK) {
            ESP_LOGE(TAG, "USB library event error: %s",
                     esp_err_to_name(error));
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            usb_host_device_free_all();
        }
    }
}

static esp_err_t initialize_usb_host(void)
{
    const usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
        .peripheral_map = BIT0,
    };
    ESP_RETURN_ON_ERROR(usb_host_install(&host_config), TAG,
                        "USB host install failed");
    if (xTaskCreatePinnedToCore(usb_library_task, "usb_events", 4096, NULL,
                                2, NULL, 0) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

static void libuvc_event_callback(libuvc_adapter_event_t event)
{
    xEventGroupSetBits(s_uvc_events, event);
}

static void wait_for_uvc_event(EventBits_t event)
{
    xEventGroupWaitBits(s_uvc_events, event, pdTRUE, pdFALSE, portMAX_DELAY);
}

static int reserve_mjpeg_slot(void)
{
    int result = -1;
    portENTER_CRITICAL(&s_slot_lock);
    for (int index = 0; index < MJPEG_SLOT_COUNT; ++index) {
        if (!s_slots[index].busy) {
            s_slots[index].busy = true;
            result = index;
            break;
        }
    }
    portEXIT_CRITICAL(&s_slot_lock);
    return result;
}

static void release_mjpeg_slot(int index)
{
    portENTER_CRITICAL(&s_slot_lock);
    s_slots[index].busy = false;
    portEXIT_CRITICAL(&s_slot_lock);
}

static void camera_frame_callback(uvc_frame_t *frame, void *user_pointer)
{
    (void)user_pointer;
    if (!s_capture_core_reported) {
        ESP_LOGI(TAG, "CAPTURE_CALLBACK core=%d", xPortGetCoreID());
        s_capture_core_reported = true;
    }
    s_received_frames++;

    const uint8_t *frame_bytes = frame->data;
    size_t frame_length = frame->data_bytes;
    if (frame->frame_format == UVC_FRAME_FORMAT_MJPEG) {
        while (frame_length >= 2 &&
               !(frame_bytes[frame_length - 2] == 0xff &&
                 frame_bytes[frame_length - 1] == 0xd9)) {
            frame_length--;
        }
        if (frame_length < 4 || frame_bytes[0] != 0xff ||
            frame_bytes[1] != 0xd8) {
            s_dropped_frames++;
            return;
        }
    } else if (frame->frame_format == UVC_FRAME_FORMAT_YUYV) {
        if (frame->step < frame->width * 2 ||
            frame_length < frame->step * frame->height) {
            s_dropped_frames++;
            return;
        }
        frame_length = frame->step * frame->height;
    } else {
        s_dropped_frames++;
        return;
    }
    if (frame_length > MJPEG_SLOT_CAPACITY) {
        s_dropped_frames++;
        return;
    }

    int slot_index = reserve_mjpeg_slot();
    if (slot_index < 0) {
        /* The decoder owns one slot and the queue owns the other. Replace the
         * queued frame so control never waits behind stale camera data. */
        int stale_slot;
        if (xQueueReceive(s_frame_queue, &stale_slot, 0) == pdPASS) {
            release_mjpeg_slot(stale_slot);
            s_dropped_frames++;
            slot_index = reserve_mjpeg_slot();
        }
    }
    if (slot_index < 0) {
        s_dropped_frames++;
        return;
    }

    memcpy(s_slots[slot_index].data, frame->data, frame_length);
    s_slots[slot_index].length = frame_length;
    s_slots[slot_index].step = frame->step;
    s_slots[slot_index].width = frame->width;
    s_slots[slot_index].height = frame->height;
    s_slots[slot_index].format = frame->frame_format;
    s_slots[slot_index].captured_at_us = esp_timer_get_time();
    if (xQueueSend(s_frame_queue, &slot_index, 0) != pdPASS) {
        release_mjpeg_slot(slot_index);
        s_dropped_frames++;
    }
}

#if 0
static void frame_display_task(void *argument)
{
    (void)argument;
    ESP_LOGI(TAG, "DECODE_VISION_TASK core=%d", xPortGetCoreID());
    int slot_index;
    uint32_t processed_frames = 0;
    uint32_t last_received_report = 0;
    uint32_t last_processed_report = 0;
    uint32_t last_displayed_report = 0;
    uint64_t decode_time_us = 0;
    uint64_t vision_time_us = 0;
    uint32_t timed_frames = 0;
    int64_t last_report_us = esp_timer_get_time();
    int64_t last_rgb_debug_us = 0;
    int64_t last_tuner_us = 0;
    int64_t last_calibration_us = 0;
    int64_t last_tft_preview_us = 0;
    int64_t latest_frame_age_ms = 0;
    int precise_trigger_frames = 0;
    bool pose_locked = false;
    int64_t last_locked_pose_report_us = 0;
    visual_pose_anchor_t pose_anchor = {0};

    while (true) {
        if (xQueueReceive(s_frame_queue, &slot_index, portMAX_DELAY) != pdPASS) {
            continue;
        }
        const int64_t captured_at_us = s_slots[slot_index].captured_at_us;

        if (CAMERA_CALIBRATION_ONLY) {
            const int64_t calibration_now_us = esp_timer_get_time();
            if (s_slots[slot_index].format == UVC_FRAME_FORMAT_MJPEG &&
                s_slots[slot_index].width == PRECISE_WIDTH &&
                s_slots[slot_index].height == PRECISE_HEIGHT &&
                calibration_now_us - last_calibration_us >=
                    CALIBRATION_INTERVAL_US) {
                emit_calibration_jpeg(&s_slots[slot_index]);
                last_calibration_us = esp_timer_get_time();
            }
            release_mjpeg_slot(slot_index);
            vTaskDelay(1);
            continue;
        }

        if (pose_locked) {
            release_mjpeg_slot(slot_index);
            const int64_t now_us = esp_timer_get_time();
            if (now_us - last_locked_pose_report_us >=
                    LOCKED_POSE_REPORT_INTERVAL_US) {
                float x_mm;
                float y_mm;
                float heading_deg;
                if (anchored_odometry_pose(&pose_anchor, &x_mm, &y_mm,
                                            &heading_deg)) {
                    ESP_LOGI(TAG,
                             "POSE source=ODOM x_mm=%d y_mm=%d heading_deg=%d",
                             (int)lroundf(x_mm), (int)lroundf(y_mm),
                             (int)lroundf(heading_deg));
                }
                last_locked_pose_report_us = now_us;
            }
            vTaskDelay(1);
            continue;
        }

        if (camera_line_follow_calibration_enabled()) {
            const int64_t calibration_now_us = esp_timer_get_time();
            if (s_slots[slot_index].format == UVC_FRAME_FORMAT_MJPEG &&
                calibration_now_us - last_calibration_us >=
                    CALIBRATION_INTERVAL_US) {
                emit_calibration_jpeg(&s_slots[slot_index]);
                last_calibration_us = esp_timer_get_time();
            }
            release_mjpeg_slot(slot_index);
            vTaskDelay(1);
            continue;
        }

        const int64_t decode_start_us = esp_timer_get_time();
        esp_jpeg_image_output_t output = {0};
        esp_err_t decode_error;
        if (s_slots[slot_index].format == UVC_FRAME_FORMAT_YUYV) {
            decode_error = downsample_yuyv_luma(&s_slots[slot_index]);
            output.width = DECODED_WIDTH;
            output.height = DECODED_HEIGHT;
        } else {
            esp_jpeg_image_cfg_t jpeg_config = {
                .indata = s_slots[slot_index].data,
                .indata_size = s_slots[slot_index].length,
                .outbuf = s_decoded_frame,
                .outbuf_size = DECODED_BUFFER_BYTES,
                .out_format = JPEG_IMAGE_FORMAT_RGB565,
                .out_scale = s_decode_scale,
                .flags = {
                    .swap_color_bytes = 1,
                },
                .advanced = {
                    .working_buffer = s_jpeg_work_buffer,
                    .working_buffer_size = JPEG_WORK_BUFFER_BYTES,
                },
            };
            decode_error = esp_jpeg_decode(&jpeg_config, &output);
        }
        if (decode_error == ESP_OK && output.width == DECODED_WIDTH &&
            output.height == DECODED_HEIGHT) {
            const int64_t decode_done_us = esp_timer_get_time();
            const bool rgb_debug = camera_line_follow_debug_enabled();
            const bool tuner = camera_line_follow_tuner_enabled();
            if (rgb_debug || tuner) {
                memcpy(s_debug_raw_frame, s_decoded_frame,
                       DECODED_BUFFER_BYTES);
            }
            black_marker_result_t marker_result;
            quarter_goal_pose_result_t pose_result;
            black_marker_vision_process(s_decoded_frame, output.width,
                                        output.height, &marker_result);
            quarter_goal_pose_process(s_decoded_frame, output.width,
                                      output.height, &marker_result,
                                      &pose_result);
            quarter_goal_pose_draw_overlay(s_decoded_frame, output.width,
                                           output.height, &pose_result);
            const bool trigger_candidate =
                pose_result.found && !marker_result.predicted &&
                pose_result.confidence >= PRECISE_TRIGGER_CONFIDENCE;
            if (trigger_candidate) {
                if (precise_trigger_frames < PRECISE_TRIGGER_FRAMES) {
                    precise_trigger_frames++;
                }
            } else {
                precise_trigger_frames = 0;
            }
            ESP_LOGI(TAG,
                     "CORNER_SEARCH found=%d confidence=%d stable=%d/%d",
                     pose_result.found, pose_result.confidence,
                     precise_trigger_frames, PRECISE_TRIGGER_FRAMES);

            if (precise_trigger_frames >= PRECISE_TRIGGER_FRAMES) {
                precise_trigger_frames = 0;
                if (s_slots[slot_index].format == UVC_FRAME_FORMAT_MJPEG &&
                    s_slots[slot_index].width == PRECISE_WIDTH &&
                    s_slots[slot_index].height == PRECISE_HEIGHT) {
                    post_line_odometry_pose_t anchor_odometry;
                    esp_jpeg_image_output_t precise_output = {0};
                    esp_jpeg_image_cfg_t precise_config = {
                        .indata = s_slots[slot_index].data,
                        .indata_size = s_slots[slot_index].length,
                        .outbuf = s_precise_frame,
                        .outbuf_size = PRECISE_BUFFER_BYTES,
                        .out_format = JPEG_IMAGE_FORMAT_RGB565,
                        .out_scale = JPEG_IMAGE_SCALE_0,
                        .flags = {
                            .swap_color_bytes = 1,
                        },
                        .advanced = {
                            .working_buffer = s_jpeg_work_buffer,
                            .working_buffer_size = JPEG_WORK_BUFFER_BYTES,
                        },
                    };
                    ESP_LOGW(TAG,
                             "PRECISE_POSE decoding one 640x480 frame; keep car still");
                    const bool odometry_ready =
                        post_line_odometry_get_pose(&anchor_odometry);
                    const esp_err_t precise_decode_error =
                        esp_jpeg_decode(&precise_config, &precise_output);
                    quarter_goal_pose_result_t precise_pose;
                    const bool precise_valid =
                        precise_decode_error == ESP_OK &&
                        precise_output.width == PRECISE_WIDTH &&
                        precise_output.height == PRECISE_HEIGHT &&
                        odometry_ready &&
                        quarter_goal_pose_refine_single_corner(
                            s_precise_frame, PRECISE_WIDTH, PRECISE_HEIGHT,
                            &marker_result, &pose_result, &precise_pose);
                    if (precise_valid) {
                        pose_anchor = (visual_pose_anchor_t) {
                            .valid = true,
                            .visual_x_mm = precise_pose.robot_field_x_mm,
                            .visual_y_mm = precise_pose.robot_field_y_mm,
                            .visual_heading_rad =
                                precise_pose.robot_heading_deg * PI_F / 180.0f,
                            .odometry = anchor_odometry,
                        };
                        pose_locked = true;
                        const quarter_goal_pose_result_t display_pose =
                            precise_pose_for_display(&precise_pose);
                        quarter_goal_pose_draw_overlay(
                            s_decoded_frame, output.width, output.height,
                            &display_pose);
                        ESP_LOGW(TAG,
                                 "VISION_LOCK x_mm=%d y_mm=%d heading_deg=%d confidence=%d corner_px_640=(%d,%d) radius_mm=%d",
                                 (int)lroundf(precise_pose.robot_field_x_mm),
                                 (int)lroundf(precise_pose.robot_field_y_mm),
                                 (int)lroundf(precise_pose.robot_heading_deg),
                                 precise_pose.confidence,
                                 precise_pose.origin_x_raw,
                                 precise_pose.origin_y_raw,
                                 (int)lroundf(precise_pose.fitted_radius_mm));
                    } else {
                        ESP_LOGW(TAG,
                                 "PRECISE_POSE rejected decode=%s output=%ux%u; returning to search",
                                 esp_err_to_name(precise_decode_error),
                                 precise_output.width, precise_output.height);
                    }
                } else {
                    ESP_LOGW(TAG,
                             "PRECISE_POSE requires 640x480 MJPEG; current=%ux%u format=%d",
                             s_slots[slot_index].width,
                             s_slots[slot_index].height,
                             s_slots[slot_index].format);
                }
            }
            const int64_t vision_done_us = esp_timer_get_time();
            decode_time_us += (uint64_t)(decode_done_us - decode_start_us);
            vision_time_us += (uint64_t)(vision_done_us - decode_done_us);
            timed_frames++;
            /* Single-corner camera-pose test only. No line-control result is
             * submitted, so the chassis stays stopped. */
            processed_frames++;
            const int64_t debug_now_us = esp_timer_get_time();
            if (!tuner && debug_now_us - last_tft_preview_us >=
                    TFT_PREVIEW_INTERVAL_US) {
                if (queue_tft_preview(s_decoded_frame)) {
                    last_tft_preview_us = debug_now_us;
                }
            }
            if (tuner &&
                debug_now_us - last_tuner_us >= TUNER_INTERVAL_US) {
                const line_vision_result_t empty_result = {0};
                emit_tuner_frame(s_debug_raw_frame, output.width,
                                 output.height, &empty_result);
                last_tuner_us = debug_now_us;
            } else if (rgb_debug &&
                debug_now_us - last_rgb_debug_us >= RGB_DEBUG_INTERVAL_US) {
                const line_vision_result_t empty_result = {0};
                emit_rgb_debug_frame(s_debug_raw_frame, output.width,
                                     output.height, &empty_result);
                last_rgb_debug_us = debug_now_us;
            }
        } else {
            ESP_LOGW(TAG, "Frame conversion failed: %s, output=%ux%u",
                     esp_err_to_name(decode_error), output.width, output.height);
        }
        release_mjpeg_slot(slot_index);

        const int64_t now_us = esp_timer_get_time();
        latest_frame_age_ms = (now_us - captured_at_us) / 1000;
        if (now_us - last_report_us >= 3000000) {
            const uint32_t received_now = s_received_frames;
            const uint32_t displayed_now = s_displayed_frames;
            const uint32_t dropped_now = s_dropped_frames;
            const uint64_t elapsed_us = (uint64_t)(now_us - last_report_us);
            const uint32_t rx_fps_x10 = (uint32_t)(
                (uint64_t)(received_now - last_received_report) *
                10000000ULL / elapsed_us);
            const uint32_t processed_fps_x10 = (uint32_t)(
                (uint64_t)(processed_frames - last_processed_report) *
                10000000ULL / elapsed_us);
            const uint32_t lcd_fps_x10 = (uint32_t)(
                (uint64_t)(displayed_now - last_displayed_report) *
                10000000ULL / elapsed_us);
            const uint32_t decode_average_us = timed_frames > 0
                                                   ? decode_time_us / timed_frames
                                                   : 0;
            const uint32_t vision_average_us = timed_frames > 0
                                                   ? vision_time_us / timed_frames
                                                   : 0;
            ESP_LOGI(TAG,
                     "VIDEO fps=%lu.%lu/%lu.%lu/%lu.%lu drop=%lu age=%lldms cost=%lu/%luus",
                     (unsigned long)(rx_fps_x10 / 10),
                     (unsigned long)(rx_fps_x10 % 10),
                     (unsigned long)(processed_fps_x10 / 10),
                     (unsigned long)(processed_fps_x10 % 10),
                     (unsigned long)(lcd_fps_x10 / 10),
                     (unsigned long)(lcd_fps_x10 % 10),
                     (unsigned long)dropped_now,
                     (long long)latest_frame_age_ms,
                     (unsigned long)decode_average_us,
                     (unsigned long)vision_average_us);
            last_received_report = received_now;
            last_processed_report = processed_frames;
            last_displayed_report = displayed_now;
            decode_time_us = 0;
            vision_time_us = 0;
            timed_frames = 0;
            last_report_us = now_us;
        }
        /* 160x120 JPEG decoding can keep CPU1 busy frame after frame.  Yield
         * once per frame so IDLE1 can reset the task watchdog during a long
         * ball-tracking test. */
        vTaskDelay(1);
    }
}
#endif

static bool refine_precise_goal_pose(
    const mjpeg_slot_t *slot, const black_marker_result_t *coarse_marker,
    const quarter_goal_pose_result_t *coarse_pose,
    quarter_goal_identity_t forced_identity,
    quarter_goal_pose_result_t *selected_pose)
{
    if (slot->format != UVC_FRAME_FORMAT_MJPEG ||
        slot->width != PRECISE_WIDTH || slot->height != PRECISE_HEIGHT) {
        ESP_LOGW(TAG,
                 "PRECISE_POSE requires 640x480 MJPEG; current=%ux%u format=%d",
                 slot->width, slot->height, slot->format);
        return false;
    }

    esp_jpeg_image_output_t output = {0};
    esp_jpeg_image_cfg_t config = {
        .indata = slot->data,
        .indata_size = slot->length,
        .outbuf = s_precise_frame,
        .outbuf_size = PRECISE_BUFFER_BYTES,
        .out_format = JPEG_IMAGE_FORMAT_RGB565,
        .out_scale = JPEG_IMAGE_SCALE_0,
        .flags = {
            .swap_color_bytes = 1,
        },
        .advanced = {
            .working_buffer = s_jpeg_work_buffer,
            .working_buffer_size = JPEG_WORK_BUFFER_BYTES,
        },
    };
    ESP_LOGW(TAG, "PRECISE_POSE decoding one 640x480 frame; chassis stopped");
    const esp_err_t decode_error = esp_jpeg_decode(&config, &output);
    quarter_goal_pose_result_t raw_pose;
    bool valid = decode_error == ESP_OK &&
        output.width == PRECISE_WIDTH && output.height == PRECISE_HEIGHT &&
        quarter_goal_pose_refine_single_corner(
            s_precise_frame, PRECISE_WIDTH, PRECISE_HEIGHT,
            coarse_marker, coarse_pose, &raw_pose);
    if (valid) {
        if (forced_identity == QUARTER_GOAL_UNKNOWN) {
            valid = choose_precise_goal_pose(&raw_pose, selected_pose);
        } else {
            *selected_pose = raw_pose;
            valid = quarter_goal_pose_assign_goal(selected_pose,
                                                  forced_identity);
        }
    }
    if (!valid) {
        ESP_LOGW(TAG,
                 "PRECISE_POSE rejected decode=%s output=%ux%u; keep stopped and retry",
                 esp_err_to_name(decode_error), output.width, output.height);
    }
    return valid;
}

static void process_ball_capture_mission(
    ball_capture_mission_t *mission, const mjpeg_slot_t *slot,
    const ball_vision_result_t *red_ball,
    const ball_vision_result_t *white_ball,
    const black_marker_result_t *marker,
    const quarter_goal_pose_result_t *goal_pose, int64_t now_us)
{
    post_line_navigation_pose_t nav_pose;
    const bool nav_pose_valid = post_line_navigation_get_pose(&nav_pose);
    if (nav_pose_valid &&
        nav_pose.state == POST_NAV_STOPPED &&
        mission->state != MISSION_DONE &&
        mission->state != MISSION_ERROR) {
        mission_fail(mission, "navigation stopped or timed out");
        return;
    }

    if (mission->state == MISSION_BOOT) {
        if (!nav_pose_valid || nav_pose.state == POST_NAV_WAITING) {
            return;
        }
        mission->target_goal_index = 0;
        mission->state = MISSION_SELECT_FIRST_BALL;
        mission->selected_ball = MISSION_BALL_NONE;
        mission->missing_ball_frames = 0;
        mission->exploring = false;
        ESP_LOGW(TAG,
                 "MISSION_SELECT_FIRST_BALL stationary; one visible ball is sufficient");
        return;
    }

    if (mission->state == MISSION_SELECT_FIRST_BALL ||
        mission->state == MISSION_SELECT_SECOND_BALL) {
        const mission_ball_kind_t candidate =
            mission->state == MISSION_SELECT_FIRST_BALL
                ? choose_ball_nearest_goal(
                      red_ball, white_ball, UPPER_GOAL_X_MM,
                      UPPER_GOAL_Y_MM)
                : (ball_kind_is_real(mission->selected_ball,
                                     red_ball, white_ball)
                       ? mission->selected_ball : MISSION_BALL_NONE);
        if (candidate == MISSION_BALL_NONE) {
            mission->candidate_frames = 0;
            if (mission->state == MISSION_SELECT_FIRST_BALL) {
                mission->selected_ball = MISSION_BALL_NONE;
            }
            if (mission->missing_ball_frames <
                    MISSION_EXPLORATION_START_FRAMES) {
                mission->missing_ball_frames++;
            }
            if (!mission->exploring &&
                mission->missing_ball_frames >=
                    MISSION_EXPLORATION_START_FRAMES) {
                if (!mission_start_exploration(mission)) {
                    mission_fail(mission, "cannot start ball exploration");
                    return;
                }
                mission->exploring = true;
            }
            return;
        }
        mission->missing_ball_frames = 0;
        if (candidate == mission->selected_ball) {
            mission->candidate_frames++;
        } else {
            mission->selected_ball = candidate;
            mission->candidate_frames = 1;
        }
        if (mission->candidate_frames >= MISSION_BALL_CANDIDATE_FRAMES) {
            post_line_navigation_pause();
            mission->state = MISSION_BALL_SETTLE;
            mission->settle_until_us = now_us + MISSION_BALL_SETTLE_US;
            memset(&mission->ball_window, 0, sizeof(mission->ball_window));
            ESP_LOGW(TAG,
                     "MISSION_BALL_CANDIDATE color=%s mode=%s; braking",
                     mission->selected_ball == MISSION_BALL_RED
                         ? "RED" : "WHITE",
                     mission->target_goal_index == 0
                         ? "NEAREST_UPPER_GOAL" : "REMAINING_BALL");
        }
        return;
    }

    const ball_vision_result_t *selected = selected_ball_result(
        mission->selected_ball, red_ball, white_ball);
    const bool selected_real = selected && selected->found &&
                               !selected->predicted;

    if (mission->state == MISSION_BALL_SETTLE) {
        if (now_us >= mission->settle_until_us) {
            memset(&mission->ball_window, 0, sizeof(mission->ball_window));
            mission->state = MISSION_BALL_CONFIRM;
            ESP_LOGW(TAG, "MISSION_BALL_CONFIRM begin stationary 8/10");
        }
        return;
    }

    if (mission->state == MISSION_BALL_CONFIRM) {
        const bool confirmed = update_goal_detection_window(
            &mission->ball_window, selected_real);
        if (confirmed) {
            if (mission->target_goal_index == 0) {
                mission->first_ball = mission->selected_ball;
            }
            mission->push_precise_locked = false;
            mission->stable_ball_frames = 0;
            mission->state = MISSION_BALL_STABLE;
            ESP_LOGW(TAG, "MISSION_BALL_CONFIRM passed=%d/%d",
                     mission->ball_window.detected_count,
                     mission->ball_window.count);
        } else if (mission->ball_window.count ==
                   GOAL_DETECTION_WINDOW_FRAMES) {
            post_line_navigation_resume();
            mission->candidate_frames = 0;
            if (mission->target_goal_index == 0) {
                mission->selected_ball = MISSION_BALL_NONE;
                mission->state = MISSION_SELECT_FIRST_BALL;
            } else {
                mission->selected_ball = mission->first_ball ==
                        MISSION_BALL_RED
                    ? MISSION_BALL_WHITE : MISSION_BALL_RED;
                mission->state = MISSION_SELECT_SECOND_BALL;
            }
            ESP_LOGW(TAG, "MISSION_BALL_CONFIRM rejected; resume search");
        }
        return;
    }

    if (mission->state == MISSION_BALL_STABLE) {
        if (mission_update_stable_ball(mission, selected, selected_real)) {
            if (!mission_move_behind_ball(mission, true)) {
                mission_fail(mission, "invalid ball geometry or move command");
                return;
            }
            mission->exploring = false;
            mission->state = MISSION_MOVE_BEHIND;
        }
        return;
    }

    if (mission->state == MISSION_MOVE_BEHIND) {
        if (!mission_command_complete(mission, &nav_pose)) return;
        mission->stable_ball_frames = 0;
        mission->settle_until_us = now_us + MISSION_BALL_SETTLE_US;
        mission->deadline_us = now_us + MISSION_PRECISE_TIMEOUT_US;
        mission->state = MISSION_FINAL_BALL_SETTLE;
        ESP_LOGW(TAG,
                 "MISSION_PUSH_READY reached ball rear; waiting for stationary ball");
        return;
    }

    if (mission->state == MISSION_FINAL_BALL_SETTLE) {
        if (now_us < mission->settle_until_us) return;
        if (now_us >= mission->deadline_us) {
            mission_fail(mission, "ball lost before push");
            return;
        }
        if (!mission_update_stable_ball(mission, selected, selected_real)) {
            return;
        }
        if (!mission_start_final_push(mission)) {
            mission_fail(mission, "cannot start push");
            return;
        }
        memset(&mission->ball_window, 0, sizeof(mission->ball_window));
        mission->state = MISSION_PUSH;
        return;
    }

    if (mission->state == MISSION_PUSH_PRECISE_LOCALIZE) {
        if (now_us < mission->settle_until_us) return;
        if (now_us >= mission->deadline_us) {
            mission->stable_goal_frames = 0;
            memset(&mission->ball_window, 0, sizeof(mission->ball_window));
            post_line_navigation_resume();
            mission->state = MISSION_PUSH;
            ESP_LOGW(TAG,
                     "MISSION_PUSH_PRECISE_RETRY goal=%s source=ODOMETRY; resume push and wait for next 8/10 trigger",
                     mission->target_goal_index == 0 ? "UPPER" : "LOWER");
            return;
        }
        const bool stable_goal = marker->found && !marker->predicted &&
            goal_pose->found &&
            goal_pose->confidence >= PRECISE_TRIGGER_CONFIDENCE;
        mission->stable_goal_frames = stable_goal
            ? mission->stable_goal_frames + 1 : 0;
        if (mission->stable_goal_frames < PRECISE_TRIGGER_FRAMES) return;

        quarter_goal_pose_result_t precise_pose;
        const quarter_goal_identity_t identity =
            mission->target_goal_index == 0
                ? QUARTER_GOAL_UPPER : QUARTER_GOAL_LOWER;
        if (!refine_precise_goal_pose(slot, marker, goal_pose, identity,
                                      &precise_pose) ||
            !post_line_navigation_correct_pose(
                precise_pose.robot_field_x_mm,
                precise_pose.robot_field_y_mm,
                precise_pose.robot_heading_deg)) {
            mission->stable_goal_frames = 0;
            return;
        }
        const quarter_goal_pose_result_t display_pose =
            precise_pose_for_display(&precise_pose);
        quarter_goal_pose_draw_overlay(s_decoded_frame, DECODED_WIDTH,
                                       DECODED_HEIGHT, &display_pose);

        mission->push_precise_locked = true;
        mission->ball_center_samples = 0;
        mission->ball_center_x_sum = 0;
        mission->deadline_us = now_us +
            MISSION_BALL_CENTER_CONFIRM_TIMEOUT_US;
        memset(&mission->ball_window, 0, sizeof(mission->ball_window));
        ESP_LOGW(TAG,
                 "MISSION_PUSH_PRECISE_LOCK goal=%s mode=%s pose=(%d,%d,%ddeg); confirm ball center",
                 mission->target_goal_index == 0 ? "UPPER" : "LOWER",
                 mission->target_goal_index == 0
                     ? "ESTABLISH_UPPER_ORIGIN"
                     : "CORRECT_FROM_KNOWN_LOWER",
                 (int)lroundf(precise_pose.robot_field_x_mm),
                 (int)lroundf(precise_pose.robot_field_y_mm),
                 (int)lroundf(precise_pose.robot_heading_deg));
        mission->state = MISSION_PUSH_CENTER_CONFIRM;
        return;
    }

    if (mission->state == MISSION_PUSH_CENTER_CONFIRM) {
        if (selected_real &&
            mission->ball_center_samples <
                MISSION_BALL_CENTER_CONFIRM_FRAMES) {
            mission->ball_center_x_sum += selected->center_x;
            mission->ball_center_samples++;
        }
        const bool enough_samples = mission->ball_center_samples >=
            MISSION_BALL_CENTER_CONFIRM_FRAMES;
        if (!enough_samples && now_us < mission->deadline_us) return;

        const int average_center_x = enough_samples
            ? mission->ball_center_x_sum / mission->ball_center_samples
            : -1;
        const int center_error_px = enough_samples
            ? average_center_x - DECODED_WIDTH / 2
            : DECODED_WIDTH;
        float corrected_ball_x = 0.0f;
        float corrected_ball_y = 0.0f;
        const bool corrected_ball = enough_samples && selected_real &&
            project_ball_to_field(selected, &corrected_ball_x,
                                  &corrected_ball_y);
        const bool ball_centered = corrected_ball &&
            abs(center_error_px) <= MISSION_BALL_CENTER_TOLERANCE_PX;
        post_line_navigation_pose_t centered_pose;
        float push_heading_error_deg = 180.0f;
        bool push_heading_aligned = false;
        if (ball_centered &&
            post_line_navigation_get_pose(&centered_pose)) {
            float goal_x;
            float goal_y;
            mission_goal_geometry(mission, &goal_x, &goal_y);
            const float desired_heading = atan2f(
                goal_y - corrected_ball_y,
                goal_x - corrected_ball_x);
            push_heading_error_deg = fabsf(normalize_radians(
                desired_heading - centered_pose.heading_rad)) *
                180.0f / PI_F;
            push_heading_aligned = push_heading_error_deg <=
                MISSION_PUSH_HEADING_TOLERANCE_DEG;
        }
        const bool push_ready = ball_centered && push_heading_aligned;
        ESP_LOGW(TAG,
                 "MISSION_PUSH_CENTER_RESULT samples=%d center_x=%d error=%dpx centered=%d heading_error=%ddeg ready=%d",
                 mission->ball_center_samples, average_center_x,
                 center_error_px, ball_centered,
                 (int)lroundf(push_heading_error_deg), push_ready);
        if (push_ready) {
            mission->ball_field_x_mm = corrected_ball_x;
            mission->ball_field_y_mm = corrected_ball_y;
            if (!mission_start_final_push(mission)) {
                mission_fail(mission, "cannot resume centered push");
                return;
            }
            mission->state = MISSION_PUSH;
            return;
        }

        if (!post_line_navigation_reverse_by(
                MISSION_RECOVERY_REVERSE_MM, MISSION_PUSH_SPEED,
                &mission->command_id)) {
            mission_fail(mission, "cannot reverse after off-center ball");
            return;
        }
        mission->missing_ball_frames = 0;
        mission->stable_ball_frames = 0;
        mission->exploring = false;
        mission->state = MISSION_PUSH_RECOVERY_REVERSE;
        ESP_LOGW(TAG,
                 "MISSION_PUSH_RECOVERY centered=%d heading_aligned=%d; reverse=%dmm then reacquire",
                 ball_centered, push_heading_aligned,
                 (int)lroundf(MISSION_RECOVERY_REVERSE_MM));
        return;
    }

    if (mission->state == MISSION_PUSH_RECOVERY_REVERSE) {
        if (!mission_command_complete(mission, &nav_pose)) return;
        mission->missing_ball_frames = 0;
        mission->stable_ball_frames = 0;
        mission->exploring = false;
        mission->state = MISSION_PUSH_RECOVERY_SEARCH;
        ESP_LOGW(TAG,
                 "MISSION_PUSH_RECOVERY_SEARCH stationary; looking for same ball");
        return;
    }

    if (mission->state == MISSION_PUSH_RECOVERY_SEARCH) {
        if (selected_real) {
            post_line_navigation_pause();
            mission->settle_until_us = now_us + MISSION_BALL_SETTLE_US;
            mission->deadline_us = now_us + MISSION_PRECISE_TIMEOUT_US;
            mission->stable_ball_frames = 0;
            mission->state = MISSION_PUSH_RECOVERY_SETTLE;
            ESP_LOGW(TAG,
                     "MISSION_PUSH_RECOVERY_BALL_SEEN color=%s; braking before replanning",
                     mission->selected_ball == MISSION_BALL_RED
                         ? "RED" : "WHITE");
            return;
        }
        if (mission->missing_ball_frames <
                MISSION_EXPLORATION_START_FRAMES) {
            mission->missing_ball_frames++;
        }
        if (!mission->exploring &&
            mission->missing_ball_frames >=
                MISSION_EXPLORATION_START_FRAMES) {
            if (!mission_start_exploration(mission)) {
                mission_fail(mission, "cannot explore for off-center ball");
                return;
            }
            mission->exploring = true;
        }
        return;
    }

    if (mission->state == MISSION_PUSH_RECOVERY_SETTLE) {
        if (now_us < mission->settle_until_us) return;
        if (now_us >= mission->deadline_us) {
            mission->missing_ball_frames = 0;
            mission->stable_ball_frames = 0;
            post_line_navigation_resume();
            mission->state = MISSION_PUSH_RECOVERY_SEARCH;
            ESP_LOGW(TAG,
                     "MISSION_PUSH_RECOVERY_RETRY ball unstable; resume slow search");
            return;
        }
        if (!mission_update_stable_ball(mission, selected, selected_real)) {
            return;
        }
        if (!mission_move_behind_ball(mission, true)) {
            mission_fail(mission, "cannot replan behind off-center ball");
            return;
        }
        mission->exploring = false;
        mission->state = MISSION_MOVE_BEHIND;
        ESP_LOGW(TAG,
                 "MISSION_PUSH_RECOVERY_REPLAN ball=(%d,%d); orbit behind and push again",
                 (int)lroundf(mission->ball_field_x_mm),
                 (int)lroundf(mission->ball_field_y_mm));
        return;
    }

    if (mission->state == MISSION_PUSH) {
        if (selected_real) {
            const float visual_right_error =
                (MISSION_VISUAL_PUSH_CENTER_X - selected->center_x) /
                (float)MISSION_VISUAL_PUSH_CENTER_X;
            post_line_navigation_set_visual_push_error(
                visual_right_error, true);
        }
        float push_ball_x = 0.0f;
        float push_ball_y = 0.0f;
        const char *ball_position_source = NULL;
        if (selected_real &&
            project_ball_to_field(selected, &push_ball_x, &push_ball_y)) {
            ball_position_source = "VISION";
        } else if (!selected_real && nav_pose_valid) {
            push_ball_x = nav_pose.x_mm +
                cosf(nav_pose.heading_rad) * MISSION_LOST_BALL_FORWARD_MM;
            push_ball_y = nav_pose.y_mm +
                sinf(nav_pose.heading_rad) * MISSION_LOST_BALL_FORWARD_MM;
            ball_position_source = "FRONT_80MM";
        }

        if (mission->push_precise_locked && ball_position_source) {
            float goal_x;
            float goal_y;
            mission_goal_geometry(mission, &goal_x, &goal_y);
            mission->ball_field_x_mm = push_ball_x;
            mission->ball_field_y_mm = push_ball_y;
            const float goal_distance = hypotf(push_ball_x - goal_x,
                                               push_ball_y - goal_y);
            if (goal_distance <= MISSION_GOAL_CAPTURE_RADIUS_MM) {
                post_line_navigation_pause();
                ESP_LOGW(TAG,
                         "MISSION_BALL_SCORED goal=%s source=%s ball=(%d,%d) distance=%dmm",
                         mission->target_goal_index == 0 ? "UPPER" : "LOWER",
                         ball_position_source,
                         (int)lroundf(push_ball_x),
                         (int)lroundf(push_ball_y),
                         (int)lroundf(goal_distance));
                if (mission->target_goal_index == 1) {
                    mission->state = MISSION_DONE;
                    ESP_LOGW(TAG, "MISSION_COMPLETE both balls pushed");
                    return;
                }
                if (!post_line_navigation_move_to(
                        INITIAL_FIELD_X_MM, INITIAL_FIELD_Y_MM,
                        MISSION_MOVE_TOLERANCE_MM, MISSION_TRAVEL_SPEED,
                        &mission->command_id)) {
                    mission_fail(mission, "cannot return to initial position");
                    return;
                }
                mission->state = MISSION_RETURN_INITIAL;
                ESP_LOGW(TAG,
                         "MISSION_RETURN target=(470,650); align to -45deg after arrival");
                return;
            }
        }

        if (nav_pose_valid && nav_pose.state == POST_NAV_RUNNING &&
            !mission->push_precise_locked &&
            update_goal_detection_window(&mission->ball_window,
                                         selected_real)) {
            mission->stable_goal_frames = 0;
            mission->settle_until_us = now_us + MISSION_BALL_SETTLE_US;
            mission->deadline_us = now_us + MISSION_PRECISE_TIMEOUT_US;
            post_line_navigation_pause();
            mission->state = MISSION_PUSH_PRECISE_LOCALIZE;
            ESP_LOGW(TAG,
                     "MISSION_PUSH_PRECISE_TRIGGER goal=%s ball=%d/%d; braking",
                     mission->target_goal_index == 0 ? "UPPER" : "LOWER",
                     mission->ball_window.detected_count,
                     mission->ball_window.count);
            return;
        }
        if (!mission_command_complete(mission, &nav_pose)) return;
        if (!post_line_navigation_reverse_by(
                MISSION_RECOVERY_REVERSE_MM, MISSION_PUSH_SPEED,
                &mission->command_id)) {
            mission_fail(mission, "cannot recover after unconfirmed goal");
            return;
        }
        mission->missing_ball_frames = 0;
        mission->stable_ball_frames = 0;
        mission->exploring = false;
        mission->state = MISSION_PUSH_RECOVERY_REVERSE;
        ESP_LOGW(TAG,
                 "MISSION_GOAL_UNCONFIRMED; reverse=%dmm then reacquire",
                 (int)lroundf(MISSION_RECOVERY_REVERSE_MM));
        return;
    }

    if (mission->state == MISSION_RETURN_INITIAL) {
        if (!mission_command_complete(mission, &nav_pose)) return;
        if (!post_line_navigation_rotate_to(
                MISSION_RETURN_HEADING_DEG, MISSION_PUSH_SPEED,
                &mission->command_id)) {
            mission_fail(mission, "cannot align after returning");
            return;
        }
        mission->state = MISSION_RETURN_ALIGN;
        ESP_LOGW(TAG,
                 "MISSION_RETURN_ALIGN target=%ddeg tolerance<=10deg",
                 (int)lroundf(MISSION_RETURN_HEADING_DEG));
        return;
    }

    if (mission->state == MISSION_RETURN_ALIGN) {
        if (!mission_command_complete(mission, &nav_pose)) return;
        mission->target_goal_index = 1;
        mission->selected_ball = mission->first_ball == MISSION_BALL_RED
            ? MISSION_BALL_WHITE : MISSION_BALL_RED;
        mission->candidate_frames = 0;
        mission->missing_ball_frames = 0;
        mission->exploring = false;
        mission->push_precise_locked = false;
        memset(&mission->ball_window, 0, sizeof(mission->ball_window));
        mission->stable_ball_frames = 0;
        mission->state = MISSION_SELECT_SECOND_BALL;
        ESP_LOGW(TAG,
                 "MISSION_SELECT_SECOND_BALL stationary color=%s heading=%ddeg",
                 mission->selected_ball == MISSION_BALL_RED
                     ? "RED" : "WHITE",
                 (int)lroundf(nav_pose.heading_deg));
    }
}

static void frame_display_task(void *argument)
{
    (void)argument;
    ESP_LOGI(TAG, "DECODE_RECOGNITION_TASK core=%d", xPortGetCoreID());
    int slot_index;
    uint32_t processed_frames = 0;
    uint32_t last_received_report = 0;
    uint32_t last_processed_report = 0;
    uint32_t last_displayed_report = 0;
    uint64_t decode_time_us = 0;
    uint64_t vision_time_us = 0;
    uint32_t timed_frames = 0;
    int64_t last_report_us = esp_timer_get_time();
    int64_t last_rgb_debug_us = 0;
    int64_t last_tuner_us = 0;
    int64_t last_calibration_us = 0;
    int64_t last_tft_preview_us = 0;
    int64_t last_object_status_us = 0;
    int64_t latest_frame_age_ms = 0;
    goal_detection_window_t detection_window = {0};
    ball_capture_mission_t mission = {.state = MISSION_BOOT};

    while (true) {
        if (xQueueReceive(s_frame_queue, &slot_index, portMAX_DELAY) != pdPASS) {
            continue;
        }
        const int64_t captured_at_us = s_slots[slot_index].captured_at_us;

        if (CAMERA_CALIBRATION_ONLY) {
            const int64_t calibration_now_us = esp_timer_get_time();
            if (s_slots[slot_index].format == UVC_FRAME_FORMAT_MJPEG &&
                s_slots[slot_index].width == PRECISE_WIDTH &&
                s_slots[slot_index].height == PRECISE_HEIGHT &&
                calibration_now_us - last_calibration_us >=
                    CALIBRATION_INTERVAL_US) {
                emit_calibration_jpeg(&s_slots[slot_index]);
                last_calibration_us = esp_timer_get_time();
            }
            release_mjpeg_slot(slot_index);
            vTaskDelay(1);
            continue;
        }

        if (camera_line_follow_calibration_enabled()) {
            const int64_t calibration_now_us = esp_timer_get_time();
            if (s_slots[slot_index].format == UVC_FRAME_FORMAT_MJPEG &&
                calibration_now_us - last_calibration_us >=
                    CALIBRATION_INTERVAL_US) {
                emit_calibration_jpeg(&s_slots[slot_index]);
                last_calibration_us = esp_timer_get_time();
            }
            release_mjpeg_slot(slot_index);
            vTaskDelay(1);
            continue;
        }

        const int64_t decode_start_us = esp_timer_get_time();
        esp_jpeg_image_output_t output = {0};
        esp_err_t decode_error;
        if (s_slots[slot_index].format == UVC_FRAME_FORMAT_YUYV) {
            decode_error = downsample_yuyv_luma(&s_slots[slot_index]);
            output.width = DECODED_WIDTH;
            output.height = DECODED_HEIGHT;
        } else {
            esp_jpeg_image_cfg_t jpeg_config = {
                .indata = s_slots[slot_index].data,
                .indata_size = s_slots[slot_index].length,
                .outbuf = s_decoded_frame,
                .outbuf_size = DECODED_BUFFER_BYTES,
                .out_format = JPEG_IMAGE_FORMAT_RGB565,
                .out_scale = s_decode_scale,
                .flags = {
                    .swap_color_bytes = 1,
                },
                .advanced = {
                    .working_buffer = s_jpeg_work_buffer,
                    .working_buffer_size = JPEG_WORK_BUFFER_BYTES,
                },
            };
            decode_error = esp_jpeg_decode(&jpeg_config, &output);
        }

        if (decode_error == ESP_OK && output.width == DECODED_WIDTH &&
            output.height == DECODED_HEIGHT) {
            const int64_t decode_done_us = esp_timer_get_time();
            const bool rgb_debug = camera_line_follow_debug_enabled();
            const bool tuner = camera_line_follow_tuner_enabled();
            if (rgb_debug || tuner) {
                memcpy(s_debug_raw_frame, s_decoded_frame,
                       DECODED_BUFFER_BYTES);
            }

            ball_vision_result_t red_ball_result;
            ball_vision_result_t white_ball_result;
            black_marker_result_t marker_result;
            quarter_goal_pose_result_t pose_result = {0};
            ball_vision_process(s_decoded_frame, output.width, output.height,
                                &red_ball_result);
            white_ball_vision_process(s_decoded_frame, output.width,
                                      output.height, &white_ball_result);
            black_marker_vision_process(s_decoded_frame, output.width,
                                        output.height, &marker_result);
            quarter_goal_pose_process(s_decoded_frame, output.width,
                                      output.height, &marker_result,
                                      &pose_result);

            const int64_t vision_now_us = esp_timer_get_time();
            const bool real_goal_detection = marker_result.found &&
                !marker_result.predicted && pose_result.found;
            update_goal_detection_window(&detection_window,
                                         real_goal_detection);
            process_ball_capture_mission(
                &mission, &s_slots[slot_index], &red_ball_result,
                &white_ball_result, &marker_result, &pose_result,
                vision_now_us);

            quarter_goal_pose_draw_overlay(s_decoded_frame, output.width,
                                           output.height, &pose_result);
            ball_vision_draw_overlay(s_decoded_frame, output.width,
                                     output.height, &red_ball_result);
            ball_vision_draw_overlay_color(s_decoded_frame, output.width,
                                           output.height, &white_ball_result,
                                           0x07ff);
            black_marker_vision_draw_overlay(s_decoded_frame, output.width,
                                             output.height, &marker_result);

            if (vision_now_us - last_object_status_us >=
                    OBJECT_STATUS_INTERVAL_US) {
                ESP_LOGI(TAG,
                         "OBJECTS red=%d/%d center=(%d,%d) white=%d/%d center=(%d,%d) goal=%d/%d center=(%d,%d) recent_goal=%d/%d localization=%s mission=%s",
                         red_ball_result.found && !red_ball_result.predicted,
                         red_ball_result.confidence,
                         red_ball_result.center_x, red_ball_result.center_y,
                         white_ball_result.found &&
                             !white_ball_result.predicted,
                         white_ball_result.confidence,
                         white_ball_result.center_x,
                         white_ball_result.center_y,
                         marker_result.found && !marker_result.predicted,
                         marker_result.confidence,
                         marker_result.center_x, marker_result.center_y,
                         detection_window.detected_count,
                         detection_window.count,
                         "PUSH_TRIGGERED",
                         mission_state_name(mission.state));
                last_object_status_us = vision_now_us;
            }

            const int64_t vision_done_us = esp_timer_get_time();
            decode_time_us += (uint64_t)(decode_done_us - decode_start_us);
            vision_time_us += (uint64_t)(vision_done_us - decode_done_us);
            timed_frames++;
            processed_frames++;
            if (!tuner && vision_done_us - last_tft_preview_us >=
                    TFT_PREVIEW_INTERVAL_US) {
                if (queue_tft_preview(s_decoded_frame)) {
                    last_tft_preview_us = vision_done_us;
                }
            }
            if (tuner && vision_done_us - last_tuner_us >=
                    TUNER_INTERVAL_US) {
                const line_vision_result_t empty_result = {0};
                emit_tuner_frame(s_debug_raw_frame, output.width,
                                 output.height, &empty_result);
                last_tuner_us = vision_done_us;
            } else if (rgb_debug &&
                       vision_done_us - last_rgb_debug_us >=
                           RGB_DEBUG_INTERVAL_US) {
                const line_vision_result_t empty_result = {0};
                emit_rgb_debug_frame(s_debug_raw_frame, output.width,
                                     output.height, &empty_result);
                last_rgb_debug_us = vision_done_us;
            }
        } else {
            ESP_LOGW(TAG, "Frame conversion failed: %s, output=%ux%u",
                     esp_err_to_name(decode_error), output.width,
                     output.height);
        }
        release_mjpeg_slot(slot_index);

        const int64_t now_us = esp_timer_get_time();
        latest_frame_age_ms = (now_us - captured_at_us) / 1000;
        if (now_us - last_report_us >= 3000000) {
            const uint32_t received_now = s_received_frames;
            const uint32_t displayed_now = s_displayed_frames;
            const uint32_t dropped_now = s_dropped_frames;
            const uint64_t elapsed_us = (uint64_t)(now_us - last_report_us);
            const uint32_t rx_fps_x10 = (uint32_t)(
                (uint64_t)(received_now - last_received_report) *
                10000000ULL / elapsed_us);
            const uint32_t processed_fps_x10 = (uint32_t)(
                (uint64_t)(processed_frames - last_processed_report) *
                10000000ULL / elapsed_us);
            const uint32_t lcd_fps_x10 = (uint32_t)(
                (uint64_t)(displayed_now - last_displayed_report) *
                10000000ULL / elapsed_us);
            const uint32_t decode_average_us = timed_frames > 0
                ? decode_time_us / timed_frames : 0;
            const uint32_t vision_average_us = timed_frames > 0
                ? vision_time_us / timed_frames : 0;
            ESP_LOGI(TAG,
                     "VIDEO fps=%lu.%lu/%lu.%lu/%lu.%lu drop=%lu age=%lldms cost=%lu/%luus",
                     (unsigned long)(rx_fps_x10 / 10),
                     (unsigned long)(rx_fps_x10 % 10),
                     (unsigned long)(processed_fps_x10 / 10),
                     (unsigned long)(processed_fps_x10 % 10),
                     (unsigned long)(lcd_fps_x10 / 10),
                     (unsigned long)(lcd_fps_x10 % 10),
                     (unsigned long)dropped_now,
                     (long long)latest_frame_age_ms,
                     (unsigned long)decode_average_us,
                     (unsigned long)vision_average_us);
            last_received_report = received_now;
            last_processed_report = processed_frames;
            last_displayed_report = displayed_now;
            decode_time_us = 0;
            vision_time_us = 0;
            timed_frames = 0;
            last_report_us = now_us;
        }
        vTaskDelay(1);
    }
}

static esp_err_t downsample_yuyv_luma(const mjpeg_slot_t *slot)
{
    if (slot->format != UVC_FRAME_FORMAT_YUYV || slot->step == 0 ||
        slot->width < DECODED_WIDTH || slot->height < DECODED_HEIGHT) {
        return ESP_ERR_INVALID_ARG;
    }
    for (size_t y = 0; y < DECODED_HEIGHT; ++y) {
        const size_t source_y = y * slot->height / DECODED_HEIGHT;
        const uint8_t *source_row = slot->data + source_y * slot->step;
        for (size_t x = 0; x < DECODED_WIDTH; ++x) {
            const size_t source_x = x * slot->width / DECODED_WIDTH;
            const uint8_t luminance = source_row[source_x * 2];
            const uint16_t gray = ((uint16_t)(luminance >> 3) << 11) |
                                  ((uint16_t)(luminance >> 2) << 5) |
                                  (luminance >> 3);
            const size_t destination = 2 * (y * DECODED_WIDTH + x);
            s_decoded_frame[destination] = gray >> 8;
            s_decoded_frame[destination + 1] = gray & 0xff;
        }
    }
    return ESP_OK;
}

static esp_err_t initialize_frame_pipeline(void)
{
    s_frame_queue = xQueueCreate(FRAME_QUEUE_LENGTH, sizeof(int));
    s_display_queue = xQueueCreate(DISPLAY_QUEUE_LENGTH, sizeof(uint8_t));
    if (s_frame_queue == NULL || s_display_queue == NULL) {
        return ESP_ERR_NO_MEM;
    }

    for (int index = 0; index < MJPEG_SLOT_COUNT; ++index) {
        s_slots[index].data = heap_caps_malloc(MJPEG_SLOT_CAPACITY,
                                               MALLOC_CAP_SPIRAM |
                                               MALLOC_CAP_8BIT);
        if (s_slots[index].data == NULL) {
            ESP_LOGE(TAG, "Cannot allocate MJPEG slot %d", index);
            return ESP_ERR_NO_MEM;
        }
    }
    s_decoded_frame = heap_caps_malloc(DECODED_BUFFER_BYTES,
                                       MALLOC_CAP_INTERNAL |
                                       MALLOC_CAP_8BIT);
    if (s_decoded_frame == NULL) {
        ESP_LOGW(TAG, "Internal decode buffer unavailable; using PSRAM");
        s_decoded_frame = heap_caps_malloc(DECODED_BUFFER_BYTES,
                                           MALLOC_CAP_SPIRAM |
                                           MALLOC_CAP_8BIT);
    } else {
        ESP_LOGI(TAG, "FAST_DECODE_BUFFER=INTERNAL bytes=%d",
                 DECODED_BUFFER_BYTES);
    }
    s_display_frame = heap_caps_malloc(DECODED_BUFFER_BYTES,
                                       MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (s_decoded_frame == NULL || s_display_frame == NULL) {
        return ESP_ERR_NO_MEM;
    }
    s_debug_raw_frame = heap_caps_malloc(DECODED_BUFFER_BYTES,
                                         MALLOC_CAP_SPIRAM |
                                         MALLOC_CAP_8BIT);
    if (s_debug_raw_frame == NULL) {
        return ESP_ERR_NO_MEM;
    }
    s_precise_frame = heap_caps_malloc(PRECISE_BUFFER_BYTES,
                                       MALLOC_CAP_SPIRAM |
                                       MALLOC_CAP_8BIT);
    if (s_precise_frame == NULL) {
        return ESP_ERR_NO_MEM;
    }
    s_tuner_mask = heap_caps_malloc(TUNER_MASK_BYTES,
                                    MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (s_tuner_mask == NULL) {
        return ESP_ERR_NO_MEM;
    }
    s_jpeg_work_buffer = heap_caps_malloc(JPEG_WORK_BUFFER_BYTES,
                                          MALLOC_CAP_INTERNAL |
                                          MALLOC_CAP_8BIT);
    if (s_jpeg_work_buffer == NULL) {
        return ESP_ERR_NO_MEM;
    }

    if (xTaskCreatePinnedToCore(frame_display_task, "camera_display", 8192,
                                NULL, 4, NULL, 1) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    if (xTaskCreatePinnedToCore(tft_display_task, "tft_preview", 4096,
                                NULL, 3, NULL, 0) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG,
             "PIPELINE decode/path-pursuit=core1 async-TFT=core0 frame=%dx%d queue=latest",
             DECODED_WIDTH, DECODED_HEIGHT);
    return ESP_OK;
}

static uvc_error_t negotiate_mjpeg_stream(uvc_device_handle_t *device_handle,
                                          uvc_stream_ctrl_t *control)
{
    typedef struct {
        enum uvc_frame_format format;
        int width;
        int height;
        int fps;
        esp_jpeg_image_scale_t scale;
    } camera_profile_t;
    static const camera_profile_t profiles[] = {
        {UVC_FRAME_FORMAT_MJPEG, 640, 480, 15, JPEG_IMAGE_SCALE_1_4},
        {UVC_FRAME_FORMAT_MJPEG, 640, 480, 30, JPEG_IMAGE_SCALE_1_4},
        {UVC_FRAME_FORMAT_MJPEG, 320, 240, 30, JPEG_IMAGE_SCALE_1_2},
        {UVC_FRAME_FORMAT_MJPEG, 320, 240, 15, JPEG_IMAGE_SCALE_1_2},
        {UVC_FRAME_FORMAT_YUYV, 160, 120, 30, JPEG_IMAGE_SCALE_0},
        {UVC_FRAME_FORMAT_YUYV, 160, 120, 15, JPEG_IMAGE_SCALE_0},
    };
    uvc_error_t result = UVC_ERROR_INVALID_MODE;
    for (size_t profile = 0;
         profile < sizeof(profiles) / sizeof(profiles[0]); ++profile) {
        if (CAMERA_CALIBRATION_ONLY &&
            (profiles[profile].format != UVC_FRAME_FORMAT_MJPEG ||
             profiles[profile].width != PRECISE_WIDTH ||
             profiles[profile].height != PRECISE_HEIGHT)) {
            continue;
        }
        for (int attempt = 1; attempt <= 2; ++attempt) {
            result = uvc_get_stream_ctrl_format_size(device_handle, control,
                                                      profiles[profile].format,
                                                      profiles[profile].width,
                                                      profiles[profile].height,
                                                      profiles[profile].fps);
            if (result == UVC_SUCCESS) {
                s_stream_width = profiles[profile].width;
                s_stream_height = profiles[profile].height;
                s_stream_fps = profiles[profile].fps;
                s_stream_format = profiles[profile].format;
                s_decode_scale = profiles[profile].scale;
                control->dwMaxPayloadTransferSize =
                    profiles[profile].format == UVC_FRAME_FORMAT_YUYV
                        ? 1023 : 512;
                return UVC_SUCCESS;
            }
        }
    }
    return result;
}

void app_main(void)
{
    if (CAMERA_CALIBRATION_ONLY) {
        ESP_LOGI(TAG, "Dedicated 640x480 camera calibration firmware");
    } else {
        ESP_LOGI(TAG,
                 "Integrated object recognition, one-shot visual pose correction and waypoint navigation");
    }
    ESP_LOGI(TAG, "UART0: TX=GPIO43 RX=GPIO44 baud=%d",
             CAMERA_CALIBRATION_ONLY ? CALIBRATION_UART_BAUD : 115200);
    ESP_LOGI(TAG, "Camera: D-=GPIO19 D+=GPIO20; motors start in SAFE STOP");

    const size_t psram_size = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
    ESP_LOGI(TAG, "PSRAM available: %u bytes", (unsigned)psram_size);
    if (psram_size == 0) {
        ESP_LOGE(TAG, "CAMERA_STATUS=NO_PSRAM");
        return;
    }

    ESP_ERROR_CHECK(initialize_motor_safe_stop());
    if (CAMERA_CALIBRATION_ONLY) {
        ESP_LOGW(TAG,
                 "CALIBRATION_ONLY: motors stopped; TFT, vision and odometry disabled");
        ESP_ERROR_CHECK(initialize_calibration_uart());
    } else {
        ESP_ERROR_CHECK(post_line_odometry_init());
        ESP_ERROR_CHECK(post_line_navigation_init(
            INITIAL_FIELD_X_MM, INITIAL_FIELD_Y_MM,
            INITIAL_FIELD_HEADING_DEG));
        ESP_ERROR_CHECK(ball_vision_init(DECODED_WIDTH, DECODED_HEIGHT));
        ESP_ERROR_CHECK(white_ball_vision_init(DECODED_WIDTH,
                                                DECODED_HEIGHT));
        ESP_ERROR_CHECK(black_marker_vision_init(DECODED_WIDTH,
                                                  DECODED_HEIGHT));
        black_marker_vision_set_logging(false);
        quarter_goal_pose_set_single_corner_mode(true);
        quarter_goal_pose_set_logging(false);
        ESP_ERROR_CHECK(camera_display_init());
        ESP_ERROR_CHECK(camera_display_show_waiting());
    }
    ESP_ERROR_CHECK(initialize_frame_pipeline());

    s_uvc_events = xEventGroupCreate();
    ESP_ERROR_CHECK(s_uvc_events == NULL ? ESP_ERR_NO_MEM : ESP_OK);
    ESP_ERROR_CHECK(initialize_usb_host());

    libuvc_adapter_config_t adapter_config = {
        .create_background_task = true,
        .task_priority = 5,
        .stack_size = 4096,
        .callback = libuvc_event_callback,
    };
    libuvc_adapter_set_config(&adapter_config);

    uvc_context_t *context = NULL;
    const uvc_error_t init_result = uvc_init(&context, NULL);
    if (init_result != UVC_SUCCESS) {
        ESP_LOGE(TAG, "uvc_init failed: %s", uvc_error_string(init_result));
        return;
    }

    while (true) {
        ESP_LOGI(TAG, "CAMERA_STATUS=WAITING_FOR_USB_CAMERA");
        wait_for_uvc_event(UVC_DEVICE_CONNECTED);

        uvc_device_t *device = NULL;
        uvc_device_handle_t *device_handle = NULL;
        uvc_stream_ctrl_t stream_control = {0};

        uvc_error_t result = uvc_find_device(context, &device, 0, 0, NULL);
        if (result != UVC_SUCCESS) {
            ESP_LOGW(TAG, "Camera find failed: %s", uvc_error_string(result));
            continue;
        }
        result = uvc_open(device, &device_handle);
        if (result != UVC_SUCCESS) {
            ESP_LOGW(TAG, "Camera open failed: %s", uvc_error_string(result));
            uvc_unref_device(device);
            continue;
        }

        ESP_LOGI(TAG, "CAMERA_STATUS=UVC_OPEN");
        result = negotiate_mjpeg_stream(device_handle, &stream_control);
        if (result == UVC_SUCCESS) {
            result = uvc_start_streaming(device_handle, &stream_control,
                                         camera_frame_callback, NULL, 0);
        }

        if (result == UVC_SUCCESS) {
            ESP_LOGI(TAG, "CAMERA_STATUS=STREAMING_%dx%d_%dFPS_%s",
                     s_stream_width, s_stream_height, s_stream_fps,
                     s_stream_format == UVC_FRAME_FORMAT_YUYV
                         ? "YUYV" : "MJPEG");
            if (!CAMERA_CALIBRATION_ONLY) {
                post_line_navigation_start();
            }
            wait_for_uvc_event(UVC_DEVICE_DISCONNECTED);
            if (!CAMERA_CALIBRATION_ONLY) {
                post_line_navigation_pause();
            }
            uvc_stop_streaming(device_handle);
        } else {
            ESP_LOGE(TAG, "CAMERA_STATUS=STREAM_FAILED error=%s",
                     uvc_error_string(result));
            wait_for_uvc_event(UVC_DEVICE_DISCONNECTED);
        }

        if (!CAMERA_CALIBRATION_ONLY) {
            camera_display_show_waiting();
        }
        uvc_close(device_handle);
        uvc_unref_device(device);
        ESP_LOGW(TAG, "CAMERA_STATUS=DISCONNECTED");
    }
}
