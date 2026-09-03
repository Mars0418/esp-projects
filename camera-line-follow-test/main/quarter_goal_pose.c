#include "quarter_goal_pose.h"

#include <float.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"

#define CALIBRATED_WIDTH 160
#define CALIBRATED_HEIGHT 120
#define GOAL_RADIUS_MM 100.0f
#define MIN_POSE_PIXELS 18
#define EDGE_BAND_MM 14.0f
#define GOAL_ID_MIN_CONFIDENCE 70
#define GOAL_ID_MIN_LATERAL_MM 80.0f
#define GOAL_ID_MIN_FORWARD_MM 0.0f
#define GOAL_ID_MAX_RADIUS_ERROR_MM 30.0f
#define GOAL_ID_CONFIRM_FRAMES 5
#define UPPER_GOAL_FIELD_X_MM 0.0f
#define UPPER_GOAL_FIELD_Y_MM 0.0f
#define LOWER_GOAL_FIELD_X_MM 900.0f
#define LOWER_GOAL_FIELD_Y_MM 0.0f
#define SQRT_HALF 0.70710678118f
#define PI_F 3.14159265359f
#define PRECISE_ROI_MARGIN_PX 12
#define PRECISE_EDGE_MIN_ALONG_MM 8.0f
#define PRECISE_EDGE_MAX_ALONG_MM 150.0f
#define PRECISE_EDGE_SEARCH_BAND_MM 28.0f
#define PRECISE_EDGE_INLIER_MM 12.0f
#define PRECISE_MIN_EDGE_PIXELS 20
#define PRECISE_MAX_AXIS_DOT 0.35f

static const char *TAG = "goal_pose";
static quarter_goal_identity_t s_selected_goal = QUARTER_GOAL_UNKNOWN;
static int s_upper_goal_votes;
static int s_lower_goal_votes;
static bool s_odometry_valid;
static float s_odometry_x_mm;
static float s_odometry_y_mm;
static float s_odometry_heading_rad;
static bool s_goal_memory_valid;
static quarter_goal_identity_t s_goal_memory_identity;
static float s_goal_reference_x_mm;
static float s_goal_reference_y_mm;
static float s_field_anchor_odometry_x_mm;
static float s_field_anchor_odometry_y_mm;
static float s_field_anchor_robot_x_mm;
static float s_field_anchor_robot_y_mm;
static float s_field_x_axis_reference_x;
static float s_field_x_axis_reference_y;
static float s_field_y_axis_reference_x;
static float s_field_y_axis_reference_y;
static bool s_single_corner_mode;
static bool s_pose_logging_enabled = true;

/* Runtime calibration copied from camera_ground_calibration.json (2026-09-03,
 * 14 samples, RMS 0.405 px). The vehicle-centre to image-bottom ground
 * distance is 80 mm and the raw camera image is rotated by 180 degrees. */
static const float FX = 62.2658588172f;
static const float FY = 63.7391172126f;
static const float CX = 79.5008738840f;
static const float CY = 53.4665883481f;
static const float K1 = 0.0815474671f;
static const float K2 = -0.0890497661f;
static const float P1 = -0.0151775104f;
static const float P2 = 0.00328599594f;
static const float K3 = 0.0191844912f;
static const float H[3][3] = {
    {12.9968406512f, -0.0218747386f, -1027.41504410f},
    {-0.1528642179f, -4.8746225132f, 1142.98429052f},
    {-0.00122387697f, 0.0475105968f, 1.0f},
};

typedef struct {
    float x;
    float y;
} point_t;

typedef struct {
    bool valid;
    int confidence;
    point_t origin;
    point_t axis_x;
    point_t axis_y;
    float radius_mm;
} pose_candidate_t;

typedef struct {
    int count;
    double sum_x;
    double sum_y;
    double sum_xx;
    double sum_xy;
    double sum_yy;
} line_stats_t;

static const char *goal_identity_name(quarter_goal_identity_t identity)
{
    switch (identity) {
    case QUARTER_GOAL_UPPER:
        return "UPPER";
    case QUARTER_GOAL_LOWER:
        return "LOWER";
    default:
        return "UNKNOWN";
    }
}

static bool goal_measurement_reliable(
    const quarter_goal_pose_result_t *result)
{
    return result->found &&
           result->confidence >= GOAL_ID_MIN_CONFIDENCE &&
           result->origin_vehicle_y_mm >= GOAL_ID_MIN_FORWARD_MM &&
           fabsf(result->fitted_radius_mm - GOAL_RADIUS_MM) <=
               GOAL_ID_MAX_RADIUS_ERROR_MM;
}

void quarter_goal_pose_set_relative_odometry(bool valid, float x_mm,
                                             float y_mm,
                                             float heading_rad)
{
    s_odometry_valid = valid;
    s_odometry_x_mm = x_mm;
    s_odometry_y_mm = y_mm;
    s_odometry_heading_rad = heading_rad;
}

void quarter_goal_pose_set_single_corner_mode(bool enabled)
{
    s_single_corner_mode = enabled;
    s_selected_goal = QUARTER_GOAL_UNKNOWN;
    s_upper_goal_votes = 0;
    s_lower_goal_votes = 0;
    s_goal_memory_valid = false;
}

void quarter_goal_pose_set_logging(bool enabled)
{
    s_pose_logging_enabled = enabled;
}

static void update_goal_identity(quarter_goal_pose_result_t *result)
{
    result->candidate_goal = QUARTER_GOAL_UNKNOWN;
    result->remembered_lateral_mm = result->origin_vehicle_x_mm;
    result->odometry_used = s_odometry_valid;
    result->odometry_x_mm = s_odometry_x_mm;
    result->odometry_y_mm = s_odometry_y_mm;
    result->odometry_heading_deg =
        s_odometry_heading_rad * 180.0f / PI_F;
    if (s_odometry_valid) {
        const float cosine = cosf(s_odometry_heading_rad);
        const float sine = sinf(s_odometry_heading_rad);
        result->remembered_lateral_mm =
            s_odometry_x_mm + cosine * result->origin_vehicle_x_mm -
            sine * result->origin_vehicle_y_mm;
    }

    const bool reliable = goal_measurement_reliable(result);
    if (reliable) {
        if (result->remembered_lateral_mm <= -GOAL_ID_MIN_LATERAL_MM) {
            result->candidate_goal = QUARTER_GOAL_LOWER;
        } else if (result->remembered_lateral_mm >=
                   GOAL_ID_MIN_LATERAL_MM) {
            result->candidate_goal = QUARTER_GOAL_UPPER;
        }
    }

    if (s_selected_goal == QUARTER_GOAL_UNKNOWN) {
        if (result->candidate_goal == QUARTER_GOAL_UPPER) {
            if (s_upper_goal_votes < GOAL_ID_CONFIRM_FRAMES) {
                s_upper_goal_votes++;
            }
            s_lower_goal_votes = 0;
        } else if (result->candidate_goal == QUARTER_GOAL_LOWER) {
            if (s_lower_goal_votes < GOAL_ID_CONFIRM_FRAMES) {
                s_lower_goal_votes++;
            }
            s_upper_goal_votes = 0;
        } else {
            s_upper_goal_votes = 0;
            s_lower_goal_votes = 0;
        }

        if (s_upper_goal_votes >= GOAL_ID_CONFIRM_FRAMES) {
            s_selected_goal = QUARTER_GOAL_UPPER;
        } else if (s_lower_goal_votes >= GOAL_ID_CONFIRM_FRAMES) {
            s_selected_goal = QUARTER_GOAL_LOWER;
        }
    }

    result->selected_goal = s_selected_goal;
    result->upper_goal_votes = s_upper_goal_votes;
    result->lower_goal_votes = s_lower_goal_votes;
}

static void map_visual_pose_to_global_field(
    quarter_goal_pose_result_t *result)
{
    if (result->position_goal == QUARTER_GOAL_UNKNOWN) return;

    float goal_x_mm = UPPER_GOAL_FIELD_X_MM;
    float goal_y_mm = UPPER_GOAL_FIELD_Y_MM;
    if (result->position_goal == QUARTER_GOAL_LOWER) {
        goal_x_mm = LOWER_GOAL_FIELD_X_MM;
        goal_y_mm = LOWER_GOAL_FIELD_Y_MM;
        /* The lower quarter circle opens toward global -X and +Y. */
        result->x_axis_vehicle_x = -result->x_axis_vehicle_x;
        result->x_axis_vehicle_y = -result->x_axis_vehicle_y;
    }
    result->goal_field_x_mm = goal_x_mm;
    result->goal_field_y_mm = goal_y_mm;

    result->robot_field_x_mm =
        goal_x_mm -
        (result->origin_vehicle_x_mm * result->x_axis_vehicle_x +
         result->origin_vehicle_y_mm * result->x_axis_vehicle_y);
    result->robot_field_y_mm =
        goal_y_mm -
        (result->origin_vehicle_x_mm * result->y_axis_vehicle_x +
         result->origin_vehicle_y_mm * result->y_axis_vehicle_y);
    result->robot_heading_deg =
        atan2f(result->y_axis_vehicle_y, result->x_axis_vehicle_y) *
        180.0f / PI_F;
    result->field_pose_valid = true;
}

bool quarter_goal_pose_assign_goal(quarter_goal_pose_result_t *result,
                                   quarter_goal_identity_t identity)
{
    if (!result || !result->position_valid ||
        identity == QUARTER_GOAL_UNKNOWN) {
        return false;
    }
    result->position_goal = identity;
    result->field_pose_valid = false;
    map_visual_pose_to_global_field(result);
    return result->field_pose_valid;
}

static void remember_visual_pose(const quarter_goal_pose_result_t *result)
{
    if (!s_odometry_valid || !goal_measurement_reliable(result)) return;

    quarter_goal_identity_t identity = result->candidate_goal;
    if (result->selected_goal != QUARTER_GOAL_UNKNOWN) {
        if (identity != QUARTER_GOAL_UNKNOWN &&
            identity != result->selected_goal) {
            return;
        }
        identity = result->selected_goal;
    }
    if (identity == QUARTER_GOAL_UNKNOWN) return;

    const float cosine = cosf(s_odometry_heading_rad);
    const float sine = sinf(s_odometry_heading_rad);
    s_goal_reference_x_mm =
        s_odometry_x_mm + cosine * result->origin_vehicle_x_mm -
        sine * result->origin_vehicle_y_mm;
    s_goal_reference_y_mm =
        s_odometry_y_mm + sine * result->origin_vehicle_x_mm +
        cosine * result->origin_vehicle_y_mm;

    s_field_anchor_odometry_x_mm = s_odometry_x_mm;
    s_field_anchor_odometry_y_mm = s_odometry_y_mm;
    s_field_anchor_robot_x_mm = result->robot_field_x_mm;
    s_field_anchor_robot_y_mm = result->robot_field_y_mm;
    s_field_x_axis_reference_x =
        cosine * result->x_axis_vehicle_x -
        sine * result->x_axis_vehicle_y;
    s_field_x_axis_reference_y =
        sine * result->x_axis_vehicle_x +
        cosine * result->x_axis_vehicle_y;
    s_field_y_axis_reference_x =
        cosine * result->y_axis_vehicle_x -
        sine * result->y_axis_vehicle_y;
    s_field_y_axis_reference_y =
        sine * result->y_axis_vehicle_x +
        cosine * result->y_axis_vehicle_y;
    s_goal_memory_identity = identity;
    s_goal_memory_valid = true;
}

static void predict_pose_from_odometry(quarter_goal_pose_result_t *result)
{
    if (!s_odometry_valid || !s_goal_memory_valid) return;

    const float cosine = cosf(s_odometry_heading_rad);
    const float sine = sinf(s_odometry_heading_rad);
    const float goal_dx = s_goal_reference_x_mm - s_odometry_x_mm;
    const float goal_dy = s_goal_reference_y_mm - s_odometry_y_mm;
    result->origin_vehicle_x_mm = cosine * goal_dx + sine * goal_dy;
    result->origin_vehicle_y_mm = -sine * goal_dx + cosine * goal_dy;

    const float odometry_dx =
        s_odometry_x_mm - s_field_anchor_odometry_x_mm;
    const float odometry_dy =
        s_odometry_y_mm - s_field_anchor_odometry_y_mm;
    result->robot_field_x_mm =
        s_field_anchor_robot_x_mm +
        odometry_dx * s_field_x_axis_reference_x +
        odometry_dy * s_field_x_axis_reference_y;
    result->robot_field_y_mm =
        s_field_anchor_robot_y_mm +
        odometry_dx * s_field_y_axis_reference_x +
        odometry_dy * s_field_y_axis_reference_y;

    const float forward_reference_x = -sine;
    const float forward_reference_y = cosine;
    const float forward_field_x =
        forward_reference_x * s_field_x_axis_reference_x +
        forward_reference_y * s_field_x_axis_reference_y;
    const float forward_field_y =
        forward_reference_x * s_field_y_axis_reference_x +
        forward_reference_y * s_field_y_axis_reference_y;
    result->robot_heading_deg =
        atan2f(forward_field_y, forward_field_x) * 180.0f / PI_F;
    result->remembered_lateral_mm = s_goal_reference_x_mm;
    result->position_goal = s_goal_memory_identity;
    result->goal_field_x_mm =
        s_goal_memory_identity == QUARTER_GOAL_LOWER
            ? LOWER_GOAL_FIELD_X_MM : UPPER_GOAL_FIELD_X_MM;
    result->goal_field_y_mm =
        s_goal_memory_identity == QUARTER_GOAL_LOWER
            ? LOWER_GOAL_FIELD_Y_MM : UPPER_GOAL_FIELD_Y_MM;
    result->position_valid = true;
    result->field_pose_valid = true;
    result->odometry_predicted = true;
}

static void log_missing_pose(const quarter_goal_pose_result_t *result,
                             int sample_count)
{
    ESP_LOGI(TAG,
             "GOAL_POSE found=0 predicted=%d position_valid=%d field_pose_valid=%d selected=%s position_goal=%s odom=%d odom_pose=(%d,%d,%ddeg) goal_field_mm=(%d,%d) goal_vehicle_mm=(%d,%d) robot_field_mm=(%d,%d) heading_deg=%d samples=%d",
             result->odometry_predicted, result->position_valid,
             result->field_pose_valid,
             goal_identity_name(result->selected_goal),
             goal_identity_name(result->position_goal),
             result->odometry_used,
             (int)lroundf(result->odometry_x_mm),
             (int)lroundf(result->odometry_y_mm),
             (int)lroundf(result->odometry_heading_deg),
             (int)lroundf(result->goal_field_x_mm),
             (int)lroundf(result->goal_field_y_mm),
             (int)lroundf(result->origin_vehicle_x_mm),
             (int)lroundf(result->origin_vehicle_y_mm),
             (int)lroundf(result->robot_field_x_mm),
             (int)lroundf(result->robot_field_y_mm),
             (int)lroundf(result->robot_heading_deg), sample_count);
}

static void handle_missing_pose(quarter_goal_pose_result_t *result,
                                int sample_count)
{
    if (s_single_corner_mode) {
        if (s_pose_logging_enabled) {
            ESP_LOGI(TAG, "CORNER_POSE found=0");
        }
        return;
    }
    update_goal_identity(result);
    predict_pose_from_odometry(result);
    log_missing_pose(result, sample_count);
}

static int clamp_int(int value, int minimum, int maximum);
static float dot(point_t a, point_t b);
static bool raw_pixel_to_vehicle_ground(float raw_x, float raw_y,
                                        point_t *ground);

static point_t point_subtract(point_t a, point_t b)
{
    return (point_t) {.x = a.x - b.x, .y = a.y - b.y};
}

static point_t point_normalize(point_t value)
{
    const float length = sqrtf(value.x * value.x + value.y * value.y);
    if (length < 0.0001f) return (point_t) {0};
    return (point_t) {.x = value.x / length, .y = value.y / length};
}

static void line_stats_add(line_stats_t *stats, point_t point)
{
    stats->count++;
    stats->sum_x += point.x;
    stats->sum_y += point.y;
    stats->sum_xx += point.x * point.x;
    stats->sum_xy += point.x * point.y;
    stats->sum_yy += point.y * point.y;
}

static point_t line_stats_mean(const line_stats_t *stats)
{
    if (stats->count <= 0) return (point_t) {0};
    return (point_t) {
        .x = (float)(stats->sum_x / stats->count),
        .y = (float)(stats->sum_y / stats->count),
    };
}

static bool line_stats_direction(const line_stats_t *stats,
                                 point_t *direction)
{
    if (!direction || stats->count < PRECISE_MIN_EDGE_PIXELS) return false;
    const double inverse_count = 1.0 / stats->count;
    const double mean_x = stats->sum_x * inverse_count;
    const double mean_y = stats->sum_y * inverse_count;
    const float covariance_xx =
        (float)(stats->sum_xx * inverse_count - mean_x * mean_x);
    const float covariance_xy =
        (float)(stats->sum_xy * inverse_count - mean_x * mean_y);
    const float covariance_yy =
        (float)(stats->sum_yy * inverse_count - mean_y * mean_y);
    if (covariance_xx + covariance_yy < 1.0f) return false;
    const float angle = 0.5f * atan2f(2.0f * covariance_xy,
                                      covariance_xx - covariance_yy);
    *direction = (point_t) {.x = cosf(angle), .y = sinf(angle)};
    return true;
}

static void orient_like(point_t *direction, point_t reference)
{
    if (dot(*direction, reference) < 0.0f) {
        direction->x = -direction->x;
        direction->y = -direction->y;
    }
}

static bool orthogonalize_axes(point_t edge_x, point_t edge_y,
                               point_t reference_x, point_t reference_y,
                               point_t *axis_x, point_t *axis_y)
{
    orient_like(&edge_x, reference_x);
    orient_like(&edge_y, reference_y);
    const float axis_dot = fabsf(dot(edge_x, edge_y));
    if (axis_dot > PRECISE_MAX_AXIS_DOT) return false;

    point_t x_from_y = {.x = edge_y.y, .y = -edge_y.x};
    orient_like(&x_from_y, edge_x);
    *axis_x = point_normalize((point_t) {
        .x = edge_x.x + x_from_y.x,
        .y = edge_x.y + x_from_y.y,
    });
    *axis_y = (point_t) {.x = -axis_x->y, .y = axis_x->x};
    if (dot(*axis_x, reference_x) < 0.0f ||
        dot(*axis_y, reference_y) < 0.0f) {
        return false;
    }
    return true;
}

static bool raw_pixel_to_vehicle_ground_scaled(float raw_x, float raw_y,
                                                size_t width, size_t height,
                                                point_t *ground)
{
    if (width < 2 || height < 2) return false;
    const float calibrated_x =
        raw_x * (CALIBRATED_WIDTH - 1) / (float)(width - 1);
    const float calibrated_y =
        raw_y * (CALIBRATED_HEIGHT - 1) / (float)(height - 1);
    return raw_pixel_to_vehicle_ground(calibrated_x, calibrated_y, ground);
}

static bool precise_boundary_pixel(const uint8_t *rgb565, size_t width,
                                   size_t height, int x, int y)
{
    const size_t index = (size_t)y * width + (size_t)x;
    if (!black_marker_vision_pixel_is_goal(rgb565, index)) return false;
    static const int offsets[4][2] = {
        {-1, 0}, {1, 0}, {0, -1}, {0, 1},
    };
    for (size_t offset = 0; offset < 4; ++offset) {
        const int neighbour_x = x + offsets[offset][0];
        const int neighbour_y = y + offsets[offset][1];
        if (neighbour_x < 0 || neighbour_y < 0 ||
            neighbour_x >= (int)width || neighbour_y >= (int)height) {
            return true;
        }
        const size_t neighbour =
            (size_t)neighbour_y * width + (size_t)neighbour_x;
        if (!black_marker_vision_pixel_is_goal(rgb565, neighbour)) {
            return true;
        }
    }
    return false;
}

bool quarter_goal_pose_refine_single_corner(
    const uint8_t *rgb565, size_t width, size_t height,
    const black_marker_result_t *coarse_basic,
    const quarter_goal_pose_result_t *coarse_pose,
    quarter_goal_pose_result_t *result)
{
    if (!rgb565 || !coarse_basic || !coarse_pose || !result ||
        !coarse_basic->found || !coarse_pose->found ||
        width < CALIBRATED_WIDTH || height < CALIBRATED_HEIGHT) {
        return false;
    }

    const float scale_x = (width - 1) / (float)(CALIBRATED_WIDTH - 1);
    const float scale_y = (height - 1) / (float)(CALIBRATED_HEIGHT - 1);
    const int roi_left = clamp_int(
        (int)floorf(coarse_basic->left * scale_x) - PRECISE_ROI_MARGIN_PX,
        1, (int)width - 2);
    const int roi_right = clamp_int(
        (int)ceilf(coarse_basic->right * scale_x) + PRECISE_ROI_MARGIN_PX,
        1, (int)width - 2);
    const int roi_top = clamp_int(
        (int)floorf(coarse_basic->top * scale_y) - PRECISE_ROI_MARGIN_PX,
        1, (int)height - 2);
    const int roi_bottom = clamp_int(
        (int)ceilf(coarse_basic->bottom * scale_y) + PRECISE_ROI_MARGIN_PX,
        1, (int)height - 2);

    const point_t coarse_origin = {
        .x = coarse_pose->origin_vehicle_x_mm,
        .y = coarse_pose->origin_vehicle_y_mm,
    };
    const point_t coarse_axis_x = {
        .x = coarse_pose->x_axis_vehicle_x,
        .y = coarse_pose->x_axis_vehicle_y,
    };
    const point_t coarse_axis_y = {
        .x = coarse_pose->y_axis_vehicle_x,
        .y = coarse_pose->y_axis_vehicle_y,
    };

    line_stats_t broad_x = {0};
    line_stats_t broad_y = {0};
    for (int y = roi_top; y <= roi_bottom; ++y) {
        for (int x = roi_left; x <= roi_right; ++x) {
            if (!precise_boundary_pixel(rgb565, width, height, x, y)) {
                continue;
            }
            point_t point;
            if (!raw_pixel_to_vehicle_ground_scaled(
                    (float)x, (float)y, width, height, &point)) {
                continue;
            }
            const point_t relative = point_subtract(point, coarse_origin);
            const float along_x = dot(relative, coarse_axis_x);
            const float along_y = dot(relative, coarse_axis_y);
            if (along_x >= PRECISE_EDGE_MIN_ALONG_MM &&
                along_x <= PRECISE_EDGE_MAX_ALONG_MM &&
                fabsf(along_y) <= PRECISE_EDGE_SEARCH_BAND_MM) {
                line_stats_add(&broad_x, point);
            }
            if (along_y >= PRECISE_EDGE_MIN_ALONG_MM &&
                along_y <= PRECISE_EDGE_MAX_ALONG_MM &&
                fabsf(along_x) <= PRECISE_EDGE_SEARCH_BAND_MM) {
                line_stats_add(&broad_y, point);
            }
        }
    }

    point_t edge_x;
    point_t edge_y;
    point_t axis_x;
    point_t axis_y;
    if (!line_stats_direction(&broad_x, &edge_x) ||
        !line_stats_direction(&broad_y, &edge_y) ||
        !orthogonalize_axes(edge_x, edge_y, coarse_axis_x, coarse_axis_y,
                            &axis_x, &axis_y)) {
        return false;
    }

    line_stats_t refined_x = {0};
    line_stats_t refined_y = {0};
    for (int y = roi_top; y <= roi_bottom; ++y) {
        for (int x = roi_left; x <= roi_right; ++x) {
            if (!precise_boundary_pixel(rgb565, width, height, x, y)) {
                continue;
            }
            point_t point;
            if (!raw_pixel_to_vehicle_ground_scaled(
                    (float)x, (float)y, width, height, &point)) {
                continue;
            }
            const point_t relative = point_subtract(point, coarse_origin);
            const float along_x = dot(relative, axis_x);
            const float along_y = dot(relative, axis_y);
            if (along_x >= PRECISE_EDGE_MIN_ALONG_MM &&
                along_x <= PRECISE_EDGE_MAX_ALONG_MM &&
                fabsf(along_y) <= PRECISE_EDGE_INLIER_MM) {
                line_stats_add(&refined_x, point);
            }
            if (along_y >= PRECISE_EDGE_MIN_ALONG_MM &&
                along_y <= PRECISE_EDGE_MAX_ALONG_MM &&
                fabsf(along_x) <= PRECISE_EDGE_INLIER_MM) {
                line_stats_add(&refined_y, point);
            }
        }
    }

    if (!line_stats_direction(&refined_x, &edge_x) ||
        !line_stats_direction(&refined_y, &edge_y) ||
        !orthogonalize_axes(edge_x, edge_y, coarse_axis_x, coarse_axis_y,
                            &axis_x, &axis_y)) {
        return false;
    }

    const point_t mean_x = line_stats_mean(&refined_x);
    const point_t mean_y = line_stats_mean(&refined_y);
    const float origin_axis_x = dot(mean_y, axis_x);
    const float origin_axis_y = dot(mean_x, axis_y);
    const point_t origin = {
        .x = origin_axis_x * axis_x.x + origin_axis_y * axis_y.x,
        .y = origin_axis_x * axis_x.y + origin_axis_y * axis_y.y,
    };
    if (hypotf(origin.x - coarse_origin.x,
               origin.y - coarse_origin.y) > 45.0f) {
        return false;
    }

    float maximum_x = 0.0f;
    float maximum_y = 0.0f;
    float nearest_origin = FLT_MAX;
    int origin_raw_x = -1;
    int origin_raw_y = -1;
    int x_end_raw_x = -1;
    int x_end_raw_y = -1;
    int y_end_raw_x = -1;
    int y_end_raw_y = -1;
    for (int y = roi_top; y <= roi_bottom; ++y) {
        for (int x = roi_left; x <= roi_right; ++x) {
            if (!precise_boundary_pixel(rgb565, width, height, x, y)) {
                continue;
            }
            point_t point;
            if (!raw_pixel_to_vehicle_ground_scaled(
                    (float)x, (float)y, width, height, &point)) {
                continue;
            }
            const point_t relative = point_subtract(point, origin);
            const float along_x = dot(relative, axis_x);
            const float along_y = dot(relative, axis_y);
            const float distance2 = along_x * along_x + along_y * along_y;
            if (distance2 < nearest_origin) {
                nearest_origin = distance2;
                origin_raw_x = x;
                origin_raw_y = y;
            }
            if (along_x > maximum_x &&
                fabsf(along_y) <= PRECISE_EDGE_INLIER_MM) {
                maximum_x = along_x;
                x_end_raw_x = x;
                x_end_raw_y = y;
            }
            if (along_y > maximum_y &&
                fabsf(along_x) <= PRECISE_EDGE_INLIER_MM) {
                maximum_y = along_y;
                y_end_raw_x = x;
                y_end_raw_y = y;
            }
        }
    }

    const float radius_mm = (maximum_x + maximum_y) * 0.5f;
    if (origin_raw_x < 0 || x_end_raw_x < 0 || y_end_raw_x < 0 ||
        radius_mm < 55.0f || radius_mm > 145.0f) {
        return false;
    }

    memset(result, 0, sizeof(*result));
    result->found = true;
    result->position_valid = true;
    result->origin_x_raw = origin_raw_x;
    result->origin_y_raw = origin_raw_y;
    result->x_axis_end_x_raw = x_end_raw_x;
    result->x_axis_end_y_raw = x_end_raw_y;
    result->y_axis_end_x_raw = y_end_raw_x;
    result->y_axis_end_y_raw = y_end_raw_y;
    result->origin_vehicle_x_mm = origin.x;
    result->origin_vehicle_y_mm = origin.y;
    result->x_axis_vehicle_x = axis_x.x;
    result->x_axis_vehicle_y = axis_x.y;
    result->y_axis_vehicle_x = axis_y.x;
    result->y_axis_vehicle_y = axis_y.y;
    result->fitted_radius_mm = radius_mm;
    result->position_goal = QUARTER_GOAL_UPPER;
    const int edge_points = clamp_int(
        (refined_x.count + refined_y.count) / 8, 0, 30);
    const int radius_points = clamp_int(
        30 - (int)(fabsf(radius_mm - GOAL_RADIUS_MM) * 0.6f), 0, 30);
    result->confidence = clamp_int(
        coarse_pose->confidence / 2 + edge_points + radius_points, 0, 100);
    map_visual_pose_to_global_field(result);
    return result->field_pose_valid;
}

static int clamp_int(int value, int minimum, int maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

static float dot(point_t a, point_t b)
{
    return a.x * b.x + a.y * b.y;
}

static bool raw_pixel_to_vehicle_ground(float raw_x, float raw_y,
                                        point_t *ground)
{
    /* The mounted sensor is upside down. Convert the raw decoder pixel to
     * the first-person calibration image before removing lens distortion. */
    const float first_x = (CALIBRATED_WIDTH - 1) - raw_x;
    const float first_y = (CALIBRATED_HEIGHT - 1) - raw_y;
    const float distorted_x = (first_x - CX) / FX;
    const float distorted_y = (first_y - CY) / FY;
    float x = distorted_x;
    float y = distorted_y;
    for (int iteration = 0; iteration < 6; ++iteration) {
        const float radius2 = x * x + y * y;
        const float radial = 1.0f + K1 * radius2 +
                             K2 * radius2 * radius2 +
                             K3 * radius2 * radius2 * radius2;
        const float estimate_x = x * radial + 2.0f * P1 * x * y +
                                 P2 * (radius2 + 2.0f * x * x);
        const float estimate_y = y * radial + P1 * (radius2 + 2.0f * y * y) +
                                 2.0f * P2 * x * y;
        x += distorted_x - estimate_x;
        y += distorted_y - estimate_y;
    }

    const float undistorted_x = FX * x + CX;
    const float undistorted_y = FY * y + CY;
    const float denominator = H[2][0] * undistorted_x +
                              H[2][1] * undistorted_y + H[2][2];
    if (fabsf(denominator) < 0.001f) return false;
    ground->x = (H[0][0] * undistorted_x +
                 H[0][1] * undistorted_y + H[0][2]) / denominator;
    ground->y = (H[1][0] * undistorted_x +
                 H[1][1] * undistorted_y + H[1][2]) / denominator;
    return isfinite(ground->x) && isfinite(ground->y) &&
           fabsf(ground->x) < 3000.0f && ground->y > -500.0f &&
           ground->y < 5000.0f;
}

bool quarter_goal_pose_project_ground_pixel(float raw_x, float raw_y,
                                            float *vehicle_right_mm,
                                            float *vehicle_forward_mm)
{
    if (!vehicle_right_mm || !vehicle_forward_mm) return false;
    point_t ground;
    if (!raw_pixel_to_vehicle_ground(raw_x, raw_y, &ground)) return false;
    *vehicle_right_mm = ground.x;
    *vehicle_forward_mm = ground.y;
    return true;
}

static bool selected_pixel(const uint8_t *rgb565, size_t width,
                           const black_marker_result_t *basic,
                           int x, int y, point_t *ground)
{
    if (x < basic->left || x > basic->right || y < basic->top ||
        y > basic->bottom) {
        return false;
    }
    const size_t index = (size_t)y * width + (size_t)x;
    return black_marker_vision_pixel_is_goal(rgb565, index) &&
           raw_pixel_to_vehicle_ground((float)x, (float)y, ground);
}

static pose_candidate_t evaluate_candidate(const uint8_t *rgb565,
                                           size_t width,
                                           const black_marker_result_t *basic,
                                           point_t bisector,
                                           int sample_count)
{
    pose_candidate_t candidate = {0};
    candidate.axis_x = (point_t) {
        .x = (bisector.x + bisector.y) * SQRT_HALF,
        .y = (-bisector.x + bisector.y) * SQRT_HALF,
    };
    candidate.axis_y = (point_t) {
        .x = (bisector.x - bisector.y) * SQRT_HALF,
        .y = (bisector.x + bisector.y) * SQRT_HALF,
    };

    float minimum_x = FLT_MAX;
    float minimum_y = FLT_MAX;
    for (int y = basic->top; y <= basic->bottom; ++y) {
        for (int x = basic->left; x <= basic->right; ++x) {
            point_t point;
            if (!selected_pixel(rgb565, width, basic, x, y, &point)) continue;
            const float projection_x = dot(point, candidate.axis_x);
            const float projection_y = dot(point, candidate.axis_y);
            if (projection_x < minimum_x) minimum_x = projection_x;
            if (projection_y < minimum_y) minimum_y = projection_y;
        }
    }
    if (minimum_x == FLT_MAX || minimum_y == FLT_MAX) return candidate;
    candidate.origin.x = minimum_x * candidate.axis_x.x +
                         minimum_y * candidate.axis_y.x;
    candidate.origin.y = minimum_x * candidate.axis_x.y +
                         minimum_y * candidate.axis_y.y;

    int inside_count = 0;
    int edge_count = 0;
    float maximum_x = 0.0f;
    float maximum_y = 0.0f;
    float maximum_radius = 0.0f;
    for (int y = basic->top; y <= basic->bottom; ++y) {
        for (int x = basic->left; x <= basic->right; ++x) {
            point_t point;
            if (!selected_pixel(rgb565, width, basic, x, y, &point)) continue;
            const point_t relative = {
                .x = point.x - candidate.origin.x,
                .y = point.y - candidate.origin.y,
            };
            const float along_x = dot(relative, candidate.axis_x);
            const float along_y = dot(relative, candidate.axis_y);
            const float radius = sqrtf(along_x * along_x + along_y * along_y);
            if (along_x >= -3.0f && along_y >= -3.0f &&
                radius <= GOAL_RADIUS_MM + 35.0f) {
                inside_count++;
            }
            if (along_x <= EDGE_BAND_MM || along_y <= EDGE_BAND_MM) {
                edge_count++;
            }
            if (along_x > maximum_x) maximum_x = along_x;
            if (along_y > maximum_y) maximum_y = along_y;
            if (radius > maximum_radius) maximum_radius = radius;
        }
    }

    candidate.radius_mm = (maximum_x + maximum_y) * 0.5f;
    const float radius_error = fabsf(candidate.radius_mm - GOAL_RADIUS_MM);
    const float arc_error = fabsf(maximum_radius - GOAL_RADIUS_MM);
    const int inside_percent = inside_count * 100 / sample_count;
    const int edge_percent = edge_count * 100 / sample_count;
    const float robot_x = -dot(candidate.origin, candidate.axis_x);
    const float robot_y = -dot(candidate.origin, candidate.axis_y);
    const int inside_points = clamp_int(inside_percent * 25 / 100, 0, 25);
    const int edge_points = clamp_int(edge_percent, 0, 25);
    const int radius_points = clamp_int(25 - (int)(radius_error * 0.5f),
                                        0, 25);
    const int arc_points = clamp_int(15 - (int)(arc_error * 0.3f), 0, 15);
    const int position_points = robot_x >= -50.0f && robot_y >= -50.0f
                                    ? 10 : 0;
    candidate.confidence = clamp_int(inside_points + edge_points +
                                     radius_points + arc_points +
                                     position_points,
                                     0, 100);
    candidate.valid = candidate.radius_mm >= 55.0f &&
                      candidate.radius_mm <= 145.0f &&
                      maximum_radius <= 155.0f &&
                      candidate.confidence >= 40;
    return candidate;
}

static void set_pixel(uint8_t *pixels, size_t width, size_t height,
                      int x, int y, uint16_t color)
{
    if (x < 0 || y < 0 || x >= (int)width || y >= (int)height) return;
    const size_t offset = 2 * ((size_t)y * width + (size_t)x);
    pixels[offset] = color >> 8;
    pixels[offset + 1] = color & 0xff;
}

static void draw_line(uint8_t *pixels, size_t width, size_t height,
                      int x0, int y0, int x1, int y1, uint16_t color)
{
    const int dx = abs(x1 - x0);
    const int sx = x0 < x1 ? 1 : -1;
    const int dy = -abs(y1 - y0);
    const int sy = y0 < y1 ? 1 : -1;
    int error = dx + dy;
    while (true) {
        set_pixel(pixels, width, height, x0, y0, color);
        if (x0 == x1 && y0 == y1) break;
        const int twice_error = 2 * error;
        if (twice_error >= dy) {
            error += dy;
            x0 += sx;
        }
        if (twice_error <= dx) {
            error += dx;
            y0 += sy;
        }
    }
}

void quarter_goal_pose_process(const uint8_t *rgb565, size_t width,
                               size_t height,
                               const black_marker_result_t *basic_result,
                               quarter_goal_pose_result_t *result)
{
    if (!result) return;
    memset(result, 0, sizeof(*result));
    result->selected_goal = s_selected_goal;
    result->origin_x_raw = -1;
    result->origin_y_raw = -1;
    result->x_axis_end_x_raw = -1;
    result->x_axis_end_y_raw = -1;
    result->y_axis_end_x_raw = -1;
    result->y_axis_end_y_raw = -1;
    if (!rgb565 || !basic_result || !basic_result->found ||
        width != CALIBRATED_WIDTH || height != CALIBRATED_HEIGHT) {
        handle_missing_pose(result, 0);
        return;
    }

    double sum_x = 0.0;
    double sum_y = 0.0;
    int sample_count = 0;
    for (int y = basic_result->top; y <= basic_result->bottom; ++y) {
        for (int x = basic_result->left; x <= basic_result->right; ++x) {
            point_t point;
            if (!selected_pixel(rgb565, width, basic_result, x, y, &point)) {
                continue;
            }
            sum_x += point.x;
            sum_y += point.y;
            sample_count++;
        }
    }
    if (sample_count < MIN_POSE_PIXELS) {
        handle_missing_pose(result, sample_count);
        return;
    }

    const point_t mean = {
        .x = (float)(sum_x / sample_count),
        .y = (float)(sum_y / sample_count),
    };
    double covariance_xx = 0.0;
    double covariance_xy = 0.0;
    double covariance_yy = 0.0;
    for (int y = basic_result->top; y <= basic_result->bottom; ++y) {
        for (int x = basic_result->left; x <= basic_result->right; ++x) {
            point_t point;
            if (!selected_pixel(rgb565, width, basic_result, x, y, &point)) {
                continue;
            }
            const float dx = point.x - mean.x;
            const float dy = point.y - mean.y;
            covariance_xx += dx * dx;
            covariance_xy += dx * dy;
            covariance_yy += dy * dy;
        }
    }
    covariance_xx /= sample_count;
    covariance_xy /= sample_count;
    covariance_yy /= sample_count;
    const float major_angle = 0.5f * atan2f((float)(2.0 * covariance_xy),
                                            (float)(covariance_xx -
                                                    covariance_yy));
    const float bisector_angle = major_angle + PI_F * 0.5f;
    const point_t bisector = {
        .x = cosf(bisector_angle),
        .y = sinf(bisector_angle),
    };
    const point_t opposite_bisector = {
        .x = -bisector.x,
        .y = -bisector.y,
    };
    pose_candidate_t first = evaluate_candidate(rgb565, width, basic_result,
                                                bisector, sample_count);
    pose_candidate_t second = evaluate_candidate(rgb565, width, basic_result,
                                                 opposite_bisector,
                                                 sample_count);
    pose_candidate_t selected = first.confidence >= second.confidence
                                    ? first : second;
    if (!selected.valid) {
        result->confidence = selected.confidence;
        handle_missing_pose(result, sample_count);
        return;
    }

    float closest_origin = FLT_MAX;
    float best_x_axis_score = -FLT_MAX;
    float best_y_axis_score = -FLT_MAX;
    for (int y = basic_result->top; y <= basic_result->bottom; ++y) {
        for (int x = basic_result->left; x <= basic_result->right; ++x) {
            point_t point;
            if (!selected_pixel(rgb565, width, basic_result, x, y, &point)) {
                continue;
            }
            const point_t relative = {
                .x = point.x - selected.origin.x,
                .y = point.y - selected.origin.y,
            };
            const float along_x = dot(relative, selected.axis_x);
            const float along_y = dot(relative, selected.axis_y);
            const float radius2 = along_x * along_x + along_y * along_y;
            if (radius2 < closest_origin) {
                closest_origin = radius2;
                result->origin_x_raw = x;
                result->origin_y_raw = y;
            }
            const float x_axis_score = along_x - 2.0f * fabsf(along_y);
            if (x_axis_score > best_x_axis_score) {
                best_x_axis_score = x_axis_score;
                result->x_axis_end_x_raw = x;
                result->x_axis_end_y_raw = y;
            }
            const float y_axis_score = along_y - 2.0f * fabsf(along_x);
            if (y_axis_score > best_y_axis_score) {
                best_y_axis_score = y_axis_score;
                result->y_axis_end_x_raw = x;
                result->y_axis_end_y_raw = y;
            }
        }
    }

    result->found = true;
    result->confidence = clamp_int((selected.confidence * 3 +
                                    basic_result->confidence) / 4,
                                   0, 100);
    result->origin_vehicle_x_mm = selected.origin.x;
    result->origin_vehicle_y_mm = selected.origin.y;
    result->x_axis_vehicle_x = selected.axis_x.x;
    result->x_axis_vehicle_y = selected.axis_x.y;
    result->y_axis_vehicle_x = selected.axis_y.x;
    result->y_axis_vehicle_y = selected.axis_y.y;
    result->fitted_radius_mm = selected.radius_mm;
    result->position_valid = true;
    if (s_single_corner_mode) {
        /* The only visible corner defines field (0, 0). Its fitted radius
         * edges retain the established +X/+Y convention. */
        result->position_goal = QUARTER_GOAL_UPPER;
        map_visual_pose_to_global_field(result);
        if (s_pose_logging_enabled) {
            ESP_LOGI(TAG,
                     "CORNER_POSE found=1 x_mm=%d y_mm=%d heading_deg=%d confidence=%d corner_px=(%d,%d) radius_mm=%d",
                     (int)lroundf(result->robot_field_x_mm),
                     (int)lroundf(result->robot_field_y_mm),
                     (int)lroundf(result->robot_heading_deg),
                     result->confidence, result->origin_x_raw,
                     result->origin_y_raw,
                     (int)lroundf(result->fitted_radius_mm));
        }
        return;
    }
    update_goal_identity(result);
    result->position_goal = result->candidate_goal != QUARTER_GOAL_UNKNOWN
                                ? result->candidate_goal
                                : result->selected_goal;
    map_visual_pose_to_global_field(result);
    remember_visual_pose(result);

    if (s_pose_logging_enabled) ESP_LOGI(TAG,
             "GOAL_POSE found=1 predicted=0 position_valid=1 field_pose_valid=%d confidence=%d candidate=%s selected=%s position_goal=%s votes=%d/%d odom=%d odom_pose=(%d,%d,%ddeg) goal_field_mm=(%d,%d) remembered_x_mm=%d origin_raw=(%d,%d) origin_vehicle_mm=(%d,%d) robot_field_mm=(%d,%d) heading_deg=%d radius_mm=%d",
             result->field_pose_valid, result->confidence,
             goal_identity_name(result->candidate_goal),
             goal_identity_name(result->selected_goal),
             goal_identity_name(result->position_goal),
             result->upper_goal_votes, result->lower_goal_votes,
             result->odometry_used,
             (int)lroundf(result->odometry_x_mm),
             (int)lroundf(result->odometry_y_mm),
             (int)lroundf(result->odometry_heading_deg),
             (int)lroundf(result->goal_field_x_mm),
             (int)lroundf(result->goal_field_y_mm),
             (int)lroundf(result->remembered_lateral_mm),
             result->origin_x_raw, result->origin_y_raw,
             (int)lroundf(result->origin_vehicle_x_mm),
             (int)lroundf(result->origin_vehicle_y_mm),
             (int)lroundf(result->robot_field_x_mm),
             (int)lroundf(result->robot_field_y_mm),
             (int)lroundf(result->robot_heading_deg),
             (int)lroundf(result->fitted_radius_mm));
}

void quarter_goal_pose_draw_overlay(uint8_t *rgb565, size_t width,
                                    size_t height,
                                    const quarter_goal_pose_result_t *result)
{
    if (!rgb565 || !result || !result->found) return;
    /* Red and blue show the fitted axes. Origin color shows goal identity. */
    draw_line(rgb565, width, height, result->origin_x_raw,
              result->origin_y_raw, result->x_axis_end_x_raw,
              result->x_axis_end_y_raw, 0xf800);
    draw_line(rgb565, width, height, result->origin_x_raw,
              result->origin_y_raw, result->y_axis_end_x_raw,
              result->y_axis_end_y_raw, 0x001f);
    const quarter_goal_identity_t displayed_goal =
        result->selected_goal != QUARTER_GOAL_UNKNOWN
            ? result->selected_goal : result->candidate_goal;
    const uint16_t origin_color =
        displayed_goal == QUARTER_GOAL_UPPER ? 0x07e0 :
        displayed_goal == QUARTER_GOAL_LOWER ? 0xf81f : 0xffff;
    for (int delta = -3; delta <= 3; ++delta) {
        set_pixel(rgb565, width, height, result->origin_x_raw + delta,
                  result->origin_y_raw, origin_color);
        set_pixel(rgb565, width, height, result->origin_x_raw,
                  result->origin_y_raw + delta, origin_color);
    }
}
