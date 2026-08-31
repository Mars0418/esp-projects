#include "three_wheel_odometry.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "three_wheel_odometry_config.h"

#define ODOM_PI 3.14159265358979323846f
#define ODOM_DETERMINANT_EPSILON 0.000001f

typedef struct {
    float counts_per_revolution;
    float wheel_radius_mm;
    float encoder_sign;
    float position_radius_mm;
    float position_angle_deg;
    float drive_angle_deg;
} wheel_geometry_t;

static const wheel_geometry_t s_geometry[3] = {
    {ODOM_A_COUNTS_PER_REV, ODOM_A_WHEEL_RADIUS_MM, ODOM_A_ENCODER_SIGN,
     ODOM_A_POSITION_RADIUS_MM, ODOM_A_POSITION_ANGLE_DEG,
     ODOM_A_DRIVE_ANGLE_DEG},
    {ODOM_B_COUNTS_PER_REV, ODOM_B_WHEEL_RADIUS_MM, ODOM_B_ENCODER_SIGN,
     ODOM_B_POSITION_RADIUS_MM, ODOM_B_POSITION_ANGLE_DEG,
     ODOM_B_DRIVE_ANGLE_DEG},
    {ODOM_D_COUNTS_PER_REV, ODOM_D_WHEEL_RADIUS_MM, ODOM_D_ENCODER_SIGN,
     ODOM_D_POSITION_RADIUS_MM, ODOM_D_POSITION_ANGLE_DEG,
     ODOM_D_DRIVE_ANGLE_DEG},
};

static float degrees_to_radians(float degrees)
{
    return degrees * ODOM_PI / 180.0f;
}

static float normalize_heading(float radians)
{
    while (radians > ODOM_PI) radians -= 2.0f * ODOM_PI;
    while (radians <= -ODOM_PI) radians += 2.0f * ODOM_PI;
    return radians;
}

static bool invert_3x3(const float matrix[3][3], float inverse[3][3])
{
    const float determinant =
        matrix[0][0] * (matrix[1][1] * matrix[2][2] -
                        matrix[1][2] * matrix[2][1]) -
        matrix[0][1] * (matrix[1][0] * matrix[2][2] -
                        matrix[1][2] * matrix[2][0]) +
        matrix[0][2] * (matrix[1][0] * matrix[2][1] -
                        matrix[1][1] * matrix[2][0]);
    if (fabsf(determinant) < ODOM_DETERMINANT_EPSILON) return false;

    const float reciprocal = 1.0f / determinant;
    inverse[0][0] = (matrix[1][1] * matrix[2][2] -
                     matrix[1][2] * matrix[2][1]) * reciprocal;
    inverse[0][1] = (matrix[0][2] * matrix[2][1] -
                     matrix[0][1] * matrix[2][2]) * reciprocal;
    inverse[0][2] = (matrix[0][1] * matrix[1][2] -
                     matrix[0][2] * matrix[1][1]) * reciprocal;
    inverse[1][0] = (matrix[1][2] * matrix[2][0] -
                     matrix[1][0] * matrix[2][2]) * reciprocal;
    inverse[1][1] = (matrix[0][0] * matrix[2][2] -
                     matrix[0][2] * matrix[2][0]) * reciprocal;
    inverse[1][2] = (matrix[0][2] * matrix[1][0] -
                     matrix[0][0] * matrix[1][2]) * reciprocal;
    inverse[2][0] = (matrix[1][0] * matrix[2][1] -
                     matrix[1][1] * matrix[2][0]) * reciprocal;
    inverse[2][1] = (matrix[0][1] * matrix[2][0] -
                     matrix[0][0] * matrix[2][1]) * reciprocal;
    inverse[2][2] = (matrix[0][0] * matrix[1][1] -
                     matrix[0][1] * matrix[1][0]) * reciprocal;
    return true;
}

static int32_t signed_count_delta(int32_t current, int32_t previous)
{
    return (int32_t)((uint32_t)current - (uint32_t)previous);
}

esp_err_t three_wheel_odometry_init(three_wheel_odometry_t *odometry,
                                    int32_t count_a,
                                    int32_t count_b,
                                    int32_t count_d)
{
    if (odometry == NULL) return ESP_ERR_INVALID_ARG;

    memset(odometry, 0, sizeof(*odometry));
    float kinematics[3][3] = {0};
    for (size_t wheel = 0; wheel < 3; ++wheel) {
        if (s_geometry[wheel].counts_per_revolution <= 0.0f ||
            s_geometry[wheel].wheel_radius_mm <= 0.0f ||
            s_geometry[wheel].position_radius_mm <= 0.0f ||
            s_geometry[wheel].encoder_sign == 0.0f) {
            return ESP_ERR_INVALID_ARG;
        }
        const float position_angle =
            degrees_to_radians(s_geometry[wheel].position_angle_deg);
        const float drive_angle =
            degrees_to_radians(s_geometry[wheel].drive_angle_deg);
        const float position_x =
            s_geometry[wheel].position_radius_mm * cosf(position_angle);
        const float position_y =
            s_geometry[wheel].position_radius_mm * sinf(position_angle);
        const float drive_x = cosf(drive_angle);
        const float drive_y = sinf(drive_angle);

        kinematics[wheel][0] = drive_x;
        kinematics[wheel][1] = drive_y;
        kinematics[wheel][2] =
            -position_y * drive_x + position_x * drive_y;
        odometry->millimeters_per_count[wheel] =
            s_geometry[wheel].encoder_sign * 2.0f * ODOM_PI *
            s_geometry[wheel].wheel_radius_mm /
            s_geometry[wheel].counts_per_revolution;
    }

    if (!invert_3x3(kinematics, odometry->inverse_kinematics)) {
        return ESP_ERR_INVALID_STATE;
    }
    odometry->previous_counts[0] = count_a;
    odometry->previous_counts[1] = count_b;
    odometry->previous_counts[2] = count_d;
    odometry->initialized = true;
    return ESP_OK;
}

bool three_wheel_odometry_update(three_wheel_odometry_t *odometry,
                                 int32_t count_a,
                                 int32_t count_b,
                                 int32_t count_d,
                                 three_wheel_pose_t *pose)
{
    if (odometry == NULL || pose == NULL || !odometry->initialized) {
        return false;
    }

    const int32_t current_counts[3] = {count_a, count_b, count_d};
    float wheel_distance_mm[3];
    for (size_t wheel = 0; wheel < 3; ++wheel) {
        const int32_t delta = signed_count_delta(
            current_counts[wheel], odometry->previous_counts[wheel]);
        odometry->previous_counts[wheel] = current_counts[wheel];
        wheel_distance_mm[wheel] =
            delta * odometry->millimeters_per_count[wheel];
    }

    float body_delta[3] = {0};
    for (size_t row = 0; row < 3; ++row) {
        for (size_t column = 0; column < 3; ++column) {
            body_delta[row] += odometry->inverse_kinematics[row][column] *
                               wheel_distance_mm[column];
        }
    }

    const float heading_mid = odometry->heading_rad + body_delta[2] * 0.5f;
    const float cosine = cosf(heading_mid);
    const float sine = sinf(heading_mid);
    odometry->x_mm += cosine * body_delta[0] - sine * body_delta[1];
    odometry->y_mm += sine * body_delta[0] + cosine * body_delta[1];
    odometry->heading_rad = normalize_heading(
        odometry->heading_rad + body_delta[2]);

    pose->x_mm = odometry->x_mm;
    pose->y_mm = odometry->y_mm;
    pose->heading_rad = odometry->heading_rad;
    pose->heading_deg = odometry->heading_rad * 180.0f / ODOM_PI;
    pose->delta_body_x_mm = body_delta[0];
    pose->delta_body_y_mm = body_delta[1];
    pose->delta_heading_rad = body_delta[2];
    return true;
}

void three_wheel_odometry_reset(three_wheel_odometry_t *odometry,
                                int32_t count_a,
                                int32_t count_b,
                                int32_t count_d)
{
    if (odometry == NULL || !odometry->initialized) return;
    odometry->previous_counts[0] = count_a;
    odometry->previous_counts[1] = count_b;
    odometry->previous_counts[2] = count_d;
    odometry->x_mm = 0.0f;
    odometry->y_mm = 0.0f;
    odometry->heading_rad = 0.0f;
}
