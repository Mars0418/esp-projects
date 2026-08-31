#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

typedef struct {
    bool initialized;
    float inverse_kinematics[3][3];
    float millimeters_per_count[3];
    int32_t previous_counts[3];
    float x_mm;
    float y_mm;
    float heading_rad;
} three_wheel_odometry_t;

typedef struct {
    float x_mm;
    float y_mm;
    float heading_rad;
    float heading_deg;
    float delta_body_x_mm;
    float delta_body_y_mm;
    float delta_heading_rad;
} three_wheel_pose_t;

esp_err_t three_wheel_odometry_init(three_wheel_odometry_t *odometry,
                                    int32_t count_a,
                                    int32_t count_b,
                                    int32_t count_d);

bool three_wheel_odometry_update(three_wheel_odometry_t *odometry,
                                 int32_t count_a,
                                 int32_t count_b,
                                 int32_t count_d,
                                 three_wheel_pose_t *pose);

void three_wheel_odometry_reset(three_wheel_odometry_t *odometry,
                                int32_t count_a,
                                int32_t count_b,
                                int32_t count_d);
