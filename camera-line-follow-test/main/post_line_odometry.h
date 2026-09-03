#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

typedef struct {
    bool valid;
    float x_mm;
    float y_mm;
    float heading_rad;
    float heading_deg;
    int32_t count_a;
    int32_t count_b;
    int32_t count_d;
} post_line_odometry_pose_t;

esp_err_t post_line_odometry_init(void);
bool post_line_odometry_get_pose(post_line_odometry_pose_t *pose);
