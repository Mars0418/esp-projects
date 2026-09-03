#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "black_marker_vision.h"

typedef enum {
    QUARTER_GOAL_UNKNOWN = 0,
    QUARTER_GOAL_UPPER,
    QUARTER_GOAL_LOWER,
} quarter_goal_identity_t;

typedef struct {
    bool found;
    bool position_valid;
    bool field_pose_valid;
    bool odometry_predicted;
    int confidence;
    quarter_goal_identity_t candidate_goal;
    quarter_goal_identity_t selected_goal;
    quarter_goal_identity_t position_goal;
    int upper_goal_votes;
    int lower_goal_votes;
    bool odometry_used;
    float odometry_x_mm;
    float odometry_y_mm;
    float odometry_heading_deg;
    float remembered_lateral_mm;
    int origin_x_raw;
    int origin_y_raw;
    int x_axis_end_x_raw;
    int x_axis_end_y_raw;
    int y_axis_end_x_raw;
    int y_axis_end_y_raw;
    float origin_vehicle_x_mm;
    float origin_vehicle_y_mm;
    float x_axis_vehicle_x;
    float x_axis_vehicle_y;
    float y_axis_vehicle_x;
    float y_axis_vehicle_y;
    float goal_field_x_mm;
    float goal_field_y_mm;
    float robot_field_x_mm;
    float robot_field_y_mm;
    float robot_heading_deg;
    float fitted_radius_mm;
} quarter_goal_pose_result_t;

void quarter_goal_pose_set_relative_odometry(bool valid, float x_mm,
                                             float y_mm,
                                             float heading_rad);
void quarter_goal_pose_set_single_corner_mode(bool enabled);
void quarter_goal_pose_set_logging(bool enabled);
void quarter_goal_pose_process(const uint8_t *rgb565, size_t width,
                               size_t height,
                               const black_marker_result_t *basic_result,
                               quarter_goal_pose_result_t *result);
bool quarter_goal_pose_refine_single_corner(
    const uint8_t *rgb565, size_t width, size_t height,
    const black_marker_result_t *coarse_basic,
    const quarter_goal_pose_result_t *coarse_pose,
    quarter_goal_pose_result_t *result);
void quarter_goal_pose_draw_overlay(uint8_t *rgb565, size_t width,
                                    size_t height,
                                    const quarter_goal_pose_result_t *result);
