#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

typedef enum {
    POST_NAV_WAITING = 0,
    POST_NAV_RUNNING,
    POST_NAV_PAUSED,
    POST_NAV_SETTLING,
    POST_NAV_COMPLETE,
    POST_NAV_STOPPED,
} post_line_navigation_state_t;

typedef enum {
    POST_NAV_COMMAND_NONE = 0,
    POST_NAV_COMMAND_MOVE,
    POST_NAV_COMMAND_ROTATE,
    POST_NAV_COMMAND_PUSH,
    POST_NAV_COMMAND_REVERSE,
    POST_NAV_COMMAND_PATH,
} post_line_navigation_command_t;

#define POST_LINE_NAVIGATION_MAX_PATH_POINTS 8

typedef struct {
    float x_mm;
    float y_mm;
} post_line_navigation_waypoint_t;

typedef struct {
    bool valid;
    float x_mm;
    float y_mm;
    float heading_rad;
    float heading_deg;
    post_line_navigation_state_t state;
    post_line_navigation_command_t command;
    uint32_t command_id;
    int waypoint_index;
    float target_x_mm;
    float target_y_mm;
    float distance_to_target_mm;
    float target_heading_deg;
} post_line_navigation_pose_t;

esp_err_t post_line_navigation_init(float initial_x_mm, float initial_y_mm,
                                    float initial_heading_deg);
void post_line_navigation_start(void);
void post_line_navigation_pause(void);
void post_line_navigation_resume(void);
void post_line_navigation_stop(void);
bool post_line_navigation_move_to(float x_mm, float y_mm,
                                  float tolerance_mm, float speed_scale,
                                  uint32_t *command_id);
bool post_line_navigation_rotate_to(float heading_deg, float speed_scale,
                                    uint32_t *command_id);
bool post_line_navigation_push_to(float x_mm, float y_mm,
                                  float heading_deg, float tolerance_mm,
                                  float speed_scale, uint32_t *command_id);
void post_line_navigation_set_visual_push_error(float right_error_normalized,
                                                bool valid);
bool post_line_navigation_reverse_by(float distance_mm, float speed_scale,
                                     uint32_t *command_id);
bool post_line_navigation_follow_path(
    const post_line_navigation_waypoint_t *points, size_t point_count,
    float final_heading_deg, float final_tolerance_mm, float speed_scale,
    uint32_t *command_id);
bool post_line_navigation_correct_pose(float x_mm, float y_mm,
                                       float heading_deg);
bool post_line_navigation_get_pose(post_line_navigation_pose_t *pose);
const char *post_line_navigation_state_name(post_line_navigation_state_t state);
