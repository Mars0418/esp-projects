#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "line_vision.h"

typedef struct {
    bool found;
    bool predicted;
    int confidence;
    int center_x;
    int center_y;
    int left;
    int top;
    int right;
    int bottom;
} push_debug_detection_t;

typedef struct {
    const char *phase_name;
    int64_t captured_at_us;
    int64_t processed_at_us;
    int mission_state;
    const char *mission_state_name;
    int push_entry;
    const char *push_entry_name;
    int selected_ball;
    int target_goal;
    bool ball_held;
    bool visual_goal_locked;
    float filtered_goal_right_mm;
    float filtered_goal_forward_mm;
    float tracked_ball_field_x_mm;
    float tracked_ball_field_y_mm;
    push_debug_detection_t red_ball;
    push_debug_detection_t purple_ball;
    push_debug_detection_t goal;
    bool corner_found;
    int corner_confidence;
    int goal_ball_gap_mm;
    int corner_x;
    int corner_y;
    bool navigation_valid;
    int navigation_state;
    int navigation_command;
    bool visual_control_active;
    float vehicle_x_mm;
    float vehicle_y_mm;
    float vehicle_heading_deg;
    float navigation_target_x_mm;
    float navigation_target_y_mm;
    float navigation_distance_mm;
    float visual_target_right_mm;
    float visual_target_forward_mm;
    int reverse_stall_duty_floor;
    int wheel_pwm_a;
    int wheel_pwm_b;
    int wheel_pwm_d;
    int encoder_delta_a;
    int encoder_delta_b;
    int encoder_delta_d;
    int stall_pulse_mask;
    line_vision_result_t line;
    bool line_control_valid;
    bool line_control_enabled;
    const char *line_control_state_name;
    int line_pwm_a;
    int line_pwm_d;
    int line_boost_a;
    int line_boost_d;
    int line_encoder_delta_a;
    int line_encoder_delta_d;
    int line_distance_mm;
} push_debug_metadata_t;

esp_err_t push_debug_stream_init(void);
esp_err_t push_debug_stream_set_enabled(bool enabled);
bool push_debug_stream_is_enabled(void);
bool push_debug_stream_queue_frame(const uint8_t *rgb565, size_t width,
                                   size_t height,
                                   const push_debug_metadata_t *metadata);
