#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#define LINE_VISION_TURN_TRIGGER_Y 21

typedef struct {
    bool found;
    bool big_turn;
    int near_x;
    int far_x;
    int lateral_error;
    int heading_error;
    int steering_error;
    int turn_direction;
    int turn_angle_deg;
    int turn_confidence;
    int corner_x;
    int corner_y;
    int vector_point_count;
    int path_point_count;
    int confidence;
    int threshold;
    int contrast;
    int component_area;
} line_vision_result_t;

typedef struct {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
} line_vision_rgb_thresholds_t;

esp_err_t line_vision_init(size_t width, size_t height);
void line_vision_set_rgb_thresholds(uint8_t red, uint8_t green, uint8_t blue);
line_vision_rgb_thresholds_t line_vision_get_rgb_thresholds(void);
bool line_vision_pixel_selected(size_t pixel_index);
void line_vision_process(uint8_t *rgb565, size_t width, size_t height,
                         line_vision_result_t *result);
