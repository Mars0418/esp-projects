#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

typedef struct {
    uint8_t luminance_max;
    uint8_t channel_max;
    uint8_t rgb_spread_max;
} goal_dark_thresholds_t;

/* The historical type name is retained so existing callers keep compiling.
 * It now describes the selected black quarter-circle goal component. */
typedef struct {
    bool found;
    bool predicted;
    int left;
    int top;
    int right;
    int bottom;
    int center_x;
    int center_y;
    int color_pixels;
    int confidence;
    int dark_pixels; /* Deprecated compatibility alias for color_pixels. */
} black_marker_result_t;

esp_err_t black_marker_vision_init(size_t width, size_t height);
void black_marker_vision_process(const uint8_t *rgb565, size_t width,
                                 size_t height,
                                 black_marker_result_t *result);
void black_marker_vision_draw_overlay(uint8_t *rgb565, size_t width,
                                      size_t height,
                                      const black_marker_result_t *result);

goal_dark_thresholds_t black_marker_vision_get_thresholds(void);
void black_marker_vision_set_thresholds(goal_dark_thresholds_t thresholds);
void black_marker_vision_set_logging(bool enabled);
bool black_marker_vision_pixel_is_goal(const uint8_t *rgb565, size_t index);
