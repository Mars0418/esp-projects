#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

typedef struct {
    bool found;
    int left;
    int top;
    int right;
    int bottom;
    int center_x;
    int center_y;
    int dark_pixels;
} black_marker_result_t;

esp_err_t black_marker_vision_init(size_t width, size_t height);
void black_marker_vision_process(const uint8_t *rgb565, size_t width,
                                 size_t height,
                                 black_marker_result_t *result);
void black_marker_vision_draw_overlay(uint8_t *rgb565, size_t width,
                                      size_t height,
                                      const black_marker_result_t *result);
