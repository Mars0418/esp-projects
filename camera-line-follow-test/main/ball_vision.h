#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

/* Red billiard-ball tracker.  The coloured shell is the detection cue;
 * bright white markings/highlights are deliberately allowed inside the box. */
typedef struct {
    bool found;
    bool predicted;
    int center_x;
    int center_y;
    int left;
    int top;
    int right;
    int bottom;
    int purple_pixels;
    int confidence;
} ball_vision_result_t;

esp_err_t ball_vision_init(size_t width, size_t height);
esp_err_t white_ball_vision_init(size_t width, size_t height);
void ball_vision_process(const uint8_t *rgb565, size_t width, size_t height,
                         ball_vision_result_t *result);
void white_ball_vision_process(const uint8_t *rgb565, size_t width,
                               size_t height, ball_vision_result_t *result);
void ball_vision_draw_overlay(uint8_t *rgb565, size_t width, size_t height,
                              const ball_vision_result_t *result);
void ball_vision_draw_overlay_color(uint8_t *rgb565, size_t width,
                                    size_t height,
                                    const ball_vision_result_t *result,
                                    uint16_t color);
