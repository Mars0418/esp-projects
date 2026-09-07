#pragma once

#include "ball_vision.h"

/* Purple detector uses the same connected-component tracker as the red ball.
 * Only the calibrated shell-colour parameters differ. */
esp_err_t purple_ball_vision_init(size_t width, size_t height);
void purple_ball_vision_process(const uint8_t *rgb565, size_t width,
                                size_t height, ball_vision_result_t *result);
void purple_ball_vision_draw_overlay(uint8_t *rgb565, size_t width,
                                     size_t height,
                                     const ball_vision_result_t *result);
void purple_ball_vision_draw_overlay_color(
    uint8_t *rgb565, size_t width, size_t height,
    const ball_vision_result_t *result, uint16_t color);
