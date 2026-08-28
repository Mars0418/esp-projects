#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

typedef struct {
    bool found;
    int near_x;
    int far_x;
    int steering_error;
    int confidence;
    int threshold;
    int contrast;
    int component_area;
} line_vision_result_t;

esp_err_t line_vision_init(size_t width, size_t height);
void line_vision_process(uint8_t *rgb565, size_t width, size_t height,
                         line_vision_result_t *result);
