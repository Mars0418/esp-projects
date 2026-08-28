#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

esp_err_t camera_display_init(void);
esp_err_t camera_display_show_waiting(void);
esp_err_t camera_display_show_rotated_rgb565(const uint8_t *pixels,
                                              size_t width,
                                              size_t height);
