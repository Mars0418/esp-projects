#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

esp_err_t camera_display_init(void);
esp_err_t camera_display_show_telemetry(int rpm_a, int rpm_b, int rpm_d,
                                        int distance_mm,
                                        bool distance_valid);
