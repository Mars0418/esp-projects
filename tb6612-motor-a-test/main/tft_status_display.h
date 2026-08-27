#pragma once

#include "esp_err.h"

esp_err_t tft_status_display_start(void);
void tft_status_display_set(int rpm_a, int rpm_b, int rpm_d,
                            int distance_mm);
