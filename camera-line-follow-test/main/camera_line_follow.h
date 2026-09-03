#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "line_vision.h"

esp_err_t camera_line_follow_init(void);
void camera_line_follow_submit(const line_vision_result_t *result,
                               int64_t captured_at_us);
void camera_line_follow_camera_disconnected(void);
bool camera_line_follow_debug_enabled(void);
bool camera_line_follow_tuner_enabled(void);
bool camera_line_follow_calibration_enabled(void);
void camera_line_follow_get_encoder_counts(int32_t *count_a,
                                           int32_t *count_b,
                                           int32_t *count_d);
