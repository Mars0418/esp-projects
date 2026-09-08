#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "line_vision.h"

typedef struct {
    bool valid;
    bool enabled;
    char state[24];
    int pwm_a;
    int pwm_d;
    int boost_a;
    int boost_d;
    int encoder_delta_a;
    int encoder_delta_d;
    int distance_mm;
} camera_line_follow_debug_status_t;

esp_err_t camera_line_follow_init(void);
void camera_line_follow_submit(const line_vision_result_t *result,
                               int64_t captured_at_us);
void camera_line_follow_camera_disconnected(void);
bool camera_line_follow_debug_enabled(void);
bool camera_line_follow_tuner_enabled(void);
bool camera_line_follow_calibration_enabled(void);
bool camera_line_follow_take_course_complete(void);
esp_err_t camera_line_follow_handoff(void);
void camera_line_follow_get_encoder_counts(int32_t *count_a,
                                           int32_t *count_b,
                                           int32_t *count_d);
bool camera_line_follow_get_debug_status(
    camera_line_follow_debug_status_t *status);
