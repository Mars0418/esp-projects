#pragma once

#include "esp_err.h"
#include "line_vision.h"

esp_err_t camera_line_follow_init(void);
void camera_line_follow_submit(const line_vision_result_t *result);
void camera_line_follow_camera_disconnected(void);
