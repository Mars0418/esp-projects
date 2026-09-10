#pragma once
#include "ball_vision.h"
#include "black_marker_vision.h"

esp_err_t wifi_debug_init(void);
bool wifi_debug_uart_command(const char *line);
void wifi_debug_publish(const uint8_t *rgb565, size_t width, size_t height,
                        int64_t captured_us,
                        const ball_vision_result_t *red,
                        const ball_vision_result_t *white,
                        const ball_vision_result_t *purple,
                        const black_marker_result_t *goal);
