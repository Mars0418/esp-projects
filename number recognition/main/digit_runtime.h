#pragma once
#include <stdint.h>
#include "esp_err.h"
esp_err_t digit_runtime_init(void);
void digit_runtime_process(const uint8_t *frame, int64_t captured_at_us);
void digit_runtime_idle(void);
void digit_runtime_disconnected(void);
