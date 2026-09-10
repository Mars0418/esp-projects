#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

esp_err_t xiaozhi_wake_init(void);
bool xiaozhi_wake_ready(void);
void xiaozhi_wake_feed(const int16_t *pcm); /* 960 samples, local only */
