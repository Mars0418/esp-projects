#pragma once
#include "esp_err.h"

/* USB PCM bring-up for XiaoZhi. UART commands only; no cloud upload. */
esp_err_t usb_audio_init(void);
