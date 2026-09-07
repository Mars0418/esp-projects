#pragma once
#include "esp_err.h"

/* Read cached descriptors only; does not claim interfaces or start audio. */
esp_err_t usb_av_probe_start(void);
