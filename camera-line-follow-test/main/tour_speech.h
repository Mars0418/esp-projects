#pragma once
#include <stdbool.h>
#include "esp_err.h"
esp_err_t tour_speech_init(void);
bool tour_speech_play(int clip);
void tour_speech_cancel(void);
bool tour_speech_ready(void);
bool tour_speech_busy(void);
bool tour_speech_failed(void);
int tour_station_clip(int route, int corner, bool finished);
