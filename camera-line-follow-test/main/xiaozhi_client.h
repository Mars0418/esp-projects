#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#define XZ_PCM_SAMPLES 960
typedef struct { uint16_t samples; int16_t pcm[1920]; } xz_playback_t;
esp_err_t xiaozhi_init(void);
bool xiaozhi_command(const char *command);
bool xiaozhi_wants_microphone(void);
bool xiaozhi_allows_playback(void);
bool xiaozhi_push_pcm(const int16_t *pcm); /* exactly 960 samples */
bool xiaozhi_take_playback(xz_playback_t *frame);
void xiaozhi_audio_fault(void);
