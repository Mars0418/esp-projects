#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
esp_err_t tour_guide_init(void);
void tour_guide_process_digit(const uint8_t *frame, int64_t captured_us);
int tour_guide_route(void);
bool tour_guide_waiting(void);
bool tour_guide_departure_ready(int64_t now_us);
void tour_guide_stop(void);
void tour_guide_digit_status(int *digit, int *score);

bool tour_guide_begin(void);
void tour_guide_enable_scan(int64_t now_us);
bool tour_guide_scanning(void);
