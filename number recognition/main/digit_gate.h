#pragma once
#include <stdbool.h>
#include <stdint.h>
#define DIGIT_MIN_SCORE 0.70f
#define DIGIT_CONFIRM_FRAMES 3
typedef struct {
    int candidate, streak, empty_frames;
    int64_t last_ms, since_ms, empty_since_ms;
    bool latched;
} digit_gate_t;
void digit_gate_reset(digit_gate_t *s);
/* Returns a digit only on a new confirmed presentation; otherwise -1. */
int digit_gate_update(digit_gate_t *s, int digit, bool has_region,
                       float score, float margin, int64_t now_ms);
