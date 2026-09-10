#pragma once
#include <stdbool.h>
#include <stdint.h>
typedef struct {
    int32_t start[3], previous[3];
    int64_t started_us, stable_since_us;
    float target_deg, angle_deg;
    bool done, failed;
} tour_start_turn_t;
void tour_start_turn_begin(tour_start_turn_t *s, int route, const int32_t counts[3], int64_t now);
/* Returns -1/0/+1 for clockwise/brake/counterclockwise. */
int tour_start_turn_step(tour_start_turn_t *s, const int32_t counts[3], int64_t now);
