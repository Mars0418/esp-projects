#pragma once
#include <stdbool.h>
#include <stdint.h>

/* Coordinates below refer to the clockwise-rotated 120x160 TFT preview. */
#define DIGIT_ROI_X 10
#define DIGIT_ROI_Y 30
#define DIGIT_ROI_SIZE 100
typedef struct {
    uint8_t gray[120 * 160];
    uint16_t labels[120 * 160];
    uint16_t queue[120 * 160];
} digit_vision_workspace_t;
typedef struct {
    bool valid;
    int x, y, width, height, area, threshold, contrast;
    const char *reason;
} digit_region_t;
digit_region_t digit_vision_prepare(digit_vision_workspace_t *s,
                                    const uint8_t rgb565[160 * 120 * 2],
                                    unsigned rotation, bool mirror,
                                    uint8_t input[784]);
