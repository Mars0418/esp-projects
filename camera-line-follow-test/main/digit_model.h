#pragma once
#include <stdint.h>

/* Single caller at a time per workspace. 32 KB scratch; weights stay in flash. */
typedef struct {
    float a[8 * 28 * 28];
    float b[8 * 14 * 14];
    float c[16 * 4 * 4];
} digit_model_workspace_t;

void digit_model_predict(digit_model_workspace_t *work, const uint8_t image[784],
                         float logits[10], float probabilities[10]);
int digit_model_selftest(digit_model_workspace_t *work, float *max_error);
