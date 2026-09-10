#include "digit_model.h"
#include "model_weights.h"
#include "model_selftest.h"
#include <math.h>

/* Exact NCHW graph of the pinned ONNX model: Conv SAME + ReLU + MaxPool,
 * Conv SAME + ReLU + MaxPool, row-major flatten + MatMul + bias.
 * FP32 is intentional for this tiny baseline; no runtime ONNX dependency. */
void digit_model_predict(digit_model_workspace_t *s, const uint8_t image[784],
                         float logits[10], float probabilities[10])
{
    for (int oc = 0; oc < 8; ++oc) {
        for (int y = 0; y < 28; ++y) {
            for (int x = 0; x < 28; ++x) {
                float sum = 0;
                for (int ky = 0; ky < 5; ++ky) {
                    int iy = y + ky - 2;
                    if (iy < 0 || iy >= 28) continue;
                    for (int kx = 0; kx < 5; ++kx) {
                        int ix = x + kx - 2;
                        if (ix >= 0 && ix < 28)
                            sum += ((float)image[iy * 28 + ix] / 255.0f) *
                                   conv1_w[oc * 25 + ky * 5 + kx];
                    }
                }
                sum += conv1_b[oc];
                s->a[(oc * 28 + y) * 28 + x] = fmaxf(0, sum);
            }
        }
    }
    for (int c = 0; c < 8; ++c)
        for (int y = 0; y < 14; ++y)
            for (int x = 0; x < 14; ++x) {
                float v = 0;
                for (int ky = 0; ky < 2; ++ky)
                    for (int kx = 0; kx < 2; ++kx)
                        v = fmaxf(v, s->a[(c * 28 + y * 2 + ky) * 28 + x * 2 + kx]);
                s->b[(c * 14 + y) * 14 + x] = v;
            }
    for (int oc = 0; oc < 16; ++oc)
        for (int y = 0; y < 14; ++y)
            for (int x = 0; x < 14; ++x) {
                float sum = 0;
                for (int ic = 0; ic < 8; ++ic)
                    for (int ky = 0; ky < 5; ++ky) {
                        int iy = y + ky - 2;
                        if (iy < 0 || iy >= 14) continue;
                        for (int kx = 0; kx < 5; ++kx) {
                            int ix = x + kx - 2;
                            if (ix >= 0 && ix < 14)
                                sum += s->b[(ic * 14 + iy) * 14 + ix] *
                                       conv2_w[((oc * 8 + ic) * 5 + ky) * 5 + kx];
                        }
                    }
                sum += conv2_b[oc];
                s->a[(oc * 14 + y) * 14 + x] = fmaxf(0, sum);
            }
    for (int c = 0; c < 16; ++c)
        for (int y = 0; y < 4; ++y)
            for (int x = 0; x < 4; ++x) {
                float v = 0;
                for (int ky = 0; ky < 3; ++ky)
                    for (int kx = 0; kx < 3; ++kx)
                        v = fmaxf(v, s->a[(c * 14 + y * 3 + ky) * 14 + x * 3 + kx]);
                s->c[(c * 4 + y) * 4 + x] = v;
            }
    float max_logit = -INFINITY;
    for (int digit = 0; digit < 10; ++digit) {
        float sum = 0;
        for (int i = 0; i < 256; ++i) sum += s->c[i] * dense_w[i * 10 + digit];
        logits[digit] = sum + dense_b[digit];
        max_logit = fmaxf(max_logit, logits[digit]);
    }
    float total = 0;
    for (int i = 0; i < 10; ++i) {
        probabilities[i] = expf(logits[i] - max_logit);
        total += probabilities[i];
    }
    for (int i = 0; i < 10; ++i) probabilities[i] /= total;
}

int digit_model_selftest(digit_model_workspace_t *s, float *max_error)
{
    int passed = 0;
    *max_error = 0;
    for (int digit = 0; digit < 10; ++digit) {
        float logits[10], probabilities[10];
        digit_model_predict(s, selftest_images[digit], logits, probabilities);
        int best = 0;
        for (int j = 0; j < 10; ++j) {
            if (probabilities[j] > probabilities[best]) best = j;
            *max_error = fmaxf(*max_error, fabsf(logits[j] - selftest_logits[digit][j]));
        }
        passed += best == digit;
    }
    return passed;
}
