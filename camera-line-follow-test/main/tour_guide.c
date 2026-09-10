#include "tour_guide.h"
#include "digit_model.h"
#include "digit_vision.h"
#include "digit_gate.h"
#include "camera_display.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include <stdatomic.h>

static digit_model_workspace_t *model;
static digit_vision_workspace_t *vision;
static digit_gate_t gate;
static atomic_bool stopped;
static atomic_int route;
static atomic_int scan_phase; /* 0=idle, 1=welcome, 2=scan */
static int64_t scan_after_us;
bool tour_guide_begin(void) { int expected=0; return !atomic_load(&stopped) && atomic_compare_exchange_strong(&scan_phase,&expected,1); }
static int64_t selected_us;
static int last_digit = -1, last_score;
void tour_guide_digit_status(int *digit, int *score) { *digit=last_digit; *score=last_score; }

esp_err_t tour_guide_init(void)
{
    model = heap_caps_calloc(1, sizeof(*model), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    vision = heap_caps_calloc(1, sizeof(*vision), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!model || !vision) return ESP_ERR_NO_MEM;
    float error;
    if (digit_model_selftest(model, &error) != 10 || error > 0.002f) return ESP_FAIL;
    digit_gate_reset(&gate);
    ESP_LOGI("TOUR", "WAIT_START: press F to play welcome, then scan digit");
    return ESP_OK;
}

bool tour_guide_waiting(void) { return atomic_load(&route) == 0 && !atomic_load(&stopped); }
int tour_guide_route(void) { return atomic_load(&stopped) ? -1 : atomic_load(&route); }
bool tour_guide_departure_ready(int64_t now_us)
{
    return tour_guide_route() > 0 && now_us - selected_us >= 2000000;
}
void tour_guide_stop(void) { atomic_store(&stopped, true); }

bool tour_guide_scanning(void) { return atomic_load(&scan_phase)==2 && tour_guide_waiting(); }
void tour_guide_enable_scan(int64_t now_us) {
    if (atomic_load(&scan_phase)!=1 || !tour_guide_waiting()) return;
    scan_after_us=now_us;
    atomic_store(&scan_phase,2);
}
void tour_guide_process_digit(const uint8_t *frame, int64_t captured_us)
{
    if (!tour_guide_waiting()) return;
    if (!tour_guide_scanning() || captured_us < scan_after_us) {
        camera_display_show_rotated_rgb565(frame,160,120);
        return;
    }
    uint8_t input[784] = {0};
    digit_region_t region = digit_vision_prepare(vision, frame, 1, false, input);
    float logits[10], probabilities[10], score = 0, margin = 0;
    int best = -1;
    if (region.valid) {
        digit_model_predict(model, input, logits, probabilities);
        best = 0;
        for (int i = 1; i < 10; ++i) if (probabilities[i] > probabilities[best]) best = i;
        float second = 0;
        for (int i = 0; i < 10; ++i) if (i != best && probabilities[i] > second) second = probabilities[i];
        score = probabilities[best];
        margin = score - second;
    }
    last_digit = best;
    last_score = (int)(score * 100 + 0.5f);
    int digit = digit_gate_update(&gate, best, region.valid, score, margin, captured_us / 1000);
    camera_display_show_digit(frame, input, &region,
        region.valid && score >= DIGIT_MIN_SCORE ? best : -1,
        score, region.valid && score >= DIGIT_MIN_SCORE && gate.streak >= DIGIT_CONFIRM_FRAMES);
    if (digit == 1 || digit == 2) {
        selected_us = captured_us;
        atomic_store(&route, digit);
        ESP_LOGI("TOUR", "ROUTE=%d %s: remove card; departure after 2s and stable track", digit, digit == 1 ? "LEFT" : "RIGHT");
    } else if (digit >= 0) digit_gate_reset(&gate);
}
