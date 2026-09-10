#include "xiaozhi_wake.h"
#include "xiaozhi_client.h"
#include <stdlib.h>
#include <string.h>
#include <stdatomic.h>
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_mn_iface.h"
#include "esp_mn_models.h"
#include "esp_mn_speech_commands.h"
#include "model_path.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/idf_additions.h"

static const char *TAG = "LEDi_WAKE";
typedef struct { int16_t pcm[XZ_PCM_SAMPLES]; } wake_frame_t;
static QueueHandle_t s_queue;
static StaticQueue_t s_queue_state;
static atomic_bool s_ready, s_gap;
static atomic_uint s_fed, s_dropped;

bool xiaozhi_wake_ready(void) { return atomic_load(&s_ready); }

void xiaozhi_wake_feed(const int16_t *pcm)
{
    if (xiaozhi_wake_ready() && xiaozhi_local_listening()) {
        if (xQueueSend(s_queue, pcm, 0) != pdTRUE) {
            atomic_store(&s_gap, true); atomic_fetch_add(&s_dropped, 1);
        } else atomic_fetch_add(&s_fed, 1);
    }
}

static void wake_task(void *arg)
{
    srmodel_list_t *models = arg;
    char *name = models ? esp_srmodel_filter(models, ESP_MN_PREFIX, "cn") : NULL;
    esp_mn_iface_t *mn = name ? esp_mn_handle_from_name(name) : NULL;
    model_iface_data_t *data = mn ? mn->create(name, 3000) : NULL;
    int chunk = data ? mn->get_samp_chunksize(data) : 0;
    int16_t *samples = chunk > 0 && chunk <= 4096 ?
        heap_caps_malloc(chunk * sizeof(int16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) : NULL;
    wake_frame_t *frame = heap_caps_malloc(sizeof(*frame), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!data || !samples || !frame || mn->get_samp_rate(data) != 16000) goto failed;
    if (esp_mn_commands_alloc(mn, data) != ESP_OK ||
        esp_mn_commands_add(1, "ni hao le di") != ESP_OK || esp_mn_commands_update()) goto failed;
    mn->set_det_threshold(data, 0.35f);
    atomic_store(&s_ready, true);
    ESP_LOGI(TAG, "WAKE_READY phrase=你好乐迪 pinyin=ni hao le di model=%s; LOCAL ONLY", name);
    size_t used = 0;
    unsigned chunks = 0, resets = 0, hits = 0;
    int64_t report_at = esp_timer_get_time();
    int64_t max_detect_us = 0;
    for (;;) {
        if (esp_timer_get_time() - report_at >= 2000000) {
            ESP_LOGI(TAG, "WAKE_MONITOR local=%d fed=%u chunks=%u dropped=%u resets=%u hits=%u queue=%u max_detect_ms=%lld chunk_samples=%d",
                     xiaozhi_local_listening(), atomic_load(&s_fed), chunks,
                     atomic_load(&s_dropped), resets, hits, (unsigned)uxQueueMessagesWaiting(s_queue),
                     (long long)(max_detect_us / 1000), chunk);
            report_at = esp_timer_get_time(); max_detect_us = 0;
        }
        if (xQueueReceive(s_queue, frame, pdMS_TO_TICKS(100)) != pdTRUE ||
            !xiaozhi_local_listening() || atomic_exchange(&s_gap, false)) {
            ++resets; used = 0; mn->clean(data); xQueueReset(s_queue); continue;
        }
        for (size_t i = 0; i < XZ_PCM_SAMPLES; ++i) {
            samples[used++] = frame->pcm[i];
            if (used != (size_t)chunk) continue;
            used = 0;
            int64_t start = esp_timer_get_time();
            esp_mn_state_t state = mn->detect(data, samples);
            int64_t elapsed = esp_timer_get_time() - start;
            if (elapsed > max_detect_us) max_detect_us = elapsed;
            ++chunks;
            if (state == ESP_MN_STATE_DETECTED) {
                esp_mn_results_t *result = mn->get_results(data);
                if (result && result->num > 0)
                    ESP_LOGI(TAG, "WAKE_CANDIDATE command=%d probability=%.3f threshold=0.350", result->command_id[0], result->prob[0]);
                if (result && result->num > 0 && result->command_id[0] == 1 && result->prob[0] >= 0.35f) {
                    ESP_LOGI(TAG, "WAKE_DETECTED 你好乐迪 probability=%.2f", result->prob[0]);
                    xiaozhi_request_wake();
                    ++hits;
                    mn->clean(data); xQueueReset(s_queue); break;
                }
                mn->clean(data);
            } else if (state == ESP_MN_STATE_TIMEOUT) mn->clean(data);
        }
    }
failed:
    ESP_LOGE(TAG, "WAKE_DISABLED: model/init failure; manual xz commands still available");
    free(samples); free(frame);
    if (data) { esp_mn_commands_free(); mn->destroy(data); }
    /* Mapping remains for the lifetime of the firmware; unmapping requires
     * an internal-RAM task stack, unlike this inference task's PSRAM stack. */
    vTaskDeleteWithCaps(NULL);
}

esp_err_t xiaozhi_wake_init(void)
{
    /* Flash mmap briefly disables cache: perform it on the caller's internal
     * stack, before starting inference on a PSRAM-backed stack. */
    srmodel_list_t *models = esp_srmodel_init("model");
    if (!models) return ESP_FAIL;
    uint8_t *storage = heap_caps_malloc(8 * sizeof(wake_frame_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!storage) { esp_srmodel_deinit(models); return ESP_ERR_NO_MEM; }
    s_queue = xQueueCreateStatic(8, sizeof(wake_frame_t), storage, &s_queue_state);
    /* Audio inference must run before the core-1 camera decoder (priority 4).
     * The blocking input queue yields the CPU between audio chunks. */
    if (xTaskCreatePinnedToCoreWithCaps(wake_task, "ledi_wake", 32768, models, 5, NULL,
                                      1, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) != pdPASS) {
        vQueueDelete(s_queue); s_queue = NULL; free(storage); esp_srmodel_deinit(models); return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}
