#include "usb_audio.h"
#include "xiaozhi_client.h"
#include "xiaozhi_wake.h"
#include "wifi_debug.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "usb/uac_host.h"

static const char *TAG = "CAR_AUDIO";
typedef struct { uint8_t address, interface; bool input; } connection_t;
typedef struct {
    uac_host_device_handle_t handle;
    bool streaming;
    bool disconnected;
    uint32_t errors;
} audio_slot_t;
static audio_slot_t s_mic, s_speaker;
static QueueHandle_t s_connections;
static portMUX_TYPE s_event_lock = portMUX_INITIALIZER_UNLOCKED;
static uint32_t s_samples, s_peak;
static uint64_t s_energy;
static int64_t s_report_us;
static bool s_cloud_mic;
static bool s_local_mic;
static int16_t s_cloud_pcm[XZ_PCM_SAMPLES];
static size_t s_cloud_used;
static int64_t s_last_playback;

static void device_event(uac_host_device_handle_t handle, uac_host_device_event_t event, void *arg)
{
    (void)handle;
    audio_slot_t *slot = arg;
    portENTER_CRITICAL(&s_event_lock);
    if (event == UAC_HOST_DRIVER_EVENT_DISCONNECTED) slot->disconnected = true;
    if (event == UAC_HOST_DEVICE_EVENT_TRANSFER_ERROR) ++slot->errors;
    portEXIT_CRITICAL(&s_event_lock);
}

static void driver_event(uint8_t address, uint8_t interface, uac_host_driver_event_t event, void *arg)
{
    (void)arg;
    connection_t connection = {address, interface, event == UAC_HOST_DRIVER_EVENT_RX_CONNECTED};
    if (xQueueSend(s_connections, &connection, 0) != pdTRUE)
        ESP_LOGW(TAG, "Connection queue full; reconnect the USB module");
}

static void open_audio(const connection_t *c)
{
    audio_slot_t *slot = c->input ? &s_mic : &s_speaker;
    if (slot->handle) return;
    portENTER_CRITICAL(&s_event_lock);
    slot->disconnected = false;
    slot->errors = 0;
    portEXIT_CRITICAL(&s_event_lock);
    const uac_host_device_config_t config = {
        .addr = c->address, .iface_num = c->interface,
        .buffer_size = 8192, .buffer_threshold = 640,
        .callback = device_event, .callback_arg = slot,
    };
    esp_err_t error = uac_host_device_open(&config, &slot->handle);
    if (error != ESP_OK) {
        slot->handle = NULL;
        ESP_LOGE(TAG, "%s open failed: %s", c->input ? "MIC" : "SPEAKER", esp_err_to_name(error));
        return;
    }
    const uac_host_stream_config_t stream = {
        .channels = 1, .bit_resolution = 16, .sample_freq = 16000,
        .flags = FLAG_STREAM_SUSPEND_AFTER_START,
    };
    error = uac_host_device_start(slot->handle, &stream);
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "%s PCM prepare failed: %s", c->input ? "MIC" : "SPEAKER", esp_err_to_name(error));
        uac_host_device_close(slot->handle);
        slot->handle = NULL;
        return;
    }
    /* Only unmute a stream when the user starts its diagnostic command. */
    ESP_LOGI(TAG, "%s_READY interface=%u PCM=16000Hz/mono/16bit (suspended)",
             c->input ? "MIC" : "SPEAKER", c->interface);
}

static void handle_disconnect(audio_slot_t *slot, const char *name)
{
    portENTER_CRITICAL(&s_event_lock);
    bool lost = slot->disconnected;
    portEXIT_CRITICAL(&s_event_lock);
    if (lost && slot->handle) {
        esp_err_t error = uac_host_device_close(slot->handle);
        ESP_LOGW(TAG, "%s disconnected, close=%s", name, esp_err_to_name(error));
        if (error == ESP_OK) {
            slot->handle = NULL;
            slot->streaming = false;
        }
    }
}

static void read_microphone(void)
{
    if (!s_mic.handle || !s_mic.streaming) return;
    uint8_t pcm[1280];
    for (unsigned n = 0; n < 8; ++n) {
        uint32_t received = 0;
        esp_err_t error = uac_host_device_read(s_mic.handle, pcm, sizeof(pcm), &received, 0);
        if (error != ESP_OK || received == 0) break;
        for (unsigned i = 0; i + 1 < received; i += 2) {
            int value = (int16_t)(pcm[i] | ((unsigned)pcm[i + 1] << 8));
            if ((s_cloud_mic && xiaozhi_wants_microphone()) ||
                (s_local_mic && xiaozhi_local_listening())) {
                s_cloud_pcm[s_cloud_used++] = value;
                if (s_cloud_used == XZ_PCM_SAMPLES) {
                    if (s_cloud_mic) xiaozhi_push_pcm(s_cloud_pcm);
                    else xiaozhi_wake_feed(s_cloud_pcm);
                    s_cloud_used = 0;
                }
            }
            unsigned magnitude = value < 0 ? -value : value;
            if (magnitude > s_peak) s_peak = magnitude;
            s_energy += (int64_t)value * value;
            ++s_samples;
        }
    }
    int64_t now = esp_timer_get_time();
    if (now - s_report_us >= 1000000) {
        ESP_LOGI(TAG, "MIC_METER samples=%lu peak=%lu rms=%u window_ms=%lld cloud_upload=%d",
                 (unsigned long)s_samples, (unsigned long)s_peak,
                 s_samples ? (unsigned)sqrt((double)s_energy / s_samples) : 0,
                 (long long)((now - s_report_us) / 1000), s_cloud_mic);
        s_samples = 0; s_peak = 0; s_energy = 0; s_report_us = now;
    }
}

static void tone(void)
{
    if (!s_speaker.handle) { ESP_LOGW(TAG, "Speaker is not ready"); return; }
    esp_err_t error = uac_host_device_set_mute(s_speaker.handle, false);
    if (error != ESP_OK && error != ESP_ERR_NOT_SUPPORTED) {
        ESP_LOGW(TAG, "Speaker unmute: %s", esp_err_to_name(error));
        return;
    }
    /* Digital amplitude capped at 1000/32768 (~-30 dBFS), 200 ms, soft edges. */
    error = uac_host_device_resume(s_speaker.handle);
    if (error != ESP_OK) { ESP_LOGW(TAG, "Speaker resume: %s", esp_err_to_name(error)); return; }
    s_speaker.streaming = true;
    int16_t pcm[320];
    for (unsigned chunk = 0; chunk < 10 && error == ESP_OK; ++chunk) {
        for (unsigned i = 0; i < 320; ++i) {
            unsigned sample = chunk * 320 + i;
            float fade = sample < 160 ? sample / 160.0f : sample >= 3040 ? (3199 - sample) / 160.0f : 1.0f;
            pcm[i] = (int16_t)(1000.0f * fade * sinf(2.0f * 3.14159265f * 600.0f * sample / 16000.0f));
        }
        error = uac_host_device_write(s_speaker.handle, (uint8_t *)pcm, sizeof(pcm), pdMS_TO_TICKS(50));
        read_microphone();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    vTaskDelay(pdMS_TO_TICKS(80));
    esp_err_t stopped = uac_host_device_suspend(s_speaker.handle);
    s_speaker.streaming = stopped != ESP_OK;
    ESP_LOGI(TAG, "TONE_QUEUED result=%s suspend=%s (confirm sound by ear)",
             esp_err_to_name(error), esp_err_to_name(stopped));
}

static void command(const char *line)
{
    if (wifi_debug_uart_command(line)) return;
    if (xiaozhi_command(line)) return;
    if ((s_cloud_mic || xiaozhi_allows_playback()) && strcmp(line, "audio status") != 0) {
        ESP_LOGW(TAG, "Cloud microphone active; use xz send or xz stop first");
        return;
    }
    if (strcmp(line, "audio mic") == 0) {
        if (!s_mic.handle) { ESP_LOGW(TAG, "Microphone is not ready"); return; }
        if (s_mic.streaming) return;
        esp_err_t error = uac_host_device_set_mute(s_mic.handle, false);
        if (error == ESP_OK || error == ESP_ERR_NOT_SUPPORTED)
            error = uac_host_device_resume(s_mic.handle);
        s_mic.streaming = error == ESP_OK;
        s_samples = 0; s_peak = 0; s_energy = 0; s_report_us = esp_timer_get_time();
        ESP_LOGI(TAG, "MIC_START=%s; PCM is measured then discarded, no recording/upload", esp_err_to_name(error));
    } else if (strcmp(line, "audio stop") == 0) {
        if (s_mic.handle && s_mic.streaming) {
            esp_err_t error = uac_host_device_suspend(s_mic.handle);
            if (error == ESP_OK) s_mic.streaming = false;
            ESP_LOGI(TAG, "MIC_STOP=%s", esp_err_to_name(error));
        }
    } else if (strcmp(line, "audio tone") == 0) {
        tone();
    } else if (strcmp(line, "audio status") == 0) {
        portENTER_CRITICAL(&s_event_lock);
        uint32_t mic_errors = s_mic.errors, speaker_errors = s_speaker.errors;
        portEXIT_CRITICAL(&s_event_lock);
        ESP_LOGI(TAG, "STATUS mic_ready=%d mic_active=%d speaker_ready=%d mic_errors=%lu speaker_errors=%lu cloud=see_xz_status motors=disabled",
                 s_mic.handle != NULL, s_mic.streaming, s_speaker.handle != NULL,
                 (unsigned long)mic_errors, (unsigned long)speaker_errors);
    } else if (line[0]) {
        ESP_LOGI(TAG, "Commands: audio status | audio mic | audio stop | audio tone");
    }
}

/* All UAC operations remain owned by this task, never by network callbacks. */
static void service_cloud_audio(void)
{
    bool wanted = xiaozhi_wants_microphone();
    bool local = !wanted && xiaozhi_wake_ready() && xiaozhi_local_listening();
    if (local != s_local_mic || (local && !s_mic.streaming)) {
        s_cloud_used = 0;
        if (local && s_mic.handle && !s_mic.streaming) {
            esp_err_t error = uac_host_device_set_mute(s_mic.handle, false);
            if (error == ESP_OK || error == ESP_ERR_NOT_SUPPORTED)
                error = uac_host_device_resume(s_mic.handle);
            s_mic.streaming = error == ESP_OK;
            s_samples=0; s_energy=0; s_peak=0; s_report_us=esp_timer_get_time();
        } else if (!local && s_local_mic && s_mic.handle && s_mic.streaming) {
            if (uac_host_device_suspend(s_mic.handle) == ESP_OK) s_mic.streaming=false;
        }
        s_local_mic = local && s_mic.handle && s_mic.streaming;
    }
    if (wanted && !s_cloud_mic) {
        if (!s_mic.handle || !s_speaker.handle) { xiaozhi_audio_fault(); return; }
        if (s_mic.streaming) uac_host_device_suspend(s_mic.handle);
        esp_err_t error = uac_host_device_set_mute(s_mic.handle, false);
        if (error == ESP_OK || error == ESP_ERR_NOT_SUPPORTED) error = uac_host_device_resume(s_mic.handle);
        s_mic.streaming = error == ESP_OK;
        s_cloud_mic = s_mic.streaming; s_cloud_used = 0;
        s_samples=0; s_energy=0; s_peak=0; s_report_us=esp_timer_get_time();
        if (!s_cloud_mic) xiaozhi_audio_fault();
    } else if (!wanted && s_cloud_mic) {
        if (s_mic.handle && s_mic.streaming) {
            esp_err_t error = uac_host_device_suspend(s_mic.handle);
            if (error != ESP_OK) xiaozhi_audio_fault();
        }
        s_cloud_mic=false; s_mic.streaming=false; s_cloud_used=0;
    }
    if (s_cloud_mic && !s_mic.handle) { s_cloud_mic=false; xiaozhi_audio_fault(); }
    static xz_playback_t playback;
    bool may_play=xiaozhi_allows_playback();
    if (s_speaker.handle && s_speaker.streaming &&
        (!may_play || esp_timer_get_time()-s_last_playback>500000)) {
        esp_err_t error=uac_host_device_suspend(s_speaker.handle);
        if (error == ESP_OK) s_speaker.streaming=false;
        else xiaozhi_audio_fault();
    }
    if (xiaozhi_take_playback(&playback)) {
        if (!may_play) return;
        if (!s_speaker.handle) { xiaozhi_audio_fault(); return; }
        esp_err_t error = ESP_OK;
        if (!s_speaker.streaming) {
            error = uac_host_device_set_mute(s_speaker.handle, false);
            if (error == ESP_OK || error == ESP_ERR_NOT_SUPPORTED) error = uac_host_device_resume(s_speaker.handle);
            s_speaker.streaming = error == ESP_OK;
        }
          /* Gain 3/8: +3.5 dB over the original 1/4, with digital headroom.
           * Use a widened intermediate; no cloud command can increase it. */
          for (unsigned i=0; i<playback.samples; ++i)
              playback.pcm[i] = (int16_t)(((int32_t)playback.pcm[i] * 3) / 8);
        if (error == ESP_OK) error=uac_host_device_write(s_speaker.handle, (uint8_t *)playback.pcm,
                                                      playback.samples*2, pdMS_TO_TICKS(80));
        if (error != ESP_OK) xiaozhi_audio_fault();
        else s_last_playback=esp_timer_get_time();
    }
}

static void audio_task(void *arg)
{
    (void)arg;
    char line[512];
    size_t used = 0;
    bool overflow = false;
    for (;;) {
        handle_disconnect(&s_mic, "MIC");
        handle_disconnect(&s_speaker, "SPEAKER");
        connection_t connection;
        while (xQueueReceive(s_connections, &connection, 0) == pdTRUE) open_audio(&connection);
        uint8_t input[64];
        int n = uart_read_bytes(UART_NUM_0, input, sizeof(input), 0);
        for (int i = 0; i < n; ++i) {
            if (input[i] == '\n' || input[i] == '\r') {
                line[used] = 0;
                if (!overflow) command(line);
                memset(line, 0, sizeof(line));
                used = 0; overflow = false;
            } else if (input[i] >= 32 && input[i] != 127) {
                if (used + 1 < sizeof(line)) line[used++] = input[i];
                else overflow = true;
            }
        }
        service_cloud_audio();
        read_microphone();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

esp_err_t usb_audio_init(void)
{
    esp_err_t error = uart_is_driver_installed(UART_NUM_0) ? ESP_OK :
        uart_driver_install(UART_NUM_0, 1024, 0, 0, NULL, 0);
    if (error != ESP_OK) return error;
    s_connections = xQueueCreate(8, sizeof(connection_t));
    if (!s_connections) return ESP_ERR_NO_MEM;
    const uac_host_driver_config_t config = {
        .create_background_task = true, .task_priority = 5, .stack_size = 4096,
        .core_id = 0, .callback = driver_event, .callback_arg = NULL,
    };
    error = uac_host_install(&config);
    if (error != ESP_OK) { vQueueDelete(s_connections); s_connections = NULL; return error; }
    if (xTaskCreate(audio_task, "car_audio", 6144, NULL, 4, NULL) != pdPASS) {
        uac_host_uninstall(); vQueueDelete(s_connections); s_connections = NULL;
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "UART0 115200: audio status | audio mic | audio stop | audio tone");
    return ESP_OK;
}
