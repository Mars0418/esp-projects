#include "xiaozhi_client.h"
#include "tour_speech.h"
#include <stdatomic.h>
#include "freertos/idf_additions.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "cJSON.h"
#include "esp_crt_bundle.h"
#include "esp_heap_caps.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_random.h"
#include "esp_sntp.h"
#include "esp_timer.h"
#include "esp_websocket_client.h"
#include "esp_opus_enc.h"
#include "esp_opus_dec.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "nvs.h"

#define DISCOVERY_URL "https://api.tenclass.net/xiaozhi/ota/"
#define RX_MAX 8192
#define WS_UP BIT0
#define WS_DOWN BIT1
#define AUDIO_FAULT BIT2
static const char *TAG = "XIAOZHI";
typedef enum { CMD_BIND=1, CMD_CONNECT, CMD_LISTEN, CMD_SEND, CMD_STOP, CMD_STATUS } command_t;
typedef struct { uint16_t len; uint8_t opcode; uint8_t bytes[RX_MAX]; } message_t;
typedef struct { int16_t pcm[XZ_PCM_SAMPLES]; } capture_t;
static QueueHandle_t s_commands, s_rx, s_tx, s_play;
static StaticQueue_t s_rx_state, s_tx_state, s_play_state;
static EventGroupHandle_t s_events;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static bool s_capture;
static bool s_playback_allowed;
static char s_mac[18], s_uuid[37], s_url[512], s_token[1024], s_session[128];
static int s_version = 1;
static esp_websocket_client_handle_t s_ws;
static void *s_encoder, *s_decoder;
static uint8_t *s_encoded;
static int s_encoded_size;
static bool s_hello, s_speaking, s_turn, s_end_listen;
static int64_t s_deadline, s_hello_deadline;
static uint32_t s_sent, s_received;
static message_t *s_assembly;
static size_t s_assembled, s_frame_base;
static bool s_fragment;
static atomic_bool s_mic_ready, s_output_active;
static atomic_int s_status;
static atomic_int s_bind_code = -1;
int xiaozhi_binding_code(void) { return atomic_load(&s_bind_code); }
static bool s_auto=true;
static int64_t s_retry_after, s_listen_after;
void xiaozhi_set_microphone_ready(bool ready) { atomic_store(&s_mic_ready,ready); }
void xiaozhi_set_output_active(bool active) { atomic_store(&s_output_active,active); }
int xiaozhi_status(void) { return atomic_load(&s_status); }

static void capture_enable(bool enabled)
{
    portENTER_CRITICAL(&s_lock); s_capture = enabled; portEXIT_CRITICAL(&s_lock);
}
bool xiaozhi_wants_microphone(void)
{
    portENTER_CRITICAL(&s_lock); bool value = s_capture; portEXIT_CRITICAL(&s_lock);
    return value && !tour_speech_busy() && !atomic_load(&s_output_active);
}
bool xiaozhi_allows_playback(void)
{
    portENTER_CRITICAL(&s_lock); bool value=s_playback_allowed; portEXIT_CRITICAL(&s_lock);
    return value && !tour_speech_busy();
}
bool xiaozhi_push_pcm(const int16_t *pcm)
{
    if (!s_tx || !xiaozhi_wants_microphone()) return false;
    if (xQueueSend(s_tx, pcm, 0) == pdTRUE) return true;
    xiaozhi_audio_fault();
    return false;
}
bool xiaozhi_take_playback(xz_playback_t *frame)
{
    bool have=s_play && xQueueReceive(s_play, frame, 0)==pdTRUE;
    if(have) atomic_store(&s_output_active,true);
    return have;
}
void xiaozhi_audio_fault(void)
{
    capture_enable(false);
    portENTER_CRITICAL(&s_lock); s_playback_allowed=false; portEXIT_CRITICAL(&s_lock);
    if (s_events) xEventGroupSetBits(s_events, AUDIO_FAULT);
}
bool xiaozhi_command(const char *text)
{
    const char *names[] = {"xz bind", "xz connect", "xz listen", "xz send", "xz stop", "xz status"};
    for (int i = 0; i < 6; ++i) {
        if (strcmp(text, names[i]) != 0) continue;
        command_t command = i + 1;
        if (command == CMD_STOP || command == CMD_SEND) capture_enable(false);
        if (command == CMD_STOP) {
            portENTER_CRITICAL(&s_lock); s_playback_allowed=false; portEXIT_CRITICAL(&s_lock);
        }
        if (!s_commands || xQueueSend(s_commands, &command, 0) != pdTRUE)
            ESP_LOGW(TAG, "Busy; retry command");
        return true;
    }
    return false;
}

static bool copy_string(const cJSON *obj, const char *key, char *out, size_t size)
{
    const cJSON *value = cJSON_GetObjectItemCaseSensitive(obj, key);
    if (!cJSON_IsString(value) || strlen(value->valuestring) >= size) return false;
    strcpy(out, value->valuestring);
    return true;
}

static bool connected_ip(void)
{
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    esp_netif_ip_info_t info;
    return netif && esp_netif_get_ip_info(netif, &info) == ESP_OK && info.ip.addr;
}

static cJSON *post_discovery(void)
{
    if (!connected_ip()) { ESP_LOGW(TAG, "No station IP; reconnect hotspot then type xz bind"); return NULL; }
    /* TLS always validates the CA chain and hostname. Never fall back to HTTP. */
    if (!esp_sntp_enabled()) {
        esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
        esp_sntp_setservername(0, "ntp.aliyun.com");
        esp_sntp_setservername(1, "pool.ntp.org");
        esp_sntp_init();
    }
    for (unsigned i = 0; time(NULL) < 1700000000 && i < 40; ++i) vTaskDelay(pdMS_TO_TICKS(500));
    if (time(NULL) < 1700000000) { ESP_LOGW(TAG, "Clock not synchronized; retry xz bind"); return NULL; }
    char body[1024];
    snprintf(body, sizeof(body),
        "{\"version\":2,\"language\":\"zh-CN\",\"chip_model_name\":\"esp32s3\","
        "\"mac_address\":\"%s\",\"uuid\":\"%s\",\"flash_size\":33554432,"
        "\"application\":{\"name\":\"ballcar-xiaozhi\",\"version\":\"0.1.0\",\"idf_version\":\"5.4.4\"},"
        "\"board\":{\"type\":\"ballcar-usb-audio\",\"name\":\"ballcar-usb-audio\",\"mac\":\"%s\"}}",
        s_mac, s_uuid, s_mac);
    const esp_http_client_config_t config = {
        .url = DISCOVERY_URL, .timeout_ms = 15000,
        .crt_bundle_attach = esp_crt_bundle_attach, .disable_auto_redirect = true,
        .buffer_size = 1024, .buffer_size_tx = 1536,
    };
    esp_http_client_handle_t http = esp_http_client_init(&config);
    if (!http) return NULL;
    esp_http_client_set_method(http, HTTP_METHOD_POST);
    esp_http_client_set_header(http, "Activation-Version", "1");
    esp_http_client_set_header(http, "Device-Id", s_mac);
    esp_http_client_set_header(http, "Client-Id", s_uuid);
    esp_http_client_set_header(http, "Content-Type", "application/json");
    esp_http_client_set_header(http, "Accept-Language", "zh-CN");
    esp_http_client_set_header(http, "User-Agent", "ballcar-usb-audio/0.1.0");
    cJSON *result = NULL;
    char *response = heap_caps_malloc(RX_MAX + 1, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!response) goto done;
    esp_err_t error = esp_http_client_open(http, strlen(body));
    if (error != ESP_OK) { ESP_LOGW(TAG, "Discovery connect=%s", esp_err_to_name(error)); goto done; }
    size_t written = 0;
    while (written < strlen(body)) {
        int n = esp_http_client_write(http, body + written, strlen(body) - written);
        if (n <= 0) goto done;
        written += n;
    }
    if (esp_http_client_fetch_headers(http) < 0) goto done;
    int status = esp_http_client_get_status_code(http);
    ESP_LOGI(TAG, "DISCOVERY_HTTP=%d device=%s", status, s_mac);
    if (status != 200) goto done;
    size_t used = 0;
    while (used < RX_MAX) {
        int n = esp_http_client_read(http, response + used, RX_MAX - used);
        if (n < 0) goto done;
        if (n == 0) break;
        used += n;
    }
    if (!esp_http_client_is_complete_data_received(http) || used == RX_MAX) goto done;
    response[used] = 0;
    result = cJSON_ParseWithLength(response, used);
done:
    if (response) { memset(response, 0, RX_MAX + 1); free(response); }
    esp_http_client_close(http);
    esp_http_client_cleanup(http);
    return result;
}

static void bind_device(void)
{
    cJSON *root = post_discovery();
    if (!root) { ESP_LOGW(TAG, "Discovery unavailable; type xz bind to retry"); return; }
    char code[32] = {0};
    const cJSON *activation = cJSON_GetObjectItemCaseSensitive(root, "activation");
    if (copy_string(activation, "code", code, sizeof(code)) && code[0])
        ESP_LOGW(TAG, "BIND_CODE=%s Open https://xiaozhi.me/ and add this device; then type xz bind", code);
    atomic_store(&s_bind_code,code[0]?atoi(code):-1);
    const cJSON *ws = cJSON_GetObjectItemCaseSensitive(root, "websocket");
    char url[512] = {0}, token[1024] = {0};
    if (copy_string(ws, "url", url, sizeof(url)) && strncmp(url, "wss://", 6) == 0 &&
        !strpbrk(url, "\r\n") && (!cJSON_HasObjectItem(ws, "token") || copy_string(ws, "token", token, sizeof(token))) &&
        !strpbrk(token, "\r\n")) {
        const cJSON *v = cJSON_GetObjectItemCaseSensitive(ws, "version");
        int version = cJSON_IsNumber(v) ? v->valueint : 1;
        if (version >= 1 && version <= 3) {
            strcpy(s_url, url); strcpy(s_token, token); s_version = version;
            ESP_LOGI(TAG, "Secure WebSocket configuration received, protocol=%d (credentials not printed)", version);
        } else ESP_LOGW(TAG, "Unsupported WebSocket protocol version");
    } else ESP_LOGW(TAG, "No supported secure WebSocket config; binding or server transport selection may be needed");
    if (!code[0]) ESP_LOGI(TAG, "No binding code in response; verify by xz connect");
    ESP_LOGI(TAG, "Firmware metadata ignored: no OTA/download/write enabled");
    memset(token, 0, sizeof(token));
    cJSON_Delete(root);
}

static bool send_text(const char *text)
{
    return s_ws && esp_websocket_client_send_text(s_ws, text, strlen(text), pdMS_TO_TICKS(1000)) == (int)strlen(text);
}

static void ws_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg; (void)base;
    esp_websocket_event_data_t *event = data;
    if (id == WEBSOCKET_EVENT_CONNECTED) xEventGroupSetBits(s_events, WS_UP);
    else if (id == WEBSOCKET_EVENT_DISCONNECTED || id == WEBSOCKET_EVENT_CLOSED || id == WEBSOCKET_EVENT_ERROR) {
        capture_enable(false);
        portENTER_CRITICAL(&s_lock); s_playback_allowed=false; portEXIT_CRITICAL(&s_lock);
        xEventGroupSetBits(s_events, WS_DOWN);
    } else if (id == WEBSOCKET_EVENT_DATA && event->op_code <= 2) {
        /* Reassemble both receive-buffer chunks and RFC6455 continuation frames. */
        if (event->payload_offset == 0) {
            if (event->op_code == 1 || event->op_code == 2) {
                if (s_fragment) { xiaozhi_audio_fault(); return; }
                s_assembled = 0; s_assembly->opcode = event->op_code;
            } else if (!s_fragment) { xiaozhi_audio_fault(); return; }
            s_frame_base = s_assembled;
        }
        if (event->payload_len < 0 || event->payload_offset < 0 || event->data_len < 0 ||
            (size_t)event->payload_len > RX_MAX - s_frame_base ||
            (size_t)event->payload_offset > (size_t)event->payload_len ||
            (size_t)event->data_len > (size_t)event->payload_len - event->payload_offset ||
            s_frame_base + event->payload_offset != s_assembled) { xiaozhi_audio_fault(); return; }
        memcpy(s_assembly->bytes + s_assembled, event->data_ptr, event->data_len);
        s_assembled += event->data_len;
        if (event->payload_offset + event->data_len == event->payload_len) {
            s_fragment = !event->fin;
            if (event->fin) {
                s_assembly->len = s_assembled;
                if (xQueueSend(s_rx, s_assembly, pdMS_TO_TICKS(250)) != pdTRUE) xiaozhi_audio_fault();
                s_assembled = 0;
            }
        }
    }
}

static void close_session(void)
{
    atomic_store(&s_status,s_auto?0:4);
    s_retry_after=esp_timer_get_time()+10000000;
    capture_enable(false); s_turn = false; s_speaking = false; s_hello = false; s_end_listen = false;
    portENTER_CRITICAL(&s_lock); s_playback_allowed=false; portEXIT_CRITICAL(&s_lock);
    if (s_ws) {
        esp_websocket_client_stop(s_ws);
        esp_websocket_client_destroy(s_ws); s_ws = NULL;
    }
    xQueueReset(s_rx); xQueueReset(s_tx); xQueueReset(s_play);
    xEventGroupClearBits(s_events, WS_UP | WS_DOWN | AUDIO_FAULT);
    s_session[0] = 0; s_fragment = false; s_assembled = 0;
    s_hello_deadline = 0;
}

static void connect_session(void)
{
    close_session();
    if (!s_url[0]) bind_device();
    if (!s_url[0] || !connected_ip()) return;
    char headers[1400];
    int n = snprintf(headers, sizeof(headers), "Device-Id: %s\r\nClient-Id: %s\r\nProtocol-Version: %d\r\n",
                     s_mac, s_uuid, s_version);
    if (s_token[0]) snprintf(headers + n, sizeof(headers) - n, "Authorization: %s%s\r\n",
                            strchr(s_token, ' ') ? "" : "Bearer ", s_token);
    const esp_websocket_client_config_t config = {
        .uri = s_url, .headers = headers, .crt_bundle_attach = esp_crt_bundle_attach,
        .disable_auto_reconnect = true, .task_stack = 6144, .task_prio = 2,
        .buffer_size = 2048, .network_timeout_ms = 5000,
    };
    s_ws = esp_websocket_client_init(&config);
    memset(headers, 0, sizeof(headers));
    if (!s_ws) return;
    esp_websocket_register_events(s_ws, WEBSOCKET_EVENT_ANY, ws_event, NULL);
    if (esp_websocket_client_start(s_ws) != ESP_OK) { close_session(); return; }
    s_hello_deadline = esp_timer_get_time() + 15000000;
    atomic_store(&s_status,1);
    ESP_LOGI(TAG, "Connecting secure XiaoZhi session (microphone off)");
}

static bool codec_init(void)
{
    esp_opus_enc_config_t enc = {
        .sample_rate=16000, .channel=1, .bits_per_sample=16, .bitrate=24000,
        .frame_duration=ESP_OPUS_ENC_FRAME_DURATION_60_MS,
        .application_mode=ESP_OPUS_ENC_APPLICATION_VOIP, .complexity=0,
        .enable_fec=false, .enable_dtx=false, .enable_vbr=true,
    };
    esp_opus_dec_cfg_t dec = {.sample_rate=16000, .channel=1,
        .frame_duration=ESP_OPUS_DEC_FRAME_DURATION_120_MS, .self_delimited=false};
    if (esp_opus_enc_open(&enc, sizeof(enc), &s_encoder) != ESP_AUDIO_ERR_OK ||
        esp_opus_dec_open(&dec, sizeof(dec), &s_decoder) != ESP_AUDIO_ERR_OK) return false;
    int in_size;
    if (esp_opus_enc_get_frame_size(s_encoder, &in_size, &s_encoded_size) != ESP_AUDIO_ERR_OK ||
        in_size != XZ_PCM_SAMPLES * 2 || s_encoded_size <= 0 || s_encoded_size > 4096) return false;
    s_encoded = heap_caps_malloc(s_encoded_size + 16, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    return s_encoded != NULL;
}

static int encode(capture_t *input)
{
    esp_audio_enc_in_frame_t in = {.buffer=(uint8_t *)input->pcm, .len=sizeof(input->pcm)};
    esp_audio_enc_out_frame_t out = {.buffer=s_encoded+16, .len=s_encoded_size};
    if (esp_opus_enc_process(s_encoder, &in, &out) != ESP_AUDIO_ERR_OK) return -1;
    return out.encoded_bytes;
}

static bool decode(uint8_t *data, size_t len, xz_playback_t *output)
{
    esp_audio_dec_in_raw_t raw = {.buffer=data, .len=len, .frame_recover=ESP_AUDIO_DEC_RECOVERY_NONE};
    esp_audio_dec_out_frame_t out = {.buffer=(uint8_t *)output->pcm, .len=sizeof(output->pcm)};
    esp_audio_dec_info_t info = {0};
    if (esp_opus_dec_decode(s_decoder, &raw, &out, &info) != ESP_AUDIO_ERR_OK ||
        out.decoded_size > sizeof(output->pcm) || out.decoded_size % 2 || raw.consumed != len) return false;
    output->samples = out.decoded_size / 2;
    /* Decode Opus directly at 16 kHz, including 24 kHz server streams. Opus supports this natively. */
    return output->samples > 0;
}

static void listen_message(bool start)
{
    cJSON *obj = cJSON_CreateObject();
    cJSON_AddStringToObject(obj, "session_id", s_session);
    cJSON_AddStringToObject(obj, "type", "listen");
    cJSON_AddStringToObject(obj, "state", start ? "start" : "stop");
    if (start) cJSON_AddStringToObject(obj, "mode", s_auto ? "auto" : "manual");
    char *text = cJSON_PrintUnformatted(obj);
    bool ok = text && send_text(text);
    cJSON_free(text); cJSON_Delete(obj);
    if (!ok) xiaozhi_audio_fault();
    else if (start) {
        xQueueReset(s_tx); s_turn = true; s_speaking = false; s_end_listen = false;
        s_deadline = esp_timer_get_time() + 15000000;
        capture_enable(true);
        atomic_store(&s_status,2);
        ESP_LOGI(TAG, "LISTENING: automatic server VAD; microphone streams to XiaoZhi; xz stop disables");
    } else {
        s_end_listen = false;
        s_deadline = esp_timer_get_time() + 60000000;
        ESP_LOGI(TAG, "Utterance sent; waiting for reply (microphone off)");
    }
}

static void process_message(message_t *message, xz_playback_t *pcm)
{
    if (message->opcode == 2) {
        if (!s_turn || !s_speaking) return;
        size_t skip = s_version == 2 ? 16 : s_version == 3 ? 4 : 0;
        uint8_t *b = message->bytes;
        if (message->len <= skip) { xiaozhi_audio_fault(); return; }
        if (s_version == 2 && (b[0] != 0 || b[1] != 2 || b[2] || b[3] ||
            (((uint32_t)b[12]<<24)|((uint32_t)b[13]<<16)|((uint32_t)b[14]<<8)|b[15]) != message->len-skip)) {
            xiaozhi_audio_fault(); return;
        }
        if (s_version == 3 && (b[0] || (((unsigned)b[2]<<8)|b[3]) != message->len-skip)) {
            xiaozhi_audio_fault(); return;
        }
        if (!decode(b+skip, message->len-skip, pcm) || xQueueSend(s_play, pcm, 0) != pdTRUE)
            xiaozhi_audio_fault();
        else ++s_received;
        return;
    }
    cJSON *obj = cJSON_ParseWithLength((const char *)message->bytes, message->len);
    char type[32];
    if (!copy_string(obj, "type", type, sizeof(type))) { cJSON_Delete(obj); return; }
    if (strcmp(type, "hello") == 0) {
        char transport[24], format[16];
        cJSON *audio = cJSON_GetObjectItemCaseSensitive(obj, "audio_params");
        cJSON *channels = cJSON_GetObjectItemCaseSensitive(audio, "channels");
        if (copy_string(obj, "transport", transport, sizeof(transport)) && strcmp(transport, "websocket") == 0 &&
            copy_string(obj, "session_id", s_session, sizeof(s_session)) && s_session[0] &&
            (!cJSON_HasObjectItem(audio, "format") || (copy_string(audio, "format", format, sizeof(format)) && strcmp(format, "opus") == 0)) &&
            (!channels || (cJSON_IsNumber(channels) && channels->valueint == 1))) {
            s_hello = true; s_hello_deadline = 0;
            ESP_LOGI(TAG, "SESSION_READY: authenticated hello; type xz listen to speak, then xz send");
        } else xiaozhi_audio_fault();
    } else if (strcmp(type, "tts") == 0) {
        char state[32];
        if (copy_string(obj, "state", state, sizeof(state))) {
            if (strcmp(state, "start") == 0 && s_turn) {
                atomic_store(&s_status,3);
                capture_enable(false); s_speaking = true; s_end_listen = false;
                s_deadline=esp_timer_get_time()+60000000;
                portENTER_CRITICAL(&s_lock); s_playback_allowed=true; portEXIT_CRITICAL(&s_lock);
                xQueueReset(s_tx); esp_opus_dec_reset(s_decoder);
            } else if (strcmp(state, "stop") == 0) {
                s_speaking = false; s_turn = false;
                s_listen_after=esp_timer_get_time()+500000;
                ESP_LOGI(TAG, "REPLY_RECEIVED frames=%lu; queued playback draining, microphone remains off", (unsigned long)s_received);
            }
        }
        char text[1024];
        if (s_turn && copy_string(obj, "text", text, sizeof(text))) ESP_LOGI(TAG, "ASSISTANT: %s", text);
    } else if (strcmp(type, "stt") == 0) {
        char text[1024];
        if (copy_string(obj, "text", text, sizeof(text))) ESP_LOGI(TAG, "USER: %s", text);
    } else if (strcmp(type, "alert") == 0) {
        char text[512];
        if (copy_string(obj, "message", text, sizeof(text))) ESP_LOGW(TAG, "SERVER_ALERT: %s", text);
    }
    /* MCP, motion, remote reboot, OTA, camera upload are deliberately not advertised/implemented. */
    cJSON_Delete(obj);
}

static void cloud_task(void *arg)
{
    (void)arg;
    message_t *message = heap_caps_malloc(sizeof(*message), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    capture_t *input = heap_caps_calloc(1, sizeof(*input), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    xz_playback_t *pcm = heap_caps_malloc(sizeof(*pcm), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!message || !input || !pcm || !codec_init()) {
        ESP_LOGE(TAG, "Audio codec allocation/init failed; cloud disabled");
        free(message); free(input); free(pcm); vTaskDeleteWithCaps(NULL); return;
    }
    /* Local encode/decode self-test: synthetic signal, no microphone or network. */
    for (unsigned i=0; i<XZ_PCM_SAMPLES; ++i) input->pcm[i] = (i%32 < 16) ? 500 : -500;
    int encoded = encode(input);
    if (encoded <= 0 || !decode(s_encoded+16, encoded, pcm) || pcm->samples != XZ_PCM_SAMPLES) {
        ESP_LOGE(TAG, "OPUS_SELFTEST_FAILED"); vTaskDeleteWithCaps(NULL); return;
    }
    esp_opus_dec_reset(s_decoder);
    ESP_LOGI(TAG, "OPUS_SELFTEST_OK PCM=1920 bytes encoded=%d decoded=1920 bytes; device=%s", encoded, s_mac);
    ESP_LOGI(TAG, "CODEC_STACK_FREE=%u internal_free=%u", (unsigned)uxTaskGetStackHighWaterMark(NULL),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    ESP_LOGI(TAG, "Continuous listening enabled; tour announcements have priority");
    for (;;) {
        EventBits_t events = xEventGroupGetBits(s_events);
        if (events & (WS_DOWN | AUDIO_FAULT)) {
            close_session(); ESP_LOGW(TAG, "Session stopped (disconnect/audio error); microphone off, xz connect to retry");
        }
        if ((events & WS_UP) && s_ws) {
            xEventGroupClearBits(s_events, WS_UP);
            char hello[256];
            snprintf(hello, sizeof(hello), "{\"type\":\"hello\",\"version\":%d,\"transport\":\"websocket\",\"audio_params\":{\"format\":\"opus\",\"sample_rate\":16000,\"channels\":1,\"frame_duration\":60}}", s_version);
            if (!send_text(hello)) xiaozhi_audio_fault();
        }
        command_t command;
        if (xQueueReceive(s_commands, &command, 0) == pdTRUE) {
            switch (command) {
            case CMD_BIND: close_session(); bind_device(); break;
            case CMD_CONNECT: s_auto=true; connect_session(); break;
            case CMD_LISTEN:
                if (s_hello && !s_turn && uxQueueMessagesWaiting(s_play)==0) listen_message(true);
                else ESP_LOGW(TAG, "Not ready or replying; xz connect first, wait SESSION_READY");
                break;
            case CMD_SEND: if (s_turn && !s_speaking) s_end_listen = true; break;
            case CMD_STOP: s_auto=false; close_session(); ESP_LOGI(TAG, "Stopped: microphone off, playback queue cleared"); break;
            case CMD_STATUS:
                ESP_LOGI(TAG, "STATE configured=%d session_ready=%d recording=%d replying=%d sent=%lu received=%lu",
                         s_url[0]!=0, s_hello, xiaozhi_wants_microphone(), s_speaking,
                         (unsigned long)s_sent, (unsigned long)s_received); break;
            }
        }
        /* Apply backpressure in the optional network task, never in motor/RX audio tasks. */
        for(int i=0;i<4 && uxQueueSpacesAvailable(s_play)>0;++i) {
            if(xQueueReceive(s_rx,message,0)!=pdTRUE) break;
            process_message(message,pcm);
        }
        if (s_hello && s_turn && !s_speaking && xQueueReceive(s_tx, input, 0)==pdTRUE) {
            int len = encode(input);
            if (len <= 0) xiaozhi_audio_fault();
            else {
                uint8_t *payload = s_encoded+16;
                if (s_version == 2) {
                    payload = s_encoded; memset(payload, 0, 16); payload[1]=2;
                    payload[12]=(len>>24)&255; payload[13]=(len>>16)&255; payload[14]=(len>>8)&255; payload[15]=len&255; len+=16;
                } else if (s_version == 3) {
                    payload = s_encoded+12; payload[0]=0; payload[1]=0; payload[2]=(len>>8)&255; payload[3]=len&255; len+=4;
                }
                if (esp_websocket_client_send_bin(s_ws, (char *)payload, len, pdMS_TO_TICKS(1000)) != len) xiaozhi_audio_fault();
                else ++s_sent;
            }
        }
        int64_t now = esp_timer_get_time();
        bool audio_available=atomic_load(&s_mic_ready) && tour_speech_ready() && !tour_speech_busy() && !atomic_load(&s_output_active);
        if(s_auto && !s_ws && audio_available && connected_ip() && now>=s_retry_after) {
            connect_session();
            s_retry_after=esp_timer_get_time()+10000000;
        }
        if(s_auto && s_hello && !s_turn && audio_available && now>=s_listen_after && uxQueueMessagesWaiting(s_play)==0) listen_message(true);
        if (!s_auto && xiaozhi_wants_microphone() && now > s_deadline) { capture_enable(false); s_end_listen=true; }
        if (s_end_listen && uxQueueMessagesWaiting(s_tx)==0) listen_message(false);
        if ((s_hello_deadline && now>s_hello_deadline) || (s_turn && !xiaozhi_wants_microphone() && !s_end_listen && now>s_deadline)) {
            close_session(); ESP_LOGW(TAG, "Session timeout; microphone off");
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

esp_err_t xiaozhi_init(void)
{
    atomic_store(&s_bind_code,-1);
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    snprintf(s_mac, sizeof(s_mac), "%02x:%02x:%02x:%02x:%02x:%02x", mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
    nvs_handle_t nvs;
    esp_err_t error = nvs_open("car_xiaozhi", NVS_READWRITE, &nvs);
    if (error != ESP_OK) return error;
    size_t len = sizeof(s_uuid);
    error = nvs_get_str(nvs, "client_uuid", s_uuid, &len);
    if (error == ESP_ERR_NVS_NOT_FOUND) {
        uint8_t random[16]; esp_fill_random(random, sizeof(random));
        random[6]=(random[6]&15)|64; random[8]=(random[8]&63)|128;
        snprintf(s_uuid, sizeof(s_uuid), "%02x%02x%02x%02x-%02x%02x-%02x%02x-%02x%02x-%02x%02x%02x%02x%02x%02x",
                 random[0],random[1],random[2],random[3],random[4],random[5],random[6],random[7],random[8],random[9],random[10],random[11],random[12],random[13],random[14],random[15]);
        error = nvs_set_str(nvs, "client_uuid", s_uuid);
        if (error == ESP_OK) error=nvs_commit(nvs);
    }
    nvs_close(nvs);
    if (error != ESP_OK) return error;
    uint8_t *rx=heap_caps_malloc(16*sizeof(message_t), MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT);
    uint8_t *tx=heap_caps_malloc(8*sizeof(capture_t), MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT);
    uint8_t *play=heap_caps_malloc(24*sizeof(xz_playback_t), MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT);
    s_assembly=heap_caps_malloc(sizeof(message_t), MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT);
    if (!rx || !tx || !play || !s_assembly) return ESP_ERR_NO_MEM;
    s_rx=xQueueCreateStatic(16,sizeof(message_t),rx,&s_rx_state);
    s_tx=xQueueCreateStatic(8,sizeof(capture_t),tx,&s_tx_state);
    s_play=xQueueCreateStatic(24,sizeof(xz_playback_t),play,&s_play_state);
    s_commands=xQueueCreate(8,sizeof(command_t)); s_events=xEventGroupCreate();
    if (!s_commands || !s_events) return ESP_ERR_NO_MEM;
    if (xTaskCreatePinnedToCoreWithCaps(cloud_task,"xiaozhi",49152,NULL,1,NULL,0,MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT)!=pdPASS) return ESP_ERR_NO_MEM;
    return ESP_OK;
}
