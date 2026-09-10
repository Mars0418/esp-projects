#include "wifi_debug.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "cJSON.h"
#include "esp_event.h"
#include "esp_heap_caps.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"

#define WIDTH 160
#define HEIGHT 120
#define FRAME_BYTES (WIDTH * HEIGHT * 2)
#define JSON_BYTES 2048

static const char *TAG = "WIFI_DEBUG";
extern const char page_start[] asm("_binary_wifi_debug_html_start");
extern const char page_end[] asm("_binary_wifi_debug_html_end");
static SemaphoreHandle_t s_lock;
static QueueHandle_t s_config_queue;
static uint8_t *s_frame;
/* HTTP handlers run serially; never hold s_lock while sending to a client. */
static uint8_t *s_response;
static uint32_t s_sequence;
static int64_t s_captured_us;
static ball_vision_result_t s_balls[3];
static black_marker_result_t s_goal;
static bool s_connected;
static char s_ip[16];
static char s_ap_name[32];
static char s_ssid[33];
static int s_disconnect_reason;
static esp_err_t s_config_error;
static nvs_handle_t s_nvs;

static esp_err_t json_send(httpd_req_t *req, cJSON *obj)
{
    char text[JSON_BYTES];
    if (!obj || !cJSON_PrintPreallocated(obj, text, sizeof(text), false)) {
        cJSON_Delete(obj);
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "JSON allocation failed");
    }
    cJSON_Delete(obj);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_sendstr(req, text);
}

static cJSON *status_json(void)
{
    ball_vision_result_t balls[3];
    black_marker_result_t goal;
    char ip[16], ssid[33];
    bool connected;
    int reason;
    esp_err_t config_error;
    uint32_t sequence;
    int64_t captured;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    memcpy(balls, s_balls, sizeof(balls));
    goal = s_goal;
    memcpy(ip, s_ip, sizeof(ip));
    memcpy(ssid, s_ssid, sizeof(ssid));
    connected = s_connected;
    reason = s_disconnect_reason;
    config_error = s_config_error;
    sequence = s_sequence;
    captured = s_captured_us;
    xSemaphoreGive(s_lock);
    cJSON *obj = cJSON_CreateObject();
    if (!obj) return NULL;
    const int64_t age = sequence ? (esp_timer_get_time() - captured) / 1000 : -1;
    cJSON_AddStringToObject(obj, "mode", "WIFI_DEBUG_MOTORS_DISABLED");
    cJSON_AddStringToObject(obj, "ap_ssid", s_ap_name);
    cJSON_AddStringToObject(obj, "ap_ip", "192.168.4.1");
    cJSON_AddBoolToObject(obj, "sta_connected", connected);
    cJSON_AddStringToObject(obj, "sta_ip", ip);
    cJSON_AddStringToObject(obj, "sta_ssid", ssid);
    cJSON_AddNumberToObject(obj, "disconnect_reason", reason);
    cJSON_AddStringToObject(obj, "config_error", esp_err_to_name(config_error));
    wifi_ap_record_t ap;
    if (connected && esp_wifi_sta_get_ap_info(&ap) == ESP_OK)
        cJSON_AddNumberToObject(obj, "rssi", ap.rssi);
    cJSON_AddNumberToObject(obj, "sequence", sequence);
    cJSON_AddNumberToObject(obj, "age_ms", (double)age);
    cJSON_AddBoolToObject(obj, "camera_fresh", sequence && age < 2000);
    cJSON_AddNumberToObject(obj, "internal_free", heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    const char *names[] = {"red", "white", "purple"};
    for (int i = 0; i < 3; ++i) {
        cJSON *ball = cJSON_AddObjectToObject(obj, names[i]);
        cJSON_AddBoolToObject(ball, "found", balls[i].found);
        cJSON_AddBoolToObject(ball, "predicted", balls[i].predicted);
        cJSON_AddNumberToObject(ball, "confidence", balls[i].confidence);
        cJSON_AddNumberToObject(ball, "x", balls[i].found ? balls[i].center_x : -1);
        cJSON_AddNumberToObject(ball, "y", balls[i].found ? balls[i].center_y : -1);
        cJSON_AddNumberToObject(ball, "pixels", balls[i].purple_pixels);
    }
    cJSON *marker = cJSON_AddObjectToObject(obj, "goal");
    cJSON_AddBoolToObject(marker, "found", goal.found);
    cJSON_AddBoolToObject(marker, "predicted", goal.predicted);
    cJSON_AddNumberToObject(marker, "confidence", goal.confidence);
    return obj;
}

static esp_err_t page_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_set_hdr(req, "X-Content-Type-Options", "nosniff");
    httpd_resp_set_hdr(req, "X-Frame-Options", "DENY");
    return httpd_resp_send(req, page_start, page_end - page_start - 1);
}

static esp_err_t status_handler(httpd_req_t *req)
{
    return json_send(req, status_json());
}

static esp_err_t frame_handler(httpd_req_t *req)
{
    // Packet: uint32 JSON byte count (little endian), JSON, big-endian RGB565.
    char header[192];
    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (!s_sequence) {
        xSemaphoreGive(s_lock);
        httpd_resp_set_status(req, "503 Service Unavailable");
        httpd_resp_set_type(req, "text/plain");
        return httpd_resp_sendstr(req, "Camera has not produced a frame");
    }
    int len = snprintf(header, sizeof(header),
        "{\"width\":%d,\"height\":%d,\"sequence\":%lu,\"age_ms\":%lld}",
        WIDTH, HEIGHT, (unsigned long)s_sequence,
        (long long)((esp_timer_get_time() - s_captured_us) / 1000));
    for (int i = 0; i < 4; ++i) s_response[i] = ((uint32_t)len >> (8 * i)) & 255;
    memcpy(s_response + 4, header, len);
    memcpy(s_response + 4 + len, s_frame, FRAME_BYTES);
    xSemaphoreGive(s_lock);
    httpd_resp_set_type(req, "application/octet-stream");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, (const char *)s_response, 4 + len + FRAME_BYTES);
}

static esp_err_t config_handler(httpd_req_t *req)
{
    // Custom header + JSON prevents cross-origin HTML forms from changing Wi-Fi.
    char guard[4];
    if (httpd_req_get_hdr_value_str(req, "X-Car-Config", guard, sizeof(guard)) != ESP_OK ||
        strcmp(guard, "1") != 0)
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Use the local configuration page");
    if (req->content_len <= 0 || req->content_len > 512)
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid configuration length");
    char body[513];
    int used = 0;
    while (used < req->content_len) {
        int received = httpd_req_recv(req, body + used, req->content_len - used);
        if (received <= 0) return ESP_FAIL;
        used += received;
    }
    body[used] = 0;
    cJSON *obj = cJSON_Parse(body);
    cJSON *ssid = cJSON_GetObjectItemCaseSensitive(obj, "ssid");
    cJSON *pass = cJSON_GetObjectItemCaseSensitive(obj, "password");
    if (!cJSON_IsString(ssid) || !cJSON_IsString(pass) ||
        strlen(ssid->valuestring) == 0 || strlen(ssid->valuestring) > 32 ||
        strlen(pass->valuestring) > 63 ||
        (strlen(pass->valuestring) != 0 && strlen(pass->valuestring) < 8)) {
        cJSON_Delete(obj);
        memset(body, 0, sizeof(body));
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
            "SSID must be 1-32 UTF-8 bytes; password empty or 8-63 bytes");
    }
    wifi_config_t config = {0};
    memcpy(config.sta.ssid, ssid->valuestring, strlen(ssid->valuestring));
    memcpy(config.sta.password, pass->valuestring, strlen(pass->valuestring));
    config.sta.threshold.authmode = strlen(pass->valuestring) ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;
    config.sta.pmf_cfg.capable = true;
    xQueueOverwrite(s_config_queue, &config);
    memset(&config, 0, sizeof(config));
    cJSON_Delete(obj);
    memset(body, 0, sizeof(body));
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"accepted\":true}");
}

static void network_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = data;
        char ip[16];
        snprintf(ip, sizeof(ip), IPSTR, IP2STR(&event->ip_info.ip));
        xSemaphoreTake(s_lock, portMAX_DELAY);
        memcpy(s_ip, ip, sizeof(ip));
        s_connected = true;
        s_disconnect_reason = 0;
        xSemaphoreGive(s_lock);
        ESP_LOGI(TAG, "LAN_URL=http://%s/ (same network)", ip);
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *event = data;
        xSemaphoreTake(s_lock, portMAX_DELAY);
        s_connected = false;
        s_ip[0] = 0;
        s_disconnect_reason = event->reason;
        xSemaphoreGive(s_lock);
        ESP_LOGW(TAG, "Station disconnected reason=%d; AP remains available", event->reason);
    }
}

bool wifi_debug_uart_command(const char *line)
{
    if (strncmp(line, "wifi config ", 12) != 0) return false;
    cJSON *obj = cJSON_Parse(line + 12);
    cJSON *ssid = cJSON_GetObjectItemCaseSensitive(obj, "ssid");
    cJSON *pass = cJSON_GetObjectItemCaseSensitive(obj, "password");
    if (!cJSON_IsString(ssid) || !cJSON_IsString(pass) ||
        strlen(ssid->valuestring) == 0 || strlen(ssid->valuestring) > 32 ||
        strlen(pass->valuestring) > 63 ||
        (strlen(pass->valuestring) != 0 && strlen(pass->valuestring) < 8)) {
        ESP_LOGW(TAG, "UART_CONFIG invalid SSID/password length");
    } else {
        wifi_config_t config = {0};
        memcpy(config.sta.ssid, ssid->valuestring, strlen(ssid->valuestring));
        memcpy(config.sta.password, pass->valuestring, strlen(pass->valuestring));
        config.sta.threshold.authmode = strlen(pass->valuestring) ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;
        config.sta.pmf_cfg.capable = true;
        xQueueOverwrite(s_config_queue, &config);
        memset(&config, 0, sizeof(config));
        ESP_LOGI(TAG, "UART_CONFIG accepted (credentials not logged)");
    }
    if (cJSON_IsString(pass)) memset(pass->valuestring, 0, strlen(pass->valuestring));
    cJSON_Delete(obj);
    return true;
}

static void network_task(void *arg)
{
    bool configured = false;
    wifi_config_t config = {0};
/* XiaoZhi bring-up: user explicitly restored use of the saved phone hotspot. */
#if defined(CONFIG_CAR_WIFI_RESTORE_STATION) || defined(CONFIG_CAR_XIAOZHI)
    size_t size = sizeof(config);
    if (nvs_get_blob(s_nvs, "sta_config", &config, &size) == ESP_OK && size == sizeof(config))
        xQueueOverwrite(s_config_queue, &config);
#else
    ESP_LOGI(TAG, "Saved station auto-connect disabled; use UART for audio bring-up");
#endif
    for (;;) {
        if (xQueueReceive(s_config_queue, &config, pdMS_TO_TICKS(10000)) == pdTRUE) {
            esp_wifi_disconnect();
            esp_err_t error = esp_wifi_set_config(WIFI_IF_STA, &config);
            if (error == ESP_OK) error = nvs_set_blob(s_nvs, "sta_config", &config, sizeof(config));
            if (error == ESP_OK) error = nvs_commit(s_nvs);
            xSemaphoreTake(s_lock, portMAX_DELAY);
            s_connected = false;
            s_ip[0] = 0;
            s_config_error = error;
            s_disconnect_reason = 0;
            memcpy(s_ssid, config.sta.ssid, 32);
            s_ssid[32] = 0;
            xSemaphoreGive(s_lock);
            memset(&config, 0, sizeof(config));
            configured = error == ESP_OK;
            ESP_LOGI(TAG, "Station configuration result=%s", esp_err_to_name(error));
        }
        xSemaphoreTake(s_lock, portMAX_DELAY);
        bool connected = s_connected;
        xSemaphoreGive(s_lock);
        if (configured && !connected) esp_wifi_connect();
    }
}

esp_err_t wifi_debug_init(void)
{
    s_lock = xSemaphoreCreateMutex();
    s_config_queue = xQueueCreate(1, sizeof(wifi_config_t));
    s_frame = heap_caps_malloc(FRAME_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    s_response = heap_caps_malloc(FRAME_BYTES + JSON_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_lock || !s_config_queue || !s_frame || !s_response) return ESP_ERR_NO_MEM;
    // Preserve all existing NVS data; never erase it automatically on errors.
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(nvs_open("car_wifi_debug", NVS_READWRITE, &s_nvs));
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    if (!esp_netif_create_default_wifi_ap() || !esp_netif_create_default_wifi_sta())
        return ESP_ERR_NO_MEM;
    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, network_event, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, network_event, NULL));
    uint8_t mac[6];
    ESP_ERROR_CHECK(esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP));
    snprintf(s_ap_name, sizeof(s_ap_name), "BallCar-%02X%02X%02X", mac[3], mac[4], mac[5]);
    char password[17];
    size_t length = sizeof(password);
    esp_err_t error = nvs_get_str(s_nvs, "ap_password", password, &length);
    if (error == ESP_ERR_NVS_NOT_FOUND) {
        snprintf(password, sizeof(password), "%08lx%08lx", (unsigned long)esp_random(), (unsigned long)esp_random());
        ESP_ERROR_CHECK(nvs_set_str(s_nvs, "ap_password", password));
        ESP_ERROR_CHECK(nvs_commit(s_nvs));
    } else if (error != ESP_OK) return error;
    wifi_config_t ap = {0};
    memcpy(ap.ap.ssid, s_ap_name, strlen(s_ap_name));
    ap.ap.ssid_len = strlen(s_ap_name);
    memcpy(ap.ap.password, password, strlen(password));
    ap.ap.authmode = WIFI_AUTH_WPA2_PSK;
    ap.ap.channel = 6;
    ap.ap.max_connection = 3;
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    httpd_config_t server_config = HTTPD_DEFAULT_CONFIG();
    server_config.stack_size = 6144;
    server_config.max_open_sockets = 4;
    server_config.lru_purge_enable = true;
    server_config.send_wait_timeout = 2;
    server_config.recv_wait_timeout = 2;
    httpd_handle_t server;
    ESP_ERROR_CHECK(httpd_start(&server, &server_config));
    const httpd_uri_t endpoints[] = {
        {.uri = "/", .method = HTTP_GET, .handler = page_handler},
        {.uri = "/api/status", .method = HTTP_GET, .handler = status_handler},
        {.uri = "/api/frame", .method = HTTP_GET, .handler = frame_handler},
        {.uri = "/api/wifi", .method = HTTP_POST, .handler = config_handler},
    };
    for (size_t i = 0; i < sizeof(endpoints) / sizeof(endpoints[0]); ++i)
        ESP_ERROR_CHECK(httpd_register_uri_handler(server, &endpoints[i]));
    if (xTaskCreate(network_task, "wifi_connect", 4096, NULL, 3, NULL) != pdPASS)
        return ESP_ERR_NO_MEM;
    ESP_LOGI(TAG, "AP_SSID=%s AP_PASSWORD=%s AP_URL=http://192.168.4.1/", s_ap_name, password);
    ESP_LOGW(TAG, "WIFI_DEBUG: motor control disabled; camera/status/configuration only");
    memset(password, 0, sizeof(password));
    return ESP_OK;
}

void wifi_debug_publish(const uint8_t *rgb565, size_t width, size_t height,
                        int64_t captured_us, const ball_vision_result_t *red,
                        const ball_vision_result_t *white, const ball_vision_result_t *purple,
                        const black_marker_result_t *goal)
{
    if (!s_lock || width != WIDTH || height != HEIGHT) return;
    // Slow clients cannot block the USB/recognition task.
    if (xSemaphoreTake(s_lock, 0) != pdTRUE) return;
    memcpy(s_frame, rgb565, FRAME_BYTES);
    s_balls[0] = *red;
    s_balls[1] = *white;
    s_balls[2] = *purple;
    s_goal = *goal;
    s_captured_us = captured_us;
    ++s_sequence;
    xSemaphoreGive(s_lock);
}
