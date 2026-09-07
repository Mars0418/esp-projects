#include "push_debug_stream.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "driver/uart.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#define DEBUG_UART UART_NUM_0
#define DEBUG_NORMAL_BAUD 115200
#define DEBUG_STREAM_BAUD 921600
#define DEBUG_WIDTH 160
#define DEBUG_HEIGHT 120
#define DEBUG_RGB565_BYTES (DEBUG_WIDTH * DEBUG_HEIGHT * 2)
#define DEBUG_RGB332_BYTES (DEBUG_WIDTH * DEBUG_HEIGHT)
#define DEBUG_SLOT_COUNT 2
#define DEBUG_INTERVAL_US 250000
#define DEBUG_METADATA_BYTES 1536
#define DEBUG_PREFIX_BYTES 20
#define DEBUG_PACKET_BYTES \
    (DEBUG_PREFIX_BYTES + DEBUG_METADATA_BYTES + DEBUG_RGB332_BYTES)
#define DEBUG_TX_DRAIN_MS 800

typedef struct {
    uint8_t *rgb565;
    push_debug_metadata_t metadata;
    char mission_state_name[32];
    char push_entry_name[32];
    bool busy;
} debug_slot_t;

static const char *TAG = "push_debug";
static const uint8_t s_magic[8] = {'\n', '@', 'P', 'D', 'B', 'G', '1', '\n'};
static QueueHandle_t s_queue;
static debug_slot_t s_slots[DEBUG_SLOT_COUNT];
static uint8_t *s_packet;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static volatile bool s_enabled;
static int64_t s_last_queued_us;
static uint32_t s_sequence;
static uint32_t s_queued_frames;
static uint32_t s_dropped_frames;

static void write_u32_le(uint8_t *destination, uint32_t value)
{
    destination[0] = (uint8_t)value;
    destination[1] = (uint8_t)(value >> 8);
    destination[2] = (uint8_t)(value >> 16);
    destination[3] = (uint8_t)(value >> 24);
}

static uint32_t crc32_update(uint32_t crc, const uint8_t *data,
                             size_t length)
{
    for (size_t index = 0; index < length; ++index) {
        crc ^= data[index];
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc >> 1) ^ (0xedb88320U &
                  (uint32_t)-(int32_t)(crc & 1U));
        }
    }
    return crc;
}

static uint8_t rgb565_to_rgb332(const uint8_t *source)
{
    const uint16_t color = ((uint16_t)source[0] << 8) | source[1];
    return (uint8_t)(((color >> 8) & 0xe0U) |
                     ((color >> 6) & 0x1cU) |
                     ((color >> 3) & 0x03U));
}

static int build_metadata(char *buffer, size_t capacity,
                          const debug_slot_t *slot, uint32_t sequence,
                          int64_t emitted_at_us)
{
    const push_debug_metadata_t *m = &slot->metadata;
    return snprintf(
        buffer, capacity,
        "{\"v\":1,\"seq\":%" PRIu32 ",\"capture_us\":%" PRId64
        ",\"processed_us\":%" PRId64 ",\"emitted_us\":%" PRId64
        ",\"width\":%d,\"height\":%d,\"format\":\"RGB332\""
        ",\"stream\":{\"queued\":%" PRIu32 ",\"dropped\":%" PRIu32 "}"
        ",\"mission\":{\"state_id\":%d,\"state\":\"%s\""
        ",\"push_entry_id\":%d,\"push_entry\":\"%s\""
        ",\"selected_ball\":%d,\"target_goal\":%d,\"ball_held\":%d"
        ",\"visual_goal_locked\":%d,\"filtered_goal_mm\":[%d,%d]"
        ",\"tracked_ball_field_mm\":[%d,%d]}"
        ",\"red\":{\"found\":%d,\"predicted\":%d,\"confidence\":%d"
        ",\"center\":[%d,%d],\"box\":[%d,%d,%d,%d]}"
        ",\"white\":{\"found\":%d,\"predicted\":%d,\"confidence\":%d"
        ",\"center\":[%d,%d],\"box\":[%d,%d,%d,%d]}"
        ",\"purple\":{\"found\":%d,\"predicted\":%d,\"confidence\":%d"
        ",\"center\":[%d,%d],\"box\":[%d,%d,%d,%d]}"
        ",\"goal\":{\"found\":%d,\"predicted\":%d,\"confidence\":%d"
        ",\"center\":[%d,%d],\"box\":[%d,%d,%d,%d]"
        ",\"corner_found\":%d,\"corner_confidence\":%d"
        ",\"corner\":[%d,%d]}"
        ",\"navigation\":{\"valid\":%d,\"state\":%d,\"command\":%d"
        ",\"source\":\"%s\",\"pose_mm_deg\":[%d,%d,%d]"
        ",\"target_field_mm\":[%d,%d],\"distance_mm\":%d"
        ",\"visual_target_mm\":[%d,%d]"
        ",\"reverse_stall_floor_pwm\":%d"
        ",\"wheel_pwm\":[%d,%d,%d]"
        ",\"encoder_delta\":[%d,%d,%d]}}",
        sequence, m->captured_at_us, m->processed_at_us, emitted_at_us,
        DEBUG_WIDTH, DEBUG_HEIGHT, s_queued_frames, s_dropped_frames,
        m->mission_state, slot->mission_state_name,
        m->push_entry, slot->push_entry_name, m->selected_ball,
        m->target_goal, m->ball_held, m->visual_goal_locked,
        (int)m->filtered_goal_right_mm,
        (int)m->filtered_goal_forward_mm,
        (int)m->tracked_ball_field_x_mm,
        (int)m->tracked_ball_field_y_mm,
        m->red_ball.found, m->red_ball.predicted, m->red_ball.confidence,
        m->red_ball.center_x, m->red_ball.center_y,
        m->red_ball.left, m->red_ball.top,
        m->red_ball.right, m->red_ball.bottom,
        m->white_ball.found, m->white_ball.predicted,
        m->white_ball.confidence,
        m->white_ball.center_x, m->white_ball.center_y,
        m->white_ball.left, m->white_ball.top,
        m->white_ball.right, m->white_ball.bottom,
        m->purple_ball.found, m->purple_ball.predicted,
        m->purple_ball.confidence,
        m->purple_ball.center_x, m->purple_ball.center_y,
        m->purple_ball.left, m->purple_ball.top,
        m->purple_ball.right, m->purple_ball.bottom,
        m->goal.found, m->goal.predicted, m->goal.confidence,
        m->goal.center_x, m->goal.center_y,
        m->goal.left, m->goal.top, m->goal.right, m->goal.bottom,
        m->corner_found, m->corner_confidence, m->corner_x, m->corner_y,
        m->navigation_valid, m->navigation_state, m->navigation_command,
        m->visual_control_active ? "VISION" : "ODOMETRY",
        (int)m->vehicle_x_mm, (int)m->vehicle_y_mm,
        (int)m->vehicle_heading_deg,
        (int)m->navigation_target_x_mm,
        (int)m->navigation_target_y_mm,
        (int)m->navigation_distance_mm,
        (int)m->visual_target_right_mm,
        (int)m->visual_target_forward_mm,
        m->reverse_stall_duty_floor,
        m->wheel_pwm_a, m->wheel_pwm_b, m->wheel_pwm_d,
        m->encoder_delta_a, m->encoder_delta_b, m->encoder_delta_d);
}

static void release_slot(int slot_index)
{
    portENTER_CRITICAL(&s_lock);
    s_slots[slot_index].busy = false;
    portEXIT_CRITICAL(&s_lock);
}

static void debug_stream_task(void *argument)
{
    (void)argument;
    int slot_index;
    while (true) {
        if (xQueueReceive(s_queue, &slot_index, portMAX_DELAY) != pdPASS) {
            continue;
        }
        debug_slot_t *slot = &s_slots[slot_index];
        if (!s_enabled) {
            release_slot(slot_index);
            continue;
        }

        const uint32_t sequence = ++s_sequence;
        const int metadata_length = build_metadata(
            (char *)(s_packet + DEBUG_PREFIX_BYTES), DEBUG_METADATA_BYTES,
            slot, sequence, esp_timer_get_time());
        if (metadata_length <= 0 || metadata_length >= DEBUG_METADATA_BYTES) {
            s_dropped_frames++;
            release_slot(slot_index);
            continue;
        }

        uint8_t *rgb332 = s_packet + DEBUG_PREFIX_BYTES + metadata_length;
        for (size_t pixel = 0; pixel < DEBUG_RGB332_BYTES; ++pixel) {
            rgb332[pixel] = rgb565_to_rgb332(slot->rgb565 + pixel * 2);
        }

        memcpy(s_packet, s_magic, sizeof(s_magic));
        write_u32_le(s_packet + 8, (uint32_t)metadata_length);
        write_u32_le(s_packet + 12, DEBUG_RGB332_BYTES);
        uint32_t crc = crc32_update(0xffffffffU,
                                    s_packet + DEBUG_PREFIX_BYTES,
                                    (size_t)metadata_length +
                                        DEBUG_RGB332_BYTES) ^ 0xffffffffU;
        write_u32_le(s_packet + 16, crc);

        const size_t packet_length = DEBUG_PREFIX_BYTES +
            (size_t)metadata_length + DEBUG_RGB332_BYTES;
        if (s_enabled && uart_write_bytes(DEBUG_UART, s_packet,
                                          packet_length) < 0) {
            s_dropped_frames++;
        }
        release_slot(slot_index);
        taskYIELD();
    }
}

esp_err_t push_debug_stream_init(void)
{
    s_queue = xQueueCreate(DEBUG_SLOT_COUNT, sizeof(int));
    if (s_queue == NULL) return ESP_ERR_NO_MEM;

    for (int index = 0; index < DEBUG_SLOT_COUNT; ++index) {
        s_slots[index].rgb565 = heap_caps_malloc(
            DEBUG_RGB565_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (s_slots[index].rgb565 == NULL) return ESP_ERR_NO_MEM;
    }
    s_packet = heap_caps_malloc(DEBUG_PACKET_BYTES,
                                MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (s_packet == NULL) return ESP_ERR_NO_MEM;

    if (xTaskCreatePinnedToCore(debug_stream_task, "push_debug", 4096,
                                NULL, 2, NULL, 0) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

esp_err_t push_debug_stream_set_enabled(bool enabled)
{
    if (s_queue == NULL) return ESP_ERR_INVALID_STATE;
    if (enabled == s_enabled) return ESP_OK;

    if (enabled) {
        static const char switching[] = "VSTREAM_SWITCH,921600\r\n";
        uart_write_bytes(DEBUG_UART, switching, sizeof(switching) - 1);
        uart_wait_tx_done(DEBUG_UART, pdMS_TO_TICKS(DEBUG_TX_DRAIN_MS));
        esp_log_level_set("*", ESP_LOG_NONE);
        ESP_RETURN_ON_ERROR(uart_set_baudrate(DEBUG_UART,
                                              DEBUG_STREAM_BAUD), TAG,
                            "Cannot switch debug UART baud");
        s_last_queued_us = 0;
        s_enabled = true;
    } else {
        s_enabled = false;
        uart_wait_tx_done(DEBUG_UART, pdMS_TO_TICKS(DEBUG_TX_DRAIN_MS));
        ESP_RETURN_ON_ERROR(uart_set_baudrate(DEBUG_UART,
                                              DEBUG_NORMAL_BAUD), TAG,
                            "Cannot restore debug UART baud");
        esp_log_level_set("*", ESP_LOG_INFO);
        ESP_LOGI(TAG, "VSTREAM disabled; UART restored to %d",
                 DEBUG_NORMAL_BAUD);
    }
    return ESP_OK;
}

bool push_debug_stream_is_enabled(void)
{
    return s_enabled;
}

bool push_debug_stream_queue_frame(const uint8_t *rgb565, size_t width,
                                   size_t height,
                                   const push_debug_metadata_t *metadata)
{
    if (!s_enabled || rgb565 == NULL || metadata == NULL ||
        width != DEBUG_WIDTH || height != DEBUG_HEIGHT ||
        metadata->processed_at_us - s_last_queued_us < DEBUG_INTERVAL_US) {
        return false;
    }

    int slot_index = -1;
    portENTER_CRITICAL(&s_lock);
    for (int index = 0; index < DEBUG_SLOT_COUNT; ++index) {
        if (!s_slots[index].busy) {
            s_slots[index].busy = true;
            slot_index = index;
            break;
        }
    }
    portEXIT_CRITICAL(&s_lock);
    if (slot_index < 0) {
        s_dropped_frames++;
        return false;
    }

    debug_slot_t *slot = &s_slots[slot_index];
    memcpy(slot->rgb565, rgb565, DEBUG_RGB565_BYTES);
    slot->metadata = *metadata;
    snprintf(slot->mission_state_name, sizeof(slot->mission_state_name),
             "%s", metadata->mission_state_name != NULL
                 ? metadata->mission_state_name : "UNKNOWN");
    slot->metadata.mission_state_name = slot->mission_state_name;
    snprintf(slot->push_entry_name, sizeof(slot->push_entry_name),
             "%s", metadata->push_entry_name != NULL
                 ? metadata->push_entry_name : "NONE");
    slot->metadata.push_entry_name = slot->push_entry_name;
    if (xQueueSend(s_queue, &slot_index, 0) != pdPASS) {
        release_slot(slot_index);
        s_dropped_frames++;
        return false;
    }
    s_last_queued_us = metadata->processed_at_us;
    s_queued_frames++;
    return true;
}
