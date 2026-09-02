#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "ball_vision.h"
#include "black_marker_vision.h"
#include "camera_display.h"
#include "camera_line_follow.h"
#include "driver/uart.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_intr_alloc.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "jpeg_decoder.h"
#include "libuvc/libuvc.h"
#include "libuvc_adapter.h"
#include "libuvc_helper.h"
#include "line_vision.h"
#include "mbedtls/base64.h"
#include "usb/usb_host.h"

#define MJPEG_SLOT_COUNT      2
#define FRAME_QUEUE_LENGTH    1
#define DISPLAY_QUEUE_LENGTH  1
#define MJPEG_SLOT_CAPACITY   (256 * 1024)
#define DECODED_WIDTH         160
#define DECODED_HEIGHT        120
#define DECODED_BUFFER_BYTES  (DECODED_WIDTH * DECODED_HEIGHT * 2)
#define JPEG_WORK_BUFFER_BYTES 8192
#define RGB_DEBUG_WIDTH       32
#define RGB_DEBUG_HEIGHT      24
#define RGB_DEBUG_INTERVAL_US 500000
#define TFT_PREVIEW_INTERVAL_US 60000
#define TUNER_INTERVAL_US     500000
#define CALIBRATION_INTERVAL_US 1000000
#define RGB_DEBUG_PIXELS      (RGB_DEBUG_WIDTH * RGB_DEBUG_HEIGHT)
#define RGB_DEBUG_BYTES       (RGB_DEBUG_PIXELS * 2)
#define RGB_DEBUG_MASK_BYTES  ((RGB_DEBUG_PIXELS + 7) / 8)
#define RGB_DEBUG_B64_BYTES   (((RGB_DEBUG_BYTES + 2) / 3) * 4 + 1)
#define MASK_DEBUG_B64_BYTES  (((RGB_DEBUG_MASK_BYTES + 2) / 3) * 4 + 1)
#define TUNER_MASK_BYTES      ((DECODED_WIDTH * DECODED_HEIGHT + 7) / 8)

typedef struct {
    uint8_t *data;
    size_t length;
    size_t step;
    uint16_t width;
    uint16_t height;
    enum uvc_frame_format format;
    int64_t captured_at_us;
    bool busy;
} mjpeg_slot_t;

static const char *TAG = "CAMERA_VIEW";
static EventGroupHandle_t s_uvc_events;
static QueueHandle_t s_frame_queue;
static QueueHandle_t s_display_queue;
static mjpeg_slot_t s_slots[MJPEG_SLOT_COUNT];
static uint8_t *s_decoded_frame;
static uint8_t *s_display_frame;
static uint8_t *s_debug_raw_frame;
static uint8_t *s_jpeg_work_buffer;
static portMUX_TYPE s_slot_lock = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE s_display_lock = portMUX_INITIALIZER_UNLOCKED;
static bool s_display_busy;
static volatile uint32_t s_received_frames;
static volatile uint32_t s_displayed_frames;
static volatile uint32_t s_dropped_frames;
static bool s_capture_core_reported;
static uint8_t s_rgb_debug_pixels[RGB_DEBUG_BYTES];
static uint8_t s_rgb_debug_mask[RGB_DEBUG_MASK_BYTES];
static unsigned char s_rgb_debug_b64[RGB_DEBUG_B64_BYTES];
static unsigned char s_mask_debug_b64[MASK_DEBUG_B64_BYTES];
static char s_rgb_debug_line[2560];
static uint32_t s_rgb_debug_sequence;
static uint8_t *s_tuner_mask;
static char s_tuner_header[256];
static uint32_t s_tuner_sequence;
static char s_calibration_header[96];
static uint32_t s_calibration_sequence;
static int s_stream_width = 640;
static int s_stream_height = 480;
static int s_stream_fps = 15;
static enum uvc_frame_format s_stream_format = UVC_FRAME_FORMAT_MJPEG;
static esp_jpeg_image_scale_t s_decode_scale = JPEG_IMAGE_SCALE_1_4;

static esp_err_t downsample_yuyv_luma(const mjpeg_slot_t *slot);

static bool queue_tft_preview(const uint8_t *pixels)
{
    bool reserved = false;
    portENTER_CRITICAL(&s_display_lock);
    if (!s_display_busy) {
        s_display_busy = true;
        reserved = true;
    }
    portEXIT_CRITICAL(&s_display_lock);
    if (!reserved) return false;

    memcpy(s_display_frame, pixels, DECODED_BUFFER_BYTES);
    const uint8_t signal = 1;
    if (xQueueSend(s_display_queue, &signal, 0) != pdPASS) {
        portENTER_CRITICAL(&s_display_lock);
        s_display_busy = false;
        portEXIT_CRITICAL(&s_display_lock);
        return false;
    }
    return true;
}

static void tft_display_task(void *argument)
{
    (void)argument;
    uint8_t signal;
    while (true) {
        if (xQueueReceive(s_display_queue, &signal, portMAX_DELAY) != pdPASS) {
            continue;
        }
        const esp_err_t error = camera_display_show_rotated_rgb565(
            s_display_frame, DECODED_WIDTH, DECODED_HEIGHT);
        if (error == ESP_OK) {
            s_displayed_frames++;
        } else {
            ESP_LOGW(TAG, "TFT preview failed: %s", esp_err_to_name(error));
        }
        portENTER_CRITICAL(&s_display_lock);
        s_display_busy = false;
        portEXIT_CRITICAL(&s_display_lock);
    }
}

static void emit_rgb_debug_frame(const uint8_t *raw_pixels,
                                 size_t width, size_t height,
                                 const line_vision_result_t *result)
{
    memset(s_rgb_debug_mask, 0, sizeof(s_rgb_debug_mask));
    for (size_t sample_y = 0; sample_y < RGB_DEBUG_HEIGHT; ++sample_y) {
        const size_t source_y = (sample_y * height + height / 2) /
                                RGB_DEBUG_HEIGHT;
        for (size_t sample_x = 0; sample_x < RGB_DEBUG_WIDTH; ++sample_x) {
            const size_t source_x = (sample_x * width + width / 2) /
                                    RGB_DEBUG_WIDTH;
            const size_t source_index = source_y * width + source_x;
            const size_t sample_index = sample_y * RGB_DEBUG_WIDTH + sample_x;
            s_rgb_debug_pixels[sample_index * 2] =
                raw_pixels[source_index * 2];
            s_rgb_debug_pixels[sample_index * 2 + 1] =
                raw_pixels[source_index * 2 + 1];
            if (line_vision_pixel_selected(source_index)) {
                s_rgb_debug_mask[sample_index / 8] |=
                    (uint8_t)(1U << (sample_index % 8));
            }
        }
    }

    size_t rgb_length = 0;
    size_t mask_length = 0;
    if (mbedtls_base64_encode(s_rgb_debug_b64,
                              sizeof(s_rgb_debug_b64),
                              &rgb_length, s_rgb_debug_pixels,
                              sizeof(s_rgb_debug_pixels)) != 0 ||
        mbedtls_base64_encode(s_mask_debug_b64,
                              sizeof(s_mask_debug_b64),
                              &mask_length, s_rgb_debug_mask,
                              sizeof(s_rgb_debug_mask)) != 0) {
        return;
    }
    s_rgb_debug_b64[rgb_length] = '\0';
    s_mask_debug_b64[mask_length] = '\0';
    const line_vision_rgb_thresholds_t thresholds =
        line_vision_get_rgb_thresholds();
    const int line_length = snprintf(
        s_rgb_debug_line, sizeof(s_rgb_debug_line),
        "@RGB,%lu,%d,%d,%u,%u,%u,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%s,%s\n",
        (unsigned long)++s_rgb_debug_sequence,
        RGB_DEBUG_WIDTH, RGB_DEBUG_HEIGHT,
        thresholds.red, thresholds.green, thresholds.blue,
        result->found, result->confidence, result->steering_error,
        result->near_x, result->far_x, result->big_turn,
        result->turn_direction, result->turn_angle_deg,
        result->turn_confidence, result->corner_y,
        s_rgb_debug_b64, s_mask_debug_b64);
    if (line_length > 0 && line_length < (int)sizeof(s_rgb_debug_line)) {
        uart_write_bytes(UART_NUM_0, s_rgb_debug_line, line_length);
    }
}

static void emit_tuner_frame(const uint8_t *raw_pixels,
                             size_t width, size_t height,
                             const line_vision_result_t *result)
{
    const size_t pixel_count = width * height;
    const size_t mask_bytes = (pixel_count + 7) / 8;
    if (width != DECODED_WIDTH || height != DECODED_HEIGHT ||
        mask_bytes > TUNER_MASK_BYTES) {
        return;
    }

    memset(s_tuner_mask, 0, mask_bytes);
    for (size_t index = 0; index < pixel_count; ++index) {
        if (line_vision_pixel_selected(index)) {
            s_tuner_mask[index / 8] |= (uint8_t)(1U << (index % 8));
        }
    }

    const line_vision_rgb_thresholds_t thresholds =
        line_vision_get_rgb_thresholds();
    const int header_length = snprintf(
        s_tuner_header, sizeof(s_tuner_header),
        "@RGB565,%lu,%u,%u,%u,%u,%u,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%u,%u\n",
        (unsigned long)++s_tuner_sequence,
        (unsigned)width, (unsigned)height,
        thresholds.red, thresholds.green, thresholds.blue,
        result->found, result->confidence, result->steering_error,
        result->near_x, result->far_x, result->big_turn,
        result->turn_direction, result->turn_angle_deg,
        result->turn_confidence, result->corner_y,
        (unsigned)(pixel_count * 2), (unsigned)mask_bytes);
    if (header_length <= 0 || header_length >= (int)sizeof(s_tuner_header)) {
        return;
    }

    uart_write_bytes(UART_NUM_0, s_tuner_header, header_length);
    uart_write_bytes(UART_NUM_0, raw_pixels, pixel_count * 2);
    uart_write_bytes(UART_NUM_0, s_tuner_mask, mask_bytes);
}

static void emit_calibration_jpeg(const mjpeg_slot_t *slot)
{
    if (slot->format != UVC_FRAME_FORMAT_MJPEG || slot->length < 4) {
        return;
    }
    const int header_length = snprintf(
        s_calibration_header, sizeof(s_calibration_header),
        "@CALJPEG,%lu,%u,%u,%u\n",
        (unsigned long)++s_calibration_sequence,
        (unsigned)slot->width, (unsigned)slot->height,
        (unsigned)slot->length);
    if (header_length <= 0 ||
        header_length >= (int)sizeof(s_calibration_header)) {
        return;
    }
    uart_write_bytes(UART_NUM_0, s_calibration_header, header_length);
    uart_write_bytes(UART_NUM_0, slot->data, slot->length);
}

static void usb_library_task(void *argument)
{
    (void)argument;
    while (true) {
        uint32_t event_flags = 0;
        const esp_err_t error = usb_host_lib_handle_events(portMAX_DELAY,
                                                           &event_flags);
        if (error != ESP_OK) {
            ESP_LOGE(TAG, "USB library event error: %s",
                     esp_err_to_name(error));
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            usb_host_device_free_all();
        }
    }
}

static esp_err_t initialize_usb_host(void)
{
    const usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
        .peripheral_map = BIT0,
    };
    ESP_RETURN_ON_ERROR(usb_host_install(&host_config), TAG,
                        "USB host install failed");
    if (xTaskCreatePinnedToCore(usb_library_task, "usb_events", 4096, NULL,
                                2, NULL, 0) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

static void libuvc_event_callback(libuvc_adapter_event_t event)
{
    xEventGroupSetBits(s_uvc_events, event);
}

static void wait_for_uvc_event(EventBits_t event)
{
    xEventGroupWaitBits(s_uvc_events, event, pdTRUE, pdFALSE, portMAX_DELAY);
}

static int reserve_mjpeg_slot(void)
{
    int result = -1;
    portENTER_CRITICAL(&s_slot_lock);
    for (int index = 0; index < MJPEG_SLOT_COUNT; ++index) {
        if (!s_slots[index].busy) {
            s_slots[index].busy = true;
            result = index;
            break;
        }
    }
    portEXIT_CRITICAL(&s_slot_lock);
    return result;
}

static void release_mjpeg_slot(int index)
{
    portENTER_CRITICAL(&s_slot_lock);
    s_slots[index].busy = false;
    portEXIT_CRITICAL(&s_slot_lock);
}

static void camera_frame_callback(uvc_frame_t *frame, void *user_pointer)
{
    (void)user_pointer;
    if (!s_capture_core_reported) {
        ESP_LOGI(TAG, "CAPTURE_CALLBACK core=%d", xPortGetCoreID());
        s_capture_core_reported = true;
    }
    s_received_frames++;

    const uint8_t *frame_bytes = frame->data;
    size_t frame_length = frame->data_bytes;
    if (frame->frame_format == UVC_FRAME_FORMAT_MJPEG) {
        while (frame_length >= 2 &&
               !(frame_bytes[frame_length - 2] == 0xff &&
                 frame_bytes[frame_length - 1] == 0xd9)) {
            frame_length--;
        }
        if (frame_length < 4 || frame_bytes[0] != 0xff ||
            frame_bytes[1] != 0xd8) {
            s_dropped_frames++;
            return;
        }
    } else if (frame->frame_format == UVC_FRAME_FORMAT_YUYV) {
        if (frame->step < frame->width * 2 ||
            frame_length < frame->step * frame->height) {
            s_dropped_frames++;
            return;
        }
        frame_length = frame->step * frame->height;
    } else {
        s_dropped_frames++;
        return;
    }
    if (frame_length > MJPEG_SLOT_CAPACITY) {
        s_dropped_frames++;
        return;
    }

    int slot_index = reserve_mjpeg_slot();
    if (slot_index < 0) {
        /* The decoder owns one slot and the queue owns the other. Replace the
         * queued frame so control never waits behind stale camera data. */
        int stale_slot;
        if (xQueueReceive(s_frame_queue, &stale_slot, 0) == pdPASS) {
            release_mjpeg_slot(stale_slot);
            s_dropped_frames++;
            slot_index = reserve_mjpeg_slot();
        }
    }
    if (slot_index < 0) {
        s_dropped_frames++;
        return;
    }

    memcpy(s_slots[slot_index].data, frame->data, frame_length);
    s_slots[slot_index].length = frame_length;
    s_slots[slot_index].step = frame->step;
    s_slots[slot_index].width = frame->width;
    s_slots[slot_index].height = frame->height;
    s_slots[slot_index].format = frame->frame_format;
    s_slots[slot_index].captured_at_us = esp_timer_get_time();
    if (xQueueSend(s_frame_queue, &slot_index, 0) != pdPASS) {
        release_mjpeg_slot(slot_index);
        s_dropped_frames++;
    }
}

static void frame_display_task(void *argument)
{
    (void)argument;
    ESP_LOGI(TAG, "DECODE_VISION_TASK core=%d", xPortGetCoreID());
    int slot_index;
    uint32_t processed_frames = 0;
    uint32_t last_received_report = 0;
    uint32_t last_processed_report = 0;
    uint32_t last_displayed_report = 0;
    uint64_t decode_time_us = 0;
    uint64_t vision_time_us = 0;
    uint32_t timed_frames = 0;
    int64_t last_report_us = esp_timer_get_time();
    int64_t last_rgb_debug_us = 0;
    int64_t last_tuner_us = 0;
    int64_t last_calibration_us = 0;
    int64_t last_tft_preview_us = 0;
    int64_t latest_frame_age_ms = 0;

    while (true) {
        if (xQueueReceive(s_frame_queue, &slot_index, portMAX_DELAY) != pdPASS) {
            continue;
        }
        const int64_t captured_at_us = s_slots[slot_index].captured_at_us;

        if (camera_line_follow_calibration_enabled()) {
            const int64_t calibration_now_us = esp_timer_get_time();
            if (s_slots[slot_index].format == UVC_FRAME_FORMAT_MJPEG &&
                calibration_now_us - last_calibration_us >=
                    CALIBRATION_INTERVAL_US) {
                emit_calibration_jpeg(&s_slots[slot_index]);
                last_calibration_us = esp_timer_get_time();
            }
            release_mjpeg_slot(slot_index);
            vTaskDelay(1);
            continue;
        }

        const int64_t decode_start_us = esp_timer_get_time();
        esp_jpeg_image_output_t output = {0};
        esp_err_t decode_error;
        if (s_slots[slot_index].format == UVC_FRAME_FORMAT_YUYV) {
            decode_error = downsample_yuyv_luma(&s_slots[slot_index]);
            output.width = DECODED_WIDTH;
            output.height = DECODED_HEIGHT;
        } else {
            esp_jpeg_image_cfg_t jpeg_config = {
                .indata = s_slots[slot_index].data,
                .indata_size = s_slots[slot_index].length,
                .outbuf = s_decoded_frame,
                .outbuf_size = DECODED_BUFFER_BYTES,
                .out_format = JPEG_IMAGE_FORMAT_RGB565,
                .out_scale = s_decode_scale,
                .flags = {
                    .swap_color_bytes = 1,
                },
                .advanced = {
                    .working_buffer = s_jpeg_work_buffer,
                    .working_buffer_size = JPEG_WORK_BUFFER_BYTES,
                },
            };
            decode_error = esp_jpeg_decode(&jpeg_config, &output);
        }
        if (decode_error == ESP_OK && output.width == DECODED_WIDTH &&
            output.height == DECODED_HEIGHT) {
            const int64_t decode_done_us = esp_timer_get_time();
            const bool rgb_debug = camera_line_follow_debug_enabled();
            const bool tuner = camera_line_follow_tuner_enabled();
            if (rgb_debug || tuner) {
                memcpy(s_debug_raw_frame, s_decoded_frame,
                       DECODED_BUFFER_BYTES);
            }
            ball_vision_result_t ball_result;
            ball_vision_result_t white_ball_result;
            black_marker_result_t marker_result;
            ball_vision_process(s_decoded_frame, output.width, output.height,
                                &ball_result);
            white_ball_vision_process(s_decoded_frame, output.width,
                                      output.height, &white_ball_result);
            black_marker_vision_process(s_decoded_frame, output.width,
                                        output.height, &marker_result);
            ball_vision_draw_overlay(s_decoded_frame, output.width,
                                     output.height, &ball_result);
            ball_vision_draw_overlay_color(s_decoded_frame, output.width,
                                           output.height, &white_ball_result,
                                           0x07ff);
            black_marker_vision_draw_overlay(s_decoded_frame, output.width,
                                             output.height, &marker_result);
            const int64_t vision_done_us = esp_timer_get_time();
            decode_time_us += (uint64_t)(decode_done_us - decode_start_us);
            vision_time_us += (uint64_t)(vision_done_us - decode_done_us);
            timed_frames++;
            /* This firmware image is deliberately a visual ball-tracking
             * test.  Do not run or submit line-following control here: the
             * chassis remains in the safe-stop state while the TFT shows the
             * adaptive red/white ball and black-goal boxes. */
            processed_frames++;
            const int64_t debug_now_us = esp_timer_get_time();
            if (!tuner && debug_now_us - last_tft_preview_us >=
                    TFT_PREVIEW_INTERVAL_US) {
                if (queue_tft_preview(s_decoded_frame)) {
                    last_tft_preview_us = debug_now_us;
                }
            }
            if (tuner &&
                debug_now_us - last_tuner_us >= TUNER_INTERVAL_US) {
                const line_vision_result_t empty_result = {0};
                emit_tuner_frame(s_debug_raw_frame, output.width,
                                 output.height, &empty_result);
                last_tuner_us = debug_now_us;
            } else if (rgb_debug &&
                debug_now_us - last_rgb_debug_us >= RGB_DEBUG_INTERVAL_US) {
                const line_vision_result_t empty_result = {0};
                emit_rgb_debug_frame(s_debug_raw_frame, output.width,
                                     output.height, &empty_result);
                last_rgb_debug_us = debug_now_us;
            }
        } else {
            ESP_LOGW(TAG, "Frame conversion failed: %s, output=%ux%u",
                     esp_err_to_name(decode_error), output.width, output.height);
        }
        release_mjpeg_slot(slot_index);

        const int64_t now_us = esp_timer_get_time();
        latest_frame_age_ms = (now_us - captured_at_us) / 1000;
        if (now_us - last_report_us >= 3000000) {
            const uint32_t received_now = s_received_frames;
            const uint32_t displayed_now = s_displayed_frames;
            const uint32_t dropped_now = s_dropped_frames;
            const uint64_t elapsed_us = (uint64_t)(now_us - last_report_us);
            const uint32_t rx_fps_x10 = (uint32_t)(
                (uint64_t)(received_now - last_received_report) *
                10000000ULL / elapsed_us);
            const uint32_t processed_fps_x10 = (uint32_t)(
                (uint64_t)(processed_frames - last_processed_report) *
                10000000ULL / elapsed_us);
            const uint32_t lcd_fps_x10 = (uint32_t)(
                (uint64_t)(displayed_now - last_displayed_report) *
                10000000ULL / elapsed_us);
            const uint32_t decode_average_us = timed_frames > 0
                                                   ? decode_time_us / timed_frames
                                                   : 0;
            const uint32_t vision_average_us = timed_frames > 0
                                                   ? vision_time_us / timed_frames
                                                   : 0;
            ESP_LOGI(TAG,
                     "VIDEO fps=%lu.%lu/%lu.%lu/%lu.%lu drop=%lu age=%lldms cost=%lu/%luus",
                     (unsigned long)(rx_fps_x10 / 10),
                     (unsigned long)(rx_fps_x10 % 10),
                     (unsigned long)(processed_fps_x10 / 10),
                     (unsigned long)(processed_fps_x10 % 10),
                     (unsigned long)(lcd_fps_x10 / 10),
                     (unsigned long)(lcd_fps_x10 % 10),
                     (unsigned long)dropped_now,
                     (long long)latest_frame_age_ms,
                     (unsigned long)decode_average_us,
                     (unsigned long)vision_average_us);
            last_received_report = received_now;
            last_processed_report = processed_frames;
            last_displayed_report = displayed_now;
            decode_time_us = 0;
            vision_time_us = 0;
            timed_frames = 0;
            last_report_us = now_us;
        }
        /* 160x120 JPEG decoding can keep CPU1 busy frame after frame.  Yield
         * once per frame so IDLE1 can reset the task watchdog during a long
         * ball-tracking test. */
        vTaskDelay(1);
    }
}

static esp_err_t downsample_yuyv_luma(const mjpeg_slot_t *slot)
{
    if (slot->format != UVC_FRAME_FORMAT_YUYV || slot->step == 0 ||
        slot->width < DECODED_WIDTH || slot->height < DECODED_HEIGHT) {
        return ESP_ERR_INVALID_ARG;
    }
    for (size_t y = 0; y < DECODED_HEIGHT; ++y) {
        const size_t source_y = y * slot->height / DECODED_HEIGHT;
        const uint8_t *source_row = slot->data + source_y * slot->step;
        for (size_t x = 0; x < DECODED_WIDTH; ++x) {
            const size_t source_x = x * slot->width / DECODED_WIDTH;
            const uint8_t luminance = source_row[source_x * 2];
            const uint16_t gray = ((uint16_t)(luminance >> 3) << 11) |
                                  ((uint16_t)(luminance >> 2) << 5) |
                                  (luminance >> 3);
            const size_t destination = 2 * (y * DECODED_WIDTH + x);
            s_decoded_frame[destination] = gray >> 8;
            s_decoded_frame[destination + 1] = gray & 0xff;
        }
    }
    return ESP_OK;
}

static esp_err_t initialize_frame_pipeline(void)
{
    s_frame_queue = xQueueCreate(FRAME_QUEUE_LENGTH, sizeof(int));
    s_display_queue = xQueueCreate(DISPLAY_QUEUE_LENGTH, sizeof(uint8_t));
    if (s_frame_queue == NULL || s_display_queue == NULL) {
        return ESP_ERR_NO_MEM;
    }

    for (int index = 0; index < MJPEG_SLOT_COUNT; ++index) {
        s_slots[index].data = heap_caps_malloc(MJPEG_SLOT_CAPACITY,
                                               MALLOC_CAP_SPIRAM |
                                               MALLOC_CAP_8BIT);
        if (s_slots[index].data == NULL) {
            ESP_LOGE(TAG, "Cannot allocate MJPEG slot %d", index);
            return ESP_ERR_NO_MEM;
        }
    }
    s_decoded_frame = heap_caps_malloc(DECODED_BUFFER_BYTES,
                                       MALLOC_CAP_INTERNAL |
                                       MALLOC_CAP_8BIT);
    if (s_decoded_frame == NULL) {
        ESP_LOGW(TAG, "Internal decode buffer unavailable; using PSRAM");
        s_decoded_frame = heap_caps_malloc(DECODED_BUFFER_BYTES,
                                           MALLOC_CAP_SPIRAM |
                                           MALLOC_CAP_8BIT);
    } else {
        ESP_LOGI(TAG, "FAST_DECODE_BUFFER=INTERNAL bytes=%d",
                 DECODED_BUFFER_BYTES);
    }
    s_display_frame = heap_caps_malloc(DECODED_BUFFER_BYTES,
                                       MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (s_decoded_frame == NULL || s_display_frame == NULL) {
        return ESP_ERR_NO_MEM;
    }
    s_debug_raw_frame = heap_caps_malloc(DECODED_BUFFER_BYTES,
                                         MALLOC_CAP_SPIRAM |
                                         MALLOC_CAP_8BIT);
    if (s_debug_raw_frame == NULL) {
        return ESP_ERR_NO_MEM;
    }
    s_tuner_mask = heap_caps_malloc(TUNER_MASK_BYTES,
                                    MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (s_tuner_mask == NULL) {
        return ESP_ERR_NO_MEM;
    }
    s_jpeg_work_buffer = heap_caps_malloc(JPEG_WORK_BUFFER_BYTES,
                                          MALLOC_CAP_INTERNAL |
                                          MALLOC_CAP_8BIT);
    if (s_jpeg_work_buffer == NULL) {
        return ESP_ERR_NO_MEM;
    }

    if (xTaskCreatePinnedToCore(frame_display_task, "camera_display", 8192,
                                NULL, 4, NULL, 1) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    if (xTaskCreatePinnedToCore(tft_display_task, "tft_preview", 4096,
                                NULL, 3, NULL, 0) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG,
             "PIPELINE decode/path-pursuit=core1 async-TFT=core0 frame=%dx%d queue=latest",
             DECODED_WIDTH, DECODED_HEIGHT);
    return ESP_OK;
}

static uvc_error_t negotiate_mjpeg_stream(uvc_device_handle_t *device_handle,
                                          uvc_stream_ctrl_t *control)
{
    typedef struct {
        enum uvc_frame_format format;
        int width;
        int height;
        int fps;
        esp_jpeg_image_scale_t scale;
    } camera_profile_t;
    static const camera_profile_t profiles[] = {
        {UVC_FRAME_FORMAT_MJPEG, 640, 480, 15, JPEG_IMAGE_SCALE_1_4},
        {UVC_FRAME_FORMAT_MJPEG, 640, 480, 30, JPEG_IMAGE_SCALE_1_4},
        {UVC_FRAME_FORMAT_MJPEG, 320, 240, 30, JPEG_IMAGE_SCALE_1_2},
        {UVC_FRAME_FORMAT_MJPEG, 320, 240, 15, JPEG_IMAGE_SCALE_1_2},
        {UVC_FRAME_FORMAT_YUYV, 160, 120, 30, JPEG_IMAGE_SCALE_0},
        {UVC_FRAME_FORMAT_YUYV, 160, 120, 15, JPEG_IMAGE_SCALE_0},
    };
    uvc_error_t result = UVC_ERROR_INVALID_MODE;
    for (size_t profile = 0;
         profile < sizeof(profiles) / sizeof(profiles[0]); ++profile) {
        for (int attempt = 1; attempt <= 2; ++attempt) {
            result = uvc_get_stream_ctrl_format_size(device_handle, control,
                                                      profiles[profile].format,
                                                      profiles[profile].width,
                                                      profiles[profile].height,
                                                      profiles[profile].fps);
            if (result == UVC_SUCCESS) {
                s_stream_width = profiles[profile].width;
                s_stream_height = profiles[profile].height;
                s_stream_fps = profiles[profile].fps;
                s_stream_format = profiles[profile].format;
                s_decode_scale = profiles[profile].scale;
                control->dwMaxPayloadTransferSize =
                    profiles[profile].format == UVC_FRAME_FORMAT_YUYV
                        ? 1023 : 512;
                return UVC_SUCCESS;
            }
        }
    }
    return result;
}

void app_main(void)
{
    ESP_LOGI(TAG,
             "USB camera ball tracker; 160x120 adaptive box on TFT preview");
    ESP_LOGI(TAG, "UART0: TX=GPIO43 RX=GPIO44 baud=115200");
    ESP_LOGI(TAG, "Camera: D-=GPIO19 D+=GPIO20; motors start in SAFE STOP");

    const size_t psram_size = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
    ESP_LOGI(TAG, "PSRAM available: %u bytes", (unsigned)psram_size);
    if (psram_size == 0) {
        ESP_LOGE(TAG, "CAMERA_STATUS=NO_PSRAM");
        return;
    }

    ESP_ERROR_CHECK(camera_line_follow_init());
    ESP_ERROR_CHECK(ball_vision_init(DECODED_WIDTH, DECODED_HEIGHT));
    ESP_ERROR_CHECK(white_ball_vision_init(DECODED_WIDTH, DECODED_HEIGHT));
    ESP_ERROR_CHECK(black_marker_vision_init(DECODED_WIDTH, DECODED_HEIGHT));
    ESP_ERROR_CHECK(camera_display_init());
    ESP_ERROR_CHECK(camera_display_show_waiting());
    ESP_ERROR_CHECK(initialize_frame_pipeline());

    s_uvc_events = xEventGroupCreate();
    ESP_ERROR_CHECK(s_uvc_events == NULL ? ESP_ERR_NO_MEM : ESP_OK);
    ESP_ERROR_CHECK(initialize_usb_host());

    libuvc_adapter_config_t adapter_config = {
        .create_background_task = true,
        .task_priority = 5,
        .stack_size = 4096,
        .callback = libuvc_event_callback,
    };
    libuvc_adapter_set_config(&adapter_config);

    uvc_context_t *context = NULL;
    const uvc_error_t init_result = uvc_init(&context, NULL);
    if (init_result != UVC_SUCCESS) {
        ESP_LOGE(TAG, "uvc_init failed: %s", uvc_error_string(init_result));
        return;
    }

    while (true) {
        ESP_LOGI(TAG, "CAMERA_STATUS=WAITING_FOR_USB_CAMERA");
        wait_for_uvc_event(UVC_DEVICE_CONNECTED);

        uvc_device_t *device = NULL;
        uvc_device_handle_t *device_handle = NULL;
        uvc_stream_ctrl_t stream_control = {0};

        uvc_error_t result = uvc_find_device(context, &device, 0, 0, NULL);
        if (result != UVC_SUCCESS) {
            ESP_LOGW(TAG, "Camera find failed: %s", uvc_error_string(result));
            continue;
        }
        result = uvc_open(device, &device_handle);
        if (result != UVC_SUCCESS) {
            ESP_LOGW(TAG, "Camera open failed: %s", uvc_error_string(result));
            uvc_unref_device(device);
            continue;
        }

        ESP_LOGI(TAG, "CAMERA_STATUS=UVC_OPEN");
        result = negotiate_mjpeg_stream(device_handle, &stream_control);
        if (result == UVC_SUCCESS) {
            result = uvc_start_streaming(device_handle, &stream_control,
                                         camera_frame_callback, NULL, 0);
        }

        if (result == UVC_SUCCESS) {
            ESP_LOGI(TAG, "CAMERA_STATUS=STREAMING_%dx%d_%dFPS_%s",
                     s_stream_width, s_stream_height, s_stream_fps,
                     s_stream_format == UVC_FRAME_FORMAT_YUYV
                         ? "YUYV" : "MJPEG");
            wait_for_uvc_event(UVC_DEVICE_DISCONNECTED);
            camera_line_follow_camera_disconnected();
            uvc_stop_streaming(device_handle);
        } else {
            ESP_LOGE(TAG, "CAMERA_STATUS=STREAM_FAILED error=%s",
                     uvc_error_string(result));
            wait_for_uvc_event(UVC_DEVICE_DISCONNECTED);
            camera_line_follow_camera_disconnected();
        }

        camera_display_show_waiting();
        uvc_close(device_handle);
        uvc_unref_device(device);
        ESP_LOGW(TAG, "CAMERA_STATUS=DISCONNECTED");
    }
}
