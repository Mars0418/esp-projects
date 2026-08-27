#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "camera_display.h"
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
#include "usb/usb_host.h"

#define CAMERA_WIDTH          640
#define CAMERA_HEIGHT         480
#define CAMERA_FPS            15
#define MJPEG_SLOT_COUNT      2
#define MJPEG_SLOT_CAPACITY   (256 * 1024)
#define DECODED_WIDTH         (CAMERA_WIDTH / 4)
#define DECODED_HEIGHT        (CAMERA_HEIGHT / 4)
#define DECODED_BUFFER_BYTES  (DECODED_WIDTH * DECODED_HEIGHT * 2)
#define JPEG_WORK_BUFFER_BYTES 8192

typedef struct {
    uint8_t *data;
    size_t length;
    bool busy;
} mjpeg_slot_t;

static const char *TAG = "CAMERA_VIEW";
static EventGroupHandle_t s_uvc_events;
static QueueHandle_t s_frame_queue;
static mjpeg_slot_t s_slots[MJPEG_SLOT_COUNT];
static uint8_t *s_decoded_frame;
static uint8_t *s_jpeg_work_buffer;
static portMUX_TYPE s_slot_lock = portMUX_INITIALIZER_UNLOCKED;
static volatile uint32_t s_received_frames;
static volatile uint32_t s_displayed_frames;
static volatile uint32_t s_dropped_frames;

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
    s_received_frames++;

    const uint8_t *frame_bytes = frame->data;
    size_t jpeg_length = frame->data_bytes;
    while (jpeg_length >= 2 &&
           !(frame_bytes[jpeg_length - 2] == 0xff &&
             frame_bytes[jpeg_length - 1] == 0xd9)) {
        jpeg_length--;
    }

    if (frame->frame_format != UVC_FRAME_FORMAT_MJPEG || jpeg_length < 4 ||
        jpeg_length > MJPEG_SLOT_CAPACITY || frame_bytes[0] != 0xff ||
        frame_bytes[1] != 0xd8) {
        s_dropped_frames++;
        return;
    }

    const int slot_index = reserve_mjpeg_slot();
    if (slot_index < 0) {
        s_dropped_frames++;
        return;
    }

    memcpy(s_slots[slot_index].data, frame->data, jpeg_length);
    s_slots[slot_index].length = jpeg_length;
    if (xQueueSend(s_frame_queue, &slot_index, 0) != pdPASS) {
        release_mjpeg_slot(slot_index);
        s_dropped_frames++;
    }
}

static void frame_display_task(void *argument)
{
    (void)argument;
    int slot_index;
    int64_t last_report_us = esp_timer_get_time();

    while (true) {
        if (xQueueReceive(s_frame_queue, &slot_index, portMAX_DELAY) != pdPASS) {
            continue;
        }

        esp_jpeg_image_cfg_t jpeg_config = {
            .indata = s_slots[slot_index].data,
            .indata_size = s_slots[slot_index].length,
            .outbuf = s_decoded_frame,
            .outbuf_size = DECODED_BUFFER_BYTES,
            .out_format = JPEG_IMAGE_FORMAT_RGB565,
            .out_scale = JPEG_IMAGE_SCALE_1_4,
            .flags = {
                .swap_color_bytes = 1,
            },
            .advanced = {
                .working_buffer = s_jpeg_work_buffer,
                .working_buffer_size = JPEG_WORK_BUFFER_BYTES,
            },
        };
        esp_jpeg_image_output_t output = {0};
        const esp_err_t decode_error = esp_jpeg_decode(&jpeg_config, &output);
        if (decode_error == ESP_OK && output.width == DECODED_WIDTH &&
            output.height == DECODED_HEIGHT) {
            const esp_err_t display_error =
                camera_display_show_rotated_rgb565(s_decoded_frame,
                                                    output.width,
                                                    output.height);
            if (display_error == ESP_OK) {
                s_displayed_frames++;
            } else {
                ESP_LOGE(TAG, "TFT frame failed: %s",
                         esp_err_to_name(display_error));
            }
        } else {
            ESP_LOGW(TAG, "JPEG decode failed: %s, output=%ux%u",
                     esp_err_to_name(decode_error), output.width, output.height);
        }
        release_mjpeg_slot(slot_index);

        const int64_t now_us = esp_timer_get_time();
        if (now_us - last_report_us >= 1000000) {
            ESP_LOGI(TAG, "FRAME_STATUS received=%lu displayed=%lu dropped=%lu",
                     (unsigned long)s_received_frames,
                     (unsigned long)s_displayed_frames,
                     (unsigned long)s_dropped_frames);
            last_report_us = now_us;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static esp_err_t initialize_frame_pipeline(void)
{
    s_frame_queue = xQueueCreate(MJPEG_SLOT_COUNT, sizeof(int));
    if (s_frame_queue == NULL) {
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
                                       MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (s_decoded_frame == NULL) {
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
    return ESP_OK;
}

static uvc_error_t negotiate_mjpeg_stream(uvc_device_handle_t *device_handle,
                                          uvc_stream_ctrl_t *control)
{
    static const int frame_rates[] = {CAMERA_FPS, 30, 0};
    uvc_error_t result = UVC_ERROR_INVALID_MODE;
    for (size_t profile = 0;
         profile < sizeof(frame_rates) / sizeof(frame_rates[0]); ++profile) {
        for (int attempt = 1; attempt <= 3; ++attempt) {
            ESP_LOGI(TAG, "Trying MJPEG %dx%d at %d fps (attempt %d)",
                     CAMERA_WIDTH, CAMERA_HEIGHT, frame_rates[profile], attempt);
            result = uvc_get_stream_ctrl_format_size(device_handle, control,
                                                      UVC_FRAME_FORMAT_MJPEG,
                                                      CAMERA_WIDTH,
                                                      CAMERA_HEIGHT,
                                                      frame_rates[profile]);
            if (result == UVC_SUCCESS) {
                control->dwMaxPayloadTransferSize = 512;
                return UVC_SUCCESS;
            }
        }
    }
    return result;
}

void app_main(void)
{
    ESP_LOGI(TAG, "USB camera to TFT test");
    ESP_LOGI(TAG, "UART0: TX=GPIO43 RX=GPIO44 baud=115200");
    ESP_LOGI(TAG, "Camera: D-=GPIO19 D+=GPIO20; motor outputs remain disabled");

    const size_t psram_size = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
    ESP_LOGI(TAG, "PSRAM available: %u bytes", (unsigned)psram_size);
    if (psram_size == 0) {
        ESP_LOGE(TAG, "CAMERA_STATUS=NO_PSRAM");
        return;
    }

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
            uvc_print_stream_ctrl(&stream_control, stderr);
            result = uvc_start_streaming(device_handle, &stream_control,
                                         camera_frame_callback, NULL, 0);
        }

        if (result == UVC_SUCCESS) {
            ESP_LOGI(TAG, "CAMERA_STATUS=STREAMING_640X480_MJPEG");
            wait_for_uvc_event(UVC_DEVICE_DISCONNECTED);
            uvc_stop_streaming(device_handle);
        } else {
            ESP_LOGE(TAG, "CAMERA_STATUS=STREAM_FAILED error=%s",
                     uvc_error_string(result));
            wait_for_uvc_event(UVC_DEVICE_DISCONNECTED);
        }

        camera_display_show_waiting();
        uvc_close(device_handle);
        uvc_unref_device(device);
        ESP_LOGW(TAG, "CAMERA_STATUS=DISCONNECTED");
    }
}
