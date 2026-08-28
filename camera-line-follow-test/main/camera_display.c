#include "camera_display.h"

#include <stdbool.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TFT_HOST       SPI2_HOST
#define TFT_PIN_SCLK   GPIO_NUM_3
#define TFT_PIN_MOSI   GPIO_NUM_4
#define TFT_PIN_CS     GPIO_NUM_0
#define TFT_PIN_DC     GPIO_NUM_38
#define TFT_WIDTH      128
#define TFT_HEIGHT     160
#define TFT_SPI_HZ     (20 * 1000 * 1000)
#define TFT_SPI_CHUNK  4096

static const char *TAG = "CAMERA_TFT";
static spi_device_handle_t s_tft;
static uint8_t *s_framebuffer;

static esp_err_t tft_write(bool data_mode, const void *data, size_t length)
{
    if (length == 0) {
        return ESP_OK;
    }
    ESP_RETURN_ON_ERROR(gpio_set_level(TFT_PIN_DC, data_mode), TAG,
                        "set D/C failed");
    spi_transaction_t transaction = {
        .length = length * 8,
        .tx_buffer = data,
    };
    return spi_device_polling_transmit(s_tft, &transaction);
}

static esp_err_t tft_command(uint8_t command)
{
    return tft_write(false, &command, 1);
}

static esp_err_t tft_command_data(uint8_t command, const uint8_t *data,
                                  size_t length)
{
    ESP_RETURN_ON_ERROR(tft_command(command), TAG, "command failed");
    return tft_write(true, data, length);
}

static esp_err_t tft_pixel_data(const void *data, size_t length)
{
    ESP_RETURN_ON_ERROR(gpio_set_level(TFT_PIN_DC, 1), TAG,
                        "set pixel D/C failed");
    ESP_RETURN_ON_ERROR(spi_device_acquire_bus(s_tft, portMAX_DELAY), TAG,
                        "acquire SPI bus failed");

    const uint8_t *bytes = data;
    size_t offset = 0;
    esp_err_t result = ESP_OK;
    while (offset < length) {
        size_t chunk = length - offset;
        if (chunk > TFT_SPI_CHUNK) {
            chunk = TFT_SPI_CHUNK;
        }
        spi_transaction_t transaction = {
            .flags = offset + chunk < length ? SPI_TRANS_CS_KEEP_ACTIVE : 0,
            .length = chunk * 8,
            .tx_buffer = bytes + offset,
        };
        result = spi_device_polling_transmit(s_tft, &transaction);
        if (result != ESP_OK) {
            break;
        }
        offset += chunk;
    }
    spi_device_release_bus(s_tft);
    return result;
}

static esp_err_t tft_set_full_window(void)
{
    static const uint8_t columns[] = {0x00, 0x00, 0x00, TFT_WIDTH - 1};
    static const uint8_t rows[] = {0x00, 0x00, 0x00, TFT_HEIGHT - 1};
    ESP_RETURN_ON_ERROR(tft_command_data(0x2a, columns, sizeof(columns)),
                        TAG, "set columns failed");
    ESP_RETURN_ON_ERROR(tft_command_data(0x2b, rows, sizeof(rows)),
                        TAG, "set rows failed");
    return tft_command(0x2c);
}

static esp_err_t tft_send_frame(void)
{
    ESP_RETURN_ON_ERROR(tft_set_full_window(), TAG, "set frame window failed");
    return tft_pixel_data(s_framebuffer, TFT_WIDTH * TFT_HEIGHT * 2);
}

static void set_pixel(size_t x, size_t y, uint16_t color)
{
    const size_t offset = 2 * (y * TFT_WIDTH + x);
    s_framebuffer[offset] = color >> 8;
    s_framebuffer[offset + 1] = color & 0xff;
}

esp_err_t camera_display_init(void)
{
    const spi_bus_config_t bus_config = {
        .mosi_io_num = TFT_PIN_MOSI,
        .miso_io_num = GPIO_NUM_NC,
        .sclk_io_num = TFT_PIN_SCLK,
        .quadwp_io_num = GPIO_NUM_NC,
        .quadhd_io_num = GPIO_NUM_NC,
        .max_transfer_sz = TFT_SPI_CHUNK,
    };
    ESP_RETURN_ON_ERROR(spi_bus_initialize(TFT_HOST, &bus_config,
                                            SPI_DMA_CH_AUTO),
                        TAG, "SPI bus init failed");

    const spi_device_interface_config_t device_config = {
        .clock_speed_hz = TFT_SPI_HZ,
        .mode = 0,
        .spics_io_num = TFT_PIN_CS,
        .queue_size = 4,
    };
    ESP_RETURN_ON_ERROR(spi_bus_add_device(TFT_HOST, &device_config, &s_tft),
                        TAG, "add TFT failed");

    s_framebuffer = heap_caps_calloc(TFT_WIDTH * TFT_HEIGHT, 2,
                                     MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    ESP_RETURN_ON_FALSE(s_framebuffer != NULL, ESP_ERR_NO_MEM, TAG,
                        "framebuffer allocation failed");

    const gpio_config_t dc_config = {
        .pin_bit_mask = 1ULL << TFT_PIN_DC,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&dc_config), TAG, "D/C init failed");

    ESP_RETURN_ON_ERROR(tft_command(0x01), TAG, "software reset failed");
    vTaskDelay(pdMS_TO_TICKS(150));
    ESP_RETURN_ON_ERROR(tft_command(0x11), TAG, "sleep out failed");
    vTaskDelay(pdMS_TO_TICKS(120));

    static const uint8_t frame_rate[] = {0x01, 0x2c, 0x2d};
    static const uint8_t frame_rate_partial[] = {
        0x01, 0x2c, 0x2d, 0x01, 0x2c, 0x2d,
    };
    static const uint8_t inversion_control[] = {0x07};
    static const uint8_t power_control_1[] = {0xa2, 0x02, 0x84};
    static const uint8_t power_control_2[] = {0xc5};
    static const uint8_t power_control_3[] = {0x0a, 0x00};
    static const uint8_t power_control_4[] = {0x8a, 0x2a};
    static const uint8_t power_control_5[] = {0x8a, 0xee};
    static const uint8_t vcom_control[] = {0x0e};
    static const uint8_t memory_access[] = {0xc8};
    static const uint8_t pixel_format[] = {0x05};
    static const uint8_t positive_gamma[] = {
        0x02, 0x1c, 0x07, 0x12, 0x37, 0x32, 0x29, 0x2d,
        0x29, 0x25, 0x2b, 0x39, 0x00, 0x01, 0x03, 0x10,
    };
    static const uint8_t negative_gamma[] = {
        0x03, 0x1d, 0x07, 0x06, 0x2e, 0x2c, 0x29, 0x2d,
        0x2e, 0x2e, 0x37, 0x3f, 0x00, 0x00, 0x02, 0x10,
    };

    ESP_RETURN_ON_ERROR(tft_command_data(0xb1, frame_rate, 3), TAG, "B1 failed");
    ESP_RETURN_ON_ERROR(tft_command_data(0xb2, frame_rate, 3), TAG, "B2 failed");
    ESP_RETURN_ON_ERROR(tft_command_data(0xb3, frame_rate_partial, 6), TAG, "B3 failed");
    ESP_RETURN_ON_ERROR(tft_command_data(0xb4, inversion_control, 1), TAG, "B4 failed");
    ESP_RETURN_ON_ERROR(tft_command_data(0xc0, power_control_1, 3), TAG, "C0 failed");
    ESP_RETURN_ON_ERROR(tft_command_data(0xc1, power_control_2, 1), TAG, "C1 failed");
    ESP_RETURN_ON_ERROR(tft_command_data(0xc2, power_control_3, 2), TAG, "C2 failed");
    ESP_RETURN_ON_ERROR(tft_command_data(0xc3, power_control_4, 2), TAG, "C3 failed");
    ESP_RETURN_ON_ERROR(tft_command_data(0xc4, power_control_5, 2), TAG, "C4 failed");
    ESP_RETURN_ON_ERROR(tft_command_data(0xc5, vcom_control, 1), TAG, "C5 failed");
    ESP_RETURN_ON_ERROR(tft_command_data(0x36, memory_access, 1), TAG, "MADCTL failed");
    ESP_RETURN_ON_ERROR(tft_command_data(0x3a, pixel_format, 1), TAG, "COLMOD failed");
    ESP_RETURN_ON_ERROR(tft_command_data(0xe0, positive_gamma, 16), TAG, "E0 failed");
    ESP_RETURN_ON_ERROR(tft_command_data(0xe1, negative_gamma, 16), TAG, "E1 failed");
    ESP_RETURN_ON_ERROR(tft_command(0x13), TAG, "normal mode failed");
    ESP_RETURN_ON_ERROR(tft_command(0x20), TAG, "disable inversion failed");
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_RETURN_ON_ERROR(tft_command(0x29), TAG, "display on failed");
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(TAG,
             "TFT ready: 128x160, SPI=20MHz, inversion=off, "
             "SCK=GPIO3 MOSI=GPIO4 CS=GPIO0 DC=GPIO38");
    return ESP_OK;
}

esp_err_t camera_display_show_waiting(void)
{
    memset(s_framebuffer, 0, TFT_WIDTH * TFT_HEIGHT * 2);
    for (size_t y = 42; y < 118; ++y) {
        for (size_t x = 18; x < 110; ++x) {
            const bool border = x < 22 || x >= 106 || y < 46 || y >= 114;
            set_pixel(x, y, border ? 0x07ff : 0x0010);
        }
    }
    return tft_send_frame();
}

esp_err_t camera_display_show_rotated_rgb565(const uint8_t *pixels,
                                              size_t width,
                                              size_t height)
{
    ESP_RETURN_ON_FALSE(pixels != NULL && width <= TFT_HEIGHT &&
                        height <= TFT_WIDTH, ESP_ERR_INVALID_ARG, TAG,
                        "invalid camera frame dimensions");

    const size_t x_offset = (TFT_WIDTH - height) / 2;
    const size_t y_offset = (TFT_HEIGHT - width) / 2;
    for (size_t source_y = 0; source_y < height; ++source_y) {
        for (size_t source_x = 0; source_x < width; ++source_x) {
            const size_t destination_x = x_offset + height - 1 - source_y;
            const size_t destination_y = y_offset + source_x;
            const size_t source_offset = 2 * (source_y * width + source_x);
            const size_t destination_offset =
                2 * (destination_y * TFT_WIDTH + destination_x);
            s_framebuffer[destination_offset] = pixels[source_offset];
            s_framebuffer[destination_offset + 1] = pixels[source_offset + 1];
        }
    }
    return tft_send_frame();
}
