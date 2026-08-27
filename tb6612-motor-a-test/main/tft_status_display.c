#include "tft_status_display.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

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
#define TFT_SPI_HZ     (2 * 1000 * 1000)
#define TFT_SPI_CHUNK  4096
#define TFT_REFRESH_MS 500

typedef struct {
    int rpm_a;
    int rpm_b;
    int rpm_d;
    int distance_mm;
} display_values_t;

typedef struct {
    char character;
    uint8_t rows[7];
} glyph_t;

static const char *TAG = "tft_status";
static spi_device_handle_t s_tft;
static uint8_t *s_pixels;
static portMUX_TYPE s_values_lock = portMUX_INITIALIZER_UNLOCKED;
static display_values_t s_values = {.distance_mm = -1};

static const glyph_t s_font[] = {
    {' ', {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
    {'-', {0x00, 0x00, 0x00, 0x1f, 0x00, 0x00, 0x00}},
    {'0', {0x0e, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0e}},
    {'1', {0x04, 0x0c, 0x04, 0x04, 0x04, 0x04, 0x0e}},
    {'2', {0x0e, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1f}},
    {'3', {0x1e, 0x01, 0x01, 0x0e, 0x01, 0x01, 0x1e}},
    {'4', {0x02, 0x06, 0x0a, 0x12, 0x1f, 0x02, 0x02}},
    {'5', {0x1f, 0x10, 0x10, 0x1e, 0x01, 0x01, 0x1e}},
    {'6', {0x0e, 0x10, 0x10, 0x1e, 0x11, 0x11, 0x0e}},
    {'7', {0x1f, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08}},
    {'8', {0x0e, 0x11, 0x11, 0x0e, 0x11, 0x11, 0x0e}},
    {'9', {0x0e, 0x11, 0x11, 0x0f, 0x01, 0x01, 0x0e}},
    {'A', {0x0e, 0x11, 0x11, 0x1f, 0x11, 0x11, 0x11}},
    {'B', {0x1e, 0x11, 0x11, 0x1e, 0x11, 0x11, 0x1e}},
    {'D', {0x1e, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1e}},
    {'I', {0x1f, 0x04, 0x04, 0x04, 0x04, 0x04, 0x1f}},
    {'M', {0x11, 0x1b, 0x15, 0x15, 0x11, 0x11, 0x11}},
    {'O', {0x0e, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0e}},
    {'P', {0x1e, 0x11, 0x11, 0x1e, 0x10, 0x10, 0x10}},
    {'R', {0x1e, 0x11, 0x11, 0x1e, 0x14, 0x12, 0x11}},
    {'S', {0x0f, 0x10, 0x10, 0x0e, 0x01, 0x01, 0x1e}},
    {'T', {0x1f, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04}},
};

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

static esp_err_t tft_init(void)
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

    s_pixels = heap_caps_malloc(TFT_WIDTH * TFT_HEIGHT * 2, MALLOC_CAP_DMA);
    ESP_RETURN_ON_FALSE(s_pixels != NULL, ESP_ERR_NO_MEM, TAG,
                        "frame buffer allocation failed");

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
    ESP_RETURN_ON_ERROR(tft_command(0x21), TAG, "MODE 3 inversion failed");
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_RETURN_ON_ERROR(tft_command(0x29), TAG, "display on failed");
    vTaskDelay(pdMS_TO_TICKS(100));
    return ESP_OK;
}

static const uint8_t *font_rows(char character)
{
    for (size_t i = 0; i < sizeof(s_font) / sizeof(s_font[0]); ++i) {
        if (s_font[i].character == character) {
            return s_font[i].rows;
        }
    }
    return s_font[0].rows;
}

static void frame_clear(uint16_t color)
{
    for (size_t i = 0; i < TFT_WIDTH * TFT_HEIGHT; ++i) {
        s_pixels[2 * i] = color >> 8;
        s_pixels[2 * i + 1] = color & 0xff;
    }
}

static void frame_rectangle(int x, int y, int width, int height,
                            uint16_t color)
{
    for (int row = y; row < y + height; ++row) {
        if (row < 0 || row >= TFT_HEIGHT) {
            continue;
        }
        for (int column = x; column < x + width; ++column) {
            if (column < 0 || column >= TFT_WIDTH) {
                continue;
            }
            const size_t offset = 2 * (row * TFT_WIDTH + column);
            s_pixels[offset] = color >> 8;
            s_pixels[offset + 1] = color & 0xff;
        }
    }
}

static void frame_character(int x, int y, char character, int scale,
                            uint16_t color)
{
    const uint8_t *rows = font_rows(character);
    for (int row = 0; row < 7; ++row) {
        for (int column = 0; column < 5; ++column) {
            if (rows[row] & (1U << (4 - column))) {
                frame_rectangle(x + column * scale, y + row * scale,
                                scale, scale, color);
            }
        }
    }
}

static void frame_text(int x, int y, const char *text, int scale,
                       uint16_t color)
{
    while (*text != '\0') {
        frame_character(x, y, *text++, scale, color);
        x += 6 * scale;
    }
}

static void format_rpm_line(char *line, size_t size, char motor, int rpm)
{
    if (rpm > 999) {
        rpm = 999;
    }
    snprintf(line, size, "%c %3d RPM", motor, rpm);
}

static esp_err_t render_values(const display_values_t *values)
{
    char line[16];
    frame_clear(0x0000);
    frame_rectangle(2, 2, TFT_WIDTH - 4, 2, 0xffff);
    frame_rectangle(2, TFT_HEIGHT - 4, TFT_WIDTH - 4, 2, 0xffff);
    frame_text(10, 10, "MOTOR RPM", 2, 0xffff);

    format_rpm_line(line, sizeof(line), 'A', values->rpm_a);
    frame_text(10, 39, line, 2, 0xffff);
    format_rpm_line(line, sizeof(line), 'B', values->rpm_b);
    frame_text(10, 65, line, 2, 0xffff);
    format_rpm_line(line, sizeof(line), 'D', values->rpm_d);
    frame_text(10, 91, line, 2, 0xffff);

    if (values->distance_mm < 0) {
        snprintf(line, sizeof(line), "DIST ---");
    } else {
        int distance = values->distance_mm;
        if (distance > 9999) {
            distance = 9999;
        }
        snprintf(line, sizeof(line), "DIST %d", distance);
    }
    frame_text(10, 126, line, 2, 0xffff);

    ESP_RETURN_ON_ERROR(tft_set_full_window(), TAG, "set frame window failed");
    return tft_pixel_data(s_pixels, TFT_WIDTH * TFT_HEIGHT * 2);
}

static void display_task(void *argument)
{
    (void)argument;
    TickType_t last_refresh = xTaskGetTickCount();
    while (true) {
        display_values_t values;
        portENTER_CRITICAL(&s_values_lock);
        values = s_values;
        portEXIT_CRITICAL(&s_values_lock);

        const esp_err_t error = render_values(&values);
        if (error != ESP_OK) {
            ESP_LOGE(TAG, "screen update failed: %s", esp_err_to_name(error));
        }
        xTaskDelayUntil(&last_refresh, pdMS_TO_TICKS(TFT_REFRESH_MS));
    }
}

esp_err_t tft_status_display_start(void)
{
    ESP_RETURN_ON_ERROR(tft_init(), TAG, "TFT initialization failed");
    ESP_RETURN_ON_FALSE(xTaskCreatePinnedToCore(display_task, "tft_display",
                                                4096, NULL, 1, NULL, 1) == pdPASS,
                        ESP_ERR_NO_MEM, TAG, "display task creation failed");
    ESP_LOGI(TAG, "MODE 3 active; RPM/distance display started");
    return ESP_OK;
}

void tft_status_display_set(int rpm_a, int rpm_b, int rpm_d,
                            int distance_mm)
{
    portENTER_CRITICAL(&s_values_lock);
    s_values.rpm_a = rpm_a;
    s_values.rpm_b = rpm_b;
    s_values.rpm_d = rpm_d;
    s_values.distance_mm = distance_mm;
    portEXIT_CRITICAL(&s_values_lock);
}
