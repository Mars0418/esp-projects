#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_err.h"
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

static const char *TAG = "tft_test";
static spi_device_handle_t s_tft;
static uint8_t *s_pixels;

static const uint8_t s_gamma_a_positive[] = {
    0x02, 0x1c, 0x07, 0x12, 0x37, 0x32, 0x29, 0x2d,
    0x29, 0x25, 0x2b, 0x39, 0x00, 0x01, 0x03, 0x10,
};
static const uint8_t s_gamma_a_negative[] = {
    0x03, 0x1d, 0x07, 0x06, 0x2e, 0x2c, 0x29, 0x2d,
    0x2e, 0x2e, 0x37, 0x3f, 0x00, 0x00, 0x02, 0x10,
};
static const uint8_t s_gamma_b_positive[] = {
    0x0f, 0x1a, 0x0f, 0x18, 0x2f, 0x28, 0x20, 0x22,
    0x1f, 0x1b, 0x23, 0x37, 0x00, 0x07, 0x02, 0x10,
};
static const uint8_t s_gamma_b_negative[] = {
    0x0f, 0x1b, 0x0f, 0x17, 0x33, 0x2c, 0x29, 0x2e,
    0x30, 0x30, 0x39, 0x3f, 0x00, 0x07, 0x03, 0x10,
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
    return tft_write(false, &command, sizeof(command));
}

static esp_err_t tft_data(const void *data, size_t length)
{
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

static esp_err_t tft_command_data(uint8_t command, const uint8_t *data,
                                  size_t length)
{
    ESP_RETURN_ON_ERROR(tft_command(command), TAG, "command 0x%02x failed",
                        command);
    return tft_data(data, length);
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
    ESP_RETURN_ON_ERROR(gpio_set_level(TFT_PIN_DC, 0), TAG, "D/C low failed");

    // The display RST pin is tied to the ESP32-S3 EN/RST line. A software reset
    // here gives the controller a clean, repeatable start after the app boots.
    ESP_RETURN_ON_ERROR(tft_command(0x01), TAG, "software reset failed");
    vTaskDelay(pdMS_TO_TICKS(150));
    ESP_RETURN_ON_ERROR(tft_command(0x11), TAG, "sleep out failed");
    vTaskDelay(pdMS_TO_TICKS(120));

    static const uint8_t frame_rate[] = {0x01, 0x2c, 0x2d};
    static const uint8_t frame_rate_idle[] = {0x01, 0x2c, 0x2d};
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

    ESP_RETURN_ON_ERROR(tft_command_data(0xb1, frame_rate,
                                          sizeof(frame_rate)), TAG, "B1 failed");
    ESP_RETURN_ON_ERROR(tft_command_data(0xb2, frame_rate_idle,
                                          sizeof(frame_rate_idle)), TAG, "B2 failed");
    ESP_RETURN_ON_ERROR(tft_command_data(0xb3, frame_rate_partial,
                                          sizeof(frame_rate_partial)), TAG, "B3 failed");
    ESP_RETURN_ON_ERROR(tft_command_data(0xb4, inversion_control,
                                          sizeof(inversion_control)), TAG, "B4 failed");
    ESP_RETURN_ON_ERROR(tft_command_data(0xc0, power_control_1,
                                          sizeof(power_control_1)), TAG, "C0 failed");
    ESP_RETURN_ON_ERROR(tft_command_data(0xc1, power_control_2,
                                          sizeof(power_control_2)), TAG, "C1 failed");
    ESP_RETURN_ON_ERROR(tft_command_data(0xc2, power_control_3,
                                          sizeof(power_control_3)), TAG, "C2 failed");
    ESP_RETURN_ON_ERROR(tft_command_data(0xc3, power_control_4,
                                          sizeof(power_control_4)), TAG, "C3 failed");
    ESP_RETURN_ON_ERROR(tft_command_data(0xc4, power_control_5,
                                          sizeof(power_control_5)), TAG, "C4 failed");
    ESP_RETURN_ON_ERROR(tft_command_data(0xc5, vcom_control,
                                          sizeof(vcom_control)), TAG, "C5 failed");
    ESP_RETURN_ON_ERROR(tft_command(0x20), TAG, "inversion off failed");
    ESP_RETURN_ON_ERROR(tft_command_data(0x36, memory_access,
                                          sizeof(memory_access)), TAG, "MADCTL failed");
    ESP_RETURN_ON_ERROR(tft_command_data(0x3a, pixel_format,
                                          sizeof(pixel_format)), TAG, "COLMOD failed");
    ESP_RETURN_ON_ERROR(tft_command_data(0xe0, s_gamma_a_positive,
                                          sizeof(s_gamma_a_positive)), TAG, "E0 failed");
    ESP_RETURN_ON_ERROR(tft_command_data(0xe1, s_gamma_a_negative,
                                          sizeof(s_gamma_a_negative)), TAG, "E1 failed");
    ESP_RETURN_ON_ERROR(tft_command(0x13), TAG, "normal mode failed");
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_RETURN_ON_ERROR(tft_command(0x29), TAG, "display on failed");
    vTaskDelay(pdMS_TO_TICKS(100));
    return ESP_OK;
}

static esp_err_t tft_set_window(uint16_t x0, uint16_t y0,
                                uint16_t x1, uint16_t y1)
{
    const uint8_t columns[] = {
        (uint8_t)(x0 >> 8), (uint8_t)x0,
        (uint8_t)(x1 >> 8), (uint8_t)x1,
    };
    const uint8_t rows[] = {
        (uint8_t)(y0 >> 8), (uint8_t)y0,
        (uint8_t)(y1 >> 8), (uint8_t)y1,
    };
    ESP_RETURN_ON_ERROR(tft_command_data(0x2a, columns, sizeof(columns)),
                        TAG, "set columns failed");
    ESP_RETURN_ON_ERROR(tft_command_data(0x2b, rows, sizeof(rows)),
                        TAG, "set rows failed");
    return tft_command(0x2c);
}

static esp_err_t tft_fill_rectangle(uint16_t x, uint16_t y, uint16_t width,
                                    uint16_t height, uint16_t color)
{
    if (width == 0 || height == 0 || x >= TFT_WIDTH || y >= TFT_HEIGHT) {
        return ESP_OK;
    }
    if (x + width > TFT_WIDTH) {
        width = TFT_WIDTH - x;
    }
    if (y + height > TFT_HEIGHT) {
        height = TFT_HEIGHT - y;
    }

    ESP_RETURN_ON_ERROR(tft_set_window(x, y, x + width - 1, y + height - 1),
                        TAG, "set window failed");

    const size_t pixel_count = width * height;
    for (size_t i = 0; i < pixel_count; ++i) {
        s_pixels[2 * i] = color >> 8;
        s_pixels[2 * i + 1] = color & 0xff;
    }
    return tft_pixel_data(s_pixels, pixel_count * 2);
}

static esp_err_t show_color(const char *name, uint16_t color)
{
    ESP_LOGI(TAG, "displaying %s", name);
    return tft_fill_rectangle(0, 0, TFT_WIDTH, TFT_HEIGHT, color);
}

static esp_err_t show_color_bars(void)
{
    static const uint16_t colors[] = {
        0xffff, // white
        0xffe0, // yellow
        0x07ff, // cyan
        0x07e0, // green
        0xf81f, // magenta
        0xf800, // red
        0x001f, // blue
        0x0000, // black
    };
    const uint16_t bar_width = TFT_WIDTH / (sizeof(colors) / sizeof(colors[0]));
    for (size_t i = 0; i < sizeof(colors) / sizeof(colors[0]); ++i) {
        ESP_RETURN_ON_ERROR(tft_fill_rectangle(i * bar_width, 0, bar_width,
                                                TFT_HEIGHT, colors[i]),
                            TAG, "color bar failed");
    }
    return ESP_OK;
}

static esp_err_t set_visual_mode(uint8_t madctl, bool inverted)
{
    ESP_RETURN_ON_ERROR(tft_command_data(0x36, &madctl, sizeof(madctl)),
                        TAG, "MADCTL mode failed");
    ESP_RETURN_ON_ERROR(tft_command(inverted ? 0x21 : 0x20), TAG,
                        "inversion mode failed");
    vTaskDelay(pdMS_TO_TICKS(20));
    return ESP_OK;
}

typedef struct {
    char character;
    uint8_t rows[7];
} glyph_t;

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
    {'D', {0x1e, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1e}},
    {'E', {0x1f, 0x10, 0x10, 0x1e, 0x10, 0x10, 0x1f}},
    {'F', {0x1f, 0x10, 0x10, 0x1e, 0x10, 0x10, 0x10}},
    {'I', {0x1f, 0x04, 0x04, 0x04, 0x04, 0x04, 0x1f}},
    {'K', {0x11, 0x12, 0x14, 0x18, 0x14, 0x12, 0x11}},
    {'L', {0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1f}},
    {'M', {0x11, 0x1b, 0x15, 0x15, 0x11, 0x11, 0x11}},
    {'O', {0x0e, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0e}},
    {'P', {0x1e, 0x11, 0x11, 0x1e, 0x10, 0x10, 0x10}},
    {'S', {0x0f, 0x10, 0x10, 0x0e, 0x01, 0x01, 0x1e}},
    {'T', {0x1f, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04}},
    {'X', {0x11, 0x11, 0x0a, 0x04, 0x0a, 0x11, 0x11}},
    {'Y', {0x11, 0x11, 0x0a, 0x04, 0x04, 0x04, 0x04}},
};

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

static void frame_rectangle(int x, int y, int width, int height, uint16_t color)
{
    for (int row = y; row < y + height; ++row) {
        if (row < 0 || row >= TFT_HEIGHT) {
            continue;
        }
        for (int column = x; column < x + width; ++column) {
            if (column < 0 || column >= TFT_WIDTH) {
                continue;
            }
            size_t offset = 2 * (row * TFT_WIDTH + column);
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

static esp_err_t show_profile_screen(int profile_number, uint16_t background,
                                     uint16_t foreground)
{
    char mode_label[] = "MODE 1";
    mode_label[5] = '0' + profile_number;

    frame_clear(background);
    frame_rectangle(3, 3, TFT_WIDTH - 6, 3, foreground);
    frame_rectangle(3, TFT_HEIGHT - 6, TFT_WIDTH - 6, 3, foreground);
    frame_rectangle(3, 3, 3, TFT_HEIGHT - 6, foreground);
    frame_rectangle(TFT_WIDTH - 6, 3, 3, TFT_HEIGHT - 6, foreground);
    frame_text(16, 24, "ESP32-S3", 2, foreground);
    frame_text(28, 65, mode_label, 2, foreground);
    frame_text(4, 106, "DISPLAY OK", 2, foreground);

    ESP_RETURN_ON_ERROR(tft_set_window(0, 0, TFT_WIDTH - 1, TFT_HEIGHT - 1),
                        TAG, "text window failed");
    return tft_pixel_data(s_pixels, TFT_WIDTH * TFT_HEIGHT * 2);
}

typedef struct {
    uint8_t madctl;
    bool inverted;
    uint8_t vcom;
    bool gamma_b;
    uint16_t background;
    uint16_t foreground;
    const char *description;
} display_profile_t;

static esp_err_t apply_profile(const display_profile_t *profile)
{
    const uint8_t *positive_gamma = profile->gamma_b
        ? s_gamma_b_positive : s_gamma_a_positive;
    const uint8_t *negative_gamma = profile->gamma_b
        ? s_gamma_b_negative : s_gamma_a_negative;

    ESP_RETURN_ON_ERROR(tft_command_data(0xc5, &profile->vcom, 1),
                        TAG, "profile VCOM failed");
    ESP_RETURN_ON_ERROR(tft_command_data(0xe0, positive_gamma, 16),
                        TAG, "profile positive gamma failed");
    ESP_RETURN_ON_ERROR(tft_command_data(0xe1, negative_gamma, 16),
                        TAG, "profile negative gamma failed");
    return set_visual_mode(profile->madctl, profile->inverted);
}

void app_main(void)
{
    ESP_LOGI(TAG, "LQ_TFT18SPIV33 test starting");
    ESP_LOGI(TAG, "pins: SCK=%d MOSI=%d CS=%d DC=%d RST=EN",
             TFT_PIN_SCLK, TFT_PIN_MOSI, TFT_PIN_CS, TFT_PIN_DC);

    ESP_ERROR_CHECK(tft_init());
    ESP_LOGI(TAG, "ST7735S initialization complete");

    static const display_profile_t profiles[] = {
        {0xc8, false, 0x0e, false, 0xffff, 0x0000, "default, light"},
        {0xc8, false, 0x0e, false, 0x0000, 0xffff, "default, dark"},
        {0xc8, true,  0x0e, false, 0x0000, 0xffff, "inverted"},
        {0xc0, false, 0x0e, false, 0x0000, 0xffff, "RGB order"},
        {0xc8, false, 0x08, false, 0x0000, 0xffff, "lower VCOM"},
        {0xc8, false, 0x18, false, 0x0000, 0xffff, "higher VCOM"},
        {0xc8, false, 0x0e, true,  0x0000, 0xffff, "alternate gamma"},
        {0xc8, true,  0x18, true,  0x0000, 0xffff, "alternate gamma + inverted"},
    };

    while (true) {
        for (size_t i = 0; i < sizeof(profiles) / sizeof(profiles[0]); ++i) {
            ESP_LOGI(TAG, "MODE %u: %s", (unsigned)(i + 1),
                     profiles[i].description);
            ESP_ERROR_CHECK(apply_profile(&profiles[i]));
            ESP_ERROR_CHECK(show_profile_screen(i + 1,
                                                profiles[i].background,
                                                profiles[i].foreground));
            vTaskDelay(pdMS_TO_TICKS(6000));
        }
    }
}
