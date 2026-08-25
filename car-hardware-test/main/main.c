#include <ctype.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "car_test_pins.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_strip.h"

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#define MOTOR_MAX_PERCENT 70
#define MOTOR_MAX_RUN_MS 3000
#define I2C_PORT I2C_NUM_0

static led_strip_handle_t s_led;
static esp_lcd_panel_handle_t s_panel;
static volatile int32_t s_encoder_a;
static volatile int32_t s_encoder_b;
static bool s_i2c_ready;
static bool s_motor_ready;
static bool s_servo_ready;

static bool pin_ok(gpio_num_t pin)
{
    return pin != GPIO_NUM_NC && GPIO_IS_VALID_GPIO(pin);
}

static bool pins_ok(const gpio_num_t *pins, size_t count)
{
    for (size_t i = 0; i < count; ++i) {
        if (!pin_ok(pins[i])) {
            return false;
        }
    }
    return true;
}

static void print_disabled(const char *module)
{
    printf("ERR %s is disabled; set its pins in main/car_test_pins.h\n", module);
}

static void led_set(uint8_t red, uint8_t green, uint8_t blue)
{
    if (!s_led) {
        return;
    }
    ESP_ERROR_CHECK(led_strip_set_pixel(s_led, 0, red, green, blue));
    ESP_ERROR_CHECK(led_strip_refresh(s_led));
}

static void init_led(void)
{
    if (!pin_ok(CAR_LED_GPIO)) {
        return;
    }
    led_strip_config_t strip_config = {
        .strip_gpio_num = CAR_LED_GPIO,
        .max_leds = 1,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
    };
    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,
        .mem_block_symbols = 64,
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &s_led));
    led_set(0, 24, 0);
}

static void init_inputs(void)
{
    const gpio_num_t pins[] = {
        CAR_LINE_LEFT_OUTER_GPIO, CAR_LINE_LEFT_INNER_GPIO,
        CAR_LINE_RIGHT_INNER_GPIO, CAR_LINE_RIGHT_OUTER_GPIO,
    };
    for (size_t i = 0; i < ARRAY_SIZE(pins); ++i) {
        if (pin_ok(pins[i])) {
            gpio_config_t config = {
                .pin_bit_mask = 1ULL << pins[i],
                .mode = GPIO_MODE_INPUT,
                .pull_up_en = GPIO_PULLUP_ENABLE,
                .pull_down_en = GPIO_PULLDOWN_DISABLE,
                .intr_type = GPIO_INTR_DISABLE,
            };
            ESP_ERROR_CHECK(gpio_config(&config));
        }
    }
}

static void read_line_sensors(void)
{
    const gpio_num_t pins[] = {
        CAR_LINE_LEFT_OUTER_GPIO, CAR_LINE_LEFT_INNER_GPIO,
        CAR_LINE_RIGHT_INNER_GPIO, CAR_LINE_RIGHT_OUTER_GPIO,
    };
    if (!pins_ok(pins, ARRAY_SIZE(pins))) {
        print_disabled("line sensors");
        return;
    }
    printf("LINE left_outer=%d left_inner=%d right_inner=%d right_outer=%d\n",
           gpio_get_level(pins[0]), gpio_get_level(pins[1]),
           gpio_get_level(pins[2]), gpio_get_level(pins[3]));
}

static bool wait_gpio_level(gpio_num_t pin, int level, int timeout_us, int64_t *at_us)
{
    int64_t deadline = esp_timer_get_time() + timeout_us;
    while (gpio_get_level(pin) != level) {
        if (esp_timer_get_time() >= deadline) {
            return false;
        }
    }
    if (at_us) {
        *at_us = esp_timer_get_time();
    }
    return true;
}

static void read_ultrasonic(void)
{
    const gpio_num_t pins[] = {CAR_ULTRASONIC_TRIG_GPIO, CAR_ULTRASONIC_ECHO_GPIO};
    if (!pins_ok(pins, ARRAY_SIZE(pins))) {
        print_disabled("HC-SR04");
        return;
    }
    gpio_set_direction(CAR_ULTRASONIC_TRIG_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(CAR_ULTRASONIC_ECHO_GPIO, GPIO_MODE_INPUT);
    gpio_set_level(CAR_ULTRASONIC_TRIG_GPIO, 0);
    esp_rom_delay_us(3);
    gpio_set_level(CAR_ULTRASONIC_TRIG_GPIO, 1);
    esp_rom_delay_us(10);
    gpio_set_level(CAR_ULTRASONIC_TRIG_GPIO, 0);

    int64_t rise;
    int64_t fall;
    if (!wait_gpio_level(CAR_ULTRASONIC_ECHO_GPIO, 1, 30000, &rise) ||
        !wait_gpio_level(CAR_ULTRASONIC_ECHO_GPIO, 0, 30000, &fall)) {
        printf("ERR HC-SR04 timeout\n");
        return;
    }
    printf("DISTANCE %.1f cm pulse=%" PRId64 " us\n", (fall - rise) / 58.0, fall - rise);
}

static bool dht_wait(int level, int timeout_us, int64_t *time_us)
{
    return wait_gpio_level(CAR_DHT11_GPIO, level, timeout_us, time_us);
}

static void read_dht11(void)
{
    if (!pin_ok(CAR_DHT11_GPIO)) {
        print_disabled("DHT11");
        return;
    }
    uint8_t data[5] = {0};
    gpio_set_pull_mode(CAR_DHT11_GPIO, GPIO_PULLUP_ONLY);
    gpio_set_direction(CAR_DHT11_GPIO, GPIO_MODE_OUTPUT_OD);
    gpio_set_level(CAR_DHT11_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(CAR_DHT11_GPIO, 1);
    esp_rom_delay_us(30);
    gpio_set_direction(CAR_DHT11_GPIO, GPIO_MODE_INPUT);

    if (!dht_wait(0, 100, NULL) || !dht_wait(1, 100, NULL) || !dht_wait(0, 100, NULL)) {
        printf("ERR DHT11 no response\n");
        return;
    }
    for (int bit = 0; bit < 40; ++bit) {
        int64_t rise;
        int64_t fall;
        if (!dht_wait(1, 100, &rise) || !dht_wait(0, 120, &fall)) {
            printf("ERR DHT11 timeout at bit %d\n", bit);
            return;
        }
        data[bit / 8] <<= 1;
        if (fall - rise > 45) {
            data[bit / 8] |= 1;
        }
    }
    if ((uint8_t)(data[0] + data[1] + data[2] + data[3]) != data[4]) {
        printf("ERR DHT11 checksum\n");
        return;
    }
    printf("DHT11 humidity=%u.%u%% temperature=%u.%uC\n",
           data[0], data[1], data[2], data[3]);
}

static void init_i2c(void)
{
    const gpio_num_t pins[] = {CAR_I2C_SDA_GPIO, CAR_I2C_SCL_GPIO};
    if (!pins_ok(pins, ARRAY_SIZE(pins))) {
        return;
    }
    i2c_config_t config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = CAR_I2C_SDA_GPIO,
        .scl_io_num = CAR_I2C_SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &config));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT, config.mode, 0, 0, 0));
    s_i2c_ready = true;
}

static esp_err_t i2c_write_reg(uint8_t address, uint8_t reg, uint8_t value)
{
    uint8_t bytes[] = {reg, value};
    return i2c_master_write_to_device(I2C_PORT, address, bytes, sizeof(bytes), pdMS_TO_TICKS(100));
}

static esp_err_t i2c_read_regs(uint8_t address, uint8_t reg, uint8_t *data, size_t size)
{
    return i2c_master_write_read_device(I2C_PORT, address, &reg, 1, data, size, pdMS_TO_TICKS(100));
}

static void scan_i2c(void)
{
    if (!s_i2c_ready) {
        print_disabled("I2C");
        return;
    }
    int found = 0;
    printf("I2C devices:");
    for (uint8_t address = 1; address < 0x7f; ++address) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t result = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(20));
        i2c_cmd_link_delete(cmd);
        if (result == ESP_OK) {
            printf(" 0x%02x", address);
            ++found;
        }
    }
    printf(found ? "\n" : " none\n");
}

static void read_mpu6500(void)
{
    if (!s_i2c_ready) {
        print_disabled("MPU6500");
        return;
    }
    uint8_t who = 0;
    if (i2c_read_regs(CAR_MPU6500_I2C_ADDRESS, 0x75, &who, 1) != ESP_OK) {
        printf("ERR MPU6500 not responding at 0x%02x\n", CAR_MPU6500_I2C_ADDRESS);
        return;
    }
    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_write_reg(CAR_MPU6500_I2C_ADDRESS, 0x6b, 0x00));
    vTaskDelay(pdMS_TO_TICKS(10));
    uint8_t bytes[14];
    if (i2c_read_regs(CAR_MPU6500_I2C_ADDRESS, 0x3b, bytes, sizeof(bytes)) != ESP_OK) {
        printf("ERR MPU6500 data read\n");
        return;
    }
    int16_t ax = (bytes[0] << 8) | bytes[1];
    int16_t ay = (bytes[2] << 8) | bytes[3];
    int16_t az = (bytes[4] << 8) | bytes[5];
    int16_t temp = (bytes[6] << 8) | bytes[7];
    int16_t gx = (bytes[8] << 8) | bytes[9];
    int16_t gy = (bytes[10] << 8) | bytes[11];
    int16_t gz = (bytes[12] << 8) | bytes[13];
    printf("MPU6500 who=0x%02x accel=[%d,%d,%d] gyro=[%d,%d,%d] temp=%.1fC\n",
           who, ax, ay, az, gx, gy, gz, temp / 333.87 + 21.0);
}

static void init_display(void)
{
    const gpio_num_t pins[] = {
        CAR_TFT_MOSI_GPIO, CAR_TFT_SCLK_GPIO, CAR_TFT_CS_GPIO,
        CAR_TFT_DC_GPIO, CAR_TFT_RST_GPIO, CAR_TFT_BACKLIGHT_GPIO,
    };
    if (!pins_ok(pins, ARRAY_SIZE(pins))) {
        return;
    }
    spi_bus_config_t bus = {
        .mosi_io_num = CAR_TFT_MOSI_GPIO,
        .miso_io_num = GPIO_NUM_NC,
        .sclk_io_num = CAR_TFT_SCLK_GPIO,
        .quadwp_io_num = GPIO_NUM_NC,
        .quadhd_io_num = GPIO_NUM_NC,
        .max_transfer_sz = CAR_TFT_WIDTH * 40 * sizeof(uint16_t),
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO));
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = CAR_TFT_DC_GPIO,
        .cs_gpio_num = CAR_TFT_CS_GPIO,
        .pclk_hz = 20 * 1000 * 1000,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };
    esp_lcd_panel_io_handle_t io;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST, &io_config, &io));
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = CAR_TFT_RST_GPIO,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io, &panel_config, &s_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(s_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(s_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_set_gap(s_panel, CAR_TFT_X_GAP, CAR_TFT_Y_GAP));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(s_panel, true));
    gpio_set_direction(CAR_TFT_BACKLIGHT_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(CAR_TFT_BACKLIGHT_GPIO, 1);
}

static void display_fill(uint16_t color)
{
    if (!s_panel) {
        print_disabled("ST7789 display");
        return;
    }
    const int rows = 20;
    uint16_t *pixels = malloc(CAR_TFT_WIDTH * rows * sizeof(*pixels));
    if (!pixels) {
        printf("ERR display buffer allocation\n");
        return;
    }
    uint16_t swapped = (color << 8) | (color >> 8);
    for (int i = 0; i < CAR_TFT_WIDTH * rows; ++i) {
        pixels[i] = swapped;
    }
    for (int y = 0; y < CAR_TFT_HEIGHT; y += rows) {
        int end = y + rows > CAR_TFT_HEIGHT ? CAR_TFT_HEIGHT : y + rows;
        ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(s_panel, 0, y, CAR_TFT_WIDTH, end, pixels));
    }
    free(pixels);
    printf("DISPLAY fill=0x%04x\n", color);
}

static void motor_stop(void)
{
    if (!s_motor_ready) {
        return;
    }
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
    gpio_set_level(CAR_MOTOR_A_IN1_GPIO, 0);
    gpio_set_level(CAR_MOTOR_A_IN2_GPIO, 0);
    gpio_set_level(CAR_MOTOR_B_IN1_GPIO, 0);
    gpio_set_level(CAR_MOTOR_B_IN2_GPIO, 0);
    gpio_set_level(CAR_MOTOR_STBY_GPIO, 0);
}

static void init_motors(void)
{
    const gpio_num_t pins[] = {
        CAR_MOTOR_STBY_GPIO, CAR_MOTOR_A_IN1_GPIO, CAR_MOTOR_A_IN2_GPIO,
        CAR_MOTOR_A_PWM_GPIO, CAR_MOTOR_B_IN1_GPIO, CAR_MOTOR_B_IN2_GPIO,
        CAR_MOTOR_B_PWM_GPIO,
    };
    if (!pins_ok(pins, ARRAY_SIZE(pins))) {
        return;
    }
    const gpio_num_t direction_pins[] = {
        CAR_MOTOR_STBY_GPIO, CAR_MOTOR_A_IN1_GPIO, CAR_MOTOR_A_IN2_GPIO,
        CAR_MOTOR_B_IN1_GPIO, CAR_MOTOR_B_IN2_GPIO,
    };
    uint64_t direction_mask = 0;
    for (size_t i = 0; i < ARRAY_SIZE(direction_pins); ++i) {
        direction_mask |= 1ULL << direction_pins[i];
    }
    gpio_config_t outputs = {
        .pin_bit_mask = direction_mask,
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&outputs));
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .freq_hz = 20000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));
    ledc_channel_config_t channels[] = {
        {.gpio_num = CAR_MOTOR_A_PWM_GPIO, .speed_mode = LEDC_LOW_SPEED_MODE,
         .channel = LEDC_CHANNEL_0, .timer_sel = LEDC_TIMER_0},
        {.gpio_num = CAR_MOTOR_B_PWM_GPIO, .speed_mode = LEDC_LOW_SPEED_MODE,
         .channel = LEDC_CHANNEL_1, .timer_sel = LEDC_TIMER_0},
    };
    for (size_t i = 0; i < ARRAY_SIZE(channels); ++i) {
        ESP_ERROR_CHECK(ledc_channel_config(&channels[i]));
    }
    s_motor_ready = true;
    motor_stop();
}

static void motor_run(char motor, int percent, int duration_ms)
{
    if (!s_motor_ready) {
        print_disabled("TB6615 motors");
        return;
    }
    if ((motor != 'a' && motor != 'b') || percent < -MOTOR_MAX_PERCENT ||
        percent > MOTOR_MAX_PERCENT || duration_ms < 1 || duration_ms > MOTOR_MAX_RUN_MS) {
        printf("ERR usage: motor a|b -70..70 1..3000\n");
        return;
    }
    gpio_num_t in1 = motor == 'a' ? CAR_MOTOR_A_IN1_GPIO : CAR_MOTOR_B_IN1_GPIO;
    gpio_num_t in2 = motor == 'a' ? CAR_MOTOR_A_IN2_GPIO : CAR_MOTOR_B_IN2_GPIO;
    ledc_channel_t channel = motor == 'a' ? LEDC_CHANNEL_0 : LEDC_CHANNEL_1;
    gpio_set_level(CAR_MOTOR_STBY_GPIO, 1);
    gpio_set_level(in1, percent > 0);
    gpio_set_level(in2, percent < 0);
    uint32_t duty = (abs(percent) * 1023U) / 100U;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
    printf("MOTOR %c percent=%d duration=%dms\n", motor, percent, duration_ms);
    vTaskDelay(pdMS_TO_TICKS(duration_ms));
    motor_stop();
    printf("MOTOR stopped\n");
}

static void init_servo(void)
{
    if (!pin_ok(CAR_SERVO_GPIO)) {
        return;
    }
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_1,
        .duty_resolution = LEDC_TIMER_14_BIT,
        .freq_hz = 50,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));
    ledc_channel_config_t channel = {
        .gpio_num = CAR_SERVO_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_2,
        .timer_sel = LEDC_TIMER_1,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel));
    s_servo_ready = true;
}

static void servo_set(int angle)
{
    if (!s_servo_ready) {
        print_disabled("MG90S servo");
        return;
    }
    if (angle < 0 || angle > 180) {
        printf("ERR usage: servo 0..180\n");
        return;
    }
    uint32_t pulse_us = 500 + (angle * 2000U) / 180U;
    uint32_t duty = (pulse_us * ((1U << 14) - 1)) / 20000U;
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2));
    printf("SERVO angle=%d pulse=%" PRIu32 "us\n", angle, pulse_us);
}

static void IRAM_ATTR encoder_a_isr(void *arg)
{
    s_encoder_a += gpio_get_level(CAR_ENCODER_A_PHASE_B_GPIO) ? 1 : -1;
}

static void IRAM_ATTR encoder_b_isr(void *arg)
{
    s_encoder_b += gpio_get_level(CAR_ENCODER_B_PHASE_B_GPIO) ? 1 : -1;
}

static void init_encoders(void)
{
    const gpio_num_t pins[] = {
        CAR_ENCODER_A_PHASE_A_GPIO, CAR_ENCODER_A_PHASE_B_GPIO,
        CAR_ENCODER_B_PHASE_A_GPIO, CAR_ENCODER_B_PHASE_B_GPIO,
    };
    if (!pins_ok(pins, ARRAY_SIZE(pins))) {
        return;
    }
    for (size_t i = 0; i < ARRAY_SIZE(pins); ++i) {
        gpio_config_t config = {
            .pin_bit_mask = 1ULL << pins[i],
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .intr_type = (i == 0 || i == 2) ? GPIO_INTR_ANYEDGE : GPIO_INTR_DISABLE,
        };
        ESP_ERROR_CHECK(gpio_config(&config));
    }
    esp_err_t result = gpio_install_isr_service(0);
    if (result != ESP_OK && result != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(result);
    }
    ESP_ERROR_CHECK(gpio_isr_handler_add(CAR_ENCODER_A_PHASE_A_GPIO, encoder_a_isr, NULL));
    ESP_ERROR_CHECK(gpio_isr_handler_add(CAR_ENCODER_B_PHASE_A_GPIO, encoder_b_isr, NULL));
}

static void print_encoders(bool reset)
{
    const gpio_num_t pins[] = {
        CAR_ENCODER_A_PHASE_A_GPIO, CAR_ENCODER_A_PHASE_B_GPIO,
        CAR_ENCODER_B_PHASE_A_GPIO, CAR_ENCODER_B_PHASE_B_GPIO,
    };
    if (!pins_ok(pins, ARRAY_SIZE(pins))) {
        print_disabled("motor encoders");
        return;
    }
    if (reset) {
        s_encoder_a = 0;
        s_encoder_b = 0;
    }
    printf("ENCODER a=%" PRId32 " b=%" PRId32 "\n", s_encoder_a, s_encoder_b);
}

static void print_status(void)
{
    const gpio_num_t line[] = {CAR_LINE_LEFT_OUTER_GPIO, CAR_LINE_LEFT_INNER_GPIO,
                               CAR_LINE_RIGHT_INNER_GPIO, CAR_LINE_RIGHT_OUTER_GPIO};
    const gpio_num_t ultrasonic[] = {CAR_ULTRASONIC_TRIG_GPIO, CAR_ULTRASONIC_ECHO_GPIO};
    printf("STATUS ESP-IDF=%s target=esp32s3\n", esp_get_idf_version());
    printf("  board_led=%s line=%s ultrasonic=%s dht11=%s i2c_mpu6500=%s\n",
           s_led ? "ready" : "disabled", pins_ok(line, ARRAY_SIZE(line)) ? "ready" : "disabled",
           pins_ok(ultrasonic, ARRAY_SIZE(ultrasonic)) ? "ready" : "disabled",
           pin_ok(CAR_DHT11_GPIO) ? "ready" : "disabled", s_i2c_ready ? "ready" : "disabled");
    printf("  display=%s motors=%s servo=%s\n",
           s_panel ? "ready" : "disabled", s_motor_ready ? "ready" : "disabled",
           s_servo_ready ? "ready" : "disabled");
}

static void print_help(void)
{
    puts("Commands:");
    puts("  status                 show configured modules");
    puts("  led green|red|blue|off test on-board RGB LED");
    puts("  line                   read four line sensors");
    puts("  distance               read HC-SR04");
    puts("  dht                     read DHT11");
    puts("  i2c                     scan I2C addresses");
    puts("  imu                     read MPU6500 raw data");
    puts("  display red|green|blue|white|black");
    puts("  encoder [reset]         read/reset encoder counts");
    puts("  servo 0..180            set MG90S angle");
    puts("  motor a|b -70..70 1..3000   signed speed and duration ms");
    puts("  stop                    stop both motors");
    puts("  help");
}

static void handle_command(char *line)
{
    char *argv[5] = {0};
    int argc = 0;
    for (char *token = strtok(line, " \t\r\n"); token && argc < (int)ARRAY_SIZE(argv);
         token = strtok(NULL, " \t\r\n")) {
        for (char *p = token; *p; ++p) {
            *p = (char)tolower((unsigned char)*p);
        }
        argv[argc++] = token;
    }
    if (!argc) return;
    if (!strcmp(argv[0], "help")) print_help();
    else if (!strcmp(argv[0], "status")) print_status();
    else if (!strcmp(argv[0], "line")) read_line_sensors();
    else if (!strcmp(argv[0], "distance")) read_ultrasonic();
    else if (!strcmp(argv[0], "dht")) read_dht11();
    else if (!strcmp(argv[0], "i2c")) scan_i2c();
    else if (!strcmp(argv[0], "imu")) read_mpu6500();
    else if (!strcmp(argv[0], "stop")) { motor_stop(); puts("MOTOR stopped"); }
    else if (!strcmp(argv[0], "encoder")) print_encoders(argc > 1 && !strcmp(argv[1], "reset"));
    else if (!strcmp(argv[0], "servo") && argc == 2) servo_set(atoi(argv[1]));
    else if (!strcmp(argv[0], "motor") && argc == 4)
        motor_run(argv[1][0], atoi(argv[2]), atoi(argv[3]));
    else if (!strcmp(argv[0], "led") && argc == 2) {
        if (!strcmp(argv[1], "green")) led_set(0, 48, 0);
        else if (!strcmp(argv[1], "red")) led_set(48, 0, 0);
        else if (!strcmp(argv[1], "blue")) led_set(0, 0, 48);
        else if (!strcmp(argv[1], "off")) led_set(0, 0, 0);
        else puts("ERR usage: led green|red|blue|off");
    } else if (!strcmp(argv[0], "display") && argc == 2) {
        if (!strcmp(argv[1], "red")) display_fill(0xf800);
        else if (!strcmp(argv[1], "green")) display_fill(0x07e0);
        else if (!strcmp(argv[1], "blue")) display_fill(0x001f);
        else if (!strcmp(argv[1], "white")) display_fill(0xffff);
        else if (!strcmp(argv[1], "black")) display_fill(0x0000);
        else puts("ERR usage: display red|green|blue|white|black");
    } else {
        puts("ERR unknown command; type help");
    }
}

void app_main(void)
{
    init_led();
    init_inputs();
    init_i2c();
    init_display();
    init_motors();
    init_servo();
    init_encoders();

    puts("\nCAR HARDWARE TEST READY");
    puts("Actuators do not move automatically. Type help for commands.");
    print_status();
    print_help();

    char line[128];
    while (true) {
        printf("car-test> ");
        fflush(stdout);
        if (fgets(line, sizeof(line), stdin)) {
            handle_command(line);
        } else {
            clearerr(stdin);
            vTaskDelay(pdMS_TO_TICKS(20));
        }
    }
}
