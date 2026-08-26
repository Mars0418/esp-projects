#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define DRIVER_STBY GPIO_NUM_5

#define PWM_FREQUENCY_HZ 20000
#define TEST_PWM_DUTY       180
#define TEST_RUN_MS         800
#define TEST_PAUSE_MS       400
#define REPORT_INTERVAL_MS  100
#define MIN_ENCODER_COUNTS    20
#define MAX_IDLE_COUNTS        5

typedef struct {
    const char *name;
    const char *encoder_name;
    gpio_num_t pwm_pin;
    gpio_num_t in1_pin;
    gpio_num_t in2_pin;
    gpio_num_t phase_a_pin;
    gpio_num_t phase_b_pin;
    ledc_channel_t pwm_channel;
    volatile int32_t encoder_count;
    volatile uint8_t previous_state;
} wheel_t;

enum {
    WHEEL_A = 0,
    WHEEL_B = 1,
    WHEEL_D = 2,
    WHEEL_COUNT = 3,
};

static const char *TAG = "three_wheel_test";

static wheel_t wheels[WHEEL_COUNT] = {
    {"A", "E1", GPIO_NUM_6, GPIO_NUM_15, GPIO_NUM_7,
     GPIO_NUM_16, GPIO_NUM_17, LEDC_CHANNEL_0, 0, 0},
    {"B", "E2", GPIO_NUM_11, GPIO_NUM_9, GPIO_NUM_10,
     GPIO_NUM_8, GPIO_NUM_18, LEDC_CHANNEL_1, 0, 0},
    {"D", "E4", GPIO_NUM_40, GPIO_NUM_42, GPIO_NUM_41,
     GPIO_NUM_2, GPIO_NUM_1, LEDC_CHANNEL_2, 0, 0},
};

static const int8_t quadrature_delta[16] = {
     0,  1, -1,  0,
    -1,  0,  0,  1,
     1,  0,  0, -1,
     0, -1,  1,  0,
};

static uint8_t encoder_state(const wheel_t *wheel)
{
    return ((uint8_t)gpio_get_level(wheel->phase_a_pin) << 1) |
           (uint8_t)gpio_get_level(wheel->phase_b_pin);
}

static void IRAM_ATTR encoder_isr(void *arg)
{
    wheel_t *wheel = (wheel_t *)arg;
    const uint8_t current = encoder_state(wheel);
    const uint8_t transition = (wheel->previous_state << 2) | current;
    wheel->encoder_count += quadrature_delta[transition];
    wheel->previous_state = current;
}

static void configure_output_low(gpio_num_t pin)
{
    ESP_ERROR_CHECK(gpio_reset_pin(pin));
    ESP_ERROR_CHECK(gpio_set_direction(pin, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_level(pin, 0));
}

static void safe_stop(void)
{
    ESP_ERROR_CHECK(gpio_set_level(DRIVER_STBY, 0));
    for (size_t i = 0; i < WHEEL_COUNT; ++i) {
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE,
                                      wheels[i].pwm_channel, 0));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE,
                                         wheels[i].pwm_channel));
        ESP_ERROR_CHECK(gpio_set_level(wheels[i].in1_pin, 0));
        ESP_ERROR_CHECK(gpio_set_level(wheels[i].in2_pin, 0));
    }
}

static void drive_one(size_t index, int direction)
{
    safe_stop();
    wheel_t *wheel = &wheels[index];
    ESP_ERROR_CHECK(gpio_set_level(wheel->in1_pin, direction > 0));
    ESP_ERROR_CHECK(gpio_set_level(wheel->in2_pin, direction < 0));
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE,
                                  wheel->pwm_channel, TEST_PWM_DUTY));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE,
                                     wheel->pwm_channel));
    ESP_ERROR_CHECK(gpio_set_level(DRIVER_STBY, 1));
}

static void reset_encoder_counts(void)
{
    for (size_t i = 0; i < WHEEL_COUNT; ++i) {
        ESP_ERROR_CHECK(gpio_intr_disable(wheels[i].phase_a_pin));
        ESP_ERROR_CHECK(gpio_intr_disable(wheels[i].phase_b_pin));
        wheels[i].encoder_count = 0;
        wheels[i].previous_state = encoder_state(&wheels[i]);
        ESP_ERROR_CHECK(gpio_intr_enable(wheels[i].phase_a_pin));
        ESP_ERROR_CHECK(gpio_intr_enable(wheels[i].phase_b_pin));
    }
}

static bool emergency_stop_requested(void)
{
    char input[8];
    const ssize_t count = read(STDIN_FILENO, input, sizeof(input));
    if (count <= 0) {
        return false;
    }
    for (ssize_t i = 0; i < count; ++i) {
        if (tolower((unsigned char)input[i]) == 'x' || input[i] == ' ') {
            return true;
        }
    }
    return false;
}

static void report_sample(const char *stage, size_t active, int direction,
                          int64_t elapsed_ms)
{
    ESP_LOGI(TAG,
             "TEST,%s,%s,dir=%d,t_ms=%" PRId64
             ",encA=%" PRId32 ",encB=%" PRId32 ",encD=%" PRId32
             ",E1=%u%u,E2=%u%u,E4=%u%u",
             stage, wheels[active].name, direction, elapsed_ms,
             wheels[WHEEL_A].encoder_count,
             wheels[WHEEL_B].encoder_count,
             wheels[WHEEL_D].encoder_count,
             (unsigned)gpio_get_level(wheels[WHEEL_A].phase_a_pin),
             (unsigned)gpio_get_level(wheels[WHEEL_A].phase_b_pin),
             (unsigned)gpio_get_level(wheels[WHEEL_B].phase_a_pin),
             (unsigned)gpio_get_level(wheels[WHEEL_B].phase_b_pin),
             (unsigned)gpio_get_level(wheels[WHEEL_D].phase_a_pin),
             (unsigned)gpio_get_level(wheels[WHEEL_D].phase_b_pin));
}

static bool run_direction(size_t index, int direction, const char *stage)
{
    const int64_t started_us = esp_timer_get_time();
    int64_t next_report_us = started_us;
    drive_one(index, direction);
    ESP_LOGW(TAG, "NOW %s wheel %s (%s), direction=%+d, PWM=%d/1023",
             stage, wheels[index].name, wheels[index].encoder_name,
             direction, TEST_PWM_DUTY);

    while ((esp_timer_get_time() - started_us) / 1000 < TEST_RUN_MS) {
        if (emergency_stop_requested()) {
            safe_stop();
            ESP_LOGE(TAG, "EMERGENCY STOP; test cancelled");
            return false;
        }
        const int64_t now_us = esp_timer_get_time();
        if (now_us >= next_report_us) {
            report_sample(stage, index, direction,
                          (now_us - started_us) / 1000);
            next_report_us = now_us + (int64_t)REPORT_INTERVAL_MS * 1000;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    safe_stop();
    return true;
}

static bool idle_encoders_are_quiet(size_t active, const int32_t *counts)
{
    for (size_t i = 0; i < WHEEL_COUNT; ++i) {
        if (i != active && abs(counts[i]) > MAX_IDLE_COUNTS) {
            return false;
        }
    }
    return true;
}

static bool run_wheel_test(size_t index)
{
    int32_t forward_counts[WHEEL_COUNT];
    int32_t reverse_counts[WHEEL_COUNT];

    safe_stop();
    reset_encoder_counts();
    ESP_LOGW(TAG, "CHECK PHYSICAL WHEEL: only channel %s should move",
             wheels[index].name);

    if (!run_direction(index, +1, "FORWARD")) {
        return false;
    }
    for (size_t i = 0; i < WHEEL_COUNT; ++i) {
        forward_counts[i] = wheels[i].encoder_count;
    }

    vTaskDelay(pdMS_TO_TICKS(TEST_PAUSE_MS));
    reset_encoder_counts();
    if (!run_direction(index, -1, "REVERSE")) {
        return false;
    }
    for (size_t i = 0; i < WHEEL_COUNT; ++i) {
        reverse_counts[i] = wheels[i].encoder_count;
    }
    safe_stop();

    const bool forward_ok = abs(forward_counts[index]) >= MIN_ENCODER_COUNTS;
    const bool reverse_ok = abs(reverse_counts[index]) >= MIN_ENCODER_COUNTS;
    const bool direction_ok =
        (int64_t)forward_counts[index] * reverse_counts[index] < 0;
    const bool mapping_ok = idle_encoders_are_quiet(index, forward_counts) &&
                            idle_encoders_are_quiet(index, reverse_counts);
    const bool passed = forward_ok && reverse_ok && direction_ok && mapping_ok;

    ESP_LOGW(TAG,
             "RESULT,%s,%s,forward=%" PRId32 ",reverse=%" PRId32
             ",direction=%s,mapping=%s",
             wheels[index].name, passed ? "PASS" : "FAIL",
             forward_counts[index], reverse_counts[index],
             direction_ok ? "PASS" : "FAIL",
             mapping_ok ? "PASS" : "FAIL");
    if (!passed) {
        ESP_LOGE(TAG,
                 "FAIL %s: verify motor movement, %s wiring, power and common GND",
                 wheels[index].name, wheels[index].encoder_name);
    }
    vTaskDelay(pdMS_TO_TICKS(TEST_PAUSE_MS));
    return passed;
}

static void run_all_tests(void)
{
    unsigned passed = 0;
    for (size_t i = 0; i < WHEEL_COUNT; ++i) {
        if (run_wheel_test(i)) {
            ++passed;
        }
    }
    safe_stop();
    ESP_LOGW(TAG, "SUMMARY,%u/%u wheels passed; SAFE STOP",
             passed, WHEEL_COUNT);
}

static void hardware_init(void)
{
    configure_output_low(DRIVER_STBY);
    for (size_t i = 0; i < WHEEL_COUNT; ++i) {
        configure_output_low(wheels[i].in1_pin);
        configure_output_low(wheels[i].in2_pin);
    }

    const ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = PWM_FREQUENCY_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    for (size_t i = 0; i < WHEEL_COUNT; ++i) {
        const ledc_channel_config_t channel = {
            .gpio_num = wheels[i].pwm_pin,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = wheels[i].pwm_channel,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = LEDC_TIMER_0,
            .duty = 0,
            .hpoint = 0,
            .flags.output_invert = 0,
        };
        ESP_ERROR_CHECK(ledc_channel_config(&channel));
    }
    safe_stop();

    uint64_t encoder_mask = 0;
    for (size_t i = 0; i < WHEEL_COUNT; ++i) {
        encoder_mask |= 1ULL << wheels[i].phase_a_pin;
        encoder_mask |= 1ULL << wheels[i].phase_b_pin;
    }
    const gpio_config_t encoder_config = {
        .pin_bit_mask = encoder_mask,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&encoder_config));
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    for (size_t i = 0; i < WHEEL_COUNT; ++i) {
        wheels[i].previous_state = encoder_state(&wheels[i]);
        ESP_ERROR_CHECK(gpio_isr_handler_add(wheels[i].phase_a_pin,
                                              encoder_isr, &wheels[i]));
        ESP_ERROR_CHECK(gpio_isr_handler_add(wheels[i].phase_b_pin,
                                              encoder_isr, &wheels[i]));
    }
}

void app_main(void)
{
    hardware_init();

    usb_serial_jtag_driver_config_t usb_config =
        USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    usb_config.rx_buffer_size = 256;
    usb_config.tx_buffer_size = 1024;
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_config));
    usb_serial_jtag_vfs_use_driver();

    const int stdin_flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    ESP_ERROR_CHECK(stdin_flags < 0 ? ESP_FAIL : ESP_OK);
    ESP_ERROR_CHECK(fcntl(STDIN_FILENO, F_SETFL,
                          stdin_flags | O_NONBLOCK) < 0 ? ESP_FAIL : ESP_OK);

    ESP_LOGW(TAG, "THREE-WHEEL DIAGNOSTIC READY; chassis must be lifted");
    ESP_LOGI(TAG, "Pins: A PWMA=6 AIN1=15 AIN2=7 E1=16/17");
    ESP_LOGI(TAG, "Pins: B PWMB=11 BIN1=9 BIN2=10 E2=8/18");
    ESP_LOGI(TAG, "Pins: D PWMD=40 DIN1=42 DIN2=41 E4=2/1");
    ESP_LOGI(TAG, "Keys: 1=A, 2=B, 4=D, T=all; X/SPACE=emergency stop");

    while (1) {
        char input[8];
        const ssize_t count = read(STDIN_FILENO, input, sizeof(input));
        if (count > 0) {
            for (ssize_t i = 0; i < count; ++i) {
                switch (tolower((unsigned char)input[i])) {
                case '1': run_wheel_test(WHEEL_A); break;
                case '2': run_wheel_test(WHEEL_B); break;
                case '4': run_wheel_test(WHEEL_D); break;
                case 't': run_all_tests(); break;
                case 'x':
                case ' ': safe_stop(); ESP_LOGW(TAG, "SAFE STOP"); break;
                default: break;
                }
            }
        } else if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            safe_stop();
            ESP_LOGE(TAG, "USB input error=%d; SAFE STOP", errno);
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
