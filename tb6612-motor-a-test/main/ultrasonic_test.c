#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>

#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/*
 * HC-SR04 test wiring:
 *   VCC  -> 5 V
 *   GND  -> ESP32-S3 GND (common ground)
 *   TRIG -> GPIO3
 *   ECHO -> GPIO4 through the baseboard level shifter or a resistor divider
 *
 * A bare HC-SR04 ECHO pin outputs about 5 V. Never connect it directly to an
 * ESP32-S3 GPIO. If your wiring uses other pins, only change these two lines.
 */
#define ULTRASONIC_TRIG_GPIO       GPIO_NUM_3
#define ULTRASONIC_ECHO_GPIO       GPIO_NUM_4

#define SAMPLE_INTERVAL_MS         250
#define ECHO_IDLE_TIMEOUT_US       10000
#define ECHO_EDGE_TIMEOUT_US       30000
#define HC_SR04_MIN_DISTANCE_CM     2.0f
#define HC_SR04_MAX_DISTANCE_CM   400.0f

static const char *TAG = "ultrasonic_test";

typedef enum {
    MEASURE_OK,
    MEASURE_ECHO_STUCK_HIGH,
    MEASURE_NO_ECHO,
    MEASURE_ECHO_DID_NOT_END,
} measure_status_t;

static bool wait_for_level(int level, int32_t timeout_us, int64_t *edge_time_us)
{
    const int64_t deadline_us = esp_timer_get_time() + timeout_us;

    while (gpio_get_level(ULTRASONIC_ECHO_GPIO) != level) {
        if (esp_timer_get_time() >= deadline_us) {
            return false;
        }
    }

    if (edge_time_us != NULL) {
        *edge_time_us = esp_timer_get_time();
    }
    return true;
}

static measure_status_t measure_distance(int64_t *pulse_width_us,
                                         float *distance_cm)
{
    /* A new trigger is only valid after ECHO has returned to idle (low). */
    if (!wait_for_level(0, ECHO_IDLE_TIMEOUT_US, NULL)) {
        return MEASURE_ECHO_STUCK_HIGH;
    }

    gpio_set_level(ULTRASONIC_TRIG_GPIO, 0);
    esp_rom_delay_us(3);
    gpio_set_level(ULTRASONIC_TRIG_GPIO, 1);
    esp_rom_delay_us(10);
    gpio_set_level(ULTRASONIC_TRIG_GPIO, 0);

    int64_t rising_edge_us = 0;
    int64_t falling_edge_us = 0;
    if (!wait_for_level(1, ECHO_EDGE_TIMEOUT_US, &rising_edge_us)) {
        return MEASURE_NO_ECHO;
    }
    if (!wait_for_level(0, ECHO_EDGE_TIMEOUT_US, &falling_edge_us)) {
        return MEASURE_ECHO_DID_NOT_END;
    }

    *pulse_width_us = falling_edge_us - rising_edge_us;
    *distance_cm = *pulse_width_us / 58.0f;
    return MEASURE_OK;
}

static void ultrasonic_gpio_init(void)
{
    const gpio_config_t trigger_config = {
        .pin_bit_mask = 1ULL << ULTRASONIC_TRIG_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&trigger_config));
    ESP_ERROR_CHECK(gpio_set_level(ULTRASONIC_TRIG_GPIO, 0));

    const gpio_config_t echo_config = {
        .pin_bit_mask = 1ULL << ULTRASONIC_ECHO_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&echo_config));
}

void app_main(void)
{
    ultrasonic_gpio_init();

    ESP_LOGI(TAG, "HC-SR04 continuous distance test ready");
    ESP_LOGI(TAG, "TRIG=GPIO%d, ECHO=GPIO%d, interval=%d ms",
             ULTRASONIC_TRIG_GPIO, ULTRASONIC_ECHO_GPIO, SAMPLE_INTERVAL_MS);
    ESP_LOGW(TAG, "ECHO must be level-shifted to 3.3 V before GPIO%d",
             ULTRASONIC_ECHO_GPIO);
    ESP_LOGI(TAG, "Move a flat object in front of the sensor; Ctrl+] exits monitor");

    uint32_t sample = 0;
    while (true) {
        int64_t pulse_width_us = 0;
        float distance_cm = 0.0f;
        const measure_status_t status = measure_distance(&pulse_width_us,
                                                         &distance_cm);
        ++sample;

        switch (status) {
        case MEASURE_OK:
            if (distance_cm < HC_SR04_MIN_DISTANCE_CM ||
                distance_cm > HC_SR04_MAX_DISTANCE_CM) {
                printf("sample=%" PRIu32 " RANGE_WARN distance=%.1f cm echo=%" PRId64 " us\n",
                       sample, distance_cm, pulse_width_us);
            } else {
                printf("sample=%" PRIu32 " OK distance=%.1f cm echo=%" PRId64 " us\n",
                       sample, distance_cm, pulse_width_us);
            }
            break;
        case MEASURE_ECHO_STUCK_HIGH:
            printf("sample=%" PRIu32 " ERROR ECHO_STUCK_HIGH (check ECHO wiring)\n",
                   sample);
            break;
        case MEASURE_NO_ECHO:
            printf("sample=%" PRIu32 " TIMEOUT NO_ECHO (check power/TRIG/object)\n",
                   sample);
            break;
        case MEASURE_ECHO_DID_NOT_END:
            printf("sample=%" PRIu32 " TIMEOUT ECHO_DID_NOT_END\n", sample);
            break;
        }
        fflush(stdout);
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_INTERVAL_MS));
    }
}
