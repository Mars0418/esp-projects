#include <inttypes.h>
#include <math.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "three_wheel_odometry.h"
#include "three_wheel_odometry_config.h"

#define DRIVER_STBY GPIO_NUM_5

typedef struct {
    gpio_num_t phase_a;
    gpio_num_t phase_b;
    volatile int32_t count;
    volatile uint8_t previous_state;
} encoder_t;

static const char *TAG = "three_wheel_odometry";

static encoder_t s_encoders[] = {
    {GPIO_NUM_16, GPIO_NUM_17, 0, 0},
    {GPIO_NUM_8, GPIO_NUM_18, 0, 0},
    {GPIO_NUM_2, GPIO_NUM_1, 0, 0},
};

static const gpio_num_t s_motor_control_pins[] = {
    DRIVER_STBY,
    GPIO_NUM_6, GPIO_NUM_15, GPIO_NUM_7,
    GPIO_NUM_11, GPIO_NUM_9, GPIO_NUM_10,
    GPIO_NUM_40, GPIO_NUM_42, GPIO_NUM_41,
};

static const int8_t s_quadrature_delta[16] = {
     0,  1, -1,  0,
    -1,  0,  0,  1,
     1,  0,  0, -1,
     0, -1,  1,  0,
};

static uint8_t encoder_read_state(const encoder_t *encoder)
{
    return ((uint8_t)gpio_get_level(encoder->phase_a) << 1) |
           (uint8_t)gpio_get_level(encoder->phase_b);
}

static void encoder_gpio_isr(void *argument)
{
    encoder_t *encoder = (encoder_t *)argument;
    const uint8_t current_state = encoder_read_state(encoder);
    const uint8_t transition =
        (uint8_t)((encoder->previous_state << 2) | current_state);
    encoder->count += s_quadrature_delta[transition];
    encoder->previous_state = current_state;
}

static void configure_output_low(gpio_num_t pin)
{
    ESP_ERROR_CHECK(gpio_reset_pin(pin));
    ESP_ERROR_CHECK(gpio_set_level(pin, 0));
    ESP_ERROR_CHECK(gpio_set_direction(pin, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_level(pin, 0));
}

static void force_motors_disabled(void)
{
    for (size_t index = 0;
         index < sizeof(s_motor_control_pins) /
                     sizeof(s_motor_control_pins[0]);
         ++index) {
        ESP_ERROR_CHECK(gpio_set_level(s_motor_control_pins[index], 0));
    }
}

static void configure_encoders(void)
{
    uint64_t pin_mask = 0;
    for (size_t index = 0;
         index < sizeof(s_encoders) / sizeof(s_encoders[0]); ++index) {
        pin_mask |= 1ULL << s_encoders[index].phase_a;
        pin_mask |= 1ULL << s_encoders[index].phase_b;
    }

    const gpio_config_t config = {
        .pin_bit_mask = pin_mask,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&config));
    ESP_ERROR_CHECK(gpio_install_isr_service(0));

    for (size_t index = 0;
         index < sizeof(s_encoders) / sizeof(s_encoders[0]); ++index) {
        s_encoders[index].previous_state =
            encoder_read_state(&s_encoders[index]);
        ESP_ERROR_CHECK(gpio_isr_handler_add(
            s_encoders[index].phase_a, encoder_gpio_isr, &s_encoders[index]));
        ESP_ERROR_CHECK(gpio_isr_handler_add(
            s_encoders[index].phase_b, encoder_gpio_isr, &s_encoders[index]));
    }
}

void app_main(void)
{
    for (size_t index = 0;
         index < sizeof(s_motor_control_pins) /
                     sizeof(s_motor_control_pins[0]);
         ++index) {
        configure_output_low(s_motor_control_pins[index]);
    }
    force_motors_disabled();
    configure_encoders();

    three_wheel_odometry_t odometry;
    ESP_ERROR_CHECK(three_wheel_odometry_init(
        &odometry, s_encoders[0].count, s_encoders[1].count,
        s_encoders[2].count));

    ESP_LOGW(TAG, "READ-ONLY TEST: all motor control pins remain LOW");
    ESP_LOGW(TAG, "Geometry and encoder signs are uncalibrated placeholders");
    ESP_LOGI(TAG, "Origin=(0,0), +x=right, +y=up/initial heading, CCW=positive");

    three_wheel_pose_t pose = {0};
    int64_t last_report_us = 0;
    while (true) {
        force_motors_disabled();
        three_wheel_odometry_update(
            &odometry, s_encoders[0].count, s_encoders[1].count,
            s_encoders[2].count, &pose);

        const int64_t now_us = esp_timer_get_time();
        if (now_us - last_report_us >=
            (int64_t)ODOM_REPORT_INTERVAL_MS * 1000) {
            ESP_LOGI(TAG,
                     "ODOM x_mm=%ld y_mm=%ld heading_mdeg=%ld countA=%" PRId32
                     " countB=%" PRId32 " countD=%" PRId32,
                     (long)lroundf(pose.x_mm), (long)lroundf(pose.y_mm),
                     (long)lroundf(pose.heading_deg * 1000.0f),
                     s_encoders[0].count, s_encoders[1].count,
                     s_encoders[2].count);
            last_report_us = now_us;
        }
        vTaskDelay(pdMS_TO_TICKS(ODOM_UPDATE_INTERVAL_MS));
    }
}
