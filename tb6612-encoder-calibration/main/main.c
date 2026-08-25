#include <inttypes.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define DRIVER_STBY GPIO_NUM_5

typedef struct {
    const char *name;
    gpio_num_t phase_a;
    gpio_num_t phase_b;
    volatile int32_t count;
    volatile uint8_t previous_state;
} encoder_t;

static const char *TAG = "encoder_calibration";

static encoder_t encoders[] = {
    {"A", GPIO_NUM_16, GPIO_NUM_17, 0, 0},
    {"B", GPIO_NUM_8,  GPIO_NUM_18, 0, 0},
    {"D", GPIO_NUM_2,  GPIO_NUM_1,  0, 0},
};

// STBY plus the PWM and direction pins for A, B and D. Keeping every one of
// these pins low makes this a read-only encoder test.
static const gpio_num_t motor_control_pins[] = {
    DRIVER_STBY,
    GPIO_NUM_6,  GPIO_NUM_15, GPIO_NUM_7,
    GPIO_NUM_11, GPIO_NUM_9,  GPIO_NUM_10,
    GPIO_NUM_40, GPIO_NUM_42, GPIO_NUM_41,
};

static const int8_t quadrature_delta[16] = {
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

static void encoder_gpio_isr(void *arg)
{
    encoder_t *encoder = (encoder_t *)arg;
    const uint8_t current_state = encoder_read_state(encoder);
    const uint8_t transition =
        (encoder->previous_state << 2) | current_state;

    encoder->count += quadrature_delta[transition];
    encoder->previous_state = current_state;
}

static void configure_output_low(gpio_num_t pin)
{
    ESP_ERROR_CHECK(gpio_reset_pin(pin));
    ESP_ERROR_CHECK(gpio_set_level(pin, 0));
    ESP_ERROR_CHECK(gpio_set_direction(pin, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_level(pin, 0));
}

static void all_motors_force_disabled(void)
{
    for (size_t i = 0;
         i < sizeof(motor_control_pins) / sizeof(motor_control_pins[0]);
         ++i) {
        ESP_ERROR_CHECK(gpio_set_level(motor_control_pins[i], 0));
    }
}

static void encoders_init(void)
{
    uint64_t encoder_pin_mask = 0;
    for (size_t i = 0; i < sizeof(encoders) / sizeof(encoders[0]); ++i) {
        encoder_pin_mask |= (1ULL << encoders[i].phase_a);
        encoder_pin_mask |= (1ULL << encoders[i].phase_b);
    }

    const gpio_config_t encoder_config = {
        .pin_bit_mask = encoder_pin_mask,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&encoder_config));
    ESP_ERROR_CHECK(gpio_install_isr_service(0));

    for (size_t i = 0; i < sizeof(encoders) / sizeof(encoders[0]); ++i) {
        encoders[i].count = 0;
        encoders[i].previous_state = encoder_read_state(&encoders[i]);
        ESP_ERROR_CHECK(gpio_isr_handler_add(encoders[i].phase_a,
                                              encoder_gpio_isr,
                                              &encoders[i]));
        ESP_ERROR_CHECK(gpio_isr_handler_add(encoders[i].phase_b,
                                              encoder_gpio_isr,
                                              &encoders[i]));
    }
}

void app_main(void)
{
    // Configure STBY first, then every PWM/direction output. The bridge stays
    // disabled for the entire lifetime of this calibration program.
    for (size_t i = 0;
         i < sizeof(motor_control_pins) / sizeof(motor_control_pins[0]);
         ++i) {
        configure_output_low(motor_control_pins[i]);
    }
    all_motors_force_disabled();
    encoders_init();

    ESP_LOGI(TAG, "MANUAL A/B/D ENCODER CALIBRATION MODE");
    ESP_LOGI(TAG, "All motors disabled: STBY, PWM and direction pins are LOW");
    ESP_LOGI(TAG, "A: E1A=GPIO16 E1B=GPIO17");
    ESP_LOGI(TAG, "B: E2A=GPIO8  E2B=GPIO18");
    ESP_LOGI(TAG, "D: E4A=GPIO2  E4B=GPIO1");

    while (1) {
        all_motors_force_disabled();
        ESP_LOGI(TAG, "count A=%" PRId32 " B=%" PRId32 " D=%" PRId32,
                 encoders[0].count, encoders[1].count, encoders[2].count);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
