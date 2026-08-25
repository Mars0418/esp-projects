#include <stddef.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define DRIVER_STBY GPIO_NUM_5

#define MOTOR_PWM_FREQUENCY_HZ 20000
#define MOTOR_PWM_DUTY         205  // 20% of the 10-bit range (0-1023)
#define START_DELAY_MS         3000
#define RUN_TIME_MS             500
#define BETWEEN_MOTORS_MS      1500

typedef struct {
    const char *name;
    gpio_num_t pwm_pin;
    gpio_num_t in1_pin;
    gpio_num_t in2_pin;
    ledc_channel_t pwm_channel;
} motor_t;

static const char *TAG = "three_motor_test";

static const motor_t motors[] = {
    {"Motor A", GPIO_NUM_6,  GPIO_NUM_15, GPIO_NUM_7,  LEDC_CHANNEL_0},
    {"Motor B", GPIO_NUM_11, GPIO_NUM_9,  GPIO_NUM_10, LEDC_CHANNEL_1},
    {"Motor D", GPIO_NUM_40, GPIO_NUM_42, GPIO_NUM_41, LEDC_CHANNEL_2},
};

static void configure_output_low(gpio_num_t pin)
{
    ESP_ERROR_CHECK(gpio_reset_pin(pin));
    ESP_ERROR_CHECK(gpio_set_level(pin, 0));
    ESP_ERROR_CHECK(gpio_set_direction(pin, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_level(pin, 0));
}

static void all_motors_safe_stop(void)
{
    // Remove PWM before disabling the bridge and clearing every direction pin.
    for (size_t i = 0; i < sizeof(motors) / sizeof(motors[0]); ++i) {
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE,
                                      motors[i].pwm_channel, 0));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE,
                                         motors[i].pwm_channel));
    }

    ESP_ERROR_CHECK(gpio_set_level(DRIVER_STBY, 0));
    for (size_t i = 0; i < sizeof(motors) / sizeof(motors[0]); ++i) {
        ESP_ERROR_CHECK(gpio_set_level(motors[i].in1_pin, 0));
        ESP_ERROR_CHECK(gpio_set_level(motors[i].in2_pin, 0));
    }
}

static void test_one_motor(const motor_t *motor)
{
    all_motors_safe_stop();

    // Choose direction while STBY is low, then enable the bridge and PWM last.
    ESP_ERROR_CHECK(gpio_set_level(motor->in1_pin, 1));
    ESP_ERROR_CHECK(gpio_set_level(motor->in2_pin, 0));
    ESP_ERROR_CHECK(gpio_set_level(DRIVER_STBY, 1));
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE,
                                  motor->pwm_channel, MOTOR_PWM_DUTY));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE,
                                     motor->pwm_channel));

    ESP_LOGI(TAG, "%s RUN: 20%% PWM for %d ms", motor->name, RUN_TIME_MS);
    vTaskDelay(pdMS_TO_TICKS(RUN_TIME_MS));

    all_motors_safe_stop();
    ESP_LOGI(TAG, "%s STOP", motor->name);
    vTaskDelay(pdMS_TO_TICKS(BETWEEN_MOTORS_MS));
}

void app_main(void)
{
    // Configure STBY first so the complete driver remains disabled while all
    // other control outputs and PWM channels are initialized.
    configure_output_low(DRIVER_STBY);
    for (size_t i = 0; i < sizeof(motors) / sizeof(motors[0]); ++i) {
        configure_output_low(motors[i].in1_pin);
        configure_output_low(motors[i].in2_pin);
    }

    const ledc_timer_config_t pwm_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = MOTOR_PWM_FREQUENCY_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&pwm_timer));

    for (size_t i = 0; i < sizeof(motors) / sizeof(motors[0]); ++i) {
        const ledc_channel_config_t pwm_channel = {
            .gpio_num = motors[i].pwm_pin,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = motors[i].pwm_channel,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = LEDC_TIMER_0,
            .duty = 0,
            .hpoint = 0,
            .flags.output_invert = 0,
        };
        ESP_ERROR_CHECK(ledc_channel_config(&pwm_channel));
    }
    all_motors_safe_stop();

    ESP_LOGI(TAG, "SAFE STOP active; A/B/D sequence starts in %d ms",
             START_DELAY_MS);
    ESP_LOGI(TAG, "STBY=5");
    ESP_LOGI(TAG, "A: PWM6 IN1=15 IN2=7");
    ESP_LOGI(TAG, "B: PWM11 IN1=9 IN2=10");
    ESP_LOGI(TAG, "D: PWM40 IN1=42 IN2=41");
    vTaskDelay(pdMS_TO_TICKS(START_DELAY_MS));

    for (size_t i = 0; i < sizeof(motors) / sizeof(motors[0]); ++i) {
        test_one_motor(&motors[i]);
    }

    all_motors_safe_stop();
    ESP_LOGI(TAG, "A/B/D test complete; SAFE STOP latched until reset");

    while (1) {
        all_motors_safe_stop();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
