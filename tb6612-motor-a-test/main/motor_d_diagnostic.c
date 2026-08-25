#include <inttypes.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define DRIVER_STBY GPIO_NUM_5

#define MOTOR_D_PWM  GPIO_NUM_40
#define MOTOR_D_IN1  GPIO_NUM_42
#define MOTOR_D_IN2  GPIO_NUM_41
#define MOTOR_D_ENC_A GPIO_NUM_2
#define MOTOR_D_ENC_B GPIO_NUM_1

#define PWM_FREQUENCY_HZ 20000
#define PWM_DUTY          307  // 30% of 0..1023
#define START_DELAY_MS   3000
#define RUN_TIME_MS      1000
#define DIRECTION_PAUSE_MS 1500
#define ENCODER_SETTLE_MS  200

static const char *TAG = "motor_d_diagnostic";
static volatile int32_t encoder_count;
static volatile uint8_t encoder_previous_state;

// A and B must remain inactive whenever the shared STBY line is raised for D.
static const gpio_num_t inactive_ab_control_pins[] = {
    GPIO_NUM_6, GPIO_NUM_15, GPIO_NUM_7,
    GPIO_NUM_11, GPIO_NUM_9, GPIO_NUM_10,
};

static const int8_t quadrature_delta[16] = {
     0,  1, -1,  0,
    -1,  0,  0,  1,
     1,  0,  0, -1,
     0, -1,  1,  0,
};

static uint8_t encoder_read_state(void)
{
    return ((uint8_t)gpio_get_level(MOTOR_D_ENC_A) << 1) |
           (uint8_t)gpio_get_level(MOTOR_D_ENC_B);
}

static void encoder_gpio_isr(void *arg)
{
    (void)arg;
    const uint8_t current_state = encoder_read_state();
    const uint8_t transition =
        (encoder_previous_state << 2) | current_state;

    encoder_count += quadrature_delta[transition];
    encoder_previous_state = current_state;
}

static void configure_output_low(gpio_num_t pin)
{
    ESP_ERROR_CHECK(gpio_reset_pin(pin));
    ESP_ERROR_CHECK(gpio_set_level(pin, 0));
    ESP_ERROR_CHECK(gpio_set_direction(pin, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_level(pin, 0));
}

static void motor_d_safe_stop(void)
{
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
    ESP_ERROR_CHECK(gpio_set_level(DRIVER_STBY, 0));
    ESP_ERROR_CHECK(gpio_set_level(MOTOR_D_IN1, 0));
    ESP_ERROR_CHECK(gpio_set_level(MOTOR_D_IN2, 0));

    for (size_t i = 0;
         i < sizeof(inactive_ab_control_pins) /
                 sizeof(inactive_ab_control_pins[0]);
         ++i) {
        ESP_ERROR_CHECK(gpio_set_level(inactive_ab_control_pins[i], 0));
    }
}

static void encoder_reset(void)
{
    ESP_ERROR_CHECK(gpio_intr_disable(MOTOR_D_ENC_A));
    ESP_ERROR_CHECK(gpio_intr_disable(MOTOR_D_ENC_B));
    encoder_count = 0;
    encoder_previous_state = encoder_read_state();
    ESP_ERROR_CHECK(gpio_intr_enable(MOTOR_D_ENC_A));
    ESP_ERROR_CHECK(gpio_intr_enable(MOTOR_D_ENC_B));
}

static void motor_d_run(int direction)
{
    motor_d_safe_stop();
    encoder_reset();

    ESP_ERROR_CHECK(gpio_set_level(MOTOR_D_IN1, direction > 0));
    ESP_ERROR_CHECK(gpio_set_level(MOTOR_D_IN2, direction < 0));
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0,
                                  PWM_DUTY));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
    ESP_ERROR_CHECK(gpio_set_level(DRIVER_STBY, 1));

    ESP_LOGI(TAG, "D direction=%d, 30%% PWM for %d ms",
             direction, RUN_TIME_MS);
    vTaskDelay(pdMS_TO_TICKS(RUN_TIME_MS));

    motor_d_safe_stop();
    vTaskDelay(pdMS_TO_TICKS(ENCODER_SETTLE_MS));
    ESP_LOGI(TAG, "D direction=%d encoder_count=%" PRId32,
             direction, encoder_count);
}

void app_main(void)
{
    configure_output_low(DRIVER_STBY);
    configure_output_low(MOTOR_D_IN1);
    configure_output_low(MOTOR_D_IN2);
    for (size_t i = 0;
         i < sizeof(inactive_ab_control_pins) /
                 sizeof(inactive_ab_control_pins[0]);
         ++i) {
        configure_output_low(inactive_ab_control_pins[i]);
    }

    const ledc_timer_config_t pwm_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = PWM_FREQUENCY_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&pwm_timer));

    const ledc_channel_config_t pwm_channel = {
        .gpio_num = MOTOR_D_PWM,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
        .flags.output_invert = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&pwm_channel));
    motor_d_safe_stop();

    const gpio_config_t encoder_config = {
        .pin_bit_mask = (1ULL << MOTOR_D_ENC_A) |
                        (1ULL << MOTOR_D_ENC_B),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&encoder_config));
    encoder_previous_state = encoder_read_state();
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(MOTOR_D_ENC_A,
                                          encoder_gpio_isr, NULL));
    ESP_ERROR_CHECK(gpio_isr_handler_add(MOTOR_D_ENC_B,
                                          encoder_gpio_isr, NULL));

    ESP_LOGI(TAG, "D-ONLY TEST starts in %d ms", START_DELAY_MS);
    ESP_LOGI(TAG, "PWMD=40 DIN1=42 DIN2=41 E4A=2 E4B=1");
    vTaskDelay(pdMS_TO_TICKS(START_DELAY_MS));

    motor_d_run(+1);
    vTaskDelay(pdMS_TO_TICKS(DIRECTION_PAUSE_MS));
    motor_d_run(-1);

    motor_d_safe_stop();
    ESP_LOGI(TAG, "D-only diagnostic complete; SAFE STOP latched until reset");

    while (1) {
        motor_d_safe_stop();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
