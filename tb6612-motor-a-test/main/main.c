#include <stdbool.h>
#include <inttypes.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define MOTOR_A_STBY GPIO_NUM_4
#define MOTOR_A_PWM  GPIO_NUM_5
#define MOTOR_A_IN1  GPIO_NUM_7
#define MOTOR_A_IN2  GPIO_NUM_6
#define MOTOR_A_ENC_A GPIO_NUM_15
#define MOTOR_A_ENC_B GPIO_NUM_16

#define MOTOR_PWM_FREQUENCY_HZ 20000
#define MOTOR_PWM_DUTY         205  // 20% of the 10-bit range (0-1023)
#define START_DELAY_MS         3000
#define RUN_TIME_MS             500
#define DIRECTION_PAUSE_MS     1500
#define ENCODER_SETTLE_MS        200

static const char *TAG = "motor_a_test";
static volatile int32_t encoder_count;
static volatile uint8_t encoder_previous_state;

// Valid quadrature transitions add or subtract one. Invalid transitions are
// ignored, which also rejects most contact/noise glitches.
static const int8_t quadrature_delta[16] = {
     0,  1, -1,  0,
    -1,  0,  0,  1,
     1,  0,  0, -1,
     0, -1,  1,  0,
};

static uint8_t encoder_read_state(void)
{
    return ((uint8_t)gpio_get_level(MOTOR_A_ENC_A) << 1) |
           (uint8_t)gpio_get_level(MOTOR_A_ENC_B);
}

static void encoder_gpio_isr(void *arg)
{
    (void)arg;
    const uint8_t current_state = encoder_read_state();
    const uint8_t transition = (encoder_previous_state << 2) | current_state;

    encoder_count += quadrature_delta[transition];
    encoder_previous_state = current_state;
}

static void encoder_init(void)
{
    const gpio_config_t encoder_config = {
        .pin_bit_mask = (1ULL << MOTOR_A_ENC_A) | (1ULL << MOTOR_A_ENC_B),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&encoder_config));

    encoder_previous_state = encoder_read_state();
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(MOTOR_A_ENC_A, encoder_gpio_isr, NULL));
    ESP_ERROR_CHECK(gpio_isr_handler_add(MOTOR_A_ENC_B, encoder_gpio_isr, NULL));
}

static void encoder_reset(void)
{
    ESP_ERROR_CHECK(gpio_intr_disable(MOTOR_A_ENC_A));
    ESP_ERROR_CHECK(gpio_intr_disable(MOTOR_A_ENC_B));
    encoder_count = 0;
    encoder_previous_state = encoder_read_state();
    ESP_ERROR_CHECK(gpio_intr_enable(MOTOR_A_ENC_A));
    ESP_ERROR_CHECK(gpio_intr_enable(MOTOR_A_ENC_B));
}

static void configure_output_low(gpio_num_t pin)
{
    ESP_ERROR_CHECK(gpio_reset_pin(pin));
    ESP_ERROR_CHECK(gpio_set_level(pin, 0));
    ESP_ERROR_CHECK(gpio_set_direction(pin, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_level(pin, 0));
}

static void motor_a_safe_stop(void)
{
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
    ESP_ERROR_CHECK(gpio_set_level(MOTOR_A_STBY, 0));
    ESP_ERROR_CHECK(gpio_set_level(MOTOR_A_IN1, 0));
    ESP_ERROR_CHECK(gpio_set_level(MOTOR_A_IN2, 0));
}

static void motor_a_run(bool reverse)
{
    // Select direction while the H-bridge is disabled. The PWM signal is
    // applied last so direction changes cannot create a sudden current path.
    ESP_ERROR_CHECK(gpio_set_level(MOTOR_A_STBY, 0));
    ESP_ERROR_CHECK(gpio_set_level(MOTOR_A_IN1, reverse ? 0 : 1));
    ESP_ERROR_CHECK(gpio_set_level(MOTOR_A_IN2, reverse ? 1 : 0));
    ESP_ERROR_CHECK(gpio_set_level(MOTOR_A_STBY, 1));
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0,
                                  MOTOR_PWM_DUTY));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
}

void app_main(void)
{
    // Keep STBY low first so the H-bridge remains disabled while the other
    // control pins are configured.
    configure_output_low(MOTOR_A_STBY);
    configure_output_low(MOTOR_A_IN1);
    configure_output_low(MOTOR_A_IN2);

    const ledc_timer_config_t pwm_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = MOTOR_PWM_FREQUENCY_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&pwm_timer));

    const ledc_channel_config_t pwm_channel = {
        .gpio_num = MOTOR_A_PWM,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
        .flags.output_invert = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&pwm_channel));
    motor_a_safe_stop();
    encoder_init();

    ESP_LOGI(TAG, "SAFE STOP active; one-shot test starts in %d ms", START_DELAY_MS);
    ESP_LOGI(TAG, "GPIO4=STBY GPIO5=PWMA GPIO7=AIN1 GPIO6=AIN2");
    ESP_LOGI(TAG, "GPIO15=E1A GPIO16=E1B; quadrature x4 edge counting");
    vTaskDelay(pdMS_TO_TICKS(START_DELAY_MS));

    // One forward/reverse cycle runs after each reset.
    encoder_reset();
    motor_a_run(false);
    ESP_LOGI(TAG, "Motor A forward: 20%% PWM for %d ms", RUN_TIME_MS);

    vTaskDelay(pdMS_TO_TICKS(RUN_TIME_MS));
    motor_a_safe_stop();
    vTaskDelay(pdMS_TO_TICKS(ENCODER_SETTLE_MS));
    const int32_t forward_count = encoder_count;
    ESP_LOGI(TAG, "Encoder forward count: %" PRId32, forward_count);
    ESP_LOGI(TAG, "Motor A stopped; direction-change pause: %d ms",
             DIRECTION_PAUSE_MS);

    vTaskDelay(pdMS_TO_TICKS(DIRECTION_PAUSE_MS));
    encoder_reset();
    motor_a_run(true);
    ESP_LOGI(TAG, "Motor A reverse: 20%% PWM for %d ms", RUN_TIME_MS);

    vTaskDelay(pdMS_TO_TICKS(RUN_TIME_MS));
    motor_a_safe_stop();
    vTaskDelay(pdMS_TO_TICKS(ENCODER_SETTLE_MS));
    const int32_t reverse_count = encoder_count;
    ESP_LOGI(TAG, "Encoder reverse count: %" PRId32, reverse_count);

    if (forward_count == 0 || reverse_count == 0) {
        ESP_LOGW(TAG, "Encoder check FAILED: one or both counts are zero");
    } else if ((forward_count > 0) != (reverse_count > 0)) {
        ESP_LOGI(TAG, "Encoder direction check PASSED: counts have opposite signs");
    } else {
        ESP_LOGW(TAG, "Encoder signals detected, but count signs are not opposite");
    }
    ESP_LOGI(TAG, "Forward/reverse test complete; SAFE STOP latched until reset");

    while (1) {
        motor_a_safe_stop();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
