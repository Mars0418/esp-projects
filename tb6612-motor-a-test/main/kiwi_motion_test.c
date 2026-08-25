#include <inttypes.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define DRIVER_STBY GPIO_NUM_5

#define PWM_FREQUENCY_HZ 20000
#define PWM_DUTY          205  // 20% of the 10-bit range
#define START_DELAY_MS   3000
#define MOVE_TIME_MS      500
#define STOP_PAUSE_MS    2000
#define ENCODER_SETTLE_MS 200

typedef struct {
    const char *name;
    gpio_num_t pwm_pin;
    gpio_num_t in1_pin;
    gpio_num_t in2_pin;
    gpio_num_t encoder_a_pin;
    gpio_num_t encoder_b_pin;
    ledc_channel_t pwm_channel;
    volatile int32_t encoder_count;
    volatile uint8_t encoder_previous_state;
} motor_t;

enum {
    MOTOR_A = 0,
    MOTOR_B = 1,
    MOTOR_D = 2,
};

static const char *TAG = "kiwi_motion_test";

static motor_t motors[] = {
    {"A", GPIO_NUM_6,  GPIO_NUM_15, GPIO_NUM_7,
          GPIO_NUM_16, GPIO_NUM_17, LEDC_CHANNEL_0, 0, 0},
    {"B", GPIO_NUM_11, GPIO_NUM_9,  GPIO_NUM_10,
          GPIO_NUM_8,  GPIO_NUM_18, LEDC_CHANNEL_1, 0, 0},
    {"D", GPIO_NUM_40, GPIO_NUM_42, GPIO_NUM_41,
          GPIO_NUM_2,  GPIO_NUM_1,  LEDC_CHANNEL_2, 0, 0},
};

static const int8_t quadrature_delta[16] = {
     0,  1, -1,  0,
    -1,  0,  0,  1,
     1,  0,  0, -1,
     0, -1,  1,  0,
};

static uint8_t encoder_read_state(const motor_t *motor)
{
    return ((uint8_t)gpio_get_level(motor->encoder_a_pin) << 1) |
           (uint8_t)gpio_get_level(motor->encoder_b_pin);
}

static void encoder_gpio_isr(void *arg)
{
    motor_t *motor = (motor_t *)arg;
    const uint8_t current_state = encoder_read_state(motor);
    const uint8_t transition =
        (motor->encoder_previous_state << 2) | current_state;

    motor->encoder_count += quadrature_delta[transition];
    motor->encoder_previous_state = current_state;
}

static void configure_output_low(gpio_num_t pin)
{
    ESP_ERROR_CHECK(gpio_reset_pin(pin));
    ESP_ERROR_CHECK(gpio_set_level(pin, 0));
    ESP_ERROR_CHECK(gpio_set_direction(pin, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_level(pin, 0));
}

static void all_motors_safe_stop(void)
{
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

static void motor_prepare(motor_t *motor, int direction, uint32_t duty)
{
    ESP_ERROR_CHECK(gpio_set_level(motor->in1_pin, direction > 0));
    ESP_ERROR_CHECK(gpio_set_level(motor->in2_pin, direction < 0));
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE,
                                  motor->pwm_channel, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE,
                                     motor->pwm_channel));
}

static void encoder_reset_all(void)
{
    for (size_t i = 0; i < sizeof(motors) / sizeof(motors[0]); ++i) {
        ESP_ERROR_CHECK(gpio_intr_disable(motors[i].encoder_a_pin));
        ESP_ERROR_CHECK(gpio_intr_disable(motors[i].encoder_b_pin));
        motors[i].encoder_count = 0;
        motors[i].encoder_previous_state = encoder_read_state(&motors[i]);
        ESP_ERROR_CHECK(gpio_intr_enable(motors[i].encoder_a_pin));
        ESP_ERROR_CHECK(gpio_intr_enable(motors[i].encoder_b_pin));
    }
}

static void log_encoder_counts(const char *label)
{
    ESP_LOGI(TAG, "%s counts: A=%" PRId32 " B=%" PRId32 " D=%" PRId32,
             label,
             motors[MOTOR_A].encoder_count,
             motors[MOTOR_B].encoder_count,
             motors[MOTOR_D].encoder_count);
}

static void run_ad_combination(const char *label, int a_direction,
                               int d_direction)
{
    all_motors_safe_stop();
    encoder_reset_all();

    // Set directions and PWM while the bridge is disabled, then raise STBY.
    motor_prepare(&motors[MOTOR_A], a_direction, PWM_DUTY);
    motor_prepare(&motors[MOTOR_B], 0, 0);
    motor_prepare(&motors[MOTOR_D], d_direction, PWM_DUTY);
    ESP_ERROR_CHECK(gpio_set_level(DRIVER_STBY, 1));

    ESP_LOGI(TAG, "%s: A direction=%d, B stopped, D direction=%d",
             label, a_direction, d_direction);
    vTaskDelay(pdMS_TO_TICKS(MOVE_TIME_MS));

    all_motors_safe_stop();
    vTaskDelay(pdMS_TO_TICKS(ENCODER_SETTLE_MS));
    log_encoder_counts(label);
}

static void hardware_init(void)
{
    configure_output_low(DRIVER_STBY);
    for (size_t i = 0; i < sizeof(motors) / sizeof(motors[0]); ++i) {
        configure_output_low(motors[i].in1_pin);
        configure_output_low(motors[i].in2_pin);
    }

    const ledc_timer_config_t pwm_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = PWM_FREQUENCY_HZ,
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

    uint64_t encoder_pin_mask = 0;
    for (size_t i = 0; i < sizeof(motors) / sizeof(motors[0]); ++i) {
        encoder_pin_mask |= 1ULL << motors[i].encoder_a_pin;
        encoder_pin_mask |= 1ULL << motors[i].encoder_b_pin;
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

    for (size_t i = 0; i < sizeof(motors) / sizeof(motors[0]); ++i) {
        motors[i].encoder_previous_state = encoder_read_state(&motors[i]);
        ESP_ERROR_CHECK(gpio_isr_handler_add(motors[i].encoder_a_pin,
                                              encoder_gpio_isr,
                                              &motors[i]));
        ESP_ERROR_CHECK(gpio_isr_handler_add(motors[i].encoder_b_pin,
                                              encoder_gpio_isr,
                                              &motors[i]));
    }
}

void app_main(void)
{
    hardware_init();

    ESP_LOGI(TAG, "KIWI A/D TRANSLATION TEST starts in %d ms", START_DELAY_MS);
    ESP_LOGI(TAG, "Lift the chassis; 20%% PWM, 500 ms per combination");
    vTaskDelay(pdMS_TO_TICKS(START_DELAY_MS));

    // The wheel-ground directions must oppose geometrically, but the actual
    // motor wiring/mounting determines each software sign. Lifted testing
    // showed that D needs its software sign inverted relative to the initial
    // hypothesis, so A=-1 and D=-1 are the candidate forward command.
    run_ad_combination("Combination 1 (candidate forward)", -1, -1);
    vTaskDelay(pdMS_TO_TICKS(STOP_PAUSE_MS));
    run_ad_combination("Combination 2 (candidate reverse)", +1, +1);

    all_motors_safe_stop();
    ESP_LOGI(TAG, "Motion polarity test complete; SAFE STOP latched until reset");

    while (1) {
        all_motors_safe_stop();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
