#include "camera_line_follow.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define DRIVER_STBY GPIO_NUM_5

#define MOTOR_A_PWM GPIO_NUM_6
#define MOTOR_A_IN1 GPIO_NUM_15
#define MOTOR_A_IN2 GPIO_NUM_7
#define MOTOR_A_CHANNEL LEDC_CHANNEL_0

#define MOTOR_B_PWM GPIO_NUM_11
#define MOTOR_B_IN1 GPIO_NUM_9
#define MOTOR_B_IN2 GPIO_NUM_10
#define MOTOR_B_CHANNEL LEDC_CHANNEL_1

#define MOTOR_D_PWM GPIO_NUM_40
#define MOTOR_D_IN1 GPIO_NUM_42
#define MOTOR_D_IN2 GPIO_NUM_41
#define MOTOR_D_CHANNEL LEDC_CHANNEL_2

#define PWM_MAX_DUTY 1023
#define FOLLOW_BASE_DUTY 180
#define FOLLOW_LOW_CONFIDENCE_DUTY 150
#define FOLLOW_INNER_MIN_DUTY 100
#define FOLLOW_MAX_DUTY 230
#define FOLLOW_CORRECTION_MAX 50
#define FOLLOW_CORRECTION_STEP 5
#define FOLLOW_ERROR_DEADBAND 45
#define FOLLOW_INTEGRAL_LIMIT 1500
#define FOLLOW_FRAME_TIMEOUT_MS 600
#define FOLLOW_MIN_CONFIDENCE 30
#define STEERING_SIGN -1

typedef struct {
    gpio_num_t pwm;
    gpio_num_t in1;
    gpio_num_t in2;
    ledc_channel_t channel;
} motor_t;

static const char *TAG = "CAMERA_FOLLOW";
static const motor_t s_motor_a = {
    MOTOR_A_PWM, MOTOR_A_IN1, MOTOR_A_IN2, MOTOR_A_CHANNEL};
static const motor_t s_motor_b = {
    MOTOR_B_PWM, MOTOR_B_IN1, MOTOR_B_IN2, MOTOR_B_CHANNEL};
static const motor_t s_motor_d = {
    MOTOR_D_PWM, MOTOR_D_IN1, MOTOR_D_IN2, MOTOR_D_CHANNEL};
static portMUX_TYPE s_result_lock = portMUX_INITIALIZER_UNLOCKED;
static line_vision_result_t s_latest_result;
static int64_t s_latest_frame_us;
static uint32_t s_result_sequence;
static bool s_enabled;
static volatile bool s_debug_enabled;
static char s_uart_line[64];
static size_t s_uart_line_length;

static int clamp_int(int value, int minimum, int maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

static void set_output_low(gpio_num_t pin)
{
    const gpio_config_t config = {
        .pin_bit_mask = 1ULL << pin,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&config));
    ESP_ERROR_CHECK(gpio_set_level(pin, 0));
}

static void motor_prepare(const motor_t *motor, int direction, int duty)
{
    ESP_ERROR_CHECK(gpio_set_level(motor->in1, direction > 0));
    ESP_ERROR_CHECK(gpio_set_level(motor->in2, direction < 0));
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, motor->channel, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, motor->channel));
}

static void stop_motors(void)
{
    motor_prepare(&s_motor_a, 0, 0);
    motor_prepare(&s_motor_b, 0, 0);
    motor_prepare(&s_motor_d, 0, 0);
    ESP_ERROR_CHECK(gpio_set_level(DRIVER_STBY, 0));
}

static void drive_forward(int a_duty, int d_duty)
{
    motor_prepare(&s_motor_a, -1, a_duty);
    motor_prepare(&s_motor_b, 0, 0);
    motor_prepare(&s_motor_d, -1, d_duty);
    ESP_ERROR_CHECK(gpio_set_level(DRIVER_STBY, 1));
}

static void process_uart_line(void)
{
    s_uart_line[s_uart_line_length] = '\0';
    int red;
    int green;
    int blue;
    int enabled;
    if (sscanf(s_uart_line, "RGB,%d,%d,%d", &red, &green, &blue) == 3) {
        red = clamp_int(red, 0, 255);
        green = clamp_int(green, 0, 255);
        blue = clamp_int(blue, 0, 255);
        line_vision_set_rgb_thresholds((uint8_t)red, (uint8_t)green,
                                       (uint8_t)blue);
        ESP_LOGI(TAG, "RGB_THRESHOLDS r=%d g=%d b=%d", red, green, blue);
    } else if (sscanf(s_uart_line, "DEBUG,%d", &enabled) == 1) {
        s_debug_enabled = enabled != 0;
        if (s_debug_enabled) {
            ESP_LOGI(TAG, "RGB_DEBUG enabled=1; normal logs paused");
            esp_log_level_set("*", ESP_LOG_NONE);
        } else {
            esp_log_level_set("*", ESP_LOG_INFO);
            ESP_LOGI(TAG, "RGB_DEBUG enabled=0; normal logs resumed");
        }
    } else if (strcmp(s_uart_line, "STATUS") == 0) {
        const line_vision_rgb_thresholds_t thresholds =
            line_vision_get_rgb_thresholds();
        ESP_LOGI(TAG, "RGB_THRESHOLDS r=%u g=%u b=%u debug=%d",
                 thresholds.red, thresholds.green, thresholds.blue,
                 s_debug_enabled);
    }
    s_uart_line_length = 0;
}

static void handle_uart_command(void)
{
    uint8_t input[64];
    const int count = uart_read_bytes(UART_NUM_0, input, sizeof(input), 0);
    for (int index = 0; index < count; ++index) {
        const uint8_t value = input[index];
        if (value == 'f' || value == 'F') {
            line_vision_result_t result;
            int64_t frame_us;
            portENTER_CRITICAL(&s_result_lock);
            result = s_latest_result;
            frame_us = s_latest_frame_us;
            portEXIT_CRITICAL(&s_result_lock);
            const bool ready = result.found &&
                result.confidence >= FOLLOW_MIN_CONFIDENCE &&
                frame_us > 0 &&
                esp_timer_get_time() - frame_us <=
                    (int64_t)FOLLOW_FRAME_TIMEOUT_MS * 1000;
            if (ready) {
                s_enabled = true;
                ESP_LOGW(TAG, "CAMERA LINE FOLLOW ENABLED");
            } else {
                s_enabled = false;
                stop_motors();
                ESP_LOGW(TAG, "START REFUSED: no fresh, confident line");
            }
        } else if (value == 'x' || value == 'X' || value == ' ') {
            s_enabled = false;
            stop_motors();
            ESP_LOGW(TAG, "CAMERA LINE FOLLOW STOPPED");
        } else if (value == '\r' || value == '\n') {
            if (s_uart_line_length > 0) {
                process_uart_line();
            }
        } else if (value >= 32 && value <= 126) {
            if (s_uart_line_length + 1 < sizeof(s_uart_line)) {
                s_uart_line[s_uart_line_length++] = (char)value;
            } else {
                s_uart_line_length = 0;
            }
        }
    }
}

static void control_task(void *argument)
{
    (void)argument;
    uint32_t processed_sequence = 0;
    int filtered_error = 0;
    int integral_error = 0;
    bool filter_initialized = false;
    int current_base = 0;
    int current_correction = 0;
    int current_a_duty = 0;
    int current_d_duty = 0;
    int64_t last_report_us = 0;
    stop_motors();

    while (true) {
        handle_uart_command();
        line_vision_result_t result;
        int64_t frame_us;
        uint32_t sequence;
        portENTER_CRITICAL(&s_result_lock);
        result = s_latest_result;
        frame_us = s_latest_frame_us;
        sequence = s_result_sequence;
        portEXIT_CRITICAL(&s_result_lock);

        const int64_t now_us = esp_timer_get_time();
        const bool frame_fresh = frame_us > 0 &&
            now_us - frame_us <= (int64_t)FOLLOW_FRAME_TIMEOUT_MS * 1000;
        const bool line_valid = result.found &&
                                result.confidence >= FOLLOW_MIN_CONFIDENCE;
        if (!s_enabled || !frame_fresh || !line_valid) {
            stop_motors();
            filtered_error = 0;
            integral_error = 0;
            filter_initialized = false;
            current_base = 0;
            current_correction = 0;
            current_a_duty = 0;
            current_d_duty = 0;
        } else if (sequence != processed_sequence) {
            const int signed_error = STEERING_SIGN * result.steering_error;
            int previous_filtered_error = filtered_error;
            if (!filter_initialized) {
                filtered_error = signed_error;
                previous_filtered_error = filtered_error;
                filter_initialized = true;
            } else {
                filtered_error =
                    (3 * filtered_error + signed_error) / 4;
            }
            const int derivative = filtered_error - previous_filtered_error;
            const int control_error = abs(filtered_error) <=
                                              FOLLOW_ERROR_DEADBAND
                                          ? 0
                                          : filtered_error;
            if (control_error == 0) {
                integral_error = integral_error * 3 / 4;
            } else {
                if ((control_error > 0 && integral_error < 0) ||
                    (control_error < 0 && integral_error > 0)) {
                    integral_error /= 2;
                }
                integral_error = clamp_int(
                    integral_error + control_error,
                    -FOLLOW_INTEGRAL_LIMIT, FOLLOW_INTEGRAL_LIMIT);
            }
            const int target_correction = clamp_int(
                control_error * 70 / 1000 + integral_error / 1000 +
                    derivative * 3 / 1000,
                -FOLLOW_CORRECTION_MAX, FOLLOW_CORRECTION_MAX);
            const int correction = clamp_int(
                target_correction,
                current_correction - FOLLOW_CORRECTION_STEP,
                current_correction + FOLLOW_CORRECTION_STEP);
            const int confidence_base = result.confidence < 50
                                            ? FOLLOW_LOW_CONFIDENCE_DUTY
                                            : FOLLOW_BASE_DUTY;
            const int turn_slowdown =
                clamp_int(abs(control_error) * 20 / 1000, 0, 20);
            const int base = confidence_base - turn_slowdown;
            const int a_duty = clamp_int(base - correction,
                                         FOLLOW_INNER_MIN_DUTY,
                                         FOLLOW_MAX_DUTY);
            const int d_duty = clamp_int(base + correction,
                                         FOLLOW_INNER_MIN_DUTY,
                                         FOLLOW_MAX_DUTY);
            current_base = base;
            current_correction = correction;
            current_a_duty = a_duty;
            current_d_duty = d_duty;
            drive_forward(a_duty, d_duty);
            processed_sequence = sequence;
        }

        if (now_us - last_report_us >= 500000) {
            ESP_LOGI(TAG,
                     "FOLLOW_STATUS enabled=%d line=%d confidence=%d points=%d lateral=%d heading=%d error=%d filtered=%d integral=%d near=%d far=%d base=%d correction=%d wheel_a=%d wheel_d=%d fresh=%d",
                     s_enabled, result.found, result.confidence,
                     result.vector_point_count,
                     result.lateral_error, result.heading_error,
                     result.steering_error, filtered_error, integral_error,
                     result.near_x, result.far_x,
                     current_base, current_correction,
                     current_a_duty, current_d_duty,
                     frame_fresh);
            last_report_us = now_us;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

esp_err_t camera_line_follow_init(void)
{
    set_output_low(DRIVER_STBY);
    const motor_t *motors[] = {&s_motor_a, &s_motor_b, &s_motor_d};
    for (size_t index = 0; index < 3; ++index) {
        set_output_low(motors[index]->in1);
        set_output_low(motors[index]->in2);
    }

    const ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 20000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_RETURN_ON_ERROR(ledc_timer_config(&timer), TAG, "PWM timer failed");
    for (size_t index = 0; index < 3; ++index) {
        const ledc_channel_config_t channel = {
            .gpio_num = motors[index]->pwm,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = motors[index]->channel,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = LEDC_TIMER_0,
            .duty = 0,
            .hpoint = 0,
        };
        ESP_RETURN_ON_ERROR(ledc_channel_config(&channel), TAG,
                            "PWM channel failed");
    }
    stop_motors();

    const esp_err_t uart_error = uart_driver_install(UART_NUM_0, 1024, 0, 0,
                                                      NULL, 0);
    if (uart_error != ESP_OK && uart_error != ESP_ERR_INVALID_STATE) {
        return uart_error;
    }
    if (xTaskCreatePinnedToCore(control_task, "camera_follow", 4096, NULL,
                                6, NULL, 0) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGW(TAG, "SAFE STOP. F=start X/SPACE=stop RGB,r,g,b DEBUG,0/1");
    return ESP_OK;
}

void camera_line_follow_submit(const line_vision_result_t *result)
{
    portENTER_CRITICAL(&s_result_lock);
    s_latest_result = *result;
    s_latest_frame_us = esp_timer_get_time();
    s_result_sequence++;
    portEXIT_CRITICAL(&s_result_lock);
}

void camera_line_follow_camera_disconnected(void)
{
    portENTER_CRITICAL(&s_result_lock);
    s_latest_frame_us = 0;
    portEXIT_CRITICAL(&s_result_lock);
    s_enabled = false;
    stop_motors();
}

bool camera_line_follow_debug_enabled(void)
{
    return s_debug_enabled;
}
