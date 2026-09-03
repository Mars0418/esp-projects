#include "post_line_odometry.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "post_line_odometry_config.h"

#define ODOM_PI 3.14159265358979323846f
#define ODOM_DETERMINANT_EPSILON 0.000001f
#define WHEEL_COUNT 3

typedef struct {
    gpio_num_t phase_a;
    gpio_num_t phase_b;
    volatile int32_t count;
    volatile uint8_t previous_state;
} encoder_t;

typedef struct {
    float counts_per_revolution;
    float wheel_radius_mm;
    float encoder_sign;
    float position_radius_mm;
    float position_angle_deg;
    float drive_angle_deg;
} wheel_geometry_t;

static const wheel_geometry_t s_geometry[WHEEL_COUNT] = {
    {POST_ODOM_A_COUNTS_PER_REV, POST_ODOM_A_WHEEL_RADIUS_MM,
     POST_ODOM_A_ENCODER_SIGN, POST_ODOM_A_POSITION_RADIUS_MM,
     POST_ODOM_A_POSITION_ANGLE_DEG, POST_ODOM_A_DRIVE_ANGLE_DEG},
    {POST_ODOM_B_COUNTS_PER_REV, POST_ODOM_B_WHEEL_RADIUS_MM,
     POST_ODOM_B_ENCODER_SIGN, POST_ODOM_B_POSITION_RADIUS_MM,
     POST_ODOM_B_POSITION_ANGLE_DEG, POST_ODOM_B_DRIVE_ANGLE_DEG},
    {POST_ODOM_D_COUNTS_PER_REV, POST_ODOM_D_WHEEL_RADIUS_MM,
     POST_ODOM_D_ENCODER_SIGN, POST_ODOM_D_POSITION_RADIUS_MM,
     POST_ODOM_D_POSITION_ANGLE_DEG, POST_ODOM_D_DRIVE_ANGLE_DEG},
};

static encoder_t s_encoders[WHEEL_COUNT] = {
    {GPIO_NUM_16, GPIO_NUM_17, 0, 0},
    {GPIO_NUM_8, GPIO_NUM_18, 0, 0},
    {GPIO_NUM_2, GPIO_NUM_1, 0, 0},
};
static const int8_t s_quadrature_delta[16] = {
     0,  1, -1,  0,
    -1,  0,  0,  1,
     1,  0,  0, -1,
     0, -1,  1,  0,
};
static float s_inverse_kinematics[WHEEL_COUNT][WHEEL_COUNT];
static float s_millimeters_per_count[WHEEL_COUNT];
static int32_t s_previous_counts[WHEEL_COUNT];
static post_line_odometry_pose_t s_pose;
static portMUX_TYPE s_pose_lock = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE s_encoder_lock = portMUX_INITIALIZER_UNLOCKED;

static uint8_t encoder_read_state(const encoder_t *encoder)
{
    return ((uint8_t)gpio_get_level(encoder->phase_a) << 1) |
           (uint8_t)gpio_get_level(encoder->phase_b);
}

static void encoder_gpio_isr(void *argument)
{
    encoder_t *encoder = (encoder_t *)argument;
    const uint8_t current_state = encoder_read_state(encoder);
    portENTER_CRITICAL_ISR(&s_encoder_lock);
    const uint8_t transition =
        (uint8_t)((encoder->previous_state << 2) | current_state);
    encoder->count += s_quadrature_delta[transition];
    encoder->previous_state = current_state;
    portEXIT_CRITICAL_ISR(&s_encoder_lock);
}

static void read_encoder_counts(int32_t *count_a, int32_t *count_b,
                                int32_t *count_d)
{
    portENTER_CRITICAL(&s_encoder_lock);
    *count_a = s_encoders[0].count;
    *count_b = s_encoders[1].count;
    *count_d = s_encoders[2].count;
    portEXIT_CRITICAL(&s_encoder_lock);
}

static esp_err_t configure_encoders(void)
{
    uint64_t pin_mask = 0;
    for (size_t index = 0; index < WHEEL_COUNT; ++index) {
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
    esp_err_t error = gpio_config(&config);
    if (error != ESP_OK) return error;
    error = gpio_install_isr_service(0);
    if (error != ESP_OK && error != ESP_ERR_INVALID_STATE) return error;

    for (size_t index = 0; index < WHEEL_COUNT; ++index) {
        s_encoders[index].previous_state =
            encoder_read_state(&s_encoders[index]);
        error = gpio_isr_handler_add(s_encoders[index].phase_a,
                                     encoder_gpio_isr,
                                     &s_encoders[index]);
        if (error != ESP_OK) return error;
        error = gpio_isr_handler_add(s_encoders[index].phase_b,
                                     encoder_gpio_isr,
                                     &s_encoders[index]);
        if (error != ESP_OK) return error;
    }
    return ESP_OK;
}

static float degrees_to_radians(float degrees)
{
    return degrees * ODOM_PI / 180.0f;
}

static float normalize_heading(float radians)
{
    while (radians > ODOM_PI) radians -= 2.0f * ODOM_PI;
    while (radians <= -ODOM_PI) radians += 2.0f * ODOM_PI;
    return radians;
}

static bool invert_3x3(const float matrix[3][3], float inverse[3][3])
{
    const float determinant =
        matrix[0][0] * (matrix[1][1] * matrix[2][2] -
                        matrix[1][2] * matrix[2][1]) -
        matrix[0][1] * (matrix[1][0] * matrix[2][2] -
                        matrix[1][2] * matrix[2][0]) +
        matrix[0][2] * (matrix[1][0] * matrix[2][1] -
                        matrix[1][1] * matrix[2][0]);
    if (fabsf(determinant) < ODOM_DETERMINANT_EPSILON) return false;

    const float reciprocal = 1.0f / determinant;
    inverse[0][0] = (matrix[1][1] * matrix[2][2] -
                     matrix[1][2] * matrix[2][1]) * reciprocal;
    inverse[0][1] = (matrix[0][2] * matrix[2][1] -
                     matrix[0][1] * matrix[2][2]) * reciprocal;
    inverse[0][2] = (matrix[0][1] * matrix[1][2] -
                     matrix[0][2] * matrix[1][1]) * reciprocal;
    inverse[1][0] = (matrix[1][2] * matrix[2][0] -
                     matrix[1][0] * matrix[2][2]) * reciprocal;
    inverse[1][1] = (matrix[0][0] * matrix[2][2] -
                     matrix[0][2] * matrix[2][0]) * reciprocal;
    inverse[1][2] = (matrix[0][2] * matrix[1][0] -
                     matrix[0][0] * matrix[1][2]) * reciprocal;
    inverse[2][0] = (matrix[1][0] * matrix[2][1] -
                     matrix[1][1] * matrix[2][0]) * reciprocal;
    inverse[2][1] = (matrix[0][1] * matrix[2][0] -
                     matrix[0][0] * matrix[2][1]) * reciprocal;
    inverse[2][2] = (matrix[0][0] * matrix[1][1] -
                     matrix[0][1] * matrix[1][0]) * reciprocal;
    return true;
}

static int32_t signed_count_delta(int32_t current, int32_t previous)
{
    return (int32_t)((uint32_t)current - (uint32_t)previous);
}

static void update_pose(int32_t count_a, int32_t count_b, int32_t count_d)
{
    const int32_t current_counts[WHEEL_COUNT] = {
        count_a, count_b, count_d};
    float wheel_distance_mm[WHEEL_COUNT];
    for (size_t wheel = 0; wheel < WHEEL_COUNT; ++wheel) {
        const int32_t delta = signed_count_delta(
            current_counts[wheel], s_previous_counts[wheel]);
        s_previous_counts[wheel] = current_counts[wheel];
        wheel_distance_mm[wheel] =
            delta * s_millimeters_per_count[wheel];
    }

    float body_delta[WHEEL_COUNT] = {0};
    for (size_t row = 0; row < WHEEL_COUNT; ++row) {
        for (size_t column = 0; column < WHEEL_COUNT; ++column) {
            body_delta[row] +=
                s_inverse_kinematics[row][column] *
                wheel_distance_mm[column];
        }
    }

    portENTER_CRITICAL(&s_pose_lock);
    const float heading_mid = s_pose.heading_rad + body_delta[2] * 0.5f;
    const float cosine = cosf(heading_mid);
    const float sine = sinf(heading_mid);
    s_pose.x_mm += cosine * body_delta[0] - sine * body_delta[1];
    s_pose.y_mm += sine * body_delta[0] + cosine * body_delta[1];
    s_pose.heading_rad = normalize_heading(
        s_pose.heading_rad + body_delta[2]);
    s_pose.heading_deg = s_pose.heading_rad * 180.0f / ODOM_PI;
    s_pose.count_a = count_a;
    s_pose.count_b = count_b;
    s_pose.count_d = count_d;
    s_pose.valid = true;
    portEXIT_CRITICAL(&s_pose_lock);
}

static void odometry_task(void *argument)
{
    (void)argument;
    TickType_t last_wake = xTaskGetTickCount();
    while (true) {
        int32_t count_a, count_b, count_d;
        read_encoder_counts(&count_a, &count_b, &count_d);
        update_pose(count_a, count_b, count_d);
        vTaskDelayUntil(&last_wake,
                        pdMS_TO_TICKS(POST_ODOM_UPDATE_INTERVAL_MS));
    }
}

esp_err_t post_line_odometry_init(void)
{
    esp_err_t error = configure_encoders();
    if (error != ESP_OK) return error;

    float kinematics[WHEEL_COUNT][WHEEL_COUNT] = {0};
    for (size_t wheel = 0; wheel < WHEEL_COUNT; ++wheel) {
        if (s_geometry[wheel].counts_per_revolution <= 0.0f ||
            s_geometry[wheel].wheel_radius_mm <= 0.0f ||
            s_geometry[wheel].position_radius_mm <= 0.0f ||
            s_geometry[wheel].encoder_sign == 0.0f) {
            return ESP_ERR_INVALID_ARG;
        }
        const float position_angle =
            degrees_to_radians(s_geometry[wheel].position_angle_deg);
        const float drive_angle =
            degrees_to_radians(s_geometry[wheel].drive_angle_deg);
        const float position_x =
            s_geometry[wheel].position_radius_mm * cosf(position_angle);
        const float position_y =
            s_geometry[wheel].position_radius_mm * sinf(position_angle);
        const float drive_x = cosf(drive_angle);
        const float drive_y = sinf(drive_angle);
        kinematics[wheel][0] = drive_x;
        kinematics[wheel][1] = drive_y;
        kinematics[wheel][2] =
            -position_y * drive_x + position_x * drive_y;
        s_millimeters_per_count[wheel] =
            s_geometry[wheel].encoder_sign * 2.0f * ODOM_PI *
            s_geometry[wheel].wheel_radius_mm /
            s_geometry[wheel].counts_per_revolution;
    }
    if (!invert_3x3(kinematics, s_inverse_kinematics)) {
        return ESP_ERR_INVALID_STATE;
    }

    int32_t count_a, count_b, count_d;
    read_encoder_counts(&count_a, &count_b, &count_d);
    s_previous_counts[0] = count_a;
    s_previous_counts[1] = count_b;
    s_previous_counts[2] = count_d;
    portENTER_CRITICAL(&s_pose_lock);
    memset(&s_pose, 0, sizeof(s_pose));
    s_pose.count_a = count_a;
    s_pose.count_b = count_b;
    s_pose.count_d = count_d;
    s_pose.valid = true;
    portEXIT_CRITICAL(&s_pose_lock);

    if (xTaskCreatePinnedToCore(odometry_task, "post_odom", 4096, NULL,
                                4, NULL, 0) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

bool post_line_odometry_get_pose(post_line_odometry_pose_t *pose)
{
    if (!pose) return false;
    portENTER_CRITICAL(&s_pose_lock);
    *pose = s_pose;
    portEXIT_CRITICAL(&s_pose_lock);
    return pose->valid;
}
