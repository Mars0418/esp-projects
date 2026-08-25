#pragma once

#include "driver/gpio.h"

/*
 * Only the on-board RGB LED pin is confirmed by the working course example.
 * Replace GPIO_NUM_NC values using the car baseboard wiring table before
 * testing external modules. Never guess motor-driver pins.
 */

#define CAR_LED_GPIO               GPIO_NUM_38

#define CAR_LINE_LEFT_OUTER_GPIO   GPIO_NUM_NC
#define CAR_LINE_LEFT_INNER_GPIO   GPIO_NUM_NC
#define CAR_LINE_RIGHT_INNER_GPIO  GPIO_NUM_NC
#define CAR_LINE_RIGHT_OUTER_GPIO  GPIO_NUM_NC

#define CAR_ULTRASONIC_TRIG_GPIO   GPIO_NUM_NC
#define CAR_ULTRASONIC_ECHO_GPIO   GPIO_NUM_NC
#define CAR_DHT11_GPIO             GPIO_NUM_NC

#define CAR_I2C_SDA_GPIO           GPIO_NUM_NC
#define CAR_I2C_SCL_GPIO           GPIO_NUM_NC
#define CAR_MPU6500_I2C_ADDRESS    0x68

#define CAR_TFT_MOSI_GPIO          GPIO_NUM_NC
#define CAR_TFT_SCLK_GPIO          GPIO_NUM_NC
#define CAR_TFT_CS_GPIO            GPIO_NUM_NC
#define CAR_TFT_DC_GPIO            GPIO_NUM_NC
#define CAR_TFT_RST_GPIO           GPIO_NUM_NC
#define CAR_TFT_BACKLIGHT_GPIO     GPIO_NUM_NC
#define CAR_TFT_WIDTH              240
#define CAR_TFT_HEIGHT             240
#define CAR_TFT_X_GAP              0
#define CAR_TFT_Y_GAP              0

#define CAR_MOTOR_STBY_GPIO        GPIO_NUM_NC
#define CAR_MOTOR_A_IN1_GPIO       GPIO_NUM_NC
#define CAR_MOTOR_A_IN2_GPIO       GPIO_NUM_NC
#define CAR_MOTOR_A_PWM_GPIO       GPIO_NUM_NC
#define CAR_MOTOR_B_IN1_GPIO       GPIO_NUM_NC
#define CAR_MOTOR_B_IN2_GPIO       GPIO_NUM_NC
#define CAR_MOTOR_B_PWM_GPIO       GPIO_NUM_NC

#define CAR_ENCODER_A_PHASE_A_GPIO GPIO_NUM_NC
#define CAR_ENCODER_A_PHASE_B_GPIO GPIO_NUM_NC
#define CAR_ENCODER_B_PHASE_A_GPIO GPIO_NUM_NC
#define CAR_ENCODER_B_PHASE_B_GPIO GPIO_NUM_NC

#define CAR_SERVO_GPIO             GPIO_NUM_NC
