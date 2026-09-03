#pragma once

/* All PWM values use the 10-bit LEDC range 0..1023. */
#define STRAIGHT_CONTROL_INTERVAL_MS          100
#define STRAIGHT_RUN_DURATION_MS             8000
#define STRAIGHT_TARGET_COUNTS_PER_INTERVAL    20

/* Safe fallbacks used before a wheel has been calibrated. */
#define STRAIGHT_FALLBACK_FLOOR_A             140
#define STRAIGHT_FALLBACK_FLOOR_B             180
#define STRAIGHT_FALLBACK_FLOOR_D             140
#define STRAIGHT_PWM_ABOVE_FLOOR               15
#define STRAIGHT_PWM_MAX                       500

/* A/D wheel speed PI and cross-wheel synchronization correction. */
#define STRAIGHT_SPEED_KP                        3
#define STRAIGHT_SPEED_KI_DIV                   10
#define STRAIGHT_SPEED_INTEGRAL_LIMIT          800
#define STRAIGHT_SYNC_KP                         2
#define STRAIGHT_SYNC_CORRECTION_MAX            50

/* B should be stationary during +y travel. It only corrects lateral rolling. */
#define STRAIGHT_B_HOLD_DEADBAND_COUNTS           2
#define STRAIGHT_B_HOLD_KP                       10
#define STRAIGHT_B_HOLD_PWM_MARGIN                5
#define STRAIGHT_B_HOLD_PWM_MAX                 500

/* Stop if either front wheel has no useful encoder movement for this long. */
#define STRAIGHT_MOVING_COUNTS_MIN                2
#define STRAIGHT_STALL_INTERVAL_LIMIT             8

/* G-key position sequence geometry. The 90 mm radius is measured roughly and
 * should be refined from the first physical 120-degree turn test. */
#define MOTION_ENCODER_COUNTS_PER_REV          406.0f
#define MOTION_WHEEL_DIAMETER_MM                55.0f
#define MOTION_FORWARD_DISTANCE_MM             380.0f
#define MOTION_FORWARD_WHEEL_PROJECTION          0.8660254f
#define MOTION_CENTER_TO_WHEEL_MM                90.0f
#define MOTION_RIGHT_TURN_DEG                    120.0f

#define MOTION_FORWARD_TIMEOUT_MS              15000
#define MOTION_TURN_TIMEOUT_MS                 10000
#define MOTION_BRAKE_MS                           80
#define MOTION_SETTLE_MS                         500
#define MOTION_POSITION_TOLERANCE_COUNTS           8
#define MOTION_FORWARD_SLOWDOWN_COUNTS            160
#define MOTION_TURN_SLOWDOWN_COUNTS               100
#define MOTION_MIN_SPEED_COUNTS_PER_INTERVAL        6
#define MOTION_TURN_SPEED_COUNTS_PER_INTERVAL       14
#define MOTION_POSITION_SYNC_DIV                    4
#define MOTION_SYNC_CORRECTION_MAX                 60
#define MOTION_PROGRESS_SPREAD_ABORT              150

/* Loaded breakaway calibration on the floor: every candidate starts at rest. */
#define PWM_CALIBRATION_START                    40
#define PWM_CALIBRATION_MAX                     500
#define PWM_CALIBRATION_STEP                     10
#define PWM_CALIBRATION_DRIVE_MS                300
#define PWM_CALIBRATION_SETTLE_MS               500
#define PWM_CALIBRATION_MIN_COUNTS                4
#define PWM_CALIBRATION_CONFIRM_TRIALS            2
#define PWM_CALIBRATION_ARM_TIMEOUT_MS          5000
