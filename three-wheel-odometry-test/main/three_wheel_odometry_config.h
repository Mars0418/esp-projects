#pragma once

/*
 * Placeholder geometry for a symmetric 120-degree three-omni-wheel chassis.
 * Initial coordinate system:
 *   +x: right in the top view
 *   +y: forward/up in the top view
 *   positive heading: counter-clockwise from initial +y
 *
 * Position and drive angles are measured counter-clockwise from robot +x. A
 * drive angle describes positive wheel travel after the corresponding encoder
 * sign is applied.
 */

#define ODOM_A_COUNTS_PER_REV 406.0f
#define ODOM_B_COUNTS_PER_REV 406.0f
#define ODOM_D_COUNTS_PER_REV 406.0f

#define ODOM_A_WHEEL_RADIUS_MM 27.5f
#define ODOM_B_WHEEL_RADIUS_MM 27.5f
#define ODOM_D_WHEEL_RADIUS_MM 27.5f

/* Provisional: verify all three signs from signed encoder logs. */
#define ODOM_A_ENCODER_SIGN (-1.0f)
#define ODOM_B_ENCODER_SIGN (-1.0f)
#define ODOM_D_ENCODER_SIGN (-1.0f)

/* Measured vehicle-center to wheel-contact distance. */
#define ODOM_A_POSITION_RADIUS_MM 90.0f
#define ODOM_B_POSITION_RADIUS_MM 90.0f
#define ODOM_D_POSITION_RADIUS_MM 90.0f

/* A=right-front, B=rear, D=left-front. */
#define ODOM_A_POSITION_ANGLE_DEG 30.0f
#define ODOM_B_POSITION_ANGLE_DEG (-90.0f)
#define ODOM_D_POSITION_ANGLE_DEG 150.0f

/* Tangential rolling directions for the placeholder symmetric chassis. */
#define ODOM_A_DRIVE_ANGLE_DEG 120.0f
#define ODOM_B_DRIVE_ANGLE_DEG 0.0f
#define ODOM_D_DRIVE_ANGLE_DEG 240.0f

#define ODOM_UPDATE_INTERVAL_MS 20
#define ODOM_REPORT_INTERVAL_MS 100
