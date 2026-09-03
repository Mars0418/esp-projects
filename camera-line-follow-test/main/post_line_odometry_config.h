#pragma once

/* Measured chassis values. Effective rolling radii may still need a
 * floor-test correction for tyre compression and slip. */
#define POST_ODOM_A_COUNTS_PER_REV 406.0f
#define POST_ODOM_B_COUNTS_PER_REV 406.0f
#define POST_ODOM_D_COUNTS_PER_REV 406.0f

#define POST_ODOM_A_WHEEL_RADIUS_MM 27.5f
#define POST_ODOM_B_WHEEL_RADIUS_MM 27.5f
#define POST_ODOM_D_WHEEL_RADIUS_MM 27.5f

/* Provisional: verify from signed logs before using absolute pose. */
#define POST_ODOM_A_ENCODER_SIGN (-1.0f)
#define POST_ODOM_B_ENCODER_SIGN (-1.0f)
#define POST_ODOM_D_ENCODER_SIGN (-1.0f)

#define POST_ODOM_A_POSITION_RADIUS_MM 90.0f
#define POST_ODOM_B_POSITION_RADIUS_MM 90.0f
#define POST_ODOM_D_POSITION_RADIUS_MM 90.0f

/* Provisional ideal 120-degree geometry. Angles are counter-clockwise from
 * vehicle +x (right); A=right-front, B=rear, D=left-front. */
#define POST_ODOM_A_POSITION_ANGLE_DEG 30.0f
#define POST_ODOM_B_POSITION_ANGLE_DEG (-90.0f)
#define POST_ODOM_D_POSITION_ANGLE_DEG 150.0f

#define POST_ODOM_A_DRIVE_ANGLE_DEG 120.0f
#define POST_ODOM_B_DRIVE_ANGLE_DEG 0.0f
#define POST_ODOM_D_DRIVE_ANGLE_DEG 240.0f

#define POST_ODOM_UPDATE_INTERVAL_MS 20
#define POST_ODOM_REPORT_INTERVAL_MS 200
