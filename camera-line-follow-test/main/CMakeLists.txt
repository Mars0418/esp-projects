idf_component_register(
    SRCS "ball_vision.c" "white_ball_vision.c"
         "black_marker_vision.c" "quarter_goal_pose.c"
         "post_line_odometry.c" "post_line_navigation.c" "line_vision.c"
         "main.c" "camera_display.c"
         "camera_line_follow_band.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES usb esp_driver_gpio esp_driver_spi esp_driver_ledc
                  esp_driver_uart esp_timer esp_psram
                  usb_host_uvc esp_jpeg mbedtls
)
