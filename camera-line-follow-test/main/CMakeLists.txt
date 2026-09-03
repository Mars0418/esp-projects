idf_component_register(
    SRCS "black_marker_vision.c" "quarter_goal_pose.c"
         "post_line_odometry.c" "line_vision.c"
         "main.c" "camera_display.c"
         "camera_line_follow_band.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES usb esp_driver_gpio esp_driver_spi esp_driver_ledc
                  esp_driver_uart esp_timer esp_psram
                  usb_host_uvc esp_jpeg mbedtls
)
