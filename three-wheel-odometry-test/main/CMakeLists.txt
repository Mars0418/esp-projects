idf_component_register(
    SRCS "main.c" "three_wheel_odometry.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_gpio esp_timer
)
