idf_component_register(
    SRCS "tour_guide.c" "digit_model.c" "digit_vision.c" "digit_gate.c" "purple_ball_vision.c" "ball_vision.c"
         "black_marker_vision.c" "quarter_goal_pose.c"
         "post_line_odometry.c" "post_line_navigation.c" "line_vision.c"
         "push_debug_stream.c" "main.c" "camera_display.c"
         "camera_line_follow_band.c" "simple_ball_push.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES usb esp_driver_gpio esp_driver_spi esp_driver_ledc
                  esp_driver_uart esp_timer esp_psram
                  usb_host_uvc esp_jpeg mbedtls
)

option(VISION_BENCHMARK "Measure vision throughput with motors disabled" OFF)
option(VISION_BENCHMARK_QVGA "Benchmark a 320x240 camera stream" OFF)
target_compile_definitions(${COMPONENT_LIB} PRIVATE
    SIMPLE_BALL_TEST=$<BOOL:${SIMPLE_BALL_TEST}>
    INTEGRATED_SIMPLE_MISSION=$<BOOL:${INTEGRATED_SIMPLE_MISSION}>
    VISION_BENCHMARK=$<BOOL:${VISION_BENCHMARK}>
    VISION_BENCHMARK_QVGA=$<BOOL:${VISION_BENCHMARK_QVGA}>)
