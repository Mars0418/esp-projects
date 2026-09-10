idf_component_register(
    SRCS "tour_network.c" "xiaozhi_client.c" "tour_start_turn.c" "tour_speech.c" "tour_stations.c" "tour_speech_data.c" "tour_guide.c" "digit_model.c" "digit_vision.c" "digit_gate.c" "purple_ball_vision.c" "ball_vision.c"
         "black_marker_vision.c" "quarter_goal_pose.c"
         "post_line_odometry.c" "post_line_navigation.c" "line_vision.c"
         "push_debug_stream.c" "main.c" "camera_display.c"
         "camera_line_follow_band.c" "simple_ball_push.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES usb esp_driver_gpio esp_driver_spi esp_driver_ledc
                  esp_driver_uart esp_timer esp_psram
                  usb_host_uvc usb_host_uac esp_jpeg mbedtls
                  esp_wifi esp_netif esp_event nvs_flash json esp_http_client esp_websocket_client esp_audio_codec
)

option(VISION_BENCHMARK "Measure vision throughput with motors disabled" OFF)
option(VISION_BENCHMARK_QVGA "Benchmark a 320x240 camera stream" OFF)
target_compile_definitions(${COMPONENT_LIB} PRIVATE
    SIMPLE_BALL_TEST=$<BOOL:${SIMPLE_BALL_TEST}>
    INTEGRATED_SIMPLE_MISSION=$<BOOL:${INTEGRATED_SIMPLE_MISSION}>
    VISION_BENCHMARK=$<BOOL:${VISION_BENCHMARK}>
    VISION_BENCHMARK_QVGA=$<BOOL:${VISION_BENCHMARK_QVGA}>)
