set(wifi_debug_sources "")
set(wifi_debug_assets "")
if(CONFIG_CAR_WIFI_DEBUG)
    list(APPEND wifi_debug_sources "wifi_debug.c" "usb_av_probe.c" "usb_audio.c" "xiaozhi_client.c" "xiaozhi_wake.c")
    list(APPEND wifi_debug_assets "wifi_debug.html")
endif()

idf_component_register(
    SRCS "purple_ball_vision.c" "ball_vision.c" "white_ball_vision.c"
         "black_marker_vision.c" "quarter_goal_pose.c"
         "post_line_odometry.c" "post_line_navigation.c" "line_vision.c"
         "main.c" "camera_display.c"
         "camera_line_follow_band.c"
         ${wifi_debug_sources}
    INCLUDE_DIRS "."
    PRIV_REQUIRES usb esp_driver_gpio esp_driver_spi esp_driver_ledc
                  esp_driver_uart esp_timer esp_psram
                  usb_host_uvc usb_host_uac esp_jpeg mbedtls
                  esp_wifi esp_netif esp_event esp_http_server nvs_flash json
                  esp_http_client esp_websocket_client esp_audio_codec esp-sr
    EMBED_TXTFILES ${wifi_debug_assets}
)
