idf_component_register(
    SRCS "main.c" "camera_display.c" "digit_model.c" "digit_vision.c"
         "digit_gate.c" "digit_runtime.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES usb esp_driver_gpio esp_driver_spi esp_driver_uart
                  esp_timer esp_psram usb_host_uvc esp_jpeg
)
set_source_files_properties(digit_model.c digit_vision.c PROPERTIES COMPILE_OPTIONS "-O2")
