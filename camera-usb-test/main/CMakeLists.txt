idf_component_register(
    SRCS "main.c" "camera_display.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES usb esp_driver_gpio esp_driver_spi esp_timer esp_psram
                  usb_host_uvc esp_jpeg
)
