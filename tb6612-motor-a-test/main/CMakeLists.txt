set(APP_SOURCE "keyboard_remote.c" CACHE STRING
    "Application source file; use three_wheel_diagnostic.c for wheel testing")

idf_component_register(
    SRCS "${APP_SOURCE}" "tft_status_display.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_gpio esp_driver_ledc esp_driver_spi esp_driver_usb_serial_jtag esp_timer vfs
)
