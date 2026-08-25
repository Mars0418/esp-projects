idf_component_register(
    SRCS "keyboard_remote.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_gpio esp_driver_ledc esp_driver_usb_serial_jtag esp_timer
)
