idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES
        esp_driver_gpio
        esp_driver_i2c
        esp_driver_ledc
        esp_driver_rmt
        esp_driver_spi
        esp_lcd
        esp_timer
        led_strip
)
