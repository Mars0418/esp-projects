# Car Hardware Test

ESP-IDF 5.5.5 interactive diagnostic firmware for the course ESP32-S3 car.

## Safety first

The course PDFs list module models but do not contain the car baseboard wiring
table. External-module pins therefore default to `GPIO_NUM_NC`. Fill in
`main/car_test_pins.h` from the actual wiring table before testing a module.
The firmware never starts motors automatically. Motor commands are limited to
70 percent duty and 3000 ms, then both motors stop.

HC-SR04 ECHO is normally 5 V. Use the interface circuit/level divider supplied
with the car; never connect a 5 V ECHO signal directly to an ESP32-S3 GPIO.
Power motors and the MG90S from the intended external supply with a shared
ground, not from an ESP32 GPIO.

## Build and flash

Open this exact folder in VS Code:

```text
D:\electronics-design\esp-projects\car-hardware-test
```

In an ESP-IDF terminal:

```powershell
idf.py -B build-v5.5.5 build
idf.py -B build-v5.5.5 -p COM5 flash
idf.py -B build-v5.5.5 -p COM5 monitor
```

The board is currently assigned COM5 (Espressif VID 303A, PID 1001), but the
number may change after reconnecting it. Use `ESP-IDF: Select Port to Use` in
VS Code if COM5 is no longer present.

## Commands

```text
status
led green|red|blue|off
line
distance
dht
i2c
imu
display red|green|blue|white|black
encoder [reset]
servo 0..180
motor a|b -70..70 1..3000
stop
help
```

The display implementation targets an ST7789 SPI panel because the course PDF
only says "TFT IPS SPI". Confirm the controller, resolution, offsets and
backlight polarity before enabling its pins.
