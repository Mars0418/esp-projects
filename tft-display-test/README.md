# LQ_TFT18SPIV33 屏幕测试

该独立工程只测试 1.8 英寸 SPI TFT，不初始化电机、编码器、巡线或超声波。

| TFT | ESP32-S3 |
|---|---:|
| RST | 开发板 RST/EN |
| D/C | GPIO38 |
| SDI | GPIO4 |
| SCK | GPIO3 |
| CS | GPIO0（建议 10kΩ 上拉到 3V3） |
| VCC | 3V3 |
| GND | GND |

程序显示 `ESP32-S3`、`TFT TEXT` 和 `DISPLAY OK` 三行大字，每五秒在白底黑字
与黑底白字之间切换。黑白画面不依赖 RGB/BGR 顺序。默认按 ST7735S、128x160、
SPI Mode 0、2 MHz 初始化，画面使用保持 CS 有效的连续分块 SPI 传输。
