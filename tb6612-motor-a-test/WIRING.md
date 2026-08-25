# TB6612 四路驱动板接线记录

## 当前最终接线

| TB6612 板 | ESP32-S3 | 作用 |
|---|---:|---|
| GND | GND | 两块板共地，必须连接 |
| STBY | GPIO5 | 驱动总使能 |
| PWMA | GPIO6 | 电机 A 速度 PWM |
| AIN1 | GPIO15 | 电机 A 方向控制 1 |
| AIN2 | GPIO7 | 电机 A 方向控制 2 |
| Motor A 六芯接口 | 电机 A 插头 | 电机电源、编码器电源和信号经过该接口 |
| Vin+ / GND | 11.1V 电源正 / 负 | 电机驱动电源；接线时关闭电源开关 |

不要把 Vin+ 的 11.1V 接到 ESP32 的 3V3、5V 或 GPIO。

## 电机 A 编码器

六芯电机线已经把编码器接到驱动板，驱动板输出的两相信号连接如下：

| TB6612 板 | 建议 ESP32-S3 GPIO | 作用 |
|---|---:|---|
| E1A | GPIO16 | 电机 A 编码器 A 相输入 |
| E1B | GPIO17 | 电机 A 编码器 B 相输入 |

编码器信号应为 3.3V 逻辑。接线前仍需按板上丝印再次确认 E1A/E1B，不能把顶部的 5V 输出接入 ESP32 GPIO。

2026-08-25 当前接线实测：电机 A 输出轴手动正转一圈，四倍频计数从 `0` 变为 `-406`；反向转回一圈后为 `-1`。GPIO16/GPIO17 两相信号和方向判断正常，一圈约 406 计数。

## 当前总接线规划

以下为当前接线。原 D 驱动通道测试异常，左轮已经改接到 C 通道；ESP32
侧仍使用原来的 GPIO40/GPIO42/GPIO41 控制线，编码器改接 E3A/E3B。

2026-08-25 实测：A、B、D 按 20% PWM 依次运行 0.5 秒，三路均能正常转动并按程序停止。

| 电机 | PWM | IN1 | IN2 | 编码器 A | 编码器 B |
|---|---:|---:|---:|---:|---:|
| A | GPIO6 | GPIO15 | GPIO7 | GPIO16 | GPIO17 |
| B | GPIO11 | GPIO9 | GPIO10 | GPIO8 | GPIO18 |
| C（当前左轮） | GPIO40 | GPIO42 | GPIO41 | GPIO2 | GPIO1 |
| D（停用） | — | — | — | — | — |

| 其他信号 | ESP32-S3 | 说明 |
|---|---:|---|
| STBY | GPIO5 | 四路共用的驱动总使能 |
| ADC | GPIO12 / ADC2_CH1 | 驱动板电源电压检测模拟量；当前测试不读取 |

对应连接驱动板丝印：

- B：PWMB、BIN1、BIN2、E2A、E2B。
- C：PWMC、CIN1、CIN2、E3A、E3B。
- D：PWMD、DIN1、DIN2、E4A、E4B。
- STBY 和 GND 四路共用，不需要每路重复连接。

GPIO3、GPIO4、GPIO46 当前不接。GPIO40-42 同时具有 JTAG 默认功能；采用这些脚以后不要再使用外置 JTAG 调试器。当前 USB 下载和串口监视不受影响。最终扩展前，应根据实际 ESP32-S3-DevKitC-1 排针丝印逐根核对。

四路巡线传感器当前连接为 OUT1→GPIO13、OUT2→GPIO14、OUT3→GPIO21、
OUT4→GPIO39，模块使用 3.3V 供电并与驱动板共地。

## 上电顺序

1. 关闭 11.1V 电源，再插拔电机和杜邦线。
2. 先用 USB 给 ESP32 上电并烧录安全程序。
3. 电机悬空，确认程序处于停止状态。
4. 最后打开 11.1V 电源。
5. 出现意外转动、异味或明显发热时立即关闭 11.1V。

## 本机开发配置

- ESP-IDF：`C:\Espressif\v5.5.5\esp-idf`
- 芯片目标：`esp32s3`
- Flash：32 MB
- USB 控制台：USB Serial/JTAG，115200 baud
- Windows 已分配端口：`COM5`（Espressif VID `303A`、PID `1001`）
- 构建目录：`build-local-v5.5.5`

主板当前未连接时，Windows 不会把 `COM5` 列为活动端口。先接好数据 USB
线，再运行根目录的 `.\car.ps1 ports` 确认；如果系统重新分配了端口，
`.\car.ps1 run` 会按 USB VID/PID 自动选择。
