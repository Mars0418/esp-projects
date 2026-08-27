# ESP32-S3 USB 摄像头 TFT 显示测试

该固件通过 USB Host 读取摄像头的 640×480 MJPEG 视频，将画面缩放到
160×120、旋转为竖屏方向，然后显示到 128×160 TFT。电机、编码器、红外和
超声波均不初始化。

这是与 `tb6612-motor-a-test` 循迹/避障固件完全分开的独立 ESP-IDF 工程。两者
只有一个能被烧录到开发板运行，摄像头工程不会调用原小车控制代码。

## 编译与烧录

已使用 ESP-IDF 5.4.4 在 ESP32-S3（16 MB PSRAM、32 MB Flash）上验证：

```bash
cd camera-usb-test
idf.py build
idf.py -p <串口> flash monitor
```

工程内保留了针对当前摄像头非标准 UVC 描述及 Bulk 分片方式的兼容组件，克隆仓库
后无需手工重复修改该组件。

## 接线

| 摄像头 | ESP32-S3 |
|---|---|
| 5V | 稳压 5V |
| GND | GND |
| D- | GPIO19 |
| D+ | GPIO20 |

UART0 日志使用 115200 baud，默认 TX=GPIO43、RX=GPIO44。测试时关闭 11.1V 电机电源。

| TFT | ESP32-S3 |
|---|---|
| SCK | GPIO3 |
| SDI/MOSI | GPIO4 |
| CS | GPIO0 |
| D/C | GPIO38 |
| RST | EN/RST |
| VCC | 3.3V |
| GND | GND |

正常工作时日志会出现 `CAMERA_STATUS=STREAMING_640X480_MJPEG`，随后每秒输出
`FRAME_STATUS`。其中 `displayed` 持续增加表示图像已解码并发送到 TFT。

屏幕 SPI 使用 20 MHz，关闭 ST7735 反显模式；摄像头画面缩放并旋转后写入
128×160 屏幕。

## 已验证设备

2026-08-27 通过 UART0 实测枚举成功：

- VID: `0x349c`
- PID: `0x3307`
- 产品名：`HD video`
- USB 速率：Full Speed
- 两个 UVC 视频接口
- 同时包含 USB Audio 接口（摄像头板载麦克风/音频功能）
- 支持 640×480 MJPEG，输入约 15 fps；TFT 实际刷新率以串口 `displayed` 为准
- 该摄像头的 UVC 描述顺序和 Bulk 分片方式不标准，工程内已加入专用兼容处理
