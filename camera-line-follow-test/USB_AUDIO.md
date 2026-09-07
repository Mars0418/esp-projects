# 小智接入：USB 音频适配进度

本地音频硬件层已完成；后续新增的激活、Opus 和会话接入见 [XIAOZHI.md](XIAOZHI.md)。以下记录为本地硬件调试阶段。
未增加 OTA、无线烧录或语音运动指令。电机仍禁用。

## 已实测的三合一模块

2026-09-07，通过 ESP32-S3 的 USB Host 和 COM9 读取设备描述符：

| 项目 | 实测值 |
| --- | --- |
| VID / PID | 349c / 3307 |
| USB 配置 | 522 字节，5 个接口 |
| 摄像头 | 接口 0 / 1，UVC |
| 音频控制 | 接口 2，UAC 1.0 |
| 麦克风 | 接口 3，alternate 1，端点 0x82 IN |
| 喇叭 | 接口 4，alternate 1，端点 0x03 OUT |
| 两路音频格式 | PCM，16000 Hz，单声道，16 bit |

继续使用已验证的 5V / GND / D- GPIO19 / D+ GPIO20 接线，不额外分配 I2S GPIO。
图片中的模块接口和 ESP32 的 GPIO 不要混为一谈。

驱动采用 `espressif/usb_host_uac == 1.5.0`，共用现有 USB Host。
保留工程内已有的非标准摄像头 UVC 兼容实现。

## 串口调试

在本项目 PowerShell 终端中编译、烧录：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\wifi-debug.ps1 -Action flash-monitor -Port COM9
```

串口 115200 波特率。输入以下完整命令并回车：

- `audio status`：接口是否准备就绪、是否采集、传输错误计数。
- `audio mic`：开始本地 PCM 音量计，每约一秒报告采样数、峰值、RMS。
- `audio stop`：停止麦克风流。
- `audio tone`：播放一次 600 Hz、约 200 ms 短音，数字幅度限制在约 -30 dBFS。

启动时两个音频流均暂停，不会自动收音或播放。
音量计只统计后丢弃 PCM，不保存、不上传；喇叭写入成功不等于已由人耳确认声音。

也可以关闭 monitor（Ctrl+]），运行自动检查：

```powershell
& 'D:\esp\tools\python\v5.4.4\venv\Scripts\python.exe' .\tools\serial_audio_check.py --port COM9
# 以下命令会额外播放一次短音：
& 'D:\esp\tools\python\v5.4.4\venv\Scripts\python.exe' .\tools\serial_audio_check.py --port COM9 --tone
```

已通过：麦克风约每秒 16000 采样、非零音量、提示音数据排队成功、正常暂停，
最终两路传输错误计数均为 0。提示音实际可听性仍待用户确认。

## 网络行为

按用户要求，新增 `CONFIG_CAR_WIFI_RESTORE_STATION`，默认为关闭。
旧手机热点凭据保留在 NVS，但开机不读取并连接。保留小车自建热点和本地网页。
在网页主动提交新的 Wi-Fi 配置仍会连接，但关闭该选项时不会在重启后自动恢复。

目前使用 COM9 数据线调试。普通 UART 不会自动共享电脑的互联网连接。
后续云端对话需要小车可用的新 Wi-Fi，或另行实现电脑串口联网桥接；当前两者尚未配置。

## 后续接入边界

1. 获得可用网络并完成小智账号设备绑定。
2. 在现有 ESP-IDF 5.4.4 工程适配小智通信协议和 Opus 音频；不直接覆盖为通用板固件。
3. 先验证静止时语音对话，再单独设计带急停和超时保护的运动指令。

官方参考源码：`E:\__JuniorSummer\xiaozhi-esp32-reference`，
基准提交 `c7241272f2d5fd140c77542f3cf12d09e717fc2f`。
