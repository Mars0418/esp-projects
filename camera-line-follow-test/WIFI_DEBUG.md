# Wi-Fi 无线调试

最新：已加入小智协议接入，操作及当前验证边界见 [XIAOZHI.md](XIAOZHI.md)。启用 `CONFIG_CAR_XIAOZHI` 会恢复已保存的热点连接；不提供无线烧录。

此版本用于摄像头和识别调试。`CONFIG_CAR_WIFI_DEBUG=y` 时不会初始化导航、编码器或电机控制任务，电机控制引脚维持安全停止。没有网页运动按钮。

2026-09-07：已加入 USB 三合一模块音频的 COM9 本地测试，参见 [USB_AUDIO.md](USB_AUDIO.md)。按用户要求，开机不再自动连接保存的手机热点；`CONFIG_CAR_WIFI_RESTORE_STATION` 默认为关闭。无线烧录未实现，也不在当前计划中。

## 编译 / 烧录

在当前项目目录的 PowerShell 中运行（脚本自动加载 D:\esp 下的 ESP-IDF 5.4.4、Python、Ninja 和工具链）：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\wifi-debug.ps1 -Action build
powershell -NoProfile -ExecutionPolicy Bypass -File .\wifi-debug.ps1 -Action flash-monitor -Port COM9
```

独立输出目录 `build-wifi-debug`，配置 `sdkconfig.local-wifi-debug`。请不要使用原先 `build-local-5.4.4-cmh-latest` 烧录来启动无线调试；那是正常小车任务固件，会启用运动。

该配置使用 1500 KiB 单应用分区以容纳网络栈，不提供 OTA；NVS 和 PHY 分区位置保持不变。烧录更新程序和分区表，不擦除 NVS。普通构建默认关闭 `CONFIG_CAR_WIFI_DEBUG`。

退出串口监视：Ctrl+]。一个串口同时只能被一个程序占用。

## 第一次连接

1. 烧录启动后，串口会打印：
   `AP_SSID=BallCar-xxxxxx AP_PASSWORD=... AP_URL=http://192.168.4.1/`
2. 手机或电脑连接这个热点，密码使用串口中的 `AP_PASSWORD`。每台设备首次启动随机生成密码并保存，重启后不变。路由器的密码不会输出到日志。
3. 若手机提示该 Wi-Fi 无互联网，选择保持连接。在浏览器地址栏输入 **http://192.168.4.1/**（不是 HTTPS）。
4. 即使没有路由器，也可查看摄像头画面、红/白/紫球与黑色目标状态。

## 连接路由器或手机热点

在网页填写 2.4 GHz Wi-Fi 名称和密码，点击“保存并连接”。支持 WPA2 网络及开放网络；不支持网页登录认证、企业账号认证。名称最多 32 个 UTF-8 字节，密码为空或 8–63 字节。

连接成功后页面和串口会显示小车的局域网 IP（`LAN_URL`）。电脑/手机切换到同一路由器，访问该 IP。路由器若启用客户端隔离，不同设备可能不能相互访问；这时继续直连小车热点。请勿在公网路由器上为调试端口做端口映射。

小车保持 AP+STA 模式：连接失败时热点仍可重新配网，每约 10 秒重试。AP 和 STA 共用无线频道，加入路由器时热点可能短暂断开，请重新连接。此固件不做热点互联网转发；拿到局域网 IP 不等于已经验证云端连接。

上述重试只在本次启动中主动提交配网、或显式开启 `CONFIG_CAR_WIFI_RESTORE_STATION` 时发生；默认开机不会恢复之前的外部 Wi-Fi。

## 画面与状态含义

- 网页显示板端 **160×120 RGB565 识别结果**，使用同一帧已经画好的检测框；不会在电脑重新跑另一套阈值。
- 预览约每 400 ms 请求一次，实际刷新受板端解码与网络影响。原始 640×480 JPEG 直播不在这一版范围内。
- 洋红色是紫球真实检测框，黄色是短时预测。表格区分“未检出 / 本帧检出 / 预测”。
- 摄像头超过 2 秒无新画面时会显示过期，不将旧图当作实时图。
- “暂停预览”只暂停浏览器取图，不停止板端检测；“画面旋转 180°”不改变算法坐标。
- 页面列出局域网 IP、信号强度、内部可用 RAM 和断线原因码，未实现完整串口日志转发。

## 串口找不到时

```powershell
& 'D:\esp\tools\python\v5.4.4\venv\Scripts\python.exe' -m serial.tools.list_ports -v
```

若显示 `no ports found`，Windows 尚未识别 USB-UART 设备。检查设备管理器、数据线、USB 接口与驱动。单独把 TX/RX 接到主板不会让电脑出现 COM 口，USB-UART 适配器本身必须插到电脑且被系统枚举。
