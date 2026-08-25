# ESP32-S3 小车工程（本机配置）

小车键盘遥控主工程是 `tb6612-motor-a-test`，当前编译入口为
`main/keyboard_remote.c`。已按这台电脑配置为 ESP-IDF 5.5.5，开发板在 Windows
中的已分配端口是 `COM5`。

## 最快启动

1. 先关闭 11.1 V 电机电源，用能传数据的 USB 线连接 ESP32-S3。
2. 在本目录打开 PowerShell，先检查端口：

   ```powershell
   .\car.ps1 ports
   ```

3. 把车轮悬空，一键编译、烧录并打开串口监视：

   ```powershell
   .\car.ps1 run
   ```

4. 看到 `USB KEYBOARD REMOTE READY` 后再打开 11.1 V 电机电源。
   在串口监视终端中按住 `W/S/A/D` 移动，`X` 或空格立即停车。
   将四路红外放在线上后按一次 `F` 启动自动巡线。
   按 `Ctrl+]` 退出监视。

只编译可以运行 `.\car.ps1 build`。如果 Windows 以后改了 COM 号，脚本会按
Espressif USB VID/PID 自动识别；也可显式指定，例如 `.\car.ps1 run -Port COM6`。

## 安全逻辑

- 启动时先拉低 TB6612 `STBY`，不会自动转动。
- 遥控 PWM 为 `260/1023`，巡线使用较低的连续 PWM。
- 运动命令超过 600 ms 未续期就自动停车；USB 线拔出或终端中断后不会持续运转。
- 当前前进/后退使用 A、C 两路，B 路保持停止；原 D 驱动通道已停用。
- 四路红外使用 GPIO13、GPIO14、GPIO21、GPIO39，黑线为低电平。
- A/C 编码器分别使用 GPIO16/17 和 GPIO2/1；巡线时启用限幅双轮同步 PI。
- 直角采用外侧轮快、内侧轮慢的圆弧转向；丢线后向最后检测方向搜索，至少两路重新检测到黑线后恢复。

引脚和上电顺序详见 [tb6612-motor-a-test/WIRING.md](tb6612-motor-a-test/WIRING.md)。
