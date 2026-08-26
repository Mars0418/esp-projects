# ESP32-S3 小车工程

小车键盘遥控主工程是 `tb6612-motor-a-test`，当前编译入口为
`main/keyboard_remote.c`。推荐使用已验证的 ESP-IDF 5.4.4；`car.ps1` 会优先
使用参数指定或当前已激活的 ESP-IDF，否则扫描常见安装目录。串口按 Espressif
USB VID 自动识别，不依赖固定 COM 号。

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

只编译可以运行 `.\car.ps1 build`。也可显式指定串口或 ESP-IDF，例如：

```powershell
.\car.ps1 run -Port COM11
.\car.ps1 build -IdfPath C:\esp\v6.1-beta1\esp-idf
```

不同 ESP-IDF 版本使用独立的 `build-local-<版本>` 和本地 `sdkconfig.local-<版本>`，
不会改写仓库共享的 `sdkconfig`。

## 安全逻辑

- 启动时先拉低 TB6612 `STBY`，不会自动转动。
- 遥控 PWM 为 `260/1023`，巡线使用较低的连续 PWM。
- 手动运动命令超过 600 ms 未续期会自动停车；自动巡线由 `F` 启动，`X` 或空格停止。
- 当前前进/后退使用 A、B 两路；物理左轮接在 B 通道，C 通道保持停止。
- 四路红外使用 GPIO13、GPIO14、GPIO21、GPIO47，黑线为低电平。
- A/B 编码器分别使用 GPIO16/17 和 GPIO8/18；巡线时启用限幅双轮速度 PI。
- 普通循迹使用柔化 PID；同侧内外传感器同时见线时锁定方向并差速原地转弯。
- 丢线后向最后检测方向搜索，至少两路重新检测到黑线后恢复；搜索超时会安全停车。
- 终点停车暂未启用；四路全黑时保持直行，不会锁定停止。

引脚和上电顺序详见 [tb6612-motor-a-test/WIRING.md](tb6612-motor-a-test/WIRING.md)。
