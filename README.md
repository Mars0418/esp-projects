# ESP32-S3 小车工程

小车键盘遥控主工程是 `tb6612-motor-a-test`，当前编译入口为
`main/keyboard_remote.c`。推荐使用已验证的 ESP-IDF 5.5.5；`car.ps1` 会优先
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
   在串口监视终端中按住 `W/S` 前后移动、`A/D` 转向、`Q/E` 左右横移，
   `X` 或空格立即停车。
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
- 遥控 PWM 为 `130/1023`，单次按键适合人工轨迹精细采集。
- 手动运动命令超过 180 ms 未续期会自动停车；自动巡线由 `F` 启动，`X` 或空格停止。
- 实际轮位为 D=左前、A=右前、B=后轮。前进、后退和巡线使用 A/D 两个前轮，B 后轮停止；三轮横移和绕障弧线使用 A/B/D，绕障后的掉头使用 A/D 并释放 B。
- 四路红外使用 GPIO13、GPIO14、GPIO21、GPIO47，黑线为低电平。
- A/B/D 编码器分别使用 GPIO16/17、GPIO8/18 和 GPIO2/1，对应 E1/E2/E4；巡线时 A/D 两个前轮启用限幅速度 PI。
- HC-SR04 使用 TRIG=GPIO48、ECHO=GPIO39（ECHO 必须分压至 3.3V）。连续三次测得距离处于 8–15cm 后，先停车，再执行左向大半径绕行直到再次找到黑线；该窗口会过滤电机启动时偶发的极近距离假回波。随后 A/D 两个前轮差速原地掉头约 180°，B 后轮释放并被动滑动，重新对准黑线后恢复巡线。
- 超声与三路霍尔遥测在所有模式下每 80ms 输出一条 `TELEM` 记录，不需要按 `F`。运动中每次测距前会短暂关闭电机桥约 2ms，以抑制电机 PWM 造成的 20–28mm 假回波，随后自动恢复原运动命令。
- 普通循迹使用柔化 PID；同侧内外传感器同时见线时锁定方向并差速转弯。
- 丢线后向最后检测方向搜索，至少两路重新检测到黑线后恢复；搜索超时会安全停车。
- 绕障完成后才启用终点判断：稳定红外状态从 `0000`（全黑）直接变为 `1111`（全白）时立即停车并退出巡线。

引脚和上电顺序详见 [tb6612-motor-a-test/WIRING.md](tb6612-motor-a-test/WIRING.md)。

## 人工轨迹采集

先关闭其他串口监视器，再运行：

```powershell
& "C:\Espressif\tools\python\v5.5.5\venv\Scripts\python.exe" .\capture_telemetry.py --port COM5
```

脚本允许直接用键盘驾驶，并把全部串口输出保存到
`telemetry_logs/manual-<时间>.log`。每条 `TELEM` 记录包含毫秒时间、模式、
动作、避障阶段、超声距离、四路红外、A/B/D 累计编码器计数、区间增量、
counts/s、三路方向/PWM 以及 A/D 速度 PI 输出。按 `Ctrl+]` 退出时脚本会先
发送 `X` 急停再关闭串口。

## 三轮诊断

`main/three_wheel_diagnostic.c` 为上电默认停止的交互式诊断固件。编译时使用
`-D APP_SOURCE=three_wheel_diagnostic.c`，不会改变默认主程序入口。烧录诊断固件后，
把底盘架空并双击 `start_wheel_test.cmd`：

- `1`：测试 A 轮与 E1；`2`：测试 B 轮与 E2；`4`：测试 D 轮与 E4。
- `T`：按 A、B、D 顺序完整测试；`X` 或空格随时急停。
- 每轮正反各转 800ms，然后输出 `RESULT,<wheel>,PASS/FAIL`。同时只应有
  当前标记的物理轮转动。
