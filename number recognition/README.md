# ESP32-S3 本地手写数字识别

独立固件：通过现有 USB 摄像头采集画面，在 ESP32-S3 上完成单个手写数字
0–9 的预处理和卷积神经网络推理，并在 TFT、串口显示结果。运行时无需电脑计算或联网。

第一版使用白纸黑字、静止展示、一次一个数字。只输出识别事件，电机 STBY=GPIO5
和全部电机输出保持低电平；`1=SPIN`、`2=FORWARD` 是事件中的后续动作建议，
尚未接入电机动作。此项目与原有巡线、避障固件分别编译。

## 当前验证状态

- ESP-IDF 5.5.5 构建通过，`digit-v1.2` 应用固件 378,032 字节，已烧录到实车；
  1 MB 应用分区剩余约 64%。构建记录见 `model/build-validation.json`。
- ONNX 原模型：MNIST 官方测试集 10,000 张，正确 9,890 张（98.9%）。
- 固件同一份 C 推理代码在电脑上与 ONNX Runtime 比较：1,000 张预测全部一致，
  最大 logit 绝对误差约 `1.34e-5`；这 1,000 张中识别正确 984 张。
- 板端模型自检 10/10 通过。向实车注入 100 张 MNIST 标准输入，板端与 ONNX
  预测 100 次全部一致，其中正确 98 张；平均推理时间约 77.51 ms，最大 logit
  绝对误差约 `7.65e-6`，记录见 `model/board-validation.json`。
- 预处理：10 张合成白纸数字卡、空白、低对比度、边缘截断、多目标拒识通过。
  另验证了手写 5 的分离上横合并，以及多帧确认、移开重置、防重复事件。
- 实车摄像头已成功以 640×480 MJPEG 运行，本地预处理、推理和 TFT 显示链路正常；
  “同一数字连续 3 帧且每帧分数 ≥0.70”已能产生识别事件。真实手写数字卡的系统
  准确率仍需用一批带标签照片统计，单次现场识别不能替代准确率测试。

详情见 `model/validation.json`、`model/host-validation.json`、
`model/board-validation.json`。

## 硬件

复用本仓库已有配置：ESP32-S3，16 MB Octal PSRAM、32 MB Flash；USB 摄像头
VID/PID `349c:3307`，640×480 MJPEG；128×160 TFT。

| 接口 | 引脚 |
|---|---|
| USB 摄像头 | D-=19，D+=20，外部稳压 5V，共地 |
| UART0 | TX=43，RX=44，115200 baud |
| TFT | SCLK=3，MOSI=4，CS=0，DC=38 |
| 电机桥 | STBY=5，识别固件拉低 |

摄像头有非标准 UVC 描述，必须保留本仓库
`camera-usb-test/managed_components/espressif__usb_host_uvc` 中的兼容实现。

## 构建、烧录、使用

当前电脑 ESP-IDF 5.5.5 已安装。在本目录运行：

```powershell
.\digit.ps1 build
# 小车接好后使用实际串口；COM6 只是本次曾检测到的串口。
.\digit.ps1 flash -Port COM6
.\digit.ps1 monitor -Port COM6
```

`flash` 使用 `app-flash`，只覆盖 `0x10000` 的应用分区，保留现有引导程序、
分区表与 NVS。适用于本次已读取并核实的 1 MB factory 分区；换板必须重新核实。
构建脚本默认使用本机 `C:\Espressif`，可通过 `-IdfPath`、`-ToolsPath` 指定位置。
Windows 的 objdump 启动器不支持中文路径中的静态库，所以中间构建目录放在
`%TEMP%/esp32-digit-recognition-5.5.5`，可通过 `-BuildDir` 指定其他纯英文路径。
成功后将应用 `.bin`、`.elf`、`.map` 复制到项目的 `firmware/` 目录。

启动先运行十个数字样本的自检；必须出现 `MODEL_SELFTEST passed=10/10` 和
`DIGIT_READY`。随后等待 `CAMERA_STATUS=STREAMING_640X480_MJPEG`。

1. 把白纸上的粗黑数字完整放到屏幕中央黄色框里，让数字大约占框高的一半到八成，
   纸张边缘留在框外；避免阴影、反光和移动模糊。
2. 青色框表示检测到的数字区域；底部左侧 28×28 缩略图是模型实际输入，应该看到
   黑底白字、完整且正立的单个数字。
3. 顶部黄色数字是候选，绿色数字表示通过稳定确认。横杠表示当前未接受识别结果。
   底部数值和进度条是模型分数（0–100），不是实测正确率。
4. 串口每约 300 ms 输出 `RESULT`，包含候选数字、分数、拒识原因、裁剪位置、
   预处理与推理时间。确认后只输出一次 `DIGIT_EVENT`。
5. 要再次触发同一数字，先移开卡片，连续至少 5 帧且至少 500 ms 没有数字区域后重置。

串口命令需要回车：

| 命令 | 作用 |
|---|---|
| `R` | 模型输入顺时针旋转 90°；重复四次回到原方向 |
| `M` | 切换模型输入水平镜像 |
| `C` | 导出下一张原始画面与模型输入，用于电脑诊断 |

旋转/镜像仅修改模型输入，摄像头预览方向保持固定；观察底部缩略图判断设置。
设置目前只在本次上电有效。快照导出需约 7 秒，期间识别刷新暂停、摄像头持续收帧，
旧帧会丢弃，结束后恢复。拍照工具会保存原始 RGB565 与输入字节供复现。

## 模型与图像处理

采用 ONNX Model Zoo 已训练的 MNIST CNN，5994 个 float32 参数，权重 23,976 字节：

`28×28 → Conv5×5/8 + ReLU → Pool2 → Conv5×5/16 + ReLU → Pool3 → FC10 → Softmax`

为了先验证现成模型的完整行为，第一版保留 FP32，用小型 C 实现精确执行该固定图。
未使用 INT8 量化，也不在 ESP32 上加载 ONNX Runtime。推理临时工作区约 32 KB，
在内部 SRAM；图像预处理工作区在 PSRAM。后续是否量化应根据实测速度决定。

摄像头 MJPEG 解码成 160×120 RGB565，按 TFT 方向旋转为 120×160。只在中间
100×100 区域找数字：灰度与 Otsu 阈值、连通域、相邻分离笔画合并、等比例缩放至
最长边 20 像素、补成 28×28、重心居中。支持的是中央框中的单数字，不是全场景目标检测。

按用户最新要求，确认规则为同一数字连续 3 帧分数均 ≥0.70。
中途低于 70 分或数字变化就重新计数；第一二名差值仅用于日志，不再作为门限。
超过 600 ms 的旧画面不触发事件。`digit_gate.c` 将分类转换为一次性事件，后续可
将 `DIGIT_EVENT` 交给独立运动控制器；运动距离、角度和结束条件尚待定义与标定。

## 电脑诊断与复现

本次已创建独立 Python 环境 `../../tmp/digit-python`，不改动 ESP-IDF 的 Python 包。
若需要重建，使用 Python 3.13 的 venv 安装 `tools/requirements.txt`。

```powershell
# 小车接好后抓取实拍图；先退出其他串口监视器。
& ..\..\tmp\digit-python\Scripts\python.exe tools\capture_camera.py --port COM6

# 标准输入注入测试：核对真实板端计算，不能用于宣称摄像头准确率。
& ..\..\tmp\digit-python\Scripts\python.exe tools\verify_board.py --port COM6 --data-dir ..\..\tmp --count 100

# 重新检查 C 推理与预处理；需要 Visual Studio C 编译器。
.\tools\build_host.cmd
& ..\..\tmp\digit-python\Scripts\python.exe tools\verify_host.py --data-dir ..\..\tmp
```

模型来源、哈希和授权信息见 `model/README.md`。生成的头文件已纳入项目，
正常固件构建不需要重新下载模型或安装 ONNX。

## 原应用备份

本次在 `../../tmp/digit-backup/` 保存了：

- `flash-prefix.bin`：Flash `0x0..0xFFFF`，包含原引导、分区表及该段数据。
- `original-app.bin`：Flash `0x10000..0x10FFFF`，完整 1 MB factory 应用分区。

没有做完整 32 MB Flash 镜像；应用烧录不覆盖其余区域。应用备份 SHA256：
`c9f69d3d3dfa5928bf305bb1ebc7ca73291b6f4646513ae8c8c56f9ce911fa58`。
如需回到原应用，用 esptool 将 `original-app.bin` 写回 `0x10000`，保留引导程序和分区表。
