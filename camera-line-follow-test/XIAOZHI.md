# v7 小智语音与导览共存

## 使用

烧录 v7 后，重新打开根目录的串口调试窗口并连接 COM6。
小智会尝试使用 NVS 中之前保存的 2.4 GHz 热点；没有配置时，先点“配置 Wi-Fi（开始前）”。
小智绑定沿用设备 MAC 和 car_xiaozhi/client_uuid。需要首次绑定时，点击“绑定”，将窗口显示的绑定码填入 https://xiaozhi.me/，完成后点“小智开启”。
等待“小智：正在监听”，可以直接说话，由服务端自动断句、通过车载 USB 喇叭回答。
“关闭监听”只关闭小智；“小智开启”恢复对话。重启默认重新开启，Disconnect 只断开电脑串口，不会关闭车载监听。

原导览流程继续使用“开始 / 停车 / 起步”：先欢迎词，再数字识别，再原地左右转 30°；每站停车播报并等待人工放行，终点停车。
欢迎词、站点和终点播报优先，抢占当前小智回复。麦克风一直采集并排空缓冲，但本地播报和小智回复期间不向云端送音，结束后自动恢复。没有回声消除，因此不支持喇叭说话时插话。
这是联网持续对话，不是本地唤醒词识别；监听时音频发送到小智服务。模型不接收摄像画面、不控制电机。

## 隔离

- 原 motor owner 保持 camera_pursuit，core 0 / priority 6；网络与 Opus worker priority 1，USB 麦克风 priority 2；不存在从云端执行电机命令的路径。
- 一个 UAC driver；喇叭只由 tour_speech worker 使用。麦克风拥有独立 RX 任务，网络通过有界队列交互，采集入队不等待。
- 本地播报 pending 与小智播放状态分开。小智断网、编码或队列失败会关闭对话并退避重连，不设置导览停车状态。物理喇叭拔出仍沿用原导览故障停车逻辑。
- 录音和播放大队列、49 KB Opus 任务栈、TLS 加密缓冲放入 PSRAM。实车首次启动发现内部 TLS 堆分配返回 -0x7F00，已将 MBEDTLS_EXTERNAL_MEM_ALLOC 加入配置。CPU 调度隔离不能替代实车帧率、USB 带宽和音频质量验证。
- Wi-Fi 配置只在首次“开始”前接收，不在行驶时保存配置。TLS 保留 CA 和主机名验证。凭证不写入仓库或串口日志，不擦除已有 NVS。
- 串口配置使用 @WIFI,<ssid>,<password> 和 @XZ,ON / OFF / BIND 换行结尾。框内的 X/F/P/空格是数据；Ctrl-C 始终能急停。调试窗口急停改为 Ctrl-C。原来的 X、空格快捷键在框外仍可用。

## 构建与烧录

运行 `./tour.ps1 build`；烧录使用 `./tour.ps1 flash -Port COM6`，先 Disconnect。
v7 增加 Wi-Fi/TLS/Opus，工厂应用分区扩至 4 MB，NVS 和 PHY 偏移不变。烧录脚本同时更新 bootloader、分区表和应用，不再只写应用，否则旧 1 MB 分区不足。不会执行 erase-flash。
Windows 链接器对中文路径下预编译库有限制，CMake 将只读音频 archive 复制到 ASCII build 目录并更新链接目标，不修改官方组件。

## 验证范围

主机回归覆盖：数字识别门控、两条路线、角点重触发、播报映射和手动放行、30° 编码器控制、串口命令隔离/溢出/急停、调试窗口 CRC 分片和小智状态/命令。
ESP-IDF 构建检查包括全部新增 C 源文件、依赖链接和分区大小。
实车仍需确认：MIC_READY、联网与绑定、OPUS_SELFTEST_OK、连续问答，以及对话同时运行时的巡线帧率和站点播报抢占。未完成这些检查前，不能声称对实车性能完全无影响。

协议依据：https://github.com/78/xiaozhi-esp32/blob/main/docs/websocket.md （listen auto、Opus 16 kHz / 60 ms，WSS）。客户端基于本地 tour-guide-review 的 xiaozhi_client.c 改造。

实车补充：硬件 AES 对 PSRAM 数据产生内部 DMA 缓冲分配失败，已关闭 MBEDTLS_HARDWARE_AES，改用软件 AES；联网失败的 10 秒退避从尝试结束后计时。

内存隔离补充：启用 SPIRAM_TRY_ALLOCATE_WIFI_LWIP；40 KB TFT 帧缓冲放在 PSRAM，仅预留 4 KB DMA 分块缓冲；短屏幕命令使用 SPI 内联数据，避免运行时为每帧申请临时 DMA 内存。调试窗口连接自动发送 @TIME 校时，TLS 仍验证证书。

2026-09-10 COM6 实车检查：最终 v7 引导程序、应用及分区表均烧录并通过哈希校验；启动检查通过 Opus 自检，麦克风/喇叭就绪，DISCOVERY_HTTP=200，SESSION_READY，LISTENING。45 秒启动采样未再次出现 TLS/屏幕内部内存分配失败。实际调试串口在 8 秒采样内收到 18 帧 160×120 图像，无 CRC/串口错误，最后小智状态为回复中、导览仍 WAIT_START、can_start=1。未发送运动指令，整条实车路线和转角尚未复测。检查后 COM6 已关闭。
