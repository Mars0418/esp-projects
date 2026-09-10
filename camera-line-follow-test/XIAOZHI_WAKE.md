# 你好乐迪：本地唤醒与连续对话

分支：`innovative-function`。此配置禁用电机控制，保留摄像头预览。

构建/烧录（ESP-IDF 5.4.4，COM9）：

```powershell
cd E:\__JuniorSummer\esp-projects-team\camera-line-follow-test
.\xiaozhi-wake.ps1 -Action build
.\xiaozhi-wake.ps1 -Action flash -Port COM9
.\xiaozhi-wake.ps1 -Action monitor -Port COM9
```

独立使用 `build-xiaozhi-wake` 和 `sdkconfig.local-xiaozhi-wake`，
烧录应用、分区表和识别模型；不擦除 NVS，保留热点及小智设备 UUID。
不要用旧 `wifi-debug.ps1` 构建唤醒版，它没有模型分区配置。

启动后等待 `WAKE_READY`，待机麦克风音频只用于本地 MultiNet 中文命令识别。
说“你好乐迪”，日志应出现 `WAKE_DETECTED`。
首次唤醒需要建立云端连接，等待 `LISTENING` 后再说问题。
本版尚无听觉就绪提示，请先配合串口观察。

唤醒后问题音频上传小智：停顿约 1.2 秒自动提交（最长 15 秒）。
回复播放结束后自动开始下一轮收音，约 8 秒未检测到讲话则回到本地唤醒。
能量端点阈值和唤醒阈值需要在实际距离、噪声下验证，不保证远场识别率。
播放时暂停收音，不支持播放中语音打断，不宣称已实现回声消除。

串口命令：

- `xz status`：查看会话、模型和本地监听状态。
- `xz wake off` 或 `xz stop`：关闭唤醒并停止云端对话。
- `xz wake on`：重新启用唤醒。
- `xz end`：结束本轮对话，保留原有唤醒开关状态。
- `xz bind` / `xz connect`：手动检查绑定/连接，不主动上传麦克风。

验收：模型就绪、MIC_METER 约 16000 samples/s、口述唤醒命中、
USER/ASSISTANT 问答日志、播放结束自动收音、无讲话返回本地、
停止命令后不再收音，以及摄像头/USB/内存无错误。

## 电脑实时监听日志

```powershell
& 'D:\esp\tools\python\v5.4.4\venv\Scripts\python.exe' -X utf8 .\tools\wake_monitor.py --port COM9 --connect
```

该窗口独占 COM9，不要同时启动 idf monitor。Ctrl+C 仅关闭窗口监听，不关闭设备唤醒。
日志同步写入 `captures/wake-monitor.log`（已忽略提交）。`WAKE_MONITOR` 中
fed/chunks/dropped/resets/hits 是累计计数，max_detect_ms 是最近窗口最大推理耗时。
`MIC_METER` 是音量，不是语音转写；`WAKE_CANDIDATE` 是模型候选，
`WAKE_DETECTED` 才表示命中；问答转写是 `USER` / `ASSISTANT`。

新增 UART 配网入口 `wifi config {"ssid":"热点名称","password":"热点密码"}`，
凭据保存在设备 NVS，不回显密码；不要将真实凭据提交到仓库。

对话收音期间说“结束对话”“停止对话”“退出对话”，收到云端 STT 后
固件关闭当前会话、清空播放并停止自动续聊，约 1.5 秒后回到本地唤醒。
完整句匹配允许末尾标点，不匹配“不要结束对话”“如何结束对话”。
这依赖网络语音转写，不是离线停机词；播放期间麦克风暂停，不能语音打断。
需要立即停掉全部收音/播放时使用 `xz stop`。角色提示词和男声音色需要
在小智控制台配置；仓库中的 `LEDI_CUSTOM_PROMPT.md` 不会自动上传或生效。
