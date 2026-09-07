# 小智：手动语音对话接入

代码位于 `innovative-function` 分支的 `camera-line-follow-test/main/xiaozhi_client.c`。
沿用 ESP-IDF 5.4.4、小车 UVC 兼容代码和 USB UAC 音频，不覆盖为小智通用板固件。
第一版不含唤醒词、MCP 运动控制、摄像头上传、自动录音或 OTA。

## 用户现在需要做什么

1. 开启已保存的手机热点，保持小车供电及 COM9 连接。
2. 在 https://xiaozhi.me/ 注册或登录自己的账号，不要把登录密码、短信验证码发到聊天里。
3. 程序打印 `BIND_CODE=...` 后，在官网设备管理的添加/绑定设备入口填写该码。
4. 网页提示绑定成功后，重新执行 `xz bind`，再执行 `xz connect`。
5. 只有串口出现 `SESSION_READY`，才开始语音测试。

绑定码由服务器生成，不能事先编造；过期时重新运行 `xz bind`。
`xz bind` 收到服务端 WebSocket 配置，不等于已通过会话认证；以 `SESSION_READY` 为准。

## 烧录和串口命令

项目目录 PowerShell：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\wifi-debug.ps1 -Action flash-monitor -Port COM9
```

在 monitor 中输入整行命令并回车：

| 命令 | 动作 |
| --- | --- |
| `xz bind` | HTTPS 获取绑定码及会话配置；会关闭已有会话 |
| `xz connect` | 建立 WSS、发送 hello、等待服务端确认；不会打开麦克风 |
| `xz listen` | 开始录音上传，最长 15 秒；需先看到 SESSION_READY |
| `xz send` | 停止录音、发送本次讲话结束标志，等待回复 |
| `xz stop` | 中止会话、停录音、清播放队列 |
| `xz status` | 查看配置/会话/录音状态与收发包计数 |

`xz listen` 后说一句话，再输入 `xz send`，会通过三合一模块的喇叭播放回答。
当前无自动连续收音；再次讲话需要再次输入 `xz listen`。
第一次测试请用普通问候，不发运动指令；固件电机一直禁用。
排查 USB 硬件用的 `audio mic/stop/tone/status` 仍保留，见 [USB_AUDIO.md](USB_AUDIO.md)。

不打开 monitor 时，可用单次串口工具（不要同时占用 COM9）：

```powershell
& 'D:\esp\tools\python\v5.4.4\venv\Scripts\python.exe' .\tools\xiaozhi_serial.py bind --seconds 45
& 'D:\esp\tools\python\v5.4.4\venv\Scripts\python.exe' .\tools\xiaozhi_serial.py connect --seconds 20
```

## 安全与数据

- 只有明确执行 `xz listen` 才把麦克风音频上传到小智服务。
- HTTPS 发现接口发送设备 MAC、专用 UUID、板型和固件信息；不发送热点密码、摄像头图像或录音。
- 官网接口路径包含 `/ota/`，官方用它同时下发激活/会话配置；此实现忽略 firmware 字段，没有固件下载/写入逻辑。
- TLS/WSS 必须验证证书链和主机名；不降级到 HTTP/WS。TLS 使用 PSRAM，避免与 USB DMA 争抢内部 RAM。
- 设备 UUID 保存在独立 NVS 命名空间 `car_xiaozhi`；会话 token 仅在内存中，不打印、不提交到仓库。
- 服务器命令无法控制电机、重启或更新固件。未声明 MCP 工具。
- 断线、音频队列溢出及超时会关闭会话；录音每次限 15 秒，回答等待/播放限时 60 秒。
- 用户与助手文字会打印到本地串口，分享日志前注意隐私。

## 实现与验证

- 上行：16 kHz / 单声道 / 16-bit PCM，每 960 个采样编码为 60 ms Opus。
- 下行：Opus 原生按 16 kHz 解码，匹配 USB 喇叭；支持最多 120 ms 单包、单声道。
- WSS 支持协议版本 1/2/3、接收缓冲分块与 continuation 分帧、长度校验。
- 网络任务与 USB 音频任务通过有界 PSRAM 队列交换数据，不在网络回调里操作 USB 设备。
- Opus 自检使用合成信号，不读取麦克风：已在板上验证 1920 字节 PCM 编码、解码回 1920 字节。
- Opus 任务栈为 48 KiB；60 ms 自检后实测剩余约 24 KiB。
- 已完成官网绑定、HTTPS 发现及 `SESSION_READY` 会话认证。实测一次问答上传 249 帧、接收 484 帧，用户确认喇叭回复清晰；结束后 recording=0、replying=0，电机禁用。
- 本地源码将回复数字增益从 1/4 小幅提高到 3/8（约 +3.5 dB），只调整回复播放，不改变麦克风或提示音。用户随后要求不烧录，已在写入前中止流程：小车仍运行原来的 1/4 音量，新参数尚未生效。当前固件没有串口、网页或 MCP 运行时音量设置接口。

启用 `CONFIG_CAR_XIAOZHI` 会按用户最新要求恢复保存的手机热点连接。
此前 UART-only 阶段关闭热点自动恢复的行为不再适用于本配置。

官方参考：https://github.com/78/xiaozhi-esp32 ，基准提交
`c7241272f2d5fd140c77542f3cf12d09e717fc2f`。
