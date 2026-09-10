"""Live, synchronized debugger for line following and ball capture."""

from __future__ import annotations

import argparse
from collections import deque
from dataclasses import dataclass
from datetime import datetime
import json
from pathlib import Path
import queue
import struct
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk, simpledialog
import zlib

try:
    import serial
    from serial.tools import list_ports
except (ImportError, ModuleNotFoundError) as exc:
    raise SystemExit("pyserial is required; run start_push_debug_viewer.ps1") from exc


MAGIC = b"\n@PDBG1\n"
PREFIX = struct.Struct("<8sIII")
NORMAL_BAUD = 115200
STREAM_BAUD = 921600
SWITCH_MARKER = b"VSTREAM_SWITCH,921600"
MAX_METADATA_BYTES = 4096
MAX_PAYLOAD_BYTES = 65536


@dataclass(frozen=True)
class DebugFrame:
    metadata: dict[str, object]
    pixels: bytes


def rgb332_to_rgb888(source: bytes) -> bytes:
    destination = bytearray(len(source) * 3)
    for index, value in enumerate(source):
        red = (value >> 5) & 0x07
        green = (value >> 2) & 0x07
        blue = value & 0x03
        offset = index * 3
        destination[offset] = (red << 5) | (red << 2) | (red >> 1)
        destination[offset + 1] = (green << 5) | (green << 2) | (green >> 1)
        destination[offset + 2] = (blue << 6) | (blue << 4) | (blue << 2) | blue
    return bytes(destination)


def frame_rgb888(frame: DebugFrame) -> bytes:
    if frame.metadata.get("format") == "RGB332":
        return rgb332_to_rgb888(frame.pixels)
    output = bytearray()
    for (v,) in struct.iter_unpack(">H", frame.pixels):
        output.extend((((v >> 11) & 31) * 255 // 31,
                       ((v >> 5) & 63) * 255 // 63, (v & 31) * 255 // 31))
    return bytes(output)


def upright_simple_frame(metadata: dict, rgb: bytes) -> tuple[dict, bytes]:
    """Display-only rotation; recorded frames and controller keep raw coordinates."""
    import copy
    result = copy.deepcopy(metadata)
    w, h = int(result["width"]), int(result["height"])
    for key in ("red", "purple", "goal"):
        item = result.get(key)
        if not isinstance(item, dict):
            continue
        if item.get("found"):
            x, y = item["center"]
            item["center"] = [w-1-x, h-1-y]
            left, top, right, bottom = item["box"]
            item["box"] = [w-1-right, h-1-bottom, w-1-left, h-1-top]
        if item.get("corner_found"):
            x, y = item["corner"]
            item["corner"] = [w-1-x, h-1-y]
    return result, b"".join(rgb[i:i+3] for i in range(len(rgb)-3, -1, -3))


class SerialWorker:
    def __init__(self, events: queue.Queue[tuple[str, object]]) -> None:
        self.events = events
        self.port: serial.Serial | None = None
        self.thread: threading.Thread | None = None
        self.stop_event = threading.Event()
        self.write_lock = threading.Lock()

    @property
    def connected(self) -> bool:
        return self.port is not None and self.port.is_open

    def connect(self, port_name: str) -> None:
        # Set reset-control lines before opening the ESP32 port.
        port = serial.Serial(
            None,
            NORMAL_BAUD,
            timeout=0.05,
            write_timeout=1.0,
            rtscts=False,
            xonxoff=False,
        )
        port.dtr = False
        port.rts = False
        port.port = port_name
        port.open()
        # Some USB-UART adapters reset the board even when DTR/RTS are preset.
        # Let boot output finish before sending a command into the new RX task.
        boot_deadline = time.monotonic() + 4.0
        while time.monotonic() < boot_deadline:
            port.read(512)
        port.reset_input_buffer()

        handshake = bytearray()
        deadline = time.monotonic() + 6.0
        next_request = 0.0
        while SWITCH_MARKER not in handshake and time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_request:
                port.write(b"\nVSTREAM,1\n")
                port.flush()
                next_request = now + 0.35
            handshake.extend(port.read(512))
        if SWITCH_MARKER not in handshake:
            port.close()
            raise serial.SerialException(
                "Firmware did not accept VSTREAM,1. Flash the new firmware "
                "and make sure idf.py monitor is closed."
            )

        time.sleep(0.08)
        port.baudrate = STREAM_BAUD
        port.reset_input_buffer()
        self.port = port
        self.stop_event.clear()
        self.thread = threading.Thread(
            target=self._read_loop, name="push-debug-serial", daemon=True
        )
        self.thread.start()
        self.write(f"@TIME,{int(time.time())}\n".encode("ascii"))

    def write(self, payload: bytes) -> None:
        port = self.port
        if port is None or not port.is_open:
            return
        with self.write_lock:
            port.write(payload)
            port.flush()

    def disconnect(self) -> None:
        port = self.port
        if port is None:
            return
        if port.is_open:
            try:
                self.write(b"VSTREAM,0\n")
                time.sleep(0.25)
            except serial.SerialException:
                pass
        self.stop_event.set()
        try:
            port.close()
        except serial.SerialException:
            pass
        if self.thread is not None and self.thread.is_alive():
            self.thread.join(timeout=0.5)
        self.thread = None
        self.port = None

    def _read_loop(self) -> None:
        assert self.port is not None
        receive = bytearray()
        try:
            while not self.stop_event.is_set():
                receive.extend(self.port.read(4096))
                self._parse_packets(receive)
        except (serial.SerialException, OSError) as exc:
            if not self.stop_event.is_set():
                self.events.put(("error", str(exc)))

    def _parse_packets(self, receive: bytearray) -> None:
        while True:
            magic_index = receive.find(MAGIC)
            if magic_index < 0:
                if len(receive) > len(MAGIC) - 1:
                    del receive[: -(len(MAGIC) - 1)]
                return
            if magic_index > 0:
                del receive[:magic_index]
            if len(receive) < PREFIX.size:
                return

            magic, metadata_length, payload_length, expected_crc = PREFIX.unpack_from(
                receive
            )
            if (
                magic != MAGIC
                or metadata_length <= 0
                or metadata_length > MAX_METADATA_BYTES
                or payload_length <= 0
                or payload_length > MAX_PAYLOAD_BYTES
            ):
                del receive[0]
                continue

            packet_length = PREFIX.size + metadata_length + payload_length
            if len(receive) < packet_length:
                return
            body = bytes(receive[PREFIX.size:packet_length])
            if zlib.crc32(body) != expected_crc:
                self.events.put(("bad_frame", "CRC mismatch"))
                del receive[0]
                continue

            metadata_raw = body[:metadata_length]
            payload = body[metadata_length:]
            try:
                metadata = json.loads(metadata_raw.decode("ascii"))
                width = int(metadata["width"])
                height = int(metadata["height"])
                pixel_bytes = {"RGB332": 1, "RGB565BE": 2}.get(metadata.get("format"), 0)
                if pixel_bytes == 0 or len(payload) != width * height * pixel_bytes:
                    raise ValueError("invalid frame geometry")
            except (UnicodeDecodeError, ValueError, KeyError, TypeError) as exc:
                self.events.put(("bad_frame", str(exc)))
                del receive[:packet_length]
                continue
            del receive[:packet_length]
            self.events.put(("frame", DebugFrame(metadata, payload)))


class PushDebugViewer:
    def __init__(
        self, root: tk.Tk, initial_port: str, repository: Path, auto_connect: bool
    ) -> None:
        self.root = root
        self.repository = repository
        self.events: queue.Queue[tuple[str, object]] = queue.Queue()
        self.serial_worker = SerialWorker(self.events)
        self.port_descriptions: dict[str, str] = {}
        self.frame_times: deque[float] = deque(maxlen=40)
        self.last_sequence: int | None = None
        self.bad_frames = 0
        self.host_dropped_frames = 0
        self.photo: tk.PhotoImage | None = None
        self.record_file = None
        self.record_directory: Path | None = None

        root.title("ESP32 Integrated Mission Debugger")
        root.geometry("1180x740")
        root.minsize(900, 620)
        root.protocol("WM_DELETE_WINDOW", self.close)

        self.port_var = tk.StringVar(value=initial_port)
        self.status_var = tk.StringVar(value="Disconnected")
        self.stats_var = tk.StringVar(value="Waiting for frames")
        self.control_var = tk.StringVar(value="Control: --")
        self.xiaozhi_var = tk.StringVar(value="小智：等待设备状态")
        self.mission_var = tk.StringVar(value="Mission: --")
        self.objects_var = tk.StringVar(value="Objects: --")
        self.record_var = tk.BooleanVar(value=True)
        self._build_ui()
        self.refresh_ports()
        root.after(30, self._poll_events)
        if auto_connect:
            root.after(250, self.toggle_connection)

    def _build_ui(self) -> None:
        outer = ttk.Frame(self.root, padding=10)
        outer.pack(fill="both", expand=True)

        toolbar = ttk.Frame(outer)
        toolbar.pack(fill="x")
        ttk.Label(toolbar, text="Serial port").pack(side="left")
        self.port_box = ttk.Combobox(
            toolbar, textvariable=self.port_var, width=12, state="readonly"
        )
        self.port_box.pack(side="left", padx=6)
        ttk.Button(toolbar, text="Refresh", command=self.refresh_ports).pack(side="left")
        self.connect_button = ttk.Button(
            toolbar, text="Connect", command=self.toggle_connection
        )
        self.connect_button.pack(side="left", padx=6)
        self.simple_start_button = ttk.Button(
            toolbar, text="开始红球→紫球", command=self.start_simple_push, state="disabled"
        )
        self.simple_start_button.pack(side="left", padx=4)
        self.pause_button = ttk.Button(toolbar, text="停车", command=self.pause_tour)
        self.pause_button.pack(side="left", padx=4)
        ttk.Button(toolbar, text="试听欢迎词", command=self.preview_speech).pack(side="left", padx=4)
        ttk.Button(toolbar, text="Emergency stop", command=self.emergency_stop).pack(
            side="left", padx=(8, 0)
        )
        ttk.Checkbutton(
            toolbar, text="Record frames", variable=self.record_var
        ).pack(side="left", padx=12)
        ttk.Label(toolbar, textvariable=self.status_var).pack(side="right")

        content = ttk.Panedwindow(outer, orient="horizontal")
        content.pack(fill="both", expand=True, pady=(10, 0))
        self.image_panel = ttk.LabelFrame(
            content, text="Algorithm frame", padding=8
        )
        details_panel = ttk.LabelFrame(content, text="Synchronized decision", padding=8)
        content.add(self.image_panel, weight=3)
        content.add(details_panel, weight=2)

        self.canvas = tk.Canvas(
            self.image_panel, width=640, height=480, background="#111111",
            highlightthickness=0
        )
        self.canvas.pack(fill="both", expand=True)

        ttk.Label(details_panel, textvariable=self.stats_var, wraplength=340).pack(
            anchor="w", fill="x", pady=(0, 8)
        )
        ttk.Label(details_panel, textvariable=self.mission_var, wraplength=340).pack(
            anchor="w", fill="x", pady=(0, 8)
        )
        ttk.Label(details_panel, textvariable=self.control_var, wraplength=340).pack(
            anchor="w", fill="x", pady=(0, 8)
        )
        ttk.Label(details_panel, textvariable=self.objects_var, wraplength=340).pack(
            anchor="w", fill="x", pady=(0, 8)
        )
        ttk.Label(details_panel, textvariable=self.xiaozhi_var, wraplength=340).pack(anchor="w", pady=4)
        voice_bar = ttk.Frame(details_panel)
        voice_bar.pack(anchor="w", pady=4)
        ttk.Button(voice_bar, text="小智开启", command=lambda: self.voice_command("XZ,ON")).pack(side="left")
        ttk.Button(voice_bar, text="关闭监听", command=lambda: self.voice_command("XZ,OFF")).pack(side="left")
        ttk.Button(voice_bar, text="绑定", command=lambda: self.voice_command("XZ,BIND")).pack(side="left")
        ttk.Button(details_panel, text="配置 Wi-Fi（开始前）", command=self.configure_wifi).pack(anchor="w")
        ttk.Label(details_panel, text="Frame metadata").pack(anchor="w")
        self.metadata_text = tk.Text(details_panel, height=20, width=46, wrap="none")
        self.metadata_text.pack(fill="both", expand=True, pady=(4, 0))
        self.metadata_text.configure(state="disabled")

    def refresh_ports(self) -> None:
        ports = sorted(list_ports.comports(), key=lambda item: item.device)
        devices = [item.device for item in ports]
        self.port_descriptions = {
            item.device: item.description or "Serial device" for item in ports
        }
        self.port_box["values"] = devices
        if self.port_var.get() not in devices:
            self.port_var.set(devices[0] if len(devices) == 1 else "")
        if not devices:
            self.status_var.set("No serial port found")

    def toggle_connection(self) -> None:
        if self.serial_worker.connected:
            self.disconnect()
            return
        port_name = self.port_var.get().strip()
        if not port_name:
            messagebox.showerror("Connection", "Select the ESP32 serial port.")
            return
        try:
            self.serial_worker.connect(port_name)
        except serial.SerialException as exc:
            messagebox.showerror("Connection failed", str(exc))
            return
        self.connect_button.configure(text="Disconnect")
        self.status_var.set(f"{port_name} @ {STREAM_BAUD}")
        self.last_sequence = None
        self.bad_frames = 0
        self.host_dropped_frames = 0
        self.frame_times.clear()
        if self.record_var.get():
            self._start_recording()

    def disconnect(self) -> None:
        self.simple_start_button.configure(state="disabled")
        self.serial_worker.disconnect()
        self._stop_recording()
        self.connect_button.configure(text="Connect")
        self.status_var.set("Disconnected")

    def emergency_stop(self) -> None:
        try:
            self.serial_worker.write(b"\x03")
        except serial.SerialException as exc:
            self.events.put(("error", str(exc)))

    def voice_command(self, command):
        if not self.serial_worker.connected:
            messagebox.showinfo("小智", "请先连接 COM6")
            return
        try:
            self.serial_worker.write(("@" + command + "\n").encode("utf-8"))
        except Exception as exc:
            self.events.put(("error", str(exc)))

    def configure_wifi(self):
        ssid = simpledialog.askstring("Wi-Fi", "2.4 GHz 热点名称（请在开始导览前配置）：", parent=self.root)
        if not ssid:
            return
        password = simpledialog.askstring("Wi-Fi", "热点密码：", show="*", parent=self.root)
        if password is None:
            return
        if (not 1 <= len(ssid.encode("utf-8")) <= 32 or "," in ssid
                or any(c in ssid + password for c in "\r\n\x03")
                or (password and not 8 <= len(password.encode("utf-8")) <= 63)):
            messagebox.showerror("Wi-Fi", "名称需为 1–32 字节且不含逗号；密码为空或 8–63 字节，不能包含换行。")
            return
        self.voice_command("WIFI," + ssid + "," + password)

    def preview_speech(self):
        if self.serial_worker.connected:
            try:
                self.serial_worker.write(b"SAY,0\n")
            except Exception as exc:
                self.events.put(("error", str(exc)))

    def pause_tour(self):
        if self.serial_worker.connected:
            try:
                self.serial_worker.write(b"P")
            except Exception as exc:
                self.events.put(("error", str(exc)))

    def start_simple_push(self) -> None:
        try:
            self.serial_worker.write(b"F" if getattr(self, "start_line_course", False) else b"SIMPLE,1\n")
            self.simple_start_button.configure(state="disabled")
        except serial.SerialException as exc:
            self.events.put(("error", str(exc)))

    def _start_recording(self) -> None:
        stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        self.record_directory = self.repository / "captures" / f"mission-debug-{stamp}"
        (self.record_directory / "frames").mkdir(parents=True, exist_ok=True)
        (self.record_directory / "metadata").mkdir(parents=True, exist_ok=True)
        self.record_file = (self.record_directory / "telemetry.jsonl").open(
            "w", encoding="utf-8", buffering=1
        )

    def _stop_recording(self) -> None:
        if self.record_file is not None:
            self.record_file.close()
        self.record_file = None

    def _poll_events(self) -> None:
        newest: DebugFrame | None = None
        try:
            while True:
                event, payload = self.events.get_nowait()
                if event == "frame":
                    frame = payload
                    assert isinstance(frame, DebugFrame)
                    self._record_frame(frame)
                    if newest is not None:
                        self.host_dropped_frames += 1
                    newest = frame
                elif event == "bad_frame":
                    self.bad_frames += 1
                elif event == "error":
                    self.disconnect()
                    messagebox.showerror("Serial error", str(payload))
        except queue.Empty:
            pass
        if newest is not None:
            self._show_frame(newest)
        self.root.after(30, self._poll_events)

    def _show_frame(self, frame: DebugFrame) -> None:
        metadata = frame.metadata
        width = int(metadata["width"])
        height = int(metadata["height"])
        rgb888 = frame_rgb888(frame)
        if metadata.get("phase") in ("SIMPLE_RED_PUSH", "SIMPLE_BALL_PUSH"):
            metadata, rgb888 = upright_simple_frame(metadata, rgb888)
        ppm = f"P6\n{width} {height}\n255\n".encode("ascii") + rgb888
        base_photo = tk.PhotoImage(data=ppm, format="PPM")

        canvas_width = max(1, self.canvas.winfo_width())
        canvas_height = max(1, self.canvas.winfo_height())
        scale = max(1, min(canvas_width // width, canvas_height // height))
        self.photo = base_photo.zoom(scale, scale)
        shown_width = width * scale
        shown_height = height * scale
        offset_x = (canvas_width - shown_width) // 2
        offset_y = (canvas_height - shown_height) // 2
        self.canvas.delete("all")
        self.canvas.create_image(offset_x, offset_y, anchor="nw", image=self.photo)
        phase = str(metadata.get("phase", "BALL_CAPTURE"))
        self.image_panel.configure(
            text=("推球画面（已转正；橙线：车→球，青线：球→门）"
                  if phase in ("SIMPLE_RED_PUSH", "SIMPLE_BALL_PUSH") else
                  f"{phase} algorithm frame ({width}x{height}, origin: top-left)")
        )
        if phase == "LINE_FOLLOW":
            self._draw_line_result(metadata.get("line"), scale, offset_x, offset_y)
        else:
            self._draw_detection(metadata.get("goal"), "#ffd54f", scale, offset_x, offset_y)
            self._draw_detection(metadata.get("red"), "#ff3b30", scale, offset_x, offset_y)
            self._draw_detection(metadata.get("purple"), "#ff00ff", scale, offset_x, offset_y)

        if phase in ("SIMPLE_RED_PUSH", "SIMPLE_BALL_PUSH"):
            for band_y in (60, 80):
                self.canvas.create_line(offset_x, offset_y + band_y * scale,
                                        offset_x + width * scale, offset_y + band_y * scale,
                                        fill="#88bb88", dash=(3, 5))
            self.canvas.create_text(offset_x+5, offset_y+61*scale, anchor="nw",
                                    text="瞄准时球心目标区域", fill="#88bb88")
            ball_key = "purple" if metadata.get("mission", {}).get("selected_ball") == 3 else "red"
            red, target = metadata.get(ball_key, {}), metadata.get("goal", {})
            if red.get("found") and not red.get("predicted"):
                bx, by = red["center"]
                ax, ay = (width-1)/2, height-1
                self.canvas.create_line(
                    offset_x + ax * scale, offset_y + ay * scale,
                    offset_x + bx * scale, offset_y + by * scale,
                    fill="#ff9f0a", width=3, arrow=tk.LAST)
                # Extend the vehicle-to-ball ray to show a miss at the goal.
                if (target.get("found") and not target.get("predicted")
                        and target.get("corner_found")):
                    gx, gy = target["corner"]
                    self.canvas.create_line(
                        offset_x + bx * scale, offset_y + by * scale,
                        offset_x + gx * scale, offset_y + gy * scale,
                        fill="#00e5ff", width=3, arrow=tk.LAST)
                    if ay-by > 1:
                        ex = ax + (bx-ax)*(ay-gy)/(ay-by)
                        ex = max(0, min(width-1, ex))
                        self.canvas.create_line(
                            offset_x + bx * scale, offset_y + by * scale,
                            offset_x + ex * scale, offset_y + gy * scale,
                            fill="#ff9f0a", width=1, dash=(5, 4))
                    tx, ty = offset_x + gx * scale, offset_y + gy * scale
                    self.canvas.create_oval(tx-6, ty-6, tx+6, ty+6,
                                            outline="#00e5ff", width=2)
                    self.canvas.create_text(tx+9, ty, anchor="w", text="直角瞄准点",
                                            fill="#00e5ff")

        goal = metadata.get("goal")
        if phase != "LINE_FOLLOW" and isinstance(goal, dict) and goal.get("corner_found"):
            corner = goal.get("corner", [-1, -1])
            if isinstance(corner, list) and len(corner) == 2:
                x = offset_x + int(corner[0]) * scale
                y = offset_y + int(corner[1]) * scale
                radius = 5
                self.canvas.create_line(x - radius, y, x + radius, y, fill="#00ff72", width=2)
                self.canvas.create_line(x, y - radius, x, y + radius, fill="#00ff72", width=2)

        if phase in ("SIMPLE_RED_PUSH", "SIMPLE_BALL_PUSH"):
            gap = metadata.get("goal", {}).get("ball_gap_mm", -1)
            self.canvas.create_text(offset_x+5, offset_y+5, anchor="nw",
                text=(f"距球门区域约 {gap} mm；≤180 mm 停车" if gap >= 0
                      else "等待球门距离；≤180 mm 停车"), fill="#ffd54f")

        now = time.monotonic()
        self.frame_times.append(now)
        while self.frame_times and now - self.frame_times[0] > 3.0:
            self.frame_times.popleft()
        fps = 0.0
        if len(self.frame_times) > 1:
            fps = (len(self.frame_times) - 1) / (
                self.frame_times[-1] - self.frame_times[0]
            )
        sequence = int(metadata["seq"])
        if self.last_sequence is not None and sequence > self.last_sequence + 1:
            self.host_dropped_frames += sequence - self.last_sequence - 1
        self.last_sequence = sequence

        capture_us = int(metadata["capture_us"])
        processed_us = int(metadata["processed_us"])
        pipeline_ms = (processed_us - capture_us) / 1000.0
        stream = metadata.get("stream", {})
        firmware_drops = stream.get("dropped", 0) if isinstance(stream, dict) else 0
        self.stats_var.set(
            f"Frame {sequence} | {fps:.1f} fps | capture-to-decision "
            f"{pipeline_ms:.1f} ms | firmware drops {firmware_drops} | "
            f"host/GUI skips {self.host_dropped_frames} | CRC errors {self.bad_frames}"
        )

        mission = metadata.get("mission", {})
        xz = metadata.get("xiaozhi", {})
        voice_state = {0: "未连接（检查 Wi-Fi / 麦克风 / 绑定）", 1: "连接中", 2: "正在监听", 3: "回复中", 4: "监听已关闭"}.get(xz.get("state"), "固件未提供状态")
        code = xz.get("binding_code", -1)
        self.xiaozhi_var.set("小智：" + voice_state + (f"\n绑定码：{code:06d}，请到 xiaozhi.me 添加设备" if code >= 0 else ""))
        tour = metadata.get("tour", {})
        is_tour = bool(tour.get("active"))
        self.start_line_course = is_tour or phase == "LINE_FOLLOW"
        line_control = metadata.get("line", {}).get("control", {})
        can_start = (self.start_line_course and not line_control.get("enabled", False)) or (
            phase in ("SIMPLE_RED_PUSH", "SIMPLE_BALL_PUSH") and
            isinstance(mission, dict) and mission.get("state") == "WAIT_START")
        self.pause_button.configure(state="normal" if is_tour and self.serial_worker.connected and bool(line_control.get("enabled")) else "disabled")
        if is_tour:
            can_start = bool(tour.get("can_start"))
        self.simple_start_button.configure(
            text=("开始" if line_control.get("state")=="WAIT_START" else "起步 / 继续") if is_tour else ("开始循迹→避障→推球" if self.start_line_course else "开始红球→紫球"),
            state="normal" if can_start and self.serial_worker.connected else "disabled"
        )
        navigation = metadata.get("navigation", {})
        line = metadata.get("line", {})
        if phase == "LINE_FOLLOW" and isinstance(line, dict):
            foot = line.get("foot", {})
            turn = line.get("turn", {})
            control = line.get("control", {})
            errors = line.get("errors", {})
            self.mission_var.set(
                f"Line: found {int(bool(line.get('found')))} | confidence "
                f"{line.get('confidence', 0)} | threshold {line.get('threshold', 0)} | "
                f"contrast {line.get('contrast', 0)}"
            )
            if isinstance(control, dict):
                self.control_var.set(
                    f"Control: {control.get('state', '?')} | enabled "
                    f"{int(bool(control.get('enabled')))} | PWM {control.get('pwm', [])} | "
                    f"boost {control.get('boost', [])} | enc {control.get('encoder_delta', [])} | "
                    f"distance {control.get('distance_mm', -1)} mm"
                )
            if isinstance(foot, dict) and isinstance(turn, dict) and isinstance(errors, dict):
                self.objects_var.set(
                    f"Track: foot {int(bool(foot.get('valid')))} centered "
                    f"{int(bool(foot.get('centered')))} | errors L/H/S="
                    f"{errors.get('lateral', 0)}/{errors.get('heading', 0)}/"
                    f"{errors.get('steering', 0)} | turn dir={turn.get('direction', 0)} "
                    f"angle={turn.get('angle_deg', 0)} confidence={turn.get('confidence', 0)}"
                )
        elif isinstance(mission, dict):
            ball_names = {0: "NONE", 1: "RED", 3: "PURPLE"}
            goal_names = {0: "UPPER", 1: "LOWER"}
            selected = ball_names.get(int(mission.get("selected_ball", 0)), "?")
            target = ("BLACK CORNER" if phase in ("SIMPLE_RED_PUSH", "SIMPLE_BALL_PUSH")
                      else goal_names.get(int(mission.get("target_goal", -1)), "?"))
            self.mission_var.set(
                f"Mission: {mission.get('state', '?')} | selected {selected} | "
                f"goal {target} | ball held {int(bool(mission.get('ball_held')))}"
                + (f" | reason {mission.get('push_entry', '?')}"
                   if phase in ("SIMPLE_RED_PUSH", "SIMPLE_BALL_PUSH") else "")
            )
        if phase != "LINE_FOLLOW" and isinstance(navigation, dict):
            source = navigation.get("source", "?")
            pose = navigation.get("pose_mm_deg", [0, 0, 0])
            target = navigation.get("target_field_mm", [0, 0])
            distance = navigation.get("distance_mm", 0)
            visual = navigation.get("visual_target_mm", [0, 0])
            if source == "VISION":
                detail = f"target right={visual[0]} mm, forward={visual[1]} mm"
            else:
                detail = (
                    f"pose=({pose[0]},{pose[1]},{pose[2]} deg), "
                    f"target=({target[0]},{target[1]}), remaining={distance} mm"
                )
            self.control_var.set(f"Control: {source} | {detail}")
        if phase != "LINE_FOLLOW":
            self.objects_var.set(
                "Objects: " + " | ".join(
                    self._object_summary(name, metadata.get(name))
                    for name in ("red", "purple", "goal")
                )
            )

        if is_tour:
            route_name = {0: "等待数字", 1: "左路", 2: "右路", -1: "停车锁定"}.get(tour.get("route"), "未知")
            self.mission_var.set(
                f"导览: {route_name} | 数字 {tour.get('digit', -1)} | 置信度 {tour.get('score', 0)}% | "
                f"起点转角 {tour.get('turn_deg', 0)}° / {tour.get('turn_target', 0)}° | 拐点 {tour.get('corners', 0)} | 最少停留 {tour.get('dwell_ms', 0)} ms | "
                f"喇叭 {'故障' if tour.get('audio_failed') else '播报中' if tour.get('audio_busy') else '就绪' if tour.get('audio_ready') else '未连接'}")
            if phase == "TOUR_DIGIT":
                state = line_control.get("state", "WAIT_START")
                self.control_var.set({"WAIT_START":"点击开始：先播放欢迎词，再识别数字", "WELCOME":"欢迎词播放中，暂不识别数字", "WAIT_DIGIT":"请展示 1 或 2；1 左转30度，2 右转30度"}.get(state,state))
                self.objects_var.set("实时相机原始画面；TFT 同时显示识别框与数字")
        self.metadata_text.configure(state="normal")
        self.metadata_text.delete("1.0", "end")
        self.metadata_text.insert("1.0", json.dumps(metadata, indent=2, ensure_ascii=False))
        self.metadata_text.configure(state="disabled")

    def _draw_line_result(
        self, value: object, scale: int, offset_x: int, offset_y: int
    ) -> None:
        if not isinstance(value, dict) or not value.get("found"):
            return
        near = value.get("near", [-1, -1])
        far = value.get("far", [-1, -1])
        if not (
            isinstance(near, list)
            and len(near) == 2
            and isinstance(far, list)
            and len(far) == 2
        ):
            return
        nx, ny = int(near[0]), int(near[1])
        fx, fy = int(far[0]), int(far[1])
        if min(nx, ny, fx, fy) < 0:
            return
        nx = offset_x + nx * scale
        ny = offset_y + ny * scale
        fx = offset_x + fx * scale
        fy = offset_y + fy * scale
        self.canvas.create_line(nx, ny, fx, fy, fill="#00e5ff", width=2)
        for x, y, color in ((nx, ny, "#00ff72"), (fx, fy, "#ffcc00")):
            radius = max(3, scale)
            self.canvas.create_line(x - radius, y, x + radius, y, fill=color, width=2)
            self.canvas.create_line(x, y - radius, x, y + radius, fill=color, width=2)

    def _draw_detection(
        self, value: object, color: str, scale: int, offset_x: int, offset_y: int
    ) -> None:
        if not isinstance(value, dict) or not value.get("found"):
            return
        box = value.get("box")
        if not isinstance(box, list) or len(box) != 4:
            return
        left, top, right, bottom = (int(item) for item in box)
        self.canvas.create_rectangle(
            offset_x + left * scale,
            offset_y + top * scale,
            offset_x + (right + 1) * scale,
            offset_y + (bottom + 1) * scale,
            outline=color,
            width=2,
        )

    @staticmethod
    def _object_summary(name: str, value: object) -> str:
        if not isinstance(value, dict):
            return f"{name}=--"
        real = bool(value.get("found")) and not bool(value.get("predicted"))
        center = value.get("center", [-1, -1])
        return f"{name}={int(real)}/{value.get('confidence', 0)}@{center}"

    def _record_frame(self, frame: DebugFrame) -> None:
        if self.record_file is None or self.record_directory is None:
            return
        sequence = int(frame.metadata["seq"])
        self.record_file.write(json.dumps(frame.metadata, ensure_ascii=False) + "\n")
        width = int(frame.metadata["width"])
        height = int(frame.metadata["height"])
        ppm = (
            f"P6\n{width} {height}\n255\n".encode("ascii")
            + frame_rgb888(frame)
        )
        frame_path = self.record_directory / "frames" / f"{sequence:08d}.ppm"
        frame_path.write_bytes(ppm)
        if frame.metadata.get("format") == "RGB565BE":
            frame_path.with_suffix(".rgb565").write_bytes(frame.pixels)
        metadata_path = self.record_directory / "metadata" / f"{sequence:08d}.json"
        metadata_path.write_text(
            json.dumps(frame.metadata, indent=2, ensure_ascii=False), encoding="utf-8"
        )

    def close(self) -> None:
        self.disconnect()
        self.root.destroy()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="", help="initial serial port, e.g. COM15")
    parser.add_argument("--connect", action="store_true", help="connect on startup")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    repository = Path(__file__).resolve().parent
    root = tk.Tk()
    PushDebugViewer(root, args.port, repository, args.connect)
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
