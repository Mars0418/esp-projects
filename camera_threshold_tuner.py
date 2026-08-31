"""Interactive RGB threshold tuner for camera-line-follow-test firmware."""

from __future__ import annotations

import argparse
import base64
from collections import deque
from dataclasses import dataclass
import queue
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk

try:
    import serial
    from serial.tools import list_ports
except (ImportError, ModuleNotFoundError) as exc:
    raise SystemExit(
        "pyserial is required. Run start_camera_threshold_tuner.ps1 so the "
        "ESP-IDF Python environment is selected automatically."
    ) from exc

if not hasattr(serial, "Serial"):
    raise SystemExit(
        "The installed 'serial' package is not pyserial. Run "
        "start_camera_threshold_tuner.ps1 instead of invoking python directly."
    )


@dataclass(frozen=True)
class DebugFrame:
    sequence: int
    width: int
    height: int
    red_threshold: int
    green_threshold: int
    blue_threshold: int
    found: bool
    confidence: int
    steering_error: int
    near_x: int
    far_x: int
    big_turn: bool
    turn_direction: int
    turn_angle_deg: int
    turn_confidence: int
    corner_y: int
    rgb565: bytes
    mask: bytes


@dataclass(frozen=True)
class TunerHeader:
    sequence: int
    width: int
    height: int
    red_threshold: int
    green_threshold: int
    blue_threshold: int
    found: bool
    confidence: int
    steering_error: int
    near_x: int
    far_x: int
    big_turn: bool
    turn_direction: int
    turn_angle_deg: int
    turn_confidence: int
    corner_y: int
    rgb_bytes: int
    mask_bytes: int


TUNER_MAGIC = b"@RGB565,"
TUNER_BAUD = 921600


def parse_tuner_header(line: bytes) -> TunerHeader:
    fields = line.decode("ascii").strip().split(",")
    if len(fields) != 19 or fields[0] != "@RGB565":
        raise ValueError("not an RGB565 tuner frame")
    values = [int(value) for value in fields[1:]]
    width = values[1]
    height = values[2]
    rgb_bytes = values[16]
    mask_bytes = values[17]
    if width <= 0 or height <= 0 or width * height > 65536:
        raise ValueError("invalid tuner frame dimensions")
    if rgb_bytes != width * height * 2:
        raise ValueError("invalid tuner RGB565 length")
    if mask_bytes != (width * height + 7) // 8:
        raise ValueError("invalid tuner mask length")
    return TunerHeader(
        sequence=values[0],
        width=width,
        height=height,
        red_threshold=values[3],
        green_threshold=values[4],
        blue_threshold=values[5],
        found=bool(values[6]),
        confidence=values[7],
        steering_error=values[8],
        near_x=values[9],
        far_x=values[10],
        big_turn=bool(values[11]),
        turn_direction=values[12],
        turn_angle_deg=values[13],
        turn_confidence=values[14],
        corner_y=values[15],
        rgb_bytes=rgb_bytes,
        mask_bytes=mask_bytes,
    )


def parse_debug_frame(line: str) -> DebugFrame:
    fields = line.strip().split(",", 18)
    if len(fields) != 19 or fields[0] != "@RGB":
        raise ValueError("not an RGB debug frame")

    width = int(fields[2])
    height = int(fields[3])
    if width <= 0 or height <= 0 or width * height > 65536:
        raise ValueError("invalid debug frame dimensions")

    rgb565 = base64.b64decode(fields[17], validate=True)
    mask = base64.b64decode(fields[18], validate=True)
    expected_rgb_bytes = width * height * 2
    expected_mask_bytes = (width * height + 7) // 8
    if len(rgb565) != expected_rgb_bytes:
        raise ValueError("invalid RGB565 payload length")
    if len(mask) != expected_mask_bytes:
        raise ValueError("invalid mask payload length")

    return DebugFrame(
        sequence=int(fields[1]),
        width=width,
        height=height,
        red_threshold=int(fields[4]),
        green_threshold=int(fields[5]),
        blue_threshold=int(fields[6]),
        found=bool(int(fields[7])),
        confidence=int(fields[8]),
        steering_error=int(fields[9]),
        near_x=int(fields[10]),
        far_x=int(fields[11]),
        big_turn=bool(int(fields[12])),
        turn_direction=int(fields[13]),
        turn_angle_deg=int(fields[14]),
        turn_confidence=int(fields[15]),
        corner_y=int(fields[16]),
        rgb565=rgb565,
        mask=mask,
    )


def rgb565_to_rgb888(rgb565: bytes) -> bytes:
    rgb888 = bytearray(len(rgb565) // 2 * 3)
    destination = 0
    for source in range(0, len(rgb565), 2):
        color = (rgb565[source] << 8) | rgb565[source + 1]
        red5 = (color >> 11) & 0x1F
        green6 = (color >> 5) & 0x3F
        blue5 = color & 0x1F
        rgb888[destination] = (red5 << 3) | (red5 >> 2)
        rgb888[destination + 1] = (green6 << 2) | (green6 >> 4)
        rgb888[destination + 2] = (blue5 << 3) | (blue5 >> 2)
        destination += 3
    return bytes(rgb888)


def build_recognition_overlay(rgb888: bytes, mask: bytes) -> bytes:
    overlay = bytearray(len(rgb888))
    for pixel in range(len(rgb888) // 3):
        destination = pixel * 3
        if mask[pixel // 8] & (1 << (pixel % 8)):
            overlay[destination : destination + 3] = b"\x00\xff\x40"
        else:
            overlay[destination] = rgb888[destination] * 2 // 5
            overlay[destination + 1] = rgb888[destination + 1] * 2 // 5
            overlay[destination + 2] = rgb888[destination + 2] * 2 // 5
    return bytes(overlay)


class SerialWorker:
    def __init__(self, event_queue: queue.Queue[tuple[str, object]]) -> None:
        self._event_queue = event_queue
        self._port: serial.Serial | None = None
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._reset_stream = threading.Event()
        self._write_lock = threading.Lock()

    @property
    def connected(self) -> bool:
        return self._port is not None and self._port.is_open

    def connect(self, port_name: str, baud: int) -> None:
        if self.connected:
            return
        port = serial.Serial(
            port_name,
            baud,
            timeout=0.1,
            write_timeout=0.5,
            rtscts=False,
            xonxoff=False,
        )
        port.dtr = False
        port.rts = False
        port.reset_input_buffer()
        self._port = port
        self._stop_event.clear()
        self._reset_stream.clear()
        self._thread = threading.Thread(
            target=self._read_loop, name="camera-debug-serial", daemon=True
        )
        self._thread.start()

    def write(self, data: bytes) -> None:
        port = self._port
        if port is None or not port.is_open:
            return
        with self._write_lock:
            port.write(data)
            port.flush()

    def switch_baud(self, baud: int) -> None:
        port = self._port
        if port is None or not port.is_open:
            return
        with self._write_lock:
            port.baudrate = baud
            port.reset_input_buffer()
            self._reset_stream.set()

    def disconnect(self, restore_logs: bool = True) -> None:
        port = self._port
        if port is None:
            return
        if port.is_open and restore_logs:
            try:
                self.write(b"xTUNER,0\n")
                time.sleep(0.08)
            except serial.SerialException:
                pass
        self._stop_event.set()
        try:
            port.close()
        except serial.SerialException:
            pass
        thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=0.5)
        self._thread = None
        self._port = None

    def _read_loop(self) -> None:
        assert self._port is not None
        receive_buffer = bytearray()
        pending_header: TunerHeader | None = None
        while not self._stop_event.is_set():
            if self._reset_stream.is_set():
                receive_buffer.clear()
                pending_header = None
                self._reset_stream.clear()
            try:
                waiting = self._port.in_waiting
                data = self._port.read(waiting if waiting else 1)
            except serial.SerialException as exc:
                if not self._stop_event.is_set():
                    self._event_queue.put(("error", str(exc)))
                return
            if not data:
                continue
            receive_buffer.extend(data)

            while True:
                if pending_header is None:
                    magic_index = receive_buffer.find(TUNER_MAGIC)
                    if magic_index < 0:
                        keep = min(len(receive_buffer), len(TUNER_MAGIC) - 1)
                        if len(receive_buffer) > keep:
                            del receive_buffer[: len(receive_buffer) - keep]
                        break
                    if magic_index > 0:
                        del receive_buffer[:magic_index]
                    newline_index = receive_buffer.find(b"\n")
                    if newline_index < 0:
                        break
                    header_line = bytes(receive_buffer[: newline_index + 1])
                    del receive_buffer[: newline_index + 1]
                    try:
                        pending_header = parse_tuner_header(header_line)
                    except (UnicodeDecodeError, ValueError) as exc:
                        self._event_queue.put(("warning", str(exc)))
                        pending_header = None
                        continue

                payload_bytes = (
                    pending_header.rgb_bytes + pending_header.mask_bytes
                )
                if len(receive_buffer) < payload_bytes:
                    break
                rgb565 = bytes(receive_buffer[: pending_header.rgb_bytes])
                mask_start = pending_header.rgb_bytes
                mask_end = mask_start + pending_header.mask_bytes
                mask = bytes(receive_buffer[mask_start:mask_end])
                del receive_buffer[:mask_end]
                frame = DebugFrame(
                    sequence=pending_header.sequence,
                    width=pending_header.width,
                    height=pending_header.height,
                    red_threshold=pending_header.red_threshold,
                    green_threshold=pending_header.green_threshold,
                    blue_threshold=pending_header.blue_threshold,
                    found=pending_header.found,
                    confidence=pending_header.confidence,
                    steering_error=pending_header.steering_error,
                    near_x=pending_header.near_x,
                    far_x=pending_header.far_x,
                    big_turn=pending_header.big_turn,
                    turn_direction=pending_header.turn_direction,
                    turn_angle_deg=pending_header.turn_angle_deg,
                    turn_confidence=pending_header.turn_confidence,
                    corner_y=pending_header.corner_y,
                    rgb565=rgb565,
                    mask=mask,
                )
                pending_header = None
                self._event_queue.put(("frame", frame))


class ThresholdTunerApp:
    def __init__(self, root: tk.Tk, initial_port: str, baud: int) -> None:
        self.root = root
        self.baud = baud
        self.events: queue.Queue[tuple[str, object]] = queue.Queue()
        self.serial_worker = SerialWorker(self.events)
        self.frame_times: deque[float] = deque(maxlen=20)
        self.threshold_after_id: str | None = None
        self.raw_photo: tk.PhotoImage | None = None
        self.overlay_photo: tk.PhotoImage | None = None
        self.port_descriptions: dict[str, str] = {}

        root.title("ESP32 摄像头巡线阈值调节")
        root.resizable(False, False)
        root.protocol("WM_DELETE_WINDOW", self.close)

        self.port_var = tk.StringVar(value=initial_port)
        self.status_var = tk.StringVar(value="未连接")
        self.applied_var = tk.StringVar(value="固件阈值：--")
        self.result_var = tk.StringVar(value="等待图像")
        self.turn_var = tk.StringVar(value="拐点：--")
        self.red_var = tk.IntVar(value=105)
        self.green_var = tk.IntVar(value=105)
        self.blue_var = tk.IntVar(value=105)

        self._build_ui()
        self.refresh_ports()
        root.after(40, self._poll_events)

    def _build_ui(self) -> None:
        outer = ttk.Frame(self.root, padding=12)
        outer.grid(sticky="nsew")

        connection = ttk.Frame(outer)
        connection.grid(row=0, column=0, columnspan=2, sticky="ew")
        ttk.Label(connection, text="串口").grid(row=0, column=0, padx=(0, 6))
        self.port_box = ttk.Combobox(
            connection, textvariable=self.port_var, width=12, state="readonly"
        )
        self.port_box.grid(row=0, column=1)
        ttk.Button(connection, text="刷新", command=self.refresh_ports).grid(
            row=0, column=2, padx=6
        )
        self.connect_button = ttk.Button(
            connection, text="连接", command=self.toggle_connection
        )
        self.connect_button.grid(row=0, column=3)
        ttk.Label(connection, textvariable=self.status_var).grid(
            row=0, column=4, padx=(10, 0)
        )

        self.raw_label = self._image_panel(outer, "相机原图", 1, 0)
        self.overlay_label = self._image_panel(outer, "固件识别结果", 1, 1)

        controls = ttk.LabelFrame(outer, text="RGB 黑色阈值", padding=10)
        controls.grid(row=2, column=0, columnspan=2, sticky="ew", pady=(12, 0))
        self._add_slider(controls, "R", self.red_var, 0)
        self._add_slider(controls, "G", self.green_var, 1)
        self._add_slider(controls, "B", self.blue_var, 2)

        details = ttk.Frame(outer)
        details.grid(row=3, column=0, columnspan=2, sticky="ew", pady=(10, 0))
        ttk.Label(details, textvariable=self.applied_var).grid(row=0, column=0, sticky="w")
        ttk.Label(details, textvariable=self.result_var).grid(row=1, column=0, sticky="w")
        ttk.Label(details, textvariable=self.turn_var).grid(row=2, column=0, sticky="w")

    def _image_panel(
        self, parent: ttk.Frame, title: str, row: int, column: int
    ) -> ttk.Label:
        panel = ttk.LabelFrame(parent, text=title, padding=6)
        panel.grid(row=row, column=column, padx=(0, 8) if column == 0 else 0,
                   pady=(12, 0))
        label = ttk.Label(panel, text="等待数据", anchor="center", width=34)
        label.grid()
        return label

    def _add_slider(
        self, parent: ttk.LabelFrame, name: str, variable: tk.IntVar, row: int
    ) -> None:
        ttk.Label(parent, text=name, width=2).grid(row=row, column=0)
        scale = tk.Scale(
            parent,
            from_=0,
            to=255,
            orient="horizontal",
            resolution=1,
            showvalue=True,
            length=500,
            variable=variable,
            command=lambda _value: self.schedule_threshold_update(),
        )
        scale.grid(row=row, column=1, sticky="ew")

    def refresh_ports(self) -> None:
        ports = sorted(list_ports.comports(), key=lambda port: port.device)
        devices = [port.device for port in ports]
        self.port_descriptions = {
            port.device: port.description or "串口设备" for port in ports
        }
        self.port_box["values"] = devices
        if self.port_var.get() not in devices:
            self.port_var.set(devices[0] if len(devices) == 1 else "")
        if not devices:
            self.status_var.set("未发现串口")

    def toggle_connection(self) -> None:
        if self.serial_worker.connected:
            self.disconnect()
            return
        port_name = self.port_var.get().strip()
        if not port_name:
            messagebox.showerror("无法连接", "请选择 ESP32 对应的 COM 口。")
            return
        try:
            self.serial_worker.connect(port_name, self.baud)
            self.serial_worker.write(b"xTUNER,1\n")
            time.sleep(0.08)
            self.serial_worker.switch_baud(TUNER_BAUD)
            self._send_thresholds()
        except serial.SerialException as exc:
            self.serial_worker.disconnect(restore_logs=False)
            messagebox.showerror("无法连接", str(exc))
            return
        description = self.port_descriptions.get(port_name, "")
        self.status_var.set(
            f"已连接 {port_name} @ {TUNER_BAUD}  {description}"
        )
        self.connect_button.configure(text="断开")

    def disconnect(self) -> None:
        self.serial_worker.disconnect()
        self.status_var.set("未连接")
        self.connect_button.configure(text="连接")

    def schedule_threshold_update(self) -> None:
        if self.threshold_after_id is not None:
            self.root.after_cancel(self.threshold_after_id)
        self.threshold_after_id = self.root.after(120, self._send_thresholds)

    def _send_thresholds(self) -> None:
        self.threshold_after_id = None
        if not self.serial_worker.connected:
            return
        command = (
            f"RGB,{self.red_var.get()},{self.green_var.get()},"
            f"{self.blue_var.get()}\n"
        )
        try:
            self.serial_worker.write(command.encode("ascii"))
        except serial.SerialException as exc:
            self.events.put(("error", str(exc)))

    def _poll_events(self) -> None:
        newest_frame: DebugFrame | None = None
        try:
            while True:
                event, payload = self.events.get_nowait()
                if event == "frame":
                    newest_frame = payload  # type: ignore[assignment]
                elif event == "error":
                    self.disconnect()
                    messagebox.showerror("串口错误", str(payload))
                elif event == "warning":
                    self.status_var.set(f"收到无效帧：{payload}")
        except queue.Empty:
            pass
        if newest_frame is not None:
            self._show_frame(newest_frame)
        self.root.after(40, self._poll_events)

    def _show_frame(self, frame: DebugFrame) -> None:
        rgb888 = rgb565_to_rgb888(frame.rgb565)
        overlay = build_recognition_overlay(rgb888, frame.mask)
        scale = max(1, 320 // frame.width)
        self.raw_photo = self._make_photo(rgb888, frame.width, frame.height, scale)
        self.overlay_photo = self._make_photo(
            overlay, frame.width, frame.height, scale
        )
        self.raw_label.configure(image=self.raw_photo, text="")
        self.overlay_label.configure(image=self.overlay_photo, text="")

        now = time.monotonic()
        self.frame_times.append(now)
        while self.frame_times and now - self.frame_times[0] > 3.0:
            self.frame_times.popleft()
        fps = 0.0
        if len(self.frame_times) >= 2:
            fps = (len(self.frame_times) - 1) / (
                self.frame_times[-1] - self.frame_times[0]
            )
        self.applied_var.set(
            "固件阈值："
            f"R={frame.red_threshold} G={frame.green_threshold} "
            f"B={frame.blue_threshold}  预览={fps:.1f}fps"
        )
        self.result_var.set(
            f"识别：line={int(frame.found)} confidence={frame.confidence} "
            f"steering={frame.steering_error} near={frame.near_x} far={frame.far_x}"
        )
        self.turn_var.set(
            f"拐点：big={int(frame.big_turn)} direction={frame.turn_direction} "
            f"angle={frame.turn_angle_deg} confidence={frame.turn_confidence} "
            f"y={frame.corner_y}  sequence={frame.sequence}"
        )

    @staticmethod
    def _make_photo(
        rgb888: bytes, width: int, height: int, scale: int
    ) -> tk.PhotoImage:
        image = tk.PhotoImage(width=width, height=height)
        for y in range(height):
            row = []
            for x in range(width):
                offset = (y * width + x) * 3
                row.append(
                    f"#{rgb888[offset]:02x}{rgb888[offset + 1]:02x}"
                    f"{rgb888[offset + 2]:02x}"
                )
            image.put("{" + " ".join(row) + "}", to=(0, y))
        return image.zoom(scale, scale)

    def close(self) -> None:
        if self.threshold_after_id is not None:
            self.root.after_cancel(self.threshold_after_id)
        self.serial_worker.disconnect()
        self.root.destroy()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Tune ESP32 camera line-following RGB thresholds."
    )
    parser.add_argument("--port", default="", help="initial serial port")
    parser.add_argument("--baud", type=int, default=115200)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    root = tk.Tk()
    ThresholdTunerApp(root, args.port.upper(), args.baud)
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
