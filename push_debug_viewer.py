"""Live, synchronized camera and navigation debugger for ball pushing."""

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
from tkinter import messagebox, ttk
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
    rgb332: bytes


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
        port = serial.Serial(
            port_name,
            NORMAL_BAUD,
            timeout=0.05,
            write_timeout=1.0,
            rtscts=False,
            xonxoff=False,
        )
        port.dtr = False
        port.rts = False
        port.reset_input_buffer()

        handshake = bytearray()
        deadline = time.monotonic() + 2.5
        next_request = 0.0
        while SWITCH_MARKER not in handshake and time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_request:
                port.write(b"VSTREAM,1\n")
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
                if metadata.get("format") != "RGB332" or len(payload) != width * height:
                    raise ValueError("invalid frame geometry")
            except (UnicodeDecodeError, ValueError, KeyError, TypeError) as exc:
                self.events.put(("bad_frame", str(exc)))
                del receive[:packet_length]
                continue
            del receive[:packet_length]
            self.events.put(("frame", DebugFrame(metadata, payload)))


class PushDebugViewer:
    def __init__(self, root: tk.Tk, initial_port: str, repository: Path) -> None:
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

        root.title("ESP32 Push Vision Debugger")
        root.geometry("1040x700")
        root.minsize(900, 620)
        root.protocol("WM_DELETE_WINDOW", self.close)

        self.port_var = tk.StringVar(value=initial_port)
        self.status_var = tk.StringVar(value="Disconnected")
        self.stats_var = tk.StringVar(value="Waiting for frames")
        self.control_var = tk.StringVar(value="Control: --")
        self.mission_var = tk.StringVar(value="Mission: --")
        self.objects_var = tk.StringVar(value="Objects: --")
        self.record_var = tk.BooleanVar(value=True)
        self._build_ui()
        self.refresh_ports()
        root.after(30, self._poll_events)

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
        ttk.Button(toolbar, text="Emergency stop", command=self.emergency_stop).pack(
            side="left", padx=(8, 0)
        )
        ttk.Checkbutton(
            toolbar, text="Record frames", variable=self.record_var
        ).pack(side="left", padx=12)
        ttk.Label(toolbar, textvariable=self.status_var).pack(side="right")

        content = ttk.Panedwindow(outer, orient="horizontal")
        content.pack(fill="both", expand=True, pady=(10, 0))
        image_panel = ttk.LabelFrame(
            content, text="160x120 algorithm frame (pixel origin: top-left)", padding=8
        )
        details_panel = ttk.LabelFrame(content, text="Synchronized decision", padding=8)
        content.add(image_panel, weight=3)
        content.add(details_panel, weight=2)

        self.canvas = tk.Canvas(
            image_panel, width=640, height=480, background="#111111",
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
        ttk.Label(details_panel, text="Frame metadata").pack(anchor="w")
        self.metadata_text = tk.Text(details_panel, height=20, width=46, wrap="none")
        self.metadata_text.pack(fill="both", expand=True, pady=(4, 0))
        self.metadata_text.configure(state="disabled")

    def refresh_ports(self) -> None:
        ports = sorted(list_ports.comports(), key=lambda item: item.device)
        devices = [item.device.upper() for item in ports]
        self.port_descriptions = {
            item.device.upper(): item.description or "Serial device" for item in ports
        }
        self.port_box["values"] = devices
        if self.port_var.get().upper() not in devices:
            self.port_var.set(devices[0] if len(devices) == 1 else "")
        if not devices:
            self.status_var.set("No serial port found")

    def toggle_connection(self) -> None:
        if self.serial_worker.connected:
            self.disconnect()
            return
        port_name = self.port_var.get().strip().upper()
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
        self.serial_worker.disconnect()
        self._stop_recording()
        self.connect_button.configure(text="Connect")
        self.status_var.set("Disconnected")

    def emergency_stop(self) -> None:
        try:
            self.serial_worker.write(b"X")
        except serial.SerialException as exc:
            self.events.put(("error", str(exc)))

    def _start_recording(self) -> None:
        stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        self.record_directory = self.repository / "captures" / f"push-debug-{stamp}"
        (self.record_directory / "frames").mkdir(parents=True, exist_ok=True)
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
        rgb888 = rgb332_to_rgb888(frame.rgb332)
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
        self._draw_detection(metadata.get("goal"), "#ffd54f", scale, offset_x, offset_y)
        self._draw_detection(metadata.get("red"), "#ff3b30", scale, offset_x, offset_y)
        self._draw_detection(metadata.get("white"), "#00e5ff", scale, offset_x, offset_y)
        self._draw_detection(metadata.get("purple"), "#ff00ff", scale, offset_x, offset_y)

        goal = metadata.get("goal")
        if isinstance(goal, dict) and goal.get("corner_found"):
            corner = goal.get("corner", [-1, -1])
            if isinstance(corner, list) and len(corner) == 2:
                x = offset_x + int(corner[0]) * scale
                y = offset_y + int(corner[1]) * scale
                radius = 5
                self.canvas.create_line(x - radius, y, x + radius, y, fill="#00ff72", width=2)
                self.canvas.create_line(x, y - radius, x, y + radius, fill="#00ff72", width=2)

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
        navigation = metadata.get("navigation", {})
        if isinstance(mission, dict):
            ball_names = {0: "NONE", 1: "RED", 2: "WHITE", 3: "PURPLE"}
            goal_names = {0: "UPPER", 1: "LOWER"}
            selected = ball_names.get(int(mission.get("selected_ball", 0)), "?")
            target = goal_names.get(int(mission.get("target_goal", -1)), "?")
            self.mission_var.set(
                f"Mission: {mission.get('state', '?')} | selected {selected} | "
                f"goal {target} | ball held {int(bool(mission.get('ball_held')))}"
            )
        if isinstance(navigation, dict):
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
        self.objects_var.set(
            "Objects: " + " | ".join(
                self._object_summary(name, metadata.get(name))
                for name in ("red", "white", "purple", "goal")
            )
        )

        self.metadata_text.configure(state="normal")
        self.metadata_text.delete("1.0", "end")
        self.metadata_text.insert("1.0", json.dumps(metadata, indent=2, ensure_ascii=False))
        self.metadata_text.configure(state="disabled")
        self._record_frame(frame, ppm)

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

    def _record_frame(self, frame: DebugFrame, ppm: bytes) -> None:
        if self.record_file is None or self.record_directory is None:
            return
        sequence = int(frame.metadata["seq"])
        self.record_file.write(json.dumps(frame.metadata, ensure_ascii=False) + "\n")
        frame_path = self.record_directory / "frames" / f"{sequence:08d}.ppm"
        frame_path.write_bytes(ppm)

    def close(self) -> None:
        self.disconnect()
        self.root.destroy()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="", help="initial serial port, e.g. COM15")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    repository = Path(__file__).resolve().parent
    root = tk.Tk()
    PushDebugViewer(root, args.port.upper(), repository)
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
