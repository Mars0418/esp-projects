"""Collect ROI-only camera line-following training samples from ESP32."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from datetime import datetime, timezone
import json
from pathlib import Path
import struct
import time
import zlib

try:
    import serial
    from serial.tools import list_ports
except (ImportError, ModuleNotFoundError) as exc:
    raise SystemExit("pyserial is required; use start_camera_training_collector.ps1") from exc

from camera_threshold_tuner import (
    TUNER_BAUD,
    TUNER_MAGIC,
    parse_tuner_header,
    rgb565_to_rgb888,
)


ROI_X_MIN = 27
ROI_X_MAX = 133
ROI_Y_MIN = 8
ROI_Y_MAX = 116
ROI_WIDTH = ROI_X_MAX - ROI_X_MIN
ROI_HEIGHT = ROI_Y_MAX - ROI_Y_MIN
NORMAL_BAUD = 115200
RESULT_MAGIC = b"@RESULT,"


@dataclass(frozen=True)
class AlgorithmResult:
    sequence: int
    found: bool
    big_turn: bool
    near_x: int
    far_x: int
    lateral_error: int
    heading_error: int
    steering_error: int
    steering_band_valid: bool
    steering_band_left_percent: int
    steering_band_right_percent: int
    steering_band_error: int
    steering_band_pixel_count: int
    turn_direction: int
    turn_angle_deg: int
    turn_confidence: int
    corner_x: int
    corner_y: int
    vector_point_count: int
    path_point_count: int
    confidence: int
    threshold: int
    contrast: int
    component_area: int
    path_points: tuple[tuple[int, int], ...]


def parse_result_line(line: bytes) -> AlgorithmResult:
    fields = line.decode("ascii").strip().split(",")
    if len(fields) < 25 or fields[0] != "@RESULT":
        raise ValueError("invalid algorithm result line")
    values = [int(value) for value in fields[1:25]]
    path_count = values[19]
    if path_count < 0 or path_count > 20:
        raise ValueError("invalid path point count")
    if len(fields) != 25 + path_count * 2:
        raise ValueError("algorithm result path length mismatch")
    point_values = [int(value) for value in fields[25:]]
    points = tuple(
        (point_values[index], point_values[index + 1])
        for index in range(0, len(point_values), 2)
    )
    return AlgorithmResult(
        sequence=values[0],
        found=bool(values[1]),
        big_turn=bool(values[2]),
        near_x=values[3],
        far_x=values[4],
        lateral_error=values[5],
        heading_error=values[6],
        steering_error=values[7],
        steering_band_valid=bool(values[8]),
        steering_band_left_percent=values[9],
        steering_band_right_percent=values[10],
        steering_band_error=values[11],
        steering_band_pixel_count=values[12],
        turn_direction=values[13],
        turn_angle_deg=values[14],
        turn_confidence=values[15],
        corner_x=values[16],
        corner_y=values[17],
        vector_point_count=values[18],
        path_point_count=path_count,
        confidence=values[20],
        threshold=values[21],
        contrast=values[22],
        component_area=values[23],
        path_points=points,
    )


class TunerStream:
    def __init__(self, port: serial.Serial) -> None:
        self.port = port
        self.buffer = bytearray()

    def _read_more(self) -> None:
        waiting = self.port.in_waiting
        data = self.port.read(waiting if waiting else 1)
        if data:
            self.buffer.extend(data)

    def _line_from_magic(self, magic: bytes) -> bytes:
        while True:
            magic_index = self.buffer.find(magic)
            if magic_index >= 0:
                if magic_index:
                    del self.buffer[:magic_index]
                newline_index = self.buffer.find(b"\n")
                if newline_index >= 0:
                    line = bytes(self.buffer[: newline_index + 1])
                    del self.buffer[: newline_index + 1]
                    return line
            elif len(self.buffer) > len(magic) - 1:
                del self.buffer[: len(self.buffer) - (len(magic) - 1)]
            self._read_more()

    def _read_bytes(self, count: int) -> bytes:
        while len(self.buffer) < count:
            self._read_more()
        data = bytes(self.buffer[:count])
        del self.buffer[:count]
        return data

    def read_sample(self) -> tuple[object, bytes, bytes, AlgorithmResult]:
        while True:
            header = parse_tuner_header(self._line_from_magic(TUNER_MAGIC))
            rgb565 = self._read_bytes(header.rgb_bytes)
            mask = self._read_bytes(header.mask_bytes)
            result = parse_result_line(self._line_from_magic(RESULT_MAGIC))
            if result.sequence == header.sequence:
                return header, rgb565, mask, result


def choose_port(requested: str) -> str:
    if requested:
        return requested.upper()
    ports = list(list_ports.comports())
    preferred = [
        port.device
        for port in ports
        if port.vid == 0x10C4 and port.pid == 0xEA60
    ]
    if len(preferred) == 1:
        return preferred[0]
    devices = [port.device for port in ports]
    if len(devices) == 1:
        return devices[0]
    raise RuntimeError(f"cannot choose ESP32 port automatically: {devices}")


def crop_rgb(rgb888: bytes, width: int, height: int) -> bytes:
    if width < ROI_X_MAX or height < ROI_Y_MAX:
        raise ValueError(f"frame {width}x{height} is smaller than the ROI")
    cropped = bytearray(ROI_WIDTH * ROI_HEIGHT * 3)
    destination = 0
    for y in range(ROI_Y_MIN, ROI_Y_MAX):
        start = (y * width + ROI_X_MIN) * 3
        end = (y * width + ROI_X_MAX) * 3
        row = rgb888[start:end]
        cropped[destination : destination + len(row)] = row
        destination += len(row)
    return bytes(cropped)


def crop_mask(mask: bytes, width: int, height: int) -> bytes:
    if width < ROI_X_MAX or height < ROI_Y_MAX:
        raise ValueError(f"frame {width}x{height} is smaller than the ROI")
    pixels = bytearray(b"\xff" * (ROI_WIDTH * ROI_HEIGHT))
    destination = 0
    for y in range(ROI_Y_MIN, ROI_Y_MAX):
        for x in range(ROI_X_MIN, ROI_X_MAX):
            source = y * width + x
            if mask[source // 8] & (1 << (source % 8)):
                pixels[destination] = 0
            destination += 1
    return bytes(pixels)


def png_chunk(kind: bytes, data: bytes) -> bytes:
    return (
        struct.pack(">I", len(data))
        + kind
        + data
        + struct.pack(">I", zlib.crc32(kind + data) & 0xFFFFFFFF)
    )


def write_png(path: Path, width: int, height: int, pixels: bytes,
              channels: int) -> None:
    if channels not in (1, 3) or len(pixels) != width * height * channels:
        raise ValueError("invalid PNG pixel buffer")
    rows = bytearray()
    row_bytes = width * channels
    for y in range(height):
        rows.append(0)
        start = y * row_bytes
        rows.extend(pixels[start : start + row_bytes])
    color_type = 0 if channels == 1 else 2
    contents = (
        b"\x89PNG\r\n\x1a\n"
        + png_chunk(
            b"IHDR",
            struct.pack(">IIBBBBB", width, height, 8, color_type, 0, 0, 0),
        )
        + png_chunk(b"IDAT", zlib.compress(bytes(rows), level=6))
        + png_chunk(b"IEND", b"")
    )
    path.write_bytes(contents)


def local_x(value: int) -> int | None:
    return value - ROI_X_MIN if ROI_X_MIN <= value < ROI_X_MAX else None


def local_y(value: int) -> int | None:
    return value - ROI_Y_MIN if ROI_Y_MIN <= value < ROI_Y_MAX else None


def result_metadata(result: AlgorithmResult) -> dict[str, object]:
    points = [
        {"x": x - ROI_X_MIN, "y": y - ROI_Y_MIN}
        for x, y in result.path_points
        if ROI_X_MIN <= x < ROI_X_MAX and ROI_Y_MIN <= y < ROI_Y_MAX
    ]
    return {
        "found": result.found,
        "confidence": result.confidence,
        "near_x": local_x(result.near_x),
        "far_x": local_x(result.far_x),
        "lateral_error": result.lateral_error,
        "heading_error": result.heading_error,
        "steering_error": result.steering_error,
        "steering_band_valid": result.steering_band_valid,
        "steering_band_left_percent": result.steering_band_left_percent,
        "steering_band_right_percent": result.steering_band_right_percent,
        "steering_band_error": result.steering_band_error,
        "steering_band_pixel_count": result.steering_band_pixel_count,
        "big_turn": result.big_turn,
        "turn_direction": result.turn_direction,
        "turn_angle_deg": result.turn_angle_deg,
        "turn_confidence": result.turn_confidence,
        "corner_x": local_x(result.corner_x),
        "corner_y": local_y(result.corner_y),
        "vector_point_count": result.vector_point_count,
        "path_point_count": len(points),
        "path_points": points,
        "threshold": result.threshold,
        "contrast": result.contrast,
        "component_area": result.component_area,
    }


def create_session(output_root: Path) -> Path:
    session = output_root / datetime.now().strftime("session_%Y%m%d_%H%M%S")
    (session / "raw").mkdir(parents=True)
    (session / "mask").mkdir()
    manifest = {
        "format_version": 1,
        "width": ROI_WIDTH,
        "height": ROI_HEIGHT,
        "source_frame": {"width": 160, "height": 120},
        "roi": {
            "x_min": ROI_X_MIN,
            "x_max_exclusive": ROI_X_MAX,
            "y_min": ROI_Y_MIN,
            "y_max_exclusive": ROI_Y_MAX,
        },
        "coordinate_system": "ROI-local; origin at the ROI top-left",
        "turn_trigger_y": 42 - ROI_Y_MIN,
    }
    (session / "manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    return session


def collect(port_name: str, output_root: Path, count: int) -> Path:
    session = create_session(output_root)
    print(f"Session: {session}")
    print(f"Port: {port_name}; ROI-only output: {ROI_WIDTH}x{ROI_HEIGHT}")
    port = serial.Serial(
        port_name,
        NORMAL_BAUD,
        timeout=0.2,
        write_timeout=0.5,
        rtscts=False,
        xonxoff=False,
    )
    port.dtr = False
    port.rts = False
    port.reset_input_buffer()
    samples_path = session / "samples.jsonl"
    saved = 0
    try:
        port.write(b"xTUNER,1\n")
        port.flush()
        time.sleep(0.1)
        port.baudrate = TUNER_BAUD
        port.reset_input_buffer()
        stream = TunerStream(port)
        with samples_path.open("a", encoding="utf-8") as samples:
            while count <= 0 or saved < count:
                header, rgb565, mask, result = stream.read_sample()
                sample_id = f"{saved + 1:06d}"
                raw_relative = Path("raw") / f"{sample_id}.png"
                mask_relative = Path("mask") / f"{sample_id}.png"
                rgb_roi = crop_rgb(
                    rgb565_to_rgb888(rgb565), header.width, header.height
                )
                mask_roi = crop_mask(mask, header.width, header.height)
                write_png(
                    session / raw_relative,
                    ROI_WIDTH,
                    ROI_HEIGHT,
                    rgb_roi,
                    channels=3,
                )
                write_png(
                    session / mask_relative,
                    ROI_WIDTH,
                    ROI_HEIGHT,
                    mask_roi,
                    channels=1,
                )
                record = {
                    "sample_id": sample_id,
                    "sequence": header.sequence,
                    "received_at": datetime.now(timezone.utc).isoformat(),
                    "width": ROI_WIDTH,
                    "height": ROI_HEIGHT,
                    "raw_image": raw_relative.as_posix(),
                    "mask_image": mask_relative.as_posix(),
                    "rgb_thresholds": {
                        "red": header.red_threshold,
                        "green": header.green_threshold,
                        "blue": header.blue_threshold,
                    },
                    "algorithm": result_metadata(result),
                }
                samples.write(json.dumps(record, ensure_ascii=False) + "\n")
                samples.flush()
                saved += 1
                print(
                    f"[{saved}] seq={header.sequence} line={int(result.found)} "
                    f"conf={result.confidence} corner={int(result.big_turn)} "
                    f"path={result.path_point_count}"
                )
    except KeyboardInterrupt:
        print("Collection stopped by user.")
    finally:
        try:
            port.write(b"xTUNER,0\n")
            port.flush()
            time.sleep(0.1)
        except serial.SerialException:
            pass
        port.close()
    print(f"Saved {saved} samples to {session}")
    return session


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Save 106x108 ROI images, masks, path points and results."
    )
    parser.add_argument("--port", default="", help="ESP32 serial port")
    parser.add_argument(
        "--output", type=Path, default=Path("camera-datasets"),
        help="dataset root directory",
    )
    parser.add_argument(
        "--count", type=int, default=0,
        help="number of samples; 0 records until Ctrl+C",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.count < 0:
        raise SystemExit("--count must be zero or positive")
    port_name = choose_port(args.port)
    collect(port_name, args.output.resolve(), args.count)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
