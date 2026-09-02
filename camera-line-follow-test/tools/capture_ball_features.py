"""Capture one camera tuner frame and report a purple-ball candidate.

The firmware changes UART0 from 115200 to 921600 baud after receiving
``TUNER,1``.  This tool captures one RGB565 frame, then restores normal UART
mode with ``TUNER,0``.
"""

from __future__ import annotations

import argparse
import colorsys
import json
import math
from pathlib import Path
import statistics
import struct
import time
from collections import deque
import zlib

import serial


NORMAL_BAUD = 115200
TUNER_BAUD = 921600


def rgb565_to_rgb(frame: bytes, index: int) -> tuple[int, int, int]:
    pixel = (frame[index * 2] << 8) | frame[index * 2 + 1]
    red = ((pixel >> 11) & 0x1F) * 255 // 31
    green = ((pixel >> 5) & 0x3F) * 255 // 63
    blue = (pixel & 0x1F) * 255 // 31
    return red, green, blue


def is_purple(red: int, green: int, blue: int) -> bool:
    hue, saturation, value = colorsys.rgb_to_hsv(red / 255, green / 255,
                                                  blue / 255)
    hue_degrees = hue * 360
    return 245 <= hue_degrees <= 335 and saturation >= 0.28 and value >= 0.16


def analyze_purple(frame: bytes, width: int, height: int) -> dict[str, object]:
    pixels = [rgb565_to_rgb(frame, index) for index in range(width * height)]
    purple = [is_purple(*pixel) for pixel in pixels]
    visited = [False] * len(purple)
    components: list[list[int]] = []

    for start, selected in enumerate(purple):
        if not selected or visited[start]:
            continue
        visited[start] = True
        component: list[int] = []
        queue: deque[int] = deque([start])
        while queue:
            index = queue.popleft()
            component.append(index)
            x, y = index % width, index // width
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    if dx == 0 and dy == 0:
                        continue
                    nx, ny = x + dx, y + dy
                    if not (0 <= nx < width and 0 <= ny < height):
                        continue
                    neighbour = ny * width + nx
                    if purple[neighbour] and not visited[neighbour]:
                        visited[neighbour] = True
                        queue.append(neighbour)
        components.append(component)

    if not components:
        return {
            "detected": False,
            "purple_pixels": 0,
            "candidate_count": 0,
            "reason": "No pixels passed the initial purple HSV threshold.",
        }

    component = max(components, key=len)
    xs = [index % width for index in component]
    ys = [index // width for index in component]
    area = len(component)
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    box_width, box_height = max_x - min_x + 1, max_y - min_y + 1

    component_set = set(component)
    perimeter = 0
    for index in component:
        x, y = index % width, index // width
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nx, ny = x + dx, y + dy
            if not (0 <= nx < width and 0 <= ny < height) or (
                ny * width + nx not in component_set
            ):
                perimeter += 1

    selected_rgb = [pixels[index] for index in component]
    mean_red = round(sum(pixel[0] for pixel in selected_rgb) / area)
    mean_green = round(sum(pixel[1] for pixel in selected_rgb) / area)
    mean_blue = round(sum(pixel[2] for pixel in selected_rgb) / area)
    hue, saturation, value = colorsys.rgb_to_hsv(mean_red / 255,
                                                  mean_green / 255,
                                                  mean_blue / 255)
    fill_ratio = area / (box_width * box_height)
    circularity = 4 * math.pi * area / (perimeter * perimeter) if perimeter else 0

    return {
        "detected": area >= 12,
        "purple_pixels": sum(purple),
        "candidate_count": len(components),
        "area_pixels": area,
        "center_xy": [round(sum(xs) / area, 1), round(sum(ys) / area, 1)],
        "bounding_box_xywh": [min_x, min_y, box_width, box_height],
        "aspect_ratio": round(box_width / box_height, 2),
        "fill_ratio": round(fill_ratio, 2),
        "circularity": round(circularity, 2),
        "mean_rgb": [mean_red, mean_green, mean_blue],
        "mean_hsv": [round(hue * 360, 1), round(saturation, 2), round(value, 2)],
    }


def summarize_center(frame: bytes, width: int, height: int) -> dict[str, object]:
    """Summarize the central half of the frame before a color model exists."""
    pixels = [rgb565_to_rgb(frame, index) for index in range(width * height)]
    x0, x1 = width // 4, width - width // 4
    y0, y1 = height // 4, height - height // 4
    center = [pixels[y * width + x] for y in range(y0, y1) for x in range(x0, x1)]
    mean = tuple(round(sum(pixel[channel] for pixel in center) / len(center))
                 for channel in range(3))
    hue_bins: dict[int, int] = {}
    for red, green, blue in center:
        hue, saturation, value = colorsys.rgb_to_hsv(red / 255, green / 255,
                                                      blue / 255)
        if saturation >= 0.2 and value >= 0.15:
            bucket = int((hue * 360) // 15) * 15
            hue_bins[bucket] = hue_bins.get(bucket, 0) + 1
    return {
        "roi_xywh": [x0, y0, x1 - x0, y1 - y0],
        "mean_rgb": list(mean),
        "dominant_colored_hue_bins_degrees": sorted(
            hue_bins.items(), key=lambda item: item[1], reverse=True
        )[:5],
    }


def analyze_dark_round_candidate(frame: bytes, width: int,
                                 height: int) -> dict[str, object]:
    """Find a small dark, approximately round candidate near image center.

    This is a useful fallback for white balls: the ball itself is nearly the
    same color as the floor, while its rim/shadow supplies the strongest cue.
    """
    pixels = [rgb565_to_rgb(frame, index) for index in range(width * height)]
    luminance = [round(0.299 * red + 0.587 * green + 0.114 * blue)
                 for red, green, blue in pixels]
    roi_x0, roi_x1 = width // 8, width - width // 8
    roi_y0, roi_y1 = height // 8, height - height // 8
    roi_values = [luminance[y * width + x] for y in range(roi_y0, roi_y1)
                  for x in range(roi_x0, roi_x1)]
    background_luma = statistics.median(roi_values)
    threshold = max(20, background_luma - 45)
    selected = [value <= threshold for value in luminance]
    visited = [False] * len(selected)
    candidates: list[list[int]] = []

    for start, active in enumerate(selected):
        if not active or visited[start]:
            continue
        visited[start] = True
        queue: deque[int] = deque([start])
        component: list[int] = []
        while queue:
            index = queue.popleft()
            component.append(index)
            x, y = index % width, index // width
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    if dx == 0 and dy == 0:
                        continue
                    nx, ny = x + dx, y + dy
                    if 0 <= nx < width and 0 <= ny < height:
                        neighbour = ny * width + nx
                        if selected[neighbour] and not visited[neighbour]:
                            visited[neighbour] = True
                            queue.append(neighbour)
        if 4 <= len(component) <= 300:
            candidates.append(component)

    if not candidates:
        return {
            "detected": False,
            "background_luminance": background_luma,
            "dark_threshold": threshold,
            "reason": "No compact dark regions were found.",
        }

    image_cx, image_cy = (width - 1) / 2, (height - 1) / 2

    def score(component: list[int]) -> float:
        cx = sum(index % width for index in component) / len(component)
        cy = sum(index // width for index in component) / len(component)
        distance = math.hypot(cx - image_cx, cy - image_cy)
        return len(component) / (1 + distance)

    component = max(candidates, key=score)
    xs = [index % width for index in component]
    ys = [index // width for index in component]
    area = len(component)
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    box_width, box_height = max_x - min_x + 1, max_y - min_y + 1
    component_set = set(component)
    perimeter = 0
    for index in component:
        x, y = index % width, index // width
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nx, ny = x + dx, y + dy
            if not (0 <= nx < width and 0 <= ny < height) or (
                ny * width + nx not in component_set
            ):
                perimeter += 1
    circularity = 4 * math.pi * area / (perimeter * perimeter) if perimeter else 0
    mean_luma = round(sum(luminance[index] for index in component) / area)
    return {
        "detected": True,
        "background_luminance": background_luma,
        "dark_threshold": threshold,
        "area_pixels": area,
        "center_xy": [round(sum(xs) / area, 1), round(sum(ys) / area, 1)],
        "bounding_box_xywh": [min_x, min_y, box_width, box_height],
        "aspect_ratio": round(box_width / box_height, 2),
        "circularity": round(circularity, 2),
        "mean_luminance": mean_luma,
    }


def write_png(path: Path, frame: bytes, width: int, height: int) -> None:
    """Write the captured RGB565 data as a dependency-free RGB PNG preview."""
    scanlines = bytearray()
    for y in range(height):
        scanlines.append(0)
        for x in range(width):
            scanlines.extend(rgb565_to_rgb(frame, y * width + x))

    def chunk(kind: bytes, data: bytes) -> bytes:
        return (struct.pack(">I", len(data)) + kind + data +
                struct.pack(">I", zlib.crc32(kind + data) & 0xFFFFFFFF))

    png = (b"\x89PNG\r\n\x1a\n" +
           chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)) +
           chunk(b"IDAT", zlib.compress(bytes(scanlines), 9)) +
           chunk(b"IEND", b""))
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(png)


def capture_frame(port_name: str, timeout: float) -> tuple[list[str], bytes]:
    port = serial.Serial(port_name, NORMAL_BAUD, timeout=0.1)
    try:
        # Opening the CP210x console resets the ESP32-S3 on this board.
        time.sleep(3.0)
        port.reset_input_buffer()
        port.write(b"TUNER,1\n")
        port.flush()
        time.sleep(0.2)
        port.baudrate = TUNER_BAUD

        buffer = bytearray()
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            buffer.extend(port.read(4096))
            start = buffer.find(b"@RGB565,")
            if start < 0:
                if len(buffer) > 4096:
                    del buffer[:-256]
                continue
            newline = buffer.find(b"\n", start)
            if newline < 0:
                continue
            fields = buffer[start:newline].decode("ascii").split(",")
            if len(fields) != 19 or fields[0] != "@RGB565":
                del buffer[:newline + 1]
                continue
            raw_bytes, mask_bytes = int(fields[-2]), int(fields[-1])
            expected = raw_bytes + mask_bytes
            payload = bytearray(buffer[newline + 1:])
            while len(payload) < expected and time.monotonic() < deadline:
                payload.extend(port.read(expected - len(payload)))
            if len(payload) >= expected:
                return fields, bytes(payload[:raw_bytes])
        raise TimeoutError("No complete RGB565 frame arrived from the camera.")
    finally:
        try:
            port.write(b"TUNER,0\n")
            port.flush()
            time.sleep(0.15)
        finally:
            port.close()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="COM6")
    parser.add_argument("--timeout", type=float, default=8.0)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    header, frame = capture_frame(args.port, args.timeout)
    width, height = int(header[2]), int(header[3])
    if args.output:
        write_png(args.output, frame, width, height)
    result = {
        "frame_sequence": int(header[1]),
        "frame_size": [width, height],
        "center_color_summary": summarize_center(frame, width, height),
        "purple_candidate": analyze_purple(frame, width, height),
        "dark_round_candidate": analyze_dark_round_candidate(frame, width, height),
    }
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
