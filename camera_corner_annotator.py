"""Annotate first-person path polylines and calibrated corner geometry."""

from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import dataclass
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import tempfile
import tkinter as tk
from tkinter import messagebox, ttk

import numpy as np
from PIL import Image, ImageTk


INITIAL_SCALE = 4
MIN_SCALE = 4
MAX_SCALE = 12
POINT_PICK_RADIUS = 4.0
ANGLE_WINDOW_POINTS = 3
SCENE_PATH = "path"
SCENE_FINISH_T = "finish_t"
SCENE_INVALID = "invalid"
FINISH_POINT_KEYS = ("junction", "crossbar_left", "crossbar_right")


def latest_session(dataset_root: Path) -> Path:
    sessions = sorted(
        (
            path
            for path in dataset_root.glob("session_*")
            if path.is_dir() and (path / "samples.jsonl").is_file()
        ),
        key=lambda path: path.stat().st_mtime,
        reverse=True,
    )
    if not sessions:
        raise FileNotFoundError(f"no dataset sessions found under {dataset_root}")
    return sessions[0]


def load_jsonl(path: Path) -> list[dict[str, object]]:
    records: list[dict[str, object]] = []
    with path.open("r", encoding="utf-8") as source:
        for line_number, line in enumerate(source, 1):
            if not line.strip():
                continue
            value = json.loads(line)
            if not isinstance(value, dict):
                raise ValueError(f"expected an object at {path}:{line_number}")
            records.append(value)
    return records


def load_path_labels(path: Path) -> dict[str, dict[str, object]]:
    if not path.exists():
        return {}
    labels: dict[str, dict[str, object]] = {}
    for record in load_jsonl(path):
        sample_id = str(record.get("sample_id", ""))
        if sample_id and "valid" in record:
            labels[sample_id] = record
    return labels


def write_path_labels(
    path: Path,
    sample_ids: list[str],
    labels: dict[str, dict[str, object]],
) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w", encoding="utf-8", newline="\n") as target:
        for sample_id in sample_ids:
            if sample_id in labels:
                target.write(json.dumps(labels[sample_id]) + "\n")
        target.flush()
    temporary.replace(path)


def label_scene_type(label: dict[str, object]) -> str:
    scene_type = label.get("scene_type")
    if scene_type in (SCENE_PATH, SCENE_FINISH_T, SCENE_INVALID):
        return str(scene_type)
    return SCENE_PATH if bool(label.get("valid")) else SCENE_INVALID


@dataclass(frozen=True)
class SessionGeometry:
    source_width: int
    source_height: int
    raw_roi_x_min: int
    raw_roi_x_max: int
    raw_roi_y_min: int
    raw_roi_y_max: int

    @property
    def width(self) -> int:
        return self.raw_roi_x_max - self.raw_roi_x_min

    @property
    def height(self) -> int:
        return self.raw_roi_y_max - self.raw_roi_y_min

    @property
    def first_roi_x_min(self) -> int:
        return self.source_width - self.raw_roi_x_max

    @property
    def first_roi_y_min(self) -> int:
        return self.source_height - self.raw_roi_y_max

    def raw_local_to_first_local(self, x: int, y: int) -> tuple[int, int]:
        return self.width - 1 - x, self.height - 1 - y

    def first_local_to_full(self, x: int, y: int) -> tuple[int, int]:
        return self.first_roi_x_min + x, self.first_roi_y_min + y


def load_session_geometry(session: Path) -> SessionGeometry:
    manifest = json.loads((session / "manifest.json").read_text(encoding="utf-8"))
    source = manifest["source_frame"]
    roi = manifest["roi"]
    return SessionGeometry(
        source_width=int(source["width"]),
        source_height=int(source["height"]),
        raw_roi_x_min=int(roi["x_min"]),
        raw_roi_x_max=int(roi["x_max_exclusive"]),
        raw_roi_y_min=int(roi["y_min"]),
        raw_roi_y_max=int(roi["y_max_exclusive"]),
    )


class GroundLookup:
    def __init__(self, path: Path, expected_size: tuple[int, int]) -> None:
        with np.load(path) as values:
            self.x_mm = values["x_mm"].copy()
            self.y_mm = values["y_mm"].copy()
            self.valid = values["valid"].copy().astype(bool)
        expected_width, expected_height = expected_size
        if self.x_mm.shape != (expected_height, expected_width):
            raise ValueError(
                f"calibration LUT shape {self.x_mm.shape} does not match "
                f"source {expected_width}x{expected_height}"
            )
        self.path = path

    def point(self, x: int, y: int) -> tuple[float, float] | None:
        if not (0 <= y < self.valid.shape[0] and 0 <= x < self.valid.shape[1]):
            return None
        if not self.valid[y, x]:
            return None
        x_mm = float(self.x_mm[y, x])
        y_mm = float(self.y_mm[y, x])
        if not math.isfinite(x_mm) or not math.isfinite(y_mm):
            return None
        return x_mm, y_mm


def algorithm_candidates(
    record: dict[str, object], geometry: SessionGeometry
) -> list[tuple[int, int]]:
    algorithm = record.get("algorithm", {})
    if not isinstance(algorithm, dict):
        return []
    result: list[tuple[int, int]] = []
    for point in algorithm.get("path_points", []):
        if isinstance(point, dict) and "x" in point and "y" in point:
            result.append(
                geometry.raw_local_to_first_local(int(point["x"]), int(point["y"]))
            )
    return result


def predicted_corner_indices(
    record: dict[str, object],
    geometry: SessionGeometry,
    points: list[tuple[int, int]],
) -> list[int]:
    algorithm = record.get("algorithm", {})
    if not isinstance(algorithm, dict):
        return []
    x = algorithm.get("corner_x")
    y = algorithm.get("corner_y")
    if x is None or y is None or not points:
        return []
    target = geometry.raw_local_to_first_local(int(x), int(y))
    return [
        min(
            range(len(points)),
            key=lambda index: math.dist(points[index], target),
        )
    ]


def calibrated_geometry(
    points: list[tuple[int, int]],
    corner_indices: list[int],
    session_geometry: SessionGeometry,
    lookup: GroundLookup,
) -> tuple[list[tuple[int, int]], list[tuple[float, float]], dict[str, object]]:
    full_points = [session_geometry.first_local_to_full(x, y) for x, y in points]
    ground_points: list[tuple[float, float]] = []
    for full_x, full_y in full_points:
        ground = lookup.point(full_x, full_y)
        if ground is None:
            return full_points, [], {
                "path_length_mm": None,
                "corners": [],
                "segments": [],
                "reason": "one or more points are outside the calibrated ground plane",
            }
        ground_points.append(ground)

    segment_lengths = [
        math.dist(ground_points[index - 1], ground_points[index])
        for index in range(1, len(ground_points))
    ]
    normalized_corners = sorted(set(corner_indices))
    measurements: dict[str, object] = {
        "path_length_mm": sum(segment_lengths),
        "corners": [],
        "segments": [],
        "angle_window_points": ANGLE_WINDOW_POINTS,
    }
    if any(
        corner_index <= 0 or corner_index >= len(ground_points) - 1
        for corner_index in normalized_corners
    ):
        measurements["reason"] = "corners must not be path endpoints"
        return full_points, ground_points, measurements

    boundaries = [0, *normalized_corners, len(ground_points) - 1]
    measurements["segments"] = [
        {
            "start_index": start,
            "end_index": end,
            "length_mm": sum(segment_lengths[start:end]),
        }
        for start, end in zip(boundaries, boundaries[1:])
        if end > start
    ]

    corner_measurements: list[dict[str, object]] = []
    for position, corner_index in enumerate(normalized_corners):
        previous_boundary = normalized_corners[position - 1] if position > 0 else 0
        next_boundary = (
            normalized_corners[position + 1]
            if position + 1 < len(normalized_corners)
            else len(ground_points) - 1
        )
        before_index = max(
            previous_boundary, corner_index - ANGLE_WINDOW_POINTS
        )
        after_index = min(next_boundary, corner_index + ANGLE_WINDOW_POINTS)
        before = ground_points[before_index]
        corner = ground_points[corner_index]
        after = ground_points[after_index]
        incoming = (corner[0] - before[0], corner[1] - before[1])
        outgoing = (after[0] - corner[0], after[1] - corner[1])
        incoming_norm = math.hypot(*incoming)
        outgoing_norm = math.hypot(*outgoing)
        angle: float | None = None
        direction: str | None = None
        if incoming_norm >= 1e-6 and outgoing_norm >= 1e-6:
            cross = incoming[0] * outgoing[1] - incoming[1] * outgoing[0]
            dot = incoming[0] * outgoing[0] + incoming[1] * outgoing[1]
            angle = math.degrees(math.atan2(cross, dot))
            direction = "left" if angle > 0.0 else "right"
        corner_measurements.append(
            {
                "corner_index": corner_index,
                "path_distance_from_near_point_mm": sum(
                    segment_lengths[:corner_index]
                ),
                "ground_x_mm": corner[0],
                "ground_y_mm": corner[1],
                "ground_distance_from_vehicle_mm": math.hypot(*corner),
                "turn_angle_deg": angle,
                "turn_direction": direction,
                "before_index": before_index,
                "after_index": after_index,
            }
        )
    measurements["corners"] = corner_measurements
    return full_points, ground_points, measurements


def calibrated_finish_geometry(
    finish_points: dict[str, tuple[int, int]],
    path_ground_points: list[tuple[float, float]],
    session_geometry: SessionGeometry,
    lookup: GroundLookup,
) -> dict[str, object]:
    missing = [key for key in FINISH_POINT_KEYS if key not in finish_points]
    if missing:
        raise ValueError(f"missing T finish points: {', '.join(missing)}")

    full_points: dict[str, tuple[int, int]] = {}
    ground_points: dict[str, tuple[float, float]] = {}
    for key in FINISH_POINT_KEYS:
        local = finish_points[key]
        full = session_geometry.first_local_to_full(*local)
        ground = lookup.point(*full)
        if ground is None:
            raise ValueError(f"T finish point {key} is outside calibration")
        full_points[key] = full
        ground_points[key] = ground

    junction = ground_points["junction"]
    left = ground_points["crossbar_left"]
    right = ground_points["crossbar_right"]
    crossbar = (right[0] - left[0], right[1] - left[1])
    crossbar_length = math.hypot(*crossbar)
    perpendicular_angle: float | None = None
    stem_reference_index: int | None = None
    if len(path_ground_points) >= 2 and crossbar_length > 1e-6:
        nearest_index = min(
            range(len(path_ground_points)),
            key=lambda index: math.dist(path_ground_points[index], junction),
        )
        stem_reference_index = max(0, nearest_index - ANGLE_WINDOW_POINTS)
        if stem_reference_index == nearest_index:
            stem_reference_index = max(0, nearest_index - 1)
        stem_reference = path_ground_points[stem_reference_index]
        stem = (
            junction[0] - stem_reference[0],
            junction[1] - stem_reference[1],
        )
        stem_length = math.hypot(*stem)
        if stem_length > 1e-6:
            cosine = max(
                -1.0,
                min(
                    1.0,
                    abs(stem[0] * crossbar[0] + stem[1] * crossbar[1])
                    / (stem_length * crossbar_length),
                ),
            )
            perpendicular_angle = math.degrees(math.acos(cosine))

    return {
        "full_frame": {key: list(full_points[key]) for key in FINISH_POINT_KEYS},
        "ground_mm": {
            key: [round(value[0], 3), round(value[1], 3)]
            for key, value in ground_points.items()
        },
        "measurements": {
            "junction_ground_distance_from_vehicle_mm": math.hypot(*junction),
            "junction_ground_x_mm": junction[0],
            "junction_ground_y_mm": junction[1],
            "crossbar_length_mm": crossbar_length,
            "stem_crossbar_angle_deg": perpendicular_angle,
            "stem_reference_index": stem_reference_index,
        },
    }


def nearest_point_index(
    points: list[tuple[int, int]], x: float, y: float
) -> int | None:
    if not points:
        return None
    index = min(range(len(points)), key=lambda i: math.dist(points[i], (x, y)))
    return index if math.dist(points[index], (x, y)) <= POINT_PICK_RADIUS else None


def point_segment_distance(
    point: tuple[float, float], start: tuple[int, int], end: tuple[int, int]
) -> float:
    px, py = point
    sx, sy = start
    ex, ey = end
    dx = ex - sx
    dy = ey - sy
    if dx == 0 and dy == 0:
        return math.dist(point, start)
    fraction = max(
        0.0,
        min(1.0, ((px - sx) * dx + (py - sy) * dy) / (dx * dx + dy * dy)),
    )
    return math.hypot(px - (sx + fraction * dx), py - (sy + fraction * dy))


class PathAnnotatorApp:
    def __init__(
        self, root: tk.Tk, session: Path, calibration_path: Path
    ) -> None:
        self.root = root
        self.session = session
        self.geometry = load_session_geometry(session)
        self.lookup = GroundLookup(
            calibration_path,
            (self.geometry.source_width, self.geometry.source_height),
        )
        self.records = load_jsonl(session / "samples.jsonl")
        if not self.records:
            raise ValueError(f"session has no samples: {session}")
        self.labels_path = session / "path_labels.jsonl"
        self.labels = load_path_labels(self.labels_path)
        self.sample_ids = [str(record["sample_id"]) for record in self.records]
        self.index = self._first_unlabeled_index()
        self.width = self.geometry.width
        self.height = self.geometry.height
        self.polyline: list[tuple[int, int]] = []
        self.corner_indices: set[int] = set()
        self.finish_points: dict[str, tuple[int, int]] = {}
        self.finish_point_mode: str | None = None
        self.selected_index: int | None = None
        self.scale = INITIAL_SCALE
        self.raw_image: Image.Image | None = None
        self.mask_image: Image.Image | None = None
        self.raw_photo: ImageTk.PhotoImage | None = None
        self.mask_photo: ImageTk.PhotoImage | None = None

        root.title("摄像头路径折线标注")
        root.resizable(False, False)
        root.protocol("WM_DELETE_WINDOW", root.destroy)
        self.progress_var = tk.StringVar()
        self.sample_var = tk.StringVar()
        self.algorithm_var = tk.StringVar()
        self.geometry_var = tk.StringVar()
        self.status_var = tk.StringVar(value="就绪")
        self._build_ui()
        self._bind_keys()
        self._load_current()

    def _first_unlabeled_index(self) -> int:
        for index, record in enumerate(self.records):
            if str(record["sample_id"]) not in self.labels:
                return index
        return 0

    @property
    def current(self) -> dict[str, object]:
        return self.records[self.index]

    def _build_ui(self) -> None:
        outer = ttk.Frame(self.root, padding=10)
        outer.grid(sticky="nsew")
        header = ttk.Frame(outer)
        header.grid(row=0, column=0, columnspan=2, sticky="ew")
        for row, variable in enumerate(
            (self.progress_var, self.sample_var, self.algorithm_var, self.geometry_var)
        ):
            ttk.Label(header, textvariable=variable).grid(
                row=row, column=0, sticky="w"
            )

        self.raw_canvas = self._image_panel(outer, "第一视角原图", 1, 0)
        self.mask_canvas = self._image_panel(outer, "第一视角二值图", 1, 1)
        for canvas in (self.raw_canvas, self.mask_canvas):
            canvas.bind("<ButtonPress-1>", self._press)
            canvas.bind("<B1-Motion>", self._drag)
            canvas.bind("<ButtonRelease-1>", self._release)
            canvas.bind("<Button-3>", self._delete_point)
            canvas.bind("<Shift-Button-1>", self._mark_clicked_corner)
            canvas.bind("<MouseWheel>", self._zoom)

        edit = ttk.Frame(outer)
        edit.grid(row=2, column=0, columnspan=2, pady=(10, 0))
        ttk.Button(edit, text="采用算法路径", command=self._use_algorithm).grid(
            row=0, column=0, padx=4
        )
        ttk.Button(edit, text="清除全部点", command=self._clear_all_points).grid(
            row=0, column=1, padx=4
        )
        ttk.Button(edit, text="切换角点", command=self._toggle_selected_corner).grid(
            row=0, column=2, padx=4
        )
        ttk.Button(edit, text="清除角点", command=self._clear_corner).grid(
            row=0, column=3, padx=4
        )
        ttk.Button(
            edit, text="路径并保存", command=lambda: self._save_scene(SCENE_PATH)
        ).grid(
            row=0, column=4, padx=4
        )
        ttk.Button(
            edit,
            text="无效并保存",
            command=lambda: self._save_scene(SCENE_INVALID),
        ).grid(
            row=0, column=5, padx=4
        )

        finish = ttk.Frame(outer)
        finish.grid(row=3, column=0, columnspan=2, pady=(8, 0))
        ttk.Button(
            finish,
            text="标T交汇点",
            command=lambda: self._set_finish_point_mode("junction"),
        ).grid(row=0, column=0, padx=4)
        ttk.Button(
            finish,
            text="标横线左端",
            command=lambda: self._set_finish_point_mode("crossbar_left"),
        ).grid(row=0, column=1, padx=4)
        ttk.Button(
            finish,
            text="标横线右端",
            command=lambda: self._set_finish_point_mode("crossbar_right"),
        ).grid(row=0, column=2, padx=4)
        ttk.Button(
            finish, text="清除T标记", command=self._clear_finish_points
        ).grid(row=0, column=3, padx=4)
        ttk.Button(
            finish,
            text="T终点并保存",
            command=lambda: self._save_scene(SCENE_FINISH_T),
        ).grid(row=0, column=4, padx=4)

        navigation = ttk.Frame(outer)
        navigation.grid(row=4, column=0, columnspan=2, pady=(10, 0))
        ttk.Button(navigation, text="上一帧", command=self._previous).grid(
            row=0, column=0, padx=4
        )
        ttk.Button(navigation, text="下一帧", command=self._next).grid(
            row=0, column=1, padx=4
        )
        ttk.Button(
            navigation, text="下一未标注", command=self._next_unlabeled
        ).grid(row=0, column=2, padx=4)
        ttk.Label(navigation, textvariable=self.status_var).grid(
            row=0, column=3, padx=(12, 0)
        )

    def _image_panel(
        self, parent: ttk.Frame, title: str, row: int, column: int
    ) -> tk.Canvas:
        panel = ttk.LabelFrame(parent, text=title, padding=5)
        panel.grid(
            row=row,
            column=column,
            padx=(0, 8) if column == 0 else 0,
            pady=(10, 0),
        )
        canvas = tk.Canvas(
            panel,
            width=self.width * INITIAL_SCALE,
            height=self.height * INITIAL_SCALE,
            highlightthickness=1,
            highlightbackground="#777777",
            cursor="crosshair",
        )
        horizontal = ttk.Scrollbar(panel, orient="horizontal", command=canvas.xview)
        vertical = ttk.Scrollbar(panel, orient="vertical", command=canvas.yview)
        canvas.configure(
            xscrollcommand=horizontal.set,
            yscrollcommand=vertical.set,
            scrollregion=(
                0,
                0,
                self.width * INITIAL_SCALE,
                self.height * INITIAL_SCALE,
            ),
        )
        canvas.grid(row=0, column=0)
        vertical.grid(row=0, column=1, sticky="ns")
        horizontal.grid(row=1, column=0, sticky="ew")
        return canvas

    def _bind_keys(self) -> None:
        self.root.bind("<Left>", lambda _event: self._previous())
        self.root.bind("<Right>", lambda _event: self._next())
        self.root.bind("<space>", lambda _event: self._next_unlabeled())
        self.root.bind("<KeyPress-c>", lambda _event: self._toggle_selected_corner())
        self.root.bind("<KeyPress-a>", lambda _event: self._use_algorithm())
        self.root.bind(
            "<KeyPress-v>", lambda _event: self._save_scene(SCENE_PATH)
        )
        self.root.bind(
            "<KeyPress-t>", lambda _event: self._save_scene(SCENE_FINISH_T)
        )
        self.root.bind(
            "<KeyPress-i>", lambda _event: self._save_scene(SCENE_INVALID)
        )

    def _load_current(self) -> None:
        record = self.current
        sample_id = str(record["sample_id"])
        try:
            raw = Image.open(self.session / str(record["raw_image"])).convert("RGB")
            mask = Image.open(self.session / str(record["mask_image"])).convert("RGB")
            self.raw_image = raw.transpose(Image.Transpose.ROTATE_180)
            self.mask_image = mask.transpose(Image.Transpose.ROTATE_180)
            self._refresh_photos()
        except OSError as exc:
            messagebox.showerror("图像读取失败", str(exc))
            return

        existing = self.labels.get(sample_id)
        if existing and bool(existing.get("valid")):
            self.polyline = [
                tuple(map(int, point)) for point in existing.get("polyline", [])
            ]
            saved_corners = existing.get("corner_indices")
            if isinstance(saved_corners, list):
                self.corner_indices = {int(index) for index in saved_corners}
            else:
                old_corner = existing.get("corner_index")
                self.corner_indices = (
                    {int(old_corner)} if old_corner is not None else set()
                )
            finish = existing.get("finish", {})
            self.finish_points = {}
            if isinstance(finish, dict):
                for key in FINISH_POINT_KEYS:
                    point = finish.get(key)
                    if isinstance(point, list) and len(point) == 2:
                        self.finish_points[key] = (int(point[0]), int(point[1]))
        elif existing:
            self.polyline = []
            self.corner_indices = set()
            self.finish_points = {}
        else:
            self.polyline = algorithm_candidates(record, self.geometry)
            self.corner_indices = set(
                predicted_corner_indices(record, self.geometry, self.polyline)
            )
            self.finish_points = {}
        self.finish_point_mode = None
        self.selected_index = None
        self._redraw()
        self._update_text()
        self.status_var.set("就绪")

    def _refresh_photos(self) -> None:
        if self.raw_image is None or self.mask_image is None:
            return
        display_size = (self.width * self.scale, self.height * self.scale)
        self.raw_photo = ImageTk.PhotoImage(
            self.raw_image.resize(display_size, Image.Resampling.NEAREST)
        )
        self.mask_photo = ImageTk.PhotoImage(
            self.mask_image.resize(display_size, Image.Resampling.NEAREST)
        )

    def _redraw(self) -> None:
        if self.raw_photo is not None:
            self._draw_canvas(self.raw_canvas, self.raw_photo)
        if self.mask_photo is not None:
            self._draw_canvas(self.mask_canvas, self.mask_photo)

    def _draw_canvas(self, canvas: tk.Canvas, photo: ImageTk.PhotoImage) -> None:
        canvas.delete("all")
        canvas.configure(
            scrollregion=(0, 0, self.width * self.scale, self.height * self.scale)
        )
        canvas.create_image(0, 0, image=photo, anchor="nw")
        coordinates = [
            value
            for point in self.polyline
            for value in (
                (point[0] + 0.5) * self.scale,
                (point[1] + 0.5) * self.scale,
            )
        ]
        if len(coordinates) >= 4:
            canvas.create_line(*coordinates, fill="#00a6ff", width=2)
        for index, (x, y) in enumerate(self.polyline):
            cx = (x + 0.5) * self.scale
            cy = (y + 0.5) * self.scale
            color = "#ff3030" if index in self.corner_indices else "#00a6ff"
            radius = 5 if index == self.selected_index else 3
            if index == self.selected_index:
                color = "#ffd23f"
            canvas.create_oval(
                cx - radius,
                cy - radius,
                cx + radius,
                cy + radius,
                fill=color,
                outline="#101010",
            )
        left = self.finish_points.get("crossbar_left")
        right = self.finish_points.get("crossbar_right")
        if left is not None and right is not None:
            canvas.create_line(
                (left[0] + 0.5) * self.scale,
                (left[1] + 0.5) * self.scale,
                (right[0] + 0.5) * self.scale,
                (right[1] + 0.5) * self.scale,
                fill="#45e06f",
                width=3,
            )
        marker_style = {
            "junction": ("J", "#ff35d3"),
            "crossbar_left": ("L", "#45e06f"),
            "crossbar_right": ("R", "#ff9f43"),
        }
        for key, (x, y) in self.finish_points.items():
            label, color = marker_style[key]
            cx = (x + 0.5) * self.scale
            cy = (y + 0.5) * self.scale
            radius = max(6, self.scale)
            canvas.create_rectangle(
                cx - radius,
                cy - radius,
                cx + radius,
                cy + radius,
                fill=color,
                outline="#101010",
                width=2,
            )
            canvas.create_text(cx, cy, text=label, fill="#101010")

    def _event_point(self, event: tk.Event) -> tuple[int, int]:
        canvas = event.widget
        canvas_x = canvas.canvasx(event.x)
        canvas_y = canvas.canvasy(event.y)
        return (
            max(0, min(self.width - 1, int(canvas_x) // self.scale)),
            max(0, min(self.height - 1, int(canvas_y) // self.scale)),
        )

    def _zoom(self, event: tk.Event) -> str:
        direction = 1 if event.delta > 0 else -1
        new_scale = max(MIN_SCALE, min(MAX_SCALE, self.scale + direction))
        if new_scale == self.scale:
            return "break"
        active_canvas = event.widget
        image_x = active_canvas.canvasx(event.x) / self.scale
        image_y = active_canvas.canvasy(event.y) / self.scale
        self.scale = new_scale
        self._refresh_photos()
        self._redraw()
        total_width = self.width * self.scale
        total_height = self.height * self.scale
        for canvas in (self.raw_canvas, self.mask_canvas):
            left = image_x * self.scale - event.x
            top = image_y * self.scale - event.y
            canvas.xview_moveto(max(0.0, min(1.0, left / total_width)))
            canvas.yview_moveto(max(0.0, min(1.0, top / total_height)))
        self.status_var.set(f"缩放 {self.scale}×")
        return "break"

    def _press(self, event: tk.Event) -> None:
        x, y = self._event_point(event)
        if self.finish_point_mode is not None:
            key = self.finish_point_mode
            self.finish_points[key] = (x, y)
            self.finish_point_mode = None
            self.selected_index = None
            self._redraw()
            self._update_geometry_text()
            self.status_var.set(f"已标记 {key}=({x},{y})")
            return
        selected = nearest_point_index(self.polyline, x, y)
        if selected is None:
            if len(self.polyline) < 2:
                self.polyline.append((x, y))
                selected = len(self.polyline) - 1
            else:
                segment = min(
                    range(len(self.polyline) - 1),
                    key=lambda i: point_segment_distance(
                        (x, y), self.polyline[i], self.polyline[i + 1]
                    ),
                )
                segment_distance = point_segment_distance(
                    (x, y), self.polyline[segment], self.polyline[segment + 1]
                )
                start_distance = math.dist((x, y), self.polyline[0])
                end_distance = math.dist((x, y), self.polyline[-1])
                if end_distance <= segment_distance + 0.5 and (
                    end_distance < start_distance
                ):
                    self.polyline.append((x, y))
                    selected = len(self.polyline) - 1
                elif start_distance <= segment_distance + 0.5:
                    self.polyline.insert(0, (x, y))
                    self.corner_indices = {
                        index + 1 for index in self.corner_indices
                    }
                    selected = 0
                else:
                    self.polyline.insert(segment + 1, (x, y))
                    self.corner_indices = {
                        index + 1 if index > segment else index
                        for index in self.corner_indices
                    }
                    selected = segment + 1
        self.selected_index = selected
        self._redraw()
        self._update_geometry_text()

    def _drag(self, event: tk.Event) -> None:
        if self.selected_index is None:
            return
        self.polyline[self.selected_index] = self._event_point(event)
        self._redraw()
        self._update_geometry_text()

    def _release(self, _event: tk.Event) -> None:
        self._update_geometry_text()

    def _delete_point(self, event: tk.Event) -> None:
        x, y = self._event_point(event)
        finish_key = next(
            (
                key
                for key, point in self.finish_points.items()
                if math.dist(point, (x, y)) <= POINT_PICK_RADIUS
            ),
            None,
        )
        if finish_key is not None:
            del self.finish_points[finish_key]
            self._redraw()
            self._update_geometry_text()
            self.status_var.set(f"已删除 T 标记 {finish_key}")
            return
        index = nearest_point_index(self.polyline, x, y)
        if index is None:
            return
        self.polyline.pop(index)
        self.corner_indices = {
            corner - 1 if corner > index else corner
            for corner in self.corner_indices
            if corner != index
        }
        self.selected_index = None
        self._redraw()
        self._update_geometry_text()

    def _mark_clicked_corner(self, event: tk.Event) -> str:
        x, y = self._event_point(event)
        index = nearest_point_index(self.polyline, x, y)
        if index is not None:
            self.selected_index = index
            self._toggle_corner(index)
            self._redraw()
            self._update_geometry_text()
        return "break"

    def _toggle_corner(self, index: int) -> None:
        if index in self.corner_indices:
            self.corner_indices.remove(index)
        else:
            self.corner_indices.add(index)

    def _toggle_selected_corner(self) -> None:
        if self.selected_index is None:
            self.status_var.set("请先选择一个路径点")
            return
        if self.selected_index in (0, len(self.polyline) - 1):
            self.status_var.set("角点不能是路径端点")
            return
        self._toggle_corner(self.selected_index)
        self._redraw()
        self._update_geometry_text()

    def _clear_corner(self) -> None:
        self.corner_indices.clear()
        self._redraw()
        self._update_geometry_text()

    def _clear_all_points(self) -> None:
        self.polyline = []
        self.corner_indices.clear()
        self.selected_index = None
        self._redraw()
        self._update_geometry_text()
        self.status_var.set("已清除当前帧全部路径点")

    def _set_finish_point_mode(self, key: str) -> None:
        if key not in FINISH_POINT_KEYS:
            return
        self.finish_point_mode = key
        self.selected_index = None
        names = {
            "junction": "T交汇点",
            "crossbar_left": "横线左端",
            "crossbar_right": "横线右端",
        }
        self.status_var.set(f"请在图中点击{names[key]}")

    def _clear_finish_points(self) -> None:
        self.finish_points.clear()
        self.finish_point_mode = None
        self._redraw()
        self._update_geometry_text()
        self.status_var.set("已清除当前帧全部 T 标记")

    def _use_algorithm(self) -> None:
        self.polyline = algorithm_candidates(self.current, self.geometry)
        self.corner_indices = set(
            predicted_corner_indices(self.current, self.geometry, self.polyline)
        )
        self.selected_index = None
        self._redraw()
        self._update_geometry_text()
        self.status_var.set("已恢复算法候选路径")

    def _current_geometry(
        self,
    ) -> tuple[
        list[tuple[int, int]],
        list[tuple[float, float]],
        dict[str, object],
    ]:
        return calibrated_geometry(
            self.polyline, sorted(self.corner_indices), self.geometry, self.lookup
        )

    def _current_finish_geometry(
        self, path_ground_points: list[tuple[float, float]] | None = None
    ) -> dict[str, object]:
        if path_ground_points is None:
            _, path_ground_points, _ = self._current_geometry()
        return calibrated_finish_geometry(
            self.finish_points,
            path_ground_points,
            self.geometry,
            self.lookup,
        )

    def _update_geometry_text(self) -> None:
        _, path_ground_points, measurements = self._current_geometry()
        path = measurements.get("path_length_mm")
        if isinstance(path, float):
            text = (
                f"人工路径：{len(self.polyline)}点  "
                f"角点={len(self.corner_indices)}  总长={path:.1f}mm"
            )
        else:
            text = f"人工路径：{len(self.polyline)}点  几何无效"
        corners = measurements.get("corners", [])
        if isinstance(corners, list) and corners:
            summaries = []
            for corner in corners:
                if not isinstance(corner, dict):
                    continue
                distance = corner.get("ground_distance_from_vehicle_mm")
                angle = corner.get("turn_angle_deg")
                direction = corner.get("turn_direction")
                if isinstance(distance, float) and isinstance(angle, float):
                    summaries.append(
                        f"{distance:.0f}mm/{angle:.1f}°/{direction}"
                    )
            if summaries:
                text += "  " + " | ".join(summaries)
        if self.finish_points:
            text += f"  T标记={len(self.finish_points)}/3"
        if len(self.finish_points) == len(FINISH_POINT_KEYS):
            try:
                finish = self._current_finish_geometry(path_ground_points)
                finish_measurements = finish["measurements"]
                distance = finish_measurements[
                    "junction_ground_distance_from_vehicle_mm"
                ]
                crossbar_length = finish_measurements["crossbar_length_mm"]
                angle = finish_measurements["stem_crossbar_angle_deg"]
                text += f"  交汇={distance:.0f}mm 横杆={crossbar_length:.0f}mm"
                if isinstance(angle, float):
                    text += f" 夹角={angle:.1f}°"
            except ValueError:
                text += "  T几何无效"
        self.geometry_var.set(text)

    def _update_text(self) -> None:
        counts = Counter(label_scene_type(value) for value in self.labels.values())
        self.progress_var.set(
            f"已标注 {len(self.labels)}/{len(self.records)}  "
            f"路径={counts[SCENE_PATH]} T终点={counts[SCENE_FINISH_T]} "
            f"无效={counts[SCENE_INVALID]}"
        )
        sample_id = str(self.current["sample_id"])
        self.sample_var.set(
            f"帧 {self.index + 1}/{len(self.records)}  sample={sample_id}"
        )
        algorithm = self.current.get("algorithm", {})
        if isinstance(algorithm, dict):
            self.algorithm_var.set(
                f"算法：line={int(bool(algorithm.get('found')))} "
                f"path={algorithm.get('path_point_count')} "
                f"corner={int(bool(algorithm.get('big_turn')))} "
                f"angle={algorithm.get('turn_angle_deg')} "
                f"confidence={algorithm.get('confidence')}"
            )
        self._update_geometry_text()

    def _save_scene(self, scene_type: str) -> None:
        if scene_type not in (SCENE_PATH, SCENE_FINISH_T, SCENE_INVALID):
            return
        valid = scene_type != SCENE_INVALID
        if valid and len(self.polyline) < 2:
            self.status_var.set("路径或 T 形主干至少需要两个点")
            return
        if (
            scene_type == SCENE_PATH
            and any(
                index in (0, len(self.polyline) - 1)
                for index in self.corner_indices
            )
        ):
            self.status_var.set("角点不能是路径端点")
            return
        sample_id = str(self.current["sample_id"])
        geometry_corners = (
            sorted(self.corner_indices) if scene_type == SCENE_PATH else []
        )
        full_points, ground_points, measurements = calibrated_geometry(
            self.polyline, geometry_corners, self.geometry, self.lookup
        )
        if valid and not ground_points:
            self.status_var.set(str(measurements.get("reason", "标定坐标无效")))
            return
        finish: dict[str, object] = {}
        if scene_type == SCENE_FINISH_T:
            if len(self.finish_points) != len(FINISH_POINT_KEYS):
                self.status_var.set("请标记 T 交汇点、横线左端和横线右端")
                return
            try:
                finish_geometry = self._current_finish_geometry(ground_points)
            except ValueError as exc:
                self.status_var.set(str(exc))
                return
            finish = {
                key: list(self.finish_points[key]) for key in FINISH_POINT_KEYS
            }
            finish.update(finish_geometry)
        self.labels[sample_id] = {
            "format_version": 4,
            "sample_id": sample_id,
            "sequence": self.current.get("sequence"),
            "valid": valid,
            "scene_type": scene_type,
            "orientation": "vehicle_first_person",
            "polyline": [list(point) for point in self.polyline] if valid else [],
            "corner_indices": geometry_corners if valid else [],
            "finish": finish,
            "first_person_roi": {
                "x_min": self.geometry.first_roi_x_min,
                "y_min": self.geometry.first_roi_y_min,
                "width": self.width,
                "height": self.height,
            },
            "full_frame_polyline": (
                [list(point) for point in full_points] if valid else []
            ),
            "ground_polyline_mm": (
                [[round(x, 3), round(y, 3)] for x, y in ground_points]
                if valid
                else []
            ),
            "measurements": measurements if valid else {},
            "calibration_file": self.lookup.path.name,
            "annotated_at": datetime.now(timezone.utc).isoformat(),
        }
        write_path_labels(self.labels_path, self.sample_ids, self.labels)
        self.status_var.set(f"已保存 {sample_id}: {scene_type}")
        self._advance_after_save()

    def _advance_after_save(self) -> None:
        for offset in range(1, len(self.records) + 1):
            candidate = (self.index + offset) % len(self.records)
            if str(self.records[candidate]["sample_id"]) not in self.labels:
                self.index = candidate
                self._load_current()
                return
        self._update_text()
        self.status_var.set("全部样本已经标注")

    def _previous(self) -> None:
        self.index = (self.index - 1) % len(self.records)
        self._load_current()

    def _next(self) -> None:
        self.index = (self.index + 1) % len(self.records)
        self._load_current()

    def _next_unlabeled(self) -> None:
        for offset in range(1, len(self.records) + 1):
            candidate = (self.index + offset) % len(self.records)
            if str(self.records[candidate]["sample_id"]) not in self.labels:
                self.index = candidate
                self._load_current()
                return
        self.status_var.set("没有未标注样本")


def run_self_test() -> None:
    geometry = SessionGeometry(160, 120, 27, 133, 8, 116)
    if geometry.raw_local_to_first_local(0, 0) != (105, 107):
        raise AssertionError("ROI rotation mismatch")
    if geometry.first_local_to_full(0, 0) != (27, 4):
        raise AssertionError("first-person ROI origin mismatch")
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory) / "lut.npz"
        yy, xx = np.mgrid[:120, :160]
        np.savez(
            path,
            x_mm=xx.astype(np.float32),
            y_mm=(120 - yy).astype(np.float32),
            valid=np.ones((120, 160), dtype=bool),
        )
        lookup = GroundLookup(path, (160, 120))
        points = [
            (53, 100), (53, 80), (53, 60), (33, 60),
            (13, 60), (13, 40), (13, 20),
        ]
        _, ground, measurements = calibrated_geometry(
            points, [2, 4], geometry, lookup
        )
        corners = measurements["corners"]
        if (
            len(ground) != 7
            or len(corners) != 2
            or len(measurements["segments"]) != 3
            or not math.isclose(
                float(corners[0]["turn_angle_deg"]), 90.0, abs_tol=1e-6
            )
            or not math.isclose(
                float(corners[1]["turn_angle_deg"]), -90.0, abs_tol=1e-6
            )
        ):
            raise AssertionError(
                f"multi-corner geometry mismatch: {measurements}"
            )
        stem_points = [(53, 100), (53, 80), (53, 60)]
        _, stem_ground, _ = calibrated_geometry(
            stem_points, [], geometry, lookup
        )
        finish = calibrated_finish_geometry(
            {
                "junction": (53, 40),
                "crossbar_left": (33, 40),
                "crossbar_right": (73, 40),
            },
            stem_ground,
            geometry,
            lookup,
        )
        finish_measurements = finish["measurements"]
        if (
            not math.isclose(
                float(finish_measurements["crossbar_length_mm"]),
                40.0,
                abs_tol=1e-6,
            )
            or not math.isclose(
                float(finish_measurements["stem_crossbar_angle_deg"]),
                90.0,
                abs_tol=1e-6,
            )
        ):
            raise AssertionError(f"T finish geometry mismatch: {finish}")
        if label_scene_type({"valid": True}) != SCENE_PATH:
            raise AssertionError("legacy valid scene migration failed")
    print("camera_corner_annotator self-test: PASS")


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description="Annotate calibrated camera paths.")
    parser.add_argument("--session", type=Path, help="dataset session directory")
    parser.add_argument(
        "--dataset-root", type=Path, default=root / "camera-datasets"
    )
    parser.add_argument(
        "--calibration",
        type=Path,
        default=root / "camera_ground_calibration_pixel_lut.npz",
    )
    parser.add_argument("--self-test", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.self_test:
        run_self_test()
        return 0
    session = (
        args.session.resolve()
        if args.session
        else latest_session(args.dataset_root.resolve())
    )
    root = tk.Tk()
    try:
        PathAnnotatorApp(root, session, args.calibration.resolve())
    except (
        FileNotFoundError,
        KeyError,
        OSError,
        TypeError,
        ValueError,
        json.JSONDecodeError,
    ) as exc:
        root.withdraw()
        messagebox.showerror("无法打开标注会话", str(exc))
        root.destroy()
        return 1
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
