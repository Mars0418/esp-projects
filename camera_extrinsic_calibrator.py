"""Interactive camera intrinsic and ground-plane extrinsic calibration."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from datetime import datetime, timezone
import json
from pathlib import Path
import queue
import tempfile
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

try:
    import cv2
    import numpy as np
    from PIL import Image, ImageTk
except (ImportError, ModuleNotFoundError) as exc:
    raise SystemExit(
        "OpenCV, NumPy and Pillow are required. Run "
        "start_camera_extrinsic_calibrator.ps1."
    ) from exc

import serial
from serial.tools import list_ports


CALIBRATION_MAGIC = b"@CALJPEG,"
CALIBRATION_BAUD = 921600
MAX_JPEG_BYTES = 256 * 1024
CALIBRATION_IMAGE_SIZE = (640, 480)
RUNTIME_IMAGE_SIZE = (160, 120)
IMAGE_ROTATION_DEGREES = 180


@dataclass(frozen=True)
class CalibrationJpegFrame:
    sequence: int
    width: int
    height: int
    jpeg: bytes


def parse_calibration_header(line: bytes) -> tuple[int, int, int, int]:
    fields = line.decode("ascii").strip().split(",")
    if len(fields) != 5 or fields[0] != "@CALJPEG":
        raise ValueError("invalid calibration JPEG header")
    sequence, width, height, payload_bytes = (
        int(value) for value in fields[1:]
    )
    if width <= 0 or height <= 0 or width * height > 4_194_304:
        raise ValueError("invalid calibration image size")
    if payload_bytes < 4 or payload_bytes > MAX_JPEG_BYTES:
        raise ValueError("invalid calibration JPEG length")
    return sequence, width, height, payload_bytes


class CalibrationSerialWorker:
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
            target=self._read_loop,
            name="camera-calibration-serial",
            daemon=True,
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
                self.write(b"xCALIB,0\n")
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
        pending_header: tuple[int, int, int, int] | None = None
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
                    magic_index = receive_buffer.find(CALIBRATION_MAGIC)
                    if magic_index < 0:
                        keep = min(
                            len(receive_buffer), len(CALIBRATION_MAGIC) - 1
                        )
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
                        pending_header = parse_calibration_header(header_line)
                    except (UnicodeDecodeError, ValueError) as exc:
                        self._event_queue.put(("warning", str(exc)))
                        pending_header = None
                        continue

                sequence, width, height, payload_bytes = pending_header
                if len(receive_buffer) < payload_bytes:
                    break
                jpeg = bytes(receive_buffer[:payload_bytes])
                del receive_buffer[:payload_bytes]
                pending_header = None
                if not (
                    jpeg.startswith(b"\xff\xd8") and jpeg.endswith(b"\xff\xd9")
                ):
                    self._event_queue.put(("warning", "invalid JPEG markers"))
                    continue
                self._event_queue.put(
                    (
                        "frame",
                        CalibrationJpegFrame(sequence, width, height, jpeg),
                    )
                )


@dataclass
class IntrinsicCalibration:
    image_size: tuple[int, int]
    pattern_size: tuple[int, int]
    square_size_mm: float
    camera_matrix: np.ndarray
    distortion: np.ndarray
    rms_error_px: float
    mean_reprojection_error_px: float
    sample_count: int


@dataclass
class GroundCalibration:
    pattern_size: tuple[int, int]
    square_size_mm: float
    bottom_center_distance_mm: float
    bottom_reference_pixel: tuple[float, float]
    bottom_reference_board_mm: np.ndarray
    homography_undistorted_pixel_to_vehicle_mm: np.ndarray
    homography_vehicle_mm_to_undistorted_pixel: np.ndarray
    rotation_vector_board_to_camera: np.ndarray
    translation_vector_board_to_camera_mm: np.ndarray
    camera_position_vehicle_mm: np.ndarray
    reprojection_error_px: float


def make_intrinsic_object_points(
    pattern_size: tuple[int, int], square_size_mm: float
) -> np.ndarray:
    columns, rows = pattern_size
    points = np.zeros((columns * rows, 3), dtype=np.float32)
    grid_x, grid_y = np.meshgrid(
        np.arange(columns, dtype=np.float32),
        np.arange(rows, dtype=np.float32),
    )
    points[:, 0] = grid_x.reshape(-1) * square_size_mm
    points[:, 1] = grid_y.reshape(-1) * square_size_mm
    return points


def make_ground_object_points(
    pattern_size: tuple[int, int], square_size_mm: float
) -> np.ndarray:
    columns, rows = pattern_size
    points = np.zeros((columns * rows, 3), dtype=np.float32)
    index = 0
    for row in range(rows):
        for column in range(columns):
            points[index, 0] = (
                column - (columns - 1) * 0.5
            ) * square_size_mm
            points[index, 1] = (rows - 1 - row) * square_size_mm
            index += 1
    return points


def normalize_corner_order(
    corners: np.ndarray, pattern_size: tuple[int, int]
) -> np.ndarray:
    columns, rows = pattern_size
    grid = corners.reshape(rows, columns, 2).copy()
    if float(np.mean(grid[0, :, 1])) > float(np.mean(grid[-1, :, 1])):
        grid = grid[::-1, :, :]
    if float(np.mean(grid[:, 0, 0])) > float(np.mean(grid[:, -1, 0])):
        grid = grid[:, ::-1, :]
    return np.ascontiguousarray(grid.reshape(-1, 1, 2), dtype=np.float32)


def detect_chessboard(
    rgb_image: np.ndarray, pattern_size: tuple[int, int]
) -> np.ndarray | None:
    gray = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2GRAY)
    sb_flags = (
        cv2.CALIB_CB_NORMALIZE_IMAGE
        | cv2.CALIB_CB_EXHAUSTIVE
        | cv2.CALIB_CB_ACCURACY
    )
    found, corners = cv2.findChessboardCornersSB(gray, pattern_size, sb_flags)
    if not found:
        classic_flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
        found, corners = cv2.findChessboardCorners(
            gray, pattern_size, flags=classic_flags
        )
        if found:
            corners = cv2.cornerSubPix(
                gray,
                corners,
                (5, 5),
                (-1, -1),
                (
                    cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER,
                    40,
                    0.001,
                ),
            )
    if not found or corners is None:
        return None
    return normalize_corner_order(corners, pattern_size)


def rotate_to_first_person(image: np.ndarray) -> np.ndarray:
    """Convert the upside-down sensor image to the vehicle's forward view."""
    return cv2.rotate(image, cv2.ROTATE_180)


def calibrate_intrinsics(
    image_points: list[np.ndarray],
    image_size: tuple[int, int],
    pattern_size: tuple[int, int],
    square_size_mm: float,
) -> IntrinsicCalibration:
    if len(image_points) < 5:
        raise ValueError("至少需要 5 张棋盘格图像，建议采集 10 至 20 张。")
    object_template = make_intrinsic_object_points(pattern_size, square_size_mm)
    object_points = [object_template.copy() for _ in image_points]
    rms, camera_matrix, distortion, rotation_vectors, translation_vectors = (
        cv2.calibrateCamera(
            object_points,
            image_points,
            image_size,
            None,
            None,
        )
    )
    errors: list[float] = []
    for object_view, image_view, rotation, translation in zip(
        object_points,
        image_points,
        rotation_vectors,
        translation_vectors,
    ):
        projected, _ = cv2.projectPoints(
            object_view, rotation, translation, camera_matrix, distortion
        )
        errors.append(
            float(cv2.norm(image_view, projected, cv2.NORM_L2) / len(projected))
        )
    return IntrinsicCalibration(
        image_size=image_size,
        pattern_size=pattern_size,
        square_size_mm=square_size_mm,
        camera_matrix=camera_matrix,
        distortion=distortion.reshape(-1),
        rms_error_px=float(rms),
        mean_reprojection_error_px=float(np.mean(errors)),
        sample_count=len(image_points),
    )


def transform_points(homography: np.ndarray, points: np.ndarray) -> np.ndarray:
    points_array = np.asarray(points, dtype=np.float64).reshape(-1, 1, 2)
    return cv2.perspectiveTransform(points_array, homography).reshape(-1, 2)


def calibrate_ground_plane(
    corners: np.ndarray,
    intrinsic: IntrinsicCalibration,
    pattern_size: tuple[int, int],
    square_size_mm: float,
    bottom_center_distance_mm: float,
) -> GroundCalibration:
    if pattern_size != intrinsic.pattern_size:
        raise ValueError("地面棋盘格内角数量必须与内参标定一致。")
    if not np.isclose(square_size_mm, intrinsic.square_size_mm):
        raise ValueError("地面棋盘格边长必须与内参标定一致。")
    object_points = make_ground_object_points(pattern_size, square_size_mm)
    solved, rotation_vector, translation_vector = cv2.solvePnP(
        object_points,
        corners,
        intrinsic.camera_matrix,
        intrinsic.distortion,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    if not solved:
        raise ValueError("无法从当前棋盘格图像求出相机外参。")

    projected, _ = cv2.projectPoints(
        object_points,
        rotation_vector,
        translation_vector,
        intrinsic.camera_matrix,
        intrinsic.distortion,
    )
    reprojection_error = float(
        cv2.norm(corners, projected, cv2.NORM_L2) / len(projected)
    )

    undistorted_corners = cv2.undistortPoints(
        corners,
        intrinsic.camera_matrix,
        intrinsic.distortion,
        P=intrinsic.camera_matrix,
    ).reshape(-1, 2)
    homography_undistorted_to_board, _ = cv2.findHomography(
        undistorted_corners,
        object_points[:, :2],
        method=0,
    )
    if homography_undistorted_to_board is None:
        raise ValueError("无法计算图像到地面的单应矩阵。")

    width, height = intrinsic.image_size
    bottom_reference_pixel = ((width - 1) * 0.5, float(height - 1))
    bottom_reference_undistorted = cv2.undistortPoints(
        np.asarray(bottom_reference_pixel, dtype=np.float64).reshape(1, 1, 2),
        intrinsic.camera_matrix,
        intrinsic.distortion,
        P=intrinsic.camera_matrix,
    ).reshape(1, 2)
    bottom_reference_board = transform_points(
        homography_undistorted_to_board, bottom_reference_undistorted
    )[0]

    board_to_vehicle = np.asarray(
        [
            [1.0, 0.0, -bottom_reference_board[0]],
            [0.0, 1.0, bottom_center_distance_mm - bottom_reference_board[1]],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    homography_undistorted_to_vehicle = (
        board_to_vehicle @ homography_undistorted_to_board
    )
    homography_vehicle_to_undistorted = np.linalg.inv(
        homography_undistorted_to_vehicle
    )

    rotation_matrix, _ = cv2.Rodrigues(rotation_vector)
    camera_position_board = -rotation_matrix.T @ translation_vector
    camera_position_vehicle = camera_position_board.reshape(3)
    camera_position_vehicle[0] -= bottom_reference_board[0]
    camera_position_vehicle[1] += (
        bottom_center_distance_mm - bottom_reference_board[1]
    )

    return GroundCalibration(
        pattern_size=pattern_size,
        square_size_mm=square_size_mm,
        bottom_center_distance_mm=bottom_center_distance_mm,
        bottom_reference_pixel=bottom_reference_pixel,
        bottom_reference_board_mm=bottom_reference_board,
        homography_undistorted_pixel_to_vehicle_mm=(
            homography_undistorted_to_vehicle
        ),
        homography_vehicle_mm_to_undistorted_pixel=(
            homography_vehicle_to_undistorted
        ),
        rotation_vector_board_to_camera=rotation_vector.reshape(3),
        translation_vector_board_to_camera_mm=translation_vector.reshape(3),
        camera_position_vehicle_mm=camera_position_vehicle,
        reprojection_error_px=reprojection_error,
    )


def build_ground_lookup_table(
    intrinsic: IntrinsicCalibration,
    ground: GroundCalibration,
    output_size: tuple[int, int] = RUNTIME_IMAGE_SIZE,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    calibration_width, calibration_height = intrinsic.image_size
    width, height = output_size
    pixel_x, pixel_y = np.meshgrid(
        np.arange(width, dtype=np.float64),
        np.arange(height, dtype=np.float64),
    )
    scale_x = (calibration_width - 1) / max(1, width - 1)
    scale_y = (calibration_height - 1) / max(1, height - 1)
    raw_pixels = np.column_stack(
        (pixel_x.reshape(-1) * scale_x, pixel_y.reshape(-1) * scale_y)
    )
    normalized_rays = cv2.undistortPoints(
        raw_pixels.reshape(-1, 1, 2),
        intrinsic.camera_matrix,
        intrinsic.distortion,
    ).reshape(-1, 2)
    undistorted_pixels = cv2.undistortPoints(
        raw_pixels.reshape(-1, 1, 2),
        intrinsic.camera_matrix,
        intrinsic.distortion,
        P=intrinsic.camera_matrix,
    ).reshape(-1, 2)
    ground_points = transform_points(
        ground.homography_undistorted_pixel_to_vehicle_mm,
        undistorted_pixels,
    )

    rotation_matrix, _ = cv2.Rodrigues(
        ground.rotation_vector_board_to_camera
    )
    plane_normal_camera = rotation_matrix[:, 2]
    plane_offset = float(
        plane_normal_camera
        @ ground.translation_vector_board_to_camera_mm.reshape(3)
    )
    rays = np.column_stack(
        (normalized_rays, np.ones(len(normalized_rays), dtype=np.float64))
    )
    denominators = rays @ plane_normal_camera
    with np.errstate(divide="ignore", invalid="ignore"):
        ray_scales = plane_offset / denominators
    valid = (
        np.isfinite(ground_points).all(axis=1)
        & np.isfinite(ray_scales)
        & (ray_scales > 0.0)
        & (np.abs(ground_points[:, 0]) < 1_000_000.0)
        & (np.abs(ground_points[:, 1]) < 1_000_000.0)
    )
    x_mm = ground_points[:, 0].reshape(height, width).astype(np.float32)
    y_mm = ground_points[:, 1].reshape(height, width).astype(np.float32)
    valid_mask = valid.reshape(height, width)
    x_mm[~valid_mask] = np.nan
    y_mm[~valid_mask] = np.nan
    return x_mm, y_mm, valid_mask


def scaled_camera_matrix(
    camera_matrix: np.ndarray,
    source_size: tuple[int, int],
    target_size: tuple[int, int],
) -> np.ndarray:
    source_width, source_height = source_size
    target_width, target_height = target_size
    scale_x = (target_width - 1) / max(1, source_width - 1)
    scale_y = (target_height - 1) / max(1, source_height - 1)
    result = camera_matrix.copy()
    result[0, 0] *= scale_x
    result[0, 2] *= scale_x
    result[1, 1] *= scale_y
    result[1, 2] *= scale_y
    return result


def calibration_to_dict(
    intrinsic: IntrinsicCalibration, ground: GroundCalibration
) -> dict[str, object]:
    runtime_camera_matrix = scaled_camera_matrix(
        intrinsic.camera_matrix,
        intrinsic.image_size,
        RUNTIME_IMAGE_SIZE,
    )
    calibration_width, calibration_height = intrinsic.image_size
    runtime_width, runtime_height = RUNTIME_IMAGE_SIZE
    runtime_to_calibration = np.asarray(
        [
            [(calibration_width - 1) / (runtime_width - 1), 0.0, 0.0],
            [0.0, (calibration_height - 1) / (runtime_height - 1), 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    runtime_homography = (
        ground.homography_undistorted_pixel_to_vehicle_mm
        @ runtime_to_calibration
    )
    return {
        "format_version": 2,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "coordinate_system": {
            "origin": "vehicle center estimate",
            "x_positive": "right in top view",
            "y_positive": "forward/up in top view",
            "heading_positive": "counter-clockwise",
            "units": "millimeters",
        },
        "image": {
            "calibration_width": intrinsic.image_size[0],
            "calibration_height": intrinsic.image_size[1],
            "calibration_pixel_format": "MJPEG decoded on PC",
            "runtime_width": RUNTIME_IMAGE_SIZE[0],
            "runtime_height": RUNTIME_IMAGE_SIZE[1],
            "runtime_mapping": "edge-aligned full-frame scaling",
            "orientation": "vehicle_first_person",
            "sensor_to_first_person_rotation_degrees": IMAGE_ROTATION_DEGREES,
            "runtime_raw_sensor_to_first_person": {
                "x": "runtime_width - 1 - x_raw",
                "y": "runtime_height - 1 - y_raw",
            },
        },
        "checkerboard": {
            "inner_columns": intrinsic.pattern_size[0],
            "inner_rows": intrinsic.pattern_size[1],
            "square_size_mm": intrinsic.square_size_mm,
        },
        "intrinsic": {
            "sample_count": intrinsic.sample_count,
            "rms_error_px": intrinsic.rms_error_px,
            "mean_reprojection_error_px": (
                intrinsic.mean_reprojection_error_px
            ),
            "camera_matrix": intrinsic.camera_matrix.tolist(),
            "runtime_camera_matrix": runtime_camera_matrix.tolist(),
            "distortion_coefficients": intrinsic.distortion.tolist(),
        },
        "ground_extrinsic": {
            "bottom_center_distance_mm": ground.bottom_center_distance_mm,
            "bottom_reference_pixel": list(ground.bottom_reference_pixel),
            "bottom_reference_board_mm": (
                ground.bottom_reference_board_mm.tolist()
            ),
            "reprojection_error_px": ground.reprojection_error_px,
            "homography_undistorted_pixel_to_vehicle_ground_mm": (
                ground.homography_undistorted_pixel_to_vehicle_mm.tolist()
            ),
            "homography_vehicle_ground_mm_to_undistorted_pixel": (
                ground.homography_vehicle_mm_to_undistorted_pixel.tolist()
            ),
            "runtime_homography_undistorted_pixel_to_vehicle_ground_mm": (
                runtime_homography.tolist()
            ),
            "rotation_vector_board_to_camera": (
                ground.rotation_vector_board_to_camera.tolist()
            ),
            "translation_vector_board_to_camera_mm": (
                ground.translation_vector_board_to_camera_mm.tolist()
            ),
            "camera_position_vehicle_mm": (
                ground.camera_position_vehicle_mm.tolist()
            ),
        },
    }


def save_calibration(
    json_path: Path,
    intrinsic: IntrinsicCalibration,
    ground: GroundCalibration,
) -> Path:
    json_path.parent.mkdir(parents=True, exist_ok=True)
    json_path.write_text(
        json.dumps(calibration_to_dict(intrinsic, ground), indent=2),
        encoding="utf-8",
    )
    x_mm, y_mm, valid = build_ground_lookup_table(intrinsic, ground)
    lookup_path = json_path.with_name(f"{json_path.stem}_pixel_lut.npz")
    np.savez_compressed(
        lookup_path,
        x_mm=x_mm,
        y_mm=y_mm,
        valid=valid,
        image_width=RUNTIME_IMAGE_SIZE[0],
        image_height=RUNTIME_IMAGE_SIZE[1],
        calibration_image_width=intrinsic.image_size[0],
        calibration_image_height=intrinsic.image_size[1],
    )
    return lookup_path


class CalibrationApp:
    def __init__(self, root: tk.Tk, initial_port: str, baud: int) -> None:
        self.root = root
        self.baud = baud
        self.events: queue.Queue[tuple[str, object]] = queue.Queue()
        self.serial_worker = CalibrationSerialWorker(self.events)
        self.latest_rgb: np.ndarray | None = None
        self.latest_corners: np.ndarray | None = None
        self.latest_image_size: tuple[int, int] | None = None
        self.image_points: list[np.ndarray] = []
        self.sample_signature: tuple[tuple[int, int], float, tuple[int, int]] | None = None
        self.intrinsic: IntrinsicCalibration | None = None
        self.ground: GroundCalibration | None = None
        self.preview_photo: ImageTk.PhotoImage | None = None
        self.port_descriptions: dict[str, str] = {}
        self.frame_times: list[float] = []

        root.title("ESP32 相机地面外参标定")
        root.resizable(False, False)
        root.protocol("WM_DELETE_WINDOW", self.close)

        self.port_var = tk.StringVar(value=initial_port)
        self.columns_var = tk.IntVar(value=10)
        self.rows_var = tk.IntVar(value=7)
        self.square_size_var = tk.DoubleVar(value=25.0)
        self.bottom_distance_var = tk.DoubleVar(value=80.0)
        self.connection_var = tk.StringVar(value="未连接")
        self.detection_var = tk.StringVar(value="等待图像")
        self.samples_var = tk.StringVar(value="内参样本：0")
        self.intrinsic_var = tk.StringVar(value="内参：未标定")
        self.ground_var = tk.StringVar(value="地面外参：未标定")

        self._build_ui()
        self.refresh_ports()
        root.after(40, self._poll_events)

    def _build_ui(self) -> None:
        outer = ttk.Frame(self.root, padding=12)
        outer.grid(sticky="nsew")

        connection = ttk.Frame(outer)
        connection.grid(row=0, column=0, sticky="ew")
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
        ttk.Label(connection, textvariable=self.connection_var).grid(
            row=0, column=4, padx=(10, 0)
        )

        settings = ttk.LabelFrame(outer, text="棋盘格与坐标参数", padding=8)
        settings.grid(row=1, column=0, sticky="ew", pady=(10, 0))
        self._number_input(settings, "横向内角", self.columns_var, 0, 0)
        self._number_input(settings, "纵向内角", self.rows_var, 0, 2)
        self._number_input(settings, "格边长 mm", self.square_size_var, 0, 4)
        self._number_input(
            settings, "底边中心距 mm", self.bottom_distance_var, 0, 6
        )

        preview_panel = ttk.LabelFrame(outer, text="相机与角点", padding=6)
        preview_panel.grid(row=2, column=0, pady=(10, 0))
        self.preview_label = ttk.Label(
            preview_panel, text="等待数据", anchor="center", width=80
        )
        self.preview_label.grid()

        actions = ttk.Frame(outer)
        actions.grid(row=3, column=0, sticky="ew", pady=(10, 0))
        ttk.Button(actions, text="采集内参样本", command=self.capture_sample).grid(
            row=0, column=0
        )
        ttk.Button(actions, text="撤销样本", command=self.remove_sample).grid(
            row=0, column=1, padx=6
        )
        ttk.Button(actions, text="计算内参", command=self.run_intrinsic).grid(
            row=0, column=2
        )
        ttk.Button(actions, text="标定地面外参", command=self.run_ground).grid(
            row=0, column=3, padx=6
        )
        ttk.Button(actions, text="导出结果", command=self.export_results).grid(
            row=0, column=4
        )

        details = ttk.Frame(outer)
        details.grid(row=4, column=0, sticky="ew", pady=(10, 0))
        ttk.Label(details, textvariable=self.detection_var).grid(
            row=0, column=0, sticky="w"
        )
        ttk.Label(details, textvariable=self.samples_var).grid(
            row=1, column=0, sticky="w"
        )
        ttk.Label(details, textvariable=self.intrinsic_var).grid(
            row=2, column=0, sticky="w"
        )
        ttk.Label(details, textvariable=self.ground_var).grid(
            row=3, column=0, sticky="w"
        )

    @staticmethod
    def _number_input(
        parent: ttk.LabelFrame,
        label: str,
        variable: tk.Variable,
        row: int,
        column: int,
    ) -> None:
        ttk.Label(parent, text=label).grid(row=row, column=column, padx=(0, 4))
        ttk.Entry(parent, textvariable=variable, width=8).grid(
            row=row, column=column + 1, padx=(0, 10)
        )

    def current_pattern(self) -> tuple[int, int]:
        pattern = (int(self.columns_var.get()), int(self.rows_var.get()))
        if pattern[0] < 3 or pattern[1] < 3:
            raise ValueError("横向和纵向内角数量都必须至少为 3。")
        return pattern

    def current_square_size(self) -> float:
        value = float(self.square_size_var.get())
        if value <= 0.0:
            raise ValueError("棋盘格边长必须大于 0。")
        return value

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
            self.connection_var.set("未发现串口")

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
            self.serial_worker.write(b"xCALIB,1\n")
            time.sleep(0.08)
            self.serial_worker.switch_baud(CALIBRATION_BAUD)
        except Exception as exc:
            self.serial_worker.disconnect(restore_logs=False)
            messagebox.showerror("无法连接", str(exc))
            return
        description = self.port_descriptions.get(port_name, "")
        self.connection_var.set(
            f"已连接 {port_name} @ {CALIBRATION_BAUD}  {description}"
        )
        self.connect_button.configure(text="断开")

    def disconnect(self) -> None:
        self.serial_worker.disconnect()
        self.connection_var.set("未连接")
        self.connect_button.configure(text="连接")

    def _poll_events(self) -> None:
        newest_frame: CalibrationJpegFrame | None = None
        try:
            while True:
                event, payload = self.events.get_nowait()
                if event == "frame":
                    newest_frame = payload  # type: ignore[assignment]
                elif event == "error":
                    self.disconnect()
                    messagebox.showerror("串口错误", str(payload))
                elif event == "warning":
                    self.connection_var.set(f"收到无效帧：{payload}")
        except queue.Empty:
            pass
        if newest_frame is not None:
            self._process_frame(newest_frame)
        self.root.after(40, self._poll_events)

    def _process_frame(self, frame: CalibrationJpegFrame) -> None:
        encoded = np.frombuffer(frame.jpeg, dtype=np.uint8)
        decoded_bgr = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        if decoded_bgr is None:
            self.connection_var.set("JPEG 解码失败")
            return
        if decoded_bgr.shape[1] != frame.width or decoded_bgr.shape[0] != frame.height:
            self.connection_var.set(
                f"JPEG 尺寸不一致：header={frame.width}x{frame.height}"
            )
            return
        if (frame.width, frame.height) != CALIBRATION_IMAGE_SIZE:
            self.connection_var.set(
                f"标定必须使用 {CALIBRATION_IMAGE_SIZE[0]}x"
                f"{CALIBRATION_IMAGE_SIZE[1]}，当前为 {frame.width}x{frame.height}"
            )
            return
        first_person_bgr = rotate_to_first_person(decoded_bgr)
        rgb = cv2.cvtColor(first_person_bgr, cv2.COLOR_BGR2RGB)
        self.latest_rgb = rgb
        self.latest_image_size = (frame.width, frame.height)
        try:
            pattern = self.current_pattern()
            corners = detect_chessboard(rgb, pattern)
        except (ValueError, tk.TclError):
            corners = None
        self.latest_corners = corners

        overlay = rgb.copy()
        if corners is not None:
            cv2.drawChessboardCorners(overlay, pattern, corners, True)
            self.detection_var.set(
                f"已识别 {pattern[0]} x {pattern[1]} 个内角；"
                f"第一视角={frame.width}x{frame.height} sequence={frame.sequence}"
            )
        else:
            self.detection_var.set(
                f"未识别棋盘格；第一视角={frame.width}x{frame.height} "
                f"sequence={frame.sequence}"
            )
        if self.ground is not None:
            point = tuple(int(round(value)) for value in self.ground.bottom_reference_pixel)
            cv2.drawMarker(
                overlay,
                point,
                (255, 0, 0),
                markerType=cv2.MARKER_CROSS,
                markerSize=10,
                thickness=1,
            )

        preview_scale = min(1.0, 720.0 / frame.width, 540.0 / frame.height)
        preview_size = (
            max(1, int(round(frame.width * preview_scale))),
            max(1, int(round(frame.height * preview_scale))),
        )
        image = Image.fromarray(overlay).resize(
            preview_size, Image.Resampling.LANCZOS
        )
        self.preview_photo = ImageTk.PhotoImage(image)
        self.preview_label.configure(image=self.preview_photo, text="")

        now = time.monotonic()
        self.frame_times.append(now)
        self.frame_times = [value for value in self.frame_times if now - value <= 3.0]
        if len(self.frame_times) >= 2:
            fps = (len(self.frame_times) - 1) / (
                self.frame_times[-1] - self.frame_times[0]
            )
            self.connection_var.set(
                f"{self.port_var.get()} @ {CALIBRATION_BAUD}  "
                f"{frame.width}x{frame.height} JPEG {fps:.1f} fps"
            )

    def capture_sample(self) -> None:
        if self.latest_corners is None or self.latest_image_size is None:
            messagebox.showerror("不能采集", "当前画面没有识别到完整棋盘格。")
            return
        try:
            signature = (
                self.current_pattern(),
                self.current_square_size(),
                self.latest_image_size,
            )
        except (ValueError, tk.TclError) as exc:
            messagebox.showerror("参数错误", str(exc))
            return
        if self.sample_signature is not None and signature != self.sample_signature:
            messagebox.showerror("参数已改变", "请先清空现有样本再改变棋盘格参数。")
            return
        self.sample_signature = signature
        self.image_points.append(self.latest_corners.copy())
        self.intrinsic = None
        self.ground = None
        self.samples_var.set(f"内参样本：{len(self.image_points)}")
        self.intrinsic_var.set("内参：样本已改变，需要重新计算")
        self.ground_var.set("地面外参：未标定")

    def remove_sample(self) -> None:
        if self.image_points:
            self.image_points.pop()
        if not self.image_points:
            self.sample_signature = None
        self.intrinsic = None
        self.ground = None
        self.samples_var.set(f"内参样本：{len(self.image_points)}")
        self.intrinsic_var.set("内参：未标定")
        self.ground_var.set("地面外参：未标定")

    def run_intrinsic(self) -> None:
        if self.sample_signature is None:
            messagebox.showerror("无法标定", "请先采集棋盘格内参样本。")
            return
        pattern, square_size, image_size = self.sample_signature
        try:
            self.intrinsic = calibrate_intrinsics(
                self.image_points, image_size, pattern, square_size
            )
        except (ValueError, cv2.error) as exc:
            messagebox.showerror("内参标定失败", str(exc))
            return
        self.ground = None
        self.intrinsic_var.set(
            f"内参：RMS={self.intrinsic.rms_error_px:.3f}px，"
            f"平均重投影={self.intrinsic.mean_reprojection_error_px:.3f}px"
        )
        self.ground_var.set("地面外参：未标定")

    def run_ground(self) -> None:
        if self.intrinsic is None:
            messagebox.showerror("无法标定", "请先完成相机内参标定。")
            return
        if self.latest_corners is None:
            messagebox.showerror("无法标定", "当前画面没有识别到完整棋盘格。")
            return
        try:
            bottom_distance = float(self.bottom_distance_var.get())
            if bottom_distance <= 0.0:
                raise ValueError("底边中心到小车中心的距离必须大于 0。")
            self.ground = calibrate_ground_plane(
                self.latest_corners,
                self.intrinsic,
                self.current_pattern(),
                self.current_square_size(),
                bottom_distance,
            )
        except (ValueError, cv2.error, np.linalg.LinAlgError, tk.TclError) as exc:
            messagebox.showerror("地面外参标定失败", str(exc))
            return
        position = self.ground.camera_position_vehicle_mm
        self.ground_var.set(
            f"地面外参：重投影={self.ground.reprojection_error_px:.3f}px，"
            f"相机估计位置=({position[0]:.1f}, {position[1]:.1f}, "
            f"{position[2]:.1f})mm"
        )

    def export_results(self) -> None:
        if self.intrinsic is None or self.ground is None:
            messagebox.showerror("不能导出", "请先完成内参和地面外参标定。")
            return
        selected = filedialog.asksaveasfilename(
            title="保存相机标定结果",
            defaultextension=".json",
            initialfile="camera_ground_calibration.json",
            filetypes=[("JSON", "*.json")],
        )
        if not selected:
            return
        try:
            lookup_path = save_calibration(
                Path(selected), self.intrinsic, self.ground
            )
        except (OSError, ValueError, cv2.error) as exc:
            messagebox.showerror("导出失败", str(exc))
            return
        messagebox.showinfo(
            "导出完成",
            f"参数：{selected}\n像素查找表：{lookup_path}",
        )

    def close(self) -> None:
        self.serial_worker.disconnect()
        self.root.destroy()


def run_self_test() -> None:
    image_size = (640, 480)
    pattern_size = (9, 6)
    square_size = 25.0
    camera_matrix = np.asarray(
        [[580.0, 0.0, 319.5], [0.0, 588.0, 239.5], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    distortion = np.asarray([-0.08, 0.015, 0.001, -0.001, 0.0])
    synthetic_image_points: list[np.ndarray] = []
    intrinsic_object_points = make_intrinsic_object_points(
        pattern_size, square_size
    )
    synthetic_poses = [
        ((-0.22, -0.16, -0.08), (-112.0, -72.0, 455.0)),
        ((-0.18, 0.14, 0.10), (-92.0, -70.0, 470.0)),
        ((0.16, -0.18, 0.05), (-110.0, -55.0, 440.0)),
        ((0.20, 0.13, -0.12), (-88.0, -58.0, 485.0)),
        ((-0.10, -0.24, 0.15), (-120.0, -65.0, 500.0)),
        ((0.11, 0.22, -0.16), (-85.0, -75.0, 460.0)),
        ((-0.28, 0.05, 0.04), (-105.0, -48.0, 475.0)),
        ((0.25, -0.04, -0.04), (-95.0, -82.0, 490.0)),
    ]
    for rotation_values, translation_values in synthetic_poses:
        projected, _ = cv2.projectPoints(
            intrinsic_object_points,
            np.asarray(rotation_values, dtype=np.float64).reshape(3, 1),
            np.asarray(translation_values, dtype=np.float64).reshape(3, 1),
            camera_matrix,
            distortion,
        )
        synthetic_image_points.append(projected.astype(np.float32))
    solved_intrinsic = calibrate_intrinsics(
        synthetic_image_points,
        image_size,
        pattern_size,
        square_size,
    )
    if solved_intrinsic.rms_error_px > 0.01:
        raise AssertionError(
            f"intrinsic calibration RMS too high: {solved_intrinsic.rms_error_px}"
        )

    intrinsic = IntrinsicCalibration(
        image_size=image_size,
        pattern_size=pattern_size,
        square_size_mm=square_size,
        camera_matrix=camera_matrix,
        distortion=distortion,
        rms_error_px=0.0,
        mean_reprojection_error_px=0.0,
        sample_count=10,
    )
    object_points = make_ground_object_points(pattern_size, square_size)
    rotation_vector = np.asarray([[-1.05], [0.02], [0.01]], dtype=np.float64)
    translation_vector = np.asarray([[0.0], [-15.0], [430.0]], dtype=np.float64)
    corners, _ = cv2.projectPoints(
        object_points,
        rotation_vector,
        translation_vector,
        camera_matrix,
        distortion,
    )
    bottom_center_distance_mm = 80.0
    ground = calibrate_ground_plane(
        corners.astype(np.float32),
        intrinsic,
        pattern_size,
        square_size,
        bottom_center_distance_mm,
    )
    bottom_raw = np.asarray(ground.bottom_reference_pixel).reshape(1, 1, 2)
    bottom_undistorted = cv2.undistortPoints(
        bottom_raw,
        camera_matrix,
        distortion,
        P=camera_matrix,
    ).reshape(-1, 2)
    bottom_vehicle = transform_points(
        ground.homography_undistorted_pixel_to_vehicle_mm,
        bottom_undistorted,
    )[0]
    if not np.allclose(
        bottom_vehicle, [0.0, bottom_center_distance_mm], atol=1e-4
    ):
        raise AssertionError(f"bottom reference mismatch: {bottom_vehicle}")
    test_image = np.arange(12, dtype=np.uint8).reshape(2, 2, 3)
    if not np.array_equal(
        rotate_to_first_person(test_image), test_image[::-1, ::-1]
    ):
        raise AssertionError("180-degree first-person rotation failed")
    x_mm, y_mm, valid = build_ground_lookup_table(intrinsic, ground)
    if x_mm.shape != (120, 160) or y_mm.shape != (120, 160):
        raise AssertionError("lookup table shape mismatch")
    if not bool(valid[119, 79]):
        raise AssertionError("bottom-center lookup point is invalid")
    if parse_calibration_header(b"@CALJPEG,7,640,480,12345\n") != (
        7,
        640,
        480,
        12345,
    ):
        raise AssertionError("calibration protocol header parsing failed")
    with tempfile.TemporaryDirectory() as temporary_directory:
        json_path = Path(temporary_directory) / "calibration.json"
        lookup_path = save_calibration(json_path, intrinsic, ground)
        if not json_path.is_file() or not lookup_path.is_file():
            raise AssertionError("calibration export failed")
        json.loads(json_path.read_text(encoding="utf-8"))
    print("camera_extrinsic_calibrator self-test: PASS")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Calibrate ESP32 camera intrinsics and ground-plane extrinsics."
    )
    parser.add_argument("--port", default="", help="initial serial port")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--self-test", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.self_test:
        run_self_test()
        return 0
    root = tk.Tk()
    CalibrationApp(root, args.port.upper(), args.baud)
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
