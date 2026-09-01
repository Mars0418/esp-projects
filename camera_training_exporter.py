"""Export annotated camera sessions to a compact temporal training dataset."""

from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import tempfile

import numpy as np
from PIL import Image

from camera_corner_annotator import (
    FINISH_POINT_KEYS,
    GroundLookup,
    SCENE_FINISH_T,
    SCENE_INVALID,
    SCENE_PATH,
    calibrated_finish_geometry,
    calibrated_geometry,
    label_scene_type,
    load_jsonl,
    load_session_geometry,
)


SCENE_CLASS = {SCENE_PATH: 0, SCENE_FINISH_T: 1, SCENE_INVALID: 2}
DEFAULT_PATH_POINT_COUNT = 24
DEFAULT_PATH_STEP_MM = 25.0
DEFAULT_CORNER_CAPACITY = 3
DEFAULT_CLIP_LENGTH = 5
DEFAULT_MAX_GAP_MS = 1000.0


@dataclass
class LoadedFrame:
    session_id: str
    split: str
    sample: dict[str, object]
    label: dict[str, object]
    geometry: object
    session_path: Path


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        while chunk := source.read(1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


def discover_sessions(dataset_root: Path) -> list[Path]:
    sessions = sorted(
        path
        for path in dataset_root.glob("session_*")
        if path.is_dir()
        and (path / "manifest.json").is_file()
        and (path / "samples.jsonl").is_file()
        and (path / "path_labels.jsonl").is_file()
    )
    if not sessions:
        raise FileNotFoundError(f"no fully annotated sessions under {dataset_root}")
    return sessions


def normalize_corner_indices(label: dict[str, object]) -> list[int]:
    values = label.get("corner_indices")
    if isinstance(values, list):
        return sorted(set(int(value) for value in values))
    old_corner = label.get("corner_index")
    return [] if old_corner is None else [int(old_corner)]


def parse_timestamp(value: object) -> datetime:
    if not isinstance(value, str):
        raise ValueError("sample is missing received_at")
    return datetime.fromisoformat(value.replace("Z", "+00:00"))


def resample_ground_path(
    points: list[tuple[float, float]],
    point_count: int,
    step_mm: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, float]:
    output = np.zeros((point_count, 2), dtype=np.float32)
    distances = np.zeros(point_count, dtype=np.float32)
    valid = np.zeros(point_count, dtype=np.uint8)
    if not points:
        return output, distances, valid, 0.0

    source = np.asarray(points, dtype=np.float64)
    if len(source) == 1:
        output[0] = source[0]
        valid[0] = 1
        return output, distances, valid, 0.0
    segment_lengths = np.linalg.norm(np.diff(source, axis=0), axis=1)
    cumulative = np.concatenate(([0.0], np.cumsum(segment_lengths)))
    total_length = float(cumulative[-1])
    if total_length <= 1e-6:
        output[0] = source[0]
        valid[0] = 1
        return output, distances, valid, total_length

    targets = list(np.arange(0.0, total_length + 1e-6, step_mm))
    if targets and total_length - targets[-1] > 1e-3 and len(targets) < point_count:
        targets.append(total_length)
    targets = targets[:point_count]
    for output_index, target in enumerate(targets):
        segment = min(
            len(segment_lengths) - 1,
            int(np.searchsorted(cumulative, target, side="right") - 1),
        )
        length = segment_lengths[segment]
        fraction = 0.0 if length <= 1e-9 else (
            target - cumulative[segment]
        ) / length
        output[output_index] = (
            source[segment] + fraction * (source[segment + 1] - source[segment])
        )
        distances[output_index] = target
        valid[output_index] = 1
    return output, distances, valid, total_length


def fit_ground_segments(
    points: list[tuple[float, float]],
    corner_indices: list[int],
    capacity: int,
) -> list[dict[str, object]]:
    if len(points) < 2:
        return []
    boundaries = [0, *corner_indices, len(points) - 1]
    if len(boundaries) - 1 > capacity:
        raise ValueError("path has more fitted segments than capacity")
    source = np.asarray(points, dtype=np.float64)
    fitted: list[dict[str, object]] = []
    for start, end in zip(boundaries, boundaries[1:]):
        segment = source[start : end + 1]
        if len(segment) < 2:
            continue
        inliers = segment
        for _ in range(3):
            centroid = inliers.mean(axis=0)
            centered = inliers - centroid
            covariance = centered.T @ centered
            _, eigenvectors = np.linalg.eigh(covariance)
            direction = eigenvectors[:, -1]
            if np.dot(direction, segment[-1] - segment[0]) < 0.0:
                direction = -direction
            normal = np.asarray((direction[1], -direction[0]))
            residuals = np.abs((segment - centroid) @ normal)
            if len(segment) <= 3:
                break
            median = float(np.median(residuals))
            mad = float(np.median(np.abs(residuals - median)))
            threshold = max(8.0, median + 2.5 * 1.4826 * mad)
            next_inliers = segment[residuals <= threshold]
            if len(next_inliers) < 2 or len(next_inliers) == len(inliers):
                break
            inliers = next_inliers
        centroid = inliers.mean(axis=0)
        centered = inliers - centroid
        covariance = centered.T @ centered
        _, eigenvectors = np.linalg.eigh(covariance)
        direction = eigenvectors[:, -1]
        if np.dot(direction, segment[-1] - segment[0]) < 0.0:
            direction = -direction
        direction /= np.linalg.norm(direction)
        normal = np.asarray((direction[1], -direction[0]))
        rho = float(np.dot(normal, centroid))
        residuals = (segment @ normal) - rho
        projections = segment @ direction
        fitted.append(
            {
                "rho_mm": rho,
                "direction_xy": direction.astype(np.float32),
                "fit_rmse_mm": float(np.sqrt(np.mean(residuals**2))),
                "projected_length_mm": float(projections.max() - projections.min()),
                "source_point_count": len(segment),
                "inlier_count": len(inliers),
            }
        )
    return fitted


def distribution(values: list[float]) -> dict[str, float] | None:
    if not values:
        return None
    array = np.asarray(values, dtype=np.float64)
    return {
        "min": float(array.min()),
        "p25": float(np.percentile(array, 25)),
        "median": float(np.median(array)),
        "p75": float(np.percentile(array, 75)),
        "max": float(array.max()),
    }


def session_split(
    session_id: str, validation_sessions: set[str], test_sessions: set[str]
) -> str:
    if session_id in validation_sessions:
        return "validation"
    if session_id in test_sessions:
        return "test"
    return "development"


def load_frames(
    sessions: list[Path],
    validation_sessions: set[str],
    test_sessions: set[str],
) -> tuple[list[LoadedFrame], dict[str, object]]:
    overlap = validation_sessions & test_sessions
    if overlap:
        raise ValueError(f"sessions assigned to both validation and test: {overlap}")
    known_sessions = {path.name for path in sessions}
    unknown = (validation_sessions | test_sessions) - known_sessions
    if unknown:
        raise ValueError(f"unknown split sessions: {sorted(unknown)}")

    frames: list[LoadedFrame] = []
    source_info: dict[str, object] = {}
    for session in sessions:
        samples = load_jsonl(session / "samples.jsonl")
        labels = load_jsonl(session / "path_labels.jsonl")
        labels_by_id = {str(label["sample_id"]): label for label in labels}
        sample_ids = [str(sample["sample_id"]) for sample in samples]
        if len(labels_by_id) != len(labels):
            raise ValueError(f"duplicate label sample_id in {session}")
        missing = set(sample_ids) - set(labels_by_id)
        extra = set(labels_by_id) - set(sample_ids)
        if missing or extra:
            raise ValueError(
                f"sample/label mismatch in {session}: missing={len(missing)} "
                f"extra={len(extra)}"
            )
        geometry = load_session_geometry(session)
        split = session_split(session.name, validation_sessions, test_sessions)
        source_info[session.name] = {
            "path": str(session),
            "split": split,
            "sample_count": len(samples),
            "labels_sha256": file_sha256(session / "path_labels.jsonl"),
        }
        for sample in samples:
            sample_id = str(sample["sample_id"])
            frames.append(
                LoadedFrame(
                    session_id=session.name,
                    split=split,
                    sample=sample,
                    label=labels_by_id[sample_id],
                    geometry=geometry,
                    session_path=session,
                )
            )
    return frames, source_info


def build_temporal_runs(
    index_records: list[dict[str, object]], max_gap_ms: float
) -> list[list[int]]:
    runs: list[list[int]] = []
    current: list[int] = []
    previous: dict[str, object] | None = None
    for record in index_records:
        frame_index = int(record["frame_index"])
        starts_new = previous is None
        if previous is not None:
            starts_new = (
                record["session_id"] != previous["session_id"]
                or int(record["sequence"]) != int(previous["sequence"]) + 1
                or float(record["dt_ms"]) <= 0.0
                or float(record["dt_ms"]) > max_gap_ms
            )
        if starts_new:
            if current:
                runs.append(current)
            current = [frame_index]
        else:
            current.append(frame_index)
        previous = record
    if current:
        runs.append(current)
    return runs


def build_clips(
    runs: list[list[int]],
    index_records: list[dict[str, object]],
    clip_length: int,
) -> list[dict[str, object]]:
    clips: list[dict[str, object]] = []
    for run_id, run in enumerate(runs):
        for offset in range(0, len(run) - clip_length + 1):
            indices = run[offset : offset + clip_length]
            clips.append(
                {
                    "clip_index": len(clips),
                    "run_id": run_id,
                    "session_id": index_records[indices[0]]["session_id"],
                    "split": index_records[indices[0]]["split"],
                    "frame_indices": indices,
                    "dt_ms": [
                        index_records[index]["dt_ms"] for index in indices[1:]
                    ],
                    "target_frame_index": indices[-1],
                }
            )
    return clips


def export_dataset(args: argparse.Namespace) -> Path:
    sessions = (
        [path.resolve() for path in args.session]
        if args.session
        else discover_sessions(args.dataset_root.resolve())
    )
    frames, source_info = load_frames(
        sessions,
        set(args.validation_session),
        set(args.test_session),
    )
    if not frames:
        raise ValueError("no labeled frames to export")

    first_geometry = frames[0].geometry
    source_size = (first_geometry.source_width, first_geometry.source_height)
    roi_size = (first_geometry.width, first_geometry.height)
    for frame in frames[1:]:
        geometry = frame.geometry
        if (
            (geometry.source_width, geometry.source_height) != source_size
            or (geometry.width, geometry.height) != roi_size
            or geometry.first_roi_x_min != first_geometry.first_roi_x_min
            or geometry.first_roi_y_min != first_geometry.first_roi_y_min
        ):
            raise ValueError("all sessions must use the same frame and ROI geometry")

    lookup = GroundLookup(args.calibration.resolve(), source_size)
    frame_count = len(frames)
    roi_width, roi_height = roi_size
    masks = np.zeros((frame_count, 1, roi_height, roi_width), dtype=np.uint8)
    luma = (
        None
        if args.without_luma
        else np.zeros((frame_count, 1, roi_height, roi_width), dtype=np.uint8)
    )
    scene_class = np.zeros(frame_count, dtype=np.int8)
    path_xy_mm = np.zeros(
        (frame_count, args.path_point_count, 2), dtype=np.float32
    )
    path_s_mm = np.zeros(
        (frame_count, args.path_point_count), dtype=np.float32
    )
    path_valid = np.zeros(
        (frame_count, args.path_point_count), dtype=np.uint8
    )
    path_length_mm = np.zeros(frame_count, dtype=np.float32)
    corner_xy_mm = np.zeros(
        (frame_count, args.corner_capacity, 2), dtype=np.float32
    )
    corner_s_mm = np.zeros(
        (frame_count, args.corner_capacity), dtype=np.float32
    )
    corner_angle_deg = np.zeros(
        (frame_count, args.corner_capacity), dtype=np.float32
    )
    corner_direction = np.zeros(
        (frame_count, args.corner_capacity), dtype=np.int8
    )
    corner_valid = np.zeros(
        (frame_count, args.corner_capacity), dtype=np.uint8
    )
    segment_capacity = args.corner_capacity + 1
    segment_rho_mm = np.zeros(
        (frame_count, segment_capacity), dtype=np.float32
    )
    segment_direction_xy = np.zeros(
        (frame_count, segment_capacity, 2), dtype=np.float32
    )
    segment_fit_rmse_mm = np.zeros(
        (frame_count, segment_capacity), dtype=np.float32
    )
    segment_valid = np.zeros(
        (frame_count, segment_capacity), dtype=np.uint8
    )
    finish_junction_mm = np.zeros((frame_count, 2), dtype=np.float32)
    finish_crossbar_length_mm = np.zeros(frame_count, dtype=np.float32)
    finish_stem_crossbar_angle_deg = np.zeros(frame_count, dtype=np.float32)
    finish_valid = np.zeros(frame_count, dtype=np.uint8)

    index_records: list[dict[str, object]] = []
    scene_counts: Counter[str] = Counter()
    format_counts: Counter[int] = Counter()
    path_lengths: list[float] = []
    corner_angles: list[float] = []
    segment_fit_errors: list[float] = []
    finish_lengths: list[float] = []
    finish_angles: list[float] = []
    errors: list[dict[str, object]] = []
    previous_time_by_session: dict[str, datetime] = {}

    for frame_index, frame in enumerate(frames):
        sample = frame.sample
        label = frame.label
        sample_id = str(sample["sample_id"])
        scene = label_scene_type(label)
        scene_counts[scene] += 1
        format_counts[int(label.get("format_version", 0))] += 1
        scene_class[frame_index] = SCENE_CLASS[scene]

        raw_path = frame.session_path / str(sample["raw_image"])
        mask_path = frame.session_path / str(sample["mask_image"])
        raw_image = Image.open(raw_path).convert("L").transpose(
            Image.Transpose.ROTATE_180
        )
        mask_image = Image.open(mask_path).convert("L").transpose(
            Image.Transpose.ROTATE_180
        )
        if raw_image.size != roi_size or mask_image.size != roi_size:
            raise ValueError(f"image size mismatch for {frame.session_id}/{sample_id}")
        masks[frame_index, 0] = (
            np.asarray(mask_image, dtype=np.uint8) >= 128
        ).astype(np.uint8)
        if luma is not None:
            luma[frame_index, 0] = np.asarray(raw_image, dtype=np.uint8)

        points = [
            tuple(map(int, point)) for point in label.get("polyline", [])
        ]
        corners = normalize_corner_indices(label) if scene == SCENE_PATH else []
        ground_points: list[tuple[float, float]] = []
        measurements: dict[str, object] = {}
        if scene != SCENE_INVALID:
            if len(points) < 2:
                errors.append(
                    {"sample_id": sample_id, "error": "fewer than 2 path points"}
                )
            else:
                _, ground_points, measurements = calibrated_geometry(
                    points, corners, frame.geometry, lookup
                )
                if not ground_points:
                    errors.append(
                        {"sample_id": sample_id, "error": "invalid ground path"}
                    )
        if ground_points:
            sampled_xy, sampled_s, sampled_valid, total_length = (
                resample_ground_path(
                    ground_points,
                    args.path_point_count,
                    args.path_step_mm,
                )
            )
            path_xy_mm[frame_index] = sampled_xy
            path_s_mm[frame_index] = sampled_s
            path_valid[frame_index] = sampled_valid
            path_length_mm[frame_index] = total_length
            path_lengths.append(total_length)
            try:
                fitted_segments = fit_ground_segments(
                    ground_points, corners, segment_capacity
                )
            except ValueError as error:
                errors.append({"sample_id": sample_id, "error": str(error)})
                fitted_segments = []
            for output_index, fitted in enumerate(fitted_segments):
                segment_rho_mm[frame_index, output_index] = float(
                    fitted["rho_mm"]
                )
                segment_direction_xy[frame_index, output_index] = fitted[
                    "direction_xy"
                ]
                segment_fit_rmse_mm[frame_index, output_index] = float(
                    fitted["fit_rmse_mm"]
                )
                segment_valid[frame_index, output_index] = 1
                segment_fit_errors.append(float(fitted["fit_rmse_mm"]))

        corner_measurements = measurements.get("corners", [])
        if len(corner_measurements) > args.corner_capacity:
            errors.append(
                {
                    "sample_id": sample_id,
                    "error": f"{len(corner_measurements)} corners exceed capacity",
                }
            )
        for output_index, corner in enumerate(
            corner_measurements[: args.corner_capacity]
        ):
            corner_xy_mm[frame_index, output_index] = (
                float(corner["ground_x_mm"]),
                float(corner["ground_y_mm"]),
            )
            corner_s_mm[frame_index, output_index] = float(
                corner["path_distance_from_near_point_mm"]
            )
            angle = corner.get("turn_angle_deg")
            if not isinstance(angle, (float, int)):
                errors.append(
                    {"sample_id": sample_id, "error": "corner angle unavailable"}
                )
                continue
            corner_angle_deg[frame_index, output_index] = float(angle)
            corner_direction[frame_index, output_index] = 1 if angle > 0 else -1
            corner_valid[frame_index, output_index] = 1
            corner_angles.append(abs(float(angle)))

        if scene == SCENE_FINISH_T and ground_points:
            finish = label.get("finish", {})
            if not isinstance(finish, dict) or any(
                key not in finish for key in FINISH_POINT_KEYS
            ):
                errors.append(
                    {"sample_id": sample_id, "error": "incomplete T finish"}
                )
            else:
                local_finish = {
                    key: tuple(map(int, finish[key])) for key in FINISH_POINT_KEYS
                }
                finish_geometry = calibrated_finish_geometry(
                    local_finish, ground_points, frame.geometry, lookup
                )
                finish_ground = finish_geometry["ground_mm"]
                finish_metrics = finish_geometry["measurements"]
                finish_junction_mm[frame_index] = finish_ground["junction"]
                finish_crossbar_length_mm[frame_index] = float(
                    finish_metrics["crossbar_length_mm"]
                )
                finish_lengths.append(
                    float(finish_metrics["crossbar_length_mm"])
                )
                finish_angle = finish_metrics["stem_crossbar_angle_deg"]
                if isinstance(finish_angle, (float, int)):
                    finish_stem_crossbar_angle_deg[frame_index] = float(
                        finish_angle
                    )
                    finish_angles.append(float(finish_angle))
                    finish_valid[frame_index] = 1
                else:
                    errors.append(
                        {"sample_id": sample_id, "error": "T angle unavailable"}
                    )

        received_at = parse_timestamp(sample.get("received_at"))
        previous_time = previous_time_by_session.get(frame.session_id)
        dt_ms = (
            0.0
            if previous_time is None
            else (received_at - previous_time).total_seconds() * 1000.0
        )
        previous_time_by_session[frame.session_id] = received_at
        index_records.append(
            {
                "frame_index": frame_index,
                "session_id": frame.session_id,
                "split": frame.split,
                "sample_id": sample_id,
                "sequence": int(sample["sequence"]),
                "received_at": received_at.isoformat(),
                "dt_ms": dt_ms,
                "scene_type": scene,
                "scene_class": SCENE_CLASS[scene],
                "raw_image": str(raw_path),
                "mask_image": str(mask_path),
                "source_label_format": int(label.get("format_version", 0)),
            }
        )

    if errors:
        preview = "; ".join(
            f"{item['sample_id']}: {item['error']}" for item in errors[:10]
        )
        raise ValueError(f"export audit failed with {len(errors)} errors: {preview}")

    runs = build_temporal_runs(index_records, args.max_gap_ms)
    for run_id, run in enumerate(runs):
        for position, frame_index in enumerate(run):
            index_records[frame_index]["run_id"] = run_id
            index_records[frame_index]["position_in_run"] = position
    clips = build_clips(runs, index_records, args.clip_length)

    split_counts = Counter(record["split"] for record in index_records)
    warnings: list[str] = []
    if len(sessions) < 3:
        warnings.append(
            "fewer than three independent sessions; validation/test generalization "
            "cannot be measured reliably"
        )
    if scene_counts[SCENE_FINISH_T] < 50:
        warnings.append(
            f"only {scene_counts[SCENE_FINISH_T]} T-finish frames; collect at least "
            "50 independent positive examples"
        )
    if not split_counts["validation"] or not split_counts["test"]:
        warnings.append("validation and test splits are not both populated")

    audit = {
        "status": "PASS",
        "frame_count": frame_count,
        "scene_counts": dict(scene_counts),
        "source_label_format_counts": {
            str(key): value for key, value in sorted(format_counts.items())
        },
        "split_counts": dict(split_counts),
        "temporal_run_count": len(runs),
        "clip_count": len(clips),
        "path_length_mm": distribution(path_lengths),
        "segment_fit_rmse_mm": distribution(segment_fit_errors),
        "corner_abs_angle_deg": distribution(corner_angles),
        "finish_crossbar_length_mm": distribution(finish_lengths),
        "finish_stem_crossbar_angle_deg": distribution(finish_angles),
        "warnings": warnings,
        "errors": [],
    }

    output = (
        args.output.resolve()
        if args.output
        else (
            Path(__file__).resolve().parent
            / "camera-training-export"
            / datetime.now().strftime("export_%Y%m%d_%H%M%S")
        )
    )
    if output.exists():
        raise FileExistsError(f"output already exists: {output}")
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(
        prefix=f".{output.name}-", dir=output.parent
    ) as temporary_directory:
        temporary = Path(temporary_directory)
        frame_arrays: dict[str, np.ndarray] = {"mask": masks}
        if luma is not None:
            frame_arrays["luma"] = luma
        np.savez_compressed(temporary / "frames.npz", **frame_arrays)
        np.savez_compressed(
            temporary / "targets.npz",
            scene_class=scene_class,
            path_xy_mm=path_xy_mm,
            path_s_mm=path_s_mm,
            path_valid=path_valid,
            path_length_mm=path_length_mm,
            corner_xy_mm=corner_xy_mm,
            corner_s_mm=corner_s_mm,
            corner_angle_deg=corner_angle_deg,
            corner_direction=corner_direction,
            corner_valid=corner_valid,
            segment_rho_mm=segment_rho_mm,
            segment_direction_xy=segment_direction_xy,
            segment_fit_rmse_mm=segment_fit_rmse_mm,
            segment_valid=segment_valid,
            finish_junction_mm=finish_junction_mm,
            finish_crossbar_length_mm=finish_crossbar_length_mm,
            finish_stem_crossbar_angle_deg=finish_stem_crossbar_angle_deg,
            finish_valid=finish_valid,
        )
        with (temporary / "index.jsonl").open("w", encoding="utf-8") as target:
            for record in index_records:
                target.write(json.dumps(record) + "\n")
        with (temporary / "clips.jsonl").open("w", encoding="utf-8") as target:
            for clip in clips:
                target.write(json.dumps(clip) + "\n")
        (temporary / "audit.json").write_text(
            json.dumps(audit, indent=2), encoding="utf-8"
        )
        manifest = {
            "format_version": 2,
            "created_utc": datetime.now(timezone.utc).isoformat(),
            "default_model_input": "mask",
            "temporal_policy": (
                "single-frame perception; clips reference frames for output-level "
                "filtering and evaluation"
            ),
            "image": {
                "orientation": "vehicle_first_person",
                "roi_width": roi_width,
                "roi_height": roi_height,
                "channels": {
                    "mask": "uint8 0/1",
                    "luma": "uint8 0..255" if luma is not None else "not exported",
                },
            },
            "path_target": {
                "coordinate_system": "vehicle ground plane millimeters",
                "sampling": "fixed arc-length from nearest annotated point",
                "point_count": args.path_point_count,
                "step_mm": args.path_step_mm,
                "usage": "visualization and backward compatibility only",
            },
            "segment_target": {
                "capacity": segment_capacity,
                "fit": "robust total least squares between labeled corners",
                "representation": "rho_mm plus near-to-far unit direction_xy",
                "deployment_quantization": "int16 rho and int8 direction",
            },
            "corner_capacity": args.corner_capacity,
            "scene_classes": SCENE_CLASS,
            "clip": {
                "length": args.clip_length,
                "max_gap_ms": args.max_gap_ms,
                "images_duplicated": False,
            },
            "calibration": {
                "path": str(args.calibration.resolve()),
                "sha256": file_sha256(args.calibration.resolve()),
            },
            "sources": source_info,
            "files": {
                "frames": "frames.npz",
                "targets": "targets.npz",
                "index": "index.jsonl",
                "clips": "clips.jsonl",
                "audit": "audit.json",
            },
        }
        (temporary / "manifest.json").write_text(
            json.dumps(manifest, indent=2), encoding="utf-8"
        )
        temporary.replace(output)

    print(f"Exported {frame_count} frames to {output}")
    print(
        f"Scenes: path={scene_counts[SCENE_PATH]} "
        f"finish_t={scene_counts[SCENE_FINISH_T]} "
        f"invalid={scene_counts[SCENE_INVALID]}"
    )
    print(f"Temporal runs={len(runs)} clips={len(clips)}")
    for warning in warnings:
        print(f"WARNING: {warning}")
    return output


def run_self_test() -> None:
    points = [(0.0, 0.0), (100.0, 0.0)]
    output, distances, valid, length = resample_ground_path(points, 8, 25.0)
    if not math.isclose(length, 100.0) or int(valid.sum()) != 5:
        raise AssertionError("path resampling count failed")
    if not np.allclose(output[:5, 0], [0, 25, 50, 75, 100]):
        raise AssertionError("path resampling coordinates failed")
    if not np.allclose(distances[:5], [0, 25, 50, 75, 100]):
        raise AssertionError("path resampling distances failed")
    fitted = fit_ground_segments(
        [(10.0, 0.0), (10.5, 50.0), (9.5, 100.0)], [], 4
    )
    if len(fitted) != 1:
        raise AssertionError("line fitting segment count failed")
    if not math.isclose(float(fitted[0]["rho_mm"]), 10.0, abs_tol=0.6):
        raise AssertionError("line fitting rho failed")
    if not np.allclose(fitted[0]["direction_xy"], [0.0, 1.0], atol=0.02):
        raise AssertionError("line fitting direction failed")
    index = [
        {
            "frame_index": value,
            "session_id": "s",
            "sequence": sequence,
            "dt_ms": dt,
            "split": "development",
        }
        for value, (sequence, dt) in enumerate(
            [(1, 0), (2, 100), (3, 100), (7, 1200), (8, 100), (9, 100)]
        )
    ]
    runs = build_temporal_runs(index, 1000.0)
    if runs != [[0, 1, 2], [3, 4, 5]]:
        raise AssertionError(f"temporal run split failed: {runs}")
    clips = build_clips(runs, index, 3)
    if len(clips) != 2:
        raise AssertionError("temporal clip generation failed")
    print("camera_training_exporter self-test: PASS")


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(
        description="Export calibrated single-frame and temporal camera data."
    )
    parser.add_argument(
        "--dataset-root", type=Path, default=root / "camera-datasets"
    )
    parser.add_argument("--session", type=Path, action="append", default=[])
    parser.add_argument("--output", type=Path)
    parser.add_argument(
        "--calibration",
        type=Path,
        default=root / "camera_ground_calibration_pixel_lut.npz",
    )
    parser.add_argument("--validation-session", action="append", default=[])
    parser.add_argument("--test-session", action="append", default=[])
    parser.add_argument(
        "--path-point-count", type=int, default=DEFAULT_PATH_POINT_COUNT
    )
    parser.add_argument("--path-step-mm", type=float, default=DEFAULT_PATH_STEP_MM)
    parser.add_argument(
        "--corner-capacity", type=int, default=DEFAULT_CORNER_CAPACITY
    )
    parser.add_argument("--clip-length", type=int, default=DEFAULT_CLIP_LENGTH)
    parser.add_argument("--max-gap-ms", type=float, default=DEFAULT_MAX_GAP_MS)
    parser.add_argument("--without-luma", action="store_true")
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.path_point_count < 2:
        parser.error("--path-point-count must be at least 2")
    if args.path_step_mm <= 0.0:
        parser.error("--path-step-mm must be positive")
    if args.corner_capacity < 1:
        parser.error("--corner-capacity must be positive")
    if args.clip_length < 2:
        parser.error("--clip-length must be at least 2")
    if args.max_gap_ms <= 0.0:
        parser.error("--max-gap-ms must be positive")
    return args


def main() -> int:
    args = parse_args()
    if args.self_test:
        run_self_test()
        return 0
    export_dataset(args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
