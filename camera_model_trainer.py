"""Train a compact multi-task camera perception model for ESP32 deployment."""

from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime
import json
from pathlib import Path
import random

import numpy as np
import torch
from torch import nn
from torch.nn import functional as functional


SCENE_NAMES = ("path", "finish_t", "invalid")
CORNER_TYPE_NAMES = ("left_60", "left_90", "right_60", "right_90")
SEGMENT_RHO_SCALE = 300.0
CORNER_SCALE = torch.tensor([300.0, 700.0])
CORNER_T_SCALE = 500.0
FINISH_SCALE = torch.tensor([300.0, 400.0, 120.0, 90.0])


class CompactPerceptionModel(nn.Module):
    def __init__(self) -> None:
        super().__init__()
        self.trunk = nn.Sequential(
            nn.Linear(27 * 27, 48),
            nn.ReLU(),
            nn.Linear(48, 24),
            nn.ReLU(),
        )
        self.scene = nn.Linear(24, 3)
        self.segment_valid = nn.Linear(24, 1)
        self.segment_rho = nn.Linear(24, 1)
        self.segment_direction = nn.Linear(24, 2)
        self.corner_valid = nn.Linear(24, 1)
        self.corner_t = nn.Linear(24, 1)
        self.corner_type = nn.Linear(24, 4)
        self.finish_geometry = nn.Linear(24, 4)

    def forward(self, features: torch.Tensor) -> dict[str, torch.Tensor]:
        hidden = self.trunk(features)
        return {
            "scene": self.scene(hidden),
            "segment_valid": self.segment_valid(hidden),
            "segment_rho": self.segment_rho(hidden),
            "segment_direction": self.segment_direction(hidden).reshape(-1, 1, 2),
            "corner_valid": self.corner_valid(hidden),
            "corner_t": self.corner_t(hidden),
            "corner_type": self.corner_type(hidden).reshape(-1, 1, 4),
            "finish_geometry": self.finish_geometry(hidden),
        }


class TinyCnnPerceptionModel(nn.Module):
    def __init__(self) -> None:
        super().__init__()
        self.features = nn.Sequential(
            nn.Conv2d(1, 8, kernel_size=3, padding=1),
            nn.ReLU(),
            nn.MaxPool2d(2),
            nn.Conv2d(8, 12, kernel_size=3, padding=1),
            nn.ReLU(),
            nn.MaxPool2d(2),
        )
        self.trunk = nn.Sequential(
            nn.Linear(12 * 6 * 6, 32),
            nn.ReLU(),
            nn.Dropout(p=0.15),
            nn.Linear(32, 24),
            nn.ReLU(),
        )
        self.scene = nn.Linear(24, 3)
        self.segment_valid = nn.Linear(24, 1)
        self.segment_rho = nn.Linear(24, 1)
        self.segment_direction = nn.Linear(24, 2)
        self.corner_valid = nn.Linear(24, 1)
        self.corner_t = nn.Linear(24, 1)
        self.corner_type = nn.Linear(24, 4)
        self.finish_geometry = nn.Linear(24, 4)

    def forward(self, features: torch.Tensor) -> dict[str, torch.Tensor]:
        image = features.reshape(-1, 1, 27, 27)
        hidden = self.trunk(self.features(image).flatten(start_dim=1))
        return {
            "scene": self.scene(hidden),
            "segment_valid": self.segment_valid(hidden),
            "segment_rho": self.segment_rho(hidden),
            "segment_direction": self.segment_direction(hidden).reshape(-1, 1, 2),
            "corner_valid": self.corner_valid(hidden),
            "corner_t": self.corner_t(hidden),
            "corner_type": self.corner_type(hidden).reshape(-1, 1, 4),
            "finish_geometry": self.finish_geometry(hidden),
        }


def build_model(architecture: str) -> nn.Module:
    if architecture == "pooled_mlp":
        return CompactPerceptionModel()
    if architecture == "tiny_cnn":
        return TinyCnnPerceptionModel()
    raise ValueError(f"unsupported architecture: {architecture}")


def decode_corner_xy(
    output: dict[str, torch.Tensor],
) -> torch.Tensor:
    """Reconstruct a corner constrained to the predicted current line."""
    direction = functional.normalize(output["segment_direction"], dim=-1)
    normal = torch.stack(
        (direction[..., 1], -direction[..., 0]), dim=-1
    )
    closest = (
        output["segment_rho"] * SEGMENT_RHO_SCALE
    ).unsqueeze(-1) * normal
    along_line = (output["corner_t"] * CORNER_T_SCALE).unsqueeze(-1)
    return closest + along_line * direction


def read_jsonl(path: Path) -> list[dict[str, object]]:
    return [
        json.loads(line)
        for line in path.read_text(encoding="utf-8").splitlines()
    ]


def pooled_mask(mask: np.ndarray) -> np.ndarray:
    if mask.ndim != 4 or mask.shape[1:] != (1, 108, 106):
        raise ValueError(f"expected mask [N,1,108,106], got {mask.shape}")
    line_pixels = 1.0 - mask.astype(np.float32)
    padded = np.pad(
        line_pixels, ((0, 0), (0, 0), (0, 0), (1, 1))
    )
    return (
        padded.reshape(-1, 1, 27, 4, 27, 4)
        .mean(axis=(3, 5))
        .reshape(-1, 729)
    )


def load_dataset(
    dataset: Path,
) -> tuple[dict[str, torch.Tensor], list[str]]:
    with np.load(dataset / "frames.npz") as frames:
        features = pooled_mask(frames["mask"])
    with np.load(dataset / "targets.npz") as targets:
        arrays = {name: targets[name].copy() for name in targets.files}
    index = read_jsonl(dataset / "index.jsonl")
    if len(index) != features.shape[0]:
        raise ValueError("index and frame counts differ")
    splits = [str(record["split"]) for record in index]
    tensors = {
        "features": torch.from_numpy(features),
        "scene": torch.from_numpy(arrays["scene_class"].astype(np.int64)),
        "segment_rho": torch.from_numpy(arrays["segment_rho_mm"][:, :1]),
        "segment_direction": torch.from_numpy(
            arrays["segment_direction_xy"][:, :1]
        ),
        "segment_valid": torch.from_numpy(
            arrays["segment_valid"][:, :1].astype(np.float32)
        ),
        "corner_xy": torch.from_numpy(arrays["corner_xy_mm"][:, :1]),
        "corner_t": torch.from_numpy(
            np.sum(
                arrays["corner_xy_mm"][:, :1]
                * arrays["segment_direction_xy"][:, :1],
                axis=-1,
            ).astype(np.float32)
        ),
        "corner_type": torch.from_numpy(
            (
                (arrays["corner_angle_deg"][:, :1] > 0.0).astype(np.int64) * 2
                + (
                    np.abs(np.abs(arrays["corner_angle_deg"][:, :1]) - 90.0)
                    < np.abs(np.abs(arrays["corner_angle_deg"][:, :1]) - 60.0)
                ).astype(np.int64)
            )
        ),
        "corner_valid": torch.from_numpy(
            arrays["corner_valid"][:, :1].astype(np.float32)
        ),
        "finish_geometry": torch.from_numpy(
            np.column_stack(
                (
                    arrays["finish_junction_mm"],
                    arrays["finish_crossbar_length_mm"],
                    arrays["finish_stem_crossbar_angle_deg"],
                )
            ).astype(np.float32)
        ),
        "finish_valid": torch.from_numpy(
            arrays["finish_valid"].astype(np.float32)
        ),
    }
    return tensors, splits


def masked_smooth_l1(
    prediction: torch.Tensor,
    target: torch.Tensor,
    valid: torch.Tensor,
) -> torch.Tensor:
    while valid.ndim < prediction.ndim:
        valid = valid.unsqueeze(-1)
    valid = valid.expand_as(prediction)
    if not torch.any(valid > 0):
        return prediction.sum() * 0.0
    return functional.smooth_l1_loss(
        prediction[valid > 0], target[valid > 0], beta=0.1
    )


def model_loss(
    output: dict[str, torch.Tensor],
    batch: dict[str, torch.Tensor],
    class_weights: torch.Tensor,
    corner_type_weights: torch.Tensor,
    corner_valid_pos_weight: torch.Tensor,
) -> tuple[torch.Tensor, dict[str, float]]:
    corner_scale = CORNER_SCALE.to(batch["features"].device)
    finish_scale = FINISH_SCALE.to(batch["features"].device)
    predicted_corner_xy = decode_corner_xy(output)
    losses = {
        "scene": functional.cross_entropy(
            output["scene"], batch["scene"], weight=class_weights
        ),
        "segment_valid": functional.binary_cross_entropy_with_logits(
            output["segment_valid"], batch["segment_valid"]
        ),
        "segment_rho": masked_smooth_l1(
            output["segment_rho"],
            batch["segment_rho"] / SEGMENT_RHO_SCALE,
            batch["segment_valid"],
        ),
        "segment_direction": masked_smooth_l1(
            output["segment_direction"],
            batch["segment_direction"],
            batch["segment_valid"],
        ),
        "corner_valid": functional.binary_cross_entropy_with_logits(
            output["corner_valid"],
            batch["corner_valid"],
            pos_weight=corner_valid_pos_weight,
        ),
        "corner_t": masked_smooth_l1(
            output["corner_t"],
            batch["corner_t"] / CORNER_T_SCALE,
            batch["corner_valid"],
        ),
        "corner_geometry": masked_smooth_l1(
            predicted_corner_xy / corner_scale,
            batch["corner_xy"] / corner_scale,
            batch["corner_valid"],
        ),
        "finish_geometry": masked_smooth_l1(
            output["finish_geometry"],
            batch["finish_geometry"] / finish_scale,
            batch["finish_valid"],
        ),
    }
    total = (
        losses["scene"]
        + 0.4 * losses["segment_valid"]
        + 0.8 * losses["segment_rho"]
        + 0.8 * losses["segment_direction"]
        + 0.4 * losses["corner_valid"]
        + 0.3 * losses["corner_t"]
        + 0.4 * losses["corner_geometry"]
        + 0.7 * losses["finish_geometry"]
    )
    corner_valid = batch["corner_valid"].bool()
    if torch.any(corner_valid):
        corner_type_loss = functional.cross_entropy(
            output["corner_type"][corner_valid],
            batch["corner_type"][corner_valid],
            weight=corner_type_weights,
        )
        total = total + corner_type_loss
        losses["corner_type"] = corner_type_loss
    return total, {
        name: float(value.detach()) for name, value in losses.items()
    }


def subset(
    data: dict[str, torch.Tensor], indices: torch.Tensor
) -> dict[str, torch.Tensor]:
    return {name: value[indices] for name, value in data.items()}


@torch.no_grad()
def metrics(
    model: nn.Module,
    data: dict[str, torch.Tensor],
    indices: torch.Tensor,
) -> dict[str, object]:
    if indices.numel() == 0:
        return {"frame_count": 0}
    batch = subset(data, indices)
    output = model(batch["features"])
    predicted_scene = output["scene"].argmax(dim=1)
    scene = batch["scene"]
    result: dict[str, object] = {
        "frame_count": int(indices.numel()),
        "scene_accuracy": float(
            (predicted_scene == scene).float().mean()
        ),
        "scene_confusion": torch.bincount(
            scene * 3 + predicted_scene, minlength=9
        ).reshape(3, 3).tolist(),
    }
    true_t = scene == 1
    predicted_t = predicted_scene == 1
    true_positive = int(torch.sum(true_t & predicted_t))
    false_positive = int(torch.sum(~true_t & predicted_t))
    false_negative = int(torch.sum(true_t & ~predicted_t))
    result["finish_t_precision"] = true_positive / max(
        1, true_positive + false_positive
    )
    result["finish_t_recall"] = true_positive / max(
        1, true_positive + false_negative
    )
    segment_valid = batch["segment_valid"].bool()
    if torch.any(segment_valid):
        predicted_rho = output["segment_rho"] * SEGMENT_RHO_SCALE
        result["segment_rho_mae_mm"] = float(
            torch.abs(
                predicted_rho[segment_valid] - batch["segment_rho"][segment_valid]
            ).mean()
        )
        predicted_direction = functional.normalize(
            output["segment_direction"], dim=-1
        )
        direction_dot = torch.sum(
            predicted_direction[segment_valid]
            * batch["segment_direction"][segment_valid],
            dim=-1,
        ).clamp(-1.0, 1.0)
        result["segment_heading_mae_deg"] = float(
            torch.rad2deg(torch.acos(direction_dot)).mean()
        )
    corner_valid = batch["corner_valid"].bool()
    predicted_corner_valid = output["corner_valid"].sigmoid() >= 0.5
    corner_true_positive = int(
        torch.sum(predicted_corner_valid & corner_valid)
    )
    corner_false_positive = int(
        torch.sum(predicted_corner_valid & ~corner_valid)
    )
    corner_false_negative = int(
        torch.sum(~predicted_corner_valid & corner_valid)
    )
    result["corner_detection_precision"] = corner_true_positive / max(
        1, corner_true_positive + corner_false_positive
    )
    result["corner_detection_recall"] = corner_true_positive / max(
        1, corner_true_positive + corner_false_negative
    )
    if torch.any(corner_valid):
        predicted_corner_type = output["corner_type"].argmax(dim=-1)
        result["corner_type_accuracy"] = float(
            (
                predicted_corner_type[corner_valid]
                == batch["corner_type"][corner_valid]
            ).float().mean()
        )
        result["corner_type_confusion"] = torch.bincount(
            batch["corner_type"][corner_valid] * 4
            + predicted_corner_type[corner_valid],
            minlength=16,
        ).reshape(4, 4).tolist()
        corner_delta = decode_corner_xy(output) - batch["corner_xy"]
        result["corner_position_mae_mm"] = float(
            torch.linalg.vector_norm(corner_delta[corner_valid], dim=-1).mean()
        )
    finish_valid = batch["finish_valid"].bool()
    if torch.any(finish_valid):
        predicted_finish = output["finish_geometry"] * FINISH_SCALE
        finish_error = torch.abs(
            predicted_finish[finish_valid]
            - batch["finish_geometry"][finish_valid]
        )
        result["finish_junction_mae_mm"] = float(
            torch.linalg.vector_norm(finish_error[:, :2], dim=-1).mean()
        )
        result["finish_crossbar_length_mae_mm"] = float(
            finish_error[:, 2].mean()
        )
    return result


@torch.no_grad()
def calibrate_finish_threshold(
    model: nn.Module,
    data: dict[str, torch.Tensor],
    indices: torch.Tensor,
    minimum_recall: float = 0.75,
) -> dict[str, float | int] | None:
    if indices.numel() == 0:
        return None
    batch = subset(data, indices)
    labels = batch["scene"] == 1
    if not torch.any(labels):
        return None
    probabilities = model(batch["features"])["scene"].softmax(dim=1)[:, 1]
    candidates: list[dict[str, float | int]] = []
    for threshold in np.arange(0.05, 0.951, 0.05):
        prediction = probabilities >= float(threshold)
        true_positive = int(torch.sum(prediction & labels))
        false_positive = int(torch.sum(prediction & ~labels))
        false_negative = int(torch.sum(~prediction & labels))
        precision = true_positive / max(1, true_positive + false_positive)
        recall = true_positive / max(1, true_positive + false_negative)
        candidates.append(
            {
                "threshold": round(float(threshold), 2),
                "precision": precision,
                "recall": recall,
                "true_positive": true_positive,
                "false_positive": false_positive,
                "false_negative": false_negative,
            }
        )
    eligible = [
        candidate
        for candidate in candidates
        if float(candidate["recall"]) >= minimum_recall
    ]
    pool = eligible or candidates
    return max(
        pool,
        key=lambda candidate: (
            float(candidate["precision"]),
            float(candidate["recall"]),
            -float(candidate["threshold"]),
        ),
    )


def save_model(
    model: nn.Module, output: Path, metadata: dict[str, object]
) -> None:
    output.mkdir(parents=True)
    torch.save(model.state_dict(), output / "model.pt")
    weights = {
        name.replace(".", "__"): value.detach().cpu().numpy()
        for name, value in model.state_dict().items()
    }
    np.savez_compressed(output / "model_weights.npz", **weights)
    (output / "model_config.json").write_text(
        json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8"
    )


def train(args: argparse.Namespace) -> Path:
    random.seed(args.seed)
    np.random.seed(args.seed)
    torch.manual_seed(args.seed)
    torch.set_num_threads(max(1, args.threads))
    data, splits = load_dataset(args.dataset)
    development = torch.tensor(
        [
            index
            for index, split_name in enumerate(splits)
            if split_name == "development"
        ],
        dtype=torch.long,
    )
    validation = torch.tensor(
        [
            index
            for index, split_name in enumerate(splits)
            if split_name == "validation"
        ],
        dtype=torch.long,
    )
    test_count = sum(split_name == "test" for split_name in splits)
    if development.numel() == 0:
        raise ValueError("dataset has no development frames")
    scene_counts = torch.bincount(
        data["scene"][development], minlength=3
    ).float()
    class_weights = scene_counts.sum() / (
        3.0 * scene_counts.clamp_min(1.0)
    )
    development_corner_valid = data["corner_valid"][development].bool()
    corner_type_counts = torch.bincount(
        data["corner_type"][development][development_corner_valid],
        minlength=4,
    ).float()
    corner_type_weights = torch.sqrt(
        corner_type_counts.sum() / corner_type_counts.clamp_min(5.0)
    )
    corner_type_weights /= corner_type_weights.mean()
    corner_positive_count = development_corner_valid.sum().float()
    corner_negative_count = development_corner_valid.numel() - corner_positive_count
    corner_valid_pos_weight = (
        corner_negative_count / corner_positive_count.clamp_min(1.0)
    ).reshape(1)
    model = build_model(args.architecture)
    optimizer = torch.optim.AdamW(
        model.parameters(),
        lr=args.learning_rate,
        weight_decay=args.weight_decay,
    )
    train_batch = subset(data, development)
    best_state = None
    best_score = float("inf")
    best_epoch = 0
    history: list[dict[str, object]] = []
    for epoch in range(1, args.epochs + 1):
        model.train()
        optimizer.zero_grad(set_to_none=True)
        output = model(train_batch["features"])
        loss, parts = model_loss(
            output,
            train_batch,
            class_weights,
            corner_type_weights,
            corner_valid_pos_weight,
        )
        loss.backward()
        torch.nn.utils.clip_grad_norm_(model.parameters(), 5.0)
        optimizer.step()
        score = float(loss.detach())
        if validation.numel():
            model.eval()
            validation_batch = subset(data, validation)
            with torch.no_grad():
                validation_output = model(validation_batch["features"])
                validation_loss, _ = model_loss(
                    validation_output,
                    validation_batch,
                    class_weights,
                    corner_type_weights,
                    corner_valid_pos_weight,
                )
            score = float(validation_loss)
        if score < best_score:
            best_score = score
            best_epoch = epoch
            best_state = {
                name: value.detach().clone()
                for name, value in model.state_dict().items()
            }
        if (
            epoch == 1
            or epoch % args.log_every == 0
            or epoch == args.epochs
        ):
            item = {
                "epoch": epoch,
                "train_loss": float(loss.detach()),
                **parts,
            }
            if validation.numel():
                item["validation_loss"] = score
            history.append(item)
            print(
                f"epoch={epoch:4d} train_loss={float(loss.detach()):.5f}"
                + (f" val_loss={score:.5f}" if validation.numel() else "")
            )
    if best_state is not None:
        model.load_state_dict(best_state)
    model.eval()
    parameter_count = sum(
        parameter.numel() for parameter in model.parameters()
    )
    report = {
        "format_version": 1,
        "dataset": str(args.dataset.resolve()),
        "seed": args.seed,
        "epochs": args.epochs,
        "best_epoch": best_epoch,
        "best_validation_loss": best_score if validation.numel() else None,
        "architecture": args.architecture,
        "selected_by": (
            "validation_loss" if validation.numel() else "training_loss"
        ),
        "scene_names": SCENE_NAMES,
        "corner_type_names": CORNER_TYPE_NAMES,
        "split_counts": dict(Counter(splits)),
        "test_frames_ignored_during_training": test_count,
        "parameter_count": parameter_count,
        "estimated_int8_parameter_bytes": parameter_count,
        "input": {
            "source": "binary mask [1,108,106], inverted to black-line=1",
            "preprocessing": (
                "pad one zero column on each side; 4x4 average pool"
            ),
            "shape": [1, 27, 27],
        },
        "control_policy": (
            "predict the current fitted line and the nearest corner's signed "
            "coordinate along that line; reconstruct the corner on the line "
            "and re-evaluate after each maneuver"
        ),
        "normalization": {
            "segment_rho_mm": SEGMENT_RHO_SCALE,
            "segment_direction_xy": "unit vector",
            "corner_t_mm": CORNER_T_SCALE,
            "corner_xy_mm": (
                "derived from segment_rho, segment_direction, and corner_t"
            ),
            "corner_type": CORNER_TYPE_NAMES,
            "finish_geometry": FINISH_SCALE.tolist(),
        },
        "development_metrics": metrics(model, data, development),
        "validation_metrics": metrics(model, data, validation),
        "finish_t_calibration": calibrate_finish_threshold(
            model, data, validation
        ),
        "history": history,
    }
    output_path = args.output or (
        Path(__file__).resolve().parent
        / "camera-models"
        / datetime.now().strftime("model_%Y%m%d_%H%M%S")
    )
    output_path = output_path.resolve()
    if output_path.exists():
        raise FileExistsError(f"output already exists: {output_path}")
    save_model(model, output_path, report)
    print(f"Saved model to {output_path}")
    print(
        json.dumps(
            report["development_metrics"],
            ensure_ascii=False,
            indent=2,
        )
    )
    if validation.numel():
        print("Validation metrics:")
        print(
            json.dumps(
                report["validation_metrics"],
                ensure_ascii=False,
                indent=2,
            )
        )
    if not validation.numel():
        print(
            "WARNING: provisional fit only; validation labels are not "
            "available yet"
        )
    return output_path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    parser.add_argument(
        "--architecture",
        choices=("pooled_mlp", "tiny_cnn"),
        default="tiny_cnn",
    )
    parser.add_argument("--epochs", type=int, default=600)
    parser.add_argument("--learning-rate", type=float, default=0.003)
    parser.add_argument("--weight-decay", type=float, default=0.0001)
    parser.add_argument("--seed", type=int, default=20260901)
    parser.add_argument("--threads", type=int, default=4)
    parser.add_argument("--log-every", type=int, default=50)
    args = parser.parse_args()
    if args.epochs < 1 or args.log_every < 1 or args.threads < 1:
        parser.error("epochs, log-every and threads must be positive")
    return args


if __name__ == "__main__":
    train(parse_args())
