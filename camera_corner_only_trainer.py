"""Train a focused corner-position model for a fixed course."""

from __future__ import annotations

import argparse
from datetime import datetime
import json
from pathlib import Path
import random

import numpy as np
import torch
from torch import nn
from torch.nn import functional as functional

from camera_model_trainer import (
    CORNER_SCALE,
    load_dataset,
    subset,
)


# Positive values turn right; negative values turn left.
COURSE_TURNS_DEG = (120, -30, -90, -90, 90)


class CornerPositionModel(nn.Module):
    """Use the same feature capacity as the multi-task tiny CNN."""

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
        self.corner_xy = nn.Linear(24, 2)

    def forward(self, features: torch.Tensor) -> torch.Tensor:
        image = features.reshape(-1, 1, 27, 27)
        hidden = self.trunk(self.features(image).flatten(start_dim=1))
        return self.corner_xy(hidden)


def corner_indices(
    data: dict[str, torch.Tensor], splits: list[str], split_name: str
) -> torch.Tensor:
    return torch.tensor(
        [
            index
            for index, name in enumerate(splits)
            if name == split_name and bool(data["corner_valid"][index, 0])
        ],
        dtype=torch.long,
    )


def focused_batch(
    data: dict[str, torch.Tensor], indices: torch.Tensor
) -> dict[str, torch.Tensor]:
    selected = subset(data, indices)
    return {
        "features": selected["features"],
        "corner_xy": selected["corner_xy"][:, 0],
    }


def model_loss(
    output: torch.Tensor, batch: dict[str, torch.Tensor]
) -> torch.Tensor:
    scale = CORNER_SCALE.to(batch["features"].device)
    return functional.smooth_l1_loss(
        output, batch["corner_xy"] / scale, beta=0.1
    )


@torch.no_grad()
def metrics(
    model: CornerPositionModel, batch: dict[str, torch.Tensor]
) -> dict[str, object]:
    model.eval()
    predicted_xy = model(batch["features"]) * CORNER_SCALE
    delta = predicted_xy - batch["corner_xy"]
    return {
        "frame_count": int(batch["features"].shape[0]),
        "corner_position_mae_mm": float(
            torch.linalg.vector_norm(delta, dim=1).mean()
        ),
        "corner_x_mae_mm": float(torch.abs(delta[:, 0]).mean()),
        "corner_y_mae_mm": float(torch.abs(delta[:, 1]).mean()),
    }


def train(args: argparse.Namespace) -> Path:
    random.seed(args.seed)
    np.random.seed(args.seed)
    torch.manual_seed(args.seed)
    torch.set_num_threads(max(1, args.threads))

    data, splits = load_dataset(args.dataset)
    development_indices = corner_indices(data, splits, "development")
    validation_indices = corner_indices(data, splits, "validation")
    if development_indices.numel() == 0 or validation_indices.numel() == 0:
        raise ValueError("corner-only training needs development and validation corners")
    development = focused_batch(data, development_indices)
    validation = focused_batch(data, validation_indices)

    model = CornerPositionModel()
    optimizer = torch.optim.AdamW(
        model.parameters(),
        lr=args.learning_rate,
        weight_decay=args.weight_decay,
    )
    best_state: dict[str, torch.Tensor] | None = None
    best_validation_loss = float("inf")
    best_epoch = 0
    history: list[dict[str, float | int]] = []

    for epoch in range(1, args.epochs + 1):
        model.train()
        optimizer.zero_grad(set_to_none=True)
        output = model(development["features"])
        training_loss = model_loss(output, development)
        training_loss.backward()
        torch.nn.utils.clip_grad_norm_(model.parameters(), 5.0)
        optimizer.step()

        model.eval()
        with torch.no_grad():
            validation_loss = model_loss(model(validation["features"]), validation)
        score = float(validation_loss)
        if score < best_validation_loss:
            best_validation_loss = score
            best_epoch = epoch
            best_state = {
                name: value.detach().clone()
                for name, value in model.state_dict().items()
            }
        if epoch == 1 or epoch % args.log_every == 0 or epoch == args.epochs:
            item: dict[str, float | int] = {
                "epoch": epoch,
                "training_loss": float(training_loss.detach()),
                "validation_loss": score,
            }
            history.append(item)
            print(
                f"epoch={epoch:4d} train_loss={item['training_loss']:.5f} "
                f"val_loss={score:.5f}"
            )

    if best_state is None:
        raise RuntimeError("training did not produce a checkpoint")
    model.load_state_dict(best_state)
    model.eval()

    report: dict[str, object] = {
        "format_version": 1,
        "task": "corner_position_only",
        "runtime_gate": (
            "line_vision must establish that a corner exists before this "
            "model output is used"
        ),
        "course_turns_deg": COURSE_TURNS_DEG,
        "course_turn_sign_convention": "positive=right, negative=left",
        "course_state_policy": (
            "advance the course index only after the commanded turn completes "
            "and the outgoing line is reacquired"
        ),
        "dataset": str(args.dataset.resolve()),
        "seed": args.seed,
        "epochs": args.epochs,
        "best_epoch": best_epoch,
        "best_validation_loss": best_validation_loss,
        "input_shape": [1, 27, 27],
        "normalization": {"corner_xy_mm": CORNER_SCALE.tolist()},
        "parameter_count": sum(
            parameter.numel() for parameter in model.parameters()
        ),
        "development_metrics": metrics(model, development),
        "validation_metrics": metrics(model, validation),
        "history": history,
    }

    output_path = (
        args.output
        or Path(__file__).resolve().parent
        / "camera-models"
        / datetime.now().strftime("model_corner_only_%Y%m%d_%H%M%S")
    ).resolve()
    if output_path.exists():
        raise FileExistsError(f"output already exists: {output_path}")
    output_path.mkdir(parents=True)
    torch.save(model.state_dict(), output_path / "model.pt")
    np.savez_compressed(
        output_path / "model_weights.npz",
        **{
            name.replace(".", "__"): value.detach().cpu().numpy()
            for name, value in model.state_dict().items()
        },
    )
    (output_path / "model_config.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(f"Saved model to {output_path}")
    print("Validation metrics:")
    print(json.dumps(report["validation_metrics"], ensure_ascii=False, indent=2))
    return output_path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--epochs", type=int, default=600)
    parser.add_argument("--learning-rate", type=float, default=0.003)
    parser.add_argument("--weight-decay", type=float, default=0.0001)
    parser.add_argument("--seed", type=int, default=20260901)
    parser.add_argument("--threads", type=int, default=4)
    parser.add_argument("--log-every", type=int, default=50)
    args = parser.parse_args()
    if args.epochs < 1 or args.threads < 1 or args.log_every < 1:
        parser.error("epochs, threads, and log-every must be positive")
    return args


if __name__ == "__main__":
    train(parse_args())
