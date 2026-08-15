from __future__ import annotations

from pathlib import Path
from typing import Any

import torch

from .model import build_model


def save_checkpoint(
    path: str | Path,
    model: torch.nn.Module,
    optimizer: torch.optim.Optimizer,
    epoch: int,
    best_iou: float,
    image_size: int,
    base_channels: int,
    metrics: dict[str, float],
) -> None:
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    torch.save(
        {
            "format_version": 1,
            "model_state": model.state_dict(),
            "optimizer_state": optimizer.state_dict(),
            "epoch": epoch,
            "best_iou": best_iou,
            "image_size": image_size,
            "base_channels": base_channels,
            "metrics": metrics,
        },
        destination,
    )


def load_checkpoint(
    path: str | Path, device: torch.device | str = "cpu"
) -> tuple[torch.nn.Module, dict[str, Any]]:
    checkpoint = torch.load(path, map_location=device, weights_only=False)
    base_channels = int(checkpoint.get("base_channels", 16))
    model = build_model(base_channels=base_channels)
    model.load_state_dict(checkpoint["model_state"])
    model.to(device)
    model.eval()
    return model, checkpoint
