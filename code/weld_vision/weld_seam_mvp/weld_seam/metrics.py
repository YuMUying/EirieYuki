from __future__ import annotations

import torch
from torch import nn


class DiceBCELoss(nn.Module):
    def __init__(self, dice_weight: float = 1.0, bce_weight: float = 1.0) -> None:
        super().__init__()
        self.dice_weight = dice_weight
        self.bce_weight = bce_weight
        self.bce = nn.BCEWithLogitsLoss()

    def forward(self, logits: torch.Tensor, targets: torch.Tensor) -> torch.Tensor:
        probabilities = torch.sigmoid(logits)
        dims = (1, 2, 3)
        intersection = (probabilities * targets).sum(dims)
        denominator = probabilities.sum(dims) + targets.sum(dims)
        dice_loss = 1.0 - ((2.0 * intersection + 1.0) / (denominator + 1.0)).mean()
        return self.bce_weight * self.bce(logits, targets) + self.dice_weight * dice_loss


class BinarySegmentationMeter:
    def __init__(self, threshold: float = 0.5) -> None:
        self.threshold = threshold
        self.reset()

    def reset(self) -> None:
        self.intersection = 0.0
        self.predicted = 0.0
        self.target = 0.0
        self.union = 0.0
        self.correct = 0.0
        self.total = 0.0

    @torch.no_grad()
    def update(self, logits: torch.Tensor, targets: torch.Tensor) -> None:
        predictions = torch.sigmoid(logits) >= self.threshold
        targets_bool = targets >= 0.5
        self.intersection += torch.logical_and(predictions, targets_bool).sum().item()
        self.predicted += predictions.sum().item()
        self.target += targets_bool.sum().item()
        self.union += torch.logical_or(predictions, targets_bool).sum().item()
        self.correct += (predictions == targets_bool).sum().item()
        self.total += targets_bool.numel()

    def compute(self) -> dict[str, float]:
        epsilon = 1e-8
        return {
            "dice": (2.0 * self.intersection + epsilon)
            / (self.predicted + self.target + epsilon),
            "iou": (self.intersection + epsilon) / (self.union + epsilon),
            "pixel_accuracy": self.correct / max(self.total, 1.0),
        }
