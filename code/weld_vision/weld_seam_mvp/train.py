from __future__ import annotations

import argparse
import json
import random
import time
from pathlib import Path

import numpy as np
import torch
from torch.utils.data import DataLoader, Subset

from weld_seam.checkpoint import save_checkpoint
from weld_seam.dataset import WeldSegmentationDataset
from weld_seam.metrics import BinarySegmentationMeter, DiceBCELoss
from weld_seam.model import build_model
from project_paths import WELD_DATASET, WELD_RESULTS_DIR


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Train the weld-area segmentation model.")
    parser.add_argument("--data", type=Path, default=WELD_DATASET, help="WES-Combined-Dataset root")
    parser.add_argument(
        "--output", type=Path, default=WELD_RESULTS_DIR / "training" / "baseline"
    )
    parser.add_argument("--epochs", type=int, default=30)
    parser.add_argument("--image-size", type=int, default=192)
    parser.add_argument("--batch-size", type=int, default=8)
    parser.add_argument("--base-channels", type=int, default=8, choices=(8, 16, 24, 32))
    parser.add_argument("--learning-rate", type=float, default=1e-3)
    parser.add_argument("--weight-decay", type=float, default=1e-4)
    parser.add_argument("--workers", type=int, default=2)
    parser.add_argument("--device", default="auto", choices=("auto", "cpu", "cuda"))
    parser.add_argument("--seed", type=int, default=20260813)
    parser.add_argument("--patience", type=int, default=6)
    parser.add_argument("--min-delta", type=float, default=1e-4)
    parser.add_argument("--original-only", action="store_true")
    parser.add_argument("--limit-train", type=int, default=0, help="Debug: use first N samples")
    parser.add_argument("--limit-val", type=int, default=0, help="Debug: use first N samples")
    return parser.parse_args()


def choose_device(name: str) -> torch.device:
    if name == "cuda" and not torch.cuda.is_available():
        raise RuntimeError("CUDA requested but is not available")
    if name == "auto":
        name = "cuda" if torch.cuda.is_available() else "cpu"
    return torch.device(name)


def seed_everything(seed: int) -> None:
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)


def limited(dataset, count: int):
    return Subset(dataset, range(min(count, len(dataset)))) if count > 0 else dataset


def run_epoch(
    model: torch.nn.Module,
    loader: DataLoader,
    criterion: DiceBCELoss,
    device: torch.device,
    optimizer: torch.optim.Optimizer | None,
) -> dict[str, float]:
    training = optimizer is not None
    model.train(training)
    meter = BinarySegmentationMeter()
    loss_total = 0.0
    sample_count = 0
    for images, masks, _ in loader:
        images = images.to(device, non_blocking=True)
        masks = masks.to(device, non_blocking=True)
        if training:
            optimizer.zero_grad(set_to_none=True)
        with torch.set_grad_enabled(training):
            logits = model(images)
            loss = criterion(logits, masks)
            if training:
                loss.backward()
                optimizer.step()
        batch_count = images.shape[0]
        loss_total += float(loss.item()) * batch_count
        sample_count += batch_count
        meter.update(logits.detach(), masks)
    metrics = meter.compute()
    metrics["loss"] = loss_total / max(sample_count, 1)
    return metrics


def main() -> None:
    args = parse_args()
    if args.epochs < 1 or args.image_size < 64 or args.batch_size < 1:
        raise ValueError("epochs and batch-size must be positive; image-size must be >= 64")
    seed_everything(args.seed)
    device = choose_device(args.device)
    args.output.mkdir(parents=True, exist_ok=True)

    train_dataset = limited(
        WeldSegmentationDataset(
            args.data,
            "train",
            args.image_size,
            augment=True,
            original_only=args.original_only,
        ),
        args.limit_train,
    )
    val_dataset = limited(
        WeldSegmentationDataset(args.data, "val", args.image_size), args.limit_val
    )
    loader_options = {
        "batch_size": args.batch_size,
        "num_workers": args.workers,
        "pin_memory": device.type == "cuda",
        "persistent_workers": args.workers > 0,
    }
    train_loader = DataLoader(train_dataset, shuffle=True, drop_last=False, **loader_options)
    val_loader = DataLoader(val_dataset, shuffle=False, drop_last=False, **loader_options)

    model = build_model(args.base_channels).to(device)
    optimizer = torch.optim.AdamW(
        model.parameters(), lr=args.learning_rate, weight_decay=args.weight_decay
    )
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(
        optimizer, T_max=max(args.epochs, 1), eta_min=args.learning_rate * 0.05
    )
    criterion = DiceBCELoss()
    best_iou = -1.0
    early_stop_iou = -1.0
    epochs_without_improvement = 0
    history: list[dict[str, object]] = []
    print(
        f"device={device} train={len(train_dataset)} val={len(val_dataset)} "
        f"parameters={sum(p.numel() for p in model.parameters()):,}"
    )

    for epoch in range(1, args.epochs + 1):
        started = time.perf_counter()
        train_metrics = run_epoch(model, train_loader, criterion, device, optimizer)
        with torch.no_grad():
            val_metrics = run_epoch(model, val_loader, criterion, device, None)
        scheduler.step()
        elapsed = time.perf_counter() - started
        record = {
            "epoch": epoch,
            "seconds": elapsed,
            "learning_rate": optimizer.param_groups[0]["lr"],
            "train": train_metrics,
            "val": val_metrics,
        }
        history.append(record)
        (args.output / "history.json").write_text(
            json.dumps(history, indent=2), encoding="utf-8"
        )
        save_checkpoint(
            args.output / "last.pt",
            model,
            optimizer,
            epoch,
            max(best_iou, val_metrics["iou"]),
            args.image_size,
            args.base_channels,
            val_metrics,
        )
        if val_metrics["iou"] > best_iou:
            best_iou = val_metrics["iou"]
            save_checkpoint(
                args.output / "best.pt",
                model,
                optimizer,
                epoch,
                best_iou,
                args.image_size,
                args.base_channels,
                val_metrics,
            )
        if val_metrics["iou"] > early_stop_iou + args.min_delta:
            early_stop_iou = val_metrics["iou"]
            epochs_without_improvement = 0
        else:
            epochs_without_improvement += 1
        print(
            f"epoch={epoch:03d} time={elapsed:.1f}s "
            f"train_loss={train_metrics['loss']:.4f} "
            f"val_loss={val_metrics['loss']:.4f} "
            f"val_dice={val_metrics['dice']:.4f} val_iou={val_metrics['iou']:.4f}"
        )
        if args.patience > 0 and epochs_without_improvement >= args.patience:
            print(
                f"early_stop epoch={epoch} patience={args.patience} "
                f"best_iou={best_iou:.4f}"
            )
            break

    print(f"best_iou={best_iou:.4f} checkpoint={args.output / 'best.pt'}")


if __name__ == "__main__":
    main()
