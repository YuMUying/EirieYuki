from __future__ import annotations

import argparse
import json
from pathlib import Path

import torch
from torch.utils.data import DataLoader

from weld_seam.checkpoint import load_checkpoint
from weld_seam.dataset import WeldSegmentationDataset
from weld_seam.metrics import BinarySegmentationMeter, DiceBCELoss
from project_paths import DEFAULT_CHECKPOINT, WELD_DATASET, WELD_RESULTS_DIR


def main() -> None:
    parser = argparse.ArgumentParser(description="Evaluate a segmentation checkpoint.")
    parser.add_argument("--data", type=Path, default=WELD_DATASET)
    parser.add_argument("--checkpoint", type=Path, default=DEFAULT_CHECKPOINT)
    parser.add_argument("--split", choices=("val", "test"), default="test")
    parser.add_argument("--batch-size", type=int, default=8)
    parser.add_argument("--workers", type=int, default=2)
    parser.add_argument("--threshold", type=float, default=0.5)
    parser.add_argument(
        "--output",
        type=Path,
        default=WELD_RESULTS_DIR / "evaluation" / "test_metrics.json",
    )
    args = parser.parse_args()

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model, checkpoint = load_checkpoint(args.checkpoint, device)
    dataset = WeldSegmentationDataset(
        args.data, args.split, image_size=int(checkpoint["image_size"])
    )
    loader = DataLoader(
        dataset,
        batch_size=args.batch_size,
        shuffle=False,
        num_workers=args.workers,
        persistent_workers=args.workers > 0,
    )
    criterion = DiceBCELoss()
    meter = BinarySegmentationMeter(args.threshold)
    total_loss = 0.0
    count = 0
    with torch.inference_mode():
        for images, masks, _ in loader:
            images = images.to(device)
            masks = masks.to(device)
            logits = model(images)
            total_loss += float(criterion(logits, masks).item()) * images.shape[0]
            count += images.shape[0]
            meter.update(logits, masks)
    result = {
        "split": args.split,
        "samples": count,
        "threshold": args.threshold,
        "loss": total_loss / max(count, 1),
        **meter.compute(),
    }
    text = json.dumps(result, indent=2)
    print(text)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text, encoding="utf-8")


if __name__ == "__main__":
    main()
