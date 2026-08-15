from __future__ import annotations

import argparse
from pathlib import Path

import torch

from weld_seam.checkpoint import load_checkpoint
from project_paths import DEFAULT_CHECKPOINT, DEFAULT_ONNX_MODEL


def main() -> None:
    parser = argparse.ArgumentParser(description="Export a checkpoint to ONNX.")
    parser.add_argument("--checkpoint", type=Path, default=DEFAULT_CHECKPOINT)
    parser.add_argument("--output", type=Path, default=DEFAULT_ONNX_MODEL)
    parser.add_argument("--opset", type=int, default=18)
    args = parser.parse_args()

    model, checkpoint = load_checkpoint(args.checkpoint, "cpu")
    image_size = int(checkpoint["image_size"])
    sample = torch.zeros(1, 3, image_size, image_size, dtype=torch.float32)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    torch.onnx.export(
        model,
        sample,
        args.output,
        input_names=["image"],
        output_names=["logits"],
        opset_version=args.opset,
        do_constant_folding=True,
        external_data=False,
    )
    print(f"exported={args.output} input_size={image_size}")


if __name__ == "__main__":
    main()
