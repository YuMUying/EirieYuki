from __future__ import annotations

import argparse
from pathlib import Path
from typing import Protocol

import cv2
import numpy as np

from weld_seam.centerline import extract_centerline
from weld_seam.io_utils import draw_overlay, save_centerline_outputs
from weld_seam.preprocess import letterbox, restore_probability
from project_paths import DEFAULT_ONNX_MODEL, WELD_RESULTS_DIR


IMAGE_SUFFIXES = {".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff"}


class Segmenter(Protocol):
    input_size: int

    def predict(self, image_bgr: np.ndarray) -> np.ndarray: ...


class TorchSegmenter:
    def __init__(self, checkpoint_path: Path, device_name: str) -> None:
        import torch

        from weld_seam.checkpoint import load_checkpoint

        if device_name == "auto":
            device_name = "cuda" if torch.cuda.is_available() else "cpu"
        self.device = torch.device(device_name)
        self.model, checkpoint = load_checkpoint(checkpoint_path, self.device)
        self.input_size = int(checkpoint["image_size"])

    def predict(self, image_bgr: np.ndarray) -> np.ndarray:
        import torch

        prepared, transform = letterbox(image_bgr, self.input_size)
        rgb = cv2.cvtColor(prepared, cv2.COLOR_BGR2RGB)
        tensor = torch.from_numpy(np.ascontiguousarray(rgb.transpose(2, 0, 1)))
        tensor = tensor.float().div_(255.0).unsqueeze(0).to(self.device)
        with torch.inference_mode():
            probability = torch.sigmoid(self.model(tensor))[0, 0].cpu().numpy()
        return restore_probability(probability, transform)


class OnnxSegmenter:
    def __init__(self, model_path: Path) -> None:
        import onnxruntime as ort

        self.session = ort.InferenceSession(
            str(model_path), providers=["CPUExecutionProvider"]
        )
        self.input_name = self.session.get_inputs()[0].name
        shape = self.session.get_inputs()[0].shape
        self.input_size = int(shape[-1])

    def predict(self, image_bgr: np.ndarray) -> np.ndarray:
        prepared, transform = letterbox(image_bgr, self.input_size)
        rgb = cv2.cvtColor(prepared, cv2.COLOR_BGR2RGB)
        tensor = np.ascontiguousarray(rgb.transpose(2, 0, 1), dtype=np.float32)
        tensor = tensor[None] / 255.0
        logits = self.session.run(None, {self.input_name: tensor})[0][0, 0]
        probability = 1.0 / (1.0 + np.exp(-np.clip(logits, -30.0, 30.0)))
        return restore_probability(probability, transform)


def image_paths(path: Path) -> list[Path]:
    if path.is_file():
        return [path]
    if not path.is_dir():
        raise FileNotFoundError(path)
    return sorted(
        item
        for item in path.iterdir()
        if item.is_file() and item.suffix.lower() in IMAGE_SUFFIXES
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Segment weld areas and output an ordered centerline curve."
    )
    parser.add_argument(
        "--model", type=Path, default=DEFAULT_ONNX_MODEL, help="PyTorch .pt or ONNX .onnx"
    )
    parser.add_argument("--input", type=Path, required=True, help="Image or flat image directory")
    parser.add_argument(
        "--output", type=Path, default=WELD_RESULTS_DIR / "inference"
    )
    parser.add_argument("--device", default="auto", choices=("auto", "cpu", "cuda"))
    parser.add_argument("--threshold", type=float, default=0.5)
    parser.add_argument("--min-area", type=int, default=100)
    parser.add_argument("--close-kernel", type=int, default=7)
    parser.add_argument("--point-spacing", type=float, default=8.0)
    parser.add_argument("--smooth-window", type=int, default=7)
    parser.add_argument(
        "--direction",
        choices=("auto", "left-to-right", "top-to-bottom"),
        default="auto",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if not 0.0 < args.threshold < 1.0:
        raise ValueError("threshold must be between 0 and 1")
    if args.model.suffix.lower() == ".onnx":
        segmenter: Segmenter = OnnxSegmenter(args.model)
    else:
        segmenter = TorchSegmenter(args.model, args.device)

    paths = image_paths(args.input)
    if not paths:
        raise RuntimeError(f"No images found: {args.input}")
    for path in paths:
        image = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if image is None:
            print(f"skip unreadable image: {path}")
            continue
        probability = segmenter.predict(image)
        raw_mask = np.where(probability >= args.threshold, 255, 0).astype(np.uint8)
        result = extract_centerline(
            raw_mask,
            min_area=args.min_area,
            close_kernel=args.close_kernel,
            point_spacing=args.point_spacing,
            smooth_window=args.smooth_window,
            direction=args.direction,
        )
        foreground = result.cleaned_mask > 0
        confidence = float(probability[foreground].mean()) if np.any(foreground) else 0.0
        overlay = draw_overlay(image, result.cleaned_mask, result.points_xy)
        destination = args.output / path.stem
        save_centerline_outputs(
            destination,
            path,
            probability,
            result.cleaned_mask,
            result.skeleton,
            result.points_xy,
            confidence,
            result.length_pixels,
            overlay,
        )
        print(
            f"{path.name}: points={len(result.points_xy)} "
            f"length_px={result.length_pixels:.1f} confidence={confidence:.3f}"
        )


if __name__ == "__main__":
    main()
