from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np

from weld_seam.centerline import extract_centerline
from weld_seam.io_utils import save_centerline_outputs


def main() -> None:
    parser = argparse.ArgumentParser(description="Extract an ordered centerline from a binary mask.")
    parser.add_argument("--mask", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--min-area", type=int, default=100)
    parser.add_argument("--close-kernel", type=int, default=7)
    parser.add_argument("--point-spacing", type=float, default=8.0)
    parser.add_argument("--smooth-window", type=int, default=7)
    parser.add_argument(
        "--direction",
        choices=("auto", "left-to-right", "top-to-bottom"),
        default="auto",
    )
    args = parser.parse_args()

    mask = cv2.imread(str(args.mask), cv2.IMREAD_GRAYSCALE)
    if mask is None:
        raise RuntimeError(f"Failed to read mask: {args.mask}")
    result = extract_centerline(
        mask,
        args.min_area,
        args.close_kernel,
        args.point_spacing,
        args.smooth_window,
        args.direction,
    )
    probability = result.cleaned_mask.astype(np.float32) / 255.0
    save_centerline_outputs(
        args.output,
        args.mask,
        probability,
        result.cleaned_mask,
        result.skeleton,
        result.points_xy,
        1.0 if len(result.points_xy) else 0.0,
        result.length_pixels,
    )
    print(f"points={len(result.points_xy)} length_px={result.length_pixels:.1f}")


if __name__ == "__main__":
    main()
