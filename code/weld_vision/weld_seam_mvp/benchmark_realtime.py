from __future__ import annotations

import argparse
import json
import platform
from pathlib import Path
import time

import cv2
import numpy as np

from project_paths import DEFAULT_ONNX_MODEL
from weld_seam.centerline import extract_centerline
from weld_seam.inference import OnnxSegmenter
from weld_seam.preprocess import restore_points
from weld_seam.rgbd_geometry import (
    CameraIntrinsics,
    DepthProjectionConfig,
    project_centerline_to_3d,
)


def measure(function, warmup: int, runs: int) -> dict[str, float | int]:
    for _ in range(warmup):
        function()
    durations = []
    for _ in range(runs):
        started = time.perf_counter()
        function()
        durations.append((time.perf_counter() - started) * 1000.0)
    values = np.asarray(durations)
    p50 = float(np.percentile(values, 50))
    return {
        "runs": runs,
        "mean_ms": float(values.mean()),
        "p50_ms": p50,
        "p95_ms": float(np.percentile(values, 95)),
        "rate_from_p50_hz": 1000.0 / p50,
    }


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Benchmark the in-memory weld localization path with synthetic RGB-D."
    )
    parser.add_argument("--model", type=Path, default=DEFAULT_ONNX_MODEL)
    parser.add_argument(
        "--config", type=Path, default=Path(__file__).parent / "config/probe_d405_mount.yaml"
    )
    parser.add_argument("--runs", type=int, default=50)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.runs < 10:
        raise ValueError("runs must be at least 10")

    rng = np.random.default_rng(20260817)
    image = rng.integers(0, 256, size=(480, 640, 3), dtype=np.uint8)
    depth = np.full((480, 640), 120, dtype=np.uint16)
    mask = np.zeros((480, 640), dtype=np.uint8)
    curve = np.array(
        [[20, 260], [160, 220], [320, 245], [480, 205], [620, 230]],
        dtype=np.int32,
    )
    cv2.polylines(mask, [curve], False, 255, 17, cv2.LINE_AA)
    intrinsics = CameraIntrinsics(640, 480, 430.0, 430.0, 319.5, 239.5)
    config = DepthProjectionConfig.load(args.config)
    segmenter = OnnxSegmenter(args.model)

    centerline_function = lambda: extract_centerline(
        mask,
        min_area=100,
        close_kernel=7,
        point_spacing=8.0,
        smooth_window=7,
    )
    centerline = centerline_function()
    projection_function = lambda: project_centerline_to_3d(
        centerline.points_xy,
        depth,
        intrinsics,
        config,
        mount_position_m=0.0,
    )

    def complete_chain():
        probability, transform = segmenter.predict_resized(image)
        binary = np.where(probability >= 0.5, 255, 0).astype(np.uint8)
        scale = transform.resized_width / transform.original_width
        result = extract_centerline(
            binary,
            min_area=max(1, int(round(100 * scale * scale))),
            close_kernel=max(1, int(round(7 * scale))),
            point_spacing=max(1.0, 8.0 * scale),
        )
        points = restore_points(result.points_xy, transform)
        return project_centerline_to_3d(
            points, depth, intrinsics, config, mount_position_m=0.0
        )

    result = {
        "scope": "synthetic in-memory compute only; camera, ROS transport and disk excluded",
        "platform": platform.platform(),
        "processor": platform.processor(),
        "model": args.model.name,
        "input": {"model_width": 192, "model_height": 192, "rgbd_width": 640, "rgbd_height": 480},
        "providers": segmenter.session.get_providers(),
        "measurements": {
            "segmenter_predict": measure(lambda: segmenter.predict(image), 10, args.runs),
            "centerline": measure(centerline_function, 10, args.runs),
            "rgbd_projection": measure(projection_function, 10, args.runs),
            "complete_compute_chain": measure(complete_chain, 5, args.runs),
        },
    }
    text = json.dumps(result, indent=2, allow_nan=False)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text + "\n", encoding="utf-8")
    print(text)


if __name__ == "__main__":
    main()
