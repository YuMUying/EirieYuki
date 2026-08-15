from __future__ import annotations

import argparse
from dataclasses import replace
from pathlib import Path

import cv2
import numpy as np

from infer import OnnxSegmenter, TorchSegmenter
from weld_seam.centerline import extract_centerline
from weld_seam.io_utils import draw_overlay, save_centerline_outputs
from weld_seam.probe_alignment import estimate_probe_alignment, save_probe_alignment
from weld_seam.rgbd_geometry import (
    CameraIntrinsics,
    DepthProjectionConfig,
    project_centerline_to_3d,
    save_projection_outputs,
)
from project_paths import DEFAULT_CAMERA_CONFIG, DEFAULT_ONNX_MODEL


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Segment one aligned RGB-D frame and output ordered 2D and 3D weld curves."
    )
    parser.add_argument("--model", type=Path, default=DEFAULT_ONNX_MODEL)
    parser.add_argument("--color", type=Path, required=True)
    parser.add_argument("--depth", type=Path, required=True)
    parser.add_argument("--intrinsics", type=Path, required=True)
    parser.add_argument("--config", type=Path, default=DEFAULT_CAMERA_CONFIG)
    parser.add_argument(
        "--depth-scale",
        type=float,
        help="Override integer depth scale in metres/unit with the captured device value",
    )
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--rail-position",
        type=float,
        help="Rail position in metres at the RGB-D acquisition timestamp",
    )
    parser.add_argument(
        "--probe-center-y-reference",
        type=float,
        help="Override the calibrated probe center Y at rail reference (metres)",
    )
    parser.add_argument("--capture-stamp-ns", type=int)
    parser.add_argument("--task-id", default="")
    parser.add_argument("--sample-index", type=int, default=0)
    parser.add_argument("--device", default="auto", choices=("auto", "cpu", "cuda"))
    parser.add_argument("--threshold", type=float, default=0.5)
    parser.add_argument("--min-area", type=int, default=100)
    parser.add_argument("--point-spacing", type=float, default=8.0)
    parser.add_argument("--direction", choices=("auto", "left-to-right", "top-to-bottom"), default="auto")
    args = parser.parse_args()

    color = cv2.imread(str(args.color), cv2.IMREAD_COLOR)
    depth = cv2.imread(str(args.depth), cv2.IMREAD_UNCHANGED)
    if color is None or depth is None:
        raise RuntimeError("Failed to read color or depth image")
    if color.shape[:2] != depth.shape[:2]:
        raise ValueError("Color and depth must already be aligned and have identical dimensions")

    segmenter = (
        OnnxSegmenter(args.model)
        if args.model.suffix.lower() == ".onnx"
        else TorchSegmenter(args.model, args.device)
    )
    probability = segmenter.predict(color)
    raw_mask = np.where(probability >= args.threshold, 255, 0).astype(np.uint8)
    centerline = extract_centerline(
        raw_mask,
        min_area=args.min_area,
        point_spacing=args.point_spacing,
        direction=args.direction,
    )
    foreground = centerline.cleaned_mask > 0
    confidence = float(probability[foreground].mean()) if np.any(foreground) else 0.0
    save_centerline_outputs(
        args.output / "curve_2d",
        args.color,
        probability,
        centerline.cleaned_mask,
        centerline.skeleton,
        centerline.points_xy,
        confidence,
        centerline.length_pixels,
        draw_overlay(color, centerline.cleaned_mask, centerline.points_xy),
    )

    intrinsics = CameraIntrinsics.load(args.intrinsics)
    config = DepthProjectionConfig.load(args.config)
    if args.depth_scale is not None:
        config = replace(config, depth_scale_m_per_unit=args.depth_scale)
    projection = project_centerline_to_3d(
        centerline.points_xy,
        depth,
        intrinsics,
        config,
        mount_position_m=args.rail_position,
    )
    save_projection_outputs(
        args.output / "curve_3d",
        projection,
        intrinsics,
        config,
        args.output / "curve_2d" / "centerline.json",
        args.depth,
        mount_position_m=args.rail_position,
    )
    if config.linear_mount_axis_base is not None:
        if args.capture_stamp_ns is None or args.capture_stamp_ns < 0:
            raise ValueError(
                "--capture-stamp-ns is required for rail-mounted camera alignment"
            )
        if args.sample_index < 0:
            raise ValueError("--sample-index cannot be negative")
        probe_center_y = (
            config.probe_center_y_at_reference_m
            if args.probe_center_y_reference is None
            else args.probe_center_y_reference
        )
        alignment = estimate_probe_alignment(
            projection.points_base_xyz_m,
            rail_position_at_capture_m=float(args.rail_position),
            probe_center_y_at_reference_m=probe_center_y,
            geometric_confidence=confidence * projection.valid_ratio,
            minimum_points=config.probe_minimum_alignment_points,
        )
        save_probe_alignment(
            args.output / "probe_alignment.json",
            alignment,
            capture_stamp_ns=args.capture_stamp_ns,
            camera_frame=config.camera_frame,
            task_id=args.task_id,
            sample_index=args.sample_index,
        )
    print(
        f"points_2d={len(centerline.points_xy)} points_3d={len(projection.source_indices)} "
        f"valid_ratio={projection.valid_ratio:.3f} confidence={confidence:.3f}"
    )


if __name__ == "__main__":
    main()
