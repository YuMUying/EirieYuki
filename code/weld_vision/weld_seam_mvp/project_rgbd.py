from __future__ import annotations

import argparse
from dataclasses import replace
from pathlib import Path

import cv2

from weld_seam.rgbd_geometry import (
    CameraIntrinsics,
    DepthProjectionConfig,
    load_centerline_points,
    project_centerline_to_3d,
    save_projection_outputs,
)
from project_paths import DEFAULT_CAMERA_CONFIG


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Project an ordered 2D weld centerline through an aligned depth image."
    )
    parser.add_argument("--centerline", type=Path, required=True)
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
        help="Rail position in metres at the image/depth acquisition timestamp",
    )
    args = parser.parse_args()

    points_xy, _ = load_centerline_points(args.centerline)
    depth = cv2.imread(str(args.depth), cv2.IMREAD_UNCHANGED)
    if depth is None:
        raise RuntimeError(f"Failed to read depth image: {args.depth}")
    intrinsics = CameraIntrinsics.load(args.intrinsics)
    config = DepthProjectionConfig.load(args.config)
    if args.depth_scale is not None:
        config = replace(config, depth_scale_m_per_unit=args.depth_scale)
    result = project_centerline_to_3d(
        points_xy,
        depth,
        intrinsics,
        config,
        mount_position_m=args.rail_position,
    )
    save_projection_outputs(
        args.output,
        result,
        intrinsics,
        config,
        args.centerline,
        args.depth,
        mount_position_m=args.rail_position,
    )
    print(
        f"points_2d={len(points_xy)} points_3d={len(result.source_indices)} "
        f"valid_ratio={result.valid_ratio:.3f} "
        f"rejected_depth={result.rejected_depth_count} "
        f"rejected_surface={result.rejected_surface_count}"
    )


if __name__ == "__main__":
    main()
