from __future__ import annotations

import argparse
import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import yaml

from project_paths import CODE_DIR, DEFAULT_CAMERA_CONFIG


def device_info(device: Any, field: Any) -> str | None:
    return device.get_info(field) if device.supports(field) else None


def intrinsics_payload(intrinsics: Any, depth_scale: float) -> dict[str, Any]:
    return {
        "width": int(intrinsics.width),
        "height": int(intrinsics.height),
        "fx": float(intrinsics.fx),
        "fy": float(intrinsics.fy),
        "cx": float(intrinsics.ppx),
        "cy": float(intrinsics.ppy),
        "distortion_model": str(intrinsics.model),
        "distortion_coefficients": [float(value) for value in intrinsics.coeffs],
        "depth_scale_m_per_unit": float(depth_scale),
        "pixel_frame": "color image; depth aligned to color",
    }


def write_runtime_config(
    template: Path,
    output: Path,
    depth_scale: float,
    role: str,
    serial_number: str | None,
) -> None:
    with template.open("r", encoding="utf-8") as handle:
        config = yaml.safe_load(handle)
    config["depth"]["scale_m_per_unit"] = float(depth_scale)
    config["camera"]["captured_device_scale"] = True
    config["camera"]["role"] = role
    config["camera"]["serial_number"] = serial_number
    output.write_text(
        yaml.safe_dump(config, sort_keys=False, allow_unicode=False), encoding="utf-8"
    )


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Capture one D405 color frame and Z16 depth frame aligned to color."
    )
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--serial", help="Optional RealSense serial number")
    parser.add_argument("--role", choices=("top", "probe"), default="top")
    parser.add_argument("--width", type=int)
    parser.add_argument("--height", type=int)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--warmup-frames", type=int, default=30)
    parser.add_argument("--timeout-ms", type=int, default=5000)
    parser.add_argument("--config-template", type=Path)
    args = parser.parse_args()
    default_size = (1280, 720) if args.role == "top" else (640, 480)
    args.width = args.width or default_size[0]
    args.height = args.height or default_size[1]
    if args.config_template is None:
        args.config_template = (
            DEFAULT_CAMERA_CONFIG
            if args.role == "top"
            else CODE_DIR / "config" / "probe_d405_mount.yaml"
        )

    try:
        import pyrealsense2 as rs
    except ImportError as exc:
        raise RuntimeError(
            "pyrealsense2 is required; install requirements-realsense.txt after "
            "installing librealsense on the target Linux system"
        ) from exc

    if args.width <= 0 or args.height <= 0 or args.fps <= 0:
        raise ValueError("Stream width, height and fps must be positive")
    if args.warmup_frames < 0:
        raise ValueError("warmup-frames cannot be negative")

    devices = list(rs.context().query_devices())
    if not devices:
        raise RuntimeError("No RealSense device found")
    available_serials = [
        device_info(device, rs.camera_info.serial_number) for device in devices
    ]
    if len(devices) > 1 and not args.serial:
        raise RuntimeError(
            "Multiple RealSense devices found; --serial is required to bind the "
            f"{args.role} role. Available serials: {available_serials}"
        )
    if args.serial and args.serial not in available_serials:
        raise RuntimeError(
            f"RealSense serial {args.serial!r} was not found; available: "
            f"{available_serials}"
        )

    pipeline = rs.pipeline()
    stream_config = rs.config()
    if args.serial:
        stream_config.enable_device(args.serial)
    stream_config.enable_stream(
        rs.stream.depth, args.width, args.height, rs.format.z16, args.fps
    )
    stream_config.enable_stream(
        rs.stream.color, args.width, args.height, rs.format.bgr8, args.fps
    )
    align_to_color = rs.align(rs.stream.color)

    profile = pipeline.start(stream_config)
    try:
        device = profile.get_device()
        depth_sensor = device.first_depth_sensor()
        depth_scale = float(depth_sensor.get_depth_scale())
        aligned_frames = None
        for _ in range(args.warmup_frames + 1):
            frames = pipeline.wait_for_frames(args.timeout_ms)
            aligned_frames = align_to_color.process(frames)
        if aligned_frames is None:
            raise RuntimeError("No RealSense frames received")

        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()
        if not color_frame or not depth_frame:
            raise RuntimeError("Aligned color/depth frame pair is incomplete")

        color = np.asanyarray(color_frame.get_data())
        depth = np.asanyarray(depth_frame.get_data())
        if color.shape[:2] != depth.shape[:2] or depth.dtype != np.uint16:
            raise RuntimeError(
                f"Unexpected aligned frame formats: color={color.shape}, "
                f"depth={depth.shape}/{depth.dtype}"
            )
        intrinsics = color_frame.profile.as_video_stream_profile().intrinsics
        serial_number = device_info(device, rs.camera_info.serial_number)

        args.output.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(args.output / "color.png"), color):
            raise RuntimeError("Failed to write color.png")
        if not cv2.imwrite(str(args.output / "depth.png"), depth):
            raise RuntimeError("Failed to write depth.png")

        (args.output / "intrinsics.json").write_text(
            json.dumps(intrinsics_payload(intrinsics, depth_scale), indent=2),
            encoding="utf-8",
        )
        write_runtime_config(
            args.config_template,
            args.output / "projection_config.yaml",
            depth_scale,
            args.role,
            serial_number,
        )
        metadata = {
            "captured_at_utc": datetime.now(timezone.utc).isoformat(),
            "camera_role": args.role,
            "camera_name": device_info(device, rs.camera_info.name),
            "serial_number": serial_number,
            "firmware_version": device_info(device, rs.camera_info.firmware_version),
            "usb_type": device_info(device, rs.camera_info.usb_type_descriptor),
            "depth_aligned_to_color": True,
            "depth_encoding": "uint16 Z16",
            "depth_scale_m_per_unit": depth_scale,
            "color_timestamp_ms": float(color_frame.get_timestamp()),
            "depth_timestamp_ms": float(depth_frame.get_timestamp()),
            "color_timestamp_domain": str(color_frame.get_frame_timestamp_domain()),
            "depth_timestamp_domain": str(depth_frame.get_frame_timestamp_domain()),
            "timestamp_mapping": "SDK timestamps are raw and not ROS/PTP mapped",
            "stream": {"width": args.width, "height": args.height, "fps": args.fps},
        }
        (args.output / "metadata.json").write_text(
            json.dumps(metadata, indent=2), encoding="utf-8"
        )
    finally:
        pipeline.stop()

    print(
        f"saved={args.output} size={args.width}x{args.height} "
        f"depth_scale={depth_scale:.9f} m/unit"
    )


if __name__ == "__main__":
    main()
