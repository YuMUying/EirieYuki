from __future__ import annotations

import csv
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import yaml


@dataclass(frozen=True)
class CameraIntrinsics:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    distortion_model: str = "none"
    distortion_coefficients: tuple[float, ...] = ()

    def __post_init__(self) -> None:
        if self.width <= 0 or self.height <= 0:
            raise ValueError("Camera image dimensions must be positive")
        if self.fx <= 0 or self.fy <= 0:
            raise ValueError("Camera focal lengths must be positive")
        if not all(
            np.isfinite(value)
            for value in (self.fx, self.fy, self.cx, self.cy, *self.distortion_coefficients)
        ):
            raise ValueError("Camera intrinsics must be finite")

    @property
    def matrix(self) -> np.ndarray:
        return np.array(
            [[self.fx, 0.0, self.cx], [0.0, self.fy, self.cy], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )

    @classmethod
    def from_mapping(cls, data: dict[str, Any]) -> "CameraIntrinsics":
        coefficients = data.get("distortion_coefficients", data.get("D", []))
        return cls(
            width=int(data["width"]),
            height=int(data["height"]),
            fx=float(data["fx"]),
            fy=float(data["fy"]),
            cx=float(data["cx"]),
            cy=float(data["cy"]),
            distortion_model=str(data.get("distortion_model", "none")),
            distortion_coefficients=tuple(float(value) for value in coefficients),
        )

    @classmethod
    def load(cls, path: str | Path) -> "CameraIntrinsics":
        with Path(path).open("r", encoding="utf-8") as handle:
            return cls.from_mapping(json.load(handle))

    def validate_for_image(self, image: np.ndarray) -> None:
        height, width = image.shape[:2]
        if (width, height) != (self.width, self.height):
            raise ValueError(
                f"Intrinsics are {self.width}x{self.height}, but the depth image is "
                f"{width}x{height}. Align depth to color and use matching intrinsics."
            )


@dataclass(frozen=True)
class DepthProjectionConfig:
    depth_scale_m_per_unit: float
    min_depth_m: float
    max_depth_m: float
    neighborhood_radius_px: int
    min_valid_neighbors: int
    max_local_depth_mad_m: float
    max_surface_distance_m: float | None
    transform_base_from_camera: np.ndarray
    camera_frame: str = "camera_color_optical_frame"
    base_frame: str = "base_surface"
    linear_mount_axis_base: tuple[float, float, float] | None = None
    linear_mount_reference_position_m: float = 0.0
    probe_center_y_at_reference_m: float = 0.0
    probe_minimum_alignment_points: int = 5

    def __post_init__(self) -> None:
        if self.depth_scale_m_per_unit <= 0:
            raise ValueError("depth scale must be positive")
        if not 0 < self.min_depth_m < self.max_depth_m:
            raise ValueError("depth range must satisfy 0 < min_depth_m < max_depth_m")
        if self.neighborhood_radius_px < 0:
            raise ValueError("neighborhood radius cannot be negative")
        if self.min_valid_neighbors < 1:
            raise ValueError("min_valid_neighbors must be at least one")
        if self.max_local_depth_mad_m < 0:
            raise ValueError("max_local_depth_mad_m cannot be negative")
        if self.max_surface_distance_m is not None and self.max_surface_distance_m < 0:
            raise ValueError("surface distance limit cannot be negative")
        validate_transform(np.asarray(self.transform_base_from_camera))
        if self.linear_mount_axis_base is not None:
            axis = np.asarray(self.linear_mount_axis_base, dtype=np.float64)
            if axis.shape != (3,) or not np.all(np.isfinite(axis)):
                raise ValueError("linear mount axis must contain three finite values")
            if not np.isclose(np.linalg.norm(axis), 1.0, atol=1e-6):
                raise ValueError("linear mount axis must be a unit vector")
        if not np.isfinite(self.linear_mount_reference_position_m):
            raise ValueError("linear mount reference position must be finite")
        if not np.isfinite(self.probe_center_y_at_reference_m):
            raise ValueError("probe center reference must be finite")
        if self.probe_minimum_alignment_points < 1:
            raise ValueError("probe minimum alignment points must be positive")

    @classmethod
    def load(cls, path: str | Path) -> "DepthProjectionConfig":
        with Path(path).open("r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle)
        depth = data["depth"]
        validation = data.get("validation", {})
        transform = np.asarray(data["transform_base_from_camera"], dtype=np.float64)
        validate_transform(transform)
        surface_limit = validation.get("max_abs_surface_z_m")
        linear_mount = data.get("linear_mount", {})
        axis = linear_mount.get("axis_base") if linear_mount.get("enabled", False) else None
        probe = data.get("probe", {})
        return cls(
            depth_scale_m_per_unit=float(depth["scale_m_per_unit"]),
            min_depth_m=float(depth["min_m"]),
            max_depth_m=float(depth["max_m"]),
            neighborhood_radius_px=int(depth.get("neighborhood_radius_px", 3)),
            min_valid_neighbors=int(depth.get("min_valid_neighbors", 3)),
            max_local_depth_mad_m=float(depth.get("max_local_depth_mad_m", 0.015)),
            max_surface_distance_m=None if surface_limit is None else float(surface_limit),
            transform_base_from_camera=transform,
            camera_frame=str(data.get("frames", {}).get("camera", "camera_color_optical_frame")),
            base_frame=str(data.get("frames", {}).get("base", "base_surface")),
            linear_mount_axis_base=(
                None if axis is None else tuple(float(value) for value in axis)
            ),
            linear_mount_reference_position_m=float(
                linear_mount.get("reference_position_m", 0.0)
            ),
            probe_center_y_at_reference_m=float(
                probe.get("center_y_at_reference_m", 0.0)
            ),
            probe_minimum_alignment_points=int(
                probe.get("minimum_alignment_points", 5)
            ),
        )

    def transform_at_mount_position(self, position_m: float | None) -> np.ndarray:
        """Return the camera extrinsic at the timestamped linear-mount position."""
        transform = np.array(self.transform_base_from_camera, dtype=np.float64, copy=True)
        if self.linear_mount_axis_base is None:
            if position_m is not None:
                raise ValueError("mount position supplied for a fixed camera profile")
            return transform
        if position_m is None or not np.isfinite(position_m):
            raise ValueError("a finite mount position is required for this camera profile")
        displacement = position_m - self.linear_mount_reference_position_m
        transform[:3, 3] += np.asarray(self.linear_mount_axis_base) * displacement
        return transform



def validate_transform(transform: np.ndarray, tolerance: float = 1e-6) -> None:
    if transform.shape != (4, 4):
        raise ValueError("transform_base_from_camera must be a 4x4 matrix")
    if not np.allclose(transform[3], [0.0, 0.0, 0.0, 1.0], atol=tolerance):
        raise ValueError("The last transform row must be [0, 0, 0, 1]")
    rotation = transform[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=tolerance):
        raise ValueError("Transform rotation is not orthonormal")
    if not np.isclose(np.linalg.det(rotation), 1.0, atol=tolerance):
        raise ValueError("Transform rotation determinant must be +1")


def recommended_d405_mount_transform(
    pitch_from_wall_normal_deg: float = 35.0,
    optical_center_xyz_m: tuple[float, float, float] = (0.135, 0.0, 0.200),
) -> np.ndarray:
    """Return base_surface <- camera_optical for a forward-tilted wall camera.

    base_surface: +X robot forward, +Y robot left, +Z away from the wall.
    camera optical: +X image right, +Y image down, +Z optical forward.
    """
    angle = np.deg2rad(pitch_from_wall_normal_deg)
    sine = float(np.sin(angle))
    cosine = float(np.cos(angle))
    rotation = np.array(
        [[0.0, -cosine, sine], [-1.0, 0.0, 0.0], [0.0, -sine, -cosine]],
        dtype=np.float64,
    )
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rotation
    transform[:3, 3] = optical_center_xyz_m
    return transform


def _undistorted_normalized_points(
    points_xy: np.ndarray, intrinsics: CameraIntrinsics
) -> np.ndarray:
    points = np.asarray(points_xy, dtype=np.float64).reshape(-1, 1, 2)
    coefficients = np.asarray(intrinsics.distortion_coefficients, dtype=np.float64)
    has_distortion = coefficients.size > 0 and np.any(np.abs(coefficients) > 1e-12)
    if has_distortion:
        model = intrinsics.distortion_model.lower().replace("distortion.", "")
        if model == "inverse_brown_conrady":
            coefficients = np.pad(coefficients[:5], (0, max(0, 5 - coefficients.size)))
            normalized = np.empty((len(points), 2), dtype=np.float64)
            normalized[:, 0] = (points[:, 0, 0] - intrinsics.cx) / intrinsics.fx
            normalized[:, 1] = (points[:, 0, 1] - intrinsics.cy) / intrinsics.fy
            original = normalized.copy()
            k1, k2, p1, p2, k3 = coefficients
            for _ in range(10):
                x, y = normalized[:, 0], normalized[:, 1]
                radius_squared = x * x + y * y
                inverse_radial = 1.0 / (
                    1.0 + k1 * radius_squared + k2 * radius_squared**2 + k3 * radius_squared**3
                )
                xq, yq = x / inverse_radial, y / inverse_radial
                delta_x = 2.0 * p1 * xq * yq + p2 * (
                    radius_squared + 2.0 * xq * xq
                )
                delta_y = 2.0 * p2 * xq * yq + p1 * (
                    radius_squared + 2.0 * yq * yq
                )
                normalized[:, 0] = (original[:, 0] - delta_x) * inverse_radial
                normalized[:, 1] = (original[:, 1] - delta_y) * inverse_radial
            return normalized
        supported_models = {
            "brown_conrady",
            "modified_brown_conrady",
            "plumb_bob",
            "opencv",
        }
        if model not in supported_models:
            raise ValueError(
                f"Unsupported non-zero distortion model: {intrinsics.distortion_model}. "
                "Use a rectified stream or Brown-Conrady color intrinsics."
            )
        normalized = cv2.undistortPoints(points, intrinsics.matrix, coefficients)
        return normalized.reshape(-1, 2)
    normalized = np.empty((len(points), 2), dtype=np.float64)
    normalized[:, 0] = (points[:, 0, 0] - intrinsics.cx) / intrinsics.fx
    normalized[:, 1] = (points[:, 0, 1] - intrinsics.cy) / intrinsics.fy
    return normalized


def _depth_in_meters(depth_image: np.ndarray, scale_m_per_unit: float) -> np.ndarray:
    if depth_image.ndim != 2:
        raise ValueError("Depth image must be single-channel")
    if np.issubdtype(depth_image.dtype, np.floating):
        return depth_image.astype(np.float64)
    if scale_m_per_unit <= 0:
        raise ValueError("depth scale must be positive for integer depth images")
    return depth_image.astype(np.float64) * scale_m_per_unit


def sample_depths(
    depth_image: np.ndarray,
    points_xy: np.ndarray,
    config: DepthProjectionConfig,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Sample robust median depth around ordered 2D points.

    Returns depth_m, valid_mask and local MAD in metres. Floating depth images are
    interpreted as metres; integer depth images use scale_m_per_unit.
    """
    depth_m = _depth_in_meters(depth_image, config.depth_scale_m_per_unit)
    height, width = depth_m.shape
    points = np.asarray(points_xy, dtype=np.float64)
    if points.size == 0:
        points = points.reshape(0, 2)
    elif points.ndim != 2 or points.shape[1] != 2:
        raise ValueError("points_xy must have shape (N, 2)")
    values = np.full(len(points), np.nan, dtype=np.float64)
    mads = np.full(len(points), np.nan, dtype=np.float64)
    valid = np.zeros(len(points), dtype=bool)
    radius = max(0, config.neighborhood_radius_px)

    for index, (x_value, y_value) in enumerate(points):
        x = int(round(x_value))
        y = int(round(y_value))
        if x < 0 or x >= width or y < 0 or y >= height:
            continue
        x0, x1 = max(0, x - radius), min(width, x + radius + 1)
        y0, y1 = max(0, y - radius), min(height, y + radius + 1)
        neighborhood = depth_m[y0:y1, x0:x1]
        candidates = neighborhood[
            np.isfinite(neighborhood)
            & (neighborhood >= config.min_depth_m)
            & (neighborhood <= config.max_depth_m)
        ]
        if candidates.size < config.min_valid_neighbors:
            continue
        median = float(np.median(candidates))
        mad = float(np.median(np.abs(candidates - median)))
        if config.max_local_depth_mad_m > 0 and mad > config.max_local_depth_mad_m:
            continue
        values[index] = median
        mads[index] = mad
        valid[index] = True
    return values, valid, mads


@dataclass(frozen=True)
class ProjectionResult:
    source_indices: np.ndarray
    points_image_xy: np.ndarray
    depths_m: np.ndarray
    points_camera_xyz_m: np.ndarray
    points_base_xyz_m: np.ndarray
    local_depth_mad_m: np.ndarray
    valid_ratio: float
    rejected_depth_count: int
    rejected_surface_count: int


def project_centerline_to_3d(
    points_xy: np.ndarray,
    depth_image: np.ndarray,
    intrinsics: CameraIntrinsics,
    config: DepthProjectionConfig,
    mount_position_m: float | None = None,
) -> ProjectionResult:
    points = np.asarray(points_xy, dtype=np.float64).reshape(-1, 2)
    if not np.all(np.isfinite(points)):
        raise ValueError("Centerline coordinates must be finite")
    intrinsics.validate_for_image(depth_image)
    depths, valid_depth, mads = sample_depths(depth_image, points, config)
    source_indices = np.flatnonzero(valid_depth)
    rejected_depth = int(len(points) - len(source_indices))
    if source_indices.size == 0:
        return ProjectionResult(
            source_indices,
            np.empty((0, 2)),
            np.empty(0),
            np.empty((0, 3)),
            np.empty((0, 3)),
            np.empty(0),
            0.0,
            rejected_depth,
            0,
        )

    selected_image = points[source_indices]
    selected_depths = depths[source_indices]
    normalized = _undistorted_normalized_points(selected_image, intrinsics)
    points_camera = np.column_stack(
        (
            normalized[:, 0] * selected_depths,
            normalized[:, 1] * selected_depths,
            selected_depths,
        )
    )
    homogeneous = np.column_stack((points_camera, np.ones(len(points_camera))))
    transform_base_from_camera = config.transform_at_mount_position(mount_position_m)
    points_base = (transform_base_from_camera @ homogeneous.T).T[:, :3]

    rejected_surface = 0
    if config.max_surface_distance_m is not None:
        surface_valid = np.abs(points_base[:, 2]) <= config.max_surface_distance_m
        rejected_surface = int(np.count_nonzero(~surface_valid))
        source_indices = source_indices[surface_valid]
        selected_image = selected_image[surface_valid]
        selected_depths = selected_depths[surface_valid]
        points_camera = points_camera[surface_valid]
        points_base = points_base[surface_valid]
        selected_mads = mads[source_indices]
    else:
        selected_mads = mads[source_indices]

    return ProjectionResult(
        source_indices=source_indices,
        points_image_xy=selected_image,
        depths_m=selected_depths,
        points_camera_xyz_m=points_camera,
        points_base_xyz_m=points_base,
        local_depth_mad_m=selected_mads,
        valid_ratio=float(len(source_indices) / max(len(points), 1)),
        rejected_depth_count=rejected_depth,
        rejected_surface_count=rejected_surface,
    )


def load_centerline_points(path: str | Path) -> tuple[np.ndarray, dict[str, Any]]:
    with Path(path).open("r", encoding="utf-8") as handle:
        payload = json.load(handle)
    points = np.array(
        [[point["x_px"], point["y_px"]] for point in payload.get("points", [])],
        dtype=np.float64,
    ).reshape(-1, 2)
    return points, payload


def save_projection_outputs(
    output_dir: str | Path,
    result: ProjectionResult,
    intrinsics: CameraIntrinsics,
    config: DepthProjectionConfig,
    source_centerline: str | Path,
    source_depth: str | Path,
    mount_position_m: float | None = None,
) -> None:
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    rows = []
    segment_ids = np.zeros(len(result.source_indices), dtype=np.int64)
    if len(segment_ids) > 1:
        segment_ids[1:] = np.cumsum(np.diff(result.source_indices) > 1)
    for output_index, source_index in enumerate(result.source_indices):
        image = result.points_image_xy[output_index]
        camera = result.points_camera_xyz_m[output_index]
        base = result.points_base_xyz_m[output_index]
        rows.append(
            {
                "index": output_index,
                "segment_id": int(segment_ids[output_index]),
                "source_centerline_index": int(source_index),
                "u_px": float(image[0]),
                "v_px": float(image[1]),
                "depth_m": float(result.depths_m[output_index]),
                "local_depth_mad_m": float(result.local_depth_mad_m[output_index]),
                "camera_x_m": float(camera[0]),
                "camera_y_m": float(camera[1]),
                "camera_z_m": float(camera[2]),
                "base_x_m": float(base[0]),
                "base_y_m": float(base[1]),
                "base_z_m": float(base[2]),
            }
        )
    payload = {
        "source_centerline": str(source_centerline),
        "source_depth": str(source_depth),
        "camera_frame": config.camera_frame,
        "base_frame": config.base_frame,
        "camera_convention": "+X right, +Y down, +Z forward",
        "base_convention": "+X robot forward, +Y robot left, +Z away from wall",
        "mount_position_m": mount_position_m,
        "transform_base_from_camera": config.transform_at_mount_position(
            mount_position_m
        ).tolist(),
        "intrinsics": {
            "width": intrinsics.width,
            "height": intrinsics.height,
            "fx": intrinsics.fx,
            "fy": intrinsics.fy,
            "cx": intrinsics.cx,
            "cy": intrinsics.cy,
            "distortion_model": intrinsics.distortion_model,
            "distortion_coefficients": list(intrinsics.distortion_coefficients),
        },
        "valid_ratio": result.valid_ratio,
        "point_count": len(rows),
        "segment_count": int(segment_ids[-1] + 1) if len(segment_ids) else 0,
        "rejected_depth_count": result.rejected_depth_count,
        "rejected_surface_count": result.rejected_surface_count,
        "points": rows,
    }
    (output / "centerline_3d.json").write_text(
        json.dumps(payload, indent=2, ensure_ascii=True), encoding="utf-8"
    )
    with (output / "centerline_3d.csv").open("w", newline="", encoding="utf-8") as handle:
        fieldnames = list(rows[0]) if rows else [
            "index",
            "segment_id",
            "source_centerline_index",
            "u_px",
            "v_px",
            "depth_m",
            "local_depth_mad_m",
            "camera_x_m",
            "camera_y_m",
            "camera_z_m",
            "base_x_m",
            "base_y_m",
            "base_z_m",
        ]
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
