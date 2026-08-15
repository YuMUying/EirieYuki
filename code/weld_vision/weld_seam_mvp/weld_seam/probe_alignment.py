from __future__ import annotations

from dataclasses import asdict, dataclass
import json
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class ProbeAlignmentEstimate:
    valid: bool
    lateral_error_m: float
    target_rail_position_m: float
    rail_position_at_capture_m: float
    seam_lateral_position_m: float
    confidence: float
    point_count: int
    detail: str

    def to_mapping(self) -> dict[str, bool | float | int | str]:
        return asdict(self)


def estimate_probe_alignment(
    points_base_xyz_m: np.ndarray,
    rail_position_at_capture_m: float,
    probe_center_y_at_reference_m: float = 0.0,
    geometric_confidence: float = 1.0,
    minimum_points: int = 5,
) -> ProbeAlignmentEstimate:
    """Estimate rail correction in base_surface +Y from a timestamped 3D curve."""
    points = np.asarray(points_base_xyz_m, dtype=np.float64).reshape(-1, 3)
    finite = points[np.all(np.isfinite(points), axis=1)]
    confidence = float(np.clip(geometric_confidence, 0.0, 1.0))
    if len(finite) < minimum_points:
        return ProbeAlignmentEstimate(
            False,
            0.0,
            float(rail_position_at_capture_m),
            float(rail_position_at_capture_m),
            0.0,
            confidence,
            len(finite),
            "insufficient_3d_points",
        )
    seam_y = float(np.median(finite[:, 1]))
    probe_y = float(probe_center_y_at_reference_m + rail_position_at_capture_m)
    error = seam_y - probe_y
    return ProbeAlignmentEstimate(
        True,
        error,
        float(rail_position_at_capture_m + error),
        float(rail_position_at_capture_m),
        seam_y,
        confidence,
        len(finite),
        "ok",
    )


def save_probe_alignment(
    path: str | Path,
    estimate: ProbeAlignmentEstimate,
    capture_stamp_ns: int | None = None,
    camera_frame: str = "probe_camera_color_optical_frame",
    task_id: str = "",
    sample_index: int = 0,
) -> None:
    payload = estimate.to_mapping()
    payload.update(
        {
            "capture_stamp_ns": capture_stamp_ns,
            "camera_frame": camera_frame,
            "task_id": task_id,
            "sample_index": sample_index,
            "lateral_axis": "base_surface +Y (robot left)",
        }
    )
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(payload, indent=2, allow_nan=False), encoding="utf-8")
