from __future__ import annotations

import csv
import json
from pathlib import Path

import cv2
import numpy as np


def save_centerline_outputs(
    output_dir: str | Path,
    source_path: str | Path,
    probability: np.ndarray,
    mask: np.ndarray,
    skeleton: np.ndarray,
    points_xy: np.ndarray,
    confidence: float,
    length_pixels: float,
    overlay_image: np.ndarray | None = None,
) -> None:
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    height, width = mask.shape
    center_x = (width - 1) / 2.0
    center_y = (height - 1) / 2.0
    points = [
        {
            "index": index,
            "x_px": float(point[0]),
            "y_px": float(point[1]),
            "x_normalized": float(point[0] / max(width - 1, 1)),
            "y_normalized": float(point[1] / max(height - 1, 1)),
            "x_offset_from_center_px": float(point[0] - center_x),
            "y_offset_from_center_px": float(point[1] - center_y),
            "x_relative_center": float((point[0] - center_x) / max(center_x, 1.0)),
            "y_relative_center": float((point[1] - center_y) / max(center_y, 1.0)),
        }
        for index, point in enumerate(points_xy)
    ]
    payload = {
        "source": str(source_path),
        "coordinate_frame": "image_top_left_x_right_y_down",
        "relative_center_range": "[-1,1]_x_right_y_down",
        "image_width": width,
        "image_height": height,
        "segmentation_confidence": confidence,
        "centerline_length_pixels": length_pixels,
        "point_count": len(points),
        "points": points,
    }
    (output / "centerline.json").write_text(
        json.dumps(payload, indent=2, ensure_ascii=True), encoding="utf-8"
    )
    with (output / "centerline.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=(
                "index",
                "x_px",
                "y_px",
                "x_normalized",
                "y_normalized",
                "x_offset_from_center_px",
                "y_offset_from_center_px",
                "x_relative_center",
                "y_relative_center",
            ),
        )
        writer.writeheader()
        writer.writerows(points)

    cv2.imwrite(str(output / "probability.png"), np.clip(probability * 255, 0, 255).astype(np.uint8))
    cv2.imwrite(str(output / "mask.png"), mask)
    edge = cv2.morphologyEx(
        mask, cv2.MORPH_GRADIENT, np.ones((3, 3), dtype=np.uint8)
    )
    cv2.imwrite(str(output / "edge.png"), edge)
    cv2.imwrite(str(output / "skeleton.png"), skeleton)
    if overlay_image is not None:
        cv2.imwrite(str(output / "overlay.png"), overlay_image)


def draw_overlay(image: np.ndarray, mask: np.ndarray, points_xy: np.ndarray) -> np.ndarray:
    overlay = image.copy()
    tint = np.zeros_like(overlay)
    tint[:, :, 1] = mask
    overlay = cv2.addWeighted(overlay, 0.75, tint, 0.25, 0.0)
    if len(points_xy) >= 2:
        polyline = np.rint(points_xy).astype(np.int32).reshape(-1, 1, 2)
        cv2.polylines(overlay, [polyline], False, (0, 0, 255), 2, cv2.LINE_AA)
    for point in points_xy:
        cv2.circle(overlay, tuple(np.rint(point).astype(int)), 2, (255, 255, 0), -1)
    return overlay
