from __future__ import annotations

import csv
import json
from dataclasses import replace
from pathlib import Path
from time import perf_counter
from typing import Any

import numpy as np
from scipy.optimize import linear_sum_assignment

from .io import load_tofd_mat, write_json
from .models import ImageGrid, TofdData
from .signal import preprocess_rf
from .tofd import (
    TofdDefect,
    TofdSaftResult,
    TofdTip,
    detect_tip_candidates,
    pair_tip_candidates,
    saft,
    select_confident_defects,
    suppress_common_mode,
)


def preprocess_tofd(data: TofdData, common_mode: bool = True) -> TofdData:
    frequency_hz = data.center_frequency_hz
    band_hz = None
    if frequency_hz is not None:
        band_hz = (
            max(0.5e6, 0.4 * frequency_hz),
            min(0.45 * data.sample_rate_hz, 1.7 * frequency_hz),
        )
    result = replace(
        data,
        rf=preprocess_rf(data.rf, data.sample_rate_hz, band_hz=band_hz),
    )
    return suppress_common_mode(result) if common_mode else result


def automatic_grid(data: TofdData, pixel_m: float = 0.25e-3) -> ImageGrid:
    if pixel_m <= 0:
        raise ValueError("pixel size must be positive")
    config = data.metadata.get("simulation_config", {})
    raw_defects = config.get("defects", [])
    defects = raw_defects if isinstance(raw_defects, list) else ([raw_defects] if raw_defects else [])
    x_limit = max(18e-3, max((abs(float(item["x_m"])) for item in defects), default=0.0) + 4e-3)
    thickness = data.plate_thickness_m
    if thickness is None:
        raise ValueError("automatic TOFD grid requires plate thickness")
    x_m = np.arange(-x_limit, x_limit + pixel_m * 0.5, pixel_m, dtype=np.float64)
    z_m = np.arange(1.5e-3, thickness - 0.75e-3, pixel_m, dtype=np.float64)
    return ImageGrid(x_m=x_m, z_m=z_m)


def analyze_tofd(
    data: TofdData,
    *,
    pixel_m: float = 0.25e-3,
    aperture_m: float | None = 40e-3,
    threshold_db: float = -36.0,
    max_defects: int = 2,
    use_coherence: bool = False,
) -> tuple[TofdSaftResult, list[TofdTip], list[TofdDefect]]:
    processed = preprocess_tofd(data)
    result = saft(
        processed,
        automatic_grid(processed, pixel_m),
        aperture_m=aperture_m,
        use_coherence=use_coherence,
        floor_db=-60.0,
    )
    tips = detect_tip_candidates(
        result,
        threshold_db=threshold_db,
        min_separation_m=max(1.0e-3, 4 * pixel_m),
        max_candidates=60,
    )
    defects = pair_tip_candidates(
        tips,
        max_defects=max_defects,
        max_x_difference_m=1.5e-3,
        min_height_m=1.5e-3,
        max_height_m=(data.plate_thickness_m or 40e-3) * 0.9,
        min_phase_difference_rad=2.0,
    )
    return result, tips, select_confident_defects(result, defects)


def tip_to_dict(tip: TofdTip) -> dict[str, float | None]:
    return {
        "x_mm": tip.x_m * 1e3,
        "depth_mm": tip.depth_m * 1e3,
        "db": tip.db,
        "coherence": tip.coherence,
        "phase_rad": tip.phase_rad,
    }


def defect_to_dict(defect: TofdDefect) -> dict[str, Any]:
    return {
        "x_mm": defect.x_m * 1e3,
        "upper_tip": tip_to_dict(defect.upper_tip),
        "lower_tip": tip_to_dict(defect.lower_tip),
        "height_mm": defect.height_m * 1e3,
        "score": defect.score,
    }


def _truth_defects(config: dict[str, Any]) -> list[dict[str, Any]]:
    raw = config.get("defects", [])
    if not raw:
        return []
    return raw if isinstance(raw, list) else [raw]


def _match_tips(
    truth: list[dict[str, Any]],
    tips: list[TofdTip],
    depth_key: str,
    tolerance_m: float,
) -> list[tuple[int, int, float]]:
    if not truth or not tips:
        return []
    cost = np.empty((len(truth), len(tips)), dtype=np.float64)
    for truth_index, item in enumerate(truth):
        for tip_index, tip in enumerate(tips):
            dx = tip.x_m - float(item["x_m"])
            dz = tip.depth_m - float(item[depth_key])
            cost[truth_index, tip_index] = np.hypot(dx, dz)
    rows, columns = linear_sum_assignment(cost)
    return [
        (int(row), int(column), float(cost[row, column]))
        for row, column in zip(rows, columns, strict=True)
        if cost[row, column] <= tolerance_m
    ]


def _match_defects(
    truth: list[dict[str, Any]], defects: list[TofdDefect], tolerance_m: float
) -> list[tuple[int, int]]:
    if not truth or not defects:
        return []
    cost = np.empty((len(truth), len(defects)), dtype=np.float64)
    for truth_index, item in enumerate(truth):
        truth_height = float(item["z_bottom_m"]) - float(item["z_top_m"])
        for defect_index, defect in enumerate(defects):
            dx = defect.x_m - float(item["x_m"])
            dz_top = defect.upper_tip.depth_m - float(item["z_top_m"])
            dz_bottom = defect.lower_tip.depth_m - float(item["z_bottom_m"])
            dh = defect.height_m - truth_height
            cost[truth_index, defect_index] = np.sqrt(dx**2 + dz_top**2 + dz_bottom**2 + dh**2)
    rows, columns = linear_sum_assignment(cost)
    return [
        (int(row), int(column))
        for row, column in zip(rows, columns, strict=True)
        if cost[row, column] <= tolerance_m
    ]


def evaluate_case(
    mat_path: str | Path,
    *,
    pixel_m: float = 0.25e-3,
    threshold_db: float = -36.0,
    tip_tolerance_m: float = 1.5e-3,
) -> dict[str, Any]:
    started = perf_counter()
    data = load_tofd_mat(mat_path)
    truth = _truth_defects(data.metadata["simulation_config"])
    _, tips, defects = analyze_tofd(
        data,
        pixel_m=pixel_m,
        threshold_db=threshold_db,
        max_defects=max(2, len(truth)),
    )
    upper_matches = _match_tips(truth, tips, "z_top_m", tip_tolerance_m)
    lower_matches = _match_tips(truth, tips, "z_bottom_m", tip_tolerance_m)
    defect_matches = _match_defects(truth, defects, 3.5 * tip_tolerance_m)
    upper_errors = [distance * 1e3 for _, _, distance in upper_matches]
    lower_errors = [distance * 1e3 for _, _, distance in lower_matches]
    height_errors = []
    for truth_index, defect_index in defect_matches:
        item = truth[truth_index]
        true_height = float(item["z_bottom_m"]) - float(item["z_top_m"])
        height_errors.append(abs(defects[defect_index].height_m - true_height) * 1e3)
    return {
        "case_id": Path(mat_path).parent.name,
        "truth_count": len(truth),
        "candidate_count": len(tips),
        "sized_defect_count": len(defects),
        "upper_tip_hits": len(upper_matches),
        "lower_tip_hits": len(lower_matches),
        "sized_defect_hits": len(defect_matches),
        "upper_tip_errors_mm": upper_errors,
        "lower_tip_errors_mm": lower_errors,
        "height_errors_mm": height_errors,
        "false_sized_defects": len(defects) - len(defect_matches),
        "processing_seconds": perf_counter() - started,
    }


def evaluate_dataset(
    dataset_dir: str | Path,
    output_dir: str | Path,
    *,
    split: str = "test",
    pixel_m: float = 0.25e-3,
    threshold_db: float = -36.0,
    limit: int | None = None,
) -> dict[str, Any]:
    dataset_dir = Path(dataset_dir).expanduser().resolve()
    output_dir = Path(output_dir).expanduser().resolve()
    with (dataset_dir / "manifest.csv").open(newline="", encoding="utf-8") as handle:
        rows = [row for row in csv.DictReader(handle) if split == "all" or row["split"] == split]
    if limit is not None:
        rows = rows[:limit]
    if not rows:
        raise ValueError(f"no TOFD cases found for split {split!r}")
    started = perf_counter()
    cases = [
        evaluate_case(
            dataset_dir / row["mat_path"],
            pixel_m=pixel_m,
            threshold_db=threshold_db,
        )
        for row in rows
    ]
    truth_count = sum(item["truth_count"] for item in cases)
    normal_count = sum(item["truth_count"] == 0 for item in cases)
    normal_false_positive = sum(
        item["truth_count"] == 0 and item["sized_defect_count"] > 0 for item in cases
    )
    upper_errors = [value for item in cases for value in item["upper_tip_errors_mm"]]
    lower_errors = [value for item in cases for value in item["lower_tip_errors_mm"]]
    height_errors = [value for item in cases for value in item["height_errors_mm"]]
    summary = {
        "dataset_dir": str(dataset_dir),
        "split": split,
        "case_count": len(cases),
        "truth_defect_count": truth_count,
        "upper_tip_recall": sum(item["upper_tip_hits"] for item in cases) / max(truth_count, 1),
        "lower_tip_recall": sum(item["lower_tip_hits"] for item in cases) / max(truth_count, 1),
        "sizing_recall": sum(item["sized_defect_hits"] for item in cases) / max(truth_count, 1),
        "normal_case_false_positive_rate": normal_false_positive / max(normal_count, 1),
        "upper_tip_mean_error_mm": float(np.mean(upper_errors)) if upper_errors else None,
        "lower_tip_mean_error_mm": float(np.mean(lower_errors)) if lower_errors else None,
        "height_mean_absolute_error_mm": float(np.mean(height_errors)) if height_errors else None,
        "pixel_mm": pixel_m * 1e3,
        "candidate_threshold_db": threshold_db,
        "processing_seconds": perf_counter() - started,
        "limitations": [
            "Metrics apply to the semi-analytical synthetic archive, not field probability of detection.",
            "Weak lower-tip signals may be below weld-clutter peaks, so sizing recall is reported separately.",
            "A sized indication is an algorithm candidate, not an acceptance-code decision.",
        ],
    }
    output_dir.mkdir(parents=True, exist_ok=True)
    write_json(output_dir / "summary.json", summary)
    with (output_dir / "cases.json").open("w", encoding="utf-8") as handle:
        json.dump(cases, handle, indent=2, ensure_ascii=False)
        handle.write("\n")
    flat_fields = [
        "case_id",
        "truth_count",
        "candidate_count",
        "sized_defect_count",
        "upper_tip_hits",
        "lower_tip_hits",
        "sized_defect_hits",
        "false_sized_defects",
        "processing_seconds",
    ]
    with (output_dir / "cases.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=flat_fields)
        writer.writeheader()
        writer.writerows({field: item[field] for field in flat_fields} for item in cases)
    return summary
