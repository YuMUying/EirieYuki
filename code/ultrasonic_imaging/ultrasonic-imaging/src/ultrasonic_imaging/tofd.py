from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray
from scipy.ndimage import maximum_filter

from .geometry import tofd_pixel_time, tofd_time_from_depth
from .models import ImageGrid, TofdData
from .signal import analytic_signal, envelope, normalized_db, sample_trace_linear


@dataclass(slots=True)
class TofdBScanResult:
    rf: NDArray[np.float32]
    envelope: NDArray[np.float32]
    db: NDArray[np.float32]
    scan_positions_m: NDArray[np.float64]
    time_s: NDArray[np.float64]


@dataclass(slots=True)
class TofdDScanResult:
    amplitude: NDArray[np.float32]
    db: NDArray[np.float32]
    scan_positions_m: NDArray[np.float64]
    depth_m: NDArray[np.float64]


@dataclass(slots=True)
class TofdSaftResult:
    image: NDArray[np.complex64]
    amplitude: NDArray[np.float32]
    db: NDArray[np.float32]
    coherence_factor: NDArray[np.float32] | None
    x_m: NDArray[np.float64]
    z_m: NDArray[np.float64]
    contributing_traces: int


@dataclass(frozen=True, slots=True)
class TofdTip:
    x_m: float
    depth_m: float
    amplitude: float
    db: float
    coherence: float | None = None
    phase_rad: float | None = None


@dataclass(frozen=True, slots=True)
class TofdDefect:
    x_m: float
    upper_tip: TofdTip
    lower_tip: TofdTip
    height_m: float
    score: float


def bscan(data: TofdData, floor_db: float = -60.0) -> TofdBScanResult:
    rf = np.asarray(data.rf, dtype=np.float32)
    amplitude = envelope(rf)
    time = data.time_offset_s + np.arange(rf.shape[1], dtype=np.float64) / data.sample_rate_hz
    return TofdBScanResult(
        rf=rf,
        envelope=amplitude,
        db=normalized_db(amplitude, floor_db),
        scan_positions_m=np.asarray(data.scan_positions_m, dtype=np.float64),
        time_s=time,
    )


def depth_corrected_scan(
    data: TofdData,
    depth_m: NDArray[np.floating],
    floor_db: float = -60.0,
) -> TofdDScanResult:
    analytic = analytic_signal(data.rf)
    common_delay_s = float(data.metadata.get("common_delay_s", 0.0))
    arrival_time = (
        tofd_time_from_depth(depth_m, data.probe_center_spacing_m, data.velocity_m_s)
        + common_delay_s
    )
    sample_position = (arrival_time - data.time_offset_s) * data.sample_rate_hz
    corrected = np.empty((data.rf.shape[0], depth_m.size), dtype=np.float32)
    for index, trace in enumerate(analytic):
        corrected[index] = np.abs(sample_trace_linear(trace, sample_position)).astype(np.float32)
    return TofdDScanResult(
        amplitude=corrected,
        db=normalized_db(corrected, floor_db),
        scan_positions_m=np.asarray(data.scan_positions_m, dtype=np.float64),
        depth_m=np.asarray(depth_m, dtype=np.float64),
    )


def saft(
    data: TofdData,
    grid: ImageGrid,
    aperture_m: float | None = None,
    use_coherence: bool = False,
    floor_db: float = -60.0,
) -> TofdSaftResult:
    analytic = analytic_signal(data.rf)
    geometry = str(data.metadata.get("travel_time_geometry", "perpendicular"))
    common_delay_s = float(data.metadata.get("common_delay_s", 0.0))
    pixel_x, pixel_z = grid.mesh
    coherent = np.zeros(pixel_x.shape, dtype=np.complex64)
    energy = np.zeros(pixel_x.shape, dtype=np.float64)
    count = np.zeros(pixel_x.shape, dtype=np.int32)
    for scan_index, scan_x in enumerate(data.scan_positions_m):
        active = np.ones(pixel_x.shape, dtype=bool)
        if aperture_m is not None:
            active = np.abs(pixel_x - scan_x) <= aperture_m * 0.5
        arrival_time = tofd_pixel_time(
            np.asarray(scan_x),
            pixel_x,
            pixel_z,
            data.probe_center_spacing_m,
            data.velocity_m_s,
            geometry=geometry,
        ) + common_delay_s
        sample_position = (arrival_time - data.time_offset_s) * data.sample_rate_hz
        values = sample_trace_linear(analytic[scan_index], sample_position).astype(np.complex64)
        values = np.where(active, values, 0)
        coherent += values
        if use_coherence:
            energy += np.abs(values).astype(np.float64) ** 2
        count += active
    coherence: NDArray[np.float32] | None = None
    if use_coherence:
        denominator = count * energy
        coherence = np.divide(
            np.abs(coherent).astype(np.float64) ** 2,
            denominator,
            out=np.zeros_like(energy),
            where=denominator > np.finfo(float).tiny,
        ).astype(np.float32)
        image = (coherent * coherence).astype(np.complex64)
    else:
        image = coherent
    amplitude = np.abs(image).astype(np.float32)
    return TofdSaftResult(
        image=image,
        amplitude=amplitude,
        db=normalized_db(amplitude, floor_db),
        coherence_factor=coherence,
        x_m=np.asarray(grid.x_m, dtype=np.float64),
        z_m=np.asarray(grid.z_m, dtype=np.float64),
        contributing_traces=int(data.rf.shape[0]),
    )


def suppress_common_mode(data: TofdData) -> TofdData:
    """Remove scan-invariant lateral/backwall energy before tip detection."""
    from dataclasses import replace

    rf = np.asarray(data.rf, dtype=np.float32)
    background = np.median(rf, axis=0, keepdims=True)
    return replace(data, rf=(rf - background).astype(np.float32))


def detect_tip_candidates(
    result: TofdSaftResult,
    threshold_db: float = -18.0,
    min_separation_m: float = 1.0e-3,
    max_candidates: int = 40,
) -> list[TofdTip]:
    if max_candidates < 1:
        raise ValueError("max_candidates must be positive")
    dx = float(np.median(np.diff(result.x_m)))
    dz = float(np.median(np.diff(result.z_m)))
    size_x = max(3, 2 * int(np.ceil(min_separation_m / dx)) + 1)
    size_z = max(3, 2 * int(np.ceil(min_separation_m / dz)) + 1)
    local = maximum_filter(result.amplitude, size=(size_z, size_x), mode="nearest")
    mask = (result.amplitude == local) & (result.db >= threshold_db)
    margin_x = size_x // 2
    margin_z = size_z // 2
    mask[:, :margin_x] = False
    mask[:, -margin_x:] = False
    mask[:margin_z, :] = False
    mask[-margin_z:, :] = False
    indices = np.argwhere(mask)
    if indices.size == 0:
        return []
    values = result.amplitude[indices[:, 0], indices[:, 1]]
    order = np.argsort(values)[::-1][:max_candidates]
    tips: list[TofdTip] = []
    for index in order:
        z_index, x_index = indices[index]
        coherence = (
            None
            if result.coherence_factor is None
            else float(result.coherence_factor[z_index, x_index])
        )
        tips.append(
            TofdTip(
                x_m=float(result.x_m[x_index]),
                depth_m=float(result.z_m[z_index]),
                amplitude=float(result.amplitude[z_index, x_index]),
                db=float(result.db[z_index, x_index]),
                coherence=coherence,
                phase_rad=float(np.angle(result.image[z_index, x_index])),
            )
        )
    return tips


def pair_tip_candidates(
    tips: list[TofdTip],
    max_defects: int = 2,
    max_x_difference_m: float = 2.0e-3,
    min_height_m: float = 1.0e-3,
    max_height_m: float | None = None,
    min_phase_difference_rad: float | None = None,
) -> list[TofdDefect]:
    pairs: list[TofdDefect] = []
    for first_index, first in enumerate(tips):
        for second in tips[first_index + 1 :]:
            upper, lower = sorted((first, second), key=lambda tip: tip.depth_m)
            height_m = lower.depth_m - upper.depth_m
            if abs(first.x_m - second.x_m) > max_x_difference_m:
                continue
            if height_m < min_height_m or (
                max_height_m is not None and height_m > max_height_m
            ):
                continue
            if min_phase_difference_rad is not None:
                if first.phase_rad is None or second.phase_rad is None:
                    continue
                phase_difference = abs(
                    np.angle(np.exp(1j * (first.phase_rad - second.phase_rad)))
                )
                if phase_difference < min_phase_difference_rad:
                    continue
            lateral_weight = np.exp(
                -0.5 * (abs(first.x_m - second.x_m) / max_x_difference_m) ** 2
            )
            score = float(np.sqrt(first.amplitude * second.amplitude) * lateral_weight)
            pairs.append(
                TofdDefect(
                    x_m=0.5 * (first.x_m + second.x_m),
                    upper_tip=upper,
                    lower_tip=lower,
                    height_m=height_m,
                    score=score,
                )
            )
    pairs.sort(key=lambda pair: pair.score, reverse=True)
    selected: list[TofdDefect] = []
    for pair in pairs:
        if any(abs(pair.x_m - item.x_m) <= max_x_difference_m for item in selected):
            continue
        selected.append(pair)
        if len(selected) >= max_defects:
            break
    return sorted(selected, key=lambda pair: pair.x_m)


def select_confident_defects(
    result: TofdSaftResult,
    defects: list[TofdDefect],
    min_peak_to_p95: float = 3.5,
    min_pair_score_ratio: float = 0.1,
) -> list[TofdDefect]:
    peak = float(np.max(result.amplitude))
    p95 = float(np.percentile(result.amplitude, 95.0))
    if peak <= np.finfo(float).tiny or peak / max(p95, np.finfo(float).tiny) < min_peak_to_p95:
        return []
    return [item for item in defects if item.score / peak >= min_pair_score_ratio]
