from __future__ import annotations

import numpy as np

from .geometry import tofd_pixel_time, tofd_time_from_depth
from .models import TofdData


def _add_toneburst(
    rf: np.ndarray,
    arrival_time_s: np.ndarray,
    sample_rate_hz: float,
    center_frequency_hz: float,
    amplitude: float,
    cycles: float = 2.5,
) -> None:
    half_width_s = cycles / center_frequency_hz
    half_samples = int(np.ceil(half_width_s * sample_rate_hz))
    offsets = np.arange(-half_samples, half_samples + 1)
    for trace, arrival in enumerate(np.asarray(arrival_time_s).reshape(-1)):
        center = int(round(arrival * sample_rate_hz))
        indices = center + offsets
        valid = (indices >= 0) & (indices < rf.shape[1])
        local_t = indices[valid] / sample_rate_hz - arrival
        window = np.exp(-((local_t * center_frequency_hz) / 1.15) ** 2)
        pulse = amplitude * window * np.cos(2.0 * np.pi * center_frequency_hz * local_t)
        rf[trace, indices[valid]] += pulse


def simulate_tofd(
    scan_start_m: float = -0.050,
    scan_end_m: float = 0.050,
    scan_step_m: float = 0.0005,
    sample_rate_hz: float = 100e6,
    sample_count: int = 4096,
    velocity_m_s: float = 5890.0,
    probe_center_spacing_m: float = 0.060,
    plate_thickness_m: float = 0.030,
    center_frequency_hz: float = 5e6,
    flaw_scan_position_m: float = 0.0,
    upper_tip_depth_m: float = 0.012,
    lower_tip_depth_m: float = 0.020,
    noise_std: float = 0.035,
    seed: int = 7,
) -> TofdData:
    """Create a traceable physics-based TOFD demonstration dataset.

    It models the lateral wave, backwall reflection and opposite-polarity upper
    and lower crack-tip diffraction hyperbolae. It is not a substitute for
    measured instrument data.
    """
    if not 0 < upper_tip_depth_m < lower_tip_depth_m < plate_thickness_m:
        raise ValueError("tip depths must be ordered inside the plate")
    scans = np.arange(scan_start_m, scan_end_m + scan_step_m * 0.5, scan_step_m)
    rng = np.random.default_rng(seed)
    rf = rng.normal(0.0, noise_std, (scans.size, sample_count)).astype(np.float32)

    lateral_time = np.full(scans.shape, probe_center_spacing_m / velocity_m_s)
    backwall_time = np.full(
        scans.shape,
        float(tofd_time_from_depth(plate_thickness_m, probe_center_spacing_m, velocity_m_s)),
    )
    upper_time = tofd_pixel_time(
        scans,
        np.asarray(flaw_scan_position_m),
        np.asarray(upper_tip_depth_m),
        probe_center_spacing_m,
        velocity_m_s,
    )
    lower_time = tofd_pixel_time(
        scans,
        np.asarray(flaw_scan_position_m),
        np.asarray(lower_tip_depth_m),
        probe_center_spacing_m,
        velocity_m_s,
    )
    _add_toneburst(rf, lateral_time, sample_rate_hz, center_frequency_hz, 1.0)
    _add_toneburst(rf, upper_time, sample_rate_hz, center_frequency_hz, 0.55)
    _add_toneburst(rf, lower_time, sample_rate_hz, center_frequency_hz, -0.45)
    _add_toneburst(rf, backwall_time, sample_rate_hz, center_frequency_hz, -0.75)
    return TofdData(
        rf=rf,
        sample_rate_hz=sample_rate_hz,
        velocity_m_s=velocity_m_s,
        scan_positions_m=scans.astype(np.float64),
        probe_center_spacing_m=probe_center_spacing_m,
        plate_thickness_m=plate_thickness_m,
        center_frequency_hz=center_frequency_hz,
        metadata={
            "source": "physics-based synthetic TOFD demonstration",
            "measured_data": False,
            "flaw_scan_position_m": flaw_scan_position_m,
            "upper_tip_depth_m": upper_tip_depth_m,
            "lower_tip_depth_m": lower_tip_depth_m,
            "modelled_signals": ["lateral_wave", "upper_tip", "lower_tip", "backwall"],
            "seed": seed,
        },
    )
