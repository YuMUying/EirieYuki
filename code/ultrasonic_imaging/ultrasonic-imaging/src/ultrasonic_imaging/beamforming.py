from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from .geometry import homogeneous_one_way_time, planar_interface_time
from .models import FmcData, ImageGrid, PwiData
from .signal import analytic_signal, normalized_db, sample_trace_linear


@dataclass(slots=True)
class BeamformResult:
    image: NDArray[np.complex64]
    amplitude: NDArray[np.float32]
    db: NDArray[np.float32]
    coherence_factor: NDArray[np.float32] | None
    x_m: NDArray[np.float64]
    z_m: NDArray[np.float64]
    contributing_channels: int


def _travel_times(
    element_x_m: NDArray[np.floating],
    pixel_x_m: NDArray[np.floating],
    pixel_z_m: NDArray[np.floating],
    specimen_velocity_m_s: float,
    element_z_m: NDArray[np.floating] | None,
    wedge_velocity_m_s: float | None,
) -> NDArray[np.float64]:
    times = np.empty((element_x_m.size, *pixel_x_m.shape), dtype=np.float64)
    if wedge_velocity_m_s is None:
        for index, x_element in enumerate(element_x_m):
            times[index] = homogeneous_one_way_time(
                float(x_element), pixel_x_m, pixel_z_m, specimen_velocity_m_s
            )
        return times
    if element_z_m is None or element_z_m.shape != element_x_m.shape:
        raise ValueError("element_z_m is required for planar wedge propagation")
    for index, (x_element, z_element) in enumerate(zip(element_x_m, element_z_m)):
        times[index] = planar_interface_time(
            float(x_element),
            float(z_element),
            pixel_x_m,
            pixel_z_m,
            wedge_velocity_m_s,
            specimen_velocity_m_s,
        )
    return times


def _finish_result(
    coherent_sum: NDArray[np.complex64],
    energy_sum: NDArray[np.float64],
    channels: int,
    grid: ImageGrid,
    use_coherence: bool,
    floor_db: float,
) -> BeamformResult:
    raw_amplitude = np.abs(coherent_sum).astype(np.float32)
    coherence: NDArray[np.float32] | None = None
    if use_coherence:
        denominator = channels * energy_sum
        coherence = np.divide(
            raw_amplitude.astype(np.float64) ** 2,
            denominator,
            out=np.zeros_like(denominator),
            where=denominator > np.finfo(float).tiny,
        ).astype(np.float32)
        image = (coherent_sum * coherence).astype(np.complex64)
    else:
        image = coherent_sum
    amplitude = np.abs(image).astype(np.float32)
    return BeamformResult(
        image=image,
        amplitude=amplitude,
        db=normalized_db(amplitude, floor_db),
        coherence_factor=coherence,
        x_m=np.asarray(grid.x_m, dtype=np.float64),
        z_m=np.asarray(grid.z_m, dtype=np.float64),
        contributing_channels=channels,
    )


def tfm_fmc(
    data: FmcData,
    grid: ImageGrid,
    use_coherence: bool = False,
    floor_db: float = -60.0,
    max_tx_rx_separation: int | None = None,
    element_z_m: NDArray[np.floating] | None = None,
    wedge_velocity_m_s: float | None = None,
) -> BeamformResult:
    """Delay-and-sum total focusing method for FMC data.

    With wedge_velocity_m_s omitted, elements lie on the specimen surface. If
    provided, element_z_m gives each element coordinate above a flat z=0
    interface (negative inside the wedge), and Fermat/Snell travel times are used.
    """
    rf = analytic_signal(data.rf)
    pixel_x, pixel_z = grid.mesh
    one_way = _travel_times(
        data.element_x_m,
        pixel_x,
        pixel_z,
        data.velocity_m_s,
        element_z_m,
        wedge_velocity_m_s,
    )
    coherent = np.zeros(pixel_x.shape, dtype=np.complex64)
    energy = np.zeros(pixel_x.shape, dtype=np.float64)
    channels = 0
    for tx in range(rf.shape[0]):
        for rx in range(rf.shape[1]):
            if max_tx_rx_separation is not None and abs(tx - rx) > max_tx_rx_separation:
                continue
            sample_position = (
                (one_way[tx] + one_way[rx] - data.time_offset_s) * data.sample_rate_hz
            )
            values = sample_trace_linear(rf[tx, rx], sample_position).astype(np.complex64)
            coherent += values
            if use_coherence:
                energy += np.abs(values).astype(np.float64) ** 2
            channels += 1
    if channels == 0:
        raise ValueError("no FMC channels selected")
    return _finish_result(coherent, energy, channels, grid, use_coherence, floor_db)


def compound_pwi(
    data: PwiData,
    grid: ImageGrid,
    use_coherence: bool = False,
    floor_db: float = -60.0,
) -> BeamformResult:
    """Coherent plane-wave imaging using the recorded steering angles."""
    rf = analytic_signal(data.rf)
    pixel_x, pixel_z = grid.mesh
    receive_times = _travel_times(
        data.element_x_m,
        pixel_x,
        pixel_z,
        data.velocity_m_s,
        element_z_m=None,
        wedge_velocity_m_s=None,
    )
    coherent = np.zeros(pixel_x.shape, dtype=np.complex64)
    energy = np.zeros(pixel_x.shape, dtype=np.float64)
    channels = 0
    for event, angle in enumerate(data.angles_rad):
        surface_delays = data.element_x_m * np.sin(angle) / data.velocity_m_s
        delay_reference = float(np.min(surface_delays))
        transmit_time = (
            pixel_x * np.sin(angle) + pixel_z * np.cos(angle)
        ) / data.velocity_m_s - delay_reference
        for rx in range(rf.shape[1]):
            sample_position = (
                (transmit_time + receive_times[rx] - data.time_offset_s)
                * data.sample_rate_hz
            )
            values = sample_trace_linear(rf[event, rx], sample_position).astype(np.complex64)
            coherent += values
            if use_coherence:
                energy += np.abs(values).astype(np.float64) ** 2
            channels += 1
    return _finish_result(coherent, energy, channels, grid, use_coherence, floor_db)
