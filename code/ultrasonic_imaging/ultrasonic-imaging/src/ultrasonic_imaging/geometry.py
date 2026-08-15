from __future__ import annotations

import numpy as np
from numpy.typing import ArrayLike, NDArray


def linear_element_positions(count: int, pitch_m: float, center_m: float = 0.0) -> NDArray[np.float64]:
    return center_m + (np.arange(count, dtype=np.float64) - (count - 1) / 2.0) * pitch_m


def homogeneous_one_way_time(
    element_x_m: float,
    pixel_x_m: NDArray[np.floating],
    pixel_z_m: NDArray[np.floating],
    velocity_m_s: float,
) -> NDArray[np.float64]:
    return np.hypot(pixel_x_m - element_x_m, pixel_z_m) / velocity_m_s


def planar_interface_time(
    element_x_m: float,
    element_z_m: float,
    pixel_x_m: NDArray[np.floating],
    pixel_z_m: NDArray[np.floating],
    wedge_velocity_m_s: float,
    specimen_velocity_m_s: float,
    iterations: int = 40,
) -> NDArray[np.float64]:
    """Fermat travel time through a flat z=0 wedge/specimen interface.

    The element must be at z<=0 and pixels at z>=0. Bisection solves Snell's
    stationary travel-time condition for each pixel without assuming an angle.
    """
    if element_z_m > 0 or np.any(pixel_z_m < 0):
        raise ValueError("planar interface requires element z<=0 and pixel z>=0")
    h = abs(float(element_z_m))
    lo = np.minimum(element_x_m, pixel_x_m).astype(np.float64, copy=True)
    hi = np.maximum(element_x_m, pixel_x_m).astype(np.float64, copy=True)
    for _ in range(iterations):
        mid = (lo + hi) * 0.5
        first = (mid - element_x_m) / (
            wedge_velocity_m_s * np.hypot(mid - element_x_m, h)
        )
        second = (mid - pixel_x_m) / (
            specimen_velocity_m_s * np.hypot(mid - pixel_x_m, pixel_z_m)
        )
        derivative = first + second
        move_left = derivative > 0
        hi = np.where(move_left, mid, hi)
        lo = np.where(move_left, lo, mid)
    crossing = (lo + hi) * 0.5
    return (
        np.hypot(crossing - element_x_m, h) / wedge_velocity_m_s
        + np.hypot(pixel_x_m - crossing, pixel_z_m) / specimen_velocity_m_s
    )


def tofd_time_from_depth(depth_m: ArrayLike, pcs_m: float, velocity_m_s: float) -> NDArray[np.float64]:
    depth = np.asarray(depth_m, dtype=np.float64)
    return 2.0 * np.sqrt((pcs_m * 0.5) ** 2 + depth**2) / velocity_m_s


def tofd_depth_from_time(time_s: ArrayLike, pcs_m: float, velocity_m_s: float) -> NDArray[np.float64]:
    time = np.asarray(time_s, dtype=np.float64)
    radicand = (velocity_m_s * time * 0.5) ** 2 - (pcs_m * 0.5) ** 2
    return np.sqrt(np.maximum(radicand, 0.0))


def tofd_pixel_time(
    scan_positions_m: NDArray[np.floating],
    pixel_scan_m: NDArray[np.floating],
    pixel_depth_m: NDArray[np.floating],
    pcs_m: float,
    velocity_m_s: float,
    geometry: str = "perpendicular",
) -> NDArray[np.float64]:
    along_scan = pixel_scan_m - scan_positions_m
    half_pcs = pcs_m * 0.5
    if geometry == "perpendicular":
        path_m = 2.0 * np.sqrt(along_scan**2 + pixel_depth_m**2 + half_pcs**2)
    elif geometry == "inline":
        tx_leg_m = np.hypot(along_scan + half_pcs, pixel_depth_m)
        rx_leg_m = np.hypot(along_scan - half_pcs, pixel_depth_m)
        path_m = tx_leg_m + rx_leg_m
    else:
        raise ValueError(f"unsupported TOFD geometry: {geometry}")
    return path_m / velocity_m_s
