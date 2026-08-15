from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np
from numpy.typing import NDArray


FloatArray = NDArray[np.floating]


@dataclass(slots=True)
class FmcData:
    rf: NDArray[np.number]
    sample_rate_hz: float
    velocity_m_s: float
    element_x_m: FloatArray
    time_offset_s: float = 0.0
    center_frequency_hz: float | None = None
    metadata: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if self.rf.ndim != 3 or self.rf.shape[0] != self.rf.shape[1]:
            raise ValueError("FMC RF must have shape [tx, rx, sample]")
        if self.element_x_m.shape != (self.rf.shape[0],):
            raise ValueError("element_x_m length must match FMC element count")
        if self.sample_rate_hz <= 0 or self.velocity_m_s <= 0:
            raise ValueError("sample rate and velocity must be positive")


@dataclass(slots=True)
class PwiData:
    rf: NDArray[np.number]
    sample_rate_hz: float
    velocity_m_s: float
    element_x_m: FloatArray
    angles_rad: FloatArray
    time_offset_s: float = 0.0
    center_frequency_hz: float | None = None
    metadata: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if self.rf.ndim != 3:
            raise ValueError("PWI RF must have shape [event, rx, sample]")
        if self.element_x_m.shape != (self.rf.shape[1],):
            raise ValueError("element_x_m length must match receiver count")
        if self.angles_rad.shape != (self.rf.shape[0],):
            raise ValueError("angles_rad length must match event count")


@dataclass(slots=True)
class TofdData:
    rf: NDArray[np.number]
    sample_rate_hz: float
    velocity_m_s: float
    scan_positions_m: FloatArray
    probe_center_spacing_m: float
    time_offset_s: float = 0.0
    plate_thickness_m: float | None = None
    center_frequency_hz: float | None = None
    metadata: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if self.rf.ndim != 2:
            raise ValueError("TOFD RF must have shape [scan_position, sample]")
        if self.scan_positions_m.shape != (self.rf.shape[0],):
            raise ValueError("scan position count must match trace count")
        if self.sample_rate_hz <= 0 or self.velocity_m_s <= 0:
            raise ValueError("sample rate and velocity must be positive")
        if self.probe_center_spacing_m <= 0:
            raise ValueError("probe center spacing must be positive")
        if self.rf.shape[1] < 2:
            raise ValueError("TOFD RF traces need at least two samples")


@dataclass(frozen=True, slots=True)
class ImageGrid:
    x_m: FloatArray
    z_m: FloatArray

    def __post_init__(self) -> None:
        if self.x_m.ndim != 1 or self.z_m.ndim != 1:
            raise ValueError("image axes must be one-dimensional")
        if self.x_m.size < 2 or self.z_m.size < 2:
            raise ValueError("image axes need at least two points")
        if np.any(np.diff(self.x_m) <= 0) or np.any(np.diff(self.z_m) <= 0):
            raise ValueError("image axes must be strictly increasing")

    @property
    def mesh(self) -> tuple[FloatArray, FloatArray]:
        return np.meshgrid(self.x_m, self.z_m, indexing="xy")
