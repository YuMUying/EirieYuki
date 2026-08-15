from __future__ import annotations

import numpy as np
from numpy.typing import ArrayLike, NDArray
from scipy.signal import butter, hilbert, sosfiltfilt


def preprocess_rf(
    rf: ArrayLike,
    sample_rate_hz: float,
    band_hz: tuple[float, float] | None = None,
    remove_dc: bool = True,
    time_gain_db_per_s: float = 0.0,
) -> NDArray[np.float32]:
    data = np.asarray(rf, dtype=np.float32).copy()
    if remove_dc:
        data -= np.mean(data, axis=-1, keepdims=True)
    if band_hz is not None:
        low, high = band_hz
        nyquist = sample_rate_hz / 2.0
        if not 0 < low < high < nyquist:
            raise ValueError(f"band must satisfy 0 < low < high < Nyquist ({nyquist:g} Hz)")
        sos = butter(4, (low / nyquist, high / nyquist), btype="bandpass", output="sos")
        data = sosfiltfilt(sos, data, axis=-1).astype(np.float32, copy=False)
    if time_gain_db_per_s:
        t = np.arange(data.shape[-1], dtype=np.float64) / sample_rate_hz
        gain = np.power(10.0, time_gain_db_per_s * t / 20.0)
        data *= gain.astype(np.float32)
    return data


def analytic_signal(rf: ArrayLike) -> NDArray[np.complex64]:
    return hilbert(np.asarray(rf), axis=-1).astype(np.complex64)


def envelope(rf: ArrayLike) -> NDArray[np.float32]:
    values = np.asarray(rf)
    if np.iscomplexobj(values):
        return np.abs(values).astype(np.float32)
    return np.abs(hilbert(values, axis=-1)).astype(np.float32)


def normalized_db(values: ArrayLike, floor_db: float = -60.0) -> NDArray[np.float32]:
    amplitude = np.abs(np.asarray(values, dtype=np.complex128))
    peak = float(np.max(amplitude)) if amplitude.size else 0.0
    if peak <= np.finfo(float).tiny:
        return np.full(amplitude.shape, floor_db, dtype=np.float32)
    result = 20.0 * np.log10(np.maximum(amplitude / peak, 10.0 ** (floor_db / 20.0)))
    return np.maximum(result, floor_db).astype(np.float32)


def sample_trace_linear(trace: NDArray[np.number], sample_positions: NDArray[np.floating]) -> NDArray[np.number]:
    """Linearly sample one trace at an arbitrary coordinate array."""
    if trace.ndim != 1:
        raise ValueError("trace must be one-dimensional")
    n = trace.shape[0]
    i0 = np.floor(sample_positions).astype(np.int64)
    fraction = sample_positions - i0
    valid = (i0 >= 0) & (i0 < n - 1)
    i0_safe = np.clip(i0, 0, n - 2)
    a = trace[i0_safe]
    b = trace[i0_safe + 1]
    result = a * (1.0 - fraction) + b * fraction
    return np.where(valid, result, 0)
