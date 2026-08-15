from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import h5py
import numpy as np
from scipy.io import loadmat

from .geometry import linear_element_positions
from .models import FmcData, PwiData, TofdData


def _read_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def _sampling_rate(metadata: dict[str, Any], time_s: np.ndarray | None = None) -> float:
    if time_s is not None and time_s.size > 1:
        return float(1.0 / np.median(np.diff(time_s)))
    return float(metadata["acquisition"]["sampling_rate_hz"])


def _element_positions(
    metadata: dict[str, Any], count: int, array_struct: Any | None = None
) -> np.ndarray:
    if array_struct is not None and hasattr(array_struct, "el_xc"):
        positions = np.asarray(array_struct.el_xc, dtype=np.float64).reshape(-1)
        if positions.size == count:
            return positions
    pitch = float(metadata["acquisition"]["element_pitch_m"])
    return linear_element_positions(count, pitch)


def load_paut_archive(metadata_path: str | Path) -> FmcData | PwiData:
    """Load one archived FMC/PWI dataset and normalize its axes."""
    metadata_path = Path(metadata_path).expanduser().resolve()
    metadata = _read_json(metadata_path)
    data_path = metadata_path.parent / metadata["data_file"]
    variable = metadata["matlab_variable"]
    mat = loadmat(data_path, squeeze_me=True, struct_as_record=False)

    if variable == "exp_data":
        exp_data = mat[variable]
        waveforms = np.asarray(exp_data.time_data)
        tx = np.asarray(exp_data.tx, dtype=np.int64).reshape(-1) - 1
        rx = np.asarray(exp_data.rx, dtype=np.int64).reshape(-1) - 1
        time_s = np.asarray(exp_data.time, dtype=np.float64).reshape(-1)
        rate = _sampling_rate(metadata, time_s)
        velocity = float(exp_data.ph_velocity)
        array_struct = exp_data.array
        center_frequency = float(array_struct.centre_freq)
        mode = metadata["acquisition"]["mode"]
        if mode == "FMC":
            count = int(metadata["acquisition"]["element_count"])
            rf = np.empty((count, count, waveforms.shape[0]), dtype=waveforms.dtype)
            rf[tx, rx, :] = waveforms.T
            return FmcData(
                rf=rf,
                sample_rate_hz=rate,
                velocity_m_s=velocity,
                element_x_m=_element_positions(metadata, count, array_struct),
                time_offset_s=float(time_s[0]),
                center_frequency_hz=center_frequency,
                metadata=metadata,
            )
        events = int(metadata["acquisition"]["transmit_event_count"])
        receivers = int(metadata["acquisition"]["element_count"])
        rf = np.empty((events, receivers, waveforms.shape[0]), dtype=waveforms.dtype)
        rf[tx, rx, :] = waveforms.T
        return PwiData(
            rf=rf,
            sample_rate_hz=rate,
            velocity_m_s=velocity,
            element_x_m=_element_positions(metadata, receivers, array_struct),
            angles_rad=np.asarray(exp_data.angles_tx, dtype=np.float64).reshape(-1),
            time_offset_s=float(time_s[0]),
            center_frequency_hz=center_frequency,
            metadata=metadata,
        )

    rf = np.asarray(mat[variable])
    stored_order = metadata["storage"]["stored_axis_order"]
    if stored_order == ["sample", "tx", "rx"]:
        rf = rf.transpose(1, 2, 0)
    elif stored_order != ["tx", "rx", "sample"]:
        raise ValueError(f"unsupported archived axis order: {stored_order}")
    count = rf.shape[0]
    acquisition = metadata["acquisition"]
    velocity = float(
        acquisition.get("estimated_longitudinal_velocity_m_s", acquisition.get("wave_velocity_m_s"))
    )
    return FmcData(
        rf=rf,
        sample_rate_hz=float(acquisition["sampling_rate_hz"]),
        velocity_m_s=velocity,
        element_x_m=_element_positions(metadata, count),
        time_offset_s=0.0,
        center_frequency_hz=float(acquisition["center_frequency_hz"]),
        metadata=metadata,
    )


def load_tofd_mat(path: str | Path, config_path: str | Path | None = None) -> TofdData:
    """Load the compact MATLAB v7.3 TOFD archive schema directly."""
    path = Path(path).expanduser().resolve()
    config_file = (
        path.with_name("simulation_config.json")
        if config_path is None
        else Path(config_path).expanduser().resolve()
    )
    config = _read_json(config_file)
    with h5py.File(path, "r") as archive:
        required = {"rf_int16", "time_s", "scan_x_m"}
        missing = sorted(required.difference(archive.keys()))
        if missing:
            raise ValueError(f"TOFD MAT is missing fields: {', '.join(missing)}")
        rf_stored = np.asarray(archive["rf_int16"])
        time_s = np.asarray(archive["time_s"], dtype=np.float64).reshape(-1)
        scan_x_m = np.asarray(archive["scan_x_m"], dtype=np.float64).reshape(-1)
    if rf_stored.shape == (time_s.size, scan_x_m.size):
        rf_stored = rf_stored.T
    elif rf_stored.shape != (scan_x_m.size, time_s.size):
        raise ValueError(
            f"TOFD MAT RF shape {rf_stored.shape} does not match "
            f"{scan_x_m.size} scans x {time_s.size} samples"
        )
    if time_s.size < 2 or np.any(np.diff(time_s) <= 0):
        raise ValueError("TOFD MAT time axis must be strictly increasing")
    if np.any(np.diff(scan_x_m) <= 0):
        raise ValueError("TOFD MAT scan axis must be strictly increasing")
    sample_rate_hz = float(1.0 / np.median(np.diff(time_s)))
    if not np.isclose(sample_rate_hz, float(config["sampling_rate_hz"]), rtol=1e-8):
        raise ValueError("TOFD MAT time axis differs from simulation configuration")
    metadata = {
        "source": "MATLAB compact synthetic TOFD archive",
        "source_path": str(path),
        "measured_data": False,
        "synthetic": True,
        "travel_time_geometry": "inline",
        "common_delay_s": float(config.get("total_wedge_delay_s", 0.0)),
        "simulation_config": config,
    }
    return TofdData(
        rf=rf_stored.astype(np.float32) / 32767.0,
        sample_rate_hz=sample_rate_hz,
        velocity_m_s=float(config["longitudinal_velocity_m_s"]),
        scan_positions_m=scan_x_m,
        probe_center_spacing_m=float(config["probe_center_spacing_m"]),
        time_offset_s=float(time_s[0]),
        plate_thickness_m=float(config["plate_thickness_m"]),
        center_frequency_hz=float(config["center_frequency_hz"]),
        metadata=metadata,
    )


def load_tofd(path: str | Path) -> TofdData:
    path = Path(path)
    if path.suffix.lower() == ".npz":
        return load_tofd_npz(path)
    if path.suffix.lower() == ".mat":
        return load_tofd_mat(path)
    raise ValueError("TOFD input must be .npz or .mat")


def save_tofd_npz(path: str | Path, data: TofdData) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        path,
        rf=np.asarray(data.rf),
        sample_rate_hz=np.float64(data.sample_rate_hz),
        velocity_m_s=np.float64(data.velocity_m_s),
        scan_positions_m=np.asarray(data.scan_positions_m, dtype=np.float64),
        probe_center_spacing_m=np.float64(data.probe_center_spacing_m),
        time_offset_s=np.float64(data.time_offset_s),
        plate_thickness_m=np.float64(data.plate_thickness_m or np.nan),
        center_frequency_hz=np.float64(data.center_frequency_hz or np.nan),
        metadata_json=json.dumps(data.metadata, ensure_ascii=True),
    )
    return path


def load_tofd_npz(path: str | Path) -> TofdData:
    path = Path(path).expanduser().resolve()
    with np.load(path, allow_pickle=False) as archive:
        required = {
            "rf",
            "sample_rate_hz",
            "velocity_m_s",
            "scan_positions_m",
            "probe_center_spacing_m",
        }
        missing = sorted(required.difference(archive.files))
        if missing:
            raise ValueError(f"TOFD NPZ is missing fields: {', '.join(missing)}")
        thickness = float(archive["plate_thickness_m"]) if "plate_thickness_m" in archive else np.nan
        frequency = float(archive["center_frequency_hz"]) if "center_frequency_hz" in archive else np.nan
        raw_metadata = str(archive["metadata_json"]) if "metadata_json" in archive else "{}"
        return TofdData(
            rf=np.asarray(archive["rf"]),
            sample_rate_hz=float(archive["sample_rate_hz"]),
            velocity_m_s=float(archive["velocity_m_s"]),
            scan_positions_m=np.asarray(archive["scan_positions_m"], dtype=np.float64),
            probe_center_spacing_m=float(archive["probe_center_spacing_m"]),
            time_offset_s=float(archive["time_offset_s"]) if "time_offset_s" in archive else 0.0,
            plate_thickness_m=None if np.isnan(thickness) else thickness,
            center_frequency_hz=None if np.isnan(frequency) else frequency,
            metadata=json.loads(raw_metadata),
        )


def write_json(path: str | Path, payload: dict[str, Any]) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, ensure_ascii=False)
        handle.write("\n")
    return path
