from pathlib import Path

import h5py
import numpy as np

from ultrasonic_imaging.io import load_tofd_mat, load_tofd_npz, save_tofd_npz
from ultrasonic_imaging.simulation import simulate_tofd


def test_tofd_npz_roundtrip(tmp_path: Path) -> None:
    original = simulate_tofd(scan_start_m=-0.002, scan_end_m=0.002, scan_step_m=0.001, sample_count=1024)
    path = save_tofd_npz(tmp_path / "tofd.npz", original)
    loaded = load_tofd_npz(path)
    np.testing.assert_allclose(loaded.rf, original.rf)
    np.testing.assert_allclose(loaded.scan_positions_m, original.scan_positions_m)
    assert loaded.probe_center_spacing_m == original.probe_center_spacing_m
    assert loaded.metadata["measured_data"] is False


def test_load_compact_matlab_tofd_archive(tmp_path: Path) -> None:
    mat_path = tmp_path / "tofd_synthetic_scan.mat"
    with h5py.File(mat_path, "w") as archive:
        archive.create_dataset("rf_int16", data=np.arange(12, dtype=np.int16).reshape(4, 3))
        archive.create_dataset("time_s", data=np.arange(4, dtype=np.float64)[:, None] / 100e6)
        archive.create_dataset("scan_x_m", data=np.array([[-0.001], [0.0], [0.001]]))
    (tmp_path / "simulation_config.json").write_text(
        '{"sampling_rate_hz": 100000000, "longitudinal_velocity_m_s": 5900, ' 
        '"probe_center_spacing_m": 0.036, "plate_thickness_m": 0.025, ' 
        '"center_frequency_hz": 5000000, "total_wedge_delay_s": 8e-7}',
        encoding="utf-8",
    )
    loaded = load_tofd_mat(mat_path)
    assert loaded.rf.shape == (3, 4)
    assert loaded.metadata["travel_time_geometry"] == "inline"
    assert loaded.metadata["common_delay_s"] == 8e-7
    np.testing.assert_allclose(loaded.scan_positions_m, [-0.001, 0.0, 0.001])
