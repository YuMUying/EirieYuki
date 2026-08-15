import numpy as np

from ultrasonic_imaging.models import ImageGrid
from ultrasonic_imaging.simulation import simulate_tofd
from ultrasonic_imaging.tofd import (
    TofdTip,
    depth_corrected_scan,
    pair_tip_candidates,
    saft,
)


def test_tofd_simulation_and_saft_localize_upper_tip() -> None:
    data = simulate_tofd(
        scan_start_m=-0.025,
        scan_end_m=0.025,
        scan_step_m=0.001,
        sample_count=2048,
        noise_std=0.005,
    )
    depths = np.linspace(0.007, 0.025, 73)
    corrected = depth_corrected_scan(data, depths)
    assert corrected.amplitude.shape == (data.rf.shape[0], depths.size)
    grid = ImageGrid(
        x_m=np.linspace(-0.015, 0.015, 61),
        z_m=depths,
    )
    result = saft(data, grid, aperture_m=0.030)
    peak_z, peak_x = np.unravel_index(np.argmax(result.amplitude), result.amplitude.shape)
    assert abs(result.x_m[peak_x]) <= 0.001
    assert abs(result.z_m[peak_z] - 0.012) <= 0.0015


def test_pair_tip_candidates_sizes_vertical_crack() -> None:
    tips = [
        TofdTip(0.0002, 0.008, 1.0, 0.0),
        TofdTip(-0.0001, 0.016, 0.8, -2.0),
        TofdTip(0.010, 0.012, 0.7, -3.0),
    ]
    defects = pair_tip_candidates(tips, max_defects=1)
    assert len(defects) == 1
    assert abs(defects[0].height_m - 0.008) < 1e-12
    assert abs(defects[0].x_m - 0.00005) < 1e-12
