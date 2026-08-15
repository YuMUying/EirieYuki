import numpy as np

from ultrasonic_imaging.beamforming import tfm_fmc
from ultrasonic_imaging.geometry import linear_element_positions
from ultrasonic_imaging.models import FmcData, ImageGrid


def test_tfm_localizes_point_reflector() -> None:
    count = 8
    samples = 1024
    sample_rate = 40e6
    velocity = 5900.0
    elements = linear_element_positions(count, 0.001)
    target_x = 0.001
    target_z = 0.020
    rf = np.zeros((count, count, samples), dtype=np.float32)
    for tx in range(count):
        for rx in range(count):
            travel = (
                np.hypot(target_x - elements[tx], target_z)
                + np.hypot(target_x - elements[rx], target_z)
            ) / velocity
            center = int(round(travel * sample_rate))
            offsets = np.arange(-8, 9)
            t = offsets / sample_rate
            rf[tx, rx, center + offsets] = np.exp(-(t * 4e6) ** 2) * np.cos(2 * np.pi * 4e6 * t)
    data = FmcData(rf, sample_rate, velocity, elements)
    grid = ImageGrid(
        x_m=np.linspace(-0.004, 0.004, 33),
        z_m=np.linspace(0.015, 0.025, 41),
    )
    result = tfm_fmc(data, grid)
    peak_z, peak_x = np.unravel_index(np.argmax(result.amplitude), result.amplitude.shape)
    assert abs(result.x_m[peak_x] - target_x) <= 0.0005
    assert abs(result.z_m[peak_z] - target_z) <= 0.0005
