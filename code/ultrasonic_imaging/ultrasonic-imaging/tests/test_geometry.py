import numpy as np

from ultrasonic_imaging.geometry import (
    planar_interface_time,
    tofd_pixel_time,
    tofd_depth_from_time,
    tofd_time_from_depth,
)


def test_tofd_time_depth_roundtrip() -> None:
    depths = np.array([0.005, 0.012, 0.020, 0.030])
    times = tofd_time_from_depth(depths, pcs_m=0.060, velocity_m_s=5890.0)
    recovered = tofd_depth_from_time(times, pcs_m=0.060, velocity_m_s=5890.0)
    np.testing.assert_allclose(recovered, depths, rtol=1e-12, atol=1e-12)


def test_inline_tofd_geometry_at_scan_center_matches_depth_conversion() -> None:
    depth = np.array([[0.012]])
    inline = tofd_pixel_time(
        np.asarray(0.0), depth * 0.0, depth, 0.060, 5890.0, geometry="inline"
    )
    expected = tofd_time_from_depth(depth, 0.060, 5890.0)
    np.testing.assert_allclose(inline, expected)


def test_planar_interface_normal_incidence() -> None:
    x = np.array([[0.0]])
    z = np.array([[0.020]])
    time = planar_interface_time(0.0, -0.010, x, z, 2330.0, 5890.0)
    expected = 0.010 / 2330.0 + 0.020 / 5890.0
    np.testing.assert_allclose(time, expected, rtol=1e-10)
