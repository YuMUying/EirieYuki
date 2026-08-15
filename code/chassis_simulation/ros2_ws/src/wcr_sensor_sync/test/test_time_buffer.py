import math

import numpy as np
import pytest

from wcr_sensor_sync.time_buffer import (
    ImuSample,
    MotionSample,
    RailSample,
    TimeBuffer,
    interpolate_imu,
    interpolate_motion,
    interpolate_rail,
    nearest_offset_s,
    slerp,
)


def test_buffer_brackets_target_timestamp():
    buffer = TimeBuffer[float]()
    buffer.append(1_000_000_000, 1.0)
    buffer.append(2_000_000_000, 2.0)
    before, after, fraction = buffer.bracket(1_250_000_000)
    assert before.value == 1.0 and after.value == 2.0
    assert math.isclose(fraction, 0.25)
    assert math.isclose(nearest_offset_s(1_250_000_000, before.stamp_ns, after.stamp_ns), 0.25)


def test_out_of_order_timestamp_is_rejected():
    buffer = TimeBuffer[int]()
    buffer.append(10, 1)
    with pytest.raises(ValueError):
        buffer.append(9, 2)


def test_large_clock_jump_resets_buffer():
    buffer = TimeBuffer[int](maximum_age_s=1.0)
    buffer.append(5_000_000_000, 1)
    buffer.append(1_000_000_000, 2)
    before, after, fraction = buffer.bracket(1_000_000_000)
    assert before.value == 2 and after.value == 2
    assert fraction == 0.0


def test_motion_interpolation_uses_slerp():
    first = MotionSample(np.zeros(3), np.array([0, 0, 0, 1]), np.zeros(3), np.zeros(3))
    second = MotionSample(np.array([2, 0, 0]), np.array([0, 0, 1, 0]), np.ones(3), np.ones(3))
    middle = interpolate_motion(first, second, 0.5)
    np.testing.assert_allclose(middle.position, [1, 0, 0])
    assert math.isclose(np.linalg.norm(middle.orientation_xyzw), 1.0)
    np.testing.assert_allclose(middle.orientation_xyzw, [0, 0, math.sqrt(0.5), math.sqrt(0.5)])


def test_imu_interpolation_is_finite():
    first = ImuSample(np.array([0, 0, 0, 1]), np.zeros(3), np.zeros(3))
    second = ImuSample(np.array([0, 0, 0, 1]), np.ones(3), np.full(3, 2.0))
    sample = interpolate_imu(first, second, 0.25)
    np.testing.assert_allclose(sample.angular_velocity, np.full(3, 0.25))
    np.testing.assert_allclose(sample.linear_acceleration, np.full(3, 0.5))


def test_slerp_handles_quaternion_sign_equivalence():
    np.testing.assert_allclose(slerp([0, 0, 0, 1], [0, 0, 0, -1], 0.5), [0, 0, 0, 1])


def test_rail_position_is_interpolated_but_switches_use_nearest_sample():
    first = RailSample(0.0, 0.0, 0.01, True, True, False, False, False, False, "")
    second = RailSample(0.02, 0.01, 0.03, True, True, True, False, False, False, "")
    sample = interpolate_rail(first, second, 0.25)
    assert math.isclose(sample.position_m, 0.005)
    assert not sample.moving
