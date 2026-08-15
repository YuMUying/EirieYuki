import math

import pytest

from wcr_probe_control.rail_control import (
    RailLimits,
    motion_allowed,
    target_from_alignment,
)


def limits():
    return RailLimits(-0.10, 0.10, 0.03, 0.20, 0.15, 0.65, 0.0005, 0.010)


def test_alignment_uses_capture_time_position():
    decision = target_from_alignment(0.004, 0.020, 0.9, 0.04, True, limits())
    assert decision.accepted
    assert math.isclose(decision.target_position_m, 0.024)


def test_alignment_rejects_stale_and_low_confidence():
    assert not target_from_alignment(0.004, 0.0, 0.9, 0.20, True, limits()).accepted
    assert not target_from_alignment(0.004, 0.0, 0.4, 0.02, True, limits()).accepted


def test_alignment_limits_correction_and_travel():
    decision = target_from_alignment(0.05, 0.095, 0.9, 0.01, True, limits())
    assert decision.accepted
    assert math.isclose(decision.target_position_m, 0.10)


def test_limit_switch_blocks_motion_toward_active_limit():
    allowed, reason = motion_allowed(True, False, True, False, -0.10, -0.11)
    assert not allowed
    assert reason == "negative_limit_active"
    assert motion_allowed(True, False, True, False, -0.10, -0.09)[0]


def test_invalid_limits_are_rejected():
    with pytest.raises(ValueError):
        RailLimits(0.1, -0.1, 0.03, 0.2, 0.1, 0.5, 0.0, 0.01)
