import math

from wcr_probe_control.seam_tracking import (
    MotionState,
    SeamCandidate,
    SelectionConfig,
    distinct_candidates,
    select_candidate,
)


def line(observation_id, start, end, y=0.0, confidence=0.9, rail=0.0):
    step = (end - start) / 5.0
    return SeamCandidate(
        observation_id,
        tuple((start + index * step, y) for index in range(6)),
        confidence,
        rail,
    )


def vertical(observation_id, x, start_y, end_y, confidence=0.9):
    step = (end_y - start_y) / 5.0
    return SeamCandidate(
        observation_id,
        tuple((x, start_y + index * step) for index in range(6)),
        confidence,
        0.0,
    )


def test_forward_motion_prefers_aligned_seam_ahead():
    config = SelectionConfig()
    forward = line("forward", 0.02, 0.14, y=0.01)
    crossing = vertical("crossing", 0.08, -0.06, 0.06, confidence=0.98)
    decision = select_candidate(
        [crossing, forward],
        {"forward": "seam_1", "crossing": "seam_2"},
        MotionState(0.0, 0.0, 0.0, 0.08, 0.0),
        "",
        config,
    )
    assert decision.selected is not None
    assert decision.selected.candidate.observation_id == "forward"
    assert decision.candidate_count == 2


def test_reverse_motion_prefers_candidate_behind_robot():
    config = SelectionConfig()
    ahead = line("ahead", 0.02, 0.14, y=0.0, confidence=0.95)
    behind = line("behind", -0.14, -0.02, y=0.0, confidence=0.85)
    decision = select_candidate(
        [ahead, behind],
        {"ahead": "seam_1", "behind": "seam_2"},
        MotionState(0.0, 0.0, 0.0, -0.08, 0.0),
        "",
        config,
    )
    assert decision.selected is not None
    assert decision.selected.candidate.observation_id == "behind"


def test_switch_hysteresis_keeps_current_mapped_seam():
    config = SelectionConfig(switch_score_margin=0.20)
    current = line("current", 0.02, 0.14, y=0.05, confidence=0.65)
    challenger = line("challenger", 0.02, 0.14, y=0.0, confidence=0.90)
    decision = select_candidate(
        [current, challenger],
        {"current": "seam_current", "challenger": "seam_other"},
        MotionState(0.0, 0.0, 0.0, 0.08, 0.0),
        "seam_current",
        config,
    )
    assert decision.selected is not None
    assert decision.selected.mapped_seam_id == "seam_current"
    assert decision.reason == "continuity_hysteresis"
    assert not decision.switched


def test_separation_threshold_collapses_duplicate_centerlines():
    config = SelectionConfig(minimum_seam_separation_m=0.02)
    first = line("first", 0.0, 0.15, y=0.0, confidence=0.9)
    duplicate = line("duplicate", 0.0, 0.15, y=0.005, confidence=0.8)
    distinct = line("distinct", 0.0, 0.15, y=0.05, confidence=0.7)
    result = distinct_candidates([duplicate, distinct, first], config)
    assert [candidate.observation_id for candidate in result] == ["first", "distinct"]


def test_lateral_error_uses_capture_time_rail_position():
    config = SelectionConfig(probe_center_y_at_reference_m=0.002)
    candidate = line("seam", 0.0, 0.15, y=0.032, rail=0.010)
    decision = select_candidate(
        [candidate],
        {"seam": "seam_1"},
        MotionState(0.0, 0.0, 0.0, 0.05, 0.0),
        "",
        config,
    )
    assert decision.selected is not None
    assert math.isclose(decision.selected.lateral_error_m, 0.020)
