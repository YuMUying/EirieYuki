from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class RailLimits:
    minimum_position_m: float
    maximum_position_m: float
    maximum_velocity_m_s: float
    maximum_acceleration_m_s2: float
    maximum_alignment_age_s: float
    minimum_confidence: float
    alignment_deadband_m: float
    maximum_correction_step_m: float

    def __post_init__(self) -> None:
        values = (
            self.minimum_position_m,
            self.maximum_position_m,
            self.maximum_velocity_m_s,
            self.maximum_acceleration_m_s2,
            self.maximum_alignment_age_s,
            self.minimum_confidence,
            self.alignment_deadband_m,
            self.maximum_correction_step_m,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("rail limits must be finite")
        if self.minimum_position_m >= self.maximum_position_m:
            raise ValueError("minimum rail position must be below maximum")
        if self.maximum_velocity_m_s <= 0 or self.maximum_acceleration_m_s2 <= 0:
            raise ValueError("rail velocity and acceleration limits must be positive")
        if self.maximum_alignment_age_s <= 0:
            raise ValueError("alignment age limit must be positive")
        if not 0 <= self.minimum_confidence <= 1:
            raise ValueError("minimum confidence must be in [0, 1]")
        if self.alignment_deadband_m < 0 or self.maximum_correction_step_m <= 0:
            raise ValueError("deadband/step limits are invalid")

    def clamp_position(self, position_m: float) -> float:
        return max(self.minimum_position_m, min(self.maximum_position_m, position_m))


@dataclass(frozen=True)
class RailDecision:
    accepted: bool
    target_position_m: float
    reason: str


def target_from_alignment(
    lateral_error_m: float,
    rail_position_at_capture_m: float,
    confidence: float,
    age_s: float,
    valid: bool,
    limits: RailLimits,
) -> RailDecision:
    values = (lateral_error_m, rail_position_at_capture_m, confidence, age_s)
    if not all(math.isfinite(value) for value in values):
        return RailDecision(False, rail_position_at_capture_m, "non_finite_alignment")
    if not valid:
        return RailDecision(False, rail_position_at_capture_m, "invalid_alignment")
    if age_s < 0 or age_s > limits.maximum_alignment_age_s:
        return RailDecision(False, rail_position_at_capture_m, "stale_alignment")
    if confidence < limits.minimum_confidence:
        return RailDecision(False, rail_position_at_capture_m, "low_confidence")
    if abs(lateral_error_m) <= limits.alignment_deadband_m:
        return RailDecision(True, limits.clamp_position(rail_position_at_capture_m), "deadband")
    correction = max(
        -limits.maximum_correction_step_m,
        min(limits.maximum_correction_step_m, lateral_error_m),
    )
    target = limits.clamp_position(rail_position_at_capture_m + correction)
    reason = "limited_correction" if correction != lateral_error_m else "tracking"
    return RailDecision(True, target, reason)


def motion_allowed(homed: bool, fault: bool, negative_limit: bool, positive_limit: bool,
                   current_position_m: float, target_position_m: float) -> tuple[bool, str]:
    if fault:
        return False, "rail_fault"
    if not homed:
        return False, "rail_not_homed"
    if negative_limit and target_position_m < current_position_m:
        return False, "negative_limit_active"
    if positive_limit and target_position_m > current_position_m:
        return False, "positive_limit_active"
    return True, "ok"
