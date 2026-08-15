from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable


Point2 = tuple[float, float]


@dataclass(frozen=True)
class MotionState:
    x_m: float
    y_m: float
    yaw_rad: float
    velocity_x_m_s: float
    velocity_y_m_s: float


@dataclass(frozen=True)
class SeamCandidate:
    observation_id: str
    points_base: tuple[Point2, ...]
    confidence: float
    rail_position_at_capture_m: float
    valid: bool = True


@dataclass(frozen=True)
class SelectionConfig:
    minimum_points: int = 5
    minimum_confidence: float = 0.65
    minimum_seam_separation_m: float = 0.015
    minimum_motion_speed_m_s: float = 0.01
    lookahead_distance_m: float = 0.08
    maximum_lateral_error_m: float = 0.10
    probe_center_y_at_reference_m: float = 0.0
    confidence_weight: float = 0.20
    heading_weight: float = 0.32
    lateral_weight: float = 0.24
    lookahead_weight: float = 0.14
    continuity_weight: float = 0.10
    switch_score_margin: float = 0.12

    def __post_init__(self) -> None:
        positive = (
            self.minimum_points,
            self.minimum_seam_separation_m,
            self.minimum_motion_speed_m_s,
            self.lookahead_distance_m,
            self.maximum_lateral_error_m,
        )
        if any(value <= 0 for value in positive):
            raise ValueError("selection distance/count limits must be positive")
        if not 0.0 <= self.minimum_confidence <= 1.0:
            raise ValueError("minimum confidence must be in [0, 1]")
        if self.switch_score_margin < 0.0:
            raise ValueError("switch score margin cannot be negative")


@dataclass(frozen=True)
class ScoredCandidate:
    candidate: SeamCandidate
    mapped_seam_id: str
    score: float
    lateral_error_m: float
    heading_score: float
    lookahead_score: float


@dataclass(frozen=True)
class SelectionDecision:
    selected: ScoredCandidate | None
    candidate_count: int
    switched: bool
    reason: str


def _finite_points(points: Iterable[Point2]) -> tuple[Point2, ...]:
    return tuple(
        (float(x), float(y))
        for x, y in points
        if math.isfinite(x) and math.isfinite(y)
    )


def transform_points(points: Iterable[Point2], pose: MotionState) -> tuple[Point2, ...]:
    cosine = math.cos(pose.yaw_rad)
    sine = math.sin(pose.yaw_rad)
    return tuple(
        (
            pose.x_m + cosine * x - sine * y,
            pose.y_m + sine * x + cosine * y,
        )
        for x, y in points
    )


def _principal_direction(points: tuple[Point2, ...]) -> Point2:
    if len(points) < 2:
        return (1.0, 0.0)
    mean_x = sum(point[0] for point in points) / len(points)
    mean_y = sum(point[1] for point in points) / len(points)
    covariance_xx = sum((x - mean_x) ** 2 for x, _ in points)
    covariance_xy = sum((x - mean_x) * (y - mean_y) for x, y in points)
    covariance_yy = sum((y - mean_y) ** 2 for _, y in points)
    angle = 0.5 * math.atan2(2.0 * covariance_xy, covariance_xx - covariance_yy)
    return (math.cos(angle), math.sin(angle))


def _median(values: list[float]) -> float:
    ordered = sorted(values)
    middle = len(ordered) // 2
    if len(ordered) % 2:
        return ordered[middle]
    return 0.5 * (ordered[middle - 1] + ordered[middle])


def _mean_nearest_distance(first: tuple[Point2, ...], second: tuple[Point2, ...]) -> float:
    if not first or not second:
        return math.inf

    def directed(source: tuple[Point2, ...], target: tuple[Point2, ...]) -> float:
        return sum(
            min(math.hypot(x - tx, y - ty) for tx, ty in target)
            for x, y in source
        ) / len(source)

    return 0.5 * (directed(first, second) + directed(second, first))


def distinct_candidates(
    candidates: Iterable[SeamCandidate], config: SelectionConfig
) -> list[SeamCandidate]:
    usable = []
    for candidate in candidates:
        points = _finite_points(candidate.points_base)
        if (
            candidate.valid
            and len(points) >= config.minimum_points
            and math.isfinite(candidate.confidence)
            and candidate.confidence >= config.minimum_confidence
            and math.isfinite(candidate.rail_position_at_capture_m)
        ):
            usable.append(
                SeamCandidate(
                    candidate.observation_id,
                    points,
                    min(1.0, max(0.0, candidate.confidence)),
                    candidate.rail_position_at_capture_m,
                    True,
                )
            )
    usable.sort(key=lambda item: item.confidence, reverse=True)
    result: list[SeamCandidate] = []
    for candidate in usable:
        if all(
            _mean_nearest_distance(candidate.points_base, accepted.points_base)
            >= config.minimum_seam_separation_m
            for accepted in result
        ):
            result.append(candidate)
    return result


def _travel_direction(motion: MotionState, config: SelectionConfig) -> Point2:
    speed = math.hypot(motion.velocity_x_m_s, motion.velocity_y_m_s)
    if speed < config.minimum_motion_speed_m_s:
        return (1.0, 0.0)
    return (motion.velocity_x_m_s / speed, motion.velocity_y_m_s / speed)


def score_candidate(
    candidate: SeamCandidate,
    mapped_seam_id: str,
    motion: MotionState,
    previous_mapped_seam_id: str,
    config: SelectionConfig,
) -> ScoredCandidate:
    direction_x, direction_y = _travel_direction(motion, config)
    tangent_x, tangent_y = _principal_direction(candidate.points_base)
    heading_score = abs(tangent_x * direction_x + tangent_y * direction_y)

    along = [x * direction_x + y * direction_y for x, y in candidate.points_base]
    target_index = min(
        range(len(along)),
        key=lambda index: abs(along[index] - config.lookahead_distance_m),
    )
    target_along = along[target_index]
    probe_y = (
        config.probe_center_y_at_reference_m
        + candidate.rail_position_at_capture_m
    )
    lateral_error = _median([point[1] for point in candidate.points_base]) - probe_y
    lateral_distance = abs(lateral_error)
    lateral_score = math.exp(-lateral_distance / config.maximum_lateral_error_m)
    lookahead_score = math.exp(
        -abs(target_along - config.lookahead_distance_m)
        / config.lookahead_distance_m
    )
    continuity_score = float(
        bool(mapped_seam_id) and mapped_seam_id == previous_mapped_seam_id
    )
    score = (
        config.confidence_weight * candidate.confidence
        + config.heading_weight * heading_score
        + config.lateral_weight * lateral_score
        + config.lookahead_weight * lookahead_score
        + config.continuity_weight * continuity_score
    )
    if abs(lateral_error) > config.maximum_lateral_error_m:
        score -= min(1.0, abs(lateral_error) / config.maximum_lateral_error_m - 1.0)
    return ScoredCandidate(
        candidate,
        mapped_seam_id,
        score,
        lateral_error,
        heading_score,
        lookahead_score,
    )


def select_candidate(
    candidates: Iterable[SeamCandidate],
    mapped_ids: dict[str, str],
    motion: MotionState,
    previous_mapped_seam_id: str,
    config: SelectionConfig,
) -> SelectionDecision:
    distinct = distinct_candidates(candidates, config)
    if not distinct:
        return SelectionDecision(None, 0, False, "no_valid_candidate")
    scored = [
        score_candidate(
            candidate,
            mapped_ids.get(candidate.observation_id, ""),
            motion,
            previous_mapped_seam_id,
            config,
        )
        for candidate in distinct
    ]
    scored.sort(key=lambda item: item.score, reverse=True)
    best = scored[0]
    previous = next(
        (
            item
            for item in scored
            if item.mapped_seam_id
            and item.mapped_seam_id == previous_mapped_seam_id
        ),
        None,
    )
    if previous is not None and best.mapped_seam_id != previous_mapped_seam_id:
        if best.score < previous.score + config.switch_score_margin:
            return SelectionDecision(previous, len(scored), False, "continuity_hysteresis")
    switched = bool(
        previous_mapped_seam_id
        and best.mapped_seam_id
        and best.mapped_seam_id != previous_mapped_seam_id
    )
    reason = "single_candidate" if len(scored) == 1 else "state_aware_score"
    return SelectionDecision(best, len(scored), switched, reason)
