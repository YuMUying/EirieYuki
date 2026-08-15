from __future__ import annotations

from bisect import bisect_left
from collections import deque
from dataclasses import dataclass
import math
from typing import Generic, TypeVar

import numpy as np


T = TypeVar("T")


@dataclass(frozen=True)
class TimedValue(Generic[T]):
    stamp_ns: int
    value: T


class TimeBuffer(Generic[T]):
    def __init__(self, maximum_age_s: float = 3.0) -> None:
        if maximum_age_s <= 0:
            raise ValueError("maximum buffer age must be positive")
        self.maximum_age_ns = int(maximum_age_s * 1e9)
        self._items: deque[TimedValue[T]] = deque()

    def append(self, stamp_ns: int, value: T) -> None:
        if self._items and stamp_ns <= self._items[-1].stamp_ns:
            if stamp_ns == self._items[-1].stamp_ns:
                self._items[-1] = TimedValue(stamp_ns, value)
                return
            if self._items[-1].stamp_ns - stamp_ns > self.maximum_age_ns:
                self._items.clear()
            else:
                raise ValueError("timestamps must be strictly increasing")
        self._items.append(TimedValue(stamp_ns, value))
        cutoff = stamp_ns - self.maximum_age_ns
        while len(self._items) > 2 and self._items[1].stamp_ns < cutoff:
            self._items.popleft()

    def bracket(self, stamp_ns: int) -> tuple[TimedValue[T], TimedValue[T], float]:
        if not self._items:
            raise LookupError("buffer is empty")
        stamps = [item.stamp_ns for item in self._items]
        index = bisect_left(stamps, stamp_ns)
        if index == 0:
            item = self._items[0]
            return item, item, 0.0
        if index == len(self._items):
            item = self._items[-1]
            return item, item, 0.0
        before, after = self._items[index - 1], self._items[index]
        fraction = (stamp_ns - before.stamp_ns) / (after.stamp_ns - before.stamp_ns)
        return before, after, float(fraction)


def nearest_offset_s(stamp_ns: int, before_ns: int, after_ns: int) -> float:
    return min(abs(stamp_ns - before_ns), abs(after_ns - stamp_ns)) / 1e9


def interpolate_vector(first, second, fraction: float) -> np.ndarray:
    first_array = np.asarray(first, dtype=np.float64)
    second_array = np.asarray(second, dtype=np.float64)
    return first_array + (second_array - first_array) * fraction


def normalize_quaternion(quaternion) -> np.ndarray:
    result = np.asarray(quaternion, dtype=np.float64)
    norm = float(np.linalg.norm(result))
    if result.shape != (4,) or norm < 1e-12 or not math.isfinite(norm):
        raise ValueError("invalid quaternion")
    return result / norm


def slerp(first, second, fraction: float) -> np.ndarray:
    first_q = normalize_quaternion(first)
    second_q = normalize_quaternion(second)
    dot = float(np.dot(first_q, second_q))
    if dot < 0.0:
        second_q = -second_q
        dot = -dot
    dot = float(np.clip(dot, -1.0, 1.0))
    if dot > 0.9995:
        return normalize_quaternion(first_q + fraction * (second_q - first_q))
    angle = math.acos(dot)
    sine = math.sin(angle)
    return (
        math.sin((1.0 - fraction) * angle) / sine * first_q
        + math.sin(fraction * angle) / sine * second_q
    )


@dataclass(frozen=True)
class MotionSample:
    position: np.ndarray
    orientation_xyzw: np.ndarray
    linear_velocity: np.ndarray
    angular_velocity: np.ndarray


def interpolate_motion(first: MotionSample, second: MotionSample, fraction: float) -> MotionSample:
    return MotionSample(
        interpolate_vector(first.position, second.position, fraction),
        slerp(first.orientation_xyzw, second.orientation_xyzw, fraction),
        interpolate_vector(first.linear_velocity, second.linear_velocity, fraction),
        interpolate_vector(first.angular_velocity, second.angular_velocity, fraction),
    )


@dataclass(frozen=True)
class ImuSample:
    orientation_xyzw: np.ndarray
    angular_velocity: np.ndarray
    linear_acceleration: np.ndarray


@dataclass(frozen=True)
class RailSample:
    position_m: float
    velocity_m_s: float
    target_position_m: float
    enabled: bool
    homed: bool
    moving: bool
    negative_limit: bool
    positive_limit: bool
    fault: bool
    fault_code: str


def interpolate_rail(first: RailSample, second: RailSample, fraction: float) -> RailSample:
    nearest = first if fraction < 0.5 else second
    return RailSample(
        float(first.position_m + (second.position_m - first.position_m) * fraction),
        float(first.velocity_m_s + (second.velocity_m_s - first.velocity_m_s) * fraction),
        float(
            first.target_position_m
            + (second.target_position_m - first.target_position_m) * fraction
        ),
        nearest.enabled,
        nearest.homed,
        nearest.moving,
        nearest.negative_limit,
        nearest.positive_limit,
        nearest.fault,
        nearest.fault_code,
    )


def interpolate_imu(first: ImuSample, second: ImuSample, fraction: float) -> ImuSample:
    return ImuSample(
        slerp(first.orientation_xyzw, second.orientation_xyzw, fraction),
        interpolate_vector(first.angular_velocity, second.angular_velocity, fraction),
        interpolate_vector(first.linear_acceleration, second.linear_acceleration, fraction),
    )
