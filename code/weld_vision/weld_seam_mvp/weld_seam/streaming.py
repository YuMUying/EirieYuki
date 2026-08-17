from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
import threading
from typing import Generic, TypeVar


T = TypeVar("T")


class LatestValueSlot(Generic[T]):
    """Thread-safe, depth-one queue that drops stale work instead of adding latency."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._value: T | None = None
        self._sequence = 0
        self._consumed_sequence = 0
        self._dropped = 0

    def put(self, value: T) -> int:
        with self._lock:
            if self._value is not None and self._consumed_sequence < self._sequence:
                self._dropped += 1
            self._sequence += 1
            self._value = value
            return self._sequence

    def take(self) -> tuple[int, T] | None:
        with self._lock:
            if self._value is None or self._consumed_sequence == self._sequence:
                return None
            self._consumed_sequence = self._sequence
            return self._sequence, self._value

    @property
    def dropped(self) -> int:
        with self._lock:
            return self._dropped


class RateGate:
    def __init__(self, target_hz: float) -> None:
        if not math.isfinite(target_hz) or target_hz <= 0.0:
            raise ValueError("target_hz must be positive and finite")
        self.period_s = 1.0 / target_hz
        self._next_start_s: float | None = None

    def delay_s(self, now_s: float) -> float:
        if not math.isfinite(now_s):
            raise ValueError("now_s must be finite")
        if self._next_start_s is None:
            return 0.0
        return max(0.0, self._next_start_s - now_s)

    def mark_started(self, now_s: float) -> None:
        if not math.isfinite(now_s):
            raise ValueError("now_s must be finite")
        if self._next_start_s is None:
            self._next_start_s = now_s + self.period_s
            return
        self._next_start_s += self.period_s
        if self._next_start_s <= now_s:
            missed = math.floor((now_s - self._next_start_s) / self.period_s) + 1
            self._next_start_s += missed * self.period_s


@dataclass(frozen=True)
class LatencySnapshot:
    samples: int
    processing_mean_ms: float
    processing_p95_ms: float
    capture_age_mean_ms: float
    capture_age_p95_ms: float
    effective_hz: float


class RollingLatency:
    def __init__(self, capacity: int = 120) -> None:
        if capacity < 2:
            raise ValueError("capacity must be at least two")
        self._processing_ms: deque[float] = deque(maxlen=capacity)
        self._capture_age_ms: deque[float] = deque(maxlen=capacity)
        self._completion_s: deque[float] = deque(maxlen=capacity)

    def add(self, processing_s: float, capture_age_s: float, completion_s: float) -> None:
        values = (processing_s, capture_age_s, completion_s)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("latency values must be finite")
        self._processing_ms.append(max(0.0, processing_s) * 1000.0)
        self._capture_age_ms.append(max(0.0, capture_age_s) * 1000.0)
        self._completion_s.append(completion_s)

    @staticmethod
    def _percentile(values: deque[float], fraction: float) -> float:
        ordered = sorted(values)
        if not ordered:
            return 0.0
        position = fraction * (len(ordered) - 1)
        lower = int(math.floor(position))
        upper = int(math.ceil(position))
        if lower == upper:
            return ordered[lower]
        weight = position - lower
        return ordered[lower] * (1.0 - weight) + ordered[upper] * weight

    def snapshot(self) -> LatencySnapshot:
        count = len(self._processing_ms)
        effective_hz = 0.0
        if len(self._completion_s) >= 2:
            duration = self._completion_s[-1] - self._completion_s[0]
            if duration > 0.0:
                effective_hz = (len(self._completion_s) - 1) / duration
        return LatencySnapshot(
            samples=count,
            processing_mean_ms=(sum(self._processing_ms) / count if count else 0.0),
            processing_p95_ms=self._percentile(self._processing_ms, 0.95),
            capture_age_mean_ms=(sum(self._capture_age_ms) / count if count else 0.0),
            capture_age_p95_ms=self._percentile(self._capture_age_ms, 0.95),
            effective_hz=effective_hz,
        )


class TimedScalarBuffer:
    def __init__(self, maximum_age_s: float = 3.0) -> None:
        if maximum_age_s <= 0.0:
            raise ValueError("maximum_age_s must be positive")
        self.maximum_age_ns = int(maximum_age_s * 1e9)
        self._items: deque[tuple[int, float]] = deque()
        self._lock = threading.Lock()

    def add(self, stamp_ns: int, value: float) -> None:
        if stamp_ns < 0 or not math.isfinite(value):
            raise ValueError("timestamp and value must be valid")
        with self._lock:
            if self._items and stamp_ns < self._items[-1][0]:
                self._items.clear()
            if self._items and stamp_ns == self._items[-1][0]:
                self._items[-1] = (stamp_ns, value)
            else:
                self._items.append((stamp_ns, value))
            cutoff = stamp_ns - self.maximum_age_ns
            while len(self._items) > 2 and self._items[1][0] < cutoff:
                self._items.popleft()

    def interpolate(self, stamp_ns: int, maximum_offset_s: float) -> float | None:
        tolerance_ns = int(maximum_offset_s * 1e9)
        with self._lock:
            items = tuple(self._items)
        if not items:
            return None
        if stamp_ns <= items[0][0]:
            return items[0][1] if items[0][0] - stamp_ns <= tolerance_ns else None
        if stamp_ns >= items[-1][0]:
            return items[-1][1] if stamp_ns - items[-1][0] <= tolerance_ns else None
        for before, after in zip(items, items[1:]):
            if before[0] <= stamp_ns <= after[0]:
                fraction = (stamp_ns - before[0]) / (after[0] - before[0])
                return before[1] + (after[1] - before[1]) * fraction
        return None
