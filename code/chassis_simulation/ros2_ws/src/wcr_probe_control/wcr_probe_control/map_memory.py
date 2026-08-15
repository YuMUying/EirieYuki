from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import math
import os
from pathlib import Path
import tempfile
from typing import Iterable

from .seam_tracking import MotionState, Point2, SeamCandidate, transform_points


@dataclass
class MappedSeam:
    id: str
    points: list[Point2]
    confidence: float
    observation_count: int
    first_seen_ns: int
    last_seen_ns: int


@dataclass
class StoredObstacle:
    id: str
    shape: int
    center_x: float
    center_y: float
    center_z: float
    radius: float
    width: float
    height: float
    yaw: float


class EnvironmentMemory:
    def __init__(
        self,
        association_distance_m: float = 0.08,
        association_angle_rad: float = math.radians(30.0),
        maximum_points_per_seam: int = 80,
    ) -> None:
        if association_distance_m <= 0.0:
            raise ValueError("association distance must be positive")
        if not 0.0 < association_angle_rad <= math.pi / 2.0:
            raise ValueError("association angle must be in (0, pi/2]")
        if maximum_points_per_seam < 2:
            raise ValueError("maximum seam points must be at least two")
        self.association_distance_m = association_distance_m
        self.association_angle_rad = association_angle_rad
        self.maximum_points_per_seam = maximum_points_per_seam
        self.seams: dict[str, MappedSeam] = {}
        self.obstacles: dict[str, StoredObstacle] = {}
        self.revision = 0
        self._next_seam_index = 1

    @staticmethod
    def _principal_angle(points: Iterable[Point2]) -> float:
        values = list(points)
        if len(values) < 2:
            return 0.0
        mean_x = sum(x for x, _ in values) / len(values)
        mean_y = sum(y for _, y in values) / len(values)
        xx = sum((x - mean_x) ** 2 for x, _ in values)
        xy = sum((x - mean_x) * (y - mean_y) for x, y in values)
        yy = sum((y - mean_y) ** 2 for _, y in values)
        return 0.5 * math.atan2(2.0 * xy, xx - yy)

    @staticmethod
    def _angle_difference(first: float, second: float) -> float:
        difference = abs((first - second + math.pi) % (2.0 * math.pi) - math.pi)
        return min(difference, abs(math.pi - difference))

    @staticmethod
    def _mean_nearest_distance(first: Iterable[Point2], second: Iterable[Point2]) -> float:
        source = list(first)
        target = list(second)
        if not source or not target:
            return math.inf
        return sum(
            min(math.hypot(x - tx, y - ty) for tx, ty in target)
            for x, y in source
        ) / len(source)

    def _compress_points(self, points: Iterable[Point2]) -> list[Point2]:
        values = list(points)
        if len(values) <= self.maximum_points_per_seam:
            return values
        angle = self._principal_angle(values)
        axis_x, axis_y = math.cos(angle), math.sin(angle)
        values.sort(key=lambda point: point[0] * axis_x + point[1] * axis_y)
        result = []
        for index in range(self.maximum_points_per_seam):
            start = index * len(values) // self.maximum_points_per_seam
            end = (index + 1) * len(values) // self.maximum_points_per_seam
            bucket = values[start:max(start + 1, end)]
            result.append(
                (
                    sum(x for x, _ in bucket) / len(bucket),
                    sum(y for _, y in bucket) / len(bucket),
                )
            )
        return result

    def observe_seams(
        self,
        candidates: Iterable[SeamCandidate],
        motion: MotionState,
        stamp_ns: int,
    ) -> dict[str, str]:
        associations: dict[str, str] = {}
        changed = False
        known_seam_ids = tuple(self.seams)
        used_seam_ids: set[str] = set()
        for candidate in candidates:
            if not candidate.valid or len(candidate.points_base) < 2:
                continue
            points = transform_points(candidate.points_base, motion)
            observation_angle = self._principal_angle(points)
            seam_id = ""
            best_distance = math.inf
            for known_id in known_seam_ids:
                if known_id in used_seam_ids:
                    continue
                seam = self.seams[known_id]
                if self._angle_difference(
                    observation_angle, self._principal_angle(seam.points)
                ) > self.association_angle_rad:
                    continue
                # Compare the current camera footprint with the accumulated map.
                # A symmetric distance grows as a seam map gets longer and would
                # eventually split one physical seam into several map IDs.
                distance = self._mean_nearest_distance(points, seam.points)
                if distance <= self.association_distance_m and distance < best_distance:
                    seam_id = known_id
                    best_distance = distance
            if not seam_id:
                seam_id = f"seam_{self._next_seam_index:06d}"
                self._next_seam_index += 1
                self.seams[seam_id] = MappedSeam(
                    seam_id,
                    self._compress_points(points),
                    candidate.confidence,
                    1,
                    stamp_ns,
                    stamp_ns,
                )
            else:
                seam = self.seams[seam_id]
                previous_count = seam.observation_count
                seam.points = self._compress_points([*seam.points, *points])
                seam.confidence = (
                    seam.confidence * previous_count + candidate.confidence
                ) / (previous_count + 1)
                seam.observation_count += 1
                seam.last_seen_ns = max(seam.last_seen_ns, stamp_ns)
            used_seam_ids.add(seam_id)
            associations[candidate.observation_id] = seam_id
            changed = True
        if changed:
            self.revision += 1
        return associations

    def upsert_obstacle(self, obstacle: StoredObstacle) -> None:
        if not obstacle.id:
            raise ValueError("obstacle id cannot be empty")
        values = (
            obstacle.center_x,
            obstacle.center_y,
            obstacle.center_z,
            obstacle.radius,
            obstacle.width,
            obstacle.height,
            obstacle.yaw,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("obstacle values must be finite")
        if obstacle.shape == 0 and obstacle.radius <= 0.0:
            raise ValueError("circle obstacle radius must be positive")
        if obstacle.shape == 1 and (
            obstacle.width <= 0.0 or obstacle.height <= 0.0
        ):
            raise ValueError("rectangle obstacle dimensions must be positive")
        if obstacle.shape not in (0, 1):
            raise ValueError("unknown obstacle shape")
        self.obstacles[obstacle.id] = obstacle
        self.revision += 1

    def remove_obstacle(self, obstacle_id: str) -> None:
        if self.obstacles.pop(obstacle_id, None) is not None:
            self.revision += 1

    def clear_obstacles(self) -> None:
        if self.obstacles:
            self.obstacles.clear()
            self.revision += 1

    def clear(self) -> None:
        if self.seams or self.obstacles:
            self.seams.clear()
            self.obstacles.clear()
            self._next_seam_index = 1
            self.revision += 1

    def to_mapping(self) -> dict:
        return {
            "version": 1,
            "revision": self.revision,
            "next_seam_index": self._next_seam_index,
            "seams": [asdict(seam) for seam in self.seams.values()],
            "obstacles": [asdict(obstacle) for obstacle in self.obstacles.values()],
        }

    def save(self, path: str | Path) -> None:
        destination = Path(path).expanduser()
        destination.parent.mkdir(parents=True, exist_ok=True)
        handle, temporary_name = tempfile.mkstemp(
            prefix=destination.name + ".", suffix=".tmp", dir=destination.parent
        )
        try:
            with os.fdopen(handle, "w", encoding="utf-8") as temporary:
                json.dump(self.to_mapping(), temporary, indent=2, allow_nan=False)
                temporary.flush()
                os.fsync(temporary.fileno())
            os.replace(temporary_name, destination)
        finally:
            if os.path.exists(temporary_name):
                os.unlink(temporary_name)

    def load(self, path: str | Path) -> bool:
        source = Path(path).expanduser()
        if not source.is_file():
            return False
        payload = json.loads(source.read_text(encoding="utf-8"))
        if payload.get("version") != 1:
            raise ValueError("unsupported environment map version")
        self.seams = {
            item["id"]: MappedSeam(
                item["id"],
                [tuple(point) for point in item["points"]],
                float(item["confidence"]),
                int(item["observation_count"]),
                int(item["first_seen_ns"]),
                int(item["last_seen_ns"]),
            )
            for item in payload.get("seams", [])
        }
        self.obstacles = {
            item["id"]: StoredObstacle(**item)
            for item in payload.get("obstacles", [])
        }
        self.revision = int(payload.get("revision", 0))
        self._next_seam_index = max(
            int(payload.get("next_seam_index", 1)),
            max(
                (
                    int(seam_id.rsplit("_", 1)[-1]) + 1
                    for seam_id in self.seams
                    if seam_id.rsplit("_", 1)[-1].isdigit()
                ),
                default=1,
            ),
        )
        return True
