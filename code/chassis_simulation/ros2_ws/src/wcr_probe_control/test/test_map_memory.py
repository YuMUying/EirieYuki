import json
import math

from wcr_probe_control.map_memory import EnvironmentMemory, StoredObstacle
from wcr_probe_control.seam_tracking import MotionState, SeamCandidate


def candidate(identifier, y, x_offset=0.0):
    return SeamCandidate(
        identifier,
        tuple((x_offset + index * 0.02, y) for index in range(6)),
        0.9,
        0.0,
    )


def test_same_frame_candidates_get_distinct_map_ids():
    memory = EnvironmentMemory(association_distance_m=0.08)
    associations = memory.observe_seams(
        [candidate("left", -0.03), candidate("right", 0.03)],
        MotionState(1.0, 2.0, 0.0, 0.1, 0.0),
        100,
    )
    assert associations["left"] != associations["right"]
    assert len(memory.seams) == 2


def test_repeated_observation_associates_and_transforms_to_odom():
    memory = EnvironmentMemory(association_distance_m=0.04)
    first = memory.observe_seams(
        [candidate("first", 0.01)],
        MotionState(1.0, 2.0, math.pi / 2.0, 0.1, 0.0),
        100,
    )
    second = memory.observe_seams(
        [candidate("second", 0.01, x_offset=-0.01)],
        MotionState(1.0, 2.01, math.pi / 2.0, 0.1, 0.0),
        200,
    )
    assert first["first"] == second["second"]
    seam = memory.seams[first["first"]]
    assert seam.observation_count == 2
    assert seam.last_seen_ns == 200
    assert all(x < 1.01 for x, _ in seam.points)


def test_short_camera_footprints_extend_one_long_mapped_seam():
    memory = EnvironmentMemory(association_distance_m=0.04)
    mapped_ids = []
    for index in range(12):
        associations = memory.observe_seams(
            [candidate(f"observation_{index}", 0.0)],
            MotionState(index * 0.08, 0.0, 0.0, 0.08, 0.0),
            100 + index,
        )
        mapped_ids.append(associations[f"observation_{index}"])
    assert len(set(mapped_ids)) == 1
    assert len(memory.seams) == 1
    assert next(iter(memory.seams.values())).observation_count == 12


def test_map_round_trip_preserves_seams_and_obstacles(tmp_path):
    path = tmp_path / "map.json"
    memory = EnvironmentMemory()
    memory.observe_seams(
        [candidate("seam", 0.0)],
        MotionState(0.0, 0.0, 0.0, 0.1, 0.0),
        123,
    )
    memory.upsert_obstacle(
        StoredObstacle("tank_nozzle", 0, 1.0, 2.0, 0.0, 0.1, 0.0, 0.0, 0.0)
    )
    memory.save(path)
    payload = json.loads(path.read_text(encoding="utf-8"))
    assert payload["version"] == 1

    restored = EnvironmentMemory()
    assert restored.load(path)
    assert set(restored.seams) == set(memory.seams)
    assert restored.obstacles["tank_nozzle"].radius == 0.1


def test_obstacles_require_valid_geometry():
    memory = EnvironmentMemory()
    try:
        memory.upsert_obstacle(
            StoredObstacle("bad", 0, 0.0, 0.0, 0.0, -0.1, 0.0, 0.0, 0.0)
        )
    except ValueError as error:
        assert "radius" in str(error)
    else:
        raise AssertionError("invalid obstacle was accepted")
