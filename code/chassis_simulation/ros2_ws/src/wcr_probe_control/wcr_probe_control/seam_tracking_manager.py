from __future__ import annotations

from collections import deque
import math
from pathlib import Path

import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger
from wcr_planning_msgs.msg import (
    EnvironmentMap,
    MappedWeldSeam,
    Obstacle,
    ObstacleArray,
    ObstacleUpdate,
    ProbeAlignment,
    WeldSeamCandidateArray,
    WeldSeamSelection,
)

from .map_memory import EnvironmentMemory, StoredObstacle
from .seam_tracking import (
    MotionState,
    SeamCandidate,
    SelectionConfig,
    distinct_candidates,
    select_candidate,
)


def stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def set_stamp(stamp, value_ns: int) -> None:
    stamp.sec = int(value_ns // 1_000_000_000)
    stamp.nanosec = int(value_ns % 1_000_000_000)


def yaw_from_quaternion(quaternion) -> float:
    norm = math.sqrt(
        quaternion.x * quaternion.x
        + quaternion.y * quaternion.y
        + quaternion.z * quaternion.z
        + quaternion.w * quaternion.w
    )
    if norm < 1e-12:
        return 0.0
    x = quaternion.x / norm
    y = quaternion.y / norm
    z = quaternion.z / norm
    w = quaternion.w / norm
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class SeamTrackingManager(Node):
    def __init__(self) -> None:
        super().__init__("seam_tracking_manager")
        self.declare_parameter("candidate_topic", "weld_seam_candidates")
        self.declare_parameter("odometry_topic", "ins/odometry")
        self.declare_parameter("alignment_topic", "probe_alignment")
        self.declare_parameter("selection_topic", "weld_seam_selection")
        self.declare_parameter("environment_map_topic", "environment_map")
        self.declare_parameter("obstacle_observation_topic", "obstacles")
        self.declare_parameter("obstacle_update_topic", "obstacle_update")
        self.declare_parameter("planner_obstacles_topic", "mapped_obstacles")
        self.declare_parameter("map_path", "~/.ros/wcr_environment_map.json")
        self.declare_parameter("map_frame", "odom")
        self.declare_parameter("maximum_odometry_offset_s", 0.10)
        self.declare_parameter("odometry_buffer_age_s", 3.0)
        self.declare_parameter("map_save_period_s", 2.0)
        self.declare_parameter("minimum_candidate_points", 5)
        self.declare_parameter("minimum_candidate_confidence", 0.65)
        self.declare_parameter("minimum_seam_separation_m", 0.015)
        self.declare_parameter("minimum_motion_speed_m_s", 0.01)
        self.declare_parameter("selection_lookahead_m", 0.08)
        self.declare_parameter("maximum_lateral_error_m", 0.10)
        self.declare_parameter("probe_center_y_at_reference_m", 0.0)
        self.declare_parameter("switch_score_margin", 0.12)
        self.declare_parameter("map_association_distance_m", 0.04)
        self.declare_parameter("map_association_angle_deg", 30.0)
        self.declare_parameter("maximum_points_per_seam", 80)

        self.config = SelectionConfig(
            minimum_points=int(self.get_parameter("minimum_candidate_points").value),
            minimum_confidence=float(
                self.get_parameter("minimum_candidate_confidence").value
            ),
            minimum_seam_separation_m=float(
                self.get_parameter("minimum_seam_separation_m").value
            ),
            minimum_motion_speed_m_s=float(
                self.get_parameter("minimum_motion_speed_m_s").value
            ),
            lookahead_distance_m=float(
                self.get_parameter("selection_lookahead_m").value
            ),
            maximum_lateral_error_m=float(
                self.get_parameter("maximum_lateral_error_m").value
            ),
            probe_center_y_at_reference_m=float(
                self.get_parameter("probe_center_y_at_reference_m").value
            ),
            switch_score_margin=float(
                self.get_parameter("switch_score_margin").value
            ),
        )
        self.memory = EnvironmentMemory(
            association_distance_m=float(
                self.get_parameter("map_association_distance_m").value
            ),
            association_angle_rad=math.radians(
                float(self.get_parameter("map_association_angle_deg").value)
            ),
            maximum_points_per_seam=int(
                self.get_parameter("maximum_points_per_seam").value
            ),
        )
        self.map_path = Path(str(self.get_parameter("map_path").value)).expanduser()
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.maximum_odometry_offset_ns = int(
            float(self.get_parameter("maximum_odometry_offset_s").value) * 1e9
        )
        self.odometry_buffer_age_ns = int(
            float(self.get_parameter("odometry_buffer_age_s").value) * 1e9
        )
        self.motion_buffer: deque[tuple[int, MotionState]] = deque()
        self.active_mapped_seam_id = ""
        self.map_dirty = False

        try:
            if self.memory.load(self.map_path):
                self.get_logger().info(f"Loaded environment map: {self.map_path}")
        except (OSError, ValueError, TypeError) as error:
            self.get_logger().error(f"Environment map load failed: {error}")

        latched = QoSProfile(depth=1)
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.alignment_publisher = self.create_publisher(
            ProbeAlignment, str(self.get_parameter("alignment_topic").value), 10
        )
        self.selection_publisher = self.create_publisher(
            WeldSeamSelection, str(self.get_parameter("selection_topic").value), 10
        )
        self.map_publisher = self.create_publisher(
            EnvironmentMap,
            str(self.get_parameter("environment_map_topic").value),
            latched,
        )
        self.obstacle_publisher = self.create_publisher(
            ObstacleArray,
            str(self.get_parameter("planner_obstacles_topic").value),
            latched,
        )
        self.create_subscription(
            WeldSeamCandidateArray,
            str(self.get_parameter("candidate_topic").value),
            self._candidate_callback,
            10,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odometry_topic").value),
            self._odometry_callback,
            100,
        )
        self.create_subscription(
            ObstacleArray,
            str(self.get_parameter("obstacle_observation_topic").value),
            self._obstacle_array_callback,
            10,
        )
        self.create_subscription(
            ObstacleUpdate,
            str(self.get_parameter("obstacle_update_topic").value),
            self._obstacle_update_callback,
            10,
        )
        self.create_service(Trigger, "environment_map/save", self._save_service)
        self.create_service(Trigger, "environment_map/clear", self._clear_service)
        self.create_timer(
            float(self.get_parameter("map_save_period_s").value), self._save_if_dirty
        )
        self.startup_obstacle_timer = self.create_timer(
            2.0, self._republish_startup_obstacles
        )
        self._publish_map()
        self._publish_obstacles()

    def _odometry_callback(self, message: Odometry) -> None:
        timestamp = stamp_ns(message.header.stamp)
        pose = message.pose.pose
        twist = message.twist.twist
        motion = MotionState(
            float(pose.position.x),
            float(pose.position.y),
            yaw_from_quaternion(pose.orientation),
            float(twist.linear.x),
            float(twist.linear.y),
        )
        if self.motion_buffer and timestamp < self.motion_buffer[-1][0]:
            self.motion_buffer.clear()
            self.get_logger().warn("Odometry time moved backwards; motion buffer reset")
        if self.motion_buffer and timestamp == self.motion_buffer[-1][0]:
            self.motion_buffer[-1] = (timestamp, motion)
        else:
            self.motion_buffer.append((timestamp, motion))
        cutoff = timestamp - self.odometry_buffer_age_ns
        while len(self.motion_buffer) > 2 and self.motion_buffer[1][0] < cutoff:
            self.motion_buffer.popleft()

    def _motion_at(self, timestamp: int) -> MotionState | None:
        if not self.motion_buffer:
            return None
        items = list(self.motion_buffer)
        if timestamp <= items[0][0]:
            return (
                items[0][1]
                if items[0][0] - timestamp <= self.maximum_odometry_offset_ns
                else None
            )
        if timestamp >= items[-1][0]:
            return (
                items[-1][1]
                if timestamp - items[-1][0] <= self.maximum_odometry_offset_ns
                else None
            )
        for (before_stamp, before), (after_stamp, after) in zip(items, items[1:]):
            if before_stamp <= timestamp <= after_stamp:
                fraction = (timestamp - before_stamp) / (after_stamp - before_stamp)
                yaw_delta = (after.yaw_rad - before.yaw_rad + math.pi) % (
                    2.0 * math.pi
                ) - math.pi
                interpolate = lambda first, second: first + (second - first) * fraction
                return MotionState(
                    interpolate(before.x_m, after.x_m),
                    interpolate(before.y_m, after.y_m),
                    before.yaw_rad + yaw_delta * fraction,
                    interpolate(before.velocity_x_m_s, after.velocity_x_m_s),
                    interpolate(before.velocity_y_m_s, after.velocity_y_m_s),
                )
        return None

    def _candidate_callback(self, message: WeldSeamCandidateArray) -> None:
        if message.header.frame_id not in ("base_surface", "base_link"):
            self._publish_invalid_selection(message, "unsupported_candidate_frame")
            return
        timestamp = stamp_ns(message.header.stamp)
        motion = self._motion_at(timestamp)
        if motion is None:
            self._publish_invalid_selection(message, "capture_time_odometry_unavailable")
            return
        candidates = []
        observation_ids: set[str] = set()
        for index, item in enumerate(message.candidates):
            observation_id = item.observation_id or f"candidate_{index}"
            if observation_id in observation_ids:
                observation_id = f"{observation_id}_{index}"
            observation_ids.add(observation_id)
            candidates.append(
                SeamCandidate(
                    observation_id,
                    tuple((float(point.x), float(point.y)) for point in item.points),
                    float(item.confidence),
                    float(item.rail_position_at_capture_m),
                    bool(item.valid),
                )
            )
        candidates = distinct_candidates(candidates, self.config)
        associations = self.memory.observe_seams(candidates, motion, timestamp)
        decision = select_candidate(
            candidates,
            associations,
            motion,
            self.active_mapped_seam_id,
            self.config,
        )
        if candidates:
            self.map_dirty = True
        self._publish_decision(message, decision)
        self._publish_map()

    def _publish_decision(self, source, decision) -> None:
        selection = WeldSeamSelection()
        selection.header = source.header
        selection.task_id = source.task_id
        selection.sample_index = source.sample_index
        selection.candidate_count = decision.candidate_count
        selection.valid = decision.selected is not None
        selection.switched = decision.switched
        selection.reason = decision.reason
        if decision.selected is None:
            self.selection_publisher.publish(selection)
            return
        selected = decision.selected
        previous_id = self.active_mapped_seam_id
        self.active_mapped_seam_id = selected.mapped_seam_id
        selection.selected_observation_id = selected.candidate.observation_id
        selection.mapped_seam_id = selected.mapped_seam_id
        selection.score = float(selected.score)
        selection.confidence = float(selected.candidate.confidence)
        selection.lateral_error_m = float(selected.lateral_error_m)
        selection.switched = bool(previous_id and previous_id != selected.mapped_seam_id)
        self.selection_publisher.publish(selection)

        alignment = ProbeAlignment()
        alignment.header = source.header
        alignment.task_id = source.task_id
        alignment.sample_index = source.sample_index
        alignment.camera_frame = source.camera_frame or source.header.frame_id
        alignment.lateral_error_m = float(selected.lateral_error_m)
        alignment.rail_position_at_capture_m = float(
            selected.candidate.rail_position_at_capture_m
        )
        alignment.confidence = float(selected.candidate.confidence)
        alignment.valid = True
        alignment.detail = (
            f"selected={selected.candidate.observation_id};"
            f"map={selected.mapped_seam_id};reason={decision.reason}"
        )
        self.alignment_publisher.publish(alignment)

    def _publish_invalid_selection(self, source, reason: str) -> None:
        selection = WeldSeamSelection()
        selection.header = source.header
        selection.task_id = source.task_id
        selection.sample_index = source.sample_index
        selection.candidate_count = len(source.candidates)
        selection.valid = False
        selection.reason = reason
        self.selection_publisher.publish(selection)
        self.get_logger().warn(reason, throttle_duration_sec=1.0)

    @staticmethod
    def _stored_obstacle(message: Obstacle) -> StoredObstacle:
        return StoredObstacle(
            message.id,
            int(message.shape),
            float(message.center.x),
            float(message.center.y),
            float(message.center.z),
            float(message.radius),
            float(message.width),
            float(message.height),
            float(message.yaw),
        )

    @staticmethod
    def _obstacle_message(stored: StoredObstacle) -> Obstacle:
        message = Obstacle()
        message.id = stored.id
        message.shape = stored.shape
        message.center.x = stored.center_x
        message.center.y = stored.center_y
        message.center.z = stored.center_z
        message.radius = stored.radius
        message.width = stored.width
        message.height = stored.height
        message.yaw = stored.yaw
        return message

    def _obstacle_array_callback(self, message: ObstacleArray) -> None:
        frame_id = message.header.frame_id or self.map_frame
        motion = None
        if frame_id in ("base_surface", "base_link"):
            motion = self._motion_at(stamp_ns(message.header.stamp))
            if motion is None:
                self.get_logger().error("Obstacle capture-time odometry unavailable")
                return
        elif frame_id != self.map_frame:
            self.get_logger().error("Unsupported obstacle observation frame")
            return
        for obstacle in message.obstacles:
            try:
                stored = self._stored_obstacle(obstacle)
                if motion is not None:
                    cosine = math.cos(motion.yaw_rad)
                    sine = math.sin(motion.yaw_rad)
                    local_x, local_y = stored.center_x, stored.center_y
                    stored.center_x = motion.x_m + cosine * local_x - sine * local_y
                    stored.center_y = motion.y_m + sine * local_x + cosine * local_y
                    stored.yaw += motion.yaw_rad
                self.memory.upsert_obstacle(stored)
            except ValueError as error:
                self.get_logger().warn(f"Ignored obstacle: {error}")
        self.map_dirty = True
        self._publish_map()
        self._publish_obstacles()

    def _obstacle_update_callback(self, message: ObstacleUpdate) -> None:
        try:
            if message.operation == ObstacleUpdate.CLEAR:
                self.memory.clear_obstacles()
            elif message.operation == ObstacleUpdate.REMOVE:
                self.memory.remove_obstacle(message.obstacle.id)
            elif message.operation == ObstacleUpdate.UPSERT:
                self.memory.upsert_obstacle(self._stored_obstacle(message.obstacle))
            else:
                self.get_logger().warn("Ignored unknown obstacle map operation")
                return
        except ValueError as error:
            self.get_logger().warn(f"Ignored obstacle update: {error}")
            return
        self.map_dirty = True
        self._publish_map()
        self._publish_obstacles()

    def _publish_map(self) -> None:
        now = self.get_clock().now().to_msg()
        environment = EnvironmentMap()
        environment.header.stamp = now
        environment.header.frame_id = self.map_frame
        environment.revision = self.memory.revision
        for seam in sorted(self.memory.seams.values(), key=lambda item: item.id):
            message = MappedWeldSeam()
            message.id = seam.id
            message.points = [Point(x=x, y=y, z=0.0) for x, y in seam.points]
            message.confidence = float(seam.confidence)
            message.observation_count = seam.observation_count
            set_stamp(message.first_seen, seam.first_seen_ns)
            set_stamp(message.last_seen, seam.last_seen_ns)
            message.selected = seam.id == self.active_mapped_seam_id
            environment.weld_seams.append(message)
        environment.obstacles = [
            self._obstacle_message(obstacle)
            for obstacle in sorted(
                self.memory.obstacles.values(), key=lambda item: item.id
            )
        ]
        self.map_publisher.publish(environment)

    def _publish_obstacles(self) -> None:
        obstacle_array = ObstacleArray()
        obstacle_array.header.stamp = self.get_clock().now().to_msg()
        obstacle_array.header.frame_id = self.map_frame
        obstacle_array.obstacles = [
            self._obstacle_message(obstacle)
            for obstacle in sorted(
                self.memory.obstacles.values(), key=lambda item: item.id
            )
        ]
        self.obstacle_publisher.publish(obstacle_array)

    def _republish_startup_obstacles(self) -> None:
        self._publish_obstacles()
        self.startup_obstacle_timer.cancel()

    def _save_if_dirty(self) -> None:
        if not self.map_dirty:
            return
        try:
            self.memory.save(self.map_path)
            self.map_dirty = False
        except OSError as error:
            self.get_logger().error(f"Environment map save failed: {error}")

    def _save_service(self, request, response):
        del request
        try:
            self.memory.save(self.map_path)
            self.map_dirty = False
            response.success = True
            response.message = str(self.map_path)
        except OSError as error:
            response.success = False
            response.message = str(error)
        return response

    def _clear_service(self, request, response):
        del request
        self.memory.clear()
        self.active_mapped_seam_id = ""
        self.map_dirty = True
        self._publish_map()
        self._publish_obstacles()
        response.success = True
        response.message = "environment map cleared"
        return response

    def destroy_node(self):
        self._save_if_dirty()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SeamTrackingManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
