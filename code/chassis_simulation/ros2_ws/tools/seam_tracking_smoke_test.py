#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path
import time

import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger
from wcr_planning_msgs.msg import (
    EnvironmentMap,
    Obstacle,
    ObstacleArray,
    ProbeAlignment,
    WeldSeamCandidate,
    WeldSeamCandidateArray,
    WeldSeamSelection,
)


class SmokeClient(Node):
    def __init__(self) -> None:
        super().__init__("seam_tracking_smoke_client")
        latched = QoSProfile(depth=1)
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.odom_publisher = self.create_publisher(Odometry, "/wcr/odom", 10)
        self.candidate_publisher = self.create_publisher(
            WeldSeamCandidateArray, "/wcr/weld_seam_candidates", 10
        )
        self.obstacle_publisher = self.create_publisher(
            ObstacleArray, "/wcr/obstacles", 10
        )
        self.selection: WeldSeamSelection | None = None
        self.alignment: ProbeAlignment | None = None
        self.environment: EnvironmentMap | None = None
        self.mapped_obstacles: ObstacleArray | None = None
        self.create_subscription(
            WeldSeamSelection,
            "/wcr/weld_seam_selection",
            self._selection_callback,
            10,
        )
        self.create_subscription(
            ProbeAlignment, "/wcr/probe_alignment", self._alignment_callback, 10
        )
        self.create_subscription(
            EnvironmentMap, "/wcr/environment_map", self._map_callback, latched
        )
        self.create_subscription(
            ObstacleArray,
            "/wcr/mapped_obstacles",
            self._obstacle_callback,
            latched,
        )
        self.save_client = self.create_client(Trigger, "/wcr/environment_map/save")

    def _selection_callback(self, message: WeldSeamSelection) -> None:
        if message.sample_index == 1:
            self.selection = message

    def _alignment_callback(self, message: ProbeAlignment) -> None:
        if message.sample_index == 1:
            self.alignment = message

    def _map_callback(self, message: EnvironmentMap) -> None:
        if message.weld_seams and message.obstacles:
            self.environment = message

    def _obstacle_callback(self, message: ObstacleArray) -> None:
        if message.obstacles:
            self.mapped_obstacles = message

    @staticmethod
    def _candidate(identifier: str, points, confidence: float) -> WeldSeamCandidate:
        candidate = WeldSeamCandidate()
        candidate.observation_id = identifier
        candidate.points = [Point(x=x, y=y, z=0.0) for x, y in points]
        candidate.confidence = confidence
        candidate.rail_position_at_capture_m = 0.0
        candidate.valid = True
        candidate.detail = "smoke"
        return candidate

    def publish_inputs(self) -> None:
        stamp = self.get_clock().now().to_msg()
        odometry = Odometry()
        odometry.header.stamp = stamp
        odometry.header.frame_id = "odom"
        odometry.child_frame_id = "base_link"
        odometry.pose.pose.position.x = 1.0
        odometry.pose.pose.position.y = 2.0
        odometry.pose.pose.orientation.w = 1.0
        odometry.twist.twist.linear.x = 0.08
        self.odom_publisher.publish(odometry)

        obstacle_array = ObstacleArray()
        obstacle_array.header.stamp = stamp
        obstacle_array.header.frame_id = "odom"
        obstacle = Obstacle()
        obstacle.id = "nozzle_1"
        obstacle.shape = Obstacle.CIRCLE
        obstacle.center.x = 1.4
        obstacle.center.y = 2.2
        obstacle.radius = 0.08
        obstacle_array.obstacles = [obstacle]
        self.obstacle_publisher.publish(obstacle_array)

        candidates = WeldSeamCandidateArray()
        candidates.header.stamp = stamp
        candidates.header.frame_id = "base_surface"
        candidates.camera_frame = "probe_camera_color_optical_frame"
        candidates.task_id = "smoke"
        candidates.sample_index = 1
        candidates.candidates = [
            self._candidate(
                "forward",
                [(0.02 * index, 0.01) for index in range(1, 7)],
                0.90,
            ),
            self._candidate(
                "crossing",
                [(0.08, -0.05 + 0.02 * index) for index in range(6)],
                0.98,
            ),
        ]
        self.candidate_publisher.publish(candidates)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--map-path", type=Path, required=True)
    parser.add_argument("--timeout", type=float, default=12.0)
    args = parser.parse_args()

    rclpy.init()
    node = SmokeClient()
    try:
        discovery_deadline = time.monotonic() + args.timeout / 2.0
        while time.monotonic() < discovery_deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if (
                node.odom_publisher.get_subscription_count() > 0
                and node.candidate_publisher.get_subscription_count() > 0
                and node.obstacle_publisher.get_subscription_count() > 0
            ):
                break
        else:
            raise RuntimeError("manager subscriptions were not discovered")

        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline:
            node.publish_inputs()
            for _ in range(5):
                rclpy.spin_once(node, timeout_sec=0.1)
            if all(
                value is not None
                for value in (
                    node.selection,
                    node.alignment,
                    node.environment,
                    node.mapped_obstacles,
                )
            ):
                break
        else:
            raise RuntimeError("timed out waiting for seam-tracking outputs")

        assert node.selection is not None
        assert node.alignment is not None
        assert node.environment is not None
        assert node.mapped_obstacles is not None
        assert node.selection.valid
        assert node.selection.candidate_count == 2
        assert node.selection.selected_observation_id == "forward"
        assert node.alignment.valid
        assert node.alignment.camera_frame == "probe_camera_color_optical_frame"
        assert abs(node.alignment.lateral_error_m - 0.01) < 1e-6
        assert len(node.environment.weld_seams) == 2
        assert any(item.id == "nozzle_1" for item in node.environment.obstacles)
        assert any(item.id == "nozzle_1" for item in node.mapped_obstacles.obstacles)

        if not node.save_client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError("map save service unavailable")
        future = node.save_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=3.0)
        response = future.result()
        if response is None or not response.success:
            raise RuntimeError("map save service failed")
        if not args.map_path.is_file():
            raise RuntimeError("map file was not created")

        print(
            "smoke_ok "
            f"selected={node.selection.selected_observation_id} "
            f"mapped={node.selection.mapped_seam_id} "
            f"seams={len(node.environment.weld_seams)} "
            f"obstacles={len(node.environment.obstacles)} "
            f"map={args.map_path}"
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
