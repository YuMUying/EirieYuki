#!/usr/bin/env python3
"""Run one deterministic WCR planner scenario and save plots plus raw data."""

import argparse
import csv
import json
import math
import sys
import time
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Polygon
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as PathMsg
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String, UInt8
from wcr_planning_msgs.msg import (
    Obstacle,
    ObstacleArray,
    TimedTrajectory,
)

ROBOT_RADIUS = 0.18
SAFETY_MARGIN = 0.02
SAFETY_INFLATION = ROBOT_RADIUS + SAFETY_MARGIN
TRACKING_ERROR_ALLOWANCE = 0.05
PLANNING_INFLATION = SAFETY_INFLATION + TRACKING_ERROR_ALLOWANCE
PHYSICS_STEP = 0.001
POSITION_TOLERANCE = 0.025
YAW_TOLERANCE = 0.05
SETTLED_SAMPLE_COUNT = 20


def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_yaw(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def path_length(points):
    return sum(
        math.hypot(end[0] - start[0], end[1] - start[1])
        for start, end in zip(points, points[1:])
    )


def point_segment_distance(point, start, end):
    delta_x = end[0] - start[0]
    delta_y = end[1] - start[1]
    denominator = delta_x * delta_x + delta_y * delta_y
    if denominator <= 1.0e-12:
        return math.hypot(point[0] - start[0], point[1] - start[1])
    ratio = (
        (point[0] - start[0]) * delta_x
        + (point[1] - start[1]) * delta_y
    ) / denominator
    ratio = max(0.0, min(1.0, ratio))
    nearest = (start[0] + ratio * delta_x, start[1] + ratio * delta_y)
    return math.hypot(point[0] - nearest[0], point[1] - nearest[1])


def polyline_distance(point, polyline):
    if not polyline:
        return float("nan")
    if len(polyline) == 1:
        return math.hypot(point[0] - polyline[0][0], point[1] - polyline[0][1])
    return min(
        point_segment_distance(point, start, end)
        for start, end in zip(polyline, polyline[1:])
    )


def obstacle_boundary_clearance(point, obstacle, inflation):
    delta_x = point[0] - obstacle["center"][0]
    delta_y = point[1] - obstacle["center"][1]
    if obstacle["shape"] == "circle":
        return (
            math.hypot(delta_x, delta_y)
            - obstacle["radius"]
            - inflation
        )

    yaw = obstacle["yaw"]
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    local_x = cosine * delta_x + sine * delta_y
    local_y = -sine * delta_x + cosine * delta_y
    return max(
        abs(local_x) - obstacle["width"] * 0.5 - inflation,
        abs(local_y) - obstacle["height"] * 0.5 - inflation,
    )


def nearest_obstacle_clearance(point, obstacles, inflation):
    clearances = [
        (obstacle_boundary_clearance(point, obstacle, inflation), obstacle["id"])
        for obstacle in obstacles
    ]
    return min(clearances) if clearances else (float("inf"), "")


def sampled_polyline_min_clearance(polyline, obstacles, inflation, step=0.001):
    minimum = (float("inf"), "", -1)
    for segment_index, (start, end) in enumerate(zip(polyline, polyline[1:])):
        distance = math.hypot(end[0] - start[0], end[1] - start[1])
        sample_count = max(1, math.ceil(distance / step))
        for sample_index in range(sample_count + 1):
            ratio = sample_index / sample_count
            point = (
                start[0] + ratio * (end[0] - start[0]),
                start[1] + ratio * (end[1] - start[1]),
            )
            clearance, obstacle_id = nearest_obstacle_clearance(
                point, obstacles, inflation
            )
            if clearance < minimum[0]:
                minimum = (clearance, obstacle_id, segment_index)
    if len(polyline) == 1:
        clearance, obstacle_id = nearest_obstacle_clearance(
            polyline[0], obstacles, inflation
        )
        minimum = (clearance, obstacle_id, 0)
    return minimum


def rotated_rectangle(center, width, height, yaw, expansion=0.0):
    half_width = width * 0.5 + expansion
    half_height = height * 0.5 + expansion
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    corners = []
    for local_x, local_y in (
        (-half_width, -half_height),
        (half_width, -half_height),
        (half_width, half_height),
        (-half_width, half_height),
    ):
        corners.append(
            (
                center[0] + cosine * local_x - sine * local_y,
                center[1] + sine * local_x + cosine * local_y,
            )
        )
    return corners


def scenario_definition(mode):
    if mode == 1:
        return {
            "title": "Mode 1: point goal with autonomous obstacle avoidance",
            "target": [(0.00, 0.00), (0.72, 0.00)],
            "obstacles": [
                {
                    "id": "circle_center",
                    "shape": "circle",
                    "center": (0.30, 0.00),
                    "radius": 0.04,
                },
                {
                    "id": "upper_box",
                    "shape": "rectangle",
                    "center": (0.52, 0.33),
                    "width": 0.08,
                    "height": 0.08,
                    "yaw": 0.0,
                },
            ],
        }
    return {
        "title": "Mode 2: discrete reference curve with obstacle priority",
        # Dense camera-style centerline samples. They are observations of one
        # curve, not stop-and-turn waypoints.
        "target": [
            (
                0.72 * index / 60.0,
                0.095 * math.sin(math.pi * index / 60.0) ** 2
                + 0.008 * math.sin(2.0 * math.pi * index / 60.0) ** 2,
            )
            for index in range(61)
        ],
        "obstacles": [
            {
                "id": "curve_circle",
                "shape": "circle",
                "center": (0.30, 0.06),
                "radius": 0.035,
            },
            {
                "id": "lower_box",
                "shape": "rectangle",
                "center": (0.55, -0.30),
                "width": 0.10,
                "height": 0.08,
                "yaw": -0.30,
            },
        ],
    }


class ScenarioRecorder(Node):
    def __init__(self, mode):
        super().__init__("wcr_planner_scenario_recorder")
        scenario = scenario_definition(mode)
        self.mode = mode
        self.obstacles = scenario["obstacles"]
        self.target = scenario["target"]
        self.title = scenario["title"]
        self.odom = []
        self.path_history = []
        self.final_path = []
        self.trajectory_profile = []
        self.status = []
        self.latest_odom = None
        self.recording = False
        self.reached = False
        self.completion_signal_received = False
        self.completion_status_received = False
        self.settled_samples = 0
        self.completion_reason = "not_completed"
        self.safety_violation_detected = False
        self.safety_violation_samples = 0
        self.start_wall = None
        self.start_sim = None
        self.curve_refresh_published = False
        self.curve_refresh_sim_time = None

        state_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(Odometry, "/wcr/odom", self.on_odom, 50)
        self.create_subscription(PathMsg, "/wcr/planned_path", self.on_path, state_qos)
        self.create_subscription(
            TimedTrajectory,
            "/wcr/planned_trajectory",
            self.on_trajectory,
            state_qos,
        )
        self.create_subscription(String, "/wcr/planner_status", self.on_status, state_qos)
        self.create_subscription(Bool, "/wcr/target_reached", self.on_reached, state_qos)
        self.mode_publisher = self.create_publisher(UInt8, "/wcr/planning_mode", 10)
        self.obstacle_publisher = self.create_publisher(
            ObstacleArray, "/wcr/obstacles", 10
        )
        self.goal_publisher = self.create_publisher(
            PoseStamped, "/wcr/target_pose", 10
        )
        self.curve_publisher = self.create_publisher(
            PathMsg, "/wcr/reference_curve", 10
        )

    def elapsed_wall_time(self):
        if self.start_wall is None:
            return 0.0
        return time.monotonic() - self.start_wall

    def on_odom(self, message):
        self.latest_odom = message
        if not self.recording:
            return
        current_stamp = stamp_seconds(message.header.stamp)
        if self.start_sim is None:
            self.start_sim = current_stamp
        pose = message.pose.pose
        twist = message.twist.twist
        safety_clearance, nearest_obstacle = nearest_obstacle_clearance(
            (pose.position.x, pose.position.y),
            self.obstacles,
            SAFETY_INFLATION,
        )
        if safety_clearance <= 0.0:
            self.safety_violation_detected = True
            self.safety_violation_samples += 1
            self.completion_reason = "safety_exclusion_zone_intrusion"
        self.odom.append(
            {
                "sim_time": current_stamp - self.start_sim,
                "wall_time": self.elapsed_wall_time(),
                "x": pose.position.x,
                "y": pose.position.y,
                "yaw": quaternion_yaw(pose.orientation),
                "vx": twist.linear.x,
                "vy": twist.linear.y,
                "wz": twist.angular.z,
                "safety_clearance": safety_clearance,
                "nearest_obstacle": nearest_obstacle,
            }
        )
        if (
            self.mode == 2
            and not self.curve_refresh_published
            and self.odom[-1]["sim_time"] >= 5.0
        ):
            self.curve_publisher.publish(self.curve_message())
            self.curve_refresh_published = True
            self.curve_refresh_sim_time = self.odom[-1]["sim_time"]
        self.update_completion_state()

    def on_path(self, message):
        points = [
            (pose.pose.position.x, pose.pose.position.y)
            for pose in message.poses
        ]
        self.path_history.append((self.elapsed_wall_time(), points))
        if points:
            self.final_path = points

    def on_trajectory(self, message):
        if message.points:
            self.trajectory_profile = [
                {
                    "point_index": index,
                    "x": point.pose.position.x,
                    "y": point.pose.position.y,
                    "arc_length": point.arc_length,
                    "curvature": point.curvature,
                    "planned_speed": point.speed,
                }
                for index, point in enumerate(message.points)
            ]

    def on_status(self, message):
        self.status.append((self.elapsed_wall_time(), message.data))
        if self.recording and message.data.startswith("safety_violation:"):
            self.safety_violation_detected = True
            self.completion_reason = "planner_safety_violation_stop"
        if self.recording and message.data == "completed":
            self.completion_status_received = True

    def on_reached(self, message):
        if self.recording and message.data:
            self.completion_signal_received = True

    def completion_errors(self):
        if self.latest_odom is None:
            return float("inf"), float("inf")
        pose = self.latest_odom.pose.pose
        final_x, final_y = self.target[-1]
        position_error = math.hypot(
            pose.position.x - final_x, pose.position.y - final_y
        )
        yaw_error = abs(normalize_angle(quaternion_yaw(pose.orientation)))
        return position_error, yaw_error

    def update_completion_state(self):
        if not self.recording or self.reached:
            return
        if self.safety_violation_detected:
            self.settled_samples = 0
            return
        position_error, yaw_error = self.completion_errors()
        signals_ready = (
            self.completion_signal_received and self.completion_status_received
        )
        pose_ready = (
            position_error <= POSITION_TOLERANCE
            and yaw_error <= YAW_TOLERANCE
        )
        if signals_ready and pose_ready:
            self.settled_samples += 1
        else:
            self.settled_samples = 0
        if self.settled_samples >= SETTLED_SAMPLE_COUNT:
            self.reached = True
            self.completion_reason = (
                "planner_completed_and_final_pose_settled"
            )

    def wait_for_system(self, timeout):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            ready = (
                self.mode_publisher.get_subscription_count() > 0
                and self.obstacle_publisher.get_subscription_count() > 0
            )
            if self.latest_odom is not None and ready:
                return True
        return False

    def obstacle_message(self):
        message = ObstacleArray()
        message.header.frame_id = "odom"
        message.header.stamp = self.get_clock().now().to_msg()
        for definition in self.obstacles:
            obstacle = Obstacle()
            obstacle.id = definition["id"]
            obstacle.center.x = definition["center"][0]
            obstacle.center.y = definition["center"][1]
            if definition["shape"] == "circle":
                obstacle.shape = Obstacle.CIRCLE
                obstacle.radius = definition["radius"]
            else:
                obstacle.shape = Obstacle.RECTANGLE
                obstacle.width = definition["width"]
                obstacle.height = definition["height"]
                obstacle.yaw = definition["yaw"]
            message.obstacles.append(obstacle)
        return message

    def curve_message(self):
        curve = PathMsg()
        curve.header.frame_id = "odom"
        curve.header.stamp = self.get_clock().now().to_msg()
        for x_position, y_position in self.target:
            pose = PoseStamped()
            pose.header = curve.header
            pose.pose.position.x = x_position
            pose.pose.position.y = y_position
            pose.pose.orientation.w = 1.0
            curve.poses.append(pose)
        return curve

    def publish_scenario(self):
        mode_message = UInt8()
        mode_message.data = self.mode
        self.mode_publisher.publish(mode_message)
        self.obstacle_publisher.publish(self.obstacle_message())

        # Let cancel/replan messages settle before starting the measured command.
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.1)

        self.recording = True
        self.start_wall = time.monotonic()
        self.completion_signal_received = False
        self.completion_status_received = False
        self.settled_samples = 0
        self.completion_reason = "not_completed"
        self.safety_violation_detected = False
        self.safety_violation_samples = 0
        self.curve_refresh_published = False
        self.curve_refresh_sim_time = None
        if self.mode == 1:
            goal = PoseStamped()
            goal.header.frame_id = "odom"
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.pose.position.x = self.target[-1][0]
            goal.pose.position.y = self.target[-1][1]
            goal.pose.orientation.w = 1.0
            self.goal_publisher.publish(goal)
            return

        self.curve_publisher.publish(self.curve_message())

    def run_until_complete(self, simulation_timeout, wall_timeout):
        wall_deadline = time.monotonic() + wall_timeout
        while (
            rclpy.ok()
            and time.monotonic() < wall_deadline
            and not self.reached
            and not self.safety_violation_detected
        ):
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.odom and self.odom[-1]["sim_time"] >= simulation_timeout:
                self.completion_reason = "simulation_timeout"
                break
        if (
            not self.reached
            and not self.safety_violation_detected
            and self.completion_reason == "not_completed"
        ):
            self.completion_reason = "wall_timeout"
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.05)
        return self.reached


def write_csv(path, fieldnames, rows):
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def save_raw_data(recorder, output_dir, trajectory_rows, planned):
    write_csv(
        output_dir / "robot_trajectory.csv",
        [
            "sim_time",
            "wall_time",
            "x",
            "y",
            "yaw",
            "vx",
            "vy",
            "wz",
            "speed",
            "planned_path_error",
            "safety_clearance",
            "nearest_obstacle",
        ],
        trajectory_rows,
    )
    write_csv(
        output_dir / "target_trajectory.csv",
        ["point_index", "x", "y"],
        [
            {"point_index": index, "x": point[0], "y": point[1]}
            for index, point in enumerate(recorder.target)
        ],
    )
    write_csv(
        output_dir / "planned_path.csv",
        ["waypoint_index", "x", "y"],
        [
            {"waypoint_index": index, "x": point[0], "y": point[1]}
            for index, point in enumerate(planned)
        ],
    )
    write_csv(
        output_dir / "planned_speed_profile.csv",
        [
            "point_index",
            "x",
            "y",
            "arc_length",
            "curvature",
            "planned_speed",
        ],
        recorder.trajectory_profile,
    )

    history_rows = []
    for replan_index, (wall_time, points) in enumerate(recorder.path_history):
        for waypoint_index, point in enumerate(points):
            history_rows.append(
                {
                    "replan_index": replan_index,
                    "wall_time": wall_time,
                    "waypoint_index": waypoint_index,
                    "x": point[0],
                    "y": point[1],
                }
            )
    write_csv(
        output_dir / "planned_path_history.csv",
        ["replan_index", "wall_time", "waypoint_index", "x", "y"],
        history_rows,
    )

    obstacle_rows = []
    for item in recorder.obstacles:
        obstacle_rows.append(
            {
                "id": item["id"],
                "shape": item["shape"],
                "center_x": item["center"][0],
                "center_y": item["center"][1],
                "radius": item.get("radius", 0.0),
                "width": item.get("width", 0.0),
                "height": item.get("height", 0.0),
                "yaw": item.get("yaw", 0.0),
                "inflation": SAFETY_INFLATION,
                "planning_inflation": PLANNING_INFLATION,
            }
        )
    write_csv(
        output_dir / "obstacles.csv",
        [
            "id",
            "shape",
            "center_x",
            "center_y",
            "radius",
            "width",
            "height",
            "yaw",
            "inflation",
            "planning_inflation",
        ],
        obstacle_rows,
    )
    write_csv(
        output_dir / "planner_status.csv",
        ["wall_time", "status"],
        [
            {"wall_time": wall_time, "status": status}
            for wall_time, status in recorder.status
        ],
    )


def draw_obstacles(axis, obstacles):
    for index, obstacle in enumerate(obstacles):
        obstacle_label = "Obstacle" if index == 0 else None
        inflation_label = "Inflated exclusion boundary" if index == 0 else None
        if obstacle["shape"] == "circle":
            axis.add_patch(
                Circle(
                    obstacle["center"],
                    obstacle["radius"] + PLANNING_INFLATION,
                    facecolor="none",
                    edgecolor="#64748b",
                    linestyle=":",
                    linewidth=1.2,
                    label="Planning boundary" if index == 0 else None,
                )
            )
            axis.add_patch(
                Circle(
                    obstacle["center"],
                    obstacle["radius"] + SAFETY_INFLATION,
                    facecolor="none",
                    edgecolor="#d97706",
                    linestyle="--",
                    linewidth=1.5,
                    label=inflation_label,
                )
            )
            axis.add_patch(
                Circle(
                    obstacle["center"],
                    obstacle["radius"],
                    facecolor="#ef4444",
                    edgecolor="#991b1b",
                    alpha=0.65,
                    label=obstacle_label,
                )
            )
        else:
            axis.add_patch(
                Polygon(
                    rotated_rectangle(
                        obstacle["center"],
                        obstacle["width"],
                        obstacle["height"],
                        obstacle["yaw"],
                        PLANNING_INFLATION,
                    ),
                    closed=True,
                    facecolor="none",
                    edgecolor="#64748b",
                    linestyle=":",
                    linewidth=1.2,
                    label="Planning boundary" if index == 0 else None,
                )
            )
            axis.add_patch(
                Polygon(
                    rotated_rectangle(
                        obstacle["center"],
                        obstacle["width"],
                        obstacle["height"],
                        obstacle["yaw"],
                        SAFETY_INFLATION,
                    ),
                    closed=True,
                    facecolor="none",
                    edgecolor="#d97706",
                    linestyle="--",
                    linewidth=1.5,
                    label=inflation_label,
                )
            )
            axis.add_patch(
                Polygon(
                    rotated_rectangle(
                        obstacle["center"],
                        obstacle["width"],
                        obstacle["height"],
                        obstacle["yaw"],
                    ),
                    closed=True,
                    facecolor="#ef4444",
                    edgecolor="#991b1b",
                    alpha=0.65,
                    label=obstacle_label,
                )
            )
        axis.text(
            obstacle["center"][0],
            obstacle["center"][1],
            obstacle["id"],
            fontsize=8,
            horizontalalignment="center",
            verticalalignment="center",
            color="#111827",
        )


def create_report(recorder, output_dir, trajectory_rows, planned, summary):
    actual = [(row["x"], row["y"]) for row in trajectory_rows]
    errors = [row["planned_path_error"] for row in trajectory_rows]
    clearances = [row["safety_clearance"] for row in trajectory_rows]
    final_goal = recorder.target[-1]

    figure = plt.figure(figsize=(13.5, 7.5), constrained_layout=True)
    grid = figure.add_gridspec(2, 2, width_ratios=(1.55, 1.0))
    trajectory_axis = figure.add_subplot(grid[:, 0])
    speed_axis = figure.add_subplot(grid[0, 1])
    error_axis = figure.add_subplot(grid[1, 1])

    draw_obstacles(trajectory_axis, recorder.obstacles)
    target_x, target_y = zip(*recorder.target)
    target_label = (
        "Point-goal direct reference"
        if recorder.mode == 1
        else "Input reference curve"
    )
    trajectory_axis.plot(
        target_x,
        target_y,
        color="#2563eb",
        linestyle=":",
        linewidth=2.2,
        marker="o",
        markersize=4,
        label=target_label,
    )
    if planned:
        planned_x, planned_y = zip(*planned)
        trajectory_axis.plot(
            planned_x,
            planned_y,
            color="#7c3aed",
            linewidth=2.2,
            marker="s",
            markersize=4,
            label="Collision-free planned path",
        )
    if actual:
        actual_x, actual_y = zip(*actual)
        trajectory_axis.plot(
            actual_x,
            actual_y,
            color="#059669",
            linewidth=2.0,
            label="Robot odometry trajectory",
        )
        trajectory_axis.scatter(
            actual_x[0], actual_y[0], marker="o", s=65, color="#111827",
            label="Start", zorder=8
        )
        trajectory_axis.scatter(
            actual_x[-1], actual_y[-1], marker="x", s=75, color="#059669",
            linewidth=2.0, label="Final pose", zorder=8
        )
    trajectory_axis.scatter(
        final_goal[0],
        final_goal[1],
        marker="*",
        s=150,
        color="#f59e0b",
        edgecolor="#78350f",
        label="Goal",
        zorder=9,
    )
    trajectory_axis.set_title(recorder.title, fontsize=13)
    trajectory_axis.set_xlabel("Odom X (m)")
    trajectory_axis.set_ylabel("Odom Y (m)")
    trajectory_axis.set_aspect("equal", adjustable="datalim")
    trajectory_axis.grid(True, color="#d1d5db", linewidth=0.7, alpha=0.75)
    trajectory_axis.legend(loc="best", fontsize=8)

    times = [row["sim_time"] for row in trajectory_rows]
    speeds = [row["speed"] for row in trajectory_rows]
    yaw_rates = [abs(row["wz"]) for row in trajectory_rows]
    speed_axis.plot(times, speeds, color="#0f766e", label="Linear speed")
    speed_axis.plot(
        times, yaw_rates, color="#b45309", alpha=0.85, label="Abs yaw rate"
    )
    speed_axis.set_title("Robot motion")
    speed_axis.set_xlabel("Simulation time (s)")
    speed_axis.set_ylabel("m/s or rad/s")
    speed_axis.grid(True, color="#d1d5db", linewidth=0.7, alpha=0.75)
    speed_axis.legend(fontsize=8)

    error_axis.plot(times, errors, color="#be123c", label="Distance to planned path")
    error_axis.plot(
        times,
        clearances,
        color="#0369a1",
        label="Clearance outside safety boundary",
    )
    error_axis.axhline(
        0.0,
        color="#dc2626",
        linestyle="-",
        linewidth=1.0,
        label="Intrusion threshold",
    )
    error_axis.axhline(
        0.025,
        color="#6b7280",
        linestyle="--",
        linewidth=1.0,
        label="Planner waypoint tolerance",
    )
    error_axis.set_title("Tracking error and safety clearance")
    error_axis.set_xlabel("Simulation time (s)")
    error_axis.set_ylabel("Distance (m)")
    error_axis.grid(True, color="#d1d5db", linewidth=0.7, alpha=0.75)
    error_axis.legend(fontsize=8)

    figure.suptitle(
        "WCR Gazebo planner test | reached={} | safe={} | final error={:.3f} m | "
        "min clearance={:.3f} m | sim={:.1f} s | wall={:.1f} s".format(
            recorder.reached,
            not recorder.safety_violation_detected,
            summary["final_position_error_m"],
            summary["minimum_actual_safety_clearance_m"],
            summary["simulation_duration_seconds"],
            summary["wall_duration_seconds"],
        ),
        fontsize=14,
    )
    figure.savefig(output_dir / "trajectory_report.png", dpi=180)
    figure.savefig(output_dir / "trajectory_report.svg")
    plt.close(figure)


def save_results(recorder, output_dir):
    output_dir.mkdir(parents=True, exist_ok=True)
    actual = [(row["x"], row["y"]) for row in recorder.odom]
    planned = recorder.final_path
    valid_path_history = [
        (wall_time, points)
        for wall_time, points in recorder.path_history
        if points
    ]
    errors = []
    history_index = 0
    for row, point in zip(recorder.odom, actual):
        while (
            history_index + 1 < len(valid_path_history)
            and valid_path_history[history_index + 1][0] <= row["wall_time"]
        ):
            history_index += 1
        active_path = (
            valid_path_history[history_index][1]
            if valid_path_history
            else planned
        )
        errors.append(polyline_distance(point, active_path))
    valid_errors = [value for value in errors if math.isfinite(value)]
    actual_clearance = sampled_polyline_min_clearance(
        actual, recorder.obstacles, SAFETY_INFLATION
    )
    planned_safety_clearance = sampled_polyline_min_clearance(
        planned, recorder.obstacles, SAFETY_INFLATION
    )
    planned_boundary_clearance = sampled_polyline_min_clearance(
        planned, recorder.obstacles, PLANNING_INFLATION
    )
    final_goal = recorder.target[-1]
    final_error = (
        math.hypot(
            actual[-1][0] - final_goal[0], actual[-1][1] - final_goal[1]
        )
        if actual
        else float("nan")
    )

    trajectory_rows = []
    for row, tracking_error in zip(recorder.odom, errors):
        output = dict(row)
        output["speed"] = math.hypot(row["vx"], row["vy"])
        output["planned_path_error"] = tracking_error
        trajectory_rows.append(output)
    interior_speeds = [
        row["speed"]
        for row in trajectory_rows
        if math.hypot(row["x"] - actual[0][0], row["y"] - actual[0][1]) > 0.08
        and math.hypot(
            row["x"] - final_goal[0], row["y"] - final_goal[1]
        ) > 0.08
    ]
    low_speed_events = 0
    refresh_window_speeds = [
        row["speed"]
        for row in trajectory_rows
        if recorder.curve_refresh_sim_time is not None
        and abs(row["sim_time"] - recorder.curve_refresh_sim_time) <= 0.75
    ]
    previously_low = False
    for speed in interior_speeds:
        currently_low = speed < 0.015
        if currently_low and not previously_low:
            low_speed_events += 1
        previously_low = currently_low
    save_raw_data(recorder, output_dir, trajectory_rows, planned)

    summary = {
        "mode": recorder.mode,
        "reached": recorder.reached,
        "completion_reason": recorder.completion_reason,
        "completion_signal_received": recorder.completion_signal_received,
        "completion_status_received": recorder.completion_status_received,
        "completion_position_tolerance_m": POSITION_TOLERANCE,
        "completion_yaw_tolerance_rad": YAW_TOLERANCE,
        "completion_settled_samples_required": SETTLED_SAMPLE_COUNT,
        "completion_settled_samples_observed": recorder.settled_samples,
        "physics_step_seconds": PHYSICS_STEP,
        "robot_radius_m": ROBOT_RADIUS,
        "safety_margin_m": SAFETY_MARGIN,
        "robot_radius_plus_margin": SAFETY_INFLATION,
        "tracking_error_allowance_m": TRACKING_ERROR_ALLOWANCE,
        "planning_inflation_m": PLANNING_INFLATION,
        "safety_violation_detected": recorder.safety_violation_detected,
        "safety_violation_samples": recorder.safety_violation_samples,
        "minimum_actual_safety_clearance_m": actual_clearance[0],
        "minimum_actual_clearance_obstacle": actual_clearance[1],
        "minimum_actual_clearance_segment": actual_clearance[2],
        "minimum_planned_safety_clearance_m": planned_safety_clearance[0],
        "minimum_planned_boundary_clearance_m": planned_boundary_clearance[0],
        "samples": len(actual),
        "planned_waypoints": len(planned),
        "input_reference_points": len(recorder.target),
        "trajectory_profile_points": len(recorder.trajectory_profile),
        "maximum_actual_speed_mps": (
            max(row["speed"] for row in trajectory_rows)
            if trajectory_rows else None
        ),
        "mean_interior_speed_mps": (
            sum(interior_speeds) / len(interior_speeds)
            if interior_speeds else None
        ),
        "minimum_interior_speed_mps": (
            min(interior_speeds) if interior_speeds else None
        ),
        "interior_low_speed_threshold_mps": 0.015,
        "interior_low_speed_events": low_speed_events,
        "curve_refresh_published": recorder.curve_refresh_published,
        "curve_refresh_sim_time": recorder.curve_refresh_sim_time,
        "minimum_speed_near_curve_refresh_mps": (
            min(refresh_window_speeds) if refresh_window_speeds else None
        ),
        "planned_peak_speed_mps": (
            max(
                point["planned_speed"]
                for point in recorder.trajectory_profile
            )
            if recorder.trajectory_profile else None
        ),
        "simulation_duration_seconds": (
            recorder.odom[-1]["sim_time"] if recorder.odom else 0.0
        ),
        "wall_duration_seconds": recorder.elapsed_wall_time(),
        "target_path_length_m": path_length(recorder.target),
        "planned_path_length_m": path_length(planned),
        "actual_path_length_m": path_length(actual),
        "final_position_error_m": final_error,
        "mean_planned_path_error_m": (
            sum(valid_errors) / len(valid_errors) if valid_errors else None
        ),
        "max_planned_path_error_m": max(valid_errors) if valid_errors else None,
        "final_planner_status": recorder.status[-1][1] if recorder.status else "missing",
    }
    with (output_dir / "summary.json").open("w", encoding="utf-8") as stream:
        json.dump(summary, stream, indent=2, ensure_ascii=True)
    create_report(recorder, output_dir, trajectory_rows, planned, summary)
    return summary


def main():
    parser = argparse.ArgumentParser(
        description="Run and record one WCR planner scenario."
    )
    parser.add_argument("--mode", type=int, choices=(1, 2), required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--startup-timeout", type=float, default=45.0)
    parser.add_argument("--simulation-timeout", type=float, default=20.0)
    parser.add_argument("--wall-timeout", type=float, default=600.0)
    arguments = parser.parse_args()

    rclpy.init()
    recorder = ScenarioRecorder(arguments.mode)
    exit_code = 0
    try:
        if not recorder.wait_for_system(arguments.startup_timeout):
            print(
                "Timed out waiting for odometry and planner subscriptions.",
                file=sys.stderr,
            )
            return 2
        recorder.publish_scenario()
        if not recorder.run_until_complete(
            arguments.simulation_timeout, arguments.wall_timeout
        ):
            print("Scenario did not reach its goal before timeout.", file=sys.stderr)
            exit_code = 3
        summary = save_results(recorder, arguments.output)
        print(json.dumps(summary, indent=2))
        if not recorder.final_path or not recorder.odom:
            exit_code = 4
        elif recorder.safety_violation_detected:
            exit_code = 5
        elif summary["minimum_actual_safety_clearance_m"] <= 0.0:
            exit_code = 6
        elif summary["minimum_planned_safety_clearance_m"] <= 0.0:
            exit_code = 7
        elif (
            not recorder.curve_refresh_published
            and summary["minimum_planned_boundary_clearance_m"] <= 0.0
        ):
            exit_code = 8
    finally:
        recorder.destroy_node()
        rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
