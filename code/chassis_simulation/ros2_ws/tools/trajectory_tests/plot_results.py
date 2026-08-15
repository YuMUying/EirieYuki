#!/usr/bin/env python3
"""Create publication-ready WCR trajectory plots from recorded CSV logs."""

import argparse
import csv
import json
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Polygon
import numpy as np

COLORS = {
    "actual": "#087f5b",
    "planned": "#6741d9",
    "target": "#1864ab",
    "obstacle": "#e03131",
    "inflation": "#f08c00",
    "planning_boundary": "#495057",
    "clearance": "#087f5b",
    "speed": "#0b7285",
    "yaw_rate": "#d9480f",
    "error": "#c2255c",
    "neutral": "#343a40",
    "grid": "#ced4da",
}


def read_csv(path):
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def numeric(rows, name):
    return np.asarray([float(row[name]) for row in rows], dtype=float)


def rotated_rectangle(center_x, center_y, width, height, yaw, expansion=0.0):
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
                center_x + cosine * local_x - sine * local_y,
                center_y + sine * local_x + cosine * local_y,
            )
        )
    return corners


def draw_obstacles(axis, obstacles):
    for index, obstacle in enumerate(obstacles):
        center_x = float(obstacle["center_x"])
        center_y = float(obstacle["center_y"])
        inflation = float(obstacle["inflation"])
        planning_inflation = float(
            obstacle.get("planning_inflation", obstacle["inflation"])
        )
        physical_label = "Physical obstacle" if index == 0 else None
        inflation_label = "Hard safety boundary" if index == 0 else None
        planning_label = "Planning boundary" if index == 0 else None
        if obstacle["shape"] == "circle":
            radius = float(obstacle["radius"])
            axis.add_patch(
                Circle(
                    (center_x, center_y),
                    radius + planning_inflation,
                    fill=False,
                    edgecolor=COLORS["planning_boundary"],
                    linewidth=1.4,
                    linestyle=":",
                    label=planning_label,
                    zorder=1,
                )
            )
            axis.add_patch(
                Circle(
                    (center_x, center_y),
                    radius + inflation,
                    fill=False,
                    edgecolor=COLORS["inflation"],
                    linewidth=1.8,
                    linestyle="--",
                    label=inflation_label,
                    zorder=2,
                )
            )
            axis.add_patch(
                Circle(
                    (center_x, center_y),
                    radius,
                    facecolor=COLORS["obstacle"],
                    edgecolor="#7f1d1d",
                    alpha=0.75,
                    label=physical_label,
                    zorder=3,
                )
            )
        else:
            width = float(obstacle["width"])
            height = float(obstacle["height"])
            yaw = float(obstacle["yaw"])
            axis.add_patch(
                Polygon(
                    rotated_rectangle(
                        center_x, center_y, width, height, yaw, planning_inflation
                    ),
                    closed=True,
                    fill=False,
                    edgecolor=COLORS["planning_boundary"],
                    linewidth=1.4,
                    linestyle=":",
                    label=planning_label,
                    zorder=1,
                )
            )
            axis.add_patch(
                Polygon(
                    rotated_rectangle(
                        center_x, center_y, width, height, yaw, inflation
                    ),
                    closed=True,
                    fill=False,
                    edgecolor=COLORS["inflation"],
                    linewidth=1.8,
                    linestyle="--",
                    label=inflation_label,
                    zorder=2,
                )
            )
            axis.add_patch(
                Polygon(
                    rotated_rectangle(center_x, center_y, width, height, yaw),
                    closed=True,
                    facecolor=COLORS["obstacle"],
                    edgecolor="#7f1d1d",
                    alpha=0.75,
                    label=physical_label,
                    zorder=3,
                )
            )
        axis.annotate(
            obstacle["id"],
            (center_x, center_y),
            xytext=(4, 5),
            textcoords="offset points",
            fontsize=8,
            color=COLORS["neutral"],
            zorder=8,
        )


def load_mode(root, mode):
    mode_dir = root / f"mode{mode}"
    with (mode_dir / "summary.json").open(encoding="utf-8") as stream:
        summary = json.load(stream)
    return {
        "mode": mode,
        "dir": mode_dir,
        "summary": summary,
        "robot": read_csv(mode_dir / "robot_trajectory.csv"),
        "target": read_csv(mode_dir / "target_trajectory.csv"),
        "planned": read_csv(mode_dir / "planned_path.csv"),
        "planned_history": read_csv(mode_dir / "planned_path_history.csv"),
        "obstacles": read_csv(mode_dir / "obstacles.csv"),
    }


def style_axis(axis):
    axis.grid(True, color=COLORS["grid"], linewidth=0.7, alpha=0.75)
    axis.set_axisbelow(True)
    for spine in axis.spines.values():
        spine.set_color("#868e96")
        spine.set_linewidth(0.8)


def draw_trajectory(axis, data, title):
    robot = data["robot"]
    target = data["target"]
    planned = data["planned"]
    actual_x = numeric(robot, "x")
    actual_y = numeric(robot, "y")
    target_x = numeric(target, "x")
    target_y = numeric(target, "y")
    planned_x = numeric(planned, "x")
    planned_y = numeric(planned, "y")
    history = data["planned_history"]

    draw_obstacles(axis, data["obstacles"])
    axis.plot(
        target_x,
        target_y,
        color=COLORS["target"],
        linewidth=2.0,
        linestyle=":",
        marker="o",
        markersize=4,
        label="Requested target/reference",
        zorder=4,
    )
    history_groups = {}
    for row in history:
        history_groups.setdefault(int(row["replan_index"]), []).append(row)
    for group_index, rows in sorted(history_groups.items()):
        if group_index == max(history_groups, default=group_index):
            continue
        axis.plot(
            numeric(rows, "x"),
            numeric(rows, "y"),
            color="#b197fc",
            linewidth=1.4,
            linestyle="--",
            alpha=0.9,
            label="Previous active path" if group_index == min(history_groups) else None,
            zorder=4,
        )
    axis.plot(
        planned_x,
        planned_y,
        color=COLORS["planned"],
        linewidth=2.2,
        marker="s",
        markersize=4,
        label="Collision-free planned path",
        zorder=5,
    )
    axis.plot(
        actual_x,
        actual_y,
        color=COLORS["actual"],
        linewidth=2.0,
        label="Robot odometry trajectory",
        zorder=6,
    )
    axis.scatter(
        actual_x[0], actual_y[0], marker="o", s=55,
        color=COLORS["neutral"], label="Start", zorder=9
    )
    axis.scatter(
        actual_x[-1], actual_y[-1], marker="x", s=70,
        color=COLORS["actual"], linewidth=2.2, label="Final pose", zorder=9
    )
    axis.scatter(
        target_x[-1], target_y[-1], marker="*", s=145,
        color=COLORS["inflation"], edgecolor="#7c2d12", label="Goal", zorder=10
    )
    axis.set_title(title, fontsize=12, fontweight="bold")
    axis.set_xlabel("Odom X (m)")
    axis.set_ylabel("Odom Y (m)")
    axis.set_aspect("equal", adjustable="datalim")
    style_axis(axis)
    axis.legend(loc="best", fontsize=7.5, framealpha=0.92)


def mode_figure(data, output_dir):
    mode = data["mode"]
    summary = data["summary"]
    robot = data["robot"]
    times = numeric(robot, "sim_time")
    speeds = numeric(robot, "speed")
    yaw_rates = np.abs(numeric(robot, "wz"))
    errors = numeric(robot, "planned_path_error")
    clearances = numeric(robot, "safety_clearance")
    yaw = numeric(robot, "yaw")

    figure = plt.figure(figsize=(14, 8.5), constrained_layout=True)
    grid = figure.add_gridspec(2, 2, width_ratios=(1.35, 1.0))
    trajectory_axis = figure.add_subplot(grid[:, 0])
    motion_axis = figure.add_subplot(grid[0, 1])
    error_axis = figure.add_subplot(grid[1, 1])

    mode_name = "point-goal planning" if mode == 1 else "reference-curve planning"
    draw_trajectory(trajectory_axis, data, f"Mode {mode}: {mode_name}")

    motion_axis.plot(times, speeds, color=COLORS["speed"], label="Linear speed (m/s)")
    motion_axis.plot(times, yaw_rates, color=COLORS["yaw_rate"], label="Abs yaw rate (rad/s)")
    motion_axis.plot(times, np.abs(yaw), color=COLORS["planned"], alpha=0.8, label="Abs yaw (rad)")
    motion_axis.set_title("Robot motion over simulation time", fontsize=11, fontweight="bold")
    motion_axis.set_xlabel("Simulation time (s)")
    motion_axis.set_ylabel("Magnitude")
    style_axis(motion_axis)
    motion_axis.legend(fontsize=8)

    error_axis.plot(times, errors * 1000.0, color=COLORS["error"], label="Tracking error")
    error_axis.plot(
        times,
        clearances * 1000.0,
        color=COLORS["clearance"],
        label="Safety clearance",
    )
    error_axis.axhline(
        0.0,
        color=COLORS["obstacle"],
        linewidth=1.1,
        label="Intrusion threshold",
    )
    error_axis.axhline(
        25.0,
        color=COLORS["neutral"],
        linestyle="--",
        linewidth=1.1,
        label="Completion position tolerance",
    )
    error_axis.scatter(
        [times[-1]], [summary["final_position_error_m"] * 1000.0],
        color=COLORS["inflation"], marker="*", s=95,
        label="Final goal error", zorder=5,
    )
    error_axis.set_title("Tracking error and obstacle clearance", fontsize=11, fontweight="bold")
    error_axis.set_xlabel("Simulation time (s)")
    error_axis.set_ylabel("Distance (mm)")
    style_axis(error_axis)
    error_axis.legend(fontsize=8)

    figure.suptitle(
        "WCR Gazebo trajectory result | Mode {} | completed={} | "
        "safe={} | samples={:,} | path={:.3f} m | final error={:.2f} mm | "
        "min safety clearance={:.2f} mm".format(
            mode,
            summary["reached"],
            not summary["safety_violation_detected"],
            summary["samples"],
            summary["actual_path_length_m"],
            summary["final_position_error_m"] * 1000.0,
            summary["minimum_actual_safety_clearance_m"] * 1000.0,
        ),
        fontsize=14,
        fontweight="bold",
    )
    figure.savefig(output_dir / f"mode{mode}_result.png", dpi=180)
    figure.savefig(output_dir / f"mode{mode}_result.svg")
    plt.close(figure)


def comparison_figure(modes, output_dir):
    figure = plt.figure(figsize=(15, 10), constrained_layout=True)
    grid = figure.add_gridspec(2, 2, height_ratios=(1.35, 1.0))
    for index, data in enumerate(modes):
        name = "Point goal" if data["mode"] == 1 else "Reference curve"
        draw_trajectory(
            figure.add_subplot(grid[0, index]), data, f"Mode {data['mode']}: {name}"
        )

    length_axis = figure.add_subplot(grid[1, 0])
    error_axis = figure.add_subplot(grid[1, 1])
    labels = [f"Mode {data['mode']}" for data in modes]
    positions = np.arange(len(labels))
    width = 0.20
    target_lengths = [data["summary"]["target_path_length_m"] for data in modes]
    planned_lengths = [data["summary"]["planned_path_length_m"] for data in modes]
    actual_lengths = [data["summary"]["actual_path_length_m"] for data in modes]
    length_axis.bar(positions - width, target_lengths, width, color=COLORS["target"], label="Target/reference")
    length_axis.bar(positions, planned_lengths, width, color=COLORS["planned"], label="Planned")
    length_axis.bar(positions + width, actual_lengths, width, color=COLORS["actual"], label="Actual")
    length_axis.set_xticks(positions, labels)
    length_axis.set_ylabel("Path length (m)")
    length_axis.set_title("Path-length comparison", fontsize=11, fontweight="bold")
    style_axis(length_axis)
    length_axis.legend(fontsize=8)

    final_errors = [data["summary"]["final_position_error_m"] * 1000.0 for data in modes]
    mean_errors = [data["summary"]["mean_planned_path_error_m"] * 1000.0 for data in modes]
    max_errors = [data["summary"]["max_planned_path_error_m"] * 1000.0 for data in modes]
    clearances = [
        data["summary"]["minimum_actual_safety_clearance_m"] * 1000.0
        for data in modes
    ]
    error_axis.bar(positions - 1.5 * width, mean_errors, width, color=COLORS["speed"], label="Mean tracking")
    error_axis.bar(positions - 0.5 * width, max_errors, width, color=COLORS["error"], label="Max tracking")
    error_axis.bar(positions + 0.5 * width, final_errors, width, color=COLORS["inflation"], label="Final goal")
    error_axis.bar(
        positions + 1.5 * width,
        clearances,
        width,
        color=COLORS["clearance"],
        label="Min safety clearance",
    )
    error_axis.axhline(25.0, color=COLORS["neutral"], linestyle="--", linewidth=1.1, label="25 mm completion tolerance")
    error_axis.set_xticks(positions, labels)
    error_axis.set_ylabel("Distance (mm)")
    error_axis.set_title("Tracking, completion, and safety clearance", fontsize=11, fontweight="bold")
    style_axis(error_axis)
    error_axis.legend(fontsize=8)

    figure.suptitle(
        "WCR Gazebo planner test comparison | both modes completed | zero safety intrusions",
        fontsize=15,
        fontweight="bold",
    )
    figure.savefig(output_dir / "modes_comparison.png", dpi=180)
    figure.savefig(output_dir / "modes_comparison.svg")
    plt.close(figure)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    arguments = parser.parse_args()
    arguments.output.mkdir(parents=True, exist_ok=True)

    modes = [load_mode(arguments.input, mode) for mode in (1, 2)]
    for mode in modes:
        mode_figure(mode, arguments.output)
    comparison_figure(modes, arguments.output)


if __name__ == "__main__":
    main()
