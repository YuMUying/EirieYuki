# WCR Gazebo Trajectory Results

Source run: `20260812_161900_completion_rerun`

Both planner modes completed successfully under the following criteria:

- Planner published `/wcr/target_reached=true`.
- Planner status reached `completed`.
- Final position error was at most 0.025 m.
- Final yaw error was at most 0.05 rad.
- The final pose remained settled for 20 consecutive odometry samples.

## Result figures

- `figures/mode1_result.png` and `.svg`: point-goal trajectory, physical
  obstacles, inflated exclusion boundaries, requested target, collision-free
  path, robot odometry, speed, yaw rate, and tracking error.
- `figures/mode2_result.png` and `.svg`: reference-curve trajectory with the
  same supporting data.
- `figures/modes_comparison.png` and `.svg`: side-by-side trajectories plus
  path-length and error comparisons.

## Mode 1

- Samples: 12,311
- Simulation duration: 12.321 s
- Planned waypoints: 5
- Actual path length: 0.910917 m
- Final position error: 0.006680 m
- Mean planned-path error: 0.003905 m
- Maximum planned-path error: 0.012531 m

## Mode 2

- Samples: 13,606
- Simulation duration: 13.619 s
- Planned waypoints: 6
- Actual path length: 0.982815 m
- Final position error: 0.014934 m
- Mean planned-path error: 0.006956 m
- Maximum planned-path error: 0.034610 m

## Logs

Each `mode1` and `mode2` directory contains:

- `robot_trajectory.csv`: timestamped pose, velocity, yaw rate, speed, and
  planned-path error.
- `target_trajectory.csv`: requested point goal or reference curve.
- `planned_path.csv`: final collision-free planner output.
- `planned_path_history.csv`: all recorded replans.
- `obstacles.csv`: obstacle geometry and inflation radius.
- `planner_status.csv`: planner state transitions.
- `summary.json`: completion and aggregate metrics.
- `simulation.log` and `recorder.log`: ROS 2, Gazebo, controller, planner, and
  recorder diagnostics.
- `trajectory_report.png` and `.svg`: report generated during the original run.

`combined_summary.json` contains both mode summaries.
