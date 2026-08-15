# WCR online trajectory planner

The planner works in the two-dimensional `odom` plane. Distances are metres and
angles are radians.

## Modes

- Mode 1 (`/wcr/planning_mode = 1`) plans from odometry to a single
  `geometry_msgs/PoseStamped` goal on `/wcr/target_pose`.
- Mode 2 (`/wcr/planning_mode = 2`) treats every pose received on
  `/wcr/reference_curve` as a sample of one continuous reference curve. These
  samples are not stop points.

Both modes use the same obstacle-aware continuous trajectory controller.

## Planning pipeline

1. Connect collision-free reference sections and use 8-connected A* around
   blocked sections.
2. Inflate obstacles by the robot radius, hard safety margin, and tracking
   allowance during planning.
3. Smooth corners only when every replacement segment remains outside the
   planning boundary.
4. Resample the path at uniform arc-length intervals.
5. Estimate curvature and apply the lateral-acceleration speed limit
   `v <= sqrt(a_lateral / abs(curvature))`.
6. Run forward and backward passes to enforce acceleration and braking limits.
7. Publish the result on `/wcr/planned_trajectory` as pose, arc length,
   curvature, and desired speed samples.

The controller projects the current robot position onto the trajectory, selects
a speed-dependent lookahead point, and combines tangent-speed feedforward with
cross-track feedback. Only the final trajectory point can publish completion.

A new mode-2 curve can replace an active trajectory without resetting the
controller speed. If tracking error has temporarily placed the robot inside the
planning allowance, the planner follows a short hard-safe section of the old
trajectory until full planning clearance is recovered, then joins the new curve.

## Topics

| Topic | Type | Purpose |
| --- | --- | --- |
| `/wcr/planning_mode` | `std_msgs/msg/UInt8` | Select goal or curve mode |
| `/wcr/mapped_obstacles` | `wcr_planning_msgs/msg/ObstacleArray` | Persistent obstacle snapshot from environment memory |
| `/wcr/mapped_obstacle_update` | `wcr_planning_msgs/msg/ObstacleUpdate` | Optional direct planner-only incremental update |
| `/wcr/target_pose` | `geometry_msgs/msg/PoseStamped` | Mode 1 goal |
| `/wcr/reference_curve` | `nav_msgs/msg/Path` | Mode 2 discrete curve observations |
| `/wcr/planned_path` | `nav_msgs/msg/Path` | Geometric path for visualization |
| `/wcr/planned_trajectory` | `wcr_planning_msgs/msg/TimedTrajectory` | Continuous speed trajectory |
| `/wcr/planner_status` | `std_msgs/msg/String` | Planner state or failure reason |
| `/wcr/target_reached` | `std_msgs/msg/Bool` | Whole-trajectory completion |

All geometric inputs must use `header.frame_id: odom`.

Raw obstacle observations enter on `/wcr/obstacles` and are merged by
`seam_tracking_manager`; the integrated launcher does not feed them directly
to the planner. See `docs/MULTI_SEAM_TRACKING_AND_MAP.md`.

## Safety

The hard execution boundary is `robot_radius + safety_margin`. The planning
boundary adds `tracking_error_allowance`. While executing, the planner checks
every odometry position and each segment between adjacent positions against the
hard boundary. Any intrusion cancels the controller immediately and publishes a
`safety_violation` status.

The trajectory and controller parameters are configured in
`wcr_launcher/config/launch_params.yaml`.
