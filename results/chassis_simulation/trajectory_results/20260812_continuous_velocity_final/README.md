# WCR continuous velocity trajectory test

Test date: 2026-08-12

## Change under test

- Reference-curve points are interpreted as samples of one continuous curve.
- Collision-safe corner smoothing and 15 mm arc-length resampling are applied.
- Curvature, lateral acceleration, acceleration, and deceleration limits produce
  an explicit speed profile.
- The controller uses speed-dependent lookahead, tangent-speed feedforward, and
  cross-track feedback.
- Intermediate points never emit completion or request a stop.
- A 61-point visual curve is refreshed at simulation time 5 s; replacement
  preserves controller speed and safely reconnects from the active trajectory.
- Existing hard-boundary runtime intrusion detection remains enabled.

## Results

| Metric | Mode 1 | Mode 2 dense curve |
| --- | ---: | ---: |
| Input reference points | 2 | 61 |
| Planned trajectory samples | 68 | 79 initial, 52 after refresh |
| Reached | true | true |
| Safety intrusion samples | 0 | 0 |
| Minimum actual hard-boundary clearance | 28.703 mm | 28.457 mm |
| Interior low-speed events below 0.015 m/s | 0 | 0 |
| Minimum interior speed | 0.0765 m/s | 0.0478 m/s |
| Mean interior speed | 0.1034 m/s | 0.0882 m/s |
| Minimum speed within 0.75 s of refresh | n/a | 0.0478 m/s |
| Peak planned speed | 0.1300 m/s | 0.1300 m/s |
| Final position error | 13.075 mm | 5.736 mm |
| Simulation duration | 12.176 s | 13.880 s |

Mode 2 verifies the intended visual-control workload: 61 curve observations are
converted to one continuous speed trajectory and refreshed once while moving,
with no intermediate stop event. The controlled recovery section can enter the
0.20-0.25 m tracking allowance, but its minimum planned hard-boundary clearance
remains positive at 32.673 mm.

Raw odometry, requested points, planned geometry, planned speed profiles,
planner status, obstacle data, summaries, and figures are stored in this
directory.
