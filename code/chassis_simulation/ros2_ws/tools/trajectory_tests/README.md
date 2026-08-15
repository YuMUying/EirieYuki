# WCR planner trajectory tests

The runner starts a fresh headless Gazebo process for each planning mode,
publishes deterministic obstacles and targets, and writes plots plus raw data
to the project-level `results/chassis_simulation/trajectory_tests` directory.

Run from WSL after building the workspace:

```bash
cd /path/to/version.1/code/chassis_simulation/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
bash tools/trajectory_tests/run_all.sh
```

Set `WCR_WORKSPACE` only when invoking the scripts from a copied or nonstandard
workspace. The scripts normally derive the workspace and result paths from
their own location.

Each `mode1` or `mode2` result directory contains trajectory plots, CSV inputs
and outputs, `summary.json`, `simulation.log`, and `recorder.log`.
