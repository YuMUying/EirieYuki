# WCR Realtime Weld Vision

This ROS 2 package runs the delivered ONNX weld model as persistent RGB-D workers. It does not read or write image files in the control path.

## Runtime contract

Each camera node subscribes to aligned color, depth and camera-info topics. The probe role also buffers rail encoder states and interpolates the rail position at the image capture timestamp. A depth-one queue always replaces pending work with the newest synchronized pair, preventing latency from growing when computation falls behind.

The probe node publishes `weld_seam_candidates` at up to 30 Hz. Existing seam selection and map memory consume this topic and publish `probe_alignment`; the rail controller therefore retains its confidence, age, limit and watchdog checks. The top node publishes `top_weld_seam_candidates` at up to 12 Hz for global mapping and task logic.

Each processed frame publishes `VisionTiming`, including processing time, end-to-end age, rolling P95, effective rate and dropped-frame count. `DeviceState` provides readiness and fault status.

## Build and launch

Create a ROS-aware virtual environment, then build the workspace while it is active. The workspace must be in a pure-ASCII Linux path because ROS interface generation is unreliable from this project's Windows-mounted path.

```bash
source /opt/ros/jazzy/setup.bash
cd /ascii/path/project/code/weld_vision/weld_seam_mvp
bash setup_ros_runtime.sh
source .venv-ros/bin/activate
cd ../../chassis_simulation/ros2_ws
colcon build --packages-up-to wcr_vision wcr_probe_control
source install/setup.bash
```

Launch both camera workers with calibrated configuration files:

```bash
ros2 launch wcr_vision dual_camera_realtime.launch.py \
  model_path:=/absolute/project/models/weld_vision/segmentation/weld_segmentation.onnx \
  top_camera_config:=/absolute/project/code/weld_vision/weld_seam_mvp/config/d405_mount.yaml \
  probe_camera_config:=/absolute/project/code/weld_vision/weld_seam_mvp/config/probe_d405_mount.yaml
```

Both RealSense devices must be started separately and bound by serial number. Their depth streams must already be aligned to their color streams. Replace both mechanical transforms, depth scales and intrinsics with physical calibration results before enabling rail tracking.

The top-camera candidates are map-only. Probe-camera candidates pass through multi-seam selection and map association before the existing rail controller receives an alignment; the vision node never commands the motor directly.

Run the repeatable, image-free compute benchmark with:

```bash
python benchmark_realtime.py --output /tmp/wcr_vision_benchmark.json
```
