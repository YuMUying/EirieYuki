# Version 1 Organization Report

Date: 2026-08-14

## Active components

| Component | Code | Active data | Models/results |
| --- | --- | --- | --- |
| ROS 2 chassis simulation | `code/chassis_simulation/ros2_ws` | ROS package assets under `src` | `results/chassis_simulation` |
| Weld vision and RGB-D | `code/weld_vision/weld_seam_mvp` | `datasets/weld_vision/WES-Combined-Dataset` | `models/weld_vision/segmentation`, `results/weld_vision` |
| Phased-array and TOFD | `code/ultrasonic_imaging/ultrasonic-imaging` | `datasets/ultrasonic/Ultrasonic_Weld_Imaging` | `results/ultrasonic_imaging` |

## Source-to-destination mapping

- `wcr_gz_magnetic` -> `code/chassis_simulation/ros2_ws` (packages moved into standard `src`)
- `AI_work/weld_seam_mvp` -> `code/weld_vision/weld_seam_mvp`
- `ultrasonic-imaging` -> `code/ultrasonic_imaging/ultrasonic-imaging`
- `焊缝数据集/WES-Combined-Dataset` -> `datasets/weld_vision/WES-Combined-Dataset`
- `焊缝数据集/Ultrasonic_Weld_Imaging` -> `datasets/ultrasonic/Ultrasonic_Weld_Imaging`
- Vision `models`, `runs`, and `outputs` -> project-level `models` and `results`
- Ultrasonic example output -> `results/ultrasonic_imaging/examples`
- Chassis trajectory output -> `results/chassis_simulation/trajectory_results`

## Archived material

The following are preserved under `archive` and excluded from the active runtime path:

- Two old/duplicate `wcr_gz` downloads
- Unrelated `wall-climber-main` ROS project
- Jetson/Foxy physical-robot stack and bundled dependencies
- Five candidate/source weld datasets not used by the delivered centerline model
- Rebuildable WSL virtual environment, caches, package metadata, and old ROS logs

See `archive/README.md` for the detailed rationale.

## Path changes

- Added `project_paths.py` for the weld-vision component.
- Training, evaluation, inference, ONNX export, and RGB-D scripts now derive defaults from the version.1 root.
- Fixed the RGB-D end-to-end test after moving the delivered model.
- ROS trajectory scripts now derive their workspace and project result paths from their own location.
- Replaced active absolute Windows/WSL paths in component and dataset documentation.
- Added a project-level `.gitignore` for generated environments, caches, and ROS build products.

## Validation

- Weld vision: 24 unit/integration tests passed, including ONNX RGB-D end-to-end inference and rail-mounted camera transforms.
- Ultrasonic imaging: 8 pytest tests passed.
- ROS 2 workspace: `colcon list` found 11 packages; all 10 packages required by `wcr_launcher` built successfully in a clean ASCII-path verification workspace.
- ROS interfaces/control: 20 tests passed across message generation, rail control, timestamp interpolation and controller launch linting.
- ROS rail smoke test: a timestamped 3 mm visual correction drove the mock rail to a 3 mm encoder state.
- Shell scripts: `bash -n` passed for visual training and ROS trajectory runners.
- New visual, rail-control, sensor-sync and launcher Python files passed syntax checks.
- Active path scan: no references to the former `AI_work`, `焊缝数据集`, `wcr_gz_magnetic`, or old absolute workspace paths.
- Image files were not opened during organization or validation.

Physical RealSense, ultrasonic hardware, PTP and real linear-motor tests remain pending. The local WSL also needs `ros-jazzy-robot-localization` before the complete launcher can run.
