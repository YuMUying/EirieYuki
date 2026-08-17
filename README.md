# 爬壁焊缝检测机器人 version.1

本目录是项目的正式工作区，包含机器人底盘 ROS 2 仿真、焊缝中心线识别与 RGB-D 三维坐标转换、相控阵/TOFD 超声成像三部分。代码、数据集、模型和运行结果已分离，旧下载工程与未进入当前方案的数据保留在 `archive`，不参与默认运行。

## 目录结构

```text
version.1/
├── code/                         # 三个正式代码组件
│   ├── chassis_simulation/       # ROS 2 + Gazebo 底盘仿真
│   ├── weld_vision/              # 焊缝分割、中心线和 RGB-D 三维投影
│   └── ultrasonic_imaging/       # FMC/PWI 相控阵和 TOFD
├── datasets/                     # 当前代码实际使用的数据集
│   ├── weld_vision/
│   └── ultrasonic/
├── models/                       # 可交付模型和模型说明
├── results/                      # 训练、推理、评估和仿真结果
├── docs/                         # 项目级设计与导读
└── archive/                      # 旧工程、实机代码和候选数据集
```

## 项目入口

- [统一研发、部署、运行与验收工作流](docs/PROJECT_WORKFLOW.md)
- [项目归档与 GitHub 发布清单](docs/ARCHIVE_MANIFEST.md)
- [数据集恢复与使用边界](datasets/README.md)
- [底盘数据契约](docs/CHASSIS_DATA_CONTRACT.md)
- [部署与时间同步](docs/DEPLOYMENT_AND_TIME_SYNC.md)
- [双相机焊缝定位实时部署](docs/REALTIME_VISION_DEPLOYMENT.md)
- [探头相机与滑轨](docs/PROBE_CAMERA_AND_RAIL.md)
- [多焊缝跟踪与地图记忆](docs/MULTI_SEAM_TRACKING_AND_MAP.md)

## 1. 底盘 ROS 2 仿真

工作空间：`code/chassis_simulation/ros2_ws`

ROS 2 Jazzy 的接口生成器在含中文的绝对路径下可能失败。保留本项目目录不变即可，但构建前应在 WSL 中将 `ros2_ws` 复制到纯 ASCII 路径（如 `~/wcr_v1/ros2_ws`）。目标机还需安装 INS 融合依赖：

```bash
sudo apt install ros-jazzy-robot-localization
```

```bash
cd code/chassis_simulation/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
ros2 launch wcr_launcher launcher.launch.py
```

轨迹测试：

```bash
bash tools/trajectory_tests/run_all.sh
```

测试结果默认写入 `results/chassis_simulation/trajectory_tests`。已整理的历史轨迹结果位于 `results/chassis_simulation/trajectory_results`。

底盘仿真现已包含前端横向超声探头滑轨、视觉误差安全控制、mock 电机和采集时刻 INS/IMU/滑轨同步服务。接口与参数见 [探头相机与滑轨](docs/PROBE_CAMERA_AND_RAIL.md) 和 [底盘数据契约](docs/CHASSIS_DATA_CONTRACT.md)。

## 2. 焊缝视觉与三维坐标

代码：`code/weld_vision/weld_seam_mvp`

数据集：`datasets/weld_vision/WES-Combined-Dataset`

交付模型：`models/weld_vision/segmentation`

```bash
cd code/weld_vision/weld_seam_mvp
bash setup_wsl.sh
source .venv/bin/activate
python -m unittest discover -s tests -v
```

主要脚本已从自身位置自动推导项目根目录。训练、评估和 ONNX 导出可直接使用默认数据集与模型路径：

```bash
python train.py
python evaluate.py
python evaluate_centerline.py
python export_onnx.py
```

单张图像推理仍需提供输入，模型和输出目录已有默认值：

```bash
python infer.py --input /path/to/frame.png
```

RGB-D 相机选型、坐标系和标定要求见 `code/weld_vision/weld_seam_mvp/RGBD_D405_DESIGN.md`。
探头位置的第二台 D405 使用动态滑轨外参，不能复用顶部相机标定；具体命令见 [探头相机与滑轨](docs/PROBE_CAMERA_AND_RAIL.md)。

实机使用常驻 ROS 2 ONNX 节点，不循环调用单帧脚本。探头相机目标 30 Hz，顶部相机 12 Hz；最新帧队列、P95 延迟诊断、启动命令和验收门限见 [双相机焊缝定位实时部署](docs/REALTIME_VISION_DEPLOYMENT.md)。

## 3. 相控阵与 TOFD 超声

代码：`code/ultrasonic_imaging/ultrasonic-imaging`

数据：`datasets/ultrasonic/Ultrasonic_Weld_Imaging`

```powershell
cd code\ultrasonic_imaging\ultrasonic-imaging
python -m pip install -e ".[test]"
python -m pytest
```

FMC/PWI 与 TOFD 命令和数据格式见该组件的 `README.md`。输出建议统一写入 `results/ultrasonic_imaging`。

三模块的推荐三机部署、两机备选、PTP/chrony 降级和 INS 数据流见 [部署与时间同步](docs/DEPLOYMENT_AND_TIME_SYNC.md)。

## 归档原则

项目分为本机完整档案和 GitHub 轻量交付仓库两层。`archive`、原始数据集、临时目录和可重建产物没有删除，但不会进入普通 Git；目录归属、可信版本和发布审计见 [项目归档与发布清单](docs/ARCHIVE_MANIFEST.md)。

GitHub 保留正式代码、交付模型、结构化结果、技术文档、数据来源/许可清单和最终报告。Python 虚拟环境、缓存、ROS 构建目录、重复模型、结果预览图和大轨迹 CSV 均为本地或可再生成内容。关键交付物可用根目录 `SHA256SUMS.txt` 校验。
