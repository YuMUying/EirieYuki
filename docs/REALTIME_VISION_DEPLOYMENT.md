# 双相机焊缝定位实时部署

## 1. 目标与边界

探头相机以最高 30 Hz 输出三维焊缝候选，为横向滑轨提供定位依据；顶部相机以 12 Hz 更新全局焊缝地图。视觉节点不直接控制电机：探头候选必须依次经过多焊缝选择、地图关联、米制对中误差生成和滑轨安全控制，继续受置信度、消息年龄、软行程、限位和 watchdog 约束。顶部候选只更新地图，禁止触发滑轨对中。

Python 只承担 ROS 2 调度、数组组织和后处理，网络推理由 ONNX Runtime 原生执行。模型会话、相机配置和线程均在节点启动时创建一次，实时循环不加载模型、不读写图片、不输出逐帧 JSON/CSV。

## 2. 实时数据流

```text
aligned color + depth + camera info
              |
              v
 capture-time synchronization gate
              |
              v
 depth-one latest-frame slot  ---> stale frame replaced, never queued
              |
              v
 ONNX segmentation at 192 x 192
              |
              v
 centerline extraction in unpadded model region
              |
              v
 exact point mapping to camera pixels
              |
              v
 depth sampling + dynamic camera transform
              |
              v
 WeldSeamCandidateArray + VisionTiming + DeviceState
```

探头相机还会缓存 100 Hz 滑轨状态，并在图像采集时刻插值编码器位置。彩色与深度时间差、采集年龄或滑轨时间差超过门限时拒绝该帧，不使用接收时刻或最新编码器值替代。

## 3. 调度与话题

| 角色 | 目标率 | 候选输出 | 下游用途 |
| --- | ---: | --- | --- |
| 探头相机 | 30 Hz | `/wcr/weld_seam_candidates` | 多焊缝选择、地图关联、滑轨对中 |
| 顶部相机 | 12 Hz | `/wcr/top_weld_seam_candidates` | 全局焊缝地图，不驱动滑轨 |

诊断输出为 `/wcr/vision/probe/timing`、`/wcr/vision/top/timing` 和对应 `state` 话题。`VisionTiming` 包含单帧处理时间、采集到发布的端到端年龄、滚动 P95、有效频率、处理数、丢帧数和 deadline 状态。

队列深度固定为一。处理跟不上时替换尚未执行的旧帧并累计丢帧数，禁止为了“每帧都处理”形成越来越长的控制延迟。

## 4. 计算基准

结构化结果见 `results/weld_vision/realtime_benchmark_20260817.json`。测试环境为 i5-12600K、WSL2、ONNX Runtime CPU 后端，输入使用随机 `640 x 480` RGB-D 数组和人工掩膜，没有读取项目图片。100 轮结果如下：

| 环节 | P50 | P95 |
| --- | ---: | ---: |
| 分割预处理与 ONNX 推理 | 14.60 ms | 18.18 ms |
| 三维投影 | 3.81 ms | 5.97 ms |
| 优化后完整计算链 | 19.93 ms | 23.04 ms |

30 Hz 帧周期为 33.33 ms，因此当前 CPU 对单路探头视觉具有计算余量。该基准不含曝光、相机 SDK、USB、ROS 序列化和下游控制，不能替代实机端到端测试。顶部和探头同时运行时还必须验证 CPU 竞争与 USB 控制器带宽。

重复基准：

```bash
python benchmark_realtime.py \
  --runs 100 \
  --output results/weld_vision/realtime_benchmark.json
```

## 5. 部署环境

ROS 工作空间必须复制到纯 ASCII 的 Linux 文件系统路径。视觉节点需要同时导入 ROS 2 的 `rclpy` 与 ONNX/OpenCV 依赖，因此使用带系统 site-packages 的专用环境：

```bash
source /opt/ros/jazzy/setup.bash
cd /ascii/path/project/code/weld_vision/weld_seam_mvp
bash setup_ros_runtime.sh
source .venv-ros/bin/activate

cd ../../chassis_simulation/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-up-to wcr_vision wcr_probe_control
source install/setup.bash
```

两台 RealSense 必须按序列号分别启动，输出对齐到彩色图的深度、彩色 CameraInfo 和统一时基的采集时间戳。随后启动：

```bash
ros2 launch wcr_vision dual_camera_realtime.launch.py \
  model_path:=/absolute/project/models/weld_vision/segmentation/weld_segmentation.onnx \
  top_camera_config:=/absolute/project/code/weld_vision/weld_seam_mvp/config/d405_mount.yaml \
  probe_camera_config:=/absolute/project/code/weld_vision/weld_seam_mvp/config/probe_d405_mount.yaml
```

机械设计外参、深度比例和探头声学中心偏置只用于软件联调，自动跟踪前必须替换为实机标定值。

## 6. 实机验收

连续运行至少 30 分钟，并分别执行静止、底盘匀速、滑轨往返、双相机并发和超声同时采集。探头视觉建议验收条件：

| 指标 | 建议门限 |
| --- | ---: |
| 有效发布率 | P50 不低于 27 Hz |
| 单帧计算 | P95 不高于 30 ms |
| 采集到发布端到端年龄 | P95 不高于 80 ms，P99 不高于 100 ms |
| 彩色/深度采集时差 | 不高于 12 ms |
| 滑轨插值最近样本偏差 | 不高于 20 ms |
| 连续超时 | 超过 250 ms 时滑轨保持 |

同时记录 `VisionTiming`、候选、对中消息、滑轨命令/状态、底盘速度和时钟同步状态。低置信度、深度无效、时间戳倒退、输入中断、处理超时和限位故障均要做注入测试。任何 deadline 失败都应计数并可追溯，不能只报告平均 FPS。
