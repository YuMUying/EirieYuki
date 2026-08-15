# 计算设备部署与时间同步

## 推荐方案：三设备

| 设备 | 运行模块 | 原因 |
| --- | --- | --- |
| A 底盘实时设备 | 电机/磁吸安全、底盘控制、INS/EKF、滑轨安全控制、时间同步服务 | 安全闭环不受 GPU、相机或超声负载影响；作为机器人统一时间权威 |
| B 视觉设备 | 顶部 D405、探头 D405、深度对齐、分割/中心线、三维转换、`ProbeAlignment` 发布 | GPU/USB 带宽集中管理，两台相机按序列号绑定 |
| C 超声设备 | 采集卡驱动、FMC/PWI/TOFD 原始数据、在线预处理/成像、上下文查询 | 采集卡吞吐和磁盘 I/O 与视觉隔离 |

设备 A 保留所有停止、限位、归零、行程和 watchdog 权限。视觉设备只能发布目标/误差，不能直接驱动电机。超声设备不能承担底盘安全控制。

## 可接受的两设备方案

- 设备 A：底盘控制、INS/EKF、滑轨安全控制、同步服务。
- 设备 B：视觉与超声采集/成像。

只有在双 D405、GPU 推理、超声吞吐和磁盘写入压力测试均满足实时要求时采用。必须为设备 B 设置 CPU 核、GPU 显存、USB 控制器和磁盘带宽预算。不得把视觉/超声高负载进程与底盘安全闭环合并到同一非实时主机。

## 网络与 DDS

- 三台设备使用有线千兆或更高带宽交换机，避免无线作为主数据链路。
- DDS 只传控制、状态、低带宽特征和必要预览；原始双相机视频和全 FMC 波形优先本机落盘或走专用数据通道。
- 固定 ROS domain、主机名和传感器序列号；按设备设置防火墙和 DDS discovery。
- 控制/状态使用 reliable；高频原始 IMU 和预览图像可用 sensor-data/best-effort。

## 统一时间处理

首选支持硬件时间戳的 IEEE 1588 PTP：

1. 设备 A 或专用 PTP Grandmaster 提供统一时钟。
2. 三机网卡用硬件时间戳同步，持续记录 offset、frequency adjustment 和 lock 状态。
3. ROS 系统时间与 PTP 时间一致，所有 Header 写采集时刻。
4. 相机 SDK 时间戳按 `timestamp_domain` 映射到 PTP/ROS 时间；不能直接把设备毫秒 tick 当 Unix 时间。
5. 超声采集卡保留原始硬件 tick，并用同机采样的 `(device_tick, ptp_time)` 映射到统一时间。
6. 网络接收时间单独保存，只用于延迟诊断。

若网卡或采集硬件不支持 PTP，降级使用 chrony/NTP，并实测最坏偏差。软件同步不能自动满足 5 ms 指标，必须通过记录的 offset 和回放验证后再启用空间拼接。

## 时间戳数据流

```text
camera/ultrasound hardware timestamp
        |
        v
local clock-domain mapper ----> unified capture_stamp (PTP/ROS)
        |                                  |
        |                                  v
raw tick + mapping saved       GetInspectionContext(stamp, task, index)
                                           |
                                           v
                         interpolated INS + IMU + rail state
                                           |
                                           v
                       timing_valid gate -> spatial fusion/storage
```

## INS/EKF

`wcr_launcher/config/ins_ekf.yaml` 使用 `robot_localization/ekf_node`：

- 100 Hz 输出 `/wcr/ins/odometry`
- 融合 `/wcr/odom` 的平面位置/速度/偏航角速度
- 融合 `/wcr/imu` 的姿态、角速度和线加速度
- 仿真时间跳变时重置

目标机需安装：

```bash
sudo apt install ros-jazzy-robot-localization
```

实机接入 INS 后，必须核对 ENU/NED、轴向、重力去除、磁航向、协方差和 frame ID。若驱动输出 NED，先转换到 ROS ENU；不能只改字段名。

## 启动与验收

1. 启动 PTP/chrony，确认三机锁定并记录 offset。
2. 设备 A 启动底盘、INS/EKF、滑轨驱动和 `inspection_context_server`。
3. 滑轨先归零，验证正方向是机器人左侧，验证正负限位与急停。
4. 设备 B 按序列号启动两台相机，验证各自内参、depth scale、时间戳域和 TF。
5. 设备 C 启动超声采集，验证每条扫描线的硬件时间戳和 sample index 单调。
6. 静止采集并比较三路 `InspectionContext` 偏差；再做匀速和滑轨往返测试。
7. 注入视觉丢帧、网络中断、低置信度、过期消息、限位和电机故障，确认滑轨保持/停止。
8. 只有 `timing_valid`、标定和失效测试均通过后，才允许自动焊缝跟踪和空间成像。

## 当前环境注意

ROS 2 Jazzy 的 `rosidl_cmake` 在本机对含中文的工作区绝对路径解析异常。项目可保留当前中文目录，但构建时应复制到纯 ASCII 的 WSL 路径，例如 `~/wcr_v1/ros2_ws`。这不影响 Python 视觉和超声代码。
