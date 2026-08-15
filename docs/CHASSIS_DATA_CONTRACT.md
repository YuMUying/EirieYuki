# 底盘数据契约

## 坐标与时间基准

- `base_surface`：`+X` 机器人前进，`+Y` 机器人左，`+Z` 离开钢板。
- `odom`：一次任务内连续的局部惯性/里程计坐标系。
- `probe_rail_joint`：正位置沿 `base_surface +Y`。
- 相机 optical frame：`+X` 图像右，`+Y` 图像下，`+Z` 光轴前。
- 所有跨设备 Header 使用统一 PTP/ROS 时基，Header 表示采集时刻，不是消息接收时刻。
- SI 单位：位置 m、速度 m/s、角度 rad、角速度 rad/s、时间 s/ns。

## 视觉需要的底盘数据

| 数据 | ROS 接口 | 建议频率/QoS | 用途 |
| --- | --- | --- | --- |
| 融合位姿与速度 | `/wcr/ins/odometry`, `nav_msgs/Odometry` | 100 Hz，reliable，depth 20 | 顶部焊缝轨迹落到任务坐标系、运动补偿 |
| 原始 IMU/INS | `/wcr/imu`, `sensor_msgs/Imu` | 100-200 Hz，sensor-data/best-effort | 姿态、角速度、振动检测、融合诊断 |
| 滑轨编码器状态 | `/wcr/probe_rail/state`, `ProbeRailState` | 100 Hz，reliable，depth 100 | 探头相机采集时动态外参和控制反馈 |
| 动态/静态 TF | `/tf`, `/tf_static` | 标准 TF QoS | `odom -> base_link/base_surface -> rail -> camera/probe` |
| 底盘任务状态 | `TaskState`/任务 ID | 事件驱动，reliable | 帧归属、启停、异常处理 |
| 底盘速度命令/实际速度 | odometry twist 与控制状态 | 50-100 Hz | 模糊风险、曝光和降速策略 |
| 时间同步状态 | 系统监控字段 | 1-10 Hz | 判断图像能否参与空间拼接 |

每个探头相机结果必须至少保留：`task_id`、图像采集 `stamp`、相机序列号、帧 ID、滑轨采集时位置、模型版本、置信度、时间同步状态。顶部与探头相机不得只靠 USB 枚举顺序区分。

## 超声成像需要的底盘数据

| 数据 | 来源 | 绑定粒度 | 用途 |
| --- | --- | --- | --- |
| 统一采集时间戳 | 超声采集卡硬件时钟映射 | 每条 TOFD A 扫；每组 FMC/PWI 事件 | 请求同步上下文 |
| 位姿/速度 | `/wcr/ins/odometry` | 插值到采集时刻 | 将 B/C/D 扫或 TFM 图像映射到工件坐标 |
| IMU | `/wcr/imu` | 插值到采集时刻 | 姿态、角速度、振动与耦合异常诊断 |
| 滑轨状态 | `/wcr/probe_rail/state` | 插值到采集时刻 | 探头声学中心横向位置 |
| 任务与样本索引 | 任务管理器/采集程序 | 每条扫描线单调递增 | 数据关联和断点恢复 |
| 探头/楔块参数 | 超声配置 | 每次任务固定并版本化 | 阵元位置、PCS、楔块延迟、声速、采样率 |
| 设备与耦合状态 | 超声硬件 | 每条扫描线或状态变化 | 拒绝失耦、过载、丢通道数据 |

FMC/PWI 波形内部的 `time_offset_s + sample_index / sample_rate_hz` 是声传播时间轴，不用于查询机器人位姿。机器人采集时间应绑定整组 FMC/PWI 发射事件，若扫描期间机器人位移不可忽略，则保留每个发射事件的硬件时间戳。TOFD 的每一行 RF/A 扫对应一个机器人采集时间戳。

## 同步服务

超声或视觉记录端调用：

```text
/wcr/inspection_context : wcr_planning_msgs/srv/GetInspectionContext
```

请求包含 `stamp + task_id + sample_index`。响应 `InspectionContext` 包含插值后的 odometry、IMU、rail state，以及到最近原始样本的三个时间偏差。默认最大允许偏差为 `5 ms`：

- `timing_valid=true`：可用于空间拼接。
- `timing_valid=false`：原始数据仍保存，但禁止进入定量空间融合。
- `success=false`：缓存为空、四元数无效或无法插值，应记录原因并等待/降级。

## 数据保存最小字段

```text
task_id
sample_index
capture_stamp_ns
host_receive_stamp_ns
clock_source
clock_sync_valid
clock_offset_estimate_ns
frame_id / sensor_serial
inspection_context
time_offsets_s
raw_data_path
calibration_id
software_model_version
```

禁止用网络接收时间覆盖采集时间。必须同时保留采集卡原始 tick、映射后的统一时间和映射参数，便于离线重算。
