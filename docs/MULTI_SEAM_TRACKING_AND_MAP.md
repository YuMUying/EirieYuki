# 多焊缝跟踪与环境地图记忆

## 1. 功能边界

本模块位于视觉识别和滑轨执行器之间。视觉计算机负责从同一采集时刻的 RGB-D 数据中输出一条或多条米制三维焊缝中心线；底盘计算机结合采集时刻的 INS 里程计、当前运动方向、滑轨位置和历史地图，选择本次应跟踪的焊缝，并只把被选焊缝的横向误差送入原有滑轨安全控制器。

模块不直接驱动电机，因此不会绕过 `probe_rail_controller` 中的置信度门限、消息年龄、死区、单步修正、软行程、限位、归零、故障保护和视觉 watchdog。

## 2. 数据流

```text
RGB-D 分割与多中心线提取
  -> /wcr/weld_seam_candidates
  -> seam_tracking_manager
       + /wcr/ins/odometry（按图像采集时刻插值）
       + 环境地图中的历史焊缝 ID
  -> /wcr/weld_seam_selection（选择结果与诊断）
  -> /wcr/probe_alignment（被选焊缝的米制横向误差）
  -> probe_rail_controller
  -> /wcr/linear_motor/command

/wcr/obstacles（障碍物观测）
  -> seam_tracking_manager（坐标转换、增量合并、持久化）
  -> /wcr/mapped_obstacles
  -> online_trajectory_planner
```

当前地图是储罐表面在 `odom` 坐标系中的二维展开地图，不是三维占据栅格。焊缝点和障碍物几何均使用米、弧度和 SI 单位。

## 3. 视觉候选接口

输入话题：`/wcr/weld_seam_candidates`

消息类型：`wcr_planning_msgs/msg/WeldSeamCandidateArray`

关键约定如下：

| 字段 | 约定 |
| --- | --- |
| `header.stamp` | RGB-D 图像的硬件采集时刻，不能使用网络接收时刻 |
| `header.frame_id` | 候选点所在坐标系，只接受 `base_surface` 或兼容的 `base_link` |
| `camera_frame` | 真实相机 optical frame，例如 `probe_camera_color_optical_frame` |
| `task_id`、`sample_index` | 一次检测任务内可追溯且单调递增的样本索引 |
| `candidate.points` | RGB-D 投影后的有序三维折线，控制与地图当前使用其 `x/y` |
| `candidate.confidence` | 分割置信度与有效深度比例形成的几何置信度，范围 `[0,1]` |
| `candidate.rail_position_at_capture_m` | 图像采集时刻插值得到的滑轨编码器位置 |
| `candidate.valid` | 深度、点数、标定和时钟均有效时才为真 |

同一帧内 `observation_id` 必须唯一。视觉侧可以先按像素中心线距离去重，底盘侧仍会按米制对称中心线距离二次去重。默认米制去重阈值为 `0.015 m`。

## 4. 多焊缝判定与选择

### 4.1 不同焊缝判定

设两条候选折线点集为 `A`、`B`，使用对称最近点平均距离：

```text
d(A,B) = 1/2 * [mean(a in A) min(b in B) ||a-b||
              + mean(b in B) min(a in A) ||b-a||]
```

当 `d(A,B) < minimum_seam_separation_m` 时，两者视为同一中心线的重复输出，只保留置信度较高者；否则作为独立焊缝参与选择。

### 4.2 行驶状态

候选消息到达后，管理器按 `header.stamp` 在里程计缓存中插值位姿和速度。最近里程计与图像时刻相差超过 `maximum_odometry_offset_s` 时，本帧选择无效，也不会下发滑轨修正。

速度大于 `minimum_motion_speed_m_s` 时，局部行驶单位向量为：

```text
t_v = (v_x, v_y) / sqrt(v_x^2 + v_y^2)
```

速度低于门限时按机器人正向 `+X` 处理。倒车时 `v_x < 0`，前视评分自然转向车后方，因此不需要单独的倒车分支。

### 4.3 候选评分

每条候选中心线使用 PCA 主方向 `t_s`，综合以下项目：

```text
S = w_c C
  + w_h |t_s dot t_v|
  + w_l exp(-|e_y| / e_max)
  + w_f exp(-|s_target - s_lookahead| / s_lookahead)
  + w_m I(same mapped seam)
```

其中：

| 符号 | 含义 |
| --- | --- |
| `C` | 候选置信度 |
| `e_y` | 候选中心线中位横向位置减去采集时刻的探头声学中心位置 |
| `s_target` | 候选点沿当前行驶方向的前视投影 |
| `I` | 候选仍关联到上一条地图焊缝时为 1 |

默认权重为置信度 `0.20`、方向 `0.32`、横向可达性 `0.24`、前视位置 `0.14`、地图连续性 `0.10`。当新焊缝得分未超过当前焊缝 `switch_score_margin=0.12` 时继续保持当前焊缝，防止十字或 T 形交汇处来回切换。

选择输出：

| 话题 | 用途 |
| --- | --- |
| `/wcr/weld_seam_selection` | 候选数、选中观测 ID、地图焊缝 ID、得分、是否切换及失败原因 |
| `/wcr/probe_alignment` | 原滑轨控制器所需的米制横向误差、采集时滑轨位置和置信度 |

## 5. 地图记忆

### 5.1 焊缝关联

候选中心线通过采集时刻的 `odom -> base_surface` 二维位姿转换到 `odom`。历史关联同时检查主方向差和“当前短视野观测到历史折线”的最近点平均距离。采用有向距离是为了避免历史焊缝逐渐变长后，整条历史曲线反向拉高距离并把同一物理焊缝错误拆成多个 ID。

同一帧的不同候选禁止关联到同一个地图 ID。地图记录：中心线点、平均置信度、观测次数、首次时间、末次时间和当前是否被选中。折线点超过上限时沿主轴分桶压缩，限制长期内存和 JSON 大小。

### 5.2 障碍物

`/wcr/obstacles` 是观测入口，支持 `odom`、`base_surface` 和 `base_link`。车体坐标观测必须带采集时间戳，管理器用同一里程计缓存转换到 `odom`。

普通 `ObstacleArray` 只做增量合并；某个障碍物暂时离开视野不会从地图消失。显式删除和清空使用 `/wcr/obstacle_update`：

| 操作 | 行为 |
| --- | --- |
| `UPSERT` | 按 ID 新增或替换；`ObstacleUpdate` 当前没有 Header，因此几何约定已在 `odom` |
| `REMOVE` | 按 `obstacle.id` 删除 |
| `CLEAR` | 清空全部障碍物 |

管理器把完整持久障碍物快照发布到 `/wcr/mapped_obstacles`。集成启动文件已把规划器输入切换到该话题。

### 5.3 持久化

默认路径：`~/.ros/wcr_environment_map.json`

地图使用临时文件、`fsync` 和原子替换保存；启动时自动恢复，并以 transient-local QoS 发布地图和规划器障碍物。可用服务：

```bash
ros2 service call /wcr/environment_map/save std_srvs/srv/Trigger '{}'
ros2 service call /wcr/environment_map/clear std_srvs/srv/Trigger '{}'
```

完整地图话题为 `/wcr/environment_map`，消息类型为 `EnvironmentMap`。

## 6. 关键参数

参数位于 `wcr_launcher/config/launch_params.yaml`：

| 参数 | 默认值 | 作用 |
| --- | ---: | --- |
| `seam_tracking_maximum_odometry_offset_s` | 0.10 s | 图像与可用里程计的最大时差 |
| `seam_tracking_minimum_candidate_points` | 5 | 有效中心线最少点数 |
| `seam_tracking_minimum_separation_m` | 0.015 m | 同帧候选去重距离 |
| `seam_tracking_minimum_motion_speed_m_s` | 0.01 m/s | 判定运动方向的速度门限 |
| `seam_tracking_lookahead_m` | 0.08 m | 沿行驶方向的评分前视距离 |
| `seam_tracking_maximum_lateral_error_m` | 0.10 m | 滑轨可达性评分尺度 |
| `seam_tracking_switch_score_margin` | 0.12 | 换线滞回分数 |
| `seam_map_association_distance_m` | 0.04 m | 历史焊缝空间关联门限 |
| `seam_map_association_angle_deg` | 30 deg | 历史焊缝方向关联门限 |
| `environment_map_path` | `~/.ros/wcr_environment_map.json` | 地图持久化文件 |

这些值是软件联调初值。实机必须用相机视场、滑轨有效行程、机器人速度、INS 漂移和焊缝间距重新标定。

## 7. 降级与安全行为

以下情况只发布无效选择，不发布新的 `/wcr/probe_alignment`：候选坐标系错误、采集时里程计不可用、候选点不足、置信度不足、数值非有限或没有可用焊缝。原滑轨控制器随后由 alignment watchdog 停止视觉跟踪运动。

地图不是安全传感器的替代品。急停、限位、驱动器故障、吸附力不足和实时防碰撞应继续由底盘安全链路处理；JSON 地图主要用于焊缝连续性选择、任务恢复和在线轨迹规划。

## 8. 验证方法

纯算法测试：

```bash
cd code/chassis_simulation/ros2_ws/src/wcr_probe_control
python -m pytest test -q
```

ROS 2 构建后冒烟测试会实际启动 `seam_tracking_manager`，发送一条顺行焊缝、一条交叉焊缝和一个障碍物，并检查选择、滑轨对准、地图发布、规划器障碍物与保存服务：

```bash
python3 tools/seam_tracking_smoke_test.py --map-path /tmp/wcr_environment_map.json
```
