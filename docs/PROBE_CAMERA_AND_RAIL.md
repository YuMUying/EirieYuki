# 探头相机选型与横向滑轨设计

## 结论

探头中间位置的小相机推荐使用第二台 **RealSense D405**。厂商产品页和 D400 系列资料给出的机械尺寸约为 `42 x 42 x 23 mm`，近距离工作范围起点约 `70 mm`，采用全局快门并支持 librealsense、Linux 和 ROS 2 wrapper。它略大于最初的 40 mm 限制，但符合后来“尺寸不要过大”的要求，而且可以复用顶部 D405 的驱动、深度对齐和三维投影代码。

采购前必须按最新机械图复核外形、光心、安装孔和 USB 插头空间，并在真实钢板、耦合剂、振动、遮挡和照明环境下做近距深度测试。规格参考：

- [RealSense D405 产品页](https://www.realsenseai.com/products/depth-camera-d405/)
- [librealsense SDK](https://github.com/realsenseai/librealsense)
- [RealSense ROS 2 wrapper](https://github.com/realsenseai/realsense-ros)

## 两台相机不能共用的参数

顶部相机和探头相机必须按序列号固定角色。每台分别保存：

- `serial_number`
- 实际流配置下的彩色内参和畸变参数
- 设备返回的 `depth_scale_m_per_unit`
- 相机到安装基准的外参
- 时间戳域及其到 PTP/ROS 时钟的映射状态

探头相机随滑轨运动，其采集时外参为：

```text
T_base_camera(q) = Translation(axis_base * (q - q_ref)) * T_base_camera(q_ref)
```

其中滑轨正方向为 `base_surface +Y`，即机器人左侧。代码位于 `weld_seam/rgbd_geometry.py`，配置位于 `config/probe_d405_mount.yaml`。如果动态相机没有提供采集时刻的滑轨位置，投影会直接报错，不会错误使用当前编码器位置。

## 采集与推理

顶部相机默认 `1280x720@30`，探头相机默认 `640x480@30`。双相机部署必须显式提供序列号：

```bash
python capture_realsense_rgbd.py \
  --role top --serial TOP_SERIAL --output captures/top/frame_0001

python capture_realsense_rgbd.py \
  --role probe --serial PROBE_SERIAL --output captures/probe/frame_0001
```

探头相机推理需要图像采集时刻的滑轨位置和同一时基的采集时间戳：

```bash
python infer_rgbd.py \
  --model /path/to/weld_segmentation.onnx \
  --color /path/to/color.png \
  --depth /path/to/depth.png \
  --intrinsics /path/to/intrinsics.json \
  --config config/probe_d405_mount.yaml \
  --rail-position 0.0124 \
  --capture-stamp-ns 1786675200123456789 \
  --task-id inspection-001 \
  --sample-index 42 \
  --output /path/to/output
```

自动控制目标使用：

```text
target_rail_position = rail_position_at_capture + lateral_error
```

视觉输出采用米制横向误差 `ProbeAlignment.msg`，不把像素误差直接送给电机。

## ROS 2 滑轨接口

高层输入：

- `/wcr/probe_rail/command`：`ProbeRailCommand`
- `/wcr/probe_alignment`：`ProbeAlignment`，Header 必须是图像采集时间

底层驱动接口：

- `/wcr/linear_motor/command`：`LinearMotorCommand`
- `/wcr/probe_rail/state`：`ProbeRailState`，Header 必须是编码器采样时间

控制器包含置信度门限、消息年龄门限、死区、单次修正限幅、软件行程、归零/故障/限位保护和视觉丢失 watchdog。仿真使用 `mock_linear_motor_driver`；真实电机型号确定后，只需实现同一底层接口并适配 CAN、RS-485 或脉冲驱动器。

URDF 已加入 `probe_rail_joint`、滑轨、探头、相机机身及 `probe_camera_color_optical_frame`。所有尺寸是软件联调初值，不是最终机械设计。

## 装机标定

1. 固定相机、保护结构和线缆应力释放，USB 弯曲力不能改变外参。
2. 用每台相机的序列号采集内参、畸变、深度 scale 和时间戳域。
3. 在滑轨参考位置标定 `T_base_camera(q_ref)`。
4. 用量块或编码器多位置采样，标定滑轨轴方向、零点、比例和反向间隙。
5. 标定探头声学中心相对滑块的 Y 偏置 `center_y_at_reference_m`。
6. 用独立位置验证“相机三维焊缝 Y、编码器 Y、探头声学中心 Y”的闭环误差。
7. 保护窗、支架、相机、分辨率或探头结构变化后重新标定。

D405 不是焊接专用防护相机。探头位置应设置可更换保护片、防耦合剂结构、遮光和散热空间，同时保证不会遮挡双目视场。
