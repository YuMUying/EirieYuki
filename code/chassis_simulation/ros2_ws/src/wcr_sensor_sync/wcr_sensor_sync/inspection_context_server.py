from __future__ import annotations

import copy

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from wcr_planning_msgs.msg import InspectionContext, ProbeRailState
from wcr_planning_msgs.srv import GetInspectionContext

from .time_buffer import (
    ImuSample,
    MotionSample,
    RailSample,
    TimeBuffer,
    interpolate_imu,
    interpolate_motion,
    interpolate_rail,
    nearest_offset_s,
)


def stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def xyz(message) -> np.ndarray:
    return np.array([message.x, message.y, message.z], dtype=np.float64)


def quaternion(message) -> np.ndarray:
    return np.array([message.x, message.y, message.z, message.w], dtype=np.float64)


class InspectionContextServer(Node):
    def __init__(self) -> None:
        super().__init__("inspection_context_server")
        self.declare_parameter("odometry_topic", "/wcr/odom")
        self.declare_parameter("imu_topic", "/wcr/imu")
        self.declare_parameter("rail_state_topic", "/wcr/probe_rail/state")
        self.declare_parameter("maximum_offset_s", 0.005)
        self.declare_parameter("buffer_age_s", 3.0)
        self.maximum_offset_s = float(self.get_parameter("maximum_offset_s").value)
        buffer_age_s = float(self.get_parameter("buffer_age_s").value)
        self.odometry_buffer = TimeBuffer[tuple[MotionSample, Odometry]](buffer_age_s)
        self.imu_buffer = TimeBuffer[tuple[ImuSample, Imu]](buffer_age_s)
        self.rail_buffer = TimeBuffer[tuple[RailSample, ProbeRailState]](buffer_age_s)

        self.create_subscription(
            Odometry,
            str(self.get_parameter("odometry_topic").value),
            self._odometry_callback,
            100,
        )
        self.create_subscription(
            Imu,
            str(self.get_parameter("imu_topic").value),
            self._imu_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            ProbeRailState,
            str(self.get_parameter("rail_state_topic").value),
            self._rail_callback,
            100,
        )
        self.create_service(
            GetInspectionContext,
            "inspection_context",
            self._service_callback,
        )

    def _odometry_callback(self, message: Odometry) -> None:
        pose = message.pose.pose
        twist = message.twist.twist
        sample = MotionSample(
            xyz(pose.position),
            quaternion(pose.orientation),
            xyz(twist.linear),
            xyz(twist.angular),
        )
        self._append(self.odometry_buffer, stamp_ns(message.header.stamp), (sample, message))

    def _imu_callback(self, message: Imu) -> None:
        sample = ImuSample(
            quaternion(message.orientation),
            xyz(message.angular_velocity),
            xyz(message.linear_acceleration),
        )
        self._append(self.imu_buffer, stamp_ns(message.header.stamp), (sample, message))

    def _rail_callback(self, message: ProbeRailState) -> None:
        sample = RailSample(
            message.position_m,
            message.velocity_m_s,
            message.target_position_m,
            message.enabled,
            message.homed,
            message.moving,
            message.negative_limit,
            message.positive_limit,
            message.fault,
            message.fault_code,
        )
        self._append(self.rail_buffer, stamp_ns(message.header.stamp), (sample, message))

    def _append(self, buffer, timestamp: int, value) -> None:
        try:
            buffer.append(timestamp, value)
        except ValueError as error:
            self.get_logger().warn(f"Dropped non-monotonic sensor sample: {error}")

    def _service_callback(self, request, response):
        target_ns = stamp_ns(request.stamp)
        context = InspectionContext()
        context.header.stamp = request.stamp
        context.header.frame_id = "odom"
        context.task_id = request.task_id
        context.sample_index = request.sample_index
        try:
            context.odometry, context.odometry_offset_s = self._odometry_at(target_ns)
            context.imu, context.imu_offset_s = self._imu_at(target_ns)
            context.rail_state, context.rail_offset_s = self._rail_at(target_ns)
        except (LookupError, ValueError) as error:
            response.success = False
            response.detail = str(error)
            context.timing_valid = False
            context.detail = str(error)
            response.context = context
            return response

        offsets = (
            context.odometry_offset_s,
            context.imu_offset_s,
            context.rail_offset_s,
        )
        context.timing_valid = max(offsets) <= self.maximum_offset_s
        context.detail = "ok" if context.timing_valid else "sensor_time_offset_exceeded"
        response.success = True
        response.detail = context.detail
        response.context = context
        return response

    def _odometry_at(self, target_ns: int):
        before, after, fraction = self.odometry_buffer.bracket(target_ns)
        sample = interpolate_motion(before.value[0], after.value[0], fraction)
        message = copy.deepcopy(before.value[1] if fraction < 0.5 else after.value[1])
        message.header.stamp.sec = target_ns // 1_000_000_000
        message.header.stamp.nanosec = target_ns % 1_000_000_000
        message.pose.pose.position.x, message.pose.pose.position.y, message.pose.pose.position.z = sample.position
        (
            message.pose.pose.orientation.x,
            message.pose.pose.orientation.y,
            message.pose.pose.orientation.z,
            message.pose.pose.orientation.w,
        ) = sample.orientation_xyzw
        message.twist.twist.linear.x, message.twist.twist.linear.y, message.twist.twist.linear.z = sample.linear_velocity
        message.twist.twist.angular.x, message.twist.twist.angular.y, message.twist.twist.angular.z = sample.angular_velocity
        return message, nearest_offset_s(target_ns, before.stamp_ns, after.stamp_ns)

    def _imu_at(self, target_ns: int):
        before, after, fraction = self.imu_buffer.bracket(target_ns)
        sample = interpolate_imu(before.value[0], after.value[0], fraction)
        message = copy.deepcopy(before.value[1] if fraction < 0.5 else after.value[1])
        message.header.stamp.sec = target_ns // 1_000_000_000
        message.header.stamp.nanosec = target_ns % 1_000_000_000
        (
            message.orientation.x,
            message.orientation.y,
            message.orientation.z,
            message.orientation.w,
        ) = sample.orientation_xyzw
        message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z = sample.angular_velocity
        message.linear_acceleration.x, message.linear_acceleration.y, message.linear_acceleration.z = sample.linear_acceleration
        return message, nearest_offset_s(target_ns, before.stamp_ns, after.stamp_ns)

    def _rail_at(self, target_ns: int):
        before, after, fraction = self.rail_buffer.bracket(target_ns)
        sample = interpolate_rail(before.value[0], after.value[0], fraction)
        message = copy.deepcopy(before.value[1] if fraction < 0.5 else after.value[1])
        message.header.stamp.sec = target_ns // 1_000_000_000
        message.header.stamp.nanosec = target_ns % 1_000_000_000
        for name in RailSample.__dataclass_fields__:
            setattr(message, name, getattr(sample, name))
        return message, nearest_offset_s(target_ns, before.stamp_ns, after.stamp_ns)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = InspectionContextServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
