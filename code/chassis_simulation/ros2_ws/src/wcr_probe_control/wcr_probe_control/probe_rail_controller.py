from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from wcr_planning_msgs.msg import (
    LinearMotorCommand,
    ProbeAlignment,
    ProbeRailCommand,
    ProbeRailState,
)

from .rail_control import RailLimits, motion_allowed, target_from_alignment


class ProbeRailController(Node):
    def __init__(self) -> None:
        super().__init__("probe_rail_controller")
        self.declare_parameter("minimum_position_m", -0.10)
        self.declare_parameter("maximum_position_m", 0.10)
        self.declare_parameter("maximum_velocity_m_s", 0.03)
        self.declare_parameter("maximum_acceleration_m_s2", 0.20)
        self.declare_parameter("maximum_alignment_age_s", 0.15)
        self.declare_parameter("minimum_confidence", 0.65)
        self.declare_parameter("alignment_deadband_m", 0.0005)
        self.declare_parameter("maximum_correction_step_m", 0.010)
        self.declare_parameter("alignment_watchdog_s", 0.25)

        value = lambda name: float(self.get_parameter(name).value)
        self.limits = RailLimits(
            value("minimum_position_m"),
            value("maximum_position_m"),
            value("maximum_velocity_m_s"),
            value("maximum_acceleration_m_s2"),
            value("maximum_alignment_age_s"),
            value("minimum_confidence"),
            value("alignment_deadband_m"),
            value("maximum_correction_step_m"),
        )
        self.alignment_watchdog_s = value("alignment_watchdog_s")
        self.state: ProbeRailState | None = None
        self.tracking_enabled = False
        self.last_accepted_alignment_ns: int | None = None
        self.watchdog_holding = False

        self.motor_publisher = self.create_publisher(
            LinearMotorCommand, "linear_motor/command", 10
        )
        self.create_subscription(
            ProbeRailState, "probe_rail/state", self._state_callback, 20
        )
        self.create_subscription(
            ProbeRailCommand, "probe_rail/command", self._command_callback, 10
        )
        self.create_subscription(
            ProbeAlignment, "probe_alignment", self._alignment_callback, 10
        )
        self.create_timer(0.05, self._watchdog)

    def _state_callback(self, message: ProbeRailState) -> None:
        self.state = message
        if message.fault:
            self.tracking_enabled = False
            self._publish_motor(LinearMotorCommand.EMERGENCY_STOP, message.position_m)

    def _command_callback(self, message: ProbeRailCommand) -> None:
        if message.mode == ProbeRailCommand.TRACK_SEAM:
            self.tracking_enabled = True
            self.last_accepted_alignment_ns = self.get_clock().now().nanoseconds
            self.watchdog_holding = False
            self.get_logger().info("Probe rail seam tracking enabled")
            return
        self.tracking_enabled = False
        if message.mode == ProbeRailCommand.DISABLE:
            self._publish_motor(LinearMotorCommand.DISABLE, 0.0)
        elif message.mode == ProbeRailCommand.HOME:
            self._publish_motor(LinearMotorCommand.HOME, 0.0)
        elif message.mode == ProbeRailCommand.STOP:
            target = self.state.position_m if self.state else 0.0
            self._publish_motor(LinearMotorCommand.EMERGENCY_STOP, target)
        elif message.mode == ProbeRailCommand.HOLD:
            target = self.state.position_m if self.state else 0.0
            self._publish_motor(LinearMotorCommand.HOLD, target)
        elif message.mode == ProbeRailCommand.POSITION:
            self._publish_position_if_safe(message.target_position_m)

    def _alignment_callback(self, message: ProbeAlignment) -> None:
        if not self.tracking_enabled or self.state is None:
            return
        stamp_ns = int(message.header.stamp.sec) * 1_000_000_000 + int(
            message.header.stamp.nanosec
        )
        age_s = (self.get_clock().now().nanoseconds - stamp_ns) / 1e9
        decision = target_from_alignment(
            message.lateral_error_m,
            message.rail_position_at_capture_m,
            message.confidence,
            age_s,
            message.valid,
            self.limits,
        )
        if not decision.accepted:
            self.get_logger().warn(
                f"Rejected probe alignment: {decision.reason}",
                throttle_duration_sec=1.0,
            )
            return
        if self._publish_position_if_safe(decision.target_position_m):
            self.last_accepted_alignment_ns = self.get_clock().now().nanoseconds
            self.watchdog_holding = False

    def _publish_position_if_safe(self, target_position_m: float) -> bool:
        if self.state is None or not math.isfinite(target_position_m):
            return False
        target = self.limits.clamp_position(target_position_m)
        allowed, reason = motion_allowed(
            self.state.homed,
            self.state.fault,
            self.state.negative_limit,
            self.state.positive_limit,
            self.state.position_m,
            target,
        )
        if not allowed:
            self.get_logger().error(f"Blocked probe rail motion: {reason}")
            self._publish_motor(LinearMotorCommand.HOLD, self.state.position_m)
            return False
        self._publish_motor(LinearMotorCommand.POSITION, target)
        return True

    def _watchdog(self) -> None:
        if not self.tracking_enabled or self.state is None:
            return
        if self.last_accepted_alignment_ns is None:
            return
        age_s = (
            self.get_clock().now().nanoseconds - self.last_accepted_alignment_ns
        ) / 1e9
        if age_s > self.alignment_watchdog_s and not self.watchdog_holding:
            self._publish_motor(LinearMotorCommand.HOLD, self.state.position_m)
            self.watchdog_holding = True
            self.get_logger().error("Probe alignment watchdog expired; rail held")

    def _publish_motor(self, mode: int, target_position_m: float) -> None:
        command = LinearMotorCommand()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = "probe_rail"
        command.mode = mode
        command.target_position_m = float(target_position_m)
        command.max_velocity_m_s = self.limits.maximum_velocity_m_s
        command.max_acceleration_m_s2 = self.limits.maximum_acceleration_m_s2
        self.motor_publisher.publish(command)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ProbeRailController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
