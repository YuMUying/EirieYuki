from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from wcr_planning_msgs.msg import LinearMotorCommand, ProbeRailState


class MockLinearMotorDriver(Node):
    def __init__(self) -> None:
        super().__init__("mock_linear_motor_driver")
        self.declare_parameter("minimum_position_m", -0.10)
        self.declare_parameter("maximum_position_m", 0.10)
        self.declare_parameter("update_rate_hz", 100.0)
        self.minimum = float(self.get_parameter("minimum_position_m").value)
        self.maximum = float(self.get_parameter("maximum_position_m").value)
        rate = float(self.get_parameter("update_rate_hz").value)
        self.position = 0.0
        self.velocity = 0.0
        self.target = 0.0
        self.max_velocity = 0.03
        self.max_acceleration = 0.20
        self.enabled = True
        self.homed = True
        self.fault = False
        self.last_ns = self.get_clock().now().nanoseconds
        self.state_publisher = self.create_publisher(
            ProbeRailState, "probe_rail/state", 20
        )
        self.sim_position_publisher = self.create_publisher(
            Float64MultiArray, "probe_rail_position_controller/commands", 10
        )
        self.create_subscription(
            LinearMotorCommand, "linear_motor/command", self._command_callback, 10
        )
        self.create_timer(1.0 / rate, self._update)

    def _command_callback(self, message: LinearMotorCommand) -> None:
        if message.mode in (LinearMotorCommand.DISABLE, LinearMotorCommand.EMERGENCY_STOP):
            self.enabled = False
            self.velocity = 0.0
        elif message.mode == LinearMotorCommand.HOME:
            self.enabled = True
            self.homed = True
            self.target = 0.0
        elif message.mode == LinearMotorCommand.HOLD:
            self.enabled = True
            self.target = self.position
        elif message.mode == LinearMotorCommand.POSITION:
            self.enabled = True
            self.target = max(self.minimum, min(self.maximum, message.target_position_m))
            self.max_velocity = max(0.001, float(message.max_velocity_m_s))
            self.max_acceleration = max(
                0.001, float(message.max_acceleration_m_s2)
            )

    def _update(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        dt = max(0.0, min(0.1, (now_ns - self.last_ns) / 1e9))
        self.last_ns = now_ns
        error = self.target - self.position
        if self.enabled and abs(error) > 1e-6:
            desired_velocity = math.copysign(
                min(self.max_velocity, abs(error) / max(dt, 1e-6)), error
            )
            velocity_step = self.max_acceleration * dt
            self.velocity += max(
                -velocity_step,
                min(velocity_step, desired_velocity - self.velocity),
            )
            self.position += self.velocity * dt
            if (error > 0 and self.position > self.target) or (
                error < 0 and self.position < self.target
            ):
                self.position = self.target
                self.velocity = 0.0
        else:
            self.velocity = 0.0

        state = ProbeRailState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.header.frame_id = "probe_rail"
        state.position_m = self.position
        state.velocity_m_s = self.velocity
        state.target_position_m = self.target
        state.enabled = self.enabled
        state.homed = self.homed
        state.moving = abs(self.velocity) > 1e-6
        state.negative_limit = self.position <= self.minimum + 1e-6
        state.positive_limit = self.position >= self.maximum - 1e-6
        state.fault = self.fault
        state.fault_code = ""
        self.state_publisher.publish(state)

        sim_command = Float64MultiArray()
        sim_command.data = [self.position]
        self.sim_position_publisher.publish(sim_command)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MockLinearMotorDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
