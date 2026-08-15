import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class AutoDrive(Node):
    def __init__(self):
        super().__init__('auto_drive')

        self.declare_parameter('speed', 1.0)
        self.declare_parameter('start_time', 2.0)
        self.declare_parameter('stop_time', 10.0)

        self.speed = self.get_parameter('speed').value
        self.start_time = self.get_parameter('start_time').value
        self.stop_time = self.get_parameter('stop_time').value

        if self.start_time < 0.0:
            raise ValueError('start_time must be non-negative')
        if self.stop_time <= self.start_time:
            raise ValueError('stop_time must be greater than start_time')

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.publish_command)
        self.motion_state = None

    def publish_command(self):
        simulation_time = self.get_clock().now().nanoseconds / 1e9
        moving = self.start_time <= simulation_time < self.stop_time

        command = Twist()
        if moving:
            command.linear.x = float(self.speed)
        self.publisher.publish(command)

        if moving != self.motion_state:
            if moving:
                self.get_logger().info(
                    f'Starting automatic forward motion at {self.speed:.3f} m/s'
                )
            elif self.motion_state is True:
                self.get_logger().info('Automatic forward motion stopped')
            self.motion_state = moving


def main(args=None):
    rclpy.init(args=args)
    node = AutoDrive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop_command = Twist()
        for _ in range(3):
            node.publisher.publish(stop_command)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
