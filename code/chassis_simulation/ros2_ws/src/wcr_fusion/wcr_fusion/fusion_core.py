import math

from geometry_msgs.msg import Twist
from wcr_planning_msgs.msg import ManualCommand


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def bounded_twist(command, max_linear_speed, max_angular_speed):
    result = Twist()
    requested = math.hypot(command.linear.x, command.linear.y)
    scale = 1.0
    if requested > max_linear_speed and requested > 0.0:
        scale = max_linear_speed / requested
    result.linear.x = command.linear.x * scale
    result.linear.y = command.linear.y * scale
    result.angular.z = clamp(
        command.angular.z, -max_angular_speed, max_angular_speed
    )
    return result


def manual_command_twist(
    message, default_linear_speed, default_angular_speed,
    max_linear_speed, max_angular_speed
):
    linear = abs(message.linear_speed) or default_linear_speed
    angular = abs(message.angular_speed) or default_angular_speed
    result = Twist()
    if message.command == ManualCommand.FORWARD:
        result.linear.x = linear
    elif message.command == ManualCommand.BACKWARD:
        result.linear.x = -linear
    elif message.command == ManualCommand.STRAFE_LEFT:
        result.linear.y = linear
    elif message.command == ManualCommand.STRAFE_RIGHT:
        result.linear.y = -linear
    elif message.command == ManualCommand.TURN_LEFT:
        result.linear.x = linear
        result.angular.z = angular
    elif message.command == ManualCommand.TURN_RIGHT:
        result.linear.x = linear
        result.angular.z = -angular
    elif message.command == ManualCommand.ROTATE_LEFT:
        result.angular.z = angular
    elif message.command == ManualCommand.ROTATE_RIGHT:
        result.angular.z = -angular
    return bounded_twist(result, max_linear_speed, max_angular_speed)
