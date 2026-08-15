import math

from geometry_msgs.msg import Twist
from wcr_fusion.fusion_core import bounded_twist, manual_command_twist
from wcr_planning_msgs.msg import ManualCommand


def command(kind, linear=0.08, angular=0.4):
    message = ManualCommand()
    message.command = kind
    message.linear_speed = linear
    message.angular_speed = angular
    return manual_command_twist(message, 0.08, 0.4, 0.15, 0.6)


def test_manual_directions():
    assert command(ManualCommand.FORWARD).linear.x > 0.0
    assert command(ManualCommand.BACKWARD).linear.x < 0.0
    assert command(ManualCommand.STRAFE_LEFT).linear.y > 0.0
    assert command(ManualCommand.STRAFE_RIGHT).linear.y < 0.0
    assert command(ManualCommand.TURN_LEFT).angular.z > 0.0
    assert command(ManualCommand.TURN_RIGHT).angular.z < 0.0
    assert command(ManualCommand.ROTATE_LEFT).linear.x == 0.0
    assert command(ManualCommand.ROTATE_RIGHT).angular.z < 0.0


def test_manual_speed_is_bounded():
    result = command(ManualCommand.TURN_LEFT, 2.0, 3.0)
    assert math.isclose(math.hypot(result.linear.x, result.linear.y), 0.15)
    assert math.isclose(result.angular.z, 0.6)


def test_twist_vector_limit_preserves_direction():
    source = Twist()
    source.linear.x = 3.0
    source.linear.y = 4.0
    result = bounded_twist(source, 0.1, 0.5)
    assert math.isclose(result.linear.x, 0.06)
    assert math.isclose(result.linear.y, 0.08)
