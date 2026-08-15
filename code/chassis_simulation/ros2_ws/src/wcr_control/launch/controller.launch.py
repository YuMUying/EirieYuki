from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def spawner(namespace, manager, controller: str, condition=None) -> Node:
    return Node(
        package='controller_manager',
        executable='spawner',
        namespace=namespace,
        arguments=[controller, '--controller-manager', manager],
        condition=condition,
    )


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    controller_manager = LaunchConfiguration('controller_manager')
    probe_rail_sim_control = LaunchConfiguration('probe_rail_sim_control')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='wcr'),
        DeclareLaunchArgument(
            'controller_manager',
            default_value='/wcr/controller_manager',
        ),
        DeclareLaunchArgument('probe_rail_sim_control', default_value='false'),
        spawner(namespace, controller_manager, 'joint_state_broadcaster'),
        spawner(namespace, controller_manager, 'steering_position_controller'),
        spawner(namespace, controller_manager, 'driving_velocity_controller'),
        spawner(
            namespace,
            controller_manager,
            'probe_rail_position_controller',
            IfCondition(probe_rail_sim_control),
        ),
    ])
