import os

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    urdf_file_arg = DeclareLaunchArgument(
        'urdf_file',
        default_value='wcr.urdf.xacro',
        description='URDF/Xacro file from wcr_description/urdf',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock',
    )
    variant_arg = DeclareLaunchArgument(
        'variant',
        default_value='mock',
        description='Robot variant: classic/effort/velocity/mimic/mix/mock',
    )
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='wcr',
        description='Namespace passed into xacro',
    )

    urdf_file = LaunchConfiguration('urdf_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    variant = LaunchConfiguration('variant')
    namespace = LaunchConfiguration('namespace')

    description_share = get_package_share_path('wcr_description')
    urdf_path = PathJoinSubstitution([description_share, 'urdf', urdf_file])
    rviz_config_path = os.path.join(description_share, 'rviz', 'wcr_sensors.rviz')

    robot_description = ParameterValue(
        Command([
            'xacro ',
            urdf_path,
            ' variant:=', variant,
            ' sim:=true',
            ' propeller_control:=false',
            ' namespace:=', namespace,
        ]),
        value_type=str,
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
            }
        ],
        output='screen',
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    return LaunchDescription([
        urdf_file_arg,
        use_sim_time_arg,
        variant_arg,
        namespace_arg,
        robot_state_publisher_node,
        rviz2_node,
    ])
