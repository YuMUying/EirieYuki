import os
from ament_index_python.packages import get_package_share_path, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from wcr_launcher.yaml_loader import LauncherConfigurator

def generate_launch_description():

    # Paths and configuration
    pkg_share = get_package_share_path('wcr_description')
    controller_share = get_package_share_path('wcr_control')
    launcher_share = get_package_share_path('wcr_launcher')
    ins_ekf_config_path = os.path.join(launcher_share, 'config', 'ins_ekf.yaml')
    config = LauncherConfigurator()
    rviz_config_path = os.path.join(pkg_share, 'rviz', config.rviz_config)
    position_target_topic = (
        f'/{config.namespace}/controller_target_pose'
        if config.trajectory_planning
        else f'/{config.namespace}/target_pose'
    )
    position_reached_topic = (
        f'/{config.namespace}/controller_target_reached'
        if config.trajectory_planning
        else f'/{config.namespace}/target_reached'
    )
    probe_rail_sim_control = (
        config.sim
        and config.variant == 'mock'
        and config.get_launch_param('probe_rail_mock_driver', True)
    )
    rviz_enabled = LaunchConfiguration('rviz')
    gz_args = LaunchConfiguration('gz_args')
    rviz_argument = DeclareLaunchArgument(
        'rviz',
        default_value=str(config.rviz).lower(),
        description='Start RViz.',
    )
    gz_args_argument = DeclareLaunchArgument(
        'gz_args',
        default_value=config.gz_args,
        description='Arguments forwarded to gz sim.',
    )

    # Robot State Publisher (CENTRAL)
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=config.namespace,
        output="both",
        parameters=[{
            'robot_description': config.build_robot_description(),
            'use_sim_time': config.use_sim_time,
            "namespace" : config.namespace,
        }]
    )

    # Include Gazebo launch (conditional)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'gz_args': gz_args}.items(),
        condition=IfCondition(str(config.sim).lower())
    )

    # Include controllers launch (conditional)
    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(controller_share, 'launch', 'controller.launch.py')
        ),
        launch_arguments={
            'namespace': config.namespace,
            'controller_manager': f'/{config.namespace}/controller_manager',
            'probe_rail_sim_control': str(probe_rail_sim_control).lower(),
        }.items(),
    )

    controller_launch = Node(
        package="wcr_controllers",
        executable="inv_kin_controller",
        name="inv_kin_controller",
        namespace=config.namespace,
        parameters=[{
            'use_sim_time': config.use_sim_time,
            'max_wheel_angular_speed': config.get_robot_param('wheel_velocity', 5.23598775),
        }],
    )

    position_controller = Node(
        package="wcr_controllers",
        executable="position_controller",
        name="position_controller",
        namespace=config.namespace,
        output="screen",
        condition=IfCondition(str(config.position_control).lower()),
        parameters=[{
            'use_sim_time': config.use_sim_time,
            'target_pose_topic': position_target_topic,
            'trajectory_topic': f'/{config.namespace}/planned_trajectory',
            'odom_topic': f'/{config.namespace}/odom',
            'cmd_vel_topic': '/cmd_vel',
            'cancel_topic': f'/{config.namespace}/controller_cancel',
            'reached_topic': position_reached_topic,
            'max_acceleration': config.get_launch_param('planner_max_acceleration', 0.12),
            'max_deceleration': config.get_launch_param('planner_max_deceleration', 0.16),
            'lookahead_distance': config.get_launch_param(
                'controller_lookahead_distance', 0.045
            ),
            'lookahead_time': config.get_launch_param('controller_lookahead_time', 0.30),
            'max_position_correction_speed': config.get_launch_param(
                'controller_max_position_correction_speed', 0.05
            ),
            'linear_gain': config.get_launch_param('controller_path_tracking_gain', 1.8),
        }],
    )

    trajectory_planner = Node(
        package="wcr_planner",
        executable="online_trajectory_planner",
        name="online_trajectory_planner",
        namespace=config.namespace,
        output="screen",
        condition=IfCondition(str(config.trajectory_planning).lower()),
        parameters=[{
            'use_sim_time': config.use_sim_time,
            'mode_topic': f'/{config.namespace}/planning_mode',
            'obstacles_topic': f'/{config.namespace}/mapped_obstacles',
            'obstacle_update_topic': f'/{config.namespace}/mapped_obstacle_update',
            'goal_topic': f'/{config.namespace}/target_pose',
            'reference_curve_topic': f'/{config.namespace}/reference_curve',
            'odom_topic': f'/{config.namespace}/odom',
            'controller_target_topic': f'/{config.namespace}/controller_target_pose',
            'controller_cancel_topic': f'/{config.namespace}/controller_cancel',
            'controller_reached_topic': f'/{config.namespace}/controller_target_reached',
            'planned_path_topic': f'/{config.namespace}/planned_path',
            'trajectory_topic': f'/{config.namespace}/planned_trajectory',
            'status_topic': f'/{config.namespace}/planner_status',
            'reached_output_topic': f'/{config.namespace}/target_reached',
            'default_mode': config.get_launch_param('default_planning_mode', 1),
            'grid_resolution': config.get_launch_param('planner_grid_resolution', 0.03),
            'planning_padding': config.get_launch_param('planner_padding', 0.75),
            'robot_radius': config.get_launch_param('planner_robot_radius', 0.18),
            'safety_margin': config.get_launch_param('planner_safety_margin', 0.02),
            'tracking_error_allowance': config.get_launch_param(
                'planner_tracking_error_allowance', 0.05
            ),
            'waypoint_tolerance': config.get_launch_param('planner_waypoint_tolerance', 0.025),
            'trajectory_sample_distance': config.get_launch_param(
                'planner_trajectory_sample_distance', 0.015
            ),
            'curve_smoothing_iterations': config.get_launch_param(
                'planner_curve_smoothing_iterations', 3
            ),
            'curve_smoothing_weight': config.get_launch_param(
                'planner_curve_smoothing_weight', 0.20
            ),
            'max_trajectory_speed': config.get_launch_param(
                'planner_max_trajectory_speed', 0.13
            ),
            'max_lateral_acceleration': config.get_launch_param(
                'planner_max_lateral_acceleration', 0.18
            ),
            'max_acceleration': config.get_launch_param('planner_max_acceleration', 0.12),
            'max_deceleration': config.get_launch_param('planner_max_deceleration', 0.16),
            'minimum_cruise_speed': config.get_launch_param(
                'planner_minimum_cruise_speed', 0.035
            ),
        }],
    )

    # ROSBridge server (for Visualization)
    auto_drive = Node(
        package="wcr_launcher",
        executable="auto_drive",
        name="auto_drive",
        namespace=config.namespace,
        output="screen",
        condition=IfCondition(str(config.auto_drive).lower()),
        parameters=[{
            'use_sim_time': config.use_sim_time,
            'speed': config.auto_drive_speed,
            'start_time': config.auto_drive_start_time,
            'stop_time': config.auto_drive_stop_time,
        }],
    )

    rosbridge_server = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rosbridge_server'), 'launch', 'rosbridge_websocket_launch.xml')
        )
    )

    # Odom node
    odometry = Node(
        package="wcr_odometry",
        executable="odometry",
        name="odometry",
        namespace=config.namespace,
        parameters=[{
            'use_sim_time': config.use_sim_time
        }],
    )

    probe_rail_controller = Node(
        package="wcr_probe_control",
        executable="probe_rail_controller",
        name="probe_rail_controller",
        namespace=config.namespace,
        output="screen",
        condition=IfCondition(
            str(config.get_launch_param('probe_rail_control', True)).lower()
        ),
        parameters=[{
            'use_sim_time': config.use_sim_time,
            'minimum_position_m': config.get_launch_param(
                'probe_rail_minimum_position_m', -0.10
            ),
            'maximum_position_m': config.get_launch_param(
                'probe_rail_maximum_position_m', 0.10
            ),
            'maximum_velocity_m_s': config.get_launch_param(
                'probe_rail_maximum_velocity_m_s', 0.03
            ),
            'maximum_acceleration_m_s2': config.get_launch_param(
                'probe_rail_maximum_acceleration_m_s2', 0.20
            ),
            'maximum_alignment_age_s': config.get_launch_param(
                'probe_alignment_maximum_age_s', 0.15
            ),
            'minimum_confidence': config.get_launch_param(
                'probe_alignment_minimum_confidence', 0.65
            ),
            'alignment_deadband_m': config.get_launch_param(
                'probe_alignment_deadband_m', 0.0005
            ),
            'maximum_correction_step_m': config.get_launch_param(
                'probe_alignment_maximum_correction_step_m', 0.010
            ),
            'alignment_watchdog_s': config.get_launch_param(
                'probe_alignment_watchdog_s', 0.25
            ),
        }],
    )

    seam_tracking_manager = Node(
        package="wcr_probe_control",
        executable="seam_tracking_manager",
        name="seam_tracking_manager",
        namespace=config.namespace,
        output="screen",
        condition=IfCondition(
            str(config.get_launch_param('seam_tracking_and_map', True)).lower()
        ),
        parameters=[{
            'use_sim_time': config.use_sim_time,
            'odometry_topic': config.get_launch_param(
                'seam_tracking_odometry_topic', f'/{config.namespace}/ins/odometry'
            ),
            'map_path': config.get_launch_param(
                'environment_map_path', '~/.ros/wcr_environment_map.json'
            ),
            'maximum_odometry_offset_s': config.get_launch_param(
                'seam_tracking_maximum_odometry_offset_s', 0.10
            ),
            'minimum_candidate_points': config.get_launch_param(
                'seam_tracking_minimum_candidate_points', 5
            ),
            'minimum_candidate_confidence': config.get_launch_param(
                'probe_alignment_minimum_confidence', 0.65
            ),
            'minimum_seam_separation_m': config.get_launch_param(
                'seam_tracking_minimum_separation_m', 0.015
            ),
            'minimum_motion_speed_m_s': config.get_launch_param(
                'seam_tracking_minimum_motion_speed_m_s', 0.01
            ),
            'selection_lookahead_m': config.get_launch_param(
                'seam_tracking_lookahead_m', 0.08
            ),
            'maximum_lateral_error_m': config.get_launch_param(
                'seam_tracking_maximum_lateral_error_m', 0.10
            ),
            'probe_center_y_at_reference_m': config.get_launch_param(
                'probe_center_y_at_reference_m', 0.0
            ),
            'switch_score_margin': config.get_launch_param(
                'seam_tracking_switch_score_margin', 0.12
            ),
            'map_association_distance_m': config.get_launch_param(
                'seam_map_association_distance_m', 0.04
            ),
            'map_association_angle_deg': config.get_launch_param(
                'seam_map_association_angle_deg', 30.0
            ),
        }],
    )

    mock_linear_motor_driver = Node(
        package="wcr_probe_control",
        executable="mock_linear_motor_driver",
        name="mock_linear_motor_driver",
        namespace=config.namespace,
        condition=IfCondition(
            str(probe_rail_sim_control).lower()
        ),
        parameters=[{
            'use_sim_time': config.use_sim_time,
            'minimum_position_m': config.get_launch_param(
                'probe_rail_minimum_position_m', -0.10
            ),
            'maximum_position_m': config.get_launch_param(
                'probe_rail_maximum_position_m', 0.10
            ),
        }],
    )

    ins_fusion = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        namespace=config.namespace,
        output="screen",
        condition=IfCondition(
            str(config.get_launch_param('ins_fusion', True)).lower()
        ),
        parameters=[ins_ekf_config_path, {'use_sim_time': config.use_sim_time}],
        remappings=[('odometry/filtered', 'ins/odometry')],
    )

    inspection_context_server = Node(
        package="wcr_sensor_sync",
        executable="inspection_context_server",
        name="inspection_context_server",
        namespace=config.namespace,
        output="screen",
        condition=IfCondition(
            str(config.get_launch_param('sensor_time_sync', True)).lower()
        ),
        parameters=[{
            'use_sim_time': config.use_sim_time,
            'odometry_topic': config.get_launch_param(
                'inspection_odometry_topic', f'/{config.namespace}/ins/odometry'
            ),
            'imu_topic': f'/{config.namespace}/imu',
            'rail_state_topic': f'/{config.namespace}/probe_rail/state',
            'maximum_offset_s': config.get_launch_param(
                'inspection_context_maximum_offset_s', 0.005
            ),
        }],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        namespace=config.namespace,
        output="screen",
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': config.use_sim_time}],
        condition=IfCondition(rviz_enabled)
    )

    return LaunchDescription([
        rviz_argument,
        gz_args_argument,
        robot_state_publisher,
        gazebo_launch,
        control_launch,
        controller_launch,
        position_controller,
        trajectory_planner,
        auto_drive,
        rosbridge_server,
        odometry,
        seam_tracking_manager,
        probe_rail_controller,
        mock_linear_motor_driver,
        ins_fusion,
        inspection_context_server,
        rviz_node
    ])
