from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def localizer(role: str, rate: float, config_argument: str, output_topic: str) -> Node:
    prefix = f"/{role}_camera"
    return Node(
        package="wcr_vision",
        executable="rgbd_weld_localizer",
        name=f"{role}_weld_localizer",
        namespace="wcr",
        output="screen",
        parameters=[{
            "camera_role": role,
            "model_path": LaunchConfiguration("model_path"),
            "camera_config_path": LaunchConfiguration(config_argument),
            "color_topic": f"{prefix}/color/image_raw",
            "depth_topic": f"{prefix}/aligned_depth_to_color/image_raw",
            "camera_info_topic": f"{prefix}/color/camera_info",
            "rail_state_topic": "/wcr/probe_rail/state",
            "candidate_topic": output_topic,
            "timing_topic": f"vision/{role}/timing",
            "state_topic": f"vision/{role}/state",
            "target_rate_hz": rate,
            "maximum_processing_s": 0.040 if role == "probe" else 0.070,
            "maximum_capture_age_s": 0.10 if role == "probe" else 0.15,
        }],
    )


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("model_path"),
        DeclareLaunchArgument("top_camera_config"),
        DeclareLaunchArgument("probe_camera_config"),
        localizer("probe", 30.0, "probe_camera_config", "weld_seam_candidates"),
        localizer("top", 12.0, "top_camera_config", "top_weld_seam_candidates"),
    ])
