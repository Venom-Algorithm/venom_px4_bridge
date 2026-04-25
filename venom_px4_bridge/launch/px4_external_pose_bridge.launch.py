import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bridge_dir = get_package_share_directory("venom_px4_bridge")
    default_config = os.path.join(bridge_dir, "config", "px4_external_pose_bridge.yaml")

    config_file = LaunchConfiguration("config_file")
    fmu_prefix = LaunchConfiguration("fmu_prefix")
    input_odom_topic = LaunchConfiguration("input_odom_topic")

    declare_args = [
        DeclareLaunchArgument(
            "config_file",
            default_value=default_config,
            description="Parameter file for px4_external_pose_bridge.",
        ),
        DeclareLaunchArgument(
            "fmu_prefix",
            default_value="/fmu",
            description="PX4 DDS topic namespace prefix.",
        ),
        DeclareLaunchArgument(
            "input_odom_topic",
            default_value="/lio/vps/odometry",
            description="Odometry topic published by the Lio/VPS algorithm.",
        ),
    ]

    pose_bridge = Node(
        package="venom_px4_bridge",
        executable="px4_external_pose_bridge",
        name="px4_external_pose_bridge",
        output="screen",
        parameters=[
            config_file,
            {
                "fmu_prefix": fmu_prefix,
                "input_odom_topic": input_odom_topic,
            },
        ],
    )

    return LaunchDescription(declare_args + [pose_bridge])
