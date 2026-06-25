from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    pkg_share = get_package_share_directory("uav_slam_sim")
    world = os.path.join(pkg_share, "worlds", "mid360_slam_8x8.sdf")
    model_path = os.path.join(pkg_share, "models")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_bridge",
            default_value="false",
            description="Start the local Gazebo-to-ROS MID360 bridge.",
        ),
        SetEnvironmentVariable(
            name="GZ_SIM_RESOURCE_PATH",
            value=[
                model_path,
                ":/home/accelerate/ardupilot_gazebo/models",
                ":/home/accelerate/ardupilot_gazebo/worlds:",
                EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value=""),
            ],
        ),
        SetEnvironmentVariable(
            name="GZ_SIM_SYSTEM_PLUGIN_PATH",
            value=[
                "/home/accelerate/ardupilot_gazebo/build:",
                EnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", default_value=""),
            ],
        ),
        ExecuteProcess(
            cmd=["gz", "sim", "-v4", "-r", world],
            output="screen",
        ),
        Node(
            package="uav_slam_sim",
            executable="gz_mid360_bridge_node",
            name="gz_mid360_bridge_node",
            parameters=[{
                "mid360_gz_topic": "/mid360/points/points",
                "mid360_ros_topic": "/mid360/points",
                "imu_gz_topic": "/mid360/imu",
                "imu_ros_topic": "/mid360/imu",
                "clock_gz_topic": "/clock",
                "lidar_frame_id": "mid360_link",
                "imu_frame_id": "imu_link",
            }],
            condition=IfCondition(LaunchConfiguration("use_bridge")),
            output="screen",
        ),
    ])
