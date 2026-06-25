from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import EnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    pkg_share = get_package_share_directory("uav_slam_sim")
    world = os.path.join(pkg_share, "worlds", "mid360_slam_8x8.sdf")
    model_path = os.path.join(pkg_share, "models")
    rviz_cfg = os.path.join(pkg_share, "config", "mid360_points.rviz")

    return LaunchDescription([
        SetEnvironmentVariable(name="LIBGL_ALWAYS_SOFTWARE", value="1"),
        SetEnvironmentVariable(name="QT_X11_NO_MITSHM", value="1"),
        SetEnvironmentVariable(name="FASTDDS_BUILTIN_TRANSPORTS", value="UDPv4"),
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
            cmd=["gz", "sim", "-s", "-r", "-v2", world],
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
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", rviz_cfg],
            output="screen",
        ),
    ])
