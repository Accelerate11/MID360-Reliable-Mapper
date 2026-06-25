import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    sim_share = get_package_share_directory("uav_slam_sim")
    real_launch = os.path.join(sim_share, "launch", "real_mid360_fastlio.launch.py")
    icp_viewer = os.path.join(sim_share, "scripts", "web_mid360_icp_mapping_viewer.py")

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(real_launch),
            launch_arguments={
                "start_fast_lio": "false",
                "rviz": "false",
            }.items(),
        ),
        ExecuteProcess(
            cmd=[
                "python3",
                icp_viewer,
                "--topic", "/livox/lidar_frame",
                "--port", "8765",
                "--period", "0.18",
                "--max-range", "25.0",
            ],
            output="screen",
        ),
    ])
