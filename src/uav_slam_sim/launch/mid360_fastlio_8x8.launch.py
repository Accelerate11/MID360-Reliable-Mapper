from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    sim_share = get_package_share_directory("uav_slam_sim")
    fast_lio_share = get_package_share_directory("fast_lio")

    world = os.path.join(sim_share, "worlds", "mid360_slam_8x8.sdf")
    fast_lio_config = os.path.join(sim_share, "config", "fast_lio_sim_mid360.yaml")
    fast_lio_launch = os.path.join(fast_lio_share, "launch", "mapping.launch.py")
    model_path = os.path.join(sim_share, "models")

    use_bridge = LaunchConfiguration("use_bridge")
    start_fast_lio = LaunchConfiguration("start_fast_lio")
    use_rviz = LaunchConfiguration("rviz")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_bridge",
            default_value="true",
            description="Start the local Gazebo-to-ROS MID360 bridge.",
        ),
        DeclareLaunchArgument(
            "start_fast_lio",
            default_value="true",
            description="Start FAST-LIO2 with the simulated MID360 configuration.",
        ),
        DeclareLaunchArgument(
            "rviz",
            default_value="false",
            description="Start RViz from FAST-LIO2 launch.",
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
            condition=IfCondition(use_bridge),
            output="screen",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(fast_lio_launch),
            launch_arguments={
                "use_sim_time": "true",
                "config_path": os.path.dirname(fast_lio_config),
                "config_file": os.path.basename(fast_lio_config),
                "rviz": use_rviz,
            }.items(),
            condition=IfCondition(start_fast_lio),
        ),
    ])
