import os
import shutil

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def copy_runtime_config(package_share, filename):
    runtime_dir = os.path.join(os.path.expanduser("~"), "rviz_runtime")
    os.makedirs(runtime_dir, exist_ok=True)
    src = os.path.join(package_share, "config", filename)
    dst = os.path.join(runtime_dir, filename)
    shutil.copyfile(src, dst)
    os.chmod(dst, 0o644)
    return dst


def generate_launch_description():
    sim_share = get_package_share_directory("uav_slam_sim")
    core_launch = os.path.join(sim_share, "launch", "real_mid360_mapping_view.launch.py")

    pointcloud_rviz = copy_runtime_config(sim_share, "real_mid360_pointcloud_view.rviz")
    grid_rviz = copy_runtime_config(sim_share, "real_mid360_grid_view.rviz")

    start_livox = LaunchConfiguration("start_livox")
    start_fast_lio = LaunchConfiguration("start_fast_lio")
    start_dual_rviz = LaunchConfiguration("start_dual_rviz")
    lidar_to_imu_x = LaunchConfiguration("lidar_to_imu_x")
    lidar_to_imu_y = LaunchConfiguration("lidar_to_imu_y")
    lidar_to_imu_z = LaunchConfiguration("lidar_to_imu_z")
    lidar_to_imu_roll_deg = LaunchConfiguration("lidar_to_imu_roll_deg")
    lidar_to_imu_pitch_deg = LaunchConfiguration("lidar_to_imu_pitch_deg")
    lidar_to_imu_yaw_deg = LaunchConfiguration("lidar_to_imu_yaw_deg")
    time_offset_lidar_to_imu = LaunchConfiguration("time_offset_lidar_to_imu")
    time_sync_en = LaunchConfiguration("time_sync_en")
    extrinsic_est_en = LaunchConfiguration("extrinsic_est_en")

    return LaunchDescription([
        DeclareLaunchArgument(
            "start_livox",
            default_value="true",
            description="Start the Livox MID360 hardware driver.",
        ),
        DeclareLaunchArgument(
            "start_fast_lio",
            default_value="true",
            description="Start FAST-LIO2 mapping.",
        ),
        DeclareLaunchArgument(
            "start_dual_rviz",
            default_value="true",
            description="Start separate point-cloud and occupancy-grid RViz windows.",
        ),
        DeclareLaunchArgument(
            "lidar_to_imu_x", default_value="-0.011",
            description="MID360 LiDAR-to-IMU translation X in meters.",
        ),
        DeclareLaunchArgument(
            "lidar_to_imu_y", default_value="-0.02329",
            description="MID360 LiDAR-to-IMU translation Y in meters.",
        ),
        DeclareLaunchArgument(
            "lidar_to_imu_z", default_value="0.04412",
            description="MID360 LiDAR-to-IMU translation Z in meters.",
        ),
        DeclareLaunchArgument(
            "lidar_to_imu_roll_deg", default_value="0.0",
            description="Horizontal preset roll in degrees.",
        ),
        DeclareLaunchArgument(
            "lidar_to_imu_pitch_deg", default_value="0.0",
            description="Horizontal preset pitch in degrees.",
        ),
        DeclareLaunchArgument(
            "lidar_to_imu_yaw_deg", default_value="0.0",
            description="Horizontal preset yaw in degrees.",
        ),
        DeclareLaunchArgument(
            "time_offset_lidar_to_imu", default_value="0.0",
            description="LiDAR timestamp minus IMU timestamp, seconds.",
        ),
        DeclareLaunchArgument(
            "time_sync_en", default_value="false",
            description="FAST-LIO2 software time sync flag.",
        ),
        DeclareLaunchArgument(
            "extrinsic_est_en", default_value="false",
            description="Let FAST-LIO2 estimate LiDAR-IMU extrinsic online.",
        ),        SetEnvironmentVariable("DISPLAY", ":0"),
        SetEnvironmentVariable("WAYLAND_DISPLAY", ""),
        SetEnvironmentVariable("QT_QPA_PLATFORM", "xcb"),
        SetEnvironmentVariable("QT_X11_NO_MITSHM", "1"),
        SetEnvironmentVariable("QT_OPENGL", "desktop"),
        SetEnvironmentVariable("LIBGL_DRI3_DISABLE", "1"),
        SetEnvironmentVariable("MESA_GL_VERSION_OVERRIDE", "3.3"),
        SetEnvironmentVariable("MESA_GLSL_VERSION_OVERRIDE", "330"),
        SetEnvironmentVariable("QT_SCALE_FACTOR", "1"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(core_launch),
            launch_arguments={
                "start_livox": start_livox,
                "start_fast_lio": start_fast_lio,
                "start_grid_map": "true",
                "start_rviz": "false",
            }.items(),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="real_mid360_pointcloud_rviz",
            arguments=["-d", pointcloud_rviz],
            output="screen",
            condition=IfCondition(start_dual_rviz),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="real_mid360_grid_rviz",
            arguments=["-d", grid_rviz],
            output="screen",
            condition=IfCondition(start_dual_rviz),
        ),
        TimerAction(
            period=4.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        "bash", "-lc",
                        "timeout 4s xdotool search --name 'real_mid360_pointcloud_view.rviz' "
                        "windowsize %@ 980 760 windowmove %@ 40 60 windowraise %@ || true; "
                        "timeout 4s xdotool search --name 'real_mid360_grid_view.rviz' "
                        "windowsize %@ 820 760 windowmove %@ 1040 60 windowraise %@ || true",
                    ],
                    output="log",
                    condition=IfCondition(start_dual_rviz),
                )
            ],
        ),
    ])

