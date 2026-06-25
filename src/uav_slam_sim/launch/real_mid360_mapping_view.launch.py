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


def generate_launch_description():
    sim_share = get_package_share_directory("uav_slam_sim")

    real_launch = os.path.join(sim_share, "launch", "real_mid360_fastlio.launch.py")
    rviz_src = os.path.join(sim_share, "config", "real_mid360_mapping_light.rviz")
    runtime_dir = os.path.join(os.path.expanduser("~"), "rviz_runtime")
    os.makedirs(runtime_dir, exist_ok=True)
    rviz_cfg = os.path.join(runtime_dir, "real_mid360_mapping_light.rviz")
    shutil.copyfile(rviz_src, rviz_cfg)

    start_livox = LaunchConfiguration("start_livox")
    start_fast_lio = LaunchConfiguration("start_fast_lio")
    start_rviz = LaunchConfiguration("start_rviz")
    start_grid_map = LaunchConfiguration("start_grid_map")
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
            "start_rviz",
            default_value="true",
            description="Start the uav_slam_sim RViz view.",
        ),
        DeclareLaunchArgument(
            "start_grid_map",
            default_value="true",
            description="Start the realtime 2D occupancy grid projection node.",
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
        SetEnvironmentVariable("QT_OPENGL", "software"),
        SetEnvironmentVariable("LIBGL_ALWAYS_SOFTWARE", "1"),
        SetEnvironmentVariable("LIBGL_DRI3_DISABLE", "1"),
        SetEnvironmentVariable("QT_SCALE_FACTOR", "1"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(real_launch),
            launch_arguments={
                "start_livox": start_livox,
                "start_fast_lio": start_fast_lio,
                "rviz": "false",
                "start_aggregator": "false",
                "lidar_to_imu_x": lidar_to_imu_x,
                "lidar_to_imu_y": lidar_to_imu_y,
                "lidar_to_imu_z": lidar_to_imu_z,
                "lidar_to_imu_roll_deg": lidar_to_imu_roll_deg,
                "lidar_to_imu_pitch_deg": lidar_to_imu_pitch_deg,
                "lidar_to_imu_yaw_deg": lidar_to_imu_yaw_deg,
                "time_offset_lidar_to_imu": time_offset_lidar_to_imu,
                "time_sync_en": time_sync_en,
                "extrinsic_est_en": extrinsic_est_en,
            }.items(),
        ),
        Node(
            package="uav_slam_sim",
            executable="fastlio_cloud_mapper_node",
            name="fastlio_cloud_mapper",
            output="screen",
            parameters=[{
                "input_topic": "/cloud_registered",
                "scan_topic": "/cloud_registered_filtered",
                "reliable_scan_topic": "/cloud_registered_reliable",
                "map_topic": "/fastlio_denoised_map",
                "output_frame": "camera_init",
                "scan_voxel": 0.05,
                "map_voxel": 0.10,
                "radius_filter": 0.22,
                "radius_min_neighbors": 1,
                "min_map_hits": 1,
                "max_scan_points": 65000,
                "max_map_voxels": 220000,
                "map_window_frames": 300,
                "map_publish_every": 1,
                "enable_quality_gate": True,
                "require_odom_for_map": True,
                "max_odom_speed_mps": 2.5,
                "max_odom_jump_m": 0.55,
                "max_z_jump_m": 0.35,
                "min_scan_map_overlap": 0.035,
                "min_overlap_map_voxels": 800,
                "overlap_warmup_frames": 12,
                "overlap_neighbor_voxels": 2,
                "overlap_sample_stride": 4,
                "min_range": 0.45,
                "max_range": 35.0,
                "z_min": -20.0,
                "z_max": 10.0,
                "restamp": True,
            }],
            condition=IfCondition(start_fast_lio),
        ),
        Node(
            package="uav_slam_sim",
            executable="pointcloud_occupancy_grid_node",
            name="fastlio_occupancy_grid",
            output="screen",
            parameters=[{
                "input_topic": "/cloud_registered_reliable",
                "odom_topic": "/Odometry",
                "grid_topic": "/fastlio_occupancy_grid",
                "marker_topic": "/fastlio_occupancy_cells",
                "output_frame": "camera_init",
                "resolution": 0.10,
                "width_m": 30.0,
                "height_m": 30.0,
                "origin_x": -15.0,
                "origin_y": -15.0,
                "min_range": 0.45,
                "max_range": 25.0,
                "z_min": -0.30,
                "z_max": 2.50,
                "point_stride": 1,
                "publish_every_n_clouds": 1,
                "use_odometry": True,
                "raycast_free_space": True,
                "occupied_increment": 12,
                "free_decrement": 4,
                "occupied_threshold": 45,
                "free_threshold": -12,
                "log_odds_decay_per_cloud": 2,
                "stale_after_clouds": 60,
                "occupied_cell_inflation": 1,
            }],
            condition=IfCondition(start_grid_map),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="real_mid360_mapping_rviz",
            arguments=["-d", rviz_cfg],
            output="screen",
            condition=IfCondition(start_rviz),
        ),
        TimerAction(
            period=4.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        "bash", "-lc",
                        "timeout 3s xdotool search --name 'real_mid360_mapping_light.rviz' "
                        "windowsize %@ 1180 780 windowmove %@ 80 60 windowraise %@ || true"
                    ],
                    output="log",
                    condition=IfCondition(start_rviz),
                )
            ],
        ),
    ])

