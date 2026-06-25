import os
import shutil

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    sim_share = get_package_share_directory("uav_slam_sim")
    fast_lio_share = get_package_share_directory("fast_lio")

    fast_lio_config = os.path.join(sim_share, "config", "fast_lio_synthetic_mid360.yaml")
    fast_lio_launch = os.path.join(fast_lio_share, "launch", "mapping.launch.py")
    synthetic_script = os.path.join(sim_share, "scripts", "synthetic_mid360_publisher.py")

    rviz_src = os.path.join(sim_share, "config", "real_mid360_mapping_light.rviz")
    runtime_dir = os.path.join(os.path.expanduser("~"), "rviz_runtime")
    os.makedirs(runtime_dir, exist_ok=True)
    rviz_cfg = os.path.join(runtime_dir, "synthetic_mid360_fastlio.rviz")
    shutil.copyfile(rviz_src, rviz_cfg)

    return LaunchDescription([
        SetEnvironmentVariable("DISPLAY", ":0"),
        SetEnvironmentVariable("WAYLAND_DISPLAY", ""),
        SetEnvironmentVariable("QT_QPA_PLATFORM", "xcb"),
        SetEnvironmentVariable("QT_X11_NO_MITSHM", "1"),
        SetEnvironmentVariable("QT_OPENGL", "software"),
        SetEnvironmentVariable("LIBGL_ALWAYS_SOFTWARE", "1"),
        SetEnvironmentVariable("QT_SCALE_FACTOR", "1"),
        ExecuteProcess(
            cmd=[
                "python3",
                synthetic_script,
                "--lidar-topic", "/synthetic/livox/lidar",
                "--imu-topic", "/synthetic/livox/imu",
            ],
            output="screen",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(fast_lio_launch),
            launch_arguments={
                "use_sim_time": "false",
                "config_path": os.path.dirname(fast_lio_config),
                "config_file": os.path.basename(fast_lio_config),
                "rviz": "false",
            }.items(),
        ),
        Node(
            package="uav_slam_sim",
            executable="fastlio_cloud_mapper_node",
            name="synthetic_fastlio_cloud_mapper",
            output="screen",
            parameters=[{
                "input_topic": "/cloud_registered",
                "scan_topic": "/cloud_registered_filtered",
                "map_topic": "/fastlio_denoised_map",
                "output_frame": "camera_init",
                "scan_voxel": 0.05,
                "map_voxel": 0.08,
                "radius_filter": 0.22,
                "radius_min_neighbors": 1,
                "min_map_hits": 1,
                "max_scan_points": 60000,
                "max_map_voxels": 180000,
                "map_publish_every": 1,
                "min_range": 0.45,
                "max_range": 25.0,
                "z_min": -4.0,
                "z_max": 6.0,
                "restamp": True,
            }],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="synthetic_mid360_fastlio_rviz",
            arguments=["-d", rviz_cfg],
            output="screen",
        ),
        TimerAction(
            period=4.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        "bash", "-lc",
                        "wmctrl -r 'synthetic_mid360_fastlio.rviz' -b add,above || true; "
                        "wmctrl -r 'synthetic_mid360_fastlio.rviz' -e 0,80,60,1100,760 || true; "
                        "xdotool search --name 'synthetic_mid360_fastlio.rviz' windowactivate %@ || true"
                    ],
                    output="log",
                )
            ],
        ),
    ])
