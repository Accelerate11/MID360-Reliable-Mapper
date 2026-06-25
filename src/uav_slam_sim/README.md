# MID360 实时可靠建图归档说明

本文档归档当前已经打通的 MID360 实雷达建图流程，目标是服务后续无人机或移动机器人实时建图、避障、规划前端开发。当前版本重点不是做最终导航控制，而是把前期地图输入做稳：实时点云、可靠点云筛选、二维占据栅格、RViz 双窗口可视化、边缘算力部署入口。

当前工程路径：

```bash
/home/accelerate/cuadc_ws
```

核心 ROS 2 包：

```bash
/home/accelerate/cuadc_ws/src/uav_slam_sim
/home/accelerate/cuadc_ws/src/FAST_LIO_ROS2
/home/accelerate/cuadc_ws/src/livox_ros_driver2_real
```

## 1. 当前归档目标

这套流程解决四件事：

1. 接入 Livox MID360 实物雷达，读取 `/livox/lidar` 和 `/livox/imu`。
2. 用 FAST-LIO2 做 LiDAR-Inertial Odometry，输出实时位姿和注册点云。
3. 在 FAST-LIO2 输出之后加入可靠地图层，过滤 IMU 飘移、位姿跳变、重叠率不足导致的坏帧，只把可信点云写入稳定地图。
4. 同时显示三维点云图和二维 SLAM 栅格图，为后续自主避障和路径规划准备可用地图输入。

当前版本已经避免把错误帧直接污染全局地图。也就是说，IMU 短时漂掉、位姿突然跳变时，系统会拒绝该帧写入稳定地图，而不是让整张图被拉飞。

## 2. 总体数据流

```text
MID360 硬件
  -> livox_ros_driver2
  -> /livox/lidar, /livox/imu
  -> FAST-LIO2
  -> /cloud_registered, /cloud_registered_body, /Odometry, /path
  -> FAST-LIO 可靠建图节点
  -> /cloud_registered_reliable, /fastlio_denoised_map
  -> 占据栅格节点
  -> /fastlio_occupancy_grid
  -> /fastlio_occupancy_free_cells
  -> /fastlio_occupancy_occupied_cells
  -> RViz 点云窗口 + RViz 栅格窗口
```

可以把系统理解成三层：

- 驱动层：把 MID360 的原始 LiDAR 和 IMU 数据接进 ROS 2。
- 里程计层：FAST-LIO2 根据 LiDAR 和 IMU 估计传感器运动，并把每帧点云注册到局部地图坐标系。
- 可靠建图层：判断 FAST-LIO2 当前输出是否可信，并把可信点云累积成可用于机器人前端感知的稳定地图。

## 3. FAST-LIO2 算法逻辑

FAST-LIO2 属于紧耦合 LiDAR-IMU 里程计算法。它不是简单地先用 IMU 推位置再拼点云，而是在滤波框架内同时处理 IMU 预测和 LiDAR 点云约束。

本工程中 FAST-LIO2 的作用是：

1. 使用 IMU 高频角速度和加速度做状态预测。
2. 用 IMU 辅助补偿一帧 LiDAR 扫描期间的运动畸变。
3. 直接把原始点或降采样后的点注册到地图中。
4. 根据点到局部地图结构的匹配误差修正位姿、速度、IMU bias 等状态。
5. 输出注册后的点云和里程计。

FAST-LIO2 输出的关键话题：

```bash
/cloud_registered
/cloud_registered_body
/Odometry
/path
```

需要注意：FAST-LIO2 本身能显著抑制 IMU 漂移，但它不是完整回环 SLAM。长时间大范围运行时，如果没有回环、GNSS、轮速、视觉或其他长期约束，仍然可能存在累计误差。因此本项目把 FAST-LIO2 输出再接一层可靠建图门控，避免明显坏帧污染后端地图。

## 4. LiDAR-IMU 外参和时间参数

真实系统中，MID360 的 LiDAR 和 IMU 外参、时间同步会直接影响建图质量。如果外参方向错、时间戳错、单位错，表现通常是：

- 静止时轨迹慢慢漂。
- 快速转动时墙体弯曲或双层。
- 点云整体倾斜、翻转、拉长。
- 开启 IMU 后比不开 IMU 更差。

因此当前启动文件已经留出了可直接覆盖的外参和时间参数。核心文件：

```bash
/home/accelerate/cuadc_ws/src/uav_slam_sim/launch/real_mid360_fastlio.launch.py
```

默认参数按 MID360 近似水平安装处理：

```text
lidar_to_imu_x = -0.011
lidar_to_imu_y = -0.02329
lidar_to_imu_z = 0.04412
lidar_to_imu_roll_deg = 0
lidar_to_imu_pitch_deg = 0
lidar_to_imu_yaw_deg = 0
time_offset_lidar_to_imu = 0
extrinsic_est_en = false
time_sync_en = false
```

启动时可以直接覆盖，例如：

```bash
ros2 launch uav_slam_sim real_mid360_dual_view.launch.py \
  lidar_to_imu_roll_deg:=0 \
  lidar_to_imu_pitch_deg:=0 \
  lidar_to_imu_yaw_deg:=0 \
  time_offset_lidar_to_imu:=0
```

运行时会自动生成 FAST-LIO2 实际使用的配置文件：

```bash
/home/accelerate/fastlio_runtime/fast_lio_real_mid360_runtime.yaml
```

这份 runtime yaml 的意义是：不直接污染原始 FAST-LIO2 配置，而是在每次启动时根据命令行参数生成当前实验配置，方便反复调试外参和时间偏移。

## 5. 可靠建图层算法逻辑

仅依赖 FAST-LIO2 原始输出时，遇到 IMU 飘移或退化场景，坏点云可能会被永久写入地图。本工程增加了一个可靠建图节点，对每帧点云做质量判断。

核心文件：

```bash
/home/accelerate/cuadc_ws/src/uav_slam_sim/src/fastlio_cloud_mapper_node.cpp
```

输入：

```bash
/cloud_registered_filtered
/Odometry
```

输出：

```bash
/cloud_registered_reliable
/fastlio_denoised_map
```

质量门控逻辑如下：

1. 空点云拒绝。
2. 如果要求里程计，且当前没有 `/Odometry`，拒绝。
3. 如果相邻里程计位姿跳变过大，拒绝。
4. 如果高度方向突然跳变过大，拒绝。
5. 如果估计速度超过阈值，拒绝。
6. 如果当前帧和已有稳定地图重叠率太低，且已经过了 warmup 阶段，拒绝。
7. 只有通过检查的帧才写入稳定地图。

核心思想是：FAST-LIO2 负责高频位姿估计，可靠建图层负责守住地图质量底线。当 IMU 或位姿短时失效时，宁可短时间不更新地图，也不要把坏帧写进去。对于机器人实时避障，这比单纯追求点数更多更重要。

## 6. 稳定地图更新逻辑

稳定点云地图不是无限累加，而是采用滑动窗口和体素降采样维护。

主要处理步骤：

1. 对当前帧做距离、高度、NaN、离群点过滤。
2. 对点云做体素降采样，降低计算量。
3. 根据质量门控判断是否接收这一帧。
4. 接收后加入稳定地图窗口。
5. 超过窗口长度后移除旧帧。
6. 对发布地图再做体素整理，输出稳定点云。

关键参数目前在 launch 文件中设置：

```text
enable_quality_gate = true
require_odom_for_map = true
max_odom_speed_mps = 2.5
max_odom_jump_m = 0.55
max_z_jump_m = 0.35
min_scan_map_overlap = 0.035
min_overlap_map_voxels = 800
overlap_warmup_frames = 12
overlap_neighbor_voxels = 2
overlap_sample_stride = 4
map_window_frames = 300
```

调参建议：

- 地图容易断更时，适当降低 `min_scan_map_overlap`。
- 坏帧仍然进图时，降低 `max_odom_jump_m` 和 `max_z_jump_m`。
- 边缘算力吃紧时，增大 `overlap_sample_stride` 或减小 `map_window_frames`。
- 刚启动阶段点云很少时，增大 `overlap_warmup_frames`。

## 7. 占据栅格算法逻辑

二维 SLAM 栅格图由可靠点云投影生成。核心文件：

```bash
/home/accelerate/cuadc_ws/src/uav_slam_sim/src/pointcloud_occupancy_grid_node.cpp
```

输入：

```bash
/cloud_registered_reliable
```

输出：

```bash
/fastlio_occupancy_grid
/fastlio_occupancy_free_cells
/fastlio_occupancy_occupied_cells
```

算法步骤：

1. 读取可靠点云。
2. 按高度范围过滤，保留对地面导航有意义的点。
3. 将三维点云投影到 XY 平面。
4. 按栅格分辨率离散化为二维 cell。
5. 对障碍物点所在 cell 增加占据概率。
6. 从机器人位置到障碍物点之间的射线经过 cell 记为空闲概率。
7. 使用 log-odds 累积概率，而不是单帧二值判断。
8. 对长期没有更新的 cell 做衰减，让旧错误逐步回到未知状态。
9. 发布标准 `nav_msgs/OccupancyGrid`，并额外发布黑白 marker，方便 RViz 在 WSLg 下稳定显示。

为什么要做衰减：

- 真实雷达有噪声。
- FAST-LIO2 偶发坏帧可能留下少量错误 cell。
- 动态物体会移动。
- 对避障来说，过时障碍不应该永久存在。

因此本工程让栅格有记忆，但不是永久记忆。可靠点持续观测会强化栅格状态，长时间不再观测会逐步回到未知。

## 8. RViz 双窗口显示

当前目标是同时看两类地图：

1. 三维点云图：检查 MID360 点云、FAST-LIO2 注册效果、地图是否拉飞。
2. 二维栅格图：检查后续导航规划可用的占据地图。

配置文件：

```bash
/home/accelerate/cuadc_ws/src/uav_slam_sim/config/real_mid360_pointcloud_view.rviz
/home/accelerate/cuadc_ws/src/uav_slam_sim/config/real_mid360_grid_view.rviz
```

双窗口启动文件：

```bash
/home/accelerate/cuadc_ws/src/uav_slam_sim/launch/real_mid360_dual_view.launch.py
```

WSLg 下 `Map` 显示插件偶尔会触发 OGRE 或 GLSL 问题，因此当前栅格窗口默认用 marker 显示：

- 灰色背景表示未知。
- 白色 cell 表示 free。
- 黑色 cell 表示 occupied。

标准 `/fastlio_occupancy_grid` 仍然正常发布，后续接 Nav2 或自研规划器时可以直接用。

## 9. 推荐启动流程

如果 WSLg 或 RViz 曾经卡住，先在 Windows PowerShell 执行：

```powershell
Stop-Process -Name msrdc -Force -ErrorAction SilentlyContinue
wsl --shutdown
wsl
```

进入 WSL 后执行：

```bash
cd /home/accelerate/cuadc_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch uav_slam_sim real_mid360_dual_view.launch.py
```

如果只想启动算法，不开 RViz：

```bash
cd /home/accelerate/cuadc_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch uav_slam_sim real_mid360_dual_view.launch.py start_dual_rviz:=false
```

如果只验证配置生成，不启动雷达和 FAST-LIO2：

```bash
cd /home/accelerate/cuadc_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch uav_slam_sim real_mid360_dual_view.launch.py start_livox:=false start_fast_lio:=false start_dual_rviz:=false
```

## 10. MID360 网络配置

当前 MID360 地址：

```text
192.168.1.141
```

笔记本或边缘计算机建议设置有线网卡：

```text
IPv4: 192.168.1.50
Mask: 255.255.255.0
```

连通性检查：

```bash
ping -c 3 192.168.1.141
```

如果能 ping 通但没有点云，优先检查：

1. Windows 防火墙或 WSL mirrored network 设置。
2. MID360 配置文件中的主机 IP。
3. 是否有其他程序占用了 Livox UDP 端口。
4. 是否重复启动了多个 livox driver。

## 11. 编译命令

只编译当前包：

```bash
cd /home/accelerate/cuadc_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select uav_slam_sim --symlink-install
source install/setup.bash
```

完整重新构建建议：

```bash
cd /home/accelerate/cuadc_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## 12. 关键话题清单

雷达输入：

```bash
/livox/lidar
/livox/imu
```

FAST-LIO2 输出：

```bash
/cloud_registered
/cloud_registered_body
/Odometry
/path
```

可靠建图输出：

```bash
/cloud_registered_reliable
/fastlio_denoised_map
```

二维栅格输出：

```bash
/fastlio_occupancy_grid
/fastlio_occupancy_free_cells
/fastlio_occupancy_occupied_cells
```

诊断命令：

```bash
ros2 topic list
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
ros2 topic hz /cloud_registered_reliable
ros2 topic hz /fastlio_occupancy_grid
ros2 topic echo /Odometry --once
```

## 13. 录包和复现实验

建议所有调参都录包，避免每次必须拿硬件复现。

```bash
cd /home/accelerate/cuadc_ws
mkdir -p bags
ros2 bag record -o bags/mid360_mapping_test \
  /livox/lidar \
  /livox/imu \
  /cloud_registered \
  /cloud_registered_reliable \
  /fastlio_denoised_map \
  /fastlio_occupancy_grid \
  /Odometry \
  /path
```

回放：

```bash
ros2 bag play bags/mid360_mapping_test
```

## 14. 外参初步标定建议

目前按 MID360 水平放置处理。建议按下面顺序逐步标定：

1. 雷达静止 10 到 20 秒，观察 `/Odometry` 是否稳定。
2. 只绕 Z 轴慢慢旋转，检查点云是否产生双层或反向旋转。
3. 只绕 X 轴小角度摆动，检查地面和墙面是否明显倾斜跳变。
4. 只绕 Y 轴小角度摆动，检查姿态响应方向是否合理。
5. 固定一面墙做快速左右转动，对比不同 `time_offset_lidar_to_imu` 下墙面厚度。
6. 如果开启 IMU 后比不开更差，优先怀疑时间同步、外参方向、单位、坐标轴。

常用微调命令示例：

```bash
ros2 launch uav_slam_sim real_mid360_dual_view.launch.py \
  lidar_to_imu_roll_deg:=0.5 \
  lidar_to_imu_pitch_deg:=0 \
  lidar_to_imu_yaw_deg:=0
```

```bash
ros2 launch uav_slam_sim real_mid360_dual_view.launch.py \
  time_offset_lidar_to_imu:=0.01
```

## 15. 边缘算力部署入口

Orin Nano 等边缘算力部署见：

```bash
/home/accelerate/cuadc_ws/src/uav_slam_sim/docs/orin_nano_edge_deployment.md
```

打包脚本：

```bash
/home/accelerate/cuadc_ws/src/uav_slam_sim/scripts/package_mid360_edge_release.sh
```

执行打包：

```bash
cd /home/accelerate/cuadc_ws
bash src/uav_slam_sim/scripts/package_mid360_edge_release.sh
```

生成物会放到：

```bash
/home/accelerate/cuadc_ws/releases
```

## 16. 当前边界和后续方向

当前版本完成的是可靠建图前端，不是完整自主导航闭环。已经具备：

- MID360 实时接入。
- FAST-LIO2 里程计。
- 外参和时间偏移启动参数。
- 点云质量门控。
- 稳定点云地图。
- 二维占据栅格。
- RViz 双窗口显示。
- 边缘部署说明和打包入口。

后续建议方向：

1. 增加诊断话题，发布每帧接收或拒绝原因。
2. 增加 rosbag 自动评估脚本，对重叠率、轨迹跳变、地图稳定性打分。
3. 做 LiDAR-IMU 外参离线标定流程。
4. 根据机器人实际高度、飞行高度、障碍物尺寸重设栅格高度范围。
5. 接 Nav2 或自研规划器前，先固定 `/fastlio_occupancy_grid` 的 frame、分辨率、更新频率和膨胀策略。
6. 需要长期大范围建图时，再加回环、轮速、视觉或 GNSS 等长期约束。

## 17. 一句话总结

当前方案是：MID360 提供原始点云和 IMU，FAST-LIO2 负责实时 LiDAR-IMU 里程计，可靠建图节点负责拒绝坏帧并维护稳定点云，占据栅格节点把可靠点云转成可规划的二维地图，RViz 双窗口同时观察三维点云和二维栅格。
