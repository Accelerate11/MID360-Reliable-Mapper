# MID360 Reliable Mapper

**MID360 Reliable Mapper** 是一套面向 **Livox MID360** 的 ROS 2 实时可靠建图系统。项目集成了 MID360 驱动、FAST-LIO2 激光雷达-惯性里程计、点云质量门控、稳定点云地图和二维占据栅格生成，目标是在无人机、移动机器人和边缘计算平台上提供稳定、可复现、可扩展的建图前端。

本项目当前重点不是完整导航闭环，而是把导航和避障前面的地图输入做可靠：当 IMU 短时漂移、位姿跳变、环境退化或点云错配时，系统会尽量拒绝坏帧写入稳定地图，避免一次错误观测污染后续规划。

## 项目特点

- 支持 Livox MID360 实物雷达接入。
- 使用 ROS 2 Humble 工作区结构。
- 集成 FAST-LIO2 作为 LiDAR-IMU 里程计前端。
- 支持运行时配置 LiDAR-IMU 外参和时间偏移。
- 在 FAST-LIO2 输出之后加入点云质量门控。
- 输出稳定去噪点云地图 `/fastlio_denoised_map`。
- 输出二维占据栅格 `/fastlio_occupancy_grid`，便于后续导航和避障使用。
- 支持 RViz 同时显示三维点云图和二维 SLAM 栅格图。
- 提供 WSL 开发流程和 RViz 异常恢复说明。
- 提供 NVIDIA Jetson Orin Nano 等边缘算力部署教程。
- 提供源码打包脚本，方便开源发布和迁移部署。

## 项目目标

真实机器人在运行时不能只依赖原始点云。对于实时避障和路径规划来说，地图至少需要满足以下要求：

- **实时性**：地图更新频率足够支撑在线规划。
- **稳定性**：不能因为一次 IMU 漂移或位姿跳变污染整张地图。
- **可解释性**：既能查看三维点云，也能查看二维占据栅格。
- **可部署性**：既能在笔记本上调试，也能迁移到 Orin Nano 等边缘算力平台。

FAST-LIO2 本身已经提供了很强的 LiDAR-IMU 里程计能力，但在真实实验中，时间同步、外参误差、快速运动、环境退化或 IMU 初始化不足仍可能导致短时错误输出。本项目在 FAST-LIO2 后面增加可靠建图层，只允许通过一致性检查的点云写入稳定地图。

## 系统架构

```text
Livox MID360
  -> livox_ros_driver2
  -> /livox/lidar, /livox/imu
  -> FAST-LIO2
  -> /cloud_registered, /Odometry, /path
  -> 可靠点云建图节点
  -> /cloud_registered_reliable, /fastlio_denoised_map
  -> 占据栅格节点
  -> /fastlio_occupancy_grid
  -> /fastlio_occupancy_free_cells
  -> /fastlio_occupancy_occupied_cells
  -> RViz 点云窗口 + RViz 栅格窗口
```

整个系统可以分为三层：

1. **传感器层**：读取 MID360 的 LiDAR 和 IMU 数据。
2. **里程计层**：通过 FAST-LIO2 完成点云去畸变、扫描匹配和位姿估计。
3. **可靠建图层**：过滤不可信帧，生成稳定点云地图和二维占据栅格。

## 目录结构

```text
src/uav_slam_sim
  本项目主包，包含 launch 文件、RViz 配置、可靠点云建图节点、占据栅格节点、脚本和中文文档。

src/FAST_LIO_ROS2
  FAST-LIO2 的 ROS 2 实现，作为 LiDAR-IMU 里程计前端。

src/livox_ros_driver2_real
  Livox ROS Driver2 源码，用于接入 MID360 实物雷达。
```

关键文件：

```text
src/uav_slam_sim/README.md
src/uav_slam_sim/docs/archive_mid360_reliable_mapping.md
src/uav_slam_sim/docs/orin_nano_edge_deployment.md
src/uav_slam_sim/launch/real_mid360_dual_view.launch.py
src/uav_slam_sim/launch/real_mid360_fastlio.launch.py
src/uav_slam_sim/src/fastlio_cloud_mapper_node.cpp
src/uav_slam_sim/src/pointcloud_occupancy_grid_node.cpp
src/uav_slam_sim/scripts/package_mid360_github_source.sh
```

## 环境要求

推荐环境：

- Ubuntu 22.04
- ROS 2 Humble
- Livox MID360
- Livox SDK2
- PCL
- Eigen3
- CMake
- colcon

开发环境建议：

- Windows 11 + WSL2 Ubuntu 22.04，或原生 Ubuntu 22.04。
- 使用 RViz 观察点云和栅格图。
- 使用有线以太网连接 MID360。

边缘部署建议：

- NVIDIA Jetson Orin Nano 或类似边缘计算平台。
- Ubuntu 22.04 系统或容器环境。
- ROS 2 Humble。
- 机器人实机运行时建议无头运行算法，把 RViz 放在调试电脑上远程查看。

## 快速开始

编译工作区：

```bash
cd ~/cuadc_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

检查 MID360 网络连接：

```bash
ping -c 3 192.168.1.141
```

启动真实 MID360 建图和双 RViz 窗口：

```bash
cd ~/cuadc_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch uav_slam_sim real_mid360_dual_view.launch.py
```

边缘算力无头运行：

```bash
ros2 launch uav_slam_sim real_mid360_dual_view.launch.py start_dual_rviz:=false
```

## MID360 网络配置

本项目默认 MID360 地址：

```text
192.168.1.141
```

建议主机有线网卡地址：

```text
IPv4: 192.168.1.50
Mask: 255.255.255.0
```

如果可以 ping 通雷达但没有点云，优先检查：

- Livox 驱动配置中的主机 IP 是否正确。
- 防火墙或 UDP 端口是否拦截。
- 是否有其他程序已经连接雷达。
- 是否重复启动了多个 ROS launch。

## 主要 ROS 话题

传感器输入：

```text
/livox/lidar
/livox/imu
```

FAST-LIO2 输出：

```text
/cloud_registered
/cloud_registered_body
/Odometry
/path
```

可靠建图输出：

```text
/cloud_registered_reliable
/fastlio_denoised_map
```

占据栅格输出：

```text
/fastlio_occupancy_grid
/fastlio_occupancy_free_cells
/fastlio_occupancy_occupied_cells
```

常用检查命令：

```bash
ros2 topic list
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
ros2 topic hz /cloud_registered_reliable
ros2 topic hz /fastlio_occupancy_grid
ros2 topic echo /Odometry --once
```

## 算法逻辑

### 1. FAST-LIO2 里程计

FAST-LIO2 提供紧耦合 LiDAR-IMU 里程计。本项目中它主要负责：

- IMU 状态预测。
- 单帧点云运动畸变补偿。
- LiDAR scan-to-map 匹配。
- 位姿、速度和 IMU bias 估计。
- 发布注册点云、里程计和轨迹。

FAST-LIO2 是高频里程计前端，但本项目不会把它输出的每一帧都默认认为是可靠地图数据。

### 2. 运行时外参配置

真实硬件安装时，MID360 的安装姿态可能和默认假设不同。因此启动文件开放了 LiDAR-IMU 外参和时间偏移参数：

```text
lidar_to_imu_x
lidar_to_imu_y
lidar_to_imu_z
lidar_to_imu_roll_deg
lidar_to_imu_pitch_deg
lidar_to_imu_yaw_deg
time_offset_lidar_to_imu
extrinsic_est_en
time_sync_en
```

示例：

```bash
ros2 launch uav_slam_sim real_mid360_dual_view.launch.py \
  lidar_to_imu_roll_deg:=0 \
  lidar_to_imu_pitch_deg:=0 \
  lidar_to_imu_yaw_deg:=0 \
  time_offset_lidar_to_imu:=0
```

启动时会自动生成 FAST-LIO2 实际使用的运行时配置文件：

```text
/home/accelerate/fastlio_runtime/fast_lio_real_mid360_runtime.yaml
```

这样可以避免反复修改 FAST-LIO2 原始配置文件，便于外参调试和实验复现。

### 3. 可靠点云门控

可靠建图节点订阅 FAST-LIO2 的注册点云和里程计。只有通过一致性检查的点云帧才会写入稳定地图。

主要检查包括：

- 点云不能为空。
- 需要里程计时，必须已经收到 `/Odometry`。
- 当前速度不能超过合理阈值。
- 相邻帧位姿跳变不能过大。
- 高度方向突变不能过大。
- warmup 结束后，当前帧和已有稳定地图需要有足够重叠。

被接受的帧会进入稳定地图，被拒绝的帧不会污染地图。

主要输出：

```text
/cloud_registered_reliable
/fastlio_denoised_map
```

### 4. 稳定点云地图维护

稳定点云地图不是无限累积原始点，而是使用过滤和滑动窗口维护：

- 距离过滤。
- 高度过滤。
- 无效点过滤。
- 体素降采样。
- 可靠帧滑动窗口。
- 发布前再次整理点云。

这样可以让地图更适合实时机器人任务，而不是变成无限增长的原始点云堆积。

### 5. 占据栅格生成

占据栅格节点使用 `/cloud_registered_reliable` 作为输入，而不是直接使用原始点云。

处理流程：

1. 读取可靠点云。
2. 根据高度范围过滤点。
3. 将三维点投影到 XY 平面。
4. 根据栅格分辨率离散化。
5. 对障碍物点所在 cell 增加占据概率。
6. 从传感器位置到障碍物点做射线更新，标记 free cell。
7. 使用 log-odds 累积概率。
8. 对长时间未更新的 cell 做衰减，使旧错误逐步回到 unknown。

输出包括标准 `nav_msgs/OccupancyGrid`，也包括用于 RViz 稳定显示的 marker 话题。

## RViz 可视化

本项目默认使用两个 RViz 窗口：

- **点云窗口**：显示实时点云、稳定点云地图和轨迹。
- **栅格窗口**：显示 free cell、occupied cell 和轨迹。

启动命令：

```bash
ros2 launch uav_slam_sim real_mid360_dual_view.launch.py
```

如果在 WSLg 中 RViz 变成灰色加载窗口，可以在 Windows PowerShell 中执行：

```powershell
Stop-Process -Name msrdc -Force -ErrorAction SilentlyContinue
wsl --shutdown
wsl
```

然后重新进入 WSL 启动 ROS。

## 边缘算力部署

在 Jetson Orin Nano 或类似边缘算力平台上，建议无头运行：

```bash
ros2 launch uav_slam_sim real_mid360_dual_view.launch.py start_dual_rviz:=false
```

推荐部署方式：

- Orin Nano 运行 Livox 驱动、FAST-LIO2 和建图节点。
- 调试电脑远程运行 RViz。
- 使用 Release 模式编译。
- CPU 压力高时增大体素尺寸、减小地图窗口、降低栅格分辨率。
- 实地测试时录制 rosbag，便于离线分析。

完整部署教程：

```text
src/uav_slam_sim/docs/orin_nano_edge_deployment.md
```

## 数据录制

完整调试录包：

```bash
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

只录原始雷达和 IMU：

```bash
ros2 bag record -o bags/mid360_raw_only /livox/lidar /livox/imu
```

## 外参调试建议

推荐初始调试流程：

1. 启动后保持雷达静止 10 到 20 秒。
2. 检查静止时 `/Odometry` 是否稳定。
3. 每次只绕一个轴缓慢转动雷达。
4. 观察墙面是否出现双层、弯曲或反向旋转。
5. 每次只调整一个外参或时间偏移参数。
6. 每组参数保存一个短 rosbag，方便离线比较。

如果开启 IMU 后地图更差，优先检查时间同步、外参方向、坐标系约定和 IMU 单位。

## 常见问题

### 没有点云

检查：

```bash
ping -c 3 192.168.1.141
ros2 topic hz /livox/lidar
```

同时检查 Livox 配置中的主机 IP、防火墙、UDP 端口和是否重复启动驱动。

### 地图漂移或突然跳变

优先检查：

- 启动后是否静止初始化。
- LiDAR-IMU 外参是否正确。
- LiDAR 和 IMU 时间偏移是否正确。
- 雷达和机体是否刚性固定。
- 当前环境是否严重退化。

### 栅格图不更新

检查：

- `/cloud_registered_reliable` 是否有频率。
- 质量门控是否拒绝了所有帧。
- 高度过滤范围是否过窄。
- frame id 是否一致。

### 边缘端运行过慢

可以尝试：

- 关闭 RViz。
- 使用 Release 编译。
- 增大体素滤波尺寸。
- 减小地图窗口长度。
- 降低栅格分辨率。
- 减少实时录包话题数量。

## 中文文档

更详细的中文说明见：

```text
src/uav_slam_sim/README.md
src/uav_slam_sim/docs/archive_mid360_reliable_mapping.md
src/uav_slam_sim/docs/orin_nano_edge_deployment.md
```

## 后续计划

后续可以继续扩展：

- 增加点云帧通过和拒绝原因的诊断话题。
- 增加基于 rosbag 的回归测试。
- 增加 LiDAR-IMU 外参离线标定流程。
- 增加局部代价地图和障碍物膨胀层。
- 增加回环检测或全局优化模块。
- 融合轮速、视觉里程计或 GNSS 等长期约束。

## 许可证

主包 `uav_slam_sim` 在 `package.xml` 中声明为 `LGPL-3.0-only`。

本仓库还包含第三方包：

- `FAST_LIO_ROS2`，其 package 元数据声明为 BSD。
- `livox_ros_driver2_real`，其 package 元数据声明为 MIT。

重新分发时请保留第三方包自带的许可证文件。详情见 `THIRD_PARTY_NOTICE.md`。

## 项目名称

```text
MID360 Reliable Mapper
```

推荐 GitHub 仓库名：

```text
mid360-reliable-mapper
```
