# Orin Nano 边缘算力部署教程

本文档说明如何把当前 MID360 可靠建图流程部署到 NVIDIA Jetson Orin Nano 或类似边缘算力平台。目标是在边缘端运行 MID360 驱动、FAST-LIO2、可靠点云门控、二维占据栅格，并按需要选择是否开启 RViz。

推荐先在笔记本 WSL 中验证算法，再迁移到 Orin Nano。边缘端部署时，建议优先无头运行算法，把 RViz 放在调试电脑上远程订阅显示，减少机载算力压力。

## 1. 推荐硬件和系统

推荐组合：

- 计算平台：Jetson Orin Nano 8GB 优先，4GB 也能尝试但需要更激进降采样。
- 系统：Ubuntu 22.04 环境优先，便于使用 ROS 2 Humble。
- ROS：ROS 2 Humble。
- 雷达：Livox MID360。
- 网络：有线以太网直连或交换机连接。
- 电源：保证 Orin Nano 和 MID360 供电稳定。

如果你的 JetPack 版本默认是 Ubuntu 20.04，需要考虑两种路线：

1. 使用容器运行 Ubuntu 22.04 + ROS 2 Humble。
2. 升级到支持 Ubuntu 22.04 的 JetPack 版本。

不要在系统版本不匹配时硬装依赖，否则后面 PCL、Eigen、ROS 包版本很容易相互冲突。

参考官方文档：

- ROS 2 Humble 安装：`https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html`
- NVIDIA JetPack 文档：`https://docs.nvidia.com/jetson/jetpack/`
- Livox SDK2：`https://github.com/Livox-SDK/Livox-SDK2`
- livox_ros_driver2：`https://github.com/Livox-SDK/livox_ros_driver2`

## 2. 网络配置

MID360 当前 IP：

```text
192.168.1.141
```

建议 Orin Nano 有线网口设置：

```text
IPv4: 192.168.1.50
Mask: 255.255.255.0
Gateway: 留空或按实际网络设置
```

连通性检查：

```bash
ping -c 3 192.168.1.141
```

如果 ping 不通，先不要启动 ROS。优先检查：

1. 网线和供电。
2. Orin 有线网卡 IP 是否在同一网段。
3. MID360 是否被其他网卡或其他电脑占用。
4. 交换机是否正常。

## 3. 安装 ROS 2 Humble

在 Ubuntu 22.04 上安装 ROS 2 Humble：

```bash
sudo apt update
sudo apt install -y software-properties-common curl gnupg lsb-release
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

```bash
sudo apt update
sudo apt install -y ros-humble-desktop ros-dev-tools
```

加入环境变量：

```bash
echo source /opt/ros/humble/setup.bash >> ~/.bashrc
source /opt/ros/humble/setup.bash
```

如果边缘端完全无显示，可以安装更小的基础包，但调试阶段建议保留 `ros-humble-desktop`，后续再裁剪。

## 4. 安装基础依赖

```bash
sudo apt update
sudo apt install -y \
  git \
  cmake \
  build-essential \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  python3-pip \
  libpcl-dev \
  libeigen3-dev
```

初始化 rosdep：

```bash
sudo rosdep init
rosdep update
```

如果 `sudo rosdep init` 提示已经存在，可以忽略。

## 5. 安装 Livox SDK2

如果系统中还没有 Livox SDK2：

```bash
cd ~
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2
mkdir -p build
cd build
cmake ..
make -j$(nproc)
sudo make install
```

安装后建议重新打开终端，避免动态库路径没有刷新。

## 6. 解包工程

假设你已经从笔记本打包得到：

```text
mid360_reliable_mapping_edge_时间戳.tar.gz
```

在 Orin Nano 上执行：

```bash
mkdir -p ~/cuadc_ws
cd ~/cuadc_ws
tar -xzf ~/mid360_reliable_mapping_edge_时间戳.tar.gz
```

解包后应该至少看到：

```bash
src/uav_slam_sim
src/FAST_LIO_ROS2
src/livox_ros_driver2_real
EDGE_DEPLOYMENT_README.md
release_manifest.txt
```

## 7. 安装 ROS 依赖并编译

```bash
cd ~/cuadc_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

编译建议使用 Release 模式：

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

如果内存不足，可以限制并行数：

```bash
MAKEFLAGS=-j2 colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

如果 4GB 内存版本仍然编译失败，可以增加 swap 或只在笔记本交叉准备源码后在 Orin 上分包编译。

## 8. 边缘端启动命令

### 8.1 无头算法运行

机器人实际运行时推荐不开 RViz：

```bash
cd ~/cuadc_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch uav_slam_sim real_mid360_dual_view.launch.py start_dual_rviz:=false
```

### 8.2 本机带显示调试

如果 Orin 接了显示器，也可以开 RViz：

```bash
cd ~/cuadc_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch uav_slam_sim real_mid360_dual_view.launch.py
```

### 8.3 只启动算法，远程 RViz

Orin 上运行：

```bash
cd ~/cuadc_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch uav_slam_sim real_mid360_dual_view.launch.py start_dual_rviz:=false
```

调试电脑上设置相同 ROS 域，然后启动 RViz 订阅 Orin 发布的话题。两台机器需要在同一局域网，并确保 DDS 发现没有被防火墙拦住。

建议两端设置相同：

```bash
export ROS_DOMAIN_ID=42
```

## 9. Orin 性能优化建议

Orin Nano 上应尽量把算力用于算法本身，而不是可视化。

建议：

1. 正式运行时关闭 RViz。
2. 使用 Release 编译。
3. 降低点云发布和地图发布频率。
4. 增大体素滤波尺寸。
5. 减小 `map_window_frames`。
6. 增大 `overlap_sample_stride`。
7. 控制栅格地图尺寸和分辨率。
8. 只保留导航需要的高度范围。

功耗模式可按设备实际支持情况开启：

```bash
sudo nvpmodel -q
sudo nvpmodel -m 0
sudo jetson_clocks
```

不同 Orin Nano 镜像的模式编号可能不同，先用 `nvpmodel -q` 确认。

## 10. 推荐边缘参数方向

如果 Orin Nano 运行压力较大，可以优先调整这些参数：

```text
map_window_frames: 300 -> 120 或 80
overlap_sample_stride: 4 -> 6 或 8
min_overlap_map_voxels: 800 -> 500
栅格分辨率: 0.10 m -> 0.15 m 或 0.20 m
点云体素尺寸: 适当增大
```

调参原则：

- 先保证实时性，再追求细节。
- 先保证稳定地图不被污染，再追求更新速度。
- 机载避障一般更依赖局部可信地图，不一定需要无限累计全局点云。

## 11. 外参和时间偏移部署

实际装机后，雷达安装姿态可能不同。启动时可以直接覆盖：

```bash
ros2 launch uav_slam_sim real_mid360_dual_view.launch.py \
  start_dual_rviz:=false \
  lidar_to_imu_roll_deg:=0 \
  lidar_to_imu_pitch_deg:=0 \
  lidar_to_imu_yaw_deg:=0 \
  time_offset_lidar_to_imu:=0
```

调试建议：

1. 静止 10 到 20 秒后再移动。
2. 先慢速移动，不要一开始快速晃动。
3. 对墙转动观察墙面厚度。
4. 外参角度每次小幅调整，不要一次改多个轴。
5. 每组参数录一个短 bag，便于离线对比。

## 12. 录包命令

边缘端建议保留录包能力，调参时非常重要。

```bash
cd ~/cuadc_ws
mkdir -p bags
ros2 bag record -o bags/mid360_edge_test \
  /livox/lidar \
  /livox/imu \
  /cloud_registered \
  /cloud_registered_reliable \
  /fastlio_denoised_map \
  /fastlio_occupancy_grid \
  /Odometry \
  /path
```

如果磁盘压力大，只录原始雷达和 IMU：

```bash
ros2 bag record -o bags/mid360_raw_only /livox/lidar /livox/imu
```

## 13. 自启动建议

实际上机后可以使用 systemd 管理建图进程。示例服务内容：

```text
[Unit]
Description=MID360 reliable mapping
After=network-online.target

[Service]
Type=simple
User=accelerate
WorkingDirectory=/home/accelerate/cuadc_ws
ExecStart=/bin/bash -lc source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch uav_slam_sim real_mid360_dual_view.launch.py start_dual_rviz:=false
Restart=on-failure
RestartSec=3

[Install]
WantedBy=multi-user.target
```

部署 systemd 前，先手动启动稳定运行，再做自启动。

## 14. 常见问题

### 14.1 能 ping 通 MID360，但没有点云

检查：

- Livox 配置中的 host IP。
- 是否多个程序同时连接雷达。
- UDP 端口是否被防火墙拦截。
- `ros2 topic hz /livox/lidar` 是否有输出。

### 14.2 FAST-LIO2 有输出但地图飞走

优先检查：

- 雷达启动后是否静止初始化。
- LiDAR-IMU 外参方向。
- 时间偏移。
- IMU 单位是否正确。
- 雷达是否松动。

### 14.3 栅格图不更新

检查：

- `/cloud_registered_reliable` 是否有频率。
- 可靠门控是否把所有帧拒绝。
- 高度过滤范围是否过窄。
- frame 是否一致。

### 14.4 边缘端 CPU 占用过高

处理：

- 关闭 RViz。
- 降低点云密度。
- 缩小地图窗口。
- 降低栅格分辨率。
- 使用 Release 编译。
- 增加体素滤波尺寸。

## 15. 最小验收流程

部署完成后，按下面顺序验收：

1. `ping -c 3 192.168.1.141` 成功。
2. `/livox/lidar` 和 `/livox/imu` 有稳定频率。
3. `/Odometry` 有输出。
4. `/cloud_registered_reliable` 有输出。
5. `/fastlio_denoised_map` 有输出。
6. `/fastlio_occupancy_grid` 有输出。
7. 缓慢移动雷达，栅格和点云方向与真实环境一致。
8. 快速晃动或遮挡时，稳定地图不会被明显拉飞。

## 16. 建议运行模式

研发调试：

```text
Orin 运行算法 + 调试电脑远程 RViz
```

实机飞行或移动机器人测试：

```text
Orin 无头运行算法 + 只发布规划所需地图话题 + 必要时录包
```

不建议实机闭环时在 Orin 上同时开大点云 RViz，这会浪费算力并增加卡顿风险。
