# MID360 + FAST-LIO2 建图算法总结与从 0 部署复现教程

本文档用于复现当前 `uav_slam_sim` 包里的真实 MID360 建图流程。目标是从一台
Windows 笔记本 + WSL2 Ubuntu 22.04 开始，最终跑通：

```text
MID360 实时数据 -> FAST-LIO2 建图 -> RViz 显示 -> PCD 导出 -> 地图分析 -> PASS/WARN/FAIL 验收
```

当前阶段只处理建图准备，不包含避障、自主导航和路径规划。

## 1. 算法总览

### 1.1 整体数据流

```text
Livox MID360
  -> 以太网 UDP
  -> livox_ros_driver2
  -> /livox/lidar, /livox/imu
  -> FAST-LIO2
  -> /cloud_registered, /Odometry, /path
  -> fastlio_cloud_mapper_node
  -> /cloud_registered_filtered, /fastlio_denoised_map
  -> RViz / PCD / PNG / JSON / readiness report
```

各模块作用：

| 模块 | 输入 | 输出 | 作用 |
| --- | --- | --- | --- |
| `livox_ros_driver2` | MID360 UDP 包 | `/livox/lidar`, `/livox/imu` | 接收真实雷达和 IMU 数据 |
| `FAST-LIO2` | `/livox/lidar`, `/livox/imu` | `/cloud_registered`, `/Odometry`, `/path` | 激光-惯性里程计和实时建图 |
| `fastlio_cloud_mapper_node` | `/cloud_registered` | `/cloud_registered_filtered`, `/fastlio_denoised_map` | 降噪、降采样、累积地图 |
| 导出/分析脚本 | `/fastlio_denoised_map` 或 PCD | PCD/PNG/JSON/Markdown | 保存地图并做质量评估 |

### 1.2 FAST-LIO2 部分

FAST-LIO2 是当前主建图算法。它将 MID360 点云和 IMU 融合，进行实时激光惯性状态估计。

本工程中 FAST-LIO2 使用的关键输入：

```yaml
common:
  lid_topic: "/livox/lidar"
  imu_topic: "/livox/imu"

preprocess:
  lidar_type: 1
  scan_line: 4
  blind: 0.5
  timestamp_unit: 3
  scan_rate: 10

mapping:
  fov_degree: 360.0
  det_range: 30.0
  extrinsic_est_en: false
  extrinsic_T: [-0.011, -0.02329, 0.04412]
  extrinsic_R: [1., 0., 0.,
                0., 1., 0.,
                0., 0., 1.]
```

配置文件：

```text
/home/accelerate/cuadc_ws/src/uav_slam_sim/config/fast_lio_real_mid360.yaml
```

FAST-LIO2 的输出中，本工程主要使用：

| 输出 | 用途 |
| --- | --- |
| `/cloud_registered` | 每帧配准到世界坐标系后的点云 |
| `/Odometry` | 雷达/机体运动估计 |
| `/path` | 轨迹显示 |

### 1.3 后处理地图节点

`fastlio_cloud_mapper_node` 是本工程加在 FAST-LIO2 后面的地图准备节点。它不是替代
FAST-LIO2，而是把 FAST-LIO2 的实时点云整理成更适合观察、导出和后续导航准备的地图。

源码：

```text
/home/accelerate/cuadc_ws/src/uav_slam_sim/src/fastlio_cloud_mapper_node.cpp
```

处理步骤：

1. 读取 `/cloud_registered` 中的 `x/y/z/intensity`。
2. 距离裁剪：去掉太近和太远的点。
3. 高度裁剪：去掉明显异常的 z 值。
4. 扫描体素降采样：减少每帧点数，降低噪声和 RViz 压力。
5. 半径离群点滤波：每个点附近必须有足够邻居，否则认为是孤立飞点。
6. 累积体素地图：把点按地图体素归并，每个体素维护平均坐标和强度。
7. 最小命中次数筛选：低置信体素可以被抑制。
8. 地图体素上限裁剪：避免长时间运行导致内存无限增长。
9. 发布：
   - `/cloud_registered_filtered`
   - `/fastlio_denoised_map`

当前真实 MID360 建图使用的参数：

```text
scan_voxel: 0.05 m
map_voxel: 0.10 m
radius_filter: 0.22 m
radius_min_neighbors: 1
min_map_hits: 1
max_scan_points: 65000
max_map_voxels: 220000
min_range: 0.45 m
max_range: 35.0 m
z_min: -20.0 m
z_max: 10.0 m
```

这些参数在启动文件中配置：

```text
/home/accelerate/cuadc_ws/src/uav_slam_sim/launch/real_mid360_mapping_view.launch.py
```

### 1.4 地图导出算法

导出脚本订阅 `/fastlio_denoised_map`，取最后收到的一帧累积地图，保存为 ASCII PCD：

```text
/home/accelerate/cuadc_ws/src/uav_slam_sim/scripts/export_mid360_mapping_cloud.py
```

导出时还会做一次基础过滤：

- 最小距离。
- 最大距离。
- z 范围。
- stride 抽样。
- 最大点数限制。

输出：

```text
*.pcd
*.json
```

JSON 内含点数、坐标边界、地图跨度、质心等信息。

### 1.5 地图分析算法

分析脚本读取 PCD：

```text
/home/accelerate/cuadc_ws/src/uav_slam_sim/scripts/analyze_mid360_pcd_map.py
```

输出内容：

- 点数。
- x/y/z 最小值和最大值。
- x/y/z 跨度。
- 质心。
- z 方向 `p01/p50/p99` 百分位。
- 5 cm、10 cm、20 cm、50 cm 体素占用统计。
- 俯视图 + 3D 图 PNG。

20 cm 体素占用数量可以粗略反映地图覆盖程度；点数很多但体素占用很少，通常说明地图重复堆叠严重或采集范围太小。

### 1.6 两张地图对比算法

对比脚本：

```text
/home/accelerate/cuadc_ws/src/uav_slam_sim/scripts/compare_mid360_pcd_maps.py
```

核心方法：

1. 分别读取两张 PCD。
2. 按 20 cm 体素把点云离散化。
3. 计算体素集合 A、B。
4. 计算：
   - 交集体素数。
   - 并集体素数。
   - `jaccard = intersection / union`。
   - `a_coverage_by_b = intersection / A`。
   - `b_coverage_by_a = intersection / B`。
   - 两张图质心差。

用途：

- 比较在线建图和离线回放是否一致。
- 比较两次采集是否覆盖相同区域。
- 比较不同滤波参数对地图的影响。

### 1.7 建图验收算法

验收脚本：

```text
/home/accelerate/cuadc_ws/src/uav_slam_sim/scripts/evaluate_mid360_mapping_readiness.py
```

它把地图和话题质量转成 `PASS/WARN/FAIL`。

主要检查项：

| 检查项 | 目的 |
| --- | --- |
| PCD 点数 | 防止空图或极少点地图 |
| XY 地图跨度 | 判断是否真的完成了空间扫描 |
| Z 地图跨度 | 检测大范围飞点或高度漂移 |
| 20 cm 体素占用 | 判断有效覆盖是否足够 |
| `/cloud_registered_filtered` 频率 | 判断实时扫描输出是否稳定 |
| `/fastlio_denoised_map` 频率 | 判断累积地图是否持续发布 |
| 最终地图点数 | 判断地图是否足够用于后续处理 |

默认结果含义：

| 结果 | 含义 |
| --- | --- |
| `PASS` | 可以作为后续地图处理基础 |
| `WARN` | 可以人工查看，但覆盖、频率或密度存在风险 |
| `FAIL` | 不建议进入下一步，应先修复采集或建图问题 |

## 2. 从 0 部署复现

以下流程假设使用：

```text
Windows 11
WSL2 Ubuntu 22.04
ROS 2 Humble
Livox MID360
MID360 IP: 192.168.1.141
Windows 以太网 IP: 192.168.1.50
```

### 2.1 Windows 网口配置

将笔记本以太网口手动设置为：

```text
IPv4 地址: 192.168.1.50
子网掩码: 255.255.255.0
网关: 留空
DNS: 可留空
```

确认 MID360 接到这个网口，并且雷达 IP 为：

```text
192.168.1.141
```

### 2.2 WSL 网络建议

为了让 WSL 能直接访问以太网中的 MID360，建议使用 WSL mirrored networking。

在 Windows PowerShell 中打开：

```powershell
notepad $env:USERPROFILE\.wslconfig
```

可参考写入：

```ini
[wsl2]
networkingMode=mirrored
firewall=true
autoProxy=true
```

保存后重启 WSL：

```powershell
wsl --shutdown
wsl
```

如果防火墙阻挡 Livox UDP，需要用管理员 PowerShell 放行。端口范围按当前 Livox
驱动配置可先放行 56000-56501：

```powershell
New-NetFirewallRule `
  -DisplayName "WSL Livox MID360 UDP" `
  -Direction Inbound `
  -Action Allow `
  -Protocol UDP `
  -LocalPort 56000-56501 `
  -RemoteAddress 192.168.1.141 `
  -Profile Any
```

### 2.3 安装 ROS 2 Humble

如果系统还没有 ROS 2 Humble，按 Ubuntu 22.04 安装 ROS 2 Humble Desktop。
安装完成后应能执行：

```bash
source /opt/ros/humble/setup.bash
ros2 --version
rviz2 --help
```

常用依赖：

```bash
sudo apt update
sudo apt install -y \
  build-essential \
  cmake \
  git \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-pip \
  ros-humble-rviz2 \
  ros-humble-sensor-msgs-py \
  ros-humble-tf2-ros \
  ros-humble-nav-msgs \
  ros-humble-geometry-msgs
```

如果是首次使用 rosdep：

```bash
sudo rosdep init
rosdep update
```

### 2.4 创建工作空间

```bash
mkdir -p /home/accelerate/cuadc_ws/src
cd /home/accelerate/cuadc_ws
source /opt/ros/humble/setup.bash
```

### 2.5 准备源码

当前工程至少需要这几个包：

```text
src/FAST_LIO_ROS2
src/livox_ros_driver2
src/uav_slam_sim
```

如果是复制已有工程，保持目录结构如下：

```text
/home/accelerate/cuadc_ws/
  src/
    FAST_LIO_ROS2/
    livox_ros_driver2/
    uav_slam_sim/
```

如果是从空 workspace 拉取，需要准备：

- FAST-LIO2 的 ROS 2 版本源码。
- Livox ROS Driver 2。
- 本包 `uav_slam_sim`。

注意：不同 Livox driver 分支的 launch/config 路径可能不同；本工程已经按当前
workspace 中的 `livox_ros_driver2` 和 `FAST_LIO_ROS2` 适配好。

### 2.6 安装依赖并编译

```bash
cd /home/accelerate/cuadc_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

只重编本包：

```bash
colcon build --packages-select uav_slam_sim --symlink-install
source install/setup.bash
```

建议加入 `~/.bashrc`：

```bash
source /opt/ros/humble/setup.bash
source /home/accelerate/cuadc_ws/install/setup.bash
```

### 2.7 检查 MID360 网络

```bash
ping -c 3 192.168.1.141
```

正常输出应为 0% packet loss。

如果 ping 不通：

1. 检查 Windows 以太网 IP。
2. 检查雷达供电和网线。
3. 检查 MID360 IP 是否确实为 `192.168.1.141`。
4. 重启 WSL。
5. 关闭或放行 Windows 防火墙后再测。

### 2.8 启动完整建图

进入工作空间：

```bash
cd /home/accelerate/cuadc_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

运行 60 秒完整采集：

```bash
/home/accelerate/cuadc_ws/install/uav_slam_sim/share/uav_slam_sim/scripts/run_real_mid360_mapping_session.sh 60
```

启动后操作建议：

1. 前 5-10 秒保持 MID360 静止。
2. 缓慢移动雷达。
3. 优先扫墙面、桌面、门框、障碍物等结构明显区域。
4. 避免急转、猛晃和快速上下摆动。
5. 尽量回到起点附近，便于观察漂移。

输出目录：

```text
/home/accelerate/cuadc_ws/mid360_mapping_sessions/session_YYYYMMDD_HHMMSS/
```

### 2.9 查看 RViz

完整建图脚本会自动启动 RViz。

如果只想启动建图和 RViz：

```bash
/home/accelerate/cuadc_ws/install/uav_slam_sim/share/uav_slam_sim/scripts/start_real_mid360_fastlio_rviz.sh
```

如果建图已经在跑，只重启 RViz：

```bash
/home/accelerate/cuadc_ws/install/uav_slam_sim/share/uav_slam_sim/scripts/start_real_mid360_rviz_only.sh
```

如果 RViz 能打开但没点云，检查：

```bash
ros2 topic hz /cloud_registered_filtered
ros2 topic hz /fastlio_denoised_map
ros2 topic echo /Odometry --once
```

如果 RViz 窗口卡住或空白，先在 Windows PowerShell 中执行：

```powershell
wsl --shutdown
```

然后重新进入 Ubuntu 再启动。

### 2.10 查看一次建图结果

进入 session 目录：

```bash
cd /home/accelerate/cuadc_ws/mid360_mapping_sessions/session_YYYYMMDD_HHMMSS
```

重点文件：

```text
summary.md
mapping_readiness.md
maps/final_fastlio_denoised_map.pcd
maps/final_fastlio_denoised_map.png
maps/final_fastlio_denoised_map.json
maps/final_fastlio_denoised_map.analysis.json
logs/final_quality.txt
bag/real_mid360_mapping/
```

最先看：

```bash
cat mapping_readiness.md
```

如果显示：

```text
verdict: PASS
```

说明这次数据可以作为后续地图处理基础。

### 2.11 手动质量检查

建图运行中可以随时执行：

```bash
/home/accelerate/cuadc_ws/install/uav_slam_sim/share/uav_slam_sim/scripts/check_real_mid360_mapping_quality.sh
```

完整诊断：

```bash
/home/accelerate/cuadc_ws/install/uav_slam_sim/share/uav_slam_sim/scripts/diagnose_real_mid360_mapping_pipeline.sh
```

正常参考：

```text
/cloud_registered_filtered: 约 10 Hz
/fastlio_denoised_map: 持续非空
frame_id: camera_init
地图范围: 室内尺度
```

### 2.12 手动导出地图

如果实时建图已经在跑，可以手动导出当前地图：

```bash
/home/accelerate/cuadc_ws/install/uav_slam_sim/share/uav_slam_sim/scripts/export_real_mid360_map.sh
```

默认输出：

```text
/home/accelerate/cuadc_ws/mid360_mapping_outputs/
```

### 2.13 手动分析 PCD

```bash
/home/accelerate/cuadc_ws/install/uav_slam_sim/share/uav_slam_sim/scripts/analyze_real_mid360_map.sh \
  /home/accelerate/cuadc_ws/mid360_mapping_sessions/session_YYYYMMDD_HHMMSS/maps/final_fastlio_denoised_map.pcd
```

输出：

```text
final_fastlio_denoised_map.analysis.json
final_fastlio_denoised_map.png
```

### 2.14 离线回放复现

不接 MID360，也可以用录好的 raw bag 重新跑 FAST-LIO2：

```bash
/home/accelerate/cuadc_ws/install/uav_slam_sim/share/uav_slam_sim/scripts/replay_real_mid360_mapping_bag.sh \
  /home/accelerate/cuadc_ws/mid360_mapping_sessions/session_YYYYMMDD_HHMMSS/bag/real_mid360_mapping
```

输出目录：

```text
/home/accelerate/cuadc_ws/mid360_mapping_replays/replay_real_mid360_mapping_YYYYMMDD_HHMMSS/
```

用途：

- 验证同一组数据能否稳定复现地图。
- 改 FAST-LIO2 或滤波参数后做 A/B 对比。
- 电脑不接雷达时继续调算法。

### 2.15 对比在线图和回放图

```bash
/home/accelerate/cuadc_ws/install/uav_slam_sim/share/uav_slam_sim/scripts/compare_real_mid360_maps.sh \
  /home/accelerate/cuadc_ws/mid360_mapping_sessions/session_YYYYMMDD_HHMMSS/maps/final_fastlio_denoised_map.pcd \
  /home/accelerate/cuadc_ws/mid360_mapping_replays/replay_xxx/maps/replayed_fastlio_denoised_map.pcd
```

输出：

```text
*.compare.json
*.compare.md
```

主要看：

```text
jaccard
a_coverage_by_b
b_coverage_by_a
centroid_delta_b_minus_a_m
```

### 2.16 停止所有相关进程

```bash
/home/accelerate/cuadc_ws/install/uav_slam_sim/share/uav_slam_sim/scripts/stop_mid360_slam.sh
```

## 3. 完整复现检查清单

按顺序确认：

1. Windows 以太网 IP 是 `192.168.1.50/24`。
2. MID360 IP 是 `192.168.1.141`。
3. `ping -c 3 192.168.1.141` 成功。
4. `source /opt/ros/humble/setup.bash` 成功。
5. `source /home/accelerate/cuadc_ws/install/setup.bash` 成功。
6. `colcon build --symlink-install` 成功。
7. `ros2 topic list` 能运行。
8. 运行 `run_real_mid360_mapping_session.sh 60`。
9. RViz 能看到 `/fastlio_denoised_map`。
10. session 目录生成 rosbag。
11. session 目录生成 PCD。
12. session 目录生成 PNG。
13. `mapping_readiness.md` 存在。
14. `mapping_readiness.md` 为 `PASS` 或至少没有 `FAIL`。
15. 可以用 replay 脚本离线复现。

## 4. 常见故障定位

### 4.1 ping 通但没有 `/livox/lidar`

检查：

```bash
ros2 topic list | grep livox
```

可能原因：

- Windows 防火墙拦截 UDP。
- WSL 网络不是 mirrored。
- Livox driver 配置中的 MID360 IP 不一致。
- 雷达还没有进入 Normal 工作模式。

### 4.2 有 `/livox/lidar`，但没有 `/cloud_registered`

检查：

```bash
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
```

可能原因：

- FAST-LIO2 没启动。
- FAST-LIO2 订阅话题配置错误。
- IMU 数据异常。
- 时间戳单位或时间同步配置不匹配。
- 雷达-IMU 外参不合适。

### 4.3 有 `/cloud_registered`，但地图很少

检查：

```bash
ros2 topic hz /cloud_registered_filtered
ros2 topic hz /fastlio_denoised_map
```

可能原因：

- MID360 没有移动。
- 场景几何结构少。
- 过滤参数过强。
- 导出时机太早。

### 4.4 地图发散或尺度很大

可能原因：

- 初始化没有静置。
- 雷达快速晃动。
- IMU 数据异常。
- 时间戳/外参错误。
- FAST-LIO2 已经漂移。

处理建议：

1. 停止所有建图进程。
2. 重新启动。
3. 启动后静置 5-10 秒。
4. 缓慢移动。
5. 重新检查 `mapping_readiness.md`。

### 4.5 RViz 卡死或窗口空白

处理：

```powershell
wsl --shutdown
```

重新进入 Ubuntu：

```bash
cd /home/accelerate/cuadc_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
/home/accelerate/cuadc_ws/install/uav_slam_sim/share/uav_slam_sim/scripts/start_real_mid360_rviz_only.sh
```

## 5. 当前实测基线

已经验证过的一组真实 MID360 数据：

```text
/home/accelerate/cuadc_ws/mid360_mapping_sessions/session_20260623_145528
```

结果：

```text
points: 9392
xy_span_m: 13.448
z_span_m: 3.142
occupied_voxels_20cm: 3367
readiness: PASS
score: 100
```

在线建图和离线回放对比：

```text
jaccard: 0.649
a_coverage_by_b: 0.655
b_coverage_by_a: 0.987
centroid_delta: x=0.031 m, y=0.114 m, z=0.053 m
```

这个基线说明：

- FAST-LIO2 主体建图链路已打通。
- 离线 replay 能复现主体结构。
- 当前点云地图可进入后续地图处理准备阶段。

## 6. 下一步建议

在进入自主导航或避障前，建议继续完成：

1. 固定 MID360 安装位姿，减少手持晃动影响。
2. 做多次同场景采集，对比 `jaccard` 和质心漂移。
3. 根据无人机尺度确定最终地图体素分辨率。
4. 把 `/fastlio_denoised_map` 转成局部占据栅格或 ESDF。
5. 再接入 D435i，并做 MID360-D435i 外参标定。
6. 最后再连接路径规划和避障模块。

原则：先保证 MID360 单传感器建图稳定，再引入 D435i 和导航算法。
