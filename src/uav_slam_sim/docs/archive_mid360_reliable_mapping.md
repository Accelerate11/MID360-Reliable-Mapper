# MID360 可靠建图阶段归档总结

本文档记录当前 MID360 真实雷达建图链路的阶段成果，作为后续继续开发自主避障、路径规划、机载部署时的基线版本。

## 1. 阶段目标

本阶段的目标不是直接完成自主导航，而是先把感知前端打牢：

1. 能稳定接入 MID360 实物雷达。
2. 能实时运行 FAST-LIO2，输出 LiDAR-IMU 里程计和注册点云。
3. 能在 RViz 中同时看到三维点云和二维 SLAM 栅格图。
4. 在 IMU 飘移、位姿跳变、点云错配时，不让错误数据污染稳定地图。
5. 形成可以迁移到边缘算力的工程结构和启动入口。

## 2. 已完成内容

### 2.1 MID360 接入

- MID360 默认 IP：`192.168.1.141`。
- 主机建议 IP：`192.168.1.50`。
- 驱动使用 `livox_ros_driver2`。
- 发布 `/livox/lidar` 和 `/livox/imu`。

### 2.2 FAST-LIO2 建图

- 已接入 `FAST_LIO_ROS2`。
- 已建立真实 MID360 启动文件。
- 已打通 `/cloud_registered`、`/Odometry`、`/path` 等输出。
- 已通过 runtime yaml 方式支持外参和时间偏移快速调试。

### 2.3 可靠建图门控

新增可靠建图节点，接收 FAST-LIO2 输出，判断当前帧是否可信。

拒绝条件包括：

- 点云为空。
- 里程计缺失。
- 位姿速度异常。
- 相邻帧位置跳变过大。
- 高度方向突变过大。
- 当前帧和已有稳定地图重叠率过低。

通过门控后才发布：

```bash
/cloud_registered_reliable
/fastlio_denoised_map
```

### 2.4 占据栅格

新增点云到二维栅格的转换流程：

- 先使用可靠点云。
- 再做高度过滤和 XY 投影。
- 使用 log-odds 累积 free 与 occupied 状态。
- 对长期未更新 cell 做衰减，避免旧错误永久留在地图中。
- 发布标准 `nav_msgs/OccupancyGrid` 和 RViz marker。

输出话题：

```bash
/fastlio_occupancy_grid
/fastlio_occupancy_free_cells
/fastlio_occupancy_occupied_cells
```

### 2.5 RViz 双窗口

当前使用两个 RViz 窗口：

- 点云窗口：显示三维点云、累计可靠地图、轨迹。
- 栅格窗口：显示二维 free、occupied、unknown 区域。

启动入口：

```bash
ros2 launch uav_slam_sim real_mid360_dual_view.launch.py
```

## 3. 核心算法解释

### 3.1 为什么不用纯 IMU 拼点云

IMU 可以短时间提供高频姿态和速度预测，但纯 IMU 积分一定会漂。若直接把 IMU 积分位姿用于拼点云，短时间看起来能动，长时间会出现整体地图拉伸、旋转、飘走。

因此本工程使用 FAST-LIO2 做紧耦合 LiDAR-IMU 里程计，让 LiDAR 匹配结果反过来修正 IMU bias 和位姿。

### 3.2 为什么 FAST-LIO2 后面还要加可靠建图层

FAST-LIO2 是前端里程计，不是永久可靠的全局地图守门员。实际硬件中可能出现：

- 时间同步抖动。
- 外参未完全标定。
- 雷达被遮挡。
- 环境退化。
- 手持雷达快速运动。
- IMU 初始化不足。

如果直接把所有 FAST-LIO2 输出写进全局地图，一帧坏数据就可能污染整张图。可靠建图层的作用就是把建图和里程计解耦：里程计可以短时抖动，但稳定地图只接收可信帧。

### 3.3 可靠点云如何判定

本阶段使用的是工程上可解释、可快速部署的规则门控，而不是黑盒模型。主要依据：

- 当前帧有没有足够点。
- 当前帧是否有同步的里程计。
- 当前速度是否超出物理合理范围。
- 当前里程计与上一帧是否跳变。
- 当前高度是否突然异常。
- 当前帧和已有地图是否仍有一定重叠。

重叠率判断的意义是：如果传感器位姿飘掉，当前点云会突然落到地图中完全不相干的位置。此时虽然点云本身可能很多，但不应该写入稳定地图。

### 3.4 占据栅格如何保证可用于规划

占据栅格不直接使用原始点云，而是使用 `/cloud_registered_reliable`。这样栅格的输入已经经过质量门控。

栅格更新使用 log-odds 而不是单帧二值图，原因是：

- 单帧点云稀疏或有噪声。
- 多帧一致观测更可信。
- 对实时避障来说，概率累积比一次性判断更稳。

同时加入衰减机制，避免动态物体或坏帧痕迹永久存在。

## 4. 当前工程入口

主要启动文件：

```bash
/home/accelerate/cuadc_ws/src/uav_slam_sim/launch/real_mid360_dual_view.launch.py
/home/accelerate/cuadc_ws/src/uav_slam_sim/launch/real_mid360_mapping_view.launch.py
/home/accelerate/cuadc_ws/src/uav_slam_sim/launch/real_mid360_fastlio.launch.py
```

主要源文件：

```bash
/home/accelerate/cuadc_ws/src/uav_slam_sim/src/fastlio_cloud_mapper_node.cpp
/home/accelerate/cuadc_ws/src/uav_slam_sim/src/pointcloud_occupancy_grid_node.cpp
/home/accelerate/cuadc_ws/src/uav_slam_sim/src/livox_custom_frame_aggregator_node.cpp
```

主要 RViz 配置：

```bash
/home/accelerate/cuadc_ws/src/uav_slam_sim/config/real_mid360_pointcloud_view.rviz
/home/accelerate/cuadc_ws/src/uav_slam_sim/config/real_mid360_grid_view.rviz
```

## 5. 推荐实验流程

1. 启动前让雷达静止 10 到 20 秒。
2. 确认 `/livox/lidar` 和 `/livox/imu` 频率稳定。
3. 启动双窗口建图。
4. 先缓慢平移或旋转雷达，不做大幅快速运动。
5. 观察三维点云是否双层、拉伸、倾斜。
6. 观察栅格图是否出现异常长条或整体飞走。
7. 若出现异常，先调外参和时间偏移，再调门控阈值。
8. 每次关键实验录制 rosbag，后续离线复现。

## 6. 推荐验收指标

阶段性验收可以先看这些指标：

- 静止 30 秒，轨迹不明显漂移。
- 缓慢转动时，墙面没有明显双层。
- 可靠点云持续更新，不频繁全拒绝。
- 快速晃动或遮挡时，错误点云不会永久写入稳定地图。
- 二维栅格中的 occupied 区域和真实障碍物方向一致。
- 栅格图不会因为一次坏帧整体偏移。

## 7. 已知边界

当前版本仍有边界：

- 没有回环检测。
- 没有全局优化图。
- 没有融合轮速、视觉或 GNSS。
- 外参仍需要进一步实测标定。
- 栅格图当前是导航前端输入，不含膨胀层和代价地图语义。

这些不是当前阶段的失败点，而是下一阶段接入导航和长期建图时需要补的模块。

## 8. 后续建议

下一阶段建议按这个顺序推进：

1. 增加门控诊断话题，明确每一帧通过或拒绝的原因。
2. 固化 MID360 安装支架，完成 LiDAR-IMU 外参标定。
3. 录制标准测试 bag，形成回归测试数据。
4. 针对无人机高度和障碍物尺寸重新设置栅格高度范围。
5. 将 `/fastlio_occupancy_grid` 接入规划前端，但先只做离线验证。
6. 再考虑实时避障控制闭环。
