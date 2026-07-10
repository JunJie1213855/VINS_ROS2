# VINS-Fusion-ROS2 使用指南

## 1. 项目概述

**VINS-Fusion** 是由香港科技大学空中机器人组（Aerial Robotics Group, HKUST）开发的一种基于优化的多传感器融合 SLAM 框架。本项目 [VINS-Fusion-ROS2](https://github.com/zinuok/VINS-Fusion-ROS2) 是其在 ROS2 Humble 上的移植版本，支持**视觉-惯性里程计 (VIO)**、**全局位姿图优化** 和 **回环检测** 三大核心功能。

### 支持的传感器组合

| 模式 | IMU | 相机数 | 典型场景 |
|------|-----|--------|----------|
| 双目+IMU | ✓ | 2 | EuRoC MAV, Realsense D435i |
| 单目+IMU | ✓ | 1 | MyntEye, 无人机 |
| 双目 (无IMU) | ✗ | 2 | KITTI Odometry |
| GPS融合 | ✓ | 1-2 | 室外大场景 |

---

## 2. 系统架构

```
┌─────────────────────────────────────────────────┐
│                    VINS-Fusion                    │
├──────────┬────────────┬─────────────┬────────────┤
│ camera_  │   vins     │ loop_fusion │  global_   │
│ models   │ (VIO核心)  │ (回环检测)  │  fusion    │
│          │            │             │ (GPS融合)  │
├──────────┼────────────┼─────────────┼────────────┤
│ 相机标定 │ 特征追踪   │ 回环检测    │ GPS全局    │
│ 参数库   │ 视觉-惯性  │ 位姿图优化  │ 位姿图优化 │
│          │ 紧耦合优化 │ 重定位      │            │
│          │ 滑动窗口   │             │            │
├──────────┼────────────┼─────────────┼────────────┤
│ library  │ vins_node  │ loop_fusion │ global_    │
│ (库)     │ (可执行)   │ _node (可执行)│ fusion_node│
└──────────┴────────────┴─────────────┴────────────┘
```

### 2.1 各包详解

#### `camera_models` — 相机模型库
通用相机标定模型库（基于 [camodocal](https://github.com/hengli/camodocal)），支持多种畸变模型：
- **PinholeCamera** — 针孔相机
- **PinholeFullCamera** — 全畸变针孔（含径向+切向）
- **CataCamera** — Mei 全景模型（catadioptric + fisheye）
- **EquidistantCamera** — 等距投影鱼眼 (Kannala-Brandt)
- **ScaramuzzaCamera** — Scaramuzza 鱼眼模型

#### `vins` — 视觉-惯性状态估计器（核心）
VIO 前端+后端紧耦合系统，处理流程为：

```
IMU数据 ──┬──> 预积分 ────────────┐
          │                      ├──> 滑动窗口紧耦合优化 ──> 位姿/速度/偏置
图像数据 ─┴──> 光流特征追踪 ──> 三角化 ──┘
```

**源码子模块：**
| 子模块 | 文件 | 功能 |
|--------|------|------|
| `featureTracker/` | `feature_tracker.h/.cpp` | KLT 光流特征追踪 |
| `estimator/` | `estimator.h/.cpp` | 滑动窗口优化器（核心后端） |
| `factor/` | `pose_local_parameterization.h/.cpp` 等 | Ceres 优化因子与参数化 |
| `initial/` | `initial_sfm.cpp` | 视觉 SfM 初始化 |
| `utility/` | `visualization.h/.cpp` | 可视化工具（路径/点云发布） |

**生成的可执行文件：**
- `vins_node` — 主 VIO 节点（实时运行）
- `kitti_odom_test` — KITTI 里程计评测
- `kitti_gps_test` — KITTI GPS 融合评测

#### `loop_fusion` — 回环检测与重定位
使用 **DBoW2** 词袋模型进行回环检测，结合位姿图优化消除累积漂移：

```
关键帧 ──> DBoW2 词袋查询 ──> 回环候选 ──> 几何验证 ──> 位姿图优化 (4-DoF/6-DoF)
```

#### `global_fusion` — GPS 全局融合
将 VIO 局部估计与 GPS 全局测量融合，适用于室外大场景：

```
VIO 位姿 ─┬──> 全局位姿图 (ceres优化) ──> 全局一致的轨迹
GPS 测量 ─┘
```

---

## 3. 环境依赖

### 3.1 系统要求

| 组件 | 版本 |
|------|------|
| 操作系统 | Ubuntu 22.04 |
| ROS2 | Humble Hawksbill |
| CMake | ≥ 3.5 |
| C++ 标准 | C++14 |

### 3.2 核心依赖库

| 库 | 版本 | 用途 |
|----|------|------|
| [Ceres Solver](http://ceres-solver.org/) | 2.0.0 (apt) | 非线性最小二乘优化 |
| [Eigen3](https://eigen.tuxfamily.org/) | ≥ 3.3 | 线性代数 |
| [OpenCV](https://opencv.org/) | ≥ 4.5 (Humble自带) | 图像处理、特征追踪 |
| [cv_bridge](https://github.com/ros-perception/vision_opencv) | Humble自带 | ROS-OpenCV 图像转换 |

### 3.3 ROS2 依赖

```
rclcpp, rcpputils, std_msgs, geometry_msgs, sensor_msgs,
visualization_msgs, nav_msgs, tf2, tf2_ros, cv_bridge,
image_transport, ament_index_cpp
```

### 3.4 可选依赖

| 库 | 用途 | 备注 |
|----|------|------|
| OpenCV CUDA | GPU 光流加速 | 需自行编译 OpenCV with CUDA |
| Ceres ≥ 2.1 | GPU 线性求解 (`ceres::CUDA`) | 当前系统 Ceres 2.0 不支持 |

---

## 4. 编译指南

### 4.1 首次编译

```bash
# 1. 进入工作空间
cd /home/ros/rosws/vins_fusion_ws

# 2. 安装缺失依赖（如需要）
sudo apt install -y ros-humble-image-transport

# 3. Source ROS2 环境
source /opt/ros/humble/setup.sh

# 4. 编译（16 线程并行）
MAKEFLAGS="-j16" colcon build --parallel-workers 16 \
  --build-base /home/ros/rosws/vins_fusion_ws/build \
  --install-base /home/ros/rosws/vins_fusion_ws/install
```

### 4.2 仅编译特定包

```bash
# 仅编译 vins
colcon build --packages-select vins --parallel-workers 16

# 编译 vins 和 loop_fusion
colcon build --packages-select vins loop_fusion --parallel-workers 16
```

### 4.3 清理重新编译

```bash
rm -rf build install log
# 同时清理源码树内可能的过期构建产物
rm -rf src/VINS-Fusion-ROS2/build src/VINS-Fusion-ROS2/install
# 重新编译
MAKEFLAGS="-j16" colcon build --parallel-workers 16 \
  --build-base /home/ros/rosws/vins_fusion_ws/build \
  --install-base /home/ros/rosws/vins_fusion_ws/install
```

### 4.4 GPU 模式编译

如需启用 GPU 光流加速（需要 OpenCV with CUDA）：

1. 编辑 `vins/src/featureTracker/feature_tracker.h` 第 14 行，**取消注释**：
   ```cpp
   #define GPU_MODE 1
   ```

2. 重新编译

> **注意：** Ceres GPU (`use_gpu_ceres`) 在 Ceres 2.0 上不可用，已通过预处理器自动禁用。即使配置文件中 `use_gpu_ceres: 1`，也会自动回退到 CPU 求解器。

---

## 5. 配置文件详解

配置文件位于 `src/VINS-Fusion-ROS2/config/`，按传感器/数据集分为 8 个子目录。

### 5.1 配置目录一览

| 目录 | 目标平台 | 传感器 |
|------|----------|--------|
| `euroc/` | EuRoC MAV 数据集 | 双目/单目 + IMU |
| `kitti_odom/` | KITTI Odometry | 双目（无IMU） |
| `kitti_raw/` | KITTI Raw | 双目 + GPS |
| `realsense_d435i/` | Intel RealSense D435i | 双目 + IMU |
| `mynteye/` | 小觅 MyntEye | 单目/双目 + IMU |
| `A3_ptgrey/` | DJI A3 + PointGrey | 单目/双目 + IMU |
| `vi_car/` | 车载视觉-惯性系统 | 双目 + IMU |
| `simulation/` | 仿真测试 | 双目 + IMU |

### 5.2 核心配置参数

以 `config/euroc/euroc_stereo_imu_config.yaml` 为例：

#### 传感器设置
```yaml
imu: 1                    # 是否使用 IMU (0/1)
num_of_cam: 2             # 相机数量 (1 或 2)
imu_topic: "/imu0"        # IMU 话题名
image0_topic: "/cam0/image_raw"   # 左相机话题
image1_topic: "/cam1/image_raw"   # 右相机话题
output_path: "~/output/"  # 输出路径
```

#### 相机标定
```yaml
cam0_calib: "cam0_mei.yaml"   # 左相机标定文件
cam1_calib: "cam1_mei.yaml"   # 右相机标定文件
image_width: 752               # 图像宽度
image_height: 480              # 图像高度
```

#### GPU 加速
```yaml
use_gpu         : 0    # GPU 光流（需 OpenCV CUDA）
use_gpu_acc_flow: 0    # GPU 加速光流
use_gpu_ceres   : 0    # GPU Ceres 求解（需 Ceres ≥ 2.1）
```

> **当前系统状态：** 三项 GPU 开关均应设为 `0`。

#### 外参估计
```yaml
estimate_extrinsic: 0   # 0=使用已有外参（信任标定结果）
                        # 1=在线优化外参（仅初始猜测）
body_T_cam0: !!opencv-matrix   # IMU到左相机变换矩阵 (4x4)
body_T_cam1: !!opencv-matrix   # IMU到右相机变换矩阵 (4x4)
```

#### 特征追踪
```yaml
max_cnt: 150            # 最大特征点数
min_dist: 30            # 特征点最小间距 (像素)
freq: 10                # 追踪结果发布频率 (Hz)
F_threshold: 1.0        # RANSAC 阈值 (像素)
show_track: 1           # 发布追踪图像话题
flow_back: 1            # 双向光流验证
```

#### 优化参数
```yaml
max_solver_time: 0.04   # 最大求解时间 (秒)，保证实时
max_num_iterations: 8   # 最大迭代次数
keyframe_parallax: 10.0 # 关键帧选择视差阈值 (像素)
```

#### IMU 噪声参数
```yaml
acc_n: 0.1              # 加速度计噪声标准差
gyr_n: 0.01             # 陀螺仪噪声标准差
acc_w: 0.001            # 加速度计随机游走
gyr_w: 0.0001           # 陀螺仪随机游走
g_norm: 9.81007         # 重力加速度幅值
```

#### 回环检测
```yaml
load_previous_pose_graph: 0         # 加载之前的位姿图
pose_graph_save_path: "~/output/pose_graph/"
save_image: 1                       # 保存图像用于可视化
```

### 5.3 相机标定文件

每个传感器目录包含独立的相机标定文件（`.yaml`），定义相机内参和畸变模型：

```yaml
# 示例: config/euroc/cam0_mei.yaml
model_type: MEI
camera_name: cam0
image_width: 752
image_height: 480
mirror_parameters:
  xi: 2.426691e+00     # Mei 模型 mirror 参数
distortion_parameters:
  k1: 9.518994e-02     # 径向畸变
  k2: -2.414967e-02
  p1: 1.763412e-05     # 切向畸变
  p2: -4.587994e-05
projection_parameters:
  gamma1: 4.616068e+02 # 焦距 (fx)
  gamma2: 4.605213e+02 # 焦距 (fy)
  u0: 3.654745e+02     # 主点 (cx)
  v0: 2.509729e+02     # 主点 (cy)
```

---

## 6. 使用方法

### 6.1 环境准备

每次新终端首先执行：

```bash
source /opt/ros/humble/setup.sh
source /home/ros/rosws/vins_fusion_ws/install/local_setup.sh
```

### 6.2 快速启动

```bash
# 启动 VINS（默认 EuRoC 配置）
ros2 launch vins euroc.launch.py

# 启动 RViz 可视化
ros2 launch vins vins_rviz.launch.py
```

### 6.3 使用自定义配置

```bash
# 方式 1：通过 launch 参数
ros2 launch vins euroc.launch.py \
  config_path:=/home/ros/rosws/vins_fusion_ws/install/vins/share/vins/config/realsense_d435i/realsense_stereo_imu_config.yaml

# 方式 2：直接用 ros2 run
ros2 run vins vins_node /path/to/your_config.yaml
```

### 6.4 典型使用场景

#### 场景 A：EuRoC MAV 数据集

```bash
# 终端 1：启动 VINS-Fusion
source install/local_setup.sh
ros2 launch vins euroc.launch.py

# 终端 2：启动 RViz 可视化
ros2 launch vins vins_rviz.launch.py

# 终端 3：播放 bag 文件
ros2 bag play MH_01_easy
```

#### 场景 B：Intel RealSense D435i 实时运行

```bash
# 终端 1：启动 RealSense 相机
ros2 launch realsense2_camera rs_launch.py

# 终端 2：启动 VINS
ros2 launch vins euroc.launch.py \
  config_path:=/home/ros/rosws/vins_fusion_ws/install/vins/share/vins/config/realsense_d435i/realsense_stereo_imu_config.yaml

# 终端 3：可视化
ros2 launch vins vins_rviz.launch.py
```

#### 场景 C：KITTI Odometry（无 IMU，纯双目）

```bash
ros2 run vins kitti_odom_test \
  /home/ros/rosws/vins_fusion_ws/install/vins/share/vins/config/kitti_odom/kitti_config00-02.yaml \
  /path/to/KITTI/dataset/sequences/00
```

#### 场景 D：GPS 全局融合

```bash
# 终端 1：启动 VINS
ros2 launch vins euroc.launch.py

# 终端 2：启动全局融合（订阅 /vins_estimator/odometry 和 /gps）
ros2 run global_fusion global_fusion_node
```

#### 场景 E：带回环检测的完整 SLAM

```bash
# 终端 1：VINS-VIO
ros2 launch vins euroc.launch.py

# 终端 2：回环检测（订阅 VIO 关键帧/位姿）
ros2 run loop_fusion loop_fusion_node /path/to/config.yaml

# 终端 3：可视化
ros2 launch vins vins_rviz.launch.py
```

### 6.5 Launch 文件说明

| Launch 文件 | 功能 |
|-------------|------|
| `vins/launch/euroc.launch.py` | 启动 VINS 节点，接收 `config_path` 参数 |
| `vins/launch/vins_rviz.launch.py` | 启动 RViz2 可视化界面 |

---

## 7. ROS2 话题接口

### 7.1 vins_node 订阅的话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/imu0` | `sensor_msgs/Imu` | IMU 数据 |
| `/cam0/image_raw` | `sensor_msgs/Image` | 左相机图像 |
| `/cam1/image_raw` | `sensor_msgs/Image` | 右相机图像（双目模式） |

### 7.2 vins_node 发布的话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/vins_estimator/imu_propagate` | `sensor_msgs/Imu` | IMU 传播结果 |
| `/vins_estimator/odometry` | `nav_msgs/Odometry` | VIO 里程计 |
| `/vins_estimator/path` | `nav_msgs/Path` | 运动轨迹 |
| `/vins_estimator/point_cloud` | `sensor_msgs/PointCloud` | 稀疏点云 |
| `/vins_estimator/margin_cloud` | `sensor_msgs/PointCloud` | 边缘化点云 |
| `/vins_estimator/key_poses` | `sensor_msgs/PointCloud` | 关键帧位姿 |
| `/vins_estimator/camera_pose` | `geometry_msgs/PoseStamped` | 相机位姿 |
| `/vins_estimator/camera_pose_visual` | `visualization_msgs/Marker` | 相机位姿可视化 |
| `/vins_estimator/keyframe_pose` | `geometry_msgs/PoseWithCovarianceStamped` | 关键帧带协方差位姿 |
| `/vins_estimator/keyframe_point` | `sensor_msgs/PointCloud` | 关键帧点云 |
| `/vins_estimator/extrinsic` | `nav_msgs/Odometry` | 在线标定的外参 |
| `/vins_estimator/image_track` | `sensor_msgs/Image` | 特征追踪可视化图像 |

### 7.3 loop_fusion_node

| 方向 | 话题 | 类型 |
|------|------|------|
| 订阅 | `/vins_estimator/key_poses` | `sensor_msgs/PointCloud` |
| 订阅 | `/vins_estimator/camera_pose` | `geometry_msgs/PoseStamped` |
| 订阅 | `/vins_estimator/keyframe_point` | `sensor_msgs/PointCloud` |
| 发布 | `/pose_graph/loop_path` | `nav_msgs/Path` |

### 7.4 global_fusion_node

| 方向 | 话题 | 类型 |
|------|------|------|
| 订阅 | `/vins_estimator/odometry` | `nav_msgs/Odometry` |
| 订阅 | `/gps` | `sensor_msgs/NavSatFix` |
| 发布 | `/global_odometry` | `nav_msgs/Odometry` |
| 发布 | `/global_path` | `nav_msgs/Path` |

---

## 8. 兼容性修改说明

本项目原针对 Ceres 2.1.0+ 和 OpenCV CUDA 设计。在当前环境（Ceres 2.0.0, ROS2 Humble）下编译时，以下修改已自动应用（详见 `docs/problem/` 中的记录）：

| 问题 | 修改文件 | 解决方案 |
|------|----------|----------|
| Ceres `Manifold` API 不存在 | 8 个 `.h/.cpp/.cc` 文件 | 降级为 `LocalParameterization` API |
| `ceres::CUDA` 不存在 | `vins/src/estimator/estimator.cpp` | `#if` 预处理器版本检查回退 |
| `AutoDiffManifold` vs `AutoDiffLocalParameterization` | `loop_fusion/src/pose_graph.h` | 改用 `operator()` 风格 functor |
| `problem.SetManifold` vs `SetParameterization` | `camera_models/src/calib/CameraCalibration.cc` | 改为 `SetParameterization` |

---

## 9. 常用调试技巧

### 9.1 验证安装

```bash
# 确认所有包已正确注册
ros2 pkg list | grep -E "vins|fusion|camera_models"

# 查看节点帮助
ros2 run vins vins_node --help
```

### 9.2 查看话题流

```bash
# 列出活跃话题
ros2 topic list

# 查看 VIO 里程计输出
ros2 topic echo /vins_estimator/odometry

# 查看话题频率
ros2 topic hz /vins_estimator/odometry
```

### 9.3 可视化调试

```bash
# RViz 可视化
ros2 launch vins vins_rviz.launch.py

# 查看计算图
rqt_graph
```

### 9.4 Bag 录制与回放

```bash
# 录制所有 VINS 话题
ros2 bag record -o vins_output \
  /vins_estimator/odometry \
  /vins_estimator/path \
  /vins_estimator/point_cloud

# 回放时可用 --remap 重映射话题
ros2 bag play vins_output --remap /imu0:=/your_imu_topic
```

---

## 10. 目录结构总览

```
vins_fusion_ws/
├── src/
│   └── VINS-Fusion-ROS2/
│       ├── vins/                  # VIO 核心 (vins_node)
│       │   ├── src/
│       │   │   ├── estimator/     # 滑动窗口优化器
│       │   │   ├── factor/        # Ceres 优化因子
│       │   │   ├── featureTracker/# 光流特征追踪
│       │   │   ├── initial/       # SfM 初始化
│       │   │   └── utility/       # 可视化、工具函数
│       │   ├── launch/            # ROS2 launch 文件
│       │   └── CMakeLists.txt
│       ├── loop_fusion/           # 回环检测与重定位
│       │   └── src/
│       │       ├── pose_graph.h/.cpp
│       │       ├── pose_graph_node.cpp
│       │       ├── keyframe.h/.cpp
│       │       └── ThirdParty/DBoW/
│       ├── global_fusion/         # GPS 全局融合
│       │   └── src/
│       │       ├── globalOpt.h/.cpp
│       │       └── globalOptNode.cpp
│       ├── camera_models/         # 相机模型库 (camodocal)
│       │   ├── include/camodocal/
│       │   └── src/
│       ├── config/                # 配置文件
│       │   ├── euroc/
│       │   ├── kitti_odom/
│       │   ├── kitti_raw/
│       │   ├── realsense_d435i/
│       │   ├── mynteye/
│       │   ├── A3_ptgrey/
│       │   ├── vi_car/
│       │   └── simulation/
│       ├── support_files/         # DBoW2 词袋文件
│       └── docker/                # Docker 支持
├── build/                         # 编译中间文件
├── install/                       # 安装目标
├── log/                           # 编译日志
└── docs/
    ├── introduction.md            # 本文档
    └── problem/                   # 编译问题记录
        ├── 001-cmake-unknown-argument-j16.md
        ├── 002-ceres-api-incompatibility.md
        ├── 003-missing-image-transport.md
        ├── 004-angle-manifold-functor-api.md
        ├── 005-ceres-cuda-not-available.md
        ├── 006-install-prefix-in-source.md
        └── 007-final-summary.md
```

---

## 11. 参考资料

- [原始 VINS-Fusion 论文](https://arxiv.org/abs/1712.03829) — Qin T., Li P., Shen S.
- [VINS-Fusion ROS1 版本](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion)
- [VINS-Fusion-ROS2 GitHub](https://github.com/zinuok/VINS-Fusion-ROS2)
- [Ceres Solver 文档](http://ceres-solver.org/)
- [DBoW2 回环检测](https://github.com/dorian3d/DBoW2)
- [camodocal 相机标定](https://github.com/hengli/camodocal)
