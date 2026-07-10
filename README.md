# VINS-Fusion

## ROS2 version of VINS-Fusion.

> 基于优化的多传感器融合 SLAM — ROS2 Humble 适配版

VINS-Fusion 由香港科技大学空中机器人组（Aerial Robotics Group, HKUST）开发，本项目是其 ROS2 移植版本，支持**视觉-惯性里程计 (VIO)**、**回环检测** 和 **GPS 全局融合**。

---

### Notices
- code has been updated so that the vins package can be executed via ros2 run or ros2 launch
- but Rviz config cannot be saved due to some issue.. still fixing
- GPU enable/disable features also have been added: refer [EuRoC config](https://github.com/zinuok/VINS-Fusion-ROS2/blob/main/config/euroc/euroc_stereo_imu_config.yaml#L19-L21) (refered from [here](https://github.com/pjrambo/VINS-Fusion-gpu) and [here](https://github.com/pjrambo/VINS-Fusion-gpu/issues/33#issuecomment-1097642597))
  - The GPU version has some CUDA library dependencies: OpenCV with CUDA. For CPU only, comment the following macro at line 14 in `vins/src/featureTracker/feature_tracker.h`:
  ```cpp
  // #define GPU_MODE 1
  ```
  > 当前系统 Ceres 2.0 不支持 `ceres::CUDA`，已通过预处理器自动禁用。`use_gpu_ceres` 会被自动忽略。

---

### Prerequisites

- **System**
  - Ubuntu 22.04
  - ROS2 Humble
- **Libraries**
  - OpenCV ≥ 4.5 (Humble 自带)
  - [Ceres Solver](http://ceres-solver.org/) 2.0.0 (apt 包)
  - [Eigen3](https://eigen.tuxfamily.org/) ≥ 3.3

```bash
# 安装 ROS2 依赖
sudo apt install -y ros-humble-image-transport ros-humble-rviz2
```

---

### 当前编译环境

| 组件 | 版本 |
|------|------|
| Ubuntu | 22.04.5 LTS |
| Kernel | 6.8.0-124-generic |
| ROS2 | Humble Hawksbill |
| GCC | 11.4.0 |
| G++ | 11.4.0 |
| CMake | 4.4.0 |
| GNU Make | 4.3 |
| Python | 3.10.20 |
| Ceres Solver | 2.0.0 (apt) |
| Eigen3 | 3.4.0 |
| OpenCV | 5.0.0 |
| CPU | 24 cores |
| ROS2 已安装包 | 318 |

---

### Sensor Setup

- camera: Intel Realsense D435i
- 使用以下脚本安装 Realsense SDK + ROS2 包：

```bash
chmod +x realsense_install.sh
bash realsense_install.sh
```

---

### Build

```bash
cd /home/ros/rosws/vins_fusion_ws

# 编译（16 线程）
MAKEFLAGS="-j16" colcon build --parallel-workers 16 \
  --build-base /home/ros/rosws/vins_fusion_ws/build \
  --install-base /home/ros/rosws/vins_fusion_ws/install
```

仅编译特定包：

```bash
colcon build --packages-select vins --parallel-workers 16
```

清理重建：

```bash
rm -rf build install log src/build src/install
MAKEFLAGS="-j16" colcon build --parallel-workers 16 \
  --build-base build --install-base install
```

---

### 环境准备

```bash
source /opt/ros/humble/setup.sh
source install/local_setup.sh
```

---

### Run

```bash
# 启动 VINS（默认 EuRoC 双目+IMU 配置）
ros2 launch vins euroc.launch.py

# 自定义配置
ros2 launch vins euroc.launch.py config_path:=/path/to/config.yaml

# 或直接用 ros2 run
ros2 run vins vins_node /path/to/config.yaml

# RViz2 可视化
ros2 launch vins vins_rviz.launch.py
```

---

### 系统架构

```
vins_fusion_ws/
├── src/VINS-Fusion-ROS2/
│   ├── vins/              # VIO 核心 — 滑动窗口紧耦合优化
│   ├── loop_fusion/       # 回环检测 — DBoW2 + 位姿图优化
│   ├── global_fusion/     # GPS 全局融合
│   ├── camera_models/     # 相机标定模型库 (camodocal)
│   └── config/            # 8 种传感器/数据集配置
├── scripts/
│   ├── euroc_publisher.py # EuRoC 数据集 ROS2 发布器
│   └── vins2tum.py        # VINS 轨迹转 TUM 格式
├── docs/
│   ├── introduction.md    # 项目详细文档
│   ├── euroc_vins_pub.md  # EuRoC 数据发布指南
│   ├── eval.md            # 精度评估指南
│   └── problem/           # 编译问题记录
├── build/
└── install/
```

#### 包清单

| 包 | 类型 | 可执行文件 | 功能 |
|----|------|-----------|------|
| `camera_models` | 库 | — | 5 种相机畸变模型 |
| `vins` | 可执行 | `vins_node` | 视觉-惯性里程计 |
| `loop_fusion` | 可执行 | `loop_fusion_node` | 回环检测与重定位 |
| `global_fusion` | 可执行 | `global_fusion_node` | GPS 全局融合 |

---

### 支持的传感器配置

| 模式 | IMU | 相机 | 配置文件 |
|------|-----|------|----------|
| 双目+IMU | ✓ | 2 | `config/euroc/euroc_stereo_imu_config.yaml` |
| 单目+IMU | ✓ | 1 | `config/euroc/euroc_mono_imu_config.yaml` |
| 双目 (无IMU) | ✗ | 2 | `config/kitti_odom/` |
| GPS融合 | ✓ | 1-2 | `config/kitti_raw/` |

#### 预置配置

| 目录 | 传感器 |
|------|--------|
| `euroc/` | EuRoC MAV 数据集 |
| `realsense_d435i/` | Intel RealSense D435i |
| `mynteye/` | 小觅 MyntEye |
| `A3_ptgrey/` | DJI A3 + PointGrey |
| `vi_car/` | 车载视觉-惯性 |
| `kitti_odom/` | KITTI Odometry |
| `kitti_raw/` | KITTI Raw |
| `simulation/` | 仿真测试 |

---

### EuRoC 数据集评估

```bash
# 终端 1：启动 VINS
source install/local_setup.sh
ros2 launch vins euroc.launch.py

# 终端 2：启动 RViz
ros2 launch vins vins_rviz.launch.py

# 终端 3：发布 EuRoC 数据（1.1× 速度）
python3 scripts/euroc_publisher.py \
  ~/dataset/Euroc/vicon_room1/V1_01_easy/mav0 --speed 1.1
```

运行结束后轨迹自动保存到 `output_path` 指定的目录（默认 `/home/ros/vins_output/vio.csv`）。

#### 评估精度

```bash
# 格式转换
python3 scripts/vins2tum.py /home/ros/vins_output/vio.csv /home/ros/vins_output/vio.tum

# ATE 评估
evo_ape euroc \
  ~/dataset/Euroc/vicon_room1/V1_01_easy/mav0/state_groundtruth_estimate0/data.csv \
  /home/ros/vins_output/vio.tum \
  --plot --plot_mode xy \
  --align
```

#### 精度参考

| 序列 | 难度 | 预期 ATE RMSE |
|------|------|---------------|
| V1_01_easy | 简单 | 0.05 ~ 0.15 m |
| V1_02_medium | 中等 | 0.05 ~ 0.20 m |
| V1_03_difficult | 困难 | 0.10 ~ 0.30 m |
| MH_01_easy | 简单 | 0.10 ~ 0.25 m |

---

### 话题接口

#### 订阅

| 话题 | 类型 | 说明 |
|------|------|------|
| `/imu0` | `sensor_msgs/Imu` | IMU 数据 |
| `/cam0/image_raw` | `sensor_msgs/Image` | 左目图像 |
| `/cam1/image_raw` | `sensor_msgs/Image` | 右目图像（双目模式） |

#### 发布

| 话题 | 类型 | 说明 |
|------|------|------|
| `/vins_estimator/odometry` | `nav_msgs/Odometry` | VIO 里程计 |
| `/vins_estimator/path` | `nav_msgs/Path` | 运动轨迹 |
| `/vins_estimator/point_cloud` | `sensor_msgs/PointCloud` | 稀疏点云 |
| `/vins_estimator/keyframe_pose` | `geometry_msgs/PoseWithCovarianceStamped` | 关键帧位姿 |
| `/vins_estimator/image_track` | `sensor_msgs/Image` | 特征追踪可视化 |

---

### 关键配置参数

```yaml
# config/euroc/euroc_stereo_imu_config.yaml

# 传感器
imu: 1                     # 是否使用 IMU
num_of_cam: 2              # 相机数量
imu_topic: "/imu0"
image0_topic: "/cam0/image_raw"
image1_topic: "/cam1/image_raw"

# 输出路径（必须为绝对路径，~ 不会被展开）
output_path: "/home/ros/vins_output/"

# 外参
estimate_extrinsic: 0      # 0=信任标定, 1=在线优化
body_T_cam0: !!opencv-matrix   # IMU → 左相机 (4×4)
body_T_cam1: !!opencv-matrix   # IMU → 右相机 (4×4)

# 特征追踪
max_cnt: 150               # 最大特征点数
freq: 10                   # 发布频率 (Hz)
flow_back: 1               # 双向光流验证

# 优化
max_solver_time: 0.04      # 最大求解时间 (s)
max_num_iterations: 8      # 最大迭代次数

# IMU 噪声
acc_n: 0.1                 # 加速度计噪声 (m/s²/√Hz)
gyr_n: 0.01                # 陀螺仪噪声 (rad/s/√Hz)
g_norm: 9.81007            # 重力加速度 (m/s²)

# GPU（当前系统全部设为 0）
use_gpu         : 0        # GPU 光流
use_gpu_acc_flow: 0        # GPU 加速光流
use_gpu_ceres   : 0        # Ceres CUDA（需 ≥ 2.1）
```

---

### 坐标系说明

VINS 以**第一帧 IMU 位姿为世界原点** `(0,0,0)`，所有估计位姿都在这个相对坐标系下。EuRoC 真值在 Vicon 房间坐标系下。评测时 evo 的 `--align` 会自动做 SE(3) 刚性对齐——这是所有 VIO/SLAM 评测的标准做法。

---

### 编译兼容性

本项目原针对 Ubuntu 20.04 + ROS2 Foxy + Ceres 2.1。当前环境（Ubuntu 22.04 + ROS2 Humble + Ceres 2.0）编译时做了以下适配（详情见 `docs/problem/`）：

| 修改 | 涉及文件 |
|------|----------|
| `ceres::Manifold` → `LocalParameterization` | 8 个 `.h/.cpp/.cc` |
| `ceres::CUDA` → `#if` 版本检查回退 | `estimator.cpp` |
| `AutoDiffManifold` → `AutoDiffLocalParameterization` | `pose_graph.h` |
| `SetManifold` → `SetParameterization` | `CameraCalibration.cc` |
| 输出目录自动创建 (`mkdir -p`) | `parameters.cpp` |
| `CMAKE_INSTALL_PREFIX` 过期缓存 | 显式指定 `--build-base` / `--install-base` |

---

### Play bag recorded at ROS1

ROS1 的 bag 文件不能直接在 ROS2 播放，需安装 [rosbags](https://gitlab.com/ternaris/rosbags) 转换：

```bash
pip install rosbags
export PATH=$PATH:~/.local/bin
rosbags-convert foo.bag --dst /path/to/bar
```

---

### 文档索引

| 文档 | 内容 |
|------|------|
| [introduction.md](docs/introduction.md) | 项目详细文档 — 架构、依赖、配置、API、调试 |
| [euroc_vins_pub.md](docs/euroc_vins_pub.md) | EuRoC 数据集 ROS2 发布指南 |
| [eval.md](docs/eval.md) | 轨迹精度评估指南 |
| [problem/](docs/problem/) | 编译问题记录（7 个） |

---

## Original Readme:

## 8. Acknowledgements
We use [ceres solver](http://ceres-solver.org/) for non-linear optimization and [DBoW2](https://github.com/dorian3d/DBoW2) for loop detection, a generic [camera model](https://github.com/hengli/camodocal) and [GeographicLib](https://geographiclib.sourceforge.io/).

## 9. License
The source code is released under [GPLv3](http://www.gnu.org/licenses/) license.

We are still working on improving the code reliability. For any technical issues, please contact Tong Qin <qintonguavATgmail.com>.

For commercial inquiries, please contact Shaojie Shen <eeshaojieATust.hk>.
