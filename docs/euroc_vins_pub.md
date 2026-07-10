# EuRoC 数据集发布指南

本文档介绍如何使用 `scripts/euroc_publisher.py` 将原始 EuRoC MAV 数据集发布为 ROS2 话题，配合 VINS-Fusion 完成视觉-惯性里程计。

---

## 1. EuRoC 数据集结构

EuRoC MAV 数据集下载后，每个序列目录结构如下（以 V1_01_easy 为例）：

```
~/dataset/Euroc/vicon_room1/V1_01_easy/
└── mav0/
    ├── body.yaml                         # 传感器安装位姿
    ├── cam0/
    │   ├── sensor.yaml                   # 左相机内参
    │   ├── data.csv                      # 时间戳 → 文件名映射
    │   └── data/
    │       ├── 1403715273262142976.png   # 灰度图像 (752×480)
    │       └── ...
    ├── cam1/
    │   ├── sensor.yaml                   # 右相机内参
    │   ├── data.csv
    │   └── data/
    │       └── ...
    ├── imu0/
    │   ├── sensor.yaml                   # IMU 参数
    │   └── data.csv                      # IMU 测量 (200 Hz)
    ├── vicon0/                           # Vicon 真值（可选）
    └── state_groundtruth_estimate0/      # 位姿真值（可选）
```

### CSV 格式

**cam0/data.csv** — 图像时间戳映射：
```csv
#timestamp [ns],filename
1403715273262142976,1403715273262142976.png
1403715273312143104,1403715273312143104.png
```

**imu0/data.csv** — IMU 测量值（200 Hz）：
```csv
#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]
1403715273262142976,-0.002094,0.017453,0.077492,9.087495,0.130755,-3.693838
```

> 字段依次为：时间戳(ns), 角速度 x/y/z (rad/s), 线加速度 x/y/z (m/s²)

---

## 2. 发布脚本

### 2.1 脚本路径

```
/home/ros/rosws/vins_fusion_ws/scripts/euroc_publisher.py
```

### 2.2 用法

```bash
# 实时播放（速度 = 1.0×）
python3 scripts/euroc_publisher.py /path/to/mav0

# 2 倍速
python3 scripts/euroc_publisher.py /path/to/mav0 --speed 2.0

# 从第 30 秒开始播放
python3 scripts/euroc_publisher.py /path/to/mav0 --speed 2.0 --start 30.0
```

### 2.3 参数说明

| 参数 | 简写 | 默认值 | 说明 |
|------|------|--------|------|
| `dataset_path` | (位置参数) | 必填 | mav0/ 目录路径 |
| `--speed` | `-s` | `1.0` | 播放速度倍率，`1.0`=实时，`2.0`=2倍速 |
| `--start` | `-t` | `0.0` | 从第几秒开始（跳过前面的数据） |

### 2.4 发布的 ROS2 话题

| 话题 | 类型 | 频率 | 说明 |
|------|------|------|------|
| `/cam0/image_raw` | `sensor_msgs/Image` | 20 Hz | 左目灰度图像 (752×480, mono8) |
| `/cam1/image_raw` | `sensor_msgs/Image` | 20 Hz | 右目灰度图像 (752×480, mono8) |
| `/imu0` | `sensor_msgs/Imu` | 200 Hz | IMU 角速度+加速度 |

### 2.5 工作原理

```
                    ┌─────────────────────────┐
EuRoC CSV/PNG ────> │  EuRoCPublisher Node    │
                    │                         │
  cam0/data.csv ───>│  camera thread (20Hz)   │──> /cam0/image_raw
  cam1/data.csv ───>│  双相机合并时间线排序    │──> /cam1/image_raw
                    │                         │
  imu0/data.csv  ──>│  IMU thread (200Hz)     │──> /imu0
                    │  按时间戳同步播放        │
                    └─────────────────────────┘
```

- 两个线程独立运行，分别按时间戳控制发布节奏
- 通过 `time.monotonic_ns()` 实现与系统时钟同步的回放
- QoS 使用 `BEST_EFFORT` + `KEEP_LAST` 模拟真实传感器行为

---

## 3. 完整运行流程

### 3.1 启动步骤

**终端 1 — 启动 VINS-Fusion：**

```bash
source /opt/ros/humble/setup.sh
source /home/ros/rosws/vins_fusion_ws/install/local_setup.sh
ros2 launch vins euroc.launch.py
```

**终端 2 — 启动 RViz 可视化：**

```bash
source /opt/ros/humble/setup.sh
source /home/ros/rosws/vins_fusion_ws/install/local_setup.sh
ros2 launch vins vins_rviz.launch.py
```

**终端 3 — 发布数据集：**

```bash
source /opt/ros/humble/setup.sh
source /home/ros/rosws/vins_fusion_ws/install/local_setup.sh

cd /home/ros/rosws/vins_fusion_ws

# 实测：1.1× 速度可补偿初始化延迟，体验最佳
python3 scripts/euroc_publisher.py \
  ~/dataset/Euroc/vicon_room1/V1_01_easy/mav0 \
  --speed 1.1
```

### 3.2 验证数据是否正常发布

```bash
# 检查是否有话题在发布
ros2 topic list | grep -E "cam0|cam1|imu0"

# 查看 IMU 消息频率
ros2 topic hz /imu0

# 查看图像消息频率
ros2 topic hz /cam0/image_raw
```

### 3.3 EuRoC 各序列参考

| 序列 | 时长 | 难度 | 轨迹特征 |
|------|------|------|----------|
| V1_01_easy | 144s | 简单 | 缓慢移动，良好光照 |
| V1_02_medium | 83s | 中等 | 较快速移动 |
| V1_03_difficult | 105s | 困难 | 快速运动，运动模糊 |
| V2_01_easy | 112s | 简单 | Vicon Room 2 |
| V2_02_medium | 115s | 中等 | 快速运动 |
| V2_03_difficult | 115s | 困难 | 快速+回环 |
| MH_01_easy | 182s | 简单 | Machine Hall 大厅 |
| MH_02_easy | 150s | 简单 | 大厅 |
| MH_03_medium | 132s | 中等 | 大厅 |
| MH_04_difficult | 99s | 困难 | 大厅，低纹理 |
| MH_05_difficult | 111s | 困难 | 大厅，低纹理 |

---

## 4. 配置文件对应关系

脚本发布的话题名与 `config/euroc/euroc_stereo_imu_config.yaml` 默认配置一致：

| 脚本发布 | YAML 配置字段 | 说明 |
|----------|--------------|------|
| `/imu0` | `imu_topic: "/imu0"` | IMU 话题 |
| `/cam0/image_raw` | `image0_topic: "/cam0/image_raw"` | 左相机话题 |
| `/cam1/image_raw` | `image1_topic: "/cam1/image_raw"` | 右相机话题 |

> 如果使用自定义话题名，需同步修改配置文件。

---

## 5. 常见问题

| 现象 | 原因 | 解决 |
|------|------|------|
| `ValueError: not enough values to unpack` | CSV 格式解析错误 | 确认使用最新版脚本 |
| `Failed to load cam0/data/xxx.png` | 图像文件缺失 | 检查 mav0 路径是否正确 |
| `[euroc_publisher]: No data found!` | 路径错误 | 传入 `mav0/` 而非上级目录 |
| VINS 初始化失败 | 前几秒无足够运动 | 加 `--start 5.0` 跳过静止段 |
| 图像和 IMU 时间戳不对齐 | 正常现象 | VINS 内部有时间偏移估计 |
| rviz 不显示 | 话题连接断开 | 确认 VINS 在数据发布前已启动 |

### 正确初始化技巧

VINS 需要足够的运动激励才能完成初始化。建议：
- 从运动开始段播放（如 5-10 秒后），避开完全静止的初始几秒
- 启动顺序：**先启动 VINS → 再发布数据**
- 等待终端出现 `init done` 或绿色轨迹表示初始化成功
