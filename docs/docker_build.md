# Docker 编译、运行与可视化指南

本文档介绍如何使用已有的 `lio_nav2:humble-dev` Docker 镜像编译和运行 VINS-Fusion-ROS2。

---

## 1. 环境说明

| 组件 | 说明 |
|------|------|
| Docker 镜像 | `lio_nav2:humble-dev`（基于 `ros:humble-desktop-full`） |
| ROS2 发行版 | Humble Hawksbill |
| 宿主机工作空间 | `/home/ros/ros_ws/VINS_ROS2` |
| 数据集路径 | `/home/ros/dataset/Euroc/` |

---

## 2. 编译

### 2.1 启动编译容器

```bash
docker run -d --name vins_build \
  -v /home/ros/ros_ws/VINS_ROS2:/ws/VINS_ROS2 \
  lio_nav2:humble-dev sleep infinity
```

### 2.2 单进程四线程编译

> **重要**：使用 `--parallel-workers 1` + `MAKEFLAGS="-j4"` 限制资源占用，防止浏览器卡死。

```bash
docker exec vins_build bash -c "
  export MAKEFLAGS='-j4'
  cd /ws/VINS_ROS2
  source /opt/ros/humble/setup.bash
  colcon build --parallel-workers 1 --executor sequential
"
```

| 包名 | 大致耗时 |
|------|----------|
| `camera_models` | ~70s |
| `global_fusion` | ~20s |
| `loop_fusion` | ~30s |
| `vins` | ~90s |
| **总计** | **~3.5 min** |

### 2.3 编译完成后清理

```bash
docker stop vins_build && docker rm vins_build
```

---

## 3. 运行 EuRoC 数据集

### 3.1 启动运行容器

```bash
docker run -d --name vins_run \
  -v /home/ros/ros_ws/VINS_ROS2:/ws/VINS_ROS2 \
  -v /home/ros/dataset/Euroc:/ws/dataset \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e DISPLAY=$DISPLAY \
  lio_nav2:humble-dev sleep infinity
```

挂载说明：

| 挂载 | 容器内路径 | 用途 |
|------|-----------|------|
| `VINS_ROS2` | `/ws/VINS_ROS2` | 已编译的 VINS 工作空间 |
| `dataset/Euroc` | `/ws/dataset` | EuRoC 数据集 |
| `/tmp/.X11-unix` | `/tmp/.X11-unix` | X11 转发（RViz 可视化） |

### 3.2 创建输出目录

```bash
docker exec vins_run mkdir -p /home/ros/vins_output/pose_graph
```

### 3.3 终端 1 — 启动 VINS 节点

```bash
docker exec -it vins_run bash -c "
  source /opt/ros/humble/setup.bash
  source /ws/VINS_ROS2/install/setup.bash
  ros2 launch vins euroc.launch.py
"
```

> 配置文件路径 **必须作为位置参数** 传入，不可使用 `--ros-args -p` 方式。
>
> VINS 启动后会输出 `waiting for image and imu...`，等待数据发布。

### 3.4 终端 2 — 播放数据集

```bash
docker exec -it vins_run bash -c "
  source /opt/ros/humble/setup.bash
  python3 /ws/VINS_ROS2/src/VINS_ROS2/scripts/euroc_publisher.py /ws/dataset/vicon_room2/V2_01_easy/mav0
"
```

可选参数：

| 参数 | 说明 | 示例 |
|------|------|------|
| `--speed 2.0` | 2 倍速播放 | `--speed 2.0` |
| `--start 10.0` | 从第 10 秒开始 | `--start 10.0` |

> **启动顺序**：必须先启动 VINS 节点，再启动数据集发布器。

### 3.5 验证运行状态

```bash
# 查看话题
docker exec vins_run bash -c "
  source /opt/ros/humble/setup.bash
  ros2 topic list | grep -E 'cam0|cam1|imu0|vins_estimator'
"

# 查看 IMU 频率
docker exec vins_run bash -c "
  source /opt/ros/humble/setup.bash
  ros2 topic hz /imu0
"
```

---

## 4. RViz 可视化

### 4.1 宿主机授权 X11

```bash
xhost +local:docker
```

> 该命令只需执行一次；若重启系统后需重新执行。

### 4.2 启动 RViz

```bash
docker exec -it vins_run bash -c "
  source /opt/ros/humble/setup.bash
  rviz2 -d /ws/VINS_ROS2/src/VINS_ROS2/config/vins_rviz_config.rviz
"
```

### 4.3 RViz 预设可视化内容

| 显示项 | 话题 | 说明 |
|--------|------|------|
| 轨迹线（绿色） | `/vins_estimator/path` | VIO 估计轨迹 |
| 当前点云（绿色） | `/vins_estimator/point_cloud` | 当前帧稀疏点云 |
| 边缘化点云（白色） | `/vins_estimator/margin_cloud` | 被边缘化的地图点 |
| 关键帧点云（黄色） | `/vins_estimator/keyframe_point` | 关键帧地图点 |
| 相机位姿 | `/vins_estimator/camera_pose_visual` | 相机位姿 MarkerArray |
| 特征追踪图 | `/vins_estimator/image_track` | 光流追踪可视化 |
| TF | — | world → body 坐标变换 |

配置中 **Fixed Frame** 为 `world`，**背景色** 为黑色。

### 4.4 如果 RViz 无法连接 X11

```bash
# 检查宿主机 X 服务器
echo $DISPLAY
ps aux | grep Xorg

# 重新授权
xhost +local:docker

# 检查容器内的 DISPLAY
docker exec vins_run bash -c 'echo $DISPLAY'
```

---

## 5. 运行完成后清理

```bash
docker stop vins_run && docker rm vins_run
```

---

## 6. 完整运行脚本

将以下内容保存为 `run_vins_euroc.sh`：

```bash
#!/bin/bash
set -e

DATASET_PATH="/ws/dataset/vicon_room2/V2_01_easy/mav0"
CONFIG_PATH="/ws/VINS_ROS2/src/VINS_ROS2/config/euroc/euroc_stereo_imu_config.yaml"
RVIZ_CONFIG="/ws/VINS_ROS2/src/VINS_ROS2/config/vins_rviz_config.rviz"

# 清理旧容器
docker rm -f vins_run 2>/dev/null || true

# 启动容器
docker run -d --name vins_run \
  -v /home/ros/ros_ws/VINS_ROS2:/ws/VINS_ROS2 \
  -v /home/ros/dataset/Euroc:/ws/dataset \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e DISPLAY=$DISPLAY \
  lio_nav2:humble-dev sleep infinity

# 创建输出目录
docker exec vins_run mkdir -p /home/ros/vins_output/pose_graph

# 启动 VINS 节点
docker exec -d vins_run bash -c "
  source /opt/ros/humble/setup.bash
  source /ws/VINS_ROS2/install/setup.bash
  ros2 run vins vins_node $CONFIG_PATH
"

echo "VINS node started. Waiting 3s for initialization..."
sleep 3

# 启动数据集发布器
docker exec -d vins_run bash -c "
  source /opt/ros/humble/setup.bash
  python3 /ws/VINS_ROS2/src/VINS_ROS2/scripts/euroc_publisher.py $DATASET_PATH
"

echo "Dataset publisher started."

# 启动 RViz
xhost +local:docker 2>/dev/null || true
docker exec -it vins_run bash -c "
  source /opt/ros/humble/setup.bash
  rviz2 -d $RVIZ_CONFIG
"

# RViz 关闭后清理
echo "Shutting down..."
docker stop vins_run && docker rm vins_run
echo "Done."
```

用法：

```bash
chmod +x run_vins_euroc.sh
./run_vins_euroc.sh
```

---

## 7. 不同 EuRoC 序列

修改 `DATASET_PATH` 即可切换序列：

| 序列 | 数据集路径 |
|------|-----------|
| V1_01_easy | `/ws/dataset/vicon_room1/V1_01_easy/mav0` |
| V1_02_medium | `/ws/dataset/vicon_room1/V1_02_medium/mav0` |
| V1_03_difficult | `/ws/dataset/vicon_room1/V1_03_difficult/mav0` |
| V2_01_easy | `/ws/dataset/vicon_room2/V2_01_easy/mav0` |
| V2_02_medium | `/ws/dataset/vicon_room2/V2_02_medium/mav0` |
| MH_01_easy | `/ws/dataset/machine_hall/MH_01_easy/mav0` |

---

## 8. 常见问题

| 现象 | 原因 | 解决 |
|------|------|------|
| `please intput: ros2 run vins vins_node [config file]` | 配置路径未作为位置参数传入 | 使用 `ros2 run vins vins_node /path/to/config.yaml`（不要用 `--ros-args`） |
| `waiting for image and imu...` | 数据集发布器未启动 | 先确认 `euroc_publisher.py` 在运行 |
| RViz 无法启动，`could not connect to display` | X11 授权未放行 | 宿主机执行 `xhost +local:docker` |
| VINS 初始化失败 | 数据起始段无足够运动 | 加 `--start 5.0` 跳过静止段 |
| 编译卡死/内存不足 | 并行线程过多 | 严格使用 `--parallel-workers 1` + `MAKEFLAGS="-j4"` |
| `ros2: command not found` | 未 source ROS2 环境 | 先执行 `source /opt/ros/humble/setup.bash` |
