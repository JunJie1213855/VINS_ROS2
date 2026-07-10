# 编译问题 006: vins/loop_fusion 安装到源码目录而非工作空间 install/

**时间:** 2026-06-30

**错误现象:**
`source install/local_setup.sh` 后 `ros2 pkg list` 找不到 vins 和 loop_fusion。

**原因分析:**
源码树中存在过期的 `src/VINS-Fusion-ROS2/build/` 目录，内含 CMakeCache.txt 将 `CMAKE_INSTALL_PREFIX` 指向 `src/VINS-Fusion-ROS2/install/` 而非工作空间的 `install/`。colcon 检测到已有 CMake 缓存后复用了错误的安装路径。

**解决方案:**
1. 删除源码内的 `build/` 和 `install/` 目录
2. 使用显式路径编译:
```bash
colcon build --parallel-workers 16 \
  --build-base /home/ros/rosws/vins_fusion_ws/build \
  --install-base /home/ros/rosws/vins_fusion_ws/install
```
