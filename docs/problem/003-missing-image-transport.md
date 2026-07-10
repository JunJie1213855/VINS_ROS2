# 编译问题 003: 缺少 image_transport 依赖

**时间:** 2026-06-30

**错误信息:**
```
CMake Error at CMakeLists.txt:21 (find_package):
  By not providing "Findimage_transport.cmake" in CMAKE_MODULE_PATH this
  project has asked CMake to find a package configuration file provided by
  "image_transport", but CMake did not find one.
```

**原因分析:**
`vins` 包依赖 ROS2 的 `image_transport` 包，但系统未安装。

**解决方案:**
```bash
sudo apt install ros-humble-image-transport
```
