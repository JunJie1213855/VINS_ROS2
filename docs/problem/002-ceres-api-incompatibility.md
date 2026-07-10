# 编译问题 002: Ceres API 不兼容 — Manifold vs LocalParameterization

**时间:** 2026-06-30

**错误信息:**
```
error: 'Manifold' is not a member of 'ceres'
ceres::Manifold* quaternion_manifold = new ceres::QuaternionManifold();
```

**原因分析:**
系统安装的是 Ceres 2.0.0（Ubuntu apt 包 `libceres-dev 2.0.0+dfsg1-5`），但 VINS-Fusion-ROS2 代码使用了 Ceres 2.2.0+ 的新 API：
- `ceres::Manifold` → Ceres 2.0 中为 `ceres::LocalParameterization`
- `ceres::QuaternionManifold` → Ceres 2.0 中为 `ceres::QuaternionParameterization`
- `ceres::AutoDiffManifold` → Ceres 2.0 中为 `ceres::AutoDiffLocalParameterization`

涉及文件：
1. `camera_models/include/camodocal/gpl/EigenQuaternionParameterization.h`
2. `loop_fusion/src/pose_graph.h` (AngleManifoldFunctor)
3. `loop_fusion/src/pose_graph.cpp`
4. `vins/src/initial/initial_sfm.cpp`
5. `global_fusion/src/globalOpt.cpp`
6. `vins/src/factor/pose_local_parameterization.h`
7. `vins/src/estimator/estimator.cpp`

**解决方案:**
将所有 `Manifold` API 调用降级为 Ceres 2.0 兼容的 `LocalParameterization` API，同时修改派生类的虚函数签名以匹配 `LocalParameterization` 接口。
