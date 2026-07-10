# 编译总结

**时间:** 2026-06-30
**结果:** ✅ 全部编译成功 (4/4 包)

## 编译包状态

| 包名 | 状态 | 备注 |
|------|------|------|
| camera_models | ✅ 成功 | 有 deprecation warnings |
| global_fusion | ✅ 成功 | 有 VLA 和 deprecation warnings |
| loop_fusion | ✅ 成功 | 有 deprecation warnings |
| vins | ✅ 成功 | 有 VLA 和 deprecation warnings |

## 修复的问题

1. **001**: `--cmake-args -j16` → 改用 `MAKEFLAGS="-j16"`
2. **002**: Ceres Manifold API → LocalParameterization API (8 个文件)
3. **003**: 安装 `ros-humble-image-transport`
4. **004**: AngleManifoldFunctor::Plus → operator()
5. **005**: ceres::CUDA → 预处理器版本检查回退到 DENSE_SCHUR

## 编译过程中剩余的 warnings（非阻塞）
- 多处在使用 deprecated `sensor_msgs::msg::XXX::ConstPtr`（Humble 中建议使用 `SharedPtr`）
- 多处 ISO C++ 禁止的 VLA (variable-length array)
- 少量 `-Wignored-qualifiers`, `-Wdeprecated-copy`, `-Wsign-compare`
