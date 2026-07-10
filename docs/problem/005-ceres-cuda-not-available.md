# 编译问题 005: ceres::CUDA 在 Ceres 2.0 中不可用

**时间:** 2026-06-30

**错误信息:**
```
error: 'CUDA' is not a member of 'ceres'
```

**原因分析:**
`ceres::CUDA` 是 Ceres 2.1+ 才引入的，Ceres 2.0 不支持。代码中当 `USE_GPU_CERES` 为 true 时尝试使用 CUDA 加速。

**解决方案:**
添加预处理器版本检查，仅在 Ceres >= 2.1 时使用 `ceres::CUDA`，否则回退到 `ceres::DENSE_SCHUR`。
