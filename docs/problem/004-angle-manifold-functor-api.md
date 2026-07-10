# 编译问题 004: AngleManifoldFunctor 与 AutoDiffLocalParameterization API 不匹配

**时间:** 2026-06-30

**错误信息:**
```
error: no match for call to '(const AngleManifoldFunctor) (const double*&, const double*&, double*&)'
```

**原因分析:**
Ceres 2.0 的 `AutoDiffLocalParameterization` 期望 functor 有 `operator()(const T* x, const T* delta, T* x_plus_delta)` 方法，但 `AngleManifoldFunctor` 使用的是 Ceres 2.2+ 的 `Plus`/`Minus` 命名方法。这两套 API 不兼容。

**解决方案:**
将 `AngleManifoldFunctor::Plus` 重命名为 `operator()`，并移除 `Minus`（`LocalParameterization` 不需要）。
