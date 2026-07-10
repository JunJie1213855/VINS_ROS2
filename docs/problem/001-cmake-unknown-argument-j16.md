# 编译问题 001: CMake 不认识 -j16 参数

**时间:** 2026-06-30

**错误信息:**
```
CMake Error: Unknown argument -j16
CMake Error: Run 'cmake --help' for all supported options.
```

**原因分析:**
`--cmake-args -j16` 被直接传给了 CMake，但 CMake 不接受 `-j16` 参数。`-j16` 是 make 的参数，不是 cmake 的。在 colcon 中，包级别的并行已经由 `--parallel-workers` 控制，而编译层面的并行需要通过环境变量 `MAKEFLAGS` 来控制。

**解决方案:**
移除 `--cmake-args -j16`，改用环境变量 `MAKEFLAGS="-j16"` 来控制 make 层面的并行编译。
