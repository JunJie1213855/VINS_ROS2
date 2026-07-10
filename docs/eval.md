# VINS-Fusion 轨迹精度评估指南

本文档介绍如何使用 [evo](https://github.com/MichaelGrupp/evo) 工具评估 VINS-Fusion 在 EuRoC 数据集上的轨迹精度。

---

## 1. 安装 evo

```bash
pip install evo --upgrade
```

验证安装：

```bash
evo --version
```

---

## 2. 数据格式

### 2.1 VINS 输出 (`vio.csv`)

VINS 自动保存的轨迹文件，路径由配置文件 `output_path` 指定：

```
/home/ros/vins_output/vio.csv
```

格式（无表头）：

```
timestamp, px, py, pz, qw, qx, qy, qz
1403715274,0.00008,-0.00015,-0.00016,0.01318,0.82980,-0.00905,0.55784
...
```

| 列 | 含义 | 单位 |
|----|------|------|
| `timestamp` | 时间戳 | 秒 (float) |
| `px, py, pz` | 位置 (x, y, z) | 米 |
| `qw, qx, qy, qz` | 四元数 (w, x, y, z) | — |

### 2.2 EuRoC 真值 (`data.csv`)

```
~/dataset/Euroc/vicon_room1/V1_01_easy/mav0/state_groundtruth_estimate0/data.csv
```

格式（有表头）：

```csv
#timestamp, p_RS_R_x [m], p_RS_R_y [m], p_RS_R_z [m], q_RS_w [], q_RS_x [], q_RS_y [], q_RS_z [], ...
1403715274302142976,0.878612,2.142470,0.947262,0.060514,-0.828459,-0.058956,-0.553641,...
```

| 列 | 含义 | 单位 |
|----|------|------|
| `#timestamp` | 时间戳 | 纳秒 (int) |
| `p_RS_R_x/y/z` | 位置 | 米 |
| `q_RS_w/x/y/z` | 四元数 | — |

---

## 3. 格式转换

evo 的 `evo_ape` 支持 EuRoC 真值格式，但 VINS 输出需转为 **TUM 格式**（`timestamp tx ty tz qx qy qz qw`）。

### 转换脚本

```python
#!/usr/bin/env python3
"""Convert VINS vio.csv to TUM format for evo evaluation."""
import sys

def vins_to_tum(input_path: str, output_path: str):
    with open(input_path) as fin, open(output_path, 'w') as fout:
        for line in fin:
            line = line.strip()
            if not line:
                continue
            parts = line.split(',')
            if len(parts) < 8:
                continue
            ts, px, py, pz = parts[0], parts[1], parts[2], parts[3]
            qw, qx, qy, qz = parts[4], parts[5], parts[6], parts[7]
            # TUM: timestamp tx ty tz qx qy qz qw
            fout.write(f"{ts} {px} {py} {pz} {qx} {qy} {qz} {qw}\n")
    print(f"Converted {input_path} -> {output_path}")

if __name__ == '__main__':
    if len(sys.argv) < 3:
        print(f"Usage: {sys.argv[0]} <vio.csv> <output.tum>")
        sys.exit(1)
    vins_to_tum(sys.argv[1], sys.argv[2])
```

使用：

```bash
python3 scripts/vins2tum.py /home/ros/vins_output/vio.csv /home/ros/vins_output/vio.tum
```

---

## 4. 精度评估

### 4.1 绝对轨迹误差 (ATE)

```bash
evo_ape euroc \
  ~/dataset/Euroc/vicon_room1/V1_01_easy/mav0/state_groundtruth_estimate0/data.csv \
  /home/ros/vins_output/vio.tum \
  --plot --plot_mode xy --save_results results/v1_01_easy.zip
```

| 参数 | 说明 |
|------|------|
| `evo_ape` | 绝对位姿误差 (Absolute Pose Error) |
| `euroc` | 真值为 EuRoC 格式（自动处理纳秒时间戳） |
| `--plot` | 显示轨迹对比图 |
| `--plot_mode xy` | 俯视图 |
| `--save_results` | 保存结果 zip，方便后续对比 |

### 4.2 相对位姿误差 (RPE)

```bash
evo_rpe euroc \
  ~/dataset/Euroc/vicon_room1/V1_01_easy/mav0/state_groundtruth_estimate0/data.csv \
  /home/ros/vins_output/vio.tum \
  --delta 1.0 --delta_unit m \
  --plot --plot_mode xy --save_results results/v1_01_easy_rpe.zip
```

| 参数 | 说明 |
|------|------|
| `--delta 1.0` | 每 1 米的相对误差 |
| `--delta_unit m` | delta 单位为米（也可用 `f` 表示帧） |

### 4.3 常用选项

```bash
--align                        # SE3 对齐（默认开启）
--align --correct_scale        # SE3 对齐 + 尺度校正（单目必需）
--plot_mode xyz                # 3D 视图
--plot_mode xz                 # 侧视图
--save_plot results/plot.pdf   # 保存图片
--verbose                      # 打印每帧误差
```

### 4.4 多序列批量对比

```bash
evo_res results/*.zip --use_filenames --save_table results/table.csv
```

---

## 5. 一键评估脚本

```bash
#!/bin/bash
# eval_vins.sh

VINS_CSV="/home/ros/vins_output/vio.csv"
GT_CSV="$HOME/dataset/Euroc/vicon_room1/V1_01_easy/mav0/state_groundtruth_estimate0/data.csv"
OUT="/home/ros/vins_output/eval"

mkdir -p "$OUT"

# 转换
python3 /home/ros/rosws/vins_fusion_ws/scripts/vins2tum.py "$VINS_CSV" "$OUT/vio.tum"

# ATE
echo "=== ATE ==="
evo_ape euroc "$GT_CSV" "$OUT/vio.tum" \
  --plot --plot_mode xy --save_plot "$OUT/ate_xy.pdf" \
  --save_results "$OUT/ate.zip"

# RPE (每米)
echo "=== RPE ==="
evo_rpe euroc "$GT_CSV" "$OUT/vio.tum" \
  --delta 1.0 --delta_unit m \
  --plot --plot_mode xy --save_plot "$OUT/rpe_xy.pdf" \
  --save_results "$OUT/rpe.zip"

echo "Done → $OUT"
```

---

## 6. 结果解读

```
APE w.r.t. translation part (m)
(with SE(3) Umeyama alignment)

       max      1.212563
      mean      0.245867
    median      0.193065
       min      0.001234
      rmse      0.298442
       std      0.168973
```

| 指标 | 含义 | V1_01_easy 优秀值 |
|------|------|-------------------|
| `rmse` | 均方根误差 | < 0.2 m |
| `mean` | 平均误差 | < 0.2 m |
| `max` | 最大误差 | < 1.0 m |
| `std` | 标准差 | < 0.15 m |

### EuRoC 各序列预期 RMSE

| 序列 | 难度 | 预期 ATE RMSE |
|------|------|---------------|
| V1_01_easy | 简单 | 0.05 ~ 0.15 m |
| V1_02_medium | 中等 | 0.05 ~ 0.20 m |
| V1_03_difficult | 困难 | 0.10 ~ 0.30 m |
| V2_01_easy | 简单 | 0.05 ~ 0.15 m |
| MH_01_easy | 简单 | 0.10 ~ 0.25 m |
| MH_03_medium | 中等 | 0.15 ~ 0.35 m |

---

## 7. 常见问题

| 问题 | 解决 |
|------|------|
| `Unsupported file format` | 真值用 `euroc` 子命令指定格式 |
| 轨迹长度不一致 | evo 自动按时间戳同步，`--t_max_diff` 调容差 |
| 尺度漂移明显 | 单目需 `--correct_scale`，双目通常不需要 |
| 时间戳单位错误 | VINS 秒，真值纳秒，evo `euroc` 自动处理 |
| VINS 位姿频率很低 | 正常，VIO 只输出关键帧（~10-20Hz） |
