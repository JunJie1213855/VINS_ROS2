# IMU 预积分与残差优化推导

本文档详细推导 VINS-Fusion 中 IMU 预积分的中值积分、残差构建和雅可比求导过程。

相关源文件：

| 文件 | 内容 |
|------|------|
| `vins/src/factor/integration_base.h` | IMU 预积分（中值积分 + 误差状态传播） |
| `vins/src/factor/imu_factor.h` | IMU 残差的 Ceres CostFunction |
| `vins/src/estimator/estimator.cpp` | 后端优化（残差项组装） |

---

## 1. 符号约定

| 符号 | 含义 | 维度 |
|------|------|------|
| $\mathbf{P}_i$ | 第 i 帧 IMU 在世界系下的位置 | 3 |
| $\mathbf{Q}_i$ | 第 i 帧 IMU 在世界系下的姿态（四元数） | 4 |
| $\mathbf{V}_i$ | 第 i 帧 IMU 在世界系下的速度 | 3 |
| $\mathbf{B}_{ai}$ | 第 i 帧加速度计偏置 | 3 |
| $\mathbf{B}_{gi}$ | 第 i 帧陀螺仪偏置 | 3 |
| $\mathbf{g}$ | 重力加速度 | 3 |
| $\Delta t$ | IMU 预积分总时长 (`sum_dt`) | 1 |
| $\Delta\mathbf{p}_{ij}$ | 从 i 到 j 的位移预积分量 | 3 |
| $\Delta\mathbf{q}_{ij}$ | 从 i 到 j 的旋转预积分量 | 4 |
| $\Delta\mathbf{v}_{ij}$ | 从 i 到 j 的速度预积分量 | 3 |

状态定义（`estimator.h:120-124`）：

```cpp
Vector3d Ps[(WINDOW_SIZE + 1)];   // 位置
Matrix3d Rs[(WINDOW_SIZE + 1)];   // 姿态旋转矩阵
Vector3d Vs[(WINDOW_SIZE + 1)];   // 速度
Vector3d Bas[(WINDOW_SIZE + 1)];  // 加速度计偏置
Vector3d Bgs[(WINDOW_SIZE + 1)];  // 陀螺仪偏置
```

Ceres 参数数组（`estimator.h:156-158`）：

```cpp
double para_Pose[i][7]       = [x, y, z, qx, qy, qz, qw]
double para_SpeedBias[i][9]  = [vx, vy, vz, bax, bay, baz, bgx, bgy, bgz]
```

误差状态索引（`parameters.h`）：

```cpp
enum StateOrder { O_P = 0, O_R = 3, O_V = 6, O_BA = 9, O_BG = 12 };
// 误差状态: [δp(3), δθ(3), δv(3), δba(3), δbg(3)] — 共 15 维
```

噪声索引：

```cpp
enum NoiseOrder { O_AN = 0, O_GN = 3, O_AW = 6, O_GW = 9 };
// 噪声向量: [n_a(3), n_g(3), n_{ba}(3), n_{bg}(3)] — 共 12 维（7.2 节中扩展为 18 维）
```

---

## 2. IMU 运动模型

### 连续时间模型

$$
\begin{aligned}
\dot{\mathbf{P}} &= \mathbf{V} \\
\dot{\mathbf{V}} &= \mathbf{R}(\mathbf{a}_m - \mathbf{b}_a - \mathbf{n}_a) + \mathbf{g} \\
\dot{\mathbf{R}} &= \mathbf{R}[\boldsymbol{\omega}_m - \mathbf{b}_g - \mathbf{n}_g]_{\times}
\end{aligned}
$$

其中 $\mathbf{a}_m, \boldsymbol{\omega}_m$ 是 IMU 原始测量值，$\mathbf{n}_a, \mathbf{n}_g$ 是高斯白噪声，$\mathbf{b}_a, \mathbf{b}_g$ 是慢变偏置（随机游走）：

$$
\begin{aligned}
\dot{\mathbf{b}}_a &= \mathbf{n}_{ba}, \quad \dot{\mathbf{b}}_g = \mathbf{n}_{bg}
\end{aligned}
$$

### 离散时间预积分（两帧之间）

在两帧图像 $t_i$ 到 $t_j$ 之间，累积 IMU 测量得到**预积分观测**（独立于 $t_i$ 时刻的绝对状态）：

$$
\begin{aligned}
\Delta\mathbf{p}_{ij} &= \iint_{t\in[i,j]} \mathbf{R}_{i,t}(\mathbf{a}_m - \mathbf{b}_a - \mathbf{n}_a) dt^2 \\
\Delta\mathbf{v}_{ij} &= \int_{t\in[i,j]} \mathbf{R}_{i,t}(\mathbf{a}_m - \mathbf{b}_a - \mathbf{n}_a) dt \\
\Delta\mathbf{q}_{ij} &= \prod \text{Exp}\left((\boldsymbol{\omega}_m - \mathbf{b}_g - \mathbf{n}_g)\delta t\right)
\end{aligned}
$$

---

## 3. 中值积分（Mid-Point Integration）

### 3.1 单步积分公式

代码 `integration_base.h:63-137`。对每一对相邻的 IMU 测量 $(\mathbf{a}_0, \boldsymbol{\omega}_0)$ 和 $(\mathbf{a}_1, \boldsymbol{\omega}_1)$（间隔 $\delta t$），用**两端点的均值**进行积分：

```
     acc_0, gyr_0              acc_1, gyr_1
         │                          │
    ─────●──────────────────────────●─────→ t
         │           δt              │
         t_k                      t_{k+1}
         │                          │
         └── 中值：(gyr_0+gyr_1)/2 ─┘
         └── 中值：(acc_0+acc_1)/2 ─┘
```

#### 名义状态传播（`midPointIntegration` 第 72-80 行）

```cpp
// ① 起始加速度转到世界系（用当前 δq 去旋转）
Vector3d un_acc_0 = delta_q * (_acc_0 - linearized_ba);

// ② 角速度中值
Vector3d un_gyr = 0.5 * (_gyr_0 + _gyr_1) - linearized_bg;

// ③ 旋转更新: Δq_{k+1} = Δq_k ⊗ [1, ½·ω̄·δt]
result_delta_q = delta_q * Quaterniond(1, un_gyr(0)*dt/2, un_gyr(1)*dt/2, un_gyr(2)*dt/2);

// ④ 终止加速度转到世界系（用新 δq 去旋转）
Vector3d un_acc_1 = result_delta_q * (_acc_1 - linearized_ba);

// ⑤ 加速度中值
Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);

// ⑥ 位移和速度更新
result_delta_p = delta_p + delta_v * dt + 0.5 * un_acc * dt * dt;
result_delta_v = delta_v + un_acc * dt;
```

数学形式：

$$
\begin{aligned}
\bar{\boldsymbol{\omega}} &= \frac{1}{2}(\boldsymbol{\omega}_0 + \boldsymbol{\omega}_1) - \bar{\mathbf{b}}_g \\
\Delta\mathbf{q}_{k+1} &= \Delta\mathbf{q}_k \otimes \begin{bmatrix} 1 \\ \frac{1}{2}\bar{\boldsymbol{\omega}}\delta t \end{bmatrix} \\
\mathbf{a}_0' &= \Delta\mathbf{q}_k \otimes (\mathbf{a}_0 - \bar{\mathbf{b}}_a) \\
\mathbf{a}_1' &= \Delta\mathbf{q}_{k+1} \otimes (\mathbf{a}_1 - \bar{\mathbf{b}}_a) \\
\bar{\mathbf{a}} &= \frac{1}{2}(\mathbf{a}_0' + \mathbf{a}_1') \\
\Delta\mathbf{p}_{k+1} &= \Delta\mathbf{p}_k + \Delta\mathbf{v}_k\delta t + \frac{1}{2}\bar{\mathbf{a}}\delta t^2 \\
\Delta\mathbf{v}_{k+1} &= \Delta\mathbf{v}_k + \bar{\mathbf{a}}\delta t
\end{aligned}
$$

> 精度为 $O(\delta t^3)$，优于欧拉积分的 $O(\delta t^2)$，比四阶龙格库塔计算量小。

#### 中值的来源

参数 `_acc_0, _gyr_0` 和 `_acc_1, _gyr_1` 的含义：

| 参数 | 来源 |
|------|------|
| `_acc_0, _gyr_0` | 上一个积分步缓存的测量值（`propagate()` 第 164-165 行 `acc_0 = acc_1; gyr_0 = gyr_1`） |
| `_acc_1, _gyr_1` | 当前刚收到的 IMU 数据（`push_back()` → `propagate()`） |

两帧 IMU 数据之间取中值，利用了被积函数在短时间内的线性近似。

---

## 4. 误差状态传播

### 4.1 误差状态定义（15 维）

$$
\delta\mathbf{x} = \begin{bmatrix}
\delta\mathbf{p} \\ \delta\boldsymbol{\theta} \\ \delta\mathbf{v} \\ \delta\mathbf{b}_a \\ \delta\mathbf{b}_g
\end{bmatrix} \in \mathbb{R}^{15}
$$

其中 $\delta\boldsymbol{\theta}$ 是姿态误差的 Lie 代数表示（切空间）：

$$
\mathbf{R} = \hat{\mathbf{R}} \cdot \text{Exp}(\delta\boldsymbol{\theta}), \quad \text{Exp}(\boldsymbol{\phi}) \approx \mathbf{I} + [\boldsymbol{\phi}]_{\times}
$$

### 4.2 连续时间误差动力学

$$
\delta\dot{\mathbf{x}} = \mathbf{A}\delta\mathbf{x} + \mathbf{B}\mathbf{n}
$$

其中 $\mathbf{n} = [\mathbf{n}_a, \mathbf{n}_g, \mathbf{n}_{ba}, \mathbf{n}_{bg}]^T \in \mathbb{R}^{12}$。

### 4.3 离散时间转移矩阵 F（15×15）

代码 `midPointIntegration` 第 82-135 行。

设 $[\mathbf{w}]_{\times}$ 为中值角速度的反对称矩阵，$[\mathbf{a}_0]_{\times}, [\mathbf{a}_1]_{\times}$ 为两端点加速度的反对称矩阵：

```cpp
Vector3d w_x   = 0.5 * (_gyr_0 + _gyr_1) - linearized_bg;   // 中值角速度
Vector3d a_0_x = _acc_0 - linearized_ba;                       // 起始加速度
Vector3d a_1_x = _acc_1 - linearized_ba;                       // 终止加速度

Matrix3d R_w_x, R_a_0_x, R_a_1_x;  // 对应的反对称矩阵
```

$\mathbf{F}$ 矩阵结构：

$$
\mathbf{F} = \begin{bmatrix}
\mathbf{I} & \mathbf{F}_{01} & \mathbf{I}\delta t & \mathbf{F}_{03} & \mathbf{F}_{04} \\
\mathbf{0} & \mathbf{I} - [\mathbf{w}]_{\times}\delta t & \mathbf{0} & \mathbf{0} & -\mathbf{I}\delta t \\
\mathbf{0} & \mathbf{F}_{21} & \mathbf{I} & \mathbf{F}_{23} & \mathbf{F}_{24} \\
\mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{I} & \mathbf{0} \\
\mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{I}
\end{bmatrix}
$$

其中各非零块的物理含义：

| 块 | 偏导 | 物理含义 | 代码 |
|-----|------|----------|------|
| $\mathbf{F}_{01}$ (F(0,3)) | $\partial\delta\mathbf{p}_{k+1}/\partial\delta\boldsymbol{\theta}_k$ | 旋转误差如何导致位置误差 | `-¼Rk·[a₀]×·dt² - ¼Rk+1·[a₁]×(I-[w]×·dt)·dt²` |
| $\mathbf{F}_{03}$ (F(0,9)) | $\partial\delta\mathbf{p}_{k+1}/\partial\delta\mathbf{b}_a$ | accel bias 误差如何导致位置误差 | `-¼(Rk+Rk+1)·dt²` |
| $\mathbf{F}_{04}$ (F(0,12)) | $\partial\delta\mathbf{p}_{k+1}/\partial\delta\mathbf{b}_g$ | gyro bias 误差如何导致位置误差 | `-¼·Rk+1·[a₁]×·dt²·(-dt)` |
| $\mathbf{F}_{11}$ (F(3,3)) | $\partial\delta\boldsymbol{\theta}_{k+1}/\partial\delta\boldsymbol{\theta}_k$ | 旋转误差传播 | `I - [w]×·dt` |
| $\mathbf{F}_{14}$ (F(3,12)) | $\partial\delta\boldsymbol{\theta}_{k+1}/\partial\delta\mathbf{b}_g$ | gyro bias 误差如何导致旋转误差 | `-I·dt` |
| $\mathbf{F}_{21}$ (F(6,3)) | $\partial\delta\mathbf{v}_{k+1}/\partial\delta\boldsymbol{\theta}_k$ | 旋转误差如何导致速度误差 | `-½Rk·[a₀]×·dt - ½Rk+1·[a₁]×(I-[w]×·dt)·dt` |
| $\mathbf{F}_{23}$ (F(6,9)) | $\partial\delta\mathbf{v}_{k+1}/\partial\delta\mathbf{b}_a$ | accel bias 误差如何导致速度误差 | `-½(Rk+Rk+1)·dt` |
| $\mathbf{F}_{24}$ (F(6,12)) | $\partial\delta\mathbf{v}_{k+1}/\partial\delta\mathbf{b}_g$ | gyro bias 误差如何导致速度误差 | `-½·Rk+1·[a₁]×·dt·(-dt)` |

### 4.4 噪声雅可比 V（15×18）

噪声向量 $\mathbf{n} \in \mathbb{R}^{18}$（6 个噪声源 × 3 维 = 18 维）：

```
n = [n_a(k), n_g(k), n_a(k+1), n_g(k+1), n_{ba}, n_{bg}]^T
```

$\mathbf{V}$ 表示各噪声分量如何影响误差状态的一阶导数：

```cpp
// 用代码注释说明各块的物理含义
V(0,0):  ∂δp/∂n_a(k)     =  ¼·Rk·dt²
V(0,3):  ∂δp/∂n_g(k)     =  ¼·-Rk+1·[a₁]×·dt²·½·dt
V(0,6):  ∂δp/∂n_a(k+1)   =  ¼·Rk+1·dt²
V(0,9):  ∂δp/∂n_g(k+1)   =  ¼·-Rk+1·[a₁]×·dt²·½·dt
V(3,3):  ∂δθ/∂n_g(k)     =  ½·I·dt
V(3,9):  ∂δθ/∂n_g(k+1)   =  ½·I·dt
V(6,0):  ∂δv/∂n_a(k)     =  ½·Rk·dt
V(6,3):  ∂δv/∂n_g(k)     =  ½·-Rk+1·[a₁]×·dt·½·dt
V(6,6):  ∂δv/∂n_a(k+1)   =  ½·Rk+1·dt
V(6,9):  ∂δv/∂n_g(k+1)   =  ½·-Rk+1·[a₁]×·dt·½·dt
V(9,12): ∂δba/∂n_ba      =  I·dt
V(12,15): ∂δbg/∂n_bg     =  I·dt
```

### 4.5 噪声协方差矩阵 N（18×18）

在构造函数 `IntegrationBase()` 中初始化（第 30-36 行）：

```cpp
noise.block<3,3>(0,0)   = (ACC_N²)  * I₃;  // n_a(k)    的方差
noise.block<3,3>(3,3)   = (GYR_N²)  * I₃;  // n_g(k)    的方差
noise.block<3,3>(6,6)   = (ACC_N²)  * I₃;  // n_a(k+1)  的方差
noise.block<3,3>(9,9)   = (GYR_N²)  * I₃;  // n_g(k+1)  的方差
noise.block<3,3>(12,12) = (ACC_W²)  * I₃;  // n_ba       的方差
noise.block<3,3>(15,15) = (GYR_W²)  * I₃;  // n_bg       的方差
```

### 4.6 协方差递推

逐帧传播（`midPointIntegration` 第 133-134 行）：

```cpp
jacobian   = F * jacobian;                                       // 误差状态转移
covariance = F * covariance * F.transpose() + V * noise * V.transpose();  // 协方差传播
```

这是标准卡尔曼滤波中协方差预测的形式：

$$
\begin{aligned}
\mathbf{J}_{k+1} &= \mathbf{F}_k \cdot \mathbf{J}_k \\[6pt]
\boldsymbol{\Sigma}_{k+1} &= \mathbf{F}_k \boldsymbol{\Sigma}_k \mathbf{F}_k^T + \mathbf{V}_k \mathbf{N} \mathbf{V}_k^T
\end{aligned}
$$

`jacobian` 矩阵（15×15）最终累积了从第一个 IMU 测量到最后一个测量之间的**全部误差状态转移**。其子块：

```cpp
dp_dba = jacobian.block<3,3>(O_P, O_BA);  // ∂Δp/∂ba  — 用于 bias 一阶修正
dp_dbg = jacobian.block<3,3>(O_P, O_BG);  // ∂Δp/∂bg
dq_dbg = jacobian.block<3,3>(O_R, O_BG);  // ∂Δq/∂bg
dv_dba = jacobian.block<3,3>(O_V, O_BA);  // ∂Δv/∂ba
dv_dbg = jacobian.block<3,3>(O_V, O_BG);  // ∂Δv/∂bg
```

---

## 5. 预积分量的一阶 Bias 修正

预积分时使用固定的线性化点 $\bar{\mathbf{b}}_a, \bar{\mathbf{b}}_g$。优化过程中 bias 变化后，用一阶泰勒展开修正（`evaluate()` 第 182-187 行）：

```cpp
Eigen::Vector3d dba = Bai - linearized_ba;
Eigen::Vector3d dbg = Bgi - linearized_bg;

Eigen::Quaterniond corrected_delta_q = delta_q * Utility::deltaQ(dq_dbg * dbg);
Eigen::Vector3d corrected_delta_v   = delta_v + dv_dba * dba + dv_dbg * dbg;
Eigen::Vector3d corrected_delta_p   = delta_p + dp_dba * dba + dp_dbg * dbg;
```

数学形式：

$$
\begin{aligned}
\Delta\tilde{\mathbf{p}}_{ij} &= \Delta\mathbf{p}_{ij} + \frac{\partial \Delta\mathbf{p}_{ij}}{\partial \mathbf{b}_a}\delta\mathbf{b}_a + \frac{\partial \Delta\mathbf{p}_{ij}}{\partial \mathbf{b}_g}\delta\mathbf{b}_g \\[6pt]
\Delta\tilde{\mathbf{v}}_{ij} &= \Delta\mathbf{v}_{ij} + \frac{\partial \Delta\mathbf{v}_{ij}}{\partial \mathbf{b}_a}\delta\mathbf{b}_a + \frac{\partial \Delta\mathbf{v}_{ij}}{\partial \mathbf{b}_g}\delta\mathbf{b}_g \\[6pt]
\Delta\tilde{\mathbf{q}}_{ij} &= \Delta\mathbf{q}_{ij} \otimes \text{Exp}\left(\frac{\partial \Delta\mathbf{q}_{ij}}{\partial \mathbf{b}_g}\delta\mathbf{b}_g\right)
\end{aligned}
$$

其中 $\delta\mathbf{b}_a = \mathbf{B}_{ai} - \bar{\mathbf{b}}_a$, $\delta\mathbf{b}_g = \mathbf{B}_{gi} - \bar{\mathbf{b}}_g$。

> **为什么只需要一阶修正？** 优化是迭代的，每次迭代 bias 变化量很小（通常在 0.01 量级），二阶及以上项可以忽略。一阶修正在大部分情况下足够精确。

> **当 bias 变化过大时**，代码中保留了重新传播的逻辑（`imu_factor.h:66-72`，当前被 `#if 0` 禁用），会用新线性化点重新调用 `repropagate()`：
> ```cpp
> if ((Bai - linearized_ba).norm() > 0.10 || (Bgi - linearized_bg).norm() > 0.01)
>     pre_integration->repropagate(Bai, Bgi);
> ```

---

## 6. IMU 残差定义

`evaluate()` 第 189-193 行。给定帧 i 和帧 j 的状态，计算 15 维残差：

$$
\mathbf{r} = \begin{bmatrix}
\mathbf{r}_p \\ \mathbf{r}_q \\ \mathbf{r}_v \\ \mathbf{r}_{ba} \\ \mathbf{r}_{bg}
\end{bmatrix} =
\begin{bmatrix}
\mathbf{Q}_i^{-1}\left(\frac{1}{2}\mathbf{g}\Delta t^2 + \mathbf{P}_j - \mathbf{P}_i - \mathbf{V}_i\Delta t\right) - \Delta\tilde{\mathbf{p}}_{ij} \\[6pt]
2\left[\Delta\tilde{\mathbf{q}}_{ij}^{-1} \otimes \left(\mathbf{Q}_i^{-1} \otimes \mathbf{Q}_j\right)\right]_{xyz} \\[6pt]
\mathbf{Q}_i^{-1}\left(\mathbf{g}\Delta t + \mathbf{V}_j - \mathbf{V}_i\right) - \Delta\tilde{\mathbf{v}}_{ij} \\[6pt]
\mathbf{B}_{aj} - \mathbf{B}_{ai} \\[6pt]
\mathbf{B}_{gj} - \mathbf{B}_{gi}
\end{bmatrix}
$$

<details>
<summary>代码对应（点击展开）</summary>

```cpp
residuals.block<3,1>(O_P, 0) =
    Qi.inverse() * (0.5 * G * sum_dt * sum_dt + Pj - Pi - Vi * sum_dt)
    - corrected_delta_p;

residuals.block<3,1>(O_R, 0) =
    2 * (corrected_delta_q.inverse() * (Qi.inverse() * Qj)).vec();

residuals.block<3,1>(O_V, 0) =
    Qi.inverse() * (G * sum_dt + Vj - Vi) - corrected_delta_v;

residuals.block<3,1>(O_BA, 0) = Baj - Bai;
residuals.block<3,1>(O_BG, 0) = Bgj - Bgi;
```

</details>

**物理含义**：

- $\mathbf{r}_p, \mathbf{r}_q, \mathbf{r}_v$: IMU 预积分预测的帧间运动 vs 优化变量推算的运动之差
- $\mathbf{r}_{ba}, \mathbf{r}_{bg}$: 鼓励相邻帧的 bias 保持一致（随机游走先验）

### 协方差加权（信息矩阵）

`imu_factor.h:78-80`：

```cpp
Eigen::Matrix<double, 15, 15> sqrt_info =
    Eigen::LLT<Eigen::Matrix<double, 15, 15>>(
        pre_integration->covariance.inverse()
    ).matrixL().transpose();
residual = sqrt_info * residual;
```

即：

$$
\mathbf{r} \gets \mathbf{L}^{-T}\cdot\mathbf{r}, \quad \text{其中 } \boldsymbol{\Sigma} = \mathbf{L}\mathbf{L}^T
$$

这等价于最小化马氏距离：

$$
\|\mathbf{r}\|_{\boldsymbol{\Sigma}^{-1}}^2 = \mathbf{r}^T \boldsymbol{\Sigma}^{-1} \mathbf{r} = \|\mathbf{L}^{-T}\mathbf{r}\|_2^2
$$

> 物理意义：协方差大的分量（不确定性高）在优化中权重小；协方差小的分量（确定性高）权重大。

---

## 7. 雅可比推导

`imu_factor.h:82-189`。所有雅可比最后都要乘以 `sqrt_info`。

### 7.1 优化变量

IMU 因子连接帧 i 和帧 j：

```cpp
class IMUFactor : public ceres::SizedCostFunction<15, 7, 9, 7, 9>
//                                                残差 Pose_i SB_i Pose_j SB_j

// 参数块:
//   parameters[0] → para_Pose[i]      [x, y, z, qx, qy, qz, qw]   (7)
//   parameters[1] → para_SpeedBias[i] [vx, vy, vz, bax, bay, baz, bgx, bgy, bgz] (9)
//   parameters[2] → para_Pose[j]      (7)
//   parameters[3] → para_SpeedBias[j]  (9)
```

### 7.2 对 Pose i 的雅可比（15×7）

代码第 100-124 行：

```cpp
jacobian_pose_i.block<3,3>(O_P, O_P) = -Qi.inverse().toRotationMatrix();
// ∂rp/∂Pi = -R_i^T

jacobian_pose_i.block<3,3>(O_P, O_R) =
    Utility::skewSymmetric(Qi.inverse() * (0.5*G*sum_dt*sum_dt + Pj - Pi - Vi*sum_dt));
// ∂rp/∂θi = [R_i^T(½gΔt² + Pj - Pi - ViΔt)]_×

jacobian_pose_i.block<3,3>(O_R, O_R) =
    -(Utility::Qleft(Qj.inverse()*Qi) * Utility::Qright(corrected_delta_q)).bottomRightCorner<3,3>();
// ∂rq/∂θi ≈ -L(Qj^{-1}⊗Qi)ᵇᵒᵗ·R(Δq̃)ᵇᵒᵗ

jacobian_pose_i.block<3,3>(O_V, O_R) =
    Utility::skewSymmetric(Qi.inverse() * (G*sum_dt + Vj - Vi));
// ∂rv/∂θi = [R_i^T(gΔt + Vj - Vi)]_×
```

矩阵形式：

$$
\frac{\partial \mathbf{r}}{\partial [\mathbf{P}_i, \boldsymbol{\theta}_i]} =
\begin{bmatrix}
-\mathbf{R}_i^T & \left[\mathbf{R}_i^T\!\left(\frac{1}{2}\mathbf{g}\Delta t^2 + \mathbf{P}_j - \mathbf{P}_i - \mathbf{V}_i\Delta t\right)\right]_{\times} \\[6pt]
\mathbf{0} & -\mathbf{L}(\mathbf{Q}_j^{-1}\!\otimes\!\mathbf{Q}_i)^{\text{bot}}\mathbf{R}(\Delta\tilde{\mathbf{q}})^{\text{bot}} \\[6pt]
\mathbf{0} & \left[\mathbf{R}_i^T(\mathbf{g}\Delta t + \mathbf{V}_j - \mathbf{V}_i)\right]_{\times} \\[6pt]
\mathbf{0} & \mathbf{0} \\[6pt]
\mathbf{0} & \mathbf{0}
\end{bmatrix}
$$

其中 $\mathbf{L}(\cdot)^{\text{bot}}$ 表示四元数左乘矩阵的右下 3×3 块，$\mathbf{R}(\cdot)^{\text{bot}}$ 同理。

### 7.3 对 SpeedBias i 的雅可比（15×9）

代码第 126-153 行：

```cpp
// ∂rp/∂Vi
jacobian_speedbias_i.block<3,3>(O_P, 0) = -Qi.inverse().toRotationMatrix() * sum_dt;

// ∂rp/∂Bai, ∂rp/∂Bgi
jacobian_speedbias_i.block<3,3>(O_P, 3) = -dp_dba;   // -∂Δp/∂ba
jacobian_speedbias_i.block<3,3>(O_P, 6) = -dp_dbg;   // -∂Δp/∂bg

// ∂rq/∂Bgi
jacobian_speedbias_i.block<3,3>(O_R, 6) =
    -Utility::Qleft(Qj.inverse()*Qi*delta_q).bottomRightCorner<3,3>() * dq_dbg;

// ∂rv/∂Vi, ∂rv/∂Bai, ∂rv/∂Bgi
jacobian_speedbias_i.block<3,3>(O_V, 0) = -Qi.inverse().toRotationMatrix();
jacobian_speedbias_i.block<3,3>(O_V, 3) = -dv_dba;
jacobian_speedbias_i.block<3,3>(O_V, 6) = -dv_dbg;

// ∂rba/∂Bai, ∂rbg/∂Bgi
jacobian_speedbias_i.block<3,3>(O_BA, 3) = -I₃;
jacobian_speedbias_i.block<3,3>(O_BG, 6) = -I₃;
```

矩阵形式（列对应 $\mathbf{V}_i, \mathbf{B}_{ai}, \mathbf{B}_{gi}$）：

$$
\frac{\partial \mathbf{r}}{\partial [\mathbf{V}_i, \mathbf{B}_{ai}, \mathbf{B}_{gi}]} =
\begin{bmatrix}
-\mathbf{R}_i^T\!\Delta t & -\frac{\partial\Delta\mathbf{p}}{\partial\mathbf{b}_a} & -\frac{\partial\Delta\mathbf{p}}{\partial\mathbf{b}_g} \\[6pt]
\mathbf{0} & \mathbf{0} & -\mathbf{L}(\mathbf{Q}_j^{-1}\!\otimes\!\mathbf{Q}_i\!\otimes\!\Delta\mathbf{q})^{\text{bot}}\frac{\partial\Delta\mathbf{q}}{\partial\mathbf{b}_g} \\[6pt]
-\mathbf{R}_i^T & -\frac{\partial\Delta\mathbf{v}}{\partial\mathbf{b}_a} & -\frac{\partial\Delta\mathbf{v}}{\partial\mathbf{b}_g} \\[6pt]
\mathbf{0} & -\mathbf{I} & \mathbf{0} \\[6pt]
\mathbf{0} & \mathbf{0} & -\mathbf{I}
\end{bmatrix}
$$

### 7.4 对 Pose j 的雅可比（15×7）

代码第 155-173 行：

```cpp
jacobian_pose_j.block<3,3>(O_P, O_P) = Qi.inverse().toRotationMatrix();
// ∂rp/∂Pj = R_i^T

jacobian_pose_j.block<3,3>(O_R, O_R) =
    Utility::Qleft(corrected_delta_q.inverse() * Qi.inverse() * Qj).bottomRightCorner<3,3>();
// ∂rq/∂θj ≈ L(Δq̃^{-1}⊗Q_i^{-1}⊗Q_j)ᵇᵒᵗ
```

矩阵形式：

$$
\frac{\partial \mathbf{r}}{\partial [\mathbf{P}_j, \boldsymbol{\theta}_j]} =
\begin{bmatrix}
\mathbf{R}_i^T & \mathbf{0} \\[6pt]
\mathbf{0} & \mathbf{L}(\Delta\tilde{\mathbf{q}}^{-1}\!\otimes\!\mathbf{Q}_i^{-1}\!\otimes\!\mathbf{Q}_j)^{\text{bot}} \\[6pt]
\mathbf{0} & \mathbf{0} \\[6pt]
\mathbf{0} & \mathbf{0} \\[6pt]
\mathbf{0} & \mathbf{0}
\end{bmatrix}
$$

### 7.5 对 SpeedBias j 的雅可比（15×9）

代码第 174-188 行。只有速度和 bias 的交叉项非零：

```cpp
jacobian_speedbias_j.block<3,3>(O_V, 0) = Qi.inverse().toRotationMatrix();  // ∂rv/∂Vj = R_i^T
jacobian_speedbias_j.block<3,3>(O_BA, 3) = I₃;                              // ∂rba/∂Baj = I
jacobian_speedbias_j.block<3,3>(O_BG, 6) = I₃;                              // ∂rbg/∂Bgj = I
```

---

## 8. 辅助数学工具（`utility.h`）

### `skewSymmetric(v)` — 反对称矩阵

$$
[\mathbf{v}]_{\times} = \begin{bmatrix}
0 & -v_z & v_y \\
v_z & 0 & -v_x \\
-v_y & v_x & 0
\end{bmatrix}
$$

### `deltaQ(v)` — Lie 代数 → 四元数

将小旋转向量 $\boldsymbol{\phi}$ 转为四元数 $\begin{bmatrix}\cos(\|\boldsymbol{\phi}\|/2),\; \sin(\|\boldsymbol{\phi}\|/2)\frac{\boldsymbol{\phi}}{\|\boldsymbol{\phi}\|}\end{bmatrix}$：

$$
\text{Exp}(\boldsymbol{\phi}) = \begin{bmatrix} 1 \\ \frac{1}{2}\boldsymbol{\phi} \end{bmatrix} \quad (\|\boldsymbol{\phi}\| \text{ 小})
$$

### `Qleft(q)` / `Qright(q)` — 四元数左乘/右乘矩阵

$$
\mathbf{q}_1 \otimes \mathbf{q}_2 = \mathbf{L}(\mathbf{q}_1)\cdot\bar{\mathbf{q}}_2 = \mathbf{R}(\mathbf{q}_2)\cdot\bar{\mathbf{q}}_1
$$

$$
\mathbf{L}(\mathbf{q}) = \begin{bmatrix}
q_w & -q_x & -q_y & -q_z \\
q_x &  q_w & -q_z &  q_y \\
q_y &  q_z &  q_w & -q_x \\
q_z & -q_y &  q_x &  q_w
\end{bmatrix}, \quad
\mathbf{R}(\mathbf{q}) = \begin{bmatrix}
q_w & -q_x & -q_y & -q_z \\
q_x &  q_w &  q_z & -q_y \\
q_y & -q_z &  q_w &  q_x \\
q_z &  q_y & -q_x &  q_w
\end{bmatrix}
$$

这两个矩阵用于旋转残差的求导链式法则。

---

## 9. 优化问题总览

### 9.1 滑动窗口结构

`estimator.cpp:1058-1206` 中构建的 Ceres 优化问题：

```
min Σ ||IMU 残差||_Σ⁻¹² + Σ ρ(||视觉残差||²) + ||边缘化先验||²

优化变量:
  para_Pose[0..10]      (7×11 = 77 维)
  para_SpeedBias[0..10]  (9×11 = 99 维)
  para_Feature[0..N]     (1×N  = N 维)
  para_Ex_Pose[0..1]     (7×2  = 14 维)
  para_Td[0]             (1×1  = 1 维)
```

### 9.2 IMU 残差的优化作用

| 残差分量 | 约束内容 |
|----------|----------|
| $\mathbf{r}_p$ (3) | 帧 i 到 j 的**位移一致性** |
| $\mathbf{r}_q$ (3) | 帧 i 到 j 的**旋转一致性** |
| $\mathbf{r}_v$ (3) | 帧 i 到 j 的**速度一致性** |
| $\mathbf{r}_{ba}$ (3) | 加速度计 bias 的**随机游走**约束 |
| $\mathbf{r}_{bg}$ (3) | 陀螺仪 bias 的**随机游走**约束 |

15 维 IMU 残差 + 2 维视觉重投影残差 + 边缘化先验 → 滑动窗口紧耦合优化 → 实时估计位姿、速度、bias 和特征深度。

---

## 10. 关键流程总结

```
IMU 数据到来 (200Hz)
    │
    ▼
processIMU()
    ├── push_back(dt, acc, gyr)   → 中值积分传播 Δp, Δq, Δv
    │   └── 同时传播 F·J, F·Σ·F^T + V·N·V^T
    │
图像数据到来 (20Hz)
    │
    ▼
processImage()
    │
    ▼
optimization()
    ├── vector2double()            → 状态 → Ceres 数组
    ├── 添加 IMU 残差 (IMUFactor)    ← 核心：连接相邻帧
    ├── 添加视觉残差 (ProjectionFactor)
    ├── 添加边缘化先验
    ├── ceres::Solve()             → 非线性优化
    └── double2vector()            → Ceres 数组 → 状态
```

---

## 参考资料

- Forster, C., Carlone, L., Dellaert, F., & Scaramuzza, D. (2017). *On-Manifold Preintegration for Real-Time Visual-Inertial Odometry.* IEEE TRO.
- Qin, T., Li, P., & Shen, S. (2018). *VINS-Mono: A Robust and Versatile Monocular Visual-Inertial State Estimator.* IEEE TRO.
- `vins/src/factor/integration_base.h` — 中值积分 + 误差状态传播
- `vins/src/factor/imu_factor.h` — IMU 残差雅可比
- `vins/src/utility/utility.h` — 反对称矩阵、四元数运算
