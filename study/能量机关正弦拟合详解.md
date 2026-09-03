# `rune_sine_fitter` 数学公式与代码对应说明

> **生成时间**：2026-08-25（UTC+8）

> **代码位置**：`rm_utils\algorithm\rune_sine_fitter.cpp` / `rm_utils\algorithm\rune_sine_fitter.hpp`

> **命名空间**：`rm_utils`

`rune_sine_fitter` 用于估计大能量机关的单变量周期轨迹。算法分为两层：

1. 外层在有限的角频率区间中枚举候选 $\omega$。
2. 内层在固定 $\omega$ 时，通过 Huber IRLS 鲁棒地求解线性参数 $a,b,C$。

最后以**标准 Huber 损失**在所有候选频率中选择最优模型。

## TODO

以下项目尚未在 C++ 实现中完成，保留在这里作为后续修改清单：

1. 在 `Result` 中保存 `t0`（或 `time_reference`），使调用方能用拟合结果正确预测绝对时间。
2. 在构造函数中检查 `num_segments_ > 0`、`max_iterations > 0`、`threshold_ > 0`、`ridge_penalty_ >= 0`。
3. 增加最小样本数和最小时间跨度门槛；仅有 3 个点不足以可靠辨识频率。
4. 全部候选频率求解失败时，将 `best_result_.is_valid` 置为 `false`，避免把历史结果误认为本次成功结果。
5. 用参数变化量或重新计算的 Huber 损失作为 IRLS 收敛判据，替代旧权重下的 $r^TWr$ 差值。
6. 按 `|r_i| \le \delta` 统计真实内点数，或将 `inliers` 改名为 `sample_count`。
7. 将 `rune_sine_fitter.cpp` 加入 `rm_utils/CMakeLists.txt`，并为正常数据、离群点、断档和无效参数补充测试。

---

## 1. 整体拟合模型

### 1.1 物理模型

将被观测量记为 $v(t)$。模型假设它围绕中心值 $C$ 做单频正弦运动：

$$
v(t)=A\sin(\omega t+\phi)+C
$$

| 符号 | 含义 | 单位 |
| --- | --- | --- |
| $A$ | 振幅 | 与 $v$ 相同 |
| $\omega$ | 角频率 | rad/s |
| $\phi$ | 初相位 | rad |
| $C$ | 偏置或运动中心 | 与 $v$ 相同 |

直接同时拟合 $A,\omega,\phi,C$ 是非线性优化问题。当前实现将 $\omega$ 放在外层搜索，其余参数在线性子问题中求解。

### 1.2 相对时间基准

代码不会直接使用绝对时间戳，而是将窗口中最早的采样时间记为：

$$
t_0=t_{\text{first}}
$$

对任意采样点定义相对时间：

$$
\tau=t-t_0
$$

所以当前实际拟合模型是：

$$
v(t)=A\sin\bigl(\omega(t-t_0)+\phi\bigr)+C
$$

对应代码：

```cpp
double t0  = fit_data_.front().first;
double tau = fit_data_[i].first - t0;

X(i, 0) = std::sin(omega * tau);
X(i, 1) = std::cos(omega * tau);
X(i, 2) = 1.0;
```

这样可以避免将很大的系统时间戳传给 `sin()` 和 `cos()`，改善三角函数计算的数值稳定性。

### 1.3 线性化展开

使用和角公式：

$$
\sin(\omega\tau+\phi)=\sin(\omega\tau)\cos\phi+\cos(\omega\tau)\sin\phi
$$

令：

$$
a=A\cos\phi,\qquad b=A\sin\phi
$$

原模型可写成：

$$
v(t)=a\sin(\omega\tau)+b\cos(\omega\tau)+C
$$

固定 $\omega$ 后，$a,b,C$ 都是线性参数，因此可由加权最小二乘直接求解。

### 1.4 参数还原

内层求解返回：

$$
\theta=\begin{bmatrix}a & b & C\end{bmatrix}^T
$$

再恢复：

$$
A=\sqrt{a^2+b^2}
$$

$$
\phi=\operatorname{atan2}(b,a)
$$

对应代码：

```cpp
temp_result_.A     = std::sqrt(theta(0) * theta(0) + theta(1) * theta(1));
temp_result_.omega = omega;
temp_result_.phi   = std::atan2(theta(1), theta(0));
temp_result_.C     = theta(2);
```

> `phi` 是以 `t0` 为零点的相位，而非绝对时间模型中的相位。当前 `Result` 没有输出 `t0`，因此不能只使用 `A/omega/phi/C` 对绝对时间 `t` 直接预测。

---

## 2. 数据录入与滑动窗口：`add_data()`

### 2.1 当前代码

```cpp
void rune_sine_fitter::add_data(double t, double v)
{
    if (!fit_data_.empty() && t - fit_data_.back().first > 5)
    {
        fit_data_.clear();
    }
    while (!fit_data_.empty() && t - fit_data_.front().first > 5)
    {
        fit_data_.pop_front();
    }
    while (fit_data_.size() >= 300)
    {
        fit_data_.pop_front();
    }
    fit_data_.emplace_back(t, v);
}
```

### 2.2 行为说明

输入点集合为：

$$
\{(t_i,v_i)\},\qquad i=1,2,\ldots,N
$$

- 若本次时间与上一点时间间隔超过 5 s，清空旧窗口，防止断档前后的两段运动混合拟合。
- 连续采样时，移除早于当前时间 5 s 的样本。
- 同时将样本数上限限制为 300，避免高帧率下矩阵无限增大。

该逻辑要求时间戳单调递增。若允许乱序输入，`t - fit_data_.front().first` 的窗口判断将失效，因此应拒绝乱序点或清空窗口重新开始。

---

## 3. 外层频率网格搜索：`fit()`

### 3.1 搜索范围

当前类将大符角频率限制在：

$$
\omega\in[1.884,2.0]\ \mathrm{rad/s}
$$

令 `num_segments_ = K`，网格步长为：

$$
\Delta\omega=\frac{\omega_{\max}-\omega_{\min}}{K}
$$

候选频率为：

$$
\omega_k=\omega_{\min}+k\Delta\omega,\qquad k=0,1,\ldots,K
$$

### 3.2 为什么使用整数索引

当前循环：

```cpp
for (int k = 0; k <= num_segments_; ++k)
{
    const double omega = min_omega + k * omega_step;
}
```

优点是：

1. `k = 0` 包含 `min_omega`。
2. `k = num_segments_` 包含 `max_omega`。
3. 不会像 `omega += omega_step` 一样累积浮点误差。

### 3.3 设计矩阵

对于 $N$ 个样本，固定一个候选频率后：

$$
X=
\begin{bmatrix}
\sin(\omega\tau_1) & \cos(\omega\tau_1) & 1\\
\sin(\omega\tau_2) & \cos(\omega\tau_2) & 1\\
\vdots & \vdots & \vdots\\
\sin(\omega\tau_N) & \cos(\omega\tau_N) & 1
\end{bmatrix}_{N\times3}
$$

$$
Y=\begin{bmatrix}v_1&v_2&\cdots&v_N\end{bmatrix}^T_{N\times1}
$$

$$
\theta=\begin{bmatrix}a&b&C\end{bmatrix}^T_{3\times1}
$$

线性关系为：

$$
X\theta\approx Y
$$

第三列恒为 1，使模型可以估计偏置 $C$；若没有该列，则等价于强制 $C=0$。

---

## 4. Huber IRLS 鲁棒求解

### 4.1 残差

在当前参数 $\theta$ 下，残差向量是：

$$
\mathbf r=Y-X\theta
$$

第 $i$ 个残差为：

$$
r_i=v_i-\left(a\sin(\omega\tau_i)+b\cos(\omega\tau_i)+C\right)
$$

### 4.2 岭正则初始化

每个候选频率先以普通最小二乘的权重 $W_0=I$ 计算初值：

$$
\theta_0=(X^TX+R)^{-1}X^TY
$$

当前正则矩阵为：

$$
R=\operatorname{diag}(\lambda,\lambda,0)
$$

对应代码：

```cpp
Eigen::Vector3d reg(ridge_penalty_, ridge_penalty_, 0.0);
Eigen::MatrixXd initXTWX = X.transpose() * X + reg.asDiagonal().toDenseMatrix();
theta = initXTWX.ldlt().solve(X.transpose() * Y);
```

只惩罚 $a,b$，不惩罚常数项 $C$。原因是 $a,b$ 与振幅相关，病态数据时容易被放大；而把 $C$ 一并惩罚会不必要地将运动中心拉向零。

### 4.3 Huber 权重函数

设阈值 $\delta=\texttt{threshold\_}$。IRLS 使用的权重为：

$$
w_i=
\begin{cases}
1,&|r_i|\le\delta\\
\dfrac{\delta}{|r_i|},&|r_i|>\delta
\end{cases}
$$

对应函数：

```cpp
double rune_sine_fitter::Huber_Weight(double residual, double threshold)
{
    if (std::fabs(residual) <= threshold)
    {
        return 1.0;
    }
    return threshold / std::fabs(residual);
}
```

小残差的权重为 1；大残差的权重会随残差绝对值增大而减小。因此离群点仍参与拟合，但影响被降低。

### 4.4 加权正则最小二乘

将所有权重写成对角矩阵：

$$
W=\operatorname{diag}(w_1,w_2,\ldots,w_N)
$$

每轮 IRLS 求解目标：

$$
\min_\theta\ (Y-X\theta)^TW(Y-X\theta)+\theta^TR\theta
$$

令其对 $\theta$ 的导数为 0，可得正规方程：

$$
(X^TWX+R)\theta=X^TWY
$$

因此：

$$
\theta=(X^TWX+R)^{-1}X^TWY
$$

对应代码：

```cpp
Eigen::MatrixXd W = weights.asDiagonal();
auto XTWX = X.transpose() * W * X + reg.asDiagonal().toDenseMatrix();
Eigen::VectorXd XTWY = X.transpose() * W * Y;
theta = XTWX.ldlt().solve(XTWY);
```

矩阵维度为：

$$
\underbrace{X^T}_{3\times N}
\underbrace{W}_{N\times N}
\underbrace{X}_{N\times3}
=
\underbrace{X^TWX}_{3\times3}
$$

### 4.5 当前停止条件

实现中计算：

```cpp
Eigen::VectorXd r = Y - X * theta;
double new_cost = (r.transpose() * W * r).value();

if (std::abs(new_cost - cost) < 1e-6)
{
    break;
}
```

该量是旧权重矩阵 $W$ 下的加权平方残差：

$$
J=r^TWr=\sum_iw_ir_i^2
$$

它可作为近似停止信号，但不是严格的 Huber 目标值，因为 $W$ 是根据上轮参数计算的。严格实现宜比较相邻两轮参数的变化量，或重新计算第 5 节的 Huber 损失。

---

## 5. 候选频率的统一择优标准

不能直接用不同候选频率各自的 $r^TWr$ 相互比较：不同候选频率的权重 $W$ 不同，数值不在同一个评价标准下。

当前代码在每个候选频率完成 IRLS 后，使用标准 Huber 损失：

$$
\rho_\delta(r)=
\begin{cases}
\frac{1}{2}r^2,&|r|\le\delta\\
\delta\left(|r|-\frac{1}{2}\delta\right),&|r|>\delta
\end{cases}
$$

总损失为：

$$
L(\omega)=\sum_{i=1}^N\rho_\delta(r_i)
$$

最终选择：

$$
\omega^*=\arg\min_{\omega_k}L(\omega_k)
$$

对应代码逻辑：

```cpp
double total_loss = 0.0;
for (size_t i = 0; i < fit_data_.size(); ++i)
{
    const double r = Y(i) - X.row(i) * theta;
    const double ar = std::fabs(r);
    total_loss += ar <= threshold_
        ? 0.5 * r * r
        : threshold_ * (ar - 0.5 * threshold_);
}

if (total_loss < best_huber_loss)
{
    // 保存本次最优的 A、omega、phi、C
}
```

---

## 6. 结果有效性与预测

### 6.1 `Result` 的当前含义

```cpp
struct Result
{
    double A;
    double omega;
    double phi;
    double C;
    int    inliers;
    bool   is_valid;
};
```

- `is_valid`：样本数少于 3 时会设为 `false`；存在可用候选模型时为 `true`。
- `inliers`：当前实际填入窗口样本总数，不是按 Huber 阈值统计的内点数量。
- `phi`：相对于窗口起点 `t0` 的相位。

### 6.2 正确的预测公式

若调用方持有本次拟合的 `t0`，则未来时刻 $t$ 的预测值为：

$$
v_{\mathrm{pred}}(t)=A\sin\bigl(\omega(t-t_0)+\phi\bigr)+C
$$

当前 `Result` 不含 `t0`，因此这是一个尚未封装完整的接口。推荐将结果扩展为：

```cpp
struct Result
{
    double A;
    double omega;
    double phi;
    double C;
    double time_reference;
    int    inliers;
    bool   is_valid;
};
```

### 6.3 最小数据要求

虽然代码用 `fit_data_.size() < 3` 拦截数据不足，但固定频率时恰好有 $a,b,C$ 三个线性未知量。仅 3 个点几乎能被很多候选频率解释，频率并不可可靠辨识。

工程上应同时要求：

1. 足够的点数；
2. 足够的时间跨度；
3. 足够的有效振幅或信噪比。

对于当前约 3.14 s 周期的搜索范围，建议窗口覆盖至少半个周期，理想情况下接近一个周期。

---

## 7. 典型使用顺序

```cpp
rm_utils::rune_sine_fitter fitter(
    10,      // max_iterations
    delta,   // Huber threshold
    lambda,  // ridge penalty
    50);     // omega grid segments

for (const auto& [t, v] : samples)
{
    fitter.add_data(t, v);
}

fitter.fit();
const auto& result = fitter.get_best_result();

if (result.is_valid)
{
    // 只有取得同一轮拟合的 t0 后，才能正确预测：
    // v_pred = result.A * sin(result.omega * (t - t0) + result.phi) + result.C;
}
```

在将该类接入业务代码前，还需要把 `rune_sine_fitter.cpp` 加入 `rm_utils/CMakeLists.txt` 的 `add_library(rm_utils STATIC ...)` 源文件列表，否则该实现不会参与构建。
