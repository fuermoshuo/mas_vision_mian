# rune_sine_fitter 数学公式与代码对应讲解

## 一、整体拟合模型
### 物理模型（大符摆动）
大能量机关做周期性正弦摆动，位置随时间变化：
$$v(t) = A\sin(\omega t + \phi) + C$$

| 符号 | 含义 |
|------|------|
| $A$ | 振幅（摆动幅度） |
| $\omega$ | 角频率（角速度） |
| $\phi$ | 初相位 |
| $C$ | 平衡位置（摆动中心） |

### 线性化展开（关键技巧）
直接拟合 $A,\omega,\phi,C$ 是非线性问题，不好解。
利用三角恒等变换，把模型改写为线性形式：
$$v(t) = a\cdot\sin(\omega t) + b\cdot\cos(\omega t) + C$$

其中：
$$a = A\cos\phi$$
$$b = A\sin\phi$$

**固定 $\omega$ 之后，$a,b,C$ 就是线性参数，可以用最小二乘直接求解。**
这就是「外层网格搜索ω + 内层线性最小二乘」的核心思路。

### 参数还原
解出 $a,b$ 之后，反推物理参数：
$$A = \sqrt{a^2 + b^2}$$
$$\phi = \mathrm{atan2}(b, a)$$

---

## 二、模块1：数据录入 add_data()
### 代码
```cpp
void rune_sine_fitter::add_data(double t, double v)
{
    if (fit_data_.size() > 0 && (t - fit_data_.back().first > 5)) fit_data_.clear();
    fit_data_.emplace_back(std::make_pair(t, v));
}
```

### 数学含义
持续收集观测点集合：
$$\{(t_i,\ v_i)\},\quad i=1,2,\dots,N$$

- $t_i$：第 i 帧的时间戳
- $v_i$：第 i 帧识别到的大符位置

断档超过 5 秒清空历史，避免新旧两段不同运动混在一起拟合。

---

## 三、模块2：外层一维ω网格搜索 fit()
### 核心思想
$\omega$ 是非线性参数，不能直接最小二乘求解。
做法：在合理区间 $[\omega_{min},\ \omega_{max}]$ 内均匀撒点，挨个试，挑拟合效果最好的那个。

大符标准角速度区间：
$$\omega \in [1.884,\ 2.0]\ \mathrm{rad/s}$$

### 2.1 生成网格步长
```cpp
double omega_step = (max_omega - min_omega) / num_segments_;
```
$$\Delta\omega = \frac{\omega_{max} - \omega_{min}}{K}$$

- $K$ = `num_segments_` 分段数量
- 分段越多，搜索越精细，计算量越大

### 2.2 遍历每一个候选ω
```cpp
for(double omega = min_omega; omega <= max_omega; omega += omega_step)
```
$$\omega_k = \omega_{min} + k\cdot\Delta\omega,\quad k=0,1,\dots,K$$

### 2.3 构建设计矩阵 X 和观测向量 Y
```cpp
Eigen::MatrixXd X(N, 3);
Eigen::VectorXd Y(N);

for(size_t i = 0; i < N; ++i)
{
    X(i,0) = std::sin(omega * t);
    X(i,1) = std::cos(omega * t);
    X(i,2) = 1.0;
    Y(i)   = v;
}
```

矩阵形式：
$$
\underbrace{
\begin{bmatrix}
\sin(\omega t_1) & \cos(\omega t_1) & 1 \\
\sin(\omega t_2) & \cos(\omega t_2) & 1 \\
\vdots & \vdots & \vdots \\
\sin(\omega t_N) & \cos(\omega t_N) & 1
\end{bmatrix}
}_{X:\ N\times 3}
\underbrace{
\begin{bmatrix}
a \\ b \\ C
\end{bmatrix}
}_{\theta:\ 3\times 1}
\approx
\underbrace{
\begin{bmatrix}
v_1 \\ v_2 \\ \vdots \\ v_N
\end{bmatrix}
}_{Y:\ N\times 1}
$$

即线性方程组：
$$X\theta \approx Y$$

> ⚠️ 注意：X 第三列全是 1，对应常数项 C。如果只有 2 列，就是强制 C=0，拟合会有永久偏移。

### 2.4 调用内层IRLS求解
```cpp
if(!Huber_loss_calculate(X, Y, theta, cost)) continue;
```
对当前候选 ω，执行鲁棒最小二乘，求出参数 $\theta=[a,b,C]^T$ 和拟合代价。

### 2.5 更新全局最优模型
```cpp
if(cost < best_cost)
{
    best_result_.A   = std::hypot(theta(0), theta(1));
    best_result_.omega = omega;
    best_result_.phi   = std::atan2(theta(1), theta(0));
    best_result_.C     = theta(2);
}
```

数学公式：
$$A = \sqrt{a^2 + b^2}$$
$$\phi = \mathrm{atan2}(b, a)$$

挑选准则：
$$\omega^* = \arg\min_{\omega_k}\ \mathcal{L}(\omega_k)$$

遍历所有候选 ω，保留**拟合代价最小**的那一组正弦参数。

---

## 四、模块3：内层 Huber IRLS 鲁棒求解
### 3.1 什么是 IRLS
**IRLS = Iteratively Reweighted Least Squares（迭代重加权最小二乘）**

普通最小二乘：所有点地位平等，一个噪声点就能把整条曲线带歪。
Huber-IRLS：每轮迭代重新计算每个点的权重，离曲线远的噪声点权重降低，影响力变小。

### 3.2 计算残差
```cpp
Eigen::VectorXd residuals = Y - X * theta;
```
$$\boldsymbol{r} = Y - X\theta$$
$$r_i = v_i - \big(a\sin\omega t_i + b\cos\omega t_i + C\big)$$

残差 = 真实观测 − 模型预测，表征第 i 个点离当前拟合曲线有多远。

### 3.3 Huber 权重函数
```cpp
double rune_sine_fitter::Huber_Weight(double residuals, double threshold_)
{
    if(abs(residuals) < threshold_)
        return 1.0;
    else
        return threshold_ / abs(residuals);
}
```

数学定义：
$$
w_i =
\begin{cases}
1 & |r_i| \le \delta \\
\dfrac{\delta}{|r_i|} & |r_i| > \delta
\end{cases}
$$

- $\delta$ = `threshold_` Huber 阈值
- 残差小 → 权重 = 1，正常点充分信任
- 残差大 → 权重 < 1，疑似噪声，降低话语权

> ⚠️ 重要区分：
> - **权重函数 w(r)**：IRLS 迭代内部用，数值 ≤ 1，离群点权重小
> - **损失函数 ρ(r)**：评估拟合好坏用，随残差增大而增长
> 两者不是一回事，不能混用。

### 3.4 构造权重对角矩阵
```cpp
Eigen::MatrixXd W = weights.asDiagonal();
```
$$
W = \mathrm{diag}(w_1, w_2, \dots, w_N) =
\begin{bmatrix}
w_1 & 0 & \dots & 0 \\
0 & w_2 & \dots & 0 \\
\vdots & \vdots & \ddots & \vdots \\
0 & 0 & \dots & w_N
\end{bmatrix}
$$

把一维权重数组变成对角矩阵，方便后续矩阵乘法。

### 3.5 加权最小二乘正规方程
```cpp
Eigen::MatrixXd XTWX = X.transpose() * W * X + ridge_penalty_ * Eigen::MatrixXd::Identity(X.cols(), X.cols());
Eigen::VectorXd XTWY = X.transpose() * W * Y;
theta = XTWX.ldlt().solve(XTWY);
```

#### 数学来源
优化目标：最小化**加权残差平方和**
$$\min_{\theta}\ \sum_{i=1}^N w_i \cdot r_i^2 = (Y-X\theta)^T W (Y-X\theta)$$

对 $\theta$ 求导并令导数为 0，推导得到**加权正规方程**：
$$(X^T W X)\ \theta = X^T W Y$$

#### 为什么要 X.transpose()？
原始 X 是 N 行 3 列的长方形矩阵，不能直接求逆。
左右两边同时左乘 $X^T$ 之后，$X^T W X$ 变成 3×3 的**方阵**，方阵才能分解求解。

维度演算：
$$\underbrace{X^T}_{3\times N} \cdot \underbrace{W}_{N\times N} \cdot \underbrace{X}_{N\times 3} = \underbrace{X^TWX}_{3\times 3\ \text{方阵}}$$

### 3.6 岭正则（Ridge）
```cpp
+ ridge_penalty_ * Eigen::MatrixXd::Identity(X.cols(), X.cols())
```

$$X^T W X + \lambda I$$

- $\lambda$ = `ridge_penalty_` 岭正则系数
- 作用：当观测点太少、矩阵接近奇异时，保证矩阵正定可逆，防止求解出现 NaN / Inf

> 进阶优化建议：只惩罚 a、b 两个正弦系数，不惩罚常数项 C，即正则矩阵用 `diag(λ, λ, 0)`。

### 3.7 求解更新参数
```cpp
theta = XTWX.ldlt().solve(XTWY);
```

解线性方程组：
$$(X^T W X + \lambda I)\ \theta = X^T W Y$$

`ldlt()` 是 Eigen 的 LDLT 矩阵分解，专门高效求解对称正定方阵。

### 3.8 计算加权代价
```cpp
Eigen::VectorXd r = Y - X * theta;
double new_cost = (r.transpose() * W * r).value();
```

$$\mathcal{J} = r^T W r = \sum_{i=1}^N w_i \cdot r_i^2$$

维度演算：
$$\underbrace{r^T}_{1\times N} \cdot \underbrace{W}_{N\times N} \cdot \underbrace{r}_{N\times 1} = \text{一个标量数字}$$

这个数字代表当前曲线整体拟合好坏，数字越小越贴合。

### 3.9 收敛判断
```cpp
if(std::abs(new_cost - cost) < 1e-6) break;
```

$$|\mathcal{J}_{new} - \mathcal{J}_{old}| < \varepsilon$$

前后两轮代价变化极小，认为参数收敛，停止迭代。

### 3.10 IRLS 完整循环流程
1. 用当前参数计算每个点的残差
2. 根据残差计算 Huber 权重（噪声点降权）
3. 构造加权正规方程，求解新一轮参数
4. 计算新代价，判断是否收敛
5. 未收敛则回到第 1 步继续迭代

---

## 五、整体算法流程图
```
输入时序观测点 (t, v)
        ↓
外层：遍历每一个候选 ω
        ↓
    固定 ω，构造矩阵 X、Y
        ↓
    内层：Huber-IRLS 迭代求解 a,b,C
        ↓
    计算当前 ω 的拟合代价
        ↓
保留代价最小的 ω 对应的正弦模型
        ↓
输出 A, ω, φ, C
```

---

## 六、当前代码存在的问题汇总
### 编译/语法类
1. `Huber_Weight` 函数里用 `abs()` 处理 double，建议改用 `std::fabs()`
2. `num_segments_` 是 double 类型，分段数量应为整数
3. `for(double omega ... += omega_step)` 浮点累加有精度误差，推荐用整数索引

### 逻辑类
4. **fit() 开头没有重置 best_result_**，拟合失败时会沿用旧参数
5. 岭正则目前惩罚全部 3 个参数，建议只惩罚 a、b，不惩罚 C
6. 当前用加权平方残差 `rᵀWr` 选 ω，数学上不严谨；推荐改用统一的 Huber 损失总和择优
7. `add_data` 只有断档清空，没有滑动窗口，数据会无限累积
8. 只有 3 个点就开始拟合，结果不可靠；应设置最少点数 + 最小时长门槛
9. Result 没有 valid 标志位，上层无法区分拟合成功/失败
10. 没有使用相对时间 τ = t - t_ref，大时间戳下三角函数精度损失

### 命名类
11. 函数名 `Huber_loss_calculate` 实际做的是 IRLS 求解，名字有歧义
12. `Huber_Weight` 返回的是权重，不是 loss，命名应统一

---

## 七、预测使用公式
拟合得到参数后，预测任意时刻 t 的位置：

$$v_{pred}(t) = A \cdot \sin\big(\omega \cdot t + \phi\big) + C$$

如果使用相对时间基准 $t_{ref}$，则：

$$v_{pred}(t) = A \cdot \sin\big(\omega \cdot (t - t_{ref}) + \phi\big) + C$$

> 相对时间版本数值稳定性更好，推荐工程使用。
