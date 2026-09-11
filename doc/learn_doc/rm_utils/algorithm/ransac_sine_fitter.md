# 正弦拟合算法选择

> **生成时间**：2026-08-25（UTC+8）

> **代码位置**：`rm_utils\algorithm\ransac_sine_fitter.cpp` / `rm_utils\algorithm\ransac_sine_fitter.hpp`

> **命名空间**：`rm_utils`

当前模型为：

```text
y(t) = A * sin(omega * t + phi) + C
```

固定 `omega` 后，可改写为 `y = A1 * sin(omega * t) + A2 * cos(omega * t) + C`。此时 `A1`、`A2`、`C` 都是线性参数，只有 `omega` 是非线性参数。

本文按应用场景选择算法。不要把打符和反陀螺看作同一个拟合问题。

## 0. 先理解问题的结构

### 0.1 为什么这个模型不是一般的四参数非线性拟合

展开相位项：

```text
A * sin(omega*t + phi)
= A * cos(phi) * sin(omega*t) + A * sin(phi) * cos(omega*t)
= A1 * sin(omega*t) + A2 * cos(omega*t)
```

其中：

```text
A1 = A * cos(phi)
A2 = A * sin(phi)
A  = hypot(A1, A2)
phi = atan2(A2, A1)
```

因此，给定一个 `omega` 后，未知量只剩 `A1`、`A2`、`C`，可以通过线性最小二乘直接求解。这种“外层只搜索一个非线性变量、内层精确解线性变量”的形式，通常称为变量投影（variable projection）。

这正是推荐一维频率搜索的根本原因：不需要用随机最小样本同时猜测全部四个参数，也不必用一般的四维非线性优化在局部极小值之间碰运气。

### 0.2 当前 RANSAC 实际在做什么

当前实现每轮执行以下步骤：

1. 从窗口随机抽取 3 个点；
2. 在 `[min_omega_, max_omega_]` 中随机抽一个连续频率；
3. 在该频率下由 3 个点求解 `A1`、`A2`、`C`；
4. 对全部点统计残差小于阈值的内点数；
5. 保留内点数最多的那一个 3 点模型。

这说明其随机性来自两个地方：随机最小样本集和随机频率。MSAC 只能改善第 4 步的评分函数，不能消除频率随机抽样，也不能让最终模型自动使用全部观测。

### 0.3 鲁棒损失的直观含义

普通最小二乘最小化 `sum(residual^2)`。残差放大十倍，影响会放大一百倍，所以一个误检点足以拉偏结果。

Huber 损失在小残差区间仍使用二次损失，保留正常高斯噪声下的精度；超过 `delta` 后变为近似线性增长，大误检不会无限放大影响。IRLS（迭代重加权最小二乘）把这种损失转为多次加权线性最小二乘：残差越大，下一轮的权重越小。

## 0.4 为什么大符选择 B：一维频率搜索 + Huber IRLS

大符满足三个关键前提：

1. **结构完全匹配。** `omega` 是唯一非线性参数；标准规则通常把它限制在很窄的范围内。历史资料常见范围为约 `[1.884, 2.000] rad/s`，但必须按当前赛季规则复核。
2. **结果必须确定。** 火控预测需要帧间稳定。网格搜索没有随机数，相同输入必定得到相同输出；RANSAC/MSAC 的随机假设会带来运行间抖动。
3. **所有正常点都有价值。** 相比只用 3 个点确定模型，Huber IRLS 会利用整个窗口的正常观测，所以估计方差更低。

在约 `0.12 rad/s` 的频率宽度内，即使使用 40 个网格，外层搜索也很轻量。每个频率只需几次 3 列线性方程求解，对几十到一百多个样本点的窗口，计算成本远低于图像处理。

## 结论速查

| 场景 | 主方案 | RANSAC/MSAC 的位置 |
| --- | --- | --- |
| 大符 | `omega` 一维搜索 + Huber IRLS | 仅在大量误检时作为前置筛选 |
| 小陀螺/反陀螺 | EKF 主估计，必要时 EKF 种子 + 局部 Huber 精修 | 不建议作为主估计器 |
| 大比例随机误检 | MSAC/LO-MSAC + Huber IRLS 重拟合 | 用于找可信初值或内点集合 |
| 正常数据、偶发尖峰 | Huber IRLS | 不需要 MSAC |

## 1. 大符：一维频率搜索 + Huber IRLS

### 适用条件

- `omega` 在一个很窄的、由规则约束的区间内；
- 一轮内频率基本恒定；
- 可保留至少一个完整周期的数据；
- 大多数速度观测正确，仅有少量尖峰或短暂误检。

这是大符的首选方案。窄区间内的频率网格搜索是全局搜索，不依赖随机采样；对相同输入会得到相同输出，适合将结果用于发射时机预测。

### 拟合流程

1. 在 `[min_omega, max_omega]` 内均匀扫描 `omega`，例如 40 个网格；
2. 对每个候选频率构造线性矩阵 `X = [sin(omega*t), cos(omega*t), 1]`；
3. 先做一次普通最小二乘得到 `(A1, A2, C)` 初值；
4. 做 3 到 5 次 Huber IRLS，降低大残差点的权重；
5. 以总 Huber 代价最小的频率作为结果；
6. 可用相邻三个网格点的代价做二次插值，再对细化后的频率求解一次；
7. 用 `A = hypot(A1, A2)`、`phi = atan2(A2, A1)` 还原振幅和相位。

### 参数与约束

- `threshold_` 应解释为 Huber 的转折点 `delta`，按观测噪声标定，而不是沿用“RANSAC 内点阈值”的含义；
- 数据时间跨度至少接近一个周期，否则 `sin`、`cos` 与常数列会接近共线，频率不可辨识；
- 规则中的频率和物理约束必须以当赛季规则为准，不能把历史常量直接固化；
- 若确认满足真实大符的物理约束，可将该约束加入拟合以减少自由度；不适用于通用正弦信号时不要强加。

### 物理约束为什么能提高鲁棒性

如果拟合对象确认是标准大符速度模型，规则或模型可能给出振幅和偏置之间的关系。历史资料中常见：

```text
speed = a * sin(omega*t + phi) + b
a + b = 2.09
a in [0.780, 1.045]
```

在这些前提成立时，约束 `C = 2.09 - A` 可以减少一个自由度，使不完整窗口或噪声条件下的拟合更稳定。它不是通用正弦拟合的合法假设：规则更新、单位不同、拟合量不是大符速度，或观测存在额外偏置时，都不能使用。

### 为什么优于当前 RANSAC

- 当前代码对连续 `omega` 做随机采样，命中正确频率附近没有保证；
- 每轮只用 3 点求模型，方差较大；
- 当前实现只按内点数选择模型，未用全部内点做最终重拟合；
- Huber IRLS 使用全部有效数据，同时平滑降低离群点影响。

### 参考实现：大符 Huber IRLS

以下是保持现有构造函数参数形式的实现草案：`max_iterations_` 复用为频率网格步数，`threshold_` 复用为 Huber `delta`。`Result` 新增 `cost` 与 `valid`，`inliers` 保留给现有调用方兼容。

```cpp
struct Result
{
    double A = 0.0, omega = 0.0, phi = 0.0, C = 0.0;
    double cost = 0.0;
    bool valid = false;
    int inliers = 0;
};

bool solve_fixed_omega(double omega, Eigen::Vector3d &beta, double &cost) const;
```

```cpp
void RansacSineFitter::fit()
{
    best_result_ = Result{};
    if (fit_data_.size() < 3) return;

    const int steps = std::max(1, max_iterations_);
    double best_cost = std::numeric_limits<double>::infinity();
    int best_i = -1;
    std::vector<double> costs(steps + 1, std::numeric_limits<double>::infinity());

    for (int i = 0; i <= steps; ++i)
    {
        const double omega = min_omega_ +
            (max_omega_ - min_omega_) * static_cast<double>(i) / steps;
        Eigen::Vector3d beta;
        double cost;
        if (!solve_fixed_omega(omega, beta, cost)) continue;

        costs[i] = cost;
        if (cost < best_cost)
        {
            best_cost = cost;
            best_i = i;
            best_result_.omega = omega;
            best_result_.A = std::hypot(beta(0), beta(1));
            best_result_.phi = std::atan2(beta(1), beta(0));
            best_result_.C = beta(2);
            best_result_.cost = cost;
            best_result_.valid = true;
        }
    }

    // 使用相邻网格点的代价做二次插值，细化 omega。
    if (best_i > 0 && best_i < steps &&
        std::isfinite(costs[best_i - 1]) && std::isfinite(costs[best_i + 1]))
    {
        const double cm = costs[best_i - 1];
        const double c0 = costs[best_i];
        const double cp = costs[best_i + 1];
        const double denom = cm - 2.0 * c0 + cp;
        if (denom > 1e-12)
        {
            const double frac = 0.5 * (cm - cp) / denom;
            const double step = (max_omega_ - min_omega_) / steps;
            const double omega_refined = best_result_.omega + frac * step;
            Eigen::Vector3d beta;
            double cost;
            if (solve_fixed_omega(omega_refined, beta, cost) && cost < best_result_.cost)
            {
                best_result_.omega = omega_refined;
                best_result_.A = std::hypot(beta(0), beta(1));
                best_result_.phi = std::atan2(beta(1), beta(0));
                best_result_.C = beta(2);
                best_result_.cost = cost;
            }
        }
    }

    if (fit_data_.size() > 150) fit_data_.pop_front();
}

bool RansacSineFitter::solve_fixed_omega(
    double omega, Eigen::Vector3d &beta, double &out_cost) const
{
    const size_t n = fit_data_.size();
    if (n < 3) return false;

    Eigen::MatrixXd X(n, 3);
    Eigen::VectorXd y(n);
    for (size_t i = 0; i < n; ++i)
    {
        const double t = fit_data_[i].first;
        X(i, 0) = std::sin(omega * t);
        X(i, 1) = std::cos(omega * t);
        X(i, 2) = 1.0;
        y(i) = fit_data_[i].second;
    }

    beta = X.colPivHouseholderQr().solve(y);
    const double delta = threshold_;
    const double ridge = 1e-9;

    for (int it = 0; it < 5; ++it)
    {
        const Eigen::VectorXd residual = y - X * beta;
        Eigen::VectorXd weights(n);
        for (size_t i = 0; i < n; ++i)
        {
            const double abs_residual = std::abs(residual(i));
            weights(i) = (abs_residual <= delta || abs_residual < 1e-12)
                ? 1.0 : delta / abs_residual;
        }

        const Eigen::MatrixXd XtW = X.transpose() * weights.asDiagonal();
        const Eigen::Matrix3d normal = XtW * X + ridge * Eigen::Matrix3d::Identity();
        const Eigen::Vector3d updated = normal.ldlt().solve(XtW * y);
        if ((updated - beta).norm() < 1e-10)
        {
            beta = updated;
            break;
        }
        beta = updated;
    }

    const Eigen::VectorXd residual = y - X * beta;
    out_cost = 0.0;
    for (size_t i = 0; i < n; ++i)
    {
        const double abs_residual = std::abs(residual(i));
        out_cost += (abs_residual <= delta)
            ? 0.5 * abs_residual * abs_residual
            : delta * (abs_residual - 0.5 * delta);
    }
    return std::isfinite(out_cost);
}
```

## 2. 小陀螺/反陀螺：EKF 为主，局部 Huber 为辅

### 与大符的本质区别

| 特性 | 大符 | 小陀螺/反陀螺 |
| --- | --- | --- |
| 频率范围 | 窄且受规则限制 | 宽且未知 |
| 频率随时间变化 | 一轮内近似恒定 | 可能加速、减速或突然停止 |
| 可观测信号 | 接近完整单正弦 | 装甲板切换带来跳变 |
| 合适窗口 | 秒级、至少一个周期 | 短窗口，通常约 0.5 到 1.0 秒 |

现有 EKF 已负责车辆中心、半径、角速度与装甲板关联，因此应继续作为反陀螺的主估计器。把一个长窗口正弦拟合器直接用于跨装甲板的原始位置，会把切换跳变错误地拟合成运动规律。

### 为什么这不是“两个估计器竞争”

反陀螺中，整车 EKF 已经负责把不同装甲板的观测关联到同一辆车，并估计中心、半径、角速度等状态。正弦拟合器若直接处理原始装甲位置，需要自行解决装甲切换和角度偏移，本质上是在重复实现且弱化 EKF。

因此正弦拟合只在一个窄的辅助用途下有价值：基于 EKF 已得到的连续轨迹，对相位或可击打时刻做低延迟的局部平滑，用于火控时间预测。它不应替代 EKF 做自旋检测、车辆状态估计或跨装甲板关联。

### 三个必须接受的限制

1. **宽频率范围不适合全局网格。** 反陀螺角速度范围宽，可能从静止到很高的值；若用大符式全局网格，成本和误选概率都会上升。
2. **长窗口会造成明显滞后。** 目标加速、减速或停转时，多秒历史数据会把旧角速度带入当前结果。
3. **单块装甲只提供短弧段。** 只见到很小一段正弦曲线时，频率、振幅和偏置可以互相补偿，数据本身不足以可靠辨识 `omega`。

这三个限制共同决定：反陀螺只能使用 EKF 种子的局部频率搜索、短时间窗口和无切换的观测量。

### 正弦拟合可承担的辅助角色

若需要更平滑的相位或用于火控时机，可在以下前提下使用：

1. 使用 EKF 输出的角速度作为 `omega_seed`；
2. 仅在 `omega_seed +/- halfspan` 的局部范围搜索频率；
3. 仅拟合单个连续装甲轨迹，或拟合已由 EKF 消除装甲切换后的中心相对量；
4. 使用短的、按时间裁剪的窗口；
5. 仍采用 Huber IRLS 抑制残余误差。

单个装甲板只覆盖短弧段时，无法仅凭这段数据可靠估计频率，所以局部拟合必须由 EKF 提供初值，不能无约束全局搜索。

### 参考实现：EKF 种子局部频率搜索

该接口复用上一节的 `solve_fixed_omega`，只替换外层频率搜索范围：

```cpp
RansacSineFitter::Result RansacSineFitter::fit_local(
    double omega_seed, double halfspan = 0.6, int local_steps = 12)
{
    best_result_ = Result{};
    if (fit_data_.size() < 3) return best_result_;

    const double lo = std::max(0.1, omega_seed - halfspan);
    const double hi = omega_seed + halfspan;
    double best_cost = std::numeric_limits<double>::infinity();

    for (int i = 0; i <= local_steps; ++i)
    {
        const double omega = lo + (hi - lo) * static_cast<double>(i) / local_steps;
        Eigen::Vector3d beta;
        double cost;
        if (!solve_fixed_omega(omega, beta, cost) || cost >= best_cost) continue;

        best_cost = cost;
        best_result_.omega = omega;
        best_result_.A = std::hypot(beta(0), beta(1));
        best_result_.phi = std::atan2(beta(1), beta(0));
        best_result_.C = beta(2);
        best_result_.cost = cost;
        best_result_.valid = true;
    }
    return best_result_;
}
```

反陀螺使用按时间裁剪的窗口，而不是只保留固定点数：

```cpp
void RansacSineFitter::add_data(double t, double v)
{
    if (!fit_data_.empty() && t - fit_data_.back().first > 5.0) fit_data_.clear();
    fit_data_.emplace_back(t, v);
    while (!fit_data_.empty() && t - fit_data_.front().first > window_sec_)
        fit_data_.pop_front();
}
```

## 3. 大比例误检：MSAC/LO-MSAC + Huber 重拟合

当错误观测经常出现，且离群点比例达到约 30% 到 40% 以上时，纯 IRLS 可能被大量异常值拉偏。此时可采用两阶段方案：

```text
MSAC 或 LO-MSAC 选取可信模型/内点集合
    -> 对内点及其邻域做 Huber IRLS 重拟合
```

MSAC 用截断平方残差评分：`sum(min(residual^2, threshold^2))`。相较只统计内点数的 RANSAC，它能区分内点质量；但它仍有随机性、阈值敏感性和最小样本解的高方差问题。

因此 MSAC 适合作为高离群率条件下的初始化或筛选器，而不是大符或反陀螺的默认持续估计器。无论使用 RANSAC 还是 MSAC，都必须在选出内点后用全部内点重新拟合。

### 为什么 MSAC 不是默认答案

RANSAC 的原始分数只有“内点数”，把残差为 `0` 的点和刚好低于阈值的点同样对待。MSAC 改用截断平方代价，因此在内点数量接近时能偏好残差更小的模型。

但 MSAC 仍有三个没有消失的问题：

- 需要随机假设，输出仍可能随随机种子变化；
- 阈值既决定哪些点被截断，也强烈影响模型排序；
- 最小样本模型仍然噪声很大，必须有局部优化或全量重拟合才能得到可用参数。

所以 MSAC 的优势是“在多数点可能错误时找到一组可信点”，而不是“比鲁棒最小二乘更适合正常连续数据”。

### 参考实现：MSAC 评分

```cpp
double RansacSineFitter::evaluate_msac(
    double A, double omega, double phi, double C) const
{
    const double threshold_sq = threshold_ * threshold_;
    double cost = 0.0;
    for (const auto &[t, value] : fit_data_)
    {
        const double residual = value - (A * std::sin(omega * t + phi) + C);
        cost += std::min(residual * residual, threshold_sq);
    }
    return cost;
}
```

MSAC 选择最小代价模型。选定后应收集 `abs(residual) < threshold_` 的内点，并用全部内点进行一次最小二乘或 Huber IRLS 重拟合；不能直接返回最小样本集的解。

## 4. Huber 与 Cauchy 的选择

| 损失函数 | 推荐场景 | 特性 |
| --- | --- | --- |
| Huber | 默认选择 | 小残差保持二次损失，大残差近似线性，稳定且较易收敛 |
| Cauchy | 极端大残差很多，但主体模型可靠 | 对大残差压制更强，但优化更容易受初值影响 |

对当前问题，优先使用 Huber。只有确认误检残差远大于正常噪声、且初值可靠时，再考虑 Cauchy。

## 5. 当前 `RansacSineFitter` 的实现注意事项

无论是否替换算法，以下问题都应被处理：

1. `best_result_` 没有在每次 `fit()` 开始时复位。数据窗口清空或滑动后，旧结果可能阻止新结果更新。
2. `fit_data_` 只按点数上限裁剪，反陀螺场景更应使用按时间裁剪的短窗口。
3. 3 点在时间跨度过短时会导致线性系统病态，应检查数据时间跨度和解的数值条件。
4. 仅以残差阈值判断内点不能区分模型质量；若保留随机采样，应采用 MSAC 评分并增加内点全量重拟合。
5. 输出结果应包含有效性、鲁棒代价和数据跨度，调用方不能只读取振幅、频率和相位。

### 可辨识性检查应如何理解

仅有三个点在数学上可解，不代表物理上可可靠估计。若样本的时间跨度远小于一个周期，正弦曲线局部接近直线或二次曲线，很多不同的 `omega`、振幅和相位都能解释同一小段数据。

对大符，应在窗口覆盖接近一个周期后再信任频率；对反陀螺，短弧段不满足这个条件，所以必须依赖 EKF 的 `omega_seed` 作为先验。建议将以下指标暴露给调用方：数据时间跨度、最佳与次佳频率代价差、矩阵条件数或有效性标志。

### 接口与工程注意事项

- 若把 `max_iterations_` 改作网格数、把 `threshold_` 改作 Huber `delta`，应在接口文档中明确语义变化；
- 新增 `cost` 与 `valid` 后，调用方应在使用预测值前检查 `valid`；
- 示例代码是算法草案，接入前需补齐 `<limits>`、窗口长度配置、异常输入处理和单元/回放测试；
- 当模型窗口因长时间未收到数据而清空时，`best_result_` 也必须同步清空，避免旧预测继续参与火控。

## 6. 验证标准

离线回放应同时比较当前 RANSAC、MSAC 和 Huber IRLS：

- 频率、相位和预测角度误差；
- 输出的帧间抖动与响应延迟；
- 10%、30%、50% 人工离群点下的失败率；
- 目标加速、减速、停转和装甲板切换时的恢复时间；
- 单次 `fit()` 的最坏执行时间。

只有在实际回放中确认高离群率占主导时，才应将 MSAC 放在主路径上。

## 7.核心结论总结

能量机关场景：方案 B 保持不变（全局网格搜索 + Huber IRLS）。

装甲陀螺场景：EKF 仍然作为旋转状态主估计器；若额外引入正弦拟合模块，则采用**EKF 提供初值、短窗口局部 Huber 精细寻优**架构，并且清晰定位模块用途 —— 仅作为射击相位时序辅助，而非和 EKF 竞争的旋转估计器。
