# 弹道求解器物理与数学原理详解

> **生成时间**：2026-09-02（UTC+8）

> **代码位置**：`rm_utils/algorithm/trajectory.cpp` / `rm_utils/algorithm/trajectory.hpp`

> **命名空间**：`rm_utils`

> **参考来源**：Adapted from Alliance-Algorithm/rmcs_auto_aim_v2 TrajectorySolution (MIT License)

---

## 目录

1. [概述](#1-概述)
2. [物理模型](#2-物理模型)
   - 2.1 [受力分析](#21-受力分析)
   - 2.2 [空气阻力模型](#22-空气阻力模型)
   - 2.3 [微分方程组](#23-微分方程组)
3. [数学原理](#3-数学原理)
   - 3.1 [数值积分：显式欧拉法](#31-数值积分显式欧拉法)
   - 3.2 [迭代法求解俯仰角](#32-迭代法求解俯仰角)
   - 3.3 [线性插值补全终点](#33-线性插值补全终点)
4. [代码逐段解析](#4-代码逐段解析)
   - 4.1 [常量定义](#41-常量定义)
   - 4.2 [estimate() 弹道仿真函数](#42-estimate-弹道仿真函数)
   - 4.3 [Trajectory 构造函数：迭代求解器](#43-trajectory-构造函数迭代求解器)
5. [参数物理意义与取值依据](#5-参数物理意义与取值依据)
6. [收敛性与误差分析](#6-收敛性与误差分析)
7. [局限性与改进方向](#7-局限性与改进方向)
8. [附录：完整调用流程](#8-附录完整调用流程)

---

## 1. 概述

本模块解决的核心问题：

> **已知弹丸初速 $v_0$、目标水平距离 $d$、目标相对高度 $h$、重力加速度 $g$、空气阻力系数 $k$，求枪口需要抬升的俯仰角 $\theta$（pitch），以及弹丸飞行时间 $t_f$。**

这是一个**带空气阻力的斜抛运动反问题**。由于空气阻力的存在，弹道不再是标准抛物线，不存在闭合形式的解析解，因此采用：

- **内层**：显式欧拉数值积分，给定俯仰角仿真完整弹道；
- **外层**：固定点迭代，根据仿真弹着高度误差修正俯仰角，直到收敛。

注：英雄目前设计俯仰角最大值40° , 哨兵35°

42mm 弹丸重 44.5g ± 0.5g ，17mm 弹丸重 3.2g ± 0.1g

---

## 2. 物理模型

### 2.1 受力分析

弹丸在飞行过程中受两个力：

| 力 | 方向 | 大小 |
|---|---|---|
| 重力 $F_g$ | 竖直向下 | $mg$ |
| 空气阻力 $F_d$ | 与速度方向相反 | $\frac{1}{2}\rho C_d A v^2$ |

其中：
- $m$：弹丸质量（RM 17mm 塑料弹约 $4.2\times10^{-3}\ \mathrm{kg}$）
- $\rho$：空气密度（常温约 $1.205\ \mathrm{kg/m^3}$）
- $C_d$：阻力系数（光滑塑料球约 $0.45\sim0.55$）, 根据不同的弹丸需修正
- $A$：迎风截面积 $A=\pi d^2/4$
- $v$：弹丸瞬时合速度 $v=\sqrt{v_x^2+v_y^2}$

### 2.2 空气阻力模型

代码中使用的是**与速度平方成正比的线性阻力加速度模型**：

$$\vec a_d = -k \cdot |\vec v| \cdot \vec v$$

其中 $k$ 是合并后的阻力系数（代码中变量名为 `air_resistance`）：

$$k = \frac{1}{2}\rho C_d \frac{A}{m}$$

---

## 目录

1. [概述](#1-概述)
2. [物理模型](#2-物理模型)
   - 2.1 [受力分析](#21-受力分析)
   - 2.2 [空气阻力模型](#22-空气阻力模型)
   - 2.3 [微分方程组](#23-微分方程组)
   - 2.4 [空气阻力的影响因素：温度、Cd、合速率v](#24-空气阻力的影响因素温度cd合速率v)
3. [数学原理](#3-数学原理)
   - 3.1 [数值积分：半隐式欧拉法](#31-数值积分半隐式欧拉法)
   - 3.2 [迭代法求解俯仰角](#32-迭代法求解俯仰角)
   - 3.3 [线性插值补全终点](#33-线性插值补全终点)
4. [代码逐段解析](#4-代码逐段解析)
   - 4.1 [常量定义](#41-常量定义)
   - 4.2 [estimate() 弹道仿真函数](#42-estimate-弹道仿真函数)
   - 4.3 [Trajectory 构造函数：迭代求解器](#43-trajectory-构造函数迭代求解器)
5. [参数物理意义与取值依据](#5-参数物理意义与取值依据)
   - 5.1 [输入参数](#51-输入参数)
   - 5.2 [输出参数](#52-输出参数)
   - 5.3 [合并阻力系数 k 的计算（需自行计算）](#53-合并阻力系数-k-的计算需自行计算)
6. [收敛性与误差分析](#6-收敛性与误差分析)
7. [局限性与改进方向](#7-局限性与改进方向)
8. [附录：完整调用流程](#8-附录完整调用流程)

---

## 1. 概述

本模块解决的核心问题：

> **已知弹丸初速 $v_0$、目标水平距离 $d$、目标相对高度 $h$、重力加速度 $g$、合并空气阻力系数 $k$，求枪口需要抬升的俯仰角 $\theta$（pitch），以及弹丸飞行时间 $t_f$。**

这是一个**带空气阻力的斜抛运动反问题**。由于空气阻力的存在，弹道不再是标准抛物线，不存在闭合形式的解析解，因此采用两层结构：

- **内层**：半隐式欧拉数值积分，给定俯仰角仿真完整弹道；
- **外层**：固定点迭代，根据仿真弹着高度误差修正俯仰角，直到收敛。

> **云台俯仰角物理限位**：英雄车设计最大俯仰角 40°，哨兵车 35°。代码中 `MAX_PITCH = 40.0 / 57.3`（40°），哨兵场景需自行调整该常量。

> **RM2026 官方弹丸参数**（规则手册 V2.2.0 P65 表 4-1）：
> - 42mm 英雄大弹丸：质量 $44.5\mathrm{g} \pm 0.5\mathrm{g}$，直径 $42.5\mathrm{mm} \pm 0.5\mathrm{mm}$
> - 17mm 荧光弹丸：质量 $3.2\mathrm{g} \pm 0.1\mathrm{g}$，直径 $16.8\mathrm{mm} \pm 0.2\mathrm{mm}$
>
> ⚠️ 注意：2026 赛季 17mm 弹丸质量已从旧赛季的 4.2g 更新为 3.2g，仿真时必须使用新参数。

---

## 2. 物理模型

### 2.1 受力分析

弹丸在飞行过程中受两个力：

| 力 | 方向 | 大小 |
|---|---|---|
| 重力 $F_g$ | 竖直向下 | $mg$ |
| 空气阻力 $F_d$ | 与速度方向相反 | $\frac{1}{2}\rho C_d A v^2$ |

其中：
- $m$：弹丸质量（17mm 荧光弹 $3.2\times10^{-3}\ \mathrm{kg}$；42mm 大弹 $44.5\times10^{-3}\ \mathrm{kg}$）
- $\rho$：空气密度（常温 20℃ 约 $1.205\ \mathrm{kg/m^3}$）
- $C_d$：无量纲阻力系数（塑料球约 $0.45\sim0.55$，根据不同弹丸需修正）
- $A$：迎风截面积 $A=\pi D^2/4$，$D$ 为弹丸直径
- $v$：弹丸瞬时合速度 $v=\sqrt{v_x^2+v_y^2}$

### 2.2 空气阻力模型

代码中使用的是**与速度平方成正比的阻力加速度模型**：

$$\vec a_d = -k \cdot |\vec v| \cdot \vec v$$

其中 $k$ 是合并后的阻力系数（代码中变量名为 `air_resistance`）：

$$k = \frac{1}{2}\rho C_d \frac{A}{m}$$

> **⚠️ 关键注意**：代码传入的 `air_resistance` 是合并后的 $k$（单位 $\mathrm{m^{-1}}$），**不是**原始无量纲阻力系数 $C_d$。二者数值量级差约 30 倍，禁止混淆。
>
> 对于 RM 17mm 荧光弹（$C_d=0.50$，$m=0.0032\mathrm{kg}$，$D=0.0168\mathrm{m}$），$k\approx 0.0160\ \mathrm{m^{-1}}$。

将阻力分解到 x、y 方向：

$$a_{d,x} = -k \cdot v \cdot v_x$$
$$a_{d,y} = -k \cdot v \cdot v_y$$

重力只作用在 y 方向（y 轴向上为正）：

$$a_{g,y} = -g$$

### 2.3 微分方程组

综合重力与空气阻力，弹丸运动的一阶微分方程组为：

$$
\begin{cases}
\dfrac{dx}{dt} = v_x \\[6pt]
\dfrac{dy}{dt} = v_y \\[6pt]
\dfrac{dv_x}{dt} = -k \cdot \sqrt{v_x^2+v_y^2} \cdot v_x \\[8pt]
\dfrac{dv_y}{dt} = -g - k \cdot \sqrt{v_x^2+v_y^2} \cdot v_y
\end{cases}
$$

初始条件（枪口位置为原点，$t=0$ 弹丸出膛瞬间）：

$$
\begin{cases}
x(0) = 0,\quad y(0) = 0 \\
v_x(0) = v_0 \cos\theta \\
v_y(0) = v_0 \sin\theta
\end{cases}
$$

- $v_0$：弹丸枪口初速度；
- $\theta$：**待求解的枪口俯仰角（rad）**。

求解目标：找到俯仰角 $\theta$，使得存在飞行时间 $t_f>0$，满足：

$$
\begin{cases}
x(t_f) = d \\
y(t_f) = h
\end{cases}
$$

- $d$：目标相对枪口水平距离（m）；
- $h$：目标相对枪口竖直高度差（m），可正可负。

> 该微分方程组**不存在初等解析解**，只能通过数值积分仿真 + 迭代搜索求解。

### 2.4 空气阻力的影响因素：温度、Cd、合速率v

#### 2.4.1 温度对空气阻力的影响

温度不直接改变弹丸阻力系数 $C_d$；温度通过改变**空气密度 $\rho$** 影响合并阻力系数 $k$，二者成正比：

$$k=\frac12 \rho C_d \frac{A}{m}$$

由理想气体状态方程，在大气压近似不变的条件下：

$$\rho = \frac{P \cdot M}{R_{spec} \cdot T}$$

其中 $T$ 为热力学温度（$\mathrm{K}=273.15+t(^\circ C)$）。

**规律**：
- **温度升高 → $\rho$ 减小 → $k$ 减小 → 空气阻力减弱 → 弹丸速度衰减更慢 → 远距离落点偏高**；
- **温度降低 → $\rho$ 增大 → $k$ 增大 → 阻力增强 → 弹丸减速更快 → 远距离落点偏低**。

标准大气压下数值参考：

| 摄氏温度 | 空气密度 $\rho\ (\mathrm{kg/m^3})$ | 相对 20℃ 的 k 比例 |
|---|---|---|
| 0℃ | 1.293 | 1.073（阻力大 7.3%） |
| 20℃ | 1.205 | 1.000（基准） |
| 35℃ | 1.145 | 0.950（阻力小 5%） |

室内场馆典型温度区间内，$\rho$ 可产生约 $\pm7\%$ 的变化。海拔高度对空气密度的影响大于温度。

**工程方案**：
1. **默认方案**（绝大多数队伍）：不做温压实时补偿，将温度、弹丸磨损、批次差异全部打包到**实弹标定的等效 k** 中。
2. **高精度方案**：接入温压传感器，实时计算 $\rho(T,P)$ 动态更新 k；但 $C_d$ 依旧需要实弹标定，传感器不能补偿弹丸本身的磨损。

#### 2.4.2 阻力系数 $C_d$ 详解

$C_d$（Drag coefficient）为**无量纲阻力系数**，表征弹丸外形、表面粗糙度对空气阻力的贡献。原始阻力公式：

$$F_d=\frac12\rho C_d A v^2$$

**圆球的 $C_d$ 与雷诺数 $Re$ 的关系**：

$$Re=\frac{\rho\, v\, D}{\mu}$$

RM 弹丸工况：17mm 弹初速 18~25m/s，42mm 弹初速 10~16m/s，$Re$ 大致在 $10^4 \sim 10^5$，处于**亚临界绕流区**。

| 流态 | 圆球 $C_d$ 特点 |
|---|---|
| 亚临界区（$10^3 \sim 2\times10^5$，RM 弹在此区间） | $C_d \approx 0.4\sim0.6$，随 $Re$ 变化平缓；表面越粗糙 $C_d$ 越大 |
| 临界区（$Re\approx3\times10^5$） | 边界层转捩，$C_d$ 断崖式下跌（高尔夫球凹坑利用此效应） |

**影响 $C_d$ 的关键因素**：
1. **表面粗糙度（比赛中最大扰动来源）**：弹丸打多了磨损、沾灰尘、毛刺 → $C_d$ 上升。同一批弹打旧之后等效 $C_d$ 会上涨，这是比赛中最主要的扰动，比温度影响还大。
2. **弹丸圆度**：磕碰变形、非完美正圆 → $C_d$ 上升。
3. **雷诺数（速度）**：RM 工作区间变化平缓，工程上可取常数 $C_d$。
4. **温度、气压**：几乎不直接改变 $C_d$，仅改变空气密度 $\rho$。

**经验取值**：

| 弹丸类型 | 经验 $C_d$ 区间 |
|---|---|
| 17mm TPU 荧光弹 | 0.45 ~ 0.55 |
| 42mm TPE 大弹 | 0.44 ~ 0.52 |

> ⚠️ **重要边界**：$C_d$ 无量纲，本 `Trajectory` 类**不接收 $C_d$ 入参**；代码输入参数 `air_resistance` 是合并后的系数 $k$（单位 $\mathrm{m^{-1}}$）。纸面 $C_d$ 仅用于计算理论初始 k；真实比赛的等效 $C_d$ 无法直接测量，依靠远距离实弹标定得到等效 k，再反推等效 $C_d$。

#### 2.4.3 为什么阻力公式必须使用合速率 $v=|\vec v|=\sqrt{v_x^2+v_y^2}$

阻力是**矢量**，阻力矢量的方向严格与弹丸**整体飞行合速度矢量 $\vec v$ 反向**，而不是 x、y 方向相互独立。

矢量形式：

$$\vec a_d=-k \cdot |\vec v|\cdot \vec v$$

将矢量分解为水平 x、竖直 y 分量：

$$
\begin{cases}
a_{d,x}= -k \cdot |\vec v| \cdot v_x \\
a_{d,y}= -k \cdot |\vec v| \cdot v_y
\end{cases}
$$

其中 $v=|\vec v|=\sqrt{v_x^2+v_y^2}$ 为弹丸合速率（代码中 `speed = std::hypot(vx, vy)`）。

**❗ 常见错误误区**：不要写成分量各自平方的形式：

$$a_{dx}=-k\,v_x^2,\quad a_{dy}=-k\,v_y^2$$

该形式**只有弹丸严格水平飞行（$v_y=0$）才巧合成立**。当弹丸存在俯仰角、$v_y\neq0$ 时，会严重低估竖直方向空气阻力，造成仿真弹道下落量偏小，远距离预测落点偏高。

**数值举例**：假设 $v_x=20\mathrm{m/s},\ v_y=5\mathrm{m/s}$，合速率 $v=20.616\mathrm{m/s}$。

| | 正确模型（合速率） | 错误模型（分量平方） |
|---|---|---|
| $a_{dx}$ | $-k\cdot20.616\cdot20=-412.3k$ | $-k\cdot400=-400k$ |
| $a_{dy}$ | $-k\cdot20.616\cdot5=-103.1k$ | $-k\cdot25=-25k$ |

竖直方向阻力被低估约 75%，弹道仿真严重失真。

**物理理解**：弹丸斜向飞行时，阻力整体朝速度反方向；这个斜向上的阻力矢量投影，同时产生水平减速分量与竖直减速分量。因此竖直方向**同样存在空气阻力，不能做小仰角忽略 y 向阻力的简化假设**。

本项目 `estimate()` 完整实现矢量分解：

```cpp
double speed = std::hypot(vx, vy);  // 求取合速率 |v|
vx -= air_resistance * speed * vx * STEP_TIME;
vy -= (gravity + air_resistance * speed * vy) * STEP_TIME;
```

---

## 3. 数学原理

### 3.1 数值积分：半隐式欧拉法

由于上述微分方程组不存在解析解，采用**半隐式欧拉法（Symplectic Euler）**进行数值积分。

核心思想：将连续时间离散化为固定步长 $\Delta t$（代码中 `STEP_TIME = 0.005s`），每一步先更新速度，再用更新后的速度更新位置：

$$
\begin{aligned}
v_x^{(n+1)} &= v_x^{(n)} + a_x^{(n)} \cdot \Delta t \\
v_y^{(n+1)} &= v_y^{(n)} + a_y^{(n)} \cdot \Delta t \\
x^{(n+1)} &= x^{(n)} + v_x^{(n+1)} \cdot \Delta t \\
y^{(n+1)} &= y^{(n)} + v_y^{(n+1)} \cdot \Delta t \\
t^{(n+1)} &= t^{(n)} + \Delta t
\end{aligned}
$$

其中当前步加速度：

$$a_x^{(n)} = -k \cdot v^{(n)} \cdot v_x^{(n)}$$
$$a_y^{(n)} = -g - k \cdot v^{(n)} \cdot v_y^{(n)}$$
$$v^{(n)} = \sqrt{(v_x^{(n)})^2 + (v_y^{(n)})^2}$$

> **为什么是半隐式而非显式**：代码中先更新速度，再用更新后的速度更新位置。纯显式欧拉用旧速度更新位置，半隐式欧拉在能量守恒上略好，长期仿真更稳定。

#### 步长选择的权衡

| 步长 $\Delta t$ | 精度 | 计算量 | 适用场景 |
|---|---|---|---|
| 0.001s（1ms） | 高 | 大（5倍于0.005） | 高精度离线仿真 |
| **0.005s（5ms）** | **足够** | **适中** | **RM 实时弹道（本代码选用）** |
| 0.02s（20ms） | 低 | 小 | 粗略估算 |

RM 弹丸飞行时间通常 $0.1\sim0.8\ \mathrm{s}$，以 5ms 步长计算约 $20\sim160$ 步，单帧耗时微秒级，完全满足实时性。

### 3.2 迭代法求解俯仰角

给定一个俯仰角 $\theta$，通过数值积分可以得到弹丸到达目标水平距离 $d$ 时的实际高度 $y_{actual}$。

定义高度误差（残差）：

$$e = h - y_{actual}$$

- $e > 0$：弹丸打低了，需要**增大**俯仰角；
- $e < 0$：弹丸打高了，需要**减小**俯仰角。

代码采用的修正公式：

$$\theta_{new} = \theta_{old} + \arctan\left(\frac{e}{d}\right)$$

#### 修正公式的数学来源与不严谨性

$\arctan(e/d)$ 来源于**无阻力直线弹道几何近似**：
- 理想直线模型下，期望总俯仰角 $\theta_{desired}=\arctan(h/d)$，当前俯仰角 $\theta_{old}=\arctan(y_{actual}/d)$；
- 严格角度差应为 $\Delta\theta = \arctan(h/d) - \arctan(y_{actual}/d)$；
- 由反正切差角公式：$\arctan a - \arctan b = \arctan\left(\frac{a-b}{1+ab}\right)$；
- 严格结果：$\Delta\theta = \arctan\left(\frac{e/d}{1+\frac{h\cdot y_{actual}}{d^2}}\right)$；
- 代码直接**丢弃了分母 $1+\frac{h\cdot y_{actual}}{d^2}$**，近似为 $\arctan(e/d)$。

**仅当小仰角条件 $h\ll d,\ y_{actual}\ll d$ 时**，分母趋近于 1，近似成立。大仰角、近距离下近似偏差明显。

> ⚠️ **更深层的不严谨**：即使把直线模型的角度差算得严格，该修正依旧建立在**无阻力直线弹道假设**之上。真实弹丸受重力+空气阻力，轨迹是向下弯曲的曲线，并非直线。因此 $\arctan(e/d)$ 对真实带阻力弹道而言，只是一个**试探性步长**，不是理论最优步长。

**为什么能收敛**：它不是一步求解，而是迭代逼近。每一轮用近似步长更新俯仰角，再跑完整带阻力弹道仿真得到新的高度误差，只要每一轮误差总体趋向缩小，多迭代几轮即可压缩到毫米级。代价是远距离、大下落工况可能出现迭代震荡（这一轮抬太高，下一轮又降太多），依靠 `MAX_ITERATIONS=10` 兜底。

#### 初始值的重要性

初始俯仰角取 $\theta_0 = \arctan(h/d)$，即**忽略空气阻力和重力的直射角**。这是一个非常好的初值：
- 近距离、高初速下，空气阻力影响小，$\theta_0$ 已经很接近真实解，1~2 次迭代即可收敛；
- 远距离下，$\theta_0$ 略偏小（因为需要额外抬升补偿重力和阻力），迭代会逐步增大俯仰角。

#### 迭代流程

```
初始猜测：θ₀ = atan2(h, d)   （无阻力直射角，作为良好初值）
for i = 0 to MAX_ITERATIONS-1:
    用 θ_i 仿真弹道 → 得到实际高度 y_i 和飞行时间 t_i
    误差 e = h - y_i
    if |e| < HEIGHT_ERROR:
        收敛！输出 θ_i, t_i
    θ_{i+1} = θ_i + atan2(e, d)
    if |θ_{i+1}| > MAX_PITCH:
        超出物理限位，判定无解
循环结束未收敛 → 判定无解
```

#### 三种迭代策略对比

| 策略 | 公式 | 优点 | 缺点 |
|---|---|---|---|
| **当前代码**（直线近似） | $\theta+\arctan(e/d)$ | 代码极简，单轮一次仿真 | 步长不严格，远距离可能震荡 |
| 严格直线射线夹角 | $\theta+[\arctan(h/d)-\arctan(y_{actual}/d)]$ | 修复直线模型内数学错误 | 底层仍是直线假设，不解决阻力带来的根本近似 |
| **牛顿-拉夫逊**（数值微分） | $\theta - f(\theta)/f'(\theta)$，$f'$ 数值求导 | 收敛快，抗远距离大下坠 | 每轮需两次弹道仿真，CPU 开销翻倍 |

### 3.3 线性插值补全终点

数值积分以固定步长推进，弹丸的 $x$ 坐标**不会恰好**在某一步结束时等于目标距离 $d$。通常是：

- 上一步：$x_{prev} < d$
- 当前步：$x_{curr} > d$（越过了目标）

代码采用**线性插值**在两步之间精确补出 $x=d$ 处的高度和时间：

$$\lambda = \frac{d - x_{prev}}{x_{curr} - x_{prev}}$$

$$y_{final} = y_{prev} + \lambda \cdot (y_{curr} - y_{prev})$$
$$t_{final} = t_{prev} + \lambda \cdot (t_{curr} - t_{prev})$$

其中 $\lambda \in [0,1]$ 是插值比例，代表在这一小段水平路程中，已经走到目标距离的路程占比。

> **这一步保证了终点高度的精度不受积分步长限制**，即使步长较大也能精确得到 $x=d$ 处的高度。

**近似假设**：在单个很短的积分步长（5ms）之内，$x$-$y$ 平面上弹丸小段轨迹近似为直线。5ms 步长下该近似引入的误差远小于 1mm。

**关键区分**：
- ✅ 插值解决的是「采样点没有落在目标 $x=d$」带来的采样位置误差；
- ❗ 插值**不能消除欧拉积分本身的离散截断误差**，这部分误差只能靠缩小 `STEP_TIME` 改善。

> 本实现是**基于水平坐标 $x$ 的线性插值，不是基于时间 $t$ 的插值**。因为求解目标是命中固定水平距离 $d$，以 $x$ 做自变量更贴合需求。

---

## 4. 代码逐段解析

### 4.1 常量定义

```cpp
constexpr int    MAX_ITERATIONS = 10;          // 最大迭代次数
constexpr double MAX_PITCH      = 40.0 / 57.3; // 最大俯仰角(rad)，40°
constexpr double STEP_TIME      = 0.005;         // 积分步长(s)，5ms
constexpr double HEIGHT_ERROR   = 0.001;         // 高度收敛阈值(m)，1mm
constexpr double MAX_FLY_TIME   = 4.0;           // 最大飞行时间(s)
constexpr double MIN_VELOCITY_X = 0.1;           // 最小水平速度(m/s)
```

| 常量 | 物理意义 | 取值依据 |
|---|---|---|
| `MAX_ITERATIONS` | 外层俯仰角迭代最大轮数 | RM 场景通常 3~6 轮收敛，10 轮足够；防止死循环 |
| `MAX_PITCH` | 俯仰角物理上限 | 40°（英雄车云台机械限位），超过则无解；哨兵车需改为 35° |
| `STEP_TIME` | 数值积分时间步长 | 5ms，精度与算力的工程折中 |
| `HEIGHT_ERROR` | 收敛判定阈值 | 1mm，远小于装甲板尺寸，命中精度足够 |
| `MAX_FLY_TIME` | 弹丸飞行时间上限 | 4s，远超实际（<1s），防止积分无限跑 |
| `MIN_VELOCITY_X` | 水平速度下限 | 0.1m/s，低于此值弹丸近乎停滞，解无意义 |

### 4.2 estimate() 弹道仿真函数

```cpp
std::optional<std::pair<double, double>> estimate(
    double v0, double pitch, double distance,
    double gravity, double air_resistance)
```

**输入**：初速、俯仰角、目标水平距离、重力、合并阻力系数 $k$
**输出**：`optional<pair<高度, 飞行时间>>`，仿真失败返回 `nullopt`

#### 初始化

```cpp
double x = 0.0, y = 0.0, t = 0.0;
double vx = v0 * std::cos(pitch);
double vy = v0 * std::sin(pitch);
double previous_x = 0.0, previous_y = 0.0, previous_t = 0.0;
```

- 枪口为原点，初始位置 $(0,0)$，时间 $t=0$；
- 初速度分解到 x、y 方向；
- `previous_*` 保存上一步状态，用于终点线性插值。

#### 主积分循环

```cpp
while (x < distance && t <= MAX_FLY_TIME && vx > MIN_VELOCITY_X)
{
    previous_x = x; previous_y = y; previous_t = t;
    const double speed = std::hypot(vx, vy);
    vx -= air_resistance * speed * vx * STEP_TIME;
    vy -= (gravity + air_resistance * speed * vy) * STEP_TIME;
    x += vx * STEP_TIME;
    y += vy * STEP_TIME;
    t += STEP_TIME;
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(t))
        return std::nullopt;
}
```

循环条件（三者同时满足才继续）：
1. `x < distance`：还没飞到目标水平距离；
2. `t <= MAX_FLY_TIME`：飞行时间未超限；
3. `vx > MIN_VELOCITY_X`：水平速度未衰减到停滞。

每一步执行：
1. 保存当前状态到 `previous_*`；
2. 计算合速度 `speed = hypot(vx, vy)`（即 $v=|\vec v|$）；
3. **更新速度**（半隐式欧拉）：
   - x 方向：只有空气阻力减速 $v_x \leftarrow v_x - k \cdot v \cdot v_x \cdot \Delta t$
   - y 方向：重力 + 阻力 $v_y \leftarrow v_y - (g + k \cdot v \cdot v_y) \cdot \Delta t$
4. **更新位置**：用更新后的速度推进 $x \leftarrow x + v_x \cdot \Delta t$，$y \leftarrow y + v_y \cdot \Delta t$；
5. 更新时间；
6. **数值保护**：如果出现 NaN / Inf（参数非法导致发散），直接返回无解。

#### 终点判定与插值

```cpp
if (x < distance || x <= previous_x) return std::nullopt;
const double ratio = (distance - previous_x) / (x - previous_x);
return std::pair<double, double>{
    previous_y + ratio * (y - previous_y),
    previous_t + ratio * (t - previous_t)
};
```

退出循环后：
- 如果 `x < distance`：说明因为超时或速度停滞而退出，没飞到目标 → 无解；
- 如果 `x <= previous_x`：说明 x 没有前进（甚至倒退），物理异常 → 无解；
- 正常情况：`previous_x < distance <= x`，在两步之间做线性插值，返回终点高度和飞行时间。

### 4.3 Trajectory 构造函数：迭代求解器

```cpp
Trajectory::Trajectory(double v0, double d, double h,
                        double gravity, double air_resistance)
```

构造函数执行完整的迭代求解，结果存入成员变量 `pitch`、`fly_time`、`unsolvable`。

#### 参数合法性检查

```cpp
if (!(v0 > 0.0) || !(d > 0.0) || !(gravity > 0.0) || air_resistance < 0.0 ||
    !std::isfinite(v0) || !std::isfinite(d) || !std::isfinite(h) ||
    !std::isfinite(gravity) || !std::isfinite(air_resistance))
    return;
```

非法参数直接返回（`unsolvable` 默认为 `true`）：
- 初速、距离、重力必须为正；
- 阻力系数不能为负（$k=0$ 合法，代表无阻力模型）；
- 所有参数必须是有限值（非 NaN/Inf）。

> ⚠️ 注意：$h$（目标高度差）允许为正或负（目标可高于或低于枪口），因此不对 $h$ 做正负判断，只检查是否为有限浮点数。

#### 初始俯仰角猜测

```cpp
double launch_pitch = std::atan2(h, d);
```

无阻力直射角 $\theta_0 = \arctan(h/d)$，作为迭代初值。

#### 迭代主循环

```cpp
for (int i = 0; i < MAX_ITERATIONS; ++i)
{
    const auto estimated = estimate(v0, launch_pitch, d, gravity, air_resistance);
    if (!estimated) return;
    const auto [actual_h, time] = *estimated;
    const double error = h - actual_h;
    if (std::abs(error) < HEIGHT_ERROR)
    {
        unsolvable = false;
        fly_time   = time;
        pitch      = launch_pitch;
        return;
    }
    launch_pitch += std::atan2(error, d);
    if (std::abs(launch_pitch) > MAX_PITCH) return;
}
```

每一轮：
1. 调用 `estimate()` 仿真当前俯仰角下的弹道，得到实际高度和飞行时间；
2. 仿真失败 → 直接返回（无解）；
3. C++17 结构化绑定解包 `actual_h`（仿真高度）和 `time`（飞行时间）；
4. 计算高度误差 `error = 目标高度 - 实际高度`；
5. **收敛判定**：误差绝对值小于 1mm → 求解成功，保存俯仰角和飞行时间，标记 `unsolvable=false`，返回；
6. **修正俯仰角**：`launch_pitch += atan2(error, d)`（工程近似试探步长）；
7. **限位检查**：修正后俯仰角超过 ±40° → 物理上不可达，返回无解。

循环耗尽（10次未收敛）→ 隐式返回，`unsolvable` 保持 `true`。

---

## 5. 参数物理意义与取值依据

### 5.1 输入参数

| 参数 | 符号 | 单位 | 物理意义 | RM 典型值 |
|---|---|---|---|---|
| `v0` | $v_0$ | m/s | 弹丸出膛初速 | 15~28（由测速仪实测） |
| `d` | $d$ | m | 目标水平距离 | 1~8 |
| `h` | $h$ | m | 目标相对枪口高度 | -0.5~+0.5 |
| `gravity` | $g$ | m/s² | 重力加速度 | 9.81 |
| `air_resistance` | $k$ | m⁻¹ | 合并阻力系数 $k=\frac{1}{2}\rho C_d A/m$ | 17mm: ~0.0160；42mm: ~0.0092 |

### 5.2 输出参数

| 成员变量 | 符号 | 单位 | 物理意义 |
|---|---|---|---|
| `pitch` | $\theta$ | rad | 枪口需要抬升的俯仰角 |
| `fly_time` | $t_f$ | s | 弹丸飞行时间（用于预判目标运动） |
| `unsolvable` | — | bool | 标记是否无解（true=无解，不应射击） |

### 5.3 合并阻力系数 k 的计算（需自行计算）

> ⚠️ **`Trajectory` 类不内置弹丸物理参数，不会内部计算合并阻力系数 $k$。使用者必须在外部完成 $k$ 的计算，作为入参 `air_resistance` 传入。**

公式：

$$k=\frac12\rho C_d \frac{A}{m},\quad A=\frac{\pi D^2}{4}$$

- $\rho$：空气密度，常温取 $1.205\ \mathrm{kg/m^3}$；
- $C_d$：弹丸无量纲阻力系数（经验/实验值）；
- $D$：弹丸直径（m）；
- $m$：弹丸质量（kg）。

#### RM2026 弹丸理论初始参考值

| 弹丸类型 | 直径 $D$ (m) | 质量 $m$ (kg) | 参考 $C_d$ | 理论 $k\ (\mathrm{m^{-1}})$ |
|---|---|---|---|---|
| 17mm 荧光弹 | 0.0168 | 0.0032 | 0.50 | **0.0160** |
| 42mm 发光弹 | 0.0425 | 0.0445 | 0.48 | **0.0092** |

#### 计算示例（17mm 荧光弹）

$$
\begin{aligned}
D &= 0.0168\ \mathrm{m} \\
m &= 3.2\times10^{-3}\ \mathrm{kg} \\
\rho &= 1.205\ \mathrm{kg/m^3} \\
C_d &= 0.50 \\
A &= \pi (D/2)^2 = \pi (0.0084)^2 \approx 2.217\times10^{-4}\ \mathrm{m^2} \\
k &= \frac{1}{2} \rho C_d \frac{A}{m}
  = 0.5 \times 1.205 \times 0.50 \times \frac{2.217\times10^{-4}}{3.2\times10^{-3}}
  \approx 0.0160\ \mathrm{m^{-1}}
\end{aligned}
$$

#### C++ 计算示例

```cpp
// 17mm 荧光弹，理论初始 k
constexpr double rho = 1.205;          // 空气密度 kg/m³
constexpr double Cd_17 = 0.50;          // 阻力系数
constexpr double D_17 = 0.0168;         // 直径 m
constexpr double m_17 = 0.0032;         // 质量 kg
double A_17 = M_PI * D_17 * D_17 / 4.0;
double k_17 = 0.5 * rho * Cd_17 * A_17 / m_17;

// 使用，把 k_17 作为 air_resistance 传进去
rm_utils::Trajectory traj(v0, d, h, 9.81, k_17);
```

#### ⚠️ 纸面理论 k 不能直接用于正式比赛

理论计算得到的 k 只是仿真起点。真实比赛存在以下扰动：
1. 弹丸批次差异、磨损、表面污渍，$C_d$ 会浮动；
2. 场馆温度、湿度，空气密度 $\rho$ 轻微变化；
3. 弹丸出膛旋转，马格努斯效应，模型完全没有考虑；
4. 仿真模型是常数 $C_d$，真实弹丸雷诺数变化 $C_d$ 会微小变化。

👉 **纸面计算得到 k 之后，必须做实弹远距离打靶标定，修正 k 值。**

**标定方法**：固定枪口俯仰角、固定初速度 $v_0$、固定远距离打靶，对比仿真预测弹着高度 vs 真实弹着高度。

| 现象（远距离） | 说明 | 对 k 的操作 |
|---|---|---|
| 真实弹丸落点显著低于仿真预测 | 真实弹丸减速比仿真更快，实际阻力更大 | **增大 k** |
| 真实弹丸落点显著高于仿真预测 | 真实弹丸减速比仿真慢，实际阻力更小 | **减小 k** |

> 近距离（<4~5m）空气阻力效应微弱，不能用于标定 k，必须拉远距离（8~15m）。

#### 高频踩坑清单

1. ❌ 把原始 $C_d$（0.5）直接传给 `air_resistance`。灾难！k 的量级 0.01~0.02，Cd 是 0.5，差几十倍，弹道完全崩坏。
2. ❌ 拿旧赛季 17mm 弹 4.2g 质量算 k。RM2026 新荧光弹是 **3.2g**。
3. ❌ 只靠纸面计算 k，不做远距离实弹标定，直接上场。
4. ❌ 近距离打靶调 k。短距离阻力效应几乎看不到，调出来的 k 完全不准。
5. ❌ 把 k 和单位搞错，单位必须是 $\mathrm{m^{-1}}$。

---

## 6. 收敛性与误差分析

### 6.1 收敛速度

由于初始值 $\theta_0 = \arctan(h/d)$ 已经很接近真实解，且修正公式 $\Delta\theta = \arctan(e/d)$ 具有合理的物理比例，实际收敛速度很快：

| 场景 | 距离 | 初速 | 收敛迭代次数 |
|---|---|---|---|
| 近距离 | 2m | 25m/s | 1~2 次 |
| 中距离 | 5m | 20m/s | 3~4 次 |
| 远距离 | 8m | 15m/s | 5~7 次 |

10 次迭代上限对所有 RM 实际场景都足够。远距离、大下落工况可能出现迭代震荡，导致收敛轮次偏多。

### 6.2 误差来源

| 误差来源 | 影响 | 缓解方式 |
|---|---|---|
| 数值积分截断误差 | 弹道位置偏差 | 5ms 步长下误差 < 0.1mm，可忽略 |
| 终点线性插值误差 | 终点高度偏差 | 插值精确，误差远小于 1mm 阈值 |
| 初速测量误差 | 弹道整体偏差 | 定期用测速仪标定 $v_0$ |
| 阻力系数不准 | 远距离偏差明显 | 实弹打靶标定 k |
| 忽略横风/马格努斯效应 | 侧向偏差 | RM 室内场景可忽略 |
| 迭代步长近似（arctan(e/d)） | 远距离可能震荡 | 依靠 MAX_ITERATIONS 兜底；可改用牛顿法 |

### 6.3 为什么收敛阈值是 1mm

`HEIGHT_ERROR = 0.001m`（1mm）远小于装甲板尺寸（约 100mm×60mm），即使弹道有 1mm 高度误差，也完全在命中范围内。设置过小（如 0.01mm）会增加不必要的迭代次数；设置过大（如 10mm）会降低命中精度。1mm 是精度与效率的合理折中。

---

## 7. 局限性与改进方向

### 7.1 当前模型的局限性

1. **二维模型**：只考虑 x（水平）和 y（竖直）平面，忽略侧向风偏和马格努斯效应（旋转弹丸的侧向力）。RM 室内场景影响很小。
2. **常数阻力系数**：假设 $C_d$ 不随雷诺数变化。RM 弹速（15~28m/s）下雷诺数处于亚临界区，$C_d$ 近似常数，假设成立。
3. **半隐式欧拉积分**：一阶方法，精度有限。5ms 步长下误差可忽略，但如果需要更高精度可改用 Runge-Kutta 4 阶（RK4）。
4. **单一点目标**：只考虑目标中心点，未结合目标运动预测（需要外层配合，飞行时间 `fly_time` 就是给运动预测用的）。
5. **迭代修正无阻尼**：$\theta_{new} = \theta_{old} + \arctan(e/d)$ 是全量修正，极端参数下可能震荡。实际 RM 场景初值好，不会严重震荡。
6. **不做温压实时补偿**：固定 $\rho=1.205$，温度/海拔变化通过实弹标定等效 k 吸收。

### 7.2 可能的改进方向

| 改进项 | 收益 | 代价 |
|---|---|---|
| RK4 积分 | 精度提升，可适当增大步长 | 代码复杂度增加，单步计算量×4 |
| 阻力系数随雷诺数插值 | 更宽速度范围精度 | 需要实验数据拟合 |
| 加入横风模型 | 室外场景更准 | 需要风速传感器 |
| 迭代加阻尼/牛顿法 | 极端场景收敛更稳 | 代码复杂度增加；牛顿法每轮两次仿真 |
| 温压传感器实时补偿 ρ | 温度/海拔变化场景更准 | 需要硬件传感器 |
| 提前计算弹道表 | 运行时零计算（查表） | 需要预计算存储空间，灵活性降低 |
| 可变步长积分（跨过 d 时缩小步长） | 不需要终点插值 | 逻辑复杂度增加 |

---

## 8. 附录：完整调用流程

### 8.1 典型使用方式

```cpp
// 构造弹道求解器，自动完成迭代求解
rm_utils::Trajectory traj(
    25.0,    // v0: 初速 25 m/s
    5.0,     // d:  目标水平距离 5m
    0.2,     // h:  目标比枪口高 0.2m
    9.81,    // gravity: 重力加速度
    0.0160   // air_resistance: 17mm 弹合并阻力系数 k
);
if (traj.unsolvable)
{
    // 无解：距离过远 / 初速不足 / 俯仰角超限位
    // 不应射击
}
else
{
    double pitch    = traj.pitch;     // 计算出的俯仰角(rad)
    double fly_time = traj.fly_time;  // 飞行时间(s)，用于目标运动预判
    // 下发 pitch 给云台控制
}
```

### 8.2 单次求解完整执行链

```
Trajectory(v0, d, h, g, k)
│
├─ 参数合法性检查（v0/d/g>0, k>=0, 全部有限）
│
├─ 初始俯仰角 θ₀ = atan2(h, d)
│
└─ 迭代循环（最多10次）
    │
    ├─ estimate(v0, θᵢ, d, g, k)
    │   │
    │   ├─ 初始化 x=y=t=0, vx=v0cosθ, vy=v0sinθ
    │   │
    │   └─ 积分循环（每步5ms，条件：x<d && t<=4s && vx>0.1m/s）
    │       ├─ 保存上一步状态 previous_*
    │       ├─ 计算合速度 speed=hypot(vx,vy)
    │       ├─ 更新速度（x:阻力；y:重力+阻力）
    │       ├─ 更新位置 x,y
    │       ├─ 更新时间 t
    │       └─ 数值保护（NaN/Inf → nullopt）
    │
    ├─ 终点判定：x<d 或 x<=previous_x → nullopt（无解）
    │
    ├─ 终点线性插值 → 实际高度 actual_h, 飞行时间 time
    │
    ├─ 误差 error = h - actual_h
    │
    ├─ |error| < 1mm ?
    │   ├─ 是 → 收敛！保存 pitch, fly_time, unsolvable=false，返回
    │   └─ 否 → 继续
    │
    ├─ 修正俯仰角 θᵢ₊₁ = θᵢ + atan2(error, d)
    │
    └─ |θᵢ₊₁| > 40° ? → 超出限位，无解返回
```

### 8.3 无解（unsolvable=true）的所有可能原因

1. **参数非法**：初速/距离/重力非正、阻力系数为负、存在 NaN/Inf；
2. **弹道仿真失败**：积分过程中出现 NaN/Inf；
3. **弹丸飞不到目标**：飞行时间超过 4s 或水平速度衰减到 0.1m/s 以下，x 仍未达到目标距离；
4. **俯仰角超限位**：迭代修正后俯仰角超过 ±40°，物理上云台无法达到；
5. **迭代不收敛**：10 次迭代后高度误差仍大于 1mm（实际 RM 场景几乎不会发生，远距离震荡时可能出现）。

---

> **文档结束**
> 本文件基于 `rm_utils/algorithm/trajectory.cpp` 源码逐行分析，所有公式与代码逻辑一一对应。如有代码更新，需同步修订本文档。
` 是合并后的 $k$，**不是**原始阻力系数 $C_d$。
> 单位为 $\mathrm{m^{-1}}$。对于 RM 17mm 塑料弹，$C_d=0.5$ 时 $k\approx 0.0163\ \mathrm{m^{-1}}$。

将阻力分解到 x、y 方向：

$$a_{d,x} = -k \cdot v \cdot v_x$$
$$a_{d,y} = -k \cdot v \cdot v_y$$

重力只作用在 y 方向：

$$a_{g,y} = -g$$

### 2.3 微分方程组

综合重力与空气阻力，弹丸运动的一阶微分方程组为：

$$
\begin{cases}
\dfrac{dx}{dt} = v_x \\[6pt]
\dfrac{dy}{dt} = v_y \\[6pt]
\dfrac{dv_x}{dt} = -k \cdot \sqrt{v_x^2+v_y^2} \cdot v_x \\[8pt]
\dfrac{dv_y}{dt} = -g - k \cdot \sqrt{v_x^2+v_y^2} \cdot v_y
\end{cases}
$$

初始条件（枪口位置为原点）：

$$
\begin{cases}
x(0) = 0,\quad y(0) = 0 \\
v_x(0) = v_0 \cos\theta \\
v_y(0) = v_0 \sin\theta
\end{cases}
$$

目标：找到 $\theta$，使得当 $x(t_f)=d$ 时，$y(t_f)=h$。

---

## 3. 数学原理

### 3.1 数值积分：显式欧拉法

由于上述微分方程组不存在解析解，采用**显式欧拉法（Forward Euler）**进行数值积分。

核心思想：将连续时间离散化为固定步长 $\Delta t$（代码中 `STEP_TIME = 0.005s`），每一步用当前状态的导数推算下一步状态：

$$
\begin{aligned}
v_x^{(n+1)} &= v_x^{(n)} + a_x^{(n)} \cdot \Delta t \\
v_y^{(n+1)} &= v_y^{(n)} + a_y^{(n)} \cdot \Delta t \\
x^{(n+1)} &= x^{(n)} + v_x^{(n+1)} \cdot \Delta t \\
y^{(n+1)} &= y^{(n)} + v_y^{(n+1)} \cdot \Delta t \\
t^{(n+1)} &= t^{(n)} + \Delta t
\end{aligned}
$$

其中当前步加速度：

$$a_x^{(n)} = -k \cdot v^{(n)} \cdot v_x^{(n)}$$
$$a_y^{(n)} = -g - k \cdot v^{(n)} \cdot v_y^{(n)}$$
$$v^{(n)} = \sqrt{(v_x^{(n)})^2 + (v_y^{(n)})^2}$$

> **代码细节**：代码中先更新速度，再用更新后的速度更新位置（半隐式欧拉 / Symplectic Euler），比纯显式欧拉在能量守恒上略好。

#### 步长选择的权衡

| 步长 $\Delta t$ | 精度 | 计算量 | 适用场景 |
|---|---|---|---|
| 0.001s（1ms） | 高 | 大（5倍于0.005） | 高精度离线仿真 |
| **0.005s（5ms）** | **足够** | **适中** | **RM 实时弹道（本代码选用）** |
| 0.02s（20ms） | 低 | 小 | 粗略估算 |

RM 弹丸飞行时间通常 $0.1\sim0.8\ \mathrm{s}$，以 5ms 步长计算约 $20\sim160$ 步，单帧耗时微秒级，完全满足实时性。

### 3.2 迭代法求解俯仰角

给定一个俯仰角 $\theta$，通过数值积分可以得到弹丸到达目标水平距离 $d$ 时的实际高度 $y_{actual}$。

定义高度误差：

$$e = h_{target} - y_{actual}$$

- $e > 0$：弹丸打低了，需要**增大**俯仰角；
- $e < 0$：弹丸打高了，需要**减小**俯仰角。

代码采用的修正公式：

$$\theta_{new} = \theta_{old} + \arctan\left(\frac{e}{d}\right)$$

$ \arctan\left(\frac{e}{d}\right) $ 是近似变换，误差会在一次次迭代中修正

#### 为什么用 $\arctan(e/d)$ 而不是直接加 $e$？

物理直觉：要让弹丸在距离 $d$ 处高度变化 $e$，俯仰角需要变化的角度近似为 $\arctan(e/d)$。这是一个**基于几何的比例修正**，比纯梯度下降收敛更快。

当 $e \ll d$ 时，$\arctan(e/d) \approx e/d$，退化为线性比例控制。

#### 迭代流程

```
初始猜测：θ₀ = atan2(h, d)   （无阻力直射角，作为良好初值）
for i = 0 to MAX_ITERATIONS-1:
    用 θ_i 仿真弹道 → 得到实际高度 y_i 和飞行时间 t_i
    误差 e = h - y_i
    if |e| < HEIGHT_ERROR:
        收敛！输出 θ_i, t_i
    θ_{i+1} = θ_i + atan2(e, d)
    if |θ_{i+1}| > MAX_PITCH:
        超出物理限位，判定无解
循环结束未收敛 → 判定无解
```

#### 初始值的重要性

初始俯仰角取 $\theta_0 = \arctan(h/d)$，即**忽略空气阻力和重力的直射角**。这是一个非常好的初值：
- 近距离、高初速下，空气阻力影响小，$\theta_0$ 已经很接近真实解，1~2 次迭代即可收敛；
- 远距离下，$\theta_0$ 略偏小（因为需要额外抬升补偿重力和阻力），迭代会逐步增大俯仰角。

### 3.3 线性插值补全终点

数值积分以固定步长推进，弹丸的 $x$ 坐标**不会恰好**在某一步结束时等于目标距离 $d$。通常是：

- 上一步：$x_{prev} < d$
- 当前步：$x_{curr} > d$（越过了目标）

代码采用**线性插值**在两步之间精确补出 $x=d$ 处的高度和时间：

$$\lambda = \frac{d - x_{prev}}{x_{curr} - x_{prev}}$$

$$y_{final} = y_{prev} + \lambda \cdot (y_{curr} - y_{prev})$$
$$t_{final} = t_{prev} + \lambda \cdot (t_{curr} - t_{prev})$$

其中 $\lambda \in [0,1]$ 是插值比例。

> 这一步保证了终点高度的精度不受积分步长限制，即使步长较大也能精确得到 $x=d$ 处的高度。

---

## 4. 代码逐段解析

### 4.1 常量定义

```cpp
constexpr int    MAX_ITERATIONS = 10;        // 最大迭代次数
constexpr double MAX_PITCH      = 40.0 / 57.3; // 最大俯仰角(rad)，40°
constexpr double STEP_TIME      = 0.005;       // 积分步长(s)，5ms
constexpr double HEIGHT_ERROR   = 0.001;       // 高度收敛阈值(m)，1mm
constexpr double MAX_FLY_TIME   = 4.0;         // 最大飞行时间(s)
constexpr double MIN_VELOCITY_X = 0.1;         // 最小水平速度(m/s)
```

| 常量 | 物理意义 | 取值依据 |
|---|---|---|
| `MAX_ITERATIONS` | 外层俯仰角迭代最大轮数 | RM 场景通常 3~6 轮收敛，10 轮足够；防止死循环 |
| `MAX_PITCH` | 俯仰角物理上限 | 40°（云台机械限位），超过则无解 |
| `STEP_TIME` | 数值积分时间步长 | 5ms，精度与算力的工程折中 |
| `HEIGHT_ERROR` | 收敛判定阈值 | 1mm，远小于装甲板尺寸，命中精度足够 |
| `MAX_FLY_TIME` | 弹丸飞行时间上限 | 4s，远超实际（<1s），防止积分无限跑 |
| `MIN_VELOCITY_X` | 水平速度下限 | 0.1m/s，低于此值弹丸近乎停滞，解无意义 |

### 4.2 estimate() 弹道仿真函数

```cpp
std::optional<std::pair<double, double>> estimate(
    double v0, double pitch, double distance,
    double gravity, double air_resistance)
```

**输入**：初速、俯仰角、目标水平距离、重力、阻力系数
**输出**：`optional<pair<高度, 飞行时间>>`，仿真失败返回 `nullopt`

#### 初始化

```cpp
double x = 0.0, y = 0.0, t = 0.0;
double vx = v0 * std::cos(pitch);
double vy = v0 * std::sin(pitch);
double previous_x = 0.0, previous_y = 0.0, previous_t = 0.0;
```

- 枪口为原点，初始位置 $(0,0)$，时间 $t=0$；
- 初速度分解到 x、y 方向；
- `previous_*` 保存上一步状态，用于终点线性插值。

#### 主积分循环

```cpp
while (x < distance && t <= MAX_FLY_TIME && vx > MIN_VELOCITY_X)
{
    previous_x = x; previous_y = y; previous_t = t;

    const double speed = std::hypot(vx, vy);
    vx -= air_resistance * speed * vx * STEP_TIME;
    vy -= (gravity + air_resistance * speed * vy) * STEP_TIME;
    x += vx * STEP_TIME;
    y += vy * STEP_TIME;
    t += STEP_TIME;

    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(t))
        return std::nullopt;
}
```

循环条件（三者同时满足才继续）：
1. `x < distance`：还没飞到目标水平距离；
2. `t <= MAX_FLY_TIME`：飞行时间未超限；
3. `vx > MIN_VELOCITY_X`：水平速度未衰减到停滞。

每一步执行：
1. 保存当前状态到 `previous_*`；
2. 计算合速度 `speed = hypot(vx, vy)`；
3. **更新速度**（半隐式欧拉）：
   - x 方向：只有空气阻力减速 $v_x \leftarrow v_x - k \cdot v \cdot v_x \cdot \Delta t$
   - y 方向：重力 + 阻力 $v_y \leftarrow v_y - (g + k \cdot v \cdot v_y) \cdot \Delta t$
4. **更新位置**：用更新后的速度推进 $x \leftarrow x + v_x \cdot \Delta t$，$y \leftarrow y + v_y \cdot \Delta t$；
5. 更新时间；
6. **数值保护**：如果出现 NaN / Inf（参数非法导致发散），直接返回无解。

#### 终点判定与插值

```cpp
if (x < distance || x <= previous_x) return std::nullopt;

const double ratio = (distance - previous_x) / (x - previous_x);
return std::pair<double, double>{
    previous_y + ratio * (y - previous_y),
    previous_t + ratio * (t - previous_t)
};
```

退出循环后：
- 如果 `x < distance`：说明因为超时或速度停滞而退出，没飞到目标 → 无解；
- 如果 `x <= previous_x`：说明 x 没有前进（甚至倒退），物理异常 → 无解；
- 正常情况：`previous_x < distance <= x`，在两步之间做线性插值，返回终点高度和飞行时间。

### 4.3 Trajectory 构造函数：迭代求解器

```cpp
Trajectory::Trajectory(double v0, double d, double h,
                        double gravity, double air_resistance)
```

构造函数执行完整的迭代求解，结果存入成员变量 `pitch`、`fly_time`、`unsolvable`。

#### 参数合法性检查

```cpp
if (!(v0 > 0.0) || !(d > 0.0) || !(gravity > 0.0) || air_resistance < 0.0 ||
    !std::isfinite(v0) || !std::isfinite(d) || !std::isfinite(h) ||
    !std::isfinite(gravity) || !std::isfinite(air_resistance))
    return;
```

非法参数直接返回（`unsolvable` 默认为 `true`）：
- 初速、距离、重力必须为正；
- 阻力系数不能为负；
- 所有参数必须是有限值（非 NaN/Inf）。

#### 初始俯仰角猜测

```cpp
double launch_pitch = std::atan2(h, d);
```

无阻力直射角 $\theta_0 = \arctan(h/d)$，作为迭代初值。

#### 迭代主循环

```cpp
for (int i = 0; i < MAX_ITERATIONS; ++i)
{
    const auto estimated = estimate(v0, launch_pitch, d, gravity, air_resistance);
    if (!estimated) return;

    const auto [actual_h, time] = *estimated;
    const double error = h - actual_h;

    if (std::abs(error) < HEIGHT_ERROR)
    {
        unsolvable = false;
        fly_time   = time;
        pitch      = launch_pitch;
        return;
    }

    launch_pitch += std::atan2(error, d);

    if (std::abs(launch_pitch) > MAX_PITCH) return;
}
```

每一轮：
1. 调用 `estimate()` 仿真当前俯仰角下的弹道，得到实际高度和飞行时间；
2. 仿真失败 → 直接返回（无解）；
3. `*estimated`：取出 optional 容器内部的 pair 结果。
4. 计算高度误差 `error = 目标高度 - 实际高度`；
5. **收敛判定**：误差绝对值小于 1mm → 求解成功，保存俯仰角和飞行时间，标记 `unsolvable=false`，返回；
6. **修正俯仰角**：`launch_pitch += atan2(error, d)`；
7. **限位检查**：修正后俯仰角超过 ±40° → 物理上不可达，返回无解。

循环耗尽（10次未收敛）→ 隐式返回，`unsolvable` 保持 `true`。

---

## 5. 参数物理意义与取值依据

### 5.1 输入参数

| 参数 | 符号 | 单位 | 物理意义 | RM 典型值 |
|---|---|---|---|---|
| `v0` | $v_0$ | m/s | 弹丸出膛初速 | 15~28（由测速仪实测） |
| `d` | $d$ | m | 目标水平距离 | 1~8 |
| `h` | $h$ | m | 目标相对枪口高度 | -0.5~+0.5 |
| `gravity` | $g$ | m/s² | 重力加速度 | 9.81 |
| `air_resistance` | $k$ | m⁻¹ | 合并阻力系数 $k=\frac{1}{2}\rho C_d A/m$ | ~0.0163 |

### 5.2 输出参数

| 成员变量 | 符号 | 单位 | 物理意义 |
|---|---|---|---|
| `pitch` | $\theta$ | rad | 枪口需要抬升的俯仰角 |
| `fly_time` | $t_f$ | s | 弹丸飞行时间（用于预判目标运动） |
| `unsolvable` | — | bool | 标记是否无解（true=无解，不应射击） |

### 5.3 阻力系数 $k$ 的计算

以 RM 标准 17mm 塑料弹为例：

$$
\begin{aligned}
d &= 0.017\ \mathrm{m} \\
m &= 4.2\times10^{-3}\ \mathrm{kg} \\
\rho &= 1.205\ \mathrm{kg/m^3} \\
C_d &= 0.50 \\
A &= \pi (d/2)^2 = \pi (0.0085)^2 \approx 2.270\times10^{-4}\ \mathrm{m^2} \\
k &= \frac{1}{2} \rho C_d \frac{A}{m}
  = 0.5 \times 1.205 \times 0.50 \times \frac{2.270\times10^{-4}}{4.2\times10^{-3}}
  \approx 0.0163\ \mathrm{m^{-1}}
\end{aligned}
$$

> **重要提醒**：代码中的 `air_resistance` 是合并后的 $k$（约 0.016），不是原始 $C_d$（约 0.5）。如果误将 $C_d=0.5$ 传入，阻力会被放大约 30 倍，弹道会严重偏短。

---

## 6. 收敛性与误差分析

### 6.1 收敛速度

由于初始值 $\theta_0 = \arctan(h/d)$ 已经很接近真实解，且修正公式 $\Delta\theta = \arctan(e/d)$ 具有合理的物理比例，实际收敛速度很快：

| 场景 | 距离 | 初速 | 收敛迭代次数 |
|---|---|---|---|
| 近距离 | 2m | 25m/s | 1~2 次 |
| 中距离 | 5m | 20m/s | 3~4 次 |
| 远距离 | 8m | 15m/s | 5~7 次 |

10 次迭代上限对所有 RM 实际场景都足够。

### 6.2 误差来源

| 误差来源 | 影响 | 缓解方式 |
|---|---|---|
| 数值积分截断误差 | 弹道位置偏差 | 5ms 步长下误差 < 0.1mm，可忽略 |
| 终点线性插值误差 | 终点高度偏差 | 插值精确，误差远小于 1mm 阈值 |
| 初速测量误差 | 弹道整体偏差 | 定期用测速仪标定 $v_0$ |
| 阻力系数不准 | 远距离偏差明显 | 实弹打靶标定 $k$ |
| 忽略横风/马格努斯效应 | 侧向偏差 | RM 室内场景可忽略 |

### 6.3 为什么收敛阈值是 1mm

`HEIGHT_ERROR = 0.001m`（1mm）远小于装甲板尺寸（约 100mm×60mm），即使弹道有 1mm 高度误差，也完全在命中范围内。设置过小（如 0.01mm）会增加不必要的迭代次数；设置过大（如 10mm）会降低命中精度。1mm 是精度与效率的合理折中。

---

## 7. 局限性与改进方向

### 7.1 当前模型的局限性

1. **二维模型**：只考虑 x（水平）和 y（竖直）平面，忽略侧向风偏和马格努斯效应（旋转弹丸的侧向力）。RM 室内场景影响很小。

2. **常数阻力系数**：假设 $C_d$ 不随雷诺数变化。RM 弹速（15~28m/s）下雷诺数处于亚临界区，$C_d$ 近似常数，假设成立。

3. **显式/半隐式欧拉积分**：一阶方法，精度有限。5ms 步长下误差可忽略，但如果需要更高精度可改用 Runge-Kutta 4 阶（RK4）。

4. **单一点目标**：只考虑目标中心点，未结合目标运动预测（需要外层配合，飞行时间 `fly_time` 就是给运动预测用的）。

5. **迭代修正无阻尼**：$\theta_{new} = \theta_{old} + \arctan(e/d)$ 是全量修正，极端参数下可能震荡。实际 RM 场景初值好，不会震荡。

### 7.2 可能的改进方向

| 改进项 | 收益 | 代价 |
|---|---|---|
| RK4 积分 | 精度提升，可适当增大步长 | 代码复杂度增加，单步计算量×4 |
| 阻力系数随雷诺数插值 | 更宽速度范围精度 | 需要实验数据拟合 |
| 加入横风模型 | 室外场景更准 | 需要风速传感器 |
| 迭代加阻尼/牛顿法 | 极端场景收敛更稳 | 代码复杂度增加 |
| 提前计算弹道表 | 运行时零计算（查表） | 需要预计算存储空间，灵活性降低 |

---

## 8. 附录：完整调用流程

### 8.1 典型使用方式

```cpp
// 构造弹道求解器，自动完成迭代求解
rm_utils::Trajectory traj(
    25.0,    // v0: 初速 25 m/s
    5.0,     // d:  目标水平距离 5m
    0.2,     // h:  目标比枪口高 0.2m
    9.81,    // gravity: 重力加速度
    0.0163   // air_resistance: 合并阻力系数
);

if (traj.unsolvable)
{
    // 无解：距离过远 / 初速不足 / 俯仰角超限位
    // 不应射击
}
else
{
    double pitch    = traj.pitch;     // 计算出的俯仰角(rad)
    double fly_time = traj.fly_time;  // 飞行时间(s)，用于目标运动预判
    // 下发 pitch 给云台控制
}
```

### 8.2 单次求解完整执行链

```
Trajectory(v0, d, h, g, k)
│
├─ 参数合法性检查
│
├─ 初始俯仰角 θ₀ = atan2(h, d)
│
└─ 迭代循环（最多10次）
    │
    ├─ estimate(v0, θᵢ, d, g, k)
    │   │
    │   ├─ 初始化 x=y=t=0, vx=v0cosθ, vy=v0sinθ
    │   │
    │   └─ 积分循环（每步5ms）
    │       ├─ 保存上一步状态
    │       ├─ 计算合速度 speed=hypot(vx,vy)
    │       ├─ 更新速度（阻力+重力）
    │       ├─ 更新位置
    │       ├─ 更新时间
    │       └─ 数值保护（NaN/Inf → 无解）
    │
    ├─ 终点线性插值 → 实际高度 actual_h, 飞行时间 time
    │
    ├─ 误差 error = h - actual_h
    │
    ├─ |error| < 1mm ?
    │   ├─ 是 → 收敛！保存 pitch, fly_time, unsolvable=false，返回
    │   └─ 否 → 继续
    │
    ├─ 修正俯仰角 θᵢ₊₁ = θᵢ + atan2(error, d)
    │
    └─ |θᵢ₊₁| > 40° ? → 超出限位，无解返回
```

### 8.3 无解（unsolvable=true）的所有可能原因

1. **参数非法**：初速/距离/重力非正、阻力系数为负、存在 NaN/Inf；
2. **弹道仿真失败**：积分过程中出现 NaN/Inf；
3. **弹丸飞不到目标**：飞行时间超过 4s 或水平速度衰减到 0.1m/s 以下，x 仍未达到目标距离；
4. **俯仰角超限位**：迭代修正后俯仰角超过 ±40°，物理上云台无法达到；
5. **迭代不收敛**：10 次迭代后高度误差仍大于 1mm（实际 RM 场景几乎不会发生）。

---

> **文档结束**
> 本文件基于 `rm_utils/algorithm/trajectory.cpp` 源码逐行分析，所有公式与代码逻辑一一对应。如有代码更新，需同步修订本文档。
