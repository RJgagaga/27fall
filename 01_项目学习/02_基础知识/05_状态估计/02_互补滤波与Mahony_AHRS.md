# 互补滤波与 Mahony AHRS

> **一句话定位**：陀螺短时准、长时漂；加速度计长时准、短时噪。互补滤波和 Mahony 算法就是把两者"取长补短"，得到又稳又准的姿态。这是 CV 技能栏里 "Kalman/Mahony complementary filtering" 中 Mahony 的那一半，也是 P3 云台 AHRS 的算法背景。

## 本章学完你能……

- 说清为什么单靠陀螺或单靠加速度计都不行，以及"互补"的频域含义。
- 从零推导一阶互补滤波 $\hat\theta_k=\alpha(\hat\theta_{k-1}+\omega\Delta t)+(1-\alpha)\theta_{acc}$，并算出时间常数 $\tau$ 与稳态偏差 $\tau b$。
- 推导 Mahony 算法：估计重力方向 $\mathbf v$、误差叉积 $\mathbf e=\mathbf a\times\mathbf v$、PI 修正陀螺、四元数积分，并解释每个符号的方向为什么对。
- 手算一步 Mahony（四元数 + 误差 + 修正），并手算一维 PI 补偿零偏的前几步，看到零偏估计收敛。
- 写出离散实现（C 代码骨架），说出 $K_p$、$K_i$ 怎么选，以及加速度计门控。
- 对比 Mahony / Madgwick / EKF，并用英文回答面试问题。

## 前置知识

- 陀螺 / 加速度计的误差模型：[传感器章](./01_传感器_IMU_编码器_激光雷达.md)
- 四元数与旋转：[欧拉角、四元数、姿态表示](../03_坐标变换与运动学/02_欧拉角_四元数_姿态表示.md)
- 线性系统极点与稳定性：[反馈控制与系统响应基础](../04_控制理论/01_反馈控制与系统响应基础.md)

## 目录

1. 姿态融合问题
2. 一阶互补滤波：推导与手算
3. 互补滤波的频域含义
4. Mahony 算法：几何思想
5. Mahony 完整推导
6. 手算：Mahony 一步（三维）
7. 手算：PI 补偿零偏（一维）
8. 离散实现与工程细节
9. Mahony、Madgwick、EKF 对比
10. 在本项目里怎么用
11. 常见错误与故障现象
12. 面试怎么问、怎么答
13. 本章要点速记、自测题、相关链接

---

## 1. 姿态融合问题

### 1.1 要解决什么问题

我们想要机体的姿态（roll、pitch、yaw），拿到的是：

- **陀螺**：角速度 $\omega_m=\omega+b+n$。积分得到角度，短时间非常好，但零偏 $b$ 积分后线性漂移。
- **加速度计**：静止时给出重力方向，所以 roll / pitch 有绝对参考，但噪声大，并且运动加速度会污染它。
- **磁力计**（可选）：给 yaw 参考，但怕磁干扰。

只用陀螺：姿态**越来越偏**。只用加速度计：姿态**噪、受运动污染**。取长补短就是"**姿态与航向参考系统**"（AHRS，Attitude and Heading Reference System）的核心思路。

### 1.2 取长补短的直觉

把信号按频率拆开：

- 陀螺积分角度的**高频部分**可信（快速变化时它跟得上），**低频部分**不可信（漂移 = 极低频）。
- 加速度计角度的**低频部分**可信（平均后噪声消掉，运动加速度往往是短暂的），**高频部分**不可信（噪声与振动）。

于是：陀螺角度**过高通**，加速度计角度**过低通**，加起来。两个滤波器的传递函数相加恰好等于 1，所以叫**互补**。

---

## 2. 一阶互补滤波：推导与手算

### 2.1 符号

| 符号 | 含义 | 单位 |
|---|---|---|
| $\theta$ | 要估计的角度（如 pitch） | rad |
| $\omega_m$ | 陀螺读数 | rad/s |
| $\theta_{acc}$ | 加速度计算出的角度 | rad |
| $\hat\theta$ | 估计值 | rad |
| $\tau$ | 互补滤波时间常数 | s |
| $\alpha$ | 离散滤波系数 | 无量纲，0~1 |
| $\Delta t$ | 采样周期 | s |

### 2.2 连续形式

最直观的写法：以陀螺积分为主干，用加速度计角度与当前估计的差，**反馈**修正：

$$
\dot{\hat\theta}=\omega_m+\frac1\tau\big(\theta_{acc}-\hat\theta\big)
$$

读这个式子：如果加速度计说角度比估计大（$\theta_{acc}>\hat\theta$），就给估计一个正的"推力"，把估计拉向加速度计，拉的速度由 $1/\tau$ 决定。

在拉氏域取 $\hat\Theta(s)$：

$$
s\hat\Theta=\Omega_m+\frac1\tau(\Theta_{acc}-\hat\Theta)\ \Rightarrow\ \hat\Theta=\frac{1}{\tau s+1}\Theta_{acc}+\frac{\tau s}{\tau s+1}\cdot\frac{\Omega_m}{s}
$$

第一项 $\dfrac1{\tau s+1}$ 是低通（作用于加速度计角度），第二项 $\dfrac{\tau s}{\tau s+1}$ 是高通（作用于陀螺积分 $\Omega_m/s$）。二者之和 $=1$：互补。

### 2.3 离散形式

用前向欧拉离散，令 $\hat\theta_{k}^{-}=\hat\theta_{k-1}+\omega_m\Delta t$（陀螺预测），则

$$
\hat\theta_k=\hat\theta_k^-+\frac{\Delta t}{\tau}\big(\theta_{acc}-\hat\theta_k^-\big)=\underbrace{\Big(1-\tfrac{\Delta t}{\tau}\Big)}_{\alpha}\hat\theta_k^-+\underbrace{\tfrac{\Delta t}{\tau}}_{1-\alpha}\theta_{acc}
$$

也常用等价写法 $\alpha=\dfrac{\tau}{\tau+\Delta t}$（后向欧拉）。两者在 $\Delta t\ll\tau$ 时几乎一致。

**怎么选 $\tau$**：$\tau$ 决定"多慢以下的变化听加速度计的"。截止频率 $f_c=1/(2\pi\tau)$。

- $\tau$ 大：更信陀螺，平滑，但零偏拉不回来的时间长，对运动加速度更不敏感。
- $\tau$ 小：更信加速度计，噪声大，运动加速度污染更重。
- 经验上取 $\tau$ 比"运动加速度持续时间"大、比"陀螺零偏造成明显误差的时间"小，常见在零点几秒到几秒之间。（具体按你的传感器和车体振动调试。）

### 2.4 稳态偏差：零偏的代价

令陀螺有恒定零偏 $b$，真实角度不变（$\omega=0$，$\theta=\theta_{acc}$ 无噪声）。稳态时 $\dot{\hat\theta}=0$：

$$
0=b+\frac1\tau(\theta_{acc}-\hat\theta)\ \Rightarrow\ \hat\theta-\theta_{acc}=\tau\,b
$$

所以**互补滤波会残留 $\tau b$ 的稳态偏差**。零偏越大、$\tau$ 越长，偏差越大。这就是后面 Mahony 加积分项的动机：积分项把零偏估出来，稳态偏差归零。

### 2.5 手算

**示例**：$\Delta t=0.01$ s，$\tau=0.5$ s。

$$
\alpha=\frac{\tau}{\tau+\Delta t}=\frac{0.5}{0.51}=0.98039
$$

设 $\hat\theta_{k-1}=9.9°$，陀螺读数 $\omega_m=0.5\ °/\text{s}$（实际静止，这是零偏），加速度计角度 $\theta_{acc}=10.0°$。

- 陀螺预测：$\hat\theta^-=9.9+0.5\times0.01=9.905°$。
- 融合：$\hat\theta_k=0.98039\times9.905+0.01961\times10.0=9.71019+0.19608=9.9063°$。

稳态偏差：$\tau b=0.5\times0.5=0.25°$。说明估计会在 $10.25°$ 处平衡（比加速度计的 10° 多出 0.25°），因为陀螺零偏每秒想把角度多推 0.5°，而反馈只在差值足够大时才拉得回来。

---

## 3. 互补滤波的频域含义

互补滤波可看成两个滤波器：

$$
G_{LP}(s)=\frac1{\tau s+1},\qquad G_{HP}(s)=\frac{\tau s}{\tau s+1},\qquad G_{LP}+G_{HP}=1
$$

画成示意（对数频率轴）：

```text
增益
1.0 ┤▔▔▔▔▔▔▔╲                      低通(加速度计)：低频通过
    │         ╲
0.7 ┤ - - - - -╳- - - - - - -     截止频率 fc = 1/(2*pi*tau)
    │         ╱ ╲
    │       ╱     ╲▁▁▁▁▁▁▁▁
0.0 ┼▁▁▁▁╱───────────────▶ 频率(对数)
        高通(陀螺积分)：高频通过
```

截止频率处两者增益都是 $1/\sqrt2\approx0.707$，相位互补使二者之和仍为 1。这解释了：**互补滤波不是"平均"，而是按频段分工**。

互补滤波的局限：

1. 对"运动加速度"的处理很粗糙：只能靠时间常数忽略它，无法知道此刻加速度计该不该信。
2. 零偏只能靠稳态偏差折中，不能真正估计零偏。
3. 三维姿态不能简单对 roll、pitch 分别做；要用旋转表示（四元数 / 旋转矩阵）。

Mahony 算法就是把互补滤波推广到三维，并加入积分项估计零偏。

---

## 4. Mahony 算法：几何思想

### 4.1 要解决什么问题

在三维空间用四元数表示姿态，并且：

1. 用加速度计判断"估计的重力方向"与"测到的重力方向"差多少；
2. 把这个差当成"角速度修正量"，反馈到陀螺上；
3. 对差做积分，学出陀螺零偏。

> 原论文：R. Mahony, T. Hamel, J.-M. Pflimlin, "Nonlinear complementary filters on the special orthogonal group", IEEE Trans. Automatic Control, 2008。（本文凭记忆引用，细节未逐页核对。）

### 4.2 符号与单位

| 符号 | 含义 |
|---|---|
| $\mathbf q=(q_w,q_x,q_y,q_z)$ | 单位四元数，表示机体相对世界的姿态 |
| $\boldsymbol\omega_m$ | 陀螺读数（机体系），rad/s |
| $\mathbf a_m$ | 加速度计读数（机体系），先**归一化**成单位向量 $\hat{\mathbf a}$ |
| $\mathbf v$ | 由当前姿态预测的"重力方向（向上）"，在机体系下的单位向量 |
| $\mathbf e$ | 测量与预测重力方向的叉积（误差） |
| $\hat{\mathbf b}$ | 陀螺零偏估计，rad/s |
| $K_p, K_i$ | 比例、积分增益 |

---

## 5. Mahony 完整推导

### 5.1 四元数运动学

单位四元数 $\mathbf q$ 表示机体到世界的旋转。机体系下的角速度 $\boldsymbol\omega$ 满足

$$
\dot{\mathbf q}=\tfrac12\,\mathbf q\otimes\begin{bmatrix}0\\ \boldsymbol\omega\end{bmatrix}
$$

其中 $\otimes$ 是四元数乘法。展开为分量：

$$
\dot q_w=-\tfrac12(q_x\omega_x+q_y\omega_y+q_z\omega_z)
$$
$$
\dot q_x=\ \ \tfrac12(q_w\omega_x+q_y\omega_z-q_z\omega_y)
$$
$$
\dot q_y=\ \ \tfrac12(q_w\omega_y-q_x\omega_z+q_z\omega_x)
$$
$$
\dot q_z=\ \ \tfrac12(q_w\omega_z+q_x\omega_y-q_y\omega_x)
$$

### 5.2 由姿态预测重力方向

世界系里"向上"是 $e_3=(0,0,1)^\top$。机体系下看到的"向上"是 $R^\top e_3$，即旋转矩阵 $R$ 的**第三行**。用四元数分量表示（标准公式）：

$$
\mathbf v=\begin{bmatrix}2(q_xq_z-q_wq_y)\\ 2(q_wq_x+q_yq_z)\\ q_w^2-q_x^2-q_y^2+q_z^2\end{bmatrix}
$$

**检验**：$\mathbf q=(1,0,0,0)$ 时 $\mathbf v=(0,0,1)$：机体与世界对齐，机体系里"上"就是 z ✓。绕 x 转 $\phi$：$q_w=\cos\frac\phi2,\ q_x=\sin\frac\phi2$，则 $v_y=2q_wq_x=\sin\phi$，$v_z=q_w^2-q_x^2=\cos\phi$，与上一章机体系"上"向量 $(0,\sin\phi,\cos\phi)$ 一致 ✓。

### 5.3 误差：叉积

加速度计归一化后 $\hat{\mathbf a}$ 是"测量到的上"，$\mathbf v$ 是"预测的上"。如果姿态估计完全正确（且无运动加速度），则 $\hat{\mathbf a}=\mathbf v$。偏差用叉积表示：

$$
\mathbf e=\hat{\mathbf a}\times\mathbf v
$$

**为什么叉积能当误差**：两个单位向量的叉积 $=\sin\gamma\cdot\hat{\mathbf n}$，方向 $\hat{\mathbf n}$ 垂直于它们所在平面，大小 $\sin\gamma$ 是夹角 $\gamma$ 的正弦。小角度时就是"旋转轴 × 角度"，正好是一个角速度量纲的修正向量。

### 5.4 方向对不对？推导 $K_p$ 项的符号

设 $\mathbf u$ 是世界系里一个固定向量，机体以角速度 $\boldsymbol\omega$（机体系）转动，则机体系下看到的 $\mathbf u$ 的变化率是

$$
\dot{\mathbf u}=-\boldsymbol\omega\times\mathbf u
$$

（机体顺时针转，固定的东西在机体眼里就逆时针转。）所以预测向量 $\mathbf v$ 在修正角速度 $\boldsymbol\omega_c$ 作用下：$\dot{\mathbf v}=-\boldsymbol\omega_c\times\mathbf v$。取 $\boldsymbol\omega_c=K_p\,\mathbf e=K_p(\hat{\mathbf a}\times\mathbf v)$：

$$
\dot{\mathbf v}=-K_p(\hat{\mathbf a}\times\mathbf v)\times\mathbf v
$$

用恒等式 $(\mathbf A\times\mathbf B)\times\mathbf C=\mathbf B(\mathbf A\cdot\mathbf C)-\mathbf A(\mathbf B\cdot\mathbf C)$，并注意 $\mathbf v\cdot\mathbf v=1$：

$$
(\hat{\mathbf a}\times\mathbf v)\times\mathbf v=\mathbf v(\hat{\mathbf a}\cdot\mathbf v)-\hat{\mathbf a}
$$
$$
\Rightarrow\ \dot{\mathbf v}=K_p\big(\hat{\mathbf a}-(\hat{\mathbf a}\cdot\mathbf v)\mathbf v\big)
$$

括号里是 $\hat{\mathbf a}$ 在垂直于 $\mathbf v$ 平面上的分量，指向 $\hat{\mathbf a}$ 的方向。所以 $\mathbf v$ 被"拉向" $\hat{\mathbf a}$，夹角指数衰减，速度 $\propto K_p$。方向正确。

### 5.5 加入积分项：补偿陀螺零偏

陀螺读数含零偏：$\boldsymbol\omega_m=\boldsymbol\omega+\mathbf b+\mathbf n$。我们用估计 $\hat{\mathbf b}$ 去掉它。Mahony 的做法：

$$
\boldsymbol\omega_c=\boldsymbol\omega_m-\hat{\mathbf b}+K_p\,\mathbf e,\qquad \dot{\hat{\mathbf b}}=-K_i\,\mathbf e
$$

为什么积分项是 $-K_i\mathbf e$？看一维线性化。令姿态误差 $\delta=\theta-\hat\theta$（真值减估计），零偏误差 $\tilde b=b-\hat b$，小角度下 $e\approx\delta$。则

$$
\dot{\hat\theta}=\omega_m-\hat b+K_p\delta=\omega+\tilde b+K_p\delta
$$
$$
\dot\delta=\omega-\dot{\hat\theta}=-\tilde b-K_p\delta,\qquad \dot{\tilde b}=-\dot{\hat b}=K_i\delta
$$

写成状态空间：

$$
\frac{d}{dt}\begin{bmatrix}\delta\\ \tilde b\end{bmatrix}=\begin{bmatrix}-K_p&-1\\ K_i&0\end{bmatrix}\begin{bmatrix}\delta\\ \tilde b\end{bmatrix}
$$

特征方程 $s^2+K_ps+K_i=0$。当 $K_p,K_i>0$ 时两个极点都在左半平面，系统**稳定**，且稳态 $\delta=0$、$\tilde b=0$：角度误差归零，零偏被学出来。

极点：$s=\dfrac{-K_p\pm\sqrt{K_p^2-4K_i}}{2}$。

- $K_p^2>4K_i$：两个负实根（过阻尼）；
- $K_p^2<4K_i$：一对共轭复根（欠阻尼，估计会振荡）。

这和 PI 控制器的整定同一回事（见[PID控制全解](../04_控制理论/02_PID控制全解.md)）：$K_p$ 管响应速度，$K_i$ 管稳态偏差。

### 5.6 离散实现步骤

每个采样周期 $\Delta t$：

1. 归一化加速度计 $\hat{\mathbf a}=\mathbf a_m/\|\mathbf a_m\|$（若模长为 0 或明显偏离 $g$，跳过修正，见 8.3）。
2. 由当前 $\mathbf q$ 计算 $\mathbf v$。
3. $\mathbf e=\hat{\mathbf a}\times\mathbf v$。
4. 积分项：$\hat{\mathbf b}\leftarrow\hat{\mathbf b}-K_i\,\mathbf e\,\Delta t$。
5. 修正角速度：$\boldsymbol\omega_c=\boldsymbol\omega_m-\hat{\mathbf b}+K_p\mathbf e$。
6. 积分四元数：$\mathbf q\leftarrow\mathbf q+\tfrac12\,\mathbf q\otimes(0,\boldsymbol\omega_c)\,\Delta t$。
7. 归一化 $\mathbf q\leftarrow\mathbf q/\|\mathbf q\|$。

---

## 6. 手算：Mahony 一步（三维）

**示例设定**（教学数值）：

- 初始 $\mathbf q=(1,0,0,0)$，即估计认为机体水平；$\hat{\mathbf b}=0$。
- 真实情况：机体绕 x 轴横滚了 $10°$，加速度计测到 $\mathbf a_m=g(0,\ \sin10°,\ \cos10°)$，归一化 $\hat{\mathbf a}=(0,\ 0.17365,\ 0.98481)$。
- 陀螺读数 $\boldsymbol\omega_m=(0,0,0)$，$K_p=2.0$，$K_i=0.005$，$\Delta t=0.01$ s。

**第 1 步：预测重力方向。** $\mathbf q=(1,0,0,0)$ 代入 5.2 的 $\mathbf v$：$v_x=0$，$v_y=0$，$v_z=1$。$\mathbf v=(0,0,1)$。

**第 2 步：误差叉积。**

$$
\mathbf e=\hat{\mathbf a}\times\mathbf v=\begin{bmatrix}a_yv_z-a_zv_y\\ a_zv_x-a_xv_z\\ a_xv_y-a_yv_x\end{bmatrix}=\begin{bmatrix}0.17365\cdot1-0\\0-0\\0\end{bmatrix}=\begin{bmatrix}0.17365\\0\\0\end{bmatrix}
$$

误差只在 x 轴，大小 $0.17365=\sin10°$，正号表示"估计要向正 roll 方向转"。符号正确。

**第 3 步：积分项。** $\hat b_x\leftarrow0-0.005\times0.17365\times0.01=-8.68\times10^{-6}$ rad/s。（极小，因为只有一步。）

**第 4 步：修正角速度。**

$$
\omega_{c,x}=0-(-8.68\times10^{-6})+2.0\times0.17365=0.347300+0.0000087=0.34731\ \text{rad/s}
$$

$\omega_{c,y}=\omega_{c,z}=0$。

**第 5 步：积分四元数。** 对 $\mathbf q=(1,0,0,0)$，$\dot{\mathbf q}=\tfrac12(0,\omega_x,\omega_y,\omega_z)$（分量代入 5.1，$q_w=1$ 其余为 0）：

$$
\mathbf q\leftarrow(1,\ 0,\ 0,\ 0)+0.01\times\tfrac12\times(0,\ 0.34731,\ 0,\ 0)=(1,\ 0.0017366,\ 0,\ 0)
$$

**第 6 步：归一化。** $\|\mathbf q\|=\sqrt{1+3.016\times10^{-6}}=1.0000015$，$\mathbf q=(0.9999985,\ 0.0017366,\ 0,\ 0)$。

**第 7 步：结果。** 对应 roll：$\phi=2\arcsin(q_x)\approx2\times0.0017366=0.003473$ rad $=0.199°$。

一步把估计从 $0°$ 推到 $0.2°$，离 $10°$ 目标还远。这是正常的：$K_p=2$ 时，误差衰减时间常数约 $1/K_p=0.5$ s，需要上百步才能收敛。这说明：**Mahony 不是一步到位，是以 $1/K_p$ 的速度把估计"拉"过去。**

---

## 7. 手算：PI 补偿零偏（一维）

用 5.5 的线性化模型做一个可以手算的数值例子。

**设定**（教学数值）：真实角度恒为 $\theta=10°$（静止），陀螺零偏 $b=0.5\ °/\text{s}$，所以陀螺读数 $\omega_m=0.5$。加速度计无噪声，$\theta_{acc}=10°$。初始 $\hat\theta=0°$，$\hat b=0$。$K_p=2$，$K_i=1$，$\Delta t=0.1$ s。

更新：

$$
e_k=\theta-\hat\theta_k,\quad \hat\theta_{k+1}=\hat\theta_k+\Delta t\big(\omega_m-\hat b_k+K_pe_k\big),\quad \hat b_{k+1}=\hat b_k-\Delta t\,K_i\,e_k
$$

**第 1 步**：$e_0=10$。
$\hat\theta_1=0+0.1\times(0.5-0+2\times10)=0.1\times20.5=2.05°$；
$\hat b_1=0-0.1\times1\times10=-1.0$。

**第 2 步**：$e_1=10-2.05=7.95$。
$\hat\theta_2=2.05+0.1\times(0.5-(-1.0)+2\times7.95)=2.05+0.1\times17.4=3.79°$；
$\hat b_2=-1.0-0.1\times7.95=-1.795$。

**第 3 步**：$e_2=6.21$。
$\hat\theta_3=3.79+0.1\times(0.5+1.795+12.42)=3.79+1.4715=5.2615°$；
$\hat b_3=-1.795-0.621=-2.416$。

后续（按相同公式迭代，数字由程序给出，可自行验证）：

| 步数 $k$ | $\hat\theta_k$（°） | $\hat b_k$（°/s） |
|---|---|---|
| 3 | 5.26 | −2.42 |
| 10 | 10.58 | −3.74 |
| 50 | 10.25 | 0.20 |
| 100 | 10.003 | 0.497 |
| 200 | 10.000 | 0.500 |

观察：

1. 角度估计在约 1 s（10 步）内就过冲到 10.58°，然后收敛到 10°。
2. 零偏估计 $\hat b$ 先朝反方向走到 −3.7，再收敛到真值 $0.5$。这是 PI 在 $K_p^2=4=4K_i$ 附近（临界阻尼）时的瞬态；最终 $\hat b\to b=0.5$，**稳态偏差为零**。对比互补滤波的 $\tau b$ 残差，这就是积分项的价值。
3. 收敛时间由极点 $s^2+2s+1=0$（双重极点 $-1$）决定，时间常数约 1 s；$K_i$ 越小，零偏收敛越慢。

> **注意**：实际代码里 $K_i$ 常取很小（如 0.005~0.1 量级，随实现的缩放而不同），极点接近 0，零偏要几十到几百秒才收敛；因此零偏**初值**用静置标定给好，积分项只做慢速跟踪。

---

## 8. 离散实现与工程细节

### 8.1 C 语言骨架（教学示例，非项目源码）

```c
#include <math.h>

typedef struct {
    float q0, q1, q2, q3;     // quaternion (w, x, y, z)
    float bx, by, bz;         // gyro bias estimate [rad/s]
    float kp, ki;
} Mahony;

void mahony_update_imu(Mahony *m,
                       float gx, float gy, float gz,   // rad/s
                       float ax, float ay, float az,   // any unit
                       float dt)
{
    float norm = sqrtf(ax*ax + ay*ay + az*az);
    if (norm > 1e-6f) {
        ax /= norm; ay /= norm; az /= norm;

        // predicted gravity direction in body frame (third row of R)
        float vx = 2.0f * (m->q1*m->q3 - m->q0*m->q2);
        float vy = 2.0f * (m->q0*m->q1 + m->q2*m->q3);
        float vz = m->q0*m->q0 - m->q1*m->q1 - m->q2*m->q2 + m->q3*m->q3;

        // error = a x v
        float ex = ay*vz - az*vy;
        float ey = az*vx - ax*vz;
        float ez = ax*vy - ay*vx;

        // integral term: b_hat += -ki * e * dt
        m->bx -= m->ki * ex * dt;
        m->by -= m->ki * ey * dt;
        m->bz -= m->ki * ez * dt;

        // corrected angular rate
        gx = gx - m->bx + m->kp * ex;
        gy = gy - m->by + m->kp * ey;
        gz = gz - m->bz + m->kp * ez;
    }

    // q_dot = 0.5 * q (x) (0, w)
    float q0 = m->q0, q1 = m->q1, q2 = m->q2, q3 = m->q3;
    float hdt = 0.5f * dt;
    m->q0 = q0 + hdt * (-q1*gx - q2*gy - q3*gz);
    m->q1 = q1 + hdt * ( q0*gx + q2*gz - q3*gy);
    m->q2 = q2 + hdt * ( q0*gy - q1*gz + q3*gx);
    m->q3 = q3 + hdt * ( q0*gz + q1*gy - q2*gx);

    float n = sqrtf(m->q0*m->q0 + m->q1*m->q1 + m->q2*m->q2 + m->q3*m->q3);
    m->q0 /= n; m->q1 /= n; m->q2 /= n; m->q3 /= n;
}
```

得到 Euler 角（ZYX）：

```c
float roll  = atan2f(2.0f*(q0*q1 + q2*q3), 1.0f - 2.0f*(q1*q1 + q2*q2));
float pitch = asinf (2.0f*(q0*q2 - q3*q1));
float yaw   = atan2f(2.0f*(q0*q3 + q1*q2), 1.0f - 2.0f*(q2*q2 + q3*q3));
```

### 8.2 6 轴与 9 轴

- **6 轴**（陀螺 + 加速度计）：如上，**yaw 无参考**，会随零偏缓慢漂移，只靠积分项也无法修正（yaw 对重力不可观）。
- **9 轴**（加磁力计）：再预测"磁场方向" $\mathbf w$，误差加上 $\hat{\mathbf m}\times\mathbf w$ 一项，yaw 有了绝对参考，但要有硬铁 / 软铁标定，并忍受磁干扰。
- 室内轮式机器人常见做法：IMU 的 yaw 只用作短时连续性，绝对 yaw 由 LiDAR 地图匹配（AMCL）提供，再由 EKF 融合。

### 8.3 加速度计门控

运动加速度会污染 $\hat{\mathbf a}$。常见做法：

- 若 $\big|\|\mathbf a_m\|-g\big|>\epsilon_a$，说明有显著线加速度，**降低 $K_p$ 甚至跳过修正**（仅用陀螺积分）；
- 或把 $K_p$ 乘以随偏差增大而衰减的权重函数。

**手算**：门限 $\epsilon_a=0.1g$。车辆急加速时读数模长 $\|\mathbf a_m\|=1.25g$，偏差 $0.25g>0.1g$，该周期不修正；若 $1.05g$，偏差 $0.05g$，正常修正。

### 8.4 增益怎么选

| 参数 | 影响 | 经验做法 |
|---|---|---|
| $K_p$ 大 | 响应快，噪声与运动加速度的影响大 | 先固定 $K_i=0$ 调 $K_p$ |
| $K_p$ 小 | 平滑但收敛慢，更依赖陀螺 | 与振动、车速相关 |
| $K_i$ 大 | 零偏收敛快，但可能振荡 | 保持 $K_p^2\ge4K_i$ |
| $K_i=0$ | 无零偏补偿 | 短时任务可用 |

调试方法：把机体静置 1 分钟，看 roll / pitch 是否稳定；人为快速倾斜后放平，看恢复时间；有运动加速度时看姿态是否被拖偏。

---

## 9. Mahony、Madgwick、EKF 对比

| 维度 | 互补滤波（一维） | Mahony | Madgwick | EKF / ESKF |
|---|---|---|---|---|
| 状态 | 单角度 | 四元数 + 零偏 | 四元数（原版不估零偏） | 四元数 + 零偏 + 协方差 |
| 修正量来源 | 角度差 | 叉积 $\hat{\mathbf a}\times\mathbf v$ | 对目标函数做梯度下降 | 卡尔曼增益 |
| 零偏估计 | 无（有稳态偏差 $\tau b$） | PI 积分项 | 原版没有，后续版本加入 | 状态之一 |
| 自适应权重 | 需手工 | 需手工门控 | 步长 $\beta$ 手工 | 由 $R$、$P$ 自动调节 |
| 计算量 | 最小 | 小 | 小 | 较大（矩阵运算） |
| 参数 | $\tau$ | $K_p,K_i$ | $\beta$ | $Q,R$ |
| 适合 | 教学、极简 MCU | 嵌入式姿态（云台、飞控） | 同左 | 需要不确定度、多传感器融合 |

**要点**：Mahony 与 Madgwick 都是"计算便宜的互补式"；EKF 提供**协方差**，可以做一致性检验，能自然融合更多传感器。想知道估计"有多可信"，只能上 EKF 类方法（见[卡尔曼滤波与 EKF](./03_卡尔曼滤波与EKF.md)）。

---

## 10. 在本项目里怎么用

- **P3 RoboMaster 英雄机器人**：CV 写云台 pitch/yaw 角度—角速度串级 PID 使用 "IMU/AHRS and motor feedback"。AHRS 的输出（姿态角）是外环反馈，陀螺角速度是内环反馈。Mahony / 互补类算法是这种场景最常见的姿态来源（典型做法）。见 [P3 README](../../01_项目课程/P3_RoboMaster英雄机器人嵌入式控制/README.md)。
- **P5 车式移动机器人**：CV 技能栏列出 "Kalman/Mahony complementary filtering"。导航中 IMU 姿态 / yaw 速率是 EKF 的输入之一，用于在 LiDAR 帧之间保持航向连续。见 [P5 README](../../01_项目课程/P5_车式移动机器人自主导航/README.md) 与 [卡尔曼滤波与 EKF](./03_卡尔曼滤波与EKF.md)。
- **P1 / P7**：平衡类项目用 IMU 得到 pitch / pitch rate 作为状态反馈，姿态来源同样可以是互补类滤波（见各自 README）。

> **【待你补全】** 你在 P3 / P5 里实际调的是 $K_p$、$K_i$ 还是 $\tau$？数值是多少？有没有加速度计门控？yaw 有没有用磁力计？CV 没写，面试官若问"你怎么调的"，要用自己的真实经历回答。

---

## 11. 常见错误与故障现象

| 现象 | 原因 | 处理 |
|---|---|---|
| 静置姿态缓慢"爬" | 零偏未补偿或 $K_i$ 太小 | 静置标定初值；适当增大 $K_i$ |
| 车一加速姿态就偏 | 未做加速度计门控 | 加模长门限；降低 $K_p$ |
| 估计振荡 | $K_i$ 过大，$K_p^2<4K_i$ | 减小 $K_i$ |
| 姿态翻转 90° 或 180° | 轴向 / 符号接错（IMU 坐标与车体不一致） | 先在静态用手倾斜验证符号 |
| yaw 长期漂移 | 6 轴无 yaw 参考 | 加磁力计，或由 LiDAR / 里程计修正 |
| 四元数发散 | 忘记归一化或 $\Delta t$ 取错 | 每步归一化；用实测 $\Delta t$ |
| 数值爆炸 | 陀螺单位是 °/s 却当 rad/s | 统一单位 |

---

## 12. 面试怎么问、怎么答

**Q1. Why do you need to fuse a gyroscope and an accelerometer? Can't you just use one?**

> A: The gyro is accurate over short times but its bias makes the integrated angle drift. The accelerometer gives an absolute gravity reference, but it is noisy and corrupted by linear acceleration. A complementary filter uses the gyro for high-frequency changes and the accelerometer as a slow correction, so I get a smooth and drift-free tilt estimate.

中文要点：陀螺短时准长时漂，加速度计长时准短时噪、运动时有污染；互补滤波按频段分工。

**Q2. Explain how the Mahony filter works.**

> A: It keeps a quaternion. From the quaternion I predict the gravity direction in the body frame, and I compare it with the normalised accelerometer vector using a cross product. That cross product acts like a rotation-axis error. I feed it back to the gyro through a proportional term, and an integral term estimates the gyro bias. Then I integrate the quaternion with the corrected angular rate and normalise it.

中文要点：四元数预测重力方向 → 与加速度计叉积得误差 → 比例项修正角速度、积分项估零偏 → 积分四元数并归一化。

追问：为什么叉积方向是对的？（答：$(\hat a\times v)\times v$ 恒等式推出 $\dot v$ 指向 $\hat a$。）$K_p$、$K_i$ 怎么选？（答：对应二阶系统 $s^2+K_ps+K_i$，保持 $K_p^2\ge4K_i$ 避免振荡。）

**Q3. Mahony versus Madgwick versus EKF?**

> A: Mahony and Madgwick are cheap and good for embedded attitude estimation, but their gains are tuned by hand and they do not give a covariance. An EKF costs more computation, but it estimates uncertainty, adapts the weights through the covariance, and extends naturally to more sensors. For a gimbal on an STM32 I would choose Mahony; for navigation fusion I would use an EKF.

**Q4. Does the accelerometer help with yaw?**

> A: No. Gravity is invariant to rotation about the vertical axis, so yaw is unobservable from the accelerometer. I need a magnetometer, or in indoor navigation a LiDAR map match, to bound yaw drift.

## 13. 本章要点速记

1. 陀螺高频可信，加速度计低频可信；互补滤波 $G_{LP}+G_{HP}=1$。
2. 一阶互补：$\hat\theta_k=\alpha(\hat\theta_{k-1}+\omega\Delta t)+(1-\alpha)\theta_{acc}$，$\alpha=\tau/(\tau+\Delta t)$。
3. 陀螺零偏 $b$ 造成互补滤波稳态偏差 $\tau b$。
4. Mahony 状态：四元数 + 零偏估计。
5. 预测重力方向 $\mathbf v$ = 旋转矩阵第三行；误差 $\mathbf e=\hat{\mathbf a}\times\mathbf v$。
6. 修正：$\omega_c=\omega_m-\hat b+K_p\mathbf e$，$\dot{\hat b}=-K_i\mathbf e$。
7. 线性化后 $s^2+K_ps+K_i=0$，$K_p^2\ge4K_i$ 不振荡。
8. 四元数积分后必须归一化；加速度计要归一化，且需门控。
9. 6 轴 yaw 不可观；9 轴要标定磁力计。
10. 要不确定度和多传感器融合时用 EKF。

## 自测题

1. 互补滤波 $\tau=1$ s，陀螺零偏 $0.2°/\text{s}$，稳态偏差多大？（答：$\tau b=0.2°$。）
2. $\Delta t=0.005$ s，$\tau=0.25$ s，求 $\alpha$。（答：$0.25/0.255=0.9804$。）
3. 四元数 $(1,0,0,0)$，加速度计归一化为 $(0.2,0,0.9798)$，求误差 $\mathbf e$。（答：$\mathbf v=(0,0,1)$；$\mathbf e=(a_yv_z-a_zv_y,\ a_zv_x-a_xv_z,\ a_xv_y-a_yv_x)=(0,\ -0.2,\ 0)$，y 轴负方向，即需要负 pitch 修正。）
4. $K_p=1$，$K_i=0.25$ 的极点？（答：$s^2+s+0.25=(s+0.5)^2$，双重极点 $-0.5$，临界阻尼。）
5. 为什么要对四元数做归一化？（要点：数值积分会让模长偏离 1，而姿态要求单位四元数；否则旋转不再是刚体旋转。）
6. 为什么 6 轴 Mahony 不能修正 yaw 漂移？（要点：重力对绕竖直轴的旋转不变，叉积误差 z 分量天然为 0。）

## 相关链接

- 上一章：[传感器：IMU、编码器、激光雷达](./01_传感器_IMU_编码器_激光雷达.md)
- 下一章：[卡尔曼滤波与 EKF](./03_卡尔曼滤波与EKF.md)
- 相关基础：[欧拉角、四元数、姿态表示](../03_坐标变换与运动学/02_欧拉角_四元数_姿态表示.md)、[PID 控制全解](../04_控制理论/02_PID控制全解.md)
- 项目：[P3](../../01_项目课程/P3_RoboMaster英雄机器人嵌入式控制/README.md)、[P5](../../01_项目课程/P5_车式移动机器人自主导航/README.md)
- 八股：[状态估计与 SLAM 八股](../../03_八股知识/03_机器人学/03_状态估计与SLAM八股.md)
