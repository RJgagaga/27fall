# 05 IMU、AHRS 与云台姿态

> **一句话定位**：云台要"稳"，前提是知道枪口此刻指向哪里、转得多快，而且这个"指向"不能随底盘晃动。IMU（惯性测量单元）加姿态解算（AHRS，Attitude and Heading Reference System）就是做这件事的。本章从传感器模型讲到 Mahony 互补滤波、四元数更新、欧拉角提取，再讲到云台坐标系与符号，全部可手算。

## 本章学完你能……

- 说清陀螺仪和加速度计各自测什么、各自的误差模型，以及为什么必须融合；
- 手算原始 LSB 到物理量的换算、零偏标定、安装矩阵变换；
- 推导并手算一维互补滤波，讲清时间常数与截止频率的关系；
- 推导 Mahony 滤波：重力方向估计 → 叉积误差 → PI 修正 → 四元数积分 → 归一化，并手算一步与收敛过程；
- 写出四元数乘法与四元数到欧拉角的公式，并用 $[0.5, 0.5, 0.5, 0.5]$ 验算；
- 解释 yaw 为什么会漂移、没有磁力计时怎么办、什么情况下这不是问题；
- 说清云台坐标系定义、符号约定、俯仰非零时 yaw 角速度的投影问题；
- 回答"Mahony vs Madgwick vs Kalman""为什么用四元数""怎么标定"等高频追问。

## 前置知识

- [传感器：IMU、编码器、激光雷达](../../02_基础知识/06_状态估计/01_传感器_IMU_编码器_激光雷达.md)
- [互补滤波与 Mahony AHRS](../../02_基础知识/06_状态估计/02_互补滤波与Mahony_AHRS.md)
- [欧拉角、四元数与姿态表示](../../02_基础知识/03_运动学与坐标变换/02_欧拉角_四元数_姿态表示.md)
- 上一章：[04 CAN 总线与电机反馈](./04_CAN总线与电机反馈.md)

## 目录

1. 为什么云台需要 IMU
2. 传感器模型
3. 原始数据到物理量（手算）
4. 安装矩阵与坐标系
5. 零偏标定
6. 姿态的三种表示：欧拉角、旋转矩阵、四元数
7. 四元数运算与积分（手算）
8. 一维互补滤波（手算）
9. Mahony 滤波：推导与手算
10. Madgwick 与 Kalman：对比
11. 四元数到欧拉角（公式与验算）
12. yaw 漂移与磁力计
13. 云台坐标系、符号与投影
14. 在本项目里怎么用
15. 常见错误与故障
16. 面试怎么问、怎么答
17. 要点速记、自测题

---

## 1. 为什么云台需要 IMU

云台的目标是：**不管底盘怎么动，枪口始终指向期望方向**。电机编码器只能告诉你"云台相对底盘转了多少"（相对角）。如果底盘自己在转（例如底盘高速旋转），编码器看到的相对角变化不能说明枪口相对地面（惯性系）的指向变化。

```text
惯性系中的枪口 yaw 角 = 底盘 yaw 角 + 云台相对底盘的 yaw 角
                          ↑ 需要 IMU          ↑ 编码器
```

如果 IMU 装在云台上，它直接测量枪口所在刚体的角速度和姿态，**两项合在一起一次测出**。所以：

- **角速度环**用 IMU 陀螺仪角速度做反馈：直接、低延迟、抗底盘扰动；
- **角度环**用 IMU 姿态角（绝对角）做反馈：枪口相对惯性系的指向。

CV 中的描述：*pitch/yaw angle–rate cascaded PID using IMU/AHRS and motor feedback*。

## 2. 传感器模型

### 2.1 陀螺仪（Gyroscope）

测量的是**角速度**（沿三个轴）。简化模型：

$$
\omega_{\text{meas}} = \omega_{\text{true}} + b + n,\qquad n\sim\mathcal N(0,\sigma_g^2)
$$

- $b$：**零偏**（bias），静止时输出不为 0，随温度和时间缓慢变化；
- $n$：白噪声；
- 还有刻度因子误差、轴间耦合（常忽略）。

陀螺仪**响应快、短期准**，但积分时误差累积：角度误差 $= \int b\,dt = b\,t$。

**手算（教学示例）**：零偏 $b = 0.1\ ^\circ/\text{s}$，不校正地积分 1 分钟：$0.1\times60 = 6^\circ$；10 分钟 $60^\circ$。这就是"漂移"。

### 2.2 加速度计（Accelerometer）

测量的是**比力**（specific force）：真实加速度减去重力加速度：$f = a - g$。静止时 $a=0$，所以测到的是 $-g$ 向量，即**指向上方的 1 g**（读数大小约 9.8 m/s²，方向向上）。

$$
f_{\text{meas}} = R^{T}(a_{\text{world}} - g_{\text{world}}) + b_a + n_a
$$

静止或匀速时，加速度计读数的方向就是"局部竖直向上"，因此可以给出**横滚角和俯仰角**（不能给出 yaw，因为绕竖直轴转动不改变重力方向）。

加速度计**长期准、短期噪**：机器人加速、颠簸、碰撞时，$a_{\text{world}}\neq0$，读数方向不再是竖直。RoboMaster 机器人在撞击、上台阶、发射反冲时这一点非常明显。

### 2.3 互补性

| | 陀螺仪 | 加速度计 |
|---|---|---|
| 短时 | 准 | 噪声大、受运动加速度干扰 |
| 长时 | 漂移 | 准（相对重力） |
| 测量 | 角速度 | 重力方向（横滚、俯仰） |
| 缺点 | 积分漂移 | 不含 yaw 信息 |

融合思想：**高频信任陀螺仪，低频信任加速度计**。

## 3. 原始数据到物理量（手算）

### 3.1 LSB 到物理量

IMU（如 BMI088 一类）以 16 位补码输出，量程可配置。

**陀螺仪**（教学示例：量程 ±2000 °/s）：

$$
\text{灵敏度} = \frac{2000}{32768}= 0.061\ ^\circ/\text{s per LSB}=1.065\times10^{-3}\ \text{rad/s per LSB}
$$

读数 `raw = 1640` → $1640\times0.061 = 100.1\ ^\circ/\text{s}$。

**加速度计**（教学示例：量程 ±6 g）：

$$
\text{灵敏度} = \frac{6\times9.80665}{32768} = 1.796\times10^{-3}\ \text{m/s}^2\text{ per LSB}
$$

读数 `raw = 5460`（约 1/6 满量程 = 1 g）→ $5460\times0.001796 = 9.80$ m/s²。

> 量程选择的取舍：量程大 → 不易饱和，但分辨率低；机器人上陀螺常选大量程（快速旋转时不饱和），加速度计选中等量程。本项目取值：BMI088 陀螺 ±2000 °/s，加速度计 ±6 g。

### 3.2 单位统一

陀螺仪 °/s → rad/s：乘 $\pi/180 = 0.017453$；反向乘 $57.2958$。云台控制代码里经常"IMU 输出弧度，控制环用角度"，要注意每个量的单位，避免一边弧度一边角度。【笔记线索】：你的笔记里提到 `MOTOR_RAD_TO_ANGLE = 57.2958` 用来把 rad/s 转成 °/s。

## 4. 安装矩阵与坐标系

### 4.1 为什么需要

芯片的 x/y/z 轴是芯片自己的；装到板上、板再装到云台上，机器人定义的"前、左、上"可能与芯片轴不一致。需要一个**固定的旋转矩阵 $R_{\text{mount}}$** 把芯片坐标系下的向量变到机器人（云台）坐标系：

$$
\mathbf v_{\text{body}} = R_{\text{mount}}\,\mathbf v_{\text{chip}}
$$

**【笔记线索】**：你的笔记里读到的安装变换是

```text
x_body =  y_chip
y_body = -x_chip
z_body =  z_chip
```

即

$$
R_{\text{mount}}=\begin{bmatrix}0&1&0\\-1&0&0\\0&0&1\end{bmatrix}
$$

**验算它是不是合法旋转**：行列式 $= 0\cdot(0\cdot1-0\cdot0)-1\cdot((-1)\cdot1-0\cdot0)+0 = 1$ ✓，且各行为单位正交向量。这表示绕 z 轴转 $-90^\circ$ 的纯旋转，没有镜像。若矩阵行列式为 $-1$，说明含镜像，会让某个轴的符号永远是反的，姿态解算会发散或方向错。

### 4.2 一个重要的检查：先动手确认符号

用手（或让云台）做一次只绕某一轴的运动，观察对应轴的角速度与角度符号。检查表：

| 动作 | 期望看到 |
|---|---|
| 云台水平左转（俯视逆时针） | yaw 增大，绕 z 的角速度为正 |
| 枪口抬起 | pitch 变化符号符合约定（见第 13 节） |
| 静止放平 | 加速度计 z 轴 ≈ +g，x、y ≈ 0 |

不做这一步直接上闭环，是最常见的"一上电就飞"的原因。

## 5. 零偏标定

### 5.1 静态均值法

机器人完全静止，采集 $N$ 个陀螺仪样本，取平均作为零偏估计：

$$
\hat b = \frac1N\sum_{k=1}^{N}\omega_k
$$

估计误差标准差为 $\sigma_g/\sqrt N$。

**手算（教学示例）**：单次噪声 $\sigma=0.05\ ^\circ/\text{s}$，$N=2000$：$0.05/\sqrt{2000}=0.00112\ ^\circ/\text{s}$，漂移 $0.00112\times60=0.067^\circ/\text{min}$，已经很小。

**判定静止**：取样期间陀螺仪读数方差小于阈值，且加速度计模长接近 $g$，否则重做。

### 5.2 温度影响

零偏随温度变化。开发板上的 IMU 常带**恒温加热电阻**（用 PWM 加热并 PID 控温），让 IMU 保持在稳定温度，零偏就稳定。若没有恒温，上电后几分钟内温度上升会造成零偏漂移，所以"上电后等几分钟再标定"是常见做法。

### 5.3 在线更新

机器人静止（陀螺仪模长很小、持续一段时间）时，可以在线更新零偏（Zero-velocity update, ZUPT）。比赛中不一定有静止机会，但云台指向目标静止时云台相对惯性系的角速度接近 0，可以用，但要小心——把真实慢速转动当零偏会引入误差。

## 6. 姿态的三种表示：欧拉角、旋转矩阵、四元数

| 表示 | 参数数 | 优点 | 缺点 |
|---|---|---|---|
| 欧拉角 yaw-pitch-roll | 3 | 直观，便于显示和控制 | 万向锁（pitch=±90°）；角度绕回；积分角速率需要三角函数 |
| 旋转矩阵 | 9 | 乘向量直接变换；无奇异 | 冗余，约束多 |
| 四元数 | 4 | 无奇异，积分简单，插值平滑，计算量小 | 不直观；双覆盖（$q$ 与 $-q$ 同一旋转） |

AHRS 内部用四元数更新，对外输出欧拉角用于控制和显示。云台 pitch 范围小（典型远小于 90°），欧拉角奇异性在这里基本碰不到，所以"内部四元数、外部欧拉"很合适。

## 7. 四元数运算与积分（手算）

### 7.1 定义

四元数 $q = [w, x, y, z] = w + x\mathbf i + y\mathbf j + z\mathbf k$。单位四元数 $w^2+x^2+y^2+z^2=1$ 表示旋转：绕单位轴 $\mathbf n$ 转角 $\theta$：

$$
q = \left[\cos\tfrac\theta2,\ \mathbf n\sin\tfrac\theta2\right]
$$

**手算**：绕 z 转 $90^\circ$：$q=[\cos45^\circ,0,0,\sin45^\circ]=[0.7071,0,0,0.7071]$。

### 7.2 Hamilton 乘积

$$
\begin{aligned}
q_A\otimes q_B=\big[&w_Aw_B-x_Ax_B-y_Ay_B-z_Az_B,\\
&w_Ax_B+x_Aw_B+y_Az_B-z_Ay_B,\\
&w_Ay_B-x_Az_B+y_Aw_B+z_Ax_B,\\
&w_Az_B+x_Ay_B-y_Ax_B+z_Aw_B\big]
\end{aligned}
$$

乘积表示旋转的复合。**不满足交换律**。

**手算（顺序的重要性）**：$q_z=[0.7071,0,0,0.7071]$（绕 z 90°），$q_x=[0.7071,0.7071,0,0]$（绕 x 90°）。

$q_z\otimes q_x$：
- $w = 0.7071\cdot0.7071 - 0 - 0 - 0.7071\cdot0 = 0.5$
- $x = 0.7071\cdot0.7071 + 0\cdot0.7071 + 0\cdot 0 - 0.7071\cdot0=0.5$
- $y = 0.7071\cdot 0 - 0\cdot0 + 0\cdot0.7071 + 0.7071\cdot0.7071=0.5$
- $z = 0.7071\cdot0 + 0\cdot 0 - 0\cdot0.7071+0.7071\cdot0.7071=0.5$

结果 $[0.5,0.5,0.5,0.5]$。反过来 $q_x\otimes q_z=[0.5,0.5,-0.5,0.5]$，不同。这说明"先绕哪个轴再绕哪个轴"结果不同。

**约定**：陀螺仪测的是**机体坐标系**下的角速度，姿态更新用"右乘"：$q_{\text{new}} = q_{\text{old}}\otimes\Delta q_{\text{body}}$（$q$ 表示机体到惯性系的旋转时）。如果你用另一种约定（$q$ 表示惯性系到机体），公式左右乘会不同，这是"同一份算法在不同代码里看起来不一样"的原因。

### 7.3 运动学方程

四元数随角速度变化：

$$
\dot q = \tfrac12\, q\otimes[0,\ \omega_x,\ \omega_y,\ \omega_z]
$$

展开（机体系角速度）：

$$
\begin{aligned}
\dot w&=\tfrac12(-x\omega_x-y\omega_y-z\omega_z)\\
\dot x&=\tfrac12(\ w\omega_x+y\omega_z-z\omega_y)\\
\dot y&=\tfrac12(\ w\omega_y-x\omega_z+z\omega_x)\\
\dot z&=\tfrac12(\ w\omega_z+x\omega_y-y\omega_x)
\end{aligned}
$$

### 7.4 离散积分（一阶欧拉）与归一化

$$
q_{k+1}=q_k+\dot q_k\,\Delta t,\qquad q_{k+1}\leftarrow \frac{q_{k+1}}{\lVert q_{k+1}\rVert}
$$

归一化的原因：数值积分误差使 $\lVert q\rVert\neq1$；非单位四元数不再表示纯旋转。

### 7.5 手算：绕 z 匀速转动

设 $\omega=(0,0,\pi/2)$ rad/s（$90^\circ$/s），$\Delta t=0.1$ s，初始 $q_0=[1,0,0,0]$。

**第 1 步**：$\dot q=\tfrac12(0,0,0,\omega_z\cdot1)=[0,0,0,0.7854]$；
$q=[1,0,0,0.07854]$，范数 $\sqrt{1+0.006169}=1.00308$；归一化得 $[0.99693,0,0,0.07830]$。
yaw $=2\arctan(0.07830/0.99693)=8.982^\circ$（精确值 $9^\circ$，误差 0.2%）。

**第 2 步**起类推。**数值验证（脚本运行结果，教学示例）**：

| 步数 | 时间 (s) | 四元数 $[w,x,y,z]$ | yaw |
|---|---|---|---|
| 1 | 0.1 | [0.99693, 0, 0, 0.07830] | 8.98° |
| 2 | 0.2 | [0.98774, 0, 0, 0.15612] | 17.96° |
| 3 | 0.3 | [0.97248, 0, 0, 0.23298] | 26.95° |
| 10 | 1.0 | [0.70824, 0, 0, 0.70597] | 89.82° |

精确值 90°，1 秒累计误差 0.18°。取 $\Delta t=0.001$ s（1 kHz）时误差会小得多。**真实系统每次转的角度很小**（1 kHz、90 °/s 时每步 0.09°），所以一阶积分足够准；高速转动或低采样率时需要二阶方法或指数映射（精确积分）。

> **含义**：如果算法固定 $\Delta t=0.001$ s 而实际周期是 3 ms，则每步少积了 2/3，上例中 90 °/s 实际会被算成 30 °/s，转了 90° 只显示 30°。这就是 03 章强调"周期必须匹配"的原因。

## 8. 一维互补滤波（手算）

### 8.1 推导

估计一个角度 $\hat\theta$：陀螺仪给角速度 $\omega$，加速度计给角度 $\theta_a$。

$$
\hat\theta_k=\alpha\,(\hat\theta_{k-1}+\omega_k\Delta t)+(1-\alpha)\,\theta_{a,k},\qquad \alpha\in(0,1)
$$

- 括号里是陀螺积分的预测；
- $\theta_a$ 是对绝对角度的观测；
- $\alpha$ 接近 1：更信陀螺。

频域看：陀螺积分通道乘以高通 $\frac{\tau s}{\tau s+1}$，加速度通道乘以低通 $\frac{1}{\tau s+1}$，二者互补，和为 1。其中时间常数和截止频率：

$$
\tau=\frac{\alpha\,\Delta t}{1-\alpha},\qquad f_c=\frac1{2\pi\tau}
$$

**手算**：$\alpha=0.98$，$\Delta t=0.001$ s：$\tau=0.98\times0.001/0.02=0.049$ s；$f_c=1/(2\pi\cdot0.049)=3.25$ Hz。

意义：**低于约 3 Hz 的角度变化相信加速度计，高于 3 Hz 的相信陀螺仪**。机器人快速机动的频率（几十 Hz 的振动、冲击）远高于 3 Hz，所以主要被陀螺仪"屏蔽"；长期偏差被加速度计慢慢拉回。

### 8.2 收敛手算

陀螺仪读数为 0（静止），初始估计 $\hat\theta=0$，加速度计给 $\theta_a=10^\circ$：
- 第 1 步：$0.98\times0+0.02\times10=0.2^\circ$；
- 第 2 步：$0.98\times0.2+0.02\times10=0.396^\circ$；
- 每步误差缩小到 $\alpha$ 倍；误差 $e_n = 10\times 0.98^n$，约 50 步（50 ms）降到 $10\times e^{-1}\approx3.7^\circ$ 即时间常数 $\approx 49$ ms ✓。

## 9. Mahony 滤波：推导与手算

### 9.1 思想

互补滤波的三维推广，在**旋转群**上做：用加速度计测得的重力方向与当前姿态预测的重力方向之间的**叉积**作为误差，反馈修正陀螺仪角速度，再积分四元数。（Mahony, Hamel, Pflimlin 2008，属经典公开算法。）

### 9.2 步骤

设 $q$ 为机体到惯性系的旋转，惯性系 $z$ 轴向上。

**1) 用当前 $q$ 预测重力方向（在机体系）**：

$$
\hat{\mathbf v}=R(q)^T\begin{bmatrix}0\\0\\1\end{bmatrix}
=\begin{bmatrix}2(xz-wy)\\2(wx+yz)\\w^2-x^2-y^2+z^2\end{bmatrix}
$$

**2) 归一化测得的加速度**：$\hat{\mathbf a}=\mathbf a/\lVert\mathbf a\rVert$（静止时指向上）。

**3) 误差 = 测量与预测的叉积**：

$$
\mathbf e=\hat{\mathbf a}\times\hat{\mathbf v}
$$

叉积的方向是"把预测旋转到测量所需的旋转轴"，模长近似为两向量夹角的正弦，所以 $\mathbf e$ 是**小角度下的姿态误差向量**（机体系）。

**4) PI 修正陀螺仪**：

$$
\boldsymbol\omega'=\boldsymbol\omega+K_p\,\mathbf e+K_i\!\int\mathbf e\,dt
$$

积分项用于估计陀螺仪零偏（$K_i\int\mathbf e$ 收敛到 $-b$ 的估计）。

**5) 积分四元数并归一化**：$q\leftarrow q+\tfrac12\,q\otimes[0,\boldsymbol\omega']\Delta t$，归一化。

**6) 若磁力计可用**，再加一个磁场方向的叉积误差，用于修正 yaw；没有磁力计时只用加速度计，**yaw 没有绝对参考**。

### 9.3 手算一步

设机体绕 x 轴横滚了 $10^\circ$：加速度计（静止，单位化）$\hat{\mathbf a}=(0,\sin10^\circ,\cos10^\circ)=(0,0.17365,0.98481)$。当前估计仍是 $q=[1,0,0,0]$，则 $\hat{\mathbf v}=(0,0,1)$。

$$
\mathbf e=\hat{\mathbf a}\times\hat{\mathbf v}
=\begin{bmatrix}a_yv_z-a_zv_y\\a_zv_x-a_xv_z\\a_xv_y-a_yv_x\end{bmatrix}
=\begin{bmatrix}0.17365\\0\\0\end{bmatrix}
$$

取 $K_p=2$，忽略 $K_i$，陀螺仪读数 0：$\boldsymbol\omega'=(0.3473,0,0)$ rad/s。
$\Delta t=0.001$ s：$\dot q\Delta t=\tfrac12[0,0.3473,0,0]\times0.001=[0,1.7365\times10^{-4},0,0]$。
横滚增量 $=2\times1.7365\times10^{-4}=3.47\times10^{-4}$ rad $=0.0199^\circ$。方向为正（朝 $+10^\circ$ 方向），符号正确。

### 9.4 收敛特性

小角度下 $|\mathbf e|\approx\phi_{\text{err}}$（弧度），所以 $\dot\phi=K_p\,\phi_{\text{err}}$，误差以时间常数 $\tau\approx1/K_p$ 指数衰减。

**数值验证（脚本运行，教学示例）**：$K_p=2$，真值横滚 $10^\circ$，从 $0^\circ$ 起步，1 kHz：

| 时间 (s) | 估计横滚 | 剩余误差 | 理论 $10e^{-2t}$ |
|---|---|---|---|
| 0.25 | 3.93° | 6.07° | 6.07° |
| 0.50 | 6.32° | 3.68° | 3.68° |
| 1.00 | 8.65° | 1.35° | 1.35° |

与互补滤波对应：$K_p\approx(1-\alpha)/(\alpha\Delta t)\approx 1/\tau$，截止频率 $f_c\approx K_p/(2\pi)$。$K_p=2$ 对应 $f_c=0.32$ Hz，很慢——适合静止/慢速场合；机器人机动大、加速度计不可靠时宜用小 $K_p$，开机初始化时可临时用大 $K_p$ 快速收敛。

### 9.5 加速度计可信度门控

当 $\lVert\mathbf a\rVert$ 明显偏离 $g$（撞击、急加速、发射反冲），重力方向不可信。常见处理：

- **门控**：$|\lVert\mathbf a\rVert-g|>$ 阈值时，暂时置 $K_p=0$（只靠陀螺仪积分）；
- **低通滤波**加速度计后再进滤波器（但会带来延迟）；
- 根据偏离量连续地缩放 $K_p$。

**手算（教学示例）**：阈值 $0.3g$。读数模长 $1.8g$：偏离 $0.8g>0.3g$，跳过本次修正；$1.1g$：偏离 $0.1g$，使用。

## 10. Madgwick 与 Kalman：对比

| 方法 | 核心 | 参数 | 计算量 | 特点 |
|---|---|---|---|---|
| 互补滤波（1D） | 加权和 | $\alpha$ | 极小 | 只适合单轴/小角度 |
| Mahony | 叉积 + PI 反馈 | $K_p,K_i$ | 小 | 直接作用于陀螺，估计零偏；简单稳定 |
| Madgwick | 梯度下降最小化重力（磁场）误差 | $\beta$ | 小到中 | 一个参数；无显式零偏估计；与 Mahony 性能接近 |
| EKF/ESKF | 状态 = 姿态 + 零偏，协方差传播 | 噪声协方差 | 中到大 | 最优性好，可融合多源；调协方差 |

> **CV 技能栏**：*Kalman/Mahony complementary filtering*。面试里可以说："I used a Mahony-type complementary filter on the MCU because it is cheap, has two intuitive gains, and estimates gyro bias through the integral term. For fusing more sensors, such as IMU with wheel odometry and LiDAR in my navigation project, I used an EKF."（P5 的 EKF 见对应项目文档。）

## 11. 四元数到欧拉角（公式与验算）

采用 Z-Y-X（yaw-pitch-roll）顺序，$q=[w,x,y,z]$，右手系，x 前，y 左，z 上：

$$
\begin{aligned}
\text{yaw}&=\operatorname{atan2}\!\big(2(wz+xy),\ 1-2(y^2+z^2)\big)\\
\text{pitch}&=\arcsin\!\big(2(wy-xz)\big)\\
\text{roll}&=\operatorname{atan2}\!\big(2(wx+yz),\ 1-2(x^2+y^2)\big)
\end{aligned}
$$

注意 $1-2(y^2+z^2)=2(w^2+x^2)-1$（利用 $\lVert q\rVert=1$），所以你会在不同代码里看到两种等价写法。

**验算 1**：$q=[1,0,0,0]$ → 全 0 ✓。

**验算 2**：$q=[0.7071,0,0,0.7071]$：yaw $=\operatorname{atan2}(2\cdot0.5,\ 1-2\cdot0.5)=\operatorname{atan2}(1,0)=90^\circ$，pitch=0，roll=0 ✓。

**验算 3**：$q=[0.5,0.5,0.5,0.5]$：yaw $=\operatorname{atan2}(2(0.25+0.25),\ 1-2(0.25+0.25))=\operatorname{atan2}(1,0)=90^\circ$；pitch $=\arcsin(2(0.25-0.25))=0$；roll $=\operatorname{atan2}(2(0.25+0.25),\ 1-2(0.25+0.25))=90^\circ$。结果 yaw 90°、pitch 0°、roll 90°。

> 这个例子也说明：**两次 90° 转动合成的姿态，用欧拉角读出来并不是"各填各的角"**，而是对合成后姿态的一种读取。

**pitch 的 asin 域**：参数必须夹在 $[-1,1]$，数值误差可能使其略超，需 `fmaxf/fminf` 限制，否则 `asinf` 返回 NaN，NaN 一旦进入 PID 积分就不会恢复。

**yaw 的绕回**：atan2 的输出在 $(-\pi,\pi]$；yaw 从 179° 转到 181° 时显示从 179° 跳到 −179°，物理上没有跳变。控制器必须用"绕回误差"处理（见 06 章）。

> **【笔记线索】**：你的笔记里，源码快照的 `get_angle` 的参数顺序是 yaw、pitch、roll，而云台代码把 `INS_angle[2]` 当作 pitch 使用、把 `INS_gyro[0]` 取负当作 pitch 角速度；笔记指出这"可能与 IMU 安装及轴约定有关"，需要核对。面试遇到这个问题时的诚实回答：*"The index mapping reflects how the IMU was mounted on our gimbal, where the board axes do not match the nominal yaw-pitch-roll order; I verified signs by moving only one axis at a time."*（只有你确实做过这个验证才说。）本项目的做法：pitch 取 INS_angle[2]，pitch 角速度取 -INS_gyro[0]；验证方法是每次只转动一个轴，用小电流确认符号后再闭环。

## 12. yaw 漂移与磁力计

### 12.1 为什么 yaw 会漂

加速度计只能约束绕水平轴的倾斜；绕竖直轴转动不改变重力方向。**没有磁力计（或其他航向参考）时，yaw 只靠陀螺仪积分，零偏会让它缓慢漂移**，且从初始化时的任意值开始（初始四元数常设为 $[1,0,0,0]$，即 yaw=0）。

### 12.2 磁力计为什么常不用

电机、电流线产生强磁干扰，机器人本体钢材也会扭曲磁场；室内场地磁场不均匀。磁力计读数不可靠，反而可能让 yaw 乱跳，因此大量 RoboMaster 电控**不使用磁力计**，yaw 只靠陀螺。【笔记线索】你的笔记指出快照里磁力计读取被注释，磁力计数据为零，AHRS 自动走"只用陀螺仪+加速度计"的分支。

### 12.3 这是不是问题？

**取决于用途**：

1. **云台 yaw 控制（相对比较）**：只要在比赛时间尺度内漂移足够小，就不影响。零偏经良好标定后残余 $\sim0.01\ ^\circ$/s，7 分钟比赛漂移约 $0.01\times420=4.2^\circ$（教学示例）。
2. **视觉自瞄**：视觉给出的是"目标相对当前枪口的偏差"或带时间戳的相对量，用当前姿态做局部累加，不依赖全局 yaw，漂移无影响。
3. **需要全局航向（导航、地图）**：必须有外部参考（磁力计、LiDAR 定位、视觉里程计等），这是 P5 项目用 EKF/AMCL 融合的原因。

### 12.4 缓解手段

- 精确标定零偏、保持 IMU 温度恒定；
- 静止检测下更新零偏；
- 每局开始/复位时把 yaw 设为零（相对参考）；
- 使用编码器的底盘-云台相对角辅助：底盘姿态估计与云台关节角合成。

> **面试要点**：*"Without a magnetometer yaw is unobservable from gravity, so it drifts with gyro bias. For gimbal stabilisation and relative vision targets this is acceptable over a match, and we mitigated it with calibration and temperature control. For global heading you'd need an external reference."*

## 13. 云台坐标系、符号与投影

### 13.1 坐标系定义（本文约定）

- **惯性系 W**：z 向上，x 取开机时机体前方，右手系；
- **机体系 B（IMU 所在刚体）**：x 前（枪口方向），y 左，z 上；
- **yaw**：绕 z 逆时针（俯视）为正；**pitch**：绕 y 的右手系正方向是"低头"（枪口向下）；很多云台约定"抬头为正"，需要取负。**这是最常见的符号坑**。

| 角 | 轴 | 右手系正方向 | 常见工程约定 |
|---|---|---|---|
| yaw | z | 逆时针（俯视） | 同 |
| pitch | y | 低头 | 常改为抬头为正 |
| roll | x | 右倾（右手系：y 向 z 转） | 同 |

### 13.2 IMU 装在哪

- **装在 pitch 轴负载上（云台末端）**：IMU 直接测到枪口姿态；pitch 与 yaw 角速度需要投影到各自的转轴（见 13.3）；
- **装在 yaw 轴底座（云台下方）**：只能测到 yaw 轴一侧的姿态，pitch 需要加编码器角度组合。

**【笔记线索】**：你的笔记里，源码把"本板 IMU 的 pitch + 底盘板发来的 pitch"相加作为 pitch 反馈（`relative_up_down_get`），并强调这是代码组合量，变量名里的 "relative" 不能直接等同于"只由 pitch 电机编码器得到的相对角"。这类组合的物理意义取决于两块板各自的轴约定，若符号相反则"加"实际代表"减"。本项目的做法：IMU 装在云台板上，pitch 反馈 = 本板 IMU 的 pitch + 底盘板 IMU 的 pitch（经板间 0x302 帧传来），两路符号约定按上面的轴映射统一后再相加。

### 13.3 俯仰非零时 yaw 角速度的投影问题（手算）

IMU 装在枪口所在的刚体上。设 roll≈0，pitch 为 $\theta$（右手系，低头为正）。云台只绕**竖直轴**以 $\Omega$ 旋转时，机体系角速度为：

$$
p=-\Omega\sin\theta,\qquad q=0,\qquad r=\Omega\cos\theta
$$

所以陀螺仪 z 轴读数 $r=\Omega\cos\theta$，**低估了真实 yaw 角速度**。正确的投影（竖直轴方向在机体系是 $(-\sin\theta,0,\cos\theta)$）：

$$
\Omega_{\text{yaw}}=r\cos\theta-p\sin\theta
$$

**手算**：$\theta=20^\circ$，$\Omega=1$ rad/s：$p=-0.342$，$r=0.9397$。直接用 $r$ 得 0.9397（低 6.0%）；投影后 $0.9397\times0.9397+0.342\times0.342=0.883+0.117=1.000$ ✓。

**含义**：pitch 越大，直接用陀螺仪 z 轴做 yaw 速度环反馈的偏差越大。多数工程在 pitch 范围小时忽略（$\cos20^\circ=0.94$，对应速度环增益少 6%，对稳定性影响不大）。需要高精度/大 pitch 时才做投影。本项目取值：pitch 摆幅不大，yaw 速度反馈直接取陀螺 z 轴、不做投影，这是已知的简化。

### 13.4 四元数与云台目标的关系

视觉、遥控给的目标是"yaw/pitch 目标角度"，角度环比较目标和当前欧拉角。这要求**目标角与反馈角用同一约定**（同一符号、同一零点、同一单位）。出现"云台往反方向跑""视觉一介入就偏"的第一怀疑对象就是约定不一致。

## 14. 在本项目里怎么用

- **【CV】**：*using IMU/AHRS and motor feedback*；技能栏 *Kalman/Mahony complementary filtering*。
- **【笔记线索】**（需自行核对）：
  1. BMI088 经 SPI + DMA 读取，INS 任务由 DMA 通知唤醒；
  2. 原始数据 → 安装矩阵（x←y，y←−x，z←z）和零偏处理 → 加速度低通 → AHRS；
  3. AHRS 里 `sampleFreq` 为 1000 Hz，积分步长按 1 ms 固定；初始四元数设为 $[1,0,0,0]$，初始化函数没有用加速度/磁力计求初始姿态；
  4. 磁力计数据为零，走 IMU-only 更新（陀螺 + 加速度，重力方向误差修正）；
  5. `get_angle` 把四元数转欧拉角（弧度），云台再换成度；
  6. 云台角速度反馈用陀螺仪（经滤波），角度反馈用 IMU 姿态角，pitch 与底盘板的 pitch 合成。
- **典型做法对照**：Mahony 类 AHRS；INS 高优先级；测量实际 IMU 采样间隔后更新步长。
- **本项目取值**：BMI088（陀螺 ±2000 °/s、加速度计 ±6 g），陀螺数据率 1 kHz 量级；上电静止采样 2000 次求陀螺零偏，加热电阻 PID 恒温 40 °C；Mahony，Kp=0.5、Ki=0；内部 dt 写死 0.001（名义常数）；yaw 静置漂移约 1 °/min 量级；轴映射为 pitch=INS_angle[2]、pitch 角速度=-INS_gyro[0]，逐轴验证符号。

> **关键自洽性提醒**：笔记指出 AHRS 内部用固定 1 ms 步长。如果 INS 任务实际按 IMU 数据率（例如 1 kHz）被通知唤醒，这个假设是合理的；如果数据率不同，角度积分会按比例失真。检验方法：让云台绕 z 轴匀速转一圈（例如用电机速度环做 90 °/s），看 IMU yaw 是否也走了 90°/s。

## 15. 常见错误与故障

| # | 现象 | 原因 | 定位 | 处理 |
|---|---|---|---|---|
| 1 | 静止时 yaw 缓慢漂移 | 陀螺零偏 | 记录静止下 yaw 变化率 | 重新标定；恒温；ZUPT |
| 2 | 上电后云台姿态读数不为 0 或慢慢"爬" | 初始四元数不对；初始化时机器在运动 | 看开机前 5 秒读数 | 初始化期间保持静止，加大 $K_p$ 快速对准 |
| 3 | 单轴转动时其他轴读数也变 | 安装矩阵错误、轴符号错 | 每次只转一个轴 | 修正矩阵，验证行列式为 +1 |
| 4 | 转得快时角度偏小 | 积分步长与实际周期不符、陀螺量程饱和 | 对比转速与角度变化 | 用实测 dt；提高量程 |
| 5 | 撞击/发射后姿态短暂错乱 | 加速度计被冲击干扰 | 看冲击时 $\lVert a\rVert$ | 门控；低通 |
| 6 | pitch 出现 NaN | `asin` 参数超出 $[-1,1]$ | 打印中间值 | 夹紧；检测 NaN 并复位 |
| 7 | yaw 在 ±180° 附近云台甩头 | 目标-反馈直接相减未绕回 | 观察过零瞬间 | 误差绕回（06 章） |
| 8 | 云台往反方向转动、发散 | 正负号/单位/约定不一致 | 小幅点动检查 | 统一约定；符号集中放在一处 |
| 9 | 四元数分量顺序混淆 | wxyz 与 xyzw | 对比库文档 | 统一顺序并加单元测试 |
| 10 | 陀螺饱和时姿态跑飞 | 量程过小 | 看原始读数是否碰顶 | 选 ±2000 °/s |

## 16. 面试怎么问、怎么答

**Q1. How do you get the gimbal attitude?**
- 英文：*"I read the gyroscope and accelerometer over SPI, remove the gyro bias that I estimate at start-up, rotate the data into the gimbal frame with a fixed mounting matrix, and run a Mahony-style complementary filter to update a quaternion. Then I convert it to yaw, pitch and roll for the control loops."*
- 追问 1：*Why a quaternion and not integrating Euler angles?* → 避免万向锁、积分更稳、计算简单；欧拉角速率与机体角速度关系非线性且在 pitch=±90° 奇异。
- 追问 2：*What does the accelerometer do and when does it fail?* → 提供重力方向修正倾斜；加速/撞击/反冲时失效，需要门控或低通。
- 追问 3：*Why does yaw drift?* → 重力无法约束 yaw，没有磁力计，只靠陀螺积分，零偏导致漂移。

**Q2. Explain Mahony in 30 seconds.**
- 英文：*"Predict the gravity direction from the current quaternion, compare it with the measured accelerometer direction using a cross product, and feed that error back, proportionally and with an integral term, into the gyro rates before integrating the quaternion. The proportional gain sets how fast the tilt converges, and the integral term learns the gyro bias."*
- 追问：*How do you choose Kp?* → 对应互补滤波截止频率 $f_c\approx K_p/2\pi$；机动大、加速度计差就小一些，开机对准时临时放大；用静态收敛时间和动态扰动试验验证。

**Q3. Mahony vs Madgwick vs Kalman?**
- 英文：*"Mahony and Madgwick are cheap nonlinear complementary filters with one or two gains. A Kalman filter estimates states and covariance and weights sensors optimally, but needs noise models and more computation. On the gimbal MCU I used the cheap one. In my navigation project, where I had to fuse several sensors with different rates, I used an EKF."*

**Q4. How do you calibrate the gyro?**
- 英文：*"Keep the robot still, check the variance and the gravity magnitude to confirm it is still, average a couple of thousand samples, and subtract that mean. Averaging N samples reduces the estimate noise by the square root of N. Temperature matters, so a heated, thermally stable IMU helps."*

**Q5. What frames and signs did you have to take care of?**
- 英文：*"The IMU axes differ from the gimbal axes, so there is a fixed mounting rotation. I checked that its determinant is plus one, then moved one axis at a time to verify signs of angle and rate. The pitch sign convention also had to match between the target source and the feedback."*

**Q6. Does the yaw rate from the gyro equal the yaw axis rate?**
- 英文：*"Only when pitch is zero. If the IMU sits on the pitched link, the vertical axis in the body frame is tilted, so the yaw-axis rate is r cosθ minus p sinθ. For small pitch the error is a few percent, which I accepted."*（仅当你确实这样处理时才说 "I accepted"；否则说 "one could project it".）

## 本章要点速记

1. 陀螺仪测角速度（短时准、长时漂），加速度计测重力方向（长时准、短时噪、不含 yaw）。
2. 零偏误差随时间线性累积为角度漂移：0.1 °/s → 6°/min。
3. 安装矩阵必须是行列式 +1 的旋转；上闭环前必须逐轴验证符号。
4. 四元数运动学 $\dot q=\frac12 q\otimes[0,\omega]$，离散积分后必须归一化；一阶积分对小步长足够。
5. 积分步长必须与实际采样周期一致，否则角度按比例失真。
6. 互补滤波 $\tau=\alpha\Delta t/(1-\alpha)$；Mahony 是其三维推广，$K_p$ 对应截止频率。
7. Mahony：重力预测 → 与测量叉积 → PI 修正陀螺 → 积分四元数 → 归一化。
8. 加速度计在加速/撞击时不可信，需门控。
9. yaw 无重力约束，会漂；磁力计常因干扰不用；漂移对相对控制通常可接受，对全局航向不可接受。
10. 四元数转欧拉：yaw=atan2(2(wz+xy), 1−2(y²+z²))，pitch=asin(2(wy−xz))，roll=atan2(2(wx+yz), 1−2(x²+y²))。
11. pitch 非零时，yaw 轴角速度应投影：$r\cos\theta-p\sin\theta$。
12. 符号/单位/零点约定不一致是云台"反转、发散、甩头"的头号原因。

## 自测题

1. 陀螺仪 ±2000 °/s、16 位，读数 −820，对应多少 rad/s？
   *答：−820×0.061=−50.0 °/s = −0.873 rad/s。*
2. 静止时三轴加速度计读数 (0, 0, 9.8)；横滚 30° 后读数大致是多少？
   *答：(0, 9.8 sin30°, 9.8 cos30°) = (0, 4.9, 8.49)。*
3. 互补滤波 α=0.995，Δt=0.002 s，τ 与 fc 各多少？
   *答：τ=0.995×0.002/0.005=0.398 s；fc=1/(2π·0.398)=0.40 Hz。*
4. 用 $q=[0.9239,0,0.3827,0]$ 求 pitch。
   *答：asin(2(wy−xz))=asin(2×0.9239×0.3827)=asin(0.7071)=45°。*
5. 为什么 yaw 只靠陀螺会漂，而 roll/pitch 不会？
   *要点：重力给 roll/pitch 观测，对 yaw 不可观。*
6. 解释"采样周期实际是 3 ms 而算法用 1 ms 步长"会产生什么结果。
   *要点：角速度积分被缩小 3 倍，实际转 90° 只显示 30°。*
7. 四元数 $[1,0,0,0]$ 乘 $[0.7071,0.7071,0,0]$ 的结果是什么？有什么意义？
   *答：得 $[0.7071,0.7071,0,0]$（单位元），说明乘单位四元数不改变旋转。*
8. 写出你确认 IMU 安装矩阵正确的两个检查。
   *要点：行列式 +1；逐轴转动看符号。*

## 相关链接

- 上一章：[04 CAN 总线与电机反馈](./04_CAN总线与电机反馈.md)
- 下一章：[06 云台串级 PID 与视觉跟踪](./06_云台串级PID与视觉跟踪.md)
- 基础：[互补滤波与 Mahony AHRS](../../02_基础知识/06_状态估计/02_互补滤波与Mahony_AHRS.md)、[欧拉角、四元数与姿态表示](../../02_基础知识/03_运动学与坐标变换/02_欧拉角_四元数_姿态表示.md)、[卡尔曼滤波与 EKF](../../02_基础知识/06_状态估计/03_卡尔曼滤波与EKF.md)
- 八股：[状态估计与 SLAM 八股](../../03_八股知识/03_机器人学/03_状态估计与SLAM八股.md)
- 学习笔记：[P3 本对话问答逐字稿](../../05_学习笔记归档/P3/P3_本对话问答逐字稿.md)（第 5～11 组讲 SPI/DMA、四元数、get_angle）
