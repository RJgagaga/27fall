# 传感器：IMU、编码器与 2D 激光雷达

> **一句话定位**：状态估计的第一步不是写滤波器，而是搞清楚"传感器到底测了什么、带着哪些误差、用什么单位、在哪个坐标系里"。本章把 P5 导航里用到的三类传感器——IMU（陀螺仪 + 加速度计 + 磁力计）、轮速编码器、2D 激光雷达——的原理和误差模型讲到能自己动手算。

## 本章学完你能……

- 用一个统一的"测量模型"（真值 × 尺度 + 零偏 + 噪声）描述任何传感器，并说出每一项的物理来源。
- 解释陀螺仪为什么会"漂"，加速度计为什么在运动中测不出重力方向，磁力计为什么怕铁。
- 算出：零偏积分后的角度误差、线加速度造成的假倾角、编码器每个计数对应的距离、M 法 / T 法的测速分辨率。
- 把 2D 激光雷达的一帧 `ranges[]` 还原成 (x, y) 点云，并手算一次最简单的扫描匹配（刚体配准）。
- 说清楚 ROS 约定（REP-103）：x 向前、y 向左、z 向上，角度单位 rad，以及单位混用会造成什么事故。
- 用英文回答"IMU 有哪些误差？""编码器怎么测速？""LiDAR 噪声怎么影响定位？"这类问题。

## 前置知识

- 概率里的均值、方差、高斯分布：[概率统计与高斯分布](../01_数学/03_概率统计与高斯分布.md)
- 旋转矩阵与坐标系：[坐标系、旋转矩阵、齐次变换](../03_坐标变换与运动学/01_坐标系_旋转矩阵_齐次变换.md)
- 差速底盘的速度与轮速关系：[差速、麦克纳姆、全向轮与车辆运动学](../02_力学与建模/04_差速_麦克纳姆_全向轮与车辆运动学.md)

## 目录

1. 传感器的统一测量模型
2. 坐标系、符号与单位
3. 陀螺仪（gyroscope）
4. 加速度计（accelerometer）
5. 磁力计（magnetometer）
6. IMU 误差汇总与标定
7. 编码器（encoder）与测速
8. 2D 激光雷达（LiDAR）与扫描匹配
9. 在本项目里怎么用
10. 常见错误与故障现象
11. 面试怎么问、怎么答
12. 本章要点速记、自测题、相关链接

---

## 1. 传感器的统一测量模型

### 1.1 它要解决什么问题

机器人要知道"我转得多快""我在往哪个方向加速""轮子转了几圈""前面 2 米有墙吗"。传感器给出的是**一个带误差的数字**，不是真值。如果不知道误差长什么样，后面的滤波器就没有依据去分配信任。

所以第一件事是给每个传感器写一个**测量模型**（measurement model）。几乎所有传感器都可以套下面这个形式：

$$
z(t) = s\cdot x(t) + b(t) + n(t)
$$

### 1.2 符号与单位

| 符号 | 含义 | 说明 |
|---|---|---|
| $z$ | 传感器读数 | 单位取决于传感器，例如 rad/s、m/s²、计数 |
| $x$ | 被测量的真值 | 例如真实角速度 |
| $s$ | 尺度因子（scale factor） | 理想为 1；$s=1.005$ 表示读数偏大 0.5% |
| $b$ | 零偏（bias） | 真值为 0 时读数不为 0 的部分；会缓慢变化 |
| $n$ | 噪声（noise） | 每次采样都不同，常近似为零均值白噪声，标准差 $\sigma$ |

### 1.3 四类误差的物理直觉

1. **零偏**：像一个没有归零的秤，空载时显示 0.02。它不随采样随机变化，而是"一直偏同一个方向"，所以对它积分会越积越大。
2. **噪声**：像手抖。每次读数围绕真值上下抖，平均后会抵消，所以它对积分的影响是"随机游走"，增长慢于零偏。
3. **温漂**：零偏 $b$ 本身随温度变化，$b=b(T)$。MEMS 器件上电后芯片升温，零偏会跟着走，所以"上电即标定"不够，需要等热稳定或做温度补偿。
4. **尺度因子误差**：读数和真值成比例地错。旋转一圈读成 361.8°，就是 $s=1.005$。它只在运动时才出现，静止时看不见。

> **提示**：区分这四类误差，就是在回答"能不能标定掉""能不能靠平均消掉""滤波器需要在线估计什么"。零偏可以在线估计（Mahony 的 PI 项、EKF 的零偏状态）；噪声只能靠滤波平滑；尺度因子一般靠离线标定。

### 1.4 一个完整的手算：零偏积分

**示例**（教学数值，非项目实测）：陀螺仪静止放在桌上，真值 $\omega=0$，零偏 $b=0.02\ \text{rad/s}$（约 1.15°/s），采样 100 Hz，$\Delta t=0.01$ s，忽略噪声。

对读数做积分得到角度：

$$
\hat\theta(T) = \int_0^T z\,dt = b\,T
$$

- $T=1$ s：$0.02\ \text{rad} = 1.15°$
- $T=10$ s：$0.20\ \text{rad} = 11.5°$
- $T=60$ s：$1.2\ \text{rad} = 68.8°$

一分钟就偏了接近 70°。这就是"陀螺仪只积分不修正会漂"的定量含义。

再看噪声：设每次采样噪声标准差 $\sigma_\omega = 0.1\ °/\text{s}$，每步角度增量噪声为 $\sigma_\omega\Delta t = 0.001°$。$n$ 步独立噪声累加，标准差按 $\sqrt n$ 增长：

$$
\sigma_\theta(n) = \sigma_\omega\,\Delta t\,\sqrt n
$$

$n=100$（1 s）时 $=0.001\times10=0.01°$。对比：零偏 1 s 积出 1.15°，噪声只有 0.01°。**短时间内零偏远比白噪声危险。**

---

## 2. 坐标系、符号与单位

### 2.1 它要解决什么问题

同一个数字，在不同坐标系里意义完全不同。陀螺仪的 z 轴转速，是绕机体的竖直轴，还是绕世界的竖直轴？两者在车身平放时一样，车身倾斜后就不同。P5 与 P6 里"坐标系不一致造成位姿跳变"的根源，就是这里。

### 2.2 常用坐标系与 ROS 约定

ROS 的 REP-103 规定（以下为通用约定）：

- 右手系；**x 向前，y 向左，z 向上**。
- 角度用弧度（rad），角速度 rad/s，线加速度 m/s²，距离 m，时间 s。
- 绕 z 轴逆时针（从上往下看）为正偏航（yaw）。

ROS 里与本章相关的坐标系：

```text
map        世界固定坐标系，全局一致，但可能有跳变（AMCL 校正）
odom       里程计坐标系，连续但会漂移
base_link  固定在车体上（常取后轴中心或车体中心）
imu_link   IMU 芯片自己的坐标系（相对 base_link 有安装外参）
laser      激光雷达坐标系（相对 base_link 有安装外参）
```

> **注意**：IMU 芯片的数据手册常用 x 向前、y 向右、z 向下（NED 习惯）或其他轴向；板子焊接方向也不一定与车身对齐。**一定要在驱动里用安装外参把数据旋转到 `base_link`**，否则 yaw 的正负会反。

### 2.3 单位陷阱

| 传感器常见输出 | ROS 标准 | 换算 |
|---|---|---|
| 陀螺仪 °/s（deg/s） | rad/s | 乘 $\pi/180\approx0.017453$ |
| 加速度 g（1 g ≈ 9.80665 m/s²） | m/s² | 乘 9.80665 |
| 磁场 Gauss 或 µT | Tesla | 1 Gauss = $10^{-4}$ T = 100 µT |
| 编码器 计数（counts） | m 或 rad | 乘每计数的距离 / 角度 |
| LiDAR 距离 mm 或 cm | m | 除以 1000 或 100 |

**典型事故**：把 °/s 当成 rad/s 直接积分，yaw 估计会放大 57.3 倍，车辆原地乱转，滤波器的一致性检验立刻爆表。

---

## 3. 陀螺仪（gyroscope）

### 3.1 它要解决什么问题

测量机体绕自身三个轴的**角速度** $\boldsymbol\omega=(\omega_x,\omega_y,\omega_z)^\top$。积分一次就是姿态变化。它的优点是响应快、短时精度高，缺点是会漂。

### 3.2 原理（MEMS 振动陀螺）

MEMS 陀螺的核心是一个被驱动着持续振动的微小质量块。机体转动时，质量块受到**科里奥利力**（Coriolis force）：

$$
\mathbf F_c = -2m\,\boldsymbol\omega\times\mathbf v
$$

其中 $m$ 是质量块质量，$\mathbf v$ 是它的振动速度。科里奥利力使质量块在垂直于振动的方向产生微小位移，这个位移被电容检测并转成电压，正比于角速度 $\omega$。注意这个原理决定了：陀螺**直接测角速度，没有任何"绝对姿态"信息**。

### 3.3 陀螺误差模型

$$
\omega_m = (1+s_g)\,\omega + b_g(T) + n_g,\qquad \dot b_g = n_{bg}
$$

最后一式表示零偏本身是**随机游走**（bias random walk）：零偏会缓慢漂移，驱动噪声 $n_{bg}$ 是白噪声。

三个常用的性能指标：

- **零偏不稳定性（bias instability）**：零偏在一段时间内的最小漂移量，单位 °/h。越小越好。
- **角度随机游走（Angle Random Walk, ARW）**：白噪声积分后的角度不确定度，单位 °/√h。对应前面的 $\sigma_\theta\propto\sqrt t$。
- **量程与带宽**：量程不够时快速转动会饱和，数据"削顶"。

这些指标一般通过 **Allan 方差**（Allan variance）从长时间静置数据里读出：把 Allan 偏差对平均时间画双对数图，斜率 −1/2 的部分对应 ARW，平底部分对应零偏不稳定性，斜率 +1/2 的部分对应零偏随机游走。

### 3.4 零偏的静态标定

最简单的做法：车辆完全静止 5 到 10 秒，取每轴读数均值作为 $\hat b_g$，之后每次读数先减去它：

$$
\hat b_g = \frac1N\sum_{k=1}^N \omega_{m,k},\qquad \omega_{\text{corrected}} = \omega_m - \hat b_g
$$

**手算（示例）**：静止采到 5 个 z 轴读数（rad/s）：0.021, 0.019, 0.020, 0.022, 0.018。均值 $=(0.021+0.019+0.020+0.022+0.018)/5=0.020$，样本标准差约 0.0016。所以 $\hat b_g=0.020$，$\sigma\approx0.0016$ rad/s。

局限：标定完成后零偏还会随温度和时间变，所以这只是**初值**，长期使用仍要在线估计（见[互补滤波与 Mahony](./02_互补滤波与Mahony_AHRS.md)的 PI 项）。

### 3.5 陀螺在项目中的角色

- 积分得到短时姿态 / yaw 增量；
- 在 EKF 里作为 yaw 速率观测（P5）；
- 在云台 / 平衡类项目里作为内环角速度反馈（P1、P3、P7）。

---

## 4. 加速度计（accelerometer）

### 4.1 它要解决什么问题

测量机体受到的**比力**（specific force）。静止时它能"闻到重力方向"，所以可以给出横滚、俯仰（roll、pitch）的绝对参考。但它**分不清重力和运动加速度**。

### 4.2 原理

MEMS 加速度计内部是一个弹簧—质量块，质量块偏移量与受力成正比。关键是它测到的是

$$
\mathbf f_b = \mathbf a_b - \mathbf g_b
$$

即**机体实际加速度减去重力加速度**（比力）。静止时 $\mathbf a=0$，所以读数 $=-\mathbf g$，方向竖直向上，大小 $\approx1\ \text{g}$。这就是为什么水平静置的 IMU 读数是 $(0,0,+9.81)$ 而不是 $0$；自由落体时读数反而是 0。

### 4.3 符号

| 符号 | 含义 |
|---|---|
| $\mathbf f_b=(f_x,f_y,f_z)$ | 机体系下加速度计读数（比力），m/s² |
| $\mathbf a_b$ | 机体系下真实线加速度 |
| $\mathbf g$ | 重力加速度，大小 $g\approx9.81$ m/s² |
| $\phi$（roll）、$\theta$（pitch） | 横滚、俯仰角，rad |

### 4.4 推导：静止时由加速度计求 roll 与 pitch

设世界系 z 向上，重力方向的"上"向量在世界系为 $(0,0,1)$。机体绕 x 转 $\phi$（roll）、绕 y 转 $\theta$（pitch）后，"上"向量在机体系中的表示是旋转矩阵的转置乘 $e_3$。按 ZYX（yaw-pitch-roll）约定，机体系里的"上"向量是

$$
\mathbf u_b = \begin{bmatrix} -\sin\theta \\ \sin\phi\cos\theta \\ \cos\phi\cos\theta \end{bmatrix}
$$

静止时加速度计读数 $\mathbf f_b = g\,\mathbf u_b$。所以：

$$
f_x=-g\sin\theta,\quad f_y=g\sin\phi\cos\theta,\quad f_z=g\cos\phi\cos\theta
$$

由第二、三式相除：$f_y/f_z=\tan\phi$，得

$$
\phi=\operatorname{atan2}(f_y,f_z)
$$

再由 $f_y^2+f_z^2=g^2\cos^2\theta$ 与 $f_x=-g\sin\theta$：

$$
\theta=\operatorname{atan2}\!\left(-f_x,\ \sqrt{f_y^2+f_z^2}\right)
$$

**注意：yaw 不出现在公式里。** 重力方向绕竖直轴旋转不变，所以加速度计对 yaw **不可观**。

### 4.5 手算

**例 1（静止，示例）**：读数 $(f_x,f_y,f_z)=(0,\ 1.703,\ 9.659)\ \text{m/s}^2$。

$$
\phi=\operatorname{atan2}(1.703,9.659)=\arctan(0.1763)=10.0°,\quad \theta=\operatorname{atan2}(0,\ 9.808)=0°
$$

**例 2（运动污染，示例）**：车辆水平行驶，以 $a=2\ \text{m/s}^2$ 向前加速，真实 pitch $=0$。机体 x 向前，所以加速度计 $f_x = a_x - g_x = 2 - 0 = 2$（水平时重力在 x 轴分量为 0），$f_z=9.81$。按静止公式反推：

$$
\theta_{\text{apparent}}=\operatorname{atan2}(-2,\ 9.81)=-11.5°
$$

车根本没有俯仰，却被估成 −11.5° 的"假倾角"。结论：**加速度计只能在"加速度小"的时候给倾角做参考**；融合算法必须做"可信度判断"，例如当 $\big|\,\|\mathbf f\|-g\,\big|$ 超过阈值时降低加速度计的权重。

### 4.6 误差模型与常见问题

$$
\mathbf f_m = (I+S_a)\,\mathbf f_b + \mathbf b_a + \mathbf n_a
$$

- 零偏 $\mathbf b_a$：静止放平时读数应为 $(0,0,g)$，偏差就是零偏。**用六面法标定**：把 IMU 六个面依次朝上静置，每次取均值，用 $\pm g$ 的对称性分离零偏和尺度。
- 轴间不正交与安装误差：$S_a$ 的非对角元。
- 振动：电机和路面振动让读数噪声很大；常见做法是硬件低通 + 软件低通。
- **用加速度计积分求速度 / 位置极不可靠**：假设零偏仅 $0.05\ \text{m/s}^2$，则位置误差 $\tfrac12 b t^2$：$t=10$ s 时 $=0.5\times0.05\times100=2.5$ m。这是 [里程计与传感器融合](./04_里程计与传感器融合.md) 里"IMU 积分不能直接当里程计"的数量依据。

---

## 5. 磁力计（magnetometer）

### 5.1 它要解决什么问题

陀螺和加速度计都无法给出绝对 yaw（陀螺会漂，加速度计 yaw 不可观）。磁力计测地磁场，相当于一个电子罗盘，可以给 yaw 一个绝对参考。

### 5.2 倾斜补偿（tilt compensation）

磁力计在机体系中测得 $\mathbf m_b=(m_x,m_y,m_z)$。如果机体有 roll $\phi$、pitch $\theta$，需要先把它旋转到"水平面"再取方向：

$$
m_{x}^{h}=m_x\cos\theta+m_y\sin\phi\sin\theta+m_z\cos\phi\sin\theta
$$

$$
m_{y}^{h}=m_y\cos\phi-m_z\sin\phi
$$

$$
\psi_{\text{mag}}=\operatorname{atan2}(-m_y^h,\ m_x^h)
$$

**手算（示例）**：水平放置（$\phi=\theta=0$），读数 $(m_x,m_y)=(20,-20)\ \mu\text{T}$。则 $\psi=\operatorname{atan2}(20,20)=45°$。（这里的符号约定是 x 向前、y 向左、磁北在地平面内；不同芯片轴向和坐标约定不同，使用前必须在实机上转动确认。）

此外，磁北与地理北有一个**磁偏角**（declination），随地点变化，需要加上。

### 5.3 误差：硬铁与软铁

- **硬铁**（hard iron）：车上永磁体、电机磁铁、电池电流产生的固定磁场，表现为读数圆心偏移 $\mathbf o$。
- **软铁**（soft iron）：钢结构扭曲了磁场，表现为圆变成椭圆。

标定：转动传感器一圈，采样点应落在一个以原点为中心的球面上；实际落在偏移的椭球上，拟合椭球参数得到 $\mathbf o$ 和矩阵 $W$：$\mathbf m_{\text{cal}}=W(\mathbf m-\mathbf o)$。

> **注意**：室内、钢结构、大电流电机附近，磁力计几乎不可信。轮式机器人室内导航通常**不依赖磁力计**，yaw 由陀螺 + 激光 / 地图匹配提供。

---

## 6. IMU 误差汇总与标定

| 误差 | 陀螺 | 加速度计 | 能否标定 | 在线估计 |
|---|---|---|---|---|
| 零偏 | $b_g$，°/s 量级 | $b_a$，mg 量级 | 静态均值 | 需要（滤波状态） |
| 噪声 | ARW | VRW（速度随机游走） | 只能统计 | 滤波平滑 |
| 温漂 | 随 T 变 | 随 T 变 | 温度表补偿 | 一般不做 |
| 尺度因子 | $s_g$ | $S_a$ | 转台 / 六面法 | 一般不做 |
| 轴间不正交 | 有 | 有 | 离线标定 | 否 |
| 安装外参 | $R_{b}^{imu}$ | 同左 | 测量 / 标定 | 否 |

**典型流程**：上电静置 → 取均值作零偏初值 → 开始积分 → 融合滤波在线修零偏。

---

## 7. 编码器（encoder）与测速

### 7.1 它要解决什么问题

轮子转了多少、转多快，是里程计最直接的信息来源。编码器把轴的转角转成数字脉冲。

### 7.2 分类

- **增量式编码器**（incremental）：输出 A、B 两路相位差 90° 的方波（正交编码）。计数的**变化量**表示角度变化，A 与 B 谁在前决定方向。上电后没有绝对位置。
- **绝对式编码器**（absolute）：每个角度对应唯一数码，上电即知位置。电机常用磁编（如 12–14 bit 单圈）。多圈需要电池保存圈数或齿轮机构。

### 7.3 正交解码与分辨率

设编码器线数（PPR，每转脉冲数）$=P$。四倍频（4x，A、B 两路的上升沿 + 下降沿都计数）后每转计数 $N_{rev}=4P$。再考虑减速比 $i$（电机转 $i$ 圈，轮转 1 圈），每转轮的计数：

$$
N_{wheel}=4P\cdot i
$$

轮半径 $r$，则每个计数对应的行驶距离：

$$
d_{count}=\frac{2\pi r}{N_{wheel}}
$$

**手算（示例）**：轮半径 $r=0.05$ m，轮端每转 2000 计数（已含四倍频和减速比）。

$$
d_{count}=\frac{2\pi\times0.05}{2000}=1.5708\times10^{-4}\ \text{m}=0.157\ \text{mm}
$$

### 7.4 测速：M 法、T 法、M/T 法

**M 法（测频法）**：固定采样周期 $T_s$，数这段时间内的计数 $\Delta N$：

$$
v=\frac{\Delta N\cdot d_{count}}{T_s}
$$

速度分辨率是**一个计数**对应的速度：$d_{count}/T_s$。

- 示例：$T_s=10$ ms：$1.5708\times10^{-4}/0.01=0.0157\ \text{m/s}$。
- 速度 $0.1$ m/s 时，每个周期只数到约 6.4 个计数，量化误差 $\pm1/6.4\approx\pm16\%$。**低速 M 法误差大。**

**T 法（测周法）**：测相邻两个脉冲之间的时间间隔 $T$（用高频定时器计 $f_{clk}$ 的节拍数 $M$，$T=M/f_{clk}$）：

$$
v=\frac{d_{count}}{T}=\frac{d_{count}\,f_{clk}}{M}
$$

- 示例：$v=0.1$ m/s，$f_{clk}=1$ MHz。脉冲间隔 $=1.5708\times10^{-4}/0.1=1.5708$ ms $=1571$ 个节拍。分辨率 $1/1571\approx0.064\%$，**低速精度好**。
- 高速时脉冲间隔太短，节拍数少，误差反而大（例如 $v=3$ m/s，间隔 52 µs，只有 52 节拍，分辨率 1.9%）。

**M/T 法**：固定一个采样窗口，同时计数窗口内的脉冲数 $\Delta N$ 和**第一个到最后一个脉冲之间**的精确时间 $T_{eff}$，$v=\Delta N\cdot d_{count}/T_{eff}$。高低速都有较好精度，是工程上最常用的折中。

| 方法 | 低速 | 高速 | 实现复杂度 |
|---|---|---|---|
| M 法 | 差 | 好 | 最简单（定时读计数器） |
| T 法 | 好 | 差 | 需要输入捕获 |
| M/T 法 | 好 | 好 | 较复杂 |

实践中，轮速再经过一阶低通或滑动平均，以牺牲少量延迟换更平滑的速度。

### 7.5 计数溢出与方向

- 硬件计数器是 16 位时每 65536 计数溢出一次；读增量时用**有符号差值**处理回绕：`int16_t delta = (int16_t)(now - last);`。
- 正交编码天然带方向。若使用**单路霍尔**测速，则**没有方向信息**，倒车时速度符号要靠指令 / 档位推断，否则里程计往错误方向积分（见[里程计与传感器融合](./04_里程计与传感器融合.md)）。

### 7.6 误差来源

- 轮径误差（轮胎磨损、载荷压缩）：距离误差与半径误差同比例。
- 轮距（track width）误差：直接影响转角估计。
- 打滑、悬空、轮子与地面的蠕变：编码器仍在数，但车没走。
- 量化误差：$\pm1$ 个计数。

---

## 8. 2D 激光雷达（LiDAR）与扫描匹配

### 8.1 它要解决什么问题

测量周围障碍物在**水平面上**的距离轮廓，用于避障、建图、基于地图的定位（AMCL 的观测）。

### 8.2 原理

常见 2D 激光雷达通过旋转测距头，按固定角度步进发射激光并测距。测距方式有：

- **飞行时间**（Time of Flight, ToF）：$d=c\,\Delta t/2$，$c$ 为光速。
- **三角测距**（triangulation）：光斑在传感器上的位置随距离变化，成本低，远距离精度下降。

### 8.3 数据格式：ROS `LaserScan`

一帧扫描包含：`angle_min`、`angle_max`、`angle_increment`、`range_min`、`range_max`、`ranges[i]`（有时还有 `intensities`），以及扫描耗时 `scan_time` 和相邻两束间时间 `time_increment`。第 $i$ 条光束的角度（相对 `laser` 坐标系 x 轴，逆时针为正）：

$$
\theta_i=\text{angle\_min}+i\cdot\text{angle\_increment}
$$

转成雷达坐标系下的点：

$$
x_i=r_i\cos\theta_i,\qquad y_i=r_i\sin\theta_i
$$

`ranges[i]` 可能是 `inf`（没打到东西或超量程）、`NaN`（无效），小于 `range_min` 或大于 `range_max` 的值应当丢弃。

**手算（示例）**：$r=2.0$ m，$\theta=30°$：$x=2\cos30°=1.732$ m，$y=2\sin30°=1.000$ m。

**角分辨率与点间距**：角分辨率 $1°=0.01745$ rad，远处点间距 $\approx r\,\Delta\theta$：$r=5$ m 时 $5\times0.01745=0.087$ m。所以**远处的细小障碍（桌腿、细杆）可能落在两束之间而漏检**。

### 8.4 噪声与误差

- 测距噪声：典型是 cm 量级的高斯噪声（具体数值看型号，【待你补全：所用雷达型号及其噪声标称】）。
- 反射率相关：黑色吸光、玻璃透射、镜面反射会造成漏检或假点。
- 运动畸变：一帧扫描耗时（例如 100 ms）内车在动，不同光束对应不同时刻的车身位姿；车速 1 m/s 时，一帧内车身移动 10 cm，点云被"拉扯"。严格处理要用 IMU / 里程计对每个点做时间插值去畸变。
- 外参：雷达相对 `base_link` 的安装位置与朝向，平移差 5 cm 就会让地图匹配出现系统偏差；yaw 差 1° 在 10 m 处造成 17 cm 横向误差。
- 时间戳：LaserScan 的 `header.stamp` 通常是**一帧开始**的时间，有的驱动填结束时间；与里程计对齐时要查清楚（见[时间同步章](./06_时间同步_坐标系一致性与数据有效性.md)）。

### 8.5 扫描匹配（scan matching）

**问题**：已知前一帧点云 $P=\{p_i\}$ 和当前帧点云 $Q=\{q_i\}$（对应点 $p_i\leftrightarrow q_i$），求把 $P$ 变换到 $Q$ 的刚体变换（旋转 $\theta$、平移 $t$），即车辆两帧之间的相对运动。

**最小二乘目标**：

$$
\min_{R,t}\ \sum_i \|q_i-(Rp_i+t)\|^2,\qquad R=\begin{bmatrix}\cos\theta&-\sin\theta\\ \sin\theta&\cos\theta\end{bmatrix}
$$

**推导**（二维闭式解）：

1. 对 $t$ 求导并令为零：$t=\bar q-R\bar p$，其中 $\bar p,\bar q$ 为质心。即先把两组点各自减去质心 $p_i'=p_i-\bar p,\ q_i'=q_i-\bar q$。
2. 代入后目标变为 $\sum\|q_i'-Rp_i'\|^2$，展开后只需最大化 $\sum q_i'^\top Rp_i'$：

$$
\sum_i q_i'^\top R p_i' = \cos\theta\sum_i(p_{ix}'q_{ix}'+p_{iy}'q_{iy}')+\sin\theta\sum_i(p_{ix}'q_{iy}'-p_{iy}'q_{ix}')
$$

3. 形如 $A\cos\theta+B\sin\theta$，最大值在 $\theta=\operatorname{atan2}(B,A)$：

$$
\theta=\operatorname{atan2}\!\Big(\sum(p_x'q_y'-p_y'q_x'),\ \sum(p_x'q_x'+p_y'q_y')\Big)
$$

**手算（示例）**：$P=\{(1,0),(-1,0)\}$，$Q=\{(2,2),(2,0)\}$（是 $P$ 逆时针转 90° 再平移 $(2,1)$ 得到的）。

- 质心：$\bar p=(0,0)$，$\bar q=(2,1)$。
- 去质心：$p'=\{(1,0),(-1,0)\}$，$q'=\{(0,1),(0,-1)\}$。
- $B=\sum(p_x'q_y'-p_y'q_x')=(1\cdot1-0\cdot0)+((-1)(-1)-0)=1+1=2$。
- $A=\sum(p_x'q_x'+p_y'q_y')=(1\cdot0+0\cdot1)+((-1)\cdot0+0\cdot(-1))=0$。
- $\theta=\operatorname{atan2}(2,0)=90°$。
- $t=\bar q-R\bar p=(2,1)-R(0,0)=(2,1)$。

恢复了真实变换。

**真实情形**的两个难点：

1. **对应关系未知**：ICP（Iterative Closest Point）每轮把每个点配给最近邻点，用上面的闭式解更新变换，再重新配对，迭代到收敛。需要一个不错的初值，否则会陷入局部最优。
2. **走廊退化**：长直走廊里点云沿走廊方向几乎没有约束，匹配沿走廊方向会"滑动"。这是 LiDAR 定位在长走廊失败的典型原因（见[粒子滤波与 AMCL](./05_粒子滤波与AMCL.md)）。

AMCL 并不做 ICP，它用的是把扫描与已知地图做似然比较（见第 5 章），但"点云—几何约束—退化"的直觉是相通的。

---

## 9. 在本项目里怎么用

按 CV 的描述，P5 是一台车式移动机器人，定位模块融合 **IMU、轮速编码器、2D LiDAR**，用 EKF/AMCL 做基于地图的定位，并处理由"时间不一致、坐标系不一致、运动方向"带来的漂移和位姿跳变（见 [P5 README](../../01_项目课程/P5_车式移动机器人自主导航/README.md)）。本章的对应关系：

| 传感器 | 在 P5 定位链中的角色（典型做法） | 对应后续章节 |
|---|---|---|
| 轮速编码器 | 车速、里程；与转向 / 差速模型一起得到 odom 位姿 | [里程计与传感器融合](./04_里程计与传感器融合.md) |
| IMU | 提供 yaw 速率与加速度，用于 EKF 的短时姿态 | [互补滤波与 Mahony](./02_互补滤波与Mahony_AHRS.md)、[卡尔曼滤波与 EKF](./03_卡尔曼滤波与EKF.md) |
| 2D LiDAR | 与已知地图比对做全局定位（AMCL） | [粒子滤波与 AMCL](./05_粒子滤波与AMCL.md) |

其他项目里同样的传感器知识：P1、P3、P7 用 IMU 做姿态与平衡 / 云台反馈，P2 用编码器闭环轮速并积分里程；P6 把多源感知与执行器反馈的时间戳、坐标系、有效性统一起来（见[时间同步章](./06_时间同步_坐标系一致性与数据有效性.md)与 [P6 README](../../01_项目课程/P6_深开鸿M-Robots平台软件/README.md)）。

> **【待你补全】** 面试官大概率追问 P5 的硬件：IMU 型号与采样频率、编码器线数与减速比、LiDAR 型号与扫描频率、雷达与 IMU 的安装位置。CV 未给出这些具体数值，请自行补全后再口述。

---

## 10. 常见错误与故障现象

| 现象 | 常见原因 | 排查方法 |
|---|---|---|
| 静止时 yaw 持续缓慢转动 | 陀螺零偏未标定或未在线估计 | 静置 10 s 看均值是否为 0 |
| 加速时 pitch / roll 估计晃动 | 融合时盲信加速度计 | 对比 $\|\mathbf f\|$ 与 $g$，加权重门限 |
| 倒车时里程计位姿往前走 | 单路测速没有方向；或速度符号丢失 | 看编码器驱动是否输出带符号速度 |
| 直线走成弧线 | 左右轮径或标定系数不同 | 走 5 m 直线测横向偏差 |
| 原地转一圈，里程计显示不是 360° | 轮距参数或打滑 | 标定轮距：转 10 圈取平均 |
| 地图匹配时车相对地图"斜着" | LiDAR 外参 yaw 错误 | 在 RViz 看激光点与墙是否整体旋转 |
| 激光点云在车转弯时糊 | 运动畸变、时间戳 | 检查扫描 stamp 是开始还是结束时刻 |
| 速度读数呈台阶状 | 低速 M 法量化 | 换 T 法 / M/T 法，或加滤波 |

---

## 11. 面试怎么问、怎么答

**Q1. What errors does an IMU have, and how do you handle them?**

> A: A gyroscope has a bias that drifts with temperature, white noise, and a scale-factor error. Integrating the bias makes the angle drift linearly, and the noise gives a random walk. An accelerometer measures specific force, so it mixes gravity with linear acceleration, and it has its own bias and noise. I calibrate the gyro bias while the robot is still, and I let a filter, like a complementary or Kalman filter, estimate the slow bias online and use the accelerometer only as a long-term reference for tilt.

中文要点：陀螺有零偏（随温度漂）、噪声、尺度因子；积分后零偏线性漂移，噪声随机游走。加速度计测比力，重力和运动加速度混在一起。静止标零偏，用互补 / 卡尔曼在线估计零偏，加速度计只作倾角的长期参考。

追问方向：为什么加速度计不能给 yaw？磁力计能解决吗？室内磁力计为什么不可靠？

**Q2. How do you measure wheel speed with an encoder? What are the trade-offs?**

> A: With a quadrature encoder I count edges. The M method counts pulses in a fixed window, so it is simple and good at high speed, but the quantisation error is large at low speed. The T method measures the time between pulses with a fast timer, which is accurate at low speed but noisy at high speed. The M/T method combines both. I usually add a small low-pass filter after that.

中文要点：M 法定窗口数脉冲，高速好低速差；T 法测脉冲间隔，低速好高速差；M/T 法折中；再加低通。

追问方向：给定 PPR、减速比、轮径，一个计数是多少米？没有方向信息的单路编码器怎么办？

**Q3. How does 2D LiDAR noise affect localisation?**

> A: The range noise itself is small, a few centimetres, so it is averaged out over many beams. The bigger problems are systematic: a wrong extrinsic calibration, motion distortion within one scan, and degenerate geometry such as a long corridor where the scan constrains only one direction. Those cause consistent bias or drift rather than random noise.

中文要点：测距随机噪声不大，多束平均；更严重的是系统性误差——外参错、运动畸变、走廊几何退化。

**Q4. What coordinate convention do you use?**

> A: REP-103: x forward, y left, z up, right-handed, radians and metres per second. I always convert sensor data into the base_link frame with the mounting extrinsics before fusing it.

中文要点：x 前 y 左 z 上，rad 与 SI 单位；融合前用安装外参统一到 base_link。

---

## 12. 本章要点速记

1. 传感器通用模型：$z=s\,x+b+n$；零偏、尺度、噪声、温漂各有不同的处理方式。
2. 陀螺积分：零偏造成角度**线性**漂移，白噪声造成 $\sqrt t$ 随机游走；短时间内零偏更危险。
3. 加速度计测**比力**，静止时 $\mathbf f=g\,\mathbf u_b$；$\phi=\operatorname{atan2}(f_y,f_z)$，$\theta=\operatorname{atan2}(-f_x,\sqrt{f_y^2+f_z^2})$；yaw 不可观。
4. 加速运动会污染倾角：$a=2\ \text{m/s}^2$ 造成约 11.5° 假俯仰。
5. 磁力计给 yaw 绝对参考，但怕硬铁 / 软铁和室内钢结构。
6. 编码器：4 倍频后每转计数 $4P\cdot i$；每计数距离 $2\pi r/N_{wheel}$；M 法低速差，T 法高速差，M/T 法折中。
7. 单路测速没有方向；倒车符号要从别处补。
8. LaserScan：$\theta_i=\text{angle\_min}+i\Delta\theta$，$(x,y)=(r\cos\theta,\ r\sin\theta)$；丢弃 inf / NaN / 超量程。
9. 扫描匹配闭式解：去质心后 $\theta=\operatorname{atan2}(\sum(p_xq_y-p_yq_x),\sum(p_xq_x+p_yq_y))$，$t=\bar q-R\bar p$。
10. 统一坐标系与单位（REP-103）是融合前的第一道关。

## 自测题

1. 陀螺零偏 0.01 rad/s，静止积分 30 s，角度误差多大？（答：$0.01\times30=0.3$ rad $\approx17.2°$。）
2. 水平静置时加速度计读数是多少？自由落体时呢？（答：静置读数 $(0,0,+g)$，因为测的是比力；自由落体读数约为 0。）
3. 车辆以 $1\ \text{m/s}^2$ 前向加速，仅用加速度计估计 pitch，会得到多大假角度？（答：$\arctan(1/9.81)=5.8°$，方向为负（按本章约定）。）
4. 编码器 500 PPR，四倍频，减速比 3:1，轮半径 0.06 m。一个计数对应多少米？（答：每轮转 $4\times500\times3=6000$ 计数；$2\pi\times0.06/6000=6.28\times10^{-5}$ m。）
5. $T_s=20$ ms 的 M 法，速度分辨率（一个计数）是多少？（答：$d_{count}/T_s$，接上题为 $6.28\times10^{-5}/0.02=3.14\times10^{-3}$ m/s。）
6. LiDAR 角分辨率 0.5°，目标距离 8 m，相邻光束点间距约多少？（答：$8\times0.5\times\pi/180=0.0698$ m $\approx7$ cm。）
7. 为什么长走廊里激光匹配会退化？（要点：点云只约束垂直于墙的方向，沿走廊方向的平移没有几何特征约束，匹配沿该方向不可观。）

## 相关链接

- 下一章：[互补滤波与 Mahony AHRS](./02_互补滤波与Mahony_AHRS.md)
- 全章节：[卡尔曼滤波与 EKF](./03_卡尔曼滤波与EKF.md)、[里程计与传感器融合](./04_里程计与传感器融合.md)、[粒子滤波与 AMCL](./05_粒子滤波与AMCL.md)、[时间同步、坐标系一致性与数据有效性](./06_时间同步_坐标系一致性与数据有效性.md)
- 项目：[P5 车式移动机器人自主导航](../../01_项目课程/P5_车式移动机器人自主导航/README.md)、[P6 深开鸿 M-Robots 平台软件](../../01_项目课程/P6_深开鸿M-Robots平台软件/README.md)、[P3 RoboMaster 英雄机器人](../../01_项目课程/P3_RoboMaster英雄机器人嵌入式控制/README.md)
- 八股：[状态估计与 SLAM 八股](../../03_八股知识/03_机器人学/03_状态估计与SLAM八股.md)
