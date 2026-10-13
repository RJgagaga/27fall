# 08 差速底盘运动学、轮速 PID 与里程计

> **一句话定位**：P2 的第三条线。给定"车体要多快前进、多快转弯"，差速运动学告诉你**左右两个轮子各要转多快**；轮速 PID 用**编码器反馈**让轮子真的转到这个速度；里程计把"测到的轮子转动"**积分**成"车走到了哪里、朝哪"。CV 原句：*mapped body velocity commands to wheel targets via differential-drive kinematics, closed the PID loop using encoder feedback and integrated wheel motion into odometry for teleoperated and autonomous modes.* 这一章是它的数学和工程细节；下一章 [09](./09_STM32H7_FDCAN底盘固件解读.md) 再对着真实固件逐行读。

**本章学完你能：**

- 推导差速模型 $v_r=v+\omega L/2,\ v_l=v-\omega L/2$ 及反算，并用瞬时转动中心（ICR）解释；
- 把车体速度换算成轮角速度（rad/s）和 rpm，处理左右电机镜像带来的符号；
- 解释为什么轮速饱和时要**等比缩放**两个轮子，而不是各自截断；
- 把编码器计数换算成轮转角和轮速，并估计分辨率与量化噪声；
- 写出离散 PID，手算 P 与 PI 在一阶对象上的阶跃响应，解释稳态误差、积分饱和、抗饱和；
- 推导里程计：$s=r\theta$、$\Delta s=(\Delta s_r+\Delta s_l)/2$、$\Delta\psi=(\Delta s_r-\Delta s_l)/L$，并比较欧拉、中点、精确圆弧三种积分；
- 估算轮径/轮距误差、左右轮差异、打滑如何变成位置漂移，并设计标定实验；
- 区分遥控与自动两种模式在"速度来源"与"保护"上的不同。

**前置知识：**

- 差速/麦轮/全向轮运动学（基础版）：[04_差速_麦克纳姆_全向轮与车辆运动学](../../02_基础知识/02_力学与动力学建模/04_差速_麦克纳姆_全向轮与车辆运动学.md)
- PID 与限幅：[02_PID控制全解](../../02_基础知识/05_控制理论/02_PID控制全解.md)、[07_前馈_增益调度_限幅与饱和](../../02_基础知识/05_控制理论/07_前馈_增益调度_限幅与饱和.md)
- 里程计：[04_里程计与传感器融合](../../02_基础知识/06_状态估计/04_里程计与传感器融合.md)、[01_传感器_IMU_编码器_激光雷达](../../02_基础知识/06_状态估计/01_传感器_IMU_编码器_激光雷达.md)
- 与其他项目的联系：P3 的底盘功率与轮速 PID（[P3 README](../P3_RoboMaster英雄机器人嵌入式控制/README.md)）、P5 的轮式里程计与 EKF（[P5 README](../P5_车式移动机器人自主导航/README.md)）

## 目录

1. 要解决的问题与坐标约定
2. 差速运动学的推导
3. 手算：车体速度 ↔ 轮速 ↔ rpm
4. 轮速饱和：为什么要等比缩放
5. 编码器：从计数到轮速
6. 轮速闭环 PID
7. 遥控与自动两种模式
8. 里程计：从 $s=r\theta$ 到位姿积分
9. 里程计误差：它为什么会漂，漂多少
10. 标定实验设计
11. 常见错误与排查
12. 面试怎么问、怎么答
13. 本章要点速记 / 自测题 / 相关链接

---

## 1. 要解决的问题与坐标约定

上层（遥控器或上位机）只会说"以 0.5 m/s 前进并同时以 1 rad/s 左转"，而底盘只有两个独立驱动的轮子（加上若干万向/脚轮支撑）。要回答：

1. 左轮、右轮各自应该转多快？（**运动学逆解**）
2. 怎么让轮子真的转到这个速度？（**速度闭环**）
3. 车现在大概在哪、朝哪？（**里程计 = 运动学正解的积分**）

**坐标约定**（ROS 惯例 REP-103：x 前、y 左、z 上，偏航角 yaw 绕 z 逆时针为正）：

| 符号 | 含义 | 单位 |
|---|---|---|
| $v$ | 车体前向线速度（沿 $+x$） | m/s |
| $\omega$ | 车体偏航角速度，逆时针为正 | rad/s |
| $L$ | 轮距（两驱动轮接地点间距） | m |
| $r$ | 驱动轮半径 | m |
| $v_l,v_r$ | 左/右轮轮缘前向线速度 | m/s |
| $\dot\varphi_l,\dot\varphi_r$ | 左/右轮角速度（向前为正） | rad/s |
| $(x,y,\psi)$ | 里程计位姿（世界/里程计系） | m, m, rad |

> **注意**：不同实现对"前进方向"轴的命名不同。后面读 STM32 固件时你会看到它把"前进"叫 `vy`、"转弯"叫 `vw`（`mini_car_calc(vy, vw)`），与 ROS 惯例的 $x$ 前不同。**读代码时一定要按它自己的约定走，别凭变量名猜。**

差速车有一个**非完整约束**（non-holonomic）：驱动轮不能侧向滑动，所以车体侧向速度 $v_y\equiv0$。因此差速底盘只有两个自由度的速度输入 $(v,\omega)$，哪怕上层给出 $v_x,v_y,\omega$ 三个量，侧向分量也只能被忽略。

---

## 2. 差速运动学的推导

### 2.1 刚体平面运动

车体上任一点的速度 = 参考点速度 + $\omega\times$ 相对位置。取两轮中点为车体参考点，车体速度 $(v,0)$ 沿 $+x$，角速度 $\omega$。左轮接地点在 $(0,\ +L/2)$，右轮在 $(0,\ -L/2)$（$y$ 向左为正）。

$\omega\hat z\times(0,\pm L/2)=\omega(\mp L/2,\ 0)$，所以接地点的前向速度：

$$
v_l=v-\frac{\omega L}{2},\qquad
v_r=v+\frac{\omega L}{2}.
$$

（逆时针转动即左转：外侧的右轮更快，所以 $v_r>v_l$ ✓。）

### 2.2 反解（里程计用）

$$
v=\frac{v_r+v_l}{2},\qquad \omega=\frac{v_r-v_l}{L}.
$$

### 2.3 瞬时转动中心（ICR）解释

若 $\omega\ne0$，车体绕 ICR 作圆周运动，转弯半径（中点到 ICR 的距离）

$$
R=\frac{v}{\omega}=\frac{L}{2}\cdot\frac{v_r+v_l}{v_r-v_l}.
$$

- $v_r=v_l$：$R\to\infty$，直行；
- $v_r=-v_l$：$R=0$，**原地转**；
- $v_l=0$：$R=L/2$，绕左轮转；
- $v_r=2v_l$：$R=\frac L2\cdot\frac{3v_l}{v_l}=\frac{3L}{2}$。

### 2.4 轮角速度

轮缘线速度 = 半径 × 角速度：

$$
\dot\varphi_{l,r}=\frac{v_{l,r}}{r}.
$$

---

## 3. 手算：车体速度 ↔ 轮速 ↔ rpm

示例：$v=0.5$ m/s，$\omega=1$ rad/s（左转），$L=0.4$ m，$r=0.1$ m。

**正解：**

$$
v_r=0.5+1\times\tfrac{0.4}{2}=0.7\ \text{m/s},\qquad
v_l=0.5-1\times\tfrac{0.4}{2}=0.3\ \text{m/s}.
$$

**轮角速度：** $\dot\varphi_r=0.7/0.1=7$ rad/s，$\dot\varphi_l=0.3/0.1=3$ rad/s。

**换成 rpm**（$1\ \text{rad/s}=60/2\pi=9.549$ rpm）：右轮 $7\times9.549=66.8$ rpm，左轮 $28.6$ rpm。

**回算验证：** $v=(0.7+0.3)/2=0.5$ ✓；$\omega=(0.7-0.3)/0.4=1$ ✓；$R=v/\omega=0.5$ m。

**一段时间后的位姿变化：** 若初始 $\psi=\pi/2$（朝 $+y$），$dt=0.1$ s，近似 $\Delta x=v\cos\psi\,dt=0$，$\Delta y=v\sin\psi\,dt=0.05$ m，$\Delta\psi=\omega\,dt=0.1$ rad ✓。

### 常见的符号问题（真实固件里一定会遇到）

两个轮电机往往**镜像安装**：车向前走时，左右电机的轴向旋转方向相反。所以同样的"向前"，驱动器那里一个要给正速度、一个要给负速度；编码器反馈也是一正一负。必须在**接口层**统一成"向前为正"，否则整个运动学会错。STM32 固件的做法见 09 章（命令和反馈各取一次负号）。

---

## 4. 轮速饱和：为什么要等比缩放

轮子/驱动器有最大转速 $v_{max}$。当 $(v,\omega)$ 要求某个轮子超速时，常见两种处理：

**(a) 各轮独立截断：** $v_r\leftarrow\min(v_r,v_{max})$，$v_l\leftarrow\min(v_l,v_{max})$。
**(b) 等比缩放：** $k=\min\big(1,\ v_{max}/\max(|v_l|,|v_r|)\big)$，$v_l\leftarrow kv_l,\ v_r\leftarrow kv_r$。

示例：$v=1.0$，$\omega=2$，$L=0.4$：$v_r=1.4$，$v_l=0.6$，$v_{max}=1.0$。

| 方法 | $v_r$ | $v_l$ | 实际 $v=\frac{v_r+v_l}2$ | 实际 $\omega=\frac{v_r-v_l}L$ | 半径 $R=v/\omega$ |
|---|---|---|---|---|---|
| 期望 | 1.4 | 0.6 | 1.0 | 2.0 | 0.50 m |
| (a) 独立截断 | 1.0 | 0.6 | 0.8 | 1.0 | **0.80 m** |
| (b) 等比缩放（$k=0.714$） | 1.0 | 0.429 | 0.714 | 1.43 | **0.50 m** |

(a) 改变了转弯半径（0.5→0.8 m），车会"转不进弯"，撞到原本能躲过的障碍；(b) **保持曲率不变**，只是整体慢一点。这是 *saturation handling* 的一个典型面试考点（同样的道理你在 [05 章](./05_约束与限位.md) 的"事后截断 vs QP"里见过）。

---

## 5. 编码器：从计数到轮速

### 5.1 换算

设编码器每转 $C$ 个计数（已含四倍频），电机到轮的减速比 $N$（电机转 $N$ 圈，轮转 1 圈；直驱轮毂电机 $N=1$）。采样间隔 $dt$ 内计数增量 $\Delta c$：

$$
\Delta\varphi_{wheel}=\frac{2\pi\,\Delta c}{C\,N},\qquad
\dot\varphi_{wheel}\approx\frac{\Delta\varphi_{wheel}}{dt},\qquad
\Delta s=r\,\Delta\varphi_{wheel}.
$$

若驱动器直接给出转速（rpm 或 0.1 rpm 单位），则先换成 rad/s：$\dot\varphi=\text{rpm}\times2\pi/60$，并明确**这是电机轴转速还是轮轴转速**（差一个 $N$）。

### 5.2 分辨率与量化噪声（示例）

$C=4096$，$N=10$，$r=0.1$ m，$dt=5$ ms：最小可分辨角 $2\pi/(4096\times10)=1.53\times10^{-4}$ rad，对应最小速度步长

$$
\Delta v_{min}=\frac{r\cdot1.53\times10^{-4}}{0.005}=3.1\times10^{-3}\ \text{m/s}.
$$

也就是"数脉冲"法（M 法）的速度分辨率 3 mm/s。低速时（比如目标 0.05 m/s，每 5 ms 只走约 16 个计数），一个计数的抖动就是 6% 的速度误差——所以低速时要么**加长采样窗口**，要么用"测脉冲间隔"法（T 法），要么对速度做低通滤波。

### 5.3 位置计数的回绕

如果用 16 位计数器：数值到 65535 后回零。取增量时要用**有符号差**：`(int16_t)(c_now - c_last)`，才能正确处理回绕。固件中常见 `ENCODER_MAX 65535` 之类的宏就是为此。

### 5.4 速度反馈 vs 位置差分

- **直接读速度**（驱动器内部已算好）：简单，但滤波/延迟未知；
- **位置差分**（用累计位置增量）：不依赖驱动器的速度估计，更适合做里程计（避免采样相位误差，见 8.5 节）。

---

## 6. 轮速闭环 PID

### 6.1 为什么要闭环

仅靠开环（把目标速度直接换成电机命令），实际转速会因为负载、地面摩擦、电池电压、坡度而偏离。闭环用编码器测得的实际速度与目标比较，修正输出。

### 6.2 离散 PID（位置式）

误差 $e_k=\dot\varphi^{*}_k-\hat{\dot\varphi}_k$，输出

$$
u_k=K_pe_k+K_i\sum_{j\le k}e_j\,dt+K_d\frac{e_k-e_{k-1}}{dt}.
$$

STM32 固件里的实现把 $dt$ 吸收进增益（`pout = p*err`，`iout += i*err`，`dout = d*(err - err_last)`），所以固件里的 $K_i,K_d$ 是"每个采样周期"的增益；换采样周期要重新整定。

实际输出 $u$ 的物理意义取决于接口：可能是电机电流/力矩指令，也可能是下一级速度指令（如果驱动器自己有电流环，$u$ 就是速度修正量）。

### 6.3 手算：P 与 PI 在一阶对象上的阶跃响应（示例，非项目实测）

对象（电机 + 车轮 + 内部电流环等效成一阶滞后）：$\omega_{k+1}=0.8\,\omega_k+0.2\,u_k$，目标 $\omega^*=10$ rad/s。

**只有 P（$K_p=1$）：**

| $k$ | $\omega_k$ | $e_k$ | $u_k=K_pe_k$ | $\omega_{k+1}$ |
|---|---|---|---|---|
| 0 | 0 | 10 | 10 | 2.000 |
| 1 | 2.000 | 8.000 | 8.000 | 3.200 |
| 2 | 3.200 | 6.800 | 6.800 | 3.920 |
| 3 | 3.920 | 6.080 | 6.080 | 4.352 |
| … | → | | | → 5.0 |

稳态：$\omega=0.8\omega+0.2K_p(10-\omega)\Rightarrow\omega_{ss}=\dfrac{K_p}{1+K_p}\times10=5$。**P 控制有稳态误差**（这里 50%）；$K_p$ 越大误差越小但永远不为零。

**PI（$K_p=1,\ K_i=0.25$，每样本累加）：**

| $k$ | $\omega_k$ | $e_k$ | $I_k=\sum e$ | $u_k=e_k+0.25I_k$ | $\omega_{k+1}$ |
|---|---|---|---|---|---|
| 0 | 0 | 10 | 10 | 12.500 | 2.500 |
| 1 | 2.500 | 7.500 | 17.5 | 11.875 | 4.375 |
| 2 | 4.375 | 5.625 | 23.125 | 11.406 | 5.781 |
| 3 | 5.781 | 4.219 | 27.344 | 11.055 | 6.836 |
| … | → | | | | 12 步后约 9.7，趋向 10 |

积分项不断累积，直到 $e\to0$ 时 $u$ 稳定在 $10$（稳态输出 $=\omega^*/\text{DC增益}=10/1$）。**I 项消除稳态误差**。

### 6.4 积分饱和（windup）与抗饱和

当输出被限幅（电机饱和、驱动器限幅）时，误差仍在，积分项还在累积，等误差反号时需要很久才能"退出来"，造成大超调——积分饱和。常用对策：

1. **积分限幅**（固件里的 `IntegralLimit`）：$|I|\le I_{max}$；
2. **条件积分（clamping）**：输出已饱和且误差同号时停止积分；
3. **反算（back-calculation）**：把限幅差回灌到积分项；
4. **模式切换时清积分**：从保护/停止模式切回运行时清零（固件里 PROTECT 模式就做了 `iout = 0`）。

### 6.5 常用改进（STM32 固件 `pid.c` 里都有，可作面试素材）

| 改进 | 做什么 | 解决什么 |
|---|---|---|
| 死区（deadband） | $|e|<$ 阈值时输出 0 | 抑制抖动、消除小误差下的无意义调整 |
| 积分限幅 + 输出限幅 | 限制 $I$ 与 $u$ | 防积分饱和、保护驱动器 |
| 梯形积分 | 用 $(e_k+e_{k-1})/2$ 累加 | 提高积分精度 |
| 变速积分 | 误差大时少积分，误差小时多积分 | 减小超调 |
| 微分先行 | 对测量值求微分而不是误差 | 避免目标突变引起的微分冲击 |
| 微分滤波 | 低通滤波 D 项 | 抑制编码器噪声放大 |

固件中 `pid_chassis_spd` 的初始化参数（源码）：`PID_struct_init(&pid_chassis_spd[i], 200, 80, 5.0f, 0.00f, 0.0f, ...)`，即输出上限 200、积分限幅 80、$K_p=5.0$、$K_i=K_d=0$。**这里 $K_i=0$**，所以如果这个 PID 被调用，它只是比例控制。**本项目取值：** 我们实际用的是**板端**这一层：STM32 每 5 ms（200 Hz）读编码器 rpm，闭环算出速度命令发给驱动器；驱动器自身的电流环不由我们调。调试后取 $K_p=5.0$、$K_i=0.5$、$K_d=0$，输出上限 200、积分限幅 80——源码初值的 $K_i=0$ 会留下稳态误差，加一点积分后阶跃稳态误差降到几 rpm 以内。

### 6.6 其他实用技巧

- **速度前馈**：把目标速度乘一个系数直接加到输出，PID 只负责补偿误差；
- **加速度限制（斜坡）**：目标速度的变化率限幅，如 $a_{max}=1$ m/s²，$dt=5$ ms 时每步最多变 0.005 m/s，避免起步打滑、电流冲击；
- **摩擦死区补偿**：小速度命令克服不了静摩擦，可加一个小的最小起动量；
- **整定顺序**：先只用 P 到略有振荡再回退，再加 I 消除稳态误差，最后视噪声情况小量加 D；记录阶跃响应的上升时间、超调、稳态误差。

---

## 7. 遥控与自动两种模式

CV：*...for teleoperated and autonomous modes.* 两种模式的**差别只在 $(v,\omega)$ 从哪来**，下游的"运动学 → 轮速 PID → 里程计"完全共用：

| | 遥控模式 | 自动模式 |
|---|---|---|
| 速度来源 | 遥控器摇杆（SBUS 串口协议，通道值 → 速度） | 上位机（ROS 2 节点）经 CAN 发来的速度指令帧 |
| 输入处理 | 死区、线性缩放、限幅 | 范围限幅、单位换算 |
| 失联保护 | 遥控信号丢失 → 零速/保护模式 | 上位机心跳丢失 → 零速 |
| 切换 | 遥控器上的开关选择 保护 / 遥控 / 自动 | 同上 |

一个好的切换设计要满足：

1. **任何模式切换都先清零速度命令**，并清除 PID 积分；
2. **保护模式优先级最高**（急停）；
3. 自动模式下上位机需要有**心跳或新鲜度检测**，不能默默沿用最后一条速度。

STM32 固件里这些逻辑如何实现（以及其中的一个心跳 bug）见 [09 章](./09_STM32H7_FDCAN底盘固件解读.md)。

---

## 8. 里程计：从 $s=r\theta$ 到位姿积分

### 8.1 为什么 $s=r\theta$

半径为 $r$ 的圆，转过角度 $\theta$（**弧度**）对应的弧长是 $s=r\theta$（弧度定义：弧长/半径）。**纯滚动**（无滑动）时，轮缘每转过的弧长，等于轮心向前推进的距离。所以

$$
\Delta s_{r,l}=r\,\Delta\varphi_{r,l}.
$$

必须用弧度，用角度要乘 $\pi/180$。验证：转 $10$ rad、$r=0.1$ m，走 1 m。

### 8.2 一个周期的车体位移

由差速运动学的正解（把速度换成同周期的位移）：

$$
\Delta s=\frac{\Delta s_r+\Delta s_l}{2},\qquad
\Delta\psi=\frac{\Delta s_r-\Delta s_l}{L}.
$$

### 8.3 积分成位姿

当前航向 $\psi$，三种近似：

| 方法 | 公式 | 误差特性 |
|---|---|---|
| 欧拉（旧航向） | $x\mathrel{+}=\Delta s\cos\psi,\ y\mathrel{+}=\Delta s\sin\psi,\ \psi\mathrel{+}=\Delta\psi$ | 步长大、转弯急时有偏差 |
| 中点 | $x\mathrel{+}=\Delta s\cos(\psi+\tfrac{\Delta\psi}{2}),\ y\mathrel{+}=\Delta s\sin(\psi+\tfrac{\Delta\psi}{2})$ | 二阶精度，最常用 |
| 精确圆弧 | $R=\Delta s/\Delta\psi$；局部位移 $(R\sin\Delta\psi,\ R(1-\cos\Delta\psi))$，再旋转到世界系 | 在恒定曲率下精确；$\Delta\psi\to0$ 时退化为直线 |

### 8.4 手算（示例）

$\Delta s_r=0.07$ m，$\Delta s_l=0.03$ m，$L=0.4$ m，起始 $x=y=\psi=0$：

$$
\Delta s=\frac{0.07+0.03}{2}=0.05\ \text{m},\qquad
\Delta\psi=\frac{0.07-0.03}{0.4}=0.1\ \text{rad}.
$$

- 欧拉：$x=0.05\cos0=0.05$，$y=0.05\sin0=0$；
- 中点：$x=0.05\cos0.05=0.049938$，$y=0.05\sin0.05=0.002499$；
- 精确圆弧：$R=0.05/0.1=0.5$ m，$x=0.5\sin0.1=0.049917$，$y=0.5(1-\cos0.1)=0.002498$。

结果：中点与精确值差 $2\times10^{-5}$ m；欧拉的 $y$ 误差是 2.5 mm（5%）。**欧拉法在单步里也看起来"差不多"，但误差会逐步累积为系统偏差**，所以推荐中点法。（在板端以 5 ms 为周期时，每步 $\Delta\psi$ 很小，欧拉与中点差距不大——但转弯快时仍值得用中点。）

### 8.5 速度积分 vs 位置差分

- **速度积分**：$\Delta s=v_{meas}\cdot dt$。若 $dt$ 取名义值（比如固定 5 ms）而实际采样间隔有抖动，距离就有误差。**应当使用实际时间戳差**。
- **位置差分**：$\Delta s=r\,\Delta\varphi$，其中 $\Delta\varphi$ 来自累计位置计数差，**不依赖采样时间**。更稳。

固件里的里程计是"速度 × 固定系数"的速度积分形式（09 章逐行讲），这是一个值得在面试时主动指出的"已知限制与改进点"。

### 8.6 发布给上位机

里程计位姿 $(x,y,\psi)$ 会通过 CAN 回传，上位机转成 ROS 的 `nav_msgs/Odometry`（`header.frame_id = odom`，`child_frame_id = base_link`），并在 TF 树里发布 `odom → base_link`。**要点**：TF 里 `odom → base_link` 应该**只有一个发布者**；时间戳要用接收时间或带同步的时间；协方差需要给出（见 [P5](../P5_车式移动机器人自主导航/README.md) 的 EKF 融合）。

---

## 9. 里程计误差：它为什么会漂，漂多少

里程计是**开环积分**，误差只会累积，不会收敛。误差来源与量级（下面为推导，示例数值）：

### 9.1 轮径误差（两轮同比例）

两轮半径都偏大 $\epsilon$：距离按 $(1+\epsilon)$ 放大，航向不受影响。例：$\epsilon=1\%$，走 10 m 会多算 0.1 m。

### 9.2 左右轮径差（最危险）

右轮半径比左轮大 $\epsilon$：两轮转同样角度 $\theta$ 时，$s_r=r(1+\epsilon)\theta$，$s_l=r\theta$，

$$
\Delta\psi=\frac{s_r-s_l}{L}=\frac{\epsilon\,r\theta}{L}=\frac{\epsilon\,s}{L},
$$

即每前进 $s$ 米，估计航向多偏 $\epsilon s/L$ 弧度。例：$\epsilon=1\%$，$L=0.4$ m：每米偏 $0.025$ rad = 1.43°，走 10 m 偏 14.3°，侧向位置误差约 $\int\psi\,ds=\dfrac{\epsilon}{L}\dfrac{s^2}{2}=\dfrac{0.01}{0.4}\cdot\dfrac{100}{2}=1.25$ m。**1% 的左右差异，10 m 后位置错 1.25 m。**

### 9.3 轮距误差

实际轮距 $L_t$，估计用 $L_e$：$\Delta\psi_e=\Delta\psi_t\cdot L_t/L_e$。$L_e$ 偏大 1%，原地转 360° 会少算约 3.6°。

### 9.4 打滑与地面

轮子空转/侧滑使 $\Delta s$ 偏大；在不平地面、地毯、急加减速时尤其明显。里程计无法发现打滑，只有外部传感器（IMU、LiDAR、视觉）能。

### 9.5 时间与量化

- 采样间隔抖动（速度积分时）；
- 编码器量化；
- 速度反馈低通造成相位延迟；
- 数值类型溢出（例如把位置压成 int16 毫米，会在 ±32.767 m 溢出，见 09 章）。

### 9.6 改善手段（概念）

标定（下一节）、用位置差分、与 IMU 陀螺融合航向（EKF/互补滤波）、与 LiDAR 定位/AMCL 融合做全局校正（P5 的内容）。

---

## 10. 标定实验设计

**目标**：估计 $r_l,r_r$（或平均 $r$ 和差异）与 $L$。

1. **直线实验**：车沿卷尺直行 5 m（多次、往返），记录里程计距离 $s_{odom}$ 与真实 $s_{true}$；$r_{new}=r_{old}\cdot s_{true}/s_{odom}$。若起终点横向有偏差，说明左右轮径差：$\epsilon\approx\dfrac{2L\,y_{err}}{s^2}$（由 9.2 节 $y=\epsilon s^2/(2L)$ 反推）；
2. **原地旋转实验**：原地转 $k$ 圈（用地面标记或陀螺仪累积角），记录里程计角度 $\psi_{odom}$ 与真实 $2\pi k$；$L_{new}=L_{old}\cdot\psi_{odom}/(2\pi k)$；
3. **方形实验**：沿 1 m × 1 m 方形顺/逆时针各走若干圈，比较终点与起点的偏差，用来检验综合误差（顺/逆分别暴露不同种类的系统误差）；
4. 记录环境：地面材质、速度、加速度，以便分析打滑。

> **本项目取值：** 轮半径 $r=0.13$ m（轮径 0.26 m）、轮距 0.75 m（固件里的 $L=0.375$ 是半轮距，见 09 章）。标定：直线推 5 m 标定轮半径、原地转 10 圈标定轮距，各重复 3 次取均值。精度：5 m 直线往返距离误差约 2%，原地转 10 圈航向累积误差约 30°（每圈约 3°）；超过十几米后需要外部定位（激光/视觉）修正。

---

## 11. 常见错误与排查

| 现象 | 可能原因 | 定位 |
|---|---|---|
| 直行时一直往一侧跑 | 左右符号/比例不一致；左右轮径差；PID 增益不同 | 比较左右轮实际转速；把目标设为相等看反馈是否相等 |
| 前进变后退 / 原地转变成直行 | 把线/角速度混淆；左右符号反 | 单轮单独测试 |
| 里程计转角偏大/偏小 | 轮距估计不准；角度公式缺少/多出系数 | 原地旋转标定 |
| 里程计距离偏大 | 轮径估计偏大；打滑 | 直线标定 |
| 速度抖动 | 编码器量化 + D 项放大噪声；采样太短 | 滤波；调 D；加长窗口 |
| 起步打滑/冲击 | 目标阶跃太大 | 加速度限制 |
| 切回运行时突然猛冲 | 积分饱和未清零；旧目标 | 模式切换清积分与目标 |
| 位置突然变成负的很大数 | 数值回绕/溢出 | 检查类型和量程 |

---

## 12. 面试怎么问、怎么答

**Q1. How do you convert a body velocity command into wheel speeds for a differential drive?**

> The right and left wheel linear speeds are v plus or minus omega times L over two, where L is the track width. Dividing by the wheel radius gives the wheel angular speed. In reverse, the body speed is the average of the two wheel speeds and the yaw rate is their difference divided by L. The left and right motors are mounted mirrored, so I fix the sign at the interface layer so that forward is positive on both sides.

中文要点：公式、除半径、反解、镜像符号。

*追问*：What if the command exceeds the wheel limit? → 等比缩放两轮，保持转弯半径；独立截断会改变曲率（0.5 m 变成 0.8 m 的例子）。

**Q2. How did you close the loop on the wheel speed?**

> I measure the wheel speed from the encoder, compare it to the target, and run a PID on each wheel. Proportional alone leaves a steady-state error, so the integral term removes it, with an integral limit and output limit to avoid windup. On mode switches I clear the integrator and the command. In our case it ran on the STM32 board at 200 hertz, with Kp 5.0, Ki 0.5 and no derivative term, an integral limit of 80 and an output limit of 200.

**Q3. Explain how you compute odometry and what its error looks like.**

> From the encoder I get each wheel's arc length, s equals r times theta. The forward distance is the average of the two arcs and the heading change is their difference over L. I integrate with a midpoint rule: advance along the heading plus half the heading change. It is dead reckoning, so errors accumulate. For example a one percent radius mismatch between the wheels gives about 1.4 degrees of heading error per metre, which becomes more than a metre of lateral error after ten metres. I calibrate with straight-line and in-place rotation tests.

中文要点：弧长、平均与差、中点积分；误差例子；标定。

**Q4. Why prefer position differences over velocity integration?**

> Velocity integration needs the exact sampling interval; if the real interval jitters, the distance is wrong. The position difference from the encoder counts is independent of the timing, so it is more robust. If I only have velocity, I multiply by the measured elapsed time rather than a nominal constant.

**Q5. How do teleoperated and autonomous modes differ in your base?**

> Only the source of the velocity command differs: joystick mapping with a dead zone in teleoperation, a CAN command from the ROS 2 side in autonomous mode. Both feed the same kinematics, wheel PID and odometry. A protect mode forces zero speed and clears the integrators, and the autonomous mode needs a heartbeat so a stale command is not kept.

---

## 13. 本章要点速记

1. 差速：$v_{r,l}=v\pm\omega L/2$；$v=\frac{v_r+v_l}2$，$\omega=\frac{v_r-v_l}L$；$R=v/\omega$。
2. 非完整约束：无侧向速度，$v_x,v_y,\omega$ 里侧向分量被忽略。
3. 轮角速度 $=v_{wheel}/r$；1 rad/s = 9.549 rpm；镜像电机在接口层统一符号。
4. 饱和用**等比缩放**保持曲率；独立截断会改变转弯半径。
5. 编码器：$\Delta\varphi=2\pi\Delta c/(CN)$；低速时量化噪声大，T 法/加长窗口/滤波。
6. 离散 PID；P 有稳态误差（一阶例：$K_p/(1+K_p)$），I 消除；积分限幅/条件积分防饱和；模式切换清积分。
7. 遥控/自动只有速度来源不同；保护模式优先；自动模式要心跳。
8. 里程计：$s=r\theta$（弧度！）、$\Delta s=\frac{\Delta s_r+\Delta s_l}2$、$\Delta\psi=\frac{\Delta s_r-\Delta s_l}L$；中点积分。
9. 误差：左右轮径差最危险（1% → 10 m 后侧向 1.25 m）；位置差分优于速度积分。
10. 标定：直线 + 原地旋转 + 方形。

## 自测题

1. $v=0.5,\ \omega=1,\ L=0.4,\ r=0.1$ 时，左右轮速度与角速度？（0.3/0.7 m/s；3/7 rad/s。）
2. $v_r=0.07,\ v_l=0.03$（每周期位移，m），$L=0.4$ 时的 $\Delta s$ 与 $\Delta\psi$？（0.05 m；0.1 rad。）
3. 为什么 P 控制有稳态误差？举数字。（一阶对象：$K_p=1$ 时稳态只到目标的一半。）
4. 速度超限，为什么要等比缩放？（保持曲率；独立截断改变转弯半径。）
5. 1% 的左右轮径差，走 10 m 的侧向误差多大？（约 1.25 m。）
6. 里程计为什么不能发现打滑？怎么补救？（只看轮子转动；融合 IMU/LiDAR。）

## 相关链接

- 上一章：[07 主从跟踪与 MIT 电机接口](./07_主从跟踪与MIT电机接口.md)
- 下一章：[09 STM32H7/FDCAN 底盘固件解读](./09_STM32H7_FDCAN底盘固件解读.md)
- 工程化：[20 工程化实践：频率周期与问题解决](./20_工程化实践_频率周期与问题解决.md)（5 ms 节拍、优化 4）
- 基础：[差速_麦克纳姆_全向轮与车辆运动学](../../02_基础知识/02_力学与动力学建模/04_差速_麦克纳姆_全向轮与车辆运动学.md)、[PID控制全解](../../02_基础知识/05_控制理论/02_PID控制全解.md)、[里程计与传感器融合](../../02_基础知识/06_状态估计/04_里程计与传感器融合.md)
- 八股：[控制八股](../../03_八股知识/03_机器人学/02_控制八股.md)、[状态估计与SLAM八股](../../03_八股知识/03_机器人学/03_状态估计与SLAM八股.md)
- 学习笔记：[逐字稿第二份：差速/里程计/弧长章节](../../05_学习笔记归档/P2/P2_本对话问答逐字稿_第二份.md)
