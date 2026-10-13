# 06 云台串级 PID 与视觉跟踪

> **一句话定位**：这是 CV 里 P3 第一条 bullet 的完整讲解——*pitch/yaw angle–rate cascaded PID using IMU/AHRS and motor feedback, converting remote-control and vision targets into continuous attitude commands for stable aiming and tracking*。本章从被控对象建模讲起，推导串级结构为什么有效，给出手算、整定流程、前馈与补偿，最后讲视觉目标的平滑、延迟补偿与模式切换。

## 本章学完你能……

- 建立云台单轴模型 $J\dot\omega=K_t i-b\omega-\tau_g-\tau_d$，并解释每一项；
- 推导"速度内环 PI + 角度外环 P"的闭环传递函数，说清为什么内环要比外环快 4～10 倍，并用 $J,K_t$ 反推增益；
- 手算一次完整的串级计算，包括限幅和积分抗饱和；
- 给出系统化整定流程（先符号、后内环、再外环、再前馈、最后扰动测试），并解释 Ziegler–Nichols 数值；
- 解释重力补偿、摩擦补偿、底盘旋转耦合（小陀螺）的前馈，并手算；
- 处理 yaw 角度绕回、目标角度的 sin/cos 滤波、目标的限速与平滑；
- 解释视觉延迟为什么必须用"带时间戳的绝对姿态"补偿，并手算误差；
- 设计遥控/视觉模式切换的无扰动策略；
- 回答"为什么串级""内外环带宽怎么选""怎么调参""延迟怎么处理"等面试追问。

## 前置知识

- [PID 控制全解](../../02_基础知识/04_控制理论/02_PID控制全解.md)
- [前馈、增益调度、限幅与饱和](../../02_基础知识/04_控制理论/07_前馈_增益调度_限幅与饱和.md)
- [反馈控制与系统响应基础](../../02_基础知识/04_控制理论/01_反馈控制与系统响应基础.md)
- 上一章：[05 IMU/AHRS 与云台姿态](./05_IMU_AHRS与云台姿态.md)；电机侧：[04 CAN 总线与电机反馈](./04_CAN总线与电机反馈.md)

## 目录

1. 云台控制问题
2. 单轴被控对象模型
3. 为什么用串级：直觉与理由
4. 内环：角速度环的推导
5. 外环：角度环的推导与带宽比
6. 手算：一次完整的串级计算
7. 数字实现：PID、限幅、抗积分饱和、yaw 绕回
8. 整定流程（逐步）
9. 示例仿真：外环增益、延迟、积分对响应的影响
10. 前馈与补偿：重力、摩擦、底盘耦合、发射反冲
11. 指令层：遥控目标、视觉目标、模式仲裁
12. 视觉目标：滤波、绕回、平滑、延迟补偿
13. 在本项目里怎么用
14. 常见错误与故障
15. 面试怎么问、怎么答
16. 要点速记、自测题

---

## 1. 云台控制问题

云台有两个自由度：

- **yaw**：绕竖直轴转动，决定枪口水平指向；
- **pitch**：绕水平轴俯仰，决定枪口高低。

要求：

| 指标 | 含义 |
|---|---|
| 静态精度 | 目标静止时，枪口偏差小 |
| 跟踪精度 | 目标运动时，滞后小 |
| 抗扰动 | 底盘运动、发射反冲、碰撞时指向仍稳 |
| 平顺性 | 不抖动，不突跳（抖动会让视觉识别失败，也会让弹道散布变大） |
| 安全 | 不撞限位，失联时安全放松 |

## 2. 单轴被控对象模型

把某一个轴（例如 pitch）的转动写成牛顿第二定律的旋转形式：

$$
J\dot\omega=K_t\,i-b\,\omega-\tau_g(\theta)-\tau_c\,\mathrm{sgn}(\omega)-\tau_d
$$

| 符号 | 含义 | 单位 |
|---|---|---|
| $\theta$ | 轴的角度 | rad |
| $\omega=\dot\theta$ | 角速度 | rad/s |
| $J$ | 转动惯量（含发射机构、弹丸、线缆） | kg·m² |
| $K_t$ | 折算到该轴的转矩常数（电流→力矩，含减速比） | N·m/A |
| $i$ | 电机电流（电调的电流环保证它跟随指令） | A |
| $b$ | 黏性摩擦系数 | N·m·s/rad |
| $\tau_g(\theta)$ | 重力矩（取决于质心偏移与角度） | N·m |
| $\tau_c$ | 库仑摩擦 | N·m |
| $\tau_d$ | 外部扰动（底盘带动、发射反冲、碰撞） | N·m |

**两个重要观察：**

1. 电流 → 力矩是线性的（电调的电流环保证），所以"从电流指令到角速度"是一阶系统（一个积分惯性环节），"从电流指令到角度"是积分再加一次积分，即二阶（含积分）。
2. 用 IMU 角速度做反馈的前提是：陀螺仪测量的就是 $\omega$，不受编码器零位、减速背隙影响。

**拉氏变换**（忽略重力矩、库仑摩擦、扰动）：

$$
\frac{\Omega(s)}{I(s)}=\frac{K_t}{Js+b},\qquad \frac{\Theta(s)}{I(s)}=\frac{K_t}{s(Js+b)}
$$

**教学示例数值（非项目实测）**：$J=0.02\ \text{kg·m}^2$，$b=0.01\ \text{N·m·s/rad}$，$K_t=0.5\ \text{N·m/A}$，电流限幅 $\pm6$ A。机械时间常数 $J/b=2$ s，意味着没有控制时转动要很久才慢下来；$K_t/J=25\ \text{rad/s}^2$ 每安培：1 A 电流产生 25 rad/s² 的角加速度。

## 3. 为什么用串级：直觉与理由

若只用一个角度环（PD）直接输出电流：$i=K_p e_\theta+K_d \dot e_\theta$，也可以工作，为什么要多一个速度环？

1. **内环把"带摩擦惯性对象"变成近似的一阶积分**：速度环闭环后，从速度指令到实际速度近似 $\omega_v/(s+\omega_v)$，外环看到的对象简单得多（近似积分器），整定独立；
2. **更好的扰动抑制**：扰动力矩直接影响角速度。速度环对扰动的响应速度是 $\omega_v$ 量级，快于通过角度偏差再反推；
3. **天然的限速**：外环输出（目标角速度）可以限幅，直接限制云台最大转速；
4. **微分不再需要**：PD 的微分项等价于对测量角度求导，噪声大；串级里速度由陀螺仪**直接测得**，无需微分；
5. **分工清晰**：速度环用 IMU 陀螺仪（低延迟、无差分噪声），角度环用 IMU 姿态角（绝对）；
6. **工程调参**：先调内环到足够快，再逐步放开外环，不会同时调两个耦合的环。

> **面试要点**：*"The inner rate loop uses the gyro measurement directly, so I don't need to differentiate an angle. It makes the plant seen by the outer loop look like a simple integrator, and it lets me limit the commanded rate."*

## 4. 内环：角速度环的推导

控制器：PI，输入速度误差 $e_\omega=\omega^*-\omega$，输出电流指令：

$$
i=K_{pv}\,e_\omega+K_{iv}\!\int e_\omega\,dt
$$

### 4.1 闭环传递函数

代入对象：

$$
\Omega(s)=\frac{K_t\,(K_{pv}s+K_{iv})}{Js^2+(b+K_tK_{pv})s+K_tK_{iv}}\;\Omega^*(s)\;-\;\frac{s}{Js^2+(b+K_tK_{pv})s+K_tK_{iv}}\;T_d(s)
$$

### 4.2 极点-零点对消的整定

选择 $K_{iv}/K_{pv}=b/J$，分母可因式分解：

$$
Js^2+(b+K_tK_{pv})s+K_tK_{pv}\tfrac bJ
=J\left(s+\tfrac bJ\right)\left(s+\tfrac{K_tK_{pv}}J\right)
$$

分子 $K_tK_{pv}(s+b/J)$ 的零点与分母的一个极点对消，得到

$$
\frac{\Omega(s)}{\Omega^*(s)}=\frac{\omega_v}{s+\omega_v},\qquad \omega_v=\frac{K_tK_{pv}}J
$$

**这就是"内环带宽"：$K_{pv}=\omega_v J/K_t$。**

**手算**：期望 $\omega_v=50$ rad/s：$K_{pv}=50\times0.02/0.5=2.0$ A/(rad/s)；$K_{iv}=K_{pv}\,b/J=2.0\times0.01/0.02=1.0$ A/rad。

### 4.3 扰动响应

扰动传递函数含因子 $s$（分子有 $s$），所以**常值扰动的稳态影响为 0**（积分作用），瞬态峰值约为：

$$
|\Delta\omega|_{\max}\approx\frac{\tau_d}{J\,\omega_v}\quad(\omega_v\gg b/J)
$$

**手算**：$\tau_d=0.5$ N·m：$0.5/(0.02\times50)=0.5$ rad/s $=28.6$ °/s 的瞬时速度偏差。带宽越高，扰动越小。这就是要求内环尽量快的原因（受延迟和噪声限制）。

### 4.4 积分的作用与代价

- $K_{iv}$ 小（对消）：对阶跃响应友好，抗常值扰动慢（积分时间常数 $K_{pv}/K_{iv}=2$ s）；
- $K_{iv}$ 大：抗扰动更快，但阶跃响应有拖尾甚至过冲，相位裕度下降。后面第 9 节用仿真给出数值。

## 5. 外环：角度环的推导与带宽比

外环：比例控制，输入角度误差，输出目标角速度：

$$
\omega^*=K_\theta\,(\theta^*-\theta)
$$

内环近似 $\omega=\frac{\omega_v}{s+\omega_v}\omega^*$，对象再积分：

$$
\frac{\Theta(s)}{\Theta^*(s)}=\frac{K_\theta\,\omega_v}{s^2+\omega_vs+K_\theta\omega_v}
$$

标准二阶系统：$\omega_n^2=K_\theta\omega_v$，$2\zeta\omega_n=\omega_v$。

| 条件 | 结果 |
|---|---|
| 临界阻尼 $\zeta=1$ | $K_\theta=\omega_v/4$ |
| $K_\theta<\omega_v/4$ | 过阻尼，无超调，较慢 |
| $K_\theta>\omega_v/4$ | 欠阻尼，有超调，$\zeta=\tfrac12\sqrt{\omega_v/K_\theta}$ |

**手算**：$\omega_v=50$：临界阻尼 $K_\theta=12.5\ \text{s}^{-1}$，$\omega_n=\sqrt{12.5\times50}=25$ rad/s，2% 调节时间 $\approx5.83/\omega_n=0.233$ s。若 $K_\theta=25$：$\zeta=\tfrac12\sqrt{50/25}=0.707$，超调 $\exp(-\pi\zeta/\sqrt{1-\zeta^2})=4.3\%$，$\omega_n=35.4$。

**带宽比的经验规则**：内环带宽 ≥ 4～10 倍外环带宽。比例 4 对应临界阻尼；再放宽外环，就会过冲并最终受内环延迟影响而失稳。

### 5.1 延迟的影响

实际系统中从传感器到电流生效有延迟 $T_d$（控制周期、CAN 发送、电调响应）。在内环带宽 $\omega_v$ 处引入相位滞后 $\omega_vT_d$：$T_d=3$ ms、$\omega_v=50$ → 0.15 rad = 8.6°；若 $\omega_v=150$ 则 25.8°。内环不能无限提高带宽，通常被延迟、噪声和电流限幅限制。

### 5.2 速度/加速度限幅

外环输出 $\omega^*$ 必须限幅 $\pm\omega_{\max}$（例如 300 °/s，**教学示例**）。大误差时外环 P 给出巨大速度指令，限幅后云台以最大速度转，接近目标时进入线性区。这相当于**速度饱和的分段控制**。

## 6. 手算：一次完整的串级计算

用第 4～5 节的参数（教学示例）：$K_\theta=12.5\ \text{s}^{-1}$，$K_{pv}=2.0$，$K_{iv}=1.0$，控制周期 $T_s=2$ ms，$\omega_{\max}=300$ °/s，电流限幅 $\pm6$ A。

**场景**：yaw 目标 30°，当前 20°，陀螺仪测得 $+25$ °/s，积分状态 $I=0.05$ A。

1. 角度误差：$e_\theta=30-20=10^\circ=0.1745$ rad。
2. 外环：$\omega^*=12.5\times0.1745=2.181$ rad/s $=125^\circ/$s，小于 300 °/s，不限幅。
3. 速度反馈：$\omega=25^\circ/\text{s}=0.4363$ rad/s。
4. 速度误差：$e_\omega=2.181-0.4363=1.745$ rad/s。
5. 积分更新：$I\leftarrow0.05+1.0\times1.745\times0.002=0.05+0.00349=0.0535$ A。
6. 电流：$i=K_{pv}e_\omega+I=2.0\times1.745+0.0535=3.543$ A，未超 6 A。
7. 指令值（M3508 类协议 ±16384↔±20 A，教学示例）：$3.543\times16384/20=2902$。

**接近目标时**：当前 29°，陀螺 $+20$ °/s：$e_\theta=1^\circ$，$\omega^*=12.5^\circ/$s $=0.2182$ rad/s；$\omega=0.3491$ rad/s；$e_\omega=-0.1309$；$i\approx2.0\times(-0.1309)+0.0535=-0.208$ A，**负电流**，这时云台还没到目标，速度环已经开始制动。这就是串级相对于单个 PD 的直观优势：外环给出"应有的速度"，速度环发现太快就刹车。

## 7. 数字实现：PID、限幅、抗积分饱和、yaw 绕回

### 7.1 PID（位置式）参考实现（教学示例，非项目源码）

```c
typedef struct {
    float kp, ki, kd;
    float i_max;      /* 积分项限幅（输出单位） */
    float out_max;    /* 输出限幅 */
    float integ;      /* 积分状态（输出单位） */
    float prev_meas;  /* 微分先行用的上次测量 */
    float d_filt;     /* 微分低通状态 */
    float d_alpha;    /* 微分滤波系数 0~1 */
} pid_t;

float pid_step(pid_t *p, float err, float meas, float dt)
{
    float pterm = p->kp * err;

    /* 条件积分（抗饱和）：仅当未饱和或误差方向会使输出退出饱和时才积分 */
    float cand = p->integ + p->ki * err * dt;
    cand = clampf(cand, -p->i_max, p->i_max);

    /* 微分先行：对测量求导，避免目标阶跃时的微分冲击 */
    float d_raw = -(meas - p->prev_meas) / dt;
    p->d_filt   = p->d_alpha * p->d_filt + (1.0f - p->d_alpha) * d_raw;
    p->prev_meas = meas;
    float dterm = p->kd * p->d_filt;

    float u = pterm + cand + dterm;
    if (fabsf(u) < p->out_max || u * err < 0.0f)  /* 条件积分 */
        p->integ = cand;
    u = pterm + p->integ + dterm;
    return clampf(u, -p->out_max, p->out_max);
}
```

要点：

- 显式乘 $dt$：积分 $K_i e\,dt$，微分 $\Delta e/dt$，使增益有物理意义，不依赖具体周期（但仍需要周期稳定）；
- 抗积分饱和（anti-windup）：输出饱和时停止继续积分，避免饱和解除后"积分放不掉"造成大过冲；
- 微分先行 + 低通：微分作用于测量值而不是误差。

> **【笔记线索】**：你的笔记指出快照里的 PID 实现把比例、积分、微分相加并对积分和总输出限幅，但**没有显式乘除 $\Delta t$**，因此增益隐含了控制周期。这是可以如实讲的设计取舍："the gains absorb the loop period, so I tuned at a fixed period and avoided changing it afterwards"；更好的做法是显式写 $dt$，见 12 数字 PID 实现、限幅、滤波与安全（未单独成章）。

### 7.2 角度误差绕回

yaw 是周期量，误差必须取最短路径：

```c
float wrap_pi(float x)           /* 返回 (-pi, pi] */
{
    while (x >  3.14159265f) x -= 6.28318531f;
    while (x <= -3.14159265f) x += 6.28318531f;
    return x;
}
float e_yaw = wrap_pi(yaw_target - yaw_now);
```

**手算**：当前 $-179^\circ$，目标 $+179^\circ$：直接相减 $358^\circ$；绕回后 $358-360=-2^\circ$，云台只需转 $-2^\circ$。没有绕回时，云台会"甩头"整整一圈。

### 7.3 限位

pitch 有机械限位。典型做法：

1. 通过标定得到 pitch 的最高/最低角（编码器值或相对角）；
2. 目标角限幅在限位内并留余量；
3. 反馈接近硬限位时减小输出或停机。

**手算（教学示例）**：机械范围 $-25^\circ\sim+30^\circ$，软限位留 2°：目标范围 $[-23^\circ, +28^\circ]$。

## 8. 整定流程（逐步）

> 在有风险的环节，务必先把电流限幅设小，并确保能一键放松（RELAX）。

### 步骤 0：确认符号和单位

1. 小电流点动：给一个很小的正电流，看电机往哪转，是否与 IMU 的正方向一致；
2. 手动转云台，看 IMU 角速度符号与角度符号；
3. 统一"目标—反馈—输出"的符号链（符号修正集中在一处）。

### 步骤 1：调速度内环（目标：快速、不振）

1. 先只用 P：$K_{pv}$ 从小到大，同时给速度阶跃（或让外环设成固定的低速指令）；
2. 观察响应：上升快慢、是否出现高频抖动/啸叫；
3. 临界振荡增益 $K_u$ 与周期 $P_u$（教学示例见下）；工程上取 $K_{pv}$ 在 $0.4\sim0.6K_u$；
4. 加积分：$K_{iv}$ 从 $K_{pv}b/J$ 量级开始，向大调，直到常值扰动（用手轻压云台）恢复足够快而阶跃不明显拖尾；
5. 微分通常不需要（陀螺本身就是速度）。若有高频噪声，用低通滤波陀螺读数，注意滤波增加延迟。

**Ziegler–Nichols（ZN）示例数值（教学示例）**：设测得 $K_u=4$，$P_u=0.04$ s。

| 类型 | $K_p$ | $T_i$ | $T_d$ | 说明 |
|---|---|---|---|---|
| P | $0.5K_u=2.0$ | — | — | |
| PI | $0.45K_u=1.8$ | $P_u/1.2=0.0333$ s → $K_i=K_p/T_i=54$ | — | |
| PID | $0.6K_u=2.4$ | $P_u/2=0.02$ s → $K_i=120$ | $P_u/8=0.005$ s → $K_d=0.012$ | 超调约 25%，对云台偏激进 |

ZN 的结果只作为起点；云台通常取更保守（例如把 $K_i$ 取小很多）。

### 步骤 2：调角度外环（目标：无超调或小超调）

1. 内环确定后，外环 P：$K_\theta$ 从小到大，取 $\le\omega_v/4$ 附近；
2. 给阶跃目标，观察超调与调节时间；
3. 若有超调：减小 $K_\theta$，或提高内环带宽；
4. 若需要更小静差：内环积分已使静差为 0，所以外环不需要积分（通常 PD 或纯 P 即可）；
5. 设置速度限幅 $\omega_{\max}$ 避免大角度机动冲击。

### 步骤 3：加前馈

1. 重力补偿（第 10.1 节）：改善 pitch 在不同角度的保持；
2. 目标速度前馈（第 10.4 节）：改善跟踪滞后；
3. 摩擦补偿：改善小幅度抖动和爬行。

### 步骤 4：扰动测试（比赛前必须）

| 测试 | 看什么 |
|---|---|
| 用手拨动云台后松手 | 恢复时间、有无振荡 |
| 底盘高速旋转 | 云台指向是否稳定（小陀螺工况） |
| 发射（含连发）时 | 云台受反冲后的偏差与恢复 |
| 电池低电压 | 控制是否变软（输出被限制） |
| 视觉目标移动 | 跟踪误差、抖动 |
| 切换模式 | 是否突跳 |

### 步骤 5：记录与回归

保存每次改参数的日志（角度误差曲线、电流峰值、温度）。别只凭手感。【待你补全：你的整定记录与最终参数量级（Kθ、Kpv、Kiv、限幅）】

## 9. 示例仿真：外环增益、延迟、积分对响应的影响

下面是用第 2 节教学参数（$J=0.02$，$b=0.01$，$K_t=0.5$，$T_s=2$ ms，额外延迟 2 ms，电流限幅 6 A，速度限幅 300 °/s，陀螺量测无噪声，条件积分）数值仿真得到的结果。**这是教学仿真，不是项目实测**。

### 9.1 10° 阶跃：外环增益 $K_\theta$ 的影响（$K_{pv}=2$，$K_{iv}=1$）

| $K_\theta$ (s⁻¹) | 超调 | 90% 上升时间 | 2% 调节时间 |
|---|---|---|---|
| 12.5（临界阻尼） | 0% | 153 ms | 238 ms |
| 25 | 3.7% | 73 ms | 146 ms |
| 40 | 13.9% | 57 ms | 175 ms |
| 60 | 24.8% | 54 ms | 202 ms |

观察：理论预测的临界阻尼 $t_s\approx233$ ms 与仿真 238 ms 吻合。$K_\theta$ 提高到 25 时上升更快，仍可接受；继续提高，超调快速增大、调节时间反而变长。

### 9.2 0.5 N·m 阶跃扰动：积分增益的影响（$K_{pv}=2$）

| $K_{iv}$ | 峰值角度偏差 | 偏差回到 0.1° 内所需时间 | 10° 阶跃 2% 调节时间 |
|---|---|---|---|
| 1 | 2.07° | 2.50 s | 238 ms |
| 5 | 1.68° | 1.34 s | 235 ms |
| 10 | 1.42° | 0.72 s | 272 ms |
| 20 | 1.14° | 0.42 s | 345 ms |
| 40 | 0.86° | 0.27 s | 334 ms |

观察：$K_{iv}$ 增大时扰动恢复明显加快、峰值下降，但阶跃的调节时间变长（积分拖尾）。这就是"抗扰动 vs 阶跃平滑"的折中。提高 $K_{pv}$（4，$K_{iv}=20$）把峰值降到 0.66°，代价是对延迟与噪声更敏感。

### 9.3 如何解读这两张表

1. 临界阻尼公式给出了可靠的起点；
2. 实际云台上，扰动抑制（发射反冲、底盘带动）通常比阶跃响应更重要，所以 $K_{iv}$ 会比"对消"值大；
3. 真实系统有噪声、量化和更复杂的摩擦，**不要把这些数字当作实机参数**；它们教你的是趋势和判断方法。

## 10. 前馈与补偿：重力、摩擦、底盘耦合、发射反冲

### 10.1 重力补偿（pitch）

若 pitch 轴的质心不在转轴上，重力矩随角度变化：

$$
\tau_g(\theta)=m\,g\,l\cos\theta
$$

其中 $m$ 是俯仰部分质量，$l$ 质心到转轴的水平偏距，$\theta$ 是俯仰角（相对水平）。

**手算（教学示例）**：$m=4$ kg，$l=0.03$ m，水平时 $\theta=0$：$\tau_g=4\times9.8\times0.03=1.176$ N·m；对应前馈电流 $i_{ff}=\tau_g/K_t=1.176/0.5=2.35$ A；$\theta=30^\circ$ 时 $\cos30^\circ=0.866$，$i_{ff}=2.04$ A。

没有前馈时，积分项必须"积"出这 2 A 级别的电流，随角度变化时积分滞后，就会出现角度偏差。有了前馈，积分只处理残差。

$m,l$ 难以精确测得，实际做法是：**让云台在不同角度静止，记录需要的保持电流**，拟合 $i_{ff}=A\cos\theta+B\sin\theta$。优良的机械设计会尽量让质心靠近转轴（配重），使该项小。【待你补全：你的 pitch 是否做重力补偿；若有，参数如何得到】

### 10.2 摩擦补偿

库仑摩擦使云台小幅移动时卡顿（stick-slip）。前馈：$i_c\,\mathrm{sgn}(\omega^*)$，$\omega^*$ 小时用饱和函数平滑（避免符号抖动）。注意过补偿会引起抖振。

### 10.3 底盘旋转耦合（"小陀螺"类工况）

**小陀螺**：底盘持续高速旋转以躲避攻击，同时云台要保持瞄准。影响：

1. **扰动力矩**：轴承与滑环的黏性拖曳 $\tau_d\approx b_c\,\Omega_c$。**手算（教学示例）**：$b_c=0.02$ N·m·s/rad，$\Omega_c=6$ rad/s：$\tau_d=0.12$ N·m → 约 0.24 A（$K_t=0.5$）；开始旋转瞬间有额外的角加速度耦合。
2. **相对角快速变化**：编码器相对角以 $-\Omega_c$ 变化；若误把相对角当绝对角用，云台会随底盘一起转。
3. **IMU 在云台上时**：速度环反馈是绝对角速度，自然抑制底盘带动；此时只需要少量前馈或较高的 $K_{iv}$。

**前馈**：$i_{ff}=b_c\,\Omega_c/K_t$，其中 $\Omega_c$ 来自底盘板/编码器估计；也可以不做，依赖积分。

**关键设计点**：云台 yaw 的**角度环**必须用 IMU 的绝对 yaw，不能只用编码器相对角，否则底盘旋转时指向会漂。同时，**底盘速度指令**需要从云台系旋转到底盘系（见 09 章），否则"向前"会随旋转乱指。

### 10.4 目标速度前馈（跟踪运动目标）

若目标角速度 $\dot\theta^*$ 已知（来自视觉或遥控增量），把它加到外环输出：

$$
\omega^*=K_\theta(\theta^*-\theta)+\dot\theta^*
$$

**手算**：目标以 20°/s 匀速运动，纯 P 外环的稳态跟踪误差 $=\dot\theta^*/K_\theta=20/12.5=1.6^\circ$。加入速度前馈后，理论稳态误差为 0（实际受延迟和速度估计误差限制）。这对自瞄跟踪移动目标很有价值。

### 10.5 发射反冲

每次发射有短时冲量。速度环积分和高带宽可以抑制，但有时在发射时刻加一个"反冲前馈"（根据发射事件，在短时间内给一个与预测冲量相反的电流脉冲）。是否使用取决于测得的反冲量级。【待你补全：发射对云台的扰动量级，及你的处理方式】

## 11. 指令层：遥控目标、视觉目标、模式仲裁

### 11.1 遥控：把摇杆变成"目标角度的增量"

遥控不是"摇杆位置 = 云台角度"，而是"摇杆位置 = 目标角度变化速度"：

$$
\theta^*_{k}=\theta^*_{k-1}+K_{\text{stick}}\cdot s_k\cdot\Delta t
$$

其中 $s_k\in[-1,1]$ 是归一化摇杆，$K_{\text{stick}}$ 是最大转速（rad/s 或 °/s）。

**手算**：$s=0.72$（见 02 章），$K_{\text{stick}}=200$ °/s，$\Delta t=0.002$ s：每步增量 $=0.72\times200\times0.002=0.288^\circ$。**若代码里省略 $\Delta t$ 而写成固定比例，周期变化会改变手感**；【笔记线索】你的笔记提到换算常量隐含了假定的任务周期，周期变化时遥控手感也会变。

鼠标：同理，鼠标位移是"每帧位移"，可低通滤波；滤波会增加延迟，需权衡。

### 11.2 视觉：目标角度/偏差

典型协议：上位机每帧发 yaw、pitch（绝对目标或相对偏差）、目标有效标志、发射建议。MCU 把它们变成云台目标。

### 11.3 模式仲裁

| 模式 | 目标来源 | 进入条件 | 退出/回退条件 |
|---|---|---|---|
| RELAX | 无（电流置零） | 遥控断连、故障、急停 | 使能后进入 ACTIVE |
| ACTIVE（手动） | 遥控增量 | 默认 | 视觉有效且操作手请求 → AUTO |
| AUTO（自瞄） | 视觉目标 | 视觉在线 + 目标有效 + 操作手请求 | 视觉离线/目标丢失/操作手释放 → ACTIVE |

**【笔记线索】**：你的笔记里出现 `GIMBAL_RELAX / GIMBAL_ACTIVE / GIMBAL_AUTO`，自动模式要求视觉状态条件满足，否则回到 ACTIVE。

### 11.4 无扰切换（bumpless transfer）

模式切换瞬间，若新目标与当前姿态差很大，外环会输出大速度指令，云台"跳"一下。做法：

1. **进入 AUTO**：不要立刻把目标设为视觉角；先令滤波器状态 = 当前目标/姿态，然后用限速（如 $\le300$ °/s）把目标逐步推向视觉值；
2. **退出 AUTO 回 ACTIVE**：把手动目标 $\theta^*_{\text{man}}$ 设为**当前姿态或当前目标**（而不是上次手动目标），否则回到旧目标会甩头；
3. **离线回退**：同样先对齐再接管。

**手算（限速）**：切换时目标差 40°，限速 300 °/s，控制周期 2 ms，每步最多 0.6°：需 $40/0.6\approx67$ 步 $=133$ ms 过渡完。

## 12. 视觉目标：滤波、绕回、平滑、延迟补偿

### 12.1 角度滤波的陷阱：不能直接平均角度

对 $179^\circ$ 和 $-179^\circ$ 直接平均得 $0^\circ$，错了（应是 $180^\circ$）。方法：对 $\sin\theta$ 和 $\cos\theta$ 分别滤波，再 `atan2` 还原：

$$
\bar s=\frac{\sin179^\circ+\sin(-179^\circ)}2=0,\qquad \bar c=\frac{\cos179^\circ+\cos(-179^\circ)}2=-0.99985
$$

$\operatorname{atan2}(0,-0.99985)=180^\circ$ ✓。

**【笔记线索】**：你的笔记里写到快照对视觉 yaw 做了 sin/cos 滤波再 atan2 还原，这是个很好的细节，可以主动讲：*"I filtered the vision yaw on the unit circle, so averaging across the plus-minus 180 wrap does not break."*

### 12.2 目标阶梯问题与平滑

视觉更新率（假设 100 Hz，教学示例）低于控制周期（2 ms，500 Hz），所以每 5 个控制周期目标才变一次，呈阶梯：

```text
目标: ──┐   ┌── ──┐   ┌──        控制周期 2ms，每 10ms 一个新值
        └───┘     └───┘
        阶梯变化 -> 外环输出速度指令突变 -> 电流突变 -> 抖动
```

**手算**：目标以 50 °/s 运动，100 Hz 视觉：每帧变化 $0.5^\circ$。外环 $K_\theta=12.5$ s⁻¹：目标阶跃 0.5° 使速度指令突变 $12.5\times0.5=6.25$ °/s。若直接用阶梯目标，速度指令呈锯齿；平滑后每个控制周期仅变化 0.1°。

三种平滑手段：

1. **一阶低通**：$\theta^*_k=\theta^*_{k-1}+\alpha(\theta_{\text{raw}}-\theta^*_{k-1})$，$\alpha=\Delta t/(\tau+\Delta t)$；$\tau=20$ ms、$\Delta t=2$ ms → $\alpha=0.091$。缺点：引入滞后（滤波时间常数）。
2. **限速（slew limit）**：每步变化不超过 $\dot\theta_{\max}\Delta t$；
3. **线性外推/插值**：用最近两帧估计目标速度 $\hat v$，在两帧之间外推：$\theta^*(t)=\theta_{\text{last}}+\hat v\,(t-t_{\text{last}})$，消除阶梯，同时补偿一部分延迟；噪声大时需滤波速度估计，并限制最大外推时间（例如不超过 2 帧）。

【待你补全：你采用的平滑方式与参数】

### 12.3 延迟补偿：用带时间戳的绝对姿态

视觉链路延迟：相机曝光 → 传输 → 识别 → 解算 → USB → MCU，典型几十毫秒（**教学示例 40 ms**）。

**错误做法**：上位机发"相对偏差 $\delta$（目标在图像中相对相机光轴的偏角）"，MCU 收到后用 $\theta^*=\theta_{\text{now}}+\delta$。问题：$\delta$ 是 40 ms 之前的相对偏差，此时云台已经转了。

**手算**：云台正以 90 °/s 转动，延迟 40 ms：云台已多转 $90\times0.04=3.6^\circ$。用 $\theta_{\text{now}}+\delta$ 会产生约 3.6° 的系统性误差，并随云台运动方向改变符号，造成追尾或过冲。

**正确做法**（在抽象层面）：

1. MCU 以 1 kHz（或控制频率）维护一个环形缓冲：记录 $(t_i,\theta_i)$；
2. 视觉帧携带图像采集时间戳 $t_{\text{cap}}$（或上位机用 MCU 之前发来的姿态时间戳反算）；
3. 目标的**绝对角** $\theta^*=\theta(t_{\text{cap}})+\delta$，其中 $\theta(t_{\text{cap}})$ 由缓冲插值得到；
4. 用该绝对角做目标，再叠加预测（目标速度外推到当前时刻 $t_{\text{now}}$）。

**插值手算**：$t=100.000$ s 时 yaw $=10.00^\circ$；$t=100.001$ s 时 $10.09^\circ$（90 °/s）；$t_{\text{cap}}=100.0004$ s：$10.00+0.4\times0.09=10.036^\circ$。

> 更常见的工程分工是：MCU 周期性把当前姿态（带时间戳）发给上位机；上位机做时间对齐、坐标变换、弹道补偿，直接下发**绝对目标角**。这样 MCU 无需维护缓冲区。你的实际分工如何：【待你补全：延迟补偿由 MCU 还是上位机完成、协议里是否有时间戳】。

### 12.4 目标丢失处理

目标有效标志为 0 时：

1. 保持上一个目标一小段时间（容忍闪烁，例如 100～200 ms，教学示例），然后回退到手动；
2. 不要把"无效目标"的数据（可能是 0 或旧值）当目标；
3. 回退时按 11.4 对齐。

### 12.5 视觉与电控的接口边界

| 内容 | 谁负责 |
|---|---|
| 目标识别、位姿解算 | 视觉上位机 |
| 弹道（下坠、飞行时间）补偿 | 通常视觉上位机（需要弹速） |
| 目标预测（运动模型） | 视觉上位机（也可简单外推在 MCU） |
| 云台闭环、限幅、模式 | MCU（你） |
| 弹速、姿态回传 | MCU → 上位机 |

你的 CV 描述是 *converting remote-control and vision targets into continuous attitude commands*：重点在"连续"——MCU 一侧负责把不连续、延迟、带噪声的外部目标变成连续的、平顺的、受限的姿态指令。

## 13. 在本项目里怎么用

- **【CV】**：*Gimbal cascaded PID: built pitch/yaw angle–rate cascaded PID using IMU/AHRS and motor feedback, converting remote-control and vision targets into continuous attitude commands for stable aiming and tracking.* 以及技能栏 cascaded PID, feedforward。
- **【笔记线索】**（需核对）：
  1. 角度环输出 `gyro_set`（目标角速度），速度环输出 `give_current`；
  2. yaw 角度环用 IMU 绝对 yaw，处理 ±180° 绕回；pitch 反馈由 IMU 与另一块板的 pitch 合成（符号需核对），pitch 输出取负以匹配方向；
  3. 角速度反馈来自 IMU 陀螺仪（单位转换后低通滤波）；
  4. 遥控摇杆/鼠标变为目标角度增量，鼠标输入滤波，pitch 目标限幅，yaw 目标绕回；
  5. 自动模式用视觉 `robot_ctrl.yaw/pitch` 作目标，视觉 yaw 做 sin/cos 滤波；视觉离线或状态不满足时退回手动；
  6. PID 实现没有显式 dt；周期结构估算约 3 ms。
- **典型补全项**：重力前馈、速度前馈、无扰切换、延迟补偿。
- **要自己补全**：参数量级、内外环带宽、整定记录、是否做重力补偿/前馈、延迟补偿方式、小陀螺工况的表现、遇到的抖动问题。

> **诚实表述建议**：如果你的实现里没有重力前馈或延迟补偿，面试时可以说："In our version, the integral term handled the gravity load and the vision latency was dealt with on the PC side; if I were to improve it I would add ..."，并给出本章的设计。这样既诚实，又展示了深度。

## 14. 常见错误与故障

| # | 现象 | 原因 | 定位 | 处理 |
|---|---|---|---|---|
| 1 | 通电后云台猛转或飞出 | 符号反、目标未对齐当前角 | 点动测试、上电打印目标/反馈 | 统一符号；使能前把目标设为当前角 |
| 2 | 高频啸叫/抖动 | 速度环 $K_{pv}$ 太大、噪声、滤波延迟、CAN 延迟 | 看电流指令频谱；降 $K_{pv}$ | 降增益；合理滤波；缩短延迟 |
| 3 | 低频晃动（几 Hz） | 外环过强或内环过慢，或积分过大 | 对比不同 $K_\theta$ | 降 $K_\theta$、提内环带宽、减小 $K_i$ |
| 4 | pitch 在某角度下沉/爬升 | 重力矩未补偿、积分不足 | 记录保持电流 vs 角度 | 加重力前馈 |
| 5 | yaw 过 ±180° 甩头 | 误差未绕回 | 观察过零时误差 | `wrap_pi` |
| 6 | 自瞄时追尾/滞后 | 视觉延迟未补偿、无速度前馈 | 对比运动目标下误差 | 绝对姿态对齐 + 外推 |
| 7 | 视觉接管瞬间突跳 | 无扰切换缺失 | 模式切换时看目标差 | 限速 + 初始化滤波器 |
| 8 | 视觉目标锯齿抖动 | 阶梯目标、帧间无平滑 | 看目标曲线 | 低通 / 限速 / 外推 |
| 9 | 小陀螺时云台指向漂 | 外环误用相对角、扰动大 | 单独测试底盘旋转 | 用 IMU 绝对角；调大 Ki；前馈 |
| 10 | 积分饱和后大过冲 | 抗饱和缺失 | 看积分项曲线 | 条件积分/限幅 |
| 11 | 低电压时变软或抖 | 电流被电池限制 | 看电池电压与电流指令 | 低压保护；增益调度 |
| 12 | 发射瞬间云台偏离 | 反冲扰动 | 对齐发射事件与角度偏差 | 提高 Ki/带宽；反冲前馈 |

## 15. 面试怎么问、怎么答

**Q1. Why a cascaded loop for the gimbal?**
- 英文：*"The inner rate loop uses the gyro measurement directly, so I do not need to differentiate the angle, and it makes the plant look like an integrator to the outer loop. It also gives fast disturbance rejection and lets me limit the commanded rate. I tune the inner loop first, then the outer loop."*
- 追问 1：*How did you choose the bandwidths?* → 内环至少 4～10 倍外环；临界阻尼 $K_\theta=\omega_v/4$；受延迟和噪声限制。
- 追问 2：*What does each loop output?* → 外环输出目标角速度，内环输出电机电流指令（由电调电流环执行）。
- 追问 3：*Why PI inner and P outer?* → 速度环需要积分以抵抗摩擦、重力、扰动，外环不需要积分（内环积分已保证位置无静差）。

**Q2. How did you tune it?**
- 英文：*"First I verified signs with small currents. Then I tuned the rate loop with proportional gain only until it was fast but not buzzing, added a small integral for load and friction, and then raised the angle loop gain until I saw slight overshoot and backed off. Finally I tested disturbances: pushing the gimbal, spinning the chassis, and firing."*
- 追问：*What if it oscillates at high frequency?* → 降速度环 Kp、检查滤波/延迟/CAN 周期、检查机械松动。
- 追问：*How do you know the oscillation is in the inner or outer loop?* → 频率：内环振荡频率高（数十 Hz），外环低（几 Hz）；断开外环固定速度指令看内环是否单独稳定。

**Q3. How do you handle vision latency?**
- 英文：*"Vision targets are tens of milliseconds old, so adding a relative offset to the current attitude is wrong while the gimbal is moving. The target must be anchored to the attitude at capture time, using timestamps, and then extrapolated forward. I also smooth the stair-stepped updates so the rate command stays continuous."*
- 追问：*How large is the error if you do not compensate?* → 速度 × 延迟，如 90°/s × 40 ms = 3.6°。
- 追问：*Where did you do it, on the MCU or the PC?* → 【待你补全；诚实地说实际分工】。

**Q4. How do you avoid a jump when switching from manual to auto?**
- 英文：*"I initialise the filtered target to the current command and rate-limit the move toward the vision target. When leaving auto I set the manual target to the current attitude."*

**Q5. What disturbances matter most for a hero gimbal?**
- 英文：*"The chassis motion and spinning, the recoil of the large projectile, and the gravity load on pitch. The IMU-based rate loop rejects the first two, and gravity can be handled with feedforward or integral action."*

**Q6. Why use the IMU rather than the motor encoder for angle feedback?**
- 英文：*"The encoder gives the angle relative to the chassis. If the chassis rotates, the pointing direction in the world changes even if the encoder doesn't. The IMU measures the absolute attitude of the gimbal body, so holding the aim in the world frame works regardless of the chassis."*

**Q7. Derive the closed-loop of the cascade.** 见第 4～5 节；白板上写出 $\omega_v=K_tK_{pv}/J$，$K_\theta=\omega_v/4$ 的临界阻尼条件即可。

## 本章要点速记

1. 云台单轴：$J\dot\omega=K_ti-b\omega-\tau_g-\tau_d$；电流环在电调里。
2. 串级：外环（角度，P）→ 目标角速度 → 内环（速度，PI）→ 电流。
3. 内环 PI 对消 $b/J$：$\omega_v=K_tK_{pv}/J$；$K_{pv}=\omega_vJ/K_t$。
4. 外环临界阻尼：$K_\theta=\omega_v/4$；内环带宽 ≥ 4～10 倍外环。
5. 外环输出限幅 = 限速；内环积分需抗饱和。
6. yaw 误差必须绕回；角度滤波用 sin/cos。
7. 重力前馈 $mgl\cos\theta/K_t$；速度前馈消除运动目标的稳态滞后 $\dot\theta^*/K_\theta$。
8. 底盘耦合：用 IMU 绝对角做反馈；小陀螺工况需要测试。
9. 视觉延迟：用带时间戳的绝对姿态补偿，误差 = 角速度 × 延迟。
10. 阶梯目标要平滑（低通/限速/外推）；模式切换要无扰。
11. 整定顺序：符号 → 内环 → 外环 → 前馈 → 扰动测试 → 记录。
12. 周期和延迟是控制参数的一部分；PID 若不含 dt，增益绑定周期。

## 自测题

1. 已知 $J=0.05$，$K_t=0.8$，要求 $\omega_v=40$ rad/s，求 $K_{pv}$；若 $b=0.02$，求对消用的 $K_{iv}$ 与临界阻尼的 $K_\theta$。
   *答：$K_{pv}=40\times0.05/0.8=2.5$；$K_{iv}=2.5\times0.02/0.05=1.0$；$K_\theta=40/4=10$ s⁻¹。*
2. 目标 25°、当前 22°、陀螺 +10 °/s，$K_\theta=12$，$K_{pv}=2$（单位按 rad），不计积分。求电流。
   *答：$e_\theta=3^\circ=0.05236$ rad；$\omega^*=0.6283$ rad/s；$\omega=0.1745$；$e_\omega=0.4538$；$i=0.908$ A。*
3. 云台 yaw 当前 +170°，目标 −175°，误差是多少？
   *答：−175−170=−345 → +15°（绕回后）。*
4. 视觉延迟 50 ms，云台角速度 60 °/s，不补偿的误差多大？
   *答：3°。*
5. 目标以 30°/s 匀速，$K_\theta=15$，纯 P 外环的稳态跟踪误差？加入速度前馈后呢？
   *答：30/15=2°；加入速度前馈后理论为 0。*
6. 解释为什么不能直接对 179° 和 −179° 平均。
   *要点：角度是周期量；用 sin/cos 平均后 atan2 还原得 180°。*
7. 为什么速度环要对积分做抗饱和？
   *要点：电流限幅饱和时积分仍累积，退出饱和后需要很长时间才能放掉，造成大过冲。*
8. 内环带宽 80 rad/s，延迟 4 ms，相位滞后多少度？
   *答：0.32 rad ≈ 18.3°。*

## 相关链接

- 上一章：[05 IMU/AHRS 与云台姿态](./05_IMU_AHRS与云台姿态.md)
- 下一章：[07 发射机构：摩擦轮与拨盘](./07_发射机构_摩擦轮与拨盘.md)
- 数字 PID 工程细节：12 数字 PID 实现、限幅、滤波与安全（未单独成章）
- 基础：[PID 控制全解](../../02_基础知识/04_控制理论/02_PID控制全解.md)、[前馈、增益调度、限幅与饱和](../../02_基础知识/04_控制理论/07_前馈_增益调度_限幅与饱和.md)
- 八股：[控制八股](../../03_八股知识/03_机器人学/02_控制八股.md)
- 相关项目：[P1 AIRS 球形机器人平衡控制](../P1_AIRS球形机器人平衡控制/README.md)（级联 PID 的另一应用）
- 学习笔记：[P3 本对话问答逐字稿](../../05_学习笔记归档/P3/P3_本对话问答逐字稿.md)（第 4 组）
