# 07 主从跟踪与 MIT 电机接口

> **一句话定位**：主从示教的第二半——"从臂怎么跟上主臂"，以及贯穿 P2 所有机械臂线的底层接口 **MIT 电机控制帧**。这一章把 MIT 帧的五个字段、8 字节怎么打包、位置–速度跟踪怎么写、主臂关节怎么映射到从臂（偏置/方向/比例）、延迟与饱和会带来什么现象，全部拆开并配手算例子。

**本章学完你能：**

- 默写 MIT 控制律 $\tau=k_p(q^*-q)+k_d(\dot q^*-\dot q)+\tau_{ff}$，说清每个字段的单位和范围；
- 手工把一帧 MIT 指令打包成 8 个字节，并解释为什么"发 0 力矩"并不严格为 0；
- 说清使能/失能/置零命令帧和回包帧的格式；
- 区分"指令"与"反馈"：同一个 $q$ 在主臂里被拿去算重力、又被拿去当从臂的指令，身份不同；
- 写出主从映射 $q_F^*=s_i(q_L-q_{L,off,i})+q_{F,off,i}$，并解释偏置、方向、减速比、夹爪；
- 推导：位置–速度跟踪中**速度前馈**怎样减小滞后；**从臂没有重力前馈**时的稳态下垂 $\tau_g/k_p$；
- 估算延迟造成的跟踪滞后，以及饱和/起始错位的风险与缓解办法；
- 描述同步启动、同步停止、失联时的合理策略。

**前置知识：**

- 上一章 [06 URDF/KDL 动力学与重力补偿](./06_URDF_KDL动力学与重力补偿.md)
- 电机、编码器与力矩控制：[05_电机_编码器_驱动与力矩控制](../../02_基础知识/11_嵌入式与通信/05_电机_编码器_驱动与力矩控制.md)
- CAN 基础：[03_CAN_FDCAN与电机通信](../../02_基础知识/11_嵌入式与通信/03_CAN_FDCAN与电机通信.md)
- PD 控制与限幅：[02_PID控制全解](../../02_基础知识/05_控制理论/02_PID控制全解.md)

## 目录

1. 要解决的问题：从臂要"同步地跟"
2. 系统结构：两只臂、一个映射、一条通信
3. MIT 控制律与五个字段
4. 一帧 MIT 报文是怎么打包的（含手算）
5. 使能、失能、置零与回包
6. 用字段组合出不同行为
7. 主臂回顾与"指令 vs 反馈"
8. 从臂：位置–速度跟踪
9. 主从关节映射：偏置、方向、比例、夹爪
10. 跟踪误差的三个来源：速度滞后、重力下垂、通信延迟
11. 饱和与起始错位
12. 启动、停止与失联策略
13. 常见问题与定位
14. 面试怎么问、怎么答
15. 本章要点速记 / 自测题 / 相关链接

---

## 1. 要解决的问题：从臂要"同步地跟"

CV 原句：*...follower position–velocity tracking, enabling low-resistance hand-guided teaching with synchronous follower motion.*

目标有三条：

1. 主臂被人拖动，**从臂的每个关节同步复现**主臂的关节角（"同步"=延迟小、误差小）；
2. 从臂**不能抖**、不能在启动时猛冲；
3. 主从各自的**零位、方向、安装**可能不一样，必须靠映射对齐。

注意主从示教与 VR 线的区别（笔记第 35 组）：**VR 线是"手→末端位姿→逆运动学→关节"，而主从线是"主臂关节→从臂关节"，中间没有逆运动学、没有雅可比。** 两臂结构相同（或相近）时，关节空间直接对应是最自然的做法。

---

## 2. 系统结构：两只臂、一个映射、一条通信

```text
         ┌──────────── 主臂 (Leader) ───────────┐
人手 ───►│ 7 个电机 + 编码器                      │
         │ ① 读 q_L, q̇_L (电机回包)             │
         │ ② 算 g(q_L) → MIT 力矩帧 (kp=kd=0)   │──► 托举
         └──────────────┬─────────────────────┘
                        │ q_L, q̇_L  (作为参考)
                        ▼
                 ┌──────────────┐
                 │ 关节映射 f     │  偏置 / 方向 / 比例
                 └──────┬───────┘
                        │ q*_F, q̇*_F
                        ▼    (ROS 2 话题 或 同进程变量)
         ┌──────────── 从臂 (Follower) ─────────┐
         │ MIT 位置速度帧: pos=q*_F vel=q̇*_F kp kd │──► 电机追踪
         │ ③ 读 q_F, q̇_F (回包, 供监控/保护)     │
         └─────────────────────────────────────┘
```

学习笔记里项目的实现描述（你们逐段核对过的 C++ 主从代码）：

- Leader 分支：读电机状态 → `motor_to_joint` → KDL `JntToGravity` → × `gravity_comp_ratio` → MIT 帧（pos/vel/kp/kd=0，torque=effort）；
- Follower 分支：读取 joint references → `joint_to_motor` → 发送 `pos, vel, Kp, Kd, torque=0`。

> **提示**：笔记中强调"主从通信传递的是参考，下游形成闭环"：主臂不直接'驱动'从臂电机；它把关节参考发出去，从臂自己的 MIT 控制器（电机内环）去追。因此从臂的稳定性只取决于从臂自己的 $k_p,k_d$、延迟、饱和，主臂只是"信号源"。

---

## 3. MIT 控制律与五个字段

MIT 模式（源自 MIT Cheetah 的关节控制接口，达妙 DM 系列电机采用）一次发送五个量：

$$
\boxed{\ \tau_{cmd}=k_p\,(q^{*}-q)+k_d\,(\dot q^{*}-\dot q)+\tau_{ff}\ }
$$

| 字段 | 符号 | 含义 | 单位 | 仓库库中的编码范围/位数 |
|---|---|---|---|---|
| `q` / `pos` | $q^*$ | 期望位置 | rad | $[-p_{\max},p_{\max}]$，16 位（$p_{\max}=12.5$） |
| `dq` / `vel` | $\dot q^*$ | 期望速度 | rad/s | $[-v_{\max},v_{\max}]$，12 位 |
| `kp` | $k_p$ | 位置刚度 | N·m/rad | $[0,500]$，12 位 |
| `kd` | $k_d$ | 速度阻尼 | N·m·s/rad | $[0,5]$，12 位 |
| `tau` | $\tau_{ff}$ | 前馈力矩 | N·m | $[-t_{\max},t_{\max}]$，12 位 |

电机内部（驱动器固件）以很高频率（通常 kHz 量级，具体由厂商固件决定，非你可控）执行这个控制律，并用电流环把 $\tau_{cmd}$ 变成电流。

**为什么这个接口妙？** 它把"位置环 + 速度环 + 前馈"打包成一个公式，且**所有项都在电机内部以高频计算**，上位机只需要以较低频率（几百 Hz）更新五个数。上位机发送的周期决定了"指令更新的粒度"，而电机内部的 PD 环保持"局部稳定"。

### 3.1 重要：作为指令的 $q^*,\dot q^*$ 和作为反馈的 $q,\dot q$

- 你发出去的 `pos/vel` 是**指令**（$q^*,\dot q^*$）；
- 电机编码器读到的是**反馈**（$q,\dot q$），电机每收到一帧指令，就回一帧状态（位置、速度、力矩）；
- 重力补偿用的是**回读的反馈**；指令里的 `pos/vel` 在 $k_p=k_d=0$ 时根本不起作用（笔记第 28 组你确认过）。

---

## 4. 一帧 MIT 报文是怎么打包的（含手算）

### 4.1 线性量化

每个浮点量用线性映射变成无符号整数（仓库 `XArmcontroller.py` 的 `_f2u`）：

$$
u=\Big\lfloor\frac{x-x_{\min}}{x_{\max}-x_{\min}}\,(2^{b}-1)\Big\rfloor .
$$

解码：$x=\dfrac{u}{2^b-1}(x_{\max}-x_{\min})+x_{\min}$。分辨率 $=(x_{\max}-x_{\min})/(2^b-1)$。

注意是 `int()` 截断（向零取整），**不是四舍五入**，所以平均有半个最低位的偏置。

### 4.2 8 字节布局

按源码 `_pack_mit`：

```text
字节:   d0       d1       d2       d3                d4       d5       d6                d7
内容:  q[15:8]  q[7:0]   dq[11:4] dq[3:0] kp[11:8]  kp[7:0]  kd[11:4] kd[3:0] tau[11:8] tau[7:0]
位数:   8        8        8        4+4               8        8        4+4               8
        └─ 位置 16 位 ─┘ └ 速度 12 位 ┘└ kp 12 位 ──────┘ └ kd 12 位 ──────┘└ tau 12 位 ─────┘
```

### 4.3 手算例子（示例，电机按 DM4340：$p_{\max}=12.5,\ v_{\max}=8,\ t_{\max}=28$）

指令：`q=0, dq=0, kp=0, kd=0.5, tau=1.25`。

| 量 | 公式 | 整数 $u$ | 十六进制 |
|---|---|---|---|
| 位置 | $\lfloor(0+12.5)/25\times65535\rfloor=32767$ | 32767 | 0x7FFF |
| 速度 | $\lfloor(0+8)/16\times4095\rfloor=2047$ | 2047 | 0x7FF |
| $k_p$ | $\lfloor0/500\times4095\rfloor=0$ | 0 | 0x000 |
| $k_d$ | $\lfloor0.5/5\times4095\rfloor=409$ | 409 | 0x199 |
| 力矩 | $\lfloor(1.25+28)/56\times4095\rfloor=2138$ | 2138 | 0x85A |

组帧：

- d0 = 0x7F，d1 = 0xFF（位置高/低字节）；
- d2 = 0x7FF >> 4 = 0x7F；
- d3 = (0x7FF & 0xF) << 4 | (0x000 >> 8) = 0xF0；
- d4 = 0x00；
- d5 = 0x199 >> 4 = 0x19；
- d6 = (0x199 & 0xF) << 4 | (0x85A >> 8) = 0x90 | 0x08 = 0x98；
- d7 = 0x85A & 0xFF = 0x5A。

得到 `7F FF 7F F0 00 19 98 5A`（已用程序核对）。解码力矩：$2138/4095\times56-28=1.2376$ N·m，比 1.25 少 0.012（截断 + 量化）。

### 4.4 "发 0 力矩"不严格为 0

力矩 $=0$ 时 $u=\lfloor4095/2\rfloor=2047$，解码 $=2047/4095\times2t_{\max}-t_{\max}=-t_{\max}/4095$：DM4310 为 $-0.0024$ N·m，DM8009 为 $-0.0132$ N·m。同理位置、速度的"0"也有微小偏置。这些偏置通常小到可以忽略，但在**调试"静止时电机为什么缓慢漂"**时要想到它。

### 4.5 分辨率一览（库中 DM 系列配置）

| 电机 | 位置分辨率（16 位） | 速度分辨率（12 位） | 力矩分辨率（12 位） |
|---|---|---|---|
| DM4310 | $3.8\times10^{-4}$ rad | 0.0147 rad/s | 0.0049 N·m |
| DM4340 | $3.8\times10^{-4}$ | 0.0039 | 0.0137 |
| DM8009 | $3.8\times10^{-4}$ | 0.0220 | 0.0264 |

> **本项目取值：** 实机电机与库里对 xArm 的配置一致：关节 1–2 DM8009、3–4 DM4340、5–7 DM4310；型号决定量程与分辨率。

---

## 5. 使能、失能、置零与回包

### 5.1 控制命令帧

仓库 `_send_cmd`：数据区为 **7 个 0xFF + 1 个命令字节**，帧 ID 为电机从机 ID（0x01–0x07，夹爪 0x08）：

| 命令 | 命令字节 | 作用 |
|---|---|---|
| 使能 | 0xFC | 进入 MIT 控制模式，开始响应 |
| 失能 | 0xFD | 退出，电机自由（无力） |
| 置零 | 0xFE | 把当前位置设为零点（改零位） |

库的 `enable_all` 对每个电机**连发 3 次**，每次间隔 2 ms，最后等待 0.2 s。重复发送的原因是 CAN 上单帧可能丢失、而使能是一个关键动作；`disable_all` 同样重复 3 次。

> **注意**：`set_zero`（0xFE）会**永久**改变电机的零位参考（或至少在断电前）。调试时误发会让整条链的零位错位，所有补偿和映射都会偏。把它放在显式的"校零流程"里，不要放在常规代码路径。

### 5.2 回包

每收到一帧指令，电机回一帧（回包 ID = 从机 ID + 0x10，称为主机 ID，如 0x11–0x17）。解析（`_parse_frame`）：

```text
d0: 电机 ID/状态位
位置 = (d1<<8 | d2)            16 位  → [-p_max, p_max]
速度 = (d3<<4 | d4>>4)          12 位  → [-v_max, v_max]
力矩 = ((d4&0xF)<<8 | d5)        12 位  → [-t_max, t_max]
(d6, d7: 温度等，视固件而定)
```

`recv_all` 等待所有 7 个主机 ID 的回包（超时默认 10 ms）。回包里的力矩是**电机反馈的输出力矩**，可以拿来和理论补偿对比（见 06 章验证步骤）。

---

## 6. 用字段组合出不同行为

只改五个数，就能得到完全不同的行为：

| 目标行为 | `pos` | `vel` | `kp` | `kd` | `tau` | 说明 |
|---|---|---|---|---|---|---|
| **刚性位置保持** | 目标 | 0 | 大 | 适中 | 0 或前馈 | 从臂追踪的基本形态 |
| **位置–速度跟踪** | $q^*$ | $\dot q^*$ | 大 | 适中 | 0 或前馈 | 带速度前馈，减滞后（第 10.1 节） |
| **纯力矩（零刚度）** | 0 | 0 | 0 | 0（或很小） | $\tau_g$ | 主臂：只出补偿力矩 |
| **黏性阻尼（电子刹车）** | 0 | 0 | 0 | $>0$ | 0 | 推它时有速度正比的阻力，像泡在油里 |
| **速度模式** | 任意 | $\dot q^*$ | 0 | $>0$ | 0 | $\tau=k_d(\dot q^*-\dot q)$，追速度 |
| **柔顺位置** | 目标 | 0 | 小 | 小 | $\tau_g$ | 重力补偿 + 弹簧，有柔顺 |

> **经验**：`torque=0` **不代表电机没力**。当 $k_p,k_d>0$ 时，位置/速度误差照样产生力矩——这是学习笔记里反复提的一个误解（"Follower 发的是 torque0，是不是没力？"→ 不是，位置速度环在出力）。

---

## 7. 主臂回顾与"指令 vs 反馈"

主臂（详见 [06 章](./06_URDF_KDL动力学与重力补偿.md)）：

$$
\text{MIT}(pos=0,\ vel=0,\ k_p=0,\ k_d\approx0,\ \tau_{ff}=\rho\cdot g(q_{L,\text{反馈}})),
$$

$\rho$ 即 `gravity_comp_ratio`（学习笔记：`[1,1,1,1,1,1,0.8]`）。

同一个主臂的 $q_L$，在系统里有**两个身份**：

1. **状态**：喂给 KDL，计算 $g(q_L)$；
2. **参考**：经映射后变成从臂指令 $q_F^*$。

初学者容易搞混的一个问题（笔记第 28 组）："电机里是不是已经有 pos/vel 了，为什么还要读？" → 电机内部确实有编码器读数，但**上位机需要通过回包拿到它**才能计算重力、做映射、做监控。这就是 `recv_all` 存在的原因。

---

## 8. 从臂：位置–速度跟踪

从臂每个周期发送：

$$
\text{MIT}\big(pos=q_F^*,\ vel=\dot q_F^*,\ k_p,\ k_d,\ \tau_{ff}=0\big).
$$

电机内部：$\tau=k_p(q_F^*-q_F)+k_d(\dot q_F^*-\dot q_F)$。

### 8.1 增益的量级（仓库参考配置，非项目实测）

`XArmcontroller.py`（v1）为每个电机 ID 设了默认 $(k_p,k_d)$：

| 电机 ID | 对应关节 | $k_p$ | $k_d$ |
|---|---|---|---|
| 0x01, 0x02 | J1, J2（大关节） | 300 | 2.5 |
| 0x03, 0x04 | J3, J4 | 150 | 2.5 |
| 0x05, 0x06 | J5, J6（腕） | 40 | 0.8 |
| 0x07 | J7 | 30 | 0.8 |
| 0x08 | 夹爪 | 10 | 0.9 |

`XArmcontroller_v2.py` 的默认 `ARM_KP_DEFAULT = [280,280,240,240,35,35,35]`、`ARM_KD_DEFAULT = [5,5,5,5,1.2,1.2,1.2]`（注释说明 $k_d$ 范围为 $[0,5]$，原值 20/10 被钳到 5）。启动时的"移动到初始位置"另用较柔的 $k_p=50$/10 和 $k_d=1.2$/0.3。

规律（也是面试可讲的"经验规则"）：**大关节（肩、肘）承担重量、惯量大 → 增益大；腕部关节惯量小 → 增益小**，否则小惯量配大增益容易振荡。

> **本项目取值：** 从臂 $k_p=[50,50,50,50,10,10,10]$、$k_d=[1.2,1.2,1.2,1.2,0.3,0.2,0.3]$，逐关节不同（大关节刚度大、腕部小）；带速度前馈（`vel` 取主臂速度并经一阶低通），重力前馈不带（`torque=0`），残余重力下沉靠较大的 $k_p$ 吃掉，稳态误差小于 0.02 rad。

### 8.2 位置–速度前馈：为什么要发 `vel`

设一个关节等效为 $J\ddot q+b\dot q=\tau$（$J$ 惯量，$b$ 黏性摩擦），目标是匀速斜坡 $q^*=\omega t$。稳态时 $\ddot q=0,\ \dot q=\omega$：

- **带速度前馈**（发 $\dot q^*=\omega$）：$\tau=k_pe+k_d(\omega-\omega)=k_pe=b\omega\Rightarrow\ e=\dfrac{b\omega}{k_p}$；
- **不带速度前馈**（发 $\dot q^*=0$）：$\tau=k_pe-k_d\omega=b\omega\Rightarrow\ e=\dfrac{(b+k_d)\omega}{k_p}$。

差别 $\dfrac{k_d\,\omega}{k_p}$。数字（示例，非实测）：$k_p=50,\ k_d=1.2,\ b=0.1,\ \omega=1$ rad/s：带前馈 $e=0.002$ rad（0.11°），不带 $e=0.026$ rad（1.5°）。

所以**位置–速度跟踪**（发 `pos` 和 `vel`）比纯位置跟踪滞后小得多——这也是 CV 里写 *position–velocity tracking* 而不是 *position tracking* 的原因。速度参考来自主臂回读的 $\dot q_L$（可先低通，因为速度反馈噪声比位置大）。

---

## 9. 主从关节映射：偏置、方向、比例、夹爪

主臂第 $i$ 个关节读数**不能原样**当从臂第 $i$ 个关节指令。需要映射：

$$
q_{F,i}^{*}=s_i\,\big(q_{L,i}-q_{L,off,i}\big)\,r_i+q_{F,off,i},\qquad
\dot q_{F,i}^{*}=s_i\,r_i\,\dot q_{L,i}.
$$

- $s_i=\pm1$：**方向**（两臂镜像安装或电机正方向不同时为 −1）；
- $q_{off}$：**偏置**（零位差）；
- $r_i$：**比例**（减速比不同、或有意缩放时；同型号手臂一般为 1）；
- 速度只受方向和比例影响，偏置在求导时消失。

另外还有两层转换（笔记里的 `motor_to_joint`、`joint_to_motor`）：**电机坐标 ↔ 关节坐标**（电机零点、减速比、方向）。完整链路：

```text
主臂电机读数 ─motor_to_joint─► 主臂关节角 ─映射 f─► 从臂关节角 ─joint_to_motor─► 从臂电机指令
```

### 9.1 手算（示例）

某关节：主臂读数 $q_L=0.50$ rad，主臂零位偏置 $q_{L,off}=0.10$，从臂偏置 $q_{F,off}=-0.05$，方向 $s=-1$（镜像），$r=1$：

$$
q_F^*=(-1)(0.50-0.10)(1)+(-0.05)=-0.40-0.05=-0.45\ \text{rad}.
$$

若主臂速度 $\dot q_L=0.8$ rad/s，则 $\dot q_F^*=-0.8$ rad/s。

### 9.2 偏置怎么标定

1. 把主、从臂都摆到同一个**机械零位/参考姿态**（用夹具或标记）；
2. 读两边的关节角，差即是 $q_{off}$；
3. 再摆第二、第三个姿态做**验证**（残差应小于 1°）；
4. 方向 $s_i$：小幅度移动主臂一个关节，看从臂对应关节是否同向。

### 9.3 夹爪

夹爪量程和语义不同（库中：0 = 夹紧，−1 = 张开，且有单独的机械限位 $[-60^\circ,0]$）。笔记提醒"夹爪需要单独映射"。夹爪常需要**更柔的增益**（库默认 $k_p=10$）并设置力矩上限，防止夹坏物体。

### 9.4 URDF 方向与电机方向不一致

URDF 里的关节轴可能是 $-x$ 而电机正方向是反的（例如右臂 URDF 中 `joint2` 的 `axis xyz="-1 0 0"`）。必须明确：**映射层只管一次符号**，别在 URDF、KDL、映射层各"修"一次，否则会相消。

---

## 10. 跟踪误差的三个来源：速度滞后、重力下垂、通信延迟

### 10.1 速度滞后（见 8.2）

带前馈时 $e=b\omega/k_p$，不带时多出 $k_d\omega/k_p$。

### 10.2 重力下垂（**从臂没有重力前馈**时）

笔记里 Follower 分支的 `torque=0`。如果同时 $k_p$ 不大，从臂在重力作用下会有**稳态偏差**：静止时 $k_pe=g(q)\Rightarrow e=g(q)/k_p$。用 06 章的 URDF 推算（右臂 `INIT_POS` 姿态，教学推算）：

| 关节 | $g$（N·m） | $k_p$（v1 默认） | 稳态偏差 $e=g/k_p$ |
|---|---|---|---|
| J1 | 3.37 | 300 | 0.0112 rad = 0.64° |
| J2 | 2.57 | 300 | 0.0086 rad = 0.49° |
| J3 | 1.92 | 150 | 0.0128 rad = 0.73° |
| J4 | 3.36 | 150 | 0.0224 rad = 1.28° |

若用启动时的软增益 $k_p=50$：J1 为 $3.37/50=0.067$ rad = 3.9°。结论：**增益越高，下垂越小；不想靠高增益，就给从臂也加重力前馈 $\tau_{ff}=g(q)$**（那时 $e\to0$）。这是一个很好的"改进空间"回答（不要说成你已做）。

### 10.3 通信/计算延迟

总延迟 $D$ 由几段相加：主臂读回包 + 计算 + 话题传输 + 从臂指令发送 + 从臂电机更新周期。跟踪滞后（匀速运动时）

$$
e_{lag}\approx\omega\,D.
$$

示例（非实测）：$D=20$ ms，$\omega=1$ rad/s → 0.02 rad = 1.1°；快速运动 $\omega=2$ → 2.3°。

延迟还影响稳定性：高 $k_p$ + 延迟 → 相位裕度下降 → 振荡。经验处理：先降 $k_p$ 到稳定，再逐步提高；$k_d$ 提供相位超前，但 $k_d$ 太大会放大速度噪声、触发抖动。

**本项目取值：** 端到端延迟约 30 ms（主臂读取约 5 ms + 发布与接收约 5 ms + 总线往返约 2 ms + 从臂电机内环响应约 15~20 ms）；测量方法是让主臂做一次快速小幅阶跃，在同一时间轴上画主从关节角，读两条曲线 10%~50% 处的时间差。同步性看叠加曲线：稳态误差小于 0.02 rad，运动中滞后约 3 个控制周期（100 Hz 下约 30 ms）。

---

## 11. 饱和与起始错位

### 11.1 一个危险的场景：启动时两臂姿态不一致

假设刚使能时从臂在 $q_F$，主臂参考是 $q_L$，差 $\Delta=0.2$ rad。立刻切入跟踪：

$$
\tau=k_p\Delta=150\times0.2=30\ \text{N·m}
$$

超过 DM4340 的 $t_{\max}=28$ N·m。电机会饱和输出，手臂**猛冲过去**。解决办法：

1. **对齐阶段**：先用"位置模式 + 平滑插值"把从臂移动到主臂当前姿态，然后再进入实时跟踪。仓库 v2 的 `arm_control` 就是这样做的：minimum-jerk 轨迹 + 先读真实起点 + 25 ms 的接入稳定阶段；
2. **参考限速**：限制 $|q^*_k-q^*_{k-1}|\le\dot q_{\max}\,dt$，保证参考不会瞬间跳变；
3. **软启动增益**：增益从小逐步增大（ramp）。

### 11.2 力矩饱和与上位限幅

电机输出受 $t_{\max}$ 和驱动器电流限制；编码范围内的指令不会越界（需要钳位，见 [05 章](./05_约束与限位.md)）。饱和后控制律变成"开环最大力矩"，跟踪误差会增大，积分类控制器会饱和卷绕（MIT 控制律没有积分项，天然免于卷绕，但你自己加了积分就要抗饱和）。

### 11.3 参考超限位

主臂可以被人拖到机械限位，映射后的从臂参考也可能超出从臂限位（偏置/镜像）。需要在映射后再做**钳位**，并记录钳位事件。

---

## 12. 启动、停止与失联策略

典型流程（教学总结，细节以你们实现为准）：

```text
启动:   使能主/从 → 读状态 → 从臂插值到主臂当前姿态(对齐) → 主臂切入重力补偿 → 从臂切入跟踪
运行:   主臂: 读 q → 算 g → 下发力矩 ;  映射 → 从臂跟踪
停止:   从臂保持/回安全位 → 主臂力矩渐降(×0.85/步) → 失能
失联:   见下
```

**失联（主臂数据停止到达）**：从臂应该怎么办？选项：

| 策略 | 做法 | 风险 |
|---|---|---|
| 保持最后参考 | 继续追最后一个 $q^*$ | 若最后参考本身是移动中的，手臂会"停在半空"，仍然出力，需要提示 |
| 参考冻结 + 增益降低 | 软化手臂 | 手臂会下垂（因从臂无重力前馈） |
| 失能 | 电机无力 | 手臂可能砸下 |
| 回安全位 | 插值到预设姿态 | 需要空间 |

实际要根据任务选。关键是：**必须有"参考年龄"检测**（时间戳比较），而不是默默沿用旧值。**本项目取值：** 参考年龄 >100 ms 时从臂冻结在当前位置（`pos` 取反馈，不沿用旧目标），>500 ms 失能；恢复后先把从臂插值到主臂当前姿态再重新开启跟踪。

仓库脚本的收尾习惯（`nullspace_control.py` 的 `finally`）：力矩渐降 60 步（每步 ×0.85，间隔 10 ms）再失能，防止突然失能下坠。

---

## 13. 常见问题与定位

| 现象 | 原因 | 定位/处理 |
|---|---|---|
| 从臂某个关节方向反 | $s_i$ 错 | 小幅度移动主臂该关节，看从臂方向 |
| 从臂整体有恒定偏移 | 偏置错 | 在两个参考姿态对比 |
| 从臂缓慢下垂 | 无重力前馈 + 增益偏低 | 提高 $k_p$ 或加 $g(q)$ 前馈 |
| 从臂追不上 | 滞后：无速度前馈 / 延迟 / 增益低 | 发 `vel`；测延迟 |
| 从臂高频抖动 | 增益过高 + 延迟 / 速度噪声 | 降 $k_p$；速度参考先低通；调 $k_d$ |
| 使能瞬间猛冲 | 起始错位 + 饱和 | 先对齐再跟踪 |
| 某关节到限位后"卡" | 映射后超限位 | 钳位并记录 |
| 夹爪夹得太紧或太松 | 夹爪映射/增益 | 单独标定，限力矩 |
| 主臂拖动变重 | 重力补偿问题（见 06） | 参考 06 的验证步骤 |
| 回包缺失 | 总线/ID/过滤问题 | 看 `recv_all` 收到的 ID 集合 |

---

## 14. 面试怎么问、怎么答

**Q1. What is the MIT motor interface, and how did you use it for both arms?**

> The MIT interface sends five numbers per motor in one CAN frame: target position, target velocity, kp, kd and a feed-forward torque. The driver computes torque as kp times position error plus kd times velocity error plus the feed-forward term, at a high rate inside the motor. For the leader I set position, velocity, kp and kd to zero and send only the gravity torque. For the follower I send the mapped position and velocity with gains and no feed-forward torque, so the motor loop tracks the leader.

中文要点：五字段 + 公式；主臂纯力矩；从臂位置速度。

*追问*：If the follower sends torque zero, does the motor produce no force? → 不是，kp/kd 误差项仍出力。

**Q2. Why send a velocity reference and not only position?**

> Velocity feed-forward removes the lag during motion. For a ramp, the steady-state error with feed-forward is friction times speed over kp, while without it there is an extra kd times speed over kp. In a small example with kp 50, kd 1.2 and speed 1 radian per second, that is 0.002 versus 0.026 radians. So the follower looks much more synchronous.

**Q3. How do you map the leader joints to the follower?**

> Per joint: direction sign, zero offset and scale, plus the motor-to-joint conversion on each side. For example q follower equals sign times the leader angle minus its offset, plus the follower offset. I calibrate the offsets by putting both arms in the same reference pose, and verify with two other poses. The velocity mapping only uses the sign and scale, because the offset disappears when you differentiate.

**Q4. What limits tracking quality?**

> Three things: velocity lag if there is no feed-forward, steady gravity sag if the follower has no gravity feed-forward and the gain is moderate, and end-to-end latency, which gives roughly speed times delay of lag. For example 20 milliseconds at 1 radian per second is about one degree. Higher gain helps the first two but makes delay-induced oscillation more likely, so I raise gains step by step on the real arm.

**Q5. How do you start the follower safely?**

> I never switch on tracking when the two arms are in different poses, because kp times the error could exceed the motor's torque limit. First I interpolate the follower to the leader's current pose with a smooth minimum-jerk trajectory, then start streaming references, and I limit the reference rate. On shutdown I ramp the torque down before disabling.

**Q6. Tell me about a problem you hit with this interface.** *(请选一个你真实遇到过的；下面是可替换的诚实素材)*

> One thing I learned is that the float-to-integer packing must clamp its inputs, otherwise an out-of-range torque wraps and the sign flips. Another is that the right-arm joint 2 needed a larger kd to stay stable under pure torque mode, which we saw in the reference code comments. I noticed the clamping issue while reading the driver code, and made sure our wrapper clamps before packing.

---

## 本章要点速记

1. MIT 控制律：$\tau=k_p(q^*-q)+k_d(\dot q^*-\dot q)+\tau_{ff}$；五字段：pos、vel、kp、kd、torque。
2. 报文 8 字节：位置 16 位 + 速度、kp、kd、力矩各 12 位；线性量化，`int()` 截断；"发 0"≠严格 0。
3. 命令帧：7×0xFF + 0xFC/0xFD/0xFE（使能/失能/置零）；回包 ID = 从机 ID + 0x10；使能重复发 3 次。
4. 主臂：kp=kd≈0，torque=ρ·g(q_反馈)；从臂：pos、vel、kp、kd，torque=0 但仍出力。
5. 同一个 $q_L$：状态（算重力）与参考（给从臂）两个身份。
6. 位置–速度跟踪：带速度前馈稳态误差 $b\omega/k_p$，不带多 $k_d\omega/k_p$。
7. 从臂无重力前馈 → 稳态下垂 $g/k_p$；加 $g(q)$ 前馈可消除。
8. 关节映射：方向 $s_i$、偏置、比例、夹爪；电机坐标 ↔ 关节坐标另有一层。
9. 延迟滞后 $\approx\omega D$；高增益 + 延迟 → 振荡。
10. 启动先对齐再跟踪；停止先渐降再失能；失联要有"参考年龄"检测。

## 自测题

1. 把 `q=0, dq=0, kp=0, kd=0.5, tau=1.25`（DM4340）打包成 8 字节。（`7F FF 7F F0 00 19 98 5A`。）
2. Leader 的 `kp=kd=0, torque=g`，为什么人推起来轻？Follower 的 `torque=0` 为什么仍有力？（无回复力；误差项仍出力。）
3. $k_p=50,k_d=1.2,b=0.1,\omega=1$，带/不带速度前馈的稳态误差？（0.002 / 0.026 rad。）
4. 从臂 J4 的 $g=3.36$、$k_p=150$，稳态下垂多少？（0.0224 rad ≈ 1.3°。）
5. 为什么启动时要先对齐？（$k_p\Delta$ 可能超过 $t_{\max}$，猛冲。）
6. 方向 $s=-1$，偏置 $q_{L,off}=0.1$，$q_{F,off}=-0.05$，$q_L=0.5$，$q_F^*$ 多少？（−0.45 rad。）

## 相关链接

- 上一章：[06 URDF/KDL 动力学与重力补偿](./06_URDF_KDL动力学与重力补偿.md)
- 下一章：[08 差速底盘运动学、轮速 PID 与里程计](./08_差速底盘运动学_轮速PID与里程计.md)
- 执行链：[10 ROS2-CAN 执行链](./10_ROS2_CAN执行链.md)
- 基础：[电机_编码器_驱动与力矩控制](../../02_基础知识/11_嵌入式与通信/05_电机_编码器_驱动与力矩控制.md)、[CAN_FDCAN与电机通信](../../02_基础知识/11_嵌入式与通信/03_CAN_FDCAN与电机通信.md)、[PID控制全解](../../02_基础知识/05_控制理论/02_PID控制全解.md)
- 八股：[控制八股](../../03_八股知识/03_机器人学/02_控制八股.md)、[嵌入式与机电八股](../../03_八股知识/03_机器人学/06_嵌入式与机电八股.md)
- 学习笔记：[逐字稿第 35–36 组](../../05_学习笔记归档/P2/P2_本对话问答逐字稿.md)
