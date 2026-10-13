# 06 MuJoCo / Mink / Pinocchio / KDL 工具链

> **一句话定位**：给你一份机器人模型文件，这四个库分别能帮你算什么——MuJoCo 是"物理仿真器兼运动学/动力学计算器"，Mink 是建在 MuJoCo 上的"逆运动学求解器"，Pinocchio 和 KDL 是"刚体运动学/动力学算法库"；本章用同一个两连杆小机械臂，把它们的数据结构、核心函数、手算结果和常见坑讲到可以直接上手和应对面试。

**本章学完你能：**

- 说清楚 MuJoCo 里 `MjModel` / `MjData`、`qpos` / `qvel` / `ctrl`、`nq` 与 `nv`、`mj_forward` 与 `mj_step` 各自是什么，以及"改 `qpos` 之后为什么必须 `mj_forward` 才能读到新的几何量"；
- 说清楚 `ctrl` 到底是力矩还是目标位置——取决于模型里执行器（actuator）怎么定义；
- 说出 Mink 的五个核心对象（Configuration、Task、Limits、`solve_ik`、`integrate`）怎样配合，手算一次阻尼最小二乘（DLS）逆运动学并解释它如何抑制奇异点附近的速度放大；
- 用 Pinocchio 调出 RNEA（逆动力学）、CRBA（质量矩阵）、广义重力项、质心；手算一个两连杆的 $g(q)$ 与 $M(q)$ 并对上程序输出；
- 说清楚 KDL 里 Tree、Chain、`ChainDynParam`、`JntToGravity` 的关系，以及"关节顺序""链端点""末端负载""基座倾斜时的重力向量"四个常见坑；
- 对比四个库，给出选型理由，并把它们与 P1 的质心前馈、P2 的 VR 约束逆运动学和主从重力补偿对上号。

**前置知识：**

- [正逆运动学与雅可比](../03_坐标变换与运动学/03_正逆运动学与雅可比.md)
- [奇异性、阻尼最小二乘、零空间](../03_坐标变换与运动学/04_奇异性_阻尼最小二乘_零空间.md)
- [机械臂动力学、重力补偿、URDF、KDL](../03_坐标变换与运动学/06_机械臂动力学_重力补偿_URDF_KDL.md)
- [最优化与二次规划](../01_数学/04_最优化与二次规划.md)（Mink 每步解一个 QP）
- [Python 工程与科学计算](./03_Python工程与科学计算.md)

> **关于代码的说明**：本章 Python 示例（MuJoCo、Mink、Pinocchio）是教学示例，非项目源码；撰写时在 Python 3.13、mujoco 3.15.0、mink 1.3.0、pin 4.1.0 环境下实际运行过，文中的"运行输出"来自该次运行。各库的 API 会随版本变化，你自己的环境请以所装版本的文档为准。KDL 的 C++ 示例**未编译核对**，仅作结构示意。

**目录**

1. 为什么需要这些库：问题与全景
2. 贯穿本章的小例子：两连杆平面臂
3. MuJoCo
4. Mink
5. Pinocchio
6. KDL
7. 四者区别与选型
8. 在本项目里怎么用（P1 / P2）
9. 常见错误与故障现象
10. 面试怎么问、怎么答
11. 自测题
12. 本章要点速记与相关链接

---

## 1. 为什么需要这些库：问题与全景

### 1.1 你手里有什么，想要什么

你有的是一个**机器人描述文件**（URDF 或 MJCF），里面写着：有哪些连杆、每个连杆多重、质心在哪、关节怎么连、轴朝哪个方向。你想要的是**数字**：

| 想要的量 | 数学名字 | 谁能算 |
|---|---|---|
| 给定关节角，末端在哪、朝向如何 | 正运动学 $T(q)$ | 四者皆可 |
| 关节速度与末端速度的关系 | 雅可比 $J(q)$ | 四者皆可 |
| 给定末端目标，关节该怎么动 | 逆运动学 | **Mink**（基于 MuJoCo 的差分 IK）；Pinocchio 也可自己搭；KDL 有 IK 求解器 |
| 保持静止需要多大关节力矩 | 重力项 $g(q)$ | MuJoCo（`qfrc_bias`）、Pinocchio、KDL |
| 给定 $q,\dot q,\ddot q$ 需要多大力矩 | 逆动力学（RNEA） | 同上 |
| 给定力矩，下一刻怎么运动 | 正动力学 + 积分 | **MuJoCo**（`mj_step`）；Pinocchio 有 ABA |
| 整机质心在哪 | $c(q)$ | Pinocchio、MuJoCo（`subtree_com`）、KDL（自己累加） |
| 接触、摩擦、碰撞 | 接触动力学 | **MuJoCo** |

### 1.2 一个常见的误解：把"库"和"SDK"和"仿真"混为一谈

学习中最容易混淆的是下面三个概念（也是用户在学习笔记里反复问到的）：

- **库（library）/ API**：一组你的程序可以调用的函数。Mink、Pinocchio、KDL、MuJoCo 的 Python/C++ 接口都是库。它们只在你的电脑里做**计算**，不会让真实电机转动一度。
- **SDK**：通常指某个**硬件厂商**提供的、用来和**真实设备**通信的开发包（比如驱动某个电机或机械臂的 CAN 接口库）。它负责"把数字变成电机命令"。
- **仿真器（simulator）**：按物理规律推进"虚拟机器人"的程序。MuJoCo 既是仿真器，也是一个很好的运动学/动力学计算库——**你完全可以不开窗口、不推进时间，只用它算"此刻这个姿势下连杆在哪里"。**

一句话：**库负责算，SDK 负责连真机，仿真器负责在电脑里"演"。**

```text
   目标 (末端要去哪)                    真机
       │                                 ▲
   [Mink 求解器: 算关节小步]               │ 电机命令
       │                                 │
   [MuJoCo/Pinocchio: 提供模型, 算 J, FK]   [厂商 SDK / CAN 驱动]
       │                                 ▲
       └────────→ 关节目标 ───────────────┘
```

---

## 2. 贯穿本章的小例子：两连杆平面臂

后面所有库都用同一个模型，结果可以互相对照。

### 2.1 参数

- 平面臂在竖直的 $x$–$z$ 平面内运动，重力沿 $-z$；
- 两根连杆，长度 $l_1=l_2=0.4\ \text{m}$，各 $m_1=m_2=1\ \text{kg}$，质心在各自杆的中点（距关节 $l_{c1}=l_{c2}=0.2\ \text{m}$）；
- 两个转动关节，轴为世界 $-y$ 方向，所以**正方向的转动让杆从 $+x$ 向 $+z$ 抬起**（$q_1$ 是第一根杆与水平线夹角，$q_2$ 是相对第一根杆再转的角）；
- 转动惯量取很小的 $0.001\ \text{kg·m}^2$（对重力项无影响）。
- 以上数值均为**教学示例**，非任何项目的实测。

```text
 z ↑
   │          ● 末端 (tip)
   │         /
   │        /  link2 (l2=0.4)
   │      ○ 关节2
   │     /
   │    /   link1 (l1=0.4)
   │   /
   ○─────────────→ x      ○ 关节1 在原点 (MuJoCo 例子里抬高 1 m)
```

### 2.2 手算 1：重力项 $g(q)$

势能（取关节 1 所在高度为零）：

$$
U(q)=m_1 g\, l_{c1}\sin q_1 + m_2 g\,\big(l_1\sin q_1 + l_{c2}\sin(q_1+q_2)\big)
$$

重力项 $g(q)=\partial U/\partial q$（保持静止所需的关节力矩，方向与关节正方向一致）：

$$
g_1=(m_1 l_{c1}+m_2 l_1)\,g\cos q_1 + m_2 l_{c2}\,g\cos(q_1+q_2),\qquad
g_2=m_2 l_{c2}\,g\cos(q_1+q_2)
$$

代入 $g=9.81$：

- $q=(0,\ 0)$，臂完全水平伸直：$g_1=(0.2+0.4)\times9.81+0.2\times9.81=7.848$，$g_2=0.2\times9.81=1.962$（单位 N·m）；
- $q=(0,\ \pi/2)$，大臂水平、小臂竖直向上：$g_1=0.6\times9.81+0.2\times9.81\times\cos 90^\circ=5.886$，$g_2=0$。第二个关节为零，因为小臂竖直，重力过关节轴，没有力臂。

**验证**：MuJoCo 的 `qfrc_bias` 与 Pinocchio 的 `computeGeneralizedGravity` 在这两个姿态下输出 `[7.848 1.962]` 与 `[5.886 0]`，与手算一致。

### 2.3 手算 2：质量矩阵 $M(q)$

对 $q_2=\pi/2$（$\cos q_2=0$）：

$$
M_{11}=I_1+I_2+m_1l_{c1}^2+m_2\big(l_1^2+l_{c2}^2+2l_1l_{c2}\cos q_2\big)=0.002+0.04+0.16+0.04=0.242
$$

$$
M_{12}=I_2+m_2\big(l_{c2}^2+l_1l_{c2}\cos q_2\big)=0.001+0.04=0.041,\qquad M_{22}=I_2+m_2l_{c2}^2=0.041
$$

Pinocchio 的 `crba` 输出 $M=\begin{bmatrix}0.242&0.041\\0.041&0.041\end{bmatrix}$，与手算一致。

### 2.4 手算 3：雅可比

末端位置（$x$–$z$ 平面，轴 $-y$，$q$ 为向上为正的角）：

$$
x=l_1\cos q_1+l_2\cos(q_1+q_2),\qquad z=l_1\sin q_1+l_2\sin(q_1+q_2)
$$

$$
J=\begin{bmatrix}-l_1\sin q_1-l_2\sin(q_1+q_2) & -l_2\sin(q_1+q_2)\\ l_1\cos q_1+l_2\cos(q_1+q_2) & l_2\cos(q_1+q_2)\end{bmatrix}
$$

在 $q=(0,\pi/2)$：$J=\begin{bmatrix}-0.4&-0.4\\0.4&0\end{bmatrix}$。MuJoCo `mj_jacSite` 的位置雅可比（取 $x$、$z$ 两行）正是 $\begin{bmatrix}-0.4&-0.4\\0.4&0\end{bmatrix}$。

### 2.5 同一个模型的两种文件写法

URDF（Pinocchio / KDL 使用）：

```xml
<?xml version="1.0"?>
<robot name="arm2r">
  <link name="base"/>
  <link name="link1">
    <inertial><origin xyz="0.2 0 0"/><mass value="1.0"/>
      <inertia ixx="0.001" iyy="0.001" izz="0.001" ixy="0" ixz="0" iyz="0"/></inertial>
  </link>
  <link name="link2">
    <inertial><origin xyz="0.2 0 0"/><mass value="1.0"/>
      <inertia ixx="0.001" iyy="0.001" izz="0.001" ixy="0" ixz="0" iyz="0"/></inertial>
  </link>
  <link name="tip"/>
  <joint name="joint1" type="revolute">
    <parent link="base"/><child link="link1"/><origin xyz="0 0 0"/>
    <axis xyz="0 -1 0"/><limit lower="-3.14" upper="3.14" effort="50" velocity="3"/>
  </joint>
  <joint name="joint2" type="revolute">
    <parent link="link1"/><child link="link2"/><origin xyz="0.4 0 0"/>
    <axis xyz="0 -1 0"/><limit lower="-3.14" upper="3.14" effort="50" velocity="3"/>
  </joint>
  <joint name="joint_tip" type="fixed">
    <parent link="link2"/><child link="tip"/><origin xyz="0.4 0 0"/>
  </joint>
</robot>
```

MJCF（MuJoCo 使用，机械臂基座抬高 1 m，另加两个 motor 执行器）：

```xml
<mujoco model="arm2r">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.002"/>
  <worldbody>
    <body name="link1" pos="0 0 1">
      <joint name="joint1" type="hinge" axis="0 -1 0"/>
      <inertial pos="0.2 0 0" mass="1" diaginertia="0.001 0.001 0.001"/>
      <body name="link2" pos="0.4 0 0">
        <joint name="joint2" type="hinge" axis="0 -1 0"/>
        <inertial pos="0.2 0 0" mass="1" diaginertia="0.001 0.001 0.001"/>
        <site name="tip" pos="0.4 0 0"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <motor name="m1" joint="joint1"/>
    <motor name="m2" joint="joint2"/>
  </actuator>
</mujoco>
```

> **注意（MJCF 的角度单位）**：MJCF 默认角度单位是**度**。如果你在 `<joint range="...">` 里写了弧度数值却没写 `<compiler angle="radian"/>`，限位会被当成度，机器人一开始就"超限"。本章撰写时作者自己就踩到了这个坑——给 Mink 设置 `ConfigurationLimit` 后求解直接报"无可行解"，原因就是 $\pm 3.1$ 被当成了 $\pm3.1^\circ$。

---

## 3. MuJoCo

### 3.1 它是什么

MuJoCo（Multi-Joint dynamics with Contact）是一个物理引擎，用于带接触的多关节系统。对我们来说，它同时是：

1. 一个**模型加载器**：读 MJCF（也能读 URDF）；
2. 一个**运动学/动力学计算库**：给定 $q$ 算各连杆位姿、雅可比、质量矩阵、偏置力；
3. 一个**仿真器**：给定控制输入，积分得到下一个状态。

### 3.2 两个核心对象

| 对象 | 内容 | 类比 |
|---|---|---|
| `MjModel`（`m`） | **静态**：结构、质量、几何、执行器定义、选项（`m.opt.timestep`、重力）。创建后一般不改 | 图纸 |
| `MjData`（`d`） | **动态**：当前状态与所有中间量：`qpos`、`qvel`、`ctrl`、各连杆位姿 `xpos`/`xmat`、`site_xpos`、`qfrc_bias`、接触、时间 `time` | 机器人此刻的状态和计算缓冲区 |

`MjData` 由 `MjModel` 创建。一个模型可以配多个 `MjData`（比如并行仿真，或者"想象一下如果……"的计算，不影响真实状态）。

### 3.3 状态与控制

| 量 | 含义 | 维度 |
|---|---|---|
| `qpos` | 广义位置（关节位置） | `nq` |
| `qvel` | 广义速度 | `nv` |
| `qacc` | 广义加速度（算出来的） | `nv` |
| `ctrl` | 执行器输入 | `nu` |
| `qfrc_bias` | 偏置力 $c(q,\dot q)$（科氏、离心、重力） | `nv` |
| `xpos` / `xquat` | 各 body 的世界位置 / 姿态 | `nbody` |
| `site_xpos` | 各 site 的世界位置 | `nsite` |

### 3.4 `nq` 与 `nv` 为什么可以不同

**铰链（hinge）/滑动（slide）关节**：位置 1 个数，速度 1 个数，所以 `nq=nv`。

**球关节与自由关节**用四元数表示姿态：

| 关节 | `qpos` 个数 | `qvel` 个数 | 为什么 |
|---|---|---|---|
| hinge / slide | 1 | 1 | 一个角/位移 |
| ball | 4（四元数） | 3（角速度） | 姿态有 3 个自由度，四元数用 4 个数表示 |
| free | 7（位置 3 + 四元数 4） | 6（线速度 3 + 角速度 3） | 同上 |

所以一个浮动基机器人 `nq` 通常比 `nv` 大。运行例子（作者核对）：一个自由体加一个铰链，`nq=8, nv=7`。

**由此带来的规则**：
- ⛔ 不要写 `qpos += qvel * dt`——维度可能不一致，而且四元数不能逐元素相加；
- ✅ 位置的积分用 `mujoco.mj_integratePos(m, qpos, qvel, dt)`，或 Mink 的 `integrate_inplace`；
- MuJoCo 的四元数顺序是 **(w, x, y, z)**，Pinocchio 是 **(x, y, z, w)**，混用两库时必须转换（见 5.6）。

### 3.5 `mj_forward` 与 `mj_step`

- **`mj_forward(m, d)`**：根据当前 `qpos`、`qvel`、`ctrl` 计算所有派生量（连杆位姿、雅可比相关量、力、加速度 `qacc`），**不推进时间**。
- **`mj_step(m, d)`**：做一次 `mj_forward` 的计算，然后用 `qacc` 积分得到下一个 `qpos`、`qvel`，`d.time += m.opt.timestep`。
- **`mj_kinematics`**：只做运动学，更轻量（想只要几何时用）。

**这就是用户疑惑过的核心**："MuJoCo 不是只负责仿真吗，为什么能算连杆位置？" ——因为连杆位置是 $q$ 的函数，MuJoCo 在 `mj_forward` 里就是在做正运动学；整个过程没有"让机器人真的动"。

**重要陷阱**：你直接改 `d.qpos` 只是改了一个数组，`d.site_xpos` 等派生量**不会自动更新**，必须再调用一次 `mj_forward`。运行结果（作者核对）：初始 tip 为 `[0.8 0 1]`，把 `qpos[1]` 改成 1.0 后不调用 `mj_forward` 读到的仍是 `[0.8 0 1]`；调用后变成 `[0.616 0 1.337]`。

### 3.6 `ctrl` 的含义取决于执行器类型

这也是用户反复问到的点：`ctrl` 不天然等于"关节角"。

| 执行器（MJCF） | `ctrl` 的含义 | 产生的力（简化） |
|---|---|---|
| `<motor joint=.../>` | 力 / 力矩（乘以 `gear`，默认 1） | $\tau = \text{gear}\cdot u$ |
| `<position joint=... kp=.../>` | 目标位置 | $\tau = k_p(u-q)$（可带阻尼项 `kv`） |
| `<velocity joint=... kv=.../>` | 目标速度 | $\tau = k_v(u-\dot q)$ |

**因此**：把关节角数组写进一个 `<motor>` 的 `ctrl`，MuJoCo 会把它当成 N·m 的力矩，而不是目标角；同样地，如果模型里是 `<position>`，你写入力矩就会得到离谱的目标。

三种"让仿真机械臂动起来"的方式，语义完全不同：

| 方式 | 做法 | 说明 |
|---|---|---|
| A. 力矩驱动 | `motor`，`ctrl = 力矩` | 你自己写控制器（PD、重力补偿） |
| B. 位置驱动 | `position`，`ctrl = 目标角` | MuJoCo 内部是一个 PD 控制，机器人会"追"目标，不是瞬间到达 |
| C. 直接设状态 | 改 `qpos` 再 `mj_forward` | 纯运动学回放，没有动力学，不模拟电机 |

**区分 `ctrl`（目标）和 `qpos`（实际）**：关节不会因为你写了目标就瞬间到位；下一周期要读新的 `qpos`，重新计算。

### 3.7 完整示例：读几何、求重力项、动力学对比（教学示例，已运行核对）

```python
import numpy as np
import mujoco

XML = """
<mujoco model="arm2r">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.002"/>
  <worldbody>
    <body name="link1" pos="0 0 1">
      <joint name="joint1" type="hinge" axis="0 -1 0"/>
      <inertial pos="0.2 0 0" mass="1" diaginertia="0.001 0.001 0.001"/>
      <body name="link2" pos="0.4 0 0">
        <joint name="joint2" type="hinge" axis="0 -1 0"/>
        <inertial pos="0.2 0 0" mass="1" diaginertia="0.001 0.001 0.001"/>
        <site name="tip" pos="0.4 0 0"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <motor name="m1" joint="joint1"/>
    <motor name="m2" joint="joint2"/>
  </actuator>
</mujoco>
"""
m = mujoco.MjModel.from_xml_string(XML)
d = mujoco.MjData(m)
print("nq, nv, nu =", m.nq, m.nv, m.nu, " dt =", m.opt.timestep)

# (1) 纯运动学: 改 qpos -> mj_forward -> 读几何; 时间不前进
d.qpos[:] = [0.0, np.pi / 2]
mujoco.mj_forward(m, d)
print("tip =", d.site_xpos[0], " time =", d.time)

# (2) 重力项: qvel=0 时, qfrc_bias 就是 g(q)
print("g(q) =", np.round(d.qfrc_bias, 3))

# (3) 雅可比 (3 x nv)
jacp = np.zeros((3, m.nv)); jacr = np.zeros((3, m.nv))
sid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "tip")
mujoco.mj_jacSite(m, d, jacp, jacr, sid)
print("jacp =\n", np.round(jacp, 3))

# (4) 动力学: 无控制 0.5 s vs 用 ctrl = g(q) 补偿 0.5 s
d1 = mujoco.MjData(m); d1.qpos[:] = [0.0, np.pi / 2]
for _ in range(250):
    mujoco.mj_step(m, d1)
d2 = mujoco.MjData(m); d2.qpos[:] = [0.0, np.pi / 2]
for _ in range(250):
    mujoco.mj_forward(m, d2)
    d2.ctrl[:] = d2.qfrc_bias           # motor: ctrl 就是力矩 (N*m)
    mujoco.mj_step(m, d2)
print("no ctrl  :", np.round(d1.qpos, 3), " t =", round(d1.time, 3))
print("g(q) comp:", np.round(d2.qpos, 3))
```

运行输出（作者核对）：

```text
nq, nv, nu = 2 2 2  dt = 0.002
tip = [0.4 0.  1.4]  time = 0.0
g(q) = [5.886 0.   ]
jacp =
 [[-0.4 -0.4]
  [ 0.   0. ]
  [ 0.4  0. ]]
no ctrl  : [-1.777 -0.975]  t = 0.5
g(q) comp: [0.    1.571]
```

解读：

- tip 在 `[0.4, 0, 1.4]`：机械臂基座在 $z=1$，大臂水平（$x=0.4$），小臂竖直向上（$z=1+0.4$）；
- `g(q) = [5.886, 0]` 与 2.2 节手算一致；
- 无控制时 0.5 s 后手臂已摆落（`q1` 变成 $-1.777$ rad，即杆摆到水平线以下）；用 `ctrl = g(q)` 补偿后，0.5 s 内几乎纹丝不动，说明**重力补偿 = 把偏置力里的重力部分用执行器抵消**——这正是主从示教里 Leader 臂"轻轻一推就动"的原理（见 8.2）；
- 雅可比 `jacp` 的第 1、3 行就是 2.4 节手算的 $\begin{bmatrix}-0.4&-0.4\\0.4&0\end{bmatrix}$（中间行是 $y$ 方向，平面臂恒为零）。

> **提示**：`qfrc_bias` 在 $\dot q\ne0$ 时还包含科氏力和离心力。只想要纯重力时，先设 `qvel=0` 再 `mj_forward`。

### 3.8 MuJoCo 常见错误

| 错误 | 现象 | 修正 |
|---|---|---|
| 改了 `qpos` 不调 `mj_forward` | 读到的 `xpos`/`site_xpos` 是旧的 | 改完状态必调 `mj_forward` |
| 往 `motor` 的 `ctrl` 里放关节角 | 手臂乱甩 / 掉落 | 查执行器类型；motor 要给力矩，要位置目标用 `position` |
| `qpos += qvel*dt` | 浮动基姿态错误、维度错 | `mj_integratePos` |
| 度/弧度混用（MJCF 默认度） | 限位、初始角不对 | `<compiler angle="radian"/>` |
| 模型里 `gravity="0 0 0"` | 仿真"漂浮"，重力补偿看不出效果 | 检查 `<option gravity>` |
| 用 `d.xpos` 当 body 的某点 | 取的是 body 原点，不是质心/末端 | 用 `site_xpos` 或 `xipos`（惯性/质心） |
| 时间步长不匹配 | 调用频率与 `timestep` 不同却当成"实时" | 控制周期 = 若干个 `timestep`，循环调用 `mj_step` |

---

## 4. Mink

### 4.1 它解决什么问题

给定末端目标位姿（来自 VR 手柄、规划器或任何上层），求关节该怎么动。直觉上就是逆运动学；工程上有三个难点：

1. 机械臂常常**冗余**（7 个关节，6 维任务）——解有无穷多个，要选一个"最好"的；
2. 有**限位、速度限制**等约束，不能只解方程；
3. 靠近**奇异点**时，纯雅可比求逆会让关节速度爆炸。

Mink 的做法是**差分逆运动学**：每个控制周期，在当前配置附近把问题线性化，解一个**二次规划（QP）**，得到这一小步的关节速度，积分后更新，下一周期重来。它本质上是一个 **Python 库（API）**，不是 SDK，也不是仿真器；它依赖 MuJoCo 的模型来算运动学和雅可比。

### 4.2 数学形式

对第 $i$ 个任务（比如"末端位姿"），记当前误差为 $e_i(q)$（在 SE(3) 切空间里的 6 维向量），任务雅可比为 $J_i(q)$，使得 $\dot e_i \approx J_i \dot q$。想让误差朝零衰减：$J_i\,\Delta q = -\alpha\, e_i$，其中 $\Delta q = v\,\Delta t$ 是这一步的关节变化，$\alpha\in[0,1]$ 是任务增益。多个任务并存时可能互相矛盾，所以解一个加权最小二乘：

$$
\min_{\Delta q}\ \sum_i \big\| J_i\,\Delta q + \alpha_i e_i \big\|^2_{W_i} + \lambda\,\|\Delta q\|^2
\quad\text{s.t.}\quad G\,\Delta q \le h
$$

- $W_i$：任务权重（Mink 里体现为各任务的 `cost`，位置与姿态分别设）；
- $\lambda$：阻尼（正则项），避免奇异点附近解爆炸；
- $G\Delta q\le h$：不等式约束，如关节限位 $q_{\min}\le q+\Delta q\le q_{\max}$、速度限制 $|\Delta q|\le v_{\max}\Delta t$。

这是一个 QP，用 OSQP、DAQP、quadprog 之类的求解器解（Mink 通过 `qpsolvers` 调用）。

> **注意**：任务权重表达的是"冲突时的偏好"，**不代表**"位置 100% 重要、姿态 50% 重要"。位置误差单位是米、姿态误差单位是弧度，量纲不同，改权重前要先理解它们的缩放。

### 4.3 五个核心对象

| 对象 | 作用 |
|---|---|
| `mink.Configuration(model)` | 持有当前关节配置 `q`，`update(q)` 后会重新做运动学；提供 `get_transform_frame_to_world` 等 |
| `mink.FrameTask(frame_name, frame_type, position_cost, orientation_cost, lm_damping, gain)` | "让某个 frame（body / geom / site）到目标位姿"的任务；`set_target(SE3)` 设目标 |
| `mink.PostureTask(model, cost)` | "尽量保持某个参考姿态"的低权重任务，用于消除冗余自由度（防止肘部漂移） |
| Limits：`ConfigurationLimit`、`VelocityLimit`、碰撞避免等 | 以不等式约束加到 QP |
| `mink.solve_ik(configuration, tasks, dt, solver, damping, limits=...)` | 构造并求解 QP，**返回关节速度 `v`**（单位 rad/s 或 m/s） |
| `configuration.integrate_inplace(v, dt)` | 沿流形积分：$q \leftarrow q \oplus v\,dt$，对四元数等也正确 |

**`solve_ik` 返回的是速度，不是最终角度。** 一个控制周期调用一次，得到 $v$，积分一小步；这就是为什么要"每个周期重新算"：雅可比随姿态变化，一步线性化只在附近有效。

### 4.4 手算：DLS 逆运动学与奇异点处的速度放大

设平面 2R 臂在 $x$–$y$ 平面（教学示例，纯数学，$l_1=l_2=1$，非前面 2.1 节的模型），$J(q)=\begin{bmatrix}-l_1 s_1-l_2 s_{12} & -l_2 s_{12}\\ l_1c_1+l_2c_{12} & l_2c_{12}\end{bmatrix}$。

**(a) 远离奇异点**：$q=(0,\ \pi/2)$，$J=\begin{bmatrix}-1&-1\\1&0\end{bmatrix}$，$\det J=1$。想让末端沿 $+x$ 移动 $e=(0.1,0)^\top$：

$$
\Delta q=J^{-1}e=\begin{bmatrix}0&1\\-1&-1\end{bmatrix}\begin{bmatrix}0.1\\0\end{bmatrix}=\begin{bmatrix}0\\-0.1\end{bmatrix}
$$

关节只需动 0.1 rad，比值 $\|\Delta q\|/\|e\|=1$。验证：$J\Delta q=(-1\cdot0+(-1)(-0.1),\ 0)=(0.1,0)$ ✓。

**(b) 靠近奇异点（接近完全伸直）**：$q=(0,\ 0.1)$：$s_{12}=0.0998$，$c_{12}=0.995$，

$$
J=\begin{bmatrix}-0.0998&-0.0998\\1.995&0.995\end{bmatrix},\quad \det J=0.0998,\quad \sigma(J)=\{2.233,\ 0.0447\}
$$

最小奇异值只有 0.0447。想要末端沿 $x$（径向，几乎伸直了，很难再伸）走 $e=(0.01,0)^\top$：

$$
\Delta q=J^{-1}e=\begin{bmatrix}0.0997\\-0.1998\end{bmatrix},\qquad \|\Delta q\|=0.223,\ \ \frac{\|\Delta q\|}{\|e\|}=22.3
$$

**1 cm 的末端移动要求 0.22 rad 的关节运动，放大 22 倍。** 若这个运动要在一个 5 ms 周期内完成，关节速度就是 $0.223/0.005=44.6$ rad/s，远超任何电机能力——这就是"奇异点附近速度放大"。

**(c) 阻尼最小二乘（DLS）**：把求逆换成

$$
\Delta q = J^\top\big(JJ^\top+\lambda^2 I\big)^{-1}e,\qquad \lambda=0.1
$$

计算：$JJ^\top=\begin{bmatrix}0.01992&-0.2984\\-0.2984&4.970\end{bmatrix}$，加上 $\lambda^2I=0.01I$ 后行列式 $0.05996$，$(JJ^\top+\lambda^2I)^{-1}e=(0.8306,\ 0.04977)^\top$，于是

$$
\Delta q=J^\top(\cdot)=\begin{bmatrix}0.01640\\-0.03338\end{bmatrix},\quad \|\Delta q\|=0.0372;\qquad J\Delta q=\begin{bmatrix}0.00170\\-0.00050\end{bmatrix}
$$

关节运动量从 0.223 降到 0.037（约 1/6），代价是末端只实现了目标 $0.01$ 的约 17%（$0.0017$），并有 $-0.0005$ 的侧向偏差。**这就是 DLS 的取舍：用一点跟踪误差换取关节速度有界。** 离奇异点远时 $\lambda$ 相对 $J$ 的奇异值很小，影响可忽略。

> **面试要点**：能用这个例子说出 "damping trades tracking accuracy for bounded joint velocity near singularities"。

对应到 Mink：`solve_ik(..., damping=...)` 给所有关节速度加 $\lambda\|\Delta q\|^2$ 的 Tikhonov 项；`FrameTask(lm_damping=...)` 是随任务误差大小缩放的 Levenberg–Marquardt 型阻尼；`VelocityLimit` 则是**硬**上限。三者作用层次不同：前两个是"软"抑制，最后一个是"硬"截断。

### 4.5 完整示例：两连杆臂跟踪一个目标点（教学示例，已运行核对）

```python
import numpy as np
import mujoco
import mink

XML = """
<mujoco model="arm2r">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.002"/>
  <worldbody>
    <body name="link1" pos="0 0 1">
      <joint name="joint1" type="hinge" axis="0 -1 0" range="-3.1 3.1"/>
      <inertial pos="0.2 0 0" mass="1" diaginertia="0.001 0.001 0.001"/>
      <body name="link2" pos="0.4 0 0">
        <joint name="joint2" type="hinge" axis="0 -1 0" range="-3.1 3.1"/>
        <inertial pos="0.2 0 0" mass="1" diaginertia="0.001 0.001 0.001"/>
        <site name="tip" pos="0.4 0 0"/>
      </body>
    </body>
  </worldbody>
</mujoco>
"""
model = mujoco.MjModel.from_xml_string(XML)
cfg = mink.Configuration(model)                 # 持有 q，并按 q 更新运动学
cfg.update(np.array([0.2, 0.8]))                # 初始关节角 (rad)

task = mink.FrameTask(frame_name="tip", frame_type="site",
                      position_cost=1.0, orientation_cost=0.0,   # 只管位置
                      lm_damping=1e-3)
task.set_target(mink.SE3.from_translation(np.array([0.5, 0.0, 1.3])))   # 世界系目标 (m)

posture = mink.PostureTask(model, cost=1e-3)    # 低权重: 没事做时别乱晃
posture.set_target_from_configuration(cfg)

limits = [mink.ConfigurationLimit(model),
          mink.VelocityLimit(model, {"joint1": 2.0, "joint2": 2.0})]   # rad/s

dt = 0.005
for k in range(401):
    v = mink.solve_ik(cfg, [task, posture], dt, "daqp", damping=1e-3, limits=limits)
    cfg.integrate_inplace(v, dt)                # q <- integrate(q, v*dt)
    if k % 100 == 0:
        err = task.compute_error(cfg)
        print(k, "q =", np.round(cfg.q, 4), "v =", np.round(v, 4),
              "|pos err| =", round(float(np.linalg.norm(err[:3])), 5))
```

运行输出（作者核对）：

```text
0 q = [0.19 0.81] v = [-2.  2.] |pos err| = 0.16303
100 q = [-0.2137  1.5082] v = [-0.  0.] |pos err| = 0.0
200 q = [-0.2137  1.5082] v = [ 0. -0.] |pos err| = 0.0
...
```

解读：

- 第 0 步目标离得远（位置误差 0.163 m），求出的速度被 `VelocityLimit` 截断在 $\pm2$ rad/s；一个 5 ms 周期 q 变化 $2\times0.005=0.01$ rad（`0.2→0.19`，`0.8→0.81`），与输出吻合；
- 到第 100 步（0.5 s）已收敛，速度归零。收敛后 $q=(-0.2137,\ 1.5082)$ rad，可用 2R 几何验证：末端 $(0.5,\ 0.3)$（相对基座），距离 $0.583$，解析解的肘角 $\cos q_2=(0.583^2-0.32)/0.32=0.0625$，$q_2=1.508$ ✓；
- 这里 `integrate_inplace` 改的是 `cfg.q`，是 Mink 内部的配置，**并没有把它写给 MuJoCo 的 `d.qpos`/`d.ctrl`**。真正用于仿真或真机时，还要把 `cfg.q` 按执行器类型转换成 `ctrl`（位置执行器写目标角），或发给硬件。

### 4.6 与真机/仿真的循环

```text
 每个控制周期 (如几毫秒):
   1. 读当前关节角 q  ← 仿真读 d.qpos；真机读编码器 (不是读上一次的计算结果!)
   2. cfg.update(q)
   3. 更新任务目标 (来自 VR 手柄/规划器)
   4. v = solve_ik(...)          # 速度
   5. q_next = integrate(q, v*dt)
   6. 写入执行器/发给电机 (按接口: 位置目标 / 力矩 / MIT 帧)
```

**第 1 步最容易被忽略**：如果一直拿自己上一步的积分结果当"当前 q"，仿真/真机的实际姿态与它会慢慢分离（电机跟不上、被碰了、限位了），形成开环漂移。真实机器人的 IK 应当每周期从**编码器**重读 $q$。

### 4.7 Mink 常见错误

| 错误 | 现象 | 修正 |
|---|---|---|
| 把 `solve_ik` 的输出当关节角 | 关节一周期跳变巨大 | 它是速度，要乘 `dt` 积分 |
| 手写 `q += v*dt` | 含球关节/自由关节时姿态错误 | `integrate_inplace` |
| frame 名与模型不符 | 找不到 frame 的报错 | 检查 `frame_name`/`frame_type` |
| 度/弧度错（见 2.5 注意） | 限位处报无可行解 | `<compiler angle="radian"/>` |
| 不设速度限制 | 目标突变时关节速度极大 | 加 `VelocityLimit`；注意不同版本对 `limits` 缺省行为不同，不要依赖缺省 |
| 目标超出可达范围 | 求解器只能"尽量靠近" | 检查工作空间；对 VR 目标做限幅/缩放 |
| 权重量纲没理解 | 姿态把位置"拖歪" | 先单独调位置任务，再逐步加姿态 |
| QP 无解异常 | `NoSolutionFound` | 多数是约束互相矛盾（如初始状态已超限） |

---

## 5. Pinocchio

### 5.1 它是什么

Pinocchio 是一个高效的**刚体动力学算法库**（C++，带 Python 绑定），实现了 RNEA、CRBA、ABA、雅可比、质心、能量等算法，并有解析导数。常见搭配：用 Pinocchio 做模型内部的精确计算，外部做控制/优化。

### 5.2 `model` 与 `data` 的分离

| 对象 | 内容 |
|---|---|
| `model`（`pin.Model`） | 静态：关节树、连杆惯量、关节限位、`nq`、`nv`、重力向量 |
| `data`（`model.createData()`） | 动态缓冲区：算法的所有中间结果与输出 |

这个设计和 MuJoCo 的 `MjModel`/`MjData` 同构。**多线程时每个线程要有自己的 `data`**。

### 5.3 核心函数

| 函数 | 算什么 | 数学 |
|---|---|---|
| `pin.rnea(model, data, q, v, a)` | 逆动力学：返回力矩 | $\tau = M(q)a + C(q,v)v + g(q)$ |
| `pin.crba(model, data, q)` | 质量矩阵 $M(q)$（**只填上三角**） | |
| `pin.nonLinearEffects(model, data, q, v)` | $C(q,v)v+g(q)$ | |
| `pin.computeGeneralizedGravity(model, data, q)` | 重力项 $g(q)$ | |
| `pin.centerOfMass(model, data, q)` / `pin.computeCenterOfMass` | 整机质心（根坐标系下） | $c=\sum m_i c_i/\sum m_i$ |
| `pin.computeTotalMass(model)` | 总质量 | |
| `pin.jacobianCenterOfMass(model, data, q)` | 质心雅可比 | $\dot c = J_c \dot q$ |
| `pin.forwardKinematics` + `pin.updateFramePlacements` | 各关节/frame 位姿，存于 `data.oMi`、`data.oMf` | |
| `pin.computeFrameJacobian(model, data, q, frame_id, ref)` | frame 雅可比（需指定参考系，如 `LOCAL_WORLD_ALIGNED`） | |
| `pin.integrate(model, q, v*dt)` | 流形积分 | |
| `pin.neutral(model)` | 中立构型（全零，四元数为单位） | |

### 5.4 完整示例（教学示例，已运行核对）

```python
import numpy as np
import pinocchio as pin

URDF = """<?xml version="1.0"?>
<robot name="arm2r">
  <link name="base"/>
  <link name="link1">
    <inertial><origin xyz="0.2 0 0"/><mass value="1.0"/>
      <inertia ixx="0.001" iyy="0.001" izz="0.001" ixy="0" ixz="0" iyz="0"/></inertial>
  </link>
  <link name="link2">
    <inertial><origin xyz="0.2 0 0"/><mass value="1.0"/>
      <inertia ixx="0.001" iyy="0.001" izz="0.001" ixy="0" ixz="0" iyz="0"/></inertial>
  </link>
  <link name="tip"/>
  <joint name="joint1" type="revolute">
    <parent link="base"/><child link="link1"/><origin xyz="0 0 0"/>
    <axis xyz="0 -1 0"/><limit lower="-3.14" upper="3.14" effort="50" velocity="3"/>
  </joint>
  <joint name="joint2" type="revolute">
    <parent link="link1"/><child link="link2"/><origin xyz="0.4 0 0"/>
    <axis xyz="0 -1 0"/><limit lower="-3.14" upper="3.14" effort="50" velocity="3"/>
  </joint>
  <joint name="joint_tip" type="fixed">
    <parent link="link2"/><child link="tip"/><origin xyz="0.4 0 0"/>
  </joint>
</robot>
"""

model = pin.buildModelFromXML(URDF)      # 也可 pin.buildModelFromUrdf("arm2r.urdf")
data = model.createData()                # data 是缓冲区: 所有中间量放这里
print("nq, nv =", model.nq, model.nv)

q = np.array([0.0, np.pi / 2])
g = pin.computeGeneralizedGravity(model, data, q)
print("g(q) =", np.round(g, 3))          # 期望 [5.886, 0]

M = pin.crba(model, data, q)             # 只填上三角
M = np.triu(M) + np.triu(M, 1).T         # 补成对称矩阵
print("M(q) =\n", np.round(M, 4))

q = np.array([0.3, 0.5]); v = np.array([0.1, -0.2]); a = np.array([0.5, 0.2])
tau = pin.rnea(model, data, q, v, a)     # 逆动力学
b = pin.nonLinearEffects(model, data, q, v)
M = pin.crba(model, data, q); M = np.triu(M) + np.triu(M, 1).T
print("rnea =", np.round(tau, 4), " M@a+b =", np.round(M @ a + b, 4))

com = pin.centerOfMass(model, data, np.array([0.0, 0.0]))
print("CoM =", np.round(com, 3), " mass =", pin.computeTotalMass(model))
```

运行输出（作者核对）：

```text
nq, nv = 2 2
g(q) = [5.886 0.   ]
M(q) =
 [[0.242 0.041]
 [0.041 0.041]]
rnea = [7.2035 1.4311]  M@a+b = [7.2035 1.4311]
CoM = [0.4 0.  0.] mass = 2.0
```

解读：

- `g(q)` 与 `M(q)` 和 2.2、2.3 节手算一致；
- `rnea` 的结果等于 $M a + b$，验证了关系 $\tau = M(q)a + C(q,v)v + g(q)$（`nonLinearEffects` 的 $b=Cv+g$）；
- 伸直水平时质心在 $x=0.4$（两根质量相同的杆，质心在 $(0.2+0.6)/2=0.4$）✓。

### 5.5 手算：质心与重力力矩——为什么质心反馈有用

教学示例（非项目实测）：一个平衡机器人，底盘质量 $m_0=20\ \text{kg}$，质心在车体原点（前后方向 $x=0$）；两条手臂各 $m=3\ \text{kg}$，伸向前方，质心在 $x=0.15\ \text{m}$。整机总质量 $M=26\ \text{kg}$。

整机质心的前后偏移：

$$
x_c=\frac{20\times 0+3\times0.15+3\times0.15}{26}=\frac{0.9}{26}=0.0346\ \text{m}
$$

这个偏移会产生绕俯仰轴的重力力矩（相对车体原点）：

$$
\tau_{\text{pitch}}\approx M g\,x_c=26\times 9.81\times0.0346\approx 8.83\ \text{N·m}
$$

若不补偿，姿态控制器要靠积分项或偏差慢慢"顶住"这个力矩；如果手臂摆动，$x_c$ 随之变化，这个力矩就是个时变扰动。**典型做法**：用 URDF + Pinocchio，每个周期根据手臂关节角 $q_{\text{arm}}$ 算出 $x_c(q_{\text{arm}})$（以及侧向偏移 $y_c$，对应 roll），乘上总质量和重力，换算成俯仰/横滚方向的前馈量，叠加到平衡控制器的输出。这样手臂动作带来的重心变化被**前馈**抵消，反馈环只处理剩余误差。更多关于前馈的概念见 [前馈、增益调度、限幅与饱和](../04_控制理论/07_前馈_增益调度_限幅与饱和.md)。

若整机质心高度 $h=0.5\ \text{m}$，平衡点相对垂直的倾角约 $\theta_{eq}\approx\arctan(x_c/h)=\arctan(0.0346/0.5)=0.0691\ \text{rad}\approx3.96^\circ$（示例）。这说明：不考虑质心偏移，"平衡点"本身就不在 0°。

### 5.6 浮动基与四元数顺序

固定基机械臂：`buildModelFromUrdf(path)`。浮动基机器人（轮腿、人形、ballbot 的整机）：

```python
mf = pin.buildModelFromUrdf("robot.urdf", pin.JointModelFreeFlyer())
# mf.nq = 7 + 关节数,  mf.nv = 6 + 关节数
```

我们的两连杆臂加自由基后：`nq=9, nv=8`，`pin.neutral(mf)` 的前 7 个数是 `[0,0,0, 0,0,0,1]`，即**四元数 (x, y, z, w) 的 w 在最后**。MuJoCo 则是 `(w, x, y, z)`。两库之间转换自由基位姿：

```python
# MuJoCo qpos[0:7] = [px, py, pz, qw, qx, qy, qz]  ->  Pinocchio q[0:7] = [px, py, pz, qx, qy, qz, qw]
q_pin_base = np.r_[q_mj[0:3], q_mj[4:7], q_mj[3]]
```

### 5.7 Pinocchio 常见错误

| 错误 | 现象 | 修正 |
|---|---|---|
| 直接用 `crba` 返回值当对称矩阵 | 下三角全是 0，计算结果错 | 用 `np.triu(M)+np.triu(M,1).T` 补全 |
| 关节顺序假设成 URDF 文件顺序 | 力矩给错关节 | 用 `model.names`、`model.joints` 按名字建立映射 |
| 四元数顺序 | 姿态翻转 | Pinocchio 为 xyzw，MuJoCo 为 wxyz |
| 在 `data` 上并发 | 多线程结果错乱 | 每线程一个 `data` |
| 惯量/质心不合理 | 重力项、质心全错 | 检查 URDF 的 `<inertial>`（质量、`origin`、惯量矩阵必须物理有效） |
| 忘记算 FK 就读 `data.oMf` | 读到旧值 | 先 `forwardKinematics` 再 `updateFramePlacements` |
| `ReferenceFrame` 选错 | 雅可比的线速度/角速度参考系不对 | 明确用 `LOCAL` / `WORLD` / `LOCAL_WORLD_ALIGNED` |

---

## 6. KDL

### 6.1 它是什么

KDL（Kinematics and Dynamics Library）是 Orocos 项目里的运动学/动力学库，**ROS 生态里非常常用**：`kdl_parser` 可以把 URDF 直接转成 KDL 的树结构；MoveIt 等也有 KDL 求解器插件。它偏向**串联链**的计算。

### 6.2 概念

| 概念 | 含义 |
|---|---|
| `Frame` | 位姿（旋转 + 平移） |
| `Joint` | 关节（旋转/平移/固定） |
| `Segment` | 一段连杆 + 其关节 + 惯量 |
| `Chain` | **串联链**：从一个 link 到另一个 link 的一串 Segment |
| `Tree` | 整棵树（含所有分支） |
| `JntArray` | 关节向量（位置/速度/力矩） |
| `ChainFkSolverPos_recursive` | 正运动学 |
| `ChainJntToJacSolver` | 雅可比 |
| `ChainDynParam` | 动力学参数：`JntToMass`（质量矩阵）、`JntToCoriolis`、`JntToGravity` |
| `ChainIdSolver_RNE` | 递归牛顿—欧拉逆动力学 |

### 6.3 典型流程：URDF → Tree → Chain → `JntToGravity`

```text
  URDF 文件
    │ kdl_parser::treeFromFile / treeFromString
    ▼
  KDL::Tree (整个机器人)
    │ tree.getChain("base_link", "tool_link", chain)
    ▼
  KDL::Chain (只含这条串联链上的 Segment)
    │ KDL::ChainDynParam dyn(chain, gravity_vector_in_base_frame)
    ▼
  dyn.JntToGravity(q, g)   →  g[i] = 保持静止需要的各关节力矩
```

C++ 结构示意（教学示例，**未编译核对**）：

```cpp
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jntarray.hpp>
#include <kdl_parser/kdl_parser.hpp>

int main() {
  KDL::Tree tree;
  if (!kdl_parser::treeFromFile("arm2r.urdf", tree)) return 1;

  KDL::Chain chain;
  if (!tree.getChain("base", "tip", chain)) return 2;      // 起止 link 名要对
  const unsigned n = chain.getNrOfJoints();                // 期望 2 (固定关节不计)

  // 重力向量: 在 chain 根坐标系下表达
  KDL::ChainDynParam dyn(chain, KDL::Vector(0.0, 0.0, -9.81));

  KDL::JntArray q(n), g(n);
  q(0) = 0.0; q(1) = M_PI / 2;
  dyn.JntToGravity(q, g);                                  // 期望 g = [5.886, 0]

  KDL::ChainFkSolverPos_recursive fk(chain);
  KDL::Frame tip;
  fk.JntToCart(q, tip);                                    // tip.p 是末端位置

  KDL::ChainJntToJacSolver jac(chain);
  KDL::Jacobian J(n);
  jac.JntToJac(q, J);                                      // 6 x n
  return 0;
}
```

按 2.2 节的手算，期望 `g = [5.886, 0]`、末端位置 $x=0.4$、$z=0.4$（相对基座）。实际使用时请自己运行对照，这是区别"教学示例"和"已核对结果"的方法。

### 6.4 KDL 的四个典型坑

**坑 1：关节顺序。** KDL `Chain` 里的关节顺序是**沿链的顺序**（只数非固定关节），既不是 URDF 文件里的声明顺序，也不一定是电机 ID 顺序，也不是字母序。你必须自己建立"电机 ID ↔ 关节名 ↔ KDL 索引"的映射表。顺序错位的后果：补偿力矩给到了别的关节，现象是"这个关节越托越重，那个关节又在乱动"。

**坑 2：链的端点与末端负载。** `ChainDynParam` 只计算**这条链上** Segment 的质量。如果夹爪、手柄、工具在链之外（比如是分支上的 link），它们的重量**不会**被算入。主从示教里 Leader 臂手柄的重量如果漏算，补偿就不足；链端点要选到真正承载全部负载的末端 link，必要时把工具质量写进最后一个 link 的惯量。

**坑 3：重力向量要在基座系下表达。** 构造 `ChainDynParam` 时的 `gravity` 是 **链根坐标系**里的重力向量。机械臂水平安装在地面时是 $(0,0,-9.81)$；如果基座是倾斜安装的，或者基座本身会倾斜（比如装在会摆动的平台上），则要用 $R_{\text{base}}^{\text{world}\ \top}\,[0,0,-9.81]^\top$ 把世界系的重力变换到基座系，否则重力补偿方向错误。这个思想和前面 5.5 节"质心偏移 + 倾角"是同一件事的两面。

**坑 4：URDF 数据质量。** KDL 完全依赖 URDF 的质量、质心、惯量。质量漏填、质心用的是几何中心而不是真实质心、关节零位与实物不一致，都会让补偿不准。另外 KDL 的 URDF 解析器对"根 link 带惯量"会给出警告（KDL 不支持根 link 带惯量），根 link 一般用空的基座 link。

### 6.5 KDL 在主从示教里的位置

以"Leader 臂重力补偿"为例（CV 描述：*built dual-arm dynamics from URDF/KDL, designed leader-arm gravity compensation*）：

1. 离线：读 URDF → KDL Tree → 取从基座到末端的 Chain → 构造 `ChainDynParam`；
2. 在线，每个控制周期：读取 Leader 当前关节角 $q$（编码器）→ `JntToGravity(q, g)` → 把 $g$ 换算成各电机的力矩命令（需要乘减速比、换算为电流/力矩指令单位——换算关系看电机协议，不能只凭变量名断言）；
3. 力矩模式下电机输出 $\tau_{\text{cmd}}=g(q)$，**抵消重力**；此时人用很小的力就能推动手臂，因为他只需克服摩擦、惯性和模型误差。

用户常问的两个问题在这里回答：

- **"为什么人可以直接推动电机？电机不是在给力吗？"**：重力补偿时电机处于**力矩（电流）控制**模式，输出的是一个与位置无关的"托住手臂重量"的力矩，而不是"保持在某个位置"的位置闭环。位置闭环才会抵抗你的推动；力矩开环（或弱阻尼）不会。
- **"手臂被推动之后重力补偿不是一直在变吗？怎么一直算？"**：对，$g(q)$ 随姿态变化。所以每个控制周期都重新读 $q$、重新算 $g(q)$ 并更新命令。KDL 的 `JntToGravity` 计算量很小（对 6~7 个关节是毫秒以下量级，具体耗时需实测），可以在几百赫兹的循环里在线算。

更深的推导见 [机械臂动力学、重力补偿、URDF、KDL](../03_坐标变换与运动学/06_机械臂动力学_重力补偿_URDF_KDL.md)。

### 6.6 KDL 的 Python 版（PyKDL）

ROS 环境里常用 `PyKDL` 与 `kdl_parser_py`，接口与 C++ 同名（`Chain`、`ChainDynParam`、`JntArray`、`JntToGravity`）。注意 PyKDL 的 `JntArray` 要用 `q[i] = ...` 赋值、没有 NumPy 自动转换，常要手写一个小转换函数。

---

## 7. 四者区别与选型

### 7.1 对比表

| 维度 | MuJoCo | Mink | Pinocchio | KDL |
|---|---|---|---|---|
| 定位 | 仿真器 + 运动学/动力学库 | 基于 MuJoCo 的逆运动学求解库 | 刚体动力学算法库 | 串联链运动学/动力学库 |
| 模型格式 | MJCF（亦可 URDF） | 使用 MuJoCo 的 `MjModel` | URDF（亦支持 MJCF 等） | URDF（经 kdl_parser） |
| 主要语言 | C，Python 绑定 | Python | C++，Python 绑定 | C++，PyKDL |
| 接触/碰撞仿真 | **强** | 不负责 | 有碰撞检测（可选），不做接触仿真 | 无 |
| 逆运动学 | 无内置（需自己搭） | **核心功能**（QP、限位、多任务） | 需自己搭（有例子） | 有 IK 求解器（牛顿—拉夫逊、LMA 等） |
| 动力学算法 | 正/逆动力学 | 不负责 | **RNEA、CRBA、ABA、解析导数** | `ChainDynParam`、RNE |
| 浮动基 | 支持 | 通过 MuJoCo | **一等公民** | 弱（链为主） |
| 质心 | `subtree_com` 等 | — | `centerOfMass` 及雅可比 | 需自己累加 |
| ROS 集成 | 一般 | 一般 | 一般（有 ROS 包） | **很好** |
| 典型用途 | 仿真、RL、遥操作可视化 | 遥操作 IK、末端跟踪 | 平衡/全身控制、重力补偿、质心、MPC/最优控制 | ROS 里的机械臂重力补偿、MoveIt 插件 |

### 7.2 怎么选（一个决策流程）

1. 需要**物理仿真**（接触、摩擦、碰撞、看画面）→ MuJoCo；
2. 需要**末端目标 → 关节小步**，带限位/速度限制/多任务 → Mink（或自搭 Pinocchio + QP）；
3. 需要**浮动基/整机动力学/质心/解析导数** → Pinocchio；
4. 机械臂是**固定基串联链**，运行在 ROS 里，只需要重力补偿 → KDL 足够，也很省心；
5. 想要**交叉验证**：同一个 URDF 分别用 Pinocchio 和 KDL 算 $g(q)$，结果应一致——不一致就要查 URDF 解析、关节顺序、重力向量。

### 7.3 组合使用的常见方式

- **MuJoCo + Mink**：遥操作 / 仿真验证：Mink 用 MuJoCo 的模型求关节小步，MuJoCo 仿真或渲染（这正是 P2 VR 遥操作的典型结构）；
- **URDF + Pinocchio**：整机质心、动力学项供控制器用（P1 的质心前馈是典型例子）；
- **URDF + KDL**：ROS 环境里的重力补偿（P2 主从示教是典型例子）。

> **面试要点**：能说出 "We used the same URDF as the single source of truth for several tools; therefore I validate the model by comparing the gravity vector from two libraries." 之类的话，体现你有交叉验证意识（前提：确实做过；没做过只当作"我会怎么做"来讲）。

---

## 8. 在本项目里怎么用

### 8.1 P1：URDF / Pinocchio 质心前馈

[P1 项目文档](../../01_项目课程/P1_AIRS球形机器人平衡控制/README.md)。CV：*estimated CoM offset from arm joint states with URDF/Pinocchio, fed roll/pitch gravity feedforward into the balance controller and verified its effect by toggling compensation under fixed arm trajectories.*

对照本章：

- 5.2–5.5 节：URDF 加载、`computeCenterOfMass`、总质量；
- 5.5 节手算：质心偏移 → 重力力矩 → 俯仰/横滚前馈；
- "verified by toggling compensation"：这是标准的对照实验（开/关前馈，固定手臂轨迹，比较姿态误差或控制量）——面试官喜欢问**你怎么证明它有效**；
- 【待你补全：URDF 的质量/质心数据来源（CAD 导出还是估算）；质心偏移到前馈量的换算公式与符号约定；前馈量是否限幅；开/关前馈对比的指标和幅度（只讲你记得的，没有就如实说）】

### 8.2 P2：VR 遥操作（MuJoCo/Mink）与主从示教（URDF/KDL）

[P2 项目文档](../../01_项目课程/P2_未来动力双臂遥操作与差速底盘/README.md)。CV：

- *mapped VR controller motion to end-effector targets and solved constrained inverse kinematics with MuJoCo/Mink, achieving end-effector tracking while suppressing velocity amplification near singularities.* → 4.2–4.5 节：任务、约束、DLS 阻尼、速度限制；
- *built dual-arm dynamics from URDF/KDL, designed leader-arm gravity compensation and follower position–velocity tracking, enabling low-resistance hand-guided teaching with synchronous follower motion.* → 6.3–6.5 节与 3.7 节演示（`ctrl = g(q)`）。

面试时建议先画 4.6 节的循环图，再讲：VR 手柄位姿如何变成末端目标（坐标系与相对位姿的建立见 [坐标系、旋转矩阵、齐次变换](../03_坐标变换与运动学/01_坐标系_旋转矩阵_齐次变换.md)）；IK 如何求；速度如何被限制；输出怎样接到 ROS2-CAN 链（见 [ROS1 / ROS2 核心概念](./04_ROS1_ROS2核心概念.md) 第 13 章）。

- 【待你补全：IK 任务的具体设置（只管位置还是位置 + 姿态、是否有 posture 任务）；用了哪些限制（限位、速度）；控制周期；输出是位置目标还是其它；"velocity amplification near singularities" 具体是怎样抑制的（阻尼系数、限速、还是别的）】
- 【待你补全：Leader 臂的链端点是什么（含不含手柄/夹爪）；关节顺序映射表；重力向量如何给；补偿后的残余摩擦如何处理；Follower 的位置—速度跟踪增益来源】

---

## 9. 常见错误与故障现象（汇总）

| 现象 | 最可能原因 | 查什么 |
|---|---|---|
| 仿真机械臂一开始就乱甩 | `ctrl` 语义用错（motor 当 position 用） | 看 MJCF 的 `<actuator>` |
| 读到的末端位置不更新 | 改了 `qpos` 没 `mj_forward` | 调用顺序 |
| Mink 报"无可行解" | 初始 q 已超限（含度/弧度单位错）；约束互相矛盾 | `<compiler angle>`；限位范围 |
| 末端接近伸直时关节速度极大 | 奇异点；阻尼过小；无速度限制 | `damping`、`lm_damping`、`VelocityLimit` |
| 重力补偿某个关节"越托越重" | 符号反；关节顺序错位；电机力矩常数/减速比错 | 先让一个关节单独补偿并核对符号 |
| 补偿后仍有明显下坠或上飘 | URDF 质量/质心错；链外负载未算；重力向量没转换到基座系 | 6.4 节四个坑 |
| 两个库算的 $g(q)$ 不一致 | URDF 解析差异；根 link 惯量；关节顺序；轴方向 | 在 3 个姿态上逐一对比 |
| 浮动基姿态算反 | 四元数顺序（xyzw vs wxyz） | 5.6 节 |
| 真机末端和仿真偏离越来越大 | IK 一直用自己积分的 $q$，没读编码器 | 4.6 节第 1 步 |

---

## 10. 面试怎么问、怎么答

**Q1. What is the difference between `qpos` and `ctrl` in MuJoCo?**

> `qpos` is the actual generalized position state of the simulated joints, and `ctrl` is the input to the actuators. What `ctrl` means depends on the actuator type. For a motor actuator it is a torque, for a position actuator it is a target angle. So writing joint angles into a motor's `ctrl` does not set the joint angles. After each step I read the new `qpos` and recompute.

中文：`qpos` 是仿真关节的实际位置状态，`ctrl` 是执行器输入；其含义取决于执行器类型（motor 是力矩，position 是目标角）；往 motor 的 `ctrl` 里写关节角不等于设置关节角；每步读新的 `qpos` 再算。
追问：`mj_forward` 和 `mj_step` 区别？（前者只算派生量不推进时间，后者还积分并增加 time。）

**Q2. Why are `nq` and `nv` different in MuJoCo or Pinocchio?**

> Because orientation is stored as a quaternion with four numbers, but the velocity of an orientation only has three degrees of freedom. A free joint has seven position values and six velocity values. That is why we cannot simply add `qvel * dt` to `qpos`; we need an integration function that works on the manifold.

中文：姿态用 4 个数的四元数存，而姿态速度只有 3 个自由度；自由关节 7 个位置、6 个速度；因此不能 `qpos += qvel*dt`，要用流形积分函数。

**Q3. Explain how you solved constrained inverse kinematics with MuJoCo and Mink, and how you handled singularities.**（对应 P2）

> The VR controller gives a target pose for the end effector. In each control cycle, Mink linearizes the problem around the current joint configuration and solves a small quadratic program for the joint velocity. The QP includes the end-effector task, constraints like joint limits and velocity limits, and a damping term. Near a singularity the plain inverse of the Jacobian would amplify the joint velocity, so damping and velocity limits keep the joint motion bounded at the cost of some tracking error. Then I integrate the velocity to get the next joint target.

中文：VR 手柄给末端目标位姿；每个控制周期 Mink 在当前配置附近线性化，解一个小 QP 得关节速度；QP 里有末端任务、限位/速度等约束和阻尼项；奇异点附近纯求逆会放大关节速度，阻尼和速度限制以一点跟踪误差换取有界速度；再积分得到下一步目标。
追问：为什么每周期都要重算？（雅可比随姿态变，线性化只在附近有效。）你的阻尼取多少？（【待你补全具体值；记不清时说明取值是调出来的，以及怎么调：看奇异附近关节速度峰值和跟踪误差。】）

**Q4. How does gravity compensation work for the leader arm in teaching?**（对应 P2）

> I build the arm model from the URDF with KDL, take the chain from the base to the end effector, and in each control cycle I read the joint angles, compute the gravity torque vector with `JntToGravity`, and command that torque to the motors in torque mode. The motors then carry the weight of the arm, so the operator only has to overcome friction and inertia. The gravity torque changes with the pose, so it is recomputed every cycle. The follower receives the leader's joint references and tracks position and velocity.

中文：用 URDF 建 KDL 模型，取基座到末端的链；每个周期读关节角，`JntToGravity` 算重力力矩，在力矩模式下发给电机；电机托住手臂重量，人只需克服摩擦和惯性；姿态变所以每周期重算；Follower 接收关节参考做位置—速度跟踪。
追问：模型不准会怎样？（补偿过度会自己飘，不足会下坠；质量质心、关节顺序、重力向量方向是主要来源。）

**Q5. How did you use Pinocchio in the balance controller?**（对应 P1）

> I used the robot's URDF and the arm joint states to compute the center of mass of the whole robot with Pinocchio. When the arms move, the center of mass shifts, which creates a gravity torque on roll and pitch. I converted that offset into a feedforward term for the balance controller, and to verify it, I toggled the compensation while the arms followed fixed trajectories and compared the result.

中文：用 URDF 和手臂关节状态通过 Pinocchio 算整机质心；手臂动时质心偏移产生俯仰/横滚重力力矩；换算成前馈加到平衡控制器；验证方式是在固定手臂轨迹下开关补偿并对比。
注意：只陈述 CV 里有的事实；具体的转换公式、数据、改善幅度请用你自己的记忆补全，记不清时："I don't have the exact figure in front of me, but the pitch deviation was clearly smaller with compensation on."（仅在确实如此时使用。）

**Q6. MuJoCo, Pinocchio, KDL: when would you use each?**

> MuJoCo when I need a physics simulation with contacts or a quick model for visualization. Pinocchio when I need efficient dynamics for a floating-base robot, like the center of mass or inverse dynamics. KDL when I'm in a ROS environment with a serial arm and just need kinematics or gravity compensation. I can cross-check any of them against another using the same URDF.

中文：要带接触的仿真或可视化用 MuJoCo；浮动基机器人的高效动力学（质心、逆动力学）用 Pinocchio；ROS 里串联臂的运动学或重力补偿用 KDL；同一份 URDF 可交叉验证。

---

## 11. 自测题

1. 一个模型有一个自由关节和 6 个铰链，`nq`、`nv` 各是多少？为什么？
   *要点：`nq=7+6=13`，`nv=6+6=12`；自由关节四元数 4 个数、角速度 3 个。*
2. 把 `d.qpos` 改成新值后立刻读 `d.site_xpos`，会得到什么？怎么修？
   *要点：旧值；调用 `mj_forward`。*
3. 手算：两连杆（2.1 节参数）在 $q=(\pi/2,\ 0)$（竖直向上）时 $g(q)$ 是多少？
   *要点：$\cos q_1=0$ 且 $\cos(q_1+q_2)=0$，$g=(0,0)$，重力过轴。*
4. 为什么 DLS 能抑制奇异点附近的速度放大？它的代价是什么？
   *要点：$J^\top(JJ^\top+\lambda^2I)^{-1}$ 把小奇异值 $\sigma$ 的增益从 $1/\sigma$ 限制为 $\sigma/(\sigma^2+\lambda^2)$；代价是该方向跟踪误差。*
5. `solve_ik` 返回什么？为什么不能直接当关节角发给电机？
   *要点：关节速度；要乘 dt 积分、读编码器闭环、按执行器接口转换。*
6. KDL 的 `ChainDynParam` 为什么可能漏算夹爪重量？
   *要点：只算链上 Segment；链端点没包含工具 link 或其惯量。*
7. 机械臂装在会前后倾斜的平台上，KDL 的重力向量怎么给？
   *要点：把世界系重力向量变换到基座系，$R^\top[0,0,-g]^\top$。*
8. MuJoCo 与 Pinocchio 的四元数顺序是什么？写一行转换。
   *要点：wxyz 与 xyzw；`np.r_[p, q[1:4], q[0]]`。*

---

## 12. 本章要点速记与相关链接

**要点速记**

1. 库负责算，SDK 负责连真机，仿真器负责"演"；MuJoCo 既是仿真器也是运动学/动力学库。
2. `MjModel` 静态、`MjData` 动态；`qpos/qvel/ctrl` 是状态与输入，`nq` 与 `nv` 在有四元数时不相等。
3. 改完状态必须 `mj_forward` 才更新派生量；`mj_step` 才推进时间。
4. `ctrl` 的语义由执行器类型决定：motor 是力矩，position 是目标角。MJCF 默认角度单位是度。
5. Mink = 差分 IK：每周期线性化、解 QP、返回速度、积分；输出是速度不是角度。
6. DLS 用跟踪误差换有界关节速度；`VelocityLimit` 是硬截断；任务权重是偏好而非百分比。
7. Pinocchio：`model`/`data` 分离；`crba` 只填上三角；`rnea` = $Ma+Cv+g$；质心与浮动基是强项；四元数 xyzw。
8. KDL：URDF → Tree → Chain → `ChainDynParam::JntToGravity`；注意关节顺序、链端点、负载、基座系重力向量。
9. 重力补偿 = 力矩模式下输出 $g(q)$，每周期随姿态重算；人只需克服摩擦、惯性和模型误差。
10. 同一份 URDF 用两个库交叉验证 $g(q)$，是排查建模错误的好办法。

**相关链接**

- 上一章：[DDS 与分布式通信](./05_DDS与分布式通信.md)
- 下一章：[URDF、TF 与坐标系实践](./07_URDF_TF_坐标系实践.md)
- 基础：[正逆运动学与雅可比](../03_坐标变换与运动学/03_正逆运动学与雅可比.md) · [奇异性、阻尼最小二乘、零空间](../03_坐标变换与运动学/04_奇异性_阻尼最小二乘_零空间.md) · [机械臂动力学、重力补偿、URDF、KDL](../03_坐标变换与运动学/06_机械臂动力学_重力补偿_URDF_KDL.md) · [最优化与二次规划](../01_数学/04_最优化与二次规划.md) · [前馈、增益调度、限幅与饱和](../04_控制理论/07_前馈_增益调度_限幅与饱和.md)
- 项目：[P1](../../01_项目课程/P1_AIRS球形机器人平衡控制/README.md) · [P2](../../01_项目课程/P2_未来动力双臂遥操作与差速底盘/README.md)
- 学习笔记原始问答（了解自己哪里容易卡）：[P2 问答逐字稿](../../05_学习笔记归档/P2/P2_本对话问答逐字稿.md)、[P2 问答逐字稿（第二份）](../../05_学习笔记归档/P2/P2_本对话问答逐字稿_第二份.md)
