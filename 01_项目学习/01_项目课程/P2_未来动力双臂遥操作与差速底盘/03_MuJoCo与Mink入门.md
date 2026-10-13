# 03 MuJoCo 与 Mink 入门

> **一句话定位**：上一章得到了每只手的末端目标 $T_d$（4×4 矩阵）。这一章回答"MuJoCo 和 Mink 各是什么、怎么配合，才能把 $T_d$ 变成每个关节下一小步该怎么走"——包括模型、`qpos/qvel/ctrl`、`Configuration`、`Task`、QP、`dt`、积分，以及一整轮 `solve_step` 的逐行讲解。学习笔记里你问得最多的就是这一章：**"为什么要用 MuJoCo？直接把控制量发给电机不行吗？"** 先给答案：行，但电机只懂"关节命令"，不懂"手去哪里"；MuJoCo 提供机械臂的数字模型，Mink 在这个模型上把"手去哪里"翻译成"关节怎么动"。

**本章学完你能：**

- 说清 MuJoCo 的 `MjModel`、`MjData`、`qpos`、`qvel`、`ctrl`、`nq` 与 `nv` 分别是什么，以及 `mj_forward` 与 `mj_step` 的区别；
- 说清 Mink 的 `Configuration`、`FrameTask`、`PostureTask`、`Limit`、`solve_ik`、`integrate_inplace` 各负责什么；
- 把"差分逆运动学 = 每步一个小 QP"这件事用一个二连杆数字例子完整算一遍；
- 默写一个 `solve_step` 的骨架，并解释每一行；
- 解释 `ctrl` 不一定是关节角，以及"IK 结果怎么变成电机命令"的三种做法；
- 解释 5 ms 的 `dt` 为什么不等于"程序真的 200 Hz"。

**前置知识：**

- 上一章 [02 VR 遥操作的相对位姿映射](./02_VR遥操作的相对位姿映射.md)（得到 $T_d$）
- 雅可比与正逆运动学：[03_正逆运动学与雅可比](../../02_基础知识/03_坐标变换与运动学/03_正逆运动学与雅可比.md)
- 二次规划：[04_最优化与二次规划](../../02_基础知识/01_数学/04_最优化与二次规划.md)
- 工具链：[06_MuJoCo_Pinocchio_KDL_工具链](../../02_基础知识/09_机器人软件/06_MuJoCo_Pinocchio_KDL_工具链.md)

## 目录

1. 先回答那个问题：为什么要用 MuJoCo
2. MuJoCo 是什么：模型、状态、控制量
3. `qpos`、`qvel`、`ctrl`、`nq` 与 `nv`
4. `mj_forward`、`mj_step` 与"算几何"和"推进仿真"
5. Mink 是什么：站在 MuJoCo 上的逆运动学求解库
6. Task：把"想要什么"写成误差
7. QP：把多个任务和约束放进一个优化问题
8. `dt` 与积分：小步走
9. 一整轮 `solve_step` 逐行讲解
10. 一个二连杆的完整数字例子
11. IK 结果怎么变成"电机命令"：三种做法
12. 项目里的配置（来自学习笔记）与口径
13. 常见错误
14. 面试怎么问、怎么答
15. 本章要点速记 / 自测题 / 相关链接

---

## 1. 先回答那个问题：为什么要用 MuJoCo

你在学习笔记第 5 组问过：**"直接把控制量发给关节电机不就可以了吗？"**

答案要拆成两半：

1. **电机只懂关节命令。** 一个电机只管一个关节，它需要的是"这个关节的目标角度/速度/力矩"。它无法理解"让手向前 5 厘米并转 30 度"。
2. **人给的是手的位姿。** VR 给出的是末端目标 $T_d$。一只 7 自由度机械臂的末端位姿由 7 个关节共同决定，要得到"每个电机该怎么动"，就必须经过一次**逆运动学（IK）**。

IK 需要一个**机器人模型**：每段连杆多长、关节怎么连、轴向是什么。MuJoCo 就是在这里出场：它加载机器人的描述文件，维护关节状态，并能快速算出"某个 frame 现在在哪里""关节动一点它怎么动"（雅可比）。Mink 则是站在 MuJoCo 上的 IK 求解库。

```text
手柄: 手要去哪里? (位置+朝向)          ← 02 章
          │
          ▼
IK: 各关节下一小步怎么动?              ← Mink(借助 MuJoCo 模型)
          │
          ▼
关节目标: 每个电机收到什么命令?
     ┌────┴────────────┐
     ▼                 ▼
MuJoCo 更新/显示     ROS 2 → CAN 命令真实电机
虚拟机械臂 (离线调试)   (第 10 章)
```

所以：

- **真机不一定需要 MuJoCo 来"动"**——真机需要的是关节命令。MuJoCo 在这条链里提供**模型**（给 Mink 算 FK 和雅可比）以及**离线可视化/调试**。
- 如果你有别的可靠模型库（如 KDL、Pinocchio），也可以替代 MuJoCo 提供运动学；选 MuJoCo 的实际好处是：Mink 直接建立在它上面，模型文件可同时用于仿真显示。

> **面试要点**：不要把 MuJoCo 说成"逆运动学算法"。MuJoCo 是物理仿真/模型库；IK 的求解是 Mink 做的。一句话：**MuJoCo 负责"现在是什么样"，Mink 负责"为了到目标下一步怎么动"。**

---

## 2. MuJoCo 是什么：模型、状态、控制量

MuJoCo（Multi-Joint dynamics with Contact）是一个开源的刚体动力学/物理仿真引擎，Python 里 `import mujoco`。要记住三个对象：

| 对象 | 含义 | 类比 |
|---|---|---|
| `MjModel`（模型） | 机器人"是什么"：连杆、关节、执行器、质量惯量、几何、重力、仿真步长 | 机器人的说明书，加载后基本不变 |
| `MjData`（数据/状态） | 机器人"现在怎样"：关节位置速度、控制输入、各 body/site 的世界位姿等 | 机器人的当前快照，每步都变 |
| 函数（`mj_forward`、`mj_step`…） | 根据模型和数据做计算 | 计算器 |

### 2.1 模型文件里有什么

模型通常是 MJCF（MuJoCo 自己的 XML）或 URDF。里面会有：

- `body`：刚体（连杆），有质量、质心、转动惯量；
- `joint`：关节（`hinge` 转动、`slide` 滑动、`ball` 球关节、`free` 自由关节），有轴向、限位；
- `geom`：几何体（碰撞/显示）；
- `site`：一个附着在 body 上的"标记点"，没有质量，常用来标记末端 TCP、传感器位置；
- `actuator`：执行器（`motor` 力矩型、`position` 位置伺服型、`velocity` 速度型…）；
- `option`：重力、仿真步长 `timestep` 等。

学习笔记里项目的 IK 求解器把重力设为零：`model.opt.gravity[:] = 0`。原因是 IK 只用到**运动学**（几何），不需要动力学；真机的重力补偿由另一条线（主臂的 KDL 重力补偿，[06 章](./06_URDF_KDL动力学与重力补偿.md)）负责。不要把"IK 仿真里重力为零"说成"我们仿真了真实动力学"。

### 2.2 一个直觉：把 MuJoCo 想成"虚拟机器人实验室"

- 给它一组关节角（`qpos`），它能告诉你每个连杆、每个 site 在世界里的位置和朝向（正运动学）；
- 给它一个 site 和当前关节角，它能算出**雅可比矩阵**（关节动一点，这个点怎么动）；
- 给它执行器的控制量（`ctrl`），再调用 `mj_step`，它推进物理，计算受力后的运动。

**不开动画窗口 MuJoCo 照样算**——窗口只是把结果画出来。这点你在笔记里确认过，面试可以直接说。

---

## 3. `qpos`、`qvel`、`ctrl`、`nq` 与 `nv`

这三个数组最容易混，也是学习笔记里反复强调的：

| 数组 | 含义 | 单位 | 长度 |
|---|---|---|---|
| `data.qpos` | 广义位置：每个关节的位置 | 转动关节 rad，滑动关节 m，自由关节含位置+四元数 | `nq` |
| `data.qvel` | 广义速度 | rad/s、m/s | `nv` |
| `data.ctrl` | 执行器输入，**含义由执行器类型决定** | 可能是力矩、目标位置、目标速度 | `nu`（执行器个数） |

### 3.1 `nq` 与 `nv` 为什么可能不相等

纯转动关节：1 个关节 → 1 个位置、1 个速度，`nq = nv`。

但自由关节（`free`）的位置是 7 个数（3 位置 + 4 四元数），速度只有 6 个数（3 线速度 + 3 角速度）；球关节位置 4 个数（四元数），速度 3 个数。因为四元数有 4 个分量却只有 3 个自由度。结果：

$$
nq\ne nv\quad(\text{含自由/球关节时}).
$$

后果：**不能把速度直接加到位置上**（长度都不一样，且四元数相加会破坏单位长度）。必须用"积分器"：`mujoco.mj_integratePos` 或 Mink 的 `integrate_inplace`，它们按关节类型正确处理。这就是笔记第 6 组里说"`configuration.update` 负责用正确的方式更新状态"的原因。

对一只纯 7 转动关节的臂，`nq = nv = 7`，双臂 14。

### 3.2 `ctrl` 不一定是关节角

笔记里反复强调的一个坑：`mj_data.ctrl` 是**执行器输入**，它是不是"关节角"取决于 XML 里执行器怎么定义：

| 执行器类型 | `ctrl` 的含义 | 产生的力 |
|---|---|---|
| `motor` | 力/力矩（乘以 `gear` 比例） | $\tau=\text{gear}\cdot\text{ctrl}$ |
| `position` | 目标位置 | $\tau=k_p(\text{ctrl}-q)-k_v\dot q$（在模型里定义 $k_p,k_v$） |
| `velocity` | 目标速度 | $\tau=k_v(\text{ctrl}-\dot q)$ |

所以"把关节角数组写进 `ctrl`"只有在执行器是 `position` 型时才合理；如果是 `motor` 型，那个数组会被当成力矩，结果完全不是你想要的。学习笔记里特别提醒过要核对这一点。**写 `ctrl` 之前一定要看 XML 里对应 `<actuator>` 的类型。**

---

## 4. `mj_forward`、`mj_step` 与"算几何"和"推进仿真"

| 函数 | 做什么 | 会推进时间吗 |
|---|---|---|
| `mj_forward(model, data)` | 由当前 `qpos/qvel/ctrl` 计算所有派生量：各 body/site 的世界位姿、雅可比、受力、加速度 | **不推进** |
| `mj_step(model, data)` | 先做 `forward`，再用加速度积分，把时间推进一个 `model.opt.timestep` | **推进** |
| `mj_kinematics` | 只算运动学（位置/朝向） | 不推进 |
| `mj_jacSite / mj_jacBody` | 算某个 site/body 的 3×nv 平移雅可比和 3×nv 转动雅可比 | 不推进 |

对 IK 来说，只需要"算几何"：设定 `qpos` → `mj_forward`/`mj_kinematics` → 读 site 的位姿和雅可比，不需要推进物理。这就是学习笔记里说的"算几何"和"推进仿真"是两种调用。

### 4.1 时间步别搞混

`model.opt.timestep`（MuJoCo 物理步长，默认常见 0.002 s）和 IK 的 `dt`（例如 1/200 = 0.005 s）是**两个独立的量**。如果每个 IK 周期只调用一次 `mj_step`，仿真时间只前进 0.002 s，而 IK 却以为过了 0.005 s，仿真里的机械臂就会比应有的慢。应当

$$
n_{\text{substeps}}=\operatorname{round}\!\left(\frac{dt_{IK}}{\text{timestep}}\right)
$$

次 `mj_step`（0.005/0.002 = 2.5，不是整数，所以更常见的做法是把物理步长设成 IK 周期的约数，如 0.001 或 0.0025/0.005）。

> **注意**：学习笔记里的旧文档提醒过一句"MuJoCo 步也是 5 ms"并不是源码已确认的事实，只是配置值之间需要协调。面试时别把"二者相等"当事实说。

---

## 5. Mink 是什么：站在 MuJoCo 上的逆运动学求解库

Mink 是一个基于 MuJoCo 的 Python **差分逆运动学（differential IK）**库。你给它：

- 当前关节配置；
- 一堆"任务"（想要手到哪里、姿态保持成什么样…）；
- 可选的"约束"（关节限位、速度限制、避碰…）；

它在每个控制周期里解一个**二次规划（QP）**，输出**关节速度**（或等价的一小步关节增量），再由你积分到关节配置里。

核心对象：

| 对象 | 作用 | 常用接口 |
|---|---|---|
| `mink.Configuration` | 机器人当前关节配置的封装，内部有自己的 `MjData`，负责 FK、雅可比、积分 | `update(q)`、`get_transform_frame_to_world(...)`、`integrate_inplace(vel, dt)`、`q` |
| `mink.FrameTask` | 让某个 frame（site/body/geom）到目标位姿（位置+朝向） | `set_target(SE3)`；参数 `position_cost`、`orientation_cost`、`gain`、`lm_damping` |
| `mink.PostureTask` | 让关节整体姿势靠近一个参考（抑制冗余自由度乱漂） | `set_target(q)`；参数 `cost` |
| `mink.Limit` 系列 | 硬约束：`ConfigurationLimit`（关节限位）、`VelocityLimit`（速度）、`CollisionAvoidanceLimit`（避碰） | 传给 `solve_ik(..., limits=[...])` |
| `mink.solve_ik` | 组装并求解 QP，返回关节速度 | `solve_ik(configuration, tasks, dt, solver, damping, limits)` |

> **提示（版本）**：以上接口名以 Mink 当前公开主线为准。学习笔记已指出：项目当时安装的 Mink 版本没有被锁定，所以"默认是否带 `ConfigurationLimit`"之类的行为，应以**你们实际装的版本**为准。面试里要讲的是"原理"，而不要断言某个版本的默认行为。

### 5.1 MuJoCo 与 Mink 的分工

| 事情 | MuJoCo | Mink |
|---|---|---|
| 加载机器人模型 | ✓ | 依赖 MuJoCo 的模型 |
| 维护 `qpos/qvel` | ✓ | `Configuration` 封装 |
| 正运动学（frame 在哪） | ✓ | 调用 MuJoCo 的结果 |
| 雅可比 | ✓（`mj_jac*`） | 读取并换到任务坐标系 |
| 物理仿真/接触/摩擦 | ✓ | ✗ |
| 逆运动学优化 | ✗ | ✓ |
| `FrameTask`/`PostureTask` | ✗ | ✓ |
| QP 求解 | ✗ | ✓（调 OSQP 等） |
| 积分（处理四元数） | 提供底层函数 | `integrate_inplace` |
| 控制真实电机 | ✗ | ✗ |

---

## 6. Task：把"想要什么"写成误差

### 6.1 FrameTask：手要到目标位姿

对某个 frame（比如左手腕的 site），当前位姿是 $T_{cur}$，目标是 $T_d$。**任务误差**是一个 6 维向量：

$$
e=\begin{bmatrix}e_p\\e_R\end{bmatrix},\quad
e_p=p_d-p_{cur}\ (\mathrm{m}),\quad
e_R=\operatorname{Log}(\ldots)\ (\mathrm{rad}).
$$

- 位置误差就是目标位置减当前位置，3 维，单位 m；
- 朝向误差**不能用欧拉角相减**（会绕回、会在万向锁附近退化）；做法是求相对旋转 $R_{err}=R_dR_{cur}^{\mathsf T}$（或 $R_{cur}^{\mathsf T}R_d$，取决于误差在哪个系里表达），再取对数 $\operatorname{Log}$ 得到**轴角向量** $\theta n$：方向是"还要绕哪个轴转"，长度是"还要转多少弧度"。

小角度时可以这样理解：设 $R_{err}$ 是绕 $z$ 轴 10° 的旋转，则 $e_R=(0,0,0.1745)^{\mathsf T}$ rad。

> 笔记里你问过 `Log` 和 $^\vee$ 是什么：可以暂时当成"把一个旋转矩阵翻译成'轴 × 角度'的三维向量"的解码器。基础文档里有完整讲解。

两个误差合起来是 6 维：3 个以 m 计，3 个以 rad 计。**它们不是同一个单位**，所以需要权重来平衡——这就是 `position_cost` 与 `orientation_cost`。

### 6.2 任务的线性化：雅可比登场

要让误差变小，需要知道"关节动一点，误差怎么变"。把任务误差关于关节做一阶近似：

$$
e(q+\Delta q)\approx e(q)-J(q)\,\Delta q.
$$

这里 $J$ 是 6×$n$ 的任务雅可比（$n$ 为被控关节数；对一只 7 自由度臂是 6×7）。要让下一步误差为零，需要

$$
J(q)\,\Delta q=\alpha\,e(q),
$$

其中 $\alpha\in(0,1]$ 是任务增益（Mink 的 `FrameTask.gain`，量纲一，含义是"这一步想消除多大比例的误差"）。**符号约定因库而异**（有的库把误差定义成"当前 − 目标"），本文统一用"目标 − 当前"，只要库内部一致即可，你不需要关心。

这是**线性**方程，并且 $J$ 随姿态变化，所以每个周期都要在新的 $q$ 上重新计算——这就是"差分 IK 每步重算"的原因。非线性的运动学 $x=f(q)$ 被拆成许多段"局部线性"。

### 6.3 PostureTask：软的姿势偏好

7 自由度臂做 6 维任务有 1 个**冗余自由度**（肘可以绕肩-腕连线转）。如果不管它，数值上它会漂移（也可能越漂越靠近限位）。`PostureTask` 的目标是 $q^*$（姿势参考），误差是 $q^*-q$，雅可比是单位阵：

$$
I\cdot\Delta q\approx\alpha_p\,(q^*-q).
$$

学习笔记里项目的做法是：**每一步把 `PostureTask` 的目标设成当前的 $q$**。于是 $q^*-q=0$，这一项的作用等价于"惩罚 $\|\Delta q\|$"（希望这一步关节别乱动），而不是拉回某个固定 home 姿态。要点：

- 它的权重要**很小**（项目里是 0.01），否则会和末端任务打架；
- 它不是硬约束，只是偏好。

### 6.4 权重（cost）怎么理解

每个任务有一个权重 $W$（Mink 里是 `position_cost`、`orientation_cost`、`cost`）。在 QP 目标里，误差被加权平方：$\|W(J\Delta q-\alpha e)\|^2$。

> **注意**：`position_cost=1`、`orientation_cost=0.5` **不表示**"位置 100% 重要、朝向 50% 重要"。因为位置误差单位是 m、朝向误差单位是 rad，二者数值尺度本来就不同；权重只是在优化里的相对偏好系数。同样，权重比较的是"平方后的误差"，所以权重 0.5 在平方后是 0.25 倍。想精确理解其效果，要结合典型误差大小。

### 6.5 `lm_damping`：误差相关的阻尼

`FrameTask` 还有一个 `lm_damping` 参数（Levenberg–Marquardt 阻尼）。直觉：在 QP 的二次项里再加一个与"误差平方"成正比的 $\mu I$：

$$
\mu=\text{lm\_damping}\cdot\|W e\|^2.
$$

含义：**误差大时阻尼大、动作保守；误差小时阻尼小、几乎不影响**。它与全局的 `damping`（`solve_ik` 参数，一个固定小数）作用不同，详见 [04 章](./04_逆运动学_雅可比_奇异性与阻尼最小二乘.md)。

---

## 7. QP：把多个任务和约束放进一个优化问题

一个周期里，Mink 把所有任务和约束放在**同一个变量** $\Delta q$ 上解：

$$
\min_{\Delta q}\ \sum_i\big\|W_i\,(J_i\Delta q-\alpha_ie_i)\big\|^2\ +\ \lambda^2\|\Delta q\|^2
\qquad\text{s.t.}\quad G\,\Delta q\le h.
$$

逐项读：

- $i$：任务编号——左手、右手、Posture；
- $J_iΔq-\alpha_ie_i$：第 $i$ 个任务的残差（希望为零）；
- $W_i$：任务权重；
- $\lambda^2\|\Delta q\|^2$：**全局阻尼项，整个问题只加一次**（学习笔记第 34 组你专门问过这一点：它惩罚的是"这一组 $\Delta q$ 有多大"，不是每个任务各一份）；
- $G\Delta q\le h$：线性不等式约束（关节限位、速度限制、避碰线性化后都写成这个形式）。

### 7.1 为什么是"二次"规划

残差平方和展开后是 $\Delta q$ 的二次函数（含 $\Delta q^{\mathsf T}H\Delta q$ 项），约束是线性的——**二次目标 + 线性约束 = QP**。标准形：

$$
\min\ \tfrac12\,\Delta q^{\mathsf T}H\,\Delta q+c^{\mathsf T}\Delta q,\quad
H=\sum_iJ_i^{\mathsf T}W_i^2J_i+\lambda^2I,\quad
c=-\sum_iJ_i^{\mathsf T}W_i^2\,\alpha_ie_i.
$$

（推导：展开平方 $\|W(J\Delta q-b)\|^2=\Delta q^{\mathsf T}J^{\mathsf T}W^2J\,\Delta q-2b^{\mathsf T}W^2J\,\Delta q+b^{\mathsf T}W^2b$，常数项不影响最小化。）

**没有约束时**，令梯度为零就得到线性方程：

$$
H\,\Delta q=-c\ \Longleftrightarrow\ \Big(\sum_iJ_i^{\mathsf T}W_i^2J_i+\lambda^2I\Big)\Delta q=\sum_iJ_i^{\mathsf T}W_i^2\alpha_ie_i.
$$

这就是**阻尼最小二乘**的"正规方程"。**有约束时**（比如某个关节速度碰到上限）就需要 QP 求解器（OSQP、DAQP 等）做迭代求解——这正是为什么 CV 里说"constrained IK"。

### 7.2 为什么左右手和姿势要放在一起解，不能分开

笔记里你专门问过："左右手不是两个运动吗，为什么要一起算？"

理由有三：

1. **可能共享关节**：很多双臂机器人有共享的腰/躯干关节，一个 $\Delta q$ 会同时影响两只手（你们的机型两臂是否共享关节【待你补全】；若两臂完全独立，QP 在数学上可以分块，但统一求解仍然方便加全局约束与统一权重）；
2. **冲突需要折中**：若分别求解再合并，两个解可能相互矛盾；放进同一个目标函数，冲突会按权重**自动折中**；
3. **约束是全局的**：关节限位对整个 $\Delta q$ 有效，必须在同一个问题里满足。

一个极简例子：两个任务共享同一个关节 $x$，任务 1 要 $x=0.2$，任务 2 要 $x=-0.2$。权重相同则折中为 0；若任务 1 权重 1、任务 2 权重 0.1，则

$$
x=\frac{1\cdot0.2+0.01\cdot(-0.2)}{1+0.01}\ \text{（权重在平方后进入）}\approx0.196,
$$

偏向权重大的任务。对 7 自由度臂的左右手，各自的 6 维任务 + 姿势任务叠成一个更大的最小二乘（例如两臂 14 个关节、两个 6 维任务 + 14 维姿势任务），冲突就这样折中。

---

## 8. `dt` 与积分：小步走

QP 求解出的是**这一步**的关节增量 $\Delta q$（Mink 内部决策变量是 $\Delta q$，`solve_ik` 返回的是 $\dot q=\Delta q/dt$）。你再积分：

$$
q_{k+1}=q_k\oplus(\dot q\cdot dt),
$$

其中 $\oplus$ 表示"按关节类型正确地加"（转动关节就是普通加法，含四元数的关节则走指数映射）。Mink 里是 `configuration.integrate_inplace(vel, dt)`。

### 8.1 为什么不一次算到位

运动学是非线性的（含 $\sin\cos$）。一次线性化只在 $q$ 附近有效，步子大了会走过头甚至发散。所以采用"小步 + 每步重新线性化"：

```text
读当前 q → 算误差 e → 算雅可比 J → 解 QP 得 Δq → q ← q + Δq → 重复
```

### 8.2 dt 的数值意义

项目的求解器配置（学习笔记记录）是 $dt=1/200\ \text{s}=5\ \text{ms}$。例：关节速度 $0.4\ \text{rad/s}$，一步走 $0.4\times0.005=0.002\ \text{rad}\approx0.115^\circ$。很小，连续做才有平滑动作。

再强调一次（同样来自笔记）：**$dt$ 是求解器的步长配置，不是 VR 数据频率，也不是电机内环频率，也不是程序实测运行频率。** 三者混淆是面试的高频坑，见 [01 章第 7 节](./01_项目背景与系统总览.md)。

### 8.3 `dt` 影响的三个地方

1. $\dot q=\Delta q/dt$ 的换算；
2. 速度限制：$|\Delta q|\le\dot q_{max}\,dt$；
3. 积分 $q\leftarrow q+\dot q\,dt$。

若实际循环周期是 10 ms 而你仍用 5 ms 积分，关节每步实际走的少了一半，跟随会变慢（目标追不上）。所以真机上要**测量实际周期**并用它。

---

## 9. 一整轮 `solve_step` 逐行讲解

下面是教学骨架，结构对应学习笔记里描述的项目求解器 `solve_step`。**这是教学示例代码，不是项目源码**（VR 目录在你的资料里为空）：

```python
import numpy as np
import mujoco
import mink

class TeachIK:
    def __init__(self, xml_path, left_site, right_site, dt=1/200):
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.model.opt.gravity[:] = 0.0             # IK 只用运动学
        self.dt = dt
        self.cfg = mink.Configuration(self.model)   # 内部持有 MjData

        self.left  = mink.FrameTask(frame_name=left_site,  frame_type="site",
                                    position_cost=1.0, orientation_cost=0.5,
                                    lm_damping=0.05)
        self.right = mink.FrameTask(frame_name=right_site, frame_type="site",
                                    position_cost=1.0, orientation_cost=0.5,
                                    lm_damping=0.05)
        self.posture = mink.PostureTask(self.model, cost=0.01)

    def solve_step(self, q_now, T_left=None, T_right=None):
        q_now = np.asarray(q_now, dtype=float).copy()
        self.cfg.update(q_now)                      # (1) 把当前关节写进配置并刷新 FK
        self.posture.set_target(q_now)              # (2) 姿势参考 = 当前 q

        tasks = [self.posture]                      # (3) 收集本周期的任务
        if T_left is not None:
            self.left.set_target(T_left)            #     T_* 是 mink.SE3
            tasks.append(self.left)
        if T_right is not None:
            self.right.set_target(T_right)
            tasks.append(self.right)

        try:
            vel = mink.solve_ik(self.cfg, tasks, self.dt,
                                solver="osqp", damping=1e-3)   # (4) 解 QP
        except Exception:
            return q_now                            # (5) 失败: 这一步不前进
        self.cfg.integrate_inplace(vel, self.dt)    # (6) 积分得到下一步配置
        return self.cfg.q.copy()                    # (7) 交给下游
```

逐行翻译：

| 步骤 | 做什么 | 为什么 |
|---|---|---|
| (1) `cfg.update(q_now)` | 把真实/当前关节写入配置，刷新所有 frame 的位姿和雅可比所需的中间量 | 每周期必须在**新状态**上重算 |
| (2) `posture.set_target(q_now)` | 姿势参考=当前 q | 让冗余自由度这一步不乱动 |
| (3) 添加任务 | 触发键按下的手才有 `FrameTask` | 没按的手只受姿势任务约束，不动 |
| (4) `solve_ik` | 组装 $H,c,G,h$，调用 OSQP，返回 $\dot q$ | 内部 $\Delta q=\dot q\cdot dt$ |
| (5) 异常 | QP 不可行/求解失败 → 返回当前 q | 项目里的"保持"行为，见下 |
| (6) `integrate_inplace` | $q\leftarrow q\oplus\dot q\,dt$ | 正确处理四元数 |
| (7) 返回 | 下一步关节配置 | 之后写入 `qpos/ctrl` 或发布到 ROS 2 |

**关于 (5) 的重要提醒**（学习笔记反复提）：返回当前 $q$ 只是"这一步没产生新动作"。如果下游的 MIT 位置环仍然使能，电机会继续**用上一个目标**出力保持，这不等于"安全停机"。真正的失联/故障策略要看下游（[10 章](./10_ROS2_CAN执行链.md)）。

此外，项目还有一个"迭代到收敛"的变体（`solve_until_converged`）：重复 `solve_step`，同时检查 (a) 关节增量足够小、(b) 位置误差（mm）足够小、(c) 姿态误差（度）足够小。**只满足 (a) 不够**：$\Delta q$ 小可能只是卡住（QP 失败、撞限位、奇异），并不表示到达了目标。

---

## 10. 一个二连杆的完整数字例子

用平面二连杆把一轮 IK 完整算一遍（示例数值，非项目实测）：$l_1=l_2=1$ m，当前 $q=(0,\ 90^\circ)$。

手端位置 $p=(\cos q_1+\cos(q_1+q_2),\ \sin q_1+\sin(q_1+q_2))=(1,\ 1)$。雅可比（对 $x$ 和 $y$ 分别对 $q_1,q_2$ 求偏导）：

$$
J=\begin{bmatrix}-\sin q_1-\sin(q_1+q_2)&-\sin(q_1+q_2)\\ \cos q_1+\cos(q_1+q_2)&\cos(q_1+q_2)\end{bmatrix}
=\begin{bmatrix}-1&-1\\1&0\end{bmatrix}.
$$

**场景**：VR 手以约 0.4 m/s 运动，在 5 ms 一步内，目标比当前位置多走了 $e=(0.002,\ 0.001)$ m。设 $\alpha=1$、位置权重 1，姿势任务权重 $w_p=0.1$（残差 $I\Delta q$，所以 $H$ 里加 $w_p^2=0.01$），全局阻尼 $\lambda^2=0.01$。

**第 1 步：组装 $H$ 与右端项。**

$$
J^{\mathsf T}J=\begin{bmatrix}-1&1\\-1&0\end{bmatrix}\begin{bmatrix}-1&-1\\1&0\end{bmatrix}=\begin{bmatrix}2&1\\1&1\end{bmatrix},
\qquad
H=J^{\mathsf T}J+(0.01+0.01)I=\begin{bmatrix}2.02&1\\1&1.02\end{bmatrix}.
$$

$$
J^{\mathsf T}e=\begin{bmatrix}-1&1\\-1&0\end{bmatrix}\begin{bmatrix}0.002\\0.001\end{bmatrix}=\begin{bmatrix}-0.001\\-0.002\end{bmatrix}.
$$

**第 2 步：解 $H\Delta q=J^{\mathsf T}e$。** $\det H=2.02\times1.02-1\times1=1.0604$，$H^{-1}=\dfrac1{1.0604}\begin{bmatrix}1.02&-1\\-1&2.02\end{bmatrix}$：

$$
\Delta q=\frac1{1.0604}\begin{bmatrix}1.02(-0.001)-1(-0.002)\\-1(-0.001)+2.02(-0.002)\end{bmatrix}
=\frac1{1.0604}\begin{bmatrix}0.00098\\-0.00304\end{bmatrix}
=\begin{bmatrix}0.000924\\-0.002867\end{bmatrix}\ \text{rad}.
$$

对比无阻尼精确解 $J^{-1}e=(0.001,\,-0.003)$：略小（阻尼和姿势项"收缩"了步子），这就是"跟踪与平稳之间的折中"。

**第 3 步：返回速度。** $\dot q=\Delta q/dt=(0.185,\ -0.573)$ rad/s。

**第 4 步：积分。** $q_{next}=(0.000924,\ 1.570796-0.002867)=(0.000924,\ 1.567929)$ rad。

**第 5 步：检验手端实际位移。**

$$
J\Delta q=\begin{bmatrix}-0.000924+0.002867\\0.000924\end{bmatrix}=\begin{bmatrix}0.001943\\0.000924\end{bmatrix},
$$

比目标位移 $(0.002,\,0.001)$ 差 $(0.000057,\,0.000076)$ m，约 0.1 mm。

**第 6 步：下一周期重新算。** 保持目标不动再迭代，位置误差依次是 2.24 mm → 0.097 mm → 0.0048 mm → 0.00024 mm……每步缩小为原来的约 4%，非常快；这是因为这个姿态下雅可比条件好（奇异值 1.62 和 0.62）。接近奇异位形时就完全不同了，见下一章。

> **提示**：这个例子里 $\alpha=1$ 且误差只有 2 mm，所以速度（0.185、0.573 rad/s）合理。如果误差是 2 cm（目标突然跳了 10 倍），同样的 $\alpha=1$ 会给出十倍的速度——这就是为什么要对目标做**速率限幅**以及在 QP 里加**速度约束**（[05 章](./05_约束与限位.md)）。

---

## 11. IK 结果怎么变成"电机命令"：三种做法

IK 输出的是"下一步关节配置/速度"，它仍然不是电机命令。常见的三种落地方式（学习笔记第 7 组列出）：

| 方式 | 做法 | 适用 | 注意 |
|---|---|---|---|
| A. 位置执行器 | 把 $q^*$ 写入 `position` 型执行器的 `ctrl`，由仿真里的执行器模型追 | 仿真里要有动力学跟随 | `ctrl` 才是目标角 |
| B. 力矩执行器 | 另写一个关节控制器 $\tau=K_p(q^*-q)+K_d(\dot q^*-\dot q)+\tau_{ff}$，把 $\tau$ 写入 `motor` 型 `ctrl` | 想接近真实电机受力运动 | 需要合理增益和动力学 |
| C. 直接改 `qpos` | 把 $q^*$ 直接写进 `data.qpos`，再 `mj_forward` | 纯运动学回放/调试 | **不是**电机经过动力学运动到目标 |

**真机**：把 $q^*$（以及可选的 $\dot q^*$）作为关节目标，发布到 ROS 2 话题；硬件节点做"关节→电机"映射后，打包成 **MIT 帧**（pos、vel、kp、kd、τ）发到 CAN；电机内环去追。这对应 B 的思路（电机驱动器内部实现 $\tau=k_p(\cdot)+k_d(\cdot)$）。编码器把真实的 $q,\dot q$ 送回来。

> **关键认识**：仿真里的 `qpos` 和真机的编码器读数是**两个不同的状态**。理想闭环是：每个周期从**真实编码器**读当前 $q$ 作为 IK 的起点，而不是一直用 IK 自己积分出来的 $q$（后者会逐渐偏离真实姿态）。学习笔记反复强调过："真实编码器的关节状态有没有同步回 Mink 配置"必须核对；这是区分"开环回放"与"闭环遥操作"的关键。

---

## 12. 项目里的配置（来自学习笔记）与口径

学习笔记里你们对 VR 路径源码逐段核对过的配置（VR 目录在本仓库为空，下列取自笔记，不是我能直接打开的文件）：

| 项 | 值 | 说明 |
|---|---|---|
| 步长 | $dt=1/200$ s | 配置值，不是实测频率 |
| `FrameTask.position_cost` | 1 | |
| `FrameTask.orientation_cost` | 0.5 | |
| `PostureTask.cost` | 0.01 | 目标=当前 q |
| `FrameTask.lm_damping` | 0.05 | 误差相关阻尼 |
| 求解器 | OSQP | |
| 模型重力 | 设为 0 | IK 只用运动学 |
| 失败处理 | 捕获异常，返回当前 q | 一步不前进 |
| 全局 `damping`、`limits` 是否显式传入 | 【待你补全：按实际代码核对】 | 笔记提醒：没有显式传 `limits` 时默认行为取决于 Mink 版本 |

> **面试口径**：你可以说"位置权重 1、朝向权重 0.5、姿势权重 0.01、LM 阻尼 0.05、200 Hz 步长、OSQP"，但要补一句"这些是设计配置，不是我声称的性能指标"。

---

## 13. 常见错误

| 错误 | 现象 | 原因/处理 |
|---|---|---|
| frame 名字写错 | 初始化报错或找不到 frame | `FrameTask(frame_name=...)` 的名字必须与 XML 中 site/body 名一致 |
| `nq≠nv` 时直接 `q += vel*dt` | 长度不符或四元数畸变 | 用 `integrate_inplace` / `mj_integratePos` |
| `ctrl` 当成关节角但执行器是 `motor` | 仿真里机械臂乱抽或不动 | 核对 `<actuator>` 类型 |
| 每周期只 `mj_step` 一次但 `dt` 更大 | 仿真比目标慢 | 子步数 = dt/timestep |
| 把 Posture 目标固定在 home | 手移动时关节被拉回 | 目标=当前 q，或权重足够小 |
| 只看"Δq 很小"就认为到位 | 实际卡在奇异/限位 | 同时检查位置、姿态误差 |
| 异常后返回当前 q 就以为安全 | 电机仍在保持上个目标 | 在下游加超时/使能策略 |
| 以为权重 1 与 0.5 表示"重要两倍" | 调参方向错 | 结合单位和典型误差调 |
| 目标突然跳变 | 关节速度尖峰 | 目标速率限幅 + QP 速度约束 |
| IK 内部 q 与真实编码器脱节 | 越用越偏 | 每周期用真实反馈刷新起点 |

---

## 14. 面试怎么问、怎么答

**Q1. What are MuJoCo and Mink, and how do they work together in your teleoperation?**

> MuJoCo is a physics engine and robot model library. I use it to load the dual-arm model and compute forward kinematics and Jacobians. Mink is an inverse-kinematics library built on top of MuJoCo. Every control cycle it takes my task targets, such as the left and right hand poses plus a posture preference, builds a small quadratic program in the joint-increment, solves it with OSQP, and returns a joint velocity. I integrate that velocity over a small time step to get the next joint configuration.

中文要点：MuJoCo=模型+FK+雅可比；Mink=任务→QP→关节速度；再积分。

*追问*：Why a QP and not just a pseudo-inverse? → 要同时处理多任务权重和不等式约束（限位/速度）；伪逆无法处理硬约束。

**Q2. Why not send the controller pose straight to the motors?**

> A motor only understands a joint command. The controller gives me a hand pose, and several joints together determine that pose. So I need inverse kinematics to turn the hand target into joint targets. The real robot does not need MuJoCo to move, but it needs those joint targets, and MuJoCo gives me the model needed to compute them.

**Q3. What is the difference between `qpos`, `qvel` and `ctrl`, and why can `nq` differ from `nv`?**

> `qpos` is the generalised position, `qvel` the generalised velocity, and `ctrl` is the input to the actuators, whose meaning depends on the actuator type. For free or ball joints the position uses a quaternion with four numbers while the velocity has three, so `nq` is not equal to `nv`. That is why I integrate with the library function instead of simply adding velocity to position.

**Q4. How do you pick the weights and the time step?**

> The time step is the control period I assume for the solver, 5 milliseconds in our configuration. The weights are relative preferences: position cost one, orientation cost 0.5, and a very small posture cost so it only breaks ties among redundant solutions. I tuned them by checking tracking error and joint motion on the arm, but I would not claim they are optimal. 【如被追问具体调参过程：补全自己的真实经历。】

**Q5. What happens if the solver fails?**

> In our solver wrapper an exception returns the current configuration, so that step produces no new motion. But I do not treat that as a safe stop: the downstream position controller may still hold the last target. Safe handling needs a timeout and an enable policy at the hardware node.

---

## 本章要点速记

1. 真机不一定要 MuJoCo 才能动；但要把"手的目标"变成"关节命令"，需要模型 + IK。MuJoCo 提供模型/FK/雅可比，Mink 做 IK。
2. `MjModel` 是说明书，`MjData` 是当前状态；`qpos/qvel/ctrl` 含义不同，`ctrl` 取决于执行器类型。
3. `nq` 可以 ≠ `nv`（四元数），积分必须用库函数。
4. `mj_forward` 只算派生量，`mj_step` 才推进时间；IK 只需前者。
5. 任务 = 误差 + 雅可比 + 权重；朝向误差用 $\operatorname{Log}$ 轴角，不用欧拉角相减。
6. QP：$\min\sum\|W(J\Delta q-\alpha e)\|^2+\lambda^2\|\Delta q\|^2$，s.t. $G\Delta q\le h$；阻尼项只加一次。
7. 多任务放同一个 QP，是因为共享关节、冲突要折中、约束是全局的。
8. 小步走：$\Delta q=\dot q\,dt$，每步重新线性化；$dt=5$ ms 是配置值。
9. IK 失败返回当前 q ≠ 安全停机；Δq 小 ≠ 到位。
10. 理想闭环用真实编码器刷新 IK 起点。

## 自测题

1. `ctrl` 一定是关节角吗？不是时有哪几种可能？（取决于执行器：motor→力矩，position→目标角，velocity→目标速度。）
2. 为什么 7 自由度臂做 6 维任务需要 PostureTask？（1 个冗余自由度，会漂；姿势任务给软偏好。）
3. 阻尼项 $\lambda^2\|\Delta q\|^2$ 是每个任务各一份吗？（不是，全局只加一次，因为 $\Delta q$ 是唯一决策变量。）
4. 用本章 10 节的 $J,\ e$，若无阻尼无姿势项，$\Delta q$ 是多少？（$J^{-1}e=(0.001,-0.003)$。）
5. 若 IK 周期实际是 10 ms 但仍用 $dt=5$ ms，会怎样？（积分步长偏小，跟随变慢。）
6. 说出把 IK 结果落地的三种方式及适用场景。（位置执行器/力矩+PD/直接改 qpos。）

## 相关链接

- 上一章：[02 VR 遥操作的相对位姿映射](./02_VR遥操作的相对位姿映射.md)
- 下一章：[04 逆运动学、雅可比、奇异性与阻尼最小二乘](./04_逆运动学_雅可比_奇异性与阻尼最小二乘.md)
- 约束：[05 约束与限位](./05_约束与限位.md)
- 基础：[正逆运动学与雅可比](../../02_基础知识/03_坐标变换与运动学/03_正逆运动学与雅可比.md)、[最优化与二次规划](../../02_基础知识/01_数学/04_最优化与二次规划.md)、[MuJoCo/Pinocchio/KDL 工具链](../../02_基础知识/09_机器人软件/06_MuJoCo_Pinocchio_KDL_工具链.md)
- 学习笔记（原始疑惑）：[P2 逐字稿第 5–7、28–34 组](../../05_学习笔记归档/P2/P2_本对话问答逐字稿.md)
