# 10 仿真平台：Gazebo、Isaac Sim、MuJoCo 对比与选型

> **一句话定位**：仿真是机器人开发的"便宜实验室"，但每个仿真器都是一组**取舍**：物理引擎怎么处理接触、传感器渲染多真、能并行多少环境、怎么接 ROS2。本章先讲清仿真器内部"时间步长—刚度—稳定性"这条硬约束，再把 Gazebo、Isaac Sim、MuJoCo 放进同一张表比较，最后讲随机化（domain randomization）与批量仿真，以及怎样根据任务选型，并诚实地区分你在项目里实际用过的部分。

**本章学完你能：**

- 解释仿真器的一次"步进"做了什么：碰撞检测、约束求解、积分，以及为什么时间步长与接触刚度必须匹配；
- 手算：给定接触刚度，算出显式/半隐式积分的最大稳定步长，并用小程序验证"超过就发散"；
- 比较三个平台在物理建模、传感器仿真、ROS2 接入、并行规模、许可与硬件要求上的差异（具体版本与特性标"未核对"）；
- 说明实时因子（RTF）与批量吞吐两个指标的含义并会算；
- 设计一个域随机化方案，并用蒙特卡洛估计随机化对任务成功率的影响；
- 列出 sim-to-real 差距的主要来源，并给出缩小差距的清单；
- 针对"控制算法验证 / 导航栈联调 / 强化学习训练 / 操作任务"四类任务给出选型建议，回答"你会选哪个仿真器、为什么"。

**前置知识：**

- [MuJoCo、Pinocchio、KDL 工具链](./06_MuJoCo_Pinocchio_KDL_工具链.md)（模型、状态、动力学库）
- [URDF、TF、坐标系实践](./07_URDF_TF_坐标系实践.md)（机器人描述）
- [ROS2 Humble 实战：launch、参数、生命周期、组件](./08_ROS2_Humble实战_launch_参数_生命周期_组件.md)（`use_sim_time`）
- [强化学习进阶：策略梯度、PPO、SAC](../10_机器学习与深度学习/09_强化学习进阶_策略梯度_PPO_SAC.md)（sim-to-real、域随机化）

> **关于事实的说明**：本机没有安装 Gazebo、Isaac Sim 或 ROS2，各平台的版本号、特性、许可、与 ROS2 Humble 的配套关系**我没有逐项核对**，凡出现处标"未核对"，面试时只讲"思想和取舍"，不要断言版本。章内所有数值（刚度、步长、蒙特卡洛结果）是**教学示例（非项目实测）**，并已用本地 Python 复核。

**目录**

1. 为什么要仿真，以及仿真的边界
2. 仿真器内部：一次步进做了什么
3. 手算：接触刚度与最大稳定步长
4. 三个平台的特点
5. 并行批量仿真与吞吐指标
6. 传感器仿真
7. 与 ROS2 的接入方式
8. 域随机化：手算蒙特卡洛
9. sim-to-real 差距的来源与缩小办法
10. 选型决策
11. 在七个项目里怎么用（含诚实边界）
12. 常见错误与故障现象
13. 面试怎么问、怎么答
14. 本章要点速记
15. 自测题
16. 相关链接

---

## 1. 为什么要仿真，以及仿真的边界

### 1.1 仿真解决什么问题

1. **安全**：摔机器人、撞人的实验先在虚拟世界做；
2. **成本与速度**：仿真可以加速运行、重置只需一次函数调用，实机重置要人搬；
3. **可重复**：同一初始条件反复运行，便于对比算法；
4. **可观测**：仿真里能读到"真值"（真实位姿、接触力），实机没有真值；
5. **并行**：一台带 GPU 的机器可以同时跑成千上万个环境，这是强化学习的基础。

### 1.2 仿真不能做的事

仿真永远是**模型**，而模型是对现实的近似。电机非线性、减速器摩擦、线缆拉扯、传感器怪毛病、网络抖动、地面材质……都只能近似。因此：

> 仿真通过 ≠ 实机通过；仿真不通过 → 基本可以肯定实机有问题。

仿真的价值在于**过滤明显的错误并提供对比基准**，而不是替代实机验证。

---

## 2. 仿真器内部：一次步进做了什么

一次 `step(dt)` 的典型流程（不同引擎细节不同）：

```text
读取当前状态 (q, v) 与控制量 u
   │
   ▼
1. 正向运动学：求每个连杆/几何体的位姿
2. 碰撞检测：粗检测（包围盒）→ 精检测（接触点、法向、穿透深度）
3. 约束与接触求解：关节限位、接触、摩擦、等式约束 → 产生约束力
4. 动力学：M(q) dv/dt = tau + J^T f - h   →   求加速度
5. 积分：用步长 dt 更新 (q, v)
6. 传感器：生成 IMU、关节编码器、相机、激光等读数
   │
   ▼
新状态 (q', v')，仿真时间 += dt
```

### 2.1 接触是难点

刚体接触是"硬约束"：物体不能穿透，接触力只能推不能拉，摩擦力受库仑锥限制。精确求解是一个**互补问题**，计算量大、可能无解或不唯一。各引擎的核心差异就在于怎么处理它：

| 处理思路 | 描述 | 特点 |
|---|---|---|
| 罚函数/弹簧阻尼（软接触） | 穿透时产生与深度成正比的弹力 | 简单，但需要大刚度才不穿透，导致刚性方程、小步长 |
| 互补/冲量法 | 解出使约束成立的冲量 | 更"硬"，对大步长友好，但求解复杂 |
| 凸松弛的软约束（MuJoCo 的思路） | 把接触化为凸优化问题，允许可调的"软度"参数 | 求解稳定、可微分性较好，物理上有一定近似 |

（以上是思想层面的归纳，各引擎具体实现细节**未核对**。）

### 2.2 时间步长的三种"步长"

- **仿真步长** $\Delta t_{\text{sim}}$：物理积分的时间间隔，如 1–2 ms；
- **控制步长** $\Delta t_{\text{ctrl}}$：控制器被调用的周期，如 5–20 ms（一个控制步内可包含多个仿真步，这叫"降采样/子步进"）；
- **渲染步长**：画面更新，如 30 FPS，与物理无关。

三者不一致是常态，也是 bug 的常见来源：控制器按 100 Hz 写，仿真却以 1 kHz 跑而没做时间对齐，会让"仿真里的时序"与"实机里的时序"系统性不同。

---

## 3. 手算：接触刚度与最大稳定步长

### 3.1 问题

软接触等价于一个弹簧：质量 $m$，刚度 $k$。弹簧质点系统的固有角频率 $\omega=\sqrt{k/m}$。数值积分有稳定条件：步长不能太大，否则能量被数值"放大"，仿真发散（物体弹飞、爆炸）。

### 3.2 推导（半隐式欧拉）

半隐式（symplectic）欧拉先更新速度再用新速度更新位置：

$$
v_{n+1}=v_n-\omega^2 x_n\,\Delta t,\qquad x_{n+1}=x_n+v_{n+1}\Delta t.
$$

写成矩阵 $\begin{bmatrix}x_{n+1}\\v_{n+1}\end{bmatrix}=A\begin{bmatrix}x_n\\v_n\end{bmatrix}$，其中

$$
A=\begin{bmatrix}1-\omega^2\Delta t^2 & \Delta t\\ -\omega^2\Delta t & 1\end{bmatrix}.
$$

$\det A=(1-\omega^2\Delta t^2)\cdot1-\Delta t\cdot(-\omega^2\Delta t)=1$；迹 $\operatorname{tr}A=2-\omega^2\Delta t^2$。特征值满足 $\lambda^2-\operatorname{tr}A\,\lambda+1=0$。特征值模为 1（临界稳定，能量近似守恒）的条件是判别式 $\le0$，即 $|\operatorname{tr}A|\le2$：

$$
|2-\omega^2\Delta t^2|\le2\;\Longrightarrow\;\Delta t\le\frac{2}{\omega}.
$$

超过 $2/\omega$，特征值变成实数且有一个模大于 1，数值解指数发散。

**对比**：显式欧拉（先用旧速度更新位置再更新速度）对无阻尼振子，每步振幅放大 $\sqrt{1+\omega^2\Delta t^2}>1$，对任何步长都缓慢发散，需要靠阻尼压住。这就是物理引擎普遍使用半隐式或隐式积分的原因。

### 3.3 手算（质量 1 kg，示例，非项目实测）

| 接触刚度 $k$（N/m） | $\omega=\sqrt{k/m}$（rad/s） | 最大步长 $2/\omega$ |
|---|---|---|
| $10^3$ | 31.6 | 63.2 ms |
| $10^5$ | 316.2 | 6.32 ms |
| $10^7$ | 3162.3 | 0.632 ms |

结论：**刚度每增大 100 倍，最大稳定步长缩小 10 倍**。要让物体几乎不穿透需要大刚度，于是步长被迫很小，仿真变慢。

### 3.4 数值验证

对 $k=10^5$，$m=1$，初始位移 0.01 m，仿真 1 s，记录最大位移：

| 步长 | 最大位移（m） | 结论 |
|---|---|---|
| 1 ms | 0.0101 | 稳定 |
| 5 ms | 0.0163 | 稳定但数值误差变大 |
| 6 ms | 0.0316 | 接近边界，误差明显 |
| 6.5 ms | $4.5\times10^{29}$ | 发散 |
| 7 ms | $5.4\times10^{54}$ | 发散 |

（Python 复核。理论边界 6.32 ms，实验在 6 ms 稳定、6.5 ms 发散，与推导一致。）

**这个小实验的教训**：

1. 仿真"爆炸"首先怀疑步长与刚度不匹配，而不是怀疑控制算法；
2. 离边界越近，数值误差越大——"没发散"不等于"准确"，留 2–5 倍余量；
3. 不同引擎用"接触时间常数/软度"之类的参数间接设刚度，通常要求时间常数不小于若干倍步长（MuJoCo 文档有类似建议，具体倍数**未核对**）。

---

## 4. 三个平台的特点

以下为定性比较，**具体版本、特性与许可条款未核对**；写面试答案时用"typically / in general"。

### 4.1 Gazebo

- **定位**：面向 ROS 的通用机器人仿真器。老版本（Gazebo Classic）与新一代（原 Ignition，现称 Gazebo Sim，使用 `ros_gz` 一类桥接包）并存，与 ROS2 Humble 的官方配套关系**未核对**，选型前要查当前文档；
- **物理**：插件式引擎，可选多种（如 DART、Bullet 等，默认与可选项**未核对**）；
- **传感器**：相机、深度、激光雷达（GPU 或 CPU 射线）、IMU、GPS、接触传感器等，带噪声模型；
- **世界与模型**：SDF 描述，可与 URDF 互转（需要补全惯量、碰撞体、插件）；
- **优点**：与 ROS 生态结合最紧，导航栈（如 Nav2）联调常用；有大量现成模型；
- **缺点**：大规模并行不是强项；接触物理精度和速度通常不及专门的操作仿真器；资源占用较高。

### 4.2 NVIDIA Isaac Sim（含 Isaac Lab）

- **定位**：基于 GPU 的高保真、可大规模并行仿真平台，渲染（光线追踪）和物理（GPU 加速）都在显卡上；
- **物理**：使用 PhysX 一类引擎的 GPU 加速版本，支持成千上万个并行环境（常用于强化学习，Isaac Lab 是其上的学习框架，细节**未核对**）；
- **传感器**：高质量的相机、深度、激光雷达、IMU，适合训练视觉模型与生成合成数据；
- **ROS2 接入**：提供 ROS2 桥接，与 Humble 的支持情况**未核对**；
- **优点**：渲染逼真，合成数据能力强；并行规模大；
- **缺点**：需要较强的 NVIDIA GPU（具体要求**未核对**）；软件体积大、启动慢；许可与使用条款需自行查阅；学习曲线陡；物理细节（接触）与 MuJoCo 不同，策略迁移要重新验证。

### 4.3 MuJoCo

- **定位**：以速度和接触建模质量见长的物理引擎，广泛用于机器人学习与控制研究；
- **模型**：MJCF（XML），也能加载 URDF（需要转换/补充参数）；采用广义坐标，动力学计算效率高；
- **接触**：凸优化形式的软接触，参数可调；对大步长较稳健；
- **并行**：CPU 多线程批量运行；另有基于加速器的版本（MJX，用 JAX 实现）用于大规模并行，具体功能与限制**未核对**；
- **渲染与传感器**：渲染相对朴素，足以生成相机图像与简单传感器读数，不追求影视级逼真；
- **ROS2 接入**：本身不是 ROS 组件，通常自己写一个节点把 MuJoCo 的状态发布成话题、订阅命令话题，或使用社区封装（**未核对**）；
- **优点**：轻量、快、接触稳定、可编程性强（Python 绑定）；
- **缺点**：不是"开箱即用的 ROS 仿真环境"，传感器和场景生态需要自己搭。

### 4.4 对比表

| 维度 | Gazebo | Isaac Sim / Isaac Lab | MuJoCo |
|---|---|---|---|
| 主要用途 | ROS 系统联调、导航 | 大规模 RL、合成数据、高保真感知 | 控制/学习研究、操作、IK 与动力学验证 |
| 物理 | 插件式，多引擎 | GPU 物理 | 自带，接触建模见长 |
| 渲染 | 一般 | 逼真 | 朴素 |
| 并行规模 | 小–中 | 大（GPU） | 中（CPU）/ 大（加速器版） |
| ROS2 接入 | 紧密（桥接包） | 提供桥接 | 自行封装 |
| 硬件要求 | CPU 为主 | 需较强 NVIDIA GPU | CPU 即可 |
| 上手难度 | 中 | 高 | 低–中 |
| 典型风险 | 大规模训练吃力 | 环境复杂、依赖重 | 要自己搭 ROS 集成 |

> **注意**：表中是"典型情况"的归纳，不是基准测试结果。若有人问"哪个最快"，正确答案是"取决于场景、接触数、机器人自由度和硬件；我会在目标任务上做小基准测试再决定"。

---

## 5. 并行批量仿真与吞吐指标

### 5.1 两个指标

**实时因子（RTF）**：仿真时间 / 墙钟时间。RTF = 1 表示与真实时间同速；RTF = 2.5 表示 10 s 的仿真用 4 s 墙钟完成。导航联调需要 RTF 接近 1 且稳定（否则传感器与 `/clock` 关系异常，参见 [第 08 章](./08_ROS2_Humble实战_launch_参数_生命周期_组件.md) 的 `use_sim_time`）；训练则希望 RTF 越大越好。

**批量吞吐（环境步/秒）**：并行环境数 × 每个环境每秒步数。

### 5.2 手算（示例，非项目实测）

- 单环境：10 s 仿真用时 4 s，RTF $=10/4=2.5$；若仿真步长 2 ms，每秒墙钟完成 $10/0.002/4=1250$ 个物理步；
- 批量：4096 个并行环境，假设每个环境每秒完成 100 个控制步（控制频率 50 Hz，即单环境 RTF = 2），总吞吐 $4096\times100=409{,}600$ 环境步/秒；折算成机器人经验时长：每墙钟秒得到 $409{,}600/50=8192$ 秒的机器人经验，即每墙钟小时约 8192 小时（Python 复核：8192）。

这就是强化学习依赖 GPU 并行仿真的原因：用真机一小时只有一小时的经验，仿真一小时可以得到上千小时。但前提是你的**奖励、重置、观测**也都在 GPU 上批量计算，否则瓶颈会转移到 Python 循环。

### 5.3 吞吐的瓶颈

1. 接触数量：接触多的场景（五指抓取、足式地形）比空旷场景慢得多；
2. 观测生成：相机渲染往往比物理更贵——渲染 30 FPS 意味着每帧预算 33 ms；
3. 数据搬运：CPU 与 GPU 之间来回拷贝；
4. 重置：环境终止后重置的代价。

---

## 6. 传感器仿真

| 传感器 | 仿真方式 | 常加的非理想特性 | 仿真与真实的典型差距 |
|---|---|---|---|
| IMU | 由刚体真值加速度/角速度加噪声 | 白噪声、偏置随机游走、缩放误差 | 真实 IMU 的温漂、振动耦合、安装误差难复现 |
| 编码器/关节 | 真值量化 | 量化、延迟、零偏 | 齿隙、柔性 |
| 2D/3D 激光 | 射线投射 | 距离噪声、丢点、最大/最小量程 | 玻璃/黑色表面、多径、运动畸变 |
| 相机 | 渲染 | 噪声、曝光、畸变、运动模糊 | 光照、材质、镜头、压缩 |
| 深度相机 | 渲染深度图 | 空洞、边缘飞点 | 红外干扰、反光 |
| 接触/力 | 接触求解结果 | 噪声、滤波 | 接触模型的近似 |

**射线数的算术**：16 线 × 360 个方位 $=5760$ 条射线/帧，10 Hz 即 57,600 条/秒；若每条射线的求交成本 $1\ \mu$s（假设），每帧 5.76 ms。这说明传感器仿真的成本会随线数与频率线性增长，要纳入 RTF 预算。

**时间戳问题**：仿真传感器的时间戳应来自仿真时钟，并与 `/clock` 一致。如果让某些节点用墙钟、另一些用仿真钟，就会复现实机里"多源时间不一致"的问题，只不过更难排查（对应 [P6 的时间戳标准化](../../01_项目课程/P6_深开鸿M-Robots平台软件/05_机器人状态管理.md)）。

---

## 7. 与 ROS2 的接入方式

三种常见接口模式：

1. **官方/社区桥接包**：仿真器内部话题（或自己的消息系统）通过桥接节点映射到 ROS2 话题，需要配置映射表（话题名、消息类型、方向）。典型问题：消息类型不匹配、QoS 不兼容（桥接节点的默认 QoS 与订阅端不同）；
2. **仿真器内的 ROS 插件**：仿真世界里的传感器/控制器插件直接是 ROS 节点；
3. **硬件接口抽象**：用 `ros2_control` 的硬件接口（见 [第 14 章](./14_ros2_control与硬件接口实战.md)）把仿真器当作"一块硬件"，控制器代码不关心下面是仿真还是实机。这是**实现"同一套控制代码，仿真与实机切换"最干净的方式**。

无论哪种，务必做到：

- 所有节点 `use_sim_time:=true`，仿真器发布 `/clock`；
- 话题名和坐标系名与实机一致（`base_link`、`odom` 等），这样导航栈的 launch 与参数文件无需修改；
- 在 launch 里用参数/条件区分仿真与实机，而不是复制两套文件。

---

## 8. 域随机化：手算蒙特卡洛

### 8.1 问题

仿真里的参数（摩擦、质量、延迟）总有误差。如果策略/控制器只在一组固定参数下调好，换一个参数就失败。**域随机化**：训练或评测时，在合理范围内随机采样这些参数，使算法对参数不敏感。

### 8.2 什么可以随机化

| 类别 | 参数 | 典型做法 |
|---|---|---|
| 动力学 | 质量、质心、惯量、摩擦、电机增益、阻尼 | 均匀或对数均匀分布，范围取自测量不确定度 |
| 执行器 | 延迟、饱和、带宽 | 随机整数步延迟 |
| 感知 | 噪声、偏置、相机位姿误差、光照 | 在合理范围采样 |
| 环境 | 地面高度、障碍位置、外力扰动 | 程序化生成 |

### 8.3 手算：随机化摩擦对停止距离的影响

物体以初速度 $v_0=2$ m/s 在地面滑行，库仑摩擦系数 $\mu$，停止距离 $d=\dfrac{v_0^2}{2\mu g}$。

**推导**：摩擦减速度 $a=\mu g$（$g=9.81$）；由 $v^2=v_0^2-2ad$，令 $v=0$ 得 $d=v_0^2/(2\mu g)$。

- 名义值 $\mu=0.6$：$d=4/(2\times0.6\times9.81)=0.3398$ m。
- 若 $\mu\sim U(0.4,0.8)$，用 10 万次蒙特卡洛采样（Python 复核）：均值 0.353 m，5% 分位 0.261 m，95% 分位 0.485 m，最大 0.510 m。

假设任务要求停在 0.40 m 以内：

- 名义参数下 0.34 m，通过；
- 随机化后通过率只有 **72.6%**（要求 0.30 m 以内则只有 30.0%）。

**教训**：名义参数下"看起来很好"的方案，在参数不确定下可能只有七成通过率；随机化把这个事实暴露在仿真阶段，而不是实机阶段。随机化范围的选取是工程判断：太窄没效果，太宽会让问题不可解或让策略过于保守。

### 8.4 随机化不是万能

- 随机化只覆盖你想到的参数；没想到的失配（如接触模型本身不同）仍会发生；
- 最后一步仍要用实机数据做**系统辨识**，把名义参数拉近真实值，再随机化。

---

## 9. sim-to-real 差距的来源与缩小办法

| 来源 | 表现 | 办法 |
|---|---|---|
| 动力学参数不准 | 实机比仿真"肉"或"滑" | 系统辨识、随机化 |
| 执行器模型过于理想 | 仿真里力矩瞬时到位，实机有带宽与饱和 | 加电机模型、延迟、限幅 |
| 延迟 | 实机感知—控制—执行有 20–50 ms 级延迟，仿真为 0 | 在仿真里加延迟并随机化 |
| 传感器非理想 | 噪声、漂移、丢帧 | 噪声模型、丢帧注入 |
| 接触模型 | 抓取、足端打滑不一致 | 随机化摩擦与接触参数，保守设计 |
| 时间 | 仿真时钟与控制周期不一致 | 统一控制周期，子步进整除 |
| 视觉 | 渲染与真实图像分布不同 | 视觉随机化、真实数据微调 |

**延迟的一个小算**：控制周期 20 ms（50 Hz），实机端到端延迟 40 ms，意味着命令生效滞后 $0.04\times50=2$ 个控制步。如果仿真里没有这 2 步延迟，在仿真里调好的高增益控制器在实机上可能因延迟引入相位滞后而振荡（参见控制理论模块的频域稳定裕度）。

**参数误差的灵敏度**：若质量被低估/高估 20%，同样力矩下系统固有频率相对变化约 $\sqrt{1/1.2}=0.913$ 或 $\sqrt{1/0.8}=1.118$（Python 复核），即 -8.7% 到 +11.8%，对依赖模型的控制器（LQR、前馈）都有影响，对强反馈的 PID 影响较小。

---

## 10. 选型决策

按"任务"而不是"品牌"选择：

| 任务 | 首选思路 | 理由 |
|---|---|---|
| 控制算法/IK/动力学验证 | MuJoCo 或 Python 自写小仿真 | 轻量、可编程、接触稳定、无需 ROS |
| 导航栈联调（定位、规划、避障） | Gazebo 类 ROS 原生仿真 | 与 ROS2 话题/TF/时钟集成度高，传感器模型够用 |
| 大规模强化学习 | GPU 并行仿真（如 Isaac Lab、MJX 类） | 吞吐是第一指标 |
| 视觉模型训练 / 合成数据 | 高保真渲染平台 | 渲染质量与标注能力 |
| 灵巧/接触丰富操作 | MuJoCo 类（接触质量） | 接触稳定与可调性 |

决策时的检查清单：

1. 我最关心的是**物理保真**、**传感器保真**还是**吞吐**？
2. 目标硬件（有无 NVIDIA GPU）？
3. 是否需要与现有 ROS2 工程无缝对接？
4. 团队学习成本与维护成本？
5. 是否可以先在最便宜的仿真器上验证，再逐级上到更真的？

**一条工程经验**：用多个仿真器做"交叉验证"——在 MuJoCo 里训练、在另一个引擎里评估——是发现"策略在利用某个引擎的物理漏洞"的有效方法（思想性建议，没有统一的量化标准）。

---

## 11. 在七个项目里怎么用（含诚实边界）

| 项目 | 项目里的实际使用 | 与本章的关系 | 性质 |
|---|---|---|---|
| [P2 MuJoCo 与 Mink](../../01_项目课程/P2_未来动力双臂遥操作与差速底盘/03_MuJoCo与Mink入门.md) | CV 写"用 MuJoCo/Mink 求解带约束的逆运动学"，MuJoCo 提供机械臂的数字模型 | 项目用的是 MuJoCo 的**模型/运动学侧**，用作 IK 求解的模型基础；并非把它当完整的接触仿真平台 | 项目做过（IK），完整物理仿真与批量仿真是延伸 |
| [P1 教学仿真实验手册](../../01_项目课程/P1_AIRS球形机器人平衡控制/11_教学仿真实验手册.md) | 用 NumPy 自写非线性倒立摆仿真验证 LQR | 本章 §3 的"步长—稳定性"同样适用于这类小仿真；这属于"轻量自写仿真" | 文档里的教学实验，不是 CV 中的实机数据 |
| [P7 手算与数值验证附录](../../01_项目课程/P7_可变腿长轮腿机器人/13_手算与数值验证附录.md) | 数值验证 LQR/VMC 相关计算 | 同上 | 教学验证 |
| P5 | 导航栈在实车上联调；本文档未声明使用 Gazebo | Gazebo 类仿真适合做 Nav2 联调，是**延伸知识** | **延伸，非项目所做** |
| P6 | 分层验证包含仿真阶段（见 [测试与验证方法](../../01_项目课程/P6_深开鸿M-Robots平台软件/10_测试与验证方法.md)） | 仿真与实机共用同一套能力接口，类似 §7 的"硬件接口抽象" | 项目设计口径，未指明具体仿真器 |
| P3、P4 | 无仿真器使用 | — | — |

> **面试要点**：被问"你用过 Gazebo / Isaac Sim 吗"，诚实回答："I have used MuJoCo as the robot model for constrained inverse kinematics in a dual-arm teleoperation project. I have not run large-scale parallel simulation, but I understand the trade-offs and I would choose the simulator based on the task."

---

## 12. 常见错误与故障现象

| 现象 | 常见原因 | 处理 |
|---|---|---|
| 物体弹飞/仿真爆炸 | 步长大于 $2/\omega$，或惯量/质量设置不合理 | 减小步长，检查惯量、质量比 |
| 机器人在地面抖动 | 接触刚度过大或步长太大 | 调接触参数，增阻尼 |
| 仿真里完美、实机失败 | 理想化的执行器与零延迟 | 加延迟、饱和、噪声并随机化 |
| 导航节点报 TF 外推错误 | 节点未用仿真时钟 | 所有节点 `use_sim_time`，仿真器发布 `/clock` |
| RTF 忽高忽低 | 渲染或激光计算占满 CPU | 降分辨率/线数，分离渲染与物理 |
| 策略在仿真里"作弊" | 利用引擎物理漏洞（穿模、免费能量） | 交叉验证、加入约束与惩罚 |
| URDF 导入后模型不动或塌陷 | 缺惯量、碰撞体或关节限位 | 补全惯量、碰撞几何、限位 |
| 随机化后训练不收敛 | 范围过宽 | 课程式逐步放宽 |

---

## 13. 面试怎么问、怎么答

**Q1. Why does a physics simulator blow up when the time step is too large?**

> A contact is effectively a stiff spring. For a semi-implicit Euler integrator, the system stays stable only if the step is below two over omega, where omega is the square root of stiffness over mass. For a one-kilogram body with a stiffness of 10 to the 5 newtons per meter, that is about 6.3 milliseconds. Above it, the numerical solution grows exponentially. So stiffer contacts need smaller steps, which is why simulators trade off contact hardness against speed.

中文：接触等效于一个刚性弹簧。半隐式欧拉稳定的条件是步长小于 $2/\omega$（$\omega=\sqrt{k/m}$）。1 kg、$10^5$ N/m 时约 6.3 ms；超过后数值解指数增长。刚度越大步长越小，所以仿真器要在接触硬度与速度之间取舍。

*Follow-up: How would you debug an exploding simulation?*

> I would check the step size against the stiffest contact first, then the inertias and mass ratios, then joint limits and initial penetration. I would reduce the step size to see if the explosion goes away. If it does, it is a numerical problem and not a control problem.

中文：先对照最大刚度检查步长，再查惯量与质量比、关节限位和初始穿透；把步长减小看是否消失，若消失则是数值问题而非控制问题。

**Q2. Compare Gazebo, Isaac Sim and MuJoCo. Which would you pick for what?**

> In general, Gazebo integrates tightly with ROS and is good for debugging a navigation stack with sensors and transforms. Isaac Sim runs physics and rendering on the GPU, so it is good for large-scale learning and synthetic image data, but it needs a capable GPU. MuJoCo is light and has stable contact modeling, so it is good for control, inverse kinematics and manipulation research, though I have to build the ROS integration myself. I would pick based on the task and verify speed with a small benchmark on my own robot model.

中文：Gazebo 与 ROS 结合紧，适合调试带传感器和坐标变换的导航栈；Isaac Sim 的物理与渲染都在 GPU 上，适合大规模学习与合成图像数据，但需要较强 GPU；MuJoCo 轻量、接触建模稳定，适合控制、IK、操作研究，但 ROS 集成需自己做。按任务选择，并用自己的机器人模型做小基准测试验证速度。

*Follow-up: Which have you used?*

> MuJoCo, as the robot model for constrained inverse kinematics in a dual-arm teleoperation project. I have not used the other two in my projects.

中文：MuJoCo，用作双臂遥操作项目中带约束逆运动学的机器人模型；其他两个在项目中没用过。

**Q3. What is domain randomization and what are its limits?**

> You randomize simulation parameters such as friction, mass, delays and noise during training or evaluation, so that the controller or policy does not depend on one exact value. For example, if friction varies uniformly between 0.4 and 0.8, a sliding stop that looks fine at the nominal 0.6 passes a 0.4 meter tolerance only about 73 percent of the time. The limits are that it covers only the parameters I thought of, and too wide a range can make the task unsolvable or the policy overly conservative. I would still identify the real system parameters and then randomize around them.

中文：训练/评估时随机化摩擦、质量、延迟、噪声等，让控制器/策略不依赖某个精确值。例如摩擦在 0.4–0.8 均匀变化，名义 0.6 下看起来达标的滑行停止，在 0.4 m 容差内只有约 73% 通过。局限：只覆盖想到的参数，范围太宽会使任务不可解或策略过保守；仍要先辨识真实参数再围绕它们随机化。

**Q4. How do you keep simulation and real hardware code the same?**

> I keep the control code behind a hardware abstraction, for example a hardware interface in ros2_control or a unified capability layer, and let the simulator provide the same interface. Topic names, frames and parameters stay identical, and a launch argument switches between simulation and hardware. The nodes use simulated time in simulation. I also add delay, saturation and noise to the simulated actuators so the timing behavior is similar.

中文：把控制代码放在硬件抽象之后（如 ros2_control 的硬件接口或统一能力层），仿真器提供同样的接口；话题名、坐标系、参数保持一致，用 launch 参数切换仿真/实机；仿真里节点用仿真时钟；并给仿真执行器加延迟、饱和、噪声，使时序行为相近。

**Q5. Your simulation passes but the real robot oscillates. What do you check?**

> First, delay: real systems have tens of milliseconds between sensing and actuation, and a controller tuned with zero delay can lose phase margin. Second, actuator limits and bandwidth. Third, sensor noise and filtering. Fourth, parameter mismatch such as mass and friction. I would add measured delay and saturation to the simulation and see whether it reproduces the oscillation. If it does, I can tune in simulation; if it does not, the missing effect is something I have not modeled yet.

中文：先看延迟（实机感知到执行有几十毫秒，零延迟下调好的控制器会失去相位裕度），其次执行器限幅与带宽，再次传感器噪声与滤波，最后质量、摩擦等参数失配。我会把测得的延迟和饱和加进仿真，看是否复现振荡；若复现就可以在仿真里调，不复现则说明还有没建模的因素。

---

## 14. 本章要点速记

1. 仿真通过不等于实机通过；仿真不过则实机几乎一定有问题。
2. 一次步进：运动学 → 碰撞检测 → 约束/接触求解 → 动力学 → 积分 → 传感器。
3. 接触是难点：软接触（弹簧）简单但需小步长；互补/凸松弛更稳但复杂。
4. 半隐式欧拉稳定条件 $\Delta t\le2/\omega$，$\omega=\sqrt{k/m}$；刚度 ×100，最大步长 ÷10。
5. 显式欧拉对无阻尼振子永远缓慢发散；物理引擎多用半隐式/隐式。
6. 三平台取舍：Gazebo（ROS 集成）、Isaac（GPU 并行与渲染）、MuJoCo（轻量与接触质量）；版本与特性未核对。
7. RTF = 仿真时间/墙钟时间；批量吞吐 = 并行环境数 × 单环境步速。
8. 仿真传感器要加噪声/延迟/丢帧，时间戳用仿真时钟；节点 `use_sim_time`。
9. 域随机化：名义参数下通过 ≠ 随机下通过（示例：摩擦随机后通过率 72.6%）。
10. 缩小 sim-to-real 差距：辨识 + 随机化 + 建模延迟与饱和 + 统一接口；本章内容里，项目实际用到的是 P2 的 MuJoCo（IK 模型侧）。

## 15. 自测题

1. $m=0.5$ kg、$k=2\times10^5$ N/m 的软接触，半隐式欧拉最大稳定步长多少？（答案：$\omega=\sqrt{4\times10^5}=632.5$ rad/s，$2/\omega=3.16$ ms。）
2. 为什么单纯"降低步长"不总是解决仿真不准的办法？（要点：模型本身与现实有差距，步长只解决数值误差。）
3. RTF = 0.5 对导航联调意味着什么？可能造成什么问题？（要点：仿真比真实慢一倍；传感器频率相对墙钟减半，若有节点用墙钟会失配。）
4. 摩擦 $\mu\sim U(0.4,0.8)$，$v_0=2$ m/s，停止距离范围是多少？（答案：最小 $4/(2\times0.8\times9.81)=0.255$ m，最大 $4/(2\times0.4\times9.81)=0.510$ m。）
5. 列出三个"仿真里能用、实机不行"的典型原因，并各给一个仿真侧的改进。

## 16. 相关链接

- 上一章：[09 ROS2 测试与 CI](./09_ROS2测试与CI_launch_testing_gtest_rosbag回放.md)
- 下一章：[11 CMake 与 ament 构建系统](./11_CMake_ament_构建系统与依赖管理.md)
- 相关：[06 MuJoCo、Pinocchio、KDL 工具链](./06_MuJoCo_Pinocchio_KDL_工具链.md)、[ML 09 强化学习进阶](../10_机器学习与深度学习/09_强化学习进阶_策略梯度_PPO_SAC.md)
- 项目：[P2 MuJoCo 与 Mink 入门](../../01_项目课程/P2_未来动力双臂遥操作与差速底盘/03_MuJoCo与Mink入门.md)
- [模块 README](./README.md)、[基础知识总览](../00_总览与学习路线.md)
