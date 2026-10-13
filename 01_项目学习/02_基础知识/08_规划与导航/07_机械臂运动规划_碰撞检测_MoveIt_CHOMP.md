# 机械臂运动规划：配置空间、碰撞检测、MoveIt 与 CHOMP/TrajOpt

> **一句话定位**：机械臂运动规划回答"从当前关节姿态到目标姿态，怎样动才能不撞到自己和环境，并且平滑可执行"。本章先讲配置空间（把机械臂变成一个点），再讲碰撞检测的几何基础（包围体、GJK/FCL 的概念），然后是 MoveIt 2（ROS2 Humble）的规划流水线，最后是 CHOMP、TrajOpt 这类轨迹优化方法，并和 P2 的"约束 IK + 实时遥操作"做区分。

**本章学完你能：**

- 解释配置空间 $\mathcal C$ 与工作空间的区别，手画/手算 2 连杆臂的配置空间障碍；
- 说明为什么工作空间里的一个小障碍在配置空间里会变成扭曲的大区域，以及"关节空间直线不一定无碰撞"；
- 描述碰撞检测的层次：包围体粗筛、凸体精确检测、距离查询；理解 GJK 的"闵可夫斯基差包含原点"思想；
- 画出 MoveIt 2 的规划流水线：规划场景、规划请求、规划器（OMPL/Pilz/CHOMP）、后处理（时间参数化）、执行；
- 写出 CHOMP 的目标函数（平滑项 + 障碍项），手算一步泛函梯度更新，解释度量矩阵 $A$ 的作用；
- 对比 OMPL 采样规划与轨迹优化的取舍，以及它们如何组合；
- 说清规划（planning）、伺服（servoing）与约束 IK 的分工，并将其与 P2 对应；
- 用英文回答 "What is configuration space / How does MoveIt plan / CHOMP vs RRT / How do you check collisions fast"。

**前置知识**：[采样规划：RRT、RRT* 与 PRM](./06_采样规划_RRT_RRT星_PRM.md)、[正逆运动学与雅可比](../03_运动学与坐标变换/03_正逆运动学与雅可比.md)、[奇异性、阻尼最小二乘与零空间](../03_运动学与坐标变换/04_奇异性_阻尼最小二乘_零空间.md)、[最优化与二次规划](../01_数学基础/04_最优化与二次规划.md)、[URDF、TF 与坐标系实践](../12_机器人软件/07_URDF_TF_坐标系实践.md)。

## 目录

1. 机械臂运动规划要解决什么问题
2. 符号与单位
3. 配置空间与配置空间障碍（含手算）
4. 碰撞检测的几何
5. GJK 的思想（含手算）
6. MoveIt 2 的规划流水线
7. OMPL 规划器与后处理
8. 轨迹优化：CHOMP（含手算）
9. TrajOpt 与 STOMP
10. 时间参数化（含手算）
11. 规划、伺服与约束 IK 的分工
12. 在本项目里怎么用
13. 常见错误与故障现象
14. 面试怎么问、怎么答
15. 本章要点速记
16. 自测题
17. 相关链接

---

## 1. 机械臂运动规划要解决什么问题

给定机械臂的起始关节角 $\mathbf q_s$ 与目标（可能是目标关节角，也可能是末端位姿，后者需要先做 IK），需要一条关节空间轨迹 $\mathbf q(t)$，满足：

1. **无碰撞**：机械臂的所有连杆既不与环境（桌子、物体）碰撞，也不与自身（双臂之间、手臂与本体）碰撞；
2. **满足限位**：关节角、速度、加速度（有时还有力矩）都在范围内；
3. **平滑可执行**：速度/加速度连续，没有突变；
4. （可选）**满足任务约束**：如保持杯子竖直，末端沿直线运动。

和移动机器人的规划相比，机械臂规划有三个特点：

- **维度高**：6–7 个关节，双臂 14 个，加上腰部和夹爪更高，栅格搜索不可行（见 [采样规划](./06_采样规划_RRT_RRT星_PRM.md)）；
- **碰撞检测复杂**：机械臂是多个刚体连杆，形状复杂，需要精确几何；
- **目标常给在任务空间**：如"夹爪到这个位姿"，而规划发生在关节空间，需要 IK 把两者连接起来。

> **提示**：口头回答时，"规划"通常指离线（或在线但非实时）的全局路径/轨迹生成，而 P2 的 VR 遥操作是"每个控制周期都实时算一次 IK"的**伺服**方式——两者目标不同，不能混为一谈。

## 2. 符号与单位

| 符号 | 含义 | 单位 / 说明 |
|---|---|---|
| $\mathbf q\in\mathbb R^n$ | 关节角向量，$n$ 为自由度 | rad |
| $\mathcal C$ | 配置空间（所有可能的 $\mathbf q$） | 维度 $n$ |
| $\mathcal W$ | 工作空间（三维物理空间） | m |
| $\mathcal O$ | 工作空间中的障碍物 | 几何体 |
| $\mathcal C_{\text{obs}}=\{\mathbf q:\ \text{Arm}(\mathbf q)\cap\mathcal O\neq\emptyset\}$ | 配置空间障碍 | $\mathcal C$ 的子集 |
| $\boldsymbol\xi=(\mathbf q_1,\dots,\mathbf q_N)$ | 离散化轨迹，$N$ 个路点 | rad |
| $F_{\text{smooth}},\ F_{\text{obs}}$ | CHOMP 的平滑代价、障碍代价 | 无量纲 |
| $A$ | 有限差分度量矩阵 | $K^\top K$ |
| $d(\mathbf x)$ | 到最近障碍物的有符号距离 | m |
| $\lambda$ | 障碍项权重 | 无量纲 |
| $\dot{\mathbf q}_{\max},\ \ddot{\mathbf q}_{\max}$ | 速度、加速度上限 | rad/s，rad/s² |

## 3. 配置空间与配置空间障碍（含手算）

### 3.1 配置空间

**配置空间**把机械臂在某时刻的整个姿态压缩成一个点 $\mathbf q$。这样"一个有形状的多连杆机器人在障碍物中移动"就被转化为"**一个点**在 $\mathcal C$ 里避开 $\mathcal C_{\text{obs}}$"。规划的统一框架（包括上一章的 RRT/PRM）都建立在这个转换之上。

### 3.2 手算：2 连杆平面臂

示例（非项目实测）：平面 2 连杆臂，$l_1=l_2=1$ m，基座在原点，关节角 $(q_1,q_2)$，$q_1\in[0^\circ,180^\circ]$，$q_2\in[-150^\circ,150^\circ]$。工作空间里有一个圆形障碍，圆心 $(1,1)$，半径 0.35 m。

正运动学：肘部 $\mathbf p_1=(\cos q_1,\sin q_1)$，末端 $\mathbf p_2=\mathbf p_1+(\cos(q_1+q_2),\sin(q_1+q_2))$。碰撞判定：两个连杆线段到圆心的最近距离是否 $<0.35$。

按 $q_1$ 步长 $15^\circ$、$q_2$ 步长 $30^\circ$ 扫描，得到配置空间栅格（`#` 表示碰撞），Python 复核：

```text
q2\q1    0   15  30  45  60  75  90 105 120 135 150 165 180
 -90     .   .   .   .   .   #   #   .   .   .   .   .   .
 -60     .   .   .   .   #   #   .   .   .   .   .   .   .
 -30     .   .   .   #   #   .   .   .   .   .   .   .   .
   0     .   .   .   #   .   .   .   .   .   .   .   .   .
  30     .   .   #   #   .   .   .   .   .   .   .   .   .
  60     .   #   #   .   .   .   .   .   .   .   .   .   .
  90     #   #   .   .   .   .   .   .   .   .   .   .   .
 其余行（q2 = -150, -120, 120, 150）无碰撞
```

143 个格子中 13 个碰撞（约 9%）。这个例子给出三个重要结论：

1. 工作空间里一个半径 0.35 m 的小圆，在配置空间里变成一条**斜向的带状区域**，形状扭曲；
2. 这条带状区域横在许多关节空间的直线路径上：想从带的一侧走到另一侧，路径必须从带的端点绕过去（或者穿过），对应手臂以不同构型从障碍的上方或下方绕过，这就是路径的不同**拓扑类别**；
3. **关节空间的直线不一定无碰撞，即使两端都无碰撞**：从 $(0^\circ,60^\circ)$ 到 $(90^\circ,-60^\circ)$ 的直线插值（19 个点），中间第 5 至第 15 个点都碰撞；从 $(0^\circ,60^\circ)$ 到 $(90^\circ,60^\circ)$ 的直线也在第 3 至第 9 个点碰撞（Python 复核）。所以必须规划，而不能简单插值。

### 3.3 为什么难

配置空间障碍的形状无法解析写出来，只能"给一个 $\mathbf q$，查一次碰撞"。这是采样规划成立的前提。维度越高，$\mathcal C_{\text{obs}}$ 越难表达：示例（非项目实测）7 维空间每维 50 格，网格有 $50^7=7.8\times10^{11}$ 个格子（Python 复核）。

## 4. 碰撞检测的几何

机械臂碰撞检测占规划时间的大部分，所以要高效。典型层次：

| 层次 | 方法 | 作用 |
|---|---|---|
| 1. 宽相（broad phase） | AABB（轴对齐包围盒）树、BVH（包围体层次） | 快速排除明显不相交的物体对 |
| 2. 窄相（narrow phase） | 凸体之间用 GJK 判相交；网格用三角形相交测试 | 精确判断是否碰撞 |
| 3. 距离/穿透查询 | GJK + EPA，或符号距离场 | 需要距离或穿透深度（用于优化） |

**形状的近似**：精确三角网格慢，常见做法是用**球体组合**（sphere decomposition）或**凸包**近似连杆，既快又保守。CHOMP 一类优化方法就特别喜欢"球体 + 符号距离场"，因为距离和梯度能 $O(1)$ 查询。

**自碰撞**：机械臂连杆之间也会撞。相邻连杆天然接触，需要忽略；MoveIt 通过 SRDF 里的**允许碰撞矩阵**（Allowed Collision Matrix, ACM）存储哪些连杆对永远不会/允许碰撞，从而减少需要检查的对数。双臂机器人要特别注意两臂之间和臂与躯干之间的检查。

**FCL**（Flexible Collision Library，Pan 等，2012，细节未核对）是 MoveIt 中常用的碰撞检测库；Bullet 等也可使用。

## 5. GJK 的思想（含手算）

GJK（Gilbert–Johnson–Keerthi）判断两个**凸**体是否相交，核心思想是：

**两个凸集 $A,B$ 相交，当且仅当它们的闵可夫斯基差 $A\ominus B=\{\mathbf a-\mathbf b\}$ 包含原点。**

直觉：若存在公共点 $\mathbf x$，那么 $\mathbf x-\mathbf x=\mathbf 0$ 就属于 $A\ominus B$。GJK 不显式构造整个闵可夫斯基差（顶点数是 $|A||B|$），而是只通过**支撑函数**（support function）按需取点：

$$
\text{support}_{A\ominus B}(\mathbf d)=\text{support}_A(\mathbf d)-\text{support}_B(-\mathbf d)
$$

支撑函数 $\text{support}_A(\mathbf d)$ 返回 $A$ 沿方向 $\mathbf d$ 最远的顶点。算法逐步构造一个单纯形（2D 是线段/三角形，3D 是四面体），不断朝原点方向取新支撑点，若单纯形包含原点则相交；若新支撑点在搜索方向上没有越过原点则不相交。

### 手算

示例（非项目实测）：方块 $A=[0,2]\times[0,2]$ 和 $B=[1.5,3.5]\times[1.5,3.5]$。

- 沿 $\mathbf d=(1,0)$：$\text{support}_A=(2,0)$ 或 $(2,2)$（取其一，如 $(2,0)$），$\text{support}_B(-\mathbf d)$ 取 $B$ 中 $x$ 最小的顶点，如 $(1.5,1.5)$，差为 $(0.5,-1.5)$（Python 复核）；
- 闵可夫斯基差是以原点附近为中心的一个正方形 $[-3.5,0.5]^2$，其 $x$ 范围含 0、$y$ 范围含 0，所以**包含原点**，两方块相交；
- 穿透深度：原点到闵可夫斯基差边界的最短距离，$\min_{\mathbf d}\max(\text{support}\cdot\mathbf d)=0.5$，即沿 $x$ 或 $y$ 轴方向推 0.5 m 可以分开（Python 复核）。

若把 $B$ 平移 $(2,2)$，闵可夫斯基差整体平移，原点落在外面，不相交。

GJK 的优点是：收敛通常只需要少数几次迭代，复杂度对顶点数几乎是线性的，且可以同时得到最近距离（不相交时）。EPA（Expanding Polytope Algorithm）在相交时用来求穿透深度。凹体需要分解成凸块。

## 6. MoveIt 2 的规划流水线

MoveIt 2 是 ROS2（Humble）下事实上的机械臂操作框架。其中的主要概念：

```mermaid
flowchart LR
  A[规划请求 起点 目标 约束] --> B[move_group]
  B --> C[规划场景 PlanningScene]
  B --> D[规划管线 Planning Pipeline]
  D --> E[请求适配器]
  E --> F[规划器 OMPL Pilz CHOMP]
  F --> G[响应适配器 时间参数化]
  G --> H[轨迹 JointTrajectory]
  H --> I[控制器执行]
```

- **规划场景（Planning Scene）**：机器人模型（URDF/SRDF）+ 当前关节状态 + 环境障碍物（来自传感器八叉树、手动添加的几何体）+ 允许碰撞矩阵。所有碰撞检测都基于场景。
- **move_group**：核心节点，接收规划请求，调用规划管线，返回轨迹并可发给控制器执行。
- **规划管线**：请求适配器（修正起始状态超限等）→ 规划器插件 → 响应适配器（时间参数化、平滑）。
- **规划器插件**：OMPL（采样规划库，含 RRTConnect、RRT*、PRM 等，MoveIt 中常见默认为 RRTConnect 一类，细节未核对）、Pilz 工业运动规划器（PTP、LIN、CIRC 确定性运动）、CHOMP、STOMP 等。
- **运动学求解器插件**：KDL（默认）、TRAC-IK、IKFast 等，把末端位姿目标转为关节目标。
- **执行**：生成的 `JointTrajectory` 通过 `ros2_control` 的控制器（如 JointTrajectoryController）下发到硬件。

> **注意**：不要说 MoveIt "是一个规划算法"。它是一个**框架**：整合了运动学、碰撞检测、规划器、时间参数化、执行接口，规划器只是其中的可替换插件。

## 7. OMPL 规划器与后处理

OMPL（Open Motion Planning Library，Şucan 等，2012，细节未核对）提供一批采样规划器，输入：配置空间的状态有效性检查器（碰撞检测）+ 起点 + 目标；输出：路径（路点序列）。

OMPL 的路径特点（与上一章相同）：随机、锯齿、不最优，没有时间信息。因此 MoveIt 在 OMPL 后面接：

1. **路径简化**（shortcut、去冗余路点）；
2. **时间参数化**（如 Time-Optimal Trajectory Generation，TOTG；或简单的迭代抛物线时间参数化，细节未核对），根据速度、加速度限位给每个路点分配时间；
3. 必要时的**平滑**。

对目标有位姿约束（如末端保持水平）时，OMPL 使用**约束采样**：投影采样点到约束流形上，复杂度显著增加，所以能用就尽量用确定性规划器（Pilz）或把约束放到优化里。

## 8. 轨迹优化：CHOMP（含手算）

### 8.1 想法

采样规划先找一个可行但很难看的路径，再靠后处理变平滑。CHOMP（Covariant Hamiltonian Optimization for Motion Planning，Ratliff 等，2009，细节未核对）反过来：**直接把一条初始轨迹（常是起终点之间的直线，哪怕它穿过障碍）作为优化变量，通过梯度下降把它推出障碍并保持平滑**。

### 8.2 目标函数

把轨迹离散成 $N$ 个路点 $\boldsymbol\xi=(\mathbf q_1,\dots,\mathbf q_N)$（起终点固定）。总代价：

$$
U(\boldsymbol\xi)=F_{\text{obs}}(\boldsymbol\xi)+\lambda\,F_{\text{smooth}}(\boldsymbol\xi)
$$

平滑项惩罚加速度或速度（有限差分）：

$$
F_{\text{smooth}}=\tfrac12\sum_{i}\lVert\mathbf q_{i+1}-\mathbf q_i\rVert^2=\tfrac12\boldsymbol\xi^\top A\,\boldsymbol\xi+\mathbf b^\top\boldsymbol\xi+c
$$

其中 $A=K^\top K$ 是由一阶差分矩阵 $K$ 构成的**三对角**矩阵（对角线 2，次对角线 −1）。障碍项把机器人身上的采样点 $\mathbf x$ 对障碍物的代价沿轨迹积分，常用由距离场 $d(\mathbf x)$ 得到的势：

$$
c(\mathbf x)=\begin{cases}\tfrac{1}{2\varepsilon}\bigl(\varepsilon-d(\mathbf x)\bigr)^2&0<d\le\varepsilon\\ -d+\tfrac{\varepsilon}2&d\le0\\0&d>\varepsilon\end{cases}
$$

这里 $\varepsilon$ 是安全距离。距离越小代价越大，在障碍内部线性增长。注意代价是距离的函数，所以需要**符号距离场**（见 [建图表示：ESDF](../07_SLAM与建图/06_建图表示_栅格_八叉树_NDT_TSDF_ESDF.md)）。

### 8.3 协变更新

普通梯度下降 $\boldsymbol\xi\leftarrow\boldsymbol\xi-\eta\nabla U$ 只改动梯度大的路点，会产生"尖刺"。CHOMP 用**度量 $A$ 下的协变梯度**：

$$
\boldsymbol\xi\leftarrow\boldsymbol\xi-\eta\,A^{-1}\nabla U
$$

$A^{-1}$ 起到平滑算子的作用：一个路点上的梯度会通过 $A^{-1}$ 分散到相邻路点，整条轨迹**平滑地**变形，而不是只拉扯一个点。这也是名字里"协变"（covariant）的含义。

### 8.4 手算

示例（非项目实测）：2D 空间的 5 个路点，端点固定 $\mathbf q_0=(0,0)$、$\mathbf q_4=(4,0)$，内部三点 $(1,0.1),(2,0.0),(3,0.1)$。障碍圆心 $(2,0.3)$，影响半径 $R=0.8$，障碍项取简化势 $c(\mathbf x)=(R-d)^2$（$d<R$），梯度 $\nabla c=-2(R-d)\frac{\mathbf x-\mathbf x_o}{d}$，权重 3。

**距离**：中点 $\mathbf q_2=(2,0)$ 离障碍 $d=0.3<R$，两侧路点距离 1.020 $>R$，不受力。

障碍项梯度（中点）：$-2(0.8-0.3)\frac{(0,-0.3)}{0.3}=(0,\,1.0)$，乘权重 3 得 $(0,3.0)$。

平滑项梯度 $2(2\mathbf q_i-\mathbf q_{i-1}-\mathbf q_{i+1})$（对内部三点）：$y$ 分量分别为 $0.4,\,-0.4,\,0.4$。合并后总梯度的 $y$ 分量：

$$
G_y=(0.4,\ \ 2.6,\ \ 0.4)
$$

（Python 复核。）取 $A=\begin{bmatrix}2&-1&0\\-1&2&-1\\0&-1&2\end{bmatrix}$，$\eta=0.3$，协变更新 $\Delta\mathbf q=-\eta A^{-1}G$：

$$
\Delta y=(-0.51,\ -0.90,\ -0.51)
$$

新轨迹 $y$ 坐标：$(-0.41,\ -0.90,\ -0.41)$。更新后三个路点到障碍的距离为 $1.226,\ 1.200,\ 1.226$ m，均大于 $R=0.8$，障碍项为零。如果用普通欧氏梯度下降（不乘 $A^{-1}$），则中点下移 0.234，而两侧路点只动 0.04 左右，轨迹出现"尖刺"，而且不一定足以离开影响范围。**协变更新把一个点上的推力分摊到了相邻的点**，一次迭代就得到平滑的绕行。

### 8.5 局限

- **局部最优**：只在初始轨迹所属的拓扑（障碍的哪一侧）里优化，不能跳到另一侧；
- 需要距离场及其梯度；
- 初始轨迹穿透太深时，梯度可能不够有效。

所以常把 OMPL 当作"拓扑选择器"，用其结果作为 CHOMP 的初值，两类方法组合使用。

## 9. TrajOpt 与 STOMP

| 方法 | 思想 | 特点 |
|---|---|---|
| CHOMP | 协变梯度下降，需要距离场与梯度 | 平滑、速度快；局部最优；需要可微代价 |
| TrajOpt（Schulman 等，2014，细节未核对） | 序列凸优化：每次迭代把非线性约束线性化，用凸碰撞检测（符号距离或扫掠体积）并用信赖域求解 | 可处理硬约束（无碰撞作为不等式约束）；对约束友好 |
| STOMP（Kalakrishnan 等，2011，细节未核对） | 随机采样噪声轨迹，按代价加权平均，不需要梯度 | 不需要可微代价，可用于不可微代价；需要多次采样 |

选择建议：要处理任意（不可微）代价函数 → STOMP；有硬约束且需要快速收敛 → TrajOpt；追求简单与平滑 → CHOMP；复杂拓扑或杂乱环境 → 先用采样规划找初值。

## 10. 时间参数化（含手算）

规划器给出的是几何路径 $\mathbf q(s)$，还需要速度曲线 $s(t)$，使 $\dot{\mathbf q}$、$\ddot{\mathbf q}$ 都不超限。最简单的**梯形速度曲线**（加速—匀速—减速）：

- 加速段时间 $t_a=v_{\max}/a_{\max}$，距离 $d_a=\tfrac12a_{\max}t_a^2$；
- 若总行程 $L\ge2d_a$，总时间 $T=2t_a+(L-2d_a)/v_{\max}$；
- 若 $L<2d_a$，三角形速度曲线，$T=2\sqrt{L/a_{\max}}$。

示例（非项目实测）：行程 $L=2$ rad，$v_{\max}=1$ rad/s，$a_{\max}=2$ rad/s²：$t_a=0.5$ s，$d_a=0.25$ rad，$2d_a=0.5$ rad $\le L$，匀速段 $(2-0.5)/1=1.5$ s，$T=1.0+1.5=2.5$ s（Python 复核）。

多关节时，必须让所有关节**同步**到达（沿同一条路径缩放），由受限最严的关节决定整体速度。更高级的 TOTG 在路径上做时间最优的积分，考虑每个点上的加速度极限。

## 11. 规划、伺服与约束 IK 的分工

| 场景 | 方法 | 特点 | 典型目标 |
|---|---|---|---|
| 抓取—放置、避障取物 | 运动规划（OMPL+时间参数化，或 CHOMP） | 离线/准实时，全局避障，起止点已知 | 到达某个目标并避碰 |
| 末端笛卡尔直线、工艺轨迹 | 笛卡尔规划（Pilz LIN/CIRC，逐点 IK） | 确定性，保证末端路径形状 | 焊接、涂胶 |
| 实时遥操作、跟随 | 伺服（MoveIt Servo 或自研 IK 回路） | 每周期一次，局部、反应式，不做全局规划 | 跟随 VR 手柄、示教 |
| 带约束的实时跟踪 | 约束 IK（QP），如 Mink | 在每个周期内加入关节限位、速度限幅、碰撞距离约束 | 遥操作的安全约束 |

关键区分：**规划是全局的、可预见未来的；伺服是局部的、每周期只看当前**。伺服容易陷入局部极小（比如被一个凹障碍卡住），也没有"绕路"能力，但延迟低、响应快。

## 12. 在本项目里怎么用

- **P2 未来动力（VR 双臂遥操作）**：P2 的做法属于第 11 节里的"**伺服 + 约束 IK**"——把 VR 手柄运动映射成末端目标，用 MuJoCo/Mink 求解带约束的逆运动学，并抑制奇异点附近的速度放大。它**没有使用 MoveIt 或 OMPL 做全局运动规划**，也没有做环境避障规划。本章的 MoveIt/CHOMP 是延伸知识：被问到"如果要让机械臂自主完成抓取，你会怎么做"时，可以回答：在 P2 现有的约束 IK 之上，加规划层：用 MoveIt 2 + OMPL 找无碰撞路径，再用 CHOMP/TrajOpt 平滑，最后用时间参数化交给 ROS2-CAN 执行链。见 [P2 逆运动学、雅可比、奇异性与阻尼最小二乘](../../01_项目课程/P2_未来动力双臂遥操作与差速底盘/04_逆运动学_雅可比_奇异性与阻尼最小二乘.md) 与 [约束与限位](../../01_项目课程/P2_未来动力双臂遥操作与差速底盘/05_约束与限位.md)。
- **P2 的 URDF/KDL**：碰撞检测与允许碰撞矩阵依赖于 URDF/SRDF；P2 的 URDF/KDL 动力学建模（见 [URDF_KDL动力学与重力补偿](../../01_项目课程/P2_未来动力双臂遥操作与差速底盘/06_URDF_KDL动力学与重力补偿.md)）与规划场景使用同一份机器人模型。
- **P1**：双臂按固定轨迹运动，不做规划；但它的平衡控制要考虑手臂运动带来的质心变化（见 [P1 README](../../01_项目课程/P1_AIRS球形机器人平衡控制/README.md)）。
- **P6**：任务规划不直接控制底层，计划经过策略检查才能执行；这与 MoveIt "规划得到轨迹，再经执行器与限位检查" 的分层精神一致。见 [任务规划与实时控制的分层](../../01_项目课程/P6_深开鸿M-Robots平台软件/07_任务规划与实时控制的分层.md)。

诚实表述：我在 P2 里做的是实时约束 IK 与遥操作，没有部署 MoveIt 做运动规划；我了解 MoveIt 的流水线、采样规划与轨迹优化的区别，也能说明如何在现有基础上扩展。

## 13. 常见错误与故障现象

| 现象 | 原因 | 处理 |
|---|---|---|
| 规划成功但真机碰撞 | 规划场景与真实环境不一致；碰撞几何过于粗糙 | 同步传感器八叉树；检查 URDF 碰撞体 |
| 规划总是失败 | 起始状态本身碰撞/超限；目标 IK 无解 | 检查起始状态有效性；换 IK 种子 |
| 在关节空间直线插值导致碰撞 | 认为两端无碰撞则中间也无碰撞 | 必须做边碰撞检测 |
| 规划速度很慢 | 碰撞检测用了精细网格；ACM 未配置 | 球体/凸包近似；配置允许碰撞矩阵 |
| 轨迹有急拐、抖动 | 忘了后处理/时间参数化；路径锯齿 | 简化+平滑+时间参数化 |
| CHOMP 不能绕过大障碍 | 局部最优，初值在障碍"错误一侧" | 用 OMPL 路径作为初值 |
| 两臂之间碰撞 | 自碰撞检查缺少臂间对 | 在 SRDF 中补全碰撞对，别误禁用 |
| 每次规划路径都不同 | 采样规划随机 | 固定种子；或使用确定性规划器（Pilz） |
| 约束规划失败率高 | 约束流形上采样困难 | 放松约束；把约束放进优化或逐点 IK |
| 执行时速度突变 | 关节限位参数与控制器不一致 | 统一 `joint_limits` 与控制器配置 |

## 14. 面试怎么问、怎么答

**Q1. What is configuration space and why is it useful?**

> Configuration space is the space of all joint configurations. Each point represents a whole arm pose, so planning for a robot with a complicated shape turns into planning for a single point that must avoid the configuration-space obstacles. It lets the same algorithms, such as RRT or PRM, work for any robot as long as I can answer a collision query for a given configuration. The catch is that the obstacle region in this space is distorted and cannot be written down analytically.

中文：配置空间是所有关节配置的空间，一个点代表整个手臂姿态，规划转成"点避障"，RRT/PRM 通用；难点是配置空间障碍形状扭曲，无法解析表达。追问：为什么关节空间直线不一定无碰撞？（配置空间障碍是非凸的弯曲区域。）

**Q2. Walk me through how MoveIt plans a motion.**

> A planning request with the start state, goal and constraints goes to the move_group node. It builds a planning scene from the robot model, the current joint state and the obstacles. The request goes through a pipeline: adapters first fix the request, then a planner plugin such as OMPL RRTConnect searches the configuration space using the collision checker, and then response adapters shorten the path and time-parameterise it with velocity and acceleration limits. The resulting joint trajectory is executed by a controller through ros2_control.

中文：请求发给 move_group，建规划场景；管线中适配器修正请求，规划器插件（如 OMPL RRTConnect）搜索，响应适配器简化并时间参数化；轨迹经 ros2_control 控制器执行。

**Q3. What is the difference between sampling-based planning and trajectory optimisation like CHOMP?**

> Sampling-based planners such as RRTConnect are global and probabilistically complete, so they can find a path in cluttered spaces, but the result is jagged and needs smoothing. CHOMP starts from an initial trajectory and uses gradient descent on a cost that combines smoothness and obstacle distance, so it produces smooth paths quickly, but it is local and can get stuck on the wrong side of an obstacle. In practice I combine them: sample to choose the homotopy class, then optimise.

中文：采样规划全局、概率完备但路径锯齿；CHOMP 对初始轨迹做梯度下降，平滑快但局部最优，可能卡在障碍错误一侧；实践中组合：先采样确定拓扑，再优化。

**Q4. How does GJK detect a collision between two convex shapes?**

> Two convex shapes intersect exactly when their Minkowski difference contains the origin. GJK never builds the full difference. It uses the support function to pick extreme points of the difference in chosen directions, builds a small simplex, and keeps moving the search direction toward the origin until either the simplex encloses the origin, meaning collision, or a new support point cannot pass the origin, meaning no collision.

中文：两凸体相交当且仅当闵可夫斯基差包含原点；GJK 用支撑函数按需取点，构造单纯形，朝原点方向迭代，包含原点则碰撞，新点无法越过原点则不碰撞。

**Q5. What is the difference between motion planning and servoing, and which one did you use?**

> Planning looks ahead: it finds a collision-free trajectory from start to goal in advance. Servoing is reactive: every control cycle it computes the next joint command from the current target, so it is fast and smooth but local and cannot go around obstacles. In my dual-arm teleoperation project I used the second approach, solving constrained inverse kinematics at every cycle from the VR controller pose, with joint limits and singularity damping, rather than a MoveIt planner.

中文：规划是预先找无碰撞轨迹；伺服每周期用当前目标算下一步，快而平滑但局部、不会绕障碍。我在双臂遥操作里用的是后者：每周期求约束 IK，加关节限位和奇异性阻尼，没有用 MoveIt 规划。

## 15. 本章要点速记

1. 配置空间把机械臂变成一个点，规划就是点在 $\mathcal C_{\text{free}}$ 里找路径。
2. 工作空间里的小障碍在配置空间里变成扭曲的带状区域；两端无碰撞，关节空间直线可能碰撞。
3. 碰撞检测分宽相（AABB/BVH）与窄相（GJK/网格），球体近似快；自碰撞用允许碰撞矩阵筛。
4. GJK：两凸体相交 ⟺ 闵可夫斯基差含原点；用支撑函数 $\text{support}_A(\mathbf d)-\text{support}_B(-\mathbf d)$。
5. MoveIt 2 是框架：规划场景 + 管线（适配器/规划器/时间参数化）+ 控制器执行；规划器和 IK 求解器是插件。
6. OMPL 输出锯齿路径，需要简化、平滑、时间参数化。
7. CHOMP：$U=F_{\text{obs}}+\lambda F_{\text{smooth}}$，协变更新 $\boldsymbol\xi\leftarrow\boldsymbol\xi-\eta A^{-1}\nabla U$，$A$ 起平滑作用。
8. TrajOpt 处理硬约束，STOMP 不需要梯度；优化是局部的，需要好的初值。
9. 梯形速度曲线：$T=2t_a+(L-2d_a)/v_{\max}$；多关节同步由最严的关节限制。
10. P2 是伺服 + 约束 IK，不是 MoveIt 规划；要诚实区分。

## 16. 自测题

1. 4 自由度臂每个关节离散 20 格，配置空间格子数？（$20^4=160000$。）
2. 判断题：两端配置都无碰撞，则关节空间直线插值一定无碰撞。（错；配置空间障碍可以夹在中间。）
3. 方块 $A=[0,1]^2$、$B=[0.5,1.5]^2$，闵可夫斯基差 $A\ominus B$ 的范围？（$x,y\in[-1.5,0.5]$，包含原点，相交，穿透深度 0.5。）
4. 行程 1 rad，$v_{\max}=2$ rad/s，$a_{\max}=2$ rad/s²，时间多少？（$d_a=\tfrac12\cdot2\cdot1^2=1$，$2d_a=2>L=1$，三角形曲线，$T=2\sqrt{1/2}=1.414$ s。）
5. CHOMP 里为什么用 $A^{-1}\nabla U$ 而不是 $\nabla U$？（$A^{-1}$ 把单点的梯度平滑地分散到相邻路点，避免尖刺，使整条轨迹光滑变形。）

## 17. 相关链接

- 上一章：[采样规划：RRT、RRT* 与 PRM](./06_采样规划_RRT_RRT星_PRM.md)
- 下一章：[行为树与任务规划：PDDL 与有限状态机](./08_行为树与任务规划_PDDL_有限状态机.md)
- 基础：[正逆运动学与雅可比](../03_运动学与坐标变换/03_正逆运动学与雅可比.md)、[机械臂动力学、重力补偿、URDF 与 KDL](../03_运动学与坐标变换/06_机械臂动力学_重力补偿_URDF_KDL.md)
- 项目：[P2 未来动力双臂遥操作 README](../../01_项目课程/P2_未来动力双臂遥操作与差速底盘/README.md)
- 模块说明：[08_规划与导航 README](./README.md)
