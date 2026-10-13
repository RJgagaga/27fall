# 多机器人路径规划：MAPF、优先级规划、编队与一致性控制

> **一句话定位**：一台机器人规划路径是"从 A 到 B 避开障碍"；多台机器人在同一空间里同时规划，多了一个新障碍——**彼此**。本章讲多智能体路径规划（Multi-Agent Path Finding, MAPF）的冲突类型、联合状态空间的指数爆炸、优先级规划与冲突基搜索（CBS），再讲编队控制与一致性协议（consensus），以及通信受限（延迟、丢包）下要注意什么，最后和 P6 多机器人平台的协同工作做诚实对应。

**本章学完你能：**

- 区分顶点冲突、边（交换）冲突、跟随冲突，并说明为什么联合状态空间不能直接搜索；
- 解释优先级规划（prioritized planning）的做法、优缺点与失败例子；
- 描述 CBS 的两层结构（高层冲突树 + 低层单机 A*），手算一个双机交换位置的小例子并说清楚最终代价；
- 用图拉普拉斯写出一致性协议，手算几步迭代，说明收敛速度由第二小特征值 $\lambda_2$ 决定、步长上界由最大特征值决定；
- 区分领航—跟随、虚拟结构、基于一致性的编队，并写出带偏移的编队一致性；
- 说明通信延迟、丢包、拓扑切换对多机协同的影响，以及"分布式 vs 集中式"的选择；
- 用英文回答 "What is MAPF / How does CBS work / What is consensus and what determines its convergence rate / How would you coordinate multiple robots in a warehouse"。

**前置知识**：[图搜索：Dijkstra 与 A*](./01_图搜索_Dijkstra_A星.md)、[信息论与图论基础](../01_数学基础/07_信息论与图论基础.md)、[线性代数](../01_数学基础/01_线性代数.md)（特征值）、[行为树与任务规划](./08_行为树与任务规划_PDDL_有限状态机.md)、[DDS 与分布式通信](../12_机器人软件/05_DDS与分布式通信.md)。

## 目录

1. 多机器人规划要解决什么问题
2. 符号与单位
3. 冲突的类型
4. 联合状态空间与复杂度
5. 解耦方法：优先级规划
6. 冲突基搜索 CBS（含手算）
7. 终身 MAPF 与交通规则
8. 编队：领航—跟随与虚拟结构
9. 一致性协议（含手算）
10. 编队一致性（含手算）
11. 通信约束：延迟、丢包与拓扑变化
12. 集中式与分布式的取舍
13. 在本项目里怎么用
14. 常见错误与故障现象
15. 面试怎么问、怎么答
16. 本章要点速记
17. 自测题
18. 相关链接

---

## 1. 多机器人规划要解决什么问题

典型场景：

- **仓储**：几十到上千台搬运机器人在同一张地图上同时运送货架，路径经常交汇；
- **集群协同**：多台无人机/地面车共同覆盖搜索区域、保持队形；
- **多机操作**：几台机械臂在同一工作台上，工作空间重叠。

单机规划器（A*、Hybrid A*、TEB）把别的机器人当作"动态障碍物"，只能被动反应，容易**死锁**（两台机器人在窄道迎面互相等待）或**活锁**（反复让来让去）。多机规划的目标是**事先协调**，保证所有机器人都能无冲突到达，同时让总代价（所有人用时之和 sum of costs）或完工时间（makespan）尽量小。

```text
单机:  机器人 ──规划──> 路径 ──> 避障控制
多机:  所有机器人 ──联合规划/协调──> 无冲突路径集 ──> 各自避障控制
```

> **提示**：MAPF 通常是**离散问题**：地图是图（栅格或路网），时间也离散，每一步每个智能体移动到相邻顶点或原地等待。它的价值是给出无冲突的**参考计划**，真正执行时仍需要局部避障和速度控制。

## 2. 符号与单位

| 符号 | 含义 | 单位 / 说明 |
|---|---|---|
| $G=(V,E)$ | 地图图，顶点 $V$ 表示位置，边 $E$ 表示可达的相邻关系 | 无量纲 |
| $k$ | 智能体（机器人）数量 | 个 |
| $s_i,\ g_i$ | 第 $i$ 个智能体的起点、目标顶点 | $V$ 中元素 |
| $\pi_i(t)$ | 智能体 $i$ 在时刻 $t$ 所在顶点 | 离散时间步 |
| SOC | 代价之和（sum of costs） | 步数之和 |
| makespan | 最后一个智能体到达的时间 | 步数 |
| $L$ | 图拉普拉斯矩阵 $L=D-A$ | 对称半正定 |
| $\lambda_2$ | $L$ 的第二小特征值（代数连通度） | 衡量连通强度 |
| $\epsilon$ | 一致性协议的离散步长 | 无量纲 |
| $\mathbf d_i$ | 编队中第 $i$ 个机器人相对编队中心的期望偏移 | m |

## 3. 冲突的类型

| 冲突 | 定义 | 说明 |
|---|---|---|
| 顶点冲突（vertex） | 两个智能体在同一时刻占据同一顶点 | $\pi_i(t)=\pi_j(t)$ |
| 边/交换冲突（edge/swap） | 两个智能体在相邻两时刻交换位置 | $\pi_i(t)=\pi_j(t+1)$ 且 $\pi_i(t+1)=\pi_j(t)$，在窄边上会对撞 |
| 跟随冲突（following） | 一个智能体移入另一个刚腾出的顶点 | 某些模型允许，某些不允许（取决于是否可同时移动） |
| 循环冲突 | 若干智能体首尾相接循环移动 | 离散模型可能禁止 |

有些场景还要考虑**物理尺寸**（机器人有半径，不能擦肩而过）和**连续时间**，这时用 CCBS（连续时间冲突基搜索）等扩展，此处只提一句，细节未核对。

## 4. 联合状态空间与复杂度

最直接的办法是把所有智能体当成一个"超级机器人"，在联合状态空间里搜索：状态是 $k$ 个位置的组合，动作是每个智能体同时选一个动作。若每个智能体有 $N$ 个顶点，联合状态数是 $N^k$：示例（非项目实测）地图 100 个顶点：

| $k$ | 联合状态数 $100^k$ |
|---|---|
| 2 | $10^4$ |
| 5 | $10^{10}$ |
| 10 | $10^{20}$ |

（Python 复核。）同时分支因子也呈 $5^k$（每个智能体 5 个动作：4 邻域 + 等待）增长。这使得联合 A* 在 $k\ge$ 几个时已不可行。已有结论表明，最优 MAPF（如最小化 SOC 或 makespan）是 **NP 难**的（Yu 和 LaValle，2013，细节未核对）。所以实用算法的主线是：**把联合问题拆开**，各自规划，再解决冲突。

## 5. 解耦方法：优先级规划

优先级规划（prioritized planning，Erdmann 和 Lozano-Pérez 1987 提出的解耦思想，细节未核对）：

1. 给智能体排序（优先级）；
2. 按顺序逐个规划；每规划完一个，就把它的**时空轨迹**（(顶点, 时间)）加入**预约表**；
3. 后面的智能体在规划时把预约表里的 (顶点, 时间) 当作动态障碍物避开（时空 A*：状态是 (位置, 时间)，可以等待）。

**优点**：简单、快、可扩展到上百个智能体。

**缺点**：

- **不完备**：优先级不当会无解。例如两个智能体在一条单行窄道上需要互相换位，如果高优先级的智能体先占用了窄道，低优先级者可能无处可躲，规划失败，即使存在无冲突的联合解（需要一方退到支路）；
- **不最优**：结果依赖优先级顺序。

改进：尝试多种优先级排序、局部重排，或者把死锁的一对提升优先级。

## 6. 冲突基搜索 CBS（含手算）

冲突基搜索（Conflict-Based Search，Sharon 等，2015，细节未核对）是最优 MAPF 的代表，两层结构：

- **低层**：对单个智能体，在其**约束集**下用时空 A* 找最短路径；
- **高层**：在**约束树**（constraint tree, CT）上搜索。每个 CT 节点包含：一组约束、一组路径（每个智能体在其约束下的最优路径）和总代价 SOC。

高层流程：

1. 根节点：无约束，每个智能体各走自己的最短路径；
2. 取 SOC 最小的节点，检查路径之间有没有冲突；**无冲突则返回**，这是最优解；
3. 有冲突（如智能体 $i,j$ 在顶点 $v$ 时刻 $t$ 冲突）：生成两个子节点——一个给 $i$ 加约束"$i$ 不能在 $v$ 时刻 $t$"，另一个给 $j$ 加同样的约束；各自只重新规划被约束的那个智能体；
4. 重复。

因为每次选 SOC 最小的节点，且每个节点内的路径是在其约束下的最优，所以第一次发现无冲突节点时，SOC 最小，**CBS 是完备且最优的**。

### 手算

示例（非项目实测，Python 实际运行 CBS 复核）。地图：线段 a–b–c，并在 b 处接一个支路 d（b 与 a、c、d 相连）。智能体 1：a→c；智能体 2：c→a。

```text
        d
        │
  a ──── b ──── c
```

**根节点**：两人各自走最短路：

- 智能体 1：a, b, c（2 步）；智能体 2：c, b, a（2 步）。SOC = 4。
- 冲突：时刻 1 同时在顶点 b（顶点冲突）。

**分裂**：

- 子节点 A：约束"智能体 1 在 t=1 不在 b" → 智能体 1 重规划：a, a, b, c（等一步，3 步）；SOC = 3 + 2 = 5；
- 子节点 B：约束"智能体 2 在 t=1 不在 b" → 智能体 2 重规划：c, c, b, a（3 步）；SOC = 2 + 3 = 5。

取 SOC=5 的节点继续：

- 节点 A 的路径：智能体 1：a,a,b,c；智能体 2：c,b,a。逐时刻检查：t=1 时智能体 1 在 a、智能体 2 在 b；t=2 时智能体 1 在 b、智能体 2 在 a。两者在 t=1 到 t=2 之间互换了位置，这是**交换冲突**，仍然不能执行，需要继续分裂（生成 SOC=6 的子节点）；节点 B 同理还会出现新的冲突。

最终（共弹出 14 个 CT 节点，Python 复核）找到无冲突解，SOC = 7：

```text
智能体 1:  a  a  b  c        （等待 1 步，3 步到达）
智能体 2:  c  b  d  b  a     （躲进支路 d，4 步到达）
```

时刻表检查：

| t | 智能体 1 | 智能体 2 | 说明 |
|---|---|---|---|
| 0 | a | c | |
| 1 | a | b | 智能体 2 先进入 b |
| 2 | b | d | 智能体 2 躲进支路 d，让出 b |
| 3 | c | b | 智能体 1 到达 c；智能体 2 从 d 回到 b |
| 4 | c | a | 智能体 2 到达 a |

这个例子说明了两点：(1) **没有支路 d 时，两机在单行通道上必然死锁**，有了 d，最优解是让其中一方躲进去，总代价 7；(2) 如果用优先级规划，让智能体 1 最先规划（路径 a,b,c，占用 b 的时刻 1），智能体 2 就必须在预约表里找出路：它要在智能体 1 通过之前或之后绕开 b，只有躲进支路 d 才行；若给它的时空搜索设了过小的时间上限或把 d 当作不可用，就会规划失败。CBS 通过逐步加约束系统性地试遍这些选择，所以结果更可靠，而且保证最优。

CBS 的缺点：冲突多时 CT 节点数指数增长。改进包括：冲突分类（cardinal conflict 优先）、启发式（CBSH）、次优版本（ECBS，容忍一个次优系数，更快）。这些名称此处只作为索引，细节未核对。

## 7. 终身 MAPF 与交通规则

仓储里任务连续到来，机器人完成一个目标就接下一个，叫**终身 MAPF**（lifelong MAPF）。一次性规划整个未来是不现实的，常用：

- **滚动时域**：只规划未来 $H$ 步，每隔一段时间重规划（类似 MPC 的思想）；
- **令牌传递（token passing）**：每个机器人领取任务后，按已有预约轨迹规划，并把自己的轨迹放回令牌；
- **交通规则**：给路网加单行规定、路口信号灯、保留区域，把冲突从规划阶段前移到设计阶段，这是工业上非常稳的做法（牺牲一点效率，换取确定性与可调试性）。

这对应实际系统里的取舍：**纯算法最优**和**工程鲁棒可解释**之间，工程通常选后者。

## 8. 编队：领航—跟随与虚拟结构

编队控制研究的是连续空间里让一组机器人保持某种几何队形并整体运动。常见思路：

| 方法 | 思想 | 优点 | 缺点 |
|---|---|---|---|
| 领航—跟随（leader–follower） | 一台领航，其余按相对偏移跟随 | 简单直观 | 依赖领航，单点故障；误差沿链放大 |
| 虚拟结构（virtual structure） | 把队形看作一个刚体，每个机器人跟踪其上的一个虚拟点 | 整体队形精确 | 需要全局信息，集中式 |
| 行为式（behaviour-based） | 每个机器人叠加"保持队形、避障、到达目标"等行为 | 鲁棒、分布式 | 难以分析最优与稳定性 |
| 基于一致性（consensus-based） | 邻居间交换信息，令相对位置趋于期望值 | 分布式、可证明收敛 | 依赖通信拓扑连通 |

对车式机器人（非完整约束），领航—跟随需要用跟踪控制器（如 [轨迹跟踪](./04_轨迹跟踪_PurePursuit_Stanley_MPC.md) 里的方法）去跟随领航的轨迹，同时保持纵向间距和横向偏移。

## 9. 一致性协议（含手算）

### 9.1 模型

$n$ 个智能体，每个有状态 $x_i$（位置、速度、温度、任意标量……），通信拓扑是无向图 $G$。**一致性协议**：每个智能体按邻居的状态与自己的差来调整：

$$
\dot x_i=\sum_{j\in\mathcal N_i}(x_j-x_i)\quad\Longleftrightarrow\quad\dot{\mathbf x}=-L\,\mathbf x
$$

$L=D-A$ 是图拉普拉斯，$D$ 为度矩阵，$A$ 为邻接矩阵。关键性质：

1. $L$ 对称半正定，特征值 $0=\lambda_1\le\lambda_2\le\dots\le\lambda_n$；
2. 图连通 $\iff\lambda_2>0$；
3. 连通图中状态趋于**平均值**：$x_i\to\bar x=\frac1n\sum x_i(0)$；
4. 收敛速度由 $\lambda_2$ 决定，$\lambda_2$ 越大收敛越快。

离散时间版本 $\mathbf x_{k+1}=(I-\epsilon L)\mathbf x_k$，要求 $0<\epsilon<2/\lambda_{\max}$ 才收敛（否则 $|1-\epsilon\lambda_{\max}|>1$ 发散）。

### 9.2 手算

示例（非项目实测）：三个智能体排成一条链 1–2–3（1 与 2、2 与 3 相连）。

$$
L=\begin{bmatrix}1&-1&0\\-1&2&-1\\0&-1&1\end{bmatrix},\qquad\text{特征值}\ 0,\ 1,\ 3
$$

（Python 复核。）$\lambda_2=1$，$\lambda_{\max}=3$，所以步长上界 $2/3=0.667$。取 $\epsilon=0.2$，初值 $\mathbf x_0=(0,\,4,\,8)$，平均值 4。

第 1 步：$x_1\leftarrow0+0.2(4-0)=0.8$，$x_2\leftarrow4+0.2[(0-4)+(8-4)]=4$，$x_3\leftarrow8+0.2(4-8)=7.2$。

| 迭代 | $x_1$ | $x_2$ | $x_3$ |
|---|---|---|---|
| 0 | 0 | 4 | 8 |
| 1 | 0.8 | 4 | 7.2 |
| 2 | 1.44 | 4 | 6.56 |
| 3 | 1.952 | 4 | 6.048 |
| 4 | 2.362 | 4 | 5.638 |
| 5 | 2.689 | 4 | 5.311 |

（Python 复核。）三个值向 4 收敛，且总和始终是 12（平均值不变——因为 $L$ 是对称的，$\mathbf 1^\top L=0$）。误差每步按 $|1-\epsilon\lambda_2|=|1-0.2\times1|=0.8$ 的因子衰减；要把误差降到 1%，需要 $\ln0.01/\ln0.8\approx20.6$，即约 21 步。

对比：如果是三角形全连接图，特征值为 $0,3,3$，$\lambda_2=3$，同样的 $\epsilon=0.2$ 时误差衰减因子为 $|1-0.6|=0.4$，收敛快得多。**连接越密，收敛越快，但通信成本越高**——这是分布式设计的核心取舍。

## 10. 编队一致性（含手算）

要让机器人保持队形而不是聚到一点，对"相对于期望偏移的位置"做一致性：令 $\tilde x_i=x_i-d_i$（$d_i$ 是期望偏移），让 $\tilde x_i$ 趋于一致：

$$
\dot x_i=\sum_{j\in\mathcal N_i}\bigl[(x_j-d_j)-(x_i-d_i)\bigr]\ \Longleftrightarrow\ \dot{\mathbf x}=-L(\mathbf x-\mathbf d)
$$

当 $\tilde x$ 一致时 $x_i-x_j=d_i-d_j$，队形形成。

手算（一维，非项目实测）：三个机器人要保持间距 2 m，偏移 $\mathbf d=(0,2,4)$，链式拓扑，$\epsilon=0.2$，初始位置 $\mathbf x_0=(0,3,9)$。

| 迭代 | $\mathbf x$ | $\mathbf x-\mathbf d$ |
|---|---|---|
| 1 | (0.2, 3.6, 8.2) | (0.2, 1.6, 4.2) |
| 2 | (0.48, 3.84, 7.68) | (0.48, 1.84, 3.68) |
| 3 | (0.752, 3.936, 7.312) | (0.752, 1.936, 3.312) |
| 4 | (0.989, 3.974, 7.037) | (0.989, 1.974, 3.037) |
| 5 | (1.186, 3.990, 6.824) | (1.186, 1.990, 2.824) |

（Python 复核。）$\mathbf x-\mathbf d$ 三项逐渐趋于同一个数（初始 $(0,1,5)$，平均 2），最后队形间距收敛到 2 m。队形整体位置落在初值平均 $\bar x-\bar d=4-2=2$ 附近，即编队中心位于由初值决定的位置。要让整个编队移动到指定位置，可以在上式中加入一个领航/目标项（领航者 pinning）：只让部分机器人知道目标，其余通过一致性跟随。

## 11. 通信约束：延迟、丢包与拓扑变化

真实的多机通信不是理想的：

| 因素 | 影响 | 应对 |
|---|---|---|
| 延迟 | 使用的是邻居的旧状态，可能引起振荡；步长过大时不稳定 | 减小 $\epsilon$；使用时间戳并外推；只用时间戳新鲜的数据 |
| 丢包 | 某些边暂时失效，等价于拓扑变化 | 只要"随时间联合连通"，一致性仍可收敛，但变慢 |
| 拓扑切换 | 机器人相距过远断开链路 | 保持连通性约束（距离不超通信半径）；或用中继 |
| 时钟不同步 | 时间戳解释不同 | 时间同步（见 [时间同步、坐标系一致性与数据有效性](../06_状态估计/06_时间同步_坐标系一致性与数据有效性.md)） |
| 带宽 | 全互联时通信量 $\propto N^2$ | 限制邻居数；降低发送频率；只发送变化量 |

示例（非项目实测）：每台机器人每次广播 24 字节（位姿），10 Hz，全互联接收，则总吞吐 $N^2\times24\times10$ 字节/秒：$N=5,20,50$ 分别是 6 kB/s、96 kB/s、600 kB/s（Python 复核）。可见规模上去后需要拓扑稀疏化或分区。

ROS2/DDS 提供 QoS 配置（可靠/尽力而为、历史深度、截止期限），可根据数据类型选择：位姿类数据用 best-effort（旧数据无价值），指令类数据用 reliable。见 [DDS 与分布式通信](../12_机器人软件/05_DDS与分布式通信.md)。

## 12. 集中式与分布式的取舍

| 维度 | 集中式（中心调度） | 分布式（自主协商） |
|---|---|---|
| 全局最优 | 可以（有全局信息） | 通常只能局部最优 |
| 可扩展性 | 受中心算力限制 | 好 |
| 容错 | 单点故障 | 无单点 |
| 通信 | 机器人—中心星形 | 机器人—机器人网状 |
| 可调试性 | 好（中心能看到全局） | 难 |
| 典型 | 仓储调度系统 | 无人机集群、野外机器人 |

工程中常用**混合**：中心做任务分配与粗路径预约，机器人本地做精细避障与速度控制，使得中心断线时机器人能安全停下或降级运行。

## 13. 在本项目里怎么用

- **P6 深开鸿 M-Robots 多机器人平台**：CV 写明实现了"设备发现、能力注册、状态同步与任务分发"，基于 M-DDS。这是多机协同的**通信与调度基础设施**：本章的一致性协议、预约表、任务指派，都需要这样的平台来交换状态和下发任务。要诚实说明：**P6 做的是平台层的协同（发现、注册、状态同步、任务分发与安全执行链），并没有实现 MAPF 或编队控制算法**。参见 [DDS 与 M-DDS 分布式协同](../../01_项目课程/P6_深开鸿M-Robots平台软件/06_DDS与M-DDS分布式协同.md)、[任务规划与实时控制的分层](../../01_项目课程/P6_深开鸿M-Robots平台软件/07_任务规划与实时控制的分层.md)、[机器人状态管理](../../01_项目课程/P6_深开鸿M-Robots平台软件/05_机器人状态管理.md)。
- **P5 单车导航**：P5 是单机；多机场景下，别的车辆会成为动态障碍物，局部规划（TEB）的动态障碍处理见 [TEB 局部轨迹优化](../../01_项目课程/P5_车式移动机器人自主导航/09_TEB局部轨迹优化.md)。更细的动态避障（如 ORCA）在本模块第 10 章。
- **P2 双臂**：两条手臂的工作空间有重叠，需要臂间碰撞检查，本质是 2 个智能体的小型 MAPF（见 [机械臂运动规划](./07_机械臂运动规划_碰撞检测_MoveIt_CHOMP.md) 的自碰撞部分）。
- **P3 / P1 / P7**：单体控制，不涉及。

诚实表述：我没有实现过 MAPF 或编队算法；我在多机平台上做过设备发现、状态同步和任务分发，因此理解多机协同需要怎样的通信与状态管理基础，并能说明常见算法如何在这种平台上运行。

## 14. 常见错误与故障现象

| 现象 | 原因 | 处理 |
|---|---|---|
| 两台机器人在窄通道迎面对峙 | 单机规划把对方当动态障碍，死锁 | 引入预约/优先级、让位点（支路）、单行规则 |
| 优先级规划无解 | 优先级顺序不当，低优先级无路可退 | 重排序、局部重规划、提升被堵者优先级 |
| CBS 运行时间爆炸 | 冲突多，约束树指数增长 | 次优 CBS、限制规模、分区规划 |
| 计划无冲突但执行时仍碰撞 | 离散模型忽略了机器人尺寸与速度差 | 加安全距离，局部避障兜底，按实际执行进度同步 |
| 一致性迭代发散 | 步长 $\epsilon>2/\lambda_{\max}$ | 减小步长；检查拓扑和度数 |
| 一致性收敛极慢 | 拓扑接近断开，$\lambda_2$ 很小 | 增加通信边；加中继 |
| 队形振荡 | 通信延迟 + 增益过大 | 降增益；用时间戳外推 |
| 编队中心漂移 | 初值平均决定队形位置，无参考 | 增加领航者 / 目标项（pinning） |
| 队形随一台机器人故障崩溃 | 领航—跟随的单点依赖 | 增加冗余邻居，或切换领航 |
| 通信带宽占满 | 全互联、高频广播 | 稀疏拓扑、降频、只发变化量 |

## 15. 面试怎么问、怎么答

**Q1. What is multi-agent path finding, and why is it hard?**

> MAPF asks for collision-free paths for many agents on a graph, usually minimising the sum of path lengths or the makespan. The difficulty is that the joint state space grows exponentially with the number of agents, so a joint search is infeasible beyond a few robots, and optimal MAPF is NP-hard. Practical algorithms decouple the problem, for example with prioritised planning, or resolve conflicts lazily as in conflict-based search.

中文：MAPF 在图上为多个智能体找无冲突路径，最小化总代价或完工时间；联合状态空间随智能体数指数增长，最优问题是 NP 难；实用方法解耦（优先级规划）或按需解决冲突（CBS）。

**Q2. How does conflict-based search work?**

> It has two levels. The low level plans a path for a single agent subject to its constraints, using space-time A*. The high level searches a constraint tree: it starts with unconstrained shortest paths, finds the first conflict, and branches into two children, each forbidding one of the conflicting agents from that vertex at that time and replanning only that agent. It always expands the node with the lowest total cost, so the first conflict-free node is optimal.

中文：两层：低层用时空 A* 在约束下为单个智能体规划；高层搜索约束树，找第一个冲突后分成两个子节点，各禁止一方在该时空占用并只重规划该方；总是扩展代价最小的节点，第一个无冲突节点即最优。

**Q3. What are the limits of prioritised planning?**

> It is fast and scales well, but it is incomplete and suboptimal. The result depends on the priority order, and a high-priority robot can occupy a corridor in a way that leaves a lower-priority robot with no feasible plan, even though a joint solution exists. I can mitigate that by trying different orders or locally replanning, or by switching to conflict-based search for small groups.

中文：快且可扩展，但不完备也不最优；结果依赖优先级，高优先级可能占死通道让低优先级无解；缓解：换顺序、局部重规划、小组内用 CBS。

**Q4. What determines the convergence speed of a consensus protocol?**

> For an undirected connected graph with the Laplacian dynamics, all states converge to the average, and the rate is set by the second-smallest eigenvalue of the Laplacian, the algebraic connectivity. A denser graph has a larger value and converges faster, but costs more communication. In discrete time the step size must also be below two over the largest eigenvalue, otherwise it diverges.

中文：无向连通图的拉普拉斯动力学使状态收敛到平均值，速度由 $\lambda_2$ 决定；图越密收敛越快、通信越多；离散时间步长须小于 $2/\lambda_{\max}$，否则发散。

**Q5. How would you handle communication delay in a multi-robot formation?**

> I would timestamp every message and use the freshest neighbour data, extrapolating it forward with the neighbour's velocity. I would lower the consensus gain so the loop stays stable with the delay, and require that the communication graph stays jointly connected over time. If a link drops, each robot keeps a safe local behaviour, like holding speed limits and obstacle avoidance, instead of waiting for the missing data.

中文：给消息打时间戳，用最新的邻居数据并按速度外推；降低一致性增益保证有延迟时仍稳定；要求通信图随时间联合连通；链路断开时各机器人保持安全的本地行为（限速、避障），不等待缺失数据。

## 16. 本章要点速记

1. 冲突类型：顶点冲突、边（交换）冲突、跟随冲突；执行时还要考虑尺寸和速度。
2. 联合状态空间 $N^k$ 指数爆炸，最优 MAPF 是 NP 难。
3. 优先级规划：按顺序规划、轨迹入预约表、后者避让；快但不完备不最优。
4. CBS：低层时空 A*、高层约束树；每次选 SOC 最小节点，首个无冲突节点即最优。
5. 小例子：线 a–b–c 加支路 d，两机交换位置，最优 SOC=7，一方必须躲进支路。
6. 终身 MAPF：滚动时域、令牌传递；工程上常加交通规则换取确定性。
7. 编队：领航—跟随、虚拟结构、行为式、一致性；各有集中/分布与鲁棒性的取舍。
8. 一致性协议 $\dot{\mathbf x}=-L\mathbf x$：收敛到平均值，速度由 $\lambda_2$ 决定，离散步长 $\epsilon<2/\lambda_{\max}$。
9. 编队一致性：对 $x_i-d_i$ 做一致；加领航项指定整体位置。
10. 通信：延迟、丢包、拓扑变化；全互联带宽 $\propto N^2$。P6 做的是平台层协同，不含 MAPF 或编队算法。

## 17. 自测题

1. 50 个顶点、4 个智能体，联合状态数？（$50^4=6.25\times10^6$。）
2. 判断：智能体 1 在 t 时刻从 u 到 v，智能体 2 同时从 v 到 u，是什么冲突？（交换/边冲突。）
3. 一条链 1–2 两个智能体，$L=\begin{bmatrix}1&-1\\-1&1\end{bmatrix}$，特征值？$\epsilon$ 上界？（0 和 2；$2/2=1$。）
4. 一致性协议初值 $(2,6,10)$，收敛到什么值？（平均值 6。）
5. 为什么 CBS 第一次找到无冲突节点就是最优？（按总代价从小到大扩展，且每个节点内路径是约束下最优，所以更便宜的节点都已有冲突且被分裂。）

## 18. 相关链接

- 上一章：[行为树与任务规划：PDDL 与有限状态机](./08_行为树与任务规划_PDDL_有限状态机.md)
- 下一章：[局部规划进阶：DWA、MPPI 与 ORCA](./10_局部规划进阶_DWA_MPPI_ORCA.md)
- 基础：[图搜索：Dijkstra 与 A*](./01_图搜索_Dijkstra_A星.md)、[多机 SLAM 与长期地图维护](../07_SLAM与建图/07_多机SLAM与长期地图维护.md)
- 项目：[P6 M-Robots 平台 README](../../01_项目课程/P6_深开鸿M-Robots平台软件/README.md)
- 模块说明：[08_规划与导航 README](./README.md)
