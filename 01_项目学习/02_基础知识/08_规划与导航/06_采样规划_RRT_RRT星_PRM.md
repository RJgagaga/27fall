# 采样规划：RRT、RRT* 与 PRM

> **一句话定位**：当状态空间维度高（机械臂 6–7 维）或约束复杂（运动学、微分约束）时，把空间离散成栅格再搜索会因维数灾难而不可行。采样规划（sampling-based planning）的做法是：**不显式构造整个空间，只随机撒点，用碰撞检测判断可行性，把可行的点和边连成图或树，再从中找路径**。本章讲 PRM、RRT、RRT-Connect、RRT*，以及渐近最优、碰撞检测、采样策略和路径后处理。

**本章学完你能：**

- 说清采样规划为什么适合高维空间，以及它与 A* 的适用边界；
- 描述 PRM 的学习阶段与查询阶段，RRT 的单次扩展，RRT* 的"选父节点 + 重连"；
- 手算一次 RRT 扩展、一次碰撞检测、一次 RRT* 选父与重连；
- 解释概率完备性（probabilistic completeness）与渐近最优性（asymptotic optimality）的含义，写出连接半径的收缩规律；
- 说出碰撞检测的离散化做法、窄通道问题，以及常用采样策略（目标偏置、Informed 采样等）；
- 对采样得到的"弯弯曲曲的路径"做最短路捷径（shortcut）与平滑；
- 用英文回答 "Why RRT for high-dimensional problems / What is the difference between RRT and RRT* / What are the downsides of sampling-based planners"。

**前置知识**：[图搜索：Dijkstra 与 A*](./01_图搜索_Dijkstra_A星.md)、[Hybrid A* 与车辆运动学约束](./02_Hybrid_A星与车辆运动学约束.md)、[代价地图、局部规划与 TEB](./03_代价地图_局部规划_TEB.md)、[信息论与图论基础](../01_数学基础/07_信息论与图论基础.md)、[概率统计与高斯分布](../01_数学基础/03_概率统计与高斯分布.md)。

## 目录

1. 采样规划要解决什么问题
2. 符号与单位
3. 为什么栅格搜索在高维里失效
4. PRM：路线图
5. RRT：快速扩展随机树（含手算与代码）
6. RRT-Connect 与目标偏置
7. RRT*：渐近最优（含手算）
8. 连接半径与渐近最优的条件
9. 碰撞检测
10. 采样策略与窄通道
11. 路径后处理：捷径与平滑（含手算）
12. 带运动学约束的采样规划
13. 与 A*、Hybrid A*、TEB 的对比
14. 在本项目里怎么用
15. 常见错误与故障现象
16. 面试怎么问、怎么答
17. 本章要点速记
18. 自测题
19. 相关链接

---

## 1. 采样规划要解决什么问题

路径规划的数学描述是：在**配置空间**（configuration space，$\mathcal C$）里，找一条从起点 $q_{\text{start}}$ 到目标 $q_{\text{goal}}$ 的连续曲线，全程落在无碰撞的自由空间 $\mathcal C_{\text{free}}$ 中。对于平面上的小车，$\mathcal C$ 是 2 或 3 维（$x,y,\theta$）；对于 7 自由度机械臂，$\mathcal C$ 是 7 维。

问题难点：$\mathcal C_{\text{free}}$ 的**显式形状**几乎无法写出来。我们只能做一件事——**给一个配置 $q$，用碰撞检测器回答"这里有没有碰撞"**。采样规划就建立在这个"黑盒碰撞检测"之上：

```text
随机采样 q ──> 碰撞检测 ──> 自由? ──是──> 加入图/树，并尝试与邻居连边
                                │
                                └──否──> 丢弃
```

> **提示**：采样规划的核心哲学是"用大量廉价的碰撞查询，代替对自由空间的显式建模"。

## 2. 符号与单位

| 符号 | 含义 | 单位 / 说明 |
|---|---|---|
| $\mathcal C,\ \mathcal C_{\text{free}},\ \mathcal C_{\text{obs}}$ | 配置空间、自由空间、障碍空间 | 维度 $d$ 等于自由度 |
| $q$ | 一个配置（点） | 如 $(x,y)$ m，或关节角 rad |
| $q_{\text{rand}},\ q_{\text{near}},\ q_{\text{new}}$ | 随机样本、树中最近点、新扩展点 | 同 $q$ |
| $\eta$ | 扩展步长（steer 的最大距离） | m 或 rad |
| $c(q)$ | 从起点到 $q$ 的路径代价（cost-to-come） | m |
| $r_n$ | $n$ 个样本时的连接半径 | m |
| $d$ | 配置空间维数 | 无量纲 |
| $\gamma$ | 半径公式中的常数 | 无量纲 |
| $\mu(\cdot)$ | 空间测度（面积/体积） | m² 或 m³ |

## 3. 为什么栅格搜索在高维里失效

A* 在栅格上搜索，每一维切成 $m$ 格，总格数为 $m^d$。示例（非项目实测）：每维 100 格，2 维是 $10^4$ 个状态，3 维是 $10^6$，6 维是 $10^{12}$——存都存不下。这就是**维数灾难**（curse of dimensionality）。

采样规划放弃了"覆盖每个格子"：它用 $n$ 个随机样本去覆盖空间，$n$ 的规模与维度无直接的指数关系。代价是：

- 不保证**最优**（RRT 找到的路径往往弯曲）；
- 只保证**概率完备**：只要存在可行路径，样本数趋于无穷时找到的概率趋于 1；
- 结果有随机性，同一问题两次运行路径不同。

低维（2D 平面导航）下，A* 类栅格搜索通常更稳定、结果可复现；高维（机械臂）下，采样规划几乎是唯一实用选择。

## 4. PRM：路线图

概率路线图（Probabilistic Roadmap, PRM，Kavraki 等，1996，细节未核对）适合**多次查询同一环境**：

**学习阶段（预处理，一次）**

1. 在 $\mathcal C$ 中均匀采样 $n$ 个点，保留无碰撞的，作为节点；
2. 对每个节点，找 $k$ 个最近邻（或半径 $r$ 内邻居），对每对尝试用局部规划器（直线）连边，边也需通过碰撞检测；
3. 得到一张无向图（路线图）。

**查询阶段（每次）**

1. 把起点、终点也连接到路线图的最近节点；
2. 在图上用 Dijkstra 或 A* 求最短路径。

特点：路线图建好后，查询非常快；缺点是需要预先建图，环境一变就要重建（或局部修补），且建图本身是 $O(n\log n)$ 级的最近邻搜索加上大量碰撞检测。

示例（非项目实测）：$n=1000$ 个节点，暴力两两比较要 $1000\times999/2\approx5\times10^5$ 次距离计算，所以实际用 KD 树做最近邻。

## 5. RRT：快速扩展随机树（含手算与代码）

快速扩展随机树（Rapidly-exploring Random Tree, RRT，LaValle，1998，细节未核对）适合**单次查询**，不需要预处理。它从起点生长一棵树，每次迭代：

1. **采样** $q_{\text{rand}}$（在整个空间中均匀随机）；
2. **找最近** $q_{\text{near}}$：树中离 $q_{\text{rand}}$ 最近的节点；
3. **延伸**（steer）：从 $q_{\text{near}}$ 朝 $q_{\text{rand}}$ 方向走最多 $\eta$，得到 $q_{\text{new}}$：

$$
q_{\text{new}}=q_{\text{near}}+\min\bigl(\eta,\ \lVert q_{\text{rand}}-q_{\text{near}}\rVert\bigr)\frac{q_{\text{rand}}-q_{\text{near}}}{\lVert q_{\text{rand}}-q_{\text{near}}\rVert}
$$

4. **碰撞检测**：线段 $q_{\text{near}}\to q_{\text{new}}$ 无碰撞则加入树（记录父节点 $q_{\text{near}}$），否则丢弃；
5. 若 $q_{\text{new}}$ 到目标足够近，从目标回溯父节点得到路径。

### 5.1 Voronoi 偏置：为什么叫"快速扩展"

最近点扩展有一个重要性质：树中每个节点被选中的概率正比于它的 **Voronoi 区域**面积。边界上的节点 Voronoi 区域大（向外无限延伸），被选中的概率大，所以树会倾向于**向未探索的区域扩展**，而不是在已探索区域内打转。这是 RRT 能快速覆盖空间的原因。

### 5.2 手算：一次扩展

示例（非项目实测）：树中有三个节点 $q_0=(1,1)$，$q_1=(3,1.5)$，$q_2=(2,4)$。步长 $\eta=1$。采样 $q_{\text{rand}}=(7,4)$。

- 到各节点的距离：$d_0=6.708$，$d_1=4.717$，$d_2=5.000$；
- 最近的是 $q_1$；方向向量 $(4,2.5)/4.717=(0.848,0.530)$；
- $q_{\text{new}}=(3,1.5)+1\times(0.848,0.530)=(3.848,\,2.030)$（Python 复核）。

### 5.3 手算：碰撞检测

障碍物为圆，圆心 $c$，半径 $\rho$。线段 $a\to b$ 与圆相交，等价于**圆心到线段的最近距离 $\le\rho$**。线段上最近点用投影参数 $t$ 并夹到 $[0,1]$：

$$
t=\mathrm{clip}\Bigl(\frac{(c-a)\cdot(b-a)}{\lVert b-a\rVert^2},\,0,\,1\Bigr),\qquad p=a+t(b-a),\qquad\text{碰撞}\iff\lVert p-c\rVert\le\rho
$$

用上面的线段 $a=(3,1.5)\to b=(3.848,2.030)$：

- 圆心 $(4,\,2.5)$、半径 0.5：$t=1$（最近点是端点 $b$），距离 0.494 $\le0.5$，**碰撞**，该次扩展丢弃；
- 圆心 $(4.5,\,3.0)$、半径 0.5：距离 1.169 $>0.5$，**无碰撞**，加入树；
- 圆心 $(4,\,3.0)$、半径 0.5：距离 0.982 $>0.5$，无碰撞。

（Python 复核。）第一种恰好擦过障碍物边缘 0.006 m 的情形最能说明问题：**如果只检查终点 $q_{\text{new}}$ 本身是否在障碍内，会漏掉穿过障碍的边**，必须检查整条线段。

### 5.4 最小代码（教学示例，已运行核对，非项目源码）

```python
import math, random

def seg_hits_circle(a, b, c, r):
    ax, ay = a; bx, by = b; cx, cy = c
    abx, aby = bx - ax, by - ay
    t = ((cx - ax) * abx + (cy - ay) * aby) / (abx * abx + aby * aby)
    t = max(0.0, min(1.0, t))
    px, py = ax + t * abx, ay + t * aby
    return math.hypot(px - cx, py - cy) <= r

def rrt(start, goal, obstacles, step=0.5, goal_bias=0.1, iters=3000, seed=1):
    random.seed(seed)
    nodes, parent = [start], [-1]
    for _ in range(iters):
        q = goal if random.random() < goal_bias else (random.uniform(0, 10), random.uniform(0, 10))
        i = min(range(len(nodes)), key=lambda k: math.dist(nodes[k], q))
        d = math.dist(nodes[i], q)
        if d < 1e-9:
            continue
        s = min(step, d)
        new = (nodes[i][0] + s * (q[0] - nodes[i][0]) / d,
               nodes[i][1] + s * (q[1] - nodes[i][1]) / d)
        if any(seg_hits_circle(nodes[i], new, c, r) for c, r in obstacles):
            continue
        nodes.append(new); parent.append(i)
        if math.dist(new, goal) < step:
            path, k = [goal], len(nodes) - 1
            while k != -1:
                path.append(nodes[k]); k = parent[k]
            return path[::-1], len(nodes)
    return None, len(nodes)
```

在 10 m × 10 m 空间、三个圆形障碍、起点 $(1,1)$、终点 $(9,9)$、步长 0.5、`seed=1` 的一次实际运行结果：树共 60 个节点，路径含 29 个点，长度 13.45 m，而起终点直线距离为 11.31 m。路径长度约是直线的 1.19 倍——这就是 RRT"能找到但不最优"的直观体现。

## 6. RRT-Connect 与目标偏置

**目标偏置（goal bias）**：以小概率（如 5%–10%）直接把目标作为 $q_{\text{rand}}$，让树有一定概率直奔目标。偏置太大会让树陷入凹形障碍的"陷阱"（一直朝着被挡住的目标撞），太小则像盲目探索。5.4 的代码里取了 0.1。

**RRT-Connect**（Kuffner 和 LaValle，2000，细节未核对）：同时从起点和终点各长一棵树，交替扩展；每次扩展一棵树之后，让另一棵树贪心地朝新点连续延伸（connect），直到被阻挡或接上。两棵树相遇就得到路径。在机械臂规划里它是非常常用的默认算法（MoveIt 的 OMPL 默认规划器之一，细节见下一章），因为"两头往中间找"对高维问题的效率提升明显。

## 7. RRT*：渐近最优（含手算）

### 7.1 RRT 为什么不最优

RRT 把 $q_{\text{new}}$ 接到 $q_{\text{near}}$ 上，**不管**这样接是不是让 $q_{\text{new}}$ 的路径代价最小，也不会在后来发现更好的连接时修改旧连接。Karaman 和 Frazzoli（2011）证明了标准 RRT 几乎必然收敛到次优解（细节未核对）。

### 7.2 RRT* 的两个新步骤

RRT*（Karaman 和 Frazzoli，2011，细节未核对）在 RRT 基础上增加：

1. **选择最优父节点**（choose parent）：在 $q_{\text{new}}$ 周围半径 $r_n$ 内的所有节点中，选使 $c(q_{\text{near}})+\lVert q_{\text{near}}-q_{\text{new}}\rVert$ 最小、且连线无碰撞的作为父节点；
2. **重连**（rewire）：对半径内的每个邻居 $q'$，检查若改以 $q_{\text{new}}$ 为父，代价是否更低：$c(q_{\text{new}})+\lVert q_{\text{new}}-q'\rVert<c(q')$，且无碰撞，则把 $q'$ 的父节点改成 $q_{\text{new}}$，并（概念上）更新其子孙的代价。

### 7.3 手算

示例（非项目实测）：树里有 5 个节点：

| 节点 | 坐标 | 父节点 | 代价 $c$ |
|---|---|---|---|
| 0 | (1, 1) | - | 0 |
| 1 | (3, 1.5) | 0 | 2.062 |
| 2 | (2, 4) | 0 | 3.162 |
| 3 | (3, 3.5) | 2 | 4.280 |
| 4 | (5, 2) | 3 | 6.780 |

节点 4 的路径是 0→2→3→4，绕了一大圈（6.78）。现在 RRT 样本得到新点 $q_{\text{new}}=(4,\,2)$，取半径 $r_n=2$（假设相关连线都无碰撞）。

**第一步，选父节点**：半径内邻居为节点 1、3、4：

| 候选父 | $c+$ 距离 | 总代价 |
|---|---|---|
| 1 | $2.062+\lVert(3,1.5)-(4,2)\rVert=2.062+1.118$ | **3.180** |
| 3 | $4.280+1.803$ | 6.083 |
| 4 | $6.780+1.000$ | 7.780 |

选节点 1 为父，$c(q_{\text{new}})=3.180$。如果是普通 RRT，它会被接到最近的节点（距离最近的是节点 4 或 1），不一定是最优的。

**第二步，重连**：检查邻居是否改接 $q_{\text{new}}$ 更好：

| 邻居 | 原代价 | 经 $q_{\text{new}}$ 的代价 | 是否重连 |
|---|---|---|---|
| 1 | 2.062 | $3.180+1.118=4.298$ | 否 |
| 3 | 4.280 | $3.180+1.803=4.982$ | 否 |
| 4 | 6.780 | $3.180+1.000=4.180$ | **是** |

节点 4 的代价从 6.78 降到 4.18，改以 $q_{\text{new}}$ 为父，那条"绕远"的路径被截直了（Python 复核）。**每插入一个新点，树都有机会被局部整理得更短**，随着样本增多，整棵树逐步趋向最短路径树。

## 8. 连接半径与渐近最优的条件

重连的半径 $r_n$ 如何选，决定了算法能不能渐近最优。太小则邻居太少，重连不起作用；太大则每次要检查的点太多，计算量爆炸。理论结果（Karaman 和 Frazzoli，2011，细节未核对）是半径应随样本数缓慢**收缩**：

$$
r_n=\gamma\Bigl(\frac{\ln n}{n}\Bigr)^{1/d},\qquad\gamma>\gamma^*=2\Bigl(1+\frac1d\Bigr)^{1/d}\Bigl(\frac{\mu(\mathcal C_{\text{free}})}{\zeta_d}\Bigr)^{1/d}
$$

其中 $\zeta_d$ 是 $d$ 维单位球的体积。不必背 $\gamma^*$ 的完整表达，记住三点：(1) $r_n\propto(\ln n/n)^{1/d}$，维数越高收缩越慢；(2) 需要比某个下界大才能保证渐近最优；(3) 每次迭代近邻数量约为 $O(\log n)$。

示例（非项目实测，取 $d=2$，$\gamma=2$）：

| $n$ | $r_n=2\sqrt{\ln n/n}$ |
|---|---|
| 100 | 0.429 |
| 1000 | 0.166 |
| 10000 | 0.061 |

（Python 复核。）样本越多，半径越小，计算量不爆炸，但仍然保证邻居数量足够。也可以用 **k 最近邻**形式（$k\propto\ln n$）代替半径。

> **注意**：渐近最优是"样本数趋于无穷"的结论。实践中给固定时间（几秒），RRT* 的解只是"比 RRT 好"，而且高维下收敛很慢，所以要结合后处理（第 11 节）。

## 9. 碰撞检测

采样规划 80% 以上的时间花在碰撞检测上，所以它是性能的关键。

### 9.1 点碰撞 vs 边碰撞

- **点碰撞**：检查单个配置 $q$ 是否在障碍内（机械臂：正运动学算出各连杆位置，再与环境几何比较）；
- **边碰撞**：检查两个配置之间的连线。常用做法是**离散化**：沿线每隔步长 $\delta$ 取一个点做点碰撞。示例（非项目实测）：边长 3 m、$\delta=0.1$ m，需要 30 次点检测。$\delta$ 要小于障碍物最小尺寸，否则会"跳过"薄障碍；更严格的做法是用保守的距离界（每个点到障碍的距离 $\ge$ 到下一个点的步长，则两点间保证无碰撞），叫做**连续碰撞检测**或**球形扫掠**。

### 9.2 粗—细两级

先用简单形状（球、AABB 包围盒）快速排除绝大部分不碰撞的情况，再对可能碰撞的用精确几何（网格）。几何算法（GJK、FCL 库）见下一章。

### 9.3 栅格地图下的碰撞

2D 导航中，常用栅格代价地图：把机器人的外形（footprint）投影到栅格，检查覆盖的格子代价是否为致命。若用内切圆/外接圆加速检测，需要注意过于保守会堵住窄通道（见 [P5 代价地图与车体外形](../../01_项目课程/P5_车式移动机器人自主导航/07_代价地图与车体外形.md)）。

## 10. 采样策略与窄通道

### 10.1 均匀采样的问题：窄通道

均匀采样下，样本落入窄通道的概率正比于通道面积占比。示例（非项目实测）：自由空间面积 100 m²，一个宽 0.5 m、长 4 m 的窄通道面积 2 m²，占比 $p=0.02$。

- 平均需要 $5/0.02=250$ 个样本，才能有 5 个落在通道里；
- 采 100 个样本，一个也没落进去的概率为 $(1-0.02)^{100}=0.133$；
- 要以 95% 概率至少有一个，需要 $\ln0.05/\ln0.98=148$ 个样本。

而且光有一个样本不够，还要把通道两端连起来，需要通道内有一串彼此可连的样本，需要的样本数更大。这就是**窄通道问题**：通道越窄，需要的样本呈数量级增长。

### 10.2 改进采样

| 策略 | 思想 |
|---|---|
| 目标偏置 | 以小概率采样目标 |
| 高斯采样（Gaussian sampler） | 采一对相距服从高斯的点，仅当一个在自由空间、一个在障碍内时保留，样本集中在障碍边界 |
| 桥测试（bridge test） | 找"两端在障碍内、中点在自由空间"的点，专门采窄通道中心 |
| 障碍物附近采样 | 在障碍物边界附近加密 |
| Informed RRT* | 找到第一个解后，只在椭球内采样（椭球的两个焦点是起终点，长轴长为当前解长度），因为只有椭球内的点才可能改进解（Gammell 等，2014，细节未核对） |
| 生成式学习 | 用神经网络学习采样分布（延伸方向） |

Informed 采样的几何很漂亮：从 $A$ 到 $B$ 经过 $P$ 的路径长度至少是 $\lVert AP\rVert+\lVert PB\rVert$，要小于当前最优 $c_{\text{best}}$，必须 $\lVert AP\rVert+\lVert PB\rVert<c_{\text{best}}$——这正是椭圆的定义。

## 11. 路径后处理：捷径与平滑（含手算）

RRT 的路径锯齿多，机器人不能直接跟。标准后处理：

1. **捷径（shortcut）**：随机选路径上两个点，若它们之间的直线无碰撞，则用直线替换中间的所有点；重复若干次；
2. **平滑**：对剩余折线做样条/贝塞尔曲线拟合，或者作为初值交给轨迹优化器（CHOMP、TEB 等）；
3. **时间参数化**：加入速度、加速度约束，得到可执行的轨迹（见 [轨迹跟踪](./04_轨迹跟踪_PurePursuit_Stanley_MPC.md)）。

手算（假设相关直线都无碰撞）：路径点 $(0,0),(1,2),(2,0.5),(3,2.5),(4,1),(5,0)$，总长 9.492。选点 $(0,0)$ 与 $(2,0.5)$ 直连，再选 $(2,0.5)$ 与 $(5,0)$ 直连，得到折线 $(0,0)\to(2,0.5)\to(5,0)$，长度 5.103，而起点到终点的直线距离是 5.000（Python 复核）。长度缩短了约 46%，非常接近下界。

## 12. 带运动学约束的采样规划

上面默认机器人能朝任意方向直线移动（完整约束，holonomic）。对车式机器人（非完整约束，non-holonomic），两点间不能走直线，必须用符合运动学的**局部规划器**（steering function）连接，如 Dubins 曲线或 Reeds-Shepp 曲线（见 [Hybrid A* 与车辆运动学约束](./02_Hybrid_A星与车辆运动学约束.md)）。此时：

- 状态空间包含朝向 $(x,y,\theta)$，距离度量也要考虑朝向；
- 精确连接两点是一个两点边值问题，求解昂贵；
- 另一类做法是 **kinodynamic RRT**：直接随机采样控制输入 $(v,\delta)$，前向仿真一小段时间，得到新状态，不需要两点连接。代价是更难逼近目标，也没有最优性保证。

实际上，车式机器人的室内导航更常用 Hybrid A*（确定、可复现），采样规划更多出现在机械臂、高维或复杂约束场景。

## 13. 与 A*、Hybrid A*、TEB 的对比

| 维度 | A* | Hybrid A* | RRT | RRT* | PRM |
|---|---|---|---|---|---|
| 空间 | 离散栅格 | 连续状态 + 离散栅格剪枝 | 连续 | 连续 | 连续 |
| 适用维度 | 低维（2–3） | 3 维 $(x,y,\theta)$ | 高维 | 高维 | 高维 |
| 完备性 | 分辨率完备 | 非完备（近似） | 概率完备 | 概率完备 | 概率完备 |
| 最优性 | 栅格上最优 | 次优 | 无 | 渐近最优 | 在图内最优 |
| 复现性 | 确定 | 确定 | 随机 | 随机 | 随机 |
| 运动学可行 | 否 | 是 | 需特殊处理 | 需特殊处理 | 需特殊处理 |
| 典型用途 | 室内移动机器人 | 车式机器人泊车/导航 | 机械臂/复杂约束 | 同左，要质量 | 同一环境多次查询 |

TEB（局部）不是搜索算法而是**轨迹优化器**：它从全局路径出发做局部优化。采样规划的输出（经捷径平滑）可以作为优化器的初值，两类方法是互补的：采样负责"找到拓扑上可行的路径"，优化负责"把它变得平滑、满足动力学"。

## 14. 在本项目里怎么用

- **P5 车式导航**：P5 全局规划选用 Hybrid A*，局部用 TEB，没有使用采样规划。选择 Hybrid A* 的理由与本章第 12、13 节一致：低维（$x,y,\theta$）、需要运动学可行、需要结果确定可复现。面试若被问"为什么不用 RRT*"，可以按这个思路回答。见 [Hybrid A* 全局规划](../../01_项目课程/P5_车式移动机器人自主导航/08_HybridAStar全局规划.md)、[TEB 局部轨迹优化](../../01_项目课程/P5_车式移动机器人自主导航/09_TEB局部轨迹优化.md)。
- **P2 双臂**：CV 中的 P2 是 VR 遥操作 + 约束逆运动学（MuJoCo/Mink），**没有使用采样规划**；遥操作把末端目标实时给到 IK，避障靠人的判断与约束，没有离线全局路径规划。采样规划是机械臂自主运动规划的标准做法，属于延伸知识，在下一章展开。见 [逆运动学、雅可比、奇异性与阻尼最小二乘](../../01_项目课程/P2_未来动力双臂遥操作与差速底盘/04_逆运动学_雅可比_奇异性与阻尼最小二乘.md)。
- **P6**：任务规划与实时控制分层，规划层给出路径/动作，不直接执行，见 [任务规划与实时控制的分层](../../01_项目课程/P6_深开鸿M-Robots平台软件/07_任务规划与实时控制的分层.md)。

诚实表述：我的项目没有实现 RRT 或 PRM，我用的是 Hybrid A* 和 TEB。我理解采样规划的原理和适用边界，能说明为什么我的场景更适合 Hybrid A*。

## 15. 常见错误与故障现象

| 现象 | 原因 | 处理 |
|---|---|---|
| 路径穿过障碍物边缘 | 只检查了节点，没检查边；离散步长太大 | 沿边密集检测，或用连续检测 |
| 路径极其弯曲 | RRT 本身不最优；未做后处理 | 捷径 + 平滑；改用 RRT* |
| 窄通道里始终找不到路径 | 均匀采样几乎采不到通道 | 桥测试、高斯采样、增加样本、目标偏置 |
| 目标偏置太大，卡在凹形障碍前 | 一直朝被挡的目标撞 | 减小偏置至 5%–10%，加随机样本 |
| RRT* 越跑越慢 | 邻居检索和重连开销，$n$ 增大 | 用 KD 树；限制半径/邻居数；Informed 采样 |
| 同样输入每次路径都不同 | 采样有随机性 | 固定随机种子便于调试；上线时接受随机性并加验证 |
| 路径不可执行（拐角过急） | 忽略运动学约束 | 用 steering function、或作为 TEB/MPC 的初值 |
| 碰撞检测成为瓶颈 | 精确网格检测过多 | 粗—细两级、包围盒、缓存、并行 |
| 路径刚好贴着障碍 | 没有安全裕度 | 对障碍物膨胀，或在代价函数里加距离惩罚 |

## 16. 面试怎么问、怎么答

**Q1. Why use sampling-based planners instead of grid search for a robot arm?**

> A 7-DoF arm has a seven-dimensional configuration space. A grid with even 100 cells per dimension would need $10^{14}$ cells, so grid search is impossible. Sampling-based planners build a graph or tree from random collision-free samples, so their cost depends on the number of samples and collision checks, not on a full grid. The trade-off is that they are only probabilistically complete and the paths are not optimal.

中文：7 自由度的配置空间每维 100 格就是 $10^{14}$ 个格子，无法搜索；采样规划只依赖样本数与碰撞检测次数；代价是只有概率完备且路径不最优。追问：概率完备是什么意思？（若存在可行路径，样本趋于无穷时找到的概率趋于 1。）

**Q2. What is the difference between RRT and RRT*?**

> RRT connects each new node to the nearest tree node, so the path is feasible but usually far from optimal and never improves its old connections. RRT* adds two steps: it chooses the parent among nearby nodes that gives the lowest cost-to-come, and it rewires neighbours through the new node when that lowers their cost. With a shrinking connection radius, this makes it asymptotically optimal.

中文：RRT 接最近节点，可行但不最优且不改旧连接；RRT* 增加选最优父节点和重连，半径随样本增加而收缩时渐近最优。追问：半径怎么选？（$r_n=\gamma(\ln n/n)^{1/d}$。）

**Q3. What is the narrow passage problem and how do you mitigate it?**

> With uniform sampling, the chance of landing in a narrow passage is proportional to its area, so it takes many samples to find even one inside it. I can bias sampling towards obstacle boundaries with a Gaussian sampler, use a bridge test to put samples in the middle of narrow gaps, or increase the number of samples and use RRT-Connect to explore from both ends.

中文：均匀采样落入窄通道的概率正比于其面积；缓解：高斯采样偏向障碍边界、桥测试采通道中心、增加样本、RRT-Connect 双向。

**Q4. How do you make an RRT path usable on a real robot?**

> The raw path is jagged and ignores dynamics. I first apply shortcutting to remove unnecessary waypoints, then smooth it with splines or use it as the initial guess for a trajectory optimiser, and finally time-parameterise it with velocity and acceleration limits. For a car-like robot I would also use a kinematically feasible steering function.

中文：先做捷径去冗余点，再样条平滑或作为轨迹优化初值，最后做带速度/加速度限制的时间参数化；车式机器人还要用满足运动学的连接曲线。

**Q5. Why did you choose Hybrid A* rather than RRT* for your car-like robot?**

> For a car-like robot in a 2D map, the state space is only three-dimensional, so a search-based planner is practical. Hybrid A* embeds the kinematic constraints directly in the motion primitives, and it is deterministic and repeatable, which matters for debugging and safety. RRT* would need a steering function for non-holonomic constraints and gives a different path every run.

中文：状态空间只有 3 维，搜索法可行；Hybrid A* 把运动学约束放进运动基元，确定可复现，利于调试和安全；RRT* 对非完整约束需要连接函数且每次路径不同。

## 17. 本章要点速记

1. 采样规划用随机样本 + 黑盒碰撞检测，避开高维栅格的 $m^d$ 爆炸。
2. PRM：预建路线图，适合多次查询；RRT：单次查询，树向 Voronoi 大区域扩展。
3. RRT 一步：采样、最近点、延伸（步长 $\eta$）、边碰撞检测、加入树。
4. 碰撞要检查整条边，而不只是端点；圆障碍用点到线段最近距离。
5. 目标偏置取 5%–10%；RRT-Connect 双向生长，机械臂常用。
6. RRT* = 选最优父节点 + 重连，$r_n=\gamma(\ln n/n)^{1/d}$ 时渐近最优。
7. 概率完备不等于快、不等于最优；窄通道需要特殊采样策略。
8. Informed 采样：解的长度给出椭球，只在椭球里采样才能改进解。
9. 后处理：捷径 + 平滑 + 时间参数化；车式机器人需满足运动学。
10. 我的项目用 Hybrid A* + TEB，没有实现采样规划，要如实说明。

## 18. 自测题

1. 6 维空间每维 50 格，网格总数是多少？（$50^6=1.56\times10^{10}$。）
2. $q_{\text{near}}=(0,0)$，$q_{\text{rand}}=(3,4)$，$\eta=2$，$q_{\text{new}}$ 是什么？（距离 5，方向 $(0.6,0.8)$，$q_{\text{new}}=(1.2,1.6)$。）
3. 圆心 $(2,0)$ 半径 0.3，线段 $(0,0.2)\to(4,0.2)$ 是否碰撞？（最近点 $(2,0.2)$，距离 0.2 < 0.3，碰撞。）
4. 为什么 RRT* 的半径要随 $n$ 收缩？（保持每次邻居数约 $O(\log n)$，既足够重连又不让计算量爆炸。）
5. 椭球采样中，起终点距离 5，当前最优路径长 6，椭球长轴是多少？（等于当前代价 6，焦距 5。）

## 19. 相关链接

- 上一章：[导航系统架构：Nav2 与 move_base 对比](./05_导航系统架构_Nav2与move_base对比.md)
- 下一章：[机械臂运动规划：碰撞检测、MoveIt 与 CHOMP](./07_机械臂运动规划_碰撞检测_MoveIt_CHOMP.md)
- 基础：[图搜索：Dijkstra 与 A*](./01_图搜索_Dijkstra_A星.md)、[Hybrid A* 与车辆运动学约束](./02_Hybrid_A星与车辆运动学约束.md)
- 项目：[P5 车式移动机器人导航 README](../../01_项目课程/P5_车式移动机器人自主导航/README.md)
- 模块说明：[08_规划与导航 README](./README.md)
