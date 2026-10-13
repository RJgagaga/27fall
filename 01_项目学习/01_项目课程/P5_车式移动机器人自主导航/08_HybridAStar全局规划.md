# 08 Hybrid-A* 全局规划

> **一句话定位**：普通 A* 在格子上搜索，路径是"格子中心的折线"，车根本开不出来；Hybrid-A* 在**连续的 $(x,y,\theta)$** 里按车的运动学**一小段弧线一小段弧线**地往外长，再用格子只做"去重"，因此得到的每一段都是车真的能开的。CV 对应：*incorporated vehicle heading and kinematic constraints into the search and generated feasible global paths under vehicle-footprint, minimum-turning-radius and obstacle-cost constraints*。

**本章学完你能：**

- 用一个小网格手算 BFS、A*、Dijkstra，并**解释为什么沿父指针一定能倒推回起点**（你笔记里的疑问）；
- 解释为什么 A* 路径对车不可行，Hybrid-A* 怎样改进；
- 说清 Hybrid-A* 的状态、"桶"（3D 离散格）、运动基元、代价函数、启发式、解析扩展；
- **手算一步扩展**：三个转向基元、新位姿、所在桶、$g/h/f$；
- 手算一条 Dubins 曲线（LSR）的长度，理解 Reeds-Shepp 解析扩展；
- 用三点曲率验证路径是否满足最小转弯半径；
- 解释路径平滑、倒车换向（cusp）与失败模式；
- 说出 Hybrid-A* 的关键参数（以 Nav2 Smac 为例）和调参思路。

**前置知识：**

- 图搜索：[../../02_基础知识/06_规划与导航/01_图搜索_Dijkstra_A星.md](../../02_基础知识/06_规划与导航/01_图搜索_Dijkstra_A星.md)
- Hybrid-A* 与车辆运动学约束（基础）：[../../02_基础知识/06_规划与导航/02_Hybrid_A星与车辆运动学约束.md](../../02_基础知识/06_规划与导航/02_Hybrid_A星与车辆运动学约束.md)
- 本项目：[02 运动学](./02_车辆运动学与里程计.md)、[07 代价地图与车体外形](./07_代价地图与车体外形.md)

**目录**

1. 从 BFS 到 A*：先把"怎么倒推路径"讲透
2. 为什么 A* 路径对车不可行
3. Hybrid-A* 的核心思想
4. 状态、桶与运动基元
5. 一步扩展的完整手算
6. 代价函数
7. 启发式函数
8. 解析扩展（Reeds-Shepp / Dubins）与 Dubins 手算
9. 算法流程与伪代码
10. 后处理：平滑与曲率检查
11. 倒车与换向
12. 参数与调参（以 Nav2 Smac 为例）
13. 失败模式与故障现象
14. 对比：A*、Hybrid-A*、状态格、RRT*
15. 规划耗时、触发与降级：把全局规划放进整个节拍
16. 面试问答 / 要点速记 / 自测题 / 链接

---

## 1. 从 BFS 到 A*：先把"怎么倒推路径"讲透

### 1.1 一个小网格

`S`=起点，`G`=终点，`#`=障碍，`.`=空闲，4 邻接（上下左右），坐标记作（行，列）：

```text
列:   0   1   2   3
行0:  S   .   .   .
行1:  #   #   .   #
行2:  .   .   .   G
```

S=(0,0)，G=(2,3)，障碍=(1,0)、(1,1)、(1,3)。

### 1.2 BFS（广度优先）逐步过程

规则：用**队列**（先进先出）；每个格子**第一次被发现时**，记下"它是从哪个格子发现的"（**父指针 parent**）和距离。邻居检查顺序：右、下、左、上。

| 步 | 取出 | 发现的新格子（parent） | 队列（取出后） |
|---|---|---|---|
| 0 | — | S=(0,0)，dist=0 | [S] |
| 1 | (0,0) | 右 (0,1)←(0,0)；下 (1,0) 是障碍 | [(0,1)] |
| 2 | (0,1) | 右 (0,2)←(0,1)；下 (1,1) 障碍；左 (0,0) 已访问 | [(0,2)] |
| 3 | (0,2) | 右 (0,3)←(0,2)；下 (1,2)←(0,2) | [(0,3),(1,2)] |
| 4 | (0,3) | 下 (1,3) 障碍；其余已访问 | [(1,2)] |
| 5 | (1,2) | 右 (1,3) 障碍；下 (2,2)←(1,2)；左 (1,1) 障碍 | [(2,2)] |
| 6 | (2,2) | 右 (2,3)=G←(2,2)；左 (2,1)←(2,2) | [G,(2,1)] |
| 7 | 取出 G | 到达终点，停止 | |

**倒推**：从 G 开始，反复"跳到自己的父格子"：

```text
G(2,3) ← (2,2) ← (1,2) ← (0,2) ← (0,1) ← S(0,0)
```

反转得路径 S → (0,1) → (0,2) → (1,2) → (2,2) → G，共 5 步。

### 1.3 为什么沿父指针"一定能找回起点"

这是你笔记里的核心疑问。有三条事实共同保证：

1. **每个格子只有一个父亲**：只在"第一次被发现"时记录，之后不再改；
2. **父亲一定比孩子先被发现**：父亲被取出处理时才会发现孩子，所以父亲的距离严格小 1（BFS）；
3. **只有起点没有父亲**。

因此从任何已被发现的格子出发，每跳一步距离严格减 1，不会成环，也不会停在中间，必然在距离 0（起点）处结束。可以走"上下左右"任意方向——父亲是**哪个方向**由发现顺序决定，与方向无关。（`#`、已访问格、越界格从一开始就没被记成"父亲"。）

**BFS 为什么得到最短路径**：BFS 按距离一层层扩展，一个格子第一次被发现的那一刻，距离就是最短的。

### 1.4 Dijkstra：边有权重时

当每步的代价不同（比如经过靠近障碍的格子更贵），BFS 的"层"不再等于代价，改用**优先队列**，每次取当前代价最小的节点；并在发现更便宜的路径时**更新父指针**（松弛，relaxation）。

> **示例**：节点 S、A、B、G；边 S–A=4，S–B=1，B–A=2，A–G=1，B–G=6。
> - 取 S(0)：A=4(parent S)，B=1(parent S)；
> - 取 B(1)：经 B 到 A 为 $1+2=3<4$，**更新** A=3(parent **B**)；经 B 到 G=$1+6=7$，G=7(parent B)；
> - 取 A(3)：经 A 到 G $=3+1=4<7$，**更新** G=4(parent **A**)；
> - 取 G(4)：结束。倒推：G←A←B←S，总代价 4。
> 注意 A 的父亲从 S 改成了 B——父指针可以被更新，但**总是指向使代价最小的前驱**，链仍然只通向起点。

### 1.5 A*：加一个"指南针"

A* 给每个节点 $f=g+h$：$g$ 是起点到它的已知代价，$h$ 是它到终点的**估计**（启发式）。优先队列按 $f$ 取。若 $h$ 从不高估（可采纳，admissible）且一致，A* 保证最优，并比 Dijkstra 少展开很多节点。

> **示例（同一网格，$h$=曼哈顿距离到 G）**：S: $g=0,h=5,f=5$；(0,1): $g=1,h=4,f=5$；(0,2): $g=2,h=3,f=5$；(0,3): $g=3,h=2,f=5$；(1,2): $g=3,h=3,f=6$。
> 展开顺序：S、(0,1)、(0,2)，再展开 f=5 的 (0,3)（死胡同），再 f=6 的 (1,2)，然后 (2,2)：$g=4,h=1,f=5$，G：$g=5,f=5$。完全不展开 (2,1)。路径同样是 5 步。

---

## 2. 为什么 A* 路径对车不可行

1. **只在格子中心，没有朝向**：路径上相邻两段可以直角拐弯，车的航向无法瞬时变化；
2. **没有曲率约束**：最小转弯半径 $R_{min}=L/\tan\delta_{max}$（例如 0.866 m），格子折线的拐角半径是 0；
3. **没有车体形状**：只检查中心点是否在障碍里；
4. **终点航向**：A* 只管到达位置，到了以后车头朝哪都不管（但窄口泊车类任务需要）。

补救办法"A* + 路径平滑"能平掉部分尖角，但（a）平滑后可能撞墙，（b）无法保证最终满足曲率，（c）车必须倒车调头的场景根本找不出来。所以要把运动学放进搜索本身——这就是 Hybrid-A*。

---

## 3. Hybrid-A* 的核心思想

出处：Dolgov、Thrun、Montemerlo、Diebel 等（Stanford 车辆 Junior 的规划工作，2008–2010 年的会议与期刊论文；具体页码与细节**未核对**，面试中只口头提及"Dolgov et al.'s Hybrid-A* from the Stanford DARPA Urban Challenge work"即可）。

核心就两句话：

1. **搜索的状态是连续的** $(x,y,\theta)$，每个节点存精确的连续位姿，节点之间由**车真能开的弧线**（运动学积分）连接；
2. **格子只用来去重/剪枝**：把 $(x,y,\theta)$ 离散成 3D 格（桶），**每个桶里最多保留代价最低的一个节点**，防止搜索量爆炸。

"Hybrid"（混合）的含义就是：**连续状态 + 离散去重**。

---

## 4. 状态、桶与运动基元

### 4.1 状态与桶

连续状态 $\mathbf s=(x,y,\theta)$，外加方向标志（前进/倒车）。桶下标：
$$i=\lfloor x/\Delta_{xy}\rfloor,\quad j=\lfloor y/\Delta_{xy}\rfloor,\quad k=\lfloor\theta/\Delta_\theta\rfloor.$$

例如 $\Delta_{xy}=0.2\ \mathrm m$，$\Delta_\theta=5^\circ$（72 个朝向 bin）。**桶是用来判断"这个位置/朝向的组合是不是已经有更好的节点来过"**，节点本身仍保存连续值（所以路径不会被格子"量化"）。

你笔记里问的"桶是怎么用的"：

- 展开出一个子节点后，计算它所在桶 $(i,j,k)$；
- 若桶里还没有节点 → 放进去；
- 若已有节点：新节点的 $g$ 更小 → 替换；否则丢弃新节点；
- 这样每个桶最多一个活跃节点，总节点数 $\le$ 桶数。

### 4.2 运动基元

从节点 $(x,y,\theta)$ 出发，固定一小段弧长 $s$，取若干个离散的曲率（转向角）：

- 转向集合：$\delta\in\{-\delta_{max},\ -\delta_{max}/2,\ 0,\ +\delta_{max}/2,\ +\delta_{max}\}$（也可只用 3 个或更多）；
- 方向集合：前进（$s>0$）、倒车（$s<0$）；
- 对每个 $(\delta,\text{方向})$，用 [02 章 7.1](./02_车辆运动学与里程计.md) 的**圆弧精确积分**得到子节点：
$$\kappa=\frac{\tan\delta}{L},\quad \theta'=\theta+\kappa s,\quad x'=x+\frac{\sin\theta'-\sin\theta}{\kappa},\quad y'=y+\frac{\cos\theta-\cos\theta'}{\kappa}\ (\kappa\to0\text{ 时走直线}).$$

**步长 $s$ 怎么选**：必须使子节点**能走出自己的桶**，否则子节点和父节点落在同一桶里，被"去重"当成已访问，搜索无法前进。一般取 $s>\sqrt2\,\Delta_{xy}$。例：$\Delta_{xy}=0.2\Rightarrow s>0.283$，取 $s=0.3\ \mathrm m$。

---

## 5. 一步扩展的完整手算

**设定（教学数值）**：$L=0.5\ \mathrm m$，$\delta_{max}=30^\circ\Rightarrow\kappa_{max}=\tan30^\circ/0.5=1.1547$；$\Delta_{xy}=0.2$，$\Delta_\theta=5^\circ$；$s=0.3\ \mathrm m$；父节点位姿 $(1.00,\ 1.00,\ 30^\circ)$，$g_{parent}=2.0$，上一段曲率 $\kappa_{prev}=0$。父节点所在桶：$(i,j,k)=(\lfloor5.0\rfloor,\lfloor5.0\rfloor,\lfloor6.0\rfloor)=(5,5,6)$。

**三个前进基元**（$\theta=30^\circ=0.5236\ \mathrm{rad}$，$\sin=0.5$，$\cos=0.8660$）：

**(a) 左转（$\kappa=+1.1547$）**：$\Delta\theta=\kappa s=0.3464\ \mathrm{rad}=19.85^\circ$，$\theta'=0.8700\ \mathrm{rad}=49.85^\circ$。
$$x'=1+\frac{\sin0.8700-\sin0.5236}{1.1547}=1+\frac{0.7643-0.5}{1.1547}=1.2289,\quad y'=1+\frac{\cos0.5236-\cos0.8700}{1.1547}=1+\frac{0.8660-0.6448}{1.1547}=1.1916.$$
桶：$i=\lfloor6.14\rfloor=6,\ j=\lfloor5.96\rfloor=5,\ k=\lfloor9.97\rfloor=9$ → $(6,5,9)$。

**(b) 直行（$\kappa=0$）**：$x'=1+0.3\cos30^\circ=1.2598,\ y'=1+0.3\sin30^\circ=1.1500,\ \theta'=30^\circ$。桶：$(6,5,6)$。

**(c) 右转（$\kappa=-1.1547$）**：$\theta'=0.5236-0.3464=0.1772\ \mathrm{rad}=10.15^\circ$。
$$x'=1+\frac{\sin0.1772-0.5}{-1.1547}=1+\frac{0.1763-0.5}{-1.1547}=1.2804,\quad y'=1+\frac{0.8660-\cos0.1772}{-1.1547}=1+\frac{0.8660-0.9843}{-1.1547}=1.1025.$$
桶：$(6,5,2)$。

**验算**（弦长法）：圆弧弦长 $=\frac{2}{|\kappa|}\sin\frac{|\Delta\theta|}{2}=\frac{2}{1.1547}\sin0.1732=0.2985$（比弧长 0.3 短），方向 $\theta+\Delta\theta/2$。左转：$x'=1+0.2985\cos0.6968=1.2289$ ✓。

**观察**：三个子节点的 $(x,y)$ 格都是 $(6,5)$，**同一个 xy 格**，但朝向 bin 不同（9、6、2），所以占据三个不同的 3D 桶——这正是为什么要把朝向放进桶里：同一个位置，朝向不同，后续能去的地方完全不同。三者的 $x$ 格都从 5 变成 6，证明 $s=0.3>\sqrt2\times0.2$ 确实走出了父亲的桶。

**代价（定义见第 6 节，示例权重）**：$c_{len}=1$（前进）；$w_{steer}=0.5$；$w_{change}=0.3$；障碍项 $o$ 取示例值 (a) 0.00、(b) 0.02、(c) 0.10。
$$\Delta g=s\cdot c_{dir}+w_{steer}|\kappa|s+w_{change}|\kappa-\kappa_{prev}|+o.$$
- (a) 左：$0.3+0.5\times1.1547\times0.3+0.3\times1.1547+0=0.3+0.1732+0.3464=0.8196$；
- (b) 直：$0.3+0+0+0.02=0.3200$；
- (c) 右：$0.3+0.1732+0.3464+0.10=0.9196$。

**启发式（示例数值，非真实计算）**：目标 $(4,3,90^\circ)$；各子节点的 $h_1$（忽略障碍的 Reeds-Shepp 长度）与 $h_2$（考虑障碍的 2D 网格距离），取 $h=\max(h_1,h_2)$：

| 子节点 | $g=2.0+\Delta g$ | $h_1$ | $h_2$ | $h$ | $f=g+h$ | 桶 |
|---|---|---|---|---|---|---|
| (a) 左 | 2.8196 | 3.40 | 3.60 | 3.60 | 6.42 | (6,5,9) |
| (b) 直 | 2.3200 | 3.35 | 3.45 | 3.45 | 5.77 | (6,5,6) |
| (c) 右 | 2.9196 | 3.55 | 3.45 | 3.55 | 6.47 | (6,5,2) |
| (d) 倒车直行 | $2.0+0.3\times2+\text{换向罚}1.0=3.60$ | 3.80 | 3.80 | 3.80 | 7.40 | — |

优先队列下一次取出 $f$ 最小的 (b)。（倒车项里 $c_{dir}=2$ 表示倒车长度加倍惩罚，另有一次性的换向惩罚。）

---

## 6. 代价函数

$g$ 代价是各段代价之和。典型项（Nav2 Smac 等实现中也有对应参数）：

| 项 | 含义 | 效果 |
|---|---|---|
| 长度项 $s$ | 路径长度 | 最短 |
| 倒车惩罚 | 倒车段长度乘系数 | 尽量前进 |
| 换向惩罚 | 前进↔倒车切换（cusp）一次性罚 | 减少折返 |
| 转向惩罚 | $|\kappa|$ 或非直行 | 减少转弯 |
| 转向变化惩罚 | $|\kappa-\kappa_{prev}|$ | 路径平顺（曲率连续） |
| 障碍代价 | 从代价地图（或 EDT）读该位姿的代价 | 离墙远 |

**硬约束**：footprint 碰撞（带朝向，见 [07 章](./07_代价地图与车体外形.md)）→ 直接丢弃该子节点；曲率 $\le\kappa_{max}$ → 运动基元中的转向集合本身保证。**软约束**：其余各项。

---

## 7. 启发式函数

启发式 $h$ 的目标：给出"从这个节点到目标还需要多少代价"的**不高估**的估计。Hybrid-A* 用两个启发式的最大值，因为二者各抓一个方面：

- **$h_1$：考虑运动学、忽略障碍**（non-holonomic-without-obstacles）。用 Reeds-Shepp（可倒车）或 Dubins（只能前进）最短曲线长度。它知道"要先绕出弯"，但不知道前方有墙。可预先离线算好查表（以相对位姿为索引）；
- **$h_2$：考虑障碍、忽略运动学**（holonomic-with-obstacles）。在 2D 栅格上从目标反向做一次 Dijkstra/A*，得到每个格子到目标的最短（避障）距离。它知道墙，但不知道车不能原地转；
- **$h=\max(h_1,h_2)$**：两者都不高估真实代价，所以最大值也不高估，而且比任何一个都更紧。

> **示例**：车头朝向和目标反向、中间有一堵墙。$h_1$ 只看到"要掉头"，给出 3.4；$h_2$ 看到墙，绕路 3.6。取 3.6，比 $h_1$ 更接近真实，减少了搜索时撞进死胡同的展开数。

注意：由于离散桶和最大值，Hybrid-A* 的 $h$ 严格说不再保证最优（它是不完备、不最优的，但实用）。

---

## 8. 解析扩展（Reeds-Shepp / Dubins）与 Dubins 手算

### 8.1 为什么需要

纯靠基元，路径末端往往"差一点点"到不了目标的**精确位姿**（因为桶是离散的，基元也是离散的）。解决：每展开若干个节点（或每次靠近目标时），尝试从当前节点直接用一条**解析曲线**连到目标位姿。如果这条曲线无碰撞，搜索立即结束。

### 8.2 曲线族

- **Dubins**：只能前进、最小半径 $R$，最短路径由"圆弧—直线—圆弧"（CSC）或"圆弧—圆弧—圆弧"（CCC）构成，共 6 种：LSL、RSR、LSR、RSL、RLR、LRL；
- **Reeds-Shepp**：允许倒车，共几十种形式（文献中为 46–48 种，含换向点）。

### 8.3 Dubins LSR 完整手算

**设定**：起点 $(0,0,0^\circ)$，终点 $(4,1,0^\circ)$，最小半径 $R=1$（示例）。先左转、再直行、再右转（LSR）。

1. 起点左转圆圆心 $C_1=(0,1)$；终点右转圆圆心：终点航向 0°，右侧是 $-y$ 方向，$C_2=(4,1-1)=(4,0)$；
2. 圆心距 $d=\sqrt{4^2+(0-1)^2}=\sqrt{17}=4.123$；圆心连线方向 $\varphi=\mathrm{atan2}(-1,4)=-14.04^\circ$；
3. LSR 是**内公切线**，直线段长度 $\ell=\sqrt{d^2-(2R)^2}=\sqrt{17-4}=3.606$；
4. 切线与圆心连线的夹角 $\beta=\arcsin\frac{2R}{d}=\arcsin0.4851=29.02^\circ$；直线段航向 $\theta_s=\varphi+\beta=14.98^\circ=0.2615\ \mathrm{rad}$；
5. 第一段左转弧：航向从 0 增到 $\theta_s$，弧长 $R\theta_s=0.2615$；第三段右转弧：从 $\theta_s$ 回到 0，弧长 $0.2615$；
6. 总长 $=0.2615+3.606+0.2615=4.129$。

**验算**：横向位移 $=R(1-\cos\theta_s)+\ell\sin\theta_s+R(1-\cos\theta_s)=0.034+0.932+0.034=1.000$ ✓；纵向 $=R\sin\theta_s\times2+\ell\cos\theta_s=0.2585\times2+3.483=4.000$ ✓。（对比直线距离 $\sqrt{17}=4.123$，只长了 0.006。）

这个例子说明：当目标只是横向偏移 1 m 时，Dubins 曲线的"弯"非常柔和（航向只偏 15°），完全满足半径限制。当目标在车身侧后方，则需要 CCC 或倒车。

---

## 9. 算法流程与伪代码

```text
输入: 起点 s0, 终点 sg, 代价地图, 车参数 (L, δmax, footprint), 离散参数
OPEN  = 优先队列 (按 f = g + h)
BUCKET[i,j,k] = 记录每个桶当前最优节点

初始化: 节点 n0 (s0, g=0, h=H(s0)) 放入 OPEN 和 BUCKET

while OPEN 非空:
    n = OPEN.pop_min_f()
    if n 已被更优节点替代: continue
    if n 在目标容差内 (位置+航向): 回溯父指针, 返回路径
    # 解析扩展: 每 N 个展开节点, 或距离目标足够近
    if 满足触发条件:
        path = ReedsShepp(n.s, sg)
        if path 无碰撞: 返回 回溯(n) + path
    for 每个 (δ, dir) in 运动基元:
        s' = 圆弧积分(n.s, δ, dir, step)
        if footprint 在 s' 处碰撞 (带朝向): continue
        g' = n.g + 代价(δ, dir, n.δ, n.dir, 障碍代价)
        b = 桶(s')
        if b 无节点 或 g' < BUCKET[b].g:
            新节点 n' (s', g', h=max(h1,h2), parent=n)
            BUCKET[b] = n'; OPEN.push(n')
返回 失败
```

**教学示例代码（Python，非项目源码，省略 h 的具体实现与碰撞函数）**：

```python
import heapq, math

def hybrid_astar(start, goal, collide, heuristic, L=0.5, dmax=math.radians(30),
                 step=0.3, res_xy=0.2, res_th=math.radians(5)):
    """start/goal: (x, y, theta). collide(state)->bool. heuristic(state)->float."""
    def bucket(s):
        return (int(s[0] // res_xy), int(s[1] // res_xy), int((s[2] % (2*math.pi)) // res_th))

    def arc(s, delta, sgn):
        k = math.tan(delta) / L
        d = sgn * step
        th2 = s[2] + k * d
        if abs(k) < 1e-9:
            return (s[0] + d * math.cos(s[2]), s[1] + d * math.sin(s[2]), s[2])
        return (s[0] + (math.sin(th2) - math.sin(s[2])) / k,
                s[1] + (math.cos(s[2]) - math.cos(th2)) / k, th2)

    open_q, best = [], {}
    n0 = (heuristic(start), 0.0, start, None)         # (f, g, state, parent)
    heapq.heappush(open_q, (n0[0], id(n0), n0))
    best[bucket(start)] = 0.0
    while open_q:
        _, _, (f, g, s, parent) = heapq.heappop(open_q)
        if math.hypot(s[0]-goal[0], s[1]-goal[1]) < 0.3:
            return s                                   # trace back parents in real code
        for delta in (-dmax, -dmax/2, 0.0, dmax/2, dmax):
            for sgn in (+1, -1):
                s2 = arc(s, delta, sgn)
                if collide(s2):
                    continue
                g2 = g + step * (1.0 if sgn > 0 else 2.0) + 0.5 * abs(delta) * step
                b = bucket(s2)
                if b not in best or g2 < best[b]:
                    best[b] = g2
                    n2 = (g2 + heuristic(s2), g2, s2, (s, parent))
                    heapq.heappush(open_q, (n2[0], id(n2), n2))
    return None
```

---

## 10. 后处理：平滑与曲率检查

### 10.1 为什么还要平滑

桶离散化和有限的转向集合，使得路径有轻微锯齿，曲率不连续（转向角会阶跃）。控制器喜欢曲率连续的路径。后处理常用梯度下降/共轭梯度，最小化：
$$J=w_o\sum_i\phi_{obs}(\mathbf p_i)+w_\kappa\sum_i\max\!\big(0,\ |\kappa_i|-\kappa_{max}\big)^2+w_s\sum_i\|\mathbf p_{i+1}-2\mathbf p_i+\mathbf p_{i-1}\|^2.$$
三项分别：离障碍远、曲率不超限、路径光滑（二阶差分小）。

> **示例手算（平滑项）**：三点 $(0,0),(1,0.2),(2,0.8)$：二阶差分 $=(2,0.8)-2(1,0.2)+(0,0)=(0,\ 0.4)$，范数平方 $0.16$。点向 $(1,0.4)$ 移动后，差分变成 $(0,0)$，该项为 0——平滑项的梯度把中间点推向两端点的连线中点。

### 10.2 用三点法检查曲率是否满足最小半径

三个相邻路径点 $\mathbf p_1,\mathbf p_2,\mathbf p_3$，边长 $a=|\mathbf p_2\mathbf p_1|,\ b=|\mathbf p_3\mathbf p_2|,\ c=|\mathbf p_3\mathbf p_1|$，面积 $A$，则（Menger 曲率）
$$\kappa=\frac{4A}{abc}.$$

> **示例**：$(0,0),(1,0.2),(2,0.8)$：$a=\sqrt{1.04}=1.0198,\ b=\sqrt{1.36}=1.1662,\ c=\sqrt{4.64}=2.1541$；$A=\tfrac12|0\cdot(0.2-0.8)+1\cdot(0.8-0)+2\cdot(0-0.2)|=0.2$；$\kappa=\frac{0.8}{1.0198\times1.1662\times2.1541}=0.312\ \mathrm{m^{-1}}$，即半径 3.20 m。与 $R_{min}=0.866\ \mathrm m$ 比较，满足。

沿整条路径算 $\kappa_i$，检查 $\max|\kappa_i|\le\kappa_{max}$（并留余量，如 $0.9\kappa_{max}$）——这是验证 CV 中 "minimum-turning-radius constraints" 的最直接工具。

---

## 11. 倒车与换向

- 倒车基元 $s<0$ 让搜索能找到"三点掉头"（前进 → 倒车 → 前进），在窄口、泊车类任务中必须；
- **换向点（cusp）**：路径在此处方向反转。输出的 `Path` 里每个位姿的 `yaw` 是**车头朝向**（倒车时车头朝向与运动方向相反）。下游必须能识别 cusp：局部规划（TEB）要允许倒车初始化，控制器要在 cusp 处让速度先降到 0 再反向，否则会"冲过头"或抖动；
- 倒车惩罚与换向惩罚决定路径是否愿意倒车：太小 → 无谓折返；太大 → 该倒时不倒，找不到路径。

---

## 12. 参数与调参（以 Nav2 Smac 为例）

> CV 只写 Hybrid-A*；本项目取的典型实现是 Nav2 Smac 的 Hybrid-A*（`SmacPlannerHybrid`）。下表参数名取自 Nav2 Smac Planner 公开文档，列出以便理解各项的作用。

| 参数 | 作用 | 调参提示 |
|---|---|---|
| `minimum_turning_radius` | $R_{min}$ | 取略大于真实值（留余量） |
| `motion_model_for_search` | `DUBIN`（只前进）/ `REEDS_SHEPP`（可倒车） | 需倒车选后者 |
| `angle_quantization_bins` | $\theta$ 的离散 bin 数 | 大→精但慢（如 72） |
| `reverse_penalty` | 倒车惩罚 | |
| `change_penalty` | 方向变化惩罚 | |
| `non_straight_penalty` | 非直行惩罚 | |
| `cost_penalty` | 代价地图代价权重 | 大→远离障碍 |
| `analytic_expansion_ratio` | 解析扩展的触发频率 | |
| `max_iterations`, `tolerance` | 搜索上限、位置容差 | 防无限搜索 |
| `allow_unknown` | 是否允许穿越未知 | 一般关闭 |
| `smooth_path` | 是否做平滑 | |

**调参顺序建议**：先让 `minimum_turning_radius` 和 footprint 与实车一致（它们决定可行性）→ 再调离散分辨率（决定速度与精度）→ 最后调各惩罚（决定风格）。

---

## 13. 失败模式与故障现象

| 现象 | 可能原因 | 处理 |
|---|---|---|
| 找不到路径 | 终点/起点在致命区；半径过大；窄通道；起点航向与通道垂直又无倒车 | 检查起终点代价；开启倒车；降低膨胀；放宽容差 |
| 搜索很慢/超时 | 分辨率太细；θ bin 太多；地图太大；启发式弱 | 降分辨率；限制范围；预计算启发式；降采样 |
| 路径锯齿 | 转向集太小；步长太大；无后处理 | 增加基元；平滑 |
| 路径贴墙 | 障碍代价权重小；膨胀小 | 增大 `cost_penalty` |
| 路径穿墙 | footprint 未生效；代价地图不一致（[07 章](./07_代价地图与车体外形.md)） | 核对地图元信息和 footprint |
| 曲率超限（车跟不上） | 平滑后曲率变大；$R_{min}$ 设小 | 平滑中加曲率项；留余量 |
| 到终点航向不对 | 解析扩展未触发；容差设大 | 调触发条件；收紧航向容差 |
| 大量无谓倒车 | 倒车惩罚太小 | 增大 |
| 重规划抖动（路径来回变） | 起点噪声；规划频率高 | 规划结果滞回；仅在偏离/受阻时重规划 |

---

## 14. 对比：A*、Hybrid-A*、状态格、RRT*

| 方法 | 状态 | 运动学可行 | 最优性 | 速度 | 特点 |
|---|---|---|---|---|---|
| 网格 A* | $(x,y)$ 格 | 否 | 离散最优 | 快 | 需后处理 |
| **Hybrid-A\*** | 连续 $(x,y,\theta)$ + 桶去重 | **是** | 不保证最优 | 中 | 路径可行、易加倒车和代价 |
| 状态格（state lattice） | 预设的运动基元网格 | 是 | 基元集内最优 | 中 | 基元离线设计，形状规则 |
| RRT* | 连续随机采样 | 取决于扩展函数 | 渐近最优 | 慢收敛 | 灵活，路径不稳定 |

选择 Hybrid-A* 的理由（面试说法）：对车式机器人，它在**可行性、实现复杂度、搜索效率**之间取得平衡，输出的路径可直接被后续局部规划（TEB）和 MPC 接收。

---

## 15. 规划耗时、触发与降级：把全局规划放进整个节拍

CV 只写了 Hybrid-A* 能"在车体外形、最小转弯半径、障碍代价约束下生成可行全局路径"。工程上还要回答：它什么时候被调用、算多久、算不出来怎么办。本节的数字是本项目的设计取值（自查表第 16 项），不声称实测。

### 15.1 什么时候规划

| 触发 | 动作 | 说明 |
|---|---|---|
| 收到新目标 | 规划一次 | 由 Nav2 的行为树 `ComputePathToPose` 节点调用规划器服务器 |
| 当前路径被新障碍阻挡 | 以 1 Hz 重规划 | 局部代价地图检测到全局路径上有致命格；重规划的起点取当前位姿 |
| 定位发生大修正（已确认的重定位） | 立即重规划 | 全局路径在 odom 系的表达已整体改变 |
| 控制层报告"卡住"（进度低于阈值 5 s） | 清局部代价地图后重规划 | 恢复行为的第一步 |

规划是**按需触发**，不是 20 Hz 的循环。这点要和 TEB、MPC 区分：TEB 与 MPC 每个周期都要运行，规划只在需要时运行，所以规划的"耗时"关系到的是响应延迟，而不是周期预算。

### 15.2 耗时预算与超时降级

| 场景 | 典型耗时 | 说明 |
|---|---|---|
| 开阔区域、10–20 m | 20–40 ms | 解析扩展很快命中，展开节点数小 |
| 走廊、有拐角 | 40–80 ms | 需要多次换向基元 |
| 窄口调头（需倒车） | 约 150–300 ms | 要展开多个航向 bin，换向惩罚使搜索更广 |
| 超时上限 | 1 s | 超过则返回失败，由行为树走恢复分支 |

降级链（超时或失败后依次尝试）：

1. 起点向外微调（最大 0.15 m）以脱离致命区；
2. 放宽 `change_direction_penalty`（1.5→1.0）并允许倒车；
3. 以更细的航向离散（72→96 bin）重试一次（只在失败时，因为耗时增加）；
4. 仍失败：等待 1 s 后让行为树执行恢复（清代价地图、后退 0.3 m、再规划）；
5. 连续 3 次失败：停车并向任务层报告"目标不可达"。

### 15.3 规划与控制的接口频率

规划器输出的是一条全局路径（`nav_msgs/Path`），只在重规划时更新。TEB 每 100 ms 取该路径中**车前方一定长度（本项目取 3 m）**的一段作为参考，再结合局部代价地图优化；全局路径的更新频率（1 Hz 上限）远低于 TEB，所以 TEB 看到的路径在两次重规划之间是不变的，路径的跳变只会出现在重规划的瞬间，并由 TEB 的时间参数化轨迹平滑吸收（见 [09 章](./09_TEB局部轨迹优化.md)）。

> **面试要点**：当被问"你的规划频率是多少"时，不要回答一个 Hz 数字了事。完整的回答是："全局规划是事件触发的，收到目标时一次，路径被阻挡时最多 1 Hz；典型 20–80 ms，窄口调头最坏约 300 ms，上限 1 s；局部 TEB 10 Hz；MPC 20 Hz。"

---

## 16. 面试问答 / 要点速记 / 自测题 / 链接

### 面试问答

**Q1. How does Hybrid A-star differ from A-star?**

> A-star searches on grid cells, so the path has no heading and corners with zero radius. Hybrid A-star keeps a continuous $(x,y,\theta)$ state in every node and expands it with short arcs from the vehicle kinematic model, within the steering limit. The grid is only used to prune: each 3D cell keeps the cheapest node. So every segment of the path is something the car can actually drive.

*追问*：Is it optimal? —— 不保证；离散化和启发式让它是"可行、近似好"，常配合平滑。

**Q2. What heuristics do you use and why two?**

> One ignores obstacles but respects the non-holonomic motion, using Reeds–Shepp length. The other ignores the vehicle motion but respects obstacles, using a 2D grid shortest path from the goal. Each captures what the other misses; the maximum of the two is still admissible and much tighter.

**Q3. What is analytic expansion?**

> Every few expansions, or when close to the goal, I try to connect the current node to the goal with a Reeds–Shepp curve. If that curve is collision-free, the search ends immediately. This gives an exact goal pose, which the discrete primitives rarely reach.

**Q4. How did you make sure the path respects the minimum turning radius?**

> The motion primitives never use more than the maximum steering angle, so each arc already respects it. After smoothing I check the curvature along the path, using three-point curvature, against the maximum curvature with a margin.
> 本项目取值约：车体最小转弯半径 $0.40/\tan30^\circ\approx0.69$ m，规划用 `minimum_turning_radius` 0.80 m 留余量；转向基元为左、直、右三种弧（含倒车共六种）；代价地图分辨率 0.05 m，航向离散 72 个 bin（5°）；footprint 为 0.60 m × 0.38 m 的矩形（后轴为原点）。

### 本章要点速记

1. BFS 回溯成立：每格仅一个父亲、父亲先被发现、起点无父亲；Dijkstra 的父指针可被松弛更新。
2. 普通 A* 对车不可行：无朝向、无曲率、无车体形状。
3. Hybrid-A*：连续 $(x,y,\theta)$ 节点 + 弧线基元 + 3D 桶去重。
4. 步长必须大于 $\sqrt2\Delta_{xy}$，否则子节点落回父桶。
5. 同一 xy 格不同朝向占不同桶。
6. $g$ = 长度 + 倒车/换向/转向/变化罚 + 障碍代价；碰撞用带朝向的 footprint。
7. $h=\max(h_1,h_2)$：非完整无障碍（RS/Dubins）与完整有障碍（2D 栅格）。
8. 解析扩展用 RS/Dubins 直接连目标；Dubins LSR 示例总长 4.129。
9. 平滑后用三点曲率验证 $\kappa\le\kappa_{max}$。
10. 倒车与 cusp 需下游配合。

### 自测题

1. 为什么 BFS 回溯父指针不会死循环？
2. 步长 0.15 m、$\Delta_{xy}=0.2$ 会出什么问题？
3. 对 $(0,0),(2,0),(4,1)$ 计算 Menger 曲率。（答：$a=2,\ b=\sqrt5=2.236,\ c=\sqrt{17}=4.123$；$A=\tfrac12|0(0-1)+2(1-0)+4(0-0)|=1$；$\kappa=4/(2\times2.236\times4.123)=0.217$。）
4. $h_1$ 与 $h_2$ 各有什么盲区？
5. 倒车惩罚设成 0 会发生什么？

### 相关链接

- 上一章：[07 代价地图与车体外形](./07_代价地图与车体外形.md)　下一章：[09 TEB 局部轨迹优化](./09_TEB局部轨迹优化.md)
- 工程化（频率、延迟、调参过程）：[20 工程化实践](./20_工程化实践_频率周期与问题解决.md)
- 返回：[README](./README.md)
- 学习笔记：[../../05_学习笔记归档/P5/P5_本对话问答逐字稿.md](../../05_学习笔记归档/P5/P5_本对话问答逐字稿.md)
