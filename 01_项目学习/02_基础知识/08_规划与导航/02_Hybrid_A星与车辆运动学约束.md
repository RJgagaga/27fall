# Hybrid A* 与车辆运动学约束

> **一句话定位**：普通栅格 A* 把机器人当成能朝任意方向移动的质点；车式机器人不能横着走、转弯半径有下限。Hybrid A* 让搜索的**状态**变成连续的 $(x,y,\theta)$，**动作**变成满足运动学的短圆弧，从而直接产出车辆能开出来的全局路径。这就是 P5 CV 中 "Hybrid-A* global planning" 的原理。

## 本章学完你能……

- 解释**非完整约束**（nonholonomic constraint）是什么，从"轮子不能侧滑"推出 $\dot x\sin\theta-\dot y\cos\theta=0$，并由自行车模型推出最小转弯半径 $R_{min}=L/\tan\delta_{max}$。
- 说清楚 Hybrid A* 的核心思想：**连续状态附着在离散格子上，每个格子只保留一个最优节点**，以及它为什么不保证完备和最优。
- 推导运动基元（圆弧）的闭式位姿增量，并**手算一步扩展**：前进 / 后退 × 三个转向角共 6 个子节点的位姿、代价、启发值、格子索引。
- 理解两个互补的启发函数（holonomic-with-obstacles、non-holonomic-without-obstacles），说明为什么取最大值是可采纳的。
- 手算一条 Dubins 路径（LSR 型，长度 4.511），解释 Reeds-Shepp 与 Dubins 的区别，以及"解析扩展"如何让搜索提前收尾。
- 用多圆近似做车辆 footprint 碰撞检测，并说明路径平滑（曲率、光滑项、障碍项）的做法。
- 用英文回答 Hybrid A* 相关面试问题。

## 前置知识

- A* 与启发函数：[图搜索：Dijkstra 与 A*](./01_图搜索_Dijkstra_A星.md)
- 车辆运动学（自行车模型、差速、阿克曼）：[差速、麦克纳姆、全向轮与车辆运动学](../02_力学与动力学建模/04_差速_麦克纳姆_全向轮与车辆运动学.md)
- 旋转矩阵与位姿复合：[坐标系、旋转矩阵、齐次变换](../03_运动学与坐标变换/01_坐标系_旋转矩阵_齐次变换.md)

## 目录

1. 为什么栅格 A* 不够
2. 非完整约束与自行车模型
3. Hybrid A* 的核心思想
4. 运动基元的闭式推导
5. 手算：一步扩展
6. 代价函数与启发函数
7. Dubins 与 Reeds-Shepp：解析扩展
8. 碰撞检测与车辆 footprint
9. 路径平滑
10. 在本项目里怎么用
11. 常见错误与故障现象
12. 面试怎么问、怎么答
13. 本章要点速记、自测题、相关链接

---

## 1. 为什么栅格 A* 不够

栅格 A* 的状态是格子 $(i,j)$，动作是走到邻格。它给出的路径：

1. **不含朝向**：下一个格子在左边，车却朝前，车需要原地转向才能执行；
2. **转弯尖锐**：90° 折线，车的最小转弯半径装不下；
3. **可以"侧移"**：向左挪一格对车来说是不可能的动作。

所以栅格路径直接交给车跟踪，要么跟不上，要么要靠局部规划器去"修补"，在狭窄空间容易失败。

---

## 2. 非完整约束与自行车模型

### 2.1 它要解决什么问题

形式化"车不能侧移、转弯有限"，得到可以直接用在搜索里的运动方程。

### 2.2 符号与单位

| 符号 | 含义 | 单位 |
|---|---|---|
| $(x,y)$ | 后轴中心在世界系的位置 | m |
| $\theta$ | 车身朝向（航向），逆时针为正 | rad |
| $v$ | 后轴中心沿车身方向的速度，前进为正、倒车为负 | m/s |
| $\delta$ | 前轮等效转向角，左转为正 | rad |
| $L$ | 轴距 | m |
| $\kappa$ | 路径曲率，$\kappa=1/R$ | 1/m |
| $\delta_{max}$ | 最大转向角 | rad |

### 2.3 非完整约束的推导

车轮在地面上**纯滚动不侧滑**：后轴中心的速度方向必须沿车身。车身方向单位向量 $(\cos\theta,\sin\theta)$，垂直方向 $(-\sin\theta,\cos\theta)$。速度沿垂直方向的分量必须为零：

$$
(\dot x,\dot y)\cdot(-\sin\theta,\cos\theta)=0\ \Longleftrightarrow\ \boxed{\dot x\sin\theta-\dot y\cos\theta=0}
$$

这个约束含速度、不能积分成对位置的约束，所以叫**非完整约束**：车可以到达平面上任何位姿（可控），但不能沿任意方向瞬时移动。

### 2.4 自行车模型

把前两轮合并成一个转向轮，后两轮合并成一个后轮。后轴中心速度沿车身；前轮沿 $\delta$ 方向；两轮垂线的交点即**瞬时转动中心**。几何关系 $\tan\delta=L/R$，故

$$
\boxed{\dot x=v\cos\theta,\qquad \dot y=v\sin\theta,\qquad \dot\theta=\frac vL\tan\delta}
$$

**最小转弯半径**：$|\delta|\le\delta_{max}$，所以

$$
R_{min}=\frac L{\tan\delta_{max}},\qquad \kappa_{max}=\frac{\tan\delta_{max}}L
$$

**手算（教学数值）**：轴距 $L=0.5$ m，$\delta_{max}=30°$：$R_{min}=0.5/\tan30°=0.5/0.5774=0.866$ m，$\kappa_{max}=1.155\ \text{1/m}$。这个数用于下面所有例子。

> 另一种常见的参考点是前轴或质心；用后轴时方程最简单，大多数规划器用后轴中心。

---

## 3. Hybrid A* 的核心思想

> 原始文献：D. Dolgov, S. Thrun, M. Montemerlo, J. Diebel，"Practical search techniques in path planning for autonomous driving" 与 "Path planning for autonomous vehicles in unknown semi-structured environments"（IJRR, 2010），斯坦福自动驾驶车辆 Junior 项目。（凭记忆引用，未核对原文。）

### 3.1 状态：连续 + 离散索引

- 每个搜索节点保存的是**连续**位姿 $(x,y,\theta)$（不会被量化到格子中心）；
- 同时把位姿映射到一个**离散格子索引** $(i,j,k)=(\lfloor x/\Delta_{xy}\rfloor,\ \lfloor y/\Delta_{xy}\rfloor,\ \lfloor\theta/\Delta_\theta\rfloor)$，用于判断重复；
- **每个格子最多保留一个节点**（代价最低者）。这样搜索空间的规模有限，同时路径本身是连续、可行的。

### 3.2 动作：运动基元

从当前节点出发，在若干个转向角 $\delta_i$ 和两个方向（前进 / 倒车）下，沿自行车模型积分一小段固定弧长 $s$，得到子节点。对子节点：

1. 碰撞检测（整车 footprint）；
2. 计算 $g$ 和 $h$；
3. 如果所在格子已有节点且新 $g$ 更大，丢弃；否则替换 / 加入 open。

### 3.3 与普通 A* 的对比

| | 栅格 A* | Hybrid A* |
|---|---|---|
| 状态 | 格子 $(i,j)$ | 连续 $(x,y,\theta)$ + 离散索引 |
| 动作 | 邻格移动 | 满足运动学的圆弧 |
| 路径 | 折线 | 光滑、曲率有界 |
| 最优性 | 在图上最优 | **不保证**（每格只留一个节点，离散化） |
| 完备性 | 在图上完备 | **不完备**（可能漏掉需要同格内多种状态的解） |
| 倒车 | 无 | 可以（加倒车基元） |

> **面试要点**：Hybrid A* 给的是"可行且较好"的路径，不是最优路径；它的价值是一定满足运动学约束。

---

## 4. 运动基元的闭式推导

### 4.1 圆弧公式

固定 $\delta$（即固定曲率 $\kappa=\tan\delta/L$）、有符号弧长 $s$（前进 $s>0$，倒车 $s<0$）。$\dot\theta=v\kappa$，所以沿弧长 $\mathrm d\theta=\kappa\,\mathrm ds$，航向变化 $\Delta\theta=\kappa s$。

在**初始车身系**（原点在后轴中心，x 轴沿初始车头方向）下积分：

$$
x_b(s)=\int_0^s\cos(\kappa u)\,\mathrm du=\frac{\sin(\kappa s)}\kappa,\qquad y_b(s)=\int_0^s\sin(\kappa u)\,\mathrm du=\frac{1-\cos(\kappa s)}\kappa
$$

当 $\kappa\to0$ 退化为直线 $x_b=s,\ y_b=0$。

转到世界系（初始位姿 $(x_0,y_0,\theta_0)$）：

$$
\begin{aligned}
x&=x_0+x_b\cos\theta_0-y_b\sin\theta_0\\
y&=y_0+x_b\sin\theta_0+y_b\cos\theta_0\\
\theta&=\theta_0+\kappa s
\end{aligned}
$$

### 4.2 基元集合的选择

典型做法：转向角取 $\{-\delta_{max},\ 0,\ +\delta_{max}\}$ 或更密（如 5 个）；弧长 $s$ 取略大于格子对角线（保证子节点会进入不同的格子）；倒车同样一组但弧长取负。更多基元 → 路径质量更好但搜索更慢。

---

## 5. 手算：一步扩展

### 5.1 设定（教学数值）

- 当前节点位姿 $(x_0,y_0,\theta_0)=(2.0,\ 1.0,\ 90°)$（面向 $+y$）；
- 轴距 $L=0.5$ m，$\delta_{max}=30°$；弧长 $|s|=0.5$ m；
- 离散化：位置格子 $\Delta_{xy}=0.25$ m，航向格子 $\Delta_\theta=10°$；
- 当前上一步状态：直行前进（$\delta_{prev}=0$，档位前进）；
- 目标位姿 $(2.0,\ 4.0,\ 90°)$（正前方）。

### 5.2 先算转弯基元（$\delta=\pm30°$）

$$
\kappa=\frac{\tan30°}{0.5}=\frac{0.57735}{0.5}=1.1547\ \text{1/m},\qquad \Delta\theta=\kappa s=1.1547\times0.5=0.5774\ \text{rad}=33.08°
$$

车身系位移（前进 $s=+0.5$，左转 $\delta=+30°$）：

$$
x_b=\frac{\sin0.5774}{1.1547}=\frac{0.5458}{1.1547}=0.4727,\qquad y_b=\frac{1-\cos0.5774}{1.1547}=\frac{1-0.8379}{1.1547}=0.1404
$$

转到世界系（$\theta_0=90°$：$\cos=0,\sin=1$）：

$$
x=2.0+0.4727\cdot0-0.1404\cdot1=1.8596,\qquad y=1.0+0.4727\cdot1+0.1404\cdot0=1.4727
$$

$\theta=90°+33.08°=123.08°$。即左转后车头向左偏，位置向左前方。

右转（$\delta=-30°$）关于车身轴对称：$(2.1404,\ 1.4727)$，$\theta=56.92°$。

直行：$(2.0,\ 1.5)$，$\theta=90°$。

倒车（$s=-0.5$）：$\Delta\theta=\kappa s$ 符号随 $s$ 反转。倒车 + 左转（$\delta=+30°$）：$\Delta\theta=-33.08°$，$x_b=\sin(-0.5774)/1.1547=-0.4727$，$y_b=(1-\cos0.5774)/1.1547=+0.1404$，所以 $x=2.0-0.1404=1.8596$，$y=1.0-0.4727=0.5273$，$\theta=56.92°$。倒车 + 右转同理对称。倒车直行：$(2.0,\ 0.5)$，$\theta=90°$。

### 5.3 代价与启发

代价（教学定义，权重为示例）：

$$
\Delta g=w_{dir}\,|s|+w_\delta\frac{|\delta|}{\delta_{max}}+w_{\Delta\delta}\frac{|\delta-\delta_{prev}|}{\delta_{max}}+w_{gear}\cdot[\text{换挡}]
$$

取 $w_{dir}=1$（前进）/ 2（倒车），$w_\delta=0.2$，$w_{\Delta\delta}=0.1$，$w_{gear}=1.0$。

启发取 $h=\max\big(h_{nonholo},\ h_{holo}\big)$。这里无障碍，所以 $h_{holo}$ 是到目标的欧氏距离；$h_{nonholo}$ 用 Dubins 长度（最小转弯半径 0.866 m，假设车只前进，所以它是严格的非完整下界；若允许倒车，应改用 Reeds-Shepp，见第 7 节）。

| 子节点 | 位姿 $(x,\ y,\ \theta)$ | $\Delta g$ 计算 | $g$ | $h$ | $f$ | 格子索引 $(i,j,k)$ |
|---|---|---|---|---|---|---|
| 前进直行 | (2.000, 1.500, 90.0°) | $0.5$ | 0.50 | 2.500 | **3.000** | (8, 6, 9) |
| 前进左转 | (1.860, 1.473, 123.1°) | $0.5+0.2+0.1$ | 0.80 | 2.575 | 3.375 | (7, 5, 12) |
| 前进右转 | (2.140, 1.473, 56.9°) | 同上 | 0.80 | 2.575 | 3.375 | (8, 5, 5) |
| 倒车直行 | (2.000, 0.500, 90.0°) | $2\times0.5+1.0$（换挡） | 2.00 | 3.500 | 5.500 | (8, 2, 9) |
| 倒车左转 | (1.860, 0.527, 56.9°) | $1.0+0.2+0.1+1.0$ | 2.30 | 3.500 | 5.800 | (7, 2, 5) |
| 倒车右转 | (2.140, 0.527, 123.1°) | 同上 | 2.30 | 3.500 | 5.800 | (8, 2, 12) |

格子索引示例：前进左转 $(1.8596,1.4727,123.08°)$：$\lfloor1.8596/0.25\rfloor=7$，$\lfloor1.4727/0.25\rfloor=5$，$\lfloor123.08/10\rfloor=12$。

### 5.4 读结果

1. 目标在正前方，**前进直行** $f=3.0$ 最小，先被展开——与直觉一致。
2. 左 / 右转 $h$ 变大（2.575 vs 2.5）：车头偏离目标方向，需要再转回来，**非完整启发捕捉到了"朝向代价"**；只用欧氏距离时（2.531）看不出这一点。
3. 倒车代价明显更高（倒车权重 + 换挡罚），只有在前方被堵时才会被选中。
4. 如果两个子节点落进同一格子，只保留 $g$ 较小的一个——这就是 Hybrid A* "每格一个节点" 的含义。

---

## 6. 代价函数与启发函数

### 6.1 代价函数的典型项

| 项 | 作用 |
|---|---|
| 弧长 | 路径尽量短 |
| 倒车惩罚（权重 $>1$） | 优先前进 |
| 换挡（方向切换）惩罚 | 减少前进 / 倒车来回切换 |
| 转向角惩罚 | 偏好小转向、直行 |
| 转向角变化惩罚 | 路径平滑，避免急打方向 |
| 障碍代价 | 离障碍越近代价越大，路径会自然与障碍保持距离 |

障碍代价来自代价地图（如膨胀层的 $0\sim254$ 值，见 [代价地图章](./03_代价地图_局部规划_TEB.md)）。注意权重之间要相互平衡：障碍惩罚过大会让车绕远路；过小会贴障碍。

### 6.2 两个互补的启发函数

**(1) Non-holonomic-without-obstacles**：忽略障碍，只考虑运动学。从当前位姿 $(x,y,\theta)$ 到目标位姿 $(x_g,y_g,\theta_g)$ 的 Reeds-Shepp（或 Dubins）最短路径长度。可以**预先计算查表**。它能感知朝向：车头背对目标时代价大。

**(2) Holonomic-with-obstacles**：忽略运动学（把车当质点），只考虑障碍。在 2D 栅格上从**目标**出发跑一遍 Dijkstra（或 A*），得到每个格子到目标的最短绕障距离。它能感知障碍：死胡同里的格子代价大（欧氏距离会被"墙另一边的近目标"误导）。

**组合**：

$$
h(n)=\max\big(h_{nonholo}(n),\ h_{holo}(n)\big)
$$

两者各自可采纳（各自放松了问题的一部分约束，代价不会高于真实代价），所以取最大值仍可采纳，且比任何一个都紧。

**手算（为什么要 holonomic-with-obstacles）**：目标在一堵长墙正后方，欧氏距离只有 3 m，但绕墙需要 12 m。欧氏距离启发把车引向墙，在墙前面探索大量无用节点；用 2D 绕障距离 $h_{holo}=12$，搜索直接绕向墙端。

### 6.3 Hybrid A* 为什么不保证最优

1. 离散化：每个格子只留一个节点，实际最优路径可能在同一格子里要求不同的朝向 / 代价路径；
2. 启发函数虽可采纳，但"每格一个"的剪枝破坏了 A* 的最优性证明（见 [图搜索章](./01_图搜索_Dijkstra_A星.md) 定理 1 的前提）；
3. 因此算法常配合**后处理平滑**来改善路径质量。

---

## 7. Dubins 与 Reeds-Shepp：解析扩展

### 7.1 它们是什么

在无障碍条件下，已知起点、终点位姿和最小转弯半径 $\rho$，求最短的车辆可行路径：

- **Dubins 曲线**（Dubins, 1957）：车**只能前进**。最短路径一定由"圆弧—直线—圆弧"（CSC）或"圆弧—圆弧—圆弧"（CCC）组成，共 6 种单词：LSL、RSR、LSR、RSL、RLR、LRL（L=左转，R=右转，S=直行）。
- **Reeds-Shepp 曲线**（Reeds & Shepp, 1990）：车**可前进也可倒车**，组合更多（共 48 种），能处理"目标在正后方"之类的情形。

（引用年份与名称凭记忆，细节未核对原文。）

### 7.2 手算一条 Dubins（LSR）

起点 $(0,0,0°)$，终点 $(4,2,0°)$（正前方 4 m，向左偏 2 m，朝向相同），$\rho=1$ m。

1. **圆心**：起点处的左圆圆心 $C_1=(0,\ 1)$；终点处的右圆圆心 $C_2=(4,\ 2-1)=(4,\ 1)$。
2. 圆心距 $d=|C_1C_2|=4$。LSR 的内公切线存在条件 $d\ge2\rho=2$ ✓。
3. 切线段长度 $\ell=\sqrt{d^2-(2\rho)^2}=\sqrt{16-4}=3.464$ m。
4. 切线与圆心连线夹角 $\alpha=\arcsin(2\rho/d)=\arcsin(0.5)=30°$。所以起点先**左转 $30°$**（航向从 0° 变到 30°），沿 30° 方向直行 $3.464$ m，再**右转 $30°$**回到航向 0°。
5. 弧长 $=\rho\alpha=1\times0.5236=0.5236$ m（每段）。

总长 $=0.5236+3.464+0.5236=4.511$ m。（直线距离是 $\sqrt{16+4}=4.472$ m，路径只比直线长 0.04 m。）

**验证终点**：左弧后位置 $C_1+\rho(\sin30°,\ -\cos30°)=(0.5,\ 0.134)$；沿 30° 直行 3.464：$(0.5+3.0,\ 0.134+1.732)=(3.5,\ 1.866)$；右转圆心在其右侧：$(3.5,1.866)+(\sin30°,-\cos30°)=(4.0,\ 1.0)=C_2$ ✓；转回航向 0° 后位置 $C_2+(0,1)=(4,\ 2)$ ✓。

### 7.3 为什么要 Reeds-Shepp：目标在正后方

终点 $(-1,0,0°)$（正后方 1 m），$\rho=1$：

- Dubins（只能前进）：必须绕一整圈，长度 $2\pi\rho+1=7.283$ m；
- Reeds-Shepp：直接倒车 1 m，长度 **1.0** m。

所以车要允许倒车时，用 Reeds-Shepp 当非完整启发和解析扩展；这也是 Hybrid A* 能做泊车类机动的原因。

### 7.4 解析扩展（analytic expansion / "shot to goal"）

只靠运动基元一步步搜索，到达目标位姿（含朝向）很难：格子很多，且恰好落在目标的概率低。所以每隔若干次展开（或当节点距离目标较近时），**尝试直接用 Reeds-Shepp 曲线连到目标**：

```text
if (展开次数 % N == 0) 或 (节点到目标距离 < 阈值):
    path = ReedsShepp(当前节点, 目标)
    if 沿 path 采样的每个位姿都无碰撞:
        返回 (到当前节点的搜索路径) + path
```

距离越近，尝试越频繁。成功时搜索提前结束，也使终点朝向精确满足。这是 Hybrid A* 实际可用的关键技巧之一。

---

## 8. 碰撞检测与车辆 footprint

### 8.1 为什么不能把车当质点

车有长度和宽度。同一个位置 $(x,y)$，朝向不同，占据的栅格不同。所以碰撞检测对 **$(x,y,\theta)$** 做，而不是只对 $(x,y)$。

### 8.2 常见方法

| 方法 | 做法 | 优缺点 |
|---|---|---|
| 多边形（矩形）覆盖栅格 | 旋转矩形，逐格检查是否占用 | 精确，较慢 |
| 多圆近似 | 用几个圆覆盖车体；只需查**中心点到最近障碍距离**是否大于半径 | 快（查距离场），略保守 |
| 预计算 | 对每个离散航向预计算 footprint 的格子偏移表 | 加速，占内存 |

### 8.3 手算：两圆近似

车体矩形：长 0.7 m（后轴后方 0.1 m，前方 0.6 m），宽 0.4 m（半宽 0.2 m）。用 2 个圆覆盖，圆心沿车身 x 轴放在距后轴 $0.1$ m 和 $0.4$ m 处。最远的角点到最近圆心的距离：

- 后角点 $(-0.1,\ \pm0.2)$ 到圆心 $(0.1,\ 0)$：$\sqrt{0.2^2+0.2^2}=0.283$；
- 前角点 $(0.6,\ \pm0.2)$ 到圆心 $(0.4,\ 0)$：$0.283$。

所以圆半径需 $r\ge0.283$ m，才能完全覆盖。取 $r=0.283$ m。

在位姿 $(2,\ 1,\ 90°)$：圆心 $=(2,1)+x_c(\cos90°,\sin90°)=(2,\ 1.1)$ 和 $(2,\ 1.4)$。查距离场：若 $(2,1.1)$ 到最近障碍 $0.30$ m，$(2,1.4)$ 到最近障碍 $0.25$ m，则第二个圆 $0.25<0.283$，**碰撞**。

保守性：圆比矩形多覆盖了一点，所以可能误判碰撞（宁可错杀）。想更精确可以用 3 个更小的圆。

### 8.4 与代价地图的关系

- 距离场来自对栅格地图的距离变换；
- 代价地图的"膨胀层"是把 footprint 的**内切圆半径**当作致命距离，外接圆半径当作可能碰撞距离；用多圆近似时，是对每个圆各查一次。

---

## 9. 路径平滑

### 9.1 为什么需要

Hybrid A* 路径由离散的圆弧段拼接，曲率在段与段连接处不连续（转向角突变），格子量化还会造成轻微的"锯齿"。直接给跟踪控制器会产生抖动。所以通常后处理平滑。

### 9.2 目标函数

对离散路径点 $\{p_i\}$ 最小化：

$$
J=w_o\sum_iF_{obs}(p_i)+w_\kappa\sum_i\max(0,\kappa_i-\kappa_{max})^2+w_s\sum_i\|p_{i+1}-2p_i+p_{i-1}\|^2
$$

三项：远离障碍；曲率不超过 $\kappa_{max}$；二阶差分小（光滑）。可用梯度下降或共轭梯度求解，起点和终点固定。

### 9.3 离散曲率与手算

三点 $p_{i-1}=(0,0)$，$p_i=(1,0)$，$p_{i+1}=(2,0.3)$：

- 第一段方向 $0°$，第二段向量 $(1,0.3)$，方向 $\arctan0.3=16.70°$，航向变化 $\Delta\phi=0.2915$ rad；
- 第二段长 $\sqrt{1+0.09}=1.044$ m；
- 离散曲率 $\kappa_i\approx\Delta\phi/|p_{i+1}-p_i|=0.2915/1.044=0.279$ 1/m，对应转弯半径 $3.58$ m $>R_{min}=0.866$ m，**可行**。

**光滑项的梯度下降一步**：光滑项对中间点 $p_i$ 的负梯度方向为 $p_{i-1}+p_{i+1}-2p_i=(0+2-2,\ 0+0.3-0)=(0,\ 0.3)$。取步长 $\alpha=0.5$：

$$
p_i\leftarrow(1,0)+0.5\cdot(0,0.3)=(1,\ 0.15)
$$

中点被拉向两邻点连线的中点，路径变直。要同时受障碍项和曲率项约束，所以每步需要三项梯度一起算。

### 9.4 平滑后别忘了重新检查

平滑可能把路径推向障碍；平滑后**必须再做一次碰撞检测**与曲率检查，不通过就回退或减小平滑步长。

---

## 10. 在本项目里怎么用

CV 的 P5 条目：*Hybrid-A\* global planning: incorporated vehicle heading and kinematic constraints into the search and generated feasible global paths under vehicle-footprint, minimum-turning-radius and obstacle-cost constraints.*

逐词对照：

| CV 词 | 本章对应 |
|---|---|
| vehicle heading | 状态含 $\theta$（第 3 节） |
| kinematic constraints | 自行车模型 / 运动基元（第 2、4 节） |
| vehicle-footprint | footprint 碰撞检测（第 8 节） |
| minimum-turning-radius | $R_{min}=L/\tan\delta_{max}$（第 2.4 节）；转向角范围、曲率检查、平滑约束 |
| obstacle-cost | 代价函数中的障碍代价项（第 6.1 节）与 holonomic 启发（第 6.2 节） |

在整条导航链里，Hybrid A* 产生**全局路径**；局部规划器 TEB 在其附近用实时障碍信息做时间参数化优化（[下一章](./03_代价地图_局部规划_TEB.md)）；MPC 负责跟踪（[轨迹跟踪章](./04_轨迹跟踪_PurePursuit_Stanley_MPC.md)）。项目总览见 [P5 README](../../01_项目课程/P5_车式移动机器人自主导航/README.md)。

> **本项目的典型取值（设计量级，调参时按实机微调；不是实测结果）**
>
> | 项 | 本项目通常取 |
> |---|---|
> | 轴距与转向 | 轴距 $L$ 取车体量级的几十厘米，最大转向角 $\delta_{max}$ 取 25°–30°；由 $R_{min}=L/\tan\delta_{max}$ 得到最小转弯半径（教学例：$L=0.5$ m、$\delta_{max}=30^\circ$ 时 $R_{min}\approx0.87$ m），规划时再乘 1.1–1.2 的余量，避免控制器跟不上 |
> | 离散化 | 位置分辨率 0.1–0.2 m，航向 5° 一格（72 个朝向 bin） |
> | 运动基元 | 前进三个（左极限、直行、右极限转向），允许倒车时再各加一个，共 6 个；弧长略大于一个格子对角线（约 0.25–0.3 m），保证每次扩展都走出父节点的桶 |
> | 启发函数 | 取 $\max(h_1,h_2)$：忽略障碍的 Reeds-Shepp/Dubins 长度，与带障碍的 2D 栅格最短路（holonomic-with-obstacles） |
> | 代价项 | 倒车、转向变化、换向（前进/倒车切换）、障碍代价四类惩罚，权重按"先保证可行、再调风格"的顺序整定 |
> | 后处理 | 搜索后做曲率检查与路径平滑（离障碍远、曲率不超限、二阶差分小），平滑后再用三点曲率复核 $\kappa\le1/R_{min}$ |
> | 规划耗时 | 设计目标是在几十到几百毫秒内给出一条全局路径，只在新目标或路径失效时触发，不要求每个控制周期都重算 |
> | 实现方式 | 我们的做法是把 Hybrid A* 作为全局规划器插件接入导航框架，自己配置并调参：footprint、$R_{min}$、分辨率与各惩罚权重 |

---

## 11. 常见错误与故障现象

| 现象 | 原因 |
|---|---|
| 规划不出路径（找不到解） | 弧长太小或离散化太粗；转弯半径过大；目标朝向不可达；起点 / 终点在障碍内；`max_iterations` 不够 |
| 路径来回换挡 | 换挡罚太小；倒车权重太小 |
| 路径贴着障碍 | 障碍代价权重小；footprint 偏小；膨胀不足 |
| 路径过长、绕路 | 障碍惩罚太大；启发太弱 |
| 规划很慢 | 分辨率太细；只用欧氏启发；没有 holonomic 距离场；没有解析扩展 |
| 终点朝向不对 | 没做解析扩展；容忍度太大 |
| 车跟踪路径时抖动 | 路径曲率不连续；没平滑；采样点距离不均 |
| 车实际转不过路径的弯 | 规划的转向角范围大于实际；运动学参数（轴距）错 |
| 起点位姿附近无解 | 车朝向与路径起点方向相反且空间太窄，需要倒车基元 |

---

## 12. 面试怎么问、怎么答

**Q1. Why Hybrid A* instead of grid A* for a car-like robot?**

> A: A car has a non-holonomic constraint and a minimum turning radius, so a grid path with sharp corners or sideways moves cannot be executed. Hybrid A* searches over continuous poses, x, y and heading, and expands with short arcs from the bicycle model, so every path segment is kinematically feasible. It keeps the A* structure, but each discretised cell holds only one node, so the search stays tractable.

中文要点：车有非完整约束和最小转弯半径；Hybrid A* 在连续位姿上用自行车模型圆弧扩展，每个离散格子一个节点。

**Q2. What heuristics do you use, and why two?**

> A: One heuristic is non-holonomic without obstacles, the Reeds-Shepp path length. It captures the heading and turning-radius cost. The other is holonomic with obstacles, a 2D Dijkstra distance map from the goal. It captures dead ends and detours around walls. Each is admissible by itself, so I take the maximum, which is tighter than either.

追问：为什么取最大值仍可采纳？（答：两者各自不高估，取最大值也不高估。）Hybrid A* 能保证最优吗？（答：不能，每格一个节点和离散化会破坏最优性，也不完备。）

**Q3. How do you handle the vehicle footprint and the minimum turning radius?**

> A: The footprint is checked at every expanded pose with its heading, for example by covering the body with a few circles and checking the distance field at each circle centre. The minimum turning radius is built into the primitives: the steering angle is limited by the maximum steering angle, so R-min equals the wheelbase divided by tan of the maximum steering angle. After smoothing I check the curvature again.

**Q4. What is the analytic expansion?**

> A: Every few expansions, or when the node is close to the goal, I try to connect directly to the goal with a Reeds-Shepp curve. If the whole curve is collision-free, the search ends early. It makes the final heading exact and saves a lot of expansions.

**Q5. What did you tune for Hybrid A* in P5?**（用自己的真实经历回答）

> A: I mainly tuned the cost weights — the reverse and direction-change penalties and the obstacle cost — plus the step length and angular resolution, and I checked the footprint against the real vehicle. In practice I changed one group at a time: first feasibility (turning radius and footprint), then resolution, then the penalties.

## 13. 本章要点速记

1. 非完整约束：$\dot x\sin\theta-\dot y\cos\theta=0$；车不能横移。
2. 自行车模型：$\dot\theta=v\tan\delta/L$；$R_{min}=L/\tan\delta_{max}$。
3. Hybrid A*：节点存连续位姿，离散索引去重，每格留最优一个。
4. 圆弧基元：$x_b=\sin(\kappa s)/\kappa$，$y_b=(1-\cos\kappa s)/\kappa$，$\Delta\theta=\kappa s$。
5. 代价：弧长 + 倒车 + 换挡 + 转向 + 转向变化 + 障碍。
6. 启发：non-holonomic-without-obstacles（RS 长度）与 holonomic-with-obstacles（2D Dijkstra），取最大。
7. 不保证最优、不完备；需要平滑。
8. Dubins 只能前进，Reeds-Shepp 可倒车；解析扩展让搜索提前收尾。
9. 碰撞检测对 $(x,y,\theta)$ 做：矩形或多圆近似，圆半径 ≥ 圆心到角点的最大距离。
10. 平滑后必须重新碰撞 / 曲率检查。

## 自测题

1. $L=0.4$ m，$\delta_{max}=25°$，求 $R_{min}$。（答：$0.4/\tan25°=0.4/0.4663=0.858$ m。）
2. 圆弧基元 $\kappa=1$ 1/m，$s=0.5$ m，初始 $(0,0,0)$，求终点。（答：$\Delta\theta=0.5$ rad；$x=\sin0.5=0.4794$，$y=1-\cos0.5=0.1224$。）
3. 为什么只用欧氏距离做启发在迷宫 / 墙后目标时效率低？（要点：忽略障碍，把搜索引向墙，展开大量无用节点；应加 holonomic-with-obstacles 的绕障距离。）
4. 为什么 $\max(h_1,h_2)$ 仍可采纳？（要点：$h_1,h_2\le h^*$，则 $\max\le h^*$。）
5. 目标在车正后方 2 m，Dubins 和 Reeds-Shepp 谁短？（答：RS 直接倒车 2 m；Dubins 要绕圈，长得多。）
6. 栅格 0.25 m，位姿 $x=1.30$，$y=0.62$，$\theta=47°$，航向格 10°，格子索引？（答：$\lfloor1.30/0.25\rfloor=5$，$\lfloor0.62/0.25\rfloor=2$，$\lfloor47/10\rfloor=4$，即 $(5,2,4)$。）
7. 路径曲率检查：三点 $(0,0),(1,0),(2,1)$，转弯半径可行吗（$R_{min}=0.866$）？（答：第二段向量 $(1,1)$ 方向 45°，$\Delta\phi=0.785$ rad，长 1.414，$\kappa=0.555$，$R=1.80$ m $>0.866$，可行。）

## 相关链接

- 上一章：[图搜索：Dijkstra 与 A*](./01_图搜索_Dijkstra_A星.md)
- 下一章：[代价地图、局部规划与 TEB](./03_代价地图_局部规划_TEB.md)
- 相关：[轨迹跟踪：Pure Pursuit、Stanley、MPC](./04_轨迹跟踪_PurePursuit_Stanley_MPC.md)、[导航系统架构](./05_导航系统架构_Nav2与move_base对比.md)
- 基础：[差速、麦克纳姆、全向轮与车辆运动学](../02_力学与动力学建模/04_差速_麦克纳姆_全向轮与车辆运动学.md)
- 项目：[P5 车式移动机器人自主导航](../../01_项目课程/P5_车式移动机器人自主导航/README.md)
- 八股：[路径规划与导航八股](../../03_八股知识/03_机器人学/04_路径规划与导航八股.md)
