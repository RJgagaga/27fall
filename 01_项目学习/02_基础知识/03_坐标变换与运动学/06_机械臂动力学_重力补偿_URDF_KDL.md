# 机械臂动力学、重力补偿、URDF 与 KDL

> **一句话定位**：运动学只回答"在哪里、多快"，动力学回答"要多大力矩"。本章从 URDF 文件怎么描述一台机器人讲起，到 KDL / Pinocchio 怎样据此算出重力项和质心，再到**重力补偿**（让主臂"轻得像没有重量"）和**质心偏移前馈**（让倾斜平衡的机器人在手臂运动时不被重力矩拖倒），最后总结符号、零位、关节顺序出错时的典型故障。

## 本章学完你能……

- 读懂一个 URDF 的 `link / joint / inertial / axis / limit`，说出每个字段的坐标系与单位，并从中手算正运动学。
- 说清 KDL 与 Pinocchio 的"链 / 树"、`nq` 与 `nv`、RNEA、重力项各是什么。
- 写出动力学方程 $M(q)\ddot q+C(q,\dot q)\dot q+g(q)=\tau$，解释每项的物理含义。
- **从势能推导**重力项 $g(q)=\partial U/\partial q$，手算二连杆的重力补偿力矩。
- 解释"质心偏移 $\to$ 重力矩 $\to$ 前馈"：手算一个 ballbot / 双臂的例子，判断 roll 与 pitch 分别对应质心的哪个分量。
- 说明主从遥操作里主臂只做重力补偿、从臂做位置速度跟踪的原因。
- 针对"符号反、零位错、关节顺序错位、质量缺失、单位错"等故障给出系统的排查流程。

## 前置知识

- [坐标系、旋转矩阵与齐次变换](./01_坐标系_旋转矩阵_齐次变换.md)、[正逆运动学与雅可比](./03_正逆运动学与雅可比.md)。
- 质心、力矩、转动惯量的初步概念：[牛顿欧拉与刚体力学](../02_力学与建模/01_牛顿欧拉与刚体力学.md)。
- 拉格朗日方程的思路（可只看结论）：[拉格朗日方程与机器人动力学](../02_力学与建模/02_拉格朗日方程与机器人动力学.md)。

## 目录

1. 要解决什么问题
2. URDF 的结构
3. 从 URDF 到正运动学（含零位与轴方向的陷阱）
4. KDL 与 Pinocchio：树、链、RNEA、重力项
5. 动力学方程
6. 重力补偿的原理与推导
7. 二连杆手算：重力、惯性、速度耦合
8. 质心偏移与重力力矩（P1 的质心前馈）
9. 主从示教里的重力补偿（P2）
10. 符号、零位、关节顺序错位等故障
11. 面试怎么问、怎么答
12. 本章要点速记、自测题、相关链接

---

## 1. 要解决什么问题

先看三个现场场景：

1. **主臂示教**：操作者握着主臂手柄拖动，希望手感像在水里拖一个"没有重量"的物体。可主臂的连杆有质量，关节电机若不出力，手臂会自己垂下去。要让它"悬停"在任意位置，电机必须输出刚好抵消重力的力矩——**重力补偿**。
2. **平衡机器人上的双臂**：整机在两轮 / 三轮上做倒立摆平衡，手臂一抬，整机的质心就偏离支点，产生一个持续的重力矩。若控制器不知道，只能靠反馈慢慢纠正，姿态会漂移甚至失稳——需要把"质心偏移"估出来做**前馈**。
3. **算这些力矩需要什么**：每一节连杆的质量、质心位置、转动惯量、关节轴方向、当前关节角——这些信息写在机器人描述文件 **URDF** 里，由 **KDL / Pinocchio** 这类库读入并计算。

> **注意**：只补重力，**不等于**完整逆动力学控制。完整逆动力学还包括惯性项和速度耦合项。凡是只补重力的系统，称"重力补偿"，不要说成"动力学补偿"。

---

## 2. URDF 的结构

### 2.1 URDF 是什么

URDF（Unified Robot Description Format）是用 XML 描述机器人**运动学树**和**惯性参数**的文件。它由两类元素组成：`link`（刚体）和 `joint`（连接两个刚体的关节）。

### 2.2 一个二连杆示例（教学示例，非项目文件）

```xml
<robot name="two_link_arm">

  <link name="base_link"/>

  <link name="link1">
    <inertial>
      <origin xyz="0.2 0 0" rpy="0 0 0"/>     <!-- 质心，在 link1 坐标系下 -->
      <mass value="2.0"/>                      <!-- kg -->
      <inertia ixx="0.0001" ixy="0" ixz="0"
               iyy="0.02667" iyz="0" izz="0.02667"/>   <!-- kg*m^2，绕质心 -->
    </inertial>
    <visual> ... </visual>
    <collision> ... </collision>
  </link>

  <link name="link2">
    <inertial>
      <origin xyz="0.15 0 0" rpy="0 0 0"/>
      <mass value="1.0"/>
      <inertia ixx="0.0001" ixy="0" ixz="0"
               iyy="0.0075" iyz="0" izz="0.0075"/>
    </inertial>
  </link>

  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child  link="link1"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>          <!-- joint1 相对 base_link 的位姿 -->
    <axis xyz="0 -1 0"/>                       <!-- 转轴，在 joint 坐标系下 -->
    <limit lower="-1.57" upper="1.57" effort="20" velocity="3.0"/>
  </joint>

  <joint name="joint2" type="revolute">
    <parent link="link1"/>
    <child  link="link2"/>
    <origin xyz="0.4 0 0" rpy="0 0 0"/>        <!-- 沿 link1 的 x 方向 0.4 m -->
    <axis xyz="0 -1 0"/>
    <limit lower="-2.0" upper="2.0" effort="10" velocity="3.0"/>
  </joint>

</robot>
```

### 2.3 字段含义与坐标系

| 元素 | 含义 | 坐标系 / 单位 |
|---|---|---|
| `link` | 一个刚体；它的坐标系叫 link 坐标系 | — |
| `joint.parent / child` | 关节连接的父、子 link | 构成**树**（每个 link 只有一个父） |
| `joint.origin xyz rpy` | **关节坐标系相对父 link 坐标系**的位姿（$q=0$ 时） | xyz：m；rpy：rad，**固定轴 roll-pitch-yaw**，$R=R_z(y)R_y(p)R_x(r)$ |
| `joint.axis` | 转轴（转动关节）或移动方向（移动关节） | 在**关节坐标系（= 子 link 坐标系）**下的单位向量 |
| `joint.type` | `revolute`（有限位转动）、`continuous`（无限位转动）、`prismatic`（移动）、`fixed`（固定）、`floating`（6 自由度浮动）、`planar` | — |
| `joint.limit` | 位置上下限（lower/upper）、最大力矩/力（effort）、最大速度（velocity） | rad 或 m；N·m 或 N；rad/s 或 m/s |
| `joint.dynamics` | 阻尼、摩擦（damping, friction） | 可选 |
| `inertial.origin` | **质心**位置与惯性主轴方向 | 在 link 坐标系下 |
| `inertial.mass` | 质量 | kg |
| `inertial.inertia` | 惯性张量 6 个独立分量（对称矩阵） | kg·m²，**绕质心**，在 `inertial` 坐标系的轴上表达 |

### 2.4 惯性张量的基本公式

均匀细杆（长 $l$、质量 $m$）绕垂直于杆的、过质心的轴：$I=\dfrac{ml^2}{12}$。绕杆端点的轴：$I=\dfrac{ml^2}{3}$（平行轴定理：$I_{end}=I_c+m(l/2)^2$）。

示例中 link1：$m=2,\ l=0.4$，$I=2\times0.16/12=0.02667$；link2：$m=1,\ l=0.3$，$I=0.09/12=0.0075$。这就是上面 `iyy`、`izz` 的来源（沿杆方向的 `ixx` 很小，取近似值）。

### 2.5 URDF 里最容易出问题的地方

| 问题 | 后果 |
|---|---|
| `mass` 缺失或为 0（CAD 导出默认值） | 重力项 / 质心算出来是 0，补偿"没效果" |
| `inertial.origin` 写成 link 原点而非真实质心 | 重力力矩偏小或偏大 |
| 从 CAD 导出的单位是 mm 或 g | 惯量差 $10^6$ 倍，质量差 $10^3$ 倍 |
| `axis` 方向与实际电机正方向相反 | 补偿力矩符号反，机器人"被推下去" |
| 左右臂镜像时 `axis` 没取反 | 一条臂补偿正确，另一条加倍下垂 |
| `limit` 的 effort 填了很小/很大的值 | 仿真被截断，或没有保护 |
| 关节在 URDF 里的顺序与电机 ID 顺序不同 | 见第 10 节"关节顺序错位" |

---

## 3. 从 URDF 到正运动学（含零位与轴方向的陷阱）

### 3.1 公式

一个关节的变换 = 固定安装变换 × 绕轴转动：

$$
{}^{parent}T_{child}(q)=T_{origin}\cdot\mathrm{Rot}(\hat a,\ q)
$$

整条链：$^{base}T_{tip}=\prod_i T_{origin,i}\,\mathrm{Rot}(\hat a_i,q_i)$。这与上一章"几何法"一致。

### 3.2 手算：示例 URDF 的末端位置

两个关节轴都是 $(0,-1,0)$。绕 $-y$ 轴转 $\theta$：$\mathrm{Rot}(-\hat y,\theta)=R_y(-\theta)=\begin{bmatrix}\cos\theta&0&-\sin\theta\\0&1&0\\\sin\theta&0&\cos\theta\end{bmatrix}$，把 $x$ 轴 $(1,0,0)$ 变成 $(\cos\theta,\ 0,\ \sin\theta)$——**向上抬**。所以 $q$ 为正表示手臂向上抬，与上一章的平面模型（角度从水平线向上量）一致。

取 $q_1=30^\circ$，$q_2=30^\circ$（`link2` 沿自身 $x$ 轴长度 $0.3$）：

- joint2 在 base 下的位置：$0.4\,(\cos30^\circ,\ 0,\ \sin30^\circ)=(0.3464,\ 0,\ 0.2)$；
- link2 的绝对方向 $q_1+q_2=60^\circ$，末端（link2 的 $x$ 方向 $0.3$ m）：$(0.3464+0.3\cos60^\circ,\ 0,\ 0.2+0.3\sin60^\circ)=(0.4964,\ 0,\ 0.4598)$。

### 3.3 关键陷阱：同样 `axis="0 1 0"` 会让手臂**向下**

若 `axis` 写成 $(0,1,0)$（$+y$ 轴），$\mathrm{Rot}(\hat y,\theta)$ 把 $x$ 轴变成 $(\cos\theta,0,-\sin\theta)$：正的 $q$ 让手臂**向下垂**。此时同样取 $q_1=q_2=30^\circ$，joint2 在 $(0.3464,\ 0,\ -0.2)$，末端在 $(0.4964,\ 0,\ -0.4598)$——高度正负恰好相反。如果你的重力补偿公式是按"向上为正"推出来的，而 URDF 里是 $+y$ 轴，公式的符号要整体翻转。**不要凭记忆写重力项的符号，要看 URDF 的 `axis`**。

### 3.4 零位

`q=0` 时手臂指向哪里，由所有 `origin` 的 `rpy` 和 link 的几何决定。CAD 的零位、电机编码器的零位、URDF 的零位**是三个不同的东西**。真机上必须有一步**零位标定**：让手臂摆到已知姿态（例如水平伸直），读取编码器，得到偏置 $q_{offset}$，之后 $q_{urdf}=\pm(q_{encoder}-q_{offset})$。

---

## 4. KDL 与 Pinocchio：树、链、RNEA、重力项

### 4.1 为什么需要库

自己手推二连杆还可以，七自由度双臂就不现实。库读入 URDF，建立内部的"运动学树 + 惯性参数"，然后提供：

- 正运动学、雅可比；
- 重力项 $g(q)$、科氏项、质量矩阵 $M(q)$；
- 逆动力学（给定 $q,\dot q,\ddot q$ 求 $\tau$）、质心。

### 4.2 KDL（Kinematics and Dynamics Library）

| KDL 概念 | 含义 |
|---|---|
| `Joint` / `Segment` | 关节 / 连杆段（一个 `Segment` 含一个关节与其后的刚体） |
| `Chain` | **串联链**：从 base 到 tip 的一条线 |
| `Tree` | 完整的树（含分支，如双臂 + 躯干） |
| `RigidBodyInertia` | 质量、质心、惯量（由 `kdl_parser` 从 URDF 的 `inertial` 转换） |
| `ChainDynParam` | 给定链和重力向量，计算 $M$、$C$、$g$：`JntToMass`、`JntToCoriolis`、`JntToGravity` |
| `ChainIdSolver_RNE` | 递归牛顿–欧拉逆动力学 |
| `kdl_parser` | URDF $\to$ `KDL::Tree`，再 `tree.getChain(base, tip)` 取出一条链 |

典型流程（伪代码）：

```text
urdf  ──kdl_parser──▶  KDL::Tree  ──getChain("base_link","tool")──▶  KDL::Chain
                                                                      │
                                         ChainDynParam(chain, gravity=(0,0,-9.81))
                                                                      │
                          JntToGravity(q_measured)  ──▶  g(q)  [N*m]  ──▶  发给电机
```

**要点**：重力向量是一个**参数**：`(0,0,-9.81)` 是在 base 坐标系下表达的重力方向。如果 base 随机身倾斜，应当把重力旋转到 base 系：$g_{base}=R^{\mathsf T}g_{world}$。

**关节顺序**：KDL 的关节顺序 = 链上有自由度的 segment 的顺序，即 base 到 tip 沿链的顺序。`JntToGravity` 返回的向量第 $i$ 个分量对应链上第 $i$ 个运动关节。

### 4.3 Pinocchio

Pinocchio 是基于空间代数的高效刚体动力学库，常见函数：

| 函数 | 作用 |
|---|---|
| `pin.buildModelFromUrdf(path)` | 读 URDF 建 `Model`；`data = model.createData()` |
| `pin.forwardKinematics(model, data, q)` | 正运动学 |
| `pin.computeJointJacobians` / `pin.getFrameJacobian` | 雅可比 |
| `pin.rnea(model, data, q, v, a)` | **逆动力学**，得到 $\tau$ |
| `pin.computeGeneralizedGravity(model, data, q)` | 重力项 $g(q)$ |
| `pin.crba(model, data, q)` | 质量矩阵 $M(q)$ |
| `pin.nonLinearEffects(model, data, q, v)` | $C(q,\dot q)\dot q+g(q)$ |
| `pin.centerOfMass(model, data, q)` | 整机质心（在 base 或 world 系） |

两个要特别记住的概念：

1. **`nq` 与 `nv`**：浮动基座（`floating`/自由关节）的位姿在 `q` 里是 7 维（位置 3 + 四元数 4，Pinocchio 里四元数顺序 $(x,y,z,w)$），而速度 `v` 是 6 维。纯转动关节臂 `nq == nv`。见[欧拉角、四元数与姿态表示](./02_欧拉角_四元数_姿态表示.md)。
2. **关节顺序**：`model.names` 的顺序是从 URDF 树遍历得到的，**与电机 CAN ID 顺序没有任何必然关系**；用字典（名字 $\to$ 索引）去对应，不要假设。

### 4.4 KDL 与 Pinocchio 对比

| | KDL | Pinocchio |
|---|---|---|
| 结构 | 以 `Chain` 为主，`Tree` 辅助 | 原生支持树、浮动基座 |
| 浮动基座 | 不方便 | 原生支持（质心、质心动量都方便） |
| 速度 | 较慢 | 很快（解析导数、代码生成友好） |
| 典型生态 | ROS / MoveIt 传统 | 机器人学研究、足式 / 平衡控制 |
| 本项目 | P2 用 KDL（主从臂的链式重力补偿） | P1 用 Pinocchio（整机质心） |

### 4.5 RNEA：递归牛顿–欧拉

逆动力学 $\tau=\mathrm{RNEA}(q,\dot q,\ddot q)$ 分两遍：

- **前向递推（base $\to$ tip）**：逐连杆算角速度、角加速度、质心加速度；
- **后向递推（tip $\to$ base）**：逐连杆算惯性力 / 力矩，加上子连杆传来的力，投影到关节轴得到 $\tau_i$。

复杂度 $O(n)$。

**小技巧（得到重力项）**：令 $\dot q=0,\ \ddot q=0$，则 $\tau=g(q)$——RNEA 在零速度零加速度下恰好只剩重力项。库里的 `computeGeneralizedGravity` 本质上就是这个；也可以等价地把重力实现为"基座有一个向上的 $+g$ 加速度"。

**一个自由度的手算**：均匀细杆（$m=2,\ l=0.4$）绕端点转，角 $q$ 从水平线向上量，$q=30^\circ$，$\dot q=\ddot q=0$：
- 质心位置 $r_c=0.2\,(\cos30^\circ,\ \sin30^\circ)$；
- 重力 $-mg$ 沿 $-z$，对转轴的力矩大小 $=mg\cdot r_{c,x}=2\times9.81\times0.2\times0.866=3.398\ \mathrm{N\cdot m}$，方向使杆下垂；
- 所以 RNEA 输出的 $\tau=g(q)=+3.398$（电机要提供这么大的**向上抬**的力矩来平衡）。

---

## 5. 动力学方程

### 5.1 一般形式

$$
\boxed{M(q)\,\ddot q+C(q,\dot q)\,\dot q+g(q)+\tau_f=\tau+J^{\mathsf T}F_{ext}}
$$

| 项 | 含义 | 单位 |
|---|---|---|
| $M(q)\in\mathbb R^{n\times n}$ | 质量 / 惯性矩阵，对称正定，与位形相关 | kg·m² |
| $C(q,\dot q)\dot q$ | 科氏力与离心力（速度耦合项），速度的二次型 | N·m |
| $g(q)$ | 重力项，只与位形有关 | N·m |
| $\tau_f$ | 摩擦（粘性 $b\dot q$、库仑 $\tau_c\,\mathrm{sign}(\dot q)$） | N·m |
| $\tau$ | 电机施加的关节力矩 | N·m |
| $J^{\mathsf T}F_{ext}$ | 外力折算到关节 | N·m |

**每一项都是 $n$ 维的"关节广义力"**。

### 5.2 怎么得到

用拉格朗日方程 $L=T-U$，$\dfrac{d}{dt}\dfrac{\partial L}{\partial\dot q}-\dfrac{\partial L}{\partial q}=\tau$。动能 $T=\tfrac12\dot q^{\mathsf T}M(q)\dot q$ 给出惯性项和科氏项，势能 $U(q)$ 给出重力项。详见 [拉格朗日方程与机器人动力学](../02_力学与建模/02_拉格朗日方程与机器人动力学.md)。库里用 RNEA 直接递推，不必手推。

### 5.3 三种常见控制律

| 控制律 | 内容 | 用途 |
|---|---|---|
| 重力补偿 | $\tau=g(q)$ | 主臂示教；PD 控制的静差消除 |
| PD + 重力补偿 | $\tau=K_p(q_d-q)+K_d(\dot q_d-\dot q)+g(q)$ | 位置控制，消除重力引起的稳态误差 |
| 计算力矩（完整逆动力学） | $\tau=M(\ddot q_d+K_pe+K_d\dot e)+C\dot q+g$ | 高性能跟踪，对模型精度要求高 |

---

## 6. 重力补偿的原理与推导

### 6.1 重力项就是势能的梯度

重力是保守力：势能 $U(q)=\sum_km_kg\,z_{c,k}(q)$（$z_{c,k}$ 是第 $k$ 个连杆质心的高度，$z$ 向上）。广义重力力矩

$$
\boxed{g(q)=\frac{\partial U}{\partial q}}
$$

物理含义：如果关节 $i$ 往"势能增大"的方向动一点，重力会反抗；要**静止不动**，电机必须提供一个与重力矩大小相等、方向相反的力矩，也就是 $+g_i(q)$（在方程 $g(q)=\tau$ 的符号约定下）。

### 6.2 通用公式（逐关节）

关节 $i$ 的转轴单位向量 $\hat a_i$，轴上一点 $p_i$，连杆 $k\ge i$ 的质心 $p_{c,k}$ 随 $q_i$ 的变化率是 $\partial p_{c,k}/\partial q_i=\hat a_i\times(p_{c,k}-p_i)$。于是

$$
g_i(q)=\sum_{k\ge i}m_k\,g\,\hat z\cdot\big[\hat a_i\times(p_{c,k}-p_i)\big]
$$

只有"在关节 $i$ 之后（更靠末端）的连杆"才对 $g_i$ 有贡献。对绕水平轴转的关节，这等于 $\sum m_kg\cdot(\text{质心到转轴的水平距离})$——**重力矩 = 重量 × 水平力臂**。

> 如果 `axis` 指向相反方向，$\hat a_i\to-\hat a_i$，整个 $g_i$ 取反——这就是 3.3 节"符号陷阱"的数学原因。

### 6.3 二连杆结果

按第 3 节的约定（角度向上为正），质量 $m_1,m_2$，质心距关节 $l_{c1},l_{c2}$，$l_1$ 为第一杆长：

$$
g_1=(m_1l_{c1}+m_2l_1)\,g\cos q_1+m_2l_{c2}\,g\cos(q_1+q_2),\qquad
g_2=m_2l_{c2}\,g\cos(q_1+q_2)
$$

形状很直观：$\cos q_1$ 是第一杆质心的水平投影；$\cos(q_1+q_2)$ 是第二杆质心相对肘的水平投影。

---

## 7. 二连杆手算：重力、惯性、速度耦合

数值（教学示例）：$m_1=2\ \mathrm{kg}$，$m_2=1\ \mathrm{kg}$，$l_1=0.4\ \mathrm m$，$l_{c1}=0.2\ \mathrm m$，$l_{c2}=0.15\ \mathrm m$（$l_2=0.3$ m），$I_1=0.02667$，$I_2=0.0075\ \mathrm{kg\cdot m^2}$（绕各自质心），$g=9.81$。

### 7.1 重力项

**位形 A：$q=(30^\circ,30^\circ)$**

- $g_1=(2\times0.2+1\times0.4)\times9.81\times\cos30^\circ+1\times0.15\times9.81\times\cos60^\circ$
  $=0.8\times9.81\times0.8660+0.15\times9.81\times0.5=6.797+0.7358=7.532\ \mathrm{N\cdot m}$
- $g_2=0.15\times9.81\times\cos60^\circ=0.7358\ \mathrm{N\cdot m}$

**用杠杆臂核对**：连杆 1 质心水平距离 $0.2\cos30^\circ=0.1732$，$m_1g\times0.1732=3.398$；连杆 2 质心距关节 1 的水平距离 $0.4\cos30^\circ+0.15\cos60^\circ=0.3464+0.075=0.4214$，$m_2g\times0.4214=4.134$；总和 $7.532$ ✔。

**位形 B：手臂水平伸直 $q=(0,0)$**：$g_1=(0.8)(9.81)+0.15\times9.81=7.848+1.4715=9.320$；$g_2=1.4715$。这是重力矩的**最大值**（水平时力臂最长）。

**位形 C：手臂竖直向上 $q=(90^\circ,0)$**：$\cos90^\circ=0$，$g=(0,0)$——质心正好在转轴上方，不需要任何力矩，只是不稳定平衡。

### 7.2 完整逆动力学

位形 A，$\dot q=(1,2)$ rad/s，$\ddot q=(1,0.5)$ rad/s²。

**惯性矩阵**：
$M_{11}=I_1+I_2+m_1l_{c1}^2+m_2\big(l_1^2+l_{c2}^2+2l_1l_{c2}\cos q_2\big)$
$=0.02667+0.0075+0.08+(0.16+0.0225+2\times0.4\times0.15\times0.866)=0.02667+0.0075+0.08+0.28642=0.4006$

$M_{12}=I_2+m_2(l_{c2}^2+l_1l_{c2}\cos q_2)=0.0075+(0.0225+0.0520)=0.0820$

$M_{22}=I_2+m_2l_{c2}^2=0.0075+0.0225=0.0300$

**速度耦合项**：令 $h=-m_2l_1l_{c2}\sin q_2=-1\times0.4\times0.15\times0.5=-0.03$，

$$
C\dot q=\begin{bmatrix}h(2\dot q_1\dot q_2+\dot q_2^2)\\-h\dot q_1^2\end{bmatrix}
=\begin{bmatrix}-0.03\,(4+4)\\0.03\times1\end{bmatrix}=\begin{bmatrix}-0.24\\0.03\end{bmatrix}
$$

**合成**：

$$
\tau=M\ddot q+C\dot q+g=
\begin{bmatrix}0.4006+0.0410\\0.0820+0.0150\end{bmatrix}+\begin{bmatrix}-0.24\\0.03\end{bmatrix}+\begin{bmatrix}7.532\\0.7358\end{bmatrix}
=\begin{bmatrix}7.734\\0.863\end{bmatrix}\ \mathrm{N\cdot m}
$$

（第一项 $M\ddot q=(0.4006\times1+0.082\times0.5,\ 0.082\times1+0.03\times0.5)=(0.4416,\ 0.0970)$。）

**一个重要的数量级观察**：重力项 $7.53$ 比惯性项 $0.44$ 与速度耦合项 $0.24$ 大一个数量级以上。在慢速（人手拖动）的场景里，**重力项占绝对主导**，所以只补重力就能获得"几乎没有重量"的手感；而在快速运动时，惯性与科氏项不能忽略。

**一个反直觉的点**：即使 $\ddot q=0$（匀速），$C\dot q$ 也可能不为零——杆间动量方向随位形变化，关节仍需出力。

### 7.3 用 Python 复核（教学示例）

```python
import numpy as np
m1, m2, l1, lc1, lc2, g = 2.0, 1.0, 0.4, 0.2, 0.15, 9.81
def gravity(q1, q2):
    return np.array([(m1*lc1 + m2*l1)*g*np.cos(q1) + m2*lc2*g*np.cos(q1+q2),
                     m2*lc2*g*np.cos(q1+q2)])
print(gravity(np.deg2rad(30), np.deg2rad(30)))   # [7.532 0.7358]
```

---

## 8. 质心偏移与重力力矩（P1 的质心前馈）

### 8.1 要解决什么问题

倒立摆式平衡机器人（ballbot、两轮、轮腿）靠"质心保持在支点正上方"站立。机身上挂着可动的手臂时，**手臂的姿态改变整机质心的位置**。质心一旦偏离支点正上方，重力就产生一个持续的**倾覆力矩**，平衡控制器要用持续的轮力矩去抵消它。如果能根据手臂关节角**预先算出**这个偏移，就可以把相应的补偿作为**前馈**加入，减轻反馈的负担。

### 8.2 整机质心公式

各刚体质量 $m_k$、质心位置 $p_{c,k}$（在机身系下，由 URDF 与当前关节角做正运动学得到）：

$$
c=\frac{\sum_km_k\,p_{c,k}}{\sum_km_k}=\frac{\sum_km_kp_{c,k}}{M}
$$

Pinocchio 的 `centerOfMass(model, data, q)` 就是计算它。

### 8.3 重力矩

设支点为机身坐标系原点（或某固定点），质心在机身系下是 $c=(c_x,c_y,c_z)$，总质量 $M$。重力在机身系下是 $g_b=R^{\mathsf T}g_w$（$R$ 是机身姿态，$g_w=(0,0,-g)$）。重力对支点的力矩：

$$
\tau_g=c\times(M g_b)
$$

机身**直立**（$R=I$）时：

$$
\tau_g=M\begin{bmatrix}c_x\\c_y\\c_z\end{bmatrix}\times\begin{bmatrix}0\\0\\-g\end{bmatrix}=M g\begin{bmatrix}-c_y\\c_x\\0\end{bmatrix}
$$

读法：

- **俯仰（pitch，绕 $y$ 轴）力矩 $=Mg\,c_x$**——质心的**前后**偏移；
- **横滚（roll，绕 $x$ 轴）力矩 $=-Mg\,c_y$**——质心的**左右**偏移。

（机体系 $x$ 前、$y$ 左、$z$ 上。）这就是 CV 里"roll/pitch gravity feedforward"的由来：两个倾斜方向各对应质心的一个水平分量。

### 8.4 手算（教学示例，非项目实测）

整机总质量 $M=20\ \mathrm{kg}$（含两条手臂，各 $2.5\ \mathrm{kg}$）。手臂向前伸直时，每条手臂的质心在机身系 $x=+0.25\ \mathrm m$；其余部分（躯干 + 底盘）的质心在 $x=0$，高度 $z=0.5\ \mathrm m$。

- 质心前后偏移：$c_x=\dfrac{2\times2.5\times0.25+15\times0}{20}=\dfrac{1.25}{20}=0.0625\ \mathrm m$；
- 俯仰重力矩：$\tau_{g,y}=Mg\,c_x=20\times9.81\times0.0625=12.26\ \mathrm{N\cdot m}$。

**对平衡的影响**：要让质心回到支点正上方，机身必须往"质心的反方向"倾斜一个平衡角。设质心高度 $h=0.5\ \mathrm m$，平衡倾角 $\theta_{eq}$ 满足 $h\sin\theta_{eq}+c_x\cos\theta_{eq}\approx0$，小角度下 $\theta_{eq}\approx-c_x/h$：$\sin\theta_{eq}=-0.0625/0.5=-0.125\Rightarrow\theta_{eq}=-0.125\ \mathrm{rad}=-7.2^\circ$（负号表示向后仰；REP-103 里俯仰为正是向前倾）。如果控制器的目标倾角固定为 $0^\circ$，实际上它是在持续对抗 $12.26\ \mathrm{N\cdot m}$ 的重力矩，靠反馈（尤其是积分项）慢慢找到这个偏置，手臂一动就会出现明显的姿态晃动或漂移。

**前馈**：根据手臂关节角实时算出 $c_x,c_y$ 后，把对应的补偿（以力矩偏置或姿态参考偏置的形式）加到平衡控制器里。典型的验证做法（与 CV 描述一致）：**固定手臂轨迹，开 / 关补偿做对比**，看姿态偏差、控制输出的峰值与稳态偏置是否减小。

### 8.5 实现要点

- **坐标系**：质心必须在机身系下表达，并与 IMU 的姿态定义一致（见[欧拉角、四元数与姿态表示](./02_欧拉角_四元数_姿态表示.md)第 11 节）。机身倾斜时，水平偏移要用 $R$ 把质心转到世界系再看水平分量。
- **哪个轴对应哪个电机**：三全向轮 ballbot 的 roll / pitch 力矩需要通过轮的运动学分配（见 [P1 项目](../../01_项目课程/P1_AIRS球形机器人平衡控制/README.md)）。
- **准静态**：这里只考虑静态质心偏移；手臂快速摆动产生的惯性力是动态项，另需处理。
- **精度取决于 URDF**：质量或质心错 $20\%$，前馈就偏 $20\%$。所以验证用"开关对比"，而不是只相信计算值。

> **面试要点**：能说清"质心偏移 $c_x\to$ 俯仰重力矩 $Mgc_x$、$c_y\to$ 横滚重力矩 $-Mgc_y$"，以及为什么是前馈（根据可测量的手臂关节角提前算，不等误差出现）。

- **前馈形式**：CV 写的是“把 roll/pitch 重力前馈送入平衡控制器”，本项目的做法是把补偿量作为前馈项加到姿态通道上（等价于平移平衡点，见 P1 第 7 章），再经运动学分配到三个轮子；关节状态到质心的计算与控制周期同步更新，量级为几十到一百赫兹，调参时按实机微调。
- **开 / 关对比的定性现象**：固定手臂轨迹下，关闭补偿时手臂前伸会使姿态出现持续偏差、反馈环需要占用更多裕度；开启后姿态偏差与控制输出的峰值应明显减小。这是设计预期的定性结论，不附具体数字。

---

## 9. 主从示教里的重力补偿（P2）

### 9.1 角色分工

对应 CV（P2）：*"built dual-arm dynamics from URDF/KDL, designed leader-arm gravity compensation and follower position–velocity tracking, enabling low-resistance hand-guided teaching with synchronous follower motion."*

| | 主臂（leader） | 从臂（follower） |
|---|---|---|
| 谁在推动 | 操作者用手拖 | 控制器驱动 |
| 控制目标 | 让手臂"没有重量"，被推到哪就停在哪 | 跟着主臂的关节位置与速度运动 |
| 控制律 | $\tau=g(q)$（重力补偿） | $\tau=K_p(q_d-q)+K_d(\dot q_d-\dot q)$（+ 可选前馈） |
| 增益 | 位置、速度增益为 0（学习笔记对源码的记载：`torque=effort`，只输出重力补偿力矩） | 位置、速度增益非 0 |

### 9.2 为什么主臂"推得动"

只补重力后，关节上剩下的阻力只有**摩擦、减速器的反向驱动阻力和惯性**，没有重力，也没有位置弹簧把手臂拉回原位，所以手感轻。位置、速度增益设为 0，就是不让电机去"保持某个位置"。

### 9.3 一个必须注意的点：用测得的 $q$

重力补偿 $g(q)$ 要用**当前测量到的关节角**（编码器反馈）来算，而不是目标 / 指令位置。因为重力矩取决于手臂**实际**所在的姿态。主臂由人推动，"目标"本来就不存在。

### 9.4 补偿不准会发生什么

| 现象 | 原因 |
|---|---|
| 松手后手臂缓慢下垂 | 补偿偏小（质量 / 质心偏小、增益 < 1） |
| 松手后手臂缓慢上飘，甚至自己抬起 | 补偿偏大（过补偿），可能失控 |
| 某些姿态下准、另一些姿态下不准 | 质心位置 / 关节零位有误差，误差随姿态变化 |
| 推手臂时有"粘滞感" | 摩擦未补偿，或减速比大导致反向阻力大 |

典型做法：**从 0.5～0.8 倍的补偿增益开始**，逐步调到松手悬停且不上飘；并加上力矩限幅（`limit.effort` 可作为上限参考）和位置 / 速度保护。本项目的做法是：主臂补偿带缩放系数 `gravity_comp_ratio`（0.5～0.8 起步，第 7 关节取更保守的 0.8 量级），活动路径只补重力，摩擦与科氏补偿函数存在但按 P2 笔记未启用；力矩限幅以 URDF 的 `limit.effort` 为上限。

### 9.5 从臂的位置速度跟踪

从臂用 PD：$\tau=K_p(q_d-q)+K_d(\dot q_d-\dot q)$，其中 $q_d$ 是主臂测得的关节角（经过映射），$\dot q_d$ 是主臂关节速度。若从臂也加了重力补偿 $g(q)$，位置误差就不用靠大增益去抵抗重力。电机驱动协议（例如 MIT 模式）通常把 $\tau_{ff}$、$K_p$、$K_d$ 作为一帧 CAN 报文的几个字段，由驱动器内部以高频完成 PD。

项目文档：[P2 未来动力双臂遥操作与差速底盘](../../01_项目课程/P2_未来动力双臂遥操作与差速底盘/README.md)。CAN 通信基础见 [CAN、FDCAN 与电机通信](../08_嵌入式与通信/03_CAN_FDCAN与电机通信.md)；工具链见 [MuJoCo、Pinocchio、KDL 工具链](../09_机器人软件/06_MuJoCo_Pinocchio_KDL_工具链.md)。

---

## 10. 符号、零位、关节顺序错位等故障

下面每一类都要熟练到"一听现象就能说出先查什么"。

### 10.1 符号错误

- **现象**：开重力补偿后，手臂不是"变轻"，而是被推着往下砸 / 往上冲，比不补偿更糟。
- **原因**：电机正方向与 URDF `axis` 正方向相反；公式里 $\hat a_i$ 的符号错；重力向量符号 / 方向错（$+g$ 还是 $-g$）；左右臂镜像时没取反。
- **排查**：关闭其他控制，单关节施加一个小的正力矩（例如 `limit.effort` 的 5%），看手臂往哪边转，与 $q$ 增大方向比较。

### 10.2 零位 / 偏置错误

- **现象**：某个位姿补偿准，其他位姿不准；左右臂的表现不一样；误差随位姿变化而不是固定偏移。
- **原因**：编码器零位与 URDF 零位不一致（偏了一个角度 $\delta$），导致代入 $g(q)$ 的 $q$ 偏了。
- **排查**：让手臂摆到明确的几何姿态（水平伸直、竖直下垂），对比读数与 URDF 预期；做零位标定。

**零位错的后果可以估计**：补偿误差 $\approx\dfrac{\partial g}{\partial q}\,\delta$。以第 7 节 $q_2=0$ 时的 $g_1=9.32\cos q_1$ 为例，$\partial g_1/\partial q_1=-9.32\sin q_1$：手臂越接近竖直（$\sin q_1$ 大），同样的零位误差造成的补偿误差越大；水平附近（$\sin q_1\approx0$）反而不敏感。例如 $q_1=60^\circ$ 处零位错 $5^\circ$（$0.087$ rad），补偿偏差约 $9.32\times0.866\times0.087\approx0.70\ \mathrm{N\cdot m}$，足以让手臂缓慢漂移。这也解释了"某些姿态准、某些不准"。

### 10.3 关节顺序错位

- **现象**：推关节 1，关节 3 的电机在出力；补偿力矩张冠李戴；双臂时左臂的电机在对右臂的量作反应。
- **原因**：KDL 链顺序、Pinocchio `model.names` 顺序、电机 CAN ID 顺序、ROS `JointState` 的 `name[]` 顺序四者并不一致，用"下标"直接对应。
- **排查**：始终用**关节名 $\to$ 索引**的字典；启动时打印并核对映射表；做"单关节微动"测试，看反馈的是哪个下标。

### 10.4 其他

| 故障 | 现象 | 排查 |
|---|---|---|
| 质量 / 质心缺失或为默认值 | 补偿几乎无效；或固定偏小 | 检查 URDF `inertial`；对照实物称重 |
| 单位错（mm、g） | 补偿力矩差 $10^3$ 倍，或为 0 | 检查 CAD 导出单位 |
| 力矩到电流的转换错（力矩常数、减速比） | 补偿比例整体不对（所有关节统一偏大 / 偏小） | 用已知质量悬挂测试，标定比例 |
| 重力方向没随基座姿态更新 | 机身倾斜时补偿变差（P1 型场景） | 用 $g_b=R^{\mathsf T}g_w$ |
| 摩擦与齿隙 | 低速推动有顿挫 | 加摩擦补偿，或接受 |
| 用指令位置而非反馈位置算 $g$ | 推动时补偿滞后 | 用编码器反馈 |
| 过补偿导致失控 | 松手后手臂缓慢爬升直到限位 | 降低补偿增益；限幅；急停 |
| 仿真里 `gravity` 设为零 | 仿真正常、硬件补偿缺失 | 区分仿真与硬件配置（P2 的 IK 仿真把重力关掉，因为那是运动学仿真，不是硬件补偿） |

### 10.5 一条通用的调试流程

```text
1. 静态：单关节小力矩，确认电机正方向 = URDF 轴方向。
2. 零位：摆到已知姿态，确认读数与 URDF 一致，必要时标定偏置。
3. 顺序：打印"关节名 -> 电机ID -> 索引"表，微动验证。
4. 参数：URDF 质量 / 质心，对照实物称重；单位核对。
5. 补偿：从低增益开始，水平位姿先试，再换姿态；松手测悬停。
6. 保护：力矩限幅、速度限幅、超限急停。
```

---

## 11. 面试怎么问、怎么答

**Q1. What is gravity compensation, and how do you compute it?**

> "Gravity compensation outputs, at each joint, the torque that exactly balances the gravity load at the current configuration, so the arm can hold any pose without drifting. It is the gradient of the potential energy with respect to the joint angles. In practice I load the robot's URDF with masses, centres of mass and inertia into a library like KDL, give it the measured joint angles, and read out the gravity vector from its dynamics solver."

（中文：每个关节输出刚好抵消当前位形重力载荷的力矩，等于势能对关节角的梯度；实际上把含质量 / 质心 / 惯量的 URDF 读入 KDL，用测得的关节角，从动力学求解器里读重力向量。）

*追问*：Why use measured, not commanded, joint angles? （答：重力矩取决于手臂实际姿态。）

**Q2. In your leader–follower teaching setup, why does the leader arm feel light?**

> "On the leader arm I only command the gravity compensation torque, with zero position and velocity gains. So the motors cancel the weight, but they do not try to hold any position. What remains is mainly friction and inertia, so the operator can move the arm with very little force, while the follower tracks the leader's joint position and velocity."

（中文：主臂只发重力补偿力矩，位置速度增益为零，电机抵消重量但不保持位置，剩摩擦和惯性，所以手感轻；从臂跟踪主臂关节位置和速度。）
> 这条与 CV 的 "low-resistance hand-guided teaching with synchronous follower motion" 一致。被追问实现细节时可答：补偿增益带缩放系数（0.5～0.8 量级起步），主要补重力、不依赖摩擦补偿，力矩受 `limit.effort` 限幅，通信频率为百赫兹量级（调参时按实机微调）。

**Q3. What did you do about centre-of-mass changes caused by the arms on your balancing robot?**

> "I estimated the centre-of-mass offset from the arm joint states using the URDF and Pinocchio, and fed roll and pitch gravity feedforward into the balance controller. The forward offset of the centre of mass creates a pitch gravity torque of M g times the offset, and the sideways offset gives the roll torque. To check it, I toggled the compensation on and off under fixed arm trajectories and compared the behaviour."

（中文：用 URDF 与 Pinocchio 从手臂关节状态估质心偏移，把 roll / pitch 重力前馈加入平衡控制器；前后偏移产生俯仰重力矩 $Mg\,c_x$，左右偏移产生横滚力矩；在固定手臂轨迹下开 / 关补偿做对比验证。）
> 与 CV 一致。具体形式与观察结果见 8.5 节：补偿作为前馈加在姿态通道，对比为定性结论。

**Q4. What can go wrong with a URDF-based dynamic model on real hardware?**

> "Mostly mismatches between the model and the hardware: wrong masses or centres of mass, joint zero offsets, sign conventions of the axes, joint ordering between the model and the motor drivers, units, and torque-to-current scaling. I debug in order: motor direction, zero position, joint order, then parameters, and I start with a low compensation gain."

**Q5. Is gravity compensation the same as inverse dynamics control?**

> "No. Gravity compensation only cancels the configuration-dependent gravity term. Full inverse dynamics also includes the inertia and velocity-coupling terms. At low speed, as in hand-guided teaching, gravity dominates, so gravity compensation alone is already a good approximation."

---

## 12. 本章要点速记

1. URDF 由 `link`（惯性：质量、质心、惯量）与 `joint`（`origin` + `axis` + `limit`）组成，只能描述树，不能描述闭环。
2. `axis` 与电机正方向必须一致；`axis` 取反，重力项整体取反。
3. 惯性张量是绕**质心**的；细杆 $I_c=ml^2/12$，绕端点 $ml^2/3$。
4. KDL 以链为主，Pinocchio 原生支持树和浮动基座；注意 `nq` 与 `nv`，以及名字到索引的映射。
5. 动力学：$M\ddot q+C\dot q+g=\tau$；重力项 $g(q)=\partial U/\partial q$，只依赖位形。
6. 重力矩 = 重量 × 水平力臂；水平伸直最大，竖直为 0。
7. RNEA 令 $\dot q=\ddot q=0$ 即得重力项。
8. 低速时重力项占主导（本例 7.5 对 0.44），所以只补重力就有很轻的手感。
9. 质心偏移：俯仰力矩 $=Mg\,c_x$，横滚力矩 $=-Mg\,c_y$；作为前馈，用开 / 关对比验证。
10. 主臂：重力补偿，位置 / 速度增益为 0，用反馈 $q$；从臂：位置 / 速度跟踪。
11. 故障排查顺序：方向 $\to$ 零位 $\to$ 关节顺序 $\to$ 参数与单位 $\to$ 增益从低到高。

## 自测题

1. 均匀细杆 $m=1.5$ kg、长 $0.6$ m，绕端点水平转轴，在水平位置时的重力补偿力矩？
   *答案*：$mg\cdot l/2=1.5\times9.81\times0.3=4.41$ N·m。
2. 同一杆在与水平成 $60^\circ$ 时的补偿力矩？
   *答案*：$4.41\times\cos60^\circ=2.21$ N·m。
3. 单关节 $m=2$ kg，质心 $0.5$ m，要以 $\ddot q=3$ rad/s² 从水平加速，忽略摩擦，需要多大力矩？（$I_{pivot}=2/3$ kg·m²）
   *答案*：$I\ddot q+mg\,l_c=\tfrac23\times3+2\times9.81\times0.5=2+9.81=11.81$ N·m。
4. 为什么 `axis` 写反会让重力补偿"变成加倍下垂"？
   *要点*：重力项是轴向量与力臂的叉积，$\hat a\to-\hat a$ 则 $g_i\to-g_i$，补偿方向与需要相反。
5. 整机质心前移 $0.04$ m，总质量 $18$ kg，估算俯仰重力矩。
   *答案*：$18\times9.81\times0.04=7.06$ N·m。
6. 为什么重力补偿要用测量的 $q$ 而不是目标位置？
   *要点*：重力矩取决于手臂实际姿态；主臂由人推动，没有目标。
7. 推关节 1，却发现关节 3 的电机出力。最可能的原因和排查办法？
   *要点*：关节顺序映射错位；用名字映射并做微动验证。
8. 低速手拖示教为何只补重力就够？
   *要点*：重力项比惯性项、科氏项大一个数量级以上，补掉重力后只剩摩擦与小惯性。

## 相关链接

- 上一章：[闭链并联机构与五连杆运动学](./05_闭链并联机构与五连杆运动学.md)
- 本组第一章：[坐标系、旋转矩阵与齐次变换](./01_坐标系_旋转矩阵_齐次变换.md)
- 基础：[牛顿欧拉与刚体力学](../02_力学与建模/01_牛顿欧拉与刚体力学.md)、[拉格朗日方程与机器人动力学](../02_力学与建模/02_拉格朗日方程与机器人动力学.md)、[前馈、增益调度、限幅与饱和](../04_控制理论/07_前馈_增益调度_限幅与饱和.md)、[MuJoCo、Pinocchio、KDL 工具链](../09_机器人软件/06_MuJoCo_Pinocchio_KDL_工具链.md)、[URDF、TF、坐标系实践](../09_机器人软件/07_URDF_TF_坐标系实践.md)
- 项目：[P2 未来动力双臂遥操作与差速底盘](../../01_项目课程/P2_未来动力双臂遥操作与差速底盘/README.md)、[P1 AIRS 球形机器人平衡控制](../../01_项目课程/P1_AIRS球形机器人平衡控制/README.md)
- 学习笔记原始问答：[P2 问答逐字稿（第二份）](../../05_学习笔记归档/P2/P2_本对话问答逐字稿_第二份.md)
