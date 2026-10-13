# 07 URDF、TF 与坐标系实践

> **一句话定位**：URDF 回答"机器人身上各个零件**固定地**怎么连"，TF 回答"此刻任意两个坐标系**相对地**在哪"；两者合起来是几乎所有机器人软件的"地基"——传感器数据能不能用、控制量朝哪个方向、规划的路径画在哪，都取决于坐标系有没有处理对。

**本章学完你能：**

- 读懂并写出一份能被 Pinocchio / KDL / rviz 正确解析的 URDF，指出常见的写法错误；
- 用 xacro 去掉重复，理解 `property`、`macro`、`arg`、`include`；
- 讲清关节零位、轴方向、`origin` 的 `rpy` 约定对 $q$ 符号的影响，并手算从电机编码器读数到 URDF 关节角的映射；
- 用 tf2 查询变换：理解 `lookupTransform(target, source, time)` 的方向和时间语义，区分静态与动态 TF；
- 手算一个 `map → odom → base_link → laser` 的点变换，并解释乘法顺序错了会得到什么；
- 按 REP-103 / REP-105 给坐标系命名，写出 `frame_id` 与时间戳的入口校验；
- 看懂 rviz 里的 TF/RobotModel/LaserScan，并系统排查跳变、外推（extrapolation）、循环/多父等 TF 故障；
- 把以上知识对应到 P1 / P2 / P5 / P6。

**前置知识：**

- [坐标系、旋转矩阵、齐次变换](../03_坐标变换与运动学/01_坐标系_旋转矩阵_齐次变换.md)
- [欧拉角、四元数、姿态表示](../03_坐标变换与运动学/02_欧拉角_四元数_姿态表示.md)
- [ROS1 / ROS2 核心概念](./04_ROS1_ROS2核心概念.md)（话题、时间、QoS）
- [时间同步、坐标系一致性与数据有效性](../05_状态估计/06_时间同步_坐标系一致性与数据有效性.md)

> **关于代码的说明**：URDF/xacro 示例的数值在 xacro 2.1.1 与 Pinocchio 4.1.0 下展开并解析核对过；Python 校验函数在本地运行核对过；涉及 ROS 运行环境的 tf2、launch 示例（`rclpy`、`rviz2`）**未运行核对**，仅作结构示意。示例数值均为教学示例，非任何项目实测。

**目录**

1. 坐标系为什么是地基
2. URDF 语法与结构
3. URDF 常见错误
4. 关节零位、轴方向与编码器映射（含手算）
5. xacro
6. TF2：变换树、查询与时间
7. 静态与动态 TF，`robot_state_publisher`
8. 坐标系命名规范：REP-103 与 REP-105
9. `frame_id` 与时间戳校验
10. rviz 可视化
11. 常见 TF 故障与排查
12. 在本项目里怎么用（P1 / P2 / P5 / P6）
13. 面试怎么问、怎么答
14. 自测题
15. 本章要点速记与相关链接

---

## 1. 坐标系为什么是地基

### 1.1 要解决的问题

一个数字没有"在哪个坐标系下"就没有意义。"障碍物在前方 1 m"——前方是机器人前方，还是地图的 $+x$？一个 IMU 读到 $a_x=2\ \text{m/s}^2$——$x$ 是芯片外壳的 $x$，还是车体的 $x$？激光点云的点相对激光雷达，规划需要的却是相对地图。

机器人上有几十个坐标系：世界/地图、里程计、车体、各传感器、每个关节、末端、工具……**TF** 把它们组织成一棵树，随时回答"A 在 B 中的位姿"；**URDF** 描述其中固定不变的那部分几何与惯性结构。

### 1.2 符号约定（全章统一）

- 记 ${}^{a}T_{b}$ 为**坐标系 $b$ 相对于坐标系 $a$ 的位姿**（$b$ 的原点与方向在 $a$ 下的表示）。
- 点的变换：${}^{a}p = {}^{a}T_{b}\ {}^{b}p$（把 $b$ 系下的点表示到 $a$ 系下）。
- 链式法则：${}^{a}T_{c}={}^{a}T_{b}\,{}^{b}T_{c}$（下标"相邻相消"，从左到右）。
- 平面情形 $SE(2)$ 的矩阵：$\begin{bmatrix}\cos\theta & -\sin\theta & x\\ \sin\theta & \cos\theta & y\\ 0&0&1\end{bmatrix}$，$\theta$ 逆时针为正（右手系，绕 $z$ 向上）。
- 单位：米、弧度、秒、千克（REP-103）。

### 1.3 记忆口诀

> "**父在左，子在右；点从右往左变。**"
> TF 里每条边的含义是：父坐标系 → 子坐标系，变换给出**子在父中的位姿**，即 ${}^{\text{parent}}T_{\text{child}}$。

---

## 2. URDF 语法与结构

### 2.1 URDF 是什么

URDF（Unified Robot Description Format）是一份 XML：机器人是一棵由 **link（连杆）** 和 **joint（关节）** 组成的**树**。它主要描述：

1. 几何/外观（visual）和碰撞体（collision）；
2. 惯性参数（inertial：质量、质心、惯量）；
3. 关节类型、轴、限位、父子关系和相对位姿。

它**不**描述控制器、传感器噪声模型、动力学接触参数（这些在别的扩展里）。URDF 相当于说明书；Pinocchio、KDL、rviz、`robot_state_publisher` 是读这本说明书的"计算器"。

### 2.2 link

```xml
<link name="link1">
  <visual>      <!-- 给人看的外观 (rviz) -->
    <origin xyz="0.2 0 0" rpy="0 1.5708 0"/>
    <geometry><cylinder radius="0.02" length="0.4"/></geometry>
  </visual>
  <collision>   <!-- 给碰撞检测用, 常用比 visual 更简单的形状 -->
    <origin xyz="0.2 0 0" rpy="0 1.5708 0"/>
    <geometry><cylinder radius="0.02" length="0.4"/></geometry>
  </collision>
  <inertial>    <!-- 给动力学用 -->
    <origin xyz="0.2 0 0" rpy="0 0 0"/>   <!-- 质心相对 link 坐标系的位置 -->
    <mass value="1.0"/>                    <!-- kg -->
    <inertia ixx="0.001" iyy="0.001" izz="0.001" ixy="0" ixz="0" iyz="0"/>
  </inertial>
</link>
```

要点：

- `inertial/origin` 是**质心**的位置（及惯量主轴方向），不是几何中心；惯量张量是**绕质心**、在 `inertial` 坐标系下表示的；
- 质量必须为正，惯量矩阵必须物理有效（对称正定，且满足三角不等式：任意两个主惯量之和不小于第三个）；
- `visual` 与 `collision` 可以不一样；碰撞体用简化形状可大幅降低计算量；
- 网格文件常写 `package://my_robot_description/meshes/arm.stl`，前提是该包已被 `source` 并能被找到；STL 的单位是**毫米还是米**要用 `scale` 说明。

### 2.3 joint

```xml
<joint name="joint2" type="revolute">
  <parent link="link1"/>
  <child  link="link2"/>
  <origin xyz="0.4 0 0" rpy="0 0 0"/>   <!-- 关节(=子link坐标系)在父link坐标系中的位姿 (q=0 时) -->
  <axis xyz="0 -1 0"/>                  <!-- 转轴, 在关节(子link)坐标系下表达 -->
  <limit lower="-3.14" upper="3.14" effort="50" velocity="3"/>
  <dynamics damping="0.01" friction="0.05"/>   <!-- 可选 -->
</joint>
```

**关节类型：**

| 类型 | 自由度 | 说明 |
|---|---|---|
| `revolute` | 1 | 有限位的转动关节，**必须**写 `limit`（lower/upper/effort/velocity） |
| `continuous` | 1 | 无限位的转动（轮子） |
| `prismatic` | 1 | 滑动（直线） |
| `fixed` | 0 | 固连（传感器安装位置） |
| `floating` | 6 | 浮动基（很少直接写进 URDF，多由库在根上加） |
| `planar` | 3 | 平面运动 |

**`origin` 的含义（最容易搞错）**：`origin` 描述的是**当关节角 $q=0$ 时，子 link 坐标系相对父 link 坐标系的位姿**。关节转动时，子 link 绕 `axis`（在子 link 坐标系下表达）转 $q$。所以 URDF 里一个关节的完整变换是

$$
{}^{\text{parent}}T_{\text{child}}(q) = \underbrace{T_{\text{origin}}}_{\text{xyz, rpy}}\ \cdot\ \text{Rot}(\text{axis},\,q)
$$

**`rpy` 约定**：绕**固定**（父）坐标系的 X、Y、Z 依次转 roll、pitch、yaw，等价的旋转矩阵是

$$
R=R_z(\text{yaw})\,R_y(\text{pitch})\,R_x(\text{roll})
$$

**一个关键直觉**：URDF 的结构是"树"——每个 link 恰有一个父关节，根 link 没有。**闭链**（五连杆、并联机构）不能直接用一棵树表示，常见做法是在建模时把闭链"剪开"，用约束处理（见 [闭链并联机构与五连杆运动学](../03_坐标变换与运动学/05_闭链并联机构与五连杆运动学.md)），或使用支持闭环的描述格式。

### 2.4 一份完整的小例子

两连杆臂，并在末端装一个相机（用于演示固定关节与光学坐标系，后面会反复用）：

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
  <link name="camera_link"/>
  <link name="camera_optical_frame"/>

  <joint name="joint1" type="revolute">
    <parent link="base"/><child link="link1"/><origin xyz="0 0 0"/>
    <axis xyz="0 -1 0"/><limit lower="-3.14" upper="3.14" effort="50" velocity="3"/>
  </joint>
  <joint name="joint2" type="revolute">
    <parent link="link1"/><child link="link2"/><origin xyz="0.4 0 0"/>
    <axis xyz="0 -1 0"/><limit lower="-3.14" upper="3.14" effort="50" velocity="3"/>
  </joint>
  <joint name="camera_mount" type="fixed">
    <parent link="link2"/><child link="camera_link"/><origin xyz="0.4 0 0.03"/>
  </joint>
  <joint name="camera_optical_joint" type="fixed">
    <parent link="camera_link"/><child link="camera_optical_frame"/>
    <origin xyz="0 0 0" rpy="-1.5708 0 -1.5708"/>   <!-- 机体系 -> 光学系 (REP-103) -->
  </joint>
</robot>
```

### 2.5 手算：从 URDF 推末端位置

取 $q_1=30^\circ,\ q_2=45^\circ$（轴为 $-y$，所以正的 $q$ 把 $+x$ 轴向 $+z$ 抬）。每个关节的 ${}^{\text{parent}}T_{\text{child}}$：

- ${}^{\text{base}}T_{\text{link1}}$：平移 $(0,0,0)$，绕 $-y$ 转 $30^\circ$（在 $x$–$z$ 平面里，等价于"向上抬 30°"）；
- ${}^{\text{link1}}T_{\text{link2}}$：先平移 $(0.4,0,0)$，再转 $45^\circ$。

所以 link2 的原点在 base 系下：

$$
x=0.4\cos 30^\circ=0.3464,\qquad z=0.4\sin30^\circ=0.2
$$

link2 的方向是 $30^\circ+45^\circ=75^\circ$。相机在 link2 里 $(0.4, 0, 0.03)$：沿 link2 的 $x$ 方向 0.4，再沿 link2 的 $z$ 方向 0.03。link2 的 $x$ 轴在 base 系里是 $(\cos75^\circ,\ \sin75^\circ)=(0.2588,\ 0.9659)$（$x$,$z$ 分量），link2 的 $z$ 轴在 base 里是 $(-\sin75^\circ,\ \cos75^\circ)=(-0.9659,\ 0.2588)$。

$$
x_{\text{cam}} = 0.3464 + 0.4\times0.2588 + 0.03\times(-0.9659) = 0.3464+0.1035-0.0290=0.4209
$$

$$
z_{\text{cam}} = 0.2 + 0.4\times0.9659 + 0.03\times0.2588 = 0.2+0.3864+0.0078=0.5942
$$

（若不计 0.03 的 z 偏移，末端为 $(0.4499,\ 0.5864)$，这是不带相机偏置的两连杆末端位置；带偏置的 $(0.4209,\ 0.5941)$ 已用 Pinocchio 核对。）**用 tf 和 FK 库算出的结果应与手算吻合；不一致就要查 `origin`、`axis`、顺序。**

### 2.6 光学坐标系（optical frame）为什么要 `rpy="-1.5708 0 -1.5708"`

机器人本体系（REP-103）：$x$ 前、$y$ 左、$z$ 上。相机算法（OpenCV 等）的**光学系**：$z$ 沿光轴向前、$x$ 向右、$y$ 向下。从机体系转到光学系，需要把光学系的 $z$ 对齐机体系的 $x$。

验证：$R=R_z(-\pi/2)R_y(0)R_x(-\pi/2)$，其三个列向量（光学系坐标轴在机体系下的方向）为

- 光学 $x$ = $(0,-1,0)$ → 即机体系的**右**（$-y$）✓；
- 光学 $y$ = $(0,0,-1)$ → 机体系的**下**（$-z$）✓；
- 光学 $z$ = $(1,0,0)$ → 机体系的**前**（$+x$）✓。

作者用 NumPy 核对了这个旋转。这个约定使得图像检测到的点云/射线可以通过 TF 直接放进机体系。

---

## 3. URDF 常见错误

| 错误 | 现象 | 原因与修正 |
|---|---|---|
| 多个根 link / 孤立 link | 解析报"多个根"；TF 树断开 | 每个 link 恰有一个父关节；用 `check_urdf` 检查 |
| `joint` 的 `parent`/`child` 写反 | 机器人"倒着长" | 检查父子；`urdf_to_graphviz` 画树 |
| `revolute` 缺 `limit` | 解析报错 | 写 lower/upper/effort/velocity |
| `axis` 在错误坐标系下理解 | 关节转向与预期相反 | `axis` 在**子 link 坐标系**（关节坐标系）下表达，且受 `origin rpy` 影响 |
| `rpy` 单位当度数 | 姿态严重歪斜 | URDF 的角度单位是**弧度** |
| 单位错（mm 当 m） | 模型巨大或极小 | STL 若为毫米，设 `scale="0.001 0.001 0.001"` |
| 质量为 0 或负；惯量非正定 | 动力学库报错、仿真爆炸 | 给物理合理的质量与惯量；根 link 通常不带惯量 |
| 质心 `origin` 填成几何中心 | 重力补偿不准 | 用 CAD 导出真实质心 |
| 惯量张量表示的参考点/坐标系错 | 动力学细微偏差 | 惯量是绕质心、在 `inertial` 坐标系下 |
| 网格路径 `package://` 找不到 | rviz 里只有坐标轴没有形状 | 包名、安装路径、`source` |
| 关节名与控制/驱动里的名字不一致 | `joint_states` 对不上 | 约定名字，用映射表 |
| 想用 URDF 表达闭链 | 无法表达 | 剪开闭链并用约束，或换格式 |

**检查命令（ROS 环境）：**

```bash
check_urdf my_robot.urdf              # 解析并打印树结构
urdf_to_graphviz my_robot.urdf        # 生成 pdf 树图
xacro my_robot.urdf.xacro > out.urdf  # xacro 展开
```

此外，一个最朴素也最有效的验证是：用 Pinocchio 或 KDL 加载同一份 URDF，在几个姿态下算 $g(q)$ 和末端位置，与手算比对（见 [MuJoCo / Mink / Pinocchio / KDL 工具链](./06_MuJoCo_Pinocchio_KDL_工具链.md)）。

---

## 4. 关节零位、轴方向与编码器映射（含手算）

### 4.1 要解决的问题

URDF 里 $q=0$ 对应的姿态，与实物电机编码器读数为 0 的姿态，通常**不是**同一个；轴的正方向与电机的正转方向也未必相同。如果没有对齐就把编码器角度直接当 URDF 关节角用，重力补偿、IK、可视化全部会出错。

### 4.2 映射公式

设电机编码器读数（电机侧，已折算到关节侧）为 $\theta_m$，URDF 零位对应的读数为 $\theta_0$（"零位偏置"），方向系数 $s\in\{+1,-1\}$（电机正转与 URDF 轴正方向相同取 $+1$）。则

$$
q_{\text{urdf}} = s\,(\theta_m-\theta_0),\qquad \theta_m = \theta_0 + s\,q_{\text{urdf}}
$$

若电机侧有减速比 $N$（电机转 $N$ 圈关节转 1 圈），则 $\theta_{\text{joint}}=\theta_{\text{motor}}/N$，要先折算。

### 4.3 手算示例（教学数值）

某关节：减速比 $N=10$；把机械臂摆到 URDF 的零位时，电机端读数为 $\theta_{\text{motor}}=21.0\ \text{rad}$，折算到关节侧 $\theta_0=2.10\ \text{rad}$；电机正转与 URDF 轴相反，$s=-1$。

| 电机端读数 $\theta_{\text{motor}}$ | 关节侧 $\theta_m=\theta_{\text{motor}}/10$ | $q_{\text{urdf}}=-(\theta_m-2.10)$ |
|---|---|---|
| 21.0 | 2.10 | 0 |
| 26.0 | 2.60 | −0.50 |
| 16.0 | 1.60 | +0.50 |

力矩要同样处理：URDF 轴方向的力矩 $\tau_{\text{urdf}}$ 对应电机力矩 $\tau_{\text{motor}}=s\,\tau_{\text{urdf}}/N$（忽略效率；$s=-1$ 时符号反）。**符号反了的后果**：重力补偿变成"推着往下压"，手感"越托越重"。

### 4.4 轴方向写反的后果（作者代码核对）

把 2.4 节两连杆臂的 `axis` 从 `0 -1 0` 改成 `0 1 0`：

| | `axis = 0 -1 0` | `axis = 0 1 0` |
|---|---|---|
| $q=(0.5,0)$ 时 link2 原点 $z$ | $+0.1918$ | $-0.1918$ |
| $q=(0.3,0.2)$ 时 $g(q)$ | $[7.345,\ 1.722]$ | $[-7.345,\ -1.722]$ |

即**手臂变成往下弯，补偿力矩整体变号**。常见排错法：让某个关节单独动一个小角度（例如 +0.1 rad），在 rviz 里看它往哪边转，与实物对比；不一致就是 `axis` 或 $s$ 有问题。

### 4.5 零位错位的后果

零位偏差 $\delta$ 会让重力力矩的力臂算错。以 2.1 节水平臂为例，$g_1\propto\cos q_1$；若 $q_1$ 真实为 $0$ 而模型认为是 $\delta=0.1\ \text{rad}$，则相对误差 $1-\cos0.1=0.5\%$，影响很小；但若零位偏差是 $\pi/2$（比如把"朝上"当成"水平"），$g_1$ 会从最大变成接近 0——补偿完全失效。**零位对齐必须用一个可测量的参考姿态（机械限位、定位销、标定位姿）而不是目测。**

---

## 5. xacro

### 5.1 它解决什么问题

URDF 是纯 XML，没有变量、没有函数。双臂机器人的左右臂几乎一样，复制粘贴会让改一处要改十处。**xacro（XML Macros）** 在 URDF 外加一层预处理：属性、宏、条件、数学表达式、文件包含，展开后得到纯 URDF。

### 5.2 常用语法

| 语法 | 作用 |
|---|---|
| `<xacro:property name="L" value="0.4"/>` | 定义常量，用 `${L}` 引用 |
| `${L/2}`、`${pi/2}` | 表达式（支持常见数学函数和 `pi`） |
| `<xacro:macro name="rod_link" params="name mass length">…</xacro:macro>` | 定义宏 |
| `<xacro:rod_link name="link1" mass="1.0" length="0.4"/>` | 调用宏 |
| `<xacro:arg name="prefix" default=""/>` + `$(arg prefix)` | 命令行参数 |
| `<xacro:include filename="$(find pkg)/urdf/x.xacro"/>` | 包含其它文件 |
| `<xacro:if value="${cond}">…</xacro:if>` | 条件 |

### 5.3 示例（作者在 xacro 2.1.1 下展开并核对）

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="arm2r">
  <xacro:property name="L" value="0.4"/>
  <xacro:property name="M" value="1.0"/>

  <xacro:macro name="rod_link" params="name mass length">
    <link name="${name}">
      <visual>
        <origin xyz="${length/2} 0 0" rpy="0 ${pi/2} 0"/>
        <geometry><cylinder radius="0.02" length="${length}"/></geometry>
      </visual>
      <inertial>
        <origin xyz="${length/2} 0 0"/>
        <mass value="${mass}"/>
        <inertia ixx="0.001" iyy="0.001" izz="0.001" ixy="0" ixz="0" iyz="0"/>
      </inertial>
    </link>
  </xacro:macro>

  <link name="base"/>
  <xacro:rod_link name="link1" mass="${M}" length="${L}"/>
  <xacro:rod_link name="link2" mass="${M}" length="${L}"/>
  <joint name="joint1" type="revolute">
    <parent link="base"/><child link="link1"/><origin xyz="0 0 0"/>
    <axis xyz="0 -1 0"/><limit lower="-3.14" upper="3.14" effort="50" velocity="3"/>
  </joint>
  <joint name="joint2" type="revolute">
    <parent link="link1"/><child link="link2"/><origin xyz="${L} 0 0"/>
    <axis xyz="0 -1 0"/><limit lower="-3.14" upper="3.14" effort="50" velocity="3"/>
  </joint>
</robot>
```

展开命令：`xacro arm.urdf.xacro > out.urdf`。作者核对：展开后 `<origin xyz="0.2 0 0" rpy="0 1.5707963267948966 0"/>` 等值正确，且用 Pinocchio 加载后 $g(0,0)=[7.848,\ 1.962]$，与第 06 章手算一致。

### 5.4 双臂的惯用写法

左右臂用同一个宏，通过 `prefix` 区分命名，通过 `reflect`（$\pm1$）镜像 `y` 方向的安装位置：

```xml
<xacro:macro name="arm" params="prefix reflect">
  <joint name="${prefix}_joint1" type="revolute"> ... <origin xyz="0 ${reflect*0.15} 0"/> ... </joint>
  ...
</xacro:macro>
<xacro:arm prefix="left"  reflect="1"/>
<xacro:arm prefix="right" reflect="-1"/>
```

> **注意**：镜像不只是坐标取反——转轴方向、关节限位的正负也要对应处理，否则左右臂的"正方向"定义不一致，主从映射会出现反向。

### 5.5 xacro 常见错误

| 错误 | 现象 | 修正 |
|---|---|---|
| 忘了声明命名空间 `xmlns:xacro` | 展开失败 | 根标签加声明 |
| 表达式里除法用整数 | 得到取整结果 | 用 `1.0/2` 或小数 |
| 宏参数没传 | 展开报缺参数 | 检查 `params` |
| 用了 URDF 里没有的标签 | 被后续工具忽略 | 写成 gazebo/ros2_control 扩展标签的正确位置 |
| 展开结果没检查就交给仿真 | 隐蔽的尺寸/符号错 | 展开后跑 `check_urdf` |

---

## 6. TF2：变换树、查询与时间

### 6.1 TF 的数据模型

- **TransformStamped**：`header.stamp`（变换有效的时间）、`header.frame_id`（**父**坐标系）、`child_frame_id`（**子**坐标系）、`transform`（平移 + 四元数，表示**子在父中的位姿**）。
- 发布到话题 `/tf`（动态）与 `/tf_static`（静态，transient local，晚加入的监听者也能拿到）。
- 监听者（`tf2_ros::Buffer` + `TransformListener`）把收到的变换按时间存成**历史缓存**（默认约 10 秒），可以回答"某个时刻"的变换。
- **树的约束**：每个子坐标系只能有**一个**父；不能有环；两个不连通的树之间无法查询。

### 6.2 查询的方向：`lookupTransform(target_frame, source_frame, time)`

返回的是把**源坐标系（source）下的数据变换到目标坐标系（target）下**的变换，即 ${}^{\text{target}}T_{\text{source}}$。使用方式：

$$
{}^{\text{target}}p = {}^{\text{target}}T_{\text{source}}\ {}^{\text{source}}p
$$

典型：`lookupTransform("map", "laser", t)` 返回 ${}^{\text{map}}T_{\text{laser}}$，也就是"laser 坐标系在 map 里的位姿"，用它把激光点变到 map 下。

> **注意**：不同工具的命令行（如 `tf2_echo`）对两个参数的叫法和输出说明在不同版本里措辞不一。用法上务必**用一个已知位置的两帧做一次实测**来确认方向，再信任它。

### 6.3 时间语义

查询带一个时间参数：

- 指定时刻 $t$：缓存里在 $t$ 前后各有一个样本时，**线性插值**平移、**球面插值**旋转；
- `Time(0)`（零时刻，rclcpp 的 `tf2::TimePointZero`）：表示**最新可用**的变换，不做时间对齐——方便但**会掩盖延迟**，只适合"我只想要现在的大概位置"，不适合处理带时间戳的传感器数据；
- $t$ 晚于缓存里最新样本 → **外推到未来**（extrapolation into the future）→ 报错；
- $t$ 早于缓存里最旧样本 → 外推到过去 → 报错。

**手算插值（教学数值）**：缓存里有 `odom→base_link` 两个样本：$t_0=10.00$ s：$x=1.00$；$t_1=10.05$ s（20 Hz）：$x=1.10$。

- 查询 $t=10.03$：$\alpha=(10.03-10.00)/0.05=0.6$，$x=1.00+0.6\times0.10=1.06$ ✓（线性插值）；旋转用 slerp，同样按 $\alpha=0.6$；
- 查询 $t=10.07$：晚于最新样本 10.05，需要外推 0.02 s → 报 *Lookup would require extrapolation into the future*；
- 解决：`waitForTransform` / 带 `timeout` 的 `lookupTransform`，等 TF 到；或用 `tf2_ros::MessageFilter` 让消息等到 TF 可用再处理。

**因此：处理激光/图像等带时间戳的数据时，用数据的采样时间 `header.stamp` 去查 TF，而不是用 `now()`。** 用 `now()` 会把"现在的车体位姿"用在"几十毫秒前的激光点"上；车在动，结果就是点云拖影。

**手算：拖影有多大。** 车速 $v=1.5\ \text{m/s}$，激光数据从采样到被处理延迟 $\Delta t=80\ \text{ms}$（示例），如果用 `now()` 的位姿而不是采样时刻的位姿，点云在地图里的系统偏移约 $v\Delta t=1.5\times0.08=0.12\ \text{m}$。转弯时旋转误差还会按距离放大：角速度 $\omega=0.5\ \text{rad/s}$ → $\Delta\theta=0.04\ \text{rad}$；10 m 外的点横向偏差 $\approx 10\times0.04=0.4\ \text{m}$。

### 6.4 手算：`map → odom → base_link → laser` 的点变换

示例（教学数值，平面）：

- ${}^{\text{map}}T_{\text{odom}}$：平移 $(2.0,\ 1.0)$，旋转 $0^\circ$（定位给的修正）；
- ${}^{\text{odom}}T_{\text{base}}$：平移 $(3.0,\ 0)$，旋转 $90^\circ$（车在 odom 下的位姿，车头朝 $+y$）；
- ${}^{\text{base}}T_{\text{laser}}$：平移 $(0.2,\ 0)$，旋转 $0^\circ$（激光雷达装在车体前方 0.2 m，静态外参）；
- 激光在自己的坐标系下看到一个点 ${}^{\text{laser}}p=(1,\ 0)$（正前方 1 m）。

逐级变换（点从右往左变）：

1. laser → base：${}^{\text{base}}p=(1+0.2,\ 0+0)=(1.2,\ 0)$；
2. base → odom：先旋转 $90^\circ$：$R(90^\circ)(1.2,0)^\top=(0,\ 1.2)^\top$，再平移 $(3,0)$：${}^{\text{odom}}p=(3.0,\ 1.2)$；
3. odom → map：平移 $(2,1)$：${}^{\text{map}}p=(5.0,\ 2.2)$。

最终 ${}^{\text{map}}p=(5.0,\ 2.2)$。**直觉校验**：车头朝 $+y$，激光点在车前方 1.2 m 处（含 0.2 m 偏置），车的位置 $(3,0)$ 平移到 map 后是 $(5,1)$，往 $+y$ 方向 1.2 m 得 $(5,\ 2.2)$ ✓。

**如果变换顺序错了**（例如把三个矩阵按相反顺序相乘，即先用 ${}^{\text{base}}T_{\text{laser}}$ 作用于点，再依次乘 ${}^{\text{odom}}T_{\text{base}}$、${}^{\text{map}}T_{\text{odom}}$ 的顺序写成了 ${}^{\text{base}}T_{\text{laser}}\,{}^{\text{odom}}T_{\text{base}}\,{}^{\text{map}}T_{\text{odom}}\,{}^{\text{laser}}p$），作者核对得到 $(2.2,\ 3.0)$，与正确答案 $(5.0,\ 2.2)$ 相差很大。顺序错、方向错（用了逆变换却当成正变换）是 TF 手算与程序里最常见的两类错误。

### 6.5 Python 查询示例（教学示例，未运行核对）

```python
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformListener, TransformException

class TfProbe(Node):
    def __init__(self):
        super().__init__("tf_probe")
        self.buf = Buffer()                           # 缓存历史变换
        self.lis = TransformListener(self.buf, self)  # 订阅 /tf 和 /tf_static
        self.create_timer(0.1, self.tick)

    def tick(self):
        try:
            # 查 "map" 下 "base_link" 的位姿, 时间=最新可用 (Time())
            tf = self.buf.lookup_transform("map", "base_link", Time(),
                                           timeout=Duration(seconds=0.0))
        except TransformException as ex:
            self.get_logger().warn(f"no tf yet: {ex}")
            return
        p = tf.transform.translation
        self.get_logger().info(f"base_link in map: x={p.x:.3f} y={p.y:.3f}")

def main():
    rclpy.init()
    rclpy.spin(TfProbe())

if __name__ == "__main__":
    main()
```

如果要把某个带时间戳的传感器消息变到另一个坐标系，应使用消息的 `header.stamp` 构造 `Time.from_msg(msg.header.stamp)` 去查，并设置合理的 `timeout`。

### 6.6 "时间旅行"查询（进阶）

有时需要"过去某时刻的传感器数据，用过去某时刻的车体位姿，变换到现在的固定坐标系"，tf2 提供六参数形式：`lookupTransform(target_frame, target_time, source_frame, source_time, fixed_frame)`。含义：先把 source 在 source_time 的数据，经**固定坐标系**（不随时间变，如 `odom` 或 `map`）转到 target_time，再变到 target。用于"用最新位姿处理旧测量"的时间补偿，比如运动畸变校正。

---

## 7. 静态与动态 TF，`robot_state_publisher`

### 7.1 区别

| | 静态 TF | 动态 TF |
|---|---|---|
| 话题 | `/tf_static` | `/tf` |
| 内容 | 固定外参：传感器安装位置、基座相对世界 | 随时间变化：里程计、定位修正、关节 |
| 发布 | 只需发一次（transient local，晚到者能收到） | 持续发布，须有稳定频率 |
| 典型发布者 | `static_transform_publisher`、`robot_state_publisher`（URDF 中的 fixed joint） | 里程计/定位节点、`robot_state_publisher`（根据 `joint_states` 的非固定关节） |

命令行发布静态变换（示例，参数格式随版本略有差别，先 `-h`）：

```bash
ros2 run tf2_ros static_transform_publisher --x 0.2 --y 0 --z 0.1 \
    --yaw 0 --pitch 0 --roll 0 --frame-id base_link --child-frame-id laser
```

### 7.2 `robot_state_publisher`

输入：参数 `robot_description`（URDF 字符串）和话题 `joint_states`。输出：URDF 里所有关节对应的 TF——固定关节发到 `/tf_static`，可动关节用 `joint_states` 里的角度算出变换发到 `/tf`。

```text
 joint_states (关节角, 来自硬件桥)        robot_description (URDF)
            \                                  /
             +---------> robot_state_publisher <--+
                            |              |
                       /tf (可动关节)   /tf_static (固定关节)
```

**两个关键点**：

1. `joint_states` 里的**关节名必须与 URDF 的关节名一致**，否则对应的 TF 不会更新（机器人在 rviz 里"站着不动"）；
2. 它发布的是**机器人内部（本体各 link 之间）**的 TF。`map→odom`、`odom→base_link` 要由定位与里程计提供，别让两者发布同一条边。

### 7.3 动态 TF 发布示例（教学示例，未运行核对）

```python
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math

def publish_odom(node, br: TransformBroadcaster, x, y, yaw, stamp):
    t = TransformStamped()
    t.header.stamp = stamp                 # 这份位姿对应的时刻 (测量/估计时刻, 不是随手 now())
    t.header.frame_id = "odom"             # 父
    t.child_frame_id = "base_link"         # 子
    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.translation.z = 0.0
    t.transform.rotation.x = 0.0           # 四元数顺序 x, y, z, w
    t.transform.rotation.y = 0.0
    t.transform.rotation.z = math.sin(yaw / 2.0)
    t.transform.rotation.w = math.cos(yaw / 2.0)
    br.sendTransform(t)
```

**常见错误**：四元数没归一化（全 0 会被 TF 拒绝，提示无效四元数）；`w` 放错位置；平面机器人只给了 yaw 而 `z` 分量与 `w` 不满足 $z^2+w^2=1$。

### 7.4 里程计与定位的分工（P5 相关）

按 REP-105：

- `odom→base_link`：由轮速/IMU 融合的里程计发布，**连续、不跳变**，但长期会漂；
- `map→odom`：由定位（如 AMCL）发布，定位修正通过这条边吸收，所以 **`map→base_link` 可以出现离散跳变，但 `odom→base_link` 不应该**；
- 定位节点发布的是 `map→odom` 而不是 `map→base_link`，是因为 `base_link` 已经有 `odom` 这个父了（一个子只能有一个父）。它的做法是：用 AMCL 估计的 ${}^{\text{map}}T_{\text{base}}$ 与里程计的 ${}^{\text{odom}}T_{\text{base}}$ 算出 ${}^{\text{map}}T_{\text{odom}}={}^{\text{map}}T_{\text{base}}\,({}^{\text{odom}}T_{\text{base}})^{-1}$ 再发布。

**手算：** AMCL 估计车在 map 下 $(5,\ 2.2)$（平面，朝向 $0^\circ$ 简化）；里程计显示车在 odom 下 $(3,\ 1.2)$、朝向同样 $0^\circ$。则 ${}^{\text{map}}T_{\text{odom}}$ 的平移 $=(5-3,\ 2.2-1.2)=(2,\ 1)$，旋转 $0^\circ$——正好是 6.4 节里的数。当定位修正发生时，这个 $(2,1)$ 会变，而 `odom→base_link` 保持连续——这是"定位修正不让速度估计跳变"的设计意图。

---

## 8. 坐标系命名规范：REP-103 与 REP-105

REP（ROS Enhancement Proposal）是 ROS 社区的约定文档。以下为面试和实践里最常用的要点（具体细节以官方 REP 为准）。

### 8.1 REP-103：单位与坐标约定

- **单位**：国际单位——米、千克、秒、弧度、牛顿等。**不要**在消息里混用度和毫米。
- **右手坐标系**。
- **机体系**：$x$ 向前、$y$ 向左、$z$ 向上。
- **相机光学系**：$z$ 向前（光轴）、$x$ 向右、$y$ 向下；这类坐标系名习惯加后缀 `_optical`，与机体系的相机 link（`camera_link`）区分。
- **地理约定**：全局坐标常用 ENU（东—北—天）；某些传感器/飞控习惯 NED（北—东—地），需转换，名字上应区分。
- **旋转表示**：消息里四元数顺序 $(x,y,z,w)$；欧拉角 RPY 为绕固定轴的 roll、pitch、yaw（URDF 同）。

### 8.2 REP-105：移动机器人的标准坐标系

| 坐标系 | 含义 | 特性 |
|---|---|---|
| `base_link` | 刚性固连在机器人本体上 | 控制/规划输出的参考；位置由实现定（常取旋转中心或几何中心） |
| `odom` | 世界固定的局部坐标系 | `odom→base_link` **连续**（无跳变），但会漂移；适合短时精确运动 |
| `map` | 世界固定的全局坐标系 | 不漂，但 `map→base_link` **允许跳变**（定位修正）；适合长期位置 |
| `earth` | 多地图共享的全局参考（可选） | 用于多机器人/多地图 |

树结构：`map → odom → base_link →（各传感器和 link）`。

### 8.3 命名建议

1. 全小写 + 下划线：`base_link`、`laser_link`、`imu_link`、`camera_optical_frame`；
2. 多个同类设备用前缀：`left_arm_link3`、`front_laser`；多机器人用命名空间前缀：`robot1/base_link`（ROS2 中 `frame_id` 通常**不带**前导 `/`；ROS1 的 tf1 才用 `tf_prefix` 或前导斜杠的做法，两者不要混用）；
3. **名字就是接口**：下游靠字符串匹配，拼错一个字母，TF 树就断了，而且是静默的；
4. 一个物理量要有单一的 `frame_id`：IMU 消息的 `frame_id` 写 IMU 芯片坐标系，不要偷偷把数据旋到 `base_link` 却仍写 `imu_link`。

---

## 9. `frame_id` 与时间戳校验

> 对应 CV 里 P6 的 *standardised timestamps, coordinate frames and validity checks across multi-source perception and actuator feedback, giving the motion-control and task layers a consistent, trusted robot state.*

### 9.1 为什么要校验

每个传感器/驱动都可能出错：忘了填 `stamp`、`frame_id` 拼错、时钟没同步、数据卡住重复发、出现 NaN。**不校验就直接信任**，错误会被一路放大到控制输出。校验层把这些问题拦在入口：要么丢弃，要么降级，并留下可追溯的原因。

### 9.2 校验清单

对每一条带 header 的输入：

| 检查项 | 判定 | 常见根因 |
|---|---|---|
| `frame_id` 非空 | 为空则拒绝 | 驱动未配置 |
| `frame_id` 属于期望集合 | 不在则拒绝/告警 | 拼写、命名空间、多余前导斜杠 |
| 有 TF 路径到目标坐标系 | `canTransform` 为假则暂缓/拒绝 | TF 树断开、外参未发布 |
| `stamp` 非零 | 零表示驱动没填 | 驱动 bug |
| `stamp` 不太旧 | $\text{now}-\text{stamp} \le T_{\max}$ | 链路延迟、卡顿 |
| `stamp` 不在未来 | $\text{stamp}-\text{now} \le \epsilon$ | 时钟不同步 |
| `stamp` 单调 | 不小于上一条（或略大于） | 重复帧、乱序 |
| 数值有限 | 无 NaN/Inf | 传感器故障、除零 |
| 范围合理 | 在物理量程内 | 单位错（deg 当 rad、mm 当 m） |
| 协方差/置信度合理 | 非负、有限 | 估计器发散 |

### 9.3 示例代码（教学示例，已运行核对）

```python
import math
from dataclasses import dataclass

@dataclass
class Verdict:
    ok: bool
    reason: str = ""

def check_stamped(frame_id, stamp, now, expected_frame, last_stamp,
                  max_age=0.2, max_future=0.01, values=()):
    """对一条带 header 的测量做入口校验。时间单位: 秒(float)。
    max_age / max_future 为教学示例阈值, 实际要按传感器频率与控制周期定。"""
    if not frame_id:
        return Verdict(False, "empty frame_id")
    if frame_id.lstrip("/") != expected_frame:
        return Verdict(False, f"unexpected frame '{frame_id}', want '{expected_frame}'")
    if stamp <= 0.0:
        return Verdict(False, "stamp is zero (driver forgot to fill header)")
    age = now - stamp
    if age > max_age:
        return Verdict(False, f"stale: age {age:.3f}s > {max_age}s")
    if age < -max_future:
        return Verdict(False, f"from the future: {-age:.3f}s ahead")
    if last_stamp is not None and stamp <= last_stamp:
        return Verdict(False, "non-monotonic stamp (duplicate or out of order)")
    if any((not math.isfinite(v)) for v in values):
        return Verdict(False, "NaN or inf in payload")
    return Verdict(True)
```

用 $\text{now}=100.00$ s、期望坐标系 `imu_link`、上一条 `stamp=99.97` 的 8 组用例核对（作者运行）：

| 输入 | 结果 |
|---|---|
| `imu_link`, stamp 99.98 | 通过 |
| stamp 99.70（age 0.30 s） | 拒绝：stale |
| stamp 100.05 | 拒绝：from the future 0.05 s |
| `frame_id` 为空 | 拒绝：empty frame_id |
| `base_link`（期望 `imu_link`） | 拒绝：unexpected frame |
| stamp 0.0 | 拒绝：stamp is zero |
| stamp 99.97（等于上一条） | 拒绝：non-monotonic |
| 数据含 NaN | 拒绝：NaN or inf |

### 9.4 阈值怎么定

- `max_age`：应小于控制周期的若干倍，并考虑传感器自身频率。例如 IMU 200 Hz（周期 5 ms），若控制 100 Hz，则数据"最多比一个控制周期再老几个 IMU 周期"仍可接受；超出就应该标记"无效"，让下游降级（用预测值或进入保守模式），而不是默默使用。
- `max_future`：取时钟同步误差的上界；未同步时，任何阈值都没有意义——先做时间同步（NTP/PTP）。
- 每一类拒绝要有**计数器和日志**，便于事后追溯"为什么这一帧被丢"。

### 9.5 统一状态：把多源数据对齐到同一时刻

校验通过后，不同来源的数据时间戳仍不同（IMU 的 $t_1$、编码器的 $t_2$、视觉的 $t_3$）。典型做法：

1. 选一个**状态时间** $t_s$（通常是控制周期的"当前"时刻）；
2. 对每一路数据，用插值/预测对齐到 $t_s$（或者用最新的有效值并带上自己的"年龄"字段）；
3. 输出一个"**状态包**"：包含统一的时间戳、坐标系名、各分量的有效性标志和年龄。

这样，控制层和任务层拿到的是"一致、可信"的状态，而不是一堆各自带着不同时间、不同坐标系的原始消息。更详细的讨论见 [时间同步、坐标系一致性与数据有效性](../05_状态估计/06_时间同步_坐标系一致性与数据有效性.md)。

---

## 10. rviz 可视化

### 10.1 它是什么

rviz（ROS2 里是 `rviz2`）是 ROS 的 3D 可视化工具，把话题数据画出来。它不改变数据，只是"看"；但看对了才能排错。

### 10.2 常用显示项（Displays）

| 显示项 | 看什么 | 要注意 |
|---|---|---|
| **Fixed Frame**（全局选项） | 所有数据被变换到这个坐标系下显示 | 必须是 TF 树里真实存在的帧，常设 `map` 或 `odom`。不存在会显示 *Fixed Frame [xxx] does not exist* |
| TF | 每个坐标系的坐标轴与父子连线 | 可打开 *Show Names*；看树是否连通、是否抖动 |
| RobotModel | 按 URDF 画出机器人 | 需要 `robot_description` 参数与 `joint_states`；它订阅描述话题时要求匹配的持久性 QoS |
| LaserScan / PointCloud2 | 激光/点云 | 传感器类话题常是 BEST_EFFORT，显示项的可靠性选项要对得上，否则看不到数据 |
| Marker / MarkerArray | 自定义调试图形（目标点、轨迹、CoM） | `frame_id` 与 `stamp` 影响位置与是否显示 |
| Odometry / Path | 里程计、规划路径 | 看是否连续、是否与地图对齐 |
| Map | 栅格地图 | 地图话题通常 transient local，晚启动也能收到 |

### 10.3 用 rviz 排错的几个套路

1. **先看 TF**：打开 TF 显示，看各坐标轴是否在预期位置；树断了会有帧"漂在原点"；
2. **用 Fixed Frame 切换**：把 Fixed Frame 从 `map` 切到 `odom` 再到 `base_link`：在 `map` 下车会不会跳？在 `odom` 下是否连续？在 `base_link` 下传感器数据是否稳定不动（静止障碍物应当随车运动，而车体本身固定）？这能快速定位是定位跳变、里程计漂还是传感器外参错；
3. **看 Message Filter 警告**：控制台里 *Message Filter dropping message: frame 'xxx' at time ... for reason 'discarding message because the queue is full'* 或 *'the timestamp on the message is earlier than all the data in the transform cache'* 说明该消息的时间戳在 TF 缓存里查不到——多半是时间不同步或 TF 发布不及时；
4. **保存配置**：`rviz2 -d my.rviz` 载入已调好的显示配置，避免每次重配。

### 10.4 与 `use_sim_time` 的关系

回放 bag 或仿真时，rviz 也要用仿真时间，否则它按墙钟判断"数据太旧/在未来"而丢帧。启动参数加 `use_sim_time:=true`。（见 [ROS1 / ROS2 核心概念](./04_ROS1_ROS2核心概念.md) 第 7 章。）

---

## 11. 常见 TF 故障与排查

### 11.1 一览表

| 故障 | 现象 | 典型根因 | 排查 |
|---|---|---|---|
| **跳变** | rviz 里机器人/点云突然瞬移或来回闪 | ① 同一条边有两个发布者；② `map→odom` 修正过大；③ 里程计有突变 | `ros2 topic echo /tf` 看同一 `child_frame_id` 的来源（authority）；`ros2 run tf2_tools view_frames` 看有无多父 |
| **外推到未来** | *Lookup would require extrapolation into the future* | 查询时间晚于最新 TF；TF 发布延迟；节点时钟不同步；用 `now()` 查 | 等 TF（带 timeout）；用数据 `stamp` 查；检查时钟与 `use_sim_time` |
| **外推到过去** | *…extrapolation into the past* | 数据太旧超出缓存；TF 来自过时的 bag | 增大缓存；丢弃过旧数据；检查回放 |
| **树不连通** | *Could not find a connection between 'a' and 'b' because they are not part of the same tree* | 缺少某条边的发布者；名字拼错；URDF 与定位没衔接 | `view_frames` 看有几棵树；补发布静态 TF |
| **坐标系不存在** | *Frame [x] does not exist* | `frame_id` 拼写或前导斜杠问题；该节点没起 | 对照 TF 树；统一命名 |
| **循环 / 多父** | TF 树有环或帧被来回抢夺 | 两个节点各自发布 A→B 与 B→A，或同一子帧有两个不同父 | 保证单父单发布者；用 `view_frames` 检查 |
| **重复/过时数据** | *TF_REPEATED_DATA / TF_OLD_DATA* 警告 | 同一时间戳重复发布；时间回退（重启 bag、sim time 复位） | 发布频率与时间戳；重启节点或清缓存 |
| **无效四元数** | 变换被忽略并报 invalid quaternion | 四元数全零、未归一化、含 NaN | 发布前归一化；用有限数检查 |
| **静态 TF 迟到** | 晚启动的节点查不到外参 | 静态 TF 的 QoS 不是 transient local，或发布者晚于监听者且不重发 | 用标准 `static_transform_publisher`；检查 QoS |
| **时间基不一致** | 某些节点 TF 正常、有些全是外推 | 部分节点 `use_sim_time` 开着，部分没开 | 全部统一 |
| **频率太低** | 插值后的位姿有台阶感，高速时误差大 | TF 发布频率低于运动变化速度 | 提高发布频率或在上游预测 |

### 11.2 手算：频率与插值误差

示例（教学数值）：车速 $v=2\ \text{m/s}$，TF 以 10 Hz 发布（间隔 100 ms）。两个样本之间线性插值，假设真实运动是匀速直线，插值**无误差**；但若查询时间落在最新样本之后（外推），位置误差随外推时间线性增长：外推 $\Delta t=50\ \text{ms}$ 就会少算 $v\Delta t=0.1\ \text{m}$。而 TF 是**插值**而不是外推（超出范围直接报错）——这反而是个保护。对高速或转弯场景：真实轨迹是曲线，线性插值的弦高误差 $\approx \frac{1}{8}\kappa\,(v\,T)^2$（$\kappa$ 为曲率，$T$ 为采样间隔）。$\kappa=0.5\ \text{1/m}$（半径 2 m）、$v=2$、$T=0.1$：$\frac18\times0.5\times(0.2)^2=0.0025\ \text{m}=2.5\ \text{mm}$，可忽略；但如果 $T=0.5$ s，则 $\frac18\times0.5\times1^2=0.0625$ m。**频率不足的 TF 在转弯时才显出问题。**

### 11.3 系统化排查流程

1. **画出你**期望**的树**（哪些边、由谁发布、什么频率）；
2. `ros2 run tf2_tools view_frames`：看实际树是否一致（有几棵、有没有多余/缺失的边）；
3. `ros2 topic echo /tf --once` 与 `/tf_static`：看 `frame_id`、`child_frame_id` 的名字、来源；
4. `ros2 topic hz /tf`：频率是否正常；
5. 用 `tf2_echo`（查两帧的实时变换），先在已知位置的两帧上**验证方向**；
6. 检查时间：`use_sim_time` 是否一致；时间戳是否零；多机时钟；
7. 在 rviz 里切 Fixed Frame 做定位（见 10.3）；
8. 若是跳变，临时只保留一个来源，其余关闭，二分定位。

---

## 12. 在本项目里怎么用

### 12.1 P5：车式移动机器人自主导航

[P5 项目文档](../../01_项目课程/P5_车式移动机器人自主导航/README.md)。CV：*fused IMU, wheel encoders and 2D LiDAR for map-based localisation, handled drift and pose jumps from inconsistent timing, frames and motion direction, and output a continuous vehicle state.*

对照本章：

- 7.4 节：`map→odom→base_link` 的分工；**输出连续的车辆状态**对应 `odom→base_link` 不跳变；
- 6.3 节：用采样时刻查 TF，避免点云拖影；
- 11 章：跳变、外推是"inconsistent timing, frames"的典型表现；
- "motion direction"：轮速符号、倒车时的方向约定，与 REP-103 的 $x$ 向前、车头方向一致；
- 本项目的做法：`odom→base_link` 由 EKF 唯一发布，`map→odom` 由 AMCL 唯一发布，`base_link→laser_link/imu_link` 是静态变换；`base_link` 取在后轴中心。激光/IMU 外参先用卷尺和 CAD 量出初值，再用激光对墙面直线的拟合残差微调。遇到的典型问题是转弯和倒车时 `map→odom` 出现几十厘米、5°–10° 的阶跃和 `extrapolation into the future` 报错：定位方法是切换 Fixed Frame 找出跳变在哪一层，再对时间戳（激光时间戳从"到达时刻"改为扫描起始并补偿约 60 ms）、方向符号（轮速带符号）和静态变换三类不一致逐一统一。

### 12.2 P6：M-Robots 平台的状态管理

[P6 项目文档](../../01_项目课程/P6_深开鸿M-Robots平台软件/README.md)。CV：*standardised timestamps, coordinate frames and validity checks across multi-source perception and actuator feedback.*

对照本章：9 章整章。面试回答的骨架：**多源输入 → 入口校验（frame、时间戳、有限值、年龄）→ 统一时间基与坐标系 → 输出带有效性标志的一致状态**。

- 本项目的做法：时间戳用设备内部的单调时钟（纳秒），跨设备用 NTP 对齐到局域网时间服务器（量级几毫秒到十几毫秒），控制回路不依赖跨设备时间；坐标系遵循 REP-103（x 前、y 左、z 上），命名为 `map`、`odom`、`base_link`、`laser_link`、`imu_link`，外参集中放在设备描述文件里在入口一次性应用；校验按有限值、量程、frame 已声明、时间戳单调、新鲜度（传感器周期的 3 倍）逐项检查，不合格的按 Invalid 丢弃，过期的降为 Stale，时间异常的标 Suspect；有效性标志是状态包里的 validity 字段（Valid / Stale / Suspect / Invalid 四级）。

### 12.3 P2：双臂 URDF 与主从映射

[P2 项目文档](../../01_项目课程/P2_未来动力双臂遥操作与差速底盘/README.md)。CV：*built dual-arm dynamics from URDF/KDL…leader-arm gravity compensation and follower position–velocity tracking.*

对照本章：

- 2–5 章：URDF 质量/质心/惯量与关节顺序，决定 KDL 重力补偿准不准；
- 4 章：关节零位、轴方向、电机方向 $s$、偏置 $\theta_0$——Leader 与 Follower 之间的关节映射也要考虑；
- 5.4 节：左右臂镜像时轴和限位的符号；
- VR 遥操作：手柄坐标系 → 机器人基座坐标系的建立（相对位姿的参考系选取），见 [坐标系、旋转矩阵、齐次变换](../03_坐标变换与运动学/01_坐标系_旋转矩阵_齐次变换.md)；
- 本项目的做法：Leader/Follower 的关节映射写成逐关节的"方向 $s=\pm1$、偏置 $\theta_0$"两项（电机内部已含减速，关节侧不再另乘减速比）；零位标定是把两臂摆到同一个已知姿态（机械零位或启动姿态），读取电机角度并置零后记下偏置；URDF 以厂商提供的模型为基础，再自己整理质量、质心和关节轴方向。

### 12.4 P1：URDF 与质心

[P1 项目文档](../../01_项目课程/P1_AIRS球形机器人平衡控制/README.md)。CV：*estimated CoM offset from arm joint states with URDF/Pinocchio.* 本章 2、3、4 章是该估计能否可信的前提：质量、质心、关节零位和轴方向都要对。本项目里 URDF 由 CAD 导出，整机质量经称重核对到约 5% 以内，并用 MuJoCo 的 `ballbotgym` 环境检查符号与质心方向。

---

## 13. 面试怎么问、怎么答

**Q1. What does `lookupTransform(target, source, time)` return, and what is special about the time argument?**

> It returns the transform that takes data from the source frame into the target frame, so the pose of the source frame expressed in the target frame. The time argument matters: a specific time interpolates between buffered samples, while time zero means the latest available, which hides latency. If the time is newer than the latest sample, TF reports an extrapolation error. For sensor data I query with the message's own timestamp, not with the current time.

中文：返回把 source 系的数据变到 target 系的变换（source 在 target 里的位姿）；指定时间则在缓存样本间插值，零时间表示最新可用（会掩盖延迟）；晚于最新样本就报外推错误；处理传感器数据要用消息自带时间戳而不是当前时间。
追问：为什么不用 `now()`？（车在动，会把"现在"的位姿用在"过去"的测量上，出现拖影。）

**Q2. What is the difference between the `map`, `odom` and `base_link` frames?**

> `base_link` is attached to the robot body. `odom` is a world-fixed local frame where the pose of `base_link` is continuous but drifts over time. `map` is a global frame that does not drift, but the pose in it can jump when the localization corrects. The localization node therefore publishes `map` to `odom`, not `map` to `base_link`, so the odometry part stays continuous.

中文：`base_link` 固连本体；`odom` 世界固定的局部系，位姿连续但会漂；`map` 全局不漂，但定位修正时位姿可跳；定位节点发布 `map→odom`，从而 `odom→base_link` 保持连续。
追问：为什么控制器通常用 `odom` 系下的状态？（连续，没有跳变，利于微分/速度估计。）

**Q3. You see the robot jumping in rviz. How do you debug it?**

> I check the TF tree first: whether one child frame has two publishers, using the frame view tool and echoing the TF topic to see the authority. Then I switch the fixed frame between `map`, `odom` and `base_link` to see where the jump appears. If the jump only appears in `map`, it is the localization correction. If it appears in `odom`, the odometry has a discontinuity, for example from the wheel direction or a time reset. I also check timestamps and whether all nodes use the same time source.

中文：先查 TF 树，看同一子帧是否有两个发布者；再切换 Fixed Frame 看跳变出现在哪一层——只在 `map` 是定位修正，在 `odom` 是里程计不连续（轮速方向、时间复位等）；同时查时间戳和时间源是否一致。

**Q4. How do you make sure data from different sensors are consistent in time and frames?**（对应 P6）

> At the input of the state layer I validate each message: the frame ID must be known, the timestamp must be non-zero, not too old, not in the future, and monotonic, and the values must be finite. Messages that fail are dropped or marked invalid with a reason. Then I align the sources to a common state time and a common frame, and publish one state with validity flags. This way the control and task layers get one consistent, trusted state instead of raw messages with different times and frames.

中文：状态层入口逐条校验：frame 已知、时间戳非零/不过旧/不在未来/单调、数值有限；不合格则丢弃或带原因标记无效；再把各源对齐到统一状态时间和坐标系，输出带有效性标志的一致状态，供控制与任务层使用。
本项目的取值（设计级，量级）：新鲜度上限取传感器周期的 3 倍（IMU 100 Hz 对应 30 ms，里程计 50 Hz 对应 60 ms，激光 10 Hz 对应 300 ms）；轮速量程 ±3 m/s；连续 5 帧完全相同判为冻结。被追问时可以说："The thresholds were based on the sensor rates and the control period." 

**Q5. In URDF, how do you define the zero position and the axis of a joint, and what goes wrong if they do not match the real robot?**

> The joint origin gives the child frame relative to the parent when the joint angle is zero, and the axis gives the rotation direction in the child frame. If the zero offset or direction does not match the real motor, the gravity compensation is wrong or has the wrong sign, and the visualization moves the wrong way. I calibrate the zero at a mechanical reference pose and keep an explicit direction sign and offset in the mapping from encoder to joint angle.

中文：origin 给出关节角为 0 时子系相对父系的位姿，axis 给出在子系下的转向；零位或方向与实物不一致，重力补偿会错或符号反，可视化方向也反；我在机械参考位姿上标零位，并在编码器到关节角的映射里显式保存方向系数和偏置。

**Q6. Why use xacro instead of plain URDF?**

> URDF has no variables or functions, so a two-arm robot means a lot of copy and paste. Xacro adds properties, macros and includes, so the left and right arm share one macro and parameters like prefix and mirror sign. It expands to plain URDF, which I still check with a parser before using it.

中文：URDF 没有变量和函数，双臂会大量复制；xacro 提供属性、宏、包含，左右臂共享一个宏，用前缀和镜像符号区分；展开成纯 URDF 后仍要用解析器检查。

---

## 14. 自测题

1. 用 ${}^{a}T_{b}$ 记号写出：把 laser 系的点变到 map 系的公式，并说明链式顺序。
   *要点：${}^{map}p={}^{map}T_{odom}\,{}^{odom}T_{base}\,{}^{base}T_{laser}\,{}^{laser}p$，相邻下标相消，点从右往左变。*
2. 手算：${}^{map}T_{odom}$ 平移 $(1,0)$ 旋转 $0$；${}^{odom}T_{base}$ 平移 $(0,2)$ 旋转 $90^\circ$；点 ${}^{base}p=(1,0)$，求 ${}^{map}p$。
   *要点：base→odom：$R(90^\circ)(1,0)=(0,1)$，加 $(0,2)$ 得 $(0,3)$；加 $(1,0)$ 得 $(1,3)$。*
3. URDF 里 `rpy="-1.5708 0 -1.5708"` 的光学坐标系变换后，光学系 $z$ 轴指向机体系哪个方向？
   *要点：$+x$（前）；光学 $x$ 为机体右，$y$ 为机体下。*
4. TF 缓存有 $t=5.00$ s（$x=2.0$）与 $t=5.10$ s（$x=2.4$）两个样本。查询 $t=5.04$ s 的 $x$ 是多少？查询 $t=5.12$ s 会怎样？
   *要点：$\alpha=0.4$，$x=2.0+0.4\times0.4=2.16$；5.12 晚于最新样本，外推到未来报错。*
5. 为什么定位节点发布 `map→odom` 而不是 `map→base_link`？
   *要点：`base_link` 已有父 `odom`（单父约束）；这样 `odom→base_link` 保持连续，修正被吸收到 `map→odom`。*
6. 列出入口校验至少 5 项，并说明时钟未同步时 `max_future` 阈值为什么没意义。
   *要点：frame 已知、stamp 非零/不旧/不在未来/单调、数值有限、量程、TF 可达；未同步时时间差本身不可信。*
7. 一个关节电机端读数 $\theta_{motor}=18.0$ rad，$N=10$，$\theta_0=2.10$ rad，$s=-1$，求 $q_{urdf}$。
   *要点：$\theta_m=1.8$，$q=-(1.8-2.1)=+0.3$ rad。*
8. 双臂镜像时，哪些量需要取反？只镜像 `origin` 的 y 够吗？
   *要点：不够，还要处理转轴方向和限位范围，保证左右"正方向"定义一致。*

---

## 15. 本章要点速记与相关链接

**要点速记**

1. 记号 ${}^{a}T_{b}$：$b$ 在 $a$ 中的位姿；点从右往左变；TF 边 = 父→子 = 子在父中的位姿。
2. URDF 是树；`origin` = $q=0$ 时子在父中的位姿；`axis` 在子系下；`rpy` 弧度、$R=R_zR_yR_x$。
3. `inertial` 填质心和绕质心的惯量，质量为正、惯量正定；质心不是几何中心。
4. 光学系：$z$ 前、$x$ 右、$y$ 下；机体系：$x$ 前、$y$ 左、$z$ 上。
5. 编码器到 URDF：$q=s(\theta_m-\theta_0)$，方向与零位错了，补偿就错或反。
6. xacro = 属性 + 宏 + 参数 + 包含，展开后仍要 `check_urdf`。
7. `lookupTransform(target, source, t)` 返回 source→target；时间零 = 最新，会掩盖延迟；传感器数据用自己的 `stamp` 查。
8. REP-105：`map→odom→base_link`；`odom` 连续会漂，`map` 不漂可跳；定位发布 `map→odom`。
9. 入口校验：frame、stamp（非零/不旧/不未来/单调）、有限值、可达；统一时间基后输出带有效性的状态。
10. TF 故障先看树：多父、断树、外推、时间基不一致；用 Fixed Frame 切换定位跳变层。

**相关链接**

- 上一章：[MuJoCo / Mink / Pinocchio / KDL 工具链](./06_MuJoCo_Pinocchio_KDL_工具链.md)
- 前一组：[ROS1 / ROS2 核心概念](./04_ROS1_ROS2核心概念.md)，[DDS 与分布式通信](./05_DDS与分布式通信.md)
- 基础：[坐标系、旋转矩阵、齐次变换](../03_坐标变换与运动学/01_坐标系_旋转矩阵_齐次变换.md) · [欧拉角、四元数、姿态表示](../03_坐标变换与运动学/02_欧拉角_四元数_姿态表示.md) · [闭链并联机构与五连杆运动学](../03_坐标变换与运动学/05_闭链并联机构与五连杆运动学.md) · [机械臂动力学、重力补偿、URDF、KDL](../03_坐标变换与运动学/06_机械臂动力学_重力补偿_URDF_KDL.md) · [里程计与传感器融合](../05_状态估计/04_里程计与传感器融合.md) · [时间同步、坐标系一致性与数据有效性](../05_状态估计/06_时间同步_坐标系一致性与数据有效性.md) · [导航系统架构：move_base 与 Nav2](../06_规划与导航/05_导航系统架构_move_base与Nav2.md)
- 八股：[ROS 与机器人软件八股](../../03_八股知识/03_机器人学/05_ROS与机器人软件八股.md)
- 项目：[P1](../../01_项目课程/P1_AIRS球形机器人平衡控制/README.md) · [P2](../../01_项目课程/P2_未来动力双臂遥操作与差速底盘/README.md) · [P5](../../01_项目课程/P5_车式移动机器人自主导航/README.md) · [P6](../../01_项目课程/P6_深开鸿M-Robots平台软件/README.md)
