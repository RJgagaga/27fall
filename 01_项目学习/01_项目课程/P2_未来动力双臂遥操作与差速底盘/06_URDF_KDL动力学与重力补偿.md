# 06 URDF/KDL 动力学与重力补偿

> **一句话定位**：主从示教的第一半——"主臂为什么能被人轻轻一推就动"。答案是：电机不去追任何位置，而是**用模型算出"此刻手臂自重在每个关节上造成的力矩"，并反向顶回去**。这一章从拉格朗日方程讲起，推出 $M\ddot q+C\dot q+g=\tau$，手算二连杆，再把 URDF 的质量/质心怎么变成 KDL 的 `JntToGravity`，最后逐段对照仓库里真实的 `gravity_comp.py`。

**本章学完你能：**

- 默写动力学方程 $M(q)\ddot q+C(q,\dot q)\dot q+g(q)=\tau$，并说出每一项的物理含义和单位；
- 用势能对关节角求偏导，推出二连杆的重力项 $g_1,g_2$，并手算数值；
- 解释为什么静止时 $\tau\approx g(q)$、为什么竖直时重力矩≈0 但重量仍在；
- 讲清 URDF 里哪些字段决定重力补偿是否算对（质量、质心、关节轴、零位），KDL/Pinocchio 如何从 URDF 得到 $g(q)$；
- 逐段读懂 `gravity_comp.py`：模型简化、MIT 帧里 `kp=0, kd=0, tau=补偿`、`GRAVITY_SCALE`、科氏与摩擦的可选补偿；
- 解释主臂"低阻力"的物理来源与代价，以及补偿比例 `gravity_comp_ratio` 的含义；
- 说出方向/符号/零位/关节顺序四类典型错误，并给出验证步骤。

**前置知识：**

- 拉格朗日方程与机器人动力学：[02_拉格朗日方程与机器人动力学](../../02_基础知识/02_力学与建模/02_拉格朗日方程与机器人动力学.md)
- 重力补偿 / URDF / KDL（基础版）：[06_机械臂动力学_重力补偿_URDF_KDL](../../02_基础知识/03_坐标变换与运动学/06_机械臂动力学_重力补偿_URDF_KDL.md)
- 电机与力矩控制：[05_电机_编码器_驱动与力矩控制](../../02_基础知识/08_嵌入式与通信/05_电机_编码器_驱动与力矩控制.md)
- 与 P1 的联系（URDF + Pinocchio 做重力前馈）：[P1 README](../P1_AIRS球形机器人平衡控制/README.md)

## 目录

1. 要解决的问题：为什么需要重力补偿
2. 动力学方程：每一项是什么
3. 二连杆：用拉格朗日推出 $M,C,g$
4. 重力项的手算与"力臂"直觉
5. 为什么只补重力就够用（以及什么时候不够）
6. URDF：重力补偿的"原材料"
7. KDL：从 URDF 到 `JntToGravity`
8. 仓库 `gravity_comp.py` 逐段对照
9. 用 URDF 离线估算 xArm 的重力矩（教学推算）
10. 主臂为什么"低阻力"，以及残余阻力来自哪里
11. 符号、零位、关节顺序：四类典型错误与验证
12. 常见故障与定位
13. 面试怎么问、怎么答
14. 本章要点速记 / 自测题 / 相关链接

---

## 1. 要解决的问题：为什么需要重力补偿

示教要求：人拖动主臂（Leader），从臂（Follower）同步复现。如果主臂**什么力矩都不出**，那么

- 一松手，它在重力作用下立刻砸下来；
- 人要一直"托着"它的全部自重，既累又难以做精细动作。

如果主臂**用位置控制把自己锁住**（高 $k_p$），人推它就像推一根很硬的弹簧，阻力大，示教体验差。

理想的主臂应该是：**不推它时，它停在原处；推它时，它几乎不费力地跟着走**。这就是"重力补偿 + 零刚度"：电机输出的力矩恰好等于抵消自重所需的力矩，不再有任何位置回复力。人只需要克服摩擦和惯性。

CV 对应原句：*designed leader-arm gravity compensation ... enabling low-resistance hand-guided teaching*。

---

## 2. 动力学方程：每一项是什么

$n$ 自由度机械臂的动力学（刚体、关节力矩驱动）：

$$
M(q)\,\ddot q+C(q,\dot q)\,\dot q+g(q)=\tau .
$$

| 项 | 名称 | 含义 | 单位 |
|---|---|---|---|
| $q,\dot q,\ddot q$ | 关节位置、速度、加速度 | 向量，$n$ 维 | rad, rad/s, rad/s² |
| $M(q)$ | 质量（惯量）矩阵 | 对称正定，$n\times n$；表示"让各关节加速需要多少力矩"，随姿态变化 | kg·m² |
| $C(q,\dot q)\dot q$ | 科氏力 + 离心力 | 与速度的**二次**相关：$\dot q_i\dot q_j$ 乘积项 | N·m |
| $g(q)$ | 重力力矩 | 当前姿态下，重力在各关节上造成的力矩；与速度加速度无关 | N·m |
| $\tau$ | 关节驱动力矩 | 电机（经减速器）输出 | N·m |

（有时还会加摩擦项 $F(\dot q)$ 和外力项 $J^{\mathsf T}f_{ext}$。）

**静止时**（$\dot q=0,\ddot q=0$）：

$$
\boxed{\ \tau=g(q)\ }
$$

意思是：**即使手臂不动，电机也要持续输出 $g(q)$ 才能不下垂。** 重力补偿就是实时计算 $g(q)$ 并作为前馈力矩下发。

---

## 3. 二连杆：用拉格朗日推出 $M,C,g$

平面二连杆（竖直平面，$y$ 向上），关节角 $q_1$（相对水平）、$q_2$（相对第一杆），杆长 $l_1,l_2$，质量 $m_1,m_2$，质心在杆中点，绕质心转动惯量 $I_1=\frac{1}{12}m_1l_1^2$，$I_2=\frac1{12}m_2l_2^2$。

### 3.1 质心位置

$$
\begin{aligned}
x_{c1}&=\tfrac{l_1}{2}\cos q_1, & y_{c1}&=\tfrac{l_1}{2}\sin q_1,\\
x_{c2}&=l_1\cos q_1+\tfrac{l_2}{2}\cos(q_1+q_2), & y_{c2}&=l_1\sin q_1+\tfrac{l_2}{2}\sin(q_1+q_2).
\end{aligned}
$$

### 3.2 势能与重力项

$$
U=g\,(m_1y_{c1}+m_2y_{c2}).
$$

广义重力就是势能对广义坐标的偏导 $g_i=\partial U/\partial q_i$：

$$
g_1=g\Big[\big(\tfrac{m_1l_1}{2}+m_2l_1\big)\cos q_1+\tfrac{m_2l_2}{2}\cos(q_1+q_2)\Big],\qquad
g_2=g\,\tfrac{m_2l_2}{2}\cos(q_1+q_2).
$$

（这里的 $g$ 既表示重力加速度，也出现在 $g_i$ 的名字里，读者不要混；下文把加速度写成 $9.81$ 或 $g_0$ 时会注明。）

这和学习笔记里的公式一致。可以用符号计算验证：对 $U$ 求偏导得到的正是上式。

### 3.3 动能与质量矩阵（供追问）

$$
M(q)=\begin{bmatrix}M_{11}&M_{12}\\M_{12}&M_{22}\end{bmatrix},
$$

$$
\begin{aligned}
M_{11}&=I_1+I_2+m_1\tfrac{l_1^2}{4}+m_2\Big(l_1^2+\tfrac{l_2^2}{4}+l_1l_2\cos q_2\Big),\\
M_{12}&=I_2+m_2\Big(\tfrac{l_2^2}{4}+\tfrac{l_1l_2}{2}\cos q_2\Big),\\
M_{22}&=I_2+m_2\tfrac{l_2^2}{4}.
\end{aligned}
$$

科氏/离心项：令 $h=-m_2l_1\tfrac{l_2}{2}\sin q_2$，

$$
C(q,\dot q)\dot q=\begin{bmatrix}h\,(2\dot q_1\dot q_2+\dot q_2^2)\\-h\,\dot q_1^2\end{bmatrix}.
$$

### 3.4 数字（示例：$m_1=m_2=1$ kg，$l_1=l_2=1$ m；已用程序验证）

- $M(q_2=0)=\begin{bmatrix}2.667&0.833\\0.833&0.333\end{bmatrix}$，$M(q_2=90^\circ)=\begin{bmatrix}1.667&0.333\\0.333&0.333\end{bmatrix}$。**伸直时惯量更大**（质量离转轴更远），所以同样的加速度，伸直时需要更大的力矩。
- $q_2=90^\circ$，$\dot q=(2,1)$ rad/s：$h=-0.5$，$C\dot q=(h(2\cdot2\cdot1+1),\ -h\cdot4)=(-2.5,\ 2.0)$ N·m。

---

## 4. 重力项的手算与"力臂"直觉

### 4.1 一根杆

1 kg 的杆，质心距关节 0.5 m，水平伸出。重力 $mg_0=9.81$ N 作用在质心，**力臂**是质心到转轴的**水平**距离 0.5 m：

$$
\tau=m g_0\,d=1\times9.81\times0.5=4.905\ \text{N·m}.
$$

杆竖直时，质心在转轴正上方/下方，水平力臂≈0，$\tau\approx0$。**重量没有消失，只是力臂为零**——这就是公式里 $\cos$ 的来历：$\cos q$ = 力臂与杆长之比。

### 4.2 二连杆（取 $m=1,l=1$，$g_0=9.81$）

| $(q_1,q_2)$ | $g_1$（N·m） | $g_2$（N·m） | 读法 |
|---|---|---|---|
| $(0^\circ,0^\circ)$（两杆水平伸直） | 19.62 | 4.905 | 最大：关节 1 要托两根杆，$9.81\times(0.5+1+0.5)$ |
| $(30^\circ,45^\circ)$ | 14.01 | 1.27 | 倾斜后力臂缩短 |
| $(90^\circ,0^\circ)$（竖直向上） | ≈0 | ≈0 | 力臂为零 |
| $(0^\circ,90^\circ)$（第一杆水平，第二杆竖直） | 14.715 | ≈0 | 关节 2 竖直无力臂；关节 1 只托第二杆在 $l_1$ 处的重量 + 第一杆 |

核对 $(30^\circ,45^\circ)$：$g_1=4.905\,[\cos30^\circ+2\cos30^\circ+\cos75^\circ]=4.905\times(0.866+1.732+0.2588)=4.905\times2.857=14.01$ ✓；$g_2=4.905\cos75^\circ=1.27$ ✓。

---

## 5. 为什么只补重力就够用（以及什么时候不够）

示教时人手慢速拖动，速度和加速度都小。方程里：

- 惯性项 $M\ddot q$：与加速度成正比；
- 科氏项 $C\dot q$：与**速度的平方**成正比；
- 重力项 $g(q)$：与速度无关，**始终存在**，往往是静态的主导项。

对上面的二连杆，$q_2=90^\circ$，$\dot q=(2,1)$ rad/s（相当快）时科氏项只有 $(−2.5,\ 2.0)$ N·m，而重力项在同样姿态下 $g_1=14.7$ N·m。若速度降到一半（1, 0.5 rad/s），科氏项降到 1/4，约 $(−0.6,\ 0.5)$ N·m。

所以：

- **主臂**用重力补偿（可选少量科氏和摩擦补偿），人只需克服剩余的惯性和摩擦；
- 如果要求高速或精确的**轨迹跟踪**（例如从臂追主臂），才需要完整的逆动力学 $\tau=M\ddot q_d+C\dot q_d+g$；
- 完整逆动力学需要精确的惯量参数，模型误差更大，实际中多数示教/遥操作只做重力 + 摩擦。

> **注意**：重力补偿 ≠ 完整动力学补偿。学习笔记里反复提醒：函数里存在 `ComputeFriction` / Coriolis 并不等于它们在活动路径上被使用。面试时只说"我们补重力（以及按比例可选地补摩擦/科氏）"，别说"完整逆动力学前馈"。

---

## 6. URDF：重力补偿的"原材料"

URDF（Unified Robot Description Format）是一个 XML 文件，描述机器人的运动学树和惯性参数。与重力补偿直接相关的字段：

```xml
<link name="xarm_right_link3">
  <inertial>
    <origin xyz="-0.00140009 -0.00018167 0.08957722" rpy="0 0 0"/>   <!-- 质心在本 link 坐标系中的位置 -->
    <mass value="0.98186876"/>                                       <!-- 质量 kg -->
    <inertia ixx="0.004372" .../>                                    <!-- 绕质心的惯量张量 -->
  </inertial>
</link>
<joint name="xarm_right_joint3" type="revolute">
  <origin xyz="0.0301 0.0 0.068" rpy="0 0 -0.0339"/>                 <!-- 子 link 相对父 link 的固定变换 -->
  <parent link="xarm_right_link2"/>
  <child  link="xarm_right_link3"/>
  <axis xyz="0 0 1"/>                                                <!-- 转轴方向（在 joint 坐标系中）-->
  <limit effort="6.8" lower="-1.5708" upper="1.5708" velocity="14"/>
</joint>
```

以上是仓库 `xarm_bimanual.urdf` 里该关节的结构（数值取自文件）。

决定 $g(q)$ 是否算对的**四个要素**：

1. **质量**与**质心位置**：直接决定重力大小与力臂。错 10%，该连杆带来的重力矩就错 10%；
2. **关节轴方向与原点**：决定每个关节对每个质心的力臂向量；
3. **关节零位**：URDF 的 $q=0$ 必须与实际电机编码器的零点对应同一个物理姿态；
4. **重力方向**：基座坐标系下重力向量，通常 $(0,0,-9.81)$。若基座不水平（例如装在倾斜的底盘上），重力方向要换算。

惯量张量对**静态重力**没有影响，只影响 $M$ 和科氏项。

> **提示**：这也解释了为什么"URDF 来自 CAD 导出"时，要特别核对**线缆、减速器、夹爪、手柄**这些 CAD 中没建模的质量——它们会造成系统性的残差。

---

## 7. KDL：从 URDF 到 `JntToGravity`

**KDL**（Kinematics and Dynamics Library，Orocos）是一个 C++ 运动学/动力学库，ROS 里很常用。它的概念链：

```text
URDF 文件 ──(kdl_parser)──► KDL::Tree ──(getChain: 根→末端)──► KDL::Chain
                                                                   │
                      KDL::ChainDynParam dyn(chain, Vector(0,0,-9.81))
                                                                   │
        JntToGravity(q, g_out)   JntToCoriolis(q, qd, c_out)   JntToMass(q, M_out)
```

学习笔记里你们项目的主从代码就是这个流程：

```text
Dynamics.Init:  解析 URDF → KDL tree → 找 start/end 链 → ChainDynParam(chain, gravity(0,0,-9.81))
GetGravity:     逐关节位置 q → JntToGravity → 返回 g(q)
```

- `JntToGravity` 返回的就是 $g(q)$——"抵消重力所需的关节力矩"（笔记里强调：它已经带好方向，理论上电机直接发这个值即可，实际是否反要看映射层，第 11 节）。
- 算法实质：**递归牛顿–欧拉（RNE）** 在 $\dot q=\ddot q=0$ 时，把重力等效为底座向上的加速度 $+9.81$ m/s²，然后从末端向基座递推各关节的力矩。复杂度 $O(n)$，对 7 自由度臂耗时在微秒量级（数量级说明，不是项目实测）。
- 仓库 Python 参考实现用的是 **Pinocchio**：`pin.computeGeneralizedGravity(model, data, q)` 或 `pin.rnea(model, data, q, 0, 0)`，数学上与 KDL 的 `JntToGravity` 等价。对照：

| 概念 | KDL | Pinocchio |
|---|---|---|
| 读 URDF | `kdl_parser` → `Tree` → `Chain` | `buildModelFromUrdf` |
| 固定其他关节（只留一条臂） | 取链 | `buildReducedModel(model, joints_to_lock, q_ref)` |
| 重力项 | `ChainDynParam::JntToGravity` | `computeGeneralizedGravity` / `rnea(q,0,0)` |
| 科氏项 | `JntToCoriolis` | `computeCoriolisMatrix(...) @ dq` |
| 质量矩阵 | `JntToMass` | `crba` |

> **口径（给你的提醒）**：CV 写的是 URDF/KDL；仓库里这份 Python 参考代码用的是 Pinocchio。面试时按 CV 讲 "URDF → KDL → gravity vector"，若面试官问到仓库代码，可以说"another implementation of the same computation uses Pinocchio; the maths is identical"。

---

## 8. 仓库 `gravity_comp.py` 逐段对照

文件头注释写明：*CAN 通信：XArmController；动力学计算：Pinocchio；补偿内容：重力 + 科氏力（可调增益）+ 摩擦（可调增益）*。下面是关键片段。以下数值是**这份参考脚本里的配置**，不是你项目的实测参数。

### 8.1 模型简化：只留一条臂

```python
class ArmDynamics:
    def __init__(self, urdf_path, arm_prefix):
        full_model = pin.buildModelFromUrdf(urdf_path)
        arm_joint_names = {f"{arm_prefix}joint{i}" for i in range(1, 8)}
        joints_to_lock = [jid for jid in range(1, full_model.njoints)
                          if full_model.names[jid] not in arm_joint_names]
        q_ref  = pin.neutral(full_model)
        self.model = pin.buildReducedModel(full_model, joints_to_lock, q_ref)
        ...
        if self.model.nv != 7: raise RuntimeError(...)
```

意思：双臂 URDF 里把**另一条臂和夹爪手指等关节锁在中立位置**，只保留所选手臂的 7 个关节，得到一个 7 自由度的简化模型（并检查 `nv==7` 作为"关节名/前缀写对了"的校验）。被锁定的关节上的质量会被**合并到相连的连杆**里，所以手和手指的重量仍然算在内。

### 8.2 计算重力与科氏

```python
def get_gravity(self, q):
    return pin.computeGeneralizedGravity(self.model, self.data, q).copy()

def get_coriolis(self, q, dq):
    C = pin.computeCoriolisMatrix(self.model, self.data, q, dq)
    return (C @ dq).copy()
```

### 8.3 摩擦模型

```python
def compute_friction(vel, Fc, k, Fv, Fo):
    return Fc * math.tanh(0.1 * k * vel) + Fv * vel + Fo
```

- $F_c\tanh(0.1k\,v)$：**平滑的库仑摩擦**。真实库仑摩擦是 $F_c\,\mathrm{sign}(v)$，在 $v=0$ 处不连续，用在控制里会引起高频抖动；用 $\tanh$ 把它在零速附近磨平；
- $F_vv$：黏性摩擦，与速度成正比；
- $F_o$：常值偏置。

脚本里的参数（每个关节一组）：`_Fc = [0.5, 0.5, 0.3, 0.3, 0.2, 0.2, 0.2]`，`_k = 10`，`_Fv = [0.1, 0.1, 0.1, 0.1, 0.05, 0.05, 0.05]`，`_Fo = 0`。**手算**关节 1，$v=1$ rad/s：

$$
F=0.5\cdot\tanh(0.1\cdot10\cdot1)+0.1\cdot1+0=0.5\times0.7616+0.1=0.481\ \text{N·m}.
$$

再乘总增益 `FRICTION_GAIN = 0.3`，实际补偿 $0.144$ N·m。（即只补偿估计摩擦的 30%，留有余量，避免过补偿导致"自己往前溜"。）

### 8.4 主循环：真正下发的力矩

```python
GRAVITY_SCALE  = 1.0
FRICTION_GAIN  = 0.3
CORIOLIS_GAIN  = 0.1

q  = np.array(ctrl.get_positions(CHANNEL))
dq = np.array(ctrl.get_velocities(CHANNEL))
g_tau   = dyn.get_gravity(q)
cor_tau = dyn.get_coriolis(q, dq)
for i in range(7):
    fric  = compute_friction(dq[i], _Fc[i], _k[i], _Fv[i], _Fo[i])
    total = g_tau[i]*GRAVITY_SCALE + cor_tau[i]*CORIOLIS_GAIN + fric*FRICTION_GAIN
    params.append(MITParam(kp=0.0, kd=KD[i], q=0.0, dq=0.0, tau=total))
ctrl.mit_control_all(CHANNEL, params)
ctrl.recv_all(channel=CHANNEL)      # 等 7 个电机的回包
```

读法：

1. 读的是**反馈**（`get_positions/velocities` 来自电机回包）——重力计算用的是**实际当前 $q$**，不是指令（笔记里你专门确认过：指令的 `q=0, dq=0` 只是占位，kp=0 时它们不起作用）；
2. 力矩 = 1.0 × 重力 + 0.1 × 科氏 + 0.3 × 摩擦；
3. MIT 帧里 `kp=0`：**没有位置回复力**；`kd` 很小（左臂 0，右臂 `RIGHT_KD = [0.5, 0.5, 0.1, 0.1, 0.1, 0.2, 0.2]`）：只提供一点速度阻尼来稳定系统；`tau` = 上面的总和；
4. `recv_all` 阻塞等待 7 个电机回包，一次循环的速度由 CAN 往返决定。

> 脚本中有一行注释：`# 右臂 per-joint kd（与 C++ 一致，joint2 kd=3.0 提供关键稳定性）`——说明在 C++ 版本里，右臂关节 2 曾经需要较大的 $k_d$ 才稳定。这是一个"调参经验"的真实线索：**纯力矩模式下，小延迟 + 模型误差会让系统有自激倾向，$k_d$ 是最简单的稳定手段，代价是增加手感阻力**。

### 8.5 与学习笔记中项目 C++ 代码的对应

学习笔记里你们项目的主从代码（`control.cpp` 等）的 Leader 分支是：

```text
gravity_comp_ratio = [1, 1, 1, 1, 1, 1, 0.8]
effort = gravity_comp_ratio × g(q)
MIT: pos=0, vel=0, kp=0, kd=0, torque=effort
```

对应关系：

| 仓库参考脚本（Python） | 项目 C++（学习笔记） |
|---|---|
| `GRAVITY_SCALE = 1.0`（标量） | `gravity_comp_ratio` 逐关节向量，第 7 关节 0.8 |
| 额外：科氏 0.1、摩擦 0.3 | 笔记提醒：Coriolis/摩擦函数存在，但活动赋值只用 ratio × gravity（混合赋值被注释） |
| `kd` 非零（右臂） | `kd=0` |

**为什么第 7 关节取 0.8？** 笔记里没有给出原因。常见理由（典型做法，不是项目事实）：末端关节后面是夹爪/手柄，模型对其质心与载荷最不确定；**宁可略欠补偿**。欠补偿的后果是手臂缓慢下沉（可见、可控）；过补偿的后果是手臂自己"飘起来"（意外加速，危险）。**【待你补全：你们为什么取 0.8？】**

### 8.6 启动：先移动到已知姿态

```python
INIT_POS = [-0.049, 0.53, -0.3313, 1.272, 0.521, 0.05855, 0.605]
kp = [50,50,50,50,10,10,10];  kd = [1.2,1.2,1.2,1.2,0.3,0.2,0.3]
steps = 220; step_ms = 10        # 线性插值，约 2.2 s
```

使能电机后，先用**位置模式（有 $k_p,k_d$）**把手臂缓慢插值到一个已知安全姿态，再切换到纯力矩的重力补偿。这样做的理由：

1. 使能瞬间手臂可能处在任意姿态，先锁住再说；
2. 已知姿态便于核对补偿方向（见第 11 节的验证）；
3. 避免从"完全无力"状态直接进入补偿时的瞬态。

---

## 9. 用 URDF 离线估算 xArm 的重力矩（教学推算）

为了让数字更有体感，我用仓库的 URDF 做了一次离线推算：把所有连杆质量、质心、关节轴/原点读入，用势能 $U=\sum m_k g_0 z_k$ 对每个关节角求数值偏导得到 $g(q)$（方向：电机需要提供的、能保持姿态的力矩）。**这是教学推算，不是实机测量**；与真机读数不会完全相同（线缆、减速器、夹爪、零位偏差都会造成差异）。

右臂运动部分总质量（link1–7 + 手 + 两指）：

$$
1.015+0.250+0.982+0.484+0.482+0.392+0.430+0.350+2\times0.036\approx4.46\ \text{kg}.
$$

在脚本的 `INIT_POS` 姿态下，各关节所需的重力补偿力矩（推算）：

| 关节 | 1 | 2 | 3 | 4 | 5 | 6 | 7 |
|---|---|---|---|---|---|---|---|
| $g_i$（N·m） | 3.37 | 2.57 | 1.92 | 3.36 | 0.013 | 0.004 | 0.80 |
| 库中配置的电机力矩范围 $t_{\max}$ | 54 | 54 | 28 | 28 | 10 | 10 | 10 |
| 占比 | 6.2% | 4.8% | 6.9% | 12.0% | 0.1% | 0.04% | 8.0% |
| MIT 力矩分辨率 $2t_{\max}/4095$ | 0.026 | 0.026 | 0.0137 | 0.0137 | 0.0049 | 0.0049 | 0.0049 |

从这张表可以读出几件"有用的事"：

1. **重量主要压在前 4 个关节**（肩、上臂、肘），腕部关节在这个姿态下几乎不承重；这与"人推的是整条臂"相吻合；
2. 关节 5、6 的重力矩只有 0.013、0.004 N·m，**小于或接近 MIT 12 位力矩的 1–3 个最小刻度**（0.0049 N·m）。也就是说在这个姿态，腕部补偿的数值量化噪声和补偿量本身同量级——所以它们的"好不好推"更多取决于摩擦，而不是补偿精度；
3. 关节 4（肘）占其电机范围的比例最高（12%），是**补偿误差最敏感**的关节；
4. 力矩分辨率的另一个后果：MIT 编码用 `int()` 截断（向零取整），所以"发送 0 力矩"实际解码为约 $-0.5$ 个 LSB（DM4310：$-0.0024$ N·m），是一个很小的负偏置。

> 想自己复现这张表：用 URDF 解析器读入各 link 的 `mass` 与 `origin xyz`，各 joint 的 `origin`/`axis`，沿运动链做 FK 得到每个质心的世界高度，对 $U$ 做中心差分。这也是一个很好的"独立校验 KDL/Pinocchio 输出"的方法。

---

## 10. 主臂为什么"低阻力"，以及残余阻力来自哪里

### 10.1 合力矩分析

人推主臂时，关节受力矩：人 $\tau_h$、重力 $-g(q)$、电机 $+g_{cmd}(q)$（补偿）、摩擦 $-\tau_f$、阻尼 $-k_d\dot q$。运动方程：

$$
M\ddot q=\tau_h+\underbrace{(g_{cmd}-g)}_{\text{补偿误差}}-\tau_f-k_d\dot q.
$$

若 $g_{cmd}=g$（补偿准确）且 $k_d$ 很小，则

$$
M\ddot q\approx\tau_h-\tau_f.
$$

人只需克服**摩擦**和**惯性**。学习笔记的写法 $\tau_{total}=\tau_{human}+g(q)-g(q)=\tau_{human}$ 是同一个意思。

### 10.2 如果设了 $k_p>0$ 会怎样

电机会试图回到 `pos` 目标，产生回复力矩 $k_p(q^*-q)$，人推起来就像推弹簧，感觉"硬"。所以 Leader 必须 $k_p=0$。

### 10.3 手感阻力的数量级（示例）

设残余的摩擦 + 补偿误差合计 0.3 N·m，操作者握持点离关节 0.4 m，则需要的推力 $F=0.3/0.4=0.75$ N，约 76 gf。这个量级是"几乎感觉不到"。若补偿缺失 20%（关节 1 的 $g=3.37$ N·m → 误差 0.67 N·m）则变成 1.7 N，约 170 gf，仍可接受，但手臂会有明显的"下沉/上飘趋势"。

### 10.4 阻力的真实来源

| 来源 | 说明 | 对策 |
|---|---|---|
| 摩擦（库仑 + 黏性） | 电机轴承、减速器 | 选低减速比/准直驱电机；可选摩擦前馈（只补一部分） |
| 反射惯量 | 转子惯量 × 减速比² | 低减速比；加速度大时才明显 |
| 补偿误差 | 质量/质心/零位/线缆 | 校准 URDF；调 `gravity_comp_ratio` |
| 控制延迟 | 补偿用的是上一周期的 $q$ | 提高循环频率 |
| 速度阻尼 $k_d$ | 为稳定而加 | 取尽量小 |
| 量化 | MIT 12 位力矩分辨率 | 无法避免，注意腕部小力矩 |

---

## 11. 符号、零位、关节顺序：四类典型错误与验证

学习笔记第 24–28 组你问过"力矩方向"问题。把链路上所有可能反的地方列出来：

```text
URDF 关节轴方向 ─► KDL 输出符号约定 ─► 关节→电机映射(±1) ─► 电机正力矩方向 ─► 减速器方向 ─► 编码器正方向
```

任何一层反了，整体就反。

### 11.1 符号错误

后果：电机力矩与重力同向，手臂**加速往下掉**，越拖越重。

**映射公式**：设 $q_{joint}=s_i\,(q_{motor}-q_{off,i})$（$s_i=\pm1$）。则关节力矩 $\tau_{joint}$ 对应的电机力矩是 $\tau_{motor}=s_i\,\tau_{joint}$。例：若某关节 $s=-1$，URDF 算出需要 $+2.1$ N·m，则必须向电机发 $-2.1$ N·m。

### 11.2 零位错误

零位偏差 $\delta$ 使得用错误的 $q+\delta$ 去算重力。以一根杆 $\tau=mg_0 r\cos q$ 为例（$m=1,r=0.5$）：在 $q=60^\circ$ 处 $\Delta\tau\approx mg_0 r\sin q\cdot\delta=4.905\times0.866\times0.05=0.21$ N·m（$\delta=0.05$ rad ≈ 2.9°）。零位偏差在力臂变化最快的姿态（接近竖直）最敏感。

### 11.3 关节顺序错位

KDL/Pinocchio 的关节顺序由 URDF 树决定；电机 ID 是 1–7。若顺序对不上，补偿会被发给错的电机——某个关节会莫名其妙被推。仓库脚本用 `nv==7` 与关节名前缀校验，并打印 `关节顺序: [...]`。

### 11.4 单位与缩放

力矩单位 N·m；MIT 编码把 N·m 映射到 12 位；`gear` 与"关节力矩 vs 电机轴力矩"要区分（MIT 报文里的力矩通常已是**输出轴**上的值，具体以电机手册为准）。**【待你补全：实机使用的力矩单位/映射】**

### 11.5 验证步骤（推荐在真机上这样做）

1. 把手臂摆到**水平**（重力矩最大、最易观察）；
2. 只发 $g(q)$（kp=kd=0），手松开；
3. 观察：保持不动或极缓慢漂移 → 方向和量级正确；**加速向下掉** → 符号反；**向上飘** → 过补偿或符号反且量大；
4. 对每个关节单独测（其他关节先用位置模式锁住）；
5. 再把 `gravity_comp_ratio` 从 0.5 逐步升到 1.0，观察临界点；
6. 可以对比"理论 $g(q)$"和电机回读的力矩 `get_torques_measured`，看比例是否接近 1。

---

## 12. 常见故障与定位

| 现象 | 可能原因 | 定位 |
|---|---|---|
| 一松手手臂下垂 | 补偿不足；比例太小；质量偏小 | 提高比例；核对 URDF 质量 |
| 一松手手臂上飘 | 过补偿；URDF 质量偏大；符号反 + 大力矩 | 降比例；做 11.5 的验证 |
| 越拖越重 | 符号反 | 映射层乘 −1 |
| 某个关节怪异，其他正常 | 关节顺序/零位/单位 | 单关节测试；核对 `joint_to_motor` |
| 手臂抖/啸叫 | 纯力矩模式下延迟 + 模型误差自激 | 增加小 $k_d$；提高控制频率；降低科氏补偿增益 |
| 摩擦感明显 | 未补摩擦或摩擦模型过小 | 少量摩擦补偿（如 30%） |
| 某关节饱和 | 补偿力矩超过编码范围 | 检查 $t_{\max}$ 与钳位；见 [05 章](./05_约束与限位.md) |
| 补偿滞后、时轻时重 | 循环太慢 | 测实际周期；减少阻塞等待 |
| 腕部总是"粘" | 小力矩被量化 + 摩擦主导 | 关节 5/6 补偿本来就小，别强求 |

---

## 13. 面试怎么问、怎么答

**Q1. Why can the leader arm be pushed so easily?**

> Because the motors only output the torque that cancels gravity, with no position stiffness. I compute the gravity torque from the URDF masses and centres of mass for the current joint angles, and send it as the feed-forward torque in the MIT frame with kp and kd set to zero. Then the torque from my hand only has to overcome friction and inertia, so it feels light.

中文要点：电机只出重力补偿力矩，kp=kd=0；人只克服摩擦+惯性。

*追问*：What if you set kp > 0? → 电机会回到 pos 目标，像弹簧，推起来硬。

**Q2. Derive the gravity term of a two-link arm.**

> I write the potential energy of the two centres of mass, U equals g times m1 y1 plus m2 y2. The gravity torque on each joint is the partial derivative of U with respect to that joint angle. For equal unit links with centres at the midpoints, with the arm stretched horizontally, joint 1 needs 19.6 newton-metres and joint 2 needs 4.9. When the links point straight up the lever arm is zero and the torque is zero, although the weight is still there.

**Q3. What is in the dynamics equation, and why is gravity compensation enough here?**

> M of q times q-double-dot is the inertia term, C times q-dot is Coriolis and centrifugal, g of q is gravity, and tau is the joint torque. In hand guiding the speeds are low and Coriolis grows with speed squared, so gravity dominates. I only compensate gravity, and optionally a small fraction of friction. Full inverse dynamics would be needed for fast trajectory tracking, but it depends on accurate inertia parameters.

**Q4. What can go wrong with gravity compensation on the real arm, and how do you check?**

> The classic issues are a wrong sign, wrong joint order, a zero-offset mismatch, and an inaccurate URDF mass. I check by holding the arm horizontal, sending only the gravity torque for one joint, and watching: if it falls faster the sign is reversed; if it floats up, I over-compensate. Then I raise the compensation ratio gradually from about half to one.

**Q5. Why is the compensation ratio of the last joint 0.8 in your code?** *(此题按你的真实原因回答)*

> The wrist and gripper payload is the least certain part of the model, so I prefer slight under-compensation: the arm sinks slowly and is easy to notice, whereas over-compensation makes it drift upward on its own, which is less safe. 【请核对是否与你们真实原因一致；若不确定，改答："I kept it at 0.8 after tuning on the arm; I don't remember the exact reason" 并补全。】

**Q6. KDL or Pinocchio — what is the difference?**

> Both compute the same rigid-body quantities from a URDF. I used KDL's chain dynamics for the gravity vector. Pinocchio offers similar functions such as computeGeneralizedGravity and also analytic derivatives. For gravity compensation of a single serial arm, the result is the same.

---

## 本章要点速记

1. $M\ddot q+C\dot q+g=\tau$；静止时 $\tau=g(q)$。
2. 二连杆：$g_1=g[(\frac{m_1l_1}2+m_2l_1)\cos q_1+\frac{m_2l_2}2\cos(q_1+q_2)]$，$g_2=g\frac{m_2l_2}{2}\cos(q_1+q_2)$；水平 19.62 / 4.905 N·m，竖直为 0。
3. 竖直时重力矩≈0 是因为力臂为零，不是重量消失。
4. 示教时速度低，科氏项 ∝ 速度²，重力为主导 → 只补重力（+少量摩擦）。
5. URDF 决定对错：质量、质心、关节轴/原点、零位、重力方向。
6. KDL：`Tree → Chain → ChainDynParam(chain, g) → JntToGravity`；Pinocchio：`computeGeneralizedGravity`。
7. 主臂 MIT 帧：`pos=0, vel=0, kp=0, kd≈0, torque=ratio×g(q_actual)`；用**反馈** $q$ 算，不是指令。
8. 低阻力来源：无位置刚度 + 补偿到位；残余阻力=摩擦+惯量+补偿误差+$k_d$。
9. 符号/零位/顺序/单位四类错误；验证：水平摆臂只发 $g$，看掉还是飘。
10. 补偿比例 <1 偏安全（下沉可控），>1 危险（上飘）。

## 自测题

1. 1 kg 杆质心 0.5 m 水平，重力矩多大？竖直呢？（4.905 N·m；≈0。）
2. 二连杆（$m=l=1$）在 $(30^\circ,45^\circ)$ 的 $g_1,g_2$？（14.01、1.27 N·m。）
3. 为什么重力补偿要用反馈的 $q$ 而不是指令 `pos`？（真实姿态才决定真实重力矩；kp=0 时指令不起作用。）
4. 主臂为什么 kp 必须为 0？kd 为什么常取很小的非零值？（kp>0 有回复力；kd 提供稳定性但增加阻力。）
5. 补偿符号反了会有什么现象？怎么修？（越拖越重；映射层乘 −1。）
6. 为什么腕部关节的补偿力矩在某些姿态下会"小于 MIT 力矩分辨率"？（负载很小、$t_{\max}$ 的 12 位量化步长 0.0049 N·m。）

## 相关链接

- 上一章：[05 约束与限位](./05_约束与限位.md)
- 下一章：[07 主从跟踪与 MIT 电机接口](./07_主从跟踪与MIT电机接口.md)
- 源码对照：[12 源码导读](./12_源码导读.md)
- 基础：[拉格朗日方程与机器人动力学](../../02_基础知识/02_力学与建模/02_拉格朗日方程与机器人动力学.md)、[机械臂动力学_重力补偿_URDF_KDL](../../02_基础知识/03_坐标变换与运动学/06_机械臂动力学_重力补偿_URDF_KDL.md)、[URDF_TF_坐标系实践](../../02_基础知识/09_机器人软件/07_URDF_TF_坐标系实践.md)
- 学习笔记：[逐字稿第 35–37 组与第二份 KDL 章节](../../05_学习笔记归档/P2/P2_本对话问答逐字稿_第二份.md)
