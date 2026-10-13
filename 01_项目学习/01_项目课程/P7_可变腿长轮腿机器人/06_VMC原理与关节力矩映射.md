# VMC 原理与关节力矩映射：虚拟力怎样变成两个电机的力矩

> **一句话定位**：LQR 给出了"轮毂力矩 $T$"和"髋部力矩 $T_p$"，腿长环给出了"支撑力 $F$"，但电机装在两个髋关节上，只认**关节力矩**。VMC（Virtual Model Control，虚拟模型控制）就是这座桥：先在**虚拟腿**上规定想要的力和力矩，再用**雅可比转置**把它换算成两个关节的力矩。

**本章学完你能：**

- 讲清 VMC 的思想（虚拟弹簧阻尼）以及它与逆运动学位置控制的区别；
- 从"杆长不变"推出足端雅可比 $J$，逐项解释 $j_{11},j_{12},j_{21},j_{22}$；
- 用虚功/功率守恒推出 $\boldsymbol\tau=J^\top\mathbf F$，并解释为什么是转置；
- 完整手算一个姿态：J、$J_p$、$\tau_\alpha,\tau_\beta$ 的数值，并用静力平衡检验；
- 把 $(F_x,F_y)$ 形式的笛卡尔 VMC（课程第十一课）与 $(F,T_p)$ 形式的虚拟腿 VMC（你的 CV）联系起来；
- 解释雅可比条件数、单位（mm 与 N·m）和左右腿镜像的坑。

**前置知识**：

- 雅可比：[正逆运动学与雅可比](../../02_基础知识/03_坐标变换与运动学/03_正逆运动学与雅可比.md)
- 力矩控制：[电机、编码器、驱动与力矩控制](../../02_基础知识/08_嵌入式与通信/05_电机_编码器_驱动与力矩控制.md)
- [02_五连杆闭链运动学.md](./02_五连杆闭链运动学.md)、[03_虚拟腿与坐标符号约定.md](./03_虚拟腿与坐标符号约定.md)

**目录**

1. VMC 要解决什么问题
2. 虚拟弹簧—阻尼：想要的"虚拟力"从哪来
3. 足端雅可比 J 的推导
4. 完整手算 J（α=135°，β=45°）
5. 为什么 τ = Jᵀ F：虚功推导
6. 虚拟腿 VMC：(F, Tp) 到关节力矩
7. 手算：一个姿态的完整力矩分配与静力检验
8. 课程代码：笛卡尔 VMC 与极坐标 VMC
9. 条件数、奇异性、单位与镜像
10. 常见错误与故障现象
11. 面试怎么问、怎么答
12. 要点速记、自测题、相关链接

---

## 1. VMC 要解决什么问题

### 1.1 位置控制 vs 力控制

腿有两种驱动方式：

| | 位置控制（逆运动学 + 关节位置环） | 力矩控制（VMC） |
|---|---|---|
| 做法 | 给目标腿端位置 → 逆解得 α、β → 关节位置环 | 给目标腿端**力** → 雅可比转置 → 关节力矩 |
| 腿的"手感" | 刚（像钢杆），地面一颠就把冲击直接传给机身 | 柔（像弹簧），可调刚度和阻尼 |
| 与平衡控制的配合 | 难：平衡环要的是"力/力矩"，而不是位置 | 自然：LQR 的 $T_p$、腿长环的 $F$ 本来就是力和力矩 |
| 典型用途 | 课程第六、九课（腿高由 IK + 关节 kp 实现） | 课程第十一课（腿力矩由 VMC 给出）；P7 |

课程把同一台机器人用两种方式做过一遍：第六课是位置控制（`inverseKinematics` + MIT 模式下的 `kp`），第十一课是力矩控制（MIT 模式下 `kp=0, kd=0, tor=VMC 力矩`）。你的项目是后者。

### 1.2 一句话

> **VMC：先想"如果腿端接了一根弹簧和一个阻尼器，它会产生什么力"，再让电机产生这个力。**

---

## 2. 虚拟弹簧—阻尼：想要的"虚拟力"从哪来

### 2.1 一维类比

想让腿保持高度 $y^*$：在身体和腿端之间想象一根弹簧（刚度 $K_p$）和一个阻尼器（阻尼 $K_d$）。腿比目标短，弹簧要伸，推出向外的力；腿端运动太快，阻尼器反方向耗能：

$$
F=K_p\,(y^*-y)-K_d\,\dot y
$$

### 2.2 虚拟力的几种组成（课程第十一课）

以 y 向下的坐标系，腿端虚拟力 $F_y$（正 = 把腿端往下推，也就是**伸腿**）：

$$
F_y=\underbrace{K_{p,h}(y^*-y)}_{\text{弹簧（腿高）}}-\underbrace{K_{d,h}\dot y}_{\text{阻尼}}+\underbrace{F_{balance}}_{\text{横滚/左右协调}}+\underbrace{F_g}_{\text{重力前馈}}
$$

课程参数：`kpHeight=0.3`，`kdHeight=0.06`，`fG=25`；$F_{balance}$ 右腿取 $+roll\_EH$，左腿取 $-roll\_EH$（一边加、一边减）。

水平方向类似：

$$
F_x=K_{p,x}(x^*-x)-K_{d,x}\dot x,\qquad K_{p,x}=0.4,\ K_{d,x}=0.05
$$

**手算（课程参数）**：高度误差 10 mm，腿端向下速度 5 mm/s，平衡修正取 0：

$$
F_y=0.3\times10-0.06\times5+0+25=3-0.3+25=27.7
$$

单位按"mm 与 N"约定，$F_y=27.7$ N（见第 9 节单位说明）。这个例子说明：**重力前馈占了大头，弹簧项只负责修正偏差**（第 7 章）。

在 P7 里，这套"虚拟力"的思想相同，但是定义在**虚拟腿的极坐标**上：沿腿方向的**支撑力 $F$**（来自腿长 PID + 前馈）和绕髋的**力矩 $T_p$**（来自 LQR）。第 6 节讲这个。

---

## 3. 足端雅可比 J 的推导

### 3.1 要推的东西

足端（腿端 B）速度与关节角速度的线性关系：

$$
\begin{bmatrix}\dot x_B\\\dot y_B\end{bmatrix}=J\begin{bmatrix}\dot\alpha\\\dot\beta\end{bmatrix},\quad
J=\begin{bmatrix}\partial x_B/\partial\alpha&\partial x_B/\partial\beta\\\partial y_B/\partial\alpha&\partial y_B/\partial\beta\end{bmatrix}
$$

$J$ 的第 $i$ 行对应 B 的一个坐标，第 $j$ 列对应一个关节；**它随姿态变化**，每个周期要重新算。单位：mm/rad（课程杆长为 mm）。

### 3.2 推导：杆长不变 ⇒ 沿杆方向速度相等

沿用第 2 章记号：$A=L_1(\cos\alpha,\sin\alpha)$，$C=(L_5+L_4\cos\beta,\ L_4\sin\beta)$，$B=A+L_2\mathbf u_1=C+L_3\mathbf u_2$，$\mathbf u_k=(\cos\theta_k,\sin\theta_k)$。

**A 点速度**（A 绕 O 转，半径 $L_1$，角速度 $\dot\alpha$）：

$$
\dot A=L_1\dot\alpha\begin{bmatrix}-\sin\alpha\\\cos\alpha\end{bmatrix}
$$

**杆 AB 长度不变** ⇒ B 相对 A 的速度垂直于 AB ⇒ B 的速度和 A 的速度在 AB 方向上的分量**相等**：

$$
\mathbf u_1\cdot\dot B=\mathbf u_1\cdot\dot A=L_1\dot\alpha(-\cos\theta_1\sin\alpha+\sin\theta_1\cos\alpha)=L_1\sin(\theta_1-\alpha)\,\dot\alpha
$$

即

$$
\cos\theta_1\,\dot x_B+\sin\theta_1\,\dot y_B=L_1\sin(\theta_1-\alpha)\,\dot\alpha
$$

**杆 CB 同理**（C 绕 D 转，半径 $L_4$，角速度 $\dot\beta$）：

$$
\cos\theta_2\,\dot x_B+\sin\theta_2\,\dot y_B=L_4\sin(\theta_2-\beta)\,\dot\beta
$$

> **提示**：右边出现 $L_1$ 和 $L_4$（主动杆的长度）而不是 $L_2,L_3$，因为被投影的是 **A 点和 C 点的圆周速度**，它们的转动半径是主动杆长；被动杆长度在"投影关系"里约掉了。这是你笔记第 25 组卡住的地方。

**写成矩阵并求解：**

$$
\underbrace{\begin{bmatrix}\cos\theta_1&\sin\theta_1\\\cos\theta_2&\sin\theta_2\end{bmatrix}}_{M_c}\begin{bmatrix}\dot x_B\\\dot y_B\end{bmatrix}
=\underbrace{\begin{bmatrix}L_1\sin(\theta_1-\alpha)&0\\0&L_4\sin(\theta_2-\beta)\end{bmatrix}}_{D_c}\begin{bmatrix}\dot\alpha\\\dot\beta\end{bmatrix}
$$

$\det M_c=\cos\theta_1\sin\theta_2-\sin\theta_1\cos\theta_2=\sin(\theta_2-\theta_1)$，所以

$$
J=M_c^{-1}D_c=\frac{1}{\sin(\theta_2-\theta_1)}\begin{bmatrix}\sin\theta_2&-\sin\theta_1\\-\cos\theta_2&\cos\theta_1\end{bmatrix}D_c
$$

展开并整理（注意 $\sin(\theta_2-\theta_1)=-\sin(\theta_1-\theta_2)$），得到课程代码里的四个系数：

$$
\boxed{\begin{aligned}
j_{11}&=\frac{L_1\sin\theta_2\,\sin(\alpha-\theta_1)}{\sin(\theta_1-\theta_2)},&
j_{12}&=\frac{L_4\sin\theta_1\,\sin(\theta_2-\beta)}{\sin(\theta_1-\theta_2)},\\[4pt]
j_{21}&=-\frac{L_1\cos\theta_2\,\sin(\alpha-\theta_1)}{\sin(\theta_1-\theta_2)},&
j_{22}&=-\frac{L_4\cos\theta_1\,\sin(\theta_2-\beta)}{\sin(\theta_1-\theta_2)}
\end{aligned}}
$$

（推导中 $\sin(\theta_1-\alpha)=-\sin(\alpha-\theta_1)$ 吸收了一个负号。）

### 3.3 它与视频的符号推导等价

视频用符号求解先解出 $\dot\theta_1$：

$$
\dot\theta_1=\frac{L_1\dot\alpha\,\sin(\theta_2-\alpha)-L_4\dot\beta\,\sin(\theta_2-\beta)}{L_2\sin(\theta_1-\theta_2)}
$$

再代入 $\dot B=\dot A+L_2\dot\theta_1(-\sin\theta_1,\cos\theta_1)$。两种做法结果一致：我用数值（有限差分）对课程的四个系数做了核对，三个姿态下完全吻合（第 13 章）。

---

## 4. 完整手算 J（α=135°，β=45°）

由第 2 章：$\theta_1=50.188^\circ$，$\theta_2=129.812^\circ$。

三角值：

| 量 | 值 |
|---|---|
| $\sin\theta_1=\sin\theta_2$ | 0.76815 |
| $\cos\theta_1$ | 0.64026 |
| $\cos\theta_2$ | −0.64026 |
| $\sin(\theta_1-\theta_2)=\sin(-79.62^\circ)$ | −0.98364 |
| $\sin(\alpha-\theta_1)=\sin84.81^\circ$ | 0.99590 |
| $\sin(\theta_2-\beta)=\sin84.81^\circ$ | 0.99590 |

逐项：

$$
j_{11}=\frac{150\times0.76815\times0.99590}{-0.98364}=\frac{114.75}{-0.98364}=-116.66
$$

$$
j_{12}=\frac{150\times0.76815\times0.99590}{-0.98364}=-116.66
$$

$$
j_{21}=-\frac{150\times(-0.64026)\times0.99590}{-0.98364}=-\frac{-95.645}{-0.98364}=-97.24
$$

$$
j_{22}=-\frac{150\times0.64026\times0.99590}{-0.98364}=+97.24
$$

所以

$$
J=\begin{bmatrix}-116.66&-116.66\\-97.24&+97.24\end{bmatrix}\ \text{mm/rad}
$$

**读这张表（对称姿态的物理含义）：**

- 第一行相同（$-116.66,-116.66$）：两个关节**同向**转动 → B 水平方向移动（摆腿）；
- 第二行相反（$-97.24,+97.24$）：两个关节**反向**转动 → B 竖直方向移动（伸缩腿）；
- 符号：$\alpha$ 增大 → OA 往左转，B 往左（$x$ 减小）也往上（$y$ 减小，腿变短）；$\beta$ 增大 → DC 往左转，B 往左、往下（腿变长）。

**用约束方程直接验证**（即 $M_c\dot{\mathbf p}=D_c\dot{\mathbf q}$）：取 $\dot\alpha=0.2,\ \dot\beta=0.1$ rad/s：

右边：$[-149.385\times0.2,\ 149.385\times0.1]=[-29.88,\ 14.94]$。

用 $J$ 得 $\dot{\mathbf p}=[-116.66\times0.3,\ -97.24\times0.2+97.24\times0.1]=[-35.00,\ -9.72]$ mm/s。

代回左边：第一行 $0.64026(-35.00)+0.76815(-9.72)=-22.41-7.47=-29.88$ ✓；第二行 $-0.64026(-35.00)+0.76815(-9.72)=22.41-7.47=14.94$ ✓。

---

## 5. 为什么 τ = Jᵀ F：虚功推导

### 5.1 方法一：虚功

设腿端受力 $\mathbf F=[F_x,F_y]^\top$（**腿对外界**施加的力的"广义等价"），关节力矩 $\boldsymbol\tau=[\tau_\alpha,\tau_\beta]^\top$。给关节一个微小转动 $\delta\mathbf q$，腿端随之移动 $\delta\mathbf p=J\,\delta\mathbf q$。

- 腿端力做的功：$\delta W=\mathbf F^\top\delta\mathbf p=\mathbf F^\top J\,\delta\mathbf q$；
- 关节力矩做的功：$\delta W=\boldsymbol\tau^\top\delta\mathbf q$。

若机构无摩擦、无惯性（准静态平衡），两者必须相等对任意 $\delta\mathbf q$ 成立：

$$
\boldsymbol\tau^\top=\mathbf F^\top J\ \Longrightarrow\ \boxed{\boldsymbol\tau=J^\top\mathbf F}
$$

### 5.2 方法二：功率守恒

对时间求导：$\boldsymbol\tau^\top\dot{\mathbf q}=\mathbf F^\top\dot{\mathbf p}=\mathbf F^\top J\dot{\mathbf q}$，同样得到 $\boldsymbol\tau=J^\top\mathbf F$。

### 5.3 一句话记忆

> **$J$ 把关节速度换成腿端速度；$J^\top$ 把腿端的力换成关节力矩。** 它不是求逆，不需要解方程，只做一次矩阵乘法；而且在奇异姿态附近，$J$ 本身会变大，但 $J^\top\mathbf F$ 仍然是良定义的（力矩有限），只是力矩对力的"放大倍数"变化。

与逆运动学相反：IK 需要解方程且有多解；VMC 的这一步是**线性的**，每个周期都能稳定地算。

---

## 6. 虚拟腿 VMC：(F, Tp) 到关节力矩

CV 说的 "mapped support force and pitch torque to joints via the Jacobian"，指的就是这一步。

### 6.1 定义

虚拟腿（第 3 章）以髋中点 H 为原点，腿长 $L_0$，腿几何角 $\phi_0$。想要的两个"广义力"：

- **$F$**：沿腿方向的**支撑力**，$F>0$ 表示腿**伸出**（把轮往下推、把机身往上顶）；
- **$T_p$**：绕髋 H 的**力矩**，$T_p>0$ 表示使腿几何角 $\phi_0$ 增大的方向（约定 C）。

它们是虚拟腿坐标 $(L_0,\phi_0)$ 的**广义力**：功率 $=F\dot L_0+T_p\dot\phi_0$。

### 6.2 映射

第 3 章已有 $[\dot L_0,\dot\phi_0]^\top=J_p[\dot\alpha,\dot\beta]^\top$，$J_p=J_rJ$。由功率守恒：

$$
\boxed{\begin{bmatrix}\tau_\alpha\\\tau_\beta\end{bmatrix}=J_p^\top\begin{bmatrix}F\\T_p\end{bmatrix}}
$$

### 6.3 与笛卡尔形式的关系（旋转一下力）

腿端力分解为沿腿（径向）和垂直腿（切向）两个分量：

- 径向单位向量 $\hat{\mathbf r}=(\cos\phi_0,\sin\phi_0)$，切向单位向量 $\hat{\mathbf t}=(-\sin\phi_0,\cos\phi_0)$（$\phi_0$ 增大方向）；
- 腿端力 $\mathbf f=F\hat{\mathbf r}+\dfrac{T_p}{L_0}\hat{\mathbf t}$（力矩除以力臂 $L_0$ 就是切向力）。

即

$$
\begin{bmatrix}F_x\\F_y\end{bmatrix}=\begin{bmatrix}\cos\phi_0&-\sin\phi_0\\\sin\phi_0&\cos\phi_0\end{bmatrix}\begin{bmatrix}F\\T_p/L_0\end{bmatrix}
$$

然后 $\boldsymbol\tau=J^\top\mathbf f$。因为 $J_p=J_rJ$ 且 $J_r$ 的两行正好是 $\hat{\mathbf r}^\top$ 和 $\hat{\mathbf t}^\top/L_0$，所以 $J_p^\top=J^\top[\hat{\mathbf r},\ \hat{\mathbf t}/L_0]$——**两种写法完全等价**。

> **面试要点**：笛卡尔 VMC 控制的是腿端的 $(F_x,F_y)$；虚拟腿 VMC 控制的是 $(F,T_p)$——"沿腿的力"和"绕髋的力矩"。后者更贴合倒立摆模型（$T_p$ 正是 LQR 的第二个输出，$F$ 对应腿长），所以 CV 里用它。

---

## 7. 手算：一个姿态的完整力矩分配与静力检验

姿态：α=135°，β=45°，$L_0=0.298105$ m，$\phi_0=90^\circ$。把 $J$ 换成 m/rad：

$$
J=\begin{bmatrix}-0.11666&-0.11666\\-0.09724&0.09724\end{bmatrix}
$$

$J_r=\begin{bmatrix}0&1\\-1/L_0&0\end{bmatrix}$，所以

$$
J_p=J_rJ=\begin{bmatrix}-0.09724&0.09724\\0.39134&0.39134\end{bmatrix}
$$

$$
J_p^\top=\begin{bmatrix}-0.09724&0.39134\\0.09724&0.39134\end{bmatrix}
$$

**情形 1：纯支撑力 $F=40$ N，$T_p=0$**

$$
\tau_\alpha=-0.09724\times40=-3.889\ \text{N·m},\qquad\tau_\beta=+3.889\ \text{N·m}
$$

两个关节力矩大小相等、符号相反（对称姿态下的伸腿 = 差模）。

**情形 2：纯髋力矩 $T_p=1$ N·m，$F=0$**

$$
\tau_\alpha=\tau_\beta=0.39134\ \text{N·m}
$$

两个关节力矩同号（共模）。

**情形 3：组合 $F=40$ N，$T_p=-1.5$ N·m**

$$
\tau_\alpha=-3.889+0.39134\times(-1.5)=-4.476,\qquad
\tau_\beta=3.889-0.587=3.302\ \text{N·m}
$$

**静力检验（教学数值）：** 机身 6 kg、每条腿承担一半 $\approx M g/2=29.4$ N。此时

$$
|\tau_\alpha|=|\tau_\beta|=0.09724\times29.43=2.86\ \text{N·m}
$$

每个髋关节要持续输出约 2.9 N·m 才能静态支撑。这给选电机和估算"站立不动也要耗电"提供依据，也说明为什么**重力前馈**必须放进支撑力（否则腿长 PID 的积分/比例项要独自承担 29 N，响应慢）。

**从笛卡尔形式核对：** 竖直向下的腿端力 $F_y=40$ N（情形 1 的 $F=40$ 在 $\phi_0=90^\circ$ 时指向 +y），$\boldsymbol\tau=J^\top[0,40]^\top=[-0.09724\times40,\ +0.09724\times40]=[-3.889,3.889]$ ✓，与虚拟腿形式完全一致。

---

## 8. 课程代码：笛卡尔 VMC 与极坐标 VMC

### 8.1 第十一课（笛卡尔形式，力矩模式）

`src/robot.cpp: calRightVMC / calLeftVMC`（要点）：

```text
1. footRightVelocity(&xdot,&ydot, …, vel[0], vel[2])   足端速度（见下）
2. forwardKinematicsRight(...)                         足端位置 x, y
3. fHeight = kpHeight*(targetHeight − y)               高度弹簧
   Fy = fHeight + fBalance − kdHeight*ydot + fG        + 横滚平衡 + 阻尼 + 重力前馈(25)
4. Fx = kpX*(targetX − x) − kdX*xdot                   前后弹簧阻尼
5. θ2 = acos((x − (L5 + L4 cosβ))/L3)                  取被动杆角
6. j11,j12,j21,j22                                    即本章 3.2 节公式
7. backTorque  = start*(j11*Fx + j21*Fy)/1000          = τ = Jᵀ F
   frontTorque = start*(j12*Fx + j22*Fy)/1000
```

关键点：

- **单位**：$J$ 为 mm/rad，$F$ 为 N，所以 $J\cdot F/1000$ 的单位是 N·m（这个 `/1000` 由量纲**推算**得到，源码没有注释，面试时不要说成"源码注释"）；
- `start` 是遥控器使能标志，关闭时力矩为零；
- 左右腿的 `front/back` 字段对应不同的节点 ID（见第 12 章）；
- 足端速度 $\dot x,\dot y$ 由 `footRightVelocity` 计算：先由约束算 $\dot\theta_1=(\dot c-\dot a\cos\theta_1-\dot b\sin\theta_1)/(b\cos\theta_1-a\sin\theta_1)$，再 $\dot x=-L_1\sin\alpha\dot\alpha-L_2\sin\theta_1\dot\theta_1$，$\dot y=L_1\cos\alpha\dot\alpha+L_2\cos\theta_1\dot\theta_1$——这是对第 3 章公式的另一种数值写法，与 $J\dot{\mathbf q}$ 等价。

### 8.2 第九课（极坐标形式）

第九课 `calVMC_Right/Left` 已经是**虚拟腿形式**：力 `F`（沿腿）与 `torque`（绕髋），$j$ 的写法包含 `polarAngle` 与 `radius`：

```text
j11 = L1 sin(polar − θ2) sin(α − θ1) / sin(θ2 − θ1)
j12 = L1 cos(polar − θ2) sin(α − θ1) / (radius sin(θ2 − θ1))
j21 = L4 sin(polar − θ1) sin(θ2 − β) / sin(θ2 − θ1)
j22 = L4 cos(polar − θ1) sin(θ2 − β) / (radius sin(θ2 − θ1))
```

我对 α=135°、β=45° 数值核对：`j11=−97.24`，`j12=0.3913`，正好是 $J_p^\top$ 的第一行（mm/rad 与 rad/rad 单位）。也就是说，**第九课的 VMC 就是本章第 6 节的虚拟腿 VMC**（$F$ 用 `48.755/cosθ1 + yError` 作支撑力：前一项是某个重力前馈，来源源码未注明）；第十一课改成了更易整定的笛卡尔弹簧阻尼形式。

> **提示**：两种形式在数学上等价，区别在"你把期望力定义在哪个坐标系"。你的项目（CV）按虚拟腿描述，所以对外讲 $(F,T_p)$ 形式即可。

---

## 9. 条件数、奇异性、单位与镜像

### 9.1 雅可比条件数（课程杆长）

对称姿态 $\alpha=180^\circ-b$，$\beta=b$：

| b（度） | 腿高 $y_B$（mm） | $\sin(\theta_1-\theta_2)$ | cond(J) |
|---|---|---|---|
| 5 | 158.4 | −0.946 | 1.40 |
| 10 | 173.7 | −0.953 | 1.37 |
| 30 | 244.3 | −0.997 | 1.09 |
| 45 | 298.1 | −0.984 | 1.20 |
| 60 | 344.1 | −0.884 | 1.66 |
| 75 | 377.0 | −0.690 | 2.50 |
| 90 | 394.1 | −0.422 | 4.52 |

**结论**：课程杆长下，$d\le L_5+2L_1=408<L_2+L_3=500$，所以被动杆 AB、CB **不可能共线**，$\sin(\theta_1-\theta_2)$ 最小只有 −0.42；课程使用的腿高范围（约 140–300 mm）内 cond(J)≈1.1–1.4，**雅可比条件极好**，力矩映射很稳定。真正会变坏的是腿**接近完全伸直**（OA 与 AB 共线，约 400 mm 处，$\sin(\alpha-\theta_1)\to0$，$\det J\to0$），此时电机无法再沿腿方向改变 B 的位置。

你实机杆长不同，结论要重新算：先算 cond(J) 随腿长的曲线，再决定工作范围。

### 9.2 单位

- 课程杆长 mm，$J$ 为 mm/rad，$F$ 为 N → 力矩 = $J\cdot F/1000$ N·m；
- 你的实机若用 m，则 $J$ 为 m/rad，直接 $\boldsymbol\tau=J^\top\mathbf F$，无需 /1000；
- **在代码里统一单位并写进注释，是避免"力矩差 1000 倍"的最便宜办法**；
- 经过 MIT 协议发送时，力矩被限幅并量化到 12 位（课程范围 ±18），第 10 章。

### 9.3 左右镜像

左右腿机构镜像，电机安装正方向相反：右腿用 `j` 的一套符号，左腿的 `front/back` 对应的电机不同。**每条腿单独标定零位和符号**，并用第 3 章 7 节的核对清单验证"$F>0$ 时腿是否伸出、$T_p>0$ 时腿是否朝预期方向摆"。

---

## 10. 常见错误与故障现象

| 现象 | 原因 | 怎么查 |
|---|---|---|
| 给 $F>0$，腿反而缩 | 关节力矩符号或 $J$ 的符号与电机正方向相反 | 用 $F=$ 小常数、$T_p=0$ 单腿测试 |
| 站立时一条腿软、一条腿硬 | 左右零位/镜像不同，$J$ 在两腿用了同一套符号 | 分别标定；对比两腿的 $J$ |
| 力矩"差 1000 倍" | mm 与 m 混用 | 统一单位，加量纲检查 |
| 腿长大时出现力矩尖峰 | 接近完全伸直，$J$ 条件变坏；或 FK 异常 | 限制最大腿长；监控 cond(J) |
| 腿端抖动 | 阻尼项用的是噪声很大的足端速度 | 用 $J\dot{\mathbf q}$ 算速度并低通 |
| 跳变 | FK 解支翻转 | 第 2 章解支选择 |

---

## 11. 面试怎么问、怎么答

**Q1. What is VMC, and why did you use it for the legs?**

> Virtual model control means I decide what force or torque the leg should produce, as if a virtual spring and damper were attached to it, and then I convert that into joint torques with the Jacobian transpose. I used it because my balance controller outputs a hip torque and my leg-length controller outputs a support force, so everything is naturally in force space. It also makes the leg compliant, so the body is less shaken by the ground, which is hard to get with pure position control through inverse kinematics.

**Q2. Why is the torque the Jacobian transpose and not the inverse?**

> By virtual work or power conservation. The Jacobian maps joint velocities to foot velocities. The power at the foot is force times foot velocity, and the power at the joints is torque times joint velocity. Setting them equal for any joint velocity gives tau equals J transpose times F. It needs no inversion, so it stays well defined even near singular poses.

追问：What happens near a singularity? → J 的元素变大；$J^\top F$ 仍有限，但力矩对力的放大倍数变化；还可能失去某个方向的力矩能力。

**Q3. How do you map the support force and the hip torque to the two joint torques?**

> I define the generalized forces on the virtual leg coordinates, the leg length and the leg angle. The support force acts along the leg and the hip torque acts about the hip joint. I compose the Jacobian of the foot with the polar transformation from the foot position to leg length and leg angle, and then transpose it. Equivalently, I turn the hip torque into a tangential force at the foot, add the radial support force, and apply the foot Jacobian transpose.

**Q4. Can you give a numerical sense of the joint torques?**

> As a teaching number, with a leg about 0.3 meters long in a symmetric pose, the Jacobian entry is about 0.097 meters per radian, so a support force of 30 newtons needs about 2.9 newton-meters on each hip joint. I don't have the exact numbers of our robot memorized, but I computed this kind of estimate to check that the motors and the torque limits were adequate.

（诚实提示：这是教学估计；实机数值请补全 README 自查表。）

---

## 12. 本章要点速记

1. VMC：期望的**力**（虚拟弹簧阻尼）→ $J^\top$ → 关节力矩；与 IK 位置控制相反。
2. 足端雅可比来自"杆长不变"：沿杆方向速度相等，得 $M_c\dot{\mathbf p}=D_c\dot{\mathbf q}$，$J=M_c^{-1}D_c$。
3. 四个系数：$j_{11}=\frac{L_1\sin\theta_2\sin(\alpha-\theta_1)}{\sin(\theta_1-\theta_2)}$ 等，右边是主动杆长 $L_1,L_4$。
4. $\boldsymbol\tau=J^\top\mathbf F$（虚功/功率守恒）；$J$ 转速度，$J^\top$ 转力。
5. 虚拟腿 VMC：$\boldsymbol\tau=J_p^\top[F,T_p]^\top$，$J_p=J_rJ$；等价于"径向力 + 切向力 $T_p/L_0$"再 $J^\top$。
6. 对称姿态：支撑力 → 关节力矩**差模**，$T_p$ → **共模**；α=135°、β=45° 时 $J_p^\top$ 第一列 $(-0.097,0.097)$，第二列 $(0.391,0.391)$。
7. 静力：每腿约 $Mg/2$，每个髋约 2.9 N·m（教学数值）。
8. 课程第九课是极坐标 VMC，第十一课是笛卡尔 VMC；二者等价。
9. 课程杆长下 cond(J) 在工作范围内≈1.1–1.4，接近完全伸直才变坏。
10. 单位（mm vs m）、左右镜像、符号是三大坑。

## 自测题

1. 推导 $\boldsymbol\tau=J^\top\mathbf F$。（答案见第 5 节。）
2. 为什么 $\dot\alpha$ 方程右边是 $L_1$ 而不是 $L_2$？（答：A 点的圆周速度半径是 $L_1$；$L_2$ 在投影中约掉。）
3. 对称姿态下，$F>0$ 与 $T_p>0$ 分别对应两个关节力矩的什么模式？（答：差模；共模。）
4. 机身 6 kg，一条腿 $J_{21}=-0.0972$ m/rad，求静态支撑的关节力矩。（答：约 2.9 N·m。）
5. 课程里的 `/1000` 从哪来？（答：$J$ 的 mm 单位到 m 的换算，推算而得。）

## 相关链接

- 上一章：[05_LQR原理与增益调度.md](./05_LQR原理与增益调度.md)
- 下一章：[07_腿长PID_力前馈与姿态协调.md](./07_腿长PID_力前馈与姿态协调.md)
- 基础：[正逆运动学与雅可比](../../02_基础知识/03_坐标变换与运动学/03_正逆运动学与雅可比.md)、[奇异性、阻尼最小二乘、零空间](../../02_基础知识/03_坐标变换与运动学/04_奇异性_阻尼最小二乘_零空间.md)
- 八股：[运动学动力学八股](../../03_八股知识/03_机器人学/01_运动学动力学八股.md)
- 相关项目：[P2 双臂遥操作（正逆解与雅可比）](../P2_未来动力双臂遥操作与差速底盘/README.md)
- 笔记原文：[P7 逐字稿](../../05_学习笔记归档/P7/P7_本对话问答逐字稿.md)（第 23–26 组）
