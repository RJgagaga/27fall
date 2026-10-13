# DH 参数、指数积（POE）与旋量理论

> **一句话定位**：把一台串联机械臂的几何写成一串矩阵乘法有两种经典方式：Denavit–Hartenberg（DH）参数用"每个关节四个数"逐关节建系，指数积（product of exponentials，POE）公式用"每个关节一根螺旋轴"在统一的基座系里描述。它们等价，但 POE 与旋量（twist）理论让雅可比、速度、力的推导干净得多——也正是 URDF、KDL、Pinocchio 这些工具在背后做的事。

**本章学完你能：**

- 说清标准 DH 与改进 DH（modified DH）四个参数的几何含义、建系规则和变换矩阵，并对一个二连杆手算 DH 表与正运动学；
- 解释旋量 $\mathcal V=(\omega,v)$、螺旋轴 $\mathcal S=(\omega,v)$ 的定义，会由关节轴方向和轴上一点求出 $\mathcal S$；
- 推导 POE 公式 $T(\theta)=e^{[\mathcal S_1]\theta_1}\cdots e^{[\mathcal S_n]\theta_n}M$，并对二连杆与三自由度空间臂验证它与 DH 结果一致（Python 复核）；
- 区分空间雅可比（space Jacobian）与体雅可比（body Jacobian），用伴随矩阵在两者间转换，并手算二连杆的两个雅可比、末端速度与奇异条件；
- 说出 URDF 的 `origin` + `axis` 与 DH/POE 的对应关系，知道为什么 URDF 不需要 DH 参数；
- 用英文回答 "DH vs POE / what is a screw axis / space vs body Jacobian" 等问题。

**前置知识**：[坐标系、旋转矩阵、齐次变换](./01_坐标系_旋转矩阵_齐次变换.md)、[正逆运动学与雅可比](./03_正逆运动学与雅可比.md)、[李群李代数与流形优化入门](../01_数学基础/06_李群李代数与流形优化入门.md)（$\exp$、$J_l$、伴随，建议先读）。

## 目录

1. 问题：怎样系统地描述一串关节
2. 符号与约定
3. 标准 DH 参数
4. 改进 DH 参数
5. 手算：平面二连杆的 DH
6. 旋量与螺旋轴
7. 指数积公式（POE）
8. 手算与验证：二连杆与三自由度空间臂
9. 空间雅可比与体雅可比
10. 手算：二连杆的雅可比
11. DH、POE 与 URDF/KDL 的对应
12. 在本项目里怎么用
13. 常见错误与故障现象
14. 面试怎么问、怎么答
15. 本章要点速记
16. 自测题
17. 相关链接

---

## 1. 问题：怎样系统地描述一串关节

串联机械臂的正运动学，就是计算"从基座到末端"的刚体变换 $T_{0n}(q)$。直接写 $T_{01}T_{12}\cdots T_{n-1,n}$ 当然可以，但每个 $T_{i-1,i}$ 里该放几个参数？一个任意的刚体变换有 6 个自由度，而相邻两个关节之间的几何关系（两根轴的相对位置）只需要 **4 个参数**来描述，因为关节轴只是直线，沿轴平移和绕轴旋转本身就是关节的"自由度"，可以与几何参数分开。这就是 DH 参数的思想：

1. 每个关节建一个坐标系，让 $z$ 轴沿关节轴；
2. 相邻两个坐标系之间的变换，通过四个有明确几何意义的参数来表达（其中一个是关节变量，另三个是常数）。

然而 DH 也有弱点：建系规则繁琐（相邻轴平行或相交时有歧义）、两种约定（标准与改进）常被混用、每个关节的坐标系位置依赖于相邻轴的公垂线，不直观。20 世纪 80 年代之后，旋量理论给了另一条路：**只在一个固定的基座系里，对每个关节写出它的螺旋轴 $\mathcal S_i$**（零位时的轴线位置与方向），再把"每个关节绕自己的轴转 $\theta_i$"表示成 $e^{[\mathcal S_i]\theta_i}$ 连乘。这就是 POE。它不需要为每个关节建局部坐标系，也不会有 DH 的歧义，而且所得的雅可比有简洁的闭式。

## 2. 符号与约定

| 符号 | 含义 | 单位 |
|---|---|---|
| $a_i$ | 连杆长度：沿公垂线（$x$ 轴）方向的距离 | m |
| $\alpha_i$ | 连杆扭角：绕 $x$ 轴把 $z_{i-1}$ 转到 $z_i$ 的角度 | rad |
| $d_i$ | 连杆偏距：沿 $z_{i-1}$ 的距离 | m |
| $\theta_i$ | 关节角：绕 $z_{i-1}$ 的转角（转动关节为变量） | rad |
| $\omega\in\mathbb{R}^3$ | 转轴单位向量（对旋量，$\lVert\omega\rVert=1$ 为"螺旋轴"，否则含速度） | 无量纲或 rad/s |
| $q\in\mathbb{R}^3$ | 轴上任意一点 | m |
| $v=-\omega\times q$ | 螺旋轴的线速度部分 | m（对单位轴） |
| $\mathcal S=(\omega,v)\in\mathbb{R}^6$ | 螺旋轴（screw axis） | |
| $[\mathcal S]\in\mathfrak{se}(3)$ | 旋量的 $4\times4$ 矩阵形式 | |
| $M$ | 零位（$\theta=0$）时末端相对基座的位姿 | |
| $\mathrm{Ad}_T$ | 伴随矩阵 | $6\times6$ |

**排序约定**：本章旋量按**角速度在前、线速度在后** $(\omega,v)$ 排列（Lynch 与 Park 的《Modern Robotics》采用的顺序），与 [李群李代数章](../01_数学基础/06_李群李代数与流形优化入门.md) 里 $\xi=(\rho,\phi)$ 的顺序**相反**。伴随矩阵相应为

$$\mathrm{Ad}_T=\begin{bmatrix}R&0\\p^\wedge R&R\end{bmatrix}\quad(\omega\text{ 在前}),$$

而 $\xi=(\rho,\phi)$ 顺序下它是 $\begin{bmatrix}R&p^\wedge R\\0&R\end{bmatrix}$。**换库、换书必须先看顺序**，否则雅可比分块对调，结果全错。

## 3. 标准 DH 参数

### 3.1 建系规则

对第 $i$ 个关节（转动或移动），沿用经典 Denavit–Hartenberg 规则：

1. $z_{i-1}$ 轴沿关节 $i$ 的运动轴（转动：转轴；移动：移动方向）；
2. $x_i$ 轴沿 $z_{i-1}$ 与 $z_i$ 的公垂线，指向远离 $z_{i-1}$ 的方向；若两轴相交，$x_i$ 取 $z_{i-1}\times z_i$ 方向；若两轴平行，公垂线不唯一，可取便于计算的一条；
3. $y_i$ 由右手定则确定。

### 3.2 四个参数的几何含义

从坐标系 $\{i-1\}$ 变到 $\{i\}$：

1. 绕 $z_{i-1}$ 转 $\theta_i$（使 $x_{i-1}$ 对齐到公垂线方向）；
2. 沿 $z_{i-1}$ 平移 $d_i$（沿轴滑到公垂线的垂足处）；
3. 沿新的 $x$ 轴（公垂线）平移 $a_i$；
4. 绕新的 $x$ 轴转 $\alpha_i$（把 $z_{i-1}$ 转到 $z_i$）。

### 3.3 变换矩阵

$$ {}^{i-1}T_i=\mathrm{Rot}_z(\theta_i)\,\mathrm{Trans}_z(d_i)\,\mathrm{Trans}_x(a_i)\,\mathrm{Rot}_x(\alpha_i)=\begin{bmatrix}c\theta_i&-s\theta_ic\alpha_i&s\theta_is\alpha_i&a_ic\theta_i\\s\theta_i&c\theta_ic\alpha_i&-c\theta_is\alpha_i&a_is\theta_i\\0&s\alpha_i&c\alpha_i&d_i\\0&0&0&1\end{bmatrix}.$$

这里 $c$、$s$ 表示 $\cos$、$\sin$。

### 3.4 缺点

- 对一般的 6 轴臂，需要仔细按规则建系，易错；
- 相邻轴平行时，公垂线位置任意，这种"奇异的 DH 参数化"使得某些参数（如 $d$）出现跳变，对标定不友好；
- 最后一个关节到末端工具坐标系需额外补一个固定变换，规则并不统一。

## 4. 改进 DH 参数

改进（modified，也称 Craig）DH 把坐标系固定在关节轴所在位置而不是连杆末端，变换顺序改为

$${}^{i-1}T_i=\mathrm{Rot}_x(\alpha_{i-1})\,\mathrm{Trans}_x(a_{i-1})\,\mathrm{Rot}_z(\theta_i)\,\mathrm{Trans}_z(d_i).$$

两种约定的区别在于：**标准 DH 里 $a_i,\alpha_i$ 属于连杆 $i$ 的末端；改进 DH 里 $a_{i-1},\alpha_{i-1}$ 属于关节 $i$ 的起点**（下标错开一位）。若平行轴、末端工具、标定等场景，改进 DH 更方便；很多教材和厂商文档使用的是标准 DH。**对同一台机械臂，两种约定得到的 DH 表数值不同，但最终的 $T_{0n}$ 相同**，所以看到一张 DH 表，第一件事是确认它用的是哪种约定。

## 5. 手算：平面二连杆的 DH

平面二连杆，连杆长度 $L_1=0.4$ m，$L_2=0.3$ m，关节角 $\theta_1=30^\circ$，$\theta_2=60^\circ$（示例，非项目实测）。关节轴都沿 $z$，$\alpha=0$，$d=0$。

| 连杆 $i$ | $\theta_i$ | $d_i$ | $a_i$ | $\alpha_i$ |
|---|---|---|---|---|
| 1 | $\theta_1$ | 0 | $L_1$ | 0 |
| 2 | $\theta_2$ | 0 | $L_2$ | 0 |

代入标准 DH 变换 ${}^{0}T_1$、${}^{1}T_2$，相乘得末端位姿：

$$T_{02}=\begin{bmatrix}\cos(\theta_1+\theta_2)&-\sin(\theta_1+\theta_2)&0&L_1c_1+L_2c_{12}\\\sin(\theta_1+\theta_2)&\cos(\theta_1+\theta_2)&0&L_1s_1+L_2s_{12}\\0&0&1&0\\0&0&0&1\end{bmatrix}.$$

数值：$x=0.4\cos30^\circ+0.3\cos90^\circ=0.34641$，$y=0.4\sin30^\circ+0.3\sin90^\circ=0.5$（Python 复核）。姿态是绕 $z$ 转 $90^\circ$。改进 DH 的表为 $(\alpha_0,a_0,\theta_1,d_1)=(0,0,\theta_1,0)$、$(\alpha_1,a_1,\theta_2,d_2)=(0,L_1,\theta_2,0)$，再加一行固定的工具变换 $(0,L_2,0,0)$，Python 复核 $T_{02}$ 与上式**完全一致**。

## 6. 旋量与螺旋轴

### 6.1 刚体速度的一般形式

沙勒（Chasles）定理：刚体的任意运动都可以表示为**沿某条直线（螺旋轴）的平移与绕该直线的转动的组合**。因此描述一个刚体的瞬时运动，需要六个数：角速度 $\omega$ 和某参考点处的线速度 $v$，合称**旋量（twist）** $\mathcal V=(\omega,v)$。把它写成 $4\times4$ 矩阵：

$$[\mathcal V]=\begin{bmatrix}[\omega]_\times&v\\0&0\end{bmatrix}\in\mathfrak{se}(3).$$

### 6.2 螺旋轴

单位螺旋轴 $\mathcal S=(\omega,v)$，满足 $\lVert\omega\rVert=1$（转动关节）或 $\omega=0,\lVert v\rVert=1$（移动关节）。对转动关节，若转轴方向 $\omega$，轴上一点 $q$：

$$\boxed{v=-\omega\times q.}$$

直觉：$v$ 是"刚体上与基座原点重合的那个点"随这根轴转动时的速度。若轴经过原点，$q=0\Rightarrow v=0$。

### 6.3 指数映射（回顾）

绕螺旋轴 $\mathcal S$ 转动 $\theta$ 的刚体变换：

$$e^{[\mathcal S]\theta}=\begin{bmatrix}e^{[\omega]\theta}&\big(I\theta+(1-\cos\theta)[\omega]+(\theta-\sin\theta)[\omega]^2\big)v\\0&1\end{bmatrix}.$$

右上角平移项等价于 $(I-e^{[\omega]\theta})q$（因为绕过 $q$ 的轴转动时，$q$ 本身不动：$p'=R(p-q)+q$）。**验证**：二连杆第二关节轴过 $q=(0.4,0,0)$，$\omega=(0,0,1)$，则 $v=-\omega\times q=(0,-0.4,0)$。取 $\theta=60^\circ$：$(I-R_z(60^\circ))q=(0.2,-0.3464,0)$；按带 $v$ 的公式展开也得 $(0.2,-0.3464,0)$（手算一致，Python 复核）。这印证了第 6.2 节的 $v=-\omega\times q$。

## 7. 指数积公式（POE）

### 7.1 推导

设机器人在零位（所有 $\theta_i=0$）时，末端相对基座的位姿为 $M$。我们想知道关节转动后末端的位姿。

1. 只转关节 $n$（最靠近末端）：末端相当于一个刚体绕关节 $n$ 的轴（在基座系中的位置固定）转动 $\theta_n$。把"绕固定轴转动"作用在末端上，有 $T=e^{[\mathcal S_n]\theta_n}M$。这里螺旋轴 $\mathcal S_n$ 用零位时的轴线定义，并且在**基座系（空间系）**中表示；
2. 再转关节 $n-1$：关节 $n-1$ 之后的整个结构（含关节 $n$ 的轴和末端）绕关节 $n-1$ 的轴转动，所以在左边再乘 $e^{[\mathcal S_{n-1}]\theta_{n-1}}$：$T=e^{[\mathcal S_{n-1}]\theta_{n-1}}e^{[\mathcal S_n]\theta_n}M$；
3. 一直到关节 1：

$$\boxed{T(\theta)=e^{[\mathcal S_1]\theta_1}e^{[\mathcal S_2]\theta_2}\cdots e^{[\mathcal S_n]\theta_n}M}$$

关键在于**每个 $\mathcal S_i$ 都是零位下、在基座系中的螺旋轴，与其他关节角无关**。DH 公式里每个关节的坐标系会随前面关节转动，而 POE 里关节"逐个由外向内"转动，不需要这种局部坐标。这是 POE 的核心概念优势。

### 7.2 体形式

若改用"末端坐标系中的螺旋轴" $\mathcal B_i=\mathrm{Ad}_{M^{-1}}\mathcal S_i$，则 $T(\theta)=Me^{[\mathcal B_1]\theta_1}\cdots e^{[\mathcal B_n]\theta_n}$。空间形式适合"从基座看"，体形式适合"从末端看"（如末端传感器、视觉伺服）。

## 8. 手算与验证：二连杆与三自由度空间臂

### 8.1 二连杆

零位 $M=\begin{bmatrix}I&(L_1+L_2,0,0)^\top\\0&1\end{bmatrix}$。关节 1 的轴过原点：$\omega_1=(0,0,1)$，$q_1=0\Rightarrow\mathcal S_1=(0,0,1,0,0,0)$。关节 2 的轴过 $q_2=(L_1,0,0)$：$v_2=-\omega\times q_2=(0,-L_1,0)$，故 $\mathcal S_2=(0,0,1,0,-0.4,0)$。代入 $\theta_1=30^\circ,\theta_2=60^\circ$：

$$T=e^{[\mathcal S_1]\theta_1}e^{[\mathcal S_2]\theta_2}M=\begin{bmatrix}0&-1&0&0.34641\\1&0&0&0.5\\0&0&1&0\\0&0&0&1\end{bmatrix}$$

与第 5 节 DH 结果逐元素相同（Python 复核）。注意 POE **没有建任何局部坐标系**，只用了"零位轴线 + 零位末端位姿"。

### 8.2 三自由度空间臂（偏航—俯仰—俯仰）

模型：第 1 关节绕竖直 $z$ 轴；第 2 关节轴水平，位于高度 $L_0=0.2$ m；第 3 关节在第 2 关节前方 $L_1=0.4$ m 处，轴方向平行第 2 关节；末端在第 3 关节前方 $L_2=0.3$ m 处。取 DH 建系使第二、第三关节轴为 $-y$ 方向，则零位末端位姿 $M=\mathrm{Rot}_x(90^\circ)$ 与位置 $(L_1+L_2,0,L_0)$。

- $\mathcal S_1=(0,0,1,0,0,0)$；
- $\mathcal S_2$：$\omega=(0,-1,0)$，$q=(0,0,L_0)$，$v=-\omega\times q=(0.2,0,0)$，即 $\mathcal S_2=(0,-1,0,0.2,0,0)$；
- $\mathcal S_3$：$\omega=(0,-1,0)$，$q=(L_1,0,L_0)$，$v=(0.2,0,-0.4)$，即 $\mathcal S_3=(0,-1,0,0.2,0,-0.4)$。

DH 表（标准）：$(\theta_1,d_1,a_1,\alpha_1)=(\theta_1,L_0,0,90^\circ)$；$(\theta_2,0,L_1,0)$；$(\theta_3,0,L_2,0)$。取 $\theta=(30^\circ,40^\circ,-50^\circ)$，两种方法得到

$$T=\begin{bmatrix}0.85287&0.15038&0.5&0.52123\\0.4924&0.08682&-0.86603&0.30093\\-0.17365&0.98481&0&0.40502\\0&0&0&1\end{bmatrix},$$

两种方法的最大元素差 $1.1\times10^{-16}$（Python 复核）。这个例子说明：**同一台机器，DH 需要 3 张局部坐标系和一个 90° 扭角；POE 只需要在基座系里读出三根轴的方向和各一个点**，更适合从机械图纸或 CAD 直接读数。

## 9. 空间雅可比与体雅可比

### 9.1 空间雅可比

末端的空间速度旋量（在基座系下表示，等价于"与基座原点重合的刚体点的速度 + 角速度"）可写成 $\mathcal V_s=J_s(\theta)\dot\theta$。对 POE 求导，利用 $e^{[\mathcal S]\theta}$ 的性质可得每一列：

$$J_{s,1}=\mathcal S_1,\qquad J_{s,i}=\mathrm{Ad}_{e^{[\mathcal S_1]\theta_1}\cdots e^{[\mathcal S_{i-1}]\theta_{i-1}}}\mathcal S_i\quad(i\ge2).$$

**推导思路**：关节 $i$ 单独转动时，其影响相当于"绕关节 $i$ 当前的轴线转动"。关节 $i$ 的当前轴线，是零位轴线 $\mathcal S_i$ 被前面 $i-1$ 个关节的运动"搬运"之后的位置，而搬运一个旋量用的正是伴随矩阵。

### 9.2 体雅可比

$\mathcal V_b=J_b(\theta)\dot\theta$，$\mathcal V_b$ 在末端坐标系下表示。两者的关系：

$$\mathcal V_b=\mathrm{Ad}_{T_{bs}}\mathcal V_s,\qquad J_b=\mathrm{Ad}_{T^{-1}}J_s,\qquad J_s=\mathrm{Ad}_{T}J_b.$$

其中 $T=T_{sb}(\theta)$ 是末端相对基座的位姿，$T_{bs}=T^{-1}$。体形式的优势：同样的 $J_b$ 可以直接乘末端传感器测到的速度命令（如视觉伺服里的相机系速度）。

### 9.3 与"几何雅可比"的关系

课本上常见的雅可比 $J_v,J_\omega$（线速度雅可比与角速度雅可比）实际是"末端点"的线速度，而非空间旋量里的 $v$。关系：末端点 $p$ 的线速度 $\dot p=v_s+\omega_s\times p$，即

$$J_{p}=J_{s,v}-[p]_\times J_{s,\omega}.$$

这是 POE 方法与初等教材的雅可比之间的桥梁。

### 9.4 奇异性

雅可比秩降低（$\mathrm{rank}\,J<\min(6,n)$）即奇异位形，物理上是某个方向的运动无法产生。用 $J_s$ 和 $J_b$ 判断奇异性等价，因为两者只差一个可逆的伴随矩阵。条件数、可操作度 $\sqrt{\det(JJ^\top)}$ 等指标的使用见 [奇异性、阻尼最小二乘、零空间](./04_奇异性_阻尼最小二乘_零空间.md)。

## 10. 手算：二连杆的雅可比

继续用二连杆，$\theta=(30^\circ,60^\circ)$：

**空间雅可比。** 第 1 列 $J_{s,1}=\mathcal S_1=(0,0,1,0,0,0)$。第 2 列需要把 $\mathcal S_2$ 经 $T_1=e^{[\mathcal S_1]\theta_1}$ 搬运：$J_{s,2}=\mathrm{Ad}_{T_1}\mathcal S_2$。直接数值（Python 复核）：

$$J_s=\begin{bmatrix}0&0\\0&0\\1&1\\0&0.2\\0&-0.34641\\0&0\end{bmatrix}.$$

物理检验：第二关节轴当前位于 $(L_1\cos30^\circ,L_1\sin30^\circ)=(0.34641,0.2)$，故 $v=-\omega\times q=-(0,0,1)\times(0.34641,0.2,0)=(0.2,-0.34641,0)$，与第 2 列的线速度部分一致。

**体雅可比。** $J_b=\mathrm{Ad}_{T^{-1}}J_s$：

$$J_b=\begin{bmatrix}0&0\\0&0\\1&1\\0.34641&0\\0.5&0.3\\0&0\end{bmatrix}.$$

对应物理：末端系下，第二关节轴离末端 $L_2=0.3$ m，故 $v_2=(0,0.3,0)$。Python 验证 $J_s=\mathrm{Ad}_TJ_b$，最大误差 $3\times10^{-17}$。

**末端速度。** 取 $\dot\theta=(1,2)$ rad/s：$\mathcal V_s=(0,0,3,0.4,-0.69282,0)$，$\mathcal V_b=(0,0,3,0.34641,1.1,0)$。末端点线速度 $=v_s+\omega_s\times p=(0.4,-0.69282)+3\,\hat z\times(0.34641,0.5)=(0.4-1.5,\ -0.69282+1.03923)=(-1.1,\ 0.34641)$ m/s；用平面二连杆的经典 $J_v$ 乘 $\dot\theta$ 得 $(-1.1,0.34641)$，一致（Python 复核）。

**奇异性。** 经典 $J_v$ 的行列式 $\det J_v=L_1L_2\sin\theta_2=0.4\times0.3\times\sin60^\circ=0.10392$（Python 复核）；$\theta_2=0$ 或 $\pi$（手臂完全伸直或折叠）时为零，是奇异位形。

### 10.1 移动关节与螺旋运动

移动关节没有转动，螺旋轴 $\mathcal S=(0,v)$，$v$ 是单位移动方向，指数映射退化为 $e^{[\mathcal S]\theta}=\begin{bmatrix}I&v\theta\\0&1\end{bmatrix}$，$\theta$ 此时是位移量（单位 m）。若转动同时带沿轴的平移，就是一般螺旋运动，其节距（pitch）$h=\omega^\top v/\lVert\omega\rVert^2$ 表示转一圈沿轴前进的距离，例如丝杠。纯转动 $h=0$，纯平移 $h=\infty$。这给出一个统一的视角：转动关节、移动关节、丝杠关节都是螺旋轴的特例，在 POE 公式里不需要区别对待，雅可比的列公式完全相同。

### 10.2 参数误差与标定

制造和装配误差会使真实的几何参数偏离名义值。把 POE 对第 $j$ 个关节的螺旋轴参数求偏导，可以得到末端位姿对几何误差的灵敏度矩阵（误差雅可比），再用若干个测量位姿（激光跟踪仪或相机标定板测得）做最小二乘，反解出参数修正量，这就是运动学标定（kinematic calibration）。POE 在这件事上优于 DH：POE 的螺旋轴参数在平行轴、相邻轴接近平行时依然连续，而 DH 的参数化在此处会奇异，这也是 DH 的 $d$ 和 $\theta$ 在平行轴上会"此消彼长"、难以独立辨识的原因。对于实际项目，这意味着：如果机械臂的末端精度不够，首先怀疑 URDF 的 `origin` 数值（来源于 CAD 名义值）与真实装配偏差，而不是控制器。

### 10.3 手腕与逆运动学的几何前提

大多数 6 轴工业臂的最后三个关节轴相交于一点（球形手腕），使得逆运动学可解耦成"前三轴决定腕心位置、后三轴决定姿态"，有闭式解（多解，一般 8 组）。POE 里这个结构体现为后三根螺旋轴的轴线共点。不满足该条件的构型（例如很多协作臂的偏置手腕）没有通用闭式解，需要数值迭代求逆（雅可比迭代或阻尼最小二乘），这也是 P2 里使用基于雅可比的约束 IK 而不是闭式解的现实原因之一。

## 11. DH、POE 与 URDF/KDL 的对应

| 概念 | DH | POE | URDF / KDL / Pinocchio |
|---|---|---|---|
| 参数 | 每关节 $(\theta,d,a,\alpha)$ | 每关节 $\mathcal S_i$ + 总的 $M$ | 每关节：父→子的固定变换（`origin xyz rpy`）+ 关节轴 `axis` |
| 坐标系 | 每关节一个，规则严格 | 只用基座系 | 每个 link 一个坐标系，原点可任意放 |
| 关节变量 | 嵌入变换（$\theta$ 或 $d$） | 指数映射的角度 | 关节变换 $\mathrm{Rot}(\text{axis},q)$ 叠加在固定变换之后 |
| 优点 | 参数少，历史悠久 | 无歧义，雅可比简洁，易从 CAD 读 | 通用，可描述树形与多分支 |
| 缺点 | 建系规则繁琐，平行轴歧义 | 需要选定零位 | 不是最小参数集，冗余但直观 |

URDF 的每个关节包含两部分：（1）父连杆坐标系到关节坐标系的固定变换 $T_{\text{fixed}}$（`origin`）；（2）关节自身绕 `axis`（在关节坐标系中给出）转动 $q$：$T_{\text{parent→child}}=T_{\text{fixed}}\cdot\mathrm{Rot}(\text{axis},q)$。只要把各关节的 $T_{\text{fixed}}$ 连乘并把关节变换插入，就得到 $T_{0n}(q)$。对比 POE：URDF 是"一个关节一个局部坐标系 + 固定变换"，POE 是"把全部固定变换预先合并成零位下的轴线"。两者是同一个几何的两种分解方式。KDL 的 `Segment`（由 `Joint` 与固定的 `Frame` 组成）与 URDF 完全同构；Pinocchio 同样基于 URDF 的关节树，内部用 $SE(3)$ 的旋量运算（空间代数）高效计算雅可比与动力学。

由此可得一个重要实践结论：**你不必手写 DH 表**；拿到 URDF，调用库的 `forward_kinematics` 与 `compute_jacobian` 即可。但 DH/POE 的知识不会白费——它让你能看懂 CAD 标定、论文里的公式，也能在 URDF 出错（轴方向写反、`origin` 漏旋转）时靠手算推导出"应该是什么"。

## 12. 在本项目里怎么用

> 说明：CV 中的 P2 使用 URDF/KDL 与 MuJoCo/Mink，没有手写 DH 表。下文说明概念如何落到项目；面试时应表述为"理解 DH/POE，实际使用 URDF 驱动的库"。

- **P2 双臂**：Mink 的任务雅可比、KDL 的雅可比和重力补偿，内部使用的就是树形关节的空间代数；手柄给的末端目标位姿，通过 $\mathrm{Log}(T_{\text{target}}T_{\text{cur}}^{-1})$ 变成 6 维误差旋量，乘雅可比伪逆得到关节速度（见 [逆运动学、雅可比、奇异性与阻尼最小二乘](../../01_项目课程/P2_未来动力双臂遥操作与差速底盘/04_逆运动学_雅可比_奇异性与阻尼最小二乘.md)、[URDF/KDL 动力学与重力补偿](../../01_项目课程/P2_未来动力双臂遥操作与差速底盘/06_URDF_KDL动力学与重力补偿.md)）。用空间雅可比还是体雅可比，决定了误差旋量要表示在世界系还是末端系——**误差与雅可比必须在同一个坐标系里**。
- **P1 质心前馈**：由臂关节状态通过 URDF/Pinocchio 算质心，其核心也是各 link 的位姿链乘（[质心前馈补偿](../../01_项目课程/P1_AIRS球形机器人平衡控制/07_质心前馈补偿.md)）。
- **P7 五连杆**：闭链结构不能直接套串联 DH/POE，需要先写闭环约束（[闭链并联机构与五连杆运动学](./05_闭链并联机构与五连杆运动学.md)），但单条支链内部仍可用 POE。
- **P6 坐标系**：设备抽象里的坐标系链与 TF 同样是 $SE(3)$ 连乘，只是没有关节变量（[URDF、TF、坐标系实践](../12_机器人软件/07_URDF_TF_坐标系实践.md)）。

## 13. 常见错误与故障现象

| 现象 | 可能原因 | 处理 |
|---|---|---|
| 同一 DH 表两个人算出不同结果 | 标准/改进约定混用 | 先问清约定；用 $T_{02}$ 数值互验 |
| 末端位置对但姿态差固定旋转 | 忽略工具坐标系或零位 $M$ 的姿态 | POE 中 $M$ 必须含姿态（如 $\mathrm{Rot}_x(90^\circ)$） |
| 螺旋轴线速度部分符号错 | $v=\omega\times q$ 与 $-\omega\times q$ 弄反 | 用"过 $q$ 的轴转动后 $q$ 不动"自检 |
| 雅可比的角、线分块对调 | 旋量顺序 $(\omega,v)$ 与 $(v,\omega)$ 混用 | 同一工程固定顺序；换库先读文档 |
| 空间雅可比乘末端系误差 | 误差与雅可比不在同一坐标系 | 统一为空间或体，或用伴随转换 |
| URDF 关节轴方向反了，实机转动反向 | `axis` 符号或父子 `origin` 旋转写错 | 用已知关节角对比末端位姿 |
| 平行轴 DH 的参数在标定时发散 | 平行轴的 DH 参数化奇异 | 用改进 DH、POE 或添加辅助参数 |
| 奇异附近关节速度暴增 | 雅可比近奇异 | 阻尼最小二乘；见奇异性章节 |

## 14. 面试怎么问、怎么答

**Q1. What is the difference between standard and modified DH parameters?**

> Both describe each joint with four parameters, but the order of the elementary transforms differs. In standard DH the transform is Rot-z, Trans-z, Trans-x, Rot-x, and the link parameters belong to the end of the link. In modified DH it is Rot-x, Trans-x, Rot-z, Trans-z, and the link parameters belong to the start of the joint. The DH tables look different but give the same end-effector pose, so I always check which convention a table uses before reusing it.

中文：二者都用四个参数；标准 DH 顺序 Rot-z、Trans-z、Trans-x、Rot-x，连杆参数属于连杆末端；改进 DH 顺序 Rot-x、Trans-x、Rot-z、Trans-z，参数属于关节起点；表不同但末端位姿相同，复用前必须确认约定。

**Q2. Why do many people prefer the product-of-exponentials formulation?**

> POE describes each joint by a screw axis in one fixed base frame, taken at the zero configuration, so I do not need to attach a frame to every link, and there is no ambiguity for parallel axes. The forward kinematics is $T=e^{[S_1]\theta_1}\cdots e^{[S_n]\theta_n}M$, and the space Jacobian columns come out in closed form as adjoint transforms of the screw axes.

中文：POE 在统一基座系里用零位的螺旋轴描述每个关节，不需要给每个连杆建系，平行轴也无歧义；正运动学是指数连乘，空间雅可比的列是螺旋轴的伴随变换。追问：什么需要预先知道？（零位末端位姿 $M$ 与各零位螺旋轴。）

**Q3. How do you get the screw axis of a revolute joint?**

> Take the unit axis direction $\omega$ and any point $q$ on the axis, expressed in the base frame at the zero configuration. Then $v=-\omega\times q$ and the screw axis is $S=(\omega,v)$. If the axis passes through the origin, $v$ is zero.

中文：取零位时基座系中的单位轴方向 $\omega$ 和轴上任意一点 $q$，则 $v=-\omega\times q$，$S=(\omega,v)$；轴过原点则 $v=0$。

**Q4. What is the difference between the space Jacobian and the body Jacobian?**

> They map joint velocities to the end-effector twist expressed in different frames: the space Jacobian in the base frame, the body Jacobian in the end-effector frame. They are related by the adjoint of the current pose, $J_s=\mathrm{Ad}_T J_b$. Singularities are the same for both, because the adjoint is invertible. In practice I pick the one that matches the frame in which my task error is defined.

中文：二者把关节速度映射到末端旋量，分别表示在基座系和末端系；关系 $J_s=\mathrm{Ad}_TJ_b$；奇异性相同，因为伴随可逆；实际选与任务误差同一坐标系的那个。

**Q5. How does URDF relate to DH parameters?**

> URDF describes each joint with a fixed transform from the parent link, the origin, plus a joint axis and the joint value. That is a more general and redundant parameterisation than DH, but it is the same geometry. Libraries like KDL and Pinocchio build the kinematic tree from it, so I almost never write DH tables by hand. I still use the DH or screw-axis picture to sanity-check a URDF, for example when an axis sign is wrong.

中文：URDF 用父连杆固定变换（origin）+ 关节轴 + 关节值描述，比 DH 冗余但几何相同；KDL、Pinocchio 由它建运动学树，所以几乎不手写 DH；但会用 DH/螺旋轴图景检查 URDF（如轴方向写反）。

## 15. 本章要点速记

1. DH：每关节 4 个参数，3 个常数 + 1 个关节变量；标准与改进约定的表不同，结果相同。
2. 标准 DH：$\mathrm{Rot}_z(\theta)\mathrm{Trans}_z(d)\mathrm{Trans}_x(a)\mathrm{Rot}_x(\alpha)$；改进 DH：$\mathrm{Rot}_x(\alpha)\mathrm{Trans}_x(a)\mathrm{Rot}_z(\theta)\mathrm{Trans}_z(d)$。
3. 旋量 $\mathcal V=(\omega,v)$；螺旋轴转动关节 $v=-\omega\times q$。
4. POE：$T=e^{[\mathcal S_1]\theta_1}\cdots e^{[\mathcal S_n]\theta_n}M$，螺旋轴取零位、基座系。
5. $e^{[\mathcal S]\theta}$ 的平移项为 $(I\theta+(1-\cos\theta)[\omega]+(\theta-\sin\theta)[\omega]^2)v$，等价 $(I-R)q$。
6. 空间雅可比第 $i$ 列 $=\mathrm{Ad}_{e^{[\mathcal S_1]\theta_1}\cdots e^{[\mathcal S_{i-1}]\theta_{i-1}}}\mathcal S_i$。
7. $J_s=\mathrm{Ad}_TJ_b$；奇异性对两者等价；误差与雅可比必须同系。
8. 点速度 $\dot p=v_s+\omega_s\times p$，经典 $J_v=J_{s,v}-[p]_\times J_{s,\omega}$。
9. 平面二连杆 $\det J_v=L_1L_2\sin\theta_2$。
10. URDF 的 `origin`+`axis` 与 POE/DH 同一几何的不同分解；不必手写 DH，但要会用它检错。

## 16. 自测题

1. 一根转动关节轴沿 $y$、过点 $(0.5,0,0.2)$，螺旋轴是多少？（$\omega=(0,1,0)$，$v=-\omega\times q=-(0,1,0)\times(0.5,0,0.2)=-(0.2,0,-0.5)=(-0.2,0,0.5)$，$\mathcal S=(0,1,0,-0.2,0,0.5)$。）
2. 平面二连杆 $L_1=L_2=1$，$\theta_1=0,\theta_2=90^\circ$，末端位置？（$(1,1)$。）
3. 该位形下 $\det J_v$ 是多少？（$L_1L_2\sin90^\circ=1$。）
4. 为什么说 POE 不需要给每个连杆建系？（每个关节用零位基座系下的螺旋轴描述，关节转动用指数映射连乘。）
5. $J_b$ 与 $J_s$ 的奇异位形相同吗？为什么？（相同；$J_b=\mathrm{Ad}_{T^{-1}}J_s$，伴随矩阵可逆，不改变秩。）
6. 标准 DH 和改进 DH 的 $\alpha$ 下标有什么区别？（标准 DH 是 $\alpha_i$（连杆 $i$ 末端），改进 DH 是 $\alpha_{i-1}$（关节 $i$ 起点），错开一位。）

## 17. 相关链接

- 上一章：[机械臂动力学、重力补偿、URDF、KDL](./06_机械臂动力学_重力补偿_URDF_KDL.md)；下一章：[机械臂轨迹生成](./08_机械臂轨迹生成_多项式_梯形速度_S曲线.md)
- 相关基础：[正逆运动学与雅可比](./03_正逆运动学与雅可比.md)、[奇异性、阻尼最小二乘、零空间](./04_奇异性_阻尼最小二乘_零空间.md)、[MuJoCo、Pinocchio、KDL 工具链](../12_机器人软件/06_MuJoCo_Pinocchio_KDL_工具链.md)
- 项目：[P2](../../01_项目课程/P2_未来动力双臂遥操作与差速底盘/README.md)、[P7](../../01_项目课程/P7_可变腿长轮腿机器人/README.md)
- 模块说明：[03_运动学与坐标变换 README](./README.md)
