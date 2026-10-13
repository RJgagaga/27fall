# 视觉惯性里程计 VIO：预积分、滑动窗口与边缘化

> **一句话定位**：视觉惯性里程计（Visual-Inertial Odometry, VIO）把相机和 IMU 放进同一个优化问题里：相机提供不漂移的几何约束，IMU 提供高频运动预测、重力方向和度量尺度。本章讲清楚紧耦合 VIO 的状态变量、IMU 预积分因子、滑动窗口与边缘化（Schur 补）、初始化，以及时间偏移标定——这是"视觉 SLAM"一问之后最常见的追问。

**本章学完你能：**

- 写出 VIO 的状态向量（位姿、速度、陀螺仪与加速度计偏置、特征逆深度）并说出每一项的单位；
- 推导 IMU 预积分的定义，解释"为什么要预积分"以及"偏置变化时如何一阶修正"；
- 手算一次 Schur 补边缘化，说明为什么边缘化会产生"填充（fill-in）"和先验信息；
- 解释滑动窗口为什么能把计算量固定，以及边缘化关键帧与普通帧的区别；
- 说明视觉惯性初始化要解决哪些量（尺度、重力、速度、偏置），并手算一次尺度估计；
- 说出 VIO 中四个不可观自由度，以及时间偏移 $t_d$ 为什么重要，估算它造成的像素误差；
- 用英文回答 "Why tightly coupled / What is IMU preintegration / What is marginalisation / How do you initialise VIO"。

**前置知识**：[视觉里程计：特征、对极几何、PnP 与 BA](./02_视觉里程计_特征_对极几何_PnP_BA.md)、[SLAM 问题：前端、后端与位姿图优化](./01_SLAM问题_前端后端_位姿图优化.md)、[传感器：IMU、编码器、激光雷达](../06_状态估计/01_传感器_IMU_编码器_激光雷达.md)、[欧拉角、四元数与姿态表示](../03_运动学与坐标变换/02_欧拉角_四元数_姿态表示.md)、[李群李代数与流形优化入门](../01_数学基础/06_李群李代数与流形优化入门.md)、[最优化与二次规划](../01_数学基础/04_最优化与二次规划.md)。

## 目录

1. VIO 要解决什么问题
2. 符号与单位
3. 状态向量与整体结构
4. IMU 测量模型
5. 预积分：定义与推导
6. 手算：预积分与偏置一阶修正
7. 视觉因子与整体代价函数
8. 滑动窗口
9. 边缘化与 Schur 补（含手算）
10. 初始化（含手算）
11. 可观性与不可观自由度
12. 时间偏移与外参标定
13. 松耦合与紧耦合、滤波与优化
14. 在本项目里怎么用
15. 常见错误与故障现象
16. 面试怎么问、怎么答
17. 本章要点速记
18. 自测题
19. 相关链接

---

## 1. VIO 要解决什么问题

纯视觉里程计有三个老毛病：

1. **单目尺度不确定**：一张图只能知道方向，不知道多远；
2. **快速运动失效**：转头太快时图像模糊，特征跟不上；
3. **纹理缺失**：白墙、强光、黑暗里没有特征可跟。

IMU 恰好互补：加速度计和陀螺仪以几百赫兹测量，不依赖外部环境，能提供**度量尺度**（加速度单位是 m/s²）和**重力方向**，并能在图像失效的几十毫秒内维持短时预测。但 IMU 自己积分两次就会漂移得很快（偏置和噪声被二次积分放大），需要视觉提供不漂移的几何约束来校正。

VIO 的任务：**在线联合估计相机/机身的位姿、速度、IMU 偏置和地图点，使视觉重投影误差与 IMU 积分误差同时最小。**

```text
        图像 30 Hz                    IMU 200 Hz
           │                              │
    特征提取与跟踪                     测量累积
           │                              │
    ┌──────▼──────────────────────────────▼──────┐
    │    滑动窗口优化：视觉因子 + IMU 预积分因子   │
    │    + 边缘化先验                             │
    └──────────────────────┬─────────────────────┘
                           │
              位姿、速度、偏置、地图点
```

> **提示**：图中的频率只是典型量级（教学示例，非项目实测）。实际 IMU 常见 100–1000 Hz，相机 20–60 Hz。

## 2. 符号与单位

| 符号 | 含义 | 单位 / 说明 |
|---|---|---|
| $\mathbf p_i^w,\ \mathbf v_i^w$ | 第 $i$ 帧 IMU 在世界系下的位置、速度 | m，m/s |
| $R_i^w$ 或 $\mathbf q_i^w$ | 第 $i$ 帧 IMU 姿态（体坐标系到世界系） | 旋转矩阵 / 四元数 |
| $\mathbf b_a,\ \mathbf b_g$ | 加速度计、陀螺仪偏置 | m/s²，rad/s |
| $\tilde{\mathbf a},\ \tilde{\boldsymbol\omega}$ | 加速度计、陀螺仪测量值（体坐标系） | m/s²，rad/s |
| $\mathbf n_a,\ \mathbf n_g$ | 测量白噪声 | 连续噪声密度，单位分别为 m/s²/√Hz 与 rad/s/√Hz |
| $\mathbf g^w$ | 重力向量，世界系 | $(0,0,-9.81)$ m/s²（取 $z$ 向上） |
| $\Delta t$ | IMU 采样间隔 | s |
| $\Delta R_{ij},\Delta\mathbf v_{ij},\Delta\mathbf p_{ij}$ | 关键帧 $i$ 到 $j$ 的预积分量 | 无量纲 / m/s / m |
| $\lambda_l$ 或 $\rho_l$ | 第 $l$ 个特征的逆深度 | 1/m |
| $t_d$ | 相机与 IMU 之间的时间偏移 | s |
| $T_{bc}$ | 相机到 IMU 的外参 | $SE(3)$ |

## 3. 状态向量与整体结构

滑动窗口里一共有 $n+1$ 个关键帧，每个关键帧有一组状态：

$$
\mathbf x_k=\bigl[\mathbf p_k^w,\ \mathbf q_k^w,\ \mathbf v_k^w,\ \mathbf b_{a,k},\ \mathbf b_{g,k}\bigr],\quad k=0,\dots,n
$$

另有 $m$ 个特征点的逆深度 $\lambda_l$，以及（可选）外参 $T_{bc}$ 与时间偏移 $t_d$。每个关键帧状态的自由度是 $3+3+3+3+3=15$（姿态用 3 维切空间扰动表示）。

优化的总代价，按 VINS-Mono（Qin 等，2018，细节未核对）的写法，由三项组成：

$$
\min_{\mathcal X}\ \Bigl\{\underbrace{\lVert\mathbf r_p-H_p\mathcal X\rVert^2}_{\text{边缘化先验}}
+\sum_{k}\underbrace{\bigl\lVert\mathbf r_B(\hat{\mathbf z}_{k,k+1},\mathcal X)\bigr\rVert^2_{\Sigma_B}}_{\text{IMU 预积分因子}}
+\sum_{(l,j)}\rho\!\Bigl(\underbrace{\bigl\lVert\mathbf r_C(\hat{\mathbf z}_l^{c_j},\mathcal X)\bigr\rVert^2_{\Sigma_C}}_{\text{视觉重投影因子}}\Bigr)\Bigr\}
$$

其中 $\rho(\cdot)$ 是鲁棒核（Huber 等），$\Sigma$ 是协方差，范数 $\lVert\mathbf r\rVert_\Sigma^2=\mathbf r^\top\Sigma^{-1}\mathbf r$ 表示按不确定性加权。每个因子把"测量与预测的差"除以它的噪声尺度，噪声小的因子话语权大。

> **面试要点**：这三项就是 VIO 的全部。回答"VIO 怎么融合"时，按顺序说：视觉因子管几何，IMU 因子管运动与尺度，先验因子保存被丢掉的历史信息。

## 4. IMU 测量模型

IMU 在体坐标系下测量的是**比力**（specific force）与角速度，含偏置和噪声：

$$
\tilde{\boldsymbol\omega}=\boldsymbol\omega+\mathbf b_g+\mathbf n_g,\qquad
\tilde{\mathbf a}=R^\top\bigl(\mathbf a^w-\mathbf g^w\bigr)+\mathbf b_a+\mathbf n_a
$$

第二式里 $\mathbf a^w$ 是世界系真实加速度，减去重力（$\mathbf g^w$ 方向向下）再旋转到体坐标系。机器人静止时 $\mathbf a^w=0$，加速度计读到 $-R^\top\mathbf g^w$，即大小约为 9.81 的"向上"比力。偏置不是常数，而是缓慢游走：

$$
\dot{\mathbf b}_a=\mathbf n_{b_a},\qquad\dot{\mathbf b}_g=\mathbf n_{b_g}
$$

两个游走噪声也是白噪声。因此偏置是要被估计的**状态**，而不是一次性标定的常数。

## 5. 预积分：定义与推导

### 5.1 为什么需要预积分

两个关键帧 $i,j$ 之间有成百个 IMU 采样。若直接把这些采样当作约束，则每个关键帧状态（尤其是 $R_i,\mathbf v_i$、偏置）一变，就需要**重新从 $i$ 到 $j$ 积分一遍**，因为积分依赖于初始状态。优化要迭代几十次，代价太高。

Forster 等（2015/2017，细节未核对）提出的**预积分**的核心想法：把 IMU 积分写成**相对量**，使积分结果**与关键帧 $i$ 的姿态、速度、位置无关**，只依赖偏置。这样偏置不变时，预积分量算一次就行，优化中反复用。

### 5.2 世界系下的积分（连续）

从 $i$ 到 $j$，对真实运动有：

$$
R_j=R_i\prod_{k=i}^{j-1}\mathrm{Exp}\bigl((\tilde{\boldsymbol\omega}_k-\mathbf b_g-\mathbf n_g)\Delta t\bigr)
$$

$$
\mathbf v_j=\mathbf v_i+\mathbf g\,\Delta t_{ij}+\sum_{k=i}^{j-1}R_k(\tilde{\mathbf a}_k-\mathbf b_a-\mathbf n_a)\Delta t
$$

$$
\mathbf p_j=\mathbf p_i+\sum_{k=i}^{j-1}\Bigl[\mathbf v_k\Delta t+\tfrac12\mathbf g\Delta t^2+\tfrac12R_k(\tilde{\mathbf a}_k-\mathbf b_a-\mathbf n_a)\Delta t^2\Bigr]
$$

这里 $\mathrm{Exp}$ 是 $\mathfrak{so}(3)\to SO(3)$ 的指数映射（见 [李群李代数](../01_数学基础/06_李群李代数与流形优化入门.md)）。

### 5.3 相对量（预积分量）

把 $R_k=R_iR_i^\top R_k$ 代入并整理，定义：

$$
\Delta R_{ij}=R_i^\top R_j=\prod_{k=i}^{j-1}\mathrm{Exp}\bigl((\tilde{\boldsymbol\omega}_k-\mathbf b_g)\Delta t\bigr)
$$

$$
\Delta\mathbf v_{ij}=R_i^\top\bigl(\mathbf v_j-\mathbf v_i-\mathbf g\Delta t_{ij}\bigr)=\sum_{k=i}^{j-1}\Delta R_{ik}(\tilde{\mathbf a}_k-\mathbf b_a)\Delta t
$$

$$
\Delta\mathbf p_{ij}=R_i^\top\bigl(\mathbf p_j-\mathbf p_i-\mathbf v_i\Delta t_{ij}-\tfrac12\mathbf g\Delta t_{ij}^2\bigr)=\sum_{k=i}^{j-1}\Bigl[\Delta\mathbf v_{ik}\Delta t+\tfrac12\Delta R_{ik}(\tilde{\mathbf a}_k-\mathbf b_a)\Delta t^2\Bigr]
$$

右边只含 IMU 测量和偏置，**不含**关键帧 $i$ 的位姿与速度——这就是"预"积分的含义。左边是由状态预测出的相对量。二者之差就是 IMU 因子的残差：

$$
\mathbf r_{\Delta R}=\mathrm{Log}\bigl(\Delta\hat R_{ij}^\top R_i^\top R_j\bigr),\quad
\mathbf r_{\Delta v}=R_i^\top(\mathbf v_j-\mathbf v_i-\mathbf g\Delta t_{ij})-\Delta\hat{\mathbf v}_{ij},\quad
\mathbf r_{\Delta p}=R_i^\top(\mathbf p_j-\mathbf p_i-\mathbf v_i\Delta t_{ij}-\tfrac12\mathbf g\Delta t_{ij}^2)-\Delta\hat{\mathbf p}_{ij}
$$

### 5.4 偏置变化时的一阶修正

优化过程中偏置估计会变（比如 $\mathbf b_a$ 从 $\bar{\mathbf b}_a$ 变到 $\bar{\mathbf b}_a+\delta\mathbf b_a$）。如果每次变化都重新积分，预积分的优势就没了。Forster 的做法：在积分时顺便计算预积分量对偏置的**雅可比**，偏置变化小时一阶修正：

$$
\Delta\mathbf v_{ij}(\mathbf b_a+\delta\mathbf b_a)\approx\Delta\mathbf v_{ij}(\bar{\mathbf b}_a)+\frac{\partial\Delta\mathbf v_{ij}}{\partial\mathbf b_a}\delta\mathbf b_a
$$

偏置变化很大时（超过某个阈值）再重新积分。

### 5.5 噪声传播

预积分量的协方差通过线性递推得到，每过一个采样 $\Sigma_{k+1}=A_k\Sigma_kA_k^\top+B_kQB_k^\top$，其中 $Q$ 为 IMU 噪声协方差。它在优化里充当权重 $\Sigma_B^{-1}$。

## 6. 手算：预积分与偏置一阶修正

用 1D 例子把"预积分对偏置的雅可比"算清楚。示例（非项目实测）：沿 $x$ 轴直线运动，姿态不变，忽略重力和噪声；加速度计采样 $N=100$ 个，$\Delta t=0.01$ s，总时长 $T=1$ s。真实加速度取 $a_{true,k}=0.5\sin u_k$，$u_k$ 在 0 到 3 之间均匀取 100 个点，加速度计偏置真值 $b_a=0.05$ m/s²，所以每个测量 $\tilde a_k=a_{true,k}+0.05$。

预积分的离散公式：

$$
\Delta p\leftarrow\Delta p+\Delta v\,\Delta t+\tfrac12(\tilde a_k-b_a)\Delta t^2,\qquad
\Delta v\leftarrow\Delta v+(\tilde a_k-b_a)\Delta t
$$

**解析雅可比**：

- $\partial\Delta v/\partial b_a=-\sum\Delta t=-N\Delta t=-T=-1.0$ s；
- $\partial\Delta p/\partial b_a=-\tfrac12T^2=-0.5$ s²（连续时间近似；离散时有极小差异）。

先用 $b_a=0$ 预积分：$\Delta p=0.18230$，$\Delta v=0.37868$。如果优化后偏置估计变成 $0.03$：

- **重新积分**得 $\Delta p=0.16730$，$\Delta v=0.34868$；
- **一阶修正**：$0.18230+(-0.5)\times0.03=0.16730$，$0.37868+(-1.0)\times0.03=0.34868$。

两者一致（Python 复核，数值积分的雅可比为 $-0.49999999,\,-1.0000000$，与解析一致）。因为偏置对加速度是**线性**作用，在这个 1D 例子里一阶修正是精确的；一般情形里旋转与偏置耦合是非线性的，所以才有"变化小用雅可比、变化大才重积分"的规则。

这个小例子还给出一个重要量级：**加速度计偏置 0.03 m/s² 看起来很小，但 1 秒就累积出 3 cm/s 的速度误差和 1.5 cm 的位置误差；10 秒就是 1.5 m**（$\tfrac12 b\,t^2$，$0.5\times0.03\times100=1.5$）。这就是纯 IMU 导航不能长时间使用的原因。

## 7. 视觉因子与整体代价函数

对每个特征 $l$，它在第 $i$ 帧首次被观测到，逆深度为 $\lambda_l$，归一化平面坐标 $\mathbf u_l^{c_i}$。把它变换到第 $j$ 帧的相机系并投影，与第 $j$ 帧的观测比较，得到重投影残差：

$$
\mathbf r_C=\begin{bmatrix}\dfrac{X}{Z}-u_j\\[4pt]\dfrac{Y}{Z}-v_j\end{bmatrix},\qquad
\begin{bmatrix}X\\Y\\Z\end{bmatrix}=R_{bc}^\top\Bigl\{R_j^\top\Bigl[R_i\bigl(R_{bc}\tfrac{1}{\lambda_l}\bar{\mathbf u}_l^{c_i}+\mathbf p_{bc}\bigr)+\mathbf p_i-\mathbf p_j\Bigr]-\mathbf p_{bc}\Bigr\}
$$

这个式子是"把点从第 $i$ 帧相机系，经 IMU 系、世界系，再回到第 $j$ 帧相机系"的链式变换。关键在于它同时含有位姿 $(R_i,\mathbf p_i,R_j,\mathbf p_j)$、外参 $(R_{bc},\mathbf p_{bc})$ 和逆深度 $\lambda_l$。用**逆深度**参数化的理由：远点（深度趋向无穷）时逆深度趋向 0，数值良好；高斯分布的假设对逆深度更成立。

## 8. 滑动窗口

### 8.1 动机

SLAM 中随着时间推移，关键帧、特征点会无限增长，BA 的规模无上限。VIO 要求**实时**，所以只保留**最近的固定数量**关键帧（典型 10 帧左右，教学示例，非项目实测）。窗口大小固定，每次优化代价近似固定。

### 8.2 状态规模

示例（非项目实测）：窗口 10 帧，每帧 15 维，共 150 维；再有 100 个特征，每个 1 维，合计 250 维。这在嵌入式 CPU 上仍然可以每帧优化一次。

### 8.3 窗口滑动时怎么处理被移出的帧

最旧的关键帧要移出窗口，但直接丢掉会**丢失信息**：它和窗口内帧之间还有 IMU 因子、共视的视觉因子，这些约束没有它就不完整。不能简单丢，要用**边缘化**把它的信息压缩成窗口内剩余变量上的一个**先验因子**。

VINS-Mono 里有两种策略（未核对实现细节）：

- 次新帧是关键帧（视差足够）：**边缘化最旧帧**及其观测的特征，保留其约束信息；
- 次新帧不是关键帧（视差不够，相邻帧太像）：**直接丢掉次新帧的视觉观测**，保留其 IMU 预积分，这样不会让窗口里挤满几乎相同的帧，也避免滑窗退化。

## 9. 边缘化与 Schur 补（含手算）

### 9.1 公式

非线性最小二乘在当前线性化点上的正规方程为：

$$
\begin{bmatrix}H_{mm}&H_{mr}\\H_{rm}&H_{rr}\end{bmatrix}
\begin{bmatrix}\delta\mathbf x_m\\\delta\mathbf x_r\end{bmatrix}=
\begin{bmatrix}\mathbf b_m\\\mathbf b_r\end{bmatrix}
$$

其中下标 $m$ 为要边缘化的变量，$r$ 为保留的变量。对第一行解出 $\delta\mathbf x_m=H_{mm}^{-1}(\mathbf b_m-H_{mr}\delta\mathbf x_r)$ 代入第二行，得到只含保留变量的方程：

$$
\underbrace{\bigl(H_{rr}-H_{rm}H_{mm}^{-1}H_{mr}\bigr)}_{H_{\text{prior}}}\delta\mathbf x_r=\underbrace{\mathbf b_r-H_{rm}H_{mm}^{-1}\mathbf b_m}_{\mathbf b_{\text{prior}}}
$$

这个 $H_{rr}-H_{rm}H_{mm}^{-1}H_{mr}$ 就是 **Schur 补**。它等价于对高斯分布做边缘分布：信息矩阵（精度矩阵）取 Schur 补，协方差矩阵直接取子块。

### 9.2 手算

示例（非项目实测）：三个标量变量 $x_0,x_1,x_2$ 的一维链：

- 先验：$x_0$ 的先验 $r=x_0-0$，权重 4；
- 因子 A：$x_1-x_0=1$，权重 2；
- 因子 B：$x_2-x_1=1$，权重 3。

线性化点取 $\mathbf x=(0,\,0.8,\,2.1)$。叠加每个因子 $w\,\mathbf J^\top\mathbf J$ 得

$$
H=\begin{bmatrix}6&-2&0\\-2&5&-3\\0&-3&3\end{bmatrix},\qquad\mathbf b=\begin{bmatrix}-0.4\\1.3\\-0.9\end{bmatrix}
$$

完整解为 $\delta\mathbf x=(0,\;0.2,\;-0.1)$。

**(a) 边缘化 $x_0$**：$H_{mm}=6$，$H_{rm}=(-2,0)^\top$，则

$$
H_{\text{prior}}=\begin{bmatrix}5&-3\\-3&3\end{bmatrix}-\frac{1}{6}\begin{bmatrix}4&0\\0&0\end{bmatrix}=\begin{bmatrix}4.333&-3\\-3&3\end{bmatrix}
$$

$\mathbf b_{\text{prior}}=(1.3,\,-0.9)^\top-\tfrac{-0.4}{6}(-2,0)^\top=(1.1667,\,-0.9)$。解得 $(0.2,\,-0.1)$，与完整解一致（Python 复核）。注意：$H_{11}$ 从 5 降到 4.333，**因为 $x_0$ 作为一个已知节点原本"拉住"了 $x_1$，现在它被边缘化，变成一个较弱的先验**。

**(b) 边缘化中间变量 $x_1$**：$H_{mm}=5$，$H_{rm}=(-2,-3)^\top$（行 $x_0,x_2$）：

$$
H_{\text{prior}}=\begin{bmatrix}6&0\\0&3\end{bmatrix}-\frac15\begin{bmatrix}4&6\\6&9\end{bmatrix}=\begin{bmatrix}5.2&-1.2\\-1.2&1.2\end{bmatrix}
$$

原来 $x_0$ 与 $x_2$ 之间没有直接联系（$H_{02}=0$），边缘化 $x_1$ 之后变成 $-1.2$：**产生了填充（fill-in）**。直觉：$x_1$ 同时与二者相连，把它抹掉，二者就变成"间接相关"，需要用一个直接的先验项来表达。保留变量的解 $(0,\,-0.1)$ 仍与完整解一致。

> **注意**：边缘化后得到的先验是在**当时的线性化点**上构造的。以后保留变量的估计变了，先验的雅可比不能再更新（否则会改变零空间）。这叫 **First-Estimate Jacobian (FEJ)** 问题——不一致的线性化点会让原本不可观的方向意外变得"可观"，使估计过度自信。VINS-Mono 的做法是固定先验的线性化点，只更新残差（细节未核对）。

### 9.3 边缘化的代价

边缘化后先验矩阵变稠密，窗口越大、边缘化的变量连接越多，稠密块越大；所以 VIO 要保持窗口小、关键帧稀疏。

## 10. 初始化（含手算）

### 10.1 为什么难

VIO 是高度非线性的，必须有合理初值才能收敛。初始化要估计：陀螺仪偏置、重力方向、**视觉尺度**、初始速度，还有各帧的位姿。典型流程（VINS-Mono 风格，细节未核对）：

1. **纯视觉 SfM**：用滑动窗口内若干帧做单目 SfM（对极几何 + 三角化 + BA），得到**无尺度**的相机位姿 $\mathbf p_c$ 序列；
2. **陀螺仪偏置标定**：视觉得到的相对旋转 $\Delta R_{ij}^{c}$ 与 IMU 预积分旋转应一致，最小化差异求出 $\mathbf b_g$；
3. **尺度、重力、速度对齐**：IMU 预积分给出度量位移，与视觉的无尺度位移比对，线性最小二乘求尺度 $s$、重力向量 $\mathbf g$、各帧速度；
4. **重力精化**：把重力大小约束为 9.81 左右，仅用两个自由度（方向）精化，使世界系 $z$ 轴与重力方向对齐。

### 10.2 手算：尺度

示例（非项目实测）：视觉 SfM 得到三段无尺度位移（沿同一方向，单位任意）$\mathbf d_c=(0.5,\,1.0,\,1.5)$；IMU 预积分得到对应的度量位移（m）$\mathbf d_i=(0.9,\,2.0,\,3.1)$。模型 $\mathbf d_i\approx s\,\mathbf d_c$，最小二乘：

$$
s=\frac{\mathbf d_c^\top\mathbf d_i}{\mathbf d_c^\top\mathbf d_c}=\frac{0.45+2.0+4.65}{0.25+1+2.25}=\frac{7.1}{3.5}=2.029
$$

Python 复核：`2.02857`，残差 $(-0.114,-0.029,0.057)$ m。意义：视觉世界里的一个单位等于真实世界 2.03 m。实际中残差包含 IMU 噪声、偏置、重力方向误差，所以要同时解尺度和重力。

### 10.3 手算：重力方向

机器人静止时，加速度计读数就是 $-R^\top\mathbf g$，用它可以得到初始的横滚 (roll) 与俯仰 (pitch)（偏航不可观）。示例（非项目实测）：读数 $\mathbf a=(0.3,\,-0.2,\,9.7)$ m/s²。

$$
\text{roll}=\mathrm{atan2}(a_y,a_z)=\mathrm{atan2}(-0.2,9.7)=-1.18^\circ,\qquad
\text{pitch}=\mathrm{atan2}\bigl(-a_x,\sqrt{a_y^2+a_z^2}\bigr)=-1.77^\circ
$$

范数 $\lVert\mathbf a\rVert=9.707$，与 9.81 差 0.1，这就是偏置/噪声和单位误差的量级提示（Python 复核）。

### 10.4 初始化的失败模式

- 初始运动太小（无视差）：SfM 退化，尺度估不准；
- 初始只有直线匀速：加速度没有被激发，尺度与加速度计偏置不可分；
- 初始就在剧烈旋转：特征跟丢。

> **注意**：好的初始化条件是"有一定平移视差，且有加速度激励"，所以很多系统会提示用户先"晃一晃"。

## 11. 可观性与不可观自由度

对紧耦合 VIO，**全局位置 $(x,y,z)$ 和绕重力轴的偏航角 $\psi$ 共 4 个自由度不可观**：把整条轨迹与地图一起平移或绕重力轴旋转，视觉与 IMU 的所有残差都不变。这解释了：

- 为什么 VIO 轨迹一定会有 yaw 漂移与位置漂移，需要回环或 GPS 去修正；
- 为什么滑动窗口里要固定第一帧的位置和偏航，防止优化沿零空间"游走"（gauge freedom）；
- 为什么错误的线性化点会"偷偷"让 yaw 变得可观（即 FEJ 问题），造成不一致。

而重力方向（roll、pitch）、速度、尺度（对单目 VIO）在**运动被充分激励**时是可观的。若机器人一直做匀速直线运动，尺度和加速度计偏置之间会出现耦合，可观性降低。

## 12. 时间偏移与外参标定

### 12.1 时间偏移

相机时间戳是曝光中点还是读出结束？IMU 时间戳是采样时刻还是总线收到时刻？两路传感器通常有几毫秒到几十毫秒的固定偏移 $t_d$。若忽略，视觉观测与 IMU 预测会在时间上错位。

特征点在图像上以速度 $\dot{\mathbf u}$ 运动时，由 $t_d$ 带来的像素误差为

$$
\delta\mathbf u\approx\dot{\mathbf u}\,t_d
$$

示例（非项目实测）：特征像素速度 200 px/s，$t_d=10$ ms，则 $\delta u=2$ px，这已经超过常见重投影鲁棒核的内点阈值的一半；机器人以 1.5 m/s 运动、时间差 20 ms，则位置误差约 0.03 m（Python 复核）。

VINS-Mono 的做法是把 $t_d$ 作为状态在线估计：用特征速度把观测在像素平面上"移动"到对应的 IMU 时刻，使残差对 $t_d$ 可导（未核对）。离线则常用 Kalibr 这类工具联合标定相机-IMU 外参与时间偏移（工具名为通用知识，参数未核对）。

### 12.2 外参

相机到 IMU 的外参 $T_{bc}$（平移几厘米，旋转精度要求以度计）：外参误差会直接造成系统性的位姿偏差。外参先离线标定，再在线精化。同时 IMU 本身的噪声密度、随机游走参数也要来自数据手册或 Allan 方差标定，不能随便设。

## 13. 松耦合与紧耦合、滤波与优化

| 方案 | 做法 | 优点 | 缺点 |
|---|---|---|---|
| 松耦合 | 视觉先算出位姿，再与 IMU 在卡尔曼滤波中融合 | 简单，模块独立 | 视觉退化时信息丢失；协方差估计粗糙 |
| 紧耦合（滤波） | 视觉特征残差直接进入滤波更新（MSCKF/OpenVINS 类） | 计算量低，状态规模小 | 线性化一次，误差难回溯 |
| 紧耦合（优化） | 滑动窗口 BA + 预积分（VINS-Mono/ORB-SLAM3 类） | 精度高，可重线性化 | 计算量大，需控制窗口 |

**为什么紧耦合更好**：IMU 的作用是约束视觉三角化与位姿，视觉的作用是校正 IMU 偏置。两者在同一问题里相互约束，偏置才能被真正估准。

## 14. 在本项目里怎么用

七个项目的 CV 没有 VIO 项目，本章属于**延伸知识**。可以诚实关联的点：

- **P1 / P7**：它们的姿态估计使用 IMU 与轮速反馈（互补滤波/Mahony），IMU 偏置、噪声、重力对齐的概念与本章完全相通，只是没有视觉约束。对应基础可参考 [互补滤波与 Mahony AHRS](../06_状态估计/02_互补滤波与Mahony_AHRS.md)；
- **P2**：VR 头显与手柄的位姿跟踪本质上是 VIO/SLAM 的产物，P2 使用的是它们给出的位姿，并非自己实现；见 [VR 遥操作的相对位姿映射](../../01_项目课程/P2_未来动力双臂遥操作与差速底盘/02_VR遥操作的相对位姿映射.md)；
- **P5**：EKF 融合 IMU 与轮速编码器，是松耦合的低维版本；P5 遇到的"时间、坐标系不一致造成漂移与跳变"，在 VIO 里表现为时间偏移 $t_d$ 与外参误差，详见 [传感器与数据质量](../../01_项目课程/P5_车式移动机器人自主导航/03_传感器与数据质量_时间坐标与运动方向.md) 和 [EKF 融合定位](../../01_项目课程/P5_车式移动机器人自主导航/04_EKF融合定位.md)；
- **P6**：多源感知的时间戳、坐标系和有效性统一，是任何 VIO 能正常工作的前提，见 [机器人状态管理](../../01_项目课程/P6_深开鸿M-Robots平台软件/05_机器人状态管理.md)。

被问到"你做过 VIO 吗"：我没有实现过 VIO，但我在导航和平衡项目里处理过 IMU 融合、时间同步和坐标系一致性问题，理解 VIO 的状态、预积分和边缘化的原理。

## 15. 常见错误与故障现象

| 现象 | 可能原因 | 处理 |
|---|---|---|
| 尺度越走越飘 | 初始化时无加速度激励；加速度计偏置估错 | 重新初始化；检查 IMU 噪声参数 |
| 快速旋转时轨迹发散 | 陀螺仪量程饱和；时间偏移未估 | 检查量程；开启在线 $t_d$ 估计 |
| 静止时位置缓慢漂走 | 偏置未收敛或静止未检测 | 加零速更新；检查重力对齐 |
| 重投影误差大但优化不降 | 外参错、相机畸变未校 | 先标定相机与外参 |
| 边缘化后估计"变僵硬" | 先验线性化点过旧，信息过度自信 | 固定先验线性化点，必要时重置窗口 |
| 轨迹出现台阶跳变 | 误匹配被鲁棒核没挡住；时间戳乱序 | 收紧内点阈值；核对时间同步 |
| 不同机型换了 IMU 就不好用 | 噪声密度/游走参数仍用旧值 | 按新 IMU 手册或 Allan 方差重设 |
| 计算延迟越来越大 | 窗口过大、特征过多 | 限制窗口、特征数；降低优化迭代 |

## 16. 面试怎么问、怎么答

**Q1. Why is visual-inertial fusion better than vision alone or IMU alone?**

> Vision gives drift-free geometric constraints but has scale ambiguity and fails under fast motion or poor texture. The IMU gives metric scale, gravity direction and high-rate motion prediction, but integrating it twice drifts quickly because of bias and noise. Fusing them tightly lets each fix the other's weakness: vision corrects the IMU bias, and the IMU supplies scale and short-term motion.

中文：视觉没有漂移但有尺度歧义，且怕快速运动和弱纹理；IMU 有尺度和重力、高频，但二次积分漂移快；紧耦合让视觉校正偏置、IMU 提供尺度和短时预测。追问：IMU 单独积分 10 秒会漂多少？（加速度偏置 0.03 m/s² 时约 1.5 m。）

**Q2. What is IMU preintegration and why is it needed?**

> Between two keyframes there are hundreds of IMU samples. Preintegration summarises them into relative rotation, velocity and position increments that do not depend on the initial pose and velocity of the first keyframe. Without it, every time the optimiser changes the keyframe state I would need to re-integrate all the samples. With it I integrate once and only apply a first-order correction when the bias estimate changes.

中文：把两关键帧间所有 IMU 样本压缩为与起始位姿、速度无关的相对增量；偏置变化时用雅可比做一阶修正，不用重新积分。追问：偏置变化太大怎么办？（重新积分。）

**Q3. What is marginalisation and what problem does it cause?**

> When a keyframe leaves the sliding window, I do not just drop it, because its constraints still carry information. I marginalise it with the Schur complement, which turns that information into a prior on the remaining variables. The side effects are that the prior becomes dense, with fill-in, and that it is fixed at an old linearisation point, which can make the estimator inconsistent unless I use first-estimate Jacobians.

中文：移出窗口的帧用 Schur 补边缘化，把信息变成剩余变量的先验；副作用是矩阵变稠密（填充）、线性化点固定会造成不一致，所以用 FEJ。追问：为什么不直接丢？（丢弃会损失约束并导致估计跳变。）

**Q4. Which degrees of freedom are unobservable in VIO?**

> Global position and the yaw angle around gravity, so four degrees of freedom. Shifting or rotating the whole trajectory about gravity does not change any visual or inertial residual. Roll, pitch, velocity and, for monocular, scale are observable when the motion is sufficiently excited.

中文：全局位置和绕重力轴的偏航共 4 个自由度不可观；滚转、俯仰、速度及（单目）尺度在运动被激励时可观。追问：怎样消除这种漂移？（回环、GPS 或地图重定位。）

**Q5. How do you initialise a VIO system?**

> I first run a vision-only structure from motion over a short window to get up-to-scale poses. Then I use the relative rotations to estimate the gyroscope bias, and align the visual translations with the preintegrated IMU displacements to solve for scale, gravity and velocities. The motion needs enough parallax and acceleration, otherwise scale and accelerometer bias are not separable.

中文：先短窗纯视觉 SfM 得无尺度位姿；用相对旋转估陀螺偏置；再对齐视觉位移与预积分位移求尺度、重力和速度；需要足够视差和加速度激励。

## 17. 本章要点速记

1. VIO = 视觉因子 + IMU 预积分因子 + 边缘化先验；每个关键帧 15 维状态。
2. 偏置是缓慢游走的状态，不是常数；加速度计偏置 0.03 m/s² 在 10 s 内造成约 1.5 m 位置误差。
3. 预积分让积分量与起始位姿、速度无关；偏置变化小时用雅可比做一阶修正，变化大才重积分。
4. 逆深度参数化对远点数值稳定，并更接近高斯。
5. 滑动窗口把计算量固定，移出的帧用 Schur 补边缘化；$H_{\text{prior}}=H_{rr}-H_{rm}H_{mm}^{-1}H_{mr}$。
6. 边缘化会引起填充（fill-in）和线性化点固定，FEJ 用来保持可观性一致。
7. 初始化：SfM、陀螺偏置、尺度/重力/速度对齐、重力精化；需要视差与加速度激励。
8. VIO 有 4 个不可观自由度（全局位置和绕重力偏航）。
9. 时间偏移造成 $\delta\mathbf u\approx\dot{\mathbf u}\,t_d$ 的像素误差，要在线估计或离线标定。
10. 紧耦合比松耦合好在：两种传感器共同约束同一组变量，偏置与尺度才能被估准。

## 18. 自测题

1. 为什么预积分量不含关键帧 $i$ 的姿态与速度？（把积分式乘以 $R_i^\top$ 并把重力、起始速度项移到状态一侧，剩下只含 IMU 测量与偏置的相对量。）
2. 窗口 8 帧、每帧 15 维、60 个逆深度特征，状态维数是多少？（$8\times15+60=180$。）
3. 对 $H=\begin{bmatrix}2&1\\1&3\end{bmatrix}$ 边缘化第一个变量，得到的先验信息是多少？（$3-1\times1/2=2.5$。）
4. 视觉位移 $(1,2)$、IMU 位移 $(3,5)$（m），最小二乘尺度是多少？（$(3+10)/(1+4)=2.6$。）
5. 特征像素速度 300 px/s，时间偏移 15 ms，造成多少像素误差？（4.5 px。）

## 19. 相关链接

- 上一章：[激光 SLAM：LOAM、LIO-SAM、Cartographer](./03_激光SLAM_LOAM_LIO-SAM_Cartographer.md)
- 下一章：[回环检测与重定位：词袋与 Scan Context](./05_回环检测与重定位_词袋_ScanContext.md)
- 基础：[欧拉角、四元数与姿态表示](../03_运动学与坐标变换/02_欧拉角_四元数_姿态表示.md)、[里程计与传感器融合](../06_状态估计/04_里程计与传感器融合.md)、[相机模型与标定](../09_感知与视觉/01_相机模型与标定.md)
- 项目：[P5 车式移动机器人导航 README](../../01_项目课程/P5_车式移动机器人自主导航/README.md)
- 模块说明：[07_SLAM与建图 README](./README.md)
