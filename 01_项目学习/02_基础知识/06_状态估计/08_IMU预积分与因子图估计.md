# IMU 预积分与因子图估计

> **一句话定位**：IMU 每秒输出几百到上千个读数，而视觉或激光的关键帧一秒只有十几个。优化器需要的是“两个关键帧之间 IMU 告诉了我们什么”——一个相对运动量，而且在优化过程中关键帧的位姿估计不断变化，却不希望每次都把几百个 IMU 读数重新积分一遍。预积分（preintegration）就是把这段 IMU 数据压缩成一个与起始位姿无关的相对运动测量，再作为“因子”放进因子图（factor graph）里，与视觉、激光、GNSS 因子一起做最大后验估计。

**本章学完你能：**

- 写出 IMU 测量模型与连续时间运动学，说明为什么直接积分的结果依赖起始位姿；
- 推导预积分量 $\Delta R_{ij},\Delta v_{ij},\Delta p_{ij}$，写出用它们预测下一个关键帧状态的公式，并手算三步例子；
- 解释偏置更新为什么用一阶雅可比修正而不重积分，并用数值验证修正精度；
- 说明预积分噪声协方差的传播，以及它与 ARW（角度随机游走）的关系，用 Monte Carlo 验证；
- 把估计问题写成因子图：变量、因子、最大后验等价于非线性最小二乘；手算一个三节点位姿链加回环的例子；
- 解释边缘化（Schur 补）与滑动窗口，手算一次边缘化生成的稠密先验；
- 说出增量平滑（iSAM2 思想）与固定滞后/滑动窗口的取舍；
- 用英文回答 "What is IMU preintegration / why factor graphs / what does marginalisation do" 等问题。

**前置知识**：[传感器：IMU、编码器、激光雷达](./01_传感器_IMU_编码器_激光雷达.md)（IMU 误差模型）、[李群李代数与流形优化入门](../01_数学基础/06_李群李代数与流形优化入门.md)（$\mathrm{Exp},\mathrm{Log}$、右雅可比）、[UKF、信息滤波与卡尔曼平滑](./07_UKF_信息滤波_卡尔曼平滑.md)（信息形式、平滑）、[SLAM问题_前端后端_位姿图优化](../07_SLAM与建图/01_SLAM问题_前端后端_位姿图优化.md)（图优化）、[随机信号…Allan 方差](../04_信号与系统/04_随机信号_自相关_功率谱_白噪声与Allan方差.md)（噪声参数）。

## 目录

1. 问题：IMU 频率高，关键帧频率低
2. 符号与约定
3. IMU 模型与运动学
4. 预积分的推导
5. 手算：三步预积分
6. 偏置更新：一阶修正
7. 噪声传播与协方差
8. 因子图：从概率到最小二乘
9. 手算：三节点位姿链加回环
10. 边缘化与滑动窗口
11. 增量平滑与 iSAM2 的思想
12. 在 VIO / LIO 中的位置
13. 在本项目里怎么用
14. 常见错误与故障现象
15. 面试怎么问、怎么答
16. 本章要点速记
17. 自测题
18. 相关链接

---

## 1. 问题：IMU 频率高，关键帧频率低

设 IMU 以 200 Hz 输出，相机或激光雷达的关键帧以 10 Hz 出现，每两个关键帧之间有 20 个 IMU 读数。我们想在后端优化里利用 IMU：它告诉我们两个关键帧之间机器人转了多少、速度变了多少、位移多少。

**朴素做法**：把关键帧 $i$ 的状态（位姿、速度）作为初值，沿着 IMU 读数逐个积分到关键帧 $j$，得到 $j$ 的预测。这个预测依赖于 $i$ 的状态。在优化迭代中，$i$ 的状态每次都在变化，每变一次，就得把 20 个读数重新积分一遍；在大窗口（几十个关键帧）里，这个代价很大。

**预积分的思路**：把 IMU 读数在“关键帧 $i$ 的机体坐标系”里积分，得到只与读数和偏置有关、与 $i$ 的世界位姿无关的**相对量** $\Delta R_{ij},\Delta v_{ij},\Delta p_{ij}$。这些量只需要算一次。优化时，$i$ 和 $j$ 的世界状态怎么变，都直接用这三个量构造残差，不必重积分。偏置估计变化不大时，用一阶雅可比做修正，也不重积分。

这个技术最早在视觉惯性里程计（VIO）中被系统化（Forster 等人的工作，年份与细节未核对），现在是 VIO（如 VINS 系列）和激光惯性里程计（如 LIO-SAM）的标准做法。

## 2. 符号与约定

| 符号 | 含义 | 单位 |
|---|---|---|
| $R_i\in SO(3)$ | 关键帧 $i$ 的姿态（机体到世界） | |
| $p_i,v_i$ | 位置、速度（世界系） | m，m/s |
| $b_g,b_a$ | 陀螺偏置、加速度计偏置 | rad/s，m/s² |
| $\omega_m,a_m$ | 陀螺、加速度计测量（机体系） | rad/s，m/s² |
| $n_g,n_a$ | 测量白噪声 | |
| $g$ | 重力向量（世界系），取 $(0,0,-9.81)$ | m/s² |
| $\Delta t$ | IMU 采样周期 | s |
| $\Delta t_{ij}$ | 关键帧间隔 | s |
| $\Delta R_{ij},\Delta v_{ij},\Delta p_{ij}$ | 预积分量（在 $i$ 系下的相对量） | |
| $J_{(\cdot)}^{b}$ | 预积分量对偏置的雅可比 | |
| $\Sigma_{ij}$ | 预积分噪声协方差（$9\times9$） | |
| $x_i=(R_i,p_i,v_i)$ | 关键帧状态 | |
| $\mathrm{Exp},\mathrm{Log},J_r$ | $SO(3)$ 指数、对数、右雅可比 | |

**约定**：加速度计测量的是**比力**（specific force，不含重力的加速度）：静止水平放置时读数为 $(0,0,+9.81)$。旋转用右扰动，$R\leftarrow R\,\mathrm{Exp}(\delta\phi)$。

## 3. IMU 模型与运动学

### 3.1 测量模型

$$\omega_m=\omega+b_g+n_g,\qquad a_m=R^\top(a-g)+b_a+n_a.$$

第二式：真实加速度 $a$（世界系）减去重力 $g$，转到机体系，再加偏置和噪声。（$g$ 在世界系 $z$ 向下，故静止时 $a=0$，$a-g=(0,0,+9.81)$，机体系读数为 $+9.81$。）

### 3.2 运动学

$$\dot R=R\,\omega^\wedge,\qquad \dot v=a=R(a_m-b_a-n_a)+g,\qquad \dot p=v.$$

### 3.3 离散积分（欧拉，$\Delta t$ 内常数）

$$R_{k+1}=R_k\,\mathrm{Exp}\big((\omega_m-b_g-n_g)\Delta t\big),$$
$$v_{k+1}=v_k+g\Delta t+R_k(a_m-b_a-n_a)\Delta t,$$
$$p_{k+1}=p_k+v_k\Delta t+\tfrac12g\Delta t^2+\tfrac12R_k(a_m-b_a-n_a)\Delta t^2.$$

## 4. 预积分的推导

### 4.1 把 $i$ 到 $j$ 的积分展开

把上面的递推从 $k=i$ 到 $j-1$ 连乘/累加（$\Delta t_{ij}=(j-i)\Delta t$）：

$$R_j=R_i\prod_{k=i}^{j-1}\mathrm{Exp}\big((\omega_k-b_g-n_g)\Delta t\big),$$
$$v_j=v_i+g\Delta t_{ij}+\sum_{k=i}^{j-1}R_k(a_k-b_a-n_a)\Delta t,$$
$$p_j=p_i+\sum_{k=i}^{j-1}\Big[v_k\Delta t+\tfrac12g\Delta t^2+\tfrac12R_k(a_k-b_a-n_a)\Delta t^2\Big].$$

注意 $R_k=R_i\,\Delta R_{ik}$，其中 $\Delta R_{ik}=\prod_{l=i}^{k-1}\mathrm{Exp}(\cdots)$ 只依赖陀螺读数。代入后，所有对 $R_i$ 的依赖被提到求和号外面：

$$\boxed{\begin{aligned}\Delta R_{ij}&=\prod_{k=i}^{j-1}\mathrm{Exp}\big((\omega_k-b_g)\Delta t\big),\\\Delta v_{ij}&=\sum_{k=i}^{j-1}\Delta R_{ik}(a_k-b_a)\Delta t,\\\Delta p_{ij}&=\sum_{k=i}^{j-1}\Big[\Delta v_{ik}\Delta t+\tfrac12\Delta R_{ik}(a_k-b_a)\Delta t^2\Big].\end{aligned}}$$

（噪声项暂略，见第 7 节。）这三个量**只由 IMU 读数与当前偏置估计决定，与 $R_i,v_i,p_i$ 无关**。递推形式（便于在线累积）：

$$\Delta p\leftarrow\Delta p+\Delta v\,\Delta t+\tfrac12\Delta R(a-b_a)\Delta t^2,\quad \Delta v\leftarrow\Delta v+\Delta R(a-b_a)\Delta t,\quad \Delta R\leftarrow\Delta R\,\mathrm{Exp}((\omega-b_g)\Delta t),$$

注意更新顺序：先 $\Delta p$，再 $\Delta v$，最后 $\Delta R$，使用的都是“更新前”的值。

### 4.2 预测与残差

用预积分量预测关键帧 $j$ 的状态：

$$R_j=R_i\Delta R_{ij},\qquad v_j=v_i+g\Delta t_{ij}+R_i\Delta v_{ij},\qquad p_j=p_i+v_i\Delta t_{ij}+\tfrac12g\Delta t_{ij}^2+R_i\Delta p_{ij}.$$

因子图里的残差就是“状态给出的相对运动”与“预积分测量”的差：

$$r_{\Delta R}=\mathrm{Log}\big(\Delta R_{ij}^\top R_i^\top R_j\big),$$
$$r_{\Delta v}=R_i^\top(v_j-v_i-g\Delta t_{ij})-\Delta v_{ij},$$
$$r_{\Delta p}=R_i^\top\big(p_j-p_i-v_i\Delta t_{ij}-\tfrac12g\Delta t_{ij}^2\big)-\Delta p_{ij}.$$

残差用协方差 $\Sigma_{ij}$ 加权，就得到一个 IMU 因子。

## 5. 手算：三步预积分

### 5.1 设定

$\Delta t=0.1$ s，共 $3$ 步（$\Delta t_{ij}=0.3$ s）。读数：$\omega_m=(0,0,0.5)$ rad/s（绕 $z$ 以 $0.5$ rad/s 偏航），$a_m=(0.5,\ 0,\ 9.81)$ m/s²。当前偏置估计 $b_g=(0,0,0.01)$，$b_a=0$（示例，非项目实测）。

### 5.2 逐步

每步旋转增量 $\mathrm{Exp}((0.5-0.01)\times0.1)=R_z(0.049)$。

- 步 1：$\Delta p\leftarrow0+0+\tfrac12\cdot I\cdot(0.5,0,9.81)\times0.01=(0.0025,0,0.04905)$；$\Delta v\leftarrow(0.05,0,0.981)$；$\Delta R\leftarrow R_z(0.049)$；
- 步 2：用 $\Delta R=R_z(0.049)$ 旋转加速度读数，$(0.5,0,9.81)\to(0.5\cos0.049,\ 0.5\sin0.049,\ 9.81)=(0.4994,0.0245,9.81)$，依此累加；
- 步 3：同理。最终（Python 复核）：

$$\Delta R_{ij}=R_z(0.147),\quad\Delta v_{ij}=(0.1497,\ 0.00734,\ 2.943),\quad\Delta p_{ij}=(0.02248,\ 0.00061,\ 0.44145).$$

$z$ 分量：$9.81\times0.3=2.943$，$0.5\times9.81\times0.09=0.44145$，符合匀加速公式。

### 5.3 预测下一帧

设 $R_i=I$，$v_i=(1,0,0)$ m/s，$p_i=0$，$g=(0,0,-9.81)$：

- $R_j=R_z(0.147)$；
- $v_j=v_i+g\Delta t_{ij}+\Delta v=(1,0,0)+(0,0,-2.943)+(0.1497,0.00734,2.943)=(1.1497,\ 0.00734,\ 0)$；
- $p_j=v_i\Delta t_{ij}+\tfrac12g\Delta t_{ij}^2+\Delta p=(0.3,0,0)+(0,0,-0.44145)+(0.02248,0.00061,0.44145)=(0.3225,\ 0.00061,\ 0)$。

$z$ 方向恰好抵消：加速度计读数里的 $+9.81$ 被重力 $-9.81$ 抵消，机器人没有上下运动，符合直觉。这个“重力抵消”检验是调试预积分时最好用的第一道检查。

## 6. 偏置更新：一阶修正

### 6.1 问题

优化迭代中，偏置估计 $b$ 会变化。理论上预积分量对 $b$ 有依赖，需要重新积分。但偏置变化通常很小，可以用一阶泰勒展开修正：

$$\Delta R_{ij}(b+\delta b)\approx\Delta R_{ij}(b)\,\mathrm{Exp}\big(J^{R}_{b_g}\delta b_g\big),$$
$$\Delta v_{ij}(b+\delta b)\approx\Delta v_{ij}(b)+J^{v}_{b_g}\delta b_g+J^{v}_{b_a}\delta b_a,$$
$$\Delta p_{ij}(b+\delta b)\approx\Delta p_{ij}(b)+J^{p}_{b_g}\delta b_g+J^{p}_{b_a}\delta b_a.$$

这些雅可比在积分过程中一并递推（右雅可比 $J_r$ 来自 $\mathrm{Exp}$ 对扰动的导数，见 [李群李代数章](../01_数学基础/06_李群李代数与流形优化入门.md) 第 9 节）：

$$J^{R}\leftarrow\mathrm{Exp}((\omega-b_g)\Delta t)^\top J^{R}-J_r((\omega-b_g)\Delta t)\Delta t,$$
$$J^{v}_{b_g}\leftarrow J^{v}_{b_g}-\Delta R\,(a-b_a)^\wedge J^{R}\Delta t,\quad J^{v}_{b_a}\leftarrow J^{v}_{b_a}-\Delta R\,\Delta t,$$
$$J^{p}_{b_g}\leftarrow J^{p}_{b_g}+J^{v}_{b_g}\Delta t-\tfrac12\Delta R\,(a-b_a)^\wedge J^{R}\Delta t^2,\quad J^{p}_{b_a}\leftarrow J^{p}_{b_a}+J^{v}_{b_a}\Delta t-\tfrac12\Delta R\,\Delta t^2.$$

（更新顺序同样用“更新前”的值。）

### 6.2 数值验证

沿用上面的例子。雅可比：$J^R$ 的 $zz$ 元素为 $-0.3$（偏航角对 $b_{g,z}$ 的灵敏度 $-\sum\Delta t=-0.3$ rad per rad/s，符合直觉）；$J^v_{b_g}$ 和 $J^p_{b_g}$ 的主元素是 $x$–$y$ 间的 $\pm0.294$ 和 $\pm0.0245$。

偏置 $b_{g,z}$ 增加 $\delta=0.002$ rad/s：

- 偏航 $\Delta R$：重新积分 $0.1464$ rad，一阶修正 $0.1464$ rad；
- $\Delta v$ 误差最大 $5\times10^{-9}$ m/s，$\Delta p$ 误差 $3.5\times10^{-10}$ m；

$\delta=0.05$ rad/s（25 倍）：偏航仍一致，$\Delta v$ 误差 $3.1\times10^{-6}$ m/s，$\Delta p$ 误差 $2.2\times10^{-7}$ m（Python 复核）。加速度计偏置 $\delta b_{a,x}=0.05$ m/s²：一阶修正与重积分结果一致到显示位数。结论：**偏置变化较小时，一阶修正非常精确**；当 $\delta b$ 变大（实践中以偏置估计变化超过阈值为准）则应重新积分。

## 7. 噪声传播与协方差

### 7.1 噪声怎样进入预积分量

假设测量噪声 $n_g,n_a$ 为离散白噪声，标准差 $\sigma_{gd}=N_g/\sqrt{\Delta t}$，$\sigma_{ad}=N_a/\sqrt{\Delta t}$（$N_g,N_a$ 为连续噪声密度，见 [随机信号…Allan 方差](../04_信号与系统/04_随机信号_自相关_功率谱_白噪声与Allan方差.md)）。用一阶近似，把预积分误差写成噪声的线性函数，递推地传播 $9\times9$ 协方差 $\Sigma$：

$$\Sigma_{k+1}=A_k\Sigma_kA_k^\top+B_kQ_nB_k^\top,$$

其中误差状态为 $[\delta\phi,\delta v,\delta p]$，$A_k,B_k$ 是线性化矩阵，$Q_n=\mathrm{diag}(\sigma_{gd}^2I,\sigma_{ad}^2I)$。

### 7.2 数值验证

取 $N_g=2\times10^{-4}$ rad/√s，$N_a=2\times10^{-3}$ m/s²/√Hz，$T=0.3$ s，Monte Carlo 2 万次：

| 量 | 仿真标准差 | 解析近似 |
|---|---|---|
| 旋转 $\delta\phi$ | $1.08$–$1.10\times10^{-4}$ rad | $N_g\sqrt T=1.095\times10^{-4}$ |
| 速度 $\delta v$ | $1.10$–$1.11\times10^{-3}$ m/s | $N_a\sqrt T=1.095\times10^{-3}$ |
| 位置 $\delta p$ | $1.86$–$1.89\times10^{-4}$ m | $N_aT^{3/2}/\sqrt3=1.90\times10^{-4}$ |

读法：旋转不确定性按 $\sqrt T$ 增长（ARW）；速度同理；位置误差按 $T^{3/2}$ 增长（速度随机游走被积分）。当 $T$ 变大 10 倍，位置不确定性大 $10^{1.5}=31.6$ 倍。**这就是 IMU 只能“短时间可靠”的数学表达**，也解释了为什么 VIO/LIO 需要不断用视觉/激光测量约束。

### 7.3 偏置的因子

偏置本身是随机游走：$b_{k+1}=b_k+\sigma_{bw}\sqrt{\Delta t_{ij}}\,\eta$。在因子图里，相邻关键帧的偏置之间加一个“偏置随机游走因子”，使偏置缓慢变化而不被锁死。

## 8. 因子图：从概率到最小二乘

### 8.1 因子图

因子图是一个二部图：**变量节点**（要估计的量，如 $x_i$、$b_i$、路标 $l_m$）与**因子节点**（概率约束，如先验、IMU、视觉观测、回环）。联合后验概率分解为因子的乘积：

$$p(X\mid Z)\propto\prod_f\phi_f(X_f).$$

### 8.2 最大后验 = 非线性最小二乘

假设各因子是高斯：$\phi_f\propto\exp(-\tfrac12\lVert r_f(X_f)\rVert^2_{\Sigma_f})$，则

$$X^\ast=\arg\max p(X\mid Z)=\arg\min_X\sum_f\lVert r_f(X_f)\rVert^2_{\Sigma_f}.$$

用高斯—牛顿或 LM 求解：每次迭代在当前线性化点解 $H\delta=-g$，$H=J^\top\Sigma^{-1}J$。变量节点只与相关因子连接，所以 $H$ 稀疏。位姿更新用流形上的 $\boxplus$（$R\leftarrow R\,\mathrm{Exp}(\delta\phi)$），见李群章。

### 8.3 滤波与图优化的比较

| 对比 | EKF/滤波 | 因子图/平滑 |
|---|---|---|
| 状态 | 当前状态 | 一段历史（窗口或全部） |
| 线性化 | 一次，固定在预测点 | 可重复线性化，迭代收敛 |
| 历史信息 | 边缘化后丢弃 | 保留，可重新线性化 |
| 回环 | 难以处理 | 自然（加一个因子） |
| 计算 | 小而稳定 | 较大，稀疏求解 |
| 精度 | 一次线性化误差累积 | 通常更高 |

## 9. 手算：三节点位姿链加回环

### 9.1 问题

一维位置 $x_0,x_1,x_2$。因子：先验 $x_0=0$（$\sigma=0.01$）；里程计 $x_1-x_0=1.0$、$x_2-x_1=1.0$（$\sigma=0.1$）；回环（直接测量 $x_2-x_0$）$=1.9$（$\sigma=0.1$）。注意里程计累积给出 $2.0$，回环给出 $1.9$，二者冲突 $0.1$ m，优化要把这个冲突按置信度分摊。

### 9.2 信息矩阵与解

权重 $w_0=1/0.01^2=10^4$，其余 $w=100$。法方程 $H=A^\top WA$：

$$H=\begin{bmatrix}10200&-100&-100\\-100&200&-100\\-100&-100&200\end{bmatrix},\qquad \eta=A^\top Wb=\begin{bmatrix}-290\\0\\290\end{bmatrix}.$$

解 $Hx=\eta$：$x=(0,\ 0.9667,\ 1.9333)$，标准差 $(0.01,\ 0.0823,\ 0.0823)$（Python 复核）。不加回环时：$x=(0,1,2)$，标准差 $(0.01,0.1005,0.1418)$。

### 9.3 读法

- 回环把 $x_2$ 从 $2.0$ 拉到 $1.933$，并把冲突误差 $0.1$ 平摊到两段里程计（每段 $-0.033$）；
- 回环使 $x_2$ 的标准差由 $0.142$ 降到 $0.082$——末端不确定性因为多了一条约束而减小；
- 矩阵 $H$ 在 $(0,2)$ 位置出现了 $-100$：**回环增加了一个非相邻节点间的连接**，在大规模系统里就是“填充”的来源。

## 10. 边缘化与滑动窗口

### 10.1 为什么要边缘化

滑动窗口 VIO/LIO 只保留最近的若干关键帧。老的关键帧不能简单丢弃（会丢掉它们带来的信息），而应该**边缘化**：把它们的信息通过概率边缘化转移到仍保留的变量上，形成一个新的先验因子。

### 10.2 Schur 补

把变量分成保留的 $a$ 与要边缘化的 $m$：$H=\begin{bmatrix}H_{aa}&H_{am}\\H_{ma}&H_{mm}\end{bmatrix}$，$\eta=\begin{bmatrix}\eta_a\\\eta_m\end{bmatrix}$。消去 $m$ 后，保留变量的信息矩阵与信息向量：

$$H_{aa}^\ast=H_{aa}-H_{am}H_{mm}^{-1}H_{ma},\qquad \eta_a^\ast=\eta_a-H_{am}H_{mm}^{-1}\eta_m.$$

### 10.3 手算

承 §9，边缘化 $x_1$，保留 $(x_0,x_2)$：$H_{mm}=200$，$H_{am}=[-100,\ -100]^\top$，

$$H_{aa}^\ast=\begin{bmatrix}10200&-100\\-100&200\end{bmatrix}-\frac{1}{200}\begin{bmatrix}10^4&10^4\\10^4&10^4\end{bmatrix}=\begin{bmatrix}10150&-150\\-150&150\end{bmatrix},$$

$\eta_a^\ast=(-290,\ 290)$。解之得 $(x_0,x_2)=(0,\ 1.9333)$——与完整问题的 $(0,\ 1.9333)$ **完全一致**（Python 复核）。边缘化得到的先验在 $x_0$–$x_2$ 之间产生了 $-150$ 的耦合（原来只有回环的 $-100$，加上经过 $x_1$ 传递的 $-50$）。

### 10.4 注意：线性化点固定与一致性

边缘化之后，那个先验因子基于边缘化时刻的线性化点。之后保留变量的估计继续变化，但先验因子的雅可比不能再更新——这会引入线性化误差，甚至导致不一致（后文可观性章节会讲，FEJ：First-Estimate Jacobian 的思想）。另外，边缘化会把原本稀疏的连接变成稠密的先验块（上面的 $-150$），窗口越大、边缘化越多，稠密块越大。

## 11. 增量平滑与 iSAM2 的思想

### 11.1 问题

批量优化每次都重新解整个问题，随着轨迹变长越来越慢。滑动窗口牺牲了长期一致性。增量平滑（incremental smoothing）的想法：新增因子时只更新受影响的部分。

### 11.2 iSAM2 的核心

Kaess 等人提出的 iSAM2（文献年份与细节未核对）：（1）把线性化后的因子图经变量消元变成 **Bayes 树**（一种团树）；（2）新增因子只影响树的一部分，只需重新计算那部分团；（3）仅当变量的估计变化超过阈值时才重新线性化（“部分重线性化”）；（4）自适应变量排序以控制填充。结果是：计算量接近滑动窗口，但保留全部历史，并能处理回环。

### 11.3 比较

| 方法 | 保留历史 | 计算量 | 一致性 | 典型用途 |
|---|---|---|---|---|
| EKF | 否 | 小 | 线性化累积 | 轻量里程计 |
| 滑动窗口 + 边缘化 | 窗口内 | 受限、稳定 | 边缘化引入近似 | VIO 实时 |
| 固定滞后平滑 | 滞后段 | 受限 | 好 | 带延迟的导航 |
| 批量图优化 | 全部 | 大 | 最好 | 离线、回环后的全局优化 |
| iSAM2 增量平滑 | 全部 | 增量 | 好 | 长时间在线建图 |

## 12. 在 VIO / LIO 中的位置

- **紧耦合 VIO**（如 VINS 系列，具体论文与年份未核对）：状态含窗口内的关键帧位姿、速度、偏置和视觉特征深度；因子含 IMU 预积分因子、视觉重投影因子、边缘化先验；
- **LIO**（如 LIO-SAM，细节未核对）：用 IMU 预积分因子给出相邻激光帧之间的运动先验，激光匹配因子、GNSS 因子、回环因子一起放在因子图里；预积分也用来给激光点云去畸变；
- **松耦合**：先分别用视觉/激光求位姿，再用滤波融合；信息损失更多，但实现简单。

紧耦合系统启动时还有一个绕不开的问题——**初始化**：重力方向、尺度（单目视觉）、初始速度和陀螺偏置在开机时都是未知的。常见做法是在初始化窗口里先用视觉或激光得到一段无尺度/有尺度的相对运动，再把它与预积分量对齐：用陀螺预积分与视觉旋转之差解出陀螺偏置，用线性最小二乘解出重力方向、速度和尺度，最后细化重力的大小与方向。初始化质量直接决定后续优化是否收敛，如果静止或匀速运动，加速度没有激励，尺度和重力方向就不可观，这与后面“可观性分析”一章的结论相呼应（见 [估计一致性、可观性分析、NEES 与 NIS](./10_估计一致性_可观性分析_NEES与NIS.md)）。

这一章的内容就是“紧耦合”里的 IMU 部分。相关的特征点、对极几何、PnP 与 BA 见 [视觉里程计：特征、对极几何、PnP、BA](../07_SLAM与建图/02_视觉里程计_特征_对极几何_PnP_BA.md)。

## 13. 在本项目里怎么用

> 说明：CV 中没有写 VIO/LIO 与预积分；P1/P3/P7 做的是 IMU/AHRS 姿态估计与互补滤波，P5 做的是 EKF/AMCL 融合。预积分与因子图属于延伸知识，面试中说成“我了解 IMU 在紧耦合系统里怎么用，以及它与我做过的滤波的关系”。

- **P1 / P3 / P7 姿态估计**：互补滤波或 Mahony 的姿态更新 $R\leftarrow R\,\mathrm{Exp}((\omega-b+k\cdot e)\Delta t)$，就是预积分中 $\Delta R$ 递推的“带反馈”版本。预积分只积分陀螺不修正；互补滤波用加速度计的重力方向修正漂移（见 [互补滤波与 Mahony AHRS](./02_互补滤波与Mahony_AHRS.md)）。
- **P5 里程计 + IMU**：EKF 中 IMU 提供偏航角速度，轮速提供前向速度；若升级为因子图，IMU/轮速里程计会成为相邻位姿之间的因子，激光/AMCL 提供绝对约束，回环提供全局约束（见 [里程计与传感器融合](./04_里程计与传感器融合.md)、[P5 EKF 与 AMCL 协同](../../01_项目课程/P5_车式移动机器人自主导航/06_EKF与AMCL协同_TF分工与连续状态.md)）。
- **P7 速度估计**：轮腿机器人的速度由轮速、关节速度和 IMU 姿态融合（见 [P7 状态估计与速度、位置](../../01_项目课程/P7_可变腿长轮腿机器人/08_状态估计与速度_位置.md)）；预积分的“把高频数据压缩成相对运动约束”的思想可以借来理解多速率融合。
- **P6 多源数据**：时间戳对齐是预积分能用的前提：IMU 时间与关键帧时间若有偏差，要么插值，要么在预积分里加上时间偏移（见 [时间同步、坐标系一致性与数据有效性](./06_时间同步_坐标系一致性与数据有效性.md)）。

## 14. 常见错误与故障现象

| 现象 | 可能原因 | 处理 |
|---|---|---|
| 静止时预积分预测在 $z$ 方向漂移 | 重力方向/符号错，或加速度计比力约定搞反 | 用“重力抵消”检验；明确 $a_m$ 含不含重力 |
| 预积分与直接积分结果不同 | 递推顺序用了更新后的值 | 先 $\Delta p$，再 $\Delta v$，最后 $\Delta R$ |
| 偏置估计改变后残差突变 | 未做一阶修正，或偏置变化过大需重积分 | 实现雅可比修正，设置重积分阈值 |
| 残差 $r_{\Delta R}$ 在 $\pm\pi$ 附近跳变 | $\mathrm{Log}$ 的分支处理 | 用稳健的 $\mathrm{Log}$，小角度分支 |
| 协方差过小，优化过度信任 IMU | 噪声参数取自静态标定，未考虑振动 | 放大噪声参数，检查 NIS/一致性 |
| 时间戳偏差导致估计滞后 | IMU 与相机时钟不同步 | 标定时间偏移，插值 |
| 边缘化后估计质量下降 | 先验因子基于旧线性化点 | FEJ，或增大窗口 |
| 计算时间随轨迹增长 | 批量优化，稠密填充 | 滑动窗口，iSAM2，变量排序 |
| 优化发散 | 初值差、尺度/重力未对齐 | 初始化：估计重力方向与偏置 |

## 15. 面试怎么问、怎么答

**Q1. What is IMU preintegration and why do we need it?**

> Between two keyframes there are many high-rate IMU samples. Integrating them directly from the keyframe state means redoing the integration every time the optimiser changes that state. Preintegration instead integrates the samples in the body frame of the first keyframe, giving relative rotation, velocity and position quantities that do not depend on the absolute pose, only on the readings and the bias. The optimiser then uses a single IMU factor between keyframes, and when the bias estimate changes slightly I correct the preintegrated terms with first-order Jacobians instead of re-integrating.

中文：两个关键帧之间有大量高频 IMU 数据；直接从关键帧状态积分，每次优化改变该状态就得重积分；预积分在第一个关键帧的机体系里积分，得到与绝对位姿无关、只依赖读数和偏置的相对旋转、速度、位置量；优化时每对关键帧只用一个 IMU 因子，偏置小幅变化时用一阶雅可比修正而不重积分。

**Q2. How would you check that your preintegration implementation is correct?**

> First, a static test: with the sensor at rest and level, the predicted velocity and position should not drift in z, because the accelerometer's plus g cancels the gravity term. Second, compare to brute-force integration for random motion; they must match to numerical precision. Third, compare the first-order bias correction to re-integration for small bias changes. Fourth, check the covariance with a Monte Carlo simulation: the standard deviations should scale as root T for rotation and velocity and T to the 1.5 for position.

中文：先做静止测试：水平静止时预测速度/位置不应在 $z$ 向漂移（加速度计 $+g$ 与重力项抵消）；其次与随机运动下的暴力积分比较应数值一致；再比较小偏置变化下一阶修正与重积分；最后 Monte Carlo 检验协方差：旋转和速度标准差按 $\sqrt T$，位置按 $T^{1.5}$。

**Q3. Why are factor graphs preferred over the EKF for SLAM and VIO?**

> A factor graph keeps a window or the whole history of states and re-linearises them, so linearisation errors do not accumulate irreversibly the way they do in an EKF. Loop closures and other non-sequential constraints are just additional factors. The normal equations are sparse, so a sparse Cholesky solver is fast. The price is higher computation and the need to manage marginalisation, but incremental solvers like iSAM2 or a sliding window keep it real-time.

中文：因子图保留窗口内或全部历史状态并可重新线性化，线性化误差不会像 EKF 那样不可逆累积；回环等非序列约束只是多一个因子；法方程稀疏，稀疏 Cholesky 很快；代价是计算量更高、要处理边缘化，但 iSAM2 或滑动窗口可保持实时。

**Q4. What does marginalisation do and what is its downside?**

> Marginalisation removes old variables from the window while keeping the information they carried, by applying a Schur complement to the information matrix. The result is a dense prior factor on the remaining variables. The downside is that the prior is frozen at the linearisation point of the time it was created, which can cause inconsistency, and it makes the problem denser. In a small example, marginalising the middle pose of a three-pose chain with a loop closure gave a coupling of minus 150 between the two remaining poses and the same solution as the full problem.

中文：边缘化通过信息矩阵的 Schur 补移除旧变量并保留其信息，得到保留变量上的稠密先验因子；缺点是先验被冻结在创建时的线性化点，可能不一致，且使问题变稠密；小例子中边缘化三位姿链中间位姿得到两端之间 −150 的耦合，解与完整问题一致。

**Q5. Why does the IMU alone drift, and by how much does the position uncertainty grow?**

> White noise on the gyro gives an angle error that grows like root t, and white noise on the accelerometer gives a velocity error growing like root t and a position error growing like t to the 1.5. On top of that, biases are integrated once or twice. With the example noise levels, 0.3 s of preintegration gave 0.2 millimetre position uncertainty, but extended to 30 s it would be about 1000 times larger from the noise alone, which is why visual or lidar measurements must constrain the estimate.

中文：陀螺白噪声使角度误差按 $\sqrt t$ 增长，加速度计白噪声使速度误差按 $\sqrt t$、位置误差按 $t^{1.5}$ 增长，偏置还会被一次或两次积分；示例噪声下 0.3 s 预积分位置不确定性约 0.2 mm，延长到 30 s 仅噪声就大约放大 1000 倍，所以需要视觉或激光测量约束。

## 16. 本章要点速记

1. 预积分：在关键帧 $i$ 的机体系里积分 IMU，得到与 $R_i,v_i,p_i$ 无关的 $\Delta R,\Delta v,\Delta p$。
2. 递推：先 $\Delta p$，再 $\Delta v$，最后 $\Delta R$；加速度计测比力，重力在预测式中单独加。
3. 预测：$R_j=R_i\Delta R$，$v_j=v_i+g\Delta t+R_i\Delta v$，$p_j=p_i+v_i\Delta t+\tfrac12g\Delta t^2+R_i\Delta p$。
4. 偏置变化小：一阶雅可比修正；变化大：重积分。示例 $\delta b_g=0.05$ 时误差仅 $10^{-6}$ 量级。
5. 噪声：旋转/速度 $\propto\sqrt T$，位置 $\propto T^{3/2}$（ARW → $N\sqrt T$）。
6. 因子图：变量 + 因子，MAP = 非线性最小二乘，$H=J^\top\Sigma^{-1}J$ 稀疏。
7. 示例：先验 + 2 里程计 + 回环冲突 0.1 m，$x_2$ 由 2.0 修正到 1.933，标准差 0.142 → 0.082。
8. 边缘化 = Schur 补，$H^\ast_{aa}=H_{aa}-H_{am}H_{mm}^{-1}H_{ma}$，产生稠密先验，要注意线性化点（FEJ）。
9. iSAM2：Bayes 树 + 部分重线性化，增量更新，保留历史。
10. 紧耦合 VIO/LIO = IMU 预积分因子 + 视觉/激光因子 + 边缘化先验。

## 17. 自测题

1. 静止水平的 IMU，加速度计读数是多少，预积分后的 $\Delta v_{ij}$（$\Delta t_{ij}=1$ s）是多少？（读数 $(0,0,+9.81)$；$\Delta v=(0,0,9.81)$，与 $g\Delta t$ 抵消，预测速度不变。）
2. $\Delta R_{ij}$ 的偏置雅可比为什么近似 $-\Delta t_{ij}$（单轴）？（偏航角 $=\sum(\omega-b)\Delta t$，对 $b$ 求导 $-\sum\Delta t$。）
3. 位姿图中为什么回环降低不确定性？（多一个约束，信息矩阵增加，协方差减小。）
4. Schur 补边缘化中，$H_{mm}$ 必须满足什么条件？（可逆，通常正定。）
5. 预积分的位置不确定性从 $T=1$ s 变到 $T=4$ s 增大几倍？（$4^{1.5}=8$ 倍。）
6. 为什么 FEJ 要求边缘化先验用旧线性化点？（保证被边缘化的不可观方向不被虚假地"观测到"，维持一致性。）

## 18. 相关链接

- 上一章：[UKF、信息滤波与卡尔曼平滑](./07_UKF_信息滤波_卡尔曼平滑.md)；下一章：[误差状态卡尔曼 ESKF 与流形上的滤波](./09_误差状态卡尔曼ESKF与流形上的滤波.md)
- 相关基础：[李群李代数与流形优化入门](../01_数学基础/06_李群李代数与流形优化入门.md)、[SLAM问题_前端后端_位姿图优化](../07_SLAM与建图/01_SLAM问题_前端后端_位姿图优化.md)、[传感器：IMU、编码器、激光雷达](./01_传感器_IMU_编码器_激光雷达.md)
- 项目：[P1](../../01_项目课程/P1_AIRS球形机器人平衡控制/README.md)、[P5](../../01_项目课程/P5_车式移动机器人自主导航/README.md)、[P7](../../01_项目课程/P7_可变腿长轮腿机器人/README.md)
- 模块说明：[06_状态估计 README](./README.md)
