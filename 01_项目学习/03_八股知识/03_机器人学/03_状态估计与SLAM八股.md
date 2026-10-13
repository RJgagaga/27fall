# 机器人学八股 03：状态估计与 SLAM

> **一句话定位**：状态估计回答"机器人现在在哪里、姿态如何、速度多少、这个数有多可信"。你的 CV 里有 Mahony 互补滤波（P1/P3 的 IMU/AHRS）、EKF/AMCL（P5）、里程计（P2/P5）、时间与坐标系一致性（P5/P6），所以本文把滤波器、定位、SLAM 和一致性问题讲到可手算、可对比、可追问。

**本章学完你能：**
- 推导卡尔曼滤波的预测与更新，解释 $Q,R,P,K$ 的含义和调节方向；
- 区分 KF/EKF/UKF/粒子滤波，说清各自适用与失效情形；
- 讲清互补滤波、Mahony、Madgwick 与 AHRS 的原理，解释 yaw 为何会漂；
- 写出差速里程计公式，解释漂移来源与融合策略；
- 讲清 AMCL 的运动模型、传感器模型、重采样、KLD 与"绑架"问题；
- 对比 gmapping、Cartographer、LIO-SAM、ORB-SLAM 的传感器、后端与适用场景；
- 解释回环检测、位姿图优化、不可观性与滤波一致性；
- 说明时间戳、坐标系和运动方向不一致如何引起漂移与跳变（对应 P5 CV bullet）。

**前置知识**：
- [传感器：IMU/编码器/激光雷达](../../02_基础知识/05_状态估计/01_传感器_IMU_编码器_激光雷达.md)
- [互补滤波与 Mahony AHRS](../../02_基础知识/05_状态估计/02_互补滤波与Mahony_AHRS.md)
- [卡尔曼滤波与 EKF](../../02_基础知识/05_状态估计/03_卡尔曼滤波与EKF.md)
- [里程计与传感器融合](../../02_基础知识/05_状态估计/04_里程计与传感器融合.md)
- [粒子滤波与 AMCL](../../02_基础知识/05_状态估计/05_粒子滤波与AMCL.md)
- [时间同步、坐标系一致性与数据有效性](../../02_基础知识/05_状态估计/06_时间同步_坐标系一致性与数据有效性.md)
- [概率统计与高斯分布](../../02_基础知识/01_数学/03_概率统计与高斯分布.md)

**优先级**：★★★ 必会；★★ 常见；★ 加分。

**目录**
1. 卡尔曼家族（Q1–Q12）
2. 姿态估计：互补滤波、Mahony、AHRS（Q13–Q20）
3. 里程计与多传感器融合（Q21–Q30）
4. AMCL 与定位（Q31–Q37）
5. SLAM 与回环、图优化（Q38–Q49）
6. 不可观性、一致性与工程问题（Q50–Q58）
7. 要点速记、自测题、相关链接

---

## 一、卡尔曼家族

### Q1. 卡尔曼滤波解决什么问题？直觉是什么？（What does a Kalman filter do?）★★★
**答案要点**
- 问题：状态不能直接测，或测量有噪声；有一个带噪声的运动模型（预测）和一个带噪声的传感器（观测），如何融合得到最优估计。
- 直觉：新估计 = 预测 + 增益 × (测量 − 预测的测量)。增益 $K$ 取决于"相信模型多一些还是相信传感器多一些"：模型不确定性大 ($P$ 大) → $K$ 大，更相信测量；测量噪声大 ($R$ 大) → $K$ 小，更相信模型。
- 在线性高斯假设下是**最小均方误差最优估计**；输出不仅有均值 $\hat x$，还有协方差 $P$（不确定度）。
- 一维例（教学示例）：预测位置 $10$，方差 $4$；测量位置 $12$，方差 $1$。$K=4/(4+1)=0.8$，$\hat x=10+0.8(12-10)=11.6$，$P=(1-0.8)\cdot4=0.8$，比任一来源都小。
**English short answer**
A Kalman filter fuses a motion-model prediction with a noisy measurement, weighting each by its uncertainty. The new estimate is the prediction plus a gain times the innovation, and the gain is large when the model is uncertain and small when the sensor is noisy. For linear systems with Gaussian noise it is the optimal estimator, and it also tells how uncertain the estimate is.
**追问方向**：为什么融合后方差更小；$K$ 的两种极端；创新（innovation）含义。

### Q2. 卡尔曼滤波五个公式？（KF equations）★★★
**答案要点**
模型：$x_k=Fx_{k-1}+Bu_k+w_k$，$w\sim\mathcal N(0,Q)$；$z_k=Hx_k+v_k$，$v\sim\mathcal N(0,R)$。
- 预测：
$$\hat x_k^-=F\hat x_{k-1}+Bu_k,\qquad P_k^-=FP_{k-1}F^{\top}+Q$$
- 更新：
$$K_k=P_k^-H^{\top}(HP_k^-H^{\top}+R)^{-1}$$
$$\hat x_k=\hat x_k^-+K_k(z_k-H\hat x_k^-),\qquad P_k=(I-K_kH)P_k^-$$
- 数值稳定写法 Joseph 形式：$P=(I-KH)P^-(I-KH)^{\top}+KRK^{\top}$，保证 $P$ 对称正定。
- 推导思路：在高斯下，最小化后验协方差迹 $\mathrm{tr}(P_k)$，对 $K$ 求导为零即得上式。
**English short answer**
Predict the state and inflate the covariance with the process noise, then compute the gain from the predicted covariance and the measurement noise, correct the state with the innovation, and shrink the covariance. In practice I use a numerically stable form to keep the covariance symmetric and positive definite.
**追问方向**：$P$ 为何要对称正定；$K$ 推导；创新协方差 $S=HPH^{\top}+R$ 的用途（门限，见 Q55）。

### Q3. $Q$ 和 $R$ 怎么调？现象如何判断？（Tuning Q and R）★★★
**答案要点**
- $R$：传感器噪声方差，可由静止时数据统计得到（IMU 静止方差、编码器测量方差）；
- $Q$：过程噪声，表示"模型不确定性"，难以直接测，靠调：
  - $Q$ 偏大：滤波器相信模型少，估计跟随测量，噪声大；
  - $Q$ 偏小：滤波器太"自信"，对真实变化反应迟钝（滞后），协方差快速收敛。
- 判据：创新序列应为零均值白噪声，且 $\nu^{\top}S^{-1}\nu$ 服从 $\chi^2$（NIS 检验，见 Q57）。
- 实际：先固定 $R$ 为测得值，调 $Q$ 的对角线比例，观察响应与噪声的折中。
- 【待你补全：P5 的 EKF 中 $Q/R$ 的来源与最终量级、如何判断调好】
**English short answer**
I estimate R from the sensor's static noise, and tune Q to reflect how much I trust the motion model. A large Q makes the filter follow the noisy measurements, a small Q makes it lag. I check that the innovations look like zero-mean white noise with a consistent covariance.
**追问方向**：NIS/NEES；自适应噪声估计。

### Q4. 卡尔曼增益趋于 0 或 1 意味着什么？（Limiting cases of K）★★
**答案要点**
- $R\to0$（测量完美）：$K\to H^{-1}$，估计等于测量；
- $R\to\infty$ 或 $P^-\to0$：$K\to0$，估计等于预测；
- 稳态：线性时不变系统 $P$ 收敛到 Riccati 解，$K$ 收敛为常数，可退化为**稳态卡尔曼滤波**（常增益），计算量低，嵌入式常用。
- 与互补滤波关系：常增益卡尔曼 ≈ 互补滤波（Q13）。
**English short answer**
If the measurement is perfect, the gain makes the estimate equal to the measurement. If the model is perfect or the sensor useless, the gain is zero. For a time-invariant system, the gain converges to a constant, giving a steady-state Kalman filter, which is essentially a complementary filter.
**追问方向**：稳态增益求法（DARE）。

### Q5. 为什么需要 EKF？EKF 怎么做？（Extended Kalman filter）★★★
**答案要点**
- 实际运动/观测模型非线性：$x_k=f(x_{k-1},u_k)+w$，$z_k=h(x_k)+v$。EKF 在当前估计点**一阶泰勒线性化**：
$$F_k=\left.\frac{\partial f}{\partial x}\right|_{\hat x_{k-1}},\ H_k=\left.\frac{\partial h}{\partial x}\right|_{\hat x_k^-}$$
  预测用非线性 $f$ 推均值，用 $F_kP F_k^{\top}+Q$ 推协方差；更新同理用 $H_k$，创新用非线性残差 $z-h(\hat x^-)$。
- 差速机器人例（教学示例）：状态 $[x,y,\theta]$，输入 $[v,\omega]$，
$$f=\begin{bmatrix}x+vT\cos\theta\\y+vT\sin\theta\\\theta+\omega T\end{bmatrix},\ F=\begin{bmatrix}1&0&-vT\sin\theta\\0&1&vT\cos\theta\\0&0&1\end{bmatrix}$$
- 局限：线性化误差、强非线性/大初始误差时发散；雅可比推错是常见 bug；角度要 wrap 到 $[-\pi,\pi]$（包括创新项的角度差）。
- 项目：P5 融合 IMU、轮速、2D LiDAR（通过 AMCL 位姿）输出连续状态。
**English short answer**
An EKF handles nonlinear models by propagating the mean through the nonlinear function and using the Jacobian of that function to propagate the covariance. It is a first-order approximation, so it can diverge with strong nonlinearity or poor initial guesses. I always wrap angle differences, and I validate the Jacobians against finite differences.
**追问方向**：雅可比数值检查；为什么角度残差要归一化；EKF 的不一致性（Q54）。

### Q6. UKF 比 EKF 好在哪里？（UKF vs EKF）★★
**答案要点**
- UKF 用 $2n+1$ 个 sigma 点穿过非线性函数后重新估计均值与协方差，**不需要雅可比**，精度达二阶（高斯）；
- 缺点：计算量略大；参数 $(\alpha,\beta,\kappa)$；对高度非线性/多峰分布仍不足；
- 选型：模型有不可导环节或雅可比难推时选 UKF；嵌入式实时性强时仍多选 EKF。
**English short answer**
The unscented Kalman filter pushes a small set of sigma points through the nonlinear function and recomputes the mean and covariance, so it needs no Jacobians and is accurate to second order. It costs a bit more compute. For a simple model like a differential-drive car the EKF is usually enough.
**追问方向**：sigma 点生成；Cholesky 分解失败（P 非正定）。

### Q7. 粒子滤波是什么？（Particle filter）★★★
**答案要点**
- 用 $N$ 个加权样本表示任意（非高斯、多峰）后验分布 $p(x_k|z_{1:k})\approx\sum w_k^{(i)}\delta(x-x_k^{(i)})$。
- 算法（SIR）：① 预测：从运动模型采样 $x_k^{(i)}\sim p(x_k|x_{k-1}^{(i)},u_k)$；② 加权：$w^{(i)}\propto p(z_k|x_k^{(i)})$；③ 归一化；④ 重采样。
- 维度灾难：粒子数随状态维数指数增长；适合低维（定位 $[x,y,\theta]$），不适合高维（SLAM 地图）→ 用 Rao-Blackwell 化（gmapping）。
**English short answer**
A particle filter represents the belief with weighted samples. Each step moves the particles with the motion model, weights them by how well they explain the measurement, and resamples. It can represent multimodal distributions, which is great for global localisation, but it scales badly with state dimension.
**追问方向**：权重更新公式；有效粒子数；粒子数如何选。

### Q8. 粒子退化与重采样怎么处理？（Degeneracy and resampling）★★★
**答案要点**
- 退化：多次迭代后少数粒子权重占优；用有效粒子数 $N_{eff}=1/\sum (w^{(i)})^2$ 度量，低于 $N/2$ 时才重采样。
- 重采样方法：多项式、系统（systematic）、分层、低方差重采样（O(N)、方差小，AMCL 采用）。
- 问题：重采样导致粒子贫化（样本多样性下降），可注入随机粒子（应对绑架问题，Q35）。
- 低方差重采样：只抽一个随机数 $r\in[0,1/N)$，等间隔指针 $r+i/N$。
**English short answer**
Over time a few particles get nearly all the weight, which I measure with the effective sample size. When it falls below about half the particle count I resample, usually with low-variance resampling that uses a single random number and evenly spaced pointers. Too frequent resampling kills diversity, so I resample only when needed.
**追问方向**：随机粒子注入；为何低方差好。

### Q9. KF / EKF / UKF / PF 如何选？（Choosing the filter）★★★
| | KF | EKF | UKF | 粒子滤波 |
|---|---|---|---|---|
| 模型 | 线性 | 弱非线性 | 较强非线性 | 任意 |
| 分布 | 高斯 | 高斯 | 高斯 | 任意/多峰 |
| 计算 | 低 | 低 | 中 | 高 |
| 需要雅可比 | 否 | 是 | 否 | 否 |
| 典型用途 | IMU 单轴 | 轮速+IMU 融合 | 姿态/复杂观测 | AMCL 全局定位 |

**English short answer**
Use a Kalman filter for linear Gaussian problems, EKF for mildly nonlinear ones with cheap Jacobians, UKF when linearisation is awkward, and particle filters when the distribution is multimodal, like global localisation. In my navigation stack, I used an EKF for continuous fusion and AMCL, which is particle-based, for map-relative pose.
**追问方向**：为什么定位用粒子，里程计融合用 EKF。

### Q10. 误差状态卡尔曼（ESKF）是什么？（Error-state KF）★★
**答案要点**
- 滤波器估计的是**误差状态** $\delta x$（小量），名义状态 $x$ 用 IMU 积分得到；$\delta x$ 接近 0 → 线性化更准；姿态误差用 3 维旋转向量，避免四元数 4 维冗余造成的协方差奇异。
- 流程：IMU 积分名义状态 → 预测 $\delta x$ 协方差 → 观测更新 $\delta x$ → 注入名义状态 → 重置 $\delta x=0$。
- 用于 VIO/LIO（如 FAST-LIO）。
**English short answer**
An error-state filter estimates the small deviation from a nominal trajectory instead of the full state. Because the error is small, the linearisation is accurate, and the orientation error has only three parameters. After each update the error is injected into the nominal state and reset.
**追问方向**：为何比 EKF 好；注入后的协方差重置。

### Q11. 滑动窗口/因子图 vs 滤波？（Smoothing vs filtering）★★
**答案要点**
- 滤波：只保留当前状态与协方差，边缘化旧状态；
- 平滑/批优化：保留多个状态，重复线性化，精度更高、可用回环约束，但计算更大；
- 滑动窗口：折中（VINS-Mono 等）。
**English short answer**
Filtering keeps only the current state and marginalises the past, which is cheap but linearises once. Smoothing keeps a window or the whole trajectory and re-linearises, which is more accurate and can absorb loop closures, at higher cost.
**追问方向**：边缘化；iSAM2。

### Q12. 状态估计中常见的"一致性"问题？（Consistency）★★
**答案要点**
- 估计器的协方差应能真实反映误差。EKF 因线性化点偏、可观性错误，常"过度自信"（协方差偏小）→ 后续异常值不再被拒绝；
- 检验：NEES $\epsilon=(x-\hat x)^{\top}P^{-1}(x-\hat x)\sim\chi^2_n$（有真值时）；NIS $\nu^{\top}S^{-1}\nu\sim\chi^2_m$（无真值）；
- 改进：UKF/ESKF/FEJ（First Estimate Jacobian）、适当膨胀 $Q$。
**English short answer**
A consistent estimator reports a covariance that matches its real error. EKFs tend to be overconfident because of linearisation, which I check with NIS or NEES statistics. Inflating the process noise or using FEJ or error-state formulations helps.
**追问方向**：为什么过度自信危险；膨胀 Q 的代价。

---

## 二、姿态估计

### Q13. 互补滤波为什么能融合陀螺仪和加速度计？（Complementary filter）★★★
**答案要点**
- 陀螺仪：高频准、积分漂移（低频差）；加速度计：测重力方向，低频准、振动/加速运动时高频差。
- 一阶互补滤波：
$$\theta_k=\alpha(\theta_{k-1}+\omega T)+(1-\alpha)\theta_{acc},\quad\alpha=\frac{\tau}{\tau+T}$$
  即对陀螺积分做高通，对加速度计做低通，两者传递函数之和为 1（"互补"）。
- 手算（教学示例）：$\tau=1$ s，$T=0.01$ s → $\alpha=0.990$；陀螺积分给 $10.0^\circ$，加计给 $12.0^\circ$ → $\theta=0.99\cdot10.0+0.01\cdot12.0=10.02^\circ$。
- 等价为常增益卡尔曼滤波（Q4），计算极小，适合 MCU。
- 项目：CV 技能栏写有 *Kalman/Mahony complementary filtering*；P3 云台使用 IMU/AHRS 姿态做角度环反馈，P1 使用 IMU 做平衡反馈。具体哪个项目用哪种滤波器【待你补全】。
**English short answer**
The gyroscope is accurate at high frequency but drifts when integrated, and the accelerometer gives a drift-free gravity direction but is noisy during motion. A complementary filter high-passes the gyro integration and low-passes the accelerometer angle, and the two weights add to one. It is cheap and works very well on a microcontroller.
**追问方向**：时间常数选择；与卡尔曼关系；剧烈运动时加计不可信怎么处理。

### Q14. Mahony 滤波怎么工作？（Mahony filter）★★★
**答案要点**
- 在 $SO(3)$ 上的非线性互补滤波。步骤：
  1. 估计的姿态（四元数 $q$）旋转得到"预测重力方向" $\hat v$（及磁场方向 $\hat w$）；
  2. 与加速度计（磁力计）测得的方向 $a$ 做叉乘得到误差 $e=a\times\hat v$（$+m\times\hat w$）；
  3. PI 修正陀螺：$\omega'=\omega+K_pe+K_i\int e$（$K_i$ 估计陀螺零偏）；
  4. 用 $\omega'$ 积分四元数 $\dot q=\tfrac12 q\otimes(0,\omega')$，归一化。
- 参数：$K_p$ 大 → 更相信加计，收敛快但噪声大；$K_i$ 大 → 更快估计零偏，易振荡。
- 优点：计算轻、无需矩阵求逆、稳定。
- 对运动加速度的处理：可根据 $\|a\|$ 偏离 $g$ 的程度降低加计增益（加速度门限）。
- 【待你补全：你们 Mahony 的 $K_p/K_i$ 取值、IMU 型号与采样率、是否带磁力计】
**English short answer**
Mahony estimates orientation as a quaternion. It predicts where gravity should point given the current estimate, compares it with the accelerometer using a cross product, and feeds that error back into the gyro rate through a PI term. The integral part also learns the gyro bias. It is light, robust, and works well for gimbals and balance control.
**追问方向**：叉乘误差的含义；$K_i$ 的作用；高动态时怎么办。

### Q15. Madgwick 与 Mahony 的差别？（Madgwick vs Mahony）★
**答案要点**
- Madgwick：梯度下降最小化加计/磁力计与预测方向之差，一个参数 $\beta$；
- Mahony：PI 反馈，$K_p,K_i$，有显式零偏估计；
- 二者精度接近，Mahony 在有零偏时略有优势，Madgwick 参数更少。
**English short answer**
Both are lightweight orientation filters. Madgwick uses a gradient-descent correction with one gain, while Mahony uses PI feedback and can estimate gyro bias explicitly.
**追问方向**：何时选哪个。

### Q16. 什么是 AHRS？与 IMU 的区别？（AHRS）★★
**答案要点**
- IMU：陀螺+加计原始数据；AHRS：IMU + 磁力计 + 姿态解算，输出姿态（roll/pitch/yaw）；
- 没有磁力计时，roll/pitch 靠重力可观，**yaw 只能陀螺积分，必然漂移**（不可观）；
- 磁力计受电机/钢结构磁干扰，室内可靠性差，机器人常用其它 yaw 源（轮速、视觉、LiDAR 定位）。
**English short answer**
An IMU gives raw gyro and accelerometer data, while an AHRS adds fusion, and usually a magnetometer, to output attitude. Without a magnetometer, yaw is unobservable from gravity, so it drifts with the gyro bias. Indoors, magnets and motors disturb the compass, so robots often take yaw from wheel odometry or LiDAR localisation.
**追问方向**：yaw 漂移怎么补偿；磁校准。

### Q17. 陀螺仪零偏、噪声怎么处理？（Gyro bias and noise）★★
**答案要点**
- 零偏：上电静止时平均作为初值；温漂；在线估计（Mahony 积分项/EKF 状态）；
- 噪声参数：角度随机游走 ARW、零偏不稳定性（Allan 方差）；
- 工程：启动时静止校准；检测静止（轮速为零 + 角速度小）时更新零偏；
- 加速度计：安装方向校准、尺度因子；振动用低通/陷波。
**English short answer**
I calibrate the gyro bias at start-up while the robot is still, and update it when the robot is detected to be stationary. A filter with a bias state or an integral term tracks slow drifts. For the accelerometer, I low-pass it and check the mounting orientation.
**追问方向**：Allan 方差；温漂。

### Q18. 为什么 pitch/roll 可观，yaw 不可观？（Observability of attitude）★★★
**答案要点**
- 重力方向 $g$ 在体系下的分量由 roll/pitch 决定；绕重力轴的旋转（yaw）不改变加计读数 → 不可观；
- 磁力计提供水平参考，使 yaw 可观；
- 对平衡车/云台来说：仰俯、横滚可靠，yaw 的长期漂移不影响平衡但影响位置/朝向。
**English short answer**
The accelerometer measures gravity, which changes with roll and pitch but not with rotation about the vertical, so yaw is unobservable without a magnetometer or other heading source. For balance, roll and pitch matter most, so a yaw drift does not stop the robot from standing, but it does hurt navigation.
**追问方向**：如何用 LiDAR/视觉修正 yaw。

### Q19. 加速度计在运动中为何不可靠？怎么缓解？（Accelerometer in motion）★★
**答案要点**
- 加计测的是比力 $f=a-g$，机体加速时混入线加速度，估出的"重力方向"倾斜；
- 缓解：降低加计权重（时间常数增大）；根据 $|\|a\|-g|$ 门限；融合轮速加速度进行补偿；
- 平衡机器人快速加速时，倾角估计会出现偏差，这是平衡控制里"状态估计不准"的典型来源。
**English short answer**
An accelerometer measures specific force, so during acceleration the apparent gravity direction tilts. I reduce its weight when the magnitude deviates from one g or compensate using the wheel acceleration. This is a classic reason why the tilt estimate gets biased on a balancing robot.
**追问方向**：如何验证；极端情况。

### Q20. 姿态估计的更新频率与延迟对控制有什么影响？（Latency in estimation）★★
**答案要点**
- 姿态估计延迟等同控制环路延迟，减小相位裕度；滤波器越慢，延迟越大（互补滤波低通时间常数 $\tau$）；
- 采样同步：IMU 中断触发、时间戳；控制回路使用最新的估计，不要混用不同时刻的数据；
- 可在控制前对状态做预测补偿。
**English short answer**
Any delay in the estimate adds to the loop delay and eats phase margin. So I keep the IMU sampling in a fixed-rate interrupt, avoid heavy filtering, and use the latest consistent state in the controller.
**追问方向**：IMU 采样频率选择。

---

## 三、里程计与多传感器融合

### Q21. 差速里程计公式？（Differential-drive odometry）★★★
**答案要点**
- 左右轮速 $v_L,v_R$，轮距 $b$：
$$v=\frac{v_R+v_L}{2},\quad\omega=\frac{v_R-v_L}{b}$$
- 离散积分（中点法）：
$$\theta_{k}=\theta_{k-1}+\omega T,\ x_k=x_{k-1}+vT\cos(\theta_{k-1}+\tfrac{\omega T}{2}),\ y_k=y_{k-1}+vT\sin(\theta_{k-1}+\tfrac{\omega T}{2})$$
- 手算（教学示例）：$v_L=0.9,v_R=1.1$ m/s，$b=0.5$ m → $v=1.0$，$\omega=0.4$ rad/s；$T=0.1$ → $\Delta\theta=0.04$。
- 项目：P2 "integrated wheel motion into odometry for teleoperated and autonomous modes"。
**English short answer**
Forward speed is the average of the wheel speeds and yaw rate is their difference divided by the track width. I integrate with the midpoint heading to reduce error. The odometry is then published as a pose and velocity for the navigation stack.
**追问方向**：圆弧积分精确公式；轮距标定。

### Q22. 里程计误差来源与漂移？（Odometry errors）★★★
**答案要点**
- 系统误差：轮径误差、轮距误差、编码器分辨率、安装不对称；
- 非系统误差：打滑、地面不平、轮子悬空、碰撞；
- 误差累积：位置误差随距离增长，航向误差使位置误差随距离**二次**增长（航向偏 $\delta\theta$，走 $d$ 距离横向误差 $d\delta\theta$）；
- 标定：UMBmark（走正方形双向）校轮径比和轮距；
- 对策：融合陀螺 yaw、LiDAR 匹配、地图定位（AMCL）。
**English short answer**
Odometry has systematic errors from wheel diameter and track width, and random errors from slip and uneven ground. Heading error is the worst because it turns into lateral position error proportional to distance. I reduce it by calibrating and by fusing a gyro and map-based localisation.
**追问方向**：UMBmark；打滑检测。

### Q23. 麦克纳姆/全向轮里程计与差速有何不同？（Mecanum / omni odometry）★★
**答案要点**
- 正运动学（四麦轮，教学约定）：
$$v_x=\frac r4(\omega_1+\omega_2+\omega_3+\omega_4),\ v_y=\frac r4(-\omega_1+\omega_2+\omega_3-\omega_4),\ \omega_z=\frac{r}{4(l_x+l_y)}(-\omega_1+\omega_2-\omega_3+\omega_4)$$
  （符号取决于轮序和滚子方向）；
- 麦轮滚子滑动，横向打滑严重，里程计精度差于差速；
- 三全向轮（ballbot 底盘）用 $3\times3$ 逆矩阵从轮速得机体速度。
- 详见 [差速、麦克纳姆、全向轮与车辆运动学](../../02_基础知识/02_力学与建模/04_差速_麦克纳姆_全向轮与车辆运动学.md)。
**English short answer**
For Mecanum or omni wheels, the body velocity is a linear map of the wheel speeds, and its form depends on the wheel layout. The side slip of the rollers makes odometry noticeably worse than with a differential drive, so I rely on other sensors to correct it.
**追问方向**：符号确定；三轮矩阵条件数。

### Q24. 松耦合与紧耦合融合有何区别？（Loosely vs tightly coupled）★★
**答案要点**
- 松耦合：各传感器各自先解算出位姿/速度，再融合（如把 AMCL 位姿当观测进 EKF）；简单、模块化，但丢失信息与相关性；
- 紧耦合：直接融合原始测量（IMU 预积分 + 激光点云约束）进同一优化/滤波（LIO-SAM、FAST-LIO），精度高、实现复杂。
**English short answer**
In loose coupling each sensor produces its own pose estimate first and the estimates are fused afterwards. In tight coupling the raw measurements enter one estimator. Loose coupling is simpler and more modular, which is why I used an EKF with a pose from the localisation module.
**追问方向**：为什么紧耦合更好；相关噪声重复计数。

### Q25. IMU + 轮速 + LiDAR 的 EKF 怎么设计？（EKF design for IMU, wheel and LiDAR）★★★
**答案要点**（典型做法）
- 状态：$[x,y,\theta,v,\omega,b_g]$（可含加计偏置）；
- 预测：用轮速/IMU 做运动模型；
- 观测：① 轮速给 $v$；② IMU 给 $\omega$（陀螺）；③ LiDAR 定位（AMCL/scan matching）给 $(x,y,\theta)$，**噪声较大但无漂移**；
- 关键：各传感器的**时间戳对齐**、**坐标系变换**（LiDAR→base_link）、**测量延迟**（激光定位结果比实时晚几十毫秒，需要用历史状态更新再重放）、**协方差合理设定**；
- 输出连续的 `odom → base_link`，AMCL 修正体现在 `map → odom`（跳变放在 map→odom，保证 odom 连续，见 Q30）。
- CV：*fused IMU, wheel encoders and 2D LiDAR for map-based localisation, handled drift and pose jumps from inconsistent timing, frames and motion direction*。
- 【待你补全：你们实际的状态向量、融合包（如 robot_localization）与话题、各传感器噪声设置、遇到的"运动方向不一致"具体表现与修复】
**English short answer**
A typical design has pose, velocity and gyro bias as the state. The wheels and the gyro give velocity and yaw rate, and the map-based localisation gives a drift-free but noisier absolute pose. The tricky parts are timestamps, frame transforms, and latency. I keep the odom frame continuous and let the map-to-odom transform absorb the localisation corrections.
**追问方向**：延迟测量处理；协方差设置；重复计数。

### Q26. 时间戳不一致会造成什么现象？如何处理？（Time synchronisation problems）★★★
**答案要点**
- 现象：转弯时位姿"抖动/拖尾"、激光点云与地图错位、融合结果后跳；
- 原因：不同传感器的采样时间≠消息到达时间；链路延迟；不同时钟（IMU 板与主机）；TF 查询时间用 `now` 而不是消息时间；
- 处理：使用传感器采样时间戳；时钟同步（PTP/NTP/硬件触发）；TF 缓冲与插值（查询消息时刻的变换）；对点云运动畸变去畸变（deskew）；滤波器乱序/延迟测量处理（保存历史状态）；
- 在多机器人/OpenHarmony M-DDS 场景中同样需要统一时间基准与有效性检查（P6：*standardised timestamps, coordinate frames and validity checks*）。
- 见 [时间同步、坐标系一致性与数据有效性](../../02_基础知识/05_状态估计/06_时间同步_坐标系一致性与数据有效性.md)。
**English short answer**
If timestamps are wrong, fused poses jitter or lag during turns and scans misalign with the map. I use the sensor's own acquisition time, synchronise clocks, look up transforms at the message time with interpolation, and handle delayed measurements by replaying from a stored state. I also add validity checks on stale or out-of-order data.
**追问方向**：TF 外推失败的报错；点云去畸变。

### Q27. 坐标系或运动方向不一致会引起什么？（Frame and direction inconsistencies）★★★
**答案要点**
- 轮速符号与坐标轴方向相反 → 倒车时里程计向前走；左右轮反装；IMU 轴与 base_link 轴差一个旋转；LiDAR 安装偏移未补偿；
- 现象：融合后位姿向相反方向漂、定位与激光对不上、旋转时中心偏离；
- 排查：① 开环小幅前进/原地转，看里程计与实际方向；② 静态检查 `tf_echo`；③ 逐传感器单独画轨迹，再叠加；④ 倒车/原地转专项测试。
- 对应 CV：*pose jumps from inconsistent timing, frames and motion direction*。
**English short answer**
If a wheel sign or an IMU axis is flipped, the fused pose drifts the wrong way, especially when reversing or turning in place. I debug by driving short, simple motions, checking each sensor alone, and verifying the static transforms. Tests for reversing and in-place rotation catch most direction bugs.
**追问方向**：如何系统化验证；哪些传感器最容易错。

### Q28. 轮速打滑/悬空如何检测？（Slip detection）★★
**答案要点**
- 轮速与 IMU 加速度/角速度不一致（残差大）；轮速与 LiDAR 位移不一致；电机电流异常；
- 融合时自适应增大轮速测量噪声 $R$（鲁棒核）；
- 对平衡/轮腿机器人：离地/打滑检测影响状态估计（P7："ground-contact transitions"）。
**English short answer**
I compare wheel odometry with the gyro and the LiDAR motion, and when they disagree I inflate the wheel noise so the filter trusts it less. For legged or wheel-legged robots, I also use motor torque to detect when the wheel has lost contact.
**追问方向**：残差门限；落地冲击。

### Q29. robot_localization 的作用？（robot_localization package）★
**答案要点**
- ROS 的 EKF/UKF 融合节点，支持多传感器（odom、imu、pose）；按配置矩阵选择使用哪些维度；输出 `odom` 或 `map` 系下的 `nav_msgs/Odometry` 并发布 TF；
- 常见配置问题：IMU 的 frame、协方差为 0 导致权重异常、2D 模式忽略 z/roll/pitch、`differential` 选项。
**English short answer**
robot_localization is a ROS package that runs an EKF or UKF over multiple odometry, IMU and pose sources, and lets me choose which dimensions each source contributes. The common pitfalls are wrong frames, zero covariances, and mixing absolute and differential inputs.
**追问方向**：你用没用；替代方案。

### Q30. 为什么要 `map→odom→base_link` 这样的 TF 结构？（Why map-odom-base_link?）★★★
**答案要点**
- `odom→base_link`：里程计/EKF，**连续**但会漂移，适合局部控制；
- `map→odom`：定位模块（AMCL）发布的**修正量**，可能跳变，但全局一致；
- `map→base_link = map→odom ∘ odom→base_link`。局部规划/控制使用 odom 避免跳变，全局规划使用 map。
- 详见 [ROS 软件八股](./05_ROS与机器人软件八股.md)、[URDF/TF/坐标系](../../02_基础知识/09_机器人软件/07_URDF_TF_坐标系实践.md)。
**English short answer**
The odom-to-base transform is smooth but drifts, so the controller can rely on it, while the map-to-odom transform is published by the localiser and holds the correction, which may jump. Composing them gives the global pose. That separation keeps the local control continuous even when localisation corrects itself.
**追问方向**：REP-105；位姿跳变的排查。

---

## 四、AMCL 与定位

### Q31. AMCL 的原理？（AMCL）★★★
**答案要点**
- 蒙特卡洛定位（MCL）：粒子滤波估计机器人在**已知地图**中的位姿 $[x,y,\theta]$；**自适应**（Adaptive）：用 KLD-采样调整粒子数、并注入随机粒子；
- 流程：
  1. 初始化：在初始位姿附近高斯采样，或全局均匀采样；
  2. 预测：用里程计运动模型（odom 模型的 $\alpha_1..\alpha_4$ 噪声）给每个粒子加噪声推进；
  3. 更新：用 2D LiDAR 对每个粒子计算似然（likelihood field 或 beam 模型），得权重；
  4. 重采样；估计位姿 = 加权均值/最大簇；
  5. 发布 `map→odom`。
- 似然场模型：每个束端点到最近障碍物距离 $d$，$p=z_{hit}\mathcal N(d;0,\sigma_{hit}^2)+z_{rand}/z_{max}$，快（可预计算距离场）。
- 项目：P5 的"map-based localisation"。
**English short answer**
AMCL is a particle filter that estimates the robot's pose on a known map. It propagates particles using the odometry motion model, weights them by how well the LiDAR scan matches the map, and resamples. It adapts the number of particles with KLD sampling, and publishes the map-to-odom correction.
**追问方向**：似然场 vs 光束模型；初始位姿；粒子数。

### Q32. AMCL 里运动模型与传感器模型怎么建？（Motion and sensor models）★★
**答案要点**
- 运动模型（里程计）：把两帧里程计差分解为 $\delta_{rot1},\delta_{trans},\delta_{rot2}$，各自加与幅度相关的噪声（$\alpha_1\ldots\alpha_4$）；
- 传感器：beam model（4 部分混合：hit、short、max、rand）或 likelihood field（只用束端点到最近障碍物距离，快、平滑）；
- 参数：`laser_max_range`、`z_hit`、`sigma_hit`、`update_min_d/a`（移动多少才更新）。
**English short answer**
The motion model splits the odometry increment into a rotation, a translation and another rotation, and adds noise that scales with each. The sensor model is usually a likelihood field: for each beam endpoint look up the distance to the nearest obstacle in the map and score it with a Gaussian plus a uniform floor.
**追问方向**：噪声参数调节；更新门限的意义。

### Q33. AMCL 的 KLD 自适应采样是什么？（KLD-sampling）★★
**答案要点**
- 粒子数根据后验分布的"占用箱数" $k$ 动态选：分布集中 → 少粒子；分散（刚启动/迷路）→ 多粒子，约束 KL 散度误差 $<\epsilon$（置信 $1-\delta$）；
- `min_particles`, `max_particles`, `kld_err`, `kld_z`。
**English short answer**
KLD sampling chooses how many particles to use based on how spread out the distribution is: few when the robot is well localised, many when it is uncertain. That saves compute without losing accuracy.
**追问方向**：参数设置。

### Q34. 什么情况下 AMCL 会失效或跳变？（AMCL failure modes）★★★
**答案要点**
- 走廊/长直墙、空旷场地（几何退化，沿走廊方向不可观）；
- 地图与现实不一致（动态障碍、布局变化）；
- 里程计方向/尺度错 → 粒子被拖离；
- 激光安装/外参错、时间戳偏差 → 似然错；
- 粒子数太少、过早收敛（过度自信）；
- 初始位姿错且没有恢复机制（绑架问题）。
**English short answer**
AMCL struggles in long corridors or open spaces where the scan does not constrain the position, when the map is outdated, when the odometry is wrong in scale or direction, or when sensor extrinsics are off. A wrong initial pose without recovery can also make it converge to the wrong place.
**追问方向**：如何诊断（粒子云形状、协方差、残差）。

### Q35. 绑架问题与全局定位怎么处理？（Kidnapped robot）★★
**答案要点**
- 绑架：机器人被挪到别处，滤波器过度自信无法恢复；
- 对策：注入随机粒子（根据 $w_{slow}$ 与 $w_{fast}$ 的比值 $\max(0,1-w_{fast}/w_{slow})$ 触发），或全局重定位（均匀撒粒子）；
- 人工给 initial pose；结合回环/场景识别。
**English short answer**
If the robot is picked up and moved, a confident particle filter cannot recover. AMCL tracks short-term and long-term average weights and injects random particles when the short-term average falls, so it can re-localise. I can also reinitialise the pose manually.
**追问方向**：$\alpha_{slow},\alpha_{fast}$ 参数。

### Q36. AMCL 的输出为什么会跳变？如何让下游平滑？（Pose jumps from AMCL）★★★
**答案要点**
- 重采样/多峰切换/地图匹配改变 → `map→odom` 突变；
- 下游避免跳变：控制/局部规划使用 `odom` 系；全局路径用 `map` 但对路径重新规划；跟踪控制器使用滤波后的状态；对 `map→odom` 做平滑限速（可选）；
- 融合时对 AMCL 观测做门限（马氏距离）拒绝野值。
- CV：*output a continuous vehicle state*。
**English short answer**
AMCL corrections show up as jumps in the map-to-odom transform. The controller works in the odom frame, so it stays continuous. I also gate the localisation measurement with a Mahalanobis threshold to reject outliers before they enter the filter.
**追问方向**：马氏距离门限设定；平滑引入的延迟。

### Q37. 扫描匹配（ICP / NDT / CSM）是什么？（Scan matching）★★
**答案要点**
- 给定两帧点云，求相对位姿使它们对齐；
- ICP：迭代 {最近点配对 → 求刚体变换（SVD）→ 更新}，需要好初值，易陷局部最优；点到线 ICP 对 2D 更稳；
- NDT：把参考点云分格，每格拟合高斯，目标点对分布求似然，平滑、对初值更宽容；
- 实时相关扫描匹配（CSM/Correlative）：在搜索窗口内枚举位姿计算得分，Cartographer 用它做前端；
- 用途：激光里程计、回环验证、定位修正。
**English short answer**
Scan matching estimates the relative transform between two point clouds. ICP alternates between nearest-neighbour association and solving for the rigid transform, and needs a good initial guess. NDT models the reference as a set of local Gaussians and optimises a smooth score. Correlative scan matching searches a window of poses exhaustively, which is robust but costly.
**追问方向**：ICP 局部最优；退化场景。

---

## 五、SLAM 与回环、图优化

### Q38. SLAM 问题的定义？（What is SLAM?）★★★
**答案要点**
- 同时定位与建图：在未知环境中，同时估计机器人轨迹 $x_{0:t}$ 与地图 $m$：$p(x_{0:t},m|z_{1:t},u_{1:t})$；
- 两大范式：滤波（EKF-SLAM、FastSLAM/粒子）与图优化（位姿图/因子图）；
- 难点：数据关联、闭环、漂移累计、动态环境、计算规模。
**English short answer**
SLAM estimates the robot trajectory and the map at the same time, using motion and sensor measurements, in an unknown environment. The two main families are filtering approaches and graph optimisation. The hard parts are data association, loop closure, drift, and scaling.
**追问方向**：为什么定位与建图互相依赖；与纯定位的区别。

### Q39. gmapping 的原理与局限？（gmapping）★★★
**答案要点**
- 基于 **Rao-Blackwellized 粒子滤波**：粒子表示轨迹，每个粒子带自己的栅格地图；改进提议分布（用扫描匹配结果）和选择性重采样；
- 优点：2D、轻量、参数少、经典；
- 局限：粒子数×地图内存大；无显式回环检测，大场景漂移；依赖里程计；不适合大范围；
- 传感器：2D LiDAR + 里程计。
**English short answer**
gmapping uses a Rao-Blackwellised particle filter where each particle carries a trajectory hypothesis and its own grid map. It is lightweight and easy for small indoor spaces, but it has no explicit loop closure, uses lots of memory with many particles, and needs decent odometry.
**追问方向**：为什么粒子数不能太大；选择性重采样。

### Q40. Cartographer 的原理？（Cartographer）★★★
**答案要点**
- Google 的 2D/3D 图优化 SLAM；
- 前端：局部子图（submap）内用 IMU 预测 + 实时相关扫描匹配/Ceres 扫描匹配，把每帧插入子图；
- 后端：子图间做**分支定界**的回环搜索，所有子图和节点构成位姿图，周期性稀疏位姿图优化（SPA，Ceres）；
- 优点：回环强、大场景一致、支持 IMU 和多传感器；缺点：配置复杂、纯建图/纯定位模式、占用资源。
**English short answer**
Cartographer builds local submaps with scan matching and IMU prediction, then searches for loop closures between submaps using branch-and-bound and optimises a sparse pose graph. It handles large environments better than gmapping, but is heavier to configure and run.
**追问方向**：子图大小；分支定界；纯定位模式。

### Q41. LIO-SAM 的原理？（LIO-SAM）★★
**答案要点**
- 激光-惯性紧耦合里程计，基于 **GTSAM 因子图**（iSAM2 增量优化）；
- 因子：IMU 预积分因子、激光里程计因子（特征点：边/面）、GPS 因子、回环因子；
- 点云 deskew（用 IMU 补偿运动畸变）；
- 适用：3D LiDAR + 9 轴 IMU，室外大场景；
- 注意 IMU 外参、时间同步、IMU 频率（高于 100 Hz）。
**English short answer**
LIO-SAM is a tightly coupled LiDAR-inertial system built on a factor graph. It adds IMU preintegration, LiDAR odometry and loop-closure factors and solves them incrementally, and it uses the IMU to de-skew the point clouds. It needs a 3D LiDAR, a good IMU and accurate time sync.
**追问方向**：预积分；外参；GPS 因子。

### Q42. ORB-SLAM 的原理？（ORB-SLAM）★★
**答案要点**
- 特征点式视觉 SLAM（单目/双目/RGB-D），ORB 特征 + 词袋回环；
- 三线程：Tracking（位姿估计，局部地图）、Local Mapping（局部 BA、关键帧管理）、Loop Closing（DBoW2 检测、Sim(3) 位姿图优化、全局 BA）；
- 单目有尺度不确定；ORB-SLAM3 加入 IMU 和多地图；
- 弱纹理、快速运动、光照变化时丢失。
**English short answer**
ORB-SLAM is a feature-based visual SLAM with three threads: tracking, local mapping with bundle adjustment, and loop closing using a bag-of-words place recognition. Monocular versions have scale ambiguity, and the system can lose tracking in low texture or fast motion.
**追问方向**：BA；关键帧；重定位。

### Q43. 常见 SLAM 方案对比表？（Comparison）★★★
| 方案 | 传感器 | 维度 | 后端 | 回环 | 特点/适用 |
|---|---|---|---|---|---|
| gmapping | 2D LiDAR + 里程计 | 2D | RBPF 粒子滤波 | 无显式 | 小场景室内，简单 |
| Cartographer | 2D/3D LiDAR + IMU | 2D/3D | 子图 + 稀疏位姿图 | 有（分支定界） | 大场景，配置复杂 |
| LIO-SAM | 3D LiDAR + IMU（+GPS） | 3D | GTSAM 因子图 | 有 | 紧耦合，室外 |
| ORB-SLAM | 相机（+IMU） | 3D 稀疏 | BA + 位姿图 | 词袋 | 视觉，纹理依赖 |

**English short answer**
gmapping is a simple 2D particle-filter SLAM, Cartographer is a submap and pose-graph system with strong loop closure, LIO-SAM is a tightly coupled 3D LiDAR-inertial factor graph, and ORB-SLAM is a feature-based visual system. I pick based on sensors, environment size, and compute. For my car-like robot with a 2D LiDAR on a known map, localisation with AMCL was the relevant tool rather than full SLAM.
**追问方向**：你更熟悉哪个；为什么不用 SLAM 而用 AMCL（已有地图）。（诚实：CV 是"map-based localisation"。）

### Q44. 回环检测是什么？为什么重要？（Loop closure）★★★
**答案要点**
- 识别"回到曾经访问过的地方"，增加一个约束把累积漂移拉回；
- 方法：2D LiDAR：子图匹配（分支定界）、Scan Context（3D）；视觉：词袋（BoW）、NetVLAD 等；
- 验证：几何一致性（ICP/RANSAC）、时间一致性（连续多次）；
- 误回环（false positive）会破坏整张地图，宁可漏检不可错检：鲁棒核（Huber/DCS）、Switchable constraints。
**English short answer**
A loop closure recognises that the robot has returned to a previously visited place and adds a constraint between the two poses, which pulls the accumulated drift back. False positives are far more damaging than misses, so I verify geometrically and use robust kernels in the back end.
**追问方向**：回环验证；退化场景下误回环。

### Q45. 位姿图优化是什么？（Pose graph optimisation）★★★
**答案要点**
- 节点 = 位姿 $x_i$；边 = 相对位姿测量 $z_{ij}$（里程计、扫描匹配、回环）及信息矩阵 $\Omega_{ij}$；
- 目标：
$$\min_X\sum_{(i,j)}e_{ij}^{\top}\Omega_{ij}e_{ij},\quad e_{ij}=\log(z_{ij}^{-1}(x_i^{-1}x_j))^{\vee}$$
- 高斯-牛顿 / LM 迭代：$H\delta x=-b$，$H=\sum J^{\top}\Omega J$ 稀疏，用稀疏 Cholesky；需固定一个节点（规范自由度）；
- 库：g2o、GTSAM、Ceres。
**English short answer**
A pose graph has poses as nodes and relative-pose measurements, from odometry or loop closures, as edges weighted by their information matrices. I minimise the sum of squared weighted residuals using Gauss-Newton or Levenberg-Marquardt, exploiting the sparsity. One node must be fixed to remove the gauge freedom.
**追问方向**：为什么 H 稀疏；信息矩阵的作用；鲁棒核。

### Q46. 因子图与 BA 的关系？（Factor graph and BA）★★
**答案要点**
- 因子图是通用表示：变量节点+因子；位姿图是仅含位姿的特例；BA（Bundle Adjustment）同时优化相机位姿和 3D 路标点的**重投影误差**，Schur 补消去路标得到相机约化系统；
- iSAM2：增量因子图，贝叶斯树，只更新受影响部分。
**English short answer**
A factor graph is a general way to write estimation problems as variables and constraints. A pose graph is the special case with only poses, and bundle adjustment adds landmark variables with reprojection factors. iSAM2 updates these incrementally.
**追问方向**：Schur 补；marginalisation。

### Q47. 栅格地图与占据概率怎么更新？（Occupancy grid mapping）★★
**答案要点**
- 每个栅格存占据概率，用对数几率更新：$l_{t}=l_{t-1}+\log\frac{p(m|z)}{1-p(m|z)}-l_0$；命中格 $+l_{occ}$，光束穿过格 $+l_{free}$（负）；上下限截断（clamp）避免锁死；
- 光线投射（Bresenham）；分辨率与内存；
- 与代价地图关系：见 [04 导航八股](./04_路径规划与导航八股.md)。
**English short answer**
An occupancy grid stores for every cell the log-odds of being occupied. A beam hit raises the value of its endpoint and the beam passing through lowers the cells along the ray, with clamping to keep values bounded. That makes updating cheap and handles noisy measurements.
**追问方向**：对数几率为什么方便；动态物体。

### Q48. 地图类型有哪些？（Map types）★
**答案要点**
- 栅格地图（2D 导航）、特征/路标地图、拓扑地图、点云/体素/八叉树/ESDF（3D 避障）、语义地图；
- 选择取决于任务：导航需要自由/占据，操作需要几何与语义。
**English short answer**
Grid maps suit 2D navigation, landmark maps suit feature SLAM, topological maps suit large-scale routing, and 3D voxel or ESDF maps suit obstacle avoidance in three dimensions.
**追问方向**：ESDF 用途。

### Q49. SLAM 精度如何评价？（ATE, RPE）★★
**答案要点**
- ATE（绝对轨迹误差）：对齐后位姿 RMSE，衡量全局一致性；
- RPE（相对位姿误差）：固定间隔的相对误差，衡量漂移率；
- 工具：evo；公开数据集 TUM、KITTI。
- 没有真值的实车：回到起点的闭合误差、重复性。
**English short answer**
Absolute trajectory error measures global consistency after aligning with the ground truth, and relative pose error measures drift over a fixed interval. On a real vehicle without ground truth I look at the loop-closure error and repeatability.
**追问方向**：对齐方式（SE(3)/Sim(3)）。

---

## 六、不可观性、一致性与工程问题

### Q50. 什么是不可观性？机器人定位里有哪些例子？（Unobservability in robotics）★★★
**答案要点**
- 某些状态方向无法由测量确定：
  1. 纯里程计：全局位置与航向不可观（需绝对观测）；
  2. 无磁力计的 AHRS：yaw 不可观；
  3. 单目视觉：尺度不可观；
  4. 走廊中的 LiDAR 匹配：沿走廊方向不可观（几何退化）；
  5. VIO：全局位置与 yaw 四维不可观（4-DoF gauge）；
- 后果：协方差沿该方向增长；若估计器错误地认为它可观（EKF 不一致），协方差会伪收缩 → 过度自信。
**English short answer**
A state is unobservable if no measurement constrains it. Examples are global position and heading with odometry only, yaw without a magnetometer, scale with a monocular camera, and the along-corridor direction for LiDAR matching in a long hallway. In these directions uncertainty keeps growing, and a naive EKF may wrongly shrink it.
**追问方向**：如何检测退化（Hessian 特征值）；退化时的应对。

### Q51. 如何检测并应对几何退化？（Degeneracy detection）★★
**答案要点**
- 检查扫描匹配 Hessian $J^{\top}J$ 的最小特征值/条件数；
- 应对：降低该方向观测权重、融合轮速/IMU 约束该方向、沿退化方向只用里程计；
- 地图中增加特征或用多传感器。
**English short answer**
I check the eigenvalues of the matching Hessian. If the smallest one is tiny, the geometry is degenerate in that direction, and I either rely on odometry and inertial data for that direction or reduce the matching weight.
**追问方向**：走廊场景。

### Q52. 数据有效性检查（validity check）具体怎么做？（Data validity checks）★★★
**答案要点**
- 时间：时间戳单调递增、延迟在阈值内、超时视为失效；
- 数值：NaN/Inf、范围检查、突变检查（速率限制）；
- 一致性：多源残差（马氏距离）、坐标系 ID 是否正确；
- 状态：给每个状态量附 `valid` 标志与时间/协方差，下游按标志降级或拒绝使用；
- P6 对应：*standardised timestamps, coordinate frames and validity checks across multi-source perception and actuator feedback*。
**English short answer**
I check timestamps for order and freshness, values for NaN and range, and consistency between sources with a Mahalanobis gate. Each state carries a validity flag so downstream controllers can degrade safely instead of using stale data.
**追问方向**：超时策略；降级策略。

### Q53. 马氏距离门限（innovation gating）是什么？（Mahalanobis gating）★★
**答案要点**
- $d^2=\nu^{\top}S^{-1}\nu$ 服从 $\chi^2_m$；取 99% 分位（m=3 约 11.34）；超过则拒绝该观测；
- 好处：拒绝 AMCL 跳变、LiDAR 误匹配、轮速打滑野值；
- 风险：滤波器过度自信时会拒绝真实观测 → 发散；需设置回退（连续被拒 N 次则重置/增大协方差）。
**English short answer**
I compute the squared Mahalanobis distance of the innovation, compare it with a chi-square threshold, and discard measurements that are too unlikely. If the filter is overconfident it may reject good data, so I add a recovery rule after several consecutive rejections.
**追问方向**：阈值选择；自由度。

### Q54. EKF 为什么容易不一致？（Why EKF is inconsistent）★
**答案要点**
- 线性化点依赖当前估计，使得可观性性质与真实系统不一致，伪造了本不可观的方向信息（例如 EKF-SLAM 中 yaw 变得"可观"）；
- 解决：FEJ、观测一致性约束（OC-EKF）、不变 EKF（InEKF）、ESKF。
**English short answer**
Because the Jacobians are evaluated at the current estimate, the EKF can accidentally gain information in directions that are really unobservable, making it overconfident. Fixes include first-estimate Jacobians and invariant or error-state formulations.
**追问方向**：yaw 伪可观。

### Q55. 状态估计常见的"延迟测量"和"乱序"如何处理？（Delayed and out-of-order measurements）★★
**答案要点**
- 方法：保存状态历史；收到延迟测量时回滚到该时间点，应用更新，重放之后的输入；或用测量时刻的预测状态（外推）近似；
- 简化：丢弃过旧测量、限制延迟窗口。
**English short answer**
I keep a short history of states and inputs. When a delayed measurement arrives, I roll back to its timestamp, apply the update and replay forward. If the delay is small, I can approximate by extrapolating the prediction.
**追问方向**：计算量；窗口长度。

### Q56. 速度怎么从编码器估计？（Velocity estimation from encoders）★★
**答案要点**
- M 法（定时间隔数脉冲）：高速好；T 法（测脉冲间隔时间）：低速好；M/T 法结合；
- 一阶差分 + 低通；或 Kalman/观测器（位置+速度）；
- 低速量化噪声大 → 速度环的微分/ P 项放大噪声（见 [02 控制八股](./02_控制八股.md) 的现象表）。
**English short answer**
At high speed I count pulses in a fixed window, and at low speed I measure the time between pulses, and combining both is the M/T method. I usually add a low-pass filter or an observer, because quantisation noise at low speed is amplified by the derivative action.
**追问方向**：分辨率与采样频率的关系。

### Q57. 如何用残差/创新做滤波器诊断？（Innovation-based diagnostics）★★
**答案要点**
- NIS 平均应接近量测维数 $m$；偏高 → 噪声低估或模型错；偏低 → 噪声高估；
- 创新序列自相关检验白噪声；
- 实机：把 $\nu$ 与 $3\sqrt{S_{ii}}$ 包络画在一起。
**English short answer**
I plot the innovations with their three-sigma bounds. If they exceed the bounds too often, I'm underestimating noise or the model is wrong, and if they stay far inside, I'm overestimating the noise.
**追问方向**：自适应滤波。

### Q58. 状态估计与控制如何配合？（Estimation in the control loop）★★★
**答案要点**
- 分离原理在非线性/饱和时不严格成立；实际要保证估计带宽 > 控制带宽，估计延迟小；
- 平衡类：状态不一致（P7："traced balance anomalies to state inconsistency"）→ 例如轮速、关节角、IMU 时间不同步、腿长与轮速计算的速度含义不一致；
- 做法：同一时刻快照所有传感器；统一坐标系/符号；估计异常时降级（降低增益/收腿/停机）。
- 【待你补全：P7 中"状态不一致"的具体案例（哪两个量不一致、如何发现、如何修复）】
**English short answer**
The estimate must be fast, low-latency and consistent, otherwise the controller reacts to something that is not true. In my wheel-legged work, balance problems were often traced to state inconsistency between sensors, so I take a synchronised snapshot of the sensors and check the validity before the controller uses it.
**追问方向**：你遇到的具体 bug；如何复现与定位。

---

## 七、本章要点速记

1. KF：预测（$FPF^{\top}+Q$）+ 更新（$K=PH^{\top}S^{-1}$）；$Q$ 小滞后，$Q$ 大噪声大。
2. EKF 用雅可比线性化；UKF 用 sigma 点；粒子滤波处理多峰但怕高维。
3. 互补滤波=陀螺高通+加计低通；Mahony 用叉乘误差 + PI 修陀螺。
4. yaw 无磁力计时不可观；加计在加速时不可靠。
5. 里程计航向误差最致命；融合陀螺与地图定位。
6. AMCL=已知地图的粒子滤波；运动模型+似然场+KLD；跳变放进 `map→odom`。
7. SLAM：gmapping（RBPF）、Cartographer（子图+位姿图）、LIO-SAM（因子图紧耦合）、ORB-SLAM（视觉 BA）。
8. 回环需验证，宁漏勿错；位姿图是稀疏最小二乘。
9. 不可观方向协方差应增长；EKF 易过度自信；用 NIS/NEES 检查。
10. 时间戳、坐标系、符号是融合 bug 的头号来源。

## 八、自测题

1. 一维 KF：预测方差 9，测量方差 1，预测 5，测量 6，求更新后的估计与方差。（答：$K=0.9$，$\hat x=5.9$，$P=0.9$）
2. 互补滤波 $\tau=0.5$ s、$T=0.005$ s，$\alpha$ 是多少？（答：$0.5/0.505\approx0.990$）
3. 为什么没有磁力计 yaw 会漂？
4. 简述 AMCL 一次迭代的步骤。
5. gmapping 与 Cartographer 的后端有什么区别？
6. 位姿图优化为什么要固定一个节点？
7. EKF 过度自信可能带来什么后果？
8. 哪些情形会让 2D LiDAR 定位退化？

**答案提示**：1 见 Q1；2 见 Q13；3 见 Q18；4 见 Q31；5 见 Q39–Q40；6 见 Q45；7 见 Q54、Q53；8 见 Q34、Q50。

## 九、相关链接

- 上一篇：[02 控制八股](./02_控制八股.md)；下一篇：[04 路径规划与导航八股](./04_路径规划与导航八股.md)
- 项目：[P1](../../01_项目课程/P1_AIRS球形机器人平衡控制/README.md)、[P2](../../01_项目课程/P2_未来动力双臂遥操作与差速底盘/README.md)、[P3](../../01_项目课程/P3_RoboMaster英雄机器人嵌入式控制/README.md)、[P5](../../01_项目课程/P5_车式移动机器人自主导航/README.md)、[P6](../../01_项目课程/P6_深开鸿M-Robots平台软件/README.md)
- 基础：[卡尔曼滤波与 EKF](../../02_基础知识/05_状态估计/03_卡尔曼滤波与EKF.md)、[粒子滤波与 AMCL](../../02_基础知识/05_状态估计/05_粒子滤波与AMCL.md)
