# SLAM 与建图八股

> **定位**：[状态估计与SLAM八股](../03_机器人学/03_状态估计与SLAM八股.md) 讲了滤波器和常见 SLAM 方案的概览；本文深入 SLAM 本身：问题的概率形式、前端后端、位姿图与因子图、视觉 SLAM（ORB-SLAM 类）、视觉惯性（预积分、滑窗、MSCKF、可观性）、激光 SLAM（Cartographer、LIO、Scan Context）、评估与工程可靠性。对应基础知识：[SLAM 问题与位姿图优化](../../02_基础知识/07_SLAM与建图/01_SLAM问题_前端后端_位姿图优化.md)、[视觉里程计](../../02_基础知识/07_SLAM与建图/02_视觉里程计_特征_对极几何_PnP_BA.md)、[激光 SLAM](../../02_基础知识/07_SLAM与建图/03_激光SLAM_LOAM_LIO-SAM_Cartographer.md)、[视觉惯性里程计](../../02_基础知识/07_SLAM与建图/04_视觉惯性里程计VIO_预积分_滑动窗口.md)、[回环检测](../../02_基础知识/07_SLAM与建图/05_回环检测与重定位_词袋_ScanContext.md)。

**本章学完你能……**
- 用概率语言定义 SLAM，说清滤波与优化两条路线的取舍；
- 推导位姿图的残差与稀疏结构，解释边缘化与规范自由度；
- 描述 ORB-SLAM 三线程、VIO 预积分与滑窗、LIO 紧耦合的核心步骤；
- 分析可观性、退化与失败模式，并给出监控与恢复策略；
- 评估 SLAM 系统，并用英文讲清自己的 SLAM / 定位经验（对应 P5 的定位与建图部分）。

**前置知识**：[状态估计与SLAM八股](../03_机器人学/03_状态估计与SLAM八股.md)；[相机几何、标定与立体视觉八股](../05_计算机视觉与3D感知/02_相机几何_标定与立体视觉八股.md)；[点云、3D 感知与传感器融合八股](../05_计算机视觉与3D感知/03_点云_3D感知与传感器融合八股.md)。

> 关于项目：CV 中 P5 涉及车式移动机器人的 AMCL 定位与导航；本文的 SLAM 内容是通用知识，用于补充面试中"从定位到建图"的追问，不代表项目里实现了这些系统。涉及具体论文的数值标"未核对"。

---

## 一、SLAM 的问题定义与优化框架

### Q1 SLAM 的概率形式是什么？在线与全轨迹 SLAM 的区别？（Probabilistic formulation）
**优先级：★★★** · 对应基础知识：[SLAM问题_前端后端_位姿图优化](../../02_基础知识/07_SLAM与建图/01_SLAM问题_前端后端_位姿图优化.md)

**答案要点**
- 变量：机器人轨迹 $\mathbf{x}_{0:t}$、地图 $m$（路标或栅格）；输入：控制/里程计 $\mathbf{u}_{1:t}$、观测 $\mathbf{z}_{1:t}$。
- **全 SLAM**：估计整条轨迹与地图的联合后验 $p(\mathbf{x}_{0:t},m|\mathbf{z}_{1:t},\mathbf{u}_{1:t})$（优化方法，保留所有历史，可重线性化）；**在线 SLAM**：只估计当前位姿与地图 $p(\mathbf{x}_t,m|\cdot)$，通过对历史位姿**边缘化**得到（滤波方法，如 EKF-SLAM）。
- 马尔可夫假设下后验分解为：先验 × 运动模型 × 观测模型的乘积；高斯噪声下取负对数即非线性最小二乘，MAP 估计。
- 关键难点：数据关联（观测对应哪个路标）、非线性、计算量随地图增长、一致性。
- 一句话：SLAM 是在线地同时估计"我在哪"和"世界什么样"，二者互为条件。

**English short answer**
SLAM estimates the robot trajectory and the map from controls and observations. Full SLAM computes the posterior over the entire trajectory and map, which optimization methods solve by maximum a posteriori inference as nonlinear least squares, while online SLAM keeps only the current pose by marginalizing past poses, as in filtering. Data association and nonlinearity make it hard, and the two estimates depend on each other.

（译：SLAM 由控制和观测估计机器人轨迹与地图；全 SLAM 计算整条轨迹与地图的后验，优化方法将其作为非线性最小二乘做 MAP 推断，在线 SLAM 通过边缘化过去位姿只保留当前位姿（如滤波）；数据关联与非线性使其困难，且两个估计量互相依赖。）

**追问方向**：为什么取负对数得到最小二乘；边缘化历史位姿会带来什么。

### Q2 SLAM 前端与后端分别做什么？（Front-end vs back-end）
**优先级：★★★** · 对应基础知识：[SLAM问题_前端后端_位姿图优化](../../02_基础知识/07_SLAM与建图/01_SLAM问题_前端后端_位姿图优化.md)

**答案要点**
- **前端（里程计/数据关联）**：从传感器数据提取特征、匹配、估计帧间相对运动（VO/LO），提供初值；检测回环候选；处理对应关系。对实时性要求高。
- **后端（优化）**：融合所有约束（里程计边、回环边、先验），估计全局一致的轨迹与地图（位姿图/BA/因子图）；代价高，可降频（关键帧）、可异步。
- 接口：前端输出相对位姿/观测及不确定性（信息矩阵），后端输出优化后的位姿用于更新前端的参考。
- 好处：模块化，可替换前端（视觉/激光）而复用后端（g2o/GTSAM/Ceres）；错误回环对后端破坏最大，所以前端需严格验证。
- 现代系统：边界模糊（紧耦合 LIO/VIO 的滑窗优化同时承担前后端职责）。

**English short answer**
The front end extracts features, matches data and estimates frame-to-frame motion and loop candidates in real time, providing initial values and uncertainties. The back end fuses all constraints, odometry, loop closures and priors, into a globally consistent trajectory and map, usually asynchronously and at a lower rate. This modularity lets different front ends share the same optimizer, and false loop closures are the biggest risk, so the front end must verify them.

（译：前端实时提取特征、匹配数据、估计帧间运动并给出回环候选，提供初值与不确定性；后端把里程计、回环和先验等所有约束融合为全局一致的轨迹与地图，通常异步且频率较低；这种模块化使不同前端可复用同一优化器，错误回环风险最大，所以前端必须验证。）

**追问方向**：信息矩阵由前端怎么提供；紧耦合系统如何模糊前后端界限。

### Q3 EKF-SLAM 的复杂度和一致性问题？（EKF-SLAM limitations）
**优先级：★★★** · 对应基础知识：[卡尔曼滤波与EKF](../../02_基础知识/06_状态估计/03_卡尔曼滤波与EKF.md)

**答案要点**
- 状态向量 = 机器人位姿 + 全部路标 $[x_r,\mathbf{m}_1,\dots,\mathbf{m}_N]$，协方差为 $(3+2N)^2$ 的稠密矩阵；每次观测更新 $O(N^2)$，路标数大时不可扩展（数百个以上就吃力）。
- **不一致**：线性化点随估计变化，使可观性性质被破坏（虚假地获得航向等不可观量的信息），协方差过于乐观 → 滤波器发散；改进：FEJ（先估计雅可比）、不变 EKF、ESKF。
- **数据关联**：错误关联会导致灾难性失败（发散），需马氏距离门限 + JCBB（联合兼容分支定界）。
- 优点：概念简单、在线、给出完整协方差；缺点：只保留当前估计，无法重线性化历史，误差无法纠正。
- 因此现代系统倾向优化（平滑）+ 稀疏性，EKF 仅在小规模/嵌入式或 MSCKF 类中使用。

**English short answer**
EKF-SLAM keeps the pose and all landmarks in one state with a dense covariance, so each update costs quadratic time in the number of landmarks and it does not scale. Linearization at changing estimates breaks observability, producing inconsistent, overconfident covariances, which first-estimate Jacobians or invariant filters mitigate. A single wrong data association can destroy the filter, so gating and joint compatibility tests are used. Smoothing with sparse optimization has largely replaced it.

（译：EKF-SLAM 把位姿和所有路标放入一个状态、协方差稠密，每次更新代价随路标数二次增长，难以扩展；在变化的估计点处线性化会破坏可观性，产生不一致、过度自信的协方差，可用 FEJ 或不变滤波缓解；一次错误数据关联即可毁掉滤波器，所以用门限与联合兼容性检验；基于稀疏优化的平滑方法已基本取代它。）

**追问方向**：JCBB 的做法；为什么 EKF 无法纠正历史误差。

### Q4 FastSLAM / Rao-Blackwellized 粒子滤波的思想？（FastSLAM and RBPF）
**优先级：★★** · 对应基础知识：[粒子滤波与AMCL](../../02_基础知识/06_状态估计/05_粒子滤波与AMCL.md)

**答案要点**
- 关键观察：**给定机器人轨迹，各路标估计条件独立**。因此将联合后验分解为 $p(\mathbf{x}_{1:t}|\cdot)\prod_i p(\mathbf{m}_i|\mathbf{x}_{1:t},\cdot)$：用**粒子滤波**估计轨迹，每个粒子内为每个路标维护小的 EKF（FastSLAM），更新 $O(M\log N)$。
- 栅格版（gmapping）：每个粒子携带一张完整的占据栅格地图；提议分布融合最新扫描匹配结果（改进提议）并选择性重采样，减少粒子需求。
- 问题：粒子贫化与**粒子耗竭**（长回环后需要很多粒子；地图拷贝成本高，大环境内存爆炸）；对回环后的修正能力有限（不重新线性化历史）。
- 现状：2D 室内中小地图仍可用（gmapping），大规模被基于图优化的方法（Cartographer、slam_toolbox）取代。

**English short answer**
Given the robot trajectory, landmark estimates are conditionally independent, so a Rao-Blackwellized particle filter samples trajectories and keeps a small filter per landmark, or a full occupancy grid per particle in gmapping. With proposals informed by scan matching and selective resampling, it works for small indoor maps. Memory grows with the number of particles and large loops cause particle depletion, so graph-based methods dominate now.

（译：给定机器人轨迹，各路标估计条件独立，因此 Rao-Blackwell 粒子滤波对轨迹采样并为每个路标（或 gmapping 中每个粒子一整张占据栅格）维护小型滤波；借助扫描匹配改进提议分布和选择性重采样，可用于中小型室内地图；内存随粒子数增长，大回环会导致粒子耗竭，所以现在基于图优化的方法占主导。）

**追问方向**：改进提议分布的作用；粒子耗竭的表现。

### Q5 位姿图优化的残差定义与雅可比结构？（Pose graph residuals）
**优先级：★★★** · 对应基础知识：[SLAM问题_前端后端_位姿图优化](../../02_基础知识/07_SLAM与建图/01_SLAM问题_前端后端_位姿图优化.md)

**答案要点**
- 节点：位姿 $\mathbf{T}_i\in SE(3)$；边 $(i,j)$：测量相对位姿 $\tilde{\mathbf{T}}_{ij}$ 及信息矩阵 $\boldsymbol\Omega_{ij}$。
- 残差：$\mathbf{e}_{ij}=\log\!\big(\tilde{\mathbf{T}}_{ij}^{-1}\,\mathbf{T}_i^{-1}\mathbf{T}_j\big)^\vee\in\mathbb{R}^6$（李代数）；代价 $\sum\mathbf{e}_{ij}^\top\boldsymbol\Omega_{ij}\mathbf{e}_{ij}$。
- 雅可比：残差只依赖 $\mathbf{T}_i$ 和 $\mathbf{T}_j$，所以 $\mathbf{J}_{ij}$ 只有对应两块非零：$\partial\mathbf{e}/\partial\delta\boldsymbol\xi_i$、$\partial\mathbf{e}/\partial\delta\boldsymbol\xi_j$，近似用伴随 $-\mathbf{J}_r^{-1}\mathrm{Ad}$ 形式（常用简化：忽略 $\mathbf{J}_r$ 即用 $\mathbf{I}$ 近似，当误差小）。
- 法方程 $\mathbf{H}\delta=-\mathbf{b}$ 的 $\mathbf{H}$ 稀疏：块 $(i,i),(j,j),(i,j)$ 非零，结构等同图的邻接；固定一个节点（锚点）消除 6 维规范自由度。
- 更新：流形上 $\mathbf{T}_i\leftarrow\exp(\delta\boldsymbol\xi_i^\wedge)\mathbf{T}_i$。

**English short answer**
Each edge compares the measured relative pose with the one implied by the current estimates, and the residual is the Lie algebra logarithm of their discrepancy weighted by the information matrix. It depends only on the two connected poses, so the Jacobian has two nonzero blocks and the normal equations are sparse with the graph's structure. Poses are updated on the manifold, and one pose is fixed to remove the gauge freedom.

（译：每条边把测得的相对位姿与当前估计隐含的相对位姿比较，残差是二者差异的李代数对数并用信息矩阵加权；它只依赖相连的两个位姿，因此雅可比只有两个非零块，法方程的稀疏结构与图一致；位姿在流形上更新，并固定一个位姿消除规范自由度。）

**追问方向**：信息矩阵的设置依据；伴随矩阵在雅可比中的作用。

### Q6 边缘化（marginalization）与舒尔补在 SLAM 中的作用？（Marginalization）
**优先级：★★★** · 对应基础知识：[视觉惯性里程计VIO_预积分_滑动窗口](../../02_基础知识/07_SLAM与建图/04_视觉惯性里程计VIO_预积分_滑动窗口.md)

**答案要点**
- 边缘化：从联合高斯中去掉某些变量，同时保留它们对剩余变量的信息：对 $\mathbf{H}=\begin{bmatrix}\mathbf{H}_{mm}&\mathbf{H}_{mr}\\\mathbf{H}_{rm}&\mathbf{H}_{rr}\end{bmatrix}$ 做舒尔补 $\mathbf{H}_{rr}-\mathbf{H}_{rm}\mathbf{H}_{mm}^{-1}\mathbf{H}_{mr}$，得到关于保留变量的**先验信息**。
- 用途：①滑动窗口 VIO/LIO 丢弃旧帧时保留其信息（避免信息丢失）；②BA 里消去路标点；③控制地图规模、保持实时。
- 副作用：边缘化使剩余变量之间产生**稠密**的新约束（fill-in，破坏稀疏性）；基于边缘化时的线性化点，之后无法重线性化 → 线性化误差累积（需 FEJ 一致性处理）。
- 与"直接丢弃"对比：直接丢弃会丢失约束，导致尺度/漂移更大。
- 实现：先把相关因子集中，再对旧状态做舒尔补，并固定线性化点生成先验因子。

**English short answer**
Marginalization removes variables from the optimization while keeping the information they provided, by taking the Schur complement of the Hessian, yielding a prior on the remaining variables. It lets sliding-window VIO and LIO drop old frames without throwing away constraints, and in bundle adjustment it eliminates landmarks. It introduces dense fill-in among remaining variables and fixes the linearization point, so first-estimate Jacobians are needed to preserve consistency.

（译：边缘化通过对 Hessian 取舒尔补，在保留旧变量所提供信息的前提下将其移出优化，得到关于剩余变量的先验；使滑窗 VIO/LIO 丢弃旧帧时不丢约束，并在光束法平差中消去路标；它会在剩余变量间引入稠密的填充并固定线性化点，因此需要 FEJ 以保持一致性。）

**追问方向**：为什么边缘化会使 Hessian 变稠密；FEJ 如何实现。

### Q7 因子图与 iSAM2 的增量求解思想？（Factor graphs and iSAM2）
**优先级：★★** · 对应基础知识：[IMU预积分与因子图估计](../../02_基础知识/06_状态估计/08_IMU预积分与因子图估计.md)

**答案要点**
- 因子图把后验写成因子乘积，变量与因子的二部图；求解 = 变量消元（等价于稀疏 QR/Cholesky 分解），消元顺序（如 COLAMD）影响填充。
- **iSAM2**：用**贝叶斯树**表示消元结果；新增因子时只**局部更新**受影响的子树（而非重新分解整个系统），并通过**增量重线性化**（仅对变化超过阈值的变量重新线性化）和**部分状态更新**保持精度；回环到来时重排序影响较大的部分。
- 优点：实时增量、保留全轨迹、可随时查询边缘协方差（恢复）；GTSAM 实现，LIO-SAM、各类 VIO 后端采用。
- 对比批处理（g2o/Ceres 每次重新求解）：批处理更简单但代价随图增大；增量法在在线场景有明显优势。
- 局限：大回环导致大范围更新；实现复杂。

**English short answer**
A factor graph factors the posterior into variable-factor relations, and solving it is sparse variable elimination whose ordering controls fill-in. iSAM2 represents the result as a Bayes tree and, when new factors arrive, updates only the affected subtree and relinearizes only variables that changed enough, giving real-time incremental smoothing with access to marginals. GTSAM implements it, and large loop closures trigger bigger updates.

（译：因子图把后验分解为变量–因子关系，求解就是稀疏变量消元，消元顺序控制填充；iSAM2 把结果表示为贝叶斯树，新因子到来时只更新受影响的子树并仅对变化足够大的变量重新线性化，实现实时增量平滑并可访问边缘分布；GTSAM 实现了它，大回环会触发更大范围更新。）

**追问方向**：贝叶斯树与团树的关系；增量重线性化的阈值。

### Q8 位姿图中的鲁棒性：错误回环怎么处理？（Robust pose graph optimization）
**优先级：★★★** · 对应基础知识：[回环检测与重定位_词袋_ScanContext](../../02_基础知识/07_SLAM与建图/05_回环检测与重定位_词袋_ScanContext.md)

**答案要点**
- 错误回环（感知混淆）会把地图扭曲，且优化会"相信"它。多层防护：①前端**严格验证**（时间一致性、几何验证、内点数、相对位姿残差阈值）；②后端**鲁棒核**（Huber/Cauchy 作用于回环边）；③**切换变量（Switchable Constraints）**：每个回环边带一个可优化的开关权重，不一致的边被自动关闭；④**动态协方差缩放（DCS）**；⑤**一致性集合最大化**：成对一致性检验（PCM）、RANSAC 式选择一致回环子集；⑥增量一致性检测（新回环与已有图的闭环残差）。
- 判别：把回环加入后，图残差（卡方）显著增大或与里程计链矛盾 → 可疑。
- 工程：保守策略——宁可漏掉回环，也不接受错误回环；对多会话/多机更需要。
- 评估：回环精确率优先于召回率。

**English short answer**
A wrong loop closure distorts the map and the optimizer will trust it, so I defend in layers: strict front-end verification by temporal consistency, geometric checks and inlier counts; robust kernels on loop edges; switchable constraints or dynamic covariance scaling that downweight inconsistent edges; and consistency selection such as pairwise consistency maximization. I prioritize precision over recall, because missing a loop is much cheaper than accepting a false one.

（译：错误回环会扭曲地图且优化器会相信它，所以分层防御：前端用时间一致性、几何检查和内点数严格验证；对回环边使用鲁棒核；用切换约束或动态协方差缩放降低不一致边的权重；用成对一致性最大化等进行一致性选择；精确率优先于召回率，因为漏掉回环的代价远小于接受错误回环。）

**追问方向**：切换约束的数学形式；回环验证的阈值设定。

---

## 二、视觉 SLAM

### Q9 ORB-SLAM 的三线程架构各做什么？（ORB-SLAM threads）
**优先级：★★★** · 对应基础知识：[视觉里程计_特征_对极几何_PnP_BA](../../02_基础知识/07_SLAM与建图/02_视觉里程计_特征_对极几何_PnP_BA.md)

**答案要点**
- **跟踪线程（Tracking）**：每帧提取 ORB 特征 → 初始位姿估计（运动模型 / 参考关键帧 / 重定位）→ 用局部地图投影匹配细化位姿（仅位姿优化的 BA）→ 决定是否插入关键帧。实时、高频。
- **局部建图线程（Local Mapping）**：处理新关键帧：剔除不良地图点、与共视关键帧三角化新点、**局部 BA**（当前关键帧 + 共视关键帧 + 它们看到的地图点）、剔除冗余关键帧。
- **回环线程（Loop Closing）**：用 DBoW2 检测回环候选 → 几何验证（计算 Sim(3)）→ 融合重复地图点 → **位姿图优化**（本质图）→ 全局 BA（独立线程）。
- 好处：重计算（BA、回环）与实时跟踪分离；共享地图用锁保护。
- ORB-SLAM3：增加 IMU（视觉惯性）、鱼眼模型、**Atlas 多地图**系统。

**English short answer**
The tracking thread extracts ORB features each frame, estimates the pose with a motion model or relocalization, refines it against the local map and decides on keyframes in real time. The local mapping thread triangulates new points, culls bad ones and runs local bundle adjustment over covisible keyframes. The loop closing thread detects loops with a bag of words, computes a Sim(3), fuses points and optimizes the essential graph, then launches global BA. Heavy optimization is thus decoupled from real-time tracking.

（译：跟踪线程每帧提取 ORB 特征，用运动模型或重定位估计位姿，对照局部地图精化并决定关键帧；局部建图线程三角化新点、剔除坏点并在共视关键帧上做局部 BA；回环线程用词袋检测回环、计算 Sim(3)、融合地图点并优化本质图，再启动全局 BA；因此重型优化与实时跟踪解耦。）

**追问方向**：三线程间的数据同步；ORB-SLAM3 的 Atlas 是什么。

### Q10 ORB-SLAM 的跟踪流程：运动模型、参考关键帧与重定位？（Tracking modes）
**优先级：★★★** · 对应基础知识：[视觉里程计_特征_对极几何_PnP_BA](../../02_基础知识/07_SLAM与建图/02_视觉里程计_特征_对极几何_PnP_BA.md)

**答案要点**
- 初始位姿的三种来源（按优先级）：①**恒速运动模型**：用上一时刻的速度预测当前位姿，投影上一帧地图点到当前帧做窄范围搜索匹配，足够内点则采用；②**参考关键帧**：与最近参考关键帧做 BoW 匹配 + PnP；③**重定位**：跟踪丢失时，用 BoW 查询候选关键帧 → 匹配 → PnP RANSAC → 位姿优化。
- 获得初始位姿后：把**局部地图**（参考关键帧及其共视邻居的地图点）投影到当前帧，搜索更多匹配，**仅位姿的 BA**（最小化重投影误差，鲁棒核）精化。
- 关键帧插入条件：距上一关键帧足够帧数、跟踪到的参考点比例下降、局部建图空闲等。
- 状态机：初始化 → 正常跟踪 → 丢失（重定位）。
- 失败：快速运动（运动模型失效）→ 退回关键帧匹配；纯旋转、模糊、弱纹理 → 丢失。

**English short answer**
Tracking first gets an initial pose from a constant-velocity motion model by projecting the previous frame's map points, falling back to bag-of-words matching with the reference keyframe plus PnP, and to relocalization against the keyframe database after loss. It then projects the local map into the frame to find more matches and refines the pose with a pose-only bundle adjustment. Keyframes are inserted when tracked-point coverage drops, and fast motion, blur or low texture cause loss.

（译：跟踪首先用恒速运动模型投影上一帧地图点获得初始位姿，失败则退回到与参考关键帧的词袋匹配加 PnP，丢失后对关键帧数据库重定位；随后将局部地图投影到当前帧找更多匹配，用仅位姿的 BA 精化；跟踪点覆盖下降时插入关键帧，快速运动、模糊或弱纹理会导致丢失。）

**追问方向**：运动模型失效的情形；重定位的 RANSAC 参数。

### Q11 ORB-SLAM 里地图点如何创建、筛选与剔除？（Map point management）
**优先级：★★**

**答案要点**
- 创建：①初始化时三角化；②局部建图中，与共视关键帧的未匹配 ORB 特征基于**极线约束**匹配后三角化，检验视差、正深度、重投影误差、尺度一致性。
- 新地图点**测试期**：创建后前 3 个关键帧内必须满足——被跟踪的帧比例 > 25%（"找到/可见"比），且至少被 ≥3 个关键帧观测，否则剔除。
- 长期保留：被观测的稳定性；描述子取所有观测中汉明距离中位数最小的代表描述子。
- 剔除关键帧：90% 以上的地图点被至少其他 3 个关键帧在相同或更精细尺度下观测，则冗余。
- 目的：防止错误点污染 BA，控制地图规模；点的平均观测方向与深度范围用于投影匹配的可见性判断。

**English short answer**
New points are triangulated from unmatched features using epipolar matching and are checked for parallax, positive depth, reprojection error and scale consistency. During a probation period of a few keyframes a point must be found in more than a quarter of the frames where it is predicted visible and be seen by at least three keyframes, otherwise it is culled. Redundant keyframes whose points are mostly observed elsewhere are also removed, keeping the map clean and bounded.

（译：用极线匹配对未匹配特征三角化新点，并检查视差、正深度、重投影误差和尺度一致性；在几个关键帧的试用期内，点必须在预测可见的帧中超过四分之一被找到且至少被三个关键帧观测，否则剔除；点大多被其他关键帧观测的冗余关键帧也被移除，使地图干净且规模受控。）

**追问方向**：试用期规则的设计意图；代表描述子的选取。

### Q12 局部 BA 的范围怎么选？（Local bundle adjustment scope）
**优先级：★★** · 对应基础知识：[视觉里程计_特征_对极几何_PnP_BA](../../02_基础知识/07_SLAM与建图/02_视觉里程计_特征_对极几何_PnP_BA.md)

**答案要点**
- 变量：当前关键帧 + 其**共视图邻居**（共享地图点多的关键帧）= 可优化关键帧集；这些关键帧观测到的所有地图点 = 可优化点集；**固定**也观测到这些点但不在优化集中的其他关键帧（作为锚点，提供约束但不更新），避免整体漂移并控制规模。
- 目的：在不做全局 BA 的情况下，实时地提高局部一致性；复杂度受共视图邻居数控制。
- 鲁棒性：Huber 核；两轮优化，第一轮后剔除外点观测（卡方检验），第二轮再优化。
- 全局 BA 仅在回环后（或单独线程）进行，耗时大。
- 窗口选择类似：VIO 用时间滑窗，ORB-SLAM 用共视滑窗——后者适合回到老区域时仍有历史约束。

**English short answer**
Local BA optimizes the current keyframe and its covisible neighbors together with all map points they observe, while other keyframes that also see those points are held fixed as anchors. This bounds the problem size while improving local consistency in real time. I use a robust kernel and two rounds, removing outlier observations after the first, and reserve global BA for after loop closures.

（译：局部 BA 优化当前关键帧及其共视邻居和它们观测到的所有地图点，同时观测这些点的其他关键帧作为锚点保持固定；这在改善局部一致性的同时限制问题规模；使用鲁棒核和两轮优化（第一轮后剔除外点观测），全局 BA 留到回环之后。）

**追问方向**：共视窗口与时间窗口的区别；为什么要固定外围关键帧。

### Q13 为什么视觉 SLAM 的回环要用 Sim(3)？（Sim(3) for monocular loop closure）
**优先级：★★★** · 对应基础知识：[回环检测与重定位_词袋_ScanContext](../../02_基础知识/07_SLAM与建图/05_回环检测与重定位_词袋_ScanContext.md)

**答案要点**
- 单目 SLAM 的尺度不可观，随时间**漂移**：回环两端的局部地图尺度不同。相对变换不是 SE(3)（刚体）而是 **Sim(3)**（旋转、平移、尺度 7 自由度）。
- 流程：对回环候选关键帧与当前关键帧的 3D–3D 对应（由匹配的地图点）用 **Horn / Umeyama** 闭式解计算 Sim(3)；RANSAC 剔除外点并非线性精化；验证内点数。
- 位姿图优化在 **Sim(3)** 上进行：残差包含尺度项 $\log s$；优化后尺度漂移被消除，环两端一致。
- 双目 / RGB-D / VIO 有度量尺度，退化为 SE(3)（尺度固定为 1）。
- 面试表述：尺度漂移 → 回环需要 Sim(3) 位姿图。

**English short answer**
A monocular system's scale drifts, so the two ends of a loop may differ by a scale factor, and their relative transform is a similarity in Sim(3) with seven degrees of freedom. I compute it in closed form from 3D-3D map point correspondences with Umeyama or Horn inside RANSAC, then optimize the pose graph over Sim(3) so scale drift is distributed along the loop. With stereo, depth or inertial data the scale is metric and SE(3) suffices.

（译：单目系统的尺度会漂移，回环两端可能相差一个尺度因子，其相对变换是 7 自由度的 Sim(3) 相似变换；在 RANSAC 内用 Umeyama 或 Horn 闭式解由 3D–3D 地图点对应计算，再在 Sim(3) 上优化位姿图使尺度漂移沿环分布；双目、深度或惯性数据提供度量尺度时，SE(3) 就够了。）

**追问方向**：Umeyama 算法步骤；Sim(3) 位姿图的残差。

### Q14 动态物体对视觉 SLAM 的影响及应对？（Dynamic scenes in visual SLAM）
**优先级：★★★** · 对应基础知识：[多机SLAM与长期地图维护](../../02_基础知识/07_SLAM与建图/07_多机SLAM与长期地图维护.md)

**答案要点**
- 问题：行人、车辆等移动物体上的特征被当作静态路标，导致位姿估计偏差与地图污染。
- 对策：①**几何一致性**：RANSAC / 极线约束剔除不符合主导运动的特征；②**语义先验**：检测/分割潜在动态类别（人、车）并剔除其上特征（DS-SLAM、DynaSLAM 类）；半静态物体（停放车辆）需要谨慎；③**光流/场景流**与运动分割；④**多视图一致性**（地图点在不同视图中的重投影一致性）；⑤鲁棒核。
- 代价：语义分割增加算力；误剔除静态特征会减少约束（人群遮挡大视野时尤其）。
- 评估：在动态序列（TUM RGB-D dynamic）上的 ATE。
- 机器人：人多的场景（商场、仓库）需要融合轮速/IMU 以应对视觉被动物体主导的帧。

**English short answer**
Features on moving objects violate the static-world assumption and bias pose estimates and pollute the map. I reject them with geometric consistency checks such as RANSAC and epipolar constraints, with semantic detectors for people and vehicles, with optical flow based motion segmentation, and with robust kernels. This costs compute and can remove too many static features in crowds, so inertial or wheel odometry helps when dynamic objects dominate the view.

（译：运动物体上的特征违反静态世界假设，会使位姿估计产生偏差并污染地图；用 RANSAC、极线约束等几何一致性检查、针对人和车辆的语义检测、基于光流的运动分割以及鲁棒核剔除它们；这会增加算力，在人群中可能误删过多静态特征，当动态物体主导视野时，惯性或轮速里程计有帮助。）

**追问方向**：半静态物体的处理；语义剔除的误差来源。

### Q15 语义 SLAM 与学习型 SLAM（DROID-SLAM 等）的现状？（Semantic and learned SLAM）
**优先级：★★**

**答案要点**
- **语义 SLAM**：在地图中加入语义标签（物体级 / 稠密语义），用于动态过滤、语义回环、人机交互与任务规划；物体级 SLAM 把物体作为路标（位姿 + 形状），提供尺度与长期稳定地标。
- **学习型前端/特征**：SuperPoint + LightGlue、DISK、学习的光流（RAFT）、深度先验，提高弱纹理 / 光照变化下的鲁棒性。
- **端到端/学习型 SLAM**：DROID-SLAM（循环网络迭代更新位姿与稠密深度，配合可微 BA）、DPVO 等，在多项基准上鲁棒性高，但计算量与显存大，泛化与实时嵌入式部署仍受限（具体数据未核对）。
- **神经隐式 SLAM**：iMAP / NICE-SLAM / GS-SLAM，用 NeRF / 3DGS 表示地图，重建质量高但实时性与大场景挑战大。
- 面试表述：传统几何方法稳定、可解释、实时；学习方法提升鲁棒性，常作为模块（特征、深度、动态剔除）被融入。

**English short answer**
Semantic SLAM adds object or pixel labels to the map for dynamic filtering, semantic loop closure and task planning. Learned front ends such as SuperPoint with LightGlue improve robustness in low texture and lighting changes. End-to-end systems like DROID-SLAM combine recurrent updates with differentiable bundle adjustment and are very robust but heavy, and neural implicit SLAM builds NeRF or Gaussian maps with high fidelity but limited scale and speed. In practice learned modules are plugged into geometric pipelines.

（译：语义 SLAM 在地图中加入物体或像素标签，用于动态过滤、语义回环和任务规划；SuperPoint+LightGlue 等学习型前端在弱纹理和光照变化下更鲁棒；DROID-SLAM 等端到端系统把循环更新与可微 BA 结合，鲁棒但很重，神经隐式 SLAM 用 NeRF 或高斯构建高保真地图但规模与速度受限；实践中学习模块常被嵌入几何流水线。）

**追问方向**：物体级 SLAM 的优势；DROID-SLAM 的计算需求。

### Q16 在嵌入式平台上跑 SLAM 如何优化？（Running SLAM on embedded hardware）
**优先级：★★★** · 对应基础知识：[OpenHarmony_Hi3861_RK3588平台基础](../../02_基础知识/11_嵌入式与通信/06_OpenHarmony_Hi3861_RK3588平台基础.md)

**答案要点**
- 瓶颈：特征提取、匹配、BA、回环；ARM CPU（如 RK3588、树莓派）算力有限。
- 优化：降低分辨率/帧率（如 640×480、15–20 fps）；减少特征点数（每帧 500–1000）；使用 NEON/多线程/OpenMP；GPU 加速特征（CUDA ORB）或 NPU 跑学习特征；缩小局部 BA 窗口、降低 BA 迭代数；关键帧稀疏化；关闭可视化；使用更快的求解器（稀疏 Cholesky）；线程绑核与优先级；固定曝光避免特征抖动。
- 取舍：精度、延迟、鲁棒性三者；先测瓶颈（profiling），按收益排序。
- 替代：激光/轮速 + 2D SLAM（计算量小得多）；VIO 的轻量方案（OpenVINS 的 MSCKF 比 VINS 更轻）。
- 监控：处理耗时、跟踪特征数、丢帧；超时降级。

**English short answer**
I profile first, then reduce resolution and frame rate, cap the number of features, use NEON and multithreading, offload feature extraction to a GPU or NPU if available, shrink the local BA window and iteration count, sparsify keyframes and disable visualization. There is a trade-off between accuracy, latency and robustness. Where possible a 2D lidar SLAM or an MSCKF-style VIO is much cheaper, and I monitor processing time and tracked feature counts to degrade gracefully.

（译：先做 profiling，再降低分辨率与帧率、限制特征数、使用 NEON 与多线程、在有条件时把特征提取交给 GPU 或 NPU、缩小局部 BA 窗口和迭代数、稀疏化关键帧并关闭可视化；精度、延迟和鲁棒性之间需权衡；可能时用 2D 激光 SLAM 或 MSCKF 式 VIO 代价小得多，并监控处理耗时和跟踪特征数以便优雅降级。）

**追问方向**：特征数与精度的关系；MSCKF 为何更轻。

---

## 三、视觉惯性里程计（VIO）

### Q17 为什么要把 IMU 与相机融合？各自优缺点？（Why VIO）
**优先级：★★★** · 对应基础知识：[视觉惯性里程计VIO_预积分_滑动窗口](../../02_基础知识/07_SLAM与建图/04_视觉惯性里程计VIO_预积分_滑动窗口.md)

**答案要点**
- IMU：高频（100–1000 Hz）、短时精确、不依赖外界、可观测重力方向与度量尺度；但积分漂移快（位置误差随时间平方），不可长期使用。
- 相机：低频、提供无漂移的环境约束（相对位姿、回环）、信息丰富；但单目尺度不可观、对快速运动 / 模糊 / 弱纹理 / 光照敏感、需要特征。
- 互补：IMU 补偿相机在快速运动 / 短暂丢失时的预测并恢复**尺度**与重力方向（俯仰/横滚可观）；相机限制 IMU 的漂移与零偏。
- 结果：度量尺度、对快速运动更鲁棒、能估计零偏。
- 代价：需要精确的时间同步、外参标定、IMU 噪声参数、较复杂的初始化。

**English short answer**
An IMU is high-rate and accurate over short intervals, observes gravity and gives metric scale, but integration drifts quadratically. A camera is low-rate but provides drift-free environmental constraints, though monocular scale is unobservable and it fails under fast motion or low texture. Fusing them yields metric scale, robust prediction during fast motion and bias estimation, at the cost of tight time synchronization, extrinsic calibration and a more complex initialization.

（译：IMU 高频、短时精确、能观测重力并给出度量尺度，但积分漂移随时间二次增长；相机低频，提供无漂移的环境约束，但单目尺度不可观，在快速运动或弱纹理下失效；融合得到度量尺度、快速运动中的鲁棒预测和零偏估计，代价是需要严格的时间同步、外参标定和更复杂的初始化。）

**追问方向**：IMU 位置误差随时间平方增长的原因；VIO 初始化为什么难。

### Q18 IMU 测量模型：零偏、噪声与重力？（IMU measurement model）
**优先级：★★★** · 对应基础知识：[传感器_IMU_编码器_激光雷达](../../02_基础知识/06_状态估计/01_传感器_IMU_编码器_激光雷达.md)

**答案要点**
- 陀螺：$\tilde{\boldsymbol\omega}=\boldsymbol\omega+\mathbf{b}_g+\mathbf{n}_g$；加速度计：$\tilde{\mathbf{a}}=\mathbf{R}^\top(\mathbf{a}_w-\mathbf{g}_w)+\mathbf{b}_a+\mathbf{n}_a$（测量的是比力：含重力反向项）。噪声 $\mathbf{n}$ 为白噪声（密度 $\sigma_g,\sigma_a$），零偏 $\mathbf{b}$ 为随机游走（$\dot{\mathbf{b}}=\mathbf{n}_b$，密度 $\sigma_{bg},\sigma_{ba}$）。
- 参数来源：数据手册或静止数据的 Allan 方差；通常取实测值的 1.5–3 倍以覆盖未建模误差（经验）。
- 此外：尺度因子/非正交/温度漂移（高精度标定中考虑）；振动整流误差、g-sensitivity。
- 重力：世界系下 $\mathbf{g}_w\approx(0,0,-9.81)^\top$ m/s²（取决于约定）；初始化时估计重力方向。
- 时间戳与离散化：离散噪声方差 = 连续密度²/Δt（白噪声）。

**English short answer**
The gyroscope measures angular velocity plus a slowly varying bias and white noise, and the accelerometer measures specific force, which includes the opposite of gravity, again with bias and noise. Biases are modeled as random walks, and noise densities come from datasheets or Allan variance, often inflated to cover unmodeled errors. In discrete time the white-noise variance is the density squared divided by the sample period, and gravity direction must be estimated at initialization.

（译：陀螺测量角速度加缓变零偏和白噪声，加速度计测量比力（含重力的反向项）同样带零偏与噪声；零偏建模为随机游走，噪声密度来自数据手册或 Allan 方差，常放大以覆盖未建模误差；离散时白噪声方差等于密度平方除以采样周期，初始化时需估计重力方向。）

**追问方向**：为什么加速度计测量的是比力；噪声参数放大多少合适。

### Q19 IMU 预积分的原理和好处？（IMU preintegration）
**优先级：★★★** · 对应基础知识：[IMU预积分与因子图估计](../../02_基础知识/06_状态估计/08_IMU预积分与因子图估计.md)

**答案要点**
- 问题：优化时关键帧 $i,j$ 之间有许多 IMU 测量；若每次状态改变（位姿、速度、零偏）都重新积分，代价巨大。
- 预积分：把区间 $[i,j]$ 的 IMU 测量积分成**与初始状态（位姿/速度）无关**的相对量 $\Delta\mathbf{R}_{ij},\Delta\mathbf{v}_{ij},\Delta\mathbf{p}_{ij}$（在 body 系 $i$ 下，用已知零偏）；之后约束为 $\mathbf{R}_j=\mathbf{R}_i\Delta\mathbf{R}_{ij}$，$\mathbf{v}_j=\mathbf{v}_i+\mathbf{g}\Delta t+\mathbf{R}_i\Delta\mathbf{v}_{ij}$，$\mathbf{p}_j=\mathbf{p}_i+\mathbf{v}_i\Delta t+\tfrac12\mathbf{g}\Delta t^2+\mathbf{R}_i\Delta\mathbf{p}_{ij}$。
- 优点：状态更新时不需重新积分（仅用雅可比对零偏做一阶修正）；预积分量和协方差递推；构成一个因子（IMU 因子）加入因子图/滑窗。
- 旋转用 SO(3) 流形（Forster 等的预积分理论），避免欧拉角奇异。
- 零偏变化大时需重新积分（预积分对偏置的一阶近似失效）。

**English short answer**
Instead of re-integrating raw IMU data every time the optimizer changes the states, preintegration summarizes the measurements between two keyframes into relative rotation, velocity and position increments that are independent of the initial state, expressed in the first body frame. The resulting factor relates consecutive states with gravity and time, its covariance is propagated recursively, and small bias changes are handled with first-order Jacobians. If the bias changes a lot, the integration must be redone.

（译：不在每次优化改变状态时重新积分原始 IMU 数据，预积分把两个关键帧之间的测量总结为与初始状态无关、在第一个体坐标系下表达的相对旋转、速度与位置增量；由此得到的因子结合重力与时间关联相邻状态，其协方差递推传播，小的零偏变化用一阶雅可比修正；零偏变化大时需重新积分。）

**追问方向**：预积分对零偏的雅可比如何递推；为什么要在 SO(3) 上处理旋转。

### Q20 VIO 的初始化为什么困难？VINS-Mono 怎么做？（VIO initialization）
**优先级：★★★** · 对应基础知识：[视觉惯性里程计VIO_预积分_滑动窗口](../../02_基础知识/07_SLAM与建图/04_视觉惯性里程计VIO_预积分_滑动窗口.md)

**答案要点**
- 难点：需要同时估计尺度、重力方向、速度、陀螺零偏、（加速度计零偏）；初期运动激励不足则不可观；错误的初始化会让整个系统后续发散。
- VINS-Mono 流程：①纯视觉 SfM 在滑窗内重建关键帧位姿和特征点（尺度未知）；②**视觉–惯性对齐**：用陀螺预积分估计陀螺零偏（旋转一致性），线性求解**速度、重力向量与尺度**（位置约束的线性最小二乘），并细化重力（流形上 2 自由度）；③将世界系对齐到重力方向，进入紧耦合优化。
- 要求：初始化阶段有足够的**加速度激励**（平移 + 非匀速运动）和视差；静止或匀速直线运动下尺度/重力不可分。
- 替代：OpenVINS 的静态初始化（静止时用加速度计估计重力方向和零偏）或动态初始化；ORB-SLAM3 的 IMU 初始化（尺度与重力的 MAP 估计，0.5–2 s 级）。
- 工程：启动时让机器人先静止几秒再运动。

**English short answer**
Initialization must jointly recover scale, gravity direction, velocity and gyro bias, which are only observable with enough excitation. VINS-Mono first runs vision-only structure from motion in a window with unknown scale, then aligns it with preintegrated IMU data: it estimates gyro bias from rotation consistency, solves velocity, gravity and scale linearly and refines gravity, before starting tight optimization. Static or constant-velocity starts make scale and gravity inseparable, so I let the robot rest first and then move with acceleration.

（译：初始化需联合恢复尺度、重力方向、速度和陀螺零偏，这些量只有在激励充分时才可观；VINS-Mono 先在窗口内做尺度未知的纯视觉 SfM，再与预积分 IMU 数据对齐：由旋转一致性估计陀螺零偏、线性求解速度、重力与尺度并精化重力，然后开始紧耦合优化；静止或匀速直线启动会使尺度与重力不可分，所以先让机器人静止再加速运动。）

**追问方向**：为什么匀速直线运动尺度不可观；重力精化的 2 自由度参数化。

### Q21 滑动窗口 VIO 的结构（VINS-Mono）？（Sliding-window VIO）
**优先级：★★★** · 对应基础知识：[视觉惯性里程计VIO_预积分_滑动窗口](../../02_基础知识/07_SLAM与建图/04_视觉惯性里程计VIO_预积分_滑动窗口.md)

**答案要点**
- 状态：窗口内 $N$ 个关键帧（约 10 帧）的位姿、速度、IMU 零偏，加窗内特征点的逆深度，以及外参（可选）。
- 因子：①IMU 预积分因子；②视觉重投影因子（特征点在各帧的观测）；③边缘化先验因子（滑出窗口的状态与特征被边缘化后的信息）。
- 流程：新图像来 → KLT 跟踪特征 → 判断关键帧（视差/跟踪质量）→ 加入窗口并优化（Ceres LM）→ 若是关键帧则边缘化最老帧，否则边缘化次新帧（保持 IMU 连续）→ 回环模块（BoW + 4-DoF 位姿图优化，因为重力使 roll/pitch 可观）。
- 优点：计算量有界（窗口大小固定）、实时、紧耦合精度高。
- 局限：窗口外信息只能通过先验保留（线性化误差），长时间漂移需要回环；可观性相关 FEJ。

**English short answer**
The window holds around ten keyframes with poses, velocities, biases and feature inverse depths, and it contains IMU preintegration factors, visual reprojection factors and a marginalization prior. After optimization, the oldest frame is marginalized if the new frame is a keyframe, otherwise the second newest is dropped. Computation is bounded, but information outside the window survives only as a linearized prior, so long-term drift is corrected by loop closure with a four-degree-of-freedom pose graph.

（译：窗口包含约十个关键帧的位姿、速度、零偏和特征逆深度，因子有 IMU 预积分因子、视觉重投影因子和边缘化先验；优化后若新帧是关键帧则边缘化最老帧，否则丢弃次新帧；计算量有界，但窗口外信息只以线性化的先验保留，长期漂移通过四自由度位姿图的回环修正。）

**追问方向**：为什么回环用 4 自由度位姿图；逆深度参数化的好处。

### Q22 MSCKF 的核心思想（多状态约束卡尔曼滤波）？（MSCKF）
**优先级：★★** · 对应基础知识：[误差状态卡尔曼ESKF与流形上的滤波](../../02_基础知识/06_状态估计/09_误差状态卡尔曼ESKF与流形上的滤波.md)

**答案要点**
- 滤波式 VIO：状态包含当前 IMU 状态 + 滑窗内最近 $N$ 个相机位姿（克隆），**不把路标点加入状态**。
- 对每个特征，当它被多帧观测完成后，**三角化**得到估计点，再把重投影残差**投影到零空间**（消去对路标点位置的依赖），得到只约束相机位姿（多状态）的残差，用 EKF 更新——因此状态维数与特征数无关，复杂度线性于特征数。
- 优点：计算量小、实现轻（OpenVINS 的基础），精度接近优化法；缺点：一次线性化，非线性误差难纠正，需 FEJ 保证一致性；回环需额外处理。
- 对比 VINS：VINS（优化）精度高、重线性化；MSCKF（滤波）更省算力。
- 适合：嵌入式 / 无人机。

**English short answer**
MSCKF is a filter-based VIO whose state holds the current IMU state and a window of past camera poses but no landmarks. When a feature track ends, it is triangulated and its reprojection residuals are projected onto the null space to remove the dependence on the landmark position, giving constraints only among camera poses used in an EKF update. The cost is linear in the number of features and light enough for embedded use, though it linearizes once and needs first-estimate Jacobians for consistency.

（译：MSCKF 是基于滤波的 VIO，状态包含当前 IMU 状态和一个窗口的过去相机位姿，但不含路标；特征轨迹结束时三角化并将重投影残差投影到零空间以消去对路标位置的依赖，得到只约束相机位姿的约束并用于 EKF 更新；代价与特征数呈线性，足够轻量可用于嵌入式，但只线性化一次，需 FEJ 来保持一致性。）

**追问方向**：零空间投影的构造；MSCKF 与 EKF-SLAM 的区别。

### Q23 VIO 的可观性：哪些量不可观？为什么重要？（VIO observability）
**优先级：★★★** · 对应基础知识：[估计一致性_可观性分析_NEES与NIS](../../02_基础知识/06_状态估计/10_估计一致性_可观性分析_NEES与NIS.md)

**答案要点**
- VIO 的标准结论：**全局位置（3）和绕重力方向的偏航角（1）共 4 个自由度不可观**；重力方向（俯仰/横滚）、尺度、速度、IMU 零偏在激励充分时可观。因此漂移主要体现在位置和 yaw。
- 退化运动：匀速直线运动、纯旋转（尺度不可观）、静止（速度零偏混淆）、仅沿重力方向运动等使额外自由度不可观。
- 对滤波器的意义：EKF 因线性化点不一致会**虚假地获得不可观方向的信息**，导致协方差过小；解决：FEJ、可观性约束 EKF（OC-EKF）、不变 EKF、ESKF。
- 对回环的意义：回环位姿图只需 4 自由度（x,y,z,yaw）。
- 工程意义：知道不可观量，才会设计合适的先验、固定参考与激励动作。

**English short answer**
In visual-inertial odometry, global position and yaw about gravity, four degrees of freedom, are unobservable, while roll, pitch, scale, velocity and biases are observable with sufficient excitation. Degenerate motions such as constant velocity or pure rotation add more unobservable directions. A naive EKF gains spurious information in unobservable directions because of linearization, so observability-constrained or first-estimate-Jacobian filters are used, and loop closure needs only a four-degree-of-freedom pose graph.

（译：视觉惯性里程计中全局位置和绕重力的偏航共 4 个自由度不可观，而俯仰、横滚、尺度、速度和零偏在激励充分时可观；匀速或纯旋转等退化运动会增加不可观方向；朴素 EKF 因线性化会在不可观方向上虚假获得信息，所以使用可观性约束或 FEJ 滤波器，回环只需 4 自由度位姿图。）

**追问方向**：为什么 yaw 不可观；OC-EKF 的做法。

### Q24 VIO 在线估计外参与时间偏移有什么注意点？（Online extrinsic and time offset）
**优先级：★★** · 对应基础知识：[时间同步_坐标系一致性与数据有效性](../../02_基础知识/06_状态估计/06_时间同步_坐标系一致性与数据有效性.md)

**答案要点**
- 外参 $\mathbf{T}_{ic}$ 和时间偏移 $t_d$ 作为状态（先验给初值，随时间收敛）。时间偏移的做法：特征观测的时刻修正为 $t+t_d$，用特征速度（像素运动）线性近似该偏移对观测位置的影响，加入优化。
- 可观性：需要足够旋转 + 平移激励；退化运动时外参/偏移不可观或与零偏耦合 → 应在不激励时**冻结**（设大先验或固定）。
- 风险：过早估计导致发散；估计值漂移影响一致性；在线估计不能替代好的离线标定，只作精化。
- 校验：估计值是否稳定收敛、是否在合理范围（机械安装限制）；日志记录。
- 实务：先离线标定（Kalibr）得到初值，运行时开启在线估计并监控。

**English short answer**
Extrinsics and the camera-IMU time offset can be added to the state, with the offset entering by shifting observation times and linearizing with feature velocity. They are only observable under sufficient rotational and translational excitation, so I freeze them when motion is degenerate and use offline calibration for the initial guess. I monitor that estimates converge to a stable, physically plausible range.

（译：外参和相机–IMU 时间偏移可加入状态，偏移通过平移观测时刻并用特征速度线性化纳入；只有旋转与平移激励充分时才可观，所以运动退化时冻结它们，并用离线标定提供初值；监控估计值是否收敛到稳定且物理合理的范围。）

**追问方向**：时间偏移线性化的推导；冻结外参的实现方式。

### Q25 VINS-Mono、ORB-SLAM3 与 OpenVINS 的对比？（VIO system comparison）
**优先级：★★** · 对应基础知识：[视觉惯性里程计VIO_预积分_滑动窗口](../../02_基础知识/07_SLAM与建图/04_视觉惯性里程计VIO_预积分_滑动窗口.md)

**答案要点**
- **VINS-Mono/Fusion**：滑窗优化 + KLT 光流 + 回环 4-DoF 位姿图；稳健、易部署、单目 + IMU，支持在线外参/时间偏移；全局一致性靠回环。
- **ORB-SLAM3**：特征点 ORB + 共视图局部 BA + 多地图 Atlas；支持单目/双目/RGB-D + IMU、鱼眼；精度高（基准上常领先），重定位与地图复用强；资源占用较大。
- **OpenVINS**：MSCKF 滤波 + FEJ + 在线标定，实现模块化、文档好，适合研究与嵌入式；精度与优化法接近，对参数较敏感。
- 其他：Basalt（视觉惯性批处理 / 滑窗）、Kimera-VIO、ROVIO（直接光度 EKF）。
- 选择：要多地图与重定位 → ORB-SLAM3；轻量部署 → OpenVINS / VINS；研究可重现 → OpenVINS（具体性能以基准为准，未核对）。

**English short answer**
VINS-Mono uses sliding-window optimization with KLT tracking and a four-degree-of-freedom loop closure, and is robust and easy to deploy. ORB-SLAM3 offers feature-based visual-inertial SLAM with multi-map Atlas, relocalization and fisheye support, with high accuracy and heavier resources. OpenVINS is an MSCKF-based filter with first-estimate Jacobians and online calibration that is modular and lightweight. I choose by need for relocalization, resources and reproducibility.

（译：VINS-Mono 用滑窗优化、KLT 跟踪和四自由度回环，鲁棒易部署；ORB-SLAM3 提供基于特征的视觉惯性 SLAM，带多地图 Atlas、重定位和鱼眼支持，精度高但资源占用较大；OpenVINS 是基于 MSCKF 的滤波器，含 FEJ 与在线标定，模块化且轻量；按重定位需求、资源与可复现性选择。）

**追问方向**：KLT 与描述子匹配的取舍；FEJ 对 OpenVINS 的意义。

### Q26 VIO 的典型失败模式与对策？（VIO failure modes）
**优先级：★★★** · 对应基础知识：[可靠性与故障诊断_FMEA_看门狗_降级策略](../../02_基础知识/13_系统工程与安全/03_可靠性与故障诊断_FMEA_看门狗_降级策略.md)

**答案要点**
- 视觉退化：弱纹理 / 过曝 / 低照 / 快速运动模糊 / 遮挡 / 动态物体占主导 → 特征数骤降；对策：IMU 短时推算、曝光管理、主动照明、提高帧率、特征筛选。
- 运动退化：静止（速度/零偏不可观）、匀速直线（尺度不可观）、纯旋转 → 估计发散或尺度漂移；对策：激励动作、速度零更新（ZUPT）、轮速约束。
- 参数错误：IMU 噪声参数过小 → 过度信任 IMU；外参 / 时间偏移不准 → 系统性偏差；时间戳抖动。
- 初始化错误：无激励或错误尺度。
- 数值：协方差失去正定、线性化误差。
- 监控与恢复：特征数、内点比、重投影残差、零偏幅值、尺度因子变化、位姿跳变；失败检测后**重新初始化**并通知上层（降速/停车）。

**English short answer**
Vision can degrade with weak texture, extreme lighting, blur or dynamic objects, in which case the IMU bridges a short gap. Motion can degrade when stationary or at constant velocity, making scale or bias unobservable, which excitation, zero-velocity updates or wheel odometry can address. Wrong noise parameters, extrinsics or timestamps cause systematic errors. I monitor tracked features, inlier ratio, residuals, bias magnitudes and pose jumps, and trigger reinitialization with a degraded mode for the robot.

（译：视觉可能因弱纹理、极端光照、模糊或动态物体而退化，IMU 可短时桥接；运动可能因静止或匀速而退化使尺度或零偏不可观，可用激励动作、零速更新或轮速里程计应对；噪声参数、外参或时间戳错误会造成系统性误差；监控跟踪特征数、内点比、残差、零偏幅值与位姿跳变，并在失败时触发重新初始化及让机器人进入降级模式。）

**追问方向**：ZUPT 的原理；位姿跳变的检测阈值。

---

## 四、激光 SLAM

### Q27 Cartographer 的子图、扫描匹配与分支定界回环？（Cartographer）
**优先级：★★★** · 对应基础知识：[激光SLAM_LOAM_LIO-SAM_Cartographer](../../02_基础知识/07_SLAM与建图/03_激光SLAM_LOAM_LIO-SAM_Cartographer.md)

**答案要点**
- **局部 SLAM**：扫描经过滤波/体素化，与当前**子图**（submap，一小块概率栅格，约几十到上百帧）通过 **Ceres 扫描匹配**（把扫描点插值到概率栅格，最大化占据概率之和，并带位姿先验惩罚）对齐；IMU 提供姿态/重力；匹配成功的扫描插入子图；子图“完成”后冻结。
- **全局 SLAM（后端）**：子图与扫描之间构建约束；对新扫描在已完成子图中搜索回环候选，用**分支定界扫描匹配**（多分辨率预计算栅格，保证在窗口内找到全局最优匹配）加速；满足得分阈值的加入**稀疏位姿图（SPA）**，周期性用 Ceres 优化。
- 特点：子图使局部累积误差小；回环无需描述子；实时 2D/3D；参数很多（匹配权重、分数阈值、约束采样比）。
- 2D 室内中型环境表现好；纯里程计初值可选。
- 面试要点：局部匹配 + 子图 + 分支定界回环 + 位姿图优化。

**English short answer**
Cartographer matches each filtered scan against the current submap, a small probability grid, with a Ceres-based scan matcher regularized by a pose prior, using IMU for gravity, and inserts it into the submap. For global consistency it searches for loop closure constraints between new scans and finished submaps using branch-and-bound scan matching on precomputed multi-resolution grids, then optimizes a sparse pose graph periodically. Submaps limit local drift and no descriptors are needed.

（译：Cartographer 用基于 Ceres 的扫描匹配（带位姿先验正则）把每个滤波后的扫描与当前子图（小块概率栅格）对齐，借助 IMU 获取重力，并插入子图；为获得全局一致性，用预计算多分辨率栅格上的分支定界扫描匹配在新扫描与已完成子图之间搜索回环约束，并周期性优化稀疏位姿图；子图限制局部漂移，无需描述子。）

**追问方向**：分支定界的上界如何构造；子图大小的选择。

### Q28 Hector SLAM 与 gmapping、Cartographer 的差别？（Hector vs gmapping vs Cartographer）
**优先级：★★**

**答案要点**
- **Hector SLAM**：不使用里程计，纯扫描匹配（高斯–牛顿，在多分辨率占据栅格上对齐当前扫描），高频（激光 40 Hz 级）、低延迟，适合手持/无人机/无轮速场景；无回环，需平稳运动、扫描质量高；地面倾斜时可结合 IMU。
- **gmapping**：RBPF 粒子滤波 + 里程计，需要良好里程计；大环境内存大；无显式回环。
- **Cartographer / slam_toolbox**：图优化 + 回环，适合大场景。
- 选择：无里程计的手持/小场景 → Hector；有轮速、中小室内 → gmapping / slam_toolbox；大场景长期运行 → Cartographer / slam_toolbox（支持序列化与地图续建）。
- 失败：Hector 在长走廊/退化场景漂移，gmapping 在回环处易错。

**English short answer**
Hector SLAM uses only high-rate scan matching on a multi-resolution occupancy grid without odometry, so it suits handheld or aerial platforms, but it has no loop closure and fails in featureless corridors. gmapping is a Rao-Blackwellized particle filter that needs good odometry and grows memory with large maps. Cartographer and slam_toolbox use graph optimization with loop closure for large environments.

（译：Hector SLAM 只在多分辨率占据栅格上做高频扫描匹配而不需要里程计，适合手持或空中平台，但无回环且在无特征走廊中失败；gmapping 是需要良好里程计的 Rao-Blackwell 粒子滤波，大地图内存增长；Cartographer 与 slam_toolbox 用带回环的图优化处理大场景。）

**追问方向**：Hector 为何不需要里程计；slam_toolbox 的特点。

### Q29 FAST-LIO2 的核心设计（IESKF + ikd-tree）？（FAST-LIO2）
**优先级：★★** · 对应基础知识：[激光SLAM_LOAM_LIO-SAM_Cartographer](../../02_基础知识/07_SLAM与建图/03_激光SLAM_LOAM_LIO-SAM_Cartographer.md)

**答案要点**
- **紧耦合**：误差状态**迭代扩展卡尔曼滤波**（IESKF，在流形上），用 IMU 做前向传播（去畸变使用传播得到的位姿），用点云对地图的**点到面**残差做迭代更新；状态含位姿、速度、IMU 零偏、重力、外参（可选）。
- **直接法**：不提取边缘/平面特征，直接把降采样后的原始点匹配到局部地图中的平面（用最近 5 个邻点拟合平面）。
- **增量地图**：**ikd-tree**（增量式 KD 树，支持点插入、删除与自动再平衡，并按体素降采样），避免每帧重建 KD 树，效率高。
- 特点：高频（可到数百 Hz 的 IMU 传播）、计算轻、鲁棒（快速运动）；无内置回环（可外接）。
- 局限：退化环境；地图随时间增大；与重定位结合需要额外模块。

**English short answer**
FAST-LIO2 runs an iterated error-state Kalman filter on the manifold: IMU measurements propagate the state, which also deskews the scan, and the filter iteratively updates with direct point-to-plane residuals against planes fitted from nearest neighbors in the map, without feature extraction. An incremental KD-tree supports insertion, deletion and rebalancing so the map is updated without rebuilding. It is fast and robust to quick motion, but has no loop closure and can degenerate in featureless scenes.

（译：FAST-LIO2 在流形上运行迭代误差状态卡尔曼滤波：IMU 测量传播状态并用于扫描去畸变，滤波器用与地图中最近邻拟合平面的直接点到面残差迭代更新，无需特征提取；增量 KD 树支持插入、删除与再平衡，无需重建即可更新地图；速度快、对快速运动鲁棒，但无回环且在无特征场景中可能退化。）

**追问方向**：迭代更新与普通 EKF 的差异；ikd-tree 的再平衡机制。

### Q30 LIO-SAM 的因子图结构？（LIO-SAM factor graph）
**优先级：★★** · 对应基础知识：[激光SLAM_LOAM_LIO-SAM_Cartographer](../../02_基础知识/07_SLAM与建图/03_激光SLAM_LOAM_LIO-SAM_Cartographer.md)

**答案要点**
- 基于 **iSAM2** 的因子图，因子类型：①**IMU 预积分因子**（关键帧间）；②**激光里程计因子**（当前关键帧与局部地图/前一关键帧配准得到的相对位姿，来自 LOAM 式边缘/平面特征匹配的 scan-to-map）；③**GPS 因子**（可选，抑制长期漂移）；④**回环因子**（基于欧氏距离的候选 + ICP 验证）。
- 流程：IMU 预积分用于点云去畸变与提供激光配准初值；仅在选取的**关键帧**上添加因子；关键帧附近的局部地图（滑动窗口内的子关键帧）用于配准，而非整个全局地图 → 实时。
- 优点：结构清晰、可融合多传感器、iSAM2 增量；缺点：特征提取依赖结构化环境、回环候选简单（欧氏距离）、不同 LiDAR 需调参。
- 与 FAST-LIO2：LIO-SAM 是“优化 + 关键帧”，FAST-LIO2 是“滤波 + 直接点云”，各有优势。

**English short answer**
LIO-SAM builds an iSAM2 factor graph over keyframes with IMU preintegration factors, LiDAR odometry factors from scan-to-local-map registration of edge and planar features, optional GPS factors and loop closure factors found by Euclidean-distance candidates verified with ICP. IMU preintegration also deskews scans and seeds registration, and matching against a local sliding map keeps it real time. It is flexible, while FAST-LIO2 takes a filter-based direct approach.

（译：LIO-SAM 在关键帧上构建 iSAM2 因子图，因子包括 IMU 预积分因子、由边缘与平面特征的帧到局部地图配准得到的激光里程计因子、可选 GPS 因子，以及由欧氏距离候选经 ICP 验证的回环因子；IMU 预积分还用于扫描去畸变并为配准提供初值，与局部滑动地图匹配保证实时；它灵活，而 FAST-LIO2 采用基于滤波的直接方法。）

**追问方向**：GPS 因子的作用；关键帧选择准则。

### Q31 激光 SLAM 的几何退化与应对？（Degeneracy in LiDAR SLAM）
**优先级：★★★** · 对应基础知识：[激光SLAM_LOAM_LIO-SAM_Cartographer](../../02_基础知识/07_SLAM与建图/03_激光SLAM_LOAM_LIO-SAM_Cartographer.md)

**答案要点**
- 退化场景：长走廊、空旷广场、隧道、平坦大草地——几何约束只在部分方向，沿走廊方向的平移不可观，扫描匹配沿该方向"滑动"。
- 检测：对 Gauss-Newton Hessian $\mathbf{J}^\top\mathbf{J}$ 做特征分解，最小特征值小于阈值（或条件数大）→ 对应特征向量是退化方向（Zhang 等的退化因子）。
- 应对：①只在可观方向更新（在退化方向保持预测值，即 solution remapping）；②融合 **IMU / 轮速里程计**（紧耦合时 IMU 提供先验）；③加入其他约束：地面、GNSS、视觉、UWB；④增强特征（强度、反射标志、人工路标）；⑤降低速度；⑥回环/地图匹配。
- 工程：走廊中常用轮速 + 激光融合；监控退化指标并上报。

**English short answer**
In corridors, tunnels or open fields, geometry constrains only some directions, so scan matching slides along the unconstrained axis. I detect it from small eigenvalues or a large condition number of the Gauss-Newton Hessian, then update only observable directions, relying on IMU or wheel odometry for the rest, and add other constraints such as GNSS, vision, intensity features or artificial landmarks. I also monitor the degeneracy metric and slow down.

（译：在走廊、隧道或空旷场地，几何只约束部分方向，扫描匹配沿未被约束的轴滑动；通过高斯–牛顿 Hessian 的小特征值或大条件数检测，仅在可观方向更新，其余方向依赖 IMU 或轮速里程计，并加入 GNSS、视觉、强度特征或人工路标等约束；同时监控退化指标并降速。）

**追问方向**：solution remapping 的实现；强度信息如何缓解走廊退化。

### Q32 Scan Context 回环检测的做法？（Scan Context）
**优先级：★★** · 对应基础知识：[回环检测与重定位_词袋_ScanContext](../../02_基础知识/07_SLAM与建图/05_回环检测与重定位_词袋_ScanContext.md)

**答案要点**
- 全局描述子：将 3D 点云按**方位角（扇区）× 半径（环）**划分极坐标网格，每格取该格内点的**最大高度**，得到 $N_s\times N_r$ 矩阵（Scan Context）；同时提取旋转不变的 **Ring Key**（每环的均值）用于快速 KD 树检索候选。
- 匹配：计算两描述子的列方向（扇区）上的余弦距离，**对列循环移位搜索**以对齐不同朝向（视角偏航变化）；最小距离 + 对应移位 → 粗略 yaw 偏差，用作 ICP 初值。
- 优点：简单、快速、对点云密度与朝向变化鲁棒、无需训练；缺点：对平移变化较敏感（同一位置不同横向偏移），对高度结构弱的场景（空旷）区分力弱；Scan Context++ 增强平移鲁棒；Intensity Scan Context 加强度。
- 回环验证：距离阈值 + ICP 内点/残差。

**English short answer**
Scan Context partitions the point cloud into azimuth sectors and radial rings, stores the maximum height in each cell to form a matrix, and uses the ring-wise average as a rotation-invariant key to retrieve candidates quickly. Candidate matching compares the matrices with column shifts to account for yaw, giving a coarse rotation for ICP. It needs no training and is robust to density and heading changes, but is sensitive to lateral offsets and weak in height-poor scenes.

（译：Scan Context 把点云划分成方位扇区和径向环，每格存最大高度形成矩阵，并用按环的平均值作为旋转不变的键来快速检索候选；候选匹配通过列移位比较矩阵以处理偏航，得到粗略旋转供 ICP 使用；无需训练、对密度和朝向变化鲁棒，但对横向偏移敏感，在高度信息少的场景较弱。）

**追问方向**：Ring Key 的设计；与学习型全局描述子的对比。

### Q33 平面运动约束与地面约束如何帮助 SLAM？（Planar and ground constraints）
**优先级：★★**

**答案要点**
- 地面机器人在近似平面上运动：$z$、roll、pitch 变化很小 → 可在状态估计中加入**软约束**（对 $z$、roll、pitch 的先验/伪观测，信息矩阵设合理权重）或直接降维到 SE(2)（2D 激光 SLAM）。
- 好处：抑制 3D SLAM 在 $z$ 方向的漂移（尤其 LiDAR 退化时）、提高鲁棒性；LIO-SAM 的 `z` 约束、地面分割 + 平面方程约束（LeGO-LOAM 用地面估计 z/roll/pitch）。
- 风险：坡道、台阶、颠簸地面会使约束错误 → 约束权重不宜过大，或仅在检测到平坦时启用；轮式机器人 pitch/roll 实际有悬挂晃动。
- 另一类约束：车辆非完整约束（侧向速度接近 0）作为伪量测，轮速 + 非完整约束可降低 IMU 漂移。

**English short answer**
For a ground robot I can add soft priors on height, roll and pitch, or reduce the state to SE(2), which suppresses vertical drift and improves robustness when lidar degenerates. Ground-plane fits provide z, roll and pitch estimates as in LeGO-LOAM. These constraints are wrong on slopes or steps, so I keep their weights moderate or enable them only on flat ground. Non-holonomic constraints on lateral velocity are another useful pseudo-measurement.

（译：对地面机器人可对高度、横滚和俯仰加软先验，或把状态降为 SE(2)，抑制垂直漂移并在激光退化时提高鲁棒性；地面平面拟合可提供 z、横滚和俯仰估计（如 LeGO-LOAM）；这些约束在坡道或台阶上是错误的，所以权重要适中或仅在平地启用；侧向速度的非完整约束也是有用的伪量测。）

**追问方向**：软约束的信息矩阵如何设置；非完整约束的适用条件。

### Q34 2D 激光里程计（PL-ICP/CSM）与 3D 激光里程计的区别？（2D laser scan matching）
**优先级：★★** · 对应基础知识：[激光SLAM_LOAM_LIO-SAM_Cartographer](../../02_基础知识/07_SLAM与建图/03_激光SLAM_LOAM_LIO-SAM_Cartographer.md)

**答案要点**
- 2D 激光扫描在平面内：位姿 SE(2)=(x,y,θ)，3 自由度，计算量小。**ICP / PL-ICP（点到线 ICP）**：把目标扫描局部拟合为线段，最小化点到线距离，比点到点收敛快；**相关扫描匹配（CSM / Real-time CSM）**：在位姿搜索窗口内枚举位姿，评分求和（栅格上的似然），可得到全局最优（窗口内）且有协方差估计，常用于回环候选与初值。
- 3D 激光：SE(3) 6 自由度，点到面/点到线，需要去畸变，数据量大，使用体素与空间索引；可用地面与 IMU 辅助。
- 选择：室内平面场景用 2D 即可，成本低；起伏地形 / 多层 / 无人机用 3D。
- 相关库：laser_scan_matcher、slam_toolbox 的 Karto 匹配、Cartographer。

**English short answer**
2D scan matching estimates a planar pose with three degrees of freedom. Point-to-line ICP converges faster than point-to-point by fitting local line segments, and correlative scan matching exhaustively scores poses in a search window against a likelihood grid, giving a global optimum within the window and a covariance, which is useful for loop closure. 3D lidar estimates six degrees of freedom with point-to-plane residuals and needs deskewing and spatial indexing. Indoor planar settings usually need only 2D.

（译：2D 扫描匹配估计 3 自由度的平面位姿；点到线 ICP 通过拟合局部线段比点到点收敛更快，相关扫描匹配在搜索窗口内遍历位姿并对似然栅格评分，窗口内可得全局最优并给出协方差，适用于回环；3D 激光估计 6 自由度，使用点到面残差并需要去畸变和空间索引；室内平面场景通常只需 2D。）

**追问方向**：CSM 的计算量如何降低；PL-ICP 的线段拟合。

### Q35 如何把 3D 地图转换为导航用的 2D 栅格地图？（3D to 2D map for navigation）
**优先级：★★★** · 对应基础知识：[代价地图_局部规划_TEB](../../02_基础知识/08_规划与导航/03_代价地图_局部规划_TEB.md)

**答案要点**
- 步骤：点云地图 → 去除天花板/过高点（高度范围）→ 地面分割 → 取机器人高度范围（离地间隙 ~ 机身高度）内的点投影到 XY 栅格 → 占据（有点）/ 自由（射线可通过）/ 未知；用射线投射或 Octomap 投影（`octomap_server` 的 2D 投影）；可结合坡度可通行性。
- 注意：悬空物（桌面）需根据高度范围判定是否影响机身；窄通道与障碍物膨胀；地图分辨率与导航代价地图一致；保存为 `pgm + yaml`（ROS map_server），原点与分辨率元数据正确；动态物体残影要清理。
- 多层建筑：每层一张 2D 地图 + 层间连接信息。
- 工具：`pointcloud_to_laserscan`（把 3D 点云切片转为伪 2D 激光，然后用 2D SLAM/定位）、`octomap_server`、`grid_map`。
- 验证：叠加 2D 地图与实时激光，检查对齐。

**English short answer**
I crop the point cloud to the height range that matters for the robot body, remove ceiling and ground, project the remaining points to an XY grid and mark occupied, free and unknown cells by ray casting or an octomap projection. Overhanging objects are included only if they intersect the body height range, the resolution and origin must match the navigation costmap, and dynamic ghosts should be cleaned. Tools include octomap_server, pointcloud_to_laserscan and grid_map.

（译：把点云裁剪到对机器人本体有意义的高度范围，去除天花板与地面，将剩余点投影到 XY 栅格，并通过射线投射或 octomap 投影标记占据、自由与未知；悬空物仅在与机身高度范围相交时才计入，分辨率与原点须与导航代价地图一致，动态残影要清理；工具有 octomap_server、pointcloud_to_laserscan 和 grid_map。）

**追问方向**：为何要有高度范围；pgm+yaml 地图元数据的含义。

---

## 五、评估、可靠性与工程实践

### Q36 SLAM 常用的公开数据集与基准有哪些？（SLAM datasets and benchmarks）
**优先级：★★** · 对应基础知识：[SLAM问题_前端后端_位姿图优化](../../02_基础知识/07_SLAM与建图/01_SLAM问题_前端后端_位姿图优化.md)

**答案要点**
- 视觉：**TUM RGB-D**（室内 RGB-D，含动态序列）、**EuRoC MAV**（无人机视觉惯性，立体 + IMU，毫米级真值）、**KITTI Odometry**（车载双目/激光，GPS/INS 真值）、**TartanAir**（合成，困难环境）、**TUM-VI**（视觉惯性）。
- 激光：**KITTI**、**NCLT**（长期，校园多会话）、**Newer College**、**HILTI SLAM**、**MulRan**（多雷达回环）、**nuScenes**；室内 **Cartographer 数据**；
- 指标：ATE、RPE、漂移百分比（KITTI：平移 %、旋转 deg/100 m）、实时性与资源占用。
- 使用注意：时间戳对齐、传感器外参、运行多次（随机性）、报告对齐方式；数据集偏向学术场景，自己的场景需自建小数据集与真值（动捕/全站仪/RTK）。
- 面试：能说出 1–2 个数据集的特点与用途。

**English short answer**
Common visual benchmarks are TUM RGB-D, EuRoC for visual-inertial flight and KITTI for driving, while lidar work uses KITTI, NCLT for long-term multi-session data, Newer College, Hilti and MulRan for loop closure. Metrics include ATE, RPE, drift per distance and runtime. I run multiple trials, state the alignment method and, since benchmarks are academic, build a small dataset with ground truth for my own environment.

（译：常见视觉基准有 TUM RGB-D、用于视觉惯性飞行的 EuRoC 和用于驾驶的 KITTI；激光研究用 KITTI、长期多会话的 NCLT、Newer College、Hilti 以及用于回环的 MulRan；指标包括 ATE、RPE、单位距离漂移和运行时间；多次运行并说明对齐方式，且由于基准偏学术，应为自己的环境建立带真值的小数据集。）

**追问方向**：EuRoC 真值的获取方式；KITTI 漂移指标的定义。

### Q37 SLAM 评估的常见陷阱？（Evaluation pitfalls）
**优先级：★★★** · 对应基础知识：[SLAM问题_前端后端_位姿图优化](../../02_基础知识/07_SLAM与建图/01_SLAM问题_前端后端_位姿图优化.md)

**答案要点**
- 时间戳未对齐或存在系统偏移 → ATE 虚高；真值与估计的坐标系/外参不一致（如 GPS 天线与雷达中心的杠杆臂）；
- 对齐方式不同（SE(3) vs Sim(3)，整体对齐 vs 前几帧对齐）导致数值不可比；单目必须 Sim(3)；
- 只报告 ATE 不报告 RPE（全局一致性与局部漂移）；只跑一次就下结论（算法/线程有随机性，ORB-SLAM 因多线程时序而结果不同）；
- 不同运行平台/实时性设定（离线放慢处理 vs 在线实时）；
- 比较时参数调优程度不同；数据集过拟合；忽略失败序列（丢失后的结果被剔除）；
- 规范：说明数据、序列、对齐、运行次数、硬件、是否实时，并报告失败率。
- 工具：evo（对齐、绘图、统计）。

**English short answer**
Common mistakes include misaligned timestamps, mismatched ground-truth frames or lever arms, comparing numbers computed with different alignments, reporting only ATE, running once despite multithreaded nondeterminism, and hiding lost tracks. I state the dataset, alignment, number of runs, hardware and real-time settings, report both ATE and RPE with failure rates, and use a tool such as evo for consistent computation.

（译：常见错误包括时间戳未对齐、真值坐标系或杠杆臂不匹配、比较用不同对齐方式算出的数值、只报告 ATE、忽略多线程非确定性而只跑一次、隐藏跟踪丢失；应说明数据集、对齐方式、运行次数、硬件和实时设定，同时报告 ATE、RPE 与失败率，并用 evo 等工具保证计算一致。）

**追问方向**：杠杆臂补偿；多次运行的统计报告。

### Q38 SLAM 系统的健康监控指标有哪些？（Health monitoring for SLAM）
**优先级：★★★** · 对应基础知识：[数据与日志_可观测性_回放与复盘](../../02_基础知识/13_系统工程与安全/05_数据与日志_可观测性_回放与复盘.md)

**答案要点**
- 前端：特征数/跟踪点数、匹配内点比、重投影或点到面残差均值、扫描匹配得分、帧处理耗时与丢帧、退化指标（Hessian 最小特征值）。
- 估计：位姿协方差迹/最大特征值、位姿跳变（帧间位移/角度异常）、速度/零偏幅值、与里程计/IMU 的一致性（新息 NIS）、尺度因子变化（单目）。
- 后端：优化残差、收敛迭代数、回环接受数与拒绝数、位姿图残差增量（回环加入后残差突增 → 可疑）。
- 系统：CPU/内存、队列延迟、时间同步状态。
- 动作：多级——警告 → 降速 → 停车/重定位；记录日志（便于回放复盘）。
- 实现：发布 `diagnostic_msgs`，规则引擎设阈值与滞后。

**English short answer**
I monitor front-end quality such as tracked features, inlier ratio, residuals and degeneracy metrics, estimator health such as covariance, pose jumps, bias magnitudes and consistency with odometry or IMU, back-end residuals and loop acceptance counts, and system resources and latency. Thresholds with hysteresis drive escalating actions from warnings to slowing and relocalization, and everything is logged so failures can be replayed.

（译：监控前端质量（跟踪特征数、内点比、残差、退化指标）、估计器健康（协方差、位姿跳变、零偏幅值及与里程计/IMU 的一致性）、后端残差和回环接受数，以及系统资源与延迟；带滞后的阈值驱动从警告到降速、重定位的逐级动作，所有信息都记录以便回放失败过程。）

**追问方向**：位姿跳变的阈值如何确定；诊断消息如何接入上层。

### Q39 丢失后的重定位与恢复策略？（Relocalization after loss）
**优先级：★★★** · 对应基础知识：[回环检测与重定位_词袋_ScanContext](../../02_基础知识/07_SLAM与建图/05_回环检测与重定位_词袋_ScanContext.md)

**答案要点**
- 检测丢失：跟踪特征数/内点比低于阈值持续若干帧、扫描匹配得分低、位姿跳变、滤波新息持续超门限。
- 恢复手段：①**短时依赖里程计/IMU 推算**（保持本地连续）；②**视觉**：词袋检索关键帧 → PnP RANSAC；**激光**：全局描述子（Scan Context）+ 配准，或 AMCL 的全局定位（粒子撒满）；③使用先验（上次已知位置、WiFi/UWB/二维码/AprilTag 等路标）缩小搜索范围；④人工干预：在 RViz 中给初始位姿。
- 防止错误重定位：验证（内点数、残差、与里程计一致性、多帧确认）后才接受；接受后平滑过渡（避免位姿跳变影响控制，必要时 map→odom 渐变）。
- 运行时策略：丢失期间降速或停车，防止在错误位姿上继续导航。
- 地图保存/加载与重定位的配合（纯定位模式）。

**English short answer**
I detect loss from a drop in tracked features or match score, pose jumps or persistent filter rejections, bridge briefly with odometry or IMU, then relocalize with bag-of-words plus PnP for vision, global descriptors plus registration for lidar, or global localization with scattered particles, narrowing the search with priors or landmarks. A relocalization is accepted only after verification over several frames, and the pose correction is applied smoothly. While lost, the robot slows or stops.

（译：根据跟踪特征数或匹配得分下降、位姿跳变或滤波持续拒绝来检测丢失，短时依靠里程计或 IMU 桥接，然后视觉用词袋加 PnP、激光用全局描述子加配准或用撒满粒子的全局定位来重定位，并借助先验或路标缩小搜索范围；重定位须经多帧验证后才接受，位姿修正平滑施加；丢失期间机器人减速或停车。）

**追问方向**：如何避免错误重定位；map→odom 渐变修正的实现。

### Q40 长期运行：地图老化与多会话建图？（Long-term and multi-session mapping）
**优先级：★★** · 对应基础知识：[多机SLAM与长期地图维护](../../02_基础知识/07_SLAM与建图/07_多机SLAM与长期地图维护.md)

**答案要点**
- 环境会变（家具移动、门开关、季节、光照）：旧地图与现实不符 → 定位退化。
- 方法：①**多会话建图**：每次运行的数据与已有地图对齐（重定位）后追加新子图/关键帧，通过跨会话回环融入同一位姿图；②**地图维护**：检测变化（新旧扫描差异）、更新或删除过时地标；保留“长期稳定地标”（高观测频次）、给地标时间戳与置信度衰减；③**场景识别**对光照变化鲁棒（学习型描述子）；④地图规模控制（关键帧剔除、体素降采样、分块加载）。
- 工程：保存时带版本、时间与传感器配置；回滚机制；对地图修改后评估定位性能回归。
- 实际：仓储/商场的半静态环境靠定期重建或在线更新。

**English short answer**
Environments change, so priors age. Multi-session mapping relocalizes in the existing map and appends new submaps or keyframes joined through cross-session loop closures into one pose graph. Map maintenance detects changes, updates or removes stale landmarks and decays confidence by age, while condition-robust place descriptors help with lighting changes. I version maps with sensor configuration and re-evaluate localization after any update.

（译：环境会变化，先验会老化；多会话建图先在现有地图中重定位，再追加新子图或关键帧并通过跨会话回环融入同一位姿图；地图维护检测变化、更新或删除过时地标并按时间衰减置信度，对光照鲁棒的场景描述子应对光照变化；地图连同传感器配置一起版本化，更新后重新评估定位性能。）

**追问方向**：变化检测的方法；地图更新的回滚设计。

### Q41 选择 SLAM/定位方案的决策框架？（Choosing a SLAM/localization approach）
**优先级：★★★** · 对应基础知识：[SLAM问题_前端后端_位姿图优化](../../02_基础知识/07_SLAM与建图/01_SLAM问题_前端后端_位姿图优化.md)

**答案要点**
- 需求维度：①环境（室内 / 室外、平面 / 3D、结构化 / 空旷、动态程度）；②传感器与预算（2D 激光、3D 激光、相机、IMU、轮速、GNSS）；③精度与实时性、算力；④是否需要地图复用/重定位/长期运行；⑤功能安全与可维护性。
- 典型选择：室内平面轮式 + 2D 激光 → slam_toolbox / Cartographer 建图 + AMCL 定位（成熟稳定，P5 即此类）；室外/起伏地形 + 3D 激光 + IMU → FAST-LIO2 / LIO-SAM；低成本相机 + IMU → VINS / OpenVINS / ORB-SLAM3；无人机 → VIO；要高精度全局 → 融合 GNSS-RTK。
- 流程：先明确要求 → 选传感器 → 建小数据集评估 2–3 个方案 → 看失败模式与可维护性 → 再做定制。
- 原则：能用成熟方案就不自研；关注失效时的降级与监控。
- 面试表述：先问约束，再给选型理由与风险。

**English short answer**
I start from the environment, sensors and budget, accuracy and compute limits, and the need for map reuse and long-term operation. For an indoor planar wheeled robot with a 2D lidar, building a map with slam_toolbox or Cartographer and localizing with AMCL is mature and stable, outdoors with 3D lidar and IMU I would evaluate FAST-LIO2 or LIO-SAM, and with cameras and an IMU I would try VINS, OpenVINS or ORB-SLAM3. I compare candidates on a small dataset, study failure modes and prefer mature solutions with monitoring and graceful degradation.

（译：从环境、传感器与预算、精度与算力限制以及地图复用和长期运行需求出发；室内平面轮式机器人配 2D 激光，用 slam_toolbox 或 Cartographer 建图并用 AMCL 定位成熟稳定，室外配 3D 激光与 IMU 则评估 FAST-LIO2 或 LIO-SAM，相机加 IMU 则试 VINS、OpenVINS 或 ORB-SLAM3；在小数据集上比较候选方案、研究失败模式，并偏好带监控与优雅降级的成熟方案。）

**追问方向**：如何设计小数据集评估；自研与现成方案的取舍。

### Q42 SLAM 与导航的接口：map→odom 的发布与定位模式？（SLAM-navigation interface）
**优先级：★★★** · 对应基础知识：[导航系统架构_Nav2与move_base对比](../../02_基础知识/08_规划与导航/05_导航系统架构_Nav2与move_base对比.md)

**答案要点**
- 导航栈需要：**地图**（`/map` 的 OccupancyGrid，通常是 transient-local 保证后订阅者收到）、`map→odom` 变换（由 SLAM/定位发布，修正里程计漂移）、`odom→base_link`（里程计，连续）。
- 建图模式（SLAM）：同时发布 `/map` 与 `map→odom`；定位模式（AMCL / slam_toolbox localization）：加载已保存的地图，只发布 `map→odom`。建图完成后用 `map_saver` 保存 `pgm+yaml`（或序列化的位姿图）。
- 定位发布频率与时间戳：`map→odom` 以略超前的时间戳发布（`transform_tolerance`），避免 TF 外推失败；位姿跳变时 `map→odom` 突变，下游（局部规划、控制）应使用 `odom` 系规划以保证连续。
- 切换：建图→导航切换时需要保证地图版本、初始位姿设定（`initialpose`）、坐标系名称一致。
- 调试：`ros2 run tf2_tools view_frames`、RViz 显示粒子云/地图叠加。

**English short answer**
Navigation needs a map topic, the map-to-odom transform published by SLAM or localization to correct odometry drift, and the continuous odom-to-base transform from odometry. In mapping mode the SLAM node publishes both the map and the transform, while in localization mode a saved map is loaded and only the transform is published. Local planning and control use the odom frame to avoid jumps when the map-to-odom correction changes, and I check frame names, map version and initial pose when switching modes.

（译：导航需要地图话题、由 SLAM 或定位发布并用于修正里程计漂移的 map→odom 变换以及里程计提供的连续 odom→base 变换；建图模式下 SLAM 节点同时发布地图与变换，定位模式下加载已保存的地图并只发布变换；局部规划与控制使用 odom 系以避免 map→odom 修正变化时的跳变；切换模式时检查坐标系名称、地图版本和初始位姿。）

**追问方向**：transient-local QoS 的作用；`transform_tolerance` 的含义。

### Q43 SLAM 的仿真与回放测试怎么做？（Testing SLAM with simulation and bags）
**优先级：★★** · 对应基础知识：[测试策略_仿真在环_硬件在环_回归](../../02_基础知识/13_系统工程与安全/04_测试策略_仿真在环_硬件在环_回归.md)

**答案要点**
- **回放（rosbag）**：录制真实传感器数据（含 TF、里程计、IMU），离线重放跑不同算法/参数，保证输入一致；使用 `use_sim_time`，控制回放速度以评估实时性；保存输出轨迹与地图，用 evo 评估。
- **仿真**：Gazebo / Isaac / 自研仿真提供真值（位姿、地图）、可重复场景、故障注入（传感器噪声、丢帧、延迟、打滑、动态物体）；注意仿真传感器噪声模型与真实差异（不能完全替代真机）。
- **回归测试**：把典型难例（走廊、动态、光照）固化为 bag 序列，CI 中自动运行并比较指标阈值，防止修改引入退化。
- **故障注入**：人为丢帧、加时间偏移、改外参，验证监控与降级逻辑。
- 记录：版本、参数、数据哈希，保证可复现。

**English short answer**
Replaying recorded bags gives identical inputs to compare algorithms and parameters, with simulated time and evo scoring. Simulators supply ground truth, repeatable scenes and fault injection such as noise, dropped frames, latency and wheel slip, though their sensor models differ from reality. I freeze difficult sequences into regression tests in CI with metric thresholds, and inject faults to verify monitoring and degradation logic.

（译：回放录制的 bag 为比较算法和参数提供一致输入，配合仿真时间与 evo 评分；仿真器提供真值、可重复场景以及噪声、丢帧、延迟、打滑等故障注入，但其传感器模型与现实有差异；把困难序列固化为 CI 中带指标阈值的回归测试，并注入故障验证监控与降级逻辑。）

**追问方向**：回放时为什么要 `use_sim_time`；如何选取回归序列。

### Q44 SLAM 常见 bug 清单与排查思路？（Common SLAM bugs）
**优先级：★★★** · 对应基础知识：[SLAM问题_前端后端_位姿图优化](../../02_基础知识/07_SLAM与建图/01_SLAM问题_前端后端_位姿图优化.md)

**答案要点**
- 系统性漂移/地图扭曲：外参方向或单位错（度/弧度、mm/m）、IMU 噪声参数过小、时间偏移、轮距/轮径标定错、里程计坐标轴反向。
- 地图“鬼影”重影：运动畸变未补偿、动态物体、回环错误、TF 时间戳不对。
- 回环后地图撕裂：错误回环、优化未固定规范自由度、信息矩阵尺度失衡（旋转与平移权重）。
- 旋转时位置抖动：外参平移（杠杆臂）未设、IMU 与相机时间偏移。
- 突然发散：初始化错误、特征过少、数值（协方差负定）、NaN。
- 排查方法：单模块验证（只看里程计、只看 IMU、只看视觉）；可视化每层中间结果（特征匹配、残差）；对比 ground truth；二分法（关闭某因子/传感器）；检查 TF 与时间戳；用仿真/已知运动验证。
- 习惯：每个传感器先独立验证（IMU 积分、轮速里程计走正方形闭环误差）再融合。

**English short answer**
Systematic drift often comes from wrong extrinsic directions or units, too-small IMU noise, time offsets or wheel calibration; ghosting from motion distortion or dynamic objects; torn maps from false loops or badly scaled information matrices; and sudden divergence from bad initialization or numerical issues. I debug by validating each sensor alone, visualizing intermediate residuals, disabling factors to bisect the cause, checking TF and timestamps, and testing with a known motion.

（译：系统性漂移常源于外参方向或单位错误、IMU 噪声过小、时间偏移或轮子标定；鬼影来自运动畸变或动态物体；地图撕裂来自错误回环或信息矩阵尺度失衡；突然发散来自错误初始化或数值问题；排查时先单独验证每个传感器、可视化中间残差、关闭因子二分定位原因、检查 TF 与时间戳，并用已知运动测试。）

**追问方向**：轮速里程计走方形的闭环验证；信息矩阵尺度失衡的表现。

### Q45 如何向面试官讲述你在定位/建图上的工作（对应 P5）？（Presenting localization and mapping work）
**优先级：★★★**

**答案要点**
- 结构：**问题与约束 → 方案 → 你的贡献 → 验证 → 局限与改进**。
- 事实基线（以 CV 为准）：P5 为车式移动机器人自主导航，涉及 AMCL 定位（粒子滤波）、Hybrid A*、TEB 局部规划、MPC 轨迹跟踪等；讲述中只陈述有依据的事实，具体参数以项目文档的“自查表”为准，不要把本文的通用 SLAM 算法说成项目里实现过的。
- 可以主动展开的深度：AMCL 的原理与失效模式（绑架、对称环境）、里程计误差与融合、TF 结构 `map→odom→base_link`、定位质量监控与恢复；对 SLAM（建图）部分，如实说明“使用现成建图包生成地图，理解其原理（gmapping / Cartographer）”。
- 英文表达要点：简洁、先结论后细节；遇到不熟的追问用诚实模板（“I haven't implemented that directly, but my understanding is…”）。
- 准备清单：能画出系统框图；能讲清一个失败案例及改进；能说出评价指标。

**English short answer**
I would describe the problem and constraints, the approach I used, my specific contribution, how I validated it, and its limitations. I should stay with what I actually did in the project, such as AMCL-based localization and navigation components, and explain general SLAM algorithms as background knowledge rather than claiming I built them. If asked about something I have not implemented, I say so and explain my understanding and how I would verify it.

（译：描述问题与约束、所用方案、我的具体贡献、验证方式及局限；只讲项目里真正做过的内容（如基于 AMCL 的定位与导航组件），把通用 SLAM 算法当作背景知识讲解而非声称自己实现；被问到没做过的内容时如实说明，并讲出我的理解以及会如何验证。）

**追问方向**：一个定位失败案例及改进；如何用英文简洁介绍系统框图。

---

## 六、补充：参数化、求解器、多源融合与误差分析

### Q46 为什么特征点常用逆深度参数化？（Inverse depth parametrization）
**优先级：★★** · 对应基础知识：[视觉惯性里程计VIO_预积分_滑动窗口](../../02_基础知识/07_SLAM与建图/04_视觉惯性里程计VIO_预积分_滑动窗口.md)

**答案要点**
- 用深度 $d$ 参数化路标时，远点 $d\to\infty$，其不确定性分布严重非高斯（长尾），线性化不稳；逆深度 $\rho=1/d$ 的不确定性接近高斯，且 $\rho\to0$ 自然表示无穷远点（无视差的点仍可约束旋转）。
- 参数化方式：一个路标由首次观测的锚点帧位姿 + 方向（单位向量或像素坐标）+ 逆深度表示；重投影时 $\mathbf{p}_j=\mathbf{R}_{ji}\dfrac{\mathbf{f}_i}{\rho}+\mathbf{t}_{ji}$ 再投影。
- 优点：初始化可用未知深度（单目）；远点不发散；变量只有 1 个标量（相对 3 个坐标更少，BA/VIO 状态维度小）。
- VINS-Mono 在滑窗内以逆深度表示特征；也有 InvDepth EKF-SLAM（Civera）。
- 局限：近距、大基线点用欧氏参数也可，要看数值条件。

**English short answer**
Depth for distant points has a heavy-tailed, non-Gaussian uncertainty and can approach infinity, while inverse depth is close to Gaussian and zero naturally represents a point at infinity that still constrains rotation. A landmark is stored as an anchor frame, a bearing and one scalar inverse depth, which also keeps the state dimension small. VINS-Mono uses it for window features, and Euclidean parametrization can be fine for near points.

（译：远点深度的不确定性是长尾的非高斯分布且可趋于无穷，而逆深度近似高斯，零值自然表示仍可约束旋转的无穷远点；路标由锚点帧、方向和一个标量逆深度表示，也使状态维度较小；VINS-Mono 对窗口内特征使用它，近点用欧氏参数化也可以。）

**追问方向**：无视差点为何仍约束旋转；锚点帧的选择。

### Q47 KLT 光流特征管理在 VINS-Mono 中的做法？（Feature tracking in VINS-Mono）
**优先级：★★** · 对应基础知识：[视觉里程计_特征_对极几何_PnP_BA](../../02_基础知识/07_SLAM与建图/02_视觉里程计_特征_对极几何_PnP_BA.md)

**答案要点**
- 每帧：对上一帧特征做**金字塔 KLT 跟踪**；用基础矩阵 RANSAC 剔除外点（反向光流检查可选）；跟踪数不足时用 Shi-Tomasi 在掩膜外补充新角点，保证数量与**均匀分布**（最小间距掩膜 25–30 px）。
- 不计算描述子，计算量远低于 ORB 匹配；每个特征有唯一 ID，贯穿多帧形成特征轨迹；轨迹长度（被观测帧数）用于判断是否进入优化。
- 关键帧判据：与最近关键帧的平均视差超过阈值（如 10 px）或跟踪数量过少则选新关键帧，否则次新帧被边缘化。
- 与特征点法对比：KLT 适合帧间运动小、高帧率（20–30 Hz）；大位移与光照变化时易丢；无描述子故回环需额外的 BRIEF/DBoW2 模块。
- 针对模糊、卷帘快门的鲁棒性：缩短曝光、提高帧率。

**English short answer**
Each frame tracks existing features with pyramidal KLT, rejects outliers with a fundamental-matrix RANSAC and refills corner features under a spacing mask to keep count and uniform distribution. Features carry IDs across frames to form tracks, with no descriptors, which is cheap. Keyframes are chosen by average parallax or tracking failure, and loop closure needs an additional descriptor-based module. KLT suits high frame rates with small inter-frame motion.

（译：每帧用金字塔 KLT 跟踪已有特征，用基础矩阵 RANSAC 剔除外点，并在间距掩膜下补充角点以保持数量与均匀分布；特征带 ID 跨帧形成轨迹且不计算描述子，开销小；按平均视差或跟踪失败选关键帧，回环需要额外的基于描述子的模块；KLT 适合高帧率、帧间运动小的情形。）

**追问方向**：视差阈值的选择；反向光流检验。

### Q48 直接稀疏里程计（DSO）的思想？（Direct Sparse Odometry）
**优先级：★★** · 对应基础知识：[视觉里程计_特征_对极几何_PnP_BA](../../02_基础知识/07_SLAM与建图/02_视觉里程计_特征_对极几何_PnP_BA.md)

**答案要点**
- 完全直接法 + 稀疏点：在梯度较大的像素上选点（均匀分布），最小化窗口内**光度误差**（8 邻域像素块，带仿射亮度参数 $a,b$ 与曝光时间），联合优化窗口内关键帧位姿、点的逆深度和光度参数。
- 滑动窗口（约 7 个关键帧）+ 边缘化；关键帧与点的选取/边缘化策略保证窗口规模。
- 需要**光度标定**（响应函数、暗角、曝光时间）才能发挥最佳性能；全局快门更优；无回环（LDSO 加入）。
- 优势：弱纹理边缘也可用、精度高（在 TUM Mono 基准上）；劣势：对初始化和快速运动敏感、依赖光度一致性、计算量不小。
- 面试：对比特征点法，说明“直接法最小化光度误差，用梯度强的像素，免去匹配”。

**English short answer**
DSO selects pixels with strong gradients spread across the image and minimizes the photometric error of small patches over a sliding window of keyframes, jointly optimizing poses, inverse depths and affine brightness parameters with marginalization. It performs best with photometric calibration and a global shutter and works on weakly textured edges, but is sensitive to initialization, fast motion and brightness changes, and has no loop closure unless extended.

（译：DSO 选取分布在整幅图像上的强梯度像素，在关键帧滑动窗口上最小化小块的光度误差，联合优化位姿、逆深度和仿射亮度参数并做边缘化；配合光度标定与全局快门效果最佳，可用于弱纹理边缘，但对初始化、快速运动与亮度变化敏感，除非扩展否则没有回环。）

**追问方向**：仿射亮度参数的作用；DSO 与 ORB-SLAM 精度对比的看法。

### Q49 GNSS 如何与 SLAM/里程计融合？（Fusing GNSS）
**优先级：★★★** · 对应基础知识：[里程计与传感器融合](../../02_基础知识/06_状态估计/04_里程计与传感器融合.md)

**答案要点**
- GNSS 提供全局无漂移位置（RTK 厘米级、普通米级），但有遮挡、多径、跳变和低频（1–10 Hz）；里程计/SLAM 连续但漂移 → 互补。
- 融合：**松耦合**：把 GNSS 位置作为观测加入 EKF（`robot_localization` 的 `navsat_transform`：把经纬度转到局部 ENU/UTM 并与 odom 对齐）或因子图中的 **GPS 因子**（LIO-SAM）；**紧耦合**：使用伪距/载波相位原始观测（GVINS 类）。
- 要点：坐标系对齐（ENU 与 odom 初始 yaw：需要初始运动估计航向或磁力计/双天线）；杠杆臂（天线与 base_link）；时间同步；GNSS 精度因子（协方差、卫星数、固定解/浮点解状态）用于加权与**异常剔除**（马氏距离门限）；多径区（城市峡谷/室内切换）降权。
- 室内外切换：GNSS 失效时无缝依赖里程计。
- 评价：RTK 固定解作真值。

**English short answer**
GNSS gives drift-free global position but is low rate and suffers occlusion and multipath, while odometry or SLAM is smooth but drifts. In a loosely coupled design I convert fixes to a local ENU frame and add them as measurements to an EKF or as GPS factors in a factor graph, weighted by reported accuracy and gated by Mahalanobis distance. The initial yaw alignment, antenna lever arm and time synchronization are essential, and indoors the system falls back on odometry.

（译：GNSS 提供无漂移的全局位置，但频率低且受遮挡和多径影响，里程计或 SLAM 平滑但漂移；松耦合设计把定位结果转换到局部 ENU 系后作为观测加入 EKF 或因子图中的 GPS 因子，用报告的精度加权并以马氏距离门限剔除异常；初始航向对齐、天线杠杆臂和时间同步至关重要，室内时系统回退到里程计。）

**追问方向**：初始 yaw 对齐的做法；RTK 固定解与浮点解的区别。

### Q50 轮速、IMU 与视觉三者融合的优势？（Wheel-inertial-visual fusion）
**优先级：★★**

**答案要点**
- 轮速里程计：度量尺度、平面运动、低成本；但打滑、悬空、轮径误差导致漂移，并且无法感知垂直/横滚。IMU 与视觉在地面机器人上常遇到**匀速直线 / 平面运动**的退化（尺度不可观、加速度激励不足）。
- 融合（如 VINS-Wheel / 轮速预积分）：用**轮速预积分因子**提供尺度与平面运动约束，帮助 VIO 初始化和匀速直线下保持尺度；同时视觉/IMU 检测轮子打滑；非完整约束（侧向速度 ≈ 0）作为伪量测。
- 外参标定：轮速系（里程计中心）到 IMU 的外参与轮径、轮距的联合标定。
- 注意：打滑 / 悬空时轮速因子要降权（鲁棒核或打滑检测）；不同地面（地毯）的系统性误差。
- 面试：说明“互补退化模式”：轮速解决 VIO 的尺度退化，VIO 解决轮速的打滑。

**English short answer**
Wheel odometry gives metric scale and planar motion cheaply, but slips and cannot see vertical motion, while visual-inertial systems degenerate in constant-velocity planar motion where scale is unobservable. Adding wheel preintegration factors and non-holonomic constraints restores scale there, and visual-inertial data detect slip so the wheel factors can be downweighted. Wheel parameters and the wheel-to-IMU extrinsics need joint calibration.

（译：轮速里程计便宜地提供度量尺度和平面运动，但会打滑且无法感知垂直运动，而视觉惯性系统在匀速平面运动下退化、尺度不可观；加入轮速预积分因子和非完整约束可在此恢复尺度，视觉惯性数据可检测打滑从而对轮速因子降权；轮参数与轮速–IMU 外参需联合标定。）

**追问方向**：轮速预积分的做法；打滑检测方法。

### Q51 路标/标签辅助定位（AprilTag、UWB、反光板）的角色？（Landmark-aided localization）
**优先级：★★**

**答案要点**
- 在 GNSS 缺失的室内，人工路标提供**全局绝对位姿观测**：**AprilTag/二维码**（视觉，位姿 6DoF，近距高精度）、**UWB 基站**（测距，亚米至厘米级，无视线要求但受 NLOS 影响）、**反光板**（激光反射强度，AGV 常用）、**RFID/蓝牙**（粗定位）。
- 融合：把观测作为 EKF/因子图的绝对位姿或测距因子；有已知地图坐标的路标可消除漂移、重定位（kidnap 恢复）。
- 布设：覆盖/密度、避免对称布置造成歧义、标签尺寸与相机分辨率匹配、UWB 基站几何（GDOP）良好。
- 局限：部署与维护成本、环境改变、遮挡/NLOS、多径；标签位姿的标定精度决定地图精度。
- 作用：与自然特征 SLAM 互补，提高可靠性（尤其重复场景、长走廊）。

**English short answer**
Indoors without GNSS, artificial landmarks provide absolute pose or range observations: AprilTags give a six-DoF pose at close range, UWB anchors give ranges without line of sight but with NLOS errors, and reflectors suit lidar-based AGVs. I add them as absolute factors in the estimator, which removes drift and recovers from kidnapping. The costs are installation and maintenance, geometry and calibration of landmark positions, and sensitivity to occlusion and multipath.

（译：室内没有 GNSS 时，人工路标提供绝对位姿或测距观测：AprilTag 在近距给出 6 自由度位姿，UWB 基站无需视线即可测距但存在 NLOS 误差，反光板适合基于激光的 AGV；将其作为绝对因子加入估计器可消除漂移并从绑架中恢复；代价是安装维护、路标位置的几何与标定以及对遮挡和多径的敏感。）

**追问方向**：UWB NLOS 的处理；标签布设的 GDOP 考量。

### Q52 g2o、Ceres、GTSAM 如何选择？（Optimization libraries）
**优先级：★★**

**答案要点**
- **Ceres Solver**：通用非线性最小二乘，自动微分，信赖域（LM/Dogleg），丰富鲁棒核与流形（LocalParameterization/Manifold）；灵活，滑窗 VIO（VINS-Mono）、标定常用；对稀疏图结构需自己定义参数块。
- **g2o**：图优化框架，顶点/边抽象，针对 SLAM 提供 SE(2)/SE(3)、BA 的现成类型，稀疏求解器（CHOLMOD/CSparse）；ORB-SLAM 使用；需要手写雅可比（或数值）。
- **GTSAM**：因子图 + iSAM2 增量求解，贝叶斯树，丰富的预积分、位姿因子；LIO-SAM、各类 VIO 后端使用；C++ 为主，Python 绑定。
- 选择：批量/自定义问题 → Ceres；经典位姿图/BA、追求速度 → g2o；需要增量平滑与协方差恢复 → GTSAM。
- 共同注意：流形参数化、鲁棒核、固定规范自由度、初值质量、求解器选择（稀疏 Cholesky / Schur）。

**English short answer**
Ceres is a flexible general least-squares solver with automatic differentiation, robust losses and manifolds, popular for sliding-window VIO and calibration. g2o is a graph-optimization framework with ready SLAM types and sparse solvers, used by ORB-SLAM. GTSAM offers factor graphs with iSAM2 incremental smoothing and preintegration factors, used in LIO-SAM. I pick by whether I need batch flexibility, speed on classic graphs or incremental solving with marginals.

（译：Ceres 是灵活的通用最小二乘求解器，带自动微分、鲁棒损失与流形，常用于滑窗 VIO 与标定；g2o 是带现成 SLAM 类型和稀疏求解器的图优化框架，ORB-SLAM 使用它；GTSAM 提供因子图与 iSAM2 增量平滑及预积分因子，LIO-SAM 使用它；按需要批量灵活性、经典图上的速度还是带边缘分布的增量求解来选择。）

**追问方向**：自动微分的开销；增量求解为何需要贝叶斯树。

### Q53 左扰动与右扰动模型（李群上的求导）有什么区别？（Left vs right perturbation）
**优先级：★★** · 对应基础知识：[李群李代数与流形优化入门](../../02_基础知识/01_数学基础/06_李群李代数与流形优化入门.md)

**答案要点**
- 在 SO(3)/SE(3) 上更新位姿：右扰动 $\mathbf{R}\leftarrow\mathbf{R}\exp(\delta\boldsymbol\theta^\wedge)$（扰动在**体坐标系**下）与左扰动 $\mathbf{R}\leftarrow\exp(\delta\boldsymbol\theta^\wedge)\mathbf{R}$（扰动在**世界系**下）；两者用伴随互相转换 $\delta\boldsymbol\theta_L=\mathbf{R}\,\delta\boldsymbol\theta_R$。
- 影响：雅可比形式不同（右扰动常见，如 $\partial(\mathbf{R}\mathbf{p})/\partial\delta\boldsymbol\theta=-\mathbf{R}\mathbf{p}^\wedge$；左扰动为 $-(\mathbf{R}\mathbf{p})^\wedge$），状态协方差所在的参考系不同（误差状态定义在体系还是世界系）。
- 一致性：误差状态卡尔曼滤波中，用哪种定义要贯穿状态传播、观测雅可比、重置步骤，混用会引入隐蔽错误；不变 EKF 利用群仿射性质使误差动态与状态无关，改善一致性。
- 实践：选一种并明确文档；VINS 用右扰动，ORB-SLAM g2o 顶点更新采用左乘形式（视实现，未核对）。

**English short answer**
A right perturbation multiplies the update on the body side and a left perturbation on the world side, and they are related by the adjoint, so the same physical error has different coordinates and different Jacobians. In an error-state filter the convention must be applied consistently across propagation, measurement Jacobians and reset, otherwise subtle bugs arise. Invariant EKFs exploit the group structure to improve consistency.

（译：右扰动在体坐标侧相乘、左扰动在世界系侧相乘，二者通过伴随矩阵相互转换，同一物理误差因此有不同的坐标表达和雅可比；在误差状态滤波中必须在传播、观测雅可比和重置过程中一致使用同一约定，否则会出现隐蔽错误；不变 EKF 利用群结构改善一致性。）

**追问方向**：伴随矩阵的作用；不变 EKF 的误差定义。

### Q54 ESKF 中名义状态与误差状态的分工？（Nominal vs error state）
**优先级：★★** · 对应基础知识：[误差状态卡尔曼ESKF与流形上的滤波](../../02_基础知识/06_状态估计/09_误差状态卡尔曼ESKF与流形上的滤波.md)

**答案要点**
- 把真状态分解为**名义状态**（用 IMU 非线性积分高频传播，保持完整精度，姿态可用四元数 / 旋转矩阵）与**误差状态**（小量，用线性化的卡尔曼滤波估计其均值与协方差）：$\mathbf{x}=\mathbf{x}_{nom}\oplus\delta\mathbf{x}$。
- 流程：预测——名义状态积分，误差状态协方差传播 $\mathbf{P}\leftarrow\mathbf{F}\mathbf{P}\mathbf{F}^\top+\mathbf{Q}$；更新——用观测估计 $\delta\hat{\mathbf{x}}$；**注入**（$\mathbf{x}_{nom}\leftarrow\mathbf{x}_{nom}\oplus\delta\hat{\mathbf{x}}$）；**重置**（$\delta\mathbf{x}=0$，并更新协方差 $\mathbf{P}\leftarrow\mathbf{G}\mathbf{P}\mathbf{G}^\top$）。
- 优点：①误差状态小，线性化准确；②姿态误差用 3 维最小表示，避免四元数约束冗余（协方差不奇异）；③高频名义积分与低频更新解耦；④雅可比简单。
- 应用：IMU 姿态估计、GNSS/INS、VIO、LIO（FAST-LIO 的 IESKF）。

**English short answer**
The nominal state is propagated at high rate by nonlinear IMU integration, while a small error state, with a minimal three-dimensional attitude error, is estimated by a Kalman filter linearized around it. After each update the estimated error is injected into the nominal state and reset to zero with a covariance adjustment. This gives more accurate linearization, a non-singular covariance for attitude and a clean separation between high-rate propagation and low-rate updates.

（译：名义状态由非线性 IMU 积分高频传播，在其附近线性化，由卡尔曼滤波估计带三维最小姿态误差表示的小误差状态；每次更新后将估计误差注入名义状态并清零同时调整协方差；这使线性化更准确、姿态协方差非奇异，并清晰分离高频传播与低频更新。）

**追问方向**：重置步骤中的雅可比 $\mathbf{G}$；为什么四元数协方差会奇异。

### Q55 里程计漂移量级怎么估算？（Drift estimation）
**优先级：★★** · 对应基础知识：[里程计与传感器融合](../../02_基础知识/06_状态估计/04_里程计与传感器融合.md)

**答案要点**
- 经验量级（教学示例）：轮速里程计漂移约 1%–5% 行驶距离（受打滑/轮径影响）；IMU 单独积分：位置误差 $\approx\frac12b_at^2$（零偏 $b_a=0.01\,\mathrm{m/s^2}$，60 s 后约 18 m）；陀螺零偏 $b_g$ 使角度误差 $\approx b_gt$（$b_g=0.01\,\mathrm{rad/s}$ → 60 s 后 0.6 rad ≈ 34°）；视觉里程计/激光里程计相对漂移约 0.1%–1% 距离（视场景）。
- 噪声引起的随机游走：陀螺角度随机游走 $\sigma_\theta(t)=\mathrm{ARW}\sqrt{t}$（Allan 方差的 ARW 系数）；加速度计速度随机游走。
- 用途：判断传感器选型是否满足任务（如需要 10 分钟内位置误差 <1 m 时 IMU 不可能单独满足）；设置融合权重与重定位周期。
- 误差来源：尺度、航向误差对位置的累积影响最大（航向误差 1° 走 100 m 产生约 1.7 m 横向误差）。

**English short answer**
A constant accelerometer bias grows position error as half bias times time squared, so a bias of 0.01 m/s squared gives about 18 m after a minute, and a gyro bias grows attitude error linearly. Wheel odometry typically drifts a few percent of distance, and visual or lidar odometry a fraction of a percent. Heading error dominates position drift: one degree over 100 m is about 1.7 m laterally. These estimates guide sensor selection and how often absolute corrections are needed.

（译：加速度计常值零偏使位置误差按 $\frac12 b t^2$ 增长，0.01 m/s² 的零偏一分钟后约 18 m，陀螺零偏使姿态误差线性增长；轮速里程计漂移通常为行驶距离的百分之几，视觉或激光里程计为百分之几以内的零点几；航向误差对位置漂移影响最大：100 m 内 1° 约造成 1.7 m 横向误差；这些估算指导传感器选型和绝对修正的频率。）

**追问方向**：ARW 单位换算；航向误差为何占主导。

### Q56 回环检测的精确率–召回率曲线怎么用？（Loop closure PR evaluation）
**优先级：★★** · 对应基础知识：[回环检测与重定位_词袋_ScanContext](../../02_基础知识/07_SLAM与建图/05_回环检测与重定位_词袋_ScanContext.md)

**答案要点**
- 对每个查询，检索器给出最相似候选及得分；真值回环由位姿距离（如 <10 m）定义。随阈值变化得到精确率–召回率（PR）曲线；SLAM 对精确率要求极高（≈100%），因为一个错误回环可能毁掉地图。
- 选择工作点：选择“最大召回率且精确率=100%”的阈值（如 ORB-SLAM / DBoW2 的评测）；叠加时间一致性与几何验证可大幅提升精确率而降低一点召回。
- 指标：AUC、最大召回@100%精确率、Recall@K；查询耗时与内存。
- 数据集：NewCollege / CityCentre、KITTI、Oxford RobotCar（季节变化）。
- 难点：视角变化、光照/季节变化、动态物体、感知混淆；反向回环（反向行驶）对描述子的方向性要求。

**English short answer**
I label true loops by pose distance, sweep the score threshold to get a precision-recall curve and pick the operating point with the highest recall at 100 percent precision, since one false loop can ruin a map. Temporal consistency and geometric verification raise precision at a small cost in recall. I also report recall at 100 percent precision, runtime and memory, and test under viewpoint, season and lighting changes and reverse traversal.

（译：以位姿距离标注真实回环，扫描得分阈值得到精确率–召回率曲线，选择精确率为 100% 时召回率最高的工作点，因为一个错误回环就可能毁掉地图；时间一致性和几何验证以少量召回为代价显著提高精确率；同时报告 100% 精确率下的召回率、运行时间与内存，并在视角、季节、光照变化及反向通行下测试。）

**追问方向**：为什么 SLAM 重视精确率；反向回环的难点。

### Q57 地图表示怎么选：稀疏特征、稠密点云、体素、栅格？（Choosing map representations）
**优先级：★★★** · 对应基础知识：[建图表示_栅格_八叉树_NDT_TSDF_ESDF](../../02_基础知识/07_SLAM与建图/06_建图表示_栅格_八叉树_NDT_TSDF_ESDF.md)

**答案要点**
- **稀疏特征地图**（ORB-SLAM）：只存路标 + 描述子，小而适合定位/重定位，不能直接用于避障；**稠密点云**：几何完整，大、查询需空间索引，适合激光定位（NDT）与可视化；**体素/ESDF/TSDF**：适合避障与规划（距离场），增量融合；**2D 占据栅格**：平面导航标准，小而快；**拓扑/语义地图**：高层规划、任务级（房间、物体）；**混合**：定位用稀疏/点云，导航用栅格/ESDF，任务用语义。
- 选择依据：用途（定位/避障/规划/交互）、传感器、内存与计算、更新频率、维度（2D/2.5D/3D）。
- 多层表示：同一 SLAM 输出多种表示（点云 → 栅格 → ESDF），保持一致坐标系与版本。
- 例：仓储 AGV：2D 栅格 + 反光板；无人机：ESDF + 稀疏 VIO 特征。

**English short answer**
Sparse landmark maps are small and good for localization but unusable for obstacle avoidance, dense point clouds suit lidar localization, TSDF and ESDF volumes support avoidance and planning, 2D occupancy grids are the standard for planar navigation, and topological or semantic maps support task-level planning. I choose by purpose, sensors, memory and update rate, and often keep several representations derived consistently from the same SLAM output.

（译：稀疏路标地图小而适合定位但不能用于避障，稠密点云适合激光定位，TSDF/ESDF 体积支持避障与规划，2D 占据栅格是平面导航的标准，拓扑或语义地图支持任务级规划；按用途、传感器、内存和更新率选择，并常保留由同一 SLAM 输出一致导出的多种表示。）

**追问方向**：多层地图的一致性维护；ESDF 构建的代价。

### Q58 双目/RGB-D SLAM 相对单目的优势与注意点？（Stereo and RGB-D SLAM）
**优先级：★★** · 对应基础知识：[立体视觉_对极几何_深度估计](../../02_基础知识/09_感知与视觉/03_立体视觉_对极几何_深度估计.md)

**答案要点**
- 优势：**度量尺度**直接可得、无需运动初始化、单帧即可三角化地图点、纯旋转/静止也可跟踪、更鲁棒；RGB-D 提供稠密深度可直接建稠密地图。
- 注意：①双目基线决定有效深度范围（远点退化为单目约束，ORB-SLAM 区分“近点”和“远点”处理）；②RGB-D 深度范围短、室内为主、强光失效、深度噪声随距离增大；③双目标定与同步质量直接影响精度；④计算量略大（双目匹配）。
- 实务：双目 + IMU 是无人机/移动机器人常见组合；轮式机器人在走廊等退化场景需要补充；深度相机适合室内操作场景。
- 评估：KITTI（双目）、TUM RGB-D、EuRoC（双目 + IMU）。

**English short answer**
Stereo and RGB-D systems give metric scale directly, triangulate points from a single frame without motion for initialization and keep tracking under pure rotation or standstill, which makes them more robust than monocular ones. The stereo baseline limits the useful depth range, RGB-D fails outdoors in sunlight and has short range, and calibration and synchronization quality govern accuracy. Stereo plus IMU is a common robust configuration.

（译：双目和 RGB-D 系统直接给出度量尺度，初始化无需运动、单帧即可三角化点，纯旋转或静止时仍能跟踪，因而比单目更鲁棒；双目基线限制有效深度范围，RGB-D 量程短且在室外阳光下失效，标定与同步质量决定精度；双目加 IMU 是常见的鲁棒配置。）

**追问方向**：近点与远点的处理区别；RGB-D 噪声模型。

### Q59 全局定位的初始位姿设置与 AMCL 初始化策略？（Initial pose and global localization）
**优先级：★★★** · 对应基础知识：[粒子滤波与AMCL](../../02_基础知识/06_状态估计/05_粒子滤波与AMCL.md)

**答案要点**
- 已知大致初始位置：在 RViz 用 `2D Pose Estimate` 或程序发布 `initialpose`（带协方差），粒子在该高斯附近播撒；位姿太偏或方向反会导致无法收敛。
- 未知初始位置：**全局定位**——粒子均匀撒满地图自由空间（AMCL `global_localization` 服务），随机运动让粒子收敛；收敛判据：粒子协方差小于阈值、`amcl_pose` 协方差下降。环境对称（重复走廊）时可能收敛到错误位置 → 需要额外标志（AprilTag）、更长运动、多假设监控。
- 辅助：上次关机位姿、WiFi/UWB/二维码、GPS 粗定位。
- 防护：启动后先小范围旋转或前进，确认定位收敛后再允许导航；监控粒子协方差与匹配得分，发散则停车。
- 动态地图/大地图：粒子数需更大（KLD 自适应）。

**English short answer**
With a roughly known start I publish an initial pose with covariance so particles are sampled around it, and a wrong heading prevents convergence. With an unknown start I trigger global localization to spread particles over free space and move the robot until the covariance shrinks, though symmetric environments can converge to the wrong place and need extra cues. I gate navigation on localization convergence and monitor match score and covariance.

（译：大致已知起点时发布带协方差的初始位姿使粒子在其附近采样，航向错误会阻碍收敛；起点未知时触发全局定位把粒子撒满自由空间并移动机器人直到协方差收敛，但对称环境可能收敛到错误位置，需要额外线索；以定位收敛作为允许导航的条件，并监控匹配得分与协方差。）

**追问方向**：对称环境的消歧；粒子协方差的收敛判据。

### Q60 轨迹平滑与离线精化（RTS smoother、全局 BA）的价值？（Smoothing and offline refinement）
**优先级：★★**

**答案要点**
- 在线滤波只利用过去数据，估计存在延迟与噪声；**平滑**同时利用未来数据，精度更高：**RTS 平滑器**对卡尔曼滤波结果做反向递推，得到最优固定区间平滑估计；位姿图/BA 的全局优化本身就是平滑。
- 用途：离线建图（高质量地图用于后续定位）、数据集真值生成、标定后精化、轨迹后处理（航迹重建）。
- 流程：在线运行得到初始轨迹与地图 → 离线全局 BA/位姿图优化（加入所有回环）→ 剔除不稳定点/关键帧 → 导出地图。
- 注意：离线优化后地图与在线运行时的轨迹差异，下游（定位）使用优化后的地图要保持坐标系一致。
- 对实时系统：固定滞后平滑（fixed-lag smoother）用于需要较低延迟但略高精度的场景。

**English short answer**
Online filters use only past data, whereas smoothing also uses future measurements. The Rauch-Tung-Striebel smoother runs a backward pass over Kalman filter results, and global bundle adjustment or pose graph optimization is smoothing in batch form. I use offline refinement to build high-quality maps and ground truth, and fixed-lag smoothing when slightly delayed but more accurate estimates are acceptable online.

（译：在线滤波只用过去数据，而平滑同时利用未来测量；RTS 平滑器对卡尔曼滤波结果做反向递推，全局 BA 或位姿图优化是批处理形式的平滑；离线精化用于构建高质量地图和真值，在线可接受略有延迟但更准的估计时用固定滞后平滑。）

**追问方向**：RTS 递推公式；固定滞后平滑的窗口。

### Q61 VIO/SLAM 的关键帧选择策略比较？（Keyframe selection policies）
**优先级：★★**

**答案要点**
- 常见准则：①**视差/位移**（与最近关键帧的平均视差或平移超过阈值）；②**跟踪质量**（共视点比例低于阈值，如 <90%/<75%）；③**时间间隔**（上限，防止长时间无关键帧）；④**旋转角**；⑤局部建图是否空闲（ORB-SLAM）；⑥信息增益 / 可观性（高级方法）。
- 窗口管理（VIO）：新帧为关键帧→边缘化最老；非关键帧→丢弃次新帧，保留 IMU 预积分（VINS）。
- 太密：冗余、算力浪费；太稀：跟踪丢失、匹配基线不足、IMU 预积分过长（零偏误差累积）。
- 激光 SLAM：按位移（如 1 m）或旋转（如 10°）选关键帧；回环用关键帧子图。
- 调参依据：运动速度、帧率、场景深度；用跟踪丢失率与轨迹精度评估。

**English short answer**
Keyframes are chosen when parallax or translation since the last keyframe exceeds a threshold, when the fraction of tracked points drops, when rotation is large, or when too much time passes. Too many keyframes waste compute, too few cause tracking loss, poor baselines and long IMU preintegration. In sliding-window VIO a new keyframe marginalizes the oldest frame while a non-keyframe replaces the second newest. For lidar I use distance and rotation thresholds.

（译：当自上一关键帧以来的视差或位移超过阈值、跟踪点比例下降、旋转较大或时间过长时选取关键帧；关键帧太多浪费算力，太少则导致跟踪丢失、基线不足和 IMU 预积分过长；滑窗 VIO 中新关键帧边缘化最老帧，非关键帧则替换次新帧；激光 SLAM 用距离和旋转阈值。）

**追问方向**：为什么非关键帧要保留 IMU 数据；阈值如何调参。

---

## 本章要点速记
1. SLAM 是 MAP 估计：全 SLAM 保留整条轨迹（优化），在线 SLAM 边缘化旧位姿（滤波）；EKF-SLAM $O(N^2)$ 且易不一致。
2. 位姿图残差在李代数上定义，Hessian 稀疏；边缘化用舒尔补但引入填充并固定线性化点；iSAM2 增量求解。
3. 错误回环要靠前端验证 + 鲁棒核/切换约束/一致性选择，精确率优先。
4. ORB-SLAM：跟踪 / 局部建图 / 回环三线程；局部 BA 固定外围关键帧；单目回环用 Sim(3)。
5. VIO：预积分 + 滑窗 + 边缘化，或 MSCKF 滤波；4 自由度不可观（位置 + yaw）；初始化需要激励。
6. 激光 SLAM：Cartographer（子图 + 分支定界回环）、LIO-SAM（因子图）、FAST-LIO2（IESKF + ikd-tree）；退化用 Hessian 特征值检测并只更新可观方向。
7. 评估：ATE/RPE、对齐方式、多次运行、失败率；监控特征数、残差、位姿跳变、退化指标。
8. 与导航接口：`map→odom` 由定位发布，局部规划用 odom；丢失时降速/停车 + 重定位并验证。

## 自测题
1. 写出位姿图边的残差，并说明 Hessian 的稀疏结构与规范自由度。
2. 为什么边缘化会让 Hessian 变稠密？FEJ 在其中起什么作用？
3. 解释 IMU 预积分的好处，并写出位置约束的表达式。
4. VIO 为什么有 4 个不可观自由度？匀速直线运动会带来什么额外问题？
5. 比较 Cartographer、LIO-SAM、FAST-LIO2 的架构与适用场景。
6. 设计一套 SLAM 健康监控与丢失恢复策略，并说明如何避免错误重定位。

## 相关链接
- 同模块：[状态估计与SLAM八股](../03_机器人学/03_状态估计与SLAM八股.md) · [路径规划与导航八股](../03_机器人学/04_路径规划与导航八股.md)
- 基础知识：[SLAM 问题与位姿图优化](../../02_基础知识/07_SLAM与建图/01_SLAM问题_前端后端_位姿图优化.md) · [视觉惯性里程计](../../02_基础知识/07_SLAM与建图/04_视觉惯性里程计VIO_预积分_滑动窗口.md) · [回环检测与重定位](../../02_基础知识/07_SLAM与建图/05_回环检测与重定位_词袋_ScanContext.md)
- 感知：[点云、3D 感知与传感器融合八股](../05_计算机视觉与3D感知/03_点云_3D感知与传感器融合八股.md)
- 八股总入口：[README](../README.md)
