# 经典机器学习补充：SVM、树模型、聚类与降维

> **一句话定位**：[机器学习基础](./01_机器学习基础.md) 讲了线性回归、逻辑回归、正则化和评价指标。本章补上面试里仍然高频的"经典四件套"：支持向量机（SVM）与核技巧、决策树与集成（随机森林、梯度提升）、无监督聚类（K-means、高斯混合模型 GMM）、降维（PCA、t-SNE）。深度学习时代它们没有消失：小样本、表格数据、可解释性、数据预处理和机器人里的几何拟合仍然靠它们。

**本章学完你能：**

- 推导硬间隔 SVM 的优化问题，手算一个二维数据集的最大间隔超平面和支持向量，解释对偶问题与核技巧；
- 区分硬间隔、软间隔（参数 $C$）、铰链损失，说明 $C$ 与 RBF 核宽度 $\gamma$ 的作用；
- 手算基尼不纯度（Gini）、信息熵与信息增益，选出最佳分裂点；
- 解释 bagging 与 boosting 的偏差—方差分工，手算一轮梯度提升（残差拟合）；
- 手算 K-means 的完整迭代，指出它是 GMM 的"硬分配、等方差"特例，手算一次 GMM 的 E 步与 M 步；
- 手算 PCA（协方差、特征分解、投影、解释方差比），说明它与 SVD 的关系；
- 用英文回答 "Explain SVM / How does a random forest work / Gradient boosting vs random forest / K-means vs GMM / PCA vs t-SNE" 及追问；
- 诚实说明这些方法与七个项目的联系（多数是延伸，不是项目所做）。

**前置知识**：[机器学习基础](./01_机器学习基础.md)、[线性代数](../01_数学基础/01_线性代数.md)、[概率统计与高斯分布](../01_数学基础/03_概率统计与高斯分布.md)、[最优化与二次规划](../01_数学基础/04_最优化与二次规划.md)。

## 目录

1. 为什么还要学经典机器学习
2. SVM：最大间隔分类器
3. SVM：对偶问题、软间隔与核技巧
4. 决策树：分裂准则与过拟合
5. 集成学习：bagging、随机森林与梯度提升
6. K-means 与 GMM：聚类
7. PCA 与 t-SNE：降维
8. 方法选择：什么时候用什么
9. 在七个项目里怎么用（含诚实边界）
10. 常见错误与故障现象
11. 面试怎么问、怎么答
12. 本章要点速记
13. 自测题
14. 相关链接

---

## 1. 为什么还要学经典机器学习

面试官问"你熟悉哪些机器学习算法"，很少只想听到"CNN 和 Transformer"。他们想判断你是否理解学习问题的骨架：模型的假设是什么、优化的目标是什么、什么时候会过拟合、数据变多或变少时选什么。经典方法的好处是每一个都能用纸笔算清楚，这正是面试里"请手算一下"的最佳素材。

另外有三个实际理由：

1. **表格数据**（传感器统计量、日志特征）上，梯度提升树常常比神经网络更稳、更省数据。
2. **几何拟合**：点云平面拟合用 PCA 求法向，聚类用 K-means 或 DBSCAN 分割障碍物，这些在 [点云处理](../09_感知与视觉/02_点云处理_滤波_配准_ICP.md) 里出现过。
3. **理解深度学习的前身**：SVM 的铰链损失与最大间隔思想，是后来"对比学习""间隔损失"的源头；PCA 是自编码器的线性特例；GMM 的 EM 与 VAE、扩散模型里的隐变量思想相通。

> **提示**：本章所有"手算"数值均用 Python（numpy）复核，文中标"Python 复核"。教学数据标"示例（非项目实测）"。

## 2. SVM：最大间隔分类器

### 2.1 它要解决什么问题

二分类里，线性可分数据可以有无数条分隔直线。哪一条最好？SVM 的回答：**离两类最近的点都尽量远**的那条，即最大化间隔（margin）。直觉是：新样本落在训练点附近时，间隔大的边界更不容易判错，泛化更稳。

### 2.2 符号与单位

| 符号 | 含义 |
|---|---|
| $\mathbf x_i\in\mathbb R^d$ | 第 $i$ 个样本的特征向量 |
| $y_i\in\{-1,+1\}$ | 类别标签 |
| $\mathbf w\in\mathbb R^d,\ b$ | 超平面 $\mathbf w^\top\mathbf x+b=0$ 的法向量与偏置 |
| $\gamma_i=y_i(\mathbf w^\top\mathbf x_i+b)$ | 函数间隔（未除以 $\lVert\mathbf w\rVert$） |
| $\frac{y_i(\mathbf w^\top\mathbf x_i+b)}{\lVert\mathbf w\rVert}$ | 几何间隔，点到超平面的有符号距离，单位与特征相同 |
| $\alpha_i\ge0$ | 拉格朗日乘子（对偶变量） |
| $C>0$ | 软间隔惩罚系数 |
| $\xi_i\ge0$ | 松弛变量 |

### 2.3 推导：从几何间隔到二次规划

点 $\mathbf x_i$ 到超平面的距离是 $y_i(\mathbf w^\top\mathbf x_i+b)/\lVert\mathbf w\rVert$（正确分类时为正）。我们要最大化最近点的距离：

$$\max_{\mathbf w,b}\ \min_i \frac{y_i(\mathbf w^\top\mathbf x_i+b)}{\lVert\mathbf w\rVert}.$$

超平面 $(\mathbf w,b)$ 乘以任意正数 $k$ 仍是同一个平面，所以我们可以**规定**最近点满足 $y_i(\mathbf w^\top\mathbf x_i+b)=1$（这是一个归一化约定，不是附加假设）。这样最近点的距离就是 $1/\lVert\mathbf w\rVert$，两侧间隔带的总宽度为 $2/\lVert\mathbf w\rVert$。最大化它等价于最小化 $\lVert\mathbf w\rVert^2/2$，约束是所有点都满足函数间隔至少为 1：

$$\min_{\mathbf w,b}\ \tfrac12\lVert\mathbf w\rVert^2\quad\text{s.t.}\quad y_i(\mathbf w^\top\mathbf x_i+b)\ge1,\ i=1,\dots,N.$$

这是凸二次规划（目标严格凸，约束线性），有唯一全局解，正是 [最优化与二次规划](../01_数学基础/04_最优化与二次规划.md) 里讲过的那一类。

### 2.4 完整手算（示例，非项目实测）

四个二维点：负类 $\mathbf x_1=(1,1)$、$\mathbf x_2=(2,1)$；正类 $\mathbf x_3=(3,3)$、$\mathbf x_4=(4,3)$。

**步骤 1：找最近的异类点对。** 负类 $(2,1)$ 与正类 $(3,3)$ 的距离为 $\sqrt{1+4}=\sqrt5\approx2.236$，其余异类点对更远（例如 $(1,1)$ 到 $(3,3)$ 是 $2\sqrt2\approx2.83$）。直觉上最大间隔超平面垂直平分这对点的连线。这两个点就是支持向量（support vector）。

**步骤 2：写出两个支持向量满足的等式。** 令 $\mathbf w=(w_1,w_2)$。支持向量处函数间隔为 1：

$$w_1\cdot3+w_2\cdot3+b=+1,\qquad w_1\cdot2+w_2\cdot1+b=-1.$$

法向量应平行于差向量 $\mathbf d=(3,3)-(2,1)=(1,2)$，设 $\mathbf w=k(1,2)$。两式相减：$\mathbf w^\top\mathbf d=2$，即 $k(1+4)=2$，$k=0.4$。于是 $\mathbf w=(0.4,0.8)$。

**步骤 3：求 $b$。** 由负支持向量：$0.4\cdot2+0.8\cdot1+b=-1\Rightarrow b=-1-1.6=-2.6$。

**步骤 4：验证全部约束。**

| 点 | $\mathbf w^\top\mathbf x+b$ | $y\cdot$ 值 | 是否 $\ge1$ |
|---|---|---|---|
| $(1,1)$ | $0.4+0.8-2.6=-1.4$ | $1.4$ | 是 |
| $(2,1)$ | $0.8+0.8-2.6=-1.0$ | $1.0$ | 是（支持向量） |
| $(3,3)$ | $1.2+2.4-2.6=+1.0$ | $1.0$ | 是（支持向量） |
| $(4,3)$ | $1.6+2.4-2.6=+1.4$ | $1.4$ | 是 |

**步骤 5：间隔宽度。** $\lVert\mathbf w\rVert=\sqrt{0.16+0.64}=\sqrt{0.8}=0.8944$，间隔宽度 $2/0.8944=2.236=\sqrt5$，与支持向量间距离一致。Python 复核：对方向做 36000 点网格搜索，得到最大间隔 2.2361，方向 $(0.447,0.894)$，$\mathbf w=(0.4,0.8)$，$b=-2.6$。

> **面试要点**：只有支持向量决定超平面；删除非支持向量 $(1,1)$、$(4,3)$ 解不变。这是 SVM"稀疏性"的来源。

## 3. SVM：对偶问题、软间隔与核技巧

### 3.1 对偶问题

写拉格朗日函数

$$L(\mathbf w,b,\boldsymbol\alpha)=\tfrac12\lVert\mathbf w\rVert^2-\sum_i\alpha_i\big[y_i(\mathbf w^\top\mathbf x_i+b)-1\big],\quad\alpha_i\ge0.$$

对 $\mathbf w$ 与 $b$ 求偏导并令其为零：

$$\frac{\partial L}{\partial\mathbf w}=\mathbf w-\sum_i\alpha_iy_i\mathbf x_i=0\ \Rightarrow\ \mathbf w=\sum_i\alpha_iy_i\mathbf x_i,\qquad \frac{\partial L}{\partial b}=-\sum_i\alpha_iy_i=0.$$

代回得对偶问题：

$$\max_{\boldsymbol\alpha}\ \sum_i\alpha_i-\tfrac12\sum_{i,j}\alpha_i\alpha_jy_iy_j\,\mathbf x_i^\top\mathbf x_j\quad\text{s.t.}\ \alpha_i\ge0,\ \sum_i\alpha_iy_i=0.$$

KKT 互补松弛条件 $\alpha_i[y_i(\mathbf w^\top\mathbf x_i+b)-1]=0$ 说明：只有函数间隔恰为 1 的点（支持向量）才有 $\alpha_i>0$。

**回到 2.4 的例子验证**：支持向量为 $\mathbf x_2$（$y=-1$）与 $\mathbf x_3$（$y=+1$），其余 $\alpha=0$。约束 $\sum\alpha_iy_i=0$ 给 $\alpha_3=\alpha_2=\alpha$。$\mathbf w=\alpha(3,3)-\alpha(2,1)=\alpha(1,2)$，与 $\mathbf w=(0.4,0.8)$ 比较得 $\alpha=0.4$。对偶目标值 $=\sum\alpha-\tfrac12\lVert\mathbf w\rVert^2=0.8-0.4=0.4$，等于原问题目标 $\tfrac12\lVert\mathbf w\rVert^2=0.4$（强对偶成立）。Python 复核：一致。

对偶的价值在于：数据只以内积 $\mathbf x_i^\top\mathbf x_j$ 出现，这为核技巧开了门。预测函数也只含内积：$f(\mathbf x)=\sum_i\alpha_iy_i\,\mathbf x_i^\top\mathbf x+b$。

### 3.2 软间隔与铰链损失

真实数据常常线性不可分或带噪声。引入松弛变量 $\xi_i\ge0$，允许部分点越过间隔：

$$\min_{\mathbf w,b,\boldsymbol\xi}\ \tfrac12\lVert\mathbf w\rVert^2+C\sum_i\xi_i\quad\text{s.t.}\ y_i(\mathbf w^\top\mathbf x_i+b)\ge1-\xi_i,\ \xi_i\ge0.$$

消去 $\xi_i$ 得到 $\xi_i=\max(0,\,1-y_if(\mathbf x_i))$，即**铰链损失**（hinge loss）。于是软间隔 SVM 等价于"铰链损失 + L2 正则"：

$$\min\ \tfrac12\lVert\mathbf w\rVert^2+C\sum_i\max\big(0,1-y_i(\mathbf w^\top\mathbf x_i+b)\big).$$

对偶问题唯一变化是 $0\le\alpha_i\le C$。$C$ 的作用：

| $C$ 大 | $C$ 小 |
|---|---|
| 重罚违反间隔，间隔窄，贴合训练集 | 容忍误分类，间隔宽，更平滑 |
| 偏差低、方差高，易过拟合 | 偏差高、方差低，可能欠拟合 |

与 [机器学习基础](./01_机器学习基础.md) 的正则化对照：$C$ 相当于 $1/\lambda$。

### 3.3 核技巧

线性不可分的数据（如一维上正类在中间、负类在两侧）可先映射到高维 $\phi(\mathbf x)$，再在高维线性分类。如果算法只依赖内积，那么只需要**核函数** $k(\mathbf x,\mathbf z)=\phi(\mathbf x)^\top\phi(\mathbf z)$，不必显式计算 $\phi$。

**手算：二次多项式核等于显式特征映射。** 取 $\mathbf x=(1,2)$、$\mathbf z=(3,1)$，核 $k(\mathbf x,\mathbf z)=(\mathbf x^\top\mathbf z)^2$。

- 直接算：$\mathbf x^\top\mathbf z=3+2=5$，$k=25$。
- 显式映射 $\phi(\mathbf x)=(x_1^2,\sqrt2x_1x_2,x_2^2)$：$\phi(\mathbf x)=(1,2\sqrt2,4)$，$\phi(\mathbf z)=(9,3\sqrt2,1)$，内积 $=9+12+4=25$。一致（Python 复核：25.0 与 25.0）。

计算核只需 $O(d)$，而显式映射维度是 $O(d^2)$。

**RBF（高斯）核**：$k(\mathbf x,\mathbf z)=\exp\!\big(-\lVert\mathbf x-\mathbf z\rVert^2/(2\sigma^2)\big)$，对应无穷维特征空间。同样取上面两点，$\lVert\mathbf x-\mathbf z\rVert^2=4+1=5$：

- $\sigma=1$：$k=e^{-2.5}=0.0821$；
- $\sigma=2$：$k=e^{-5/8}=0.5353$（Python 复核）。

$\sigma$ 小则每个点只影响很小邻域，决策边界蜿蜒，易过拟合；$\sigma$ 大则接近线性。库里常写 $\gamma=1/(2\sigma^2)$，与前面的"间隔 $\gamma_i$"同名不同义，注意区分。

核函数必须是正半定的（Mercer 条件）：任意点集的核矩阵（Gram 矩阵）要对称半正定。判断一个"相似度"能否当核，就看 Gram 矩阵特征值是否非负。

### 3.4 复杂度与实践

训练核 SVM 需要处理 $N\times N$ 核矩阵，典型实现的时间在 $O(N^2)$ 到 $O(N^3)$ 之间（视求解器而定），样本数过十万时开始吃力；线性 SVM 可以用随机梯度下降，复杂度 $O(Nd)$。特征需要标准化，否则量纲大的特征主导距离。多分类用 one-vs-rest 或 one-vs-one 组合二分类器。

## 4. 决策树：分裂准则与过拟合

### 4.1 它解决什么问题

决策树把特征空间切成矩形区域，每个叶子给一个预测（分类取多数类，回归取均值）。它不要求特征缩放，能处理类别特征和缺失值（视实现而定），结果可解释为一串"if–else"规则。

### 4.2 不纯度度量

设某结点样本中第 $k$ 类比例为 $p_k$：

$$\text{Gini}=1-\sum_kp_k^2,\qquad H=-\sum_kp_k\log_2p_k.$$

分裂后的加权不纯度 $\sum_{c\in\{L,R\}}\frac{N_c}{N}\text{Gini}_c$；信息增益 $=H(\text{父})-\sum_c\frac{N_c}{N}H_c$。回归树用方差（平方误差）。

### 4.3 完整手算（示例，非项目实测）

10 个样本，单一特征取值 1 到 10，标签依次为 $0,0,0,1,0,1,1,1,1,1$（前半段混入了一个 1，所以不能被一刀切干净）。统计：标签 1 共 6 个、0 共 4 个。

- 根结点：$p_1=0.6$，$\text{Gini}=1-0.36-0.16=0.48$（Python 复核 0.48）。
- 候选阈值 3.5：左 $\{1,2,3\}$ 全为 0，$\text{Gini}_L=0$；右 7 个样本里 1 有 6 个，$p_1=6/7$，$\text{Gini}_R=1-(6/7)^2-(1/7)^2=0.2449$；加权 $=0.3\times0+0.7\times0.2449=0.1714$。
- 候选阈值 4.5：左 $\{0,0,0,1\}$，$\text{Gini}_L=1-0.75^2-0.25^2=0.375$；右 $\{0,1,1,1,1,1\}$，$\text{Gini}_R=0.2778$；加权 $=0.4\times0.375+0.6\times0.2778=0.3167$。
- 候选阈值 5.5：左 $\{0,0,0,1,0\}$，$\text{Gini}_L=1-0.2^2-0.8^2=0.32$；右全为 1，$0$；加权 $=0.5\times0.32=0.16$。

最小的是阈值 5.5（0.16），略优于 3.5（0.1714）。这说明贪心分裂并不看"哪个叶子最纯"，只看加权和。信息增益版本：根熵 $H=0.9710$；若按阈值 4.5 分裂，$H_L=0.8113$、$H_R=0.6500$，增益 $=0.9710-(0.4\times0.8113+0.6\times0.6500)=0.2564$（Python 复核）。

### 4.4 过拟合与剪枝

不限制深度的树能把训练集完全分对（每个叶子一个样本），方差极大。控制手段：最大深度、叶子最小样本数、最小增益；**预剪枝**在生长时停止，**后剪枝**先长满再按验证集或代价复杂度（$R_\alpha(T)=R(T)+\alpha|T|$）裁剪。单棵树对数据微小扰动敏感，这正是集成学习要解决的。

## 5. 集成学习：bagging、随机森林与梯度提升

### 5.1 偏差—方差的两条路

集成的总原则：把许多"弱"模型组合成强模型。两条路线分别对付两种误差：

| 方法 | 基学习器 | 训练方式 | 主要降低 |
|---|---|---|---|
| Bagging / 随机森林 | 深树（低偏差高方差） | 并行，各自独立 | 方差 |
| Boosting | 浅树（高偏差低方差） | 串行，每棵纠正前一轮错误 | 偏差 |

**方差降低的推导**：$B$ 个同分布、方差 $\sigma^2$、两两相关系数 $\rho$ 的预测取平均，方差为

$$\operatorname{Var}\Big(\frac1B\sum_b\hat f_b\Big)=\rho\sigma^2+\frac{1-\rho}{B}\sigma^2.$$

$B\to\infty$ 后只剩 $\rho\sigma^2$，所以要降低树之间的相关性。

### 5.2 随机森林

做法：(1) 对训练集做自助采样（bootstrap，有放回抽 $N$ 个），约有 $1-(1-1/N)^N\to1-e^{-1}\approx63.2\%$ 的样本被选中，其余约 36.8% 为"袋外"（out-of-bag, OOB）样本；(2) 每个结点分裂时只随机选 $m$ 个特征（分类常取 $\sqrt d$，回归常取 $d/3$）挑最优；(3) 分类投票、回归平均。袋外样本可当作免费的验证集，得到 OOB 误差。随机选特征正是为了降低 $\rho$。

### 5.3 梯度提升

加法模型 $F_M(\mathbf x)=F_0+\eta\sum_{m=1}^Mh_m(\mathbf x)$。每一轮在**函数空间做梯度下降**：对损失 $L(y,F)$，计算负梯度（伪残差）$r_i=-\partial L(y_i,F)/\partial F\big|_{F=F_{m-1}(\mathbf x_i)}$，拟合一棵小树 $h_m$ 去逼近 $r_i$，再以学习率 $\eta$ 加入模型。平方损失 $L=\tfrac12(y-F)^2$ 时负梯度恰好是普通残差 $y-F$，所以"拟合残差"是梯度提升的特例；逻辑损失下伪残差是 $y-p$。

**完整手算（示例，非项目实测）**：6 个点，$x=1,\dots,6$，目标 $y=(1.0,1.2,1.1,3.0,3.2,2.9)$，用平方损失、树桩（一次分裂）、学习率 $\eta=0.5$。

1. 初值 $F_0=\bar y=2.0667$。
2. 残差 $r=(-1.0667,-0.8667,-0.9667,0.9333,1.1333,0.8333)$。
3. 树桩候选阈值的残差平方和：1.5 为 4.308；2.5 为 2.870；3.5 为 0.0667；4.5 为 2.7725；5.5 为 4.840。选阈值 3.5，左叶均值 $-0.9667$，右叶均值 $+0.9667$。
4. 更新 $F_1=F_0+0.5h$：左 $2.0667-0.4833=1.5833$，右 $2.0667+0.4833=2.55$。
5. 新残差 $(-0.5833,-0.3833,-0.4833,0.45,0.65,0.35)$；均方误差从 $0.9456$ 降到 $0.2447$（Python 复核）。

继续迭代，残差会逐轮缩小。学习率 $\eta$ 越小需要的树越多，但泛化通常更好（"收缩"正则）。常用的工程实现（XGBoost、LightGBM 等）在此基础上加入二阶泰勒展开、正则项、直方图加速等改进，具体细节本文未核对，面试时说"二阶梯度加正则"即可。

### 5.4 随机森林与梯度提升对照

| 维度 | 随机森林 | 梯度提升 |
|---|---|---|
| 并行性 | 树可并行 | 树串行（树内可并行） |
| 过拟合风险 | 低，加树不会更糟 | 高，需早停、收缩、限深 |
| 调参 | 较少，默认效果好 | 较多（深度、学习率、树数、子采样） |
| 典型精度 | 良好 | 通常更高 |
| 对噪声标签 | 较鲁棒 | 敏感 |

## 6. K-means 与 GMM：聚类

### 6.1 它解决什么问题

没有标签时，把相似样本分成簇。机器人里的例子：对激光点云分割物体；对检测框尺寸聚类得到锚框（anchor）；对示教轨迹分模式。

### 6.2 K-means

目标是最小化簇内平方和（SSE，也叫 inertia）：

$$J=\sum_{k=1}^K\sum_{\mathbf x_i\in S_k}\lVert\mathbf x_i-\boldsymbol\mu_k\rVert^2.$$

Lloyd 算法交替两步：

1. 分配：每个点归到最近的中心；
2. 更新：每个中心设为簇内均值。

两步各自都不增加 $J$（更新步的均值正是平方误差的最小点），而 $J$ 有下界，所以必收敛，但只收敛到**局部最优**，依赖初始化。常用 K-means++：第一个中心随机，后续中心按到已选中心最近距离的平方为概率抽取，使初始中心分散。

**完整手算（示例，非项目实测）**：7 个二维点 $A(1,1),B(1.5,2),C(3,4),D(5,7),E(3.5,5),F(4.5,5),G(3.5,4.5)$，$K=2$，初始中心取 $A$ 与 $D$。

第 1 轮分配（欧氏距离平方，$c_1=(1,1)$、$c_2=(5,7)$）：

| 点 | 到 $c_1$ | 到 $c_2$ | 归属 |
|---|---|---|---|
| A | 0 | 52 | 1 |
| B | 1.25 | 37.25 | 1 |
| C | 13 | 13 | 1（平局取先者） |
| D | 52 | 0 | 2 |
| E | 22.25 | 6.25 | 2 |
| F | 28.25 | 4.25 | 2 |
| G | 18.5 | 8.5 | 2 |

更新：$c_1=\big((1+1.5+3)/3,(1+2+4)/3\big)=(1.833,2.333)$，$c_2=\big((5+3.5+4.5+3.5)/4,(7+5+5+4.5)/4\big)=(4.125,5.375)$，SSE $=12.21$。

第 2 轮：点 $C$ 到 $c_1$ 距离平方 4.139，到 $c_2$ 为 3.156，改归簇 2；$A,B$ 留在簇 1。新中心 $c_1=(1.25,1.5)$，$c_2=(3.9,5.1)$，SSE $=8.525$。

第 3 轮：归属不变，算法收敛。Python 复核：SSE 序列 12.208、8.525、8.525。

**选 $K$**：肘部法（看 SSE 随 $K$ 的下降拐点）、轮廓系数（silhouette）、业务先验。K-means 隐含假设：簇近似球形、大小相近、各向同性，对离群点敏感，需要标准化特征。

### 6.3 高斯混合模型与 EM

GMM 假设数据来自 $K$ 个高斯的混合：

$$p(\mathbf x)=\sum_{k=1}^K\pi_k\,\mathcal N(\mathbf x\mid\boldsymbol\mu_k,\boldsymbol\Sigma_k),\quad\sum_k\pi_k=1.$$

对数似然里有"对和取对数"，无闭式解。EM 引入隐变量 $z_i$（点属于哪个分量）：

- **E 步**：计算责任度 $\gamma_{ik}=\dfrac{\pi_k\mathcal N(\mathbf x_i\mid\boldsymbol\mu_k,\boldsymbol\Sigma_k)}{\sum_j\pi_j\mathcal N(\mathbf x_i\mid\boldsymbol\mu_j,\boldsymbol\Sigma_j)}$；
- **M 步**：$N_k=\sum_i\gamma_{ik}$，$\boldsymbol\mu_k=\frac1{N_k}\sum_i\gamma_{ik}\mathbf x_i$，$\boldsymbol\Sigma_k=\frac1{N_k}\sum_i\gamma_{ik}(\mathbf x_i-\boldsymbol\mu_k)(\mathbf x_i-\boldsymbol\mu_k)^\top$，$\pi_k=N_k/N$。

每次迭代似然不降（这是 EM 的基本性质，由 Jensen 不等式给出下界保证），同样只到局部最优。

**K-means 与 GMM 的关系**：若固定 $\boldsymbol\Sigma_k=\sigma^2\mathbf I$ 且 $\sigma^2\to0$，责任度趋于 0/1 的硬分配，EM 退化为 K-means。因此 K-means 是"等方差、硬分配"的 GMM；GMM 给出软分配和各簇的形状与大小。

**一维完整手算（示例，非项目实测）**：5 个点 $x=(1.0,1.2,4.8,5.0,2.9)$，两个分量初始 $\mu=(1,5)$、$\sigma=(1,1)$、$\pi=(0.5,0.5)$。

对 $x=2.9$：$\mathcal N(2.9\mid1,1)=\frac1{\sqrt{2\pi}}e^{-1.805}=0.3989\times0.1645=0.0656$；$\mathcal N(2.9\mid5,1)=0.3989\times e^{-2.205}=0.3989\times0.1102=0.0440$。责任度 $\gamma_{\cdot1}=0.0656/(0.0656+0.0440)=0.599$，$\gamma_{\cdot2}=0.401$（Python 复核 0.5987、0.4013）。其余点几乎完全属于某一分量（如 $x=1.0$ 的责任度 0.9997）。

M 步：$N_1=2.599$、$N_2=2.401$；$\mu_1=1.516$、$\mu_2=4.564$；$\sigma_1^2=0.587$、$\sigma_2^2=0.570$；$\pi=(0.520,0.480)$。可见模糊点 2.9 把两个均值都向中间"拉"了一点，这是软分配的体现。

**奇异性问题**：若某个分量的中心塌缩到单个数据点，协方差趋于零，似然趋于无穷。实践中给协方差加小对角项（正则）。选 $K$ 用 BIC/AIC。

## 7. PCA 与 t-SNE：降维

### 7.1 PCA 要解决什么问题

数据有 $d$ 维，但变化主要集中在少数方向。PCA 找到方差最大的正交方向，把数据投影到前 $k$ 个方向，用最小的信息损失降维、去噪、可视化。

### 7.2 推导

先中心化 $\tilde{\mathbf x}_i=\mathbf x_i-\bar{\mathbf x}$。投影到单位向量 $\mathbf u$ 的方差为 $\mathbf u^\top\mathbf S\mathbf u$，其中样本协方差 $\mathbf S=\frac1{N-1}\sum_i\tilde{\mathbf x}_i\tilde{\mathbf x}_i^\top$。最大化 $\mathbf u^\top\mathbf S\mathbf u$ 约束 $\mathbf u^\top\mathbf u=1$，拉格朗日：$\mathbf u^\top\mathbf S\mathbf u-\lambda(\mathbf u^\top\mathbf u-1)$，求导得 $\mathbf S\mathbf u=\lambda\mathbf u$，此时方差等于 $\lambda$。所以**第一主成分是最大特征值对应的特征向量**，第二主成分是在与第一正交的约束下的下一个，以此类推。等价地，PCA 是"最小重构误差"的线性投影：重构误差等于被舍弃的特征值之和（乘以 $N-1$）。

与 SVD 的关系：设中心化数据矩阵 $\tilde{\mathbf X}\in\mathbb R^{N\times d}$，$\tilde{\mathbf X}=\mathbf U\boldsymbol\Sigma\mathbf V^\top$，则 $\mathbf V$ 的列是主成分方向，特征值 $\lambda_j=\sigma_j^2/(N-1)$。工程上直接做 SVD，比显式算协方差数值更稳，详见 [数值线性代数与稀疏求解](../01_数学基础/08_数值线性代数与稀疏求解.md)。

### 7.3 完整手算（示例，非项目实测）

5 个二维点：$(2,1),(3,2),(4,3),(5,5),(6,4)$。

1. 均值 $\bar{\mathbf x}=(4,3)$。中心化后为 $(-2,-2),(-1,-1),(0,0),(1,2),(2,1)$。
2. 协方差：$S_{11}=(4+1+0+1+4)/4=2.5$，$S_{22}=(4+1+0+4+1)/4=2.5$，$S_{12}=(4+1+0+2+2)/4=2.25$。
3. 特征值：对称矩阵 $\begin{bmatrix}2.5&2.25\\2.25&2.5\end{bmatrix}$ 的特征值为 $2.5\pm2.25=4.75$ 与 $0.25$，对应特征向量 $(1,1)/\sqrt2$ 与 $(-1,1)/\sqrt2$。
4. 解释方差比：$4.75/5=95\%$，第一主成分保留 95% 的方差。
5. 投影到第一主成分：$z=(\tilde x_1+\tilde x_2)/\sqrt2=(-2.828,-1.414,0,2.121,2.121)$。
6. 重构误差平方和 $=1.0=0.25\times(N-1)$，即被舍弃特征值乘 $N-1$。Python 复核：特征值 $[0.25,4.75]$，SVD 平方奇异值除以 $N-1$ 同样为 $[4.75,0.25]$，重构误差 1.0。

**选 $k$**：累积解释方差达 90%–99%，或观察特征值"碎石图"的拐点。PCA 前必须中心化，量纲不同的特征要先标准化，否则方差大的量纲统治第一主成分。

> **提示**：点云平面拟合就是 PCA 的直接应用：邻域点协方差最小特征值对应的特征向量就是平面法向，最小特征值与总和之比衡量"平面度"。参见 [点云处理](../09_感知与视觉/02_点云处理_滤波_配准_ICP.md)。

### 7.4 PCA 的局限与 t-SNE

PCA 只捕捉线性结构，且最大方差方向不一定是对任务最有用的方向（有监督的做法见 LDA）。**t-SNE** 是非线性可视化方法：在高维用高斯核定义点对相似度 $p_{ij}$，在低维用自由度为 1 的 t 分布（重尾）定义 $q_{ij}$，最小化 $\mathrm{KL}(P\Vert Q)$。重尾使得低维里中等距离的点被推得更开，缓解"拥挤问题"。使用时的注意事项：

- 只用于可视化，不可当作特征提取（映射不可逆、对新样本无显式映射）；
- 簇之间的距离、簇的大小没有定量意义；
- 结果依赖困惑度（perplexity，粗略理解为有效邻居数，常见范围 5–50）与随机种子；
- 高维数据常先用 PCA 降到几十维再做 t-SNE 以提速降噪。

UMAP 是另一种常见的非线性方法，其细节本文未核对。

## 8. 方法选择：什么时候用什么

| 场景 | 优先考虑 | 理由 |
|---|---|---|
| 表格特征、样本几千到几十万 | 梯度提升树 | 精度高、免特征缩放、可处理异质特征 |
| 小样本、高维、需要稳健边界 | SVM（RBF 或线性） | 最大间隔泛化好，有凸优化保证 |
| 需要解释规则 | 浅层决策树 | 规则可读 |
| 基线与快速验证 | 随机森林 | 几乎不用调参 |
| 无标签、先看结构 | K-means，其次 GMM | K-means 快；GMM 给不确定性 |
| 特征太多、共线 | PCA | 去相关、压缩 |
| 图像、语音、文本 | 深度网络 | 表示学习替代手工特征 |

决策原则是"从最简单的基线开始"：先上逻辑回归或随机森林建立基线，再决定是否值得上复杂模型。

## 9. 在七个项目里怎么用（含诚实边界）

七个项目的 CV 表述中，没有一项使用 SVM、树模型或聚类来做核心功能。以下是**知识上的联系**，面试时请明确说这是延伸理解，而不是项目所做。

| 项目 | 联系点 | 性质 |
|---|---|---|
| [P4 智能园艺：YOLOv5/RKNN 推理](../../01_项目课程/P4_OpenHarmony智能园艺系统/06_YOLOv5输出解码_NMS与结构化事件.md) | YOLO 系列的锚框先验通常由对训练集标注框尺寸做聚类（K-means 思想，常以 $1-\text{IoU}$ 为距离）得到；训练工具里自动锚框功能的具体实现版本差异本文未核对 | 延伸，非项目所做（项目使用现成训练流程） |
| [P4 决策链](../../01_项目课程/P4_OpenHarmony智能园艺系统/10_LangGraph决策链.md) | 规则快速路径本质上是一棵手写的决策树（阈值判断 if–else），可用决策树理解其结构与可解释性 | 类比 |
| [P4 Hi3861 滞回状态机](../../01_项目课程/P4_OpenHarmony智能园艺系统/04_Hi3861滞回状态机_继电器与手动优先.md) | 土壤湿度上下阈值是人工设定的"决策桩"；如果日后想从数据中学习阈值，可用单特征决策树选分裂点 | 延伸 |
| P5 激光点云（[点云处理](../09_感知与视觉/02_点云处理_滤波_配准_ICP.md)） | PCA 求局部法向、欧氏聚类分割障碍物，与 K-means 同属"聚类"思想 | 通用知识；导航栈中障碍物以代价地图表示 |
| P1/P7 状态估计 | 协方差、特征分解、"高斯分布的形状"在卡尔曼滤波中同样出现；PCA 的椭圆就是协方差椭圆 | 数学联系 |
| P2 示教数据 | 若对多次示教轨迹做模式分析，可用 GMM 或 PCA（例如运动基元的参数化思想），见 [模仿学习](./10_模仿学习_行为克隆_DAgger_扩散策略.md) | 延伸，非项目所做 |

> **面试要点**：被问到"你用过 SVM/随机森林吗"时，诚实的回答是："我的项目偏控制和部署，主要用到的学习模型是 YOLO 检测网络。经典方法我系统学过，能手推 SVM 并解释树模型和 PCA，下面我讲讲我对它们的理解。"这比硬说用过要稳得多，并符合你在申请材料中"实践为主、想补理论"的定位。

## 10. 常见错误与故障现象

| 现象 | 原因 | 处理 |
|---|---|---|
| SVM 训练极慢或不收敛 | 特征未标准化；$C$ 过大；样本过多 | 标准化、减小 $C$、改用线性 SVM 或采样 |
| RBF-SVM 训练集 100% 测试集很差 | $\gamma$（核宽度倒数）过大 | 交叉验证联合搜索 $C$ 与 $\gamma$ |
| 决策树训练准确率 100% | 未限制深度 | 剪枝、限制叶子样本数 |
| 随机森林特征重要性误导 | 高基数特征、相关特征被稀释 | 用置换重要性，注意相关性 |
| 数据泄漏：交叉验证过于乐观 | 标准化/PCA 在全数据上拟合后才划分 | 只在训练折上拟合预处理，用管道（pipeline） |
| K-means 结果每次不同 | 随机初始化 | K-means++，多次初始化取最小 SSE |
| K-means 把一个大簇劈成两半 | 簇不是球形或大小差异大 | 换 GMM / DBSCAN |
| GMM 似然变成 NaN 或无穷 | 协方差奇异 | 给对角加小正数，减小 $K$ |
| PCA 第一主成分被某特征垄断 | 未标准化，量纲不同 | 先标准化 |
| t-SNE 图里"簇间距离"被过度解读 | 该距离无定量意义 | 只作定性观察，配合定量指标 |
| 类别不平衡下准确率虚高 | 多数类主导 | 用精确率/召回率/PR 曲线，加权或重采样 |

## 11. 面试怎么问、怎么答

**Q1. Explain how an SVM works and what the kernel trick is.**

> An SVM looks for the separating hyperplane with the largest margin, meaning the one that stays as far as possible from the closest points of both classes. Writing the margin as two over the norm of w, maximizing it becomes a convex quadratic program. The solution depends only on a few support vectors. In the dual form the data appear only through inner products, so I can replace them with a kernel function, such as an RBF kernel, and get a nonlinear boundary without ever computing the high-dimensional features explicitly.

中文：SVM 寻找离两类最近点最远的超平面；间隔 $2/\lVert w\rVert$，最大化转化为凸二次规划，只有支持向量起作用；对偶形式里数据只以内积出现，用核函数替换内积即可得到非线性边界，不用显式算高维特征。

*Follow-up: What do C and gamma do?*

> C is the penalty on margin violations: a large C fits the training data tightly with a narrow margin and risks overfitting, a small C gives a wider and smoother margin. Gamma in the RBF kernel controls how far one point's influence reaches: a large gamma gives a very wiggly boundary. I tune them together by cross-validation.

中文：$C$ 惩罚越界；大则窄间隔、易过拟合。$\gamma$ 控制单点影响范围；大则边界蜿蜒。二者用交叉验证联合调。

**Q2. How does a random forest work, and why does it reduce variance?**

> Each tree is trained on a bootstrap sample, and at each split only a random subset of features is considered. The trees are deep, so each has low bias but high variance. Averaging many of them reduces the variance, and the randomness in the features lowers the correlation between trees, which is what limits the benefit. The out-of-bag samples give a free validation estimate.

中文：每棵树用自助样本，每次分裂只考虑随机特征子集；深树低偏差高方差，平均降方差；特征随机降低树间相关性；袋外样本提供免费验证。

*Follow-up: What is the variance of the average of B correlated trees?*

> It is rho times sigma squared plus one minus rho over B times sigma squared. As B grows, only the rho term remains, so decorrelating the trees matters.

中文：方差为 $\rho\sigma^2+\frac{1-\rho}{B}\sigma^2$，$B$ 大时只剩 $\rho\sigma^2$，所以要降低相关性。

**Q3. Gradient boosting versus random forest: what is the difference and when would you pick each?**

> A random forest builds deep trees in parallel and averages them to cut variance. Gradient boosting builds shallow trees sequentially, each one fitting the negative gradient of the loss, which for squared error is just the residual; it mainly cuts bias. Boosting is usually more accurate but needs careful tuning with learning rate, depth and early stopping, and it is more sensitive to noisy labels. I would start with a random forest as a robust baseline and move to boosting when I need the last bit of accuracy.

中文：随机森林并行深树平均降方差；梯度提升串行浅树拟合损失的负梯度（平方损失即残差），主要降偏差；提升更准但调参多且对噪声敏感；先用森林做基线，再上提升。

**Q4. What is the relationship between K-means and a Gaussian mixture model?**

> K-means is a special case of a Gaussian mixture fitted by EM, where all clusters share the same spherical covariance and the covariance goes to zero, so the soft responsibilities become hard assignments. GMM is more flexible: it gives soft membership, different cluster shapes and sizes, and a proper likelihood for model selection with BIC. But it is more expensive and can suffer from singular covariances.

中文：K-means 是等方差球形协方差且方差趋于 0 的 GMM，软责任度变成硬分配；GMM 有软归属、各簇形状大小不同、可用 BIC 选模型，但更贵且会出现协方差奇异。

**Q5. Explain PCA and when it fails.**

> PCA finds orthogonal directions of maximum variance. Mathematically they are the eigenvectors of the covariance matrix, and the eigenvalues are the variances along them; equivalently it minimizes reconstruction error, and in practice I compute it with an SVD of the centered data. It fails when the structure is nonlinear, when features are not standardized so that units dominate, or when the highest-variance direction is not the informative one for the task.

中文：PCA 找最大方差正交方向，即协方差矩阵特征向量，特征值是方差；等价于最小重构误差，实际用中心化数据的 SVD 计算。失效情形：非线性结构、未标准化、方差最大方向并非任务相关方向。

**Q6. Have you used these methods in your projects?**（项目对照）

> Not as core components. My projects were mostly control and deployment, and the main learned model was a YOLOv5 detector on an RK3588. But I studied the classical methods properly. For example, I know YOLO anchors come from clustering box shapes, and I use PCA-style reasoning for plane fitting in point clouds. I am happy to go through any of them on the whiteboard.

中文：不是核心部件。项目主要是控制与部署，学习模型是 RK3588 上的 YOLOv5。经典方法我系统学过，例如锚框来自框形状聚类、点云平面拟合用 PCA 思路，愿意在白板上讲。

## 12. 本章要点速记

1. 硬间隔 SVM：$\min\frac12\lVert w\rVert^2$ s.t. $y_i(w^\top x_i+b)\ge1$；间隔宽度 $2/\lVert w\rVert$；只有支持向量决定解。
2. 对偶：$w=\sum\alpha_iy_ix_i$，$\sum\alpha_iy_i=0$；软间隔加 $0\le\alpha_i\le C$；等价于铰链损失 + L2。
3. 核技巧：把内积换成 $k(x,z)$；核矩阵必须半正定；RBF 对应无穷维特征。
4. 基尼 $1-\sum p_k^2$，熵 $-\sum p_k\log_2p_k$；贪心分裂看加权不纯度；单树方差大，需剪枝。
5. 随机森林：bootstrap + 随机特征，降方差；约 63.2% 样本进入每棵树，其余为袋外样本。
6. 梯度提升：函数空间梯度下降，每轮拟合负梯度；平方损失下即残差；学习率与早停控制过拟合。
7. K-means：分配与更新交替，SSE 单调不增，收敛于局部最优；用 K-means++ 初始化。
8. GMM/EM：E 步算责任度，M 步更新 $\pi,\mu,\Sigma$；K-means 是其硬分配、等方差特例。
9. PCA：协方差的特征分解 = 中心化数据的 SVD；解释方差比 $\lambda_k/\sum\lambda$；先中心化与标准化。
10. t-SNE 只用于可视化；本章方法与七个项目多为"延伸"，面试时诚实说明。

## 13. 自测题

1. 硬间隔 SVM 中 $\lVert\mathbf w\rVert=0.5$，间隔带总宽度是多少？（$2/0.5=4$。）
2. 一个结点 8 个样本，5 正 3 负，基尼不纯度是多少？（$1-(5/8)^2-(3/8)^2=1-0.3906-0.1406=0.4688$。）
3. 数据集有 $N=1000$ 个样本，随机森林每棵树大约有多少个不同样本被选中？（约 $63.2\%$，即 632 个，其余为袋外样本。）
4. 一个 K-means 簇含点 $(0,0),(2,0),(1,3)$，更新后的中心是什么？（$(1,1)$。）
5. 协方差矩阵特征值为 $(6,3,1)$，前两个主成分保留多少方差？（$9/10=90\%$。）

## 14. 相关链接

- 上一章：[模仿学习：行为克隆、DAgger、动作分块与扩散策略](./10_模仿学习_行为克隆_DAgger_扩散策略.md)；下一章：[概率图模型、贝叶斯方法与高斯过程](./12_概率图模型_贝叶斯方法_高斯过程.md)
- 前置：[机器学习基础](./01_机器学习基础.md)、[最优化与二次规划](../01_数学基础/04_最优化与二次规划.md)
- 相关：[点云处理](../09_感知与视觉/02_点云处理_滤波_配准_ICP.md)、[数值线性代数与稀疏求解](../01_数学基础/08_数值线性代数与稀疏求解.md)
- 八股速查：[机器学习八股](../../03_八股知识/02_人工智能/01_机器学习八股.md)
- 模块说明：[10_机器学习与深度学习 README](./README.md)
