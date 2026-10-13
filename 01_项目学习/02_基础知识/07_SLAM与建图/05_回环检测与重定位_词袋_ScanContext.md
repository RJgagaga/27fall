# 回环检测与重定位：视觉词袋、Scan Context 与几何验证

> **一句话定位**：回环检测（loop closure detection）回答"我是不是回到了以前来过的地方"，重定位（relocalisation）回答"我现在到底在已知地图的哪里"。两者本质都是**地点识别（place recognition）**：先用便宜的全局描述子在大量历史帧里找候选，再用昂贵的几何方法验证，最后才把约束交给后端。本章讲视觉词袋（Bag of Words, BoW）、TF-IDF、倒排索引，激光的 Scan Context，几何验证，以及精确率与召回率的取舍。

**本章学完你能：**

- 说出回环在 SLAM 里的作用，以及为什么"误回环比漏回环更危险"；
- 构造视觉词典（词汇树），手算 TF-IDF 加权向量与相似度，并解释倒排索引为什么快；
- 解释感知混淆（perceptual aliasing），说出时序一致性与几何验证如何缓解；
- 描述 Scan Context 的构造（极坐标分块 + 最大高度）、环键（ring key）与列平移比较，手算一次距离；
- 区分回环检测、全局重定位与"被绑架机器人"问题，并说明它们与 AMCL 的关系；
- 手算精确率、召回率，解释阈值如何折中；
- 用英文回答 "How does loop closure work / What is a bag of visual words / How does Scan Context achieve rotation invariance"。

**前置知识**：[SLAM 问题：前端、后端与位姿图优化](./01_SLAM问题_前端后端_位姿图优化.md)、[视觉里程计：特征、对极几何、PnP 与 BA](./02_视觉里程计_特征_对极几何_PnP_BA.md)、[激光 SLAM](./03_激光SLAM_LOAM_LIO-SAM_Cartographer.md)、[粒子滤波与 AMCL](../06_状态估计/05_粒子滤波与AMCL.md)、[信息论与图论基础](../01_数学基础/07_信息论与图论基础.md)（TF-IDF 的对数来自信息量思想）。

## 目录

1. 回环与重定位要解决什么问题
2. 符号与单位
3. 总流程：候选—验证—约束
4. 视觉词袋：词典怎么建
5. TF-IDF 与相似度（含手算）
6. 倒排索引与检索效率
7. 感知混淆与时序一致性
8. 几何验证
9. Scan Context：激光的全局描述子（含手算）
10. 学习式地点识别（概述）
11. 评价：精确率、召回率与阈值（含手算）
12. 回环约束进入后端
13. 重定位与被绑架机器人
14. 在本项目里怎么用
15. 常见错误与故障现象
16. 面试怎么问、怎么答
17. 本章要点速记
18. 自测题
19. 相关链接

---

## 1. 回环与重定位要解决什么问题

SLAM 的前端（视觉里程计、激光里程计）只能估计相邻帧的相对运动。每一步都有小误差，累积起来就是**漂移**：机器人沿一圈走廊走回起点，估计的位置却离起点差了几十厘米甚至几米。

回环检测要做的是：

1. 发现"现在看到的场景和很久以前某一时刻看到的是同一个地方"；
2. 估计两个时刻之间的相对位姿；
3. 把这个约束加入位姿图，让后端优化把整条轨迹"拉直"。

一个回环约束的价值很大：它是除了 GPS 以外**唯一能直接消除全局漂移**的信息来源（见 [SLAM 问题](./01_SLAM问题_前端后端_位姿图优化.md) 的位姿图优化）。

重定位的场景稍有不同：机器人已有一张地图，但**不知道自己在哪**（刚开机、跟踪丢失、被人搬走），需要仅用当前观测在地图中找到自己的位姿。技术上与回环相同，只是"历史帧"换成了"地图里的关键帧"。

> **注意**：回环检测的质量要求不对称。漏掉一个回环（假阴性）只是漂移多保留一会儿；加入一个错误回环（假阳性）会把整张地图扭曲，几乎不可恢复。所以回环检测追求**接近 100% 的精确率**，宁可少检、不能错检。

## 2. 符号与单位

| 符号 | 含义 | 单位 / 说明 |
|---|---|---|
| $N$ | 数据库中的图像（关键帧）总数 | 个 |
| $n_w$ | 包含视觉单词 $w$ 的图像数 | 个 |
| $\text{idf}_w=\ln(N/n_w)$ | 逆文档频率 | 无量纲 |
| $\text{tf}_{w,d}$ | 单词 $w$ 在图像 $d$ 中的词频（相对） | 无量纲 |
| $\mathbf v_d$ | 图像 $d$ 的词袋向量 | 维度 = 词典大小 |
| $s(\mathbf v_1,\mathbf v_2)$ | 相似度得分 | 越大越相似 |
| $k,L$ | 词汇树的分支数、层数 | 词典大小 $k^L$ |
| $N_r,N_s$ | Scan Context 的环数、扇区数 | 个 |
| $\text{TP},\text{FP},\text{FN}$ | 真阳性、假阳性、假阴性 | 个 |

## 3. 总流程：候选—验证—约束

不论视觉还是激光，几乎所有系统都是同一个三段式：

```mermaid
flowchart LR
  A[当前帧] --> B[全局描述子]
  B --> C[数据库检索候选]
  C --> D[时序一致性]
  D --> E[几何验证]
  E --> F[相对位姿]
  F --> G[加入位姿图]
```

- **候选**：全局描述子（词袋向量、Scan Context）在数据库里做快速近邻检索，给出若干可能的历史帧；
- **验证**：用几何一致性（RANSAC 估计位姿、内点数）筛掉假回环；
- **约束**：验证成功后得到相对位姿，作为一条边加入后端。

候选要快（毫秒级、能扩展到数万帧），验证要准。把两者分开的思路，就是"粗筛 + 精验"。

## 4. 视觉词袋：词典怎么建

### 4.1 想法

把一张图看成一篇"文章"，把局部特征描述子（ORB、SIFT 等）看成"字"。直接比较两张图的几百个描述子太慢，也无法扩展。词袋模型的想法：**先把描述子空间离散成有限个"单词"，每张图只记录每个单词出现了多少次**，得到一个定长向量，这样比较两张图就只是比较两个向量。

### 4.2 建词典

1. 在大量训练图像上提取描述子（与场景无关的大数据集，或者场景自己的数据）；
2. 对描述子做 **k 均值聚类**，分成 $k$ 个簇，每个簇中心就是一个单词；
3. 对每个簇继续递归聚类，构成 **词汇树（vocabulary tree）**：分支数 $k$，深度 $L$，叶子数 $k^L$ 就是词典大小。

示例（非项目实测）：$k=10,L=6$ 得 $10^6=100$ 万个单词。查词时，从根开始每层只需比较 $k$ 个中心，总共 $k\times L=60$ 次距离计算就能把一个描述子归到一个叶子，而不是和 100 万个中心逐一比较。这是树结构的价值：从 $O(k^L)$ 降到 $O(kL)$。

> **提示**：对二值描述子（如 ORB/BRIEF），聚类中心用逐位多数投票，距离用汉明距离，所以词典构建和查词都很快。这是 DBoW2（Gálvez-López 和 Tardós，2012，细节未核对）一类库的做法。

### 4.3 词典的一个问题

词典在训练场景上建好后固定。如果部署场景与训练场景差异大，词典区分力下降。同时词典太小（单词少）会把不同东西合成同一个单词，太大则相似的东西被拆开，都影响检索质量。

## 5. TF-IDF 与相似度（含手算）

### 5.1 为什么需要加权

有些单词到处都是（如"墙面的竖直边缘"），对区分地点没有帮助；有些单词很稀有（如"一个特殊的海报"），出现一次就高度可疑。TF-IDF 的权重就是给**稀有单词更高的权重**：

$$
\text{idf}_w=\ln\frac{N}{n_w},\qquad \text{tf}_{w,d}=\frac{n_{w,d}}{n_d},\qquad \text{weight}_{w,d}=\text{tf}_{w,d}\cdot\text{idf}_w
$$

其中 $n_{w,d}$ 为单词 $w$ 在图像 $d$ 中出现的次数，$n_d$ 为图像 $d$ 的单词总数。idf 的对数形式和信息量 $-\ln p$ 同源：单词出现概率 $p=n_w/N$ 越小，信息量越大。

### 5.2 相似度

得到两张图的加权向量 $\mathbf v_1,\mathbf v_2$ 后，可以用余弦相似度，或 DBoW 常用的 L1 得分：

$$
s_{\cos}=\frac{\mathbf v_1\cdot\mathbf v_2}{\lVert\mathbf v_1\rVert\lVert\mathbf v_2\rVert},\qquad
s_{L1}=1-\frac12\Bigl\lVert\frac{\mathbf v_1}{\lVert\mathbf v_1\rVert_1}-\frac{\mathbf v_2}{\lVert\mathbf v_2\rVert_1}\Bigr\rVert_1
$$

L1 得分的取值范围是 0 到 1，越大越相似；两个归一化向量完全相同时得 1。

### 5.3 手算

示例（非项目实测）：词典只有 6 个单词（chair, desk, door, floor, wall, window），数据库 4 张图（用单词序列表示）：

| 图 | 单词序列 |
|---|---|
| A | door, window, door, wall |
| B | window, wall, desk, desk |
| C | door, door, door, floor |
| D | desk, chair, floor, wall |

$N=4$。各单词出现在多少张图（$n_w$）：chair 1，desk 2，door 2（A,C），floor 2（C,D），wall 3（A,B,D），window 2（A,B）。

$$
\text{idf}_{chair}=\ln4=1.386,\ \text{idf}_{desk}=\text{idf}_{door}=\text{idf}_{floor}=\text{idf}_{window}=\ln2=0.693,\ \text{idf}_{wall}=\ln(4/3)=0.288
$$

查询图 Q：door, window, door, floor。各词 tf：door 2/4=0.5，window 0.25，floor 0.25。加权：door $0.5\times0.693=0.347$，window $0.25\times0.693=0.173$，floor $0.173$。排好序（chair, desk, door, floor, wall, window）：$\mathbf q=(0,0,0.347,0.173,0,0.173)$。

图 A 的向量：door tf=0.5 得 0.347，window tf=0.25 得 0.173，wall tf=0.25 得 $0.25\times0.288=0.072$，所以 $\mathbf a=(0,0,0.347,0,0.072,0.173)$。

$$
\mathbf q\cdot\mathbf a=0.347^2+0.173^2=0.1204+0.0299=0.1503,\quad\lVert\mathbf q\rVert=0.4245,\ \lVert\mathbf a\rVert=0.3943
$$

$s_{\cos}(Q,A)=0.1503/(0.4245\times0.3943)=0.898$（Python 复核 `0.898`）。

Python 复核全部结果：

| 数据库图 | 余弦相似度 | L1 得分 |
|---|---|---|
| A | 0.898 | 0.75 |
| B | 0.180 | 0.25 |
| C | 0.904 | 0.75 |
| D | 0.164 | 0.226 |

结论有两条：(1) 与 Q 共享"door/window/floor"的 A、C 得分高，B、D 得分低，检索能把相关图排前面；(2) **C 的余弦得分（0.904）比 A（0.898）还略高**——C 只有 door×3 和 floor，却被判得比同时含 door、window 的 A 更像。这是词袋"不看空间位置、只看出现频率"的局限，也引出下一节的感知混淆。

## 6. 倒排索引与检索效率

如果每次查询都和数据库每一张图比较，复杂度是 $O(N)$ 张图 × 向量维度，数万帧时不可接受。**倒排索引（inverted index）** 的想法和搜索引擎一样：不按"图 → 单词"存，而是按"单词 → 包含它的图列表"存：

```text
单词 door   -> [A (权重0.347), C (权重0.520)]
单词 window -> [A (0.173), B (0.173)]
单词 floor  -> [C (0.173), D (0.173)]
...
```

查询图有哪些单词，就只访问这些单词对应的列表，只累加这些列表里的图的得分。词典越大，每个单词出现在的图越少，列表越短，检索越快。如果查询图含 200 个单词，每个单词平均只在 $N/10^4$ 张图出现，则总访问量约 $200\times N/10^4=N/50$ 张次，远小于 $N$。

倒排索引还有一个副产品：它自然支持**增量添加**，新来一帧只需要把它挂到对应单词的列表上。

## 7. 感知混淆与时序一致性

### 7.1 感知混淆

感知混淆（perceptual aliasing）：不同地点看起来相同。办公楼的走廊、同款的货架、重复的窗户。词袋忽略了空间布局，所以更容易被混淆。

### 7.2 缓解手段

1. **排除近邻帧**：时间上刚过去的几十秒内的帧一定相似，但它们不是回环，必须排除；
2. **得分归一化**：用当前帧与上一帧的相似度作为参考，只有显著高于"连续两帧的相似度"的得分才算候选；
3. **时序一致性**：要求连续 $k$ 帧都命中同一区域附近的候选，才接受。若单帧误检概率为 $p=0.1$ 且相互独立（教学简化，实际相关），则连续 2 帧都误检的概率为 $p^2=0.01$，连续 3 帧为 $0.001$（Python 复核）。这是用"时间换精确率"；
4. **几何验证**：下一节的重点。

> **面试要点**：别只说"用词袋检索"。完整答案是"词袋检索出候选，用时序一致性和几何验证过滤，最后才当作回环约束"。

## 8. 几何验证

候选帧的得分再高，也不能直接信，必须验证：

- **视觉**：对两帧做特征匹配，用 RANSAC 估计基础矩阵/本质矩阵（见 [视觉里程计](./02_视觉里程计_特征_对极几何_PnP_BA.md)），或者利用候选帧里有三维地图点，用 PnP + RANSAC 估计相对位姿。内点数（如不少于一个阈值）足够多，才接受；
- **激光**：用 ICP/NDT（见 [点云处理](../09_感知与视觉/02_点云处理_滤波_配准_ICP.md)）对两帧点云或子图做精配准，配准后的重叠率或残差低于阈值才接受；
- **位姿一致性检验**：把新得到的回环相对位姿与当前里程计估计比较，差异过大（超出漂移可能的范围）则怀疑。

这些验证手段属于"强几何约束"，对相似但不同的地点很有效：两个看起来相似的走廊，特征点的**空间分布**无法被同一个刚体变换解释，RANSAC 内点会很少。

## 9. Scan Context：激光的全局描述子（含手算）

### 9.1 动机

2D/3D 激光没有图像纹理，词袋不适用。需要一种直接从点云得到的、**对旋转（偏航）不敏感**的全局描述子。Scan Context（Kim 和 Kim，2018，细节未核对）是常见的一种。

### 9.2 构造

以传感器为中心，把水平面划分为极坐标网格：

- 径向分 $N_r$ 个**环（ring）**，从近到远；
- 角向分 $N_s$ 个**扇区（sector）**，每个扇区角度 $360^\circ/N_s$；
- 每个格子里取点云的**最大高度**（或其他统计量），空格为 0。

结果是一个 $N_r\times N_s$ 矩阵，叫 Scan Context。示例（非项目实测）：若半径范围 80 m 分 20 环、圆周分 60 扇区，则每环宽 4 m，每扇区 $6^\circ$，在 80 m 处弧长约 8.4 m（Python 复核：`2π×80/60=8.378`）。

### 9.3 对旋转的处理

偏航变化 $\Delta\psi$ 会让矩阵的**列循环平移** $\Delta\psi/(360^\circ/N_s)$ 列。所以比较两个 Scan Context 时，把一个矩阵的列依次循环平移，每次计算一个距离，取最小的：

$$
d(I^q,I^c)=\min_{n\in[0,N_s)}\ \frac1{N_s}\sum_{j=1}^{N_s}\Bigl(1-\frac{\mathbf c^q_j\cdot\mathbf c^c_{j+n}}{\lVert\mathbf c^q_j\rVert\lVert\mathbf c^c_{j+n}\rVert}\Bigr)
$$

其中 $\mathbf c_j$ 是矩阵第 $j$ 列（一个方向上从近到远的高度剖面），用余弦距离比较列向量；取最小值的平移量 $n^*$ 同时给出了**偏航角估计**，可作为后续 ICP 的初值。

### 9.4 环键（Ring Key）加速检索

对数据库里每个 Scan Context，每行（环）的平均值构成一个 $N_r$ 维向量，叫环键。它对列平移**不变**（平均值与列顺序无关），所以可以用 KD 树做近邻检索，先找出若干候选，再对候选做上面的完整距离比较，大大减少了计算。

### 9.5 手算

示例（非项目实测）：用 3 环 × 6 扇区的玩具矩阵（每扇区 $60^\circ$）：

$$
SC=\begin{bmatrix}2&3&1&0&0&4\\1&5&2&1&0&2\\0&2&6&3&1&0\end{bmatrix}
$$

环键为每行均值：$(10/6,\,11/6,\,12/6)=(1.667,\,1.833,\,2.0)$（Python 复核）。

设查询矩阵 $B$ 是 $SC$ 的列循环平移 2 列（等价于车辆偏航 $120^\circ$）。逐个平移 $n=0,\dots,5$ 计算上述距离，只有 $n=4$（即平移回去 2 列，因为 $4\equiv-2\pmod 6$）时距离为 0（Python：`dist=-1.85e-17≈0`，$n^*=4$），其余平移距离均明显大于 0。因此可以确认两者是同一地点，并估计相对偏航为 $\pm120^\circ$（符号取决于平移方向约定）。

再加入扰动：对 $SC$ 加标准差 0.3 的高斯噪声再平移 3 列，最小距离 0.0065；而与一个随机矩阵的最小距离是 0.199。差距明显，可以设阈值（如 0.05，教学示例）来区分。真实系统里 $N_s$ 取到几十，阈值需要在数据上调。

> **注意**：Scan Context 对**偏航**平移有不变性，对 $x,y$ 平移是近似的（因为平移改变了各格子的归属）。所以 Scan Context 只能提供候选和偏航初值，最终仍需要点云配准来细化。

## 10. 学习式地点识别（概述）

近年的方法用神经网络直接输出全局描述子，如图像的 NetVLAD（Arandjelović 等，2016，未核对）、点云的 PointNetVLAD（Uy 和 Lee，2018，未核对）。它们对光照、季节变化更鲁棒，但需要训练数据，泛化到新环境可能下降，并且部署在嵌入式设备上要考虑量化与算力（见 [模型量化与边缘部署](../10_机器学习与深度学习/04_模型量化与边缘部署_RKNN.md)）。无论哪种描述子，"候选加验证"的框架不变。

## 11. 评价：精确率、召回率与阈值（含手算）

对每一次"判为回环"的决策，有四种结果：

- TP：真回环被检出；FP：假回环被误检；FN：真回环被漏掉；TN：非回环被正确放过。

$$
\text{精确率}\ P=\frac{\text{TP}}{\text{TP}+\text{FP}},\qquad\text{召回率}\ R=\frac{\text{TP}}{\text{TP}+\text{FN}}
$$

示例（非项目实测）：一轮实验中 TP=8、FP=2、FN=4，则 $P=8/10=0.8$，$R=8/12=0.667$（Python 复核）。

调高相似度阈值：FP 减少、FN 增加，即精确率升、召回率降；调低则相反。对 SLAM，我们通常把阈值调到精确率接近 1，然后用**时序一致性、几何验证**提升召回率，而不是降低阈值。**精确率—召回率曲线**用来选择工作点；与单纯的 ROC 曲线相比，它更适合正负样本极不均衡的场景（回环在所有帧对里只占极少比例）。

## 12. 回环约束进入后端

验证通过后得到相对位姿 $\hat T_{ij}$，加入位姿图成为一条回环边：

$$
\mathbf e_{ij}=\mathrm{Log}\bigl(\hat T_{ij}^{-1}\,T_i^{-1}T_j\bigr)
$$

权重由配准的不确定性（如 ICP 的协方差估计或内点数）决定。为了防止个别错误回环毁掉整个图，后端通常使用**鲁棒核**（Huber、Cauchy）或**切换约束（switchable constraints）/ 动态协方差缩放**（Sünderhauf 和 Protzel，2012，未核对）等方法，让错误回环的权重自动被降下来。

回环生效后，整条轨迹被调整，地图上的位姿会发生**离散跳变**。导航里 `map->odom` 的变换会跳，而 `odom->base_link` 保持连续，这是 P5 的 TF 分工所体现的设计原则。

## 13. 重定位与被绑架机器人

| 场景 | 描述 | 典型做法 |
|---|---|---|
| 跟踪丢失 | VO 失败后重新找位姿 | 地点识别 + PnP/ICP 在地图中定位 |
| 全局定位 | 开机时位置未知 | AMCL 全局初始化（粒子均匀撒满地图）或地点识别 |
| 被绑架（kidnapped） | 运行中被搬到别处 | 检测到粒子云与观测严重不符，重新撒粒子 |
| 回环 | 在线建图时回到旧地 | 本章全流程 |

AMCL 的全局定位是"暴力"的：粒子数很大地覆盖整个地图，随着观测逐渐收敛；地点识别则是"检索式"的：直接从观测查到最像的位置，再精配准。两者可以组合：地点识别给出粗位姿，AMCL 以此为初值收敛，见 [粒子滤波与 AMCL](../06_状态估计/05_粒子滤波与AMCL.md)。

一个实用提醒：AMCL 重新初始化的粒子云如果收敛到**对称的错误位置**（比如两个相同的走廊），位姿会"正确地"收敛但是错误。此时需要额外的先验（如起点区域）或唯一性特征去区分。

## 14. 在本项目里怎么用

七个项目的 CV 没有回环检测的实现，本章属于**延伸知识**。可诚实关联：

- **P5**：使用已建好的地图做 EKF/AMCL 定位。AMCL 的粒子重采样、全局初始化与位姿跳变，对应本章第 13 节的"重定位"和第 12 节的"跳变"。P5 区分 `map->odom` 与 `odom->base_link`，使局部规划不被地图修正的跳变破坏。参见 [EKF 与 AMCL 协同：TF 分工与连续状态](../../01_项目课程/P5_车式移动机器人自主导航/06_EKF与AMCL协同_TF分工与连续状态.md) 与 [AMCL 粒子滤波定位](../../01_项目课程/P5_车式移动机器人自主导航/05_AMCL粒子滤波定位.md)。
- **P6**：在多机器人平台上，设备发现与能力注册类似"识别这个设备是不是我见过的那个"；任务与状态的有效性检查与"几何验证"的精神一致：不要盲信单一信号，必须交叉验证后再执行。见 [安全执行链](../../01_项目课程/P6_深开鸿M-Robots平台软件/08_安全执行链.md)。
- **P4**：YOLO 的目标识别可以视为"地点/物体识别"的一个简单形式，但属于检测而非检索。

诚实表述：我没有实现过回环检测。但我的导航项目里遇到过定位位姿跳变与连续性问题，所以我理解为什么需要把回环/重定位导致的跳变与局部控制隔离。

## 15. 常见错误与故障现象

| 现象 | 原因 | 处理 |
|---|---|---|
| 地图突然扭曲、墙体折断 | 误回环（假阳性）进入后端 | 提高阈值、做几何验证、使用鲁棒核 |
| 走了很远仍没有回环 | 阈值过严、词典不匹配场景、视角变化大 | 在目标场景重训词典；多视角关键帧 |
| 相邻帧被当成回环 | 没有排除时间近邻帧 | 设置最小时间间隔或最小关键帧间隔 |
| 重复走廊误匹配 | 感知混淆 | 引入位置先验、时序一致性、更大的验证阈值 |
| Scan Context 偏航估计偏差 | 扇区分辨率粗；传感器倾斜 | 以估计值作为 ICP 初值再精配准 |
| 回环后定位大跳变导致控制抖动 | 控制直接使用 `map` 系位姿 | 控制使用连续的 `odom` 系，仅目标用 `map` |
| 光照/季节变化后识别失败 | 手工特征对外观变化敏感 | 使用学习式描述子或多时段词典 |
| 检索越来越慢 | 数据库无限增长，无剪枝 | 限制关键帧密度；定期合并或删除冗余关键帧 |

## 16. 面试怎么问、怎么答

**Q1. What is loop closure and why does SLAM need it?**

> Odometry accumulates drift, so the estimated trajectory slowly diverges from the truth. Loop closure detects that the robot has returned to a previously visited place and computes the relative pose between the two visits. That adds a constraint to the pose graph, and optimisation spreads the correction along the whole trajectory, which removes the accumulated drift.

中文：里程计会累积漂移；回环检测发现回到旧地点并估计相对位姿，作为约束加入位姿图，优化把修正分摊到整条轨迹，消除累计漂移。追问：回环错了会怎样？（地图严重扭曲，所以追求高精确率。）

**Q2. Explain the bag-of-visual-words approach.**

> I cluster feature descriptors into a vocabulary, usually with a k-means tree. Each image becomes a histogram of visual words weighted by TF-IDF, so rare words count more. To find similar images quickly I use an inverted index that maps each word to the images containing it, so only the words in the query are visited. The weakness is that it ignores spatial layout, so I verify candidates geometrically.

中文：聚类描述子形成词典（k 均值树）；每张图是 TF-IDF 加权的词频直方图，稀有词权重高；倒排索引只访问查询里出现的单词；弱点是忽略空间布局，所以要几何验证。追问：词典太大或太小有什么问题？（太小区分力不足，太大相似的东西被拆开，匹配变稀疏。）

**Q3. What is perceptual aliasing and how do you handle it?**

> Different places can look alike, for example identical corridors or shelves, and bag-of-words ignores spatial arrangement, so it may retrieve a wrong place with a high score. I handle it by excluding temporal neighbours, requiring several consecutive consistent matches, and running geometric verification such as RANSAC for relative pose, which wrong places usually fail.

中文：不同地点外观相似（走廊、货架），词袋忽略空间布局会高分误检；用排除时间近邻、连续多帧一致、RANSAC 几何验证来处理。

**Q4. How does Scan Context handle rotation, and what is the ring key for?**

> Scan Context bins the point cloud into rings and sectors around the sensor and stores the maximum height per cell. A yaw rotation becomes a circular column shift, so I compare the two matrices over all column shifts and take the minimum distance; the best shift also gives a yaw estimate. The ring key is the row-wise mean, which does not change under column shifts, so I can use it in a KD-tree to retrieve candidates quickly before the full comparison.

中文：极坐标分环和扇区，格内存最大高度；偏航变成列循环平移，比较时遍历所有平移取最小，最优平移还给出偏航初值；环键是每行均值，对列平移不变，可用 KD 树快速找候选。

**Q5. How do you evaluate a loop closure detector, and which metric matters most?**

> I use precision and recall, usually as a precision-recall curve. In SLAM, precision matters most because one false loop closure can corrupt the whole map, while a missed loop only leaves some drift. So I set the threshold for near-100 per cent precision and then use temporal consistency and geometric verification to recover recall.

中文：用精确率—召回率曲线；SLAM 里精确率更重要，一个假回环会毁掉地图，漏检只是多留点漂移；阈值取接近 100% 精确率，再靠时序和几何验证提高召回。

## 17. 本章要点速记

1. 回环 = 地点识别 + 相对位姿估计 + 加入位姿图；重定位是同一技术换成"与地图关键帧匹配"。
2. 精确率远比召回率重要：一个假回环能毁掉整张地图。
3. 词袋：描述子聚类成词典，图像 = TF-IDF 加权词频向量；idf $=\ln(N/n_w)$。
4. 词汇树：$k^L$ 个单词，查询只需 $k\times L$ 次比较。
5. 倒排索引：单词 → 图列表，只访问查询里的单词，可增量添加。
6. 感知混淆靠"排除近邻 + 时序一致 + 几何验证"缓解。
7. Scan Context：环 × 扇区的最大高度矩阵；偏航 = 列平移；环键对列平移不变。
8. 几何验证：视觉用 RANSAC + 本质矩阵/PnP，激光用 ICP/NDT 看内点或残差。
9. 回环使位姿图整体调整，`map->odom` 会跳，局部控制应使用连续的 `odom` 系。
10. 本人项目只有"用地图定位"，没有实现回环检测，要如实说明。

## 18. 自测题

1. 词典有 $N=1000$ 张图，某单词出现在 10 张图里，idf 是多少？（$\ln100=4.605$。）
2. $k=8,L=5$ 的词汇树有多少单词？查一个描述子要比较多少次？（$8^5=32768$；$8\times5=40$。）
3. TP=15、FP=0、FN=15，精确率和召回率各是多少？说明这个工作点是否适合 SLAM。（P=1，R=0.5；适合，精确率为 1，只是漏检一半，通过更多关键帧仍可得到足够的回环。）
4. Scan Context 有 60 个扇区，两帧最优列平移为 15，偏航差是多少？（$15\times6^\circ=90^\circ$。）
5. 单帧误检概率 0.05，连续 3 帧独立都误检的概率？（$0.05^3=1.25\times10^{-4}$。）

## 19. 相关链接

- 上一章：[视觉惯性里程计 VIO：预积分与滑动窗口](./04_视觉惯性里程计VIO_预积分_滑动窗口.md)
- 下一章：[建图表示：栅格、八叉树、NDT、TSDF 与 ESDF](./06_建图表示_栅格_八叉树_NDT_TSDF_ESDF.md)
- 基础：[SLAM 问题与位姿图优化](./01_SLAM问题_前端后端_位姿图优化.md)、[点云处理与 ICP](../09_感知与视觉/02_点云处理_滤波_配准_ICP.md)
- 项目：[P5 车式移动机器人导航 README](../../01_项目课程/P5_车式移动机器人自主导航/README.md)
- 模块说明：[07_SLAM与建图 README](./README.md)
