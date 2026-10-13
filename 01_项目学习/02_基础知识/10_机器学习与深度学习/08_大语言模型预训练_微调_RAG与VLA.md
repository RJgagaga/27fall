# 大语言模型：预训练、微调、对齐、推理、RAG 与 VLA

> **一句话定位**：[Transformer、LLM 与 Agent](./05_Transformer_LLM与Agent_LangGraph.md) 讲了注意力和 Agent 编排；本章把"一个大模型是怎么来的、怎么用、怎么部署"和"它怎样变成机器人的大脑（VLA）"讲成一条完整的流水线：分词 → 预训练 → 监督微调与参数高效微调（LoRA）→ 偏好对齐（RLHF/DPO）→ 解码与推理成本（KV cache）→ 检索增强（RAG）→ 视觉语言模型（VLM）→ 视觉-语言-动作模型（VLA）。

**本章学完你能：**

- 手算一次 BPE 合并，解释分词对多语言、数字、代码的影响；
- 写出自回归语言模型的目标，换算 loss 与困惑度（perplexity），估算训练 FLOPs（$6ND$）与 GPU 小时；
- 估算参数量（$\approx12Ld^2$）与 KV cache 内存，解释为什么长上下文昂贵；
- 解释 SFT、LoRA（并算参数量占比）、RLHF 与 DPO（手算 DPO 损失）；
- 解释温度、top-k、top-p 采样，手算 softmax 随温度的变化；
- 设计一个最小 RAG（切块、嵌入、余弦相似度检索、拼接提示），并知道失败模式；
- 说明 VLM（视觉编码器 + 投影 + LLM）的结构；
- 区分 VLA 的三类动作表示（离散 token、连续回归、扩散/流匹配头），说出代表性工作的思路与局限；
- 讲清把大模型接入机器人时的安全约束（输出校验、白名单、策略检查）；
- 用英文回答 "How is an LLM trained / what is LoRA / what is RAG / what is a VLA and what are its limitations"。

**前置知识**：[神经网络与反向传播](./02_神经网络与反向传播.md)、[Transformer、LLM 与 Agent](./05_Transformer_LLM与Agent_LangGraph.md)、[序列模型、残差网络、ViT 与生成模型](./07_序列模型_残差网络_ViT与生成模型.md)、[强化学习与具身智能入门](./06_强化学习与具身智能入门.md)。

> **关于事实核对**：本章涉及的具体模型名（RT-2、OpenVLA、$\pi_0$ 等）、数据规模与结构，来自公开论文的大意，**细节未逐条核对**；面试里讲思想和取舍即可，不要背具体数字。

## 目录

1. 流水线总览
2. 分词与 BPE（手算）
3. 预训练：目标、规模与成本
4. 参数量、KV cache 与推理成本（手算）
5. 监督微调与 LoRA（手算）
6. 偏好对齐：RLHF 与 DPO（手算）
7. 解码：温度、top-k、top-p（手算）
8. RAG：检索增强生成
9. 评估、幻觉与局限
10. 量化与部署
11. 多模态：VLM
12. VLA：视觉-语言-动作模型
13. 在机器人上落地：安全与延迟
14. 在本项目里怎么用
15. 常见错误与故障现象
16. 面试怎么问、怎么答
（补充）补充专题：注意力显存、pass@k 与机器人提示词模板
17. 本章要点速记
18. 自测题
19. 相关链接

---

## 1. 流水线总览

```text
 海量文本 ──> [分词] ──> [预训练: 预测下一个token] ──> 基座模型 (base)
                                                         │
                          指令数据 ──> [SFT 监督微调 / LoRA] ──> 指令模型
                                                         │
                          偏好数据 ──> [RLHF / DPO 对齐] ──> 对话/助手模型
                                                         │
        推理: [解码+KV cache] <── 提示 + [RAG检索] / [工具调用] ── 部署(量化)
                                                         │
        图像/动作数据 ──> [VLM / VLA 微调] ──> 机器人策略
```

各阶段的数据、成本、能力逐级递减/变化：预训练昂贵、给出"知识与语言能力"；微调便宜、给出"听指令与格式"；对齐调整"偏好"；推理侧用提示、检索、工具弥补知识过期与幻觉。

## 2. 分词与 BPE（手算）

### 2.1 为什么分词

模型的词表是有限的，不能每个单词一个符号（词汇无限），也不能只用字符（序列太长）。**子词（subword）**折中：高频词整体成一个 token，罕见词拆成片段。常用算法：BPE（Byte Pair Encoding）、WordPiece、Unigram；GPT 类模型常用**字节级 BPE**，任何字符串都可表示。

### 2.2 BPE 训练过程

1. 初始词表 = 单个字符（加词尾标记 `</w>`）；
2. 统计相邻符号对的频次；
3. 合并最高频的一对为新符号；
4. 重复，直到词表达到目标大小。

### 2.3 手算

语料词频：`low`×5，`lower`×2，`newest`×6，`widest`×3。初始：`l o w </w>`、`l o w e r </w>`、`n e w e s t </w>`、`w i d e s t </w>`。

- 第 1 次：`e s` 出现 $6+3=9$ 次（并列最高，取 `es`）→ 合并成 `es`；
- 第 2 次：`es t` 9 次 → `est`；
- 第 3 次：`est </w>` 9 次 → `est</w>`；
- 第 4 次：`l o` 7 次（5+2）→ `lo`；
- 第 5 次：`lo w` 7 次 → `low`。

最终 `newest` → `n e w est</w>`，`widest` → `w i d est</w>`，`low` → `low </w>`，`lower` → `low e r </w>`（Python 复核）。**性质**：词尾 `est</w>` 作为共享的子词被学出来，模型因此能泛化到没见过的 `fastest`。

### 2.4 分词的工程影响

- 数字、代码、中文每 token 的信息量不同：中文一个字常占 1–2 个 token，数字可能被切成不规则片段，使算术变难；
- **token 数决定成本和上下文窗口**，不是字数；
- 提示里的奇怪空格、大小写都会改变分词。

## 3. 预训练：目标、规模与成本

### 3.1 目标

自回归语言模型：给定前文预测下一个 token，最大化
$$\max_\theta\sum_t\log p_\theta(x_t\mid x_{<t}),\qquad L=-\frac1T\sum_t\log p_\theta(x_t\mid x_{<t}).$$
**困惑度**：$\mathrm{PPL}=e^{L}$。含义：模型平均在 PPL 个同样可能的 token 里犹豫。**手算**：$L=2.0$ nats → $\mathrm{PPL}=e^{2}=7.39$。

### 3.2 训练成本估算

训练 FLOPs 约 $6ND$：$N$ 参数，$D$ 训练 token 数（前向 $2ND$ + 反向 $4ND$）。**手算**：$N=7\times10^9$，$D=10^{12}$：$6ND=4.2\times10^{22}$ FLOPs。设单卡峰值 $312$ TFLOPs（bf16）、利用率 40%：有效 $1.25\times10^{14}$ FLOPs/s，所需 $3.36\times10^8$ 秒 $=93{,}000$ GPU 小时（Python 复核）。千卡并行约 3.9 天（理想情况）。这就是为什么预训练只有少数机构做，大多数人是微调与使用。

> **经验性规律**：缩放定律（scaling laws）观察到 loss 随参数、数据、算力呈幂律下降，"在固定算力下参数与数据应按比例增长"（如 Chinchilla 的结论）。具体系数未核对，面试只需知道"规模带来可预测的提升，且数据量要匹配"。

### 3.3 数据

清洗、去重、质量过滤、混合比例（网页、书籍、代码、数学）对结果影响巨大。数据泄漏（测试集混入训练集）会让评估虚高。

## 4. 参数量、KV cache 与推理成本（手算）

### 4.1 参数量估算

每层：注意力 $4d^2$（$W_Q,W_K,W_V,W_O$），MLP 约 $8d^2$（隐层 $4d$，两个矩阵）；共约 $12d^2$。$L$ 层：$N\approx12Ld^2$（加嵌入表 $Vd$）。

**手算**：$L=32$，$d=4096$：$12\times32\times4096^2=6.44\times10^9$（约 6.4B；实际 7B 级模型因 SwiGLU 的 MLP 形状和嵌入略有出入）。

### 4.2 KV cache

自回归生成时，已处理 token 的 $K,V$ 缓存起来，避免重算。每 token 每层存 $2\times n_{heads}\times d_{head}$ 个数。

**手算**：32 层、32 头、头维 128（总宽 4096）、序列 4096、bf16（2 B）：
$$2\ (K,V)\times32\times4096\times4096\times2\text{ B}=2.15\times10^9\text{ B}=2.0\ \text{GiB}\ \text{（每条序列）。}$$
批量 16 条就是 32 GiB。所以：**长上下文、大批量下，KV cache 占的显存常常超过权重**。缓解：多查询/分组查询注意力（MQA/GQA，K、V 头数更少）、KV 量化、分页管理（PagedAttention 思想）。

### 4.3 解码是带宽瓶颈

生成每个 token 需要把所有权重从显存读一遍。**估算**：FP16 权重 14 GB，显存带宽 1 TB/s：上限 $\approx1000/14=71$ token/s（单序列）。权重量化为 INT4（3.5 GB）后上限约 $286$ token/s。边缘设备带宽低得多：带宽 50 GB/s、INT4 7B 模型：$50/3.5\approx14$ token/s。**这对机器人实时控制至关重要**：LLM 的生成速度远低于控制频率（见第 13 节）。

## 5. 监督微调与 LoRA（手算）

### 5.1 SFT

用（指令，回答）对继续训练，损失只在回答 token 上计算（对提示部分 mask）。数据量可以小（几千到几十万条），重质量。目的：学会遵循指令、输出格式。

### 5.2 全参数微调的问题

显存：优化器状态（Adam 需要 2 份）+ 梯度 + 权重，约是权重的 16 B/参数（混合精度常见估算）：7B 模型约 $7\times10^9\times16=112$ GB，单卡放不下。

### 5.3 LoRA

冻结原权重 $W\in\mathbb R^{d\times d}$，只训练低秩增量：
$$W'=W+\frac{\alpha}{r}BA,\quad B\in\mathbb R^{d\times r},\ A\in\mathbb R^{r\times d},\ r\ll d.$$
**手算**：$d=4096$，$r=8$：增量参数 $2\times4096\times8=65{,}536$，对比全矩阵 $4096^2=16{,}777{,}216$，占 $0.39\%$。若只对每层的 $W_Q,W_V$ 加 LoRA，32 层共 $32\times2\times65{,}536=4.19\times10^6$ 参数（Python 复核），约占 7B 的 0.06%。

优点：显存小、可保存多套适配器、推理时可合并 $W'=W+\Delta W$ 无额外延迟。QLoRA 在 4-bit 量化的基座上训练 LoRA，进一步降显存。缺点：表达能力受秩限制；对大幅改变行为（学新语言）可能不足。

### 5.4 其他参数高效方法

Prefix/Prompt tuning（学习虚拟 token）、Adapter（层内小模块）、BitFit（只调偏置）。

## 6. 偏好对齐：RLHF 与 DPO（手算）

### 6.1 为什么需要

SFT 只教会"像样的回答"，不知道人类更偏好哪个。偏好数据：同一提示下的两个回答 $(y_w,y_l)$，人标注 $y_w$ 更好。

### 6.2 RLHF 三步

1. SFT；
2. **奖励模型**：用 Bradley–Terry 目标 $L_{RM}=-\log\sigma\big(r(x,y_w)-r(x,y_l)\big)$ 训练；
3. 用 **PPO** 最大化奖励，同时用 KL 惩罚拉住与参考模型的距离：$\max\ \mathbb E[r(x,y)]-\beta\,\mathrm{KL}(\pi_\theta\|\pi_{ref})$。

**手算** RM 损失：$r_w-r_l=1.5$ → $L=-\log\sigma(1.5)=0.201$；差距 0.3 → $0.554$（Python 复核）。

### 6.3 DPO

直接把"最优策略与奖励的闭式关系"代入偏好损失，**不需要显式奖励模型与 RL**：
$$L_{DPO}=-\log\sigma\Big(\beta\Big[\log\frac{\pi_\theta(y_w|x)}{\pi_{ref}(y_w|x)}-\log\frac{\pi_\theta(y_l|x)}{\pi_{ref}(y_l|x)}\Big]\Big).$$
**手算**：$\beta=0.1$；当前模型相对参考模型，好回答的对数概率比 $+2.0$，坏回答 $-1.0$：括号 $=3.0$，乘 $\beta$ 得 $0.3$，$L=-\log\sigma(0.3)=0.554$。训练推动好回答概率上升、坏回答下降；若已经拉开很多，损失趋近 0。

对比：DPO 简单稳定、省显存；RLHF-PPO 灵活但调参难。近年还有基于可验证奖励的强化学习（如数学/代码的规则奖励）用于推理能力提升，概述性了解即可（细节未核对）。

## 7. 解码：温度、top-k、top-p（手算）

模型输出 logits $z$，概率 $p_i=\dfrac{e^{z_i/T}}{\sum_je^{z_j/T}}$。

**手算** $z=[2.0,1.0,0.1]$：

| 温度 $T$ | $p$ |
|---|---|
| 1 | [0.659, 0.242, 0.099] |
| 0.5 | [0.864, 0.117, 0.019] |
| 2 | [0.502, 0.304, 0.194] |

$T$ 小 → 更确定、更重复；$T$ 大 → 更随机。机器人的**结构化输出**（JSON 指令、动作参数）应使用低温（常取 0 或接近 0）以提高稳定性。

- **top-k**：只在概率最高的 $k$ 个里采样；
- **top-p（nucleus）**：取累积概率达到 $p$ 的最小集合。上例 $T=1$ 的累积：[0.659, 0.901, 1.0]，$p=0.9$ 时保留前两个（累积 0.901 ≥ 0.9）；
- **贪心**：每步取 argmax，可能陷入循环；
- **束搜索（beam search）**：保留多条候选。

**约束解码**（constrained decoding）：用语法/JSON schema 屏蔽非法 token，保证输出格式合法，是 Agent 系统的重要工程手段。

## 8. RAG：检索增强生成

### 8.1 要解决的问题

模型知识有截止时间、可能不含你的私有文档，且会编造。RAG 在回答前先**检索**相关片段，拼进提示。

### 8.2 流水线

1. **离线**：文档 → 切块（chunk，如 200–500 token，带重叠）→ 嵌入模型 → 向量库；
2. **在线**：问题 → 嵌入 → 近邻检索 top-$k$ → （可选重排序 reranker）→ 拼入提示 → LLM 生成并标注来源。

### 8.3 相似度手算

余弦相似度 $\cos=\dfrac{a\cdot b}{\|a\|\|b\|}$。查询向量 $a=(1,2,0)$，文档 $b=(2,3,0)$：$\dfrac{8}{\sqrt5\sqrt{13}}=0.992$；文档 $c=(0,1,3)$：$\dfrac{2}{\sqrt5\sqrt{10}}=0.283$（Python 复核）。检索到 $b$。

### 8.4 失败模式

- 切块不当（切断关键句）→ 检索不到；
- 嵌入模型与领域不匹配；
- 检索到但模型忽略（"lost in the middle"）；
- 过时/冲突文档；
- 提示注入：文档里写着"忽略之前指令"，要把检索内容当数据而非指令；
- 评估：检索召回率、答案忠实度（faithfulness）要分别测。

混合检索（关键词 BM25 + 向量）与重排序常显著提升效果。

## 9. 评估、幻觉与局限

- **幻觉（hallucination）**：模型生成流畅但不真实的内容。缓解：RAG、引用来源、让模型说"不知道"、结构化输出校验、关键事实交给工具（计算器、数据库）；
- **评估**：基准测试（容易被污染）、人评、LLM-as-judge（有偏差）、针对任务的自动校验（单元测试、模拟器）；
- **局限**：不擅长精确算术与长链推理（可用工具或"思维链"缓解）、对提示敏感、不保证确定性、有安全与隐私风险。

## 10. 量化与部署

权重 FP16（2 B）→ INT8（1 B）→ INT4（0.5 B）：7B 模型从 $14$ GB 降到 $7$ GB 再到 $3.5$ GB。方法：训练后量化（PTQ，如 GPTQ/AWQ 一类，细节未核对）、量化感知训练。代价是精度略降。推理引擎（vLLM、llama.cpp 一类）实现连续批处理、KV cache 管理、算子融合。原理与 [模型量化与边缘部署：RKNN](./04_模型量化与边缘部署_RKNN.md) 相通：缩小位宽 → 更少内存带宽 → 更高吞吐。

## 11. 多模态：VLM

典型结构（LLaVA 类思路）：

```text
 图像 ──> 视觉编码器(ViT, 如 CLIP/SigLIP) ──> 视觉特征 ──> 投影层(MLP) ──┐
 文本提示 ──────────────────────────────────> 词嵌入 ───────────────────┼─> LLM ──> 文本
                                                                          │
                           视觉token与文本token拼接成同一序列 ────────────┘
```

训练：先只训投影层对齐（图文对），再指令微调。图像 token 数：$224$ 分辨率 patch 14 → $16\times16=256$ 个 token，每张图占用 256 个上下文位置；高分辨率或多图会很快占满上下文。

## 12. VLA：视觉-语言-动作模型

### 12.1 定义

**VLA（Vision-Language-Action）**：输入相机图像 + 语言指令（+ 机器人本体状态），输出机器人动作（如末端位姿增量 + 夹爪开合，或关节目标）。把预训练 VLM 的语义/常识迁移到控制。

### 12.2 动作怎么表示

| 方式 | 做法 | 代表思路（未核对细节） | 优缺点 |
|---|---|---|---|
| 离散 token | 把每维动作量化为 256 个 bin，当作语言 token 自回归输出 | RT-2、OpenVLA 一类思路 | 直接复用 LLM；量化分辨率有限（范围 $[-1,1]$ 时每 bin 约 $0.0078$），生成慢（每步动作需要 $7\sim8$ 个 token） |
| 连续回归 | 动作头直接回归连续值 | 早期 RT-1 类 Transformer 策略 | 简单；多模态分布被平均 |
| 扩散/流匹配头 | 动作头生成一段动作序列（action chunk） | $\pi_0$ 一类思路（流匹配）、扩散策略 | 能表示多模态、动作平滑、高频；计算更复杂 |

### 12.3 手算：为什么要 action chunk 和专用动作头

若动作是 7 维，每维 1 个 token，7B 模型自回归每 token 约 30 ms（示例）：一次动作 $7\times30=210$ ms → 控制频率约 $4.8$ Hz。操作任务通常需要 10–50 Hz 以上的平滑控制，所以：

- 一次预测一段 **动作块**（如 50 步），执行时滚动更新，摊薄推理延迟；
- 用小的**动作专家**（action expert）并行生成整段动作，而不是逐 token 解码；
- 低层仍由经典控制器（阻抗/PID/IK）以高频执行，VLA 只给 5–10 Hz 的目标。这与本仓库一贯的分层结构一致：**慢速高层决策 + 快速低层控制**（见 [强化学习与具身智能入门](./06_强化学习与具身智能入门.md)）。

### 12.4 数据与训练

- 数据来自遥操作示教（VR/leader-follower）、跨机器人数据集（如 Open X-Embodiment 汇集了多个机构的数据，具体规模未核对）；
- 训练常为：预训练 VLM → 机器人数据上微调（可与互联网视觉语言数据**协同训练**，防止遗忘语义能力）；
- 评估：真机成功率（方差大）、仿真基准。

### 12.5 局限（面试加分项）

- **延迟与算力**：见上；
- **安全性**：没有形式化保证，会输出越界动作，需要外部安全层；
- **泛化**：对新物体/新场景常常好，对新的动力学、接触丰富的精细操作仍弱；
- **数据成本**：真机示教昂贵；
- **可解释性与可复现性**：难调试，评估噪声大；
- **与分层方案的取舍**：VLA 端到端简单但黑箱；"LLM 规划 + 经典运动规划/控制"可解释、可验证，但接口设计复杂。

## 13. 在机器人上落地：安全与延迟

这是与 P6 最直接的联系：大模型输出**不可信**，必须经过确定性的检查才能执行。

```text
 自然语言/观测 ──> [LLM/VLA: 提出动作 action proposal]
                       │
                       ▼
              [策略检查 policy check]  <── 白名单、参数范围、安全规则、当前状态
                       │ 通过
                       ▼
              [执行 execution] ──> 底层控制器(限幅、看门狗)
                       │
                       ▼
              [状态反馈 state feedback] ──> 回到决策层/重规划
```

要点：

1. **只允许结构化的、有限集合的动作**（函数调用/JSON），用 schema 校验；
2. **参数边界**：速度、力、工作空间限幅；
3. **状态前置条件**：只有处于允许状态才执行（例如急停未触发、机械臂已回零）；
4. **执行后验证**：对照反馈确认动作达到，失败则回退/重试/上报；
5. **永不让模型直接写电机命令**；
6. **延迟预算**：LLM 决策秒级、规划百毫秒级、控制毫秒级，各层频率不同，不能互相阻塞（见 [时间同步、坐标系一致性与数据有效性](../06_状态估计/06_时间同步_坐标系一致性与数据有效性.md)）。

## 14. 在本项目里怎么用

- **P4 智能园艺**：基于 LangGraph 的 Agent 与规则快速路径（见 [Transformer、LLM 与 Agent](./05_Transformer_LLM与Agent_LangGraph.md)）。本章的解码低温、约束解码、输出校验、RAG 失败模式可直接用于解释该系统的设计（"为什么先走规则、再走 LLM""为什么要白名单"）。
- **P6 深开鸿**：CV 写的"action-proposal、policy-check、execution、state-feedback 链"正对应第 13 节；面试时可把它与"VLA/LLM 控制机器人必须有的安全层"自然联系起来。
- **P2 遥操作**：VR 遥操作与 leader–follower 采集的数据，是训练 VLA/模仿学习策略的标准数据来源（见 [模仿学习](./10_模仿学习_行为克隆_DAgger_扩散策略.md)）。CV 没有写训练 VLA，不可说成已做。
- **申请叙事**：对"AI + 机器人"方向的项目，可以讲"我动手做过的是控制/估计/规划/平台，理解并关注 VLA 如何与这条经典栈对接"。

## 15. 常见错误与故障现象

| 现象 | 原因 | 处理 |
|---|---|---|
| LLM 输出的 JSON 偶尔非法 | 无约束解码 | 约束解码/schema 校验/重试 |
| 同一输入输出不同 | 采样温度 > 0 | 控制指令用低温，保留日志 |
| 长对话后显存爆 | KV cache 增长 | GQA、截断/摘要上下文、KV 量化 |
| RAG 回答与文档不符 | 检索错误或模型忽略上下文 | 评估检索，要求引用，重排序 |
| 微调后丢失通用能力 | 灾难性遗忘 | 混入通用数据，LoRA，小学习率 |
| VLA 动作抖动/越界 | 无平滑与限幅，量化粗 | 动作块平滑，外层限幅 |
| 推理太慢无法控制 | 自回归逐 token | 动作头并行生成、动作块、分层 |
| 提示注入改变行为 | 把外部文本当指令 | 隔离数据与指令，策略检查 |

## 16. 面试怎么问、怎么答

**Q1. How is a large language model trained?**

> In three broad stages. First, pre-training on a huge text corpus to predict the next token, which gives general language and knowledge. Second, supervised fine-tuning on instruction-response pairs, often with parameter-efficient methods like LoRA. Third, preference alignment, for example RLHF with a reward model or the simpler DPO, to match human preferences. At inference time, I use sampling controls, retrieval and tools to improve reliability.

中文：三阶段：预训练（预测下一个 token）、监督微调（常用 LoRA）、偏好对齐（RLHF 或 DPO）；推理时用采样控制、检索和工具提升可靠性。

**Q2. What is LoRA and why is it useful?**

> LoRA freezes the pre-trained weight and learns a low-rank update B times A, with rank r much smaller than the dimension. For d equal to 4096 and r equal to 8 that is about 0.4 percent of the parameters of one matrix. It cuts memory, lets me keep several adapters for one base model, and the update can be merged into the weights for inference with no extra latency.

中文：冻结原权重，学习低秩更新 $BA$；$d=4096,r=8$ 时约占一个矩阵参数的 0.4%；显存小、可多适配器、可合并无额外延迟。

**Q3. What is RAG and when does it fail?**

> Retrieval-augmented generation first retrieves relevant chunks of documents using embeddings and cosine similarity, then puts them in the prompt so the model answers from them. It fails when chunking cuts the key information, when the retriever returns irrelevant text, when the model ignores the context, or when the documents contain injected instructions. I evaluate retrieval and answer faithfulness separately.

中文：先用嵌入+余弦相似度检索文档块，再拼进提示；失败于切块不当、检索不准、模型忽略上下文、文档含注入指令；检索与忠实度要分别评估。

**Q4. What is a VLA model and what are its limitations?**

> A vision-language-action model takes camera images and a language instruction and outputs robot actions, reusing the semantic knowledge of a pre-trained vision-language model. Actions are either discretised into tokens, or generated by a diffusion or flow-matching head as action chunks. Limitations are inference latency, lack of safety guarantees, data cost, and weak performance on precise contact-rich tasks, so I would keep a classical low-level controller and a safety layer around it.

中文：输入图像+语言，输出动作，复用 VLM 的语义知识；动作表示为离散 token 或扩散/流匹配头的动作块；局限：延迟、无安全保证、数据成本、精细接触任务弱；所以保留经典底层控制器和安全层。

**Q5. How would you safely connect an LLM to a robot?**

> The model only proposes structured actions from a whitelist. A deterministic policy check validates the schema, parameter ranges and the robot's current state. Only then does the executor run it through the normal controller with limits and a watchdog, and the state feedback is checked against the expected outcome. The model never writes motor commands directly. This is the same pattern as the action-proposal, policy-check, execution and state-feedback chain in my platform work.

中文：模型只提出白名单内的结构化动作；确定性策略检查校验 schema、参数范围和当前状态；通过后走常规控制器（限幅+看门狗）；反馈与预期对照；模型不直接写电机命令。对应我平台工作中的 proposal–policy–execution–feedback 链。

## 补充专题：注意力显存、pass@k 与机器人提示词模板

### 注意力矩阵的显存与 FlashAttention 的动机

朴素注意力要显式存储 $N\times N$ 的得分矩阵。**手算**：序列 $N=4096$，32 个头，fp16（2 字节）：$4096^2\times2\times32=1.07\times10^9$ 字节 $\approx1.0$ GiB，每层！（Python 复核）若 32 层同时保存用于反向，显存立刻爆。FlashAttention 一类的实现把计算分块放进 GPU 的片上 SRAM，边算边合并 softmax，**不物化完整的 $N\times N$ 矩阵**，把显存从 $O(N^2)$ 降为 $O(N)$，并减少对显存带宽的访问（底层细节未核对）。意义：长上下文成为可能，推理和训练都更快。这也说明"算得对"和"算得快"在大模型时代是两个独立的工程问题。

### pass@k：评估生成式模型的概率

代码生成等任务，对每个题生成 $n$ 个答案，其中 $c$ 个正确，则"尝试 $k$ 次至少一次对"的无偏估计
$$\text{pass@}k=1-\frac{\binom{n-c}{k}}{\binom nk}.$$
**手算**：$n=10$，$c=3$。$k=1$：$1-\binom71/\binom{10}1=1-0.7=0.30$；$k=5$：$1-\binom75/\binom{10}5=1-21/252=0.917$（Python 复核）。意义：同一模型多采样几次，成功率明显提高；对机器人，对应"规划器生成多个候选，由校验器选出可执行的"策略。这也提醒：报告大模型能力时要说清**采样次数和选择机制**，否则数字不可比。

### 给机器人用的提示词与输出约束（教学示例）

下面是一个把自然语言转成结构化指令的最小模板，思想对应 P4/P6 的"提议—检查—执行"：

```text
系统提示：你是机器人任务规划器。只能输出一个 JSON 对象，不能输出其他文字。
允许的动作：move_to(location), pick(object), place(location), stop()
location 取值只能来自 ["shelf_a", "shelf_b", "table"]
object 取值只能来自 ["cup", "box"]
如果指令无法用上述动作完成，输出 {"action": "refuse", "reason": "..."}
用户指令：把杯子放到桌上
期望输出：{"action": "pick", "object": "cup"}   （一次只给下一步动作）
```

要点：(1) **闭集合**取值，避免模型发明不存在的位置；(2) 一次只输出下一步，便于每步校验与重规划；(3) 有"拒绝"出口，避免硬编造；(4) 温度设低；(5) 输出后用 JSON schema 校验，不合格则重试或回退到规则；(6) 记录输入输出日志，便于复盘。这种设计把 LLM 当成**不可信的建议者**，可靠性由确定性的检查层保证。

### 上下文窗口的管理

长任务中，把全部历史塞进提示既昂贵（KV cache）又会让模型"迷失在中间"。常用策略：保留系统提示与最近 $n$ 轮，其余做摘要；把需要精确保留的状态（任务队列、机器人当前位置）放进结构化"状态块"，每轮由程序重新写入，而不是靠模型记忆。

### 推理服务的吞吐与延迟

在线服务指标：首 token 延迟（TTFT，取决于提示长度，预填充阶段算力受限）、每 token 延迟（TPOT，解码阶段带宽受限）、吞吐（token/s）。**连续批处理**把不同请求的解码步合并，提升 GPU 利用率；缺点是单请求延迟可能增大。机器人场景一般请求量小、延迟敏感，往往本地小模型 + 量化比云端大模型更合适，同时避免网络抖动（需要权衡能力与延迟）。

## 17. 本章要点速记

1. 流水线：分词 → 预训练 → SFT/LoRA → 对齐 → 推理（解码 + KV cache + RAG/工具）。
2. BPE：反复合并最高频相邻符号对；token 数决定成本。
3. 预训练目标：最小化 $-\log p(x_t|x_{<t})$；PPL $=e^{L}$。
4. 训练 FLOPs $\approx6ND$；参数量 $\approx12Ld^2$。
5. KV cache：$2\times L\times T\times d\times\text{字节数}$（每序列）；解码是带宽瓶颈。
6. LoRA：$W+\tfrac\alpha rBA$，$d=4096,r=8$ 占 0.39%。
7. RLHF：RM（Bradley–Terry）+ PPO + KL；DPO 直接用偏好对损失。
8. 温度低更确定；结构化输出用低温 + 约束解码。
9. RAG：切块 → 嵌入 → 余弦检索 → 拼提示；失败模式要逐一评估。
10. VLA：图像+语言 → 动作；离散 token / 回归 / 扩散与流匹配头；动作块缓解延迟。
11. 安全：提议 → 策略检查 → 执行 → 反馈；模型不直写电机命令。

## 18. 自测题

1. PPL 为 10 对应的平均 loss？（$\ln10=2.30$ nats。）
2. 13B 模型 FP16 权重内存？INT4？（26 GB；6.5 GB。）
3. $d=2048$，$r=16$ 的 LoRA 增量参数 vs 全矩阵占比？（$2\times2048\times16=65{,}536$；全矩阵 $4{,}194{,}304$；占 $1.56\%$。）
4. 为什么 DPO 里要有参考模型？（约束新策略不要偏离初始模型太远，对应 RLHF 的 KL 项。）
5. 动作范围 $[-0.5,0.5]$ m，256 个 bin，分辨率？（$1/256=0.0039$ m $\approx3.9$ mm。）
6. 为什么 LLM 不能直接输出电机力矩指令？（无保证、延迟高、不确定、可被注入；需确定性检查与底层限幅。）

## 19. 相关链接

- 上一章：[序列模型、残差网络、ViT 与生成模型](./07_序列模型_残差网络_ViT与生成模型.md)
- 下一章：[强化学习进阶：策略梯度、PPO、SAC](./09_强化学习进阶_策略梯度_PPO_SAC.md)、[模仿学习](./10_模仿学习_行为克隆_DAgger_扩散策略.md)
- 相关：[Transformer、LLM 与 Agent](./05_Transformer_LLM与Agent_LangGraph.md)、[模型量化与边缘部署：RKNN](./04_模型量化与边缘部署_RKNN.md)
- 八股速查：[Transformer 与大模型八股](../../03_八股知识/02_人工智能/04_Transformer与大模型八股.md)、[Agent 与 RAG 八股](../../03_八股知识/02_人工智能/07_Agent与RAG八股.md)
- 模块说明：[10_机器学习与深度学习 README](./README.md)
