# Transformer 与大模型八股（Transformer & LLM Interview Q&A）

> **定位**：覆盖 Self-attention 推导、多头、位置编码、归一化与结构、KV cache、预训练目标、SFT/RLHF/DPO、LoRA、幻觉、上下文长度、Prompt、评估、推理优化，以及 ViT、CLIP、多模态与具身 VLA 简述。是 AI/具身智能面试的核心章节。

**本章学完你能：**
- 在白板上推导 scaled dot-product attention，解释 $\sqrt{d_k}$、多头、因果掩码、复杂度；
- 比较正弦位置编码、RoPE、ALiBi；比较 Post-LN/Pre-LN 与 RMSNorm；
- 讲清 KV cache 的显存公式和 MQA/GQA/FlashAttention 的作用；
- 说明预训练 → SFT → RLHF/DPO 的流水线，以及 LoRA 的数学与参数量；
- 谈论幻觉、长上下文、评估与推理优化，并简要介绍 ViT、CLIP、VLA。

**前置知识**：[05_Transformer_LLM与Agent_LangGraph](../../02_基础知识/07_感知与机器学习/05_Transformer_LLM与Agent_LangGraph.md)、[02_深度学习八股](./02_深度学习八股.md)。

**优先级**：★★★ 必背，★★ 常问，★ 加分。涉及具体论文结论或数字，处理原则：只写我确定的经典结论，细节标「未核对」。

## 目录
1. 注意力机制（Q1–Q8）
2. 位置编码与结构细节（Q9–Q16）
3. 预训练与扩展（Q17–Q22）
4. 解码与推理（Q23–Q30）
5. 微调与对齐（Q31–Q39）
6. 幻觉、上下文、Prompt、评估（Q40–Q47）
7. 视觉与多模态、VLA（Q48–Q54）
8. 速记、自测、链接

---

## 一、注意力机制

### Q1. 为什么需要注意力 / Transformer 相比 RNN 的优势 ★★★
**答案要点**
RNN 的问题：时间上串行不能并行；长距离依赖路径长（信息要经过 $O(T)$ 步传递，梯度易消失）；固定隐状态瓶颈。Transformer 用自注意力让任意两位置直接交互（路径长度 $O(1)$），整层可并行计算（矩阵乘），训练更快且易扩展。代价：自注意力对序列长度 $n$ 是 $O(n^2)$ 时间与内存。

**English short answer**
> RNNs process tokens sequentially, so they can't be parallelized, and information between distant tokens passes through many steps. Self-attention connects every pair of positions directly and is computed with matrix multiplications, so it parallelizes well and handles long-range dependencies. The cost is quadratic complexity in the sequence length.

**追问方向**：为什么说 Transformer 是"硬件友好"的？

### Q2. Scaled Dot-Product Attention 的推导 ★★★
**答案要点**
输入 $X\in\mathbb R^{n\times d}$，线性投影得查询、键、值：$Q=XW_Q,\ K=XW_K,\ V=XW_V$，$Q,K\in\mathbb R^{n\times d_k}$，$V\in\mathbb R^{n\times d_v}$。
$$\mathrm{Attention}(Q,K,V)=\mathrm{softmax}\Big(\frac{QK^T}{\sqrt{d_k}}\Big)V$$
直观：$QK^T$ 的 $(i,j)$ 元素是第 $i$ 个位置的 query 与第 $j$ 个位置的 key 的相似度；softmax（按行）把分数变为权重；输出是 value 的加权和（软检索）。
手算（教学示例）：两个 token，$d_k=2$，$q_1=(1,0)$，$k_1=(1,0),k_2=(0,1)$，$v_1=(1,2),v_2=(3,4)$。分数 $(1,0)/\sqrt2=(0.707,0)$，softmax $=(e^{0.707},1)/(e^{0.707}+1)=(0.669,0.331)$，输出 $0.669(1,2)+0.331(3,4)\approx(1.66,2.66)$。

**English short answer**
> Each token is projected into a query, a key and a value. The dot product of a query with all keys gives similarity scores, which I scale by the square root of the key dimension, normalize with softmax across keys, and use as weights to average the values. So it's a soft, differentiable lookup where each token gathers information from the others.

**追问方向**：Q、K、V 为什么用不同的投影而不是同一个？

### Q3. 为什么要除以 $\sqrt{d_k}$？ ★★★
**答案要点**
设 $q,k$ 各分量独立、均值 0、方差 1，则 $q\cdot k=\sum_{i=1}^{d_k}q_ik_i$ 均值 0、方差 $d_k$（标准差 $\sqrt{d_k}$）。$d_k$ 大时分数幅度大，softmax 进入饱和区，输出接近 one-hot，梯度 $\partial p/\partial z=p(1-p)\to0$，训练困难。除以 $\sqrt{d_k}$ 使分数方差回到 1，保持 softmax 梯度健康。

**English short answer**
> If query and key components have unit variance, their dot product has variance equal to the key dimension. Large scores push softmax into saturation, where the gradients are almost zero. Dividing by the square root of the dimension brings the variance back to one and keeps training stable.

**追问方向**：用加性注意力会需要缩放吗？

### Q4. 多头注意力（Multi-Head Attention）★★★
**答案要点**
把 $d_{model}$ 分成 $h$ 个头，每头 $d_k=d_{model}/h$，各自有独立的 $W_Q^i,W_K^i,W_V^i$，并行做注意力后拼接，再乘 $W_O$：
$$\mathrm{MHA}=\mathrm{Concat}(\mathrm{head}_1,\dots,\mathrm{head}_h)W_O$$
作用：让模型在不同子空间关注不同类型的关系（句法、指代、位置邻近等），总计算量与单头相当。参数量：$4d_{model}^2$（$W_Q,W_K,W_V,W_O$）。经验观察：不少头可剪枝，不同头功能有冗余。

**English short answer**
> Multi-head attention splits the model dimension into several heads, each with its own projections, runs attention in parallel, and concatenates the results with an output projection. Different heads can focus on different kinds of relations in different subspaces, at roughly the same compute as a single full-width head.

**追问方向**：头数增加对参数量有影响吗？（总参数不变，头数只是切分维度）

### Q5. 因果掩码与填充掩码 ★★★
**答案要点**
解码器自回归，位置 $i$ 不能看见 $j>i$：在 softmax 之前给上三角位置加 $-\infty$（实际用很大负数），softmax 后权重为 0。`Mask` 矩阵 $M_{ij}=0$（$j\le i$）或 $-\infty$。Padding mask 屏蔽补齐位置。训练时一次前向并行算出所有位置的下一词预测（teacher forcing），推理时逐词生成。

**English short answer**
> For autoregressive models I add a mask of minus infinity to the upper triangle of the score matrix before softmax, so each token can only attend to itself and earlier tokens. That allows training to compute the next-token loss for all positions in parallel, while generation still happens one token at a time.

**追问方向**：Encoder 为什么不需要因果掩码？Prefix-LM 是什么？

### Q6. 自注意力的复杂度与优化 ★★★
**答案要点**
时间 $O(n^2d)$，注意力矩阵内存 $O(n^2)$；FFN 是 $O(nd^2)$。当 $n<d$ 时 FFN 占主导，长序列时注意力占主导。优化方向：稀疏/局部注意力（Longformer 滑窗）、线性注意力、低秩近似、FlashAttention（IO 感知，精确注意力，通过分块 tiling 与在线 softmax 避免实体化 $n\times n$ 矩阵，显存 $O(n)$，速度提升来自减少 HBM 读写）、MQA/GQA（缩小 KV）。

**English short answer**
> Attention costs quadratic time and memory in sequence length, while the feed-forward part is linear in length but quadratic in width. For long sequences attention dominates. FlashAttention keeps the exact result but tiles the computation to avoid writing the full attention matrix to GPU memory, which makes it faster and lighter. Sparse and linear attention approximate it.

**追问方向**：FlashAttention 的在线 softmax 怎么做？

### Q7. 交叉注意力与自注意力 ★★
**答案要点**
自注意力：Q、K、V 来自同一序列；交叉注意力：Q 来自解码器，K、V 来自编码器输出（或另一个模态，如图像特征）。多模态模型中文本 query 读取视觉特征即交叉注意力；Stable Diffusion 的文本条件注入同理。

**English short answer**
> In self-attention, queries, keys and values come from the same sequence. In cross-attention, queries come from one sequence, such as the decoder, and keys and values come from another, such as the encoder output or image features. It's how conditioning from another modality is injected.

**追问方向**：Encoder-Decoder 与 Decoder-only 的取舍？

### Q8. 注意力的可解释性与注意力汇聚（attention sink）★
**答案要点**
注意力权重不等于因果解释；常观察到很多头把大量权重放在第一个 token（attention sink），原因被认为是 softmax 必须分配总和为 1 的权重，模型用它做"空操作"。这被用于流式长文本推理（保留 sink token + 滑窗 KV）。此处细节属于近期研究现象，面试时用"观察到"措辞，不要说成定论。

**English short answer**
> Attention weights aren't a faithful explanation of the model's reasoning. Many heads put a lot of weight on the first token, which seems to act as a no-op sink because softmax must sum to one. This observation is used in streaming inference by keeping the first tokens plus a sliding window.

**追问方向**：为什么 softmax 的归一化约束会产生这种现象？

---

## 二、位置编码与结构细节

### Q9. 为什么需要位置编码？ ★★★
**答案要点**
自注意力是置换等变的：打乱输入顺序，输出同样打乱，模型本身不知道顺序。需要把位置信息注入。方式：绝对位置编码（加到输入嵌入上）、相对位置编码（注意力分数中引入相对距离）、旋转位置编码 RoPE、线性偏置 ALiBi。

**English short answer**
> Self-attention has no notion of order, since permuting the tokens just permutes the outputs. So we inject position information, either as absolute encodings added to the embeddings, or as relative encodings inside the attention scores, like RoPE or ALiBi.

**追问方向**：去掉位置编码的因果 LM 能否隐式学到位置？（因果掩码本身泄露了顺序信息，有研究观察到可以部分学到）

### Q10. 正弦位置编码 ★★★
**答案要点**
$$PE_{(pos,2i)}=\sin\Big(\frac{pos}{10000^{2i/d}}\Big),\quad PE_{(pos,2i+1)}=\cos\Big(\frac{pos}{10000^{2i/d}}\Big)$$
不同维度对应不同波长（从 $2\pi$ 到 $10000\cdot2\pi$）。性质：$PE_{pos+k}$ 可表示为 $PE_{pos}$ 的线性函数（三角恒等式 $\sin(a+b)=\sin a\cos b+\cos a\sin b$），便于学习相对位置；无需训练参数，理论上可外推（实际外推效果有限）。BERT/GPT-2 用可学习位置嵌入，长度受限于训练上限。

**English short answer**
> Sinusoidal encoding assigns each position a vector of sines and cosines at different frequencies. Because of trigonometric identities, the encoding of a shifted position is a linear function of the original, which makes relative offsets easy to learn. It has no parameters, but in practice it doesn't extrapolate well to much longer sequences.

**追问方向**：可学习位置嵌入 vs 正弦的优缺点？

### Q11. RoPE（旋转位置编码）★★★
**答案要点**
思想：对 query 和 key 按位置施加旋转，使内积只依赖相对位置。把向量两两分组视为复平面上的点，位置 $m$ 处第 $i$ 组旋转角 $m\theta_i$，$\theta_i=10000^{-2i/d}$：
$$f(q,m)=R_m q,\qquad \langle R_mq,R_nk\rangle=q^TR_m^TR_nk=q^TR_{n-m}k$$
因为旋转矩阵满足 $R_m^TR_n=R_{n-m}$，所以注意力分数只与相对距离 $n-m$ 有关。优点：无需额外参数、天然相对位置、随距离有衰减趋势、易做长度外推（位置插值 PI、NTK-aware、YaRN 等通过缩放角频率扩展上下文）。LLaMA 系列广泛使用。

**English short answer**
> RoPE rotates pairs of dimensions in the query and key by an angle proportional to the token position. Since rotations compose, the dot product between a query at position m and a key at n depends only on their relative offset. It adds no parameters, encodes relative position naturally, and context extension methods work by rescaling the rotation frequencies.

**追问方向**：位置插值为什么能扩展上下文？高频维度的作用？

### Q12. ALiBi 与相对位置偏置 ★
**答案要点**
ALiBi 不改嵌入，直接在注意力分数上加 $-m_h\cdot|i-j|$（$m_h$ 为每个头固定斜率），距离越远惩罚越大，外推到更长序列效果较好。T5 使用可学习的相对位置偏置（按距离分桶）。

**English short answer**
> ALiBi adds a penalty proportional to the distance between tokens directly to the attention scores, with a fixed slope per head. It extrapolates to longer sequences than it was trained on better than absolute embeddings. T5 uses learned relative position biases.

**追问方向**：RoPE 与 ALiBi 的外推差别？

### Q13. Transformer 块的完整结构与参数量 ★★★
**答案要点**
一个 Decoder 块：Pre-LN → 多头自注意力 → 残差；Pre-LN → FFN → 残差。FFN：$W_2\,\phi(W_1x)$，$W_1\in\mathbb R^{d\times 4d}$（隐层通常 4 倍，LLaMA 用 SwiGLU 并把中间维度调到 $\approx\frac83d$ 以保持参数量）。每层参数量约 $4d^2$（注意力）$+8d^2$（FFN）$=12d^2$。$L$ 层总参数约 $12Ld^2$（加嵌入 $Vd$）。示例（教学示例）：$d=4096,L=32$ 时 $12\times32\times4096^2\approx6.4\times10^9$，加词表嵌入后在 7B 量级。

**English short answer**
> Each block has self-attention and a feed-forward network, each wrapped with normalization and a residual connection. The attention has about four d squared parameters, and the FFN with a four-times expansion has about eight d squared, so a model has roughly twelve times layers times d squared parameters, plus the embedding matrix.

**追问方向**：为什么 FFN 被认为存储"知识"？

### Q14. LayerNorm、RMSNorm 与 Pre-LN 的选择 ★★★
**答案要点**
LN：$\frac{x-\mu}{\sqrt{\sigma^2+\epsilon}}\gamma+\beta$。RMSNorm：$\frac{x}{\sqrt{\frac1d\sum x_i^2+\epsilon}}\gamma$，去掉均值中心化和偏置，计算更省，效果相当（LLaMA 采用）。Pre-LN 更稳定（见 [02_深度学习八股](./02_深度学习八股.md) Q16）。

**English short answer**
> RMSNorm drops the mean subtraction and bias from layer norm, dividing only by the root mean square, which is cheaper and works about as well. Most modern LLMs use it in a pre-norm position because it trains more stably at depth.

**追问方向**：为什么大模型更偏好 Pre-LN？

### Q15. Encoder-only / Decoder-only / Encoder-Decoder ★★★
**答案要点**
| 类型 | 代表 | 注意力 | 典型任务 |
|---|---|---|---|
| Encoder-only | BERT | 双向 | 分类、抽取、检索嵌入 |
| Decoder-only | GPT、LLaMA | 因果 | 生成、对话、通用 LLM |
| Encoder-Decoder | T5、BART | 编码双向 + 解码因果+交叉 | 翻译、摘要 |
当前 LLM 主流 Decoder-only：训练目标统一（预测下一词）、数据利用率高、规模化后涌现通用能力、推理有 KV cache 优势。

**English short answer**
> Encoder-only models like BERT read text bidirectionally and are good for understanding tasks and embeddings. Decoder-only models like GPT are causal and generate text. Encoder-decoder models like T5 suit sequence-to-sequence tasks. Today's LLMs are mostly decoder-only because the next-token objective is simple, scales well, and works for almost any task via prompting.

**追问方向**：为什么 decoder-only 能做理解任务？

### Q16. 词表与分词（BPE）★★
**答案要点**
BPE（Byte Pair Encoding）：从字符/字节开始，反复合并语料中最频繁的相邻符号对，直到词表达标。Byte-level BPE 可表示任何输入，无 OOV。词表大小权衡：大 → 序列短、嵌入矩阵大；小 → 序列长。分词影响：数字、代码、低资源语言切得碎，影响算术与成本；中文按字/词混合切分，token 数与字数不等。

**English short answer**
> BPE builds a vocabulary by repeatedly merging the most frequent adjacent pair of symbols. Byte-level BPE can represent any text, so there are no unknown tokens. Vocabulary size is a trade-off between sequence length and embedding size, and tokenization quirks affect arithmetic, code and non-English languages.

**追问方向**：为什么 LLM 数不清 "strawberry" 里的 r？（token 化后看不见字符）

---

## 三、预训练与扩展

### Q17. 预训练目标：CLM、MLM、Span Corruption ★★★
**答案要点**
- 因果语言建模（CLM）：$\max\sum_t\log p(x_t|x_{<t})$，即交叉熵；GPT 系列；
- 掩码语言建模（MLM）：随机遮盖 15% token 预测被遮盖的，双向上下文；BERT（另有 NSP，后被证明作用有限）；
- Span corruption：T5，遮盖连续片段，用哨兵 token 表示；
- 此外：Prefix LM、UL2 混合目标。
困惑度 $PPL=\exp\big(-\frac1N\sum\log p(x_t|x_{<t})\big)$。

**English short answer**
> GPT-style models use causal language modeling: predict the next token given all previous ones, trained with cross-entropy. BERT uses masked language modeling, predicting randomly masked tokens from bidirectional context. T5 masks spans and reconstructs them. Perplexity is the exponential of the average negative log-likelihood.

**追问方向**：BERT 的 80/10/10 掩码策略为何这样设计？

### Q18. Scaling Law 的直觉 ★★
**答案要点**
经验规律：在足够数据和算力下，测试损失随参数量 $N$、数据量 $D$、算力 $C$ 呈幂律下降，如 $L(N)\approx(N_c/N)^{\alpha}$。Chinchilla 的结论是在固定算力下，模型参数与训练 token 数应近似同比例增长（大约每参数 20 个 token 的量级，这是常被引用的经验结论，面试时可说"roughly"并标未核对精确数值）；因此许多早期大模型"训练不足"。实践中为降低推理成本，会选择小模型多训数据（over-train）。

**English short answer**
> Empirically the loss falls as a power law with model size, data and compute. The Chinchilla analysis suggested that for a fixed compute budget, parameters and training tokens should grow together, roughly twenty tokens per parameter. In practice people often train smaller models on far more data to make inference cheaper.

**追问方向**：Scaling law 的局限？（外推、数据质量、下游任务非平滑）

### Q19. 预训练数据与训练稳定性 ★★
**答案要点**
数据：清洗、去重（MinHash）、质量过滤、领域配比、去污染（避免评测泄漏）、隐私与版权。训练稳定性：loss spike → 降低学习率、梯度裁剪、跳过坏 batch、BF16、QK-norm、Pre-LN、检查点回滚。优化器 AdamW，cosine 衰减 + warmup。

**English short answer**
> Data quality matters a lot: deduplication, quality filtering, domain mixing and removing benchmark contamination. For stability, large runs use pre-norm, bf16, gradient clipping, warmup, and can roll back from checkpoints when the loss spikes.

**追问方向**：数据去重为什么重要？

### Q20. 混合专家（MoE）★★
**答案要点**
把 FFN 替换为 $E$ 个专家 + 路由器（gating），每个 token 只激活 top-$k$ 个（如 2/8）专家，总参数大但每 token 计算量近似稠密小模型。问题：负载不均衡（加辅助均衡损失）、通信开销（专家并行）、显存仍需装下全部专家、训练不稳定。优点：同等计算下容量更大。

**English short answer**
> A mixture of experts replaces the dense feed-forward layer with many expert networks and a router that sends each token to only a few of them. So the total parameter count is large, but compute per token stays low. It needs load-balancing losses and costs memory and communication.

**追问方向**：路由崩塌（所有 token 选同一专家）如何避免？

### Q21. 模型结构的现代化改动（LLaMA 类）★★
**答案要点**
RMSNorm + Pre-LN、SwiGLU 激活 $\mathrm{SwiGLU}(x)=(\mathrm{SiLU}(xW_1)\odot xW_3)W_2$、RoPE、去掉偏置、GQA、更大词表。这些组合已成为开源 LLM 的事实标准结构。

**English short answer**
> A typical modern open LLM block uses RMSNorm in pre-norm position, SwiGLU feed-forward layers, rotary position embeddings, no bias terms, and grouped-query attention.

**追问方向**：门控 FFN 为什么更好？

### Q22. 词嵌入、权重共享与 logits ★
**答案要点**
输入嵌入矩阵 $E\in\mathbb R^{V\times d}$；输出层 $W_{out}\in\mathbb R^{d\times V}$ 常与 $E^T$ 共享（权重绑定）省参数。logits $=hW_{out}$，softmax 得下一词分布。大词表时输出层与 softmax 计算/显存显著，训练时可分块计算损失。

**English short answer**
> The input embedding maps token IDs to vectors, and the output projection to vocabulary logits is often tied to the same matrix to save parameters. With a large vocabulary the final softmax layer is a notable cost in memory and compute.

**追问方向**：为什么能共享？

---

## 四、解码与推理

### Q23. 自回归解码与 KV Cache ★★★
**答案要点**
生成第 $t$ 个 token 时，注意力需要所有历史 token 的 K、V。每步重新计算全部历史的 K、V 是浪费：历史 token 的 K、V 不变，只需缓存，新 token 只算自己的 $q,k,v$，把 $k,v$ 追加到缓存，然后 $q$ 与全部缓存 $K$ 做注意力。复杂度从每步 $O(t^2)$（含重算）降为 $O(t)$。
显存：
$$M_{KV}=2\times L\times n_{kv}\times d_{head}\times T\times B\times \text{bytes}$$
其中 2 表示 K 和 V，$L$ 层数，$n_{kv}$ KV 头数，$T$ 序列长度，$B$ batch。示例（教学示例）：$L=32,n_{kv}=32,d_{head}=128,T=4096,B=1$，FP16（2 字节）：$2\times32\times32\times128\times4096\times2\approx2.1\text{ GB}$。长上下文、大 batch 下 KV cache 往往成为显存瓶颈，推理阶段分为预填充（prefill，并行处理 prompt，算力受限）和解码（decode，逐 token，访存受限）。

**English short answer**
> During generation, the keys and values of earlier tokens never change, so we cache them. Each new step only computes the query, key and value for the new token, appends the key and value to the cache, and attends to the cache. This avoids recomputation, but the cache grows with sequence length, batch size and layers, and often dominates memory. Prefill is compute-bound, while decode is memory-bandwidth-bound.

**追问方向**：为什么 decode 是访存瓶颈？如何估算 KV 显存？

### Q24. MQA 与 GQA ★★★
**答案要点**
MHA：每个头独立的 K、V；MQA：所有 query 头共享 1 组 K、V，KV cache 缩小 $h$ 倍，质量略降；GQA：query 头分成 $g$ 组，每组共享一组 K、V，在质量与显存间折中（LLaMA-2 70B、LLaMA-3 等采用）。MLA（DeepSeek）用低秩压缩 KV 进一步缩小缓存，属于更新近的做法。

**English short answer**
> In multi-query attention, all query heads share one key-value head, which shrinks the KV cache a lot but can hurt quality. Grouped-query attention shares a key-value head within each group of query heads, which is a good compromise and widely used in recent open models.

**追问方向**：为什么减少 KV 头数主要加速 decode？

### Q25. 解码策略：Greedy、Beam、Top-k、Top-p、温度 ★★★
**答案要点**
- Greedy：每步取最大概率，确定但易重复；
- Beam search：保留 $b$ 条最优部分序列，适合翻译等有"正确答案"的任务，开放生成易乏味；
- 温度 $T$：$p_i\propto e^{z_i/T}$，$T<1$ 更尖锐，$T>1$ 更平；
- Top-k：只在概率前 $k$ 个里采样；
- Top-p（nucleus）：取累计概率达 $p$ 的最小集合再采样，集合大小自适应；
- 重复惩罚、频率/存在惩罚缓解重复。
需要事实性/结构化输出（如工具调用 JSON）时用低温度或 greedy；创意任务用较高温度。

**English short answer**
> Greedy decoding always picks the most likely token, beam search keeps several candidate sequences, and sampling draws from the distribution. Temperature reshapes the distribution, top-k restricts to the k most likely tokens, and top-p restricts to the smallest set whose probability mass reaches p. For structured outputs like tool calls I use low temperature or greedy decoding.

**追问方向**：温度为 0 是否保证确定？（浮点非确定性、批处理影响）

### Q26. 投机解码（Speculative Decoding）★★
**答案要点**
用小的草稿模型一次生成 $\gamma$ 个候选 token，大模型一次并行验证（一次前向给出这 $\gamma$ 个位置的分布），按接受概率 $\min(1,p_{large}/p_{draft})$ 决定接受，拒绝处从修正分布重采样，从而保证输出分布与大模型一致。因为 decode 访存受限，一次前向验证多个 token 几乎不增加时间，故能加速 2–3 倍量级（取决于接受率）。

**English short answer**
> A small draft model proposes several tokens, and the large model verifies them all in one parallel forward pass. Accepted tokens are kept, and the first rejection is resampled so the output distribution matches the large model exactly. It helps because decoding is memory-bound, so verifying many tokens costs about the same as one.

**追问方向**：接受率低时会怎样？

### Q27. 连续批处理与 PagedAttention ★★
**答案要点**
静态批处理要等最长序列结束，GPU 空转。连续批处理（continuous batching）：每个 decode 步都可以让完成的请求退出、新请求加入。PagedAttention（vLLM）：像操作系统分页一样把 KV cache 切成固定大小块，按需分配，消除碎片与预留浪费，并支持共享前缀（如共同 system prompt）。

**English short answer**
> Continuous batching lets finished requests leave and new ones join at every decoding step, so the GPU stays busy. PagedAttention stores the KV cache in fixed-size blocks like virtual memory pages, which removes fragmentation and allows sharing common prefixes between requests.

**追问方向**：前缀缓存（prefix caching）对 Agent 场景有何价值？（system prompt 与工具描述固定）

### Q28. LLM 推理的性能指标 ★★
**答案要点**
TTFT（首 token 延迟，主要由 prefill 决定）、TPOT（每输出 token 时间，由 decode 决定）、吞吐（tokens/s）、并发度、显存。优化：量化（权重 INT8/INT4：GPTQ/AWQ；KV cache 量化）、FlashAttention、算子融合、张量并行、投机解码、缓存。成本分析：decode 受限于 `权重字节数 / 内存带宽`，因此权重量化直接提速。

**English short answer**
> Time to first token depends on the prefill, and time per output token depends on decode. Decode is bounded by how fast we can read the weights from memory, so quantizing weights directly speeds it up. Other levers are FlashAttention, batching, speculative decoding and parallelism.

**追问方向**：为什么 batch 大了吞吐更高？（权重读取被多个请求分摊）

### Q29. 长上下文扩展的方法 ★★
**答案要点**
位置插值/NTK/YaRN 缩放 RoPE 频率后少量长文本继续训练；稀疏/滑窗注意力；检索增强（把长文档放在外部，见 [07_Agent与RAG八股](./07_Agent与RAG八股.md)）；KV cache 压缩/淘汰。问题：训练长度外的泛化、"中间丢失"（lost in the middle：关键信息在长上下文中间时利用率低，是已有研究观察到的现象，细节标未核对）、显存与延迟随长度增长。

**English short answer**
> To extend context I rescale the rotary frequencies and fine-tune on longer texts, use sparse or sliding window attention, or compress the KV cache. Models often use information in the middle of a long context less reliably, so for large documents retrieval is often better than stuffing everything into the prompt.

**追问方向**：长上下文能否替代 RAG？

### Q30. 结构化输出与约束解码 ★★
**答案要点**
让 LLM 输出合法 JSON/函数调用：提示约束 + 解析校验 + 重试；更强：约束解码，在每步根据语法（JSON schema/正则/CFG）屏蔽非法 token 的 logits，保证语法合法（但不保证语义正确）。对 Agent 的工具调用关键：语法合法后仍要做参数范围校验（见 [07_Agent与RAG八股](./07_Agent与RAG八股.md)）。

**English short answer**
> To get valid structured output I can prompt for a schema and validate with retries, or use constrained decoding, which masks tokens that would violate a grammar at each step. That guarantees valid syntax but not correct content, so I still validate argument values before acting on them.

**追问方向**：为什么语法合法不等于安全？

---

## 五、微调与对齐

### Q31. 预训练、SFT、RLHF 的整体流水线 ★★★
**答案要点**
```text
预训练 (海量文本, 预测下一词)  -> 基座模型 (base)
   -> SFT 指令微调 (人工/合成的 指令-回答对, 监督学习)  -> 能听指令
   -> 偏好对齐: RLHF (奖励模型 + PPO) 或 DPO  -> 更有用、更无害、更诚实
```
基座模型会续写但不一定遵循指令；SFT 教格式与行为；偏好学习教"哪个回答更好"，用比较数据而非唯一标准答案。

**English short answer**
> First, pretraining on a huge text corpus gives a base model that continues text. Then supervised fine-tuning on instruction-response pairs teaches it to follow instructions. Finally preference optimization, such as RLHF or DPO, aligns the outputs with human preferences for helpfulness and safety.

**追问方向**：为什么 SFT 之后还要 RLHF？

### Q32. SFT 细节 ★★★
**答案要点**
数据格式化为对话模板；损失只在回答部分计算（对 prompt 的 token 做 loss mask）；学习率小（$10^{-5}$ 量级）、轮数少（1–3 epoch）以防遗忘；数据质量远比数量重要（少量高质量指令数据即可激发能力，是被广泛引用的观点，LIMA 等；细节未核对）。风险：过拟合、灾难性遗忘、让模型学会自信地编造它不知道的知识（诱发幻觉）。

**English short answer**
> In SFT I format data with a chat template, compute the loss only on the response tokens, use a small learning rate and few epochs. Data quality matters more than quantity. A risk is teaching the model to answer confidently about facts it doesn't actually know, which can increase hallucination.

**追问方向**：为什么要 mask 掉 prompt 的 loss？

### Q33. RLHF：奖励模型与 PPO ★★★
**答案要点**
1) 奖励模型（RM）：对同一 prompt 的两个回答 $y_w\succ y_l$ 用 Bradley–Terry 模型：
$$L_{RM}=-E\big[\log\sigma\big(r_\phi(x,y_w)-r_\phi(x,y_l)\big)\big]$$
2) 策略优化：用 PPO 最大化奖励，同时用 KL 惩罚约束与参考（SFT）模型的距离：
$$\max_\pi\ E_{x,\,y\sim\pi}\big[r_\phi(x,y)\big]-\beta\,D_{KL}\big(\pi(\cdot|x)\|\pi_{ref}(\cdot|x)\big)$$
KL 防止策略为钻奖励模型漏洞而跑偏（奖励黑客，reward hacking）与语言质量崩坏。PPO 需要同时维护策略、参考、奖励、价值（critic）四个模型，显存与调参成本高。PPO 本身见 [05_强化学习八股](./05_强化学习八股.md)。

**English short answer**
> RLHF first trains a reward model on human comparisons with a Bradley-Terry loss, so the preferred answer gets a higher score. Then it optimizes the language model with PPO to maximize the reward, with a KL penalty that keeps it close to the SFT model, preventing reward hacking. It's powerful but complex, because it needs several models in memory and is sensitive to hyperparameters.

**追问方向**：奖励黑客的例子？为什么需要 KL？

### Q34. DPO（Direct Preference Optimization）★★★
**答案要点**
RLHF 目标的最优策略满足 $\pi^*(y|x)\propto\pi_{ref}(y|x)\exp(r(x,y)/\beta)$，反解出奖励 $r=\beta\log\frac{\pi^*}{\pi_{ref}}+\beta\log Z(x)$，代入 Bradley–Terry 后 $Z(x)$ 抵消，得到直接在偏好对上训练策略的分类式损失：
$$L_{DPO}=-E\Big[\log\sigma\Big(\beta\log\frac{\pi_\theta(y_w|x)}{\pi_{ref}(y_w|x)}-\beta\log\frac{\pi_\theta(y_l|x)}{\pi_{ref}(y_l|x)}\Big)\Big]$$
无需显式奖励模型与 RL 采样，更简单稳定；局限：依赖离线偏好数据、可能降低好回答的绝对概率、对分布外泛化弱于在线方法。

**English short answer**
> DPO shows that the optimal RLHF policy implies a reward expressed through the log-ratio of the policy and reference model. Plugging that into the preference loss removes the need for a separate reward model and for RL sampling, so training becomes a simple classification-like loss on chosen versus rejected responses. It's more stable and cheaper, but it relies on offline preference data.

**追问方向**：β 的作用？DPO 与 PPO 各适合什么情形？

### Q35. LoRA 的原理与参数量 ★★★
**答案要点**
全参数微调代价大。假设微调时权重变化 $\Delta W$ 具有低内在秩，LoRA 冻结 $W_0\in\mathbb R^{d\times k}$，学习 $\Delta W=BA$，$B\in\mathbb R^{d\times r},A\in\mathbb R^{r\times k}$，$r\ll\min(d,k)$：
$$h=W_0x+\frac{\alpha}{r}BAx$$
初始化 $A$ 随机高斯、$B=0$，使初始 $\Delta W=0$。可训练参数从 $dk$ 降到 $r(d+k)$。示例（教学示例）：$d=k=4096,r=8$：$r(d+k)=65536$，相比 $16.8\text{M}$ 约 0.39%。推理时可把 $BA$ 合并进 $W_0$，无额外延迟；可为不同任务保存多个小适配器，按需切换。常加在注意力的 $W_Q,W_V$（或全部线性层）。

**English short answer**
> LoRA freezes the pretrained weight and learns a low-rank update, the product of two small matrices of rank r. B is initialized to zero so training starts from the original model. This cuts trainable parameters by orders of magnitude, and after training the update can be merged into the weights, so there's no extra inference latency.

**追问方向**：为什么假设低秩合理？秩 r 如何选？

### Q36. QLoRA 与其他 PEFT 方法 ★★
**答案要点**
QLoRA：基座权重量化为 4-bit（NF4）冻结，LoRA 适配器用 BF16 训练，配合分页优化器，单卡可微调大模型。其他 PEFT：Adapter（层间插入小 MLP）、Prefix/Prompt Tuning（学习虚拟 token）、BitFit（只训偏置）、IA³。选择依据：显存、任务数量、部署便利。

**English short answer**
> QLoRA keeps the base model in 4-bit precision and trains LoRA adapters on top, so large models can be fine-tuned on a single GPU. Other parameter-efficient methods include adapters, prompt tuning and bias-only tuning.

**追问方向**：量化基座会不会损失精度？

### Q37. 对齐的其他方法与问题 ★★
**答案要点**
RLAIF（用 AI 反馈）、Constitutional AI（按原则自我批评修订）、拒绝采样微调（best-of-$n$ 再 SFT）、GRPO（去掉 critic，用同组多个采样的相对奖励作基线）、可验证奖励的强化学习（数学/代码可自动判对错）。问题：奖励黑客、谄媚（sycophancy）、对齐税、偏好数据噪声。

**English short answer**
> Besides RLHF and DPO, there is AI feedback, constitutional approaches, rejection sampling, and group-relative methods such as GRPO that drop the value model and use the relative rewards within a group of samples. Common issues are reward hacking and sycophancy.

**追问方向**：可验证奖励相比人类偏好的优劣？

### Q38. 灾难性遗忘与持续学习 ★
**答案要点**
在新任务上微调会降低旧能力。缓解：混入预训练/通用数据（回放）、小学习率、LoRA、正则（EWC）、模型合并。

**English short answer**
> Fine-tuning on a narrow task can degrade general ability. I mitigate it by mixing in general data, using a low learning rate or LoRA, and evaluating on a held-out general benchmark.

**追问方向**：如何监控遗忘？

### Q39. 知识蒸馏在 LLM 中的用法 ★★
**答案要点**
学生拟合教师的软标签分布：$L=(1-\lambda)L_{CE}+\lambda T^2D_{KL}\big(\mathrm{softmax}(z_t/T)\|\mathrm{softmax}(z_s/T)\big)$。LLM 常用"序列级蒸馏"：用大模型生成数据（合成指令/思维链）训练小模型。边缘部署时是缩小模型的重要手段，见 [06_模型部署与量化八股](./06_模型部署与量化八股.md)。

**English short answer**
> In distillation the student matches the teacher's softened output distribution, using a temperature, in addition to the hard labels. For LLMs it's often done at the data level, by generating synthetic training data from a strong model. It's a key way to get a small model for edge deployment.

**追问方向**：为什么软标签比硬标签信息多？（类间相似度的暗知识）

---

## 六、幻觉、上下文、Prompt、评估

### Q40. 幻觉（Hallucination）的成因与缓解 ★★★
**答案要点**
定义：模型生成流畅但与事实或给定上下文不符的内容。成因：(1) 训练目标是"像人话"而非"真话"，对没见过的事实也会补出高概率文本；(2) 数据中的错误与过时；(3) SFT 教会模型总要回答；(4) 解码的采样随机性；(5) 上下文过长或检索噪声。缓解：RAG（用检索到的证据约束，要求引用）、工具调用（计算器、数据库）、让模型"不知道就说不知道"的对齐、低温度、事后校验（自洽性、引用核对、另一个模型/规则校验）、结构化输出与 schema 校验。对机器人：LLM 的输出不能直接驱动执行器，必须经过校验层（见 [07_Agent与RAG八股](./07_Agent与RAG八股.md) 与 P6 的"提案-校验-执行-反馈"链）。

**English short answer**
> A hallucination is fluent output that isn't supported by facts or the given context. It happens because the model is trained to produce plausible text, not verified truth, and it will fill in gaps. I reduce it with retrieval and citations, tool use for facts and calculations, lower temperature, training the model to abstain, and post-hoc verification. For robots, model outputs should never go straight to actuators; they go through a validation layer first.

**追问方向**：如何度量幻觉？（事实性评测、引用一致性）

### Q41. In-context Learning 与 Few-shot ★★
**答案要点**
不更新权重，仅靠提示中的示例让模型完成新任务。示例的格式、顺序、标签分布都影响结果。机制上有"隐式梯度下降""归纳头（induction heads）"等解释，属于研究中的假说。局限：占用上下文、对示例敏感。

**English short answer**
> In-context learning means giving a few examples in the prompt and the model performs the task without any weight updates. Results are sensitive to the format and order of the examples. The mechanisms are still an active research topic.

**追问方向**：什么时候该微调而不是 few-shot？

### Q42. Prompt 工程要点与思维链 ★★★
**答案要点**
结构清晰：角色/任务、上下文、约束、输出格式（给 schema 或示例）、负面约束。技巧：少样本示例、分步推理（Chain-of-Thought：让模型先写出推理再给答案，对多步推理有提升；Self-Consistency：多次采样取多数答案）、拆解为子任务（prompt chaining）、给出"不确定时输出 UNKNOWN"。安全：提示注入（prompt injection）——不可信文本里夹带指令；对策是隔离数据与指令、最小权限工具、输出校验。

**English short answer**
> A good prompt states the role and task, gives the relevant context, lists constraints, and specifies the output format, ideally with an example. Chain-of-thought asks the model to reason step by step, which helps on multi-step problems, and self-consistency samples several reasoning paths and votes. I also treat untrusted text as data, not instructions, to resist prompt injection.

**追问方向**：思维链为什么有效？CoT 的推理是否忠实？

### Q43. 提示注入与安全 ★★
**答案要点**
直接注入（用户要求"忽略以上指令"）与间接注入（网页/文档/邮件里夹带恶意指令被 Agent 读取）。对策：权限最小化、工具调用白名单与参数校验、对外部内容做隔离与标记、敏感操作人工确认、输出过滤、审计日志。详见 [07_Agent与RAG八股](./07_Agent与RAG八股.md)。

**English short answer**
> Prompt injection is when untrusted content tries to override the instructions. The defense is not just a better prompt: give the agent least-privilege tools, validate every action, require human confirmation for sensitive operations, and log everything.

**追问方向**：为什么间接注入更难防？

### Q44. LLM 评估：困惑度、基准、LLM-as-judge ★★★
**答案要点**
内在：困惑度（只在同分词器下可比）。外在：多选题基准（如 MMLU）、代码（HumanEval，用 pass@k 衡量：$pass@k=1-\binom{n-c}{k}/\binom{n}{k}$，$n$ 个样本中 $c$ 个通过）、数学（GSM8K）、指令遵循与对话（人类偏好、Elo 排名、LLM-as-judge）。问题：基准污染（训练数据含测试题）、过拟合基准、LLM 评审的位置偏好与长度偏好。应用层评估：构造贴合业务的测试集，指标含准确率、拒答率、延迟、成本、失败案例分类。

**English short answer**
> Perplexity measures language modeling quality, but task-level evaluation uses benchmarks, for example multiple-choice knowledge tests, code tests with pass-at-k, and human or LLM-judged preferences for chat. Benchmarks can be contaminated or overfit, and LLM judges have position and length biases. For real applications I build my own test set that mirrors the use case.

**追问方向**：pass@k 的无偏估计怎么算？

### Q45. 检索增强 RAG 在大模型中的位置（指针）★★
**答案要点**
RAG：检索外部知识放入上下文，减少幻觉、可更新知识、可溯源。流程与失败模式见 [07_Agent与RAG八股](./07_Agent与RAG八股.md)。与微调的取舍：知识频繁变化/需引用 → RAG；风格与行为 → 微调；二者可结合。

**English short answer**
> RAG retrieves external documents and puts them into the prompt, which reduces hallucination, keeps knowledge updatable and allows citations. I'd use RAG for changing facts and fine-tuning for style and behavior.

**追问方向**：为什么微调不适合注入新知识？

### Q46. 涌现能力与"思考型"模型 ★
**答案要点**
有观点认为某些能力随规模突然出现（涌现），也有研究指出是评测指标的非连续性造成的假象，争议存在。近期趋势：通过强化学习训练模型生成较长的推理链（test-time compute），以更多推理 token 换取准确率。面试时说明这是进行中的研究。

**English short answer**
> Some abilities appear to emerge suddenly with scale, though others argue it's partly an artifact of discontinuous metrics. A recent trend is to spend more compute at inference time by letting the model produce long reasoning chains, often trained with reinforcement learning.

**追问方向**：test-time compute 的成本与收益？

### Q47. 对 LLM 做成本与延迟估算 ★★
**答案要点**
训练算力经验式 $C\approx6ND$（$N$ 参数，$D$ token），因为前向约 $2ND$、反向约 $4ND$ FLOPs。推理每 token 约 $2N$ FLOPs。示例（教学示例）：7B 模型处理 $10^{12}$ token 约 $6\times7\times10^9\times10^{12}=4.2\times10^{22}$ FLOPs。decode 速度上界 ≈ 内存带宽 / 权重字节：带宽 1 TB/s、权重 14 GB（7B FP16）→ 约 70 token/s（单序列上界，推算）。

**English short answer**
> Training cost is roughly six times parameters times tokens in FLOPs, and inference is about two times parameters per token. For single-sequence decoding, the speed is bounded by memory bandwidth divided by the model size in bytes, which is why quantizing the weights speeds up generation.

**追问方向**：batch 增大后上界怎么变？

---

## 七、视觉与多模态、VLA

### Q48. ViT（Vision Transformer）★★★
**答案要点**
把图像切成 $P\times P$ 的 patch（如 $224\times224$、$P=16$ → $14\times14=196$ 个），每个 patch 展平后线性投影为 $d$ 维 token（等价于 stride=$P$、核=$P$ 的卷积），加上可学习位置嵌入与一个 `[CLS]` token，送入标准 Transformer 编码器，用 `[CLS]`（或平均池化）接分类头。特点：归纳偏置弱，小数据上不如 CNN，大规模预训练（或强增强/蒸馏）后表现好；全局注意力随 token 数二次增长，高分辨率代价大（Swin 用移位窗口注意力）。

**English short answer**
> ViT splits an image into fixed-size patches, flattens and linearly projects each into a token, adds position embeddings and a class token, and feeds them to a standard Transformer encoder. It has weaker inductive biases than CNNs, so it needs large-scale pretraining, but then it scales very well. Attention cost grows quadratically with the number of patches.

**追问方向**：patch 大小对精度/计算的影响？位置嵌入如何插值到新分辨率？

### Q49. CLIP 与对比式图文预训练 ★★★
**答案要点**
图像编码器与文本编码器分别输出嵌入，在 batch 内做对称 InfoNCE：
$$L=-\frac1{2N}\sum_i\Big[\log\frac{e^{s_{ii}/\tau}}{\sum_je^{s_{ij}/\tau}}+\log\frac{e^{s_{ii}/\tau}}{\sum_je^{s_{ji}/\tau}}\Big],\quad s_{ij}=\cos(v_i,t_j)$$
得到对齐的图文空间，支持零样本分类（用"a photo of a {class}"构造类别文本，选相似度最高者）、图文检索，是多模态大模型视觉编码器的常见来源。局限：细粒度、计数、空间关系较弱。

**English short answer**
> CLIP trains an image encoder and a text encoder so that matching image-text pairs have high cosine similarity and mismatched pairs in the batch have low similarity, using a symmetric contrastive loss. That gives a shared embedding space, enabling zero-shot classification by comparing an image with text prompts for each class. It's weaker on counting and fine spatial relations.

**追问方向**：batch 越大越好的原因？温度 $\tau$ 的作用？

### Q50. 视觉语言模型（VLM）的典型结构 ★★
**答案要点**
视觉编码器（ViT/CLIP）→ 投影/连接器（线性层、MLP 或 Q-Former/重采样器）→ 把视觉 token 作为"前缀"输入 LLM，与文本 token 一同自回归。训练常分两阶段：先对齐（只训连接器），再指令微调（解冻 LLM 或用 LoRA）。常见失败：幻觉（描述不存在的物体）、空间推理弱、分辨率限制。

**English short answer**
> A vision-language model uses a vision encoder to turn an image into tokens, a projector that maps them into the LLM's embedding space, and the LLM that reads them with the text. Training usually first aligns the projector, then does instruction tuning. Typical weaknesses are object hallucination and weak spatial reasoning.

**追问方向**：为什么要先只训连接器？

### Q51. 具身智能中的 VLA（Vision-Language-Action）简述 ★★★
**答案要点**
VLA：输入相机图像 + 语言指令（+ 本体状态），输出机器人动作的模型。主要思路：(1) 把动作离散化为 token，与文本一起自回归（RT-2 类思路，把机器人动作表示为文本 token 与网络规模的视觉语言数据共同训练）；(2) 动作头直接回归连续动作，或用扩散/流匹配生成动作块（action chunking）；(3) 层次式：LLM 做高层任务规划，底层用传统控制或学习策略执行。优势：语言泛化、语义理解、少样本迁移。难点：动作数据稀缺、推理延迟（大模型几 Hz 到十几 Hz 难满足高频控制）、安全性、sim2real。工程上常见折中：VLA/LLM 输出子目标或轨迹，由经典控制（PID/MPC/IK）+ 安全层以高频执行。（具体模型的名称与指标以论文为准，此处仅作概念性介绍，未逐篇核对。）这与你 P6 的"高层计划必须经提案-校验-执行-反馈链再到物理动作"的理念一致，可作为自然的引子。

**English short answer**
> A vision-language-action model takes camera images and a language instruction and outputs robot actions. Some represent actions as discrete tokens so one transformer predicts text and actions, others use a continuous or diffusion-based action head that outputs chunks of actions. The big challenges are scarce robot data, slow inference compared to control rates, and safety. A common practical design uses the model for high-level goals and a classical controller with a safety layer for execution.

**追问方向**：为什么大模型的低频输出需要 action chunking 或下层高频控制器？

### Q52. 多模态融合策略 ★
**答案要点**
早期融合（输入层拼接）、晚期融合（各模态独立后合并）、中间融合（交叉注意力、Q-Former）。机器人感知中还包括时间同步与坐标系对齐问题，见 [06_时间同步_坐标系一致性与数据有效性](../../02_基础知识/05_状态估计/06_时间同步_坐标系一致性与数据有效性.md)。

**English short answer**
> Fusion can happen early by concatenating inputs, late by combining separate predictions, or in the middle through cross-attention. In robotics, time synchronization and coordinate frame alignment between modalities are just as important as the fusion architecture.

**追问方向**：模态缺失时模型如何鲁棒？

### Q53. 从 Transformer 看机器人策略学习 ★★
**答案要点**
Decision Transformer 把 RL 当序列建模（条件于回报预测动作）；ACT 等用 Transformer 预测动作块做模仿学习；扩散策略建模多模态动作分布。共同点：用大序列模型吸收示教数据。详见 [05_强化学习八股](./05_强化学习八股.md)。（以上均为概念性描述，具体论文细节未核对。）

**English short answer**
> Transformers are used in robot learning by treating trajectories as sequences, for example predicting action chunks from demonstrations, or conditioning on return as in Decision Transformer. They are a flexible way to absorb large amounts of demonstration data.

**追问方向**：为什么动作分块可以缓解复合误差？

### Q54. 大模型的部署：量化与端侧 ★★
**答案要点**
权重量化 INT8/INT4（GPTQ、AWQ 保护显著通道）、KV 量化、LLM.int8() 的离群特征处理。端侧限制：内存容量与带宽、功耗。可参考 [06_模型部署与量化八股](./06_模型部署与量化八股.md)。

**English short answer**
> LLM weights are commonly quantized to eight or four bits, with methods that protect outlier or salient channels. On edge devices the limits are memory capacity, bandwidth and power, so a small quantized model is often combined with cloud calls.

**追问方向**：为什么 LLM 激活量化比 CNN 更难？（激活中存在系统性离群值）

---

## 本章要点速记
1. 注意力 $=\mathrm{softmax}(QK^T/\sqrt{d_k})V$；除 $\sqrt{d_k}$ 为防 softmax 饱和。
2. 多头 = 子空间并行；总参数 $4d^2$；一层块约 $12d^2$。
3. 因果掩码：上三角加 $-\infty$；训练并行、推理串行。
4. RoPE 使分数只依赖相对位置：$R_m^TR_n=R_{n-m}$。
5. KV cache 省重算但占显存：$2LnT\,d_{head}B\cdot$bytes；prefill 算力受限、decode 带宽受限。
6. GQA/MQA 缩 KV；FlashAttention 精确注意力、减少 HBM 读写。
7. 流水线：预训练 → SFT → RLHF/DPO；RLHF = RM(BT) + PPO + KL；DPO 去掉 RM 与 RL。
8. LoRA：$W_0+\frac{\alpha}{r}BA$，$B$ 零初始化，可合并无延迟。
9. 幻觉靠 RAG、工具、校验层缓解；机器人上 LLM 输出必须经校验才能执行。
10. ViT = patch 嵌入 + Transformer；CLIP = 对称 InfoNCE；VLA 常配经典控制+安全层。

## 自测题
1. 证明 $q\cdot k$ 的方差为 $d_k$，并说明缩放因子的来源。（独立同分布单位方差分量，和的方差相加。）
2. 7B 参数模型 FP16 在带宽 1 TB/s 设备上 decode 的速度上界？（约 $1\text{TB/s}\div14\text{GB}\approx70$ token/s，推算。）
3. LoRA 对 $4096\times4096$ 矩阵 $r=16$ 的可训练参数量？（$16\times8192=131072$。）
4. 写出 DPO 损失，并说明它为什么不需要奖励模型。
5. 为什么 decoder-only 模型推理要用 KV cache，而训练不需要？（训练一次并行算全序列，推理需逐步生成并复用历史 K、V。）

## 相关链接
- 上一章：[03_计算机视觉与目标检测八股](./03_计算机视觉与目标检测八股.md)；下一章：[05_强化学习八股](./05_强化学习八股.md)
- 基础：[05_Transformer_LLM与Agent_LangGraph](../../02_基础知识/07_感知与机器学习/05_Transformer_LLM与Agent_LangGraph.md)、[06_强化学习与具身智能入门](../../02_基础知识/07_感知与机器学习/06_强化学习与具身智能入门.md)
- Agent 与 RAG：[07_Agent与RAG八股](./07_Agent与RAG八股.md)
