# 生成模型、模仿学习与 VLA 八股

> **定位**：具身智能方向的面试几乎一定会问"策略怎么学"：行为克隆有什么问题、为什么用扩散策略、动作分块解决什么、VLA 与传统模块化的区别、数据从哪里来。本文把**生成模型基础（VAE / GAN / 扩散 / 流匹配）→ 模仿学习 → VLA 与具身基础模型 → 数据与部署工程**整理成问答，60 余题。相关基础：[模仿学习](../../02_基础知识/10_机器学习与深度学习/10_模仿学习_行为克隆_DAgger_扩散策略.md)、[序列模型与生成模型](../../02_基础知识/10_机器学习与深度学习/07_序列模型_残差网络_ViT与生成模型.md)、[世界模型与具身基础模型](../../02_基础知识/10_机器学习与深度学习/14_世界模型与具身基础模型综述.md)。

**本章学完你能……**
- 比较 VAE、GAN、扩散模型、流匹配的目标函数与优缺点；
- 解释扩散模型的前向 / 反向过程、采样加速与条件控制；
- 说清行为克隆的协变量偏移、多模态动作与 DAgger / 动作分块 / 扩散策略如何缓解；
- 描述 VLA 的结构、动作表示、训练数据与局限；
- 讨论数据采集、评估与安全部署，并用英文表达。

**前置知识**：[强化学习八股](../02_人工智能/05_强化学习八股.md)（行为克隆与 DAgger 初步）；[Transformer 与大模型八股](../02_人工智能/04_Transformer与大模型八股.md)。

> 涉及具体论文的数字（成功率、数据量、模型规模、发布年份）一律不作为事实陈述；只写方法思想，具体数值标"未核对"。

---

## 一、生成模型基础

### Q1 生成模型有哪几大类？目标各是什么？（Families of generative models）
**优先级：★★★** · 对应基础知识：[序列模型_残差网络_ViT与生成模型](../../02_基础知识/10_机器学习与深度学习/07_序列模型_残差网络_ViT与生成模型.md)

**答案要点**
- **自回归**：把联合分布分解 $p(x)=\prod p(x_t|x_{<t})$，最大化似然（GPT、PixelCNN）；采样顺序、慢，但似然精确、训练稳定。
- **VAE**：隐变量模型，最大化证据下界 ELBO，兼顾重构与先验约束；采样快，输出常偏模糊。
- **GAN**：生成器与判别器对抗，隐式分布，不要求似然；样本锐利，训练不稳定、模式坍缩。
- **流模型**（normalizing flow）：可逆变换，精确似然，需要可逆结构与雅可比行列式。
- **扩散 / 分数模型**：学习逐步去噪，训练稳定、多样性好、质量高，但采样需多步；与流匹配（flow matching）相通。
- 选择：要精确似然 → 自回归 / 流；要高质量多样 → 扩散；要极快采样 → GAN / 蒸馏后的扩散 / 一致性模型。

**English short answer**
Autoregressive models factor the likelihood and are trained by maximum likelihood, VAEs optimize an evidence lower bound over latent variables, GANs train a generator against a discriminator, flows use invertible maps with exact likelihood, and diffusion models learn to denoise step by step. Diffusion gives high quality and diversity with stable training but slow sampling, which is why distillation and fewer-step samplers matter.

（译：自回归分解似然并最大似然训练，VAE 在隐变量上优化证据下界，GAN 让生成器与判别器对抗，流模型用可逆映射得精确似然，扩散模型学习逐步去噪；扩散质量高、多样、训练稳定但采样慢，所以蒸馏与少步采样器很重要。）

**追问方向**：为何 GAN 训练不稳定；扩散与流匹配的关系。

### Q2 VAE 的原理与 ELBO？为什么重构会模糊？（VAE）
**优先级：★★★** · 对应基础知识：[序列模型_残差网络_ViT与生成模型](../../02_基础知识/10_机器学习与深度学习/07_序列模型_残差网络_ViT与生成模型.md)

**答案要点**
- 编码器 $q_\phi(z|x)$ 输出高斯参数，解码器 $p_\theta(x|z)$ 重构；目标 ELBO $=\mathbb{E}_q[\log p_\theta(x|z)]-\mathrm{KL}(q_\phi(z|x)\|p(z))$。
- **重参数化技巧**：$z=\mu+\sigma\odot\epsilon,\ \epsilon\sim\mathcal{N}(0,I)$，使梯度可通过采样反传。
- 模糊成因：高斯解码器（MSE）对不确定输出取均值，多个可能输出平均后变糊；KL 正则也压缩信息。
- 变体：$\beta$-VAE（调节 KL 权重得到解耦表示）、VQ-VAE（离散隐变量 + 码本，用于图像 / 音频 token 化）、条件 VAE（CVAE，在机器人 ACT 中用来表示动作风格）。
- 常见问题：后验坍缩（decoder 太强，忽略 $z$）。

**English short answer**
A VAE trains an encoder and decoder by maximizing the evidence lower bound, which combines a reconstruction term with a KL penalty pulling the latent posterior toward the prior, and uses the reparameterization trick so gradients flow through sampling. Reconstructions tend to be blurry because a Gaussian decoder averages over possible outputs. VQ-VAE uses discrete codes and underlies tokenizers for images and audio, and conditional VAEs appear in robot policies such as ACT.

（译：VAE 通过最大化 ELBO（重构项加把隐变量后验拉向先验的 KL 项）训练编码器与解码器，并用重参数化技巧使梯度穿过采样；高斯解码器对可能输出取平均导致重构模糊；VQ-VAE 用离散码本，是图像/音频分词的基础，条件 VAE 出现在 ACT 等机器人策略中。）

**追问方向**：后验坍缩的缓解；VQ-VAE 的码本崩溃。

### Q3 GAN 的训练目标、模式坍缩与稳定化方法？（GANs）
**优先级：★★** · 对应基础知识：[序列模型_残差网络_ViT与生成模型](../../02_基础知识/10_机器学习与深度学习/07_序列模型_残差网络_ViT与生成模型.md)

**答案要点**
- 极小极大博弈：$\min_G\max_D\mathbb{E}_x\log D(x)+\mathbb{E}_z\log(1-D(G(z)))$；最优判别器下，生成器目标等价于最小化 JS 散度。
- 问题：训练振荡 / 不收敛、梯度消失（判别器太强）、**模式坍缩**（生成器只产出少数样本）、评估困难。
- 稳定化：WGAN（Wasserstein 距离 + 梯度惩罚 / 谱归一化）、非饱和损失、两时间尺度更新（TTUR）、渐进式训练、数据增强（DiffAugment）、R1 正则。
- 评估：IS、FID（比较真实与生成样本的 Inception 特征高斯分布的 Fréchet 距离）、精确率/召回率。
- 现状：扩散模型在多数生成任务上取代 GAN，但 GAN 在实时/一步生成与超分辨率中仍有价值。

**English short answer**
A GAN is a minimax game between a generator and a discriminator, which at the optimum minimizes the Jensen-Shannon divergence. It suffers from oscillation, vanishing gradients and mode collapse. WGAN with gradient penalty or spectral normalization, careful learning rates and augmentation stabilize it, and quality is measured with FID. Diffusion models have largely displaced GANs, though GANs remain attractive for one-step generation.

（译：GAN 是生成器与判别器的极小极大博弈，最优时最小化 JS 散度；存在振荡、梯度消失和模式坍缩；WGAN 加梯度惩罚或谱归一化、合理学习率与数据增强可稳定训练，用 FID 评估；扩散模型已大体取代 GAN，但 GAN 在一步生成上仍有吸引力。）

**追问方向**：FID 的计算方式及局限；为什么 WGAN 梯度更好。

### Q4 扩散模型（DDPM）的前向与反向过程？（Diffusion models: DDPM）
**优先级：★★★** · 对应基础知识：[序列模型_残差网络_ViT与生成模型](../../02_基础知识/10_机器学习与深度学习/07_序列模型_残差网络_ViT与生成模型.md)

**答案要点**
- **前向**：逐步加高斯噪声 $q(x_t|x_{t-1})=\mathcal{N}(\sqrt{1-\beta_t}x_{t-1},\beta_tI)$；闭式 $x_t=\sqrt{\bar\alpha_t}x_0+\sqrt{1-\bar\alpha_t}\,\epsilon$，$\bar\alpha_t=\prod(1-\beta_s)$，$T$ 步后接近标准高斯。
- **反向**：学习去噪 $p_\theta(x_{t-1}|x_t)$；常见做法是网络预测噪声 $\epsilon_\theta(x_t,t)$，**简化损失** $\mathbb{E}\|\epsilon-\epsilon_\theta(x_t,t)\|^2$。
- **采样**：从 $x_T\sim\mathcal{N}(0,I)$ 开始，迭代去噪 $T$ 步（原始 DDPM 约上千步）。
- 等价视角：预测 $x_0$、预测 $v$、预测分数 $\nabla_x\log p_t(x)$（分数匹配）；噪声调度（线性、余弦）影响质量。
- 网络：U-Net 或 Transformer（DiT），时间步 $t$ 以嵌入形式注入。

**English short answer**
The forward process gradually adds Gaussian noise so that any noisy sample can be written in closed form from the clean one. The network learns the reverse step, typically by predicting the added noise with a simple mean-squared loss. Sampling starts from pure noise and iteratively denoises. This is equivalent to score matching, and the denoiser is a U-Net or a transformer conditioned on the timestep.

（译：前向过程逐步加高斯噪声，任一时刻的带噪样本可由干净样本闭式写出；网络学习反向去噪，通常用简单均方损失预测所加噪声；采样从纯噪声开始迭代去噪；这等价于分数匹配，去噪器是以时间步为条件的 U-Net 或 Transformer。）

**追问方向**：为什么预测噪声而非 $x_0$；噪声调度对质量的影响。

### Q5 如何加速扩散采样？（Accelerating diffusion sampling）
**优先级：★★★** · 对应基础知识：[序列模型_残差网络_ViT与生成模型](../../02_基础知识/10_机器学习与深度学习/07_序列模型_残差网络_ViT与生成模型.md)

**答案要点**
- **DDIM**：非马尔可夫的确定性采样，可大幅跳步（几十步），同一初始噪声得到确定输出，可做插值与反演。
- **高阶 ODE 求解器**：DPM-Solver / DPM-Solver++、Heun、UniPC，把采样视为概率流 ODE，用更少步数（10–25 步）。
- **蒸馏**：渐进蒸馏、一致性模型（consistency models，一步 / 少步映射到 $x_0$）、对抗蒸馏、分数蒸馏。
- **流匹配 / 校正流**：学习近直线的传输路径，少步采样更准。
- 权衡：步数↓速度↑，但多样性 / 细节可能下降；对机器人策略，常用 DDIM 10 步以内以满足控制频率。

**English short answer**
DDIM uses a deterministic non-Markovian sampler that skips most steps, higher-order ODE solvers such as DPM-Solver treat sampling as integrating a probability-flow ODE in a few dozen steps, and distillation methods such as consistency models compress it to one or a few steps. Flow matching learns nearly straight paths that sample accurately with fewer steps. For robot policies I use very few denoising steps to meet the control rate.

（译：DDIM 用确定性非马尔可夫采样跳过大多数步骤，DPM-Solver 等高阶 ODE 求解器把采样视作概率流 ODE 在几十步内积分，一致性模型等蒸馏方法可压缩到一步或少数几步，流匹配学习近似直线的路径，少步也能准确采样；机器人策略为满足控制频率会用极少的去噪步数。）

**追问方向**：一致性模型的训练目标；少步采样对多模态覆盖的影响。

### Q6 条件生成与分类器无关引导（Classifier-Free Guidance）？（Conditional generation and CFG）
**优先级：★★★** · 对应基础知识：[序列模型_残差网络_ViT与生成模型](../../02_基础知识/10_机器学习与深度学习/07_序列模型_残差网络_ViT与生成模型.md)

**答案要点**
- 条件扩散：把条件 $c$（文本、图像、状态、语言指令）输入去噪网络（拼接、交叉注意力、FiLM / AdaLN）。
- **CFG**：训练时以一定概率（如 10%）丢弃条件，使同一网络同时学习条件与无条件预测；采样时 $\tilde\epsilon=\epsilon_\theta(x_t,\varnothing)+w\,[\epsilon_\theta(x_t,c)-\epsilon_\theta(x_t,\varnothing)]$，引导强度 $w>1$ 提高与条件的一致性、降低多样性。
- 对比分类器引导：需要另训练带噪分类器；CFG 无需额外模型。
- 机器人策略：对观测（图像、本体感觉）和语言条件化；CFG 在策略中有时用于增强指令遵循，但要谨慎（可能产生过激动作）。
- 其他控制：ControlNet（空间条件）、inpainting、潜空间编辑。

**English short answer**
Conditional diffusion feeds the condition into the denoiser through concatenation, cross-attention or FiLM. Classifier-free guidance trains one network with the condition randomly dropped, then at sampling extrapolates from the unconditional prediction toward the conditional one with a guidance weight, increasing adherence to the condition at the cost of diversity. It avoids training a separate classifier, and in robot policies it must be used carefully to avoid extreme actions.

（译：条件扩散通过拼接、交叉注意力或 FiLM 把条件送入去噪器；CFG 训练时随机丢弃条件使同一网络兼得条件与无条件预测，采样时按引导权重从无条件预测向条件预测外推，提高条件一致性但降低多样性；无需额外分类器，用于机器人策略时需谨慎以免动作过激。）

**追问方向**：引导权重过大会怎样；FiLM 与交叉注意力的区别。

### Q7 潜在扩散（Latent Diffusion）为什么高效？（Latent diffusion）
**优先级：★★** · 对应基础知识：[序列模型_残差网络_ViT与生成模型](../../02_基础知识/10_机器学习与深度学习/07_序列模型_残差网络_ViT与生成模型.md)

**答案要点**
- 先用自编码器（VAE，常带感知 / 对抗损失）把图像压缩到低维潜空间（如 8× 下采样），在潜空间做扩散；像素数大幅减少，训练与采样成本降低一个量级以上，质量损失很小。
- 文本条件通过 CLIP / T5 编码，交叉注意力注入 U-Net / DiT。
- 视频 / 动作：同样可对视频压缩到时空潜空间；机器人世界模型常在潜空间中预测。
- 局限：自编码器重建的上限决定细节；潜空间的偏差会影响精细几何。
- 面试表述：把"感知压缩"与"生成建模"解耦。

**English short answer**
Latent diffusion first compresses images with an autoencoder to a much smaller latent space and runs diffusion there, cutting compute by an order of magnitude with little quality loss. Text conditioning enters through cross-attention. It decouples perceptual compression from generative modeling, and the same idea is used for video and for world models that predict in latent space.

（译：潜在扩散先用自编码器把图像压到小得多的潜空间并在其中做扩散，计算量降低一个数量级而质量损失小；文本条件经交叉注意力注入；它把感知压缩与生成建模解耦，同样思路用于视频及在潜空间预测的世界模型。）

**追问方向**：自编码器的重建上限；视频潜空间的时序压缩。

### Q8 流匹配（Flow Matching）与扩散模型的区别？（Flow matching）
**优先级：★★** · 对应基础知识：[序列模型_残差网络_ViT与生成模型](../../02_基础知识/10_机器学习与深度学习/07_序列模型_残差网络_ViT与生成模型.md)

**答案要点**
- 流匹配直接学习一个**速度场** $v_\theta(x,t)$，使 ODE $\dot x=v_\theta$ 把简单分布（高斯）搬运到数据分布；训练目标回归"条件速度"，如线性插值路径 $x_t=(1-t)x_0+tx_1$ 的速度 $x_1-x_0$。
- 与扩散：扩散是随机过程 + 去噪分数；流匹配是确定性 ODE，路径可设计为近直线（校正流 rectified flow），采样步数少、训练简单（无需复杂噪声调度）。
- 优点：简洁、稳定、少步采样好；在大规模图像、机器人动作（如 $\pi_0$ 类策略采用流匹配，细节未核对）中使用。
- 两者可统一在"随机插值 / 概率流"框架下。

**English short answer**
Flow matching regresses a velocity field whose ODE transports a simple noise distribution to the data distribution, with a target such as the straight-line velocity between a noise sample and a data sample. Compared with diffusion's stochastic noising and score estimation, it is deterministic, simple to train and samples well in few steps, which is why recent large generative models and robot action heads adopt it.

（译：流匹配回归一个速度场，其 ODE 把简单噪声分布搬运到数据分布，目标如噪声样本与数据样本间的直线速度；与扩散的随机加噪及分数估计相比，它确定性、训练简单、少步采样效果好，所以近期大型生成模型和机器人动作头采用它。）

**追问方向**：校正流（rectified flow）的重流程；流匹配的采样器选择。

### Q9 如何评估生成模型？（Evaluating generative models）
**优先级：★★** · 对应基础知识：[机器学习基础](../../02_基础知识/10_机器学习与深度学习/01_机器学习基础.md)

**答案要点**
- 样本质量与多样性：**FID**（Fréchet Inception Distance，越低越好）、IS、精确率–召回率、CLIP Score（图文一致）。
- 似然：自回归 / 流可直接算对数似然 / bits per dim；VAE 给 ELBO；扩散可算 ELBO 但不直观。
- 人工评估：配对偏好测试、Elo；有主观性。
- 机器人策略生成：不看图像质量，而看**任务成功率、轨迹平滑度、模态覆盖**、碰撞率；多模态覆盖可用动作分布的 NLL 或 coverage 指标。
- 陷阱：FID 依赖特征提取器与样本量；指标与人感知不总一致；过拟合（记忆训练集）需检查最近邻。

**English short answer**
For images I use FID for fidelity and diversity, precision and recall, and CLIP score for text alignment, plus human preference tests. Autoregressive models and flows give likelihoods directly. For robot policies image metrics are irrelevant, so I use task success rate, smoothness, collision rate and coverage of action modes. I also check for memorization with nearest-neighbor retrieval.

（译：图像用 FID 衡量保真与多样性、精确率/召回率、CLIP Score 衡量图文一致，再加人工偏好测试；自回归与流模型可直接给似然；机器人策略不看图像指标，而看任务成功率、平滑度、碰撞率与动作模态覆盖；还要用最近邻检索检查是否记忆训练集。）

**追问方向**：FID 的偏差来源；策略多模态覆盖怎么量化。

---

## 二、模仿学习

### Q10 行为克隆（BC）的基本做法与根本局限？（Behavior cloning）
**优先级：★★★** · 对应基础知识：[模仿学习_行为克隆_DAgger_扩散策略](../../02_基础知识/10_机器学习与深度学习/10_模仿学习_行为克隆_DAgger_扩散策略.md)

**答案要点**
- 把示教数据 $(o_t,a_t)$ 当监督学习：$\min_\theta\mathbb{E}\|\pi_\theta(o)-a\|$，简单、离线、稳定。
- 局限一：**协变量偏移 / 误差累积**——训练分布来自专家状态，部署时策略一出小错就进入没见过的状态，错误复合，总误差随时域 $T$ 呈 $O(T^2\epsilon)$ 增长。
- 局限二：**多模态**——同一观测可能有多种合理动作（绕左或绕右），MSE 回归取平均得到"撞墙"的中间动作。
- 局限三：因果混淆、数据分布窄、缺少恢复行为、人类示教的噪声和不一致。
- 优点：样本效率高（相对 RL）、无需奖励；常作为 RL 的初始化 / 预训练。

**English short answer**
Behavior cloning regresses expert actions from observations, which is simple and stable but suffers from compounding errors: a small mistake leads to states absent from the demonstrations, and the error grows roughly quadratically with horizon. It also averages multimodal demonstrations into invalid actions. Remedies include DAgger, action chunking, expressive policy classes such as diffusion, and data augmentation.

（译：行为克隆把专家动作作为监督信号回归，简单稳定，但存在误差累积：小错误会进入示教中没有的状态，误差随时域大致二次增长；还会把多模态示教平均成无效动作；对策有 DAgger、动作分块、扩散等表达力强的策略类和数据增强。）

**追问方向**：为什么误差是 $T^2$；多模态下 MSE 的失败例子。

### Q11 DAgger 如何缓解协变量偏移？它的代价？（DAgger）
**优先级：★★★** · 对应基础知识：[模仿学习_行为克隆_DAgger_扩散策略](../../02_基础知识/10_机器学习与深度学习/10_模仿学习_行为克隆_DAgger_扩散策略.md)

**答案要点**
- 流程：用当前策略（或混合策略）在环境中 rollout，在策略访问的状态上让**专家重新标注动作**，把新数据并入数据集，重新训练；迭代。
- 效果：训练数据覆盖策略自身会到达的状态分布，误差降为 $O(T\epsilon)$。
- 代价：需要**在线专家**（人或最优控制器）即时给标签，人工成本高；标注时看着策略的状态可能不自然（人需要给出"如果在这个状态怎么办"的动作）；机器人上不安全。
- 变体：HG-DAgger（人在回路，人接管时收集）、SafeDAgger、DART（在示教时注入噪声以扩大状态覆盖）、用仿真中的特权专家（privileged expert）蒸馏到视觉策略。
- 面试要点：说明"为何需要专家在线"。

**English short answer**
DAgger rolls out the learner, asks the expert to label the actions for the states the learner actually visits, aggregates them into the dataset and retrains, so the training distribution matches deployment and the error grows linearly rather than quadratically with horizon. The cost is needing an online expert. Human-gated variants collect data at takeovers, DART injects noise during demonstrations, and a privileged simulation expert can be distilled into a vision policy.

（译：DAgger 让学习者 rollout，由专家为学习者实际访问的状态标注动作并汇总重训，使训练分布匹配部署分布，误差随时域线性而非二次增长；代价是需要在线专家；变体有人在回路接管采集、示教时注入噪声的 DART、把仿真特权专家蒸馏到视觉策略。）

**追问方向**：HG-DAgger 的数据采集流程；特权专家蒸馏的优缺点。

### Q12 多模态动作分布为什么难？有哪些表示方法？（Multimodal action distributions）
**优先级：★★★** · 对应基础知识：[模仿学习_行为克隆_DAgger_扩散策略](../../02_基础知识/10_机器学习与深度学习/10_模仿学习_行为克隆_DAgger_扩散策略.md)

**答案要点**
- 同一观测对应多种合理动作（不同示教者、不同策略）。单高斯 + MSE 回归 → 取均值，落在无效区域。
- 表示方法：①**混合密度网络 / GMM 头**；②**离散化动作**（分箱 + 分类 + 残差偏移，如 BeT 类）；③**条件 VAE**（隐变量表示风格，ACT）；④**能量模型 / 隐式策略**（IBC，通过优化或采样求 argmin）；⑤**扩散 / 流匹配策略**（直接建模动作分布，表达力最强）。
- 取舍：GMM 头简单但模态数需预设；扩散表达力最强，推理要多步迭代；隐式策略推理需优化，训练较不稳定。
- 评估多模态：在模拟中构造已知多解任务（如推 T 块绕两侧）。

**English short answer**
A single Gaussian regressed with mean squared error averages different valid behaviors into an invalid action. Alternatives include mixture density heads, discretized action tokens, conditional VAEs with latent style variables, implicit energy-based policies, and diffusion or flow policies, which have the highest expressiveness but need iterative inference. I choose by required expressiveness and inference latency.

（译：单高斯加 MSE 会把不同的有效行为平均成无效动作；替代方案有混合密度头、离散化动作 token、带隐式风格变量的条件 VAE、隐式能量模型以及表达力最强但推理需迭代的扩散/流策略；按所需表达力与推理延迟选择。）

**追问方向**：隐式行为克隆的推理方式；混合密度网络的坍缩问题。

### Q13 动作分块（Action Chunking，ACT）解决什么问题？（Action chunking）
**优先级：★★★** · 对应基础知识：[模仿学习_行为克隆_DAgger_扩散策略](../../02_基础知识/10_机器学习与深度学习/10_模仿学习_行为克隆_DAgger_扩散策略.md)

**答案要点**
- 策略一次预测未来 $k$ 步动作序列（chunk），而不是单步动作；执行时可开环执行整块或滚动重规划。
- 好处：①**缩短有效决策时域**（$T/k$），减轻误差累积；②对示教中的停顿、噪声更鲁棒；③动作更平滑、时间相关性被建模；④降低推理频率需求（每 $k$ 步推理一次）。
- ACT（Action Chunking with Transformers）：Transformer + 条件 VAE（隐变量建模示教风格），在低成本双臂遥操作平台上学习精细操作。
- **时间集成**（temporal ensembling）：每步都预测，对重叠的多个预测按指数权重加权平均，得到平滑且反应及时的动作；代价是每步推理。
- 注意：chunk 过长对扰动反应慢；过短则失去好处。

**English short answer**
Instead of predicting one action, the policy predicts a short sequence of future actions. This shortens the effective horizon so compounding errors shrink, makes the trajectory smoother and tolerates pauses in demonstrations, and it reduces how often inference is needed. ACT pairs this with a transformer and a conditional VAE, and temporal ensembling averages overlapping predictions for smooth, responsive control. Very long chunks react slowly to disturbances.

（译：策略不预测单步动作而预测未来一小段动作序列，这缩短有效时域从而减小误差累积，使轨迹更平滑并容忍示教中的停顿，还降低推理频率需求；ACT 把它与 Transformer 和条件 VAE 结合，时间集成对重叠预测加权平均以获得平滑而灵敏的控制；过长的 chunk 对扰动反应慢。）

**追问方向**：时间集成的权重如何设定；chunk 长度与控制频率的关系。

### Q14 扩散策略（Diffusion Policy）为什么有效？（Diffusion policy）
**优先级：★★★** · 对应基础知识：[模仿学习_行为克隆_DAgger_扩散策略](../../02_基础知识/10_机器学习与深度学习/10_模仿学习_行为克隆_DAgger_扩散策略.md)

**答案要点**
- 把策略表示为**以观测为条件的动作序列去噪过程**：从噪声动作序列出发，迭代去噪得到动作 chunk；训练是条件去噪损失，不是直接回归。
- 优点：①天然表达**多模态**动作分布；②输出高维动作序列且**时间一致**；③训练稳定（避免能量模型的负采样不稳）；④对观测噪声较鲁棒；⑤动作序列预测 + 滚动时域控制。
- 实现：CNN（1D U-Net）或 Transformer 去噪器，观测经视觉编码器（ResNet）输入，FiLM 条件化；推理用 DDIM 少步。
- 代价：推理需多步网络前向（延迟），需 GPU；对控制频率高的任务要降步数或蒸馏、异步推理。
- 对比 ACT：ACT 单次前向、更快；扩散策略建模多模态更强（具体对比因任务而异，数据未核对）。

**English short answer**
Diffusion Policy represents the policy as a conditional denoising process that turns noise into an action sequence given the observation. It naturally captures multimodal action distributions, produces temporally consistent action chunks, trains stably, and works with receding-horizon execution. The cost is iterative inference, so I use few DDIM steps, distillation or asynchronous inference to meet the control rate.

（译：扩散策略把策略表示为以观测为条件的去噪过程，把噪声变成动作序列；天然刻画多模态动作分布、输出时间一致的动作 chunk、训练稳定、配合滚动时域执行；代价是推理需迭代，用少步 DDIM、蒸馏或异步推理满足控制频率。）

**追问方向**：为何不直接回归；推理延迟如何管理。

### Q15 逆强化学习（IRL）与 GAIL 的思想？（IRL and GAIL）
**优先级：★★** · 对应基础知识：[模仿学习_行为克隆_DAgger_扩散策略](../../02_基础知识/10_机器学习与深度学习/10_模仿学习_行为克隆_DAgger_扩散策略.md)

**答案要点**
- IRL：从专家演示反推**奖励函数**，使专家行为在该奖励下近似最优，再用 RL 训练策略；能更好泛化（奖励比策略更"可迁移"），但每次迭代需解一个 RL，昂贵且奖励不唯一（歧义）。最大熵 IRL 用最大熵原理解决歧义。
- **GAIL**：不显式恢复奖励，用判别器区分专家与策略的状态–动作分布，判别器输出作为奖励训练策略（对抗式分布匹配）；需要环境交互，训练不稳定。
- 与 BC 对比：BC 离线、简单但有协变量偏移；IRL/GAIL 通过交互匹配状态分布，抗偏移但需要大量交互样本和仿真。
- 现状：机器人真机上 BC / 扩散策略更常用；GAIL/AMP（对抗运动先验）在仿真中学习风格化运动（人形、四足）常见。

**English short answer**
Inverse RL infers a reward function under which the demonstrations look optimal and then trains a policy with RL, which can generalize better but is expensive and ambiguous. GAIL skips the explicit reward and trains a discriminator between expert and policy state-action distributions, using it as the reward. Both need environment interaction, while behavior cloning is offline and simpler, and adversarial motion priors are popular for stylized locomotion in simulation.

（译：逆强化学习推断使示教看起来最优的奖励再用 RL 训练策略，泛化可能更好但昂贵且存在歧义；GAIL 不显式恢复奖励，而用区分专家与策略状态–动作分布的判别器输出作奖励；二者都需要环境交互，而行为克隆是离线且更简单的；对抗运动先验常用于仿真中的风格化运动学习。）

**追问方向**：最大熵 IRL 的核心思想；AMP 如何表达运动风格。

### Q16 示教数据怎么采集？遥操作系统有哪些？（Collecting demonstrations）
**优先级：★★★** · 对应基础知识：[机械臂动力学_重力补偿_URDF_KDL](../../02_基础知识/03_运动学与坐标变换/06_机械臂动力学_重力补偿_URDF_KDL.md)

**答案要点**
- 方式：**遥操作**（主从臂 leader–follower、VR 手柄 / 手套、空间鼠标、GELLO 类低成本主臂、外骨骼）、**动觉示教**（kinesthetic teaching，直接拖动机械臂，需重力补偿 / 阻抗模式）、**视频 / 人手示范**（手部姿态重定向）、**可穿戴 / 手持夹爪**（UMI 类，无需机器人，采集便携）。
- 质量要点：动作一致性（同任务风格统一）、覆盖（初始位置、物体位姿、光照多样）、包含恢复行为、时间同步（相机 / 关节 / 动作）、对齐的动作定义（绝对 vs 相对、关节 vs 末端）。
- 成本：一个技能通常需数十到数百条示教（因任务而异，具体数字未核对）；数据质量比数量更关键。
- 与 P2 的联系：双臂遥操作是此类数据采集的基础设施，系统延迟与映射精度直接影响数据质量。

**English short answer**
Demonstrations come from teleoperation with leader-follower arms, VR controllers or low-cost leader devices, from kinesthetic teaching by physically guiding the arm, or from handheld grippers and human video retargeted to the robot. Quality depends on consistent style, coverage of start poses and object placements, recovery behaviors and tight time synchronization. Teleoperation latency and mapping accuracy directly shape data quality.

（译：示教数据来自主从臂/VR 手柄/低成本主臂的遥操作、拖动机械臂的动觉示教，或手持夹爪与重定向到机器人的人手视频；质量取决于风格一致、起始位姿与物体摆放的覆盖、恢复行为及严格时间同步；遥操作的延迟与映射精度直接影响数据质量。）

**追问方向**：leader-follower 的映射与延迟；动觉示教对力控的要求。

### Q17 动作空间怎么选：关节、末端位姿、增量还是绝对？（Action space design）
**优先级：★★★** · 对应基础知识：[正逆运动学与雅可比](../../02_基础知识/03_运动学与坐标变换/03_正逆运动学与雅可比.md)

**答案要点**
- **关节位置**：直接、无需 IK、避免奇异性问题；跨机器人迁移差。**末端位姿（EE pose）**：与任务空间对齐，跨本体迁移更好，需 IK / 操作空间控制，可能遇奇异。**速度 / 增量**：对绝对误差更鲁棒、与观测偏移不敏感，但会累积漂移；**绝对目标**：不累积，但对标定敏感。
- 常见实践：策略输出相对末端增量或绝对位置目标，由底层阻抗 / 位置控制器跟随；夹爪单独一维（连续或开合）。
- 归一化：每个维度按训练集统计量归一化到 $[-1,1]$；旋转用 6D 表示或四元数（避免欧拉角不连续）。
- 控制频率：策略输出频率（5–20 Hz）与底层控制（数百 Hz–kHz）分层，低频目标 + 高频跟踪 + 插值。

**English short answer**
Joint-position actions are direct and avoid inverse kinematics but transfer poorly across robots. End-effector pose actions align with the task and transfer better but need IK and can hit singularities. Delta actions are robust to calibration offsets yet can drift, absolute targets do not drift but are sensitive to calibration. I normalize each dimension, represent rotation continuously, and let a high-rate low-level controller track the low-rate policy output.

（译：关节位置动作直接、免 IK 但跨机器人迁移差；末端位姿动作与任务对齐、迁移更好但需 IK 且可能遇奇异；增量动作对标定偏移鲁棒但会漂移，绝对目标不漂移但对标定敏感；对每维归一化、用连续旋转表示，并让高频底层控制器跟踪低频策略输出。）

**追问方向**：旋转的 6D 连续表示；增量动作的漂移如何缓解。

### Q18 视觉编码器与观测设计怎么选？（Observation and visual encoders）
**优先级：★★** · 对应基础知识：[目标检测_分割_跟踪_评价指标](../../02_基础知识/09_感知与视觉/04_目标检测_分割_跟踪_评价指标.md)

**答案要点**
- 输入：多视角 RGB（腕部相机 + 第三视角）、本体感觉（关节角、夹爪状态）、历史帧或状态；深度 / 点云可选。
- 编码器：ResNet-18 等小 CNN（常用 GroupNorm 替代 BatchNorm 以稳定小批量训练）、预训练的 ViT / DINOv2 / CLIP / R3M / VC-1 / SigLIP；是否冻结取决于数据量（数据少时冻结或低学习率微调）。
- 数据增强：随机裁剪 / 平移（强力有效）、颜色抖动；有助于对相机位姿微小变化的鲁棒性。
- 观测历史：堆叠 1–2 帧即可，过长历史易引入因果混淆（策略"抄"上一时刻动作）。
- 对延迟敏感：保证图像、关节状态时间对齐，训练与部署使用相同的预处理（尺寸、裁剪、色彩空间）。

**English short answer**
Policies typically take wrist and external camera images plus proprioception. Encoders are small ResNets with group normalization or pretrained ViTs such as DINOv2, frozen or lightly fine-tuned depending on data size. Random crop and shift augmentation matters a lot for robustness. I keep history short to avoid copying previous actions, and I keep preprocessing and time alignment identical between training and deployment.

（译：策略通常以腕部与外部相机图像加本体感觉为输入；编码器用带组归一化的小 ResNet 或 DINOv2 等预训练 ViT，视数据量冻结或轻度微调；随机裁剪/平移增强对鲁棒性很重要；历史帧要短以免抄上一步动作；训练与部署的预处理和时间对齐必须一致。）

**追问方向**：为什么用 GroupNorm；因果混淆的表现。

### Q19 因果混淆（causal confusion）与"抄袭上一动作"问题？（Causal confusion）
**优先级：★★** · 对应基础知识：[模仿学习_行为克隆_DAgger_扩散策略](../../02_基础知识/10_机器学习与深度学习/10_模仿学习_行为克隆_DAgger_扩散策略.md)

**答案要点**
- 现象：模仿学习中，加入历史观测 / 动作后训练损失下降快，但部署时性能变差——策略学到了"当前动作≈上一动作"的捷径（动作自相关强），而非依据场景决策；或利用与动作相关但非因果的特征（如指示灯、刹车灯）。
- 原因：专家动作随时间平滑，历史信息与目标动作高度相关；分布偏移后捷径失效。
- 缓解：减少或丢弃历史动作输入、对历史做 dropout、动作分块 + 少量帧堆叠、干预式数据收集（DAgger）、因果表示学习、更多样的数据。
- 诊断：对比有无历史输入的开环验证损失与闭环成功率；闭环评测才是真指标（开环 MSE 不能代表性能）。

**English short answer**
When the policy sees past observations or actions, it can learn the shortcut that the next action equals the previous one, which lowers training loss but fails in closed loop once small deviations occur. I limit or drop history, apply dropout on history inputs, use action chunking, and collect interventional data with DAgger. Closed-loop success, not open-loop error, is the metric that reveals this.

（译：策略能看到历史观测/动作时，可能学到"下一动作等于上一动作"的捷径，训练损失降低但闭环时一旦出现偏差就失败；对策是限制或丢弃历史、对历史输入做 dropout、用动作分块、用 DAgger 收集干预数据；闭环成功率而非开环误差才能暴露该问题。）

**追问方向**：开环误差与闭环成功率为何不一致；历史输入 dropout 的做法。

### Q20 隐式/特权信息蒸馏：Teacher–Student 在机器人中的用法？（Privileged teacher-student distillation）
**优先级：★★** · 对应基础知识：[强化学习与具身智能入门](../../02_基础知识/10_机器学习与深度学习/06_强化学习与具身智能入门.md)

**答案要点**
- 在仿真中训练一个能访问**特权信息**（地形高度图、摩擦系数、质量、接触力、物体精确位姿）的教师策略（常用 RL），再把它蒸馏（DAgger 式、在学生策略自己的状态分布上）到只使用**可部署观测**（本体感觉历史、图像）的学生策略。
- 例：四足行走（ANYmal / RMA 类，学生用本体感觉历史隐式估计地形和动力学参数）、灵巧手操作的视觉学生。
- 好处：绕过直接在部分可观测下用 RL 学习视觉策略的困难；教师使用的特权信息使学习更易。
- 注意：学生必须真的能从观测中恢复所需信息，否则存在不可约的差距；蒸馏使用在线数据（DAgger）才能避免分布偏移。

**English short answer**
A teacher policy trained in simulation with privileged information such as terrain height, friction and exact object poses is distilled into a student that only uses deployable observations like proprioceptive history or images. This avoids the difficulty of learning vision-based control directly with RL. The student must be able to infer the needed hidden quantities from its inputs, and on-policy distillation avoids distribution shift.

（译：在仿真中用地形高度、摩擦、精确物体位姿等特权信息训练教师策略，再蒸馏给只用本体感觉历史或图像等可部署观测的学生，绕开直接用 RL 学习视觉控制的困难；学生必须能从输入推断所需隐藏量，并用在线蒸馏避免分布偏移。）

**追问方向**：RMA 的适应模块；学生与教师性能差距的来源。

### Q21 示教数据的 scaling：多少数据够？怎么提升数据效率？（Data scaling and efficiency）
**优先级：★★** · 对应基础知识：[世界模型与具身基础模型综述](../../02_基础知识/10_机器学习与深度学习/14_世界模型与具身基础模型综述.md)

**答案要点**
- 经验观察：成功率随示教数量上升但边际递减；泛化（新物体位姿、背景、光照）所需数据远多于单一场景。成功依赖**多样性**比单纯数量更重要（具体数值未核对）。
- 提升效率：预训练视觉表征 / 多任务预训练 + 少量任务微调；数据增强；动作分块 / 扩散等强表示；仿真数据混合真实数据；人视频 / 手持设备便宜采集；自动数据扩增（轨迹回放 + 物体位姿变化，如 MimicGen 类）。
- 共训练（co-training）：在互联网图文数据和机器人数据上联合训练，保持语义泛化能力（RT-2 类思路）。
- 质量控制：筛掉失败 / 犹豫的示教，统一示教风格；记录元数据（操作者、日期、设备）用于分析。

**English short answer**
Success rises with demonstrations but with diminishing returns, and diversity of object poses, backgrounds and lighting matters more than raw count. To improve efficiency I use pretrained visual representations, multitask pretraining, augmentation, expressive policy classes, simulation data mixed with real data, cheap human-collected data and automated trajectory augmentation. Co-training on web vision-language data preserves semantic generalization.

（译：成功率随示教数量上升但边际递减，物体位姿/背景/光照的多样性比数量更重要；提升效率可用预训练视觉表征、多任务预训练、数据增强、强表达策略、仿真与真实数据混合、廉价的人类采集数据和自动轨迹扩增；与网络图文数据共训练可保持语义泛化。）

**追问方向**：MimicGen 类自动扩增的原理；共训练的数据配比。

### Q22 模仿学习策略如何评估？（Evaluating imitation policies）
**优先级：★★★** · 对应基础知识：[机器学习基础](../../02_基础知识/10_机器学习与深度学习/01_机器学习基础.md)

**答案要点**
- **闭环成功率**为核心：固定一组评测初始条件（物体位姿网格 / 随机种子），多次试验（每个条件 ≥10–50 次，视任务而定），报告成功率及置信区间（如 Wilson 区间）。
- 辅助：完成时间、轨迹平滑度（jerk）、碰撞 / 安全事件数、人工干预次数、失败模式分类。
- 避免：只看验证集动作误差（开环）；只展示成功视频；在训练物体位姿上评测却声称泛化。
- 泛化轴：新物体实例、新位置、新背景、新光照、干扰物、扰动恢复。
- 对比实验：同一评测协议下比较不同策略；仿真中便于大规模统计，真机评测需谨慎安排（顺序随机化 A/B 交替，减少环境漂移影响）。

**English short answer**
I evaluate by closed-loop success rate over a fixed set of initial conditions with many trials per condition and report confidence intervals, alongside completion time, smoothness, safety events, interventions and categorized failure modes. Open-loop validation error is not a reliable indicator. I test generalization along new objects, positions, backgrounds and lighting, and on real robots I interleave policies in randomized order to avoid drift in conditions.

（译：以固定初始条件集上的闭环成功率为核心，每个条件多次试验并报告置信区间，辅以完成时间、平滑度、安全事件、干预次数和失败模式分类；开环验证误差不可靠；沿新物体/位置/背景/光照测试泛化；真机上随机交错比较不同策略以避免条件漂移。）

**追问方向**：试验次数如何确定；真机评测的随机化设计。

---

## 三、VLA 与具身基础模型

### Q23 什么是 VLA？与传统"感知–规划–控制"流水线有何不同？（VLA vs modular pipeline）
**优先级：★★★** · 对应基础知识：[大语言模型预训练_微调_RAG与VLA](../../02_基础知识/10_机器学习与深度学习/08_大语言模型预训练_微调_RAG与VLA.md)

**答案要点**
- VLA（Vision-Language-Action）：以预训练视觉语言模型（VLM）为骨干，输入图像 + 语言指令（+ 本体状态），**直接输出机器人动作**（token 或连续动作头），端到端。
- 传统流水线：感知（检测 / 位姿估计）→ 任务 / 运动规划 → 控制，每个模块手工设计，可解释、可验证、安全边界清晰，但接口脆弱、需大量工程、泛化到新语义困难。
- VLA 优势：继承互联网语义知识（识别新物体、理解指令）、统一接口、可通过数据扩展；劣势：动作精度与实时性、可解释性与安全保证弱、数据昂贵、分布外行为不可预测。
- 实践：多采用**分层 / 混合**——VLA 做高层语义理解与子目标，底层仍用经典控制与安全层兜底。

**English short answer**
A VLA builds on a pretrained vision-language model and maps images, a language instruction and robot state directly to actions. A modular pipeline separates perception, planning and control, which is interpretable and easier to verify but brittle and laborious to extend semantically. VLAs inherit web-scale semantic knowledge but are weaker in precision, latency and safety guarantees, so practical systems often combine a VLA for high-level intent with classical control and safety layers.

（译：VLA 以预训练视觉语言模型为基础，把图像、语言指令和机器人状态直接映射为动作；模块化流水线分离感知、规划、控制，可解释、易验证但脆弱且语义扩展费力；VLA 继承网络级语义知识，但精度、延迟和安全保证较弱，实际系统常以 VLA 处理高层意图、经典控制与安全层兜底。）

**追问方向**：何时选 VLA、何时选模块化；分层架构如何分工。

### Q24 VLA 的动作怎么表示？离散 token 还是连续头？（Action representations in VLAs）
**优先级：★★★** · 对应基础知识：[大语言模型预训练_微调_RAG与VLA](../../02_基础知识/10_机器学习与深度学习/08_大语言模型预训练_微调_RAG与VLA.md)

**答案要点**
- **离散化为 token**（RT-2 / OpenVLA 一类）：把每个动作维度分箱（如 256 档）映射成词表中的 token，沿用语言模型的自回归交叉熵训练，实现简单、可共用 LLM 基础设施；缺点：分箱精度有限、逐 token 自回归解码慢、动作维度多时序列长。
- **连续动作头**：在 VLM 骨干后接扩散 / 流匹配头（$\pi_0$ 类）或回归 / 混合密度头，直接输出连续动作 chunk；精度和频率更好，可做高频灵巧操作。
- **压缩 / 频域 token**（如 FAST 类，基于 DCT + BPE 的动作 tokenizer，细节未核对）：减少 token 数、保留平滑性。
- 选择：任务精度要求高、频率高（灵巧操作）→ 连续头；追求与语言模型统一与简单 → 离散 token。
- 注意动作归一化与反归一化、跨本体的动作空间对齐。

**English short answer**
Some VLAs discretize each action dimension into bins and treat them as vocabulary tokens, reusing language-model training and inference, at the cost of limited precision and slow token-by-token decoding. Others attach a continuous head based on diffusion or flow matching that outputs smooth action chunks at higher rates, which suits dexterous manipulation. Compressed action tokenizers try to get the best of both. I choose by precision and frequency requirements.

（译：有些 VLA 把每个动作维度分箱并作为词表 token，复用语言模型的训练与推理，代价是精度有限且逐 token 解码慢；另一些接扩散或流匹配连续动作头，以更高频率输出平滑动作 chunk，适合灵巧操作；压缩动作 tokenizer 试图兼得；按精度与频率需求选择。）

**追问方向**：分箱数量对精度的影响；连续头与 VLM 骨干的联合训练。

### Q25 RT-1、RT-2、OpenVLA、$\pi_0$ 的思想脉络？（Landmark robot foundation models）
**优先级：★★** · 对应基础知识：[世界模型与具身基础模型综述](../../02_基础知识/10_机器学习与深度学习/14_世界模型与具身基础模型综述.md)

**答案要点**
- **RT-1**：Transformer 策略，在大规模真机示教数据上训练，图像 + 语言 → 离散化动作 token；证明大数据 + Transformer 可做多任务。
- **RT-2**：把预训练 VLM 与机器人动作数据**共训练**，动作以文本 token 表示；展示了语义泛化与基本推理（对未见物体 / 指令）。
- **OpenVLA**：开源 VLA，基于开放 VLM 骨干 + 开放的多机器人数据集（Open X-Embodiment）训练；支持 LoRA 等高效微调（具体规模未核对）。
- **$\pi_0$ 系列**：VLM 骨干 + 流匹配动作专家，面向高频灵巧操作与跨本体预训练（细节未核对）。
- 共同趋势：更大更多样的数据、预训练 VLM、统一多本体、更强的动作头。
- 面试提醒：只讲思想与对比，不报具体数字；承认自己不确定的细节。

**English short answer**
RT-1 showed that a transformer trained on large-scale real robot demonstrations can perform many tasks with discretized actions. RT-2 co-trained a pretrained vision-language model on web data and robot data, expressing actions as text tokens, which transferred semantic knowledge. OpenVLA made an open model trained on a multi-robot dataset with efficient fine-tuning, and pi-zero adds a flow-matching action expert for fast dexterous control. The trend is bigger diverse data, pretrained backbones and stronger action heads.

（译：RT-1 表明在大规模真机示教上训练的 Transformer 可用离散化动作完成多任务；RT-2 把预训练视觉语言模型与网络数据及机器人数据共训练，动作用文本 token 表示，迁移了语义知识；OpenVLA 提供在多机器人数据集上训练且支持高效微调的开源模型；$\pi_0$ 增加流匹配动作专家以实现快速灵巧控制；趋势是更大更多样的数据、预训练骨干和更强的动作头。）

**追问方向**：共训练为何保持语义泛化；跨本体训练的难点。

### Q26 跨本体（cross-embodiment）学习的挑战是什么？（Cross-embodiment learning）
**优先级：★★** · 对应基础知识：[世界模型与具身基础模型综述](../../02_基础知识/10_机器学习与深度学习/14_世界模型与具身基础模型综述.md)

**答案要点**
- 不同机器人在自由度、运动学、动作空间、相机配置、控制频率上差异很大，数据难以直接混合。
- 方法：统一动作表示（末端位姿 + 夹爪，归一化）、本体 / 相机配置作为条件输入（embodiment embedding）、按本体分动作头、大规模混合数据集（Open X-Embodiment 汇集多机构数据）。
- 收益：正迁移（一个机器人的技能 / 视觉先验帮助另一个）；风险：负迁移、数据分布不均（某机器人数据占主导）。
- 现实：动作空间对齐和标定差异（相机外参、夹爪开合）常是主要的工程问题；零样本迁移到全新本体仍困难，通常需少量微调。
- 面试表述：说清"统一接口 + 本体条件 + 微调"三步。

**English short answer**
Robots differ in degrees of freedom, kinematics, action spaces, camera setups and control rates, so their data cannot simply be pooled. Approaches include a unified normalized action representation such as end-effector pose plus gripper, conditioning on embodiment, embodiment-specific heads and large mixed datasets. Positive transfer is possible, but calibration and action-space alignment are major engineering issues and zero-shot transfer to a new body is still hard, so small fine-tuning sets are typical.

（译：不同机器人在自由度、运动学、动作空间、相机配置和控制频率上差异很大，数据不能简单混合；方法有统一归一化动作表示（末端位姿加夹爪）、本体条件化、按本体分动作头和大规模混合数据集；存在正迁移，但标定与动作空间对齐是主要工程问题，零样本迁移到新本体仍难，通常需少量微调。）

**追问方向**：负迁移的表现；新本体的最小微调数据量。

### Q27 语言指令在策略中如何起作用？（Language conditioning）
**优先级：★★** · 对应基础知识：[大语言模型预训练_微调_RAG与VLA](../../02_基础知识/10_机器学习与深度学习/08_大语言模型预训练_微调_RAG与VLA.md)

**答案要点**
- 编码：预训练文本编码器（CLIP / T5 / 句向量）或 VLM 内部处理；注入方式：FiLM 条件化视觉特征、拼接 token、交叉注意力。
- 能力层次：①任务选择（"拿起苹果"）；②组合泛化（新物体 / 新属性组合）；③指令推理（"给我能切东西的"）；④多步长时程指令。
- 难点：动作数据中的语言标注稀少且重复；模型易忽略语言（仅依据视觉场景决策——"语言被忽略"）；歧义指令。
- 改进：重新标注 / 自动生成多样语言描述（用 VLM 对示教视频打标）、保留预训练 VLM 的语言能力（冻结或共训练）、训练中语言扰动。
- 评估：同一场景不同指令是否产生不同行为（指令遵循率）。

**English short answer**
Language is encoded by a pretrained text encoder or by the VLM itself and injected through FiLM, token concatenation or cross-attention. Policies range from simple task selection to compositional generalization and reasoning over instructions. A common failure is ignoring language and acting from the visual scene alone, so I diversify instructions with automatic relabeling, preserve the VLM's language ability through co-training, and test instruction-following by changing the instruction in the same scene.

（译：语言由预训练文本编码器或 VLM 自身编码，经 FiLM、token 拼接或交叉注意力注入；能力从简单任务选择到组合泛化和指令推理；常见失败是忽略语言仅按视觉场景行动，所以用自动重标注丰富指令、通过共训练保持 VLM 语言能力，并在同一场景改变指令来测试指令遵循。）

**追问方向**：语言被忽略的诊断；自动重标注的质量控制。

### Q28 分层 VLA / 高低层策略分工是怎样的？（Hierarchical policies）
**优先级：★★** · 对应基础知识：[行为树与任务规划_PDDL_有限状态机](../../02_基础知识/08_规划与导航/08_行为树与任务规划_PDDL_有限状态机.md)

**答案要点**
- 高层（大模型 / VLM，低频 1–几 Hz）：理解指令、场景推理、任务分解、输出子目标（语言子任务、关键点、航点、可供性）；低层（小而快的策略，10–100+ Hz）：执行子目标的闭环控制。
- 优点：把慢而强的推理与快而精的控制解耦，满足实时性；高层可替换 / 可验证；低层可单独用示教或 RL 训练。
- 接口设计是关键：子目标的表示（语言 / 关键点 / 图像目标）决定低层可执行性与泛化。
- 失败：高层输出不可执行或与场景不一致；高低层时间尺度错配；需要反馈（执行结果回传高层重规划）。
- 示例思路：SayCan（语言模型打分 × 可行性价值函数）、Code as Policies（LLM 生成调用感知 / 控制 API 的代码）、VoxPoser（价值图）等。

**English short answer**
A hierarchy lets a slow, powerful model interpret the instruction, decompose the task and emit subgoals such as language steps, keypoints or waypoints, while a fast low-level policy executes them in closed loop. This decouples reasoning from precise control and meets real-time constraints. The subgoal interface determines executability, and execution feedback must return to the planner for replanning when things go wrong.

（译：分层让慢而强的模型理解指令、分解任务并输出子目标（语言步骤、关键点、航点），快速的低层策略闭环执行；这使推理与精确控制解耦并满足实时性；子目标接口决定可执行性，执行出错时反馈必须回传高层重规划。）

**追问方向**：SayCan 的可供性打分；高低层频率不匹配的处理。

### Q29 机器人世界模型（World Model）是什么？有什么用？（World models）
**优先级：★★** · 对应基础知识：[基于模型的强化学习_离线RL_与MPC结合](../../02_基础知识/10_机器学习与深度学习/13_基于模型的强化学习_离线RL_与MPC结合.md)

**答案要点**
- 世界模型预测"在当前状态采取动作后，未来观测 / 潜状态是什么"：$\hat z_{t+1}=f(z_t,a_t)$（常在潜空间，如 Dreamer 系列的 RSSM），或直接预测视频（视频扩散 / 自回归世界模型）。
- 用途：①基于模型的规划（在想象中搜索 / MPC）；②在想象 rollout 中训练策略（低成本数据）；③作为神经仿真器做策略评估、数据增强；④表征学习。
- 难点：长时域误差累积、接触丰富场景预测不准、对动作的因果性（可控性）、计算开销、"幻觉"物理。
- 与经典物理仿真对比：物理仿真精确可解释但需建模；学习的世界模型从数据学、可处理视觉、但不保证物理一致性与安全边界。

**English short answer**
A world model predicts how the observation or latent state evolves under an action, either in a learned latent space or as video. It enables planning by imagination, training policies on imagined rollouts, acting as a neural simulator for policy evaluation, and representation learning. Challenges are compounding long-horizon errors, contact-rich dynamics and physical inconsistency. Unlike physics simulators it learns from data and handles vision but offers no guarantees.

（译：世界模型预测动作作用下观测/潜状态如何演化，可在学习的潜空间或视频层面进行；用于想象中的规划、在想象 rollout 上训练策略、充当神经仿真器评估策略及表征学习；难点是长时域误差累积、接触丰富的动力学和物理不一致；与物理仿真不同，它从数据学习、能处理视觉，但不提供保证。）

**追问方向**：RSSM 的结构；神经仿真器用于策略评估的可信度。

### Q30 为什么 VLA 的推理延迟是难题？怎么优化？（Latency of VLAs）
**优先级：★★★** · 对应基础知识：[大模型训练与推理工程](../../02_基础知识/10_机器学习与深度学习/15_大模型训练与推理工程.md)

**答案要点**
- 大模型（数十亿参数）单次推理几十到几百毫秒量级（硬件相关，数值未核对），而灵巧操作需 10–50 Hz 甚至更高的控制；自回归动作 token 解码更慢。
- 优化：①动作 chunk（一次预测 $k$ 步，摊薄推理）；②**异步 / 解耦推理**（模型在后台推理，控制器按上一块动作继续执行，新块到达后平滑衔接，需处理延迟补偿）；③量化（INT8/INT4）、KV cache、TensorRT / vLLM 部署；④小模型 / 蒸馏；⑤连续头少步采样；⑥高低层分层。
- 延迟补偿：预测时考虑推理时延，对观测时间戳回溯，用时间集成平滑。
- 安全：推理超时必须有回退（保持当前位姿 / 低层安全控制）。

**English short answer**
Billion-parameter models need tens to hundreds of milliseconds per inference, yet dexterous control wants high rates. I amortize inference with action chunking, run inference asynchronously while the controller keeps executing the previous chunk with blending, quantize and optimize the runtime, distill smaller models, use few-step continuous heads and split the system hierarchically. Timeouts must fall back to a safe low-level behavior.

（译：十亿参数模型每次推理需几十到几百毫秒，而灵巧控制需要高频率；用动作分块摊薄推理，异步推理时控制器继续执行上一块并平滑衔接，同时量化优化运行时、蒸馏小模型、使用少步连续动作头、分层架构；超时必须回退到安全的底层行为。）

**追问方向**：异步推理的衔接策略；延迟补偿的实现。

### Q31 VLA 的局限与安全问题有哪些？（Limitations and safety of VLAs）
**优先级：★★★** · 对应基础知识：[功能安全基础_急停_安全链_ISO13849与IEC61508](../../02_基础知识/13_系统工程与安全/02_功能安全基础_急停_安全链_ISO13849与IEC61508.md)

**答案要点**
- 局限：精度与力控能力弱（多为位置控制，缺触觉 / 力信息）、长时域任务易漂移、分布外行为不可预测、对相机视角 / 光照变化敏感、数据昂贵、评估困难、难以解释失败原因。
- 安全：端到端输出不具备形式化保证；应**在策略之外**设置确定性安全层——关节 / 速度 / 力限位、工作空间与碰撞检查、急停、看门狗与超时回退；高风险动作需确认；对语言指令做权限与范围校验（提示注入风险：场景文字被模型当作指令）。
- 部署原则：先仿真与低速、人在回路逐级放开；监控置信度 / 不确定性触发降级；记录全量日志用于复盘。
- 面试要点：表达"模型提议、确定性层强制执行"的架构态度，与 P6 的安全执行链思路一致（具体做法以项目文档为准）。

**English short answer**
Current VLAs are weak in precision and force control, drift on long tasks, behave unpredictably out of distribution, are sensitive to viewpoint and lighting, and are costly to train and evaluate. Because their outputs carry no formal guarantees, I put deterministic safety layers outside the model: joint, speed and force limits, workspace and collision checks, emergency stop and watchdog timeouts, plus confirmation for risky actions. I roll out gradually with a human in the loop and log everything.

（译：当前 VLA 在精度和力控上较弱、长任务易漂移、分布外行为不可预测、对视角光照敏感且训练评估昂贵；输出没有形式化保证，所以在模型之外设置确定性安全层：关节/速度/力限位、工作空间与碰撞检查、急停与看门狗超时、高风险动作确认；逐级放开并保持人在回路、全量记录日志。）

**追问方向**：场景文字的提示注入如何防御；怎样定义可接受的失败率。

### Q32 用 LLM/VLM 做机器人任务规划的优缺点？（LLM-based task planning）
**优先级：★★★** · 对应基础知识：[行为树与任务规划_PDDL_有限状态机](../../02_基础知识/08_规划与导航/08_行为树与任务规划_PDDL_有限状态机.md)

**答案要点**
- 优点：开放词汇指令理解、常识推理、任务分解、与自然语言交互、迁移新任务快（少样本提示）。
- 缺点：不理解物理 / 几何约束与可行性（"拿起桌子"）、幻觉、输出格式不稳、缺少闭环反馈、延迟高、对环境状态的感知依赖外部模块。
- 缓解：约束解码 / 结构化输出（技能库白名单 + 参数 schema）；**可供性（affordance）打分**（SayCan）；用感知把场景转成文本 / 场景图；执行结果与失败原因回传（Inner Monologue 类闭环）；形式化验证层（PDDL 规划器校验前置条件）；人在回路确认。
- 架构：LLM 做"提案"，符号规划器 / 校验层做"检验"，低层技能执行；P4 的决策链 / P6 的"提案–校验–执行–反馈"思路与之对应。

**English short answer**
LLMs offer open-vocabulary understanding, commonsense reasoning and fast adaptation to new tasks, but they ignore physical feasibility, hallucinate, produce unstable formats and lack closed-loop feedback. I constrain outputs to a whitelist of skills with schemas, score affordances, feed perception and execution results back, and validate plans with a symbolic checker. The LLM proposes, a verification layer checks, and low-level skills execute.

（译：LLM 提供开放词汇理解、常识推理和对新任务的快速适应，但忽视物理可行性、会幻觉、输出格式不稳且缺少闭环反馈；我把输出约束为带 schema 的技能白名单，做可供性打分，回传感知与执行结果，并用符号校验器验证计划；LLM 提案、校验层检验、底层技能执行。）

**追问方向**：Inner Monologue 的闭环机制；PDDL 校验的作用。

### Q33 强化学习与模仿学习如何结合？（Combining RL and imitation）
**优先级：★★★** · 对应基础知识：[强化学习进阶_策略梯度_PPO_SAC](../../02_基础知识/10_机器学习与深度学习/09_强化学习进阶_策略梯度_PPO_SAC.md)

**答案要点**
- **BC 预训练 + RL 微调**：用示教初始化策略（缓解探索难和样本效率低），再用 RL 提升成功率和鲁棒性；注意 RL 微调初期会"忘记"BC（加 KL 约束到 BC 策略或使用残差）。
- **残差 RL**：固定基策略（BC / 经典控制器），RL 学习小幅残差修正，安全且高效。
- **演示增强的 RL**：示教放入回放池（DDPGfD、RLPD 类：在线 RL 中混合离线示教数据）；奖励塑形使用示教。
- **离线 RL**：从混合质量数据中学习，需处理分布外动作（保守 Q）。
- **人类干预学习**：HIL-SERL 类，人在线纠错，样本效率高（细节未核对）。
- 选择：有示教且需要提升精度 / 速度 / 鲁棒性 → BC + RL 微调；仅需复现示教 → BC / 扩散策略。

**English short answer**
I can initialize with behavior cloning to overcome exploration and then fine-tune with RL, usually with a KL anchor to avoid forgetting. Residual RL keeps a base policy fixed and learns small corrections safely. Demonstrations can also seed the replay buffer in online RL, and offline RL learns from mixed-quality data with conservatism. Human interventions during training provide efficient corrective signal.

（译：先用行为克隆初始化以克服探索困难，再用 RL 微调（通常加 KL 锚点防遗忘）；残差 RL 固定基策略、只学小的修正，安全；示教也可放入在线 RL 的回放池；离线 RL 用保守性从混合质量数据学习；训练中的人类干预提供高效的纠正信号。）

**追问方向**：RL 微调为何会遗忘 BC；残差策略的幅度限制。

### Q34 视频预训练/人类视频如何帮助机器人学习？（Learning from human video）
**优先级：★★** · 对应基础知识：[世界模型与具身基础模型综述](../../02_基础知识/10_机器学习与深度学习/14_世界模型与具身基础模型综述.md)

**答案要点**
- 动机：机器人数据昂贵，而互联网上有海量人类操作视频。
- 方式：①**视觉表征预训练**（R3M、VC-1、DINO 类，用视频时间对比 / 语言对齐学习对操作有用的特征）；②**奖励 / 价值学习**（从视频学习进度函数）；③**手部姿态/轨迹重定向**（估计人手位姿映射到机器人末端）；④**潜动作预训练**（从无动作标签的视频中学习潜动作，再对齐到真实动作）；⑤视频生成模型预测未来画面 + 逆动力学模型取动作。
- 难点：具身差距（人手 vs 夹爪）、视角差异、缺乏力 / 动作标签、深度与尺度估计。
- 实用结论：视频预训练更多提升视觉表征和高层意图理解，精细动作仍需机器人数据。

**English short answer**
Web-scale human videos can pretrain visual representations useful for manipulation, supply reward or progress signals, provide hand trajectories that are retargeted to the robot, or train latent action models from unlabeled video. The obstacles are the embodiment gap, viewpoint differences and missing force and action labels, so in practice video mainly improves representations and high-level intent, while precise control still needs robot data.

（译：网络规模的人类视频可预训练对操作有用的视觉表征、提供奖励/进度信号、给出重定向到机器人的手部轨迹，或用于无标签视频的潜动作学习；障碍是具身差距、视角差异和缺少力/动作标签，因此视频主要提升表征与高层意图理解，精细控制仍需机器人数据。）

**追问方向**：潜动作模型如何对齐到真实动作；手部重定向的误差源。

---

## 四、策略学习的工程与部署

### Q35 策略训练中的数据归一化与增强细节？（Normalization and augmentation details）
**优先级：★★★** · 对应基础知识：[神经网络与反向传播](../../02_基础知识/10_机器学习与深度学习/02_神经网络与反向传播.md)

**答案要点**
- 动作 / 状态按训练集统计量（均值方差或 min–max 到 $[-1,1]$）归一化，部署时用**相同统计量**反归一化；统计量保存进模型文件，否则换数据集会出错。
- 图像：统一分辨率（如 224 或 96），随机裁剪（训练）/ 中心裁剪（测试）、小幅色彩抖动；不要翻转（会改变左右语义与动作方向）。
- 动作噪声注入（DART）、状态噪声增强，提升对偏差的鲁棒性。
- EMA（指数滑动平均）权重用于评测，通常显著稳定扩散策略。
- 学习率与权重衰减：AdamW + 余弦调度 + 预热；梯度裁剪。
- 数据划分：按**示教轨迹**而不是按帧划分训练 / 验证，避免相邻帧泄漏。

**English short answer**
I normalize actions and states with training-set statistics saved with the model so deployment denormalizes identically, use random crops with mild color jitter but no flips because they change action semantics, and keep an exponential moving average of weights for evaluation, which stabilizes diffusion policies. I split data by trajectory rather than by frame to avoid leakage between neighboring frames.

（译：用训练集统计量归一化动作与状态，并把统计量随模型保存以便部署时一致反归一化；用随机裁剪加轻微颜色抖动，不做翻转（会改变动作语义）；评测使用权重 EMA 可稳定扩散策略；按轨迹而非按帧划分数据以避免相邻帧泄漏。）

**追问方向**：为什么不能翻转图像；EMA 衰减率的选择。

### Q36 策略部署的实时架构：感知、推理、控制如何分线程？（Real-time deployment architecture）
**优先级：★★★** · 对应基础知识：[ROS2_Humble核心概念与ROS1对比](../../02_基础知识/12_机器人软件/04_ROS2_Humble核心概念与ROS1对比.md)

**答案要点**
- 典型分层：相机采集线程（带时间戳）→ 推理进程 / 线程（GPU，输出动作 chunk）→ 动作队列 / 插值器 → 高频控制线程（1 kHz 阻抗 / 位置控制，读最新目标）→ 机器人驱动。
- 关键点：①时间戳对齐与延迟补偿；②动作队列的有界性与新旧 chunk 的平滑衔接；③推理线程与控制线程解耦，控制线程**绝不等待**推理；④超时、NaN、越界的**安全回退**（保持当前位置 / 阻尼模式）；⑤日志（观测、动作、推理时延、队列长度）。
- 实时性：控制线程用实时调度（SCHED_FIFO）、避免动态内存分配与锁竞争；推理在非实时核。
- 通信：ROS 2 话题（QoS 选 best-effort 传图像）、共享内存（大图像零拷贝）。

**English short answer**
Camera capture, inference and the high-rate controller run in separate threads or processes: inference produces action chunks into a bounded queue, an interpolator feeds the latest targets to a real-time controller, and the controller never waits on inference. I timestamp observations, blend successive chunks, check for timeouts, NaNs and limit violations with a safe fallback, and log latency and queue depth for diagnosis.

（译：相机采集、推理和高频控制器在不同线程/进程中运行：推理把动作 chunk 放入有界队列，插值器向实时控制器提供最新目标，控制器绝不等待推理；观测打时间戳、平滑衔接相邻 chunk、检查超时/NaN/越限并安全回退，记录时延与队列深度用于诊断。）

**追问方向**：新旧 chunk 衔接的方法；控制线程优先级设置。

### Q37 失败分析：策略出错了怎么诊断？（Failure analysis for learned policies）
**优先级：★★★** · 对应基础知识：[数据与日志_可观测性_回放与复盘](../../02_基础知识/13_系统工程与安全/05_数据与日志_可观测性_回放与复盘.md)

**答案要点**
- 先分类失败：感知（目标没看到 / 位姿误差）、决策（选错子任务 / 犹豫停顿）、控制（抓不稳、碰撞、卡死）、系统（延迟、丢帧、标定漂移）、环境（光照 / 背景变化）。
- 手段：回放视频 + 动作曲线对齐；检查观测输入（是否与训练分布一致，如图像裁剪、曝光）；可视化注意力 / 特征；对比训练数据最近邻（是否缺乏该情形的数据）；开环重放示教动作验证硬件与标定；在仿真或离线数据上复现。
- 对症：补数据（针对失败情形收集示教 / 干预数据）、增强、调整 chunk 长度 / 控制频率、加入恢复示教、改进观测（加腕部相机）、加规则兜底。
- 记录：失败案例库做成回归测试集，每次迭代都在其上评测。

**English short answer**
I first categorize failures as perception, decision, control, system or environment problems. I replay video aligned with action curves, check that inputs match training preprocessing, compare against nearest training examples to find missing coverage, and replay recorded demonstrations open loop to verify hardware and calibration. The fix is usually targeted data collection, better observations or fallback rules, and failures become a regression set.

（译：先把失败归类为感知、决策、控制、系统或环境问题；回放与动作曲线对齐的视频、检查输入是否与训练预处理一致、与最近邻训练样本比较发现覆盖缺口、开环重放示教验证硬件与标定；常见对策是针对性采集数据、改进观测或规则兜底，并把失败案例沉淀为回归集。）

**追问方向**：如何区分数据不足与模型容量不足；开环重放示教的意义。

### Q38 小样本适配：LoRA 微调 VLA 与少样本示教？（Adapting foundation policies）
**优先级：★★** · 对应基础知识：[大语言模型预训练_微调_RAG与VLA](../../02_基础知识/10_机器学习与深度学习/08_大语言模型预训练_微调_RAG与VLA.md)

**答案要点**
- 全量微调大模型成本高、易遗忘；**LoRA / 适配器**只训练低秩增量，显存小、易保存多个任务头；常见做法：冻结视觉编码器与大部分 LLM 层，微调注意力投影与动作头。
- 数据：几十到几百条目标任务示教，混合少量原预训练数据（回放）以保持泛化。
- 注意：动作归一化统计量与预训练一致（或重新估计并正确处理）；学习率要小；监控语义能力是否退化（遗忘）。
- 评估：在目标任务与保留任务上同时评估，避免牺牲通用性。
- 替代：提示 / 上下文示例（目前对动作的作用有限）、检索增强（用相似示教做条件）。

**English short answer**
Full fine-tuning of a large policy is expensive and risks forgetting, so I use LoRA or adapters on attention projections and the action head, freezing most of the backbone, and train on tens to hundreds of target demonstrations mixed with some replay of the original data. I use a small learning rate, handle action normalization consistently and evaluate both on the target task and on held-out capabilities to detect forgetting.

（译：大策略全量微调昂贵且易遗忘，所以在注意力投影和动作头上用 LoRA/适配器并冻结大部分骨干，用几十到几百条目标示教加少量原数据回放训练；学习率要小、动作归一化保持一致，并同时在目标任务和保留能力上评估以发现遗忘。）

**追问方向**：LoRA 秩的选择；遗忘的检测指标。

### Q39 模拟到真实（Sim2Real）与"数据金字塔"如何组合使用？（Sim2real and the data pyramid）
**优先级：★★** · 对应基础知识：[强化学习与具身智能入门](../../02_基础知识/10_机器学习与深度学习/06_强化学习与具身智能入门.md)

**答案要点**
- 数据金字塔（概念）：底层是海量互联网文本图像视频（语义），中层是仿真与合成数据（规模大、多样、成本低、有域差距），顶层是真机数据（少、贵、最贴近真实）。
- 组合：预训练覆盖语义与视觉；仿真提供大规模动作多样性、特权信息、RL 训练；真机数据微调并校准；共训练平衡。
- 缩小域差距：域随机化（视觉、动力学参数）、系统辨识校准仿真、真实-仿真联合训练、视觉用强增强或真实图像风格迁移。
- 注意：仿真中"能做"不等于真机可做（接触建模、摩擦、延迟差异）；评估必须在真机。
- 用例：仿真中批量生成示教（运动规划器作专家）再蒸馏成视觉策略。

**English short answer**
Web data provides semantics, simulation provides scalable, diverse data with privileged information, and real robot data, scarce but most faithful, calibrates the final policy. I narrow the sim-to-real gap with domain randomization, system identification and co-training, and always evaluate on hardware because contact, friction and latency differ. A planner can generate simulated demonstrations that are distilled into a vision policy.

（译：网络数据提供语义，仿真提供可扩展的多样数据与特权信息，稀少但最真实的真机数据校准最终策略；用域随机化、系统辨识和共训练缩小仿真到真实的差距，并始终在真机评估，因为接触、摩擦和延迟不同；规划器可批量生成仿真示教，再蒸馏成视觉策略。）

**追问方向**：域随机化的范围如何确定；仿真示教与真机示教的分布差异。

---

## 五、补充：扩散的数学视角、经典示教与具身评估

### Q40 分数匹配与随机微分方程（SDE）视角下的扩散？（Score matching and SDE view）
**优先级：★★** · 对应基础知识：[序列模型_残差网络_ViT与生成模型](../../02_基础知识/10_机器学习与深度学习/07_序列模型_残差网络_ViT与生成模型.md)

**答案要点**
- 分数 $s(x)=\nabla_x\log p(x)$ 指向概率密度增大的方向；去噪分数匹配：学习带噪分布的分数 $s_\theta(x_t,t)$，与"预测噪声"仅差一个系数：$s=-\epsilon/\sigma_t$。
- 连续时间视角：前向 SDE $dx=f(x,t)dt+g(t)dw$ 把数据逐渐变成噪声；反向 SDE $dx=[f-g^2\nabla\log p_t]dt+g\,d\bar w$ 用分数把噪声变回数据；对应的**概率流 ODE** 与反向 SDE 有相同边缘分布，但是确定性的（DDIM、DPM-Solver 的理论基础）。
- 意义：统一 DDPM / NCSN / 流匹配，解释各种采样器（SDE 采样随机性大、ODE 采样可反演）。
- 面试层面：能说出"扩散 = 学习分数 + 反向时间积分"即可。

**English short answer**
The score is the gradient of the log density, and denoising score matching learns it for noisy data, which is equivalent to noise prediction up to a scale. In continuous time the forward SDE turns data into noise and the reverse SDE uses the learned score to turn noise back into data, with a deterministic probability-flow ODE sharing the same marginals. This unifies DDPM, DDIM and flow-based samplers.

（译：分数是对数密度的梯度，去噪分数匹配为带噪数据学习它，与预测噪声仅差一个尺度；连续时间下前向 SDE 把数据变成噪声，反向 SDE 用学到的分数把噪声变回数据，确定性的概率流 ODE 与之边缘分布相同；这统一了 DDPM、DDIM 与基于流的采样器。）

**追问方向**：为什么 ODE 采样可反演；SDE 与 ODE 采样的多样性差异。

### Q41 DiT（扩散 Transformer）相对 U-Net 的特点？（Diffusion Transformers）
**优先级：★★** · 对应基础知识：[序列模型_残差网络_ViT与生成模型](../../02_基础知识/10_机器学习与深度学习/07_序列模型_残差网络_ViT与生成模型.md)

**答案要点**
- 把图像 / 潜变量切成 patch 作为 token，用 Transformer 作去噪器；时间步和条件通过 **AdaLN（自适应层归一化）** 或交叉注意力注入。
- 优势：扩展性好（模型与计算越大效果越好，符合 Transformer 缩放规律）、结构统一（便于多模态 / 视频）、长程依赖建模强；U-Net 归纳偏置强（局部卷积、多尺度），小数据 / 低算力时仍有效。
- 机器人：Diffusion Policy 的去噪器可用 1D 卷积 U-Net 或 Transformer；高维动作序列、多相机 token 融合时 Transformer 灵活。
- 代价：注意力二次复杂度，token 数多时昂贵；需要大量数据才能发挥。

**English short answer**
A diffusion transformer patches the latent into tokens and denoises with a transformer, injecting timestep and condition through adaptive layer normalization or cross-attention. It scales well with model size and handles multimodal conditioning uniformly, while U-Nets carry stronger locality biases that help with little data. Robot policies use either a 1D convolutional U-Net or a transformer depending on data and sequence length.

（译：扩散 Transformer 把潜变量切成 patch token，用 Transformer 去噪，时间步和条件经自适应层归一化或交叉注意力注入；随模型规模扩展好、对多模态条件处理统一，而 U-Net 的局部性偏置在数据少时有利；机器人策略按数据与序列长度选 1D 卷积 U-Net 或 Transformer。）

**追问方向**：AdaLN 的零初始化技巧；token 数与算力的关系。

### Q42 离散 token 化（VQ-VAE/VQGAN）与自回归生成？（Discrete tokenization）
**优先级：★★** · 对应基础知识：[序列模型_残差网络_ViT与生成模型](../../02_基础知识/10_机器学习与深度学习/07_序列模型_残差网络_ViT与生成模型.md)

**答案要点**
- 用 VQ-VAE 把图像 / 音频 / 动作压缩为离散码本索引序列，再用 Transformer 自回归建模（语言模型式），可统一多模态到同一序列模型。
- 训练细节：直通估计器（straight-through）反传、承诺损失、EMA 更新码本；常见问题**码本坍缩**（只用少数码）——用重启未用码、较小码本维度、FSQ（有限标量量化）缓解。
- 在机器人：把连续动作 chunk 编码为少量离散 token（如 FAST/VQ 类动作 tokenizer），用 LLM 骨干预测；对比扩散头各有取舍。
- 优缺点：离散化易于与语言模型对接，但有量化误差，对精细连续控制不利。

**English short answer**
A VQ-VAE compresses images, audio or action chunks into sequences of discrete codebook indices that a transformer can model autoregressively, unifying modalities into one sequence model. Training uses a straight-through estimator and commitment loss, and codebook collapse is mitigated by restarts or finite scalar quantization. For actions it integrates easily with language models but adds quantization error for fine control.

（译：VQ-VAE 把图像、音频或动作 chunk 压成离散码本索引序列，供 Transformer 自回归建模，统一多模态；训练用直通估计器和承诺损失，通过重启未用码或有限标量量化缓解码本坍缩；用于动作时易与语言模型对接，但量化误差不利于精细控制。）

**追问方向**：直通估计器的原理；码本坍缩的诊断。

### Q43 用扩散做轨迹规划（Diffuser 类）与策略的区别？（Diffusion for planning）
**优先级：★★** · 对应基础知识：[基于模型的强化学习_离线RL_与MPC结合](../../02_基础知识/10_机器学习与深度学习/13_基于模型的强化学习_离线RL_与MPC结合.md)

**答案要点**
- Diffuser 类方法：对**整条状态–动作轨迹**做扩散建模，采样时用奖励 / 价值梯度或约束引导，使生成的轨迹既可行又高回报，把规划变成条件采样。
- 与扩散策略区别：策略以观测为条件**预测动作序列**，在线闭环执行；轨迹扩散则同时生成状态与动作，可显式施加目标 / 约束（goal-conditioned inpainting）。
- 优点：多模态、长时域一致、可组合约束（引导）；缺点：计算重、需要准确动力学一致性（生成的状态序列未必物理可行）、对实时闭环不友好。
- 实践：常与 MPC 结合——生成候选轨迹，再由控制器 / 优化器修正、跟踪。

**English short answer**
Trajectory diffusion models generate whole state-action trajectories and steer sampling with reward gradients or constraints, turning planning into conditional sampling. Diffusion policies instead predict actions given the current observation for closed-loop execution. Planning-style diffusion handles goals and constraints explicitly and gives long-horizon coherence, but is heavier and may produce dynamically infeasible states, so it is often paired with MPC for tracking.

（译：轨迹扩散生成整条状态–动作轨迹并用奖励梯度或约束引导采样，把规划变成条件采样；扩散策略则以当前观测为条件预测动作用于闭环执行；规划式扩散能显式处理目标与约束并保持长时域一致，但更重且可能生成动力学上不可行的状态，常配合 MPC 做跟踪。）

**追问方向**：引导采样的具体形式；动力学可行性如何保证。

### Q44 视频生成模型能否直接做机器人策略或规划？（Video models for robotics）
**优先级：★★** · 对应基础知识：[世界模型与具身基础模型综述](../../02_基础知识/10_机器学习与深度学习/14_世界模型与具身基础模型综述.md)

**答案要点**
- 思路：给定当前图像和语言目标，用视频扩散模型生成未来画面（想象的成功过程），再用**逆动力学模型**（由相邻帧推动作）或跟踪模块把画面变成动作（UniPi 类思路）。
- 优势：可利用互联网视频与大模型先验；语义泛化好；无需动作标签即可预训练视频部分。
- 挑战：推理慢；生成视频的物理一致性、几何精度；逆动力学模型的精度；闭环频率低；幻觉画面导致不可执行动作。
- 现状：多作为高层规划 / 世界模型 / 数据增强来源，精细控制仍由低层策略负责。

**English short answer**
A video diffusion model can imagine the future frames of a successful execution given the current image and goal, and an inverse dynamics model converts adjacent frames into actions. This leverages web video and semantic priors and needs no action labels for pretraining, but inference is slow, generated frames can be physically inconsistent, and closed-loop rates are low. It is mostly used for high-level planning or data generation.

（译：视频扩散模型可根据当前图像和目标想象成功执行的未来帧，再由逆动力学模型把相邻帧转换成动作；这利用网络视频和语义先验、预训练不需要动作标签，但推理慢、生成帧可能物理不一致、闭环频率低；多用于高层规划或数据生成。）

**追问方向**：逆动力学模型的训练数据；如何过滤不可行的想象。

### Q45 经典示教学习：DMP 与 ProMP 的思想？（DMP and ProMP）
**优先级：★★** · 对应基础知识：[机械臂轨迹生成_多项式_梯形速度_S曲线](../../02_基础知识/03_运动学与坐标变换/08_机械臂轨迹生成_多项式_梯形速度_S曲线.md)

**答案要点**
- **DMP（动态运动基元）**：用一个稳定的二阶系统（弹簧–阻尼，吸引到目标）加一个可学习的非线性强迫项 $f(s)$（由相位变量 $s$ 驱动、用高斯基函数加权）表示运动；强迫项由单条示教拟合，保证收敛到目标，并可调节目标、时间尺度与空间缩放。
- **ProMP（概率运动基元）**：用基函数线性表示轨迹，权重服从高斯分布，从多条示教学得均值 / 协方差；可做条件化（通过路径点）、轨迹混合、协同。
- 优缺点：样本少（一两条示教即可）、稳定性好、可解释，但表达力有限、无法利用视觉大规模泛化。
- 与现代深度模仿学习对比：DMP/ProMP 常作为运动原语层或安全收敛层，与学习策略组合。

**English short answer**
A dynamic movement primitive combines a stable spring-damper attractor toward a goal with a learned nonlinear forcing term driven by a phase variable, so one demonstration reproduces the motion while guaranteeing convergence and allowing goal, speed and scale changes. Probabilistic movement primitives model trajectory weights as a Gaussian distribution across demonstrations and support conditioning on via-points. They are sample efficient and interpretable but not very expressive.

（译：DMP 把朝向目标的稳定弹簧阻尼吸引子与由相位变量驱动的可学习非线性强迫项相结合，一条示教即可复现运动，保证收敛并可调整目标、速度和尺度；ProMP 把轨迹权重建模为跨示教的高斯分布，支持路径点条件化；二者样本效率高、可解释但表达力有限。）

**追问方向**：DMP 的相位变量作用；ProMP 的条件化公式。

### Q46 目标条件模仿与事后重标注（HER）？（Goal-conditioned imitation）
**优先级：★★** · 对应基础知识：[强化学习进阶_策略梯度_PPO_SAC](../../02_基础知识/10_机器学习与深度学习/09_强化学习进阶_策略梯度_PPO_SAC.md)

**答案要点**
- 目标条件策略 $\pi(a|o,g)$：同一网络完成多种目标（目标图像、目标位姿、语言）。
- **事后重标注**：把轨迹中后来到达的状态当作"目标"重新标注该轨迹（即使原目标失败，也是到达该状态的成功示范），大幅提高数据利用率；也用于离线 GCRL / 目标条件 BC。
- 优点：无需为每个目标单独采集；自动覆盖多目标；可与图像目标结合实现"拼接"（组合不同轨迹的片段）。
- 缺点：目标空间覆盖不足时泛化差；分布中包含大量无意义目标；对高维图像目标需要好表征。
- 机器人：玩耍数据（play data）+ 目标重标注学习通用技能。

**English short answer**
A goal-conditioned policy takes a goal such as a target image or pose. Hindsight relabeling treats states reached later in a trajectory as goals for that trajectory, turning every trajectory into successful data for some goal and greatly improving data efficiency, including for unstructured play data. It generalizes poorly if goals are poorly covered or representations are weak.

（译：目标条件策略以目标图像或位姿等作为输入；事后重标注把轨迹中后来到达的状态当作该轨迹的目标，使每条轨迹对某个目标都成为成功数据，大幅提升数据效率，也适用于无结构的玩耍数据；目标覆盖不足或表征弱时泛化差。）

**追问方向**：HER 在离线数据中的用法；目标采样分布对性能的影响。

### Q47 多任务策略：任务条件如何设计？会遇到什么问题？（Multi-task policies）
**优先级：★★** · 对应基础知识：[世界模型与具身基础模型综述](../../02_基础知识/10_机器学习与深度学习/14_世界模型与具身基础模型综述.md)

**答案要点**
- 条件形式：任务 ID（one-hot）、语言、目标图像、示例视频（one-shot imitation）；语言最灵活。
- 优势：共享视觉与运动表征，提升数据效率与泛化；一个模型部署所有技能。
- 问题：**梯度冲突 / 负迁移**（不同任务梯度方向相反）、任务数据不平衡（简单任务主导）、容量限制（模型容量不够导致相互干扰）、灾难性遗忘（新增任务）。
- 缓解：平衡采样 / 加权、更大模型、梯度手术（PCGrad）、任务专家混合（MoE）、共享骨干 + 任务头、持续学习回放。
- 评估：每个任务单独的成功率与平均值，避免平均掩盖个别任务退化。

**English short answer**
Tasks can be specified by ID, language, goal image or a demonstration. Multitask training shares representations and improves data efficiency, but gradients can conflict, easy tasks can dominate, capacity can be insufficient, and new tasks can cause forgetting. I balance sampling, scale the model, use mixture-of-experts or gradient surgery, and report per-task success to avoid averages hiding regressions.

（译：任务可用 ID、语言、目标图像或示范指定；多任务训练共享表征、提升数据效率，但梯度可能冲突、简单任务可能主导、容量可能不足、新增任务会遗忘；用平衡采样、扩大模型、专家混合或梯度手术，并报告每个任务的成功率避免平均掩盖退化。）

**追问方向**：PCGrad 的原理；任务数据不平衡的采样策略。

### Q48 力/触觉信息在模仿学习中怎么用？（Force and tactile in imitation learning）
**优先级：★★** · 对应基础知识：[力控与阻抗_导纳_混合力位控制](../../02_基础知识/05_控制理论/12_力控与阻抗_导纳_混合力位控制.md)

**答案要点**
- 接触丰富任务（插拔、擦拭、拧螺丝）仅靠视觉难以判断接触状态和力大小；加入腕部六维力 / 力矩、触觉传感器（视触觉 GelSight 类、电子皮肤）作为观测，或让策略输出**目标力 / 刚度**（变阻抗）。
- 示教采集：遥操作需力反馈（双边遥操作）才能让人示范合适的力；动觉示教天然带力信息。
- 控制：策略输出位姿 + 刚度参数，底层阻抗 / 导纳控制执行，增强接触鲁棒性。
- 挑战：传感器噪声、漂移、标定、接触高频动态与低频策略的频率差；数据同步。
- 价值：在位姿误差几毫米时仍能完成装配（顺应性）。

**English short answer**
Contact-rich tasks like insertion are hard to judge from vision alone, so I add wrist force-torque and tactile sensing as observations and let the policy command target stiffness or force through an impedance controller. Demonstrations need force feedback in teleoperation or kinesthetic teaching to capture proper forces. Sensor drift, calibration and the frequency gap between a low-rate policy and high-rate contact dynamics are the main challenges.

（译：插拔等接触丰富任务仅凭视觉难以判断，所以把腕部力/力矩和触觉作为观测，并让策略通过阻抗控制器输出目标刚度或力；示教需遥操作力反馈或动觉示教来捕捉合适的力；传感器漂移、标定及低频策略与高频接触动态的频率差是主要挑战。）

**追问方向**：变阻抗控制的参数；双边遥操作的稳定性。

### Q49 逆动力学模型与"从观测序列恢复动作"？（Inverse dynamics models）
**优先级：★★** · 对应基础知识：[机器人动力学进阶_操作空间_接触_阻抗](../../02_基础知识/02_力学与动力学建模/05_机器人动力学进阶_操作空间_接触_阻抗.md)

**答案要点**
- 逆动力学模型（IDM）$a_t=f(o_t,o_{t+1})$：由前后观测推断中间动作。训练数据可来自机器人自己的随机 / 探索交互（无需专家）。
- 用途：①给无动作标签的视频伪标注动作（训练 VPT 类模型）；②与视频生成 / 世界模型联合：想象未来 → IDM 取动作；③跟踪参考轨迹。
- 与经典概念区分：机器人学的"逆动力学"是 $\boldsymbol\tau=\mathbf{M}\ddot{\mathbf{q}}+\dots$；这里指观测空间的动作推断。
- 局限：动作歧义（不同动作产生相似观测）、需要覆盖足够的交互数据、遮挡。

**English short answer**
An inverse dynamics model infers the action taken between two consecutive observations. Trained on the robot's own interaction data, it can pseudo-label actions for unlabeled videos or convert imagined future frames from a world model into executable actions. Its weaknesses are action ambiguity, limited coverage of interaction data and occlusion. It is distinct from the rigid-body inverse dynamics that maps motion to joint torques.

（译：逆动力学模型由两个连续观测推断中间执行的动作；用机器人自身交互数据训练后，可给无标签视频伪标注动作，或把世界模型想象的未来帧转换成可执行动作；弱点是动作歧义、交互数据覆盖有限和遮挡；它不同于把运动映射为关节力矩的刚体逆动力学。）

**追问方向**：VPT 的思路；与刚体逆动力学的区别。

### Q50 常见的机器人学习基准与仿真评测怎么用？（Benchmarks）
**优先级：★★** · 对应基础知识：[世界模型与具身基础模型综述](../../02_基础知识/10_机器学习与深度学习/14_世界模型与具身基础模型综述.md)

**答案要点**
- 仿真基准（举例，细节未核对）：LIBERO、CALVIN、RLBench、robomimic、ManiSkill、Meta-World、Franka Kitchen、SimplerEnv 等，提供标准任务、示教数据与评测协议。
- 用途：算法对比的统一协议、快速迭代、消融；真实世界评测成本高。
- 注意：仿真基准成绩与真机表现的相关性不一定高（域差距、任务简化）；指标是任务成功率，需要多种子、置信区间；注意训练 / 测试分割（物体位姿是否泄漏）。
- 真机评测：建立标准化评测集（固定物体、位置网格、光照），采用盲评 + 随机顺序；公开协议便于复现。
- 面试：能说出 1–2 个基准的特点即可，不要背数字。

**English short answer**
Simulation benchmarks such as LIBERO, CALVIN, RLBench and robomimic provide standard tasks, demonstration data and evaluation protocols, enabling controlled comparisons and ablations. Their scores do not always correlate with real-robot performance, so I report multiple seeds with confidence intervals and check for train-test leakage. Real-world evaluation needs a standardized protocol with fixed object placements, randomized trial order and blind scoring.

（译：LIBERO、CALVIN、RLBench、robomimic 等仿真基准提供标准任务、示教数据与评测协议，便于受控对比和消融；其成绩与真机表现不一定相关，所以报告多种子及置信区间并检查训练-测试泄漏；真实世界评测需要固定物体摆放、随机化试验顺序和盲评的标准化协议。）

**追问方向**：仿真与真机成功率不一致的原因；如何设计真机评测协议。

### Q51 VLM 在空间推理与指向（pointing）上的局限，如何改进？（Spatial reasoning limits of VLMs）
**优先级：★★** · 对应基础知识：[3D感知_点云检测_BEV_占据网络](../../02_基础知识/09_感知与视觉/05_3D感知_点云检测_BEV_占据网络.md)

**答案要点**
- 通用 VLM 擅长语义（识别、描述），对**精确空间关系**（左 / 右、距离、3D 位置、物体可抓取点）较弱，输出的像素 / 坐标不够精确。
- 改进：用机器人 / 具身相关数据微调（指向、边界框、可供性、关键点预测任务）；输入深度 / 点云或已标定的相机内外参；用 VLM 输出 2D 点，再用深度和相机模型反投影成 3D；多视角几何一致；链式推理（先定位后规划）；与专用检测 / 分割 / 位姿估计模块配合（Grounding DINO + SAM 类）。
- 评估：指向准确率、抓取点成功率。
- 工程：把几何交给确定性模块，VLM 只做语义选择，可靠性更高。

**English short answer**
General VLMs are good at semantics but weak at precise spatial relations, distances and 3D locations. I improve them by fine-tuning on pointing, box and affordance data, lifting predicted 2D points to 3D with depth and calibrated cameras, and combining them with dedicated detectors and segmenters. A robust design leaves geometry to deterministic modules and uses the VLM only for semantic selection.

（译：通用 VLM 擅长语义，但对精确空间关系、距离与三维位置较弱；通过在指向、框和可供性数据上微调、用深度与标定相机把预测的 2D 点提升到 3D、并与专用检测分割模块结合来改进；稳健设计把几何交给确定性模块，VLM 只做语义选择。）

**追问方向**：2D 点反投影到 3D 的步骤；Grounding DINO + SAM 的流水线。

### Q52 具身推理（embodied chain-of-thought）能否提升 VLA？（Reasoning in VLAs）
**优先级：★**

**答案要点**
- 思路：让 VLA 在输出动作前先生成中间推理（子任务描述、目标物体边界框 / 关键点、末端轨迹草图），再基于推理结果预测动作；类似 LLM 的思维链。
- 好处：可解释性提高、长时域任务分解更好、可利用语言监督；失败时更容易诊断。
- 代价：推理 token 增加延迟；需要额外标注（可用 VLM / 规划器自动生成）；错误推理会传导到动作。
- 实践：高层推理低频运行，低层动作头高频运行，二者异步；具体效果依赖任务和数据（未核对）。
- 面试表述：说明"推理提升可解释性与泛化，但要权衡延迟"。

**English short answer**
A VLA can first generate intermediate reasoning such as subtasks, target boxes or keypoints and then predict actions conditioned on it, like chain-of-thought in language models. This improves interpretability and long-horizon decomposition and eases diagnosis, but adds latency, needs extra annotation and lets reasoning errors propagate. Reasoning typically runs at low frequency asynchronously from the fast action head.

（译：VLA 可先生成子任务、目标框或关键点等中间推理再据此预测动作，类似语言模型的思维链；提升可解释性和长时域分解能力并便于诊断，但增加延迟、需额外标注且推理错误会传导；推理通常低频异步运行，与高频动作头分离。）

**追问方向**：中间推理的标注自动化；推理与动作的异步实现。

### Q53 对 VLA 做 RL 微调有哪些思路与风险？（RL fine-tuning of VLAs）
**优先级：★★**

**答案要点**
- 动机：模仿学习的成功率上限受示教质量限制，RL 可通过环境奖励继续提升（精度、速度、鲁棒性）。
- 思路：在仿真或真机上用稀疏任务成功奖励 / 学习到的奖励模型；PPO / GRPO 类策略梯度微调；用 KL 约束保持与 SFT 策略接近；只微调动作头或 LoRA 参数；残差策略；人类干预作为在线信号。
- 风险：真机探索不安全且样本昂贵；奖励设计困难与奖励黑客；大模型 RL 不稳定（梯度方差、灾难性遗忘语义能力）；仿真到真机差距。
- 安全措施：受限动作范围、保守探索、人在回路、回退到 SFT 策略。
- 现状：研究热点，工业界多以示教 + 少量 RL / 人类纠错为主（未核对）。

**English short answer**
Imitation is capped by demonstration quality, so RL can improve success rate, speed and robustness using environment rewards. I fine-tune with policy gradient methods such as PPO or GRPO under a KL constraint to the supervised policy, update only the action head or LoRA parameters, or learn a residual. Risks are unsafe real-world exploration, reward hacking, instability and forgetting, so I explore conservatively with a human in the loop and a fallback policy.

（译：模仿受限于示教质量，RL 可借环境奖励提升成功率、速度和鲁棒性；在对监督策略加 KL 约束下用 PPO/GRPO 等策略梯度微调，只更新动作头或 LoRA 参数，或学习残差；风险有真机探索不安全、奖励黑客、不稳定和遗忘，所以保守探索、人在回路并保留回退策略。）

**追问方向**：KL 约束的方向；GRPO 与 PPO 的区别。

### Q54 扩散模型的训练技巧：EMA、噪声调度与损失加权？（Training tricks for diffusion）
**优先级：★★** · 对应基础知识：[序列模型_残差网络_ViT与生成模型](../../02_基础知识/10_机器学习与深度学习/07_序列模型_残差网络_ViT与生成模型.md)

**答案要点**
- **EMA 权重**（衰减 0.999–0.9999 量级）用于采样，显著改善样本质量与稳定性。
- **噪声调度**：线性、余弦、zero terminal SNR 修正；不同分辨率需调整（高分辨率需更大噪声偏移）。
- **损失加权**：不同时间步的重要性不同，min-SNR 加权、$v$-预测提高稳定；时间步采样分布（logit-normal）。
- **数据**：规范化到 $[-1,1]$；条件 dropout 以支持 CFG；机器人动作需要对每个维度归一化。
- **学习率 / 批大小 / 预热**；混合精度；梯度裁剪。
- 诊断：采样出的轨迹是否抖动 / 偏离、不同随机种子下覆盖模态是否合理。

**English short answer**
I sample with an exponential moving average of the weights, choose a noise schedule suited to the data scale, weight timesteps with schemes such as min-SNR or use v-prediction for stability, normalize inputs and drop conditions for guidance. Warm-up, gradient clipping and mixed precision are standard. For robot policies I check that sampled trajectories are smooth and cover the expected modes across seeds.

（译：采样用权重的指数滑动平均，选择适合数据尺度的噪声调度，用 min-SNR 加权或 $v$-预测提高稳定性，输入归一化并丢弃条件以支持引导；预热、梯度裁剪和混合精度为标配；对机器人策略检查采样轨迹是否平滑并在不同随机种子下覆盖预期模态。）

**追问方向**：$v$-预测的定义；min-SNR 加权的动机。

### Q55 示教中的操作者差异与数据质量控制？（Operator variability and data quality）
**优先级：★★** · 对应基础知识：[模仿学习_行为克隆_DAgger_扩散策略](../../02_基础知识/10_机器学习与深度学习/10_模仿学习_行为克隆_DAgger_扩散策略.md)

**答案要点**
- 不同操作者速度、路径、抓取点不同；混合数据导致多模态；同一人也会不一致、犹豫、纠错。
- 质量控制：制定采集规范与检查清单；培训操作者；统一速度范围；剔除失败 / 严重犹豫轨迹（或标注）；保留少量**恢复示教**（故意制造偏差再纠正）；分析数据分布（起始位姿、物体位置覆盖）。
- 数据元信息：操作者 ID、日期、环境、设备版本；便于诊断（某操作者数据导致性能下降）。
- 反思：数据量小而质量高常优于大而杂；用筛选策略（如按任务成功、轨迹平滑度、完成时间过滤）。
- 自动化：用价值函数 / 进度模型对示教评分，低分过滤。

**English short answer**
Different operators demonstrate at different speeds, paths and grasp points, and even one operator is inconsistent, so I define collection protocols, train operators, filter failed or hesitant trajectories, include a few recovery demonstrations and record metadata like operator and setup. A smaller clean dataset often beats a large noisy one. Scoring demonstrations by success, smoothness or a learned progress model helps automate filtering.

（译：不同操作者的速度、路径、抓取点不同，同一人也不一致，所以制定采集规范、培训操作者、过滤失败或犹豫的轨迹、加入少量恢复示教并记录操作者与环境等元信息；小而干净的数据往往优于大而杂的数据；按成功、平滑度或学习到的进度模型给示教打分可自动化过滤。）

**追问方向**：恢复示教怎么采集；数据过滤会不会引入偏差。

### Q56 动作抖动（jitter）与策略输出平滑化？（Action smoothing）
**优先级：★★★** · 对应基础知识：[数字滤波器_FIR_IIR_采样与混叠](../../02_基础知识/04_信号与系统/02_数字滤波器_FIR_IIR_采样与混叠.md)

**答案要点**
- 现象：学习策略的输出在相邻时刻不一致，机械臂抖动、卡顿，磨损并影响抓取。
- 原因：单步预测噪声、多模态在模态间切换、观测噪声、推理延迟变化。
- 缓解：动作分块 + 时间集成（加权平均重叠预测）、低通滤波 / EMA、输出限速限加速度、扩散策略的 chunk 一致性、训练中加动作平滑正则、降低控制环硬度（阻抗）。
- 取舍：平滑越强响应越滞后（相位滞后）；对精细接触任务要小心。
- 监测：记录动作 jerk、关节速度尖峰；设阈值触发保护（限幅 / 降速）。

**English short answer**
Learned policies can jitter because of per-step prediction noise, switching between modes, observation noise or variable latency. I use action chunking with temporal ensembling, light low-pass filtering, rate and acceleration limits and an impedance controller, accepting a small amount of lag. I monitor jerk and joint-speed spikes and trigger protective slowdowns when they exceed thresholds.

（译：学习策略因单步预测噪声、模态切换、观测噪声或延迟变化而抖动；用动作分块加时间集成、轻度低通滤波、速率与加速度限制和阻抗控制器，接受少量滞后；监控 jerk 与关节速度尖峰，超阈值时触发保护性降速。）

**追问方向**：时间集成权重的选取；限加速度如何实现。

### Q57 如何判断"需要更多数据"还是"需要更好模型"？（Data vs model diagnosis）
**优先级：★★** · 对应基础知识：[机器学习基础](../../02_基础知识/10_机器学习与深度学习/01_机器学习基础.md)

**答案要点**
- 学习曲线：增加训练数据量（25%、50%、100%）看闭环成功率是否持续上升——仍上升则缺数据；趋于饱和而训练损失仍高 → 模型 / 优化问题。
- 过拟合诊断：训练轨迹上的开环误差很低但验证轨迹上高 → 泛化不足（数据多样性不够或正则不足）。
- 失败聚类：失败集中在某类情形（新位置、光照）→ 针对性补数据；失败随机分布 → 可能是噪声 / 精度问题。
- 容量消融：更大编码器 / 更长训练是否提升；换预训练骨干是否提升。
- 经验：多数机器人任务早期瓶颈在数据多样性与质量，而不是网络结构。

**English short answer**
I plot closed-loop success against training set size: continuing improvement means I need more data, while saturation with high training error points to model or optimization limits. A large gap between training and validation open-loop error indicates poor generalization. If failures cluster in specific situations, I collect targeted data. In practice early bottlenecks are usually data diversity and quality rather than architecture.

（译：绘制闭环成功率随训练数据量变化的曲线：持续上升说明需要更多数据，趋于饱和且训练误差仍高则指向模型或优化限制；训练与验证开环误差差距大说明泛化不足；失败集中在特定情形时针对性补数据；实践中早期瓶颈通常是数据多样性与质量而非网络结构。）

**追问方向**：学习曲线的对数坐标解读；失败聚类的方法。

### Q58 为什么视觉策略对相机位姿/外观变化敏感？怎么提升鲁棒性？（Visual robustness）
**优先级：★★★** · 对应基础知识：[目标检测_分割_跟踪_评价指标](../../02_基础知识/09_感知与视觉/04_目标检测_分割_跟踪_评价指标.md)

**答案要点**
- 原因：策略学到的是与训练视角 / 光照 / 背景绑定的视觉特征；相机位置漂移几厘米、光照变化、桌布颜色变化都会造成分布偏移。
- 提升：①数据多样性（多视角、光照、背景、干扰物）；②强数据增强（随机裁剪 / 平移、颜色抖动、随机擦除）；③预训练视觉表征（DINOv2、CLIP 等）；④多相机冗余（腕部相机对外部相机漂移鲁棒）；⑤对物体用分割 / 关键点等不变性更强的中间表示；⑥标定并固定相机安装，定期复检。
- 仿真中域随机化；真机采用同一相机型号与曝光设置。
- 监控：部署前的"环境健康检查"（采集一帧与训练分布对比）。

**English short answer**
Visual policies overfit to the training viewpoint, lighting and background, so small camera shifts or color changes cause distribution shift. I improve robustness with diverse data, strong crop and color augmentation, pretrained visual representations, redundant cameras such as a wrist camera, and invariant intermediate representations such as keypoints. I also fix and recheck camera mounting and run a pre-deployment image check against the training distribution.

（译：视觉策略会过拟合训练视角、光照和背景，相机轻微偏移或颜色变化就造成分布偏移；用多样数据、强裁剪与颜色增强、预训练视觉表征、腕部相机等冗余相机、关键点等不变中间表示提升鲁棒性，同时固定并复查相机安装，部署前把图像与训练分布做健康检查。）

**追问方向**：腕部相机的优势；环境健康检查的实现。

### Q59 评估时如何避免"在训练演示上评测"的偏差？（Evaluation hygiene）
**优先级：★★** · 对应基础知识：[机器学习基础](../../02_基础知识/10_机器学习与深度学习/01_机器学习基础.md)

**答案要点**
- 常见偏差：评测时物体摆放与训练示教几乎一样；挑选成功视频展示；多次重试直到成功；评测者知道策略变体（非盲）；环境条件（光照 / 磨损）随时间变化。
- 做法：保留**未见过的初始条件**作为测试集；预先固定试验次数与成功判据；随机化顺序、盲评；报告所有试验（含失败）；记录环境条件；统计置信区间。
- 泛化分级：同分布（训练条件）、中度泛化（新位置）、强泛化（新物体 / 新背景），分别报告。
- 公开可复现：评测脚本与随机种子、失败案例总结。
- 面试加分点：主动说明评测局限与失败模式。

**English short answer**
To avoid biased evaluation I hold out initial conditions that were not in the demonstrations, fix the number of trials and the success criterion in advance, randomize and blind the order, report all trials including failures, and record environment conditions. I report in-distribution, moderate and strong generalization separately with confidence intervals, and I state the limitations and failure modes openly.

（译：为避免评测偏差，保留示教中没有的初始条件作为测试集，预先固定试验次数和成功判据，随机化并盲化顺序，报告包括失败在内的所有试验并记录环境条件；分别报告同分布、中度与强泛化的结果及置信区间，并坦率说明局限与失败模式。）

**追问方向**：盲评如何实施；成功判据的量化。

---

## 六、再补充

### Q60 如何用 VLM 为示教视频自动生成语言标注？（Automatic language relabeling）
**优先级：★★** · 对应基础知识：[大语言模型预训练_微调_RAG与VLA](../../02_基础知识/10_机器学习与深度学习/08_大语言模型预训练_微调_RAG与VLA.md)

**答案要点**
- 机器人示教常只有任务 ID 或单一指令，语言多样性不足，策略易忽略语言。
- 做法：用 VLM 对每条示教的关键帧 / 视频片段生成多种自然语言描述（任务描述、子步骤、物体属性、同义改写），并配合人工抽检；对子任务分段（夹爪开合、速度极小点作为分割线索）生成分步指令。
- 好处：增加语言多样性与组合泛化；支持分层规划（子任务标签）。
- 风险：VLM 幻觉造成错误标注（指向错物体）、描述与动作不一致 → 需一致性过滤（用另一个模型验证、规则检查）。
- 评估：指令遵循率在改写指令下是否保持。

**English short answer**
Demonstrations often carry a single instruction, so a VLM can generate varied descriptions, paraphrases and step-by-step subtask labels from keyframes or clips, which increases language diversity and supports hierarchical planning. Hallucinated or inconsistent labels are the risk, so I filter them with consistency checks and spot checks, and I verify that instruction following survives paraphrased commands.

（译：示教常只有单一指令，可用 VLM 从关键帧或片段生成多样描述、改写和分步子任务标签，增加语言多样性并支持分层规划；风险是幻觉或不一致的标注，需用一致性检查和抽检过滤，并验证改写指令下指令遵循是否保持。）

**追问方向**：用夹爪事件分割子任务；标注一致性检查的实现。

### Q61 开环回放（replay）实验能说明什么？（Open-loop replay as a diagnostic）
**优先级：★★** · 对应基础知识：[测试策略_仿真在环_硬件在环_回归](../../02_基础知识/13_系统工程与安全/04_测试策略_仿真在环_硬件在环_回归.md)

**答案要点**
- 做法：把采集到的示教动作序列原样发送给机器人（不看观测），观察是否复现任务。
- 结论判读：能复现 → 硬件、标定、动作映射、控制器跟踪正常，问题在策略；不能复现 → 示教与执行之间有系统偏差（延迟、增益、坐标系、夹爪映射、零点漂移），先修系统再训练策略。
- 其他诊断：把训练集观测输入策略，比较输出与示教动作（开环误差）；用仿真器回放验证数据格式。
- 价值：区分"系统问题"和"学习问题"，节省大量调试时间。
- 注意：物体初始位置略有差异时开环回放也会失败，所以该实验应使用与采集时一致的场景。

**English short answer**
Sending recorded demonstration actions back to the robot without feedback tests whether the hardware, calibration, action mapping and tracking controller can reproduce the task. If replay works, the problem lies in the learned policy; if it fails, there is a systematic mismatch such as latency, gains, frames or gripper mapping that must be fixed first. It cleanly separates system problems from learning problems.

（译：把录制的示教动作无反馈地发回机器人，可检验硬件、标定、动作映射和跟踪控制器能否复现任务；回放成功说明问题在学习到的策略，失败则存在延迟、增益、坐标系或夹爪映射等系统性失配，需先修复；它清晰地区分了系统问题与学习问题。）

**追问方向**：回放失败时如何定位延迟；初始场景不一致的影响。

---

## 本章要点速记
1. 生成模型：自回归（似然）、VAE（ELBO）、GAN（对抗）、扩散/流匹配（逐步去噪/搬运）；扩散质量高但采样慢，靠 DDIM、高阶求解器、蒸馏加速。
2. 行为克隆的两大问题：协变量偏移（误差累积 $O(T^2)$）与多模态平均；DAgger、动作分块、扩散策略分别缓解。
3. 动作分块缩短有效时域，时间集成平滑；扩散策略条件去噪，多模态与时间一致。
4. 动作空间：关节 vs 末端、增量 vs 绝对，归一化与反归一化须一致；旋转用连续表示。
5. VLA = VLM 骨干 + 动作头（离散 token 或扩散/流匹配连续头）；语义强、精度与安全保证弱，实际用分层/混合架构。
6. 评估看闭环成功率及置信区间，不看开环误差；按轨迹划分数据避免泄漏。
7. 部署：推理与控制解耦、有界队列、超时回退；确定性安全层位于模型之外。
8. 数据金字塔：互联网语义 + 仿真规模 + 真机校准；共训练与 LoRA 微调保持泛化。

## 自测题
1. 说明为什么 MSE 回归的行为克隆在多模态示教下会失败，并给出两种改进。
2. 推导扩散前向过程的闭式 $x_t$，并说明为什么训练目标可以是预测噪声。
3. 动作分块如何降低误差累积？时间集成解决了什么？
4. 比较 ACT、Diffusion Policy 与离散 token VLA 的动作表示与推理特点。
5. 设计一个让 VLA 输出安全可执行的系统架构（至少列出三层保障）。
6. 如何评估一个新训练的操作策略是否真的泛化到了新物体位姿？

## 相关链接
- 同模块：[强化学习八股](../02_人工智能/05_强化学习八股.md) · [Transformer 与大模型八股](../02_人工智能/04_Transformer与大模型八股.md) · [Agent 与 RAG 八股](../02_人工智能/07_Agent与RAG八股.md) · [大模型训练与推理工程八股](./08_大模型训练与推理工程八股.md)
- 基础知识：[模仿学习](../../02_基础知识/10_机器学习与深度学习/10_模仿学习_行为克隆_DAgger_扩散策略.md) · [世界模型与具身基础模型](../../02_基础知识/10_机器学习与深度学习/14_世界模型与具身基础模型综述.md)
- 项目挂钩：P2 双臂遥操作（示教数据基础设施）、P6 安全执行链（模型提案、确定性层执行）
- 八股总入口：[README](../README.md)
