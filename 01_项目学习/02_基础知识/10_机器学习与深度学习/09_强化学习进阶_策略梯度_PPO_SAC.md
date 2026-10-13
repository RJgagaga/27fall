# 强化学习进阶：策略梯度、GAE、PPO、SAC 与 sim-to-real

> **一句话定位**：[强化学习与具身智能入门](./06_强化学习与具身智能入门.md) 讲了 MDP、Bellman 方程、Q-learning 和 PPO 的直觉。本章补上"为什么这样更新"的数学：策略梯度定理的推导、基线与方差、广义优势估计（GAE）、PPO 的裁剪目标、连续控制的 DDPG/TD3/SAC，以及机器人真正关心的奖励设计、域随机化和 sim-to-real。

**本章学完你能：**

- 用对数导数技巧推导策略梯度定理，写出 REINFORCE、reward-to-go 与基线版本，并解释基线为什么降方差而不引入偏差；
- 手算 softmax 策略的梯度，手算基线对方差的影响；
- 手算 TD 误差、GAE 优势，解释 $\lambda$ 在偏差和方差间的折中；
- 手算 PPO 裁剪目标的四种情形，说出裁剪保护什么；
- 说明 DDPG/TD3/SAC 的结构与改进点，手算 TD3 的双 Q 取小与 SAC 的熵项；
- 区分 on-policy 与 off-policy、模型自由与基于模型；
- 设计机器人奖励函数，解释势函数形式的奖励塑形（reward shaping）为何不改最优策略；
- 讲清 sim-to-real 的主要手段：域随机化、系统辨识、特权信息师生蒸馏、动作平滑与底层 PD；
- 对比 RL 与 LQR/MPC，给出合理选择；
- 用英文回答 "Derive the policy gradient / what is GAE / why does PPO clip / SAC vs PPO / how do you do sim-to-real"。

**前置知识**：[强化学习与具身智能入门](./06_强化学习与具身智能入门.md)、[神经网络与反向传播](./02_神经网络与反向传播.md)、[概率统计与高斯分布](../01_数学基础/03_概率统计与高斯分布.md)、[最优化与二次规划](../01_数学基础/04_最优化与二次规划.md)。

## 目录

1. 为什么要学"进阶"
2. 目标与记号
3. 策略梯度定理（推导）
4. 基线、优势与 Actor-Critic
5. GAE（手算）
6. TRPO 到 PPO（手算）
7. 连续控制：DDPG、TD3、SAC（手算）
8. On-policy 与 Off-policy、样本效率
9. 奖励设计与奖励塑形
10. Sim-to-Real
11. 机器人 RL 的典型流水线（以足式/轮足为例）
12. RL 与 LQR/MPC：如何选择
13. 在本项目里怎么用
14. 常见错误与故障现象
15. 面试怎么问、怎么答
（补充）补充专题：观测归一化、tanh 压缩修正、PPO 伪代码与调试清单
16. 本章要点速记
17. 自测题
18. 相关链接

---

## 1. 为什么要学"进阶"

面试官问 RL，常常不满足于"PPO 是一种策略梯度算法"，而会追问：策略梯度从哪来？优势函数是什么？PPO 为什么要裁剪？为什么 SAC 样本效率更高？这些问题的答案都在下面的推导里。对机器人方向的面试官，还会追问 sim-to-real：你在仿真训出来的策略为什么能上真机？

## 2. 目标与记号

| 符号 | 含义 |
|---|---|
| $s_t,a_t,r_t$ | 状态、动作、奖励 |
| $\pi_\theta(a|s)$ | 参数为 $\theta$ 的随机策略 |
| $\gamma\in[0,1)$ | 折扣因子；有效视野约 $1/(1-\gamma)$（$\gamma=0.99$ 时 100 步，$0.9$ 时 10 步） |
| $\tau=(s_0,a_0,\dots)$ | 轨迹 |
| $R(\tau)=\sum_t\gamma^tr_t$ | 回报 |
| $V^\pi(s)$ | 状态价值 |
| $Q^\pi(s,a)$ | 动作价值 |
| $A^\pi(s,a)=Q^\pi-V^\pi$ | 优势函数：该动作比平均好多少 |

目标：最大化期望回报 $J(\theta)=\mathbb E_{\tau\sim\pi_\theta}[R(\tau)]$。

## 3. 策略梯度定理（推导）

### 3.1 对数导数技巧

轨迹概率 $p_\theta(\tau)=p(s_0)\prod_t\pi_\theta(a_t|s_t)p(s_{t+1}|s_t,a_t)$。
$$\nabla_\theta J=\nabla_\theta\int p_\theta(\tau)R(\tau)d\tau=\int p_\theta(\tau)\,\nabla_\theta\log p_\theta(\tau)\,R(\tau)d\tau.$$
这里用了 $\nabla p=p\nabla\log p$。因为环境转移 $p(s'|s,a)$ 不依赖 $\theta$，$\nabla_\theta\log p_\theta(\tau)=\sum_t\nabla_\theta\log\pi_\theta(a_t|s_t)$。得到
$$\boxed{\nabla_\theta J=\mathbb E_\tau\Big[\sum_t\nabla_\theta\log\pi_\theta(a_t|s_t)\,R(\tau)\Big]}.$$
**不需要知道环境模型**——这就是"model-free"的来历。用采样平均作无偏估计。

### 3.2 因果性：reward-to-go

时刻 $t$ 的动作不能影响 $t$ 之前的奖励，所以可以把 $R(\tau)$ 换成"从 $t$ 起的回报" $G_t=\sum_{k\ge t}\gamma^{k-t}r_k$，期望不变、方差更小：
$$\nabla_\theta J=\mathbb E\Big[\sum_t\nabla_\theta\log\pi_\theta(a_t|s_t)\,G_t\Big]\quad\text{（REINFORCE）}.$$

### 3.3 手算：softmax 策略的单步梯度

两个动作，$\pi(a)=\mathrm{softmax}(\theta)$，奖励 $R(a_0)=1$，$R(a_1)=0$。$\theta=(0,0)$ 时 $\pi=(0.5,0.5)$，$J=0.5$。

精确梯度：$\partial J/\partial\theta_a=\pi_a(R_a-J)$ → $(0.25,-0.25)$（提高动作 0 的 logit）。采样估计：若采到动作 0，$\nabla\log\pi=e_0-\pi=(0.5,-0.5)$，乘以回报 1 得 $(0.5,-0.5)$；采到动作 1 回报为 0，贡献 0；期望 $=0.5\times(0.5,-0.5)=(0.25,-0.25)$，与精确梯度一致（Python 复核）✓。

## 4. 基线、优势与 Actor-Critic

### 4.1 基线不引入偏差

对任意只依赖状态的 $b(s)$：$\mathbb E_{a\sim\pi}[\nabla\log\pi(a|s)\,b(s)]=b(s)\nabla\sum_a\pi(a|s)=b(s)\nabla1=0$。所以
$$\nabla J=\mathbb E\Big[\sum_t\nabla\log\pi(a_t|s_t)\,(G_t-b(s_t))\Big]$$
仍是无偏的，但方差可以大幅降低。最佳常用选择 $b=V^\pi(s)$，则 $G_t-V(s_t)\approx A(s_t,a_t)$。

### 4.2 手算：基线降方差

两个动作各 0.5 概率，回报 10 和 12，把得分函数简化成 $\pm1$（教学示例）。无基线：单样本梯度估计取值 $\{10,-12\}$，均值 $-1$，方差 $121$；取 $b=11$：取值 $\{-1,-1\}$，均值 $-1$，方差 $0$（Python 复核）。**均值不变（无偏），方差从 121 降到 0**。

### 4.3 Actor-Critic

用一个网络（critic）估计 $V_\phi(s)$：用 TD 误差 $\delta_t=r_t+\gamma V_\phi(s_{t+1})-V_\phi(s_t)$ 作为优势的单步估计。Actor 按 $\nabla\log\pi\cdot\hat A$ 更新，critic 最小化 $(V_\phi(s_t)-\text{target})^2$。

## 5. GAE（手算）

### 5.1 偏差—方差的两端

- 1 步 TD：$\hat A^{(1)}_t=\delta_t$，方差小，但偏差大（依赖 critic 的准确度）；
- 蒙特卡洛：$\hat A^{(\infty)}_t=G_t-V(s_t)$，无偏，但方差大。

### 5.2 GAE 定义

$$\hat A^{GAE(\gamma,\lambda)}_t=\sum_{l=0}^{\infty}(\gamma\lambda)^l\delta_{t+l}=\delta_t+\gamma\lambda\hat A_{t+1}.$$
$\lambda=0$ 退化为 1 步 TD，$\lambda=1$ 为蒙特卡洛。实践中 $\lambda\approx0.95$。

### 5.3 手算

三步轨迹，奖励 $r=[1,1,1]$，价值估计 $V=[0.5,0.6,0.7]$，终止后 $V=0$，$\gamma=0.99$，$\lambda=0.95$。

- $\delta_2=1+0-0.7=0.3$；
- $\delta_1=1+0.99\times0.7-0.6=1.093$；
- $\delta_0=1+0.99\times0.6-0.5=1.094$。

逆向递推 $\hat A_t=\delta_t+\gamma\lambda\hat A_{t+1}$，$\gamma\lambda=0.9405$：

- $\hat A_2=0.3$；
- $\hat A_1=1.093+0.9405\times0.3=1.375$；
- $\hat A_0=1.094+0.9405\times1.375=2.387$。

价值目标 $\hat A_t+V_t=[2.887,1.975,1.0]$。对比蒙特卡洛回报 $[2.970,1.990,1.0]$（Python 复核）：GAE 估计略低，因为 critic 对后续价值的低估在这里作了折中。

## 6. TRPO 到 PPO（手算）

### 6.1 问题

策略梯度每次更新只能用当前策略采样的数据，更新太大会让策略崩溃（性能无法恢复）。需要**限制每次更新的幅度**。

### 6.2 重要性采样与代理目标

用旧策略的数据评估新策略：
$$L(\theta)=\mathbb E_t\big[r_t(\theta)\hat A_t\big],\quad r_t(\theta)=\frac{\pi_\theta(a_t|s_t)}{\pi_{\theta_{old}}(a_t|s_t)}.$$
TRPO 在 KL 约束下优化 $L$，需要二阶近似，较复杂。

### 6.3 PPO 裁剪

$$L^{CLIP}(\theta)=\mathbb E_t\Big[\min\big(r_t\hat A_t,\ \operatorname{clip}(r_t,1-\epsilon,1+\epsilon)\hat A_t\big)\Big],\quad\epsilon=0.2.$$
**手算**：

| 比值 $r$ | 优势 $\hat A$ | 未裁剪 $r\hat A$ | 裁剪项 | 取 min |
|---|---|---|---|---|
| 1.3 | +2 | 2.6 | $1.2\times2=2.4$ | **2.4** |
| 0.7 | +2 | 1.4 | $0.8\times2=1.6$ | **1.4** |
| 0.7 | −1 | −0.7 | $0.8\times(-1)=-0.8$ | **−0.8** |
| 1.3 | −1 | −1.3 | $1.2\times(-1)=-1.2$ | **−1.3** |
| 1.1 | +2 | 2.2 | 2.2 | 2.2 |

（Python 复核。）读法：
- 好动作（$A>0$）已经被提升到 $r>1.2$ 后，目标值不再增加（2.4 封顶），梯度为 0，不再继续推；
- 坏动作（$A<0$）已降到 $r<0.8$ 后，目标不再进一步改善（−0.8 对应的是裁剪项，梯度为 0）；
- 但如果更新方向"反了"（好动作概率降低、坏动作概率升高），min 保留未裁剪项，仍有梯度去纠正——这是 min 的作用（保守，**悲观下界**）。

### 6.4 PPO 的实际配方

- 每轮并行采样 $N_{env}\times T$ 步（如 4096 个并行环境 × 24 步 $=98{,}304$ 个样本，GPU 并行仿真中常见的量级，教学示例）；
- 计算 GAE，**归一化优势**（减均值除标准差）；
- 对同一批数据做若干个 epoch、小批量更新（常 3–5 个 epoch）；
- 总损失 $=-L^{CLIP}+c_1L^{VF}-c_2\mathcal H[\pi]$：价值损失 + 熵奖励（鼓励探索）；
- 梯度裁剪；学习率可按 KL 自适应；
- 观测归一化（运行均值/方差）；
- 连续动作用高斯策略，$\log\sigma$ 为可学习参数或网络输出。

### 6.5 为什么机器人爱用 PPO

稳定、并行仿真友好（大规模并行环境能在几分钟到几小时内训出行走策略）、超参数相对不敏感。代价是**on-policy**，样本不能复用，样本效率低（对真机训练不现实，因此主要在仿真中训练）。

## 7. 连续控制：DDPG、TD3、SAC（手算）

这三者都是 **off-policy actor-critic**：用经验回放池（replay buffer）反复利用历史数据，样本效率高。

### 7.1 DDPG

确定性策略 $a=\mu_\phi(s)$，critic $Q_\theta(s,a)$。Critic 目标 $y=r+\gamma Q_{\theta'}(s',\mu_{\phi'}(s'))$（目标网络 $\theta',\phi'$ 缓慢跟随）。Actor 沿 $\nabla_a Q$ 方向更新：$\nabla_\phi J=\mathbb E[\nabla_aQ(s,a)|_{a=\mu(s)}\nabla_\phi\mu_\phi(s)]$。问题：Q 值**过估计**，训练不稳。

### 7.2 TD3 的三个改进

1. **双 Q 取小**：两个 critic，目标用 $\min(Q_1,Q_2)$ 抑制过估计；
2. **延迟策略更新**：critic 更新几次才更新一次 actor；
3. **目标策略平滑**：给目标动作加裁剪过的噪声。

**手算**：$r=1,\ \gamma=0.99,\ Q_1'=10.2,\ Q_2'=9.8$：取小目标 $y=1+0.99\times9.8=10.702$；取大则 $11.098$。多次自举会逐步放大过估计误差，取小可抑制它。

### 7.3 SAC（Soft Actor-Critic）

在奖励里加上**策略熵**，最大化 $\mathbb E[\sum_t r_t+\alpha\mathcal H(\pi(\cdot|s_t))]$。随机策略（高斯 + tanh 压缩到动作范围）。Critic 目标：
$$y=r+\gamma\Big(\min_{i}Q_i'(s',a')-\alpha\log\pi(a'|s')\Big),\quad a'\sim\pi(\cdot|s').$$
**手算**：$\alpha=0.2$，$\log\pi(a'|s')=-1.2$：$y=1+0.99\times(9.8-0.2\times(-1.2))=1+0.99\times10.04=10.940$。熵项把"不确定性"当作奖励，所以策略不会过早塌缩到确定性，探索更充分、对扰动更鲁棒。

一维高斯策略的熵 $\mathcal H=\tfrac12\ln(2\pi e\sigma^2)$：$\sigma=0.5$ 时 $0.726$，$\sigma=1$ 时 $1.419$，$\sigma=0.1$ 时 $-0.884$（熵可为负，Python 复核）。**自动温度调节**：把 $\alpha$ 当变量，让策略熵逼近目标熵（常取 $-\dim(\mathcal A)$），避免手动调 $\alpha$。

### 7.4 选择

| 场景 | 推荐 |
|---|---|
| 大规模并行仿真、稳健训练 | PPO |
| 样本昂贵（真机微调、慢仿真） | SAC / TD3 |
| 离散动作 | DQN 系列、PPO |
| 需要多模态动作分布 | 扩散策略/混合策略（见 [模仿学习](./10_模仿学习_行为克隆_DAgger_扩散策略.md)） |

## 8. On-policy 与 Off-policy、样本效率

- **On-policy**（REINFORCE、A2C、PPO）：数据必须来自当前策略，用完即丢；稳定、简单，样本效率低；
- **Off-policy**（DQN、DDPG、TD3、SAC）：数据可来自任意旧策略，存回放池反复用；样本效率高，但"离策略 + 函数逼近 + 自举"（deadly triad）可能不稳定；
- **基于模型（model-based）**：学习动力学模型，在模型里想象/规划（Dyna、MBPO、Dreamer 一类，概述，未核对）；样本效率最高，受模型误差限制；
- **离线 RL（offline RL）**：只用固定数据集，不再与环境交互；难点是**分布外动作的价值过估计**，需保守性约束（CQL、IQL 一类，概述）。

## 9. 奖励设计与奖励塑形

### 9.1 机器人奖励通常是多项加权和

以"轮足/足式行走"为例（典型做法，权重为教学示例，非项目实测）：
$$r=w_1\cdot r_{vel\_track}+w_2\cdot r_{alive}-w_3\cdot\|\tau\|^2-w_4\cdot\|\ddot q\|^2-w_5\cdot\|a_t-a_{t-1}\|^2-w_6\cdot r_{orient}-w_7\cdot r_{fall}.$$
速度跟踪项是任务目标；其余是正则（能耗、平滑、姿态、不跌倒）。

### 9.2 奖励设计原则

- **密集** vs 稀疏：稀疏奖励（成功 1，否则 0）难以探索；密集奖励要避免被"钻空子"（reward hacking）；
- **尺度**：各项量级相近，避免某一项主导；
- **终止条件**：跌倒/越界立即终止，是强信号；
- **课程学习**：从简单（平地低速）逐步增加难度（斜坡、推力、高速）；
- 先保证**可行**，再加正则。

### 9.3 势函数塑形

直接加入"离目标越近奖励越高"可能改变最优策略。**势函数形式**的塑形不会：
$$F(s,a,s')=\gamma\Phi(s')-\Phi(s),$$
Ng 等证明加入它后最优策略不变。**手算**：$\Phi(s)=-\text{距离}$，$\gamma=0.99$，从距离 3 走到距离 2：$F=0.99\times(-2)-(-3)=1.02>0$，鼓励靠近目标；走远则为负；在环路上累加总和约为 0，不会被循环刷分。

## 10. Sim-to-Real

仿真训练的策略直接上真机常失败（reality gap）：摩擦、延迟、电机特性、质量分布、传感器噪声与仿真不同。

| 手段 | 做法 |
|---|---|
| **域随机化（domain randomization）** | 训练时随机化质量、摩擦、电机增益、传感器噪声、延迟、外力推动等，使策略对一族环境鲁棒 |
| **系统辨识** | 测量真实电机/惯量参数，让仿真更准（执行器网络：用真机数据训练一个网络拟合电机响应） |
| **特权信息师生蒸馏** | 教师策略训练时可看仿真中的特权信息（地形高度、真实摩擦），学生只用机载可得观测去模仿教师 |
| **非对称 actor-critic** | critic 吃特权观测，actor 只吃可部署观测 |
| **动作空间设计** | 策略输出关节位置目标或残差，由底层 PD 实现，而不是直接输出力矩（更平滑、更易迁移） |
| **平滑与限幅** | 动作低通滤波、变化率惩罚、关节限位与力矩限幅 |
| **观测历史** | 输入过去几帧观测/动作，让网络隐式推断环境参数（自适应） |
| **安全层** | 部署时加安全监控、急停、力矩/速度限制 |

> **注意**：随机化范围不能太大（策略保守、学不会）也不能太小（迁移失败）；是否真的需要，要看真机失败模式来判断。

## 11. 机器人 RL 的典型流水线（以足式/轮足为例）

```text
 GPU 并行仿真(数千环境) ──> 观测(关节、IMU、命令、上一动作) ──> 策略MLP/LSTM
        ▲                                                   │
        │  奖励/终止/随机化/课程                             ▼ 动作(关节目标)
        └────────── PPO 更新 <── 经验 <── 底层 PD 控制 ──> 仿真/真机执行
```

要点：策略频率常低于底层控制频率（如策略 50 Hz，PD 1 kHz）；真机上延迟与状态估计误差要在仿真里建模；部署前做域外测试（推、坡、负载）与安全检查。这条流水线的"低层 PD + 高层策略"与 P7 的"LQR + 关节力矩"是相通的分层思想（见 [P7 README](../../01_项目课程/P7_可变腿长轮腿机器人/README.md)）。

## 12. RL 与 LQR/MPC：如何选择

| 维度 | LQR / MPC | RL |
|---|---|---|
| 需要模型 | 需要（至少线性/可预测） | 不需要（但需要仿真器） |
| 约束处理 | MPC 显式约束 | 通过奖励/终止隐式 |
| 稳定性保证 | 可分析（LQR）、可设计（MPC） | 无形式保证 |
| 数据/调参 | 少数据，调权重 | 大量仿真，调奖励/随机化 |
| 复杂接触、高维、非线性 | 困难 | 强项 |
| 可解释/可调试 | 强 | 弱 |
| 实时计算 | MPC 需要求解 QP | 推理一次前向 |

实践结论：**模型清楚、约束重要、需要保证 → LQR/MPC；接触丰富、模型复杂、难以手工建模 → RL；常见的组合：RL 做高层/残差，经典控制做底层与安全**。

## 13. 在本项目里怎么用

CV 的七个项目里**没有 RL 训练**。本章用于：

- **基础面试**：展示你理解现代机器人学习的数学基础；
- **与 P1/P7 对比**：能清晰回答"为什么平衡用了 LQR/PID 而不是 RL"——模型已知（倒立摆）、有稳定性分析、实机调参成本低、安全性要求高；
- **与 P2 衔接**：示教数据可作模仿学习或 RL 的预训练/初始化（见 [模仿学习](./10_模仿学习_行为克隆_DAgger_扩散策略.md)）；
- **P3**：RoboMaster 类场景若引入 RL，也应把 RL 输出作为目标，由嵌入式层的闭环 PID/滑模去执行并限幅。

## 14. 常见错误与故障现象

| 现象 | 可能原因 | 处理 |
|---|---|---|
| 训练曲线平坦不涨 | 奖励太稀疏、尺度不对 | 增加密集项，归一化奖励/优势 |
| 训练突然崩溃 | 学习率太大、策略更新过猛 | 降学习率，减小 $\epsilon$，梯度裁剪 |
| 策略"钻空子"（原地抖动刷分） | 奖励漏洞 | 检查奖励，加入惩罚项，观察回放视频 |
| 仿真好真机差 | reality gap | 域随机化，执行器模型，延迟建模 |
| Q 值爆炸 | 过估计 | TD3 双 Q、目标网络、降学习率 |
| 熵塌缩过早收敛 | 熵系数太小 | 加大熵奖励/自动温度 |
| 动作抖动 | 无平滑 | 动作变化率惩罚，低通滤波 |
| 观测未归一化导致不稳定 | 量纲差异 | 运行均值方差归一化 |
| 随机种子差异大 | RL 方差本来就大 | 多种子报告均值与置信区间 |

## 15. 面试怎么问、怎么答

**Q1. Derive the policy gradient.**

> I write the objective as the expected return over trajectories, differentiate the trajectory probability using the log-derivative trick, and note that the environment dynamics do not depend on the policy parameters, so only the sum of log-policy gradients remains. That gives the expectation of the gradient of log pi times the return. I then use reward-to-go for causality and subtract a state-dependent baseline, typically the value function, which keeps the estimate unbiased but reduces the variance.

中文：用对数导数技巧对轨迹概率求导，环境转移不依赖 $\theta$，只剩 $\nabla\log\pi$ 之和乘回报；用 reward-to-go 和状态基线（常取 $V$）降方差且无偏。

**Q2. What is GAE and how do you choose lambda?**

> GAE is an exponentially weighted sum of TD errors, with discount gamma times lambda. Lambda equal to zero is the one-step TD advantage, low variance but biased by the critic. Lambda equal to one is the Monte Carlo advantage, unbiased but noisy. A value around 0.95 is a common compromise.

中文：GAE 是 TD 误差的指数加权和，权重 $(\gamma\lambda)^l$；$\lambda=0$ 低方差有偏，$\lambda=1$ 无偏高方差；常取 0.95 折中。

**Q3. Why does PPO clip the ratio?**

> The surrogate objective uses data from the old policy, so it is only trustworthy near it. Clipping the probability ratio to one plus or minus epsilon removes the incentive to move the policy far in one update, and taking the minimum with the unclipped term makes the objective a pessimistic bound. It gives most of the stability of trust-region methods with only first-order optimisation.

中文：代理目标基于旧策略数据，只在其附近可信；裁剪比值消除大幅更新的动机，取 min 得到悲观下界；用一阶优化获得类似信赖域的稳定性。

**Q4. SAC versus PPO: when would you use which?**

> PPO is on-policy: stable and easy to parallelise in simulation, so I would use it with thousands of simulated environments. SAC is off-policy with a replay buffer and an entropy bonus, so it is much more sample-efficient and explores better, which suits expensive data, for example fine-tuning on a real robot. SAC needs more care with Q-value stability.

中文：PPO on-policy，稳定、易并行，适合大规模仿真；SAC off-policy+熵，样本效率高、探索好，适合数据昂贵（如真机微调），但 Q 值稳定性需要注意。

**Q5. How do you make a policy trained in simulation work on the real robot?**

> I randomise physical parameters, delays and noise during training, identify the actuator dynamics from real data, make the policy output joint targets that a low-level PD controller tracks, penalise action jerk, and in some setups distil a privileged teacher into a student that only uses deployable observations. On the real robot I add torque and velocity limits and a safety monitor.

中文：训练时随机化参数/延迟/噪声；用真机数据辨识执行器；策略输出关节目标由底层 PD 跟踪；惩罚动作抖动；必要时特权教师→学生蒸馏；真机加力矩/速度限制与安全监控。

**Q6. Why did you use LQR and PID rather than reinforcement learning for balancing?**（项目对照）

> For a ballbot the physics is well understood, so a linearised model with LQR plus a PID cascade gives stability that I can analyse and tune on hardware in a short time, and safety matters. RL would need a high-fidelity simulator and a careful sim-to-real process, without a stability guarantee. I see RL as promising for contact-rich or poorly modelled behaviours, ideally on top of a classical low-level controller.

中文：球形机器人物理清楚，线性化+LQR+PID 可分析、上机调参快、安全要求高；RL 需要高保真仿真和 sim-to-real 且无稳定性保证；RL 更适合接触丰富/难建模的行为，最好叠在经典底层控制之上。

## 补充专题：观测归一化、tanh 压缩修正、PPO 伪代码与调试清单

### 观测归一化（运行均值与方差）

机器人观测量纲差异很大（关节角 rad、角速度 rad/s、位置 m），不归一化会让网络难以训练。常用**运行统计**（Welford 在线算法）：每来一批样本更新均值和方差，再用 $\hat o=\dfrac{o-\mu}{\sqrt{\sigma^2+\epsilon}}$（并裁剪到 $[-10,10]$）。**手算**：依次来到 $[1,2,3,4]$，均值 $2.5$，方差 $1.25$（总体方差），归一化后 $[-1.34,-0.45,0.45,1.34]$。**注意**：部署时必须使用与训练一致的均值/方差（把统计量保存进模型文件），否则输入分布不匹配，策略行为会漂移。奖励也常做归一化（按回报标准差缩放）。

### tanh 压缩高斯策略的对数概率修正（SAC 的细节）

动作需要限制在 $[-1,1]$，SAC 的做法：先从高斯采样 $u\sim\mathcal N(\mu,\sigma^2)$，再 $a=\tanh(u)$。变量替换后概率密度需要加雅可比修正：
$$\log\pi(a|s)=\log\mathcal N(u;\mu,\sigma^2)-\sum_i\log\big(1-\tanh^2(u_i)\big).$$
**手算**：$u=0.5$，$\tanh(0.5)=0.4621$，$1-0.4621^2=0.7865$，$-\log0.7865=+0.2402$（Python 复核）。所以 $\log\pi(a|s)$ 比未修正的高斯对数密度大 $0.24$。忘记这项会让熵估计错误，策略熵与自动温度调节出错。当 $|u|$ 很大时（$\tanh$ 饱和），$1-\tanh^2\to0$，修正项趋于很大，是数值不稳定的来源，实现中加小量 $\epsilon$。

### PPO 的一个训练迭代（伪代码，教学示例，未运行核对）

```python
for iteration in range(N_iter):
    # 1. 采样: 在 N_env 个并行环境中各跑 T 步，存 obs, act, logp, rew, done, value
    buf = rollout(policy, value_fn, envs, T)
    # 2. 计算 GAE 与回报目标 (逆序递推)
    adv, ret = compute_gae(buf, gamma=0.99, lam=0.95)
    adv = (adv - adv.mean()) / (adv.std() + 1e-8)
    # 3. 多个 epoch、小批量更新
    for epoch in range(K):
        for mb in minibatches(buf, adv, ret):
            ratio = exp(policy.logp(mb.obs, mb.act) - mb.logp_old)
            surr  = min(ratio * mb.adv, clip(ratio, 1-eps, 1+eps) * mb.adv)
            loss  = -surr.mean() + c1 * (value_fn(mb.obs) - mb.ret).pow(2).mean() \
                    - c2 * policy.entropy(mb.obs).mean()
            optimizer.zero_grad(); loss.backward()
            clip_grad_norm_(params, max_norm=1.0); optimizer.step()
```

读这段代码时对应本章内容：GAE（第 5 节）、裁剪（第 6 节）、熵奖励与价值损失（6.4）。监控指标：平均回报、回合长度、**近似 KL**（$\mathbb E[\log\pi_{old}-\log\pi]$，若远大于 0.01–0.02 说明更新过猛，数值为常见经验）、裁剪比例（被裁剪样本占比，若超过约 30% 可能学习率偏大）、熵（过快下降说明过早收敛）、价值解释方差。

### 典型超参数范围（经验，非项目实测）

| 超参 | PPO 常见范围 |
|---|---|
| $\gamma$ | 0.97–0.995 |
| $\lambda$（GAE） | 0.9–0.97 |
| 裁剪 $\epsilon$ | 0.1–0.3 |
| 学习率 | $10^{-4}$–$10^{-3}$（常配合衰减或按 KL 自适应） |
| epoch 数 | 3–10 |
| 熵系数 | 0–0.01 |
| 并行环境数 | 数百–数千 |

不同任务差异很大，调参时一次只改一个量，并用多个随机种子对比。

### 多种子评估

RL 方差很大，必须多种子。**手算**：5 个随机种子最终回报 $[100,120,90,110,130]$，均值 $110$，样本标准差 $15.8$，均值标准误 $15.8/\sqrt5=7.07$。若方法 B 均值 $125$、标准差同样 $15.8$，均值之差为 $15$；两组均值差的标准误为 $\sqrt{7.07^2+7.07^2}=10$，比值 $15/10=1.5$，低于常用的约 $2$ 的显著性门槛，所以统计上不能说 B 更好。**结论**：只跑 1–2 个种子就下结论，是 RL 论文与实验里最常见的错误。

### RL 调试清单

1. 先在简化环境验证（单个环境、无随机化、短回合），确认能学会；
2. 检查奖励各项的量级与符号，逐项记录曲线；
3. 看回放视频，找"钻空子"行为；
4. 观测/动作归一化与边界是否正确，动作是否被裁剪；
5. 终止条件是否过严或过松；
6. 学习率、批大小、并行环境数；
7. 对比"随机策略"与"零动作策略"的基线回报；
8. 保存检查点，失败时可回退；
9. 部署前做域外压力测试。

### 补充：离散动作与连续动作的选择

足式/轮足机器人的关节目标是连续量，通常使用高斯策略；开关类决策（抓取与否、是否切换步态）是离散量，可以用分类头。混合动作空间可以让网络同时输出一个连续向量和一个类别 logits，对数概率相加。离散化连续动作（例如每维 11 档）能使用 DQN，但维度 $d$ 时动作数 $11^d$ 爆炸，不适合高维关节控制，这就是连续控制转向 actor-critic 的原因。

### 补充：为什么 RL 在机器人上训练常需要 GPU 并行仿真

PPO 是 on-policy，每次更新要新鲜数据。若单个环境每秒产生 $10^3$ 个样本，要攒够一次更新用的 $10^5$ 样本需要 100 s；用 4096 个并行环境，每步同时产生 4096 个样本，几乎实时得到一批数据。并行仿真把"样本效率低"的弱点转化为"吞吐量高"的优势：训练 wall-clock 时间从几天缩短到几十分钟到数小时（量级，视任务与硬件而定，未核对）。代价是仿真器必须支持大规模并行、物理精度与真实之间存在差距，故需要域随机化。

## 16. 本章要点速记

1. 策略梯度：$\nabla J=\mathbb E[\sum_t\nabla\log\pi(a_t|s_t)\,G_t]$，不需要环境模型。
2. 基线 $b(s)$ 无偏降方差；常取 $V(s)$，得到优势 $A$。
3. TD 误差 $\delta_t=r_t+\gamma V(s_{t+1})-V(s_t)$；GAE $\hat A_t=\delta_t+\gamma\lambda\hat A_{t+1}$。
4. PPO：$\min(r\hat A,\operatorname{clip}(r,1\pm\epsilon)\hat A)$，$\epsilon=0.2$；on-policy。
5. 优势归一化、多 epoch 小批量、熵奖励、梯度裁剪是 PPO 标配。
6. TD3：双 Q 取小、延迟更新、目标平滑；SAC：最大熵 + 双 Q + 自动温度。
7. Off-policy 样本效率高，但 deadly triad 不稳；on-policy 稳但费样本。
8. 奖励：任务项 + 正则项；势函数塑形 $\gamma\Phi(s')-\Phi(s)$ 不改最优策略。
9. Sim-to-real：域随机化、执行器辨识、PD 目标动作、特权蒸馏、平滑限幅。
10. 模型清楚、需保证用 LQR/MPC；接触丰富/难建模用 RL；常见组合是 RL 高层 + 经典底层。

## 17. 自测题

1. $\gamma=0.95$ 的有效视野？（$1/(1-0.95)=20$ 步。）
2. PPO 中 $r=0.6$，$A=+1$，$\epsilon=0.2$，目标值？（$\min(0.6,0.8)=0.6$，且未裁剪、有梯度。）
3. 为什么 GAE 递推要从后往前？（$\hat A_t$ 依赖 $\hat A_{t+1}$。）
4. TD3 为什么用两个 critic 取小？（抑制过估计偏差。）
5. $\sigma=2$ 的一维高斯熵？（$\tfrac12\ln(2\pi e\cdot4)=\tfrac12\ln(68.3)=2.11$。）
6. 为什么真机直接输出力矩的策略很难迁移？（电机/延迟/摩擦差异大且力矩直接影响稳定；输出位置目标+PD 更平滑、PD 本身提供阻尼。）

## 18. 相关链接

- 上一章：[大语言模型预训练、微调、RAG 与 VLA](./08_大语言模型预训练_微调_RAG与VLA.md)；下一章：[模仿学习](./10_模仿学习_行为克隆_DAgger_扩散策略.md)
- 前置：[强化学习与具身智能入门](./06_强化学习与具身智能入门.md)
- 经典控制对照：[LQR 最优控制](../05_控制理论/04_LQR最优控制.md)、[MPC 模型预测控制](../05_控制理论/05_MPC模型预测控制.md)
- 八股速查：[强化学习八股](../../03_八股知识/02_人工智能/05_强化学习八股.md)
- 模块说明：[10_机器学习与深度学习 README](./README.md)
