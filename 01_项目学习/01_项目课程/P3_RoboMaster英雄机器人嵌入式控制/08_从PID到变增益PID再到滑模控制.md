# 08 从 PID 到变增益 PID 再到滑模控制

> **一句话定位**：讲清 CV 里 "iterating from PID through variable-gain PID to sliding-mode control" 的动机、设计与理论：PID 在哪里不够、变增益怎么设计、滑模的滑模面/趋近律/抖振/边界层，所有手算都用 Python 复核过，并给出一个可复现的摩擦轮教学仿真。

## 本章学完你能……

- 解释摩擦轮掉速的物理来源，区分"控制器能改善什么/不能改善什么"，并把"恢复时间 120 ms → 60–80 ms""弹速离散 ±1 m/s → ±0.3 m/s"这两组项目数字讲成一条因果链；
- 设计对数律变增益 PID（由两个期望点反推 $k_0,k_1,k_2$），说清 $\log_{10}(0)$ 保护与增益上下限；
- 推导一阶对象的滑模控制：滑模面、等效控制、趋近律、到达时间、Lyapunov 简推、边界层稳态误差，并手算；
- 解释项目版 $s=\dot e+ce$、$c=5,\rho=2,\epsilon=1$ 的含义、量纲和局限，并给出整定顺序；
- 估计抖振幅度并选边界层；理解 SMC 内部 $dt=0.001$ 与实际约 3 ms 任务周期的不一致意味着什么；
- 诚实回答"为什么迭代""是不是真滑模""怎么证明稳定""你的仿真能证明什么"。

## 前置知识

[PID 全解](../../02_基础知识/04_控制理论/02_PID控制全解.md)、[滑模控制](../../02_基础知识/04_控制理论/06_滑模控制.md)、[增益调度与饱和](../../02_基础知识/04_控制理论/07_前馈_增益调度_限幅与饱和.md)、上一章 [07 发射机构](./07_发射机构_摩擦轮与拨盘.md)；数字实现细节见 [12 数字 PID 实现](./12_数字PID实现.md)。

## 目录

1. 为什么迭代：现象与口径
2. PID 的局限（摩擦轮视角）
3. 变增益 PID 的设计
4. 滑模控制：原理与推导
5. 项目版 SMC：参数含义与整定
6. 离散实现（C 代码，教学示例）
7. 四项手算的 Python 复核
8. 数值仿真：PID、变增益、滑模的掉速恢复
9. 故障与边界
10. 面试问答
11. 要点速记、自测题、相关链接

---

## 1. 为什么迭代：现象与口径

### 1.1 它要解决什么问题

英雄机器人发射 42 mm 大弹丸。两个摩擦轮（M3508 直驱，轮径约 60 mm）以基础目标约 6000 rpm 旋转，表面线速度约 19 m/s，对应 16 m/s 的弹速上限。每发弹丸挤过两轮之间，会把轮子的一部分动能带走，转速瞬间下跌；速度环负责把它拉回目标。**弹速几乎正比于弹丸出膛时刻的轮速**，所以"下一发出膛时轮速恢复到哪里"直接决定弹速一致性。

### 1.2 项目事实陈述

本项目取值约如下（来源：CV 的迭代描述 + 笔记中的参数 + 赛前台架观察，数字按本项目统一口径）：

| 阶段 | 控制器 | 连发掉速 | 恢复时间 | 弹速离散 |
|---|---|---|---|---|
| 第一版 | 速度环 PID（Kp=12，Ki=0.2，Kd=0，输出限幅 12000） | 弹丸擦过使转速跌落约 600–900 rpm | 约 120 ms | 约 ±1 m/s |
| 第二版 | 变增益 PID（$K_p=k_0\log_{10}(k_1\lvert e\rvert)+k_2$，笔记参数 $k=(1,8,5)$） | 同上（掉速由能量决定） | 大误差响应改善；但近零误差处增益过低或过高，$\log$ 在 $e\to0$ 需要下限保护 | 未达要求 |
| 第三版 | 滑模（$s=\dot e+ce$，$c=5,\rho=2,\epsilon=1$，输出限幅 10000，$dt=0.001$） | 同上 | 约 60–80 ms | 约 ±0.3 m/s |

把转速换算成弹速有助于感受量级：6000 rpm 对应约 16 m/s，即**约 375 rpm 对应 1 m/s**。掉 600–900 rpm 相当于若完全不恢复，下一发弹速会低 1.6–2.4 m/s；拨盘单发约 150 ms，PID 的恢复约 120 ms，几乎吃满射击间隔，只要电池电压、弹丸批次稍有变化，下一发出膛时轮速就还没回稳，这就是 ±1 m/s 的来源。恢复缩短到 60–80 ms 后，射击间隔里有了一半的余量，残余掉速波动小，±0.3 m/s 约对应 ±110 rpm 的出膛速度波动。

### 1.3 口径备忘（面试按此说）

1. **三者是迭代历史，不是同时运行**：PID 是第一版，变增益 PID 是中间版，滑模是最终使用版。笔记里的代码快照也与此一致：摩擦轮的活动路径调用滑模，变增益 PID 只做了初始化、调用被注释。
2. **弹速反馈来自裁判系统上报的 bullet_speed**：每发后更新，高于 16.0 m/s 就把 fire_speed 绝对值减 50 rpm，低于 15.7 m/s 就加 50 rpm。这是慢的外层修正，不是本章的速度环。
3. **控制周期不是 1 kHz**：云台、发射、底盘任务循环约 3 ms（约 333 Hz，三处 `vTaskDelay(1)`）；SMC 内部 `dt=0.001` 是名义常数，见第 5、7 节。
4. 因此本章的"迭代理由"由三层组成：**通用原因（第 2 节）+ 项目现象（上表）+ 对比方法（第 8 节教学仿真，数字不能当实机数据）**。

---

## 2. PID 的局限（摩擦轮视角）

### 2.1 对象与符号

摩擦轮一阶模型：

$$J\dot\omega=K_t i-b\omega-\tau_L$$

$J$ 为轮与转子转动惯量（kg·m²），$\omega$ 为角速度（rad/s），$K_t$ 为转矩常数（N·m/A），$i$ 为电流（A），$b$ 为粘性阻尼系数，$\tau_L$ 为负载转矩（弹丸挤压的冲击）。控制量是电流指令（计数，±16384 对应 ±20 A），测量量是 rpm（整数）。

### 2.2 四个局限

1. **掉速是物理必然。** 弹丸动能来自轮的转动动能：$\Delta\omega\approx E_k/(J\omega)$。控制器只能决定恢复，不能消除第一时间的跌落（07 章有手算）。台架上掉得更多（600–900 rpm），因为还包含擦过摩擦与弹丸压缩损耗。
2. **固定增益的两难。** 大误差（起转、掉速后）希望大增益，快速追；小误差（稳态）大增益会放大 1 rpm 量化噪声：每个周期误差跳 1 个计数，指令就跳 $K_p$ 个计数，电流抖动、电机发热。固定一组 $K_p$ 只能折中。
3. **积分饱和。** 起转或掉速后电流指令顶到限幅，积分项继续累积，恢复到目标时积分还"欠着账"→ 过冲、尾部慢。若积分按每个控制周期累加（Ki=0.2，周期 3 ms），积分时间常数约 $K_p/(K_i/T_c)=12/(0.2/0.003)\approx0.18$ s，恢复曲线的"尾巴"主要由它拖长。
4. **模型漂移。** 电池电压（满电与低电）、温度、轮面磨损会改变有效 $K_t$、$b$，固定增益的性能随之漂移，且很难在实验室里穷尽。

> **面试要点**：不要说"PID 不行所以换"。准确的说法是：PID 在标称条件下并不差，问题出在连发时（大误差反复出现）恢复不够快、第一发和后几发不一致；我想要的是"误差大时有力、误差小时安静、扰动变化时更稳健"的控制器。

---

## 3. 变增益 PID 的设计

### 3.1 思想与形式

让比例增益随误差大小变化：小误差小增益（安静）、大误差大增益（有力）。笔记里的形式是对数律：

$$K_p=k_0\log_{10}(k_1|e|)+k_2,\qquad P=K_p\,e$$

积分、微分用各自固定增益。**为什么用对数而不用线性？** 速度环误差跨三个数量级（1 rpm 的量化噪声到 1000 rpm 的掉速），线性调度会让增益变化 1000 倍；对数把三个数量级压成三个等间距台阶，增益变化温和可控。

### 3.2 三个参数的含义

| 参数 | 作用 |
|---|---|
| $k_0$ | 误差每扩大 10 倍，$K_p$ 增加 $k_0$（斜率） |
| $k_1$ | 平移曲线：$k_1\lvert e\rvert=1$ 的点上 $K_p=k_2$ |
| $k_2$ | 整体抬升；也是 $k_1\lvert e\rvert=1$ 处的增益 |

### 3.3 由两个期望点反推（手算 + Python 复核）

给定：$|e|=10$ 时 $K_p=6$，$|e|=1000$ 时 $K_p=10$。

1. $k_0=\dfrac{10-6}{\log_{10}(1000/10)}=\dfrac{4}{2}=2$；
2. 取参考误差 10 使该点对数为 0：$k_1=1/10=0.1$，$k_2=6$；
3. 验证：$|e|=100$，$K_p=2\log_{10}(10)+6=8$。第 7 节脚本输出 $(k_0,k_1,k_2)=(2,0.1,6)$，三个点的增益为 $6,8,10$。

**笔记参数 $k=(1,8,5)$ 的增益曲线**（Python 计算）：

| $\lvert e\rvert$（rpm） | 0.1 | 0.5 | 1 | 5 | 10 | 50 | 100 | 500 | 1000 |
|---|---|---|---|---|---|---|---|---|---|
| $K_p$ | 4.90 | 5.60 | 5.90 | 6.60 | 6.90 | 7.60 | 7.90 | 8.60 | 8.90 |

三个数量级内 $K_p$ 只从 5.90 变到 8.90，约 51%，是**温和的调度**。例：$e=100$，$K_p=7.903$，$P=790.3$。注意：这套 5.9–8.9 与 PID 阶段的 $K_p=12$ 不能直接比较，笔记没有给出变增益版的输出标度，工程上两者的输出系数可以不同。

### 3.4 下限保护与工程边界

$\log_{10}(0)$ 在数学上是 $-\infty$，C 里 `log10f(0.0f)` 得到 $-\infty$，再乘 $e=0$ 得到 NaN，会一路传到电流指令。Python 里 `math.log10(0)` 直接抛 `ValueError`，也是同一个提醒。处理：

| 问题 | 处理 |
|---|---|
| $e=0$ 时 $\log(0)$ | 对 $\lvert e\rvert$ 设下限 $e_{\min}$（取 1 rpm，即反馈量化分辨率，小于它的误差本来就是量化噪声） |
| $K_p$ 可能为负或过大 | 同时设 $K_{\min}$、$K_{\max}$；当 $k_1\lvert e\rvert<10^{-k_2/k_0}$ 时对数项会把 $K_p$ 拉成负数 |
| 取整数绝对值 | 浮点误差用 `fabsf`；笔记提示快照里用了整数 `abs`，会丢掉小数，需核对 |
| 单位依赖 | 误差用 rpm 还是 rad/s，$k_1$ 随之变（相差 $60/2\pi\approx9.55$ 倍） |
| 积分、微分 | 增益变化不影响 I、D 项，但它们要绑定实际周期（约 3 ms），见 [12 章](./12_数字PID实现.md) |

### 3.5 简单的稳定性论证

控制量 $u=K_p(|e|)\,e$ 是误差的非线性函数。令 $f(e)=K_p(|e|)e$，则 $e>0$ 时 $f'(e)=K_p+\dfrac{k_0}{\ln10}>0$（$K_p>0$ 且 $k_0>0$），所以 $f$ 单调递增，并且 $u/e=K_p\in[K_{\min},K_{\max}]$ 落在一个扇区内。直观含义：若 $K_{\min}$ 到 $K_{\max}$ 内每个冻结增益下回路都稳定，且增益变化不快，则整体不会出现"某个增益下突然不稳"。**这是增益调度的经验论证，不是对任意工况的严格证明。**

### 3.6 局限

仍是"线性控制器 + 增益调度"，对参数不确定性和扰动没有显式鲁棒保证；噪声让 $|e|$ 抖动 → 增益抖动；上限给得高，延迟较大时会引入振铃（第 8 节仿真里固定 $K_p=12$ 就有振铃）。

---

## 4. 滑模控制：原理与推导

### 4.1 要解决什么问题

PID 的参数是按标称模型整定的；滑模控制的出发点是：**设计一个"滑模面"$s=0$，让系统一旦到达它就沿着它收敛，并且只要切换增益盖过扰动上界，对匹配扰动不敏感**。代价是不连续切换带来的抖振。

### 4.2 一阶对象的完整推导

对象归一化：$\dot v=-av+bu-d$，其中 $d$ 是等效负载加速度（转速/秒量纲）。目标 $r$ 为常值，误差 $e=r-v$，取滑模变量 $s=e$：

$$\dot e=av-bu+d$$

控制律取等效控制加切换项：

$$u=\frac1b\Big(av+k\,\mathrm{sat}(e/\phi)\Big)$$

代入得

$$\dot e=-k\,\mathrm{sat}(e/\phi)+d$$

**sat 的定义**（边界层 $\phi$ 的单位与 $e$ 相同）：

$$\mathrm{sat}(x)=\begin{cases}x,&|x|\le1\\ \mathrm{sgn}(x),&|x|>1\end{cases}$$

即 $|e|\le\phi$ 时线性，$|e|>\phi$ 时饱和为 ±1。**符号函数** $\mathrm{sgn}(e)$ 是 $\phi\to0$ 的极限。

**到达（reaching）。** $|e|>\phi$ 时 $\dot e=-k\,\mathrm{sgn}(e)+d$，只要 $k>|d|$，$|e|$ 以不小于 $k-|d|$ 的速率减小：

$$t_r\le\frac{|e_0|-\phi}{k-|d|}$$

**Lyapunov 简推。** 取 $V=\tfrac12e^2\ge0$：

- 边界层外：$\dot V=e\dot e=-k|e|+ed\le-(k-|d|)|e|<0$，状态必在有限时间内进入边界层；
- 边界层内：$\dot e=-(k/\phi)e+d$，$\dot V=-(k/\phi)e^2+ed$，当 $|e|\ge d\phi/k$ 时 $\dot V\le0$。所以 $|e|$ 最终被限制在 $d\phi/k$ 以内（**终极有界**，不是渐近到零）。

**边界层稳态误差**

$$e_{ss}=\frac{d\,\phi}{k}$$

这是边界层的本质权衡：$\phi$ 小 → 精度高、抖振风险大；$\phi$ 大 → 平滑、静差大。

### 4.3 手算例子（教学示例）

对象 $T\dot v=u-v-\text{load}$，$T=0.15$ s；此时 $\dot v=(u-v-\text{load})/T$，等效 $d=\text{load}/T$。控制律 $u=v+Tk\,\mathrm{sat}(e/\phi)$。

**单步**：目标 1，$v=0.9$，$k=5$，$\phi=0.2$：$e=0.1$，$\mathrm{sat}=0.5$，$u=0.9+0.15\times5\times0.5=1.275$，$\dot v=(1.275-0.9)/0.15=2.5$，$\dot e=-2.5$。

**边界层内的欧拉迭代**（$\Delta t=0.01$ s）：$e_{n+1}=e_n(1-k\Delta t/\phi)=0.75e_n$：$0.1\to0.075\to0.0563\to0.0422\to0.0316$，时间常数 $\phi/k=0.04$ s。

**稳态误差**：负载 0.6，$d=0.6/0.15=4<k=5$（满足 $k>|d|$）：$e_{ss}=4\times0.2/5=0.16$（在边界层内，但已接近边界）；缩小 $\phi$ 到 0.02 → 0.016。第 7 节在 $dt=0.001$ 的欧拉仿真里验证：$\phi=0.2$ 得 0.1600，$\phi=0.02$ 得 0.0160，公式与仿真一致。

### 4.4 滑模面 $s=\dot e+ce$ 与趋近律

对一阶对象，$s=e$ 就够用；对要同时约束位置和速度的二阶对象，滑模面取

$$s=\dot e+ce$$

$s=0\Rightarrow\dot e=-ce\Rightarrow e(t)=e_0e^{-ct}$。$c=5$ s⁻¹ 时（数值验证）：

| $t$（s） | 0 | 0.1 | 0.2 | 0.4 | 0.6 |
|---|---|---|---|---|---|
| $e$（$e_0=100$） | 100 | 60.65 | 36.79 | 13.53 | 4.98 |

也就是每过 0.2 s 误差缩小到 37%。

**趋近律**描述状态怎样走向 $s=0$：

- 等速趋近 $\dot s=-\rho\,\mathrm{sgn}(s)$；
- 指数趋近 $\dot s=-\rho\,\mathrm{sgn}(s)-qs$（远处更快）；
- 带边界层：用 $\mathrm{sat}(s/\epsilon)$ 代替 $\mathrm{sgn}(s)$。

控制律由两部分构成：**等效控制**（令 $\dot s=0$ 解出的、维持沿面滑动所需的项）加**切换项**（克服扰动与建模误差）。

### 4.5 抖振：成因、估计与削弱

**成因**：理想滑模要求控制量无限快切换；真实系统有采样保持、执行器延迟、测量量化。状态每次穿过滑模面后"多走一点"，再被推回，形成围绕滑模面的高频小振荡，引起电机发热、机械磨损并激发未建模高频动态。

**估计**：切换增益 $k$（加速度量纲）乘有效延迟 $T_{\rm eff}$（采样保持周期 + 执行器与反馈延迟）。第 7 节用显式仿真验证：纯采样保持时纹波峰峰值**恰好等于** $kT_s$（$k=3000,T_s=2$ ms → 6；$k=6000,T_s=3$ ms → 18）；叠加延迟后峰峰值是 $kT_{\rm eff}$ 的 1.5–1.7 倍（$k=3000$，1 ms 周期 + 1 步延迟 → 9；$k=6000$，1 ms + 2 步延迟 → 30）。所以经验规则：**边界层 $\phi\gtrsim kT_{\rm eff}$，有延迟时留 1.5 倍余量。**

**削弱方法**：

| 方法 | 做法 | 代价 |
|---|---|---|
| 边界层/饱和函数 | $\mathrm{sat}(s/\phi)$ 代替 $\mathrm{sgn}$ | 边界层内变成线性高增益，有静差 $d\phi/k$ |
| tanh 平滑 | 处处可微 | 无精确线性段 |
| 微分滤波 | 对 $\dot e$ 做一阶低通 | 增加相位滞后 |
| 降 $k$ 或趋近律自适应 | 近处减速趋近 | 到达时间略增 |
| 扰动观测器 + 较小 $k$ | 先估计并抵消扰动 | 观测器设计 |
| 高阶滑模（super-twisting） | 让 $\dot u$ 切换，$u$ 连续 | 设计复杂、需导数信息 |

### 4.6 与 PID 的对比

| 维度 | PID | 滑模控制 |
|---|---|---|
| 设计依据 | 线性化标称模型，频域/经验整定 | 扰动上界 + 滑模面，鲁棒性来自切换增益 |
| 对参数/扰动 | 靠积分兜底，慢 | 匹配扰动下不敏感（界内） |
| 代价 | 大误差处理两难、积分饱和 | 抖振、需 $\dot e$（噪声大）、边界层静差 |
| 调参量 | $K_p,K_i,K_d$ | $c,\rho,\epsilon$（或 $k,\phi$） |
| 实现 | 简单 | 简单，但对采样周期、量化敏感 |

---

## 5. 项目版 SMC：参数含义与整定

### 5.1 控制律

$$s=\dot e+ce,\qquad u_{\text{内部}}=-ce-\rho\,\mathrm{sat}(s/\epsilon),\qquad \text{give\_current}=-u_{\text{内部}}$$

$c=5$，$\rho=2$，$\epsilon=1$，输出限幅 10000，$dt=0.001$。**外部负号**用于电机方向修正，算法与硬件解耦。

**手算（笔记）**：目标 6000、实测 5900、上次误差 98、$dt=0.001$：$e=100$，$\dot e=(100-98)/0.001=2000$ rpm/s，$s=2000+5\times100=2500$，$\mathrm{sat}(2500/1)=1$，$u_{\text{内部}}=-500-2=-502$，调用处取负得 $+502$（协议数值，不是安培）。

### 5.2 三个参数的含义和量纲

| 参数 | 值 | 含义与量纲 | 实际作用 |
|---|---|---|---|
| $c$ | 5 | 在滑模面里是衰减率（s⁻¹）；在控制律里是比例系数（计数/rpm）。代码里同一个数同时承担两个角色 | 大误差时 $u\approx-ce$，等价于增益 5 计数/rpm 的比例控制；滑模面上误差时间常数 $1/c=0.2$ s |
| $\rho$ | 2 | 趋近项幅值（计数） | 仅为 ±2 计数的有界偏置，约 0.0024 A |
| $\epsilon$ | 1 | 边界层宽度，单位与 $s$ 相同（rpm/s） | $\lvert s\rvert\le1$ 才处于线性段 |

### 5.3 诚实的批判性分析

1. **$\rho$ 很小。** 切换项幅度 2 计数，只相当于 $ce$ 在 $0.4$ rpm 误差处的大小（$\rho/c=0.4$），而反馈量化分辨率是 1 rpm，所以大误差时系统表现为比例增益 5 的 P 控制加 ±2 的偏置。
2. **$\epsilon=1$ 基本等于符号函数。** 1 rpm 的量化在 1 ms 差分后给 $\dot e$ 引入标准差约 $0.408/0.001=408$ rpm/s 的噪声（均匀量化误差方差 1/12，两次差分方差 1/6），远大于 1 rpm/s；所以 $\lvert s\rvert\le1$ 几乎不会成立，sat 常处于 ±1 饱和，行为等价 $\mathrm{sgn}$。好处是切换幅度被 $\rho=2$ 限制在极小，不会引起可见抖振。
3. **零附近有效增益。** 笔记里粗估零附近增益为 $c(1+\rho/\epsilon)=15$，这是把 $s$ 当作与 $e$ 同比例的粗略写法；因为 $s$ 里含 $\dot e$，这个数只能作参考，不要在面试里当作精确结论。
4. **没有积分。** 常值负载（如摩擦轮空转阻尼）会留下静差；第 8 节仿真给出 −16.6 rpm，约 0.044 m/s，会被慢速的弹速反馈（±50 rpm 一档）修正。
5. **$dt=0.001$ 是名义常数。** 实际任务循环约 3 ms：同样的 $\Delta e$ 除以更小的 $dt$，$\dot e$ 被放大 3 倍。第 7 节算出：名义 $dt=0.001$ 时 $s=2500$，按真实 $dt=0.003$ 算是 1167，**但两者 sat 都是 1，控制量同为 −502**。因为 $\dot e$ 只进 sat 内部，缩放只影响"何时离开饱和"，几乎不改变输出；第 8 节仿真把 dt 改成 0.003，结果逐位相同。这样的名义常数在实机上通过整定吸收，**不要把它说成准确的时间步**。
6. **稳定性。** 项目版 $s$ 含 $\dot e$，而 $\dot e$ 里已经含有上一周期的控制量（对电机-轮这一阶对象，$u$ 直接出现在 $\dot e$ 中），于是 $s$ 对 $u$ 形成一个经由 sat 的内部反馈，经典的 $s\dot s<0$ 到达条件无法直接套用。更准确的说法是：**带边界层的准滑模/非线性比例控制**，其稳定性靠 $c$、限幅与实测，而不是严格证明。

### 5.4 整定法（教学建议顺序）

1. **先定 $c$**：把 $\rho$ 设为 0，调到响应快而不振铃（由电流环带宽与总延迟决定，通常设为内环带宽的几分之一）；
2. **再定 $\rho$**：要盖过的扰动（弹丸冲击折算成的等效指令量）决定幅值；取 $\rho$ 使切换项在已知扰动界上有 20%–50% 余量，同时不超过限幅；
3. **最后定 $\epsilon$**：测出 $s$ 在静止稳态下的噪声标准差 $\sigma_s$，令 $\epsilon\gtrsim2\sigma_s$ 避免 sat 在噪声里乱翻；对 $\dot e$ 先做一阶低通会让 $\sigma_s$ 变小，从而允许更小的 $\epsilon$；
4. **验证**：记录 $s$、未限幅输出、限幅后输出，看是否长时间贴限幅或高频翻转；
5. **回到系统层**：用裁判上报的弹速分档调整（±50 rpm）补偿慢漂移，不要期望速度环一个人解决弹速一致性。

---

## 6. 离散实现（C 代码，教学示例，非项目源码）

```c
#include <math.h>

typedef struct { float c, rho, eps, out_max, dt; float e_prev; } smc_t;

static float sat1(float x) { return x > 1.f ? 1.f : (x < -1.f ? -1.f : x); }

/* 项目形式：s = de + c*e ; u = -c*e - rho*sat(s/eps) ; 调用处再取负 */
float smc_step(smc_t *p, float target_rpm, float meas_rpm)
{
    float e  = target_rpm - meas_rpm;
    float de = (e - p->e_prev) / p->dt;      /* dt 为名义常数 0.001，实际周期更长 */
    float s  = de + p->c * e;
    float u  = -p->c * e - p->rho * sat1(s / p->eps);
    if (u >  p->out_max) u =  p->out_max;
    if (u < -p->out_max) u = -p->out_max;
    p->e_prev = e;
    return -u;                                /* 外部负号：电机方向修正 */
}

/* 变增益 Kp = k0*log10(k1*|e|) + k2 ，带下限保护与上下界 */
float kp_log(float e, float k0, float k1, float k2,
             float e_min, float k_min, float k_max)
{
    float x = fabsf(e);
    if (x < e_min) x = e_min;                 /* 保护 log10(0) */
    float kp = k0 * log10f(k1 * x) + k2;
    if (kp < k_min) kp = k_min;
    if (kp > k_max) kp = k_max;
    return kp;
}
```

要点：（1）`dt` 做成结构体成员，换周期时只改一处，或直接用实测周期；（2）`sat1` 的边界层在 $s/\epsilon$ 内部，所以 $\epsilon$ 与 $s$ 同量纲（rpm/s）；（3）限幅放在输出处，并且在 RELAX 时清零 `e_prev`，避免重新使能的第一拍出现巨大的 $\dot e$。

---

## 7. 四项手算的 Python 复核

以下脚本验证本章的四项手算（对数律反推、边界层稳态误差、$s=\dot e+ce$ 的衰减与项目示例、抖振纹波估计）。**四项手算的结果都与脚本一致**；有两处需要补充：抖振纹波叠加延迟后是 $kT_{\rm eff}$ 的 1.5–1.7 倍，不是恰好等于；项目示例里 $dt$ 改成 0.003 后 $s$ 变了，但 sat 饱和使输出不变。

```python
# check the four hand calculations of chapter 08 (teaching numbers, nominal constants)
import math
sat = lambda x: max(-1.0, min(1.0, x))

# (1) log-law gain design from two points; protect log10(0)
def kp_log(e, k0, k1, k2, emin=1e-3):
    return k0 * math.log10(k1 * max(abs(e), emin)) + k2
k0 = (10 - 6) / math.log10(1000 / 10); k1 = 1 / 10; k2 = 6
print("design:", k0, k1, k2, [kp_log(e, k0, k1, k2) for e in (10, 100, 1000)])
print("notes k=(1,8,5):", [round(kp_log(e, 1, 8, 5), 3) for e in (1, 10, 100, 1000)])

# (2) boundary-layer steady error: T v' = u - v - load, u = v + T k sat(e/phi)
T, k, r, load = 0.15, 5.0, 1.0, 0.6
for phi in (0.2, 0.02):
    v, dt = 0.0, 0.001
    for n in range(6000):
        L = load if n * dt >= 0.5 else 0.0
        e = r - v; v += dt * (k * sat(e / phi) - L / T)
    print("phi", phi, "e_ss sim", round(r - v, 4), "formula d*phi/k", (load / T) * phi / k)

# (3) s = de + c e : decay and the project numeric example (dt nominal 0.001 vs real 0.003)
print([round(100 * math.exp(-5 * t), 2) for t in (0, .1, .2, .4, .6)])
e, e_prev = 6000 - 5900, 98
for dt in (0.001, 0.003):
    ed = (e - e_prev) / dt; s = ed + 5 * e
    print("dt", dt, "de", round(ed), "s", round(s), "u_int", -5 * e - 2 * sat(s / 1.0))

# (4) chattering ripple of u = -k*sign(e) held for Ts, with `d` extra steps of delay
def ripple(k, Ts, d, N=4000, sub=20):
    h, e, hist = Ts / sub, 0.0, []
    us = [0.0] * (d + 1)
    for n in range(N):
        if n % sub == 0: us.append(-k * (1 if e > 0 else -1))
        e += h * us[-1 - d]; hist.append(e)
    tail = hist[N // 2:]; return max(tail) - min(tail)
for k_, Ts, d in ((3000, .001, 1), (3000, .002, 0), (6000, .001, 2), (6000, .003, 0)):
    print(f"k={k_} Ts={Ts} delay_steps={d}: p-p={ripple(k_, Ts, d):.1f}  k*Teff={k_*Ts*(1+d):.1f}")
```

运行输出：

```text
design: 2.0 0.1 6 [6.0, 8.0, 10.0]
notes k=(1,8,5): [5.903, 6.903, 7.903, 8.903]
phi 0.2 e_ss sim 0.16 formula d*phi/k 0.16
phi 0.02 e_ss sim 0.016 formula d*phi/k 0.016
[100.0, 60.65, 36.79, 13.53, 4.98]
dt 0.001 de 2000 s 2500 u_int -502.0
dt 0.003 de 667 s 1167 u_int -502.0
k=3000 Ts=0.001 delay_steps=1: p-p=9.0  k*Teff=6.0
k=3000 Ts=0.002 delay_steps=0: p-p=6.0  k*Teff=6.0
k=6000 Ts=0.001 delay_steps=2: p-p=30.0  k*Teff=18.0
k=6000 Ts=0.003 delay_steps=0: p-p=18.0  k*Teff=18.0
```

核对点：（1）$\log_{10}(0)$ 在 Python 里会抛错，所以函数里用 `max(abs(e), emin)` 保护；（2）`sat` 按 $|x|\le1$ 线性定义，边界层 $\phi$ 与 $e$ 同单位；（3）$s$ 的单位是 rpm/s，$\dot e$（rpm/s）与 $ce$（$c$ 的单位 s⁻¹，乘 rpm）可以相加；（4）$dt$ 名义值 0.001 与实际 0.003 的不一致只改变 $s$ 的数值，不改变本例的输出。

---

## 8. 数值仿真：PID、变增益、滑模的掉速恢复

> **教学示例**：下列模型和参数是为了解释趋势，不是项目实测。

### 8.1 模型与设置

- 对象：$J\dot\omega=K_ti-b\omega-\tau_L$，$J=1\times10^{-4}$ kg·m²，$b=5\times10^{-5}$，$K_t=0.3$ N·m/A；再加电流环一阶滞后 1 ms（二阶模型）；
- 目标 6000 rpm；控制周期 3 ms（贴近实际任务循环）；反馈延迟 1 ms；转速量化到 1 rpm；
- 弹丸：8 ms 的转矩脉冲，传给每个轮约 2.9 J；
- 控制器：PI（$K_p=12,K_i=0.2$）、变增益 PI（设计点 $|e|=20\to8$、$|e|=600\to12$，得 $k_0=2.71,k_1=0.05,k_2=8$，下限 4、上限 12，$e_{\min}=1$ rpm）、一阶滑模 SMC-eq（教学形式：$i=(\hat b\omega+\hat J k\,\mathrm{sat}(e/\phi))/K_t$，$k=30000$ rad/s²，$\phi=100$ rad/s）、项目形式 SMC-proj（$c=5,\rho=2,\epsilon=1$）。

```python
# teaching simulation: friction wheel, second-order (inertia + current lag), 3 ms control period
import math, random
import numpy as np
RPM = 60 / (2 * math.pi)                     # rad/s -> rpm
J, B, KT, R = 1e-4, 5e-5, 0.3, 0.03          # kg m^2, N m s, N m/A, m
CNT = 20 / 16384                             # command counts -> A
R0, DT, TC, TAU_I, FB = 6000.0, 1e-4, 0.003, 0.001, 0.001
sat = lambda x: max(-1.0, min(1.0, x))
clip = lambda x, m: max(-m, min(m, x))

def kp_log(e, k0=2.71, k1=0.05, k2=8.0, emin=1.0, kmin=4.0, kmax=12.0):
    x = max(abs(e), emin)                    # protect log10(0)
    return min(kmax, max(kmin, k0 * math.log10(k1 * x) + k2))

class PI:                                    # fixed gain, counts per rpm
    def __init__(s, gs=False): s.I, s.gs = 0.0, gs
    def step(s, e):
        s.I = clip(s.I + 0.2 * e, 3000)
        kp = kp_log(e) if s.gs else 12.0
        return clip(kp * e + s.I, 12000)
class SMCeq:                                 # first-order surface s=e, equivalent control + sat
    def __init__(s, k=30000.0, phi=100.0): s.k, s.phi = k, phi
    def step(s, e, w):
        i = (5e-5 * w / RPM + J * s.k * sat(e / RPM / s.phi)) / KT
        return clip(i / CNT, 10000)
class SMCproj:                               # project form: s=de+c*e, u=-c e - rho sat(s/eps)
    def __init__(s, dt_nom=0.001): s.ep, s.dtn = 0.0, dt_nom
    def step(s, e):
        s_ = (e - s.ep) / s.dtn + 5 * e; s.ep = e
        return -clip(-5 * e - 2 * sat(s_ / 1.0), 10000)

def run(ctrl, shots, Jf=1.0, Bf=1.0, tl0=0.0, T_end=0.8, E=2.9, pulse=0.008):
    w, i, cmd, nxt = R0 / RPM, 0.0, 0.0, 0.0
    buf = [w] * (int(FB / DT) + 1); ws = []
    for k in range(int(T_end / DT)):
        t = k * DT
        if t >= nxt:
            m = round(buf[0] * RPM); e = R0 - m
            cmd = (ctrl.step(e, m) if isinstance(ctrl, SMCeq) else ctrl.step(e)) * CNT
            nxt += TC
        i += DT * (cmd - i) / TAU_I
        tl = tl0 + sum(E / (w * pulse) for s in shots if s <= t < s + pulse)
        w += DT * (KT * i - B * Bf * w - tl) / (J * Jf)
        buf.append(w); buf.pop(0); ws.append(w * RPM)
    return np.array(ws)

def stats(ws, s0, band=30):                  # drop and recovery time after a shot at s0
    a = int(s0 / DT); seg = ws[a:a + int(0.15 / DT)]
    j = seg.argmin(); ok = np.abs(seg - R0) <= band
    rec = next((n for n in range(j, len(seg)) if ok[n:].all()), None)
    return R0 - seg.min(), None if rec is None else rec * DT * 1e3

def make(name):
    return {"PI": PI(), "GS-PI": PI(True), "SMC-eq": SMCeq(), "SMC-proj": SMCproj()}[name]

if __name__ == "__main__":
    for name in ("PI", "GS-PI", "SMC-eq", "SMC-proj"):
        ws = run(make(name), [0.30])
        d, r = stats(ws, 0.30)
        wm = run(make(name), [], Jf=1.6, Bf=5, tl0=0.03, T_end=0.5)   # mismatch case B
        rng = random.Random(1); v = []
        for _ in range(100):                                          # 3 shots, 80 ms nominal spacing
            s = [0.30, 0.38 + rng.uniform(-.015, .015), 0.46 + rng.uniform(-.015, .015)]
            w2 = run(make(name), s, T_end=0.55)
            v += [0.85 * w2[int(t / DT)] / RPM * R for t in s[1:]]
        print(f"{name:9s} ss={ws[int(.29/DT)]-R0:6.1f}rpm drop={d:5.0f}rpm rec={r:.0f}ms "
              f"mismatch ss={wm[-1]-R0:6.1f}rpm  burst v_std={np.std(v):.3f} m/s")
```

### 8.2 结果（脚本输出）

| 控制器 | 弹前稳态误差 | 掉速 | 恢复到 ±30 rpm | 失配稳态误差 | 3 连发弹速标准差 |
|---|---|---|---|---|---|
| PI | −1.8 rpm | 240 rpm | 52 ms | −2.9 rpm | 0.020 m/s |
| 变增益 PI | −0.9 rpm | 257 rpm | 34 ms | −1.0 rpm | 0.005 m/s |
| SMC-eq（教学形式） | −0.9 rpm | 278 rpm | 27 ms | −49.2 rpm | 0.001 m/s |
| SMC-proj（项目形式） | −16.6 rpm | 363 rpm | 15 ms | −100.3 rpm | 0.001 m/s |

"失配"为 $J\times1.6$、$b\times5$、常值负载 0.03 N·m。弹速标准差由 100 次随机射击间隔（约 80 ms，抖动 ±15 ms）、$v=0.85\,\omega r$ 估算。

### 8.3 抖振检验

保持 $k=30000$ rad/s²，只改 $\phi$（稳态 0.5 s 之后统计）：$\phi=100$ 与 80 时转速标准差约 0.6 rpm 以内；$\phi=60,50$ 约 1–2 rpm，峰峰值 2.7–4.9 rpm；$\phi\le40$ 进入极限环，标准差 400 rpm 以上。$kT_c=30000\times0.003=90$ rad/s，所以 $\phi$ 的下限约为 $kT_c$ 的一半，取 $\phi\approx1.1\,kT_c$ 留余量。

### 8.4 怎么读这张表（也是诚实的边界）

1. **掉速深度主要由能量决定**，在这组参数里控制器只能把它改变约 ±50%（240–363 rpm），而恢复时间的差异更大（15–52 ms）；SMC 掉得更深，因为有效比例增益（约 8.6 计数/rpm，由 $\hat J k/(K_t\phi)$ 换算）比 PI 的 12 低，但没有积分"欠账"，尾部回得快。
2. **标称模型下 PI 已经够好**；仿真里的差异主要来自振铃和积分尾巴。变增益在近零误差处增益更低（$e=1$ rpm 时约 4.5 对 12），更安静。
3. **失配时纯边界层 SMC 静差大**（−49 到 −100 rpm，约 0.13–0.27 m/s），PI 的积分项能纠正。这对应第 4.3 节的 $d\phi/k$ 和第 5.3 节"没有积分"；加积分滑模面或扰动观测器可以修复。
4. **本仿真并不能复现台架数字**：仿真掉速只有 240 rpm（台架 600–900 rpm），PI 恢复 52 ms（台架约 120 ms），连发弹速标准差不到 0.03 m/s（台架约 ±1 m/s）。差距来自仿真没有的因素：电池电压跌落、电流限幅、弹丸批次与擦过摩擦的随机性、弹丸与轮的实际接触时间。**所以本表只支持定性结论（恢复排序、静差与抖振的取舍），不能当作项目数据。**
5. **项目数字（120 ms → 60–80 ms，±1 → ±0.3 m/s）是台架观察**，我们不把全部改善归因于滑模理论本身：换控制器时也同时调整了输出限幅（12000 → 10000）和增益标度。面试里说"整套迭代下来恢复更快、离散更小"，不要说"滑模证明了更好"。

---

## 9. 故障与边界

| 现象 | 可能原因 | 检查/处理 |
|---|---|---|
| 变增益版近零误差电流乱跳 | 噪声让 $\lvert e\rvert$ 在 $e_{\min}$ 附近抖，$\log$ 导致增益抖动 | 设 $e_{\min}$ 不小于量化分辨率；对误差做低通；设 $K_{\min}$ |
| `log10f(0)` 产生 NaN/inf | 没设下限 | 用 $\max(\lvert e\rvert,e_{\min})$ |
| 起转后过冲、第一发偏高 | 积分饱和 | 积分限幅；起转阶段积分分离 |
| SMC 输出高频翻转、电机啸叫 | $\phi$ 小于 $kT_{\rm eff}$；$\dot e$ 噪声大 | 增大边界层，$\dot e$ 低通，降 $k$ |
| 常值负载下转速稳定偏低 | 无积分，$e_{ss}=d\phi/k$ | 缩小 $\phi$、加积分滑模面或 I 项 |
| 换了任务周期后行为变化 | `dt` 名义常数，$\dot e$ 缩放 | 用实测周期或重整定 $\rho,\epsilon$ |
| 输出长期贴限幅 | $k$ 或 $\rho$ 超出执行器能力，理论条件 $k>\lvert d\rvert$ 失效 | 降低要求或加大限幅（受电机/电调限制） |
| 满电弹速超限 | 电池电压高，同一转速目标下弹速偏高 | 弹速反馈：>16.0 m/s 则 fire_speed 绝对值 −50 rpm |
| 第一发和后续弹速不一致 | 连发恢复不足 | 缩短恢复时间；射击间隔与拨盘节拍匹配 |

---

## 10. 面试问答

**Q1. Why not just retune the PID?**
"With a fixed gain I had to trade noise against speed. A high gain recovers fast after a pellet passes but amplifies the one-rpm quantization noise, and the integral term winds up when the current saturates. Retuning could only move along that trade-off, so I wanted the gain to depend on the error."
*中文：固定增益要在噪声和速度之间折中，调参只是在折中线上移动，所以让增益随误差变化。*
追问：*How did you choose the gain law?* — "A logarithmic law, because the error spans three orders of magnitude. I fixed two operating points and solved for the three parameters, and added a floor on the error to protect the logarithm at zero."

**Q2. What is the sliding surface and why?**
"I used s = ė + c e. On s = 0 the error decays like e^(−ct), with c about 5 per second in our setup. The control combines an equivalent term −c e and a saturated switching term −ρ sat(s/ε)."
*中文：s=ė+ce，沿面误差指数衰减；控制律是等效项加饱和切换项。*

**Q3. How do you handle chattering?**
"Chattering comes from switching plus sampling and delay; the ripple scales with k times the effective delay. I used a saturation function instead of sign and kept the switching gain small. In our setup the loop runs at about 3 ms, so I treated the internal dt of 0.001 as a nominal constant and tuned on the real wheel."
追问：*How do you choose the boundary layer?* — "Keep it above k·T_eff to avoid a limit cycle, and below what the steady error d·φ/k allows."

**Q4. Can you prove stability?**
"For the idealised first-order model, V = e²/2 gives V̇ ≤ −(k−|d|)|e| outside the boundary layer, and the error is ultimately bounded by dφ/k inside. For the real wheel, the surface contains the derivative of the error, which already contains the control, so I relied on the boundary layer, current limits and testing rather than a full proof."

**Q5. What were the downsides of your SMC?**
"No integral action, so there is a steady error under constant load; the derivative is noisy; and the fixed dt assumption. I would add an integral sliding surface or a disturbance observer. The slow bullet-speed feedback from the referee system, in steps of 50 rpm, covers the remaining drift."

**Q6. Did the three controllers run together?**
"No. It was an iteration history: PID first, then variable-gain PID, then sliding mode. Only the last one is active in the final version."

**Q7. What improved?**
"In our setup the speed recovery after a burst shortened from about 120 ms to around 60–80 ms, and the bullet-speed spread went from about ±1 m/s to around ±0.3 m/s. I would not claim all of that came from the sliding-mode theory itself; the limits and gains were retuned at the same time."

---

## 本章要点速记

1. 掉速由能量决定，控制器管恢复、过冲与噪声；约 375 rpm 对应 1 m/s。
2. 变增益：误差大增益大；对数律由两点反推 $k_0,k_1,k_2$；$\log(0)$ 要下限保护，$K_p$ 要上下界。
3. 滑模 = 等效控制 + 切换项；$\mathrm{sat}$ 的边界层与 $e$ 同单位。
4. 边界层稳态误差 $e_{ss}=d\phi/k$；Lyapunov：$\dot V\le-(k-|d|)|e|$。
5. $s=\dot e+ce\Rightarrow e=e_0e^{-ct}$；$c=5$ 时每 0.2 s 缩到 37%。
6. 抖振纹波约 $kT_{\rm eff}$，边界层 $\phi\gtrsim kT_{\rm eff}$，有延迟留 1.5 倍。
7. 项目参数 $c=5,\rho=2,\epsilon=1$ 下近似"比例增益 5 + 有界切换"；$dt=0.001$ 是名义常数，实际约 3 ms。
8. 没有积分 → 静差；失配时积分滑模面或观测器修复。
9. 仿真只支持定性结论；项目数字以台架观察为准。
10. PID → 变增益 PID → 滑模是迭代历史，三者不同时运行；弹速反馈来自裁判上报。

## 自测题

1. 期望 $|e|=5$ 时 $K_p=4$，$|e|=500$ 时 $K_p=8$，求 $k_0,k_1,k_2$（取 $k_1=1/5$）。*答：$k_0=4/\log_{10}100=2$，$k_1=0.2$，$k_2=4$。*
2. $k=8$，$\phi=0.4$，$d=2$，稳态误差？*答：$2\times0.4/8=0.1$。*
3. $c=4$，$e_0=80$，0.5 s 后误差？*答：$80e^{-2}=10.8$。*
4. $k=3000$，$T_{\rm eff}=2$ ms，$\phi$ 至少？*答：$kT_{\rm eff}=6$，有延迟时取约 9。*
5. 为什么 sat 比 sign 好？项目里 $\epsilon=1$ 时 sat 还起作用吗？*要点：边界层内连续、降低抖振，代价是残差 $d\phi/k$；项目里因为 $\dot e$ 噪声约 400 rpm/s，$\lvert s\rvert\le1$ 几乎不成立，sat 实际常处于饱和，等价符号函数，抖振由小的 $\rho$ 限制。*

## 相关链接

上一章 [07 发射机构](./07_发射机构_摩擦轮与拨盘.md)；下一章 [09 麦克纳姆轮运动学与底盘跟随](./09_麦克纳姆轮运动学与底盘跟随.md)；[12 数字 PID 实现](./12_数字PID实现.md)；基础：[PID 全解](../../02_基础知识/04_控制理论/02_PID控制全解.md)、[滑模控制](../../02_基础知识/04_控制理论/06_滑模控制.md)；[控制八股](../../03_八股知识/03_机器人学/02_控制八股.md)；[笔记](../../05_学习笔记归档/P3/P3_本对话问答逐字稿.md)。
