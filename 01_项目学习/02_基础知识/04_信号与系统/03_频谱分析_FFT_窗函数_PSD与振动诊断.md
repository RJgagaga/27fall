# 频谱分析：FFT、窗函数、PSD 与振动诊断

> **一句话定位**：当关节发出啸叫、云台高速时抖动、IMU 读数里藏着看不见的周期扰动，你需要一个"频率显微镜"来回答：**能量集中在哪些频率、有多强、是不是同一个共振**。FFT 是显微镜的镜头，窗函数决定镜头的清晰度和杂散光，功率谱密度（PSD）给出可比较的"能量尺子"，Welch 方法让这把尺子不再抖动。

**本章学完你能：**

- 算出 DFT 的频率轴、单边幅值谱的校准系数，说明零填充只做插值、不提高分辨率；
- 解释频谱泄漏（spectral leakage）与栅栏效应（scalloping），手算 50 Hz 与 50.5 Hz 正弦在矩形窗/Hann 窗下的峰值差异；
- 说出矩形窗、Hann、Hamming、Blackman 的相干增益、等效噪声带宽（ENBW）、旁瓣电平，并据此选窗；
- 写出 PSD 的缩放公式，用 Welch 方法估计功率谱，解释平均段数如何降低方差；
- 从 PSD 里读出共振频率与阻尼比（半功率带宽法），区分结构共振、电机阶次谐波、电源干扰与混叠；
- 用 numpy 写出一个可运行的 Welch PSD 函数，并知道 scipy 里对应的参数；
- 用英文回答 "How do you find a resonance from IMU data / why use a window / what is PSD / Welch's method" 等问题。

**前置知识**：[信号与系统基础](./01_信号与系统基础_卷积_傅里叶_拉普拉斯_Z变换.md)（DFT、4 点手算、泄漏初步）、[数字滤波器](./02_数字滤波器_FIR_IIR_采样与混叠.md)（采样、混叠、陷波）、[概率统计与高斯分布](../01_数学基础/03_概率统计与高斯分布.md)（方差、期望）。

## 目录

1. 问题：怎样"看见"振动
2. 符号与单位
3. DFT 的实用细节
4. 频谱泄漏与窗函数
5. 窗函数的选择与手算
6. 功率谱密度（PSD）与 Welch 方法
7. PSD 的单位与读法
8. 振动与共振诊断流程
9. 谱图（spectrogram）：时变频谱
10. 代码：numpy 实现 Welch
11. 在本项目里怎么用
12. 常见错误与故障现象
13. 面试怎么问、怎么答
14. 本章要点速记
15. 自测题
16. 相关链接

---

## 1. 问题：怎样"看见"振动

先设想三个场景。

**场景一**：云台高速转动时能听到明显的"嗡嗡"，画面也抖。打开 IMU 记录，在时域上只看到一团毛刺，根本看不出规律。如果把这段数据变成频谱，可能在某个频率出现一座孤峰——比如 87 Hz——那就是结构共振（或电机/减速器的某个阶次）。知道频率，才能决定是加陷波、改增益还是改机械。

**场景二**：控制器本来稳定，但把某个增益稍微调大，就出现固定频率的啸叫。这是闭环在某个频率上越过了稳定边界。频谱帮助你找到这个频率，并和模型预测的共振相比较。

**场景三**：你要给卡尔曼滤波的测量噪声 $R$ 取值。传感器数据去掉均值后的方差，其实是整个频带上噪声功率的总和；如果噪声主要集中在高频，而滤波器只关心低频，那么 $R$ 应当取"滤波器通带内"的噪声，而不是全频带。PSD 告诉你噪声在频率上怎么分布。

三个场景共同的工具是**谱估计**：把一段有限长的、带噪的采样数据，估计成"各频率上有多少能量"。有限长度和噪声使它不是一个简单的"算一下 FFT"就完事——必须处理泄漏、校准和方差。

## 2. 符号与单位

| 符号 | 含义 | 单位 |
|---|---|---|
| $f_s$ | 采样频率 | Hz |
| $N$ | 一段数据的采样点数 | 无量纲 |
| $\Delta f=f_s/N$ | DFT 频率间隔（频率分辨率的基准） | Hz |
| $T=N/f_s$ | 观测时长 | s |
| $x[n]$ | 采样序列 | 视信号（m/s²、rad/s、V…） |
| $w[n]$ | 窗函数 | 无量纲 |
| $X[k]=\sum_{n=0}^{N-1}x[n]w[n]e^{-j2\pi kn/N}$ | 加窗 DFT | 同 $x$ |
| $\mathrm{CG}=\frac1N\sum w[n]$ | 相干增益（coherent gain） | 无量纲 |
| $\mathrm{ENBW}=N\frac{\sum w^2}{(\sum w)^2}$ | 等效噪声带宽（单位：频点数 bin） | bin |
| $S_{xx}(f)$ | 单边功率谱密度 | $\text{(x 的单位)}^2$/Hz |
| $K$ | Welch 平均段数 | 无量纲 |

**约定**：频点 $k$ 对应频率 $f_k=kf_s/N$，$k=0,\dots,N/2$ 为非负频率（实信号的谱共轭对称，只需一半）。

## 3. DFT 的实用细节

### 3.1 频率轴与分辨率

DFT 的频点间隔 $\Delta f=f_s/N=1/T$。观测越长，间隔越小。例：$f_s=1000$ Hz，$N=1000$（观测 1 s）→ $\Delta f=1$ Hz；若想分辨相距 0.1 Hz 的两个频率，至少要观测约 10 s。**能分辨的最小频率差由观测时长决定，与采样率无关。**

### 3.2 单边幅值谱的校准

对一个幅值为 $A$ 的正弦，若频率恰好落在某个频点上，不加窗的 DFT 在该点的模为 $AN/2$。所以单边幅值谱：

$$\hat A[k]=\frac{2\lvert X[k]\rvert}{N\cdot\mathrm{CG}},\quad k=1,\dots,N/2-1,$$

分母的 CG 补偿加窗带来的幅值衰减。直流分量（$k=0$）与奈奎斯特点不乘 2。

### 3.3 零填充的真相

在数据末尾补零再做更长的 FFT，得到更密的频点。但它只是对**同一个**连续谱（DTFT）做更密的采样（插值），并不提高真实的分辨率：两个相近频率在原谱里混在一起，补零之后依然是一个宽峰。它对于"更准确地定位单个峰"有用，对"分开两个相邻峰"没用。

### 3.4 预处理

1. **去均值/去趋势**：直流和缓慢漂移的能量在零频附近泄漏，淹没低频信息；
2. **确认采样均匀**：FFT 假设等间隔采样。若数据有丢帧、时间戳抖动，要先检查（见 [时间同步、坐标系一致性与数据有效性](../06_状态估计/06_时间同步_坐标系一致性与数据有效性.md)），必要时重采样；
3. **确认没有混叠**：见 [数字滤波器](./02_数字滤波器_FIR_IIR_采样与混叠.md)。若 IMU 内部低通 DLPF 设置得比采样率的一半高，高于 $f_s/2$ 的振动会折叠到低频；
4. **平稳性**：谱分析假设信号统计特性在观测窗内不变，电机变速、负载变化过程的数据需要分段，或用谱图。

## 4. 频谱泄漏与窗函数

### 4.1 为什么泄漏

DFT 隐含地把截取的 $N$ 个点当成无限周期序列的一个周期。若信号在窗内包含的周期数不是整数，周期延拓处波形不连续，相当于在信号上乘了一个矩形窗，时域乘积对应频域卷积：纯正弦的谱线被卷成了 $\mathrm{sinc}$ 形状的主瓣和一串旁瓣，能量"泄漏"到远处。

### 4.2 窗函数的作用与代价

加窗就是把窗两端平滑地拉向零，降低延拓处的不连续，代价是**主瓣变宽**：

$$\text{窗越平滑}\Rightarrow\text{旁瓣越低（泄漏越少）}\ \&\ \text{主瓣越宽（分辨率越差）}.$$

这是时频测不准关系的又一个体现，没有"既无泄漏又无主瓣展宽"的窗。

### 4.3 栅栏效应（scalloping）

DFT 只在离散频点上有值。若信号频率落在两个频点正中间（偏离半个 bin），峰值被"栅栏"挡住一部分，幅值读数偏低，这个损失叫 scalloping loss。不同窗的损失不同。

## 5. 窗函数的选择与手算

### 5.1 数值汇总

对 $N=1000$ 的周期型窗（Python 计算）：

| 窗 | 相干增益 CG | ENBW/bin | 峰值旁瓣 | 半 bin 偏移时的幅值读数 | scalloping |
|---|---|---|---|---|---|
| 矩形 | 1.000 | 1.000 | −13.3 dB | 0.640 | 3.88 dB |
| Hann | 0.500 | 1.500 | −31.5 dB | 0.849 | 1.42 dB |
| Hamming | 0.540 | 1.363 | −42.7 dB | 0.818 | 1.75 dB |
| Blackman | 0.420 | 1.727 | −58.1 dB | 0.881 | 1.10 dB |

读法：矩形窗分辨率最高（ENBW 最小）但旁瓣最差，Hann 是最常用的折中，Blackman 旁瓣最低但主瓣最宽。（Flat-top 窗的幅值精度最好，常用于幅值校准，但分辨率最差，此处未列数值。）

### 5.2 手算：50 Hz 与 50.5 Hz

$f_s=1000$ Hz，$N=1000$，$\Delta f=1$ Hz，输入幅值 1 的余弦。

- **50 Hz**（恰在频点）：任何窗校准后读数都是 $1.000$；
- **50.5 Hz**（偏半 bin）：矩形窗读数 $0.640$（低了 36%，即 3.88 dB），Hann 读数 $0.849$，Blackman 读数 $0.881$。

远离峰值 10 个 bin 处的泄漏幅值：矩形窗约 $0.03$（即 −30 dB），Hann 约 $0.0003$（−71 dB）。这解释了为什么要看小幅度的谐波时必须加窗：矩形窗的旁瓣会把大峰的泄漏盖在小峰上。

### 5.3 手算：两个相近的音

两个幅值为 1 的余弦，频率 50 Hz 与 52 Hz（相差 2 个 bin）。矩形窗下，谱在 50 与 52 各有一个独立的峰，中间 51 Hz 为零，可分辨；Hann 窗下，49–53 Hz 的读数依次为 $0.5,\ 1,\ 1,\ 1,\ 0.5$（Python 复核），形成一个平台，两个峰**已经分不开**。结论：Hann 窗的主瓣宽度约 $\pm2$ bin，想分辨相隔小于约 2–3 bin 的成分，要么延长观测时间（减小 $\Delta f$），要么换矩形窗并接受更高的泄漏。

### 5.4 选窗经验

1. 信号恰好含整数个周期（同步采样，如电机转速已知）→ 矩形窗最好；
2. 通用的"找峰/看噪声"→ Hann；
3. 需要精确幅值读数 → flat-top（读数偏差小，分辨率差）；
4. 需要看清强信号旁边的微弱信号 → Blackman 或更低旁瓣的窗；
5. 瞬态信号（锤击响应、冲击）→ 力窗/指数窗，不适合周期窗。

## 6. 功率谱密度（PSD）与 Welch 方法

### 6.1 为什么要 PSD

幅值谱适合"几根孤立的正弦"；噪声（连续谱）用幅值谱表达会随 $N$ 变化——$N$ 越大，每个 bin 里的噪声幅值越小，因为每个 bin 变窄了。需要一个**与 $N$ 无关**的量：功率谱密度，单位是"功率/Hz"。

### 6.2 缩放公式

周期图（periodogram）估计，单边 PSD：

$$\hat S_{xx}[k]=\frac{2\,\lvert X[k]\rvert^2}{f_s\sum_nw[n]^2},\quad k=1,\dots,N/2-1.$$

直流与奈奎斯特点不乘 2。验证（Parseval）：对 $\hat S_{xx}$ 在频率上积分 $\sum_k\hat S_{xx}[k]\Delta f$ 应该等于信号方差。

**手算**：方差 $\sigma^2$ 的白噪声，采样率 $f_s$，其单边 PSD 是常数 $S=2\sigma^2/f_s$。取 $\sigma=0.5$，$f_s=1000$：$S=0.0005$（单位² /Hz）。Python 用 10 s 数据（$N_{\text{seg}}=1024$，Hann，重叠 50%，共 $K=18$ 段）估计，噪声底的中位数为 $4.93\times10^{-4}$，与理论 $5\times10^{-4}$ 相差 1.4%。再把峰值周围 7 个 bin 积分，叠加的 50 Hz 正弦（幅值 1，功率 $A^2/2=0.5$）得到 $0.507$，与理论 $0.5$ 相差 1.3%。这说明缩放公式既能还原噪声密度，也能还原窄带信号的功率。

### 6.3 周期图的方差问题

周期图是 PSD 的无偏估计，但**方差很大**：对白噪声，每个频点估计值的标准差与均值相当（相对标准差约 100%），并且不随 $N$ 增大而减小——数据更长只是让频点更多、更密，每个点依然很抖。Python 做 2000 次蒙特卡洛，单个频点周期图的相对标准差是 $0.986$。

### 6.4 Welch 方法

Welch 的思想：把长数据切成 $K$ 个（可重叠的）短段，每段加窗、算周期图，再对各段的周期图**取平均**。平均 $K$ 个近似独立的估计，相对标准差约降为 $1/\sqrt K$，代价是每段变短，频率分辨率变差（$\Delta f=f_s/N_{\text{seg}}$）。常用设置：Hann 窗、重叠 50%。

**数值验证**：10 s 数据，$N_{\text{seg}}=1024$，重叠 50%，$K=18$。蒙特卡洛 400 次，单个频点的相对标准差 $0.239$，对比理论 $1/\sqrt{18}=0.236$（Python 复核）。把 300 个频点再平均，相对标准差只有 $0.020$。这就是"噪声底用 Welch，再对频带平均"能把数据读到 2% 精度的原因。

### 6.5 分辨率与方差的权衡

段长 $N_{\text{seg}}$ 越长→分辨率越好，但段数 $K$（总数据固定时）越少→方差越大。实践：先根据需要分辨的频率差确定 $N_{\text{seg}}\gtrsim\text{ENBW}\cdot f_s/\Delta f_{\text{need}}$，再用剩余数据尽量多地平均；不够就多采一些数据。

## 7. PSD 的单位与读法

1. **PSD** 单位是"信号单位²/Hz"；对加速度（m/s²）是 (m/s²)²/Hz；
2. **ASD（幅值谱密度）**：PSD 的平方根，单位"信号单位/√Hz"。IMU 数据手册常给的"噪声密度"（如 μg/√Hz、°/s/√Hz）就是 ASD；
3. **积分**：在 $[f_1,f_2]$ 上对 PSD 积分得到该频带内的方差，开根号为 RMS。例如已知噪声密度 $\sigma_f$（ASD，信号单位/√Hz），白噪声带宽 $B$ 内的 RMS $=\sigma_f\sqrt B$；
4. **与离散方差的关系**：采样率 $f_s$ 下，离散白噪声方差 $\sigma_d^2=S\cdot f_s/2$（单边 $S$）。这一关系把数据手册的噪声密度转换成滤波器里的 $R$ 或 $Q$（下一章 [随机信号、自相关、白噪声与 Allan 方差](./04_随机信号_自相关_功率谱_白噪声与Allan方差.md) 详细讲）；
5. **dB**：功率比取 $10\log_{10}$，幅值比取 $20\log_{10}$；比较两个 PSD 时用对数纵轴（双对数更常见）。

## 8. 振动与共振诊断流程

### 8.1 步骤

1. **设定采集**：采样率至少是关心的最高频率的 2.5–5 倍；开 IMU 的内部抗混叠滤波；记录足够长（至少 $K\ge20$ 段）；
2. **激励**：稳态运行（固定转速）、随机激励（白噪声指令）、扫频或锤击；
3. **预处理**：去均值，剔除含丢帧的段，分段；
4. **计算 Welch PSD**，用对数纵轴；
5. **找峰**：峰频、峰高、半功率带宽；
6. **判断来源**；
7. **验证**：改变条件（转速、负载、采样率），观察峰是否按预期移动。

### 8.2 判断峰的来源

| 峰的特征 | 可能来源 | 验证方法 |
|---|---|---|
| 频率固定，不随转速变，峰较窄 | 结构共振（机械固有频率） | 改转速/指令，峰频不变；敲击测试 |
| 频率与转速成正比（基频及其倍频） | 电机阶次、齿轮啮合、风扇、编码器误差 | 改变转速看峰是否同比例移动 |
| 50/60 Hz 及其谐波 | 电源/电磁干扰 | 断开/接地后看峰是否消失 |
| 采样率一半附近的"假峰" | 混叠 | 改采样率，峰是否移动 |
| 低频宽缓隆起 | 控制环响应、重力/摆动 | 开/关控制器对比 |
| 宽带抬高 | 噪声增大、电流环 PWM 噪声 | 与关机状态对比 |

### 8.3 从半功率带宽估计阻尼比

对二阶系统在白噪声激励下，响应的 PSD 是 $\lvert H(j\omega)\rvert^2$。在共振频率 $f_n$ 附近，幅度平方降到峰值一半的两点（半功率点）之间的宽度 $B_{3\text{dB}}\approx2\zeta f_n$，所以

$$\zeta\approx\frac{B_{3\text{dB}}}{2f_n}.$$

**仿真验证**（教学示例）：二阶系统 $f_n=87$ Hz，$\zeta=0.05$，白噪声激励，$f_s=1000$ Hz，段长 2048，Hann 窗，重叠 50%。理论半功率带宽 $2\zeta f_n=8.7$ Hz。Python 结果：

- 57 段平均：峰在 86.9 Hz，半功率带宽 6.35 Hz，估得 $\zeta\approx0.037$（**偏低 27%**）；
- 389 段平均：峰在 86.9 Hz，半功率带宽 9.16 Hz，$\zeta\approx0.053$（偏差 5%）。

两个教训：（1）频率读数很稳；（2）阻尼估计对 PSD 的方差敏感，噪声峰会抬高"一半峰值"的阈值，使带宽被低估，段数不够时要平滑，或用曲线拟合代替读点；也要注意频率分辨率 $\Delta f=0.49$ Hz 相对于带宽 8.7 Hz 是否足够细。

### 8.4 从频谱到控制动作

找到共振 $f_r$ 后，回到 [柔性关节、弹性驱动、摩擦与接触建模](../02_力学与动力学建模/07_柔性关节_弹性驱动_摩擦_接触建模.md) 和 [数字滤波器](./02_数字滤波器_FIR_IIR_采样与混叠.md)：把控制环的穿越频率压到 $f_r$ 的约 1/3 以下，或在 $f_r$ 设置陷波器，陷波宽度覆盖由负载变化引起的 $f_r$ 漂移范围。

### 8.5 采集参数怎么定

一个常见的问题是"采样率到底设多少"。经验上分三步：先估计你关心的最高频率 $f_{\max}$（例如结构共振在 100 Hz 量级，则 $f_{\max}$ 取 150–200 Hz）；再让采样率不低于 $2.5$ 到 $5$ 倍的 $f_{\max}$，留出抗混叠滤波器的过渡带；最后确认传感器内部的低通（IMU 的 DLPF）截止频率设在 $f_s/2$ 以下，否则在传感器输出端就已经存在无法去除的混叠。观测时长则由所需的频率分辨率和平均段数共同决定：如果希望分辨约 1 Hz，Hann 窗下每段至少 2 s；再要 20 段以上的平均，总计至少 20 s 的稳态数据。很多"谱峰读不准"的问题，其实是数据太短，而不是算法不对。

另一个容易忽略的点是**记录时的运行条件要稳定**。谱分析的前提是平稳性：如果在 20 s 的记录里电机转速在缓慢爬升，阶次谱线会被拖成一条宽带，看起来像噪声。此时要么固定转速分档测量，要么用谱图把每个时刻分开看。对机器人关节，通常固定指令、保持负载不变的稳态运行（或周期性往复运动，取整数个周期）最容易得到干净的谱。

## 9. 谱图（spectrogram）：时变频谱

整段数据一个 PSD 会把"某时刻出现的啸叫"平均掉。**短时傅里叶变换（STFT）**每次只取一小段（例如 0.25 s）加窗做 FFT，沿时间滑动，得到"时间—频率—强度"的二维图，即谱图。它让你看到：（1）啸叫是在哪一个动作开始的；（2）与转速成比例的阶次线随时间的移动；（3）冲击事件的宽带能量。代价是**时间分辨率与频率分辨率的反比关系**：窗越短，时间分辨率越好，频率分辨率越差。

## 10. 代码：numpy 实现 Welch

下面是教学示例（非项目源码），已用 numpy 实际运行：

```python
import numpy as np

def welch_psd(x, fs, nper=1024, overlap=0.5):
    """单边 PSD，Hann 窗（周期型），线性去均值，返回 f, P, K。"""
    w = np.hanning(nper + 1)[:-1]            # 周期型 Hann
    step = int(nper * (1 - overlap))
    U = np.sum(w ** 2)
    segs = [x[i:i + nper] for i in range(0, len(x) - nper + 1, step)]
    P = np.mean([np.abs(np.fft.rfft((s - s.mean()) * w)) ** 2
                 for s in segs], axis=0) * 2.0 / (fs * U)
    P[0] /= 2.0                               # 直流、奈奎斯特不乘 2
    P[-1] /= 2.0
    return np.fft.rfftfreq(nper, 1.0 / fs), P, len(segs)
```

对应 scipy 的写法是 `scipy.signal.welch(x, fs, window='hann', nperseg=1024, noverlap=512, scaling='density')`（参数名为一般用法，具体默认值随版本可能变化，未逐版本核对）。读 IMU 数据时务必记录单位（g 还是 m/s²，°/s 还是 rad/s），否则 PSD 的数值无法比较。

## 11. 在本项目里怎么用

> 说明：CV 没有写频谱分析项目；以下是在 P1/P3/P7 等项目的调试里**可能用到**的位置，面试时说成"我如何诊断振动问题的方法"。

- **P3 云台**：高速转动时云台抖，可以采集 IMU 角速度或电机电流，做 Welch PSD。若峰值频率与 [电机编码器与力矩控制](../11_嵌入式与通信/05_电机_编码器_驱动与力矩控制.md) 里的某个传动共振吻合，就在速度环加陷波或降低速度环增益（见 [云台串级 PID 与视觉跟踪](../../01_项目课程/P3_RoboMaster英雄机器人嵌入式控制/06_云台串级PID与视觉跟踪.md)）。
- **P1 / P7 平衡**：IMU 陀螺的噪声谱与机体振动谱决定了姿态环应取多大的滤波截止频率；轮腿落地冲击的宽带能量，可用谱图观察（见 [数字滤波器](./02_数字滤波器_FIR_IIR_采样与混叠.md) 和 [P7 调试与故障手册](../../01_项目课程/P7_可变腿长轮腿机器人/11_调试与故障手册.md)）。
- **P2 主从示教**：leader 臂手拖产生的力矩信号，其频谱用于判断人手抖动的频带，从而决定指令平滑滤波器的截止频率（见 [机械臂轨迹生成](../03_运动学与坐标变换/08_机械臂轨迹生成_多项式_梯形速度_S曲线.md) 中的在线平滑）。
- **全局**：为卡尔曼滤波的 $Q$、$R$ 提供依据（见 [卡尔曼滤波与 EKF](../06_状态估计/03_卡尔曼滤波与EKF.md)）。

## 12. 常见错误与故障现象

| 现象 | 可能原因 | 处理 |
|---|---|---|
| 小幅值谱线被大峰的"裙边"盖住 | 矩形窗泄漏 | 改 Hann/Blackman |
| 同一个正弦，每次测得幅值不同 | 频率落在 bin 间，栅栏效应 | 用 flat-top 校幅值，或 FFT 峰值插值 |
| 噪声底随 $N$ 变化 | 用了幅值谱而非 PSD | 改用 PSD 并正确缩放 |
| PSD 曲线毛刺很多 | 周期图方差大，未平均 | 用 Welch，增大 $K$ |
| 想分开两个近峰，加了零填充还是一个峰 | 零填充不提高分辨率 | 延长观测或换更窄主瓣的窗 |
| 低频有巨大的"斜坡" | 未去均值/去趋势 | 去直流与线性趋势 |
| 出现不存在的高频峰 | 混叠 | 提高采样率/开抗混叠滤波 |
| 谱峰位置对不上机械共振 | 采样时间戳不均匀 | 检查时间戳，重采样 |
| 估得的阻尼比忽大忽小 | PSD 方差大，读点方法敏感 | 增加平均、曲线拟合 |
| 数值与数据手册对不上 | 单位或单边/双边混淆 | 核对 g、dB、单边/双边、√Hz |

## 13. 面试怎么问、怎么答

**Q1. How would you find a mechanical resonance from IMU data?**

> I would log the IMU at a rate well above the frequencies of interest with its anti-aliasing filter on, excite the system, for example with a broadband command or by tapping, and compute a Welch power spectral density with a Hann window and many averages. A narrow peak that stays at the same frequency when I change speed is a structural resonance. I would read the peak frequency and estimate damping from the half-power bandwidth, then decide on a notch filter or a lower loop bandwidth.

中文：用高于关注频率的采样率记录 IMU（开抗混叠）；用宽带指令或敲击激励；做 Hann 窗、多段平均的 Welch PSD；转速改变时频率不动的窄峰是结构共振；读峰频并用半功率带宽估阻尼；再决定加陷波或降带宽。

**Q2. Why do we apply a window before the FFT?**

> The FFT assumes the data segment is one period of a periodic signal. If the segment does not contain an integer number of periods, the abrupt ends cause spectral leakage and strong signals hide weak ones. A window tapers the ends to reduce leakage, at the cost of a wider main lobe and so a coarser resolution. Hann is the usual compromise.

中文：FFT 假设数据段是周期信号的一个周期，非整数周期的截断造成泄漏，大峰淹没小峰；窗函数把两端拉向零以降泄漏，代价是主瓣变宽、分辨率变差；Hann 是常用折中。

**Q3. What is the difference between amplitude spectrum and PSD?**

> The amplitude spectrum is good for isolated sinusoids, but for noise its level depends on the number of points. The power spectral density normalises by the frequency resolution and the window power, so the noise level has units of signal squared per hertz and does not depend on the record length. Integrating the PSD over a band gives the variance in that band, and the square root of the PSD is the amplitude spectral density that data sheets quote.

中文：幅值谱适合孤立正弦，噪声的幅值谱随点数变化；PSD 对频率分辨率和窗功率归一化，单位是信号²/Hz，与记录长度无关；频带积分得方差，开方得数据手册常用的 ASD。

**Q4. Why Welch's method and not a single long FFT?**

> A single periodogram is an unbiased estimate but its variance does not shrink with more data; each frequency point has a standard deviation about equal to its mean. Welch's method splits the data into overlapping windowed segments and averages their periodograms, which reduces the variance roughly by the number of segments, at the cost of coarser frequency resolution. In a Monte Carlo test with 18 segments the relative standard deviation fell from about 1.0 to about 0.24.

中文：单个周期图无偏但方差不随数据变长而减小；Welch 切段加窗再平均，方差约降为 $1/K$，代价是分辨率变粗；蒙特卡洛 18 段时相对标准差由约 1.0 降到约 0.24。

**Q5. Two peaks 2 Hz apart merge into one in your spectrum. What do you do?**

> The frequency resolution is limited by the record length and the window main lobe, and zero padding will not help. I either record longer so that the bin spacing is much finer than 2 Hz, or use a window with a narrower main lobe such as the rectangular window if leakage is acceptable. I also check that the two peaks are not an artefact of non-stationary speed.

中文：分辨率由记录长度和窗主瓣决定，零填充无效；要么延长记录使 bin 间隔远小于 2 Hz，要么换主瓣更窄的窗（可接受泄漏时用矩形窗）；并检查是否因转速不稳造成假峰。

## 14. 本章要点速记

1. $\Delta f=f_s/N=1/T$；分辨率由观测时长决定，与采样率无关。
2. 单边幅值谱 $2\lvert X\rvert/(N\cdot\mathrm{CG})$；零填充只插值，不提高分辨率。
3. 泄漏源于非整数周期截断；窗越平滑，旁瓣越低、主瓣越宽。
4. Hann：CG 0.5，ENBW 1.5 bin，旁瓣 −31.5 dB；矩形：ENBW 1，旁瓣 −13.3 dB；半 bin 偏移矩形窗读数低 3.88 dB，Hann 1.42 dB。
5. 单边 PSD $=2\lvert X\rvert^2/(f_s\sum w^2)$；白噪声 PSD $=2\sigma^2/f_s$。
6. 周期图相对标准差约 100%；Welch 平均 $K$ 段降为约 $1/\sqrt K$。
7. PSD 积分得方差；ASD=√PSD，单位 /√Hz。
8. 结构共振的峰不随转速移动；电机阶次随转速移动；50/60 Hz 是电源干扰；混叠峰随采样率移动。
9. 阻尼比 $\zeta\approx B_{3\text{dB}}/(2f_n)$，估计前要充分平均。
10. 谱图用于时变信号，时间与频率分辨率此消彼长。

## 15. 自测题

1. $f_s=2000$ Hz，$N=4096$，频率分辨率？（$0.488$ Hz。）
2. 想分辨 0.2 Hz 的两个频率，至少要观测多久？（约 $1/0.2=5$ s，用 Hann 窗则需更长，约 $2$–$3$ 倍。）
3. 方差 $0.04$ 的白噪声，$f_s=500$ Hz，单边 PSD 多大？（$2\times0.04/500=1.6\times10^{-4}$）
4. 为什么 Welch 比单次 FFT 噪声底更平滑？（对 $K$ 段周期图取平均，方差降为约 $1/K$。）
5. 峰值 87 Hz，半功率带宽 9 Hz，阻尼比？（$9/(2\times87)=0.052$。）
6. 怎样判断一个峰是不是混叠假峰？（改变采样率或抗混叠滤波，假峰的位置会移动，真峰不动。）

## 16. 相关链接

- 上一章：[数字滤波器：FIR、IIR、采样与混叠](./02_数字滤波器_FIR_IIR_采样与混叠.md)；下一章：[随机信号、自相关、功率谱、白噪声与 Allan 方差](./04_随机信号_自相关_功率谱_白噪声与Allan方差.md)
- 相关基础：[信号与系统基础](./01_信号与系统基础_卷积_傅里叶_拉普拉斯_Z变换.md)、[柔性关节、弹性驱动、摩擦与接触建模](../02_力学与动力学建模/07_柔性关节_弹性驱动_摩擦_接触建模.md)、[传感器：IMU、编码器、激光雷达](../06_状态估计/01_传感器_IMU_编码器_激光雷达.md)
- 项目：[P3](../../01_项目课程/P3_RoboMaster英雄机器人嵌入式控制/README.md)、[P1](../../01_项目课程/P1_AIRS球形机器人平衡控制/README.md)、[P7](../../01_项目课程/P7_可变腿长轮腿机器人/README.md)
- 模块说明：[04_信号与系统 README](./README.md)
