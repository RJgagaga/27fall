# ADC、PWM、定时器与 FOC 电机控制原理

> **一句话定位**：你的 CAN 命令最终变成电机里的电流，靠的是一条"定时器产生 PWM → 逆变桥 → 电机电流 → ADC 采样 → 坐标变换 → 电流环"的链。本章把这条链从下往上讲透：PWM 的频率与分辨率、死区、ADC 采样与 PWM 同步、Clarke/Park 变换、dq 电流环 PI 整定、SVPWM 与完整 FOC 流程，使你知道 P3 的 M3508/GM6020 和 P7 的关节电机"电调里面"在做什么。

**本章学完你能：**

- 由定时器时钟、预分频、自动重装载值计算 PWM 频率、占空比分辨率，区分边沿对齐与中心对齐；
- 解释死区（dead time）为什么必须有，并算出 STM32 高级定时器的死区寄存器值；
- 设计 ADC 的采样时刻（与 PWM 同步），由 ADC 码值反算相电流；
- 手算 Clarke 与 Park 变换并验证 $i_d,i_q$；
- 由电感、电阻、目标带宽整定电流环 PI 参数，写出 dq 解耦项；
- 解释 SVPWM（最小-最大注入）的占空比计算与线性调制范围 $V_{dc}/\sqrt3$；
- 画出完整 FOC 的一个 PWM 周期内要做的事，并估算 CPU 负载；
- 说清楚本仓库项目里"哪些是我们做的，哪些是电机驱动器内部做的"。

**前置知识：**[电机、编码器、驱动与力矩控制](./05_电机_编码器_驱动与力矩控制.md)（电机模型、FOC 概念）；[STM32 一章](./01_STM32_时钟_GPIO_中断_DMA.md)（定时器、中断、DMA）；[PID 控制全解](../05_控制理论/02_PID控制全解.md)；[坐标变换与旋转矩阵](../03_运动学与坐标变换/01_坐标系_旋转矩阵_齐次变换.md)（旋转的线性代数含义）。

**相关项目：**[P3 CAN 总线与电机反馈](../../01_项目课程/P3_RoboMaster英雄机器人嵌入式控制/04_CAN总线与电机反馈.md)（电调接收电流指令）、[P7 VMC 原理与关节力矩映射](../../01_项目课程/P7_可变腿长轮腿机器人/06_VMC原理与关节力矩映射.md)、[P7 STM32H7/FreeRTOS/FDCAN](../../01_项目课程/P7_可变腿长轮腿机器人/10_STM32H7_FreeRTOS_FDCAN软件结构.md)（MIT 协议力矩命令）。

## 目录

1. 要解决什么问题
2. 符号与单位
3. 定时器与 PWM
4. 死区
5. ADC 与电流采样
6. 采样与 PWM 的同步
7. Clarke 与 Park 变换
8. dq 电流环与 PI 整定
9. SVPWM 与占空比计算
10. 完整 FOC 流程与 CPU 负载
11. 完整手算汇总
12. 在七个项目里怎么用
13. 常见错误与故障现象
14. 英文面试问答
15. 要点速记、自测题、相关链接

---

## 1. 要解决什么问题

在 [电机一章](./05_电机_编码器_驱动与力矩控制.md) 里我们知道：永磁同步电机（PMSM）的力矩与 q 轴电流成正比 $T=\tfrac32 p\,\psi_f\,i_q$。要控制力矩，就要**精确控制 $i_q$**。但电机的三相电流是随转子角度变化的正弦量；直接用 PI 控制正弦量会有稳态误差（PI 对直流无差，对正弦有差）。FOC（Field-Oriented Control，磁场定向控制）的思路：

1. 测相电流；
2. 通过坐标变换把三相交流量变成**旋转坐标系**下的直流量 $i_d,i_q$；
3. 对直流量用 PI 控制，输出 $v_d,v_q$；
4. 反变换回三相电压，用 PWM 产生。

整条链路有三个"硬件细节"决定成败：**PWM 怎么产生**、**死区怎么设**、**ADC 在什么时刻采样**。这些在 STM32 上由定时器与 ADC 外设实现。

---

## 2. 符号与单位

| 符号 | 含义 | 单位 |
|---|---|---|
| $f_{tim}$ | 定时器输入时钟 | Hz |
| PSC、ARR | 预分频、自动重装载寄存器值 | 计数值 |
| $f_{pwm}$ | PWM 频率 | Hz |
| $D$ | 占空比 | 0–1 |
| $t_{dead}$ | 死区时间 | s（ns–µs） |
| $V_{dc}$ | 直流母线电压 | V |
| $i_a,i_b,i_c$ | 三相电流 | A |
| $i_\alpha,i_\beta$ | 静止两相电流 | A |
| $i_d,i_q$ | 旋转坐标系电流 | A |
| $\theta_e$ | 转子电角度 | rad |
| $p$ | 极对数 | 无 |
| $\theta_e=p\,\theta_m$ | 电角度与机械角度关系 | rad |
| $L,R$ | 相电感、相电阻（dq 等效） | H、Ω |
| $\psi_f$ | 永磁磁链 | Wb |
| $\omega_e$ | 电角速度 | rad/s |
| $k_t$ | 转矩常数 | N·m/A |

本章采用**幅值不变**的 Clarke 变换约定：变换后 $\alpha\beta$ 分量幅值等于相电流幅值，此时 $T=\tfrac32 p\psi_f i_q$。若采用功率不变约定，系数不同，**面试时先说清约定**。

---

## 3. 定时器与 PWM

### 3.1 计数与频率

STM32 通用/高级定时器有一个 16 位（或 32 位）计数器。时钟经预分频后每个计数周期递增。

- **边沿对齐（edge-aligned）**：向上计数 0 → ARR，然后回 0。一个周期有 ARR+1 个计数。
  $$ f_{pwm}=\frac{f_{tim}}{(PSC+1)(ARR+1)}. $$
- **中心对齐（center-aligned）**：先向上到 ARR，再向下回 0（上下计数）。一个周期是 $2\cdot ARR$ 个计数（视模式有 $2ARR$ 或 $2ARR+...$ 的细微差别，以参考手册为准）。
  $$ f_{pwm}\approx\frac{f_{tim}}{(PSC+1)\cdot 2\cdot ARR}. $$

电机控制几乎总用**中心对齐**，原因：三相 PWM 的开关沿对称分布，电流纹波更小；更关键的是，**所有相的"低侧全导通中点"在同一时刻（计数器下溢/上溢处）**，ADC 可以在那个时刻采样电流，避开开关噪声（第 6 节）。

### 3.2 手算：20 kHz PWM

STM32F407 的 TIM1 挂在 APB2，定时器时钟 168 MHz（教学示例，沿用 F407 的典型配置）。要中心对齐 20 kHz，PSC = 0：

$$ ARR=\frac{f_{tim}}{2\,f_{pwm}}=\frac{168\times10^6}{2\times 20\times10^3}=4200. $$

占空比分辨率：占空比取值 0–4200，约 $\log_2 4200=12.04$ 位。若 $f_{pwm}$ 提高到 40 kHz，ARR = 2100，约 11 位。**频率与分辨率相互制约**。

### 3.3 为什么选 16–20 kHz

- 高于人耳听觉上限（约 20 kHz），避免电机啸叫；
- 开关频率越高，电流纹波越小，但开关损耗正比于频率，MOSFET 发热增加；
- 电流环带宽一般取 PWM 频率的 1/10 到 1/20。20 kHz 对应 1–2 kHz 的电流环带宽。

### 3.4 占空比寄存器与更新

CCR（捕获比较寄存器）决定占空比。PWM 模式 1，向上计数时 CNT < CCR 为有效电平。**更新应在周期边界（更新事件）同步生效**：使能 CCR 预装载（preload），写入的新值在下一个更新事件才生效，避免在周期中间改变导致毛刺。

---

## 4. 死区

### 4.1 要解决什么问题

逆变桥每一相由上管（高侧）和下管（低侧）两个开关串联接在母线和地之间。如果上下管**同时导通**，母线直接短路（称为"直通 / shoot-through"），电流瞬间达到数百安培，烧毁功率管。理想 PWM 里上下管是互补的：一个关断的同时另一个开通。但 MOSFET 的关断需要时间（栅极电荷放电，数十到数百纳秒），驱动器还有传播延迟。所以必须在互补信号之间加一段**两管都关断**的时间——死区（dead time）。

### 4.2 死区取值

$$ t_{dead}\ge t_{off,max}-t_{on,min}+t_{margin}, $$

其中 $t_{off,max}$ 是最慢的关断延迟（含驱动器），$t_{on,min}$ 是最快的开通延迟，$t_{margin}$ 是裕量。常见量级 0.5–2 µs（视器件而定，**以器件手册为准**）。死区的副作用：在死区期间，相电压由电流方向决定的二极管续流决定，造成输出电压偏差（电流过零附近畸变，导致力矩脉动）。因此死区**不能无限大**，通常还有死区补偿。

### 4.3 STM32 的 DTG 寄存器

高级定时器 TIM1/TIM8 的 `BDTR` 寄存器中 DTG[7:0] 决定死区，基准是 $t_{DTS}$（死区时钟，常等于定时器时钟周期，由 CKD 选择）：

| DTG[7:5] | 死区时间 |
|---|---|
| 0xx | $DTG[7:0]\times t_{DTS}$ |
| 10x | $(64+DTG[5:0])\times 2\,t_{DTS}$ |
| 110 | $(32+DTG[4:0])\times 8\,t_{DTS}$ |
| 111 | $(32+DTG[4:0])\times 16\,t_{DTS}$ |

**手算**：$t_{DTS}=1/168\ \text{MHz}=5.952$ ns。要 1 µs 死区：1 µs / 5.952 ns = 168 个 $t_{DTS}$。

- 第一档最大 127 个，不够；
- 第二档 $(64+x)\times2=168$，得 $x=20$，即 DTG[5:0] = 010100，DTG[7:5] = 100，整体 `0b10010100 = 0x94 = 148`；
- 验证：$(64+20)\times2\times5.952\ \text{ns}=1000\ \text{ns}$。

```python
f = 168e6
tdts = 1/f
units = 1e-6/tdts
x = units/2 - 64
print(units, x, bin(0x94), (64+20)*2*tdts*1e9)   # 168.0 20.0 0b10010100 1000.0
```

### 4.4 刹车与保护

高级定时器有 `BKIN` 刹车输入：外部硬件（过流比较器输出）触发时，**硬件**立即把 PWM 输出置为安全状态，不经过 CPU。这是电机驱动的硬件过流保护，响应在纳秒到微秒级，软件无法比拟。

---

## 5. ADC 与电流采样

### 5.1 ADC 基本量

12 位 ADC，参考电压 3.3 V：

$$ LSB=\frac{3.3}{4096}=0.806\ \text{mV},\qquad V=\text{code}\times LSB. $$

转换时间：$t_{conv}=\dfrac{t_{sample}+12}{f_{ADC}}$（12 位分辨率下的转换周期数为 12；采样周期数可配置，如 3、15、28、56、84、112、144、480 个 ADC 时钟）。

**手算**：$f_{ADC}=21$ MHz（教学示例），采样 15 周期：$t_{conv}=(15+12)/21\times10^6=1.286\ \mu s$。若一次要采 3 路电流（顺序转换），约 3.9 µs，占 50 µs 的 PWM 周期的 7.7%。

采样周期太短而信号源阻抗太高，采样电容充电不足，读数偏低；需要按 $R_{src}$ 与采样电容计算最小采样时间，或在运放后面直接采。

### 5.2 电流检测方式

| 方式 | 说明 | 特点 |
|---|---|---|
| 相线串联采样电阻 | 在电机相线上串联电阻，差分放大 | 采样不受开关状态影响，但共模电压是高速开关电压，要求高共模抑制的放大器 |
| 下桥臂（低侧）采样 | 采样电阻在每个低侧管与地之间 | 成本低；**只有低侧管导通时才能采**，所以采样时刻要在 PWM 的低侧导通期 |
| 母线单电阻 | 母线上一个电阻 | 最便宜，需要重构各相电流，复杂 |
| 霍尔/磁电流传感器 | 隔离测量 | 带宽与成本折中 |

典型低侧三电阻方案：电阻 $R_{sh}=5\ \text{m}\Omega$，放大器增益 $G=20$ V/V，输出偏置 1.65 V（使双向电流映射到 0–3.3 V）：

$$ V_{adc}=1.65+G R_{sh}\,i. $$

### 5.3 手算：码值转电流

读到 ADC 码值 2500：$V=2500\times0.806\ \text{mV}=2.014$ V。电流：

$$ i=\frac{V-1.65}{G R_{sh}}=\frac{2.014-1.65}{20\times0.005}=\frac{0.364}{0.1}=3.64\ \text{A}. $$

量程：$V$ 在 0–3.3 V，对应电流 $\pm1.65/0.1=\pm16.5$ A，分辨率 $0.806\ \text{mV}/0.1\ \text{V/A}=8.06$ mA/LSB。

**偏置（offset）校准**：电机不供电时（电流为 0）采集多次取平均，得到零电流时的码值 $c_0$。实际使用 $i=(c-c_0)\cdot LSB/(G R_{sh})$。不校准时偏置误差（如 15 个码值 ≈ 0.12 A）会产生**直流电流偏置**，经 Park 变换后变为转子角频率的力矩脉动，这是常见的"电机低速抖动"来源。

```python
lsb = 3.3/4096
V = 2500*lsb
i = (V-1.65)/(20*0.005)
print(V, i)          # 2.014  3.64
```

### 5.4 三相只需采两相

三相电流满足 $i_a+i_b+i_c=0$（星形或三角形无中线）。所以采 $i_a,i_b$ 两相，$i_c=-i_a-i_b$。三相都采可冗余检测，也可在占空比接近 100% 的相（低侧管几乎不导通、无法采样）选择另外两相计算。

---

## 6. 采样与 PWM 的同步

### 6.1 为什么必须同步

相电流有开关纹波。如果在随机时刻采样，每次采到的纹波相位不同，信号像叠加了噪声。而且开关瞬间（上下管切换时）有 dv/dt 引起的尖峰。采样要放在**电流纹波的平均值位置**：

- 中心对齐 PWM 下，在每相的低侧管开通期间的**中点**（计数器在 0，即下溢处）采样，这一刻的电流恰等于一个 PWM 周期内电流的平均值（对电感负载，三角纹波的中点）；
- 此时没有开关动作，噪声最小。

### 6.2 STM32 的实现

```text
TIM1 (中心对齐计数) :   计数器  /\    /\    /\
                              /  \  /  \  /  \
                       0 ----/    \/    \/    \----
                              ^         ^
                       更新事件/下溢     TRGO / CC4 触发 ADC
                       ADC 注入组被触发 -> 转换完成产生 JEOC 中断
```

具体设置：

1. 定时器设为中心对齐，选择 **TRGO（或 OC4REF）** 在计数器下溢（或上溢）时输出触发；
2. ADC 使用**注入组**（injected channel）外部触发，触发源选 TIM1 TRGO，这样 ADC 硬件自动在对的时刻开始转换，不受 CPU 中断延迟影响；
3. 注入转换结束产生中断（JEOC），在 ISR 里执行 FOC 算法（读码值、Clarke/Park、PI、反变换、更新 CCR）；
4. 更新的 CCR 在下一个周期生效，**所以电流环有 1.5 个 PWM 周期的延迟**（采样延迟 + 计算延迟 + PWM 更新延迟），这是电流环带宽取 PWM 频率 1/10 的原因之一。

### 6.3 采样时间窗口约束

低侧管开通宽度 $=(1-D)T_{pwm}$（对应占空比越高，低侧开通越短）。ADC 采样+保持时间必须落在其内，并避开死区后的振铃（典型 0.5–1 µs）。**手算**：$f_{pwm}=20$ kHz，$T=50\ \mu$s，$D_{max}=0.95$ 时低侧导通 2.5 µs，半宽 1.25 µs，必须小于 ADC 采样时间加振铃时间，因此需要限制最大占空比（如 0.95 以下），或采样时刻要动态移动。限制最大占空比等价于限制最大电压，这是高速电机的弱点。

---

## 7. Clarke 与 Park 变换

### 7.1 Clarke（三相 → 静止两相）

三相电流 $i_a,i_b,i_c$ 在空间上互差 120°。把它们投影到互相垂直的 $\alpha$ 轴（与 a 轴重合）和 $\beta$ 轴上（幅值不变约定）：

$$
\begin{bmatrix} i_\alpha\\ i_\beta\end{bmatrix}=
\frac23\begin{bmatrix}1&-\tfrac12&-\tfrac12\\0&\tfrac{\sqrt3}{2}&-\tfrac{\sqrt3}{2}\end{bmatrix}
\begin{bmatrix} i_a\\ i_b\\ i_c\end{bmatrix}.
$$

由 $i_a+i_b+i_c=0$ 化简（只需两相）：

$$ i_\alpha=i_a,\qquad i_\beta=\frac{i_a+2i_b}{\sqrt3}. $$

推导检查：$i_\beta=\tfrac23\cdot\tfrac{\sqrt3}{2}(i_b-i_c)=\tfrac{1}{\sqrt3}(i_b-i_c)$，代入 $i_c=-i_a-i_b$ 得 $\tfrac{1}{\sqrt3}(i_a+2i_b)$。

### 7.2 Park（静止 → 旋转）

以转子电角度 $\theta_e$ 旋转坐标系，d 轴沿磁极方向：

$$
\begin{bmatrix} i_d\\ i_q\end{bmatrix}=
\begin{bmatrix}\cos\theta_e&\sin\theta_e\\-\sin\theta_e&\cos\theta_e\end{bmatrix}
\begin{bmatrix} i_\alpha\\ i_\beta\end{bmatrix}.
$$

逆变换用转置：

$$
\begin{bmatrix} v_\alpha\\ v_\beta\end{bmatrix}=
\begin{bmatrix}\cos\theta_e&-\sin\theta_e\\ \sin\theta_e&\cos\theta_e\end{bmatrix}
\begin{bmatrix} v_d\\ v_q\end{bmatrix}.
$$

Park 是旋转矩阵（正交矩阵，逆等于转置）。

### 7.3 完整手算

设三相电流幅值 3 A，电流矢量电角度 $\delta=60^\circ$（相对 a 轴）：

- $i_a=3\cos60^\circ=1.5$；
- $i_b=3\cos(60^\circ-120^\circ)=3\cos(-60^\circ)=1.5$；
- $i_c=3\cos(60^\circ-240^\circ)=3\cos(-180^\circ)=-3$；
- 和为 $1.5+1.5-3=0$ ✓。

Clarke：$i_\alpha=1.5$，$i_\beta=(1.5+2\times1.5)/\sqrt3=4.5/1.732=2.598$。校验：幅值 $\sqrt{1.5^2+2.598^2}=\sqrt{2.25+6.75}=3$ ✓，角度 $\arctan(2.598/1.5)=60^\circ$ ✓。

转子电角度 $\theta_e=30^\circ$，Park：

- $i_d=1.5\cos30^\circ+2.598\sin30^\circ=1.299+1.299=2.598$；
- $i_q=-1.5\sin30^\circ+2.598\cos30^\circ=-0.75+2.25=1.5$。

解释：电流矢量相对 d 轴领先 $\delta-\theta_e=30^\circ$，所以 $i_d=3\cos30^\circ=2.598$，$i_q=3\sin30^\circ=1.5$ ✓。FOC 希望 $i_d\approx0$（表贴式 PMSM），电流全部给 q 轴；此例中电流矢量与 d 轴夹角 30°，说明角度没有对准，力矩只用了一部分。

转矩：设 $p=7$，$\psi_f=0.005$ Wb（教学示例），$k_t=\tfrac32 p\psi_f=0.0525$ N·m/A，$i_q=1.5$ A 时 $T=0.0788$ N·m。

```python
import math
s3 = math.sqrt(3)
ia, ib = 1.5, 1.5
al, be = ia, (ia + 2*ib)/s3
th = math.radians(30)
d = al*math.cos(th) + be*math.sin(th)
q = -al*math.sin(th) + be*math.cos(th)
print(al, be, d, q)             # 1.5 2.598 2.598 1.5
print(1.5*7*0.005, 1.5*7*0.005*1.5)   # 0.0525 0.07875
```

### 7.4 电角度与零位

$\theta_e=p\,\theta_m+\theta_{offset}$。$\theta_{offset}$ 是编码器零位与转子 d 轴对准位置的差，必须**对齐标定**：给 d 轴注入电流，让转子被吸到 d 轴位置，读取此时编码器值作为偏置。零位标错 $\Delta\theta$，则实际力矩只有 $\cos\Delta\theta$ 倍，且产生 d 轴电流。

---

## 8. dq 电流环与 PI 整定

### 8.1 dq 电机模型

在 dq 坐标系下（表贴式，$L_d=L_q=L$）：

$$
v_d=Ri_d+L\frac{di_d}{dt}-\omega_eLi_q,\qquad
v_q=Ri_q+L\frac{di_q}{dt}+\omega_eLi_d+\omega_e\psi_f.
$$

右侧的 $-\omega_e L i_q$、$+\omega_e L i_d$ 是 dq 轴间交叉耦合项，$\omega_e\psi_f$ 是反电动势。

### 8.2 解耦与前馈

控制器输出：

$$ v_d^*=u_d-\omega_eLi_q,\qquad v_q^*=u_q+\omega_eLi_d+\omega_e\psi_f, $$

其中 $u_d,u_q$ 是 PI 输出。前馈把耦合和反电动势抵消，剩下的被控对象是一阶系统：

$$ L\frac{di}{dt}+Ri=u\ \Rightarrow\ G(s)=\frac{1}{Ls+R}. $$

### 8.3 PI 整定（极点-零点抵消）

PI 控制器 $C(s)=K_p+\dfrac{K_i}{s}=K_p\dfrac{s+K_i/K_p}{s}$。选 $K_i/K_p=R/L$，则 PI 的零点抵消电机极点，开环 $=\dfrac{K_p}{Ls}$，闭环为一阶：

$$ \frac{i}{i^*}=\frac{K_p/L}{s+K_p/L}=\frac{\omega_c}{s+\omega_c}. $$

所以：

$$ K_p=\omega_c L,\qquad K_i=\omega_c R, $$

其中 $\omega_c$ 是期望闭环带宽（rad/s）。

**手算**：$L=0.5$ mH，$R=0.3\ \Omega$，目标带宽 1 kHz（$\omega_c=2\pi\times1000=6283$ rad/s）：

- $K_p=6283\times0.5\times10^{-3}=3.14$ V/A；
- $K_i=6283\times0.3=1885$ V/(A·s)；
- 电气时间常数 $\tau_e=L/R=1.67$ ms，对应电机自身极点 $1/\tau_e=600$ rad/s（95 Hz）；
- 离散实现：$u[k]=K_p e[k]+I[k]$，$I[k]=I[k-1]+K_iT_se[k]$，$T_s=50\ \mu$s，则 $K_iT_s=0.094$。

上限：带宽必须小于 PWM 频率的 1/10 左右，且要考虑 1.5 个周期的延迟（75 µs）带来的相位滞后：1 kHz 处 $360^\circ\times1000\times75\times10^{-6}=27^\circ$，还可接受；若取 3 kHz 则达 81°，已不稳定。

电压限幅：$v_d^2+v_q^2\le V_{max}^2$。限幅时要对 PI 积分做抗饱和（见 [前馈、增益调度、限幅与饱和](../05_控制理论/07_前馈_增益调度_限幅与饱和.md)），通常优先保证 $v_d$，剩下的给 $v_q$，或按比例缩放。

```python
import math
wc = 2*math.pi*1000
L, R = 0.5e-3, 0.3
print(wc*L, wc*R, L/R*1e3)        # 3.14  1885  1.667
print(360*1000*75e-6)              # 27 度
```

### 8.4 外环：速度环与位置环

电流环（最内环，~1 kHz）→ 速度环（~100–200 Hz）→ 位置环（~10–50 Hz）。每个外环带宽约为内环的 1/5–1/10，这就是[级联控制](../05_控制理论/02_PID控制全解.md)的原则。P3 的拨盘位置—速度双环在 CAN 外面，驱动器内部还有一圈电流环，所以实际是三层。

---

## 9. SVPWM 与占空比计算

### 9.1 目标

给定 $v_\alpha,v_\beta$（来自逆 Park），需要三相占空比 $d_a,d_b,d_c\in[0,1]$。最朴素是正弦 PWM（SPWM）：$d_x=\tfrac12+\tfrac{v_x}{V_{dc}}$，线性范围 $|v|\le V_{dc}/2$。**SVPWM（空间矢量 PWM）**把线性范围扩大到 $V_{dc}/\sqrt3$（提升约 15.5% 的母线利用率）。

### 9.2 最小-最大注入（等价于 SVPWM）

先算三相参考电压 $v_a,v_b,v_c$（逆 Clarke）：

$$ v_a=v_\alpha,\quad v_b=-\tfrac12v_\alpha+\tfrac{\sqrt3}{2}v_\beta,\quad v_c=-\tfrac12v_\alpha-\tfrac{\sqrt3}{2}v_\beta. $$

注入零序分量 $v_0=-\tfrac12(\max+\min)$，使三相在同一偏移下，占空比：

$$ d_x=\frac12+\frac{v_x-\tfrac12(\max(v_a,v_b,v_c)+\min(v_a,v_b,v_c))}{V_{dc}}. $$

零序分量不产生线电压（线电压是相间差），所以电机电流不变，但三相更"居中"，能用更大的输出。

线性调制极限：$|v|_{max}=V_{dc}/\sqrt3$。

### 9.3 手算

$V_{dc}=24$ V，线性上限 $24/\sqrt3=13.86$ V。取电压矢量幅值 10 V，角度 60°：

- $v_a=10\cos60^\circ=5$；$v_b=10\cos(-60^\circ)=5$；$v_c=10\cos(180^\circ)=-10$；
- 最大 5，最小 −10，$v_0=-\tfrac12(5-10)=2.5$，即偏移 $\tfrac12(\max+\min)=-2.5$，$v_x-(-2.5)=v_x+2.5$：7.5，7.5，−7.5；
- $d_a=0.5+7.5/24=0.8125$，$d_b=0.8125$，$d_c=0.5-7.5/24=0.1875$。

对比 SPWM：$d_a=0.5+5/24=0.708$，$d_c=0.5-10/24=0.083$。SVPWM 的占空比更居中（0.1875–0.8125 对 0.083–0.708），所以同样母线下还能输出更大电压。验证线电压不变：$d_a-d_c$ 乘 $V_{dc}$：SVPWM $(0.8125-0.1875)\times24=15$ V；SPWM $(0.708-0.083)\times24=15$ V ✓。

```python
import math
Vdc = 24
mag, ang = 10, math.radians(60)
va = mag*math.cos(ang)
vb = mag*math.cos(ang - 2*math.pi/3)
vc = mag*math.cos(ang + 2*math.pi/3)
off = (max(va, vb, vc) + min(va, vb, vc))/2
print([0.5 + (v-off)/Vdc for v in (va, vb, vc)])   # [0.8125, 0.8125, 0.1875]
print(Vdc/math.sqrt(3))                             # 13.856
```

---

## 10. 完整 FOC 流程与 CPU 负载

### 10.1 一个 PWM 周期内要做的事

```mermaid
flowchart LR
  A[ADC注入完成中断] --> B[读两相电流并去偏置]
  B --> C[Clarke变换]
  C --> D[读编码器得电角度]
  D --> E[Park变换得id iq]
  E --> F[PI电流环加解耦前馈]
  F --> G[电压限幅与抗饱和]
  G --> H[逆Park]
  H --> I[SVPWM占空比]
  I --> J[写CCR预装载]
```

外加：速度环/位置环在较慢的定时器中断里运行（如 1–5 kHz）；保护（过流、过温、欠压）；通信（CAN 命令解析）。

### 10.2 CPU 负载估算（教学示例）

PWM 20 kHz，周期 50 µs。Cortex-M4F @168 MHz，单周期乘加。FOC 运算（含 `sinf/cosf`，可用查表或 CORDIC）约 600–1200 个周期（教学估计，**必须实测**）：

- 800 周期 $/168$ MHz $=4.76\ \mu s$，占 9.5%；
- 加上中断进出（约 30 周期）和 ADC 读取：约 10%；
- 速度环和通信再占 10–20%。

结论：F4 完全可以单片机跑 FOC。实际工程里，MCU 专用于一个电机；而机器人多电机场景常把 FOC 放在**每个关节的驱动板**，由主控通过 CAN 发力矩/位置命令，这正是 P3（电调）和 P7（MIT 协议关节电机）的做法。

### 10.3 无传感器与有传感器

- **有传感器**：霍尔 / 绝对编码器 / 增量编码器给出 $\theta_e$；
- **无传感器**：用观测器（反电动势估计、滑模观测器等）估 $\theta_e$，低速性能差。

本章不展开观测器。

---

## 11. 完整手算汇总

| 量 | 公式 | 结果（教学示例） |
|---|---|---|
| ARR | $f_{tim}/(2f_{pwm})$ | 4200（168 MHz，20 kHz） |
| 分辨率 | $\log_2ARR$ | 12.04 位 |
| 死区寄存器 | 1 µs @ $t_{DTS}=5.95$ ns | DTG = 0x94（148） |
| ADC LSB | $3.3/4096$ | 0.806 mV |
| 电流 | $(V-1.65)/(GR_{sh})$ | 码值 2500 → 3.64 A |
| 转换时间 | $(15+12)/21$ MHz | 1.286 µs |
| Clarke/Park | | $i_d=2.598,\ i_q=1.5$ |
| $k_t$ | $\tfrac32p\psi_f$ | 0.0525 N·m/A |
| PI | $K_p=\omega_cL,\ K_i=\omega_cR$ | 3.14，1885 |
| 线性极限 | $V_{dc}/\sqrt3$ | 13.86 V（24 V） |

---

## 12. 在七个项目里怎么用

先说清边界：**CV 没有写你自己实现了 FOC。** 你的项目里电机电流环是驱动器内部实现的，你实现的是更上层的控制与通信。本章的价值是理解"你发出的命令最后是怎么变成力矩的"，并能回答面试官的深挖问题。

| 项目 | 关联点 | 性质 |
|---|---|---|
| P3 | 云台、摩擦轮、底盘用 GM6020/M3508，经 C620 等电调控制。主控 STM32F407 通过 CAN 发送电流/电压指令；电流环、换相在电调/电机内部完成（见 [P3 CAN 总线与电机反馈](../../01_项目课程/P3_RoboMaster英雄机器人嵌入式控制/04_CAN总线与电机反馈.md)）。所以 P3 的内环 PID 输出（如 ±30000）是电压/电流指令量，而不是 PWM 占空比 | 项目事实 |
| P7 | 关节与轮电机是 MIT 协议的一体化关节电机，命令是 $\tau=K_p(p^*-p)+K_d(v^*-v)+\tau_{ff}$ 类的力矩/阻抗帧，电机内部电流环把力矩变成 $i_q$（$i_q=\tau/k_t$）。我们的控制算法（LQR/VMC）输出关节力矩与轮力矩，限幅后经 FDCAN 发出（[P7 软件结构](../../01_项目课程/P7_可变腿长轮腿机器人/10_STM32H7_FreeRTOS_FDCAN软件结构.md)）。力矩限幅、热保护对应电流限幅 | 项目事实 + 本章原理 |
| P2 | 主从示教用 MIT 电机接口；差速底盘用 H7 固件通过 FDCAN 与轮驱动器通信，轮速 PID 使用编码器反馈（[P2 主从跟踪与 MIT 电机接口](../../01_项目课程/P2_未来动力双臂遥操作与差速底盘/07_主从跟踪与MIT电机接口.md)） | 项目事实 |
| P1 | CAN 电机执行；力矩饱和与"执行器饱和诊断"对应电流/电压限幅 | 项目事实 |
| P4 | Hi3861 用 PWM/GPIO 驱动继电器水泵（开关量），不涉及 FOC；PWM 概念同本章第 3 节 | 概念 |
| P5 / P6 | 上层导航与平台软件，不接触 PWM 层 | 不适用 |

> **诚实边界**：如果面试官问"你实现过 FOC 吗"，回答："Not myself. In my projects the motor drivers run the current loop internally, and I send torque or current commands over CAN. But I understand the loop structure and I can explain it." 如果你自学做过仿真或在开发板上实现过，则按实际情况补充。

---

## 13. 常见错误与故障现象

1. **死区过小**：上下管直通，驱动板发热、过流保护动作甚至炸管。调试时先降低母线电压并限流。
2. **ADC 不同步**：电流波形有周期性毛刺，电流环噪声大。
3. **电流偏置未校准**：低速时力矩脉动、电机抖动、转一圈出现一次周期性抖动。
4. **电角度零位错误**：电机发热大、力矩小、方向反了。
5. **极对数填错**：电角度与机械角度关系错，电机无法旋转或剧烈抖动。
6. **Park 用错电角度符号**：$i_d,i_q$ 混乱；正反转行为不对称。
7. **PI 积分饱和**：电压限幅时积分继续累积，退出饱和时过冲。
8. **PWM 周期内改 CCR 未用预装载**：占空比毛刺。
9. **ISR 里做太多事**：FOC 中断超时，下一次采样被推迟，电流环不稳。
10. **电源反灌**：电机急减速时能量回到母线抬高电压，需要制动电阻/吸收。

---

## 14. 英文面试问答

### Q1. Can you explain Field-Oriented Control in simple words?

**Answer.** In field-oriented control we want to control torque by controlling the motor current precisely. The three phase currents are sinusoidal, so we transform them to a frame that rotates with the rotor. First we use the Clarke transform to get two stationary components, then the Park transform with the rotor electrical angle to get d and q currents. In that frame they are almost DC. The q current makes torque, and the d current is normally kept at zero for a surface-mounted motor. We run two PI controllers on them, transform the output voltages back, and generate the PWM using space vector modulation. In my projects the motor drivers do this internally, and I send torque commands.

**中文译文**：FOC 的目标是通过精确控制电流来控制力矩。三相电流是正弦量，所以变换到随转子旋转的坐标系：先 Clarke 变换得到两个静止分量，再用转子电角度做 Park 变换得到 d、q 电流，此时它们近似直流。q 轴电流产生力矩，表贴式电机通常把 d 轴电流设为零。对两者各用一个 PI，输出电压再反变换，用空间矢量调制产生 PWM。我的项目里这些由电机驱动器内部完成，我发送力矩命令。

**Follow-up 1:** Why is dead time needed? — To prevent the high-side and low-side switches from conducting at the same time, which would short the bus. The turn-off is not instant, so we leave both off for a short time. （防止上下管同时导通短路母线；关断不是瞬时的，所以留一段都关断的时间。）

**Follow-up 2:** What is the cost of dead time? — It distorts the output voltage near current zero crossings and causes torque ripple, so it should be as short as safely possible and sometimes compensated. （电流过零附近输出电压畸变，造成力矩脉动，所以应尽量短并可补偿。）

### Q2. Why do you synchronise the ADC with the PWM?

**Answer.** Phase current has switching ripple and noise spikes at the switching instants. With center-aligned PWM, the point where the counter is at the bottom is the middle of the low-side on-time, where the ripple is at its average value and no switch is changing state. So I trigger the ADC from the timer, by hardware, at that moment. This gives a clean average current each cycle, and the trigger does not depend on interrupt latency. The price is that the sampling window is short when the duty cycle is high, so the maximum duty is limited.

**中文译文**：相电流有开关纹波，开关瞬间还有尖峰。中心对齐 PWM 下，计数器到底部的时刻是低侧导通期的中点，此时纹波等于平均值且没有开关动作。所以我用定时器硬件触发 ADC，在该时刻采样；这样每周期得到干净的平均电流，触发也不受中断延迟影响。代价是占空比高时采样窗口很短，需要限制最大占空比。

**Follow-up 1:** How many phases do you need to measure? — Two, because the three currents sum to zero. （两相，因为三相和为零。）

**Follow-up 2:** What happens if the current offset is not calibrated? — A DC error in the phase current becomes torque ripple at the rotor frequency after the Park transform. （相电流的直流误差经 Park 变换变成转子频率的力矩脉动。）

### Q3. How do you tune the current loop PI?

**Answer.** The plant in the dq frame is a first-order system with resistance and inductance, after the cross-coupling and back-EMF are compensated by feedforward. I choose the integral gain over proportional gain equal to R over L, so the PI zero cancels the motor pole. Then the closed loop is a first-order system with bandwidth omega c. So Kp is omega c times L and Ki is omega c times R. For example, with 0.5 millihenry, 0.3 ohm, and 1 kilohertz bandwidth, Kp is about 3.1 volts per amp and Ki is about 1900. The bandwidth must stay well below the PWM frequency because of the sampling and update delay.

**中文译文**：在前馈补偿了交叉耦合与反电动势后，dq 轴被控对象是含电阻和电感的一阶系统。我取 Ki/Kp = R/L，让 PI 零点抵消电机极点，闭环就是带宽为 ωc 的一阶系统，所以 Kp = ωc·L，Ki = ωc·R。例如 0.5 mH、0.3 Ω、带宽 1 kHz，Kp 约 3.1 V/A，Ki 约 1900。因为有采样和更新延迟，带宽必须远低于 PWM 频率。

**Follow-up 1:** Why not set the bandwidth at the PWM frequency? — The loop has about one and a half PWM periods of delay, which adds a lot of phase lag at high frequency and makes the loop unstable. （环路有约 1.5 个 PWM 周期的延迟，高频相位滞后大，会失稳。）

**Follow-up 2:** What do you do when the voltage saturates? — Limit the voltage vector and use anti-windup on the integrators. （限制电压矢量并对积分做抗饱和。）

### Q4. In your robot, who runs the current loop?

**Answer.** In the RoboMaster robot, the motor controllers on each motor run the field-oriented control and the current loop. My STM32 sends current or voltage commands over CAN, and my loops are the gimbal, launcher and chassis loops around them. In the wheel-legged robot, the joint motors use the MIT-style protocol: I send a torque feedforward plus optional position and velocity terms, and the motor converts torque to q-axis current internally. So my responsibility is to compute torques, limit them, and make sure the data arrives on time. I understand the inner loop, but I did not implement it.

**中文译文**：RoboMaster 项目里每个电机上的驱动器运行 FOC 和电流环；我的 STM32 通过 CAN 发电流或电压指令，我负责的是围绕它们的云台、发射和底盘回路。轮腿机器人的关节电机用 MIT 风格协议：我发送力矩前馈加可选的位置速度项，电机内部把力矩换算成 q 轴电流。所以我的职责是计算力矩、限幅并保证数据按时到达。我理解内环，但没有自己实现它。

**Follow-up 1:** How do you convert desired torque to current? — With the torque constant: i_q equals torque divided by k_t, subject to the current limit. （q 轴电流 = 力矩 / k_t，并受电流限幅。）

**Follow-up 2:** What happens when torque is limited in your LQR? — The effective gain drops, and the balance may fail; I handled this by ramping speed commands and reducing weights. （有效增益下降，平衡可能失败；通过速度指令斜坡与降低权重处理。）

---

## 15. 要点速记、自测题、相关链接

### 要点速记

1. 中心对齐 PWM：$f=f_{tim}/(2\cdot ARR)$（PSC=0），频率与分辨率互相制约。
2. 死区防直通：$t_{dead}\ge t_{off,max}-t_{on,min}+t_{margin}$；STM32 用 DTG 分档编码。
3. 电流采样放在低侧导通中点，用定时器 TRGO 硬件触发 ADC 注入组。
4. 电流 $i=(V-1.65)/(GR_{sh})$；偏置必须校准。
5. 三相和为零，只采两相。
6. Clarke：$i_\alpha=i_a,\ i_\beta=(i_a+2i_b)/\sqrt3$；Park 是旋转。
7. $T=\tfrac32p\psi_fi_q$，d 轴电流常设为零。
8. PI 整定：$K_p=\omega_cL$，$K_i=\omega_cR$；带宽取 PWM 频率的 1/10 左右。
9. SVPWM 线性范围 $V_{dc}/\sqrt3$，比 SPWM 高约 15.5%。
10. 本仓库项目里电流环在驱动器内，你发力矩/电流指令。

### 自测题

1. TIM 时钟 84 MHz，中心对齐，要 16 kHz PWM，ARR 应为多少？
2. 为什么中心对齐 PWM 更适合同步电流采样？
3. 已知 $i_a=2,\ i_b=-1$，求 $i_c,\ i_\alpha,\ i_\beta$。
4. 若电机 $L=1$ mH，$R=0.5\ \Omega$，目标带宽 500 Hz，求 PI 参数。
5. 为什么 SVPWM 的最大线性电压是 $V_{dc}/\sqrt3$？（用线电压说明）

**参考答案要点**：1. $84\times10^6/(2\times16000)=2625$。2. 所有相的低侧导通中点重合于计数器下溢，采样点无开关动作。3. $i_c=-1$；$i_\alpha=2$；$i_\beta=(2-2)/\sqrt3=0$。4. $\omega_c=3142$；$K_p=3.14$，$K_i=1571$。5. 相电压矢量最大圆内切于六边形，六边形内切圆半径为 $V_{dc}/\sqrt3$（线电压最大 $V_{dc}$，对应相电压幅值 $V_{dc}/\sqrt3$）。

### 相关链接

- 上一章：[电源管理、低功耗、看门狗、Bootloader 与 OTA](./08_电源管理_低功耗_看门狗_Bootloader_OTA.md)
- 下一章：[嵌入式调试](./10_嵌入式调试_JTAG_SWD_逻辑分析仪_故障定位.md)
- 相关：[电机、编码器、驱动与力矩控制](./05_电机_编码器_驱动与力矩控制.md)、[CAN 与 FDCAN](./03_CAN_FDCAN与电机通信.md)、[数字控制与实时性](../05_控制理论/08_数字控制与实时性.md)
- 八股：[嵌入式与机电八股](../../03_八股知识/03_机器人学/06_嵌入式与机电八股.md)
- 项目：[P3](../../01_项目课程/P3_RoboMaster英雄机器人嵌入式控制/README.md)、[P7](../../01_项目课程/P7_可变腿长轮腿机器人/README.md)、[P2](../../01_项目课程/P2_未来动力双臂遥操作与差速底盘/README.md)
