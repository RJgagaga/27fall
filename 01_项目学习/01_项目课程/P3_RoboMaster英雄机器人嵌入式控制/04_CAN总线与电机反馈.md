# 04 CAN 总线与电机反馈

> **一句话定位**：电机是闭环里的"手"，CAN 是手和大脑之间的"神经"。这一章把 CAN 帧、ID、电机反馈量、电流指令、多圈角度处理、总线负载、离线检测讲到可以手算、可以白板推导的程度。

## 本章学完你能……

- 说清 CAN 的物理层、帧格式、仲裁、位填充、错误处理与总线关闭（bus-off）；
- 手算一条 CAN 总线的负载，并据此解释为什么电机要分到两条总线上；
- 写出（或口述）STM32 bxCAN 的初始化、过滤器、接收回调、发送流程，并解释 3 个发送邮箱的含义；
- 解析典型 RoboMaster 电机反馈帧（角度、转速、转矩电流、温度）和电流指令帧，完成 rpm → rad/s、指令值 → 安培 → 力矩的换算；
- 完整解释编码器**多圈（multi-turn）累计**算法、它的适用条件，以及丢帧时怎么失效；
- 设计电机离线检测与总线异常处理；
- 回答"CAN 帧格式""CAN 为什么抗干扰""总线负载怎么算""多圈角度怎么处理"等高频追问。

## 前置知识

- [CAN / FDCAN 与电机通信（基础）](../../02_基础知识/08_嵌入式与通信/03_CAN_FDCAN与电机通信.md)
- [电机、编码器、驱动与力矩控制（基础）](../../02_基础知识/08_嵌入式与通信/05_电机_编码器_驱动与力矩控制.md)
- 上一章：[03 FreeRTOS 任务设计与时序](./03_FreeRTOS任务设计与时序.md)

## 目录

1. 为什么是 CAN
2. 物理层
3. 帧格式（标准数据帧）
4. 仲裁、位填充、错误检测与总线关闭
5. 位时序与波特率：手算
6. 总线负载：手算
7. STM32F407 bxCAN 外设与 HAL 用法
8. 典型 RoboMaster 电机的协议
9. 反馈量的物理换算（手算）
10. 编码器多圈累计
11. 云台关节角：零位、相对角与绕回
12. 电流指令与力矩
13. 离线检测与异常处理
14. 在本项目里怎么用
15. 常见故障
16. 面试怎么问、怎么答
17. 要点速记、自测题

---

## 1. 为什么是 CAN

机器人上要接多个电机、多个传感器节点，并且在大电流、强电磁干扰环境下工作。CAN（Controller Area Network）的特点正好匹配：

| 特性 | 含义 | 对机器人的好处 |
|---|---|---|
| 差分信号 | CAN_H / CAN_L 两线电压差表示逻辑 | 共模干扰被抵消，抗干扰强 |
| 多主广播 | 任一节点想发就发，所有节点都能收 | 反馈帧一发多收，布线简单 |
| 非破坏性仲裁 | ID 小的优先，输的节点自动退让 | 重要消息低延迟，不丢数据 |
| 硬件 CRC + 错误帧 | 通信错误硬件自动检测并重发 | 可靠 |
| 定长短帧（≤8 字节） | 延迟可预测 | 适合实时控制 |

CAN 的代价：速率有限（经典 CAN 最高 1 Mbps），不适合传大数据（所以视觉图像走 USB）。

> **与 P7 的区别**：P7 用 FDCAN（CAN FD），数据段可变速率、每帧最多 64 字节。P3 的 bxCAN 是经典 CAN（CAN 2.0）。

## 2. 物理层

- **两线**：CAN_H、CAN_L，总线两端各接一个 **120 Ω** 终端电阻；
- **显性/隐性**：显性（dominant，逻辑 0）：H≈3.5 V、L≈1.5 V；隐性（recessive，逻辑 1）：H≈L≈2.5 V；显性压倒隐性，这是"线与"，也是仲裁的基础；
- **收发器**：MCU 的 CAN 控制器输出 TX/RX 逻辑电平，经收发器芯片转成差分；
- **测量终端电阻**：断电后在 H 与 L 之间量应约 60 Ω（两个 120 Ω 并联）。若只有 120 Ω，说明只有一个终端电阻；若 ∞，说明没接。

> **典型故障**：终端电阻缺失或多余、H/L 接反、没有共地、线太长或分支太长，会造成偶发错误帧、丢包、电机间歇性掉线。

## 3. 帧格式（标准数据帧）

经典 CAN 标准帧（11 位 ID）：

```text
| SOF | ID(11) | RTR | IDE | r0 | DLC(4) | DATA(0~8B) | CRC(15) | CRC界定 | ACK槽 | ACK界定 | EOF(7) | IFS(3) |
| 1   |  11    |  1  |  1  | 1  |   4    |   64 位    |   15    |    1    |   1   |    1    |   7    |   3    |
```

字段含义：

- **SOF**：帧起始位（显性）；
- **ID**：标识符，既是消息类型标识，也是**优先级**（数值越小优先级越高）；
- **RTR**：远程帧请求位（数据帧为 0）；
- **IDE**：扩展帧标志（标准帧为 0）；
- **DLC**：数据长度码 0～8；
- **DATA**：0～8 字节数据；
- **CRC**：15 位循环冗余校验；
- **ACK 槽**：发送方发隐性，接收方只要正确收到就拉成显性——因此发送方知道"至少有一个节点收到了"；
- **EOF**：7 位隐性；**IFS**：帧间隔 3 位。

### 3.1 帧长度计算

8 字节数据的标准数据帧：SOF 1 + ID 11 + RTR 1 + IDE 1 + r0 1 + DLC 4 = 19 位；数据 64 位；CRC 15 + 界定符 1 + ACK 2 + EOF 7 = 25 位；合计 $19+64+25=108$ 位，再加帧间隔 3 位 = **111 位**。这是不含位填充的名义值。

## 4. 仲裁、位填充、错误检测与总线关闭

### 4.1 仲裁（Arbitration）

两个节点同时发送时，从 ID 最高位开始逐位比较。显性(0)覆盖隐性(1)。发隐性但看到总线是显性的节点，知道有更高优先级的在发，**立即退出发送并转为接收**，没有数据丢失，也没有时间浪费。

**手算（教学示例）**：节点 A 发 ID=0x200（二进制 010 0000 0000），节点 B 发 ID=0x1FF（001 1111 1111）。第 1 位都是 0；第 2 位 A 发 1，B 发 0；总线显性（0）；A 发隐性却读到显性，A 输。故 0x1FF 优先于 0x200。

**意义**：ID 即优先级。设计 ID 时，把紧急或关键的消息（如急停）设为小 ID。

### 4.2 位填充（Bit Stuffing）

为了保证有足够的电平跳变用于同步，发送方在连续 5 个相同电平后插入 1 个相反电平的"填充位"，接收方自动删除。适用范围：从 SOF 到 CRC 序列。

**最坏情况**：从 SOF 到 CRC 共 98 位（19+64+15），最坏每 4 位多插 1 位 → 约 $\lfloor 97/4\rfloor = 24$ 个填充位 → 帧长最多 $111+24=135$ 位。

### 4.3 错误检测

五种错误：位错误、填充错误、CRC 错误、格式错误、ACK 错误。检测到错误的节点发送**错误帧**，所有节点丢弃当前帧，发送方自动重发（可配置）。

### 4.4 错误计数与总线关闭

每个节点有发送错误计数 TEC、接收错误计数 REC：

| 状态 | 条件 | 行为 |
|---|---|---|
| 错误主动 Error Active | TEC、REC ≤ 127 | 正常，发主动错误帧 |
| 错误被动 Error Passive | TEC 或 REC > 127 | 发被动错误帧，发送后需额外等待 |
| 总线关闭 Bus-Off | TEC > 255 | 节点自行脱离总线，不再发送，需恢复序列 |

**典型现象**：只有一个节点在总线上（其他节点都没上电/没接好）时，它发送的帧得不到 ACK，产生 ACK 错误，TEC 不断增加 → 进入 error passive → 可能 bus-off。bxCAN 有"自动离线管理 ABOM"位，打开后在 bus-off 后自动恢复。

## 5. 位时序与波特率：手算

一个位被分成若干时间量子（time quantum, tq）：

```text
|SYNC(1tq)|     BS1 (PROP+PHASE1)      | BS2 (PHASE2) |
                                   ↑ 采样点
```

$$
t_q = \frac{\text{Prescaler}}{f_{\text{CAN}}},\qquad
\text{位时间} = (1 + \text{BS1} + \text{BS2})\, t_q,\qquad
\text{采样点} = \frac{1+\text{BS1}}{1+\text{BS1}+\text{BS2}}
$$

**教学示例**：STM32F407 的 CAN 挂在 APB1，常见 $f_{\text{CAN}}=42$ MHz。目标 1 Mbps：

- Prescaler = 3 → $t_q = 3/42\text{ MHz} = 71.43$ ns；
- 每位需 $1\ \mu s / 71.43\ ns = 14$ tq；
- 选 BS1 = 10，BS2 = 3：$1+10+3=14$ ✓；采样点 $= 11/14 = 78.6\%$。
- 也可选 BS1=9，BS2=4：采样点 $10/14 = 71.4\%$。

采样点一般取 75%～87.5%，总线越长越要权衡。**所有节点的波特率必须一致**，采样点也应相近。【待你补全：你的波特率与 BS1/BS2 配置】

## 6. 总线负载：手算

总线负载 = 每毫秒内所有帧占用的时间 / 1 ms。

### 6.1 单帧时间

1 Mbps 下 1 位 = 1 µs。名义 111 位 = **111 µs**；最坏 135 µs。

### 6.2 DJI 类电机的报文量（典型）

- 每个电机每 1 ms 发 1 帧反馈（8 字节）；
- 每组最多 4 个电机共用 1 帧命令（8 字节 = 4 × int16）。

**教学示例 1：底盘 4 个电机一条总线，控制周期 1 ms**

- 反馈：4 帧 × 111 µs = 444 µs；
- 命令：1 帧 × 111 µs = 111 µs；
- 合计 555 µs / 1000 µs = **55.5%**；最坏填充：$5\times135=675$ µs → 67.5%。

**教学示例 2：8 个电机一条总线**

- 反馈：8 × 111 = 888 µs；命令：2 × 111 = 222 µs；合计 1110 µs > 1000 µs → **负载 111%，不可行**。

所以必须把电机分到两条总线（CAN1/CAN2）。经验上总线负载希望控制在 50%～70% 以内，以保留给重发和突发。

> **面试要点**：能现场做这个算术，回答 "Why two CAN buses?" 就非常有说服力。你的实际分配：【待你补全：每条总线挂几个电机、实测负载或估算】。

### 6.3 控制周期与命令帧频率

命令帧发送频率可以低于反馈频率，但**太低会给控制回路增加延迟**。命令每 2 ms 发一次，则平均延迟多 1 ms。

## 7. STM32F407 bxCAN 外设与 HAL 用法

### 7.1 资源

- **2 个 bxCAN 控制器**（CAN1 为主、CAN2 为从；从控制器没有独立的过滤器，需要通过 CAN1 的过滤器组分配）；
- **3 个发送邮箱**（TX mailbox），按 ID 优先级或先进先出发送（`TransmitFifoPriority` 配置）；
- **2 个接收 FIFO**（RX FIFO0/1），每个 3 级深度；
- **过滤器组**（共 28 组，在 CAN1/CAN2 间划分；`SlaveStartFilterBank` 指定从哪组开始属于 CAN2）。

### 7.2 过滤器（Filter）

目的：只让关心的 ID 进入 FIFO，减少 CPU 负担。**掩码模式**：$(\text{ID}_{\text{rx}} \,\&\, \text{Mask}) == (\text{FilterID} \,\&\, \text{Mask})$ 才接收。

**教学示例**：想接收 0x201～0x208（电机反馈 ID）。0x201 = 0010 0000 0001，0x208 = 0010 0000 1000。高 8 位 0x20（0010 0000）固定，低 3 位变化范围 1～8（需要 4 位）。取掩码 0x7F0 会接收 0x200～0x20F（含多余 ID，如 0x200 与 0x209～0x20F），可接受。32 位尺度下，标准 ID 位于 [31:21]，所以：

```c
CAN_FilterTypeDef f;
f.FilterBank           = 0;
f.FilterMode           = CAN_FILTERMODE_IDMASK;
f.FilterScale          = CAN_FILTERSCALE_32BIT;
f.FilterIdHigh         = 0x200 << 5;      /* 标准 ID 左移 5 位放入高 16 位 */
f.FilterIdLow          = 0;
f.FilterMaskIdHigh     = 0x7F0 << 5;
f.FilterMaskIdLow      = 0;
f.FilterFIFOAssignment = CAN_RX_FIFO0;
f.FilterActivation     = ENABLE;
f.SlaveStartFilterBank = 14;              /* CAN2 的过滤器组从 14 开始 */
HAL_CAN_ConfigFilter(&hcan1, &f);
```

> 教学示例代码，非项目源码。许多团队为了省事把过滤器设成"全接收"（掩码为 0）。这在总线上节点少时可行；节点多、帧多时 CPU 中断压力大。

### 7.3 初始化与接收

```c
HAL_CAN_Start(&hcan1);
HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx;
    uint8_t d[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx, d);
    if (hcan == &hcan1) {
        switch (rx.StdId) {
            case 0x201: case 0x202: case 0x203: case 0x204:
                decode_motor(&chassis_motor[rx.StdId - 0x201], d);
                break;
            /* ... */
        }
    }
}
```

【笔记线索】你的笔记指出：快照里就是在 CAN 接收回调（中断上下文）里按 ID 分发并解码电机反馈。这样处理的优点是延迟最小；缺点是中断时间变长（要控制解码量，不做浮点重运算）。

### 7.4 发送

```c
CAN_TxHeaderTypeDef tx = {.StdId = 0x200, .IDE = CAN_ID_STD,
                          .RTR = CAN_RTR_DATA, .DLC = 8};
uint32_t mailbox;
uint8_t d[8] = { /* 4 个 int16，大端 */ };
if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0)
    HAL_CAN_AddTxMessage(&hcan1, &tx, d, &mailbox);
```

**3 个邮箱**：如果在极短时间内连发多于 3 帧，邮箱占满，`HAL_CAN_AddTxMessage` 返回错误（帧丢失）。这就是为何不少代码在发送之间加延时；更好的做法见 03 章 4.3 节。

## 8. 典型 RoboMaster 电机的协议

> **【典型】** 以下是 RoboMaster 常用电机（M3508 + C620、M2006 + C610、GM6020）公开手册层面的通用协议。**具体数值以电机说明书为准**；你的机器人用了哪几种电机，【待你补全】。

### 8.1 M3508（C620 电调）/ M2006（C610 电调）

- **反馈帧 ID** = 0x200 + 电调 ID（ID 1～8 → 0x201～0x208）；
- **反馈帧数据（8 字节，大端）**：

| 字节 | 内容 | 范围/单位 |
|---|---|---|
| 0–1 | 转子机械角度（单圈编码器） | 0～8191，对应 0～360° |
| 2–3 | 转子转速 | int16，单位 rpm |
| 4–5 | 实际转矩电流 | int16，协议单位 |
| 6 | 电机温度 | °C |
| 7 | 空 | — |

- **命令帧 ID**：控制 ID 1～4 的电机用 0x200；控制 ID 5～8 的电机用 0x1FF；
- **命令数据**：4 个 int16（大端）依次对应该组的 4 个电机，范围 M3508：−16384～+16384 对应 −20 A～+20 A；M2006：−10000～+10000 对应 −10 A～+10 A。
- **减速比**：M3508 P19 约 3591/187 ≈ **19.2 : 1**；M2006 P36 为 36 : 1。**反馈的角度和转速是转子侧**（减速器之前），不是输出轴。

### 8.2 GM6020（云台电机，直驱）

- 反馈帧 ID = 0x204 + 电机 ID（ID 1～7 → 0x205～0x20B），反馈数据格式与上类似；
- 命令：电压模式与电流模式；不同模式、不同 ID 段使用不同的命令帧 ID（如 0x1FF/0x2FF 对应电压，0x1FE/0x2FE 对应电流），范围和单位依固件版本不同。**务必查手册**；
- 直驱（无减速），转子角度 = 输出角度。

### 8.3 解码代码

```c
typedef struct {
    uint16_t ecd;          /* 本次单圈编码值 0~8191 */
    uint16_t last_ecd;
    int16_t  speed_rpm;
    int16_t  torque_cur;   /* 反馈电流（协议值），注意与"命令电流"区分 */
    uint8_t  temp;
    int32_t  round_cnt;
    int32_t  total_ecd;    /* 多圈累计编码 */
    uint32_t last_rx_ms;   /* 最近一次收到时间 */
} motor_t;

static void decode_motor(motor_t *m, const uint8_t *d)
{
    m->last_ecd   = m->ecd;
    m->ecd        = (uint16_t)((d[0] << 8) | d[1]);
    m->speed_rpm  = (int16_t)((d[2] << 8) | d[3]);
    m->torque_cur = (int16_t)((d[4] << 8) | d[5]);
    m->temp       = d[6];
    m->last_rx_ms = HAL_GetTick();
    update_round(m);
}
```

**要点**：高字节在前（大端）；`speed_rpm` 和 `torque_cur` 是**有符号 int16**，要强制类型转换，否则负速度会变成大正数。

> **【笔记线索】**：你的笔记强调了一个易混点——结构里的 `given_current`（反馈）与 `give_current`（命令）名字相近但含义相反。这类命名坑在面试里可以主动提："I separate feedback fields from command fields in naming to avoid mixing them up."

## 9. 反馈量的物理换算（手算）

### 9.1 rpm → rad/s

$$
\omega\ [\text{rad/s}] = \text{rpm} \times \frac{2\pi}{60}
$$

**手算**：600 rpm → $600 \times 0.10472 = 62.83$ rad/s。

### 9.2 转子 → 输出轴

M3508 P19（$i=19.2032$）：转子 600 rpm → 输出轴 $600/19.2032 = 31.24$ rpm = 3.272 rad/s。

### 9.3 轮速 → 车速

轮半径 $r$（教学示例：麦轮直径 152 mm，$r=0.076$ m）。轮速 $\omega_w$（rad/s）→ 轮缘线速度 $v = r \omega_w$。

**手算**：车速 2 m/s → $\omega_w = 2/0.076 = 26.32$ rad/s = 251.3 rpm（轮）→ 转子侧 $251.3\times19.2032 = 4826$ rpm。这在 M3508 常见的限速范围内（几千 rpm）。

### 9.4 编码值 → 角度

$$
\theta\ [\text{deg}] = \text{ecd} \times \frac{360}{8192}\quad(\approx 0.0439^\circ/\text{count})
$$

单圈分辨率 8192（13 位），对转子侧 0.044°。对 19:1 减速后输出轴分辨率 $0.044/19.2 = 0.0023^\circ$。

## 10. 编码器多圈累计

### 10.1 问题

电机只报告**单圈**位置 0～8191。转过一圈回到 0。如果要知道"总共转了多少"（拨盘要知道转了几个 60°，底盘要做里程计），必须在软件里**数圈**。

### 10.2 算法（半圈阈值法）

每次收到新值 `ecd`，与上次 `last_ecd` 比较：

```c
static void update_round(motor_t *m)
{
    int32_t diff = (int32_t)m->ecd - (int32_t)m->last_ecd;
    if (diff > 4096)        m->round_cnt--;   /* 实际是反向越过 0：8191->0 方向反了 */
    else if (diff < -4096)  m->round_cnt++;   /* 正向越过一圈 */
    m->total_ecd = m->round_cnt * 8192 + (int32_t)m->ecd - m->offset_ecd;
}
```

**假设**：相邻两次采样之间，转子转动**不超过半圈**（4096 个计数）。在这个假设下，"差值的绝对值 > 4096"只能是越过了 0 点，而不是真的转了那么多。

### 10.3 手算

**正向越过**：`last_ecd = 8100`，`ecd = 100`。直接差 $100-8100=-8000 < -4096$ → `round_cnt++`。真实位移：$8192-8100+100 = +192$ 计数（正转）。累计值：$\text{total}_{\text{新}} = (r+1)\cdot8192 + 100$，与旧值 $r\cdot8192+8100$ 之差 $= 8192+100-8100 = 192$ ✓。

**反向越过**：`last_ecd = 100`，`ecd = 8100`。差 $+8000>4096$ → `round_cnt--`。真实位移 $-192$ 计数。

**正常情况**：`last=3000`，`ecd=3050`：差 50，不触发。

### 10.4 什么时候会失效（面试重点）

假设转子侧转速 $n$ rpm，相邻两帧间隔 $\Delta t$。每帧转过的计数：

$$
N = 8192 \times \frac{n}{60}\times \Delta t
$$

**手算（教学示例）**：$n=8000$ rpm，$\Delta t=1$ ms：$N = 8192\times 133.3\times 0.001 = 1092$ 计数 < 4096，安全。若连续丢 3 帧，间隔 4 ms：$N=4369>4096$ → **混叠**，圈数计错，累计角度跳变 ±8192。因此：

- 高速且可能丢帧时（8000 rpm 附近），3～4 帧丢失就会计错；
- 实用对策：限制最大转速、检测帧间隔（时间戳）> 阈值时标记"多圈累计不可信"并重新对准、限制电流/速度，以及在启动时用绝对零位对齐。

### 10.5 偏移（零位）

`offset_ecd` 是定义"零位"的编码值：上电时取当前 `ecd`（上电位置为 0），或在机械零点标定后写死。云台关节通常用**机械零位标定值**（保证重新上电后零位一致）。

**拨盘/发射机构**：不需要绝对零位，上电时把 `total_ecd` 当前值当作参考，之后每发给一个相对增量。

### 10.6 数据类型

`total_ecd` 用 `int32_t`。8192 × 圈数：输出轴转 100 圈、减速比 19.2 → 转子 1920 圈 → 15.7M 计数，仍在 `int32` 范围（±2.1G）内。

## 11. 云台关节角：零位、相对角与绕回

### 11.1 直驱电机的相对角

yaw 轴用直驱电机（如 GM6020），机械零位对应编码值 `center_ecd`。相对角（编码值差）需要绕回到 $[-4096, 4095]$：

```c
int16_t rel = (int16_t)(ecd - center_ecd);
if (rel >  4096) rel -= 8192;
if (rel < -4096) rel += 8192;
float angle_rad = rel * (2.0f * 3.14159265f / 8192.0f);
```

**手算**：`ecd = 100`，`center_ecd = 8000`：`rel = -7900` → 小于 −4096 → +8192 = 292；$292/8192\times360^\circ = 12.83^\circ$。意味着从 8000 到 100 沿正方向越过了 0 点，只走了 292 个计数。

### 11.2 为什么需要相对角

- **底盘跟随云台**：底盘需要知道"云台 yaw 相对于底盘转了多少"，才能让底盘转向云台方向；
- **云台指令坐标变换**：操作手推杆"前进"应沿枪口方向，需要把云台系速度指令旋转到底盘系，旋转角 = 云台相对底盘 yaw。

详见 09 底盘：麦克纳姆运动学与跟随（未单独成章）。

### 11.3 与 IMU 角度的区别

电机编码器给的是**相对于机械结构**的角度（云台相对底盘），IMU 给的是**相对于惯性系**的角度。底盘转动时二者的变化不同。详见 05、06 章。

## 12. 电流指令与力矩

### 12.1 指令值 → 电流 → 力矩（M3508 + C620 的例子）

- 指令范围 ±16384 ↔ ±20 A：1 个指令单位 = $20/16384 = 1.2207$ mA；
- M3508 P19 手册中常用的转矩常数约 **0.3 N·m/A**，指的是**减速器输出轴**侧（对应转子侧约 $0.3/19.2\approx0.0156$ N·m/A）。**以官方说明书为准**。

**手算**：指令 8000 → 电流 $8000\times20/16384 = 9.766$ A → 输出轴力矩约 $0.3\times9.766 = 2.93$ N·m（理想，未计损耗、温度、饱和）。

**用途**：
1. 判断"这个电流指令对应多少力矩"，用于整定、功率估算（P ≈ τω + R I²）和堵转保护；
2. 限幅：给每类电机设**电流上限**（比协议最大值保守得多）以保护机构和电机。

### 12.2 命令的符号与方向

电机正方向由安装和接线决定。常见做法：**算法输出统一用"正方向 = 期望方向"，在驱动层/调用处乘 ±1 修正电机实际方向**。这使算法通用、方向问题集中在一处。

> **【笔记线索】**：你的笔记里，摩擦轮滑模控制器内部输出取负、调用处又取一次负，pitch 的输出额外取负——都是方向约定的体现。详见 08 章。

### 12.3 电流环在谁那里？

电调（C620 等）内部有**电流环**（FOC），MCU 发送的是"期望转矩电流"。MCU 上的 PID 是**速度/位置环**，输出电流指令。所以云台"角度环→角速度环→电流"的最内层电流闭环由电调完成，MCU 控制的是它外面的两层。这是一个常被忽略但面试官喜欢确认的细节："Where is the current loop?" 答案："Inside the motor driver."

## 13. 离线检测与异常处理

### 13.1 时间戳法

每个电机结构体记录 `last_rx_ms`；检测任务周期性地检查：

```c
bool motor_online(const motor_t *m, uint32_t now_ms, uint32_t timeout_ms)
{
    return (now_ms - m->last_rx_ms) < timeout_ms;
}
```

**阈值选择（教学示例）**：反馈周期 1 ms，阈值取 20～50 ms（容忍短暂丢帧，又足够快检测掉线）。【待你补全：你的离线阈值与检测周期】

### 13.2 检测到离线后做什么

| 离线对象 | 典型动作 |
|---|---|
| 云台 yaw/pitch 电机 | 云台进入 RELAX；发零电流 |
| 摩擦轮电机 | 停止发射，摩擦轮目标置零或放松 |
| 拨盘电机 | 停止送弹，状态置失败 |
| 底盘任一电机 | 底盘停车（避免只有部分轮子出力导致跑偏） |
| 板间 CAN | 另一块板进入保守模式 |

### 13.3 总线级异常

- 检查 `HAL_CAN_ErrorCallback` 与错误码；
- 开启 ABOM，使 bus-off 后自动恢复；
- 恢复后重新初始化电机状态（`total_ecd` 参考点可能失效）。

## 14. 在本项目里怎么用

- **【CV】**：*STM32F407/FreeRTOS and CAN*；*IMU/AHRS and motor feedback*；*friction-wheel speed loops and feeder position–velocity dual loop*；*allocated body velocity to four Mecanum wheels*。这些回路的反馈全部来自本章讲的电机 CAN 反馈。
- **【笔记线索】**：电机反馈在 CAN 接收中断中分发并解码；解码得到 `ecd`、`speed_rpm`、`given_current`、温度，有 `DJI_Round_Count` 一类的多圈处理（笔记示例 8100→100 得 +192）；底盘速度环用 `speed_rpm`；拨盘位置环用累计编码，单发目标步进约 26225 个计数，堵转反转步进约 52450；`M3508_DECELE_RATIO = 1/19`、`M3508_MAX_RPM = 8000`；板间 CAN 帧（ID 0x113 之类）传递 pitch 等量。
- **推算（推算，非实机）**：拨盘一格 60° 对应 $8192\times19.2032/6 = 26219$ 个转子侧计数，与笔记里的 26225 差 6 个计数（0.02%），说明拨盘很可能是减速比约 19.2 的 M3508（60° 一格，即 6 格拨盘）。这是一个**很好的自洽性检验**：面试时可以说 "26,000-ish counts correspond to 60 degrees of the feeder through a roughly 19:1 gearbox"，但要加 "if I recall correctly"。【待你补全：拨盘电机型号、每格角度、单发步进计数】
- **要自己补全**：总线分配、波特率、过滤器设置、离线阈值、电流限幅值、是否用过 GM6020、遇到的 CAN 故障。

## 15. 常见故障

| # | 现象 | 原因 | 定位 | 处理 |
|---|---|---|---|---|
| 1 | 单个电机偶发掉线 | 接触不良、ID 冲突、线束干扰 | 看该电机 `last_rx_ms` 间隔直方图 | 重新压线、改 ID、屏蔽/绞线 |
| 2 | 所有电机同时间歇掉线 | 终端电阻、供电跌落、共地 | 示波器看 CAN 波形和电源 | 检查 120 Ω、供电余量、地 |
| 3 | 速度突然跳变一整圈 | 多圈混叠（丢帧 + 高速） | 记录相邻帧间隔和 ecd 差 | 限速、检测间隔、重对准 |
| 4 | 电机方向反了、系统发散 | 符号约定错误 | 小电流点动测试 | 在调用处加 −1 |
| 5 | 命令丢失，输出忽有忽无 | 发送邮箱占满 | 检查 `AddTxMessage` 返回值 | 查空闲邮箱，专职发送任务 |
| 6 | 负速度读成大正数 | 没转成有符号类型 | 打印原始字节 | `(int16_t)` 强转 |
| 7 | 云台上电时猛转 | 零位偏移没标定或初始目标不是当前角 | 上电打印 offset 与目标 | 先使能前对齐目标到当前角 |
| 8 | 板间数据时有时无 | 过滤器或 ID 与电机冲突 | 分析仪抓帧 | 重新规划 ID 段 |

## 16. 面试怎么问、怎么答

**Q1. Why did you use CAN for the motors?**
- 英文：*"We have many nodes sharing one pair of wires, in an electrically noisy environment. CAN is differential, so it rejects common-mode noise, it has hardware arbitration and CRC, and the short fixed frames give predictable latency."*
- 追问 1：*What's on the bus and how did you check the load?* → 用本章第 6 节的方法算；给出你的数字或"I estimated it as ... about 55% for four motors at 1 kHz"（先确认是你的情况）。
- 追问 2：*What happens when two nodes transmit at the same time?* → 仲裁，ID 小的赢，输者自动重发。
- 追问 3：*What if the bus load is too high?* → 分总线、降低命令频率、减少帧数（4 个电机合一帧）、过滤器、提高波特率（经典 CAN 上限 1 Mbps）。

**Q2. How do you get a continuous angle from a 13-bit encoder?**
- 英文：*"The motor reports a single-turn count from 0 to 8191. I compare consecutive samples: if the difference is more than half a turn I treat it as a wrap, increment or decrement a turn counter, and compute total counts as turns times 8192 plus the current count minus an offset. It assumes less than half a turn between samples, so I check the frame interval and limit the speed."*
- 追问：*When does it fail?* → 丢帧 + 高速，如 8000 rpm 下丢 3 帧后一帧转 4369 计数超过半圈。

**Q3. What do the motor feedback frames contain and how do you convert them?**
- 英文：*"Rotor angle 0 to 8191, signed speed in rpm, torque current in protocol units and temperature. I convert rpm to rad per second by 2π/60, divide by the gear ratio for the output shaft, and convert the current command to amps with 20 over 16384 for this motor type."*（按你的电机型号核对。）

**Q4. Where is the current loop?**
- 英文：*"Inside the motor driver. My controllers on the STM32 output a current command to the driver, which regulates it. So the gimbal's angle and rate loops sit on top of the driver's current loop."*

**Q5. How do you detect a motor that went offline?**
- 英文：*"Each motor has a timestamp updated in the receive callback. A monitoring task checks the age against a threshold a few times the feedback period. If it is stale, we mark the motor offline and move the subsystem to a safe state, usually zero current."*

**Q6. Why is the sign correction in the caller and not the controller?**
- 英文：*"The controller is generic; motor mounting direction is a hardware fact. Putting the correction at one place at the output stage keeps controllers reusable and makes direction bugs easy to find."*

## 本章要点速记

1. CAN 差分、多主、ID 即优先级、硬件 CRC/重发，适合机器人电机总线。
2. 标准数据帧 8 字节名义 111 位（含帧间隔），最坏填充约 135 位；1 Mbps 下约 111～135 µs。
3. 4 个电机 1 kHz 反馈 + 1 个命令帧 ≈ 55% 负载；8 个电机一条总线会超载，要分两条。
4. bxCAN：3 发送邮箱、2 个 3 级 FIFO、过滤器组；连发 >3 帧会丢，需检查空闲邮箱。
5. DJI 电机反馈：角度 0～8191（转子侧）、转速 rpm（有符号）、转矩电流、温度；大端。
6. 命令 ±16384 ↔ ±20 A（M3508）；1 单位 ≈ 1.22 mA；M3508 的 0.3 N·m/A 指输出轴侧。
7. 多圈累计用半圈阈值法，假设相邻采样转动不超过半圈；高速 + 丢帧会混叠。
8. 云台关节相对角需绕回到 ±半圈；它是相对结构的角，不是惯性系角。
9. 电流环在电调里；MCU 的 PID 是其外的速度/位置环。
10. 离线检测靠时间戳；离线后子系统进入安全状态。
11. 方向修正集中放在输出层；反馈字段与命令字段命名要区分。

## 自测题

1. 计算：1 Mbps 下 6 个电机反馈（1 kHz）+ 2 个命令帧/ms 的总线负载。
   *答：(6+2)×111 = 888 µs → 88.8%，偏高；最坏填充会超过 100%，应拆分总线。*
2. 编码值从 50 变到 8150，圈数怎么变？真实位移是多少？
   *答：差 8100>4096 → round_cnt−1；真实位移 −92（50 − 8150 + 8192 = 92，方向为反向），即 −92 计数。*
3. 转子 3000 rpm，帧间隔 1 ms，一帧转多少计数？最多丢几帧仍安全？
   *答：8192×50×0.001=409.6 计数；允许间隔 4096/409.6=10 ms，即最多丢 9 帧。*
4. 指令值 −4000 对应多少安培？M3508 输出轴力矩约多少（0.3 N·m/A）？
   *答：−4000×20/16384 = −4.883 A；约 −1.465 N·m。*
5. CAN 1 Mbps、f_CAN=42 MHz，Prescaler=2，要求位时间 1 µs，需要多少 tq？BS1+BS2 取多少？
   *答：tq=2/42M=47.6 ns；21 tq；BS1+BS2=20，例如 BS1=16、BS2=4（采样点 17/21=81%）。*
6. 解释"仲裁输家为什么不丢数据"。
   *要点：逐位比较，输家在仲裁阶段退出并转接收，帧继续由赢家发送，输家稍后重发。*
7. 为什么 `speed_rpm` 要强转为 `int16_t`？
   *要点：反馈为有符号补码，不转会把负速度当大正数。*

## 相关链接

- 上一章：[03 FreeRTOS 任务设计与时序](./03_FreeRTOS任务设计与时序.md)
- 下一章：[05 IMU/AHRS 与云台姿态](./05_IMU_AHRS与云台姿态.md)
- 基础：[CAN / FDCAN 与电机通信](../../02_基础知识/08_嵌入式与通信/03_CAN_FDCAN与电机通信.md)、[电机、编码器、驱动与力矩控制](../../02_基础知识/08_嵌入式与通信/05_电机_编码器_驱动与力矩控制.md)
- 相关项目：[P7 轮腿机器人 README](../P7_可变腿长轮腿机器人/README.md)（FDCAN 对比）
- 学习笔记：[P3 本对话问答逐字稿](../../05_学习笔记归档/P3/P3_本对话问答逐字稿.md)
