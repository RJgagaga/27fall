# STM32H7 / FreeRTOS / FDCAN 软件结构

> **一句话定位**：算法再对，也要在固定周期里读传感器、算控制、把力矩可靠地送到电机。本章讲 CV 里"STM32H7/FreeRTOS + FDCAN 执行链"的典型实现：任务划分、控制周期与顺序、数据共享、FDCAN 配置、MIT 风格电机帧（对照课程仓库真实代码）、总线负载和安全措施。本项目取值：控制与估计 500 Hz（2 ms），IMU 1 kHz，FDCAN 力矩命令 500 Hz，关节与轮各用一路 1 Mbps 经典 CAN；课程仓库是 ESP32-S3 + Arduino，只作结构参考。

**本章学完你能：**
1. 画出控制任务、IMU 任务、FDCAN 接收中断、监控任务之间的数据流与优先级关系；
2. 解释"固定周期""快照""过期检测"各自解决什么问题；
3. 手算 MIT 风格帧的量化与解码误差，并说清力矩模式与位置模式的区别；
4. 估算总线负载，说明为什么本项目把关节与轮分到两路 FDCAN；
5. 说出至少五条安全措施，并指出课程代码缺哪些；
6. 用英文回答"一条力矩命令怎样到达电机"和"控制周期怎么选"。

**前置知识：** [FreeRTOS](../../02_基础知识/08_嵌入式与通信/02_FreeRTOS实时系统.md)、[CAN/FDCAN 与电机通信](../../02_基础知识/08_嵌入式与通信/03_CAN_FDCAN与电机通信.md)、[数字控制](../../02_基础知识/04_控制理论/08_数字控制与实时性.md)

**目录：** 1 解决什么问题 · 2 符号与单位 · 3 任务划分 · 4 一个周期的顺序 · 5 数据共享与过期检测 · 6 FDCAN 配置与收发 · 7 MIT 帧与手算 · 8 总线负载 · 9 安全措施 · 10 本项目怎么用 · 11 常见错误 · 12 面试问答 · 13 自测题

---

## 1. 它要解决什么问题

平衡控制是一个**强实时**的闭环：机身是不稳定对象，开环极点约 $\pm18.9$ rad/s，对应发散时间常数约 53 ms（脚本一）。如果控制周期抖动、传感器数据过期，或者力矩命令丢帧，闭环就会等效地多出一段延迟，轻则振荡，重则倒下。软件结构要回答四个问题：

1. **什么时候算**：固定周期，抖动足够小；
2. **用什么数据算**：同一时刻的一致快照，且知道它是否过期；
3. **怎么发出去**：总线带宽够不够，编码精度够不够，丢帧怎么办；
4. **出了问题怎么办**：掉线、过温、数据过期、力矩越界时的降级与停机。

这对应 CV 中"在 STM32H7/FreeRTOS 与 FDCAN 链上集成反馈与力矩输出，把平衡异常追溯到状态不一致、力矩限幅与离地切换"。

## 2. 符号与单位

| 符号 | 含义 | 本项目取值 / 单位 |
|---|---|---|
| $T_s$ | 控制周期 | 2 ms（500 Hz） |
| $f_{imu}$ | IMU 采样率 | 1 kHz |
| $t_{comp}$ | 单周期计算时间 | 典型 0.1–0.2 ms（H7 带 FPU，单精度） |
| $x_{min},x_{max}$ | 编码范围 | 力矩 ±10 N·m，位置 ±12.5 rad，速度 ±30 rad/s |
| $n$ | 编码位数 | 位置 16，速度/Kp/Kd/力矩 12 |
| $u$ | 量化后的整数 | 0 到 $2^n-1$ |
| $B$ | 总线速率 | 1 Mbps（标称） |
| $N_{stale}$ | 数据过期判定周期数 | 3 个周期（6 ms） |

## 3. 任务划分

典型 FreeRTOS 划分如下（本项目取值写在最后两列）：

| 任务 | 触发 | 内容 | 优先级 | 周期 / 栈 |
|---|---|---|---|---|
| 控制 | `vTaskDelayUntil` 固定周期 | 取快照→运动学→估计→模式判断→LQR→腿长 PID→VMC→限幅→发 CAN | 最高 | 2 ms / 4 KB |
| IMU | 数据就绪中断唤醒（或定时） | 读 SPI IMU，Mahony 姿态解算，写快照 | 高 | 1 ms / 2 KB |
| CAN 接收 | FDCAN RX FIFO 中断 | 解码电机反馈，写共享结构并记时间戳 | 中断 | 事件触发 |
| 通信/遥控 | 低速 | 解析遥控帧，速度指令斜坡 | 低 | 20 ms（50 Hz） |
| 监控 | 低速 | 数据过期、过温、掉线、急停、日志 | 中 | 10 ms（100 Hz） |

要点：

- **优先级表示谁先运行，不等于频率。** 控制任务优先级最高，是因为它错过截止期的代价最大，而不是因为它最频繁。
- **实际周期必须测量。** 用 DWT 周期计数器或硬件定时器给每个控制周期打时间戳，统计最大抖动；`vTaskDelayUntil(&last, pdMS_TO_TICKS(2))` 要求 tick 为 1 kHz 或更高（本项目 tick 1 kHz，2 ms 恰好 2 个 tick；若要求更小的周期，可改用硬件定时器中断通知任务）。
- **中断里只做短事。** FDCAN 接收中断只解码与记录时间戳；不能做浮点重计算，也不能调用阻塞 API。

```text
 IMU(SPI) --1kHz--> [IMU 任务] --写快照--+
                                          v
 FDCAN RX(中断) --解码+时间戳--> [共享状态: imu, joints, wheels, 时间戳, 计数]
                                          |  一次性取快照
                                          v
                              [控制任务 2 ms] --限幅--> FDCAN TX(关节总线、轮总线)
                                          ^
          [通信任务 50Hz] --速度指令(斜坡)-+      [监控任务 100Hz] 过期/过温/急停
```

## 4. 一个周期的顺序

控制任务一个周期内的顺序（本项目）：

1. 等待到周期起点（`vTaskDelayUntil`），记录时间戳 $t_k$；
2. **取一次快照**：IMU 姿态与角速度、4 个关节的角度与角速度、2 个轮的速度，连同各自时间戳和计数器；
3. 过期检测：若任一数据距 $t_k$ 超过 $N_{stale}T_s$，进入降级；
4. 运动学（腿长、腿角、速度）→ 状态估计 → 离地判断；
5. LQR（增益由腿长多项式计算，在线 clamp）→ 腿长 PID + 前馈 → VMC → 关节力矩；
6. 限幅与编码范围夹紧，写入 FDCAN 发送缓冲，发送；
7. 记录日志（状态、力矩、模式、周期实际用时）。

**课程的顺序陷阱。** 课程第十一课主循环是：先 `wheel_control`，再 `CAN_Control`（收反馈并**发送上次算好的力矩**），之后才算本轮 VMC。所以新力矩下一周期才发出，多一拍延迟；轮控制用的腿几何也来自上一拍。本项目的顺序是"读—算—发"同周期完成，没有这多出的一拍：从快照到命令发出，延迟约等于计算时间（0.1–0.2 ms）加发送时间（约 0.1 ms）。

**延迟预算。** 反馈链上的总延迟约为：IMU 采样延迟（最多 1 ms）+ 关节反馈延迟（上一个电机反馈帧，最多 2 ms）+ 计算 0.2 ms + 发送 0.2 ms + 电机内部电流环响应（约 0.5 ms）。总计约 3–4 ms，相对于不稳定极点的时间常数 53 ms 约占 7%，相位损失可接受；若周期放到 10 ms，延迟占比就会接近 30%，闭环容易振荡。

## 5. 数据共享与过期检测

**为什么需要快照。** 控制任务不能一边读一边被 IMU 任务或接收中断改写：姿态读到一半新一半旧，会得到一个从未存在过的状态（撕裂读）。常见做法有三种：

1. **临界区拷贝**：关中断或拿互斥锁，把整个结构体拷到局部变量，再解锁。结构体小（几十个 float），耗时微秒级；
2. **双缓冲 / 序号锁**：写者递增序号，读者读前后各看一次序号，不相等则重读；
3. **消息队列**：长度为 1 的队列 `xQueueOverwrite`，读者用 `xQueuePeek`。

本项目用第 1 种加每份数据的**时间戳与计数器**，示例结构：

```c
typedef struct {
    float    pitch, pitch_rate, roll, roll_rate;  // IMU
    uint32_t imu_stamp_us, imu_cnt;
    float    q[4], dq[4], tau[4];                 // 关节反馈
    float    wheel_w[2];                          // 轮速
    uint32_t fb_stamp_us[6], fb_cnt[6];           // 每个电机最近一帧
} RobotSnapshot_t;                                // 教学示例，非项目源码
```

**过期检测。** 若某个电机反馈帧的时间戳距当前超过 $N_{stale}$ 个周期（本项目 3 个周期，即 6 ms），则认为数据过期（线断了、电机掉线、总线错误）。处理：先把该电机命令降为零力矩，若连续过期超过 20 ms 或关节过期，则整机进入安全模式。**绝不能用旧数据继续算**——这是 [11](./11_调试与故障手册.md) 里 F7"像有延迟、易振荡"的根本原因。

## 6. FDCAN 配置与收发

STM32H7 的 FDCAN 外设有独立的消息 RAM，需要配置：

- **位时序**：以核时钟 80 MHz 为例，预分频 5，时间量子 16 个（同步 1 + 段 1 为 13 + 段 2 为 2），得到 $80/5/16=1$ Mbps，采样点 $(1+13)/16=87.5\%$。
- **帧格式**：本项目关节电机为经典 CAN 8 字节帧（`FDCAN_FRAME_CLASSIC`），因为电机固件按经典帧；若电机支持 CAN FD，可开 BRS 加速数据相位。
- **过滤器**：只接收电机反馈 ID 段，其余丢弃，减轻中断负担。
- **发送**：`HAL_FDCAN_AddMessageToTxFifoQ`，每次检查返回值与 FIFO 是否已满，满则计入丢帧计数；
- **接收**：RX FIFO0 新消息回调，只做解码与记录时间戳；
- **错误处理**：监控 `bus-off` 与 `error passive` 状态，进入 bus-off 后按退避重启，计数并告警。

```c
// 教学示例：接收回调，非项目源码
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *h, uint32_t it) {
    FDCAN_RxHeaderTypeDef hdr; uint8_t d[8];
    while (HAL_FDCAN_GetRxMessage(h, FDCAN_RX_FIFO0, &hdr, d) == HAL_OK) {
        int idx = id_to_index(hdr.Identifier);       // 节点 ID -> 0..5
        if (idx < 0) continue;
        decode_feedback(d, &g_fb[idx]);              // pos/vel/tor
        g_fb_stamp_us[idx] = micros(); g_fb_cnt[idx]++;
    }
}
```

## 7. MIT 风格电机帧（课程真实代码对照）

课程 `sendMITCommand`：先把 pos/vel/kp/kd/tor 各自限幅到范围（pos ±12.56，vel ±45，kp 0–500，kd 0–5，tor ±18），按公式量化：

$$u=\frac{x-x_{min}}{x_{max}-x_{min}}(2^n-1)$$

位置 16 位，速度/kp/kd/力矩各 12 位，打包成 8 字节：

```text
byte0-1: pos(16)   byte2: vel[11:4]   byte3: vel[3:0] | kp[11:8]
byte4: kp[7:0]     byte5: kd[11:4]    byte6: kd[3:0] | tor[11:8]   byte7: tor[7:0]
```

帧 ID = 功能 ID（课程为 0x200）+ 节点 ID；使能/失能/置零是固定字节序列（前 7 字节 0xFF，末字节 0xFC/0xFD/0xFE）。反馈帧按相反位布局解码为 pos/vel/tor。

**本项目的编码范围**：pos ±12.5 rad，vel ±30 rad/s，kp 0–500，kd 0–5，tor ±10 N·m（与课程不同，由电机型号决定）。

**手算（课程范围）：** tor=9，范围 ±18，12 位：$u=27/36\times4095=3071.25\to3071$，解码 $3071\times36/4095-18=8.9978$，误差 0.002；pos=1.0 rad，16 位范围 ±12.56：$u=35376$，解码 0.99986。

**手算（本项目范围）：** tor=3.0 N·m，范围 ±10，12 位：$u=\dfrac{3+10}{20}\times4095=2661.75\to2661$，解码 $2661\times20/4095-10=2.9963$，误差 0.0037。力矩分辨率 $20/4095=0.0049$ N·m；位置 16 位：分辨率 $25/65535=0.00038$ rad。pos=1.0 rad：$u=35388$，解码 0.99966。越界 12 N·m 被夹到 10（脚本三验证）。

**两种模式：**

- **力矩模式**：`pos=vel=kp=kd=0`，只填 `tor`（课程第十一课腿关节如此，本项目关节与轮也如此）。电机内部的电流环把力矩命令变成电流；
- **位置模式**：填 pos 与 kp（及 kd），电机内部做 $\tau=k_p(p_{des}-p)+k_d(v_{des}-v)+\tau_{ff}$（第六课用法）。位置模式适合上电时让腿缓慢到初始姿态。

**为什么用力矩模式做平衡：** 平衡与 VMC 的输出就是力矩，位置模式会引入电机内部的弹簧，与上层控制器抢权。

## 8. 总线负载

经典 CAN 标准帧 8 字节约 111 位，最坏位填充约 135 位；1 Mbps 下 0.111–0.135 ms。

- **一路总线：4 个关节命令 + 4 个反馈 = 8 帧**，约 0.89–1.08 ms，占 2 ms 周期的 44%–54%；
- 如果 6 个电机（4 关节 + 2 轮）全在一路：12 帧，1.33–1.62 ms，占 67%–81%，余量不足，一旦重发或出现 bus 错误就会丢帧；
- **本项目方案**：关节一路 FDCAN、轮一路 FDCAN，各约 50% 负载；
- 若要 1 kHz 控制：8 帧 ≈ 1 ms 占满整个周期，必须降帧数或使用 CAN FD 数据相位加速。

细节与脚本见 [13](./13_手算与数值验证附录.md)。

## 9. 安全措施

| 措施 | 做法 | 课程代码 |
|---|---|---|
| 力矩限幅与编码夹紧 | 关节 6 N·m、轮 3 N·m（软件限幅）；编码范围夹紧；夹紧发生时计数并报警（静默饱和是隐患） | 有范围夹紧，无报警 |
| 数据过期检测 | 每路反馈带时间戳，超过 3 个周期降级 | 无 |
| 使能序列 | 逐个使能电机，间隔延时，确认第一帧反馈后才进入平衡 | 有使能帧，无确认 |
| 失联保护 | 超时无反馈 → 零力矩；上位机命令超时 → 速度指令归零 | 无 |
| 急停 | 硬件按键/遥控急停：所有力矩置零（轮电机失能） | 无 |
| 看门狗 | 独立看门狗，由监控任务在所有任务健康时喂 | 无 |
| DLC / 长度检查 | 收到的帧长度必须为 8，否则丢弃 | 无 |
| 过温降额 | 电机温度升高时线性降低力矩限幅 | 无 |

课程代码**没有**数据过期检查、DLC 检查和失联保护，需自己补。课程还有两处易误导的地方：`SEND_INTERVAL=1` ms 只是发送门槛而非实测频率；`twai_transmit` 的超时是 1000 ms 而注释写 1 ms。

## 10. 本项目怎么用

1. **启动流程**：上电自检（IMU 零偏采样 1 s、电机依次使能并读到反馈）→ 腿缓慢收到初始姿态（位置模式）→ 切换到力矩模式并启用平衡；
2. **周期**：控制 500 Hz，IMU 1 kHz，监控 100 Hz，遥控 50 Hz；
3. **总线**：FDCAN1 接 4 个关节电机，FDCAN2 接 2 个轮电机，各 1 Mbps 经典帧；
4. **日志**：每周期把状态、力矩、模式、周期用时写入环形缓冲，监控任务低速上传，用于 [11](./11_调试与故障手册.md) 的定位；
5. **与算法的接口**：算法层只读快照、只写"关节力矩数组"，不直接碰 HAL；这样把算法在 PC 上仿真和上机共用一份代码。

## 11. 常见错误

| 现象 | 原因 | 对策 |
|---|---|---|
| 周期时快时慢 | 用 `vTaskDelay` 而不是 `vTaskDelayUntil`；优先级被抢 | 用 Until；测量最大抖动 |
| 偶发力矩尖峰 | 撕裂读（读到半新半旧的状态） | 快照加临界区 |
| 命令有时不生效 | TX FIFO 满，丢帧未检查 | 检查返回值，计数 |
| 电机偶尔失控 | 使能序列错乱，或上电时 kp/kd 残留 | 使能前先发零命令 |
| 反馈一直是旧值 | RX 回调里没清标志或过滤器配置错 | 看 RX 计数 |
| 力矩比预期小 | 编码范围与电机固件不一致，静默夹紧 | 对照电机手册，静态加载核对 |
| 提高频率反而更差 | 总线接近饱和，重发和错误增多 | 看负载，降频或拆总线 |

## 12. 面试怎么问、怎么答

**Q1. How does a torque command reach the motor?**
> The control task computes joint torques, clamps them to the limits, scales each field into 12 or 16 bits, packs eight bytes, adds the node ID to the frame ID and queues it on FDCAN. The motor replies with its position, velocity and torque, decoded in the receive interrupt with a timestamp.

（中文：控制任务算力矩、限幅、量化、打包 8 字节、加节点 ID、入 FDCAN 发送队列；电机回传位置速度力矩，接收中断解码并打时间戳。）

**Q2. What is the risk of a high control rate on CAN?**
> Bus load: eight 8-byte frames take about a millisecond at 1 Mbps, so a 2 millisecond cycle is already about half used. For a higher rate I would need CAN FD or fewer frames, so I put joints and wheels on two buses.

**Q3. How do you make sure the controller does not use stale data?**
> Every feedback has a timestamp and a counter. At the start of each cycle the control task takes one snapshot and checks the age. If a motor is older than three cycles I command zero torque for it and, if it persists, go to a safe mode.

**Q4. Why is the control task the highest priority?**
> A missed deadline in the balance loop is the most costly failure. Priority decides who runs first, not the frequency; the actual period is measured with timestamps.

**Q5. How do you choose the control period?**
> It must be much faster than the closed-loop dynamics, and the delay it adds must be small compared with the unstable pole's time constant, about fifty milliseconds here. Two milliseconds gives a few milliseconds of total delay, and it leaves about half of the bus free.

追问方向：为什么不用 1 kHz（总线和计算余量）；如果电机在运行中掉线（过期检测与降级）；IMU 与关节的时间对齐（周期开头取快照，必要时用时间戳插值）。

## 13. 自测题

1. 优先级等于频率吗？（否，优先级决定抢占顺序。）
2. tor=9、范围 ±18、12 位的编码值？（3071。）
3. 本项目 tor=3.0 N·m 的编码值与解码值？（2661；2.9963。）
4. 为何降频或用 FD？（总线负载。）
5. 为什么平衡时用力矩模式而不是位置模式？（上层输出就是力矩，位置模式的内部弹簧会与上层抢权。）
6. 过期检测超过几个周期降级？（3 个周期，6 ms。）

## 要点速记

1. 任务：控制（最高）、IMU、FDCAN 接收中断、通信、监控。
2. `vTaskDelayUntil` 固定周期，实测抖动。
3. 周期内"取快照—算—发"，不多一拍。
4. 快照 + 时间戳 + 计数器，过期 3 个周期降级。
5. MIT 帧 8 字节：位置 16 位，其余 12 位；力矩模式 kp=kd=0。
6. 1 Mbps 下 8 帧约 1 ms；关节与轮分两路总线。
7. 限幅、过期、使能序列、急停、看门狗、DLC 检查。
8. 课程代码缺安全保护，不能照搬。

## 相关链接

上一章：[09](./09_离地检测与接触切换.md)　下一章：[11](./11_调试与故障手册.md)　源码：[12](./12_课程仓库源码导读.md)　数值：[13](./13_手算与数值验证附录.md)
