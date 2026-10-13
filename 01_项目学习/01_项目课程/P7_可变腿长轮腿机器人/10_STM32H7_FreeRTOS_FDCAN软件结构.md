# STM32H7 / FreeRTOS / FDCAN 软件结构

> **一句话定位**：算法再对，也要在固定周期里读传感器、算控制、把力矩可靠地送到电机。本章讲典型的任务划分、控制周期、MIT 风格电机帧（对照课程仓库真实代码）和总线负载。你的实机细节【待你补全】；课程仓库是 ESP32-S3 + Arduino，仅作结构参考。

**前置知识：** [FreeRTOS](../../02_基础知识/08_嵌入式与通信/02_FreeRTOS实时系统.md)、[CAN/FDCAN 与电机通信](../../02_基础知识/08_嵌入式与通信/03_CAN_FDCAN与电机通信.md)

## 1. 典型任务划分

| 任务 | 周期/触发 | 内容 | 优先级 |
|---|---|---|---|
| IMU | 数据就绪中断/定时 | 读 IMU，姿态解算，写快照 | 高 |
| 控制 | `vTaskDelayUntil` 固定周期 | 取快照→运动学→估计→LQR→VMC→限幅→发 CAN | 最高 |
| CAN 接收 | FDCAN RX FIFO 中断 | 解码电机反馈，写入共享结构并记时间戳 | 中断 |
| 通信/遥控 | 低速 | 指令解析，斜坡 | 低 |
| 监控 | 低速 | 数据过期、过温、掉线、急停 | 中 |

要点：优先级表示谁先运行，不等于频率；实际周期必须用时间戳测量。共享数据用互斥或"整块快照"，并带计数器检测过期。

## 2. 一个周期的顺序陷阱

课程第十一课主循环：先 `wheel_control`，再 `CAN_Control`（收反馈并**发送上次算好的力矩**），之后才算本轮 VMC。所以新力矩下一周期才发出，多一拍延迟；轮控制用的腿几何也来自上一拍。你的实机顺序【待你补全】，但"先读—算—发"并保证一次快照是正确做法。

## 3. MIT 风格电机帧（课程真实代码）

课程 `sendMITCommand`：先把 pos/vel/kp/kd/tor 各自限幅到范围（pos ±12.56，vel ±45，kp 0–500，kd 0–5，tor ±18），按公式量化：

$$u=\frac{x-x_{min}}{x_{max}-x_{min}}(2^n-1)$$

位置 16 位，速度/kp/kd/力矩各 12 位，打包成 8 字节：

```text
byte0-1: pos(16)   byte2: vel[11:4]   byte3: vel[3:0] | kp[11:8]
byte4: kp[7:0]     byte5: kd[11:4]    byte6: kd[3:0] | tor[11:8]   byte7: tor[7:0]
```

帧 ID = 功能 ID(0x200) + 节点 ID；使能/失能/置零是固定字节序列（末字节 0xFC/0xFD/0xFE）。反馈帧按相反位布局解码为 pos/vel/tor。

**手算：** tor=9，范围 ±18，12 位：$u=27/36\times4095=3071.25\to3071$，解码 $3071\times36/4095-18=8.9978$，误差 0.002；pos=1.0 rad，16 位范围 ±12.56：$u=35376$，解码 0.99986。

力矩模式：`pos=vel=kp=kd=0`，只填 `tor`（课程第十一课腿关节如此）；位置模式：填 pos 与 kp（第六课）。**你的电机协议和范围【待你补全】。**

## 4. FDCAN 要点（STM32H7，典型）

- 配置：标称波特率（如 1 Mbps）、经典 CAN 或 FD+BRS、消息 RAM、过滤器只接收电机反馈 ID；
- 发送：`HAL_FDCAN_AddMessageToTxFifoQ`，检查返回与 FIFO 满；
- 接收：RX FIFO 新消息中断/回调，只做解码与时间戳，不做重计算；
- 错误：总线关闭（bus-off）、错误被动要监控并恢复，丢帧计数。

## 5. 总线负载（推算）

经典 CAN 标准帧 8 字节约 111 位，最坏位填充约 135 位；1 Mbps 下 0.11–0.135 ms。每周期 4 个关节命令 + 4 个反馈 = 8 帧 ≈ 0.9–1.1 ms；加两个轮电机则更多。所以 1 kHz 控制在 1 Mbps 经典 CAN 上接近饱和，要么降频，要么用 CAN FD 数据相位加速，要么减少帧。你的实际总线速率、电机数、周期【待你补全】。

## 6. 安全措施

力矩限幅与编码范围夹紧（静默饱和要报警）、数据过期检测（超 N 周期不更新→降级或零力矩）、使能序列（逐个使能、延时）、急停、看门狗。课程代码**没有**数据过期检查、DLC 检查和失联保护，需自己补。

## 7. 面试怎么问

**Q. How does a torque command reach the motor?**
> The control task computes joint torques, clamps them to the limits, scales each field into 12 or 16 bits, packs eight bytes, adds the node ID to the frame ID and queues it on FDCAN. The motor replies with its position, velocity and torque, decoded in the receive interrupt with a timestamp.

**Q. What is the risk of a high control rate on CAN?**
> Bus load: eight 8-byte frames take about a millisecond at 1 Mbps, so the rate is limited unless I use CAN FD or fewer frames.

## 要点速记

1. 任务：IMU/控制/CAN 中断/监控。2. 固定周期+一次快照。3. MIT 帧 8 字节，量化 12/16 位。4. 力矩模式 kp=kd=0。5. 1 Mbps 下 8 帧≈1 ms。6. 过期检测与安全。

## 自测题

1. 优先级等于频率吗？（否）2. tor=9 编码值？（3071）3. 为何降频或用 FD？（总线负载）

## 相关链接

上一章：[09](./09_离地检测与接触切换.md)　下一章：[11](./11_调试与故障手册.md)　源码：[12](./12_课程仓库源码导读.md)
