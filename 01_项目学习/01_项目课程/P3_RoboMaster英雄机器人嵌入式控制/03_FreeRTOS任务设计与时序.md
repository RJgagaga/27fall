# 03 FreeRTOS 任务设计与时序

> **一句话定位**：这一章回答面试官最爱追问的一类问题——"你的控制环到底多快？任务怎么划分？中断和任务怎么配合？数据会不会读到一半？"。重点不是背 API，而是建立"配置的周期 ≠ 实际周期"的时序直觉，并学会测量它。

## 本章学完你能……

- 讲清任务（task）、就绪/运行/阻塞状态、优先级抢占、时间片、tick 的含义；
- 区分 `vTaskDelay`、`vTaskDelayUntil`、`osDelay`，并手算一个循环里多处 `vTaskDelay(1)` 的真实周期；
- 设计一套适合云台/发射/底盘/IMU/通信的任务划分，并说明优先级和周期的理由；
- 用任务通知（task notification）把"DMA/中断收到数据"可靠地交给任务，并解释通知合并导致丢样的问题；
- 解释队列、二值信号量、互斥量、临界区各自适用的场景，以及优先级反转与优先级继承；
- 用利用率和最坏响应时间的方法做一次最简调度分析（手算）；
- 用 GPIO 翻转或 DWT 周期计数器测量实际周期与抖动；
- 诚实回答"你的控制频率是多少"。

## 前置知识

- [FreeRTOS 实时系统（基础）](../../02_基础知识/11_嵌入式与通信/02_FreeRTOS实时系统.md)
- [STM32 时钟、GPIO、中断、DMA](../../02_基础知识/11_嵌入式与通信/01_STM32_时钟_GPIO_中断_DMA.md)
- [数字控制与实时性](../../02_基础知识/05_控制理论/08_数字控制与实时性.md)
- 上一章：[02 整机电控架构与数据流](./02_整机电控架构与数据流.md)

## 目录

1. 为什么先要学"时序"
2. 任务、状态与调度规则
3. tick 与延时：三个函数的真实含义
4. 手算：一个循环真实的周期是多少
5. 典型任务划分与优先级设计
6. 中断到任务：通知、队列、信号量
7. 共享数据：为什么会读到"拼接数据"
8. 互斥量、优先级反转与优先级继承
9. Cortex-M 中断优先级与 FreeRTOS 的配合
10. 栈、堆与常见崩溃
11. 简易调度分析：利用率与响应时间（手算）
12. 测量：周期、抖动、延迟
13. 在本项目里怎么用（CV / 笔记线索 / 典型）
14. 常见错误与故障现象
15. 面试怎么问、怎么答
16. 要点速记、自测题

---

## 1. 为什么先要学"时序"

数字控制器的所有增益（比例、积分、微分）都隐含了一个**采样周期**。如果代码里的积分写成 $I \mathrel{+}= K_i e$ 而没有乘 $\Delta t$，那么增益里就"吸收"了周期；周期一变，等效积分增益就变。IMU 解算里的积分步长、滑模控制里的误差微分、遥控杆到目标角的换算——都依赖周期。所以：

> **周期是控制参数的一部分。** 你的控制器"调好了"，是在**某个特定周期（含抖动）**下调好的。

而 FreeRTOS 的 `vTaskDelay(1)` **不等于**"每 1 ms 运行一次"。这就是本章的核心。

## 2. 任务、状态与调度规则

### 2.1 任务（Task）

一个任务是一个**永不返回的函数**加上它自己的栈和一个任务控制块（TCB，记录优先级、栈指针、状态等）：

```c
void gimbal_task(void *argument)
{
    for (;;) {
        /* 读反馈 -> 计算 -> 发命令 */
        vTaskDelay(1);   /* 让出 CPU */
    }
}
```

单核 Cortex-M4 在任一时刻只运行一个任务（或中断）。"多任务"是通过快速切换实现的。

### 2.2 四种状态

```text
              创建
                │
                ▼
   ┌───────▶ Ready (就绪) ◀────────────┐
   │            │  被调度器选中          │ 事件到达 / 延时到期
   │            ▼                       │
   │       Running (运行) ──────────▶ Blocked (阻塞)
   │            │          等延时/队列/通知/信号量
   │            │ vTaskSuspend           
   │            ▼                        
   └──── Suspended (挂起)
```

- **Running**：正在占用 CPU 的那一个；
- **Ready**：可以运行，但在等 CPU（因为有更高或同级任务在跑）；
- **Blocked**：在等某个事件（时间到、队列有数据、通知到达）；阻塞的任务不占 CPU；
- **Suspended**：被显式挂起，不参与调度。

### 2.3 调度规则（FreeRTOS 默认配置）

1. **固定优先级抢占**：就绪任务中**优先级最高**的运行；高优先级任务一就绪，立即抢占低优先级任务（在下一个调度点——例如中断返回、tick）。
2. **同优先级轮转**：同一优先级的就绪任务按时间片（通常 1 tick）轮流运行（`configUSE_TIME_SLICING` 打开时）。
3. **优先级不是频率**：优先级只决定"同时就绪时谁先"，不决定任务多久运行一次；运行频率由任务自己的阻塞方式决定（延时、等通知等）。

> **常见误解**：高优先级 = 高频率。**错。** 一个高优先级任务如果每 100 ms 才醒一次，它的频率就是 10 Hz；一个低优先级任务如果每 1 ms 醒一次，它的频率就是 1 kHz（只要 CPU 够用）。

### 2.4 Idle 任务与空闲优先级

调度器自动创建 Idle 任务（优先级 0）。CMSIS-RTOS 封装里的 `osPriorityIdle` 是一个**优先级档位标签**，用户创建的任务如果设为该档位，则与 Idle 任务同级、会和它轮流运行——**这不是同一个任务**。

## 3. tick 与延时：三个函数的真实含义

### 3.1 tick

FreeRTOS 用 SysTick 定时器每隔 $1/f_{\text{tick}}$ 触发一次中断，`xTickCount` 加 1。典型配置 `configTICK_RATE_HZ = 1000`，即 1 tick = 1 ms。

> **【笔记线索】**：你的笔记里读到的源码快照 tick 为 1000 Hz。

### 3.2 `vTaskDelay(n)`：相对延时

含义：**把当前任务阻塞，直到 `xTickCount` 增加 n**。唤醒时刻 = 调用时的 tick 计数 + n。

关键点：调用时刻通常处于两个 tick 之间的某个位置，而唤醒总是对齐在 tick 边界上。所以：

$$
\text{实际睡眠时间} \in (n-1,\; n]\ \text{个 tick}
$$

对 `vTaskDelay(1)`：实际睡眠可以短至接近 0（在 tick 边界前一刻调用），长至接近 1 ms（刚过 tick 边界就调用）。

```text
情况 A：在 0.1 ms 调用 vTaskDelay(1)
tick:  0 ms |----------------------| 1 ms
任务:         调用 ------------------↑ 唤醒    睡了约 0.9 ms

情况 B：在 0.99 ms 调用 vTaskDelay(1)
tick:  0 ms |----------------------| 1 ms
任务:                          调用 ↑ 唤醒    睡了约 0.01 ms
```

还要注意：**"变成就绪"不等于"立刻运行"**。若此时有更高优先级的任务或中断，还要继续等。

### 3.3 `vTaskDelayUntil(&xLastWakeTime, period)`：绝对周期延时

含义：**从上一次"应该被唤醒"的时刻起算，下一个唤醒时刻 = 上一个 + period**。它用一个不会被执行时间影响的基准，因此长期平均周期**严格等于** period（只要每次执行时间 < period），每次的唤醒时刻也只带有 tick 级的抖动。

```c
TickType_t last = xTaskGetTickCount();
for (;;) {
    vTaskDelayUntil(&last, pdMS_TO_TICKS(2));   /* 2 ms 周期 */
    control_step();
}
```

对比：

| 项 | `vTaskDelay(n)` | `vTaskDelayUntil(&t, n)` |
|---|---|---|
| 基准 | 调用时刻（相对） | 上一次计划唤醒时刻（绝对） |
| 周期 | 执行时间 + 睡眠，随执行时间漂移 | 固定为 n（执行时间 < n 时） |
| 适合 | "做完后稍等一下" | 固定周期控制环 |

### 3.4 `osDelay(ms)`：CMSIS-RTOS 封装

`osDelay` 是 CMSIS-RTOS v1 接口，参数是**毫秒**，内部换算成 tick 再调 `vTaskDelay`，且至少 1 tick。在 tick = 1 ms 的配置下，`osDelay(1)` ≈ `vTaskDelay(1)`；但如果 tick = 10 ms，`osDelay(1)` 会睡大约一个 10 ms 的 tick。

> **【笔记线索】**：你的笔记核对过 `cmsis_os.c`：毫秒除以 tick 毫秒数后调用 `vTaskDelay`，不足一个 tick 按一个 tick 算。

### 3.5 为什么这对控制至关重要

控制环如果用 `vTaskDelay(1)`，那么周期"大约是 1 ms 加上执行时间的某种量化"。若循环里还有别的延时（例如 CAN 发送之间插入延时），周期会成倍增加——下一节手算。

## 4. 手算：一个循环真实的周期是多少

### 4.1 例 1：单次 `vTaskDelay(1)`，执行 0.4 ms（教学示例）

tick 边界在 0、1、2、3 ms。任务在 t=0（刚好 tick 边界）醒来，执行 0.4 ms，于 t=0.4 调用 `vTaskDelay(1)`：此时 `xTickCount` 仍为 0，唤醒目标是 tick 1，即 t=1.0 ms。所以**周期 = 1.0 ms**，而不是常有人误算的 0.4+1=1.4 ms。

如果执行时间变成 1.2 ms：在 t=1.2 调用（当前 tick 计数=1），唤醒目标 tick 2，即 t=2.0 ms；周期 = 2.0 ms。于是：

$$
T = \left\lceil \frac{\text{执行时间}}{1\ \text{ms}} \right\rceil \times 1\ \text{ms}\quad(\text{任务起点在 tick 边界})
$$

即周期被**量化成整数个 tick**。

### 4.2 例 2：循环里有三处 `vTaskDelay(1)`（教学示例，模拟笔记里的结构）

【笔记线索】：你的笔记指出，快照里云台和底盘的任务循环里，**发送函数内部有两处 `vTaskDelay(1)`，循环末尾还有一处**（`GIMBAL_PERIOD`/`CHASSIS_PERIOD` 常量为 1）。结构如下：

```c
for (;;) {
    read_feedback();        /* 约 0.2 ms */
    send_data_to_other_board();   /* 内含：send(); vTaskDelay(1); send(); vTaskDelay(1); */
    control_calc();         /* 约 0.1 ms */
    send_motor_cmd();
    vTaskDelay(GIMBAL_PERIOD);   /* = 1 tick */
}
```

假设每段执行都远小于 1 ms，且任务起点在 tick 边界：

| 时刻 | 动作 |
|---|---|
| 0.0 | 醒来，读反馈 0.2 ms、第一次发送；到 0.2 调用 `vTaskDelay(1)` → 目标 tick 1 |
| 1.0 | 醒来，第二次发送；调用 `vTaskDelay(1)` → 目标 tick 2 |
| 2.0 | 醒来，计算、发电机命令；调用末尾 `vTaskDelay(1)` → 目标 tick 3 |
| 3.0 | 下一轮开始 |

**周期 ≈ 3 ms，频率 ≈ 333 Hz**（结构估算，任务被抢占或执行超过 1 ms 会更长）。所以不能仅凭 `PERIOD = 1` 说"1 kHz 控制环"。

另外，在这个结构里，**从读到反馈到发出电机命令之间隔了约 2 ms**（两次延时），这是控制回路里的**纯延迟**，会减少相位裕度（见第 11 节和 06 章）。

> **诚实回答模板**：*"The nominal tick was 1 ms, but inside the loop there were several one-tick delays, so the effective period was a few milliseconds, not 1 kHz. I would verify it with a GPIO toggle."*
> 译：名义 tick 是 1 ms，但循环里有几处一 tick 的延时，所以实际周期是几毫秒而不是 1 kHz；我会用 GPIO 翻转来核实。

### 4.3 这些延时可能是为了什么？

你需要在面试前想清楚（因为面试官一定会问"为什么在发送之间加延时"）：

STM32 bxCAN 只有 **3 个发送邮箱**。如果在很短时间内连续调用 `HAL_CAN_AddTxMessage` 发送多于 3 帧，邮箱会被占满，后续发送失败（返回错误或丢帧）。在帧与帧之间插入 1 tick 延时是一种"粗暴但有效"的避免方式。更好的做法有：

1. 发送前检查 `HAL_CAN_GetTxMailboxesFreeLevel()`；
2. 使用发送完成中断/回调释放信号量；
3. 设一个专门的 CAN 发送任务，通过队列接收各任务的发送请求并统一节奏。

本项目的情况：发送之间加 1 tick 延时的原因就是 bxCAN 只有 3 个发送邮箱，连发 0x200/0x1FF 等多帧时怕邮箱占满丢帧；这些延时累加使任务循环约 3 ms（约 333 Hz），拖慢了控制周期，这一点是后来用结构估算才意识到的。诚实地说并讲改进方案，是很好的面试回答（"what would you improve"）。

### 4.4 `vTaskDelayUntil` 下的对比（教学示例）

把上面的循环改成 `vTaskDelayUntil(&last, 3)`，每轮目标为 3 ms 整，抖动仅来自抢占。更好的做法是把"发给另一块板的数据"从控制环里拿出去，让控制环只做"读、算、发"，周期 1 或 2 ms，用 `vTaskDelayUntil` 固定。

## 5. 典型任务划分与优先级设计

### 5.1 设计原则

1. **事件触发的数据采集高优先级**（IMU）：它决定姿态新鲜度，且工作量短。
2. **闭环控制次之**：按"带宽高 → 优先级高"（速度环比位置环快，云台通常比底盘对延迟更敏感）。
3. **通信解析再次**：解析本身不重，但数据缓冲区不能溢出。
4. **UI、LED、日志最低**。
5. **每个任务尽量短**；不在高优先级任务里做 printf、阻塞等待外部设备、长循环。
6. 优先级数量尽量少，避免"优先级复杂度"。

### 5.2 典型任务表（教学设计，数值为示例）

| 任务 | 触发 | 周期/事件 | 优先级（高→低） | 主要工作 | 预计执行时间（示例） |
|---|---|---|---|---|---|
| INS（姿态解算） | IMU 数据就绪通知 | 与 IMU 数据率相同 | 最高 | 读取 DMA 数据、零偏/安装矩阵、滤波、AHRS 更新 | 0.25 ms |
| Gimbal（云台+发射） | `vTaskDelayUntil` | 1～2 ms | 高 | 模式、目标、角度环/角速度环、发射控制、CAN 发送 | 0.3 ms |
| Chassis（底盘） | `vTaskDelayUntil` | 1～2 ms | 中高 | 运动学、功率限制、轮速 PID、CAN 发送 | 0.2 ms |
| Detect（离线检测） | 周期 | 10 ms | 中 | 检查各设备时间戳，更新离线标志 | 0.05 ms |
| Comm（USB/视觉解码） | 数据到达通知 | 事件 | 中 | 帧解析、CRC、更新视觉目标 | 0.1 ms |
| Referee（裁判解析） | 空闲中断通知 | 事件 | 中 | 帧解析、分发 | 0.1 ms |
| UI / LED / Cap | 周期 | 20～100 ms | 最低 | 绘制 UI、LED 状态、电容通信 | 1 ms |

> **【笔记线索】**：你的笔记显示快照创建了 `default / calibrate / chassis / gimbal / imu / detect / usb / decode / ui / led / cap` 这些任务；其中**云台、USB、Decode 优先级较高，底盘较低，IMU 任务被设成 idle 档**。这一点与上面的"典型设计"不同（典型设计里 INS 应高于控制任务）。

### 5.3 面试官可能追问：为什么 IMU 任务优先级这么低？

诚实回答三步：

1. **先承认事实**（如果你确认源码确实如此）："In the version I reviewed, the INS task was at a low priority."
2. **解释为什么仍可工作**：INS 任务是通知驱动的，只要 CPU 负载不高，它在其他任务阻塞时就会运行并及时处理；低优先级只会在高负载时让姿态更新延迟。
3. **说明你会怎么改**：把 INS 提到不低于控制任务的优先级，并测量 INS 的通知到完成延迟与丢样计数。

本项目的情况：云台、USB、Decode 任务优先级较高，底盘较低，IMU 任务按笔记设置（偏低）；当时的考虑是 IMU 任务由通知唤醒、CPU 负载不高时不会饿死。被追问时就用上面这套"现状 → 风险 → 改进"的回答结构。

## 6. 中断到任务：通知、队列、信号量

### 6.1 为什么中断里要"少做事"

中断会打断所有任务，且在中断里不能阻塞。所以标准做法是：**中断里只做最少的事（取数据、清标志），然后唤醒一个任务来处理**。

### 6.2 任务通知（Task Notification）：最轻量

```c
extern TaskHandle_t ins_task_handle;

/* DMA 完成或 IMU 数据就绪 中断回调 */
void imu_dma_done_callback(void)
{
    BaseType_t woken = pdFALSE;
    vTaskNotifyGiveFromISR(ins_task_handle, &woken);
    portYIELD_FROM_ISR(woken);   /* 若唤醒了更高优先级任务，退出中断时立刻切换 */
}

void ins_task(void *arg)
{
    for (;;) {
        uint32_t n = ulTaskNotifyTake(pdFALSE, portMAX_DELAY);  /* 等通知；返回累计次数 */
        /* n > 1 说明上次处理太慢，丢了 n-1 次采样机会 */
        process_imu_and_ahrs();
    }
}
```

要点：

- `ulTaskNotifyTake(pdTRUE, ...)` 会把计数清零（**合并**多次通知）；`pdFALSE` 是"减 1"，可以数出积压了多少次。
- **通知合并 = 丢样风险**：若 INS 任务处理时间超过 IMU 数据周期，通知会累积或合并。AHRS 的积分步长若固定为 1 ms，丢样就意味着角度积分少积一步。
- `portYIELD_FROM_ISR` 很重要：不加的话，被唤醒的高优先级任务要等到下一个 tick 才有机会运行，白白增加 0～1 ms 延迟。

### 6.3 队列（Queue）

适合"有数据要传、可能积压、多个生产者或消费者"的场景，例如多个任务向一个 CAN 发送任务提交发送请求：

```c
typedef struct { uint32_t id; uint8_t data[8]; } can_tx_req_t;
QueueHandle_t can_tx_q;   /* 创建时：xQueueCreate(16, sizeof(can_tx_req_t)) */

/* 任一控制任务 */
xQueueSend(can_tx_q, &req, 0);   /* 满了立即返回，不阻塞控制环 */

/* CAN 发送任务（专职） */
for (;;) {
    can_tx_req_t r;
    if (xQueueReceive(can_tx_q, &r, portMAX_DELAY) == pdTRUE) {
        wait_free_mailbox();
        hal_can_send(&r);
    }
}
```

好处：发送节奏集中管理，控制环不再被"等邮箱"拖慢。

### 6.4 二值信号量 vs 通知

二值信号量也能做"中断通知任务"。通知更轻（不需要另建对象、更快、更省 RAM），但只能一个任务等；信号量可以多个任务争用。

### 6.5 DMA 空闲中断收变长帧（遥控/裁判/视觉）

典型流程：UART + DMA 循环接收；产生 IDLE（空闲）中断说明一帧结束；中断里计算收到的长度，通知处理任务，任务从缓冲区解析、校验后更新数据结构。要点：缓冲区要**双缓冲或在处理完前不被覆盖**，否则解析一半数据被新帧改写。

## 7. 共享数据：为什么会读到"拼接数据"

### 7.1 现象

【笔记线索】你的笔记指出：电机反馈在 CAN 接收中断里更新结构体，云台/底盘任务稍后读取。Cortex-M 上 **32 位对齐的读写是原子的**，但**多个字段不是**。可能的交错：

```text
任务读取 motor.ecd (旧一帧)
          ↓ 中断来了，更新了 ecd, speed, current
任务继续读取 motor.speed, motor.current (新一帧)
```

于是一次控制计算混合了两帧的数据。

### 7.2 影响有多大？

对速度环（反馈相差 1 ms）影响通常可忽略；但对**需要同一时刻的多量**（例如用同一时刻的角度和角速度做某种估计）则可能带来小误差。需要时才处理，别过度设计。

### 7.3 常见解决方法（从轻到重）

| 方法 | 说明 | 适用 |
|---|---|---|
| 接受并说明 | 差 1 ms 的数据对多数 PID 不敏感 | 一般反馈 |
| 临界区拷贝 | `taskENTER_CRITICAL(); local = shared; taskEXIT_CRITICAL();` 关中断时间极短（几十个字节） | 小结构体快照 |
| 双缓冲 + 序号 | 中断写 A/B 缓冲之一，完成后翻转"当前"索引 | 稍大数据 |
| Seqlock | 写前后序号加 1，读者检查序号一致才有效 | 单写多读 |
| 带时间戳 | 每帧带上接收时间，使用方判断新鲜度 | 离线检测 |

### 7.4 数据新鲜度

即使数据完整，也可能是旧的（CAN 断了，结构体还保留着上次的值）。所以每个设备要有**时间戳和离线判定**，控制环使用前先检查。

## 8. 互斥量、优先级反转与优先级继承

### 8.1 互斥量（Mutex）

保护**需要独占访问的资源**（例如一个共用的 SPI 或串口发送缓冲）。注意：**不能在中断里使用互斥量**；中断里用 `FromISR` 版本的信号量/通知/队列。

### 8.2 优先级反转（Priority Inversion）

场景（教学示例）：

- L：低优先级任务（如 UI），持有互斥量，在用串口发送；
- H：高优先级任务（如云台），也想用同一串口，被阻塞等待互斥量；
- M：中优先级任务（如通信解析），不需要该资源，却抢占了 L。

结果：H 被 M 间接阻塞（M 比 H 低，却把 L 挤走，L 释放不了互斥量）。阻塞时间不再可控，甚至无限。

### 8.3 FreeRTOS 的解决

FreeRTOS 的互斥量（`xSemaphoreCreateMutex`）支持**优先级继承**：当 H 等 L 持有的锁时，L 的优先级被临时提升到 H 的优先级，使 M 无法抢占它，L 尽快释放锁。**二值信号量没有优先级继承**，所以保护资源用互斥量，做事件通知才用信号量/通知。

### 8.4 设计上的更好做法

控制环（高优先级）尽量**不使用共享的、可能被低优先级任务长期占用的资源**：串口日志放到低优先级任务，通过队列提交；控制环只写队列且不阻塞（超时 0）。

## 9. Cortex-M 中断优先级与 FreeRTOS 的配合

STM32F4 有 4 位中断优先级（16 个级别）；**数值越小优先级越高**。FreeRTOS 定义了：

- `configKERNEL_INTERRUPT_PRIORITY`：内核（SysTick、PendSV）使用的最低优先级；
- `configMAX_SYSCALL_INTERRUPT_PRIORITY`：能调用 FreeRTOS `FromISR` API 的中断的**最高允许优先级**（数值上的下界）。

规则：

1. 调用 `xxxFromISR` 的中断，其优先级数值必须 **≥ configMAX_SYSCALL_INTERRUPT_PRIORITY 对应的数值**（即逻辑上不能比它更高）；否则内核可能被打断在临界操作中，导致随机崩溃；
2. 比这个阈值更高优先级的中断（数值更小）永远不会被 FreeRTOS 屏蔽，延迟最小，但**不能调用任何 FreeRTOS API**；
3. CubeMX 默认把外设中断设为优先级 5（对应常见配置），如果你手动改成 0～4 又在里面发通知，就会出现断言失败或 HardFault。

**典型症状**：偶发的 HardFault，调试器看到栈在 `vPortEnterCritical` 或队列操作里。

## 10. 栈、堆与常见崩溃

### 10.1 栈大小

每个任务有独立的栈（单位：字，4 字节）。使用 `float`、`printf`、`snprintf`、大局部数组、深递归的任务需要更大栈。查看余量：

```c
UBaseType_t left = uxTaskGetStackHighWaterMark(NULL);  /* 历史最小剩余字数 */
```

设置原则：运行一段时间覆盖所有分支后，余量保留 20%～30%。打开 `configCHECK_FOR_STACK_OVERFLOW = 2`，并实现 `vApplicationStackOverflowHook`（例如亮灯并停车）。

### 10.2 堆

`heap_4` 是常见选择（支持合并碎片）。任务、队列、信号量动态创建都会消耗堆。如果 `pvPortMalloc` 返回 NULL，创建失败（任务没跑起来）。检查 `xPortGetFreeHeapSize()`。

### 10.3 常见崩溃症状

| 症状 | 常见原因 |
|---|---|
| 启动后某任务不运行 | 堆不足导致任务创建失败 |
| 偶发复位 / HardFault | 栈溢出；`FromISR` 优先级配置错误；野指针 |
| 控制数据偶发尖峰 | 共享数据被中断改写；DMA 缓冲被覆盖 |
| 整体变慢、遥控延迟 | 高优先级任务忙等或死循环不让出 |

## 11. 简易调度分析：利用率与响应时间（手算）

### 11.1 CPU 利用率

任务 $i$ 的执行时间 $C_i$、周期 $T_i$，利用率 $U=\sum C_i/T_i$。

**教学示例（非项目数值）**：

| 任务 | $C$ (ms) | $T$ (ms) | $C/T$ |
|---|---|---|---|
| INS | 0.25 | 1 | 0.250 |
| Gimbal | 0.30 | 2 | 0.150 |
| Chassis | 0.20 | 2 | 0.100 |
| Comm | 0.15 | 5 | 0.030 |
| UI | 1.00 | 50 | 0.020 |

$U = 0.25+0.15+0.10+0.03+0.02 = 0.55$。

Liu & Layland 的速率单调（RM）充分条件：$U \le n(2^{1/n}-1)$。$n=5$ 时界为 $5(2^{0.2}-1) = 5\times 0.1487 = 0.7435$。$0.55<0.7435$，**可调度**（充分条件满足）。若 $U$ 超过该界但小于 1，仍可能可调度，需要做精确的响应时间分析。

### 11.2 最坏响应时间（Response Time Analysis）

对任务 $i$：

$$
R_i^{(0)} = C_i,\qquad R_i^{(k+1)} = C_i + \sum_{j \in hp(i)} \left\lceil \frac{R_i^{(k)}}{T_j} \right\rceil C_j
$$

迭代至收敛，其中 $hp(i)$ 是比 $i$ 优先级高的任务集合。设优先级：INS > Gimbal > Chassis。

- Gimbal：$R^{(0)}=0.30$；$R^{(1)}=0.30+\lceil 0.30/1\rceil\times 0.25=0.55$；$R^{(2)}=0.30+\lceil 0.55/1\rceil\times0.25=0.55$，收敛。**$R_{\text{Gimbal}}=0.55$ ms**。
- Chassis：$R^{(0)}=0.20$；$R^{(1)}=0.20+\lceil0.20/1\rceil 0.25+\lceil0.20/2\rceil 0.30=0.75$；$R^{(2)}=0.20+\lceil0.75/1\rceil 0.25+\lceil0.75/2\rceil 0.30=0.75$。**$R_{\text{Chassis}}=0.75$ ms**。

结论：在示例参数下，两个控制任务的最坏响应时间都小于它们的周期（2 ms），不会错过截止期。实际还应加上中断占用（CAN 接收中断、DMA 中断）带来的额外阻塞，以及 tick 量化（0～1 ms）。

> **面试要点**：能说出"I do a back-of-envelope utilization check and a response-time check; the real numbers come from measuring execution times with the DWT cycle counter"，就已经比大多数同龄人专业。

### 11.3 延迟对控制稳定性的影响

若控制回路总延迟（采样到命令生效）为 $T_d$，则在带宽 $\omega_c$ 处引入相位滞后 $\phi = \omega_c T_d$（弧度）。

**教学示例**：速度环希望带宽 $\omega_c=60$ rad/s，延迟 $T_d = 3$ ms ⇒ $\phi = 0.18$ rad $\approx 10.3^\circ$。若延迟变成 6 ms，则 $20.6^\circ$。相位裕度本来只有 45°～60° 时，这是不能忽略的。这就是"周期和延迟是控制参数的一部分"的定量说明。详见 [数字控制与实时性](../../02_基础知识/05_控制理论/08_数字控制与实时性.md)。

## 12. 测量：周期、抖动、延迟

### 12.1 GPIO 翻转 + 逻辑分析仪

```c
void gimbal_task(void *arg)
{
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        HAL_GPIO_WritePin(DBG_GPIO_Port, DBG_Pin, GPIO_PIN_SET);    /* 任务开始 */
        control_step();
        HAL_GPIO_WritePin(DBG_GPIO_Port, DBG_Pin, GPIO_PIN_RESET);  /* 任务结束 */
        vTaskDelayUntil(&last, pdMS_TO_TICKS(2));
    }
}
```

逻辑分析仪上：高电平宽度 = 执行时间（含被抢占时间）；相邻上升沿间隔 = 周期；多次测量的离散程度 = 抖动。

### 12.2 DWT 周期计数器（无需外设）

```c
static inline void dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}
static inline uint32_t dwt_cycles(void) { return DWT->CYCCNT; }
```

168 MHz 下 1 个周期约 5.95 ns；32 位计数约 25.5 s 回绕。用差值（无符号减法自动处理回绕）计算：

```c
uint32_t t0 = dwt_cycles();
control_step();
uint32_t dt_cycles = dwt_cycles() - t0;
float dt_us = dt_cycles / 168.0f;      /* 168 MHz => 168 cycles/us */
```

**教学示例**：测得 `dt_cycles = 50400` ⇒ $50400/168 = 300\ \mu s = 0.30$ ms。

### 12.3 记录相邻周期并统计

用环形缓冲记录每次任务开始时的 DWT 计数，离线（或通过调试串口低优先级发送）计算：平均周期、最大周期、最小周期、标准差。**不要在控制环内 printf**，打印本身会改变被测对象。

### 12.4 延迟测量

"反馈到命令"的延迟：在反馈到达（CAN 接收中断）时记录时间戳，在对应命令发出时再读时间，二者之差。也可以用 GPIO 在中断入口置位、在 CAN 发送完成清零，示波器看间隔。

## 13. 在本项目里怎么用（CV / 笔记线索 / 典型）

- **【CV】**：STM32F407/FreeRTOS + CAN，负责云台、发射、底盘控制回路。你**确实**在 FreeRTOS 上组织了多任务控制。
- **【笔记线索】**：任务有 gimbal、chassis、imu、detect、usb、decode、ui、led、cap 等；INS 任务由 DMA 通知唤醒，云台任务中同时调用发射控制；循环内多处 `vTaskDelay(1)`；SMC 中假设 $dt=0.001$ s；AHRS 采样频率常量为 1000 Hz；PID 离散实现中没有显式乘除 $\Delta t$。
- **【典型】**：通知唤醒 INS、`vTaskDelayUntil` 控制环、专职 CAN 发送任务、离线检测。
- **本项目取值**：任务循环约 3 ms（约 333 Hz），IMU 任务由 DMA/DRDY 通知唤醒、约 1 kHz；任务清单为 default/calibrate/chassis/gimbal/imu/detect/usb/decode/ui/led/cap；云台、USB、Decode 优先级较高，底盘较低；CAN 发送在帧间加 1 tick 延时；有 10 ms 周期的 detect 离线检测任务。

> **一个重要的"自洽性"提醒**：如果算法里写死了 $dt=0.001$（例如 SMC 的误差微分、AHRS 的积分步长），而实际周期是 3 ms，那么 $\dot e$ 会被**高估 3 倍**，积分角速度会**少积 2/3**。面试中如果你知道这一点并能说"我们后来用实测周期校准/改用实际 dt"，会非常加分；如果当时没有，就诚实说"这是我现在会改进的地方"。

## 14. 常见错误与故障现象

| # | 现象 | 原因 | 定位 | 处理 |
|---|---|---|---|---|
| 1 | 控制手感随改动忽好忽坏 | 任务周期变了但增益没重调 | 测周期 | 固定周期（`vTaskDelayUntil`），在固定周期下调参 |
| 2 | 姿态角速度积分偏小 | AHRS 步长假设 1 ms，实际更长 | 对比陀螺积分与已知转角 | 用实测 dt 或固定周期 |
| 3 | 视觉/遥控偶尔延迟尖峰 | 低优先级任务持锁或高优先级任务长时间运行 | 逻辑分析仪看任务占用 | 缩短临界区，拆分任务 |
| 4 | CAN 偶发丢命令帧 | 3 个发送邮箱被占满 | 检查 `HAL_CAN_AddTxMessage` 返回值 | 检查空闲邮箱、专职发送任务 |
| 5 | 偶发 HardFault | 栈溢出或 FromISR 优先级错误 | 栈余量、NVIC 优先级 | 增栈、改优先级 |
| 6 | IMU 数据周期性缺口 | 通知被合并（INS 处理太慢） | 统计通知计数 > 1 的次数 | 提高 INS 优先级、缩短处理 |
| 7 | 日志一打开控制就变差 | printf 阻塞 | 关闭日志对比 | 日志低优先级 + 队列 |

## 15. 面试怎么问、怎么答

**Q1. How fast did your control loops run?**
- 诚实回答：*"The tick was 1 ms, and I designed the loops for roughly millisecond-scale updates. I know the effective period was longer than 1 ms because of delays inside the loop, so I measured it with a GPIO toggle."*（如果你确实测过，补上数值；没测过就不要说你测过——说"I would measure it by ..."。）
- 追问 1：*What happens to a PID if the period changes?* → 若实现没有乘除 dt，则等效 Ki 与 Kd 随周期改变；要么固定周期，要么在公式中显式使用 dt。
- 追问 2：*What is the difference between vTaskDelay and vTaskDelayUntil?* → 相对 vs 绝对；见 3 节。
- 追问 3：*How would you reduce jitter?* → 提高控制任务优先级、缩短高优先级任务和中断、`vTaskDelayUntil`、避免关中断、用硬件定时器触发。

**Q2. How did you get IMU data into the control loop?**
- 英文：*"The IMU is read over SPI with DMA. When the transfer completes, an interrupt sends a task notification to the attitude task, which decodes the data, applies the bias and mounting correction, and updates the AHRS. The control task then reads the latest attitude and angular rate."*
- 追问：*What if the attitude task is slower than the IMU rate?* → 通知合并、丢样；用 `ulTaskNotifyTake(pdFALSE)` 数积压；提高优先级；缩短处理。

**Q3. What is priority inversion and have you seen it?**
- 英文：*"A low-priority task holds a lock needed by a high-priority task, and a medium-priority task preempts the low one, so the high one is blocked indirectly. FreeRTOS mutexes use priority inheritance to bound it. I avoid sharing locks between the control loops and low-priority tasks."*
- 若你没遇到过，别编；说"I know the failure mode and designed to avoid it"。

**Q4. How do you pass data safely from an ISR to a task?**
- 英文：*"For events, task notifications or semaphores; for streams, queues or a ring buffer. In the ISR I only copy data and notify, and the processing happens in the task. For shared structs I copy a snapshot in a short critical section or use double buffering with a sequence number."*

**Q5. Why not put everything in one big loop?**
- 英文：*"With different rates and event-driven pieces, a single loop makes the slowest part set everyone's timing. Separate tasks with priorities isolate that."*

**Q6. How do you size a task's stack?**
- 英文：*"Start generous, run all code paths, read the high-water mark, then trim leaving some margin, and enable the stack overflow hook."*

## 本章要点速记

1. 周期是控制参数的一部分；改周期等于改增益。
2. 优先级只决定同时就绪时谁先，不决定频率。
3. `vTaskDelay(n)` 是相对延时，唤醒对齐 tick 边界；实际睡眠在 $(n-1, n]$ tick。
4. `vTaskDelayUntil` 才适合固定周期控制环。
5. 一个循环里多处 `vTaskDelay(1)`，周期是几个 tick，不是 1 ms（本章 3 处例子 ≈ 3 ms）。
6. `osDelay(ms)` 是 CMSIS 毫秒接口，内部转 tick，最少 1 tick。
7. 中断里少做事：取数据 → 通知/入队 → `portYIELD_FROM_ISR`；通知可能合并丢样。
8. 多字段共享数据可能被中断拼接；用快照、双缓冲、序号、时间戳。
9. 互斥量有优先级继承，二值信号量没有；中断里不能用互斥量。
10. `FromISR` API 要求中断优先级不高于 `configMAX_SYSCALL_INTERRUPT_PRIORITY`。
11. 利用率 + 响应时间分析可手算；测量用 GPIO/DWT，不在环内打印。
12. 如果算法写死 $dt$，实际周期必须匹配，否则微分/积分按比例失真。

## 自测题

1. 任务在 t=0.7 ms（tick 边界在整数 ms）调用 `vTaskDelay(1)`，何时唤醒？睡了多久？
   *答：目标 tick 为当前+1，即 t=1.0 ms 唤醒，睡 0.3 ms。*
2. 循环内三处 `vTaskDelay(1)`，每段执行 0.3 ms，任务起点在 tick 边界，周期是多少？
   *答：3 ms（每段在 tick 内完成，各自等到下一个 tick）。*
3. `ulTaskNotifyTake(pdTRUE, …)` 与 `pdFALSE` 的区别及对丢样检测的意义？
   *要点：pdTRUE 清零（合并），pdFALSE 减 1（可计数积压）。*
4. 为什么保护共享串口要用互斥量而不是二值信号量？
   *要点：互斥量带优先级继承，避免优先级反转。*
5. 用 DWT 测得 `dt_cycles=33600` @168 MHz，是多少微秒？
   *答：33600/168 = 200 µs。*
6. 利用率示例：任务 (0.4,2)、(0.3,5)、(0.2,10)，用 RM 界判断可调度性。
   *答：U=0.2+0.06+0.02=0.28 < 3(2^{1/3}−1)=0.7798，可调度。*
7. 控制回路延迟 4 ms，带宽 50 rad/s，引入相位滞后？
   *答：0.2 rad ≈ 11.5°。*

## 相关链接

- 上一章：[02 整机电控架构与数据流](./02_整机电控架构与数据流.md)
- 下一章：[04 CAN 总线与电机反馈](./04_CAN总线与电机反馈.md)
- 基础：[FreeRTOS 实时系统](../../02_基础知识/11_嵌入式与通信/02_FreeRTOS实时系统.md)、[数字控制与实时性](../../02_基础知识/05_控制理论/08_数字控制与实时性.md)
- 八股：[并发多线程与实时系统八股](../../03_八股知识/01_计算机基础/07_并发多线程与实时系统八股.md)、[嵌入式与机电八股](../../03_八股知识/03_机器人学/06_嵌入式与机电八股.md)
- 学习笔记：[P3 本对话问答逐字稿](../../05_学习笔记归档/P3/P3_本对话问答逐字稿.md)（第 1～3 组讲 tick、osDelay、vTaskDelayUntil）
- 工程化：[20 工程化实践：频率周期与问题解决](./20_工程化实践_频率周期与问题解决.md)（任务周期、延迟预算、CPU 与 CAN 负载手算）
