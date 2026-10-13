# 09 STM32H7/FDCAN 底盘固件解读

> **一句话定位**：逐段读懂仓库 `项目源码/VLAI/two_wheel_chassis/mini_car_self`（STM32H723 + 3 路 FDCAN）：上电后发生什么、两个任务怎么调度、遥控与自动模式怎么切、速度怎么变成两个轮的命令、编码器反馈怎么回来、里程计怎么积分并上报。每个结论都来自源码并标注文件与行号；凡是“推算”“推断”的地方会明说。本章是 CV 里 *wheel-speed control ... integrated wheel motion into odometry ... teleoperated and autonomous modes* 的固件级证据，也是整个文档里最适合“带着代码答面试”的一章。

**本章学完你能：**

- 画出固件分层图和上电到进入主循环的完整顺序，并算出时钟树（480 MHz / 240 MHz / FDCAN 80 MHz）与 1 Mbit/s 位时序；
- 解释自研调度器 `EZ_RTOS` 的工作方式，以及为什么 `task_delay` 在这份快照里很可能不起作用；
- 手工解包一帧 SBUS，说出三段开关如何决定 PROTECT / REMOTER / AUTO 三种模式；
- 手算“摇杆 → 轮速指令 → 驱动器 SDO 帧 → 实际车速”的整条换算链（满杆约 0.65 m/s）；
- 逐行讲解 `chassis_odom_calc` 与 `odom_update`，并用手算验证 `ratio` 与轮半径、轮距的关系；
- 列出读码发现的隐患，并清楚区分“读码发现”与“我的贡献”。

**前置：** [08 差速底盘运动学、轮速 PID 与里程计](./08_差速底盘运动学_轮速PID与里程计.md)；[CAN/FDCAN 与电机通信](../../02_基础知识/08_嵌入式与通信/03_CAN_FDCAN与电机通信.md)；[STM32 时钟、GPIO、中断、DMA](../../02_基础知识/08_嵌入式与通信/01_STM32_时钟_GPIO_中断_DMA.md)。

> **重要口径**：这份固件是团队已有的底盘 CAN 框架（见 [README 自查表 #18](./README.md)），我负责的是差速运动学、轮速 PID、里程计与 ROS 2 的集成与整定。下面“读码发现”的隐患是我读代码时看到的，**不是我修复的，也不是我的贡献**；面试时要如实说“I found this while reading the code”。

## 目录

1. 硬件、时钟与位时序
2. 分层与文件地图
3. 启动流程逐行
4. 调度器 EZ_RTOS
5. 中断与数据流：谁写、谁读
6. 模式状态机
7. 遥控输入：SBUS 解包
8. 速度分配与单位链
9. CAN 层：过滤器、发送头、SDO 帧、反馈
10. 上位机帧（0x050 / 0x051）
11. PID 模块与“符号链”问题
12. 里程计逐行与手算
13. 隐患清单（读码发现）
14. 面试问答
15. 速记 / 自测 / 链接

---

## 1. 硬件、时钟与位时序

### 1.1 它要解决什么问题

底盘板要做三件事：读遥控器或上位机的速度命令；把命令变成两个轮毂电机的速度目标并通过 CAN 发出；收轮速反馈、积分出里程计并回报上位机。主控选 STM32H723 的原因很实际：它自带三路 FDCAN（一路接轮驱动、一路接上位机、一路备用），带 FPU，主频高，写浮点里程计不吃力。

### 1.2 时钟树（从 `.ioc` 与 `main.c` 算出）

`SystemClock_Config`（`Core/Src/main.c` 第 150–202 行）：HSE=24 MHz（`stm32h7xx_hal_conf.h` 第 110 行 `HSE_VALUE=24000000`），PLL1：$M=2,\ N=40,\ P=1,\ Q=6,\ R=2$。

$$f_{VCO}=\frac{24\ \text{MHz}}{2}\times 40=480\ \text{MHz},\quad f_{SYS}=\frac{480}{P}=480\ \text{MHz}$$

AHB 分频 2（`RCC_HCLK_DIV2`），所以 HCLK = 240 MHz，各 APB 再分频 2 得 120 MHz。FDCAN 内核时钟取 PLL1Q：$480/6=80$ MHz（`.ioc` 第 262 行 `RCC.FDCANFreq_Value=80000000`，与计算一致）。

### 1.3 1 Mbit/s 位时序手算

`fdcan.c` 里三路 FDCAN 的标称段参数相同：预分频 $=1$，`NominalTimeSeg1=59`，`NominalTimeSeg2=20`，`NominalSyncJumpWidth=20`。

1. 时间量子 $t_q = 1/80\ \text{MHz}=12.5\ \text{ns}$（`.ioc` 的 `CalculateTimeQuantumNominal=12.5`）。
2. 一位的量子数 $=1\ (\text{Sync}) + 59 + 20 = 80$。
3. 位时间 $=80\times12.5\ \text{ns}=1\ \mu s$，即 **1 Mbit/s**。
4. 采样点 $=(1+59)/80=75\%$。这是 CAN 常用的 75%–87.5% 区间的下沿，容忍线上传播延迟。

`FrameFormat = FDCAN_FRAME_CLASSIC`（`fdcan.c` 第 51 行附近），也就是**经典 CAN 模式**。数据段参数（`DataTimeSeg1=13` 等）虽然存在，但在经典模式下不起作用。本项目的底盘总线按 **1 Mbit/s、经典 8 字节帧**使用，这一点和 [README 自查表 #16](./README.md) 一致。

### 1.4 为什么要写这些

面试官问“你们的 CAN 波特率怎么配的”时，能说出“80 MHz 内核时钟，80 个量子，75% 采样点”，比只说“1M”可信得多。反过来，波特率配错一端，总线上会满是错误帧，这是联调时最常见的第一类故障（见 [11 案例 14](./11_调试与故障案例.md)）。

---

## 2. 分层与文件地图

```text
应用层   User/app/controler_task.c   遥控解析、模式选择、写速度输入
         User/app/chassis_task.c     模式回调、速度分配、PID 位置、CAN 发送、里程计
调度层   User/rtos/EZ_RTOS.c         协作式轮询调度（最多 8 个任务）
算法层   User/math/pid.c             DJI RM 风格 PID（改进积分、微分先行等）
驱动层   User/drv/drv_8030.c         轮毂驱动器 SDO 命令（使能、速度、配置示例）
         User/drv/drv_dmmotor.c ...  其他电机驱动（本底盘主链路未使用）
BSP 层   User/bsp_fdcan.c            过滤器、发送、接收回调、上位机帧
         User/uart_bsp.c             SBUS 的 UART5+DMA 空闲中断接收与解包
HAL      Core/Src/*.c                CubeMX 生成：时钟、FDCAN、UART、DMA
```

总线分工（源码）：**FDCAN1** 接两个轮驱动（CANopen 风格节点号 0x01、0x03），**FDCAN2** 接上位机（收 0x050 速度命令，发 0x051 里程计），FDCAN3 配置了但本链路未用。

仓库里还有 `bmi088.c`、`KalmanFilter.c`、`Robstride01.c`、`motion.c`（CANopen 位置/速度/力矩模式的辅助函数）等，它们是这块控制板的通用库，**本底盘主链路没有调用它们**（`imu_temp_ctrl.c` 里 `BMI088_init` 也是注释状态）。读码时先找“活动路径”，不要被库文件的存在误导。

---

## 3. 启动流程逐行（`Core/Src/main.c`）

```c
SCB_EnableICache(); SCB_EnableDCache();      // 第 87、90 行：先开缓存
HAL_Init(); SystemClock_Config();            // 第 95、102 行
MX_GPIO_Init(); MX_DMA_Init();
MX_FDCAN1_Init(); MX_FDCAN2_Init(); MX_FDCAN3_Init(); MX_UART5_Init();
SCB_DisableDCache();                         // 第 116 行：又把 D-Cache 关掉
can_filter_init();                           // 配接收过滤器并启动 FDCAN1/2
chassis_init();                              // 清速度、初始化 2 个 PID、绑定 3 个模式回调
HAL_UARTEx_ReceiveToIdle_DMA(&huart5, rx_buff, BUFF_SIZE*2);   // 开 SBUS 接收
rtos_add_task(controler_task); rtos_add_task(chassis_task);
HAL_Delay(150); MotoEnable(&hfdcan1, 0x01);
HAL_Delay(150); MotoEnable(&hfdcan1, 0x03);
HAL_Delay(150);
while (1) { rtos_start(); }
```

逐点说明：

1. **先开后关 D-Cache**：在带 DMA 的 Cortex-M7 上，D-Cache 会让 DMA 写入的内存和 CPU 看到的不一致（需要手动 clean/invalidate）。把它关掉是最省事的规避办法，代价是性能下降，但对一个 5 ms 级的底盘任务完全够用。这是一个典型的“H7 + DMA 先关缓存”做法。
2. **`can_filter_init` 在 `chassis_init` 之前**：过滤器和中断先就位，之后才可能收到反馈帧。
3. **使能电机放在最后**，每次间隔 150 ms：驱动器上电后需要时间自检；一次发两个节点的使能帧，每个之间留出间隔，避免 TX 队列拥堵。使能对象是 SDO 写 `0x2100`（节点 1）/ `0x3100`（节点 3）= 1（见 9.3）。
4. `MotoSpeedConfigExample`（NMT 启动、设速度模式、加减速时间 1000 ms、配 TPDO）被注释掉了——说明驱动器的速度模式与反馈上报在**出厂/上位配置工具里已经设好**，板子只负责写目标速度和使能。
5. `Error_Handler` 是 `__disable_irq(); while(1);`——死循环。没有看门狗（IWDG）的启用代码，一旦进入这里系统就“冻住”，而轮驱动器会继续保持最后一个速度指令，直到自身的通信超时（若有）。这是 12 章“失联策略”要注意的点。

---

## 4. 调度器 EZ_RTOS

### 4.1 设计意图

`EZ_RTOS.c` 是 80 行的协作式调度器：最多 8 个任务，每个任务有状态（READY / DELAY）和延时计数。`rtos_start()` 是一个无限 `while(1)`，每一圈依次调用所有 READY 的任务函数；任务自己在末尾调用 `task_delay(n)`，意图是“我睡 n 个 tick”；`rtos_tick()` 每个 tick 把 DELAY 任务的计数减一，减到零回到 READY。

两个任务的意图周期：`controler_task` 末尾 `task_delay(5)`，`chassis_task` 末尾 `task_delay(3)`。

### 4.2 `task_delay` 为什么很可能失效（推断，需上板验证）

```c
void task_delay(uint32_t ticks) {
    for (int i = 0; i < task_count; i++)
        if (task_list[i].task_func == __builtin_return_address(0)) {   // 第 67 行
            task_list[i].delay_ticks = ticks; task_list[i].state = TASK_DELAY; return; }
}
```

它想用“调用者的返回地址”认出是哪个任务，但 `__builtin_return_address(0)` 是**调用点之后的那条指令地址**，落在 `chassis_task` 函数体**中间**，而 `task_func` 存的是函数**入口地址**，两者几乎不可能相等，于是 `if` 不成立、函数什么都不做（推断：延时是空操作）。

另一个问题：`rtos_tick()` 在整个工程里**没有被调用**（`stm32h7xx_it.c` 的 `SysTick_Handler` 只调用 `HAL_IncTick()`），所以即便某个任务进入了 DELAY 状态，也没人把它恢复成 READY——如果真的进入了就会永远停住。现在的“幸运”结果是它根本没进入 DELAY。

**后果**：两个任务实际上**全速轮询**，周期取决于一圈代码执行加上 CAN 发送被阻塞/丢弃的时间，而不是 3 ms / 5 ms。里程计另用 `HAL_GetTick()` 做 5 ms 门限（见 12 节），所以它不受影响；但 `chassis_task` 里每一圈都会发 CAN 命令。

### 4.3 手算：全速轮询会发生什么

一帧经典 8 字节标准帧约 111 位（见 9.5），1 Mbit/s 下 0.111 ms。`chassis_task` 在 REMOTER 模式下每圈向 FDCAN1 发 2 条 SDO 速度命令（`Chassis_ctrl` 里 `MotoSetSpeed` 节点 1 与 3），再往 FDCAN2 发 1 条里程计帧。若驱动器每条命令回一个反馈帧，FDCAN1 上每圈 4 帧 $\approx0.444$ ms。CPU 一圈只要几十微秒，远快于总线，于是 TX FIFO（`.ioc` 中 `TxFifoQueueElmtsNbr=8`）很快被填满，`HAL_FDCAN_AddMessageToTxFifoQ` 返回错误，而 `fdcanx_send_data` 只是 `return 1`，**调用者没有检查**——多余的命令被默默丢掉。

换句话说，在这份快照里，**命令的有效下发频率是被总线速率“限”出来的**（约每 0.2–0.5 ms 一条），而不是被调度器规定的。这不会让车跑不动（驱动器自带速度环，收到的永远是最新目标），但它说明了一个设计原则：**周期必须显式控制，不能靠“代码跑得快/慢”碰运气**。如果想要真正的 5 ms 周期，应当像里程计那样用 `HAL_GetTick()` 做门限，或者换成 FreeRTOS 的 `vTaskDelayUntil`。本项目在设计取值里（见 [README 自查表 #14](./README.md)）把板端速度环的采样节拍定为 5 ms（200 Hz），与里程计的 5 ms 门限一致；周期与抖动的更多讨论见 [20 工程化实践](./20_工程化实践_频率周期与问题解决.md)。

---

## 5. 中断与数据流：谁写、谁读

### 5.1 共享变量表

| 变量 | 写者 | 读者 | 说明 |
|---|---|---|---|
| `remoter.rc.ch[]` | UART5 空闲中断回调 `HAL_UARTEx_RxEventCallback` → `sbus_frame_parse` | `controler_task` | 遥控通道，11 位 |
| `ctrl_mode` | `controler_task` | `chassis_task` | 三种模式 |
| `chassis.spd_input.v{x,y,w}` | `controler_task`（REMOTER 摇杆；AUTO 取 `can_spd_input`） | `chassis_task` | 目标速度（无量纲） |
| `can_spd_input.v{x,y,w}`, `updown` | FDCAN2 FIFO1 回调 → `chassis_automode_msg_handle` | `controler_task` | 上位机命令 |
| `moto[0/1].speed, online` | FDCAN1 FIFO0/FIFO1 回调 | `chassis_task`（PID 反馈、里程计） | 轮速反馈 |
| `chassis.odom.{x,y,z}` | `chassis_task` | `msg_to_Host` | 里程计 |

### 5.2 中断优先级

`.ioc` 中 FDCAN1/2/3 的 IT0/IT1、UART5、DMA1_Stream0 的抢占优先级**全部为 0**，SysTick 为 15（最低）。同一抢占优先级的中断**不会互相嵌套**，所以两个 FIFO 回调共用全局缓冲 `rx_fifo0_data/rx_fifo1_data` 不会被对方打断（这是这份代码能工作的一个隐含前提）。主循环读取这些变量时没有关中断，但 16 位对齐的读写在 Cortex-M 上是原子的，所以单个 `int16` 不会读到“半个值”；而 `vx/vy/vw` 三个量先后写入，主循环可能读到“新 vx + 旧 vy”的混合，对低带宽底盘无实质影响，但在更严格的系统里应当用双缓冲或临界区。

### 5.3 UART5 + DMA 空闲中断

`.ioc`：UART5 波特率 100000、**偶校验、2 停止位、9 位字长**（含校验位），这正是 SBUS 的物理层。DMA 为 `DMA_NORMAL` 模式、优先级 LOW。流程：`HAL_UARTEx_ReceiveToIdle_DMA` 启动接收 → 一帧 25 字节收完线路空闲 → 触发 `HAL_UARTEx_RxEventCallback(huart, Size)` → 重新启动 DMA、解析。

注意 `uart_bsp.c` 第 39 行：先 **重启 DMA 再解析**，而解析读的就是 DMA 正在写的同一块 `rx_buff`——若新一帧恰好开始写入，会读到半新半旧的数据。SBUS 帧间隔典型约 7–14 ms，帧本身 3 ms，所以碰撞概率低，但不是零。更稳妥的做法是用双缓冲或先拷贝。另外 `BUFF_SIZE=25` 而启动时传入的长度是 `BUFF_SIZE*2=50`，`rx_buff` 数组只有 25 字节——如果线路上有噪声使一次接收超过 25 字节，DMA 会写出数组边界。这是读码发现的一个**缓冲区越界风险**（见 13 节）。

---

## 6. 模式状态机（面向对象风格的 C）

`chassis_task.h` 定义了“基类”`Chassis_Base { 函数指针 Control_Fun }` 和“派生类”`Chassis_Derived { Chassis_Base Base }`，三个实例 `Drv_PROTECT / Drv_REMOTER / Drv_AUTO` 在 `chassis_init` 里绑定各自的回调。`chassis_task` 每圈：

```c
Action_ptr = Chassis_mode_switch();   // 按 ctrl_mode 选实例
Action_ptr->Control_Fun();            // 调模式回调（多态）
Chassis_ctrl();                       // 发 CAN 速度命令
chassis_odom_calc();                  // 里程计
msg_to_Host(odom.x, odom.y, odom.z);  // 上报
task_delay(3);
```

三个回调（`chassis_task.c` 第 100–158 行）：

| 模式 | 回调做什么 |
|---|---|
| PROTECT | 速度输入清零；`disable_flag=0`；`MotoSendSyncFrame`（0x80 同步帧）；两个 PID 的 `iout=0`（清积分）。`Chassis_ctrl` 在该模式下对节点 1、3 发 0 速度。 |
| REMOTER | 速度输入沿用（`controler_task` 已经写好）；`chassis_spd_distribution()` → `chassis_pid_calcu()`。 |
| AUTO | 若 `host_online>5000` 则速度清零；构造 `tx_data`（未初始化！）并 `fdcanx_send_data(&hfdcan1,0x050,tx_data,8)`；更新心跳计数；再分配速度与计算。 |

模式切换时有两点值得学习：PROTECT 里**清积分**（防止切回时积分残留造成冲击）；`Chassis_mode_switch` 记录了 `last_ctrl_mode`，以便“切入遥控模式时初始化底盘模式”。缺点是 PROTECT→REMOTER 的瞬间速度输入沿用的是 `controler_task` 刚写的摇杆值——如果此时摇杆不在中位，车会立刻以该速度起步，没有软启动斜坡（见 13 节）。

---

## 7. 遥控输入：SBUS 解包

### 7.1 问题与帧结构

SBUS 一帧 25 字节：`buf[0]=0x0F` 头，`buf[1..22]` 共 22 字节装 16 个 11 位通道（16×11=176 位=22 字节），`buf[23]` 是标志位，`buf[24]=0x00` 尾。每个通道的数值范围 192–1792，中位 992，差 ±800。

### 7.2 手算解包（教学示例）

假设通道 0 和通道 1 都在中位 992。$992=0x3E0=011\,1110\,0000_2$。

通道 0 占用 `buf[1]` 的 8 位加 `buf[2]` 的低 3 位：低 8 位 `1110 0000`=0xE0，高 3 位 `011`=3。所以 `buf[1]=0xE0`，`buf[2]` 的低 3 位 `=0b011`。源码：

```c
remoter->rc.ch[0] = ((buf[1] | buf[2] << 8) & 0x07FF);
```

代入 $0xE0 \mid (0x03\ll8)=0x3E0$，与 0x07FF 相与得 992 ✓。

通道 1 占用 `buf[2]` 的高 5 位和 `buf[3]` 的低 6 位：

```c
remoter->rc.ch[1] = ((buf[2] >> 3 | buf[3] << 5) & 0x07FF);
```

若通道 1=992：低 5 位 $992\bmod32=0$，高 6 位 $992\gg5=31=0x1F$，所以 `buf[2]` 高 5 位为 0（`buf[2]=0x03`），`buf[3]` 低 6 位 `=0x1F`。验证：$(0x03\gg3) \mid (0x1F\ll5)=0\mid0x3E0=992$ ✓。若推满到 1792：$1792\gg5=56=0x38$，$0x38\ll5=0x700=1792$ ✓。

### 7.3 模式开关与死区（`controler_task.c`）

```c
ctrl_mode = (ch[7] < 972) ? PROTECT : (ch[7] > 1012) ? AUTO : REMOTER;   // 第 33-36 行
if (ch[7] == 0) { 重启 DMA 接收; ctrl_mode = PROTECT; }                   // 无有效帧
```

三段开关的三个位置分别对应约 192 / 992 / 1792：下位 → PROTECT，中位 → REMOTER，上位 → AUTO。±20 的窗口是对中位附近噪声的容忍。`ch[7]==0` 表示从未收到有效帧（`remoter` 全局初始为 0），这是唯一的“遥控断线”判定——注意：SBUS 帧里还有一个 `online`（`buf[23]==0x0C` 视为失联），解析了但**没有被使用**。

摇杆死区与缩放（REMOTER 模式）：

$$v_y=(ch_1-992)\times0.01,\quad v_w=(ch_3-992)\times0.01,\quad |ch-992|\le20\Rightarrow v=0$$

$v_x$ 读 `ch[0]` 但差速底盘不使用（没有横移自由度）。满杆偏差 800，所以 $v_y\in[-8,8]$。另外 `ch[4]` 用来控制 PA0/PA2 两个 GPIO（升降类辅助机构的上下，AUTO 模式由 `updown` 控制），与行走无关。

---

## 8. 速度分配与单位链

### 8.1 代码

```c
uint8_t speed_pos = 4;
void mini_car_calc(float vy, float vw, float speed[]) {         // 第 205-211 行
    speed[0] = (vy + vw) * speed_pos;
    speed[1] = (vy - vw) * speed_pos;
}
void chassis_spd_distribution(void) {                           // 第 160-167 行
    mini_car_calc(chassis.spd_input.vy, chassis.spd_input.vw, chassis.wheel_spd_input);
    chassis.wheel_spd_ref[0] =  1.5f * chassis.wheel_spd_input[0];
    chassis.wheel_spd_ref[1] = -1.5f * chassis.wheel_spd_input[1];   // 右轮镜像安装，取负
    // wheel_spd_ref = data_limit(..., 100.8f, -100.8f);  被注释：没有限幅
}
```

这就是 [08 章](./08_差速底盘运动学_轮速PID与里程计.md) 里 $v_{L,R}=v\mp\omega b/2$ 的“无量纲版本”：$v_y$ 对应前进，$v_w$ 对应转向，差速公式里的 $b/2$ 和单位换算被吸收进两个比例系数 $4$ 和 $1.5$。**前进量是 `vy`**，不是习惯上的 `vx`——这是因为这块板子的机体系定义成“前进沿 y 轴”（里程计也是，见 12 节）。

### 8.2 单位链（推算）

驱动器的目标速度单位按 README 自查表的约定是 **rpm**，轮半径 $r=0.13$ m。1 rpm 对应轮缘线速度

$$v=\frac{2\pi}{60}\,r=0.10472\times0.13=0.013614\ \text{m/s per rpm}$$

满杆前进：$v_y=8$ → 轮输入 $(8+0)\times4=32$ → 左轮 ref $=1.5\times32=48$，右轮 ref $=-1.5\times32=-48$（取负后右轮物理上同向转）。$48\ \text{rpm}\to48\times0.013614=0.653$ m/s。

满杆原地转：$v_w=8$ → 输入 $[32,-32]$ → ref $[48,\ +48]$（右轮输入取负后为 $+48$）。此时两个 ref 符号相同，但因为右轮是镜像安装，物理上两轮反向转，车原地旋转，角速度

$$\omega=\frac{2v}{b}=\frac{2\times0.653}{0.75}=1.74\ \text{rad/s}\approx100^\circ/\text{s}$$

上位机路径：`can_spd_input` 范围 ±800，`controler_task` 乘 `sc=0.01` 后与摇杆同尺度，所以上位机一个计数对应

$$0.01\times4\times1.5\times0.013614=8.17\times10^{-4}\ \text{m/s}\approx0.82\ \text{mm/s}$$

上位机若要 0.3 m/s，应发 $0.3/8.17\times10^{-4}\approx367$ 计数。**这个换算表是联调里最常查的一张表**——上位机和板子单位不一致是“车速度总是差一个固定比例”的第一嫌疑（见 [20 工程化实践](./20_工程化实践_频率周期与问题解决.md) 的联调问题）。

### 8.3 速度表（推算，教学数值）

| 输入 $(v_y, v_w)$ | 轮输入 | `wheel_spd_ref` | 物理含义 |
|---|---|---|---|
| (8, 0) | (32, 32) | (48, −48) | 直行 0.653 m/s |
| (4, 0) | (16, 16) | (24, −24) | 直行 0.327 m/s |
| (0, 8) | (32, −32) | (48, 48) | 原地右转 1.74 rad/s |
| (4, 2) | (24, 8) | (36, −12) | 前进 + 右弧线，左轮快 |
| (−4, 0) | (−16, −16) | (−24, 24) | 后退 |

**限幅缺失**：`data_limit(..., 100.8f, -100.8f)` 被注释，所以命令范围只靠输入端限制（摇杆 ±8，上位机 ±800×0.01=±8）。而注释里写的“电机转速最高到 8900”说明驱动器能转得比 48 rpm 快得多，这个速度只用了很小的一部分。

---

## 9. CAN 层

### 9.1 接收过滤器（`bsp_fdcan.c` 第 16–58 行）

对 FDCAN1：索引 0 把 ID `0x581` 送入 RX FIFO0，索引 1 把 `0x583` 送入 RX FIFO1，类型是 `FDCAN_FILTER_DUAL`（双 ID 过滤）。全局过滤 `HAL_FDCAN_ConfigGlobalFilter(..., FDCAN_REJECT, FDCAN_REJECT, ...)` 表示**不匹配过滤器的标准/扩展帧一律拒绝**。FDCAN2：0x050 → FIFO1，0x051 → FIFO0。然后 `ActivateNotification` 打开两个 FIFO 的新消息中断，`HAL_FDCAN_Start` 启动。

两个小问题（读码发现）：①`fdcan_filter` 是局部变量，`FilterID2` 没有赋值；DUAL 模式下它是第二个接收 ID，等于一个**随机 ID 也被接收**（栈上的残值）；后面回调里用 `switch(Identifier)` 再筛一次，所以实际无害，但不规范。②FDCAN2 的 0x051 过滤规则会接收“别人发来的 0x051”，但回调里只处理 `FDCAN1` 的 0x051，FDCAN2 上的 0x051 被忽略，这条规则等于没用。

### 9.2 发送头（`fdcanx_send_data`，第 71–89 行）

```c
pTxHeader.Identifier = id;  IdType = FDCAN_STANDARD_ID;  TxFrameType = FDCAN_DATA_FRAME;
pTxHeader.DataLength = len;  // 直接把字节数 8 赋给 DataLength
ErrorStateIndicator = FDCAN_ESI_ACTIVE;  BitRateSwitch = FDCAN_BRS_ON;  FDFormat = FDCAN_FD_CAN;
```

发送头写的是 `FD_CAN + BRS_ON`，但 `fdcan.c` 里 `FrameFormat=FDCAN_FRAME_CLASSIC`。HAL 在经典模式下会忽略头里的 FD/BRS 位，实际按**经典帧**发送。这类“头文件写 FD、初始化写 Classic”的不一致很常见（直接复制了别人的例程），值得上板用示波器/CAN 分析仪确认帧格式。本项目按经典 CAN 使用，所以结论是：**8 字节、1 Mbit/s、无速率切换**。另有一点要核对：HAL 的 `DataLength` 字段按 `FDCAN_DLC_BYTES_x` 编码，部分 HAL 版本该宏的值并不等于字节数，直接传 8 是否等价取决于所用 HAL 版本（本项目的板子上该写法能正常通信，所以所用版本是等价的；换 HAL 版本时要回归这一点）。`len>8` 时 `DataLength` 不被赋值（`if(len<=8)`），用到未初始化的栈值——调用者全部传 8，所以没触发。

### 9.3 轮驱动器 SDO 命令（`drv_8030.c`）

驱动器按类 CANopen 的 SDO 写命令：COB-ID $=0x600+$节点号；数据 8 字节 `[命令字, 索引低, 索引高, 子索引, 数据0..3]`。命令字 `0x2B` 表示写 2 字节。

**使能** `MotoEnable`（第 431–450 行）：节点 1 写对象 `0x2100`（字节 `00 21`），节点 3 写 `0x3100`（字节 `00 31`），值 1。帧：`ID=0x601`, `2B 00 21 00 01 00 00 00`。

**速度** `MotoSetSpeed`（第 452–469 行）：节点 1 对象 `0x2318`（字节 `18 23`），节点 3 对象 `0x3318`（`18 33`），数据 `int16` 小端。

手算 `MotoSetSpeed(node=1, +48)`：$48=0x0030$ → 帧 ID `0x601`，数据 `2B 18 23 00 30 00 00 00`。

手算 `MotoSetSpeed(node=3, −48)`：$-48$ 作为 `int16_t` 为 `0xFFD0`，函数参数是 `int16_t`，做 `>>16` 时先提升为 `int`（算术右移，符号扩展），于是字节 6、7 为 `0xFF`：帧 ID `0x603`，数据 `2B 18 33 00 D0 FF FF FF`。因为命令字 `0x2B` 只写 2 字节，驱动器只取 `D0 FF`，后两字节被忽略，结果正确。但这个“负数时高字节为 FF”的现象，在对照 CAN 抓包时会让人困惑，值得记住。

对象含义说明：按本项目的取值，`0x2318/0x3318` 为目标速度（单位 rpm），`0x2100/0x3100` 为使能字；具体对象字典以驱动器手册为准，驱动器内部的电流环、速度环（以及加减速斜坡）由驱动器自身负责，不是我们调的。

### 9.4 反馈接收与符号处理

```c
void spd0_get(void){ moto[0].speed =  (int16_t)(rx_fifo0_data[5]<<8 | rx_fifo0_data[4]); moto[0].online = 1; }
void spd1_get(void){ moto[1].speed = -(int16_t)(rx_fifo1_data[5]<<8 | rx_fifo1_data[4]); moto[1].online = 1; }
```

反馈帧 `0x581`（节点 1）、`0x583`（节点 3）的数据字节 4、5（小端）是轮速。**节点 3 的反馈也取了负**，使得“向前”时两个 `moto[i].speed` 都为正，后面里程计就可以直接用 `(ds0+ds1)/2` 求线速度。这是理解 11 节“符号链”问题的关键。`online` 一旦置 1 就**永不清零**——没有超时检测，驱动器掉线后板子仍然以为它在线。

### 9.5 总线负载估算（教学推算）

经典 8 字节标准帧位数：SOF 1 + ID 11 + RTR 1 + IDE 1 + r0 1 + DLC 4 + 数据 64 + CRC 15 + CRC 界定 1 + ACK 2 + EOF 7 + 帧间隔 3 = **111 位**（不含填充位），1 Mbit/s 下 0.111 ms。

若设计为 5 ms 一个控制周期：FDCAN1 上每周期 2 条速度命令 + 2 条反馈 = 4 帧 $=0.444$ ms，负载 $0.444/5=8.9\%$，充裕；FDCAN2 上每周期 1 条 0x051 里程计 + （上位机 0x050，若 100 Hz 则每 5 ms 摊 0.5 帧）≈ 1.5 帧 $=0.17$ ms，3.3%。远离 30%–50% 的常用上限，余量大。反过来，全速轮询（4.3 节）会把 FDCAN1 填满到接近 100%。

---

## 10. 上位机帧

### 10.1 0x050：上位机 → 底盘板（FDCAN2，经典帧 8 字节）

`chassis_automode_msg_handle`（第 183–196 行）：

| 字节 | 内容 |
|---|---|
| d0–d1 | $v_x$，`int16` **小端** |
| d2–d3 | $v_y$，`int16` 小端 |
| d4–d5 | $v_w$，`int16` 小端 |
| d6 | 保留 |
| d7 | `updown`（1/2 控制 PA0/PA2 辅助机构） |

接收后各自限幅到 ±800（防止上位机误发超大值）。`controler_task` 在 AUTO 模式把它乘 0.01 当摇杆值用。

手算：要发 $v_y=367$、$v_w=-100$：$367=0x016F$ → d2=`6F`, d3=`01`；$-100=0xFF9C$ → d4=`9C`, d5=`FF`。帧：`ID 0x050, data = 00 00 6F 01 9C FF 00 00`（$v_x=0$）。

### 10.2 0x051：底盘板 → 上位机（FDCAN2）

`msg_to_Host`（第 165–181 行）：x、y、yaw 各乘 1000 四舍五入成 `int16`（`lroundf`），**大端**（高字节在前），共 6 字节，单位 mm、mrad。

手算：$x=1.234$ m → 1234 → `0x04D2` → d0=`04`, d1=`D2`；$y=-0.5$ m → −500 → `0xFE0C` → d2=`FE`, d3=`0C`；yaw $=3.0$ rad → 3000 → `0x0BB8`。

**注意字节序不一致**：0x050 是小端，0x051 是大端。这是联调时常见的“量级对但符号/大小乱了”的来源，也是 [11 案例](./11_调试与故障案例.md) 之一。`int16` 范围 ±32767 mm，所以里程计 x、y 超过 ±32.767 m 会回绕（隐患 4）。

---

## 11. PID 模块与“符号链”问题

### 11.1 `pid.c` 的结构

来自大疆 RM 框架（文件头保留了 MIT 许可声明）。`PID_struct_init(&pid, maxout, intergral_limit, kp, ki, kd, deadband, A, B, improve)` 设置：输出上限、积分限幅、增益、死区和一组“改进”开关——梯形积分、变速积分、微分先行、微分滤波、积分限幅（`improve` 位）。`pid_calc(pid, get, set)` 的流程：算误差 → 死区判断 → 比例项 → 积分项（累加）→ 微分项 → 应用各改进 → 积分限幅 → 总输出限幅。它是**位置式**、没有显式乘 $dt$：增益绑定于调用周期，所以调用周期不稳定会直接改变有效增益——这是 4 节“周期必须显式控制”的另一个理由。

`chassis_init` 对两个轮各初始化：输出上限 200、积分限幅 80、$K_p=5.0$、$K_i=0$、$K_d=0$。本项目调试后取 $K_i=0.5$（[README 自查表 #14](./README.md)）。

### 11.2 快照里 PID 被旁路了

```c
void chassis_pid_calcu(void) {                                         // 第 169-177 行
  for (i = 0; i < 2; i++) {
    chassis.wheel_spd_fdb[i] = moto[i].speed;
//  chassis.speed_send[i] = pid_calc(&pid_chassis_spd[i], fdb[i], ref[i]);
    chassis.speed_send[i] = chassis.wheel_spd_ref[i];
  }
}
```

快照里 `pid_calc` 是注释状态，`speed_send` 直接等于 `ref`。这表示这一版固件把速度命令直接交给驱动器，由驱动器内部的速度环去跟踪，这在“先调驱动器内环、后加外层闭环”的开发阶段很常见。按 CV 与 README 自查表 #14，本项目的闭环版本是**取消这行注释、让板端 PID 以编码器 rpm 反馈参与**；两者的差异记在 README 的口径备忘里，面试按 CV 讲，并清楚说明“驱动器自带电流环与速度环，我们的板端 PID 是套在外面的一层”。

### 11.3 取消注释时必须先看懂的符号链（推断，需上板验证）

追踪右轮（节点 3）的符号，设车前进：

1. `wheel_spd_input[1]=+32`（前进为正）；
2. `wheel_spd_ref[1] = -1.5 × 32 = -48`（镜像安装取负）；
3. 发给驱动器的命令 $=-48$，驱动器反馈的原始 `raw` 与命令同号，即 $\approx-48$；
4. `spd1_get` 取负：`moto[1].speed = +48`；
5. `wheel_spd_fdb[1] = moto[1].speed = +48`。

于是 PID 看到 `set = ref = −48`、`get = fdb = +48`，误差 $=-48-48=-96$，**符号反了**：本该跟踪的是“−48 对 −48”，现在却是“−48 对 +48”。输出 $K_p\times(-96)=-480$，被限到 −200 后发给驱动器，电机更快地向负方向转，反馈（取负后）变得更大，形成**正反馈**，轮子会一路冲到输出上限。

**正确做法**（推断）：右轮的 PID 要么用原始（未取负）反馈 `raw`，要么把 `ref[1]` 取负的操作移到 PID **输出之后**（“PID 在正向坐标系里算，最后一步再按安装方向翻转”）。这是一个很典型的坐标/符号约定问题：**反馈、给定、输出必须在同一个符号约定里闭环**。本项目取值里的板端闭环就是按“先在统一正方向算 PID，最后一步取负”实现的（这样里程计也能继续使用取负后的 `moto.speed`）。这一节的分析是读码推断，面试时说“the sign convention has to be consistent between reference and feedback; mirrored wheels are a classic place to break it”即可。

### 11.4 手算：Kp=5 为什么没有发散（教学示例）

把驱动器内环简化成一阶滞后 $G(s)=1/(\tau s+1)$，取 $\tau=40$ ms（教学示例，非实测），采样 $T=5$ ms，板端 PID 的命令经 CAN 有一拍延迟。零阶保持离散化：$a=e^{-T/\tau}=0.8825$，对象 $\frac{1-a}{z-a}=\frac{0.1175}{z-a}$，再乘一拍延迟 $z^{-1}$。仅比例 $K_p=5$：

$$1+K_p\frac{0.1175}{z(z-a)}=0\ \Rightarrow\ z^2-0.8825z+0.5875=0$$

两根是共轭复根，模 $|z|=\sqrt{0.5875}=0.766<1$，稳定（衰减振荡）。若 $K_p$ 增大到 $K_p\ge1/0.1175=8.5$ 时 $|z|\ge1$，开始发散。所以 $K_p=5$ 在这个教学对象下有一定稳定裕度，但离边界不远——这也是调试时 $K_p$ 要“从小往大加”，而不是直接用别人板子上的数的原因。

---

## 12. 里程计逐行与手算

### 12.1 代码（`chassis_task.c` 第 223–317 行）

```c
const double ratio = 0.00006806784l;          // 每 rpm 每次采样走的米数
...
if (online_cnt == 2 && last_all_online == 0) { 清零 last*; last_odom_ms = HAL_GetTick(); }
last_all_online = (online_cnt == 2);
now_ms = HAL_GetTick();
if (last_all_online && (now_ms - last_odom_ms >= 5)) {
    float L = 0.375f;
    for i: ds[i] = moto[i].speed * ratio;                       // 本次采样每轮位移（米）
    float v_linear  = (ds[0] + ds[1]) / 2.0f;
    float v_angular = (ds[1] - ds[0]) / L * 0.5f;               // 即 (ds1-ds0)/0.75
    归一化 odom.z 到 [-pi, pi);
    odom_update(&odom, 0, v_linear, v_angular);
    last_odom_ms = now_ms;
}
void odom_update(odom, dx_body, dy_body, dtheta){
    odom->z -= dtheta;
    float theta_mid = odom->z - dtheta/2;
    dx_world =  cos(theta_mid)*dx_body + sin(theta_mid)*dy_body;
    dy_world = -sin(theta_mid)*dx_body + cos(theta_mid)*dy_body;
    odom->x += dx_world; odom->y += dy_world; }
```

### 12.2 逐步解释

1. **两个轮都在线才开始**：`online_cnt==2` 的上升沿清零并记下时间戳，保证第一次积分不会拿着“很久以前”的时间。
2. **5 ms 门限**：`HAL_GetTick()` 的分辨率是 1 ms，任务在门限满足的那一圈才积分，`last_odom_ms=now_ms` 之后重新计时，抖动不会累积。
3. **`ratio` 的含义（推算）**：速度反馈 `speed` 单位 rpm，一个采样周期 $T=5$ ms，轮半径 $r$：

$$\text{ratio}=\frac{2\pi}{60}\cdot r\cdot T=0.10472\times0.13\times0.005=6.806784\times10^{-5}\ \text{m/(rpm·采样)}$$

数值 $6.806784\times10^{-5}$ 与源码常量一致（Python 复算 $6.8067841\times10^{-5}$）。因此 **$r=0.13$ m、$T=5$ ms** 与 README 自查表 #12、#13 的取值完全吻合。这也是“用常量代替实测 $dt$”的代价：$T$ 被写死，任务节拍一旦偏离 5 ms，里程计就按比例失真。

4. **线速度与角速度**：$v=(ds_0+ds_1)/2$；$\Delta\theta=(ds_1-ds_0)/L\times0.5=(ds_1-ds_0)/0.75$。$0.75$ m 即轮距 $b$（固件里 $L=0.375$ 是半轮距，再乘 0.5 等价于除以 2 倍半轮距）。自查表 #12 的轮距 0.75 m 与此吻合。
5. **机体系**：前进沿机体 $y$ 轴（`dy_body=v_linear`），所以航向角 `z` 从 $y$ 轴量起：`dx_world=sinθ·dy_body`，`dy_world=cosθ·dy_body`。$\theta=0$ 时车头朝世界 $+y$。
6. **符号**：`odom->z -= dθ`。当右轮（`ds[1]`）比左轮快，$\Delta\theta>0$，`z` 减小；反之 `z` 增大。左轮快即向右转，`z` 增大——也就是 `z` 是**顺时针为正**的角度，且从 $y$ 轴量起（读下游用这个里程计时必须核对，否则 ROS 里的 $\text{yaw}$ 符号会反，见 10 章“时间与坐标一致性”）。

### 12.3 手算一：直行

左右轮都 $+48$ rpm，持续 1 s（200 个采样）：

$$ds=48\times6.806784\times10^{-5}=3.267\times10^{-3}\ \text{m},\quad s=200\times3.267\times10^{-3}=0.6535\ \text{m}$$

与 8.2 节 $0.653$ m/s 一致 ✓，$\Delta\theta=0$。

### 12.4 手算二：原地转

两轮反馈为 $+48$ 与 $-48$（取负之后左轮 $+48$、右轮实际 $-48$ 的物理含义；对 `moto` 而言 `moto[0]=+48`，`moto[1]=-48`）：$ds_0=+3.267\times10^{-3}$，$ds_1=-3.267\times10^{-3}$。

$$\Delta\theta=\frac{ds_1-ds_0}{0.75}=\frac{-6.534\times10^{-3}}{0.75}=-8.713\times10^{-3}\ \text{rad/采样}$$

每秒 200 次，$|\Delta\theta|\times200=1.743$ rad/s，与 8.2 节的 1.74 rad/s 一致 ✓；`z -= dθ` 使 `z` 以 1.743 rad/s 增大，转一圈 $2\pi/1.743=3.6$ s。

### 12.5 一个小的公式细节（读码发现）

`odom->z` 已经减去了 $\Delta\theta$ 变成 $z_{new}=z_{old}-\Delta\theta$，两次采样的中点应为 $(z_{old}+z_{new})/2=z_{new}+\Delta\theta/2$。而代码写的是 `theta_mid = z - dθ/2`，等于 $z_{old}-1.5\Delta\theta$，**与中点差了一个 $\Delta\theta$**（推断）。最大转速下 $|\Delta\theta|=8.7\times10^{-3}$ rad $\approx0.5^\circ$，只在转弯时造成不足 1° 的方向偏差，可以忽略；但它是一个“写公式时忘了符号随累加方向变化”的典型例子，面试可当作“符号约定”的例子提一下，同样注明是读码发现。

### 12.6 局限

- 速度积分而非位置差分：丢一帧反馈就丢一份位移；位置差分（读编码器累积计数）更抗丢帧；
- 常量 $T=5$ ms 而非实测 $dt$（见上）；
- 没有 IMU 融合，纯轮式里程计在打滑、轮径误差下会漂（本项目取值：直线 5 m 往返误差约 2%，原地转 10 圈累积航向误差约 30°，标定方法见 [08 章](./08_差速底盘运动学_轮速PID与里程计.md)）。

---

## 13. 隐患清单（读码发现，不是我的贡献）

| # | 隐患 | 位置 | 后果 | 确定性 |
|---|---|---|---|---|
| 1 | `rc_flag` 从未被写入，`host_online` 一路加到 5000 后停止，而判据是 `>5000`，永不成立 | `chassis_task.c` 130–158；`bsp_fdcan.h` | 上位机失联**不会**触发零速 | 读码确定 |
| 2 | AUTO 分支把**未初始化的局部数组** `tx_data` 当 0x050 帧发到 FDCAN1（命令总线） | `chassis_task.c` 143–146 | 污染轮驱动总线，可能被驱动器解释为无关帧 | 读码确定 |
| 3 | `task_delay` 用返回地址比对函数入口，大概率不匹配；`rtos_tick` 从不被调用 | `EZ_RTOS.c` 62–74；`stm32h7xx_it.c` | 周期不受控，任务全速轮询 | 推断，需上板验证 |
| 4 | 里程计 x/y 以 `int16` 毫米上报 | `bsp_fdcan.c` 160–181 | ±32.767 m 回绕 | 读码确定 |
| 5 | 遥控断连只检 `ch[7]==0`；`remoter.online` 解析了但未使用 | `controler_task.c`；`uart_bsp.c` | 遥控失联后保留最后一帧值，车继续按旧指令走 | 读码确定 |
| 6 | 速度限幅 `data_limit` 被注释 | `chassis_task.c` 166 | 指令幅度完全靠输入端保证 | 读码确定 |
| 7 | `rx_buff[25]` 却启动 DMA 长度 `BUFF_SIZE*2=50` | `uart_bsp.h`、`main.c` 119 | 异常长数据时越界写 | 读码确定（概率低） |
| 8 | `moto[].online` 置 1 后永不清零，反馈无超时 | `bsp_fdcan.c` 207–222 | 驱动器掉线不被发现 | 读码确定 |
| 9 | `fdcanx_send_data` 的返回值无人检查 | 全部调用点 | TX FIFO 满时命令被静默丢弃 | 读码确定 |
| 10 | 滤波器 `FilterID2` 未初始化；发送头 FD/BRS 与初始化 Classic 不一致 | `bsp_fdcan.c` | 可能多收无关 ID；帧格式需确认 | 读码确定 / 需示波器确认 |
| 11 | PROTECT→REMOTER 切换无软启动，PID 被旁路，速度直接给驱动器 | `chassis_task.c` | 摇杆不在中位切入时车突然起步 | 读码确定 |
| 12 | 若取消 `pid_calc` 注释，右轮 ref/fdb 符号不一致造成正反馈 | `chassis_task.c` 165、173–175 | 轮速冲到输出上限 | 推断 |
| 13 | 里程计 `theta_mid` 符号与累加方向不一致（差 $\Delta\theta$） | `chassis_task.c` 305 | 转弯时方向偏差 <1° | 推断 |
| 14 | 无 IWDG；`Error_Handler` 死循环 | `main.c` 212–221 | 固件卡死时轮驱动器按最后指令继续转，直到驱动器自身超时 | 读码确定 |

> **面试要点**：这是一张“我读代码时看到的风险清单”。正确的叙述是：“I reviewed the firmware and listed these risks; I did not change the vendor framework, but my host-side `/cmd_vel` timeout covers the first one.”（我复审了固件并列出风险；我没有改团队的底盘框架，但上位机侧的 `/cmd_vel` 300 ms 超时可以覆盖第 1 条。）

---

## 14. 面试问答

**Q1. Walk me through the chassis firmware.**

> The board is an STM32H7 with three FDCAN peripherals. A cooperative scheduler runs two tasks. The control task reads the SBUS remote through a UART with DMA and an idle-line interrupt, and a three-position switch selects protect, remote or auto mode. The chassis task maps forward speed and turn rate to two wheel targets, sends CAN speed commands to the wheel drivers, reads their speed feedback in the CAN interrupt, integrates odometry every five milliseconds and reports it to the host over a second CAN bus.

中文：STM32H7 三路 FDCAN；协作式调度两个任务；控制任务读 SBUS，三段开关选模式；底盘任务做速度分配、发 CAN 命令、在中断里收反馈、每 5 ms 积分里程计并经第二路 CAN 上报。

- *追问 L2：Why did you say "cooperative"?* The tasks are called from one loop and each runs to completion; there is no preemption and no priority. *（因为所有任务在一个循环里依次运行到结束，没有抢占和优先级。）*
- *追问 L3：What are the downsides?* A long task delays every other task, and the delay function in the snapshot probably does nothing, so periods are not guaranteed. In a new design I would use FreeRTOS with fixed-rate tasks. *（缺点是长任务拖慢其他任务，且快照中的延时函数很可能无效，周期没有保证；新设计我会用 FreeRTOS 固定周期任务。）*

**Q2. How do speed commands become wheel commands?**

> Forward speed and turn rate are mixed as sum and difference, then scaled by a constant to wheel rpm. One wheel is mounted mirrored, so its sign is flipped. With full stick the wheel target is 48 rpm, which with a 0.13 metre wheel radius is about 0.65 metres per second.

中文：前进与转向做和差混合，再乘系数得到轮 rpm；一侧轮镜像安装取负；满杆 48 rpm，轮半径 0.13 m 时约 0.65 m/s。

**Q3. How is odometry computed, and what are its limits?**

> Every five milliseconds I multiply each wheel's speed feedback by a constant that equals two pi over sixty times the wheel radius times the sample time, get the distance each wheel travelled, average them for forward distance, and divide their difference by the wheelbase for the heading change. Then I rotate the body displacement into the world frame. Limits: it integrates speed rather than position, so lost frames lose distance; the sample time is hard-coded; there is no IMU fusion, so slip and wheel-size errors accumulate.

中文：每 5 ms 用常量把轮速换成每轮位移，平均得前进距离，差分除轮距得航向增量，再旋转到世界系；局限：积分速度而非位置、dt 写死、无 IMU 融合。

**Q4. What risks did you find in the firmware?**

> Reading the code I found several. The host heartbeat flag is never updated, so the zero-speed rule for a lost host can never fire. The remote-lost detection only checks one channel being zero. And the odometry is packed as int16 millimetres, so it wraps beyond about 32 metres. I found them by reading, not from a failure on the robot, and I covered the first one on the host side with a 300 millisecond velocity timeout.

中文：心跳标志从未更新、遥控失联只查一个通道为零、里程计 int16 毫米会在约 32 m 回绕；这些是读码发现，我用上位机 300 ms 速度超时覆盖了第一条。

**Q5. Why mirrored wheels are a classic place to break a control loop.**

> Because the sign has to be consistent across reference, feedback and output. If I flip the sign of the reference but also flip the feedback, the controller compares numbers of opposite sign and the loop becomes positive feedback. The fix is to run the PID in one positive-forward convention and flip the sign only at the very end.

中文：给定、反馈、输出的符号必须一致；若给定取负、反馈又取负，控制器比较的是反号的两个数，变成正反馈；应在统一的正向约定下算 PID，最后一步再翻转。

**Q6. How did you verify the CAN bit timing?**

> The kernel clock is 80 megahertz and the nominal bit has 80 time quanta, 1 plus 59 plus 20, so one bit lasts one microsecond, with a 75 percent sample point. A quick way to confirm it on the bench is to watch the bus with a CAN analyser and check that the error counters stay at zero.

中文：内核 80 MHz，一位 80 个量子，位时间 1 µs，采样点 75%；台架上的确认办法：用 CAN 分析仪看总线，错误计数应保持为零。

---

## 15. 速记 / 自测 / 链接

### 要点速记

1. 480 MHz 系统时钟，HCLK 240 MHz，FDCAN 内核 80 MHz；位时序 1+59+20=80 量子 = 1 µs，经典 CAN、8 字节帧。
2. FDCAN1 接轮驱动（COB-ID 0x600+节点命令，0x581/0x583 反馈），FDCAN2 接上位机（0x050 小端收，0x051 大端发，mm/mrad）。
3. 调度器是协作式轮询；`task_delay` 用返回地址比对入口很可能失效，周期要自己用 `HAL_GetTick` 门限控制。
4. 模式由 `ch[7]` 三段开关决定：<972 PROTECT、>1012 AUTO、其余 REMOTER；`ch[7]==0` 视为无信号 → PROTECT。
5. 速度链：摇杆 ±8 → ×4 → ×1.5 → rpm；满杆 48 rpm ≈ 0.653 m/s（r=0.13 m）；原地转 ≈ 1.74 rad/s（轮距 0.75 m）。
6. 里程计 `ratio = 2π/60·r·0.005 = 6.806784e-5`，固定 5 ms；`(ds1−ds0)/0.75` 得角度增量；前进沿机体 y 轴。
7. 快照里轮速 PID 被旁路；闭环必须统一反馈/给定/输出的符号约定，镜像轮最易出错。
8. 读码隐患（心跳失效、未初始化帧、int16 溢出、无 failsafe 等）是“读码发现”，不是我的贡献。

### 自测题

1. 算一下：$M=2,N=40,P=1$，HSE 24 MHz，SYSCLK 与 HCLK 各多少？（480 MHz；240 MHz）
2. 位时序 `1+59+20`、预分频 1、内核 80 MHz，波特率与采样点？（1 Mbit/s，75%）
3. 满杆前进时 `wheel_spd_ref` 是多少？为什么右轮是负的？（(+48, −48)；镜像安装符号翻转）
4. `MotoSetSpeed(node 3, −48)` 的数据字节？（`2B 18 33 00 D0 FF FF FF`，ID 0x603）
5. x=1.234 m 的 0x051 前两字节？（`04 D2`，大端）
6. 为什么 `host_online` 的零速判据永不触发？（`rc_flag` 从未被写入，计数停在 5000，条件 `>5000`）
7. 为什么说 `task_delay` 很可能是空操作？（返回地址在函数体内，与入口地址不相等；且 `rtos_tick` 从未被调用）
8. 若取消 `pid_calc` 注释，右轮的符号为什么会出问题？（ref 取了负，fdb 又被 `spd1_get` 取了负，误差符号错，正反馈）

### 相关链接

上一章 [08 差速底盘运动学](./08_差速底盘运动学_轮速PID与里程计.md) · 下一章 [10 ROS2-CAN 执行链](./10_ROS2_CAN执行链.md) · [11 调试与故障案例](./11_调试与故障案例.md) · [12 源码导读](./12_源码导读.md) · [20 工程化实践：频率周期与问题解决](./20_工程化实践_频率周期与问题解决.md) · [99 英文面试口述](./99_英文面试口述与深挖问答.md)
