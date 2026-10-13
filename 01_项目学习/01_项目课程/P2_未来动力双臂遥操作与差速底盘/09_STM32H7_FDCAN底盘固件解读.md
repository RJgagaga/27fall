# 09 STM32H7/FDCAN 底盘固件解读

> **一句话定位**：读懂仓库 `two_wheel_chassis/mini_car_self`（STM32H723 + 3 路 FDCAN）：任务怎么调度、遥控与自动模式怎么切、速度怎么变成轮命令、编码器反馈怎么回来、里程计怎么积分、哪里有隐患。所有结论来自源码，推算处会标注。

**本章学完你能：** 画出固件分层与启动流程；解释 CAN 帧 ID 与字段；逐行说出 `chassis_odom_calc`；指出 6 个读码发现的隐患。

**前置：** [08 差速底盘](./08_差速底盘运动学_轮速PID与里程计.md)；[CAN/FDCAN](../../02_基础知识/08_嵌入式与通信/03_CAN_FDCAN与电机通信.md)；[STM32 基础](../../02_基础知识/08_嵌入式与通信/01_STM32_时钟_GPIO_中断_DMA.md)。

## 目录
1. 硬件与分层　2. 启动流程　3. 调度器 EZ_RTOS　4. 模式状态机　5. 遥控输入（SBUS）　6. 速度分配　7. CAN 与电机协议　8. 上位机帧　9. PID 模块　10. 里程计逐行　11. 隐患清单　12. 面试问答　13. 速记/自测/链接

## 1. 硬件与分层
- 主控 STM32H723，开 I/D Cache（后又 `SCB_DisableDCache()`）；FDCAN1/2/3，UART5（SBUS）+DMA 空闲中断。
- `.ioc`：三路 FDCAN 标称波特率均为 **1 Mbit/s**（`CalculateBaudRateNominal=1000000`），预分频 1，TS1=59，TS2=20；UART5 波特率 100000（SBUS）。
- 分层：HAL → BSP（`bsp_fdcan.c`、`uart_bsp.c`）→ 驱动（`drv_8030.c` 轮驱动、`drv_dmmotor.c`/`Robstride01.c` 其他电机）→ 算法（`pid.c`、滤波、CRC）→ 应用（`controler_task.c`、`chassis_task.c`）→ 调度（`EZ_RTOS.c`）。
- 总线分工（源码）：FDCAN1 接两个轮驱动（节点 0x01、0x03）；FDCAN2 接上位机（收 0x050、发 0x051）。

## 2. 启动流程（`main.c`）
初始化外设 → `can_filter_init()` → `chassis_init()`（清速度、初始化 2 个 PID、绑定三个模式回调）→ 启动 UART5 DMA 空闲接收 → `rtos_add_task(controler_task)`、`rtos_add_task(chassis_task)` → 延时 150 ms 依次 `MotoEnable(0x01)`、`MotoEnable(0x03)` → `while(1) rtos_start()`。

## 3. 调度器 EZ_RTOS
自研协作式轮询：任务数组，`rtos_start` 循环调用 READY 的任务。注意读码发现（**推断，需上板验证**）：
- `task_delay` 用 `__builtin_return_address(0)` 与 `task_func`（函数入口）比较，返回地址落在函数体内，通常不相等 → 延时基本是空操作；
- `rtos_tick()` 在工程里未被调用（SysTick 只调 `HAL_IncTick`），即便进入 DELAY 也不会恢复。
所以两个任务实际上全速轮询，`task_delay(3)`/`(5)` 不能当作 3/5 ms 周期。里程计另用 `HAL_GetTick()` 做 5 ms 门限，不依赖它。

## 4. 模式状态机（面向对象风格 C）
`Chassis_Base { Control_Fun }` 函数指针；三个实例 PROTECT / REMOTER / AUTO。`Chassis_mode_switch()` 按 `ctrl_mode` 选择回调：
- PROTECT：速度清零，`MotoSendSyncFrame`，两个 PID 的 `iout=0`；
- REMOTER：`chassis_spd_distribution()` + `chassis_pid_calcu()`；
- AUTO：若 `host_online>5000` 速度清零，否则沿用；再分配与计算。
`Chassis_ctrl()`：PROTECT 时对节点 0x01/0x03 发 0 速度；否则发 `my_speed[i]`。

## 5. 遥控输入（SBUS）
`sbus_frame_parse`：帧头 0x0F、帧尾 buf[24]=0x00；11 位通道按位拆包，如 `ch[0]=(buf[1]|buf[2]<<8)&0x07FF`。通道中位 992，范围 192–1792（±800）。
- `ch[7]`（三段开关）：<972 → PROTECT，>1012 → AUTO，否则 REMOTER；`ch[7]==0`（未收到有效帧）→ PROTECT 并重启 DMA。
- 摇杆：偏离中位 ±20 以内视为死区；`vy=(ch1−992)×0.01`，`vw=(ch3−992)×0.01`（`vx` 读 ch0 但差速不使用）。满杆 800×0.01=8.0。
- 注意：`remoter.online` 被解析但模式判断没用它。

## 6. 速度分配
```c
void mini_car_calc(float vy, float vw, float speed[]){
  speed[0] = (vy + vw) * speed_pos;   // speed_pos = 4
  speed[1] = (vy - vw) * speed_pos; }
// chassis_spd_distribution:
wheel_spd_ref[0] =  1.5f * wheel_spd_input[0];
wheel_spd_ref[1] = -1.5f * wheel_spd_input[1];
```
这里"前进"是 `vy`。`(vy±vw)` 就是差速公式 $v\pm\omega L/2$ 的无量纲版本（$L/2$ 吸收进比例）。右轮取负是镜像安装的符号处理。**算例（源码尺度）**：满杆前进 vy=8 → 8×4×1.5=48，两轮 +48/−48；原地转 vw=8 → 48/48 同号（因第二个轮再取负）。单位是驱动器的速度单位（本项目按 rpm 理解，驱动器对象字典里速度对象的单位为 1 rpm）。限幅 `data_limit(…100.8…)` 被注释。

## 7. CAN 与电机协议
- 轮驱动按 **CANopen SDO** 风格：命令帧 ID=0x600+节点号。`MotoSetSpeed`：`0x2B`（写 2 字节）+ 厂商对象索引（节点 1 为 0x2318，节点 3 为 0x3318）+ int16 速度；`MotoEnable` 写 0x2100/0x3100=1。对象含义以驱动器手册为准（本项目取值：0x2318/0x3318 为目标速度，单位 rpm；0x2100/0x3100 为使能字）。
- 反馈：过滤器把 0x581（节点 1）送 FIFO0、0x583（节点 3）送 FIFO1；回调里 `moto[0].speed=int16(d[5]<<8|d[4])`，`moto[1].speed=-int16(...)`（反馈也取负），置 `online=1`。
- 发送头：标准 ID、`FDFormat=FD_CAN`、`BRS_ON`，而 `.ioc` 的 `FrameFormat=CLASSIC`；两者不一致时按经典帧行为，值得上板确认。
- 帧位数估算（经典 8 字节帧约 111 位，1 Mbit/s ≈ 0.11 ms）。

## 8. 上位机帧
- 0x050（上位机→板，FDCAN2）：`vx=int16(d1<<8|d0)`，`vy=int16(d3<<8|d2)`，`vw=int16(d5<<8|d4)`（小端），各限幅 ±800，`updown=d7`；AUTO 下 `vy=can_vy×0.01`（与摇杆同尺度）。
- 0x051（板→上位机）：`msg_to_Host` 把 x、y、yaw 乘 1000 取整成 3 个 int16，**大端**（高字节在前）。即单位 mm、mrad。
- 手算：x=1.234 m → 1234 → 0x04D2 → 字节 `04 D2`。int16 范围 ±32.767 m（隐患，见 11）。

## 9. PID 模块（`pid.c`，源自 DJI RM 框架）
结构含 p/i/d、`IntegralLimit`、`MaxOutput`、死区、梯形积分、变速积分、微分先行、微分滤波。`PID_struct_init(&pid, 200, 80, 5.0f, 0, 0, 0, 0, 0, Integral_Limit)`。快照中 `chassis_pid_calcu` 读取反馈 `moto[i].speed` 但 `pid_calc` 调用被注释，`speed_send = wheel_spd_ref`。PID 在你们实机上的实现层级见 README 自查表 #14。

## 10. 里程计逐行（`chassis_odom_calc`）
1. 两轮 `online` 都为真且首次：重置，记 `last_odom_ms`；
2. 距上次 ≥5 ms：`ds[i]=moto[i].speed×ratio`，`ratio=6.806784e-5`；
3. `v_linear=(ds0+ds1)/2`；`v_angular=(ds1−ds0)/L×0.5`，`L=0.375`；
4. 归一化 `odom.z` 到 [−π,π)；`odom_update`：`z −= dθ`；`θ_mid=z−dθ/2`；`dx_world=sin·dy_body`，`dy_world=cos·dy_body`（机体前进沿 y，故角度从 y 轴量起）。
**推算**：若反馈单位为 rpm、采样 5 ms，则 `ratio=2π/60×r×0.005` 对应 **r=0.130 m**（算得 0.12999999），很可能就是轮半径；`×0.5/L` 等价于轮距 0.75 m，或为标定系数。本项目取值：轮半径 $r=0.13$ m，轮距 0.75 m（$L=0.375$ 为半轮距），与这里的反推一致。
**局限**：用固定系数（隐含 dt=5 ms）而非实测时间间隔；速度积分而非位置差分。

## 11. 隐患清单（读码发现）
1. `rc_flag` 从未被写入，`host_online` 一路自增到 5000 后停；判据 `>5000` 永不成立 → 上位机失联不会触发零速；
2. AUTO 分支向 FDCAN1 发 ID 0x050 的 `tx_data`（未初始化）→ 总线污染；
3. `task_delay`/`rtos_tick` 失效（见 3）；
4. 里程计 int16 毫米溢出 ±32.767 m；
5. 遥控断连无独立 failsafe（`online` 未使用，除非 `ch[7]==0`）；
6. 速度限幅被注释，满杆指令直达驱动器。

## 12. 面试问答
**Q. Walk me through the chassis firmware.** 
> A cooperative scheduler runs two tasks. The control task reads the SBUS remote and selects protect, remote or auto mode; the chassis task maps forward and turn commands to the two wheel targets, sends CAN speed commands to the drivers, reads speed feedback in the CAN interrupt, integrates odometry every 5 ms and reports it to the host.

**Q. What risks did you find?** 
> The host-heartbeat counter never resets because the flag it watches is never updated, so a lost host would not trigger zero speed; and the odometry is packed in int16 millimetres, which overflows beyond 32 m. Both are fixable. I found both while reading the firmware, not from a failure on the robot.

## 速记
1. 1 Mbit/s；FDCAN1 轮、FDCAN2 上位机。2. 0x600+id 命令，0x581/583 反馈。3. 模式由 ch7 决定。4. 速度×4×1.5，右轮取负。5. 0x050 小端、0x051 大端 mm。6. 里程计固定 5 ms 系数。
## 自测
1. 满杆前进的 wheel_ref？（±48）2. 为何 `host_online` 不触发？ 3. x=1.234 m 的帧字节？（04 D2）4. ratio 反推 r？（0.13 m，需确认）5. task_delay 为什么可能无效？
## 链接
上一章 [08](./08_差速底盘运动学_轮速PID与里程计.md)；下一章 [10 ROS2-CAN](./10_ROS2_CAN执行链.md)；[12 源码导读](./12_源码导读.md)
