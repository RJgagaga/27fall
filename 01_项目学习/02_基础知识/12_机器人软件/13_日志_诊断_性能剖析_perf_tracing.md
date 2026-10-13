# 13 日志、诊断、性能剖析与 tracing：让机器人"看得见、说得清、算得准"

> **一句话定位**：机器人出问题时，现场通常只有三句话："它抖了一下""它停了""它变慢了"。本章教你把这三句话变成证据：用**结构化日志**回答"发生了什么"，用**诊断（diagnostics）**回答"哪个部件处于什么健康状态"，用**性能剖析（perf、火焰图）和 tracing**回答"时间花在哪、延迟从哪来"，最后用**延迟预算**把它们串成一套可以复现、可以量化的排查方法。

**本章学完你能：**

- 区分日志级别的用途，用节流（throttle）、一次性（once）输出避免日志洪水，并估算日志的数据量；
- 设计诊断信息的结构：哪些量进诊断、阈值怎么定、OK/WARN/ERROR/STALE 各代表什么；
- 用 `perf record` + 火焰图定位 CPU 热点，说明采样剖析的统计误差（手算），并理解为什么要保留栈帧指针；
- 解释 tracing 与采样剖析的区别，说明 `ros2_tracing`（基于 LTTng）能回答什么问题；
- 做延迟预算：把端到端延迟拆成各级，**用分位数而不是平均值**，手算"各级 p99 之和"与"总延迟 p99"为什么不相等；
- 用阿姆达尔定律估算优化某一部分的收益上限，避免"优化了不该优化的地方"；
- 走一遍性能排查案例：从"变慢了"到定位根因的步骤；
- 诚实说明自己项目里的排查实践和延伸知识。

**前置知识：**

- [ROS2 Humble 核心概念与 ROS1 对比](./04_ROS2_Humble核心概念与ROS1对比.md)（话题、定时器、执行器）
- [ROS2 Humble 实战：launch、参数、生命周期、组件](./08_ROS2_Humble实战_launch_参数_生命周期_组件.md)（QoS、组件）
- [Linux 与 Shell、Git](./01_Linux与Shell_Git.md)（进程、线程、`top`）
- [概率统计与高斯分布](../01_数学基础/03_概率统计与高斯分布.md)（分位数、置信区间）

> **关于代码与命令的说明**：本机没有 ROS2、perf、LTTng，所以涉及它们的命令**未在目标环境运行**，只作结构示意；具体选项在 Humble 与所用内核版本下的行为凡不确定处标"未核对"。**纯 Python 的部分已在本机运行复核**（NumPy 模拟延迟分布、`cProfile` 剖析一个小流水线、折叠栈汇总、采样误差与日志数据量的算术），文中标"Python 复核"。所有数值是**教学示例（非项目实测）**。

**目录**

1. 为什么要把"观测"当作系统的一部分
2. 日志：级别、节流与结构化
3. 诊断：把健康状态做成数据
4. 性能剖析：perf 与火焰图
5. tracing：事件级的时间线
6. 延迟预算：分位数与不可加性
7. 优化的收益上限：阿姆达尔定律
8. 排查案例："变慢了"怎么查
9. 在七个项目里怎么用（含诚实边界）
10. 常见错误与故障现象
11. 面试怎么问、怎么答
12. 本章要点速记
13. 自测题
14. 相关链接

---

## 1. 为什么要把"观测"当作系统的一部分

### 1.1 要解决的问题

调试机器人和调试普通程序有三点不同：

1. **不可暂停**：电机在转、车在动，不能随便打断点停住；
2. **时间敏感**：问题常常只出现在特定的时序下，加一行 `print` 改变了时序，问题就"消失"；
3. **现场不可重复**：场地、人员、电池都是一次性的，错过了就再也抓不到。

所以，观测能力必须**事先设计好**，而不是出事后再加。一个好的做法是把"观测"分三个层次：

| 层次 | 回答的问题 | 工具 |
|---|---|---|
| 事件 | 发生了什么、什么时候 | 日志（logging） |
| 状态 | 现在各部件健不健康 | 诊断（diagnostics）、状态话题 |
| 性能 | 时间花在哪、延迟从哪来 | 采样剖析（perf）、tracing |

### 1.2 观测本身的成本

观测不是免费的：日志写磁盘占 I/O 和 CPU，诊断占带宽，tracing 占 CPU。**观测的开销不能改变被观测的行为**，否则就是"观察者效应"。所以每种工具都要问：开销多少？能否动态开关？对实时线程有没有影响？

> **面试要点**：谈观测时，同时说"能发现什么"和"代价是什么"，显得有工程经验。

---

## 2. 日志：级别、节流与结构化

### 2.1 级别

ROS2 的日志级别（由低到高）：DEBUG、INFO、WARN、ERROR、FATAL。建议的用法：

| 级别 | 用途 | 例子 |
|---|---|---|
| DEBUG | 开发时的细节，默认关闭 | 每次回调的中间量 |
| INFO | 正常的关键事件（低频） | "节点已激活""收到新目标" |
| WARN | 异常但可继续 | "目标已过期 120 ms，使用上一拍" |
| ERROR | 功能受损，需要关注 | "CAN 发送失败" |
| FATAL | 无法继续，即将退出 | "配置缺失，无法启动" |

**判断标准**：WARN 之上的日志应该"有人要看"。如果每秒刷 100 条 WARN，等于没有 WARN。

### 2.2 日志洪水与节流

高频回调里直接 `RCLCPP_INFO` 会产生日志洪水：既占 CPU（格式化字符串、写终端/文件），又把真正重要的信息淹没。

**手算数据量（示例，非项目实测）**：每条日志约 200 字节，一个 100 Hz 的回调每次打印一条，10 个这样的节点：

$$
200\ \text{B}\times100\ \text{Hz}\times10=2\times10^5\ \text{B/s}=200\ \text{kB/s}.
$$

一天 $2\times10^5\times86400=1.728\times10^{10}$ B $\approx17{,}280$ MB（约 17 GB）。若节流到 1 Hz：$200\times1\times10\times86400/10^6=172.8$ MB/天（Python 复核）。差了两个数量级。

**节流的手段**：

```cpp
// C++（rclcpp，结构示意，未在本机运行）
RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/, "state ok, v=%.2f", v);
RCLCPP_WARN_ONCE(get_logger(), "parameter v_max not set, using default");
RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "command age %d ms", age_ms);
```

```python
# Python（rclpy，结构示意）
self.get_logger().info('state ok', throttle_duration_sec=1.0)
self.get_logger().warn('param missing', once=True)
```

要点：

- 节流使用的是**节点时钟**。在仿真/回放里使用仿真时间，回放暂停时节流计时也会暂停；
- 一次性（once）适合"启动时只说一次"的信息；
- 对**状态变化**打日志，而不是对状态本身持续打——"从 OK 变为 WARN"一条，比"仍然 WARN"一千条有用。

### 2.3 日志的内容：写给三个月后的自己

一条好日志包含：**什么事件、哪个对象、关键数值、与阈值的关系**。

| 差 | 好 |
|---|---|
| `error occurred` | `can_send failed: id=0x005, errno=105 (no buffer space), retry=2/3` |
| `timeout` | `cmd_vel age 230 ms > limit 200 ms, holding last command for 1 cycle` |
| `value wrong` | `imu accel norm 14.9 m/s^2 outside [8.8, 10.8], frame dropped` |

结构化的意思是：**固定格式的键值**，便于 `grep`、脚本解析。例如统一用 `key=value` 形式，并保持字段顺序，之后可以用一行命令统计某种事件的次数：

```bash
grep "cmd_vel age" robot.log | wc -l
```

### 2.4 控制日志级别与格式

```bash
# 运行时设置某个节点的日志级别（Humble，ros-args 方式）
ros2 run demo_chassis speed_limiter --ros-args --log-level speed_limiter:=debug

# 环境变量调整输出格式（具体占位符以所用版本为准，未核对）
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{time}] [{name}]: {message}"
```

日志默认也会写入文件（默认位于用户目录下的 `.ros/log`，可用 `ROS_LOG_DIR` 改变，**路径行为未核对**），并发布到 `/rosout` 话题（类型 `rcl_interfaces/msg/Log`），所以也能在别的节点或工具中订阅。注意：`/rosout` 在节点数多、日志频繁时本身会成为带宽负担。

### 2.5 日志不能替代的东西

- 日志是离散事件，**不适合记录连续信号**（如 1 kHz 的电流）。连续信号用 rosbag 录制或环形缓冲区（见 [第 09 章](./09_ROS2测试与CI_launch_testing_gtest_rosbag回放.md)）；
- 实时线程里不要直接打日志：格式化与 I/O 有不确定延迟。常见做法是把日志事件放入无锁队列，由后台线程输出。

---

## 3. 诊断：把健康状态做成数据

### 3.1 与日志的区别

日志是"事件流"，要靠人去读；诊断是"状态量"，设计成**机器可聚合、可显示、可告警**的数据。ROS 的诊断体系：各节点发布 `DiagnosticStatus`（名称、级别、消息、键值对）到 `/diagnostics`，可以由聚合器（aggregator）按层级汇总，再由监控界面显示（具体包名与用法在 Humble 上**未核对**）。

### 3.2 四个级别

| 级别 | 值 | 含义 |
|---|---|---|
| OK | 0 | 正常 |
| WARN | 1 | 降级，仍可工作 |
| ERROR | 2 | 故障，功能受损 |
| STALE | 3 | 状态过期：很久没有收到更新 |

**STALE 很重要**：节点崩溃时，它不会再发"我 ERROR 了"。如果监控只看最后一条消息，会误以为一切正常。STALE 通过**超时**判定：超过若干个预期周期没收到某个诊断，就标为 STALE。这和 [P1](../../01_项目课程/P1_AIRS球形机器人平衡控制/08_ROS_CAN安全执行闭环.md)、[P2](../../01_项目课程/P2_未来动力双臂遥操作与差速底盘/10_ROS2_CAN执行链.md) 里"数据有效性年龄检查"的思想一致：沉默本身就是一种故障。

### 3.3 应该放哪些诊断量

| 部件 | 键值 | 阈值思路（示例） |
|---|---|---|
| 传感器 | 实际频率 / 标称频率；最后一帧年龄；丢帧计数 | 频率低于标称 90% 为 WARN；年龄超过 3 个周期为 ERROR |
| 通信（CAN） | 错误帧计数、总线负载、发送失败次数 | 错误计数增长速率；负载超过某比例 |
| 电机 | 温度、电流、故障码、使能状态 | 温度分级 |
| 控制环 | 周期抖动、最大计算时间、命令年龄 | 计算时间超过周期的 70% 为 WARN |
| 系统 | CPU、内存、温度、磁盘 | 分级阈值 |

阈值要**有来源**：来自规格书（电机最高温度）、来自设计预算（周期的 70%）、来自基线测量（正常运行的 p99）。拍脑袋的阈值要么天天告警（被忽略），要么永不告警。

### 3.4 避免告警疲劳

1. **迟滞**：WARN 触发阈值与恢复阈值不同（如 80 ℃ 触发、75 ℃ 恢复），避免在阈值附近反复切换；
2. **去抖**：连续 $n$ 次超限才升级，与 [P1 跌倒检测的去抖](../../01_项目课程/P1_AIRS球形机器人平衡控制/08_ROS_CAN安全执行闭环.md)同一思路；
3. **分层**：底层部件的多个 WARN 汇总为一个高层状态（如"底盘：降级"），操作员看高层，工程师展开看细节。

### 3.5 诊断量作为"安全输入"

诊断不仅给人看，也可以反馈到系统：比如"定位协方差过大"→ 限速；"电机温度 WARN"→ 降低最大力矩。这时诊断就进入了**控制决策链**，必须像其他安全量一样考虑延迟、去抖、失效安全。

---

## 4. 性能剖析：perf 与火焰图

### 4.1 问题

"控制节点 CPU 占用 80%"——这句话没有行动价值。需要知道 **CPU 时间花在哪些函数上**。采样剖析（sampling profiler）的原理：以固定频率（如 99 Hz）中断程序，记录当时的调用栈，运行一段时间后统计每个函数出现的次数，占比近似于该函数占用的 CPU 时间比例。

### 4.2 命令（Linux，结构示意，未在本机运行）

```bash
# 对正在运行的进程采样 10 秒，采样频率 99 Hz，记录调用栈
sudo perf record -F 99 -g -p <PID> -- sleep 10
# 文本报告
sudo perf report --stdio | head -50
# 生成火焰图（需要 FlameGraph 脚本，来源与版本未核对）
sudo perf script | stackcollapse-perf.pl | flamegraph.pl > flame.svg
```

为什么采样频率用 99 而不是 100：避免与周期为 10 ms 的定时任务"同步"，那样每次采样都落在同一相位，得到有偏的结果。

**调用栈要可读，编译时需要**：

- 保留栈帧指针：`-fno-omit-frame-pointer`；或者使用 `perf record --call-graph dwarf`（开销更大，需要调试信息 `-g`）；
- 但**要用 Release 优化后的二进制**来剖析（`-O2` 加 `-g`，即 `RelWithDebInfo`），Debug 构建的热点分布与实际运行不同，会误导优化方向。

### 4.3 读火焰图

```text
火焰图（示意）：横轴 = 占样本的比例（不是时间顺序！），纵轴 = 调用深度

|------------------------------ main (100%) -----------------------------|
|--------------------------- ctrl_loop (100%) --------------------------|
| read_state 30% | ----------- compute_mpc 60% ----------- | publish 10% |
|                | build_matrices 15% |  qp_solve 45%     |
```

读法：

1. **宽度即占比**：平顶宽的函数是自己消耗时间的热点；
2. 从底向上是调用关系，下面的调用上面的；
3. 横向顺序没有意义（按字母排序），不要解读为时间先后；
4. 找"又宽又平"的顶部，那是自身耗时（self time）。

**折叠栈汇总（Python 复核）**：把每行"调用栈 样本数"汇总（4 条栈，样本数 30/45/15/10）：`read_state` 占 0.30、`compute_mpc` 占 0.60（其中 `qp_solve` 0.45、`build_matrices` 0.15）、`publish` 占 0.10。这就是火焰图背后的数据：按栈前缀累加样本数。

### 4.4 采样剖析的统计误差（手算）

以 99 Hz 采样 10 s，共 $n=990$ 个样本。某函数真实占比为 $p$，则估计占比的标准误为 $\sqrt{p(1-p)/n}$：

| 真实占比 $p$ | 标准误 | 95% 误差范围（约） |
|---|---|---|
| 5% | 0.69% | ±1.4% |
| 20% | 1.27% | ±2.5% |
| 50% | 1.59% | ±3.1% |

（Python 复核。）要把 $p=0.2$ 的估计误差压到 ±1%（95%），需要 $n\ge1.96^2\times0.2\times0.8/0.01^2\approx6147$ 个样本，约 62 s 的 99 Hz 采样。启示：

- 采样剖析**擅长找大头**（占比 > 10% 的函数），不适合比较两个占比 2% 与 3% 的函数；
- 想看更小的差别就延长采样时间，或换成精确计数（tracing / 插桩）；
- 采样要在**有代表性的负载**下做，空载下剖析没有意义。

### 4.5 不只是 CPU：几类相关的工具

| 现象 | 工具 | 说明 |
|---|---|---|
| 哪个线程占 CPU | `top -H -p <PID>`，`pidstat -t` | 看线程级占用 |
| 线程是否被抢占/迁移 | `perf sched` 等（用法未核对） | 抖动的常见原因 |
| 内存增长 | `/proc/<PID>/status`，`valgrind`/ASan（开销大） | 泄漏 |
| I/O 等待 | `iostat`、`iotop` | 日志写盘卡顿 |
| 锁竞争 | `perf` 锁相关事件，或 tracing | 回调组互斥 |
| 网络 | `ros2 topic bw`、`ros2 topic hz`、`ip -s link` | 带宽、丢包 |

Python 节点可以用标准库的 `cProfile` 做函数级剖析（精确计数，开销大于采样）。**Python 复核示例**：一个小流水线循环 40 次，每次调用 `parse`（列表推导）和 `heavy`（20000 次 `math.sqrt` 累加），`cProfile` 输出显示 `heavy` 的累计时间约占总时间的 94%（0.103 s / 0.109 s），`parse` 仅约 5%。结论很清楚：优化 `heavy`，而不是 `parse`。具体时间每次运行略不同，占比结论稳定。

---

## 5. tracing：事件级的时间线

### 5.1 与采样剖析的区别

| 维度 | 采样剖析 | tracing |
|---|---|---|
| 记录什么 | 周期性快照的调用栈 | **每一个**关键事件（发布、接收、回调开始/结束）及时间戳 |
| 回答 | CPU 时间花在哪些函数 | 一条消息从发布到被处理经历了多久；回调之间如何排队 |
| 开销 | 低，与采样频率有关 | 取决于事件数量；设计为低开销 |
| 结果 | 统计占比 | 完整时间线，可看到偶发的长尾 |

采样剖析告诉你"平均上"哪里重，tracing 告诉你"那一次"为什么慢。**长尾延迟（偶发的 50 ms 抖动）是 tracing 的主场**，采样剖析几乎抓不到。

### 5.2 ros2_tracing 的思路

`ros2_tracing` 在 `rclcpp`/`rcl`/`rmw` 层预埋了跟踪点（tracepoint）：创建发布者/订阅者、发布消息、取到消息、回调开始/结束、定时器触发等。使用 LTTng 作为底层的低开销内核/用户态跟踪框架，把事件写入环形缓冲区，之后离线分析。

典型流程（结构示意，**未在本机运行**，包名与命令在 Humble 上的细节**未核对**）：

```bash
# 1) 开始跟踪会话（默认记录 ROS2 相关跟踪点）
ros2 trace -s my_session
# 2) 运行被测系统，复现问题
# 3) 停止并保存；之后用分析工具读取（如 tracetools_analysis）
```

可以从跟踪数据中计算：

- 每个回调的执行时长分布（不只是平均值）；
- 发布到订阅回调开始的延迟；
- 执行器中回调的排队等待时间；
- 周期性定时器的实际触发间隔与抖动。

### 5.3 跟踪的注意事项

1. **时钟**：跟踪使用单调时钟，与消息里的 `header.stamp` 是两个概念，分析时别混；
2. **事件风暴**：高频话题（1 kHz IMU）的每次发布都有事件，缓冲区会很快写满，要调整缓冲区大小或只开关键跟踪点；
3. **跟踪本身引入延迟**：对硬实时环要评估开销，必要时只在调试构建中启用；
4. **关联事件**：要把"同一条消息"在发布端与订阅端的事件关联起来，需要工具支持（消息指针/序列号等，细节未核对）。

---

## 6. 延迟预算：分位数与不可加性

### 6.1 为什么不能用平均值

控制系统关心的是**最坏情况**：一次长延迟就可能导致失稳或碰撞。平均延迟 10 ms 但偶尔 80 ms 的系统，和稳定 12 ms 的系统，后者更安全。所以用**分位数**（p50 中位数、p95、p99）和**最大值**描述延迟，并给出**截止时间（deadline）**。

### 6.2 端到端预算

把"传感器 → 滤波 → 规划 → 控制 → 桥接 → 执行器"拆成各级，每级给预算：

```text
传感器采集 → 滤波/估计 → 规划 → 控制 → 总线/桥接 → 执行器
   5 ms        2 ms       12 ms    1 ms     3 ms        (示例，中位数)
```

### 6.3 手算：p99 不能直接相加

**模拟设定（示例，非项目实测）**：各级延迟独立，服从对数正态分布（右偏长尾，更符合真实延迟）。各级中位数（ms）与对数标准差：

| 阶段 | 中位数 | 形状参数 $\sigma$ | 模拟 p50 | 模拟 p99 |
|---|---|---|---|---|
| 传感器 | 5 | 0.15 | 5.00 | 7.09 |
| 滤波 | 2 | 0.30 | 2.00 | 4.02 |
| 规划 | 12 | 0.40 | 12.01 | 30.57 |
| 控制 | 1 | 0.20 | 1.00 | 1.59 |
| 桥接 | 3 | 0.50 | 3.00 | 9.63 |

对 20 万次独立模拟求和（`numpy.random.default_rng(0)`，Python 复核）：

- 各级 p99 之和 $=7.09+4.02+30.57+1.59+9.63=52.9$ ms；
- **总延迟的 p99 实际为 42.8 ms**；
- 总延迟 p50 为 23.7 ms，平均 24.6 ms。

**为什么不等**：各级同时处于 p99 的最坏情形概率极低（独立时约 $10^{-10}$ 量级）；总和的分布比单级窄，所以"各级 p99 之和"是**保守的上界**，而非总 p99。但如果各级延迟**相关**（比如同一个 CPU 过载使所有阶段同时变慢），情形就不同了，各级 p99 之和会更接近真实，甚至低估。**结论**：预算要靠对**总延迟**直接测量，并检查是否存在相关的共因（CPU 抢占、同一线程）。

### 6.4 截止时间错过率

若端到端延迟要求不超过 30 ms，按上面的模拟，超过 30 ms 的比例为 15.7%（超过 35 ms 为 5.4%）。尽管中位数 23.7 ms 远低于 30 ms，**每 6–7 次就有一次错过**，这对 50 Hz 的控制是不可接受的。看长尾，而不是中位数。其中规划阶段贡献了最大的长尾（p99 = 30.6 ms），优先优化它或把它从控制回路中解耦（异步规划，控制环用上一次的结果，见 [P5 的规划与控制分层](../../01_项目课程/P5_车式移动机器人自主导航/20_工程化实践_频率周期与问题解决.md)）。

### 6.5 周期抖动

一个 20 ms 周期的定时器，实际周期服从均值 20 ms、标准差 1.5 ms 的高斯近似。周期超过 25 ms 的概率 $P(T>25)=\tfrac12\operatorname{erfc}\!\big(\tfrac{25-20}{1.5\sqrt2}\big)\approx4.3\times10^{-4}$（Python 复核），即约每 2300 次一次，50 Hz 下约每 47 秒一次。**看起来很小的概率，在高频系统里每分钟就会发生一次**——所以对周期超时必须有应对策略（跳过、补偿、报警），而不是假设它不会发生。真实分布常是厚尾的，高斯近似会低估。

---

## 7. 优化的收益上限：阿姆达尔定律

### 7.1 公式

若程序中占比 $f$ 的部分被加速 $s$ 倍，整体加速比为

$$
S=\frac{1}{(1-f)+f/s}.
$$

**推导**：原来总时间归一化为 1，其中 $f$ 部分变为 $f/s$，其余 $(1-f)$ 不变；新总时间 $(1-f)+f/s$；加速比是旧时间除以新时间。

### 7.2 手算

- $f=0.2$，加速 4 倍：$S=1/(0.8+0.05)=1.176$；
- $f=0.4$，加速 4 倍：$S=1/(0.6+0.1)=1.429$。（Python 复核）

即使把 20% 的部分优化 4 倍，整体只快了 17.6%。**极限**：$s\to\infty$ 时 $S\to1/(1-f)$，$f=0.2$ 时上限 1.25。所以**先剖析、再优化，从最大的块下手**；如果最大块只占 20%，不要期待奇迹。

### 7.3 机器人上的补充

- 优化的目标往往不是"平均快"，而是"最坏情况快"（长尾），这时要找的是**偶发路径**（内存分配、锁竞争、日志写盘、垃圾回收），采样剖析抓不到，需要 tracing；
- 把耗时的非实时工作（规划、地图更新、日志）移出实时回路，比把它们优化一点更有效。

---

## 8. 排查案例："变慢了"怎么查

**现象（示例）**：升级后，导航中的局部规划偶尔滞后，车辆在转弯处抖动。操作员描述："有时候反应慢半拍。"

**步骤**：

1. **量化现象**：`ros2 topic hz /cmd_vel` 看实际频率，与标称比较；`ros2 topic delay`（用于带 header 的消息，可用性未核对）看端到端延迟。发现命令话题标称 20 Hz，实际偶发降到 12 Hz；
2. **看诊断/资源**：`top -H` 发现规划节点的一个线程占 CPU 95%，其他线程空闲；
3. **采样剖析**（`perf record -F 99 -g`，Release + 栈帧指针）：火焰图显示 `costmap::update` 占 55%，其中某个"膨胀层"函数占大头；
4. **看变更**：升级时代价地图分辨率从 0.10 m 改成 0.05 m，格子数变为原来的 4 倍；
5. **假设与验证**：计算量近似按格子数线性增长，4 倍格子 → 约 4 倍更新耗时；若原来占周期的 20%，现在约 80%，加上抖动就会超时。把更新频率降低或把分辨率改回，或限制更新窗口，验证偶发降频消失；
6. **回归保护**：把"代价地图更新耗时 p99 小于周期的 50%"写成诊断量，并加入回放回归指标（见 [第 09 章](./09_ROS2测试与CI_launch_testing_gtest_rosbag回放.md)），以后类似改动能被早发现。

**这个案例体现的方法论**：现象量化 → 定位线程/进程 → 定位函数 → 找变更 → 做出可检验的预测（耗时 4 倍）→ 验证 → 把教训固化为监控与测试。每一步的产出都是**数字**，不是感觉。

---

## 9. 在七个项目里怎么用（含诚实边界）

| 项目 | 项目里涉及的观测 | 与本章的关系 | 性质 |
|---|---|---|---|
| [P5 工程化实践](../../01_项目课程/P5_车式移动机器人自主导航/20_工程化实践_频率周期与问题解决.md) | 各层频率与周期、MPC 求解时间预算、通信与调度抖动、六次优化记录 | §6 延迟预算、§8 排查方法；文中数值是该项目的**设计取值**，不是实测 | 项目文档口径；perf/tracing 工具本身**CV 未写** |
| [P6 工程化实践](../../01_项目课程/P6_深开鸿M-Robots平台软件/20_工程化实践_频率周期与问题解决.md) | 各层频率、安全链延迟预算（设计预算 100 ms 内）、状态发布频率、健康模型 | §3 诊断、§6 预算 | 项目设计口径；OpenHarmony 上的观测工具与 ROS2 的不同 |
| [P6 生命周期、配置与可观测性](../../01_项目课程/P6_深开鸿M-Robots平台软件/15_生命周期_配置与可观测性.md) | 健康模型与可观测性 | §3 的 OK/WARN/ERROR/STALE 思想 | 项目设计口径 |
| [P1 数据有效性](../../01_项目课程/P1_AIRS球形机器人平衡控制/08_ROS_CAN安全执行闭环.md)、[P2 分层看门狗](../../01_项目课程/P2_未来动力双臂遥操作与差速底盘/10_ROS2_CAN执行链.md) | 数据年龄、NaN、丢帧检查；分层超时 | §3.2 "沉默即故障"（STALE） | 项目做过的执行链检查 |
| [P3 调试案例](../../01_项目课程/P3_RoboMaster英雄机器人嵌入式控制/11_调试与实战故障案例.md)、[P7 调试手册](../../01_项目课程/P7_可变腿长轮腿机器人/11_调试与故障手册.md) | 嵌入式侧排查：串口/示波器、状态不一致、力矩限幅 | 观测思想相通；MCU 上用不同工具 | 项目做过嵌入式调试 |
| `perf`、火焰图、`ros2_tracing` 的实际使用 | CV 与项目文档未写 | — | **延伸，非项目所做** |

> **面试要点**：诚实的回答——"On my projects I diagnosed problems with logs, topic rates, timestamps and data-age checks, and I set explicit time budgets per layer. I know how to use perf and tracing to find CPU hotspots and tail latency, and I would first quantify the symptom, then find the thread, the function, and the change that caused it."

---

## 10. 常见错误与故障现象

| 现象 | 原因 | 处理 |
|---|---|---|
| 日志文件一天几十 GB | 高频回调直接打印 | 节流；只记状态变化 |
| 出了问题日志里什么也没有 | 日志级别过高；关键路径没打点 | 事先设计打点；运行时可调级别 |
| 实时线程偶发卡顿 | 线程里打印日志、写盘 | 无锁队列 + 后台线程 |
| 诊断显示一切 OK，但节点已崩溃 | 没有 STALE 超时判定 | 对每个诊断源设置过期检测 |
| 阈值天天告警 | 拍脑袋阈值，无迟滞 | 基线测量、迟滞、去抖 |
| 火焰图全是 `[unknown]` | 缺栈帧指针或符号 | `-fno-omit-frame-pointer`、`-g`、`RelWithDebInfo` |
| 火焰图热点与实际不符 | 用 Debug 构建或空载 | Release 加符号，在代表性负载下采样 |
| 误把横轴当时间 | 火焰图横轴是占比 | 记住宽度即占比 |
| 比较 2% 与 3% 的函数 | 采样误差大于差异 | 延长采样，或用精确计数 |
| 平均延迟正常但偶发失控 | 长尾 | 看 p99、最大值、tracing |
| 预算按各级 p99 相加仍超标 | 阶段相关（共因） | 直接测总延迟；找共享资源 |
| 优化无效果 | 优化了占比很小的部分 | 先剖析；用阿姆达尔估收益上限 |
| tracing 导致系统变慢 | 事件太多、缓冲区太小 | 仅开关键跟踪点 |

---

## 11. 面试怎么问、怎么答

**Q1. How do you find out why a robot node is using too much CPU?**

> First I quantify it: which process and which thread, using top in thread mode. Then I run a sampling profiler such as perf with a call graph on a release build with frame pointers and symbols, under a realistic load, and look at the flame graph. The wide, flat boxes at the top are where the time goes. I also keep in mind the statistical error: with about 1000 samples, a function at 20 percent has an uncertainty of about plus or minus 2.5 percent, so the profiler is good at finding big hotspots, not at ranking tiny ones. Before optimizing I use Amdahl's law to estimate the maximum gain.

中文：先量化：哪个进程、哪个线程（top 线程模式）。再用采样剖析器（如 perf 带调用图），在带栈帧指针和符号的 Release 构建、代表性负载下采样，看火焰图——顶部又宽又平的块是时间所在。注意统计误差：约 1000 个样本时占 20% 的函数有约 ±2.5% 的不确定度，所以适合找大热点而非给小项排名。优化前用阿姆达尔定律估算最大收益。

*Follow-up: Why is the x-axis of a flame graph not time?*

> Because the boxes are merged by call stack and sorted alphabetically, so the width represents the share of samples, not the time order.

中文：因为方框按调用栈合并并按字母排序，宽度代表样本占比，而不是时间顺序。

**Q2. What is the difference between sampling profiling and tracing, and when do you use each?**

> A sampling profiler takes periodic snapshots of the call stack and gives a statistical view of where CPU time goes, with low overhead. Tracing records every key event with a timestamp, such as publish, take and callback start and end, so I can reconstruct the timeline of a single message. I use sampling to find average hotspots and tracing to investigate tail latency, like one callback that occasionally takes 50 milliseconds, which sampling would almost never catch.

中文：采样剖析周期性抓取调用栈快照，给出 CPU 时间去向的统计视图，开销低；tracing 记录每个关键事件（发布、接收、回调起止）的时间戳，可重建单条消息的时间线。用采样找平均热点，用 tracing 查长尾延迟（如偶发 50 ms 的回调），采样几乎抓不到这种。

**Q3. You add up the 99th-percentile latency of each stage and get 53 ms, but the measured end-to-end p99 is 43 ms. Why?**

> Percentiles do not add. If the stages are roughly independent, it is very unlikely that all of them are at their worst at the same time, so the sum of p99s is a conservative upper bound. The total distribution is narrower. But if the stages share a cause, for example CPU contention, they become correlated and the sum is no longer conservative. So I measure the end-to-end latency directly, and I look for shared resources.

中文：分位数不能直接相加。各级大致独立时，同时处于最坏情形的概率极低，各级 p99 之和是保守上界，总分布更窄；但若各级有共因（如 CPU 争用）则相关，和就不再保守。因此直接测端到端延迟，并找共享资源。

**Q4. What makes a good diagnostic or health signal for a robot?**

> It should be machine-readable and have a defined level, such as OK, warning, error and stale. The stale state matters because a crashed node cannot report that it crashed, so I detect missing updates by timeout. Thresholds should come from the datasheet, the design budget or a measured baseline, with hysteresis and debouncing so it does not flap. The most important health signals are the data age, the actual versus nominal rate, error counters, and the control loop's execution time against its period.

中文：应机器可读、有明确级别（OK/WARN/ERROR/STALE）。STALE 很重要，因为崩溃的节点无法报告自己崩溃，要靠超时发现更新缺失。阈值来自规格书、设计预算或实测基线，并加迟滞与去抖避免来回切换。最重要的信号：数据年龄、实际与标称频率、错误计数、控制环执行时间与周期之比。

**Q5. Have you used perf, flame graphs or ROS 2 tracing on your projects?**

> Not directly. On my projects I worked with logs, topic rates, timestamps and age checks, and I set explicit time budgets for each layer, which is the same method in a simpler form. I understand how profiling and tracing work, and I would apply the same steps: quantify the symptom, find the thread and function, link it to a change, and turn the lesson into a monitored metric and a regression test.

中文：没有直接用过。项目里用日志、话题频率、时间戳和年龄检查，并为每一层设时间预算——是同一方法的简化形式。我理解剖析与 tracing 的原理，会按同样步骤：量化现象、找线程与函数、关联变更、把教训变成被监控的指标和回归测试。

---

## 12. 本章要点速记

1. 观测要事先设计：事件靠日志，状态靠诊断，性能靠剖析与 tracing；观测本身有开销。
2. 日志：级别有分工；高频回调必须节流；对状态变化打日志；一条日志含对象、数值、阈值。示例：200 B × 100 Hz × 10 节点 = 约 17 GB/天，节流到 1 Hz 降到 173 MB/天。
3. 实时线程里不直接写日志；连续信号用 bag 而不是日志。
4. 诊断四级：OK/WARN/ERROR/STALE；STALE 靠超时判定——沉默即故障；阈值要有来源，要有迟滞与去抖。
5. perf：`-F 99 -g`；Release + 符号 + 栈帧指针；火焰图宽度即占比，横轴不是时间。
6. 采样误差：990 个样本时，20% 的函数标准误 1.27%；要 ±1% 需约 6000 个样本。
7. tracing 记录每个事件，擅长长尾与排队问题；采样擅长平均热点。
8. 延迟用分位数描述；各级 p99 之和是保守上界（示例 52.9 ms vs 总 p99 42.8 ms）；共因会让阶段相关。
9. 阿姆达尔：$S=1/((1-f)+f/s)$；占 20% 的部分加速 4 倍整体只快 17.6%。
10. 排查方法：量化 → 定位线程 → 定位函数 → 找变更 → 可检验预测 → 验证 → 固化为监控与测试。

## 13. 自测题

1. 一个 50 Hz 的回调每次打印一条 150 B 的日志，运行 8 小时产生多少数据？（答案：$150\times50\times28800=2.16\times10^8$ B $\approx216$ MB。）
2. 采样 99 Hz、20 s，某函数占 10%，估计的标准误是多少？（答案：$n=1980$，$\sqrt{0.1\times0.9/1980}=0.67\%$。）
3. 某函数占总时间 50%，被加速 10 倍，整体加速比？上限是多少？（答案：$1/(0.5+0.05)=1.82$；上限 2。）
4. 为什么诊断里要有 STALE，而不是只有 OK/WARN/ERROR？
5. 平均延迟 10 ms、p99 80 ms 的系统与稳定 12 ms 的系统，控制环选哪个？为什么？（要点：控制看最坏情况，长尾可能导致失稳。）

**参考要点**：4. 崩溃节点无法发出 ERROR，只能靠"超时未更新"识别。

## 14. 相关链接

- 上一章：[12 Docker 与部署：交叉编译、systemd](./12_Docker与部署_交叉编译_systemd.md)
- 下一章：[14 ros2_control 与硬件接口实战](./14_ros2_control与硬件接口实战.md)
- 相关：[09 ROS2 测试与 CI](./09_ROS2测试与CI_launch_testing_gtest_rosbag回放.md)、[08 ROS2 Humble 实战](./08_ROS2_Humble实战_launch_参数_生命周期_组件.md)、[05 DDS 与分布式通信](./05_DDS与分布式通信.md)
- 系统工程视角：[数据与日志：可观测性、回放与复盘](../13_系统工程与安全/05_数据与日志_可观测性_回放与复盘.md)
- 项目：[P5 工程化实践](../../01_项目课程/P5_车式移动机器人自主导航/20_工程化实践_频率周期与问题解决.md)、[P6 工程化实践](../../01_项目课程/P6_深开鸿M-Robots平台软件/20_工程化实践_频率周期与问题解决.md)
- [模块 README](./README.md)、[基础知识总览](../00_总览与学习路线.md)
