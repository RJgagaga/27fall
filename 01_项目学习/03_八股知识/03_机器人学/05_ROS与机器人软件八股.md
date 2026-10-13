# 机器人学八股 05：ROS 与机器人软件

> **一句话定位**：机器人面试几乎一定会问 ROS/ROS2：通信模型、QoS、TF、实时性、ros2_control、导航与操作栈、仿真。你的 CV 里有 ROS/ROS2（P1、P2、P5）、ROS2-CAN 执行链（P2）、ROS/ROS2 指令映射到底层驱动与 M-DDS 分布式协调（P6）。本文把这些概念讲成"能解释设计动机"的程度。

**本章学完你能：**
- 说清 ROS1 与 ROS2 的架构差异，以及为什么要换到 DDS；
- 解释 QoS 各策略（Reliability、Durability、History、Deadline…）以及典型传感器/控制/静态数据的选择，诊断"QoS 不兼容收不到数据"；
- 区分 topic、service、action、parameter 的使用场景；
- 讲清 TF 树、静态变换、时间外推错误、REP-103/105 约定；
- 解释 executor、callback group、组件化与进程内通信，以及 ROS2 的实时性局限与做法；
- 描述 ros2_control 的架构（controller manager、hardware interface、controllers）与 ROS2-CAN 执行链；
- 简述 Nav2 与 MoveIt2 的架构；
- 对比 Gazebo、MuJoCo、Isaac 的定位，说明 sim2real 缺口；
- 对应到 P1/P2/P5/P6 的 CV 条目，并给出可直接复述的项目回答。

**前置知识**：
- [ROS1/ROS2 核心概念](../../02_基础知识/09_机器人软件/04_ROS1_ROS2核心概念.md)
- [DDS 与分布式通信](../../02_基础知识/09_机器人软件/05_DDS与分布式通信.md)
- [MuJoCo、Pinocchio、KDL 工具链](../../02_基础知识/09_机器人软件/06_MuJoCo_Pinocchio_KDL_工具链.md)
- [URDF、TF、坐标系实践](../../02_基础知识/09_机器人软件/07_URDF_TF_坐标系实践.md)
- [CAN/FDCAN 与电机通信](../../02_基础知识/08_嵌入式与通信/03_CAN_FDCAN与电机通信.md)

**优先级**：★★★ 必会；★★ 常见；★ 加分。

**目录**
1. ROS 基础与 ROS1 vs ROS2（Q1–Q6）
2. DDS 与 QoS（Q7–Q14）
3. 通信模式与节点模型（Q15–Q24）
4. TF 与坐标系（Q25–Q30）
5. 实时性（Q31–Q35）
6. ros2_control 与硬件执行链（Q36–Q43）
7. Nav2、MoveIt、bag 与工具（Q44–Q50）
8. 多机器人与分布式（Q51–Q54）
9. 仿真：Gazebo、MuJoCo、Isaac（Q55–Q60）
10. 软件架构与安全执行链（Q61–Q64）
11. 要点速记、自测题、相关链接

---

## 一、ROS 基础与 ROS1 vs ROS2

### Q1. ROS 是什么？它解决什么问题？（What is ROS?）★★★
**答案要点**
- ROS（Robot Operating System）不是操作系统，而是**机器人中间件 + 工具链 + 生态**：节点（进程）间通信、包管理与构建、坐标变换、可视化与调试、仿真与大量现成算法包（导航、操作、SLAM 等）。
- 核心思想：把机器人软件拆成许多松耦合节点，通过话题等机制通信，便于复用、替换、独立调试和回放数据。
- 项目：P1 用 ROS 做状态处理并与 CAN 电机执行层集成；P2 用 ROS2-CAN 链；P5 用 ROS 导航栈。
**English short answer**
ROS is robotics middleware plus tools and a large ecosystem. It splits a robot system into loosely coupled nodes that communicate over topics, services and actions, and gives me coordinate transforms, logging, visualisation and ready-made packages. I used it as the glue between estimation, planning and the CAN motor layer.
**追问方向**：为何松耦合；节点粒度怎么选。

### Q2. ROS1 与 ROS2 的主要区别？（ROS1 vs ROS2）★★★
| | ROS1 | ROS2 |
|---|---|---|
| 通信 | 自研 TCPROS/UDPROS，需 `roscore`（master） | DDS（可插拔 RMW），**去中心化发现** |
| QoS | 无 | 丰富的 QoS 策略 |
| 实时性 | 不支持 | 可做实时（需配合 RT 内核、executor、无动态分配） |
| 多机器人 | 单 master，困难 | 域 ID、命名空间、发现服务器 |
| 生命周期 | 无 | 托管节点（lifecycle） |
| 语言/平台 | Linux 为主 | Linux/Windows/macOS/RTOS（micro-ROS） |
| 构建 | catkin | ament/colcon |
| 参数 | 全局参数服务器 | 节点内参数，可动态回调 |
| 状态 | 已 EOL（Noetic 2025） | 持续维护 |

**English short answer**
ROS2 replaces the custom transport and the central master with DDS, so discovery is decentralised and there are QoS policies. It also adds lifecycle nodes, better multi-robot support, real-time capability and cross-platform builds. ROS1 reached end of life, so new work goes to ROS2.
**追问方向**：为什么需要 QoS；迁移方式（ros1_bridge）。

### Q3. 为什么 ROS2 选择 DDS？（Why DDS?）★★★
**答案要点**
- DDS（Data Distribution Service，OMG 标准）是工业成熟的发布订阅中间件（航空、军工），具备：去中心化发现、QoS、可靠/尽力传输、实时友好、多厂商实现（Fast DDS、Cyclone DDS、Connext）；
- ROS2 通过 RMW（ROS Middleware Interface）抽象，可切换实现（`RMW_IMPLEMENTATION`）；
- 代价：配置复杂、跨网络发现开销、默认多播在 Wi-Fi 上不稳。
**English short answer**
DDS is a mature, standardised publish-subscribe middleware with decentralised discovery and rich QoS, designed for real-time systems. ROS2 wraps it through the RMW layer so the vendor can be swapped. The cost is configuration complexity, and discovery over Wi-Fi needs care.
**追问方向**：RMW 差异；Fast DDS vs Cyclone DDS 选型。

### Q4. ROS2 的节点、话题、服务、动作、参数怎么选？（Communication patterns）★★★
| 机制 | 模式 | 适用 |
|---|---|---|
| Topic | 发布/订阅，多对多，异步 | 传感器流、状态、连续命令 |
| Service | 请求/响应，同步语义（客户端可异步），短任务 | 获取/设置配置、触发一次性动作（如清地图） |
| Action | 目标/反馈/结果，可取消，长任务 | 导航到点、机械臂运动 |
| Parameter | 节点配置，可动态修改 | 增益、阈值、开关 |

- 经验：连续数据用 topic；耗时过程用 action；别用 service 做长时间阻塞。
**English short answer**
Topics are for streams like sensor data and continuous commands, services are for short request-response calls, actions are for long-running goals with feedback and cancellation, and parameters are for configuration. I would never use a service for something that takes seconds.
**追问方向**：action 的状态机；topic 丢数据。

### Q5. 一个 ROS2 节点的典型结构？（Node structure）★★
**答案要点**
- 继承 `rclcpp::Node`，声明参数、创建 publisher/subscriber/timer/service；回调由 executor 调度；
- 控制类节点用 timer 周期触发；订阅回调只缓存最新数据，控制逻辑在定时器中读取（避免回调乱序）。
```cpp
// 教学示例，非项目源码
class Ctrl : public rclcpp::Node {
 public:
  Ctrl() : Node("ctrl") {
    declare_parameter("kp", 1.0);
    sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "imu", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::Imu::SharedPtr m){ imu_ = *m; });
    pub_ = create_publisher<std_msgs::msg::Float64>("cmd", 10);
    timer_ = create_wall_timer(std::chrono::milliseconds(10), [this]{ step(); });
  }
 private:
  void step() { /* 读 imu_，计算，发布 */ }
  sensor_msgs::msg::Imu imu_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
```
**English short answer**
A node declares its parameters, creates subscriptions, publishers and a timer, and an executor runs the callbacks. For control, I cache the latest measurements in the subscriber callbacks and compute the command in a periodic timer, so the loop timing does not depend on message arrival.
**追问方向**：回调组；时钟源。

### Q6. 为什么控制回路不要直接写在订阅回调里？（Control in callbacks?）★★
**答案要点**
- 订阅回调的触发频率取决于消息到达（抖动、丢包、乱序）；
- 多个输入源到来时间不同，回调中使用的是不同时刻的数据；
- 定时器回路让控制周期固定，读取"最新快照"，并能检测数据过期（超时即降级）。
**English short answer**
Callback timing follows message arrival, which jitters, while the controller wants a fixed period. So I use a timer, read the latest cached data, and check its age so I can degrade safely if data is stale.
**追问方向**：数据快照一致性；过期阈值。

---

## 二、DDS 与 QoS

### Q7. DDS 的基本概念：Domain、Participant、Topic、Publisher/Subscriber、DataWriter/DataReader？（DDS concepts）★★
**答案要点**
- Domain（域，ID 0–232）：隔离的通信空间，`ROS_DOMAIN_ID` 决定；
- Participant：域内的参与者（一个进程一般一个）；
- Topic：名字+类型；DataWriter/Reader 是发布/订阅的端点，QoS 在端点上匹配；
- 发现：多播（SPDP/SEDP），或 Discovery Server（单播，适合 Wi-Fi/跨网段）。
**English short answer**
A DDS domain is an isolated communication space chosen by an ID. Each process has a participant, and publishers and subscribers are matched by topic name, type and compatible QoS. Discovery normally uses multicast, but a discovery server is better on Wi-Fi.
**追问方向**：域 ID 冲突；discovery server。

### Q8. 常用 QoS 策略有哪些？（Key QoS policies）★★★
| 策略 | 取值 | 含义 |
|---|---|---|
| Reliability | RELIABLE / BEST_EFFORT | 是否重传保证到达 |
| Durability | VOLATILE / TRANSIENT_LOCAL | 晚加入订阅者是否能拿到历史（latched） |
| History | KEEP_LAST(N) / KEEP_ALL | 缓存多少 |
| Depth | N | KEEP_LAST 的深度 |
| Deadline | 时间 | 期望更新间隔，超时回调 |
| Lifespan | 时间 | 消息有效期，过期丢弃 |
| Liveliness | AUTOMATIC / MANUAL | 发布者"活着"的检测与租约 |

**English short answer**
The main ones are reliability, durability, history with depth, deadline, lifespan and liveliness. Reliability decides whether lost samples are retransmitted. Durability decides whether late joiners get old data. Deadline and liveliness let me detect that a publisher has gone quiet.
**追问方向**：liveliness 用于安全；lifespan 用于丢弃陈旧命令。

### Q9. 不同数据类型怎么选 QoS？（Choosing QoS）★★★
| 数据 | 推荐 | 原因 |
|---|---|---|
| 高频传感器（IMU、LiDAR、图像） | BEST_EFFORT, KEEP_LAST(小) | 新数据比旧数据重要，不要重传拖延 |
| 控制命令 | RELIABLE 或 BEST_EFFORT + 小深度，加 lifespan/deadline | 丢一帧没关系，但不能积压旧命令 |
| 静态/配置（地图、robot_description） | RELIABLE + TRANSIENT_LOCAL | 晚加入者也要拿到 |
| 事件/状态变化 | RELIABLE | 不能丢 |

- `rclcpp::SensorDataQoS()` 就是 best-effort + 小深度的预设。
**English short answer**
For high-rate sensor streams I use best-effort with a small queue, because fresh data matters more than retransmission. For maps and robot descriptions I use reliable and transient-local so late subscribers still receive them. For control commands I avoid deep queues, so old commands never pile up.
**追问方向**：深度过大造成延迟累积。

### Q10. "订阅者收不到数据"最常见原因？（QoS incompatibility）★★★
**答案要点**
- QoS 不兼容：发布者 BEST_EFFORT 而订阅者要求 RELIABLE → **不匹配**（反过来可匹配）；Durability：订阅者要求 TRANSIENT_LOCAL 而发布者 VOLATILE 不匹配；
- 规则：**订阅者的要求不能高于发布者的承诺**；
- 其他：Domain ID 不一致、命名空间/话题名重映射、跨网络多播被阻断、防火墙、RMW 不一致、类型不匹配；
- 排查：`ros2 topic info -v` 查 QoS；`ros2 doctor`；rviz 里订阅 QoS 设置。
**English short answer**
The most common cause is a QoS mismatch, for example a best-effort publisher with a reliable subscriber, since a subscriber cannot ask for more than the publisher offers. Other causes are different domain IDs, topic name remaps, and blocked multicast. I check with ros2 topic info in verbose mode.
**追问方向**：RViz 订阅点云看不到的典型原因。

### Q11. 消息序列化与零拷贝？（Serialization and zero-copy）★★
**答案要点**
- 跨进程需序列化（CDR）+ 拷贝；大消息（图像/点云）占用带宽与 CPU；
- 优化：进程内通信（intra-process，composable node + `unique_ptr` 发布）零拷贝；共享内存传输（Fast DDS/CycloneDDS+iceoryx）；使用压缩（image_transport）；降频/降分辨率；
- 固定大小消息类型便于实时（避免动态分配）。
**English short answer**
Between processes, messages are serialised and copied, which is expensive for images and point clouds. I reduce this with composable nodes and intra-process communication, shared-memory transports, compression, or by lowering the rate.
**追问方向**：loaned messages；iceoryx。

### Q12. 时间戳与 `use_sim_time`？（Time and sim time）★★
**答案要点**
- 消息 header 的 `stamp` 应为传感器采样时间；
- `use_sim_time=true` 时节点用 `/clock`（仿真/rosbag 回放），否则系统时间；混用会导致 TF 外推错误；
- 回放 bag：`--clock`；注意所有相关节点一致。
**English short answer**
The header stamp should be the acquisition time of the data. When simulating or replaying a bag, all nodes must use sim time from the clock topic, otherwise TF lookups fail because of mixed time bases.
**追问方向**：时间源混用现象。

### Q13. 生命周期节点（Lifecycle node）是什么？（Lifecycle）★★
**答案要点**
- 状态机：Unconfigured → Inactive → Active → Finalized，转换 configure/activate/deactivate/cleanup/shutdown；
- 用途：硬件驱动与导航栈（Nav2 全面使用）可控启动顺序、失败可恢复、激活前不发布；
- 对安全执行：可在 Inactive 时禁止输出命令。
**English short answer**
A lifecycle node has explicit states such as unconfigured, inactive and active, with defined transitions. It is used for drivers and navigation components to control start-up order and to make sure nothing publishes commands before it is activated.
**追问方向**：失败转移；managed 启动。

### Q14. DDS 在无线网络上的问题？（DDS over Wi-Fi）★★
**答案要点**
- 多播发现在 Wi-Fi 下不稳定、广播风暴；大消息分片丢包；
- 对策：Discovery Server（单播）、限制参与者（`ROS_AUTOMATIC_DISCOVERY_RANGE`/`ROS_LOCALHOST_ONLY`）、降低大数据频率、使用压缩、调整 `max_message_size` 与发送队列；
- 多机器人：每台独立 Domain ID，用桥接转发选定话题。
**English short answer**
Multicast discovery and large fragmented messages behave badly on Wi-Fi. I use unicast discovery servers, restrict who discovers whom, compress heavy streams, and bridge only the topics that must cross between robots.
**追问方向**：多机器人 domain 设计。

---

## 三、通信模式与节点模型

### Q15. Action 的内部机制？（Actions）★★
**答案要点**
- 由 3 个 service（send_goal、cancel_goal、get_result）+ 2 个 topic（feedback、status）组成；
- 目标状态：ACCEPTED → EXECUTING → SUCCEEDED/ABORTED/CANCELED；
- 客户端异步，支持抢占（新目标替换旧目标）；
- 导航、机械臂轨迹执行（FollowJointTrajectory）都是 action。
**English short answer**
An action is built from three services and two topics: send goal, cancel, get result, plus feedback and status streams. The goal moves through accepted, executing and a final state. It is the right pattern for navigation and for trajectory execution.
**追问方向**：取消语义；抢占。

### Q16. Executor 和回调组（Callback groups）是什么？（Executors）★★★
**答案要点**
- Executor 负责从 DDS 取事件并调度回调：`SingleThreadedExecutor`（串行，无竞争）、`MultiThreadedExecutor`（并行，需线程安全）、`StaticSingleThreadedExecutor`（开销低）；
- 回调组：`MutuallyExclusive`（组内互斥，默认）、`Reentrant`（允许并行）；
- 常见坑：在回调里同步调用 service（`future.get()`）会死锁（单线程）；长耗时回调阻塞 timer；
- 对策：耗时工作放独立线程/回调组；用异步调用。
**English short answer**
An executor dispatches callbacks, either on one thread or a pool. Callback groups say which callbacks may run in parallel. A classic bug is blocking inside a callback while waiting for a service response, which deadlocks a single-threaded executor. I put heavy work into separate threads or groups.
**追问方向**：定时器漂移；多线程数据竞争。

### Q17. 组件化（Composable nodes）有什么好处？（Composition）★★
**答案要点**
- 多节点装载进同一进程，进程内零拷贝，减少上下文切换与序列化；运行时动态加载（component container）；
- 代价：故障隔离差（一个崩溃全挂）。
**English short answer**
Composable nodes run in the same process, so messages can be passed without copying and with fewer context switches. The trade-off is weaker fault isolation, since one crash can take down the container.
**追问方向**：何时合并何时拆分。

### Q18. 参数（parameters）怎么用？（Parameters）★★
**答案要点**
- 节点声明 `declare_parameter`；通过 YAML/launch 传入；运行时 `ros2 param set`；动态回调 `add_on_set_parameters_callback` 校验；
- 调参套路：控制增益做成参数，实机热调，不重编译；记录最终值到 YAML 版本管理。
**English short answer**
I declare gains and limits as parameters, load them from YAML in the launch file, and change them live with the param tool or a callback that validates them. That lets me tune on the robot without recompiling, and I commit the final values.
**追问方向**：参数覆盖优先级；参数服务重启后丢失。

### Q19. Launch 文件做什么？（Launch）★★
**答案要点**
- 描述启动哪些节点、参数、重映射、命名空间、条件和包含关系；ROS2 launch 可用 Python（最灵活）、XML、YAML；
- 常用：`LaunchConfiguration`、`DeclareLaunchArgument`、`IncludeLaunchDescription`、`OnProcessExit` 事件；
- 最佳实践：仿真与实机用同一 launch，通过 `use_sim_time`/参数切换。
**English short answer**
A launch file starts the nodes with their parameters, remappings and namespaces. I write them in Python for flexibility and share the same launch between simulation and hardware, switching with arguments.
**追问方向**：启动顺序依赖；重启策略 respawn。

### Q20. 话题重映射与命名空间？（Remap, namespace）★
**答案要点**
- `--ros-args -r old:=new` 重映射；命名空间隔离多机器人（`/robot1/cmd_vel`）；
- TF 在 ROS2 中不自动带命名空间，多机器人需前缀 frame id。
**English short answer**
Remapping renames a topic at launch without changing code, and namespaces keep multiple robots separate. TF frames need explicit prefixes per robot.
**追问方向**：TF 多机冲突。

### Q21. 自定义消息设计原则？（Message design）★★
**答案要点**
- 带 `Header`（stamp + frame_id）；单位 SI；字段含义和坐标系明确；避免变长大数组；版本演进可兼容；
- P6 中的"统一能力模型"类似：为传感器/执行器/控制器定义统一接口与状态消息，含时间戳、坐标系、有效性标志。
**English short answer**
I include a header with timestamp and frame, use SI units, document each field, and keep messages small and fixed-size where possible. Validity flags and timestamps are crucial for downstream safety checks.
**追问方向**：向后兼容。

### Q22. ROS2 的 rosbag2 怎么用？（rosbag2）★★★
**答案要点**
- 录制/回放话题（mcap/sqlite3 格式）：`ros2 bag record -a`、`ros2 bag play --clock`；
- 用途：离线调试、算法回归、数据集；回放时用 `use_sim_time`；
- 注意：QoS 覆盖（回放 transient_local）、磁盘带宽（图像/点云）、时间戳连续；
- 导航/平衡调试：同时记录输入、状态、命令与饱和标志。
**English short answer**
I record the relevant topics into a bag and replay them with the clock enabled, so I can debug offline and rerun algorithms on the same data. For tuning, I record sensor input, estimated state, commands and saturation flags together.
**追问方向**：回放 QoS；bag 太大怎么办。

### Q23. 常用调试工具？（Debug tools）★★
**答案要点**
- `ros2 topic list/echo/hz/bw/info`、`ros2 node info`、`ros2 service call`、`ros2 param`、`ros2 doctor`；
- `rqt_graph`、`rqt_plot`、PlotJuggler（时序曲线，调 PID 必备）、RViz2、`tf2_tools view_frames`；
- `ros2 run tf2_ros tf2_echo a b`。
**English short answer**
I use the ros2 command-line tools for topics, nodes and parameters, rqt_graph to see the connections, PlotJuggler for time series, RViz for visualisation, and tf2_echo for transforms.
**追问方向**：延迟测量。

### Q24. Python（rclpy）与 C++（rclcpp）怎么选？（rclpy vs rclcpp）★
**答案要点**
- C++：性能、确定性、实时、零拷贝；Python：开发快，适合上层逻辑、调试、AI 推理；
- 控制回路与驱动用 C++；任务编排用 Python。
**English short answer**
I write drivers and control loops in C++ for performance and determinism, and use Python for orchestration, prototyping and AI components.
**追问方向**：GIL。

---

## 四、TF 与坐标系

### Q25. TF2 是什么？为什么需要？（TF2）★★★
**答案要点**
- 维护一棵**随时间变化的坐标系树**，缓冲一段历史（默认 10 s），提供任意两个坐标系在任意时刻的变换查询（含插值）；
- 发布者：`robot_state_publisher`（URDF 固定与关节变换）、`static_transform_publisher`、定位/里程计节点；
- 规则：树（每个 frame 仅一个父节点）、无环；
- 使用：`lookupTransform(target, source, time)`；传感器数据按其时间戳转换。
**English short answer**
TF keeps a tree of coordinate frames over time and lets me ask for the transform between any two frames at any recent time, with interpolation. It saves me from manually chaining transforms, as long as every frame has exactly one parent.
**追问方向**：为何一个父节点；缓存时长。

### Q26. REP-103 与 REP-105 约定？（REP-103/105）★★★
**答案要点**
- REP-103：右手系；x 前、y 左、z 上；单位 SI（m、rad、s）；
- REP-105：`map → odom → base_link`，`odom` 连续漂移，`map` 全局一致，`map→odom` 由定位发布；
- 光学坐标系（相机）z 前、x 右、y 下，需要单独的 optical frame。
**English short answer**
The convention is right-handed with x forward, y left, z up, in SI units. For navigation, the chain is map to odom to base_link, where odom is smooth and drifts and map is globally consistent, and the localiser publishes the map to odom correction.
**追问方向**：相机光学坐标系；多地图。

### Q27. 常见 TF 报错及原因？（TF errors）★★★
- `Lookup would require extrapolation into the future/past`：时间戳不同步，请求的时间超出缓存；原因：时钟不一致、`use_sim_time` 混用、传感器时间戳取了 now、发布延迟；解决：用消息时间戳、`waitForTransform`/超时、统一时间源；
- `Could not find a connection between frames`：树断开/缺发布者；
- `TF_REPEATED_DATA` / 两个发布者争同一 frame：多个节点发布同一对 frame 造成跳变；
- 忘记 static transform（传感器外参）。
**English short answer**
Extrapolation errors mean the requested time is outside the buffered range, usually from mismatched clocks or timestamps. A missing connection means a break in the tree. Two nodes publishing the same transform cause jumping poses. I check time sources first.
**追问方向**：典型案例：IMU、轮速计与 LiDAR 的时间戳来源不一致，导致定位位姿跳变和 TF 外推报错；做法是统一用消息自带时间戳、统一时间源并加入有效性检查，对应 CV 中 *drift and pose jumps from inconsistent timing, frames and motion direction*。

### Q28. 静态变换如何确定（传感器外参）？（Extrinsic calibration）★★
**答案要点**
- 测量/CAD 初值，再标定（LiDAR-IMU、相机-IMU、手眼标定）；
- 精度影响定位与障碍位置；放在 URDF 的固定关节中，由 `robot_state_publisher` 发布。
**English short answer**
I start from CAD or measured mounting offsets and refine by calibration. They live in the URDF as fixed joints. Errors here show up directly as localisation and obstacle position errors.
**追问方向**：手眼标定。

### Q29. URDF / xacro / SRDF 是什么？（URDF family）★★
**答案要点**
- URDF：link/joint/惯量/几何/限位；xacro：宏与参数化；SRDF：MoveIt 的规划组、自碰撞矩阵、预设姿态；
- 注意惯量设置影响重力补偿与仿真（见 [01 八股](./01_运动学动力学八股.md) Q50）。
**English short answer**
URDF describes links, joints, inertias and limits. Xacro adds macros and parameters, and SRDF adds planning groups and collision pairs for MoveIt. Correct inertial parameters matter for gravity compensation and simulation.
**追问方向**：惯量单位与参考系。

### Q30. 多传感器时间对齐：message_filters 怎么用？（Message filters）★★
**答案要点**
- `ApproximateTimeSynchronizer` 按时间戳近似对齐多话题；`ExactTime` 要求相同；
- 注意队列大小与慢速源；对高频 IMU 常用插值。
- P6 的"标准化时间戳"可看作同类问题在设备层的统一处理。
**English short answer**
Message filters pair up messages from several topics by timestamp, either exactly or approximately within a tolerance. For high-rate sensors like an IMU I often interpolate to the time of the slower sensor.
**追问方向**：容差设定。

---

## 五、实时性

### Q31. ROS2 是"实时"的吗？（Is ROS2 real-time?）★★★
**答案要点**
- ROS2 **支持**构建实时系统，但默认不实时：需要 ① PREEMPT_RT 内核或 RTOS；② 实时友好的 executor 与不在实时路径中做动态内存分配、锁、系统调用；③ 线程优先级/CPU 绑定；④ DDS 可配置；
- 区分硬实时与软实时：电机电流环/平衡环等硬实时放 MCU（STM32+FreeRTOS，P3/P7）；ROS2 在主控机上做软实时的规划、估计、上层控制；
- 抖动来源：调度、页面错误（`mlockall`）、GC/分配、DDS 线程、日志 IO。
**English short answer**
ROS2 can be used in real-time systems, but it is not real-time out of the box: it needs a real-time kernel, careful memory handling, priorities, and a deterministic executor. In practice, I put the hard real-time loops on a microcontroller and run ROS2 for soft real-time layers like estimation and planning.
**追问方向**：延迟抖动测量（cyclictest）；mlockall。

### Q32. 控制频率和通信周期如何设计？（Rates and latency）★★★
**答案要点**
- 分层频率：电流环 kHz（驱动器）→ 底层控制 200 Hz–1 kHz（MCU）→ 上位机 50–200 Hz（ROS2 控制/IK）→ 规划 1–20 Hz；
- 上层命令抵达底层有延迟，底层需要超时保护（命令过期则回到安全状态）；
- 具体 P2/P1：本项目通常取 ROS2 控制节点 100–200 Hz、CAN 命令周期 1 kHz 级、电机反馈 1 kHz 级，端到端延迟在几毫秒量级（典型/教学示例，以实机为准）；命令超时后回到安全状态。
**English short answer**
Frequencies decrease as we go up: current loops in kilohertz in the drive, joint control at hundreds of hertz to a kilohertz on the microcontroller, upper-level control at tens to hundreds of hertz in ROS2, and planning at a few hertz. The low-level layer has a timeout so that stale commands drive the robot to a safe state.
**追问方向**：命令超时；时间戳。

### Q33. 如何减少 ROS2 节点的延迟抖动？（Reducing jitter）★★
**答案要点**
- 设置线程优先级（SCHED_FIFO）、CPU 亲和、`isolcpus`；
- 避免日志/打印、动态分配；预分配消息；
- 减少节点间序列化（组件化）；
- 使用硬件时间戳与固定周期定时器（`create_wall_timer` 受系统时钟影响，注意）。
**English short answer**
I pin the control thread to an isolated core with real-time priority, avoid allocations and printing in the loop, preallocate messages, and use composition to avoid serialisation.
**追问方向**：Linux 调度策略。

### Q34. 优先级反转？（Priority inversion）★★
**答案要点**
- 低优先级任务持有高优先级任务需要的锁，被中优先级抢占 → 高优先级被间接阻塞；
- 解决：优先级继承/天花板（FreeRTOS 互斥量有继承；二值信号量没有）；
- 详见 [FreeRTOS 实时系统](../../02_基础知识/08_嵌入式与通信/02_FreeRTOS实时系统.md)、[06 嵌入式八股](./06_嵌入式与机电八股.md)。
**English short answer**
Priority inversion happens when a low-priority task holding a lock is preempted by a medium-priority task while a high-priority task waits. Priority inheritance on mutexes solves it.
**追问方向**：Mars Pathfinder。

### Q35. 看门狗和心跳在 ROS 系统里怎么用？（Watchdog, heartbeat）★★★
**答案要点**
- 上位机 → MCU 周期心跳/命令，MCU 超时即进入安全态（刹车/零力矩/保持）；
- 节点存活监控：Liveliness QoS、lifecycle、自定义 health topic；
- MCU 内部：独立看门狗（IWDG）防程序跑飞。
**English short answer**
The host sends periodic commands or a heartbeat, and the microcontroller falls back to a safe state if they stop arriving. On the ROS side I can use liveliness QoS or a health topic, and the MCU has its own hardware watchdog.
**追问方向**：超时阈值；安全态定义。

---

## 六、ros2_control 与硬件执行链

### Q36. ros2_control 的架构？（ros2_control architecture）★★★
**答案要点**
```text
 controller_manager (周期 update 循环, 常 100~1000 Hz)
   |-- Controllers (插件): joint_trajectory_controller, diff_drive_controller,
   |                        forward_command_controller, 自写控制器
   |-- Resource Manager
         |-- Hardware Interface (插件): read() / write()
               state interfaces:  position, velocity, effort
               command interfaces: position, velocity, effort
```
- 循环：`read()`（从硬件取状态）→ `update()`（各控制器计算）→ `write()`（下发命令）；
- 硬件接口类型：System（整机，如多关节）、Sensor、Actuator；
- 好处：控制器与硬件解耦，同一控制器可用于仿真（gz_ros2_control）和实机；
- 接口声明与"claim"机制保证一个命令接口同时只被一个控制器使用。
**English short answer**
ros2_control has a controller manager running a periodic loop: read the hardware states, update the controllers, write the commands. Hardware is wrapped in plugins exposing state and command interfaces like position, velocity and effort. It decouples controllers from hardware, so the same controller works in simulation and on the real robot.
**追问方向**：controller 切换；实时线程；接口 claim。

### Q37. 如何写一个硬件接口？（Writing a hardware interface）★★
**答案要点**
- 实现 `on_init/on_configure/on_activate`、`export_state_interfaces/export_command_interfaces`、`read/write`；
- `read` 从 CAN/串口取最新反馈（非阻塞），`write` 打包发送；
- 要点：`read/write` 必须快速、非阻塞；错误处理（返回 ERROR 触发停机）；
- P2/P1 的 CAN 链路可以类比：ROS 状态处理 → CAN 电机执行。
**English short answer**
A hardware interface exports state and command handles and implements read and write. Both must be non-blocking and fast, because they run inside the control loop. Errors are reported so the manager can stop safely.
**追问方向**：CAN 超时处理；读写频率。

### Q38. ROS2-CAN 执行链是什么样？（ROS2-CAN chain）★★★
**答案要点**
- 典型链路：ROS2 节点（IK/控制）→ 命令话题 → CAN 驱动节点/硬件接口（SocketCAN，`can0`）→ CAN 总线 → 电机驱动器；反馈相反方向；
- SocketCAN：Linux 内核 CAN 栈，`ip link set can0 type can bitrate 1000000 up`，`candump/cansend` 调试；
- 帧格式：ID（标准 11 位/扩展 29 位）、8 字节数据；电机协议（MIT 模式 5 量打包、电流模式等）；
- 设计要点：发送节拍、反馈解析与时间戳、超时检测、限幅（力矩/电流/速度）、总线负载率、错误帧处理；
- CV 对应：P2 *validated on real hardware through the ROS2-CAN execution chain*；P1 *integrated ROS state processing with CAN motor execution*。
- 我们的做法：通过 SocketCAN 在 ROS2 节点中收发，命令按固定节拍下发、反馈带时间戳解析；报文周期取毫秒级，控制量按电机协议打包并限幅；总线负载按帧率和帧长估算并留余量；丢帧或反馈超时时按超时处理，输出回到安全态（典型做法，以实机为准）。
**English short answer**
The chain goes from the ROS2 control node to a CAN driver, usually through SocketCAN, onto the bus and into the motor drives, with feedback coming back the same way. The important design points are the send rate, parsing feedback with timestamps, timeouts, limiting torque and speed, and keeping an eye on the bus load.
**追问方向**：总线负载；丢帧；packet 打包。

### Q39. 控制器怎么切换且不冲击？（Controller switching）★★
**答案要点**
- `switch_controller` 服务；切换前先让新控制器的输出对齐当前状态（bumpless）；
- 命令接口互斥；停用旧控制器时确保最后命令平滑（保持位置或零力矩）。
**English short answer**
I switch controllers through the controller manager, and I make sure the new controller starts from the current state so there is no jump. Command interfaces are exclusive, so the old controller is deactivated first.
**追问方向**：模式切换（位置→力矩）。

### Q40. 差速底盘怎么接入 ROS？（Diff-drive in ROS）★★
**答案要点**
- `cmd_vel`（Twist）→ 差速运动学 → 左右轮速目标 → 轮速 PID（编码器反馈）→ 电机；反馈轮速 → 里程计（`odom` 话题 + TF）；
- 使用 `diff_drive_controller` 或自写：参数包括轮径、轮距、速度/加速度限制；
- P2：*mapped body velocity commands to wheel targets via differential-drive kinematics, closed the PID loop using encoder feedback and integrated wheel motion into odometry*。
**English short answer**
A velocity command is turned into left and right wheel targets by the differential-drive kinematics. The wheel loops use encoder feedback, and the measured wheel speeds are integrated into an odometry message and transform. That is the base interface the navigation stack expects.
**追问方向**：轮距标定；cmd_vel 超时。

### Q41. 命令超时、限幅、急停的实现层次？（Safety layers）★★★
**答案要点**
- 层次：驱动器硬件保护（过流/过压/过温）→ MCU 固件（限幅、超时、看门狗）→ ROS 控制节点（软限位、速率限制）→ 上层（策略检查）；
- 任何一层都不能假设上一层可靠；急停(E-stop)应有硬件通道；
- 对应 P1：*output limiting and fall-recovery safeguards*。
**English short answer**
I layer the protections: drive-level hardware protection, firmware limits and timeouts, ROS-level soft limits and rate limits, and high-level policy checks. No layer trusts the one above, and the emergency stop has a hardware path.
**追问方向**：fall recovery 的触发条件。

### Q42. 遥操作系统的数据流？（Teleoperation pipeline）★★
**答案要点**
- VR 控制器位姿 → 坐标变换到机器人基座 → 缩放/滤波/限速 → IK（Mink）→ 关节命令 → 执行器；反馈：关节状态 → 可视化/力反馈；
- 要点：离合机制、坐标对齐、延迟、安全（超出范围/丢包时保持）。
- 对应 P2。
**English short answer**
The VR controller pose is transformed into the robot base frame, scaled and filtered, solved by the IK solver into joint commands, and sent to the actuators over the ROS2 and CAN chain. Clutching, frame alignment, latency and safe behaviour on packet loss are the main design points.
**追问方向**：双臂如何分别处理；延迟补偿。

### Q43. 主从示教的软件结构？（Leader-follower structure）★★
**答案要点**
- leader：读关节 → 重力补偿 → 低阻力；发布关节状态；follower：订阅 leader 状态 → 位置-速度跟踪（力矩/位置指令）；
- 同步性：时间戳、插值、速率限制；
- 对应 P2。
**English short answer**
The leader arm publishes its joint states while running gravity compensation, and the follower subscribes and tracks position and velocity. Timestamps, interpolation and rate limiting keep the motion synchronous and safe.
**追问方向**：follower 滞后。

---

## 七、Nav2、MoveIt、其他

### Q44. Nav2 架构？（Nav2 architecture）★★★
**答案要点**
- 服务器：Planner Server（全局规划插件：NavFn、Smac Hybrid-A*/2D/Lattice）、Controller Server（局部控制/跟踪插件：DWB、TEB、MPPI、Regulated Pure Pursuit）、Smoother Server、Behavior Server（Spin、BackUp、Wait…）、Waypoint Follower、Costmap；
- 编排：BT Navigator（行为树，`NavigateToPose`）；
- 状态：AMCL 或其它定位；Lifecycle manager 管理节点；
- 与 ROS1 move_base 对比：move_base 单体状态机，Nav2 模块化+行为树+生命周期。
- P5 的 Hybrid-A*、TEB 在 Nav2 中分别对应 Smac Hybrid-A* 与 TEB 控制器插件的形态（本项目的导航链按 ROS 栈组织，规划与控制以插件形式接入，概念上与 Nav2 的 Smac/TEB 插件一一对应）。
**English short answer**
Nav2 consists of planner, controller, smoother and behaviour servers, coordinated by a behaviour tree navigator, with costmaps, localisation and lifecycle management. Compared with the monolithic ROS1 move_base, it is modular and uses behaviour trees to define recovery logic. Smac provides Hybrid-A* planning.
**追问方向**：行为树；插件系统；你用哪一个。

### Q45. 行为树（Behavior Tree）相比状态机的好处？（BT vs FSM）★★
**答案要点**
- 节点类型：Sequence、Fallback（Selector）、Parallel、Decorator、Action/Condition；
- 模块化、可复用、易于扩展恢复逻辑，不会出现状态机的"转移爆炸"；
- 缺点：需要设计 tick 机制与黑板（blackboard）数据。
**English short answer**
A behaviour tree composes small reusable actions and conditions with sequences and fallbacks, so adding a recovery step does not need rewiring transitions as a state machine would.
**追问方向**：反应性；黑板。

### Q46. MoveIt2 简述？（MoveIt2）★★
**答案要点**
- 机械臂运动规划框架：Planning Scene（碰撞世界）、规划器（OMPL、Pilz 工业运动、CHOMP/STOMP）、运动学插件（KDL、TRAC-IK、IKFast）、时间参数化（TOTG）、`MoveGroup` 接口、Servo（实时遥控，速度级）；
- MoveIt Servo 适合遥操作：笛卡尔速度 → 关节速度（含奇异点减速、关节限位）。
- 与 P2 的关系：P2 的 IK 用 MuJoCo/Mink，而非 MoveIt；可说明为何（更灵活的约束与任务加权）。
**English short answer**
MoveIt2 provides planning scenes, planners, kinematics plugins and time parameterisation for arms, and MoveIt Servo for real-time Cartesian teleoperation with singularity slow-down and joint limits. In my teleoperation work I used a Mink-based QP IK so I could weight tasks and constrain velocities directly.
**追问方向**：Servo 的奇异处理；与 Mink 对比。

### Q47. ROS 包与构建：colcon/ament？（Build system）★★
**答案要点**
- 包含 `package.xml`（依赖）、`CMakeLists.txt`（C++）或 `setup.py`（Python）；`colcon build --symlink-install`；`source install/setup.bash`；
- 依赖用 `rosdep`；常见错误：忘记 source、依赖未声明、消息包未生成接口。
**English short answer**
A package declares its dependencies in package.xml and builds with CMake or setup.py through colcon. The usual mistakes are forgetting to source the workspace or missing dependencies.
**追问方向**：overlay/underlay。

### Q48. ROS 里的单位与坐标常见 bug？（Common bugs）★★
- 角度度/弧度混用；四元数顺序（ROS: x,y,z,w；Eigen 构造 w,x,y,z）；
- frame_id 填错；Twist 的坐标系（base_link）；时间戳 now vs 采样；
- 左右轮反装；
- QoS 不匹配；节点名/话题名冲突。
**English short answer**
Typical bugs are degrees versus radians, quaternion component order between ROS and Eigen, wrong frame ids, timestamps set to the current time instead of acquisition time, and QoS mismatches.
**追问方向**：如何系统验证。

### Q49. 单元测试与回归？（Testing）★
**答案要点**
- gtest/pytest、launch_testing；把 bag 当回归数据；仿真 CI；
- 控制算法先在 MuJoCo/Gazebo 验证，再上实机；每次改动保留日志对比。
**English short answer**
I test pure algorithm code with unit tests, replay recorded bags for regression, and validate controllers in simulation before the real robot.
**追问方向**：硬件在环。

### Q50. Docker/部署与启动管理？（Deployment）★
**答案要点**
- 容器化环境一致性；systemd 管理开机启动；日志轮转；
- OpenHarmony 平台上的部署方式不同，见下节 P6。
**English short answer**
I use containers or systemd units for reproducible deployment, with log rotation so the disk never fills on the robot.
**追问方向**：设备权限（CAN、USB）。

---

## 八、多机器人与分布式

### Q51. 多机器人通信怎么设计？（Multi-robot communication）★★★
**答案要点**
- 拓扑：集中式（调度中心）/去中心（点对点）；
- ROS2：每台机器人独立 Domain 或统一 Domain + 命名空间；Discovery Server；Zenoh 桥接；
- 需要同步的信息：设备状态、任务、位姿；一致性与时间同步；
- P6：基于 **M-DDS** 实现设备发现、能力注册、状态同步和任务分发（CV）。
- 我们的做法（典型设计，以实机为准）：主题按设备状态、能力、任务、反馈划分；状态用 best-effort 加最新值，任务与指令用 reliable；设备上线时广播并注册能力，用心跳判断离线；能力记录包含设备 ID、类型、接口、参数范围、坐标系与版本；任务分发采用确认加超时重试，超过次数后上报失败。
**English short answer**
For multi-robot systems I separate robots by namespace or domain, use discovery that works over the network in use, and share only what is needed: device state, capability, and task messages. In my platform work, I implemented device discovery, capability registration, state synchronisation and task dispatch on top of M-DDS.
**追问方向**：任务分发的可靠性；设备离线检测。

### Q52. 设备发现与能力注册是什么？（Discovery and capability registration）★★
**答案要点**
- 发现：设备上线广播/订阅；能力注册：设备声明"我有哪些传感器/执行器/控制接口、参数范围、坐标系"；上层按能力选择设备；
- 好处：硬件无关，控制代码复用（P6：unified capability model）；
- 需要：心跳、版本、有效性。
**English short answer**
Discovery tells the system a device is online, and capability registration describes what it can do, with its interfaces and limits. Upper layers then choose devices by capability rather than by model, which makes motion code reusable across hardware.
**追问方向**：设备下线；能力版本。

### Q53. 统一运动接口（HAL）怎么设计？（Unified motion interface）★★★
**答案要点**
- 抽象层：Sensor / Actuator / Controller 的统一能力模型；上层只依赖接口（如 `setVelocity`、`setJointTarget`、`getState`），具体驱动实现隔离硬件差异；
- ROS/ROS2 命令映射：`cmd_vel`/`JointTrajectory` → 统一接口 → 低层驱动（CAN/串口/厂商 SDK）；
- 注意：单位/坐标系/限位在接口层统一；错误码和降级；C++ 虚基类/插件；
- CV：*built a unified capability model for sensors, actuators and controllers using C++ on OpenHarmony and mapped ROS/ROS2 motion commands to low-level drivers*。
- 我们的做法：以 Sensor / Actuator / Controller 三类虚基类为核心，提供 `getState`、`setVelocity`、`setJointTarget` 等方法；针对不同硬件各写一个驱动适配（如 CAN、串口、厂商 SDK）；命令映射示例：`cmd_vel` 经接口层统一单位与限位后转换为底层速度目标（典型做法）。
**English short answer**
I defined common interfaces for sensors, actuators and controllers, so upper layers depend only on the abstract capability, and each hardware driver implements it. ROS and ROS2 motion commands are mapped to these interfaces and then down to the low-level drivers, which let the same motion-control code run on different hardware.
**追问方向**：接口抽象的代价；如何统一单位与坐标。

### Q54. 设备状态同步中的一致性问题？（State consistency）★★
**答案要点**
- 多源状态时间戳不同；网络延迟；乱序；需要统一时间基准、版本号/序号、有效性判断；
- 控制层使用"快照" + 数据新鲜度；
- P6：*giving the motion-control and task layers a consistent, trusted robot state*。
**English short answer**
State from different sources arrives at different times, so I stamp everything on a common time base, check validity and freshness, and give the control layer a consistent snapshot rather than raw, mixed-age values.
**追问方向**：时钟同步；时延容忍。

---

## 九、仿真

### Q55. Gazebo、MuJoCo、Isaac Sim 的对比？（Simulators comparison）★★★
| | Gazebo (gz sim) | MuJoCo | Isaac Sim / Isaac Lab |
|---|---|---|---|
| 物理 | ODE/DART/Bullet 等多引擎 | 专用高精度接触，凸接触+软约束 | PhysX（GPU） |
| 强项 | ROS 集成、传感器插件、世界编辑 | 速度快、接触稳定、适合控制/RL/优化 | GPU 并行、高保真渲染、大规模 RL |
| 弱项 | 接触精度一般 | 渲染较简、ROS 需桥接 | 资源重、依赖 NVIDIA GPU |
| 典型用 | 导航系统联调 | 机械臂/腿足控制、IK 验证 | 强化学习、合成数据 |

- 项目：P2 用 MuJoCo/Mink 做约束 IK。
**English short answer**
Gazebo has the best ROS integration and sensor plugins, so it is good for navigation system testing. MuJoCo has fast, accurate contact dynamics and is popular for control and learning. Isaac uses GPU physics and rendering for massively parallel reinforcement learning and synthetic data, but needs heavier hardware.
**追问方向**：为什么选 MuJoCo 做 IK；sim 速度。

### Q56. 仿真与实机差距（sim2real gap）从哪来？（Sim2real gap）★★★
**答案要点**
- 动力学参数（质量、摩擦、电机特性）、执行器模型（延迟、带宽、力矩-速度曲线）、传感器噪声/延迟、接触模型、传输延迟、地面特性；
- 缩小：系统辨识、域随机化、仿真加延迟/噪声、实机微调；
- 实践：仿真验证逻辑与稳定性趋势，不当作最终数值。
**English short answer**
The gap comes from imperfect dynamics parameters, actuator models, sensor noise and latency, and contact modelling. I reduce it by identification, adding noise and delay in simulation, and domain randomisation, and I treat simulation as a check of logic, not of final numbers.
**追问方向**：域随机化；执行器网络。

### Q57. 用 MuJoCo 做机械臂 IK 验证的好处？（MuJoCo for IK）★★
**答案要点**
- 模型加载 URDF/MJCF；快速计算雅可比 `mj_jacSite`；Mink 在其上构建任务；无需实时 GUI；可做碰撞检查、关节限位验证；
- 先验证无奇异、速度不过大，再上实机。
**English short answer**
MuJoCo gives fast kinematics and Jacobians from the robot model, and Mink builds IK tasks on top of it. I can verify limits, collisions and singularity behaviour in simulation before moving the real arm.
**追问方向**：模型与实机差异。

### Q58. 强化学习与仿真关系（简述）？（RL and simulation）★
**答案要点**
- 大规模并行仿真（Isaac Lab、MJX）训练策略 → 部署到实机；需要观测对齐、动作延迟、安全层；
- 本人经验以实践为主（PS），理论基础想补；详见 [强化学习八股](../02_人工智能/05_强化学习八股.md)。
**English short answer**
Policies are trained in massively parallel simulation and then deployed with matched observations, action latency and a safety layer. My own experience is practical, and I want to build deeper theory in the master's programme.
**追问方向**：策略输出如何过安全链（P6 的 action-proposal→policy-check）。

### Q59. 仿真中怎么模拟延迟与噪声？（Injecting latency/noise）★
**答案要点**
- 命令队列延迟 $k$ 步；传感器加高斯噪声+偏置；执行器低通；
- 用于检验控制器鲁棒性。
**English short answer**
I add a delay buffer on the commands, Gaussian noise and bias on the sensors, and a low-pass on the actuators to test how robust the controller is.
**追问方向**：延迟取值依据。

### Q60. Gazebo 与 ROS2 的接口（gz_ros2_control）？（Gazebo-ROS2 bridge）★
**答案要点**
- `ros_gz_bridge` 转话题；`gz_ros2_control` 插件把仿真关节暴露为 ros2_control 硬件接口，实机仿真同控制器配置。
**English short answer**
A bridge maps Gazebo topics to ROS2, and the gz_ros2_control plugin exposes simulated joints as a hardware interface, so the same controllers can run in simulation and on hardware.
**追问方向**：时钟同步。

---

## 十、软件架构与安全执行链

### Q61. "AI 决策如何变成物理动作"——安全执行链是什么？（Safe execution chain）★★★
**答案要点**
- 链路：**action proposal（行动提案）→ policy check（策略检查）→ execution（执行）→ state feedback（状态反馈）**；高层任务计划（来自规划器/LLM/Agent）不能直接下发给机器人；
- 各阶段：
  1. 提案：结构化动作（目标、参数、前置条件）；
  2. 策略检查：安全规则（速度/力/区域/互斥/资源/权限）、状态有效性、冲突检测；不通过则拒绝或降级；
  3. 执行：转换为底层命令，带超时/限幅；
  4. 反馈：执行结果与状态回传，闭环验证（成功/失败/中止）、触发重规划或恢复；
- 价值：在不确定的 AI 决策与确定性的物理执行间加一层可审计的安全边界。
- CV 对应：P6 *gated task plans through an action-proposal, policy-check, execution and state-feedback chain*；PS 主线。
- 我们的做法（典型规则，以实机为准）：策略检查包含速度与加速度上限、允许区域、设备互斥与资源冲突、状态有效性；不通过时拒绝或降级并返回原因给上层；状态反馈包含执行结果（成功、失败、中止）与当前状态，并设置超时，超时视为失败，触发重规划或恢复。
**English short answer**
A high-level plan never goes straight to the robot. It first becomes a structured action proposal, a policy check validates it against safety rules and the current state, only then is it executed with limits and timeouts, and the state feedback closes the loop. This puts an auditable safety boundary between an uncertain AI decision and the physical actuators.
**追问方向**：规则从哪来；LLM 输出幻觉怎么被拦；反馈如何触发重规划。

### Q62. 任务分发和状态机怎么设计？（Task dispatch）★★
**答案要点**
- 任务状态：Pending → Assigned → Running → Succeeded/Failed/Cancelled；
- 分发：按设备能力匹配 + 设备状态；确认（ACK）、超时重试、幂等；
- 异常：设备离线→重分配；
- 状态机或行为树。
**English short answer**
A task goes through states from pending to assigned, running and a final state. The dispatcher matches tasks to devices by capability and current state, uses acknowledgements and timeouts, and reassigns when a device goes offline.
**追问方向**：幂等；重复分发。

### Q63. 软件分层设计的原则？（Layering principles）★★
**答案要点**
- 硬件抽象层 → 驱动 → 状态估计/管理 → 控制 → 任务/策略；单向依赖；每层只暴露必要接口；
- 实机/仿真可替换；配置外部化；日志与可观测性。
**English short answer**
I layer the software from hardware abstraction up to tasks, with dependencies pointing downward only, configuration kept outside the code, and logging at every boundary. That keeps simulation and hardware interchangeable.
**追问方向**：层间数据结构。

### Q64. 如何在面试中讲"ROS 经验"？（Telling your ROS story）★★★
**答案要点**
- 抓三个点：① 你负责的链路（P2 ROS2-CAN、P1 状态处理+CAN、P5 导航栈、P6 命令映射与 M-DDS）；② 你遇到并解决的问题（时间戳/坐标系/QoS/延迟）；③ 你如何验证（bag 回放、可视化、实机测试）。
**English short answer**
I focus on three things: the link I owned, for example the ROS2 to CAN chain; a concrete problem I solved, such as timestamp or frame inconsistencies; and how I verified it, using bag replays, plots and hardware tests. For example: the ROS2 to CAN chain with timeouts and limits, timestamp and frame alignment across sensors, and replay plus hardware tests to verify. （例如：带超时与限幅的 ROS2-CAN 链、多传感器时间戳与坐标系对齐、回放加实机测试验证。）
**追问方向**：被追问细节时的诚实回答模板。

---

## 十一、本章要点速记

1. ROS2 = DDS 去中心化 + QoS + lifecycle + 实时友好 + 跨平台。
2. 传感器流 best-effort；配置/地图 reliable+transient_local；订阅要求不能高于发布承诺。
3. 连续流用 topic，长任务用 action，短调用用 service，配置用 parameter。
4. 控制回路放定时器，读最新快照并检查过期，别塞进订阅回调。
5. TF 是树；`map→odom→base_link`；外推错误先查时间源。
6. ROS2 非开箱实时：硬实时放 MCU，ROS2 做软实时。
7. ros2_control：read → update → write；控制器与硬件解耦。
8. ROS2-CAN：SocketCAN + 限幅 + 超时 + 心跳 + 反馈解析。
9. Nav2 = 服务器 + BT；MoveIt Servo 做实时遥控。
10. 仿真：Gazebo 联调、MuJoCo 控制/IK、Isaac 大规模 RL；sim2real 要加延迟噪声。
11. AI 决策必须经 提案 → 策略检查 → 执行 → 反馈 的链条。

## 十二、自测题

1. QoS 中"订阅者不能要求高于发布者"具体指什么？举例。
2. 为什么控制回路不放在订阅回调中？
3. `Lookup would require extrapolation` 通常怎么排查？
4. ros2_control 的一个控制周期里发生什么？
5. ROS2 默认是否实时？怎样做到实时友好？
6. 说出 Nav2 中至少 4 个服务器。
7. 安全执行链四个阶段分别做什么？
8. 为什么控制命令话题不宜使用很深的队列？

**答案提示**：1 见 Q10；2 见 Q6；3 见 Q27；4 见 Q36；5 见 Q31；6 见 Q44；7 见 Q61；8 见 Q9（旧命令积压造成延迟）。

## 十三、相关链接

- 上一篇：[04 路径规划与导航八股](./04_路径规划与导航八股.md)；下一篇：[06 嵌入式与机电八股](./06_嵌入式与机电八股.md)
- 项目：[P1](../../01_项目课程/P1_AIRS球形机器人平衡控制/README.md)、[P2](../../01_项目课程/P2_未来动力双臂遥操作与差速底盘/README.md)、[P5](../../01_项目课程/P5_车式移动机器人自主导航/README.md)、[P6](../../01_项目课程/P6_深开鸿M-Robots平台软件/README.md)
- 基础：[ROS1/ROS2 核心概念](../../02_基础知识/09_机器人软件/04_ROS1_ROS2核心概念.md)、[DDS](../../02_基础知识/09_机器人软件/05_DDS与分布式通信.md)
