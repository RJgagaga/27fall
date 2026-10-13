# 04 ROS/ROS2 运动命令映射与跨硬件复用

> 一句话定位：CV 第一条 bullet 的后半句是 "mapped ROS/ROS2 motion commands to low-level drivers"。本章把"命令映射"拆开讲清：ROS/ROS2 的运动消息长什么样，怎么变成统一能力模型里的命令，再变成各类底盘/关节的底层指令；以及"跨硬件复用"怎么用契约测试来证明，而不是只说一句口号。

**本章学完你能……**

- 说出 `Twist`（线速度 + 角速度）的字段含义、单位、坐标约定（REP-103），并指出它**没有**什么（时间戳、有效期、来源）。
- 把 `Twist` 映射为差速、麦克纳姆、阿克曼（car-like）、三轮全向底盘的底层指令，手算数值。
- 区分 ROS 命令映射中的 5 个环节：接收、校验、转换、限幅/平滑、下发，以及每个环节的失败处理。
- 讲出 ROS/ROS2 接入方式的几种典型选择（桥接节点 / 原生客户端 / 话题适配），并说明本项目选择桥接的理由。
- 设计"契约测试"证明同一份运动代码在多种硬件适配器上行为一致。

**前置知识**

- [ROS1/ROS2 核心概念](../../02_基础知识/09_机器人软件/04_ROS1_ROS2核心概念.md)
- [差速、麦克纳姆、全向轮与车辆运动学](../../02_基础知识/02_力学与建模/04_差速_麦克纳姆_全向轮与车辆运动学.md)
- 上一章：[03 统一能力模型与设备抽象](./03_统一能力模型与设备抽象.md)

**目录**

1. 要解决什么问题
2. ROS/ROS2 运动消息速览
3. 命令映射的五个环节
4. Twist 到各类底盘的数学
5. 关节类命令的映射
6. 接入方式的典型选择
7. 平滑、限幅与超时
8. 教学版映射器代码
9. 跨硬件复用的验证：契约测试
10. 常见错误与故障现象
11. 面试问答
12. 要点速记 / 自测题 / 链接

---

## 1. 要解决什么问题

ROS/ROS2 生态里存在大量"产生运动命令"的软件：遥控节点、导航栈（Nav2/move_base）、跟踪控制器、示教工具。它们输出的通常是标准消息，比如 `geometry_msgs/Twist` 发到 `/cmd_vel`。这些软件不知道、也不应该知道底盘是什么构型、电机用什么协议。

平台要做的是：**让这些现成的 ROS/ROS2 软件不用修改，就能驱动 OpenHarmony 上的各种硬件。** 这需要一层"命令映射"：

```text
 ROS/ROS2 世界                   平台                        硬件世界
┌───────────────┐   Twist   ┌───────────────┐  PlanarVelocity ┌──────────────┐  轮速/总线帧
│ Nav2 / 遥控 /  │ ───────▶ │ 映射器         │ ──────────────▶ │ 设备适配器    │ ───────────▶ 电机
│ 跟踪控制器     │          │ 校验·转换·限幅 │  (统一能力接口)  │ (差速/麦轮/...)│
└───────────────┘          └───────────────┘                  └──────────────┘
```

这也解释了为什么 03 章要先建立统一能力模型：映射的目标是"统一能力接口"，而不是某个具体底盘。**两段式映射**（ROS 消息 → 统一命令 → 硬件指令）把组合爆炸降成加法：$N$ 种 ROS 消息 $\times$ $M$ 种硬件，只需要 $N+M$ 个转换，而不是 $N\times M$。

---

## 2. ROS/ROS2 运动消息速览

### 2.1 Twist

`geometry_msgs/Twist` 是最常见的速度命令：

| 字段 | 类型 | 含义 | 单位 |
|---|---|---|---|
| `linear.x` | float64 | 沿 body 坐标系 $x$（前）的线速度 | m/s |
| `linear.y` | float64 | 沿 $y$（左）的线速度 | m/s |
| `linear.z` | float64 | 沿 $z$（上）的线速度 | m/s |
| `angular.x/y` | float64 | 绕 $x/y$ 轴角速度（地面机器人通常不用） | rad/s |
| `angular.z` | float64 | 绕 $z$ 轴角速度，逆时针为正 | rad/s |

坐标约定遵循 ROS 的 REP-103（SI 单位、右手系：$x$ 前、$y$ 左、$z$ 上）。地面平面机器人只用 `linear.x`、`linear.y`、`angular.z` 三个自由度，其余应为 0。

### 2.2 Twist 没有什么

这是面试加分点——**Twist 是个"裸"速度**：

- 没有时间戳（ROS2 后来有 `TwistStamped`，带 header）；
- 没有有效期（发一次是否一直有效，取决于下游如何实现）；
- 没有来源/优先级（谁发的，是否被覆盖）；
- 没有 frame 声明（默认 body 坐标系，是约定而非字段）。

所以映射器要**补上这些信息**：接收时打上本地时间戳、绑定超时、标注来源，才能接入安全链（08 章）和状态管理（05 章）。

### 2.3 其他常见运动相关消息（典型）

| 消息 | 用途 | 平台侧映射目标 |
|---|---|---|
| `Twist` / `TwistStamped` | 平面速度 | 平面速度能力 |
| `Odometry` | 里程计（位姿 + 速度） | 状态输出（非命令） |
| `JointState` | 关节位置/速度/力矩的状态 | 关节状态传感器能力 |
| `JointTrajectory` | 关节轨迹点序列 | 关节轨迹执行能力 |
| 自定义 Action（如导航目标） | 带反馈和取消的长时任务 | 任务层（07 章），不是直接运动命令 |

> **注意**：Action（动作）和"我们的 action-proposal"不是一回事。ROS2 的 Action 是带目标/反馈/结果的异步接口；P6 的 action-proposal 是安全链第一阶段的"动作提案"。面试时要避免混淆，必要时主动说明："Not to be confused with ROS2 actions."

---

## 3. 命令映射的五个环节

```mermaid
flowchart LR
  R[1 接收 订阅话题] --> V[2 校验 格式 有限值 来源]
  V --> C[3 转换 到统一命令]
  C --> L[4 限幅与平滑]
  L --> D[5 下发 到适配器]
  V -.拒绝.-> X[丢弃并记录]
  D -.失败.-> E[上报状态与错误]
```

| 环节 | 输入 → 输出 | 做什么 | 失败怎么办 |
|---|---|---|---|
| 1 接收 | ROS 消息 → 带接收时间的内部对象 | 订阅、反序列化、记录接收时刻 | 订阅失败/无数据 → 超时看门狗触发停车 |
| 2 校验 | 内部对象 → 通过/拒绝 | 非 NaN/Inf、分量是否被设备支持、来源是否有权限 | 拒绝并计数、日志；严重则触发降级 |
| 3 转换 | Twist → `PlanarVelocity` | 字段对应、坐标/单位核对；`TwistStamped` 的 frame 转换到 body | 无法转换（未知 frame）→ 拒绝 |
| 4 限幅/平滑 | 命令 → 受限命令 | 速度上限、加速度限幅、斜坡 | 超限则钳位并标记"被限幅" |
| 5 下发 | 受限命令 → 适配器调用 | 调用统一能力接口 | 适配器返回错误 → 上报、必要时停车 |

关键点：映射器**不是透明管道**。它位于 ROS 世界与安全链之间，是"把 ROS 命令变成可审计的提案/命令"的位置。当 ROS 命令来自 Nav2 这类受信的低层控制器，且已被平台配置为直通场景时，策略可以更宽松；当命令来自上层任务计划或 AI，必须经过完整安全链（08 章）。本项目口径：`/cmd_vel` 这类连续速度命令走"直通 + 网关限幅"——不逐条做完整提案流程，但执行网关仍强制有效期、超时看门狗、速度与加速度限幅；而导航目标、机械臂动作、模式切换等离散任务命令必须走完整提案链。直通与门控的边界由设备配置中的 `passthrough: [cmd_vel]` 白名单决定。

---

## 4. Twist 到各类底盘的数学

统一命令：$\mathbf{u}=(v_x,v_y,\omega_z)$。设轮半径 $r$（m）。

### 4.1 差速（Differential drive）

参数：轮距 $L$。
$$\omega_L=\frac{v_x-\omega_z L/2}{r},\qquad \omega_R=\frac{v_x+\omega_z L/2}{r}$$
$v_y$ 无法实现（非完整约束）。

**手算（教学示例）**：$r=0.05$，$L=0.4$，$v_x=0.3$，$\omega_z=0.5$ → $\omega_L=4$，$\omega_R=8$ rad/s（详见 03 章 8.1）。

### 4.2 麦克纳姆（Mecanum, X 型）

见 03 章 8.2 的四轮公式；符号表需按实际安装标定。

### 4.3 阿克曼 / car-like（前轮转向）

参数：轴距 $L_w$（前后轴距离）。自行车模型（bicycle model）下，转弯半径 $R=v/\omega$，转向角：
$$\delta=\arctan\left(\frac{L_w\,\omega_z}{v_x}\right)\quad(v_x\neq 0)$$

推导：自行车模型中后轴中心沿圆弧，转弯半径 $R$ 满足 $\tan\delta=L_w/R$；而 $R=v_x/\omega_z$，所以 $\tan\delta=L_w\omega_z/v_x$。

**手算（教学示例）**：$L_w=0.6$ m，$v_x=0.5$ m/s，$\omega_z=0.4$ rad/s：
$\tan\delta=0.6\times0.4/0.5=0.48$，$\delta=\arctan 0.48\approx 25.6^\circ$。

边界情况：

- $v_x\to 0$ 但 $\omega_z\neq0$：$\delta\to\pm90^\circ$，物理上做不到（阿克曼车无法原地转）。映射器应**返回不支持/钳位到最大转向角**，并让能力描述声明 `min_turning_radius`。
- $v_x<0$（倒车）：转向角符号约定要在适配器内处理清楚。
- 超出最大转向角 $\delta_{max}$：反过来限制 $\omega_z\le v_x\tan\delta_{max}/L_w$。

这再次说明**能力描述里需要最小转弯半径**——同样是"平面速度"能力，阿克曼的可行域和差速的不同，这是上层规划要读能力参数的原因。

### 4.4 三轮全向（Omni, 120° 间隔）

轮 $i$ 的安装方位角为 $\theta_i$（沿径向与 $x$ 轴夹角），底盘中心到轮的距离 $R$。一种常见的运动学（不同厂家符号约定不同）：
$$v_i=-\sin\theta_i\,v_x+\cos\theta_i\,v_y+R\,\omega_z$$

**手算（教学示例）**：$\theta=0^\circ,120^\circ,240^\circ$，$R=0.15$，$v_x=0.3$，$v_y=0$，$\omega_z=0$：
$v_1=0$，$v_2=-\sin120^\circ\times0.3\approx-0.260$，$v_3=-\sin240^\circ\times0.3\approx+0.260$ m/s。

### 4.5 小结表

| 底盘 | 可实现自由度 | 参数 | 特殊情形 |
|---|---|---|---|
| 差速 | $v_x,\omega_z$ | $r,L$ | $v_y$ 不支持 |
| 麦轮 | $v_x,v_y,\omega_z$ | $r,l_x,l_y$、轮符号 | 打滑使里程计误差大 |
| 阿克曼 | $v_x$ + 受限的 $\omega_z$ | $L_w,\delta_{max}$ | $v_x=0$ 无法转向 |
| 三轮全向 | $v_x,v_y,\omega_z$ | $\theta_i,R$ | 符号/排序需标定 |

---

## 5. 关节类命令的映射

机械臂/云台等的命令通常是关节空间的：位置、速度或力矩。典型映射：

| ROS 侧 | 统一能力 | 适配器职责 |
|---|---|---|
| `JointTrajectory` 点序列 | 关节轨迹执行能力 | 按时间插值、检查关节限位/速度/加速度、转成电机位置环/速度环指令 |
| 关节位置命令 | 关节位置能力 | 单位换算（rad ↔ 电机编码器计数、减速比）、零点偏置 |
| 关节力矩命令 | 关节力矩能力 | 电流/力矩换算、限流 |

关节映射里最容易出事的是：**零点偏置、减速比、方向符号、限位**。这些参数应放在配置（类似 HCS/配置文件）里，不是硬编码；更重要的是，限位必须在适配器内强制（和 03 章"最后一道防线"同理）。

任务级命令（如"导航到目标点"）不是运动接口的事，是任务层的事，见 07 章。

---

## 6. 接入方式的典型选择

ROS/ROS2 如何"接入"平台，业界典型有三种做法，各有利弊：

| 方式 | 做法 | 优点 | 缺点 |
|---|---|---|---|
| 桥接节点 | 一个 ROS2 节点订阅 `/cmd_vel`，调用平台的统一能力接口（IPC/DDS） | 对 ROS 侧无侵入，部署简单 | 多一跳延迟，节点崩溃需监控 |
| 原生客户端库 | 平台进程直接链接 ROS2 客户端库（rclcpp）订阅 | 少一跳 | 平台依赖 ROS 库，构建与版本耦合 |
| 话题/DDS 适配 | 因 ROS2 底层是 DDS，在 DDS 层直接对接 Topic | 同为 DDS 体系，天然贴合 | 需要对齐类型定义与 QoS |

本项目的做法：**桥接适配节点**。ROS2 侧是一个 rclcpp 节点（订阅 `/cmd_vel`，发布 `/odom`、`/imu` 等），ROS1 通过 ros1_bridge 接入同一个节点，所以 ROS 与 ROS2 共用一份命令映射逻辑，差别只在桥接层。选桥接而不是原生客户端库的理由：不把 ROS 依赖带进 OpenHarmony 的核心进程，ROS 侧升级或替换不影响能力模型。

面试里即便不能说细节，也可以讲"选择依据"：看**是否能容忍多一跳延迟、构建依赖是否可接受、是否需要同时支持 ROS1 与 ROS2**。ROS1 与 ROS2 的通信机制差异很大（ROS1 用 TCPROS 与 master，ROS2 用 DDS），"同时支持 ROS/ROS2" 通常意味着为两者各写一个前端、共用一个映射核心。

---

## 7. 平滑、限幅与超时

### 7.1 为什么要平滑

遥控器或上层控制器可能给出阶跃命令（0 → 0.5 m/s 瞬间）。底盘电机不可能瞬时达到，强行阶跃会造成：轮子打滑、电流冲击、里程计误差、机械冲击。所以映射/适配器层通常加**加速度限幅**（斜坡）。

**推导：加速度限幅**。设周期 $T_s$（s），最大加速度 $a_{max}$，上一周期输出 $v_{k-1}$，目标 $v^*_k$，则：
$$v_k=v_{k-1}+\mathrm{clip}\big(v^*_k-v_{k-1},\,-a_{max}T_s,\,+a_{max}T_s\big)$$
每个周期速度变化不超过 $a_{max}T_s$。

**手算（教学示例）**：$a_{max}=1.0$ m/s$^2$，$T_s=0.02$ s（50 Hz，示例），$v_{k-1}=0$，目标 $0.5$：单周期最大变化 $0.02$，所以 $v_k=0.02$；到达 0.5 需 $0.5/0.02=25$ 个周期 $=0.5$ s。

### 7.2 命令超时（看门狗）

Twist 没有有效期。典型做法是映射器/适配器给每条命令**绑定超时**：若在 $T_{cmd}$ 内没收到新命令，就把目标速度置零并减速（而不是保持上一个速度）。这就是 ROS 世界中常见的 "cmd_vel timeout"。

超时时间要权衡：太短 → 网络抖动就频繁停车；太长 → 上层死掉后机器人继续冲出去。取值应由**最大速度和制动距离**决定，而不是拍脑袋：

$$T_{cmd}\cdot v_{max}+\frac{v_{max}^2}{2a_{brake}}\le d_{safe}$$

**手算（教学示例）**：$v_{max}=0.5$ m/s，$a_{brake}=1.0$ m/s$^2$，$d_{safe}=0.3$ m：制动距离 $0.25/2=0.125$ m；留给超时的距离 $0.3-0.125=0.175$ m；$T_{cmd}\le0.175/0.5=0.35$ s。

本项目取值（设计级，随设备类型在配置文件中加载）：命令看门狗约 200 ms（约为 20 Hz 下发周期的 4 倍）；默认档位最大线速度 0.8 m/s，线加速度限幅 1.0 m/s²，角加速度限幅 2.0 rad/s²。取值依据是"由制动距离 $v^2/(2a)$ 与网络抖动反推"：0.8 m/s 下以 1.0 m/s² 制动，停车距离约 0.32 m。被问到时说 "roughly, and derived from stopping distance"。

---

## 7.3 一条命令的"速率账本"：下发周期、限幅周期与执行周期

很多映射问题其实是三个周期没有对齐造成的。把它们放在一张账本上看：

| 环节 | 典型周期 | 说明 |
|---|---|---|
| 上层下发 `/cmd_vel` | 50 ms（20 Hz） | 本项目默认下发频率；可到 20 ms（50 Hz） |
| 映射与限幅线程 | 20 ms（50 Hz）或随到达事件 | 平滑按固定周期 $T_s$ 推进，而不是按"命令到来的时刻" |
| 适配器到 CAN 发送 | 10 ms（100 Hz） | 重复发送最新目标，丢一帧不致命 |
| MCU 电机控制环 | 1 ms 到 10 ms（100～1000 Hz） | 驱动层，平台只给目标 |

**要点 1：平滑必须按固定周期推进**。若限幅只在命令到达时计算，那么 20 Hz 下发会让加速度限幅退化为 $a_{max}\cdot 0.05$ 的阶梯，而上层偶尔多发一条命令就会让速度提前爬升。正确做法是平滑线程自己有周期 $T_s$，每个周期取"最新命令（未过期）"作目标，再按 $v_k=v_{k-1}+\mathrm{clip}(\cdot)$ 推进。

**要点 2：保持与超时要配合**。执行周期（10 ms）比下发周期（50 ms）快，意味着中间 4 个周期要"保持上一个目标"。保持有上限：本项目的看门狗约 200 ms（4 个下发周期），且采用"连续 2 次判定"，所以最长约 250 ms 内会进入减速停车。手算停车距离见 07 章公式：$d=vT_w+\frac{v^2}{2a}$，以 $v=0.8$、$T_w=0.2$、$a=1.0$ 计，$d=0.16+0.32=0.48$ m。该数值要小于设备档位里的安全距离。

**要点 3：下发周期与看门狗的比例**。看门狗周期取下发周期的 4 倍，是在"网络抖动不频繁误停"与"失联后不冲太远"之间折中。如果下发频率降到 10 Hz，同样 200 ms 只有 2 个周期，抖动一次就停车；这时要么把下发频率提回去，要么把看门狗放宽，并用停车距离公式重新校核。

**要点 4：混合源的有效期**。操作员遥控（人手，事件稀疏）与规划器（定频）混用时，用同一个看门狗会误伤遥控。典型做法是每个来源有自己的 `valid_until_ms`，网关按来源仲裁，手动模式永远优先（08 章）。

## 7.4 映射阶段容易出现的四类错误

1. **符号与方向**：差速里 $\omega_z>0$ 是逆时针，左轮应慢、右轮应快；符号写反在仿真里看着"转了"，实机上方向相反。契约测试里必须有"原地左转"用例。
2. **单位**：话题是 m/s、rad/s，适配器内部若用 rpm 或每秒编码器计数，换算只能在适配器内部发生一次，且写成带单位的类型或命名。
3. **坐标系**：`Twist` 的参考系默认是 `base_link`；有的上层把速度写在 `odom` 系，没有转换就下发。映射器要检查 `frame_id`，不匹配时拒绝而不是猜。
4. **不可达命令**：阿克曼车辆给出 `vx=0` 且 `wz≠0`，公式里 $\arctan(L_w\omega_z/v_x)$ 发散。应在映射阶段返回"不可达"，并在回执里给原因码，而不是钳位成一个看似合理的角度。

这些错误的共同点是：**映射层最该做的不是"算对"，而是"在不可能算对时明确说不"**。

---

## 8. 教学版映射器代码

> **教学示例，非项目源码。** 用纯 C++ 表示映射逻辑，不依赖 rclcpp，便于理解；接入 ROS2 时由订阅回调调用 `Map()`。

```cpp
#include <chrono>
#include <cmath>
#include <optional>

struct TwistMsg {              // mirrors geometry_msgs/Twist for planar robots
  double lx = 0, ly = 0, lz = 0;
  double ax = 0, ay = 0, az = 0;
};

struct PlanarVelocity { double vx = 0, vy = 0, wz = 0; };

struct MapResult {
  bool accepted = false;
  PlanarVelocity cmd;
  const char* reason = "";
};

class TwistMapper {
 public:
  TwistMapper(double max_lin, double max_ang, bool holonomic)
      : max_lin_(max_lin), max_ang_(max_ang), holonomic_(holonomic) {}

  MapResult Map(const TwistMsg& t) const {
    MapResult r;
    if (!std::isfinite(t.lx) || !std::isfinite(t.ly) || !std::isfinite(t.az)) {
      r.reason = "non-finite";
      return r;
    }
    // Planar robot: reject out-of-plane components instead of silently dropping.
    if (std::fabs(t.lz) > 1e-9 || std::fabs(t.ax) > 1e-9 || std::fabs(t.ay) > 1e-9) {
      r.reason = "out-of-plane component";
      return r;
    }
    if (!holonomic_ && std::fabs(t.ly) > 1e-9) {
      r.reason = "lateral velocity unsupported";
      return r;
    }
    r.cmd.vx = clip(t.lx, max_lin_);
    r.cmd.vy = clip(t.ly, max_lin_);
    r.cmd.wz = clip(t.az, max_ang_);
    r.accepted = true;
    r.reason = (r.cmd.vx != t.lx || r.cmd.vy != t.ly || r.cmd.wz != t.az)
                   ? "clamped" : "ok";
    return r;
  }

 private:
  static double clip(double x, double m) { return x > m ? m : (x < -m ? -m : x); }
  double max_lin_, max_ang_;
  bool holonomic_;
};

// Rate limiter: bounded change per control period.
inline double RampTo(double prev, double target, double a_max, double Ts) {
  const double step = a_max * Ts;
  const double d = target - prev;
  return prev + (d > step ? step : (d < -step ? -step : d));
}
```

注意这里的设计选择：对不支持的分量**显式拒绝并给出理由**，而不是静默丢弃——审计时才能知道"为什么这条 ROS 命令没执行"。这与 03 章第 11 节的"静默忽略"问题呼应。

---

## 9. 跨硬件复用的验证：契约测试

"复用"如果不能被验证，就只是口号。典型方法是**契约测试（contract test）**：写一组针对接口的测试，对每个适配器都跑一遍。

| 测试项 | 对每个适配器验证 |
|---|---|
| 零命令 | 发 $(0,0,0)$ → 所有轮速为 0 |
| 纯前进 | 发 $(v,0,0)$ → 运动学逆解后，**正向运动学**还原出 $v$（误差在容差内） |
| 纯旋转 | 发 $(0,0,\omega)$ → 还原出 $\omega$ |
| 限幅 | 发超大速度 → 输出被钳位到能力描述的上限 |
| 非有限值 | 发 NaN → 返回 `InvalidArgument`，轮速不变 |
| 停止幂等 | 连续 `Stop()` 多次 → 结果一致 |
| 超时 | 停止发命令超过 $T_{cmd}$ → 自动归零 |
| 生命周期 | Init 前调用返回 `NotReady`；Shutdown 后不可再用 |

**往返一致性（round-trip）**是最有力的一个：对任意命令 $\mathbf{u}$，逆运动学得到轮速 $\mathbf{w}$，再用正运动学算回 $\mathbf{u}'$，应满足 $\|\mathbf{u}-\mathbf{u}'\|<\epsilon$（在不被限幅的范围内）。

其中 on-hardware 的口径：差速底盘与阿克曼式底盘两类适配器都在台架实机上跑过同一套速度命令用例，验证方式是下发阶跃与斜坡速度命令，检查轮速反馈与限幅、超时行为是否符合契约。

---

## 10. 常见错误与故障现象

| 现象 | 常见原因 |
|---|---|
| 机器人原地转而不是前进 | 坐标轴/符号约定不一致；轮序错误 |
| 速度偏大/偏小固定倍数 | 轮半径、减速比、单位（mm vs m）配错 |
| 上层停止后车继续走 | 没有命令超时；超时被事件循环阻塞 |
| 起步抖动、轮子打滑 | 没有加速度限幅 |
| 倒车转向反了 | 阿克曼倒车转向角符号处理 |
| 偶发"命令被忽略" | 校验拒绝但没有日志；或消息 QoS 不匹配导致没收到 |
| ROS 节点和平台状态对不上 | 映射器没保存最近命令；状态时间戳与命令时间戳混用 |
| 横移命令无效 | 非完整底盘不支持 $v_y$，适配器静默丢弃 |

---

## 11. 面试问答

**Q1. How did you map ROS/ROS2 motion commands to the low-level drivers?**

> Two steps. First, the incoming command, for example a Twist, is validated and converted into a unified planar-velocity command, with limits applied. Second, the device adapter for the specific chassis converts that into wheel or joint commands and the bus protocol. This way N kinds of ROS commands and M kinds of hardware need N plus M converters rather than N times M.

译：两段式：ROS 命令 → 统一命令（校验、限幅），再由适配器 → 硬件指令；N+M 而非 N×M。

追问：Twist 没有时间戳怎么办？→ 接收时打本地时间戳并绑定超时；必要时使用 TwistStamped。

**Q2. How does a Twist become wheel speeds on a differential base versus a car-like base?**

> On a differential base I solve wheel speeds from the forward speed and the turning rate using the track width. On a car-like base I compute a steering angle from the wheelbase, with arctan of wheelbase times yaw rate over speed, and handle the zero-speed case as unsupported because a car cannot rotate in place.

追问：速度接近 0 时怎么办？→ 钳位到最大转向角、返回不支持，并把最小转弯半径写进能力描述。

**Q3. What happens if the ROS node publishing velocity crashes?**

> The command has no lifetime of its own, so the platform must add one. A command timeout drives the target velocity to zero and the base decelerates. I would choose the timeout from the maximum speed and braking distance rather than guess it.

**Q4. Is it a bridge or native integration?**

> In our setup it is a bridge-style adapter node. I'd rather not go into the exact integration, as the internship is ongoing, but the design choice is a trade-off between an extra hop and build coupling to the ROS libraries.

**Q5. How do you prove the code is really reusable across hardware?**

> Contract tests against the interface, run on every adapter, including a round-trip check — inverse kinematics to wheel speeds, forward kinematics back — plus checks on real hardware where available.

---

## 12. 本章要点速记

1. 两段式映射：ROS 消息 → 统一命令 → 硬件指令，复杂度 N+M。
2. Twist 是裸速度：无时间戳、无有效期、无来源，映射器必须补。
3. 映射五环节：接收、校验、转换、限幅/平滑、下发；每环节都有失败处理。
4. 差速、麦轮、阿克曼、三轮全向的数学各不同，能力描述需包含构型参数与约束。
5. 阿克曼：$\delta=\arctan(L_w\omega_z/v_x)$，$v_x\to0$ 不可行。
6. 加速度限幅：$v_k=v_{k-1}+\mathrm{clip}(\Delta,\pm a_{max}T_s)$。
7. 命令超时由制动距离反推，不是拍脑袋。
8. 复用用契约测试 + 往返一致性验证，不是口号。

## 自测题

1. Twist 里哪些字段对地面差速机器人有意义？其余应如何处理？
2. 手算：阿克曼 $L_w=0.8$，$v_x=1.0$，$\omega_z=0.5$ 的转向角。（答：$\tan\delta=0.4$，$\delta\approx21.8^\circ$。）
3. 为什么命令超时不能保持上一个速度？
4. 解释"N+M 而非 N×M"。
5. 设计一个往返一致性测试验证麦轮适配器。

## 相关链接

- 上一章：[03 统一能力模型与设备抽象](./03_统一能力模型与设备抽象.md)
- 下一章：[05 机器人状态管理](./05_机器人状态管理.md)
- 安全链：[08 安全执行链](./08_安全执行链.md)
- 基础：[ROS1/ROS2 核心概念](../../02_基础知识/09_机器人软件/04_ROS1_ROS2核心概念.md)
