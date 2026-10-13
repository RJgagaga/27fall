# 导航系统架构：move_base 与 Nav2

> **一句话定位**：前面几章是零件（定位、全局规划、局部规划、跟踪），本章讲它们怎样装成一个能"给个目标点就自己走过去"的系统：ROS1 的 move_base，以及 ROS2 的 Nav2。重点是接口、数据流、TF 树和失败恢复。

## 本章学完你能……

- 画出 move_base 的数据流和状态机（PLANNING / CONTROLLING / CLEARING），说清全局 / 局部规划器插件接口。
- 理解 `map→odom→base_link` 的 TF 树（REP-105）：谁发布哪一段、为什么这样分。
- 说明 costmap_2d 在系统里的位置，以及 recovery 行为的顺序与作用。
- 对比 Nav2 与 move_base：行为树、生命周期节点、服务器化插件，以及 Smac Hybrid-A*、控制器插件。
- 把 P5 的 EKF/AMCL + Hybrid A* + TEB + MPC 映射到该架构，并分清哪些模块是自行实现的、哪些是配置公开插件。

## 前置知识

[ROS1/ROS2 核心概念](../09_机器人软件/04_ROS1_ROS2核心概念.md)、[URDF、TF、坐标系实践](../09_机器人软件/07_URDF_TF_坐标系实践.md)、本组前四章。

## 目录
1. 整体数据流
2. TF 树
3. move_base
4. 规划器接口
5. recovery 行为
6. Nav2
7. 对比表
8. 在本项目里怎么用
9. 常见错误
10. 面试问答
11. move_base 状态机逐周期拆解
12. 规划器接口与 costmap_2d 分层
13. 恢复行为与车式底盘适配
14. TF 树深入
15. Nav2 深入：行为树与生命周期
16. move_base 与 Nav2：差异与本项目对应
17. 速记、自测、链接

---

## 1. 整体数据流

要解决的问题：一个目标位姿进来，一串速度 / 转向指令出去，中间有谁、传什么。

```text
 目标 goal ──▶ 导航核心 ──▶ cmd_vel / 转向指令 ──▶ 底盘驱动
                │  ▲
   全局规划器 ◀─┘  └─▶ 局部规划器(控制器)
        │                  │
   global costmap     local costmap ◀── 激光/点云
        └──────┬───────────┘
        定位: map→odom (AMCL) , odom→base_link (EKF/里程计)
        地图: map_server
```

要点：
1. 传感器 → 代价地图（障碍）；
2. 定位提供车在 `map` 中的位姿；
3. 全局规划器在 global costmap 上出路径；
4. 局部规划器在 local costmap 上沿路径出指令；
5. 失败时触发恢复行为，仍失败则上报目标失败。

---

## 2. TF 树

REP-105 约定的链：

```text
 map ──▶ odom ──▶ base_link ──▶ laser / imu_link / ...
```

| 坐标系 | 特性 | 谁发布 |
|---|---|---|
| `map` | 全局固定，可能被校正而跳变 | 树根 |
| `odom` | 连续，会漂 | `map→odom`：AMCL（定位）；一般不由里程计发布 |
| `base_link` | 车体 | `odom→base_link`：EKF 或里程计驱动（二选一，不可重复发布） |
| 传感器系 | 固定外参 | `robot_state_publisher` / 静态 TF |

**为什么分成两段**（承接[AMCL 章](../05_状态估计/05_粒子滤波与AMCL.md) 第 11 节）：全局校正产生的跳变只出现在 `map→odom`，`odom→base_link` 始终连续。因此：
- 局部规划与控制在 `odom` 系（或连续的局部代价地图）下工作 → 不受跳变影响；
- 全局规划在 `map` 系 → 与地图对齐。
变换复合：$T_{map\,base}=T_{map\,odom}\,T_{odom\,base}$。
手算：$T_{map\,odom}=(9.866,2.768,60°)$，$T_{odom\,base}=(2,1,30°)$ → $T_{map\,base}=(10,5,90°)$（见 AMCL 章）。

时间问题（[时间同步章](../05_状态估计/06_时间同步_坐标系一致性与数据有效性.md)）：导航节点用 TF 查询 `map→base_link`，TF 缓存、`transform_tolerance` 配置不当会出现外推异常，是"导航起不来"的最常见原因之一。

---

## 3. move_base

### 3.1 结构

`move_base` 节点是一个 **action server**（`move_base_msgs/MoveBaseAction`），也订阅 `/move_base_simple/goal`（RViz 点目标）。内部包含：全局规划器、局部规划器、两张代价地图、恢复行为列表。

### 3.2 状态机

```text
 收到目标 ─▶ PLANNING(全局规划) ──成功──▶ CONTROLLING(局部规划/发指令)
               │失败                           │失败(振荡/无有效指令)
               ▼                               ▼
          CLEARING(恢复行为) ◀────────────────┘
               │恢复后重试 / 全部失败 ──▶ ABORTED
 CONTROLLING 中到达目标 ─▶ SUCCEEDED
```

规划循环：全局规划以 `planner_frequency` 触发（0 表示仅在新目标或失败时规划）；控制循环以 `controller_frequency` 运行（如 10~20 Hz）。重要参数：`planner_patience`、`controller_patience`、`oscillation_timeout`、`oscillation_distance`、`max_planning_retries`（名称凭记忆，以文档为准）。

---

## 4. 规划器接口（插件）

move_base 通过 `nav_core` 抽象接口加载插件（pluginlib）：

```cpp
// 概念性签名（教学示例）
class BaseGlobalPlanner {
  virtual bool makePlan(const PoseStamped& start, const PoseStamped& goal,
                        std::vector<PoseStamped>& plan) = 0;
};
class BaseLocalPlanner {
  virtual bool setPlan(const std::vector<PoseStamped>& plan) = 0;
  virtual bool computeVelocityCommands(Twist& cmd_vel) = 0;
  virtual bool isGoalReached() = 0;
};
```

含义：全局规划器给 `start→goal` 的路径；局部规划器拿到路径，每个周期算 `cmd_vel`，并判断是否到达。这样 Hybrid A*、A*、TEB、DWA 都能"换插件不换系统"。
常见插件：`navfn`、`global_planner`（A*/Dijkstra）；`dwa_local_planner`、`teb_local_planner`。注意：**`cmd_vel` 是线速度 + 角速度**，车式底盘要在底盘层或规划器层转成转向角（见 [TEB 章](./03_代价地图_局部规划_TEB.md) 第 9 节）。
Hybrid A* 在 move_base 里需要自己的全局规划器插件（或使用其他实现），本项目的做法是把 Hybrid A* 包成全局规划器插件：`makePlan` 内部把起点、终点转成带航向的 $(x,y,\theta)$，在全局代价地图上搜索，再把平滑后的位姿序列填进 `plan`；车体外形与 $R_{min}$ 作为插件参数读入。

### costmap_2d 的位置

`costmap_2d` 被 move_base 内部实例化两次（全局、局部），参数分别配置（frame、更新频率、层、footprint、膨胀）。详见 [代价地图章](./03_代价地图_局部规划_TEB.md)。

---

## 5. Recovery 行为

规划或控制失败（找不到路、被困、振荡）时按顺序尝试：

| 序号 | 行为 | 作用 |
|---|---|---|
| 1 | conservative reset | 清除机器人一定半径外的障碍（假设是传感器残留） |
| 2 | rotate recovery | 原地旋转，刷新周围障碍信息（**车式底盘无法原地转，要禁用或换成倒车/摆动行为**） |
| 3 | aggressive reset | 清除更大范围代价 |
| 4 | 再次旋转 | 同上 |
| 5 | 全部失败 | ABORTED，上报 |

**车式底盘的特殊性**：原地旋转不可行，需自定义恢复（倒车、前后摆动、重规划）。
设计原则：先便宜（清地图）、后昂贵（动车）；限制重试次数，避免无限循环；失败要上报给任务层。

---

## 6. Nav2（ROS2）

### 6.1 架构

move_base 是一个"大单体"节点；Nav2 拆成多个**服务器**，由**行为树**编排：

| 组件 | 职责 |
|---|---|
| `bt_navigator` | 行为树导航器，接收 `NavigateToPose` 动作，执行行为树 |
| `planner_server` | 加载全局规划插件（NavFn、**Smac Planner**：2D A*、**Hybrid-A***、State Lattice 等），动作 `ComputePathToPose` |
| `controller_server` | 加载控制器插件（DWB、**MPPI**、**Regulated Pure Pursuit** 等；TEB 为社区移植），动作 `FollowPath`；内含局部代价地图 |
| `smoother_server` | 路径平滑 |
| `behavior_server` | 恢复行为：spin、backup、wait、drive-on-heading 等 |
| `waypoint_follower` | 多航点任务 |
| `map_server` / `amcl` | 地图与定位 |
| `lifecycle_manager` | 管理各节点生命周期 |

（组件与插件名称按记忆列出，以所用发行版文档为准。）

### 6.2 行为树

行为树（Behavior Tree）用树结构表达"先做什么、失败怎么办"。典型 `NavigateToPose`：

```text
 RecoveryNode(重试次数 N)
 ├─ PipelineSequence
 │    ├─ RateController(1 Hz) → ComputePathToPose   (规划)
 │    └─ FollowPath                                  (控制)
 └─ ReactiveFallback / Sequence → 清除代价地图, spin, wait, backup   (恢复)
```

节点类型：Sequence（依次，一个失败则整体失败）、Fallback（依次尝试直到一个成功）、Decorator（重试、限频）、Action/Condition（叶子，调用 ROS2 action / 检查条件）。相比 move_base 固定的状态机，行为树**可配置、可组合、可观察**，恢复策略直接改 XML，无需改代码。

### 6.3 生命周期节点

ROS2 的 lifecycle node 有状态：`unconfigured → inactive → active → finalized`（+ 错误状态）。Nav2 的 `lifecycle_manager` 按顺序 configure / activate 各服务器，保证"地图服务和定位起来之后规划才启动"，也便于出错后单独重启。

### 6.4 其他改进

ROS2 DDS 通信、动作接口替代 move_base 的 action；代价地图逐层组合；可对每种机器人换插件（差速、全向、阿克曼）；Smac Hybrid-A* 对车式底盘开箱可用。

---

## 7. 对比表

| 维度 | move_base (ROS1) | Nav2 (ROS2) |
|---|---|---|
| 结构 | 单节点 + 插件 | 多服务器 + 行为树 |
| 任务流程 | 固定状态机 | 行为树，可配置 |
| 恢复行为 | 固定列表 | 行为树中任意编排 |
| 生命周期 | 无 | lifecycle 节点 |
| 通信 | ROS1 TCPROS | ROS2 DDS |
| 车式底盘 | 需自写规划器 / 恢复 | Smac Hybrid-A*、MPPI/RPP 可用 |
| 全局规划 | navfn / global_planner | NavFn / Smac 系列 |
| 局部规划 | DWA / TEB | DWB / MPPI / RPP / TEB(社区) |

---

## 8. 在本项目里怎么用

CV 的 P5 链：EKF/AMCL 定位 → Hybrid-A* 全局规划 → TEB 局部优化 → MPC 跟踪。映射：
- 定位层：`odom→base_link`（EKF）+ `map→odom`（AMCL），见[里程计章](../05_状态估计/04_里程计与传感器融合.md)、[AMCL 章](../05_状态估计/05_粒子滤波与AMCL.md)；
- 全局规划：Hybrid A*（[本组第二章](./02_Hybrid_A星与车辆运动学约束.md)）；
- 局部规划：TEB 在 local costmap 上（[第三章](./03_代价地图_局部规划_TEB.md)）；
- 跟踪：MPC（[第四章](./04_轨迹跟踪_PurePursuit_Stanley_MPC.md)），把规划输出变为车的转向与速度。
本文不假设你用了 move_base 还是 Nav2，也不假设哪些模块是自写的。见 [P5 README](../../01_项目课程/P5_车式移动机器人自主导航/README.md)。

> **本项目的做法（架构层面的设计取值，非实测）**
> - 框架：整条链按 move_base 式的分层来组织数据流（全局规划器、局部规划器、双代价地图、恢复行为）；若换到 ROS2，则一一对应到 Nav2 的 planner_server、controller_server、behavior_server，思路不变（对应关系见第 16 节）。
> - 自写与配置的分工：定位融合（EKF/AMCL 的 TF 分工）、Hybrid A* 规划器插件、MPC 跟踪器是需要针对车式底盘专门实现或适配的部分；代价地图层、TEB 等是公开机制，我们的工作是配置与调参（footprint、膨胀、权重、限速）。
> - TF 树：`map → odom → base_link → laser/imu`，`odom→base_link` 由 EKF 发布，`map→odom` 由 AMCL 发布，传感器外参为静态变换。
> - 恢复行为：车式底盘不做原地旋转；顺序为清局部代价地图 → 倒车一小段（空间允许时）→ 清全局地图并重规划 → 上报失败。

## 9. 常见错误

| 现象 | 原因 |
|---|---|
| "Could not get robot pose" / TF 外推 | `map→base_link` 查不到：AMCL 未发布、时间戳 / 时钟问题 |
| 两个节点发布 `odom→base_link` | 驱动与 EKF 重复 |
| 车原地旋转失败 / 卡住 | 车式底盘的 rotate recovery 不可行 |
| 目标点在障碍内 → 规划失败 | 目标需吸附或容差 |
| 全局路径变化引发车突然转向 | 全局重规划频率太高 |
| Nav2 节点没起来 | lifecycle 未 activate；依赖顺序 |
| 局部规划器和控制频率不匹配 | 控制周期内算不完 |

## 10. 面试问答

**Q1. Describe the TF tree for navigation.**
> A: It is map to odom to base_link. The odom to base_link transform comes from the EKF or wheel odometry and is continuous but drifts. The map to odom transform comes from AMCL and absorbs the global corrections, so jumps appear only there. Planning on the map frame and control in the odom frame keeps the vehicle command smooth.

**Q2. How does move_base work?**
> A: It is an action server with a global planner, a local planner, two costmaps and recovery behaviours. When a goal arrives, it plans a global path, then the local planner computes velocity commands each cycle. If planning or control fails, it clears the costmaps or rotates in place, and aborts if everything fails. The planners are plugins with a simple interface, makePlan and computeVelocityCommands.

**Q3. What does Nav2 change?**
> A: It splits the monolith into servers for planning, control, behaviours and smoothing, orchestrated by a behaviour tree, and uses lifecycle nodes for managed startup. Recovery strategies become configurable trees, and there are plugins for car-like robots such as Smac Hybrid-A* and MPPI.

**Q4. Why is rotate recovery a problem for your vehicle?** （车式）
> A: A car-like vehicle cannot rotate in place because of the minimum turning radius, so I replace it with reversing a short distance, or a back-and-forth manoeuvre, and then replan. The order is cheap first: clear the local costmap, back up if there is room, clear the global costmap and replan, and only then report failure.

## 11. move_base 状态机逐周期拆解

### 11.1 它要解决什么问题

前面第 3 节只给了状态图。面试和调试时更常见的问题是：**某一秒里到底是谁在运行、频率是多少、失败了往哪走**。这一节把状态机拆到"周期"粒度。

### 11.2 符号与单位

| 符号 | 含义 | 单位 |
|---|---|---|
| $f_p$ | `planner_frequency`，全局规划触发频率；0 表示只在新目标或失败时规划 | Hz |
| $f_c$ | `controller_frequency`，控制循环频率 | Hz |
| $T_{pp}$ | `planner_patience`，规划允许失败的最长时间 | s |
| $T_{cp}$ | `controller_patience`，控制找不到有效指令的最长时间 | s |
| $T_{osc}$、$d_{osc}$ | `oscillation_timeout`、`oscillation_distance`：给定时间内必须离开原位置至少这么远 | s、m |

（参数名按公开文档记忆列出，**未核对**具体发行版，以所用版本为准。）

### 11.3 一次导航任务的时间线（逐步）

```text
t=0     收到目标      → 状态 PLANNING，调用 makePlan(start, goal)
t=0.2   规划成功      → 路径交给局部规划器 setPlan，进入 CONTROLLING
t=0.2~  每 1/f_c 秒    → computeVelocityCommands(cmd) ，发布到底盘
        每 1/f_p 秒    → （若 f_p>0）后台重新规划，更新局部规划器的路径
        每个周期       → 检查 isGoalReached，检查振荡
到达    isGoalReached  → SUCCEEDED
失败    规划/控制超时 → CLEARING，依次执行恢复行为，再回 PLANNING
```

### 11.4 完整手算：一段 12 m 路径要跑多少次规划、多少次控制

教学示例数值（非项目实测）：路径长 12 m，平均速度 0.6 m/s，$f_c=10$ Hz，$f_p=1$ Hz。

1. 行驶时间 $t=12/0.6=20$ s。
2. 控制周期数 $N_c=t\cdot f_c=20\times10=200$ 次 `computeVelocityCommands`。
3. 规划次数 $N_p=t\cdot f_p=20$ 次（另加最初那一次）。若 $f_p=0$，整段只规划 1 次，局部规划器全程贴着同一条全局路径。
4. 每个控制周期预算 $1/f_c=0.1$ s。局部规划器若一次要 0.12 s，就会"错过周期"，指令间隔不均匀；这就是第 9 节里"局部规划器和控制频率不匹配"的来源。
5. 20 次重规划意味着路径每秒可能略有变化。若 Hybrid A* 对相近起点给出形状差异很大的路径，车就会"突然转向"。**我们的做法是**把 $f_p$ 取得很低（只在新目标、路径被障碍堵住、或定位大幅修正时规划），让局部规划器去处理动态障碍。

### 11.5 振荡检测手算

设 $T_{osc}=10$ s，$d_{osc}=0.5$ m。在 $t=0$ 记录位置 $(0,0)$。到 $t=10$ s 时车在 $(0.2,0.1)$（在窄道来回摆）：

$$d=\sqrt{0.2^2+0.1^2}=0.2236\ \text{m}<0.5\ \text{m}$$

没有离开 0.5 m 圆，判定为振荡，进入 CLEARING。反之若 $t=6$ s 时已到 $(0.5,0.3)$，距离 $0.583>0.5$，计时器复位。

### 11.6 本项目里怎么用

TEB 在窄道里偶发"来回摆"，move_base 层面的振荡检测是最后一道保险；但更好的做法是先在 TEB 层面抑制（同伦类滞回、限速），见[TEB 章](./03_代价地图_局部规划_TEB.md)。

### 11.7 常见错误

- 把 $f_p$ 设得过高，车一直换路径；
- 控制周期算不完；
- 以为 PLANNING 失败后马上 ABORTED：实际是先 CLEARING 再重试，几轮都失败才 ABORTED。

---

## 12. 规划器接口与 costmap_2d 分层

### 12.1 要解决的问题

障碍信息来自多种来源（静态地图、激光、深度相机），既要融合，又要给不同规划器共用。`costmap_2d` 用**分层**的办法：每层只管一件事，按顺序叠加到同一张栅格上。

### 12.2 符号与代价值（常用约定）

栅格代价为 0–255 的整数：

| 值 | 含义 |
|---|---|
| 0 | 自由空间 |
| 1–252 | 越大越靠近障碍（膨胀产生） |
| 253 | 内切（inscribed）：机器人中心在此处，footprint 必碰障碍 |
| 254 | 致命（lethal）：障碍本身 |
| 255 | 未知 |

### 12.3 常见的层（按顺序）

1. **static layer**：来自 `map_server` 的静态地图，作底。
2. **obstacle layer**：订阅激光，把命中点标记为致命，并用射线清除（raytrace）穿过的格子，处理动态障碍的"消失"。
3. **inflation layer**：把致命格子向外膨胀出一圈递减的代价，最后一层。

顺序有意义：膨胀必须在障碍都标完之后做，所以总是放最后。

### 12.4 膨胀代价公式与手算

设内切半径 $r_{ins}=0.30$ m，衰减系数 $\alpha=3.0$（`cost_scaling_factor`），则距障碍距离 $d\ge r_{ins}$ 的格子代价（取整）为

$$c(d)=253\cdot e^{-\alpha\,(d-r_{ins})}$$

逐点计算：

| $d$ (m) | $d-r_{ins}$ | $e^{-3(d-r_{ins})}$ | $c$ |
|---|---|---|---|
| 0.30 | 0 | 1 | 253 |
| 0.40 | 0.10 | 0.7408 | 187 |
| 0.50 | 0.20 | 0.5488 | 138 |
| 0.60 | 0.30 | 0.4066 | 102 |
| 0.70 | 0.40 | 0.3012 | 76 |
| 0.80 | 0.50 | 0.2231 | 56 |

再看 $\alpha$ 的影响：同在 $d=0.50$ m 处（$d-r_{ins}=0.2$），$\alpha=1$ 得 207，$\alpha=3$ 得 138，$\alpha=5$ 得 93，$\alpha=10$ 得 34。**$\alpha$ 越大衰减越快**，路径允许更贴近障碍；$\alpha$ 越小，路径被推得离障碍更远。

### 12.5 两张图：全局与局部

`costmap_2d` 被实例化两次：

| | global costmap | local costmap |
|---|---|---|
| 坐标系 | `map` | `odom`（滚动窗口） |
| 尺寸 | 整张地图 | 车周围几米 |
| 更新频率 | 较低 | 较高 |
| 用途 | Hybrid A* 全局规划 | TEB 局部优化 |
| 层 | static + obstacle + inflation | 常去掉 static，仅 obstacle + inflation |

### 12.6 规划器接口回顾与车式注意点

`makePlan` 只管几何路径；`computeVelocityCommands` 输出 `Twist`。车式底盘需要转向角，所以局部规划器或底盘层要把 $(v,\omega)$ 转成 $\delta=\arctan(L\omega/v)$。例如 $L=0.5$ m、$v=0.6$ m/s、$\omega=0.4$ rad/s：

$$\delta=\arctan\frac{0.5\times0.4}{0.6}=\arctan0.3333=18.43^\circ$$

注意 $v\to0$ 时 $\delta$ 发散：低速时要限幅或直接输出曲率，这也是 TEB 提供"用转向角代替角速度"选项的原因。

### 12.7 常见错误

- 膨胀半径小于车体外接半径，路径贴墙；
- obstacle layer 没做射线清除，动态障碍走了仍留"鬼影"；
- 局部图太小，TEB 看不到前方障碍。

---

## 13. 恢复行为与车式底盘适配

### 13.1 问题

规划失败的原因通常是**地图里的信息过时或过于保守**（传感器残影），而不是世界真的无路。恢复行为的思路是：先更新信息，再动车。

### 13.2 默认序列与车式改造

| 步骤 | 默认行为 | 车式底盘改造 |
|---|---|---|
| 1 | 清除较远处障碍（conservative reset） | 保留，代价最低 |
| 2 | 原地旋转 | 去掉；车无法原地转 |
| 3 | 更激进地清除代价地图 | 保留 |
| 4 | 再旋转 | 换成"倒车一小段" |
| 5 | ABORTED | 上报任务层 |

### 13.3 倒车恢复的设计与手算

倒车行为的核心是：**后方是否有足够自由空间**。教学例：车体后沿到后方障碍的距离 $d_{back}=0.9$ m，要求倒车 $s=0.5$ m，后向安全余量 $m=0.2$ m。判据：

$$d_{back}-s\ge m\ \Rightarrow\ 0.9-0.5=0.4\ge0.2\ \text{成立，可以倒}$$

若 $d_{back}=0.6$，则 $0.6-0.5=0.1<0.2$，不执行，直接进入下一步（清地图重规划）。倒车速度取低值（例如 0.2 m/s），耗时 $0.5/0.2=2.5$ s，在此期间把局部代价地图在新位置重新刷新一遍，再重规划。

### 13.4 本项目里怎么用

我们的做法：清局部图 → 倒车（有空间才倒）→ 清全局图并重规划 → 上报；并给整个序列设置最大次数，避免死循环。

### 13.5 常见错误

- 恢复行为里不检查后方空间，倒车撞墙；
- 无限重试；
- 目标本身在障碍里，恢复也无济于事，应上报并让任务层换目标。

---

## 14. TF 树深入：复合、跳变与外推

### 14.1 问题

规划在 `map`，控制要在车体系；传感器数据带时间戳。导航里 80% 的"起不来"问题都在 TF。

### 14.2 复合手算（含旋转）

教学示例：$T_{map\,odom}=(2,1,90^\circ)$，$T_{odom\,base}=(3,0,0^\circ)$。

车在 `map` 中的位置 = 平移 + 旋转后的相对位置：

$$\begin{bmatrix}x\\y\end{bmatrix}=\begin{bmatrix}2\\1\end{bmatrix}+\begin{bmatrix}\cos90^\circ&-\sin90^\circ\\\sin90^\circ&\cos90^\circ\end{bmatrix}\begin{bmatrix}3\\0\end{bmatrix}=\begin{bmatrix}2\\1\end{bmatrix}+\begin{bmatrix}0\\3\end{bmatrix}=\begin{bmatrix}2\\4\end{bmatrix}$$

（注意：旋转后 $(3,0)\to(0,3)$，所以是 $(2,4)$。）航向 $90^\circ+0^\circ=90^\circ$。

激光相对车体安装在 $(0.3,0)$：它在 `map` 中为 $(2,4)+R(90^\circ)(0.3,0)=(2,4)+(0,0.3)=(2,4.3)$。

### 14.3 AMCL 修正造成的跳变

AMCL 把 $T_{map\,odom}$ 修正为 $(2.1,1.1,90^\circ)$，里程计链 $T_{odom\,base}$ 不变。则车在 `map` 中为 $(2.1,1.1)+(0,3)=(2.1,4.1)$。跳变量：

$$\Delta=(2.1-2.0,\ 4.1-4.0)=(0.1,0.1),\quad \|\Delta\|=0.1414\ \text{m}$$

关键观察：**`odom→base_link` 完全没有跳**，所以在 `odom` 系里工作的局部规划器与控制器感觉不到这 0.14 m；只有全局路径相对车的位置变了一点，下一次全局重规划会自然吸收它。这就是 TF 分两段的价值。

### 14.4 时间外推手算

激光点云时间戳 $t_s=10.00$ s，TF 缓存中最新的 `odom→base_link` 时间为 9.95 s。查询 $t_s$ 时刻的变换，需要向前外推 0.05 s。若 `transform_tolerance`$=0.1$ s，则允许；若设成 0，则抛出外推异常，代价地图更新失败。这解释了第 9 节里的 "TF 外推" 现象。检查方法：看 TF 发布频率是否过低（例如 EKF 只有 5 Hz，间隔 0.2 s 就会超过 0.1 s 容限）。

### 14.5 本项目里怎么用

只允许 EKF 发布 `odom→base_link`（驱动里要关掉自己的 TF 发布），AMCL 发布 `map→odom`，两者配合见 [P5 的 EKF 与 AMCL 协同章](../../01_项目课程/P5_车式移动机器人自主导航/06_EKF与AMCL协同_TF分工与连续状态.md)。

### 14.6 常见错误

- 两个节点发布同一对 TF，变换来回抖动；
- 仿真/回放时没有统一使用仿真时间；
- 静态变换 (base→laser) 的偏移方向写反。

---

## 15. Nav2 深入：行为树与生命周期

### 15.1 问题

move_base 的状态机写死在 C++ 里，改恢复顺序要改代码。Nav2 把"流程"外置成行为树，把"节点启动"交给生命周期管理。

### 15.2 行为树的返回值与节点语义

每个节点每次被"tick"返回 SUCCESS、FAILURE 或 RUNNING。

| 节点 | 语义 |
|---|---|
| Sequence | 依次执行子节点；某个 FAILURE 则自己 FAILURE；全 SUCCESS 才 SUCCESS |
| Fallback | 依次尝试；某个 SUCCESS 则自己 SUCCESS；全 FAILURE 才 FAILURE |
| RateController | 装饰器：限制子节点被 tick 的频率（如 1 Hz 重规划） |
| RecoveryNode | 子树失败时执行恢复子树，然后重试，最多 N 次 |
| Action 叶子 | 调用 ROS2 action（`ComputePathToPose`、`FollowPath`、`BackUp`、`Wait` 等） |

### 15.3 一次 tick 的手算

设重试上限 6（即最多 7 次尝试）。假设每次"规划 + 跟随"成功率 $p=0.7$（教学示例）：

- 第 1 次尝试失败 → 执行恢复子树（清图/后退）→ 第 2 次尝试；
- 全部 7 次都失败的概率 $0.3^7=2.19\times10^{-4}$，整体成功率 $1-2.19\times10^{-4}=0.99978$。

当然实际中各次尝试不独立（同一个死路会反复失败），所以这是**上界估计**，说明"重试 + 恢复"能显著提高成功率，但无法解决结构性问题（目标在障碍里）。

行为树 tick 的时间线：RUNNING 状态下，控制器节点（`FollowPath`）每个周期返回 RUNNING；RateController 以 1 Hz 让 `ComputePathToPose` 重规划；一旦 `FollowPath` 返回 FAILURE，整棵 PipelineSequence 返回 FAILURE，由 RecoveryNode 接管。

### 15.4 生命周期节点

| 状态 | 含义 | 转换 |
|---|---|---|
| unconfigured | 刚创建 | configure → inactive |
| inactive | 已读参数、建好资源，但不工作 | activate → active |
| active | 正常工作 | deactivate → inactive |
| finalized | 终止 | shutdown |

`lifecycle_manager` 的启动顺序（典型）：地图服务与定位先 configure + activate，然后规划/控制/行为服务器，最后 `bt_navigator`。若某个节点 configure 失败，管理器可以整体回退，而不是留下一个半启动系统。

### 15.5 与插件的关系

Nav2 的规划/控制/平滑/行为都是服务器加载的插件，各自有自己的 action 接口；代价地图则内嵌在 planner_server 与 controller_server 里。替换算法 = 改参数文件里的插件名。具体插件和参数名以所用发行版文档为准（**未核对**）。

### 15.6 常见错误

- 地图服务没 activate，规划服务器拿不到地图；
- 行为树 XML 里引用了没有加载的插件；
- 把 RecoveryNode 的重试数设得太大，机器人长时间原地打转。

---

## 16. move_base 与 Nav2：差异与本项目对应

### 16.1 逐项差异

| 关注点 | move_base | Nav2 | 对车式底盘的意义 |
|---|---|---|---|
| 流程 | 硬编码状态机 | 行为树 XML | 恢复策略可改为倒车而不改代码 |
| 规划/控制关系 | 同一进程，同步调用 | 不同服务器，action 通信 | 规划耗时不阻塞控制 |
| 全局规划插件 | 接口 `makePlan` | 接口 `createPlan`（以文档为准，未核对） | Hybrid A* 写成插件即可 |
| 控制 | 局部规划器直接出 `cmd_vel` | 控制器插件 `computeVelocityCommands` | MPC 可作为控制器插件 |
| 启动 | roslaunch，无依赖管理 | lifecycle 管理 | 启动顺序明确 |
| 恢复 | 固定序列 | 行为树中编排 | 车式适配更直观 |

### 16.2 本项目的映射

| 本项目环节 | move_base 式 | Nav2 式 |
|---|---|---|
| Hybrid A* | 全局规划器插件 | planner_server 插件 |
| TEB | 局部规划器 | controller 插件（或作为局部轨迹优化模块） |
| MPC | 在局部规划器之后跟踪 | controller 层的跟踪 |
| 代价地图 | 两个 costmap_2d | 两个代价地图（各服务器内） |
| 恢复 | recovery 列表 | behavior_server + 行为树 |

### 16.3 面试问答（补充）

**Q5. Why do you split the TF into map→odom and odom→base_link?**
> A: Because the global corrections from AMCL can jump, but the controller needs a continuous pose. So the jump is absorbed in map to odom, and the local planner works in the odom frame. In a small example, a 0.14 m correction in map to odom changes nothing in odom to base_link.

**Q6. How does the inflation layer affect the path?**
> A: It adds a decaying cost around obstacles, so the planner prefers staying away from them. The decay is exponential with a scaling factor. A larger factor lets the path go closer to obstacles; a smaller one pushes it away. I match the inscribed radius to the real footprint first, then tune the scaling.

**Q7. What happens when planning fails in Nav2?**
> A: The behaviour tree catches the failure in a recovery node, runs the recovery subtree — for example clear costmaps or back up — and retries up to a configured limit. For a car-like vehicle I avoid spinning in place and use backing up and replanning instead.

**Q8. Why do you use lifecycle nodes?**
> A: They give a controlled start-up order and let me configure, activate, deactivate or restart one node without bringing the whole stack down.

### 16.4 一个端到端的故障排查例子

现象：车到达目标前几米时突然停住，日志提示"控制器找不到有效指令"，随后进入恢复行为。

排查按"由数据流上游到下游"的顺序，每一步对应前面某一节：

1. **TF 与时间（第 14 节）**：检查 `map→base_link` 是否能在当前时间查到，TF 发布频率是否高于容限的倒数。若 EKF 只有 5 Hz 而容限 0.1 s，间歇外推失败会让代价地图停更，局部规划器以为前方有残留障碍。
2. **代价地图（第 12 节）**：在可视化里看局部代价地图，目标附近是否有未被射线清除的"鬼影"障碍；膨胀是否把窄口堵死。设内切半径 0.30 m、通道宽 0.70 m，则两侧膨胀至少占去 $2\times0.30=0.60$ m，剩余 $0.10$ m，意味着车中心只能走中线，任何定位误差都会让代价升到 253 以上，规划器判为不可通行。
3. **规划器（第 4 节）**：全局路径是否经过了这个窄口？Hybrid A* 的 footprint 检测与局部图的膨胀口径是否一致；口径不一致就会出现"全局能走、局部不让走"。
4. **局部规划器与限速（TEB 章）**：窄口里限速、转弯半径约束是否让可行解为空。
5. **恢复行为（第 13 节）**：恢复动作是否真的改变了信息（清图），还是只在原地重复。

结论的写法：先确认是"信息问题"（TF、鬼影）还是"约束问题"（口径不一致、通道本身过窄）。前者清图或修时间即可；后者必须改参数或换目标。

### 16.5 本章与其他章的衔接

- 与[图搜索](./01_图搜索_Dijkstra_A星.md)：全局规划器的搜索算法，对应本章的 `makePlan`；
- 与[Hybrid A* 章](./02_Hybrid_A星与车辆运动学约束.md)：车式全局规划器的内部机理；
- 与[TEB 章](./03_代价地图_局部规划_TEB.md)：局部规划器的内部机理与代价地图细节；
- 与[轨迹跟踪章](./04_轨迹跟踪_PurePursuit_Stanley_MPC.md)：局部规划器之后的跟踪控制器。

读完本章，应当能够不看笔记地讲出：一个目标位姿从进入系统到变成转向指令，依次经过哪些节点、哪些坐标系、哪些频率，以及哪一步失败时由谁负责恢复。

### 16.6 补充自测

1. 12 m 路径、0.6 m/s、10 Hz 控制，共多少次控制调用？（200。）
2. $r_{ins}=0.3$、$\alpha=3$，$d=0.5$ m 处代价？（138。）
3. $T_{map\,odom}=(2,1,90°)$，$T_{odom\,base}=(3,0,0°)$，车在 map 的位置？（$(2,4)$。）
4. 激光时间戳 10.00，最新 TF 9.95，容限 0.04，会怎样？（外推 0.05>0.04，失败。）
5. Fallback 与 RecoveryNode 的区别？（Fallback 是同层替代方案；RecoveryNode 是"失败后做恢复再重试同一子树"。）

---

## 17. 速记、自测、链接

**速记**：链 `map→odom→base_link`；`map→odom` 吸收跳变；move_base = action server + 全局/局部插件 + 双 costmap + recovery；接口 `makePlan` / `computeVelocityCommands`；恢复先便宜后昂贵；车式禁原地旋转；Nav2 = 多服务器 + 行为树 + lifecycle；Smac 含 Hybrid-A*。

**自测**：
1. 谁发布 `map→odom`？为什么不是里程计？（AMCL；里程计只知道相对运动，无地图信息，且要保持连续。）
2. $T_{map\,odom}=(4,0,0°)$，$T_{odom\,base}=(1,0,0°)$，$T_{map\,base}$？（(5,0,0°)。）
3. move_base 局部规划器必须实现哪些方法？（setPlan、computeVelocityCommands、isGoalReached。）
4. 行为树 Sequence 与 Fallback 的区别？（Sequence 全成功才成功；Fallback 一个成功即成功。）
5. lifecycle 节点解决什么问题？（受控的启动顺序、可配置/激活/停用/重启。）

**链接**：上一章 [轨迹跟踪](./04_轨迹跟踪_PurePursuit_Stanley_MPC.md)；[图搜索](./01_图搜索_Dijkstra_A星.md)；[ROS1/ROS2 核心概念](../09_机器人软件/04_ROS1_ROS2核心概念.md)；[P5](../../01_项目课程/P5_车式移动机器人自主导航/README.md)；[路径规划与导航八股](../../03_八股知识/03_机器人学/04_路径规划与导航八股.md)。
