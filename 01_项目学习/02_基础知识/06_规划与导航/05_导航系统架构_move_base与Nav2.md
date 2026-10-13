# 导航系统架构：move_base 与 Nav2

> **一句话定位**：前面几章是零件（定位、全局规划、局部规划、跟踪），本章讲它们怎样装成一个能"给个目标点就自己走过去"的系统：ROS1 的 move_base，以及 ROS2 的 Nav2。重点是接口、数据流、TF 树和失败恢复。

## 本章学完你能……

- 画出 move_base 的数据流和状态机（PLANNING / CONTROLLING / CLEARING），说清全局 / 局部规划器插件接口。
- 理解 `map→odom→base_link` 的 TF 树（REP-105）：谁发布哪一段、为什么这样分。
- 说明 costmap_2d 在系统里的位置，以及 recovery 行为的顺序与作用。
- 对比 Nav2 与 move_base：行为树、生命周期节点、服务器化插件，以及 Smac Hybrid-A*、控制器插件。
- 把 P5 的 EKF/AMCL + Hybrid A* + TEB + MPC 映射到该架构，并诚实说明哪些是你实现的、哪些是配置。

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
11. 速记、自测、链接

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
Hybrid A* 在 move_base 里需要自己的全局规划器插件（或使用其他实现），P5 具体如何接入【待你补全】。

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

> **【待你补全】** 你用的是 ROS1 还是 ROS2、move_base 还是 Nav2，哪些插件是自己写或改的，TF 树实际是什么样，恢复行为如何适配车式底盘。面试时区分"我实现的"与"我配置的"。

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
> A: A car-like vehicle cannot rotate in place because of the minimum turning radius, so I replace it with reversing or a back-and-forth manoeuvre, or just replan. 【请按你实际做法回答】

## 11. 速记、自测、链接

**速记**：链 `map→odom→base_link`；`map→odom` 吸收跳变；move_base = action server + 全局/局部插件 + 双 costmap + recovery；接口 `makePlan` / `computeVelocityCommands`；恢复先便宜后昂贵；车式禁原地旋转；Nav2 = 多服务器 + 行为树 + lifecycle；Smac 含 Hybrid-A*。

**自测**：
1. 谁发布 `map→odom`？为什么不是里程计？（AMCL；里程计只知道相对运动，无地图信息，且要保持连续。）
2. $T_{map\,odom}=(4,0,0°)$，$T_{odom\,base}=(1,0,0°)$，$T_{map\,base}$？（(5,0,0°)。）
3. move_base 局部规划器必须实现哪些方法？（setPlan、computeVelocityCommands、isGoalReached。）
4. 行为树 Sequence 与 Fallback 的区别？（Sequence 全成功才成功；Fallback 一个成功即成功。）
5. lifecycle 节点解决什么问题？（受控的启动顺序、可配置/激活/停用/重启。）

**链接**：上一章 [轨迹跟踪](./04_轨迹跟踪_PurePursuit_Stanley_MPC.md)；[图搜索](./01_图搜索_Dijkstra_A星.md)；[ROS1/ROS2 核心概念](../09_机器人软件/04_ROS1_ROS2核心概念.md)；[P5](../../01_项目课程/P5_车式移动机器人自主导航/README.md)；[路径规划与导航八股](../../03_八股知识/03_机器人学/04_路径规划与导航八股.md)。
