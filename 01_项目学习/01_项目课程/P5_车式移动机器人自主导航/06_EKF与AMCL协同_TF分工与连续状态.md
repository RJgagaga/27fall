# 06 EKF 与 AMCL 协同：TF 分工、位姿跳变与连续车辆状态

> **一句话定位**：EKF 负责"平滑但会漂"，AMCL 负责"不漂但会跳"。把两者按 `map → odom → base_link` 分工，再在下游用限速、门限、坐标系选择把"跳"挡住，才能得到 CV 里说的 **continuous vehicle state**。

**本章学完你能：**

- 说出 `odom→base_link` 与 `map→odom` 分别由谁发布、各自的数学性质（连续/离散、漂/不漂）；
- 手算 $T_{map\,odom}=T_{map\,base}T_{odom\,base}^{-1}$，并验证后续位姿一致；
- 列出位姿跳变的 8 类成因，给每类一个判别方法；
- 设计"限速平滑 + 门限 + 确认"的修正策略，并手算平滑所需步数；
- 解释控制层为什么放在 `odom` 系，以及全局路径是怎样被"搬进" `odom` 系的；
- 定义一个"连续车辆状态"接口，说明各字段的约定；
- 给出一套从现象到根因的跳变排查流程。

**前置知识：**

- [04 EKF 融合定位](./04_EKF融合定位.md)、[05 AMCL](./05_AMCL粒子滤波定位.md)
- 齐次变换：[../../02_基础知识/03_运动学与坐标变换/01_坐标系_旋转矩阵_齐次变换.md](../../02_基础知识/03_运动学与坐标变换/01_坐标系_旋转矩阵_齐次变换.md)
- 时间同步与坐标系一致性：[../../02_基础知识/06_状态估计/06_时间同步_坐标系一致性与数据有效性.md](../../02_基础知识/06_状态估计/06_时间同步_坐标系一致性与数据有效性.md)

**目录**

1. 为什么需要两个定位器
2. 分工：连续 vs 不漂
3. map→odom 的数学与手算
4. 位姿跳变的成因（8 类）
5. 平滑与门限策略
6. 下游如何"隔离"跳变
7. 连续车辆状态接口
8. 时间对齐与延迟
9. 倒车与方向对协同的影响
10. 跳变排查流程
11. 在 ROS2 里怎么落地：节点、话题、QoS 与 TF 发布权
12. 把平滑做成一个可验证的节点：逐周期手算
13. 日志判读示例：三份典型 map→odom 序列
14. 重定位、丢失与安全降级
15. 面试问答 / 要点速记 / 自测题 / 链接

---

## 1. 为什么需要两个定位器

| 需求 | 只用 EKF（里程+IMU） | 只用 AMCL（激光+地图） |
|---|---|---|
| 平滑、连续 | 满足 | 不满足：5–10 Hz，修正成块 |
| 高频（控制需要 30–50 Hz） | 满足 | 不满足 |
| 不漂移（长期位置准确） | 不满足：误差积累 | 满足 |
| 抗环境退化（走廊、玻璃、人群） | 满足：不依赖外界 | 不满足 |
| 抗打滑、模型误差 | 不满足 | 满足 |

没有一个定位器同时满足所有需求。所以采用"**互补**"设计：高频连续的负责瞬时，低频全局的负责长期。这在导航里是通用做法，REP-105 把它固化成 TF 约定。

---

## 2. 分工：连续 vs 不漂

```text
时间 ──────────────────────────────────────────────►

odom→base_link  (EKF)    ╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱  光滑，但会慢慢偏离真值
map→odom        (AMCL)   ───┐   ┌──┐    ┌───────┐        阶梯，每次修正一个台阶
                            └───┘  └────┘       └──
map→base_link   (合成)   ╱╱╱╱┐╱╱╱╱╱╱┐╱╱╱╱╱╱╱╱╱┐╱╱╱╱      不漂，但带台阶
                         (斜坡 + 台阶)
```

- `odom→base_link`：**连续**（速度有限，位姿对时间可微），**会漂**。用于局部规划、控制、局部代价地图。
- `map→odom`：**分段常数**（阶梯），**抵消漂移**。每次 AMCL 更新才改变一次，更新之间保持不变，因此合成的 `map→base_link` 在两次更新之间依然光滑。
- 合成：$T_{map\,base}(t)=T_{map\,odom}(t)\,T_{odom\,base}(t)$。

> **面试要点**：用一句话——"I keep the smooth part and the corrected part in separate transforms, so consumers that need continuity use odom, and consumers that need global consistency use map."

---

## 3. map→odom 的数学与手算

### 3.1 SE(2) 组合

位姿 $(x,y,\theta)$ 的组合 $A\cdot B$（先 $B$ 在 $A$ 系下，再 $A$）：
$$(x_A,y_A,\theta_A)\cdot(x_B,y_B,\theta_B)=\big(x_A+x_B\cos\theta_A-y_B\sin\theta_A,\ \ y_A+x_B\sin\theta_A+y_B\cos\theta_A,\ \ \theta_A+\theta_B\big).$$
逆：$(x,y,\theta)^{-1}=(-x\cos\theta-y\sin\theta,\ \ x\sin\theta-y\cos\theta,\ \ -\theta)$。

### 3.2 完整手算（教学数值）

- EKF（里程系）说：车在 odom 系的位姿 $T_{odom\,base}=(10,\ 0,\ 0)$；
- AMCL 说：车在 map 系的位姿 $T_{map\,base}=(9.7,\ 0.1,\ 0.02\ \mathrm{rad})$。

要发布 $T_{map\,odom}$：
$$T_{odom\,base}^{-1}=(-10,\ 0,\ 0)\ (\text{因 }\theta=0).$$
$$T_{map\,odom}=T_{map\,base}\cdot T_{odom\,base}^{-1}:\quad x=9.7+(-10)\cos0.02-0=9.7-9.998=-0.298,$$
$$y=0.1+(-10)\sin0.02+0=0.1-0.19999=-0.0999,\quad \theta=0.02+0=0.02.$$
所以 $T_{map\,odom}=(-0.298,\ -0.0999,\ 0.02)$。

**验证**：车继续前进，EKF 报 $T_{odom\,base}=(10.5,\ 0,\ 0)$，则
$$T_{map\,base}=T_{map\,odom}\cdot(10.5,0,0)=(-0.298+10.5\cos0.02,\ -0.0999+10.5\sin0.02,\ 0.02)=(10.200,\ 0.110,\ 0.02).$$
车向前 0.5 m，map 里位置变化 $(10.200-9.7,\ 0.110-0.1)=(0.500,\ 0.010)$——沿着航向 $0.02$ rad 的方向走了 0.5 m ✓（$0.5\sin0.02=0.01$）。

### 3.3 跳变有多大

在 AMCL 修正前，假设 $T_{map\,odom}$ 是恒等变换，则车在 map 中是 $(10,0,0)$；修正后是 $(9.7,\ 0.1,\ 0.02)$。一次台阶：
$$\Delta=\sqrt{0.3^2+0.1^2}=0.316\ \mathrm m,\quad \Delta\theta=0.02\ \mathrm{rad}=1.15^\circ.$$

车速 $1\ \mathrm{m/s}$，控制周期 50 ms 时，这个台阶相当于单个周期内 $0.316\ \mathrm m$ 的位置误差突变。若控制器直接用 map 系位姿，用 Stanley 公式估算：横向误差突变 $e\approx0.1$–$0.3$ m，$\delta\approx\arctan(ke/v)=\arctan(0.3)=16.7^\circ$（教学估算，$k=1,v=1$）的转向瞬态——车会明显"抽一下"。

---

## 4. 位姿跳变的成因（8 类）

| # | 成因 | 机理 | 判别方法 |
|---|---|---|---|
| 1 | **里程计漂移累积后被一次性纠正** | AMCL 修正落在更新点上 | 修正量与自上次修正以来的行驶距离成比例；跳变发生在 AMCL 更新点 |
| 2 | **AMCL 门限造成"成块"修正** | `update_min_d/a` 大，更新稀 | 减小门限后台阶变小变密 |
| 3 | **粒子簇切换** | 多峰，主簇换了 | 看 `/particle_cloud`（`nav2_amcl` 发布的粒子云话题）是否出现两团 |
| 4 | **初始位姿/重定位** | 手动 re-initial pose，或 recovery 注入随机粒子 | 跳变与操作/日志时间一致 |
| 5 | **时间戳错位** | 激光/里程/IMU 时间不一致，匹配拉偏 | 跳变量与速度、角速度成正比，转弯时更明显 |
| 6 | **坐标系/静态变换错** | `laser_link` 安装位姿错，匹配系统性偏 | 位姿总偏一个固定量；直行 vs 转弯偏差不同 |
| 7 | **方向（倒车）符号错** | 里程预测方向与激光观测矛盾 | 倒车时跳变，前进时正常 |
| 8 | **TF 重复发布/树错误** | 两个节点发布同一变换；或 TF 缓冲外推 | `ros2 run tf2_tools view_frames` / `rqt_tf_tree` 父节点冲突；TF 在两个值间闪烁 |

其中第 5、6、7 类正是 CV 里 "inconsistent timing, frames and motion direction" 的三个维度。处理它们的优先顺序：**先排除数据本身的不一致（5/6/7/8），再谈 AMCL 参数（1/2/3/4）**。数据不一致的情况下调 AMCL 参数是掩盖问题。

---

## 5. 平滑与门限策略

下面讲"典型做法"，我们的做法是 5.1 的限速平滑加 5.2 的门限确认两者叠加，不使用 5.3 的第二个 EKF。

### 5.1 限速平滑（slew limit / 低通）

把 AMCL 的目标修正 $T_{target}$ 与当前已发布的 $T_{pub}$ 之差拆成每周期最大步长：

```text
delta = T_target - T_pub            (平移向量 + 角度差, 角度差先归一化)
if |delta_xy| > d_max : delta_xy *= d_max / |delta_xy|
if |delta_th| > a_max : delta_th  = sign * a_max
T_pub += delta
```

> **示例手算**：修正量 $(-0.298,\ -0.0999)$，norm $=0.314\ \mathrm m$，$0.02\ \mathrm{rad}$。每周期限幅 $d_{max}=0.05\ \mathrm m$、$a_{max}=0.01\ \mathrm{rad}$，则平移需要 $\lceil0.314/0.05\rceil=7$ 个周期，角度 2 个周期；总计 7 个周期（10 Hz 时 0.7 s）完成。期间等价于一个 $0.5\ \mathrm{m/s}$ 的"附加速度"叠加在车上——要和车速相比：车速 1 m/s 时影响显著，所以限速必须随车速调（修正率通常取车速的小分数，如 10–20%）。

一阶低通：$T_{pub}\leftarrow T_{pub}+\alpha(T_{target}-T_{pub})$。$\alpha=0.3$ 时，剩余误差 $0.7^n$，要降到 5% 以下需要 $n\ge9$。

**取舍**：平滑越强，下游越安心，但"真值"到达越慢；若 AMCL 的修正是对的，平滑相当于**有意引入的滞后**。

### 5.2 门限：拒绝可疑的大跳变

AMCL 偶尔给出明显错误的跳变（簇切换、杂波）。可以设置：

- 修正量超过阈值（如 0.5 m 或 10°）→ 不立即应用，需要连续 $N$ 次一致（滞回确认）才应用；
- 同时检查 AMCL 协方差：协方差变大 + 大跳变 → 更可疑；
- 与 EKF 的短期预测比较：若 EKF 的 IMU/轮速一致认为"车根本没怎么动"，而 AMCL 给出 0.5 m 位移 → 拒绝。

```python
# teaching sketch: confirm-before-apply for large map->odom corrections
def accept_correction(delta_xy, delta_th, cov_trace, state):
    big = (delta_xy > 0.5) or (abs(delta_th) > 0.17)   # 0.5 m or ~10 deg
    if not big:
        state["pending"] = 0
        return True
    state["pending"] += 1                  # count consecutive big corrections
    return state["pending"] >= 3 and cov_trace < 0.3   # confirm 3 times and cloud is tight
```

### 5.3 把 AMCL 作为测量融合进第二个 EKF

架构 B（[04 章](./04_EKF融合定位.md)）：EKF-2 以 AMCL 位姿为测量，用 $R$ 表达可信度，用马氏距离门限拒绝离群，由卡尔曼增益**自然**限制修正量（$K<1$）。优点是统一、有统计基础；缺点是需要给 AMCL 位姿一个合理的 $R$（AMCL 的协方差往往偏乐观）。

---

## 6. 下游如何"隔离"跳变

跳变不能完全消除，要保证它**不穿透**到控制。三道防线：

**防线一：局部模块放在 `odom` 系**。局部代价地图（滚动窗口）、局部规划、控制在 `odom` 系里工作，障碍和轨迹相对 `odom` 保持不动，`map→odom` 的台阶不会让它们"瞬移"。

**防线二：全局路径"搬进"odom**。全局规划在 `map` 系。每个周期用当前 $T_{odom\,map}=T_{map\,odom}^{-1}$ 把全局路径变换到 `odom`。路径点 $p_{map}$ → $p_{odom}=T_{map\,odom}^{-1}p_{map}$。AMCL 台阶发生时，**路径在 odom 里平移了一个台阶**，控制器看到的是参考的突变——所以这条通道依然要靠第 5 节的限速平滑来保护。

**防线三：控制器自身的鲁棒性**。控制器对参考的跳变要能限幅（转角速率限制、$\Delta u$ 惩罚）、对状态噪声要滤波、对异常有降速/停车的 fallback。MPC 的输入变化率约束就是这条线（见 [10 章](./10_MPC轨迹跟踪.md)）。

---

## 7. 连续车辆状态接口

CV 说 "output a continuous vehicle state"。工程上这是一份**契约**：

| 字段 | 内容 | 约定 |
|---|---|---|
| `stamp` | 状态对应的时间 | **测量时刻**，不是发布时刻；单调不倒退 |
| `frame` | 位姿所在坐标系 | 控制/局部用 `odom`；全局用 `map` |
| `x, y` | 参考点位置 | m；参考点 = 后轴中心（与模型一致；本项目实车即取后轴中心） |
| `yaw` | 航向 | rad，$[-\pi,\pi]$ 或 unwrap，文档写清 |
| `v` | 纵向速度 | m/s，**带符号**（倒车为负） |
| `omega` | 偏航角速度 | rad/s，来自陀螺融合 |
| `kappa`（可选） | 曲率 | $\omega/v$ 或 $\tan\delta/L$，$|v|$ 小时要保护 |
| `steer`（可选） | 前轮转角 | rad，反馈值或估计 |
| `cov` | 协方差 | 位置、航向方差 |
| `valid` | 有效性 | 数据新鲜度、协方差阈值、门限都通过时为真 |

**连续性的判据**（可作为自检指标）：

- 相邻状态位置差 $\Delta p\le v_{max}\Delta t+\epsilon$（不存在瞬移）；
- 航向差 $|\Delta\psi|\le\omega_{max}\Delta t+\epsilon$；
- `v`、`omega` 与位姿差分一致（一致性检查）。

---

## 8. 时间对齐与延迟

1. **用测量时刻查 TF**：AMCL 位姿对应激光时刻，而不是当前；融合/使用时用 `lookupTransform(…, scan_stamp)`；
2. **向前外推**：控制器需要"现在"的状态，EKF 要从最近一次更新外推到当前时刻；
3. **`transform_tolerance`**：AMCL 把 `map→odom` 的时间戳向未来延长一小段，避免消费者查询最新时刻时外推失败；设得太大，会在 AMCL 掉线时仍然使用过期变换，应配合心跳检查；
4. **延迟补偿**：MPC 用状态外推补偿控制链路延迟（见 10 章）；
5. **统一时钟**：多机场景必须同步（chrony/PTP），否则 TF 时间对不上。

---

## 9. 倒车与方向对协同的影响

倒车时：

- EKF：若速度符号正确，里程预测方向对，协方差增长正常；
- AMCL：运动模型须对称处理倒车（见 05 章 3.3 节），否则粒子云炸开，观测更新后选到错误簇 → `map→odom` 大跳；
- 两者一旦方向不一致（EKF 认为在前进，激光看到在后退），AMCL 的修正会很大，并且在每次倒车段都复发。

排查"倒车才跳"类问题的标准动作：同一路径前进 vs 倒车各跑一次，对比 `map→odom` 的阶跃序列。

---

## 10. 跳变排查流程

1. **量化**：用 `ros2 bag record` 录包；对 `map→odom` 序列计算相邻发布的差 $\Delta p,\Delta\theta$；找出大于阈值的时刻；
2. **对齐事件**：该时刻车在做什么（转弯/倒车/启动/刚收到初始位姿）、AMCL 日志、粒子云形状、协方差；
3. **分流**：
 - 跳变与转弯角速度相关 → 时间戳/去畸变（03 章）；
 - 跳变仅在倒车 → 方向符号（02/03/05 章）；
 - 跳变总是同一个固定偏移 → 静态变换/安装（03 章）；
 - 跳变随行驶距离周期性出现 → 漂移累积修正（调 AMCL 门限/加平滑）；
 - 跳变与操作同时 → 初始位姿/relocalization；
 - TF 在两个值之间闪烁 → 重复发布；
4. **修数据，再调参数**；
5. **验证**：同一场景复现，对比跳变直方图（次数、最大值），并在控制侧观察转向指令的突变是否消失。

```python
# teaching sketch: find steps in a map->odom log (list of (t, x, y, yaw))
import math
def find_steps(log, d_thr=0.05, a_thr=0.02):
    steps = []
    for (t0, x0, y0, a0), (t1, x1, y1, a1) in zip(log, log[1:]):
        dp = math.hypot(x1 - x0, y1 - y0)
        da = (a1 - a0 + math.pi) % (2 * math.pi) - math.pi   # wrap to [-pi, pi)
        if dp > d_thr or abs(da) > a_thr:
            steps.append((t1, dp, da))
    return steps
```

---

## 11. 在 ROS2 里怎么落地：节点、话题、QoS 与 TF 发布权

前十节讲的是原理，这一节回答"在 ROS2 Humble 上，谁发布什么、怎么配置、怎么检查"。这是面试官很爱追问的一层：你说"EKF 发 odom→base_link，AMCL 发 map→odom"，他会接着问"怎么保证没有第三个人也在发？"

### 11.1 谁是 TF 的唯一发布者

| 变换 | 发布者（本项目） | 关键参数 | 发布频率 |
|---|---|---|---|
| `odom→base_link` | `robot_localization` 的 `ekf_node` | `publish_tf: true`、`world_frame: odom`、`odom_frame: odom`、`base_link_frame: base_link` | 与 EKF 频率一致，30 Hz |
| `map→odom` | 本项目取值：`nav2_amcl` 发布原始值，再由一个自写的"修正管理"节点做平滑后发布 | AMCL 侧 `tf_broadcast: false`（交给管理节点），管理节点发布 | 与 EKF 同步，30 Hz（台阶之间重复发布同一值，平滑期内逐周期更新） |
| `base_link→laser_link`、`base_link→imu_link` | `robot_state_publisher`（读 URDF）或 `static_transform_publisher` | URDF 中的 `origin xyz rpy` | 静态（`/tf_static`） |

> **注意**：`nav2_amcl` 默认自己发布 `map→odom`（参数 `tf_broadcast` 默认为 true）。如果你再写一个节点也发 `map→odom`，TF 树里同一个子节点就有了两个发布者，表现为 RViz 里车辆在两个位置之间闪。这正是 8 类成因里的第 8 类。本项目的做法是关掉 AMCL 自带广播，只让平滑节点发布，并在启动自检里检查 `/tf` 中 `child_frame_id == odom` 的消息只有一个 `header.frame_id` 来源。

这里"平滑节点"的角色在 ROS2 里可以这样组织：

```text
nav2_amcl ──(amcl_pose, PoseWithCovarianceStamped, 10 Hz 上下)──┐
                                                                │
ekf_node ──(odom→base_link via /tf, 30 Hz)──────────────────────┤
                                                                ▼
                                       map_odom_manager (rclcpp 节点, 30 Hz 定时器)
                                       1) 取 amcl_pose，用 tf2 buffer 在该消息时刻查 odom→base_link
                                       2) 算目标 T_map_odom = T_map_base * inv(T_odom_base)
                                       3) 门限确认 + 限速平滑
                                       4) 发布 /tf (map→odom)，stamp = 当前时间 + transform_tolerance
```

"在该消息时刻查 odom→base_link"这一步非常关键：AMCL 的位姿对应激光时刻（比当前时刻早约 0.1 s），必须用同一时刻的 `odom→base_link`，否则 $T_{map\,odom}$ 里会混入"车在这 0.1 s 里走的距离"，速度 0.5 m/s 时就是 5 cm，转弯时还会带角度误差。tf2 的 `lookupTransform(target, source, tf2::TimePoint)` 就是干这个的，缓冲区长度默认 10 s，足够。

### 11.2 话题与 QoS

| 话题 | 类型 | 发布者 | 订阅者 | QoS 设置（本项目取值） |
|---|---|---|---|---|
| `/imu/data` | `sensor_msgs/Imu` | IMU 驱动 | `ekf_node` | `SensorDataQoS`（BestEffort，深度 5）|
| `/wheel/odom` | `nav_msgs/Odometry` | 底盘桥接节点 | `ekf_node` | `SensorDataQoS` 或 Reliable 深度 10，两端必须一致 |
| `/scan` | `sensor_msgs/LaserScan` | 激光驱动 | `nav2_amcl`、局部代价地图 | `SensorDataQoS` |
| `/odometry/filtered` | `nav_msgs/Odometry` | `ekf_node` | MPC、TEB、状态接口 | Reliable，深度 10 |
| `/amcl_pose` | `PoseWithCovarianceStamped` | `nav2_amcl` | 修正管理节点、监控 | `Transient Local + Reliable`（`nav2_amcl` 的默认），便于晚加入的节点拿到最后一条 |
| `/tf` | `tf2_msgs/TFMessage` | 各 TF 发布者 | tf2 监听者 | 默认 Reliable，深度 100 |
| `/tf_static` | 同上 | 静态发布者 | tf2 监听者 | `Transient Local`，所以后启动的节点也能拿到 |

ROS2 里最常见的"没数据"原因是 **QoS 不兼容**：订阅者要 Reliable、发布者发的是 BestEffort 时，两边根本不会匹配，节点也不报错，只是收不到。排查命令是 `ros2 topic info /scan -v`，它会列出每个端点的 Reliability、Durability、History。本项目早期出现过一次"激光驱动发 BestEffort、代价地图用 Reliable 订阅"，表现为局部代价地图一片空白，不是算法问题。

### 11.3 参数怎么管

所有可调量都放在一个参数 YAML 里，用 `ros2 launch` 的 Python launch 文件加载。修正管理节点的参数示意（教学示例，数值与自查表一致）：

```yaml
map_odom_manager:
  ros__parameters:
    publish_rate: 30.0            # Hz，与 EKF 一致
    rate_ratio: 0.2               # 修正速率 = 0.2 * |v|
    rate_min: 0.05                # m/s
    rate_max: 0.20                # m/s
    yaw_rate_max: 0.15            # rad/s
    jump_dist: 0.5                # m，超过视为可疑
    jump_yaw_deg: 10.0            # deg
    confirm_count: 3              # 连续一致次数
    cov_trace_max: 0.3            # 协方差迹上限
    transform_tolerance: 0.3      # s
```

参数改了怎么生效：可以用 `ros2 param set` 在线改（节点要在回调里处理 `add_on_set_parameters_callback`），也可以改 YAML 重启。实车调参时我们的习惯是：在线改只用来试，**结论一律写回 YAML 并提交**，否则下次启动参数就丢了（见 [20 章](./20_工程化实践_频率周期与问题解决.md)的参数管理一节）。

### 11.4 生命周期与启动顺序

`nav2_amcl`、`map_server`、代价地图都是生命周期节点（lifecycle node），由 `nav2_lifecycle_manager` 依次 `configure → activate`。定位相关的启动顺序必须是：

1. `robot_state_publisher` 与静态 TF；
2. 传感器驱动、底盘桥接；
3. `ekf_node`（等有 IMU 和轮速后才开始发 `odom→base_link`）；
4. `map_server`（发布地图，Transient Local）；
5. `nav2_amcl`（激活后等待初始位姿，没有初始位姿它不会发 `map→odom`）；
6. 修正管理节点；
7. 规划、控制。

如果 5 比 3 早，AMCL 会在没有 `odom→base_link` 的情况下报 `Failed to transform initial pose in time`；如果 7 比 6 早，规划器会拿到一个"没有 map→odom"的世界。启动脚本里用 lifecycle manager 的 `autostart` 加 `bond_timeout`，并让上层任务节点在 `/amcl_pose` 的协方差迹小于阈值后才接受目标——这也是"有效性标志 `valid`"的来源。

---

## 12. 把平滑做成一个可验证的节点：逐周期手算

5.1 节给了一个 7 个周期的例子。实际节点的限速不是固定 0.05 m/周期，而是随车速变化，下面完整走一遍，帮你理解"为什么选车速的 20%"。

**设定**（教学示例，与自查表口径一致）：定时器 30 Hz，$\Delta t=1/30=0.0333$ s；车速 $v=0.5$ m/s；修正速率 $r=\mathrm{clip}(0.2v,\,0.05,\,0.2)=\mathrm{clip}(0.1,\,0.05,\,0.2)=0.1$ m/s；每周期最大平移 $d_{max}=r\Delta t=0.1\times0.0333=0.00333$ m。

目标修正 $\Delta=(−0.298,\,−0.0999)$，模长 $0.314$ m。所需周期数 $n=\lceil0.314/0.00333\rceil=95$ 个周期，约 3.2 s。这比 5.1 节"7 个周期"慢很多——因为那个例子是 10 Hz、每周期 5 cm（等效 0.5 m/s 的附加速度），而本项目的取值是车速的 20%。**为什么要慢？** 附加速度 0.1 m/s 叠加在车速 0.5 m/s 上，对横向偏差的影响是 $0.1/0.5=0.2$，约 11° 的等效航向偏差，MPC 能吸收；如果用 0.5 m/s 附加速度，等效 45°，车会明显"斜着走"。

**和一次性跳变相比**：直接跳 0.314 m，MPC 在下一周期看到横向误差 0.3 m，$\delta$ 指令瞬间饱和到 0.5 rad；平滑后，每个周期横向误差只变化 3.3 mm，对转角指令的影响小于 0.001 rad/周期。

**角度**：$\Delta\theta=0.02$ rad，$\dot\theta_{max}=0.15$ rad/s，每周期 $0.005$ rad，4 个周期就够，比平移早完成，不是瓶颈。但要注意**平移与旋转同时修正会交叉**：$T_{map\,odom}$ 的平移分量是在 map 系下表达的，旋转改变后，由它算出的 $T_{map\,base}$ 位置会绕 odom 原点偏转，偏转量等于 $|p_{odom\,base}|\cdot\Delta\theta$。车离 odom 原点 20 m 远时，$0.02$ rad 的旋转修正会让 map 里的位置变化 $20\times0.02=0.4$ m。这是一个常见误解：以为 $\Delta\theta=1°$ 是小量，其实在远离原点处效果很大。所以门限里的角度阈值要比直觉更严，也解释了为什么本项目把"10°"和"0.5 m"并列放进门限。

**节点的伪代码（教学示例，非项目源码）**：

```python
class MapOdomManager:
    def __init__(self):
        self.T_pub = SE2(0.0, 0.0, 0.0)      # 已发布的 map->odom
        self.T_target = SE2(0.0, 0.0, 0.0)   # 最近一次被接受的目标
        self.pending = 0

    def on_amcl(self, msg, T_odom_base_at_stamp):
        T_map_base = SE2.from_pose(msg.pose.pose)
        T_new = T_map_base * T_odom_base_at_stamp.inverse()
        d = (T_new.xy - self.T_pub.xy).norm()
        a = wrap(T_new.yaw - self.T_pub.yaw)
        cov_tr = msg.pose.covariance[0] + msg.pose.covariance[7]
        if d > 0.5 or abs(a) > deg2rad(10):
            self.pending += 1
            if self.pending < 3 or cov_tr > 0.3:
                return                     # 暂不接受
        else:
            self.pending = 0
        self.T_target = T_new

    def on_timer(self, v_abs, dt):
        r = min(max(0.2 * v_abs, 0.05), 0.20)
        delta = self.T_target.xy - self.T_pub.xy
        step = min(delta.norm(), r * dt)
        if delta.norm() > 1e-9:
            self.T_pub.xy += delta / delta.norm() * step
        dyaw = wrap(self.T_target.yaw - self.T_pub.yaw)
        self.T_pub.yaw += max(-0.15 * dt, min(0.15 * dt, dyaw))
        self.publish(self.T_pub)
```

**验证办法**：（1）单元测试：给一个 0.3 m 的阶跃目标，检查节点输出的相邻差始终不超过 $r\Delta t$，且最终收敛到目标；（2）回放测试：回放含跳变的 bag，对比平滑前后 `map→odom` 的 `find_steps` 结果，期望平滑后相邻周期阶跃小于 0.05 m（自查表第 12 项的验收标准）；（3）实车测试：直接观察 MPC 转角指令的 $\Delta\delta$ 直方图，大于 0.05 rad 的次数应显著减少。

---

## 13. 日志判读示例：三份典型 `map→odom` 序列

下面是三份教学用的序列（时间、x、y、yaw 角度），用来练习"看一眼就分类"。

**序列 A（漂移累积后被纠正）**：

```text
t(s)   x(m)    y(m)   yaw(deg)
10.0  -0.012   0.020   0.4
10.1  -0.012   0.020   0.4
...                                 (停了 8 s 没更新, 因为车几乎没动, 未达 update_min_d)
18.0  -0.012   0.020   0.4
18.1  -0.140   0.071   1.1         <- 一个台阶: 0.14 m, 0.7 deg
18.2  -0.140   0.071   1.1
```

特征：台阶发生在 AMCL 更新点，大小与上次修正以来的行驶距离近似成正比（走了约 12 m，漂了 1% 就是 0.12 m），方向稳定。这是**正常现象**，对策是平滑，不是调数据。

**序列 B（转弯时时间戳错位）**：

```text
t(s)   x(m)    y(m)   yaw(deg)   车辆角速度 (deg/s)
30.0   0.100   0.030   0.5        2
30.5   0.102   0.031   0.5        3
31.0   0.180   0.095   3.8        25   <- 开始转弯
31.5   0.050   -0.060 -2.9        32
32.0   0.210   0.110   4.2        30   <- 来回晃
```

特征：阶跃大小与角速度成正比，符号来回变（因为每帧观测的拖影方向不同）。60 ms 的时间戳偏差，在 30 deg/s 的角速度下等效 $30\times0.06=1.8°$ 的角度偏差，在 5 m 外的墙体上就是 $5\times\tan1.8°\approx0.16$ m 的横向偏差——数量级与序列 B 的台阶一致。对策：统一时间戳（03 章）。

**序列 C（倒车方向符号错）**：

```text
t(s)   x(m)    y(m)   yaw(deg)   挡位
50.0   1.00    0.50    1.0       前进
50.5   1.00    0.50    1.0       前进
51.0   1.15    0.52    1.2       倒车开始
51.5   1.55    0.60    1.9       倒车
52.0   2.10    0.70    2.6       倒车     <- x 随倒车距离单调增加, 持续漂
```

特征：只有倒车段出现，且台阶总是同一个方向（因为 odom 把倒车当作前进，每走一米多出两米的距离差，AMCL 每次更新把它拉回一点）。对策：方向符号（02/03/05 章）。

> **面试要点**：能说出"我不是先调 AMCL 参数，而是先看这条序列的形状"，会明显加分。形状和成因是一一对应的：随行驶距离走的是漂移，随角速度走的是时间，只在倒车的是符号，固定偏移的是静态变换，两个值来回闪的是重复发布。

---

## 14. 重定位、丢失与安全降级

连续状态接口里有一个 `valid`，这里说明它什么时候变假、变假之后系统怎么办，这是"状态可信"链路的最后一环。

| 触发条件 | 检测方法 | 动作 |
|---|---|---|
| `/amcl_pose` 超过 1 s 没更新（车在动） | 修正管理节点内的看门狗 | 继续用最后一个 `map→odom`，`valid` 仍为真，但上调定位不确定度；3 s 仍无更新则降速 |
| 协方差迹 > 0.3 持续 2 s | 读 `amcl_pose.covariance` | `valid=false`，MPC 速度上限降到 0.2 m/s，任务层暂停发新目标 |
| 候选修正超过门限且未确认 | 门限逻辑的 `pending` 计数 | 不应用，累计到 3 次若协方差仍大则触发全局重定位（`reinitialize_global_localization` 服务） |
| 全局重定位后位姿跳 > 1 m | 对比重定位前后 | 停车、清空局部代价地图、取消当前导航，等待操作员确认 |
| EKF 输入缺失（IMU 或轮速超过 100 ms 没来） | EKF 的诊断话题 `/diagnostics` | 立即 `valid=false`，MPC 减速停车，不等定位 |

**为什么重定位后要停车？** 因为重定位后 $T_{map\,odom}$ 可能一次改变 1 m 以上，全局路径在 odom 系的表达整体平移，TEB 与 MPC 的参考作废。继续行驶只会把一个错误的状态放大。自查表第 22 项里的两次失败之一"玻璃门处重定位错误"就属于这一类——这次失败里，门限逻辑拒绝了一次大跳变，但随后连续 3 次一致地给出了错误的簇，协方差也很小（玻璃门让激光看到的是对面的墙，匹配得非常自信），于是系统接受了错误的重定位。教训是：协方差小不等于对，在玻璃多的场景要给地图加"虚拟墙"或限制全局重定位的使用，这一点在 [11 章](./11_联调与故障案例.md) 的复盘里也提到。

---

## 15. 面试问答 / 要点速记 / 自测题 / 链接

### 面试问答

**Q1. Why does AMCL publish map→odom rather than map→base_link?**

> Because base_link already has odom as its parent, and a frame can only have one parent. So AMCL computes the map-to-base pose it estimated, multiplies it by the inverse of the current odom-to-base transform, and publishes the result as map-to-odom. Then looking up map-to-base gives the AMCL pose, but odom-to-base stays smooth.

**Q2. How did you make the vehicle state continuous?**

> There are two parts. First the source of the continuous state is the EKF fusing wheel speed and the IMU yaw rate, which is smooth by construction. Second, the localisation correction is kept in the map-to-odom transform, so the planner and controller operate in odom and don't see the steps directly; large corrections are rate-limited or confirmed before they are applied. The inconsistent timing, frames and motion direction were fixed at the data level first.
> 本项目取值约：修正速率上限取车速的约 20%，下限 0.05 m/s、上限 0.2 m/s，角度 0.15 rad/s；单次修正超过 0.5 m 或 10° 视为可疑，需连续 3 次一致且粒子云协方差迹小于 0.3 才应用；平滑带来的延迟约 1 s，可接受。

**Q3. How would you debug a pose jump on the real vehicle?**

> I log a bag, compute the step sizes of map-to-odom over time, and correlate them with what the vehicle was doing: turning, reversing, starting or relocalising. If steps scale with yaw rate I suspect timing; if they only happen in reverse I suspect sign handling; a constant offset points to a static transform; periodic steps mean accumulated drift being corrected. I fix the data first and tune the filter afterwards.

*追问*：What is the downside of smoothing? —— 引入滞后，若修正是对的，真值到达变慢。

### 本章要点速记

1. EKF 连续会漂，AMCL 不漂会跳；`odom→base_link`（EKF）+ `map→odom`（AMCL）互补。
2. $T_{map\,odom}=T_{map\,base}T_{odom\,base}^{-1}$；合成后 `map→base_link` 在两次更新间依然光滑。
3. 示例：0.316 m 的台阶会让 Stanley 类控制器产生约 17° 的转向瞬态。
4. 跳变 8 类成因，优先排查数据一致性（时间/坐标/方向/重复 TF）。
5. 限速平滑 7 步完成 0.3 m 修正；滞回确认挡住可疑大跳。
6. 三道防线：局部模块在 `odom`；全局路径变换进 `odom` 并平滑；控制器限幅。
7. 连续车辆状态：带测量时刻、带符号速度、带协方差与有效性标志。
8. 倒车时需验证 AMCL 的运动模型与 EKF 的符号。

### 自测题

1. 若 $T_{odom\,base}=(5,2,\pi/2)$，$T_{map\,base}=(7,1,\pi/2)$，求 $T_{map\,odom}$。（答：逆 $=(-x\cos\theta-y\sin\theta,\ x\sin\theta-y\cos\theta,\ -\theta)=(-5\cdot0-2\cdot1,\ 5\cdot1-2\cdot0,\ -\pi/2)=(-2,5,-\pi/2)$；组合 $T_{map\,base}\cdot$ 逆 $=(7+(-2)\cos\frac\pi2-5\sin\frac\pi2,\ 1+(-2)\sin\frac\pi2+5\cos\frac\pi2,\ 0)=(2,\ -1,\ 0)$。）
2. 为什么局部代价地图放在 `odom` 系？
3. 限速平滑的缺点是什么？如何随车速选取限速？
4. 说出两种区分"时间错位"与"静态变换错"的判据。
5. 架构 A 与架构 B 的区别、各自的优缺点？

### 相关链接

- 上一章：[05 AMCL](./05_AMCL粒子滤波定位.md)　下一章：[07 代价地图与车体外形](./07_代价地图与车体外形.md)
- 联调案例：[11 联调与故障案例](./11_联调与故障案例.md)
- 工程化：[20 工程化实践](./20_工程化实践_频率周期与问题解决.md)
- 返回：[README](./README.md)
