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
- 齐次变换：[../../02_基础知识/03_坐标变换与运动学/01_坐标系_旋转矩阵_齐次变换.md](../../02_基础知识/03_坐标变换与运动学/01_坐标系_旋转矩阵_齐次变换.md)
- 时间同步与坐标系一致性：[../../02_基础知识/05_状态估计/06_时间同步_坐标系一致性与数据有效性.md](../../02_基础知识/05_状态估计/06_时间同步_坐标系一致性与数据有效性.md)

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
11. 面试问答 / 要点速记 / 自测题 / 链接

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
| 3 | **粒子簇切换** | 多峰，主簇换了 | 看 `/particlecloud` 是否出现两团 |
| 4 | **初始位姿/重定位** | 手动 re-initial pose，或 recovery 注入随机粒子 | 跳变与操作/日志时间一致 |
| 5 | **时间戳错位** | 激光/里程/IMU 时间不一致，匹配拉偏 | 跳变量与速度、角速度成正比，转弯时更明显 |
| 6 | **坐标系/静态变换错** | `laser_link` 安装位姿错，匹配系统性偏 | 位姿总偏一个固定量；直行 vs 转弯偏差不同 |
| 7 | **方向（倒车）符号错** | 里程预测方向与激光观测矛盾 | 倒车时跳变，前进时正常 |
| 8 | **TF 重复发布/树错误** | 两个节点发布同一变换；或 TF 缓冲外推 | `rqt_tf_tree` 父节点冲突；TF 在两个值间闪烁 |

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

1. **量化**：录 bag；对 `map→odom` 序列计算相邻发布的差 $\Delta p,\Delta\theta$；找出大于阈值的时刻；
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

## 11. 面试问答 / 要点速记 / 自测题 / 链接

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
- 返回：[README](./README.md)
