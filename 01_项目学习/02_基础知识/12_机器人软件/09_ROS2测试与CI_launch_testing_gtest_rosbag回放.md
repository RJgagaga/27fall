# 09 ROS2 测试与 CI：单元测试、launch_testing、rosbag2 回放与流水线

> **一句话定位**：机器人软件最贵的 bug 不是"编译不过"，而是"改了一行滤波参数，三天后实车才发现定位漂了"。本章讲怎样把"我觉得没问题"变成"流水线告诉我没问题"：纯逻辑用单元测试（gtest/pytest）锁住，多节点交互用 `launch_testing` 在真实 DDS 上验证，真实传感器数据用 rosbag2 回放做回归基准，最后把它们串成一条 CI 流水线。

**本章学完你能：**

- 画出机器人软件的测试金字塔，说清单元测试、节点集成测试、回放回归、硬件在环、实车各自抓什么、不抓什么；
- 把控制逻辑写成"与 ROS 无关的纯函数 + 薄薄的节点外壳"，从而能用 gtest/pytest 秒级测试；
- 写出一个 `launch_testing` 测试：拉起节点、等待就绪、发消息、断言输出、检查进程退出码；
- 用 `ros2 bag record/play/info` 做数据回放，说明 `--clock` 与 `use_sim_time` 的配合，并设计"指标 + 容差"的回归判据；
- 解释为什么测试会"间歇失败"（flaky），手算它对流水线通过率的冲击，并给出隔离、重试、确定性三类对策；
- 写出一份 GitHub Actions 风格的 ROS2 Humble 流水线（构建、测试、结果汇总），并说清哪些命令名属于"未核对"；
- 说明自己项目里的"测试与回归"做了什么、哪些 CI 工作只是延伸知识。

**前置知识：**

- [ROS2 Humble 核心概念与 ROS1 对比](./04_ROS2_Humble核心概念与ROS1对比.md)（节点、话题、QoS）
- [ROS2 Humble 实战：launch、参数、生命周期、组件](./08_ROS2_Humble实战_launch_参数_生命周期_组件.md)（Python launch、参数、`use_sim_time`）
- [Linux 与 Shell、Git](./01_Linux与Shell_Git.md)（命令行、环境变量）
- [Python 工程与科学计算](./03_Python工程与科学计算.md)（pytest 基础）

> **关于代码的说明**：本机没有安装 ROS2 与 Docker，所以 `launch_testing`、`ament_cmake_gtest`、GitHub Actions 的片段**没有在 ROS 环境里运行**，只做结构示意，凡涉及具体包名、动作名、参数名且不能确认的地方标"未核对"。能离线运行的部分——纯 Python 的限幅函数、pytest 用例、回放回归指标与互相关延迟估计、flaky 概率算术——都已在本地运行复核（文中标"Python 复核"）。所有数值为**教学示例（非项目实测）**。

**目录**

1. 为什么机器人需要自动化测试
2. 测试金字塔与分层策略
3. 单元测试：把逻辑从 ROS 里"拔出来"
4. ament 里的测试：gtest、pytest 与 lint
5. launch_testing：多节点集成测试
6. rosbag2 回放：用真实数据做回归基准
7. 回归测试怎么设计：指标、容差与基线
8. flaky 测试：为什么间歇失败，怎么治
9. CI 流水线：从提交到报告
10. 在七个项目里怎么用（含诚实边界）
11. 常见错误与故障现象
12. 面试怎么问、怎么答
13. 本章要点速记
14. 自测题
15. 相关链接

---

## 1. 为什么机器人需要自动化测试

### 1.1 要解决的问题

软件界的"修改—测试"循环假设测试便宜、结果确定。机器人上这两点都被破坏：

1. **测试贵**：实车测试要人、场地、电量，一天做不了几次；
2. **结果不确定**：传感器噪声、调度抖动、网络丢包，让同一份代码两次运行的结果不完全相同；
3. **改动牵一发动全身**：改一个坐标系约定、一个 QoS、一个时间戳来源，下游多个节点同时受影响；
4. **失败代价高**：执行层的错误可能变成真实的动作。

所以必须把"能自动判断的部分"尽可能前移，让实车测试只负责"自动化测不到的东西"。这本质上是把测试成本从"每次都花一个下午"压缩到"每次提交花几分钟"。

### 1.2 自动化测试到底能保证什么

诚实的边界：自动化测试**不能**证明机器人"在实车上一定好用"，它能做的是：

- 逻辑回归：改动没有破坏已经验证过的行为；
- 接口契约：话题名、消息类型、QoS、参数名没被无意改掉；
- 数据回放一致性：同一段真实数据，输出指标没有劣化；
- 安全兜底：非法输入（NaN、超时、越界）仍走到安全分支。

它**不能**替代：真实摩擦、真实延迟、真实电机的行为。这些仍由台架与实车完成，见本章 §2。

> **面试要点**：谈测试时先讲"每一层抓什么 bug"，而不是罗列工具名。工具会换，分层的道理不会。

---

## 2. 测试金字塔与分层策略

```text
            ▲  少、慢、贵、最真实
           /█\        实车测试（场地、整机）
          /███\       硬件在环 / 台架（真实控制器 + 假负载）
         /█████\      回放回归（真实数据，rosbag2）
        /███████\     节点集成测试（launch_testing，真实 DDS）
       /█████████\    单元测试（gtest / pytest，纯逻辑）
      ▼  多、快、便宜、较抽象
```

### 2.1 每层抓什么

| 层 | 运行位置 | 典型耗时 | 能抓 | 抓不到 |
|---|---|---|---|---|
| 单元测试 | 开发机/CI | 毫秒–秒 | 算术、状态机分支、边界值、NaN 处理 | 节点间接口、时序 |
| 节点集成 | CI（真实 DDS） | 秒–十几秒 | 话题连通、QoS 兼容、参数加载、启动顺序 | 真实传感器分布 |
| 回放回归 | CI 或开发机 | 秒–分钟 | 算法在真实数据上的指标劣化 | 闭环效应（回放是开环） |
| 硬件在环/台架 | 实验室 | 分钟 | 驱动、总线、实时性、使能与保护 | 整机动力学 |
| 实车 | 场地 | 小时 | 一切，但不可重复 | — |

**关键的一点——回放是开环的**：回放把录好的传感器数据重新喂给算法，但算法的输出不会反过来改变传感器数据（车不会因为你的新控制器而走出另一条轨迹）。所以回放适合测**估计、感知、地图、规划的输出质量**，不适合测**闭环稳定性**。闭环稳定性要靠仿真或实车。

### 2.2 一条工程准则

> 能在下一层测的，不要留给上一层。每往上走一层，成本大约增加一个数量级，而可重复性下降。

---

## 3. 单元测试：把逻辑从 ROS 里"拔出来"

### 3.1 问题：节点里的逻辑怎么测

最常见的写法是把所有东西塞进节点类的回调里：订阅回调里算限幅、查超时、改状态、发布。要测它，就得启动整个 ROS 上下文。结果是测试慢、依赖多、没人愿意跑。

### 3.2 做法：纯函数核心 + 薄外壳

```text
节点外壳 (rclcpp / rclpy)           纯逻辑核心 (无 ROS 依赖)
  订阅 → 取出数值与时间戳  ───►   limit_cmd(v_cmd, v_prev, v_max, a_max, dt)
  ◄── 发布结果             ◄───   返回新的命令
```

核心只吃数字、返回数字，所有 ROS 类型的转换留在外壳。这样核心能在没有 ROS 的机器上被 pytest/gtest 测试。

### 3.3 一个可运行的例子（教学示例，非项目源码）

被测函数：速度命令限幅 + 加速度限幅 + NaN 失效安全（fail-safe）。

```python
# limiter.py（教学示例，非项目源码）
import math

def limit_cmd(v_cmd, v_prev, v_max, a_max, dt):
    """限制 |v| <= v_max 且 |dv|/dt <= a_max；非有限值按 0 处理（失效安全）。"""
    if not math.isfinite(v_cmd):
        return 0.0
    v = max(-v_max, min(v_max, v_cmd))
    dv = a_max * dt
    return max(v_prev - dv, min(v_prev + dv, v))
```

测试用例：

```python
# test_limiter.py
import pytest
from limiter import limit_cmd

def test_clamp_speed():
    assert limit_cmd(5.0, 1.0, 1.0, 100.0, 0.1) == 1.0

def test_rate_limit():
    # 上一拍 0，加速度上限 0.5 m/s^2，dt=0.1 s，一拍最多增加 0.05
    assert limit_cmd(1.0, 0.0, 2.0, 0.5, 0.1) == pytest.approx(0.05)

def test_nan_failsafe():
    assert limit_cmd(float('nan'), 0.3, 1.0, 1.0, 0.1) == 0.0
```

**Python 复核**：运行 `pytest -q`，5 个用例（含一个参数化用例）全部通过。

**手算验证 `test_rate_limit`**：$\Delta v_{\max}=a_{\max}\Delta t=0.5\times0.1=0.05$；目标 1.0 被先钳到 $\min(2.0,1.0)=1.0$；再被变化率限制到 $0+0.05=0.05$。与断言一致。

> **注意**：这个函数对 NaN 返回 0，而不是返回 `v_prev`。"NaN 时保持上次速度"看似平滑，但如果上游一直输出 NaN，车会按旧速度一直开下去——这是危险的默认行为。失效安全的默认应该是**减速到停**。测试必须把这个选择写成断言，否则以后有人"优化"掉。

### 3.4 单元测试该覆盖的"五类输入"

1. **正常值**：典型输入，验证数值；
2. **边界值**：恰好等于限幅、恰好为 0；
3. **符号对称**：正反方向行为对称（容易漏的 bug 来源）；
4. **非法值**：NaN、inf、负的 $\Delta t$、$\Delta t=0$（除零）；
5. **状态序列**：多拍调用，验证状态机转移（如迟滞、去抖）。

---

## 4. ament 里的测试：gtest、pytest 与 lint

### 4.1 C++ 包（ament_cmake）

在 `package.xml` 里声明测试依赖：

```xml
<test_depend>ament_lint_auto</test_depend>
<test_depend>ament_lint_common</test_depend>
<test_depend>ament_cmake_gtest</test_depend>
<test_depend>launch_testing_ament_cmake</test_depend>
```

在 `CMakeLists.txt` 里用 `BUILD_TESTING` 包住测试目标，使得在不需要测试时（例如发布构建）不编译测试：

```cmake
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_limiter test/test_limiter.cpp)
  target_link_libraries(test_limiter limiter_core)

  add_launch_test(test/test_chain.launch.py)
endif()
```

`ament_add_gtest` 会把 gtest 可执行文件注册为 CTest 目标，`colcon test` 通过它收集结果。gtest 用例的基本形式：

```cpp
// test/test_limiter.cpp（教学示例，非项目源码）
#include <gtest/gtest.h>
#include <cmath>
#include "limiter_core/limiter.hpp"

TEST(Limiter, RateLimit) {
  EXPECT_NEAR(limit_cmd(1.0, 0.0, 2.0, 0.5, 0.1), 0.05, 1e-12);
}
TEST(Limiter, NanIsZero) {
  EXPECT_EQ(limit_cmd(std::nan(""), 0.3, 1.0, 1.0, 0.1), 0.0);
}
```

浮点比较用 `EXPECT_NEAR`，不要用 `EXPECT_EQ` 比较算术结果（舍入误差）。只有"精确的特殊值"（如 NaN 分支返回字面量 0.0）才适合精确比较。

### 4.2 Python 包（ament_python）

`ament_python` 包的测试放在包内的 `test/` 目录，用 pytest。`setup.py` 里声明 `tests_require=['pytest']`。通过 `ros2 pkg create --build-type ament_python` 生成的模板，通常会附带 `test_copyright.py`、`test_flake8.py`、`test_pep257.py` 三个风格检查测试（具体模板内容随版本变化，**未核对**）。

### 4.3 lint 也是测试

`ament_lint_auto` 会自动收集已声明的 lint 依赖并执行：代码风格（`ament_cpplint`、`ament_uncrustify`、`ament_flake8`）、静态分析（`ament_cppcheck`）、版权、XML 格式等。把风格问题放进 `colcon test` 的好处是**每个人的提交都被同一把尺子检查**，评审时不必争论缩进。

### 4.4 常用命令

```bash
# 构建
colcon build --symlink-install
# 只测某个包，并显示失败输出
colcon test --packages-select demo_chassis --event-handlers console_direct+
# 汇总结果（失败用例会列出）
colcon test-result --verbose
```

`colcon test-result` 读取各包 `build/<pkg>/` 下的 XML 报告并汇总，CI 里常用它的退出码判断流水线成败（具体行为以所用 colcon 版本为准，**未核对**）。

---

## 5. launch_testing：多节点集成测试

### 5.1 解决什么问题

单元测试保证"函数对"，但"限幅节点真的订阅了 `/cmd_vel`、发布了 `cmd_vel_safe`、QoS 是否兼容、参数能否加载"，只有真正启动节点才能知道。`launch_testing` 的思路：**用 launch 文件拉起被测节点，再在同一个测试里跑 unittest，通过真实的 DDS 发消息、收消息、断言。**

### 5.2 结构

一个 `launch_testing` 文件通常包含三部分：

1. `generate_test_description()`：返回要启动的 LaunchDescription，最后加上 `launch_testing.actions.ReadyToTest()`，表示"到这里，被测系统可以开始测试了"；
2. 若干 `unittest.TestCase` 类：在进程**运行期间**执行的测试；
3. 以 `@launch_testing.post_shutdown_test()` 装饰的类：在进程**全部退出之后**执行，用来检查退出码和日志。

### 5.3 示例（结构示意，**未在 ROS 环境运行**）

```python
# test/test_chain.launch.py（教学示例，非项目源码）
import unittest
import rclpy
from geometry_msgs.msg import Twist
import launch
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
import launch_testing.asserts


def generate_test_description():
    limiter = Node(
        package='demo_chassis', executable='speed_limiter',
        name='speed_limiter',
        parameters=[{'v_max': 1.0, 'a_max': 0.5}],
    )
    return (
        launch.LaunchDescription([
            limiter,
            launch_testing.actions.ReadyToTest(),
        ]),
        {'limiter': limiter},
    )


class TestLimiterRuntime(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_probe')

    def tearDown(self):
        self.node.destroy_node()

    def test_output_is_bounded(self):
        received = []
        self.node.create_subscription(
            Twist, 'cmd_vel_safe', lambda m: received.append(m.linear.x), 10)
        pub = self.node.create_publisher(Twist, 'cmd_vel_in', 10)

        msg = Twist()
        msg.linear.x = 5.0           # 故意超过上限
        end = self.node.get_clock().now().nanoseconds + 3_000_000_000
        while self.node.get_clock().now().nanoseconds < end and len(received) < 5:
            pub.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.assertGreater(len(received), 0, '没有收到任何输出，接口或 QoS 可能不通')
        self.assertTrue(all(v <= 1.0 + 1e-9 for v in received))


@launch_testing.post_shutdown_test()
class TestExit(unittest.TestCase):
    def test_exit_code(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
```

### 5.4 要点与坑

1. **等待而不是睡眠**：发布者和订阅者之间有"发现"过程（见 [DDS 与分布式通信](./05_DDS与分布式通信.md)），刚启动时发的消息可能丢。正确做法是循环发布直到收到或超时，而不是 `sleep(1)` 之后发一条然后断言；
2. **超时要有**：任何等待都必须有上限，并在失败信息里说明"最可能是接口没通"；
3. **隔离 DDS 域**：多个测试并行时可能互相收到对方的消息。保守做法是为每个测试设不同的 `ROS_DOMAIN_ID`，或设置 `ROS_LOCALHOST_ONLY=1`；ament 工具链对此有内置隔离机制，具体名称与行为在 Humble 中**未核对**，不要在面试里断言；
4. **等待话题出现**：`launch_testing_ros` 提供了等待话题的辅助工具（`WaitForTopics` 一类），是否在你所用的 Humble 版本中可用**未核对**；不确定时用上面的"循环发布 + 超时"最稳；
5. **退出码检查**：`assertExitCodes` 能发现"节点其实崩溃了但测试碰巧通过"的情况，不要省略；
6. **被测节点需要 `use_sim_time`** 时，要在 launch 里传参数，并保证回放或测试提供 `/clock`。

---

## 6. rosbag2 回放：用真实数据做回归基准

### 6.1 为什么回放

真实传感器数据有仿真给不了的"脏"：噪声分布、丢帧、时间戳抖动、偶发异常。把一次现场录下来的数据当成固定的"考卷"，每次改算法都用同一份考卷打分，就得到了可比较的回归基准。

### 6.2 录制与回放命令

```bash
# 录制指定话题（Humble）
ros2 bag record -o run_001 /imu/data /wheel/odom /scan /tf /tf_static

# 查看包的信息：话题、消息类型、条数、时长
ros2 bag info run_001

# 回放：发布 /clock，并按原速（或倍速）发布消息
ros2 bag play run_001 --clock --rate 1.0
```

要点：

- `--clock` 让回放器发布 `/clock`。被测节点必须设置 `use_sim_time:=true`，否则它仍用墙钟（系统时间），而消息时间戳是录制时的时间，二者对不上，TF 查询会报"外推到未来/过去"；
- 回放速率 `--rate`：倍速回放更快，但如果算法有计算延迟，倍速会放大"来不及处理"的问题，使结果与实机不可比。做回归时通常用 1.0 倍速以求保真，做冒烟检查时可加速；
- 存储格式：Humble 默认存储插件为 sqlite3；MCAP 存储插件在 Humble 中的可用情况**未核对**；
- 录制 `/tf_static` 要特别注意：静态变换是"锁存"（transient local）的，只在开头发一次，**从中途开始录就会漏掉它**，回放时 TF 树不完整。解决：录制时包含该话题并从系统启动时就开始录，或回放时从 URDF 重新发布。

### 6.3 回放的确定性问题

回放本身是确定的（同一份数据），但下游算法不一定：

- 多线程执行器下回调的执行顺序可能不同；
- 算法里的随机数（粒子滤波的重采样）若不固定种子，结果每次略有差别；
- 回放与算法进程的启动顺序：回放太早开始，节点还没起来，就丢了前几秒数据。

对策：固定随机种子；先启动被测节点，等待就绪，再启动回放（`launch_testing` 里用 `ReadyToTest` 后再启动 `ros2 bag play` 进程）；对仍有随机性的算法，不比较逐点输出，而比较**统计指标并给容差**。

### 6.4 可运行的离线示意：回放回归指标

下面用一段合成数据模拟"回放后记录到的参考轨迹与实测轨迹"，计算两个回归指标：均方根误差（RMSE）、最大误差，以及用互相关估计延迟。（教学示例，非项目实测。）

```python
# replay.py（教学示例，非项目源码）
import math, random
random.seed(1)
dt = 0.02                                  # 50 Hz
bag = [(dt*i, math.sin(dt*i), math.sin(dt*i - 0.03) + random.gauss(0, 0.01))
       for i in range(500)]                # (t, 参考, 实测)，实测滞后 0.03 s 并带噪声
err = [m - r for _, r, m in bag]
rmse = math.sqrt(sum(e*e for e in err) / len(err))
emax = max(abs(e) for e in err)
best = min(range(-10, 11),
           key=lambda k: sum((bag[i][2] - bag[i-k][1])**2 for i in range(10, 490)))
print(rmse, emax, best*dt)
```

**Python 复核**：RMSE $\approx0.0239$，最大误差 $\approx0.0568$，互相关估计的滞后为 2 个采样点（0.04 s，真实滞后 0.03 s 介于 1 与 2 个采样点之间，取最近的整数点，噪声下偏向 2）。

**手算核对 RMSE 的量级**：无噪声时，滞后 $\tau=0.03$ s 的正弦 $\sin(t)$ 与 $\sin(t-\tau)$ 之差的幅值约为 $\tau\cdot\omega=0.03\times1=0.03$，RMS 约为 $0.03/\sqrt2=0.0212$；叠加标准差 0.01 的噪声后 $\sqrt{0.0212^2+0.01^2}=0.0235$，与计算值 0.0239 吻合。这个小例子说明：**回归指标应该能用手算量级解释**，不要只信一个黑箱数字。

---

## 7. 回归测试怎么设计：指标、容差与基线

### 7.1 四个要素

| 要素 | 含义 | 例子（示例，非项目取值） |
|---|---|---|
| 数据集 | 固定的一组录制数据，标明场景 | 直线、转弯、停走三段 |
| 指标 | 能量化好坏的数 | 位置 RMSE、航向 RMSE、里程计漂移率、最大跳变、延迟 |
| 基线 | 当前被接受版本在该数据上的指标 | RMSE = 0.024 |
| 容差 | 允许的劣化范围 | 相对基线劣化不超过 10% 或绝对不超过 0.005 |

### 7.2 容差的设定

容差太紧：随机波动就会把流水线打红，大家学会"忽略红灯"，测试失去意义；
容差太松：真正的劣化溜过去。

做法：**先对基线版本重复运行若干次，测出指标的自然波动（标准差 $\sigma$），容差取 $3\sigma$ 或更大**。如果指标波动本身就为零（完全确定性），容差可以收紧到浮点舍入量级。

### 7.3 "金标准"文件的维护

基线数值不应该写死在代码里，而应放在版本库中的小文件里（如 YAML），更新基线必须是一次**显式的、需要评审的提交**，并在提交说明里写明"为什么指标变了"（改进了算法？换了数据？）。否则回归测试会变成"谁改了都通过"。

### 7.4 对"安全不变量"做逐点断言

有些要求不能用统计容差：比如"输出速度绝不超过上限""任何时候命令年龄不超过超时值""NaN 输入必须走停车分支"。这些是**不变量**，在回放过程中对每一条输出做断言，一次违反即失败。统计指标管"好不好"，不变量管"安不安全"。

---

## 8. flaky 测试：为什么间歇失败，怎么治

### 8.1 现象与来源

flaky 测试指：代码没改，有时过有时挂。机器人测试里的来源：

1. 发现延迟：节点刚起，消息发早了；
2. 固定 `sleep` 假设：机器慢时不够；
3. 资源争用：CI 机器同时跑多个测试，CPU 抢占，定时器抖动；
4. 共享 DDS 域：并行测试互相串扰；
5. 随机性：未固定种子；
6. 时间依赖：断言与墙钟时间相关。

### 8.2 对流水线通过率的冲击（手算）

设每个测试独立地以概率 $p$ 间歇失败，共 $n=200$ 个测试，则一次全绿的概率为 $(1-p)^{200}$。

| 单测试 flaky 率 $p$ | 全绿概率 |
|---|---|
| 2% | 1.76% |
| 0.5% | 36.7% |
| 0.1% | 81.9% |

（Python 复核。）即使每个测试"只有 0.5% 概率抽风"，200 个测试放在一起也有 63% 的概率红灯。**红灯多了，大家就不信红灯。**

### 8.3 三类对策

1. **根治确定性**：用事件（等待话题出现、服务可用）代替 `sleep`；固定随机种子；隔离 DDS 域；为超时设宽松值（失败才耗时，成功不耗时）；
2. **隔离与标记**：把已知 flaky 测试单独标记，放入"不阻塞合并"的作业，同时登记问题限期修复；
3. **有限重试**：失败后自动重试至多 2 次。若失败彼此独立，$p=2\%$ 时三次都失败的概率 $0.02^3=8\times10^{-6}$，200 个测试全绿概率升到 99.84%（Python 复核）。**但重试会掩盖真问题**，且真实的 flaky 往往不独立（如机器整体过载），所以重试只是止痛，不能代替根治，并且必须记录"重试了多少次"以便统计。

> **面试要点**："flaky 测试的危害不在于它本身，而在于它训练团队忽略失败。"

---

## 9. CI 流水线：从提交到报告

### 9.1 流水线的阶段

```text
提交/合并请求
   │
   ▼
1. 环境：拉取 ROS2 Humble 容器镜像，安装系统依赖 (rosdep)
   ▼
2. 构建：colcon build（可开启告警当错误）
   ▼
3. 测试：colcon test（单元 + lint + launch_testing）
   ▼
4. 汇总：colcon test-result --verbose，上传 XML/日志为构建产物
   ▼
5. （可选）回放回归：对固定 bag 跑指标，与基线比较
   ▼
6. 报告：在合并请求里给出通过/失败与指标变化
```

### 9.2 一份流水线配置的结构示意（GitHub Actions 风格，**未在 CI 上运行**）

```yaml
# .github/workflows/ci.yml（教学示例，非项目配置）
name: ci
on: [push, pull_request]
jobs:
  build-test:
    runs-on: ubuntu-22.04
    container:
      image: ros:humble          # 镜像名与标签使用前需核对，未核对
    steps:
      - uses: actions/checkout@v4
        with:
          path: src/robot_pkgs
      - name: Install dependencies
        run: |
          apt-get update
          rosdep update
          rosdep install --from-paths src --ignore-src -y
      - name: Build
        run: |
          . /opt/ros/humble/setup.sh
          colcon build --event-handlers console_cohesion+
      - name: Test
        run: |
          . /opt/ros/humble/setup.sh
          . install/setup.sh
          colcon test --event-handlers console_direct+
          colcon test-result --verbose
```

说明：

- Humble 对应 Ubuntu 22.04，runner 与容器镜像版本要匹配；
- 第三方 Action（如 ROS 社区提供的 setup/ci 动作）的名称与用法**未核对**，上面只用通用的 checkout 与 shell 步骤；
- 构建缓存（`ccache`、依赖缓存）能显著缩短时间，但要注意缓存失效策略；
- 机器人 CI 的现实约束：无 GPU、无真实硬件、无 DDS 多机网络，所以硬件相关测试只能放在实验室的自建 runner 或手动触发的作业里。

### 9.3 一个时间预算的算术

假设构建 6 分钟，测试 4 分钟，总计 10 分钟。若团队每天合并 30 次，串行排队时每天占用 5 小时的 runner 时间；缓存把构建降到 2 分钟，则变成 3 小时。**流水线的耗时直接决定团队愿不愿意频繁提交**——超过 10–15 分钟，人就会攒大批改动再提交，而大批改动正是 bug 难定位的原因。

### 9.4 质量门槛的设置

合并前的门槛建议逐步加严：先要求"构建通过 + 单元测试通过"，稳定后加入 lint，再加入集成测试，最后加入回放回归的指标门槛。一次性全上，往往因为 flaky 与历史问题堆积，直接让人放弃。

---

## 10. 在七个项目里怎么用（含诚实边界）

| 项目 | 项目里做过什么 | 与本章的关系 | 性质 |
|---|---|---|---|
| [P1](../../01_项目课程/P1_AIRS球形机器人平衡控制/08_ROS_CAN安全执行闭环.md) | 执行链的安全测试方法与回归清单（见该章 §12）：初始化、限幅、饱和、跌倒检测、恢复 | "不变量"思想与失效安全断言；清单可以对应自动化用例 | 项目做过的是**手工 + 台架的回归清单**，没有说用 CI |
| [P2](../../01_项目课程/P2_未来动力双臂遥操作与差速底盘/11_调试与故障案例.md) | 逐层联调与故障复现 | 回放与单元测试可以把"逐层联调"前移 | 项目做过联调；自动化回放是延伸 |
| [P5](../../01_项目课程/P5_车式移动机器人自主导航/11_联调与故障案例.md) | 导航栈联调；时间、坐标系、运动方向导致的漂移与跳变 | 回放回归最适合测定位与代价地图类的输出质量；开环限制在 §2.1 | 项目做过联调；rosbag 回放作为方法是通用做法，**CV 未写** |
| [P6](../../01_项目课程/P6_深开鸿M-Robots平台软件/10_测试与验证方法.md) | 仿真 → 单机 → 多台台架的分层验证；安全链的拒绝用例 | 测试金字塔的实践形态；策略检查的拒绝路径应有逐点断言 | 项目设计口径 |
| P3、P4、P7 | 嵌入式与边缘端的联调 | 嵌入式单元测试可以在主机上跑纯逻辑核心（PID、状态机）；P4 的状态机适合做序列测试 | 方法类比，不要说成"我们做了" |
| CI 流水线本身 | CV 与项目文档未涉及 | — | **延伸，非项目所做** |

> **面试要点**：被问"你们怎么测"时，诚实又有分量的回答是——"我们分层验证：先把控制逻辑当纯函数在主机上测边界和故障输入，再仿真，再单台台架，最后整机；所有安全相关的拒绝路径都有对应用例。自动化流水线我了解它的搭法，但在我的项目里主要是手动加清单的方式。"

---

## 11. 常见错误与故障现象

| 现象 | 原因 | 处理 |
|---|---|---|
| `launch_testing` 偶尔收不到消息 | 发布太早，发现未完成 | 循环发布 + 超时，不用固定 sleep |
| 并行测试互相干扰 | 共享 DDS 域 | 不同 `ROS_DOMAIN_ID` 或仅本机通信 |
| 回放时 TF 报"外推" | 没开 `use_sim_time`、没发 `/clock`，或缺 `/tf_static` | 回放加 `--clock`，节点设 `use_sim_time`，录制包含静态变换 |
| 回放结果每次略不同 | 随机数未定种子、多线程顺序 | 固定种子；比统计指标 |
| 倍速回放指标变差 | 算法来不及处理 | 回归用 1 倍速 |
| 单元测试比较浮点失败 | 用了精确相等 | `EXPECT_NEAR` / `pytest.approx` |
| CI 本地能过线上挂 | 环境差异（依赖版本、时区、CPU 核数） | 本地用同一容器复现 |
| 红灯没人管 | flaky 太多 | 治理 flaky，见 §8 |
| 测试通过但节点其实崩溃 | 没检查退出码 | `post_shutdown_test` + `assertExitCodes` |
| 基线被悄悄改 | 基线与代码同提交且无评审 | 基线改动单独评审并说明原因 |

---

## 12. 面试怎么问、怎么答

**Q1. How do you test robot software when real hardware tests are slow and expensive?**

> I use layers. At the bottom, I pull the control logic out of the ROS callbacks into pure functions and unit-test them, including boundaries and invalid inputs like NaN. Next, I use integration tests that launch the real nodes and check topics, QoS and parameters. Then I replay recorded sensor data and compare metrics against a baseline. Finally I use a test bench and the real robot for what automation cannot cover, like friction and real latency. Each layer should catch what the cheaper layer below it cannot.

中文：分层。最底层把控制逻辑从回调里抽成纯函数做单元测试，含边界与 NaN；其次用集成测试拉起真实节点验证话题、QoS、参数；再用录制的传感器数据回放，把指标与基线比较；最后用台架和实车覆盖自动化测不到的东西（摩擦、真实延迟）。每一层要抓下面一层抓不到的问题。

*Follow-up: What can't a bag replay test?*

> Closed-loop behavior. A replay is open loop: the recorded data does not react to the new controller's output. So replay is good for estimation, perception, and planning output quality, but not for closed-loop stability. For that, I need simulation or the real robot.

中文：闭环行为。回放是开环的，录好的数据不会响应新控制器的输出；适合测估计、感知、规划输出质量，不适合测闭环稳定性，后者靠仿真或实车。

**Q2. What is a flaky test and why does it matter?**

> A flaky test sometimes fails without any code change. In ROS, the typical causes are discovery delays, fixed sleeps, shared DDS domains and unseeded randomness. It matters because the failure rates multiply: with 200 tests and a 0.5 percent flaky rate each, about 63 percent of runs would be red. Then people learn to ignore red, and the real failures get missed. I would replace sleeps with waiting on events, isolate domains, fix seeds, and use retries only as a temporary measure while counting them.

中文：flaky 测试指代码没改却间歇失败。ROS 里常见原因是发现延迟、固定 sleep、共享 DDS 域、随机数未定种子。危害在于失败率会累积：200 个测试各 0.5% 抽风，约 63% 的运行是红灯，团队学会忽略红灯，真失败被漏掉。对策：用等待事件替代 sleep、隔离域、固定种子，重试只作临时手段并统计次数。

**Q3. How do you design a regression test using a rosbag?**

> I pick a fixed set of recorded scenarios, define metrics such as position RMSE and maximum jump, and store a baseline from an accepted version. I measure the natural run-to-run variation to choose a tolerance, so the test does not fire on noise. Changing the baseline requires a reviewed commit. For safety rules, like never exceeding the speed limit, I assert on every output sample instead of using a statistical tolerance.

中文：选一组固定场景，定义指标（位置 RMSE、最大跳变），以已接受版本为基线；先测自然波动再定容差，避免噪声触发；改基线必须经评审的提交；对安全规则（如绝不超速）对每个输出逐点断言，而非用统计容差。

*Follow-up: Why use_sim_time when replaying?*

> Because the message timestamps come from the recording. If the node uses wall time, it compares recorded stamps to the current clock, and the transform lookups fail with extrapolation errors. The player publishes a clock topic, and the node must follow it.

中文：消息时间戳来自录制时刻；节点若用墙钟，会拿旧时间戳与当前时间比，TF 查询报外推错误。回放器发布 `/clock`，节点必须跟随它。

**Q4. What would you put in a CI pipeline for a ROS 2 workspace?**

> Build with colcon in a container that matches the target distribution, which for me is Humble on Ubuntu 22.04. Then run unit tests, linters and launch tests, and collect the test-result report as an artifact. I would add a replay regression job once the earlier stages are stable. I would keep hardware-dependent tests off the default pipeline and run them on a lab machine or manually. Keeping the whole thing under about ten minutes matters, because slow pipelines make people batch their changes.

中文：在与目标发行版一致的容器（Humble / Ubuntu 22.04）里用 colcon 构建，运行单元测试、lint、launch 测试并把结果报告作为产物；前面稳定后再加回放回归；依赖硬件的测试不放默认流水线，在实验室机器或手动运行；整体控制在约十分钟，否则大家会攒大批改动再提交。

**Q5. Did you use automated tests or CI in your projects?**

> In my projects the verification was layered but mostly manual and checklist-based: simulation first, then a single robot on a bench, then the full system, with explicit negative cases for the safety chain. I did not build a CI pipeline for them. I understand how to do it, and the first thing I would automate is the pure-logic parts and the safety rejection cases, because those are cheap and catch the most dangerous mistakes.

中文：我的项目里验证是分层的，但主要是手动加清单：先仿真，再单台台架，再整机，安全链有明确的负面用例；没有为它们搭 CI。我了解怎么搭，首先自动化纯逻辑部分和安全拒绝用例，因为便宜且能抓最危险的错误。

---

## 13. 本章要点速记

1. 测试金字塔：单元 → 节点集成 → 回放回归 → 台架 → 实车；越往下越快越便宜，越往上越真实。
2. 把逻辑抽成与 ROS 无关的纯函数，外壳只做类型转换，单元测试才能秒级运行。
3. NaN、超时、越界等非法输入的默认行为应当是失效安全（减速到停），并写成断言。
4. ament 里：`BUILD_TESTING` 包住测试，`ament_add_gtest`、`ament_lint_auto`、`add_launch_test`；`colcon test` 后用 `colcon test-result --verbose` 汇总。
5. `launch_testing` 的三件套：`ReadyToTest`、运行期 `TestCase`、`post_shutdown_test` 检查退出码。
6. 等事件、不要 sleep；发现延迟会让"发太早"的消息丢失。
7. 回放：`ros2 bag play --clock` + 节点 `use_sim_time`；回放是开环，测不了闭环稳定性；注意 `/tf_static`。
8. 回归 = 数据集 + 指标 + 基线 + 容差；容差取自然波动的数倍；基线更改要评审；安全不变量逐点断言。
9. flaky 率会累积：0.5% × 200 个测试 → 63% 红灯；根治确定性，重试只是止痛。
10. CI 要快（约十分钟内）、环境与目标发行版一致，硬件相关测试不进默认流水线；项目里没做 CI 就如实说。

## 14. 自测题

1. 回放回归为什么不能验证控制器的闭环稳定性？应当用什么验证？
2. 写出 `limit_cmd(0.8, 0.0, 2.0, 0.5, 0.1)` 的返回值并说明理由。（答案：0.05；速度上限不触发，变化率限制为 $0.5\times0.1=0.05$。）
3. 100 个独立测试，每个 flaky 率 1%，一次全绿的概率是多少？（答案：$0.99^{100}\approx36.6\%$。）
4. 为什么 `launch_testing` 里不能"启动后 sleep 1 秒、发一条消息、断言收到"？应当怎么改？
5. 回放时 TF 报"extrapolation into the future"，列出三个最可能的原因。（要点：节点没设 `use_sim_time`；回放没开 `--clock`；缺少 `/tf_static` 或录制时间戳与时钟源不一致。）

**参考要点**：1. 录好的数据不响应新输出；用仿真/实车；4. 发现延迟，应循环发布 + 超时等待。

## 15. 相关链接

- 上一章：[08 ROS2 Humble 实战：launch、参数、生命周期、组件](./08_ROS2_Humble实战_launch_参数_生命周期_组件.md)
- 下一章：[10 仿真平台：Gazebo、Isaac、MuJoCo 对比](./10_仿真平台_Gazebo_Isaac_MuJoCo对比.md)
- 相关：[04 ROS2 Humble 核心概念](./04_ROS2_Humble核心概念与ROS1对比.md)、[05 DDS 与分布式通信](./05_DDS与分布式通信.md)、[11 CMake 与 ament 构建](./11_CMake_ament_构建系统与依赖管理.md)
- 系统工程视角：[测试策略：仿真在环、硬件在环、回归](../13_系统工程与安全/04_测试策略_仿真在环_硬件在环_回归.md)
- 项目：[P6 测试与验证方法](../../01_项目课程/P6_深开鸿M-Robots平台软件/10_测试与验证方法.md)、[P1 ROS-CAN 安全执行闭环](../../01_项目课程/P1_AIRS球形机器人平衡控制/08_ROS_CAN安全执行闭环.md)
- [模块 README](./README.md)、[基础知识总览](../00_总览与学习路线.md)
