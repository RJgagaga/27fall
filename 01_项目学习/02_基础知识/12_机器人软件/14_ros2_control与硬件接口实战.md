# 14 ros2_control 与硬件接口实战

> **一句话定位**：在 ROS2 里，"让关节按指令转起来"有两种写法：一种是自己写一个节点，订阅命令、驱动 CAN、发布状态（本仓库项目的做法）；另一种是用 `ros2_control` 框架，把"硬件怎么读写"和"控制器怎么算"拆成两个可替换的插件，由一个固定节拍的控制器管理器（controller manager）统一调度。本章讲清第二种的结构：硬件接口、命令/状态接口、控制器、`<ros2_control>` 描述、`read → update → write` 循环、控制器链与实时性，并用 CAN 总线的负载算术和一个可运行的 Python 小模型把"一个周期里到底发生了什么"算清楚。

**本章学完你能：**

- 画出 `ros2_control` 的架构：硬件组件（system/actuator/sensor）、资源管理器、控制器管理器、控制器，以及它们之间"借用接口"的关系；
- 说清命令接口（command interface）与状态接口（state interface）各是什么，为什么控制器之间不能同时写同一个命令接口；
- 看懂 URDF 里的 `<ros2_control>` 标签与控制器 YAML，并写出差速底盘的配置骨架；
- 写出一个硬件接口的生命周期方法（`on_init`、`export_*_interfaces`、`on_activate`、`read`、`write`）的职责清单，并说明哪些事不能在 `read/write` 里做；
- 手算：差速底盘的轮速与车体速度互换、里程计积分；CAN 帧占用与控制周期的关系；
- 理解 `read → update → write` 固有的一拍延迟，并用小模型复现；
- 解释控制器链（chained controllers）和实时性的基本要求，指出 Python/动态分配/阻塞 I/O 的风险；
- 把你项目里的"自写硬件节点"映射到 `ros2_control` 的概念上，并诚实说明哪部分是延伸。

**前置知识：**

- [ROS2 Humble 核心概念与 ROS1 对比](./04_ROS2_Humble核心概念与ROS1对比.md)（节点、话题、定时器、执行器）
- [ROS2 Humble 实战：launch、参数、生命周期、组件](./08_ROS2_Humble实战_launch_参数_生命周期_组件.md)（生命周期节点、参数文件、launch）
- [URDF、TF、坐标系实践](./07_URDF_TF_坐标系实践.md)（关节、连杆）
- [CMake、ament 构建系统与依赖管理](./11_CMake_ament_构建系统与依赖管理.md)（库导出、插件）
- [Docker 与部署](./12_Docker与部署_交叉编译_systemd.md)（SocketCAN 与权限）

> **关于代码的说明**：本机没有 ROS2，所以 `ros2_control` 的 C++ 接口、URDF 标签、YAML 键名、命令行子命令**未在 ROS 环境运行核对**，只作结构示意；方法签名在不同发行版之间有变化（例如 `read/write` 是否带时间参数），凡不确定处标"未核对"，写代码前要以所用 Humble 版本的文档与头文件为准。**能离线运行的部分已用 Python 复核**：一个"读—算—写"循环的教学小模型（不是真实 API）、差速运动学与里程计积分、CAN 总线占用算术。所有数值是**教学示例（非项目实测）**。

**目录**

1. 为什么需要 ros2_control
2. 架构：四个角色
3. 命令接口与状态接口
4. URDF 里的 `<ros2_control>` 与控制器配置
5. 硬件接口的生命周期与职责
6. 一个周期里发生什么：read → update → write
7. 手算：差速底盘、里程计与 CAN 预算
8. 控制器链与常用控制器
9. 实时性与 CAN 桥接的注意事项
10. 命令行与排错
11. 在七个项目里怎么用（含诚实边界）
12. 常见错误与故障现象
13. 面试怎么问、怎么答
14. 本章要点速记
15. 自测题
16. 相关链接

---

## 1. 为什么需要 ros2_control

### 1.1 自写硬件节点的问题

最直接的写法（本仓库 [P2 的 ROS2-CAN 执行链](../../01_项目课程/P2_未来动力双臂遥操作与差速底盘/10_ROS2_CAN执行链.md)就是这样）：一个节点里同时做五件事——订阅目标、检查年龄、限幅、发送 CAN、接收回包并发布状态。它工作得很好，但随着机器人变多，问题出现了：

1. **换机器人就重写**：换一种电机或总线，整个节点都要改，因为"算控制"和"读写硬件"缠在一起；
2. **控制器无法复用**：关节轨迹控制、差速底盘控制、力矩控制，每个都要自己对接硬件；
3. **时序不统一**：每个节点自己定时，读写时机不一致，延迟和抖动各不相同；
4. **仿真与实机切换麻烦**：仿真里得另写一套"假硬件"。

### 1.2 ros2_control 的思路

把关系拆开：

```text
控制器（算什么）              硬件接口（怎么读写）
 diff_drive_controller          自写的 CAN 电机驱动 / 仿真器插件 / 假硬件
 joint_trajectory_controller             ▲
        │  通过"接口"借用                 │ 暴露"接口"
        ▼                                │
   ┌────────────────────────────────────────┐
   │  controller manager：固定节拍调用         │
   │  read()  →  controllers update()  →  write() │
   └────────────────────────────────────────┘
```

控制器只认**接口**（"左轮的速度命令""左轮的位置状态"），不认具体硬件；硬件接口负责把接口值与真实世界互换。这样同一个差速控制器，既能接真实的 CAN 电机，也能接仿真器或假硬件。**这和 [P6](../../01_项目课程/P6_深开鸿M-Robots平台软件/03_统一能力模型与设备抽象.md) 的"统一能力模型"是同一个抽象思想**：上层不管下面是谁。

> **面试要点**：用一句话概括——"ros2_control 是控制器与硬件之间的标准化接口加一个固定节拍的调度器。"

---

## 2. 架构：四个角色

| 角色 | 职责 | 说明 |
|---|---|---|
| 硬件组件（hardware component） | 与真实硬件通信，暴露命令/状态接口 | 三类：`System`（多个关节，一个整体，如整机底盘或多电机总线）、`Actuator`（单个执行器）、`Sensor`（只读传感器）。以插件形式（pluginlib）加载 |
| 资源管理器（resource manager） | 管理所有硬件组件及其接口，处理接口的"借用/归还" | 保证同一命令接口同一时刻只被一个控制器占用 |
| 控制器（controller） | 在每个周期里读状态接口、写命令接口，实现控制律 | 如关节轨迹控制器、差速控制器、广播器（只读状态并发布话题） |
| 控制器管理器（controller manager） | 一个 ROS 节点，加载/配置/激活控制器，并以固定频率执行 `read → update → write` | 提供服务与命令行接口 |

**数据流方向**：

- 状态：硬件 → `read()` 写入状态接口 → 控制器读取；
- 命令：控制器写入命令接口 → `write()` 读取并发给硬件。

**两个关键约束**：

1. 接口是**共享内存里的数值**（不是话题）。控制器读写接口没有序列化和网络开销，因此可以在高频率、低延迟下运行；
2. 控制器管理器只有**一个**循环，它按顺序调用，所以时序是确定的：先全部读，再全部算，最后全部写。

---

## 3. 命令接口与状态接口

### 3.1 定义

接口用"关节名/接口类型"命名。常见类型：`position`、`velocity`、`effort`（力矩）；状态接口还可以有传感器值（如 IMU 的各个分量）。

| 概念 | 含义 | 例 |
|---|---|---|
| 命令接口 | 控制器**写**、硬件**读**的量 | `left_wheel_joint/velocity`（目标角速度，rad/s） |
| 状态接口 | 硬件**写**、控制器**读**的量 | `left_wheel_joint/position`（累计转角，rad）、`left_wheel_joint/velocity`（实测角速度） |

### 3.2 独占与借用

一个控制器在**激活**时"借用"它需要的命令接口。同一个命令接口不能同时被两个控制器借用——否则两个控制律互相覆盖，输出由谁最后写决定，行为不可预测。因此切换控制模式（比如从"位置控制器"换到"力矩控制器"）要用"切换控制器"操作：先停旧的、再启新的，管理器检查接口冲突。状态接口则可被多个控制器同时读取。

### 3.3 为什么用接口而不是话题

话题通信要序列化、经过 DDS、在执行器里排队，延迟和抖动无法保证；接口值是同进程内的内存读写，时序完全由管理器的循环决定。需要对外发布的时候，让**广播器控制器**（如 `joint_state_broadcaster`）读取状态接口，再发布成话题。**高频闭环留在接口内，低频的监控、指令走话题**，这是 ros2_control 的基本分工。

---

## 4. URDF 里的 `<ros2_control>` 与控制器配置

### 4.1 硬件描述（结构示意）

```xml
<ros2_control name="chassis" type="system">
  <hardware>
    <plugin>demo_hw/CanDiffDriveHardware</plugin>
    <param name="can_interface">can0</param>
    <param name="left_motor_id">1</param>
    <param name="right_motor_id">2</param>
    <param name="max_wheel_speed">20.0</param>
  </hardware>

  <joint name="left_wheel_joint">
    <command_interface name="velocity"/>
    <state_interface name="velocity"/>
    <state_interface name="position"/>
  </joint>
  <joint name="right_wheel_joint">
    <command_interface name="velocity"/>
    <state_interface name="velocity"/>
    <state_interface name="position"/>
  </joint>
</ros2_control>
```

要点：

- `<plugin>` 指向硬件接口的插件类名；`<param>` 是传给 `on_init` 的字符串参数；
- 每个关节声明它提供哪些命令和状态接口，**声明多少，硬件实现就必须导出多少**，不一致会在加载时报错；
- 这些信息放在 URDF 里，意味着"机器人的描述"和"它的硬件接口"在同一个文件，参见 [第 07 章](./07_URDF_TF_坐标系实践.md)。

### 4.2 控制器配置（YAML，结构示意）

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100                     # Hz，管理器循环频率
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
    diff_drive_controller:
      type: diff_drive_controller/DiffDriveController

diff_drive_controller:
  ros__parameters:
    left_wheel_names: ["left_wheel_joint"]
    right_wheel_names: ["right_wheel_joint"]
    wheel_separation: 0.40               # m，示例
    wheel_radius: 0.075                  # m，示例
    cmd_vel_timeout: 0.5                 # s，超时则停车
    publish_rate: 50.0
    base_frame_id: base_link
    odom_frame_id: odom
    enable_odom_tf: true
```

（键名以 Humble 版本的 `diff_drive_controller` 文档为准，**未核对**。）

### 4.3 加载与激活

```bash
# 启动控制器管理器（通常由 launch 带起来，并读取 URDF 的 robot_description 与上面的 YAML）
ros2 run controller_manager spawner joint_state_broadcaster
ros2 run controller_manager spawner diff_drive_controller
ros2 control list_controllers
ros2 control list_hardware_interfaces
```

`spawner` 做的是"加载 → 配置 → 激活"。命令行子命令的具体名字与输出格式在 Humble 上**未核对**。

---

## 5. 硬件接口的生命周期与职责

硬件接口是一个带生命周期的组件，其状态与 [第 08 章](./08_ROS2_Humble实战_launch_参数_生命周期_组件.md)的生命周期节点一致（Unconfigured → Inactive → Active）。

### 5.1 方法清单与"该做什么/不该做什么"

| 方法 | 何时调用 | 该做 | 不该做 |
|---|---|---|---|
| `on_init(info)` | 加载时 | 解析 `info` 里的参数（CAN 接口名、电机 ID、限幅）；校验配置；分配内存 | 打开设备、发送总线数据 |
| `export_state_interfaces()` / `export_command_interfaces()` | 加载后 | 声明并返回每个接口（指向成员变量） | 在这里读硬件 |
| `on_configure()` | 配置时 | 打开 SocketCAN 套接字、设为非阻塞、设置过滤器 | 使能电机 |
| `on_activate()` | 激活时 | **安全地使能**：先读一次状态，把命令接口初始化为当前状态（避免跳变），再使能电机 | 假设上次状态仍有效 |
| `read()` | 每周期 | **非阻塞**读取最新反馈，写入状态接口 | 长时间阻塞、动态内存分配、打印大量日志 |
| `write()` | 每周期 | 读取命令接口，限幅，打包，**非阻塞发送** | 在命令无效（NaN）时照发 |
| `on_deactivate()` | 停用时 | **安全收尾**：把速度命令置零、失能电机 | 直接断开连接而不失能 |
| `on_cleanup()` / `on_shutdown()` | 清理/关闭 | 关闭套接字、释放资源 | — |

### 5.2 一个结构示意（C++，签名随版本变化，**未核对**）

```cpp
// 教学示例（非项目源码）：结构示意
class CanDiffDriveHardware : public hardware_interface::SystemInterface {
public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;
private:
  double pos_[2]{0, 0}, vel_[2]{0, 0}, cmd_vel_[2]{0, 0};   // 接口指向这些成员
  int can_fd_{-1};
};
```

接口对象保存的是**成员变量的指针**，所以成员变量的生命周期必须覆盖整个运行期，且不能在加载后改变地址（例如不要用会重新分配的容器去承载它们）。

### 5.3 与 P2 自写硬件节点的映射

| P2 硬件节点里的步骤 | ros2_control 里的对应 |
|---|---|
| 订阅 `/dual_xarm/joints` 得到目标 | 控制器写命令接口（或上层控制器写） |
| 长度检查、`safe_clamp` 钳位 | `write()` 内的限幅与有效性检查 |
| `send_mit` 打包并发 CAN | `write()` |
| `recv_feedback` 收回包 | `read()` |
| `publish_joint_states` | `joint_state_broadcaster`（读状态接口并发布） |
| 目标年龄分档（冻结/失能） | 放在控制器侧（如差速控制器的 `cmd_vel_timeout`），或硬件侧的看门狗 |
| 使能序列 / 收尾序列 | `on_activate()` / `on_deactivate()` |

这个映射说明：**你在项目里设计的那些安全逻辑，一个都没有变少，只是被放到了框架规定的位置**。

---

## 6. 一个周期里发生什么：read → update → write

### 6.1 固定的三步

控制器管理器以 `update_rate`（如 100 Hz，周期 10 ms）的节拍重复：

1. `read()`：所有硬件组件把最新反馈写入状态接口；
2. `update()`：所有活动的控制器按依赖顺序读状态、算控制律、写命令接口；
3. `write()`：所有硬件组件把命令接口里的值发给硬件。

### 6.2 固有的一拍延迟

在第 $k$ 个周期，控制器用的是 **`read()` 在第 $k$ 周期开头读到的反馈**，算出的命令在**第 $k$ 周期末尾的 `write()`** 发出，硬件在其后才响应。所以从"传感器真实状态"到"命令生效"，至少有 **一个周期加通信与执行延迟**。周期越长，这项延迟越大，控制带宽越受限（参见控制理论中的采样与延迟对相位裕度的影响）。

### 6.3 可运行的教学小模型（非真实 API，Python 复核）

下面用 Python 把这个循环写成最小形式：一个"硬件接口"（`read` 复制电机转速到状态句柄、`write` 把命令句柄传给电机并推进一阶电机模型），一个带前馈的速度 PI "控制器"。

```python
# 教学小模型：模仿 ros2_control 的 read -> update -> write 顺序（不是真实 API）
class Handle:
    def __init__(self, name): self.name = name; self.value = 0.0

class MockMotor:                      # 一阶响应，时间常数 0.05 s
    def __init__(self): self.w = 0.0; self.cmd = 0.0

class HardwareInterface:
    def __init__(self):
        self.motor = MockMotor()
        self.state = {'velocity': Handle('wheel/velocity'),
                      'position': Handle('wheel/position')}
        self.command = {'velocity': Handle('wheel/velocity_cmd')}
        self.dt = 0.01
    def read(self):                   # 非阻塞：把最新反馈拷进状态句柄
        self.state['velocity'].value = self.motor.w
        self.state['position'].value += self.motor.w * self.dt
    def write(self):                  # 取命令句柄的值，发给“电机”并推进模型
        self.motor.cmd = self.command['velocity'].value
        tau = 0.05
        self.motor.w += (self.motor.cmd - self.motor.w) * self.dt / tau

class VelocityController:
    def __init__(self, hw, kp=0.8, ki=6.0, umax=20.0):
        self.hw, self.kp, self.ki, self.i, self.ref, self.umax = hw, kp, ki, 0.0, 0.0, umax
    def update(self, dt):
        e = self.ref - self.hw.state['velocity'].value
        self.i += e * dt
        u = self.kp * e + self.ki * self.i + self.ref      # PI + 前馈
        self.hw.command['velocity'].value = max(-self.umax, min(self.umax, u))

hw = HardwareInterface(); ctrl = VelocityController(hw); ctrl.ref = 5.0
for k in range(300):
    hw.read(); ctrl.update(hw.dt); hw.write()
```

**Python 复核输出**（目标 5.0 rad/s，周期 10 ms，记录各时刻速度）：

| 时间（s） | 0.01 | 0.10 | 0.20 | 0.50 | 1.00 | 3.00 |
|---|---|---|---|---|---|---|
| 实测速度（rad/s） | 0.000 | 5.288 | 5.295 | 5.100 | 5.016 | 5.000 |

读法：

- 第 1 个周期末记录到的速度是 0.000——因为 `read()` 先于 `write()`，第一次读到的还是初始状态，**这就是"一拍延迟"的直观体现**；
- 约 0.1 s 时有约 6% 的超调（5.288），1 s 内收敛到 0.3% 以内，3 s 稳态为 5.000，PI 的积分作用消除了稳态误差；
- 这个小模型只用来说明**调用顺序和接口值传递**，电机模型和增益是教学示例。

> **注意**：真实 `ros2_control` 中，状态接口和命令接口是被控制器和硬件共享的指针式内存；上面用 `Handle` 对象模仿这一点。

---

## 7. 手算：差速底盘、里程计与 CAN 预算

### 7.1 差速运动学（与 [P2 差速底盘](../../01_项目课程/P2_未来动力双臂遥操作与差速底盘/08_差速底盘运动学_轮速PID与里程计.md)同一类关系）

符号：车体线速度 $v$（m/s，向前为正），角速度 $\omega$（rad/s，逆时针为正），左右轮角速度 $\omega_l,\omega_r$（rad/s），轮半径 $r$（m），轮距 $L$（m）。

左轮线速度 $v_l=v-\omega L/2$，右轮线速度 $v_r=v+\omega L/2$，轮角速度 $\omega_{l,r}=v_{l,r}/r$。反过来：

$$
v=\frac{r(\omega_l+\omega_r)}{2},\qquad \omega=\frac{r(\omega_r-\omega_l)}{L}.
$$

**手算（示例，非项目实测）**：$r=0.075$ m，$L=0.40$ m，命令 $v=0.5$ m/s，$\omega=0.5$ rad/s。

- $\omega L/2=0.5\times0.40/2=0.10$；
- 左轮线速度 $0.5-0.10=0.40$ m/s → $\omega_l=0.40/0.075=5.333$ rad/s；
- 右轮线速度 $0.5+0.10=0.60$ m/s → $\omega_r=0.60/0.075=8.000$ rad/s；
- 反算：$v=0.075\times(5.333+8.000)/2=0.5$，$\omega=0.075\times(8.000-5.333)/0.40=0.5$。

（Python 复核：5.3333、8.0、反算 0.5 与 0.5。）这正是 `diff_drive_controller` 在 `update()` 里做的事：把 `cmd_vel` 变成两个轮速命令写入命令接口，并用轮子的状态接口反算里程计。

### 7.2 里程计积分

控制周期 20 ms（50 Hz），保持 $v=0.5$、$\omega=0.5$ 运行 1 s，使用中点法更新：$x\mathrel{+}=v\Delta t\cos(\theta+\Delta\theta/2)$，$y\mathrel{+}=v\Delta t\sin(\theta+\Delta\theta/2)$，$\theta\mathrel{+}=\omega\Delta t$。

结果（Python 复核）：$x=0.4794$ m，$y=0.1224$ m，$\theta=0.5$ rad。解析解（半径 $R=v/\omega=1$ m 的圆弧）：$x=R\sin(\omega t)=0.4794$，$y=R(1-\cos\omega t)=0.1224$，完全一致。如果用欧拉法（用旧角度）则会有系统性偏差；周期越长、转弯越急偏差越大——这也是里程计漂移的来源之一（参见 [P5 的车辆运动学与里程计](../../01_项目课程/P5_车式移动机器人自主导航/02_车辆运动学与里程计.md)）。

### 7.3 CAN 总线占用与周期

经典 CAN 一帧 8 字节标准帧约 111 位，1 Mbit/s 下约 $111\ \mu$s（与 [P2 §7.1](../../01_项目课程/P2_未来动力双臂遥操作与差速底盘/10_ROS2_CAN执行链.md) 一致，未计填充位的最坏情形）。每个周期对每个电机发 1 帧命令、收 1 帧反馈。

| 电机数 | 每周期帧数 | 每周期总线时间 | 100 Hz 时负载 | 200 Hz 时负载 |
|---|---|---|---|---|
| 6 | 12 | 1.332 ms | 13.3% | 26.6% |
| 8 | 16 | 1.776 ms | 17.8% | 35.5% |

（Python 复核。）经验是总线负载应保持在 50% 以内。**对 ros2_control 的意义**：`update_rate` 受总线容量限制；并且 `read()` 与 `write()` 都要在周期内完成——若周期是 5 ms（200 Hz），一次读写的总线时间 1.78 ms 已占 35%，再加控制计算 1.0 ms、其他开销，整个周期预算（示例）：读 0.2 ms + 控制 1.0 ms + 写 0.3 ms $=1.5$ ms，远小于 5 ms，剩余用于抖动裕量。

---

## 8. 控制器链与常用控制器

### 8.1 常用控制器

| 控制器 | 作用 | 典型接口 |
|---|---|---|
| `joint_state_broadcaster` | 读所有关节状态接口，发布 `/joint_states` | 只读状态 |
| `joint_trajectory_controller` | 跟踪关节轨迹（路径点 + 时间），做插值 | 写位置/速度命令接口 |
| `diff_drive_controller` | 把 `cmd_vel` 变轮速，并发布里程计与 TF | 写轮速命令，读轮状态 |
| `forward_command_controller` | 把话题上的数组直接转发到命令接口 | 调试、直通 |
| 力矩/阻抗控制器 | 重力补偿、阻抗等（常为自定义或第三方，具体包**未核对**） | 写力矩命令，读位置/速度状态 |

### 8.2 控制器链（chained controllers）

有时需要"控制器套控制器"：外环输出作为内环的参考。例如底盘速度环（内）+ 位置环（外）、关节力矩环（内）+ 阻抗环（外）。控制器链允许一个控制器**导出自己的"参考接口"**，另一个控制器像写硬件命令接口一样写它，而不是通过话题。好处：同周期、同内存、无序列化；顺序由管理器按依赖排好（先外环后内环，或按链的定义）。该功能在 Humble 的可用程度与使用方式**未核对**，用之前请查所用版本的文档。

与 [P1 的级联 PID](../../01_项目课程/P1_AIRS球形机器人平衡控制/06_级联PID控制.md)、[P3 云台串级 PID](../../01_项目课程/P3_RoboMaster英雄机器人嵌入式控制/06_云台串级PID与视觉跟踪.md) 的关系：那些项目在 MCU/节点内部自己实现内外环（外环限幅后给内环），思想与控制器链同构，差别在于是否由框架提供"内外环之间的接口与调度"。

### 8.3 控制模式切换

从"位置控制"切到"力矩控制"这类切换，要保证：旧控制器停用时输出变为安全值；新控制器激活时其内部状态（积分项等）被正确初始化，命令接口初值等于当前状态，避免突变。这与 P1 的"状态初始化五步"同一原则。

---

## 9. 实时性与 CAN 桥接的注意事项

### 9.1 实时循环的约束

`read/update/write` 在一个固定周期的循环里，若某次执行超过周期，就是**周期超时（overrun）**，下一周期被延迟或丢失，控制出现抖动。因此循环内的代码应满足：

1. **不阻塞**：不要在 `read()` 里等待串口/总线数据直到超时；应读"当前已有的最新数据"，没有就沿用上一次并计数；
2. **不做不确定耗时的操作**：动态内存分配（`new`、容器扩容）、文件 I/O、打印日志、获取可能被占用的互斥锁；
3. **有界耗时**：每个周期最坏耗时要有上限，并小于周期的一定比例（如 70%，见 [第 13 章的诊断阈值](./13_日志_诊断_性能剖析_perf_tracing.md)）；
4. **与外部通信用无锁结构**：话题回调和实时循环之间用实时安全的缓冲（`realtime_tools` 里的 `RealtimeBuffer`、`RealtimePublisher` 一类，具体接口**未核对**），不要让回调线程持锁阻塞实时线程；
5. **线程优先级与锁内存**：给控制器管理器线程设置实时优先级、锁定内存页（见 [第 12 章](./12_Docker与部署_交叉编译_systemd.md) §4.4 的权限）。

**Python 不适合写实时硬件接口**：垃圾回收、全局解释器锁和动态分配造成不确定延迟。ros2_control 的硬件接口与控制器以 C++ 为主，是有原因的。Python 适合上层（任务、参数、监控）。

### 9.2 CAN 桥接的几种写法

| 方式 | 做法 | 优缺点 |
|---|---|---|
| 硬件接口直接操作 SocketCAN | `read()` 非阻塞 `recv`，`write()` 非阻塞 `send` | 延迟最低，需自己处理帧解析和错误；实时性取决于实现 |
| 独立线程收发 + 硬件接口读写缓存 | 后台线程阻塞收帧写入环形缓冲，`read()` 只取最新 | 解耦阻塞 I/O，实时循环更干净；需要线程同步 |
| MCU 做实时环，Linux 只发上层命令 | ROS2 侧发"速度/力矩目标"，MCU 跑高频闭环 | 硬实时最稳；本仓库 P3、P7、P2 底盘固件就是这类分层 |

**MCU + Linux 分层**是机器人工程的常见折中：电机电流环、速度环在 MCU 上做到千赫兹级，Linux 侧以百赫兹级发目标。此时 ros2_control 的 `write()` 实际是"发送目标命令帧"，`read()` 是"收取状态帧"，频率远低于电机内环，Linux 侧抖动对内环无影响。

### 9.3 安全相关的硬件侧逻辑

无论放在哪里，下列逻辑**不能依赖上层按时到达**：命令超时失效（看门狗）、限幅、使能互锁、跌倒/过流保护。它们应在硬件接口或 MCU 固件中，作为与上层控制器**无关的**最后防线。与 [P1 三层限幅](../../01_项目课程/P1_AIRS球形机器人平衡控制/08_ROS_CAN安全执行闭环.md)、[P2 限幅分层与分层看门狗](../../01_项目课程/P2_未来动力双臂遥操作与差速底盘/10_ROS2_CAN执行链.md)的设计一致。

---

## 10. 命令行与排错

| 目的 | 命令（Humble，子命令名**未核对**） |
|---|---|
| 查看控制器及状态 | `ros2 control list_controllers` |
| 查看硬件接口及占用 | `ros2 control list_hardware_interfaces` |
| 切换控制器 | `ros2 control switch_controllers --deactivate A --activate B` |
| 查看控制器话题 | `ros2 topic list`，`ros2 topic echo /joint_states` |
| 查看参数 | `ros2 param list /diff_drive_controller` |

典型故障与排查：

| 现象 | 可能原因 | 处理 |
|---|---|---|
| 控制器加载失败，提示找不到类型 | 控制器包未安装或 YAML 的 `type` 写错 | 检查包和类型字符串 |
| 控制器配置失败，提示接口不存在 | URDF 里没声明该关节的该接口类型 | 对照 `<ros2_control>` 与控制器要求 |
| 激活失败，提示接口被占用 | 另一个控制器已借用该命令接口 | 先停用冲突控制器 |
| 机器人不动，但命令话题有数据 | 控制器没激活；`cmd_vel` 话题名/是否带时间戳不符；超时 | `list_controllers`；`ros2 topic info` |
| 周期报警 / 控制抖动 | `read/write` 阻塞或耗时过长 | 用 §9 的原则排查；测量单次耗时 |
| 激活时电机突然冲一下 | `on_activate` 没把命令初始化为当前状态 | 先读状态，再设命令，再使能 |
| 仿真正常，实机异常 | 实机延迟、饱和、反馈噪声 | 第 10 章仿真注意事项 |
| 里程计漂移 | 轮半径/轮距参数不准；打滑；积分方式 | 校准参数；融合 IMU |

---

## 11. 在七个项目里怎么用（含诚实边界）

| 项目 | 项目里的实际做法 | 与 ros2_control 的关系 | 性质 |
|---|---|---|---|
| [P2 ROS2-CAN 执行链](../../01_项目课程/P2_未来动力双臂遥操作与差速底盘/10_ROS2_CAN执行链.md) | CV 写 "validated on real hardware through the ROS2-CAN execution chain"；项目文档中是自写硬件节点：订阅目标、检查年龄、限幅、发 MIT 帧、收回包、发布状态 | 同样的职责，只是没有使用 ros2_control 框架；§5.3 给出一一映射 | 项目做过**自写执行链**；ros2_control 本身是**延伸，非项目所做** |
| [P2 差速底盘](../../01_项目课程/P2_未来动力双臂遥操作与差速底盘/08_差速底盘运动学_轮速PID与里程计.md) | CV 写 "mapped body velocity commands to wheel targets via differential-drive kinematics, closed the PID loop using encoder feedback and integrated wheel motion into odometry" | 正是 `diff_drive_controller` 的功能；§7 的手算与之同类 | 项目做过（自己实现）；框架控制器为延伸 |
| [P1 安全执行闭环](../../01_项目课程/P1_AIRS球形机器人平衡控制/08_ROS_CAN安全执行闭环.md) | CV 写 "integrated ROS state processing with CAN motor execution and implemented state initialisation, output limiting and fall-recovery safeguards" | 初始化、限幅、安全收尾对应 `on_activate`/`write`/`on_deactivate`；存量代码为 ROS1 风格，ros2_control 是 ROS2 的框架（ROS1 有 `ros_control`，概念对应，**未核对**细节） | 项目做过；面试按概念讲 |
| [P6 统一能力模型](../../01_项目课程/P6_深开鸿M-Robots平台软件/03_统一能力模型与设备抽象.md)、[ROS/ROS2 命令映射](../../01_项目课程/P6_深开鸿M-Robots平台软件/04_ROS_ROS2命令映射与跨硬件复用.md) | CV 写 "built a unified capability model for sensors, actuators and controllers ... mapped ROS/ROS2 motion commands to low-level drivers" | 与"控制器—接口—硬件"的抽象思想一致；P6 不是 ros2_control | 项目做过**同类抽象**；平台自有实现 |
| [P3](../../01_项目课程/P3_RoboMaster英雄机器人嵌入式控制/04_CAN总线与电机反馈.md)、[P7](../../01_项目课程/P7_可变腿长轮腿机器人/10_STM32H7_FreeRTOS_FDCAN软件结构.md) | MCU 上实时内环 | §9.2 "MCU 做实时环，Linux 发目标"的分层 | 项目做过嵌入式内环 |
| P4、P5 | 无 ros2_control 使用说明 | — | — |

> **面试要点**：被问"你用过 ros2_control 吗"，诚实的回答——"I have not used the ros2_control framework itself. In my projects I wrote the hardware node by hand with the same responsibilities: reading feedback, applying limits and validity checks, sending commands, and safe enable and shutdown sequences. I understand how those map onto the hardware interface lifecycle and the controller manager loop, and I would use the framework when I need to reuse controllers across different hardware."

---

## 12. 常见错误与故障现象

| 错误 | 后果 | 改正 |
|---|---|---|
| 在 `read()` 里阻塞等待总线数据 | 周期超时、控制抖动 | 非阻塞读取最新值，必要时独立收发线程 |
| 在 `write()` 里不检查 NaN/越界 | 电机收到异常命令 | 限幅 + 有效性检查，失效时输出安全值 |
| `on_activate` 不初始化命令接口 | 激活瞬间电机冲向旧命令 | 命令接口初值设为当前状态 |
| `on_deactivate` 不失能 | 控制器停了，电机仍按最后命令转 | 置零命令并失能 |
| 两个控制器写同一命令接口 | 输出互相覆盖 | 用控制器切换，保证独占 |
| 以话题方式做高频内环 | 延迟抖动大 | 内环走接口，话题只做监控/低频命令 |
| URDF 与硬件导出的接口不一致 | 加载失败 | 对照 `<ros2_control>` 声明 |
| 用 Python 写实时硬件接口 | 不确定延迟 | C++ 实现；Python 放上层 |
| 忽略 `read→update→write` 的一拍延迟 | 高增益下振荡 | 把一拍延迟计入设计，缩短周期 |
| 周期超过总线容量 | 丢帧、反馈陈旧 | 按 §7.3 估算负载，保持在 50% 以内 |

---

## 13. 面试怎么问、怎么答

**Q1. What problem does ros2_control solve and how is it structured?**

> It separates what to compute from how to talk to the hardware. Controllers, like a differential-drive controller, only read state interfaces and write command interfaces. Hardware components, which are plugins, expose those interfaces and handle the real communication, such as CAN. A controller manager runs a fixed-rate loop of read, update and write. Because interfaces are shared memory and not topics, the loop has deterministic timing and no serialization. And I can reuse the same controller on different hardware, or on a simulator, by swapping the hardware plugin.

中文：它把"算什么"和"怎么与硬件通信"分开。控制器（如差速控制器）只读状态接口、写命令接口；硬件组件（插件）暴露这些接口并处理真实通信（如 CAN）；控制器管理器以固定频率循环 read、update、write。接口是共享内存而非话题，所以时序确定、没有序列化；换硬件插件就能把同一控制器用到不同硬件或仿真器上。

*Follow-up: Why are command interfaces exclusive?*

> If two controllers wrote to the same command at once, the last writer would win and the behavior would depend on the call order. So the resource manager lets only one active controller claim a command interface; switching modes means stopping one controller and starting another.

中文：若两个控制器同时写同一命令，最后写入者胜出，行为取决于调用顺序。所以资源管理器只允许一个激活控制器占用某命令接口；切换模式即停一个、启另一个。

**Q2. What should and should not be done inside read() and write()?**

> They run inside a fixed-period real-time loop, so they must be bounded in time. I read the latest available feedback without blocking, convert it, and write it to the state interfaces. In write I check validity and limits, and send without blocking. I avoid dynamic allocation, file I/O, heavy logging, and waiting on locks. The one-time work, like opening the socket, belongs to configure, and enabling the motors belongs to activate, where I first initialize the command to the current state to avoid a jump.

中文：它们在固定周期实时循环里，耗时必须有界。read 里非阻塞读最新反馈、换算、写状态接口；write 里检查有效性与限幅，非阻塞发送。避免动态分配、文件 I/O、大量日志、等待锁。一次性工作（如打开套接字）放 configure，使能电机放 activate，并先把命令初始化为当前状态避免跳变。

**Q3. What is the inherent delay in the read-update-write loop?**

> The controller uses feedback read at the beginning of the cycle and its command is sent in the write at the end of the cycle, so there is at least one cycle between the measured state and the effect of the command, plus communication and actuator delays. For a 100 Hz loop that is 10 milliseconds just from the sampling structure. I include it in the design: shorter period, enough phase margin, and delay compensation if needed. In a small simulation, the first sample after the first cycle still shows zero velocity because the read happens before the first write.

中文：控制器用的是周期开头读到的反馈，命令在周期末尾的 write 发出，所以测量状态与命令生效之间至少有一个周期，再加通信与执行器延迟。100 Hz 时仅采样结构就有 10 ms。设计时计入：缩短周期、保证相位裕度、必要时延迟补偿。小模型里第一周期后速度仍为零，就是因为 read 在第一次 write 之前。

**Q4. How would you map your own hand-written hardware node onto ros2_control?**

> The subscription to targets becomes a controller writing command interfaces. The clamping and validity checks go into write. Sending and receiving CAN frames go into write and read. The joint-state publisher becomes a broadcaster controller. The enable sequence goes into activate and the shutdown sequence into deactivate. The target-age watchdog goes into the controller, such as a command timeout, and I keep a second independent watchdog on the hardware side or the microcontroller, so safety does not depend on the upper layer arriving on time.

中文：订阅目标变成控制器写命令接口；钳位与有效性检查进 write；CAN 收发进 write 与 read；joint state 发布变成广播器控制器；使能序列进 activate，收尾序列进 deactivate；目标年龄看门狗放到控制器（如命令超时），同时在硬件侧或单片机保留第二个独立看门狗，使安全不依赖上层按时到达。

**Q5. Would you run a motor current loop through ros2_control on Linux?**

> No. A current loop needs kilohertz rates with tight jitter, which a general Linux process cannot guarantee unless it is a carefully tuned real-time setup. I would run the current and velocity loops on the microcontroller and let ros2_control on Linux send velocity or torque targets at around a hundred hertz. That is also the structure of my embedded projects, where the inner loops run on the STM32 and the upper layer only provides targets.

中文：不会。电流环需要千赫兹速率和很小的抖动，普通 Linux 进程除非精心调优为实时配置，否则无法保证。我会把电流环和速度环放在单片机上，Linux 侧的 ros2_control 以约百赫兹发送速度或力矩目标；这也是我嵌入式项目的结构：内环在 STM32，上层只给目标。

---

## 14. 本章要点速记

1. ros2_control = 控制器与硬件之间的标准接口 + 固定节拍调度器；换硬件不改控制器。
2. 四个角色：硬件组件（System/Actuator/Sensor 插件）、资源管理器、控制器、控制器管理器。
3. 命令接口：控制器写、硬件读；状态接口：硬件写、控制器读；命令接口独占，状态接口可共享。
4. 接口是共享内存，不是话题：高频内环走接口，监控与低频命令走话题；广播器把状态发布成话题。
5. 硬件生命周期：`on_init` 解析配置，`on_configure` 开设备，`on_activate` 先读状态再初始化命令再使能，`on_deactivate` 置零并失能。
6. `read/write` 必须非阻塞、有界耗时、不做动态分配和日志洪水；Python 不适合写实时接口。
7. 一个周期：read → update → write，固有至少一拍延迟（小模型第一周期速度仍为 0）。
8. 差速：$v=r(\omega_l+\omega_r)/2$，$\omega=r(\omega_r-\omega_l)/L$；示例 $v=0.5,\omega=0.5$ → $\omega_l=5.333,\omega_r=8.0$ rad/s；中点法里程计与圆弧解析解一致。
9. CAN 预算：111 µs/帧，8 电机一周期 1.776 ms，100 Hz 负载 17.8%，保持在 50% 以内。
10. 硬实时内环放 MCU，Linux 发目标；看门狗、限幅、使能互锁在硬件侧独立保留。你项目里是自写节点，ros2_control 是延伸，映射关系见 §5.3。

## 15. 自测题

1. 列出 `diff_drive_controller` 需要的命令接口与状态接口。（答案：命令：左右轮 velocity；状态：左右轮 position 与 velocity（用于里程计）。）
2. $r=0.1$ m，$L=0.5$ m，命令 $v=0.3$ m/s，$\omega=-0.4$ rad/s，求 $\omega_l,\omega_r$。（答案：$\omega L/2=-0.1$；$v_l=0.4$，$v_r=0.2$；$\omega_l=4.0$，$\omega_r=2.0$ rad/s。）
3. 10 个电机，经典 CAN 1 Mbit/s，每电机每周期 1 命令帧 + 1 反馈帧，周期 5 ms，总线负载是多少？是否合适？（答案：20 帧 × 111 µs = 2.22 ms；2.22/5 = 44.4%；偏高，接近 50% 的经验上限，需留意最坏填充位。）
4. 为什么 `on_activate` 要先读状态再设命令再使能？
5. 控制器 A 与控制器 B 都想写 `joint1/position` 命令接口，应如何处理？

**参考要点**：4. 避免命令接口初值与实际状态不一致导致激活瞬间的跳变；5. 同一时刻只激活一个，用控制器切换，或用控制器链让 A 的输出成为 B 的参考。

## 16. 相关链接

- 上一章：[13 日志、诊断、性能剖析、tracing](./13_日志_诊断_性能剖析_perf_tracing.md)
- 相关：[08 ROS2 Humble 实战：launch、参数、生命周期、组件](./08_ROS2_Humble实战_launch_参数_生命周期_组件.md)、[07 URDF、TF、坐标系实践](./07_URDF_TF_坐标系实践.md)、[12 Docker 与部署](./12_Docker与部署_交叉编译_systemd.md)
- 控制与运动学：[控制理论模块](../05_控制理论/README.md)、[运动学与坐标变换模块](../03_运动学与坐标变换/README.md)
- 项目：[P2 ROS2-CAN 执行链](../../01_项目课程/P2_未来动力双臂遥操作与差速底盘/10_ROS2_CAN执行链.md)、[P2 差速底盘](../../01_项目课程/P2_未来动力双臂遥操作与差速底盘/08_差速底盘运动学_轮速PID与里程计.md)、[P1 安全执行闭环](../../01_项目课程/P1_AIRS球形机器人平衡控制/08_ROS_CAN安全执行闭环.md)
- [模块 README](./README.md)、[基础知识总览](../00_总览与学习路线.md)
