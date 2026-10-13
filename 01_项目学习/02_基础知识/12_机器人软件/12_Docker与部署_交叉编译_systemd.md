# 12 Docker 与部署：容器化环境、交叉编译与 systemd 自启动

> **一句话定位**：代码在你的笔记本上跑通，只完成了一半；另一半是让它**在机器人上、断电重启后、没有人敲命令时**依然可靠地起来、出错时能被拉起、升级时能回滚。本章讲三件事：用 Docker 把"依赖环境"固定成可复制的镜像（镜像分层、多阶段构建、设备映射、实时权限），用交叉编译把产物编到 ARM 板上（回扣 [第 11 章](./11_CMake_ament_构建系统与依赖管理.md)），用 systemd 把"开机—启动顺序—崩溃重启—看门狗"管起来，并手算镜像缓存、重启限流和启动时间预算。

**本章学完你能：**

- 解释容器与虚拟机的本质区别（共享内核），说清镜像、层、容器、卷（volume）各是什么；
- 写出一份适合机器人工作区的多阶段 Dockerfile，并通过**调整指令顺序**让缓存命中，手算重建时间的差别；
- 说明让容器访问串口、USB 相机、CAN 总线、GPU 时需要哪些映射/权限，以及为什么不要轻易用 `--privileged`；
- 解释 DDS 在容器里为什么常需要 `--network host`，以及它的代价（具体行为标"未核对"）；
- 说明交叉编译、ARM 原生构建、QEMU 模拟构建三种路线的取舍并估算耗时；
- 写出 systemd 的服务单元文件：依赖顺序（`After`/`Requires`/`Wants`）、`Restart`、重启限流（`StartLimit*`）、资源限制、看门狗，并手算崩溃循环何时被限流；
- 设计"升级—验证—回滚"流程，并说明机器人为什么不应该在运行中无保护地热更新；
- 说清自己的项目里哪些部署做法是真实经历，哪些是延伸知识。

**前置知识：**

- [Linux 与 Shell、Git](./01_Linux与Shell_Git.md)（进程、权限、环境变量、`systemctl` 基础）
- [CMake、ament 构建系统与依赖管理](./11_CMake_ament_构建系统与依赖管理.md)（工具链文件、sysroot、`rosdep`）
- [DDS 与分布式通信](./05_DDS与分布式通信.md)（发现、多播）
- [ROS2 Humble 实战：launch、参数、生命周期、组件](./08_ROS2_Humble实战_launch_参数_生命周期_组件.md)（生命周期节点、启动顺序）

> **关于代码的说明**：本机（Windows）没有安装 Docker、systemd 和 ROS2，所以 Dockerfile、Compose、systemd 单元、udev 规则**未在目标环境运行**，只作结构示意，且 Dockerfile 的基础镜像名与标签、各选项在具体版本上的行为凡不确定处标"未核对"。能离线运行的部分——镜像层缓存重建时间的算术、systemd 重启限流的模拟、单元文件的 INI 语法解析、启动时间预算——已用本地 Python 复核（标"Python 复核"）。所有数值是**教学示例（非项目实测）**。

**目录**

1. 为什么需要"部署"这一层
2. Docker 的最小模型：镜像、层、容器
3. 机器人工作区的 Dockerfile：多阶段与缓存（手算）
4. 容器访问硬件：串口、USB、CAN、GPU、实时权限
5. 网络与 DDS：host 网络、域隔离、共享内存
6. 交叉编译、原生构建与 QEMU 的取舍
7. systemd：把机器人变成"开机自启的服务"
8. 手算：重启限流、看门狗与启动时间预算
9. 升级、验证与回滚
10. 在七个项目里怎么用（含诚实边界）
11. 常见错误与故障现象
12. 面试怎么问、怎么答
13. 本章要点速记
14. 自测题
15. 相关链接

---

## 1. 为什么需要"部署"这一层

### 1.1 要解决的问题

机器人软件交付时典型的痛点：

1. **"在我机器上能跑"**：开发机装了某个版本的库，板子上没有或版本不同；
2. **环境漂移**：板子上被人 `apt upgrade` 了一次，行为变了；
3. **手工启动**：每次上电要有人 SSH 进去开三个终端，现场演示常因此出错；
4. **崩溃无人管**：节点崩溃了没人发现，机器人定在原地或更糟；
5. **升级即风险**：升级后出问题，不知道怎么回到上一个可用版本。

对应的工具：Docker 固定环境，交叉编译/构建流水线产出对应架构的产物，systemd 管理进程生命周期，版本化的升级流程提供回滚。

### 1.2 整体图景

```text
开发机/CI                                  机器人（ARM 板）
  源码 ──► 构建镜像(Dockerfile) ──► 镜像仓库 ──► docker pull ──► 容器(运行时镜像)
                                                      ▲
                                    systemd 服务 ─────┘  开机启动、崩溃重启、看门狗
                                    udev/can-setup 服务 → 设备名稳定、总线配置
```

> **面试要点**：把部署理解为"可复现（环境）+ 可自启（进程管理）+ 可恢复（回滚与看门狗）"三件事，回答就不会只剩"用 Docker"一句话。

---

## 2. Docker 的最小模型：镜像、层、容器

### 2.1 容器不是虚拟机

虚拟机虚拟出一整套硬件，里面跑独立内核；**容器共享宿主机内核**，只是用命名空间（namespace）隔离进程、网络、文件系统视图，用 cgroup 限制资源。结果是：

- 启动是"启动一个进程"的量级（毫秒到秒），不是"启动一个操作系统"；
- 开销小，几乎没有性能损失（对 CPU 密集任务）；
- 但**内核是共享的**：容器内不能换内核，也不能用宿主机内核没有的功能（比如宿主机没有 `vcan` 模块，容器里也没有）；
- 隔离不如虚拟机强。

### 2.2 概念

| 概念 | 含义 |
|---|---|
| 镜像（image） | 只读的文件系统快照 + 启动元数据，由若干**层**叠加而成 |
| 层（layer） | 每条会修改文件系统的 Dockerfile 指令产生一层，层可缓存、可共享 |
| 容器（container） | 镜像加上一层可写层，再加运行时配置，运行起来的进程 |
| 卷（volume）/ 绑定挂载 | 把宿主机目录挂进容器，用于持久化数据、配置、日志 |
| 仓库（registry） | 存放镜像，按"名称:标签"拉取 |

**镜像层的要点**：层是**不可变**的。你在后面的层里删除前面层里的文件，镜像体积不会变小（文件还在下面的层里，只是被"盖住"）。所以"安装—清理"必须在**同一条 `RUN`** 里完成。

### 2.3 数据不要写进容器可写层

容器删除后可写层就没了。需要保留的东西——日志、录制的 bag、地图、标定文件、参数——必须放在卷或绑定挂载里。机器人上常见做法是把 `/data` 挂载进容器，镜像本身保持只读。

---

## 3. 机器人工作区的 Dockerfile：多阶段与缓存（手算）

### 3.1 缓存规则

Docker 构建按指令顺序逐层执行。对每条指令，如果**该指令文本和它所依赖的输入（如 `COPY` 的文件内容）与上次相同，且它之前所有层都命中缓存**，就直接复用；一旦某层失效，**其后所有层都要重建**。因此：

> **把变化最少的放前面，变化最多的放后面。**

### 3.2 一个多阶段 Dockerfile（结构示意，**未构建运行**）

```dockerfile
# ---------- 阶段一：构建 ----------
FROM ros:humble-ros-base AS build        # 基础镜像名与标签未核对
SHELL ["/bin/bash", "-c"]

# 1) 系统依赖（变化少）
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential ccache python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
# 2) 只先拷贝 package.xml，用来安装依赖（依赖清单变化不频繁）
COPY src/demo_chassis/package.xml src/demo_chassis/package.xml
COPY src/chassis_msgs/package.xml src/chassis_msgs/package.xml
RUN apt-get update && rosdep install --from-paths src --ignore-src -y \
    && rm -rf /var/lib/apt/lists/*

# 3) 最后才拷贝源码（变化最频繁）并构建
COPY src src
RUN . /opt/ros/humble/setup.bash && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# ---------- 阶段二：运行 ----------
FROM ros:humble-ros-core AS runtime
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*     # 具体运行期依赖按项目而定，包名未核对
COPY --from=build /ws/install /opt/robot/install
COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "demo_chassis", "chassis.launch.py"]
```

`entrypoint.sh` 的典型内容：

```bash
#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
source /opt/robot/install/setup.bash
exec "$@"        # exec 使主进程成为 PID 1，能正确接收 SIGTERM
```

要点：

1. **多阶段**：构建阶段含编译器、头文件、缓存，体积大；运行阶段只拷贝 `install/` 和运行期依赖，镜像小、攻击面小、拉取快；
2. **先拷 `package.xml`、再 `rosdep`、最后拷源码**：改源码时不会触发重新安装依赖；
3. **`exec "$@"`**：让真正的程序成为 PID 1。否则 shell 是 PID 1，不转发信号，`docker stop` 等 10 秒后被强杀，节点来不及执行"收尾序列"（如电机失能、复位）。这对**执行器节点尤其关键**；
4. **`rm -rf /var/lib/apt/lists/*` 与安装在同一条 `RUN`**，见 §2.2。

### 3.3 手算：缓存命中能省多少时间

假设各层大小与耗时（示例，非项目实测）：

| 层 | 内容 | 大小 | 构建耗时 |
|---|---|---|---|
| 0 | 基础镜像（拉取，不计构建） | 700 MB | — |
| 1 | 系统依赖 apt | 350 MB | 90 s |
| 2 | rosdep 依赖 | 200 MB | 60 s |
| 3 | COPY 源码 | 5 MB | 1 s |
| 4 | colcon build | 120 MB | 240 s |

按上面的**推荐顺序**，改动不同位置的重建时间（Python 复核）：

| 改动 | 失效的层 | 重建耗时 |
|---|---|---|
| 只改源码 | 3、4 | $1+240=241$ s |
| 改了 `package.xml`（加依赖） | 2、3、4 | $60+241=301$ s |
| 改了 apt 依赖清单 | 1、2、3、4 | $90+60+241=391$ s |

若写成"一开始就 `COPY . .`"，任何改动都使其后全部失效，**每次都是 391 s**。日常开发中 90% 的改动是改源码：推荐顺序下约 4 分钟，而且当依赖层命中缓存时，实际省下 150 s，也就是约 38%。每天构建 20 次，就是约 50 分钟（Python 复核：$150\times20/60=50$ 分钟）。

**镜像体积**：构建镜像约 $700+350+200+5+120+800(\text{工具与缓存})\approx2175$ MB；运行镜像只含基础与产物，约 $700+120=820$ MB（示例数字，Python 复核）。多阶段让部署镜像缩小约 62%，对带宽受限的现场升级很重要。

> **注意**：`latest` 标签不是版本。生产镜像必须用**固定版本标签**（甚至用摘要 digest 固定），否则"同一份 Dockerfile 今天和下个月构建出的东西不同"，复现性就没了。

---

## 4. 容器访问硬件：串口、USB、CAN、GPU、实时权限

默认容器看不到宿主机的设备，机器人需要按需放行，**原则是最小权限**。

### 4.1 设备映射

| 需求 | 做法 | 说明 |
|---|---|---|
| 串口（IMU、舵机、遥控接收机） | `--device=/dev/ttyUSB0` 或稳定别名 `--device=/dev/imu` | 用 udev 规则把设备固定成稳定名字，见 §4.2 |
| USB 相机（V4L2） | `--device=/dev/video0` | 同上 |
| GPU | 与 NVIDIA 容器运行时配合（如 `--gpus all`，配置方式未核对） | 需要宿主机驱动 |
| SocketCAN | 见 §4.3 | CAN 接口是**网络接口**，不是设备文件 |
| 全部设备 | `--privileged` | **不要**：几乎等于 root 访问宿主机，仅用于临时排错 |

### 4.2 udev 规则：给设备稳定的名字

USB 串口的 `/dev/ttyUSB0`、`/dev/ttyUSB1` 编号取决于插入顺序，重启后可能互换，导致"IMU 数据被当成电机反馈"。解决：按设备的厂商 ID、产品 ID、序列号建符号链接。

```text
# /etc/udev/rules.d/99-robot.rules（结构示意；ID 为示例，非真实设备）
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="IMU0001", SYMLINK+="imu", MODE="0660", GROUP="dialout"
```

之后用 `--device=/dev/imu`。设备权限用 `GROUP`/`MODE` 控制，而不是在容器里用 root。

### 4.3 SocketCAN 与容器

Linux 的 CAN 接口（`can0`）是一个**网络接口**，由宿主机内核管理。在容器里使用它：

1. 在**宿主机**上完成接口配置（比特率、启用）：

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

   这与 [P2 ROS2-CAN 执行链](../../01_项目课程/P2_未来动力双臂遥操作与差速底盘/10_ROS2_CAN执行链.md) §5 里的命令是同一类操作；

2. 让容器看见这个网络接口：最简单的是 `--network host`（容器与宿主机共享网络命名空间）；
3. 若要在容器内配置接口，需要 `--cap-add=NET_ADMIN`；只收发则不需要。

**更稳妥的分工**：接口配置放在宿主机的 systemd 服务里（§7.3 的 `can-setup.service`），容器只收发，权限最小。

### 4.4 实时权限

高频控制线程要用实时调度（`SCHED_FIFO`）、锁定内存（避免缺页中断），容器需要：

```bash
docker run --cap-add=SYS_NICE --ulimit rtprio=95 --ulimit memlock=-1 ...
```

同时宿主机需要是支持实时调度的内核配置（是否打了 RT 补丁见后续嵌入式 Linux 章节），并且 cgroup 对实时任务的 CPU 配额不能把实时线程限死（这类细节与具体 Docker/内核版本有关，**未核对**）。如果控制环对延迟非常敏感，最稳妥的做法是：**最底层的硬实时环放在 MCU 上**（如 STM32 的 FreeRTOS 任务），Linux 侧只发送带限幅的上层命令，不依赖容器里的实时性——这也是本仓库各项目的实际分层。

---

## 5. 网络与 DDS：host 网络、域隔离、共享内存

### 5.1 为什么容器里的 ROS2 常常"看不到"别的节点

DDS 的发现通常基于 UDP 多播（见 [第 05 章](./05_DDS与分布式通信.md)）。Docker 默认的桥接网络（bridge）会给容器一个独立 IP 和虚拟网络，多播不会自然跨出这个桥。表现为：容器里 `ros2 topic list` 看不到宿主机或另一块板上的话题。

处理方案：

| 方案 | 做法 | 代价 |
|---|---|---|
| host 网络 | `--network host` | 容器与宿主机共享网络栈，最简单；网络隔离丧失，端口冲突需自己管 |
| 配置 DDS 单播发现 | 用 DDS 配置文件列出对端地址 | 不依赖多播，但需维护地址列表（配置方式依 DDS 实现，**未核对**） |
| 同机多容器 | 都用 host 网络 | 共享内存传输还需共享 IPC（见 §5.3） |

### 5.2 域隔离

同一网段上有多台机器人时，用不同 `ROS_DOMAIN_ID` 隔离，否则互相能看到对方的话题，轻则数据串扰，重则一台的速度命令被另一台订阅。在 systemd 单元或容器环境变量里显式设置。开发机上则容易忘记设置，在实验室共用网络里造成"莫名其妙收到别人的数据"。

### 5.3 共享内存

部分 DDS 实现在同机进程之间自动用共享内存传输以降低拷贝。容器之间默认各有自己的 `/dev/shm` 与 IPC 命名空间，此时共享内存路径不通，通信会回退到 UDP 回环，或者出现"发现了对端却收不到数据"的现象。需要共享 IPC（如 `--ipc=host`）并保证两边使用同一种 DDS 实现与配置。具体行为取决于 DDS 实现和版本，**未核对**，遇到此类现象时先通过排除法确认：把两个节点放在同一容器里是否正常。

### 5.4 容器里的时间

容器共享宿主机的系统时钟。时间同步（NTP/PTP）要在**宿主机**上做；容器内不必也不应单独同步。多机器人时间一致性问题见 [P6 的时间戳与时钟同步](../../01_项目课程/P6_深开鸿M-Robots平台软件/20_工程化实践_频率周期与问题解决.md)。

---

## 6. 交叉编译、原生构建与 QEMU 的取舍

机器人常见的目标板是 ARM64（如 RK3588 等）。把 x86 开发机上写的代码变成能在 ARM 上跑的产物，有三条路线：

| 路线 | 原理 | 优点 | 缺点 |
|---|---|---|---|
| 交叉编译 | x86 上运行产出 ARM 指令的编译器，链接 sysroot 里的 ARM 库 | 速度快（开发机算力） | 配置 sysroot 与工具链复杂；依赖的所有库都要有 ARM 版；调试困难 |
| 板上原生构建 | 直接在 ARM 板上编译 | 最简单，环境与运行一致 | 板子算力、内存小，慢；占用部署设备 |
| QEMU 模拟 + 多架构 Docker 构建 | 在 x86 上用模拟器运行 ARM 容器来构建 | 复用现成 ARM 基础镜像，工具链简单 | 模拟开销大，速度可能慢一个数量级 |

**手算（示例，非项目实测）**：某工作区在 x86 上原生构建耗时 240 s。QEMU 模拟若慢约 10 倍，则需要 $240\times10=2400$ s $=40$ 分钟（Python 复核）；而在 ARM 板上原生构建若慢约 5 倍，约 20 分钟。交叉编译接近 240 s 加上配置成本。选择取决于构建频率：每天要构建几十次就值得投入交叉编译；一周一次的发布构建，QEMU 的"零配置"更划算。

另一个现实选择是：**用 ARM 构建机或云端 ARM 实例做 CI 的构建**，既不用模拟、也不用配置工具链。

工具链文件的写法见 [第 11 章 §9](./11_CMake_ament_构建系统与依赖管理.md)。多架构镜像可以用 `docker buildx` 一次产出 amd64 与 arm64 两个版本（用法与要求**未核对**）。

---

## 7. systemd：把机器人变成"开机自启的服务"

### 7.1 为什么不是"开机脚本里 `ros2 launch &`"

用 `rc.local` 或 crontab `@reboot` 启动有几个问题：没有依赖顺序（网络、CAN 还没起就启动了）、崩溃不会重启、日志散落、无法限制资源、无法查询状态。systemd 把每个进程变成一个**单元（unit）**，由它负责依赖、启动、重启、日志和资源。

### 7.2 单元文件的三个区块

```ini
[Unit]
Description=Robot bringup (chassis + localization)
After=network-online.target can-setup.service
Wants=network-online.target
Requires=can-setup.service
StartLimitIntervalSec=60
StartLimitBurst=5

[Service]
Type=simple
User=robot
Environment=ROS_DOMAIN_ID=7
ExecStart=/opt/robot/bin/bringup.sh
Restart=on-failure
RestartSec=2
LimitRTPRIO=95
LimitMEMLOCK=infinity

[Install]
WantedBy=multi-user.target
```

（本机用 Python `configparser` 解析确认语法为合法 INI，三个区块 `Unit/Service/Install` 均被识别；**systemd 本身没有在本机运行**。）

逐项说明：

| 指令 | 含义 | 常见误解 |
|---|---|---|
| `After=` | **仅仅是顺序**：本单元在列出的单元**之后**启动 | `After` 不会自动启动对方 |
| `Wants=` | 弱依赖：希望对方也启动，对方失败不影响我 | |
| `Requires=` | 强依赖：对方启动失败，我也不启动；对方停止，我也停止 | 需要与 `After` 配合才有顺序 |
| `Type=simple` | `ExecStart` 的进程就是主进程，启动即视为成功 | 若进程要做长时间初始化，systemd 无法知道何时"真的就绪"，可用 `Type=notify` 配合 sd_notify |
| `Restart=on-failure` | 非零退出码、被信号杀死、超时才重启，正常退出 0 不重启 | `always` 会把你手动 `stop` 之外的退出都重启 |
| `RestartSec=2` | 重启前等待 2 s | 太小会形成忙循环 |
| `StartLimitIntervalSec` / `StartLimitBurst` | 在时间窗口内最多允许多少次启动，超出就**不再重启**，进入 failed | 默认值各发行版不同，**未核对** |
| `LimitRTPRIO` / `LimitMEMLOCK` | 允许使用的实时优先级、可锁定内存 | 与 §4.4 对应 |
| `WantedBy=multi-user.target` | `systemctl enable` 后，开机进入该目标时启动 | |

**`After=network-online.target` 的一个坑**：`network.target` 只表示网络栈初始化，`network-online.target` 才表示"网络已经联通"。需要访问网络的服务应使用后者，并配合 `Wants=`。但纯本机 DDS 通信的服务，若用 `network-online.target`，在网线未接时可能一直等到超时，拖慢开机——要按实际需要选择。

### 7.3 CAN 接口配置服务

```ini
# /etc/systemd/system/can-setup.service（结构示意）
[Unit]
Description=Configure can0
After=sys-subsystem-net-devices-can0.device
BindsTo=sys-subsystem-net-devices-can0.device

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/sbin/ip link set can0 type can bitrate 1000000
ExecStart=/sbin/ip link set can0 up
ExecStop=/sbin/ip link set can0 down

[Install]
WantedBy=multi-user.target
```

要点：`Type=oneshot` + `RemainAfterExit=yes` 表示"执行完就算启动成功，并保持 active 状态"，这样 `Requires=can-setup.service` 的依赖者能判断它已就绪。比特率 1 Mbit/s 是教学示例，与 P2 的项目取值一致，实际要与总线上所有节点一致。

### 7.4 常用命令

```bash
sudo systemctl daemon-reload              # 修改单元文件后必须
sudo systemctl enable --now bringup       # 开机自启并立即启动
systemctl status bringup                  # 状态与最近日志
journalctl -u bringup -f                  # 跟踪日志
journalctl -u bringup -b --no-pager       # 本次开机以来的日志
sudo systemctl restart bringup
systemd-analyze blame                     # 各单元启动耗时
systemd-analyze critical-chain bringup    # 关键启动链
```

日志统一进 journald，可按单元、时间、优先级查询，避免"日志散落在 N 个终端"。注意 journald 默认可能把日志放内存，断电即失，要持久化需要配置（`Storage=persistent`，是否默认**未核对**）。

### 7.5 看门狗

两层含义，不要混淆：

1. **systemd 的软件看门狗**：设置 `WatchdogSec=N`，并要求进程定期调用 `sd_notify("WATCHDOG=1")`；若超过 N 秒没有"喂狗"，systemd 认为进程卡死，杀掉并按 `Restart=` 处理。建议喂狗间隔小于 $N/2$。它能发现"进程没崩但卡住了"（死循环、死锁）；
2. **硬件看门狗**：MCU 或 SoC 上的计数器，软件定期清零，否则整个芯片复位。它能发现"整个系统卡死"。

重点：**喂狗必须由真正代表"系统健康"的代码路径完成**（比如控制循环每次成功完成时喂），而不是单独起一个定时线程无脑喂——否则主逻辑死了，喂狗线程依然活着，看门狗形同虚设。这和 [P2 的分层看门狗](../../01_项目课程/P2_未来动力双臂遥操作与差速底盘/10_ROS2_CAN执行链.md)、[P1 的数据有效性检查](../../01_项目课程/P1_AIRS球形机器人平衡控制/08_ROS_CAN安全执行闭环.md)是同一设计原则。

---

## 8. 手算：重启限流、看门狗与启动时间预算

### 8.1 崩溃循环何时被限流

配置：`RestartSec=2`、`StartLimitIntervalSec=60`、`StartLimitBurst=5`。

**场景 A**：程序启动 1 s 后崩溃。每个周期 $1+2=3$ s。启动时刻为 $0,3,6,9,12,15,\dots$ s。在任意 60 s 窗口内允许最多 5 次启动，第 6 次（$t=15$ s）超限，服务进入 failed 并**不再自动重启**。（Python 模拟复核：第 6 次启动发生在 15 s。）

**场景 B**：程序能稳定运行 10 s 才崩溃。周期 $10+2=12$ s，60 s 窗口内启动次数 $\lceil60/12\rceil=5$ 次，**刚好不超过上限**（模拟 200 s 内共 17 次启动、未触发限流）。也就是说，限流只抓得住"崩得很快"的循环；"慢崩溃"会无限重启下去。

**场景 C**：把 `RestartSec` 改成 10，仍是 1 s 崩溃：周期 11 s，启动在 $0,11,22,33,44,55$，第 6 次发生在 55 s 时触发限流。

**对机器人的含义**：

1. 限流是为了避免"疯狂重启"耗尽 CPU、刷爆日志、反复触发硬件使能/失能；
2. **执行层不应被无限重启**：电机驱动节点反复崩溃重启，意味着执行器在反复被使能，这本身危险。更稳妥的做法是：限流后进入 failed，让上层**保持安全状态**（停车/失能）并向运维报警，等人工检查，而不是一直重试——对应 [P1 的"人工、受控、重新走初始化"](../../01_项目课程/P1_AIRS球形机器人平衡控制/08_ROS_CAN安全执行闭环.md)原则；
3. 对感知或可视化这类非安全节点，可使用较宽松的重启策略。

### 8.2 看门狗间隔

`WatchdogSec=4` 时，喂狗间隔应小于 $4/2=2$ s。如果控制循环 50 Hz（20 ms），每 100 次循环（2 s）喂一次就卡在边界上，偏危险；每 25 次循环（0.5 s）喂一次更稳。

### 8.3 开机到可用的时间预算

示例（非项目实测）：内核加载 8 s，systemd 基本服务 6 s，等待网络联通 5 s，容器守护进程与容器启动 4 s，ROS 节点初始化 6 s，合计 $8+6+5+4+6=29$ s。

- 若要求"上电后 30 s 内可接收指令"，余量只有 1 s，任何一环抖动就超标；
- 对策：去掉不必要的 `network-online` 等待（本机通信不需要），用 `systemd-analyze critical-chain` 找关键链，把非关键服务延后启动；
- 机器人的"就绪"不应只是"进程起来了"，而应是"传感器数据流稳定且自检通过"——这与 [第 08 章](./08_ROS2_Humble实战_launch_参数_生命周期_组件.md)生命周期节点的 `activate` 以及 [P6 的启动/就绪状态机](../../01_项目课程/P6_深开鸿M-Robots平台软件/15_生命周期_配置与可观测性.md)呼应。

---

## 9. 升级、验证与回滚

### 9.1 为什么不能直接覆盖

直接在机器人上 `git pull` + `colcon build` 的问题：中途断电或构建失败，机器人处于半成品状态；无法保证与测试过的版本一致；出问题没法快速回去。

### 9.2 基于镜像的升级流程

```text
1. CI 构建并测试 → 推送带版本号的镜像 robot:1.4.2（固定标签）
2. 机器人拉取新镜像（不影响正在运行的旧版本）
3. 切换：停止旧容器 → 启动新容器（或蓝绿：新旧并存，验证后切换）
4. 健康检查：自检通过、关键话题在预期频率、无错误日志（给定观察窗口，如 60 s）
5. 通过 → 保留旧镜像一段时间；失败 → 切回上一个版本标签
```

要点：

1. **版本化**：每个部署对应一个不可变的版本号，回滚就是"切换到上一个标签"；
2. **配置与镜像分离**：参数文件、标定数据放卷里，升级镜像不丢配置；但配置本身也要有版本（否则新版本读不了旧配置）；
3. **升级窗口**：不要在机器人运动中升级，先让其进入安全静止状态；
4. **迁移**：若新旧版本的消息接口不兼容，要么保证升级期间两端同时更新，要么保持接口向后兼容（新增字段不改旧字段），这对多机器人协作（如多台设备通过 DDS 通信的 P6 平台）尤为重要。

### 9.3 可观测性

部署不仅是"起来"，还要"看得见"：日志（journald）、关键状态话题、`ros2 doctor` 一类自检（输出细节**未核对**）、资源占用（CPU、内存、温度）。无法观测的部署，出了问题只能靠猜。

---

## 10. 在七个项目里怎么用（含诚实边界）

| 项目 | 项目里的实际情况 | 与本章的关系 | 性质 |
|---|---|---|---|
| [P2 ROS2-CAN 执行链](../../01_项目课程/P2_未来动力双臂遥操作与差速底盘/10_ROS2_CAN执行链.md) | SocketCAN 配置命令（1 Mbit/s）、使能与收尾序列、分层看门狗 | §4.3 的 CAN 配置命令是同一类操作；"收尾序列要能在退出时执行"对应 §3.2 的 `exec` 与信号处理 | 项目做过 CAN 配置与看门狗；**容器化、systemd 自启动在 CV 中没有写** |
| [P1 安全执行闭环](../../01_项目课程/P1_AIRS球形机器人平衡控制/08_ROS_CAN安全执行闭环.md) | 状态初始化、输出限幅、跌倒恢复由人工受控 | §8.1 "执行层不无限重启"的设计原则 | 原则一致；部署工具是延伸 |
| [P6](../../01_项目课程/P6_深开鸿M-Robots平台软件/15_生命周期_配置与可观测性.md) | 启动/就绪/关闭状态机、配置作为数据、健康模型；运行在 OpenHarmony 上 | §8.3、§9 的"就绪"与"可观测"思想；OpenHarmony 的部署与 systemd/Docker 不同体系 | 项目设计口径；平台不用 Docker/systemd（**未核对**） |
| [P4 平台基础](../../01_项目课程/P4_OpenHarmony智能园艺系统/02_平台基础_OpenHarmony_Hi3861_RK3588.md) | RK3588 板上部署边缘视觉，服务端运行 Agent | §6 的 ARM 目标板、交叉编译概念；服务端可容器化是通用做法 | 项目在板上部署；容器化是延伸 |
| [P5](../../01_项目课程/P5_车式移动机器人自主导航/20_工程化实践_频率周期与问题解决.md) | 实车 ROS2 Humble 导航栈 | §5 的 DDS 域与网络、§8.3 的启动时间预算 | 概念适用；容器化为延伸 |
| P3、P7 | STM32 固件，无操作系统层面的容器 | 固件升级对应"版本化 + 回滚"思想 | 方法类比 |
| Docker 镜像、systemd 单元、CI 构建 ARM 镜像 | CV 未写 | — | **延伸，非项目所做** |

> **面试要点**：诚实的回答——"On the robots I worked on, startup was handled by launch files and scripts, and I put the safety logic in the controller and the MCU side rather than relying on process restarts. I know how to containerize and manage the process with systemd, and I would apply two rules: pin the versions, and never let an actuator node restart in an endless loop."

---

## 11. 常见错误与故障现象

| 现象 | 原因 | 处理 |
|---|---|---|
| 镜像巨大 | 单阶段、apt 缓存没清、构建工具留在运行镜像 | 多阶段；清理与安装同一 `RUN` |
| 每次改一行代码都重装依赖 | `COPY . .` 放得太前 | 先拷 `package.xml`，后拷源码 |
| 容器里 `ros2 topic list` 为空 | bridge 网络下多播不通；域号不一致 | `--network host` 或单播发现；统一域号 |
| 同机两个容器发现了对方但收不到数据 | 共享内存传输被隔离 | 共享 IPC，或禁用共享内存传输（配置依实现，未核对） |
| `docker stop` 要等 10 秒 | PID 1 不是你的程序，不转发信号 | `exec "$@"` 或使用 init |
| 电机失能/收尾序列没执行 | 容器被强杀，没有走信号处理 | 处理 SIGTERM，设置合适的停止超时 |
| 串口编号重启后变了 | 依赖插入顺序 | udev 稳定别名 |
| 容器里读不到 CAN | 没有 host 网络，或接口没在宿主机配置 | 宿主机 systemd 配置 CAN，容器用 host 网络 |
| `Permission denied` 访问设备 | 用户不在设备组，权限映射缺失 | `GROUP`/`MODE`，容器 `--group-add` |
| 服务不自启动 | 没 `enable`；没 `daemon-reload` | `systemctl enable --now` |
| 服务启动顺序不对 | 只写了 `After` 没有 `Requires/Wants` | 同时声明依赖与顺序 |
| 服务一直重启 | `Restart=always` 无限流 | 设置 `StartLimit*`，执行层崩溃后进入安全状态 |
| 开机很慢 | 等待 `network-online`、服务串行 | `systemd-analyze critical-chain` 优化 |
| 升级后无法回到旧版 | 用 `latest` 标签，没保留旧镜像 | 固定版本标签，保留上一版 |
| 日志重启后丢失 | journald 非持久化 | 配置持久化存储 |

---

## 12. 面试怎么问、怎么答

**Q1. Why do you use multi-stage Docker builds and how do you order the instructions?**

> A multi-stage build separates the build environment, which has compilers and caches, from the runtime image, which only needs the installed artifacts and runtime dependencies. That makes the deployed image much smaller and faster to pull on a robot with limited bandwidth. For the instruction order, I put the least frequently changing steps first: system packages, then copying only the package manifests and installing dependencies, and finally copying the source and building. That way a source edit does not invalidate the dependency layers. In my example it cuts a rebuild from about 390 seconds to about 240.

中文：多阶段构建把含编译器和缓存的构建环境与只含产物和运行期依赖的运行镜像分开，部署镜像小、在带宽有限的机器人上拉取快。指令顺序按变化频率从低到高：系统包 → 只拷 package.xml 装依赖 → 最后拷源码并构建，这样改源码不会使依赖层失效。示例中重建从约 390 秒降到约 240 秒。

*Follow-up: Why `exec` in the entrypoint?*

> So that the real program becomes PID 1 and receives the termination signal. Otherwise the shell receives it, does not forward it, and the container is killed after the timeout, so a node never runs its shutdown sequence such as disabling the motors.

中文：让真正的程序成为 PID 1 接收终止信号；否则 shell 收到信号不转发，超时后容器被强杀，节点来不及执行收尾序列（如失能电机）。

**Q2. How would you give a container access to hardware like a CAN bus and a serial IMU?**

> For serial devices I would create stable names with udev rules and pass them with the device option, so that a reboot does not swap the port numbers. CAN is a network interface rather than a device file, so I configure the bitrate and bring it up on the host, preferably in a systemd oneshot service, and let the container share the host network for read and write. I would avoid privileged mode, and add only the capabilities I need, such as a real-time priority limit for the control thread.

中文：串口用 udev 规则建稳定名字并用 device 选项传入，避免重启后编号互换；CAN 是网络接口不是设备文件，在宿主机（最好用 systemd oneshot 服务）配置比特率并启用，容器共享宿主机网络收发；避免 privileged，只加所需能力，如控制线程的实时优先级限制。

**Q3. A systemd service that drives motors keeps crashing. What restart policy would you choose?**

> I would not restart it forever. I would set a restart delay and a start limit, for example at most five starts per minute, so a fast crash loop ends in a failed state. Repeated restarts mean the actuators are repeatedly re-enabled, which is dangerous. After the limit is reached, the system should hold a safe state, such as stopped and disabled, and raise an alarm for a human to check. For a non-safety node such as visualization, a looser policy is fine. Note that the limit only catches fast loops: a program that crashes every ten seconds would restart indefinitely, so I also log the restart count and alert on it.

中文：不会无限重启。设置重启延迟和启动限流（例如每分钟最多 5 次），使快速崩溃循环落入 failed 状态；反复重启意味着执行器被反复使能，危险。达到上限后系统应保持安全状态（停止、失能）并报警等人检查；非安全节点（如可视化）可以宽松。注意限流只抓得住快速循环，每 10 秒崩一次的程序会无限重启，所以还要记录重启次数并告警。

*Follow-up: What is the difference between After and Requires?*

> After only defines the start order and does not start the other unit. Requires defines a hard dependency: if the other unit fails to start, mine will not start either. I normally use them together.

中文：After 只定义启动顺序，不会拉起对方；Requires 是强依赖，对方启动失败我也不启动；通常两者一起用。

**Q4. How do you update software on a robot safely and roll back if it fails?**

> I deploy immutable, versioned images and keep configuration and calibration data in a volume separate from the image. The robot pulls the new image while the old one is still running, then switches only when it is in a safe stationary state. After the switch, a health check looks at self-test results and key topic rates for a window. If it fails, I switch back to the previous version tag, which is still on the device. I avoid the latest tag and avoid building in place on the robot.

中文：部署不可变、带版本号的镜像，配置与标定数据放在镜像之外的卷里；机器人在旧版本运行时拉取新镜像，仅在安全静止状态下切换；切换后在窗口内检查自检结果和关键话题频率；失败就切回设备上保留的上一版本标签。避免 latest 标签，避免在机器人上就地构建。

**Q5. Have you used Docker or systemd on your robots?**

> Not as a central part of my projects. Startup there was done with launch files and scripts, and I put the safety logic in the controller and the microcontroller, with limits, watchdogs and a safe shutdown sequence. I do know how to containerize and manage services, and I know the main risks: device access, DDS networking, signal handling and restart loops. I would be happy to apply this on a new platform.

中文：不是我项目的核心部分。项目里用 launch 文件和脚本启动，把安全逻辑放在控制器和单片机侧（限幅、看门狗、安全收尾）。我了解容器化和服务管理，也清楚主要风险：设备访问、DDS 网络、信号处理、重启循环；愿意在新平台上应用。

---

## 13. 本章要点速记

1. 容器共享宿主机内核；镜像由不可变层叠成；删除要和安装在同一条 `RUN`。
2. Dockerfile 顺序：少变在前、多变在后；先 `package.xml`+`rosdep`，后源码。示例：改源码重建 241 s，全重建 391 s。
3. 多阶段构建缩小运行镜像（示例 2175 MB → 820 MB）；生产用固定版本标签，不用 `latest`。
4. `exec "$@"` 让程序成为 PID 1，保证 SIGTERM 到达、收尾序列能执行。
5. 设备：udev 稳定别名 + `--device`；CAN 是网络接口，宿主机配置、容器 host 网络；避免 `--privileged`。
6. DDS 多播在 bridge 网络下不通：host 网络或单播发现；域号隔离；共享内存需共享 IPC（未核对）。
7. ARM 构建三路线：交叉编译快但配置复杂，板上原生简单但慢，QEMU 免配置但慢约一个数量级。
8. systemd：`After` 管顺序，`Requires/Wants` 管依赖；`Restart` + `StartLimit*` 防崩溃循环；`oneshot`+`RemainAfterExit` 用于一次性配置。
9. 重启限流只抓快速崩溃：1 s 崩溃、间隔 2 s、窗口内最多 5 次，则第 6 次在 15 s 触发；执行层不应无限重启。
10. 升级用版本化镜像 + 配置分离 + 健康检查 + 可回滚；项目里实际做的是 CAN 配置与看门狗，容器化与 systemd 为延伸。

## 14. 自测题

1. 为什么在后一条 `RUN rm -rf` 删除前一层装的文件，镜像体积不减小？（要点：层不可变，文件仍在下层，只是被覆盖。）
2. `RestartSec=5`，程序启动 2 s 后崩溃，`StartLimitBurst=5`，`StartLimitIntervalSec=60`，会不会触发限流？第几次、什么时间？（答案：周期 7 s，启动在 0,7,14,21,28,35；第 6 次在 35 s 时触发。）
3. 容器里 `ros2 topic list` 看不到另一块板子的话题，列出三个排查方向。（要点：网络模式、域号、防火墙/多播、DDS 配置。）
4. 为什么 CAN 的比特率配置宜放在宿主机的 systemd 服务里，而不是容器里？（要点：接口属于宿主机内核；最小权限；开机顺序可控。）
5. 看门狗 `WatchdogSec=6`，喂狗间隔应取多少以内？喂狗应放在哪里？（答案：小于 3 s；放在真正代表控制环健康的路径上。）

## 15. 相关链接

- 上一章：[11 CMake、ament 构建系统与依赖管理](./11_CMake_ament_构建系统与依赖管理.md)
- 下一章：[13 日志、诊断、性能剖析](./13_日志_诊断_性能剖析_perf_tracing.md)
- 相关：[05 DDS 与分布式通信](./05_DDS与分布式通信.md)、[01 Linux 与 Shell、Git](./01_Linux与Shell_Git.md)、[08 ROS2 Humble 实战](./08_ROS2_Humble实战_launch_参数_生命周期_组件.md)
- 项目：[P2 ROS2-CAN 执行链](../../01_项目课程/P2_未来动力双臂遥操作与差速底盘/10_ROS2_CAN执行链.md)、[P6 生命周期、配置与可观测性](../../01_项目课程/P6_深开鸿M-Robots平台软件/15_生命周期_配置与可观测性.md)
- [模块 README](./README.md)、[基础知识总览](../00_总览与学习路线.md)
