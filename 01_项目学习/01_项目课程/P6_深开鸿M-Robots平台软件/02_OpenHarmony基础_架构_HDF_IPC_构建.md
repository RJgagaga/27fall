# 02 OpenHarmony 基础：系统架构、HDF 驱动模型、进程线程与 IPC、C++ 构建

> 一句话定位：P6 的所有代码都跑在 OpenHarmony（OH）上。这一章不讲 M-Robots 专有内容，只讲"一个做机器人平台软件的人必须懂的 OH 通用知识"，让你在面试里被问到"你的 C++ 代码在 OH 上是怎么跑起来、怎么访问硬件、怎么和别的进程通信、怎么编译的"时能答得有条理。

**本章学完你能……**

- 讲清 OpenHarmony 的分层（内核、系统服务、框架、应用）与三种系统类型（轻量/小型/标准）的区别。
- 解释 HDF（Hardware Driver Foundation）驱动模型：Bind/Init/Release、HCS 配置、HDI 的位置，以及"为什么机器人平台要经过 HDF"。
- 区分进程、线程、IPC（Binder 风格）、分布式软总线（DSoftBus）各自解决什么问题。
- 说出 C++ 代码在 OH 上用 GN + Ninja 构建的基本流程，以及 hvigor 和 NAPI 在哪一端。
- 对任何你不确定的 OH 细节，知道用"typically"措辞回答，并知道哪些是 M-Robots 专有、只讲到设计层。

**前置知识**

- [OpenHarmony / Hi3861 / RK3588 平台基础](../../02_基础知识/11_嵌入式与通信/06_OpenHarmony_Hi3861_RK3588平台基础.md)
- [C++ 现代特性与工程实践](../../02_基础知识/12_机器人软件/02_C++现代特性与工程实践.md)
- [操作系统八股](../../03_八股知识/01_计算机基础/02_操作系统.md)
- 上一章：[01 平台总览与分层架构](./01_平台总览与分层架构.md)

**目录**

1. 为什么机器人工程师要懂 OH
2. 系统架构总览
3. 内核与系统类型
4. HDF 驱动框架
5. 进程、线程与 IPC
6. 分布式能力：软总线与设备管理
7. C++ 构建：GN、Ninja、bundle.json
8. 应用侧：hvigor、ArkTS 与 NAPI
9. 把 OH 知识映射回 P6
10. 常见错误与故障现象
11. 面试问答
12. 要点速记 / 自测题 / 链接

> **说明**：OpenHarmony 版本迭代快，API 名称与目录布局随版本变化。本章写的是"典型、稳定的概念和骨架"，凡涉及具体 API 签名、目录路径、版本差异，面试里请用 "typically" 或 "in the version I used"。本项目统一口径：OpenHarmony 4.1 一类的 LTS 版本（API 11），**标准系统（standard system）**，Linux 内核 5.10，开发板为 RK3568 一类；面试前向导师核对具体版本号即可，口述用 "a standard-type environment on a Linux kernel"。

---

## 1. 为什么机器人工程师要懂 OH

问题：你写了一个 C++ 的底盘适配器，它要读 CAN 帧、发速度命令、被另一个进程调用、还要加载到系统里。这些动作在 OH 上都有"规定走法"：驱动怎么接入、进程怎么互相调用、代码怎么编进镜像。不懂这些，你只能写"能在笔记本上跑的 C++"，而不是"能在板子上被系统管理的组件"。

直觉：OH 对机器人平台来说相当于"带驱动模型、IPC 和分布式能力的嵌入式 Linux/RTOS 发行版"。ROS2 通常跑在 Ubuntu 上，而 P6 的平台是跑在 OH 上，这就是它和常规 ROS2 开发的最大区别，也是 CV 把"OpenHarmony"单独写出来的原因。

---

## 2. 系统架构总览

OH 典型的分层（自底向上）：

```text
┌──────────────────────────────────────────────────────────┐
│ 应用层       应用（HAP）、系统应用                         │
├──────────────────────────────────────────────────────────┤
│ 框架层       Ability 框架、ArkUI、多媒体、通信等框架       │
├──────────────────────────────────────────────────────────┤
│ 系统服务层   系统基本能力子系统集（SAMGR、分布式软总线、    │
│              分布式数据管理、安全、电源……）                 │
│              + 基础软件服务（事件、DFX）                    │
├──────────────────────────────────────────────────────────┤
│ 驱动层       HDF 驱动框架 + 平台驱动 + 外设驱动 + HDI 接口  │
├──────────────────────────────────────────────────────────┤
│ 内核层       内核子系统（LiteOS-M/A 或 Linux）             │
│              + 驱动子系统（HDF）                           │
└──────────────────────────────────────────────────────────┘
```

要点：

1. **子系统**：OH 把功能切成子系统（subsystem），子系统由部件（component/part）构成。每个部件有自己的 `bundle.json` 描述依赖、对外接口和构建目标。
2. **系统服务**由 **SAMGR（System Ability Manager）** 统一管理：每个系统服务是一个 "system ability"，有编号，可以按需加载，被其他进程通过 IPC 调用。
3. **驱动层**与内核紧邻：HDF 把驱动做成"可配置、可加载"的模块，向上通过 HDI 暴露稳定接口。

> **面试要点**：回答 "What is OpenHarmony?" 时，一句话版本："It is an open-source, layered operating system for devices from small embedded boards to standard devices, with a driver framework (HDF), a system-ability model with IPC, and distributed capabilities such as the soft bus."

---

## 3. 内核与系统类型

OH 按设备资源分三类系统（典型表述）：

| 系统类型 | 资源规模（典型） | 内核（典型） | 例子 |
|---|---|---|---|
| 轻量系统 | 很小的 MCU，内存 KB 到 MB 级 | LiteOS-M | Hi3861 类 Wi-Fi 芯片（P4 的灌溉控制就在这类设备上） |
| 小型系统 | 带 MMU 的中等设备，内存数十 MB | LiteOS-A（或 Linux） | 摄像头类、小型网关 |
| 标准系统 | 内存百 MB 以上的富设备 | Linux 内核（也可选 LiteOS-A） | RK3588 开发板、手机/平板类 |

对 P6 来说，**机器人平台软件通常需要进程隔离、IPC、多线程和较丰富的库，因此运行在标准系统一类的环境**：本项目口径是 RK3568 一类开发板上的 OpenHarmony 标准系统（Linux 内核 5.10）；底层电机控制器是独立的 MCU，通过 CAN 与上位机相连，不运行 OpenHarmony。

### 3.1 内核里和机器人最相关的几件事

- **调度**：Linux 内核标准系统默认用 CFS（完全公平调度器）调度普通线程；实时线程可以用 `SCHED_FIFO`/`SCHED_RR` 策略（典型 Linux 能力）。这意味着：不加处理的普通线程**没有硬实时保证**。
- **时钟**：`CLOCK_MONOTONIC`（单调时钟，不受系统校时影响）与 `CLOCK_REALTIME`（墙上时间，可被 NTP 校正而跳变）。状态管理里时间戳用哪个是关键设计（05 章）。
- **内存**：用户态进程有独立地址空间；跨进程传大数据要用共享内存或序列化后走 IPC。
- **文件/设备节点**：驱动向用户态暴露设备节点或服务接口，用户态通过它们访问硬件。

---

## 4. HDF 驱动框架

### 4.1 要解决什么问题

问题：不同厂家的芯片、不同的内核（LiteOS / Linux），如果每个驱动都直接依赖内核 API，驱动就无法复用。HDF（Hardware Driver Foundation）想做到：**驱动一次开发，多内核部署；驱动按需加载；配置与代码分离；向上提供稳定的硬件接口**。

### 4.2 核心概念

| 概念 | 含义（典型） |
|---|---|
| Driver（驱动） | 一个以 `HdfDriverEntry` 注册的模块，包含 Bind/Init/Release 生命周期函数 |
| Device host / Device node | 驱动按 host 分组加载；host 是承载一组驱动的容器（隔离故障） |
| HDF Device Manager | 管理驱动的加载、绑定、服务发布 |
| HCS（HDF Configuration Source） | 描述设备和驱动的配置语言，编译后随镜像分发，驱动运行时读取 |
| Platform driver | 对总线/控制器的抽象，如 I2C、SPI、GPIO、UART 等 |
| Peripheral driver | 具体外设驱动（传感器、显示、电机等） |
| HDI（Hardware Device Interface） | 面向上层的硬件设备接口，通常用 IDL 定义，向系统服务暴露稳定能力 |

### 4.3 一个典型驱动的骨架

> **教学示例（典型骨架，不是项目源码；API 以你使用的版本为准）**

```c
#include "hdf_device_desc.h"
#include "hdf_log.h"

#define HDF_LOG_TAG demo_motor

static int32_t DemoMotorBind(struct HdfDeviceObject *obj)
{
    /* 绑定阶段：发布服务对象，让别人能找到我 */
    HDF_LOGI("demo motor bind");
    return HDF_SUCCESS;
}

static int32_t DemoMotorInit(struct HdfDeviceObject *obj)
{
    /* 初始化阶段：读 HCS 配置，申请资源，使能硬件 */
    HDF_LOGI("demo motor init");
    return HDF_SUCCESS;
}

static void DemoMotorRelease(struct HdfDeviceObject *obj)
{
    /* 释放阶段：对称地回收 Init 里申请的一切 */
    HDF_LOGI("demo motor release");
}

struct HdfDriverEntry g_demoMotorEntry = {
    .moduleVersion = 1,
    .moduleName = "demo_motor",
    .Bind = DemoMotorBind,
    .Init = DemoMotorInit,
    .Release = DemoMotorRelease,
};
HDF_INIT(g_demoMotorEntry);
```

对应的 HCS 配置（典型示意）：

```text
root {
    device_info {
        match_attr = "hdf_manager";
        device_demo :: device {
            device0 :: deviceNode {
                policy = 2;                  // 对外发布服务
                priority = 100;              // 加载顺序
                moduleName = "demo_motor";   // 与驱动 moduleName 对应
                serviceName = "demo_motor_service";
            }
        }
    }
}
```

理解三个阶段的**意图**比背函数名重要：

1. **Bind**：我是谁、我提供什么服务（让系统知道有这个驱动）。
2. **Init**：把硬件真正准备好（读配置、申请 GPIO/总线句柄、上电、自检）。
3. **Release**：与 Init 对称地释放。**泄漏和顺序错误往往出在这里。**

### 4.4 机器人平台为什么要靠 HDF/HDI

- 底盘电机驱动、IMU、激光雷达的总线访问都属于"硬件相关"。让它们放在驱动层/HDI 之后，M-Robots 的设备抽象层才能只面对"稳定接口"，而不是每个板子一套寄存器操作。
- HCS 配置让同一个驱动在不同电机型号（不同减速比、不同通信地址）上通过**配置而非改代码**适配，这与"统一能力模型 + 参数化"的思路一致（03 章）。
- 驱动与上层隔离在不同进程里（驱动在 device host 进程，平台服务在另一个进程）：**驱动崩溃不一定拖垮整个平台**，这与安全执行链里的故障隔离思想一致。

本项目的做法：标准外设（IMU、串口类设备）通过 HDI 接口访问；电机控制器经 CAN，用我们自己的**用户态适配器**访问，并按 HDF 驱动模型把设备节点暴露给适配层。也就是说"HDI 能覆盖的走 HDI，覆盖不了的在用户态自己写适配器"，两种路径对能力模型来说是等价的，因为能力接口只看适配器。

---

## 5. 进程、线程与 IPC

### 5.1 进程与线程

| 概念 | 含义 | 机器人平台中的用法（典型） |
|---|---|---|
| 进程 | 独立地址空间的运行实体，崩溃相互隔离 | 驱动服务、状态服务、运动服务、策略校验服务可分进程 |
| 线程 | 进程内的调度单位，共享内存 | 一个服务内的周期线程（读传感器）、事件线程（处理命令）、工作线程池 |
| 线程优先级 | 调度参考 | 控制线程高、日志线程低 |

在 C++ 里，典型是用 `std::thread`、`std::mutex`、`std::condition_variable`，底层是 pthread（OH 标准系统使用的 C 库通常是 musl，细节随版本）。机器人平台里最常见的并发问题包括：**状态读写竞争、回调里死锁、周期线程被阻塞导致控制抖动**。这些在 [故障案例库](./11_故障案例库.md) 中有对应案例。

### 5.2 IPC：Binder 风格的跨进程调用

问题：运动服务想调用状态服务的接口，但它们在不同进程，不能直接调用函数。

典型机制：OH 的 IPC 使用类似 Binder 的模型：

```text
 客户端进程                      服务端进程
 ┌──────────┐   IRemoteObject   ┌──────────┐
 │ Proxy    │ ───────────────▶  │ Stub     │
 │ (代理)   │   内核/驱动转发    │ (存根)   │
 └──────────┘   序列化的消息     └────┬─────┘
       ▲         MessageParcel         │ 调用真实实现
       └───────── 返回 Reply ──────────┘
```

- 服务端把自己的实现包成 **Stub**，客户端拿到 **Proxy**（代理）；
- 参数通过 **MessageParcel** 序列化；
- 服务通过 SAMGR 按 system ability ID 查找；
- 接口一般用 IDL 描述，工具生成 Proxy/Stub 代码。

**IPC 的几个本质特性**（面试常问）：

1. 同步调用会阻塞客户端线程直到服务端返回；服务端慢就会拖垮客户端 → 要设超时或用异步/单向调用。
2. 参数要序列化，不适合高频大数据；高频大数据用共享内存或专用通道。
3. 服务端崩溃时客户端会收到"死亡通知"（death recipient），需要重连与状态恢复。
4. IPC 调用点是**天然的信任边界**——权限检查常加在这里。

### 5.3 IPC 与 DDS 的分工

IPC 解决"同一设备上不同进程之间的服务调用"；M-DDS 这类数据分发中间件解决"跨设备或松耦合的数据发布订阅"。一个典型设计是：**同设备内服务调用走 IPC，跨设备状态/任务走 M-DDS**，边界在 06 章详细讨论。

---

## 6. 分布式能力：软总线与设备管理

OH 的"分布式"卖点来自几个系统服务（典型表述）：

| 能力 | 作用 |
|---|---|
| 分布式软总线（DSoftBus） | 设备发现、组网、连接、数据传输的统一抽象；屏蔽 Wi-Fi/蓝牙等差异 |
| 分布式设备管理（Device Manager） | 管理可信设备组、设备认证、设备信息 |
| 分布式数据管理 | 跨设备数据同步 |
| 分布式任务调度 | 跨设备的任务拉起与迁移 |

**注意区分**：DSoftBus 是 OH 系统自带的设备连接底座；**M-DDS 是 CV 中平台用于机器人协同的 DDS 类中间件**。两者都和"发现、传输"相关，但抽象层次不同——软总线更偏向设备连接，DDS 更偏向"以数据为中心"的话题发布订阅与 QoS。本项目口径：软总线负责设备组网、认证和传输通道，M-DDS 在其之上（或并行）承载机器人 Topic 与 QoS；发现则分两层——软总线发现"有哪个设备在网内"，M-DDS 的公告发现"这个设备有哪些能力、是否在线"。组网细节属于中间件内部，面试里只讲到这个层次。

面试时遇到 "How is your M-DDS related to the OpenHarmony soft bus?" 的稳妥回答：先讲两者的抽象差异，再说 "I used M-DDS as the data layer for discovery, registration, state sync and task dispatch; the lower-level networking is provided by the platform."（有出入就按事实改。）

---

## 7. C++ 构建：GN、Ninja、bundle.json

### 7.1 构建系统的基本事实

- OH 标准系统使用 **GN**（Generate Ninja）生成构建描述，再由 **Ninja** 执行编译。顶层工具常用 `hb` 命令（`hb set` 选择产品，`hb build` 构建）——具体命令随版本变化。
- 源码以**部件**为单位组织。每个部件根目录有 `bundle.json`，描述部件名、所属子系统、依赖的其他部件、对外暴露的头文件、构建目标。
- 构建目标写在 `BUILD.gn` 里，典型使用 `ohos_shared_library`、`ohos_executable`、`ohos_static_library` 这类模板。

### 7.2 一个典型的 BUILD.gn

> **教学示例（典型写法，非项目源码）**

```gn
import("//build/ohos.gni")

ohos_shared_library("device_abstraction") {
  sources = [
    "src/capability_registry.cpp",
    "src/chassis_adapter.cpp",
  ]
  include_dirs = [ "include" ]
  cflags_cc = [ "-std=c++17", "-Wall", "-Werror" ]
  external_deps = [
    "hilog:libhilog",       # 日志
    "ipc:ipc_core",         # IPC
  ]
  part_name = "robot_platform"
  subsystem_name = "robotics"
}
```

- `sources`：参与编译的 .cpp。
- `include_dirs`：头文件路径。
- `external_deps`：依赖其他**部件**提供的库（格式 `部件名:目标名`），而不是随意写路径——这样依赖关系可被系统检查。
- `part_name` / `subsystem_name`：这个目标属于哪个部件、子系统。

`bundle.json` 的作用是声明部件整体，典型字段有 `name`、`component.subsystem`、`component.deps.components`、`component.build.sub_component`、`component.build.inner_kits`（对外头文件）。

### 7.3 C++ 工程实践在 OH 上的要点

1. **ABI 与标准库**：OH 工具链使用自带的 clang 与 libc++；不要假定有 glibc 特有函数。
2. **异常与 RTTI**：嵌入式环境下常关闭或限制，接口设计倾向用**返回码/`Result` 类型**而不是异常（03 章的接口示例会这样做）。
3. **日志**：使用系统的 hilog，而不是 `printf`。
4. **依赖最小化**：每多一个部件依赖，多一份版本耦合。
5. **单元测试**：典型使用部件自带的测试框架（gtest 风格），与代码一起声明在 `BUILD.gn` 的测试目标里（10 章）。

---

## 8. 应用侧：hvigor、ArkTS 与 NAPI

虽然 P6 的主体是 C++，面试官可能问你"OH 的应用开发是什么"：

| 工具/概念 | 说明（典型） |
|---|---|
| ArkTS | OH 应用的主要开发语言（TypeScript 的扩展） |
| hvigor | OH 应用工程的构建工具（在 DevEco Studio 中使用），产出 HAP/HSP/HAR 包 |
| HAP | 应用的安装包单元 |
| Stage 模型 | 应用组件模型（UIAbility、ExtensionAbility） |
| NAPI（Node-API 风格） | ArkTS 与 C++ 原生库互相调用的桥：C++ 写 Native 模块，通过 NAPI 导出函数给 ArkTS |

**区分**：**hvigor 构建的是应用（ArkTS 侧）；GN/Ninja 构建的是系统部件与原生库。**两者在不同端，别混。

NAPI 的典型流程：C++ 写 `napi_value` 风格函数 → 用 `napi_define_properties`/模块注册导出 → 编成 `.so` → ArkTS 里 `import` 调用。本项目口径：上位调试与监控界面（ArkTS 应用）通过 NAPI 调用一个很薄的 C++ 查询接口（读取状态快照、能力列表），**不提供**从 UI 直接下发运动命令的 NAPI 接口——运动命令只能走提案链（08 章）。

---

## 9. 把 OH 知识映射回 P6

| P6 的工作 | 用到的 OH 知识 |
|---|---|
| C++ 统一能力模型 | 部件化构建、共享库、hilog、线程与锁 |
| 驱动适配 | HDF/HDI、设备节点、HCS 参数化配置 |
| 状态管理 | 单调时钟、线程安全的状态存储、跨进程读取（IPC） |
| M-DDS 协同 | 网络栈、软总线（若使用）、多线程回调 |
| 安全执行链 | 进程隔离、IPC 权限检查、看门狗/超时 |

一个具体设计问题：**为什么安全校验最好做成独立服务/进程，而不是库函数？**

- 库函数在调用者进程里，调用者可以绕过（根本不调用）。
- 独立服务通过 IPC 提供，执行路径只接受校验服务签发的"通过的命令"，**绕过需要突破进程边界**。
- 这是"唯一通道"原则在 OH 上的自然实现（08 章）。本项目口径：策略校验是**独立服务进程**，执行网关只接受带批准票据的命令；这样规划进程崩溃或被替换也绕不过校验。

---

## 10. 常见错误与故障现象

| 现象 | 常见原因 |
|---|---|
| 驱动加载失败 | HCS 里的 `moduleName` 与驱动不一致；依赖驱动优先级排序不对 |
| 重启后设备打不开 | Release 没有对称释放，或 Init 顺序错误 |
| IPC 调用卡住 | 同步调用对端阻塞/死锁；没有超时 |
| 服务崩溃后一直失败 | 没处理死亡通知，缓存了失效的 proxy |
| 构建报找不到头文件 | `external_deps` 缺失或部件没有在 `inner_kits` 里声明对外头文件 |
| 时间戳跳变 | 用墙上时间而非单调时钟 |
| 周期线程抖动 | 在周期线程里做了阻塞 IPC、打日志、分配大内存 |

---

## 10.5 一条运动命令穿越系统的完整路径（典型设计）

前面各节分别讲了内核、HDF、IPC、构建。这一节把它们串成一条路径：一条来自 ROS2 的速度命令，从进入平台到变成电机上的力矩指令，要经过哪些进程、哪些 IPC 跳转、每一跳大致要花多少时间。数值是设计预算（教学示例），用来说明"为什么进程划分会影响延迟与可调试性"，不是实测。

```text
ROS2 /cmd_vel
   │  ① 桥接适配进程（rclcpp 节点，用户态）          ≤2 ms
   ▼
提案封装 → ② IPC 调用策略校验服务（独立进程）        每跳 0.3～1 ms
   │  ③ 校验（schema/权限/有效期/状态新鲜度/范围）    ≤5 ms
   ▼
批准票据 → ④ 执行网关进程（验票、限幅、平滑）        ≤1 ms
   ▼
⑤ 设备适配器（C++ 能力实现）→ ⑥ HDF/用户态 CAN 适配  ≤3 ms
   ▼
⑦ CAN 总线 → MCU 下一个控制周期取用                  ≤2 ms（MCU 100～1000 Hz）
```

**为什么每个进程边界都值得保留**：

1. 桥接进程崩溃只会丢失命令来源，不会带走策略校验与驱动，重启后看门狗保证机器人先停下来。
2. 策略校验是独立服务，网关只认票据，所以"绕过校验"需要突破进程边界（见本章 9 节）。
3. 驱动适配与设备节点属于最靠近硬件的一层，权限最大，因此代码量要最小、逻辑最少，只做"统一命令 → 总线帧"的转换。

**延迟账**：把上图七段加起来，名义值约 10 ms，上限约 30 ms；平台给"命令到执行器"的设计预算是小于 100 ms，余量主要留给 Wi-Fi 重传、线程调度抖动与偶发 GC/缺页。这个余量是否够用，不能靠推算，要靠 10 章的方法测：在每个进程入口和出口写入同一条单调时钟时间戳，用回放工具统计分位数。

**调试时的读法**：出现"机器人动作迟钝"，按这七段逐段看 hilog 里各进程的入口时间差，定位是哪一跳在排队。这比"盯着最后的现象猜"快得多，这也是 15 章强调统一日志字段（seq、proposal_id、stamp）的原因。

面试里可以这样收束：*In a design like this, a velocity command crosses roughly four process boundaries before it becomes a CAN frame. I keep the boundaries because they give fault isolation and a single enforcement point, and I pay for them with a budget of a few milliseconds per hop.* 译：这类设计中，一条速度命令在变成 CAN 帧之前大约要跨过四个进程边界。我保留这些边界是因为它们带来故障隔离和唯一的约束点，代价是每跳几毫秒的预算。

---

## 11. 面试问答

**Q1. Describe the OpenHarmony architecture in your own words.**

> From the bottom: the kernel and the HDF driver framework, then system services managed by a system-ability manager and reached through IPC, then the application framework. Different device classes use different kernels, so the same upper-layer model can run from small boards to richer devices. For my work, the key pieces were the driver layer for hardware access and the C++ libraries I built as components.

译：自底向上是内核+HDF、系统服务（SAMGR+IPC）、应用框架；不同设备类型用不同内核。我关心的是驱动层和我做的 C++ 部件。

追问：你用的是哪种系统类型？→ 按事实答，不确定就说 "standard-type environment" 之前先核对。

**Q2. What is HDF and why use it?**

> HDF is OpenHarmony's driver framework. It separates driver code from configuration, loads drivers on demand, and exposes hardware through stable interfaces, so a driver can be reused across kernels and boards. For a robot platform, that means my device abstraction layer talks to stable interfaces instead of board-specific registers.

追问：Bind、Init、Release 各做什么？→ 见 4.3。

**Q3. How do two processes talk in OpenHarmony?**

> Typically through IPC with a proxy and stub model. The client holds a proxy, the service implements a stub, and arguments are serialised into a parcel. A synchronous call blocks the caller, so I would put timeouts on any call that sits on a control path.

追问：高频数据怎么传？→ 共享内存或发布订阅，不走同步 IPC。

**Q4. How is C++ code built on OpenHarmony?**

> The system uses GN to generate Ninja build files. Code is organised into components, each with a bundle.json and BUILD.gn targets such as shared libraries. Dependencies on other components are declared explicitly, so the build can check them. Application code in ArkTS is built separately with hvigor, and NAPI bridges the two.

**Q5. Why not just use Linux and ROS2 directly?**

> ROS2 on Linux is great for research, but the platform I work on targets OpenHarmony devices, with its driver framework, componentised build and distributed capabilities. My job was to make ROS/ROS2-style motion commands work on that system through a unified capability layer.

---

## 12. 本章要点速记

1. OH 是分层、组件化的操作系统，设备分轻量/小型/标准三类，内核可选。
2. 系统服务由 SAMGR 管理，用 IPC（Proxy/Stub/Parcel）互相调用。
3. HDF = 驱动与配置分离 + 按需加载 + 稳定的 HDI；驱动三阶段 Bind/Init/Release。
4. IPC 同步阻塞，需超时；服务崩溃要处理死亡通知。
5. 软总线是 OH 的设备连接底座，M-DDS 是机器人协同的数据中间件，别混。
6. 构建：GN + Ninja，bundle.json 描述部件，external_deps 声明依赖。
7. hvigor 构建 ArkTS 应用，NAPI 连接 ArkTS 与 C++。
8. 时间戳用单调时钟，周期线程不要做阻塞调用。

## 自测题

1. HDF 为什么要把配置（HCS）与代码分开？（要点：同一驱动适配多型号，不改代码。）
2. 画出 IPC 的 Proxy/Stub 关系并说明同步调用的风险。
3. GN/Ninja 与 hvigor 各构建什么？
4. 为什么周期控制线程里不应做同步 IPC？（要点：阻塞时间不确定，破坏周期。）
5. 为什么策略校验做成独立服务能让"唯一通道"更强？（要点：进程边界，难绕过。）

## 相关链接

- 上一章：[01 平台总览与分层架构](./01_平台总览与分层架构.md)
- 下一章：[03 统一能力模型与设备抽象](./03_统一能力模型与设备抽象.md)
- 基础：[OpenHarmony 平台基础](../../02_基础知识/11_嵌入式与通信/06_OpenHarmony_Hi3861_RK3588平台基础.md)
- 相关项目：[P4 OpenHarmony 智能园艺系统](../P4_OpenHarmony智能园艺系统/README.md)
