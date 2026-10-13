# 机器人软件：学习顺序与前置关系

> **一句话定位**：软件工程与机器人中间件：Linux/Git、C++、Python、ROS2 Humble（ROS1 仅对比）、DDS、仿真与动力学工具链、URDF/TF。

**本模块学完你能：**

- 熟练使用 Linux 与 Git 工作流；
- 写出现代 C++ 并理解所有权与并发；
- 解释 ROS2 的节点、话题、QoS、执行器与 TF；
- 解释 DDS 发现与 QoS；
- 使用 MuJoCo/Pinocchio/KDL 工具链。

**前置知识**：[微积分与常微分方程（时间与离散化）](../01_数学基础/05_数值方法与离散化.md)。

## 学习顺序（已完成）

| 序号 | 文件 | 内容 | 来源 |
|---|---|---|---|
| 01 | [Linux与Shell_Git](./01_Linux与Shell_Git.md) | Linux、Shell、Git | 原有章节（重组） |
| 02 | [C++现代特性与工程实践](./02_C++现代特性与工程实践.md) | C++ 现代特性 | 原有章节（重组） |
| 03 | [Python工程与科学计算](./03_Python工程与科学计算.md) | Python 工程与科学计算 | 原有章节（重组） |
| 04 | [ROS2_Humble核心概念与ROS1对比](./04_ROS2_Humble核心概念与ROS1对比.md) | ROS2 Humble 核心概念，ROS1 仅作对比 | 原有章节（重组） |
| 05 | [DDS与分布式通信](./05_DDS与分布式通信.md) | DDS、发现、QoS | 原有章节（重组） |
| 06 | [MuJoCo_Pinocchio_KDL_工具链](./06_MuJoCo_Pinocchio_KDL_工具链.md) | 仿真与动力学库 | 原有章节（重组） |
| 07 | [URDF_TF_坐标系实践](./07_URDF_TF_坐标系实践.md) | URDF、TF | 原有章节（重组） |
| 09 | [ROS2测试与CI_launch_testing_gtest_rosbag回放](./09_ROS2测试与CI_launch_testing_gtest_rosbag回放.md) | 单元测试、launch_testing、rosbag2 回放回归、flaky 与 CI | 本轮新写 |
| 10 | [仿真平台_Gazebo_Isaac_MuJoCo对比](./10_仿真平台_Gazebo_Isaac_MuJoCo对比.md) | 步长与刚度稳定性、Gazebo/Isaac/MuJoCo 取舍、域随机化 | 本轮新写 |
| 11 | [CMake_ament_构建系统与依赖管理](./11_CMake_ament_构建系统与依赖管理.md) | 现代 CMake、package.xml 依赖、ament_cmake/python、colcon、交叉编译 | 本轮新写 |
| 12 | [Docker与部署_交叉编译_systemd](./12_Docker与部署_交叉编译_systemd.md) | 镜像分层与多阶段、设备映射、DDS 网络、systemd 重启限流、升级回滚 | 本轮新写 |
| 13 | [日志_诊断_性能剖析_perf_tracing](./13_日志_诊断_性能剖析_perf_tracing.md) | 日志节流、诊断四级、perf 火焰图、tracing、延迟预算 | 本轮新写 |
| 14 | [ros2_control与硬件接口实战](./14_ros2_control与硬件接口实战.md) | 硬件接口、控制器管理器、read-update-write 循环、控制器链、实时性与 CAN 桥接 | 本轮新写 |

## 后续章节（计划，下一轮补写）

下表为已规划、尚未写成的章节；详细要点与篇幅要求见 [结构说明与扩展计划](../结构说明与扩展计划.md)。在文件写成之前，它们不会出现链接，避免断链。

| 预定文件 | 主题 | 篇幅 |
|---|---|---|
| `08_ROS2_Humble实战_launch_参数_生命周期_组件.md` | Python launch 文件，参数声明与动态重配置，生命周期节点，组件与进程内通信，QoS 调优 | 约 7000 汉字 |

## 与项目的关联

P1/P2/P5/P6 的 ROS 工程，P6 的 DDS 与 TF，P2 的 MuJoCo/Pinocchio/KDL。

## 阅读顺序建议

Linux/Git → C++ → Python → ROS2 核心 → DDS → 工具链与 URDF/TF。

## 相关链接

- [基础知识总览与学习路线](../00_总览与学习路线.md)
- [结构说明与扩展计划](../结构说明与扩展计划.md)
- [项目学习 README](../../README.md)
