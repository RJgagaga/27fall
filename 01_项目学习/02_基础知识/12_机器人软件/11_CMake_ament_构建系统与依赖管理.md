# 11 CMake、ament 构建系统与依赖管理

> **一句话定位**：写代码只是机器人软件的一半，另一半是"让别人（和三个月后的你）能一条命令把它编出来、装上去、跑起来"。本章从 CMake 的最小模型讲起（目标、可见性、安装、导出），再讲 ROS2 Humble 在其上叠加的 ament_cmake / ament_python、`package.xml` 的依赖语义、colcon 的工作区与覆盖层（overlay），最后讲交叉编译到 ARM 以及最常见的链接/找包错误怎么读、怎么修。

**本章学完你能：**

- 用"目标（target）+ 属性"的方式理解现代 CMake，解释 `PUBLIC/PRIVATE/INTERFACE` 的区别并写出一个能被别人 `find_package` 的库；
- 说明 `package.xml` 里 `buildtool_depend`、`depend`、`build_depend`、`exec_depend`、`test_depend` 各自在什么时候起作用，并说出 `rosdep` 解决什么问题；
- 写出一个 `ament_cmake` 包（节点可执行文件、库、消息依赖、安装 launch/config、导出）和一个 `ament_python` 包的骨架，并指出两者的关键差异；
- 手算一个包依赖图的构建顺序、并行层数与关键路径，并说明为什么增加 `-j` 超过某个值就不再变快；
- 解释 colcon 的 underlay/overlay、`install/setup.bash` 做了什么、`--symlink-install` 的好处与坑；
- 读懂 `undefined reference`、`Could not find a package configuration file`、`fatal error: xxx.hpp: No such file` 三类典型错误并给出处理顺序；
- 写出一份交叉编译工具链文件的骨架，说明 sysroot 与宿主机库的区别；
- 说明自己项目里的构建实践，并诚实区分哪些是延伸知识。

**前置知识：**

- [C++ 现代特性与工程实践](./02_C++现代特性与工程实践.md)（头文件、链接、模板）
- [Linux 与 Shell、Git](./01_Linux与Shell_Git.md)（环境变量、`PATH`）
- [ROS2 Humble 核心概念与 ROS1 对比](./04_ROS2_Humble核心概念与ROS1对比.md)（包、节点、工作区）
- [ROS2 测试与 CI](./09_ROS2测试与CI_launch_testing_gtest_rosbag回放.md)（`BUILD_TESTING`、`colcon test`）

> **关于代码的说明**：本机没有 ROS2，所以 `ament_cmake`、`ament_python`、`colcon`、`rosdep` 的内容**未在 ROS 环境运行**，只作结构示意，具体宏名、选项在 Humble 上的行为凡不确定处标"未核对"。**纯 CMake 的部分是在本机真实运行过的**（CMake 3.26.3 + MinGW GCC 8.1.0，Windows）：库的构建、安装、导出、被另一个项目 `find_package` 并链接，以及三类典型错误的真实输出，都在文中给出（标"本机复核"）。依赖图的拓扑排序与关键路径用 Python 复核。

**目录**

1. 为什么构建系统是"软件工程的地基"
2. 现代 CMake 的最小模型
3. 动手：一个库的构建、安装、导出与使用（本机复核）
4. package.xml：依赖的语义
5. ament_cmake：在 CMake 上叠加了什么
6. ament_python：纯 Python 包
7. colcon：工作区、覆盖层与并行构建
8. 手算：构建顺序、并行与关键路径
9. 交叉编译到 ARM
10. 常见错误与排查
11. 在七个项目里怎么用（含诚实边界）
12. 面试怎么问、怎么答
13. 本章要点速记
14. 自测题
15. 相关链接

---

## 1. 为什么构建系统是"软件工程的地基"

### 1.1 要解决的问题

一个机器人工程动辄几十个包：消息定义、驱动、滤波、规划、控制、桥接。它们互相依赖，用不同语言（C++、Python），依赖第三方库（Eigen、Boost、OpenCV、Pinocchio、yaml-cpp……）。构建系统要回答四个问题：

1. **编什么**：哪些源文件组成哪个库/可执行文件；
2. **依赖谁**：编译时需要谁的头文件，链接时需要谁的库，运行时需要谁；
3. **装到哪**：编出来的东西放在哪里，别的包怎么找到它；
4. **按什么顺序**：几十个包的依赖形成一张图，必须按拓扑顺序编。

### 1.2 层次关系

```text
colcon        ← 工作区级：发现包、按依赖拓扑排序、并行调用下面的构建工具
  │
  ├─ ament_cmake 包  ← 包级：CMake + ament 宏（导出、资源索引、测试）
  │      └─ CMake    ← 目标级：编译器、链接器、安装规则
  └─ ament_python 包 ← 包级：setuptools（setup.py / setup.cfg）
rosdep        ← 系统依赖：把 package.xml 里的依赖键翻译成 apt/pip 包并安装
```

> **面试要点**：能说清"colcon 管工作区，ament 管 ROS 包约定，CMake 管编译细节，rosdep 管系统包"这四层分工，就已经超过了大多数只会 `colcon build` 的候选人。

---

## 2. 现代 CMake 的最小模型

### 2.1 目标与属性

"现代 CMake"（3.x 之后推荐的写法）围绕**目标（target）**。一个目标是一个库或可执行文件，带有一组**属性**：源文件、头文件目录、编译选项、链接依赖。要点是**不要用全局变量去影响所有目标**（旧式的 `include_directories`、`link_libraries`、`add_definitions`），而是把信息挂在目标上，通过依赖关系自动传递。

```cmake
add_library(limiter_core src/limiter.cpp)
target_include_directories(limiter_core PUBLIC include)
target_link_libraries(app PRIVATE limiter_core)
```

### 2.2 可见性：PUBLIC / PRIVATE / INTERFACE

假设库 A 链接库 B：

| 关键字 | 含义 | A 自己编译时 | 链接 A 的目标 C 编译时 |
|---|---|---|---|
| `PRIVATE` | B 只是 A 的实现细节 | 使用 B 的头文件/库 | **不**继承 B |
| `PUBLIC` | A 的头文件里暴露了 B | 使用 | **继承** B |
| `INTERFACE` | A 自己不用，只是下游需要（如头文件库） | 不使用 | 继承 |

**判断办法**：看 A 的**公开头文件**里有没有 `#include <B的头>` 或使用了 B 的类型。有就是 `PUBLIC`；只有 `.cpp` 里用，就是 `PRIVATE`。可见性写错的后果：写成 `PRIVATE` 会让下游报"找不到头文件"；全写 `PUBLIC` 会让依赖无谓扩散，增加编译时间并制造耦合。

### 2.3 构建树与源码树分离

最佳实践是"源码外构建"：`cmake -S <源码目录> -B <构建目录>`。构建产物都在构建目录，删掉它就是干净重来，源码目录不被污染。`colcon` 正是为每个包创建独立的 `build/<包名>/` 目录。

### 2.4 配置类型

`CMAKE_BUILD_TYPE` 取 `Debug`、`Release`、`RelWithDebInfo`、`MinSizeRel`。ROS2 默认不设置构建类型时，编译出的代码可能没有优化（只有 `-O0` 或无优化标志，视具体设置，**未核对**）——这是"算法在笔记本上很慢"的常见原因。部署前应显式指定：

```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

对控制与规划节点，Release 与 Debug 的运行时间可能相差数倍（取决于代码，下面的数不是实测）。**性能测量必须在 Release 下做**。

---

## 3. 动手：一个库的构建、安装、导出与使用（本机复核）

下面做一个完整的小实验：把速度限幅库做成可安装、可被他人 `find_package` 的形式。这正是 `ament_cmake` 里 `ament_export_targets` 等宏背后做的事——了解裸 CMake 的版本，你就明白那些宏在帮你省什么。

### 3.1 文件结构

```text
lib/
  CMakeLists.txt
  include/limiter/limiter.hpp
  src/limiter.cpp
app/
  CMakeLists.txt
  main.cpp
```

### 3.2 库的 CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.16)
project(limiter_core VERSION 0.1.0 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)

add_library(limiter_core src/limiter.cpp)
target_include_directories(limiter_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS limiter_core EXPORT limiter_coreTargets ARCHIVE DESTINATION lib)
install(EXPORT limiter_coreTargets NAMESPACE limiter_core:: DESTINATION lib/cmake/limiter_core)

file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/limiter_coreConfig.cmake
  "include(\"\${CMAKE_CURRENT_LIST_DIR}/limiter_coreTargets.cmake\")\n")
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/limiter_coreConfig.cmake
        DESTINATION lib/cmake/limiter_core)
```

解释关键行：

- `$<BUILD_INTERFACE:...>` 与 `$<INSTALL_INTERFACE:...>`：同一个头文件目录，在"构建树里直接用"和"安装后被别人用"时路径不同，用生成器表达式分别指定；
- `install(TARGETS ... EXPORT ...)`：安装库文件，同时把"这个目标的属性"记入一个导出集合；
- `install(EXPORT ... NAMESPACE limiter_core::)`：生成 `limiter_coreTargets.cmake`，下游就能写 `limiter_core::limiter_core`。加命名空间的好处是写错名字时**配置阶段就报错**，而不是被当成普通库名到链接阶段才失败；
- `limiter_coreConfig.cmake`：`find_package(limiter_core)` 实际找的就是这个文件，它再包含上面生成的 Targets 文件。

### 3.3 使用方

```cmake
cmake_minimum_required(VERSION 3.16)
project(app LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
find_package(limiter_core REQUIRED)
add_executable(app main.cpp)
target_link_libraries(app limiter_core::limiter_core)
```

### 3.4 实际运行与输出（本机复核）

```bash
cmake -S lib -B build_lib -G "MinGW Makefiles" -DCMAKE_INSTALL_PREFIX=$PWD/install
cmake --build build_lib
cmake --install build_lib
cmake -S app -B build_app -G "MinGW Makefiles" -DCMAKE_PREFIX_PATH=$PWD/install
cmake --build build_app
./build_app/app.exe
```

关键输出（节选）：

```text
[100%] Linking CXX static library liblimiter_core.a
-- Installing: .../install/include/limiter/limiter.hpp
-- Installing: .../install/lib/liblimiter_core.a
-- Installing: .../install/lib/cmake/limiter_core/limiter_coreTargets.cmake
-- Installing: .../install/lib/cmake/limiter_core/limiter_coreConfig.cmake
[100%] Linking CXX executable app.exe
0.050
```

最后一行 `0.050` 是 `limit_cmd(1.0, 0.0, 2.0, 0.5, 0.1)` 的结果，与上一章（[第 09 章](./09_ROS2测试与CI_launch_testing_gtest_rosbag回放.md)）手算值 $0.5\times0.1=0.05$ 一致。要点：`app` 的 `CMakeLists.txt` 没有写任何头文件路径，`limiter_core::limiter_core` 把 `include` 目录和库文件**一起传递**给了它——这就是"目标携带属性"的好处。

### 3.5 故意制造两种错误（本机复核）

**错误一：找不到包。** 在配置 `app` 时不提供 `CMAKE_PREFIX_PATH`：

```text
CMake Error at CMakeLists.txt:4 (find_package):
  By not providing "Findlimiter_core.cmake" in CMAKE_MODULE_PATH this project
  has asked CMake to find a package configuration file provided by
  "limiter_core", but CMake did not find one.
```

含义：CMake 去 `CMAKE_PREFIX_PATH`（以及系统默认位置）下找 `limiter_coreConfig.cmake` 或 `limiter_core-config.cmake`，没找到。排查顺序：这个包装在哪？`CMAKE_PREFIX_PATH`（ROS 里由 `source install/setup.bash` 设置 `AMENT_PREFIX_PATH`/`CMAKE_PREFIX_PATH`）包含它吗？

**错误二：链接缺符号。** 只给头文件路径，不链接库，直接用 g++ 编译：

```text
undefined reference to `limiter::limit_cmd(double, double, double, double, double)'
collect2.exe: error: ld returned 1 exit status
```

含义：**编译阶段**只需要声明（头文件里有），所以通过；**链接阶段**需要定义（在库的目标文件里），找不到所以失败。`undefined reference` 一律是"缺定义"——要么没链接那个库，要么链接顺序不对（静态库依赖顺序），要么声明与定义的签名不一致（比如函数在 `extern "C"` 与否、`const` 不同、命名空间不同）。

> **注意**：编译错误（`fatal error: xxx.hpp: No such file`）与链接错误（`undefined reference`）是两类问题，处理方式完全不同。先判断它发生在哪个阶段，是最重要的第一步。

---

## 4. package.xml：依赖的语义

### 4.1 作用

`package.xml` 是 ROS 包的元数据：名字、版本、维护者、许可，**最重要的是依赖声明**。它被两类工具读取：`colcon`（用来排拓扑顺序）和 `rosdep`（用来安装系统依赖）。

### 4.2 依赖标签

| 标签 | 含义 | 什么时候需要 |
|---|---|---|
| `buildtool_depend` | 构建工具本身（如 `ament_cmake`、`ament_python`） | 构建时，宿主机上 |
| `build_depend` | 编译时需要（头文件、库） | 只在编译 |
| `build_export_depend` | 下游编译我的头文件时也需要它（头文件里暴露了它） | 对应 CMake 的 `PUBLIC` |
| `exec_depend` | 运行时需要（动态库、Python 模块、其他节点） | 运行 |
| `depend` | 等价于 `build_depend` + `build_export_depend` + `exec_depend` | 最省事，最常用 |
| `test_depend` | 仅测试需要（gtest、launch_testing） | `BUILD_TESTING` |

**对应关系**：`package.xml` 里的 `build_export_depend` 与 CMake 里的 `PUBLIC`、`ament_export_dependencies` 说的是同一件事的两个侧面；写错的后果是"我这能编，下游编不过"。

### 4.3 最小例子

```xml
<?xml version="1.0"?>
<package format="3">
  <name>demo_chassis</name>
  <version>0.1.0</version>
  <description>Chassis speed limiter</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>geometry_msgs</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### 4.4 rosdep：把键翻译成系统包

`rosdep` 读取所有 `package.xml` 里的依赖，在工作区里找得到源码的包跳过，其余按"rosdep 键 → 当前操作系统的包名"的数据库翻译并调用 `apt`/`pip` 安装：

```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

意义：新机器上"装好 ROS2 → 拉源码 → 一条命令装齐系统依赖 → colcon build"，不再靠口头交接"还要装这个那个"。找不到键会报"Cannot locate rosdep definition"，这时要么拼写错，要么该库不在数据库里（需要自己写 rosdep 规则或改为源码依赖）。

---

## 5. ament_cmake：在 CMake 上叠加了什么

### 5.1 一个典型的 CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.8)
project(demo_chassis)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# 库：纯逻辑核心（可被测试和其他包复用）
add_library(limiter_core src/limiter.cpp)
target_include_directories(limiter_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# 可执行文件：ROS 节点外壳
add_executable(speed_limiter src/speed_limiter_node.cpp)
target_link_libraries(speed_limiter limiter_core)
ament_target_dependencies(speed_limiter rclcpp geometry_msgs)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS limiter_core EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS speed_limiter DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp geometry_msgs)
ament_package()
```

### 5.2 逐行理解关键宏

| 宏 | 作用 | 对应裸 CMake 的什么 |
|---|---|---|
| `find_package(ament_cmake REQUIRED)` | 引入 ament 的宏 | — |
| `ament_target_dependencies(tgt dep...)` | 给目标加上依赖包的头文件目录和库 | 一串 `target_include_directories` + `target_link_libraries`（Humble 时代常用；较新版本更推荐直接链接带命名空间的目标，具体推荐写法**未核对**） |
| `install(TARGETS speed_limiter DESTINATION lib/${PROJECT_NAME})` | 把可执行文件装到 `lib/<包名>/`，`ros2 run` 才找得到 | 约定位置，不是随便写的 |
| `install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})` | 安装 launch 与参数文件，`FindPackageShare` 才能定位 | 约定位置 |
| `ament_export_targets(...)` | 导出目标，让下游 `find_package(demo_chassis)` 能用 `demo_chassis::limiter_core` | 本章 §3 手写的 `install(EXPORT)` 加 Config 文件 |
| `ament_export_dependencies(...)` | 告诉下游：用我的头文件时需要先 `find_package` 这些依赖 | `find_dependency` |
| `ament_package()` | **必须放在最后**，生成资源索引、Config 文件 | — |

**"必须最后"**：`ament_package()` 之后再加的安装规则或导出可能不被处理，这是新手常见错误。

**资源索引（ament index）**：安装时 ament 在 `share/ament_index/resource_index/packages/<包名>` 放一个标记文件。`ros2 pkg list`、`FindPackageShare`、`ros2 run` 靠它找包。因此"编译成功但 `ros2 run` 找不到可执行文件"的常见原因是：没 `source install/setup.bash`，或可执行文件没装到 `lib/<包名>/`。

### 5.3 自定义消息

自定义消息用 `rosidl_generate_interfaces`，包要在 `package.xml` 里声明 `rosidl_interface_packages` 分组与 `rosidl_default_generators`/`rosidl_default_runtime`。消息包最好**独立成一个纯接口包**（如 `chassis_msgs`），不放实现代码——这样下游只依赖接口、不被迫依赖实现，也避免循环依赖。

### 5.4 组件（composable node）注册

若按第 08 章把节点做成组件，需要 `rclcpp_components_register_node(...)`，并在库里注册；这决定该节点可以被组件容器动态加载（具体宏参数**未核对**，以所用 Humble 文档为准）。

---

## 6. ament_python：纯 Python 包

### 6.1 结构

```text
demo_chassis_py/
  package.xml                # buildtool_depend: ament_python
  setup.py
  setup.cfg
  resource/demo_chassis_py   # 空标记文件，ament 资源索引用
  demo_chassis_py/
    __init__.py
    speed_limiter.py
  launch/  config/  test/
```

### 6.2 setup.py 的关键点

```python
from setuptools import setup
import os
from glob import glob

package_name = 'demo_chassis_py'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'speed_limiter = demo_chassis_py.speed_limiter:main',
        ],
    },
)
```

- `data_files`：除了 Python 模块外，launch、config 要显式列出才会被安装。**新增一个 launch 文件却忘了改 `data_files`，是最常见的"本地能跑，别人装了找不到"**；
- `entry_points`：生成 `ros2 run demo_chassis_py speed_limiter` 对应的可执行入口；
- `setup.cfg` 里需要把脚本装到 `lib/<包名>`（`[develop] script_dir=$base/lib/<pkg>` 与 `[install] install_scripts=$base/lib/<pkg>`），否则 `ros2 run` 找不到；
- `--symlink-install` 对 Python 包很有用：源码改动立即生效，不必重新安装；但对 C++ 包只对"安装的非编译文件"（launch、yaml）有软链接的好处。

### 6.3 ament_cmake 与 ament_python 对比

| 维度 | ament_cmake | ament_python |
|---|---|---|
| 语言 | C++（也可混合 Python） | 纯 Python |
| 构建 | 编译、链接 | 打包、安装 |
| 消息定义 | 支持 `rosidl_generate_interfaces` | 不支持（需要放在 cmake 包） |
| 配置文件安装 | `install(DIRECTORY ...)` | `data_files` |
| 修改后 | 需重新编译 | `--symlink-install` 时多数改动即时生效 |
| 适用 | 性能敏感、需导出库/接口 | 快速原型、胶水脚本、上层逻辑 |

---

## 7. colcon：工作区、覆盖层与并行构建

### 7.1 工作区结构

```text
ws/
  src/        # 源码，包含多个包
  build/      # 每包一个构建目录（中间产物）
  install/    # 安装目录（运行时使用）
  log/        # 构建日志
```

`colcon build` 做的事：扫描 `src/` 下所有包 → 读取 `package.xml` 得到依赖图 → 拓扑排序 → 按依赖并行调用各包的构建工具 → 安装到 `install/`。

### 7.2 underlay 与 overlay

`source /opt/ros/humble/setup.bash` 把系统安装的 ROS2 放到环境里（**underlay**）；`source install/setup.bash` 再把本工作区叠加在上面（**overlay**）。叠加的含义：同名包以后者为准，工作区里的版本**覆盖**系统版本。这是你可以"改一个官方包、只覆盖它"的机制，也是"以为在用自己的包，其实用的是系统的"这种 bug 的来源（环境变量 `AMENT_PREFIX_PATH` 的顺序决定）。

排查：

```bash
echo $AMENT_PREFIX_PATH | tr ':' '\n'      # 看顺序
ros2 pkg prefix demo_chassis                # 看当前解析到哪一个
```

**不要在同一个 shell 里 source 多个互不相干的工作区**，因为它们会叠成一个难以推断的环境；开新终端最干净。

### 7.3 常用选项

| 选项 | 作用 |
|---|---|
| `--packages-select A B` | 只构建指定包 |
| `--packages-up-to A` | 构建 A 及其所有依赖 |
| `--packages-above A` | 构建依赖于 A 的包 |
| `--symlink-install` | 安装时用符号链接指向源码 |
| `--cmake-args -DCMAKE_BUILD_TYPE=Release` | 传给 CMake |
| `--parallel-workers N` | 同时构建的包数 |
| `--event-handlers console_direct+` | 实时显示输出，便于看错误 |

### 7.4 清洁构建与缓存

构建行为异常（找不到刚加的文件、莫名其妙的旧符号）时，先删除对应包的 `build/<包名>` 与 `install/<包名>` 再重编；全部删除也可以。头文件依赖更改（比如改了消息定义）而下游没重编，会导致运行时类型哈希不匹配等难查的问题，所以**改动接口包后，应让所有依赖它的包重新构建**（`--packages-above`）。

`ccache` 能缓存编译结果，在 CI 或频繁清洁构建时显著缩短时间（需要配置编译器启动器，细节**未核对**）。

---

## 8. 手算：构建顺序、并行与关键路径

### 8.1 问题

有 6 个包，依赖关系如下（`A: B C` 表示 A 依赖 B 和 C），构建耗时（秒，示例，非项目实测）：

| 包 | 依赖 | 耗时 |
|---|---|---|
| chassis_msgs | — | 40 |
| limiter_core | — | 10 |
| chassis_driver | chassis_msgs, limiter_core | 60 |
| localization | chassis_msgs | 90 |
| planner | localization, chassis_msgs | 120 |
| bringup | chassis_driver, localization, planner | 5 |

### 8.2 拓扑分层

先构建没有依赖的：第 0 层 `chassis_msgs`、`limiter_core`；第 1 层 `chassis_driver`、`localization`；第 2 层 `planner`；第 3 层 `bringup`。（Python 的 `graphlib.TopologicalSorter` 复核，层结构一致。）

### 8.3 串行时间与关键路径

- 串行总时间 $40+10+60+90+120+5=325$ s；
- **关键路径**：最长依赖链 `chassis_msgs → localization → planner → bringup`，耗时 $40+90+120+5=255$ s。这是无限多核也无法低于的下限；
- 用 2 个并行任务时，贪心调度得到的完成时间 255 s（Python 复核）；用 3 个或 8 个也是 255 s。

**结论**：并行带来 325 → 255 s（约 22% 的节省），但再增加 worker 数**毫无用处**，因为瓶颈是关键路径。要继续缩短，只能：拆分关键路径上的包（如把 `planner` 拆成可并行的两部分）、减少关键路径上的重依赖、使用 `ccache`、或减少每个包的编译时间。

### 8.4 循环依赖

若 `A` 依赖 `B` 而 `B` 依赖 `A`，拓扑排序失败。Python 复核：`graphlib.CycleError`，报告环为 `['a','b','a']`。`colcon` 遇到循环依赖会直接报错。处理方法：把共同依赖的部分（通常是消息、接口、常量）抽成第三个包，让 A 和 B 都依赖它。

---

## 9. 交叉编译到 ARM

### 9.1 为什么要

目标板（如 RK3588 这类 ARM64 板卡）算力不如开发机，在板上编译慢、占内存、不便做持续集成。于是在 x86 开发机上**交叉编译**：用能产出 ARM 指令的编译器，链接 ARM 版的库。

### 9.2 三个概念

- **host**：运行编译器的机器（x86）；
- **target**：运行产物的机器（aarch64）；
- **sysroot**：一份目标板文件系统的副本（含目标架构的头文件和库）。链接时必须用它，而不是宿主机的 `/usr/lib`。

### 9.3 工具链文件骨架（结构示意，未在本机运行）

```cmake
# toolchain-aarch64.cmake
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

set(CMAKE_C_COMPILER   aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)

set(CMAKE_SYSROOT /opt/sysroot-aarch64)
set(CMAKE_FIND_ROOT_PATH ${CMAKE_SYSROOT})

# 程序在宿主机找，库/头文件/包只在 sysroot 里找
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
```

使用：

```bash
cmake -S . -B build_arm -DCMAKE_TOOLCHAIN_FILE=toolchain-aarch64.cmake
```

在 colcon 里则通过 `--cmake-args -DCMAKE_TOOLCHAIN_FILE=...` 传入。ROS2 工作区交叉编译还要面对：依赖的 ROS 包也要有 ARM 版本（sysroot 里要有 ROS2 的 aarch64 安装），Python 包、消息生成工具（运行在宿主机）与产物（运行在目标）之分。这一整套的细节与官方推荐流程**未核对**，真正动手前要查所用版本的文档。

### 9.4 常见错误

| 现象 | 原因 |
|---|---|
| 拷到板子上运行报 `Exec format error` | 产出的是 x86 二进制，工具链没生效 |
| 运行报 `GLIBC_2.xx not found` | sysroot 的 libc 比目标板新，运行时找不到对应版本 |
| 链接时拿到宿主机的库 | `FIND_ROOT_PATH_MODE` 没设 `ONLY`，混用了宿主机库 |
| `error while loading shared libraries` | 目标板缺动态库或 `LD_LIBRARY_PATH` 不对 |

可以用 `file <产物>` 确认架构（应显示 ARM aarch64）。

---

## 10. 常见错误与排查

| 现象 | 阶段 | 常见原因 | 处理 |
|---|---|---|---|
| `fatal error: xxx.hpp: No such file or directory` | 编译 | 没 `find_package`/没链接目标；依赖可见性写成 `PRIVATE`；`package.xml` 漏依赖 | 补依赖，检查 `PUBLIC` |
| `undefined reference to ...` | 链接 | 没链接库；顺序不对；签名/命名空间不一致；忘了把 `.cpp` 加入目标 | 查目标的源文件列表与 `target_link_libraries` |
| `Could not find a package configuration file provided by "X"` | 配置 | 包未安装；前缀路径未设置；未 source | `rosdep install`；`source`；检查 `CMAKE_PREFIX_PATH` |
| `ros2 run` 找不到可执行文件 | 运行 | 没 source；没装到 `lib/<包名>/`；Python 缺 `setup.cfg` 脚本目录 | 检查 install 规则 |
| launch 找不到 YAML/launch | 运行 | 没把 `launch/ config/` 安装到 `share/<包名>` | `install(DIRECTORY ...)` 或 `data_files` |
| 改了消息但下游行为不对 | 运行 | 下游没重新编译 | `--packages-above`，清洁构建 |
| 改了源码没生效 | 运行 | 没重编；`--symlink-install` 对 C++ 不适用 | 重编 |
| 构建很慢 | 构建 | 单线程、无缓存、关键路径过长 | 并行、ccache、拆包 |
| 本机能编，CI 不能 | 构建 | 隐含依赖（本机装过）、版本不同 | 在干净容器里 `rosdep` 复现 |
| 性能异常差 | 运行 | 没设 `Release` | `-DCMAKE_BUILD_TYPE=Release` |
| 循环依赖 | 配置 | 包间互相引用 | 抽公共接口包 |

**排查顺序建议**：① 判定阶段（配置/编译/链接/运行）；② 读第一条错误，不是最后一条（后面多是连锁）；③ 在干净环境里复现；④ 二分法缩小范围（删除一半包、一半源文件）。

---

## 11. 在七个项目里怎么用（含诚实边界）

| 项目 | 项目里涉及的构建 | 与本章的关系 | 性质 |
|---|---|---|---|
| [P5](../../01_项目课程/P5_车式移动机器人自主导航/01_系统总览与导航栈分层.md) | ROS2 Humble 导航栈由多个包组成；CV 提到 ROS 与算法链 | 包/依赖/覆盖层的概念直接适用；使用 Nav2 时的 colcon 工作区与 overlay 就是 §7 | 项目做过 ROS2 工程；具体 CMake 技巧是通用知识 |
| [P2](../../01_项目课程/P2_未来动力双臂遥操作与差速底盘/10_ROS2_CAN执行链.md) | ROS2-CAN 执行链，C++/Python 混合的可能性 | 执行链硬件节点常为 C++ 包，上层脚本为 Python 包；消息包独立（§5.3） | 项目做过 ROS2 执行链；包结构细节以项目文档为准 |
| [P1](../../01_项目课程/P1_AIRS球形机器人平衡控制/08_ROS_CAN安全执行闭环.md) | ROS 状态处理与 CAN 电机执行 | 同上；存量代码为 ROS1 风格，ROS1 用 `catkin`，ROS2 用 `ament`——概念对应见 [第 04 章](./04_ROS2_Humble核心概念与ROS1对比.md) | 项目做过；面试按概念讲 |
| [P6](../../01_项目课程/P6_深开鸿M-Robots平台软件/02_OpenHarmony基础_架构_HDF_IPC_构建.md) | OpenHarmony 的构建体系（GN/Ninja 一类，未核对），与 ROS 的 CMake 不同 | 思想相通（目标、依赖、安装），工具不同 | 项目内容；与 ament 是不同体系 |
| P3、P7 | STM32 嵌入式工程用 Keil/CMake/Makefile 等 | 交叉编译思想（宿主机与目标）完全一致 | 方法类比 |
| P4 | RK3588 与 Hi3861 的构建 | RK3588 上的编译/交叉编译与 §9 对应 | 方法类比 |
| 交叉编译 ROS2 到 ARM 的完整流程 | CV 没有写 | — | **延伸，非项目所做** |

> **面试要点**：被问到构建系统时，不要只说"我用 colcon build"。更好的回答是："我按包的依赖关系组织代码，把纯逻辑核心做成库、ROS 外壳做成薄层，接口放独立的消息包，这样既能在没有 ROS 的环境下测试，又避免循环依赖。"

---

## 12. 面试怎么问、怎么答

**Q1. What is the difference between PUBLIC, PRIVATE and INTERFACE in CMake?**

> They describe how a dependency propagates. PRIVATE means the dependency is only used inside the target's own source files, so consumers do not inherit it. PUBLIC means the target's public headers expose it, so both the target and its consumers need it. INTERFACE means only the consumers need it, which fits header-only libraries. A simple test is whether my public header includes the dependency's header or uses its types. If it does, it is PUBLIC.

中文：描述依赖如何传播。PRIVATE：只在目标自身源文件里用，下游不继承；PUBLIC：公开头文件暴露了它，目标自身与下游都需要；INTERFACE：只有下游需要，如纯头文件库。简单判断：公开头文件是否包含了依赖的头文件或使用其类型；是则 PUBLIC。

*Follow-up: What goes wrong if you mark everything PUBLIC?*

> Dependencies leak to every downstream target, builds get slower, and unrelated packages become coupled, so a change in a low-level dependency forces a rebuild of everything above it.

中文：依赖泄漏到所有下游，构建变慢，不相关的包被耦合，底层依赖一变就要重编上层所有东西。

**Q2. What are `depend`, `build_depend`, `exec_depend` and `build_export_depend` in package.xml?**

> `build_depend` is needed at compile time. `exec_depend` is needed at run time. `build_export_depend` is needed when downstream packages compile against my headers, which corresponds to PUBLIC in CMake. `depend` is shorthand for all three and is what I use most of the time. Tools read it: colcon uses it for the build order, and rosdep uses it to install system packages.

中文：`build_depend` 编译时需要；`exec_depend` 运行时需要；`build_export_depend` 在下游针对我的头文件编译时需要（对应 CMake 的 PUBLIC）；`depend` 是三者的简写，最常用。工具读取它：colcon 用来排构建顺序，rosdep 用来安装系统包。

**Q3. `colcon build` succeeds but `ros2 run` says the executable is not found. What do you check?**

> First, whether I sourced the workspace's install setup file in this shell. Second, whether the executable is installed into lib slash package name, because that is where ros2 run looks. For a Python package, I check the entry points and that setup.cfg puts scripts in that directory. Third, whether another workspace earlier in the prefix path is shadowing my package; ros2 pkg prefix shows which one is resolved.

中文：先看这个 shell 里是否 source 了工作区的 install setup；其次可执行文件是否装到 `lib/<包名>`（`ros2 run` 找这里）；Python 包则看 entry_points 与 setup.cfg 是否把脚本放到该目录；再看前缀路径里更靠前的其他工作区是否遮蔽了我的包，`ros2 pkg prefix` 可显示解析到哪一个。

**Q4. How would you make a ROS 2 package cross-compile for an ARM board?**

> I would use a CMake toolchain file that sets the target system and processor, the cross compilers, and a sysroot that is a copy of the target's libraries. I set the find-root-path modes so that libraries and headers are searched only in the sysroot while programs are searched on the host. Then I pass the toolchain file through colcon's CMake arguments, and verify the output with the file command. The harder part is that all dependent ROS packages also need aarch64 builds in the sysroot, so in practice I might build natively in an ARM container instead, if the speed is acceptable.

中文：用 CMake 工具链文件设置目标系统/处理器、交叉编译器和 sysroot（目标板库的副本）；设置 find-root-path 模式使库和头文件只在 sysroot 中查找、程序在宿主机找；通过 colcon 的 CMake 参数传入，并用 `file` 命令验证产物架构。难点是所有 ROS 依赖也要有 aarch64 版本放进 sysroot，所以速度可接受时我可能改用 ARM 容器原生构建。

**Q5. Why do you split a package into a core library and a ROS node?**

> The core library has the pure logic, such as limiting, state machines and filters, and does not include ROS headers. That means I can unit-test it quickly without a ROS context, reuse it from other nodes or from non-ROS code, and keep the node as a thin shell for subscriptions, timers and parameters. It also makes the dependency graph cleaner, since the interface package has no implementation dependencies.

中文：核心库放纯逻辑（限幅、状态机、滤波），不含 ROS 头文件，所以可无需 ROS 上下文快速单测，也能被其他节点或非 ROS 代码复用；节点只做订阅、定时器、参数的薄壳。依赖图也更干净，接口包不带实现依赖。

---

## 13. 本章要点速记

1. 四层分工：colcon（工作区）→ ament（ROS 包约定）→ CMake（编译）→ rosdep（系统依赖）。
2. 现代 CMake 以目标为中心；信息挂在目标上并随依赖传递；源码外构建。
3. `PUBLIC` = 公开头文件暴露了依赖；`PRIVATE` = 仅实现用；`INTERFACE` = 仅下游用。
4. 带命名空间的导出目标（`pkg::lib`）能在配置阶段就发现拼写错误。
5. `undefined reference` 是链接阶段缺定义；`No such file` 是编译阶段缺声明；`Could not find a package configuration file` 是配置阶段缺包。
6. `package.xml`：`depend` = build + export + exec；`build_export_depend` 对应 PUBLIC；`rosdep` 一条命令装齐系统依赖。
7. ament_cmake：可执行文件装到 `lib/<包名>`，launch/config 装到 `share/<包名>`，`ament_package()` 放最后。
8. ament_python：`data_files` 要列出 launch/config，`resource/<包名>` 标记文件，`entry_points` 提供入口。
9. overlay 覆盖 underlay；`AMENT_PREFIX_PATH` 顺序决定解析；`ros2 pkg prefix` 可查；改接口包后重编下游。
10. 构建时间下限是关键路径（示例 255 s，串行 325 s），增加并行度到一定程度不再有用；性能测试用 Release。

## 14. 自测题

1. 库 A 的头文件里使用了 Eigen 的类型，A 的 `.cpp` 里使用了 yaml-cpp。分别应当用什么可见性链接？（答案：Eigen `PUBLIC`，yaml-cpp `PRIVATE`。）
2. 一个包依赖图：`a` 无依赖（耗时 20），`b` 依赖 `a`（30），`c` 依赖 `a`（50），`d` 依赖 `b,c`（10）。串行时间、关键路径、两核最短完成时间各是多少？（答案：110；$20+50+10=80$；80。）
3. `ros2 launch` 报找不到 `chassis.yaml`，但文件在源码的 `config/` 目录。可能原因和修复？（答案：没安装到 `share/<包名>/config`；加 `install(DIRECTORY config DESTINATION share/${PROJECT_NAME})` 或 `data_files`。）
4. 为什么消息定义最好放在独立的接口包？（要点：避免循环依赖，下游只依赖接口而不依赖实现。）
5. 拷到 ARM 板上的程序报 `Exec format error`，下一步查什么？（答案：`file` 命令看产物架构，检查工具链文件是否生效。）

## 15. 相关链接

- 上一章：[10 仿真平台：Gazebo、Isaac、MuJoCo 对比](./10_仿真平台_Gazebo_Isaac_MuJoCo对比.md)
- 下一章：[12 Docker 与部署：交叉编译、systemd](./12_Docker与部署_交叉编译_systemd.md)
- 相关：[02 C++ 现代特性与工程实践](./02_C++现代特性与工程实践.md)、[09 ROS2 测试与 CI](./09_ROS2测试与CI_launch_testing_gtest_rosbag回放.md)、[04 ROS2 Humble 核心概念](./04_ROS2_Humble核心概念与ROS1对比.md)
- 项目：[P5 系统总览](../../01_项目课程/P5_车式移动机器人自主导航/01_系统总览与导航栈分层.md)、[P2 ROS2-CAN 执行链](../../01_项目课程/P2_未来动力双臂遥操作与差速底盘/10_ROS2_CAN执行链.md)
- [模块 README](./README.md)、[基础知识总览](../00_总览与学习路线.md)
