# Python 工程与科学计算

> **一句话定位**：Python 是机器人研发里的"胶水与实验室"：仿真（MuJoCo/Mink）、数据分析、优化与标定、Agent 服务端（LangGraph）、原型验证都靠它。这一章不讲语法入门，而是讲**把 Python 用对、用快、用稳**所需的核心：数据模型、GIL、asyncio、numpy 向量化、scipy 的优化与线性代数（含用 `solve_continuous_are` 算 LQR）、matplotlib 作图、类型标注与打包。

**本章学完你能：**

- 解释"名字绑定对象""可变/不可变""可变默认参数陷阱"，并写出 dunder 方法、上下文管理器、生成器、装饰器、`dataclass`；
- 说清 GIL 是什么、什么情况下线程有用、什么时候必须多进程；
- 用 `asyncio` 写并发 I/O 程序，手追任务时间线，识别阻塞调用把事件循环卡死的错误；
- 用 numpy 向量化代替循环，掌握广播规则、视图与拷贝、轴（axis）；
- 用 `scipy.optimize` 做最小二乘/最小化/求根，手算一步 Gauss-Newton；用 `scipy.linalg` 解线性方程、特征值、矩阵指数、Riccati 方程（LQR）；
- 用 matplotlib 的面向对象接口画出可发表质量的图；
- 写带类型标注的代码，组织项目并打包。

**前置知识：**基本 Python 语法；线性代数（[线性代数](../01_数学基础/01_线性代数.md)）；最优化基础（[最优化与二次规划](../01_数学基础/04_最优化与二次规划.md)）。

**相关项目：**[P2](../../01_项目课程/P2_未来动力双臂遥操作与差速底盘/README.md)（MuJoCo/Mink 约束逆运动学、KDL/URDF 脚本）；[P4](../../01_项目课程/P4_OpenHarmony智能园艺系统/README.md)（LangGraph 决策链、服务端、网页交互）；也广泛用于 P1/P5/P7 的离线分析与调参。

## 目录

1. Python 数据模型
2. GIL 与并发选择
3. asyncio
4. numpy 向量化
5. scipy.optimize
6. scipy.linalg 与 LQR
7. matplotlib
8. 类型标注
9. 项目组织与打包
10. 常见错误与故障现象
11. 面试问答
12. 要点速记、自测题、链接

---

## 1. Python 数据模型

### 1.1 一切皆对象，变量只是名字

`x = [1, 2]` 不是"把列表放进 x 这个盒子"，而是**创建一个列表对象，让名字 `x` 指向它**。赋值 `y = x` 只是让 `y` 也指向同一个对象。

```python
a = [1, 2]
b = a            # b 与 a 指向同一对象
b.append(3)
print(a)         # [1, 2, 3]  —— a 也变了
c = a.copy()     # 新对象（浅拷贝）
c.append(4)
print(a)         # [1, 2, 3]  —— 不受影响
```

- **可变（mutable）**：`list`、`dict`、`set`、`bytearray`、自定义对象——原地可修改；
- **不可变（immutable）**：`int`、`float`、`str`、`tuple`、`frozenset`、`bytes`——"修改"实际是创建新对象；
- `==` 比较**值**，`is` 比较**是否同一对象**（`is None` 用于判空，比较数值不要用 `is`）；
- **浅拷贝 vs 深拷贝**：`copy.copy` 只复制外层，内层对象仍共享；`copy.deepcopy` 递归复制。`[[0]*3]*3` 会创建 3 行**同一个**内层列表，改一行三行都变——应写 `[[0]*3 for _ in range(3)]`。

### 1.2 可变默认参数陷阱（手追）

```python
def add(x, buf=[]):      # 默认值只在函数定义时创建一次
    buf.append(x)
    return buf

add(1)   # [1]
add(2)   # [1, 2]   —— 与预期的 [2] 不同！
```

**原因**：`buf=[]` 在 `def` 执行时求值一次，之后所有调用共享同一个列表对象。**修复**：

```python
def add(x, buf=None):
    if buf is None:
        buf = []
    buf.append(x)
    return buf
```

### 1.3 dunder（魔术）方法：让自定义类像内建类型

| 方法 | 触发 | 用途 |
|---|---|---|
| `__init__` | 构造后初始化 | |
| `__repr__` / `__str__` | `repr(x)` / `print(x)` | 调试可读性 |
| `__eq__` / `__hash__` | `==` / 作为字典键 | 定义 `__eq__` 后若可变应不可哈希 |
| `__len__` / `__getitem__` / `__iter__` | `len(x)` / `x[i]` / `for` | 序列/容器协议 |
| `__call__` | `x(...)` | 可调用对象（如控制器 `ctrl(err)`） |
| `__enter__` / `__exit__` | `with` | 上下文管理器 |
| `__add__` / `__mul__` | `+` / `*` | 运算符重载 |

### 1.4 上下文管理器 = Python 的 RAII

```python
from contextlib import contextmanager

@contextmanager
def can_bus(ifname):
    bus = open_bus(ifname)
    try:
        yield bus              # with 块在这里执行
    finally:
        bus.close()            # 无论是否异常都执行

with can_bus("can0") as bus:
    bus.send(frame)
```

对应 C++ 的 [RAII](./02_C++现代特性与工程实践.md)：确保资源释放、锁释放。

### 1.5 迭代器与生成器

`for` 调用 `iter()` 得到迭代器，反复 `next()` 直到 `StopIteration`。**生成器**用 `yield` 写出惰性序列，不一次性占满内存：

```python
def read_chunks(f, n=4096):
    while chunk := f.read(n):      # 海象运算符 :=
        yield chunk
```

**手算**：处理 10 GB 日志，若 `readlines()` 一次读入内存会爆；生成器每次只持有 4 KB，内存占用恒定。列表推导式 `[f(x) for x in xs]` 立即求值，生成器表达式 `(f(x) for x in xs)` 惰性。

### 1.6 装饰器

装饰器是接受函数返回函数的函数，用于计时、重试、日志、缓存：

```python
import time, functools

def timed(fn):
    @functools.wraps(fn)            # 保留原函数名与文档
    def wrapper(*args, **kwargs):
        t0 = time.perf_counter()    # 单调高精度计时器
        try:
            return fn(*args, **kwargs)
        finally:
            print(f"{fn.__name__}: {(time.perf_counter()-t0)*1e3:.2f} ms")
    return wrapper

@timed
def solve_ik(...): ...
```

### 1.7 dataclass：数据结构的首选

```python
from dataclasses import dataclass, field

@dataclass(slots=True)
class JointState:
    pos: float
    vel: float = 0.0
    stamp: float = 0.0               # 单位：秒
    names: list[str] = field(default_factory=list)   # 可变默认值要用 default_factory
```

自动生成 `__init__`、`__repr__`、`__eq__`。`frozen=True` 得到不可变对象。

---

## 2. GIL 与并发选择

### 2.1 GIL 是什么

CPython 有一把**全局解释器锁（Global Interpreter Lock）**：同一时刻只有一个线程在执行 Python 字节码。原因：CPython 用**引用计数**管理内存，计数增减若无保护，多线程会数据损坏；一把全局锁最简单。

后果：

- **CPU 密集型**任务用多线程**不会更快**（反而因切换更慢）；
- **I/O 密集型**任务（网络、文件、串口、`sleep`）线程在等待 I/O 时会释放 GIL，所以线程有用；
- **numpy/scipy 的底层 C 计算通常释放 GIL**，多线程可以并行大数组运算；
- 较新的 Python 版本提供了实验性的"无 GIL"构建，但生态兼容仍在发展，面试时知道这件事即可，不要过度承诺。

### 2.2 手算对比

4 个任务，每个纯 Python 计算 1 s，机器 4 核：

| 方案 | 总耗时（理想） |
|---|---|
| 串行 | 4 s |
| 4 线程（CPU 密集） | ≈ 4 s（受 GIL，甚至略慢） |
| 4 进程（`multiprocessing`/`ProcessPoolExecutor`） | ≈ 1 s + 进程启动与数据序列化开销 |
| 4 线程（每个任务 1 s 都在 `sleep`/网络等待） | ≈ 1 s |
| `asyncio` 4 个协程各 `await sleep(1)` | ≈ 1 s（单线程） |

### 2.3 选择指南

| 场景 | 选择 |
|---|---|
| 大量并发网络 I/O（WebSocket、HTTP、MQTT） | `asyncio`（单线程，开销小，数千连接） |
| 少量阻塞 I/O（串口、第三方阻塞库） | 线程 / `asyncio.to_thread` |
| 纯 Python CPU 密集 | 多进程 |
| 数值计算 | numpy 向量化 / 调用 C/C++ 扩展 |
| 实时控制（硬实时） | **不要用 Python**；用 C/C++（Python 的 GC 与 GIL 带来不确定延迟） |

> **面试要点**：能说出"Python 适合在上层做编排与实验，高频底层闭环放 C++/MCU"，与项目中把高频力矩环放在 MCU/驱动器、ROS2 里放低频参考的分工一致（见 [CAN 章 13.2](../11_嵌入式与通信/03_CAN_FDCAN与电机通信.md)）。

---

## 3. asyncio

### 3.1 它要解决什么问题

一个服务端要同时处理很多连接（每个客户端一条 WebSocket），大部分时间在**等对方发数据**。每个连接一个线程太重（内存、切换）。**asyncio** 用**单线程 + 事件循环 + 协程**：协程遇到 `await`（需要等）就把控制权交还事件循环，循环去运行别的已就绪协程；等的事情完成再回来继续。这是**协作式**并发：靠 `await` 让出，没有抢占。

### 3.2 核心概念

- **协程函数**：`async def f(): ...`；调用它**不会执行**，只返回一个协程对象；
- **`await x`**：等待可等待对象（协程、Task、Future）完成，期间让出控制权；只能在 `async def` 里用；
- **事件循环（event loop）**：调度中心，管理 I/O 就绪事件、定时器、就绪协程；
- **Task**：`asyncio.create_task(coro)` 把协程包装成任务**立刻开始并发运行**（在下次让出时）；
- `asyncio.run(main())`：创建循环、运行 `main`、结束时清理。

### 3.3 手追时间线

```python
import asyncio, time

async def job(name, secs):
    print(f"{time.perf_counter()-t0:4.1f}s  {name} start")
    await asyncio.sleep(secs)              # 让出，不阻塞循环
    print(f"{time.perf_counter()-t0:4.1f}s  {name} done")
    return name

async def main():
    r = await asyncio.gather(job("A", 2), job("B", 1), job("C", 3))
    print(r)

t0 = time.perf_counter()
asyncio.run(main())
```

时间线：

| t (s) | 事件 |
|---|---|
| 0.0 | A start（遇 await，让出）→ B start → C start |
| 1.0 | B 的 sleep 完成：B done |
| 2.0 | A done |
| 3.0 | C done；`gather` 返回 `["A","B","C"]`（顺序按传入顺序，不按完成顺序） |

总耗时 3 s（= 最长的那个），而顺序执行 `await job(...)` 三次是 2+1+3 = 6 s。

### 3.4 常用 API

```python
task = asyncio.create_task(job("X", 1))          # 并发运行；必须保存引用，避免被垃圾回收
res = await asyncio.wait_for(job("Y", 5), 2.0)   # 超时抛 asyncio.TimeoutError
done = await asyncio.gather(*coros, return_exceptions=True)  # 收集异常而非中断
q = asyncio.Queue(maxsize=100)                    # 协程间队列；await q.put(x) / await q.get()
async with asyncio.timeout(3): ...                # Python 3.11+
```

取消：`task.cancel()` 会在任务的下一个 `await` 点抛出 `CancelledError`；清理逻辑放 `finally`。

### 3.5 最大的坑：在协程里调用阻塞函数

```python
async def bad():
    time.sleep(2)              # 阻塞整个事件循环！所有协程停摆 2 s
    requests.get(url)          # 同步网络库，同样阻塞
    heavy_numpy_call()         # 长计算，同样卡住

async def good():
    await asyncio.sleep(2)                       # 非阻塞
    data = await asyncio.to_thread(requests.get, url)   # 把阻塞调用放线程池
    res = await loop.run_in_executor(pool, cpu_heavy)    # 把 CPU 密集放进程池
```

**手算**：100 个客户端的 WebSocket 服务，每次处理里有一个 `time.sleep(0.01)`（阻塞 10 ms）。每个请求阻塞整个循环，吞吐上限 = 1/0.01 = 100 请求/秒，且其它连接的延迟被迫串行叠加；换成 `await asyncio.sleep(0.01)` 后，100 个请求可交错并发，总耗时 ≈ 10 ms。

### 3.6 asyncio vs 线程 vs 多进程

| | asyncio | 线程 | 多进程 |
|---|---|---|---|
| 并发模型 | 协作式，单线程 | 抢占式（受 GIL） | 真并行 |
| 开销 | 很小（数千～数万并发） | 中（MB 级栈） | 大 |
| 共享状态 | 单线程内天然无竞争（两个 await 之间不被打断） | 要加锁 | 需 IPC |
| 适合 | 大量 I/O 并发 | 少量阻塞调用 | CPU 密集 |
| 风险 | 阻塞调用卡死循环；忘记 await | 数据竞争 | 序列化开销 |

注意"单线程内无竞争"只在两个 `await` 之间成立；跨 `await` 的"检查再使用"（check-then-act）仍可能被别的协程插入，需要 `asyncio.Lock`。

### 3.7 在项目里怎么用

P4：服务端 Agent 与网页之间的 WebSocket 推送（观察结果、系统状态更新）典型使用 asyncio——一个事件循环同时处理多个客户端连接，并在收到视觉事件时向前端广播。LangGraph 的节点可以是异步函数（`async def`），调用 LLM/HTTP 的等待时间不阻塞其它请求。本项目的设计是：WebSocket 用 websockets 库（端口 8765，静态页另走 HTTP 8000），一个 asyncio 事件循环处理所有客户端；视觉事件进入 `asyncio.Queue`（maxsize=100，满则丢最旧并计数），LangGraph 节点按需写成 `async def`，等待 LLM/HTTP 时不阻塞其它请求。

---

## 4. numpy 向量化

### 4.1 为什么

Python 循环每次迭代有解释器开销（约几十到上百纳秒的量级），numpy 把整个数组运算交给 C 的紧凑循环（且常用 SIMD）。典型加速 10–100 倍。

### 4.2 ndarray 的基本概念

- `shape`（各维长度）、`dtype`（元素类型，`float64`/`float32`/`int16`）、`strides`（每维跨越的字节数）；
- 内存连续的数据 + 元数据；**切片返回视图（view）**，共享内存，改视图会改原数组；`copy()` 才是新数据。

```python
import numpy as np
a = np.arange(6).reshape(2, 3)    # [[0 1 2],[3 4 5]]
v = a[:, 1]                       # 视图：[1, 4]
v[0] = 99                         # a[0,1] 也变成 99
b = a[:, [0, 2]]                  # 花式索引 -> 拷贝
```

`axis`：沿哪个维度**消去**。`a.sum(axis=0)` 对行方向求和（每列一个结果，shape (3,)），`a.sum(axis=1)` 每行一个结果。

### 4.3 广播（broadcasting）规则（手算）

两个数组运算时，从**最后一维**向前对齐：每一维要么相等，要么其中一个是 1（会被"复制拉伸"），或其中一个缺这一维（视为 1）。

例 1：`(3,1) + (1,4)` → 对齐：3 vs 1 → 3；1 vs 4 → 4 → 结果 `(3,4)`。

```python
x = np.array([[1],[2],[3]])        # (3,1)
y = np.array([[10,20,30,40]])      # (1,4)
x + y   # [[11 21 31 41],[12 22 32 42],[13 23 33 43]]
```

例 2：`(3,)` 与 `(3,1)` 相加：对齐为 `(1,3)` vs `(3,1)` → 结果 `(3,3)`——**新手最容易意外得到一个矩阵**。

例 3：`(2,3)` 与 `(2,)`：最后一维 3 vs 2 不相容 → 报错。要让它按行加，应写 `a + v[:, None]`（把 `v` 变成 `(2,1)`）。

### 4.4 机器人中的向量化范例

**批量旋转点**：2D 旋转矩阵 $R$（2×2）作用于 $N$ 个点（形状 $(N,2)$）：

```python
th = np.deg2rad(90)
R = np.array([[np.cos(th), -np.sin(th)],
              [np.sin(th),  np.cos(th)]])
P = np.array([[1, 0], [0, 1], [2, 3]])      # (3,2)
Q = P @ R.T                                  # (3,2)：每行是一个点，右乘 R 的转置
```

手算：$R(90°)=\begin{bmatrix}0&-1\\1&0\end{bmatrix}$。点 (1,0)→(0,1)；(0,1)→(−1,0)；(2,3)→(−3,2)。✓ （这里行向量写法要用 $R^T$：$q=Rp \Leftrightarrow q^T=p^TR^T$。）

**点到点距离矩阵**（一次算出全部，无需双重循环）：

```python
A = np.random.rand(1000, 2)   # (1000,2)
B = np.random.rand(500, 2)    # (500,2)
D = np.linalg.norm(A[:, None, :] - B[None, :, :], axis=2)   # (1000,500)
```

手算 shape：`(1000,1,2) - (1,500,2)` → `(1000,500,2)`，对最后一维求范数 → `(1000,500)`。内存：1000×500×2×8 B = 8 MB 中间数组。

**避免循环计算误差**：`np.cumsum`、`np.diff`、`np.interp`（插值）、`np.unwrap`（角度展开，类似 [电机章 6.2](../11_嵌入式与通信/05_电机_编码器_驱动与力矩控制.md) 的多圈展开）都是向量化工具。

### 4.5 线性代数与数值注意

- 矩阵乘 `A @ B`（不是 `*`，`*` 是逐元素）；`np.linalg.solve(A, b)` 解 $Ax=b$，**不要用 `inv(A) @ b`**（慢且数值差）；
- `np.linalg.lstsq(A, b, rcond=None)`：最小二乘；`pinv`：伪逆（IK 的阻尼最小二乘可基于它，见 [奇异性、阻尼最小二乘](../03_运动学与坐标变换/04_奇异性_阻尼最小二乘_零空间.md)）；
- 浮点比较用 `np.allclose(a, b, atol=1e-9)`，不用 `==`；
- `float32` 约 7 位有效数字，`float64` 约 15–16 位；累加大量小数值用 `float64`；
- 就地操作（`a += b`）省内存，但要注意视图共享与 dtype 提升。

### 4.6 手算：向量化的收益

计算 $10^6$ 个点的 $\sqrt{x^2+y^2}$。纯 Python 循环每次约 0.2–0.5 μs 的解释器开销之外还有函数调用，总计约 0.3–0.5 s 量级；numpy `np.hypot(x, y)` 约几毫秒（量级估计，实际依机器而异，应以 `timeit` 实测为准）。面试答案要强调"向量化减少解释器开销、利用连续内存与 SIMD"。

---

## 5. scipy.optimize

### 5.1 它要解决什么问题

机器人里到处是"找参数使某个误差最小"：标定相机/IMU、辨识电机摩擦、拟合轨迹、求逆运动学、调控制器参数。`scipy.optimize` 提供通用求解器。

### 5.2 常用函数

| 函数 | 问题 | 例子 |
|---|---|---|
| `least_squares(fun, x0, ...)` | 非线性最小二乘 $\min\sum r_i(x)^2$，支持边界、鲁棒损失（`loss='huber'`） | 标定、曲线拟合 |
| `curve_fit(f, x, y, p0)` | 拟合模型 $y=f(x;p)$（封装了上者） | 摩擦模型辨识 |
| `minimize(fun, x0, method=...)` | 通用标量函数最小化，支持约束（`SLSQP`、`trust-constr`）、`L-BFGS-B` 带界 | 小规模 NLP |
| `root` / `brentq` / `fsolve` | 求根 | 解非线性方程 |
| `linprog` | 线性规划 | |
| `milp` | 混合整数线性规划 | |

> 稠密凸 QP 通常用专用库（OSQP、qpOASES、cvxpy），`minimize` 作原型即可；见 [最优化与二次规划](../01_数学基础/04_最优化与二次规划.md)。MPC 实际求解多用专用 QP 求解器。

### 5.3 手算：Gauss-Newton 一步（理解 `least_squares` 的内核）

模型 $y=e^{kx}$，数据 $(x,y)$：(1, 2.8)，(2, 7.5)。初值 $k_0=1$。

1. 预测：$e^{1}=2.718$，$e^{2}=7.389$；
2. 残差 $r_i=\hat y_i-y_i$：$r_1=2.718-2.8=-0.082$，$r_2=7.389-7.5=-0.111$；
3. 雅可比 $J_i=\partial\hat y_i/\partial k=x_ie^{kx_i}$：$J_1=2.718$，$J_2=2\times7.389=14.778$；
4. 正规方程 $J^TJ\,\Delta k=-J^Tr$：$J^TJ=2.718^2+14.778^2=7.389+218.39=225.78$；$J^Tr=2.718\times(-0.082)+14.778\times(-0.111)=-0.2229-1.6404=-1.8633$；
5. $\Delta k=1.8633/225.78=0.00825$，$k_1=1.0083$。

迭代下去 $k\to1.008$ 附近收敛（用 $\ln7.5/2=1.0075$ 和 $\ln2.8=1.0296$ 的折中验证合理）。**要点**：Gauss-Newton 用 $J^TJ$ 近似 Hessian；Levenberg-Marquardt 在其上加阻尼 $\lambda I$（与 IK 的阻尼最小二乘是同一思想）。

```python
from scipy.optimize import least_squares
import numpy as np

x = np.array([1.0, 2.0]); y = np.array([2.8, 7.5])
res = least_squares(lambda k: np.exp(k[0]*x) - y, x0=[1.0])
print(res.x, res.cost)        # cost = 0.5 * sum(residual^2)
```

### 5.4 实战要点

- 初值很重要：非凸问题可能陷入局部最优；
- 尺度：参数量级差异大（如 $10^{-3}$ 与 $10^{3}$）时要归一化，或用 `x_scale='jac'`；
- 提供解析雅可比可显著加速并提高精度；
- 检查 `res.success`、`res.status`、`res.message`，不要只看 `res.x`；
- 鲁棒损失（Huber/soft_l1）抵抗离群点（如标定时的坏数据）。

---

## 6. scipy.linalg 与 LQR

### 6.1 常用

| 函数 | 用途 |
|---|---|
| `solve(A, b, assume_a='pos')` | 解线性方程（对称正定可加速） |
| `lstsq`、`pinv` | 最小二乘 / 伪逆 |
| `eig`、`eigh`（对称） | 特征值分解，判断稳定性（连续系统：实部全 <0） |
| `svd` | 奇异值分解，条件数、秩、奇异性诊断 |
| `cholesky`、`qr`、`lu` | 分解 |
| `expm(A*dt)` | 矩阵指数，精确离散化 $A_d=e^{A\,dt}$ |
| `solve_continuous_are(A,B,Q,R)` | 连续代数 Riccati 方程（LQR） |
| `solve_discrete_are(A,B,Q,R)` | 离散 Riccati |
| `solve_continuous_lyapunov` | Lyapunov 方程 |

### 6.2 LQR 的最小可手算例子

系统 $\dot x=ax+bu$，代价 $J=\int(qx^2+ru^2)dt$。标量 Riccati 方程：

$$2aP-\frac{b^2}{r}P^2+q=0,\qquad K=\frac{bP}{r},\qquad u=-Kx$$

取 $a=1$（不稳定），$b=1$，$q=1$，$r=1$：$P^2-2P-1=0\Rightarrow P=1+\sqrt2=2.414$；$K=2.414$；闭环 $a-bK=1-2.414=-1.414<0$，稳定 ✓。

```python
import numpy as np
from scipy.linalg import solve_continuous_are

A = np.array([[1.0]]); B = np.array([[1.0]])
Q = np.array([[1.0]]); R = np.array([[1.0]])
P = solve_continuous_are(A, B, Q, R)            # [[2.4142]]
K = np.linalg.solve(R, B.T @ P)                 # K = R^{-1} B^T P -> [[2.4142]]
print(np.linalg.eigvals(A - B @ K))             # [-1.4142]
```

多维情形下，$K=R^{-1}B^TP$，同一段代码适用于 P1 的四状态倒立摆线性化模型与 P7 的轮腿模型（A、B 矩阵来自 [倒立摆建模与线性化](../02_力学与动力学建模/03_倒立摆建模与线性化.md)）。**增益调度**（P7：按腿长调度）就是在一组腿长网格上各解一次 Riccati 方程，得到 $K(l)$ 表后插值（`np.interp`/`scipy.interpolate`）。理论见 [LQR 最优控制](../05_控制理论/04_LQR最优控制.md)。

> 项目叙述以 CV 为准：P1 用 LQR 做**分析**（权重与稳定性—控制量权衡，作为 PID 调参和饱和诊断的参考）；P7 是**增益调度 LQR**。上面代码只是通用工具演示，不代表项目实现。

### 6.3 离散化与可控性检查

```python
from scipy.linalg import expm
dt = 0.002
Ad = expm(A * dt)                 # 精确零阶保持（对 B 需增广矩阵法，或用 scipy.signal.cont2discrete）
ctrb = np.hstack([B, A@B])        # 可控性矩阵；np.linalg.matrix_rank(ctrb) == n 则可控
```

---

## 7. matplotlib

### 7.1 面向对象接口（推荐）

```python
import matplotlib
matplotlib.use("Agg")                      # 无显示环境（服务器/SSH）直接存图
import matplotlib.pyplot as plt
import numpy as np

t = np.linspace(0, 2, 2001)
fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(7, 5), constrained_layout=True)
ax1.plot(t, np.sin(2*np.pi*2*t), label="angle")
ax1.set_ylabel("angle [rad]"); ax1.legend(); ax1.grid(True)
ax2.plot(t, np.cos(2*np.pi*2*t), color="C1", label="torque")
ax2.set_xlabel("time [s]"); ax2.set_ylabel("torque [N m]"); ax2.legend(); ax2.grid(True)
fig.savefig("run01.png", dpi=150)
plt.close(fig)                              # 循环作图一定要关闭，防内存泄漏
```

### 7.2 实验图的规范

- **坐标轴必须有名称和单位**；多条曲线用不同线型/颜色并有图例；
- 多子图共享时间轴（`sharex=True`）以便对齐"指令—反馈—饱和标志"；
- 阶跃响应图标出超调、调节时间；
- 同一参数的对比实验放同一张图，并在标题或图例写出参数（如 `Kp=…`）；
- 保存时带版本或参数名，方便追溯；数据与脚本放同一目录，**图应能一键重画**；
- 大数据量（百万点）用 `ax.plot(..., rasterized=True)` 或降采样；
- 机器人调试常见三联图：位置/姿态—速度—控制输出（含饱和限值线 `ax.axhline`）。

### 7.3 常见错误

- 用全局 `plt.plot` 在多图脚本里串图；
- 忘记 `plt.close`；
- 单位混用（rad/deg）；
- 颜色不可区分（打印为灰度）；
- `plt.show()` 在无显示环境阻塞/报错。

---

## 8. 类型标注

### 8.1 为什么

Python 运行时**不检查**类型标注（它是给人和工具看的）。好处：可读、IDE 补全、`mypy`/`pyright` 在运行前发现错误、接口文档化。机器人代码里"单位和形状"的约定尤其需要写清楚。

### 8.2 常用写法（Python 3.9+/3.10+）

```python
from typing import Optional, Callable, Protocol, TypedDict, Sequence
import numpy as np
from numpy.typing import NDArray

def clamp(x: float, lo: float, hi: float) -> float: ...

def solve_ik(target: NDArray[np.float64],        # shape (6,)：位置+姿态；单位在文档字符串里写明
             q0: NDArray[np.float64],
             tol: float = 1e-4) -> Optional[NDArray[np.float64]]:   # 失败返回 None
    ...

class Controller(Protocol):                       # 结构化子类型（鸭子类型 + 类型检查）
    def step(self, err: float, dt: float) -> float: ...

Callback = Callable[[float], None]

class Obs(TypedDict):
    label: str
    conf: float
    stamp: float
```

- `X | None` 等价 `Optional[X]`（3.10+）；`list[int]`、`dict[str, float]` 可直接使用内建泛型（3.9+）；
- `Protocol` 对应 C++ 的"接口"但无需显式继承，适合 [抽象设备接口](./02_C++现代特性与工程实践.md) 的 Python 版本；
- 数组形状无法在类型里完全表达，用文档字符串和断言（`assert x.shape == (6,)`）补充。

### 8.3 工具链

`mypy --strict` / `pyright`（类型）、`ruff`（风格与常见 bug，替代 flake8/isort）、`black`（格式化）、`pytest`（测试）。放进 CI 或 pre-commit。

---

## 9. 项目组织与打包

### 9.1 虚拟环境与依赖

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt           # 或 pip install -e .[dev]
pip freeze > requirements.lock            # 锁定版本，复现实验环境
```

不同项目依赖版本冲突时，**每个项目一个虚拟环境**。ROS2 的 Python 包用系统 Python 与 `colcon`，要注意与 venv 的搭配（常用 `--system-site-packages`）。

### 9.2 推荐目录结构

```text
myproj/
├── pyproject.toml          # 项目元数据、依赖、入口点、工具配置
├── src/myproj/
│   ├── __init__.py
│   ├── kinematics.py
│   ├── control.py
│   └── cli.py
├── tests/
│   └── test_kinematics.py
├── scripts/                # 一次性脚本
└── README.md
```

### 9.3 pyproject.toml 最小示例

```toml
[build-system]
requires = ["setuptools>=68"]
build-backend = "setuptools.build_meta"

[project]
name = "myproj"
version = "0.1.0"
requires-python = ">=3.9"
dependencies = ["numpy>=1.24", "scipy>=1.10"]

[project.optional-dependencies]
dev = ["pytest", "mypy", "ruff"]

[project.scripts]
myproj-run = "myproj.cli:main"     # 安装后得到命令 myproj-run
```

`pip install -e .`（可编辑安装）：改源码立即生效，且 `import myproj` 在任何目录都能用，避免 `sys.path` 手工拼接的混乱。

### 9.4 入口与日志

```python
import logging, argparse

log = logging.getLogger(__name__)

def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--rate", type=float, default=100.0, help="Hz")
    args = ap.parse_args(argv)
    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    log.info("start at %.1f Hz", args.rate)    # 用 % 风格参数，延迟格式化
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
```

要点：用 `logging` 不用 `print`；`if __name__ == "__main__"` 防止被 import 时执行；退出码返回给 shell（见 [Linux 章 3.3](./01_Linux与Shell_Git.md)）。

### 9.5 测试

`pytest` 的 `assert` 即可；数值比较用 `pytest.approx` 或 `np.testing.assert_allclose`。对机器人算法，最有价值的测试是：已知解析解的小问题（如标量 LQR 的 $K=2.414$）、单位/符号回归测试、随机输入下的不变量（旋转矩阵 $R^TR=I$，$\det R=1$）。

---

## 10. 常见错误 / 边界 / 故障现象

| 现象 | 原因 |
|---|---|
| 函数默认参数"记住"了上次结果 | 可变默认参数 |
| 改了副本原数据也变 | 浅拷贝 / numpy 视图 |
| 多线程没提速 | GIL；CPU 密集 |
| asyncio 程序整体卡顿 | 协程里有阻塞调用（`time.sleep`、同步 `requests`、长计算） |
| 协程"没执行" | 没 `await`，只创建了协程对象；Task 引用丢失被回收 |
| numpy 结果形状不对 | 广播意外；一维数组与二维混用（`(3,)` vs `(3,1)`） |
| 数值不稳定 | 用 `inv`；`float32` 累加；比较浮点用 `==` |
| 优化不收敛 | 初值差、尺度悬殊、模型错、没提供边界 |
| 图保存后空白/内存涨 | 没 `savefig` 前 `close`；没 `close` |
| `ImportError`（项目内） | 目录没装成包，靠当前目录偶然可用 |
| 打包后缺数据文件 | 没声明 package-data |

---

## 11. 面试怎么问、怎么答

**Q1. What is the GIL, and how do you handle concurrency in Python?**

> The GIL is a lock in CPython that lets only one thread run Python bytecode at a time, mainly to keep reference counting safe. So threads don't speed up CPU-bound pure Python, but they work for I/O because the lock is released while waiting. For many network connections I use asyncio, for CPU-bound work I use multiprocessing or numpy, and for hard real-time loops I don't use Python at all.

**Q2. What happens if you call a blocking function inside an async function?**

> It blocks the whole event loop, because asyncio is cooperative and runs in one thread. Every other coroutine is stalled until the call returns. I either use an async version of the library, or move the call to a thread with asyncio.to_thread, or to a process pool if it's CPU-heavy.

追问：`gather` 与 `create_task` 区别？（gather 并发等待一组并按序返回结果；create_task 立即安排运行、需要自己 await/保存引用）；怎么取消？（`task.cancel()`，在 await 点抛 CancelledError）。

**Q3. Why vectorise with numpy, and what is broadcasting?**

> A numpy operation runs in compiled loops on contiguous memory, so it avoids the interpreter overhead per element. Broadcasting lets arrays of different shapes combine without copying: shapes are aligned from the last dimension, and each pair of dimensions must be equal or one of them must be one. I check shapes carefully, since an accidental (3,) plus (3,1) silently becomes a 3 by 3 matrix.

**Q4. Have you used scipy for optimisation or LQR?**

> Yes, as an offline tool. For example, solve_continuous_are returns the Riccati solution, from which I compute the LQR gain K equals R inverse B transpose P, and I check the closed-loop eigenvalues. For fitting a model or a calibration, least_squares with a robust loss handles outliers. I always check convergence status and the scaling of the parameters.

我们的做法是：P1 的 LQR 在第 05 章的归一化模型上离线求解（Python 用 `scipy.linalg.solve_continuous_are`，或 MATLAB 的 `lqr`），取 $Q=\operatorname{diag}(4,4,100,1)$、$R=0.25$，得到的 $K$ 只作参照与分析（解释内环增益比例、判断饱和），不直接上机，上机的是级联 PID；P7 的增益调度 LQR 是按腿长取若干网格点离线求一组 $K$，运行时按腿长插值。P2 里先用 MuJoCo/Mink 的 Python 脚本在仿真中验证 IK 任务与限位配置，再接到 ROS2 与真机。（取值为设计量级，调参时按实机微调。）

**Q5. How do you structure and share a Python project?**

> A virtual environment per project, a pyproject.toml with dependencies and an entry point, a src layout installed in editable mode, type hints checked by mypy, and tests with pytest. I log with the logging module and pin versions so experiments are reproducible.

---

## 本章要点速记

1. 变量是名字，赋值是绑定；可变对象共享需小心；默认参数别用可变对象（用 `None` 或 `default_factory`）；
2. `with` / `contextmanager` 是 Python 的 RAII；生成器惰性省内存；`dataclass` 做数据结构；
3. GIL：CPU 密集多进程，I/O 密集线程/asyncio，数值用 numpy；高频闭环不用 Python；
4. asyncio：协作式单线程；`await` 让出；`gather` 并发；阻塞调用卡死循环 → `to_thread`；
5. `gather(A=2s,B=1s,C=3s)` 总耗时 3 s 而非 6 s；
6. numpy：切片是视图；广播从最后一维对齐，`(3,)+(3,1)` 得 `(3,3)`；`solve` 优于 `inv`；浮点用 `allclose`；
7. `least_squares`：Gauss-Newton 一步 $\Delta=-(J^TJ)^{-1}J^Tr$，LM 加阻尼；检查 `success`；
8. LQR：`solve_continuous_are` → $K=R^{-1}B^TP$；标量例 $a=b=q=r=1$ 得 $K=1+\sqrt2$；
9. matplotlib 用 OO 接口，轴标签带单位，`close` 防泄漏，无显示环境用 `Agg`；
10. 类型标注不被运行时检查，靠 mypy/pyright；`Protocol` 做接口；
11. `pyproject.toml` + `pip install -e .` + venv + pytest + logging。

## 自测题

1. 下面输出什么？`def f(x, l=[]): l.append(x); return l` 连续 `f(1)`、`f(2)`、`f(3, [])`？（要点：[1]；[1,2]；[3]。）
2. 4 个协程分别 `await asyncio.sleep(1,2,3,4)` 并用 `gather`，总耗时？改成顺序 `await` 呢？（要点：4 s；10 s。）
3. `a.shape=(4,1)`，`b.shape=(3,)`，`a+b` 的形状？`a.shape=(4,2)`，`b.shape=(4,)` 呢？（要点：(4,3)；报错，需 `b[:,None]`。）
4. 对 $y=ax$ 的三个点 (1,2.1),(2,3.9),(3,6.2) 最小二乘求 $a$。（要点：$a=\sum xy/\sum x^2=28.5/14=2.036$。）
5. 标量系统 $a=2,b=1,q=1,r=1$ 的 LQR 增益？闭环极点？（要点：$P^2-4P-1=0\Rightarrow P=2+\sqrt5=4.236$；$K=4.236$；极点 $2-4.236=-2.236$。）
6. 为什么在 asyncio 里 `requests.get` 是 bug？怎么改？（要点：同步阻塞整个循环；用 `httpx.AsyncClient`/`aiohttp` 或 `asyncio.to_thread`。）
7. 为什么 `inv(A)@b` 不如 `solve(A,b)`？（要点：多做一次求逆，条件数差时误差更大，计算量更多。）

## 相关链接

- 上一章：[C++ 现代特性与工程实践](./02_C++现代特性与工程实践.md)；下一章：[ROS2 Humble 核心概念与 ROS1 对比](./04_ROS2_Humble核心概念与ROS1对比.md)
- 数学与控制：[线性代数](../01_数学基础/01_线性代数.md)、[最优化与二次规划](../01_数学基础/04_最优化与二次规划.md)、[LQR 最优控制](../05_控制理论/04_LQR最优控制.md)、[MuJoCo / Pinocchio / KDL 工具链](./06_MuJoCo_Pinocchio_KDL_工具链.md)
- 八股：[Python 八股](../../03_八股知识/01_计算机基础/06_Python八股.md)
- 项目：[P2](../../01_项目课程/P2_未来动力双臂遥操作与差速底盘/README.md)、[P4](../../01_项目课程/P4_OpenHarmony智能园艺系统/README.md)
