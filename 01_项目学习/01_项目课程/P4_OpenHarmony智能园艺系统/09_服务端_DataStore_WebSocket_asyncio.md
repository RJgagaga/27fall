# 09 服务端：DataStore、WebSocket 与 asyncio

> **一句话定位**：服务端是 P4 的"汇聚点"——两路设备数据从**阻塞的 TCP 线程**进来，写进**线程安全的 DataStore 快照**，再通过**asyncio 事件循环**用 WebSocket 推给网页、送入决策链。本章讲清楚这三件事怎么拼在一起，尤其是**asyncio 和跨线程调用**——学习笔记显示这是最容易困惑的地方。

## 本章学完你能……

- 画出服务端的线程/事件循环结构，说明每个组件在哪个执行上下文里运行。
- 设计一个 **DataStore**：存什么、为什么带来源/时间/序号、为什么用短锁、如何避免读到"撕裂"的快照。
- 讲清 asyncio：事件循环、协程、`await`、任务、`gather`，并**手工推演**两个协程如何交错运行。
- 解释为什么 WebSocket 服务适合 asyncio，以及一个阻塞调用会怎样卡死整个循环。
- 讲清**跨线程**调用：`run_coroutine_threadsafe` / `call_soon_threadsafe` 做了什么，为什么不能在其他线程里直接 `create_task`，以及"怎么知道是哪个任务"。
- 估算队列积压，设计满队列策略；讲清广播中慢客户端的问题。
- 说明前端如何消费消息，并把**五类信息分开显示**、标明确认级别。

## 前置知识

- Python 工程：[Python 工程与科学计算](../../02_基础知识/09_机器人软件/03_Python工程与科学计算.md)
- 并发八股：[并发多线程与实时系统八股](../../03_八股知识/01_计算机基础/07_并发多线程与实时系统八股.md)
- Python 八股：[Python 八股](../../03_八股知识/01_计算机基础/06_Python八股.md)
- 上一章：[08 TCP 通信与协议设计](./08_TCP通信与协议设计.md)

## 目录

1. 服务端的整体结构
2. DataStore：共享状态
3. asyncio 基础
4. 手算：两个协程的交错
5. 为什么 WebSocket 用 asyncio
6. 阻塞调用的危害
7. 跨线程：从 TCP 线程到事件循环
8. 队列与背压
9. WebSocket 广播与慢客户端
10. 前端如何消费
11. 用户命令的路径
12. 在本项目里怎么用 / 实现线索
13. 教学示例代码
14. 常见错误与故障现象
15. 面试怎么问、怎么答
16. 要点速记与自测

---

## 1. 服务端的整体结构

```text
┌───────────────────────────── 服务端进程（Python）──────────────────────────────┐
│                                                                                │
│  线程 A: TCP 接收（阻塞 recv，帧重组）──写入──▶ DataStore（锁保护的快照）       │
│        │                                          ▲                            │
│        │ run_coroutine_threadsafe / call_soon_threadsafe                        │
│        ▼                                          │ 读取快照                    │
│  ┌──────────── asyncio 事件循环（线程 B）────────────────────────────────┐     │
│  │  WebSocket 服务器：接入网页、广播                                      │     │
│  │  Agent 消费协程：从队列取事件 → LangGraph → 结果                       │     │
│  │  命令发送协程：执行层通过的命令 → 回发设备（经 TCP 发送通道）           │     │
│  └────────────────────────────────────────────────────────────────────────┘     │
└────────────────────────────────────────────────────────────────────────────────┘
          ▲ TCP                                              │ WebSocket
 Hi3861 / RK3588                                          网页前端
```

关键事实：
- **TCP 接收是阻塞式线程**（`recv` 会睡眠等数据），WebSocket/Agent 是 **asyncio**，两者"并存"，需要安全地跨线程传递；
- **共享状态**只有一个：DataStore；
- **慢操作（LLM、网络）**不能卡住事件循环，也不能在持锁时做。

---

## 2. DataStore：共享状态

### 2.1 要解决的问题

多个执行体（TCP 线程、事件循环里的 WebSocket 协程、Agent 协程、执行层）需要读写**同一份"最新状态"**。直接共享裸变量会产生竞争。DataStore 就是**带锁的状态容器 + 对外只给快照**。

### 2.2 存什么

| 数据 | 来源 | 关键元数据 |
|---|---|---|
| 传感器快照 | Hi3861 | `moisture`、`pump`、`mode`、`seq`、`received_at`、`device_time?` |
| 视觉观测 | RK3588 | `detections`、`event_id`、`captured_at`、`received_at`、`has_image` |
| 执行状态 | 执行层/设备反馈 | 最近命令 `action_id`、状态阶段（requested/acked/reported）、时间 |
| 连接状态 | 各接收线程 | `online`、`last_seen` |

**每一条都带来源、时间、序号**——为了回答"这个数是哪来的、多新、是不是重复"。笔记要点："每条数据带来源；每条数据带年龄；同一画面重复广播不等于新检测。"

### 2.3 短锁原则

> 笔记原话："DataStore 共享快照需短锁；阻塞 LLM/网络调用不能卡住所有 Web 连接。"

- 持锁区内只做**内存读写**（几微秒～几十微秒）；
- **绝不**在持锁时做网络 I/O、LLM 调用、磁盘写入——否则其他线程全被卡住；
- 读取时**拷贝**一份快照出来再释放锁，然后在锁外做慢事。

### 2.4 为什么要"整体更新"：撕裂读

假设传感消息有 `moisture` 和 `pump` 两个字段，写线程分两步更新：先写 `moisture=66`，再写 `pump=off`。读线程恰好在两步之间读到 `moisture=66, pump=on`——一个**从未真实存在过的组合**（撕裂读，torn read）。

后果例子：Agent 看到"湿度 66 但泵还开着"，可能误判为泵故障。

解决：**同一次消息的所有字段在同一把锁里一次写完**，读者在同一把锁里一次拷贝。

### 2.5 年龄与新鲜度

存入 `received_at`（服务端时钟），读取时计算：

$$\text{age}=t_{\text{now}}-t_{\text{received}}$$

快照对象提供 `age()`，使用方根据自己的阈值判断可否使用。**不要在 DataStore 里"删除旧数据"**——保留旧值并带上年龄，让使用方决定；否则"数据不见了"和"数据很旧"无法区分。

---

## 3. asyncio 基础

### 3.1 它解决什么问题

服务端要同时处理**很多个"大部分时间在等"的任务**：100 个网页连接，每个大部分时间在等对方发消息或等自己发完。asyncio 用**一个线程 + 事件循环**，在任务"等 I/O"的空档切换到别的任务，让一个线程"看起来同时"处理很多连接。

### 3.2 核心概念

| 概念 | 含义 |
|---|---|
| 事件循环（event loop） | 调度中心：决定下一个运行哪个任务；`asyncio.run(main())` 创建、运行、关闭它 |
| 协程（coroutine） | `async def` 定义的函数，调用它得到一个协程对象，**还没运行** |
| `await` | "我在这等某个操作完成，等的时候请把 CPU 让给别人" |
| 任务（Task） | 被事件循环调度的协程；`asyncio.create_task(coro())` |
| `gather` | 并发运行多个协程并等待全部完成 |
| Future | 代表"将来会有的结果" |

**关键认知**：
- 协程是**协作式**的：只有在遇到 `await` 才会让出控制权；
- **同一时刻只有一个协程在运行**——不是并行，是**交替执行**；
- 所以 asyncio 对 **I/O 密集** 有效，对 **CPU 密集**（比如纯 Python 的大循环）无效，且会卡住别的协程。

### 3.3 与多线程对比

| | asyncio | 多线程 |
|---|---|---|
| 并发单位 | 协程 | 线程 |
| 调度者 | 事件循环（协作式，在 `await` 处切换） | 操作系统（抢占式） |
| 切换开销 | 很小 | 较大（上下文切换、每线程栈） |
| 共享状态 | 同一线程内，两次 `await` 之间不会被打断，**通常不需要锁** | 需要锁 |
| 适合 | 大量 I/O 等待 | 阻塞库、CPU 工作（受 GIL 限制） |
| 心智负担 | 要记住"别阻塞循环" | 要记住"加锁、避免死锁" |

---

## 4. 手算：两个协程的交错

```python
import asyncio

async def task_a():
    print("A: 开始")
    await asyncio.sleep(2)
    print("A: 结束")

async def task_b():
    print("B: 开始")
    await asyncio.sleep(1)
    print("B: 结束")

async def main():
    await asyncio.gather(task_a(), task_b())

asyncio.run(main())
```

**逐步推演（时间轴）**：

| 时刻 | 事件循环做了什么 | 输出 |
|---|---|---|
| 0.0 s | 创建 A、B 两个任务，先运行 A | `A: 开始` |
| 0.0 s | A 遇到 `await sleep(2)`：登记"2 s 后唤醒我"，让出 | |
| 0.0 s | 循环转去运行 B | `B: 开始` |
| 0.0 s | B 遇到 `await sleep(1)`：登记"1 s 后唤醒"，让出 | |
| 0.0–1.0 s | 没有可运行任务；循环睡眠，等最早的定时器 | |
| 1.0 s | 唤醒 B | `B: 结束` |
| 2.0 s | 唤醒 A | `A: 结束` |

**总耗时 2 s**（不是 3 s）。如果把 `await asyncio.sleep` 换成阻塞的 `time.sleep`，A 的 2 s 里 B 根本得不到运行，总共 3 s，且这期间整个循环（包括所有 WebSocket 连接）都被冻住——见下一节。

---

## 5. 为什么 WebSocket 用 asyncio

### 5.1 WebSocket 是什么

HTTP 是"请求—响应"，服务端不能主动推。WebSocket 在 HTTP 握手（`Upgrade: websocket`）之后，把同一条 TCP 连接变成**全双工的消息通道**：服务端可以随时推，网页也可以随时发。

用途：**展示与交互**（传感器曲线、检测框、建议、执行状态；用户点"手动关"）。

> 笔记要点：**WebSocket 是显示/交互接口，不是硬实时内环**。关键控制由设备本地闭环完成。

### 5.2 为什么适合 asyncio

假设 100 个网页客户端同时在线：
- **线程模型**：100 个线程，每个线程大部分时间睡在 `recv` 上；每个线程要独立栈空间，切换有开销；共享的客户端列表要加锁；
- **asyncio 模型**：1 个线程，100 个协程；每个协程在 `await ws.recv()` 时让出；内存与切换开销都小得多；共享列表只在同一线程内，不用锁。

---

## 6. 阻塞调用的危害

**事件循环是单线程**。如果任何协程里调用了一个**阻塞**函数，整个循环被冻住：

```python
async def bad_handler(ws):
    result = call_llm_blocking(prompt)      # 阻塞 3 秒的同步 HTTP 请求
    await ws.send(result)
```

**手算影响**：循环被冻 3 s，这期间：
- 所有其它 WebSocket 客户端收不到推送；
- 心跳（ping/pong）无法处理，客户端可能判定超时断开；
- 其它协程（Agent 消费者、命令发送）全部停摆。

**正确做法**：

| 场景 | 做法 |
|---|---|
| LLM/HTTP 请求 | 使用异步客户端（`await client.chat(...)`） |
| 只能用同步库 | `await loop.run_in_executor(None, call_llm_blocking, prompt)`，把阻塞调用丢到线程池 |
| CPU 密集 | 进程池或独立进程 |
| 简单延时 | `await asyncio.sleep()` 而不是 `time.sleep()` |

---

## 7. 跨线程：从 TCP 线程到事件循环

### 7.1 问题

线程 A（TCP 接收）收到数据，想让事件循环（线程 B）去广播。**直接在线程 A 里调用**：

```python
asyncio.create_task(broadcast(msg))   # 错误！
```

会出错或行为未定义：事件循环的内部数据结构**不是线程安全的**，`create_task`、`queue.put_nowait` 等必须在**事件循环所在的线程**里调用。

### 7.2 类比

两个人在两个房间，不能直接说话，只能通过**门缝递纸条**。`run_coroutine_threadsafe` 就是那个门缝。

### 7.3 正确做法

```python
loop = asyncio.get_running_loop()      # 在事件循环线程里获取并保存

# 在线程 A 里：
fut = asyncio.run_coroutine_threadsafe(broadcast(parsed), loop)
# fut 是 concurrent.futures.Future，可 fut.result(timeout=...) 等结果（会阻塞线程 A）
```

或者更轻量、"发了就不管"：

```python
loop.call_soon_threadsafe(queue.put_nowait, parsed)   # 往 asyncio.Queue 里放
```

### 7.4 底层发生了什么

1. 在线程 A 里，把"要执行的回调/协程"加到事件循环的**待办队列**（用锁保护）；
2. 通过**唤醒机制**（常见实现是向一个内部 socket pair/管道写一个字节）叫醒可能正在 `select/epoll` 里睡觉的事件循环；
3. 事件循环线程醒来，在**自己的线程**里取出并执行这个回调（所以后续对 `create_task` 等的调用都在正确的线程里）。

### 7.5 "怎么知道是哪个任务"

提问来自学习笔记："通知线程 B 的广播，那知道是哪个任务需要的吗？"

答案：**参数在调用时就绑定了**。

```python
data = sock.recv(...)
parsed = parse(data)
asyncio.run_coroutine_threadsafe(broadcast(parsed), loop)
#                                 └── broadcast(parsed) 是一个已绑定参数的协程对象
```

`broadcast(parsed)` 创建了一个**带着 `parsed` 的协程对象**，事件循环将来运行它时，用的就是当时传入的那份数据。它不需要"知道是谁要的"，因为任务内容就是被传过去的那个对象。

**广播给谁**：通常是**所有**当前连接的客户端；如需区分，消息里带 `type` 字段，前端按 `type` 分发；或者服务端按客户端订阅的主题过滤。

---

## 8. 队列与背压

### 8.1 为什么要队列

TCP 线程产生事件的速度和 Agent 处理事件的速度不同：事件来得快、LLM 处理慢。中间放一个**有界队列**解耦，并决定"来不及时怎么办"。

### 8.2 手算：队列会多快满

教学示例：事件到达速率 $\lambda=5$ 个/s；LLM 路径处理一个事件平均 2 s，即服务速率 $\mu=0.5$ 个/s；队列容量 100。

净积压速率 $=\lambda-\mu=4.5$ 个/s。

$$t_{\text{满}}=\frac{100}{4.5}\approx22.2\ \text{s}$$

22 秒后队列满。之后要么丢事件，要么阻塞生产者——如果生产者是 TCP 接收线程，阻塞它会让设备数据堆在内核缓冲区，进而拖慢设备。

### 8.3 满队列策略

| 策略 | 做法 | 适用 |
|---|---|---|
| 丢新（drop newest） | `put_nowait` 抛 `QueueFull` → 记录日志并丢弃新事件 | 简单；线索显示源码的满队列处理是记录日志并丢弃新事件 |
| 丢旧（drop oldest） | 弹出最旧的，放入新的 | 状态类事件，只关心最新 |
| 合并（coalesce） | 同类事件只保留最新一条（如每个 location 的最新观测） | 最适合"最新状态"语义 |
| 限速 | 在入口按 location 节流（如每 10 s 最多一次分析） | 减少 LLM 调用 |
| 分级 | fast path 事件走独立高优先级通道，不被 LLM 积压拖住 | **极端状态不能排在慢事件后面** |

> **重要**：丢事件要有**计数器和日志**（`dropped_events`）。"系统安静"和"系统在丢数据"必须能区分。**【待你补全：队列大小与满队列的实际处理】**

---

## 9. WebSocket 广播与慢客户端

### 9.1 简单实现的问题

```python
async def broadcast(data):
    for ws in list(clients):
        await ws.send(json.dumps(data))      # 顺序等待每个客户端
```

如果某个客户端网络很差，它的 `send` 要等很久（发送缓冲区满）——**后面所有客户端都被它拖住**。

**手算**：100 个客户端，每次发送 1 ms，其中第 10 个卡住 5 s：顺序发送，第 11–100 个客户端的延迟变成 $5\text{ s}+\ldots$；总时间 $\approx5.1$ s。

### 9.2 改进

```python
async def send_one(ws, payload):
    try:
        await asyncio.wait_for(ws.send(payload), timeout=0.5)
    except Exception:
        clients.discard(ws)                  # 超时或出错：踢掉

async def broadcast(data):
    payload = json.dumps(data)
    await asyncio.gather(*(send_one(ws, payload) for ws in list(clients)))
```

最坏延迟被限制在约 0.5 s，慢客户端被剔除（它重连后会重新拿到一次全量快照）。更健壮的做法是**每个客户端一个发送队列 + 发送协程**，队列满则丢旧。

### 9.3 新客户端连上来

先发一份**当前全量快照**（带年龄），再开始推增量。否则网页刷新后会一片空白，直到下一条消息到来。

---

## 10. 前端如何消费

前端（`app.js`）通过 WebSocket 接收 JSON 消息，按 `type` 分发：

```javascript
ws.onmessage = (event) => {
  const msg = JSON.parse(event.data);
  switch (msg.type) {
    case "sensor":    updateSensorPanel(msg);    break;
    case "vision":    updateVisionPanel(msg);    break;
    case "advice":    updateAdvicePanel(msg);    break;
    case "execution": updateExecutionPanel(msg); break;
    default: console.warn("unknown message type", msg.type);
  }
};
```

### 10.1 设计要点

1. **五类信息分开显示**：传感读数、模型观测、建议、命令请求、设备反馈各一个区域，不要混在一个"状态"里；
2. **显示确认级别**：同一个"泵"，显示"已请求（pending）/ 设备已确认 / 设备报告已开"，笔记要点："前端 watering 显示可能是上报字段或命令状态，需说明其确认级别"；
3. **显示年龄**：每块面板显示"3 s 前"，超过阈值变灰/标红（"数据过期"）；
4. **断线重连**：`onclose` 后指数退避重连，重连成功后以服务端发来的全量快照为准；
5. **不信任消息内容**：渲染到 DOM 时对文本做转义（防 XSS），不要直接 `innerHTML = msg.text`；LLM 生成的建议文本尤其如此；
6. **图像**：检测框叠加在 JPEG 上时，用事件里的 `image_size` 与框坐标在 canvas 上缩放。

### 10.2 一个具体的显示状态机（命令）

```text
用户点击"手动关"
   → UI: 命令 pending（灰色，转圈）
   → 服务端收到并通过校验 → UI: 已发送
   → 设备应用层 ACK       → UI: 设备已收到
   → 设备状态反馈 pump=off → UI: 设备报告已关 ✓
   → 超时未反馈           → UI: 未确认（黄色告警）
```

---

## 11. 用户命令的路径

```text
网页(按钮) ──WebSocket/HTTP──▶ 服务端接口 ──▶ 执行层（白名单+模式+权限+新鲜度）
                                                  │ 通过
                                                  ▼
                                   命令发送协程 ──TCP──▶ Hi3861 ──▶ 状态反馈 ──▶ DataStore ──▶ 广播
```

要点：
- **用户命令也要过执行层**，不是直通设备；人工 off 优先于自动建议，是在执行层里实现的；
- 用户命令带 `source="user-web"`，日志可追溯；
- 详见 [11 执行边界](./11_执行边界_白名单_幂等与设备反馈.md)。

---

## 12. 在本项目里怎么用 / 实现线索

| 项 | 线索（以实机为准） |
|---|---|
| CV 对应 | "orchestrated decisions with a server-side agent and closed the loop through web interaction and device-side irrigation" |
| 文件职责 | `server.py`：汇聚/广播/执行接口；`agent_bridge.py`：线程与 asyncio 之间的事件/结果队列；`plant_graph.py`：LangGraph 图与去重；`rule_engine.py`：规则阈值；`app.js`：前端状态 |
| 并发 | TCP 阻塞线程 + asyncio 广播；线程安全方式提交事件；满队列时记录日志并丢弃新事件 |
| 记忆组件 | 有数据库/向量检索，且有关键词/内存降级路径（是否启用【待你补全】） |
| WebSocket 库、端口 | **【待你补全】** |

> 不要说："我实现了高并发服务器"。可以说："the server has a thread for blocking TCP reads and an asyncio loop for WebSocket and the agent, with a thread-safe bridge between them."

---

## 13. 教学示例代码

```python
# 教学示例：线程 + asyncio 桥接 + DataStore。非项目源码。
import asyncio, json, threading, time
from dataclasses import dataclass, field

@dataclass
class Snapshot:
    data: dict
    received_at: float = field(default_factory=time.monotonic)
    def age(self) -> float:
        return time.monotonic() - self.received_at

class DataStore:
    def __init__(self):
        self._lock = threading.Lock()
        self._sensor = None
        self._vision = None
    def update_sensor(self, d: dict):
        snap = Snapshot(dict(d))              # 先在锁外构造
        with self._lock:                      # 锁内只做赋值
            self._sensor = snap
    def update_vision(self, d: dict):
        snap = Snapshot(dict(d))
        with self._lock:
            self._vision = snap
    def snapshot(self) -> dict:
        with self._lock:
            s, v = self._sensor, self._vision  # 一次性拿走两个引用
        return {"sensor": s, "vision": v}      # 锁外使用

store = DataStore()
clients = set()
event_q: asyncio.Queue = None
loop: asyncio.AbstractEventLoop = None
dropped = 0

def tcp_thread():                              # 线程 A：模拟阻塞接收
    seq = 0
    while True:
        time.sleep(1)                          # 假装 recv 阻塞
        seq += 1
        msg = {"type": "sensor", "moisture": 50 + seq, "seq": seq}
        store.update_sensor(msg)               # 线程安全
        loop.call_soon_threadsafe(_offer, msg) # 通知事件循环（线程安全）

def _offer(msg):                               # 在事件循环线程里运行
    global dropped
    try:
        event_q.put_nowait(msg)
    except asyncio.QueueFull:
        dropped += 1                           # 计数 + 日志，不能静默

async def agent_consumer():
    while True:
        msg = await event_q.get()
        # 这里调用 LangGraph（异步或放线程池），得到结果后广播
        await broadcast({"type": "advice", "for_seq": msg["seq"]})

async def send_one(ws, payload):
    try:
        await asyncio.wait_for(ws.send(payload), timeout=0.5)
    except Exception:
        clients.discard(ws)

async def broadcast(data):
    payload = json.dumps(data)
    await asyncio.gather(*(send_one(ws, payload) for ws in list(clients)))

async def main():
    global loop, event_q
    loop = asyncio.get_running_loop()
    event_q = asyncio.Queue(maxsize=100)
    threading.Thread(target=tcp_thread, daemon=True).start()
    await agent_consumer()
```

---

## 14. 常见错误与故障现象

| 现象 | 可能原因 | 排查 |
|---|---|---|
| 网页卡住/不更新 | 事件循环被阻塞调用冻住；持锁做了慢操作 | 日志打印循环延迟；检查 `time.sleep`/同步 HTTP |
| `RuntimeError: no running event loop` / 随机崩溃 | 在非循环线程直接调用 `create_task`/`Queue.put_nowait` | 改用 `call_soon_threadsafe`/`run_coroutine_threadsafe` |
| 状态面板数据对不上 | 撕裂读：多字段分开更新 | 同锁整体更新+整体读取 |
| 重复广播同一画面 | 把重发当作新检测 | 用 `event_id/seq` 去重 |
| 慢客户端拖慢所有人 | 顺序 await 发送 | 并发+超时+踢出 |
| 数据"消失" | 把旧数据删除而不是带年龄 | 保留旧值+年龄 |
| 队列满事件丢失 | 处理速度低于到达速度 | 计数、合并、限速、fast path 单独通道 |
| 内存持续增长 | 客户端集合不清理；队列无界 | 断线清理；有界队列 |
| 网页显示"已浇水"但没浇 | 前端把命令状态当成设备反馈 | 区分确认级别 |

---

## 15. 面试怎么问、怎么答

**Q1. How is your server structured, and why both threads and asyncio?**

> "A blocking thread reads the TCP streams from the devices and reassembles frames. It writes the parsed state into a lock-protected data store, and hands the event to the asyncio loop. The loop runs the WebSocket server and the agent consumer. I used asyncio for WebSockets because many mostly-idle connections are cheap as coroutines, and kept the blocking socket reads in a thread so they can't stall the loop."

（译：一个阻塞线程读 TCP 并重组帧，写入带锁的数据存储，再把事件交给 asyncio 循环；循环里跑 WebSocket 和 Agent 消费者。用 asyncio 是因为大量空闲连接作为协程很便宜，阻塞的 socket 读放线程里不会卡住循环。）

追问 1：**How do you call into the event loop from another thread?**
> "You can't call `create_task` from another thread because the loop isn't thread-safe. I use `run_coroutine_threadsafe` or `call_soon_threadsafe`, which put the work in the loop's ready queue under a lock and wake the loop. The argument is bound when I create the coroutine, so the loop runs it with exactly that data."

追问 2：**What would happen if a handler called a blocking function?**
> "The whole loop would stop, so every WebSocket client would stop receiving, heartbeats would time out, and the agent consumer would stall. I use async clients or run blocking calls in an executor."

追问 3：**Why a lock in the data store, if asyncio is single-threaded?**
> "Because the TCP thread is a different thread. Anything shared between that thread and the loop needs a lock. I keep the critical section to plain memory assignments and copy the snapshot out before doing anything slow."

**Q2. What happens when events arrive faster than the agent can process them?**

> "The queue fills. With, say, five events per second and two seconds per LLM path, a queue of 100 fills in about 22 seconds. I don't want silent loss, so I count and log drops. Better options are coalescing to the latest observation per location, rate-limiting analysis, and keeping the rule-based fast path on a separate channel so urgent states don't wait behind slow LLM calls."

**Q3. How does the front end know what a message is, and what does it show?**

> "Each message has a type field. The page keeps separate panels for sensor readings, detections, advice and execution status, and shows the age of each. For a command it shows the confirmation level: requested, acknowledged by the device, or reported by the device. A 'sent' state is not displayed as 'pump on'."

**Q4. Is WebSocket suitable for control?**
> "No, it's for display and interaction. Irrigation control stays in the device-side loop, because a web connection has no real-time guarantee and can drop."

---

## 本章要点速记

1. 结构：阻塞 TCP 线程 + 带锁 DataStore + asyncio 循环（WebSocket、Agent 消费者、命令发送）。
2. DataStore：带来源/时间/序号；短锁；整体更新整体读；保留旧值带年龄，不删除。
3. asyncio：单线程、协作式、`await` 处切换；I/O 密集有效；CPU/阻塞调用会冻住整个循环。
4. 两个协程示例：2 s + 1 s 并发总耗时 2 s，输出顺序 A开始、B开始、B结束、A结束。
5. 跨线程：`run_coroutine_threadsafe` / `call_soon_threadsafe`；不能在其它线程里 `create_task`。
6. 参数在创建协程对象时已绑定；广播给所有客户端，前端按 `type` 分发。
7. 队列积压：$t=\text{容量}/(\lambda-\mu)$；丢事件要计数；fast path 走独立通道。
8. 广播要并发+超时，防慢客户端；新连接先发全量快照。
9. 前端：五类信息分开、显示确认级别和年龄、转义文本、断线重连。

## 自测题

1. 为什么在线程 A 里直接 `asyncio.create_task(...)` 是错误的？正确做法是什么？
2. 队列容量 50，到达 4 个/s，处理 1 个/s，多久满？
3. 说出两种让 LLM 同步调用不卡死事件循环的办法。
4. 什么是撕裂读？如何避免？
5. 前端该如何显示一次"开泵"命令的状态？为什么不能直接显示"已开泵"？

**参考答案**：
1. 事件循环非线程安全，必须在循环线程操作；用 `run_coroutine_threadsafe`/`call_soon_threadsafe`。
2. $50/(4-1)=16.7$ s。
3. 异步客户端；`run_in_executor` 丢线程池。
4. 读到未同时存在的字段组合；同锁整体写、整体读（拷贝快照）。
5. 分级显示 pending → 已发送 → 设备已收到 → 设备报告已开；命令状态不等于设备状态。

## 相关链接

- 上一章：[08 TCP 通信与协议设计](./08_TCP通信与协议设计.md)
- 下一章：[10 LangGraph 决策链](./10_LangGraph决策链.md)
- 执行边界：[11 执行边界](./11_执行边界_白名单_幂等与设备反馈.md)
- 系统集成：[12 系统集成与可追踪系统状态](./12_系统集成与可追踪系统状态.md)
- 返回：[P4 README](./README.md)
