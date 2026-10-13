# 08 TCP 通信与协议设计

> **一句话定位**：TCP 保证"字节按序、不丢地到"，但**不保证"一次 send 对应一次 recv"**。P4 里 Hi3861、RK3588 和服务端之间的每一条消息，都是靠**自己约定的边界**（长度前缀、分段）从字节流里重新拼出来的。

## 本章学完你能……

- 解释 TCP 是**字节流**，说出粘包/半包为什么发生，以及"TCP 本身没有粘包问题，是应用层没定义边界"的准确含义。
- 比较分隔符、定长、长度前缀、自定界（JSON 括号配对）四种成帧方式。
- 设计一个 `type + length + payload` 的帧格式，处理字节序与最大长度保护。
- 手工推演一段字节流被任意切块后的解析过程。
- 说明 **JSON + JPEG 分段**的发送顺序、配对与带宽考虑；计算 base64 膨胀。
- 讲清连接管理：重连、超时、心跳、半开连接，以及命令重发为什么要幂等。
- 英文回答 "Why did you add a length prefix?" 等问题。

## 前置知识

- 网络基础：[UART、SPI、I2C、以太网与 TCP](../../02_基础知识/11_嵌入式与通信/04_UART_SPI_I2C_以太网_TCP.md)
- 八股：[计算机网络](../../03_八股知识/01_计算机基础/03_计算机网络.md)
- 上一章：[07 INT8 量化与 RKNN 转换](./07_INT8量化与RKNN转换.md)

## 目录

1. TCP 是字节流
2. 粘包与半包
3. 四种成帧方式
4. 长度前缀帧的设计
5. 字节序与最大长度保护
6. 手算：切块字节流的解析
7. JSON + JPEG 分段
8. Hi3861 的 JSON 流与括号配对解析
9. 连接管理：重连、超时、心跳、半开
10. 命令与反馈：可靠传输之上的应用层可靠
11. 为什么是 TCP，而不是 UDP / HTTP / MQTT
12. 在本项目里怎么用 / 实现线索
13. 教学示例代码
14. 常见错误与故障现象
15. 面试怎么问、怎么答
16. 要点速记与自测

---

## 1. TCP 是字节流

**TCP 提供**：可靠（丢包重传）、有序、流量/拥塞控制、面向连接。
**TCP 不提供**：消息边界。

把 TCP 想成一根水管：你往里倒水（`send`），对面接水（`recv`），水是连续的，没有"这一杯那一杯"的概念。内核可以把你一次 `send` 的数据切成多个 TCP 段发出，也可以把多次 `send` 合并成一个段；接收端的 `recv(n)` 返回**当前缓冲区里有的至多 n 个字节**，可能比你期望的少，也可能多出下一条消息的开头。

> **准确表述**："粘包/半包"不是 TCP 的缺陷，而是应用层**没有定义消息边界**造成的错觉。

---

## 2. 粘包与半包

| 现象 | 含义 | 例子 |
|---|---|---|
| 半包 | 一条消息还没收全就返回了 | 想读 108 字节，这次 `recv` 只拿到 20 字节 |
| 粘包 | 一次 `recv` 里含多条消息 | 一次收到 消息1 全部 + 消息2 的前 30 字节 |
| 分包 | 一条消息被拆成多次 `recv` | 半包的另一面 |

**手算**（沿用学习笔记的例子）：发送端写了 `[8 字节长度头][100 字节 JSON]`，共 108 字节。

- 第一次 `recv` 得到 20 字节 → 还缺 $108-20=88$ 字节；**不能把这 20 字节当完整 JSON**；
- 也可能一次 `recv` 就拿到 108 字节；
- 也可能拿到 108 字节后面还粘着下一条消息的开头。

一次 `recv` 就当一条消息的程序，在局域网里小消息偶尔能跑通，**在大消息（JPEG）、网络拥塞、Wi-Fi 重传时一定出错**，而且是难复现的间歇性错误。

---

## 3. 四种成帧方式

| 方式 | 做法 | 优点 | 缺点 |
|---|---|---|---|
| 分隔符 | 以 `\n` 等特殊字节结尾 | 人可读，调试方便 | 载荷里不能出现分隔符，需转义；二进制（JPEG）不适用 |
| 定长 | 每条消息固定 N 字节 | 最简单 | 浪费，不适合变长数据 |
| **长度前缀** | 先发长度，再发 payload | 通用，支持二进制，解析简单 | 要定义长度字段宽度与字节序 |
| 自定界 | 格式本身可判断结束（如 JSON 括号配对） | 无需额外头 | 需要小型解析器，遇到字符串里的括号要小心 |

P4 里两种都出现过：**RK3588 → 服务端用长度前缀分段**（JSON + JPEG），**Hi3861 → 服务端的传感 JSON 线索是靠括号配对取对象**（见第 8 节）。

---

## 4. 长度前缀帧的设计

### 4.1 基本格式

```text
┌──────────┬───────────────────┬─────────────────────┐
│ type 1 B │ length N B (大端) │ payload (length 字节) │
└──────────┴───────────────────┴─────────────────────┘
```

学习笔记里的线索：`tcp_send_data` 先发 **length（8 字节）**，再发 payload；外层 `tcp_send_detection_result` 又**按类型/JSON/JPEG 分段**发送。也就是说"每一段"都是 `length + payload`，段与段之间按协议顺序排列。

**教学示例（不代表你的实际字段）**：

| 段 | 内容 |
|---|---|
| 1 | 类型码（如 `0x01` 表示检测结果） |
| 2 | JSON 长度 + JSON 字节 |
| 3 | JPEG 长度 + JPEG 字节（可为 0） |

**我们的做法**：RK3588 到服务端的每条消息是 `[4 字节大端 JSON 长度 N][4 字节大端 JPEG 长度 M]` 共 8 字节头，后面依次是 N 字节 JSON 与 M 字节 JPEG；只有结构化事件没有配图时 M=0。这样同时解释了“8 字节头”和“每个长度字段 4 字节”两种说法：头是 8 字节，由两个 4 字节的大端长度组成。没有 magic 和类型码，靠“JSON 里带 `type` 字段”区分消息种类；接收端先读满 8 字节，再按 N、M 读满两段，任何长度超过上限（本项目取值约 JSON 64 KB、JPEG 2 MB）就断开连接重连，避免错位后一直解析垃圾。

### 4.2 为什么要有 type

同一条连接上要承载多种消息（检测结果、心跳、图像）；`type` 让接收端知道后面是什么，以及下一步用哪个解析器。

### 4.3 为什么要有 magic（可选）

在流中插入固定魔数（如 `0xA55A`）帮助**重新同步**：一旦某处长度字段出错，接收端能扫描到下一个魔数继续解析，而不是整个连接报废。更简单的办法是出错就**关闭连接重连**。

---

## 5. 字节序与最大长度保护

### 5.1 字节序

多字节整数在内存中的字节顺序有两种：

- **大端（big-endian，网络字节序）**：高位字节在前；
- **小端（little-endian）**：低位字节在前（x86、ARM 默认小端）。

**手算**：长度 $100=0x00000064$（4 字节）：
- 大端发送的字节：`00 00 00 64`；
- 小端发送的字节：`64 00 00 00`；
- 如果发送端小端、接收端按大端读：`64 00 00 00` → $0x64000000=100\times2^{24}=1{,}677{,}721{,}600$（约 1.6 GB）。

**后果**：接收端会以为要等 1.6 GB 的数据——要么一直等下去，要么试图分配 1.6 GB 内存而崩溃。所以：
1. **协议文档必须写明字节序**（建议网络字节序）；
2. **接收端必须设最大长度**。

### 5.2 最大长度保护

```python
MAX_PAYLOAD = 2 * 1024 * 1024        # 教学取 2 MB：略大于最大 JPEG
if length > MAX_PAYLOAD:
    raise ProtocolError("frame too large")   # 关闭连接，等待重连
```

否则一个错误或恶意的长度字段就能让服务端分配巨大内存（内存耗尽型拒绝服务）。

---

## 6. 手算：切块字节流的解析

### 6.1 设定（教学示例）

使用 `[length 8 B][payload]` 的"段"。发送端依次发送两段：
- 段 1：JSON，100 字节 → 8+100 = 108 字节；
- 段 2：JPEG，20,000 字节 → 8+20000 = 20,008 字节。

字节流总长 $108+20008=20116$。接收端每次 `recv(4096)`，实际按网络切成若干块。

### 6.2 解析器状态

```text
状态 WAIT_HEADER: 需要 8 字节
状态 WAIT_BODY(n): 需要 n 字节
缓冲区 buf：累积收到的字节
```

### 6.3 逐步推演

| 步骤 | 本次 `recv` 到手 | 解析动作 | 解析后状态 |
|---|---|---|---|
| ① | 4096 B（第 1 块） | 读头 8 B → length=100；缓冲区够 → 取出 100 B JSON ✓（段 1 完成，共消耗 108 B）；继续读下一段头 8 B → length=20000；此时已有 payload 字节 $4096-108-8=3980$ | WAIT_BODY(20000)，已有 3980，缺 16,020 |
| ② | 4096 B（第 2 块） | 追加到 payload | 已有 8,076，缺 11,924 |
| ③ | 4096 B（第 3 块） | 追加 | 已有 12,172，缺 7,828 |
| ④ | 4096 B（第 4 块） | 追加 | 已有 16,268，缺 3,732 |
| ⑤ | 3,732 B（第 5 块） | 追加 → 恰好凑满 20,000 B → JPEG 取出 ✓（段 2 完成） | WAIT_HEADER |

（核对：$4096\times4+3732=20116=108+20008$ ✓。）

**要点**：
- 半包：步骤 ②~⑤，一条消息被多次 `recv` 拼成；
- 粘包：步骤 ①，一次 `recv` 里含完整的段 1 与段 2 的开头；
- 解析器必须是"**有状态**"的，并且每次 `recv` 后循环尽可能多地取出完整消息。

---

## 7. JSON + JPEG 分段

### 7.1 为什么分两段

| | JSON（观测） | JPEG（图像） |
|---|---|---|
| 大小 | 几百字节～几 KB | 几十～几百 KB |
| 重要性 | **核心**（决策靠它） | 可选（给人看、做留证） |
| 可丢弃 | 不能丢 | 可丢（带宽紧时） |
| 编码 | 文本 | 二进制 |

分段让接收端**先拿到 JSON、先决策**，JPEG 慢慢收；JPEG 丢了也不影响观测。

### 7.2 发送顺序与配对

建议顺序：先 JSON，后 JPEG；两者共享 `event_id`（或序号）以配对。线索里的提醒：**JSON 与 JPEG 分开发送时，要用 event_id 确认配对**；"同一画面重复广播不等于新检测"。

配对规则：
- 收到 JSON（event_id=E）后，期望下一段 JPEG 属于 E；
- 若 JPEG 缺失/超时 → 事件仍有效，只是无图；
- 若序号错位（JPEG 的 id 与 JSON 不一致）→ 丢弃图像，记录协议错误计数。

### 7.3 带宽估算（教学示例）

- JSON：300 B/次 × 5 次/s = 1.5 KB/s ≈ 12 kbit/s；
- JPEG：150 KB × 1 张/s = 150 KB/s ≈ 1.2 Mbit/s；
- 若每个观测都带图：150 KB × 5 = 750 KB/s ≈ 6 Mbit/s。

在 Wi-Fi 上 6 Mbit/s 不算太大，但要考虑和 Hi3861 的 Wi-Fi 共享 2.4 GHz 信道；**节流**（如每秒最多 1 张图、有检测才发图）是常见做法。我们的做法是：结构化事件只在“同类连续 3 帧置信度不低于 0.5”时生成，同类事件最短间隔 5 s；只有事件才带 JPEG（质量 80），无事件时不发图，所以平均带宽远低于上面的 6 Mbit/s 上限。

### 7.4 base64 嵌入的代价

如果把 JPEG 用 base64 塞进 JSON 字符串里：每 3 字节变成 4 个字符，膨胀 $4/3$。

$$150{,}000\text{ B}\ \to\ \lceil150000/3\rceil\times4=200{,}000\text{ B}$$

多 50 KB，还要在两端编解码 CPU 开销。分段二进制发送更省。但网页前端要显示图片时，可能在**服务端转 base64 或提供图片 URL**，这是服务端到浏览器的另一段链路，与板子到服务端的链路分开考虑。

---

## 8. Hi3861 的 JSON 流与括号配对解析

### 8.1 线索

此前阅读线索：Hi3861 上报的是**没有长度头的 JSON 流**，服务端按**括号层级**从缓冲区里取出一个个 JSON 对象；而 RK3588 一侧是长度前缀分段。这是两条连接、两种成帧方式。我们的做法正是如此：Hi3861 每 2 s 发送一条紧凑 JSON，不带换行也不带长度头，服务端用括号配对从缓冲区切出对象，并且识别字符串内部的括号与转义，避免把字符串里的 `}` 当成对象结尾。

### 8.2 括号配对算法

```text
depth = 0; in_string = False; escape = False
对缓冲区逐字节：
  若 in_string：
      若 escape：escape=False
      elif ch == '\\': escape=True
      elif ch == '"':  in_string=False
  else：
      '"' → in_string=True
      '{' → 若 depth==0 记录起点；depth += 1
      '}' → depth -= 1；若 depth==0 → 一条完整对象，取出 [起点, 当前]
```

### 8.3 手算

缓冲区收到：`{"m":55,"p":1}{"m":5` （第二个对象还没收完）。

- 解析第一个 `{...}`：depth 0→1→…→0，取出 `{"m":55,"p":1}` ✓；
- 剩 `{"m":5`：depth=1，未配对 → 保留在缓冲区，等待下次数据。

下一块到来 `6,"p":1}`：缓冲区变为 `{"m":56,"p":1}` → depth 回 0 → 取出 ✓。

### 8.4 陷阱

1. **字符串中的括号**：`{"note":"a}b"}` 若不跟踪 `in_string`，会在 `}` 处错误结束；
2. **转义引号**：`"a\"b"` 的 `\"` 不结束字符串；
3. **无界缓冲**：必须有最大长度，否则一直没配对的垃圾会吃光内存；
4. **更简单的替代**：每条 JSON 以换行结尾（NDJSON），按 `\n` 切分。对 MCU 一侧更省事。

---

## 9. 连接管理：重连、超时、心跳、半开

### 9.1 为什么 TCP 连着不代表对方活着

- **半开连接（half-open）**：对方断电、拔网线、Wi-Fi 掉线时，TCP 不一定立刻知道；你这边 `send` 可能还成功写进内核缓冲区，很久以后才报错；
- 空闲连接可能被路由器 NAT 表回收。

### 9.2 常见措施

| 措施 | 做法 | 作用 |
|---|---|---|
| 应用层心跳 | 每 $T_h$ 发一个小消息（或设备状态上报本身当心跳） | 接收端据此判断对方是否在线 |
| 读超时 | `recv` 超过 $T_{\text{to}}$ 无数据 → 判离线、关闭、重连 | 防永久阻塞 |
| TCP keepalive | 内核级探活 | 辅助，间隔通常很长 |
| 重连退避 | 失败后等 1、2、4、8…（封顶）秒再连 | 避免风暴 |
| TCP_NODELAY | 关闭 Nagle 算法 | 小消息低延迟（命令/反馈） |

**手算：指数退避**：初始 1 s，倍增，封顶 30 s：第 1 次失败等 1 s，第 2 次 2 s，第 3 次 4 s，第 4 次 8 s，第 5 次 16 s，第 6 次起 30 s（因 32>30 封顶）。累计前 6 次等待 $1+2+4+8+16+30=61$ s。

### 9.3 对 P4 的意义

- 传感/观测**连接中断**时，服务端应该把对应数据标记为"**过期**"（带 age），而不是继续展示旧值当新值；
- **重连后**服务端不要假设泵状态——要**重新同步设备状态**（笔记里的要点："服务重启→重新同步 device state 而非直接假设泵关"）。

---

## 10. 命令与反馈：可靠传输之上的应用层可靠

TCP 的"可靠"只保证字节到了对方内核，**不保证对方应用处理了**，更不保证泵动了。

| 层次 | 能说明什么 |
|---|---|
| `send()` 返回成功 | 数据进了本机内核缓冲区 |
| TCP ACK | 数据到了对方内核 |
| 应用层 ACK（"收到命令 id=X"） | 对方进程解析了命令 |
| 状态反馈（"pump=on, action_id=X"） | 对方执行了状态变化 |
| 流量/土壤变化 | 水真的浇了 |

对应本项目的措辞："**请求已发送 ≠ 设备收到 ≠ 水流出**"。

**重发与幂等**：断线重连后服务端可能重发命令；命令必须带 `action_id`，设备记住最近处理过的 id，重复到达就不再次执行（返回上次结果）。否则"浇水 20 秒"被收到两次就变成 40 秒。详见 [11 执行边界](./11_执行边界_白名单_幂等与设备反馈.md)。

---

## 11. 为什么是 TCP，而不是 UDP / HTTP / MQTT

| 协议 | 特点 | 适合 | P4 的取舍 |
|---|---|---|---|
| 原生 TCP | 简单、双向、自己定帧 | 设备与自家服务端的长连接 | **P4 用它**：实现简单，MCU 上 lwIP 支持好 |
| UDP | 无连接、可能丢/乱序 | 低延迟、可丢数据（视频流） | 命令/观测不能丢，所以不选；若用要自己做重传 |
| HTTP | 请求-响应，开销大 | 浏览器/REST 接口 | 浏览器与服务端交互用它（网页、WebSocket 握手） |
| MQTT | 发布订阅、QoS、遗嘱消息 | 多设备 IoT | 更"标准"的方案；P4 没用（个人项目，设备少）。可作为未来改进 |

> **面试加分**：被问到"为什么不用 MQTT"，答："MQTT would be a natural choice for many nodes — QoS levels, retained state, last-will for offline detection. For a handful of devices I kept a raw TCP protocol to keep the MCU side small, at the cost of implementing framing and reconnection myself."

**安全局限**（诚实说）：局域网内明文、无认证。若部署到公网需要 TLS 与令牌；对 Hi3861 这类设备，TLS 的内存开销需评估。

---

## 12. 在本项目里怎么用 / 实现线索

| 项 | 线索（以实机为准） |
|---|---|
| CV 对应 | System integration —— 把多设备的信息统一成可追踪的系统状态 |
| RK3588 → 服务端 | 长度前缀分段：类型/JSON/JPEG（4 字节大端 JSON 长度加 4 字节大端 JPEG 长度，共 8 字节头） |
| Hi3861 → 服务端 | JSON 流，服务端括号配对取对象，时间字段未必可靠 |
| 服务端接收 | 阻塞式 TCP 接收线程；重组后写 DataStore，再通知 asyncio 部分广播 |
| 命令方向 | 服务端 → Hi3861 的命令文本；命令名称映射需核对（见 README 口径备忘） |
| 回执 | 有：设备回 `{"ack":action_id,"accepted":true,"pump":0/1,"mode":...}`，并在下一条传感上报里带泵状态 |

---

## 13. 教学示例代码

### 13.1 服务端：精确读取与帧解析（Python）

```python
import socket, struct

MAX_PAYLOAD = 2 * 1024 * 1024
HDR = struct.Struct(">BI")           # 1 字节 type + 4 字节大端 length（教学示例）

def recv_exact(sock: socket.socket, n: int) -> bytes:
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:                          # 对端关闭
            raise ConnectionError("peer closed")
        buf += chunk
    return bytes(buf)

def read_frame(sock):
    t, length = HDR.unpack(recv_exact(sock, HDR.size))
    if length > MAX_PAYLOAD:
        raise ValueError(f"frame too large: {length}")
    return t, recv_exact(sock, length)

def send_frame(sock, t: int, payload: bytes):
    sock.sendall(HDR.pack(t, len(payload)) + payload)   # sendall 保证发完
```

### 13.2 不依赖 socket 的增量解析器（便于单元测试）

```python
class FrameParser:
    def __init__(self):
        self.buf = bytearray()
    def feed(self, data: bytes):
        self.buf += data
        out = []
        while True:
            if len(self.buf) < HDR.size:
                break
            t, length = HDR.unpack_from(self.buf, 0)
            if length > MAX_PAYLOAD:
                raise ValueError("frame too large")
            if len(self.buf) < HDR.size + length:
                break                          # 半包：等更多数据
            payload = bytes(self.buf[HDR.size:HDR.size + length])
            del self.buf[:HDR.size + length]
            out.append((t, payload))           # 粘包：循环继续取
        return out

# 测试：任意切块都应得到相同结果
msg = HDR.pack(1, 5) + b"hello" + HDR.pack(2, 3) + b"abc"
for step in (1, 2, 3, 7, len(msg)):
    p, got = FrameParser(), []
    for i in range(0, len(msg), step):
        got += p.feed(msg[i:i + step])
    assert got == [(1, b"hello"), (2, b"abc")]
```

---

## 14. 常见错误与故障现象

| 现象 | 可能原因 | 排查 |
|---|---|---|
| JSON 解析间歇失败 | 一次 `recv` 当一条消息 | 改成成帧解析器；抓包看分段 |
| 数据"错位"、后面全乱 | 字节序错；长度字段宽度不一致；某次读漏字节 | 打印头 16 字节十六进制 |
| 服务端内存暴涨/崩溃 | 无最大长度；长度读成巨大值 | 加 MAX_PAYLOAD；校验 |
| 图像损坏/图文不配 | 分段顺序错；event_id 未核对 | 加 id 校验与长度校验 |
| 连接"在线"但没数据 | 半开连接；对端卡死 | 心跳 + 读超时 |
| 断线重连后状态不对 | 服务端假设泵状态 | 重连后同步设备状态 |
| 命令执行两次 | 重发无幂等 | action_id + 设备去重 |
| 延迟偶发 40 ms 以上 | Nagle + 延迟 ACK 交互（小包） | 命令/反馈连接开 TCP_NODELAY |
| 收到括号配对错位 | 字符串里的括号/转义没处理 | 用带 in_string 的解析器或 NDJSON |

---

## 15. 面试怎么问、怎么答

**Q1. TCP is reliable — why do you need your own framing?**

> "TCP is a byte stream: it guarantees order and delivery, not message boundaries. One send can arrive as several receives, and several sends can arrive together. So I prefix each message with its length and parse with a small state machine: read the header, then read exactly that many bytes. I also cap the maximum length so a corrupted header can't make the server allocate gigabytes."

（译：TCP 保证有序与送达，不保证消息边界……所以我给每条消息加长度前缀，用小状态机解析，并设最大长度保护。）

追问 1：**What if the length field is corrupted?**
> "The parser would desynchronise. The max-length check catches absurd values; for a plausible but wrong length I would close the connection and let the device reconnect, since resynchronising in a binary stream is error-prone. A magic number can help resynchronise, but for this scale reconnecting is simpler."

追问 2：**What about endianness?**
> "The protocol fixes network byte order. If a little-endian sender were read as big-endian, a length of 100 would be read as about 1.6 billion, which is exactly why the max-length guard matters."

**Q2. Why send JSON and JPEG as separate segments?**

> "The JSON observation is small and decision-critical; the JPEG is large and optional. Separating them lets the server act on the JSON immediately, and if I need to drop images under poor Wi-Fi, the observation is unaffected. They share an event id so the server can pair them, and I avoid base64 inside JSON, which would add a third to the image size."

**Q3. How do you detect that a device went offline?**

> "A TCP connection can stay half-open, so I don't rely on it. The devices report periodically, which doubles as a heartbeat, and the server marks data stale after a timeout. I'd also use a read timeout and reconnect with exponential backoff."

**Q4. When the server sends a pump command, how do you know it worked?**

> "I don't treat a successful send as success. The command carries an action id, the device acknowledges it and reports the actual pump state with that id, and the server records each level separately: requested, acknowledged, reported. Even then, I'd need a flow or soil moisture change to confirm that water actually flowed."

**Q5. Why not MQTT?**
> （见第 11 节加分回答。）

---

## 补充专题：带宽、超时与重连的数字账

协议设计常被讲成"格式问题"，其实一半是"数字问题"：多大的消息、多快的频率、多久算断线。这一节把前面分散的取值汇成一张账，让每个超时都有来由。

### 一、端口与方向一览

| 端口 | 方向 | 内容 | 成帧 | 频率 |
|---|---|---|---|---|
| 8001 | Hi3861 到服务端（命令与 ACK 沿同一连接回传） | 紧凑 JSON：soil、raw、temp、hum、lux、pump、mode、seq | 括号配对 | 每 2 s 一条 |
| 8002 | RK3588 到服务端 | 8 字节头 + JSON（+ JPEG） | 长度前缀 | 事件门通过时发事件；每 1 s 一条心跳 |
| 8765 | 服务端到浏览器 | WebSocket 状态推送 | WebSocket 帧 | 状态变化即推，最低每 1 s |
| 8000 | 服务端到浏览器 | HTTP 静态页 | HTTP | 一次性 |

### 二、带宽账（教学估算，非实测）

- Hi3861 上报：一条约 120 字节，每 2 s 一条，平均约 60 B/s，几乎可以忽略。
- RK3588 心跳：JSON 约 200 字节、无图（M=0），每 1 s 一条，约 200 B/s。
- RK3588 事件：JSON 约 400 字节加一张 JPEG（质量 80，1280x720，约 100–150 KB），事件最短间隔 5 s，同类最多约 30 KB/s；三类并发时上限约 90 KB/s，约合 0.7 Mbit/s。
- 若不做事件门而每帧都发图：150 KB $\times$ 15 FPS $\approx$ 2.2 MB/s，约 18 Mbit/s，在 2.4 GHz Wi-Fi 上与 Hi3861 争信道，会显著增加 Hi3861 的丢包与重传。

结论：事件门既是语义层的去抖，也是网络层的限流；两者其实是同一个设计带来的双重好处。

### 三、超时的来由：从新鲜度倒推

超时不是拍脑袋，而是从上层的新鲜度要求倒推：

| 项目 | 取值 | 推导 |
|---|---|---|
| 传感新鲜度上限 | 10 s | fast path 需要在小于 10 s 的数据上才触发 |
| Hi3861 上报周期 | 2 s | 10 s 内至少应有 5 次上报机会 |
| 读超时（判定离线） | 6 s（3 个周期） | 连续丢 2 次上报仍视为在线（考虑 Wi-Fi 抖动），丢 3 次判离线 |
| 离线后标记过期 | 立即 | 在新鲜度窗口耗尽之前已提前标记，避免 fast path 误用 |
| 重连退避 | 1、2、4、8、16、30 s 封顶 | 避免服务端重启时所有设备同时重连造成风暴 |
| 命令 ACK 超时 | 2 s，最多重试 3 次（2、4、8 s） | 设备正常响应通常在数百毫秒内，2 s 已足够宽松 |

由此可算出**最坏情况下的检测延迟**：设备在两次上报之间的某一刻断电，服务端最晚在 6 s 内判定离线；这比 10 s 的新鲜度窗口更短，所以任何依赖该数据的自动决策都不会基于死数据执行。

### 四、手算：一次服务端重启后的恢复时间

服务端重启耗时约 3 s。设备在重启开始的 0.5 s 处读超时失败（因为写入发现 RST），按退避等待 1 s 后第一次重连，此时服务端尚未就绪，失败；再等 2 s 重连（t=3.5 s 时刻）成功。设备从发现断线到恢复约 3.5 s，加上下一次上报 2 s，服务端大约在 5.5 s 内重新拿到有效传感值。**服务端重启期间不假设泵状态**，重启后在收到第一条带 pump 字段的上报之前，泵状态显示为"未知"，fast path 不会基于"泵当前是关"的假设发命令。

### 五、发送侧的背压与阻塞

Hi3861 的内存很小，发送缓冲有限。若服务端卡住，设备的 `send` 可能阻塞或返回错误。本项目对设备侧的规则是：发送设 2 s 超时，失败即关闭 socket 并进入重连，**绝不在发送失败时阻塞状态机任务**。传感采样与状态机在独立任务中运行，网络任务只读取"最新一条待上报结果"，不排长队：网络差时丢弃旧上报，只保留最新。这与服务端"事件队列满时丢最旧"的策略一致：对状态类数据，最新值比完整历史重要。

RK3588 侧类似：发送线程有一个深度很小的队列（本项目取值为 4），队满时丢弃最旧的 JPEG 而保留事件 JSON，因为 JSON 才是决策依据，图像是辅助。

### 六、协议版本与演进

JSON 内带版本字段 `v`（本项目取值为 1）与 `type`。新增字段是**向后兼容**的：旧解析器忽略未知字段。删改字段则必须升版本号并在服务端同时兼容旧版本一段时间。8 字节头本身不带版本，是因为版本放在 JSON 里更灵活；代价是，一旦头部定义要变（例如把长度扩成 8 字节），就必须同时改两端，所以头部被设计得足够小而稳定。

### 七、切块模糊测试

粘包与半包类缺陷无法靠"跑通"发现。测试方法：把一段由若干完整消息拼成的字节流，用随机切块大小（1 字节、7 字节、随机 1–4096 字节）送入解析器，断言解析出的消息序列与原序列完全一致，并额外注入非法长度（超出上限、负数）与中途断开的流，断言解析器断开连接而不是死循环。每次改协议都必须跑，这条测试是故事 3（TCP 粘包）的防回归手段。

> **面试要点**：被问"你的超时怎么定的"，答："我从上层新鲜度要求倒推：传感数据 10 s 内才可用，设备 2 s 报一次，所以读超时取 3 个周期即 6 s，保证在新鲜度窗口耗尽前就把数据标成过期。命令 ACK 超时 2 s，最多重试 3 次并带同一个 action_id。"

---

## 本章要点速记

1. TCP = 可靠有序字节流，无消息边界；"粘包"是应用层没定义边界。
2. 成帧四法：分隔符、定长、长度前缀、自定界；二进制用长度前缀。
3. 帧：`type + length + payload`；必须约定字节序（网络序）并设最大长度。
4. 解析器有状态：`recv_exact`，循环取出所有完整帧；半包等待，粘包继续。
5. 100 字节 JSON + 8 字节头 = 108；首次 `recv` 只有 20 字节则还缺 88。
6. JSON 与 JPEG 分段，共享 event_id 配对；base64 膨胀 4/3。
7. Hi3861 JSON 流用括号配对（注意字符串与转义）或换行分隔。
8. 心跳 + 读超时 + 指数退避；半开连接不会自己报错。
9. `send` 成功 ≠ 对方收到 ≠ 对方执行 ≠ 泵动；命令带 id 并幂等。

## 自测题

1. 为什么一次 `recv` 不保证拿到完整 JSON？给出至少两种可能。
2. 长度 300 的大端 4 字节表示是什么？按小端误读得多大？
3. 用括号配对解析 `{"a":"x}"}{"b":1}`，若忽略字符串会得到什么错误？
4. 150 KB 的 JPEG 以 base64 嵌入 JSON，增加多少字节？
5. 为什么重连后不能直接假设泵是关的？

**参考答案**：
1. 半包（消息被拆分到多次 recv）；粘包（多条消息被合并进一次 recv）。
2. $300=0x012C$ → `00 00 01 2C`；发送端大端字节 `00 00 01 2C`，若接收端按小端读，则为 $0x2C010000=738{,}263{,}040$（约 7.4 亿）。
3. 在字符串内的 `}` 处错误地认为对象结束，得到 `{"a":"x}` 这种残缺对象。
4. $200{,}000-150{,}000=50{,}000$ 字节。
5. 断线期间泵可能因本地滞回/手动已变化；应请求/等待设备上报状态再更新。

## 相关链接

- 上一章：[07 INT8 量化与 RKNN 转换](./07_INT8量化与RKNN转换.md)
- 下一章：[09 服务端：DataStore、WebSocket 与 asyncio](./09_服务端_DataStore_WebSocket_asyncio.md)
- 命令与反馈：[11 执行边界](./11_执行边界_白名单_幂等与设备反馈.md)
- 基础：[UART、SPI、I2C、以太网与 TCP](../../02_基础知识/11_嵌入式与通信/04_UART_SPI_I2C_以太网_TCP.md)
- 返回：[P4 README](./README.md)
- 工程化（频率、预算与优化）：[20 工程化实践](./20_工程化实践_频率周期与问题解决.md)
