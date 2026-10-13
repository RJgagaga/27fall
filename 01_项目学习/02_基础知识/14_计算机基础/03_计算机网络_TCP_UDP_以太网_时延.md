# 计算机网络：分层模型、TCP 可靠传输与拥塞控制、UDP、时延与吞吐、与 DDS 的对比

> **一句话定位**：网络把"一台机器上的一段数据"可靠（或不那么可靠）地送到"另一台机器上的一个进程"。本章先讲清分层模型和一个数据包的旅程，再把 TCP 的可靠传输、滑动窗口、拥塞控制讲到能手算 BDP、RTO、慢启动窗口序列；然后对比 UDP，最后落到机器人系统：ROS2 的 DDS 为什么基于 UDP 并自己做可靠性，P4 的 TCP 协议为什么要自己做"帧边界"。八股速查见 [计算机网络八股](../../03_八股知识/01_计算机基础/03_计算机网络.md)，嵌入式总线层面的内容见 [UART、SPI、I2C、以太网与 TCP](../11_嵌入式与通信/04_UART_SPI_I2C_以太网_TCP.md)。

**本章学完你能：**

- 画出 TCP/IP 四层（或 OSI 七层）模型，说出每层的数据单位与代表协议，描述封装与解封装；
- 把端到端时延拆成处理、排队、发送（传输）、传播四项，并手算发送时延与传播时延；
- 手算带宽时延积（BDP），解释为什么窗口小于 BDP 时链路用不满；
- 说清 TCP 三次握手/四次挥手、序号与确认、超时重传（RTO）的估计公式、快速重传、滑动窗口、流量控制与拥塞控制（慢启动、拥塞避免、快速恢复）；
- 手算子网划分与 Internet 校验和；
- 解释为什么 TCP 是字节流（会"粘包"）以及长度前缀分帧；
- 对比 TCP、UDP 与 DDS/RTPS 的 QoS，并用英文回答 "Why does ROS2 use UDP-based DDS instead of TCP?"

**前置知识**：[操作系统](./02_操作系统_进程线程_内存_调度_同步.md) 中的套接字与 IO 模型；二进制与十六进制。

**相关项目**：[P4 TCP 通信与协议设计](../../01_项目课程/P4_OpenHarmony智能园艺系统/08_TCP通信与协议设计.md)、[P6 DDS 与 M-DDS 分布式协同](../../01_项目课程/P6_深开鸿M-Robots平台软件/06_DDS与M-DDS分布式协同.md)、[DDS 与分布式通信（基础知识）](../12_机器人软件/05_DDS与分布式通信.md)、[P2 ROS2-CAN 执行链](../../01_项目课程/P2_未来动力双臂遥操作与差速底盘/10_ROS2_CAN执行链.md)。

## 目录

1. 要解决什么问题
2. 符号与术语
3. 分层模型与封装
4. 时延、带宽与带宽时延积
5. IP 与子网、以太网
6. UDP
7. TCP：可靠传输
8. TCP：流量控制与拥塞控制
9. 完整手算（Python 复核）
10. 在七个项目里怎么用
11. 常见错误
12. 英文面试问答
13. 要点速记、自测题、相关链接

---

## 1. 要解决什么问题

两台机器之间隔着铜线、光纤、交换机、路由器。底层只能提供**尽力而为（best effort）**的服务：包可能丢、乱序、重复、延迟变化很大。应用程序想要的却是"把 1 MB 的文件完整、按序送到"或"每 10 ms 送一帧控制指令，慢了就算了"。网络协议栈就是在不可靠的底层上，构建出两种常见的服务：

- **可靠字节流**（TCP）：不丢、不重、按序、有流量与拥塞控制；代价是重传带来的延迟抖动；
- **不可靠数据报**（UDP）：保留消息边界、没有任何保证，开销小、延迟可预测；可靠性（如果需要）由上层负责。

机器人工程师遇到的典型问题：为什么用 TCP 发实时控制指令会偶发卡顿？为什么 ROS2 跨主机通信丢消息？为什么我的 TCP 接收端把两条消息读成一条？这些都能从本章的原理推出来。

## 2. 符号与术语

| 术语 / 符号 | 含义 |
|---|---|
| 报文段 / 数据报 / 帧 / 包 | TCP 的段（segment）、UDP 与 IP 的数据报/包（datagram/packet）、链路层的帧（frame） |
| MTU / MSS | 链路最大传输单元（以太网常见 1500 B）/ TCP 最大报文段数据长度（常见 1460 B = 1500 − 20 IP − 20 TCP，不含选项） |
| $B$ | 带宽（链路速率），bit/s |
| $L$ | 帧长，bit |
| RTT | 往返时间（Round-Trip Time） |
| BDP | 带宽时延积 $=B\times\text{RTT}$，链路上"在途"的数据量 |
| cwnd / rwnd | 拥塞窗口 / 接收窗口（接收方通告） |
| ssthresh | 慢启动阈值 |
| ACK | 确认；累积确认：ACK = $n$ 表示 $n$ 之前的字节都已收到 |
| RTO | 重传超时时间 |
| 吞吐量 vs 带宽 | 吞吐量是实际达到的速率，带宽是理论上限 |
| 端口 | 16 位，标识主机上的进程；(IP, 端口) 构成套接字地址 |
| 五元组 | 协议、源 IP、源端口、目的 IP、目的端口，唯一标识一个连接 |

## 3. 分层模型与封装

### 3.1 为什么分层

每一层只依赖下一层提供的服务，向上一层提供服务，各层可独立替换：应用不用关心下面是以太网还是 Wi-Fi，以太网也不关心上面是 HTTP 还是 DDS。代价是少量的封装开销，以及跨层优化困难。

| TCP/IP 层 | OSI 对应 | 数据单位 | 代表协议 | 解决的问题 |
|---|---|---|---|---|
| 应用层 | 应用/表示/会话 | 消息 | HTTP、MQTT、DDS/RTPS、SSH | 应用语义 |
| 传输层 | 传输 | 段 / 数据报 | TCP、UDP | 进程到进程、可靠性 |
| 网络层 | 网络 | 包 | IP、ICMP | 主机到主机、路由 |
| 链路层 | 数据链路 + 物理 | 帧 | 以太网、Wi-Fi、CAN | 相邻节点传输、介质访问 |

### 3.2 封装

发送时每层加头（链路层还会加尾部校验）：

```text
应用数据:                       [        data        ]
TCP 段:                    [TCP头 |        data        ]
IP 包:               [IP头 | TCP头 |        data        ]
以太网帧: [以太头 | IP头 | TCP头 | data | FCS]
          14 B     20 B   20 B           4 B
```

接收时逐层剥离。路由器只看到网络层，二层交换机只看到链路层。一个 1460 字节负载的 TCP 段，在以太网上总共占用 14（以太头）+ 20 + 20 + 1460 + 4（FCS）= 1518 字节（物理层还有前导码与帧间隙，此处略）。

## 4. 时延、带宽与带宽时延积

### 4.1 端到端时延的四个成分

$$d_{\text{total}}=d_{\text{proc}}+d_{\text{queue}}+d_{\text{trans}}+d_{\text{prop}}$$

- **处理时延** $d_{\text{proc}}$：路由器检查头部、查表，通常微秒级；
- **排队时延** $d_{\text{queue}}$：在缓冲区里等待，**最不确定**，拥塞时可达数十到数百毫秒；
- **发送时延（传输时延）** $d_{\text{trans}}=L/B$：把一帧所有比特推上链路所需时间；
- **传播时延** $d_{\text{prop}}=\text{距离}/\text{传播速度}$：信号在介质中传播，约为光速的三分之二，铜缆与光纤约 $2\times10^8$ m/s。

**手算**（Python 复核）：

- 1500 字节 = 12000 bit。100 Mbit/s 链路：$d_{\text{trans}}=12000/10^8=120\,\mu\text{s}$；1 Gbit/s：$12\,\mu\text{s}$；
- 100 m 铜缆：$d_{\text{prop}}=100/(2\times10^8)=0.5\,\mu\text{s}$；1000 km 光纤：$10^6/(2\times10^8)=5\,\text{ms}$。

结论：局域网里发送时延远大于传播时延（距离短），广域网里传播时延主导。**升级带宽只减少发送时延，对传播时延毫无作用**——跨洋通信的延迟下限由光速决定。

### 4.2 带宽时延积

链路可以看作一根水管：带宽是"粗细"，时延是"长度"，BDP 是管子的"容积"，即**在 ACK 回来之前，发送方必须一直发出去的数据量**，才能把管子填满。

$$\text{BDP}=B\times\text{RTT}$$

**手算**：$B=100\,\text{Mbit/s}$，$\text{RTT}=20\,\text{ms}$：$\text{BDP}=10^8\times0.02=2\times10^6\,\text{bit}=250{,}000\,\text{B}$（Python 复核）。约合 $\lceil250000/1460\rceil=172$ 个满载 MSS 段（Python 复核）。

**意义**：若窗口 $W<\text{BDP}$，吞吐上限是 $W/\text{RTT}$。传统 TCP 窗口字段 16 位，最大 65535 B（未启用窗口缩放选项）；此时 $\text{thr}=65535\times8/0.02=26.2\,\text{Mbit/s}$（Python 复核），远低于 100 Mbit/s 带宽，这就是"长肥管道（long fat network）"问题，解决办法是窗口缩放选项（RFC 7323，具体条文本章未逐条核对）。

> **注意**：停等协议（发一个包等一个 ACK）的链路利用率：1500 B 帧、100 Mbit/s、RTT 20 ms：利用率 $=\frac{120\,\mu\text{s}}{120\,\mu\text{s}+20\,\text{ms}}\approx0.6\%$（Python 复核 0.0059642）。这就是为什么需要滑动窗口流水线。

## 5. IP 与子网、以太网

### 5.1 IPv4 地址与子网

IPv4 地址 32 位，写作点分十进制。`/n` 表示前 $n$ 位是网络号，其余 $32-n$ 位是主机号。一个子网的可用主机数为 $2^{32-n}-2$（去掉全 0 的网络地址和全 1 的广播地址）。

**手算 1**：`192.168.10.0/26`：子网掩码 $26$ 个 1 即 `255.255.255.192`；主机位 6 位，地址总数 64，可用主机 62；网络地址 `192.168.10.0`，广播 `192.168.10.63`，可用范围 `.1`–`.62`。（Python 复核。）

**手算 2**：`10.1.5.77/20`：前 20 位为网络号，第三个字节的前 4 位属于网络号，`5 = 0000 0101`，保留前 4 位为 `0000` 得 0，所以网络地址 `10.1.0.0/20`，广播 `10.1.15.255`。（Python 复核。）

**手算 3（子网划分）**：把 `192.168.1.0/24` 切成 4 个等大小子网：借 2 位，得 4 个 `/26`：`.0/26`、`.64/26`、`.128/26`、`.192/26`。

两台主机通信时，先用掩码判断对方是否同一子网：同子网直接在链路上用 ARP 找到对方 MAC 地址发送；否则发给**默认网关**（路由器）。路由器按**最长前缀匹配**在路由表里选下一跳。

### 5.2 以太网

以太网帧：目的 MAC（6 B）、源 MAC（6 B）、类型（2 B）、负载（46–1500 B）、FCS（4 B，CRC-32）。交换机通过学习源 MAC 地址建立"MAC→端口"表，已知目的 MAC 则单播转发，未知则泛洪。现代有线以太网是全双工、点对点连接，不再有碰撞；CSMA/CD 是共享介质（集线器）时代的机制，常作历史背景。

### 5.3 校验和

Internet 校验和：把数据按 16 位分段，做反码求和（进位回卷），结果取反。接收端对包括校验和字段在内的所有段重新求和，若无错误结果为全 1（取反后为 0）。

**手算示例**（Python 复核）：取一个 20 字节的 IPv4 头 `45 00 00 73 00 00 40 00 40 11 [00 00] c0 a8 00 01 c0 a8 00 c7`（校验和字段先置 0）。计算得校验和 `0xB861`；把它填回去重新校验，结果为 0，说明校验通过。校验和只能发现简单错误，链路层的 CRC 更强，不能防篡改。

## 6. UDP

### 6.1 特性

UDP 头只有 8 字节：源端口、目的端口、长度、校验和。没有连接、没有重传、没有顺序、没有流量或拥塞控制。**保留消息边界**：发送一次 `sendto` 一个数据报，对端一次 `recvfrom` 恰好得到这一个数据报（或根本丢失）。

优点：无握手延迟、头部小、无队头阻塞（head-of-line blocking）、天然支持广播与组播（多播）。缺点：不可靠；应用要自己处理丢包、乱序；缺乏拥塞控制，可能挤占网络。

### 6.2 UDP 的数据报大小

IP 层若包超过 MTU，会被**分片**，任一片丢失则整个数据报丢失，所以实践中 UDP 负载尽量控制在 MTU 以下（以太网常取 1472 B = 1500 − 20 − 8，或更保守）。DDS 对大消息会在 RTPS 层自己分片（不依赖 IP 分片），ROS2 跨主机传输点云、图像时需要注意这一点。

### 6.3 何时选 UDP

适合：实时音视频、遥测、周期性状态发布（新数据覆盖旧数据，丢一个无所谓）；发现协议（组播）；需要自定义可靠性的系统（DDS/RTPS、QUIC 都构建在 UDP 上）。

## 7. TCP：可靠传输

### 7.1 连接建立：三次握手

```text
  客户端                          服务端
    | -- SYN, seq=x ------------->  |   客户端选初始序号 x
    | <- SYN+ACK, seq=y, ack=x+1 -  |   服务端选 y，确认 x
    | -- ACK, ack=y+1 ----------->  |   客户端确认 y
    |        连接建立               |
```

为什么三次而不是两次：(1) 需要双方都确认"我的序号被对方收到了"，即确认两个方向的初始序号；(2) 防止旧的重复连接请求（延迟的 SYN）被服务端误当新请求而建立"半开"连接。第三次 ACK 让服务端知道客户端确实收到了 SYN+ACK 且仍有意愿通信。

### 7.2 连接关闭：四次挥手

TCP 是全双工，每个方向要单独关闭：一方发 FIN 表示"我没数据发了"，对方 ACK；对方等自己的数据发完，再发 FIN，己方 ACK。主动关闭方在最后进入 **TIME_WAIT** 状态，持续 $2\times\text{MSL}$（最大报文段生存时间，常见实现里固定为 60 秒，因系统而异）：(1) 保证最后一个 ACK 丢失时能重传；(2) 让旧连接的残留包在网络中消失，避免被同一五元组的新连接误收。大量短连接的服务器可能积累大量 TIME_WAIT。

### 7.3 序号、确认与累积 ACK

TCP 把数据看成字节流，每个字节有序号。报文段头部的序号是该段第一个字节的序号；ACK 号表示"期望收到的下一个字节序号"。**累积确认**：ACK = $n$ 意味着序号小于 $n$ 的字节都已按序收到。乱序到达的数据暂存，ACK 号不前进（发出重复 ACK）。选择性确认 SACK（选项）可以告知"我还收到了哪些不连续的块"。

### 7.4 超时重传与 RTO 估计

发送后若在 RTO 内没收到 ACK，就重传。RTO 太小造成无谓重传，太大则丢包恢复慢。TCP 用平滑估计（Jacobson/Karels 算法，RFC 6298 描述，本章按常见写法给出，条文未逐字核对）：

$$\text{SRTT}\leftarrow(1-\alpha)\,\text{SRTT}+\alpha\,R,\quad \alpha=\tfrac18$$
$$\text{RTTVAR}\leftarrow(1-\beta)\,\text{RTTVAR}+\beta\,|\text{SRTT}-R|,\quad\beta=\tfrac14$$
$$\text{RTO}=\text{SRTT}+4\,\text{RTTVAR}$$

首次测量 $R_1$：$\text{SRTT}=R_1$，$\text{RTTVAR}=R_1/2$。注意更新 RTTVAR 时用的是**更新前的 SRTT**。

**手算**（RTT 样本 100、120、80、200 ms，示例，非实测；Python 复核）：

| 样本 $R$ | SRTT | RTTVAR | RTO |
|---|---|---|---|
| 100 | 100 | 50 | 300 |
| 120 | $0.875\times100+0.125\times120=102.5$ | $0.75\times50+0.25\times|100-120|=42.5$ | $102.5+170=272.5$ |
| 80 | $0.875\times102.5+0.125\times80=99.6875$ | $0.75\times42.5+0.25\times|102.5-80|=37.5$ | $249.6875$ |
| 200 | $112.2266$ | $0.75\times37.5+0.25\times|99.6875-200|=53.2031$ | $325.0391$ |

观察：RTT 突然变大时 RTTVAR 随之变大，RTO 自动加宽。**Karn 算法**：重传过的包的 RTT 样本不可用于估计（无法判断 ACK 对应哪次发送）。重传时 RTO 指数退避（翻倍）。

### 7.5 快速重传

等待 RTO 太慢。若收到 **3 个重复 ACK**（说明后面的包到了但某个包丢了），不等超时立即重传丢失的段。收到重复 ACK 比超时说明网络"仍在传包"，拥塞比超时轻，所以后续的窗口缩减也更温和。

### 7.6 滑动窗口

发送方允许有多个未确认的段在途，上限是 $\min(\text{cwnd},\text{rwnd})$。窗口随 ACK 滑动。这使得链路可以被流水线式填充（对比停等协议的 0.6% 利用率）。

## 8. TCP：流量控制与拥塞控制

### 8.1 流量控制：保护接收方

接收方在每个 ACK 里通告接收窗口 rwnd（缓冲区剩余空间）。发送方在途数据不得超过 rwnd。若接收应用读得慢，缓冲区满，rwnd 变 0，发送方暂停（并用零窗口探测）。这是**端到端**的，只考虑接收方能力。

### 8.2 拥塞控制：保护网络

拥塞（路由器缓冲满、丢包）是中间网络的问题，接收方并不知道。发送方根据**丢包 / 时延**的信号自己推断并调整拥塞窗口 cwnd。核心思想是 **AIMD**（加性增、乘性减）：没丢包时线性增长（试探可用带宽），丢包时乘性减半（快速退让）。AIMD 被证明能使多个流收敛到公平分享（Chiu–Jain 分析，本章仅引述结论，细节未核对）。

经典 Reno 的四个阶段：

1. **慢启动（slow start）**：cwnd 初始较小（常见初始值 10 个 MSS，历史上是 1–4 个，版本因系统不同），**每个 RTT 翻倍**（每收到一个 ACK 增加 1 MSS），指数增长直到 cwnd 达到 ssthresh。
2. **拥塞避免（congestion avoidance）**：每个 RTT 增加 1 MSS，线性增长。
3. **三个重复 ACK（轻度拥塞）**：快速重传 + 快速恢复，$\text{ssthresh}\leftarrow\text{cwnd}/2$，$\text{cwnd}\leftarrow\text{ssthresh}$（简化叙述），继续拥塞避免。
4. **超时（重度拥塞）**：$\text{ssthresh}\leftarrow\text{cwnd}/2$，$\text{cwnd}\leftarrow1\,\text{MSS}$，重回慢启动。

**手算**（教学简化模型，初始 cwnd=1 MSS，ssthresh=16 MSS，每个 RTT 一轮；Python 复核）：

cwnd 序列：$1,2,4,8,16,17,18,19,\dots$（前 5 轮慢启动，第 5 轮达到阈值后转为拥塞避免，每轮 +1）。若 cwnd=20 时收到三个重复 ACK，则 $\text{ssthresh}=10$，$\text{cwnd}=10$，随后 $11,12,\dots$；若同样在 cwnd=20 时发生超时，则 $\text{ssthresh}=10$、cwnd 回到 1，再次慢启动 $1,2,4,8,10,\dots$。

**稳态吞吐的量级（Mathis 公式）**：$\text{thr}\approx\frac{\text{MSS}}{\text{RTT}}\cdot\frac{C}{\sqrt p}$，$C\approx1.22$（常引用的近似，未核对），$p$ 为丢包率。示例（非实测）：MSS=1460 B，RTT=20 ms，$p=1\%$：$\frac{1460\times8}{0.02}\times\frac{1.22}{0.1}\approx7.1\,\text{Mbit/s}$（Python 复核 7.12）。意义：**即使带宽很大，丢包率 1% 就能把 TCP 吞吐压到几 Mbit/s**，所以无线链路（丢包与拥塞无关）上 TCP 表现差。

现代系统默认拥塞控制算法不止 Reno（Linux 常见 CUBIC，另有基于带宽估计的 BBR），具体默认值因系统和版本不同，未逐一核对。

### 8.3 TCP 是字节流：粘包与分帧

TCP 不保留消息边界。发送端 `send("AAA")`、`send("BBB")`，接收端一次 `recv` 可能得到 `AAABBB`（合并），也可能只得到 `AAAB`（拆分）。这被口语称为"粘包/拆包"，**根源不是 bug 而是协议语义**。（本机回环实测：连续两次 `sendall` 后一次 `recv` 得到 `b'AAABBB'`，Python 复核。）

解决：**应用层分帧**，常用三种：

1. **定长**：每条消息固定长度；
2. **分隔符**：如换行 `\n`（消息内部不能出现分隔符，需转义）；
3. **长度前缀**：头部固定字节写入负载长度，接收端先读头再读 $L$ 字节。

长度前缀示例（Python 复核输出 `[b'abc', b'hello']`）：

```python
# 教学示例，非项目源码：4 字节大端长度前缀分帧
import struct

def frame(msg: bytes) -> bytes:
    return struct.pack("!I", len(msg)) + msg      # ! = 网络字节序(大端)

def parse(buf: bytes):
    out, i = [], 0
    while i + 4 <= len(buf):
        (n,) = struct.unpack("!I", buf[i:i+4])
        if i + 4 + n > len(buf):                  # 半包，等待更多数据
            break
        out.append(buf[i+4:i+4+n]); i += 4 + n
    return out, buf[i:]                           # 返回已解析消息和剩余字节

msgs, rest = parse(frame(b"abc") + frame(b"hello"))
print(msgs)    # [b'abc', b'hello']
```

必须处理三种情形：一个完整帧、多个帧粘在一起、半个帧（等待）。**字节序**也要显式约定，网络字节序为大端；STM32（小端）与 PC 通信时若直接发结构体，需要处理对齐与字节序。

## 9. 完整手算（Python 复核）

本章前面的手算已逐项标注。这里补一个综合题：**机器人经 Wi-Fi 上传点云**。

> 设链路带宽 54 Mbit/s（示例，非项目实测），单帧点云 600 kB（$4.8\times10^6$ bit）。发送时延 $d_{\text{trans}}=4.8\times10^6/5.4\times10^7\approx0.0889\,\text{s}=88.9\,\text{ms}$。若控制频率要求每帧延迟 < 50 ms，则无论 RTT 多小，这条链路都放不下 20 Hz 的点云流，必须压缩、降采样，或只传感兴趣区域。

再算 UDP 分片：600 kB 在 UDP 负载 1472 B 下需 $\lceil600000/1472\rceil=408$ 个数据报；任何一个丢失（RTPS 若配置为可靠则重传）整帧不完整——对传感器流常用"尽力而为"，新帧到来直接覆盖旧帧。

## 10. 在七个项目里怎么用

| 项目 | 网络相关内容 | 性质 |
|---|---|---|
| P4 | 设备端与服务端用 TCP 通信，需要设计应用层协议（消息格式、分帧、心跳、重连）；服务端用 asyncio 处理多连接。分帧与"TCP 是字节流"的原理直接适用，见 [TCP 通信与协议设计](../../01_项目课程/P4_OpenHarmony智能园艺系统/08_TCP通信与协议设计.md) | 项目内 |
| P6 | M-DDS 的设备发现、能力注册、状态同步和任务分发：发现通常靠组播/广播，状态发布靠 UDP 类传输，QoS 决定可靠性。属于分布式协同，见 [DDS 与 M-DDS 分布式协同](../../01_项目课程/P6_深开鸿M-Robots平台软件/06_DDS与M-DDS分布式协同.md) | 项目内 |
| P1/P2/P5 | ROS2 默认 DDS：节点间话题底层是 UDP（同机可走共享内存，依实现与配置）；跨主机时多播发现、网络接口、域 ID（ROS_DOMAIN_ID）会影响能否互相发现 | 项目内（概念） |
| P2/P3/P7 | CAN/FDCAN 是链路层总线，不是 TCP/IP；但"带宽、帧长、时延、仲裁"的分析框架相通。例：标准帧 8 字节数据约 111 位（不含位填充），1 Mbit/s 下约 111 μs；含最坏位填充会更长（约 130 位量级，未核对精确值） | 项目内（类比） |
| 通用 | 不同主机的时钟同步（NTP、PTP）依赖网络时延的对称假设，见 [数据库与分布式基础](./07_数据库与分布式基础_一致性_时钟.md) | 延伸 |

> **面试要点**：当被问"实时控制指令为什么不用 TCP"，可用本章逻辑回答：TCP 保证可靠按序，但丢包后重传和**队头阻塞**会让后到的新指令也被迫等待旧指令，对控制来说"过期的数据没用，最新的更重要"，所以常选 UDP 或带 QoS 的 DDS best-effort。

## 11. 常见错误

1. **把带宽当时延**：升级带宽并不降低传播时延。
2. **TCP 当消息协议用**：不做分帧，在高负载下才暴露粘包。
3. **忽略字节序与结构体对齐**：直接把 `struct` 内存发送，跨平台解析错误。
4. **阻塞 recv 没超时**：对端静默断开（如拔网线）时 TCP 可能很久没有任何错误，需要应用层心跳或 keepalive。
5. **用 UDP 却假设可靠有序**：丢包乱序时状态机卡死。
6. **MTU 导致的黑洞**：大包分片、DF 位 + 中间设备 MTU 更小而被丢弃，表现为"小包通、大包不通"。
7. **TIME_WAIT 堆积**：短连接高频建立，端口耗尽。服务端可使用长连接或设置 `SO_REUSEADDR`。
8. **在 Wi-Fi 上把丢包全算作拥塞**：无线随机丢包让 TCP 误降速。
9. **多机 ROS2 不通**：防火墙阻挡组播/UDP、网卡选择错误、`ROS_DOMAIN_ID` 不一致。
10. **忘记 Nagle 与延迟确认的交互**：小包连续写入出现 40 ms 级延迟时，可考虑 `TCP_NODELAY`（Nagle 算法会合并小包，延迟 ACK 与之叠加；具体延迟值依系统）。

## 12. 英文面试问答

### Q1. Explain how TCP achieves reliable delivery.

"TCP numbers every byte. The receiver sends cumulative acknowledgments saying which byte it expects next. If the sender doesn't get an ACK within a retransmission timeout, or sees three duplicate ACKs, it retransmits. The timeout is estimated from smoothed round-trip time plus four times the variation. A sliding window lets many segments be in flight, and the receiver's advertised window prevents overflowing its buffer. On top of that, congestion control adjusts the sending rate to what the network can carry."

中文要点：字节序号 + 累积 ACK；超时或 3 个重复 ACK 触发重传；RTO = SRTT + 4·RTTVAR；滑动窗口；接收窗口做流量控制；拥塞控制调节速率。

**追问**：

- *"Difference between flow control and congestion control?"* — "Flow control protects the receiver and uses the advertised window; congestion control protects the network and uses a window the sender computes from loss or delay signals. The actual window is the minimum of the two."
- *"Explain slow start."* — "cwnd starts small and doubles every RTT until it hits the threshold, then grows by one segment per RTT. After a timeout it goes back to one segment and the threshold is halved."

### Q2. When would you choose UDP over TCP in a robotics system?

"When fresh data matters more than complete data. For example, a stream of pose or sensor messages at a fixed rate: if one is lost, retransmitting it makes the receiver wait on an old value while newer ones sit behind it. UDP has no head-of-line blocking and a small header, so latency is predictable. If I need reliability for some messages, I can add it at the application layer, which is what DDS does with its QoS settings."

中文要点：新数据比完整数据重要时选 UDP；TCP 重传带来队头阻塞；UDP 延迟可预测；需要可靠时在应用层加（DDS 的 QoS）。

**追问**：*"Then why did you use TCP in your gardening system?"* — "There the messages are low rate and each one matters, for example commands and state updates, and I wanted ordered, reliable delivery with simple connection semantics. Latency at the second level is fine for irrigation. I added my own framing and message format on top because TCP is a byte stream."（需与 P4 实际协议一致；细节见项目章节。）

### Q3. What is the bandwidth-delay product and why does it matter?

"It is bandwidth times round-trip time, the amount of data in flight when the pipe is full. If the window is smaller than that, the sender stalls waiting for ACKs and can't use the whole link. For example, 100 megabit per second with 20 milliseconds RTT gives about 250 kilobytes, so a 64 kilobyte window caps the throughput near 26 megabit per second."

中文要点：BDP = 带宽 × RTT；窗口小于 BDP 则链路用不满；100 Mbit/s、20 ms 得 250 kB，64 kB 窗口只能达到约 26 Mbit/s。

### Q4. TCP is a byte stream. What problem does that cause and how do you handle it?

"Message boundaries are not preserved, so two sends can arrive as one read, or one send as two reads. I add framing at the application layer, usually a length prefix: read the header, then read exactly that many bytes, and buffer partial frames. I also fix the byte order explicitly, usually network byte order."

中文要点：TCP 不保边界；用长度前缀分帧，缓冲半包；显式约定字节序。

## 13. 要点速记、自测题、相关链接

### 要点速记

1. 四层：应用、传输、网络、链路；每层加头封装，逐层剥离。
2. 时延 = 处理 + 排队 + 发送（$L/B$）+ 传播（距离/介质速度）；带宽只改发送时延。
3. BDP $=B\times\text{RTT}$；窗口小于 BDP 则用不满链路。
4. 子网：`/n` 前 $n$ 位为网络号，可用主机 $2^{32-n}-2$。
5. UDP：8 字节头、无连接、保边界、无保证；TCP：字节流、可靠、有序。
6. 三次握手确认双向序号并防旧请求；TIME_WAIT 保证最后 ACK 与清除残留包。
7. RTO $=\text{SRTT}+4\,\text{RTTVAR}$，Karn 算法忽略重传样本，超时指数退避。
8. 拥塞控制 AIMD：慢启动指数、拥塞避免线性；三个重复 ACK 轻度减半，超时重置为 1。
9. 稳态吞吐 $\propto\text{MSS}/(\text{RTT}\sqrt p)$，丢包率对吞吐影响很大。
10. TCP 会"粘包"，用长度前缀分帧并约定字节序；实时指令优先考虑 UDP/DDS best-effort。

### 自测题

1. 1 Gbit/s 链路上 9000 B 的巨型帧发送时延是多少？
   *要点*：$72000/10^9=72\,\mu\text{s}$。
2. 200 Mbit/s、RTT 40 ms，BDP 是多少字节？窗口 128 KB 时吞吐上限？
   *要点*：BDP $=2\times10^8\times0.04/8=10^6$ B；上限 $131072\times8/0.04\approx26.2$ Mbit/s。
3. `172.16.37.9/21` 的网络地址与广播地址？
   *要点*：第三字节 37 = 00100101，前 5 位保留得 00100000 = 32；网络 `172.16.32.0/21`，广播 `172.16.39.255`。
4. SRTT=100，RTTVAR=20，新样本 R=140，求更新后的 RTO（α=1/8，β=1/4）。
   *要点*：RTTVAR=0.75×20+0.25×|100−140|=25；SRTT=0.875×100+0.125×140=105；RTO=105+100=205 ms。
5. 为什么 UDP 比 TCP 更适合周期性遥测？
   *要点*：新数据覆盖旧数据，丢包无需重传；无队头阻塞；延迟可预测；头部小。

### 相关链接

- 八股速查：[计算机网络](../../03_八股知识/01_计算机基础/03_计算机网络.md)
- 上一章：[操作系统](./02_操作系统_进程线程_内存_调度_同步.md)；下一章：[计算机体系结构](./04_计算机体系结构_缓存_流水线_SIMD_GPU.md)
- 相关：[UART、SPI、I2C、以太网与 TCP](../11_嵌入式与通信/04_UART_SPI_I2C_以太网_TCP.md)、[CAN/FDCAN 与电机通信](../11_嵌入式与通信/03_CAN_FDCAN与电机通信.md)、[DDS 与分布式通信](../12_机器人软件/05_DDS与分布式通信.md)
- [模块 README](./README.md)
