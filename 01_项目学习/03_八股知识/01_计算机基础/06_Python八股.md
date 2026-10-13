# Python 八股

> **定位**：Python 是机器人脚本、AI 推理、数据处理的主力（CV：Python、LangGraph、MuJoCo/Mink、ROS 节点）。面试关心语言机制：GIL、装饰器、生成器、拷贝、内置数据结构实现、asyncio、多线程与多进程、垃圾回收、类型系统、numpy 广播。

**本章学完你能……**
- 解释 GIL 对多线程的真实影响，并为 CPU 密集/IO 密集任务选对并发模型；
- 手写装饰器、生成器，讲清闭包与迭代器协议；
- 说出 list/dict 的底层实现与复杂度，引用计数 + 分代 GC 的工作方式；
- 讲清 asyncio 事件循环和 numpy 广播规则。

**前置知识**：[操作系统](./02_操作系统.md)（线程与进程）、[计算机网络](./03_计算机网络.md)（IO 多路复用）。工程补充：[Python工程与科学计算](../../02_基础知识/12_机器人软件/03_Python工程与科学计算.md)。

> 说明：P4 用 LangGraph 构建决策链，该框架基于 Python（异步图执行）；P2 用 MuJoCo/Mink 求解约束 IK。我们的做法是：P4 服务端是混合模型——设备数据由阻塞式 TCP 接收线程读入并写进线程安全的 DataStore 快照，WebSocket 推送与 Web 交互跑在 asyncio 事件循环里，两者通过线程安全的方式交接（细节见 [P4 第 09 章](../../01_项目课程/P4_OpenHarmony智能园艺系统/09_服务端_DataStore_WebSocket_asyncio.md)）。并发请求主要是多个网页客户端同时连接，属于 IO 密集，适合 asyncio；决策链中的阻塞调用要放到线程里，避免卡住事件循环。

---

## 一、GIL 与并发

### Q1 什么是 GIL？为什么存在？（GIL）
**优先级：★★★**

**答案要点**
- GIL（Global Interpreter Lock）是 CPython 解释器里的一把全局互斥锁：同一时刻只有一个线程在执行 Python 字节码。
- 存在原因：CPython 的内存管理用**引用计数**，`refcount` 的增减不是原子的；用一把全局锁保护比给每个对象加锁简单且单线程更快，也便于集成非线程安全的 C 扩展。
- 影响：CPU 密集型任务用多线程**不能**加速（甚至更慢，有锁争抢）；IO 密集型任务线程在等待 IO 时会释放 GIL，所以仍有效。
- 调度：线程每执行一段时间（默认约 5ms 的切换间隔 `sys.getswitchinterval()`）被请求释放 GIL。
- C 扩展（numpy、OpenCV）的重计算部分通常会释放 GIL，所以 numpy 多线程可并行。
- 发展：CPython 3.13 起提供实验性的 free-threaded（无 GIL）构建（PEP 703），默认构建仍有 GIL（具体版本与状态以官方发行说明为准，未核对）。

**English short answer**
The GIL is a global lock in CPython that lets only one thread execute Python bytecode at a time. It exists because CPython uses non-atomic reference counting for memory management, and one global lock is simpler and faster for single-threaded code. As a result, threads do not speed up CPU-bound pure Python code, but they work well for IO-bound tasks because the lock is released while waiting. Heavy native libraries such as numpy release the GIL too.

**追问方向**：为什么 `x += 1` 在多线程下仍不安全（字节码级多步）；GIL 与线程安全；free-threaded Python。

### Q2 CPU 密集型和 IO 密集型怎么选并发模型？（Threads vs processes vs asyncio）
**优先级：★★★**

| 场景 | 推荐 | 原因 |
|---|---|---|
| IO 密集，连接数中等 | `threading` / `ThreadPoolExecutor` | 等待时释放 GIL，代码简单 |
| IO 密集，海量连接 | `asyncio` | 单线程事件循环，开销小 |
| CPU 密集（纯 Python） | `multiprocessing` / `ProcessPoolExecutor` | 绕开 GIL，多核并行 |
| CPU 密集（numpy/C 扩展） | 线程或向量化 | 扩展释放 GIL |

**答案要点**：进程间不共享内存，数据需序列化（pickle）通信，有开销；`multiprocessing.shared_memory` 可共享 numpy 数组。多进程在 Windows/macOS 默认用 spawn，需要 `if __name__ == "__main__":` 保护。

**English short answer**
For IO-bound work I use threads or asyncio, because waiting releases the GIL. For CPU-bound pure Python I use multiple processes, each with its own interpreter and GIL, and share big arrays through shared memory to avoid serialization cost. For numeric code I first try vectorization with numpy.

**追问方向**：进程池序列化开销；`concurrent.futures`；协程与线程的比较。

### Q3 Python 线程怎么同步？线程安全吗？（Thread synchronization）
**优先级：★★**

**答案要点**：`threading.Lock/RLock/Condition/Semaphore/Event`；`queue.Queue` 内部带锁，是线程安全的生产者—消费者通道。字典和列表的单个操作（如 `list.append`）因 GIL 在 CPython 中实际是原子的，但"检查再修改"的复合操作不是，仍要加锁（不要依赖实现细节）。`with lock:` 是上下文管理器写法，异常时自动释放。

```python
import threading, queue
q = queue.Queue(maxsize=100)
def producer():
    for i in range(10): q.put(i)
    q.put(None)
def consumer():
    while (x := q.get()) is not None:
        print(x)
threading.Thread(target=producer).start()
threading.Thread(target=consumer).start()
```

**English short answer**
I use locks, events and conditions from threading, and for passing data between threads I prefer queue.Queue, which is already thread safe. Even though single operations like list append are atomic under the GIL, check-then-act sequences are not, so I still protect them with a lock using a with statement.

**追问方向**：死锁；守护线程；`threading.local`。

### Q4 asyncio 的事件循环如何工作？`async/await` 是什么？（asyncio event loop）
**优先级：★★★**

**答案要点**
- 协程（coroutine）是可暂停的函数；`await` 处让出控制权给事件循环。事件循环是单线程的调度器，内部用 `selectors`（底层 epoll/kqueue）监听 IO，就绪后恢复对应协程。**协作式**多任务：一个协程不 await 就会独占线程。
- `asyncio.run(main())` 启动循环；`asyncio.create_task(coro)` 并发调度；`asyncio.gather(*coros)` 等待多个；`asyncio.sleep` 非阻塞等待；`asyncio.Queue` 协程间通信。
- **坑**：在协程里调用阻塞函数（`time.sleep`、`requests.get`、重计算）会卡住整个循环——用 `await asyncio.to_thread(func)` / `loop.run_in_executor` 放到线程池。
- 取消与超时：`asyncio.wait_for`、`task.cancel()` 抛 `CancelledError`。

```python
import asyncio
async def fetch(i):
    await asyncio.sleep(1)          # 模拟 IO
    return i
async def main():
    res = await asyncio.gather(*(fetch(i) for i in range(5)))
    print(res)                       # 总耗时约 1 秒而不是 5 秒
asyncio.run(main())
```

**English short answer**
asyncio runs coroutines on a single-threaded event loop. When a coroutine awaits something, it yields control, and the loop uses epoll-style IO multiplexing to resume whichever coroutine is ready. Multitasking is cooperative, so a blocking call inside a coroutine freezes everything; blocking work must be moved to a thread pool. I use gather or create_task to run several coroutines concurrently.

**追问方向**：与线程相比的优劣；`async` 生成器；任务取消；LangGraph 的异步节点。

### Q5 `asyncio` 与线程、多进程组合时的注意点？（Mixing asyncio with threads）
**优先级：★★**

**答案要点**：事件循环不是线程安全的；从其他线程提交协程用 `asyncio.run_coroutine_threadsafe`，回调用 `loop.call_soon_threadsafe`。CPU 重任务放 `ProcessPoolExecutor`。ROS2 的 `rclpy` 回调与 asyncio 混用时要特别注意线程模型。

**English short answer**
The event loop is not thread safe, so from another thread I schedule work with run_coroutine_threadsafe or call_soon_threadsafe. CPU-heavy tasks go to a process pool via run_in_executor.

**追问方向**：`rclpy` executor；背压。

---

## 二、函数、装饰器、闭包、生成器

### Q6 什么是闭包？`nonlocal` 是什么？（Closures）
**优先级：★★★**

**答案要点**：内部函数引用了外层函数的局部变量，且外层函数返回后该变量依旧可用，即闭包；变量保存在 `__closure__` 的 cell 中。内部赋值外层变量需 `nonlocal` 声明，否则被当作新的局部变量。晚绑定陷阱：`[lambda: i for i in range(3)]` 全部返回 2，因为 `i` 在调用时才取值，修复：`lambda i=i: i`。

```python
def counter():
    n = 0
    def inc():
        nonlocal n
        n += 1
        return n
    return inc
c = counter(); c(); c()   # 2
```

**English short answer**
A closure is an inner function that remembers variables from its enclosing scope even after the outer function returned. To rebind such a variable inside the inner function I need the nonlocal keyword. A classic pitfall is late binding in loops, where all lambdas see the final loop value, and the fix is to bind the value as a default argument.

**追问方向**：`global` 与 `nonlocal`；LEGB 规则。

### Q7 装饰器怎么写？带参数的装饰器？（Decorators）
**优先级：★★★**

**答案要点**：装饰器是接收函数返回函数的可调用对象，`@dec` 等价于 `f = dec(f)`。要用 `functools.wraps` 保留原函数元数据。带参数的是三层嵌套。类装饰器也可用 `__call__`。典型用途：计时、日志、重试、缓存（`functools.lru_cache`）、权限检查。

```python
import functools, time
def timed(label):
    def deco(fn):
        @functools.wraps(fn)
        def wrapper(*args, **kwargs):
            t0 = time.perf_counter()
            try:
                return fn(*args, **kwargs)
            finally:
                print(label, time.perf_counter() - t0)
        return wrapper
    return deco

@timed("step")
def step(x): return x * 2
```

**English short answer**
A decorator is a callable that takes a function and returns a wrapped function, so at-sign dec is shorthand for f equals dec of f. I use functools.wraps to preserve the original name and docstring. A decorator with arguments adds one more level of nesting. I use them for timing, logging, retries and caching.

**追问方向**：多个装饰器的执行顺序（自下而上包装，自上而下调用）；类方法装饰器；`lru_cache` 原理。

### Q8 迭代器、可迭代对象、生成器的区别？（Iterator vs iterable vs generator）
**优先级：★★★**

**答案要点**
- 可迭代对象实现 `__iter__`，返回迭代器；迭代器实现 `__next__`（耗尽抛 `StopIteration`）和 `__iter__`（返回自己）。`for` 循环就是反复调用 `next`。
- 生成器：含 `yield` 的函数，调用后返回生成器对象（是迭代器），**惰性**求值，状态由帧保存，内存 $O(1)$。生成器表达式 `(x*x for x in range(10**9))` 不会占用大内存。
- `yield from` 委托；`send()` 向生成器传值（协程的早期形式，asyncio 前身）。
- 用途：处理大文件逐行读取、传感器数据流管道、无限序列。

```python
def read_frames(path, n):
    with open(path, "rb") as f:
        while chunk := f.read(n):
            yield chunk
```

**English short answer**
An iterable can produce an iterator, and an iterator has a next method that raises StopIteration when finished. A generator is a function with yield that returns an iterator lazily, keeping its local state between calls, so it can stream huge or infinite sequences with constant memory. It is great for processing sensor logs line by line.

**追问方向**：生成器与列表推导式内存对比；`itertools`；协程原理。

### Q9 `*args` 与 `**kwargs`；可变默认参数陷阱？（args, kwargs, mutable default）
**优先级：★★★**

**答案要点**：`*args` 收集多余位置参数为元组，`**kwargs` 收集关键字参数为字典；也用于解包调用。**可变默认参数**只在函数定义时求值一次，被所有调用共享：

```python
def add(x, buf=[]):      # 错误
    buf.append(x); return buf
add(1); add(2)           # [1, 2]  共享了同一个列表

def add(x, buf=None):    # 正确
    if buf is None: buf = []
    buf.append(x); return buf
```

**English short answer**
args collects extra positional arguments into a tuple and kwargs collects keyword arguments into a dict. A default argument is evaluated once when the function is defined, so a mutable default like an empty list is shared across calls. The fix is to use None and create the list inside the function.

**追问方向**：仅限关键字参数 `*`；位置参数 `/`。

### Q10 上下文管理器与 `with`？（Context managers）
**优先级：★★**

**答案要点**：`with` 调用 `__enter__` 和 `__exit__`，保证出异常也释放资源（对应 C++ RAII）。可用 `contextlib.contextmanager` 以生成器写法实现。用途：文件、锁、数据库事务、计时器、临时切换状态（如电机使能块）。

```python
from contextlib import contextmanager
@contextmanager
def motor_enabled(m):
    m.enable()
    try: yield m
    finally: m.disable()
```

**English short answer**
A with statement calls enter on the way in and exit on the way out, even when an exception occurs, so it is Python's version of RAII. I can write my own with a class or with the contextmanager decorator around a generator.

**追问方向**：`__exit__` 返回值；`ExitStack`。

---

## 三、数据模型与内置结构

### Q11 Python 的数据类型：可变与不可变？（Mutable vs immutable）
**优先级：★★★**

**答案要点**：不可变：`int`、`float`、`str`、`tuple`、`frozenset`、`bytes`；可变：`list`、`dict`、`set`、`bytearray`。不可变对象可作 dict 键（可哈希）。`tuple` 本身不可变，但包含的可变元素仍可变。函数参数传递是"传对象引用"（pass by assignment）：修改可变对象会影响调用者，重新赋值不会。`is` 比较身份，`==` 比较值；小整数（-5~256）缓存使 `a is b` 偶然为真，不要依赖。

**English short answer**
Strings, numbers and tuples are immutable, while lists, dicts and sets are mutable. Only hashable immutable objects can be dictionary keys. Arguments are passed as object references, so mutating a list inside a function affects the caller but rebinding the name does not. Use is for identity and equals for value.

**追问方向**：为什么 `list` 不能做 key；`id()`；字符串驻留。

### Q12 深拷贝与浅拷贝？（Shallow vs deep copy）
**优先级：★★★**

**答案要点**
- 赋值 `b = a` 只是多一个名字。浅拷贝（`a.copy()`、`a[:]`、`copy.copy`）创建新容器，但元素仍是原对象的引用；深拷贝（`copy.deepcopy`）递归复制所有层。
- 例：`a = [[1,2],[3]]`；`b = copy.copy(a)`；`b[0].append(9)` 后 `a[0]` 也变了；`copy.deepcopy` 则不会。
- `[[0]*3]*3` 创建三个同一行的引用，修改一个所有行都变；应写 `[[0]*3 for _ in range(3)]`。
- `deepcopy` 能处理循环引用（用 memo），但慢。numpy 的切片是**视图**（共享内存），需 `.copy()`。

**English short answer**
Assignment just adds another name. A shallow copy makes a new container but shares the elements, so mutating a nested list shows through both copies. A deep copy recursively duplicates everything. A frequent bug is creating a two-dimensional list by multiplying a list, which repeats references to the same row.

**追问方向**：numpy 视图与拷贝；`__deepcopy__`。

### Q13 `list` 是如何实现的？复杂度？（list internals）
**优先级：★★★**

**答案要点**：`list` 是**动态数组**，存的是对象指针（所以可混装类型）。`append` 摊还 $O(1)$（over-allocation，约 1.125 倍增长），`pop()` 尾部 $O(1)$，`insert(0,x)`/`pop(0)` 为 $O(n)$（需搬移）；索引 $O(1)$；`x in list` 为 $O(n)$；排序 Timsort $O(n\log n)$，稳定，对已部分有序数据接近 $O(n)$。队列用 `collections.deque`（双端 $O(1)$）。大量数值计算用 `array` 或 numpy 连续存储，避免对象指针开销。

**English short answer**
A Python list is a dynamic array of pointers to objects, so indexing is constant time and append is amortized constant time, but inserting or popping at the front is linear. Membership tests are linear too. For a queue I use collections.deque, and for numerical data I use numpy arrays to avoid per-element object overhead.

**追问方向**：Timsort 的原理；`list` vs `tuple` 内存；`bisect`。

### Q14 `dict` 是如何实现的？为什么 3.7 起有序？（dict internals）
**优先级：★★★**

**答案要点**：哈希表 + **开放寻址**（探测序列带扰动）。3.6+ 采用紧凑实现：一个稀疏的索引数组 + 一个按插入顺序存放的稠密条目数组，因此保持插入顺序（3.7 起为语言规范）。平均查找/插入/删除 $O(1)$。负载因子约 2/3 时扩容。键必须可哈希（实现 `__hash__` 和 `__eq__`，且一致）。遍历中修改大小会 `RuntimeError`。`set` 同样是哈希表。

**English short answer**
A dict is a hash table with open addressing. Since Python 3.6 it stores entries in a compact array in insertion order plus a sparse index table, which also made it ordered by language guarantee from 3.7. Operations are average constant time, and keys must be hashable, meaning their hash never changes during their lifetime.

**追问方向**：哈希冲突；为什么 `list` 不可哈希；`defaultdict`、`OrderedDict`。

### Q15 列表推导式、生成器表达式、`map/filter`？（Comprehensions）
**优先级：★★**

**答案要点**：列表推导式立即生成完整列表，通常比等价的 for+append 快；生成器表达式惰性；字典/集合推导式同理。推导式有自己的作用域，变量不泄漏。过度嵌套降低可读性。

**English short answer**
A list comprehension builds the whole list immediately and is usually faster than an explicit loop with append, while a generator expression produces items lazily and uses constant memory.

**追问方向**：海象运算符 `:=`。

### Q16 `__slots__`、`dataclass`、魔术方法？（slots, dataclass, dunder methods）
**优先级：★★**

**答案要点**：实例默认用 `__dict__` 存属性，灵活但占内存；`__slots__` 固定属性集合，省内存且更快，禁止动态加属性。`@dataclass` 自动生成 `__init__/__repr__/__eq__`，`frozen=True` 不可变，`slots=True` (3.10+)。常用魔术方法：`__init__`、`__repr__`、`__eq__`、`__hash__`、`__len__`、`__getitem__`、`__call__`、`__enter__/__exit__`。定义 `__eq__` 而不定义 `__hash__` 则对象不可哈希。

**English short answer**
By default instances store attributes in a dictionary. __slots__ fixes the attribute set to save memory and speed up access. dataclass generates boilerplate like init, repr and eq. If I define eq without hash, the object becomes unhashable.

**追问方向**：`@property`；描述符。

### Q17 `@staticmethod`、`@classmethod`、实例方法的区别？（Method types）
**优先级：★★**

**答案要点**：实例方法第一个参数 `self`；`classmethod` 第一个参数 `cls`，常用作替代构造器（`from_config`），子类调用时 `cls` 是子类；`staticmethod` 无隐式参数，只是放在类命名空间里的普通函数。`@property` 把方法变成属性访问。

**English short answer**
An instance method receives self, a class method receives the class and is often used as an alternative constructor, and a static method receives nothing and is just a function namespaced in the class.

**追问方向**：描述符协议；多重继承 MRO。

### Q18 多重继承与 MRO、`super()`？（MRO）
**优先级：★★**

**答案要点**：Python 3 使用 C3 线性化确定方法解析顺序（`Class.__mro__`）；`super()` 按 MRO 找下一个类，不是简单的"父类"。菱形继承下每个基类只会被调用一次（前提是都用 `super()` 协作）。

**English short answer**
Python resolves methods with the C3 linearization, visible in the __mro__ attribute. super follows that order rather than just calling the parent, which makes cooperative multiple inheritance work and avoids calling a shared base twice in a diamond.

**追问方向**：Mixin 设计。

---

## 四、内存管理与垃圾回收

### Q19 Python 的垃圾回收机制？（Garbage collection）
**优先级：★★★**

**答案要点**
- **引用计数**为主：对象引用数降为 0 立即释放，优点是即时、确定；缺点是无法处理循环引用，增减计数有开销（GIL 的由来）。
- **分代回收**（generational GC）处理循环引用：对象分三代（0/1/2），新对象在第 0 代，经历多次回收存活则晋升；基于"大多数对象朝生夕死"。对容器对象做可达性分析（标记—清除思路，找出不可达的环）。
- `gc.collect()` 手动触发；`weakref` 弱引用不增加计数；`__del__` 调用时机不确定，不要依赖它释放关键资源，用 `with`。
- 实时敏感的循环中，GC 暂停可造成抖动；可 `gc.disable()` 并手动在空闲时回收，或避免产生循环引用。

**English short answer**
CPython mainly uses reference counting: an object is freed as soon as its count hits zero. To handle reference cycles it adds a generational garbage collector that periodically finds unreachable cycles among container objects, with three generations based on the idea that most objects die young. I avoid relying on __del__ for important cleanup and use context managers instead.

**追问方向**：循环引用例子；weakref；内存泄漏排查（`tracemalloc`）。

### Q20 Python 内存池 / `is` 与小整数缓存？（Memory allocation, interning）
**优先级：★**

**答案要点**：小对象（≤512 字节）由 pymalloc 的内存池分配，减少碎片和系统调用；小整数（-5~256）和部分短字符串被缓存/驻留，所以 `a = 256; b = 256; a is b` 为 True，`257` 则不一定。身份比较请用 `==`，`is` 只用于 `None` 等单例。

**English short answer**
CPython allocates small objects from its own memory pools, and caches small integers and some strings, which is why identity comparison can seem to work by accident. I use equals for values and is only for None.

**追问方向**：`sys.getsizeof`。

### Q21 内存泄漏在 Python 里怎么出现、怎么排查？（Memory leaks）
**优先级：★★**

**答案要点**：常见原因：全局容器/缓存无限增长；循环引用且含 `__del__`（旧版本）；闭包/回调持有大对象；C 扩展泄漏；线程未结束。排查：`tracemalloc` 快照对比、`objgraph`、`gc.get_objects()`、监控进程 RSS。服务端 Agent 长期运行（如 P4 的服务端）要注意会话/历史消息列表无限增长，需截断或窗口化。

**English short answer**
Leaks usually come from unbounded caches or global lists, callbacks that keep large objects alive, or reference cycles. I compare snapshots with tracemalloc to find which lines allocate growing memory. In a long-running agent service, conversation history needs a size limit.

**追问方向**：`lru_cache(maxsize)`；weakref 缓存。

---

## 五、类型系统与语言特性

### Q22 动态类型、鸭子类型、类型注解？（Typing）
**优先级：★★**

**答案要点**：Python 是动态强类型：变量无类型，对象有类型，不做隐式的跨类型转换（`"1"+1` 报错）。鸭子类型：关心"能做什么"而不是"是什么"。类型注解（PEP 484）运行时**不强制**，由 `mypy`/`pyright` 静态检查；`typing` 提供 `Optional`、`Union`、`Protocol`（结构化子类型）、`TypedDict`、`Generic`；`pydantic` 在运行时校验。大项目、LLM 输出结构（如 Agent 的结构化建议）用 `pydantic`/`TypedDict` 约束字段，是常见做法。

**English short answer**
Python is dynamically but strongly typed: names have no type, objects do, and there are no implicit conversions between unrelated types. Type hints are not enforced at runtime, but checkers like mypy use them, and libraries like pydantic validate data. Duck typing means I care about what an object can do rather than its class.

**追问方向**：`Protocol` vs ABC；泛型。

### Q23 `==` 与 `is`；`None` 的判断；真值判断？（is vs ==, truthiness）
**优先级：★★**

**答案要点**：`is None` 而非 `== None`；空容器、0、空串、`None` 为假。陷阱：`if x:` 当 `x=0` 是合法值时会误判，应写 `if x is not None:`。浮点比较用 `math.isclose`。

**English short answer**
I check for None with is, not equals. Because zero and empty containers are falsy, I write is not None when zero is a valid value, such as a joint angle of zero.

**追问方向**：`all()`/`any()`。

### Q24 异常处理：`try/except/else/finally`？（Exceptions）
**优先级：★★**

**答案要点**：`else` 在没异常时执行，`finally` 总执行。只捕获具体异常，不要裸 `except:`（会吞掉 `KeyboardInterrupt`）。`raise ... from e` 保留因果链。自定义异常继承 `Exception`。EAFP（先做后捕获）比 LBYL 更 Pythonic，但在循环热点中异常开销大。机器人脚本里不要吞掉异常导致设备停在危险状态，应在 `finally` 里做安全收尾。

**English short answer**
The else block runs when no exception occurred and finally always runs. I catch specific exceptions rather than a bare except, and I use finally to bring hardware into a safe state even if something fails.

**追问方向**：异常链；`contextlib.suppress`。

### Q25 `import` 机制与模块、包？循环导入？（Imports）
**优先级：★★**

**答案要点**：`sys.modules` 缓存已导入模块，第二次导入直接复用；搜索路径 `sys.path`；包用 `__init__.py`（3.3+ 命名空间包可省略）。循环导入：A 导入 B，B 导入 A，可能得到未初始化的模块；解决：重构公共部分、延迟导入（函数内导入）、只导入模块而非名字。`if __name__ == "__main__":` 区分脚本运行与被导入。

**English short answer**
Imported modules are cached in sys.modules, so a module executes only once. Circular imports can produce partially initialized modules; I fix them by restructuring shared code or importing inside a function. The main guard separates script execution from import.

**追问方向**：虚拟环境；相对导入。

### Q26 `@lru_cache` 与 `functools` 常用工具？（functools）
**优先级：★★**

**答案要点**：`lru_cache(maxsize)` 缓存函数结果（参数需可哈希，适合纯函数，如递归 DP：斐波那契从 $O(2^n)$ 降到 $O(n)$）；`partial` 固定部分参数；`wraps`；`reduce`；`singledispatch`。注意缓存可能无限增长，且对有副作用函数无意义。

**English short answer**
lru_cache memoizes a pure function with hashable arguments, which turns exponential recursion like Fibonacci into linear time. partial fixes some arguments of a function, and wraps preserves metadata in decorators.

**追问方向**：`cache` vs `lru_cache`；方法上的缓存导致实例泄漏。

---

## 六、numpy 与科学计算

### Q27 numpy 为什么比纯 Python 快？（Why numpy is fast）
**优先级：★★★**

**答案要点**：①连续内存、统一类型（无对象指针开销）；②底层 C/Fortran 实现，循环在 C 层；③SIMD 向量化、调用 BLAS/LAPACK；④释放 GIL。关键习惯：**向量化**，避免 Python 级循环；注意中间数组的内存拷贝。`dtype` 选择（float32 vs float64）影响精度与速度。

**English short answer**
numpy stores homogeneous data in contiguous memory and runs loops in compiled code, often with SIMD and optimized linear algebra libraries. So I avoid Python loops and write vectorized expressions that operate on whole arrays at once.

**追问方向**：`einsum`；`numba` 加速。

### Q28 numpy 广播（broadcasting）规则？（Broadcasting）
**优先级：★★★**

**答案要点**
- 规则：形状**从最右维开始对齐**，每一维要么相等，要么其中之一为 1（1 被"拉伸"复制），缺失的维度视为 1。否则报 `ValueError`。
- 例（教学示例）：`A.shape=(3,1)`、`B.shape=(1,4)` → 结果 `(3,4)`；`(5,3)` 与 `(3,)` → `(5,3)`；`(5,3)` 与 `(5,)` → 报错（3≠5），需 `x[:, None]` 把 `(5,)` 变 `(5,1)`。
- 例：对 N 个点（`(N,3)`）都减去质心 `c`（`(3,)`）：`pts - c`；两两距离 `d = np.linalg.norm(a[:,None,:] - b[None,:,:], axis=2)` 形状 `(Na, Nb)`。

```python
import numpy as np
pts = np.random.rand(100, 3)
c = pts.mean(axis=0)            # (3,)
centered = pts - c              # (100,3) - (3,) 广播
```

**English short answer**
Broadcasting aligns shapes from the trailing dimension. Two dimensions are compatible if they are equal or one of them is one, and missing leading dimensions count as one. The smaller array is virtually stretched without copying. For example subtracting the centroid, a vector of three, from an array of N by three points just works, and adding a new axis with None lets me compute pairwise distances.

**追问方向**：`keepdims`；内存占用（两两距离的 $N^2$）。

### Q29 numpy 视图 view 与拷贝 copy？（View vs copy）
**优先级：★★★**

**答案要点**：基本切片（`a[1:3]`）返回**视图**，与原数组共享内存，修改会互相影响；花式索引（整数数组、布尔掩码）返回**拷贝**。`.copy()` 显式拷贝；`np.shares_memory` 检查；`reshape` 在可能时返回视图。函数中就地修改参数数组是常见 bug 来源。

**English short answer**
Basic slicing returns a view that shares memory with the original array, so changing it changes the original, while fancy or boolean indexing returns a copy. When I need independence I call copy explicitly.

**追问方向**：`ravel` vs `flatten`；`C` 与 `F` 内存顺序。

### Q30 numpy 常用操作的坑：矩阵乘法、`dtype`、`axis`？（numpy pitfalls）
**优先级：★★**

**答案要点**：`*` 是逐元素乘，矩阵乘用 `@` / `np.matmul` / `np.dot`；`axis=0` 沿行方向聚合（对每列求值）；整数数组除法/累加溢出（如 `uint8` 溢出回绕）；`np.float32` 与 `float64` 混用；`np.linalg.inv` 尽量用 `solve` 代替求逆（更稳）；坐标变换用齐次矩阵 `(4,4) @ (4,N)`。见 [线性代数](../../02_基础知识/01_数学基础/01_线性代数.md)。

**English short answer**
The star operator is element-wise, and matrix multiplication uses the at operator. axis zero aggregates down the rows, giving one result per column. Integer dtypes like uint8 can silently wrap around, and I prefer solve over explicit matrix inversion for numerical stability.

**追问方向**：`einsum`；条件数。

---

## 七、工程实践

### Q31 Python 性能优化思路？（Performance tuning）
**优先级：★★**

**答案要点**：先 profile 再优化（`cProfile`、`line_profiler`、`time.perf_counter`）。手段：选对算法与数据结构（set/dict 查找）；numpy 向量化；减少属性查找与全局变量；`join` 拼接字符串；生成器节省内存；`lru_cache`；多进程；`numba`/Cython/C++ 扩展（pybind11）；避免在热路径创建大量小对象。

**English short answer**
I profile first to find the real bottleneck. Then I look at algorithm and data structure choices, move numeric loops to numpy, cache pure computations, and for heavy parts use multiprocessing or compiled extensions like numba or pybind11.

**追问方向**：为什么字符串 `+=` 慢；`__slots__`。

### Q32 虚拟环境与依赖管理？（Virtual environments）
**优先级：★★**

**答案要点**：`venv`/`conda` 隔离依赖；`requirements.txt`/`pyproject.toml` 固定版本；可复现性：锁定版本、记录 Python 版本。ROS 与 Python 环境冲突时，注意 `source setup.bash` 与虚拟环境的先后以及 `PYTHONPATH`。

**English short answer**
I use virtual environments to isolate dependencies and pin versions in a requirements or pyproject file for reproducibility. With ROS I pay attention to the order of sourcing the workspace and activating the environment.

**追问方向**：conda vs pip；Docker。

### Q33 日志、测试、代码规范？（Logging, testing, style）
**优先级：★★**

**答案要点**：用 `logging` 模块（级别、格式、handler）而不是 `print`；`pytest` 写单元测试，`fixture` 管理准备；`mock` 替换硬件；`black/ruff` 格式化与 lint；类型检查 `mypy`。机器人代码把硬件依赖放在接口后面，测试时换成仿真实现。

**English short answer**
I use the logging module with levels instead of print, write unit tests with pytest and mock hardware interfaces, and run a formatter and linter in CI. Keeping hardware behind an interface makes the logic testable.

**追问方向**：参数化测试；覆盖率。

### Q34 Python 如何调用 C/C++？ctypes、pybind11、Cython？（Calling C/C++）
**优先级：★★**

**答案要点**：`ctypes`/`cffi` 加载动态库，不需编译扩展，但类型要手写；`pybind11` 用 C++ 模板绑定，易用（Pinocchio、Mink 依赖的库有 Python 绑定）；Cython 把类 Python 代码编译成 C。传递 numpy 数组可零拷贝共享缓冲区。

**English short answer**
ctypes loads a shared library without compiling anything but needs manual type declarations. pybind11 binds C++ code with templates and is very convenient, and many robotics libraries expose Python bindings that way. Cython compiles Python-like code to C. Numpy arrays can be shared without copying.

**追问方向**：GIL 释放；缓冲协议。

### Q35 Python 中的随机数、浮点、整数大小？（Numbers）
**优先级：★**

**答案要点**：`int` 为任意精度，不会溢出（但速度慢，与 C 不同）；`float` 是 IEEE 754 双精度；`random.seed` 保证可复现；`//` 向负无穷取整，`%` 结果与除数同号，与 C 的 `/`、`%` 不同；`round` 是银行家舍入。

**English short answer**
Python integers have arbitrary precision so they do not overflow, floats are IEEE double precision, floor division rounds toward negative infinity, and round uses banker's rounding. For reproducible experiments I set the random seed.

**追问方向**：`decimal`；大数乘法复杂度。

### Q36 `yield` 与 `return`、协程与生成器的联系？（yield vs return）
**优先级：★★**

**答案要点**：`return` 结束函数；`yield` 暂停并保留栈帧，下次从暂停处继续。`async def` 协程基于生成器的思想：`await` 类似 `yield from`，由事件循环驱动。理解这一点就理解了 asyncio 为什么是"协作式"。

**English short answer**
return ends a function, while yield suspends it and keeps its state so it can resume later. Coroutines are built on the same idea, with await suspending the coroutine and the event loop resuming it.

**追问方向**：`send()`；`async for`。

### Q37 Python 在机器人开发中的典型位置与限制？（Python in robotics）
**优先级：★★**

**答案要点**：适合原型、数据分析、仿真脚本（MuJoCo）、AI 推理（PyTorch/ONNX/RKNN-Toolkit）、ROS2 `rclpy` 上层逻辑、LangGraph 决策。不适合硬实时闭环（GIL、GC 暂停、解释器开销），这类放 C++/MCU。常见架构：Python 做感知/决策，输出目标给 C++/MCU 控制闭环。与 CV 一致：P4 的 Agent 做决策建议，设备端 Hi3861 做本地泵状态机闭环；P2 的约束 IK 由 Python（MuJoCo/Mink）求解。

**English short answer**
Python is great for prototyping, simulation, perception and high-level decision making, but not for hard real-time control loops because of the GIL, garbage collection pauses and interpreter overhead. A common architecture is Python for decisions and perception, with the fast control loop in C++ or on a microcontroller.

**追问方向**：延迟预算；rclpy 回调线程；PREEMPT_RT。

### Q38 `async` 版 HTTP 服务框架与 WebSocket 推送，怎么选？（Web frameworks）
**优先级：★**

**答案要点**：Flask 同步轻量；FastAPI 基于 ASGI + 类型注解 + 自动文档，原生支持 async 与 WebSocket；Django 全栈。Agent 服务端常用 FastAPI（典型做法）。生产部署用 uvicorn/gunicorn 多 worker；注意在 async 路由里不要写阻塞调用。

**English short answer**
Flask is simple and synchronous, FastAPI is built on ASGI with type hints and native async and WebSocket support, and Django is a full-stack framework. For an agent backend that streams updates, FastAPI is a typical choice, as long as I do not block the event loop.

**追问方向**：WSGI vs ASGI。

### Q39 `property` 与描述符（descriptor）是什么？（property and descriptors）
**优先级：★★**

**答案要点**
- `property` 把方法伪装成属性：读 `obj.x` 时调用 getter，赋值 `obj.x = v` 时调用 setter，可在 setter 里做校验（例如关节角限幅），对外接口不变。
- 本质是**描述符**：实现了 `__get__`、`__set__`、`__delete__` 之一的类，放在类属性位置时，属性访问会被它接管。函数本身也是描述符（`__get__` 把函数绑定成方法，所以 `self` 能自动传入）。
- 区分**数据描述符**（定义了 `__set__`）与**非数据描述符**：属性查找顺序为 数据描述符 > 实例 `__dict__` > 非数据描述符/类属性。
- 机器人场景：配置对象里用 `property` 做单位换算与范围检查；`dataclass` 字段、ORM 字段、`functools.cached_property` 都建立在描述符之上。

```python
# 教学示例: 带校验的 property
class Joint:
    def __init__(self): self._q = 0.0
    @property
    def q(self): return self._q
    @q.setter
    def q(self, v):
        if not -3.14 <= v <= 3.14: raise ValueError("out of range")
        self._q = v
```

**English short answer**
A property lets me expose a method as an attribute, so I can validate or compute a value on access without changing the interface. Under the hood it is a descriptor, an object with get and set methods that Python calls when the attribute is accessed on a class. Functions are descriptors too, which is how self gets bound to methods.
（中文：property 把方法当属性用，可在访问时做校验或计算而不改接口；它本质是描述符，函数也是描述符，所以方法能自动绑定 self。）

**追问方向**：`__getattr__` 与 `__getattribute__`；`cached_property`；属性查找顺序。

### Q40 `multiprocessing` 如何通信？`fork` 与 `spawn` 的区别？（Multiprocessing and IPC）
**优先级：★★**

**答案要点**
- 多进程绕过 GIL，各进程有独立解释器与内存，适合 CPU 密集（图像预处理、批量规划）。代价：进程启动慢、数据需**序列化（pickle）**传输。
- 通信方式：`Queue`/`Pipe`（pickle 后经管道，有拷贝开销）；`multiprocessing.shared_memory` 或 `Array/Value`（共享内存，零拷贝，需自己同步，配合 numpy `frombuffer` 使用）；`Manager`（代理对象，慢但方便）。
- 启动方式：`fork`（Linux 默认，复制父进程，快，但在已有多线程、CUDA、持有锁时 fork 可能死锁或出错）；`spawn`（Windows/macOS 默认，新起解释器并重新 import 主模块，所以入口必须写 `if __name__ == "__main__":`）；`forkserver`。
- 常用 `concurrent.futures.ProcessPoolExecutor`；传给子进程的函数和参数必须可 pickle（lambda、局部函数不行）。
- 工程建议：大数组用共享内存传递，只传小消息；子进程异常要在父进程收集；退出时 `join` 并处理僵尸进程。

**English short answer**
Multiprocessing sidesteps the GIL by running separate interpreters, so it suits CPU-bound work. Processes talk through queues and pipes, which pickle the data, or through shared memory for large arrays without copying. Fork copies the parent process and is fast but unsafe with threads or GPU contexts, while spawn starts a fresh interpreter, which is why the entry point needs a main guard.
（中文：多进程绕过 GIL，适合 CPU 密集；通过队列/管道（pickle 拷贝）或共享内存（大数组零拷贝）通信；fork 快但与线程/GPU 混用不安全，spawn 起新解释器，所以入口要有 main 保护。）

**追问方向**：pickle 限制；共享内存同步；与线程池的取舍。

### Q41 `str`、`bytes`、`bytearray` 与 `struct`：怎么处理二进制协议？（Bytes and struct）
**优先级：★★**

**答案要点**
- `str` 是 Unicode 文本，`bytes` 是不可变字节序列，`bytearray` 是可变字节序列；二者通过 `encode()`/`decode()` 转换，需显式指定编码（常用 UTF-8），混用会 `TypeError`。
- 解析串口/CAN/TCP 二进制帧用 `struct.pack/unpack`：格式串 `"<BBhf"` 中 `<` 小端、`>` 大端、`!` 网络序；`B` 1 字节无符号、`h` 2 字节有符号、`f` 4 字节浮点。字节序与对齐是常见错误源（带 `<`/`>` 前缀时不做对齐填充，默认 `@` 会按本机对齐填充）。
- `memoryview` 可零拷贝切片大缓冲；numpy 的 `np.frombuffer` 可把字节流直接当数组。
- 校验：CRC 用 `binascii.crc32` 或自写查表；TCP 是字节流，要自己处理粘包/半包（长度前缀或帧头+长度+CRC）。

```python
# 教学示例: 解析一帧 [类型 B][长度 B][float 数据]
import struct
kind, n, val = struct.unpack("<BBf", frame[:6])
```

**English short answer**
Text is str and raw data is bytes, and I convert explicitly with an encoding such as UTF-8. For binary protocols I use struct with an explicit byte order prefix to pack and unpack fixed-layout frames, and memoryview or numpy frombuffer to avoid copies. Because TCP is a byte stream, I also need framing with a length and a checksum.
（中文：文本用 str，原始数据用 bytes，显式指定编码转换；二进制协议用 struct 并显式写字节序前缀，用 memoryview/frombuffer 避免拷贝；TCP 是字节流，需要自己做长度加校验的分帧。）

**追问方向**：大小端；粘包处理；`ctypes.Structure`。

---

## 本章要点速记
1. GIL：同一时刻一个线程跑字节码；CPU 密集用多进程，IO 密集用线程/asyncio。
2. asyncio 单线程协作式，阻塞调用会卡死事件循环。
3. 装饰器 = 接收函数返回函数；加 `functools.wraps`。
4. 生成器惰性、$O(1)$ 内存；迭代器协议 `__iter__/__next__`。
5. 可变默认参数只求值一次；闭包晚绑定。
6. 浅拷贝共享内层对象；numpy 基本切片是视图。
7. list = 动态数组，dict = 开放寻址哈希表且保持插入顺序。
8. 引用计数 + 分代 GC 处理循环引用；用 `with` 不靠 `__del__`。
9. numpy 广播：从右对齐，相等或为 1。

## 自测题
1. 为什么多线程不能加速纯 Python 的 CPU 密集任务？怎样加速？
2. 写出一个带参数的重试装饰器。
3. `a=[[0]*3]*3; a[0][0]=1` 的结果是什么，为什么？
4. 形状 `(4,1)` 与 `(3,)` 相加得到什么形状？
5. 描述 CPython 如何回收一个循环引用。
6. 在 asyncio 协程里调用 `time.sleep(1)` 会发生什么？

## 相关链接
- 上一章：[C++八股](./05_C++八股.md)　下一章：[并发多线程与实时系统八股](./07_并发多线程与实时系统八股.md)
- 相关：[Python工程与科学计算](../../02_基础知识/12_机器人软件/03_Python工程与科学计算.md)、[P4 项目 README](../../01_项目课程/P4_OpenHarmony智能园艺系统/README.md)、[八股总入口](../README.md)
