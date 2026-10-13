# Linux 与 Shell、Git

> **一句话定位**：机器人软件几乎都跑在 Linux 上（ROS2、RK3588、工控机、服务器）。这一章教你把 Linux 当作"能读懂的系统"来用：文件与权限、进程与信号、管道与日志、ssh 远程、udev 设备命名、systemd 开机自启，以及团队协作必备的 Git 工作流（branch / merge / rebase / cherry-pick / stash / 冲突处理）。每一节都配了可以在纸上手算的例子。

**本章学完你能：**

- 读懂并手算文件权限（`rwxr-xr--` ↔ 754）与 `umask`；
- 解释进程、信号、退出码、前后台，并写出能排障的管道命令；
- 区分 stdout/stderr，正确写 `> file 2>&1`；
- 用 ssh 密钥、`~/.ssh/config`、端口转发、rsync 管理远程机器人；
- 写 udev 规则固定串口/CAN 设备名，写 systemd 单元让程序开机自启并自动重启；
- 画出 Git 提交图，解释 merge 与 rebase 的区别、cherry-pick、stash、reset 的三种模式、reflog 救命；
- 带着冲突标记手动解决 Git 冲突；
- 用英文说清团队里的分支策略与事故恢复。

**前置知识：**基本命令行使用；一点 C/Python 编程经验。

**相关项目：**所有项目的部署与协作环节，尤其 [P2](../../01_项目课程/P2_未来动力双臂遥操作与差速底盘/README.md)（ROS2-CAN）、[P4](../../01_项目课程/P4_OpenHarmony智能园艺系统/README.md)（RK3588 部署）、[P6](../../01_项目课程/P6_深开鸿M-Robots平台软件/README.md)。

## 目录

1. 文件系统与常用命令
2. 权限与用户组
3. 进程与信号
4. 管道、重定向与文本处理
5. 日志与排障
6. ssh 与远程
7. udev：固定设备名
8. systemd：服务管理
9. Git：对象模型与日常工作流
10. merge、rebase、cherry-pick、stash、reset
11. 冲突处理
12. 面试问答
13. 要点速记、自测题、链接

---

## 1. 文件系统与常用命令

### 1.1 目录结构

```text
/           根
├── bin sbin usr   系统程序与库（/usr/local 放自装软件）
├── etc            配置文件（/etc/udev/rules.d、/etc/systemd/system）
├── home/<user>    用户目录（~）
├── dev            设备文件：/dev/ttyUSB0 /dev/video0 /dev/null
├── proc sys       内核信息的虚拟文件系统
├── var/log        日志
├── tmp            临时文件
└── opt            第三方软件（ROS 常装 /opt/ros/<distro>）
```

**"一切皆文件"**：设备、管道、套接字都以文件形式访问，所以 `cat /dev/ttyUSB0`、`echo 1 > /sys/class/gpio/...` 才有意义。

### 1.2 速查表

| 任务 | 命令 | 说明 |
|---|---|---|
| 当前目录/切换 | `pwd` / `cd` | `cd -` 回上一个目录 |
| 列文件 | `ls -lah` | l 详细，a 含隐藏，h 人类可读大小 |
| 查看文件 | `cat` `less` `head -n 20` `tail -n 50` `tail -f` | `tail -f` 追踪新增日志 |
| 复制/移动/删除 | `cp -r` `mv` `rm -r` | `rm -rf` 无确认且不可恢复 |
| 查找文件 | `find . -name "*.launch.py"` | 按名字/大小/时间查 |
| 查找内容 | `grep -rn "pattern" src/` | r 递归，n 显示行号；更快的有 `rg` |
| 磁盘 | `df -h` / `du -sh *` | 文件系统用量 / 目录大小 |
| 内存、CPU | `free -h` / `top` / `htop` | |
| 环境变量 | `export ROS_DOMAIN_ID=7` / `env` / `echo $PATH` | 只对当前 shell 及子进程有效 |
| 软件包 | `apt update && apt install` | Debian/Ubuntu |
| 压缩 | `tar -czf a.tgz dir/` / `tar -xzf a.tgz` | c 创建 x 解压 z gzip f 文件名 |
| 链接 | `ln -s target link` | 符号链接 |
| 看帮助 | `man cmd` / `cmd --help` | |

### 1.3 环境变量与 `PATH`

输入命令 `foo` 时，shell 依次在 `$PATH`（用冒号分隔）的各目录里找可执行文件，找到第一个就执行。`source /opt/ros/humble/setup.bash` 的作用就是把 ROS 的目录加入 `PATH`、`LD_LIBRARY_PATH`、`AMENT_PREFIX_PATH` 等变量——所以**每开一个新终端都要 source**，或写进 `~/.bashrc`。

**手算**：`PATH=/usr/local/bin:/usr/bin:/bin`，两个目录都有 `python3`，实际用的是 `/usr/local/bin/python3`（先找到先用）。`which -a python3` 可以列出全部。

---

## 2. 权限与用户组

### 2.1 它要解决什么问题

多用户系统里要控制"谁能读/写/执行哪个文件"。每个文件有一个属主（user）、一个属组（group），以及三组权限：属主 / 属组 / 其他人，每组 r（读）w（写）x（执行）。

### 2.2 数字表示（手算）

r = 4，w = 2，x = 1，各组相加：

- `rwxr-xr--`：属主 rwx = 4+2+1 = **7**；属组 r-x = 4+0+1 = **5**；其他 r-- = **4** → **754**；
- `chmod 640 key.pem`：属主 6 = rw-，属组 4 = r--，其他 0 = ---（私钥常用）；
- `chmod +x run.sh`：给所有人加执行位；`chmod u+x` 只给属主。

目录的 x 位表示"能进入"（`cd`），r 表示"能列出"。没有目录的 x 权限，即使文件本身可读也进不去。

### 2.3 umask

创建文件时，默认权限（文件 666、目录 777）会减去 `umask`。`umask 022`：文件 666 − 022 = **644**，目录 777 − 022 = **755**。（严格说是按位清除，但在这种数值下相减等价。）

### 2.4 用户组与设备权限

串口 `/dev/ttyUSB0` 通常属组 `dialout`，权限 `crw-rw----`。普通用户要访问需加入该组：

```bash
sudo usermod -aG dialout $USER     # -a 追加，不加会覆盖原有组！
# 重新登录后生效
```

常见"Permission denied: '/dev/ttyUSB0'"就是这个原因。其他例子：`video` 组（摄像头）、`docker` 组。

### 2.5 sudo 与最小权限

`sudo` 以 root 执行。不要把整个开发环境都用 root 跑（出错容易破坏系统，且 ROS 产生的文件属主变成 root 之后普通用户无法修改）。

---

## 3. 进程与信号

### 3.1 进程

每个运行中的程序是一个**进程**，有 PID、父进程 PPID、独立的地址空间、打开的文件描述符。`fork()` 复制出子进程，`exec()` 载入新程序。常用：

```bash
ps aux | grep ros2        # 查进程
top / htop                # 实时占用
kill 1234                 # 默认发 SIGTERM(15)，请求优雅退出
kill -9 1234              # SIGKILL，无法被捕获，强制杀死
pkill -f my_node          # 按命令行匹配杀
```

### 3.2 信号

| 信号 | 编号 | 含义 | 能否捕获 |
|---|---|---|---|
| SIGINT | 2 | Ctrl-C | 能 |
| SIGTERM | 15 | 请求终止（`kill` 默认） | 能 |
| SIGKILL | 9 | 强制杀死 | **不能** |
| SIGSEGV | 11 | 非法内存访问（段错误） | 能（一般不处理） |
| SIGHUP | 1 | 终端断开（也用作"重新加载配置"） | 能 |

好的程序捕获 SIGINT/SIGTERM 做清理（停止电机、关文件）。`kill -9` 不给清理机会，所以机器人程序先 `kill`，不行再 `-9`。**安全相关**：机器人进程被杀时要保证电机进入安全状态（另一层看门狗/驱动器超时保护），不能依赖进程自己"优雅退出"。

### 3.3 退出码

进程结束返回 0–255 的整数：0 = 成功，非 0 = 失败。`$?` 取上一条命令的退出码。`cmd1 && cmd2` 只有 cmd1 成功才执行 cmd2；`cmd1 || cmd2` 失败才执行 cmd2。

### 3.4 前后台、会话

- `cmd &`：后台运行；`jobs`、`fg`、`bg`；
- 关闭终端会向前台作业发 SIGHUP，进程被杀。长任务用 `tmux`/`screen`（会话保持，ssh 断了也在）或 `nohup cmd &`；
- 机器人上的长期服务请用 systemd（第 8 节）。

---

## 4. 管道、重定向与文本处理

### 4.1 三个标准流

每个进程默认有三个文件描述符：0 = stdin（输入），1 = stdout（正常输出），2 = stderr（错误输出）。

- `cmd > out.txt`：stdout 写文件（覆盖）；`>>` 追加；
- `cmd 2> err.txt`：stderr 写文件；
- `cmd > all.txt 2>&1`：先把 stdout 指向 all.txt，再让 stderr 指向"当前 stdout 所指的地方"→ 两者都进文件；
- **顺序重要**：`cmd 2>&1 > all.txt`：先让 stderr 指向"当时的 stdout（终端）"，再把 stdout 改指文件 → stderr 仍在屏幕上。

### 4.2 管道

`A | B`：A 的 stdout 接到 B 的 stdin，字节流式传递，A 和 B 并发运行。注意：管道只传 stdout，stderr 要 `2>&1` 才进管道。

### 4.3 手算：日志统计

文件 `node.log` 内容（每行：级别 节点 消息）：

```text
INFO  ctrl  start
WARN  can   timeout motor 3
ERROR can   bus off
WARN  can   timeout motor 3
INFO  ctrl  tick
WARN  imu   stale data
```

命令 `grep -E "WARN|ERROR" node.log | awk '{print $2}' | sort | uniq -c | sort -rn`：

1. `grep` 保留 WARN/ERROR 行：第 2、3、4、6 行；
2. `awk '{print $2}'` 取第 2 列：`can can can imu`（分别来自 4 行：can、can、can、imu）；
3. `sort`：`can can can imu`；
4. `uniq -c`（计数相邻重复）：`3 can`、`1 imu`；
5. `sort -rn`：按数字降序，最终 `3 can / 1 imu` → CAN 节点告警最多。

### 4.4 常用文本工具

`grep`（匹配）、`sed`（流编辑）、`awk`（按列处理）、`cut`、`sort`、`uniq`、`wc -l`（行数）、`xargs`（把输入变命令参数）、`tee`（既写文件又输出屏幕）、`diff`。

### 4.5 shell 脚本健壮性

脚本开头：

```bash
#!/usr/bin/env bash
set -euo pipefail    # -e 出错即退出；-u 使用未定义变量报错；pipefail 管道中任一步失败整体失败
```

变量引用要加引号 `"$var"`，避免含空格的路径被拆开。

---

## 5. 日志与排障

| 场景 | 命令 |
|---|---|
| 内核消息（驱动、USB 插拔、CAN 错误、OOM） | `dmesg -T | tail -50`（`-T` 人类可读时间） |
| systemd 服务日志 | `journalctl -u myservice -f`（f 追踪）、`--since "10 min ago"` |
| 程序自身日志 | `tail -f`、`grep -C 3 ERROR file`（前后各 3 行） |
| ROS2 日志 | `~/.ros/log/`、`ros2 topic echo`、`ros2 node list` |
| 设备 | `lsusb`、`ls /dev/tty*`、`ip -details link show can0`、`candump can0` |
| 网络 | `ip a`、`ping`、`ss -tlnp`（谁在监听端口）、`tcpdump`、`nc` |
| 资源 | `top`、`free -h`、`df -h`、`iostat` |

**排障习惯**：先复现 → 看日志（时间对齐）→ 缩小范围（硬件/驱动/进程/网络）→ 二分法（关掉一半组件）→ 记录结论。机器人调试要同时记录"现象发生的时刻"，方便和日志、rosbag 对齐。

---

## 6. ssh 与远程

### 6.1 密钥登录

```bash
ssh-keygen -t ed25519           # 生成密钥对 ~/.ssh/id_ed25519 (私钥) 和 .pub (公钥)
ssh-copy-id user@192.168.1.20   # 把公钥追加到远端 ~/.ssh/authorized_keys
ssh user@192.168.1.20
```

原理：远端用你的公钥发起挑战，你用私钥签名证明身份；**私钥永不离开本机**，权限必须是 600，否则 ssh 拒绝使用。

### 6.2 `~/.ssh/config`

```text
Host rk3588
    HostName 192.168.1.20
    User user
    IdentityFile ~/.ssh/id_ed25519
    ServerAliveInterval 30
```

之后 `ssh rk3588`、`scp file rk3588:/tmp/` 即可。

### 6.3 传文件与转发

- `scp -r dir rk3588:/home/user/`；`rsync -avz --delete src/ rk3588:dst/`（增量同步，只传变化部分，`-a` 保留属性，`--delete` 让目标与源一致，**小心用错方向**）；
- 端口转发：`ssh -L 8080:localhost:80 rk3588`，本机访问 `localhost:8080` 即访问远端的 80 端口；
- 首次连接会提示指纹并写入 `~/.ssh/known_hosts`；设备重装系统后指纹变化会报 "REMOTE HOST IDENTIFICATION HAS CHANGED"，确认无误后 `ssh-keygen -R <host>`。

### 6.4 在项目里怎么用

P4：向 RK3588 部署交叉编译产物（见 [平台基础章 9 节](../08_嵌入式与通信/06_OpenHarmony_Hi3861_RK3588平台基础.md)）；P2：在机器人工控机上跑 ROS2 节点、抓 CAN 日志（见 [CAN 章 13 节](../08_嵌入式与通信/03_CAN_FDCAN与电机通信.md)）。

---

## 7. udev：固定设备名

### 7.1 问题

多个 USB 串口/CAN 适配器插到电脑上，系统按枚举顺序命名 `/dev/ttyUSB0`、`ttyUSB1`……**重启或换插口后顺序可能变**，你的程序打开了错误的设备（比如把电机指令发给了 IMU）。

### 7.2 解决：udev 规则

**udev** 是内核设备事件的用户空间管理器：设备插拔时，按规则文件匹配设备属性，创建符号链接、设权限等。步骤：

1. 查设备属性：`udevadm info -a -n /dev/ttyUSB0 | grep -E "idVendor|idProduct|serial"`；
2. 写规则 `/etc/udev/rules.d/99-robot.rules`（示例值为占位，需换成自己设备的）：

```text
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", ATTRS{serial}=="A1234567", SYMLINK+="imu_serial", MODE="0666"
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="remote_serial", GROUP="dialout", MODE="0660"
```

3. 重新加载：`sudo udevadm control --reload-rules && sudo udevadm trigger`；
4. 之后程序统一使用 `/dev/imu_serial`。

**要点**：同型号适配器 VID/PID 相同时，要靠 `serial`（序列号）或 USB 口物理位置（`KERNELS=="1-2.3"`）区分。CAN 适配器（SocketCAN）则用 `ip link` 命名接口，也可以用 udev 或 systemd-networkd 配置固定波特率。

---

## 8. systemd：服务管理

### 8.1 它要解决什么问题

机器人上电后，希望程序**自动启动、崩溃自动重启、有统一日志、有依赖顺序**。手动在终端运行不行。systemd 是主流 Linux 的 init 系统，用"单元（unit）"描述服务。

### 8.2 一个服务单元

`/etc/systemd/system/robot-ctrl.service`：

```ini
[Unit]
Description=Robot control node
After=network-online.target
Wants=network-online.target

[Service]
User=robot
WorkingDirectory=/home/robot/ws
ExecStartPre=/usr/sbin/ip link set can0 up type can bitrate 1000000
ExecStart=/home/robot/ws/install/ctrl/lib/ctrl/ctrl_node
Restart=on-failure
RestartSec=2
Environment=ROS_DOMAIN_ID=7

[Install]
WantedBy=multi-user.target
```

```bash
sudo systemctl daemon-reload        # 改了单元文件后
sudo systemctl enable --now robot-ctrl   # 开机自启并立即启动
systemctl status robot-ctrl
journalctl -u robot-ctrl -f
sudo systemctl restart robot-ctrl
```

### 8.3 要点

- `After=` 只决定顺序，`Wants=/Requires=` 才是依赖；
- `Restart=on-failure` 崩溃后自动拉起，但**机器人安全不能只靠重启**：重启后控制节点会重新使能电机，需要有状态初始化与安全检查；
- 守护进程的日志写到 stdout/stderr 即自动进入 journal；
- 对比 `nohup`/tmux：systemd 管理生命周期、日志、权限和资源限制，更适合产品化。

---

## 9. Git：对象模型与日常工作流

### 9.1 它要解决什么问题

多人同时改同一批代码，需要：保存历史、回到任意版本、并行开发（分支）、合并工作、追责/定位引入 bug 的提交。

### 9.2 对象模型（理解了就不再怕 Git）

Git 内部是一个**内容寻址的对象库**：

- **blob**：文件内容；**tree**：目录（指向 blob/tree）；**commit**：一次快照 = 一个 tree + 父提交（0 个、1 个或 2 个）+ 作者 + 说明；
- 每个对象用 SHA-1/SHA-256 哈希命名；
- **分支只是一个指向某 commit 的可移动指针**；`HEAD` 指向当前分支（或某个 commit）；
- 所以"创建分支"几乎不花代价，"提交"就是新建一个 commit 并把当前分支指针前移。

```text
A---B---C   <- main
         \
          D---E   <- feature      (HEAD 在 feature)
```

### 9.3 日常命令

```bash
git status / git diff / git diff --staged
git add -p                    # 逐块选择要暂存的修改
git commit -m "ctrl: clamp torque to motor limit"
git log --oneline --graph --all
git switch -c feature/x       # 新建并切换分支 (旧写法 checkout -b)
git push -u origin feature/x
git pull --rebase             # 拉取并把本地提交变基到最新
git fetch                     # 只下载不合并
```

### 9.4 团队工作流（常见做法）

1. `main` 保持可用；每个任务一个短分支 `feature/…` / `fix/…`；
2. 小步提交，提交信息写"做了什么、为什么"；
3. 推送后开 Pull Request，代码评审、CI；
4. 合并方式：merge commit、squash 或 rebase-and-merge（团队约定）；
5. **不要提交**：二进制大文件、仿真原始输出、密钥/令牌（用 `.gitignore`；误提交密钥必须视为泄露并更换，仅删除历史不足以保证安全）；
6. 用 tag 标记发布/实验版本（`git tag v0.3`），方便复现实机数据对应的代码版本。

---

## 10. merge、rebase、cherry-pick、stash、reset

### 10.1 merge

把 `feature` 合并进 `main`：

```bash
git switch main
git merge feature
```

两种结果：

- **快进（fast-forward）**：`main` 没有新提交，只是把指针前移；
- **三方合并**：两边都有新提交，Git 找**共同祖先**（merge-base），比较"祖先→main"和"祖先→feature"的差异，合并成一个新的**合并提交**（有两个父提交）。

**手算**：

```text
        C---D   main
       /
A---B
       \
        E---F   feature
```

共同祖先 = B。合并后：

```text
A---B---C---D---M   main        (M 的父提交 = D 和 F)
     \         /
      E-------F     feature
```

M 包含 C、D、E、F 全部修改。若 D 和 F 修改了同一文件的同一行，则冲突，需手工解决（第 11 节）。

### 10.2 rebase

把 `feature` 的提交"搬到" `main` 的最新提交之后，让历史成为一条直线：

```bash
git switch feature
git rebase main
```

```text
合并前:               rebase 后:
A---B---C---D  main   A---B---C---D  main
     \                              \
      E---F  feature                 E'---F'  feature
```

**注意**：E'、F' 是**新的提交**（哈希不同，因为父提交变了）。所以：**已经推送并被他人使用的分支，不要 rebase**（会改写公共历史，别人 pull 时出现重复提交/大量冲突）。自己的本地分支随便 rebase。

交互式变基 `git rebase -i HEAD~3`：合并（squash）、重排、修改说明——整理提交历史的利器。

**merge vs rebase 取舍**：merge 保留真实历史（分叉合并清晰、安全）；rebase 得线性历史（好读、便于 bisect），但改写历史。

### 10.3 cherry-pick

只把某一个提交"复制"到当前分支：

```bash
git switch release/1.0
git cherry-pick 3f2a9c1       # 把那个修复提交挑过来
```

用例：主干上修了个 bug，要把这个修复也放到发布分支。会生成新提交（哈希不同）。

### 10.4 stash

临时放下未提交的修改，去处理别的事：

```bash
git stash push -m "wip: tuning"   # 保存工作区+暂存区修改，工作区变干净
git switch other-branch           # ...做别的事
git switch -
git stash pop                     # 恢复并删除这个 stash（git stash list 查看列表）
```

### 10.5 reset 的三种模式

`git reset <commit>` 移动当前分支指针到 `<commit>`：

| 模式 | 分支指针 | 暂存区 | 工作区 | 用途 |
|---|---|---|---|---|
| `--soft` | 移动 | 保留 | 保留 | 撤销提交但保留改动（重新提交） |
| `--mixed`（默认） | 移动 | 重置 | 保留 | 撤销提交和暂存 |
| `--hard` | 移动 | 重置 | **重置（丢失改动）** | 彻底回到该提交 |

`git revert <commit>`：新增一个"反向提交"来撤销某次提交——**对已公开的历史应使用 revert，而不是 reset**。

### 10.6 reflog：误操作后的救命稻草

`git reflog` 记录 HEAD 的每次移动（保留约 90 天）。执行了错误的 `reset --hard` 或删了分支？`git reflog` 找到之前的 commit 哈希，`git switch -c rescue <hash>` 即可找回。**只要提交过，几乎都能找回；没提交的改动被 `--hard` 抹掉则无法找回**。

### 10.7 二分查找 bug：git bisect

已知 v1 好、当前坏，中间 64 个提交，用 `git bisect start; git bisect bad; git bisect good v1`，Git 每次检出中间提交，你测试后标记 good/bad，最多 $\lceil\log_2 64\rceil=6$ 步定位到引入 bug 的提交。

---

## 11. 冲突处理

### 11.1 冲突怎么来的

两个分支修改了同一文件的相邻/同一位置，Git 无法自动判断保留哪边。合并/变基/cherry-pick/stash pop 都可能冲突。

### 11.2 冲突标记

`git merge feature` 报冲突后，文件里出现：

```text
<<<<<<< HEAD
double max_torque = 18.0;
=======
double max_torque = 12.0;
>>>>>>> feature
```

- `<<<<<<< HEAD` 到 `=======`：**当前分支**（你正在合并进的分支）的版本；
- `=======` 到 `>>>>>>> feature`：**被合并分支**的版本；
- 注意：在 **rebase** 过程中，"HEAD"一侧是上游（你要变基到的分支），另一侧才是你自己的提交——方向和 merge 相反，容易搞混。

### 11.3 解决步骤（手算流程）

1. `git status` 看哪些文件冲突（"both modified"）；
2. 打开文件，**理解双方意图**——不是随便选一边：本例是两人对 `max_torque` 的修改，要问：谁的限制对？依据是电机手册还是实机测试？假设结论取 12.0（更保守的力矩限幅）；
3. 编辑成最终内容，**删除所有标记行**：`double max_torque = 12.0;`；
4. `git add file`，然后 `git merge --continue`（rebase 则 `git rebase --continue`；放弃用 `--abort`）；
5. **编译并运行测试**——冲突解决后代码可能逻辑上不通（两边各自正确，合起来不行）；
6. 复杂冲突用 `git mergetool` 或 IDE 三栏视图（左：你的，右：他的，中：结果）。

### 11.4 减少冲突

小而频繁的提交；经常 `git pull --rebase`；不要对同一文件做大范围格式化；约定模块归属；参数放配置文件而不是散落在代码里。

---

## 12. 面试怎么问、怎么答

**Q1. Explain the difference between git merge and git rebase.**

> Merge combines two branches with a merge commit and keeps the real history. Rebase replays my commits on top of the target branch, giving a linear history, but it creates new commits, so I only rebase my own unpushed branches. In my team workflow I rebase my feature branch on main before opening a pull request, and we merge to main.

【待你补全：你们实际的分支策略（单人仓库/团队 PR/直接提交 main）；如实回答】

**Q2. You pushed a bad commit to main. What do you do?**

> If others may have pulled it, I use git revert to add a commit that undoes it, rather than rewriting history. If it's only on my local branch, I can reset. I use reflog if I lose something by accident. After that I add a test or check so the same mistake is caught earlier.

**Q3. How do you keep a robot program running after boot, and what are the risks?**

> I run it as a systemd service with restart on failure and logs in the journal. A risk is that an automatic restart re-enables actuators, so the node must initialise into a safe state, check feedback, and the motor drivers should have their own timeout to stop if commands disappear.

**Q4. A USB serial device name changes between reboots. How do you fix it?**

> I write a udev rule that matches the vendor ID, product ID, and serial number and creates a stable symlink such as /dev/imu_serial. Then the program opens the symlink, so the device order doesn't matter.

---

## 本章要点速记

1. 权限 r4 w2 x1：754 = rwxr-xr--；umask 022 → 文件 644、目录 755；
2. 串口权限问题 → `dialout` 组，`usermod -aG`（记得 `-a`）；
3. `kill` = SIGTERM 可清理，`kill -9` 不可捕获；机器人安全不能靠进程优雅退出；
4. `cmd > f 2>&1` 与 `cmd 2>&1 > f` 不同，顺序决定去向；
5. 脚本开头 `set -euo pipefail`，变量加引号；
6. ssh 密钥登录，私钥 600，`~/.ssh/config` 简化，rsync 增量同步；
7. udev 规则按 idVendor/idProduct/serial 创建符号链接固定设备名；
8. systemd：`enable --now`、`Restart=on-failure`、`journalctl -u`；
9. Git：分支是指针，commit 是快照；merge 保留历史、rebase 改写历史（不改写公共历史）；
10. cherry-pick 复制单个提交；stash 暂存；reset 三种模式；reflog 救命；公开历史用 revert；
11. 冲突：理解意图再选，删净标记，`add` → `continue`，然后编译测试；
12. 不提交密钥、大文件、原始仿真输出。

## 自测题

1. `chmod 640 a.txt` 后属主、属组、其他分别有什么权限？`umask 077` 下新文件权限？（要点：rw- / r-- / ---；666−077 = 600。）
2. `ls /nonexist > out.txt 2>&1` 与 `ls /nonexist 2>&1 > out.txt` 的 out.txt 内容？（要点：前者含错误信息；后者 out.txt 为空，错误信息仍在终端。）
3. 画出 A-B-C(main)，B-D-E(feature)，merge-base 是什么？合并后 M 的父提交？（要点：B；C 和 E。）
4. 为什么已推送的分支不应 rebase？（要点：重写提交哈希，别人的历史与之分叉，造成重复提交和冲突。）
5. 128 个提交的区间 bisect 最多几步？（要点：⌈log₂128⌉ = 7。）
6. 写出让 `/dev/ttyUSB*` 中序列号 `S123` 的设备命名为 `/dev/motor_uart` 的规则要点。（要点：SUBSYSTEM=="tty"，ATTRS{serial}=="S123"，SYMLINK+="motor_uart"，然后 reload + trigger。）

## 相关链接

- 下一章：[C++ 现代特性与工程实践](./02_C++现代特性与工程实践.md)
- 平台：[OpenHarmony / Hi3861 / RK3588](../08_嵌入式与通信/06_OpenHarmony_Hi3861_RK3588平台基础.md)；CAN 命令行：[CAN 章 13 节](../08_嵌入式与通信/03_CAN_FDCAN与电机通信.md)
- 八股：[Linux、Git、编译构建八股](../../03_八股知识/01_计算机基础/08_Linux_Git_编译构建八股.md)、[操作系统](../../03_八股知识/01_计算机基础/02_操作系统.md)
- 项目：[P2](../../01_项目课程/P2_未来动力双臂遥操作与差速底盘/README.md)、[P4](../../01_项目课程/P4_OpenHarmony智能园艺系统/README.md)
