# 质心动力学、LIPM、ZMP 与足式/轮足建模

> **一句话定位**：当机器人"站在地上"（足式、轮足、轮腿、球形底盘），它能否保持平衡，归根结底取决于一件事：**接触力能否把质心推到想去的地方，而且接触力本身是否合法**（不拉地面、不超过摩擦锥、作用点在支撑区域内）。质心动力学、线性倒立摆模型（LIPM）、零力矩点（ZMP）和捕获点（capture point）是回答这个问题的四件工具；轮腿机器人则是把"脚"换成"轮子"后的变体。

**本章学完你能：**

- 从牛顿—欧拉方程推出质心动量方程，说明为什么"控制质心 = 控制接触力之和"；
- 不跳步地推导 LIPM 方程 $\ddot x=\frac{g}{z_c}(x-p)$，并解出 $\cosh/\sinh$ 形式的轨迹和稳定/不稳定模态；
- 定义 ZMP 与压力中心（COP），手算多接触点下的 COP，并判断给定加速度下 ZMP 是否越界；
- 推导捕获点（capture point）$\xi=x+\dot x/\omega$ 满足 $\dot\xi=\omega(\xi-p)$，用它判断"不迈步能否停下"；
- 建立轮摆（wheel–pendulum）简化模型，推出极点 $\pm\sqrt{g/L}$，解释变腿长带来的 $-2(\dot L/L)\dot\theta$ 项，并设计一个 PD 恢复控制器、手算并用 Python 仿真验证；
- 说明轮地接触下 ZMP 约束为什么退化、哪些约束接替它；
- 用英文回答 "What is ZMP / what is the capture point / why does leg length change the balance problem" 等问题。

**前置知识**：[牛顿—欧拉与刚体力学](./01_牛顿欧拉与刚体力学.md)（力、力矩、质心）、[倒立摆建模与线性化](./03_倒立摆建模与线性化.md)（线性化、极点）、[机器人动力学进阶](./05_机器人动力学进阶_操作空间_接触_阻抗.md)（接触、摩擦锥，第 8、9 节）、[状态空间](../05_控制理论/03_状态空间_可控可观_稳定性.md)（极点与稳定性）。

## 目录

1. 问题：站着也是动力学问题
2. 符号、单位与坐标系
3. 质心动量方程
4. LIPM 的推导
5. ZMP 与压力中心（COP）
6. LIPM 的解与手算
7. 捕获点（capture point）
8. 变高度质心与步行控制概览
9. 轮摆模型：轮足/轮腿的简化
10. 变腿长对平衡的影响
11. 手算：轮摆的 PD 恢复与约束
12. 轮足与足式的约束对比
13. 在本项目里怎么用
14. 常见错误与故障现象
15. 面试怎么问、怎么答
16. 本章要点速记
17. 自测题
18. 相关链接

---

## 1. 问题：站着也是动力学问题

一个机械臂固定在桌子上，它的动力学方程 $M\ddot q+C\dot q+g=\tau$ 里，基座是不动的：不管你怎么用力，基座都能提供任意大小的反力。但球形机器人（ballbot）、轮腿机器人、人形与四足机器人的"基座"是浮动的：地面只能通过有限的接触力去推它。这件事带来三个新问题。

**第一，谁来保持不倒？** 机器人没有固定在地上，它倒不倒取决于质心（center of mass，CoM）的运动，而质心的加速度只由外力决定（重力和接触力）。内部关节再怎么转，只要接触力之和不变，质心就不会多动一厘米。所以平衡问题天然是"质心 + 接触力"问题。

**第二，接触力有限制。** 地面只能推、不能拉（单边约束）；推的方向不能偏离法线太多（摩擦锥）；对于有脚掌的机器人，合力的作用点必须落在脚掌范围内，否则脚会绕边缘翻转。

**第三，模型要简化到能设计控制器。** 完整的多体模型有几十个自由度，不可能直接用来推公式。研究者发现一个很有效的降维：**把机器人当成"一个质点 + 一根无质量的腿"**，质心高度近似常数，这就是线性倒立摆模型（LIPM）。轮足与轮腿机器人则用"轮 + 摆"的类似降维。

学这一章，你应当获得的能力是：看到一个站立平衡的机器人，能马上说出"它的质心在哪、不稳定极点是多少、要靠什么力把它拉回来、这个力会不会超出地面能给的上限"。

## 2. 符号、单位与坐标系

| 符号 | 含义 | 单位 |
|---|---|---|
| $m$ | 整机质量 | kg |
| $c=(x_c,y_c,z_c)$ | 质心位置（世界系） | m |
| $g$ | 重力加速度，取 $9.81$ | m/s² |
| $f_i$ | 第 $i$ 个接触点对机器人的力 | N |
| $p_i$ | 第 $i$ 个接触点位置 | m |
| $L_G$ | 绕质心的角动量 | kg·m²/s |
| $p=(p_x,p_y)$ | ZMP（或 COP）位置 | m |
| $\omega=\sqrt{g/z_c}$ | LIPM 的特征频率 | rad/s |
| $\xi=x+\dot x/\omega$ | 捕获点（divergent component of motion，DCM，一维时） | m |
| $L$ | 轮腿的腿长（轮心到质心的距离） | m |
| $\theta$ | 腿（或摆）相对竖直的倾角，向前为正 | rad |
| $a$ | 轮（支点）的水平加速度，向前为正 | m/s² |
| $\mu$ | 轮地摩擦系数 | 无量纲 |

**坐标约定**：世界系 $x$ 向前、$z$ 向上，重力 $g=(0,0,-g)$。本章为简化只在矢状面（$x$–$z$ 平面）内推导，侧向（$y$）完全对称。

## 3. 质心动量方程

### 3.1 推导

把机器人看成许多质点 $m_k$ 的集合，每个质点受外力 $F_k$（含重力、接触力）和内力。对所有质点求和，内力成对抵消（牛顿第三定律），得到线动量定理：

$$m\ddot c=\sum_if_i+mg.$$

对角动量，取质心为参考点：质点相对质心的位置为 $r_k=x_k-c$，则 $L_G=\sum_kr_k\times m_k\dot r_k$，其变化率等于外力对质心的力矩：

$$\dot L_G=\sum_i(p_i-c)\times f_i+\sum_i\tau_i.$$

其中 $\tau_i$ 是接触点处的纯力矩（脚掌与地面的接触通常可以等效成作用点上的一个力加一个绕垂直轴的摩擦力矩）。重力作用于质心，对质心不产生力矩，所以不出现。

### 3.2 结论与意义

1. **质心加速度只与接触力之和有关**：$\ddot c=\frac1m\sum f_i+g$。
2. 一旦忽略内部运动引起的角动量变化（$\dot L_G\approx0$），接触力就必须满足 $\sum(p_i-c)\times f_i=0$：也就是总接触力的作用线必须过质心。这个条件就是后面 ZMP 公式的来源。
3. 现代全身控制里，把线动量与角动量合成"质心动量" $h_G=(m\dot c,\ L_G)$，并写成 $h_G=A_G(q)\dot q$，$A_G$ 称为质心动量矩阵（centroidal momentum matrix）。这样就能在不展开所有关节动力学的前提下，把"质心要怎么动"变成对关节速度的线性约束。本章不展开 $A_G$ 的构造，只需记住：**质心动量层是连接整机动力学与降维模型的桥**（构造方法在文献中有标准推导，此处不做逐条核对）。

> **面试要点**：为什么平衡控制可以只看质心？因为内力不改变系统总动量，外力只有接触力和重力。控制质心的本质，是控制接触力的合力和合力矩。

## 4. LIPM 的推导

### 4.1 假设

1. 质心高度恒定：$z_c=\text{const}$，所以 $\ddot z_c=0$；
2. 忽略绕质心的角动量变化：$\dot L_G=0$；
3. 机器人等效为"集中质量 $m$ + 无质量伸缩腿"，腿的末端落在 ZMP 点 $p$（地面上），只在 $x$–$z$ 平面内运动。

```text
          ● c (m)               z
         /|                     ↑
        / |  z_c                |
       /  |                     +---→ x
  ----p-----------  ground
      ZMP
```

### 4.2 推导

取地面上 ZMP 点 $p$ 为参考点，ZMP 的定义是：绕该点合外力矩（水平分量）为零。作用在系统上的力有重力 $mg$（作用于质心）和惯性力 $-m\ddot c$。把牛顿第二定律写成达朗贝尔平衡形式，对 $p$ 取矩（只取绕 $y$ 轴的分量）：

重力的力矩：质心在 $x_c$，重力向下，对 $p$ 的力臂为 $(x_c-p)$，力矩（绕 $y$ 轴，向前倾为正）为 $mg(x_c-p)$；惯性力 $-m\ddot x_c$ 水平，作用高度 $z_c$，力矩为 $-m\ddot x_cz_c$（水平向后的惯性力使机体后仰）。平衡：

$$mg(x_c-p)-m\ddot x_cz_c=0\ \Longrightarrow\ \boxed{\ddot x_c=\frac{g}{z_c}(x_c-p)=\omega^2(x_c-p)}.$$

反解 ZMP：$p=x_c-\dfrac{z_c}{g}\ddot x_c$。

### 4.3 直觉

方程说的是：若质心在 ZMP（脚）前方（$x_c>p$），它就会向前加速，离开 ZMP 越远加速越快——这是一个**不稳定**的正反馈（像把铅笔立在手指上）。唯一能"抓住"它的办法是移动 ZMP：让 ZMP 跑到质心前方，质心就会向后减速。控制量因此是 ZMP 的位置 $p$（实际由脚踝力矩或迈步实现）。

### 4.4 变高度与有角动量时的推广

保留 $\ddot z_c\ne0$ 时，推广成 $\ddot x_c=\dfrac{g+\ddot z_c}{z_c}(x_c-p)$（变高度倒立摆，VHIP 的一种形式）。保留角动量时 $p=x_c-\dfrac{z_c}{g}\ddot x_c+\dfrac{\dot L_G}{mg}$，多出的一项就是"上身摆动补偿"：上身快速前倾能在不移动脚的情况下产生一个等效的 ZMP 偏移。

## 5. ZMP 与压力中心（COP）

### 5.1 定义

地面反力的分布可以等效为一个合力 $f$ 作用在地面某一点。使该合力对地面的水平力矩为零的点叫压力中心（center of pressure，COP）。在 LIPM 的平面假设下，**ZMP 与 COP 是同一个点**（当 ZMP 在支撑区域内时严格相等；若按公式算出的 ZMP 在支撑区域之外，物理上的 COP 只能停在边缘，多出来的力矩使机器人绕边缘转动，也就是开始翻倒）。

对于多个接触点，COP 就是法向力加权的平均位置：

$$p=\frac{\sum_ip_if_{z,i}}{\sum_if_{z,i}}.$$

### 5.2 手算：双脚支撑

双脚在 $x=-0.1$ m 与 $x=+0.1$ m，法向力分别为 $300$ N 与 $200$ N（示例，非项目实测）：

$$p=\frac{-0.1\times300+0.1\times200}{500}=-0.02\ \text{m}.$$

压力中心偏向受力大的脚，在中间偏后 2 cm（Python 复核）。

### 5.3 手算：加速度对 ZMP 的要求

$z_c=0.8$ m，$g=9.81$。若要质心以 $\ddot x_c=1$ m/s² 加速，由 $p=x_c-\frac{z_c}{g}\ddot x_c$，ZMP 必须在质心投影后方 $0.8/9.81=0.0815$ m 处。若脚掌的支撑范围是质心投影前后各 $0.1$ m，则能产生的最大加速度为

$$\ddot x_{c,\max}=\frac{g}{z_c}\times0.1=12.26\times0.1=1.226\ \text{m/s}^2.$$

超过这个加速度，ZMP 越出脚掌边缘，机器人会绕脚尖翻转。这解释了一个经验：**脚掌越长、质心越低，能承受的加速度越大**；以及为什么人形机器人在行走时质心高度变化不大，上身也不能随意大幅摆动。

### 5.4 单点接触的退化情形

若支撑只是一个点（尖脚、球形底盘的单点接触、轮地的接触线在纵向上很窄），支撑多边形退化，ZMP 被迫等于接触点，约束 $p\in\text{支撑区域}$ 变成等式 $p=p_{\text{contact}}$。这意味着 LIPM 的 $\ddot x_c=\omega^2(x_c-p)$ 中 $p$ 不能自由选择，必须靠**移动接触点**（迈步或让轮子加速）来控制。这是轮式平衡系统与有脚掌的人形机器人的根本差别，第 12 节再讨论。

## 6. LIPM 的解与手算

### 6.1 解析解

固定 $p=0$（取 ZMP 为原点），方程 $\ddot x=\omega^2x$ 是二阶线性常微分方程，特征根 $\pm\omega$。设初值 $x(0)=x_0$，$\dot x(0)=v_0$：

$$x(t)=x_0\cosh\omega t+\frac{v_0}{\omega}\sinh\omega t,\qquad \dot x(t)=x_0\omega\sinh\omega t+v_0\cosh\omega t.$$

等价地，写成稳定/不稳定模态之和：

$$x(t)=\underbrace{\tfrac12\big(x_0-\tfrac{v_0}{\omega}\big)e^{-\omega t}}_{\text{收敛}}+\underbrace{\tfrac12\big(x_0+\tfrac{v_0}{\omega}\big)e^{\omega t}}_{\text{发散}}.$$

发散模态的系数 $\frac12(x_0+v_0/\omega)=\frac12\xi_0$ 正好是下一节的捕获点。

### 6.2 手算

$z_c=0.8$ m，$\omega=\sqrt{9.81/0.8}=3.502$ rad/s，$1/\omega=0.2856$ s。初值 $x_0=0.05$ m，$v_0=0.10$ m/s（示例，非项目实测）。Python 复核：

| $t$/s | $x$/m | $\dot x$/(m/s) |
|---|---|---|
| 0 | 0.0500 | 0.1000 |
| 0.2 | 0.0844 | 0.2584 |
| 0.5 | 0.2281 | 0.7857 |
| 1.0 | 1.3034 | 4.5619 |

初始只偏了 5 cm、速度 10 cm/s，一秒后质心已经偏离 1.3 m。时间常数 0.2856 s 意味着每过约 $0.69/\omega\approx0.198$ s，发散部分翻倍。这说明：**不稳定极点 $\omega$ 决定了控制器必须多快响应**。稳定模态系数 $0.0107$、不稳定模态系数 $0.0393$，不稳定部分更大，正是因为初始速度朝外。

### 6.3 质心高度的两面性

$\omega=\sqrt{g/z_c}$：质心越高，$\omega$ 越小，倒得越慢，控制器反应时间越宽裕；质心越低，$\omega$ 越大，倒得越快。这是"高个子更容易平衡"的物理解释，也是为什么调试平衡机器人时，**不得不关心质心高度**：向上加一个配重、让腿伸长，都会改变极点，需要的响应速度也随之改变。

## 7. 捕获点（capture point）

### 7.1 定义与推导

一维 LIPM 里，令

$$\xi=x+\frac{\dot x}{\omega}.$$

对时间求导，并使用 $\ddot x=\omega^2(x-p)$：

$$\dot\xi=\dot x+\frac{\ddot x}{\omega}=\dot x+\omega(x-p)=\omega\Big(x+\frac{\dot x}{\omega}-p\Big)=\omega(\xi-p).$$

于是复杂的二阶系统被分解成：一个不稳定的一阶系统 $\dot\xi=\omega(\xi-p)$（捕获点），加上一个稳定的一阶系统（$x$ 跟随 $\xi$）。**要让机器人停下来，只需让 $\xi$ 停下来，即把 ZMP 放到捕获点上：$p=\xi$ 时 $\dot\xi=0$。**

### 7.2 物理含义

"如果我现在把脚踩在 $\xi$ 这个点上，质心会刚好在这个点上方停住"。因此：

- 若 $\xi$ 位于当前支撑区域内：可以靠脚踝/重心调整站住，无需迈步；
- 若 $\xi$ 超出支撑区域：无论怎么调整 ZMP 都停不住，必须迈步，落脚点应取在 $\xi$ 附近。

### 7.3 手算

承上例 $x_0=0.05$、$v_0=0.10$：$\xi_0=0.05+0.10/3.502=0.0786$ m。若脚掌范围是 $[-0.1,0.1]$ m，$\xi_0$ 在范围内，原地可以停住（把 ZMP 放到 $0.0786$ m 处）。若是推了一把，$v_0=0.5$ m/s：$\xi_0=0.05+0.1428=0.193$ m，已经超出 $0.1$ m，必须迈步，落脚点至少要到 $0.193$ m 附近。此外 $\xi(t)=\xi_0e^{\omega t}$（当 $p=0$）：$t=0.5$ s 时 $\xi=0.0786\times e^{1.751}=0.4525$ m，与 $x+\dot x/\omega=0.2281+0.7857/3.502=0.4525$ 一致（Python 复核）。

## 8. 变高度质心与步行控制概览

### 8.1 变高度的意义

LIPM 要求质心高度恒定，但真实行走时质心会起伏，轮腿机器人伸缩腿更是主动改变 $z_c$。变高度模型 $\ddot x=\frac{g+\ddot z}{z}(x-p)$ 表明：向下加速质心（$\ddot z<0$，如迅速屈膝）等效于**重力变小**，$\omega^2$ 减小，倒得更慢；向上加速则等效于重力变大。手算：$z=0.8$ m，$\ddot z=-2$ m/s²：$\omega^2=(9.81-2)/0.8=9.7625$（对比恒高度 $12.2625$），$\omega=3.12$ rad/s。这就是"落地缓冲时容易稳"、"起跳瞬间难稳"的直观。

### 8.2 步行控制的几种思路（概览）

下列方法的细节本章不展开，列出是为了让你在面试里知道各自的位置：

1. **ZMP 预观控制（preview control）**：给定未来的 ZMP 参考轨迹，用 LQ 最优预瞄控制质心轨迹；
2. **捕获点/DCM 控制**：直接控制 $\xi$，用落脚点调整来稳定，计算量小、反应快；
3. **模型预测控制**：在 LIPM 上滚动优化未来若干步的质心与落脚点，带约束；
4. **全身控制/力矩控制**：把质心动量层的期望分配到关节力矩，结合摩擦锥与力矩限制做 QP（见 [机器人动力学进阶](./05_机器人动力学进阶_操作空间_接触_阻抗.md)）。

以上流派和具体论文的细节、作者与年份，凡写不确定者均未核对，面试时用概念层次回答即可。

## 9. 轮摆模型：轮足/轮腿的简化

### 9.1 动机

轮腿机器人（P7 类型）和球形机器人（P1 类型）都是"在轮子上倒立"。把整机等效为一个质点 $m$，通过无质量的杆连接到轮心，杆长 $L$，杆相对竖直方向的倾角为 $\theta$。轮子是整个系统的"支点"，它可以通过力矩让轮心产生水平加速度 $a$，从而"把脚移动到质心下面"。

```text
        ● m
       /
      /  L
     /  θ
    O ←→ a      (轮心，可以水平加速)
 ======================= ground
```

### 9.2 推导

质点相对轮心的位置 $r=L(\sin\theta,\cos\theta)$（$x$ 向前，$z$ 向上）。在轮心加速的非惯性系中，质点受到重力 $mg$ 和惯性力 $-ma$（水平）。绕轮心取矩（杆无质量、轮心铰接）：

$$mL^2\ddot\theta=mgL\sin\theta-maL\cos\theta\ \Longrightarrow\ \ddot\theta=\frac{g\sin\theta-a\cos\theta}{L}.$$

小角度线性化（$\sin\theta\approx\theta$，$\cos\theta\approx1$）：

$$\boxed{\ddot\theta=\frac gL\theta-\frac aL.}$$

这是一个典型的倒立摆：输入是轮心加速度 $a$，系数 $-1/L$ 表示轮向前加速会让摆向后倒（$\theta$ 减小）。

### 9.3 极点

开环特征方程 $s^2-g/L=0$，极点 $s=\pm\sqrt{g/L}$。数值（Python 复核）：

| 腿长 $L$/m | 0.1 | 0.2 | 0.3 | 0.4 |
|---|---|---|---|---|
| $\sqrt{g/L}$/(rad/s) | 9.90 | 7.00 | 5.72 | 4.95 |
| 时间常数 $1/\sqrt{g/L}$/ms | 101 | 143 | 175 | 202 |

腿越短，极点越大，倒得越快。这与 LIPM 的结论一致（$L$ 对应 $z_c$）。

### 9.4 与 P7 完整模型的对照

[P7 动力学建模与状态方程](../../01_项目课程/P7_可变腿长轮腿机器人/04_动力学建模与状态方程.md) 给出了含轮子惯量、机身俯仰的六状态模型，教学示例中的大不稳定极点为 $26.71,\ 18.91,\ 15.46$ rad/s（腿长 $0.1,\ 0.2,\ 0.3$ m）。把它们与简化模型的 $\sqrt{g/L}$ 相比：比值分别是 $26.71/9.90=2.70$、$18.91/7.00=2.70$、$15.46/5.72=2.70$（Python 复核），随腿长保持近似常数。说明两者遵循同一个 $1/\sqrt L$ 标度，绝对值差一个常数因子，来自简化模型忽略的轮子转动惯量、杆质量分布和耦合（该倍数是教学示例参数下的数值现象，不是通用常数）。

## 10. 变腿长对平衡的影响

### 10.1 带伸缩的运动方程

如果腿长随时间变化，$L=L(t)$，质点在极坐标下的角动量为 $mL^2\dot\theta$。角动量定理：

$$\frac{d}{dt}\big(mL^2\dot\theta\big)=mgL\sin\theta-maL\cos\theta.$$

展开左边：$mL^2\ddot\theta+2mL\dot L\dot\theta$。整理，线性化：

$$\boxed{\ddot\theta=\frac gL\theta-\frac{2\dot L}{L}\dot\theta-\frac aL.}$$

多出的 $-\dfrac{2\dot L}{L}\dot\theta$ 是科氏（Coriolis）项，相当于"阻尼"：腿伸长（$\dot L>0$）时它是正阻尼，腿缩短（$\dot L<0$）时它是负阻尼。

### 10.2 手算：伸缩对极点的影响

开环特征方程 $s^2+\frac{2\dot L}{L}s-\frac gL=0$。取 $L=0.2$ m：

| $\dot L$/(m/s) | 阻尼系数 $2\dot L/L$ | 极点（rad/s） |
|---|---|---|
| $+0.2$（伸腿） | 2.0 | $+6.07,\ -8.07$ |
| 0 | 0 | $+7.00,\ -7.00$ |
| $-0.2$（缩腿） | $-2.0$ | $+8.07,\ -6.07$ |

（Python 复核。）缩腿时不稳定极点从 7.00 增大到 8.07，伸腿时减小到 6.07。物理上就是花样滑冰运动员收拢手臂转得更快：角动量守恒，半径变小角速度变大。

### 10.3 对控制的启示

1. **腿长是一个慢变化的"参数"，也是一个有动态影响的"输入"**。如果腿长变化很慢（$\dot L\approx0$），可以冻结腿长，对每个腿长设计一套增益，这就是增益调度；
2. 如果腿长变化很快（例如跳跃、越障），$\dot L$ 项和 $\dot L$ 引起的质心高度变化不能忽略；
3. 缩腿的时刻本身就在增加不稳定程度，所以高速缩腿通常与平衡控制的"暂缓"或"提高增益"配合，实际系统会限制 $\dot L$ 的上限（见 P7 的腿长 PID 与力前馈）。

> **注意**：上面的 $\dot L$ 项是在"质点 + 无质量腿"假设下的结果。真实机器人腿有质量，伸缩时还有腿本身的动能变化，具体系数不同，但"缩腿等效负阻尼"的方向性结论成立（教学简化，不代表项目实测）。

## 11. 手算：轮摆的 PD 恢复与约束

### 11.1 PD 反馈设计

对线性化轮摆取控制律 $a=k_\theta\theta+k_d\dot\theta$，代入 $\ddot\theta=\frac gL\theta-\frac aL$：

$$\ddot\theta=\frac{g-k_\theta}{L}\theta-\frac{k_d}{L}\dot\theta.$$

特征方程 $s^2+\dfrac{k_d}{L}s+\dfrac{k_\theta-g}{L}=0$。**必须 $k_\theta>g$**：否则静态项仍是正反馈，摆照样倒。

以 $L=0.2$ m，期望闭环极点 $-5,-6$（$s^2+11s+30$，示例）：

$$\frac{k_d}{L}=11\Rightarrow k_d=2.2\ \text{m/s},\qquad \frac{k_\theta-g}{L}=30\Rightarrow k_\theta=9.81+6=15.81\ \text{m/s}^2/\text{rad}.$$

### 11.2 恢复需要多大加速度

若初始倾角 $\theta_0=0.1$ rad（5.7°），初速度为零，起始需要的轮心加速度 $a_0=k_\theta\theta_0=1.581$ m/s²。Python 用**非线性模型** $\ddot\theta=(g\sin\theta-a\cos\theta)/L$ 仿真 2 s（步长 $10^{-4}$ s，欧拉法）：$\theta(2\text{ s})=2.5\times10^{-5}$ rad，峰值加速度确为 $1.581$ m/s²，倾角被拉回零。线性化设计在这个小角度下与非线性仿真吻合。

### 11.3 约束：摩擦与力矩

轮心加速度 $a$ 要靠轮地摩擦力提供，所以 $|a|\le\mu g_{\text{eff}}$。取 $\mu=0.6$（示例）：$a_{\max}\approx5.886$ m/s²，则线性设计下能恢复的最大初始倾角约 $a_{\max}/k_\theta=0.372$ rad（21°）。此外电机转矩限制：设轮半径 $R=0.07$ m、等效质量 $7$ kg、峰值转矩 $3$ N·m（示例），则驱动力约束给出 $a\le\tau_{\max}/(mR)=3/(7\times0.07)=6.12$ m/s²。**两个上限里更小的那个决定实际可恢复范围。** 这与 P1/P7 调试中常见的"大扰动下输出饱和、控制器看似失效"完全对应：不是增益不对，而是需求超过了执行器与地面能给的上限（见 [前馈、增益调度、限幅与饱和](../05_控制理论/07_前馈_增益调度_限幅与饱和.md)）。

### 11.4 仅有角度反馈的缺陷

上面的 PD 只镇住了角度，没有约束轮子的位置与速度：稳态时 $\theta\to0$，但轮子可能以某个恒定速度漂走，位置无界。完整的平衡控制必须同时反馈位置与速度，这就是 LQR（状态 $[\theta,\dot\theta,x,\dot x]$ 或更多）与级联 PID（位置外环生成有界的姿态参考）的意义，详见 [LQR 最优控制](../05_控制理论/04_LQR最优控制.md) 与 P1、P7 的控制章节。

## 12. 轮足与足式的约束对比

| 维度 | 足式（人形、四足） | 轮足/轮腿（P7 类） | 球形底盘（P1 类） |
|---|---|---|---|
| 支撑区域 | 脚掌多边形，面积可观 | 轮地接触（线或点），面积极小 | 三个轮的接触点，机身整体倒立 |
| ZMP 的自由度 | 脚掌内可调，是主要控制手段 | 几乎无自由度，被锁在接触点 | 同轮式 |
| 平衡手段 | 脚踝力矩、上身角动量、迈步 | 轮子加速（把"脚"移到质心下方） | 轮子加速（三向全向） |
| 主要约束 | ZMP 在脚掌内、摩擦锥、关节限位 | 摩擦锥、轮电机转矩与速度、离地 | 摩擦锥、电机转矩、打滑 |
| 不稳定极点 | $\sqrt{g/z_c}$，$z_c\sim0.5$–$1$ m | $\sqrt{g/L}$ 量级，$L$ 更短，极点更大 | $\sqrt{g/l_{\text{com}}}$ 量级 |
| 模型 | LIPM、LIP + 角动量 | 轮摆、六状态模型 | 倒立摆 + 全向轮运动学 |

读法：**轮式系统不再依赖"ZMP 在脚掌内"，而是依赖"轮子够快够有力把支点追到质心下面"。** 所以轮腿机器人的极限不是翻脚，而是执行器带宽与扭矩、摩擦、离地（轮子短暂离开地面就失去全部控制能力，见 P7 的离地检测与接触切换章节）。

## 13. 在本项目里怎么用

> 说明：本章的 LIPM/ZMP/捕获点属于足式机器人理论；CV 中没有写足式行走项目，面试时不要说"我做过 ZMP 行走"。它们作为平衡问题的通用语言，用于解释 P1、P7 的物理。

- **P7 可变腿长轮腿**：腿长变化改变 $A,B$ 矩阵，不稳定极点按 $1/\sqrt L$ 移动，是 LQR 增益调度的物理原因（见 [P7 动力学建模与状态方程](../../01_项目课程/P7_可变腿长轮腿机器人/04_动力学建模与状态方程.md) 与 [LQR 原理与增益调度](../../01_项目课程/P7_可变腿长轮腿机器人/05_LQR原理与增益调度.md)）；缩腿的"负阻尼"效应解释了为什么腿长 PID 与力前馈必须与姿态环协调（[腿长 PID、力前馈与姿态协调](../../01_项目课程/P7_可变腿长轮腿机器人/07_腿长PID_力前馈与姿态协调.md)）；ZMP 约束的退化解释了为什么该项目关注的是摩擦、转矩限幅与离地检测（[离地检测与接触切换](../../01_项目课程/P7_可变腿长轮腿机器人/09_离地检测与接触切换.md)）。
- **P1 球形机器人（ballbot）**：同类轮式倒立摆，上身带双臂，手臂运动会移动质心，这就是 CoM 前馈补偿要解决的问题——质心偏移 $\Delta c$ 等价于 LIPM 里 ZMP 的偏移，必须用姿态参考的偏置补回去（见 [质心前馈补偿](../../01_项目课程/P1_AIRS球形机器人平衡控制/07_质心前馈补偿.md)）。
- **P3 RoboMaster 英雄**：底盘不是倒立摆，但炮管发射时的后坐与质心高度仍会影响翻车风险，可用"质心高度与加速度决定翻倒阈值"的思路作定性判断。
- **P2 双臂底盘**：差速底盘上的双臂伸出时，质心偏移会改变驱动轮法向力分配，加速度大时可能前轮抬起；COP 的计算思路与本章 §5.2 相同。

## 14. 常见错误与故障现象

| 现象 | 可能原因 | 处理 |
|---|---|---|
| 把质心高度当常数，腿长变化后增益不对 | 忽略 $\omega=\sqrt{g/z_c}$ 与 $L$ 的关系 | 按腿长调度；检查 $A(L),B(L)$ |
| ZMP 公式里正负号反 | 前向、加速度方向约定混乱 | 画图验证：质心前冲时 ZMP 应在质心后方 |
| 用 ZMP 约束判断轮式机器人 | 轮地接触面极小 | 改用摩擦锥与转矩限制判断 |
| 恢复时输出饱和，控制器"失灵" | 需求加速度超过摩擦或转矩上限 | 限制期望倾角、加大安全裕度、降低速度 |
| 缩腿瞬间倾角增长变快 | 缩腿相当于负阻尼（角动量守恒） | 限制 $\dot L$，缩腿时提高姿态增益或预加速 |
| 只用角度 PD，机器人缓慢漂走 | 没有位置/速度反馈 | 加位置外环或使用 LQR |
| 质心估计错误导致站不直 | 手臂/载荷质心未补偿 | 用 URDF 计算质心前馈并验证开关效果 |
| 把 LIPM 的不稳定极点当成"数据噪声" | 不理解开环不稳定系统本来就在发散 | 先看开环极点再调增益 |

## 15. 面试怎么问、怎么答

**Q1. What is the ZMP and why do we care about it?**

> The zero moment point is the point on the ground where the horizontal components of the total moment of the contact forces are zero. For a robot with feet, if the ZMP stays inside the support polygon the feet stay flat and the robot does not tip over. It is useful because it turns the dynamic balance problem into a simple constraint: the planned centre-of-mass motion must keep the ZMP inside the support region.

中文：ZMP 是地面上使接触力合力矩水平分量为零的点；有脚的机器人只要 ZMP 在支撑多边形内脚就不会翻；它把动态平衡变成简单约束。追问：ZMP 和 COP 的区别？（支撑区域内相同，COP 物理上不能出支撑区，ZMP 公式算出的值可以。）

**Q2. Derive the linear inverted pendulum model.**

> Assume the centre of mass stays at a constant height $z_c$ and the angular momentum is negligible. Taking moments about the ZMP gives $mg(x-p)=m\ddot x\,z_c$, so $\ddot x=\frac{g}{z_c}(x-p)$. The poles are $\pm\sqrt{g/z_c}$, one unstable. The only way to stabilise it is to move the ZMP or the foothold relative to the centre of mass.

中文：假设质心高度恒定、忽略角动量，对 ZMP 取矩得 $\ddot x=\frac g{z_c}(x-p)$；极点 $\pm\sqrt{g/z_c}$，一个不稳定；稳定办法只能移动 ZMP 或落脚点。追问：质心更高会怎样？（极点更小，倒得更慢，更容易控制。）

**Q3. What is the capture point?**

> It is $\xi=x+\dot x/\omega$. Its dynamics are $\dot\xi=\omega(\xi-p)$, so it is a first-order unstable system whose input is the ZMP. If I place the ZMP at the capture point, the centre of mass comes to rest above it. If the capture point is outside the support region, the robot cannot stop without taking a step.

中文：$\xi=x+\dot x/\omega$，$\dot\xi=\omega(\xi-p)$，是以 ZMP 为输入的一阶不稳定系统；把 ZMP 放在 $\xi$ 处质心会在其上方停下；$\xi$ 超出支撑区则必须迈步。

**Q4. Why does leg length matter for a wheel-legged robot's balance?**

> The leg length sets the height of the centre of mass above the wheel, and therefore the unstable pole of the inverted pendulum, roughly $\sqrt{g/L}$. Shorter legs fall faster and need a faster response, and the input gain changes too, so a single fixed LQR gain does not suit all leg lengths. That is why I scheduled the gains by leg length. Also, retracting the leg acts like negative damping by conservation of angular momentum, so fast retraction makes balance harder.

中文：腿长决定质心高度，进而决定不稳定极点约 $\sqrt{g/L}$；短腿倒得更快，输入增益也变，所以固定增益不通用，要按腿长调度；缩腿由角动量守恒相当于负阻尼。追问：怎么知道增益该调多少？（离线对不同腿长解 LQR 后拟合 $K(L)$，不是在线重解。这里强调这是我们项目的做法，数值按项目文档。）

**Q5. A wheeled robot saturates its motors when it is pushed. What limits recovery?**

> Two things: the friction between the wheels and the ground limits the horizontal acceleration to about $\mu g$, and the motor torque limits the drive force. The smaller of the two bounds the largest tilt angle the robot can recover from. If the push needs more than that, no controller gain can help, so I add output limiting and fall-recovery logic rather than just raising the gains.

中文：轮地摩擦限制加速度约 $\mu g$，电机转矩限制驱动力，两者取小决定可恢复的最大倾角；超出后任何增益都无济于事，应加输出限幅与摔倒恢复逻辑。

## 16. 本章要点速记

1. 质心加速度只由接触力之和与重力决定：$m\ddot c=\sum f_i+mg$；内力不改变总动量。
2. LIPM：$\ddot x=\omega^2(x-p)$，$\omega=\sqrt{g/z_c}$，一个不稳定极点；质心越高越慢越好控。
3. ZMP $p=x_c-\frac{z_c}{g}\ddot x_c$；有脚机器人要求 $p$ 在支撑多边形内；多接触 COP 按法向力加权。
4. 解：$x=x_0\cosh\omega t+\frac{v_0}{\omega}\sinh\omega t$；发散模态系数是 $\xi_0/2$。
5. 捕获点 $\xi=x+\dot x/\omega$，$\dot\xi=\omega(\xi-p)$；落脚点放在 $\xi$ 即可停住。
6. 变高度：$\omega^2=(g+\ddot z)/z$，向下加速质心等效重力变小。
7. 轮摆：$\ddot\theta=\frac gL\theta-\frac aL$，极点 $\pm\sqrt{g/L}$；PD 必须 $k_\theta>g$。
8. 变腿长多出 $-2\frac{\dot L}{L}\dot\theta$：缩腿相当于负阻尼，伸腿相当于正阻尼。
9. 轮式系统的限制是摩擦（$a\le\mu g$）和转矩，不是脚掌 ZMP。
10. 只镇角度不够，必须同时约束位置与速度（LQR 或级联位置外环）。

## 17. 自测题

1. 质心高度 $0.5$ m 的 LIPM，$\omega$ 与时间常数？（$\sqrt{9.81/0.5}=4.43$ rad/s；$0.226$ s。）
2. $z_c=0.6$ m 机器人要以 $2$ m/s² 加速，ZMP 相对质心投影在何处？（$p=x_c-0.6\times2/9.81=x_c-0.122$ m，质心后方 12.2 cm。）
3. 为什么说 $\xi$ 只依赖不稳定模态？（$\xi=x+\dot x/\omega$ 恰好是 $2\times$ 发散模态系数，$\dot\xi=\omega(\xi-p)$ 的特征根只有 $+\omega$。）
4. 轮摆 $L=0.3$ m，开环不稳定极点是多少？（$\sqrt{9.81/0.3}=5.72$ rad/s。）
5. 若 $L=0.2$ m、缩腿速度 $0.2$ m/s，开环不稳定极点变为多少？（$s^2-2s-49.05=0$，$s=8.07$ rad/s。）
6. 摩擦系数 $0.6$ 时轮式平衡机器人的最大水平加速度约多少？（$0.6\times9.81=5.89$ m/s²。）

## 18. 相关链接

- 上一章：[机器人动力学进阶：操作空间、接触、阻抗](./05_机器人动力学进阶_操作空间_接触_阻抗.md)；下一章：[柔性关节、弹性驱动、摩擦与接触建模](./07_柔性关节_弹性驱动_摩擦_接触建模.md)
- 相关基础：[倒立摆建模与线性化](./03_倒立摆建模与线性化.md)、[LQR 最优控制](../05_控制理论/04_LQR最优控制.md)、[前馈、增益调度、限幅与饱和](../05_控制理论/07_前馈_增益调度_限幅与饱和.md)
- 项目：[P7](../../01_项目课程/P7_可变腿长轮腿机器人/README.md)、[P1](../../01_项目课程/P1_AIRS球形机器人平衡控制/README.md)
- 模块说明：[02_力学与动力学建模 README](./README.md)
