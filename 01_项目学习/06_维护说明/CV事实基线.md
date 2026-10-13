# CV 事实基线（唯一权威来源）

> 申请人：Yongyi Lai（赖咏怡）。以下为英文通用 CV（`00_当前申请/03_通用证明材料/00_申请通用材料_跨学校复用/05_简历/英文通用简历/YongyiLaiCV.pdf`）的完整文字，已清理 PDF 抽取乱码。
> 规则：**所有文档以本文件为准**。源码、旧文档、学习笔记、中文简历与本文件冲突时，以本文件为准。面试全程英文。

## 抬头
Yongyi Lai — Robotics Algorithms · Motion Control · Autonomous Navigation
Shenzhen, China

## Education
Shenzhen Technology University, BEng in Computer Science and Technology, Sep 2023 – Jun 2027 (expected)
- Average score 87.63/100; ranked 6 of 156 (top 4%) | English: IELTS Academic 6.5
- Relevant courses: Principles of Automatic Control, Modern Control Theory, Linear Algebra, Probability and Statistics, Data Structures and Algorithms, Operating Systems
（注：中文简历写 7/156、前 5%，与英文 CV 不一致；PS、推荐信均写 6 of 156。统一用 6/156、top 4%。均分用 87.63。）

## Awards
- RoboMaster 2025 University Championship, National Final — Third Prize (national level); South Division — Second Prize
- RoboMaster 2025 University League, 3V3 Match — Second Prize
- 9th Huawei ICT Competition, Programming (OpenHarmony) — Third Prize, China National Final; First Prize, Provincial Round
- Shenzhen Technology University "Star of Learning" and "Star of Innovation" — Second Prize (2024–2025)

## Internships（时间倒序）

### P6 — Shenzhen Kaihong Digital Industry Development Co., Ltd. — Robotics Platform Intern — Jun 2026 – Present
Overview: device abstraction and motion coordination on the M-Robots OpenHarmony multi-robot platform; independently developed and integrated device abstraction, state management, distributed coordination and the safe-execution chain.
- Unified motion interface: built a unified capability model for sensors, actuators and controllers using C++ on OpenHarmony and mapped ROS/ROS2 motion commands to low-level drivers, enabling motion-control code reuse across hardware.
- Robot state management: standardised timestamps, coordinate frames and validity checks across multi-source perception and actuator feedback, giving the motion-control and task layers a consistent, trusted robot state.
- Coordination and safe execution: implemented device discovery, capability registration, state synchronisation and task dispatch over M-DDS, and gated task plans through an action-proposal, policy-check, execution and state-feedback chain.

### P2 — Shenzhen Future Power Co., Ltd. — Robotics Software Intern — Mar 2026 – May 2026
Overview: dual-arm and chassis control, covering VR teleoperation, leader–follower teaching and base motion control, all validated on real hardware through the ROS2-CAN execution chain.
- VR dual-arm teleoperation: mapped VR controller motion to end-effector targets and solved constrained inverse kinematics with MuJoCo/Mink, achieving end-effector tracking while suppressing velocity amplification near singularities.
- Leader–follower teaching: built dual-arm dynamics from URDF/KDL, designed leader-arm gravity compensation and follower position–velocity tracking, enabling low-resistance hand-guided teaching with synchronous follower motion.
- Wheel-speed control: mapped body velocity commands to wheel targets via differential-drive kinematics, closed the PID loop using encoder feedback and integrated wheel motion into odometry for teleoperated and autonomous modes.

### P1 — Shenzhen Institute of AI and Robotics for Society (AIRS) — Robotics Software Intern — Sep 2025 – Mar 2026
Overview: dynamic balance control of a three-omni-wheel ballbot with a dual-arm upper body; responsible for balance control development and on-robot tuning; the chassis maintained dynamic balance while both arms moved along fixed trajectories.
- Cascaded balance control: designed and deployed a PID cascade using IMU and wheel-speed feedback, a position outer loop feeding bounded attitude references to the attitude inner loop and outputs allocated to three wheels via kinematics.
- LQR modelling and analysis: modelled the system as a linearised four-state inverted pendulum and used LQR to analyse the stability–control-effort trade-off of state weights, as a model reference for PID tuning and actuator-saturation diagnosis.
- Centre-of-mass feedforward: estimated CoM offset from arm joint states with URDF/Pinocchio, fed roll/pitch gravity feedforward into the balance controller and verified its effect by toggling compensation under fixed arm trajectories.
- Hardware safety loop: integrated ROS state processing with CAN motor execution and implemented state initialisation, output limiting and fall-recovery safeguards, keeping the chassis balanced under arm motion.

## Projects

### P7 — Variable-Leg-Length Wheel-Legged Robot — Lab project · LQR/VMC balance control — Nov 2025 – Mar 2026
Overview: laboratory wheel-legged platform; responsible for balance control algorithms and on-robot tuning; the robot maintained body balance and tracked velocity commands while leg length changed.
- Gain-scheduled LQR: built body-pitch and motion states from IMU, wheel-speed and joint feedback, scheduled LQR gains by leg length and converted balance and motion errors into wheel-hub and hip torques.
- VMC torque control: computed virtual leg length and posture from closed-chain kinematics, mapped support force and pitch torque to joints via the Jacobian, and coordinated leg extension with leg-length PID and force feedforward.
- On-robot tuning: integrated feedback and torque output on an STM32H7/FreeRTOS and FDCAN chain, traced balance anomalies to state inconsistency, torque limits and ground-contact transitions, and tuned parameters on the robot.

### P5 — Mobile Robot Autonomous Navigation — Lab project · Localisation, planning and tracking — Jun 2025 – Oct 2025
Overview: car-like mobile robot; responsible for the algorithm chain from multi-sensor localisation and obstacle-aware planning to MPC tracking, achieving autonomous navigation on the real vehicle.
- EKF/AMCL localisation: fused IMU, wheel encoders and 2D LiDAR for map-based localisation, handled drift and pose jumps from inconsistent timing, frames and motion direction, and output a continuous vehicle state.
- Hybrid-A* global planning: incorporated vehicle heading and kinematic constraints into the search and generated feasible global paths under vehicle-footprint, minimum-turning-radius and obstacle-cost constraints.
- TEB local trajectory optimisation: optimised time-parameterised local trajectories under global-path and local-costmap constraints, outputting continuous pose and curvature references to the controller as local obstacles change.
- MPC trajectory tracking: based on a car-like kinematic model, solved receding-horizon control under position, heading, speed and steering constraints, closing the loop from planned trajectory to vehicle motion on the real vehicle.

### P3 — RoboMaster Hero Robot — Competition project · Embedded closed-loop control — Sep 2024 – May 2025
Overview: embedded control of the university team's hero robot on STM32F407/FreeRTOS and CAN; responsible for gimbal, launcher and chassis control loops; the robot competed in the 2025 national final and won third prize.
- Gimbal cascaded PID: built pitch/yaw angle–rate cascaded PID using IMU/AHRS and motor feedback, converting remote-control and vision targets into continuous attitude commands for stable aiming and tracking.
- Launcher control loops: built friction-wheel speed loops and a feeder position–velocity dual loop, iterating from PID through variable-gain PID to sliding-mode control with muzzle-velocity feedback for stable continuous firing.
- Chassis power control: allocated body velocity to four Mecanum wheels, estimated power demand from real-time power and buffer-energy feedback and scaled wheel speeds, keeping the chassis manoeuvrable under power limits.

### P4 — OpenHarmony Smart Gardening System — Personal project · Edge vision and autonomous control — May 2024 – Aug 2024
Overview: edge-vision perception, agent decision-making and autonomous irrigation on OpenHarmony and RK3588; independently built the full loop from visual observation and decision to device-side execution.
- Edge vision inference: built a YOLOv5/RKNN INT8 inference chain on RK3588, converting camera frames into structured observations with confidence and timestamps.
- Agent decision chain: built an environment and vision-event decision chain with LangGraph, with a rule-based fast path for extreme states, issuing structured recommendations or constrained requests to the execution layer.
- Local irrigation control: implemented a soil-moisture-driven pump state machine on Hi3861 with independent start/stop thresholds and manual-priority override for a stable local closed loop.
- System integration: unified visual observations, environment state and user commands into a traceable system state, orchestrated decisions with a server-side agent and closed the loop through web interaction and device-side irrigation.

## Skills
- Control: cascaded PID, gain-scheduled LQR, VMC, feedforward and sliding-mode control, MPC
- Planning & navigation: Hybrid-A*, TEB, EKF/AMCL, Kalman/Mahony complementary filtering, odometry, trajectory tracking
- Modelling and kinematics: inverted-pendulum linearisation, constrained IK, closed-chain kinematics, KDL/Pinocchio, MuJoCo
- Perception and AI: YOLOv5/RKNN edge inference, LangGraph decision chains, multi-source time/frame alignment
- Software and embedded: C/C++, Python, ROS/ROS2, Linux, STM32, FreeRTOS, CAN/FDCAN, OpenHarmony, RK3588, Git

## 项目编号对照（本仓库学习文档统一使用）
| 编号 | CV 中的名称 | 类型 | 时间 |
|---|---|---|---|
| P1 | AIRS — 三全向轮球形机器人（ballbot）双臂平衡控制 | 实习 | 2025.09–2026.03 |
| P2 | 未来动力 — VR 遥操作 / 主从示教 / 差速底盘 | 实习 | 2026.03–2026.05 |
| P3 | RoboMaster 英雄机器人嵌入式闭环控制 | 竞赛项目 | 2024.09–2025.05 |
| P4 | OpenHarmony 智能园艺系统（边缘视觉+Agent+灌溉） | 个人项目 | 2024.05–2024.08 |
| P5 | 车式移动机器人自主导航（EKF/AMCL+Hybrid-A*+TEB+MPC） | 实验室项目 | 2025.06–2025.10 |
| P6 | 深开鸿 M-Robots 平台（设备抽象/状态管理/安全执行链） | 实习（在职） | 2026.06–至今 |
| P7 | 可变腿长轮腿机器人（增益调度 LQR + VMC） | 实验室项目 | 2025.11–2026.03 |

## 来自 PS / 推荐信 / 申请材料的补充事实（均与 CV 一致）
- 推荐人 1：Fu Xianghua（傅向华），深圳技术大学人工智能学院副院长；2023 年起任教 University Computer Science A，指导 P7（轮腿）与 P5（导航）两个实验室项目；亲见 P3、P4 演示。
- 推荐人 2：Chen Dansheng，AIRS 机器人开发工程师，直接指导 P1。
- 推荐信关键语句：P1 中先做级联 PID，随后"返回模型分析"，推导四状态倒立摆、线性化、用 LQR 分析 position/velocity/tilt/tilt-rate 权重对稳定性与控制量的权衡；该分析用作"解释已部署控制器增益、定位状态异常和执行器饱和"的参考。
- PS 主线（AI 方向版）：高层任务计划不能直接发给机器人，须经 action-proposal → policy-check → execution → state-feedback 链（P6）；"AI 决策如何变成物理动作"；承认自己的机器学习经验以实践为主，想补理论基础；目标是大湾区机器人算法工程师（感知、导航、基于学习的控制）。
- 申请项目（11 个）：PolyU MSc Intelligent Robotics Engineering；CUHK-Shenzhen MSc AI and Robotics；CUHK-Shenzhen MSc Computer Science；CUHK MSc Robotics；CityU HK MSc Computer Science (P53)；CityU (Dongguan) MSc Artificial Intelligence；CityU (Dongguan) MSc Computer Science；UM MSc Robotics and Autonomous Systems；UM MSc Artificial Intelligence；MUST MSc Artificial Intelligence；HKUST(GZ) MSc Embodied Intelligence。
- 各校 PS 不同：见 `00_当前申请/01_学校正式提交包/*/03_可编辑源稿/01_Personal_Statement_*.md`。
