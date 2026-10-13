# 分类题库 12：运动学、动力学、状态估计与 SLAM（94 题，旋转表示、DH/POE、雅可比、奇异、动力学、KF/EKF/UKF/粒子滤波、互补滤波、IMU、SLAM 基础、相机模型）

> **一句话定位**：机器人岗位的技术面试，几乎一定会问"姿态怎么表示""雅可比是什么""卡尔曼滤波怎么工作""SLAM 前端后端是什么"。本文件把这一族问题做成 94 道英文问答，覆盖旋转与位姿、正逆运动学与奇异、刚体与机械臂动力学、传感器与滤波（互补滤波、KF、EKF、UKF、粒子滤波）、SLAM 与视觉几何、相机模型与标定，每题都标明对应的项目，并给出"追问到你的项目怎么接"的一句话。

**本章学完你能……**

- 用 60–90 秒讲清旋转矩阵、欧拉角、四元数、DH 与 POE、雅可比与奇异性，并用英文读出公式；
- 讲清卡尔曼滤波的预测—更新、增益的直觉、EKF 与粒子滤波的取舍，并能接到 P5 的 EKF 与 AMCL；
- 讲清互补滤波与 IMU 误差模型，并接到 P1、P3、P7 的姿态估计；
- 讲清 SLAM 的前端、后端、回环，以及相机模型、标定、对极几何和 PnP 的基础；
- 对没做过的内容（例如视觉 SLAM、紧耦合 VIO）诚实说明经验边界，再用概念接回你做过的相邻工作。

**前置知识**：[CV 事实基线](../../06_维护说明/CV事实基线.md)；[港硕面试真题调研](../00_港硕面试真题调研.md)（机器人学题目见其第 2.6.2 节与第 3.2–3.3 节）；[数字、公式、单位的英文读法](../../07_英语学习/09_数字_公式_单位的英文读法.md)；[技术词汇表：估计、规划与导航](../../07_英语学习/03_技术词汇表_估计规划导航.md)。基础教材：[坐标系与齐次变换](../../02_基础知识/03_运动学与坐标变换/01_坐标系_旋转矩阵_齐次变换.md)、[欧拉角与四元数](../../02_基础知识/03_运动学与坐标变换/02_欧拉角_四元数_姿态表示.md)、[正逆运动学与雅可比](../../02_基础知识/03_运动学与坐标变换/03_正逆运动学与雅可比.md)、[奇异性与阻尼最小二乘](../../02_基础知识/03_运动学与坐标变换/04_奇异性_阻尼最小二乘_零空间.md)、[闭链五连杆](../../02_基础知识/03_运动学与坐标变换/05_闭链并联机构与五连杆运动学.md)、[机械臂动力学与 URDF](../../02_基础知识/03_运动学与坐标变换/06_机械臂动力学_重力补偿_URDF_KDL.md)、[牛顿—欧拉](../../02_基础知识/02_力学与动力学建模/01_牛顿欧拉与刚体力学.md)、[拉格朗日](../../02_基础知识/02_力学与动力学建模/02_拉格朗日方程与机器人动力学.md)、[传感器](../../02_基础知识/06_状态估计/01_传感器_IMU_编码器_激光雷达.md)、[互补滤波](../../02_基础知识/06_状态估计/02_互补滤波与Mahony_AHRS.md)、[卡尔曼与 EKF](../../02_基础知识/06_状态估计/03_卡尔曼滤波与EKF.md)、[里程计融合](../../02_基础知识/06_状态估计/04_里程计与传感器融合.md)、[粒子滤波与 AMCL](../../02_基础知识/06_状态估计/05_粒子滤波与AMCL.md)、[时间同步与数据有效性](../../02_基础知识/06_状态估计/06_时间同步_坐标系一致性与数据有效性.md)、[SLAM 问题](../../02_基础知识/07_SLAM与建图/01_SLAM问题_前端后端_位姿图优化.md)、[视觉里程计](../../02_基础知识/07_SLAM与建图/02_视觉里程计_特征_对极几何_PnP_BA.md)、[相机模型与标定](../../02_基础知识/09_感知与视觉/01_相机模型与标定.md)。

## 目录

1. 本文件怎么用
2. A 旋转、位姿与坐标系（Q1–Q12）
3. B 正逆运动学、雅可比与奇异性（Q13–Q28）
4. C 动力学与建模（Q29–Q43）
5. D 传感器与状态估计（Q44–Q66）
6. E SLAM、视觉几何与相机标定（Q67–Q94）
7. 自测与练习安排

---

## 1. 本文件怎么用

1. **每题的结构**：英文题目、中文题意、考察点、项目关联、英文回答（口语，第一人称）、中文翻译、两个追问、"追问到你的项目怎么接"、关键词与易错点。先遮住英文，只看中文题意，自己讲一遍，再对照。
2. **项目口径（必须记牢，与各项目 README 自查表一致）**：
   - **P1**：IMU 是 FDI 系列 AHRS 模块，板载姿态解算，输出约 100 Hz；启动时机体扶稳，采 200 帧（约 2 s）估计 roll、pitch 偏置；位置只靠轮速里程计积分，静止 60 s 漂移约 5 cm；URDF 来自 MuJoCo 的 ballbotgym 环境，每臂 4 个关节参与质心计算，根坐标在球心。
   - **P2**：Mink 约束 IK，全局阻尼 10 的负 3 次方，最小奇异值低于 0.05 标红；差速底盘轮半径 0.13 m、轮距 0.75 m。
   - **P3**：BMI088（SPI + DMA），陀螺 ±2000 °/s，上电静止采 2000 个样本求陀螺零偏，加热恒温 40 °C；AHRS 为 Mahony，Kp 为 0.5、Ki 为 0，磁力计未启用；yaw 静置漂移约 1 °/min。
   - **P5**：ROS2 Humble；robot_localization 的 EKF，30 Hz，二维模式，融合轮速、由 v tan(delta)/L 得到的偏航角速度、陀螺 omega z；nav2_amcl，粒子 500–3000，似然场模型；激光时间戳问题（到达时刻 vs 扫描起始，约 60 ms）；倒车方向符号问题（轮速无符号）。
   - **P6**：REP-103（x 前、y 左、z 上）；单调时钟；新鲜度上限取传感器周期的 3 倍；冻结检测（连续 5 帧完全相同）；NTP 跨设备精度约几毫秒到十几毫秒；这些是**设计取值，不是公司内部实测**。
   - **P7**：五连杆 L1 = L4 = 150 mm、L2 = L3 = 250 mm、L5 = 108 mm；姿态用 Mahony 型互补滤波（Kp 约 2、Ki 约 0.005），速度用一维卡尔曼（Q 约 0.001、R 约 0.01）。
3. **诚实规则**：你没做过视觉 SLAM、紧耦合 VIO、UKF，被问到时说 "I know the concept but I have not built it"，再用概念和你做过的 EKF/AMCL/互补滤波接回。读码发现的隐患说 "I noticed"。
4. **公式一律按英文口语读**；文中公式只供核对。

---

## 2. A 旋转、位姿与坐标系（Q1–Q12）

### Q1. Compare rotation matrices, Euler angles, axis–angle and quaternions.
**中文题意**：比较旋转矩阵、欧拉角、轴角和四元数。
**考察点**：姿态表示的取舍。
**项目关联**：P1、P2、P3（姿态与坐标变换）。
**English answer（口语）**：
> A rotation matrix has nine numbers with six constraints. It composes by simple multiplication and rotates vectors directly, but it is redundant and can drift from orthogonality. Euler angles use three numbers, which is intuitive and good for display, but they suffer from gimbal lock and the order of rotations matters. Axis–angle is also compact, a unit axis and an angle, and it is natural for angular velocity integration, but it is awkward to compose. A unit quaternion has four numbers with one constraint; it has no gimbal lock, is cheap to compose and normalise, and interpolates smoothly. In my work I use quaternions inside estimators, matrices for transforms, and Euler angles for display and for simple limits.

**中文翻译**：旋转矩阵有九个数和六个约束，用乘法就能复合、直接旋转向量，但冗余且可能偏离正交性。欧拉角用三个数，直观且适合显示，但存在万向锁，且旋转顺序很重要。轴角也很紧凑，是单位轴和角度，适合角速度积分，但不便复合。单位四元数有四个数和一个约束，没有万向锁，复合与归一化便宜，插值平滑。我的工作里，估计器内部用四元数，变换用矩阵，显示和简单限制用欧拉角。
**追问 1**：*Which is best for optimisation?* A minimal local parametrisation, such as a rotation vector around the current estimate, avoids both redundancy and singularities.
　中文：围绕当前估计的旋转向量这种最小局部参数化，同时避免冗余和奇异。
**追问 2**：*How many parameters does each have?* Matrix nine, Euler three, axis–angle three or four, quaternion four, and the true degrees of freedom are three.
　中文：矩阵 9、欧拉角 3、轴角 3 或 4、四元数 4，真正的自由度是 3。
**如果追问到你的项目**：*In P3 the board sent the attitude quaternion to the vision computer, and for the gimbal channels I used angles with care at the plus or minus 180 degree wrap.*
　中文：P3 里电控板把姿态四元数发给视觉计算机，云台通道用角度并小心处理 ±180° 回绕。
**关键词与易错点**：rotation matrix、Euler angles、quaternion、gimbal lock、redundancy。

### Q2. What is gimbal lock, and which Euler convention do you use?
**中文题意**：什么是万向锁？你用哪种欧拉角约定？
**考察点**：约定与奇异。
**项目关联**：P1、P3（roll/pitch/yaw）。
**English answer（口语）**：
> In a three-angle sequence, when the middle angle reaches a special value, two rotation axes line up and one degree of freedom is lost. For the common Z–Y–X sequence, yaw then pitch then roll, that happens when pitch is plus or minus ninety degrees: yaw and roll then rotate about the same axis, so only their sum matters, and the mapping between angular velocity and Euler rates becomes singular. Ground robots rarely reach it, since pitch stays small, so Euler angles are fine for balance control, where roll and pitch are small. For anything that can point straight up, like a drone or an arm end effector, I would avoid Euler angles in the computation and use quaternions or matrices.

**中文翻译**：在三角度序列中，当中间角达到特殊值时，两个旋转轴对齐，丢失一个自由度。对常见的 Z–Y–X 序列（先 yaw 再 pitch 再 roll），当 pitch 为 ±90 度时发生：yaw 和 roll 绕同一个轴旋转，只有它们的和有意义，角速度与欧拉角速率的映射变得奇异。地面机器人 pitch 通常很小，很少到达，所以对 roll、pitch 都很小的平衡控制，欧拉角没问题。对于能朝上指的物体（如无人机或机械臂末端），我会在计算中避免欧拉角，改用四元数或矩阵。
**追问 1**：*What does singular mapping mean?* Euler rates blow up as pitch approaches ninety degrees, even for a moderate angular velocity.
　中文：pitch 趋近 90 度时，即使角速度适中，欧拉角速率也会爆炸。
**追问 2**：*Is gimbal lock a physical or a mathematical problem?* Mathematical for the representation; in a real mechanical gimbal it is physical.
　中文：对表示来说是数学问题；真实的机械万向架里则是物理问题。
**如果追问到你的项目**：*In my balance work the roll and pitch stayed within a few degrees, and the yaw wrap at plus or minus 180 degrees was the real issue.*
　中文：我的平衡工作里 roll 和 pitch 保持在几度之内，真正的问题是 yaw 在 ±180° 处的回绕。
**关键词与易错点**：gimbal lock、Z–Y–X、singular mapping、representation。

### Q3. How do quaternions compose, and what is the difference between Hamilton and JPL conventions?
**中文题意**：四元数如何复合？Hamilton 与 JPL 约定有何不同？
**考察点**：约定一致性（常见 bug）。
**项目关联**：P3、P7（IMU 四元数）。
**English answer（口语）**：
> Quaternion composition is the Hamilton product. If q1 rotates first and q2 second, the combined rotation is q2 times q1, in the convention where the quaternion acts as q times v times q conjugate. To rotate a vector, I embed it as a pure quaternion, multiply by q on the left and q conjugate on the right. The Hamilton convention, used by ROS and Eigen, has the real part first or last depending on the library, and i j k satisfy i j equals k. The JPL convention, common in older aerospace and some visual–inertial code, uses a left-handed product and a different order of multiplication. Mixing conventions silently gives wrong rotations, so I always check the convention and the element order of every library and message.

**中文翻译**：四元数复合是 Hamilton 乘积。若 q1 先旋转、q2 后旋转，在四元数按 q 乘 v 乘 q 共轭作用的约定下，合成旋转是 q2 乘 q1。旋转向量时，把它嵌入为纯四元数，左乘 q、右乘 q 共轭。ROS 和 Eigen 用的 Hamilton 约定，实部在前还是在后取决于库，且 i j 等于 k。JPL 约定常见于较老的航天和部分视觉—惯性代码，使用左手系乘积和不同的相乘顺序。混用约定会静默地得到错误旋转，所以我总是检查每个库和消息的约定和元素顺序。
**追问 1**：*What is the element order in ROS messages?* x, y, z, w, with w last; some libraries store w first.
　中文：x, y, z, w，w 在最后；有些库把 w 放在第一位。
**追问 2**：*How do you test a convention quickly?* Rotate a known vector by ninety degrees about z and compare.
　中文：把已知向量绕 z 旋转 90 度并比较。
**如果追问到你的项目**：*A sign or order mismatch is one of the first things I check when an attitude looks mirrored, as I did when integrating IMU data on the robots.*
　中文：姿态看起来像镜像时，符号或顺序不匹配是我首先检查的事项之一，在集成机器人 IMU 数据时就是这样做的。
**关键词与易错点**：Hamilton product、JPL、element order、pure quaternion。

### Q4. How do homogeneous transforms chain, and how do you invert one?
**中文题意**：齐次变换如何串联？如何求逆？
**考察点**：基础变换。
**项目关联**：P2、P5、P6（TF）。
**English answer（口语）**：
> A homogeneous transform T from frame B to frame A, written T A B, maps coordinates in B to coordinates in A. Chaining is multiplication with matching inner frames: T A C equals T A B times T B C. The inner frame label B cancels. The inverse is not the transpose of T. If T has rotation R and translation p, the inverse has rotation R transpose and translation minus R transpose p. I write every transform with both frame names in the variable, such as T base tool, so a mismatch is visible. In ROS the TF tree does the same chain for me, but I still check the direction, because a parent–child convention can be confusing.

**中文翻译**：从坐标系 B 到 A 的齐次变换 T（记作 T A B）把 B 中的坐标映射到 A 中的坐标。串联是乘法，内侧坐标系匹配：T A C 等于 T A B 乘 T B C，内侧的 B 抵消。逆不是 T 的转置。若 T 的旋转为 R、平移为 p，逆的旋转是 R 转置，平移是负 R 转置 p。我在变量名里写出两个坐标系名称，如 T base tool，使不匹配一眼可见。ROS 的 TF 树替我完成同样的串联，但我仍检查方向，因为父子约定容易混淆。
**追问 1**：*How do you transform a point from tool to base?* Multiply T base tool by the point in homogeneous coordinates.
　中文：用 T base tool 乘齐次坐标下的点。
**追问 2**：*What is a common error?* Using T tool base instead of T base tool, which gives mirror-like or wrongly offset results.
　中文：用了 T tool base 而不是 T base tool，会得到镜像或偏移错误的结果。
**如果追问到你的项目**：*In P5 and P6 many bugs were frame or timestamp inconsistencies, so naming transforms with both frames became a habit.*
　中文：P5、P6 里许多 bug 是坐标系或时间戳不一致，所以用双坐标系命名变换成了习惯。
**关键词与易错点**：homogeneous transform、inner frame cancellation、inverse、naming。

### Q5. What are the standard ROS frames and conventions (REP-103, map, odom, base_link)?
**中文题意**：ROS 的标准坐标系和约定（REP-103、map、odom、base_link）是什么？
**考察点**：导航栈的坐标系分工。
**项目关联**：P5、P6（核心）。
**English answer（口语）**：
> REP-103 sets the axes: x forward, y left, z up, with a right-handed system and angles in radians. For a mobile robot, the frames are chained as map, then odom, then base_link, then sensor frames like laser_link and imu_link. The odom frame is continuous: the pose of the robot in odom never jumps, but it drifts over time; it comes from wheel odometry and the EKF. The map frame is globally consistent but can jump when localisation corrects. The map-to-odom transform absorbs the correction from localisation, such as AMCL. Control should be done in the odom frame so that corrections do not cause jumps in the commands.

**中文翻译**：REP-103 规定坐标轴：x 向前、y 向左、z 向上，右手系，角度用弧度。对移动机器人，坐标系依次是 map、odom、base_link，再到传感器坐标系，如 laser_link 和 imu_link。odom 坐标系是连续的：机器人在 odom 中的位姿不会跳变，但会随时间漂移，来自轮里程计和 EKF。map 坐标系全局一致，但定位修正时会跳变。map 到 odom 的变换吸收定位（如 AMCL）的修正。控制应在 odom 坐标系里进行，使修正不会造成指令跳变。
**追问 1**：*Who publishes which transform?* The EKF publishes odom to base_link; AMCL publishes map to odom.
　中文：EKF 发布 odom 到 base_link；AMCL 发布 map 到 odom。
**追问 2**：*What is base_footprint versus base_link?* base_footprint is the projection on the ground; base_link is the robot body centre or a chosen reference point.
　中文：base_footprint 是地面投影；base_link 是机体中心或选定的参考点。
**如果追问到你的项目**：*In P5 the base_link was at the rear-axle centre, I did control in the odom frame, and I smoothed the map-to-odom correction with a rate limit and a threshold.*
　中文：P5 里 base_link 在后轴中心，我在 odom 坐标系里做控制，并用速率限制和门限平滑 map 到 odom 的修正。
**关键词与易错点**：REP-103、map、odom、base_link、continuous vs global。

### Q6. What is the difference between active and passive rotations?
**中文题意**：主动旋转与被动旋转有何区别？
**考察点**：概念清晰度。
**项目关联**：P2（VR 轴映射）。
**English answer（口语）**：
> An active rotation moves a vector within a fixed frame: the point rotates. A passive rotation keeps the point still and rotates the coordinate frame, so the coordinates change in the opposite way. The two use rotation matrices that are transposes of each other. Most robotics texts use R A B to mean the orientation of frame B expressed in frame A, which also converts coordinates from B to A. I always say which frame the vector is expressed in. A mistake with this gives a motion in the mirrored or opposite direction, which is typical when mapping VR controller motion to a robot.

**中文翻译**：主动旋转在固定坐标系里移动向量：点在转。被动旋转保持点不动而旋转坐标系，所以坐标以相反的方式变化。两者使用的旋转矩阵互为转置。多数机器人学教材用 R A B 表示坐标系 B 在 A 中表达的姿态，同时也把坐标从 B 变换到 A。我总是说明向量是在哪个坐标系中表达的。这里的错误会得到镜像或相反方向的运动，这在把 VR 手柄运动映射到机器人时很典型。
**追问 1**：*How do you find such a mistake?* Move the controller along one axis at a time and watch the robot, then fix the axis-mapping matrix, checking its determinant is plus one.
　中文：一次沿一个轴移动手柄并观察机器人，然后修正轴映射矩阵，并检查行列式为 +1。
**追问 2**：*What is the relation between R A B and R B A?* They are transposes (inverses) of each other.
　中文：互为转置（互逆）。
**如果追问到你的项目**：*In P2 the axis map from the VR frame to the robot base had determinant plus one: the VR minus z became base x, minus x became y, and y became z.*
　中文：P2 里从 VR 坐标系到机器人基座的轴映射行列式为 +1：VR 的 −z 变成基座 x，−x 变成 y，y 变成 z。
**关键词与易错点**：active rotation、passive rotation、frame expression、determinant。

### Q7. How is angular velocity related to Euler-angle rates?
**中文题意**：角速度与欧拉角速率的关系是什么？
**考察点**：陀螺数据使用。
**项目关联**：P1、P3、P7（陀螺角速率）。
**English answer（口语）**：
> A gyroscope measures angular velocity in the body frame, not the derivative of the Euler angles. They are related by a matrix that depends on the current angles. For the Z–Y–X convention, the roll rate equals the body rate p plus sine roll tangent pitch times q plus cosine roll tangent pitch times r, and similar expressions for the other two. For small roll and pitch, the matrix is close to the identity, so the gyro readings are approximately the Euler rates. That is why for a balancing robot with small tilt I can use the gyro rate directly as the derivative of the tilt angle. For large angles, I must use the full relation or work with quaternions.

**中文翻译**：陀螺仪测量的是机体系下的角速度，而不是欧拉角的导数。两者通过依赖当前角度的矩阵联系。对 Z–Y–X 约定，roll 角速率等于机体角速度 p 加 sin roll 乘 tan pitch 乘 q 再加 cos roll 乘 tan pitch 乘 r，另两个有类似表达式。roll、pitch 都很小时，该矩阵接近单位阵，所以陀螺读数近似等于欧拉角速率。因此对小倾角的平衡机器人，我可以直接把陀螺角速率当作倾角的导数。大角度时必须使用完整关系或使用四元数。
**追问 1**：*What happens near pitch of ninety degrees?* The tangent blows up, which is the gimbal-lock singularity.
　中文：tan 趋于无穷，即万向锁奇异。
**追问 2**：*How do you integrate gyro data properly?* Use the quaternion or rotation update with the rotation vector omega times dt.
　中文：用旋转向量 omega 乘 dt 做四元数或旋转更新。
**如果追问到你的项目**：*In P3 and P1 the gyro rate is used directly as the derivative term in the inner loops, which is valid for small tilt.*
　中文：P3 和 P1 里陀螺角速率直接作为内环的微分项，小倾角下成立。
**关键词与易错点**：body angular velocity、Euler rates、small angle、quaternion update。

### Q8. What is a lever arm, and why does the sensor position matter?
**中文题意**：什么是杆臂（lever arm）？为什么传感器位置重要？
**考察点**：传感器外参的影响。
**项目关联**：P5（IMU 装在车体中心偏后）。
**English answer（口语）**：
> The lever arm is the offset vector between the point I want the motion of, such as the robot centre, and the sensor location. When the body rotates, a sensor away from the centre sees extra velocity and acceleration: velocity equals the centre velocity plus omega cross r, and acceleration includes centripetal and tangential terms. For a gyroscope, the position does not matter, since angular velocity is the same everywhere on a rigid body. For an accelerometer, it does, so for a precise estimate I either mount the IMU near the centre of rotation or compensate with the known offset. Ignoring it adds a systematic error during turns or fast tilt.

**中文翻译**：杆臂是我想要其运动的点（如机器人中心）与传感器位置之间的偏移向量。机体转动时，偏离中心的传感器会看到额外的速度和加速度：速度等于中心速度加 omega 叉乘 r，加速度包含向心和切向项。对陀螺仪，位置无关，因为刚体上各处角速度相同；对加速度计则相关，所以要精确估计就要把 IMU 装在转动中心附近，或用已知偏移补偿。忽略它会在转弯或快速倾斜时引入系统误差。
**追问 1**：*Does the gyro need a lever-arm correction?* No.
　中文：不需要。
**追问 2**：*How do you measure the offset?* From CAD or a tape measure, and refine by calibration.
　中文：来自 CAD 或卷尺测量，再通过标定细化。
**如果追问到你的项目**：*In P5 the IMU sat slightly behind the vehicle centre, and the external pose of each sensor relative to base_link was kept in one place and applied once.*
　中文：P5 里 IMU 位于车体中心略靠后，每个传感器相对 base_link 的外参集中放在一处并只应用一次。
**关键词与易错点**：lever arm、centripetal acceleration、extrinsics、rigid body。

### Q9. How do you handle a left-handed versus right-handed frame mistake?
**中文题意**：遇到左手系/右手系错误怎么处理？
**考察点**：调试坐标系问题。
**项目关联**：P2、P5。
**English answer（口语）**：
> The symptoms are mirrored motion: moving the controller left moves the robot right, or the robot turns the wrong way. The quick test is the determinant of the axis-mapping matrix: plus one is a proper rotation, minus one hides a reflection. Then I check each axis with a single-axis motion and a visual cross-check, for example in RViz with the TF axes shown. I also check sign conventions of angles, such as whether positive yaw is counter-clockwise seen from above, since a few platforms use clockwise. I fix it in one place, the transform from the sensor frame to the robot frame, and never patch signs in several downstream places.

**中文翻译**：症状是镜像运动：手柄向左移动，机器人向右移动，或机器人转向相反。快速检验是轴映射矩阵的行列式：+1 是合法旋转，−1 隐含了反射。然后我用单轴运动和视觉对照（如在 RViz 里显示 TF 轴）逐轴检查。我还检查角度的符号约定，例如正 yaw 从上往下看是否为逆时针，有些平台用顺时针。我在唯一的地方修复，即传感器坐标系到机器人坐标系的变换，绝不在下游多处修补符号。
**追问 1**：*Where does a clockwise yaw convention appear?* In the P2 base-board firmware frame, which needed a conversion to ROS: x ROS equals y board, y ROS equals minus x board, yaw ROS equals minus yaw board.
　中文：P2 底盘板固件的坐标系，需要转到 ROS：x_ROS 等于 y_板，y_ROS 等于 −x_板，yaw_ROS 等于 −yaw_板。
**追问 2**：*Why fix it in one place?* Patching in several places makes later changes error-prone.
　中文：多处修补使以后的修改容易出错。
**如果追问到你的项目**：*In P5, I unified the steering sign so that positive means left, and I parsed the wheel speed as a signed value, which fixed the reverse-driving odometry.*
　中文：P5 里我统一转向符号为向左为正，并把轮速解析为带符号值，修复了倒车里程计。
**关键词与易错点**：handedness、determinant、single-axis test、centralised fix。

### Q10. What is extrinsic calibration between two sensors, and how can it go wrong?
**中文题意**：两个传感器之间的外参标定是什么？可能出什么错？
**考察点**：标定的系统影响。
**项目关联**：P5（激光—IMU—车体）、P6。
**English answer（口语）**：
> Extrinsic calibration finds the rigid transform between two sensor frames, for example LiDAR to IMU or camera to base. Any error in rotation or translation appears as a systematic error: points are placed at the wrong position, the scan does not match the map, or fused sensors disagree during turns. Methods range from CAD values and tape measure for a rough estimate, to optimisation that makes sensor observations agree, such as aligning motion estimated by each sensor. A time offset is a second extrinsic-like quantity: a 60 millisecond offset at a turning rate of one radian per second produces a 3.4 degree heading error. So I check both space and time alignment when fused data looks inconsistent.

**中文翻译**：外参标定求两个传感器坐标系之间的刚体变换，如激光雷达到 IMU 或相机到底盘。旋转或平移的任何误差都表现为系统误差：点被放在错误位置，扫描与地图不匹配，或融合的传感器在转弯时互相矛盾。方法从用 CAD 数值和卷尺粗估，到使不同传感器观测一致的优化，例如对齐各传感器估计的运动。时间偏移是第二个类似外参的量：1 rad/s 转速下 60 ms 偏移产生 3.4 度的航向误差。所以当融合数据看起来不一致时，我同时检查空间与时间对齐。
**追问 1**：*Check the number.* 0.06 seconds times 1 radian per second is 0.06 radian, about 3.4 degrees.
　中文：0.06 s 乘 1 rad/s 为 0.06 rad，约 3.4 度。
**追问 2**：*How do you estimate a time offset?* Cross-correlate angular rate from two sensors and find the lag at the peak.
　中文：对两个传感器的角速率做互相关，在峰值处找滞后。
**如果追问到你的项目**：*In P5 the laser timestamp was the arrival time rather than the scan start, about 60 milliseconds off from the IMU; after fixing it and compensating, the cross-correlation offset fell to under about 5 milliseconds.*
　中文：P5 里激光时间戳是到达时刻而不是扫描起始，与 IMU 相差约 60 ms；修复并补偿后互相关偏移降到约 5 ms 以内。
**关键词与易错点**：extrinsic calibration、time offset、cross-correlation、systematic error。

### Q11. Why does TF sometimes say "extrapolation into the future", and how did you fix it?
**中文题意**：TF 为什么会报"外推到未来（extrapolation into the future）"？你如何修复？
**考察点**：时间一致性。
**项目关联**：P5（核心）。
**English answer（口语）**：
> TF stores a buffer of transforms with timestamps and interpolates between them. If I ask for a transform at a time that is later than the latest available transform, it cannot interpolate, and it reports extrapolation into the future. In P5, the laser scan carried the time of arrival, so scan timestamps and transforms disagreed by tens of milliseconds, and at turns the wall appeared smeared. The fixes: use the scan start time and compensate a known fixed delay; synchronise clocks across machines with chrony; and set a transform tolerance, in my case 0.3 seconds, so that the lookup waits a little for the transform to arrive. The first fix is the root cause; the tolerance is only a safety margin.

**中文翻译**：TF 保存带时间戳的变换缓冲并在其间插值。若请求的时刻晚于最新可用变换，就无法插值，报告外推到未来。P5 里激光扫描携带的是到达时刻，扫描时间戳与变换相差几十毫秒，转弯时墙体出现拖影。修复办法：使用扫描起始时间并补偿已知的固定延迟；用 chrony 同步各机器时钟；设置变换容忍度（我的取值 0.3 秒），让查询稍等变换到达。第一个是根因，容忍度只是安全余量。
**追问 1**：*Why not just increase the tolerance?* It hides the inconsistency and adds latency; the data would still be misaligned.
　中文：它掩盖不一致并增加延迟；数据仍然未对齐。
**追问 2**：*What happens with the wall smear at turns?* The scan is matched to the wrong pose, so wall points spread along the rotation.
　中文：扫描被匹配到错误位姿，墙点沿旋转方向被拉开。
**如果追问到你的项目**：*That was one of my three key P5 faults; after the fix, the pose jumps shrank to usually below 5 centimetres between adjacent cycles.*
　中文：这是我 P5 三个关键故障之一；修复后相邻周期的位姿跳变一般小于 5 cm。
**关键词与易错点**：TF buffer、timestamp、extrapolation、transform tolerance、chrony。

### Q12. What are the coordinate frames and conventions for a car-like robot, and where do you put base_link?
**中文题意**：车式机器人的坐标系与约定是什么？base_link 放在哪里？
**考察点**：对运动学模型的影响。
**项目关联**：P5。
**English answer（口语）**：
> For a car-like robot I put base_link at the centre of the rear axle. That choice makes the kinematic model simple: the rear-axle point moves tangent to its path, without lateral slip, so the equations are x dot equals v cos psi, y dot equals v sin psi, psi dot equals v over L times tan delta. The footprint is then described relative to this point; for us it was a 0.60 by 0.38 metre rectangle with overhangs of about 0.10 metre. If I chose the geometric centre instead, the model would be more complex. Sensor frames such as the laser and IMU are defined relative to base_link, and positive steering is left.

**中文翻译**：对车式机器人，我把 base_link 放在后轴中心。这个选择使运动学模型简单：后轴点沿路径切线运动、无侧滑，方程为 x 导数等于 v cos psi，y 导数等于 v sin psi，psi 导数等于 v 除以 L 乘 tan delta。然后相对该点描述轮廓；我们的是 0.60 × 0.38 米的矩形，前后悬各约 0.10 米。如果选几何中心，模型会更复杂。激光和 IMU 等传感器坐标系相对 base_link 定义，转向正方向为向左。
**追问 1**：*What is the minimum turning radius?* The wheelbase divided by the tangent of the maximum steering angle: 0.40 over tan 30 degrees, about 0.69 metre.
　中文：轴距除以最大转向角的正切：0.40 除以 tan30°，约 0.69 米。
**追问 2**：*Why does the footprint matter for planning?* The planner checks collisions of the whole shape, especially the front corners when turning.
　中文：规划器检查整个形状的碰撞，尤其是转弯时的前角。
**如果追问到你的项目**：*I configured the footprint and the turning radius in the planner and costmap; for planning I used 0.8 metre rather than the geometric 0.69 to leave a margin.*
　中文：我在规划器和代价地图里配置了轮廓与转弯半径；规划取 0.8 米而不是几何的 0.69 米以留余量。
**关键词与易错点**：rear-axle reference、no lateral slip、footprint、minimum turning radius。

---

## 3. B 正逆运动学、雅可比与奇异性（Q13–Q28）

### Q13. Explain forward kinematics with DH parameters.
**中文题意**：用 DH 参数解释正运动学。
**考察点**：串联臂建模。
**项目关联**：P2（URDF/KDL 的等价）。
**English answer（口语）**：
> Forward kinematics computes the end-effector pose from the joint angles. With the Denavit–Hartenberg convention, each link is described by four parameters: the link length a, the link twist alpha, the link offset d, and the joint angle theta. Each joint gives a homogeneous transform, a rotation about z by theta, a translation along z by d, a translation along x by a, and a rotation about x by alpha. The end-effector pose is the product of these transforms from the base to the tool. There are two variants, standard and modified DH, which place the frames differently, so I always say which one I use. In practice, URDF describes the same chain with joint origins and axes instead.

**中文翻译**：正运动学由关节角计算末端位姿。DH 约定下，每个连杆由四个参数描述：连杆长度 a、扭角 alpha、偏移 d 和关节角 theta。每个关节给出一个齐次变换：绕 z 转 theta、沿 z 平移 d、沿 x 平移 a、绕 x 转 alpha。末端位姿是从基座到工具的这些变换之积。有标准与改进两种 DH 变体，坐标系放置不同，所以我总是说明用的是哪种。实际中，URDF 用关节原点和轴来描述同样的链。
**追问 1**：*Why do many people prefer URDF or POE?* They avoid the awkward frame placement rules of DH and handle arbitrary joint axes in a uniform way.
　中文：避免 DH 麻烦的坐标系放置规则，并统一处理任意关节轴。
**追问 2**：*How many parameters does a six-joint arm need in DH?* Four per link, so 24, of which six are the joint variables.
　中文：每个连杆四个，共 24 个，其中六个是关节变量。
**如果追问到你的项目**：*In P2 the arm geometry came from the URDF and was evaluated by KDL, so I used the same idea through those tools.*
　中文：P2 里臂的几何来自 URDF 并由 KDL 计算，所以我通过这些工具使用了同样的思想。
**关键词与易错点**：Denavit–Hartenberg、link twist、standard vs modified、URDF。

### Q14. What is the product of exponentials formulation, and when is it better than DH?
**中文题意**：指数积（POE）公式是什么？何时优于 DH？
**考察点**：现代运动学表示。
**项目关联**：概念题。
**English answer（口语）**：
> In the product of exponentials formulation, I describe each joint by its screw axis in the base frame, a unit direction plus a point on the axis. The forward kinematics is the product of matrix exponentials of each screw axis times its joint angle, multiplied by the home configuration M of the end effector: T equals e to the S one theta one, up to e to the S n theta n, times M. It is better than DH in a few ways. It needs only a base frame and a tool frame, with no frame assignment rules; it handles any joint axes uniformly; and the Jacobian comes out naturally from the screw axes. It is the formulation used in modern robotics textbooks and fits well with Lie group tools.

**中文翻译**：指数积公式用每个关节在基座系下的螺旋轴来描述，即一个单位方向加轴上一点。正运动学是各螺旋轴乘关节角的矩阵指数之积，再乘末端的初始位形 M：T 等于 e 的 S1 theta1 次方，直到 e 的 Sn thetan 次方，再乘 M。它在几方面优于 DH：只需要基座系和工具系，没有坐标系分配规则；统一处理任意关节轴；雅可比自然地由螺旋轴得出。它是现代机器人学教材使用的形式，与李群工具配合得很好。
**追问 1**：*What is a screw axis?* A representation of a rigid motion as a rotation about an axis combined with a translation along it.
　中文：把刚体运动表示为绕某轴的旋转加沿该轴的平移。
**追问 2**：*Does POE give the same result as DH?* Yes; they describe the same kinematics in different parametrisations.
　中文：是；它们是同一运动学的不同参数化。
**如果追问到你的项目**：*I did not write POE code; I know it as the cleaner theory behind the Jacobians in the libraries I used.*
　中文：我没有写过 POE 代码；我把它当作我所用库中雅可比背后更干净的理论来了解。
**关键词与易错点**：product of exponentials、screw axis、home configuration、matrix exponential。诚实说明。

### Q15. What is the difference between analytical and numerical inverse kinematics?
**中文题意**：解析逆运动学与数值逆运动学有何区别？
**考察点**：IK 方法选择。
**项目关联**：P2（数值/QP IK）。
**English answer（口语）**：
> Analytical IK derives closed-form formulas for the joint angles. It is fast, gives all solutions, and has no convergence issues, but it exists only for special geometries, such as six-joint arms with a spherical wrist, and it is hard to add constraints. Numerical IK iterates from an initial guess, using the Jacobian to reduce the task error, as in Newton methods or the damped least-squares method. It works for any arm, including redundant ones, and constraints and secondary tasks can be added, but it can fail to converge, gives one solution depending on the initial guess, and costs more computation. For teleoperation of a seven-joint arm, a numerical, constrained approach was the natural choice.

**中文翻译**：解析 IK 推导出关节角的闭式公式。速度快、给出全部解、没有收敛问题，但只适用于特殊几何，如带球形手腕的六关节臂，且难以加入约束。数值 IK 从初始猜测迭代，用雅可比减小任务误差，如牛顿法或阻尼最小二乘。它适用于任何臂（包括冗余臂），可加入约束和次要任务，但可能不收敛，只给出取决于初值的一个解，计算量也更大。对七关节臂的遥操作，数值带约束的方法是自然选择。
**追问 1**：*How many solutions does a typical six-joint arm have?* Up to eight in general for a spherical-wrist arm (shoulder, elbow, wrist flips).
　中文：带球形手腕的臂通常最多八个解（肩、肘、腕翻转）。
**追问 2**：*How do you choose among solutions?* The closest to the current joint configuration, within limits and away from singularities.
　中文：选最接近当前关节构型、在限位内且远离奇异的解。
**如果追问到你的项目**：*In P2 each arm has seven joints, redundant for a six-dimensional pose, so I used a constrained QP-based IK with Mink, started from the current joint state each cycle.*
　中文：P2 里每条臂七个关节，对六维位姿冗余，所以我用 Mink 的带约束 QP 式 IK，每周期从当前关节状态出发。
**关键词与易错点**：closed-form IK、numerical IK、spherical wrist、multiple solutions、initial guess。

### Q16. Define the Jacobian of a manipulator, and compute it for a two-link planar arm.
**中文题意**：定义机械臂的雅可比，并算一个平面二连杆的雅可比。
**考察点**：能现场推导。
**项目关联**：P2、P7。
**English answer（口语）**：
> The Jacobian maps joint velocities to the end-effector velocity: x dot equals J of q times q dot. For a planar two-link arm with link lengths l1 and l2, the tip position is x equals l1 cosine q1 plus l2 cosine of q1 plus q2, and y equals l1 sine q1 plus l2 sine of q1 plus q2. Differentiating, the first row of J is minus l1 sine q1 minus l2 sine q12, and minus l2 sine q12. The second row is l1 cosine q1 plus l2 cosine q12, and l2 cosine q12. The determinant is l1 l2 sine q2, so the arm is singular when q2 is zero or pi, which means fully stretched or fully folded.

**中文翻译**：雅可比把关节速度映射为末端速度：x 导数等于 J(q) 乘 q 导数。对连杆长度为 l1 和 l2 的平面二连杆臂，末端位置 x 等于 l1 cos q1 加 l2 cos(q1 加 q2)，y 等于 l1 sin q1 加 l2 sin(q1 加 q2)。求导后，J 的第一行是 负 l1 sin q1 减 l2 sin q12，以及 负 l2 sin q12；第二行是 l1 cos q1 加 l2 cos q12，以及 l2 cos q12。行列式为 l1 l2 sin q2，所以 q2 为零或 pi 时奇异，即完全伸直或完全折叠。
**追问 1**：*What does the Jacobian transpose do?* It maps a tip force to joint torques: tau equals J transpose F.
　中文：把末端力映射为关节力矩：tau 等于 J 转置 F。
**追问 2**：*What is the geometric versus analytical Jacobian?* The geometric one maps to linear and angular velocity; the analytical one maps to the derivative of a chosen orientation parametrisation, and they differ by a matrix that depends on the parametrisation.
　中文：几何雅可比映射到线速度和角速度；解析雅可比映射到所选姿态参数化的导数，两者差一个依赖于参数化的矩阵。
**如果追问到你的项目**：*I use this relation for torque mapping in the wheel-legged robot, where the leg Jacobian maps the virtual leg force to hip torques.*
　中文：轮腿机器人里我用这个关系做力矩映射，腿的雅可比把虚拟腿的力映射到髋关节力矩。
**关键词与易错点**：Jacobian、determinant、singular configuration、J transpose。

### Q17. What is a kinematic singularity, and what types are there?
**中文题意**：什么是运动学奇异点？有哪些类型？
**考察点**：奇异性理论。
**项目关联**：P2（核心）。
**English answer（口语）**：
> A singularity is a configuration where the Jacobian loses rank, so the end effector cannot move in some direction no matter how the joints move, and conversely, moving in the neighbouring direction needs very large joint speeds. There are two classic types. Boundary singularities happen at the edge of the workspace, such as the arm fully stretched. Internal singularities happen inside the workspace, where axes line up, for example the wrist singularity when two wrist axes become collinear. Near a singularity the inverse Jacobian blows up, so joint velocities become huge for a small task command. Remedies: avoid them in the planning, use damped least squares, and limit the joint velocities.

**中文翻译**：奇异点是雅可比降秩的位形，此时无论关节怎么动，末端都不能向某些方向运动；反过来，向邻近方向运动需要非常大的关节速度。有两类经典情形。边界奇异出现在工作空间边缘，如臂完全伸直。内部奇异出现在工作空间内部，轴线对齐时，如两个手腕轴共线造成的腕部奇异。奇异点附近雅可比的逆发散，小的任务指令对应巨大的关节速度。对策：规划时避开、使用阻尼最小二乘、限制关节速度。
**追问 1**：*How do you detect proximity to a singularity?* Monitor the smallest singular value, the determinant, or the condition number of the Jacobian.
　中文：监控雅可比的最小奇异值、行列式或条件数。
**追问 2**：*What is the cost of damping?* A tracking error in the singular direction, in exchange for bounded speeds.
　中文：在奇异方向上产生跟踪误差，换取有界速度。
**如果追问到你的项目**：*In P2 I monitored the smallest singular value, marked red below 0.05 and reduced target speed; a real fault was a joint spinning quickly near a singular pose, which damping and speed clamps addressed.*
　中文：P2 里我监控最小奇异值，低于 0.05 标红并降低目标速度；一次真实故障是近奇异姿态下关节猛转，用阻尼和速度钳位解决。
**关键词与易错点**：singularity、rank loss、boundary vs internal、wrist singularity、smallest singular value。

### Q18. How does a redundant arm use its extra degree of freedom?
**中文题意**：冗余机械臂如何利用多余的自由度？
**考察点**：零空间与次要任务。
**项目关联**：P2（七自由度臂）。
**English answer（口语）**：
> A seven-joint arm has one more degree of freedom than the six needed for the pose. The extra freedom appears as a null space of the Jacobian: self-motion of the joints that does not move the end effector, like swinging the elbow. It can be used for secondary objectives: staying away from joint limits, avoiding obstacles, maintaining a comfortable posture, or moving away from singularities. In velocity-level IK, the joint velocity is the primary-task solution plus the null-space projector times a secondary velocity. In a QP formulation, I can express this as multiple weighted tasks with priorities. The primary task is the end-effector pose, and the secondary tasks have a smaller weight.

**中文翻译**：七关节臂比位姿所需的六个自由度多一个。多出的自由度体现为雅可比的零空间：不移动末端的关节自运动，比如摆动肘部。它可用于次要目标：远离关节限位、避障、保持舒适姿态或远离奇异点。在速度层 IK 中，关节速度等于主任务解加零空间投影器乘次要速度。在 QP 形式里，可以表示成带优先级的多个加权任务：主任务是末端位姿，次要任务权重较小。
**追问 1**：*How do priorities appear in a weighted QP?* Through weights, a larger weight for the primary task, so it wins in conflicts; strict priorities need hierarchical methods.
　中文：通过权重，主任务权重大，冲突时占优；严格优先级需要分层方法。
**追问 2**：*What can go wrong with a pure null-space motion?* Joint drift in the null space, since the primary task does not constrain it.
　中文：零空间里的关节漂移，因为主任务不约束它。
**如果追问到你的项目**：*In P2, I mainly relied on joint limits and damping to keep the solution well-behaved, rather than designing an explicit secondary posture task.*
　中文：P2 里我主要靠关节限位和阻尼保持解的良态，而不是设计显式的次要姿态任务。
**关键词与易错点**：redundancy、null space、secondary task、weighted tasks。

### Q19. How do joint limits enter inverse kinematics?
**中文题意**：关节限位如何进入逆运动学？
**考察点**：约束 IK 实践。
**项目关联**：P2（核心）。
**English answer（口语）**：
> In velocity-level IK, position limits become velocity bounds: the joint velocity times the time step must not take the joint beyond its limit. So lower limit minus current position over dt is a lower bound on the velocity, and upper limit minus position over dt is an upper bound. I also add velocity limits. These are linear inequality constraints on the unknown joint velocity, so the problem remains a QP. I keep the limits slightly inside the physical ones, for example by 0.05 radian, and apply the speed limit again at the driver as a second layer, because I should not depend on one layer only. If the target is unreachable under the limits, the QP returns the closest feasible motion.

**中文翻译**：速度层 IK 里，位置限位变成速度界：关节速度乘时间步长不能使关节越过限位。于是（下限减当前位置）除以 dt 是速度下界，（上限减位置）除以 dt 是上界。我还加入速度限制。它们是关于未知关节速度的线性不等式约束，所以问题仍是 QP。我让限位略小于物理限位（如内缩 0.05 rad），并在驱动器处再施加一次速度限制作为第二层，因为不应只依赖一层。若目标在限位下不可达，QP 返回最接近的可行运动。
**追问 1**：*Which speed limits did you use?* About 2.5 radians per second on the first four joints and 4 on the last three, much below the URDF limits.
　中文：前四个关节约 2.5 rad/s，后三个约 4 rad/s，远低于 URDF 限制。
**追问 2**：*Why not rely on the solver default limits?* A default may change between versions; I pass the limits explicitly and do not assert any version's default behaviour.
　中文：默认行为可能随版本改变；我显式传入限制，不断言任何版本的默认行为。
**如果追问到你的项目**：*That is how P2 worked: joint limits as bounds in the QP, extra speed limits at the target generator and at the driver, and the position limits inset by 0.05 radian.*
　中文：P2 就是这样：关节限位作为 QP 的界，目标生成端和驱动端再加速度限制，位置限位内缩 0.05 rad。
**关键词与易错点**：joint limits、velocity bounds、linear inequality、two layers of limits。

### Q20. Why use damped least squares rather than the plain pseudo-inverse in IK, and how do you choose the damping?
**中文题意**：IK 中为什么用阻尼最小二乘而不是普通伪逆？阻尼怎么选？
**考察点**：与数学题互补的运动学角度。
**项目关联**：P2。
**English answer（口语）**：
> The pseudo-inverse gives the minimum-norm joint velocity, which behaves well away from singularities but blows up near them. Damped least squares trades a bit of accuracy for bounded speed. The damping can be constant, which is simple, or adaptive, increasing only when the smallest singular value is below a threshold. I choose the constant by looking at the trade-off: too small and the arm still whips near singular poses; too large and the end effector lags and feels sluggish. I test by slowly reaching toward the workspace boundary and watching the joint speed, and I set a monitor on the smallest singular value. For teleoperation, a small lag near the boundary is much better than a violent joint motion.

**中文翻译**：伪逆给出范数最小的关节速度，远离奇异点时表现良好，靠近时发散。阻尼最小二乘用一点精度换取有界速度。阻尼可以是常数（简单），也可以自适应，仅在最小奇异值低于阈值时增大。我通过权衡来选常数：太小，近奇异姿态下臂仍会猛甩；太大，末端滞后、手感迟钝。我通过缓慢伸向工作空间边界并观察关节速度来测试，并对最小奇异值设置监控。对遥操作，边界附近小的滞后远好于猛烈的关节运动。
**追问 1**：*What numbers did you use?* A global damping of about ten to the minus three, and a task damping of about 0.05, with the singular-value alarm at 0.05.
　中文：全局阻尼约 10 的负 3 次方，任务阻尼约 0.05，奇异值告警 0.05。
**追问 2**：*Is there a regression test?* Slowly reach toward the boundary twenty times and verify the joint speeds stay below the limits.
　中文：缓慢伸向边界 20 次，验证关节速度不超限。
**如果追问到你的项目**：*That is my P2 CV statement of suppressing velocity amplification near singularities.*
　中文：这就是我 P2 CV 里"抑制奇异点附近速度放大"的内容。
**关键词与易错点**：damped least squares、adaptive damping、trade-off、singular value monitor。

### Q21. Explain the closed-chain five-bar leg kinematics in your wheel-legged robot.
**中文题意**：请讲讲轮腿机器人的闭链五连杆腿运动学。
**考察点**：闭链运动学的核心步骤。
**项目关联**：P7（核心）。
**English answer（口语）**：
> The leg is a five-bar linkage: two motors at the hip drive two active links, and the other two links are passive and join at the knee, with a fifth, fixed link between the motor axes. Link lengths are about 150, 250 and 108 millimetres, with the two upper links 150 and the two lower links 250, and the fixed base 108. Forward kinematics: from the two motor angles I compute the two elbow points, then find the foot point as the intersection of two circles of radius 250 around the elbows. Choosing the right intersection is a branch choice. Then I define the virtual leg from the hip centre to the foot, which gives a leg length and a leg angle. The Jacobian maps motor velocities to virtual-leg motion.

**中文翻译**：腿是五连杆机构：髋部的两个电机驱动两个主动杆，另两个杆是被动杆，在膝部相接，电机轴之间有第五个固定杆。杆长约 150、250 和 108 毫米，上面两杆 150，下面两杆 250，固定基座 108。正运动学：由两个电机角度计算两个肘点，再把脚点求为以两个肘点为圆心、半径 250 的两个圆的交点，选择正确的交点是分支选择。然后定义从髋中心到脚的虚拟腿，得到腿长和腿角。雅可比把电机速度映射为虚拟腿运动。
**追问 1**：*When does the leg become singular?* When the lower links are collinear with the upper ones, such as a fully stretched leg, or when the two lower links overlap; the working leg length is kept within about 0.10 to 0.30 metre.
　中文：下杆与上杆共线（如腿完全伸直）或两下杆重合时；工作腿长保持在约 0.10–0.30 m。
**追问 2**：*Why use a closed chain?* Both motors sit at the hip, so the leg is light and the inertia low; the price is more complex kinematics.
　中文：两个电机都在髋部，腿轻、惯量低；代价是运动学更复杂。
**如果追问到你的项目**：*This is the CV statement: virtual leg length and posture from closed-chain kinematics, then support force and pitch torque mapped to the joints by the Jacobian.*
　中文：这是 CV 的描述：由闭链运动学得到虚拟腿长和姿态，再用雅可比把支撑力和俯仰力矩映射到关节。
**关键词与易错点**：five-bar linkage、two-circle intersection、branch choice、virtual leg、Jacobian。

### Q22. What sign conventions did you need to fix for the virtual leg?
**中文题意**：虚拟腿方面你需要统一哪些符号约定？
**考察点**：工程细节与诚实。
**项目关联**：P7。
**English answer（口语）**：
> There are three places where signs matter. The leg angle: I define it relative to the vertical with forward lean positive, and make sure that the sign of the motor angle mapping matches that. The pitch of the body: forward lean positive, with the equilibrium offset of about 1.5 degrees removed. And the torque signs: a positive virtual-leg force should extend the leg, and a positive pitch torque on the leg should push the body in the direction that reduces the forward lean. I checked each by a bench test with the robot lifted: apply a small torque on each motor, and check the direction against the model. Doing this before any closed-loop run prevented the classic problem that the controller has positive feedback.

**中文翻译**：有三处符号很关键。腿角：相对竖直定义，前倾为正，并确保电机角映射的符号与之匹配。机身俯仰：前倾为正，并扣除约 1.5 度的平衡点偏置。力矩符号：正的虚拟腿力应使腿伸长，腿上正的俯仰力矩应把机身推向减小前倾的方向。我在机器人悬空时用台架测试逐个验证：对每个电机施加小力矩，对照模型检查方向。在任何闭环运行之前做这件事，避免了控制器出现正反馈的经典问题。
**追问 1**：*What happens with a wrong sign?* The controller amplifies the error instead of reducing it, and the robot falls immediately.
　中文：控制器放大误差而不是减小，机器人立刻倒下。
**追问 2**：*How do you document it?* One table of directions, zero positions and signs, kept with the code.
　中文：一张方向、零位和符号表，与代码放在一起。
**如果追问到你的项目**：*In P7 I asked for the written table of motor directions and zero positions before applying any torque, so a reversed sign was caught on the bench.*
　中文：P7 里我要求在施加任何力矩之前先有书面的电机方向和零位表，所以反号在台架上就被发现了。
**关键词与易错点**：sign convention、leg angle、equilibrium offset、bench test。

### Q23. Compare differential-drive, Ackermann and Mecanum kinematics.
**中文题意**：比较差速、阿克曼和麦克纳姆运动学。
**考察点**：底盘模型。
**项目关联**：P2（差速）、P3（麦轮）、P5（阿克曼）。
**English answer（口语）**：
> A differential drive has two independently driven wheels. It can spin in place and has the simple relation v equals the mean of the wheel speeds and omega equals the difference over the track width. It is nonholonomic: it cannot move sideways. An Ackermann or car-like vehicle steers the front wheels, has a minimum turning radius of wheelbase over tangent of the maximum steering angle, and cannot spin in place, so it needs planners that respect curvature. A Mecanum platform has four wheels with angled rollers, so it is holonomic: it can translate in any direction and rotate independently, at the cost of lower efficiency, more slip and odometry error. The choice depends on speed, payload, terrain and manoeuvrability.

**中文翻译**：差速驱动有两个独立驱动的轮。它可以原地旋转，关系简单：v 等于两轮速度的平均，omega 等于速度差除以轮距。它是非完整约束的：不能侧向移动。阿克曼或车式车辆转向前轮，最小转弯半径为轴距除以最大转向角的正切，不能原地旋转，所以需要考虑曲率的规划器。麦克纳姆平台有四个带斜向滚子的轮，是完整约束的：可向任意方向平移并独立旋转，代价是效率较低、滑移与里程误差更大。选择取决于速度、载重、地形和机动性。
**追问 1**：*Which of these did you build?* Differential drive in P2, Mecanum in P3, and Ackermann in P5.
　中文：P2 差速，P3 麦轮，P5 阿克曼。
**追问 2**：*Why is Mecanum odometry worse?* Roller slip is larger and depends on the surface, so wheel-based odometry is less reliable.
　中文：滚子滑移更大并取决于地面，轮式里程计更不可靠。
**如果追问到你的项目**：*I have worked with all three, so I can compare them from the planning side: curvature constraints for the car, none for the Mecanum.*
　中文：三种我都用过，所以能从规划角度比较：车有曲率约束，麦轮没有。
**关键词与易错点**：differential drive、Ackermann、Mecanum、holonomic、nonholonomic。

### Q24. What is a nonholonomic constraint, and why does it matter for planning?
**中文题意**：什么是非完整约束？为什么对规划重要？
**考察点**：车辆规划的数学基础。
**项目关联**：P5（Hybrid-A*）。
**English answer（口语）**：
> A nonholonomic constraint restricts the velocities but not the reachable positions. A car cannot move sideways instantaneously: the velocity must be along the heading, written as x dot sine psi minus y dot cosine psi equals zero. Yet it can reach any position and heading by manoeuvring, like parallel parking. This matters because a planner that ignores it, for example a grid A-star that plans a path of straight segments with sharp corners, produces paths that the car cannot follow. Planners such as Hybrid-A-star include the heading in the state and expand along kinematically feasible arcs with a minimum turning radius, so that the path is feasible.

**中文翻译**：非完整约束限制速度而不限制可达位置。车不能瞬时侧移：速度必须沿航向，写成 x 导数乘 sin psi 减 y 导数乘 cos psi 等于零。但通过机动（比如侧方位停车）它可以到达任何位置与航向。这对规划重要：忽略它的规划器（如在栅格上规划带尖锐拐角直线段的 A*）会产生车无法跟随的路径。Hybrid-A* 这样的规划器把航向放进状态，沿具有最小转弯半径的运动学可行弧线扩展，使路径可行。
**追问 1**：*Is a differential drive nonholonomic too?* Yes, but it can rotate in place, so path feasibility is much easier.
　中文：是，但它能原地旋转，所以路径可行性容易得多。
**追问 2**：*How does Reeds–Shepp relate?* It gives the shortest paths for a car with forward and reverse motion, used as an analytic expansion toward the goal.
　中文：给出可前进与倒车的车的最短路径，用作朝目标的解析扩展。
**如果追问到你的项目**：*In P5, Hybrid-A-star used 72 heading bins of 5 degrees, a turning radius of 0.8 metre, and Reeds–Shepp analytic expansion.*
　中文：P5 里 Hybrid-A* 用 72 个航向格（每格 5°）、0.8 m 转弯半径和 Reeds–Shepp 解析扩展。
**关键词与易错点**：nonholonomic、velocity constraint、minimum turning radius、Reeds–Shepp。

### Q25. How do you convert a velocity command into wheel speeds for a car-like robot?
**中文题意**：车式机器人如何把速度指令转换成轮指令？
**考察点**：阿克曼转向几何。
**项目关联**：P5、P6（Twist 映射）。
**English answer（口语）**：
> A Twist command has a linear speed v and a yaw rate omega. For a car-like robot with wheelbase L, the steering angle follows from the bicycle model: tan delta equals L times omega over v, so delta equals the arctangent of L omega over v. I clip delta to the maximum steering angle, which also limits the achievable omega at a given speed. At very low speed the formula is ill-defined, so I handle it with a minimum speed or treat it as a special case. The rear wheel speed is v. For a more careful treatment, the inner and outer steered wheels have slightly different angles, following Ackermann geometry. A twist with omega and no speed cannot be executed by a car, so the layer above must know that.

**中文翻译**：Twist 指令包含线速度 v 和偏航角速度 omega。对轴距为 L 的车式机器人，转向角来自自行车模型：tan delta 等于 L 乘 omega 除以 v，delta 等于 L omega/v 的反正切。我把 delta 限制在最大转向角内，这也限制了给定速度下可达的 omega。速度很低时公式未定义，用最小速度或特殊处理。后轮速度为 v。更精细的处理中，内外侧转向轮的角度略有不同，遵循阿克曼几何。只有 omega 而没有速度的 Twist 车无法执行，所以上层必须知道这一点。
**追问 1**：*What is the limit of this model for a large steering angle?* The inner and outer wheels differ, and tyre slip appears at high speed.
　中文：大转向角时内外轮不同，高速时出现轮胎滑移。
**追问 2**：*How did this appear in your platform work?* The capability model needs to tell the upper layer that a Twist alone is not enough for an Ackermann base, because it needs a speed and a steering limit.
　中文：能力模型需要告诉上层，对阿克曼底盘仅有 Twist 不够，还需要速度和转向限制。
**如果追问到你的项目**：*In P6 mapping ROS Twist to different bases was one part of the work, with the steering angle formula and acceleration limits as typical examples.*
　中文：P6 里把 ROS Twist 映射到不同底盘是工作的一部分，转向角公式和加速度限幅是典型例子。
**关键词与易错点**：Twist、bicycle model、steering angle、Ackermann geometry、minimum speed。

### Q26. What is manipulability and workspace, and why do they matter when choosing a posture?
**中文题意**：可操作度和工作空间是什么？选姿态时为何重要？
**考察点**：姿态选择的实际依据。
**项目关联**：P2。
**English answer（口语）**：
> The workspace is the set of end-effector poses the arm can reach. The dexterous workspace is the subset where the arm can reach with any orientation. Manipulability measures how easily the end effector can move in a given configuration, based on the singular values of the Jacobian. Near the workspace boundary, manipulability is low, and in the middle it is high. When choosing a posture for teaching or teleoperation, I prefer configurations with high manipulability and joint angles away from their limits, so the operator has a comfortable range of motion and the IK stays well-conditioned. That also affects where the operator should stand relative to the robot base.

**中文翻译**：工作空间是机械臂能到达的末端位姿集合。灵巧工作空间是其中能以任意姿态到达的子集。可操作度衡量在给定位形下末端移动的难易，基于雅可比的奇异值。工作空间边界附近可操作度低，中间高。为示教或遥操作选姿态时，我偏好可操作度高、关节角远离限位的位形，使操作者有舒适的运动范围，IK 保持良态。这也影响操作者相对机器人基座应该站在哪里。
**追问 1**：*How can you visualise it?* Plot the smallest singular value over the reachable workspace as a colour map.
　中文：把可达工作空间上的最小奇异值画成色图。
**追问 2**：*Does a larger workspace always help?* More reach often means lower stiffness and accuracy at the extremes.
　中文：更大的作用范围常意味着极限处刚度和精度更低。
**如果追问到你的项目**：*In P2 the workspace limit of about plus or minus 0.4 metre around the reference pose kept the target away from the boundary.*
　中文：P2 里以参考位姿为中心约 ±0.4 m 的工作空间限制，使目标远离边界。
**关键词与易错点**：workspace、dexterous workspace、manipulability、well-conditioned。

### Q27. What is the relationship between forward and inverse kinematics at the velocity level?
**中文题意**：速度层的正运动学与逆运动学是什么关系？
**考察点**：Jacobian 的两种用法。
**项目关联**：P2。
**English answer（口语）**：
> Forward velocity kinematics is x dot equals J q dot: given the joint velocities, I get the end-effector velocity. It is direct and always defined. Inverse velocity kinematics is the opposite: given a desired end-effector velocity, find q dot. If J is square and invertible, q dot equals J inverse x dot. In general, I solve it by least squares, so q dot is the pseudo-inverse times x dot, plus any null-space motion. To track a target pose, I use a velocity command proportional to the pose error, x dot equals K times the error, and then integrate the joint velocities over the time step. This is closed-loop inverse kinematics, and it avoids drift from pure integration.

**中文翻译**：速度正运动学是 x 导数等于 J q 导数：给定关节速度，得到末端速度，直接且总有定义。速度逆运动学相反：给定期望末端速度，求 q 导数。若 J 是方阵且可逆，q 导数等于 J 逆乘 x 导数。一般情况下用最小二乘求解，q 导数等于伪逆乘 x 导数，再加任意零空间运动。为跟踪目标位姿，我用与位姿误差成比例的速度指令，x 导数等于 K 乘误差，然后在时间步上积分关节速度。这是闭环逆运动学，避免了纯积分的漂移。
**追问 1**：*Why closed-loop?* Pure integration of q dot drifts because of numerical error; feeding back the pose error removes it.
　中文：纯积分 q 导数因数值误差漂移；反馈位姿误差可消除。
**追问 2**：*What if K is too large?* The discrete update can oscillate or overshoot; K times dt should stay below about one.
　中文：离散更新可能振荡或过冲；K 乘 dt 应小于约 1。
**如果追问到你的项目**：*In P2 the IK ran each cycle from the current measured joints with a pose-error task, which is this closed-loop form.*
　中文：P2 里 IK 每个周期从当前测量的关节出发、以位姿误差为任务，就是这种闭环形式。
**关键词与易错点**：velocity kinematics、pseudo-inverse、closed-loop IK、drift。

### Q28. How would you check a kinematic model quickly on a new robot?
**中文题意**：在新机器人上如何快速检查运动学模型？
**考察点**：验证方法。
**项目关联**：P2、P7。
**English answer（口语）**：
> I use several simple checks. Zero pose: put the robot at the known zero configuration, and check that the model gives the expected end-effector position. Single-joint motions: move one joint by a small known angle, and check the direction and magnitude of the tool displacement against the model. Repeatability: go to a few poses and compare the model and the measurement with a ruler or a calibration board. Consistency: compare forward kinematics with the inverse kinematics, which should recover the joint angles. Numerical Jacobian check: compare the analytical Jacobian with finite differences of the forward kinematics. Any mismatch usually comes from a sign convention, a zero offset or a link-length typo.

**中文翻译**：我用几个简单检查。零位：让机器人处于已知零位，检查模型给出的末端位置是否符合预期。单关节运动：把一个关节移动一个已知小角度，对照模型检查工具位移的方向和大小。重复性：到几个位姿，用尺子或标定板比较模型与测量。一致性：比较正运动学与逆运动学，逆解应能恢复关节角。数值雅可比检验：把解析雅可比与正运动学的有限差分比较。任何不一致通常来自符号约定、零位偏置或连杆长度的笔误。
**追问 1**：*Why compare with finite differences?* It catches errors in the analytical Jacobian cheaply.
　中文：廉价地发现解析雅可比中的错误。
**追问 2**：*What if the model is right but the robot differs?* Then I calibrate the link lengths and joint offsets from measurements.
　中文：则根据测量标定杆长和关节偏置。
**如果追问到你的项目**：*In P2, I aligned the master and follower arms to the same known pose and measured the joint offsets, which were small, under 0.03 radian.*
　中文：P2 里我把主从臂摆到同一已知姿态并测量关节偏置，量级很小，小于 0.03 rad。
**关键词与易错点**：zero pose、finite differences、sign check、calibration。

---

## 4. C 动力学与建模（Q29–Q43）

### Q29. When would you use Newton–Euler and when Lagrange to derive robot dynamics?
**中文题意**：什么时候用牛顿—欧拉，什么时候用拉格朗日推导机器人动力学？
**考察点**：方法选择。
**项目关联**：P1、P7（倒立摆）、P2（机械臂）。
**English answer（口语）**：
> Newton–Euler writes force and moment balance for each body. It gives the constraint forces explicitly, and it is recursive, so it is efficient for computing the dynamics numerically, which is what libraries do. Lagrange works with energies: I write the kinetic and potential energy in generalised coordinates, and the equations of motion come out by differentiation, with no constraint forces to eliminate. So for deriving a compact analytical model by hand, such as a cart–pole or a two-link arm, I prefer Lagrange. For numerical computation of a full arm, I use Newton–Euler-based algorithms in Pinocchio or KDL. Both give the same equations of motion.

**中文翻译**：牛顿—欧拉对每个刚体写力和力矩平衡。它显式给出约束力，并且是递归的，所以数值计算动力学很高效，库就是这样做的。拉格朗日用能量：在广义坐标下写动能和势能，运动方程通过微分得到，无需消去约束力。所以手工推导紧凑的解析模型（如小车摆或二连杆臂）时，我偏好拉格朗日；数值计算完整机械臂时，用 Pinocchio 或 KDL 中基于牛顿—欧拉的算法。两者给出相同的运动方程。
**追问 1**：*What is the complexity of recursive Newton–Euler?* Linear in the number of joints.
　中文：随关节数线性增长。
**追问 2**：*Which would you use for a robot with closed chains?* Lagrange with constraints and multipliers, or a method for closed-chain systems.
　中文：带约束和乘子的拉格朗日，或闭链系统的专用方法。
**如果追问到你的项目**：*I derived the inverted-pendulum models for the balance projects this way, and used the libraries for the arm's gravity terms.*
　中文：平衡类项目的倒立摆模型我用这种方式推导，机械臂的重力项用库计算。
**关键词与易错点**：Newton–Euler、Lagrange、generalised coordinates、recursive、constraint forces。

### Q30. Write the manipulator equation and state its main properties.
**中文题意**：写出机械臂运动方程并说明其主要性质。
**考察点**：动力学结构。
**项目关联**：P2。
**English answer（口语）**：
> The equation is M of q times q double dot, plus C of q and q dot times q dot, plus g of q, equals tau, where tau includes the motor torques and external torques. The inertia matrix M is symmetric and positive definite, and it depends on the configuration. The Coriolis and centrifugal matrix C is chosen so that M dot minus 2C is skew-symmetric, which is the key to passivity-based stability proofs. The gravity vector g is the gradient of the potential energy. The dynamics are linear in a suitable set of inertial parameters, which is used in identification and adaptive control. These structural properties give simple controllers like PD with gravity compensation a clean stability argument.

**中文翻译**：方程是 M(q) q 二阶导，加 C(q, q 导数) q 导数，加 g(q)，等于 tau，tau 包含电机力矩和外力矩。惯性矩阵 M 对称正定，并依赖位形。科氏与离心矩阵 C 的选取使 M 的导数减 2C 反对称，这是基于无源性的稳定性证明的关键。重力向量 g 是势能的梯度。动力学对一组适当的惯性参数是线性的，用于辨识和自适应控制。这些结构性质给 PD 加重力补偿这样的简单控制器提供了清晰的稳定性论证。
**追问 1**：*Why is M positive definite?* It comes from the kinetic energy, which is positive for any non-zero velocity.
　中文：来自动能，对任何非零速度为正。
**追问 2**：*What does the Coriolis term do?* It couples joint velocities; it does no net work on the system.
　中文：耦合关节速度；对系统不做净功。
**如果追问到你的项目**：*I used the gravity term g of q from the URDF for the leader arm's compensation in P2.*
　中文：P2 里主臂补偿使用了来自 URDF 的重力项 g(q)。
**关键词与易错点**：inertia matrix、Coriolis、gravity vector、skew-symmetric、linear in parameters。

### Q31. Write the dynamics of a two-link planar arm.
**中文题意**：写出平面二连杆臂的动力学。
**考察点**：能现场写出。
**项目关联**：P2（概念）。
**English answer（口语）**：
> Let the link masses be m1 and m2, lengths l1 and l2, centre-of-mass distances lc1 and lc2, and inertias I1 and I2 about the centres. Then the inertia matrix has M11 equal to m1 lc1 squared plus m2 times l1 squared plus lc2 squared plus two l1 lc2 cosine q2, plus I1 plus I2. M12 equals m2 times lc2 squared plus l1 lc2 cosine q2, plus I2. M22 equals m2 lc2 squared plus I2. With h equal to m2 l1 lc2 sine q2, the Coriolis matrix has the first row minus h q2 dot and minus h times q1 dot plus q2 dot, and the second row h q1 dot and zero. Gravity is g1 equal to m1 lc1 plus m2 l1 times g cosine q1 plus m2 lc2 g cosine q12, and g2 equal to m2 lc2 g cosine q12, with angles from horizontal.

**中文翻译**：设连杆质量为 m1、m2，长度 l1、l2，质心距离 lc1、lc2，绕质心的转动惯量 I1、I2。惯性矩阵 M11 等于 m1 lc1 平方，加 m2 乘 (l1 平方加 lc2 平方加 2 l1 lc2 cos q2)，再加 I1 加 I2。M12 等于 m2 乘 (lc2 平方加 l1 lc2 cos q2) 加 I2。M22 等于 m2 lc2 平方加 I2。令 h 等于 m2 l1 lc2 sin q2，科氏项由 h 与 q1 导数、q2 导数的乘积构成（C 矩阵第一行为 负 h q2 导数 与 负 h (q1 导数加 q2 导数)，第二行为 h q1 导数 与 0）。重力项 g1 等于 (m1 lc1 加 m2 l1) g cos q1 加 m2 lc2 g cos q12，g2 等于 m2 lc2 g cos q12，角度从水平方向量起。
**追问 1**：*What does M12 tell you?* The coupling inertia between the joints; it depends on q2 only.
　中文：关节之间的耦合惯量，只依赖 q2。
**追问 2**：*Where is the gravity torque largest?* When the arm is stretched horizontally, where the cosines are one.
　中文：手臂水平伸直时，余弦为 1，重力矩最大。
**如果追问到你的项目**：*I did not hand-derive the seven-joint arm; I computed g of q by KDL from the URDF, and the two-link case is the sanity check of the structure.*
　中文：我没有手推七关节臂；g(q) 由 KDL 根据 URDF 计算，二连杆情形是对结构的合理性检查。
**关键词与易错点**：inertia matrix、Coriolis term、gravity term、angle reference。注意重力项角度是从水平方向量起。

### Q32. What are forward and inverse dynamics, and what are RNEA and ABA?
**中文题意**：什么是正动力学与逆动力学？RNEA 与 ABA 是什么？
**考察点**：算法层面的了解。
**项目关联**：P2（KDL/Pinocchio）。
**English answer（口语）**：
> Inverse dynamics computes the joint torques needed for a given motion: tau from q, q dot and q double dot. Forward dynamics goes the other way: given the torques, it computes the joint accelerations, which a simulator integrates. The recursive Newton–Euler algorithm, RNEA, does inverse dynamics in linear time in the number of joints: a forward pass propagates velocities and accelerations out along the chain, and a backward pass propagates forces back. The articulated body algorithm, ABA, does forward dynamics also in linear time. Libraries such as Pinocchio and KDL provide both. Gravity compensation is RNEA with zero velocity and acceleration.

**中文翻译**：逆动力学计算给定运动所需的关节力矩：由 q、q 导数、q 二阶导求 tau。正动力学反过来：给定力矩，计算关节加速度，由仿真器积分。递归牛顿—欧拉算法 RNEA 以关节数线性时间完成逆动力学：前向遍历沿链向外传播速度和加速度，后向遍历传回力。关节体算法 ABA 同样以线性时间完成正动力学。Pinocchio 和 KDL 等库两者都提供。重力补偿就是速度和加速度为零的 RNEA。
**追问 1**：*Why is the zero-velocity RNEA the gravity term?* With q dot and q double dot zero, only the gravity-induced torque remains.
　中文：q 导数和 q 二阶导为零，只剩下重力引起的力矩。
**追问 2**：*Which is used in simulation?* Forward dynamics via ABA or a constraint solver.
　中文：通过 ABA 或约束求解器的正动力学。
**如果追问到你的项目**：*In P2 the leader-arm gravity compensation was this RNEA-style gravity term from KDL, about 200 hertz.*
　中文：P2 里主臂重力补偿就是 KDL 的这类 RNEA 式重力项，约 200 Hz。
**关键词与易错点**：inverse dynamics、forward dynamics、RNEA、ABA、linear time。

### Q33. How does URDF define the dynamics of your robot, and what can go wrong?
**中文题意**：URDF 如何定义机器人的动力学？可能出什么问题？
**考察点**：模型参数质量。
**项目关联**：P1、P2（核心）。
**English answer（口语）**：
> URDF describes links and joints. Each link has a mass, a centre of mass and an inertia tensor, and each joint has an axis, an origin and limits. A dynamics library builds the equations of motion from these. What can go wrong: the mass or the centre of mass is from the CAD and ignores cables, screws, or payloads; the inertia is given about the wrong point or in the wrong frame; the units are off, for example millimetres versus metres; or the joint axis sign is reversed. So I always check the total mass against a weighed value, and test the gravity compensation by holding poses. In my ballbot work, the URDF mass was checked against the weighed total within about five percent.

**中文翻译**：URDF 描述连杆和关节。每个连杆有质量、质心和惯性张量，每个关节有轴、原点和限位。动力学库据此构建运动方程。可能出错的地方：质量或质心来自 CAD，忽略了线缆、螺钉或负载；惯量给在错误的点或错误的坐标系；单位不对，如毫米与米；或关节轴符号反了。所以我总是把总质量与称重值核对，并通过保持姿态来测试重力补偿。球形机器人工作里，URDF 质量与称重总质量核对在约 5% 以内。
**追问 1**：*How do you check inertia?* Compare simple motions, or identify the parameters from data; for gravity compensation, mass and centre of mass matter most.
　中文：比较简单运动，或从数据辨识参数；对重力补偿，质量与质心最关键。
**追问 2**：*Where did the P1 URDF come from?* From the ballbotgym MuJoCo environment, originally exported from CAD, with the root frame at the ball centre.
　中文：来自 ballbotgym 的 MuJoCo 环境，最初由 CAD 导出，根坐标在球心。
**如果追问到你的项目**：*For P2 the gravity compensation parameters were tuned with a ratio of 1 and 0.8, because the last joint carries an uncertain payload.*
　中文：P2 重力补偿比例取 1 和 0.8，因为最后一个关节载荷不确定。
**关键词与易错点**：URDF、inertia tensor、centre of mass、units、weighed mass。

### Q34. Derive the linearised cart–pole model.
**中文题意**：推导线性化的小车摆模型。
**考察点**：经典建模题。
**项目关联**：P1、P7（倒立摆）。
**English answer（口语）**：
> Take a cart of mass M and a pendulum of point mass m on a massless rod of length l. The tilt theta is from the vertical, and the force F acts on the cart. By Newton, the horizontal force balance on the whole system gives M plus m times x double dot plus m l theta double dot equals F, for small angles. The pendulum equation gives x double dot plus l theta double dot equals g theta. Solving these two, the cart acceleration is F minus m g theta, over M, and the angular acceleration is M plus m times g theta minus F, over M l. So the state-space matrix has a positive coefficient on theta in the angular acceleration row, which is the unstable pole at the square root of M plus m times g over M l.

**中文翻译**：设小车质量 M，摆为无质量杆末端质量为 m、杆长 l。倾角 theta 从竖直方向量起，力 F 作用在小车上。小角度时，对整个系统的水平力平衡由牛顿定律得 (M 加 m) x 二阶导加 m l theta 二阶导等于 F。摆的方程给出 x 二阶导加 l theta 二阶导等于 g theta。联立求解，小车加速度为 (F 减 m g theta) 除以 M，角加速度为 ((M 加 m) g theta 减 F) 除以 (M l)。所以状态空间矩阵在角加速度行的 theta 系数为正，对应不稳定极点，位于根号下 (M 加 m) g/(M l)。
**追问 1**：*What are the four states?* Cart position, cart velocity, tilt and tilt rate.
　中文：小车位置、速度、倾角和倾角速率。
**追问 2**：*What changes for a wheel-driven body?* The input is wheel torque rather than force, and the wheel's rotational inertia adds to the effective mass.
　中文：输入是轮力矩而非力，轮的转动惯量会加到等效质量上。
**如果追问到你的项目**：*P1 and P7 use this structure with extra terms: a rolling ball with arms in P1, and a variable-length leg with a wheel in P7.*
　中文：P1 和 P7 用这个结构并加上额外项：P1 是带手臂的滚动球，P7 是带轮的变长腿。
**关键词与易错点**：cart–pole、linearisation、unstable pole、state vector。

### Q35. What assumptions did you make when modelling the ballbot?
**中文题意**：给球形机器人建模时做了哪些假设？
**考察点**：模型局限的诚实表述。
**项目关联**：P1。
**English answer（口语）**：
> I made the usual simplifications to get a clean four-state model. The robot is a rigid body on a rolling ball, with the centre of mass at a fixed height above the ball centre. The ball rolls without slip on the floor, and the omni wheels drive the ball without slip. I treat the two planes, x–z and y–z, as decoupled, so each is an inverted pendulum, and yaw is handled separately. Arm motion is a disturbance, which I compensate with a feedforward. The wheel dynamics are not modelled, since the drivers have their own speed loop. I use the model as a reference to understand weights, stability and saturation, not as an exact plant.

**中文翻译**：为得到清晰的四状态模型，我做了常见的简化。机器人是滚动球上的刚体，质心在球心上方固定高度。球在地面上无滑动滚动，全向轮无滑动地驱动球。我把 x–z 和 y–z 两个平面视为解耦，每个平面是一个倒立摆，yaw 另外处理。手臂运动被视为扰动，用前馈补偿。轮的动力学不建模，因为驱动器自带速度环。我把模型用作理解权重、稳定性和饱和的参考，而不是精确的对象。
**追问 1**：*Where are these assumptions weakest?* Slip when pushed hard, the changing centre of mass with arms, and coupling between planes at large tilt.
　中文：用力推时的滑移、手臂导致的质心变化，以及大倾角时平面之间的耦合。
**追问 2**：*What is the centre of mass height?* About 0.60 metre above the ball centre with the arms down, and moving by 3 to 4 centimetres when the arms are raised forward.
　中文：手臂下垂时约在球心上方 0.60 m，手臂前平举时移动 3–4 cm。
**如果追问到你的项目**：*These are the assumptions behind the CV statement: a linearised four-state inverted pendulum used as a model reference for tuning and saturation diagnosis.*
　中文：这些就是 CV 描述背后的假设：线性化四状态倒立摆，作为整定和饱和诊断的模型参考。
**关键词与易错点**：rigid body、no slip、decoupled planes、feedforward for arms、model reference。

### Q36. What is the rolling constraint and how does rotor inertia affect the effective mass?
**中文题意**：滚动约束是什么？转子惯量如何影响等效质量？
**考察点**：驱动轮的动力学等效。
**项目关联**：P1、P7。
**English answer（口语）**：
> A rolling wheel without slip has the constraint that the speed of the contact point is zero: the translational speed equals the wheel radius times the angular speed. Because of it, the kinetic energy of the rotating wheel adds to the translational kinetic energy: a wheel of mass m and rotational inertia I behaves, for translation, like a mass m plus I over r squared. With a gearbox of ratio N, the motor rotor inertia is reflected to the wheel side as N squared times the rotor inertia, which can be large for a high-ratio gearbox. So when I model a wheeled balancing robot, I include this rolling equivalent mass. It is part of why a small direct-drive wheel and a geared wheel behave differently in the loop.

**中文翻译**：无滑动滚动的轮子有接触点速度为零的约束：平动速度等于轮半径乘角速度。因此旋转轮的动能加到平动动能上：质量为 m、转动惯量为 I 的轮子在平动时等效于质量 m 加 I/r 平方。带传动比 N 的减速箱时，电机转子惯量折算到轮侧为 N 平方乘转子惯量，高传动比时可能很大。所以建模轮式平衡机器人时，我包含这个滚动等效质量。这也是小型直驱轮和带减速的轮在回路中表现不同的原因之一。
**追问 1**：*What is reflected inertia in general?* The load inertia seen by the motor is J load over N squared; the rotor inertia seen by the load is N squared times J rotor.
　中文：电机看到的负载惯量是 J 负载除以 N 平方；负载看到的转子惯量是 N 平方乘 J 转子。
**追问 2**：*Why does the gear ratio help control?* It reduces the sensitivity to load inertia variation, at the cost of friction and backlash.
　中文：降低对负载惯量变化的敏感度，代价是摩擦和间隙。
**如果追问到你的项目**：*In my docs, the rolling equivalent mass is a part of the P1 model, and in P1 the wheel actuator is a geared brushless motor at about 9 to 1.*
　中文：在我的文档里，滚动等效质量是 P1 模型的一部分，P1 的轮电机是约 9:1 减速的无刷电机。
**关键词与易错点**：rolling constraint、equivalent mass、reflected inertia、gear ratio。

### Q37. How does the leg length change the dynamics of the wheel-legged robot?
**中文题意**：腿长如何改变轮腿机器人的动力学？
**考察点**：增益调度的物理依据。
**项目关联**：P7。
**English answer（口语）**：
> The robot behaves like an inverted pendulum on wheels whose pendulum length is related to the leg length. A longer leg raises the centre of mass, lowering the unstable pole frequency of the pendulum, since it goes like the square root of g over the height, so the fall is slower, but the moment of inertia grows and the torque needed to correct a lean changes. The coupling between the leg angle, the body pitch and the wheel motion also changes with the geometry. So A and B in the linear model depend on the leg length, and a gain designed for one length gives less margin at another. That is why I scheduled the LQR gain by a cubic polynomial in the leg length.

**中文翻译**：机器人表现得像轮子上的倒立摆，摆长与腿长相关。腿越长，质心越高，摆的不稳定极点频率越低（它按 g/高度的平方根变化），倾倒更慢，但转动惯量增大，纠正倾斜所需的力矩也变化。腿角、机身俯仰与轮运动之间的耦合也随几何变化。因此线性模型中的 A 和 B 依赖腿长，为某个长度设计的增益在另一长度上裕度更小。这就是我用腿长的三次多项式调度 LQR 增益的原因。
**追问 1**：*Does the longer leg make balance easier?* The pole is slower, which helps, but the actuator torque and the coupling make it not simply easier.
　中文：极点更慢有利，但执行器力矩和耦合使它并非简单地更容易。
**追问 2**：*What is the leg length range?* About 0.10 to 0.30 metres, with the fit and the clamp over this range.
　中文：约 0.10–0.30 m，拟合与钳位都在这个范围。
**如果追问到你的项目**：*I fitted twelve gain entries as cubic polynomials, which gave forty-eight coefficients; with a fixed gain the margin dropped when the leg moved far from the design length.*
　中文：我把 12 个增益元素拟合为三次多项式，共 48 个系数；固定增益在腿长远离设计长度时裕度下降。
**关键词与易错点**：variable leg length、centre of mass height、scheduled gain、coupling。

### Q38. Describe a DC motor model and how torque control is achieved.
**中文题意**：描述直流电机模型，以及如何实现力矩控制。
**考察点**：执行器基础。
**项目关联**：P3、P2、P7（电机）。
**English answer（口语）**：
> For a brushed or brushless motor in the simple DC model, the voltage equals resistance times current plus the back-EMF, which is proportional to the speed. The torque equals the torque constant times the current. The mechanical equation is inertia times acceleration equals the motor torque minus friction minus the load. Since the torque is proportional to current, torque control means current control. A drive does it with a fast current loop, in the kilohertz range, on top of the inverter. A speed loop sits above it, and a position loop above that. So from the controller side, I send a current or torque command, and the drive does the fast inner loops. The torque limit is the current limit.

**中文翻译**：在简单的直流模型中，对有刷或无刷电机，电压等于电阻乘电流加反电动势，反电动势与转速成正比。转矩等于转矩常数乘电流。机械方程是惯量乘加速度等于电机转矩减摩擦减负载。因为转矩与电流成正比，力矩控制就是电流控制。驱动器在逆变器之上用快速电流环（千赫兹量级）实现。速度环在其上，位置环更在其上。所以从控制器一侧，我发送电流或力矩指令，驱动器完成快速内环。力矩限制就是电流限制。
**追问 1**：*What limits the maximum speed?* The supply voltage: the back-EMF rises with speed until it equals the supply.
　中文：电源电压：反电动势随速度上升直到等于电源电压。
**追问 2**：*What is the difference between voltage mode and current mode for the GM6020?* Voltage mode sends a voltage command, so the torque depends on the speed through the back-EMF; current mode regulates the torque directly.
　中文：电压模式发电压指令，力矩通过反电动势依赖速度；电流模式直接调节力矩。
**如果追问到你的项目**：*In P3 the gimbal motors were in voltage mode with commands up to plus or minus 30000, and the chassis and friction wheels used current commands over CAN; in P1 the wheel drivers took speed targets with their own inner current loop at about 1 kilohertz.*
　中文：P3 里云台电机用电压模式（指令至 ±30000），底盘与摩擦轮通过 CAN 发电流指令；P1 里轮驱动器接受速度目标，内部自带约 1 kHz 的电流环。
**关键词与易错点**：back-EMF、torque constant、current loop、voltage mode、current limit。

### Q39. What friction models do you know, and how do they affect control?
**中文题意**：你了解哪些摩擦模型？它们如何影响控制？
**考察点**：非线性摩擦。
**项目关联**：P2（低阻力示教）、P3。
**English answer（口语）**：
> The simplest is viscous friction, proportional to speed. Coulomb friction is a constant force opposite to the motion, with the sign of the velocity. Static friction, stiction, is larger than Coulomb friction, so there is a break-away force. The Stribeck effect is the decrease of friction at low speeds just after break-away. Combined, they explain stick–slip. In control, friction causes steady errors, limit cycles with integral action, and a dead zone near zero speed. Compensation can be a feedforward with Coulomb plus viscous terms, estimated from slow constant-speed motions, or high-gain feedback. For hand-guided teaching, low friction is what makes the leader arm feel light.

**中文翻译**：最简单的是黏性摩擦，与速度成正比。库仑摩擦是与运动方向相反的恒力，符号取决于速度方向。静摩擦大于库仑摩擦，所以有起动力。Stribeck 效应是突破静摩擦后低速时摩擦下降。综合起来可解释粘滑。控制里摩擦导致稳态误差、带积分时的极限环，以及零速附近的死区。补偿可以是由慢速恒速运动估计的库仑加黏性前馈，或高增益反馈。对手把手示教，低摩擦使主臂手感轻。
**追问 1**：*How do you measure it?* Slow constant-speed motions in both directions; the intercept at zero speed is Coulomb friction and the slope is viscous.
　中文：双向慢速恒速运动；零速截距为库仑摩擦，斜率为黏性。
**追问 2**：*Is friction always bad?* No; it also gives damping and helps stability, and in a balancing robot some friction is useful.
　中文：不是；它也提供阻尼并有利稳定，平衡机器人里一定的摩擦是有用的。
**如果追问到你的项目**：*In P2 the follower arm's gains and the leader's gravity compensation ratio were the main tools to deal with friction and payload uncertainty.*
　中文：P2 里从臂增益和主臂重力补偿比例是处理摩擦和负载不确定性的主要手段。
**关键词与易错点**：Coulomb friction、viscous friction、stiction、Stribeck、stick–slip。

### Q40. What is the linear inverted pendulum model, and what is the ZMP?
**中文题意**：什么是线性倒立摆模型？什么是 ZMP？
**考察点**：足式机器人基础知识。
**项目关联**：概念题（P7 轮腿的相邻知识）。
**English answer（口语）**：
> The linear inverted pendulum model treats the robot as a point mass at a constant height z c above the ground, supported by a massless leg on a support point p. The horizontal dynamics are x double dot equals g over z c times x minus p. It is unstable, with the same square root of g over z c time constant, and linear because the height is fixed. The zero moment point, ZMP, is the point on the ground where the horizontal moments of the ground reaction forces vanish. In the model it equals x minus z c over g times x double dot. For stable walking, the ZMP must stay inside the support polygon. I know this as background for legged robots; my wheel-legged project uses a wheeled pendulum model instead.

**中文翻译**：线性倒立摆模型把机器人看作高度恒为 z c 的质点，由无质量的腿支撑在支撑点 p 上。水平动力学为 x 二阶导等于 g/z c 乘 (x 减 p)。它是不稳定的，时间常数同样是根号 g/z c，且因高度固定而线性。零力矩点 ZMP 是地面上使地面反力水平力矩为零的点。在模型中它等于 x 减 (z c/g) 乘 x 二阶导。稳定行走要求 ZMP 保持在支撑多边形内。我是把它作为足式机器人的背景知识来了解；我的轮腿项目用的是轮式摆模型。
**追问 1**：*Capture point?* The point where the robot must step to come to rest: x plus x dot times the square root of z c over g.
　中文：机器人必须迈步才能停下的位置：x 加 x 导数乘根号 z c/g。
**追问 2**：*ZMP versus centre of pressure?* They coincide when the foot is flat and in contact.
　中文：脚平放接触时两者重合。
**如果追问到你的项目**：*It is background knowledge; the closest to my own work is the idea that a controlled quantity lives in a reduced model, like the wheeled pendulum.*
　中文：这是背景知识；与我自己工作最接近的是把被控量放在降阶模型里的思想，如轮式摆。
**关键词与易错点**：LIPM、ZMP、support polygon、capture point。诚实说明。

### Q41. How are contacts and impacts modelled in simulation?
**中文题意**：仿真里如何建模接触与碰撞？
**考察点**：仿真概念。
**项目关联**：P1/P2（MuJoCo）。
**English answer（口语）**：
> There are two broad ways. In penalty or spring–damper models, contact creates a force proportional to the penetration depth and velocity. They are simple and smooth, but need small time steps for stiff contacts and can be inaccurate. In constraint-based models, contact is a unilateral constraint: the normal force is non-negative, the gap is non-negative, and their product is zero, a complementarity condition, with friction limited by a cone. MuJoCo solves a softened version of this as a convex optimisation. Impacts are handled either by tiny time steps, impulse-based resolution with a restitution coefficient, or by soft contacts that absorb the impact over a few steps.

**中文翻译**：有两大类方法。罚函数或弹簧—阻尼模型中，接触产生与穿透深度和速度成正比的力。简单平滑，但刚性接触需要小时间步，可能不精确。基于约束的模型中，接触是单边约束：法向力非负、间隙非负，且二者乘积为零（互补条件），摩擦受锥限制。MuJoCo 把它的软化版本作为凸优化求解。碰撞通过极小时间步、带恢复系数的冲量法，或在几步内吸收冲击的软接触来处理。
**追问 1**：*What is a friction cone?* The set of contact forces for which the tangential force is at most mu times the normal force.
　中文：切向力不超过 mu 乘法向力的接触力集合。
**追问 2**：*Why is there a sim-to-real issue with contact?* Contact parameters such as stiffness and friction are hard to identify and vary.
　中文：刚度和摩擦等接触参数难以辨识且会变化。
**如果追问到你的项目**：*I used MuJoCo in P1 to check signs and trajectories and in P2 as the base for the IK pipeline; I do not claim to have modelled contacts in detail.*
　中文：P1 里我用 MuJoCo 检查符号和轨迹，P2 里用它作为 IK 流水线的基础；我不声称详细建模了接触。
**关键词与易错点**：penalty method、complementarity、friction cone、restitution、soft contact。

### Q42. What is reflected inertia, and why do gear ratios matter for control?
**中文题意**：什么是折算惯量？传动比为何对控制重要？
**考察点**：执行器与负载的匹配。
**项目关联**：P1（9:1）、P3（19.2:1）、P7（约 10:1）。
**English answer（口语）**：
> With a gearbox of ratio N, speed is reduced by N and torque is increased by N. The load inertia seen from the motor is the load inertia divided by N squared, and the rotor inertia seen from the load is N squared times the rotor inertia. A high ratio reduces the impact of load variations on the motor loop, which makes the control easier and gives high torque from a small motor. But it adds friction, backlash and reduces back-drivability, which matters for hand-guided teaching and for sensing the external force from the current. A direct-drive motor has the opposite properties: low friction, high back-drivability, but low torque density.

**中文翻译**：带传动比 N 的减速箱把速度降低 N 倍、力矩增大 N 倍。从电机看到的负载惯量是负载惯量除以 N 平方，从负载看到的转子惯量是 N 平方乘转子惯量。高传动比减小负载变化对电机回路的影响，使控制更容易，并让小电机输出大力矩。但它增加摩擦、间隙并降低反向驱动性，这对手把手示教和从电流感知外力很重要。直驱电机有相反的特性：低摩擦、高反向驱动性，但力矩密度低。
**追问 1**：*What gear ratios did your robots use?* About 9 to 1 in the ballbot wheels, 19.2 to 1 in the Robomaster M3508 for the feeder and chassis, direct drive for the gimbal GM6020 and the friction wheels, and about 10 to 1 in the wheel-legged joint motors.
　中文：球形机器人轮约 9:1，RoboMaster 的 M3508 拨盘与底盘 19.2:1，云台 GM6020 与摩擦轮直驱，轮腿关节电机约 10:1。
**追问 2**：*Why can direct drive be good for teleoperation?* Better back-drivability and less friction.
　中文：反向驱动性更好、摩擦更小。
**如果追问到你的项目**：*Back-drivability is one reason the leader arm felt light with gravity compensation in P2.*
　中文：反向驱动性是 P2 主臂加重力补偿后手感轻的原因之一。
**关键词与易错点**：reflected inertia、gear ratio、back-drivability、direct drive、torque density。

### Q43. How do you simplify a complex robot model for control design?
**中文题意**：如何为控制设计简化复杂的机器人模型？
**考察点**：建模判断力。
**项目关联**：P1、P7。
**English answer（口语）**：
> I ask what the controller needs to capture. Start with the dominant dynamics, the unstable mode in a balancing robot, and treat the rest as disturbances or fast actuators. I reduce the model to its main bodies, with lumped masses and effective lengths, and I make the equations small enough to reason about and to run in real time. I decouple the planes, if the coupling is weak, and treat slow parameter changes, such as leg length, by scheduling. Then I validate: compare the reduced model with the full model or measurements, and check that the controller works on the full simulation and on the real robot. A simple model that is wrong in a known way is better than a complex model I cannot tune.

**中文翻译**：我问控制器需要捕捉什么。从主导动力学开始（平衡机器人的不稳定模态），其余当作扰动或快执行器。把模型化简为主要物体，用集中质量和等效长度，使方程小到可以推理并实时运行。耦合弱时解耦各平面，缓慢变化的参数（如腿长）用调度处理。然后验证：把降阶模型与完整模型或测量比较，并检查控制器在完整仿真和真机上都能工作。一个以已知方式出错的简单模型，好过一个我调不了的复杂模型。
**追问 1**：*How do you know you simplified too much?* The real robot shows behaviour the reduced model cannot reproduce, such as an arm-pose-dependent oscillation.
　中文：真实机器人表现出降阶模型无法复现的行为，如依赖手臂姿态的振荡。
**追问 2**：*Which unmodelled effect hurt you most?* The centre-of-mass shift with arm motion in the ballbot, handled by a feedforward.
　中文：球形机器人手臂运动带来的质心偏移，用前馈处理。
**如果追问到你的项目**：*This is how I treated the arm motion in P1 and the leg length in P7.*
　中文：我就是这样处理 P1 的手臂运动和 P7 的腿长的。
**关键词与易错点**：reduced model、dominant dynamics、lumped parameters、validation。

---

## 5. D 传感器与状态估计（Q44–Q66）

### Q44. Walk me through the Kalman filter equations and the intuition.
**中文题意**：请讲一下卡尔曼滤波的公式与直觉。
**考察点**：KF 核心，必会。
**项目关联**：P5、P7。
**English answer（口语）**：
> The filter has two steps. In the prediction step, I propagate the state with the model: x minus equals A x plus B u, and the covariance grows: P minus equals A P A transpose plus Q. Q is the process noise, which says how much I distrust the model. In the update step, I compare the measurement z with the predicted measurement H x minus; the difference is the innovation. The Kalman gain is K equals P minus H transpose, times H P minus H transpose plus R, inverse. The state is corrected by K times the innovation, and the covariance shrinks to I minus K H times P minus. Intuitively, K balances my trust in the model and in the sensor: a noisy sensor gives a small K.

**中文翻译**：滤波器有两步。预测步，用模型传播状态：x 先验等于 A x 加 B u，协方差增长：P 先验等于 A P A 转置加 Q。Q 是过程噪声，表示我对模型的不信任程度。更新步，把测量 z 与预测测量 H x 先验比较，差值就是新息。卡尔曼增益 K 等于 P 先验 H 转置乘 (H P 先验 H 转置加 R) 的逆。状态用 K 乘新息修正，协方差缩小为 (I 减 K H) 乘 P 先验。直觉上 K 在对模型的信任与对传感器的信任之间取平衡：传感器噪声大则 K 小。
**追问 1**：*What does a larger Q do?* The filter trusts the measurements more and reacts faster, but is noisier.
　中文：滤波器更信任测量，反应更快，但更嘈杂。
**追问 2**：*What is the Joseph form?* A numerically safer covariance update that keeps the matrix symmetric and positive semi-definite.
　中文：数值上更安全的协方差更新，保持对称半正定。
**如果追问到你的项目**：*In P5 the EKF was this filter with a nonlinear car-like model, fusing wheel speed, a yaw rate derived from steering, and the gyro; in P7 it was one-dimensional for body velocity.*
　中文：P5 的 EKF 就是用非线性车式模型的这个滤波器，融合轮速、由转向得到的偏航率和陀螺；P7 是机身速度的一维版本。
**关键词与易错点**：prediction、update、innovation、Kalman gain、process noise。

### Q45. Do a one-step scalar Kalman update by hand.
**中文题意**：手算一步标量卡尔曼更新。
**考察点**：会算。
**项目关联**：P5、P7。
**English answer（口语）**：
> Say my prior estimate is zero with variance four, and I get a measurement of two with variance one. The innovation is two minus zero, which is two. The gain is the prior variance divided by prior variance plus measurement variance, four over five, which is 0.8. The updated state is zero plus 0.8 times two, which is 1.6. The updated variance is one minus 0.8 times four, which is 0.8. So the estimate moves most of the way toward the measurement because the sensor is more precise than my prior, and the variance, 0.8, is smaller than both four and one. It equals the inverse-variance fusion result.

**中文翻译**：假设先验估计为零、方差 4，得到测量值 2、方差 1。新息为 2 减 0，即 2。增益为先验方差除以（先验方差加测量方差），4/5，即 0.8。更新后的状态为 0 加 0.8 乘 2，即 1.6。更新后的方差为（1 减 0.8）乘 4，即 0.8。所以估计向测量移动了大部分，因为传感器比我的先验更精确；方差 0.8 比 4 和 1 都小。它等于逆方差融合的结果。
**追问 1**：*What if the measurement variance is 100?* The gain is 4 over 104, about 0.038, so the estimate barely moves.
　中文：增益为 4/104，约 0.038，估计几乎不动。
**追问 2**：*What if the prior variance is zero?* The gain is zero; the filter ignores the measurement, which is a typical mistake when P is initialised too small.
　中文：增益为零，滤波器忽略测量；P 初始化过小时的典型错误。
**如果追问到你的项目**：*In P7 the speed filter's process noise of about 0.001 and measurement noise of about 0.01 make the gain small, so the wheel speed corrects the acceleration integral slowly.*
　中文：P7 速度滤波器的过程噪声约 0.001、测量噪声约 0.01 使增益较小，所以轮速缓慢地修正加速度积分。
**关键词与易错点**：innovation、gain、variance update、inverse-variance fusion。

### Q46. How do you tune Q and R, and how do you know if the filter is consistent?
**中文题意**：怎么整定 Q 和 R？怎么知道滤波器是否一致？
**考察点**：滤波器整定实践。
**项目关联**：P5。
**English answer（口语）**：
> I get R from the sensor: its datasheet or the variance of a static recording. Q is harder, since it represents model error and unmodelled motion, so I start from physical reasoning, like the maximum acceleration I expect, and tune by looking at the results. The best check is the innovation: if the filter is consistent, the innovations should be zero-mean, white, and with the covariance that the filter predicts. The normalised innovation squared averaged over time should be close to the measurement dimension. If the innovations are too large, my covariances are too optimistic; if they are tiny, too pessimistic. A filter that lags behind needs larger Q; a filter that is jittery needs smaller Q or larger R.

**中文翻译**：R 来自传感器：数据手册或静态记录的方差。Q 更难，因为它代表模型误差和未建模的运动，所以我从物理推理（比如预期的最大加速度）出发，再根据结果调整。最好的检验是新息：若滤波器一致，新息应零均值、白，并具有滤波器预测的协方差。归一化新息平方的时间平均应接近测量维数。新息太大说明我的协方差过于乐观；新息很小则过于悲观。滤波器滞后需要更大的 Q；抖动则需要更小的 Q 或更大的 R。
**追问 1**：*Where did your R values come from?* The wheel speed standard deviation of about 0.05 metres per second and gyro of about 0.01 radians per second came from static recordings and design reasoning.
　中文：轮速标准差约 0.05 m/s、陀螺约 0.01 rad/s，来自静态记录和设计推理。
**追问 2**：*What is the symptom of an overconfident filter?* It rejects good measurements or reacts slowly to a real change.
　中文：拒绝好的测量，或对真实变化反应迟缓。
**如果追问到你的项目**：*In P5, I set diagonal process noise entries between ten to the minus three and ten to the minus two in the robot_localization config.*
　中文：P5 里我在 robot_localization 配置中把过程噪声对角元设在 10 的负 3 次方到负 2 次方之间。
**关键词与易错点**：innovation、consistency、NIS、process noise、measurement noise。

### Q47. What is the extended Kalman filter, and how is it different from the linear one?
**中文题意**：扩展卡尔曼滤波是什么？与线性版本有何区别？
**考察点**：EKF 的假设与失效。
**项目关联**：P5（核心）。
**English answer（口语）**：
> The EKF handles nonlinear models x next equals f of x and u, and z equals h of x. It propagates the mean through the nonlinear function, but uses the Jacobians F and H, evaluated at the current estimate, in the covariance equations. So it is a first-order approximation. It works well when the nonlinearity is mild over the size of the uncertainty. It can fail when the initial estimate is poor, the nonlinearity is strong, or the noise is large, giving divergence or an overconfident covariance. In my car, the motion model contains cosine and sine of heading, so the Jacobian depends on yaw; a big yaw error then spreads into position errors in the wrong direction.

**中文翻译**：EKF 处理非线性模型 x 下一步等于 f(x, u)，z 等于 h(x)。它通过非线性函数传播均值，但在协方差方程中使用在当前估计处求值的雅可比 F 和 H，所以是一阶近似。当不确定性范围内非线性较弱时效果良好。初始估计差、非线性强或噪声大时可能失效，造成发散或协方差过于乐观。我的车的运动模型含航向的 cos 和 sin，所以雅可比依赖 yaw；大的 yaw 误差会把位置误差扩散到错误方向。
**追问 1**：*What are alternatives?* The unscented Kalman filter, iterated EKF, particle filters, or smoothing and optimisation.
　中文：无迹卡尔曼滤波、迭代 EKF、粒子滤波，或平滑与优化。
**追问 2**：*What is the error-state EKF?* It estimates the small error around a nominal state, which suits attitude estimation.
　中文：估计名义状态周围的小误差，适合姿态估计。
**如果追问到你的项目**：*In P5, I used robot_localization's ekf_node at 30 hertz in two-dimensional mode.*
　中文：P5 里我用 robot_localization 的 ekf_node，30 Hz，二维模式。
**关键词与易错点**：EKF、Jacobian、first-order approximation、divergence。

### Q48. Compare EKF, UKF and particle filters.
**中文题意**：比较 EKF、UKF 与粒子滤波。
**考察点**：滤波器选型。
**项目关联**：P5（EKF 与粒子滤波）。
**English answer（口语）**：
> The EKF linearises the model with Jacobians; it is cheap and widely used but it can be inaccurate for strong nonlinearities and needs derivatives. The UKF propagates a small deterministic set of sigma points through the nonlinear function and recovers the mean and covariance; it captures the nonlinearity to second order, needs no Jacobians, and costs a little more. Both assume a unimodal Gaussian belief. A particle filter represents the belief with many weighted samples, so it can handle strong nonlinearity and multi-modal beliefs, at a high computational cost that grows with the dimension. So for continuous local state tracking, I use EKF or UKF; for global localisation with ambiguity, a particle filter.

**中文翻译**：EKF 用雅可比线性化模型；便宜且应用广泛，但强非线性时可能不准，且需要导数。UKF 把一小组确定性 sigma 点通过非线性函数传播，再恢复均值和协方差；能捕捉到二阶非线性，不需要雅可比，计算量略大。两者都假设单峰高斯信念。粒子滤波用许多加权样本表示信念，可处理强非线性和多峰信念，但计算代价高且随维数增长。所以对连续的局部状态跟踪，我用 EKF 或 UKF；对有歧义的全局定位，用粒子滤波。
**追问 1**：*Why is the particle filter bad in high dimensions?* The number of samples needed grows exponentially with dimension.
　中文：所需样本数随维数指数增长。
**追问 2**：*How many sigma points does a UKF use?* Two n plus one for n states.
　中文：n 个状态用 2n+1 个。
**如果追问到你的项目**：*In P5, I combined an EKF for the continuous odometry-level state and AMCL, a particle filter, for global localisation on the map.*
　中文：P5 里我把用于连续里程计级状态的 EKF 和用于地图上全局定位的粒子滤波 AMCL 结合起来。
**关键词与易错点**：EKF、UKF、particle filter、sigma points、multi-modal。我没做过 UKF。

### Q49. Explain how AMCL works, and its key parameters.
**中文题意**：解释 AMCL 的工作原理与关键参数。
**考察点**：蒙特卡洛定位。
**项目关联**：P5（核心）。
**English answer（口语）**：
> AMCL, adaptive Monte Carlo localisation, is a particle filter on a known occupancy-grid map. Each particle is a pose hypothesis. In the prediction step, I move each particle with the odometry plus noise, using the motion model. In the update step, each particle is weighted by how well the laser scan fits the map from that pose, in my case the likelihood field model. Then I resample: particles with high weights are duplicated, low ones are dropped. The adaptive part changes the number of particles with the uncertainty, between 500 and 3000 in my setup, using the KLD sampling. I update only after the robot moves about 0.15 metre or turns 0.2 radian, and use about 60 beams per scan.

**中文翻译**：AMCL（自适应蒙特卡洛定位）是已知占据栅格地图上的粒子滤波。每个粒子是一个位姿假设。预测步，用运动模型把每个粒子按里程计加噪声移动。更新步，按激光扫描在该位姿下与地图的吻合程度给每个粒子加权，我用的是似然场模型。然后重采样：高权重粒子被复制，低权重被丢弃。自适应部分是用 KLD 采样根据不确定性改变粒子数，我的设置是 500 到 3000。机器人移动约 0.15 m 或转 0.2 rad 之后才更新，每次扫描用约 60 条射线。
**追问 1**：*What is the likelihood field model?* It scores each beam endpoint by its distance to the nearest obstacle in the map, so it is smooth and cheap compared with ray casting.
　中文：按每条射线终点到地图中最近障碍的距离打分，与光线投射相比平滑且便宜。
**追问 2**：*What are the motion-model noise parameters?* The alphas for rotation and translation noise; I used about 0.15, 0.10, 0.20 and 0.10.
　中文：旋转和平移噪声的 alpha 参数；我用约 0.15、0.10、0.20、0.10。
**如果追问到你的项目**：*This is the AMCL part of my CV statement, with the nav2_amcl package, ROS2 Humble.*
　中文：这是 CV 里 AMCL 的部分，使用 nav2_amcl 包，ROS2 Humble。
**关键词与易错点**：particle filter、likelihood field、resampling、KLD sampling、update thresholds。

### Q50. How do the EKF and AMCL work together, and who publishes which transform?
**中文题意**：EKF 与 AMCL 如何协同？谁发布哪个变换？
**考察点**：定位架构。
**项目关联**：P5（核心）。
**English answer（口语）**：
> They solve two different problems and are linked by the transform tree. The EKF fuses wheel odometry, yaw rate and the gyro into a smooth, continuous estimate of the pose in the odom frame, and publishes odom to base_link. It drifts, but it never jumps. AMCL localises the robot in the map using the laser, and publishes map to odom, which is the correction that makes the odom estimate consistent with the map. The product, map to odom to base_link, gives the global pose. The controller uses the continuous odom pose, and the planner uses the global pose. When AMCL corrects, the jump goes into map to odom, not into the control loop.

**中文翻译**：它们解决两个不同的问题，通过变换树连接。EKF 把轮里程计、偏航率和陀螺融合成 odom 系下平滑连续的位姿估计，并发布 odom 到 base_link。它会漂移，但不会跳变。AMCL 用激光在地图中定位机器人，并发布 map 到 odom，即使 odom 估计与地图一致的修正。乘积 map 到 odom 到 base_link 给出全局位姿。控制器用连续的 odom 位姿，规划器用全局位姿。AMCL 修正时，跳变进入 map 到 odom，而不是控制回路。
**追问 1**：*What did you do about pose jumps?* I limited the rate of the map-to-odom change to about 20 percent of the vehicle speed and required three consistent updates with a small covariance for jumps above 0.5 metres or 10 degrees.
　中文：我把 map 到 odom 的变化率限制在车速的约 20%，超过 0.5 m 或 10° 的跳变需三次一致且协方差小的更新。
**追问 2**：*What was the result?* Adjacent-cycle steps were generally below about 5 centimetres.
　中文：相邻周期阶跃一般小于约 5 cm。
**如果追问到你的项目**：*That is the continuous vehicle state in my CV: fused IMU, wheel encoders and 2D LiDAR, handling drift and pose jumps from timing, frame and direction inconsistencies.*
　中文：这就是 CV 里的连续车辆状态：融合 IMU、轮编码器和 2D 激光，处理由时间、坐标系和方向不一致引起的漂移和位姿跳变。
**关键词与易错点**：odom to base_link、map to odom、continuous state、jump handling。

### Q51. Explain the complementary filter for attitude estimation.
**中文题意**：解释用于姿态估计的互补滤波。
**考察点**：IMU 融合的核心。
**项目关联**：P1、P3、P7。
**English answer（口语）**：
> A gyroscope is accurate over short times but drifts, because integrating it accumulates the bias. An accelerometer gives the gravity direction, so it provides an absolute tilt with no drift, but it is noisy and corrupted by linear accelerations. The complementary filter combines them by frequency: it uses the gyro at high frequencies and the accelerometer at low frequencies. In discrete form, angle equals alpha times the previous angle plus gyro rate times dt, plus one minus alpha times the accelerometer angle. With alpha of 0.98 and dt of 10 milliseconds, the time constant is alpha dt over one minus alpha, 0.49 seconds, so the crossover is at about 0.3 hertz. It is simple, cheap and robust.

**中文翻译**：陀螺仪在短时间内准确，但会漂移，因为积分累积偏置。加速度计给出重力方向，因此提供无漂移的绝对倾角，但噪声大且受线加速度干扰。互补滤波按频率组合两者：高频用陀螺，低频用加速度计。离散形式：角度等于 alpha 乘（上一角度加陀螺角速率乘 dt），再加（1 减 alpha）乘加速度计角度。alpha 取 0.98、dt 取 10 ms 时，时间常数为 alpha dt/(1 减 alpha)，即 0.49 秒，交叉频率约 0.3 Hz。它简单、便宜、鲁棒。
**追问 1**：*How do you choose alpha?* A larger alpha trusts the gyro longer and removes more accelerometer noise but corrects drift slowly; it sets the time constant.
　中文：alpha 越大越久信任陀螺、滤除更多加速度计噪声，但漂移修正更慢；它决定时间常数。
**追问 2**：*Why not just a Kalman filter?* A KF with constant gains reduces to a complementary filter; the complementary filter has fewer parameters and is easier to tune.
　中文：增益恒定的 KF 退化为互补滤波；互补滤波参数更少，更易整定。
**如果追问到你的项目**：*I used Mahony-type complementary filters: in P3 with Kp 0.5 and Ki 0, in P7 with Kp about 2 and Ki about 0.005, and in P1 an AHRS module with on-board attitude solution.*
　中文：我用的是 Mahony 型互补滤波：P3 里 Kp 0.5、Ki 0，P7 里 Kp 约 2、Ki 约 0.005，P1 用带板载姿态解算的 AHRS 模块。
**关键词与易错点**：gyro、accelerometer、complementary filter、crossover frequency、time constant。

### Q52. How does the Mahony filter work?
**中文题意**：Mahony 滤波如何工作？
**考察点**：互补滤波的非线性形式。
**项目关联**：P3、P7。
**English answer（口语）**：
> The Mahony filter works on the rotation group. It predicts the gravity direction in the body frame from the current attitude estimate, and compares it with the measured, normalised accelerometer direction. The cross product of the two is a rotation error vector. A proportional-integral correction of this error is added to the gyro rate, and the sum is integrated into the attitude. The proportional gain Kp sets how fast the accelerometer corrects the attitude; the integral gain Ki estimates the gyro bias. A small Kp trusts the gyro; a large Kp follows the accelerometer, and noise or linear acceleration pass in. In P3 I used Kp equal to 0.5 and Ki equal to zero, because the bias was calibrated at start-up.

**中文翻译**：Mahony 滤波工作在旋转群上。它由当前姿态估计预测机体系下的重力方向，并与测得的归一化加速度计方向比较。两者的叉积是旋转误差向量。对该误差做比例—积分修正，加到陀螺角速率上，再将和积分成姿态。比例增益 Kp 决定加速度计修正姿态的速度；积分增益 Ki 估计陀螺偏置。Kp 小则信任陀螺；Kp 大则跟随加速度计，噪声和线加速度会进入。P3 里我用 Kp 为 0.5、Ki 为零，因为偏置在启动时已标定。
**追问 1**：*Does the accelerometer correct yaw?* No; gravity has no information about yaw, so yaw is only gyro-integrated unless a magnetometer or another reference is used.
　中文：不；重力不含 yaw 信息，所以 yaw 只靠陀螺积分，除非用磁力计或其他参考。
**追问 2**：*What is the weakness during acceleration?* The accelerometer reads gravity plus linear acceleration, so Kp should be small, or the correction should be gated by the acceleration magnitude.
　中文：加速度计读到重力加线加速度，所以 Kp 应小，或按加速度幅值对修正做门限。
**如果追问到你的项目**：*In P7 the Kp was about 2 and Ki about 0.005, a more aggressive correction because the robot's body pitch must be precise for balance.*
　中文：P7 里 Kp 约 2、Ki 约 0.005，修正更积极，因为机身俯仰对平衡必须精确。
**关键词与易错点**：Mahony、cross product error、PI correction、gyro bias、yaw unobservable。

### Q53. Compare Mahony, Madgwick and Kalman-based AHRS.
**中文题意**：比较 Mahony、Madgwick 与基于卡尔曼的 AHRS。
**考察点**：算法取舍。
**项目关联**：概念题；P3、P7。
**English answer（口语）**：
> Mahony and Madgwick are both lightweight complementary-type filters that run well on a microcontroller. Mahony uses a PI feedback to estimate the gyro bias; Madgwick uses a gradient-descent step on the accelerometer error and has a single gain, but it does not explicitly estimate the bias. A Kalman-based AHRS, especially an error-state EKF, models the noise statistics and the bias as states, and can give better accuracy, adapt to sensor quality and give an uncertainty, but it is heavier to implement and tune. For a balancing robot where the main need is robust, low-latency roll and pitch, a Mahony filter is a sensible choice.

**中文翻译**：Mahony 和 Madgwick 都是轻量的互补型滤波器，在微控制器上运行良好。Mahony 用 PI 反馈估计陀螺偏置；Madgwick 对加速度计误差做梯度下降步，只有一个增益，但不显式估计偏置。基于卡尔曼的 AHRS（尤其是误差状态 EKF）把噪声统计和偏置建模为状态，能给出更好的精度、适应传感器质量并给出不确定度，但实现和整定更重。对主要需要稳健、低延迟 roll 和 pitch 的平衡机器人，Mahony 滤波是合理的选择。
**追问 1**：*What does a Kalman AHRS give that Mahony doesn't?* An uncertainty estimate and a principled way to weigh sensors.
　中文：不确定度估计和对传感器加权的原则性方法。
**追问 2**：*What is the cost of heavier filters on an MCU?* CPU time and the risk of tuning mistakes.
　中文：CPU 时间以及整定出错的风险。
**如果追问到你的项目**：*I used Mahony in P3 and P7 since it fits the STM32 loop budget; I know the Kalman variant at the concept level.*
　中文：P3 和 P7 里我用 Mahony，因为符合 STM32 的循环预算；卡尔曼变体我是概念层面的了解。
**关键词与易错点**：Mahony、Madgwick、error-state EKF、bias estimation、microcontroller。

### Q54. What errors do an IMU's gyroscope and accelerometer have?
**中文题意**：IMU 的陀螺仪和加速度计有哪些误差？
**考察点**：传感器误差模型。
**项目关联**：P1、P3、P7。
**English answer（口语）**：
> Both have bias, which is a constant offset, and often changes with temperature and time. They have scale-factor errors, where the output gain is not exactly one, and axis misalignment, where the sensor axes are not perfectly orthogonal or aligned with the body. They have noise, usually modelled as white noise plus a slowly varying bias random walk. The gyroscope's noise integrates into angle random walk, and its bias gives a linear drift in angle. The accelerometer is also sensitive to vibration, and to linear acceleration, which is not an error of the sensor but of the measurement model. I deal with them by calibrating the bias at start-up, keeping the temperature stable, and fusing the two sensors.

**中文翻译**：两者都有偏置，即常值偏移，并常随温度和时间变化。有比例因子误差（输出增益不精确为 1）和轴失准（传感器轴不完全正交或与机体对齐）。有噪声，通常建模为白噪声加缓变的偏置随机游走。陀螺噪声积分成角度随机游走，偏置使角度线性漂移。加速度计对振动和线加速度也敏感，后者不是传感器误差，而是测量模型的问题。我的处理办法是启动时标定偏置、保持温度稳定，并融合两种传感器。
**追问 1**：*Why does temperature matter?* Bias varies with temperature, so a heater or a warm-up and a temperature compensation reduce the drift.
　中文：偏置随温度变化，加热器、预热和温度补偿能减小漂移。
**追问 2**：*How do you calibrate the accelerometer?* Place it in several known orientations and fit the bias, scale and misalignment.
　中文：放在几个已知朝向，拟合偏置、比例和失准。
**如果追问到你的项目**：*In P3, the BMI088 gyro had ±2000 degrees per second range, the heater held the temperature at 40 degrees Celsius, and bias was calibrated from 2000 samples at power-up.*
　中文：P3 里 BMI088 陀螺量程 ±2000 °/s，加热恒温 40 °C，上电时用 2000 个样本标定偏置。
**关键词与易错点**：bias、scale factor、misalignment、white noise、random walk、temperature。

### Q55. How do you calibrate the gyro bias at start-up, and what can go wrong?
**中文题意**：启动时如何标定陀螺偏置？可能出什么问题？
**考察点**：实践细节。
**项目关联**：P1（200 帧）、P3（2000 样本）。
**English answer（口语）**：
> At start-up I hold the robot still, collect a few seconds of data, and average the gyro readings: the mean is the bias, and the spread tells the noise level. I subtract the bias afterward. For tilt, in the ballbot I average 200 frames, about 2 seconds, of roll and pitch while the robot is steadied, and use the mean as the zero offset. What can go wrong: if the robot is not still, the bias is wrong; if the sample is too short, noise leaks into the mean; if I do not repeat on every re-enable, the bias can be stale. In one of my debug cases, a stale offset made the robot creep slowly at rest, with a pitch mean of about 0.9 degrees.

**中文翻译**：启动时保持机器人静止，采集几秒数据，对陀螺读数取平均：均值是偏置，离散程度反映噪声水平，之后减去偏置。对倾角，球形机器人在被扶稳时平均 200 帧（约 2 秒）的 roll 和 pitch，以均值为零位偏置。可能出的问题：机器人不静止则偏置错误；样本太短则噪声泄漏进均值；每次重新使能不重做则偏置可能过期。我的一个调试案例里，过期的偏置使机器人静止时缓慢爬行，pitch 均值约 0.9 度。
**追问 1**：*Why 200 frames?* At 100 hertz, 2 seconds reduce the noise of the mean by a factor of about 14.
　中文：100 Hz 下 2 秒使均值噪声降低约 14 倍。
**追问 2**：*Do you re-calibrate every time?* Yes, at every re-enable, together with clearing integrators and filters.
　中文：是，每次重新使能时重做，同时清除积分和滤波。
**如果追问到你的项目**：*In P3 I averaged 2000 samples at power-up with the IMU heated to a stable temperature.*
　中文：P3 里我在 IMU 加热到稳定温度后，上电时平均 2000 个样本。
**关键词与易错点**：static calibration、averaging、stale offset、re-enable。

### Q56. Why does yaw drift, and how can you reduce it?
**中文题意**：yaw 为什么会漂移？怎样减小？
**考察点**：可观性。
**项目关联**：P3（约 1 °/min）。
**English answer（口语）**：
> Roll and pitch can be corrected by the gravity direction, but gravity gives no information about rotation around the vertical axis. So yaw is determined only by integrating the gyro, and any bias or noise accumulates without correction. The ways to reduce it are: calibrate the bias carefully and keep the temperature stable; use a magnetometer to give an absolute heading, which works outdoors but is easily disturbed by motors, cables and steel indoors; use another heading source, like wheel odometry, visual features or a LiDAR map match; or accept the drift if only the relative heading matters over short periods. In my RoboMaster robot the magnetometer was not enabled, and the drift was about one degree per minute at rest.

**中文翻译**：roll 和 pitch 可以由重力方向修正，但重力不提供绕垂直轴旋转的信息。所以 yaw 只能靠陀螺积分确定，任何偏置或噪声都会在无修正下累积。减小的办法：仔细标定偏置并保持温度稳定；用磁力计给出绝对航向，室外可行，但室内易受电机、线缆和钢材干扰；使用其他航向来源，如轮里程计、视觉特征或激光地图匹配；或者在只需短期相对航向时接受漂移。我的 RoboMaster 机器人没启用磁力计，静置漂移约每分钟 1 度。
**追问 1**：*What is the drift rate in degrees per second?* About 0.017 degrees per second.
　中文：约 0.017 度/秒。
**追问 2**：*What does the vision system use for heading?* The target error angle relative to the gimbal, so absolute yaw drift matters less.
　中文：用相对云台的目标误差角，所以绝对 yaw 漂移影响较小。
**如果追问到你的项目**：*In P5 the yaw reference came from the map through AMCL, which bounds the heading drift of the dead-reckoning chain.*
　中文：P5 里 yaw 参考来自 AMCL 的地图匹配，约束了航迹推算链的航向漂移。
**关键词与易错点**：yaw unobservable、absolute reference、magnetometer、bias、dead reckoning。

### Q57. Why can't you use the accelerometer alone for tilt on a moving robot?
**中文题意**：为什么运动中的机器人不能只用加速度计测倾角？
**考察点**：加速度计的局限。
**项目关联**：P1、P7。
**English answer（口语）**：
> The accelerometer measures specific force, which is gravity plus the acceleration of the body. When the robot accelerates, the measured vector no longer points along gravity, and the tilt computed from it is wrong. For a balancing robot, accelerating is exactly what it does to keep balance, so the accelerometer tilt would be biased in precisely the moments that matter. Vibration adds noise too. The gyro does not have this problem in the short term. That is why the filter trusts the gyro at high frequency and uses the accelerometer only as a slow correction; the time constant should be longer than the dynamics of the acceleration.

**中文翻译**：加速度计测量的是比力，即重力加上机体的加速度。机器人加速时，测得的向量不再指向重力方向，由此算出的倾角是错的。平衡机器人为了保持平衡恰恰要加速，所以加速度计倾角恰在关键时刻有偏。振动也增加噪声。陀螺在短期内没有这个问题。因此滤波器在高频信任陀螺，只把加速度计作为缓慢的修正；时间常数应长于加速度变化的动态。
**追问 1**：*How can you gate the accelerometer?* Reduce its weight when the measured magnitude differs from one g by more than a threshold.
　中文：测得幅值偏离 1 g 超过阈值时降低其权重。
**追问 2**：*Is there a model-based way?* Include the known commanded acceleration in the model.
　中文：把已知的指令加速度放进模型。
**如果追问到你的项目**：*In my balance robots the gyro rate is the primary source for fast loops, and the accelerometer only corrects slowly through the AHRS.*
　中文：我的平衡机器人里，陀螺角速率是快环的主要来源，加速度计仅通过 AHRS 缓慢修正。
**关键词与易错点**：specific force、linear acceleration、gating、time constant。

### Q58. What are the error sources of wheel odometry, and how do you calibrate it?
**中文题意**：轮式里程计的误差来源有哪些？如何标定？
**考察点**：里程计误差模型。
**项目关联**：P2、P5。
**English answer（口语）**：
> Systematic errors are repeatable: the wheel radius is not what I assumed, the track width is different, or the encoder resolution is slightly off. They cause a scale error in distance and a bias in turning. Non-systematic errors are random: wheel slip, uneven floors, and quantisation. Heading errors matter more than distance errors, since a heading error spreads into position error with distance. For calibration I drive straight for a known distance, like 5 metres, to calibrate the wheel radius, and rotate in place for a number of full turns to calibrate the track width, repeating each three times. After calibration the 5 metre straight run had about 2 percent error in my differential base.

**中文翻译**：系统误差是可重复的：轮半径不是我假设的值、轮距不同，或编码器分辨率略有偏差。它们造成距离的比例误差和转向的偏置。非系统误差是随机的：轮子打滑、地面不平和量化。航向误差比距离误差更重要，因为航向误差会随距离扩散为位置误差。标定时，我直行已知距离（如 5 m）标定轮半径，原地旋转若干整圈标定轮距，各重复三次。标定之后，差速底盘 5 m 直行误差约 2%。
**追问 1**：*What was the heading error?* About 30 degrees after ten in-place turns.
　中文：原地转十圈后约 30 度。
**追问 2**：*How do you reduce the drift further?* Fuse a gyro for heading and use a map-based localisation to bound the position.
　中文：用陀螺融合航向，并用基于地图的定位约束位置。
**如果追问到你的项目**：*In P2 odometry was wheel integration at 200 hertz on the board, published at 50 hertz, with a covariance that grows with distance and rotation.*
　中文：P2 里里程计是板端 200 Hz 的轮速积分，50 Hz 发布，协方差随距离与转角增长。
**关键词与易错点**：systematic error、wheel radius、track width、slip、calibration。

### Q59. Loosely coupled versus tightly coupled sensor fusion: what is the difference?
**中文题意**：松耦合与紧耦合传感器融合有何区别？
**考察点**：融合架构。
**项目关联**：P5（松耦合）。
**English answer（口语）**：
> In a loosely coupled design, each sensor subsystem first produces its own estimate, such as a pose from LiDAR matching or a velocity from wheel encoders, and then a filter fuses these estimates. It is modular, easy to debug and to swap sensors, but it loses information, since the correlations in the raw data and the internal uncertainties are partly hidden. In a tightly coupled design, the raw measurements, like feature observations or pseudo-ranges, go directly into one estimator. This is more accurate and more robust when individual sensors are degraded, but it is more complex and harder to debug. My navigation stack was loosely coupled: the EKF fused velocities and yaw rate, and AMCL used the laser on the map.

**中文翻译**：松耦合设计中，每个传感器子系统先产生自己的估计（如激光匹配的位姿或编码器的速度），再由滤波器融合这些估计。模块化，易调试，易替换传感器，但会丢失信息，因为原始数据中的相关性和内部不确定性部分被隐藏。紧耦合设计中，原始测量（如特征观测或伪距）直接进入同一个估计器。更准确，在单个传感器退化时更鲁棒，但更复杂，更难调试。我的导航栈是松耦合的：EKF 融合速度和偏航率，AMCL 在地图上使用激光。
**追问 1**：*Where is tight coupling used?* GNSS–INS and visual–inertial odometry, for example.
　中文：如 GNSS-INS 和视觉惯性里程计。
**追问 2**：*Why choose loose coupling for a student project?* Simplicity, reuse of tested packages, and easier fault isolation.
　中文：简单、复用经过测试的包、更容易隔离故障。
**如果追问到你的项目**：*I have not built a tightly coupled system; the loosely coupled design let me isolate timing and frame problems in each stage.*
　中文：我没有构建过紧耦合系统；松耦合让我能够在每个阶段隔离时间和坐标系问题。
**关键词与易错点**：loosely coupled、tightly coupled、modularity、information loss。

### Q60. Why do time synchronisation and timestamps matter in sensor fusion?
**中文题意**：为什么时间同步和时间戳在传感器融合中重要？
**考察点**：数据一致性。
**项目关联**：P5、P6（核心）。
**English answer（口语）**：
> A fusion algorithm assumes the measurements describe the same moment, or at least that I know their true times. A fixed offset of 60 milliseconds at a turning rate of one radian per second gives about 3.4 degrees of heading error. Common problems: stamping at arrival instead of at capture, different clocks on different machines, and wall clocks jumping when synchronised. The good practice: stamp as close to the sensor as possible; use a monotonic clock inside a device; synchronise clocks across machines with NTP or chrony, and know the achievable accuracy, which is several to ten-odd milliseconds with NTP on a LAN; and interpolate transforms to the measurement time. I only use cross-device time for logging and rough alignment, not for control.

**中文翻译**：融合算法假设测量描述同一时刻，或至少知道其真实时间。1 rad/s 转速下 60 ms 的固定偏移会造成约 3.4 度航向误差。常见问题：以到达时刻而非采集时刻打戳、不同机器时钟不同，以及同步时墙上时钟跳变。良好做法：尽可能靠近传感器打戳；设备内部用单调时钟；跨机器用 NTP 或 chrony 同步，并了解可达精度（局域网 NTP 为几毫秒到十几毫秒）；并把变换插值到测量时刻。我只把跨设备时间用于日志和粗对齐，不用于控制。
**追问 1**：*How did you find the laser offset in P5?* Cross-correlating the angular rate from the IMU and from scan matching showed a lag of about 60 ms.
　中文：对 IMU 和扫描匹配的角速率做互相关，发现约 60 ms 滞后。
**追问 2**：*Why not PTP?* More accurate but needs hardware support; for the design here NTP was enough, and control loops do not rely on cross-device time.
　中文：更准确但需要硬件支持；此设计里 NTP 足够，控制回路不依赖跨设备时间。
**如果追问到你的项目**：*In P6 this is a design principle: stamp with a monotonic clock, keep a wall stamp for logs, and design the state manager so a clock jump marks data suspect rather than silently using it. These are design values, not company measurements.*
　中文：P6 里这是设计原则：用单调时钟打戳，保留墙上时间戳用于日志，设计状态管理器使时钟跳变将数据标为可疑而不是悄悄使用。这些是设计取值，不是公司实测。
**关键词与易错点**：timestamp、monotonic clock、NTP、chrony、capture time。

### Q61. What data validity checks do you apply before fusing sensor data?
**中文题意**：融合传感器数据之前做哪些有效性检查？
**考察点**：数据治理。
**项目关联**：P6（核心）、P5。
**English answer（口语）**：
> I check several things at the entrance of the state manager. Is the value finite, not NaN or infinity? Is it inside the physical range, such as a wheel speed within plus or minus 3 metres per second? Is the frame declared? Is the timestamp monotonic? Is the data fresh, with an age below about three times the sensor period? Is the signal frozen, for example five identical frames in a row? Does the rate of change look physically possible? Each data item gets a validity level, valid, stale, suspect or invalid, and downstream modules decide how to degrade. Invalid data must not be used silently; it is better to disable a feature than to feed it bad numbers.

**中文翻译**：我在状态管理器入口检查几件事：值是否有限（非 NaN 或无穷）？是否在物理范围内（如轮速在 ±3 m/s 内）？坐标系是否声明？时间戳是否单调？数据是否新鲜（年龄低于传感器周期的约 3 倍）？信号是否冻结（如连续 5 帧完全相同）？变化率是否物理上可能？每个数据项获得一个有效性等级：有效、过期、可疑或无效，下游模块决定如何降级。无效数据不能被悄悄使用；关闭一个功能好过给它喂坏数据。
**追问 1**：*Why three times the period?* It allows a missed sample or jitter without declaring a failure, while still being quick to detect real loss.
　中文：允许丢一帧或抖动而不判定失效，同时仍能快速检测真实丢失。
**追问 2**：*What do you do with stale data?* Hold the last value briefly for non-critical uses, and switch to a safe behaviour or a lower mode for critical ones.
　中文：非关键用途短暂保持最后一个值，关键用途切换到安全行为或更低模式。
**如果追问到你的项目**：*This is my P6 state-management work, standardising timestamps, frames and validity so the motion and task layers get a consistent, trusted state; the thresholds are design values.*
　中文：这是我 P6 的状态管理工作：统一时间戳、坐标系和有效性，使运动和任务层得到一致可信的状态；阈值是设计取值。
**关键词与易错点**：validity、freshness、frozen detection、range check、degradation。

### Q62. How did you configure robot_localization, and what were the pitfalls?
**中文题意**：你如何配置 robot_localization？有哪些坑？
**考察点**：实际配置经验。
**项目关联**：P5。
**English answer（口语）**：
> I used the ekf_node in two-dimensional mode at 30 hertz. The inputs were wheel-derived speed, the yaw rate computed from speed and steering angle as v tan delta over L, and the gyro z rate. For each input I choose which variables to fuse, so I do not fuse a quantity twice from correlated sources, and I give each input a realistic covariance. Wheel speed noise was about 0.05 metres per second and the gyro about 0.01 radians per second. Pitfalls I met: the sign of the speed when reversing, because the lower board reported unsigned speed; the timestamps; the frame of each input; and over-trusting the odometry yaw rate when the wheels slip.

**中文翻译**：我用 ekf_node 的二维模式，30 Hz。输入是由轮子得到的速度、由速度和转向角计算的偏航率 v tan delta / L，以及陀螺 z 轴角速率。对每个输入，我选择融合哪些变量，避免把相关来源的同一个量融合两次，并给每个输入设置现实的协方差。轮速噪声约 0.05 m/s，陀螺约 0.01 rad/s。我遇到的坑：倒车时速度符号，因为下位机上报的是无符号速度；时间戳；每个输入的坐标系；以及轮子打滑时过度信任里程计的偏航率。
**追问 1**：*How did you fix the reversing problem?* I parsed the speed as signed, added a gear check, and verified with a 3-metre reverse drive within about 5 centimetres of the tape measure.
　中文：把速度解析为带符号，加入挡位校验，并用 3 m 倒车验证，与卷尺相差约 5 cm。
**追问 2**：*What happened before the fix?* The odometry integrated forward during reverse, and the AMCL particle cloud spread out.
　中文：倒车时里程计仍向前积分，AMCL 粒子云炸开。
**如果追问到你的项目**：*That was one of my three key P5 faults; the root cause was a lost sign bit, not the filter.*
　中文：这是我 P5 三个关键故障之一；根因是丢失的符号位，而不是滤波器。
**关键词与易错点**：robot_localization、two_d_mode、covariance、signed speed、frame。

### Q63. What are NEES and NIS, and how do you use them?
**中文题意**：NEES 与 NIS 是什么？如何使用？
**考察点**：滤波器一致性检验。
**项目关联**：概念题；P5。
**English answer（口语）**：
> NIS, the normalised innovation squared, is the innovation transposed times the inverse innovation covariance times the innovation. It can be computed in the real system without ground truth. NEES, the normalised estimation error squared, is the state error transposed times the inverse state covariance times the error; it needs the ground truth, typically in simulation. If the filter is consistent, both follow a chi-square distribution with degrees of freedom equal to the measurement dimension for NIS and the state dimension for NEES, so their averages should be about those dimensions. If the average is much larger, the filter is overconfident; if much smaller, it is conservative. I use NIS for tuning on real data.

**中文翻译**：NIS（归一化新息平方）是新息转置乘新息协方差的逆乘新息。它可以在没有真值的真实系统里计算。NEES（归一化估计误差平方）是状态误差转置乘状态协方差的逆乘误差；它需要真值，通常在仿真中使用。若滤波器一致，两者都服从卡方分布，自由度对 NIS 是测量维数、对 NEES 是状态维数，所以其平均值应约等于这些维数。平均值远大于它说明滤波器过度自信；远小于则偏保守。我在真实数据上用 NIS 做整定。
**追问 1**：*What does an overconfident filter look like in practice?* It rejects valid measurements and corrects too slowly.
　中文：拒绝有效测量，修正太慢。
**追问 2**：*How do you set a pass band?* Use the chi-square confidence bounds for the average over a window.
　中文：对窗口平均值使用卡方置信界。
**如果追问到你的项目**：*I did not compute NIS in a formal way; I checked consistency by watching the covariance ellipse and pose-jump sizes during runs.*
　中文：我没有正式计算 NIS；我通过观察运行中的协方差椭圆和位姿跳变大小来检查一致性。
**关键词与易错点**：NIS、NEES、chi-square、consistency、ground truth。诚实说明。

### Q64. What is observability in state estimation, with an example?
**中文题意**：状态估计中的可观性是什么？举例。
**考察点**：可估性判断。
**项目关联**：P5、P1。
**English answer（口语）**：
> A state is observable if it can be determined from the measurements and the inputs over some time. If not, no filter can estimate it, however it is tuned. An example: with only wheel odometry and an IMU, the absolute position and heading in a map are not observable, since only changes are measured; the error grows without bound. Adding a map-based measurement, such as laser matching, makes the pose observable. Another example: the accelerometer gives roll and pitch, but not yaw. In an EKF, an unobservable state shows up as a covariance that keeps growing in that direction. The practical lesson is to ask, before tuning, whether the measurements can determine what I want to know.

**中文翻译**：如果一个状态能由一段时间内的测量和输入确定，就是可观的。否则无论如何调参，任何滤波器都无法估计它。例子：仅有轮里程计和 IMU 时，地图中的绝对位置和航向不可观，因为只测量了变化；误差无界增长。加入基于地图的测量（如激光匹配）后位姿变为可观。另一个例子：加速度计给出 roll 和 pitch，不给出 yaw。在 EKF 中，不可观的状态表现为该方向上的协方差持续增长。实践的教训是：在调参之前先问测量能否确定我想知道的量。
**追问 1**：*How do you test observability for a linear system?* The rank of the observability matrix C, C A, up to C A to the n minus one.
　中文：可观性矩阵 C, C A, …, C A 的 n 减 1 次方的秩。
**追问 2**：*What do you do for an unobservable bias?* Add an absolute reference, or accept the drift and reset it periodically.
　中文：加入绝对参考，或接受漂移并定期复位。
**如果追问到你的项目**：*In P1 the position was wheel odometry only, with a drift of about 5 centimetres in 60 seconds at rest, because there was no absolute position reference.*
　中文：P1 里位置只有轮里程计，静止 60 s 漂移约 5 cm，因为没有绝对位置参考。
**关键词与易错点**：observability、unobservable state、absolute reference、covariance growth。

### Q65. How does dead-reckoning error grow with distance, and what is a typical figure?
**中文题意**：航迹推算误差如何随距离增长？典型数字是多少？
**考察点**：误差增长认识。
**项目关联**：P1、P2、P5。
**English answer（口语）**：
> For dead reckoning from wheel odometry, the position error grows with distance travelled, typically as a percentage of the path length, because scale and heading errors accumulate. A heading error is the most serious, since a constant heading error gives a position error that grows linearly with distance and is perpendicular to the path. With a gyro-integrated heading, the heading error itself grows with time, so the position error grows faster than linearly. Typical figures for a wheeled robot are a few percent of the distance. In my ballbot, the drift was about 3 to 5 percent of the path, and about 5 centimetres in 60 seconds at rest. Bounded errors need an absolute reference such as a map or beacons.

**中文翻译**：从轮里程计做航迹推算，位置误差随行驶距离增长，通常是路径长度的百分比，因为比例误差和航向误差累积。航向误差最严重：恒定的航向误差使位置误差随距离线性增长，方向垂直于路径。用陀螺积分得到航向时，航向误差本身随时间增长，位置误差增长就快于线性。轮式机器人的典型数字是距离的百分之几。我的球形机器人漂移约路径的 3%–5%，静止 60 s 约 5 cm。要使误差有界，需要地图或信标等绝对参考。
**追问 1**：*Estimate: with 1 degree of heading error over 10 metres?* About 0.17 metre, since 10 metres times 0.0175 radian.
　中文：约 0.17 m，即 10 m 乘 0.0175 rad。
**追问 2**：*How do you bound it?* Map-based localisation, landmarks, or loop closure.
　中文：基于地图的定位、路标或回环。
**如果追问到你的项目**：*In the differential base of P2, a 5 metre straight run had about 2 percent error and ten in-place turns about 30 degrees of heading error; in P5 AMCL bounded this against the map.*
　中文：P2 差速底盘 5 m 直行误差约 2%，原地转 10 圈航向误差约 30°；P5 里 AMCL 相对地图约束了它。
**关键词与易错点**：dead reckoning、heading error、error growth、absolute reference。

### Q66. What is the difference between filtering, smoothing and prediction?
**中文题意**：滤波、平滑与预测有何区别？
**考察点**：估计问题分类。
**项目关联**：P5、SLAM 背景。
**English answer（口语）**：
> Filtering estimates the current state using all measurements up to now. Prediction estimates a future state, usually by propagating the model, and the uncertainty grows. Smoothing estimates a past state using measurements both before and after that time, so it is more accurate, but it needs a delay or an offline computation. In real-time control I use filtering, and prediction to compensate for delay. Smoothing is used offline, for example in mapping, or as fixed-lag smoothing with a small lag when a short delay is acceptable. Graph-based SLAM can be seen as smoothing, since it optimises the whole trajectory with all measurements.

**中文翻译**：滤波使用到现在为止的全部测量估计当前状态。预测估计未来状态，通常通过传播模型，不确定性增长。平滑使用该时刻前后的测量估计过去的状态，所以更准确，但需要延迟或离线计算。实时控制里我用滤波，并用预测补偿延迟。平滑用于离线场景（如建图），或在可接受短延迟时作为小滞后的定滞后平滑。基于图的 SLAM 可看作平滑，因为它用所有测量优化整条轨迹。
**追问 1**：*Why is smoothing more accurate?* It uses future information to refine past estimates.
　中文：它用未来信息细化过去的估计。
**追问 2**：*Where did you use prediction?* To compensate for the 120 millisecond delay in the MPC by propagating the state with the commands already sent.
　中文：用已发出的指令前推状态，补偿 MPC 中 120 ms 的延迟。
**如果追问到你的项目**：*My localisation was filtering in real time; for the map, slam_toolbox built it offline with graph optimisation.*
　中文：我的定位是实时滤波；地图由 slam_toolbox 离线以图优化构建。
**关键词与易错点**：filtering、smoothing、prediction、fixed-lag、graph optimisation。

---

## 6. E SLAM、视觉几何与相机标定（Q67–Q94）

### Q67. What is SLAM, and why is it hard?
**中文题意**：什么是 SLAM？为什么难？
**考察点**：问题陈述。
**项目关联**：P5（使用 slam_toolbox 建图，自己做定位）。
**English answer（口语）**：
> SLAM, simultaneous localisation and mapping, is the problem of building a map of an unknown environment while estimating the robot's pose in it. It is hard for three reasons. First, it is a chicken-and-egg problem: a good map needs good poses, and good poses need a good map. Second, errors accumulate: odometry drifts, and mapping with drifted poses distorts the map. Third, data association: deciding whether an observation is of a landmark seen before is ambiguous in repetitive places, and a wrong association can ruin the map. The standard remedies are probabilistic estimation, scan or feature matching, loop closure to cancel drift, and optimisation of the whole trajectory.

**中文翻译**：SLAM（同时定位与建图）是在未知环境中一边构建地图、一边估计机器人在其中位姿的问题。它难有三个原因。第一，鸡生蛋蛋生鸡：好的地图需要好的位姿，好的位姿需要好的地图。第二，误差累积：里程计会漂移，用漂移的位姿建图会使地图变形。第三，数据关联：判断一次观测是否是之前见过的路标，在重复场景中是模糊的，错误关联会毁掉地图。标准对策是概率估计、扫描或特征匹配、用回环抵消漂移，以及优化整条轨迹。
**追问 1**：*What is the difference between SLAM and localisation?* Localisation assumes a known map; SLAM builds the map as well.
　中文：定位假设地图已知；SLAM 还要建图。
**追问 2**：*What do you need to run SLAM?* A motion estimate, an exteroceptive sensor like LiDAR or a camera, and a back-end that handles loop closures.
　中文：运动估计、激光或相机等外部感知传感器，以及处理回环的后端。
**如果追问到你的项目**：*In P5 I built the map offline with slam_toolbox, and my own contribution was localisation on that map using EKF and AMCL, so I know SLAM from the user side and at the concept level.*
　中文：P5 里我用 slam_toolbox 离线建图，我自己的贡献是在该地图上用 EKF 和 AMCL 定位，所以我从使用者角度和概念层面了解 SLAM。
**关键词与易错点**：SLAM、chicken-and-egg、drift、data association、loop closure。不要声称自己写了 SLAM 算法。

### Q68. What are the front end and back end of a SLAM system?
**中文题意**：SLAM 系统的前端和后端分别是什么？
**考察点**：系统结构。
**项目关联**：概念题；P5。
**English answer（口语）**：
> The front end processes the raw sensor data into constraints. For LiDAR, it does scan matching between consecutive scans to get relative motion; for cameras it extracts and tracks features and estimates relative pose. It also detects loop closure candidates. The back end takes these constraints and finds the best globally consistent trajectory and map. In graph SLAM, the poses are nodes and the constraints are edges, and the back end solves a nonlinear least-squares problem to minimise the total error. The front end is fast and local; the back end is slower and global, and runs when loops are closed or in the background. Separating them makes the system modular.

**中文翻译**：前端把原始传感器数据处理成约束。对激光，前端在相邻扫描之间做扫描匹配得到相对运动；对相机则提取并跟踪特征，估计相对位姿。它还检测回环候选。后端接收这些约束，找到全局一致的最优轨迹和地图。图 SLAM 中位姿是节点、约束是边，后端求解非线性最小二乘使总误差最小。前端快速而局部；后端较慢而全局，在回环闭合时或后台运行。把它们分开使系统模块化。
**追问 1**：*What does the back end need to know about each constraint?* The measurement and its covariance, so it weighs constraints properly.
　中文：每个约束的测量及其协方差，以便正确加权。
**追问 2**：*Can the back end run in real time?* Incremental solvers like iSAM2 do, or it runs at a lower rate.
　中文：增量求解器（如 iSAM2）可以，或以较低频率运行。
**如果追问到你的项目**：*My navigation work sat after the SLAM: I used its map output and localised with EKF and AMCL.*
　中文：我的导航工作在 SLAM 之后：使用它输出的地图，并用 EKF 和 AMCL 定位。
**关键词与易错点**：front end、back end、scan matching、pose graph、loop closure candidate。

### Q69. What is loop closure, and why is it important?
**中文题意**：什么是回环检测？为什么重要？
**考察点**：漂移消除。
**项目关联**：概念题。
**English answer（口语）**：
> Loop closure is recognising that the robot has returned to a place it has visited before. That gives a constraint between the current pose and an old pose that are far apart in time, and it lets the back end distribute the accumulated drift along the whole loop, so the map becomes globally consistent. Detection uses place recognition: for LiDAR, matching scan descriptors or scan-to-submap; for cameras, bag-of-words on visual features. Verification is crucial, because a false loop closure, especially in repetitive corridors, can badly distort the map. So a candidate is geometrically verified, and robust back-end kernels or consistency checks can reject outliers.

**中文翻译**：回环检测是识别出机器人回到了曾经到过的地方。它给出当前位姿与时间上相隔很远的旧位姿之间的约束，使后端把累积的漂移沿整个回环分摊，让地图全局一致。检测使用场所识别：激光用扫描描述子匹配或扫描到子图匹配；相机用视觉特征的词袋。验证至关重要，因为错误的回环（尤其在重复的走廊里）会严重扭曲地图。所以候选要经几何验证，并可用鲁棒的后端核函数或一致性检验剔除离群点。
**追问 1**：*What is perceptual aliasing?* Different places that look the same, leading to false matches.
　中文：不同地点看起来一样，导致错误匹配。
**追问 2**：*How do you verify a loop closure?* Check the geometric fit with RANSAC or ICP residuals, and require temporal consistency.
　中文：用 RANSAC 或 ICP 残差检查几何吻合，并要求时间上一致。
**如果追问到你的项目**：*In my P5 localisation, the AMCL relocalisation error at a glass door was one of the two failures out of 22 runs, a similar issue of ambiguity.*
　中文：我 P5 的定位中，玻璃门处 AMCL 重定位错误是 22 次运行中的两次失败之一，是类似的歧义问题。
**关键词与易错点**：loop closure、place recognition、bag-of-words、false positive、verification。

### Q70. Compare filter-based, particle-based and graph-based SLAM.
**中文题意**：比较基于滤波、基于粒子和基于图的 SLAM。
**考察点**：SLAM 算法族。
**项目关联**：概念题。
**English answer（口语）**：
> EKF-SLAM keeps the robot pose and all landmarks in one Gaussian state. It is elegant but the covariance grows quadratically with the number of landmarks, and linearisation errors are permanent since old states are marginalised. Particle-based SLAM, such as FastSLAM or gmapping, uses a particle filter for the trajectory and a map per particle, so it handles non-Gaussian pose uncertainty, but the number of particles and memory become a problem in large areas. Graph-based SLAM keeps the whole trajectory and constraints and optimises them, re-linearising as needed. It is sparse, scales well, and is the current standard, for example in Cartographer and slam_toolbox. I use these as tools and understand their principles.

**中文翻译**：EKF-SLAM 把机器人位姿和所有路标放在一个高斯状态里。很优雅，但协方差随路标数二次增长，且旧状态被边缘化后线性化误差永久保留。基于粒子的 SLAM（如 FastSLAM 或 gmapping）用粒子滤波处理轨迹、每个粒子一张地图，可处理非高斯的位姿不确定性，但大场景中粒子数和内存成问题。基于图的 SLAM 保留整条轨迹和约束并优化它们，按需重新线性化。它稀疏、可扩展，是当前标准，如 Cartographer 和 slam_toolbox。我把这些当作工具使用，并理解其原理。
**追问 1**：*Why does graph optimisation exploit sparsity?* Each constraint involves only two poses, so the information matrix is sparse.
　中文：每个约束只涉及两个位姿，因此信息矩阵稀疏。
**追问 2**：*Why is marginalisation a problem for EKF-SLAM?* Past linearisation points cannot be revisited.
　中文：过去的线性化点无法重新访问。
**如果追问到你的项目**：*I used slam_toolbox, which is graph-based, to build the map offline for P5.*
　中文：P5 里我用基于图的 slam_toolbox 离线建图。
**关键词与易错点**：EKF-SLAM、FastSLAM、graph SLAM、marginalisation、sparsity。

### Q71. How does an occupancy grid map work?
**中文题意**：占据栅格地图如何工作？
**考察点**：地图表示与更新。
**项目关联**：P5（0.05 m 分辨率的地图与代价地图）。
**English answer（口语）**：
> An occupancy grid divides the environment into cells, each holding the probability that it is occupied. Each LiDAR beam tells me that the cells along the ray are free and the cell at the endpoint is occupied. To update, I use the log-odds form: log-odds equals the log of p over one minus p. The update is additive: add a positive increment for a hit, a negative one for a pass-through. That is numerically stable and cheap, with each cell treated as independent. To read the map, I convert back with the sigmoid, and I threshold into free, occupied and unknown. The resolution matters: 0.05 metre is common for indoor robots, and it sets the memory and the planning cost.

**中文翻译**：占据栅格把环境划分成单元格，每个格子保存其被占据的概率。每条激光射线告诉我射线上的格子是空闲的，终点格子被占据。更新用对数几率形式：对数几率等于 p/(1 减 p) 的对数。更新是加性的：命中则加正增量，穿过则加负增量。数值稳定且便宜，每个格子被视为独立。读取地图时用 sigmoid 转换回来，再用阈值分成空闲、占据和未知。分辨率很重要：室内机器人常用 0.05 m，它决定内存和规划代价。
**追问 1**：*What assumption is made?* Cells are independent, and the environment is static.
　中文：格子相互独立，环境是静态的。
**追问 2**：*What is the costmap built on top?* The occupancy grid plus an inflation layer, which grows obstacles by the robot radius and a safety margin.
　中文：占据栅格再加膨胀层，按机器人半径和安全余量扩大障碍。
**如果追问到你的项目**：*My P5 map was built with slam_toolbox at 0.05 metre resolution in a hall and corridor of about 25 by 15 metres.*
　中文：我的 P5 地图由 slam_toolbox 以 0.05 m 分辨率建成，场景为约 25 × 15 米的大厅与走廊。
**关键词与易错点**：occupancy grid、log-odds、ray casting、inflation、resolution。

### Q72. Explain ICP scan matching step by step.
**中文题意**：逐步解释 ICP 扫描匹配。
**考察点**：点云配准算法。
**项目关联**：概念题；P5（AMCL 的相关思想）。
**English answer（口语）**：
> Iterative closest point aligns two point sets. Start with an initial transform guess. Step one: for each point in the source set, find the nearest point in the target set; these pairs are the correspondences. Step two: estimate the rigid transform that minimises the sum of squared distances between the pairs; this has a closed-form solution through the SVD, the Kabsch method, by centring both sets and decomposing their cross-covariance matrix. Step three: apply the transform to the source. Repeat until the change is small. ICP only converges to a local minimum, so a good initial guess from odometry is important. Variants use point-to-line or point-to-plane distances, outlier rejection and k-d trees for fast nearest-neighbour search.

**中文翻译**：迭代最近点配准两个点集。从初始变换猜测开始。第一步：对源点集中的每个点，找到目标点集中最近的点，这些点对就是对应关系。第二步：估计使点对距离平方和最小的刚体变换；通过 SVD 有闭式解（Kabsch 方法），即把两个点集去中心化后分解其互协方差矩阵。第三步：把变换作用到源点集。重复直到变化很小。ICP 只收敛到局部极小，所以来自里程计的好初值很重要。变体使用点到线或点到面距离、离群点剔除，以及用 k-d 树加速最近邻搜索。
**追问 1**：*What are ICP's failure modes?* A poor initial guess, partial overlap, and geometric degeneracy such as a long straight corridor.
　中文：初值差、部分重叠，以及长直走廊这样的几何退化。
**追问 2**：*What is NDT?* The normal distributions transform represents the target as local Gaussians in cells, and maximises the likelihood of the source points; it is smooth and avoids explicit correspondences.
　中文：正态分布变换把目标表示为栅格内的局部高斯，并最大化源点的似然；平滑且无需显式对应。
**如果追问到你的项目**：*I use ICP as a concept; the localisation in my navigation used AMCL's likelihood-field rather than ICP.*
　中文：我把 ICP 作为概念使用；我导航中的定位用的是 AMCL 的似然场而不是 ICP。
**关键词与易错点**：ICP、correspondences、Kabsch、SVD、local minimum、k-d tree。

### Q73. What is the difference between tracking, global localisation and the kidnapped robot problem?
**中文题意**：位姿跟踪、全局定位与绑架机器人问题有何区别？
**考察点**：定位问题分类。
**项目关联**：P5。
**English answer（口语）**：
> In pose tracking, the initial pose is roughly known and the filter just keeps the estimate accurate. In global localisation, the initial pose is unknown, so the belief must cover the whole map, which a particle filter handles by spreading particles everywhere, and it has to converge as the robot moves and sees more. The kidnapped robot problem is a harder case: the robot is moved to a new place without the filter knowing. To recover, the filter must detect that the estimate no longer fits the measurements and re-initialise, for example by injecting random particles when the average likelihood falls. In practice I give AMCL an initial pose from the operator, which turns the task into tracking.

**中文翻译**：位姿跟踪中，初始位姿大致已知，滤波器只需保持估计准确。全局定位中，初始位姿未知，信念必须覆盖整张地图，粒子滤波通过在各处撒粒子处理，并在机器人移动、看到更多信息时收敛。绑架机器人问题更难：机器人在滤波器不知情的情况下被搬到新位置。要恢复，滤波器必须检测到估计不再符合测量并重新初始化，例如平均似然下降时注入随机粒子。实践中我给 AMCL 由操作员提供的初始位姿，把任务变成跟踪。
**追问 1**：*What does your AMCL do when lost?* The covariance grows and the localisation quality check fails; the navigation should stop rather than continue.
　中文：协方差增大，定位质量检查失败；导航应停止而不是继续。
**追问 2**：*Which failure did you see in testing?* A relocalisation error at a glass door, where the laser sees through the glass.
　中文：玻璃门处的重定位错误，激光穿透玻璃。
**如果追问到你的项目**：*Of the 22 runs recorded, 20 succeeded, and one of the two failures was a relocalisation error at the glass door.*
　中文：记录的 22 次运行中 20 次成功，两次失败之一是玻璃门处的重定位错误。
**关键词与易错点**：pose tracking、global localisation、kidnapped robot、random particle injection。

### Q74. Which SLAM packages do you know, and how do they differ?
**中文题意**：你了解哪些 SLAM 软件包？有何区别？
**考察点**：工具层面的认知。
**项目关联**：P5（slam_toolbox）。
**English answer（口语）**：
> For 2D LiDAR, gmapping is a particle-filter approach, simple and light, but memory heavy in big maps and needs odometry. Cartographer, from Google, is graph-based with submaps and loop closure, and supports 2D and 3D. slam_toolbox is graph-based on the Karto approach, with a pose graph, good tools to save and edit maps, and it supports both online mapping and lifelong mapping; it is the common choice in ROS2 Nav2 systems. For vision, ORB-SLAM is feature-based and widely used, and there are visual–inertial systems like VINS. For LiDAR in 3D there are LOAM-style methods. I used slam_toolbox in P5 because it integrates well with Nav2.

**中文翻译**：对 2D 激光，gmapping 是粒子滤波方法，简单轻量，但大地图内存重且需要里程计。Google 的 Cartographer 是基于图的，带子图和回环，支持 2D 和 3D。slam_toolbox 基于 Karto 方法的图优化，带位姿图，有保存与编辑地图的良好工具，支持在线建图和终身建图，是 ROS2 Nav2 系统里常见的选择。视觉方面，ORB-SLAM 基于特征且广泛使用，还有 VINS 等视觉惯性系统。3D 激光则有 LOAM 类方法。我在 P5 里用 slam_toolbox，因为它与 Nav2 集成良好。
**追问 1**：*Do you need odometry for slam_toolbox?* It benefits from a good odometry, but scan matching can also work with weak odometry.
　中文：好的里程计有益，但扫描匹配在里程计较弱时也能工作。
**追问 2**：*What can you save from a map session?* The occupancy map for localisation, and the serialised pose graph for continued mapping.
　中文：用于定位的占据地图，以及用于继续建图的序列化位姿图。
**如果追问到你的项目**：*I used it offline to build the map, then ran AMCL on the saved map.*
　中文：我离线用它建图，然后在保存的地图上运行 AMCL。
**关键词与易错点**：gmapping、Cartographer、slam_toolbox、ORB-SLAM、LOAM。

### Q75. What is motion distortion in a LiDAR scan, and how do you correct it?
**中文题意**：激光扫描里的运动畸变是什么？如何校正？
**考察点**：传感器时序细节。
**项目关联**：P5（激光时间戳）。
**English answer（口语）**：
> A rotating LiDAR takes a scan over about a tenth of a second, so if the robot moves or turns during the scan, the points are captured from different poses. If I treat them as one snapshot, the scan is skewed, and a wall can look curved. The correction is deskewing: assign each point its own capture time, interpolate the robot pose at that time from odometry or IMU, and transform all points to a common reference time. At a 10 hertz scan and a turn of one radian per second, the heading changes by 0.1 radian, about 5.7 degrees, during a scan, which is large. A related issue is the timestamp convention of the driver: the scan start time versus arrival time.

**中文翻译**：旋转激光雷达在约十分之一秒内完成一次扫描，所以机器人在扫描期间移动或转动时，各点是从不同位姿采集的。把它们当作一次快照会使扫描倾斜，墙可能看起来是弯的。校正叫去畸变：给每个点分配各自的采集时间，用里程计或 IMU 插值出该时刻的机器人位姿，把所有点变换到共同的参考时间。10 Hz 扫描、转速 1 rad/s 时，一次扫描期间航向变化 0.1 rad（约 5.7 度），这很大。一个相关问题是驱动的时间戳约定：扫描起始时刻与到达时刻。
**追问 1**：*Is deskewing needed at low speed?* It matters less, but turning is the critical case.
　中文：低速时不那么重要，但转弯是关键情形。
**追问 2**：*What did you change in your driver?* The timestamp is the scan start, with a fixed delay of about 60 milliseconds compensated.
　中文：时间戳改为扫描起始，并补偿约 60 ms 的固定延迟。
**如果追问到你的项目**：*That was in P5; the wall smear in turns went away after the timestamp fix.*
　中文：这是 P5 的事；时间戳修复后转弯时墙体拖影消失。
**关键词与易错点**：motion distortion、deskewing、timestamp convention、scan time。

### Q76. When does LiDAR SLAM or scan matching degenerate?
**中文题意**：激光 SLAM 或扫描匹配何时退化？
**考察点**：失效模式。
**项目关联**：P5（走廊场景）。
**English answer（口语）**：
> Scan matching needs geometric constraints in all directions. In a long, straight corridor, the walls constrain the lateral position and the heading, but not the position along the corridor, so the matcher can slide along it. In an open hall, few points are within range, so there are not enough features. Glass and mirrors give wrong or no returns; highly dynamic scenes with many people move the points that should be static. Symptoms are a pose that drifts along the corridor or a map that is stretched. Remedies are fusing odometry and the IMU, which constrain the unobservable direction, using landmarks or reflectors, adding another sensor such as a camera, or detecting degeneracy from the eigenvalues of the matching problem and relying on other sensors in that direction.

**中文翻译**：扫描匹配需要各个方向上的几何约束。在长直走廊里，墙约束横向位置和航向，但不约束沿走廊方向的位置，所以匹配器会沿走廊滑动。在开阔大厅里，范围内点很少，特征不足。玻璃和镜子给出错误或没有回波；有很多行人的高动态场景会移动本应静止的点。症状是位姿沿走廊漂移或地图被拉伸。对策是融合里程计与 IMU 以约束不可观方向、使用路标或反光板、增加相机等其他传感器，或通过匹配问题的特征值检测退化，并在该方向依赖其他传感器。
**追问 1**：*How do you detect degeneracy numerically?* Check the eigenvalues of the matching Hessian; a small one marks a weakly constrained direction.
　中文：检查匹配 Hessian 的特征值，小特征值标志约束弱的方向。
**追问 2**：*Which of these did you face?* The glass-door failure and the importance of odometry in long corridors.
　中文：玻璃门失败，以及长走廊中里程计的重要性。
**如果追问到你的项目**：*In P5 the EKF odometry gave the constraint along the corridor while AMCL corrected the lateral position and heading.*
　中文：P5 里 EKF 里程计给出沿走廊方向的约束，AMCL 修正横向位置和航向。
**关键词与易错点**：degeneracy、corridor、glass、dynamic objects、Hessian eigenvalue。

### Q77. How do you handle dynamic objects in mapping and localisation?
**中文题意**：建图和定位中如何处理动态物体？
**考察点**：环境假设。
**项目关联**：P5。
**English answer（口语）**：
> Most SLAM and localisation methods assume a static world, so moving people or objects cause wrong matches. There are several practical approaches. First, the likelihood-field model in AMCL is robust to a modest number of unexpected obstacles, since it includes a random-measurement term. Second, use only a subset of beams, and gate the beams whose range disagrees strongly with the map. Third, in mapping, build the map when the environment is quiet, or remove dynamic points by comparing multiple scans. Fourth, use semantic segmentation to remove people. Fifth, fuse odometry so that a temporary mismatch does not move the pose much. In the costmap, dynamic obstacles are marked and cleared by raytracing.

**中文翻译**：大多数 SLAM 和定位方法假设世界静止，所以移动的人或物体会造成错误匹配。有几种实用做法。第一，AMCL 的似然场模型对少量意外障碍比较鲁棒，因为它包含随机测量项。第二，只使用部分射线，并对距离与地图严重不符的射线做门限。第三，建图时选环境安静的时候，或通过比较多帧扫描去除动态点。第四，用语义分割去除行人。第五，融合里程计，使暂时的失配不会大幅移动位姿。代价地图里，动态障碍通过光线追踪被标记和清除。
**追问 1**：*What is the random-measurement term?* A uniform component in the beam model that accounts for unexplained readings.
　中文：射线模型中解释不明读数的均匀分量。
**追问 2**：*How are dynamic obstacles cleared in the costmap?* Raytracing from the sensor marks free space between the sensor and the hit.
　中文：从传感器出发的光线追踪把传感器与命中点之间标为空闲。
**如果追问到你的项目**：*My lab hall tests had people walking, and I used about 60 beams per scan and sensible update thresholds so single mismatches did not move the pose.*
　中文：我在实验室大厅测试时有人走动，我每次扫描用约 60 条射线并设置合理的更新门限，使单次失配不会移动位姿。
**关键词与易错点**：static world assumption、random measurement、beam subset、raytracing。

### Q78. How do you evaluate a SLAM or localisation result?
**中文题意**：如何评估 SLAM 或定位结果？
**考察点**：评价指标。
**项目关联**：P5。
**English answer（口语）**：
> With ground truth, I use the absolute trajectory error, ATE, which is the root-mean-square difference between estimated and true positions after aligning the trajectories, and the relative pose error, RPE, which measures local drift over a fixed distance or time. ATE shows the global consistency, RPE shows the drift rate. Without ground truth, I use proxies: the closure error when returning to the start, consistency of the map such as thin straight walls, the covariance, and repeatability across runs. On a real robot, I also use task metrics, like the final position error at a goal, measured with a tape measure. In P5 the goal position error was under 0.15 metres and the heading under 8 degrees.

**中文翻译**：有真值时，我用绝对轨迹误差 ATE（对齐轨迹后估计位置与真实位置差的均方根）和相对位姿误差 RPE（在固定距离或时间内的局部漂移）。ATE 反映全局一致性，RPE 反映漂移率。没有真值时用代理指标：回到起点的闭合误差、地图一致性（如墙是否细而直）、协方差，以及多次运行的可重复性。在真实机器人上我还用任务指标，比如用卷尺测得的目标点最终位置误差。P5 里到点位置误差低于 0.15 m，航向低于 8 度。
**追问 1**：*Why align before computing ATE?* The estimated and true frames differ by an unknown rigid transform.
　中文：估计与真值的坐标系相差一个未知刚体变换。
**追问 2**：*What is a good RPE interval?* A distance relevant to the task, such as 10 metres, or a segment length.
　中文：与任务相关的距离，如 10 m，或一段长度。
**如果追问到你的项目**：*My P5 numbers: mean lateral tracking error at 0.5 metres per second was about 6 centimetres with a maximum of about 15, and 20 of 22 runs completed.*
　中文：我的 P5 数字：0.5 m/s 下平均横向跟踪误差约 6 cm、最大约 15 cm，22 次运行完成 20 次。
**关键词与易错点**：ATE、RPE、ground truth、closure error、task metric。

### Q79. What map representations exist, and when would you use each?
**中文题意**：有哪些地图表示？各自何时使用？
**考察点**：地图选型。
**项目关联**：P5。
**English answer（口语）**：
> A 2D occupancy grid is the standard for planar navigation: simple, dense, and compatible with costmaps. A 3D voxel grid or OctoMap represents volumetric occupancy, with memory-efficient octrees, useful for 3D obstacle avoidance. Signed distance fields such as TSDF give smooth surfaces and fast distance queries, used in dense reconstruction and for planning. Feature or landmark maps store sparse points with descriptors, used by visual SLAM for localisation. Topological maps store places and connections, good for large-scale route planning. Point cloud maps keep raw geometry, used in LiDAR localisation by registration. I choose by the task: planning, localisation or reconstruction, and the memory budget.

**中文翻译**：2D 占据栅格是平面导航的标准：简单、稠密，并与代价地图兼容。3D 体素栅格或 OctoMap 表示体积占据，用内存高效的八叉树，适合 3D 避障。有符号距离场（如 TSDF）给出平滑表面和快速距离查询，用于稠密重建和规划。特征或路标地图存储带描述子的稀疏点，被视觉 SLAM 用于定位。拓扑地图存储地点与连接，适合大范围路径规划。点云地图保留原始几何，用于通过配准的激光定位。我按任务选择：规划、定位还是重建，以及内存预算。
**追问 1**：*Why do costmaps use a grid?* Cell lookup is O(1), and inflation and collision checks are fast on a grid.
　中文：栅格查询 O(1)，膨胀和碰撞检测快。
**追问 2**：*What is a drawback of grid maps at scale?* Memory grows with area at a fixed resolution.
　中文：固定分辨率下内存随面积增长。
**如果追问到你的项目**：*For P5 I used a 2D occupancy map at 0.05 metre resolution plus a costmap with the vehicle footprint.*
　中文：P5 里我用 0.05 m 分辨率的 2D 占据地图，加带车体轮廓的代价地图。
**关键词与易错点**：occupancy grid、OctoMap、TSDF、landmark map、topological map。

### Q80. Describe the pipeline of feature-based visual odometry.
**中文题意**：描述基于特征的视觉里程计流程。
**考察点**：视觉 SLAM 前端。
**项目关联**：概念题（P4 相机相关）。
**English answer（口语）**：
> First, detect features in each frame, like corners with FAST or ORB. Second, describe them and match between frames, or track them with optical flow. Third, reject wrong matches with RANSAC using a geometric model. Fourth, estimate the relative motion: for a monocular camera, use the essential matrix from the epipolar constraint and decompose it into rotation and translation up to scale; for known 3D points, use PnP. Fifth, triangulate new 3D points from two views, and keep a local map. Sixth, refine the poses and points by bundle adjustment over a window, and add keyframes. The result is a trajectory, with drift, and loop closure can reduce it.

**中文翻译**：第一，在每帧中检测特征，如用 FAST 或 ORB 的角点。第二，描述它们并在帧间匹配，或用光流跟踪。第三，用基于几何模型的 RANSAC 剔除错误匹配。第四，估计相对运动：单目相机用对极约束得到本质矩阵，并分解为旋转和尺度未知的平移；已知 3D 点时用 PnP。第五，从两个视图三角化新的 3D 点，并保持局部地图。第六，在窗口内用光束法平差优化位姿和点，并添加关键帧。结果是带漂移的轨迹，回环可以减小漂移。
**追问 1**：*Why keyframes?* To avoid redundant computation and to keep a well-conditioned baseline between views.
　中文：避免冗余计算，并保持视图间良好条件的基线。
**追问 2**：*What goes wrong with fast motion?* Motion blur and little overlap break tracking.
　中文：运动模糊和重叠少会破坏跟踪。
**如果追问到你的项目**：*I have not built a visual odometry system; I used camera-based detection in P4, and I understand the VO pipeline at the concept level.*
　中文：我没有构建过视觉里程计系统；P4 里我用了基于相机的检测，我在概念层面理解 VO 流程。
**关键词与易错点**：feature detection、matching、RANSAC、essential matrix、PnP、bundle adjustment。诚实说明。

### Q81. Compare Harris, FAST, ORB and SIFT features.
**中文题意**：比较 Harris、FAST、ORB 与 SIFT 特征。
**考察点**：特征基础。
**项目关联**：概念题。
**English answer（口语）**：
> Harris is a corner detector based on the image gradient structure matrix: it finds points where the intensity changes strongly in two directions. FAST checks a ring of pixels around a candidate and is very fast, but gives no descriptor and is not scale-invariant by itself. ORB combines a FAST detector with an oriented BRIEF binary descriptor, adds a scale pyramid, and is fast and free to use, which is why it is popular in real-time SLAM. SIFT is scale-invariant and rotation-invariant, with a floating-point descriptor, very robust but slow, historically patent-restricted, and now commonly available. Binary descriptors are matched by Hamming distance, which is much faster than Euclidean distance on float descriptors.

**中文翻译**：Harris 是基于图像梯度结构矩阵的角点检测器：找到强度在两个方向上都强烈变化的点。FAST 检查候选点周围一圈像素，速度极快，但没有描述子，本身也不具尺度不变性。ORB 把 FAST 检测器与带方向的 BRIEF 二进制描述子结合，加了尺度金字塔，快且可自由使用，所以在实时 SLAM 中流行。SIFT 具有尺度和旋转不变性，使用浮点描述子，非常鲁棒但很慢，历史上受专利限制，现在已普遍可用。二进制描述子用汉明距离匹配，比浮点描述子的欧氏距离快得多。
**追问 1**：*Why is Hamming matching fast?* XOR and a population count on a few machine words.
　中文：对几个机器字做异或和位计数。
**追问 2**：*What makes a good feature?* Repeatable under viewpoint and lighting change, distinctive, and well distributed over the image.
　中文：在视角和光照变化下可重复、有区分性，并在图像中分布良好。
**如果追问到你的项目**：*I used a learned detector in P4, not hand-crafted features, so this is background knowledge for SLAM.*
　中文：P4 里我用的是学习式检测器而不是手工特征，所以这是 SLAM 的背景知识。
**关键词与易错点**：Harris、FAST、ORB、SIFT、Hamming distance、scale invariance。

### Q82. Explain epipolar geometry and the essential matrix.
**中文题意**：解释对极几何与本质矩阵。
**考察点**：多视图几何核心。
**项目关联**：概念题。
**English answer（口语）**：
> For two views of the same 3D point, the point, the two camera centres and the two image points lie in one plane, the epipolar plane. This gives a constraint: a point in the first image must lie on a line, the epipolar line, in the second image. With normalised camera coordinates, the constraint is x two transpose times E times x one equals zero. E is the essential matrix, equal to the skew-symmetric matrix of the translation times the rotation. With pixel coordinates and unknown intrinsics, the same constraint uses the fundamental matrix F. From at least five correspondences, I can estimate E; from eight, F by the linear method. Decomposing E gives the rotation and the translation direction, four possible solutions, and the correct one puts points in front of both cameras.

**中文翻译**：对同一 3D 点的两个视图，该点、两个相机中心和两个像点位于同一平面，即对极平面。这给出一个约束：第一幅图像中的点必须落在第二幅图像的一条直线（对极线）上。在归一化相机坐标下，约束是 x2 转置乘 E 乘 x1 等于零。E 是本质矩阵，等于平移的反对称矩阵乘旋转。在像素坐标且内参未知时，同样的约束用基础矩阵 F。至少用五个对应可估计 E；八个对应可用线性方法估计 F。分解 E 得到旋转和平移方向，有四种可能解，正确解使点位于两个相机前方。
**追问 1**：*Why is the translation only up to scale?* The constraint is homogeneous in the translation, so scaling it does not change the equation.
　中文：约束对平移是齐次的，缩放它不改变方程。
**追问 2**：*What is the relation between E and F?* F equals K two inverse transpose times E times K one inverse.
　中文：F 等于 K2 逆的转置乘 E 乘 K1 的逆。
**如果追问到你的项目**：*This is background; my nearest practical experience is the camera frame handling in P4 and the time/frame alignment in P5 and P6.*
　中文：这是背景知识；我最接近的实践经验是 P4 的相机处理以及 P5、P6 的时间与坐标系对齐。
**关键词与易错点**：epipolar line、essential matrix、fundamental matrix、five-point、eight-point、scale ambiguity。

### Q83. Why does monocular vision have scale ambiguity, and how is it resolved?
**中文题意**：为什么单目视觉有尺度歧义？如何解决？
**考察点**：深度恢复的基本限制。
**项目关联**：概念题。
**English answer（口语）**：
> A single camera projects 3D onto a 2D image, and scaling the whole scene and the translation by the same factor gives identical images. So from images alone, the translation and the map are known only up to an unknown scale. It can be resolved with additional information: a stereo camera, where the known baseline sets the scale; an IMU, whose accelerations are metric, which is why visual–inertial odometry has a metric scale; a depth camera or LiDAR; a known object size; or an assumption such as a known camera height above a flat ground. Monocular systems also have scale drift over time, because the scale is propagated through the chain of frames.

**中文翻译**：单个相机把 3D 投影到 2D 图像，把整个场景和平移按同一比例缩放会得到相同的图像。所以仅从图像，平移和地图只能确定到一个未知尺度。可以用额外信息解决：双目相机，已知基线确定尺度；IMU，其加速度是度量的，因此视觉惯性里程计有度量尺度；深度相机或激光雷达；已知物体尺寸；或像已知平坦地面上相机高度这样的假设。单目系统还有尺度漂移，因为尺度通过帧链传播。
**追问 1**：*Can you recover scale from a single image with a neural network?* Learned depth gives a prior on scale, but with errors that do not generalise reliably.
　中文：学习的深度给出尺度先验，但误差不能可靠泛化。
**追问 2**：*Why does the IMU give scale?* Double integration of metric acceleration gives metric displacement, which observes the scale when excited.
　中文：对度量加速度二次积分得到度量位移，激励充分时可观测尺度。
**如果追问到你的项目**：*I have not used monocular SLAM; for scale in my robots I rely on wheel odometry and LiDAR, which are metric.*
　中文：我没用过单目 SLAM；我的机器人靠轮里程计和激光获得尺度，它们都是度量的。
**关键词与易错点**：scale ambiguity、baseline、visual–inertial、scale drift。

### Q84. What is PnP, and how is RANSAC used with it?
**中文题意**：PnP 是什么？RANSAC 如何配合使用？
**考察点**：位姿估计与鲁棒拟合。
**项目关联**：概念题。
**English answer（口语）**：
> Perspective-n-point estimates the camera pose, rotation and translation, from n known 3D points and their 2D image projections. The minimum is three points, which gives up to four solutions, so a fourth point disambiguates; in practice, I use more points with a non-linear refinement that minimises the reprojection error. Because some matches are wrong, I wrap it in RANSAC: repeatedly pick a minimal random subset, compute a pose, count how many points agree within a pixel threshold, the inliers, and keep the best pose. With an inlier ratio of one half and four points per sample, about 72 iterations give a 99 percent chance of one clean sample. Then I refit using all inliers.

**中文翻译**：Perspective-n-point 由 n 个已知 3D 点及其 2D 图像投影估计相机位姿（旋转和平移）。最少三个点，最多给出四个解，所以第四个点用于消歧；实际中我用更多点，并用最小化重投影误差的非线性精化。因为有些匹配是错的，我把它包进 RANSAC：反复随机选取最小子集、计算位姿、统计在像素阈值内一致的点数（内点），保留最好的位姿。内点比例为一半、每次采样 4 个点时，约 72 次迭代就有 99% 的概率得到一次干净样本。然后用所有内点重新拟合。
**追问 1**：*Derive the iteration count.* N equals log of one minus p over log of one minus w to the s; with p of 0.99, w of 0.5 and s of 4, it is about 72.
　中文：N 等于 log(1 减 p) 除以 log(1 减 w 的 s 次方)；p 为 0.99、w 为 0.5、s 为 4 时约 72。
**追问 2**：*What is the reprojection error?* The pixel distance between the observed point and the projection of its 3D point with the estimated pose.
　中文：观测点与用估计位姿投影的 3D 点之间的像素距离。
**如果追问到你的项目**：*I know RANSAC and the idea of inlier gating well; the nearest example in my work is the gating on map-to-odom jumps in P5.*
　中文：我熟悉 RANSAC 和内点门限的思想；我工作中最接近的例子是 P5 对 map 到 odom 跳变的门限处理。
**关键词与易错点**：PnP、RANSAC、inliers、reprojection error、iteration count。

### Q85. What is bundle adjustment?
**中文题意**：什么是光束法平差？
**考察点**：视觉 SLAM 后端。
**项目关联**：概念题。
**English answer（口语）**：
> Bundle adjustment jointly refines the camera poses and the 3D point positions by minimising the total reprojection error: the squared pixel distance between each observed feature and the projection of its 3D point into that camera. It is a nonlinear least-squares problem, solved with Gauss–Newton or Levenberg–Marquardt. The key to making it fast is the sparse structure: each observation involves one camera and one point, so the normal equations have a block structure that can be solved with the Schur complement, which eliminates the points first. Full bundle adjustment is expensive, so real-time systems run it over a local window of keyframes, and globally only after loop closure.

**中文翻译**：光束法平差通过最小化总重投影误差来联合优化相机位姿和 3D 点位置：每个观测特征与其 3D 点投影到该相机的像素距离平方。这是非线性最小二乘问题，用高斯—牛顿或 LM 求解。让它变快的关键是稀疏结构：每个观测只涉及一个相机和一个点，所以法方程有分块结构，可以用 Schur 补先消去点来求解。完整的光束法平差代价高，所以实时系统在关键帧的局部窗口内运行，只有回环后才做全局优化。
**追问 1**：*What is the gauge freedom?* The overall pose and scale can be moved without changing the error, so they are fixed by anchoring.
　中文：整体位姿和尺度可以移动而不改变误差，所以要通过锚定固定。
**追问 2**：*Why use a robust loss?* Wrong matches would otherwise dominate the squared error.
　中文：否则错误匹配会主导平方误差。
**如果追问到你的项目**：*I use the same least-squares tooling idea in the TEB optimisation of P5, though with different variables.*
　中文：P5 的 TEB 优化用了同样的最小二乘工具思想，只是变量不同。
**关键词与易错点**：bundle adjustment、reprojection error、Schur complement、sparse、local window。

### Q86. What is visual–inertial odometry, and what is IMU preintegration?
**中文题意**：什么是视觉惯性里程计？什么是 IMU 预积分？
**考察点**：多传感器紧耦合的概念。
**项目关联**：概念题。
**English answer（口语）**：
> Visual–inertial odometry fuses a camera with an IMU. The camera gives rich, drift-limited relative information, but it fails in fast motion, in blur, and has scale ambiguity; the IMU gives high-rate motion that bridges the gaps and gives metric scale and gravity direction, but it drifts quickly. Together they are complementary. In an optimisation-based system, the IMU measurements between two keyframes are summarised into a single relative-motion constraint, called a preintegration factor, which can be reused without re-integrating when the linearisation point changes. The state also includes the IMU biases. It is a tightly coupled design, and it is the basis of systems such as VINS and ORB-SLAM3 with an IMU.

**中文翻译**：视觉惯性里程计融合相机与 IMU。相机提供丰富的、漂移受限的相对信息，但在快速运动、模糊时会失效，且有尺度歧义；IMU 提供高频运动信息填补空隙并给出度量尺度和重力方向，但漂移很快。两者互补。在基于优化的系统里，两个关键帧之间的 IMU 测量被汇总成一个相对运动约束（预积分因子），线性化点改变时无需重新积分即可复用。状态还包括 IMU 偏置。这是紧耦合设计，是 VINS 和带 IMU 的 ORB-SLAM3 等系统的基础。
**追问 1**：*Why preintegrate?* Otherwise every change in the optimisation would require integrating hundreds of IMU samples again.
　中文：否则优化中每次变化都需要重新积分数百个 IMU 样本。
**追问 2**：*What must be calibrated?* The camera–IMU extrinsics and the time offset, plus the IMU noise parameters.
　中文：相机—IMU 外参和时间偏移，以及 IMU 噪声参数。
**如果追问到你的项目**：*I have not built VIO. The relevant experience I have is timing, frame and bias handling for IMU-based attitude and for fusing sensors in P5 and P7.*
　中文：我没构建过 VIO。我相关的经验是 IMU 姿态的时间、坐标系与偏置处理，以及 P5、P7 的传感器融合。
**关键词与易错点**：visual–inertial odometry、preintegration、bias、tightly coupled、extrinsics。诚实说明。

### Q87. Compare direct and feature-based visual SLAM.
**中文题意**：比较直接法与特征法视觉 SLAM。
**考察点**：方法族。
**项目关联**：概念题。
**English answer（口语）**：
> Feature-based methods extract and match keypoints, then minimise the reprojection error. They are robust to illumination changes and large motion, and the features serve for loop closure, but they use only a sparse subset of the image and fail in textureless areas. Direct methods minimise the photometric error, the intensity difference between the image and its warp from the previous frame, using pixels with gradient, even in edges. They use more information, work in low-texture scenes, and can produce semi-dense maps, but they assume brightness constancy, so they are sensitive to exposure changes and need good photometric calibration. Hybrid methods such as SVO combine both ideas.

**中文翻译**：特征法提取并匹配关键点，然后最小化重投影误差。它们对光照变化和大运动鲁棒，特征还可用于回环，但只使用图像的稀疏子集，在无纹理区域会失效。直接法最小化光度误差，即图像与由上一帧变换得到的图像之间的强度差，使用有梯度的像素，包括边缘。它们使用更多信息，能在低纹理场景工作，可生成半稠密地图，但假设亮度恒定，对曝光变化敏感，需要良好的光度标定。SVO 等混合方法结合两种思想。
**追问 1**：*What is brightness constancy?* The assumption that a scene point has the same intensity in consecutive frames.
　中文：场景点在连续帧中强度相同的假设。
**追问 2**：*Which is better for a robot indoors?* Feature-based is more robust to auto-exposure changes; direct can do better in low-texture corridors with good calibration.
　中文：特征法对自动曝光变化更鲁棒；直接法在良好标定的低纹理走廊可能更好。
**如果追问到你的项目**：*It is conceptual knowledge for me, as my navigation stack is LiDAR-based.*
　中文：这对我是概念知识，因为我的导航栈基于激光雷达。
**关键词与易错点**：direct method、photometric error、reprojection error、brightness constancy。

### Q88. Explain the pinhole camera model and the intrinsic matrix.
**中文题意**：解释针孔相机模型与内参矩阵。
**考察点**：相机模型，必会。
**项目关联**：P4（摄像头）。
**English answer（口语）**：
> In the pinhole model, a 3D point X, Y, Z in the camera frame, with Z along the optical axis, projects to the pixel u equals f x times X over Z plus c x, and v equals f y times Y over Z plus c y. The focal lengths f x and f y are in pixels, equal to the focal length divided by the pixel size, and c x, c y is the principal point, near the image centre. In matrix form, the intrinsic matrix K has f x, f y on the diagonal, c x and c y in the last column, and a skew term that is usually zero. Dividing by Z is the perspective effect: farther objects appear smaller. For a width of 640 pixels and f x of 500, the horizontal field of view is about 65 degrees.

**中文翻译**：针孔模型中，相机系下的 3D 点 (X, Y, Z)（Z 沿光轴）投影为像素 u 等于 f x 乘 X/Z 加 c x，v 等于 f y 乘 Y/Z 加 c y。焦距 f x 和 f y 以像素为单位，等于物理焦距除以像素尺寸，(c x, c y) 是主点，接近图像中心。矩阵形式下，内参矩阵 K 的对角线是 f x、f y，最后一列是 c x、c y，还有通常为零的倾斜项。除以 Z 是透视效应：越远的物体看起来越小。宽 640 像素、f x 为 500 时，水平视场角约 65 度。
**追问 1**：*Check the field of view.* Two times arctangent of 320 over 500 is two times 32.6, about 65 degrees.
　中文：2 乘 arctan(320/500)，即 2 乘 32.6，约 65 度。
**追问 2**：*How do you back-project a pixel with known depth?* X equals Z times u minus c x over f x, and Y equals Z times v minus c y over f y.
　中文：X 等于 Z 乘 (u 减 c x) 除以 f x，Y 等于 Z 乘 (v 减 c y) 除以 f y。
**如果追问到你的项目**：*In P4 the camera was a USB camera, 1280 by 720 at 30 frames per second, with the model input letterboxed to 640 by 640.*
　中文：P4 里相机是 USB 摄像头，1280×720@30 帧，模型输入经 letterbox 到 640×640。
**关键词与易错点**：pinhole model、intrinsic matrix、focal length in pixels、principal point、field of view。

### Q89. What is lens distortion, and how do you correct it?
**中文题意**：什么是镜头畸变？如何校正？
**考察点**：标定的一部分。
**项目关联**：P4。
**English answer（口语）**：
> Real lenses bend light, so straight lines in the world appear curved. The main types are radial distortion, where points are displaced along the radius, giving barrel or pincushion shapes, and tangential distortion, from a slightly misaligned lens. The common model, Brown–Conrady, writes the distorted normalised coordinates as the ideal ones times one plus k1 r squared plus k2 r to the fourth, plus tangential terms with p1 and p2. Calibration estimates these coefficients along with the intrinsics. To correct an image, I undistort it, by mapping each output pixel to the source position through the model. For wide-angle or fisheye lenses a different model is needed. Detection and geometry both benefit from undistortion.

**中文翻译**：真实镜头会弯曲光线，所以世界中的直线在图像里显得弯曲。主要类型是径向畸变（点沿半径方向位移，呈桶形或枕形）和切向畸变（镜头略有错位引起）。常用的 Brown–Conrady 模型把畸变后的归一化坐标写成理想坐标乘（1 加 k1 r 平方加 k2 r 四次方），再加含 p1、p2 的切向项。标定会连同内参一起估计这些系数。校正图像时做去畸变：通过模型把每个输出像素映射到源位置。广角或鱼眼镜头需要不同的模型。检测和几何都受益于去畸变。
**追问 1**：*Does YOLO need undistorted images?* Not strictly; mild distortion is tolerated if training and deployment are consistent, but geometry such as measuring positions needs undistortion.
　中文：不严格需要；训练与部署一致时可容忍轻微畸变，但测量位置等几何计算需要去畸变。
**追问 2**：*What is the typical coefficient size?* k1 is typically a few tenths in magnitude for ordinary lenses.
　中文：普通镜头的 k1 量级通常为零点几。
**如果追问到你的项目**：*In P4 the detection used the raw frames with letterboxing, and I did not need metric geometry from the camera.*
　中文：P4 里检测使用带 letterbox 的原始帧，我不需要从相机得到度量几何。
**关键词与易错点**：radial distortion、tangential distortion、Brown–Conrady、undistortion。

### Q90. How does camera calibration with a checkerboard work?
**中文题意**：用棋盘格标定相机的原理是什么？
**考察点**：张正友标定法。
**项目关联**：概念题。
**English answer（口语）**：
> I take images of a planar checkerboard in many different poses. In each image I detect the corners to sub-pixel accuracy. Each image gives a homography between the board plane and the image. Because the board is a plane with known geometry, each homography gives two constraints on the intrinsic matrix, using the fact that the rotation columns are orthonormal; Zhang's method solves for the intrinsics linearly from three or more images. Then I estimate the extrinsics of each view, and finally refine all intrinsics, distortion coefficients and extrinsics together by minimising the reprojection error with Levenberg–Marquardt. A good result has a reprojection RMS below about half a pixel, with varied board poses covering the image.

**中文翻译**：我在许多不同姿态下拍摄平面棋盘格的图像。每幅图像中以亚像素精度检测角点。每幅图像给出棋盘平面到图像的单应。由于棋盘是已知几何的平面，每个单应利用旋转列向量正交归一的性质，对内参矩阵给出两个约束；张正友方法从三幅以上图像线性求解内参。然后估计每个视图的外参，最后用 LM 最小化重投影误差，联合精化内参、畸变系数和外参。好的结果是重投影均方根低于约半个像素，棋盘姿态多样并覆盖整个图像。
**追问 1**：*Why do I need varied poses?* Planar views that are too similar give degenerate constraints on the intrinsics.
　中文：过于相似的平面视图会使内参约束退化。
**追问 2**：*How do I check the result?* The reprojection error, the stability of the parameters across subsets, and the visual quality after undistortion.
　中文：重投影误差、不同子集上参数的稳定性，以及去畸变后的视觉质量。
**如果追问到你的项目**：*I know the method from the concept side; in P4 I used the camera without a metric calibration.*
　中文：我从概念上了解这个方法；P4 里相机没有做度量标定。
**关键词与易错点**：checkerboard、homography、Zhang's method、reprojection error、sub-pixel corners。

### Q91. What is hand–eye calibration?
**中文题意**：什么是手眼标定？
**考察点**：外参标定的经典问题。
**项目关联**：概念题；与 P2 机械臂+相机的联系。
**English answer（口语）**：
> Hand–eye calibration finds the fixed transform between a camera and a robot frame. In the eye-in-hand case, the camera is mounted on the robot arm and I find the transform from the gripper to the camera. In the eye-to-hand case, the camera is fixed in the world and I find its transform to the robot base. The classic method moves the robot to several poses while the camera observes a calibration target. For each pair of motions, I get an equation A X equals X B, where A is the gripper motion from kinematics, B is the camera motion from the target, and X is the unknown transform. Solving for X needs at least two motions with non-parallel rotation axes, and more for robustness.

**中文翻译**：手眼标定求相机与机器人坐标系之间的固定变换。眼在手上的情形，相机装在机械臂上，求夹爪到相机的变换。眼在手外的情形，相机固定在世界里，求其相对机器人基座的变换。经典方法让机器人移动到若干位姿，同时相机观测标定板。对每对运动，得到方程 A X 等于 X B，A 是来自运动学的夹爪运动，B 是来自标定板的相机运动，X 是未知变换。求解 X 至少需要两组旋转轴不平行的运动，为稳健需要更多。
**追问 1**：*Why must the rotation axes differ?* Parallel axes leave part of the transform unobservable.
　中文：平行的轴使变换的一部分不可观。
**追问 2**：*What errors dominate?* Calibration target detection, robot kinematic errors, and synchronisation between the pose and the image.
　中文：标定板检测、机器人运动学误差，以及位姿与图像的同步。
**如果追问到你的项目**：*I have not performed a hand–eye calibration; the nearest experience is aligning frames and offsets between the master and follower arms in P2.*
　中文：我没有做过手眼标定；最接近的经验是 P2 里主从臂之间的坐标系和偏置对齐。
**关键词与易错点**：hand–eye calibration、A X equals X B、eye-in-hand、eye-to-hand。诚实说明。

### Q92. How does stereo depth estimation work, and how accurate is it?
**中文题意**：双目深度估计如何工作？精度如何？
**考察点**：深度公式与误差分析。
**项目关联**：概念题。
**English answer（口语）**：
> For a rectified stereo pair with baseline B and focal length f in pixels, a point appears at horizontal positions that differ by the disparity d. The depth is Z equals f times B over d. So larger disparity means closer. Disparity is found by matching along the same image row. The depth error grows with the square of the distance: a disparity error delta d gives a depth error of about Z squared times delta d over f B. With f of 500 pixels, a baseline of 0.1 metre, and a one-pixel disparity error, at 5 metres the depth error is about 0.5 metre. So stereo is good for short range; longer baselines and higher resolution improve range accuracy.

**中文翻译**：对基线为 B、焦距为 f 像素的校正后立体图像对，同一点在两幅图中的水平位置之差是视差 d。深度 Z 等于 f 乘 B 除以 d，所以视差越大越近。视差通过沿同一图像行匹配得到。深度误差随距离平方增长：视差误差 delta d 给出约 Z 平方乘 delta d 除以 (f B) 的深度误差。f 为 500 像素、基线 0.1 米、视差误差 1 像素时，5 米处深度误差约 0.5 米。所以双目适合近距离；更长的基线和更高分辨率改善远距离精度。
**追问 1**：*What are the failure cases?* Textureless surfaces, repeated patterns, occlusions and specular surfaces.
　中文：无纹理表面、重复图案、遮挡和镜面。
**追问 2**：*What is rectification?* Warping the images so that epipolar lines become horizontal rows.
　中文：对图像做变换，使对极线成为水平行。
**如果追问到你的项目**：*I used LiDAR for range in P5, so I did not rely on stereo depth; this is conceptual.*
　中文：P5 里我用激光测距，没有依赖双目深度；这是概念知识。
**关键词与易错点**：disparity、baseline、depth error、rectification、f B over d。

### Q93. Compare stereo, structured light, time-of-flight and LiDAR as depth sensors.
**中文题意**：比较双目、结构光、飞行时间和激光雷达的深度传感。
**考察点**：传感器选型。
**项目关联**：P5、概念题。
**English answer（口语）**：
> Passive stereo is cheap and works outdoors, but needs texture and its error grows with the square of the range. Structured light projects a pattern and is accurate at close range indoors, but fails in sunlight and has short range. Time-of-flight cameras measure the travel time of modulated light per pixel, giving dense depth at moderate range, but they suffer from multipath, low resolution and sunlight. LiDAR measures range by laser time of flight with high accuracy over long range, and works in the dark and in sunlight, but is more expensive and, in 2D form, only sees a plane. For an indoor mobile robot, a 2D LiDAR is a robust choice for localisation and obstacle avoidance, with a depth camera adding 3D obstacle information.

**中文翻译**：被动双目便宜且可在室外使用，但需要纹理，误差随距离平方增长。结构光投射图案，室内近距离准确，但在阳光下失效且量程短。飞行时间相机逐像素测量调制光的传播时间，在中等量程给出稠密深度，但有多路径、低分辨率和阳光问题。激光雷达用激光飞行时间测距，长距离精度高，黑暗和阳光下都能工作，但更贵，且 2D 形态只看到一个平面。对室内移动机器人，2D 激光雷达是定位和避障的稳健选择，深度相机增加 3D 障碍信息。
**追问 1**：*Which would you pick for glass and black surfaces?* Each has issues: LiDAR can pass through glass; black surfaces reduce return for ToF and LiDAR; ultrasonic or bumpers can help.
　中文：各有问题：激光可能穿透玻璃；黑色表面降低 ToF 和激光回波；超声或碰撞传感器可作补充。
**追问 2**：*Why combine sensors?* Their failure modes differ, so fusion adds robustness.
　中文：失效模式不同，融合可增加鲁棒性。
**如果追问到你的项目**：*In P5 I used a 360-degree 2D LiDAR at 10 hertz with a range of about 12 metres, a sensor class like the LD19.*
　中文：P5 里我用 360° 2D 激光雷达，10 Hz，量程约 12 m，类似 LD19 级。
**关键词与易错点**：stereo、structured light、time-of-flight、LiDAR、multipath、failure modes。

### Q94. For a new indoor robot, would you choose LiDAR SLAM or visual SLAM, and why?
**中文题意**：对一台新的室内机器人，你选激光 SLAM 还是视觉 SLAM？为什么？
**考察点**：工程选型判断。
**项目关联**：P5。
**English answer（口语）**：
> It depends on the requirements, but for a wheeled indoor robot that needs reliable navigation, I would start with 2D LiDAR SLAM. It gives metric scale, works in changing light, is accurate, and the algorithms and the Nav2 integration are mature. Visual SLAM is cheaper in hardware, gives rich semantic information, and works in 3D, but is sensitive to lighting, texture, motion blur and needs more computation and careful calibration. For robots that need 3D understanding or to work in environments where a 2D plane is not enough, I would add a depth camera or a 3D LiDAR, or use visual–inertial SLAM. A fusion of the two is a strong design.

**中文翻译**：取决于需求，但对需要可靠导航的室内轮式机器人，我会从 2D 激光 SLAM 开始。它给出度量尺度，在光照变化下可用，精度高，算法与 Nav2 集成成熟。视觉 SLAM 硬件更便宜，提供丰富的语义信息，并在 3D 中工作，但对光照、纹理和运动模糊敏感，需要更多计算和细致标定。对需要 3D 理解或 2D 平面不够的环境，我会增加深度相机或 3D 激光雷达，或使用视觉惯性 SLAM。两者融合是很强的设计。
**追问 1**：*What are the cost considerations?* LiDARs are more expensive than cameras, but have dropped in price; development effort and robustness matter as much.
　中文：激光雷达比相机贵，但价格已下降；开发工作量和鲁棒性同样重要。
**追问 2**：*What about outdoor use?* Sunlight and weather affect cameras and LiDAR differently; GNSS is added for outdoor robots.
　中文：阳光和天气对相机与激光影响不同；室外机器人会加入 GNSS。
**如果追问到你的项目**：*This is the choice I made in P5: 2D LiDAR with IMU and wheel odometry, using the Nav2 stack on ROS2 Humble.*
　中文：这就是我在 P5 的选择：2D 激光加 IMU 和轮里程计，使用 ROS2 Humble 上的 Nav2 栈。
**关键词与易错点**：LiDAR SLAM、visual SLAM、metric scale、robustness、sensor fusion。

---

## 7. 自测与练习安排

### 7.1 自测题（答案见各题）

1. 写出旋转矩阵的两个性质，并说出齐次变换求逆的公式。（Q1、Q4）
2. 写出平面二连杆的雅可比，并说明它在何处奇异。（Q16：q2 为 0 或 pi）
3. 解释为什么重力补偿等于零速度、零加速度的逆动力学。（Q32）
4. 手算一步标量卡尔曼：先验 0、方差 4，测量 2、方差 1。（Q45：K 为 0.8，估计 1.6，方差 0.8）
5. 互补滤波里 alpha 为 0.98、dt 为 10 ms，时间常数是多少？（Q51：约 0.49 s）
6. 为什么 yaw 会漂移？有哪些办法？（Q56）
7. AMCL 的预测、权重、重采样各做什么？你 P5 的粒子数范围是多少？（Q49：500–3000）
8. 双目 f 为 500 像素、基线 B 为 0.1 m、视差 10 像素，深度 Z 是多少？（Q92：5 m）
9. 激光时间戳的两种约定会造成什么问题？你怎么修？（Q11、Q75）
10. 回环检测为什么重要？怎么防止错误回环？（Q69）

### 7.2 三轮练习安排

| 轮次 | 目标 | 做法 | 用时 |
|---|---|---|---|
| 第一轮：读懂 | 理解 94 题的概念和公式 | 每天 10–12 题；先读中文翻译，再读英文；A、B 两节先于其他 | 约 9 天 |
| 第二轮：开口 | 60–90 秒讲完一个问题 | 遮住英文，只看中文题意，计时说一遍并录音；重点练 Q1、Q4、Q5、Q16、Q17、Q20、Q44–Q52、Q56、Q60、Q67–Q69、Q88、Q92 | 约 8 天 |
| 第三轮：追问 | 接两层追问并转到项目 | 随机抽题，对方问 "Why?" "What number?" "What did you do yourself?"；被问到没做过的内容时用诚实模板 | 约 6 天 |

### 7.3 优先级提醒

- 最高频的 15 题：Q1、Q4、Q5、Q16、Q17、Q20、Q29、Q30、Q44、Q45、Q47、Q49、Q51、Q67、Q88。
- 与你项目绑定最紧、最容易被深挖的 12 题：Q5（P5 坐标系分工）、Q9、Q11（P5 时间戳）、Q17、Q20（P2 奇异与阻尼）、Q21（P7 五连杆）、Q35（P1 建模假设）、Q47–Q50（P5 EKF 与 AMCL）、Q51–Q52（互补滤波）、Q55（偏置标定）、Q60–Q61（P6 时间同步与有效性）、Q62（robot_localization）。
- 红线再提醒：P1 位置只靠轮速里程计、无外部定位；P3 磁力计未启用、AHRS 用 Mahony；P5 用 ROS2 Humble；P6 的数字是设计取值；视觉 SLAM 和 VIO 你只在概念层面了解，必须诚实说明。
- 数字被追问到精确值时用诚实模板："I don't remember the exact number, but it was roughly …"。

### 7.4 相关链接

- 基础教材：[运动学与坐标变换 README](../../02_基础知识/03_运动学与坐标变换/README.md)　[状态估计 README](../../02_基础知识/06_状态估计/README.md)　[SLAM 与建图 README](../../02_基础知识/07_SLAM与建图/README.md)　[感知与视觉 README](../../02_基础知识/09_感知与视觉/README.md)　[力学与动力学建模 README](../../02_基础知识/02_力学与动力学建模/README.md)　[机器人动力学进阶：操作空间、接触、阻抗](../../02_基础知识/02_力学与动力学建模/05_机器人动力学进阶_操作空间_接触_阻抗.md)
- 项目文档：[P1](../../01_项目课程/P1_AIRS球形机器人平衡控制/README.md)　[P2](../../01_项目课程/P2_未来动力双臂遥操作与差速底盘/README.md)　[P3](../../01_项目课程/P3_RoboMaster英雄机器人嵌入式控制/README.md)　[P5 EKF 与 AMCL 协同](../../01_项目课程/P5_车式移动机器人自主导航/06_EKF与AMCL协同_TF分工与连续状态.md)　[P5 传感器与数据质量](../../01_项目课程/P5_车式移动机器人自主导航/03_传感器与数据质量_时间坐标与运动方向.md)　[P6 机器人状态管理](../../01_项目课程/P6_深开鸿M-Robots平台软件/05_机器人状态管理.md)　[P7 五连杆闭链运动学](../../01_项目课程/P7_可变腿长轮腿机器人/02_五连杆闭链运动学.md)
- 面试准备：[港硕面试真题调研](../00_港硕面试真题调研.md)　[高频 100 问](../04_模拟与冲刺/01_高频100问_英文速答.md)　[跨项目技术对比问答](../03_项目口述总览/02_跨项目技术对比问答.md)　[CV 事实基线](../../06_维护说明/CV事实基线.md)
- 同一分类题库：[10 数学、优化与概率](./10_题库_数学_优化_概率.md)　[11 控制理论与机器人控制](./11_题库_控制理论与机器人控制.md)　[13 规划、导航与感知](./13_题库_规划_导航_感知.md)　[14 机器学习、深度学习、强化学习与大模型](./14_题库_机器学习_深度学习_强化学习_大模型.md)
