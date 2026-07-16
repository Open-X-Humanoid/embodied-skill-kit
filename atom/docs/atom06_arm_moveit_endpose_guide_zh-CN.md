# 原6 · 手臂（Arm）· XARM MoveIt 末端（笛卡尔）控制

[English](atom06_arm_moveit_endpose_guide.md) | **简体中文**

**一句话**：给 **MoveIt** 一个"末端位姿"目标——手要去空间中的哪个点（x/y/z）、以什么朝向（四元数）——它反解 IK、规划无碰撞轨迹并执行。和 atom05 的区别只有一处：**atom05 给关节角，atom06 给末端位姿**；其余（使能、切控制器、发 `/move_action`）完全一样。

| 配套 | 路径 |
|---|---|
| 代码（简洁版） | `atom/demos/atom06_arm_moveit_endpose.py` |
| 演示视频 | 与配套代码同名，见 `atom/assets/videos/` |

> 本原子暂无 `_robust` 版。遇控制器冲突 / `CONTROL_FAILED(-4)`，参考 `atom/demos/atom05_arm_moveit_robust.py` 的做法（严格切换 + 避让占用本臂的控制器），套路可直接搬过来。

> 建议先读 atom05：使能、切控制器、`/move_action` 这三件事两个原子共用，本文不再重复展开。

## 1. 速览

### 1.1 跑起来（和 atom05 完全相同的前提）

- **板子**：**x86、ubuntu 用户**（和 body_control 同板；XARM 装在 `/home/ubuntu/XARM`）。
- ★**跑前 source 的是 XARM，不是 ros2ws**：`source /home/ubuntu/XARM/install/setup.bash`（`moveit_msgs`、`tf2_ros` 都在这）。
- **真机 SOP 共 5 步**（缺任一步臂不动）：起 body_control → 起 XARM 本体 → 起 MoveIt 组件 → **使能手臂** → **切 MoveIt 控制器** →（跑 demo 下发）。后两步 demo 已自动做。

**一键前置**（推荐，见 `scripts/start_xarm.sh`）：

```bash
# 先仿真验证（零风险，不接真机/不需 body_control；用 RViz 或看 /joint_states）
bash scripts/start_xarm.sh sim

# 真机（先在 x86 起 body_control）
bash scripts/start_body_control.sh          # 另开终端，见《前置 · 环境配置》
bash scripts/start_xarm.sh real

# 跑 demo 的终端配环境后运行
source scripts/start_xarm.sh                # = source /home/ubuntu/XARM/install/setup.bash
python3 atom/demos/atom06_arm_moveit_endpose.py
```

demo 做的事：读当前末端 TCP 位姿 → **保持姿态不变、只让末端上抬 5cm** → 再回到起始位姿。可复位、低速。

> **强烈建议先 sim**：末端控制比关节控制更容易撞到自己（同一个末端点可能对应多组关节解）。ROS 基础环境见《前置 · 环境配置》(`atom/docs/environment_setup_zh-CN.md`)。

### 1.2 接口（比 atom05 多一个 TF）

| 接口 | 类型 | 作用 |
|---|---|---|
| `/move_action` | `moveit_msgs/action/MoveGroup`（**Action**） | 送**末端位姿约束**，MoveIt 反解 IK、规划并执行 |
| `/EAIHardware/set_arm_enable` | `std_srvs/SetBool`（Service） | real 模式使能手臂（sim 无此服务） |
| `/controller_manager/switch_controller` | ros2_control（Service） | 激活 `moveit_left_arm_controller` |
| **`/tf`、`/tf_static`** | `tf2_msgs/TFMessage`（Topic） | **读当前末端 TCP 位姿作起点**（atom05 用 `/joint_states`，这里用 TF） |

> 为什么起点要改用 TF：atom05 的起点是"7 个关节角"，`/joint_states` 直接给。atom06 的起点是"末端在空间的位姿"——关节角要经**正运动学**才能算出末端在哪，而 TF 树已经由 `robot_state_publisher` 实时算好并发布了，直接查即可，不用自己算。

### 1.3 ★两个坐标系（atom06 的头号坑）

末端控制的一切都建立在两个 frame 上，**名字写错 demo 直接超时退出**：

| 常量 | 值 | 是什么 |
|---|---|---|
| `BASE_FRAME` | **`base`** | 参考基坐标系。末端位姿"相对谁"而言 |
| `EE_LINK` | **`left_tcp_link`** | 左臂末端 TCP（工具中心点）link，就是"手要去哪"的那个点 |

⚠ **`base` 不是 `base_link`**。很多 ROS 教程里根 link 习惯叫 `base_link`，但天轶 2.0 的 URDF 里**根本没有 `base_link` 这个 frame**。实际结构是：

```
base_footprint          ← URDF 的真正根 link
    │  world_to_base_link（fixed，偏移全零 xyz="0 0 0" rpy="0 0 0"）
    ↓
base                    ← 本 demo 用的 BASE_FRAME
```

`base_footprint` 与 `base` 之间是**零偏移的固定关节**，两个 frame 在空间上完全重合——所以用哪个都算得对，demo 取 `base`。写成 `base_link` 则会报：

```
Invalid frameID "base_link" passed to canTransform argument target_frame - frame does not exist
```

跑之前先自己核一遍（应打印出平移/旋转数值，而不是报错）：

```bash
ros2 run tf2_ros tf2_echo base left_tcp_link
```

想看完整 TF 树（根 frame 叫什么、末端 frame 叫什么，一目了然）：

```bash
ros2 run tf2_tools view_frames        # 生成 frames.pdf
```

> 右臂把 `left_tcp_link` 换成 `right_tcp_link`；`BASE_FRAME` 不变（同一个根）。

### 1.4 限位与可达性（末端控制的"限位"长什么样）

关节限位仍然生效，只是**换了个方式拦你**：atom05 里你直接给关节角、超限被拒；atom06 里你给的是末端位姿，**MoveIt 先反解 IK**，如果算出的关节角超限、或这个点根本够不着，就返回 `NO_IK_SOLUTION(-31)`——**不是"限位报错"，而是"无解"**。

- 7 个关节的具体限位表见《原5 · 手臂 · XARM MoveIt 关节运动》(`atom/docs/atom05_arm_moveit_guide_zh-CN.md`) 的 1.3 节，此处不重复。
- **末端能到的空间叫工作空间**（workspace），大致是以肩为中心、以臂展为半径的一块壳形区域，且受关节限位裁剪——**不是一个规整的球**。
- 实用判据：**从当前位姿小步增量**（demo 默认 5cm），比凭空指定一个绝对坐标靠谱得多。够不着时先把臂移到目标附近再规划。

## 2. 四件必做（比 atom05 多一步"读 TF"）

### 2.1 使能手臂（真机头号坑，同 atom05）

**XARM 的"使能"不是电机上电，而是"XARM 要不要向 body_control 发指令"。** 不使能，MoveIt 报成功但物理臂纹丝不动。demo 里 `enable_arm()` 自动做；手动等价：

```bash
ros2 service call /EAIHardware/set_arm_enable std_srvs/srv/SetBool "{data: true}"
```

### 2.2 切 MoveIt 控制器（同 atom05）

执行前必须激活 `moveit_left_arm_controller`，否则 `error_code=-4 CONTROL_FAILED`。demo 里 `activate_moveit_controller()` 自动切。

### 2.3 ★读当前末端位姿（atom06 新增）

`read_ee_pose()` 用 TF 查 `base → left_tcp_link`，拿到当前 TCP 的位置 + 姿态作为起点。读不到就**返回 None 并拒绝运动**——这是安全设计：不知道手在哪，就绝不能动。

### 2.4 发位姿目标（和 atom05 的真正差异）

atom05 给 7 个 `JointConstraint`；atom06 给**一个位置约束 + 一个姿态约束**，都作用在 `EE_LINK` 上：

| 约束 | 类型 | demo 的容差 | 含义 |
|---|---|---|---|
| 位置 | `PositionConstraint` | 半径 **0.01m** 的球 | TCP 落进这个小球里就算到位 |
| 姿态 | `OrientationConstraint` | **0.05 rad ≈ 2.9°** | 三轴各自允许的角度偏差 |

> 位置约束用一个 `SolidPrimitive.SPHERE` 表达"容差区域"——这是 MoveIt 的通用写法：目标不是一个数学上的点，而是**一小块可接受区域**。容差调太小会规划不出来，调太大会不准。

## 3. 代码解读（核心）

| 模块 | 代码锚点 | 职责 | 换右臂要改？ |
|---|---|---|---|
| 使能 | `enable_arm()` → `/EAIHardware/set_arm_enable` | real 模式开"发指令的门" | 不变 |
| 切控制器 | `activate_moveit_controller()` → `switch_controller` | 激活 moveit 控制器 | 改 `MOVEIT_CONTROLLER` |
| **读起点** | `read_ee_pose()` → TF `base → left_tcp_link` | 拿当前末端位姿作起点，读不到拒动 | 改 `EE_LINK` |
| **建约束** | `_pose_goal()` → Position + Orientation Constraint | 把一个 Pose 翻成 MoveIt 的约束 | 改 `EE_LINK` |
| 规划执行 | `move_to_pose()` → `MoveGroup` action | 发 /move_action、查 error_code | 改 `GROUP` |

**逐模块一句话**：

- `read_ee_pose()`：循环 `spin_once` + `lookup_transform`，5 秒超时返回 None。用 `Time()`（零时刻）表示"最新可用的变换"。
- `_pose_goal()`：位置约束填一个球形容差区（球心 = 目标点），姿态约束直接填目标四元数 + 三轴容差。注意球体本身的朝向填了单位四元数 `Quaternion(w=1.0)`——球是各向同性的，朝向无意义，填什么都行，但字段不能空。
- `move_to_pose()`：建 `MotionPlanRequest`（组名、规划次数、限速）→ 塞约束 → `plan_only=False`（规划完直接执行）→ 阻塞等 result → `error_code == 1` 为成功。
- `main()`：读起点 → `copy.deepcopy` 出目标、**只加平移不动姿态** → 去 → 回。

**举一反三**：

- **换右臂**：`GROUP="right_arm"` + `EE_LINK="right_tcp_link"` + `MOVEIT_CONTROLLER="moveit_right_arm_controller"`。`BASE_FRAME` 不变。
- **换朝向**：demo 只平移不转。要改朝向就替换 `target.orientation`（填目标四元数）。手搓四元数容易错，实践中常用 `tf_transformations.quaternion_from_euler(r, p, y)` 从 RPY 转。
- **走一串路点**：连续调 `move_to_pose()` 即可，但每段是独立规划（段间会减速到 0）。要真正连贯的直线/圆弧轨迹，见第 6 节的 Cartesian Path。

## 4. 和 atom05 的区别

| | atom05（关节空间） | atom06（末端 / 笛卡尔） |
|---|---|---|
| 你给什么 | 7 个关节角 | 末端位姿（x/y/z + 四元数） |
| 起点从哪读 | `/joint_states` | **TF**（`base → left_tcp_link`） |
| 目标怎么表达 | 7 个 `JointConstraint` | 1 个 `PositionConstraint` + 1 个 `OrientationConstraint` |
| MoveIt 多做了什么 | 直接规划 | **先反解 IK**，再规划 |
| 失败新增模式 | 超限 | **`NO_IK_SOLUTION(-31)`**（够不着 / 无解） |
| 适合 | "我知道每个关节要转到多少" | **"我知道手要去哪"**（更贴近抓取、放置等真实任务） |

> 底层完全相同：两者都发 `/move_action` → `moveit_left_arm_controller` → XARM → `/arm/cmd_*` → body_control → 电机。**差别只在"目标怎么描述"**，不在执行链路。

**什么时候用哪个**：手要对准一个物体（抓箱、放料）用 atom06——你知道物体在哪，不知道该转多少关节；做固定姿态切换（回 home、举手示意）用 atom05——关节角是确定的，还省掉 IK 这一步。

## 5. 排错

| 现象 | 原因 | 处理 |
|---|---|---|
| `Invalid frameID "xxx" ... frame does not exist` | frame 名写错（**根 link 是 `base` 不是 `base_link`**） | `ros2 run tf2_ros tf2_echo base left_tcp_link` 核实；改 `BASE_FRAME`/`EE_LINK` |
| `超时未读到 ... 位姿（TF）` | frame 名错，或 TF 没人发（MoveIt/robot_state_publisher 没起） | 先 `ros2 topic list \| grep tf` 确认 TF 在；再核 frame 名 |
| `error_code=-31` NO_IK_SOLUTION | 目标够不着 / IK 解超关节限位 | 减小平移量；先把臂移到目标附近；对照 atom05 限位表 |
| `error_code=-18` INVALID_LINK_NAME | `EE_LINK` 不是 URDF 里的 link | 核 `left_tcp_link` 拼写 |
| `error_code=-21` FRAME_TRANSFORM_FAILURE | `BASE_FRAME` 不是有效 frame | 同第一行 |
| `error_code=-1` PLANNING_FAILED | 规划不出无碰撞路径 | 容差放宽一点；换个起始姿态；加大 `allowed_planning_time` |
| `error_code=-4` CONTROL_FAILED | moveit 控制器没激活 | demo 已自动切；手动 `ros2 control switch_controllers --activate moveit_left_arm_controller` |
| 规划执行报成功，但臂不动 | 没使能（real 头号坑）；或在 sim | `set_arm_enable true`；sim 看 RViz |
| `error_code=-15` INVALID_GROUP_NAME | 规划组名不对 | `ros2 param get /move_group robot_description_semantic` 核 SRDF |

自查三条：

```bash
ros2 run tf2_ros tf2_echo base left_tcp_link          # frame 通不通
ros2 control list_controllers | grep -i moveit        # 控制器 active？
ros2 action list | grep move_action                   # MoveIt 组件起了吗
```

## 6. 进阶

- **Cartesian Path（真直线）**：本 demo 是"给个末端目标点，MoveIt 自由规划过去"——中间路径**不保证是直线**。要末端严格走直线/圆弧（如插销、拖拽、贴面），用 MoveIt 的 `/compute_cartesian_path` 服务（给一串 waypoints，返回一条笛卡尔插值轨迹），再单独送给控制器执行。
- **姿态自由度放宽**：抓圆柱物体时绕自身轴的旋转往往无所谓——把 `OrientationConstraint` 对应轴的容差调到 `3.14`（等于不约束），能大幅提高 IK 成功率。这是真实项目里最常用的调参。
- **加避障物体**：往 MoveIt 的 planning scene 里加 `CollisionObject`（如箱子、桌面），规划就会自动绕开。这是 MoveIt 相对裸控最大的价值。
- **arm_mode（硬件工作模式）**：`0` 柔顺(力位混合，默认) / `3` 高刚度位置环 / `1` 高刚度速度环(需实时内核) / `2` 重力补偿。切：`ros2 service call /EAIHardware/set_arm_mode eai_manipulator_msgs/srv/Mode "{mode: 3}"`。
