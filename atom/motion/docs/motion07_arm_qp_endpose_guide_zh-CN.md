# 运控7 · 手臂（Arm）· XARM QP 末端（笛卡尔）控制

[English](motion07_arm_qp_endpose_guide.md) | **简体中文**

**一句话**：给 XARM 的 **QP 末端控制器**（`endpose_single_arm_qp_L_controller`）一个"末端位姿"目标，它做**局部响应式跟踪**把 TCP 送过去，自带防碰撞。和 motion05 的区别：**motion05 是 MoveIt"一次规划整条轨迹"，motion07 是 QP"在线小步跟踪"**——目标离当前太远会被直接拒绝（误差限），只能小幅增量喂。

| 配套 | 路径 |
|---|---|
| 代码（简洁版） | `atom/motion/motion07_arm_qp_endpose.py` |
| 演示视频 | 与配套代码同名，见 `atom/motion/assets/videos/` |

> 建议先读 motion05（末端控制的坐标系/TF 读起点）和 motion06（QP 的使能/切控制器/互斥）；本文只展开 motion07 独有的部分。

## 1. 速览

### 1.1 跑起来（同 motion06——QP 不需要 MoveIt 组件）

- **板子**：**x86、ubuntu 用户**；★跑 demo 需 `source /home/ubuntu/XARM/install/setup.bash`（`eai_manipulator_msgs` 在 XARM 里）。

```bash
# ⓪ 停遥控服务（占 /arm/cmd_pos，不停则使能失败；重启机器后自动恢复）
sudo systemctl stop teleop_robot

# 真机：body_control → XARM 本体（不需要 MoveIt 组件）
bash scripts/start_body_control.sh
bash scripts/start_xarm.sh real

# 跑 demo
source /home/ubuntu/XARM/install/setup.bash
python3 atom/motion/motion07_arm_qp_endpose.py
```

demo 做的事：切 QP 末端控制器 → **压速**（`vel_limits` 设 0.5 rad/s）→ TF 读当前 TCP 位姿 → **姿态不变、末端上抬 5cm** → 回起始位姿。

> 本控制器**自带防碰撞**（身体 2 球 + 手臂 7 球模型），比 motion06 的关节控制器安全一档；但仍需臂周围无人无物、急停在手。

### 1.2 接口

| 接口 | 类型 | 作用 |
|---|---|---|
| `/endpose_single_arm_qp_L_controller/endPosSingleTarget` | `eai_manipulator_msgs/action/EndPosSingleTarget`（**Action**） | 送末端位姿目标（`ArmTargetPose`），到位后返回 `success` |
| `/endposetarget_L` | `eai_manipulator_msgs/msg/ArmTargetPose`（Topic，流式） | 连续喂位姿的流接口——与 action **二选一** |
| `/endpose_single_arm_qp_L_controller/set_parameters` | `rcl_interfaces/SetParameters`（Service） | **代码级调速**（改 `vel_limits`，见 2.3） |
| `/EAIHardware/set_arm_enable` 等 | `std_srvs/SetBool`（Service） | real 模式使能（demo 自动探测候选名） |
| `/controller_manager/switch_controller` | ros2_control（Service） | 激活本控制器（自动停占用本臂的其它控制器） |
| TF `base → left_tcp_link` | Topic | 读当前末端 TCP 位姿作起点 |

### 1.3 ★目标怎么表达：ArmTargetPose（QP 特有）

不用 MoveIt 那套约束——一个消息说清"谁到谁的期望位姿"：

| 字段 | demo 的值 | 含义 |
|---|---|---|
| `from_frame` | `base` | 参考系（XARM 手册对天轶的建议值；`base` 与根 `base_footprint` 零偏移重合，见 motion05 guide 1.3） |
| `to_frame` | `left_tcp_link` | 末端 TCP link |
| `target` | 目标 Pose | **从 from_frame 到 to_frame 的期望位姿** |
| `offset_x/y/z` | 0 | 末端系下的参考点偏移（如工具尖端），demo 不用 |

### 1.4 ★误差限：目标太远直接拒绝（和 motion05 最大的不同）

本控制器是**局部**跟踪器：期望位姿必须离当前末端足够近（距离 ≤ `dis_err_bound`、角度 ≤ `ori_err_bound`），否则**拒绝执行并报警、保持不动**。所以只能"小步喂"——demo 一步 5cm 上抬。想走远距离就分多步，每步都从新的当前位姿出发。

跑前可查当前误差限：

```bash
ros2 param get /endpose_single_arm_qp_L_controller dis_err_bound
ros2 param get /endpose_single_arm_qp_L_controller ori_err_bound
```

> 对照 motion05：MoveIt 给多远都行（它整条规划）；QP 给远了直接拒。这不是缺陷，是"响应式跟踪器"的设计——它假设上游（如视觉）在高频喂邻近目标。

## 2. 核心操作

### 2.1 使能 + 切控制器（同 motion06,不重复）

`enable_arm()` 多候选探测 + `_scan_arm_controllers()` 自动停占用本臂的控制器 + STRICT 切换。

### 2.2 发位姿目标

`move_to_pose()` 填 `ArmTargetPose`（from/to/target/offset）→ 发 action → 阻塞等 `result.success`。**本 action 是等到位才返回的**（按 `dis_threshold`/`ori_threshold`/`step_threshold` 停止），与 motion06 的"返回≠到位"不同——末端版不需要额外的 wait_reached。

### 2.3 ★代码级调速（QP 与 MoveIt 的关键区别）

MoveIt 的速度在**下发时**给（`max_velocity_scaling_factor`）；**QP 的速度是控制器的常驻参数** `vel_limits`。demo 的 `set_vel_limits()` 通过参数服务在切完控制器后把它压到 `[0.5]×7 rad/s`（越小越慢），等价于命令行：

```bash
ros2 param set /endpose_single_arm_qp_L_controller vel_limits '[0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5]'
```

改 `VEL_LIMITS` 常量即可调快慢；设 `None` 则不改、用控制器当前值。⚠ 只往小调是安全方向。

## 3. 代码解读（核心）

| 模块 | 代码锚点 | 职责 | 换右臂要改？ |
|---|---|---|---|
| 使能 | `enable_arm()` + `_already_enabled()` | 多候选探测 + `arm_enable` 兜底 | 不变 |
| 查冲突/切控制器 | `_scan_arm_controllers()` + `activate_qp_controller()` | 停占用本臂者 + STRICT 激活 | 改 `QP_CONTROLLER`（L→R） |
| **调速** | `set_vel_limits()` → `set_parameters` 服务 | 把 `vel_limits` 压到安全速度 | 不变 |
| 读起点 | `read_ee_pose()` → TF `base → left_tcp_link` | 当前 TCP 位姿作起点，读不到拒动 | 改 `TO_FRAME` |
| 发目标 | `move_to_pose()` → `EndPosSingleTarget` action | 填 ArmTargetPose、等到位 | 改 `TO_FRAME` |

**举一反三**：换右臂 = `QP_CONTROLLER` 的 `L`→`R` + `TO_FRAME="right_tcp_link"`；`FROM_FRAME` 不变。要"沿工具尖端对准"就填 `offset_x/y/z`（末端系下的偏移），目标点语义变成"让工具尖端到那儿"。

## 4. 和 motion05（MoveIt 末端）的区别

| | motion05（MoveIt） | motion07（QP） |
|---|---|---|
| 工作方式 | 反解 IK + 离线规划整条轨迹 | 在线小步跟踪（局部优化） |
| 前置 | XARM 本体 + **MoveIt 组件** | 只要 XARM 本体 |
| 目标表达 | Position + Orientation Constraint | 一个 `ArmTargetPose` 消息 |
| 目标距离 | 多远都行（规划过去） | **必须邻近**（超误差限直接拒绝） |
| 失败模式 | `NO_IK_SOLUTION(-31)` 等 error_code | 拒绝执行 + 报警、保持不动 |
| 防碰撞 | planning scene 避障 | 内置球模型防碰撞 |
| 调速 | 下发时 `max_velocity_scaling_factor` | 控制器参数 `vel_limits` |
| 适合 | 一次性"手去那儿"（抓取/放置定点） | 连续跟踪（视觉伺服、遥操作跟随） |

## 5. 排错

| 现象 | 原因 | 处理 |
|---|---|---|
| 目标被拒绝 / 报警不动 | **超误差限**（一步给太远） | 减小 `DEMO_DELTA_XYZ`；查 `dis_err_bound`；分多步走 |
| `超时未读到 ... 位姿（TF）` | frame 名错（**是 `base` 不是 `base_link`**）或 TF 没人发 | `ros2 run tf2_ros tf2_echo base left_tcp_link` 核实 |
| QP action 连不上（10s 超时） | 控制器没激活 / 本臂被别族占着 | `ros2 control list_controllers`；demo 已自动避让 |
| 动作太快 | `vel_limits` 是控制器参数 | demo 已内置压速；再调小 `VEL_LIMITS` 或命令行 `ros2 param set` |
| `设 vel_limits 未成功` | 该控制器可能不允许运行时改参 | 退回命令行 `ros2 param set`；仍不行则接受默认速度、减小单步 |
| 使能相关报错 | 与 motion04/07 同源 | 见《原5》排错表；`sudo systemctl stop teleop_robot` |
| 发 topic 没反应 | action 与 topic 二选一 | 停 action 客户端再发流 |

## 6. 进阶

- **姿态自由度放宽**：参数 `OriWeight: [x, y, z]`（0.0~1.0）可把某轴姿态权重设 0——做 5 自由度（直线位姿）/4 自由度（平面位姿）控制，配合工具坐标系用。抓圆柱体时放开绕轴旋转能显著提高可达性。
- **冗余臂角**：7 自由度臂到同一末端位姿有无数种"肘部朝向"。`redundant_degrees`（肩 yaw 参考角）+ `redundant_degrees_weight`（0~500，越大越贴参考臂角、末端精度略降）可控制零空间臂角——躲障碍、摆好看的肘位都靠它。
- **流式控制**：改用 `/endposetarget_L` topic 高频喂位姿（视觉伺服/遥操作跟随的完全体）。先停 action 客户端。
- **额外避障球**：`collision_freeBall_pos`（x, y, z, r）可在空间里加一个自由碰撞球，让臂绕开指定区域——比 MoveIt planning scene 轻量的"临时禁区"。
- **双臂**：双臂协同末端控制见 `endpose_dual_arm_qp_controller`（本教学库暂未覆盖，接口同族）。
