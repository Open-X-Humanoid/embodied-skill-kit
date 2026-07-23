# 原5 · 手臂（Arm）· XARM MoveIt 关节运动

[English](atom05_arm_moveit_guide.md) | **简体中文**

**一句话**：让 **MoveIt** 帮你规划，把一条 7 自由度手臂平滑运动到指定关节角——不是自己硬发角度（那是 atom04），而是"告诉它去哪，它自己算一条无碰撞轨迹再执行"。本原子用**标准 MoveIt2 的 `/move_action`**（不依赖工程封装），执行经 XARM 的 `moveit_*_arm_controller`。

| 配套 | 路径 |
|---|---|
| 代码（简洁版） | `atom/demos/atom05_arm_moveit.py` |
| 代码（生产版 robust） | `atom/demos/atom05_arm_moveit_robust.py`（切控制器前自动避让占用本臂的控制器 + 严格超时校验） |
| 演示视频 | 与配套代码同名，见 `atom/assets/videos/` |

> 简洁版看懂原理（假设 XARM 刚起、本臂控制器无冲突）；真机遇"控制器冲突 / CONTROL_FAILED(-4)"用 robust 版。

> 和 atom04 的关系见第 4 节：atom04 是"你亲手发角度"，atom05 是"MoveIt 规划后由 XARM 发"，底层殊途同归。

## 1. 速览

### 1.1 跑起来（前提比 atom04 重，务必按顺序）

- **板子**：**x86、ubuntu 用户**（和 body_control 同板；XARM 装在 `/home/ubuntu/XARM`）。
- ★**跑 demo 只需基础 ROS 2**（`/opt/ros/humble`，`~/.bashrc` 已自动 source）：demo 只 import 标准消息包（`moveit_msgs`、`controller_manager_msgs`、`std_srvs`、`sensor_msgs` 都在基础 ROS 里），**不需要 source XARM**。source XARM（`/home/ubuntu/XARM/install`，含 `tianyi2_bringup`）只用于**启动** XARM 本体+MoveIt，`start_xarm.sh` 已自动做；个别机器若 `import moveit_msgs` 失败再手动 `source /home/ubuntu/XARM/install/setup.bash`。
- **真机官方 SOP 共 5 步**（缺任一步臂不动）：起 body_control → 起 XARM 本体 → 起 MoveIt 组件 → **使能手臂** → **切 MoveIt 控制器** →（跑 demo 下发）。后两步 demo 已自动做。

**一键前置**（推荐，见 `scripts/start_xarm.sh`）：

```bash
# ⓪ 停遥控服务（开机自启、占 /arm/cmd_pos，不停则使能会失败；重启机器自动恢复）
#   已停止时再执行也无害，每次开机后跑一遍即可。想先确认状态（可选）：
#   systemctl is-active teleop_robot   # active=在跑，需停；inactive=已停；报"找不到该服务"=本机没有它（旧系统），跳过本步
sudo systemctl stop teleop_robot

# 真机（已验证）：先在 x86 起 body_control，再起 XARM + MoveIt
bash scripts/start_body_control.sh          # 另开终端，见《前置 · 环境配置》
bash scripts/start_xarm.sh real

# 跑 demo（基础 ROS 已自动 source，直接跑；无需 source XARM）
python3 atom/demos/atom05_arm_moveit.py
```

> ⓪ 的原因：遥控器调度服务 `teleop_robot` 开机自启并注册为 `/arm/cmd_pos` 发布者，与"程序控手臂"互斥（XARM 使能门检测到占用会拒绝）。只影响手臂类原子；头/语音/相机/底盘不受影响。想用遥控器时 `sudo systemctl start teleop_robot` 或重启机器即可。

> **仿真模式**（`bash scripts/start_xarm.sh sim`，带 RViz、不接真机/不需 body_control）：无真机时可用它先看规划/执行效果。⚠ 本项目未在 sim 下实测，命令理论可用、以你机器为准；已验证的是 real。ROS 基础环境见《前置 · 环境配置》(`atom/docs/environment_setup_zh-CN.md`)。

### 1.2 接口（一个 action + 两个 service）

| 接口 | 类型 | 作用 |
|---|---|---|
| `/move_action` | `moveit_msgs/action/MoveGroup`（**Action**） | 送目标关节角，MoveIt 规划并执行 |
| `/moveit_controller_enable`（旧版为 `/EAIHardware/set_arm_enable`） | `std_srvs/SetBool`（Service） | real 模式使能手臂；服务名随 XARM 版本变，demo 运行时自动探测（sim 无此服务） |
| `/controller_manager/switch_controller` | ros2_control（Service） | 激活 `moveit_left_arm_controller` |
| `/joint_states` | `sensor_msgs/JointState`（Topic） | 读当前关节角作运动起点 |

> 一个关键概念：**Action = 长任务**（下 goal → 有进度 feedback → 拿 result，可取消）。运动要几秒、可能失败，所以 MoveIt 用 action，不是一次性的 service。

### 1.3 关节限位（左臂 7 关节，以 URDF 为准）

MoveIt 会**拒绝超限的目标**（返回 `INVALID_GOAL/规划失败`）。改目标角前先对照：

| 关节 | 关节名 | 硬限位(rad) | ≈度数 |
|---|---|---|---|
| J1 肩·俯仰 | `shoulder_pitch_l_joint` | −2.967 ~ +2.967 | ≈±170° |
| J2 肩·侧展 | `shoulder_roll_l_joint` | −0.262 ~ +2.618 | ≈−15°~+150° |
| J3 肩·旋转 | `shoulder_yaw_l_joint` | −2.967 ~ +2.967 | ≈±170° |
| J4 肘·弯曲 | `elbow_pitch_l_joint` | −2.618 ~ +0.262 | ≈−150°~+15° |
| J5 肘·旋转 | `elbow_yaw_l_joint` | −2.967 ~ +2.967 | ≈±170° |
| J6 腕·俯仰 | `wrist_pitch_l_joint` | −0.785 ~ +1.047 | ≈−45°~+60° |
| J7 腕·旋转 | `wrist_roll_l_joint` | −1.658 ~ +1.309 | ≈−95°~+75° |

> 右臂把 `_l_joint` 换成 `_r_joint`，部分关节（如肩侧展）限位方向镜像，以 URDF 为准。

## 2. real 模式三件必做（缺一臂不动，demo 已自动做）

### 2.1 ★使能手臂（真机头号坑）

**XARM 的"使能"不是电机上电，而是"XARM 要不要向 body_control 发指令"。** 不使能，MoveIt 规划/执行都报成功、控制器也 OK，但 XARM 根本不发 `/arm/cmd_pos` → 物理臂纹丝不动。demo 里 `enable_arm()` 依次探测使能服务（新版 `/moveit_controller_enable`、旧版 `/EAIHardware/set_arm_enable`），**调失败自动换下一个**；都失败时再查 `/EAIHardware/debug`——`arm_enable: 1` 就按已使能继续（使能是跨进程持续的硬件状态，上个 demo 已使能时重复使能常被拒，属正常噪音）。手动等价（用你机器上实际存在的那个名字）：

```bash
ros2 service call /EAIHardware/set_arm_enable std_srvs/srv/SetBool "{data: true}"
```

> 使能会**检查冲突**：若有别的程序在发 `/arm/cmd_pos`（比如残留的 atom04），使能会失败甚至被强制关闭——先 `bash scripts/stop_all.sh` 清场。

### 2.2 切 MoveIt 控制器

XARM 控制器互斥可插拔，MoveIt 执行前必须激活 `moveit_left_arm_controller`，否则执行失败 `error_code=-4 CONTROL_FAILED`。demo 里 `activate_moveit_controller()` 自动切。

### 2.3 发规划目标

`move_to_joints()` 给 7 个关节各建一个 `JointConstraint`，组成 `MoveGroup` 目标发给 `/move_action`；`error_code == 1` 即成功。

## 3. 代码解读（核心）

| 模块 | 代码锚点 | 职责 | 换右臂/双臂要改？ |
|---|---|---|---|
| 使能 | `enable_arm()` → `ENABLE_SRV_CANDIDATES`（探测 `/moveit_controller_enable` 或旧 `/EAIHardware/set_arm_enable`） | real 模式开"发指令的门" | 不变（`set_all_enable` 可一次全使能） |
| 切控制器 | `activate_moveit_controller()` → `switch_controller` | 激活 moveit 控制器 | 改 `MOVEIT_CONTROLLER` 名 |
| 读起点 | `read_current()` 订 `/joint_states` | 拿当前角作运动起点 | 改 `JOINT_NAMES` |
| 规划执行 | `move_to_joints()` → `MoveGroup` action | 建 JointConstraint、发 /move_action、查 error_code | 改 `GROUP` + `JOINT_NAMES` |

**举一反三**：换右臂 = `GROUP="right_arm"` + 关节名 `_l_`→`_r_` + 控制器 `moveit_right_arm_controller`；双臂 = 用双臂规划组（名以 SRDF 为准），关节约束填 14 个。发 MoveGroup 的套路完全一样。

## 4. 和 atom04 的区别

| | atom04（裸控） | atom05（MoveIt） |
|---|---|---|
| 谁算轨迹 | 你自己（直发目标角，无规划、无避障） | MoveIt（规划无碰撞平滑轨迹） |
| 走什么 | 直接 publish `/arm/cmd_pos` | `/move_action` → XARM 控制器 → 底层仍落到 `/arm/cmd_pos` 或 `/arm/cmd_ctrl` |
| 要使能吗 | 不用（自己发话题，不经 XARM） | **要**（经 XARM，有"使能门"） |
| 适合 | 理解原理、单关节点动 | 生产：多关节协调、避障、平滑 |

> 底层殊途同归：两者最终都落到 `/arm/cmd_*` → body_control → 电机。差别只在"人肉直发" vs "MoveIt 规划后由 XARM 发"。也正因抢同一个 `/arm/cmd_pos`，**atom04 和 XARM 使能双向互斥**：跑着 atom04 时 XARM 使能会失败；反过来 **XARM 使能中（跑过本原子后即如此）它以 250Hz 持续发流，atom04 的直发被瞬间覆盖 → 臂"看似不动且无报错"**。一条臂同一时刻只能有一个指令源，使能开关就是"交接方向盘"。两个方向 demo 都已自动处理：本原子自动开使能，atom04 检测到使能会自动关掉接管——来回切换零手动。

## 5. 排错

| 现象 | error_code / 原因 | 处理 |
|---|---|---|
| 规划/执行报成功，但臂不动 | 没使能（real 头号坑）；或在 sim（物理臂本就不动） | `set_arm_enable true`；sim 看 RViz/joint_states |
| `error_code=-4` CONTROL_FAILED | 没激活 moveit 控制器 | 切控制器（demo 已自动；手动 `switch_controllers --activate`） |
| `error_code=-15` INVALID_GROUP_NAME | 规划组名不对 | 核 SRDF：`ros2 param get /move_group robot_description_semantic` |
| 目标被拒绝 / 规划失败 | 目标超限或有碰撞 | 对照 1.3 限位；把臂先移到接近目标的安全姿态 |
| `/move_action` 连不上 | MoveIt 组件没起 | `ros2 action list \| grep move_action`；起 `tianyi2_moveit.launch.py` |
| 使能失败 / 自动关使能 | 别的程序在发 `/arm/cmd_pos`（如残留 atom04；也可能是遥控器 `teleop_dispatcher`） | `bash scripts/stop_all.sh` 清场后重来；`ros2 topic info /arm/cmd_pos -v` 查发布者，正常应只剩 1 个 |
| `跳过使能`（找不到使能服务） | **XARM 升级后使能服务改名**：旧 `/EAIHardware/set_arm_enable` → 新 `/moveit_controller_enable`（类型都是 `SetBool`） | demo 已运行时自动探测两个名字、调失败自动换下一个；手动核实用 `ros2 service list \| grep -iE 'enable'`、`ros2 service type <名>` |
| 日志刷`使能未成功…尝试下一个候选`，随后`臂已是使能状态(arm_enable=1)…继续` | **正常噪音**：使能是跨进程持续的硬件状态，上个 demo 已使能，本次重复使能被拒 | 无需处理；使能真值以 `ros2 service call /EAIHardware/debug eai_manipulator_msgs/srv/Info` 的 `arm_enable` 为准 |
| 起 XARM 后 spawner 全卡 `waiting for /controller_manager/list_controllers` | **controller_manager 加载硬件插件失败**（常见于系统升级后缺库，如 pinocchio 3.9 被换成 4.0，`dlopen libpinocchio_parsers.so.3.9.0` 失败） | ① `ldd /home/ubuntu/XARM/install/tianyi_hardware/lib/libtianyi_hardware.so \| grep 'not found'` 有缺 → ② 厂商给的兼容库目录写进 `~/.bashrc`：`export LD_LIBRARY_PATH=<lib目录1>:<lib目录2>:$LD_LIBRARY_PATH`（须指到含 .so 的**子目录**）→ 清场重起。出厂正常机器 `ldd` 干净则**不需要**此步；XARM 重编适配后应删掉该行 |
| 真机"看门狗超时" | body_control 没起 / 没收到 `/arm/status` | 先起 body_control，再起 XARM |

自查两条：`ros2 service list | grep -iE 'enable'`（确认使能服务名、类型 `ros2 service type <名>` 应为 `std_srvs/srv/SetBool`）、`ros2 control list_controllers | grep -i moveit`（看控制器 active）。

## 6. 进阶

- **arm_mode（硬件工作模式）**：`0` 柔顺(力位混合，本 demo 默认) / `3` 高刚度位置环（跟随更硬）/ `1` 高刚度速度环(需实时内核) / `2` 重力补偿。切：`ros2 service call /EAIHardware/set_arm_mode eai_manipulator_msgs/srv/Mode "{mode: 3}"`。mode 还决定 XARM 发 `/arm/cmd_pos` 还是 `/arm/cmd_ctrl`。
- **末端（笛卡尔）控制**：本原子是**关节空间** MoveIt；想给"末端位姿"目标（而非关节角），见 **atom06（MoveIt 末端控制）**。
- **库 vs 原生 action**：本 demo 直接发 `/move_action`（最轻、最透明）；也可用 MoveIt 的 Python 库 `moveit_py`（`.plan()/.execute()`），但要额外加载 MoveIt 配置参数。
