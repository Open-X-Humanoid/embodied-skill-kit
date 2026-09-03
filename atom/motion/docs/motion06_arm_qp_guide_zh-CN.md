# 运控6 · 手臂（Arm）· XARM QP 关节空间控制

[English](motion06_arm_qp_guide.md) | **简体中文**

**一句话**：把 7 个目标关节角交给 XARM 的 **QP 关节控制器**（`jointspace_arm_L_controller`），它在线做速度/加速度/jerk 平滑与限位检查后执行。和 motion04 的区别只有一处：**motion04 是 MoveIt"一次规划整条轨迹再执行"，motion06 是 QP"响应式在线跟踪"**——收到新目标立即平滑跟过去，适合目标不断变化的场景（如视觉伺服）。

| 配套 | 路径 |
|---|---|
| 代码（简洁版） | `atom/motion/motion06_arm_qp.py` |
| 演示视频 | 与配套代码同名，见 `atom/motion/assets/videos/` |

> 建议先读 motion04：使能、切控制器这两件事各手臂原子共用；本文只展开 QP 特有的部分。

## 1. 速览

### 1.1 跑起来（比 motion04 少一步——QP 不需要 MoveIt 组件）

- **板子**：**x86、ubuntu 用户**（和 body_control 同板；XARM 装在 `/home/ubuntu/XARM`）。
- ★**跑 demo 需要 source XARM**（QP 的消息包 `eai_manipulator_msgs` 在 XARM 里，不在基础 ROS）：`source /home/ubuntu/XARM/install/setup.bash`，每个新终端都要执行。

```bash
# ⓪ 停遥控服务（开机自启、占 /arm/cmd_pos，不停则使能会失败；重启机器后会自动恢复）
sudo systemctl stop teleop_robot

# 真机：先起 body_control，再起 XARM 本体（QP 不需要 tianyi2_moveit.launch.py）
bash scripts/start_body_control.sh          # 另开终端，见《前置 · 环境配置》
bash scripts/start_xarm.sh real

# 跑 demo（先 source XARM）
source /home/ubuntu/XARM/install/setup.bash
python3 atom/motion/motion06_arm_qp.py
```

demo 做的事：读当前关节角 → **只动肘俯仰 +0.3 rad（≈17°）** → 等实际到位 → 回起始角。可复位、小幅。

> ⚠ **本控制器无防碰撞**（XARM 手册明示；防自碰版是 `jointspace_arm_qpik_L_controller`，但它不支持 action）。臂周围无人无物、急停在手。ROS 基础环境见《前置 · 环境配置》(`docs/environment_setup_zh-CN.md`)。

### 1.2 接口（XARM 原生，不走 /move_action）

| 接口 | 类型 | 作用 |
|---|---|---|
| `/jointspace_arm_L_controller/jointspace` | `eai_manipulator_msgs/action/JointSpace`（**Action**） | 送 7 个目标关节角，`result.success` 报成败 |
| `/jointspace_commands_L` | `std_msgs/Float64MultiArray`（Topic，流式） | 连续喂目标的流接口——**与 action 二选一，不能同时用** |
| `/EAIHardware/set_arm_enable` 等 | `std_srvs/SetBool`（Service） | real 模式使能（名随 XARM 版本变，demo 自动探测候选） |
| `/controller_manager/switch_controller` | ros2_control（Service） | 激活 `jointspace_arm_L_controller` |
| `/joint_states` | `sensor_msgs/JointState`（Topic） | 读当前关节角作起点 + **到位判断** |

> action 与 topic 的分工：**action** 适合"给一个目标、等它到位"（本 demo）；**topic** 适合"高频连续喂目标"（视觉伺服）。两者只能用其一——用 action 时确保没人往 `/jointspace_commands_L` 发流。

### 1.3 关节限位（同 motion04，此处不重复）

7 关节限位表见《运控4 · 手臂 · XARM MoveIt 关节运动》(`atom/motion/docs/motion04_arm_moveit_guide_zh-CN.md`) 1.3 节。QP 控制器**自带限位检查**：超上限报 `600101`、超下限报 `600102`，指令被拒、臂不动。

## 2. 核心操作（含一个 QP 特有的实测坑）

### 2.1 使能 + 切控制器（同 motion04/06 套路）

使能开"XARM→body_control 的发指令门"；切控制器时 **moveit/jointspace/endpose 各族互斥**（都抢同一条臂的关节接口），demo 的 `_scan_arm_controllers()` 自动查占用并停掉，再 STRICT 切换。

### 2.2 发目标（比 MoveIt 简单得多）

不用建约束——action 的 goal 就是一个 7 元素数组：

```python
goal = JointSpace.Goal(target_positions=[0.0, 1.18, 0.0, -1.3, 0.0, -0.13, 0.18])
```

顺序 = J1 肩俯仰 → J7 腕旋转（同 `JOINT_NAMES`）。

### 2.3 ★等实际到位（本原子的头号实测坑）

**QP action 的 `success` 返回 = 目标已接受，不等于已到位**（在线跟踪型控制器的普遍行为）。如果返回后立刻发下一个目标，上一目标会被覆盖 → 臂几乎不动。所以 demo 在 action 返回后还要 `wait_reached()`：轮询 `/joint_states`，等全部关节进入 `目标 ±0.05 rad` 才算真到位（15s 超时告警）。

> 对照 motion04：MoveIt 的 action result 是**执行完才返回**（轨迹控制器确认到位），所以 motion04 不需要这一步。这是"离线规划"和"在线跟踪"在使用手感上最重要的差别。

## 3. 代码解读（核心）

| 模块 | 代码锚点 | 职责 | 换右臂要改？ |
|---|---|---|---|
| 使能 | `enable_arm()` + `_already_enabled()` | 多候选探测使能服务（名随版本变）；重复使能被拒时查 `arm_enable` 兜底 | 不变 |
| 查冲突 | `_scan_arm_controllers()` → `list_controllers` | 按 `claimed_interfaces` 找占用本臂的 active 控制器 | 改 `JOINT_NAMES` |
| 切控制器 | `activate_qp_controller()` → STRICT 切换 | 停冲突者 + 激活 QP，失败如实报错 | 改 `QP_CONTROLLER`（L→R） |
| 读起点 | `read_current()` 订 `/joint_states` | 当前角作起点 | 改 `JOINT_NAMES` |
| 发目标 | `move_to_joints()` → `JointSpace` action | 7 元素数组直发，收 `result.success` | 不变 |
| **等到位** | `wait_reached()` 轮询 `/joint_states` | **action 返回≠到位**，等实际角收敛 ±0.05rad | 改 `JOINT_NAMES` |

**举一反三**：换右臂 = `QP_CONTROLLER` 的 `L`→`R` + 关节名 `_l_`→`_r_`。发目标的套路完全一样。

## 4. 和 motion04（MoveIt 关节）的区别

| | motion04（MoveIt） | motion06（QP） |
|---|---|---|
| 工作方式 | 离线规划整条轨迹 → 执行 | 在线响应式跟踪，收到目标立即平滑跟 |
| 前置 | XARM 本体 + **MoveIt 组件** | 只要 XARM 本体 |
| 目标表达 | 7 个 `JointConstraint` | 一个 7 元素数组（简单得多） |
| action 返回含义 | **执行完成** | **目标已接受**（要自己等到位） |
| 防碰撞 | 有（MoveIt 规划避障） | **无**（qpik 变体有但不支持 action） |
| 适合 | 一次性定点运动、要避障 | 目标连续变化（视觉伺服）、低延迟跟踪 |

> 底层殊途同归：两者都经 XARM → `/arm/cmd_*` → body_control → 电机。差别在"谁生成关节轨迹"。

## 5. 排错

| 现象 | 原因 | 处理 |
|---|---|---|
| `600101` / `600102` | 目标超上/下限位 | 对照 motion04 限位表；从当前角小步增量 |
| `600100` | QP 优化不可行 | 减小增量；查起始姿态是否已在奇异/极限附近 |
| action 返回成功但臂不动 / 只动一下 | ①没使能 ②action 返回≠到位、目标被下一指令覆盖 | ①查 `arm_enable=1` ②靠 `wait_reached()` 等收敛再发下一个 |
| QP action 连不上（10s 超时） | 控制器没激活 / 被别族控制器占着 | `ros2 control list_controllers` 查；demo 已自动避让，手动则先 deactivate 占用者 |
| 使能相关报错 | 与 motion04 完全同源：服务改名、teleop 占话题、重复使能噪音 | 见《原5》排错表逐条对号；`sudo systemctl stop teleop_robot` |
| 臂动得太快 | QP 速度是**控制器参数**，不在下发接口里 | `ros2 param set /jointspace_arm_L_controller vel_limits '[0.5,0.5,0.5,0.5,0.5,0.5,0.5]'`（rad/s，越小越慢，即时生效） |
| 发 topic 没反应 | 有人在用 action（二选一互斥） | 停掉 action 客户端再发流 |

信息代码速查（控制器日志）：`100101` 收到新目标（info）、`100102` 内部规划完成（info，可用于到位判断）、`6001xx` 见上表。

## 6. 进阶

- **调速**：`vel_limits`（7 关节速度上限 rad/s）是最直接的总阀门，命令行 `ros2 param set` 即时生效；只读参考 `acc_limits`/`jerk_limits`。⚠ 手册明示 vel_limits 不应超过物理机构执行能力——**只往小调是安全方向**。代码级调法见 motion07 的 `set_vel_limits()`（同一套参数服务，可直接搬过来）。
- **流式控制（视觉伺服的正确姿势）**：改用 `/jointspace_commands_L` topic 高频喂目标，控制器自动平滑衔接——这才是 QP"响应式"的完全体。记得先停 action 客户端。
- **要防自碰**：换 `jointspace_arm_qpik_L_controller`（防自碰 + 双臂互碰），但只支持 topic、无 action；手册建议确定不会自碰时优先用本控制器（更轻）。
- **末端版**：想给"末端位姿"而不是关节角，见《运控7 · 手臂 · XARM QP 末端控制》(`atom/motion/docs/motion07_arm_qp_endpose_guide_zh-CN.md`)。
