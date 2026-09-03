# 技能1 · 手指点按（Finger Tap）· 感知—手臂—灵巧手协调，用食指点按墙上卡片

[English](skill01_finger_tap_guide.md) | **简体中文**

**一句话**：Orin 上的相机看到墙上的 AprilTag 卡片、算出它在机器人 `base` 系的位置与朝向，通过话题告诉 x86；x86 上的手臂先摆好点按手型（只伸食指），伸到卡前 8cm、闭环补一刀把误差压到 ±1cm，再让食指点按卡片，最后收手归位。这是第一个「技能」——多个部位（感知 + 手臂 + 灵巧手）协调成一个完整动作。

> 当前是 Skill 的第一阶段迁移：保留已经真机迭代过的执行流程与人工确认闸。标准 Goal、`SkillResult`、明确的成功/失败、统一超时、恢复和数据记录接口将在后续迭代补充。

| 配套 | 路径 |
|---|---|
| 感知（Orin 跑） | `skill/skill01_finger_tap/tag_locator.py` |
| 运控（x86 跑，主文件） | `skill/skill01_finger_tap/finger_tap.py` |
| 配置常量（调参只改这里） | `skill/skill01_finger_tap/config.py` |
| 演示视频 | 与配套代码同名，见 `skill/skill01_finger_tap/assets/videos/` |

> 技能层主线是**裸 rclpy 直调 XARM**（`import rclpy` + 标准 msg，不 `import xarm_sdk`/`xrocs`）——透明、少依赖。手臂的四种控制模式（MoveIt/QP × 关节/末端）在原子层 atom05~08 已单独讲透，本文只讲**怎么把它们协调起来完成点按**，不重复展开单个控制器。建议先读《原6 · 手臂 MoveIt 末端》(`atom/docs/atom06_arm_moveit_endpose_guide_zh-CN.md`) 与《原8 · 手臂 QP 末端》(`atom/docs/atom08_arm_qp_endpose_guide_zh-CN.md`)。

## 1. 速览（点进来先看这块）

### 1.1 跑起来

这个 demo **跨两块板子**（天轶 2.0 双板：Orin 感知 / x86 运控），两板要在**同一个 `ROS_DOMAIN_ID`** 下才能互通话题。

**① Orin（`nvidia` 用户）——起相机 + 感知：**

```bash
bash scripts/start_camera.sh                                  # 起 Orbbec 相机驱动
python3 skill/skill01_finger_tap/tag_locator.py   # 检测 tag、发目标点；不动机器人，零风险
```

> ⚠ **相机话题名会自动探测，一般不用管**（细节见《前置 · 环境配置》第 4 节，`atom/docs/environment_setup_zh-CN.md`）：命名空间因机器而异，启动时扫 ROS 图自动认出、日志里会打印；多相机或想强制指定才 `export CAMERA_NS=<命名空间>`。但**相机若已由出厂服务自启，要跳过上面的 `start_camera.sh`**，否则会起第二个驱动抢 USB。

`tag_locator.py` 只做「看」：检测卡片 → 算出中心 + 法线 → 持续发到话题 `/skill01/target_point`。它**不动机器人**，可以先单独跑、拿 `ros2 topic echo` 看数值是否合理。

**② x86（`ubuntu` 用户）——起本体控制 + 跑主程序：**

```bash
bash scripts/start_body_control.sh          # 另开终端，等 "All devices ready."
bash scripts/start_xarm.sh real             # XARM 本体（MoveIt 后端需 MoveIt 组件），等 ~20s

source /home/ubuntu/XARM/install/setup.bash # 跑 demo 的终端 source 一次 XARM（QP 消息包 + 使能兜底都要）
python3 skill/skill01_finger_tap/finger_tap.py
```

**预期现象**（每个 `input()` 停下等你回车，看清坐标再放行）：打印 tag 坐标 → 回车 → 手摆成点按手型（只伸食指）→ 关节空间回预备姿态 → 伸手到卡前 8cm → 闭环补刀 → 回车 → 食指按下（过冲 1.5cm，柔顺吸收）→ **停住等你回车确认点按姿态** → 退回停驻位 → 关节空间归位 + 手张开。

> ⚠ **执行前逐条确认（安全）**：
> 1. 卡片**固定在静物上**（纸箱/桌沿），**人退出手臂可达范围**——目标就是卡片，别拿在手里。
> 2. **`SAFE_BOX` 默认关闭**（`config.SAFE_BOX=None`）：动前打印目标坐标 + 回车确认是**唯一的人工闸**，回车前务必核对坐标合理。要软件闸可在 config 重开三轴范围。
> 3. 急停在手。QP 慢速 `VEL_LIMITS` / MoveIt `VEL_SCALE=0.1` + arm_mode 0 柔顺兜底。
> 4. 出意外臂停半空：`python3 .../finger_tap.py --recover` 收臂（见第 6 节）。

> ⚠ **高频坑 `error_code=99999`（MoveIt 后端伸手时）**：MoveIt 限位表比 URDF 紧，起点越界或目标够不到都会秒拒。**把卡片放在机器人左前方（左手舒适区）**最省事；先跑 `reach_check.py` 三档预检定位（臂不动零风险）。详见第 5 节。

### 1.2 接口

| 方向 | 名称 | 类型 | 作用 |
|---|---|---|---|
| 跨板话题 | `/skill01/target_point` | `geometry_msgs/PoseStamped`（frame=`head_roll_link`） | tag_locator 发、finger_tap 收：`position`=卡片中心，`orientation` 的 z 轴=卡面法线 |
| 下发 | `/inspire_hand/ctrl/left_hand` | `sensor_msgs/JointState` | 摆点按手型（食指伸直、其余蜷起）；见《原3 · 灵巧手》 |
| TF | `base ← head_roll_link` | tf2 | 把目标点/法线从相机标定系变换到 `base` 系 |
| TF | `base ← left_tcp_link` | tf2 | 读手腕末端 tcp 当前位姿（起点、闭环实测） |
| TF | `left_tcp_link ← left_index_2` | tf2（静态） | 指尖补偿：查 tcp→指尖的恒定偏移 |
| 伸手（MoveIt 后端） | `/move_action` | `moveit_msgs/action/MoveGroup` | 一步规划到卡前（`ARM_BACKEND="moveit"`，默认） |
| 伸手/微调/按下/退回 | `/endpose_single_arm_qp_L_controller/endPosSingleTarget` | `eai_manipulator_msgs/action/EndPosSingleTarget` | endpose QP 末端短程（微调/按下/退回恒用它；`ARM_BACKEND="qp"` 时伸手也用它） |
| 回预备/归位 | `/jointspace_arm_L_controller/jointspace` | `eai_manipulator_msgs/action/JointSpace` | 关节空间大范围移动（回 READY、结束归位） |
| Service | 使能 + `switch_controller` / `list_controllers` | — | 同 atom06/08 已验证套路（使能服务名自动探测） |

> ⚠ **`base` 不是 `base_link`**：天轶 2.0 URDF 里根是 `base_footprint`，经零偏移固定关节接 `base`，**没有 `base_link` 这个 frame**。写错直接 TF 超时退出。核实：`ros2 run tf2_ros tf2_echo base left_tcp_link`。

### 1.3 心智模型：3 个位置 × 3 个控制器 × 4 个阶段

整个动作是手臂在 **3 个位置**间往返，只用 **2 种控制方式**（大范围走关节、卡片附近走末端）：

```
(开机)
  └─关节空间──▶ ①READY 预备姿态（手朝卡片、离卡 20~30cm）
                   └─末端空间──▶ ②卡前停驻点（8cm）──▶ ③按下（过冲 1.5cm）
                                    ▲                        │(回车确认姿态)
                                    └──④退回停驻──────────────┘
  ◀─关节空间──── ⑤归位 READY + 手张开
```

| 控制方式 | 控制器 | 给什么 | 用在哪 | 为什么 |
|---|---|---|---|---|
| **关节空间**（jointspace QP） | `jointspace_arm_L_controller` | 7 个关节角(rad) | 回 READY、结束归位 | 大范围位移不经 IK、不扭曲、稳；QP 碰撞球不含手，预备位由人确认安全 |
| **末端空间**（MoveIt） | `moveit_left_arm_controller` | tcp 位姿(base 系) | 伸手（默认后端） | 一步规划、会绕障；卡片附近的精细活 |
| **末端空间**（endpose QP） | `endpose_single_arm_qp_L_controller` | tcp 位姿(base 系) | 微调/按下/退回（恒用） | 短程分段小步，是它的舒适区；姿态可锁死不给容差第二次机会 |

**四个阶段**（日志里就是这么打的：`阶段N/4` + 动作圈号 `①-⑤` + `[控制器｜关节/末端]`）：

1. **看**：收 tag_locator 发的中心+法线 → TF 变换到 base 系 → 算 tcp 目标。
2. **就位**：使能 → 摆手型 → 回 READY → 伸手到卡前 8cm → 闭环补刀压误差。
3. **点按**：从停驻点前进按下（过冲 1.5cm）→ 回车确认点按姿态。
4. **收手归位**：退回停驻位 → 关节空间归位 + 手张开。

> `ARM_BACKEND`（config）只切「伸手」用 MoveIt 还是 endpose QP；**微调/按下/退回恒用 endpose QP，回 READY/归位恒用 jointspace QP**——这三件事不受后端开关影响。

## 2. 三个核心概念（「协调」体现在哪）

技能层的新东西不是某个控制器（那是原子层的事），而是**怎么把感知、手臂、灵巧手这三个部位拧成一个动作**。三个概念先懂，代码就顺了。

### 2.1 双板协作 + 一次快照语义

「看」在 Orin（GPU/相机在那），「动」在 x86（XARM 接口在那），中间只靠一根话题 `/skill01/target_point` 传坐标。两板共享同一个 ROS2 图（同 `ROS_DOMAIN_ID`）。

- **感知发的是 `head_roll_link` 系坐标**（相机手眼外参标定在头部 link 下），运控收到后先用 TF 变换到 `base` 系再用——见 `finger_tap.to_base()`。
- **一次快照语义（安全设计）**：`finger_tap` 启动时取一次目标点（`wait_target()` 凑满 `N_SAMPLES=10` 帧取中位数压抖动），**之后不再更新、绝不追踪移动目标**。执行中挪卡片，手臂仍去原定点——这是刻意的，防止手追着动的东西跑。

### 2.2 tcp vs 指尖：运控只控 tcp，指尖被「带着走」

这是最容易踩的坑，务必先懂：

- **运控只控制 tcp（手腕 `left_tcp_link`）**，三个控制器都不认识指尖。
- **`left_index_2` 刚性挂在 tcp 前方**，被 tcp 带着走。代码先通过 TF 读取 tcp→`TAP_LINK` 的完整旋转和平移。
- **真指肚不等于 `TAP_LINK` 原点**：`PAD_LOCAL_OFFSET=[0.01365, 0.04307, 0.00499]` 描述指肚相对 `left_index_2` 局部坐标系的三维固定偏移。代码把它按手指实际姿态旋转到 tcp/base 系，再与 TF 偏移合成；手腕姿态变化时仍按刚体关系计算，不再假设指肚只沿一条轴前伸。
- **使用者关心指尖，tcp 只是手段**。所以日志里末端动作打两行：`手腕 tcp（位置+rpy，参考）` + `指尖指肚（位置，← 核心）`。看轨迹对不对，看指肚那行。

> 手型也是「协调」的一部分：抬臂前先把灵巧手摆成**只伸食指、其余三指蜷起**（`POINT_POSE`）。① 接近时其它手指不蹭卡片所在的板面；② 食指成唯一凸出点。★食指必须保持伸直——`PAD_LOCAL_OFFSET` 来自伸直手指的 STL 网格，蜷起后该标定不再成立。

> ⚠ `PAD_LOCAL_OFFSET` 来自离线网格计算，尚未在真机上重复验证。首次接触前应先用 `PRESS_ENABLE=False` 检查指肚停驻位置，并确认仓库 STL 与真机装配一致。

### 2.3 三层误差与标定：固定的一次标定掉，随机的闭环压到地板

点按精度靠三层误差治理，**分清哪层是固定误差、哪层是随机误差**是调参的关键：

| 层 | 误差来源 | 固定/随机 | 治它的参数 |
|---|---|---|---|
| 感知 | 手眼外参偏差（**最大误差源**，±1~3cm） | 固定 | `extrinsics.json` / `AIM_BIAS_BASE` 吸收 |
| 感知 | 内参 / `TAG_SIZE` 量错（按比例放大全部距离） | 固定 | `camera_intrinsics.json` / `TAG_SIZE`（★量准卡片黑框边长） |
| 感知 | tag 检测抖动 | 随机 | `N_SAMPLES` 取中位数；法线太抖→`ORIENT_MODE="level"` |
| 模型 | 指肚 ≠ `TAP_LINK` 原点 | 固定 | 三维局部偏移→`PAD_LOCAL_OFFSET`；整体瞄准偏差→`AIM_BIAS_BASE` |
| 执行 | 手腕姿态容差 × 17cm 杠杆 | 随机 | 闭环补刀 `correct_tip`（`CORRECT_TOL`/`CORRECT_MAX`） |

**心法：先确认 `PAD_LOCAL_OFFSET` 的几何标定，再用 `AIM_BIAS_BASE` 吸收整体固定偏差；随机误差用「中位数 + 闭环」压低。**

- **怎么把偏差翻译成参数**：整体、稳定的左右/上下偏差可调 `AIM_BIAS_BASE`；若指肚相对 `TAP_LINK` 的三维几何本身不准，应重新标定 `PAD_LOCAL_OFFSET`，不要再用单一深度标量硬补。
- **`AIM_BIAS_BASE` 符号**（机器人视角，base +y=机器人左）：食指偏右→y 增大、偏左→y 减小；偏下→z 增大、偏上→z 减小。
- **闭环补刀（`correct_tip`）的病根与修法**：同一目标三连跑落点差 ±2~3cm、方向各异，而感知三次一字不差——散布全在执行侧（MoveIt 姿态容差 ±17°方向/±34°自旋 × 腕→指尖 17cm 杠杆）。修法：到位后 TF 实测 Δ → endpose QP 短程平移 −Δ（**姿态保持当前实际值，不给容差第二次机会**）→ 重测，`|Δ|≤CORRECT_TOL` 收工，最多 `CORRECT_MAX` 轮。就是人「发现偏了补一刀」的动作。

## 3. 代码解读（核心）

### 3.1 文件地图

| 文件 | 跑在哪 | 作用 | 动机器人？ |
|---|---|---|---|
| `tag_locator.py` | Orin | 阶段1 感知：检测 tag → 算 base 系坐标+法线 → 发话题 | ❌ 只看 |
| `finger_tap.py` | x86 | 阶段2-4 运控：收目标 → 伸手 → 闭环 → 点按 → 归位（**主文件**） | ✅ |
| `config.py` | 两板共用 | 所有可调参数（调参只改这里，不动逻辑代码） | — |
| `pose_math.py` | — | 姿态数学（四元数/旋转/rpy 换算，纯 numpy） | — |
| `reach_check.py` | x86 | 可达性预检（plan_only 三档，不动臂零风险）——排 99999 用它 | ❌ 只规划 |

### 3.2 finger_tap.py 模块地图

主文件按「四阶段」组织，锚点用函数名（稳定、可搜）：

| 阶段 | 代码锚点 | 职责 |
|---|---|---|
| 通用 · 收目标 | `_on_target` / `wait_target` | 凑满 N 帧取中位数（一次快照，之后不更新） |
| 通用 · 变换 | `to_base` | head_roll_link → base（点带平移+旋转，法线只旋转） |
| ①看 · 算目标 | `build_approach` | 由中心+法线算 (tcp 目标, 目标姿态, 指尖目标)；含 `ORIENT_MODE`/`AIM_BIAS`/指尖补偿 |
| 指尖补偿 | `pad_offset` / `read_pad_pos` | 合成 tcp→`TAP_LINK` TF 与局部 `PAD_LOCAL_OFFSET`，算真指肚位置 |
| ②就位 · 使能切控 | `enable_arm` / `activate_arm_controller` / `set_vel_limits` | 使能 + 按后端切控制器（自动避让占臂者）+ 慢速 |
| ②就位 · 手型 | `set_point_pose` / `set_hand_open` | 摆点按手型 / 归位时张开 |
| ②就位 · 回预备 | `goto_ready` | jointspace QP 回 `READY_JOINTS` |
| ①伸手 | `moveit_move_to_pose`（moveit）/ `move_segmented`（qp） | 到卡前停驻点 |
| 到位报告 | `report_tip_error` | TF 实测指尖 Δ，拆「左右/上下/前后 cm」 |
| ②微调 | `correct_tip` | endpose QP 闭环补刀，平移 −Δ 姿态不动 |
| ③按下 | `press_only` | 前进 8+过冲 → 触阻力停 → 回车确认姿态 |
| ④退回 | `retract_to_standoff` | 沿原路撤回停驻位 |
| 安全 · 落盘/恢复 | `save_start_pose` / `recover` | 动臂前落盘出发位；`--recover` 据此收臂 |
| 主流程 | `main` | 串起四阶段，每步 `input()` 人工确认 |

### 3.3 逐模块一句话（挑关键的）

- **`wait_target`**：凑满 `N_SAMPLES` 帧，位置和法线各取**中位数**（个别离群帧不带偏结果）；抖动 >5cm 告警提示卡片放近。
- **`build_approach`**：核心换算。`ORIENT_MODE="level"` 时接近方向取**法线的水平投影**；先算真指肚应停点（中心+`AIM_BIAS_BASE` 沿接近方向退 `APPROACH_OFFSET`），再用 `pad_offset()` 合成的 tcp→指肚三维偏移反算 tcp 目标。开环规划和闭环测量现在统一以**真指肚目标**为基准。
- **`correct_tip`**：见 2.3。恒走 endpose QP、恒压速；每轮平移 −Δ 后等 0.4s 让臂落稳再重测（QP action「目标已接受」就返回、臂还在追）。
- **`press_only`**：从停驻位沿 −n 前进 `APPROACH_OFFSET + PRESS_DEPTH`，触卡推不动而超时=已按到（`move_segmented(..., contact_ok=True)`）。**按到后停住不退，`input()` 等人工回车确认点按姿态**——这是最关键的人工检查点（不再用定时「保持」）。
- **`goto_ready`**：两后端统一在**开跑前**和**结束**都走一次——MoveIt 后端曾因不走 READY、以上次残留姿态为起点而跨次累计漂移（踩过）。
- **`moveit_move_to_pose` 的 `_pose_goal`**：位置球约束 + **逐轴**姿态约束，z 轴（绕指向轴自旋）单独放开到 `SPIN_TOL`——全紧会 99999 过约束，全松 MoveIt 会随便挑解、掌心外翻，逐轴才是正解。

### 3.4 举一反三

- **换点按的手指**：改 `config.TAP_LINK`，并在新 link 的局部坐标系中重新标定三维 `PAD_LOCAL_OFFSET`；`POINT_POSE` 里让目标手指伸直、其余蜷起。不能沿用左食指的偏移向量。
- **换右手/右臂**：`EE_LINK`→`right_tcp_link`、三个控制器名换右臂版、`ARM_JOINT_NAMES` 换右臂关节、`HAND_CMD_TOPIC` 换右手；`HAND_SPIN` 符号可能要重定（左右镜像）。`BASE_FRAME` 不变。
- **换目标类型**（不是 tag，是别的视觉目标）：只要 `tag_locator` 那侧改成发同样的 `PoseStamped`（中心 + 朝向），运控侧一行不用改——这正是话题解耦的好处。

## 4. 改一改，看变化

调参**只改 `config.py`**。先预测再跑，对照日志的「左右/上下/前后 cm」验证。

| 改什么 | 会怎样 / 何时改 |
|---|---|
| `AIM_BIAS_BASE` 的 y | 指尖左右平移（食指偏右→y 增大）。连跑 2~3 次**方向一致**再调，忽左忽右是噪声别调 |
| `AIM_BIAS_BASE` 的 z | 指尖上下平移（偏下→增大）。同上，方向一致才调 |
| `PAD_LOCAL_OFFSET` | 指肚相对 `TAP_LINK` 的三维局部偏移。只有确认网格/装配或标定有误时才改，三轴要作为一个几何量重新校准 |
| `APPROACH_OFFSET` | 停驻点离卡片多远（默认 8cm）。太近留给闭环的余量小 |
| `PRESS_DEPTH` | 按下过冲量（默认 1.5cm）。★真机首验从 0.5cm 起、急停在手，确认柔顺吸收正常再加大 |
| `SPIN_TOL` | MoveIt 对自旋的容差。`reach_check` ★档 99999 过约束时放宽到 1.0 再试 |
| `ORIENT_MODE` | `"level"`=手指水平指卡片（默认，抗法线噪声）；`"tag"`=完全跟随法线（斜卡垂直接近，法线噪声大时发飘） |
| `PRESS_ENABLE=False` | 只做阶段2（伸到 8cm 停、不接触），调伸手/闭环时用 |
| `HAND_POSE_ENABLE=False` | 不控手（保持当前手型），只调臂时用 |
| `ARM_BACKEND="qp"` | 伸手改用 endpose QP 分段（对照用；⚠长距扫掠 QP 碰撞模型不含手，会把手拖着撞身体） |

## 5. 排错

| 现象 | 原因 / 处理 |
|---|---|
| 超时未收到目标 | Orin 上 `tag_locator` 没跑 / 卡片不在视野 / 两板 `ROS_DOMAIN_ID` 不一致。`ros2 topic echo /skill01/target_point` 确认有数据 |
| **`error_code=99999`（MoveIt 伸手秒失败）** | ① 起点越界：QP/遥控把关节停在 MoveIt 界外（MoveIt 限位比 URDF 紧）→ `tmux capture-pane -t xarm.1 -p -J -S -400 \| grep -i 'outside bounds'` 找点名关节、用 QP 挪回；② 目标够不到/过约束：**卡片放机器人左前方（左手舒适区）**、先跑 `reach_check.py` 三档预检、必要时放宽 `SPIN_TOL`。详见《原5》排错表 99999 行 |
| 掌心朝向不对（外翻） | `HAND_SPIN` 名义值（level 模式下：0=掌心朝下按按钮、+1.57=朝内、3.14=朝上）；或 `SPIN_TOL` 太松让 MoveIt 挑了扭曲解 |
| 垂直偏下 ~3cm（某些卡位） | 感知/瞄准残留，在 ±2~4cm 地板内。连跑 2~3 次方向一致再调 `AIM_BIAS_BASE` 的 z（如 -0.01→+0.02）；忽上忽下是噪声别调 |
| ±2cm 随机散布 | 手腕姿态容差 × 指长，闭环只对齐骨架点、治不到，是地板，接受 |
| 手不动 / 摆手型跳过 | `/inspire_hand/ctrl/left_hand` 无订阅者（手驱动 inspire_hand 没起）；`publish` 对零订阅者不报错、消息直接丢弃 |
| 使能相关 / 规划成功但臂不动 / spawner 卡 | 与 atom05/06 同源（使能服务改名、teleop 占 `/arm/cmd_pos`、升级缺 pinocchio 库等），见《原5》排错表逐条对号 |
| TF 超时（`base ← ...`） | frame 名写错（是 `base` 不是 `base_link`），或 XARM/body 没起。`ros2 run tf2_ros tf2_echo base left_tcp_link` 核实 |
| 查不到 tcp→指尖 TF，警告不补偿 | `TAP_LINK` 名字对吗？`ros2 run tf2_ros tf2_echo left_tcp_link left_index_2` 核实 |
| 臂停在半空（段失败/Ctrl-C/崩溃） | `python3 .../finger_tap.py --recover` 按落盘出发位 QP 慢速原路退回、再关节归位（见第 6 节） |

## 6. 进阶

- **`--recover` 收臂**：每次动臂前出发位姿已落盘 `_last_start_pose.json`。`python3 skill/skill01_finger_tap/finger_tap.py --recover` → QP 慢速原路退回出发位 →（若配置）`READY_JOINTS` 归位；动前回车确认。无可用恢复目标时程序不动作，由现场人员按安全规程处置。
- **`reach_check.py` 三档可达性预检**：plan_only 只规划不动臂（零风险），★档=运控同款约束，过了即可实跑——排 99999 先用它定位是「够不到」还是「起点越界」。
- **看细节日志**：控制器切换 / 逐段路点 / 内部数学都是 debug 级，默认不显示。跑时加 `--ros-args --log-level debug`。
- **（规划中）xrocs 封装版对照**：技能层约定给「同一动作，xrocs 一行 vs 裸 rclpy N 行」的对照，让读者看清封装省了什么。本 demo 的裸 rclpy 版就是「N 行」那侧。
