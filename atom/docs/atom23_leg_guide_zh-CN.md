# 原23 · 下肢（Leg，腰+腿协调）· 协调升降与带力保持

[English](atom23_leg_guide.md) | **简体中文**

**一句话**：让机器人下蹲/抬升——升降是 **hip + knee + 腰pitch 三关节协调**（不是纯腿单关节），从当前姿态**小步插值**逼近，停止时**带力保持**、绝不瘫软。

> ⚠ 承重原子：升降腿托着整个上身。天轶是**轮式底座**机型，横向由底座撑稳；真正要防的是**失力时上身坠落**（靠"带力保持"）。动手前读完本页。

| 配套代码 | 演示视频 |
|---|---|
| `atom/demos/atom23_leg_ros2.py`（ROS2 原生版）、`atom/demos/atom23_leg_ros2_robust.py`（生产版） | `atom/assets/videos/` 下与配套代码同名，**尤其建议先看**别人跑对是什么样 |

## 1. 速览（点进来先看这块）

### 1.1 跑起来

前提：已按《前置 · 环境配置》（`atom/docs/environment_setup_zh-CN.md`）起好 body_control。**物理准备**：底座停稳、升降空间内无人无物、物理急停在手。（轮式底座已提供横向稳定，一般无需吊挂；仅在试**大幅度**升降或配合手臂大幅伸展时才需额外防护。）

```bash
ssh ubuntu@<机器人IP>
source /home/ubuntu/ros2ws/install/setup.bash   # 每个新终端都要执行
python3 atom/demos/atom23_leg_ros2.py
# 输入 GO 确认 + 5 秒倒计时 → 下蹲约几 cm → 停 1s → 抬回起点 → 带力保持
```

⚠ 中途 `Ctrl-C` 会自动**带力保持当前姿态**（不会瘫软塌下去）。

### 1.2 接口

| 项 | 值 |
|---|---|
| 腿 下发/状态 | `/leg/cmd_pos` / `/leg/status` → `bodyctrl_msgs/CmdSetMotorPosition` · `MotorStatusMsg` |
| 腰 下发/状态 | `/waist/cmd_pos` / `/waist/status`（升降要腰一起动） |
| 电机 ID | 腿 `51`=hip(髋) `52`=knee(膝)；腰 `32`=pitch(前倾，参与升降) `31`=yaw(保持不动) |
| 升降方向 | 下蹲(变矮)：hip↓ knee↓ waist↑；抬升(变高)：hip↑ knee↑ waist↓ |
| `cur` | 电机**最大电流限值**(A) = **力矩上限**（力矩∝电流，是封顶不是实际值）；承重关节取电机额定 **20A**，才有力矩托住上身、又不超电机能力 |

> **为什么没有"纯腿"原子**：腿单独动会让上身姿态歪掉、重心偏移。升降靠腰腿协调、变高变矮时保持上身竖直稳定，所以直接给"协调升降"，不给"只动腿"。

### 1.3 关节限位与安全

⚠ 以 URDF 为准。但注意：**本 demo 不是发绝对目标角，而是从当前姿态做极小相对位移**，所以真正的安全约束是"小步 + 协调 + 带力保持"，不是撞不撞绝对限位。

| 关节 | ID | URDF 硬限位 | 本 demo 怎么动 |
|---|---|---|---|
| hip 髋 | 51 | (−0.419, 0.908) rad（−24°~52°） | 从当前值 ±0.08 rad 小幅 |
| knee 膝 | 52 | (−1.745, 0.506) rad（−100°~29°） | 从当前值 ±0.08 rad 小幅 |
| waist pitch 前倾 | 32 | (−0.785, 2.094) rad（−45°~120°） | 与腿反向 ±0.08 rad 协调 |
| waist yaw 转腰 | 31 | (−2.967, 3.142) rad | **保持不动** |

**安全核心三条（缺一不可，代码已内置）**：

1. **先读状态才动**：读不到腿+腰四个当前角，**绝不运动**。
2. **小步慢速插值**：只从当前姿态蹲约 `0.08 rad`（几 cm）再原路抬回，每 50ms 一步、默认 4s 走完——**绝不跳到某标定姿态**。
3. ⚠ **停止 = 带力保持（`spd=0, cur=20A`）**：急停/结束/异常都保持当前姿态。**★绝不能把承重关节 `cur` 设 0**——那等于腿失力、上身直接坠落砸下来。

⚠ **电压/电机故障**：腿承重，供电不足时电机会报错（如欠压 `12832`）。生产版会自检电机错误码、发现就立即带力保持并退出（见第 6 节）。

## 2. 三个核心操作

### 2.1 让它动 —— 三关节协调 + 小步插值

升降要**同时**发腿（hip/knee）和腰（pitch），且**从当前姿态分多步逼近**，不跳变：

```python
# 一组目标角分发到腿、腰两个话题（cur=20A 托住上身）
leg.cmds  = [SetMotorPosition(name=51, pos=hip,  spd=spd, cur=20.0),
             SetMotorPosition(name=52, pos=knee, spd=spd, cur=20.0)]
wst.cmds  = [SetMotorPosition(name=32, pos=waist, spd=spd, cur=20.0),
             SetMotorPosition(name=31, pos=yaw,   spd=0.3, cur=20.0)]   # yaw 保持

# move_to：从当前(hip0,knee0,waist0)分 n 步线性插值到目标，每步 sleep 50ms
for k in range(1, n + 1):
    r = k / n
    _publish(hip0 + (hip_t-hip0)*r, knee0 + (knee_t-knee0)*r, waist0 + (waist_t-waist0)*r, yaw, spd)
    time.sleep(0.05)
```

### 2.2 读状态 —— 读齐腿+腰四个角才动

腿和腰**用同一个回调**存进一个 `self.pos`（电机 ID 51/52/31/32 各不同，不冲突）：

```python
def _on_status(self, msg):
    for s in msg.status:
        self.pos[s.name] = s.pos      # 腿、腰都进这一个字典

# wait_status: spin 到 hip/knee/waist_pitch/waist_yaw 四个都读到才返回，否则拒动
```

### 2.3 ★ 带力保持 —— 承重关节的"停"

普通关节"停"可以不发指令；但**承重关节一松力，上身就坠落**。正确的"停"是**把最近姿态用 `spd=0, cur=20A` 再发一遍**——位置锁住、力还在：

```python
def hold(self):
    if self._last is not None:
        self._publish(*self._last, spd=0.0)   # ★ spd=0 但 cur 仍是 20A，绝不设 0
```

`Ctrl-C`、结束、异常——**任何时候的"停"都调 `hold()`**，绝不让腿瘫软。

## 3. 代码解读（核心）

`atom23_leg_ros2.py`（简洁版，已瘦身）全文 = **7 个模块**。它比其它原子复杂，因为要**跨两个话题控 4 个电机 + 插值 + 带力保持**。

### 3.1 模块地图

| # | 模块 | 代码锚点 | 职责 |
|---|---|---|---|
| 1 | 配置常量 | `LEG_CMD` / `WAIST_CMD` / `HIP..YAW` / `SQUAT_DELTA` | 话题、电机ID、幅度/速度/电流 |
| 2 | 建节点与收发 | `LegDemo.__init__` | 2 个 publisher + 2 个订阅（**合用一个回调**） |
| 3 | 读状态 | `_on_status` / `wait_status` | 腿腰状态存进一个 `pos`；读齐 4 个才动 |
| 4 | 下发一组姿态 | `_publish` | 一组目标角分发到腿+腰两话题，并记 `_last` |
| 5 | ★带力保持 | `hold` | 最近姿态 `spd=0, cur=20A` 再发一遍 |
| 6 | 协调插值 | `move_to` | 从当前姿态分多步线性插值逼近目标 |
| 7 | 主流程 | `confirm` / `main` / `_sig` | GO确认→读状态→下蹲→抬回→保持；Ctrl-C 也 `hold` |

### 3.2 逐模块看

- **模块 2 `__init__`**：建腿/腰两个 publisher；两个订阅**都接 `_on_status`**——因为电机 ID 全局唯一，存进同一个 `self.pos` 不冲突（这是简洁版最大的可读性简化）。`_last` 记最近下发姿态供 `hold` 用。
- **模块 4 `_publish`**：把 (hip,knee,waist,yaw) 拆成腿消息(51/52) + 腰消息(32/31) 两条分别发；每次都更新 `_last`。
- **模块 5 `hold`**：承重原子的命门——见 2.3。
- **模块 6 `move_to`**：先读当前 (hip0,knee0,waist0)，再分 `n` 步线性插值，每步 `_publish` + `sleep`。**开环插值、从当前值出发、不跳变**。

### 3.3 举一反三（务必谨慎）

```python
SQUAT_DELTA = 0.05      # 只能调更小，别调大（承重原子）
# 想动到某个"标定姿态"（绝对目标），别用简洁版——用 robust 并加更多防护、逐步逼近
```

> 注意：腿是**承重耦合关节**，和头/臂的"发了就完"完全不同——它的每个安全动作（读状态、小步、带力保持）都是防上身坠落的，别为了简化砍掉。

## 4. 改一改，看变化（务必谨慎）

| 改什么 | 会怎样 | 建议 |
|---|---|---|
| `SQUAT_DELTA` | 下蹲幅度 | 只能调**更小**，别调大 |
| `MOVE_TIME` | 单程时长 | 调大 = 更慢更稳 |
| ~~动 `yaw`~~ | 转腰 | ⚠ 本原子保持 yaw，别在升降里动它 |

## 5. 排错

| 现象 | 原因 / 处理 |
|---|---|
| 读不到 status，脚本不动 | **这是对的**（安全）：确认 body_control 已起、`/leg/status` 和 `/waist/status` 都有数据 |
| ★ 电机报错（如欠压 `12832`） | 承重供电不足或电机故障。生产版会立即**带力保持并退出**；查电量/供电，排除后再来 |
| 中途想停 | `Ctrl-C`——脚本会带力保持当前姿态，不会瘫软 |
| `import bodyctrl_msgs` 报错 | 没 source：`source /home/ubuntu/ros2ws/install/setup.bash` |

## 6. 进阶

- **生产级加固**（见 `atom/demos/atom23_leg_ros2_robust.py`）：**电机错误逐步自检**（发现即带力保持退出）、单关节**位移硬上限** `SAFETY_CAP`、线程锁保护共享状态、**标志位式信号处理**（不在信号处理器里 `shutdown`，避免竞态）、`destroy_node` 收尾。承重原子真上业务用这版。
- **命名姿态 / 坐标升降**：进一步是"升到某标定高度"或"按坐标 IK 升降"，属于后续技能/场景阶段的内容。
