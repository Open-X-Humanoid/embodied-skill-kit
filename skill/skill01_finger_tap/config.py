#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
skill01_finger_tap · 全部配置常量（两板共用；调参只改这里，不动逻辑代码）
"""

from pathlib import Path

_HERE = Path(__file__).resolve().parent

# ── AprilTag（实测）──
TAG_FAMILY = "tag36h11"
TAG_ID = 2                        # april_36h11-2
TAG_SIZE = 0.04                   # 黑框外缘边长(m)，实测 4cm；填错则深度按比例错

# ── 相机 ──
# 话题命名空间【不写死】：节点启动时扫 ROS 图自动认出来（见 camera_ns.py）。
# orbbec 驱动默认 `camera`，部分机器出厂配成 `ob_camera_head`，自动探测两者都认。
# 想强制指定（多相机 / 跳过探测）： export CAMERA_NS=<命名空间>
INTRINSICS_FILE = _HERE / "camera_intrinsics.json"
EXPECTED_SIZE = (1280, 720)       # 内参标定时的分辨率(w,h)；实际流不同会自动等比缩放内参并告警

# ── 手眼外参（相机 → head_roll_link，按头部姿态分组标定）──
EXTRINSICS_FILE = _HERE / "extrinsics.json"
EXTRINSICS_GROUP = "tienyi2_l_[0.0, 0.0, 0.0]"   # 头部回零的标定组；demo 时头俯仰不同则换组
CALIB_PARENT_FRAME = "head_roll_link"            # 外参矩阵的父 frame（URDF/TF 树里存在）

# ── frame / 跨板话题 ──
BASE_FRAME = "base"
TARGET_TOPIC = "/skill01/target_point"        # tag_locator 发、finger_tap 收（PoseStamped：position=中心, orientation.z 轴=法线）

# ── 阶段2/3 用（手臂/手指，阶段1不读）──
APPROACH_OFFSET = 0.08            # 伸手停驻点：目标前方 8cm（阶段2 只到这、不接触）
N_SAMPLES = 10                    # 取多帧目标点求中位数（压检测抖动与离群帧）

# ── 阶段3：点按（从停驻点前进按下 → 保持 → 退回）──
PRESS_ENABLE = True               # False=只做阶段2（伸到 8cm 停），不按下
PRESS_DEPTH = 0.015               # 过冲深度：指肚触卡面后再进 1.5cm，由手臂柔顺（末端阻抗）
                                  # 吸收——像手指按硬物自然微弯，不硬顶。★真机首验从 0.005 起、
                                  # 急停在手；QP 触阻力推不动会超时→代码判为"已按到"停下、不报错，
                                  # 这是主安全网（实测 QP 卡阻确会超时）。确认柔顺吸收正常再加大。
# 前进总量 = APPROACH_OFFSET + PRESS_DEPTH：从停驻点推进到卡面并过冲。
# 按到后不靠定时"保持"，而是停住等人工回车确认点按姿态（见 finger_tap.py press_only）。

# ── 点按手型（skill 的"手+臂协调"：抬臂前先摆手型）──
# 只伸食指（+拇指）点按、其余三指蜷起：① 抬臂/接近时其它手指不再蹭到卡片所在的板面
# ② 食指成唯一凸出点，指尖补偿(PAD_LOCAL_OFFSET)更干净、更像人"用食指点"的姿势。
# 用因时 Inspire 手接口（见 atom/motion/motion09_hand_ros2.py）：position=张合百分比，
# 1.0=完全张开/伸直，0.0=握紧；手指ID "1"小指 "2"无名 "3"中 "4"食 "5"拇弯 "6"拇旋。
# ★食指须保持伸直(4=1.0)：URDF 里手指是 fixed 关节，left_index_2 的 TF 按"伸直"算，
#   PAD_LOCAL_OFFSET 也是在伸直姿态的网格上标定的——食指蜷了补偿就失准。蜷其余指不影响
#   食指 TF（各指独立）。
HAND_POSE_ENABLE = True           # False=不控手（保持当前手型，回到老行为）
HAND_CMD_TOPIC = "/inspire_hand/ctrl/left_hand"
POINT_POSE = {"1": 0.2, "2": 0.2, "3": 0.2, "4": 1.0, "5": 1.0, "6": 1.0}
                                  # 小/无名/中指蜷到 0.2（留机械余量不顶死）、食指伸直、拇指伸展

# 工作空间白名单（base 系，米）：目标点出盒直接拒动。审的是感知输出的数字（防 TAG_SIZE/
# 外参错导致的整体偏移、误检），不管卡片摆放自由。
# None=关闭（2026-07 拍板：可控环境不设固定位置约束，摆卡自由；此时动前回车确认是唯一
# 人工闸，★回车前务必核对打印的目标坐标；可达性由 reach_check/MoveIt 规划失败兜底）。
# 要重开就换回三轴范围，建议描"手臂工作区"而非"卡片位置"（宽松、几乎不用再改）：
# SAFE_BOX = {"x": (0.25, 0.95), "y": (-0.7, 0.7), "z": (0.7, 1.6)}
SAFE_BOX = None

# 手臂后端："moveit"=MoveIt 一步规划（motion05 套路，真机已验证，默认）；"qp"=QP 末端分段喂
# （motion07 套路，留作对照——⚠长距扫掠会把手拖着撞身体：QP 碰撞模型不含灵巧手，实测在
# 垂臂 READY 下 24 段长扫第 9 段卡手超时；短程（如到位修正 2~3cm）是它的舒适区）。
# moveit 报 99999 的主因已实锤=起点越界（MoveIt 限位比 URDF 紧，QP/遥控跑完常触发；
# xarm.1 日志 grep 'outside bounds' 点名关节、QP 挪回即恢复，见 motion04 guide 排错表 99999 行）
ARM_BACKEND = "moveit"
QP_CONTROLLER = "endpose_single_arm_qp_L_controller"
JOINTSPACE_CONTROLLER = "jointspace_arm_L_controller"
QP_STEP = 0.04                    # QP 分段步长(m)：每段 ≤4cm，保证在 dis_err_bound 误差限内
ORI_STEP = 0.2                    # QP 分段姿态步长(rad)：手腕姿态每段最多转 ~11°（slerp 渐变）
# ── 姿态模式（决定"手指朝哪、掌心朝哪"）──
ORIENT_MODE = "level"             # "level"=手指【水平】指向卡片（法线的水平投影；像人按面前按钮，
                                  #   姿态相对 base 确定、每次一致——推荐）
                                  # "tag"=完全跟随卡面法线（斜卡片垂直接近；卡片近朝上时法线噪声大、姿态发飘）
HAND_SPIN = 0.0                   # 绕指向轴的自旋(rad)：决定掌心朝向的名义值。level 模式下相对 base
                                  # 固定，调一次全程可复现；不对就按 ±0.5 步进调。
                                  # ★由权威 URDF 推算的拨盘表（左手、手指水平指向卡片时）：
                                  #   0=掌心朝下(按按钮)  +1.57=朝内  3.14=朝上  -1.57=朝外(外翻病态解)
                                  # 推导：手指限位只允许朝掌心弯→掌心=L_base +x=tcp −y；level 模式 spin=0
                                  # 时 tcp y 轴严格朝上→掌心朝下。换算链已用指尖 TF 实测值毫米级验证。
                                  # 旧经验值 2.5(≈掌心朝上偏内)是"钉死乱挑解"的过渡值。
                                  # ★2026-07-24 真机验收：0.0 时掌心朝地面，与推导一致，钉死。
SPIN_TOL = 1.0                    # MoveIt 对自旋的容差(rad)：1.0≈±57°——放宽以更易找解（够不到/
                                  # 过约束 99999 时用）；越大 MoveIt 越可能挑到掌心外翻的解，够用即可
LEVEL_WRIST = True                # 按下前把手腕强制摆到名义水平朝向。
                                  # ★2026-08：本开关当初是为了消掉旧 pad 模型（沿 tcp→index_2 连线做
                                  # 标量外推）的"姿态相关误差"——那个近似只在名义姿态下标定准，手掌一歪
                                  # 指肚就离卡面差几毫米。现在 read_pad_pos 已改为"TAP_LINK 实际姿态 +
                                  # 局部固定偏移 PAD_LOCAL_OFFSET"，偏移随手指刚体旋转，姿态怎么变理论上
                                  # 都物理正确，本开关存在的必要性存疑，但尚未真机重新验证，暂保留、不改
                                  # 默认值。False=用实际朝向（老行为）

# ── 指尖补偿（灵巧手装在 tcp 之后；不补偿则规划以腕端为准，手会冲过目标）──
# 原理：URDF 里灵巧手经 fixed 关节挂在 left_tcp_link 下，tcp→TAP_LINK 的偏移（旋转+平移）
# 恒定，查静态 TF 即得；再叠加 TAP_LINK 局部系下"指肚相对骨架点"的固定偏移
# PAD_LOCAL_OFFSET，两段合成即为 tcp→指肚 的完整偏移。把 tcp 目标沿该偏移后撤 → 指肚
# （而非骨架点/腕端）正好落在接近点。PAD_LOCAL_OFFSET 随 TAP_LINK 实际姿态刚体旋转，
# 不再像旧的标量外推那样假设"沿 tcp→TAP_LINK 连线方向"，姿态怎么变都物理正确。
TAP_LINK = "left_index_2"         # 点按用指尖的 URDF link（换手指改这里；置 "" 关闭补偿）
# 指肚相对 TAP_LINK（left_index_2）局部系原点的固定偏移(m)：由 standard/left_index_2.STL
# 网格算出——取局部 +y 轴（手指长轴，由网格 bbox 判断）方向上的支撑函数极值顶点。
# 验证过该点非孤立毛刺（3mm 邻域 160 个顶点、投影值平缓）、且对方向扰动稳健（±5°锥内
# 500 次随机采样 >75% 落在同一片 ≤1mm 范围内）——因此不需要像早期草稿那样先精确推算
# "真实按压方向"（要绕 ORIENT_MODE/HAND_SPIN/关节链）再投影，网格自身长轴就够稳。
# ★2026-08：仅离线网格计算，尚未真机验证——上机后建议尺量指肚实际悬空位置核对，
#   并确认这版 STL 与真机当前装配的手指一致。
PAD_LOCAL_OFFSET = [0.01365, 0.04307, 0.00499]
VEL_LIMITS = [1.0] * 7            # QP 速度：7 关节速度上限 rad/s（越小越慢；两个控制器都设）。
                                  # 1.0 比早期 0.5 快一倍——★首次真机跑先确认路径无人、急停在手

# ── 到位闭环修正（治"同一目标每次落点差 ±2~3cm"）──
# 病根实测（2026-07-24 三连跑）：感知三次完全一致，Δ 却 1.5/2.7/3.7cm 方向各异——
# MoveIt 姿态容差(±17°方向/±34°自旋)×腕→指尖 17cm 杠杆 = 指尖随机散布。
# 修法：到位后 TF 实测 Δ → QP 短程平移 −Δ（姿态保持当前实际值，不给容差第二次机会）→ 重测。
TIP_CORRECT = True                # False=关闭闭环修正（回到一步开环）
CORRECT_TOL = 0.004               # |Δ| ≤ 此值(m)视为到位，不修/停止迭代。4mm：闭环多补一两轮压更准
CORRECT_MAX = 4                   # 最多修正轮数（每轮后重测 Δ）。实测 2 轮不够：
                                  # 4.9→3.7→1.6cm 仍在收敛就被掐停（2026-07-24）

# ── 固定瞄准偏置（base 系，米）──
# 吸收两个"闭环治不了"的固定横向/纵向误差：① 手眼外参的固定偏差（目标点本身就偏一截）；
# ② left_index_2 帧 ≠ 指肚实际接触点的固定差。闭环只能把 index_2 帧驱到"算出来的目标"，
# 但目标本身偏、且帧≠指肚 → 物理指肚稳定偏一截、且方向固定（实测三跑 Δy 恒负）。
# ★标定：跑一次到位后，尺量"指肚 vs 你真想按的点"的偏移，填到这里（符号见下），指肚会
#   反向移动同样的量。base 系：x=前后(depth)、y=机器人左(+)/右(-)、z=上(+)/下(-)。
#   仅对当前 EXTRINSICS_GROUP/头部姿态有效；换头部姿态要重标。
AIM_BIAS_BASE = [0.0, 0.005, 0.005]  # 冻结目标实测：y=0.02 时落点稳定左偏 1.5cm → y 减 0.015 到 0.005 抵消；
                                     # z=0.005 补此前 ~5mm 偏下（真机已设）。
                                     # 调法（1cm=0.01）：食指偏右→y 增大、偏左→y 减小；
                                     # 偏下→z 增大、偏上→z 减小。再跑还偏就继续微调。

# ★预备姿态（7 关节角，J1肩俯仰→J7腕旋转）：手臂先关节空间走到这里（姿态自然、手朝前、
# 离卡片 20~30cm），再做末端接近——避免"保持垂臂手腕朝向跨大范围"导致的臂/手扭曲。
# None=跳过预备段（不推荐，会扭曲）。录制方法见 finger_tap.py 文档注释。
# 当前值=2026-07-24 真机记录的垂臂初始态。可用但非理想：
# 末端接近段会比较长；若实跑中后段 QP 超时/手腕扭曲，优先换成"手朝卡片、离卡 20~30cm"
# 的安全预备姿态，并按 finger_tap.py 文件头说明记录关节角。
READY_JOINTS = [-0.0533, 0.1279, -0.0492, -0.0651, 0.7732, -0.0382, -0.0278]

# 手臂（MoveIt 末端，同 motion05 已验证参数）
GROUP = "left_arm"
EE_LINK = "left_tcp_link"
MOVEIT_CONTROLLER = "moveit_left_arm_controller"
ARM_JOINT_NAMES = [               # 左臂 7 关节（判断"谁占着本臂"用，控制器避让）
    "shoulder_pitch_l_joint", "shoulder_roll_l_joint", "shoulder_yaw_l_joint",
    "elbow_pitch_l_joint", "elbow_yaw_l_joint", "wrist_pitch_l_joint", "wrist_roll_l_joint",
]
VEL_SCALE = 0.1                   # MoveIt 速度/加速度缩放（慢速）
ACC_SCALE = 0.1
