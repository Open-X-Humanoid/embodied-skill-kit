#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
skill02_bottle_grasp · 全部配置常量

看见桌上/箱子上的矿泉水瓶 → 定位 → 抓取 → 搬运放下。调参只改这里。
"""

from pathlib import Path

_HERE = Path(__file__).resolve().parent
_SKILL01 = _HERE.parent / "skill01_finger_tap"   # 同一颗相机，复用已标定好的内外参文件

# ── 相机 ──
# 话题命名空间【不写死】：节点启动时扫 ROS 图自动认出来（见 camera_ns.py）。
# orbbec 驱动默认 `camera`，部分机器出厂配成 `ob_camera_head`，自动探测两者都认。
# 想强制指定（多相机 / 跳过探测）： export CAMERA_NS=<命名空间>
# 深度图 16UC1、单位 mm。
# RGB 与深度像素坐标直接对应，无需额外配准；若更换相机型号/话题需重新确认这条假设。

# ── 内参（复用已标定文件）──
INTRINSICS_FILE = _SKILL01 / "camera_intrinsics.json"
EXPECTED_SIZE = (1280, 720)

# ── 手眼外参（相机 → head_roll_link）──
EXTRINSICS_FILE = _SKILL01 / "extrinsics.json"
EXTRINSICS_GROUP = "tienyi2_l_[0.0, 0.0, 0.0]"     # 头部回零姿态下标定的外参组
# 若实际拍摄时头部俯仰角跟标定时不同，这组外参会有系统性偏差——真机跑之前先确认当前
# 头部姿态是否也是回零；不是则需要新标定一组，或接受额外误差。
CALIB_PARENT_FRAME = "head_roll_link"              # 相机外参的父 frame（中间量，内部用）
BASE_FRAME = "base"
# 圆心拟合要求"支撑面水平"，这个假设只在 base 系下才成立（z 轴垂直向上），所以
# TF(base←head_roll_link) 是硬依赖，拟合本身就在 base 系里做。

# ── YOLO 检测/分割 ──
YOLO_MODEL = "yolov8n-seg.pt"      # 用分割版而非纯检测版：要拿逐像素 mask 找瓶底边界曲线
                                    # COCO 预训练自带"bottle"类别(id 39)，不用额外训练
TARGET_CLASS_NAME = "bottle"
CONF_THRES = 0.4
INFER_PERIOD_S = 0.5               # 定位是一次快照式的，不需要高频推理，省算力

# ── 瓶底边界曲线提取（mask → 每列最下沿像素）──
CONTACT_PIXEL_INSET = 3            # 每列往上收几像素再采样深度，避开 mask 边缘本身的
                                    # 锯齿/深度失效
CONTACT_WINDOW = 7                 # 深度采样邻域边长(像素)，取中位数压噪声
BOUNDARY_EDGE_MARGIN_FRAC = 0.15   # 排除mask左右两端各15%——那是瓶身侧面切线轮廓，不是
                                    # 瓶底圆弧，混进来会把"圆柱侧面"误当成"圆弧上的点"，带偏拟合
BOUNDARY_COL_STRIDE = 4            # 每隔几列取一个边界点，省算力
MIN_BOUNDARY_POINTS = 6            # 有效边界点少于这个数就放弃本帧，拟合不可信

# ── 半径测量（独立于圆心拟合，取mask中段宽度，避开瓶颈收缩和瓶底边缘噪声）──
RADIUS_ROW_BAND = (0.5, 0.85)      # mask 包围盒高度的取样区间（0=顶 1=底）；上半段可能
                                    # 已进入瓶颈收缩区，下半段贴近底边缘噪声大，取中段最稳

# ── 固定半径圆心拟合（2自由度，比自由拟合圆(3自由度)在短弧上更良态）──
FIT_ITERS = 8                      # 高斯-牛顿迭代次数，这个问题维度低、收敛快

# ── 发布（frame=base，拟合本身就在 base 系里做，省一次下游转换）──
TARGET_TOPIC = "/skill02/target_point"    # 瓶子中轴线在支撑面上的位置估计
DIAMETER_TOPIC = "/skill02/diameter_m"    # mask 中段宽度 + 深度反算的实际直径估计(m)

# ── 多帧聚合：wait_target() 用来压 bottle_locator 逐帧输出的抖动 ──
N_SAMPLES = 10   # 凑够这么多帧目标点求中位数

# ── 箱子检测（box_locator.py，借瓶子已测出的支撑面高度当种子，不重新猜）──
BOX_TOPIC = "/skill02/box_pose"       # 箱子中心位姿(base系)，orientation只编码yaw
BOX_SIZE_TOPIC = "/skill02/box_size"  # geometry_msgs/Vector3：(长, 宽, 高)
BOX_INFER_PERIOD_S = 1.0                   # 箱子是静态的，不用像瓶子那样高频重算
BOX_SCAN_STRIDE = 4                        # 深度图全画幅反投影时的像素步长，省算力
BOX_HEIGHT_TOL = 0.02                      # 高度筛选容差(m)：落在"箱顶高度±此值"内才算箱顶点
BOX_SEARCH_RADIUS = 0.5                    # 水平搜索半径(m)，以瓶子(x,y)为锚点——假设箱子
                                            # footprint不会比这个离谱地大，排除同高度的其它家具
BOX_MIN_POINTS = 30                        # 箱顶候选点少于此数就放弃本帧，拟合不可信
# 箱子数据不会注册进 MoveIt 碰撞体，只喂给 grasp_bottle.py 自己算的安全路径点（见下方
# build_intermediate_point 相关常量）。BOX_XY_MARGIN 兜的是箱子边缘拟合本身的误差
# （深度点漏检导致的低估 + 执行到位的位置容差），跟安全中间点的余量是两层不同的量。
BOX_XY_MARGIN = 0.02                       # 拟合出的长宽各边再加一点余量(m)，覆盖边缘漏点低估
INTERMEDIATE_Y_MARGIN = 0.1                # build_intermediate_point 在箱子y方向最大边界
                                            # 基础上再加的安全余量(m)
# 箱子高度维度：box_locator.py 会算出并发布（配合 BOX_DOWN_MARGIN），但 grasp_bottle.py
# 目前只用箱子的水平投影（长/宽/朝向/中心）算安全中间点，这个高度值暂未被消费。
BOX_DOWN_MARGIN = 0.7

# ══════════════════ 手臂/灵巧手 ══════════════════
GROUP = "left_arm"
EE_LINK = "left_tcp_link"
MOVEIT_CONTROLLER = "moveit_left_arm_controller"
QP_CONTROLLER = "endpose_single_arm_qp_L_controller"
JOINTSPACE_CONTROLLER = "jointspace_arm_L_controller"
ARM_JOINT_NAMES = [               # 左臂 7 关节（判断"谁占着本臂"用，控制器切换时避让）
    "shoulder_pitch_l_joint", "shoulder_roll_l_joint", "shoulder_yaw_l_joint",
    "elbow_pitch_l_joint", "elbow_yaw_l_joint", "wrist_pitch_l_joint", "wrist_roll_l_joint",
]
VEL_SCALE = 0.1                   # MoveIt 速度/加速度缩放（慢速）
ACC_SCALE = 0.1
VEL_LIMITS = [0.5] * 7            # QP 速度：7 关节速度上限 rad/s（越小越慢）
QP_STEP = 0.04                    # QP 分段步长(m)：每段 ≤4cm，保证在 dis_err_bound 误差限内
ORI_STEP = 0.2                    # QP 分段姿态步长(rad)

HAND_CMD_TOPIC = "/inspire_hand/ctrl/left_hand"
# 真机标定值：食指对面、其余四指+拇指弯包裹，拇指旋转固定 0.0（转出，不挡其余手指收拢
# 空间）。抓紧手型 0.65 是针对"这一个瓶子+当前手指装配"实测出来的值（能稳定包住、不推倒
# 不压瘪）——★换瓶子、换手指装配都需要由合格人员在真机上重新验证。
HAND_OPEN_POSE = {"1": 1.0, "2": 1.0, "3": 1.0, "4": 1.0, "5": 1.0, "6": 0.0}
HAND_GRASP_POSE = {"1": 0.65, "2": 0.65, "3": 0.65, "4": 0.65, "5": 0.65, "6": 0.0}

# 预备姿态（7 关节角）：手臂先关节空间走到这里，再做末端接近，避免大范围姿态跨越导致
# 臂/手扭曲。None=跳过预备段（不推荐）。
# 当前值是在真机上记录的"手朝箱子、离箱 20~30cm"安全预备姿态；硬件装配或工作位变化
# 后，应由合格人员按现场标定规程重新记录并验证。
READY_JOINTS = [0.1997, 0.0649, 0.1572, -0.3731, 0.0053, 0.0892, -0.1348]   # 真机录制值

# ══════════════════ 抓握几何 ══════════════════
# 设计依据：瓶子旋转对称，不需要按目标方向算接近姿态——姿态整体固定，只有"目标位置"
# 随瓶子位置变化。四个固定量，全部靠拖动示教实测（不是靠假设算出来的）：
#   GRASP_ORIENT     手部姿态，全程不变
#   TCP_OFFSET       "瓶子实测位置" 到 "tcp 该在哪" 的完整 3D 偏移（base系向量）——直接包含了
#                    抓握高度 + 前伸距离 + 任何侧向偏移，不假设偏移只沿一个轴
#   GRASP_DIR        预抓停驻点 → 最终闭合点的方向（base系单位向量），也是接近的行进方向
#   STANDOFF_MARGIN  预抓停驻点比最终闭合点多退多远(m)，沿 -GRASP_DIR
#
# ★这四个常量是针对当前这一个瓶子 + 当前手指装配 + 当前箱子高度标定出来的固定值，换任何
#   一项都需要重新标定（见 docs/ 里的局限性说明）。
#
# 标定关系（真机标定须由合格人员按现场安全规程执行）：
#   记录预抓停驻位 (xyz_standoff, quat_standoff)、最终闭合位 (xyz_final, quat_final)，
#   以及 bottle_locator.py 同时输出的 bottle_xyz，按下面公式计算：
#     GRASP_ORIENT    = quat_final（用验证过"真的夹住"的姿态，不用停驻位的）
#     TCP_OFFSET       = bottle_xyz − xyz_final
#     GRASP_DIR        = normalize(xyz_final − xyz_standoff)
#     STANDOFF_MARGIN  = norm(xyz_standoff − xyz_final)
GRASP_ORIENT = [0.0207, -0.7051, 0.0015, 0.7088]      # 真机拖动示教标定值
TCP_OFFSET = [0.1584, -0.0587, -0.0807]                # 真机拖动示教标定值
GRASP_DIR = [0.8721, -0.4892, 0.0159]                  # 真机拖动示教标定值
STANDOFF_MARGIN = 0.10    # 真机拖动示教实测值（约10cm）——真机验证时留意这个距离手是否
                          # 真的完全清空瓶子；不放心就调大，只影响停驻点沿 GRASP_DIR
                          # 退多远，不影响其余三个已标定常量

# ── 安全中间点（大范围移动时先绕开箱子，见 grasp_bottle.py build_intermediate_point）──
# 中间点取法：
#   x = standoff（或撤回时 retreat_xyz）的 x，z = 该点的 z（不做竖直抬高，避免撞关节限位），
#   y = 箱子在 base 系 y 方向能延伸到的最大值 + INTERMEDIATE_Y_MARGIN（轴对齐包围盒公式，
#       对 length/width 具体哪个对应哪条边不敏感，箱子转多少度都适用，不需要认出具体
#       顶点、也不依赖看见箱子完整轮廓）。
# 没有真正意义上的闭环修正：手指是 fixed 关节，没有标定"缺口相对 tcp"的偏移帧，测不到
# 缺口/瓶子有没有真的对齐，只能核对"tcp 有没有精确到达下发目标"（诊断用，不做修正）。
# 精度完全依赖开环：感知准 + 上面这些几何常量标定准。
POSITION_CHECK_TOL = 0.01   # 诊断用：tcp 实际位置离下发目标 > 此值(m) 就打印提醒，不做修正

LIFT_HEIGHT = 0.05     # 抓稳后竖直提起多高(m)，兼作抓取确认动作；放下时也用这个高度下降

# ── 提起后交互式搬运（wsad+回车 微调水平位置，q 结束）──
# w=+x(远离机器人/前) s=-x(后) a=+y(机器人左) d=-y(机器人右)，同 ROS body frame 惯例
# (x前/y左/z上)，z 不在这里动，下降放下由 LIFT_HEIGHT 单独处理。
# ⚠ 纯开环：没有视觉验证新位置下方是否有支撑面，全靠人眼判断，见 docs/ 局限性说明。
PLACE_JOG_STEP = 0.01        # 每次按键的平移量(m)
PLACE_JOG_MAX_TOTAL = 0.15   # 累计水平位移上限(m)，超过拒绝该次移动，防止连续误按跑远
