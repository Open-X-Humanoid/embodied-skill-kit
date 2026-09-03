#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
skill01 · reach_check —— MoveIt 可达性预检（只规划、不执行；手臂不动，零风险）

一句话
  和 finger_tap 用同一套目标获取逻辑（收 tag → 中位数 → TF 到 base → SAFE_BOX →
  接近点=卡片中心沿法线退 APPROACH_OFFSET + 感知姿态），把同一个目标位姿交给 MoveIt
  用 plan_only=True 纯规划：规划成功 = moveit 后端可达；并打印轨迹末点 7 关节角
  （可直接抄进 motion04 的目标做慢速实跑验证）。

判读（三档姿态约束依次规划；实测 2026-07-24：严格档 99999、全松档给贴限位扭曲解，★档为正解）
  ①严格（xyz 全 ±0.05）      过 → 姿态毫无问题
  ②★方向紧自旋松（xy=0.3, z=3.14）过 → 点按任务的正解：指尖方向对准卡面、绕轴自旋自由，
                                    finger_tap moveit 后端已用同款默认约束，可直接切后端实跑
  ③全松（xyz=3.14）           过 → 仅位置可达；只有③过=姿态约束仍太紧，需调 HAND_SPIN
  三档全败 → 首查起点越界（99999 高频坑）：
       tmux capture-pane -t xarm.1 -p -J -S -400 | grep -i 'outside bounds'
     点名关节后用 QP 挪回（见 motion04 guide 排错表）；无越界则目标真够不着/有碰撞。
  ★每档成功都打印轨迹末点 7 关节角，并对照 URDF 软限位标记贴限位的关节（|余量|<0.1rad ⚠）。

跑在哪（x86；前提：Orin 的 tag_locator 在跑、XARM 本体+MoveIt 组件在跑）
  source /home/ubuntu/XARM/install/setup.bash
  python3 skill/skill01_finger_tap/reach_check.py
"""

import numpy as np
import rclpy
from geometry_msgs.msg import Pose

import config as C
import pose_math as PM
from finger_tap import FingerTap, MOVE_ACTION   # 复用：收目标/TF/安全盒/约束构造

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, PlanningOptions

# 轨迹末点按此顺序打印（= motion04 JOINT_NAMES 顺序，可直接抄作它的目标）
JOINT_ORDER = [
    "shoulder_pitch_l_joint", "shoulder_roll_l_joint", "shoulder_yaw_l_joint",
    "elbow_pitch_l_joint", "elbow_yaw_l_joint", "wrist_pitch_l_joint", "wrist_roll_l_joint",
]

# 左臂 URDF 软限位（同 motion03_robust 表；用于标记"贴限位的扭曲解"）
URDF_LIMITS = {
    "shoulder_pitch_l_joint": (-2.96, 2.96), "shoulder_roll_l_joint": (-0.26, 2.61),
    "shoulder_yaw_l_joint": (-2.96, 2.96),   "elbow_pitch_l_joint": (-2.61, 0.26),
    "elbow_yaw_l_joint": (-2.96, 2.96),      "wrist_pitch_l_joint": (-0.78, 1.04),
    "wrist_roll_l_joint": (-1.65, 1.30),
}


def fmt_joints(joints):
    """按 motion04 顺序格式化关节角；贴限位（余量<0.1rad）的标 ⚠。"""
    out = []
    for name in JOINT_ORDER:
        v = joints[name]
        lo, hi = URDF_LIMITS[name]
        mark = "⚠" if (v - lo < 0.1 or hi - v < 0.1) else ""
        out.append(f"{v:+.3f}{mark}")
    return "[" + ", ".join(out) + "]（⚠=贴 URDF 限位，构型可能扭曲）" if "⚠" in "".join(out) \
        else "[" + ", ".join(out) + "]"


def plan_only(node, pose, ori_tol, ori_tol_z=None):
    """纯规划（plan_only=True，不执行）。返回 (error_code, 末点关节dict或None)。"""
    req = MotionPlanRequest()
    req.group_name = C.GROUP
    req.num_planning_attempts = 10
    req.allowed_planning_time = 5.0
    req.goal_constraints.append(node._pose_goal(pose, ori_tol=ori_tol, ori_tol_z=ori_tol_z))
    goal = MoveGroup.Goal(request=req, planning_options=PlanningOptions(plan_only=True))
    send = node.moveit_client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send, timeout_sec=10.0)
    gh = send.result()
    if gh is None or not gh.accepted:
        return None, None
    rf = gh.get_result_async()
    rclpy.spin_until_future_complete(node, rf, timeout_sec=30.0)
    res = rf.result()
    if res is None:
        return None, None
    code = res.result.error_code.val
    if code != 1:
        return code, None
    jt = res.result.planned_trajectory.joint_trajectory
    last = dict(zip(jt.joint_names, jt.points[-1].positions))
    return 1, last


def main():
    rclpy.init()
    node = FingerTap()   # 复用其订阅/TF/安全盒；本工具不使能、不切控制器、不动臂
    log = node.get_logger()
    log.info("reach_check · MoveIt 可达性预检（plan_only，臂不动）")

    try:
        if not node.moveit_client.wait_for_server(timeout_sec=10.0):
            log.error(f"MoveGroup {MOVE_ACTION} 不在——MoveIt 组件起了吗（start_xarm.sh real）？")
            return
        pt_head, n_head = node.wait_target()
        if pt_head is None:
            return
        pt_base, n = node.to_base(pt_head, n_head)
        if pt_base is None:
            return
        if n[0] > 0:          # 法线兜底：应大致指向机器人，反了翻转（同 finger_tap）
            n = -n
        p = pt_base.point
        log.info(f"卡片中心(base) = ({p.x:+.3f}, {p.y:+.3f}, {p.z:+.3f})")
        if not node.check_safe(pt_base):
            return
        approach, quat, _ = node.build_approach(p, n)  # 与 finger_tap 完全同源（含指尖补偿）
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = map(float, approach)
        (pose.orientation.x, pose.orientation.y,
         pose.orientation.z, pose.orientation.w) = quat
        log.info(f"预检目标 = tcp 目标点({approach[0]:+.3f}, {approach[1]:+.3f}, {approach[2]:+.3f})"
                 " + 感知姿态（已含指尖补偿）")

        # ── 三档姿态约束依次预检 ──
        code1, j1 = plan_only(node, pose, ori_tol=0.05)
        if code1 == 1:
            log.info("① 严格姿态（±0.05）：✓ 过")
            log.info(f"   末点关节角: {fmt_joints(j1)}")
        else:
            log.warn(f"① 严格姿态（±0.05）：✗ error_code={code1}")

        code2, j2 = plan_only(node, pose, ori_tol=0.3, ori_tol_z=C.SPIN_TOL)
        if code2 == 1:
            log.info(f"②★方向紧自旋适度（xy=0.3, z={C.SPIN_TOL}，绕 HAND_SPIN 名义值）：✓ 过 —— "
                     "finger_tap moveit 后端同款约束，可直接切 ARM_BACKEND=\"moveit\" 实跑")
            log.info(f"   末点关节角（可抄去 motion04 慢速实跑验证）: {fmt_joints(j2)}")
            return
        log.warn(f"②★方向紧自旋松：✗ error_code={code2}")

        code3, j3 = plan_only(node, pose, ori_tol=3.14)
        if code3 == 1:
            log.info("③ 全松（仅位置）：✓ 过 —— 位置够得着，但连'指尖对准卡面'都规划不出，"
                     "调 config.HAND_SPIN 或把卡片挪到更正对手臂的位置再试")
            log.info(f"   末点关节角: {fmt_joints(j3)}")
            return
        log.error(
            f"✗ 三档全败（严格={code1} / ★={code2} / 全松={code3}）。按序排查：\n"
            "  ① 起点越界（99999 高频坑）："
            "tmux capture-pane -t xarm.1 -p -J -S -400 | grep -i 'outside bounds'\n"
            "     有点名关节 → 用 QP 挪回（motion04 guide 排错表 99999 行）后重跑本工具\n"
            "  ② 无越界 → 目标真够不着/有碰撞：挪近卡片或调整预备姿态")
    except KeyboardInterrupt:
        log.warn("用户中断")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
