#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
skill01 · 姿态小工具（纯 numpy，无额外依赖）
四元数一律 [x, y, z, w] 顺序（与 geometry_msgs/Quaternion 字段一致）。
"""

import numpy as np


def quat_to_mat(q):
    """四元数 → 3×3 旋转矩阵。"""
    x, y, z, w = np.asarray(q, dtype=float)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def mat_to_quat(R):
    """3×3 旋转矩阵 → 四元数（Shepperd 法，数值稳定）。"""
    R = np.asarray(R, dtype=float)
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        w, x = 0.25 * s, (R[2, 1] - R[1, 2]) / s
        y, z = (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w, x = (R[2, 1] - R[1, 2]) / s, 0.25 * s
        y, z = (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w, x = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s
        y, z = 0.25 * s, (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w, x = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s
        y, z = (R[1, 2] + R[2, 1]) / s, 0.25 * s
    q = np.array([x, y, z, w])
    return q / np.linalg.norm(q)


def rotate_vec(q, v):
    """用四元数旋转向量。"""
    return quat_to_mat(q) @ np.asarray(v, dtype=float)


def slerp(q0, q1, t):
    """四元数球面插值，t∈[0,1]。姿态渐变的标准做法（分段移动时手腕不突变）。"""
    q0 = np.asarray(q0, float) / np.linalg.norm(q0)
    q1 = np.asarray(q1, float) / np.linalg.norm(q1)
    d = float(np.dot(q0, q1))
    if d < 0:                       # 取短弧
        q1, d = -q1, -d
    if d > 0.9995:                  # 近乎同向：线性插值防除零
        q = q0 + t * (q1 - q0)
        return q / np.linalg.norm(q)
    th = np.arccos(np.clip(d, -1.0, 1.0))
    return (np.sin((1 - t) * th) * q0 + np.sin(t * th) * q1) / np.sin(th)


def quat_angle(q0, q1):
    """两姿态间的夹角(rad)。"""
    d = abs(float(np.dot(np.asarray(q0, float), np.asarray(q1, float))))
    return 2.0 * np.arccos(np.clip(d, -1.0, 1.0))


def quat_to_rpy(q):
    """四元数 [x,y,z,w] → (roll, pitch, yaw) rad（ZYX，与 URDF rpy 同约定）。仅供日志人看。"""
    x, y, z, w = np.asarray(q, dtype=float)
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1.0, 1.0))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return np.array([roll, pitch, yaw])


def quat_from_zaxis(z_axis, up=(0.0, 0.0, 1.0), spin=0.0):
    """构造"z 轴对准给定方向"的姿态：
    z = z_axis；x 取与世界 up 的水平叉积（消除绕 z 自旋，手不歪着拧）；
    spin = 绕 z 的附加自旋(rad)，是相对量、可微调。z_axis 近竖直时退化用 x 轴兜底。"""
    z = np.asarray(z_axis, dtype=float)
    z = z / np.linalg.norm(z)
    x = np.cross(np.asarray(up, dtype=float), z)
    if np.linalg.norm(x) < 1e-6:    # 法线近竖直（卡片平躺）——用世界 x 兜底
        x = np.cross([1.0, 0.0, 0.0], z)
    x = x / np.linalg.norm(x)
    y = np.cross(z, x)
    R = np.column_stack([x, y, z])
    if abs(spin) > 1e-9:
        c, s = np.cos(spin), np.sin(spin)
        R = R @ np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    return mat_to_quat(R)
