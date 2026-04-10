#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
YSC ZED - Camera extrinsic calibration
将 ZED SDK 输出的相机位姿（左目为原点）变换到机器狗 base_link 位姿。

坐标系约定（ZED RIGHT_HANDED_Z_UP）：
  - ZED 相机：X=右, Y=前, Z=上
  - base_link ：X=前, Y=左, Z=上

CAM_TO_BASE_PARAMS 中的平移量从 base_link 原点量到左目镜头中心（米），
安装偏角为相机实际安装相对零偏角（正摆、水平、无侧倾）的偏差。
"""

import math
import threading


# ================= 外参安装参数（按实测修改） =================
CAM_TO_BASE_PARAMS = {
    "tx": 0.38,         # 相机在 base_link 前向（X）偏移，米
    "ty": 0.065,        # 相机在 base_link 左向（Y）偏移，米
    "tz": 0.18,         # 相机在 base_link 上向（Z）偏移，米
    "roll_deg": 0.0,    # 绕相机 Y 轴（前向）右倾为正，度
    "pitch_deg": 45.0,  # 绕相机 X 轴（右向）低头为正，度
    "yaw_deg": 0.0,     # 绕相机 Z 轴（上向）逆时针为正，度
}


def _matmul3(A, B):
    """3x3 矩阵乘法（A @ B）。"""
    return [
        [
            A[0][0] * B[0][0] + A[0][1] * B[1][0] + A[0][2] * B[2][0],
            A[0][0] * B[0][1] + A[0][1] * B[1][1] + A[0][2] * B[2][1],
            A[0][0] * B[0][2] + A[0][1] * B[1][2] + A[0][2] * B[2][2],
        ],
        [
            A[1][0] * B[0][0] + A[1][1] * B[1][0] + A[1][2] * B[2][0],
            A[1][0] * B[0][1] + A[1][1] * B[1][1] + A[1][2] * B[2][1],
            A[1][0] * B[0][2] + A[1][1] * B[1][2] + A[1][2] * B[2][2],
        ],
        [
            A[2][0] * B[0][0] + A[2][1] * B[1][0] + A[2][2] * B[2][0],
            A[2][0] * B[0][1] + A[2][1] * B[1][1] + A[2][2] * B[2][1],
            A[2][0] * B[0][2] + A[2][1] * B[1][2] + A[2][2] * B[2][2],
        ],
    ]


def _matvec3(A, v):
    """3x3 矩阵乘 3x1 向量（A @ v）。"""
    return [
        A[0][0] * v[0] + A[0][1] * v[1] + A[0][2] * v[2],
        A[1][0] * v[0] + A[1][1] * v[1] + A[1][2] * v[2],
        A[2][0] * v[0] + A[2][1] * v[1] + A[2][2] * v[2],
    ]


def _euler_to_R(roll_deg: float, pitch_deg: float, yaw_deg: float):
    """
    安装偏角（度）→ 3x3 额外旋转矩阵。
    约定：
      roll_deg  : 绕相机 Y 轴（前向）右倾为正
      pitch_deg : 绕相机 X 轴（右向）低头为正
      yaw_deg   : 绕相机 Z 轴（上向）逆时针为正
    返回 R = Rz(yaw) @ Rx(pitch) @ Ry(roll)
    """
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)

    cy, sy = math.cos(yaw), math.sin(yaw)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cr, sr = math.cos(roll), math.sin(roll)

    Rz = [[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]]
    Rx = [[1.0, 0.0, 0.0], [0.0, cp, -sp], [0.0, sp, cp]]
    Ry = [[cr, 0.0, sr], [0.0, 1.0, 0.0], [-sr, 0.0, cr]]

    return _matmul3(Rz, _matmul3(Rx, Ry))


def _quat_to_R(qx, qy, qz, qw):
    """
    四元数 → 3x3 旋转矩阵 R_world_cam。
    将相机坐标系中的向量旋转到 ZED 世界坐标系。
    """
    xx, yy, zz = qx * qx, qy * qy, qz * qz
    xy, xz, yz = qx * qy, qx * qz, qy * qz
    wx, wy, wz = qw * qx, qw * qy, qw * qz
    return [
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
        [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
    ]


def _build_cam_base_RT(params: dict):
    """
    由安装参数构建外参矩阵 T_cam_base（base_link → 相机坐标系）。

    返回：
      R_cam_base : 3x3，把 base_link 坐标系向量转到相机坐标系
      t_cam_base : 3x1，base_link 原点在相机坐标系中的坐标
    """
    tx = float(params["tx"])
    ty = float(params["ty"])
    tz = float(params["tz"])
    roll_deg = float(params["roll_deg"])
    pitch_deg = float(params["pitch_deg"])
    yaw_deg = float(params["yaw_deg"])

    R_CAM_FROM_BASE_NOMINAL = [
        [0.0, -1.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
    ]

    R_mount = _euler_to_R(roll_deg, pitch_deg, yaw_deg)
    R_cam_base = _matmul3(R_mount, R_CAM_FROM_BASE_NOMINAL)

    t_base = [tx, ty, tz]
    Rt = _matvec3(R_cam_base, t_base)
    t_cam_base = [-Rt[0], -Rt[1], -Rt[2]]

    return R_cam_base, t_cam_base


class ZedExtrinsic:
    """
    ZED 相机外参标定器。

    每帧由 ZED 线程调用 update()，将相机位姿变换为 base_link 位姿；
    校准线程或导航函数通过属性或 get() 读取结果。
    """

    def __init__(self, params: dict):
        self._lock = threading.Lock()
        self._R_cam_base, self._t_cam_base = _build_cam_base_RT(params)

        self.base_tx = None
        self.base_ty = None
        self.base_heading_cw = None
        self.base_forward_xy = None
        self.base_left_xy = None

    def update(self, qx, qy, qz, qw, zed_tx: float, zed_ty: float):
        """
        ZED 线程每帧调用。
        输入：ZED SDK 给出的相机四元数姿态和左目平移量（ZED 世界系，米）。
        """
        R_world_cam = _quat_to_R(qx, qy, qz, qw)
        base_R_world = _matmul3(R_world_cam, self._R_cam_base)
        t_in_world = _matvec3(R_world_cam, self._t_cam_base)

        fx = base_R_world[0][0]
        fy = base_R_world[1][0]
        lx = base_R_world[0][1]
        ly = base_R_world[1][1]

        with self._lock:
            self.base_tx = zed_tx + t_in_world[0]
            self.base_ty = zed_ty + t_in_world[1]
            self.base_heading_cw = math.degrees(math.atan2(fx, fy))
            self.base_forward_xy = (fx, fy)
            self.base_left_xy = (lx, ly)

    def get(self):
        """
        线程安全地一次性读取全部属性的快照。
        返回：(base_tx, base_ty, base_heading_cw, base_forward_xy, base_left_xy)
        """
        with self._lock:
            return (
                self.base_tx,
                self.base_ty,
                self.base_heading_cw,
                self.base_forward_xy,
                self.base_left_xy,
            )


cam_extrinsic = ZedExtrinsic(CAM_TO_BASE_PARAMS)
