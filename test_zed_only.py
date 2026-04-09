#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
zed_only.py — 纯视觉导航（仅使用 ZED 相机，无 RTK/GPS）

坐标系约定（ZED RIGHT_HANDED_Z_UP）：
  - X 轴：向右，Y 轴：向前，Z 轴：向上
  - Yaw 正方向：逆时针（俯视），右手坐标系
  - 坐标原点：程序启动时 ZED 的初始位置

所有位置和航向均基于 ZED 视觉里程计，单位：米 / 度。
航点文件格式：[[x1, y1], [x2, y2], ...]（base_link 在 ZED 世界坐标系，米）
"""

import time
import sys
import datetime
import threading
import math
import json
import os

import pyzed.sl as sl
import matplotlib.pyplot as plt
from flask import Flask, jsonify
from flask_cors import CORS

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.b2.sport.sport_client import SportClient


# ================= ZED 坐标约定 =================
# yaw_raw: 逆时针为正（右手系）
# 导航内部统一使用顺时针航向（CW），故取 ZED_YAW_SIGN = -1
# current_heading_cw = ZED_YAW_SIGN * zed_yaw_raw
ZED_YAW_SIGN = -1

# ================= 全局变量 - ZED =================
zed_yaw_raw = None      # ZED 原始 yaw（度，CCW 正）
zed_tx = None           # ZED 当前 x 位置（右方向，米）
zed_ty = None           # ZED 当前 y 位置（前方向，米）
zed_ready = False       # ZED 是否已就绪

# ================= 外参：相机 -> 机械狗 base_link =================
# 你提供的参数含义与 `calib_zed_base.py` 保持一致：
#   - base_link：X=前, Y=左, Z=上
#   - ZED 相机：X=右, Y=前, Z=上
#   - params：相机在 base_link 下的平移(tx,ty,tz) + 安装偏角(roll/pitch/yaw)
#
# 运行时把 ZED SDK 给出的“相机在 ZED 世界系的位姿”转换成“base_link 在同一世界系的位姿”，
# 用于距离/航向计算、航点记录与 /api/location 输出。
CAM_TO_BASE_PARAMS = {
    "tx": 0.41,
    "ty": 0.065,
    "tz": 0.285,
    "roll_deg": 1.0,
    "pitch_deg": 9.3,
    "yaw_deg": -1.9,
}

# base_link 在“ZED 世界坐标系(X=右,Y=前,Z=上)”下的实时量
base_tx = None          # base_link 当前 x（世界系右方向，米）
base_ty = None          # base_link 当前 y（世界系前方向，米）
base_heading_cw = None # base_link 当前航向（顺时针为正，度）
base_forward_xy = None # base_link 前向(X=前)在世界XY的分量 (forward_x_right, forward_y_front)
base_left_xy = None    # base_link 左向(Y=左)在世界XY的分量 (left_x_right, left_y_front)

# ================= 机械狗状态机 =================
STATE_IDLE   = 1
STATE_CRUISE = 2
STATE_PAUSE  = 3
STATE_RETURN = 4
STATE_CHARGE = 5

robot_state = STATE_IDLE
current_wp_index = 0
waypoints = []
return_waypoints = []
return_index = 0
target_pause_wp_index = None
target_pause_done = False

# ================= 控制参数 =================
max_yaw_rate  = 0.4
max_vx        = 0.6
kp_dist       = 0.9
arrival_radius = 0.1
vx_current    = 0.0
# 正误差（当前顺时针偏多）→ 需逆时针修正 → 正 vyaw（sport_client 中 vy 正=左/CCW）
VYAW_SIGN = 1

# ================= 文件通信 =================
PAUSE_STATE_FILE = "tmp/robotdog_nav_pause_state.json"
NAV_STATE_FILE   = "tmp/robotdog_nav_state.json"


# ================= 工具函数 =================
def normalize_angle(angle):
    """归一化到 (-180, 180]"""
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle


# ================= 坐标变换工具（纯 python，不依赖 numpy） =================
def _matmul3(A, B):
    """3x3 矩阵乘法（A@B）"""
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
    """3x3 矩阵乘 3x1 向量（A@v）"""
    return [
        A[0][0] * v[0] + A[0][1] * v[1] + A[0][2] * v[2],
        A[1][0] * v[0] + A[1][1] * v[1] + A[1][2] * v[2],
        A[2][0] * v[0] + A[2][1] * v[1] + A[2][2] * v[2],
    ]


def _euler_to_R(roll_deg: float, pitch_deg: float, yaw_deg: float):
    """
    安装偏角（度）→ 3x3 额外旋转矩阵 D。
    约定与 `calib_zed_base.py` 保持一致：
      - roll_deg  : 绕相机 Y 轴（前向）右倾为正
      - pitch_deg : 绕相机 X 轴（右向）低头为正
      - yaw_deg   : 绕相机 Z 轴（上向）逆时针为正
    返回 R = Rz(yaw) @ Rx(pitch) @ Ry(roll)
    """
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)

    cy = math.cos(yaw)
    sy = math.sin(yaw)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cr = math.cos(roll)
    sr = math.sin(roll)

    Rz = [
        [cy, -sy, 0.0],
        [sy,  cy, 0.0],
        [0.0, 0.0, 1.0],
    ]
    Rx = [
        [1.0, 0.0, 0.0],
        [0.0,  cp, -sp],
        [0.0,  sp,  cp],
    ]
    Ry = [
        [cr, 0.0, sr],
        [0.0, 1.0, 0.0],
        [-sr, 0.0, cr],
    ]

    return _matmul3(Rz, _matmul3(Rx, Ry))


def _quat_to_R(qx, qy, qz, qw):
    """
    四元数 → 3x3 旋转矩阵。
    返回把“相机坐标系向量”旋转到“世界坐标系”的 R_world_cam。
    """
    xx = qx * qx
    yy = qy * qy
    zz = qz * qz
    xy = qx * qy
    xz = qx * qz
    yz = qy * qz
    wx = qw * qx
    wy = qw * qy
    wz = qw * qz

    return [
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz),       2.0 * (xz + wy)],
        [2.0 * (xy + wz),       1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy),       2.0 * (yz + wx),       1.0 - 2.0 * (xx + yy)],
    ]


def _build_cam_base_RT(params: dict):
    """
    构建外参 T_cam_base（base_link -> ZED 相机坐标系）。

    返回：
      R_cam_base: 3x3
      t_cam_base: 3x1（base 原点在 cam 坐标系中的坐标）
    """
    tx = float(params["tx"])
    ty = float(params["ty"])
    tz = float(params["tz"])
    roll_deg = float(params["roll_deg"])
    pitch_deg = float(params["pitch_deg"])
    yaw_deg = float(params["yaw_deg"])

    # base_link: X=前, Y=左, Z=上
    # ZED cam  : X=右, Y=前, Z=上
    # 零安装偏角下的固定轴映射：R_CAM_FROM_BASE_NOMINAL
    R_CAM_FROM_BASE_NOMINAL = [
        [0.0, -1.0, 0.0],
        [1.0,  0.0, 0.0],
        [0.0,  0.0, 1.0],
    ]

    R_mount = _euler_to_R(roll_deg, pitch_deg, yaw_deg)
    R_cam_base = _matmul3(R_mount, R_CAM_FROM_BASE_NOMINAL)

    # 相机在 base 下的位置：t_base=(tx,ty,tz)
    # base 原点在 cam 坐标系：t_cam = -R_cam_base @ t_base
    t_base = [tx, ty, tz]
    Rt = _matvec3(R_cam_base, t_base)
    t_cam_base = [-Rt[0], -Rt[1], -Rt[2]]
    return R_cam_base, t_cam_base


# 外参矩阵构建一次，避免在 ZED 线程里重复算
_R_CAM_BASE, _t_CAM_BASE = _build_cam_base_RT(CAM_TO_BASE_PARAMS)


# ================= Flask 服务 =================
app = Flask(__name__)
CORS(app)


@app.route('/api/location', methods=['GET'])
def get_location():
    if base_tx is not None and base_ty is not None:
        return jsonify({
            # 这里返回的是 base_link 在 ZED 世界坐标系下的位置/航向
            'x':          base_tx,
            'y':          base_ty,
            # 航向：当前系统内部用 heading_cw(顺时针为正)，这里 yaw_raw(CCW正)做个兼容字段
            'yaw_raw':    -base_heading_cw if base_heading_cw is not None else None,
            'heading_cw': base_heading_cw,
            'vx':         vx_current,
            'timestamp':  time.time(),
        })
    return jsonify({'error': 'ZED not ready'}), 503


def start_flask_server():
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)


# ================= ZED 线程 =================
def task_zed():
    global zed_yaw_raw, zed_tx, zed_ty, zed_ready
    global base_tx, base_ty, base_heading_cw, base_forward_xy, base_left_xy

    zed = sl.Camera()

    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.AUTO
    init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP
    init_params.coordinate_units  = sl.UNIT.METER

    status = zed.open(init_params)
    if status != sl.ERROR_CODE.SUCCESS:
        print("[ZED] 打开相机失败:", status)
        return

    tracking_params = sl.PositionalTrackingParameters()
    tracking_params.mode = sl.POSITIONAL_TRACKING_MODE.GEN_3
    status = zed.enable_positional_tracking(tracking_params)
    if status != sl.ERROR_CODE.SUCCESS:
        print("[ZED] 启动定位失败:", status)
        return

    print("[ZED] 相机已启动，视觉里程计运行中...")

    pose = sl.Pose()

    while True:
        if zed.grab() == sl.ERROR_CODE.SUCCESS:
            tracking_state = zed.get_position(pose, sl.REFERENCE_FRAME.WORLD)

            if tracking_state == sl.POSITIONAL_TRACKING_STATE.OK:
                # ---- 位置（X=右, Y=前）----
                translation = pose.get_translation()
                t_world_cam = translation.get()  # (x,y,z) in ZED 世界坐标系
                zed_tx = float(t_world_cam[0])
                zed_ty = float(t_world_cam[1])

                # ---- 航向（四元数 → yaw）----
                orientation = pose.get_orientation()
                qx = orientation.get()[0]
                qy = orientation.get()[1]
                qz = orientation.get()[2]
                qw = orientation.get()[3]

                sin_yaw = 2 * (qw * qz + qx * qy)
                cos_yaw = 1 - 2 * (qy * qy + qz * qz)
                zed_yaw_raw = math.degrees(math.atan2(sin_yaw, cos_yaw))

                # ---- 外参融合：计算 base_link 位姿（仍在 ZED 世界XY）----
                # base_R_world = R_world_cam @ R_cam_base
                R_world_cam = _quat_to_R(qx, qy, qz, qw)
                base_R_world = _matmul3(R_world_cam, _R_CAM_BASE)

                # base_t_world = t_world_cam + R_world_cam @ t_cam_base
                t_cam_base_in_world = _matvec3(R_world_cam, _t_CAM_BASE)
                base_tx = zed_tx + t_cam_base_in_world[0]
                base_ty = zed_ty + t_cam_base_in_world[1]

                # base 前向(X=前)与左向(Y=左)在世界XY的分量
                forward_x = base_R_world[0][0]
                forward_y = base_R_world[1][0]
                base_forward_xy = (forward_x, forward_y)

                left_x = base_R_world[0][1]
                left_y = base_R_world[1][1]
                base_left_xy = (left_x, left_y)

                # base_heading_cw：顺时针为正（当前向指向世界 +Y 时为 0）
                base_heading_cw = math.degrees(math.atan2(forward_x, forward_y))

                if not zed_ready:
                    zed_ready = True
                    print("[ZED] 位姿数据就绪，坐标原点已锁定")

        time.sleep(0.01)


# ================= 暂停控制 =================
def is_navigation_paused():
    try:
        with open(PAUSE_STATE_FILE, "r", encoding="utf-8") as f:
            data = json.load(f)
        return bool(data.get("paused"))
    except Exception:
        return False


def write_navigation_pause_state(paused, reason=None):
    state = {"paused": bool(paused), "timestamp": time.time()}
    if reason:
        state["reason"] = reason
    try:
        with open(PAUSE_STATE_FILE, "w", encoding="utf-8") as f:
            json.dump(state, f, ensure_ascii=False)
        print(f"[暂停] 已更新暂停状态 paused={bool(paused)}")
    except Exception as exc:
        print(f"[暂停] 写入失败: {exc}")


# ================= 状态机控制 =================
def trigger_return_mode():
    global robot_state, current_wp_index, return_waypoints, return_index

    if not waypoints:
        print("[返航] 无航点，忽略 backTask")
        return

    if current_wp_index <= 0:
        start_idx = 0
    elif current_wp_index >= len(waypoints):
        start_idx = len(waypoints) - 1
    else:
        start_idx = current_wp_index - 1

    segment = waypoints[: start_idx + 1]
    if not segment:
        print("[返航] 返航段为空")
        return

    return_waypoints = list(reversed(segment))
    return_index = 0
    robot_state = STATE_RETURN
    print(f"[返航] 从索引 {start_idx} 逆序返航，共 {len(return_waypoints)} 个点")


def trigger_start_task(waypoint_file):
    global robot_state, current_wp_index, return_waypoints, return_index, target_pause_done

    if not waypoint_file:
        waypoint_file = "tmp/robotdog_turning_waypoints.json"

    ok = load_waypoints_from_file(waypoint_file)
    if not ok:
        print(f"[导航] 航点加载失败，保持待机: {waypoint_file}")
        robot_state = STATE_IDLE
        return

    current_wp_index = 0
    return_index = 0
    return_waypoints = []
    target_pause_done = False
    robot_state = STATE_CRUISE
    print(f"[导航] startTask 已加载航点并开始巡航: {waypoint_file}")


def read_nav_state():
    global robot_state

    try:
        with open(NAV_STATE_FILE, "r", encoding="utf-8") as f:
            data = json.load(f)
    except Exception:
        return

    cmd = data.get("command")
    if cmd == "finishTask":
        robot_state = STATE_IDLE
        try:
            os.remove(NAV_STATE_FILE)
        except Exception:
            pass
        return
    if cmd == "startTask":
        trigger_start_task(data.get("waypoint_file"))
        try:
            os.remove(NAV_STATE_FILE)
        except Exception:
            pass
        return
    if cmd == "backTask":
        if robot_state != STATE_RETURN:
            trigger_return_mode()
        try:
            os.remove(NAV_STATE_FILE)
        except Exception:
            pass


# ================= 自动导航 =================
def navigate_to(target_x, target_y, sport_client):
    """
    base_link 纯视觉导航控制（在 ZED 世界系下 X=右，Y=前）。

    0.9m 以外：三阶段锁头（粗锁/细锁/吸附） + 渐进加速，vyaw 转向
    0.9m 以内：vyaw=0（机头朝向不变），用 vx/vy 联合调整位置

    航向约定：
      - current_heading_cw = base_heading_cw（顺时针为正）
      - target_heading_cw  = atan2(dx, dy)（目标从 +Y 轴顺时针的角度）
      - error = normalize(current_heading_cw - target_heading_cw)
      - 正误差 → 当前朝向偏顺时针 → 需逆时针修正 → 正 vyaw ✓
    """
    global vx_current

    print(f"\n========== 开始导航 → 目标:({target_x:.3f}, {target_y:.3f}) ==========\n")

    yaw_gain_coarse = 0.08
    yaw_gain_fine   = 0.05
    yaw_gain_lock   = 0.03

    brake_strength  = 3.5
    yaw_acc_limit   = 0.05

    coarse_threshold = 40.0
    fine_threshold   = 12.0
    heading_scale    = 35.0

    accel_limit = 0.02
    min_vx      = 0.3

    last_vyaw   = 0.0
    vy_cmd      = 0.0
    vyaw        = 0.0
    print_timer = time.time()

    paused_last = False
    prev_cur_x  = None
    prev_cur_y  = None

    while True:
        # ---- 任务结束 ----
        read_nav_state()
        if robot_state == STATE_IDLE:
            sport_client.StopMove()
            vx_current = 0.0
            print("\n[导航] 收到任务结束指令，已停止并进入待机")
            return

        # ---- 暂停控制 ----
        if is_navigation_paused():
            if not paused_last:
                sport_client.StopMove()
                print("\n[导航] 收到暂停指令，已停下")
                last_vyaw = 0.0
                vx_current = 0.0
                prev_cur_x = None
                prev_cur_y = None
                paused_last = True
            time.sleep(0.05)
            continue
        else:
            if paused_last:
                print("\n[导航] 收到继续指令，恢复导航")
                paused_last = False

        if (
            not zed_ready
            or base_tx is None
            or base_ty is None
            or base_heading_cw is None
            or base_forward_xy is None
            or base_left_xy is None
        ):
            time.sleep(0.05)
            continue

        cur_x = base_tx
        cur_y = base_ty

        dx = target_x - cur_x
        dy = target_y - cur_y
        dist = math.hypot(dx, dy)

        # 中点补偿（防止高速越点）
        midpoint_dist = None
        if prev_cur_x is not None and prev_cur_y is not None:
            mid_x = (cur_x + prev_cur_x) / 2.0
            mid_y = (cur_y + prev_cur_y) / 2.0
            midpoint_dist = math.hypot(target_x - mid_x, target_y - mid_y)

        if dist < arrival_radius or (
            midpoint_dist is not None and midpoint_dist < arrival_radius
        ):
            sport_client.StopMove()
            if midpoint_dist is not None and midpoint_dist < arrival_radius and dist >= arrival_radius:
                print(f"\n已到达目标点（中点补偿判定 {midpoint_dist:.2f}m）\n")
            else:
                print("\n已到达目标点\n")
            break

        # 目标方向的顺时针航向（从 ZED 前向 +Y 起算，顺时针为正）
        # atan2(dx, dy)：dx>0(右) → 90°，dy>0(前) → 0°，dx<0(左) → -90°
        target_heading_cw = math.degrees(math.atan2(dx, dy))

        # 当前 base_link 航向（顺时针为正）
        current_heading_cw = base_heading_cw

        error = normalize_angle(current_heading_cw - target_heading_cw)
        abs_error = abs(error)

        if dist >= 0.6:
            # ---- 远距离阶段（≥ 0.9m）：三阶段锁头 + 渐进加速 ----
            if abs_error > coarse_threshold:
                yaw_gain   = yaw_gain_coarse
                stage      = "粗锁"
                allow_move = False
            elif abs_error > fine_threshold:
                yaw_gain   = yaw_gain_fine
                stage      = "细锁"
                allow_move = False
            else:
                yaw_gain   = yaw_gain_lock
                stage      = "吸附"
                allow_move = True

            # tanh 限幅
            vyaw_target = max_yaw_rate * math.tanh(yaw_gain * error / max_yaw_rate)

            # 动态刹车
            braking_angle = (last_vyaw ** 2) / (2 * brake_strength + 1e-6)
            brake_factor  = math.exp(-abs_error / (braking_angle + 0.5))
            vyaw_target  *= (1 - brake_factor)

            # 角加速度限制
            delta     = max(min(vyaw_target - last_vyaw, yaw_acc_limit), -yaw_acc_limit)
            vyaw      = last_vyaw + delta
            last_vyaw = vyaw
            vyaw_send = VYAW_SIGN * vyaw

            # 动态误差阈值（速度越快要求越严）
            dynamic_threshold = max(5.0, fine_threshold - vx_current * 6.0)
            if abs_error > dynamic_threshold:
                allow_move = False

            # ---- 前进速度 ----
            if allow_move:
                heading_factor = 1.0 / (1.0 + (error / heading_scale) ** 2)
                vx_target = max(min(kp_dist * dist, max_vx) * heading_factor, min_vx)
            else:
                vx_target = 0.0

            if vx_target > vx_current:
                vx_current = min(vx_current + accel_limit, vx_target)
            else:
                vx_current = vx_target

            sport_client.Move(vx_current, 0, vyaw_send)

        else:
            # ---- 近距离阶段（< 0.9m）：vyaw=0，vx/vy 联合调整位置 ----
            stage = "近点调整"
            last_vyaw = 0.0
            vyaw      = 0.0
            vyaw_send = 0.0

            # 将世界坐标系位置误差投影到 base_link 机体坐标系（X=前, Y=左）
            forward_x, forward_y = base_forward_xy
            left_x, left_y = base_left_xy
            vx_body = dx * forward_x + dy * forward_y
            vy_body = dx * left_x + dy * left_y

            close_kp    = 0.5
            close_min_v = 0.25
            close_max_v = 0.3
            vx_cmd = max(min(close_kp * vx_body, close_max_v), -close_max_v)
            vy_cmd = max(min(close_kp * vy_body, close_max_v), -close_max_v)
            # 最低速保护：非零分量不足 0.3m/s 时补足（机械狗过低速不响应）
            if 1e-4 < abs(vx_cmd) < close_min_v:
                vx_cmd = math.copysign(close_min_v, vx_cmd)
            if 1e-4 < abs(vy_cmd) < close_min_v:
                vy_cmd = math.copysign(close_min_v, vy_cmd)

            vx_current = math.hypot(vx_cmd, vy_cmd)
            sport_client.Move(vx_cmd, vy_cmd, 0.0)

        # ---- 状态输出 ----
        if time.time() - print_timer > 0.1:
            extra = f" | vy:{vy_cmd:6.3f}" if dist < 0.9 else ""
            print(
                f"\r阶段:{stage} | "
                f"距离:{dist:6.2f}m | "
                f"误差:{error:7.2f}° | "
                f"角速:{vyaw:6.3f} | "
                f"base_heading_cw:{base_heading_cw:7.2f}° | "
                f"目标方向(CW):{target_heading_cw:7.2f}° | "
                f"前速:{vx_current:6.3f}"
                f"{extra}",
                end=""
            )
            print_timer = time.time()

        prev_cur_x = cur_x
        prev_cur_y = cur_y
        time.sleep(0.05)


# ================= 地图显示线程 =================
def task_plot():
    plt.ion()
    fig, ax = plt.subplots()

    while True:
        ax.clear()

        # 原点（ZED 启动位置）
        ax.scatter(0, 0, c='red', s=100, label='Start(Origin)', zorder=5)

        # 航点
        if waypoints:
            xs = [p[0] for p in waypoints]
            ys = [p[1] for p in waypoints]
            ax.scatter(xs, ys, c='blue', s=60, label='Waypoints')
            for i, (x, y) in enumerate(waypoints):
                ax.annotate(str(i + 1), (x, y),
                            textcoords="offset points", xytext=(5, 5), fontsize=9)
            # 连线
            ax.plot(xs, ys, 'b--', linewidth=0.8, alpha=0.5)

        # 当前位置与朝向（base_link）
        if base_tx is not None and base_ty is not None:
            ax.scatter(base_tx, base_ty, c='green', s=100, label='Dog', zorder=5)

            if base_forward_xy is not None:
                arrow_len = 0.3
                adx = arrow_len * base_forward_xy[0]
                ady = arrow_len * base_forward_xy[1]
                ax.arrow(
                    base_tx, base_ty, adx, ady,
                    head_width=0.12, head_length=0.12,
                    fc='green', ec='green', linewidth=2
                )

        # 文字信息
        if base_tx is not None and base_heading_cw is not None:
            info = (
                f"base X(右): {base_tx:7.3f} m\n"
                f"base Y(前): {base_ty:7.3f} m\n"
                f"Heading CW: {base_heading_cw:7.2f}°"
            )
        else:
            info = "ZED not ready"

        ax.text(0.02, 0.98, info, transform=ax.transAxes, fontsize=11,
                verticalalignment='top',
                bbox=dict(facecolor='white', alpha=0.7))

        ax.set_xlabel("X / 右 (m)")
        ax.set_ylabel("Y / 前 (m)")
        ax.set_title("ZED-Only Navigation Map")
        ax.legend(loc='upper right')
        ax.grid(True)
        ax.axis("equal")
        plt.pause(0.5)


# ================= 航点加载 =================
def load_waypoints_from_file(waypoint_file):
    """
    从 JSON 文件加载航点（base_link 在 ZED 世界坐标，米）。
    文件格式：[[x1, y1], [x2, y2], ...]
    """
    global waypoints

    try:
        with open(waypoint_file, "r", encoding="utf-8") as f:
            points = json.load(f)
    except Exception as exc:
        print(f"[航点] 读取失败 {waypoint_file}: {exc}")
        return False

    if not isinstance(points, list):
        print("[航点] 文件格式错误，应为数组")
        return False

    waypoints.clear()
    for idx, item in enumerate(points):
        try:
            x = float(item[0])
            y = float(item[1])
            waypoints.append((x, y))
            print(f"[航点] {idx + 1}: (x={x:.3f}, y={y:.3f})")
        except Exception:
            print(f"[航点] 第 {idx} 个点格式错误: {item}")

    print(f"[航点] 共加载 {len(waypoints)} 个航点")
    return len(waypoints) > 0


# ================= 导航主循环 =================
def run_navigation_loop(sport_client):
    global robot_state, current_wp_index, return_index
    global target_pause_wp_index, target_pause_done

    print("\n========== 开始多航点导航（状态机常驻）==========")
    idle_logged = False

    while True:
        read_nav_state()

        if robot_state == STATE_CRUISE:
            idle_logged = False
            if not waypoints:
                print("[导航] 无可用航点，切回待机")
                robot_state = STATE_IDLE
                time.sleep(0.1)
                continue
            if current_wp_index >= len(waypoints):
                print(
                    f"\n{'='*50}\n"
                    f"[导航] 本次巡航完成（共 {len(waypoints)} 个航点）\n"
                    f"{'='*50}"
                )
                try:
                    ans = input("是否再次执行巡航？[y/N]：").strip().lower()
                except (EOFError, KeyboardInterrupt):
                    ans = 'n'
                if ans == 'y':
                    current_wp_index = 0
                    target_pause_done = False
                    print("[导航] 重新开始巡航...\n")
                else:
                    print("[导航] 进入待机状态")
                    robot_state = STATE_IDLE
                time.sleep(0.1)
                continue

            x, y = waypoints[current_wp_index]
            print(
                f"\n>>> [CRUISE] 前往航点 {current_wp_index + 1}/{len(waypoints)} "
                f"目标:({x:.3f}, {y:.3f})"
            )
            navigate_to(x, y, sport_client)

            if (
                target_pause_wp_index is not None
                and not target_pause_done
                and current_wp_index == target_pause_wp_index
            ):
                write_navigation_pause_state(True, reason="target_nearest_waypoint")
                target_pause_done = True
                print(f"[target] 到达目标航点 {target_pause_wp_index + 1}，已自动暂停")

            current_wp_index += 1

        elif robot_state == STATE_RETURN:
            idle_logged = False
            if not return_waypoints or return_index >= len(return_waypoints):
                print("[返航] 返航完成，进入待机")
                robot_state = STATE_IDLE
                time.sleep(0.1)
                continue

            x, y = return_waypoints[return_index]
            print(
                f"\n>>> [RETURN] 前往返航点 {return_index + 1}/{len(return_waypoints)} "
                f"目标:({x:.3f}, {y:.3f})"
            )
            navigate_to(x, y, sport_client)
            return_index += 1

        elif robot_state == STATE_CHARGE:
            print("[状态] 充电模式（占位）")
            time.sleep(0.2)

        elif robot_state == STATE_IDLE:
            if not idle_logged:
                print("[状态] 待机，等待指令...")
                idle_logged = True
            time.sleep(0.2)

        else:
            time.sleep(0.1)


# ================= 主程序 =================
if __name__ == "__main__":
    """
    用法：
      1) python3 zed_only.py eno1
         → 手动采集航点（base_link 在 ZED 世界系下的坐标）

      2) python3 zed_only.py eno1 waypoints.json
         → 从文件加载航点并自动导航（base_link 坐标系）
    """
    if len(sys.argv) < 2:
        print(f"Usage: python3 {sys.argv[0]} networkInterface [waypoint_json_file]")
        sys.exit(-1)

    network_interface = sys.argv[1]
    waypoint_file = sys.argv[2] if len(sys.argv) >= 3 else None

    ChannelFactoryInitialize(0, network_interface)

    sport_client = SportClient()
    sport_client.SetTimeout(10.0)
    sport_client.Init()

    # 启动后台线程
    threading.Thread(target=task_zed,           daemon=True).start()
    threading.Thread(target=start_flask_server, daemon=True).start()
    threading.Thread(target=task_plot,          daemon=True).start()

    print("ZED-Only 导航已启动，API: http://0.0.0.0:5000/api/location")
    print("等待 ZED 就绪...")
    while not zed_ready:
        time.sleep(0.2)
    print("[主程序] ZED 就绪，坐标原点已确定\n")

    # ==========================
    # 航点来源：文件 / 手动
    # ==========================
    if waypoint_file is not None:
        print(f"[主程序] 使用文件航点: {waypoint_file}")
        ok = load_waypoints_from_file(waypoint_file)
        if not ok:
            print("[主程序] 航点加载失败，退出")
            sys.exit(1)
    else:
        print("========== 航点采集模式 ==========")
        print("将机器人移动到目标点，按 Enter 记录，输入 q 结束采集\n")

        while True:
            cmd = input("记录航点 or q退出：")
            if cmd.lower() == 'q':
                break
            if not zed_ready or base_tx is None or base_ty is None:
                print("[警告] ZED 未就绪，请稍候")
                continue

            x, y = base_tx, base_ty
            waypoints.append((x, y))
            print(
                f"航点 {len(waypoints)} 已记录: "
                f"(x={x:.3f}, y={y:.3f})  "
                f"heading_cw={base_heading_cw:.2f}°"
            )

        if waypoints:
            default_path = (
                f"waypoints_base_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
            )
            save_path = input(
                f"保存航点到文件（回车使用默认 {default_path}）："
            ).strip()
            if not save_path:
                save_path = default_path
            try:
                with open(save_path, "w", encoding="utf-8") as f:
                    json.dump([[x, y] for x, y in waypoints], f,
                              ensure_ascii=False, indent=2)
                print(f"[航点] 已保存 {len(waypoints)} 个航点到 {save_path}")
            except Exception as exc:
                print(f"[航点] 保存失败: {exc}")
        else:
            print("[航点] 未采集任何航点")

    # ==========================
    # 开始导航（状态机驱动）
    # ==========================
    if waypoints:
        current_wp_index = 0
        return_index = 0
        robot_state = STATE_CRUISE
    else:
        robot_state = STATE_IDLE

    run_navigation_loop(sport_client)


