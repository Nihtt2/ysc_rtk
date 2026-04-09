#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test.py — 通过 4G 模块串口读取的 RTK+ZED 融合导航

与 rtk.py 的通信方式一致：差分/定位数据由 4G 模块侧提供，
本程序仅通过串口读取 AGRICA，不再依赖主机外网 CORS/NTRIP。

RTK + ZED 融合导航：
  - RTK 航向角仅用于首次启动校准，建立 ZED 相对于真北的偏移量
  - ZED 相机航向角（校准后）作为主要转向控制来源
  - RTK GPS 坐标用于位置/距离计算

坐标约定：
  ZED RIGHT_HANDED_Z_UP 中 yaw 正方向 = 逆时针（俯视）
  罗盘/RTK 航向正方向 = 顺时针
  因此校准公式：calibrated_heading = (offset - zed_yaw) % 360
  若实测方向相反，将 ZED_YAW_SIGN 改为 +1
"""

import time
import sys
import datetime
import argparse
import serial
import threading
import math
import pyzed.sl as sl
import matplotlib.pyplot as plt
from pyproj import Transformer
import json
import os
import urllib.request
import urllib.error
from nav_utils import (
    normalize_angle,
    get_utm_epsg,
    compute_route_length_meters,
    parse_agrica,
    parse_target_lonlat,
    find_nearest_waypoint_index_lonlat,
)
from route_segment_planner import build_named_route, plan_route

from motion import LynxM20Client

from unitree_zed import cam_extrinsic
from unitree_waypoint import Waypoint

from unitree_db.route_db import (
    load_persistence_settings,
    get_route_progress,
    set_route_progress,
    set_route_status,
    read_active_route_meta,
    update_active_route_meta,
    clear_active_route_meta,
)

# ================= ZED 坐标约定 =================
# RIGHT_HANDED_Z_UP: X=右, Y=前, Z=上；俯视时正 yaw = 逆时针，与罗盘顺时针相反，故符号取 -1
# 如果实测校准后转向方向相反，将此值改为 1
ZED_YAW_SIGN = -1

# ================= 串口 =================
# 使用 /dev/serial/by-id 固定到“同一块 RTK 设备”，避免 ttyUSB0/1 反复跳变
# 你当前机器上对应关系（来自：ls -l /dev/serial/by-id/）：
#   usb-Silicon_Labs_HandsFree_RTK_USB_to_UART_Bridge_Controller_0001-if00-port0 -> ../../ttyUSB0
RTK_SERIAL_PATH = (
    "/dev/serial/by-id/"
    "usb-Silicon_Labs_HandsFree_RTK_USB_to_UART_Bridge_Controller_0001-if00-port0"
)

# GGA/AGRICA 质量：3=PPS，4=RTK 固定，5=RTK 浮点（与 NMEA 及 Unicore 说明一致）
RTK_QUALITY_OK_SET = frozenset({3, 4, 5})
# 质量持续不在 {3,4,5} 超过此时长 → 向接收机发 CONFIG ALGRESET RTK1
RTK_QUALITY_WATCHDOG_SEC = 30.0
# 两次 ALGRESET 最小间隔，避免反复打断解算
RTK_ALGRESET_COOLDOWN_SEC = 30.0

def open_rtk_serial():
    while True:
        try:
            s = serial.Serial(RTK_SERIAL_PATH, 115200, timeout=1)
            # 丢弃启动时缓存，避免把旧数据当新数据解析
            s.flushInput()
            time.sleep(0.5)
            print(f"[RTK] 串口已打开: {RTK_SERIAL_PATH}")
            return s
        except Exception as exc:
            print(f"[RTK] 串口未就绪/打开失败: {exc}，等待重试...")
            time.sleep(0.5)


serport = open_rtk_serial()

# ================= 全局变量 - RTK =================
last_utm_x = None
last_utm_y = None
origin_utm_x = None
origin_utm_y = None
filtered_heading = None     # RTK 滤波航向，仅用于校准阶段
gga_quality = 0
rtk_heading_status = 0   # RTK 航向状态，0=无效，4/5=双天线固定解
_rtk_bad_quality_since = None   # 质量 ∉ {3,4,5} 的起始时间；None 表示当前正常
_rtk_last_algreset_time = 0.0   # 上次发送 ALGRESET 的时间戳
latest_fields = None
lon = None
lat = None
waypoints = []      # List[Waypoint]，包含名称/经纬度/本地坐标/动作类型/停留时间

# ================= 机械狗状态机 =================
STATE_IDLE = 1      # 待机
STATE_CRUISE = 2    # 正常巡航
STATE_PAUSE = 3     # 暂停（目前主要由 PAUSE_STATE_FILE 控制）
STATE_RETURN = 4    # 返航
STATE_CHARGE = 5    # 充电模式（预留）

robot_state = STATE_IDLE
current_wp_index = 0
return_waypoints = []   # 返航用航点序列（UTM）
return_index = 0        # 当前返航目标索引
target_pause_wp_index = None   # RobotDogTaskTurning.target 对应的最近航点索引
target_pause_done = False      # 是否已触发过一次目标点自动暂停
task_has_target = False        # 当前任务是否带有效 target；为 true 时跳过 action_type
cli_point_a = None             # 启动命令行传入的 A 点（点名）
cli_point_b = None             # 启动命令行传入的 B 点（点名）
cli_is_loop = 0                # 启动命令行传入的环线标志：1/0

# MySQL 路线持久化：当前任务 routes.id（与 robotdog_ws_client 下发的 route_id 一致）
active_route_id = None

# ================= 全局变量 - ZED =================
zed_yaw_raw = None              # ZED 原始 yaw（度，RIGHT_HANDED_Z_UP）
zed_calibrated_heading = None   # 校准后的 ZED 航向（真北顺时针 0-360°）
zed_calibration_offset = None   # 校准偏移量：offset = rtk_ref - ZED_YAW_SIGN * zed_ref
zed_calibrated = False          # 是否已完成 RTK->ZED 校准

# ================= 全局变量 - ZED定位 =================

stable_rtk_x = None
stable_rtk_y = None

fused_utm_x = None
fused_utm_y = None

zed_ref_tx = None
zed_ref_ty = None
zed_last_raw_yaw = None  # 用于检测“仅旋转”时避免 ZED 位置漂移

# ================= 在线轨迹航向校准状态 =================
_online_calib_ref_utm_x = None  # 本段轨迹起点 UTM X
_online_calib_ref_utm_y = None  # 本段轨迹起点 UTM Y

# ================= 控制参数 =================
alpha = 0.2
max_yaw_rate = 0.35
arrival_radius = 1
vx_current = 0.0
# 若实际转向与预期相反（靠近目标时一直转圈），改为 -1
VYAW_SIGN = 1

# 导航速度参数：这几项可直接按现场需要调整
NAV_MAX_VX = 0.5                 # 远距离最高前进速度
NAV_MIN_VX = 0.2                 # 常规最低巡航速度
NAV_DECEL_START_DIST = 2.5       # 开始减速距离
NAV_MIN_VX_REACHED_DIST = 1.0    # 在该距离减速到最低巡航速度
NAV_LONG_SEGMENT_TRIGGER_DIST = 8.0
NAV_LONG_SEGMENT_STEP_DIST = 5.0
NAV_CLOSE_STAGE_DIST = 0.7

# ================= 导航暂停控制 =================
# 与 WebSocket 客户端通过文件通信，实现“狗停下但后台线程不停止”
PAUSE_STATE_FILE = "tmp/robotdog_nav_pause_state.json"

# backTask 等导航状态控制命令通过此文件传递
NAV_STATE_FILE = "tmp/robotdog_nav_state.json"

# ================= 路线上传接口配置 =================
CONFIG_PATH = "config.json"


def _load_android_config():
    """从 config.json 读取 Android 端 host 和 api_port(8080)，失败时回退到默认值"""
    config_path = CONFIG_PATH
    try:
        with open(config_path, "r", encoding="utf-8") as f:
            cfg = json.load(f)
        host = cfg.get("android_host", "10.65.42.98")
        port = int(cfg.get("api_port", 8080))
        return host, port
    except Exception:
        return "10.65.42.98", 8080


_ANDROID_HOST, _API_PORT = _load_android_config()
STOP_TASK_URL    = f"http://{_ANDROID_HOST}:{_API_PORT}/api/stop-task"


def route_persistence_enabled():
    ok, _ = load_persistence_settings(CONFIG_PATH)
    return ok


def apply_stored_route_progress():
    """
    在已加载 waypoints 且（可选）已做分段规划后调用：
    根据库中 next_wp_index / status 设置 current_wp_index；必要时置为待机。
    使用全局 active_route_id；若启用 --point-a/--point-b 则强制从 0 开始（与库顺序可能不一致）。
    """
    global current_wp_index, robot_state, active_route_id

    if not active_route_id or not route_persistence_enabled():
        return
    if cli_point_a or cli_point_b:
        print("[unitree_db] 已启用 A/B 分段，忽略库中进度，从索引 0 开始")
        current_wp_index = 0
        return

    row = get_route_progress(active_route_id, CONFIG_PATH)
    if not row:
        current_wp_index = 0
        return

    next_idx, st = row
    n = len(waypoints)
    if st == "completed" or (n > 0 and next_idx >= n):
        robot_state = STATE_IDLE
        active_route_id = None
        print("[unitree_db] 路线在库中已完成或进度已走完，保持待机")
        return
    if st != "active":
        current_wp_index = 0
        print(f"[unitree_db] 路线状态为 {st}，从索引 0 开始")
        return

    if n <= 0:
        current_wp_index = 0
        return
    current_wp_index = min(max(0, next_idx), n - 1)
    print(
        f"[unitree_db] 恢复进度 current_wp_index={current_wp_index} "
        f"(库 next_wp_index={next_idx})"
    )


def apply_resume_progress_from_meta(resume_meta):
    """优先按 MySQL 进度恢复；不可用时回退到 active_route_meta.json 中的 next_wp_index。"""
    global current_wp_index, robot_state, active_route_id

    if cli_point_a or cli_point_b:
        print("[恢复] 已启用 A/B 分段，忽略持久化进度，从索引 0 开始")
        current_wp_index = 0
        return

    n = len(waypoints)
    if active_route_id and route_persistence_enabled():
        row = get_route_progress(active_route_id, CONFIG_PATH)
        if row:
            next_idx, st = row
            if st == "completed" or (n > 0 and next_idx >= n):
                robot_state = STATE_IDLE
                active_route_id = None
                clear_active_route_meta()
                print("[恢复] 路线在库中已完成或进度已走完，保持待机")
                return
            if st != "active":
                robot_state = STATE_IDLE
                active_route_id = None
                clear_active_route_meta()
                print(f"[恢复] 路线状态为 {st}，不自动续跑")
                return
            if n <= 0:
                current_wp_index = 0
                return
            current_wp_index = min(max(0, next_idx), n - 1)
            print(
                f"[恢复] 按 MySQL 进度续跑 current_wp_index={current_wp_index} "
                f"(库 next_wp_index={next_idx})"
            )
            return
        print("[恢复] 未读取到 MySQL 进度，回退到本地恢复信息")

    if not isinstance(resume_meta, dict):
        current_wp_index = 0
        return

    try:
        next_idx = int(resume_meta.get("next_wp_index", 0))
    except (TypeError, ValueError):
        next_idx = 0
    st = str(resume_meta.get("status") or "active")

    if st == "completed" or (n > 0 and next_idx >= n):
        robot_state = STATE_IDLE
        active_route_id = None
        clear_active_route_meta()
        print("[恢复] 本地恢复信息显示任务已完成，保持待机")
        return
    if st != "active":
        robot_state = STATE_IDLE
        active_route_id = None
        clear_active_route_meta()
        print(f"[恢复] 本地恢复状态为 {st}，不自动续跑")
        return

    if n <= 0:
        current_wp_index = 0
        return
    current_wp_index = min(max(0, next_idx), n - 1)
    print(f"[恢复] 按本地进度续跑 current_wp_index={current_wp_index}")


def _load_rtk_status_push_url():
    """推送到本机 robotdog_ws_client 的 HTTP 服务，与 config 中 odom_http_port 一致（默认 8091）。"""
    try:
        with open(CONFIG_PATH, "r", encoding="utf-8") as f:
            cfg = json.load(f)
        host = cfg.get("rtk_status_host", "127.0.0.1")
        port = int(cfg.get("odom_http_port", 8091))
        return f"http://{host}:{port}/robotdog/rtk"
    except Exception:
        return "http://127.0.0.1:8091/robotdog/rtk"


RTK_STATUS_PUSH_URL = _load_rtk_status_push_url()
RTK_STATUS_PUSH_INTERVAL = 0.25  # 秒，避免阻塞串口线程
_rtk_status_last_push = 0.0


def try_push_rtk_status_http():
    """
    将当前 RTK/导航状态 POST 到 robotdog_ws_client，写入其 Status，
    随 WebSocket type=statu 推给前端（lon/lat/quality/timestamp/heading，不含 vx）。
    """
    global _rtk_status_last_push
    if lon is None or lat is None:
        return
    now = time.time()
    if now - _rtk_status_last_push < RTK_STATUS_PUSH_INTERVAL:
        return
    _rtk_status_last_push = now
    body = json.dumps(
        {
            "lon": lon,
            "lat": lat,
            "quality": gga_quality,
            "timestamp": now,
            "heading": zed_calibrated_heading,
        },
        allow_nan=False,
    ).encode("utf-8")
    req = urllib.request.Request(
        RTK_STATUS_PUSH_URL,
        data=body,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    try:
        with urllib.request.urlopen(req, timeout=2) as resp:
            if resp.status != 200:
                print(f"[rtk-http] 推送状态 HTTP {resp.status}")
    except Exception as exc:
        print(f"[rtk-http] 推送失败（可忽略）: {exc}")


def call_stop_task(stop_type: int = 0):
    """通知 Android 端停止当前任务。stop_type=0 正常停止，1 紧急停止。"""
    body = json.dumps({"type": stop_type}, ensure_ascii=False).encode("utf-8")
    req = urllib.request.Request(
        STOP_TASK_URL,
        data=body,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    try:
        with urllib.request.urlopen(req, timeout=5) as resp:
            print(f"[stop-task] 已通知前端停止任务，HTTP {resp.status}")
    except Exception as exc:
        print(f"[stop-task] 通知前端失败（可忽略）: {exc}")


# ================= RTK 线程（串口读取 AGRICA）=================
def task_rtk():
    global last_utm_x, last_utm_y, origin_utm_x, origin_utm_y
    global filtered_heading, gga_quality, rtk_heading_status, latest_fields, lon, lat
    global stable_rtk_x, stable_rtk_y
    global fused_utm_x, fused_utm_y
    global zed_ref_tx, zed_ref_ty
    global zed_last_raw_yaw
    global _rtk_bad_quality_since, _rtk_last_algreset_time

    while True:
        line = serport.readline().decode(errors='ignore').strip()
        fields = parse_agrica(line)
        if not fields:
            continue

        latest_fields = fields

        try:
            rtk_status = int(fields[8])
            heading_status = int(fields[9])
            heading = float(fields[19])
            lat = float(fields[29])
            lon = float(fields[30])
        except:
            continue

        gga_quality = rtk_status
        rtk_heading_status = heading_status

        now = time.time()
        if rtk_status in RTK_QUALITY_OK_SET:
            _rtk_bad_quality_since = None
        else:
            if _rtk_bad_quality_since is None:
                _rtk_bad_quality_since = now
            elif (
                now - _rtk_bad_quality_since >= RTK_QUALITY_WATCHDOG_SEC
                and now - _rtk_last_algreset_time >= RTK_ALGRESET_COOLDOWN_SEC
            ):
                try:
                    serport.write(b"CONFIG ALGRESET RTK1\r\n")
                    serport.flush()
                    print(
                        f"[RTK] 质量={rtk_status} 已持续约 {now - _rtk_bad_quality_since:.0f}s（非3/4/5），"
                        "已发送 CONFIG ALGRESET RTK1"
                    )
                except Exception as exc:
                    print(f"[RTK] CONFIG ALGRESET RTK1 发送失败: {exc}")
                _rtk_last_algreset_time = now
                _rtk_bad_quality_since = now

        # 仅在 heading_status 4/5（双天线固定解）时更新 RTK 滤波航向
        if heading_status in [4, 5]:
            if filtered_heading is None:
                filtered_heading = heading
            else:
                diff = normalize_angle(heading - filtered_heading)
                filtered_heading = filtered_heading + alpha * diff
                filtered_heading %= 360

        epsg = get_utm_epsg(lon, lat)
        transformer = Transformer.from_crs("EPSG:4326", f"EPSG:{epsg}", always_xy=True)
        utm_x, utm_y = transformer.transform(lon, lat)

        last_utm_x = utm_x
        last_utm_y = utm_y
        
        if rtk_status == 4:

            stable_rtk_x = utm_x
            stable_rtk_y = utm_y

            if fused_utm_x != utm_x:   # 有跳变，清除上一帧缓存
                fused_utm_x = utm_x
                fused_utm_y = utm_y
                # 重置ZED参考点（防止跳变）
                zed_ref_tx = None
                zed_ref_ty = None
                zed_last_raw_yaw = None

        if origin_utm_x is None and rtk_status == 4:
            origin_utm_x = utm_x
            origin_utm_y = utm_y
            print("\n[RTK] 地图原点已设定")

        try_push_rtk_status_http()


# ================= ZED 线程 =================
def task_zed():
    global zed_yaw_raw, zed_calibrated_heading
    global fused_utm_x, fused_utm_y
    global stable_rtk_x, stable_rtk_y
    global zed_ref_tx, zed_ref_ty
    global gga_quality
    global zed_calibration_offset
    global zed_last_raw_yaw

    zed = sl.Camera()

    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.AUTO
    init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP
    init_params.coordinate_units = sl.UNIT.METER

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

    print("[ZED] 相机已启动，等待 RTK 校准...")

    pose = sl.Pose()

    while True:
        if zed.grab() == sl.ERROR_CODE.SUCCESS:
            tracking_state = zed.get_position(pose, sl.REFERENCE_FRAME.WORLD)

            if tracking_state == sl.POSITIONAL_TRACKING_STATE.OK:
                orientation = pose.get_orientation()

                qx = orientation.get()[0]
                qy = orientation.get()[1]
                qz = orientation.get()[2]
                qw = orientation.get()[3]

                # quaternion → 相机原始 yaw（保留供 calibrate_zed_by_walking 直线保持使用）
                sin_yaw = 2 * (qw * qz + qx * qy)
                cos_yaw = 1 - 2 * (qy * qy + qz * qz)
                raw_yaw = math.degrees(math.atan2(sin_yaw, cos_yaw))
                zed_yaw_raw = raw_yaw

                # ================= 外参合成：相机位姿 → base_link 位姿 =================

                translation = pose.get_translation()
                zed_cam_tx = float(translation.get()[0])
                zed_cam_ty = float(translation.get()[1])

                cam_extrinsic.update(qx, qy, qz, qw, zed_cam_tx, zed_cam_ty)

                # 后续桥接与航向使用 base_link 坐标，不再用相机原点
                tx = cam_extrinsic.base_tx
                ty = cam_extrinsic.base_ty

                # RTK不是固定解时，用ZED桥接（位移量现在基于 base_link 坐标）
                if gga_quality != 4 and stable_rtk_x is not None and zed_calibration_offset is not None:

                    if zed_ref_tx is None:
                        zed_ref_tx = tx
                        zed_ref_ty = ty

                    dx_zed = tx - zed_ref_tx   # base_link X（ZED世界右向）位移
                    dy_zed = ty - zed_ref_ty   # base_link Y（ZED世界前向）位移

                    # 旋转到 UTM（东、北）
                    # ZED RIGHT_HANDED_Z_UP: X=右, Y=前, Z=上
                    # 机头朝向 H₀ = zed_calibration_offset（真北顺时针 0–360°）
                    # 前向(Y) → (sin(H₀), cos(H₀))；右侧(X) → (cos(H₀), -sin(H₀))
                    # dE = dx_zed*cos(H₀) + dy_zed*sin(H₀),  dN = -dx_zed*sin(H₀) + dy_zed*cos(H₀)
                    h0_rad = math.radians(zed_calibration_offset)

                    dx_utm = (
                        dx_zed * math.cos(h0_rad)
                        + dy_zed * math.sin(h0_rad)
                    )
                    dy_utm = (
                        -dx_zed * math.sin(h0_rad)
                        + dy_zed * math.cos(h0_rad)
                    )

                    # 仅旋转时 ZED 视觉容易漂移，导致“距离越判越远”；此时不更新位置
                    # 用 base_link 航向做旋转判断，与桥接坐标系一致
                    base_hdg = cam_extrinsic.base_heading_cw
                    delta_yaw = abs(normalize_angle(base_hdg - zed_last_raw_yaw)) if zed_last_raw_yaw is not None else 0
                    linear_m = math.hypot(dx_zed, dy_zed)
                    if zed_last_raw_yaw is None or delta_yaw <= 3.0 or linear_m >= 0.02:
                        fused_utm_x = stable_rtk_x + dx_utm
                        fused_utm_y = stable_rtk_y + dy_utm
                    zed_last_raw_yaw = base_hdg

                # 校准完成后实时更新校准后航向（使用 base_link 航向，无需 ZED_YAW_SIGN）
                if zed_calibration_offset is not None:
                    zed_calibrated_heading = (cam_extrinsic.base_heading_cw + zed_calibration_offset) % 360

        time.sleep(0.01)


# ================= ZED 校准（一次性，主线程调用）=================
def calibrate_zed_with_rtk():
    """
    等待 RTK 航向稳定后，以 RTK 真北方向校准 ZED 航向偏移量。

    校准偏移公式：
        offset = rtk_heading_ref - base_heading_cw_ref

    校准后实时航向：
        calibrated_heading = (base_heading_cw + offset) % 360
    """
    global zed_calibration_offset, zed_calibrated

    print("\n========== ZED 航向校准 ==========")
    print("等待 RTK 双天线航向就绪（heading_status=4/5）...")

    while filtered_heading is None:
        time.sleep(0.2)

    print(f"[校准] RTK 航向初步就绪: {filtered_heading:.2f}°，再等 3 秒稳定滤波...")
    time.sleep(3)

    print("等待 ZED 外参位姿就绪...")
    while cam_extrinsic.base_heading_cw is None:
        time.sleep(0.1)

    rtk_ref = filtered_heading
    base_heading_ref = cam_extrinsic.base_heading_cw

    # 计算偏移量，使得校准后航向在此刻等于 rtk_ref
    zed_calibration_offset = (rtk_ref - base_heading_ref) % 360
    zed_calibrated = True

    print(f"[校准] 完成！")
    print(f"       RTK 参考航向      : {rtk_ref:.2f}°")
    print(f"       base_link 航向基准 : {base_heading_ref:.2f}°")
    print(f"       计算偏移量         : {zed_calibration_offset:.2f}°")
    print(f"       校准验证航向       : {(base_heading_ref + zed_calibration_offset) % 360:.2f}° （应≈{rtk_ref:.2f}°）")
    print("       后续转向将由 ZED 主控，RTK 仅提供位置\n")


def calibrate_zed_by_walking(sport_client, walk_dist=5.0, vx=0.4, yaw_gain=0.08):
    """
    当 RTK heading_status==0 无法提供航向时，通过行走校准：
    1. 记录起点坐标和 ZED yaw
    2. 机械狗直走 walk_dist 米（用 ZED 保持直线）
    3. 根据起点、终点 UTM 坐标计算真北航向角
    4. 用该航向替代 RTK 校准 ZED
    5. 退回 walk_dist 米
    """
    global zed_calibration_offset, zed_calibrated, filtered_heading

    print("\n========== 行走校准（RTK 航向无效）==========")
    print(f"将直走 {walk_dist}m，通过坐标计算真北航向以校准 ZED...")

    # 等待位置和 ZED 就绪
    while fused_utm_x is None or fused_utm_y is None or origin_utm_x is None:
        print("[行走校准] 等待 RTK 位置就绪...")
        time.sleep(0.3)
    while cam_extrinsic.base_heading_cw is None:
        print("[行走校准] 等待 ZED 外参位姿就绪...")
        time.sleep(0.1)

    # 记录起点
    x1 = fused_utm_x
    y1 = fused_utm_y
    base_heading_cw_start = cam_extrinsic.base_heading_cw
    print(f"[行走校准] 起点: UTM({x1:.2f}, {y1:.2f}), base_heading={base_heading_cw_start:.2f}°")

    def _walk_straight(dist_target, forward=True):
        """沿当前方向走 dist_target 米，forward=True 为前进"""
        sign = 1 if forward else -1
        start_x = fused_utm_x
        start_y = fused_utm_y
        ref_heading = base_heading_cw_start
        last_print = time.time()
        while True:
            dx = fused_utm_x - start_x
            dy = fused_utm_y - start_y
            dist = math.hypot(dx, dy)
            if dist >= dist_target:
                sport_client.stop()
                break
            # 用 base_link 航向保持直线：偏航修正
            cur_hdg = cam_extrinsic.base_heading_cw
            yaw_err = normalize_angle(ref_heading - cur_hdg) if cur_hdg is not None else 0
            vyaw = max(min(yaw_gain * yaw_err, max_yaw_rate), -max_yaw_rate)
            sport_client.move(sign * vx, 0, vyaw)
            if time.time() - last_print > 0.5:
                print(f"\r[行走校准] 已走 {dist:.2f}m / {dist_target}m", end="")
                last_print = time.time()
            time.sleep(0.05)
        print()

    # 前进 walk_dist 米
    print(f"[行走校准] 开始前进 {walk_dist}m...")
    _walk_straight(walk_dist, forward=True)
    time.sleep(0.3)  # 稳定

    # 记录终点并计算真北航向
    x2 = fused_utm_x
    y2 = fused_utm_y
    dx = x2 - x1
    dy = y2 - y1
    dist_actual = math.hypot(dx, dy)

    if dist_actual < 1.0:
        print(f"[行走校准] 错误：实际仅移动 {dist_actual:.2f}m，校准取消")
        sport_client.stop()
        return False

    # UTM: x=东, y=北。真北航向（顺时针0-360°）：atan2(dx, dy)
    heading_true_north = (90 -math.degrees(math.atan2(dy, dx)) ) % 360
    zed_calibration_offset = (heading_true_north - base_heading_cw_start) % 360
    zed_calibrated = True
    filtered_heading = heading_true_north  # 供备用

    print(f"[行走校准] 终点: UTM({x2:.2f}, {y2:.2f}), 实际移动 {dist_actual:.2f}m")
    print(f"[行走校准] 计算真北航向: {heading_true_north:.2f}°")
    print(f"[行走校准] ZED 偏移量: {zed_calibration_offset:.2f}°，校准完成")

    # 退回 walk_dist 米
    print(f"[行走校准] 退回 {walk_dist}m...")
    _walk_straight(walk_dist, forward=False)
    sport_client.stop()
    print("[行走校准] 已退回起点\n")
    return True


def try_online_heading_calibration(min_dist=3.0, correction_gain=0.5, max_correction=8.0):
    """
    在线轨迹航向校准：直线行驶超过 min_dist 米后，用 RTK GPS 轨迹方向渐进修正 zed_calibration_offset。

    触发条件（由调用方保证）：
      - gga_quality == 4（RTK 固定解）
      - abs_error < 15°（直线行驶，非转弯）
      - vx_current > 0.2（有效移动）
      - dist > 3.5（距目标足够远）
    """
    global zed_calibration_offset, _online_calib_ref_utm_x, _online_calib_ref_utm_y

    if gga_quality != 4 or zed_calibration_offset is None or fused_utm_x is None:
        _online_calib_ref_utm_x = None
        _online_calib_ref_utm_y = None
        return

    if _online_calib_ref_utm_x is None:
        _online_calib_ref_utm_x = fused_utm_x
        _online_calib_ref_utm_y = fused_utm_y
        return

    dx = fused_utm_x - _online_calib_ref_utm_x
    dy = fused_utm_y - _online_calib_ref_utm_y
    seg_dist = math.hypot(dx, dy)

    if seg_dist < min_dist:
        return

    # GPS 轨迹方向（真北顺时针 0-360°）
    gps_heading = (90 - math.degrees(math.atan2(dy, dx))) % 360

    if zed_calibrated_heading is None:
        _online_calib_ref_utm_x = fused_utm_x
        _online_calib_ref_utm_y = fused_utm_y
        return

    delta = normalize_angle(gps_heading - zed_calibrated_heading)

    # 超出单次最大修正范围则视为跳变噪声，仅重置参考点
    if abs(delta) > max_correction:
        _online_calib_ref_utm_x = fused_utm_x
        _online_calib_ref_utm_y = fused_utm_y
        return

    correction = correction_gain * delta
    zed_calibration_offset = (zed_calibration_offset + correction) % 360

    print(
        f"\n[在线校准] GPS轨迹航向:{gps_heading:.1f}° | "
        f"ZED航向:{zed_calibrated_heading:.1f}° | "
        f"偏差:{delta:.1f}° | 修正:{correction:+.1f}° | "
        f"新偏移量:{zed_calibration_offset:.1f}° | 段长:{seg_dist:.1f}m"
    )

    _online_calib_ref_utm_x = fused_utm_x
    _online_calib_ref_utm_y = fused_utm_y


# ================= 获取当前导航航向 =================
def get_current_heading():
    """
    返回当前用于导航的航向角（0-360°，真北顺时针）。
    校准完成后优先使用 ZED 校准航向，否则回退到 RTK 航向。
    """
    if zed_calibrated and zed_calibrated_heading is not None:
        return zed_calibrated_heading
    return filtered_heading


def is_navigation_paused():
    """
    从 PAUSE_STATE_FILE 读取暂停状态。
    文件形如：{"paused": true/false, "timestamp": ...}
    """
    try:
        with open(PAUSE_STATE_FILE, "r", encoding="utf-8") as f:
            data = json.load(f)
        return bool(data.get("paused"))
    except Exception:
        return False


def write_navigation_pause_state(paused, reason=None):
    """
    写入导航暂停状态文件，供 navigate_to() 周期读取。
    """
    state = {
        "paused": bool(paused),
        "timestamp": time.time(),
    }
    if reason:
        state["reason"] = reason
    try:
        with open(PAUSE_STATE_FILE, "w", encoding="utf-8") as f:
            json.dump(state, f, ensure_ascii=False)
        print(f"[暂停] 已更新暂停状态 paused={bool(paused)}")
    except Exception as exc:
        print(f"[暂停] 写入暂停状态失败: {exc}")


def find_nearest_waypoint_index(target_lon, target_lat):
    """
    根据经纬度 target 计算距离最近的航点索引（基于局部坐标）。
    """
    return find_nearest_waypoint_index_lonlat(
        target_lon=target_lon,
        target_lat=target_lat,
        waypoints_xy=[(wp.x, wp.y) for wp in waypoints],
        origin_utm_x=origin_utm_x,
        origin_utm_y=origin_utm_y,
    )


def has_target_value(raw_target):
    if raw_target is None:
        return False
    if isinstance(raw_target, str):
        return bool(raw_target.strip())
    if isinstance(raw_target, (list, tuple, dict, set)):
        return len(raw_target) > 0
    return True


def configure_target_pause(raw_target):
    """根据 raw_target 设置最近航点自动暂停逻辑，并决定是否跳过 action_type。"""
    global target_pause_wp_index, task_has_target

    target_pause_wp_index = None
    task_has_target = has_target_value(raw_target)
    parsed_target = parse_target_lonlat(raw_target)
    if parsed_target is None:
        if task_has_target:
            print("[target] target 有值但无法解析，当前任务仍按 target 模式跳过动作")
        return

    nearest_idx = find_nearest_waypoint_index(parsed_target[0], parsed_target[1])
    if nearest_idx is not None:
        target_pause_wp_index = nearest_idx
        print(
            f"[target] 已设置目标暂停航点索引: {target_pause_wp_index + 1}/{len(waypoints)}"
        )
        print("[target] 当前任务带 target，已禁用 action_type 动作")
    else:
        print("[target] 未能计算最近航点索引，但当前任务仍按 target 模式跳过动作")


def apply_segment_route_if_needed(point_a=None, point_b=None, is_loop=0):
    """
    当传入 A/B 外参时，按“最近点 -> A -> B”重排当前 waypoints。
    """
    global waypoints
    if point_a is None or point_b is None:
        return

    if fused_utm_x is None or fused_utm_y is None or origin_utm_x is None or origin_utm_y is None:
        print("[路线规划] 当前位置或原点未就绪，跳过 A->B 分段规划")
        return

    try:
        current_xy = (fused_utm_x - origin_utm_x, fused_utm_y - origin_utm_y)
        named_route = build_named_route(waypoints)
        planned = plan_route(
            current_xy=current_xy,
            route_points=named_route,
            point_a=point_a,
            point_b=point_b,
            is_loop=is_loop,
        )
        new_wps = []
        for p in planned["final_points"]:
            wp = Waypoint(name=p["name"], lon=p["lon"], lat=p["lat"],
                          action_type=p["action_type"], time=p["time"])
            wp.x = p["x"]
            wp.y = p["y"]
            new_wps.append(wp)
        waypoints = new_wps
        print(
            "[路线规划] 已应用分段路线："
            f"最近点={planned['nearest_idx'] + 1}, "
            f"A={planned['a_idx'] + 1}, B={planned['b_idx'] + 1}, "
            f"is_loop={int(is_loop)}, 点名序列={planned['final_names']}"
        )
    except Exception as exc:
        print(f"[路线规划] 规划失败，回退原始路线: {exc}")


def trigger_return_mode():
    """
    切换到返航模式：从当前目标点的前一个点开始，沿既有航点逆序返回。
    """
    global robot_state, current_wp_index, return_waypoints, return_index, waypoints

    if not waypoints:
        print("[返航] 当前无航点，忽略 backTask")
        return

    # 当前目标点的前一个点；如果已经超过末尾，则从最后一个点开始
    if current_wp_index <= 0:
        start_idx = 0
    elif current_wp_index >= len(waypoints):
        start_idx = len(waypoints) - 1
    else:
        start_idx = current_wp_index - 1

    segment = waypoints[: start_idx + 1]
    if not segment:
        print("[返航] 返航段为空，忽略 backTask")
        return

    return_waypoints = list(reversed(segment))
    return_index = 0
    robot_state = STATE_RETURN

    print(
        f"[返航] 接收到 backTask，"
        f"从航点索引 {start_idx} 开始逆序返航，共 {len(return_waypoints)} 个点"
    )


def trigger_start_task(waypoint_file, raw_target=None, route_id=None):
    """
    切换到巡航模式：从 waypoint_file 重新加载航点。
    route_id 若提供且启用持久化，则从库恢复 next_wp_index（未启用 A/B 分段时）。
    """
    global robot_state, current_wp_index, return_waypoints, return_index
    global target_pause_done, active_route_id

    active_route_id = route_id if route_id else None

    if not waypoint_file:
        waypoint_file = "tmp/robotdog_turning_waypoints.json"

    ok = load_waypoints_from_file(waypoint_file)
    if not ok:
        print(f"[导航] startTask 航点加载失败，保持待机: {waypoint_file}")
        active_route_id = None
        robot_state = STATE_IDLE
        clear_active_route_meta()
        return

    current_wp_index = 0
    return_index = 0
    return_waypoints = []
    target_pause_done = False
    write_navigation_pause_state(False, reason="start_task_reset")

    # startTask 经常发生在上一单已进入待机之后，这里先显式切回巡航态；
    # 后续若持久化状态判定该路线不应执行，再由 apply_stored_route_progress() 改回待机。
    robot_state = STATE_CRUISE

    apply_segment_route_if_needed(point_a=cli_point_a, point_b=cli_point_b, is_loop=cli_is_loop)

    apply_stored_route_progress()
    if robot_state != STATE_CRUISE:
        clear_active_route_meta()
        return

    configure_target_pause(raw_target)
    update_active_route_meta(
        route_id=active_route_id,
        waypoint_file=waypoint_file,
        target=raw_target,
        next_wp_index=current_wp_index,
        status="active",
    )

    print(f"[导航] 接收到 startTask，已开始巡航: {waypoint_file}")


def read_nav_state():
    """
    从 NAV_STATE_FILE 读取导航状态指令，处理 finishTask/backTask/startTask。
    """
    global robot_state, active_route_id

    try:
        with open(NAV_STATE_FILE, "r", encoding="utf-8") as f:
            data = json.load(f)
    except Exception:
        return

    cmd = data.get("command")
    if cmd == "finishTask":
        rid = active_route_id
        if rid and route_persistence_enabled():
            set_route_status(rid, "aborted", CONFIG_PATH)
        clear_active_route_meta()
        active_route_id = None
        robot_state = STATE_IDLE
        try:
            os.remove(NAV_STATE_FILE)
        except Exception:
            pass
        call_stop_task(0)
        return
    if cmd == "startTask":
        trigger_start_task(
            data.get("waypoint_file"),
            data.get("target"),
            data.get("route_id"),
        )
        try:
            os.remove(NAV_STATE_FILE)
        except Exception:
            pass
        return
    if cmd == "backTask":
        rid = active_route_id
        if rid and route_persistence_enabled():
            set_route_status(rid, "aborted", CONFIG_PATH)
        clear_active_route_meta()
        active_route_id = None
        if robot_state != STATE_RETURN:
            trigger_return_mode()
        # 标记已处理：删除文件，避免重复触发
        try:
            os.remove(NAV_STATE_FILE)
        except Exception:
            pass


def compute_quadratic_approach_speed(dist):
    """
    按距离返回前向速度目标值：
      - >= 2.5m：保持最高速
      - 2.5m -> 1.5m：二次函数减速到最低巡航速度
      - < 1.5m：保持最低巡航速度，直到到点
    """
    if dist <= arrival_radius:
        return 0.0

    if dist >= NAV_DECEL_START_DIST:
        return NAV_MAX_VX

    if dist >= NAV_MIN_VX_REACHED_DIST:
        ratio = (
            (dist - NAV_MIN_VX_REACHED_DIST)
            / max(NAV_DECEL_START_DIST - NAV_MIN_VX_REACHED_DIST, 1e-6)
        )
        ratio = max(0.0, min(1.0, ratio))
        return NAV_MIN_VX + (NAV_MAX_VX - NAV_MIN_VX) * (ratio ** 2)

    return NAV_MIN_VX


def build_long_distance_checkpoints(start_x, start_y, target_x, target_y):
    """
    当前点到目标点距离大于阈值时，每 5m 生成一个中间点，最后再追加真实目标点。
    """
    dx = target_x - start_x
    dy = target_y - start_y
    total_dist = math.hypot(dx, dy)

    if total_dist <= NAV_LONG_SEGMENT_TRIGGER_DIST:
        return [(target_x, target_y, False, total_dist)]

    checkpoints = []
    mid_idx = 0
    next_dist = NAV_LONG_SEGMENT_STEP_DIST
    while next_dist < total_dist:
        ratio = next_dist / total_dist
        checkpoints.append(
            (
                start_x + dx * ratio,
                start_y + dy * ratio,
                True,
                next_dist,
            )
        )
        mid_idx += 1
        next_dist = (mid_idx + 1) * NAV_LONG_SEGMENT_STEP_DIST

    checkpoints.append((target_x, target_y, False, total_dist))
    return checkpoints


def navigate_with_segment_points(target_wp, sport_client, expected_state):
    """
    对长距离目标按中间点分段导航，只在最终真实航点完成后返回 True。
    """
    if (
        fused_utm_x is None
        or fused_utm_y is None
        or origin_utm_x is None
        or origin_utm_y is None
    ):
        return navigate_to(target_wp.x, target_wp.y, sport_client, expected_state)

    start_x = fused_utm_x - origin_utm_x
    start_y = fused_utm_y - origin_utm_y
    checkpoints = build_long_distance_checkpoints(start_x, start_y, target_wp.x, target_wp.y)
    segmented = len(checkpoints) > 1

    if segmented:
        print(
            f"[长距离] 当前段 {checkpoints[-1][3]:.2f}m，"
            f"已按每 {NAV_LONG_SEGMENT_STEP_DIST:.1f}m 插入 {len(checkpoints) - 1} 个中间点"
        )

    for idx, (sub_x, sub_y, is_midpoint, seg_dist) in enumerate(checkpoints, start=1):
        if is_midpoint:
            print(
                f"[长距离] 前往中间点 {idx}/{len(checkpoints)} "
                f"({sub_x:.2f}, {sub_y:.2f})，累计距离 {seg_dist:.2f}m"
            )
        elif segmented:
            print(
                f"[长距离] 前往最终目标点 {idx}/{len(checkpoints)} "
                f"({sub_x:.2f}, {sub_y:.2f})"
            )
        if not navigate_to(sub_x, sub_y, sport_client, expected_state):
            return False

    return True


# ================= 自动导航 =================
def navigate_to(target_x, target_y, sport_client, expected_state=None):
    """
    三阶段锁头 + 渐进加速 + 动态阈值导航控制。
    航向来源：ZED 校准航向（主）/ RTK 航向（备用）
    """
    global vx_current
    print("\n========== 开始导航 ==========\n")

    yaw_gain_coarse = 0.08
    yaw_gain_fine   = 0.05
    yaw_gain_lock   = 0.03

    brake_strength  = 3.5
    yaw_acc_limit   = 0.05

    coarse_threshold = 40.0
    fine_threshold   = 12.0
    heading_scale    = 35.0

    accel_limit = 0.02

    last_vyaw   = 0.0
    vy_cmd      = 0.0
    print_timer = time.time()

    paused_last = False  # 上一次循环是否处于暂停状态
    prev_cur_x = None
    prev_cur_y = None

    while True:
        # ========== 任务结束控制（finishTask -> STATE_IDLE） ==========
        read_nav_state()
        if robot_state == STATE_IDLE or (
            expected_state is not None and robot_state != expected_state
        ):
            sport_client.stop()
            vx_current = 0.0
            if robot_state == STATE_IDLE:
                print("\n[导航] 收到任务结束指令，已停止并进入待机")
            else:
                print(f"\n[导航] 导航状态已切换为 {robot_state}，当前目标取消")
            return False

        # ========== 暂停控制 ==========
        if is_navigation_paused():
            if not paused_last:
                sport_client.stop()
                print("\n[导航] 收到暂停指令，狗已停下，但 RTK/ZED 仍在运行")
                last_vyaw = 0.0
                vx_current = 0.0
                prev_cur_x = None
                prev_cur_y = None
                paused_last = True
            time.sleep(0.05)
            continue
        else:
            if paused_last:
                print("\n[导航] 收到继续指令，恢复自动导航")
                paused_last = False

        current_heading = get_current_heading()

        if fused_utm_x is None or origin_utm_x is None or current_heading is None:
            prev_cur_x = None
            prev_cur_y = None
            time.sleep(0.05)
            continue

        cur_x = fused_utm_x - origin_utm_x
        cur_y = fused_utm_y - origin_utm_y

        dx = target_x - cur_x
        dy = target_y - cur_y
        dist = math.hypot(dx, dy)

        # 跳变保护：相邻帧位移超过阈值视为 RTK/ZED 跳变，清除中点避免虚假到达
        if prev_cur_x is not None:
            frame_jump = math.hypot(cur_x - prev_cur_x, cur_y - prev_cur_y)
            if frame_jump > 0.5:
                prev_cur_x = None
                prev_cur_y = None

        midpoint_dist = None
        if prev_cur_x is not None and prev_cur_y is not None:
            mid_x = (cur_x + prev_cur_x) / 2.0
            mid_y = (cur_y + prev_cur_y) / 2.0
            midpoint_dist = math.hypot(target_x - mid_x, target_y - mid_y)

        if dist < arrival_radius or (
            midpoint_dist is not None and midpoint_dist < arrival_radius
        ):
            sport_client.stop()
            vx_current = 0.0
            if midpoint_dist is not None and midpoint_dist < arrival_radius and dist >= arrival_radius:
                print(f"\n已到达目标点（中点补偿判定 {midpoint_dist:.2f}m）\n")
            else:
                print("\n已到达目标点\n")
            return True

        # 目标航向（真北顺时针，0-360°）
        angle = math.degrees(math.atan2(dy, dx))
        target_heading = (90 - angle) % 360

        error = normalize_angle(current_heading - target_heading)
        abs_error = abs(error)
        distance_speed = compute_quadratic_approach_speed(dist)

        if dist >= NAV_CLOSE_STAGE_DIST:
            # ---- 远距离阶段（≥ 0.7m）：三阶段锁头 + 渐进加速 ----
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

            # 动态误差阈值（速度越快要求越严格）
            dynamic_threshold = max(5.0, fine_threshold - vx_current * 6.0)
            if abs_error > dynamic_threshold:
                allow_move = False

            # ---- 前进速度 ----
            if allow_move:
                heading_factor = 1.0 / (1.0 + (error / heading_scale) ** 2)
                vx_target = distance_speed * heading_factor
                vx_target = max(vx_target, NAV_MIN_VX)
            else:
                vx_target = 0.0

            if vx_target > vx_current:
                vx_current = min(vx_current + accel_limit, vx_target)
            else:
                vx_current = vx_target

            sport_client.move(vx_current, 0, vyaw_send)

            # ---- 在线轨迹航向校准 ----
            if abs_error > 10.0 or vx_current < 0.2:
                # 转弯或速度过低：重置轨迹段，避免跨越转弯段累积误差
                _online_calib_ref_utm_x = None
                _online_calib_ref_utm_y = None
            elif dist > 3.5:
                # 直线行驶且距目标足够远：尝试在线校准
                try_online_heading_calibration(min_dist=3.0)

        else:
            # ---- 近距离阶段（< 0.7m）：vx/vy 联合调整位置 ----
            stage = "近点调整"
            last_vyaw = 0.0
            vyaw = 0.0
            vyaw_send = 0.0

            # 将世界坐标系（UTM：x=东, y=北）位置误差投影到机体坐标系
            # 机头朝向 H（真北顺时针 0-360°）
            # 机体前向：dx*sin(H) + dy*cos(H)
            # 机体左向（vy正=左）：-dx*cos(H) + dy*sin(H)
            H_rad = math.radians(current_heading)
            vx_body = dx * math.sin(H_rad) + dy * math.cos(H_rad)
            vy_body = -dx * math.cos(H_rad) + dy * math.sin(H_rad)

            close_kp = 0.5
            close_min_vx = NAV_MIN_VX
            close_min_vy = close_min_vx
            close_max_vx = max(distance_speed, close_min_vx)
            close_max_vy = max(distance_speed, close_min_vy)
            vx_cmd = max(min(close_kp * vx_body, close_max_vx), -close_max_vx)
            vy_cmd = max(min(close_kp * vy_body, close_max_vy), -close_max_vy)
            # 最低速保护：非零分量过小会导致机械狗不响应
            if 1e-4 < abs(vx_cmd) < close_min_vx:
                vx_cmd = math.copysign(close_min_vx, vx_cmd)
            if 1e-4 < abs(vy_cmd) < close_min_vy:
                vy_cmd = math.copysign(close_min_vy, vy_cmd)

            vx_current = math.hypot(vx_cmd, vy_cmd)
            sport_client.move(vx_cmd, vy_cmd, 0.0)

        # ---- 状态输出 ----
        if time.time() - print_timer > 0.1:
            src = "ZED" if (zed_calibrated and zed_calibrated_heading is not None) else "RTK"
            midpoint_text = (
                f" | 中点距:{midpoint_dist:6.2f}m"
                if midpoint_dist is not None else ""
            )
            if dist < 0.9:
                extra = f" | vy:{vy_cmd:6.3f}"
            else:
                extra = ""
            print(
                f"\r阶段:{stage} | "
                f"距离:{dist:6.2f}m | "
                f"误差:{error:7.2f}° | "
                f"角速:{vyaw:6.3f} | "
                f"航向[{src}]:{current_heading:7.2f}° | "
                f"目标:{target_heading:7.2f}° | "
                f"前速:{vx_current:6.3f}"
                f"{extra}"
                f"{midpoint_text}",
                end=""
            )
            print_timer = time.time()

        prev_cur_x = cur_x
        prev_cur_y = cur_y

        time.sleep(0.05)

# ================= 地图显示线程 =================

def task_plot():

    global last_utm_x, last_utm_y
    global origin_utm_x, origin_utm_y
    global filtered_heading
    global lon, lat
    global zed_yaw_raw
    global zed_calibration_offset
    global waypoints

    plt.ion()

    fig, ax = plt.subplots()

    while True:

        ax.clear()

        # 原点
        if origin_utm_x is not None:

            ax.scatter(
                0, 0,
                c='red',
                s=100,
                label='Origin'
            )

        # 航点
        if waypoints:

            xs = [p.x for p in waypoints]
            ys = [p.y for p in waypoints]

            ax.scatter(
                xs, ys,
                c='blue',
                s=60,
                label='Waypoints'
            )

        # 当前点
        if fused_utm_x is not None and origin_utm_x is not None:

            cur_x = fused_utm_x - origin_utm_x
            cur_y = fused_utm_y - origin_utm_y

            ax.scatter(
                cur_x, cur_y,
                c='green',
                s=100,
                label='Dog'
            )
            
        
        
        # ================= 朝向箭头 =================

        # 优先使用 ZED 校准航向
        if zed_yaw_raw is not None and zed_calibration_offset is not None:

            heading = (ZED_YAW_SIGN * zed_yaw_raw + zed_calibration_offset) % 360

        elif filtered_heading is not None:

            heading = filtered_heading

        else:

            heading = None


        if heading is not None:

            # 转换为数学坐标角度（X轴为0°逆时针）
            angle_rad = math.radians(90 - heading)

            arrow_length = 0.2   # 箭头长度（米）

            dx = arrow_length * math.cos(angle_rad)
            dy = arrow_length * math.sin(angle_rad)

            ax.arrow(
                cur_x,
                cur_y,
                dx,
                dy,
                head_width=0.2,
                head_length=0.2,
                fc='green',
                ec='green',
                linewidth=2
            )
        
        
        
        ax.set_title("Navigation Map")


        # ================= 航向角显示 =================

        zed_text = (
            f"ZED_raw: {zed_yaw_raw:7.2f}°"
            if zed_yaw_raw is not None
            else "ZED_raw:   N/A"
        )

        rtk_text = (
            f"RTK_heading: {filtered_heading:7.2f}°"
            if filtered_heading is not None
            else "RTK_heading:   N/A"
        )

        offset_text = (
            f"Offset : {zed_calibration_offset:7.2f}°"
            if zed_calibration_offset is not None
            else "Offset :   N/A"
        )

        if zed_yaw_raw is not None and zed_calibration_offset is not None:

            fused = (ZED_YAW_SIGN * zed_yaw_raw + zed_calibration_offset) % 360

            fused_text = f"ZED_verti: {fused:7.2f}°"

        else:

            fused_text = "ZED_verti:   N/A"


        info_text = (
            zed_text + "\n" +
            rtk_text + "\n" +
            offset_text + "\n" +
            fused_text
        )

        ax.text(
            0.02,
            0.98,
            info_text,
            transform=ax.transAxes,
            fontsize=12,
            verticalalignment='top',
            bbox=dict(facecolor='white', alpha=0.7)
        )


        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")

        ax.legend()

        ax.grid(True)

        ax.axis("equal")

        plt.pause(0.5)


def load_waypoints_from_file(waypoint_file):
    """
    从 JSON 文件加载航点，支持两种格式：
      新格式（对象数组）：
        [{"name":"1","lon":120.1,"lat":30.2,"action_type":0,"time":0}, ...]
        action_type: 0 无  1 停留 time 秒  2 坐下 time 秒后起立+standard
      旧格式（兼容）：
        [[lon, lat], ...]
    然后：
      - 等待 RTK 把 origin_utm_x / origin_utm_y 设好
      - 把每个经纬度转成 UTM，再减去原点，得到局部坐标 (x, y)
    """
    global waypoints, origin_utm_x, origin_utm_y

    try:
        with open(waypoint_file, "r", encoding="utf-8") as f:
            gps_points = json.load(f)
    except Exception as exc:
        print(f"[航点] 读取航点文件失败 {waypoint_file}: {exc}")
        return False

    if not isinstance(gps_points, list):
        print("[航点] 航点文件格式错误，应为数组")
        return False

    # 等待 RTK 原点就绪
    while origin_utm_x is None or origin_utm_y is None:
        print("[航点] 等待 RTK 原点就绪...")
        time.sleep(0.5)

    waypoints.clear()

    for idx, item in enumerate(gps_points):
        try:
            if isinstance(item, dict):
                # 新格式：对象
                lon_val    = float(item["lon"])
                lat_val    = float(item["lat"])
                wp_name    = item.get("name", f"{idx + 1}")
                act        = int(item.get("action_type", 0))
                t          = float(item.get("time", 0.0))
            else:
                # 旧格式：[lon, lat]
                lon_val    = float(item[0])
                lat_val    = float(item[1])
                wp_name    = f"{idx + 1}"
                act        = 0
                t          = 0.0
        except Exception:
            print(f"[航点] 第 {idx} 个点格式错误: {item}")
            continue

        epsg = get_utm_epsg(lon_val, lat_val)
        transformer = Transformer.from_crs("EPSG:4326", f"EPSG:{epsg}", always_xy=True)
        utm_x, utm_y = transformer.transform(lon_val, lat_val)

        x = utm_x - origin_utm_x
        y = utm_y - origin_utm_y
        wp = Waypoint(name=wp_name, lon=lon_val, lat=lat_val,
                      action_type=act, time=t)
        wp.x = x
        wp.y = y
        waypoints.append(wp)

        print(
            f"[航点] 已添加 {idx + 1}: "
            f"lat={lat_val:.6f}, lon={lon_val:.6f} -> "
            f"(x={x:.2f}, y={y:.2f})  name={wp_name}  action={act}  time={t}"
        )

    print(f"[航点] 共加载 {len(waypoints)} 个航点")
    return len(waypoints) > 0


def _waypoint_action_sit_then_stand(sport_client, wp):
    """
    action_type==2：调用 sit（坐下）-> 保持 wp.time 秒 -> stand（起立）-> standard。
    """
    dur = max(0.0, float(wp.time))
    print(
        f"[动作] 航点 '{wp.name}' 坐下，保持 {dur} 秒后起立并 standard"
    )
    sport_client.stop()
    time.sleep(0.15)
    sport_client.sit()
    if dur > 0:
        time.sleep(dur)
    else:
        time.sleep(0.3)
    sport_client.stand()
    time.sleep(0.25)
    sport_client.standard()


def run_waypoint_action_if_needed(sport_client, wp, scope_label):
    if task_has_target:
        if wp.action_type in (1, 2):
            print(f"[动作] {scope_label} '{wp.name}' 检测到 target，跳过 action_type={wp.action_type}")
        return

    if wp.action_type == 1 and wp.time > 0:
        print(f"[动作] {scope_label} '{wp.name}' 停留 {wp.time} 秒")
        time.sleep(wp.time)
    elif wp.action_type == 2:
        time.sleep(1)
        _waypoint_action_sit_then_stand(sport_client, wp)


def run_navigation_loop(sport_client):
    """
    基于状态机的导航主循环：
      - STATE_CRUISE：按顺序依次前往 waypoints 中的航点
      - STATE_RETURN：按照 return_waypoints 中的航点逆序返航
      - STATE_IDLE：保持待机并持续读取状态，等待下一条指令
    """
    global robot_state, current_wp_index, return_index, waypoints, return_waypoints
    global target_pause_wp_index, target_pause_done, active_route_id
    global cli_point_a, cli_point_b

    print("\n========== 开始多航点导航（状态机常驻） ==========")
    idle_logged = False

    while True:
        # 检查是否有新的导航控制指令（例如 backTask）
        read_nav_state()

        if robot_state == STATE_CRUISE:
            idle_logged = False
            if not waypoints:
                print("[导航] 巡航状态下无可用航点，切回待机")
                robot_state = STATE_IDLE
                time.sleep(0.1)
                continue
            if current_wp_index >= len(waypoints):
                if (
                    active_route_id
                    and route_persistence_enabled()
                    and not cli_point_a
                    and not cli_point_b
                ):
                    set_route_status(active_route_id, "completed", CONFIG_PATH)
                clear_active_route_meta()
                active_route_id = None
                print("[导航] 巡航完成，进入待机状态")
                robot_state = STATE_IDLE
                call_stop_task(0)
                time.sleep(0.1)
                continue

            wp = waypoints[current_wp_index]
            x, y = wp.x, wp.y
            print(
                f"\n>>> [CRUISE] 前往航点 {current_wp_index + 1}/{len(waypoints)} "
                f"目标:({x:.2f}, {y:.2f})  name={wp.name}"
            )
            reached = navigate_with_segment_points(wp, sport_client, STATE_CRUISE)
            if not reached:
                time.sleep(0.05)
                continue

            if (
                target_pause_wp_index is not None
                and not target_pause_done
                and current_wp_index == target_pause_wp_index
            ):
                write_navigation_pause_state(True, reason="target_nearest_waypoint")
                target_pause_done = True
                print(
                    f"[target] 到达最近航点 {target_pause_wp_index + 1}，已自动进入暂停"
                )

            run_waypoint_action_if_needed(sport_client, wp, "航点")

            current_wp_index += 1
            if (
                active_route_id
                and route_persistence_enabled()
                and not cli_point_a
                and not cli_point_b
            ):
                set_route_progress(active_route_id, current_wp_index, CONFIG_PATH)
            update_active_route_meta(next_wp_index=current_wp_index, status="active")

        elif robot_state == STATE_RETURN:
            idle_logged = False
            if not return_waypoints or return_index >= len(return_waypoints):
                print("[返航] 返航完成，进入待机状态")
                robot_state = STATE_IDLE
                call_stop_task(0)
                time.sleep(0.1)
                continue

            wp = return_waypoints[return_index]
            x, y = wp.x, wp.y
            print(
                f"\n>>> [RETURN] 前往返航点 {return_index + 1}/{len(return_waypoints)} "
                f"目标:({x:.2f}, {y:.2f})  name={wp.name}"
            )
            reached = navigate_with_segment_points(wp, sport_client, STATE_RETURN)
            if not reached:
                time.sleep(0.05)
                continue
            run_waypoint_action_if_needed(sport_client, wp, "返航点")
            return_index += 1

        elif robot_state == STATE_CHARGE:
            # 预留：未来与充电/回桩逻辑对接
            print("[状态] 当前处于充电模式（占位），暂不执行导航")
            time.sleep(0.2)
            continue

        elif robot_state == STATE_IDLE:
            if not idle_logged:
                print("[状态] 当前为待机状态，等待下一条指令")
                idle_logged = True
            time.sleep(0.2)
            continue

        else:
            # 未知状态，简单休眠避免空转
            time.sleep(0.1)


# ================= 主程序 =================
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="RTK+ZED 融合导航（4G 串口）")
    parser.add_argument("robot_ip", nargs="?", default="10.21.31.103", help="云深处 M20 Pro IP 地址，默认 10.21.31.103")
    parser.add_argument("waypoint_file", nargs="?", default=None, help="可选：航点 JSON 文件")
    parser.add_argument("--point-a", dest="point_a", default=None, help="路线起点名称（默认点名为 1..N）")
    parser.add_argument("--point-b", dest="point_b", default=None, help="路线终点名称（默认点名为 1..N）")
    parser.add_argument("--is-loop", dest="is_loop", type=int, default=0, choices=[0, 1], help="是否环线：1=是，0=否")
    parser.add_argument(
        "--resume-active",
        action="store_true",
        help="从 tmp/active_route_meta.json 读取上次任务并续跑，优先使用 MySQL 进度，失败时回退到本地进度",
    )
    args = parser.parse_args()

    robot_ip = args.robot_ip
    waypoint_file = args.waypoint_file
    # 兼容旧启动方式：`python rtk_cors_4g.py 1.json`
    # 当第一个位置参数看起来像 json 文件且未提供第二个参数时，将其识别为 waypoint_file。
    if waypoint_file is None and isinstance(robot_ip, str) and robot_ip.lower().endswith(".json"):
        waypoint_file = robot_ip
        robot_ip = "10.21.31.103"
        print(f"[主程序] 检测到旧参数写法，已将 '{waypoint_file}' 识别为航点文件")
    cli_point_a = args.point_a
    cli_point_b = args.point_b
    cli_is_loop = int(args.is_loop)
    resume_meta = None
    if args.resume_active:
        resume_meta = read_active_route_meta()
        if not resume_meta:
            print("[主程序] --resume-active：未找到或无效的 tmp/active_route_meta.json")
            sys.exit(1)
        waypoint_file = resume_meta.get("waypoint_file")
        if not waypoint_file:
            print("[主程序] --resume-active：meta 缺少 waypoint_file")
            sys.exit(1)

    sport_client = LynxM20Client(robot_ip=robot_ip, use_udp=True)
    sport_client.prepare_for_manual_motion()

    # 启动各后台线程（4G 模块侧提供差分，本程序只读串口）
    threading.Thread(target=task_rtk,           daemon=True).start()
    threading.Thread(target=task_zed,           daemon=True).start()
    threading.Thread(target=task_plot,         daemon=True).start()

    print(
        f"[RTK] 4G 串口模式；位置状态 POST 到 {RTK_STATUS_PUSH_URL} "
        "（需先启动 robotdog_ws_client）"
    )

    # 启动后等待 10 秒，检查 RTK 航向状态
    print("\n[校准] 等待 10 秒，检测 RTK heading_status...")
    heading_always_zero = True
    for i in range(16):  # 20 * 0.5s = 10s
        time.sleep(0.5)
        if rtk_heading_status not in (0, None):
            heading_always_zero = False
        if (i + 1) % 4 == 0:
            print(f"  {int((i + 1) * 0.5)}s, heading_status={rtk_heading_status}")

    if heading_always_zero:
        # RTK 航向一直无效，通过行走 5m 用坐标计算真北航向校准 ZED
        ok = calibrate_zed_by_walking(sport_client, walk_dist=3.0)
        if not ok:
            print("[校准] 行走校准失败，请检查 RTK 定位与 ZED 状态后重试")
            sys.exit(1)
    else:
        # RTK 航向有效，用 RTK 校准 ZED
        calibrate_zed_with_rtk()

    # ==========================
    # 航点来源：文件 / 手动
    # ==========================
    if waypoint_file is not None:
        print(f"[主程序] 使用文件航点: {waypoint_file}")
        ok = load_waypoints_from_file(waypoint_file)
        if not ok:
            print("[主程序] 航点加载失败，退出")
            sys.exit(1)
        apply_segment_route_if_needed(point_a=cli_point_a, point_b=cli_point_b, is_loop=cli_is_loop)
    else:
        print("========== 航点采集模式 ==========")
        print("移动机器人到目标点，按 Enter 记录，输入 q 结束采集\n")

        while True:
            cmd = input("记录航点 or q退出：")
            if cmd.lower() == 'q':
                break

            if fused_utm_x is None or origin_utm_x is None:
                print("[警告] RTK 未就绪（尚无固定解坐标），请稍候")
                continue

            if gga_quality != 4:
                print(f"[警告] 当前 RTK 精度不足（gga_quality={gga_quality}，需为 4=固定解），"
                      f"坐标可能不准，建议等待固定解后再打点")

            x = fused_utm_x - origin_utm_x
            y = fused_utm_y - origin_utm_y

            wp_num  = len(waypoints) + 1
            wp_name = input(f"航点 {wp_num} 名称（回车默认 {wp_num}）：").strip() or f"{wp_num}"

            wp = Waypoint(
                name=wp_name,
                lon=lon if lon is not None else 0.0,
                lat=lat if lat is not None else 0.0,
            )
            wp.x = x
            wp.y = y
            waypoints.append(wp)
            print(f"航点 {wp_num} 已记录: ({x:.2f}, {y:.2f})  name={wp_name}")
            if lon is not None and lat is not None:
                print(f"当前路径长度: {compute_route_length_meters([[w.lon, w.lat] for w in waypoints]):.2f} m")

        # 将航点保存到本地 JSON（新格式：对象数组），不再上传到前端
        if waypoints:
            default_path = f"waypoints_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
            save_path = input(f"保存航点到文件（回车使用默认 {default_path}）：").strip()
            if not save_path:
                save_path = default_path
            try:
                with open(save_path, "w", encoding="utf-8") as f:
                    json.dump(
                        [{"name": w.name, "lon": w.lon, "lat": w.lat,
                          "action_type": w.action_type, "time": w.time}
                         for w in waypoints],
                        f, ensure_ascii=False, indent=2,
                    )
                print(f"[航点] 已保存 {len(waypoints)} 个航点到 {save_path}")
            except Exception as exc:
                print(f"[航点] 保存失败 {save_path}: {exc}")

    # ==========================
    # 开始导航（状态机驱动）
    # ==========================
    if waypoints:
        return_index = 0
        current_wp_index = 0
        robot_state = STATE_CRUISE
        if resume_meta:
            active_route_id = resume_meta.get("route_id") or None
            target_pause_done = False
            apply_resume_progress_from_meta(resume_meta)
            if robot_state != STATE_IDLE:
                configure_target_pause(resume_meta.get("target"))
                update_active_route_meta(
                    route_id=active_route_id,
                    waypoint_file=waypoint_file,
                    route_name=resume_meta.get("route_name"),
                    target=resume_meta.get("target"),
                    next_wp_index=current_wp_index,
                    status="active",
                )
    else:
        # 无初始航点时保持待机，等待 startTask 指令
        robot_state = STATE_IDLE

    try:
        run_navigation_loop(sport_client)
    finally:
        call_stop_task(0)
        sport_client.stop()
        sport_client.close()
