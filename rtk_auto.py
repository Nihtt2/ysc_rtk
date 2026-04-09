#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
rtk_auto.py — 通过 4G 模块串口读取的 RTK 自动打点

通信方式与 rtk.py 一致：差分/定位数据由 4G 模块侧提供，
本程序仅通过串口读取 AGRICA，不再依赖主机外网 CORS/NTRIP。

用法：
  python3 rtk_auto.py [最大段长m=5.0] [最大偏差m=0.2] [最小步进m=0.05]
"""

import time
import sys
import datetime
import serial
import threading
import math
import matplotlib.pyplot as plt
from pyproj import Transformer, Geod
import json
import os
import urllib.request
import urllib.error


# ================= Waypoint =================

class Waypoint:
    def __init__(self, name="", lon=0.0, lat=0.0, action_type=0, time=0.0):
        self.name = name
        self.lon = lon
        self.lat = lat
        self.action_type = action_type
        self.time = time
        self.x = 0.0
        self.y = 0.0


# ================= 串口 =================
serport = serial.Serial("/dev/ttyUSB0", 115200, timeout=1)
serport.flushInput()
time.sleep(0.5)

# ================= 全局变量 - RTK =================
last_utm_x = None
last_utm_y = None
origin_utm_x = None
origin_utm_y = None
filtered_heading = None     # RT 滤波航向
gga_quality = 0
rtk_heading_status = 0      # RTK 航向状态，0=无效，4/5=双天线固定解
latest_fields = None
lon = None
lat = None
waypoints = []              # List[Waypoint]，x/y 为运行期派生属性
auto_waypoint_active = False  # 自动打点线程运行标志

# ================= 滤波参数 =================
alpha = 0.2

# ================= 路线上传接口配置 =================
CONFIG_PATH = "config.json"


def _load_android_config():
    """从 config.json 读取 Android 端 host 和 api_port(8080)，失败时回退到默认值"""
    try:
        with open(CONFIG_PATH, "r", encoding="utf-8") as f:
            cfg = json.load(f)
        host = cfg.get("android_host", "10.65.42.98")
        port = int(cfg.get("api_port", 8080))
        return host, port
    except Exception:
        return "10.65.42.98", 8080


_ANDROID_HOST, _API_PORT = _load_android_config()
ROUTE_UPLOAD_URL = f"http://{_ANDROID_HOST}:{_API_PORT}/api/robot-route/upload"


# ================= 工具函数 =================
def normalize_angle(angle):
    """将角度归一化到 (-180, 180]"""
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle


def get_utm_epsg(lon, lat):
    zone = int((lon + 180) / 6) + 1
    if lat >= 0:
        return 32600 + zone
    else:
        return 32700 + zone


def compute_route_length_meters(gps_coords):
    """根据经纬度列表计算路径总长度（米），WGS84 大地线。点数<2 返回 0。

    约定 gps_coords 中每个点为 [lon, lat]。
    """
    if not gps_coords or len(gps_coords) < 2:
        return 0.0
    geod = Geod(ellps="WGS84")
    total = 0.0
    for i in range(len(gps_coords) - 1):
        lon1, lat1 = float(gps_coords[i][0]), float(gps_coords[i][1])
        lon2, lat2 = float(gps_coords[i + 1][0]), float(gps_coords[i + 1][1])
        _, _, dist = geod.inv(lon1, lat1, lon2, lat2)
        total += dist
    return total


# ================= 自动打点检测器 =================

def _point_to_line_dist(px, py, ax, ay, bx, by):
    """计算点 (px, py) 到直线 (ax,ay)-(bx,by) 的垂直距离（米）"""
    dx = bx - ax
    dy = by - ay
    dlen = math.hypot(dx, dy)
    if dlen < 1e-9:
        return math.hypot(px - ax, py - ay)
    return abs(dy * px - dx * py + bx * ay - by * ax) / dlen


class AutoWaypointDetector:
    """
    在线直线段自动打点检测器。

    算法：
      - 维护当前段点列表 _seg = [(x, y, lon, lat), ...]
      - 每次 add_point() 传入新点 Pk，尝试以 seg[0]–Pk 为参考线扩展当前段：
          · 总长约束：dist(seg[0], Pk) ≤ max_total_len
          · 偏差约束：所有中间点到参考线垂直距离 ≤ max_lateral_dev
        若全部满足 → 扩展段继续；
        若违反 → 封段并重启：
          - m >= 1（段内已有 ≥ 2 点）：记录 [seg[0], seg[-1]] 为一段，从 seg[-1] 开始新段
          - m == 0（段内只有起点，Pk 是第一个扩展点但已违反）：记录 [seg[0], Pk] 为强制段，从 Pk 开始新段
      - flush() 在采集结束时把最后未封段（≥ 2 点）强制输出
    """

    def __init__(self, max_total_len=5.0, max_lateral_dev=0.2, min_step_move=0.05):
        self.max_total_len = max_total_len
        self.max_lateral_dev = max_lateral_dev
        self.min_step_move = min_step_move
        self._seg = []  # [(x, y, lon, lat), ...]

    def add_point(self, x, y, lon=None, lat=None):
        """
        加入一个新采样点。
        返回值：[(wp_start, wp_end), ...]，每个元素为 (x, y, lon, lat) 元组对；
                当前段无需封闭时返回空列表。
        """
        # 低速/抖动过滤：移动距离不足 min_step_move 则跳过
        if self._seg:
            last = self._seg[-1]
            if math.hypot(x - last[0], y - last[1]) < self.min_step_move:
                return []

        new_pt = (x, y, lon, lat)

        if not self._seg:
            self._seg.append(new_pt)
            return []

        P0 = self._seg[0]
        Pm = self._seg[-1]
        m = len(self._seg) - 1  # 当前段除起点外的点数（m=0 表示段内只有起点）

        # 以 Pk=(x,y) 为新终点，用 P0–Pk 参考线检查约束
        total_ok = math.hypot(x - P0[0], y - P0[1]) <= self.max_total_len
        lateral_ok = all(
            _point_to_line_dist(Pj[0], Pj[1], P0[0], P0[1], x, y) <= self.max_lateral_dev
            for Pj in self._seg[1:]   # 中间点：P1 … Pm（不含 P0 和新 Pk）
        )

        if total_ok and lateral_ok:
            self._seg.append(new_pt)
            return []

        # 封段
        if m == 0:
            # 段内只有起点，Pk 是首个扩展点但已违反约束（如单步 > max_total_len）
            # 强制将 [P0, Pk] 作为一段直线输出
            emitted = [(P0, new_pt)]
            self._seg = [new_pt]
        else:
            # 正常封段：[P0, Pm] 为合格直线段，从 Pm 起点开始新段
            emitted = [(P0, Pm)]
            self._seg = [Pm, new_pt]

        return emitted

    def flush(self):
        """
        采集结束时调用，将当前未封闭段（≥ 2 点）强制输出为最后一段。
        返回格式与 add_point 相同。
        """
        if len(self._seg) < 2:
            self._seg.clear()
            return []
        P0 = self._seg[0]
        Pm = self._seg[-1]
        self._seg.clear()
        return [(P0, Pm)]


# ================= AGRICA 解析 =================
def parse_agrica(line):
    try:
        if "#AGRICA" not in line:
            return None
        data_part = line.split(";")[1]
        data_part = data_part.split("*")[0]
        return data_part.split(",")
    except:
        return None


# ================= RTK 线程（串口读取 AGRICA）=================
def task_rtk():
    global last_utm_x, last_utm_y, origin_utm_x, origin_utm_y
    global filtered_heading, gga_quality, rtk_heading_status, latest_fields, lon, lat

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

        if origin_utm_x is None and rtk_status == 4:
            origin_utm_x = utm_x
            origin_utm_y = utm_y
            print("\n[RTK] 地图原点已设定")


# ================= 地图显示线程 =================

def task_plot():
    global last_utm_x, last_utm_y
    global origin_utm_x, origin_utm_y
    global filtered_heading
    global waypoints

    plt.ion()
    fig, ax = plt.subplots()

    while True:
        ax.clear()

        # 原点
        if origin_utm_x is not None:
            ax.scatter(0, 0, c='red', s=100, label='Origin')

        # 航点
        if waypoints:
            xs = [p.x for p in waypoints]
            ys = [p.y for p in waypoints]
            ax.scatter(xs, ys, c='blue', s=60, label='Waypoints')

        # 当前位置（纯 RTK）
        if last_utm_x is not None and origin_utm_x is not None:
            cur_x = last_utm_x - origin_utm_x
            cur_y = last_utm_y - origin_utm_y
            ax.scatter(cur_x, cur_y, c='green', s=100, label='Dog')

            # 朝向箭头（RTK 航向）
            if filtered_heading is not None:
                angle_rad = math.radians(90 - filtered_heading)
                arrow_length = 0.2
                dx = arrow_length * math.cos(angle_rad)
                dy = arrow_length * math.sin(angle_rad)
                ax.arrow(
                    cur_x, cur_y, dx, dy,
                    head_width=0.2, head_length=0.2,
                    fc='green', ec='green', linewidth=2
                )

        ax.set_title("Navigation Map")

        # RTK 航向显示
        rtk_text = (
            f"RTK_heading: {filtered_heading:7.2f}°"
            if filtered_heading is not None
            else "RTK_heading:   N/A"
        )
        ax.text(
            0.02, 0.98,
            rtk_text,
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


# ================= 自动打点线程 =================

def task_auto_waypoint(detector, interval=1.0):
    """
    每隔 interval 秒采样当前 RTK 位置，送入 AutoWaypointDetector。
    每当检测器封段，自动将起、终点追加到全局 waypoints。
    通过全局 auto_waypoint_active 标志控制停止。
    """
    global waypoints, auto_waypoint_active

    print("[自动打点] 线程已启动，等待 RTK 原点就绪...")
    while origin_utm_x is None or origin_utm_y is None:
        time.sleep(0.2)

    print(
        f"[自动打点] 开始记录 "
        f"(max_len={detector.max_total_len}m, "
        f"max_dev={detector.max_lateral_dev}m, "
        f"min_step={detector.min_step_move}m)"
    )

    def _append_wp(wp):
        """将 (x, y, lon, lat) 航点追加到全局列表，自动去掉与上一个点重复的情况。"""
        x, y, wlon, wlat = wp
        if waypoints and math.hypot(waypoints[-1].x - x, waypoints[-1].y - y) < 0.01:
            return
        wp_obj = Waypoint(
            name=str(len(waypoints) + 1),
            lon=wlon if wlon is not None else 0.0,
            lat=wlat if wlat is not None else 0.0,
        )
        wp_obj.x = x
        wp_obj.y = y
        waypoints.append(wp_obj)
        print(f"[自动打点] 航点 {len(waypoints):3d}: ({x:.2f}, {y:.2f})")

    while auto_waypoint_active:
        time.sleep(interval)

        if last_utm_x is None or origin_utm_x is None:
            continue

        cx = last_utm_x - origin_utm_x
        cy = last_utm_y - origin_utm_y
        cur_lon = lon
        cur_lat = lat

        pairs = detector.add_point(cx, cy, cur_lon, cur_lat)
        for wp_start, wp_end in pairs:
            _append_wp(wp_start)
            _append_wp(wp_end)

    # 采集结束：刷出最后一段
    for wp_start, wp_end in detector.flush():
        _append_wp(wp_start)
        _append_wp(wp_end)

    print(f"[自动打点] 采集结束，共记录 {len(waypoints)} 个航点")


def upload_route_to_frontend(route_name, gps_coords):
    """
    调用 Android 端 /api/robot-route/upload 接口，上报当前记录的路线。
    """
    if not gps_coords:
        print("[路线] 无可用航点，跳过上传")
        return

    if not route_name:
        route_name = "自定义路线"
    if len(route_name) > 15:
        route_name = route_name[:15]

    body_obj = {
        "routeName": route_name,
        "routeLength": "%.2f" % compute_route_length_meters(
            [[wp.lon, wp.lat] for wp in gps_coords]
        ),
        "coordinates": [
            {
                "name":        wp.name,
                "lon":         wp.lon,
                "lat":         wp.lat,
                "action_type": wp.action_type,
                "time":        wp.time,
            }
            for wp in gps_coords
        ],
    }

    data = json.dumps(body_obj, ensure_ascii=False).encode("utf-8")
    req = urllib.request.Request(
        ROUTE_UPLOAD_URL,
        data=data,
        headers={"Content-Type": "application/json"},
        method="POST",
    )

    print(f"[路线] 正在上传路线到前端: {ROUTE_UPLOAD_URL}")

    try:
        with urllib.request.urlopen(req, timeout=5) as resp:
            resp_body = resp.read().decode("utf-8", errors="ignore")
            print(f"[路线] 上传成功，HTTP {resp.status}: {resp_body}")
    except urllib.error.HTTPError as e:
        try:
            err_body = e.read().decode("utf-8", errors="ignore")
        except Exception:
            err_body = ""
        print(f"[路线] 上传失败 HTTP {e.code}: {err_body}")
    except Exception as e:
        print(f"[路线] 上传失败: {e}")


# ================= 主程序 =================
if __name__ == "__main__":

    # 用法：python3 rtk_auto.py [max_len=5.0] [max_dev=0.2] [min_step=0.05]
    _auto_max_len  = float(sys.argv[1]) if len(sys.argv) >= 2 else 5.0
    _auto_max_dev  = float(sys.argv[2]) if len(sys.argv) >= 3 else 0.2
    _auto_min_step = float(sys.argv[3]) if len(sys.argv) >= 4 else 0.05

    # 启动后台线程
    threading.Thread(target=task_rtk,           daemon=True).start()
    threading.Thread(target=task_plot,          daemon=True).start()
    print("RTK 自动打点服务已启动（4G 模块串口模式）")

    # ==========================
    # 自动打点模式
    # ==========================
    print("========== 自动打点模式 ==========")
    print(f"参数: max_len={_auto_max_len}m  max_dev={_auto_max_dev}m  min_step={_auto_min_step}m")
    print("机器人开始行走，系统将自动识别直线段并记录航点。")
    input("准备好后按 Enter 开始自动打点：")

    _detector = AutoWaypointDetector(
        max_total_len=_auto_max_len,
        max_lateral_dev=_auto_max_dev,
        min_step_move=_auto_min_step,
    )
    auto_waypoint_active = True
    _auto_thread = threading.Thread(
        target=task_auto_waypoint, args=(_detector,), daemon=True
    )
    _auto_thread.start()

    input("采集完成后按 Enter 停止...\n")
    auto_waypoint_active = False
    _auto_thread.join(timeout=3.0)

    if waypoints:
        print("\n========== 路线上传 ==========")
        route_name = input("请输入本次路线名称（不超过15个字符，回车默认自定义路线）：").strip()
        upload_route_to_frontend(route_name, waypoints)
    else:
        print("[路线] 未采集到任何航点，跳过上传")

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
