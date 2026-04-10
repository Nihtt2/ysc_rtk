#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
robotdog_ws_client.py

机器狗本地 WebSocket 客户端：
  - 在独立线程中连接 Android App 本地 WebSocket 服务
  - 建立连接后打印连接成功日志
  - 收到 RobotDogTaskTurning 消息时：
      * 把 data 里的经纬度航点写入 JSON 文件
      * 激活 conda 环境并启动 zed_rtk_move.py
  - 处理服务端 ping 消息并回复 pong
  - 支持断线自动重连（指数退避）
"""

import json
import threading
import time
import subprocess
import os
import re
import shlex
import websocket
import http.server
import socketserver

from ysc.db import (
    load_persistence_settings,
    upsert_route_and_waypoints,
    normalized_waypoint_rows_from_turning_data,
    write_active_route_meta,
    read_active_route_meta,
)


CONFIG_PATH = "config.json"

# 向前端推送状态的时间间隔（秒）
STATUS_PUSH_INTERVAL = 5.0


def _load_status_http_port():
    try:
        with open(CONFIG_PATH, "r", encoding="utf-8") as f:
            cfg = json.load(f)
        return int(cfg.get("status_http_port", cfg.get("odom_http_port", 8091)))
    except Exception:
        return 8091


STATUS_HTTP_PORT = _load_status_http_port()


# ================= 机器狗状态 =================
class Status:
    """机器狗当前 RTK/导航状态，通过 WebSocket 定期推送给前端。"""

    def __init__(self):
        # RTK+导航（由 rtk_cors_4g.py POST /robotdog/rtk 更新：lon/lat/quality/timestamp/heading）
        self.rtk_lon = None
        self.rtk_lat = None
        self.rtk_quality = None
        self.rtk_timestamp = None
        self.rtk_heading = None

    def to_dict(self):
        return {
            "lon": self.rtk_lon,
            "lat": self.rtk_lat,
            "quality": self.rtk_quality,
            "timestamp": self.rtk_timestamp,
            "heading": self.rtk_heading,
        }


_status = Status()
_status_lock = threading.Lock()


def _load_android_config():
    """从 config.json 读取 Android 端 host/port，WebSocket 用 ws_port(8888)，HTTP API 用 api_port(8080)"""
    config_path = CONFIG_PATH
    try:
        with open(config_path, "r", encoding="utf-8") as f:
            cfg = json.load(f)
        host = cfg.get("android_host", "10.65.42.98")
        port = int(cfg.get("ws_port", 8888))
        return host, port
    except Exception:
        return "10.65.42.98", 8888


_ANDROID_HOST, _WS_PORT = _load_android_config()

# ================= 连接配置 =================
WS_URL = f"ws://{_ANDROID_HOST}:{_WS_PORT}"
RECONNECT_MIN_SECONDS = 3
RECONNECT_MAX_SECONDS = 10
OPEN_TIMEOUT_SECONDS = 10


# ================= ZED 导航脚本配置 =================
# 注意：以下路径和命令需要根据你在 Linux 上的实际环境修改
CONDA_ACTIVATE_CMD = "source ~/anaconda3/bin/activate && conda activate unitree"
ZED_SCRIPT_PATH = "rtk_cors_4g.py"  # 修改成 zed_rtk_move.py 在机器人上的实际绝对路径或 ~ 路径
WAYPOINTS_FILE = "tmp/robotdog_turning_waypoints.json"

# ================= 导航控制 =================
# 通过这些文件与导航进程通信，实现暂停/继续/返航等控制
PAUSE_STATE_FILE = "tmp/robotdog_nav_pause_state.json"
NAV_STATE_FILE = "tmp/robotdog_nav_state.json"


# ================= 全局状态 =================
stop_event = threading.Event()
connected_event = threading.Event()
_zed_process = None
_zed_process_lock = threading.Lock()
_ws_app_ref = None        # 当前活跃的 WebSocketApp 实例
_ws_app_ref_lock = threading.Lock()

class ThreadingHTTPServer(socketserver.ThreadingMixIn, http.server.HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


class RobotdogHTTPHandler(http.server.BaseHTTPRequestHandler):
    """本地 HTTP：POST /robotdog/rtk（RTK+导航状态）。"""

    def do_POST(self):
        if self.path == "/robotdog/rtk":
            self._handle_rtk()
            return
        self.send_error(404, "Not Found")

    def _read_json_body(self):
        try:
            content_length = int(self.headers.get("Content-Length", "0"))
        except Exception:
            content_length = 0
        raw = self.rfile.read(content_length) if content_length > 0 else b"{}"
        try:
            return json.loads(raw.decode("utf-8"))
        except Exception:
            return None

    def _send_ok(self):
        resp = json.dumps({"ok": True}, ensure_ascii=False).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(resp)))
        self.end_headers()
        self.wfile.write(resp)

    def _handle_rtk(self):
        payload = self._read_json_body()
        if not isinstance(payload, dict):
            self.send_error(400, "Bad Request")
            return

        def _f(key):
            v = payload.get(key)
            if v is None:
                return None
            try:
                return float(v)
            except (TypeError, ValueError):
                return None

        with _status_lock:
            lon = _f("lon")
            lat = _f("lat")
            if lon is not None:
                _status.rtk_lon = lon
            if lat is not None:
                _status.rtk_lat = lat
            q = payload.get("quality")
            if q is not None:
                try:
                    _status.rtk_quality = int(q)
                except (TypeError, ValueError):
                    pass
            ts = payload.get("timestamp")
            if ts is not None:
                try:
                    _status.rtk_timestamp = float(ts)
                except (TypeError, ValueError):
                    pass
            h = payload.get("heading")
            if h is not None:
                try:
                    _status.rtk_heading = float(h)
                except (TypeError, ValueError):
                    pass

        self._send_ok()

    def log_message(self, fmt, *args):
        # 避免 HTTPServer 默认日志刷屏
        return


# ================= 工具函数 =================
def build_pong_message(timestamp):
    """
    生成服务端 ping 对应的 pong 消息。
    """
    return json.dumps(
        {
            "type": "pong",
            "timestamp": timestamp if timestamp is not None else int(time.time() * 1000),
        },
        ensure_ascii=False,
    )


def _normalize_waypoint_dict(item, seq=None):
    if not isinstance(item, dict):
        return None

    try:
        lon = float(item["lon"])
        lat = float(item["lat"])
    except (KeyError, TypeError, ValueError):
        return None

    name = item.get("name")
    if name is None or name == "":
        name = str(seq) if seq is not None else ""

    action_raw = item.get("action_type", item.get("actionType", 0))
    time_raw = item.get("time", 0.0)
    try:
        action_type = int(action_raw)
    except (TypeError, ValueError):
        action_type = 0
    try:
        stay_time = float(time_raw)
    except (TypeError, ValueError):
        stay_time = 0.0

    return {
        "lon": lon,
        "lat": lat,
        "name": str(name),
        "action_type": action_type,
        "time": stay_time,
    }


def _unwrap_turning_data_raw(data_raw):
    if not isinstance(data_raw, dict):
        return data_raw
    for key in ("routeData", "data", "waypoints", "points", "pathPoints"):
        if key in data_raw:
            return data_raw.get(key)
    if "lon" in data_raw and "lat" in data_raw:
        return [data_raw]
    return data_raw


def _parse_kv_style_waypoints(raw: str):
    """
    解析 Java/Map 风格字符串:
    [{lat=33.6, lon=107.7, name=1}, {lat=33.7, lon=107.8, name=2}]
    返回标准航点对象列表。
    """
    text = (raw or "").strip()
    if not (text.startswith("[") and text.endswith("]")):
        return None

    chunks = re.findall(r"\{([^{}]+)\}", text)
    if not chunks:
        return None

    points = []
    for idx, chunk in enumerate(chunks, start=1):
        item = {}
        for part in chunk.split(","):
            if "=" not in part:
                continue
            k, v = part.split("=", 1)
            k = k.strip()
            v = v.strip()

            if not k:
                continue

            lv = v.lower()
            if lv == "true":
                val = True
            elif lv == "false":
                val = False
            else:
                try:
                    if any(c in v for c in (".", "e", "E")):
                        val = float(v)
                    else:
                        val = int(v)
                except Exception:
                    val = v
            item[k] = val

        row = _normalize_waypoint_dict(item, seq=idx)
        if row is not None:
            points.append(row)

    return points if points else None


def _normalize_turning_data(data_raw):
    """
    支持多种输入：
    1) [[lon, lat], ...]
    2) [{"lon":..,"lat":..,"name":..,"action_type"/"actionType":..,"time":..}, ...]
    3) JSON 字符串或 k=v 风格字符串
    4) 前端外层 data 包裹：{"routeData": ..., "routeName": ..., ...}
    最终返回：对象数组或 [lon, lat] 数组。
    """
    data = _unwrap_turning_data_raw(data_raw)

    if isinstance(data, str):
        s = data.strip()
        try:
            data = json.loads(s)
        except Exception:
            data = _parse_kv_style_waypoints(s)
    elif isinstance(data, dict):
        data = [data]

    if not isinstance(data, list) or not data:
        return None

    if isinstance(data[0], dict):
        out = []
        for idx, p in enumerate(data, start=1):
            row = _normalize_waypoint_dict(p, seq=idx)
            if row is not None:
                out.append(row)
        return out if out else None

    if isinstance(data[0], (list, tuple)):
        out = []
        for p in data:
            if not isinstance(p, (list, tuple)) or len(p) < 2:
                continue
            try:
                out.append([float(p[0]), float(p[1])])
            except Exception:
                continue
        return out if out else None

    return None


def _build_navigation_command(resume_active=False, waypoint_file=None):
    cmd = f"python {shlex.quote(ZED_SCRIPT_PATH)}"
    if resume_active:
        cmd += " --resume-active"
    elif waypoint_file:
        cmd += f" --waypoint-file {shlex.quote(waypoint_file)}"
    return f"{CONDA_ACTIVATE_CMD} && {cmd}"


def _launch_zed_navigation_process(resume_active=False, waypoint_file=None, reason=None):
    """
    启动 ZED + RTK 导航脚本。

    resume_active=True 时使用 --resume-active 从 active_route_meta.json 续跑；
    否则可选传入 waypoint_file 作为启动参数。
    """
    global _zed_process

    with _zed_process_lock:
        if _zed_process is not None and _zed_process.poll() is None:
            print("[ZED] 导航程序已在运行，跳过本次启动")
            return

        cmd = _build_navigation_command(
            resume_active=resume_active,
            waypoint_file=waypoint_file,
        )

        if reason:
            print(f"[ZED] {reason}")
        print(f"[ZED] 即将启动导航进程: {cmd}")
        try:
            # 使用 bash -lc 以便支持 source / conda 等 shell 命令
            _zed_process = subprocess.Popen(
                ["bash", "-lc", cmd],
                stdout=None,
                stderr=None,
            )
        except Exception as exc:
            print(f"[ZED] 启动导航进程失败: {exc}")


def _handle_robotdog_task_turning(payload):
    """
    处理前端下发的 RobotDogTaskTurning 指令。

    兼容两类格式：
      1) 旧格式：payload 直接携带 id/routeName/data
      2) 新格式：payload.data 为对象，其中 routeData 是字符串，id/routeName/target 也在该对象内
    """
    task_data = payload.get("data") if isinstance(payload.get("data"), dict) else {}

    data_raw = (
        task_data.get("routeData")
        or task_data.get("data")
        or task_data.get("waypoints")
        or task_data.get("points")
        or task_data.get("pathPoints")
        or payload.get("routeData")
        or payload.get("waypoints")
        or payload.get("points")
        or payload.get("pathPoints")
        or payload.get("data")
    )
    target = task_data.get("target", payload.get("target"))

    raw_rid = (
        payload.get("id")
        or payload.get("routeId")
        or payload.get("navigationId")
        or task_data.get("id")
        or task_data.get("routeId")
        or task_data.get("navigationId")
    )
    route_id = str(raw_rid).strip() if raw_rid is not None and str(raw_rid).strip() else None
    route_name = (
        payload.get("name")
        or payload.get("routeName")
        or task_data.get("name")
        or task_data.get("routeName")
    )

    data = _normalize_turning_data(data_raw)
    if not data:
        print(f"[WS] RobotDogTaskTurning 数据为空或格式错误，raw={data_raw!r}")
        return

    # 确保目录存在
    waypoints_dir = os.path.dirname(WAYPOINTS_FILE)
    if waypoints_dir and not os.path.exists(waypoints_dir):
        try:
            os.makedirs(waypoints_dir, exist_ok=True)
        except Exception as exc:
            print(f"[WS] 创建航点目录失败: {exc}")
            return

    try:
        with open(WAYPOINTS_FILE, "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False)
        print(f"[WS] 已写入 {len(data)} 个航点到 {WAYPOINTS_FILE}")
    except Exception as exc:
        print(f"[WS] 写入航点文件失败: {exc}")
        return

    write_active_route_meta(
        route_id,
        WAYPOINTS_FILE,
        route_name,
        target=target,
        next_wp_index=0,
        status="active",
    )

    wp_rows = normalized_waypoint_rows_from_turning_data(data)
    if wp_rows:
        ok_persist, _ = load_persistence_settings(CONFIG_PATH)
        if ok_persist:
            if not route_id:
                print("[WS] 已启用路线持久化但未提供路线 id（id/routeId/navigationId），仅写入本地恢复信息")
            elif upsert_route_and_waypoints(
                route_id, route_name, wp_rows, CONFIG_PATH
            ):
                print(f"[WS] 路线已写入 MySQL: route_id={route_id!r}")
            else:
                print("[WS] 路线写入 MySQL 失败，已保留本地恢复信息")
        elif route_id:
            print("[WS] 未启用 route_persistence_enabled，跳过 MySQL")

    # 通知 rtk_cors.py 重新加载航点并进入巡航
    start_state = {
        "command": "startTask",
        "timestamp": time.time(),
        "waypoint_file": WAYPOINTS_FILE,
        "target": target,
    }
    if route_id:
        start_state["route_id"] = route_id
    if route_name is not None:
        start_state["route_name"] = route_name
    try:
        with open(NAV_STATE_FILE, "w", encoding="utf-8") as f:
            json.dump(start_state, f, ensure_ascii=False)
        print(f"[WS] startTask 已写入启动指令到 {NAV_STATE_FILE}")
    except Exception as exc:
        print(f"[WS] startTask 写入失败: {exc}")

    _launch_zed_navigation_process(waypoint_file=WAYPOINTS_FILE)


def _handle_stop_task(payload):
    """
    处理前端下发的 stopTask 指令：
      - payload 形如：
        {
          "type": "stopTask",
          "ackId": "...",
          "data": {
              "continueType": true/false,
              "sn": "1581F8HXX233S00A05PG"
          }
        }
      - continueType == False: 暂停自动导航（狗停下，但后台线程继续）
      - continueType == True : 继续自动导航

    具体做法：将暂停状态写入 PAUSE_STATE_FILE，rtk.py 周期性读取该文件。
    """
    data = payload.get("data") or {}
    cont = data.get("continueType")

    if not isinstance(cont, bool):
        print("[WS] stopTask.continueType 缺失或类型错误，已忽略")
        return

    paused = not cont  # continueType=False -> paused=True

    state = {
        "paused": paused,
        "timestamp": time.time(),
        "sn": data.get("sn"),
    }

    try:
        with open(PAUSE_STATE_FILE, "w", encoding="utf-8") as f:
            json.dump(state, f, ensure_ascii=False)
        print(f"[WS] stopTask 已更新暂停状态: paused={paused}, 写入 {PAUSE_STATE_FILE}")
    except Exception as exc:
        print(f"[WS] stopTask 写入暂停状态失败: {exc}")


def _handle_back_task(payload):
    """
    处理前端下发的 backTask 指令：
      - payload 形如：
        {
          "type": "backTask",
          "ackId": "...",
          "data": {
              "navigationId": "...",
              "pathPoints": "...",
              "sn": "1581F8HXX233S00A05PG"
          }
        }
      - 将一个简单的返航命令写入 NAV_STATE_FILE，供 rtk_cors.py 读取
    """
    data = payload.get("data") or {}

    state = {
        "command": "backTask",
        "timestamp": time.time(),
        "navigationId": data.get("navigationId"),
        "pathPoints": data.get("pathPoints"),
        "sn": data.get("sn"),
    }

    try:
        with open(NAV_STATE_FILE, "w", encoding="utf-8") as f:
            json.dump(state, f, ensure_ascii=False)
        print(f"[WS] backTask 已写入返航指令到 {NAV_STATE_FILE}")
    except Exception as exc:
        print(f"[WS] backTask 写入返航指令失败: {exc}")


def _handle_finish_task(payload):
    """
    处理前端下发的 finishTask 指令：只要 type 为 finishTask 即结束任务。
    将命令写入 NAV_STATE_FILE，供 rtk_cors.py 读取并进入待机（STATE_IDLE）。
    """
    state = {
        "command": "finishTask",
        "timestamp": time.time(),
    }
    try:
        with open(NAV_STATE_FILE, "w", encoding="utf-8") as f:
            json.dump(state, f, ensure_ascii=False)
        print(f"[WS] finishTask 已写入任务结束指令到 {NAV_STATE_FILE}")
    except Exception as exc:
        print(f"[WS] finishTask 写入失败: {exc}")


def _read_pending_start_task():
    try:
        with open(NAV_STATE_FILE, "r", encoding="utf-8") as f:
            data = json.load(f)
    except Exception:
        return None
    if not isinstance(data, dict) or data.get("command") != "startTask":
        return None
    return data


def _maybe_resume_navigation_on_startup():
    meta = read_active_route_meta()
    if isinstance(meta, dict):
        waypoint_file = meta.get("waypoint_file")
        status = str(meta.get("status") or "active")
        if waypoint_file and os.path.exists(waypoint_file) and status == "active":
            print(f"[启动恢复] 检测到未完成任务，准备按上次进度续跑: {waypoint_file}")
            _launch_zed_navigation_process(
                resume_active=True,
                reason="开机自动恢复未完成路线",
            )
            return
        if waypoint_file and status in ("completed", "aborted"):
            print(f"[启动恢复] 上次任务状态为 {status}，跳过自动续跑")

    pending = _read_pending_start_task()
    if pending is None:
        return

    waypoint_file = pending.get("waypoint_file")
    if waypoint_file and not os.path.exists(waypoint_file):
        print(f"[启动恢复] 检测到未消费的 startTask，但航点文件不存在: {waypoint_file}")
        return

    print("[启动恢复] 检测到未消费的 startTask，准备重新拉起导航进程")
    _launch_zed_navigation_process(
        reason="开机恢复待执行的 startTask",
    )


# ================= WebSocket 回调 =================
def on_open(ws):
    global _ws_app_ref
    with _ws_app_ref_lock:
        _ws_app_ref = ws
    connected_event.set()
    print(f"[WS] 连接成功: {WS_URL}")


def on_message(ws, message):
    print(f"[WS] 收到消息: {message}")

    try:
        payload = json.loads(message)
    except Exception:
        # 非 JSON 消息按原样打印即可，不做其他处理
        return

    if not isinstance(payload, dict):
        return

    msg_type = payload.get("type")

    # 心跳
    if msg_type == "ping":
        pong_msg = build_pong_message(payload.get("timestamp"))
        ws.send(pong_msg)
        print(f"[WS] 已回复心跳: {pong_msg}")
        return

    # 自动导航任务
    if msg_type == "RobotDogTaskTurning":
        _handle_robotdog_task_turning(payload)
        return

    # 暂停 / 继续 自动导航任务
    if msg_type == "stopTask":
        _handle_stop_task(payload)
        return

    # 返航任务
    if msg_type == "backTask":
        _handle_back_task(payload)
        return

    # 任务结束（进入待机，可再下发返航）
    if msg_type == "finishTask":
        _handle_finish_task(payload)
        return


def on_error(ws, error):
    print(f"[WS] 连接异常: {error}")


def on_close(ws, close_status_code, close_msg):
    global _ws_app_ref
    with _ws_app_ref_lock:
        _ws_app_ref = None
    connected_event.clear()
    print(f"[WS] 连接关闭: code={close_status_code}, msg={close_msg}")


# ================= WebSocket 线程 =================
def task_websocket():
    """
    长连接线程：
      - run_forever 负责维持单次连接生命周期
      - 连接断开后按指数退避重连
    """
    reconnect_wait = RECONNECT_MIN_SECONDS

    while not stop_event.is_set():
        ws_app = websocket.WebSocketApp(
            WS_URL,
            on_open=on_open,
            on_message=on_message,
            on_error=on_error,
            on_close=on_close,
        )

        try:
            print(f"[WS] 尝试连接: {WS_URL}")
            ws_app.run_forever(
                ping_interval=20,
                ping_timeout=8,
                ping_payload="keepalive",
                http_proxy_host=None,
                http_proxy_port=None,
            )
        except Exception as exc:
            print(f"[WS] run_forever 异常: {exc}")

        if stop_event.is_set():
            break

        print(f"[WS] {reconnect_wait}s 后重连...")
        time.sleep(reconnect_wait)
        reconnect_wait = min(reconnect_wait * 2, RECONNECT_MAX_SECONDS)


# ================= 状态推送线程 =================
def task_status_push():
    """
    后台线程：每隔 STATUS_PUSH_INTERVAL 秒读取一次状态数据，
    并通过 WebSocket 向前端推送 type=statu 消息。
    """
    while not stop_event.is_set():
        time.sleep(STATUS_PUSH_INTERVAL)

        with _status_lock:
            status_data = _status.to_dict()

        with _ws_app_ref_lock:
            ws = _ws_app_ref

        if ws is None:
            continue

        msg = json.dumps(
            {"type": "statu", "data": status_data},
            ensure_ascii=False,
        )
        try:
            ws.send(msg)
            print(f"[STATUS] 已推送状态: {msg}")
        except Exception as exc:
            print(f"[STATUS] 推送状态失败: {exc}")


def task_status_http_server():
    """本地 HTTP 服务：接收 RTK 状态（供 WebSocket status 推送）。"""
    server = ThreadingHTTPServer(("0.0.0.0", STATUS_HTTP_PORT), RobotdogHTTPHandler)
    print(
        f"[HTTP] 本地服务已启动: 0.0.0.0:{STATUS_HTTP_PORT} "
        f"POST /robotdog/rtk"
    )
    while not stop_event.is_set():
        server.handle_request()
    server.server_close()


if __name__ == "__main__":
    ws_thread = threading.Thread(target=task_websocket, daemon=True)
    ws_thread.start()

    status_thread = threading.Thread(target=task_status_push, daemon=True)
    status_thread.start()

    status_http_thread = threading.Thread(target=task_status_http_server, daemon=True)
    status_http_thread.start()

    _maybe_resume_navigation_on_startup()

    # 等待首次连接结果（超时仅提示，不退出）
    if not connected_event.wait(timeout=OPEN_TIMEOUT_SECONDS):
        print("[WS] 首次连接超时，已进入后台自动重连模式")

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n[WS] 收到退出信号，准备停止...")
        stop_event.set()
        ws_thread.join(timeout=2)
        print("[WS] 已退出")
