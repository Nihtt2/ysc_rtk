#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
模块名称：里程计总里程持续订阅 + HTTP上报
版本号：V1.1
功能说明：订阅 ROS2 话题 /dog_odom(nav_msgs/msg/Odometry)，
          累计本进程启动后的总里程（米）和线速度（m/s），
          并通过 HTTP 上报到 robotdog_ws_client.py。
          导入后可通过 odom_distance_monitor.total_distance_m 读取总里程。
"""

import argparse
import json
import math
import threading
import time
import urllib.request
import urllib.error
import sys

try:
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry
except Exception:
    rclpy = None
    Node = object
    Odometry = object


PRINT_INTERVAL = 2.0


class OdomDistanceMonitor:
    def __init__(self):
        self._lock = threading.Lock()
        self.total_distance_m = 0.0
        self.linear_speed_mps = 0.0
        self._last_position = None  # (x, y, z)
        self._has_msg = False

    def update(self, x: float, y: float, z: float, vx: float, vy: float, vz: float):
        with self._lock:
            if self._last_position is None:
                self._last_position = (x, y, z)
                self.linear_speed_mps = math.sqrt(vx * vx + vy * vy + vz * vz)
                self._has_msg = True
                return

            lx, ly, lz = self._last_position
            step = math.sqrt((x - lx) ** 2 + (y - ly) ** 2 + (z - lz) ** 2)
            self.total_distance_m += step
            self._last_position = (x, y, z)
            self.linear_speed_mps = math.sqrt(vx * vx + vy * vy + vz * vz)
            self._has_msg = True

    def get(self):
        with self._lock:
            return self.total_distance_m, self.linear_speed_mps, self._has_msg


class _DogOdomNode(Node):
    def __init__(self, topic_name: str):
        super().__init__("dog_odom_distance_monitor")
        self._sub = self.create_subscription(
            Odometry,
            topic_name,
            self._cb,
            10,
        )

    def _cb(self, msg: Odometry):
        pos = msg.pose.pose.position
        lin = msg.twist.twist.linear
        odom_distance_monitor.update(
            float(pos.x), float(pos.y), float(pos.z),
            float(lin.x), float(lin.y), float(lin.z),
        )


odom_distance_monitor = OdomDistanceMonitor()
_spin_thread = None
_node = None
_inited = False


def init_odom_distance(topic_name: str = "/dog_odom"):
    """
    初始化 ROS2 订阅并在后台线程中持续累计总里程。
    在整个进程生命周期内只需调用一次。
    """
    global _spin_thread, _node, _inited
    if _inited:
        return

    if rclpy is None:
        print("[ODOM] rclpy/nav_msgs 导入失败，里程订阅未启动")
        return

    try:
        rclpy.init(args=None)
        _node = _DogOdomNode(topic_name=topic_name)
    except Exception as exc:
        print(f"[ODOM] 初始化失败，里程订阅未启动: {exc}")
        return

    def _spin():
        try:
            rclpy.spin(_node)
        except Exception as exc:
            print(f"[ODOM] 订阅线程异常: {exc}")

    _spin_thread = threading.Thread(target=_spin, daemon=True)
    _spin_thread.start()
    _inited = True
    print(f"[ODOM] 里程订阅已启动: {topic_name}")


def post_odom_http(endpoint: str, timeout: float = 2.0):
    total_m, speed_mps, has_msg = odom_distance_monitor.get()
    if not has_msg:
        return

    payload = {
        "odom_speed": round(speed_mps, 4),
        "odom_distance": round(total_m, 3),
    }
    data = json.dumps(payload).encode("utf-8")
    req = urllib.request.Request(
        endpoint,
        data=data,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            if resp.status != 200:
                print(f"[ODOM] HTTP状态异常: {resp.status}")
    except urllib.error.URLError as exc:
        print(f"[ODOM] HTTP发送失败: {exc}")
    except Exception as exc:
        print(f"[ODOM] HTTP发送异常: {exc}")


def parse_args():
    parser = argparse.ArgumentParser(description="里程计订阅并HTTP上报")
    parser.add_argument("--topic", default="/dog_odom", help="里程计topic，默认 /dog_odom")
    parser.add_argument(
        "--endpoint",
        default="http://127.0.0.1:8091/robotdog/odom",
        help="HTTP接收地址",
    )
    parser.add_argument("--interval", type=float, default=0.5, help="HTTP上报间隔(秒)")
    return parser.parse_args()


def main():
    args = parse_args()
    init_odom_distance(args.topic)

    last_print = time.time()
    last_post = 0.0
    try:
        while True:
            total_m, speed_mps, has_msg = odom_distance_monitor.get()
            now = time.time()
            if has_msg and now - last_print >= PRINT_INTERVAL:
                print(f"开机后总里程: {total_m:.3f} m, 线速度: {speed_mps:.3f} m/s")
                last_print = now

            if now - last_post >= max(0.1, args.interval):
                post_odom_http(args.endpoint)
                last_post = now

            time.sleep(0.2)
    except KeyboardInterrupt:
        print("\n停止里程订阅")
        if rclpy is not None:
            try:
                if _node is not None:
                    _node.destroy_node()
                rclpy.shutdown()
            except Exception:
                pass
        sys.exit(0)


if __name__ == "__main__":
    main()

