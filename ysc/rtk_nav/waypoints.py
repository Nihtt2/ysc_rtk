# -*- coding: utf-8 -*-
"""航点文件读写与到点动作处理。"""

from __future__ import annotations

import json
import time
from typing import Callable, List, Optional, Tuple

from pyproj import Transformer

from nav_utils import get_utm_epsg
from ysc.waypoint import Waypoint


def load_waypoints_from_file(
    waypoint_file: str,
    wait_for_origin: Callable[[], Tuple[float, float]],
) -> Optional[List[Waypoint]]:
    """
    从 JSON 文件加载航点，支持两种格式：
      新格式（对象数组）：
        [{"name":"1","lon":120.1,"lat":30.2,"action_type":0,"time":0}, ...]
      旧格式（兼容）：
        [[lon, lat], ...]
    返回带局部坐标 x/y 的 Waypoint 列表。
    """
    try:
        with open(waypoint_file, "r", encoding="utf-8") as f:
            gps_points = json.load(f)
    except Exception as exc:
        print(f"[航点] 读取航点文件失败 {waypoint_file}: {exc}")
        return None

    if not isinstance(gps_points, list):
        print("[航点] 航点文件格式错误，应为数组")
        return None

    origin_utm_x, origin_utm_y = wait_for_origin()
    waypoints: List[Waypoint] = []

    for idx, item in enumerate(gps_points):
        try:
            if isinstance(item, dict):
                lon_val = float(item["lon"])
                lat_val = float(item["lat"])
                wp_name = item.get("name", f"{idx + 1}")
                action_type = int(item.get("action_type", 0))
                stay_time = float(item.get("time", 0.0))
            else:
                lon_val = float(item[0])
                lat_val = float(item[1])
                wp_name = f"{idx + 1}"
                action_type = 0
                stay_time = 0.0
        except Exception:
            print(f"[航点] 第 {idx} 个点格式错误: {item}")
            continue

        epsg = get_utm_epsg(lon_val, lat_val)
        transformer = Transformer.from_crs("EPSG:4326", f"EPSG:{epsg}", always_xy=True)
        utm_x, utm_y = transformer.transform(lon_val, lat_val)

        x = utm_x - origin_utm_x
        y = utm_y - origin_utm_y
        wp = Waypoint(
            name=wp_name,
            lon=lon_val,
            lat=lat_val,
            action_type=action_type,
            time=stay_time,
        )
        wp.x = x
        wp.y = y
        waypoints.append(wp)

        print(
            f"[航点] 已添加 {idx + 1}: "
            f"lat={lat_val:.6f}, lon={lon_val:.6f} -> "
            f"(x={x:.2f}, y={y:.2f})  name={wp_name}  action={action_type}  time={stay_time}"
        )

    print(f"[航点] 共加载 {len(waypoints)} 个航点")
    return waypoints


def _waypoint_action_sit_then_stand(sport_client, wp) -> None:
    """
    action_type==2：调用 sit（坐下）-> 保持 wp.time 秒 -> stand（起立）-> standard。
    """
    duration = max(0.0, float(wp.time))
    print(f"[动作] 航点 '{wp.name}' 坐下，保持 {duration} 秒后起立并 standard")
    sport_client.stop()
    time.sleep(0.15)
    sport_client.sit()
    if duration > 0:
        time.sleep(duration)
    else:
        time.sleep(0.3)
    sport_client.stand()
    time.sleep(0.25)
    sport_client.standard()


def run_waypoint_action_if_needed(
    sport_client,
    wp,
    scope_label: str,
    task_has_target: bool,
) -> None:
    if task_has_target:
        if wp.action_type in (1, 2):
            print(
                f"[动作] {scope_label} '{wp.name}' 检测到 target，跳过 action_type={wp.action_type}"
            )
        return

    if wp.action_type == 1 and wp.time > 0:
        print(f"[动作] {scope_label} '{wp.name}' 停留 {wp.time} 秒")
        time.sleep(wp.time)
    elif wp.action_type == 2:
        time.sleep(1)
        _waypoint_action_sit_then_stand(sport_client, wp)
