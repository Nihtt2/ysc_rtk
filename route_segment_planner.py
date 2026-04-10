#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
分段路线规划模块：
1) 先找整条路线上离当前位置最近的点
2) 从最近点到 A（环线时走更短方向）
3) 从 A 到 B
"""

import math
from nav_utils import find_nearest_waypoint_index_xy
from ysc.waypoint import Waypoint


def build_named_route(waypoints):
    """
    将 [Waypoint, ...] 转成内部处理用的字典列表，保留所有字段。
    final_points 返回时同样包含全量字段，可直接还原为 Waypoint。
    """
    named = []
    for idx, wp in enumerate(waypoints):
        named.append({
            "name":        wp.name or str(idx + 1),
            "lon":         wp.lon,
            "lat":         wp.lat,
            "x":           float(wp.x),
            "y":           float(wp.y),
            "action_type": wp.action_type,
            "time":        wp.time,
        })
    return named


def _distance(p1, p2):
    return math.hypot(float(p1["x"]) - float(p2["x"]), float(p1["y"]) - float(p2["y"]))


def _path_distance(route_points, idx_list):
    if not idx_list or len(idx_list) < 2:
        return 0.0
    total = 0.0
    for i in range(len(idx_list) - 1):
        total += _distance(route_points[idx_list[i]], route_points[idx_list[i + 1]])
    return total


def _forward_indices(start_idx, end_idx, n, is_loop):
    if is_loop:
        out = [start_idx]
        i = start_idx
        while i != end_idx:
            i = (i + 1) % n
            out.append(i)
        return out
    if start_idx <= end_idx:
        return list(range(start_idx, end_idx + 1))
    return []


def _backward_indices(start_idx, end_idx, n, is_loop):
    if is_loop:
        out = [start_idx]
        i = start_idx
        while i != end_idx:
            i = (i - 1 + n) % n
            out.append(i)
        return out
    if start_idx >= end_idx:
        return list(range(start_idx, end_idx - 1, -1))
    return []


def path_between_indices(start_idx, end_idx, route_points, is_loop):
    """
    返回 start_idx -> end_idx 的索引路径。
    - is_loop=1: 正反两方向都计算，取几何长度更短
    - is_loop=0: 优先正向，若不可行再反向
    """
    n = len(route_points)
    if n == 0:
        return []
    if start_idx < 0 or end_idx < 0 or start_idx >= n or end_idx >= n:
        return []
    if start_idx == end_idx:
        return [start_idx]

    loop_flag = bool(int(is_loop))
    fwd = _forward_indices(start_idx, end_idx, n, loop_flag)
    bwd = _backward_indices(start_idx, end_idx, n, loop_flag)

    if loop_flag:
        if not fwd:
            return bwd
        if not bwd:
            return fwd
        return fwd if _path_distance(route_points, fwd) <= _path_distance(route_points, bwd) else bwd

    if fwd:
        return fwd
    return bwd


def _resolve_point_index(route_points, point_ref):
    """
    支持按名称或按索引指定点:
    - 名称: "12"
    - 索引: 12 (0-based)
    """
    if point_ref is None:
        return None
    if isinstance(point_ref, int):
        if 0 <= point_ref < len(route_points):
            return point_ref
        return None
    target_name = str(point_ref)
    for idx, p in enumerate(route_points):
        if str(p.get("name")) == target_name:
            return idx
    return None


def plan_route(current_xy, route_points, point_a, point_b, is_loop):
    """
    生成完整执行路线（点索引与点对象）:
      当前 -> 最近点 -> A -> B
    """
    if not route_points:
        raise ValueError("route_points 不能为空")

    a_idx = _resolve_point_index(route_points, point_a)
    b_idx = _resolve_point_index(route_points, point_b)
    if a_idx is None:
        raise ValueError(f"A 点不存在: {point_a}")
    if b_idx is None:
        raise ValueError(f"B 点不存在: {point_b}")

    waypoints_xy = [(p["x"], p["y"]) for p in route_points]
    nearest_idx = find_nearest_waypoint_index_xy(current_xy, waypoints_xy)
    if nearest_idx is None:
        raise ValueError("无法计算最近点")

    nearest_to_a = path_between_indices(nearest_idx, a_idx, route_points, is_loop)
    a_to_b = path_between_indices(a_idx, b_idx, route_points, is_loop)

    merged = []
    for seg in (nearest_to_a, a_to_b):
        for idx in seg:
            if not merged or merged[-1] != idx:
                merged.append(idx)

    return {
        "is_loop": int(is_loop),
        "nearest_idx": nearest_idx,
        "a_idx": a_idx,
        "b_idx": b_idx,
        "nearest_to_a_indices": nearest_to_a,
        "a_to_b_indices": a_to_b,
        "final_indices": merged,
        "final_points": [route_points[i] for i in merged],
        "final_names": [str(route_points[i]["name"]) for i in merged],
    }
