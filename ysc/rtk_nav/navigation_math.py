# -*- coding: utf-8 -*-
"""导航控制中的纯数学与分段规划。"""

from __future__ import annotations

import math
from typing import List, Tuple


def compute_quadratic_approach_speed(
    dist: float,
    arrival_radius: float,
    nav_max_vx: float,
    nav_min_vx: float,
    nav_decel_start_dist: float,
    nav_min_vx_reached_dist: float,
) -> float:
    """
    按距离返回前向速度目标值：
      - >= nav_decel_start_dist：保持最高速
      - nav_decel_start_dist -> nav_min_vx_reached_dist：二次函数减速到最低巡航速度
      - < nav_min_vx_reached_dist：保持最低巡航速度，直到到点
    """
    if dist <= arrival_radius:
        return 0.0

    if dist >= nav_decel_start_dist:
        return nav_max_vx

    if dist >= nav_min_vx_reached_dist:
        ratio = (
            (dist - nav_min_vx_reached_dist)
            / max(nav_decel_start_dist - nav_min_vx_reached_dist, 1e-6)
        )
        ratio = max(0.0, min(1.0, ratio))
        return nav_min_vx + (nav_max_vx - nav_min_vx) * (ratio ** 2)

    return nav_min_vx


def build_long_distance_checkpoints(
    start_x: float,
    start_y: float,
    target_x: float,
    target_y: float,
    trigger_dist: float,
    step_dist: float,
) -> List[Tuple[float, float, bool, float]]:
    """
    当前点到目标点距离大于阈值时，每 step_dist 米生成一个中间点，最后再追加真实目标点。
    返回：(x, y, is_midpoint, cumulative_dist)。
    """
    dx = target_x - start_x
    dy = target_y - start_y
    total_dist = math.hypot(dx, dy)

    if total_dist <= trigger_dist:
        return [(target_x, target_y, False, total_dist)]

    checkpoints = []
    mid_idx = 0
    next_dist = step_dist
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
        next_dist = (mid_idx + 1) * step_dist

    checkpoints.append((target_x, target_y, False, total_dist))
    return checkpoints
