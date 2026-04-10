# -*- coding: utf-8 -*-
"""任务恢复、目标点处理与路线重排。"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

from nav_utils import find_nearest_waypoint_index_lonlat, parse_target_lonlat
from route_segment_planner import build_named_route, plan_route

from ysc.db import (
    clear_active_route_meta,
    get_route_progress,
    load_persistence_settings,
)
from ysc.waypoint import Waypoint


@dataclass
class MissionProgressState:
    current_wp_index: int
    robot_state: int
    active_route_id: Optional[str]


def route_persistence_enabled(config_path: str = "config.json") -> bool:
    ok, _ = load_persistence_settings(config_path)
    return ok


def apply_stored_route_progress(
    state: MissionProgressState,
    waypoint_count: int,
    *,
    cli_point_a: str | None = None,
    cli_point_b: str | None = None,
    config_path: str = "config.json",
    state_idle: int,
) -> None:
    """
    在已加载 waypoints 且（可选）已做分段规划后调用：
    根据库中 next_wp_index / status 设置 current_wp_index；必要时置为待机。
    若启用 --point-a/--point-b 则强制从 0 开始（与库顺序可能不一致）。
    """
    if not state.active_route_id or not route_persistence_enabled(config_path):
        return
    if cli_point_a or cli_point_b:
        print("[ysc_db] 已启用 A/B 分段，忽略库中进度，从索引 0 开始")
        state.current_wp_index = 0
        return

    row = get_route_progress(state.active_route_id, config_path)
    if not row:
        state.current_wp_index = 0
        return

    next_idx, status = row
    if status == "completed" or (waypoint_count > 0 and next_idx >= waypoint_count):
        state.robot_state = state_idle
        state.active_route_id = None
        print("[ysc_db] 路线在库中已完成或进度已走完，保持待机")
        return
    if status != "active":
        state.current_wp_index = 0
        print(f"[ysc_db] 路线状态为 {status}，从索引 0 开始")
        return

    if waypoint_count <= 0:
        state.current_wp_index = 0
        return
    state.current_wp_index = min(max(0, next_idx), waypoint_count - 1)
    print(
        f"[ysc_db] 恢复进度 current_wp_index={state.current_wp_index} "
        f"(库 next_wp_index={next_idx})"
    )


def apply_resume_progress_from_meta(
    state: MissionProgressState,
    resume_meta,
    waypoint_count: int,
    *,
    cli_point_a: str | None = None,
    cli_point_b: str | None = None,
    config_path: str = "config.json",
    state_idle: int,
) -> None:
    """优先按 MySQL 进度恢复；不可用时回退到 active_route_meta.json 中的 next_wp_index。"""
    if cli_point_a or cli_point_b:
        print("[恢复] 已启用 A/B 分段，忽略持久化进度，从索引 0 开始")
        state.current_wp_index = 0
        return

    if state.active_route_id and route_persistence_enabled(config_path):
        row = get_route_progress(state.active_route_id, config_path)
        if row:
            next_idx, status = row
            if status == "completed" or (waypoint_count > 0 and next_idx >= waypoint_count):
                state.robot_state = state_idle
                state.active_route_id = None
                clear_active_route_meta()
                print("[恢复] 路线在库中已完成或进度已走完，保持待机")
                return
            if status != "active":
                state.robot_state = state_idle
                state.active_route_id = None
                clear_active_route_meta()
                print(f"[恢复] 路线状态为 {status}，不自动续跑")
                return
            if waypoint_count <= 0:
                state.current_wp_index = 0
                return
            state.current_wp_index = min(max(0, next_idx), waypoint_count - 1)
            print(
                f"[恢复] 按 MySQL 进度续跑 current_wp_index={state.current_wp_index} "
                f"(库 next_wp_index={next_idx})"
            )
            return
        print("[恢复] 未读取到 MySQL 进度，回退到本地恢复信息")

    if not isinstance(resume_meta, dict):
        state.current_wp_index = 0
        return

    try:
        next_idx = int(resume_meta.get("next_wp_index", 0))
    except (TypeError, ValueError):
        next_idx = 0
    status = str(resume_meta.get("status") or "active")

    if status == "completed" or (waypoint_count > 0 and next_idx >= waypoint_count):
        state.robot_state = state_idle
        state.active_route_id = None
        clear_active_route_meta()
        print("[恢复] 本地恢复信息显示任务已完成，保持待机")
        return
    if status != "active":
        state.robot_state = state_idle
        state.active_route_id = None
        clear_active_route_meta()
        print(f"[恢复] 本地恢复状态为 {status}，不自动续跑")
        return

    if waypoint_count <= 0:
        state.current_wp_index = 0
        return
    state.current_wp_index = min(max(0, next_idx), waypoint_count - 1)
    print(f"[恢复] 按本地进度续跑 current_wp_index={state.current_wp_index}")


def find_nearest_waypoint_index(
    target_lon: float,
    target_lat: float,
    waypoints: Sequence[Waypoint],
    origin_utm_x: float | None,
    origin_utm_y: float | None,
) -> Optional[int]:
    """根据经纬度 target 计算距离最近的航点索引（基于局部坐标）。"""
    return find_nearest_waypoint_index_lonlat(
        target_lon=target_lon,
        target_lat=target_lat,
        waypoints_xy=[(wp.x, wp.y) for wp in waypoints],
        origin_utm_x=origin_utm_x,
        origin_utm_y=origin_utm_y,
    )


def has_target_value(raw_target) -> bool:
    if raw_target is None:
        return False
    if isinstance(raw_target, str):
        return bool(raw_target.strip())
    if isinstance(raw_target, (list, tuple, dict, set)):
        return len(raw_target) > 0
    return True


def configure_target_pause(
    raw_target,
    waypoints: Sequence[Waypoint],
    origin_utm_x: float | None,
    origin_utm_y: float | None,
) -> Tuple[Optional[int], bool]:
    """根据 raw_target 设置最近航点自动暂停逻辑，并决定是否跳过 action_type。"""
    target_pause_wp_index = None
    task_has_target = has_target_value(raw_target)
    parsed_target = parse_target_lonlat(raw_target)
    if parsed_target is None:
        if task_has_target:
            print("[target] target 有值但无法解析，当前任务仍按 target 模式跳过动作")
        return target_pause_wp_index, task_has_target

    nearest_idx = find_nearest_waypoint_index(
        parsed_target[0],
        parsed_target[1],
        waypoints,
        origin_utm_x,
        origin_utm_y,
    )
    if nearest_idx is not None:
        target_pause_wp_index = nearest_idx
        print(
            f"[target] 已设置目标暂停航点索引: {target_pause_wp_index + 1}/{len(waypoints)}"
        )
        print("[target] 当前任务带 target，已禁用 action_type 动作")
    else:
        print("[target] 未能计算最近航点索引，但当前任务仍按 target 模式跳过动作")

    return target_pause_wp_index, task_has_target


def apply_segment_route_if_needed(
    waypoints: Sequence[Waypoint],
    fused_utm_x: float | None,
    fused_utm_y: float | None,
    origin_utm_x: float | None,
    origin_utm_y: float | None,
    *,
    point_a: str | None = None,
    point_b: str | None = None,
    is_loop: int = 0,
) -> list[Waypoint]:
    """
    当传入 A/B 参数时，按“最近点 -> A -> B”重排当前 waypoints。
    若条件不足或规划失败，返回原始路线。
    """
    if point_a is None or point_b is None:
        return list(waypoints)

    if (
        fused_utm_x is None
        or fused_utm_y is None
        or origin_utm_x is None
        or origin_utm_y is None
    ):
        print("[路线规划] 当前位置或原点未就绪，跳过 A->B 分段规划")
        return list(waypoints)

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
        new_waypoints: list[Waypoint] = []
        for point in planned["final_points"]:
            wp = Waypoint(
                name=point["name"],
                lon=point["lon"],
                lat=point["lat"],
                action_type=point["action_type"],
                time=point["time"],
            )
            wp.x = point["x"]
            wp.y = point["y"]
            new_waypoints.append(wp)
        print(
            "[路线规划] 已应用分段路线："
            f"最近点={planned['nearest_idx'] + 1}, "
            f"A={planned['a_idx'] + 1}, B={planned['b_idx'] + 1}, "
            f"is_loop={int(is_loop)}, 点名序列={planned['final_names']}"
        )
        return new_waypoints
    except Exception as exc:
        print(f"[路线规划] 规划失败，回退原始路线: {exc}")
        return list(waypoints)
