# -*- coding: utf-8 -*-
"""RTK + ZED 导航业务封装。"""

from .config import load_android_api_config, load_rtk_status_push_url
from .file_state import is_navigation_paused, write_navigation_pause_state
from .mission import (
    MissionProgressState,
    apply_resume_progress_from_meta,
    apply_segment_route_if_needed,
    apply_stored_route_progress,
    configure_target_pause,
    find_nearest_waypoint_index,
    has_target_value,
    route_persistence_enabled,
)
from .navigation_math import (
    build_long_distance_checkpoints,
    compute_quadratic_approach_speed,
)
from .waypoints import load_waypoints_from_file, run_waypoint_action_if_needed

__all__ = [
    "MissionProgressState",
    "apply_resume_progress_from_meta",
    "apply_segment_route_if_needed",
    "apply_stored_route_progress",
    "build_long_distance_checkpoints",
    "compute_quadratic_approach_speed",
    "configure_target_pause",
    "find_nearest_waypoint_index",
    "has_target_value",
    "is_navigation_paused",
    "load_android_api_config",
    "load_rtk_status_push_url",
    "load_waypoints_from_file",
    "route_persistence_enabled",
    "run_waypoint_action_if_needed",
    "write_navigation_pause_state",
]
