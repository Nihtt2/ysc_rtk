# -*- coding: utf-8 -*-
"""YSC 路线持久化与恢复能力。"""

from .route_db import (
    ACTIVE_ROUTE_META_FILE,
    ACTIVE_ROUTE_STATUS_SET,
    clear_active_route_meta,
    ensure_schema,
    get_route_progress,
    load_persistence_settings,
    normalized_waypoint_rows_from_turning_data,
    read_active_route_meta,
    set_route_progress,
    set_route_status,
    update_active_route_meta,
    upsert_route_and_waypoints,
    write_active_route_meta,
)

__all__ = [
    "ACTIVE_ROUTE_META_FILE",
    "ACTIVE_ROUTE_STATUS_SET",
    "clear_active_route_meta",
    "ensure_schema",
    "get_route_progress",
    "load_persistence_settings",
    "normalized_waypoint_rows_from_turning_data",
    "read_active_route_meta",
    "set_route_progress",
    "set_route_status",
    "update_active_route_meta",
    "upsert_route_and_waypoints",
    "write_active_route_meta",
]
