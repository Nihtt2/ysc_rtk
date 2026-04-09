#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MySQL 路线持久化：routes（id 主键、name、next_wp_index、status）与 route_waypoints。
供 robotdog_ws_client（写入）与 rtk_cors_4g（读进度、更新）使用。

手动建库建表可参考同目录 schema.sql；程序也会在首次连接时 ensure_schema。
"""

from __future__ import annotations

import json
import os
from typing import Any, Dict, List, Optional, Tuple

# 与 robotdog_ws_client / rtk_cors_4g 约定的 meta 路径（相对运行目录）
ACTIVE_ROUTE_META_FILE = "tmp/active_route_meta.json"
ACTIVE_ROUTE_STATUS_SET = frozenset({"active", "completed", "aborted"})

_SCHEMA_OK = False
_mysql_params: Optional[Dict[str, Any]] = None
_UNSET = object()


def load_persistence_settings(config_path: str = "config.json") -> Tuple[bool, Optional[Dict[str, Any]]]:
    """返回 (是否启用, pymysql 连接参数字典或 None)。"""
    try:
        with open(config_path, "r", encoding="utf-8") as f:
            cfg = json.load(f)
    except Exception:
        return False, None
    if not cfg.get("route_persistence_enabled"):
        return False, None
    mysql = cfg.get("mysql")
    if not isinstance(mysql, dict):
        return False, None
    for key in ("host", "user", "database"):
        if not mysql.get(key):
            return False, None
    params = {
        "host": mysql["host"],
        "port": int(mysql.get("port", 3306)),
        "user": mysql["user"],
        "password": mysql.get("password", ""),
        "database": mysql["database"],
        "charset": "utf8mb4",
        "autocommit": True,
    }
    return True, params


def _get_mysql_params(config_path: str = "config.json") -> Optional[Dict[str, Any]]:
    global _mysql_params
    ok, params = load_persistence_settings(config_path)
    if not ok or not params:
        _mysql_params = None
        return None
    _mysql_params = params
    return params


def _connect(config_path: str = "config.json"):
    import pymysql

    params = _get_mysql_params(config_path)
    if not params:
        return None
    try:
        return pymysql.connect(**params)
    except Exception as exc:
        print(f"[unitree_db] MySQL 连接失败: {exc}")
        return None


def ensure_schema(conn) -> bool:
    global _SCHEMA_OK
    if _SCHEMA_OK:
        return True
    try:
        with conn.cursor() as cur:
            cur.execute(
                """
                CREATE TABLE IF NOT EXISTS routes (
                  id VARCHAR(128) NOT NULL PRIMARY KEY,
                  name VARCHAR(256) NOT NULL DEFAULT '',
                  next_wp_index INT UNSIGNED NOT NULL DEFAULT 0,
                  status ENUM('active', 'completed', 'aborted') NOT NULL DEFAULT 'active',
                  created_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
                  updated_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP
                ) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4
                """
            )
            cur.execute(
                """
                CREATE TABLE IF NOT EXISTS route_waypoints (
                  route_id VARCHAR(128) NOT NULL,
                  seq INT UNSIGNED NOT NULL,
                  name VARCHAR(256) NOT NULL DEFAULT '',
                  lon DOUBLE NOT NULL,
                  lat DOUBLE NOT NULL,
                  action_type INT NOT NULL DEFAULT 0,
                  `time` DOUBLE NOT NULL DEFAULT 0,
                  PRIMARY KEY (route_id, seq),
                  CONSTRAINT fk_route_waypoints_route
                    FOREIGN KEY (route_id) REFERENCES routes(id) ON DELETE CASCADE
                ) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4
                """
            )
        _SCHEMA_OK = True
        return True
    except Exception as exc:
        print(f"[unitree_db] 建表失败: {exc}")
        return False


def upsert_route_and_waypoints(
    route_id: str,
    route_name: Optional[str],
    waypoint_rows: List[Dict[str, Any]],
    config_path: str = "config.json",
) -> bool:
    """
    waypoint_rows: 每项含 seq, name, lon, lat, action_type, time。
    同一 route_id 再次调用：重置子表、next_wp_index=0、status=active。
    """
    if not route_id:
        return False
    conn = _connect(config_path)
    if not conn:
        return False
    if not ensure_schema(conn):
        conn.close()
        return False
    name = (route_name or "").strip()
    try:
        with conn.cursor() as cur:
            cur.execute(
                """
                INSERT INTO routes (id, name, next_wp_index, status)
                VALUES (%s, %s, 0, 'active')
                ON DUPLICATE KEY UPDATE
                  name = VALUES(name),
                  next_wp_index = 0,
                  status = 'active',
                  updated_at = CURRENT_TIMESTAMP
                """,
                (route_id, name),
            )
            cur.execute("DELETE FROM route_waypoints WHERE route_id = %s", (route_id,))
            for row in waypoint_rows:
                cur.execute(
                    """
                    INSERT INTO route_waypoints
                      (route_id, seq, name, lon, lat, action_type, `time`)
                    VALUES (%s, %s, %s, %s, %s, %s, %s)
                    """,
                    (
                        route_id,
                        int(row["seq"]),
                        str(row.get("name", "")),
                        float(row["lon"]),
                        float(row["lat"]),
                        int(row.get("action_type", 0)),
                        float(row.get("time", 0.0)),
                    ),
                )
        return True
    except Exception as exc:
        print(f"[unitree_db] upsert_route_and_waypoints 失败: {exc}")
        return False
    finally:
        conn.close()


def get_route_progress(
    route_id: str, config_path: str = "config.json"
) -> Optional[Tuple[int, str]]:
    """返回 (next_wp_index, status)，无记录返回 None。"""
    if not route_id:
        return None
    conn = _connect(config_path)
    if not conn:
        return None
    if not ensure_schema(conn):
        conn.close()
        return None
    try:
        with conn.cursor() as cur:
            cur.execute(
                "SELECT next_wp_index, status FROM routes WHERE id = %s",
                (route_id,),
            )
            row = cur.fetchone()
            if not row:
                return None
            return int(row[0]), str(row[1])
    except Exception as exc:
        print(f"[unitree_db] get_route_progress 失败: {exc}")
        return None
    finally:
        conn.close()


def set_route_progress(
    route_id: str, next_wp_index: int, config_path: str = "config.json"
) -> bool:
    if not route_id:
        return False
    conn = _connect(config_path)
    if not conn:
        return False
    try:
        with conn.cursor() as cur:
            cur.execute(
                "UPDATE routes SET next_wp_index = %s WHERE id = %s",
                (int(next_wp_index), route_id),
            )
        return True
    except Exception as exc:
        print(f"[unitree_db] set_route_progress 失败: {exc}")
        return False
    finally:
        conn.close()


def set_route_status(
    route_id: str, status: str, config_path: str = "config.json"
) -> bool:
    if not route_id or status not in ("active", "completed", "aborted"):
        return False
    conn = _connect(config_path)
    if not conn:
        return False
    try:
        with conn.cursor() as cur:
            cur.execute(
                "UPDATE routes SET status = %s WHERE id = %s",
                (status, route_id),
            )
        return True
    except Exception as exc:
        print(f"[unitree_db] set_route_status 失败: {exc}")
        return False
    finally:
        conn.close()


def _write_active_route_meta_file(meta: Dict[str, Any]) -> None:
    d = os.path.dirname(ACTIVE_ROUTE_META_FILE)
    if d and not os.path.exists(d):
        os.makedirs(d, exist_ok=True)
    try:
        with open(ACTIVE_ROUTE_META_FILE, "w", encoding="utf-8") as f:
            json.dump(meta, f, ensure_ascii=False, indent=2)
    except Exception as exc:
        print(f"[unitree_db] 写入 {ACTIVE_ROUTE_META_FILE} 失败: {exc}")


def write_active_route_meta(
    route_id: Optional[str],
    waypoint_file: str,
    route_name: Optional[str] = None,
    target: Any = _UNSET,
    next_wp_index: int = 0,
    status: str = "active",
) -> None:
    """供断电后 --resume-active 读取。route_id 可为空。"""
    if not waypoint_file:
        return

    meta: Dict[str, Any] = {
        "waypoint_file": waypoint_file,
        "next_wp_index": max(0, int(next_wp_index)),
        "status": status if status in ACTIVE_ROUTE_STATUS_SET else "active",
    }
    rid = str(route_id).strip() if route_id is not None else ""
    if rid:
        meta["route_id"] = rid
    if route_name is not None:
        meta["route_name"] = route_name
    if target is not _UNSET:
        meta["target"] = target
    _write_active_route_meta_file(meta)


def read_active_route_meta() -> Optional[Dict[str, Any]]:
    try:
        with open(ACTIVE_ROUTE_META_FILE, "r", encoding="utf-8") as f:
            data = json.load(f)
        if isinstance(data, dict) and data.get("waypoint_file"):
            return data
    except Exception:
        pass
    return None


def update_active_route_meta(
    *,
    route_id: Any = _UNSET,
    waypoint_file: Any = _UNSET,
    route_name: Any = _UNSET,
    target: Any = _UNSET,
    next_wp_index: Any = _UNSET,
    status: Any = _UNSET,
) -> None:
    meta = read_active_route_meta() or {}

    if waypoint_file is not _UNSET:
        if waypoint_file:
            meta["waypoint_file"] = waypoint_file
        else:
            meta.pop("waypoint_file", None)
    if route_id is not _UNSET:
        rid = str(route_id).strip() if route_id is not None else ""
        if rid:
            meta["route_id"] = rid
        else:
            meta.pop("route_id", None)
    if route_name is not _UNSET:
        if route_name is None:
            meta.pop("route_name", None)
        else:
            meta["route_name"] = route_name
    if target is not _UNSET:
        if target is None:
            meta.pop("target", None)
        else:
            meta["target"] = target
    if next_wp_index is not _UNSET:
        meta["next_wp_index"] = max(0, int(next_wp_index))
    if status is not _UNSET:
        meta["status"] = status if status in ACTIVE_ROUTE_STATUS_SET else "active"

    if not meta.get("waypoint_file"):
        return
    _write_active_route_meta_file(meta)


def clear_active_route_meta() -> None:
    try:
        os.remove(ACTIVE_ROUTE_META_FILE)
    except FileNotFoundError:
        return
    except Exception as exc:
        print(f"[unitree_db] 删除 {ACTIVE_ROUTE_META_FILE} 失败: {exc}")


def normalized_waypoint_rows_from_turning_data(data: List[Any]) -> Optional[List[Dict[str, Any]]]:
    """
    将 RobotDogTaskTurning 的 data 转为写库行。
    支持 [[lon,lat],...] 或 [{"lon","lat",...},...]。
    """
    if not isinstance(data, list) or not data:
        return None
    rows: List[Dict[str, Any]] = []
    if isinstance(data[0], dict):
        for i, p in enumerate(data):
            if not isinstance(p, dict):
                continue
            try:
                lon = float(p["lon"])
                lat = float(p["lat"])
            except (KeyError, TypeError, ValueError):
                continue
            rows.append(
                {
                    "seq": i + 1,
                    "name": str(p.get("name", str(i + 1))),
                    "lon": lon,
                    "lat": lat,
                    "action_type": int(p.get("action_type", p.get("actionType", 0))),
                    "time": float(p.get("time", 0.0)),
                }
            )
    elif isinstance(data[0], (list, tuple)):
        for i, p in enumerate(data):
            if not isinstance(p, (list, tuple)) or len(p) < 2:
                continue
            try:
                lon = float(p[0])
                lat = float(p[1])
            except (TypeError, ValueError):
                continue
            rows.append(
                {
                    "seq": i + 1,
                    "name": str(i + 1),
                    "lon": lon,
                    "lat": lat,
                    "action_type": 0,
                    "time": 0.0,
                }
            )
    return rows if rows else None
