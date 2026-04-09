#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
导航公共工具函数。

本模块只放不依赖业务全局变量的函数，供多个脚本复用。
"""

import json
import math
from pyproj import Geod, Transformer


def normalize_angle(angle):
    """将角度归一化到 (-180, 180]。"""
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle


def get_utm_epsg(lon, lat):
    """根据经纬度返回 UTM EPSG 编号。"""
    zone = int((lon + 180) / 6) + 1
    if lat >= 0:
        return 32600 + zone
    return 32700 + zone


def compute_route_length_meters(gps_coords):
    """根据经纬度列表计算路径总长度（米），输入点格式为 [lon, lat]。"""
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


def parse_agrica(line):
    """解析 AGRICA 字符串，成功返回字段列表，失败返回 None。"""
    try:
        if "#AGRICA" not in line:
            return None
        data_part = line.split(";")[1]
        data_part = data_part.split("*")[0]
        return data_part.split(",")
    except Exception:
        return None


def _strip_nmea_checksum(line):
    if not line:
        return ""
    body = line.strip()
    if "*" in body:
        body = body.split("*", 1)[0]
    return body


def _parse_nmea_latlon(value, hemi):
    raw = float(value)
    degrees = int(raw / 100)
    minutes = raw - degrees * 100
    decimal = degrees + minutes / 60.0
    if hemi in ("S", "W"):
        decimal = -decimal
    return decimal


def parse_nmea_gga(line):
    """
    解析 GPGGA/GNGGA，返回:
      {
        "lat": float,
        "lon": float,
        "quality": int,
        "altitude_msl": float,
        "geoid_separation": float,
        "altitude_ellipsoid": float,
      }
    """
    try:
        body = _strip_nmea_checksum(line)
        if not (body.startswith("$GPGGA,") or body.startswith("$GNGGA,")):
            return None
        fields = body.split(",")
        if len(fields) < 15:
            return None

        lat = _parse_nmea_latlon(fields[2], fields[3])
        lon = _parse_nmea_latlon(fields[4], fields[5])
        quality = int(fields[6]) if fields[6] else 0
        altitude_msl = float(fields[9]) if fields[9] else 0.0
        geoid_separation = float(fields[11]) if fields[11] else 0.0

        return {
            "lat": lat,
            "lon": lon,
            "quality": quality,
            "altitude_msl": altitude_msl,
            "geoid_separation": geoid_separation,
            "altitude_ellipsoid": altitude_msl + geoid_separation,
        }
    except Exception:
        return None


def parse_nmea_gst(line):
    """
    解析 GPGST/GNGST，返回:
      {
        "rms": float,
        "smjr_std": float,
        "smnr_std": float,
        "lat_std": float,
        "lon_std": float,
        "alt_std": float,
      }
    """
    try:
        body = _strip_nmea_checksum(line)
        if not (body.startswith("$GPGST,") or body.startswith("$GNGST,")):
            return None
        fields = body.split(",")
        if len(fields) < 9:
            return None

        return {
            "rms": float(fields[2]) if fields[2] else 0.0,
            "smjr_std": float(fields[3]) if fields[3] else 0.0,
            "smnr_std": float(fields[4]) if fields[4] else 0.0,
            "lat_std": float(fields[6]) if fields[6] else 0.0,
            "lon_std": float(fields[7]) if fields[7] else 0.0,
            "alt_std": float(fields[8]) if fields[8] else 0.0,
        }
    except Exception:
        return None


def parse_target_lonlat(raw_target):
    """
    解析 startTask 携带的 target，支持:
      - 字符串 JSON: "[lon, lat]"
      - 数组: [lon, lat]
    严格空判定：仅缺失/None/"" 视为空。
    """
    if raw_target is None or raw_target == "":
        return None

    target_obj = raw_target
    if isinstance(raw_target, str):
        try:
            target_obj = json.loads(raw_target)
        except Exception:
            return None

    if not isinstance(target_obj, list) or len(target_obj) < 2:
        return None

    try:
        lon_val = float(target_obj[0])
        lat_val = float(target_obj[1])
        return lon_val, lat_val
    except Exception:
        return None


def find_nearest_waypoint_index_xy(target_xy, waypoints_xy):
    """
    在平面坐标航点列表中，寻找离 target_xy 最近的点索引。

    参数:
      - target_xy: (x, y)
      - waypoints_xy: [(x, y), ...]
    """
    if not waypoints_xy:
        return None
    tx, ty = float(target_xy[0]), float(target_xy[1])
    nearest_idx = None
    nearest_dist = None
    for idx, (x, y) in enumerate(waypoints_xy):
        dist = math.hypot(tx - float(x), ty - float(y))
        if nearest_dist is None or dist < nearest_dist:
            nearest_dist = dist
            nearest_idx = idx
    return nearest_idx


def find_nearest_waypoint_index_lonlat(target_lon, target_lat, waypoints_xy, origin_utm_x, origin_utm_y):
    """
    根据经纬度 target 计算离局部坐标航点列表最近的索引。

    参数:
      - target_lon, target_lat: 目标经纬度
      - waypoints_xy: 局部坐标航点 [(x, y), ...]
      - origin_utm_x, origin_utm_y: 局部坐标原点 UTM
    """
    if not waypoints_xy:
        return None
    if origin_utm_x is None or origin_utm_y is None:
        return None
    try:
        epsg = get_utm_epsg(target_lon, target_lat)
        transformer = Transformer.from_crs("EPSG:4326", f"EPSG:{epsg}", always_xy=True)
        utm_x, utm_y = transformer.transform(target_lon, target_lat)
        target_x = utm_x - origin_utm_x
        target_y = utm_y - origin_utm_y
    except Exception:
        return None
    return find_nearest_waypoint_index_xy((target_x, target_y), waypoints_xy)
