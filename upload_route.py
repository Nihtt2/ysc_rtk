#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
upload_route_only.py — 独立的“路径上传”工具

用途：
  你已经在本地录好了路径（JSON 文件），但没法传给前端时，用这个脚本单独上传。

支持两种输入格式（自动识别）：
  1) 新格式对象数组（与 rtk_cors_4g.py 当前一致）：
       [{"name":"1","lon":..,"lat":..,"action_type":0,"time":0}, ...]
  2) 旧格式数组（向后兼容）：
       [[lon, lat], [lon, lat], ...]

默认会读取同目录的 config.json：
  {
    "android_host": "10.65.42.98",
    "api_port": 8080
  }

用法：
  python upload_route_only.py --file waypoints_20260319_120000.json --name "测试路线"
"""

import argparse
import json
import math
import os
import urllib.request
import urllib.error


CONFIG_PATH = "config.json"


def _load_android_config(config_path: str):
    try:
        with open(config_path, "r", encoding="utf-8") as f:
            cfg = json.load(f)
        host = cfg.get("android_host", "10.65.42.98")
        port = int(cfg.get("api_port", 8080))
        return host, port
    except Exception:
        return "10.65.42.98", 8080


def _looks_like_lonlat(points) -> bool:
    """粗略判断是否是经纬度点集（支持对象数组和 [lon,lat] 数组）。"""
    if not points or not isinstance(points, list):
        return False
    for p in points[:5]:
        try:
            if isinstance(p, dict):
                lon = float(p["lon"])
                lat = float(p["lat"])
            else:
                if not isinstance(p, (list, tuple)) or len(p) < 2:
                    return False
                lon = float(p[0])
                lat = float(p[1])
        except Exception:
            return False
        if not (-180.0 <= lon <= 180.0 and -90.0 <= lat <= 90.0):
            return False
    return True


def _haversine_m(lon1, lat1, lon2, lat2) -> float:
    """WGS84 近似球面距离（米），用于不依赖 pyproj 的长度计算。"""
    R = 6378137.0
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = phi2 - phi1
    dlmb = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlmb / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c


def compute_route_length_meters_lonlat(gps_coords) -> float:
    if not gps_coords or len(gps_coords) < 2:
        return 0.0
    total = 0.0
    for i in range(len(gps_coords) - 1):
        p1 = gps_coords[i]
        p2 = gps_coords[i + 1]
        if isinstance(p1, dict):
            lon1, lat1 = float(p1["lon"]), float(p1["lat"])
        else:
            lon1, lat1 = float(p1[0]), float(p1[1])
        if isinstance(p2, dict):
            lon2, lat2 = float(p2["lon"]), float(p2["lat"])
        else:
            lon2, lat2 = float(p2[0]), float(p2[1])
        total += _haversine_m(lon1, lat1, lon2, lat2)
    return total


def _to_upload_coordinates(points):
    """
    统一转换为当前接口要求的对象数组：
    [{"name","lon","lat","action_type","time"}, ...]
    """
    out = []
    for idx, p in enumerate(points):
        if isinstance(p, dict):
            out.append({
                "name": str(p.get("name", idx + 1)),
                "lon": float(p["lon"]),
                "lat": float(p["lat"]),
                "action_type": int(p.get("action_type", 0)),
                "time": float(p.get("time", 0)),
            })
        else:
            out.append({
                "name": str(idx + 1),
                "lon": float(p[0]),
                "lat": float(p[1]),
                "action_type": 0,
                "time": 0.0,
            })
    return out


def _to_route_coordinates_lon_lat(points):
    """
    生成 polyline 点集：
      coordinates: [[lon, lat], ...]
    """
    out = []
    for p in points:
        if isinstance(p, dict):
            lon = float(p["lon"])
            lat = float(p["lat"])
        else:
            lon = float(p[0])
            lat = float(p[1])
        out.append([lon, lat])
    return out


def upload_route(route_name: str, coords, android_host: str, api_port: int) -> None:
    if not coords:
        raise ValueError("路径为空：coordinates 为空或点数为 0")

    route_name = (route_name or "自定义路线").strip()
    if len(route_name) > 15:
        route_name = route_name[:15]

    url = f"http://{android_host}:{api_port}/api/robot-route/upload"

    route_data = _to_upload_coordinates(coords)
    coordinates = _to_route_coordinates_lon_lat(coords)
    is_lonlat = _looks_like_lonlat(coords)
    route_len = compute_route_length_meters_lonlat(route_data) if is_lonlat else 0.0

    body_obj = {
        "routeName": route_name,
        "routeLength": "%.2f" % route_len,
        "coordinates": coordinates,
        "routeData": route_data,
    }

    data = json.dumps(body_obj, ensure_ascii=False).encode("utf-8")
    req = urllib.request.Request(
        url,
        data=data,
        headers={"Content-Type": "application/json"},
        method="POST",
    )

    print(f"[路线] 上传地址: {url}")
    print(
        f"[路线] 名称: {route_name} | 点数: {len(route_data)} | 长度: {route_len:.2f} m | "
        f"lonlat={is_lonlat} | coordinates=lonlat[{len(coordinates)}]"
    )

    try:
        with urllib.request.urlopen(req, timeout=8) as resp:
            resp_body = resp.read().decode("utf-8", errors="ignore")
            print(f"[路线] 上传成功，HTTP {resp.status}: {resp_body}")
    except urllib.error.HTTPError as e:
        try:
            err_body = e.read().decode("utf-8", errors="ignore")
        except Exception:
            err_body = ""
        raise RuntimeError(f"上传失败 HTTP {e.code}: {err_body}") from e
    except Exception as e:
        raise RuntimeError(f"上传失败: {e}") from e


def main():
    parser = argparse.ArgumentParser(description="独立上传本地路径 JSON 到前端(Android)")
    parser.add_argument("--file", required=True, help="本地路径 JSON 文件（对象数组或旧 [[lon,lat],...]）")
    parser.add_argument("--name", default="自定义路线", help="路线名称（最长 15 字符）")
    parser.add_argument("--host", default=None, help="覆盖 config.json 的 android_host")
    parser.add_argument("--port", type=int, default=None, help="覆盖 config.json 的 api_port")
    parser.add_argument("--config", default=CONFIG_PATH, help="config.json 路径（默认同目录）")
    args = parser.parse_args()

    path = args.file
    if not os.path.exists(path):
        raise FileNotFoundError(f"找不到文件: {path}")

    with open(path, "r", encoding="utf-8") as f:
        coords = json.load(f)

    host, port = _load_android_config(args.config)
    if args.host:
        host = args.host
    if args.port is not None:
        port = args.port

    upload_route(args.name, coords, host, port)


if __name__ == "__main__":
    main()


