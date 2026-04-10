# -*- coding: utf-8 -*-
"""RTK 导航配置读取。"""

from __future__ import annotations

import json
from typing import Tuple


def load_android_api_config(config_path: str = "config.json") -> Tuple[str, int]:
    """从 config.json 读取 Android HTTP API host/port，失败时回退默认值。"""
    try:
        with open(config_path, "r", encoding="utf-8") as f:
            cfg = json.load(f)
        host = cfg.get("android_host", "10.65.42.98")
        port = int(cfg.get("api_port", 8080))
        return host, port
    except Exception:
        return "10.65.42.98", 8080


def load_rtk_status_push_url(config_path: str = "config.json") -> str:
    """读取 RTK 状态上报地址。"""
    try:
        with open(config_path, "r", encoding="utf-8") as f:
            cfg = json.load(f)
        host = cfg.get("rtk_status_host", "127.0.0.1")
        port = int(cfg.get("status_http_port", cfg.get("odom_http_port", 8091)))
        return f"http://{host}:{port}/robotdog/rtk"
    except Exception:
        return "http://127.0.0.1:8091/robotdog/rtk"
