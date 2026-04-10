# -*- coding: utf-8 -*-
"""导航暂停等文件状态控制。"""

from __future__ import annotations

import json
import time


def is_navigation_paused(pause_state_file: str) -> bool:
    """
    从暂停状态文件读取当前是否暂停。
    文件形如：{"paused": true/false, "timestamp": ...}
    """
    try:
        with open(pause_state_file, "r", encoding="utf-8") as f:
            data = json.load(f)
        return bool(data.get("paused"))
    except Exception:
        return False


def write_navigation_pause_state(
    pause_state_file: str,
    paused: bool,
    reason: str | None = None,
) -> None:
    """写入导航暂停状态文件，供导航循环周期读取。"""
    state = {
        "paused": bool(paused),
        "timestamp": time.time(),
    }
    if reason:
        state["reason"] = reason
    try:
        with open(pause_state_file, "w", encoding="utf-8") as f:
            json.dump(state, f, ensure_ascii=False)
        print(f"[暂停] 已更新暂停状态 paused={bool(paused)}")
    except Exception as exc:
        print(f"[暂停] 写入暂停状态失败: {exc}")
