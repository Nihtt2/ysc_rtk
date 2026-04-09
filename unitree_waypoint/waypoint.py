#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''
模块名称：航点数据类
版本号：V1.0
功能说明：定义 Waypoint 数据类，包含名称、经纬度、本地坐标、动作类型和停留时间。
          供 rtk_cors_4g.py、route_segment_planner.py 等模块统一使用。
'''
from dataclasses import dataclass


@dataclass
class Waypoint:
    name: str = ""
    lon: float = 0.0
    lat: float = 0.0
    action_type: int = 0  # 0=无动作  1=停留
    time: float = 0.0     # 停留秒数（action_type=1 时生效）
