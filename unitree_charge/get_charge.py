#!/usr/bin/env python3
# -*- coding: utf-8 -*-

'''
模块名称：电池电量持续订阅
版本号：V1.0
功能说明：本模块用于持续订阅并打印机器狗电池电量（SOC）。
          可作为独立脚本运行，也可作为模块被其他程序导入。
          导入后通过 battery_monitor.soc 直接读取当前电量值。
'''

import sys
import time
import threading

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_go.msg.dds_._LowState_ import LowState_

DT = 1
PRINT_INTERVAL = 2.0


class BatteryMonitor:
    def __init__(self):
        self._lock = threading.Lock()
        self.soc = None  # 当前电量百分比，供外部直接读取

    def cb(self, msg: LowState_):
        soc = int(msg.bms_state.soc)
        with self._lock:
            self.soc = soc

    def get(self):
        with self._lock:
            return self.soc


# 模块级公开实例，供其他模块直接 import 使用
battery_monitor = BatteryMonitor()

_sub = None  # 保持订阅者引用，防止被 GC 回收


def init_battery(netif: str):
    """
    初始化 DDS 频道并启动电量订阅。
    在整个进程生命周期内只需调用一次。
    """
    global _sub
    ChannelFactoryInitialize(0, netif)
    _sub = ChannelSubscriber("rt/lowstate", LowState_)
    _sub.Init(battery_monitor.cb, 10)


def main():
    if len(sys.argv) < 2:
        print("用法: python get_charge.py <网卡名>")
        print("例如: python get_charge.py eno1")
        sys.exit(1)

    init_battery(sys.argv[1])

    last_print_time = time.time()

    try:
        while True:
            soc = battery_monitor.get()
            if soc is None:
                time.sleep(0.5)
                continue

            now = time.time()
            if now - last_print_time >= PRINT_INTERVAL:
                print(f"当前 SOC: {soc}%")
                last_print_time = now

            time.sleep(DT)
    except KeyboardInterrupt:
        print("\n停止订阅")


if __name__ == "__main__":
    main()