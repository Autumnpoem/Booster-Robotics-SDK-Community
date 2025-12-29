#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
遥控器状态监控脚本
用于查看真实遥控器发送的消息内容

作者：He
日期：2025-12-29
"""

import time
import sys

try:
    from booster_robotics_sdk_python import (
        B1RemoteControllerStateSubscriber,
        ChannelFactory
    )
except ImportError:
    print("❌ Error: booster_robotics_sdk_python not found.")
    sys.exit(1)


def main():
    print("=" * 60)
    print("  🎮 遥控器状态监控")
    print("=" * 60)
    print("  正在初始化...")
    
    # 初始化通道
    ChannelFactory.Instance().Init(0, "127.0.0.1")
    
    def callback(msg):
        """遥控器状态回调"""
        # 只有在有输入时才打印
        if (abs(msg.lx) > 0.01 or abs(msg.ly) > 0.01 or 
            abs(msg.rx) > 0.01 or abs(msg.ry) > 0.01 or
            msg.lt or msg.rt or msg.a or msg.b or msg.x or msg.y):
            print(f"event={msg.event:4d} | "
                  f"L: x={msg.lx:+.2f} y={msg.ly:+.2f} | "
                  f"R: x={msg.rx:+.2f} y={msg.ry:+.2f} | "
                  f"LT={int(msg.lt)} RT={int(msg.rt)} "
                  f"A={int(msg.a)} B={int(msg.b)} X={int(msg.x)} Y={int(msg.y)}")
    
    # 订阅遥控器状态
    sub = B1RemoteControllerStateSubscriber(callback)
    sub.InitChannel()
    
    print("  ✓ 初始化成功")
    print("  按 Ctrl+C 退出")
    print("=" * 60)
    print("  event  |  左摇杆 (lx, ly)  |  右摇杆 (rx, ry)  |  按键")
    print("-" * 60)
    
    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n\n✅ 监控已停止")
        sub.CloseChannel()


if __name__ == '__main__':
    main()
