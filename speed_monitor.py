#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
速度监控脚本 - 对比遥控器输入速度与程序发送速度
用于调试和分析遥控器是否比程序发送速度更快

使用方法:
    python speed_monitor.py

作者: He
日期: 2025-12-28
"""

import time
import sys
import threading
from collections import deque

# 导入 SDK
try:
    from booster_robotics_sdk_python import (
        ChannelFactory,
        B1RemoteControllerStateSubscriber,
    )
    SDK_AVAILABLE = True
except ImportError:
    print("❌ Error: booster_robotics_sdk_python not found.")
    SDK_AVAILABLE = False
    sys.exit(1)


# =================== 配置参数 ===================
CONFIG = {
    # 遥控器摇杆到速度的映射系数 (基于实测)
    # 测试: SDK 2.0m/s 走 3.4秒, 手柄走 3.0秒 → 手柄速度 = 2.0 * 3.4/3.0 = 2.27
    "JOYSTICK_VX_SCALE": 2.27,   # 左摇杆Y -> 前进速度 (m/s) - 实测值
    "JOYSTICK_VY_SCALE": 1.7,    # 左摇杆X -> 侧移速度 (m/s) - 估算
    "JOYSTICK_VYAW_SCALE": 1.5,  # 右摇杆X -> 转向速度 (rad/s)
    
    # move.py 程序中设置的速度
    "PROGRAM_VX": 2.0,           # 程序设置的前进速度
    "PROGRAM_VYAW": 1.2,         # 程序设置的转向速度
    
    # 监控参数
    "UPDATE_HZ": 10,             # 更新频率 (Hz)
    "HISTORY_SIZE": 50,          # 历史记录大小
}


class SpeedMonitor:
    """速度监控器"""
    
    def __init__(self):
        # 遥控器当前状态
        self.rc_lx = 0.0
        self.rc_ly = 0.0
        self.rc_rx = 0.0
        self.rc_ry = 0.0
        
        # 计算出的速度
        self.rc_vx = 0.0      # 遥控器前进速度
        self.rc_vy = 0.0      # 遥控器侧移速度
        self.rc_vyaw = 0.0    # 遥控器转向速度
        
        # 历史记录
        self.rc_vx_history = deque(maxlen=CONFIG["HISTORY_SIZE"])
        self.rc_vyaw_history = deque(maxlen=CONFIG["HISTORY_SIZE"])
        
        # 统计数据
        self.rc_update_count = 0
        self.last_update_time = 0
        self.rc_update_rate = 0.0  # 遥控器更新频率
        
        # 线程锁
        self.lock = threading.Lock()
        
        # 运行状态
        self.running = True
        
        # 初始化 SDK
        self._init_sdk()
        
    def _init_sdk(self):
        """初始化 SDK"""
        try:
            ChannelFactory.Instance().Init(0, "127.0.0.1")
            print("✓ SDK 初始化成功")
        except Exception as e:
            print(f"❌ SDK 初始化失败: {e}")
            sys.exit(1)
            
        # 初始化遥控器订阅
        try:
            def rc_callback(msg):
                self._update_rc_state(msg)
            
            self.rc_sub = B1RemoteControllerStateSubscriber(rc_callback)
            self.rc_sub.InitChannel()
            print("✓ 遥控器状态订阅初始化成功")
        except Exception as e:
            print(f"❌ 遥控器订阅初始化失败: {e}")
            sys.exit(1)
    
    def _update_rc_state(self, msg):
        """更新遥控器状态"""
        current_time = time.time()
        
        with self.lock:
            # 更新原始摇杆值
            self.rc_lx = msg.lx
            self.rc_ly = msg.ly
            self.rc_rx = msg.rx
            self.rc_ry = msg.ry
            
            # 计算对应的速度
            # 注意: 摇杆值范围通常是 -1 到 1
            # 前进速度 = 左摇杆Y * 速度系数 (向前推是正值)
            self.rc_vx = self.rc_ly * CONFIG["JOYSTICK_VX_SCALE"]
            # 侧移速度 = 左摇杆X * 速度系数
            self.rc_vy = self.rc_lx * CONFIG["JOYSTICK_VY_SCALE"]
            # 转向速度 = 右摇杆X * 速度系数
            self.rc_vyaw = self.rc_rx * CONFIG["JOYSTICK_VYAW_SCALE"]
            
            # 记录历史
            self.rc_vx_history.append(self.rc_vx)
            self.rc_vyaw_history.append(self.rc_vyaw)
            
            # 计算更新频率
            self.rc_update_count += 1
            if self.last_update_time > 0:
                dt = current_time - self.last_update_time
                if dt > 0:
                    self.rc_update_rate = 0.9 * self.rc_update_rate + 0.1 * (1.0 / dt)
            self.last_update_time = current_time
    
    def get_status(self):
        """获取当前状态"""
        with self.lock:
            return {
                "rc_lx": self.rc_lx,
                "rc_ly": self.rc_ly,
                "rc_rx": self.rc_rx,
                "rc_ry": self.rc_ry,
                "rc_vx": self.rc_vx,
                "rc_vy": self.rc_vy,
                "rc_vyaw": self.rc_vyaw,
                "rc_update_rate": self.rc_update_rate,
                "rc_update_count": self.rc_update_count,
                "rc_vx_max": max(self.rc_vx_history) if self.rc_vx_history else 0,
                "rc_vyaw_max": max(abs(v) for v in self.rc_vyaw_history) if self.rc_vyaw_history else 0,
            }
    
    def display_loop(self):
        """显示循环"""
        interval = 1.0 / CONFIG["UPDATE_HZ"]
        
        # 清屏
        print("\033[2J\033[H", end="")
        
        print("=" * 70)
        print("  📊 速度监控器 - 对比遥控器与程序速度")
        print("=" * 70)
        print()
        print("按 Ctrl+C 退出")
        print()
        
        while self.running:
            try:
                status = self.get_status()
                
                # 移动光标到固定位置
                print("\033[7;0H", end="")
                
                print("─" * 70)
                print()
                
                # 遥控器原始输入
                print("📱 遥控器摇杆原始值:")
                print(f"   左摇杆 X (侧移)  : {status['rc_lx']:+7.3f}")
                print(f"   左摇杆 Y (前进)  : {status['rc_ly']:+7.3f}")
                print(f"   右摇杆 X (转向)  : {status['rc_rx']:+7.3f}")
                print(f"   右摇杆 Y         : {status['rc_ry']:+7.3f}")
                print()
                
                # 遥控器计算速度
                print("🎮 遥控器计算速度 (假设映射):")
                print(f"   前进速度 vx  : {status['rc_vx']:+7.3f} m/s   (系数: {CONFIG['JOYSTICK_VX_SCALE']})")
                print(f"   侧移速度 vy  : {status['rc_vy']:+7.3f} m/s   (系数: {CONFIG['JOYSTICK_VY_SCALE']})")
                print(f"   转向速度 vyaw: {status['rc_vyaw']:+7.3f} rad/s (系数: {CONFIG['JOYSTICK_VYAW_SCALE']})")
                print()
                
                # 程序设置速度
                print("🤖 程序发送速度 (move.py 配置):")
                print(f"   前进速度 vx  : {CONFIG['PROGRAM_VX']:+7.3f} m/s")
                print(f"   转向速度 vyaw: {CONFIG['PROGRAM_VYAW']:+7.3f} rad/s")
                print()
                
                # 对比
                print("📈 速度对比:")
                rc_vx = abs(status['rc_vx'])
                prog_vx = CONFIG['PROGRAM_VX']
                rc_vyaw = abs(status['rc_vyaw'])
                prog_vyaw = abs(CONFIG['PROGRAM_VYAW'])
                
                if rc_vx > 0.1:  # 只在有输入时显示
                    vx_ratio = rc_vx / prog_vx * 100 if prog_vx > 0 else 0
                    vx_status = "⚡ 更快" if rc_vx > prog_vx else "🐢 更慢" if rc_vx < prog_vx else "= 相等"
                    print(f"   前进速度: 遥控器 {rc_vx:.2f} vs 程序 {prog_vx:.2f}  → {vx_status} ({vx_ratio:.0f}%)")
                else:
                    print(f"   前进速度: 遥控器 {rc_vx:.2f} vs 程序 {prog_vx:.2f}  → (无输入)")
                
                if rc_vyaw > 0.1:  # 只在有输入时显示
                    vyaw_ratio = rc_vyaw / prog_vyaw * 100 if prog_vyaw > 0 else 0
                    vyaw_status = "⚡ 更快" if rc_vyaw > prog_vyaw else "🐢 更慢" if rc_vyaw < prog_vyaw else "= 相等"
                    print(f"   转向速度: 遥控器 {rc_vyaw:.2f} vs 程序 {prog_vyaw:.2f}  → {vyaw_status} ({vyaw_ratio:.0f}%)")
                else:
                    print(f"   转向速度: 遥控器 {rc_vyaw:.2f} vs 程序 {prog_vyaw:.2f}  → (无输入)")
                print()
                
                # 历史最大值
                print("📊 历史最大速度:")
                print(f"   遥控器最大 vx  : {status['rc_vx_max']:+7.3f} m/s")
                print(f"   遥控器最大 vyaw: {status['rc_vyaw_max']:+7.3f} rad/s")
                print()
                
                # 状态信息
                print("ℹ️  更新信息:")
                print(f"   遥控器更新频率: {status['rc_update_rate']:.1f} Hz")
                print(f"   总更新次数    : {status['rc_update_count']}")
                print()
                print("─" * 70)
                print()
                print("💡 提示: 速度系数可能需要调整以匹配实际的遥控器映射")
                print("   如果对比不准确，请修改脚本中的 JOYSTICK_*_SCALE 参数")
                
                time.sleep(interval)
                
            except KeyboardInterrupt:
                break
            except Exception as e:
                print(f"\n❌ 显示错误: {e}")
                time.sleep(1)
    
    def run(self):
        """运行监控器"""
        try:
            self.display_loop()
        except KeyboardInterrupt:
            pass
        finally:
            self.running = False
            print("\n\n👋 监控器已停止")


def main():
    """主函数"""
    print("🚀 启动速度监控器...")
    print()
    
    monitor = SpeedMonitor()
    monitor.run()


if __name__ == "__main__":
    main()
