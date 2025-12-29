#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
机器人端 UDP 接收程序 (Robot Sink)
=====================================
功能：
  - 通过 UDP 接收上位机发送的 [vx, vy, wz] 指令
  - 调用 loco.Move(vx, vy, wz) 控制机器人运动
  - Watchdog 机制：200ms 内未收到有效包 → 立即执行 Move(0,0,0)

使用方法：
  在机器人上运行：
  $ python3 robot_sink.py

  测试模式（不连接真实SDK）：
  $ python3 robot_sink.py --test-mode

通信协议：
  UDP包格式："{vx},{vy},{wz}"
  例如："0.5,0.0,0.2"

作者：He
日期：2025-12-28
"""

import socket
import struct
import time
import threading
import argparse
import sys

# ================= 配置参数 =================
CONFIG = {
    "UDP_PORT": 5000,                # UDP 监听端口
    "WATCHDOG_TIMEOUT_MS": 200,      # Watchdog 超时时间 (毫秒)
    "CONTROL_HZ": 50,                # 控制频率 (Hz)
    "MAX_VX": 2.0,                   # 最大前进速度 (m/s)
    "MAX_VY": 1.5,                   # 最大侧向速度 (m/s)
    "MAX_WZ": 2.0,                   # 最大转向速度 (rad/s)
}


class RobotSink:
    """
    机器人端 UDP 接收器
    
    核心安全机制：
    - Watchdog：200ms 内未收到有效指令 → 自动停止机器人
    - 速度限制：限制最大速度范围，防止危险指令
    """
    
    def __init__(self, port=5000, watchdog_timeout_ms=200, test_mode=False):
        """
        初始化接收器
        
        Args:
            port: UDP 监听端口
            watchdog_timeout_ms: Watchdog 超时时间 (毫秒)
            test_mode: 测试模式 (不连接真实SDK)
        """
        self.port = port
        self.watchdog_timeout = watchdog_timeout_ms / 1000.0  # 转换为秒
        self.test_mode = test_mode
        self.running = False
        
        # 当前指令 (线程安全)
        self.cmd_lock = threading.Lock()
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_wz = 0.0
        self.last_recv_time = 0.0
        
        # 统计信息
        self.packet_count = 0
        self.watchdog_trigger_count = 0
        
        # SDK 初始化
        self.loco = None
        if not test_mode:
            self._init_sdk()
        else:
            print("🧪 测试模式：跳过 SDK 初始化")
    
    def _init_sdk(self):
        """初始化机器人 SDK"""
        try:
            from booster_robotics_sdk_python import B1LocoClient, ChannelFactory
            
            ChannelFactory.Instance().Init(0, "127.0.0.1")
            self.loco = B1LocoClient()
            self.loco.Init()
            time.sleep(0.5)
            
            # 确保初始状态为停止
            for _ in range(10):
                self.loco.Move(0.0, 0.0, 0.0)
                time.sleep(0.01)
            
            print("✓ SDK 初始化成功")
        except ImportError:
            print("❌ 错误：无法导入 booster_robotics_sdk_python")
            print("   请确保在机器人上运行此程序")
            sys.exit(1)
        except Exception as e:
            print(f"❌ SDK 初始化失败: {e}")
            sys.exit(1)
    
    def _clamp_velocity(self, vx, vy, wz):
        """
        限制速度范围
        
        Args:
            vx, vy, wz: 原始速度指令
            
        Returns:
            限制后的速度 (vx, vy, wz)
        """
        vx = max(-CONFIG["MAX_VX"], min(CONFIG["MAX_VX"], vx))
        vy = max(-CONFIG["MAX_VY"], min(CONFIG["MAX_VY"], vy))
        wz = max(-CONFIG["MAX_WZ"], min(CONFIG["MAX_WZ"], wz))
        return vx, vy, wz
    
    def _parse_packet(self, data):
        """
        解析 UDP 数据包
        
        支持格式：
        1. 文本格式："vx,vy,wz" (例如 "0.5,0.0,0.2")
        2. 二进制格式：struct.pack("<fff", vx, vy, wz)
        
        Args:
            data: bytes 数据
            
        Returns:
            (vx, vy, wz) 或 None (解析失败)
        """
        try:
            # 尝试文本格式
            text = data.decode('utf-8').strip()
            parts = text.split(',')
            if len(parts) == 3:
                vx = float(parts[0])
                vy = float(parts[1])
                wz = float(parts[2])
                return self._clamp_velocity(vx, vy, wz)
        except:
            pass
        
        try:
            # 尝试二进制格式 (3个float, 12字节)
            if len(data) == 12:
                vx, vy, wz = struct.unpack("<fff", data)
                return self._clamp_velocity(vx, vy, wz)
        except:
            pass
        
        return None
    
    def _recv_loop(self):
        """UDP 接收循环"""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(('0.0.0.0', self.port))
        sock.settimeout(0.1)  # 100ms 超时，用于检测 running 状态
        
        print(f"📡 UDP 监听端口: {self.port}")
        
        while self.running:
            try:
                data, addr = sock.recvfrom(1024)
                result = self._parse_packet(data)
                
                if result is not None:
                    vx, vy, wz = result
                    
                    with self.cmd_lock:
                        self.current_vx = vx
                        self.current_vy = vy
                        self.current_wz = wz
                        self.last_recv_time = time.time()
                        self.packet_count += 1
                    
                    # 调试输出 (每100个包输出一次)
                    if self.packet_count % 100 == 0:
                        print(f"  📦 收到 {self.packet_count} 个包，当前指令: "
                              f"vx={vx:.2f}, vy={vy:.2f}, wz={wz:.2f}")
                else:
                    print(f"⚠️ 无法解析数据包: {data[:50]}...")
                    
            except socket.timeout:
                continue
            except Exception as e:
                print(f"❌ 接收错误: {e}")
        
        sock.close()
        print("📡 UDP 接收停止")
    
    def _control_loop(self):
        """
        控制循环 + Watchdog 机制
        
        核心安全逻辑：
        - 持续以 CONTROL_HZ 频率发送指令
        - 如果超过 watchdog_timeout 未收到新指令 → 立即停止
        """
        control_interval = 1.0 / CONFIG["CONTROL_HZ"]
        last_log_time = 0
        watchdog_active = False
        
        print(f"🐕 Watchdog 已启动 (超时: {CONFIG['WATCHDOG_TIMEOUT_MS']}ms)")
        
        while self.running:
            now = time.time()
            
            with self.cmd_lock:
                vx = self.current_vx
                vy = self.current_vy
                wz = self.current_wz
                time_since_last = now - self.last_recv_time
            
            # ============ Watchdog 检测 ============
            if self.last_recv_time > 0 and time_since_last > self.watchdog_timeout:
                # 超时！立即停止机器人
                if not watchdog_active:
                    print(f"🚨 Watchdog 触发！{time_since_last*1000:.0f}ms 未收到指令，执行紧急停止")
                    self.watchdog_trigger_count += 1
                    watchdog_active = True
                
                vx, vy, wz = 0.0, 0.0, 0.0
                
                with self.cmd_lock:
                    self.current_vx = 0.0
                    self.current_vy = 0.0
                    self.current_wz = 0.0
            else:
                if watchdog_active:
                    print("✅ 通信恢复，继续执行指令")
                    watchdog_active = False
            
            # ============ 发送指令 ============
            if self.loco is not None:
                try:
                    self.loco.Move(vx, vy, wz)
                except Exception as e:
                    print(f"❌ 发送指令失败: {e}")
            elif self.test_mode:
                # 测试模式：每秒输出一次当前状态
                if now - last_log_time > 1.0:
                    status = "🚨 Watchdog" if watchdog_active else "✅ 正常"
                    print(f"[TEST] 状态: {status}, 指令: vx={vx:.2f}, vy={vy:.2f}, wz={wz:.2f}")
                    last_log_time = now
            
            time.sleep(control_interval)
        
        # 退出前确保停止
        if self.loco is not None:
            for _ in range(20):
                self.loco.Move(0.0, 0.0, 0.0)
                time.sleep(0.01)
        
        print("🛑 控制循环停止")
    
    def start(self):
        """启动接收器"""
        if self.running:
            print("⚠️ 接收器已经在运行")
            return
        
        self.running = True
        self.last_recv_time = 0  # 重置
        
        # 启动线程
        self.recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
        self.control_thread = threading.Thread(target=self._control_loop, daemon=True)
        
        self.recv_thread.start()
        self.control_thread.start()
        
        print("=" * 50)
        print("  🤖 Robot Sink 已启动")
        print(f"  📡 UDP端口: {self.port}")
        print(f"  🐕 Watchdog超时: {CONFIG['WATCHDOG_TIMEOUT_MS']}ms")
        print("=" * 50)
        print("\n等待上位机连接...")
        print("按 Ctrl+C 停止\n")
    
    def stop(self):
        """停止接收器"""
        print("\n正在停止...")
        self.running = False
        
        if hasattr(self, 'recv_thread') and self.recv_thread.is_alive():
            self.recv_thread.join(timeout=2.0)
        if hasattr(self, 'control_thread') and self.control_thread.is_alive():
            self.control_thread.join(timeout=2.0)
        
        print(f"📊 统计: 收到 {self.packet_count} 个包, "
              f"Watchdog 触发 {self.watchdog_trigger_count} 次")
        print("✅ Robot Sink 已停止")
    
    def wait(self):
        """阻塞等待直到收到停止信号"""
        try:
            while self.running:
                time.sleep(0.1)
        except KeyboardInterrupt:
            self.stop()


def main():
    parser = argparse.ArgumentParser(description="机器人端 UDP 接收程序")
    parser.add_argument("--port", type=int, default=CONFIG["UDP_PORT"],
                        help=f"UDP 监听端口 (默认: {CONFIG['UDP_PORT']})")
    parser.add_argument("--timeout", type=int, default=CONFIG["WATCHDOG_TIMEOUT_MS"],
                        help=f"Watchdog 超时 (毫秒, 默认: {CONFIG['WATCHDOG_TIMEOUT_MS']})")
    parser.add_argument("--test-mode", action="store_true",
                        help="测试模式 (不连接真实SDK)")
    args = parser.parse_args()
    
    CONFIG["UDP_PORT"] = args.port
    CONFIG["WATCHDOG_TIMEOUT_MS"] = args.timeout
    
    print("=" * 60)
    print("  🤖 多码融合电子围栏系统 - 机器人端接收程序")
    print("=" * 60)
    
    sink = RobotSink(
        port=args.port,
        watchdog_timeout_ms=args.timeout,
        test_mode=args.test_mode
    )
    
    sink.start()
    sink.wait()


if __name__ == "__main__":
    main()
