#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Booster Robotics SDK 手柄控制自动模式切换程序
============================================
功能：通过手柄组合键（LT + LB + 十字上）切换自动/手动模式
组合键：同时按下 左扳机(LT) + 左肩键(LB) + 十字键上(hat_u)
作者：Antigravity Assistant
日期：2024-12-21
"""

import sys
import time
import threading
from datetime import datetime

# ==================== 在这里修改配置参数 ====================
CONFIG = {
    # 网络配置
    "NETWORK_INTERFACE": "127.0.0.1",  # ← 修改网络接口或 IP
    "DOMAIN_ID": 0,                     # ← 修改 DDS Domain ID
    
    # 控制频率
    "CONTROL_HZ": 20,                   # 控制频率 (Hz)
    
    # 自动模式运动参数 - 在这里直接修改速度
    "AUTO_VX": 0.3,       # ← 自动模式前进速度 (m/s)
    "AUTO_VY": 0.0,       # ← 自动模式侧向速度 (m/s)
    "AUTO_VYAW": 0.3,     # ← 自动模式转向速度 (rad/s)
    
    # 组合键防抖时间（秒）
    "COMBO_DEBOUNCE": 0.5,
    
    # 是否使用键盘控制模式（True=键盘, False=手柄）
    "USE_KEYBOARD": False,  # ← 如果手柄不可用，改为 True
}
# =============================================================

# 导入 SDK
try:
    from booster_robotics_sdk_python import (
        ChannelFactory,
        B1LowStateSubscriber,
        B1OdometerStateSubscriber,
        B1LocoClient,
        RobotMode,
    )
    SDK_AVAILABLE = True
except ImportError:
    print("❌ Error: booster_robotics_sdk_python not found!")
    print("   Please install the SDK first.")
    SDK_AVAILABLE = False

# 尝试导入 RemoteControllerState 订阅者
try:
    from booster_robotics_sdk_python import B1RemoteControllerStateSubscriber
    RC_SUBSCRIBER_AVAILABLE = True
except ImportError:
    RC_SUBSCRIBER_AVAILABLE = False
    print("⚠️ Warning: B1RemoteControllerStateSubscriber not available")
    print("   Will use keyboard fallback mode")


# ==================== 全局状态 ====================
class ControllerState:
    """手柄状态存储类"""
    def __init__(self):
        # 手柄按键状态
        self.lt = False      # 左扳机
        self.lb = False      # 左肩键
        self.rb = False      # 右肩键
        self.hat_u = False   # 十字键上
        
        # 摇杆状态
        self.lx = 0.0
        self.ly = 0.0
        self.rx = 0.0
        self.ry = 0.0
        
        # 其他按键
        self.a = False
        self.b = False
        self.x = False
        self.y = False
        
        # 状态更新时间
        self.last_update = time.time()
        
        # 锁
        self.lock = threading.Lock()


class AutoModeController:
    """自动模式控制器"""
    def __init__(self):
        self.auto_mode_enabled = False      # 自动模式开关
        self.last_combo_time = 0            # 上次组合键触发时间
        self.combo_debounce = CONFIG["COMBO_DEBOUNCE"]
        
        # 运动参数 - 直接从 CONFIG 读取
        self.vx = CONFIG["AUTO_VX"]
        self.vy = CONFIG["AUTO_VY"]
        self.vyaw = CONFIG["AUTO_VYAW"]
        
        # 锁
        self.lock = threading.Lock()
    
    def check_combo_key(self, controller: ControllerState) -> bool:
        """
        检查组合键是否触发
        组合键: LT + LB + 十字上 (hat_u)
        返回: 是否触发了模式切换
        """
        with controller.lock:
            lt = controller.lt
            lb = controller.lb
            hat_u = controller.hat_u
        
        current_time = time.time()
        
        # 检查组合键
        if lt and lb and hat_u:
            # 防抖：确保距离上次触发超过防抖时间
            if current_time - self.last_combo_time > self.combo_debounce:
                self.last_combo_time = current_time
                return True
        
        return False
    
    def toggle_auto_mode(self):
        """切换自动模式"""
        with self.lock:
            self.auto_mode_enabled = not self.auto_mode_enabled
            return self.auto_mode_enabled
    
    def is_auto_enabled(self) -> bool:
        """获取自动模式状态"""
        with self.lock:
            return self.auto_mode_enabled


# 全局实例
controller_state = ControllerState()
auto_controller = AutoModeController()


# ==================== 回调函数 ====================
def remote_controller_handler(msg):
    """手柄状态回调"""
    global controller_state
    with controller_state.lock:
        # 更新按键状态
        controller_state.lt = msg.lt
        controller_state.lb = msg.lb
        controller_state.rb = msg.rb
        controller_state.hat_u = msg.hat_u
        
        # 更新摇杆状态
        controller_state.lx = msg.lx
        controller_state.ly = msg.ly
        controller_state.rx = msg.rx
        controller_state.ry = msg.ry
        
        # 更新其他按键
        controller_state.a = msg.a
        controller_state.b = msg.b
        controller_state.x = msg.x
        controller_state.y = msg.y
        
        controller_state.last_update = time.time()


# ==================== 主控制循环 ====================
def control_loop(client: 'B1LocoClient'):
    """主控制循环 - 手柄模式"""
    global controller_state, auto_controller
    
    control_interval = 1.0 / CONFIG["CONTROL_HZ"]
    
    print("\n" + "=" * 60)
    print("  🎮 手柄自动模式控制已启动")
    print("=" * 60)
    print(f"  组合键: LT + LB + 十字上 → 切换自动/手动模式")
    print(f"  自动模式速度: vx={CONFIG['AUTO_VX']}, vy={CONFIG['AUTO_VY']}, vyaw={CONFIG['AUTO_VYAW']}")
    print(f"  控制频率: {CONFIG['CONTROL_HZ']} Hz")
    print("=" * 60)
    print("\n⏳ 等待手柄输入...")
    
    last_status_print = 0
    
    while True:
        try:
            current_time = time.time()
            
            # 检查组合键
            if auto_controller.check_combo_key(controller_state):
                new_state = auto_controller.toggle_auto_mode()
                status = "🟢 自动模式已启动" if new_state else "🔴 自动模式已停止"
                print(f"\n{datetime.now().strftime('%H:%M:%S')} {status}")
                
                # 如果关闭自动模式，立即停止运动
                if not new_state:
                    client.Move(0.0, 0.0, 0.0)
                    print("  ⏹️  机器人已停止")
            
            # 如果自动模式启用，持续发送运动指令
            if auto_controller.is_auto_enabled():
                client.Move(
                    auto_controller.vx,
                    auto_controller.vy,
                    auto_controller.vyaw
                )
            
            # 每3秒打印一次状态
            if current_time - last_status_print > 3.0:
                with controller_state.lock:
                    data_age = current_time - controller_state.last_update
                    lt = controller_state.lt
                    lb = controller_state.lb
                    hat_u = controller_state.hat_u
                
                mode_str = "🟢 自动" if auto_controller.is_auto_enabled() else "🔴 手动"
                data_status = "✓" if data_age < 1.0 else "✗"
                combo_str = f"LT:{lt} LB:{lb} ↑:{hat_u}"
                
                print(f"\r[{datetime.now().strftime('%H:%M:%S')}] 模式: {mode_str} | 数据: {data_status} | {combo_str}    ", end="", flush=True)
                last_status_print = current_time
            
            time.sleep(control_interval)
            
        except KeyboardInterrupt:
            print("\n\n👋 正在退出...")
            client.Move(0.0, 0.0, 0.0)
            break


# ==================== 键盘输入备用方案 ====================
def keyboard_input_fallback(client: 'B1LocoClient'):
    """
    当手柄订阅不可用时的键盘备用方案
    按 't' 切换自动模式
    """
    global auto_controller
    
    print("\n" + "=" * 60)
    print("  ⌨️  键盘控制模式")
    print("=" * 60)
    print("  按键说明:")
    print("    t   → 切换自动/手动模式")
    print("    s   → 停止运动")
    print("    q   → 退出程序")
    print(f"  自动模式速度: vx={CONFIG['AUTO_VX']}, vy={CONFIG['AUTO_VY']}, vyaw={CONFIG['AUTO_VYAW']}")
    print("=" * 60)
    
    control_interval = 1.0 / CONFIG["CONTROL_HZ"]
    
    # 使用非阻塞输入
    import select
    
    while True:
        try:
            # 如果自动模式启用，持续发送运动指令
            if auto_controller.is_auto_enabled():
                client.Move(
                    auto_controller.vx,
                    auto_controller.vy,
                    auto_controller.vyaw
                )
            
            # 检查键盘输入（非阻塞）
            if select.select([sys.stdin], [], [], 0.0)[0]:
                key = sys.stdin.readline().strip().lower()
                
                if key == 't':
                    new_state = auto_controller.toggle_auto_mode()
                    status = "🟢 自动模式已启动" if new_state else "🔴 自动模式已停止"
                    print(f"\n{datetime.now().strftime('%H:%M:%S')} {status}")
                    if not new_state:
                        client.Move(0.0, 0.0, 0.0)
                        print("  ⏹️  机器人已停止")
                
                elif key == 's':
                    client.Move(0.0, 0.0, 0.0)
                    print("⏹️  运动已停止")
                
                elif key == 'q':
                    print("\n👋 退出程序...")
                    client.Move(0.0, 0.0, 0.0)
                    break
            
            time.sleep(control_interval)
            
        except KeyboardInterrupt:
            print("\n\n👋 正在退出...")
            client.Move(0.0, 0.0, 0.0)
            break


# ==================== 主函数 ====================
def main():
    if not SDK_AVAILABLE:
        print("SDK 不可用，程序退出")
        sys.exit(1)
    
    print("=" * 60)
    print("  🤖 Booster Robotics 手柄自动模式控制程序")
    print("=" * 60)
    print(f"  网络接口: {CONFIG['NETWORK_INTERFACE']}")
    print(f"  Domain ID: {CONFIG['DOMAIN_ID']}")
    print(f"  自动模式速度: vx={CONFIG['AUTO_VX']}, vy={CONFIG['AUTO_VY']}, vyaw={CONFIG['AUTO_VYAW']}")
    print(f"  控制模式: {'键盘' if CONFIG['USE_KEYBOARD'] else '手柄'}")
    print("=" * 60)
    print("\n正在初始化...")
    
    try:
        # 初始化 Channel Factory
        ChannelFactory.Instance().Init(CONFIG["DOMAIN_ID"], CONFIG["NETWORK_INTERFACE"])
        print("✓ ChannelFactory 初始化成功")
        
        # 创建 LocoClient
        client = B1LocoClient()
        client.Init()
        print("✓ B1LocoClient 初始化成功")
        
        # 切换到行走模式
        print("正在切换到行走模式...")
        res = client.ChangeMode(RobotMode.kWalking)
        if res != 0:
            print(f"⚠️ 模式切换返回: {res}")
        else:
            print("✓ 已切换到行走模式")
        
        time.sleep(1)
        
        # 根据配置选择控制模式
        if CONFIG["USE_KEYBOARD"]:
            print("使用键盘控制模式...")
            keyboard_input_fallback(client)
        elif RC_SUBSCRIBER_AVAILABLE:
            print("正在初始化手柄状态订阅...")
            try:
                rc_sub = B1RemoteControllerStateSubscriber(remote_controller_handler)
                rc_sub.InitChannel()
                print("✓ RemoteController 订阅者初始化成功")
                
                # 进入手柄控制循环
                control_loop(client)
                
            except Exception as e:
                print(f"⚠️ 手柄订阅初始化失败: {e}")
                print("切换到键盘控制模式...")
                keyboard_input_fallback(client)
        else:
            print("手柄订阅不可用，使用键盘控制模式...")
            keyboard_input_fallback(client)
        
    except Exception as e:
        print(f"\n❌ 初始化失败: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
    finally:
        print("\n正在清理资源...")
        try:
            client.Move(0.0, 0.0, 0.0)
            print("✓ 资源清理完成")
        except:
            pass


if __name__ == "__main__":
    main()
