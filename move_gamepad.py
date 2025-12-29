#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
虚拟手柄控制程序 - 使用 evdev.UInput 创建虚拟手柄

安装:
    pip3 install evdev

运行:
    sudo python3 move_gamepad.py

控制方式:
    m → 切换自动/手动模式
    l → 左圈模式
    r → 右圈模式
    s → 直走模式
    q 或 Ctrl+C → 停止退出

作者: He
日期: 2025-12-28
"""

import time
import sys
import threading
import select
import signal
import os

# 信号处理
def signal_handler(signum, frame):
    print("\n🛑 收到退出信号，强制退出...")
    os._exit(0)

signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

# evdev
try:
    from evdev import UInput, ecodes, AbsInfo
    EVDEV_AVAILABLE = True
except ImportError:
    EVDEV_AVAILABLE = False
    print("❌ evdev 未安装")
    print("安装: pip3 install evdev")
    sys.exit(1)

# 终端控制
try:
    import termios
    import tty
    TERMIOS_AVAILABLE = True
except ImportError:
    TERMIOS_AVAILABLE = False


# =================== 配置 ===================
CONFIG = {
    # 摇杆值 (0-255, 128=中间)
    "CENTER": 128,
    "FULL_FORWARD": 0,       # ly=0 表示完全向前推
    "FULL_BACKWARD": 255,
    "LEFT_TURN": 200,        # rx 用于转向
    "RIGHT_TURN": 56,
    
    # 更新频率
    "UPDATE_HZ": 50,
    
    # 持续时间 (秒)
    "CIRCLE_DURATION": 5000.0,
    
    # 初始模式
    "WALK_MODE": "left_circle",
}


class KeyboardController:
    def __init__(self):
        self.old_settings = None
    
    def setup(self):
        if TERMIOS_AVAILABLE:
            try:
                self.old_settings = termios.tcgetattr(sys.stdin)
                tty.setcbreak(sys.stdin.fileno())
            except:
                pass
    
    def restore(self):
        if self.old_settings and TERMIOS_AVAILABLE:
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
            except:
                pass
    
    def check_key(self):
        if not TERMIOS_AVAILABLE:
            return None
        try:
            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                return sys.stdin.read(1).lower()
        except:
            pass
        return None


class VirtualGamepad:
    """虚拟手柄 - 使用 evdev.UInput"""
    
    def __init__(self):
        print("🎮 虚拟手柄初始化 (evdev)...")
        
        self.running = True
        self.is_manual_mode = False
        
        # 摇杆值 (0-255)
        self.lx = CONFIG["CENTER"]
        self.ly = CONFIG["CENTER"]
        self.rx = CONFIG["CENTER"]
        self.ry = CONFIG["CENTER"]
        
        # 方向
        self.circle_direction = "left"
        if CONFIG["WALK_MODE"] == "right_circle":
            self.circle_direction = "right"
        elif CONFIG["WALK_MODE"] == "straight":
            self.circle_direction = "straight"
        
        self.keyboard = KeyboardController()
        self._init_virtual_gamepad()
        
        print("✅ 虚拟手柄就绪")
        self._print_help()
    
    def _init_virtual_gamepad(self):
        """创建虚拟手柄设备"""
        try:
            # 定义手柄的轴
            cap = {
                ecodes.EV_ABS: [
                    (ecodes.ABS_X, AbsInfo(value=128, min=0, max=255, fuzz=0, flat=0, resolution=0)),
                    (ecodes.ABS_Y, AbsInfo(value=128, min=0, max=255, fuzz=0, flat=0, resolution=0)),
                    (ecodes.ABS_RX, AbsInfo(value=128, min=0, max=255, fuzz=0, flat=0, resolution=0)),
                    (ecodes.ABS_RY, AbsInfo(value=128, min=0, max=255, fuzz=0, flat=0, resolution=0)),
                    (ecodes.ABS_Z, AbsInfo(value=0, min=0, max=255, fuzz=0, flat=0, resolution=0)),   # LT
                    (ecodes.ABS_RZ, AbsInfo(value=0, min=0, max=255, fuzz=0, flat=0, resolution=0)),  # RT
                    (ecodes.ABS_HAT0X, AbsInfo(value=0, min=-1, max=1, fuzz=0, flat=0, resolution=0)),
                    (ecodes.ABS_HAT0Y, AbsInfo(value=0, min=-1, max=1, fuzz=0, flat=0, resolution=0)),
                ],
                ecodes.EV_KEY: [
                    ecodes.BTN_A, ecodes.BTN_B, ecodes.BTN_X, ecodes.BTN_Y,
                    ecodes.BTN_TL, ecodes.BTN_TR,  # LB, RB
                    ecodes.BTN_SELECT, ecodes.BTN_START,
                    ecodes.BTN_THUMBL, ecodes.BTN_THUMBR,  # LS, RS
                ],
            }
            
            self.device = UInput(cap, name="Virtual Gamepad", vendor=0x045e, product=0x028e)
            print(f"✓ 虚拟手柄设备已创建")
            time.sleep(1)  # 等待设备注册
            
            # 发送初始值
            self._sync()
            
        except PermissionError:
            print("❌ 权限不足，请使用 sudo 运行")
            sys.exit(1)
        except Exception as e:
            print(f"❌ 创建虚拟手柄失败: {e}")
            sys.exit(1)
    
    def _sync(self):
        """更新并同步虚拟手柄状态"""
        self.device.write(ecodes.EV_ABS, ecodes.ABS_X, self.lx)
        self.device.write(ecodes.EV_ABS, ecodes.ABS_Y, self.ly)
        self.device.write(ecodes.EV_ABS, ecodes.ABS_RX, self.rx)
        self.device.write(ecodes.EV_ABS, ecodes.ABS_RY, self.ry)
        self.device.syn()
    
    def _print_help(self):
        print()
        print("=" * 60)
        print("  控制方式:")
        print("  m → 切换自动/手动模式")
        print("  l → 左圈模式")
        print("  r → 右圈模式")
        print("  s → 直走模式")
        print("  q 或 Ctrl+C → 停止退出")
        print("=" * 60)
        print()
    
    def _update_loop(self):
        """更新循环"""
        interval = 1.0 / CONFIG["UPDATE_HZ"]
        last_print = 0
        
        while self.running:
            try:
                self._sync()
                
                now = time.time()
                if now - last_print > 2.0 and not self.is_manual_mode:
                    print(f"[发送] ly={self.ly}, rx={self.rx} | {CONFIG['UPDATE_HZ']}Hz")
                    last_print = now
                
                time.sleep(interval)
            except Exception as e:
                print(f"\n❌ 更新错误: {e}")
                time.sleep(0.1)
    
    def _command_sequence(self):
        """运动序列"""
        count = 0
        while self.running:
            if self.is_manual_mode:
                time.sleep(0.1)
                continue
            
            count += 1
            d = self.circle_direction
            
            if d == "left":
                self.ly = CONFIG["FULL_FORWARD"]
                self.rx = CONFIG["LEFT_TURN"]
                name = "左圈"
            elif d == "right":
                self.ly = CONFIG["FULL_FORWARD"]
                self.rx = CONFIG["RIGHT_TURN"]
                name = "右圈"
            else:
                self.ly = CONFIG["FULL_FORWARD"]
                self.rx = CONFIG["CENTER"]
                name = "直走"
            
            self.lx = CONFIG["CENTER"]
            print(f"\n=== 第 {count} 个{name}: ly={self.ly}, rx={self.rx} ===")
            
            start = time.time()
            while self.running and (time.time() - start) < CONFIG["CIRCLE_DURATION"]:
                if self.is_manual_mode:
                    break
                time.sleep(0.1)
            
            if not self.running:
                break
            
            self._stop()
            print(f"=== 第 {count} 个{name}完成 ===")
            time.sleep(2.0)
    
    def _keyboard_loop(self):
        self.keyboard.setup()
        try:
            while self.running:
                key = self.keyboard.check_key()
                if key == 'q':
                    print("\n⚠️ 'q' 键，退出...")
                    self._emergency_stop()
                    break
                elif key == 'm':
                    self.is_manual_mode = not self.is_manual_mode
                    print(f"\n[键盘] m: {'手动' if self.is_manual_mode else '自动'}模式")
                    if self.is_manual_mode:
                        self._stop()
                elif key == 'l':
                    self.circle_direction = "left"
                    print("\n[键盘] l: 左圈")
                elif key == 'r':
                    self.circle_direction = "right"
                    print("\n[键盘] r: 右圈")
                elif key == 's':
                    self.circle_direction = "straight"
                    print("\n[键盘] s: 直走")
                time.sleep(0.05)
        finally:
            self.keyboard.restore()
    
    def _stop(self):
        self.lx = CONFIG["CENTER"]
        self.ly = CONFIG["CENTER"]
        self.rx = CONFIG["CENTER"]
        self.ry = CONFIG["CENTER"]
    
    def _emergency_stop(self):
        self.running = False
        self._stop()
        self._sync()
        print("🛑 已停止")
    
    def run(self):
        threads = [
            threading.Thread(target=self._update_loop, daemon=True),
            threading.Thread(target=self._command_sequence, daemon=True),
            threading.Thread(target=self._keyboard_loop, daemon=True),
        ]
        for t in threads:
            t.start()
        
        try:
            while self.running:
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\n⚠️ Ctrl+C，停止...")
            self._emergency_stop()
        finally:
            self.running = False
            self.keyboard.restore()
            if hasattr(self, 'device'):
                self.device.close()
            print("\n👋 已退出")


def main():
    print()
    print("=" * 60)
    print("  🎮 虚拟手柄控制程序 (evdev)")
    print("  创建虚拟手柄设备模拟真实手柄")
    print("=" * 60)
    print()
    
    gamepad = VirtualGamepad()
    gamepad.run()


if __name__ == "__main__":
    main()
