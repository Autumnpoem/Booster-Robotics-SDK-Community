#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Booster Robotics SDK 传感器监控程序
====================================
功能：实时读取并显示 IMU、里程计、电机状态等数据
作者：He
日期：2025-12-21
"""

import sys
import time
import threading
import argparse
from datetime import datetime

# 导入 SDK
try:
    from booster_robotics_sdk_python import (
        ChannelFactory,
        B1LowStateSubscriber,
        B1OdometerStateSubscriber,
        B1LocoClient,
        GetModeResponse,
        RobotMode
    )
    SDK_AVAILABLE = True
except ImportError:
    print("❌ Error: booster_robotics_sdk_python not found!")
    print("   Please install the SDK first.")
    SDK_AVAILABLE = False

# ==================== 配置参数 ====================
CONFIG = {
    "NETWORK_INTERFACE": "127.0.0.1",  # 网络接口或 IP
    "DOMAIN_ID": 0,                     # DDS Domain ID
    "PRINT_INTERVAL": 0.1,              # 打印间隔（秒），可通过命令行参数调整
    "CLEAR_SCREEN": True,               # 是否清屏刷新
}

# ==================== 全局数据存储 ====================
class SensorData:
    """传感器数据存储类"""
    def __init__(self):
        # IMU 数据
        self.imu_rpy = [0.0, 0.0, 0.0]      # Roll, Pitch, Yaw
        self.imu_gyro = [0.0, 0.0, 0.0]     # 角速度
        self.imu_acc = [0.0, 0.0, 0.0]      # 加速度
        
        # 里程计数据
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_theta = 0.0
        
        # 上次里程计时间（用于计算速度）
        self.last_odom_time = time.time()
        
        # 计算得到的速度（滤波后）
        self.velocity_x = 0.0
        self.velocity_y = 0.0
        self.velocity_theta = 0.0
        
        # 电机状态
        self.motor_count_serial = 0
        self.motor_count_parallel = 0
        self.motor_states_serial = []
        self.motor_states_parallel = []
        
        # 时间戳
        self.imu_timestamp = 0.0
        self.odom_timestamp = 0.0
        
        # 更新计数
        self.imu_update_count = 0
        self.odom_update_count = 0
        
        # 锁
        self.lock = threading.Lock()

# 全局传感器数据实例
sensor_data = SensorData()

# ==================== 回调函数 ====================
def low_state_handler(msg):
    """低级状态回调（IMU + 电机状态）"""
    global sensor_data
    with sensor_data.lock:
        # 更新 IMU 数据
        imu = msg.imu_state
        sensor_data.imu_rpy = [imu.rpy[0], imu.rpy[1], imu.rpy[2]]
        sensor_data.imu_gyro = [imu.gyro[0], imu.gyro[1], imu.gyro[2]]
        sensor_data.imu_acc = [imu.acc[0], imu.acc[1], imu.acc[2]]
        
        # 更新电机状态
        sensor_data.motor_count_serial = len(msg.motor_state_serial)
        sensor_data.motor_count_parallel = len(msg.motor_state_parallel)
        
        # 保存部分电机状态（前6个）
        sensor_data.motor_states_serial = []
        for i, motor in enumerate(msg.motor_state_serial[:6]):
            sensor_data.motor_states_serial.append({
                'index': i,
                'q': motor.q,
                'dq': motor.dq,
                'tau': motor.tau_est
            })
        
        sensor_data.motor_states_parallel = []
        for i, motor in enumerate(msg.motor_state_parallel[:6]):
            sensor_data.motor_states_parallel.append({
                'index': i,
                'q': motor.q,
                'dq': motor.dq,
                'tau': motor.tau_est
            })
        
        sensor_data.imu_timestamp = time.time()
        sensor_data.imu_update_count += 1


def odometer_handler(msg):
    """里程计回调"""
    global sensor_data
    with sensor_data.lock:
        current_time = time.time()
        dt = current_time - sensor_data.last_odom_time
        
        # 计算速度（通过差分）
        # 注意: 使用 msg.x - odom_x (上一次的值) 来计算
        if dt > 0.001:  # 避免除以零
            raw_vx = (msg.x - sensor_data.odom_x) / dt
            raw_vy = (msg.y - sensor_data.odom_y) / dt
            raw_vtheta = (msg.theta - sensor_data.odom_theta) / dt
            
            # 应用低通滤波平滑速度 (防止噪声导致的突变)
            alpha = 0.3  # 滤波系数，越小越平滑
            sensor_data.velocity_x = alpha * raw_vx + (1 - alpha) * sensor_data.velocity_x
            sensor_data.velocity_y = alpha * raw_vy + (1 - alpha) * sensor_data.velocity_y
            sensor_data.velocity_theta = alpha * raw_vtheta + (1 - alpha) * sensor_data.velocity_theta
            
            # 应用死区阈值 (低于此值视为0，消除站立时的漂移)
            VELOCITY_DEADZONE = 0.05  # m/s
            ANGULAR_DEADZONE = 0.02   # rad/s
            if abs(sensor_data.velocity_x) < VELOCITY_DEADZONE:
                sensor_data.velocity_x = 0.0
            if abs(sensor_data.velocity_y) < VELOCITY_DEADZONE:
                sensor_data.velocity_y = 0.0
            if abs(sensor_data.velocity_theta) < ANGULAR_DEADZONE:
                sensor_data.velocity_theta = 0.0
        
        # 更新时间戳
        sensor_data.last_odom_time = current_time
        
        # 更新里程计数据 (作为下一次计算的"上一次值")
        sensor_data.odom_x = msg.x
        sensor_data.odom_y = msg.y
        sensor_data.odom_theta = msg.theta
        
        sensor_data.odom_timestamp = time.time()
        sensor_data.odom_update_count += 1


# ==================== 显示函数 ====================
def clear_screen():
    """清屏"""
    print("\033[H\033[J", end="")


def print_separator(char="=", length=60):
    """打印分隔线"""
    print(char * length)


def print_header():
    """打印头部信息"""
    print_separator()
    print("  🤖 Booster Robotics SDK 传感器监控程序")
    print(f"  📅 时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print_separator()


def print_imu_data():
    """打印 IMU 数据"""
    global sensor_data
    with sensor_data.lock:
        rpy = sensor_data.imu_rpy
        gyro = sensor_data.imu_gyro
        acc = sensor_data.imu_acc
        count = sensor_data.imu_update_count
    
    print("\n📐 IMU / 陀螺仪数据")
    print("-" * 50)
    print(f"  姿态角 (RPY):")
    print(f"    Roll  (横滚): {rpy[0]:>10.4f} rad  ({rpy[0]*57.3:>8.2f}°)")
    print(f"    Pitch (俯仰): {rpy[1]:>10.4f} rad  ({rpy[1]*57.3:>8.2f}°)")
    print(f"    Yaw   (偏航): {rpy[2]:>10.4f} rad  ({rpy[2]*57.3:>8.2f}°)")
    print()
    print(f"  角速度 (Gyro):")
    print(f"    X: {gyro[0]:>10.4f} rad/s")
    print(f"    Y: {gyro[1]:>10.4f} rad/s")
    print(f"    Z: {gyro[2]:>10.4f} rad/s")
    print()
    print(f"  加速度 (Acc):")
    print(f"    X: {acc[0]:>10.4f} m/s²")
    print(f"    Y: {acc[1]:>10.4f} m/s²")
    print(f"    Z: {acc[2]:>10.4f} m/s²")
    print(f"\n  更新次数: {count}")


def print_odometer_data():
    """打印里程计数据"""
    global sensor_data
    with sensor_data.lock:
        x = sensor_data.odom_x
        y = sensor_data.odom_y
        theta = sensor_data.odom_theta
        vx = sensor_data.velocity_x
        vy = sensor_data.velocity_y
        vtheta = sensor_data.velocity_theta
        count = sensor_data.odom_update_count
    
    print("\n📍 里程计数据")
    print("-" * 50)
    print(f"  位置:")
    print(f"    X:     {x:>10.4f} m")
    print(f"    Y:     {y:>10.4f} m")
    print(f"    Theta: {theta:>10.4f} rad  ({theta*57.3:>8.2f}°)")
    print()
    print(f"  速度 (估算):")
    print(f"    Vx:     {vx:>10.4f} m/s")
    print(f"    Vy:     {vy:>10.4f} m/s")
    print(f"    Vtheta: {vtheta:>10.4f} rad/s")
    print(f"\n  更新次数: {count}")


def print_motor_data():
    """打印电机状态"""
    global sensor_data
    with sensor_data.lock:
        serial_count = sensor_data.motor_count_serial
        parallel_count = sensor_data.motor_count_parallel
        serial_motors = sensor_data.motor_states_serial.copy()
        parallel_motors = sensor_data.motor_states_parallel.copy()
    
    print("\n⚙️  电机状态")
    print("-" * 50)
    print(f"  串行电机数量: {serial_count}")
    print(f"  并行电机数量: {parallel_count}")
    
    if serial_motors:
        print("\n  串行电机 (前6个):")
        print(f"  {'Index':<6} {'位置(q)':<12} {'速度(dq)':<12} {'力矩(τ)':<12}")
        for m in serial_motors:
            print(f"    {m['index']:<4} {m['q']:>10.4f}  {m['dq']:>10.4f}  {m['tau']:>10.4f}")
    
    if parallel_motors:
        print("\n  并行电机 (前6个):")
        print(f"  {'Index':<6} {'位置(q)':<12} {'速度(dq)':<12} {'力矩(τ)':<12}")
        for m in parallel_motors:
            print(f"    {m['index']:<4} {m['q']:>10.4f}  {m['dq']:>10.4f}  {m['tau']:>10.4f}")


def print_status():
    """打印状态信息"""
    global sensor_data
    with sensor_data.lock:
        imu_time = sensor_data.imu_timestamp
        odom_time = sensor_data.odom_timestamp
    
    current = time.time()
    imu_age = current - imu_time if imu_time > 0 else -1
    odom_age = current - odom_time if odom_time > 0 else -1
    
    print("\n📊 数据状态")
    print("-" * 50)
    
    imu_status = "🟢 正常" if 0 <= imu_age < 1.0 else "🔴 超时" if imu_age >= 0 else "⚪ 无数据"
    odom_status = "🟢 正常" if 0 <= odom_age < 1.0 else "🔴 超时" if odom_age >= 0 else "⚪ 无数据"
    
    print(f"  IMU 状态:     {imu_status}  (最后更新: {imu_age:.2f}s 前)" if imu_age >= 0 else f"  IMU 状态:     {imu_status}")
    print(f"  里程计状态:   {odom_status}  (最后更新: {odom_age:.2f}s 前)" if odom_age >= 0 else f"  里程计状态:   {odom_status}")


def print_footer(interval):
    """打印底部信息"""
    print_separator()
    print(f"  刷新间隔: {interval}s | 按 Ctrl+C 退出")
    print_separator()


def display_loop(interval, clear=True):
    """显示循环"""
    while True:
        try:
            if clear:
                clear_screen()
            
            print_header()
            print_imu_data()
            print_odometer_data()
            print_motor_data()
            print_status()
            print_footer(interval)
            
            time.sleep(interval)
            
        except KeyboardInterrupt:
            print("\n\n👋 程序已退出")
            break


# ==================== 主函数 ====================
def main():
    parser = argparse.ArgumentParser(
        description="Booster Robotics SDK 传感器监控程序",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  python sensor_monitor.py                    # 默认参数运行
  python sensor_monitor.py -i 0.1             # 0.1秒刷新间隔
  python sensor_monitor.py -i 1.0 --no-clear  # 1秒间隔，不清屏
  python sensor_monitor.py --network eth0     # 指定网络接口
        """
    )
    parser.add_argument(
        "-i", "--interval",
        type=float,
        default=CONFIG["PRINT_INTERVAL"],
        help=f"打印间隔（秒），默认: {CONFIG['PRINT_INTERVAL']}"
    )
    parser.add_argument(
        "--no-clear",
        action="store_true",
        help="不清屏，追加打印（适合日志记录）"
    )
    parser.add_argument(
        "--network",
        type=str,
        default=CONFIG["NETWORK_INTERFACE"],
        help=f"网络接口或IP，默认: {CONFIG['NETWORK_INTERFACE']}"
    )
    parser.add_argument(
        "--domain",
        type=int,
        default=CONFIG["DOMAIN_ID"],
        help=f"DDS Domain ID，默认: {CONFIG['DOMAIN_ID']}"
    )
    
    args = parser.parse_args(['-i', '1', '--network', '127.0.0.1'])    
    if not SDK_AVAILABLE:
        print("SDK 不可用，程序退出")
        sys.exit(1)
    
    print("=" * 60)
    print("  🚀 Booster Robotics SDK 传感器监控程序")
    print("=" * 60)
    print(f"  网络接口: {args.network}")
    print(f"  Domain ID: {args.domain}")
    print(f"  刷新间隔: {args.interval}s")
    print(f"  清屏模式: {'关闭' if args.no_clear else '开启'}")
    print("=" * 60)
    print("\n正在初始化...")
    
    try:
        # 初始化 Channel Factory
        ChannelFactory.Instance().Init(args.domain, args.network)
        print("✓ ChannelFactory 初始化成功")
        
        # 创建订阅者
        low_state_sub = B1LowStateSubscriber(low_state_handler)
        low_state_sub.InitChannel()
        print("✓ LowState 订阅者初始化成功")
        
        odom_sub = B1OdometerStateSubscriber(odometer_handler)
        odom_sub.InitChannel()
        print("✓ Odometer 订阅者初始化成功")
        
        print("\n初始化完成！等待数据...")
        time.sleep(1)
        
        # 开始显示循环
        display_loop(args.interval, clear=not args.no_clear)
        
    except Exception as e:
        print(f"\n❌ 初始化失败: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
    finally:
        print("\n正在清理资源...")
        try:
            low_state_sub.CloseChannel()
            odom_sub.CloseChannel()
            print("✓ 资源清理完成")
        except:
            pass


if __name__ == "__main__":
    main()
