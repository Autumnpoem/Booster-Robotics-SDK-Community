#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
快速180度转身程序 - 使用陀螺仪闭环控制 (带延迟补偿)
=====================================================
功能：使用IMU陀螺仪实时检测角度，实现快速且精确的180度转身
原理：
  1. 记录起始航向角
  2. 给一个较大的转向速度 + 小的前进速度(VX=0.1)保持稳定
  3. 实时读取陀螺仪累计角度
  4. 使用角速度预测延迟时间内的角度变化，提前停止
  5. 当预测角度接近180度时提前发出停止指令

关键：延迟补偿
  - IMU数据有传输延迟(约50-100ms)
  - 使用当前角速度预测: 预测角度 = 当前角度 + 角速度 × 延迟时间
  - 根据预测角度来判断是否停止

作者：He
日期：2025-12-28
"""

import rclpy
from rclpy.node import Node
import time
import sys
import threading
import math

# ================= 配置参数 =================
CONFIG = {
    # -------- 网络配置 --------
    "NETWORK_INTERFACE": "127.0.0.1",
    "DOMAIN_ID": 0,
    
    # -------- 转身参数 --------
    "TURN_SPEED": 1.5,           # 转向角速度 (rad/s) - 可以设得比较大
    "FORWARD_SPEED": 0.1,        # 转身时的小前进速度，保持稳定
    "TARGET_ANGLE_DEG": 180.0,   # 目标转过的角度（度）
    "ANGLE_TOLERANCE_DEG": 3.0,  # 角度容差（度）- 减小容差提高精度
    
    # -------- 方向控制 --------
    "TURN_DIRECTION": 1,         # 1=左转(逆时针), -1=右转(顺时针)
    
    # -------- 控制参数 --------
    "CONTROL_HZ": 100,           # 控制频率 (Hz)，100Hz更精确
    
    # -------- 延迟补偿参数 --------
    # IMU延迟 = 传感器采样延迟 + 通信延迟 + 处理延迟 + 执行延迟
    "IMU_DELAY_MS": 80,          # IMU总延迟估计 (毫秒)，可根据实测调整
    "STOP_DELAY_MS": 50,         # 机器人停止响应延迟 (毫秒)
    
    # -------- 减速阈值 --------
    "SLOW_DOWN_ANGLE": 40.0,     # 快到目标前多少度开始减速
    "MIN_TURN_SPEED": 0.4,       # 减速时的最小转向速度
}


# 导入 SDK
try:
    from booster_robotics_sdk_python import (
        B1LocoClient, 
        ChannelFactory,
        B1LowStateSubscriber,
    )
    SDK_AVAILABLE = True
except ImportError:
    print("❌ Error: booster_robotics_sdk_python not found.")
    SDK_AVAILABLE = False
    sys.exit(1)


class IMUData:
    """IMU数据存储 - 带延迟补偿"""
    def __init__(self):
        self.yaw = 0.0           # 当前航向角 (rad)
        self.gyro_z = 0.0        # Z轴角速度 (rad/s)
        
        self.start_yaw = 0.0     # 起始航向
        self.total_yaw_deg = 0.0 # 累计转过的角度（度）
        self.last_yaw = 0.0
        self.initialized = False
        self.started = False     # 是否开始计算
        
        self.update_time = 0.0
        self.lock = threading.Lock()
    
    def start_tracking(self):
        """开始追踪角度"""
        with self.lock:
            self.start_yaw = self.yaw
            self.last_yaw = self.yaw
            self.total_yaw_deg = 0.0
            self.started = True
            self.initialized = True
            print(f"📍 开始追踪角度，起始航向: {math.degrees(self.start_yaw):.1f}°")
    
    def update(self, rpy, gyro):
        """更新IMU数据"""
        with self.lock:
            self.yaw = rpy[2]
            self.gyro_z = gyro[2]
            self.update_time = time.time()
            
            if self.started and self.initialized:
                # 计算增量角度
                delta = math.degrees(self.yaw - self.last_yaw)
                
                # 处理跳变 (-180 到 +180)
                if delta > 180:
                    delta -= 360
                elif delta < -180:
                    delta += 360
                
                self.total_yaw_deg += delta
                self.last_yaw = self.yaw
    
    def get_total_angle(self):
        """获取累计转过的角度（度）"""
        with self.lock:
            return self.total_yaw_deg
    
    def get_gyro_z(self):
        """获取当前Z轴角速度 (rad/s)"""
        with self.lock:
            return self.gyro_z
    
    def get_predicted_angle(self, delay_ms):
        """
        获取预测角度（考虑延迟补偿）
        
        原理：
        - 当前检测到的角度已经是 delay_ms 毫秒前的数据
        - 使用当前角速度预测这段时间内会继续转过的角度
        - 预测角度 = 当前角度 + 角速度 × 延迟时间
        
        参数：
            delay_ms: 总延迟时间（毫秒）= IMU延迟 + 停止响应延迟
        
        返回：
            预测的累计转过角度（度）
        """
        with self.lock:
            # 将延迟转换为秒
            delay_sec = delay_ms / 1000.0
            
            # 预测在延迟时间内会额外转过的角度（度）
            # gyro_z 单位是 rad/s，需要转换为度/s
            extra_angle = abs(self.gyro_z) * delay_sec * (180.0 / math.pi)
            
            # 预测的总角度
            predicted_angle = abs(self.total_yaw_deg) + extra_angle
            
            return predicted_angle
    
    def reset(self):
        """重置"""
        with self.lock:
            self.total_yaw_deg = 0.0
            self.started = False
            self.initialized = False


# 全局IMU数据
imu_data = IMUData()


class FastTurnSystem(Node):
    def __init__(self):
        super().__init__('fast_turn_system')
        self.get_logger().info("🤖 快速转身系统初始化...")
        
        self.running = True
        self.turning = False
        self.turn_complete = False
        
        self.current_vx = 0.0
        self.current_wz = 0.0
        
        self._init_sdk()
        self._init_imu_subscriber()
        
        # 控制线程
        self.control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self.turn_thread = threading.Thread(target=self._turn_loop, daemon=True)
        
        self.control_thread.start()
        time.sleep(0.5)
        self.turn_thread.start()
        
        self.get_logger().info("✅ 系统就绪")
        self.get_logger().info(f"📐 目标角度: {CONFIG['TARGET_ANGLE_DEG']}°")
        self.get_logger().info(f"🔄 转向速度: {CONFIG['TURN_SPEED']} rad/s")
        self.get_logger().info(f"➡️  前进速度: {CONFIG['FORWARD_SPEED']} m/s")
        
        # 等待IMU数据
        self.get_logger().info("⏳ 等待IMU数据...")
        time.sleep(1.0)
        
        # 自动开始转身
        self.start_turn()
    
    def _init_sdk(self):
        try:
            ChannelFactory.Instance().Init(CONFIG["DOMAIN_ID"], CONFIG["NETWORK_INTERFACE"])
            self.loco = B1LocoClient()
            self.loco.Init()
            time.sleep(1)
            self.loco.Move(0.0, 0.0, 0.0)
            self.get_logger().info("✓ SDK初始化成功")
        except Exception as e:
            self.get_logger().error(f"SDK初始化失败: {e}")
            self.loco = None
    
    def _init_imu_subscriber(self):
        global imu_data
        
        def imu_cb(msg):
            imu_data.update(msg.imu_state.rpy, msg.imu_state.gyro)
        
        try:
            self.imu_sub = B1LowStateSubscriber(imu_cb)
            self.imu_sub.InitChannel()
            self.get_logger().info("✓ IMU订阅成功")
        except Exception as e:
            self.get_logger().error(f"IMU订阅失败: {e}")
    
    def start_turn(self):
        """开始转身"""
        global imu_data
        
        if self.turning:
            self.get_logger().info("已在转身中...")
            return
        
        # 开始追踪角度
        imu_data.start_tracking()
        
        self.turning = True
        self.turn_complete = False
        
        direction_str = "左转(逆时针)" if CONFIG["TURN_DIRECTION"] > 0 else "右转(顺时针)"
        self.get_logger().info(f"🚀 开始快速{direction_str}转身!")
    
    def _control_loop(self):
        """发送运动指令"""
        interval = 1.0 / CONFIG["CONTROL_HZ"]
        
        while self.running and self.loco:
            try:
                self.loco.Move(self.current_vx, 0.0, self.current_wz)
                time.sleep(interval)
            except Exception as e:
                self.get_logger().error(f"发送指令失败: {e}")
                time.sleep(0.01)
    
    def _turn_loop(self):
        """
        转身主逻辑 - 使用延迟补偿
        
        关键：使用预测角度来判断停止时机
        - 当前读取的角度已经是 IMU_DELAY_MS 毫秒前的数据
        - 发出停止指令后，机器人还需要 STOP_DELAY_MS 毫秒才能停下
        - 因此需要提前停止，使用预测角度来判断
        """
        global imu_data
        
        last_print = 0
        target_deg = CONFIG["TARGET_ANGLE_DEG"]
        direction = CONFIG["TURN_DIRECTION"]
        
        # 计算总延迟（用于预测）
        total_delay_ms = CONFIG["IMU_DELAY_MS"] + CONFIG["STOP_DELAY_MS"]
        
        while self.running:
            try:
                if not self.turning:
                    self.current_vx = 0.0
                    self.current_wz = 0.0
                    time.sleep(0.05)
                    continue
                
                current_time = time.time()
                
                # 获取当前角度和预测角度
                current_deg = abs(imu_data.get_total_angle())
                gyro_z = imu_data.get_gyro_z()
                predicted_deg = imu_data.get_predicted_angle(total_delay_ms)
                
                # 使用预测角度来计算剩余角度
                remaining_deg = target_deg - predicted_deg
                
                # 检查是否应该停止（基于预测角度）
                if remaining_deg <= CONFIG["ANGLE_TOLERANCE_DEG"]:
                    final_angle = abs(imu_data.get_total_angle())
                    self.get_logger().info(f"✅ 转身完成!")
                    self.get_logger().info(f"   预测角度: {predicted_deg:.1f}°")
                    self.get_logger().info(f"   当前角度: {final_angle:.1f}°")
                    self.get_logger().info(f"   当前角速度: {abs(gyro_z):.2f} rad/s")
                    
                    # 立即停止
                    self.current_vx = 0.0
                    self.current_wz = 0.0
                    self.turning = False
                    self.turn_complete = True
                    
                    # 等待稳定后读取最终角度
                    time.sleep(0.5)
                    final_angle = abs(imu_data.get_total_angle())
                    overshoot = final_angle - target_deg
                    self.get_logger().info(f"   最终角度: {final_angle:.1f}° (误差: {overshoot:+.1f}°)")
                    
                    # 停几秒然后退出
                    time.sleep(2.0)
                    self.running = False
                    break
                
                # 计算转向速度（带平滑减速）
                # 使用预测角度来计算速度，确保平滑减速
                actual_remaining = target_deg - current_deg
                
                if actual_remaining <= CONFIG["SLOW_DOWN_ANGLE"]:
                    # 接近目标，减速
                    # 速度与剩余角度成比例
                    speed_ratio = actual_remaining / CONFIG["SLOW_DOWN_ANGLE"]
                    speed_ratio = max(0.1, min(1.0, speed_ratio))  # 限制在 0.1~1.0
                    turn_speed = CONFIG["MIN_TURN_SPEED"] + (CONFIG["TURN_SPEED"] - CONFIG["MIN_TURN_SPEED"]) * speed_ratio
                else:
                    # 全速转
                    turn_speed = CONFIG["TURN_SPEED"]
                
                # 设置速度
                self.current_vx = CONFIG["FORWARD_SPEED"]
                self.current_wz = turn_speed * direction
                
                # 打印状态
                if current_time - last_print > 0.2:  # 每200ms打印一次
                    self.get_logger().info(
                        f"[转身中] 当前: {current_deg:.1f}° 预测: {predicted_deg:.1f}° / {target_deg}° | "
                        f"剩余: {remaining_deg:.1f}° | "
                        f"速度: wz={self.current_wz:.2f} | 角速度: {abs(gyro_z):.2f}"
                    )
                    last_print = current_time
                
                time.sleep(0.01)  # 10ms
                
            except Exception as e:
                self.get_logger().error(f"转身错误: {e}")
                self.current_vx = 0.0
                self.current_wz = 0.0
                time.sleep(0.5)
    
    def stop(self):
        self.running = False
        self.turning = False
        if self.loco:
            for _ in range(20):
                self.loco.Move(0.0, 0.0, 0.0)
                time.sleep(0.01)
        self.get_logger().info("✓ 已停止")


def main(args=None):
    direction = "左转(逆时针)" if CONFIG["TURN_DIRECTION"] > 0 else "右转(顺时针)"
    total_delay = CONFIG["IMU_DELAY_MS"] + CONFIG["STOP_DELAY_MS"]
    
    print("=" * 60)
    print("  🤖 快速180度转身系统 (陀螺仪闭环 + 延迟补偿)")
    print("=" * 60)
    print(f"  方向: {direction}")
    print(f"  目标角度: {CONFIG['TARGET_ANGLE_DEG']}°")
    print(f"  转向速度: {CONFIG['TURN_SPEED']} rad/s")
    print(f"  前进速度: {CONFIG['FORWARD_SPEED']} m/s (保持稳定)")
    print(f"  减速阈值: 剩余 {CONFIG['SLOW_DOWN_ANGLE']}° 时开始减速")
    print("-" * 60)
    print("  📡 延迟补偿参数:")
    print(f"     IMU延迟: {CONFIG['IMU_DELAY_MS']}ms")
    print(f"     停止延迟: {CONFIG['STOP_DELAY_MS']}ms")
    print(f"     总延迟: {total_delay}ms")
    print(f"     (预测角度 = 当前角度 + 角速度 × {total_delay}ms)")
    print("=" * 60)
    print("  🚀 启动后自动开始转身！")
    print("  ⌨️  Ctrl+C 可中断")
    print("=" * 60)
    
    rclpy.init(args=args)
    node = FastTurnSystem()
    
    try:
        while node.running:
            rclpy.spin_once(node, timeout_sec=0.1)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n停止...")
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()
        print("✅ 系统已关闭")


if __name__ == '__main__':
    main()

