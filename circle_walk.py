#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
基于陀螺仪+里程计的闭环圆形行走程序 V2.1
======================================
功能：使用IMU和里程计双闭环控制，实现稳定圆形行走并回到原点
原理：
  1. 使用陀螺仪追踪累计转过的角度
  2. 使用里程计记录起始位置
  3. 快完成一圈时，平滑过渡引导回起点
作者：He
日期：2025-12-21

手柄切换组合键: LT + RT + 十字键上
"""

import rclpy
from rclpy.node import Node
import time
import sys
import threading
import math

# ================= 配置参数 =================
# 📖 详细说明见 circle_walk_guide.md
CONFIG = {
    # -------- 网络配置 --------
    "NETWORK_INTERFACE": "127.0.0.1",  # SDK通信地址，127.0.0.1=本机
    "DOMAIN_ID": 0,                     # DDS Domain ID，通常不需要改
    
    # -------- 圆形行走参数 --------
    # 圆的大小 ≈ 前进速度 / 转向速度
    # 例如: 0.25/0.25 ≈ 1m半径, 0.4/0.2 ≈ 2m半径
    "CIRCLE_RADIUS": 1.8,               # 目标圆半径 (米) - 仅作参考
    "CIRCLE_FORWARD_SPEED": 1.8,        # 前进速度 (m/s)，建议 0.1~0.5
    "CIRCLE_TURN_SPEED": 1.2,           # 转向角速度 (rad/s)，建议 0.1~0.5
    
    # -------- 控制参数 --------
    "CONTROL_HZ": 50,                   # 控制频率 (Hz)，50Hz更平滑
    
    # -------- 角速度闭环控制 --------
    # 使用陀螺仪反馈，让实际转向速度接近目标
    # 值越大校正越激进（可能抖动），值越小校正越温和
    "YAW_RATE_KP": 0.3,                 # 角速度P增益，建议 0.1~0.5
    
    # -------- 回原点控制 --------
    # 当累计转过 RETURN_THRESHOLD_DEG 度后，开始引导回起点
    "POSITION_KP": 0.5,                 # 位置P增益（保留）
    "HEADING_KP": 1.0,                  # 航向P增益，控制朝向起点的灵敏度
    "RETURN_THRESHOLD_DEG": 330,        # 转过多少度后开始回原点，建议 300~350
    "POSITION_TOLERANCE": 0.1,          # 距起点多近算"到达" (米)，放宽避免微调
    
    # -------- 方向控制 --------
    "CIRCLE_DIRECTION": 1,              # 1=左圈(逆时针), -1=右圈(顺时针)
    
    # -------- 运行模式 --------
    "SINGLE_CIRCLE_MODE": False,        # True=走一圈停止, False=连续走圈
}
# ===========================================

# 导入 SDK
try:
    from booster_robotics_sdk_python import (
        B1LocoClient, 
        ChannelFactory,
        B1LowStateSubscriber,
        B1OdometerStateSubscriber,
    )
    SDK_AVAILABLE = True
except ImportError:
    print("❌ Error: booster_robotics_sdk_python not found.")
    SDK_AVAILABLE = False
    sys.exit(1)

# 手柄订阅
try:
    from booster_robotics_sdk_python import B1RemoteControllerStateSubscriber
    RC_SUBSCRIBER_AVAILABLE = True
except ImportError:
    RC_SUBSCRIBER_AVAILABLE = False


# ==================== 传感器数据 ====================
class SensorData:
    """传感器数据存储"""
    def __init__(self):
        # IMU数据
        self.yaw = 0.0
        self.gyro_z = 0.0
        
        # 里程计数据
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_theta = 0.0
        
        # 起始位置
        self.start_x = 0.0
        self.start_y = 0.0
        self.start_yaw = 0.0
        self.start_recorded = False
        
        # 累计转过的角度
        self.total_yaw_deg = 0.0
        self.last_yaw = 0.0
        self.yaw_initialized = False
        
        # 圈数
        self.circle_count = 0
        self.circle_completed = False
        
        # 更新时间
        self.imu_update_time = 0.0
        self.odom_update_time = 0.0
        
        self.lock = threading.Lock()
    
    def record_start_position(self):
        """记录起始位置"""
        with self.lock:
            self.start_x = self.odom_x
            self.start_y = self.odom_y
            self.start_yaw = self.yaw
            self.start_recorded = True
            self.total_yaw_deg = 0.0
            self.circle_completed = False
            print(f"📍 起始位置已记录: x={self.start_x:.3f}, y={self.start_y:.3f}")
    
    def update_imu(self, rpy, gyro):
        """更新IMU数据"""
        with self.lock:
            self.yaw = rpy[2]
            self.gyro_z = gyro[2]
            
            # 计算累计角度
            if self.start_recorded:
                if self.yaw_initialized:
                    delta = math.degrees(self.yaw - self.last_yaw)
                    # 处理跳变
                    if delta > 180:
                        delta -= 360
                    elif delta < -180:
                        delta += 360
                    
                    self.total_yaw_deg += delta
                    
                    # 检查是否完成一圈
                    if abs(self.total_yaw_deg) >= 360:
                        self.circle_count += 1
                        self.circle_completed = True
                else:
                    self.yaw_initialized = True
                
                self.last_yaw = self.yaw
            
            self.imu_update_time = time.time()
    
    def update_odom(self, x, y, theta):
        """更新里程计数据"""
        with self.lock:
            self.odom_x = x
            self.odom_y = y
            self.odom_theta = theta
            self.odom_update_time = time.time()
    
    def _get_distance_to_start_internal(self):
        """获取到起点的距离（内部版本，调用前需已持有锁）"""
        dx = self.odom_x - self.start_x
        dy = self.odom_y - self.start_y
        return math.sqrt(dx*dx + dy*dy)
    
    def _get_angle_to_start_internal(self):
        """获取指向起点的角度（内部版本，调用前需已持有锁）"""
        dx = self.start_x - self.odom_x
        dy = self.start_y - self.odom_y
        return math.atan2(dy, dx)
    
    def get_distance_to_start(self):
        """获取到起点的距离"""
        with self.lock:
            return self._get_distance_to_start_internal()
    
    def get_angle_to_start(self):
        """获取指向起点的角度"""
        with self.lock:
            return self._get_angle_to_start_internal()
    
    def reset(self):
        """重置状态"""
        with self.lock:
            self.total_yaw_deg = 0.0
            self.circle_completed = False
            self.yaw_initialized = False
            self.start_recorded = False


class ControllerState:
    """
    手柄状态类 - 参考 move.py 实现
    组合键: LT + RT + 十字键上
    触发方式: 组合键释放时触发 -> 退出程序
    """
    def __init__(self):
        self.exit_requested = False  # 改为退出请求
        self.last_toggle_time = 0
        self.toggle_cooldown = 0.5
        
        # 手柄按键状态
        self.lt_pressed = False
        self.rt_pressed = False
        self.hat_up_pressed = False
        
        # 用于释放检测
        self.combo_was_pressed = False
        
        # Debug
        self.debug_enabled = True
        self.last_debug_time = 0
        self.debug_interval = 0.2
        
        self.lock = threading.Lock()
    
    def check_toggle_combo(self, lt, rt, hat_up):
        """
        检测 LT + RT + 十字上 组合键
        触发方式：组合键全部按下后，释放时触发切换
        """
        current_time = time.time()
        
        with self.lock:
            combo_now_pressed = lt and rt and hat_up
            
            # Debug输出（限制频率）
            if self.debug_enabled and (current_time - self.last_debug_time >= self.debug_interval):
                if (lt or rt or hat_up) or self.combo_was_pressed:
                    print(f"[DEBUG 按键] LT={lt}, RT={rt}, 十字上={hat_up} | "
                          f"组合键完整={combo_now_pressed}, 曾完整按下={self.combo_was_pressed}")
                    self.last_debug_time = current_time
            
            # 更新按键状态
            self.lt_pressed = lt
            self.rt_pressed = rt
            self.hat_up_pressed = hat_up
            
            # 释放检测逻辑：
            if combo_now_pressed:
                if not self.combo_was_pressed:
                    print(f"[DEBUG] ✓ 组合键已完整按下 (LT+RT+十字上)，等待释放...")
                self.combo_was_pressed = True
                return False
            
            # 如果组合键曾经完整按下，现在释放了任一按键，触发退出
            if self.combo_was_pressed and not combo_now_pressed:
                self.combo_was_pressed = False
                
                if current_time - self.last_toggle_time > self.toggle_cooldown:
                    self.last_toggle_time = current_time
                    self.exit_requested = True  # 请求退出
                    print(f"[DEBUG] ★ 组合键释放！请求退出程序")
                    return True
                else:
                    print(f"[DEBUG] ⚠️ 组合键释放但在防抖时间内，忽略")
            
            return False
    
    def update_from_sdk(self, msg):
        """从SDK手柄消息更新状态"""
        current_time = time.time()
        
        with self.lock:
            prev_lt = self.lt_pressed
            prev_rt = self.rt_pressed
            prev_hat_up = self.hat_up_pressed
            
            # SDK原始数据debug输出
            if self.debug_enabled:
                key_changed = (prev_lt != msg.lt or prev_rt != msg.rt or prev_hat_up != msg.hat_u)
                if key_changed:
                    print(f"[SDK 回调] 按键变化: LT={msg.lt}, RT={msg.rt}, 十字上={msg.hat_u}")
        
        # 检查组合键 (LT + RT + 十字上)
        return self.check_toggle_combo(msg.lt, msg.rt, msg.hat_u)


# 全局实例
sensor_data = SensorData()
controller_state = ControllerState()


# ==================== 主系统 ====================
class ClosedLoopCircleWalk(Node):
    def __init__(self):
        super().__init__('closed_loop_circle_walk')
        self.get_logger().info("🤖 闭环圆形行走系统 V2.1 初始化...")
        
        self.running = True
        self.walking = False
        self.returning = False  # 回原点阶段
        
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_wz = 0.0
        
        # 目标角速度
        self.target_wz = CONFIG["CIRCLE_TURN_SPEED"] * CONFIG["CIRCLE_DIRECTION"]
        
        self._init_sdk()
        self._init_subscribers()
        self._init_keyboard()
        
        # 控制线程
        self.control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self.walk_thread = threading.Thread(target=self._walk_loop, daemon=True)
        
        self.control_thread.start()
        time.sleep(0.5)
        self.walk_thread.start()
        
        self.get_logger().info("✅ 系统就绪")
        self.get_logger().info("🎮 LT+RT+↑ 退出程序 | ⌨️ 'q'=退出")
        
        # 🚀 立即开始走圈！不等待
        self.start_circle()
        self.get_logger().info("🚀 已自动开始走圈！按组合键暂停")
    
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
    
    def _init_subscribers(self):
        global sensor_data, controller_state
        
        # IMU订阅
        def imu_cb(msg):
            sensor_data.update_imu(msg.imu_state.rpy, msg.imu_state.gyro)
        
        try:
            self.imu_sub = B1LowStateSubscriber(imu_cb)
            self.imu_sub.InitChannel()
            self.get_logger().info("✓ IMU订阅成功")
        except Exception as e:
            self.get_logger().error(f"IMU订阅失败: {e}")
        
        # 里程计订阅
        def odom_cb(msg):
            sensor_data.update_odom(msg.x, msg.y, msg.theta)
        
        try:
            self.odom_sub = B1OdometerStateSubscriber(odom_cb)
            self.odom_sub.InitChannel()
            self.get_logger().info("✓ 里程计订阅成功")
        except Exception as e:
            self.get_logger().error(f"里程计订阅失败: {e}")
        
        # 手柄订阅 - 使用 update_from_sdk 方法
        if RC_SUBSCRIBER_AVAILABLE:
            def gamepad_cb(msg):
                if controller_state.update_from_sdk(msg):
                    if controller_state.exit_requested:
                        self.get_logger().info(f"[手柄] 收到退出请求！")
                        self.running = False  # 停止程序
            
            try:
                self.gamepad_sub = B1RemoteControllerStateSubscriber(gamepad_cb)
                self.gamepad_sub.InitChannel()
                self.get_logger().info("✓ 手柄订阅成功")
            except Exception as e:
                self.get_logger().warn(f"手柄订阅失败: {e}")
    
    def _init_keyboard(self):
        def kb_thread():
            import select
            print("\n[键盘] 's'=开始走圈 'm'=切换模式 'r'=重置 'q'=退出")
            
            while self.running:
                try:
                    if sys.stdin in select.select([sys.stdin], [], [], 0.1)[0]:
                        key = sys.stdin.read(1).lower()
                        if key == 's':
                            self.start_circle()
                        elif key == 'm':
                            # 模拟组合键按下再释放
                            controller_state.check_toggle_combo(True, True, True)
                            time.sleep(0.1)
                            controller_state.check_toggle_combo(False, False, False)
                            mode = "手动(暂停)" if controller_state.is_manual_mode else "自动(走圈)"
                            self.get_logger().info(f"[键盘] 模式: {mode}")
                        elif key == 'r':
                            sensor_data.reset()
                            self.walking = False
                            self.returning = False
                            self.get_logger().info("[键盘] 已重置")
                        elif key == 'q':
                            self.running = False
                except:
                    time.sleep(0.1)
        
        self.kb_thread = threading.Thread(target=kb_thread, daemon=True)
        self.kb_thread.start()
    
    def start_circle(self):
        """开始走圈 - 立即启动，不等待"""
        global sensor_data
        
        if self.walking:
            self.get_logger().info("已在走圈中...")
            return
        
        # 立即设置目标速度，不做任何等待
        self.current_vx = CONFIG["CIRCLE_FORWARD_SPEED"]
        self.current_wz = self.target_wz
        
        # 记录起始位置（用于后续回原点）
        sensor_data.record_start_position()
        
        self.walking = True
        self.returning = False
        self.get_logger().info(f"🚀 开始走圈! 立即速度: vx={self.current_vx}, wz={self.current_wz:.2f}")
    
    def _control_loop(self):
        """发送运动指令"""
        global controller_state
        interval = 1.0 / CONFIG["CONTROL_HZ"]
        
        while self.running and self.loco:
            try:
                # 退出请求：立即停止
                if controller_state.exit_requested or not self.running:
                    self.loco.Move(0.0, 0.0, 0.0)
                elif not self.walking:
                    self.loco.Move(0.0, 0.0, 0.0)
                else:
                    self.loco.Move(self.current_vx, self.current_vy, self.current_wz)
                time.sleep(interval)
            except:
                time.sleep(0.1)
    
    def _walk_loop(self):
        """
        走圈主逻辑 - 三阶段控制
        
        阶段1: 开环走圈（转够360度）
        阶段2: 停下来等IMU稳定
        阶段3: 直线走回原点
        """
        global sensor_data, controller_state
        
        last_print = 0
        
        # 状态机
        STATE_CIRCLING = 0       # 走圈中
        STATE_STABILIZING = 1    # 等待IMU稳定
        STATE_RETURNING = 2      # 返回原点
        STATE_DONE = 3           # 完成
        
        state = STATE_CIRCLING
        stabilize_start_time = 0
        STABILIZE_DURATION = 2.0  # 等待IMU稳定的时间（秒）
        
        # 返回原点相关
        return_start_yaw = 0.0   # 开始返回时的航向
        target_yaw = 0.0         # 目标航向（指向原点）
        
        while self.running:
            try:
                # 退出请求：停止循环
                if controller_state.exit_requested or not self.running:
                    break
                
                if not self.walking:
                    time.sleep(0.1)
                    continue
                
                current_time = time.time()
                
                # 获取传感器数据
                with sensor_data.lock:
                    total_deg = sensor_data.total_yaw_deg
                    dist_to_start = sensor_data._get_distance_to_start_internal()
                    angle_to_start = sensor_data._get_angle_to_start_internal()
                    current_yaw = sensor_data.yaw
                    odom_x = sensor_data.odom_x
                    odom_y = sensor_data.odom_y
                    start_x = sensor_data.start_x
                    start_y = sensor_data.start_y
                
                abs_deg = abs(total_deg)
                
                # ========================================
                # 状态机
                # ========================================
                
                if state == STATE_CIRCLING:
                    # 阶段1: 开环走圈
                    if abs_deg < 360:
                        self.current_vx = CONFIG["CIRCLE_FORWARD_SPEED"]
                        self.current_wz = self.target_wz
                    else:
                        # 转够360度，停下来
                        self.get_logger().info(f"✅ 走圈完成! 累计转过 {abs_deg:.1f}°")
                        self.current_vx = 0.0
                        self.current_wz = 0.0
                        state = STATE_STABILIZING
                        stabilize_start_time = current_time
                        self.get_logger().info(f"⏸️ 停下等待IMU稳定 ({STABILIZE_DURATION}秒)...")
                
                elif state == STATE_STABILIZING:
                    # 阶段2: 停下等待IMU稳定
                    self.current_vx = 0.0
                    self.current_wz = 0.0
                    
                    if current_time - stabilize_start_time >= STABILIZE_DURATION:
                        # IMU稳定了，计算回原点的航向
                        target_yaw = angle_to_start  # 指向原点的角度
                        return_start_yaw = current_yaw
                        
                        self.get_logger().info(f"🎯 开始返回原点! 距离: {dist_to_start:.2f}m")
                        self.get_logger().info(f"   当前航向: {math.degrees(current_yaw):.1f}°")
                        self.get_logger().info(f"   目标航向: {math.degrees(target_yaw):.1f}°")
                        
                        state = STATE_RETURNING
                
                elif state == STATE_RETURNING:
                    # 阶段3: 直线走回原点
                    
                    # 重新计算目标航向（实时更新）
                    dx = start_x - odom_x
                    dy = start_y - odom_y
                    target_yaw = math.atan2(dy, dx)
                    
                    # 计算航向误差
                    heading_error = target_yaw - current_yaw
                    # 归一化到 [-π, π]
                    while heading_error > math.pi:
                        heading_error -= 2 * math.pi
                    while heading_error < -math.pi:
                        heading_error += 2 * math.pi
                    
                    # 检查是否到达原点
                    if dist_to_start < CONFIG["POSITION_TOLERANCE"]:
                        self.get_logger().info(f"✅ 到达原点! 误差: {dist_to_start:.2f}m")
                        self.current_vx = 0.0
                        self.current_wz = 0.0
                        
                        if CONFIG["SINGLE_CIRCLE_MODE"]:
                            state = STATE_DONE
                            self.walking = False
                        else:
                            # 连续模式：重新开始
                            with sensor_data.lock:
                                sensor_data.total_yaw_deg = 0.0
                                sensor_data.yaw_initialized = False
                                sensor_data.record_start_position()
                            state = STATE_CIRCLING
                            self.get_logger().info("🔄 开始下一圈...")
                    else:
                        # 直线走向原点
                        # 先转向，再前进
                        if abs(heading_error) > 0.3:  # 航向误差大于17度，先原地转
                            self.current_vx = 0.0
                            self.current_wz = CONFIG["HEADING_KP"] * heading_error
                        else:
                            # 航向OK，边走边修正
                            self.current_vx = CONFIG["CIRCLE_FORWARD_SPEED"] * 0.6
                            self.current_wz = CONFIG["HEADING_KP"] * heading_error
                        
                        # 限幅
                        max_wz = 1.0
                        self.current_wz = max(-max_wz, min(max_wz, self.current_wz))
                
                elif state == STATE_DONE:
                    self.current_vx = 0.0
                    self.current_wz = 0.0
                
                # 打印状态
                if current_time - last_print > 2.0:
                    if state == STATE_CIRCLING:
                        self.get_logger().info(f"[🔄 走圈] 累计:{total_deg:.1f}°")
                    elif state == STATE_STABILIZING:
                        remaining = STABILIZE_DURATION - (current_time - stabilize_start_time)
                        self.get_logger().info(f"[⏸️ 稳定中] 剩余:{remaining:.1f}秒")
                    elif state == STATE_RETURNING:
                        heading_err_deg = math.degrees(target_yaw - current_yaw)
                        self.get_logger().info(f"[🎯 返回] 距离:{dist_to_start:.2f}m 航向误差:{heading_err_deg:.1f}°")
                    last_print = current_time
                
                time.sleep(0.015)  # 15ms
                
            except Exception as e:
                self.get_logger().error(f"行走错误: {e}")
                self.current_vx = 0.0
                self.current_wz = 0.0
                time.sleep(0.5)
    
    def stop(self):
        self.running = False
        self.walking = False
        if self.loco:
            for _ in range(20):
                self.loco.Move(0.0, 0.0, 0.0)
                time.sleep(0.01)
        self.get_logger().info("✓ 已停止")


def main(args=None):
    direction = "左圈(逆时针)" if CONFIG["CIRCLE_DIRECTION"] > 0 else "右圈(顺时针)"
    mode = "单圈模式" if CONFIG["SINGLE_CIRCLE_MODE"] else "连续模式"
    
    print("=" * 60)
    print("  🤖 闭环圆形行走系统 V2.1 (陀螺仪+里程计)")
    print("=" * 60)
    print(f"  方向: {direction}")
    print(f"  模式: {mode}")
    print(f"  前进速度: {CONFIG['CIRCLE_FORWARD_SPEED']} m/s")
    print(f"  回原点阈值: {CONFIG['RETURN_THRESHOLD_DEG']}°")
    print(f"  位置容差: {CONFIG['POSITION_TOLERANCE']} m")
    print("=" * 60)
    print("  🎮 手柄: LT+RT+↑ 暂停/继续")
    print("  ⌨️ 键盘: 'm'=暂停/继续 'r'=重置 'q'=退出")
    print("  🚀 启动后自动开始走圈！")
    print("=" * 60)
    
    rclpy.init(args=args)
    node = ClosedLoopCircleWalk()
    
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
