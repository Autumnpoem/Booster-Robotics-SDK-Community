#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
鲁棒闭环圆形行走程序 V1.1
========================
功能：使用陀螺仪+里程计实现稳定圆形行走，走完指定圈数后回到原点
特点：
  1. 手柄随时打断 (LT+RT+↑)
  2. 再次按键从当前位置重新开始
  3. 数据滤波+抗抖动
  4. 平滑加减速
  5. 每圈重新校准起点，避免漂移累积
作者：He
日期：2025-12-21
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
    
    # -------- 圆形行走参数 --------
    "CIRCLE_FORWARD_SPEED": 0.3,      # 前进速度 (m/s)，建议 0.2~0.5
    "CIRCLE_TURN_SPEED": 0.3,         # 转向角速度 (rad/s)，建议 0.2~0.5
    "CIRCLE_DIRECTION": 1,            # 1=左圈(逆时针), -1=右圈(顺时针)
    "CIRCLE_COUNT": 1,                # 走几圈
    
    # -------- 回原点控制 --------
    "RETURN_THRESHOLD_DEG": 330,      # 转过多少度后开始回原点
    "POSITION_TOLERANCE": 0.3,        # 到达原点的容差 (米)
    "HEADING_KP": 0.8,                # 航向P增益
    "MAX_RETURN_WZ": 1.0,             # 回原点最大角速度
    
    # -------- 控制参数 --------
    "CONTROL_HZ": 20,                 # 控制频率 (Hz)
    "SMOOTH_RATE": 0.05,              # 速度平滑变化率
    
    # -------- 滤波参数 --------
    "FILTER_ALPHA": 0.3,              # 低通滤波系数 (越小越平滑)
    "SENSOR_TIMEOUT": 1.0,            # 传感器超时 (秒)
}
# =============================================


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
    print("⚠️ 手柄订阅不可用，仅支持键盘控制")


# ==================== 工具类 ====================
class LowPassFilter:
    """低通滤波器"""
    def __init__(self, alpha=0.3):
        self.alpha = alpha
        self.value = 0.0
        self.initialized = False
    
    def update(self, raw):
        if not self.initialized:
            self.value = raw
            self.initialized = True
        else:
            self.value = self.alpha * raw + (1 - self.alpha) * self.value
        return self.value
    
    def reset(self):
        self.initialized = False
        self.value = 0.0


def smooth_transition(current, target, rate):
    """平滑过渡，避免速度突变"""
    diff = target - current
    if abs(diff) < rate:
        return target
    return current + rate if diff > 0 else current - rate


def normalize_angle(angle):
    """归一化角度到 [-π, π]"""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


# ==================== 传感器数据 ====================
class SensorData:
    """传感器数据存储与处理"""
    def __init__(self):
        # IMU数据
        self.yaw = 0.0
        self.gyro_z = 0.0
        self.gyro_filter = LowPassFilter(CONFIG["FILTER_ALPHA"])
        
        # 里程计数据
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_theta = 0.0
        
        # 起始位置（每圈开始时的位置）
        self.start_x = 0.0
        self.start_y = 0.0
        self.start_yaw = 0.0
        self.start_recorded = False
        
        # 全局起点（第一圈的起点，用于最终回归）
        self.global_start_x = 0.0
        self.global_start_y = 0.0
        self.global_start_recorded = False
        
        # 累计转过的角度 - 使用 yaw 差值法（更稳定）
        self.total_yaw_deg = 0.0
        self.last_yaw = 0.0
        self.yaw_initialized = False
        
        # 完成状态
        self.circles_completed = 0
        
        # 更新时间 - 初始化为当前时间，避免启动时误报超时
        self.imu_update_time = time.time()
        self.odom_update_time = time.time()
        
        self.lock = threading.Lock()
    
    def record_start_position(self, is_first_circle=True):
        """记录起始位置（从当前位置开始）"""
        with self.lock:
            self.start_x = self.odom_x
            self.start_y = self.odom_y
            self.start_yaw = self.yaw
            self.start_recorded = True
            
            # 第一圈时，同时记录全局起点
            if is_first_circle:
                self.global_start_x = self.odom_x
                self.global_start_y = self.odom_y
                self.global_start_recorded = True
                self.circles_completed = 0
            
            # 重置累计角度
            self.total_yaw_deg = 0.0
            self.yaw_initialized = False
            self.last_yaw = self.yaw
            self.gyro_filter.reset()
            
            print(f"📍 起始位置已记录: x={self.start_x:.3f}, y={self.start_y:.3f}, yaw={math.degrees(self.start_yaw):.1f}°")
    
    def update_imu(self, rpy, gyro):
        """更新IMU数据，使用yaw差值计算累计角度（比纯陀螺仪积分更稳定）"""
        with self.lock:
            current_yaw = rpy[2]
            self.yaw = current_yaw
            
            # 滤波处理陀螺仪数据（备用）
            self.gyro_z = self.gyro_filter.update(gyro[2])
            
            current_time = time.time()
            
            # 使用 yaw 差值法计算累计角度（避免纯陀螺仪积分漂移）
            if self.start_recorded:
                if self.yaw_initialized:
                    # 计算 yaw 变化量
                    delta_yaw = current_yaw - self.last_yaw
                    
                    # 处理 yaw 跳变（从 π 跳到 -π 或反之）
                    if delta_yaw > math.pi:
                        delta_yaw -= 2 * math.pi
                    elif delta_yaw < -math.pi:
                        delta_yaw += 2 * math.pi
                    
                    self.total_yaw_deg += math.degrees(delta_yaw)
                else:
                    self.yaw_initialized = True
                
                self.last_yaw = current_yaw
            
            self.imu_update_time = current_time
    
    def update_odom(self, x, y, theta):
        """更新里程计数据"""
        with self.lock:
            self.odom_x = x
            self.odom_y = y
            self.odom_theta = theta
            self.odom_update_time = time.time()
    
    def get_distance_to_start(self):
        """获取到起点的距离"""
        with self.lock:
            dx = self.odom_x - self.start_x
            dy = self.odom_y - self.start_y
            return math.sqrt(dx*dx + dy*dy)
    
    def get_angle_to_start(self):
        """获取指向起点的角度"""
        with self.lock:
            dx = self.start_x - self.odom_x
            dy = self.start_y - self.odom_y
            return math.atan2(dy, dx)
    
    def is_sensor_valid(self):
        """检查传感器数据是否有效（未超时）"""
        current = time.time()
        with self.lock:
            imu_age = current - self.imu_update_time
            odom_age = current - self.odom_update_time
        return imu_age < CONFIG["SENSOR_TIMEOUT"] and odom_age < CONFIG["SENSOR_TIMEOUT"]
    
    def reset(self):
        """完全重置"""
        with self.lock:
            self.total_yaw_deg = 0.0
            self.circles_completed = 0
            self.start_recorded = False
            self.gyro_initialized = False
            self.gyro_filter.reset()


# ==================== 手柄控制 ====================
class ControllerState:
    """
    手柄状态类
    组合键: LT + RT + 十字键上
    触发方式: 组合键释放时触发
    """
    def __init__(self):
        self.is_manual_mode = False
        self.last_toggle_time = 0
        self.toggle_cooldown = 0.5
        
        self.lt_pressed = False
        self.rt_pressed = False
        self.hat_up_pressed = False
        self.combo_was_pressed = False
        
        self.lock = threading.Lock()
    
    def check_toggle_combo(self, lt, rt, hat_up):
        """检测组合键，返回是否触发切换"""
        current_time = time.time()
        
        with self.lock:
            combo_now_pressed = lt and rt and hat_up
            
            self.lt_pressed = lt
            self.rt_pressed = rt
            self.hat_up_pressed = hat_up
            
            # 组合键按下
            if combo_now_pressed:
                if not self.combo_was_pressed:
                    print(f"[手柄] ✓ 组合键按下，等待释放...")
                self.combo_was_pressed = True
                return False
            
            # 组合键释放
            if self.combo_was_pressed and not combo_now_pressed:
                self.combo_was_pressed = False
                
                if current_time - self.last_toggle_time > self.toggle_cooldown:
                    self.last_toggle_time = current_time
                    self.is_manual_mode = not self.is_manual_mode
                    return True
            
            return False
    
    def update_from_sdk(self, msg):
        """从SDK手柄消息更新状态"""
        return self.check_toggle_combo(msg.lt, msg.rt, msg.hat_u)


# 全局实例
sensor_data = SensorData()
controller_state = ControllerState()


# ==================== 主系统 ====================
class RobustCircleWalk(Node):
    def __init__(self):
        super().__init__('robust_circle_walk')
        self.get_logger().info("🤖 鲁棒闭环圆形行走系统 V1.0 初始化...")
        
        self.running = True
        self.walking = False
        self.returning = False
        
        # 目标速度和当前速度（用于平滑过渡）
        self.target_vx = 0.0
        self.target_wz = 0.0
        self.current_vx = 0.0
        self.current_wz = 0.0
        
        # 目标角速度
        self.base_wz = CONFIG["CIRCLE_TURN_SPEED"] * CONFIG["CIRCLE_DIRECTION"]
        
        self._init_sdk()
        self._init_subscribers()
        self._init_keyboard()
        
        # 启动控制线程
        self.control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self.walk_thread = threading.Thread(target=self._walk_loop, daemon=True)
        
        self.control_thread.start()
        time.sleep(0.5)
        self.walk_thread.start()
        
        self.get_logger().info("✅ 系统就绪")
        self.get_logger().info("🎮 LT+RT+↑ 暂停/继续 | ⌨️ 'm'=切换 'r'=重置 'q'=退出")
        
        # 自动开始
        time.sleep(1.0)
        self.start_circle()
    
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
        
        # 手柄订阅
        if RC_SUBSCRIBER_AVAILABLE:
            def gamepad_cb(msg):
                if controller_state.update_from_sdk(msg):
                    self._on_mode_toggle()
            
            try:
                self.gamepad_sub = B1RemoteControllerStateSubscriber(gamepad_cb)
                self.gamepad_sub.InitChannel()
                self.get_logger().info("✓ 手柄订阅成功")
            except Exception as e:
                self.get_logger().warn(f"手柄订阅失败: {e}")
    
    def _init_keyboard(self):
        def kb_thread():
            import select
            print("\n[键盘] 's'=开始 'm'=暂停/继续 'r'=重置 'q'=退出")
            
            while self.running:
                try:
                    if sys.stdin in select.select([sys.stdin], [], [], 0.1)[0]:
                        key = sys.stdin.read(1).lower()
                        if key == 's':
                            self.start_circle()
                        elif key == 'm':
                            controller_state.check_toggle_combo(True, True, True)
                            time.sleep(0.1)
                            controller_state.check_toggle_combo(False, False, False)
                            self._on_mode_toggle()
                        elif key == 'r':
                            self._reset()
                        elif key == 'q':
                            self.running = False
                except:
                    time.sleep(0.1)
        
        self.kb_thread = threading.Thread(target=kb_thread, daemon=True)
        self.kb_thread.start()
    
    def _on_mode_toggle(self):
        """模式切换回调"""
        global controller_state, sensor_data
        
        if controller_state.is_manual_mode:
            # 切换到手动模式：暂停
            self.get_logger().info("⏸️ 已暂停（手动模式）")
            self.target_vx = 0.0
            self.target_wz = 0.0
        else:
            # 切换回自动模式：从当前位置重新开始
            self.get_logger().info("▶️ 从当前位置重新开始")
            sensor_data.record_start_position()
            self.walking = True
            self.returning = False
    
    def _reset(self):
        """重置状态"""
        global sensor_data
        sensor_data.reset()
        self.walking = False
        self.returning = False
        self.target_vx = 0.0
        self.target_wz = 0.0
        self.get_logger().info("🔄 已重置")
    
    def start_circle(self):
        """开始走圈"""
        global sensor_data
        
        if self.walking:
            self.get_logger().info("已在走圈中...")
            return
        
        time.sleep(0.5)
        sensor_data.record_start_position()
        self.walking = True
        self.returning = False
        self.get_logger().info(f"🚀 开始走圈! 目标: {CONFIG['CIRCLE_COUNT']}圈")
    
    def _control_loop(self):
        """发送运动指令，带平滑过渡"""
        global controller_state
        interval = 1.0 / CONFIG["CONTROL_HZ"]
        
        while self.running and self.loco:
            try:
                # 手动模式或未走圈：停止
                if controller_state.is_manual_mode or not self.walking:
                    self.target_vx = 0.0
                    self.target_wz = 0.0
                
                # 平滑过渡
                self.current_vx = smooth_transition(
                    self.current_vx, self.target_vx, CONFIG["SMOOTH_RATE"]
                )
                self.current_wz = smooth_transition(
                    self.current_wz, self.target_wz, CONFIG["SMOOTH_RATE"]
                )
                
                self.loco.Move(self.current_vx, 0.0, self.current_wz)
                time.sleep(interval)
            except Exception as e:
                self.get_logger().error(f"控制错误: {e}")
                time.sleep(0.1)
    
    def _walk_loop(self):
        """走圈主逻辑"""
        global sensor_data, controller_state
        
        last_print = 0
        
        while self.running:
            try:
                # 手动模式：暂停处理
                if controller_state.is_manual_mode:
                    time.sleep(0.1)
                    continue
                
                if not self.walking:
                    time.sleep(0.1)
                    continue
                
                # 检查传感器
                if not sensor_data.is_sensor_valid():
                    self.get_logger().warn("⚠️ 传感器数据超时，停止")
                    self.target_vx = 0.0
                    self.target_wz = 0.0
                    time.sleep(0.5)
                    continue
                
                # 获取状态
                with sensor_data.lock:
                    total_deg = sensor_data.total_yaw_deg
                    dist_to_start = math.sqrt(
                        (sensor_data.odom_x - sensor_data.start_x)**2 +
                        (sensor_data.odom_y - sensor_data.start_y)**2
                    )
                    angle_to_start = math.atan2(
                        sensor_data.start_y - sensor_data.odom_y,
                        sensor_data.start_x - sensor_data.odom_x
                    )
                    current_heading = sensor_data.odom_theta
                    circles_done = sensor_data.circles_completed
                
                abs_deg = abs(total_deg)
                target_deg = 360 * CONFIG["CIRCLE_COUNT"]
                
                # ========== 阶段判断 ==========
                if abs_deg < CONFIG["RETURN_THRESHOLD_DEG"]:
                    # 阶段1: 开环走圈
                    self.returning = False
                    self.target_vx = CONFIG["CIRCLE_FORWARD_SPEED"]
                    self.target_wz = self.base_wz
                    
                elif abs_deg < target_deg:
                    # 阶段2: 闭环回原点
                    if not self.returning:
                        self.returning = True
                        self.get_logger().info(f"🎯 转够 {abs_deg:.0f}°，开始返回原点...")
                    
                    # 检查是否到达
                    if dist_to_start < CONFIG["POSITION_TOLERANCE"]:
                        with sensor_data.lock:
                            sensor_data.circles_completed += 1
                            completed = sensor_data.circles_completed
                        
                        self.get_logger().info(f"✅ 第 {completed} 圈完成! 误差: {dist_to_start:.2f}m")
                        
                        if completed >= CONFIG["CIRCLE_COUNT"]:
                            self.get_logger().info(f"🎉 全部 {CONFIG['CIRCLE_COUNT']} 圈完成!")
                            self.walking = False
                            self.target_vx = 0.0
                            self.target_wz = 0.0
                        else:
                            # ★ 关键修复：每圈完成后，从当前位置重新记录起点
                            # 这样可以避免累计误差
                            sensor_data.record_start_position(is_first_circle=False)
                            self.returning = False
                            self.get_logger().info(f"🔄 开始第 {completed+1} 圈... (起点已校准)")
                        continue
                    
                    # 航向控制：朝向起点
                    heading_error = normalize_angle(angle_to_start - current_heading)
                    
                    self.target_vx = CONFIG["CIRCLE_FORWARD_SPEED"] * 0.7
                    self.target_wz = CONFIG["HEADING_KP"] * heading_error
                    self.target_wz = max(-CONFIG["MAX_RETURN_WZ"], 
                                         min(CONFIG["MAX_RETURN_WZ"], self.target_wz))
                
                # 打印状态
                current_time = time.time()
                if current_time - last_print > 2.0:
                    phase = "🎯 回原点" if self.returning else "🔄 走圈"
                    self.get_logger().info(
                        f"[{phase}] 累计:{total_deg:.1f}° | "
                        f"距起点:{dist_to_start:.2f}m | "
                        f"圈数:{circles_done}/{CONFIG['CIRCLE_COUNT']}"
                    )
                    last_print = current_time
                
                time.sleep(0.05)
                
            except Exception as e:
                self.get_logger().error(f"行走错误: {e}")
                self.target_vx = 0.0
                self.target_wz = 0.0
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
    
    print("=" * 60)
    print("  🤖 鲁棒闭环圆形行走系统 V1.1 (多圈漂移修复版)")
    print("=" * 60)
    print(f"  方向: {direction}")
    print(f"  圈数: {CONFIG['CIRCLE_COUNT']}")
    print(f"  前进速度: {CONFIG['CIRCLE_FORWARD_SPEED']} m/s")
    print(f"  转向速度: {CONFIG['CIRCLE_TURN_SPEED']} rad/s")
    print(f"  回原点阈值: {CONFIG['RETURN_THRESHOLD_DEG']}°")
    print(f"  位置容差: {CONFIG['POSITION_TOLERANCE']} m")
    print("=" * 60)
    print("  🎮 手柄: LT+RT+↑ 暂停/继续")
    print("  ⌨️ 键盘: 's'=开始 'm'=暂停/继续 'r'=重置 'q'=退出")
    print("  🚀 启动后自动开始!")
    print("=" * 60)
    
    rclpy.init(args=args)
    node = RobustCircleWalk()
    
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
