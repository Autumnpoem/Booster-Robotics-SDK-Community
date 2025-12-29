#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
鲁棒闭环直线往返程序 V1.0
========================
功能：机器人走直线到指定距离，然后转身返回，可循环往返
特点：
  1. 航向保持：使用陀螺仪闭环校正偏航
  2. 距离跟踪：里程计计算行走距离
  3. 手柄随时打断 (LT+RT+↑)
  4. 再次按键从当前位置重新开始
  5. 数据滤波+平滑加减速
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
    
    # -------- 直线行走参数 --------
    "LINE_DISTANCE": 2.0,             # 单向行走距离 (米)
    "LINE_SPEED": 1.8,                # 前进速度 (m/s)
    "ROUND_TRIP_COUNT": 1,            # 往返次数 (1 = 去一次，回一次)
    
    # -------- 返回模式 --------
    # "backward" = 倒退返回（不转身）
    # "turn"     = 转身180°后前进返回
    # "one_way"  = 单向模式，只前进到目标点就停止，不返回
    "RETURN_MODE": "one_way",        # 选择返回模式
    
    # -------- 航向保持控制 --------
    "HEADING_KP": 1.5,                # 航向P增益（校正偏航，增大以更快校正）
    "MAX_CORRECTION_WZ": 0.8,         # 最大校正角速度 (rad/s)
    
    # -------- 转身参数（仅 turn 模式用） --------
    "TURN_SPEED": 0.5,                # 转身角速度 (rad/s)
    "TURN_TOLERANCE": 0.1,            # 转身角度容差 (rad，约5.7°)
    
    # -------- 容差参数 --------
    "DISTANCE_TOLERANCE": 0.1,        # 距离容差 (米)
    "ARRIVAL_TOLERANCE": 0.2,         # 返回原点容差 (米)
    
    # -------- 控制参数 --------
    "CONTROL_HZ": 20,                 # 控制频率 (Hz)
    # 移除 SMOOTH_RATE - 直接设置速度，无需平滑过渡
    
    # -------- 滤波参数 --------
    "FILTER_ALPHA": 0.3,              # 低通滤波系数
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


# ==================== 状态机 ====================
class WalkState:
    IDLE = "IDLE"
    FORWARD = "FORWARD"       # 前进到目标点
    TURNING = "TURNING"       # 转身180度
    BACKWARD = "BACKWARD"     # 返回起点
    ARRIVED = "ARRIVED"       # 到达


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
    """平滑过渡"""
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
        
        # 起始位置和目标航向
        self.start_x = 0.0
        self.start_y = 0.0
        self.target_heading = 0.0  # 目标航向（直线方向）
        self.start_recorded = False
        
        # 更新时间 - 初始化为当前时间，避免启动时误报超时
        self.imu_update_time = time.time()
        self.odom_update_time = time.time()
        
        self.lock = threading.Lock()
    
    def record_start_position(self):
        """记录起始位置和目标航向"""
        with self.lock:
            self.start_x = self.odom_x
            self.start_y = self.odom_y
            self.target_heading = self.odom_theta  # 当前朝向作为直线方向
            self.start_recorded = True
            self.gyro_filter.reset()
            
            print(f"📍 起始位置: x={self.start_x:.3f}, y={self.start_y:.3f}")
            print(f"🧭 目标航向: {math.degrees(self.target_heading):.1f}°")
    
    def update_imu(self, rpy, gyro):
        """更新IMU数据"""
        with self.lock:
            self.yaw = rpy[2]
            self.gyro_z = self.gyro_filter.update(gyro[2])
            self.imu_update_time = time.time()
    
    def update_odom(self, x, y, theta):
        """更新里程计数据"""
        with self.lock:
            self.odom_x = x
            self.odom_y = y
            self.odom_theta = theta
            self.odom_update_time = time.time()
    
    def get_distance_from_start(self):
        """获取从起点行走的距离"""
        with self.lock:
            dx = self.odom_x - self.start_x
            dy = self.odom_y - self.start_y
            return math.sqrt(dx*dx + dy*dy)
    
    def get_heading_error(self):
        """获取航向误差（当前朝向与目标航向的差）"""
        with self.lock:
            error = normalize_angle(self.target_heading - self.odom_theta)
            return error
    
    def get_reverse_heading_error(self):
        """获取反向航向误差（用于返回）"""
        with self.lock:
            reverse_heading = normalize_angle(self.target_heading + math.pi)
            error = normalize_angle(reverse_heading - self.odom_theta)
            return error
    
    def is_sensor_valid(self):
        """检查传感器数据是否有效"""
        current = time.time()
        with self.lock:
            imu_age = current - self.imu_update_time
            odom_age = current - self.odom_update_time
        return imu_age < CONFIG["SENSOR_TIMEOUT"] and odom_age < CONFIG["SENSOR_TIMEOUT"]
    
    def reset(self):
        """重置"""
        with self.lock:
            self.start_recorded = False
            self.gyro_filter.reset()


# ==================== 手柄控制 ====================
class ControllerState:
    """手柄状态类"""
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
        """检测组合键"""
        current_time = time.time()
        
        with self.lock:
            combo_now_pressed = lt and rt and hat_up
            
            self.lt_pressed = lt
            self.rt_pressed = rt
            self.hat_up_pressed = hat_up
            
            if combo_now_pressed:
                if not self.combo_was_pressed:
                    print(f"[手柄] ✓ 组合键按下，等待释放...")
                self.combo_was_pressed = True
                return False
            
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
class RobustLineWalk(Node):
    def __init__(self):
        super().__init__('robust_line_walk')
        self.get_logger().info("🤖 鲁棒闭环直线往返系统 V1.0 初始化...")
        
        self.running = True
        self.state = WalkState.IDLE
        self.round_count = 0  # 已完成往返次数
        
        # 目标速度和当前速度
        self.target_vx = 0.0
        self.target_wz = 0.0
        self.current_vx = 0.0
        self.current_wz = 0.0
        
        # 转身开始时的航向
        self.turn_start_heading = 0.0
        
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
        self.start_walk()
    
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
                            self.start_walk()
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
            self.get_logger().info("⏸️ 已暂停（手动模式）")
            self.target_vx = 0.0
            self.target_wz = 0.0
        else:
            self.get_logger().info("▶️ 从当前位置重新开始")
            sensor_data.record_start_position()
            self.state = WalkState.FORWARD
            self.round_count = 0
    
    def _reset(self):
        """重置状态"""
        global sensor_data
        sensor_data.reset()
        self.state = WalkState.IDLE
        self.round_count = 0
        self.target_vx = 0.0
        self.target_wz = 0.0
        self.get_logger().info("🔄 已重置")
    
    def start_walk(self):
        """开始直线行走"""
        global sensor_data
        
        if self.state != WalkState.IDLE and self.state != WalkState.ARRIVED:
            self.get_logger().info("已在行走中...")
            return
        
        # 等待传感器数据就绪
        self.get_logger().info("等待传感器数据...")
        wait_start = time.time()
        while not sensor_data.is_sensor_valid():
            time.sleep(0.1)
            if time.time() - wait_start > 3.0:
                self.get_logger().warn("⚠️ 传感器数据等待超时，继续尝试")
                break
        
        time.sleep(0.5)
        sensor_data.record_start_position()
        self.state = WalkState.FORWARD
        self.round_count = 0
        self.get_logger().info(f"🚀 开始直线行走! 距离: {CONFIG['LINE_DISTANCE']}m, 往返: {CONFIG['ROUND_TRIP_COUNT']}次")
    
    def _control_loop(self):
        """发送运动指令 - 直接设置速度，和 move.py 一致"""
        global controller_state
        interval = 1.0 / CONFIG["CONTROL_HZ"]
        
        while self.running and self.loco:
            try:
                # 手动模式或空闲状态：停止
                if controller_state.is_manual_mode or self.state == WalkState.IDLE:
                    self.loco.Move(0.0, 0.0, 0.0)
                else:
                    # 直接发送目标速度，无平滑过渡
                    self.loco.Move(self.target_vx, 0.0, self.target_wz)
                
                time.sleep(interval)
            except Exception as e:
                self.get_logger().error(f"控制错误: {e}")
                time.sleep(0.1)
    
    def _walk_loop(self):
        """行走主逻辑"""
        global sensor_data, controller_state
        
        last_print = 0
        
        while self.running:
            try:
                # 手动模式：暂停
                if controller_state.is_manual_mode:
                    time.sleep(0.1)
                    continue
                
                if self.state == WalkState.IDLE or self.state == WalkState.ARRIVED:
                    time.sleep(0.1)
                    continue
                
                # 传感器检查（仅警告，不停止，和 circle_walk.py 一致）
                # if not sensor_data.is_sensor_valid():
                #     self.get_logger().warn("⚠️ 传感器数据可能过期")
                #     # 不停止，继续使用最近的数据
                
                # 获取状态
                distance = sensor_data.get_distance_from_start()
                heading_error = sensor_data.get_heading_error()
                reverse_heading_error = sensor_data.get_reverse_heading_error()
                
                # ========== 状态机 ==========
                if self.state == WalkState.FORWARD:
                    # 前进：航向保持 + 距离跟踪
                    if distance >= CONFIG["LINE_DISTANCE"] - CONFIG["DISTANCE_TOLERANCE"]:
                        self.get_logger().info(f"📍 到达目标点! 距离: {distance:.2f}m")
                        
                        if CONFIG["RETURN_MODE"] == "one_way":
                            # 单向模式：到达目标点后停止，不返回
                            self.round_count += 1
                            self.get_logger().info(f"🎯 单向模式完成! 往返 {self.round_count}/{CONFIG['ROUND_TRIP_COUNT']}")
                            if self.round_count >= CONFIG["ROUND_TRIP_COUNT"]:
                                self.get_logger().info(f"🎉 全部 {CONFIG['ROUND_TRIP_COUNT']} 次前进完成!")
                                self.state = WalkState.ARRIVED
                            else:
                                # 重置起始位置，继续下一次
                                sensor_data.record_start_position()
                                self.get_logger().info(f"🔄 准备第 {self.round_count+1} 次前进...")
                            self.target_vx = 0.0
                            self.target_wz = 0.0
                            time.sleep(0.5)
                            continue
                        elif CONFIG["RETURN_MODE"] == "backward":
                            # 倒退模式：直接进入倒退状态，不转身
                            self.state = WalkState.BACKWARD
                            self.get_logger().info("⬅️ 倒退模式：开始倒退返回")
                        else:
                            # 转身模式：先转身180°
                            self.state = WalkState.TURNING
                            self.turn_start_heading = sensor_data.odom_theta
                            self.get_logger().info("🔄 转身模式：开始转身180°")
                        
                        self.target_vx = 0.0
                        self.target_wz = 0.0
                        time.sleep(0.5)  # 短暂停顿
                        continue
                    
                    # 前进 + 航向校正
                    self.target_vx = CONFIG["LINE_SPEED"]
                    correction = CONFIG["HEADING_KP"] * heading_error
                    self.target_wz = max(-CONFIG["MAX_CORRECTION_WZ"],
                                         min(CONFIG["MAX_CORRECTION_WZ"], correction))
                
                elif self.state == WalkState.TURNING:
                    # 转身180度
                    if abs(reverse_heading_error) < CONFIG["TURN_TOLERANCE"]:
                        self.get_logger().info("🔄 转身完成!")
                        self.state = WalkState.BACKWARD
                        self.target_wz = 0.0
                        time.sleep(0.3)
                        continue
                    
                    # 原地转向
                    self.target_vx = 0.0
                    self.target_wz = CONFIG["TURN_SPEED"] if reverse_heading_error > 0 else -CONFIG["TURN_SPEED"]
                
                elif self.state == WalkState.BACKWARD:
                    # 返回：距离跟踪
                    if distance < CONFIG["ARRIVAL_TOLERANCE"]:
                        self.round_count += 1
                        self.get_logger().info(f"✅ 返回原点! 往返 {self.round_count}/{CONFIG['ROUND_TRIP_COUNT']}")
                        
                        if self.round_count >= CONFIG["ROUND_TRIP_COUNT"]:
                            self.get_logger().info(f"🎉 全部 {CONFIG['ROUND_TRIP_COUNT']} 次往返完成!")
                            self.state = WalkState.ARRIVED
                            self.target_vx = 0.0
                            self.target_wz = 0.0
                        else:
                            # 继续下一次往返
                            if CONFIG["RETURN_MODE"] == "backward":
                                # 倒退模式：直接前进
                                self.state = WalkState.FORWARD
                            else:
                                # 转身模式：需要转身
                                self.state = WalkState.TURNING
                            self.get_logger().info(f"🔄 准备第 {self.round_count+1} 次往返...")
                        continue
                    
                    if CONFIG["RETURN_MODE"] == "backward":
                        # 倒退模式：负速度 + 航向校正
                        # 注意：倒退时角速度校正方向要反转！
                        self.target_vx = -CONFIG["LINE_SPEED"]
                        correction = CONFIG["HEADING_KP"] * heading_error
                        # 关键修复：倒退时校正方向取反
                        self.target_wz = -max(-CONFIG["MAX_CORRECTION_WZ"],
                                              min(CONFIG["MAX_CORRECTION_WZ"], correction))
                    else:
                        # 转身模式：正向前进 + 反向航向校正
                        self.target_vx = CONFIG["LINE_SPEED"]
                        correction = CONFIG["HEADING_KP"] * reverse_heading_error
                        self.target_wz = max(-CONFIG["MAX_CORRECTION_WZ"],
                                             min(CONFIG["MAX_CORRECTION_WZ"], correction))
                
                # 打印状态
                current_time = time.time()
                if current_time - last_print > 2.0:
                    state_emoji = {
                        WalkState.FORWARD: "➡️ 前进",
                        WalkState.TURNING: "🔄 转身",
                        WalkState.BACKWARD: "⬅️ 返回",
                    }
                    self.get_logger().info(
                        f"[{state_emoji.get(self.state, self.state)}] "
                        f"距离: {distance:.2f}m | "
                        f"航向误差: {math.degrees(heading_error):.1f}° | "
                        f"往返: {self.round_count}/{CONFIG['ROUND_TRIP_COUNT']}"
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
        self.state = WalkState.IDLE
        if self.loco:
            for _ in range(20):
                self.loco.Move(0.0, 0.0, 0.0)
                time.sleep(0.01)
        self.get_logger().info("✓ 已停止")


def main(args=None):
    mode_names = {
        "backward": "倒退返回",
        "turn": "转身返回",
        "one_way": "单向模式(只前进)"
    }
    return_mode = mode_names.get(CONFIG["RETURN_MODE"], CONFIG["RETURN_MODE"])
    
    print("=" * 60)
    print("  🤖 鲁棒闭环直线往返系统 V1.2")
    print("=" * 60)
    print(f"  行走距离: {CONFIG['LINE_DISTANCE']} m")
    print(f"  行走速度: {CONFIG['LINE_SPEED']} m/s")
    print(f"  返回模式: {return_mode}")
    print(f"  往返次数: {CONFIG['ROUND_TRIP_COUNT']}")
    print("=" * 60)
    print("  🎮 手柄: LT+RT+↑ 暂停/继续")
    print("  ⌨️ 键盘: 's'=开始 'm'=暂停/继续 'r'=重置 'q'=退出")
    print("  🚀 启动后自动开始!")
    print("=" * 60)
    
    rclpy.init(args=args)
    node = RobustLineWalk()
    
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
