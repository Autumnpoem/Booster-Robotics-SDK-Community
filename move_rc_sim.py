#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# V22.0: 圆弧行走模式 - 支持手柄切换左右转圈
# 作者：He
# 日期：2025-12-28
'''
打开robowork运行这个，点击Termianl(每次更换电池后需要重新运行)
cd /Downloads/booster_robotics_sdk
sudo bash -c "source /opt/ros/humble/setup.bash && python3 move_rc_sim.py"

如个卡死可以运行这个指令关闭这个程序
sudo pkill -9 -f move_rc_sim.py
'''
# 手柄操作流程:
# 1. LT + RT + 十字键上 → 激活手动模式（暂停自动行走）
# 2. LT + RT + 十字键左 → 切换到左圈模式
# 3. LT + RT + 十字键右 → 切换到右圈模式  
# 4. LT + RT + 十字键上 → 恢复自动模式，开始走选定方向的圈

import rclpy
from rclpy.node import Node
import time
import sys
import signal
import threading

# 导入 SDK
try:
    from booster_robotics_sdk_python import (
        B1LocoClient, 
        ChannelFactory,
        B1LowStateSubscriber
    )
    SDK_AVAILABLE = True
except ImportError:
    print("❌ Error: booster_robotics_sdk_python not found.")
    SDK_AVAILABLE = False
    sys.exit(1)

# 尝试导入手柄状态订阅器和发布器
try:
    from booster_robotics_sdk_python import (
        B1RemoteControllerStateSubscriber,
        B1RemoteControllerStatePublisher,
        RemoteControllerState
    )
    RC_SUBSCRIBER_AVAILABLE = True
    RC_PUBLISHER_AVAILABLE = True
except ImportError:
    RC_SUBSCRIBER_AVAILABLE = False
    RC_PUBLISHER_AVAILABLE = False
    print("⚠️ B1RemoteControllerStateSubscriber/Publisher 不可用，将使用键盘切换模式")

# 导入 Rerun (可选，用于状态显示)
try:
    import rerun as rr
    import rerun.blueprint as rrb
    RERUN_INSTALLED = True
except ImportError:
    RERUN_INSTALLED = False
    print("⚠️ Rerun SDK not installed. Visualization disabled.")

# ================= 配置参数 =================
CONFIG = {
    "RERUN_IP": "192.168.30.99",    
    "RERUN_PORT": 9876,

    # ============ 遥控器模拟参数（核心！）============
    # 左摇杆 ly：前进/后退（负值=前进，正值=后退）
    # 右摇杆 rx：左/右转向（负值=左转，正值=右转）
    # 值范围：-1.0 到 1.0
    "JOYSTICK_LY_FORWARD": -1.0,    # 左摇杆前进（推满=-1.0）
    "JOYSTICK_RX_TURN": 1.0,        # 右摇杆转向强度（0.0~1.0）
    
    # 圆弧行走时间
    "CIRCLE_DURATION": 5000.0,      # 每个圆弧持续时间（秒）
    "PAUSE_BETWEEN": 0,             # 动作间暂停时间
    
    # 预热直走参数 - 每次从暂停恢复或模式切换后，先直走一段时间建立速度，避免原地转圈
    "WARMUP_STRAIGHT_TIME": 0.8,    # 预热直走时间（秒）
    
    # 控制频率
    "CONTROL_HZ": 30,               # 控制频率（Hz）
}

# =================== 选择运动模式 ===================
# 直接修改这里选择模式，然后运行程序：
#   "square"          - 正方形行走
#   "left_circle"     - 走左圈（向左前方弧线行走）
#   "right_circle"    - 走右圈（向右前方弧线行走）
#   "left_turn_right" - 左圈 → 180°转身 → 右圈

WALK_MODE = "left_circle"  # ← 在这里修改模式！

# ===========================================


class ControllerState:
    """
    手柄状态类 - 用于存储从SDK读取的手柄状态
    
    操作流程:
    1. LT + RT + 十字键上 → 激活/恢复 手动/自动模式
    2. LT + RT + 十字键左 → 切换到左圈
    3. LT + RT + 十字键右 → 切换到右圈
    4. LT + RT + 十字键下 → 切换到直走
    """
    
    def __init__(self):
        self.is_manual_mode = False
        self.last_toggle_time = 0
        self.toggle_cooldown = 0.5  # 防抖时间（秒）
        
        # 手柄按键状态（用于组合键检测）
        self.lt_pressed = False
        self.rt_pressed = False
        self.hat_up_pressed = False
        self.hat_left_pressed = False   # 十字键左
        self.hat_right_pressed = False  # 十字键右
        self.hat_down_pressed = False   # 十字键下
        
        # 其他按键状态（从SDK读取，保留完整状态）
        self.lb_pressed = False
        self.rb_pressed = False
        self.a_pressed = False
        self.b_pressed = False
        self.x_pressed = False
        self.y_pressed = False
        
        # 摇杆状态
        self.lx = 0.0
        self.ly = 0.0
        self.rx = 0.0
        self.ry = 0.0
        
        # 手动控制速度
        self.manual_vx = 0.0
        self.manual_vy = 0.0
        self.manual_vyaw = 0.0
        
        # 数据更新时间
        self.last_update = 0
        
        # 手柄活动检测 - 用于暂停模拟发送，避免冲突
        self.last_gamepad_activity = 0
        self.gamepad_quiet_time = 0.5  # 手柄活动后暂停发送的时间（秒）
        
        # 发布状态标志 - 正在发布模拟数据时简化SDK回调处理
        self.is_publishing = False
        
        # SDK操作锁 - 确保回调和发布不同时执行
        self.sdk_lock = threading.Lock()
        
        # Debug模式
        self.debug_enabled = True
        self.last_debug_time = 0
        self.debug_interval = 0.2
        
        # 用于释放检测的状态
        self.combo_was_pressed = False
        self.combo_left_was_pressed = False   # 左键组合曾按下
        self.combo_right_was_pressed = False  # 右键组合曾按下
        self.combo_down_was_pressed = False   # 下键组合曾按下
        
        # 运动模式控制: "left", "right", "straight"
        self.circle_direction = "left"  # 默认左圈
        self.direction_changed = False  # 标记方向是否刚切换
        
        # 使用 RLock（可重入锁）避免嵌套调用死锁
        self.lock = threading.RLock()
    
    def check_toggle_combo(self, lt, rt, hat_up):
        """
        检测 LT + RT + 十字上 组合键
        触发方式：组合键全部按下后，释放时触发切换
        返回是否触发切换
        """
        current_time = time.time()
        
        with self.lock:
            combo_now_pressed = lt and rt and hat_up
            
            if self.debug_enabled and (current_time - self.last_debug_time >= self.debug_interval):
                if (lt or rt or hat_up) or self.combo_was_pressed:
                    print(f"[DEBUG 按键] LT={lt}, RT={rt}, 十字上={hat_up} | "
                          f"组合键完整={combo_now_pressed}, 曾完整按下={self.combo_was_pressed} | "
                          f"当前模式={'手动' if self.is_manual_mode else '自动'} | "
                          f"圈方向={self.circle_direction}")
                    self.last_debug_time = current_time
            
            self.lt_pressed = lt
            self.rt_pressed = rt
            self.hat_up_pressed = hat_up
            
            if combo_now_pressed:
                if not self.combo_was_pressed:
                    print(f"[DEBUG] ✓ 组合键已完整按下 (LT+RT+十字上)，等待释放...")
                self.combo_was_pressed = True
                return False
            
            if self.combo_was_pressed and not combo_now_pressed:
                self.combo_was_pressed = False
                
                if current_time - self.last_toggle_time > self.toggle_cooldown:
                    self.last_toggle_time = current_time
                    self.is_manual_mode = not self.is_manual_mode
                    print(f"[DEBUG] ★ 组合键释放！触发模式切换 → {'手动' if self.is_manual_mode else '自动'}")
                    if not self.is_manual_mode:
                        # 恢复自动模式时，标记方向已改变以触发重新开始
                        self.direction_changed = True
                    return True
                else:
                    print(f"[DEBUG] ⚠️ 组合键释放但在防抖时间内，忽略")
            
            return False
    
    def check_direction_switch(self, lt, rt, hat_left, hat_right, hat_down):
        """
        检测 LT + RT + 十字左/右/下 组合键
        只有在手动模式下才允许切换运动模式
        返回: (是否触发, 新方向)
        方向: "left", "right", "straight"
        """
        current_time = time.time()
        
        with self.lock:
            # 只有在手动模式下才能切换方向
            if not self.is_manual_mode:
                self.combo_left_was_pressed = False
                self.combo_right_was_pressed = False
                self.combo_down_was_pressed = False
                return (False, None)
            
            # 检测左键组合
            combo_left_now = lt and rt and hat_left and not hat_right and not hat_down
            # 检测右键组合
            combo_right_now = lt and rt and hat_right and not hat_left and not hat_down
            # 检测下键组合（直走）
            combo_down_now = lt and rt and hat_down and not hat_left and not hat_right
            
            # 更新按键状态
            self.hat_left_pressed = hat_left
            self.hat_right_pressed = hat_right
            self.hat_down_pressed = hat_down
            
            # 左键组合释放检测
            if combo_left_now:
                if not self.combo_left_was_pressed:
                    print(f"[DEBUG] ✓ 左键组合已按下 (LT+RT+十字左)，等待释放...")
                self.combo_left_was_pressed = True
            elif self.combo_left_was_pressed and not combo_left_now:
                self.combo_left_was_pressed = False
                if current_time - self.last_toggle_time > self.toggle_cooldown:
                    self.last_toggle_time = current_time
                    self.circle_direction = "left"
                    print(f"[DEBUG] ★ 切换到左圈模式！")
                    return (True, "left")
            
            # 右键组合释放检测
            if combo_right_now:
                if not self.combo_right_was_pressed:
                    print(f"[DEBUG] ✓ 右键组合已按下 (LT+RT+十字右)，等待释放...")
                self.combo_right_was_pressed = True
            elif self.combo_right_was_pressed and not combo_right_now:
                self.combo_right_was_pressed = False
                if current_time - self.last_toggle_time > self.toggle_cooldown:
                    self.last_toggle_time = current_time
                    self.circle_direction = "right"
                    print(f"[DEBUG] ★ 切换到右圈模式！")
                    return (True, "right")
            
            # 下键组合释放检测（直走）
            if combo_down_now:
                if not self.combo_down_was_pressed:
                    print(f"[DEBUG] ✓ 下键组合已按下 (LT+RT+十字下)，等待释放...")
                self.combo_down_was_pressed = True
            elif self.combo_down_was_pressed and not combo_down_now:
                self.combo_down_was_pressed = False
                if current_time - self.last_toggle_time > self.toggle_cooldown:
                    self.last_toggle_time = current_time
                    self.circle_direction = "straight"
                    print(f"[DEBUG] ★ 切换到直走模式！")
                    return (True, "straight")
            
            return (False, None)
    
    def update_from_sdk(self, msg):
        """
        从SDK手柄消息更新状态
        """
        current_time = time.time()
        
        with self.lock:
            prev_lt = self.lt_pressed
            prev_rt = self.rt_pressed
            prev_hat_up = self.hat_up_pressed
            prev_hat_left = self.hat_left_pressed
            prev_hat_right = self.hat_right_pressed
            prev_hat_down = self.hat_down_pressed
            
            self.lt_pressed = msg.lt
            self.lb_pressed = msg.lb
            self.rb_pressed = msg.rb
            self.rt_pressed = msg.rt
            self.hat_up_pressed = msg.hat_u
            self.hat_left_pressed = getattr(msg, 'hat_l', False)
            self.hat_right_pressed = getattr(msg, 'hat_r', False)
            self.hat_down_pressed = getattr(msg, 'hat_d', False)
            
            self.a_pressed = msg.a
            self.b_pressed = msg.b
            self.x_pressed = msg.x
            self.y_pressed = msg.y
            
            self.lx = msg.lx
            self.ly = msg.ly
            self.rx = msg.rx
            self.ry = msg.ry
            
            self.last_update = current_time
            
            # 检测手柄是否有任何按键活动
            any_button = msg.lt or msg.rt or msg.hat_u or self.hat_left_pressed or self.hat_right_pressed or self.hat_down_pressed
            if any_button:
                self.last_gamepad_activity = current_time
            
            if self.debug_enabled:
                key_changed = (prev_lt != msg.lt or prev_rt != msg.rt or 
                               prev_hat_up != msg.hat_u or 
                               prev_hat_left != self.hat_left_pressed or 
                               prev_hat_right != self.hat_right_pressed or
                               prev_hat_down != self.hat_down_pressed)
                if key_changed:
                    print(f"[SDK 回调] 按键变化: LT={msg.lt}, RT={msg.rt}, "
                          f"十字上={msg.hat_u}, 十字左={self.hat_left_pressed}, "
                          f"十字右={self.hat_right_pressed}, 十字下={self.hat_down_pressed}")
        
        # 先检查方向切换 (LT + RT + 十字左/右/下)
        direction_result = self.check_direction_switch(
            msg.lt, msg.rt, 
            getattr(msg, 'hat_l', False), 
            getattr(msg, 'hat_r', False),
            getattr(msg, 'hat_d', False)
        )
        
        # 再检查模式切换 (LT + RT + 十字上)
        mode_changed = self.check_toggle_combo(msg.lt, msg.rt, msg.hat_u)
        
        return mode_changed or direction_result[0]
    
    def set_manual_velocity(self, vx, vy, vyaw):
        """设置手动模式速度"""
        with self.lock:
            self.manual_vx = vx
            self.manual_vy = vy
            self.manual_vyaw = vyaw
    
    def get_manual_velocity(self):
        """获取手动模式速度"""
        with self.lock:
            if self.is_manual_mode:
                return (self.manual_vx, self.manual_vy, self.manual_vyaw)
            return None
    
    def get_circle_direction(self):
        """获取当前圈方向"""
        with self.lock:
            return self.circle_direction
    
    def check_and_clear_direction_changed(self):
        """检查并清除方向改变标志"""
        with self.lock:
            if self.direction_changed:
                self.direction_changed = False
                return True
            return False


# 全局手柄状态
controller_state = ControllerState()


class CircleWalkSystem(Node):
    def __init__(self):
        super().__init__('booster_circle_walk')
        self.get_logger().info(f"🤖 机器人行走系统初始化 (模式: {WALK_MODE})...")

        # 运动状态变量（保留兼容性，用于Move API降级）
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_wz = 0.0
        self.running = True
        self.is_moving = False
        self.current_action = "停止"
        
        # 遥控器模拟：直接使用摇杆值
        self.target_ly = 0.0   # 左摇杆Y（负值=前进）
        self.target_lx = 0.0   # 左摇杆X（侧移）
        self.target_rx = 0.0   # 右摇杆X（负值=左转）

        self._init_rerun()
        self._init_locomotion()
        self._init_keyboard_listener()

        # 创建控制线程
        self.control_thread = threading.Thread(target=self.control_loop, daemon=True)
        self.command_thread = threading.Thread(target=self.command_sequence, daemon=True)

        # 启动线程
        self.control_thread.start()
        time.sleep(0.5)
        self.command_thread.start()

        self.get_logger().info(f"✅ 机器人行走系统就绪，模式: {WALK_MODE}")
        self.get_logger().info("🎮 手柄控制: 按 LT + RT + 十字键上 切换自动/手动模式")
        self.get_logger().info("⌨️  键盘控制: 按 'm' 键也可切换模式 (在终端中)")
        
        # 初始化SDK手柄状态订阅
        self._init_gamepad_subscriber()

    def _init_rerun(self):
        """初始化Rerun可视化（可选）"""
        if not RERUN_INSTALLED: 
            return
        try:
            rr.init("Booster_K1_CircleWalk", spawn=False)
            addr = f"{CONFIG['RERUN_IP']}:{CONFIG['RERUN_PORT']}"
            rr.connect(addr)
            blueprint = rrb.Blueprint(
                rrb.TextLogView(origin="/log", name="状态日志"),
            )
            rr.send_blueprint(blueprint)
            self.get_logger().info("Rerun 连接成功.")
        except Exception as e:
            self.get_logger().warn(f"Rerun 连接失败: {e}")

    def _init_locomotion(self):
        """初始化运动控制"""
        try:
            ChannelFactory.Instance().Init(0, "127.0.0.1")
            self.loco = B1LocoClient()
            self.loco.Init()
            time.sleep(1)
            for _ in range(10):
                self.loco.Move(0.0, 0.0, 0.0)
                time.sleep(0.01)
            self.get_logger().info("✓ 运动控制初始化成功")
            
            # 初始化遥控器状态发布器（模拟左右摇杆分开控制）
            if RC_PUBLISHER_AVAILABLE:
                self.rc_publisher = B1RemoteControllerStatePublisher()
                self.rc_publisher.InitChannel()
                self.get_logger().info("✓ 遥控器模拟发布器初始化成功")
            else:
                self.rc_publisher = None
                self.get_logger().warn("遥控器模拟发布器不可用，使用 Move() API")
                
        except Exception as e:
            self.get_logger().error(f"运动控制初始化失败: {e}")
            self.loco = None
            self.rc_publisher = None

    def _init_gamepad_subscriber(self):
        """初始化SDK手柄状态订阅"""
        global controller_state
        
        if not RC_SUBSCRIBER_AVAILABLE:
            self.get_logger().warn("手柄订阅器不可用，仅支持键盘控制")
            return
        
        def gamepad_callback(msg):
            """手柄状态回调 - 使用非阻塞锁避免SDK冲突"""
            global controller_state
            
            # 非阻塞获取锁，如果获取失败直接跳过（正在发布中）
            if not controller_state.sdk_lock.acquire(blocking=False):
                return  # 正在发布，跳过此次回调
            
            try:
                # 记录手柄活动时间
                controller_state.last_gamepad_activity = time.time()
                
                # 如果正在发布模拟数据，只检测完整组合键
                if controller_state.is_publishing:
                    if msg.lt and msg.rt and msg.hat_u:
                        controller_state.combo_was_pressed = True
                    elif controller_state.combo_was_pressed:
                        controller_state.combo_was_pressed = False
                        controller_state.is_manual_mode = True
                        controller_state.is_publishing = False
                        print(f"🎮 切换到手动模式")
                    return
                
                # 手动模式：正常处理所有按键
                if controller_state.update_from_sdk(msg):
                    if controller_state.is_manual_mode:
                        mode_str = f"🎮 手动模式 (当前圈方向: {controller_state.get_circle_direction()})"
                        self.set_movement(0.0, 0.0, 0.0, "手动模式")
                    else:
                        mode_str = f"🤖 自动模式 (圈方向: {controller_state.get_circle_direction()})"
                    self.get_logger().info(f"[手柄] {mode_str}")
            finally:
                controller_state.sdk_lock.release()
        
        try:
            self.gamepad_subscriber = B1RemoteControllerStateSubscriber(gamepad_callback)
            self.gamepad_subscriber.InitChannel()
            self.get_logger().info("✓ SDK手柄状态订阅初始化成功")
        except Exception as e:
            self.get_logger().warn(f"手柄订阅初始化失败: {e}")

    def _init_keyboard_listener(self):
        """初始化键盘监听（用于调试，按 'm' 切换模式）"""
        def keyboard_thread():
            global controller_state
            import select
            
            print("\n[键盘控制] 在终端按 'm' 切换自动/手动模式, 'q' 退出")
            
            while self.running:
                try:
                    # 非阻塞检测键盘输入
                    if sys.stdin in select.select([sys.stdin], [], [], 0.1)[0]:
                        key = sys.stdin.read(1).lower()
                        if key == 'm':
                            # 模拟组合键触发
                            controller_state.check_toggle_combo(True, True, True)
                            mode_str = "🎮 手动控制" if controller_state.is_manual_mode else "🤖 自动模式"
                            self.get_logger().info(f"[键盘] 模式切换: {mode_str}")
                        elif key == 'q':
                            self.running = False
                            break
                except:
                    time.sleep(0.1)
        
        # 启动键盘监听线程
        self.keyboard_thread = threading.Thread(target=keyboard_thread, daemon=True)
        self.keyboard_thread.start()

    def control_loop(self):
        """
        持续发送运动指令的控制循环
        
        使用 RemoteControllerStatePublisher 模拟遥控器的左右摇杆分开控制：
        - 左摇杆 ly = 前进速度（-1.0 到 1.0）
        - 右摇杆 rx = 转向（-1.0 到 1.0）
        
        这样完全模拟"左摇杆推满 + 右摇杆只打转向"的操控方式！
        """
        global controller_state
        control_interval = 1.0 / CONFIG["CONTROL_HZ"]
        last_debug = 0

        while self.running and self.loco is not None:
            try:
                # 检查是否为手动模式
                if controller_state.is_manual_mode:
                    # 手动模式：不发送任何指令，让遥控器直接控制机器人
                    time.sleep(control_interval)
                    continue
                
                # 检测手柄是否有活动，如果有则暂停发送模拟数据，避免SDK冲突
                time_since_gamepad = time.time() - controller_state.last_gamepad_activity
                if time_since_gamepad < controller_state.gamepad_quiet_time:
                    # 手柄刚有活动，暂停发送，避免冲突
                    time.sleep(control_interval)
                    continue
                
                # 自动模式：使用 RemoteControllerStatePublisher 模拟遥控器输入
                if self.rc_publisher is not None:
                    # 获取SDK锁，确保不与回调冲突
                    with controller_state.sdk_lock:
                        # 标记正在发布
                        controller_state.is_publishing = True
                        
                        # 创建遥控器状态消息
                        rc_msg = RemoteControllerState()
                        
                        # 关键：设置event=1536，这是真实遥控器使用的值
                        rc_msg.event = 1536
                        
                        # 直接使用摇杆值
                        rc_msg.ly = self.target_ly
                        rc_msg.lx = self.target_lx
                        rc_msg.rx = self.target_rx
                        rc_msg.ry = 0.0
                        
                        # 发布遥控器状态
                        self.rc_publisher.Write(rc_msg)
                    
                    # 调试输出
                    now = time.time()
                    if now - last_debug > 2.0:
                        print(f"[遥控器模拟] ly={self.target_ly:.2f}(前进), rx={self.target_rx:.2f}(转向)")
                        last_debug = now
                else:
                    # 如果遥控器发布器不可用，降级使用 Move() API
                    self.loco.Move(self.current_vx, self.current_vy, self.current_wz)
                    
                    now = time.time()
                    if now - last_debug > 2.0:
                        print(f"[Move API] vx={self.current_vx:.2f}, wz={self.current_wz:.2f}")
                        last_debug = now

                time.sleep(control_interval)

            except Exception as e:
                self.get_logger().error(f"发送指令失败: {e}")
                time.sleep(0.1)

    def set_movement(self, vx, vy, wz, action_name):
        """设置运动参数"""
        global controller_state
        
        # 如果是手动模式，跳过自动设置
        if controller_state.is_manual_mode:
            return
            
        self.current_vx = vx
        self.current_vy = vy
        self.current_wz = wz
        self.current_action = action_name
        self.is_moving = (vx != 0 or vy != 0 or wz != 0)

        timestamp = time.strftime("%H:%M:%S")
        log_msg = f"[{timestamp}] {action_name}: vx={vx:.2f}, vy={vy:.2f}, wz={wz:.2f}"
        self.get_logger().info(log_msg)

        if RERUN_INSTALLED:
            color = [0, 255, 0] if action_name != "停止" else [255, 0, 0]
            rr.log("world/status", rr.TextLog(log_msg, color=color))

    def wait_duration(self, duration, action_name):
        """
        等待指定时间
        返回: True 如果方向改变需要重新开始, False 正常完成
        
        修复：手动模式时直接返回，让上层循环处理，避免死循环
        """
        global controller_state
        start_time = time.time()

        while self.running and (time.time() - start_time) < duration:
            # 手动模式：立即返回，让上层循环处理
            # 不要在这里死等，避免死循环
            if controller_state.is_manual_mode:
                return True  # 返回 True 让上层重新开始
            
            # 检查方向是否改变
            if controller_state.check_and_clear_direction_changed():
                self.get_logger().info(f"🔄 方向已切换，重新开始...")
                return True
            
            time.sleep(0.1)
        
        return False

    def command_sequence(self):
        """根据模式执行对应的运动序列"""
        if WALK_MODE == "square":
            self._run_square_sequence()
        elif WALK_MODE == "left_circle" or WALK_MODE == "right_circle":
            # 使用手柄可切换方向的圈行走
            self._run_dynamic_circle_sequence()
        elif WALK_MODE == "left_turn_right":
            self._run_left_turn_right_sequence()
        else:
            self.get_logger().error(f"未知的行走模式: {WALK_MODE}")

    def _run_dynamic_circle_sequence(self):
        """
        动态行走 - 可通过手柄切换左圈/右圈/直走
        
        新增：每次从暂停恢复后，先直走一段时间（WARMUP_STRAIGHT_TIME）建立速度，
        然后再开始圆弧行走，避免直接推斜杆导致原地转圈的问题。
        
        操作流程:
        1. LT+RT+上键 → 暂停（手动模式）
        2. LT+RT+左键 → 切换到左圈
        3. LT+RT+右键 → 切换到右圈
        4. LT+RT+下键 → 切换到直走
        5. LT+RT+上键 → 继续走（自动模式）→ 先直走预热 → 再走圆弧
        
        使用遥控器模拟：
        - 左摇杆 ly = -1.0 (推满前进)
        - 右摇杆 rx = ±0.8 (左转/右转)
        - 预热机制：先只发 ly，建立前进速度后再加入 rx
        """
        global controller_state
        sequence_count = 0
        
        # 根据初始WALK_MODE设置方向
        if WALK_MODE == "right_circle":
            controller_state.circle_direction = "right"

        while self.running:
            # 检查是否在手动模式
            if controller_state.is_manual_mode:
                # 手动模式：停止发送，让遥控器直接控制
                self.target_ly = 0.0
                self.target_rx = 0.0
                time.sleep(0.1)
                continue
            
            # 清除可能残留的方向改变标志
            controller_state.check_and_clear_direction_changed()
            
            # 获取当前运动模式
            direction = controller_state.get_circle_direction()
            sequence_count += 1
            
            # 根据模式设置摇杆值
            ly_forward = CONFIG["JOYSTICK_LY_FORWARD"]  # 左摇杆前进值
            rx_turn = CONFIG["JOYSTICK_RX_TURN"]        # 右摇杆转向强度
            
            if direction == "left":
                direction_name = "左圈"
                self.target_ly = ly_forward   # 左摇杆推满前进
                self.target_lx = 0.0          # 不侧移
                self.target_rx = -rx_turn     # 右摇杆左转（负值）
            elif direction == "right":
                direction_name = "右圈"
                self.target_ly = ly_forward   # 左摇杆推满前进
                self.target_lx = 0.0          # 不侧移
                self.target_rx = rx_turn      # 右摇杆右转（正值）
            else:  # straight
                direction_name = "直走"
                self.target_ly = ly_forward   # 左摇杆推满前进
                self.target_lx = 0.0          # 不侧移
                self.target_rx = 0.0          # 不转向
            
            self.get_logger().info(f"=== 开始第 {sequence_count} 个{direction_name} ===")

            try:
                # ============ 预热直走阶段（关键！）============
                # 如果不是直走模式，先只发 ly（前进），不发 rx（转向）
                # 建立前进速度后再加入转向，模拟真实手柄操作，避免原地转圈
                warmup_time = CONFIG.get("WARMUP_STRAIGHT_TIME", 0.8)
                if direction != "straight" and warmup_time > 0:
                    self.get_logger().info(f">>> 预热阶段: 直走 {warmup_time} 秒建立速度...")
                    
                    # 预热时：只设置前进，不设置转向
                    self.target_ly = ly_forward   # 左摇杆推满前进
                    self.target_lx = 0.0          # 不侧移
                    self.target_rx = 0.0          # 先不转向！
                    
                    # 等待预热时间
                    if self.wait_duration(warmup_time, "预热直走"):
                        self.get_logger().info(f"🔄 预热期间检测到模式切换，重新开始...")
                        continue  # 立即重新开始循环
                    
                    if not self.running: break
                    self.get_logger().info(f">>> 预热完成，开始{direction_name}...")
                
                # ============ 正式圆弧行走阶段 ============
                # 现在再加入转向
                if direction == "left":
                    self.target_ly = ly_forward   # 左摇杆推满前进
                    self.target_lx = 0.0          # 不侧移
                    self.target_rx = -rx_turn     # 右摇杆左转（负值）
                elif direction == "right":
                    self.target_ly = ly_forward   # 左摇杆推满前进
                    self.target_lx = 0.0          # 不侧移
                    self.target_rx = rx_turn      # 右摇杆右转（正值）
                # straight 模式之前已经设置好了
                
                self.get_logger().info(f"[遥控器模拟] ly={self.target_ly:.1f}, rx={self.target_rx:.1f}")
                
                # 等待指定时间，期间持续发送摇杆值
                # wait_duration 返回 True 表示方向改变，需要立即重新开始
                if self.wait_duration(CONFIG["CIRCLE_DURATION"], direction_name):
                    self.get_logger().info(f"🔄 检测到模式切换，重新开始...")
                    continue  # 立即重新开始循环，使用新方向
                    
                if not self.running: break

                # 停止
                self.target_ly = 0.0
                self.target_rx = 0.0
                time.sleep(CONFIG["PAUSE_BETWEEN"])

                self.get_logger().info(f"=== 第 {sequence_count} 个{direction_name}完成 ===")
                time.sleep(2.0)

            except KeyboardInterrupt:
                break
            except Exception as e:
                self.get_logger().error(f"命令序列错误: {e}")
                self.target_ly = 0.0
                self.target_rx = 0.0
                time.sleep(1.0)

    def _run_left_circle_sequence(self):
        """走左圈 (保留兼容性)"""
        self._run_dynamic_circle_sequence()

    def _run_right_circle_sequence(self):
        """走右圈 (保留兼容性)"""
        global controller_state
        controller_state.circle_direction = "right"
        self._run_dynamic_circle_sequence()

    def _run_left_turn_right_sequence(self):
        """左圈 -> 180度转身 -> 右圈"""
        global controller_state
        sequence_count = 0

        while self.running:
            if controller_state.is_manual_mode:
                time.sleep(0.1)
                continue
                
            sequence_count += 1
            self.get_logger().info(f"=== 开始第 {sequence_count} 次 左圈-转身-右圈 ===")

            try:
                # 1. 走左圈
                self.get_logger().info(">>> 阶段1: 走左圈")
                self.set_movement(
                    vx=CONFIG["CIRCLE_FORWARD_SPEED"],
                    vy=0.0,
                    wz=CONFIG["CIRCLE_TURN_SPEED"],
                    action_name="左前走圈"
                )
                self.wait_duration(CONFIG["CIRCLE_DURATION"], "左前走圈")
                if not self.running: break

                self.set_movement(0.0, 0.0, 0.0, "暂停")
                time.sleep(CONFIG["PAUSE_BETWEEN"])
                if not self.running: break

                # 2. 转身180度
                self.get_logger().info(">>> 阶段2: 原地转身180度")
                self.set_movement(
                    vx=0.0, vy=0.0, wz=CONFIG["TURN_180_SPEED"],
                    action_name="转身180度"
                )
                self.wait_duration(CONFIG["TURN_180_DURATION"], "转身180度")
                if not self.running: break

                self.set_movement(0.0, 0.0, 0.0, "暂停")
                time.sleep(CONFIG["PAUSE_BETWEEN"])
                if not self.running: break

                # 3. 走右圈
                self.get_logger().info(">>> 阶段3: 走右圈")
                self.set_movement(
                    vx=CONFIG["CIRCLE_FORWARD_SPEED"],
                    vy=0.0,
                    wz=-CONFIG["CIRCLE_TURN_SPEED"],
                    action_name="右前走圈"
                )
                self.wait_duration(CONFIG["CIRCLE_DURATION"], "右前走圈")
                if not self.running: break

                self.get_logger().info(f"=== 第 {sequence_count} 次组合动作完成 ===")
                self.set_movement(0.0, 0.0, 0.0, "停止")
                time.sleep(2.0)

            except KeyboardInterrupt:
                break
            except Exception as e:
                self.get_logger().error(f"命令序列错误: {e}")
                self.set_movement(0.0, 0.0, 0.0, "错误停止")
                time.sleep(1.0)

    def _run_square_sequence(self):
        """正方形行走"""
        global controller_state
        sequence_count = 0

        while self.running:
            if controller_state.is_manual_mode:
                time.sleep(0.1)
                continue
                
            sequence_count += 1
            self.get_logger().info(f"=== 开始第 {sequence_count} 个正方形 ===")

            try:
                # 1. 前进
                self.set_movement(vx=CONFIG["SPEED_FORWARD"], vy=0.0, wz=0.0, action_name="前进")
                self.wait_duration(CONFIG["FORWARD_TIME"], "前进")
                if not self.running: break

                self.set_movement(0.0, 0.0, 0.0, "暂停")
                time.sleep(CONFIG["PAUSE_BETWEEN"])
                if not self.running: break

                # 2. 右侧走
                self.set_movement(vx=0.0, vy=CONFIG["SPEED_SIDEWAYS"], wz=0.0, action_name="右侧走")
                self.wait_duration(CONFIG["RIGHT_TIME"], "右侧走")
                if not self.running: break

                self.set_movement(0.0, 0.0, 0.0, "暂停")
                time.sleep(CONFIG["PAUSE_BETWEEN"])
                if not self.running: break

                # 3. 后退
                self.set_movement(vx=CONFIG["SPEED_BACKWARD"], vy=0.0, wz=0.0, action_name="后退")
                self.wait_duration(CONFIG["BACKWARD_TIME"], "后退")
                if not self.running: break

                self.set_movement(0.0, 0.0, 0.0, "暂停")
                time.sleep(CONFIG["PAUSE_BETWEEN"])
                if not self.running: break

                # 4. 左侧走
                self.set_movement(vx=0.0, vy=-CONFIG["SPEED_SIDEWAYS"], wz=0.0, action_name="左侧走")
                self.wait_duration(CONFIG["LEFT_TIME"], "左侧走")
                if not self.running: break

                self.get_logger().info(f"=== 第 {sequence_count} 个正方形完成 ===")
                self.set_movement(0.0, 0.0, 0.0, "停止")
                time.sleep(2.0)

            except KeyboardInterrupt:
                break
            except Exception as e:
                self.get_logger().error(f"命令序列错误: {e}")
                self.set_movement(0.0, 0.0, 0.0, "错误停止")
                time.sleep(1.0)

    def stop(self):
        """停止机器人"""
        self.get_logger().info("正在停止机器人...")
        self.running = False

        if self.loco is not None:
            try:
                for _ in range(20):
                    self.loco.Move(0.0, 0.0, 0.0)
                    time.sleep(0.01)
                self.get_logger().info("✓ 机器人已停止")
            except Exception as e:
                self.get_logger().error(f"停止指令发送失败: {e}")

        # 等待线程结束
        if hasattr(self, 'control_thread') and self.control_thread.is_alive():
            self.control_thread.join(timeout=2.0)
        if hasattr(self, 'command_thread') and self.command_thread.is_alive():
            self.command_thread.join(timeout=2.0)


def toggle_mode():
    """
    外部调用此函数可切换自动/手动模式
    可在notebook中直接调用：toggle_mode()
    """
    global controller_state
    controller_state.check_toggle_combo(True, True, True)
    mode_str = "手动控制" if controller_state.is_manual_mode else "自动模式"
    print(f"模式切换: {mode_str}")
    return controller_state.is_manual_mode


def set_manual_mode(enable=True):
    """
    直接设置手动模式
    enable=True: 切换到手动模式
    enable=False: 切换到自动模式
    """
    global controller_state
    controller_state.is_manual_mode = enable
    mode_str = "手动控制" if enable else "自动模式"
    print(f"模式设置: {mode_str}")


def get_current_mode():
    """获取当前模式"""
    global controller_state
    return "手动" if controller_state.is_manual_mode else "自动"


def main(args=None):
    print("=" * 60)
    print("  🤖 Booster Robotics 圆弧行走系统 V22.1")
    print("=" * 60)
    print(f"  行走模式: {WALK_MODE}")
    print("  ")
    print("  🎮 手柄操作说明:")
    print("  ─────────────────────────────────────")
    print("  LT + RT + 十字键上  → 暂停/继续 (切换手动/自动模式)")
    print("  LT + RT + 十字键左  → 切换到左圈 (需先暂停)")
    print("  LT + RT + 十字键右  → 切换到右圈 (需先暂停)")
    print("  LT + RT + 十字键下  → 切换到直走 (需先暂停)")
    print("  ─────────────────────────────────────")
    print(f"  ⌨️  键盘切换: 按 'm' 键")
    if RC_SUBSCRIBER_AVAILABLE:
        print(f"  ✓ SDK手柄订阅: 已启用")
    else:
        print(f"  ⚠️ SDK手柄订阅: 不可用（仅键盘控制）")
    print("=" * 60)
    
    # 全局节点引用，用于信号处理
    node = None
    
    def signal_handler(signum, frame):
        """信号处理函数 - 强制退出"""
        print("\n⚠️ 接收到 Ctrl+C，强制退出...")
        if node is not None:
            node.running = False
        # 强制退出，避免死锁
        import os
        os._exit(0)
    
    # 注册信号处理
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    rclpy.init(args=args)
    node = CircleWalkSystem()

    try:
        while node.running:
            rclpy.spin_once(node, timeout_sec=0.1)
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n接收到停止信号...")
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()
        print("✅ 系统已关闭")


if __name__ == '__main__':
    main()
