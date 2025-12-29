#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# V17.1: 正方形行走模式 - 持续发送指令版本

import rclpy
from rclpy.node import Node
import time
import sys
import threading

# 🚨 关键修复：将 'robototics' 修正为 'robotics'
try:
    from booster_robotics_sdk_python import B1LocoClient, ChannelFactory
except ImportError:
    print("❌ Error: SDK not found.")
    sys.exit(1)

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

    # 正方形行走参数（单位：秒）
    "FORWARD_TIME": 3.0,      # 前进3秒
    "RIGHT_TIME": 3.0,        # 向右走3秒
    "BACKWARD_TIME": 3.0,     # 后退3秒
    "LEFT_TIME": 3.0,         # 向左走3秒
    "PAUSE_BETWEEN": 0.5,     # 动作间暂停时间

    # 速度参数
    "SPEED_FORWARD": 0.3,     # 前进速度
    "SPEED_SIDEWAYS": 0.3,    # 侧向移动速度
    "SPEED_BACKWARD": -0.3,   # 后退速度

    # 控制频率
    "CONTROL_HZ": 20,         # 控制频率（Hz）
}
# ===========================================

class SquareWalkSystem(Node):
    def __init__(self):
        super().__init__('booster_square_walk')
        self.get_logger().info("🤖 正方形行走系统初始化 (V17.1 - 持续指令版)...")

        # 运动状态变量
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_wz = 0.0
        self.running = True
        self.is_moving = False
        self.current_action = "停止"

        self._init_rerun()
        self._init_locomotion()

        # 创建控制线程
        self.control_thread = threading.Thread(target=self.control_loop, daemon=True)
        self.command_thread = threading.Thread(target=self.command_sequence, daemon=True)

        # 启动线程
        self.control_thread.start()
        time.sleep(0.5)  # 等待控制线程启动
        self.command_thread.start()

        self.get_logger().info("✅ 正方形行走系统就绪，开始执行...")

    def _init_rerun(self):
        """初始化Rerun可视化（可选）"""
        if not RERUN_INSTALLED: 
            return
        try:
            rr.init("Booster_K1_SquareWalk", spawn=False)
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
            # 初始发送停止指令
            for _ in range(10):  # 连续发送多次确保停止
                self.loco.Move(0.0, 0.0, 0.0)
                time.sleep(0.01)
            self.get_logger().info("✓ 运动控制初始化成功.")
        except Exception as e:
            self.get_logger().error(f"运动控制初始化失败: {e}")
            self.loco = None

    def control_loop(self):
        """持续发送运动指令的控制循环"""
        control_interval = 1.0 / CONFIG["CONTROL_HZ"]

        while self.running and self.loco is not None:
            try:
                # 持续发送当前运动指令
                self.loco.Move(self.current_vx, self.current_vy, self.current_wz)

                # 记录发送频率（每秒一次）
                current_time = time.time()
                if hasattr(self, 'last_debug_time'):
                    if current_time - self.last_debug_time >= 1.0:
                        self.get_logger().debug(
                            f"发送指令: vx={self.current_vx:.2f}, "
                            f"vy={self.current_vy:.2f}, "
                            f"wz={self.current_wz:.2f}, "
                            f"动作: {self.current_action}"
                        )
                        self.last_debug_time = current_time
                else:
                    self.last_debug_time = current_time

                time.sleep(control_interval)

            except Exception as e:
                self.get_logger().error(f"发送指令失败: {e}")
                time.sleep(0.1)

    def set_movement(self, vx, vy, wz, action_name):
        """设置运动参数"""
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
        """等待指定时间，每秒更新状态"""
        start_time = time.time()

        while self.running and (time.time() - start_time) < duration:
            elapsed = time.time() - start_time
            remaining = duration - elapsed

            # 每秒更新一次剩余时间
            if int(elapsed) != int(elapsed - 0.1):
                self.get_logger().info(f"  {action_name}中... 已执行: {elapsed:.1f}s, 剩余: {remaining:.1f}s")

            time.sleep(0.1)

    def command_sequence(self):
        """执行正方形行走命令序列"""
        sequence_count = 0

        while self.running:
            sequence_count += 1
            self.get_logger().info(f"=== 开始第 {sequence_count} 个正方形 ===")

            if RERUN_INSTALLED:
                rr.log("world/sequence", rr.TextLog(f"开始第 {sequence_count} 个正方形", color=[255, 255, 0]))

            try:
                # 1. 前进 (Forward) - 3秒
                self.set_movement(
                    vx=CONFIG["SPEED_FORWARD"],
                    vy=0.0,
                    wz=0.0,
                    action_name="前进"
                )
                self.wait_duration(CONFIG["FORWARD_TIME"], "前进")
                if not self.running: break

                # 暂停
                self.set_movement(0.0, 0.0, 0.0, "暂停")
                time.sleep(CONFIG["PAUSE_BETWEEN"])
                if not self.running: break

                # 2. 右侧走 (Right) - 3秒
                self.set_movement(
                    vx=0.0,
                    vy=CONFIG["SPEED_SIDEWAYS"],
                    wz=0.0,
                    action_name="右侧走"
                )
                self.wait_duration(CONFIG["RIGHT_TIME"], "右侧走")
                if not self.running: break

                # 暂停
                self.set_movement(0.0, 0.0, 0.0, "暂停")
                time.sleep(CONFIG["PAUSE_BETWEEN"])
                if not self.running: break

                # 3. 后退 (Backward) - 3秒
                self.set_movement(
                    vx=CONFIG["SPEED_BACKWARD"],
                    vy=0.0,
                    wz=0.0,
                    action_name="后退"
                )
                self.wait_duration(CONFIG["BACKWARD_TIME"], "后退")
                if not self.running: break

                # 暂停
                self.set_movement(0.0, 0.0, 0.0, "暂停")
                time.sleep(CONFIG["PAUSE_BETWEEN"])
                if not self.running: break

                # 4. 左侧走 (Left) - 3秒
                self.set_movement(
                    vx=0.0,
                    vy=-CONFIG["SPEED_SIDEWAYS"],  # 负值表示向左
                    wz=0.0,
                    action_name="左侧走"
                )
                self.wait_duration(CONFIG["LEFT_TIME"], "左侧走")
                if not self.running: break

                # 正方形完成
                self.get_logger().info(f"=== 第 {sequence_count} 个正方形完成 ===")

                # 停止并等待
                self.set_movement(0.0, 0.0, 0.0, "停止")
                if RERUN_INSTALLED:
                    rr.log("world/complete", rr.TextLog(f"第 {sequence_count} 个正方形完成!", color=[0, 255, 255]))

                # 等待2秒后开始下一个正方形
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

        # 发送停止指令
        if self.loco is not None:
            try:
                for _ in range(20):  # 连续发送多次确保停止
                    self.loco.Move(0.0, 0.0, 0.0)
                    time.sleep(0.01)
                self.get_logger().info("✓ 机器人已停止")
            except Exception as e:
                self.get_logger().error(f"停止指令发送失败: {e}")

        # 等待线程结束
        if self.control_thread.is_alive():
            self.control_thread.join(timeout=2.0)
        if self.command_thread.is_alive():
            self.command_thread.join(timeout=2.0)

def main(args=None):
    rclpy.init(args=args)
    node = SquareWalkSystem()

    try:
        # 保持节点运行
        while node.running:
            rclpy.spin_once(node, timeout_sec=0.1)
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n接收到停止信号...")
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()
        print("✅ 系统已关闭")

if __name__ == '__main__':
    main()