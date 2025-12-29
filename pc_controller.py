#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
上位机控制程序 (PC Controller)
=====================================
功能：
  - 摄像头采集和透视校正 (7x7m 场地 → 500x500 像素俯视图)
  - ArUco 多码融合定位 (ID 99 主码 + ID 88 辅助码)
  - 电子围栏边界检测与脚本拦截
  - UDP 发送运动指令到机器人

使用方法：
  $ python3 pc_controller.py --robot-ip 192.168.x.x

  仅测试视觉 (不连接机器人)：
  $ python3 pc_controller.py --vision-test

  透视标定模式：
  $ python3 pc_controller.py --calibrate

作者：He
日期：2025-12-28

=================================================================================
偏移量测量指南 (IMPORTANT)
=================================================================================

  机器人俯视图：

       (前方)
         ↑
    ┌─────────────┐
    │   ID 88     │  ← 胸部码 (垂直贴装)
    │     ●       │
    │     │       │
    │     │ OFFSET_DISTANCE (测量这个距离!)
    │     │       │
    │     ▼       │
    │   ID 99     │  ← 背部码 (平贴)
    │     ●       │
    └─────────────┘
       (后方)

  测量步骤：
  1. 用尺子测量 ID_88 中心 到 ID_99 中心的物理距离 (单位: 米)
  2. 填入下方 CONFIG["OFFSET_CHEST_TO_CENTER"] 中
  3. OFFSET 是一个向量，方向从 ID_88 指向 ID_99 (即机器人后方)

=================================================================================
"""

import socket
import struct
import time
import threading
import argparse
import sys
import math
import numpy as np

try:
    import cv2
    from cv2 import aruco
except ImportError:
    print("❌ 错误：需要安装 opencv-python 和 opencv-contrib-python")
    print("   pip install opencv-python opencv-contrib-python")
    sys.exit(1)

# ================= 配置参数 =================
CONFIG = {
    # 机器人通信
    "ROBOT_IP": "192.168.1.100",     # 机器人 IP 地址 (需要用户修改)
    "ROBOT_PORT": 5000,              # UDP 端口

    # 摄像头
    "CAMERA_ID": 0,                  # 摄像头 ID
    "FRAME_WIDTH": 1280,             # 摄像头分辨率
    "FRAME_HEIGHT": 720,

    # 场地参数
    "FIELD_SIZE_M": 7.0,             # 场地大小 (米)
    "VIEW_SIZE_PX": 500,             # 俯视图大小 (像素)
    "PIXELS_PER_METER": 500 / 7.0,   # 像素/米 换算比例

    # ArUco 码
    "ARUCO_DICT": aruco.DICT_4X4_100,  # ArUco 字典
    "ID_MAIN": 99,                     # 主码 ID (背部中心)
    "ID_AUX": 88,                      # 辅助码 ID (胸部)

    # ============================================
    # 偏移量参数 (需要用户测量后填入!)
    # ============================================
    # ID_88 (胸部) 到 ID_99 (背部中心) 的距离 (米)
    # 这是机器人局部坐标系中的偏移向量，方向为从胸部指向背部
    "OFFSET_CHEST_TO_CENTER": 0.30,  # 示例: 30厘米，请根据实际测量修改!

    # 电子围栏 (像素坐标)
    # 格式: [x_min, x_max, y_min, y_max]
    # 默认留出 10% 边距
    "SAFE_ZONE": [50, 450, 50, 450],

    # 控制参数
    "CONTROL_HZ": 20,                # 控制频率 (Hz)
    "CORRECTION_SPEED": 0.3,         # 纠偏速度 (m/s)
    "CENTER_THRESHOLD": 30,          # 回到中心的判定阈值 (像素)

    # 安全
    "MARKER_LOST_TIMEOUT_S": 1.0,    # 标记丢失超时 (秒)
}

# ================= 预设动作脚本 =================
# 每个动作: (vx, vy, wz, duration_s, name)
ACTION_SCRIPT = [
    (0.5, 0.0, 0.0, 3.0, "前进"),
    (0.0, 0.0, 0.5, 2.0, "左转"),
    (0.0, 0.3, 0.0, 2.0, "右移"),
    (0.0, 0.0, -0.5, 2.0, "右转"),
    (-0.3, 0.0, 0.0, 2.0, "后退"),
]


class PCController:
    """
    上位机控制器
    
    核心功能：
    1. 视觉定位：透视校正 + ArUco 检测 + 多码融合
    2. 电子围栏：越界检测 + 脚本拦截 + 自动纠偏
    3. 通信：UDP 发送指令到机器人
    """

    def __init__(self, robot_ip=None, camera_id=0, vision_test=False):
        """
        初始化控制器
        
        Args:
            robot_ip: 机器人 IP 地址
            camera_id: 摄像头 ID
            vision_test: 仅测试视觉模式 (不发送指令)
        """
        self.robot_ip = robot_ip or CONFIG["ROBOT_IP"]
        self.camera_id = camera_id
        self.vision_test = vision_test
        self.running = False

        # 透视变换矩阵 (需要标定)
        self.perspective_matrix = None
        self.calibration_points = []

        # ArUco 检测器
        self.aruco_dict = aruco.getPredefinedDictionary(CONFIG["ARUCO_DICT"])
        self.aruco_params = aruco.DetectorParameters()

        # 机器人状态
        self.robot_position = None       # (x, y) 像素坐标
        self.robot_heading = 0.0         # 朝向角度 (弧度)
        self.position_source = None      # 'main' 或 'aux'
        self.last_detection_time = 0.0
        self.markers_lost = False

        # 控制状态
        self.script_blocked = False      # 脚本是否被拦截
        self.current_action_idx = 0      # 当前动作索引
        self.action_start_time = 0.0     # 当前动作开始时间

        # UDP socket
        self.sock = None
        if not vision_test:
            self._init_udp()

        # 摄像头
        self.cap = None
        self.current_frame = None
        self.warped_frame = None
        self.frame_lock = threading.Lock()

        print(f"✓ PCController 初始化完成")
        if vision_test:
            print("  📷 视觉测试模式 (不发送指令)")
        else:
            print(f"  🤖 机器人 IP: {self.robot_ip}:{CONFIG['ROBOT_PORT']}")

    def _init_udp(self):
        """初始化 UDP socket"""
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        print(f"✓ UDP 初始化完成")

    def send_command(self, vx, vy, wz):
        """
        发送运动指令到机器人
        
        Args:
            vx: 前进速度 (m/s)
            vy: 侧向速度 (m/s)
            wz: 转向速度 (rad/s)
        """
        if self.vision_test or self.sock is None:
            return

        try:
            # 文本格式
            msg = f"{vx:.3f},{vy:.3f},{wz:.3f}"
            self.sock.sendto(msg.encode('utf-8'),
                             (self.robot_ip, CONFIG["ROBOT_PORT"]))
        except Exception as e:
            print(f"❌ 发送指令失败: {e}")

    def emergency_stop(self):
        """紧急停止"""
        print("🚨 紧急停止!")
        for _ in range(10):
            self.send_command(0.0, 0.0, 0.0)
            time.sleep(0.01)

    # ============ 透视校正 ============

    def calibrate_perspective(self):
        """
        透视校正标定
        
        用户点击画面中的 4 个角点 (按顺序：左上, 右上, 右下, 左下)
        这些点对应实际场地的 4 个角
        """
        print("\n========== 透视校正标定 ==========")
        print("请在画面中依次点击场地的 4 个角点：")
        print("  1. 左上角  2. 右上角  3. 右下角  4. 左下角")
        print("按 'r' 重置，按 'q' 退出，按 Enter 确认")

        self.calibration_points = []

        def mouse_callback(event, x, y, flags, param):
            if event == cv2.EVENT_LBUTTONDOWN:
                if len(self.calibration_points) < 4:
                    self.calibration_points.append([x, y])
                    print(f"  点 {len(self.calibration_points)}: ({x}, {y})")

        cv2.namedWindow("Calibration")
        cv2.setMouseCallback("Calibration", mouse_callback)

        cap = cv2.VideoCapture(self.camera_id)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, CONFIG["FRAME_WIDTH"])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CONFIG["FRAME_HEIGHT"])

        while True:
            ret, frame = cap.read()
            if not ret:
                continue

            display = frame.copy()

            # 绘制已标定的点
            for i, pt in enumerate(self.calibration_points):
                cv2.circle(display, tuple(pt), 8, (0, 255, 0), -1)
                cv2.putText(display, str(i + 1), (pt[0] + 10, pt[1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

            # 连接点
            if len(self.calibration_points) >= 2:
                for i in range(len(self.calibration_points) - 1):
                    cv2.line(display,
                             tuple(self.calibration_points[i]),
                             tuple(self.calibration_points[i + 1]),
                             (0, 255, 0), 2)
                if len(self.calibration_points) == 4:
                    cv2.line(display,
                             tuple(self.calibration_points[3]),
                             tuple(self.calibration_points[0]),
                             (0, 255, 0), 2)

            cv2.putText(display, f"Points: {len(self.calibration_points)}/4",
                        (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)

            cv2.imshow("Calibration", display)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('r'):
                self.calibration_points = []
                print("  已重置")
            elif key == 13 and len(self.calibration_points) == 4:  # Enter
                break

        cap.release()
        cv2.destroyAllWindows()

        if len(self.calibration_points) == 4:
            self._compute_perspective_matrix()
            print("✓ 透视校正标定完成")
            return True
        else:
            print("⚠️ 标定取消")
            return False

    def _compute_perspective_matrix(self):
        """计算透视变换矩阵"""
        src = np.float32(self.calibration_points)
        size = CONFIG["VIEW_SIZE_PX"]
        dst = np.float32([[0, 0], [size, 0], [size, size], [0, size]])
        self.perspective_matrix = cv2.getPerspectiveTransform(src, dst)

    def set_perspective_points(self, points):
        """
        直接设置透视校正点 (用于加载预设)
        
        Args:
            points: [[x1,y1], [x2,y2], [x3,y3], [x4,y4]] 左上/右上/右下/左下
        """
        self.calibration_points = points
        self._compute_perspective_matrix()
        print(f"✓ 已加载透视校正点: {points}")

    def apply_perspective(self, frame):
        """
        应用透视变换
        
        Args:
            frame: 原始图像
            
        Returns:
            变换后的 500x500 俯视图
        """
        if self.perspective_matrix is None:
            return None
        size = CONFIG["VIEW_SIZE_PX"]
        return cv2.warpPerspective(frame, self.perspective_matrix, (size, size))

    # ============ ArUco 检测 ============

    def detect_markers(self, frame):
        """
        检测 ArUco 标记
        
        Args:
            frame: 图像 (应该是透视校正后的俯视图)
            
        Returns:
            dict: {marker_id: (center_x, center_y, heading_rad)}
        """
        if frame is None:
            return {}

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.aruco_params
        )

        results = {}
        if ids is not None:
            for i, marker_id in enumerate(ids.flatten()):
                # 计算中心点
                corner = corners[i][0]
                center_x = np.mean(corner[:, 0])
                center_y = np.mean(corner[:, 1])

                # 计算朝向 (从标记的顶部两个角计算)
                top_center = (corner[0] + corner[1]) / 2
                bottom_center = (corner[2] + corner[3]) / 2
                heading = math.atan2(
                    top_center[1] - bottom_center[1],
                    top_center[0] - bottom_center[0]
                )

                results[marker_id] = (center_x, center_y, heading)

        return results

    def get_robot_position(self, markers):
        """
        多码融合定位 (核心算法)
        
        优先级：
        1. ID 99 (主码/背部) - 直接使用
        2. ID 88 (辅助码/胸部) - 加偏移补偿
        3. 都没有 - 返回 None
        
        Args:
            markers: detect_markers() 的返回值
            
        Returns:
            (x, y, heading, source) 或 None
        """
        id_main = CONFIG["ID_MAIN"]
        id_aux = CONFIG["ID_AUX"]

        if id_main in markers:
            # 主码检测到 - 直接使用
            x, y, heading = markers[id_main]
            return (x, y, heading, 'main')

        elif id_aux in markers:
            # 辅助码检测到 - 需要补偿偏移
            x_aux, y_aux, heading = markers[id_aux]

            # 计算偏移向量
            # 偏移方向：从辅助码 (胸部) 指向主码 (背部)
            # 在机器人局部坐标系中，这是"向后"的方向
            offset_distance = CONFIG["OFFSET_CHEST_TO_CENTER"] * CONFIG["PIXELS_PER_METER"]

            # 偏移方向与机器人朝向相反 (向后)
            offset_x = -offset_distance * math.cos(heading)
            offset_y = -offset_distance * math.sin(heading)

            x = x_aux + offset_x
            y = y_aux + offset_y

            return (x, y, heading, 'aux')

        return None

    # ============ 电子围栏 ============

    def check_fence(self, x, y):
        """
        检查是否在安全区域内
        
        Args:
            x, y: 机器人位置 (像素坐标)
            
        Returns:
            bool: True = 在安全区内, False = 越界
        """
        zone = CONFIG["SAFE_ZONE"]
        return zone[0] <= x <= zone[1] and zone[2] <= y <= zone[3]

    def calculate_correction(self, x, y):
        """
        计算纠偏指令
        
        当机器人越界时，计算一个朝向区域中心的速度向量
        
        Args:
            x, y: 机器人当前位置 (像素坐标)
            
        Returns:
            (vx, vy, wz): 纠偏速度指令
        """
        size = CONFIG["VIEW_SIZE_PX"]
        center_x = size / 2
        center_y = size / 2

        # 计算到中心的向量
        dx = center_x - x
        dy = center_y - y
        distance = math.sqrt(dx * dx + dy * dy)

        if distance < CONFIG["CENTER_THRESHOLD"]:
            return (0.0, 0.0, 0.0)  # 已回到中心

        # 归一化并缩放到纠偏速度
        speed = CONFIG["CORRECTION_SPEED"]
        vx = (dx / distance) * speed
        vy = (dy / distance) * speed

        # 转换到机器人坐标系
        # 注意：俯视图坐标系与机器人坐标系可能不同
        # 这里假设 y 轴向上对应机器人前进方向
        robot_vx = -vy  # 向上 = 前进
        robot_vy = vx   # 向右 = 右移

        return (robot_vx, robot_vy, 0.0)

    def is_at_center(self, x, y):
        """检查是否已回到中心区域"""
        size = CONFIG["VIEW_SIZE_PX"]
        dx = x - size / 2
        dy = y - size / 2
        return math.sqrt(dx * dx + dy * dy) < CONFIG["CENTER_THRESHOLD"]

    # ============ 脚本控制 ============

    def get_script_command(self):
        """
        获取当前脚本动作的指令
        
        Returns:
            (vx, vy, wz) 或 None (脚本完成)
        """
        if self.script_blocked:
            return None

        if self.current_action_idx >= len(ACTION_SCRIPT):
            return None  # 脚本完成

        action = ACTION_SCRIPT[self.current_action_idx]
        vx, vy, wz, duration, name = action

        now = time.time()
        if self.action_start_time == 0:
            self.action_start_time = now
            print(f"  ▶ 执行动作 [{self.current_action_idx + 1}/{len(ACTION_SCRIPT)}]: {name}")

        if now - self.action_start_time >= duration:
            # 当前动作完成，进入下一个
            self.current_action_idx += 1
            self.action_start_time = 0

            if self.current_action_idx >= len(ACTION_SCRIPT):
                print("  ✓ 脚本执行完成")
                return None

        return (vx, vy, wz)

    def block_script(self):
        """拦截脚本"""
        if not self.script_blocked:
            print("  ⚠️ 越界！脚本已拦截")
            self.script_blocked = True

    def resume_script(self):
        """恢复脚本"""
        if self.script_blocked:
            print("  ✅ 已回到安全区，恢复脚本")
            self.script_blocked = False

    # ============ 主循环 ============

    def _vision_loop(self):
        """视觉处理线程"""
        cap = cv2.VideoCapture(self.camera_id)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, CONFIG["FRAME_WIDTH"])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CONFIG["FRAME_HEIGHT"])

        print("📷 摄像头已启动")

        while self.running:
            ret, frame = cap.read()
            if not ret:
                time.sleep(0.01)
                continue

            # 透视变换
            if self.perspective_matrix is not None:
                warped = self.apply_perspective(frame)
            else:
                warped = frame

            # ArUco 检测
            markers = self.detect_markers(warped)

            # 多码融合定位
            pos = self.get_robot_position(markers)

            with self.frame_lock:
                self.current_frame = frame
                self.warped_frame = warped

                if pos is not None:
                    self.robot_position = (pos[0], pos[1])
                    self.robot_heading = pos[2]
                    self.position_source = pos[3]
                    self.last_detection_time = time.time()
                    self.markers_lost = False
                else:
                    # 检查超时
                    if time.time() - self.last_detection_time > CONFIG["MARKER_LOST_TIMEOUT_S"]:
                        if not self.markers_lost:
                            print("⚠️ 标记丢失超时！")
                            self.markers_lost = True

            time.sleep(0.01)

        cap.release()
        print("📷 摄像头已关闭")

    def _control_loop(self):
        """控制决策线程"""
        control_interval = 1.0 / CONFIG["CONTROL_HZ"]
        last_log_time = 0

        print("🎮 控制循环已启动")

        while self.running:
            now = time.time()

            with self.frame_lock:
                pos = self.robot_position
                lost = self.markers_lost

            # ============ 安全检查 ============
            if lost:
                # 标记丢失 - 紧急停止
                self.send_command(0.0, 0.0, 0.0)
                time.sleep(control_interval)
                continue

            if pos is None:
                time.sleep(control_interval)
                continue

            x, y = pos

            # ============ 电子围栏检测 ============
            if not self.check_fence(x, y):
                # 越界！拦截脚本，执行纠偏
                self.block_script()
                vx, vy, wz = self.calculate_correction(x, y)
                self.send_command(vx, vy, wz)

                if now - last_log_time > 0.5:
                    print(f"  📍 纠偏中: ({x:.0f}, {y:.0f}) → 中心, 指令: vx={vx:.2f}")
                    last_log_time = now

            else:
                # 在安全区内
                if self.script_blocked:
                    # 检查是否回到中心
                    if self.is_at_center(x, y):
                        self.resume_script()
                    else:
                        # 继续纠偏
                        vx, vy, wz = self.calculate_correction(x, y)
                        self.send_command(vx, vy, wz)
                        time.sleep(control_interval)
                        continue

                # 执行脚本
                cmd = self.get_script_command()
                if cmd is not None:
                    vx, vy, wz = cmd
                    self.send_command(vx, vy, wz)
                else:
                    self.send_command(0.0, 0.0, 0.0)

            time.sleep(control_interval)

        print("🎮 控制循环已停止")

    def _display_loop(self):
        """显示线程 (调试用)"""
        print("🖥️ 显示窗口已启动")

        while self.running:
            with self.frame_lock:
                warped = self.warped_frame.copy() if self.warped_frame is not None else None
                pos = self.robot_position
                source = self.position_source

            if warped is None:
                time.sleep(0.03)
                continue

            # 绘制安全区域
            zone = CONFIG["SAFE_ZONE"]
            cv2.rectangle(warped,
                          (zone[0], zone[2]),
                          (zone[1], zone[3]),
                          (0, 255, 0), 2)

            # 绘制中心点
            center = CONFIG["VIEW_SIZE_PX"] // 2
            cv2.circle(warped, (center, center), 5, (255, 255, 0), -1)

            # 绘制机器人位置
            if pos is not None:
                x, y = int(pos[0]), int(pos[1])
                color = (0, 255, 0) if source == 'main' else (255, 165, 0)
                cv2.circle(warped, (x, y), 10, color, -1)

                # 显示定位源
                text = f"ID {CONFIG['ID_MAIN']}" if source == 'main' else f"ID {CONFIG['ID_AUX']} (补偿)"
                cv2.putText(warped, text, (x + 15, y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

                # 状态文字
                status = "越界!" if self.script_blocked else "正常"
                cv2.putText(warped, f"Status: {status}", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

            cv2.imshow("PC Controller", warped)

            key = cv2.waitKey(30) & 0xFF
            if key == ord('q'):
                self.running = False
                break

        cv2.destroyAllWindows()
        print("🖥️ 显示窗口已关闭")

    def start(self):
        """启动控制器"""
        if self.running:
            print("⚠️ 控制器已在运行")
            return

        self.running = True

        # 启动线程
        self.vision_thread = threading.Thread(target=self._vision_loop, daemon=True)
        self.control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self.display_thread = threading.Thread(target=self._display_loop, daemon=True)

        self.vision_thread.start()
        time.sleep(0.5)  # 等待摄像头启动
        self.control_thread.start()
        self.display_thread.start()

        print("=" * 50)
        print("  🖥️ PC Controller 已启动")
        print(f"  🤖 目标机器人: {self.robot_ip}:{CONFIG['ROBOT_PORT']}")
        print(f"  📐 安全区域: {CONFIG['SAFE_ZONE']}")
        print("=" * 50)
        print("\n按 'q' 退出\n")

    def stop(self):
        """停止控制器"""
        print("\n正在停止...")
        self.running = False

        # 紧急停止
        self.emergency_stop()

        # 等待线程结束
        if hasattr(self, 'vision_thread') and self.vision_thread.is_alive():
            self.vision_thread.join(timeout=2.0)
        if hasattr(self, 'control_thread') and self.control_thread.is_alive():
            self.control_thread.join(timeout=2.0)
        if hasattr(self, 'display_thread') and self.display_thread.is_alive():
            self.display_thread.join(timeout=2.0)

        if self.sock:
            self.sock.close()

        print("✅ PC Controller 已停止")

    def wait(self):
        """阻塞等待"""
        try:
            while self.running:
                time.sleep(0.1)
        except KeyboardInterrupt:
            self.stop()


def main():
    parser = argparse.ArgumentParser(description="上位机控制程序")
    parser.add_argument("--robot-ip", type=str, default=CONFIG["ROBOT_IP"],
                        help=f"机器人 IP 地址 (默认: {CONFIG['ROBOT_IP']})")
    parser.add_argument("--camera", type=int, default=CONFIG["CAMERA_ID"],
                        help=f"摄像头 ID (默认: {CONFIG['CAMERA_ID']})")
    parser.add_argument("--vision-test", action="store_true",
                        help="仅测试视觉 (不发送指令)")
    parser.add_argument("--calibrate", action="store_true",
                        help="透视校正标定模式")
    args = parser.parse_args()

    print("=" * 60)
    print("  🖥️ 多码融合电子围栏系统 - 上位机控制程序")
    print("=" * 60)

    controller = PCController(
        robot_ip=args.robot_ip,
        camera_id=args.camera,
        vision_test=args.vision_test
    )

    # 标定模式
    if args.calibrate:
        controller.calibrate_perspective()
        return

    # 检查是否有透视矩阵
    if controller.perspective_matrix is None:
        print("\n⚠️ 未找到透视校正数据")
        print("   首次运行请使用 --calibrate 进行标定")
        print("   或在代码中调用 set_perspective_points() 设置预设坐标")
        
        # 使用默认的简单映射 (假设摄像头已经是俯视)
        print("\n   使用默认透视 (假设摄像头为俯视)...")
        default_points = [[0, 0], [640, 0], [640, 480], [0, 480]]
        controller.set_perspective_points(default_points)

    controller.start()
    controller.wait()


if __name__ == "__main__":
    main()
