# Booster Robotics SDK Python 程序完整指南

本文档详细介绍 SDK 中所有 Python 控制程序的功能、设计、配置变量和使用方法。

---

## 目录

1. [程序总览](#程序总览)
2. [sensor_monitor.py - 传感器监控](#sensor_monitorpy---传感器监控)
3. [move.py - 基础运动控制](#movepy---基础运动控制)
4. [auto_control.py - 手柄自动模式](#auto_controlpy---手柄自动模式)
5. [circle_walk.py - 闭环圆形行走](#circle_walkpy---闭环圆形行走)
6. [robust_circle_walk.py - 鲁棒圆形行走](#robust_circle_walkpy---鲁棒圆形行走)
7. [robust_line_walk.py - 鲁棒直线往返](#robust_line_walkpy---鲁棒直线往返)
8. [核心设计模式](#核心设计模式)
9. [调参指南](#调参指南)

---

## 程序总览

| 程序 | 功能 | 控制方式 | 复杂度 |
|------|------|----------|--------|
| `sensor_monitor.py` | 实时显示传感器数据 | 纯监控，无控制 | ⭐ |
| `move.py` | 基础运动模式（正方形、圆弧） | 开环时间控制 | ⭐⭐ |
| `auto_control.py` | 手柄切换自动/手动模式 | 手柄组合键 | ⭐⭐ |
| `circle_walk.py` | 闭环圆形行走+回原点 | 陀螺仪+里程计 | ⭐⭐⭐ |
| `robust_circle_walk.py` | 鲁棒多圈行走 | 双闭环+漂移修复 | ⭐⭐⭐⭐ |
| `robust_line_walk.py` | 直线往返+航向保持 | 闭环航向控制 | ⭐⭐⭐⭐ |

---

## sensor_monitor.py - 传感器监控

### 功能描述

实时读取并显示机器人的 IMU、里程计、电机状态等传感器数据。

### 运行方式

```bash
python3 sensor_monitor.py
python3 sensor_monitor.py -i 0.5          # 0.5秒刷新
python3 sensor_monitor.py --no-clear      # 不清屏，追加打印
```

### 显示内容

```
📐 IMU / 陀螺仪数据
  姿态角 (RPY): Roll, Pitch, Yaw
  角速度 (Gyro): X, Y, Z
  加速度 (Acc): X, Y, Z

📍 里程计数据
  位置: X, Y, Theta
  速度: Vx, Vy, Vtheta

⚙️ 电机状态
  串行电机: 位置、速度、力矩
  并行电机: 位置、速度、力矩
```

### 配置变量

```python
CONFIG = {
    "NETWORK_INTERFACE": "127.0.0.1",  # SDK通信地址
    "DOMAIN_ID": 0,                     # DDS Domain ID
    "PRINT_INTERVAL": 0.1,              # 刷新间隔（秒）
    "CLEAR_SCREEN": True,               # 是否清屏刷新
}
```

### 代码结构

```
sensor_monitor.py
├── SensorData            # 传感器数据存储类
│   ├── imu_rpy           # 姿态角
│   ├── imu_gyro          # 角速度
│   ├── odom_x/y/theta    # 里程计位置
│   └── velocity_x/y      # 计算速度
├── low_state_handler()   # IMU+电机状态回调
├── odometer_handler()    # 里程计回调
└── display_loop()        # 显示循环
```

### 关键技术点

**速度计算 - 差分法：**
```python
velocity_x = (current_x - last_x) / dt
```

**低通滤波平滑：**
```python
filtered = 0.3 * raw + 0.7 * filtered_prev
```

**死区处理：**
```python
if abs(velocity) < 0.05:  # 小于阈值视为0
    velocity = 0.0
```

---

## move.py - 基础运动控制

### 功能描述

提供多种预设运动模式：正方形行走、左圈、右圈、左圈转身右圈。

### 运行方式

```bash
python3 move.py
```

修改 `WALK_MODE` 变量选择模式：
```python
WALK_MODE = "left_circle"   # 可选: square, left_circle, right_circle, left_turn_right
```

### 运动模式

| 模式 | 轨迹 | 说明 |
|------|------|------|
| `square` | 正方形 | 前-右-后-左 循环 |
| `left_circle` | 逆时针圆弧 | 前进+左转 |
| `right_circle` | 顺时针圆弧 | 前进+右转 |
| `left_turn_right` | 左圈→转180°→右圈 | 组合动作 |

### 配置变量

```python
CONFIG = {
    # ===== 正方形参数 =====
    "FORWARD_TIME": 3.0,       # 前进时间（秒）
    "RIGHT_TIME": 3.0,         # 右移时间
    "BACKWARD_TIME": 3.0,      # 后退时间
    "LEFT_TIME": 3.0,          # 左移时间
    "PAUSE_BETWEEN": 0,        # 动作间暂停
    
    "SPEED_FORWARD": 0.3,      # 前进速度 (m/s)
    "SPEED_SIDEWAYS": 0.3,     # 侧向速度
    "SPEED_BACKWARD": -0.3,    # 后退速度
    
    # ===== 圆弧参数 =====
    "CIRCLE_FORWARD_SPEED": 1.8,  # 走圈前进速度
    "CIRCLE_TURN_SPEED": 1.5,     # 转向角速度 (rad/s)
    "CIRCLE_DURATION": 5000.0,    # 持续时间
    
    # ===== 转身参数 =====
    "TURN_180_SPEED": 0.5,        # 转身角速度
    "TURN_180_DURATION": 3.14,    # 转身时间 (π/速度)
    
    # ===== 手柄参数 =====
    "GAMEPAD_MAX_VX": 0.8,        # 手动最大前进速度
    "GAMEPAD_MAX_VY": 0.4,        # 手动最大侧移速度
    "GAMEPAD_MAX_VYAW": 1.0,      # 手动最大转向速度
}
```

### 手柄控制

**组合键：`LT + RT + 十字键上`**

| 状态 | 行为 |
|------|------|
| 自动模式 | 执行预设运动 |
| 手动模式 | 手柄直接控制 |

**触发方式：** 组合键释放时触发（非按下时）

### 代码结构

```
move.py
├── CONFIG                    # 配置参数
├── ControllerState           # 手柄状态类
│   ├── check_toggle_combo()  # 组合键检测
│   └── update_from_sdk()     # SDK数据更新
├── CircleWalkSystem          # 主系统
│   ├── control_loop()        # 指令发送循环
│   ├── command_sequence()    # 运动序列调度
│   ├── _run_square_sequence()
│   ├── _run_left_circle_sequence()
│   └── _run_right_circle_sequence()
└── main()
```

---

## auto_control.py - 手柄自动模式

### 功能描述

通过手柄组合键切换自动/手动模式，自动模式下以固定速度运动。

### 运行方式

```bash
python3 auto_control.py
```

### 组合键

**`LT + LB + 十字键上`** → 切换自动/手动

### 配置变量

```python
CONFIG = {
    # 网络配置
    "NETWORK_INTERFACE": "127.0.0.1",
    "DOMAIN_ID": 0,
    
    # 控制频率
    "CONTROL_HZ": 20,
    
    # 自动模式速度
    "AUTO_VX": 0.3,       # 前进速度 (m/s)
    "AUTO_VY": 0.0,       # 侧向速度
    "AUTO_VYAW": 0.3,     # 转向速度 (rad/s)
    
    # 组合键防抖
    "COMBO_DEBOUNCE": 0.5,
    
    # 控制方式
    "USE_KEYBOARD": False,  # True=键盘控制（手柄不可用时）
}
```

### 键盘备用

当手柄不可用时，设置 `USE_KEYBOARD = True`：

| 按键 | 功能 |
|------|------|
| `t` | 切换自动/手动 |
| `s` | 停止运动 |
| `q` | 退出程序 |

---

## circle_walk.py - 闭环圆形行走

### 功能描述

使用陀螺仪+里程计实现闭环圆形行走，走完一圈后回到原点。

### 运行方式

```bash
python3 circle_walk.py
```

### 配置变量

```python
CONFIG = {
    # ===== 圆形参数 =====
    "CIRCLE_RADIUS": 1.8,           # 目标半径（参考值）
    "CIRCLE_FORWARD_SPEED": 1.8,    # 前进速度 (m/s)
    "CIRCLE_TURN_SPEED": 1.2,       # 转向角速度 (rad/s)
    
    # ===== 闭环控制 =====
    "YAW_RATE_KP": 0.3,             # 角速度P增益
    "POSITION_KP": 0.5,             # 位置P增益
    "HEADING_KP": 1.0,              # 航向P增益
    
    # ===== 回原点 =====
    "RETURN_THRESHOLD_DEG": 330,    # 开始回原点的角度
    "POSITION_TOLERANCE": 0.5,      # 位置容差 (米)
    
    # ===== 方向 =====
    "CIRCLE_DIRECTION": 1,          # 1=左圈, -1=右圈
    
    # ===== 运行模式 =====
    "SINGLE_CIRCLE_MODE": False,    # True=单圈, False=连续
}
```

### 控制逻辑

```
阶段1 (0° ~ 330°): 开环走圈
  └── 固定速度，不做闭环调整

阶段2 (330° 后): 闭环回原点
  ├── 计算到起点的角度
  ├── 航向P控制
  └── 到达起点后重新开始
```

### 与 robust_circle_walk.py 的区别

| 特性 | circle_walk.py | robust_circle_walk.py |
|------|----------------|----------------------|
| 角度计算 | yaw差值法 | yaw差值法 |
| 多圈支持 | 有漂移问题 | 每圈重新校准起点 |
| 滤波 | 无 | 低通滤波 |
| 速度控制 | 直接设置 | 渐变平滑 |

---

## robust_circle_walk.py - 鲁棒圆形行走

### 功能描述

多圈稳定行走，每圈完成后自动校准起点，解决漂移累积问题。

### 运行方式

```bash
python3 robust_circle_walk.py
```

### 配置变量

```python
CONFIG = {
    # ===== 圆形行走 =====
    "CIRCLE_FORWARD_SPEED": 0.3,    # 前进速度 (m/s)
    "CIRCLE_TURN_SPEED": 0.3,       # 转向角速度 (rad/s)
    "CIRCLE_DIRECTION": 1,          # 1=左圈, -1=右圈
    "CIRCLE_COUNT": 1,              # 走几圈
    
    # ===== 回原点 =====
    "RETURN_THRESHOLD_DEG": 330,    # 开始回原点的角度
    "POSITION_TOLERANCE": 0.3,      # 位置容差 (米)
    "HEADING_KP": 0.8,              # 航向P增益
    "MAX_RETURN_WZ": 1.0,           # 回原点最大角速度
    
    # ===== 控制参数 =====
    "CONTROL_HZ": 20,               # 控制频率
    "SMOOTH_RATE": 0.05,            # 速度平滑率
    
    # ===== 滤波参数 =====
    "FILTER_ALPHA": 0.3,            # 低通滤波系数
    "SENSOR_TIMEOUT": 1.0,          # 传感器超时
}
```

### 状态机

```
┌─────────────────────────────────────────────┐
│                  状态机                      │
├─────────────────────────────────────────────┤
│                                             │
│  IDLE ──开始──→ 走圈(开环) ──330°──→ 回原点(闭环)
│    ↑              ↑                    │    │
│    │              │                    ↓    │
│  暂停          校准起点 ←────── 到达原点    │
│                                             │
└─────────────────────────────────────────────┘
```

### 多圈漂移修复

**问题：** 前2-3圈正常，后面满场乱跑

**原因：** 每圈只重置角度，起点位置不变，累积误差导致找不到原点

**解决：**
```python
if dist_to_start < POSITION_TOLERANCE:
    circles_completed += 1
    if circles_completed < CIRCLE_COUNT:
        # ★ 关键：从当前位置重新记录起点
        record_start_position(is_first_circle=False)
```

### 控制方式

| 输入 | 功能 |
|------|------|
| 🎮 `LT+RT+↑` | 暂停/继续（从当前位置重新开始） |
| ⌨️ `s` | 开始走圈 |
| ⌨️ `m` | 暂停/继续 |
| ⌨️ `r` | 重置状态 |
| ⌨️ `q` | 退出 |

---

## robust_line_walk.py - 鲁棒直线往返

### 功能描述

机器人直线前进到指定距离，然后返回起点，支持倒退或转身两种返回模式。

### 运行方式

```bash
python3 robust_line_walk.py
```

### 配置变量

```python
CONFIG = {
    # ===== 直线行走 =====
    "LINE_DISTANCE": 2.0,           # 单向距离 (米)
    "LINE_SPEED": 1.8,              # 行走速度 (m/s)
    "ROUND_TRIP_COUNT": 1,          # 往返次数
    
    # ===== 返回模式 =====
    # "backward" = 倒退返回（不转身）
    # "turn"     = 转身180°后前进返回
    "RETURN_MODE": "backward",
    
    # ===== 航向保持 =====
    "HEADING_KP": 0.8,              # 航向P增益
    "MAX_CORRECTION_WZ": 0.5,       # 最大校正角速度
    
    # ===== 转身参数（仅 turn 模式） =====
    "TURN_SPEED": 0.5,              # 转身角速度
    "TURN_TOLERANCE": 0.1,          # 转身容差 (rad)
    
    # ===== 容差 =====
    "DISTANCE_TOLERANCE": 0.1,      # 到达目标点容差
    "ARRIVAL_TOLERANCE": 0.2,       # 返回起点容差
}
```

### 状态机

```
                    ┌───────────────────────────────────┐
                    │             状态机                 │
                    ├───────────────────────────────────┤
                    │                                   │
┌──────┐  开始   ┌────────┐  到达目标   ┌───────────────┴───┐
│ IDLE │ ─────→ │ FORWARD │ ──────────→ │ backward/turn 模式 │
└──────┘        └────────┘             └───────────────────┘
   ↑                                           │
   │                                    ┌──────┴──────┐
   │                                    ↓             ↓
   │                              ┌──────────┐  ┌──────────┐
   │                              │ BACKWARD │  │ TURNING  │
   │                              │ (倒退)   │  │ (转身)   │
   │                              └────┬─────┘  └─────┬────┘
   │                                   │              │
   │                                   ↓              ↓
   │                              ┌──────────────────────┐
   │                              │   返回起点 ARRIVED   │
   │                              └──────────────────────┘
   │                                         │
   └─────────────── 全部完成 ←───────────────┘
```

### 返回模式对比

| 模式 | 行为 | 优点 | 缺点 |
|------|------|------|------|
| `backward` | 到达后直接倒退 | 快速、无需转身 | 倒退可能不稳定 |
| `turn` | 到达后转180°再前进 | 稳定 | 需要转身时间 |

### 航向保持控制

```python
# 计算航向误差
heading_error = target_heading - current_heading

# P控制校正
correction = HEADING_KP * heading_error

# 限幅
correction = clamp(correction, -MAX_CORRECTION_WZ, MAX_CORRECTION_WZ)

# 应用
target_wz = correction
```

---

## 核心设计模式

### 1. 传感器数据类

所有程序使用统一的 `SensorData` 类存储传感器数据：

```python
class SensorData:
    def __init__(self):
        # IMU数据
        self.yaw = 0.0
        self.gyro_z = 0.0
        
        # 里程计
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_theta = 0.0
        
        # 起点
        self.start_x = 0.0
        self.start_y = 0.0
        
        # 线程锁
        self.lock = threading.Lock()
```

### 2. 手柄状态类

```python
class ControllerState:
    def check_toggle_combo(self, lt, rt, hat_up):
        """
        组合键检测
        触发方式：释放时触发（非按下时）
        """
        if combo_now_pressed:
            self.combo_was_pressed = True
            return False  # 按下时不触发
        
        if self.combo_was_pressed and not combo_now_pressed:
            self.combo_was_pressed = False
            return True  # 释放时触发
```

### 3. 低通滤波器

```python
class LowPassFilter:
    def __init__(self, alpha=0.3):
        self.alpha = alpha
        self.value = 0.0
    
    def update(self, raw):
        self.value = self.alpha * raw + (1 - self.alpha) * self.value
        return self.value
```

### 4. 速度平滑

```python
def smooth_transition(current, target, rate):
    """避免速度突变"""
    diff = target - current
    if abs(diff) < rate:
        return target
    return current + rate if diff > 0 else current - rate
```

### 5. 角度归一化

```python
def normalize_angle(angle):
    """归一化到 [-π, π]"""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle
```

---

## 调参指南

### 通用问题

| 问题 | 可能原因 | 解决方法 |
|------|----------|----------|
| 运动抖动 | 控制增益过大 | 减小 `HEADING_KP` |
| 响应慢 | 控制增益过小 | 增大 `HEADING_KP` |
| 速度突变 | 平滑率过大 | 减小 `SMOOTH_RATE` |
| 传感器噪声 | 滤波不足 | 减小 `FILTER_ALPHA` |

### 圆形行走

| 问题 | 调整 |
|------|------|
| 圆太大 | 增大 `CIRCLE_TURN_SPEED` |
| 圆太小 | 减小 `CIRCLE_TURN_SPEED` |
| 多圈漂移 | 使用 `robust_circle_walk.py` |
| 回不到原点 | 减小 `RETURN_THRESHOLD_DEG` |

### 直线行走

| 问题 | 调整 |
|------|------|
| 走歪 | 增大 `HEADING_KP` |
| 过冲 | 增大 `DISTANCE_TOLERANCE` |
| 倒退不稳 | 改用 `RETURN_MODE = "turn"` |

---

## 快速参考

### 程序启动命令

```bash
# 传感器监控
python3 sensor_monitor.py

# 基础运动
python3 move.py

# 手柄自动模式
python3 auto_control.py

# 闭环圆形
python3 circle_walk.py
python3 robust_circle_walk.py

# 直线往返
python3 robust_line_walk.py
```

### 通用控制键

| 输入 | 功能 |
|------|------|
| 🎮 `LT+RT+↑` | 暂停/继续 |
| ⌨️ `m` | 暂停/继续 |
| ⌨️ `r` | 重置 |
| ⌨️ `q` | 退出 |
| `Ctrl+C` | 强制退出 |
