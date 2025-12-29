# Booster Robotics SDK 使用指南

> 本文档详细介绍 Booster Robotics SDK 支持的所有功能、参数配置和使用方法。

---

## 📋 目录

- [SDK 概述](#sdk-概述)
- [环境要求](#环境要求)
- [安装方法](#安装方法)
- [传感器数据读取](#传感器数据读取)
- [运动控制参数](#运动控制参数)
- [机器人模式切换](#机器人模式切换)
- [控制频率与速度关系](#控制频率与速度关系)
- [move.py 参数详解](#movepy-参数详解)
- [手柄控制说明](#手柄控制说明)
- [API 参考](#api-参考)
- [代码示例](#代码示例)

---

## SDK 概述

Booster Robotics SDK 提供简单易用的接口，用于控制 Booster Robotics 机器人产品。SDK 支持：

- ✅ **运动控制** - 前进、后退、侧移、转向
- ✅ **传感器读取** - IMU/陀螺仪、里程计、电机状态
- ✅ **模式切换** - 多种运动模式
- ✅ **手臂控制** - 末端执行器、夹爪、灵巧手
- ✅ **其他功能** - 舞蹈、声音播放、轨迹录制等

---

## 环境要求

| 项目 | 要求 |
|------|------|
| 操作系统 | Ubuntu 22.04 LTS |
| CPU架构 | aarch64 / x86_64 |
| 编译器 | gcc 11.4.0 |
| Python | 3.x (需安装 pybind11) |

---

## 安装方法

### 1. 安装 SDK
```bash
sudo ./install.sh
```

### 2. 安装 Python 依赖
```bash
pip3 install pybind11
pip3 install pybind11-stubgen
```

### 3. 编译 Python 绑定
```bash
mkdir build && cd build
cmake .. -DBUILD_PYTHON_BINDING=on
make
sudo make install
```

---

## 传感器数据读取

### ✅ 支持 IMU/陀螺仪数据读取

SDK **完全支持** IMU 数据的读取，包括：

| 数据类型 | 字段 | 说明 |
|---------|------|------|
| 姿态角 | `rpy[0], rpy[1], rpy[2]` | Roll, Pitch, Yaw (rad) |
| 角速度 | `gyro[0], gyro[1], gyro[2]` | 陀螺仪数据 (rad/s) |
| 加速度 | `acc[0], acc[1], acc[2]` | 加速度计数据 (m/s²) |

#### Python 示例：读取 IMU 数据
```python
from booster_robotics_sdk_python import ChannelFactory, B1LowStateSubscriber

def handler(low_state_msg):
    imu_state = low_state_msg.imu_state
    print(f"姿态角 RPY: {imu_state.rpy[0]}, {imu_state.rpy[1]}, {imu_state.rpy[2]}")
    print(f"角速度 Gyro: {imu_state.gyro[0]}, {imu_state.gyro[1]}, {imu_state.gyro[2]}")
    print(f"加速度 Acc: {imu_state.acc[0]}, {imu_state.acc[1]}, {imu_state.acc[2]}")

ChannelFactory.Instance().Init(0)
subscriber = B1LowStateSubscriber(handler)
subscriber.InitChannel()
```

### ✅ 支持里程计/速度数据读取

SDK 支持通过 `Odometer` 读取机器人的位置信息：

| 字段 | 说明 |
|------|------|
| `x` | X 方向位移 (m) |
| `y` | Y 方向位移 (m) |
| `theta` | 航向角 (rad) |

#### Python 示例：读取里程计数据
```python
from booster_robotics_sdk_python import ChannelFactory, B1OdometerStateSubscriber

def handler(odometer_msg):
    print(f"位置: x={odometer_msg.x}, y={odometer_msg.y}, theta={odometer_msg.theta}")

ChannelFactory.Instance().Init(0)
subscriber = B1OdometerStateSubscriber(handler)
subscriber.InitChannel()
```

### ✅ 支持电机状态读取

通过 `LowState` 可以读取所有电机的状态：

| 字段 | 说明 |
|------|------|
| `q` | 关节位置 (rad) |
| `dq` | 关节速度 (rad/s) |
| `ddq` | 关节加速度 (rad/s²) |
| `tau_est` | 估计力矩 (Nm) |
| `temperature` | 电机温度 |

---

## 运动控制参数

### Move() 函数参数

```python
client.Move(vx, vy, vyaw)
```

| 参数 | 类型 | 单位 | 说明 |
|------|------|------|------|
| `vx` | float | m/s | X方向线速度（前进/后退）|
| `vy` | float | m/s | Y方向线速度（左右侧移）|
| `vyaw` | float | rad/s | 角速度（转向）|

> ⚠️ **重要提示**：SDK 本身没有速度限制，实际速度限制由机器人固件内部执行。即使设置很大的速度值（如 10 m/s），机器人也会被固件限制在安全速度范围内。

---

## 机器人模式切换

### ✅ 支持特殊模式切换

SDK 支持以下机器人模式：

| 模式 | 枚举值 | 说明 |
|------|--------|------|
| `kDamping` | 0 | 阻尼模式 - 所有电机进入阻尼状态，机器人会倒下 |
| `kPrepare` | 1 | 准备模式 - 机器人保持双脚站立，可切换到行走模式 |
| `kWalking` | 2 | 行走模式 - 可以移动、旋转、踢球等 |
| `kCustom` | 3 | 自定义模式 - 执行自定义动作 |
| `kSoccer` | 4 | ⚡ **足球模式** - 可执行足球相关动作，可能有更高速度限制 |

#### 模式切换示例
```python
from booster_robotics_sdk_python import B1LocoClient, RobotMode

client = B1LocoClient()
client.Init()

# 切换到准备模式
client.ChangeMode(RobotMode.kPrepare)

# 切换到行走模式
client.ChangeMode(RobotMode.kWalking)

# 切换到足球模式（可能支持更高速度）
client.ChangeMode(RobotMode.kSoccer)

# 获取当前模式
from booster_robotics_sdk_python import GetModeResponse
response = GetModeResponse()
client.GetMode(response)
print(f"当前模式: {response.mode}")
```

### 身体控制模式 (BodyControl)

| 模式 | 说明 |
|------|------|
| `kHumanlikeGait` | 人形步态 - 像人一样行走 |
| `kSoccerGait` | ⚡ **足球步态** - 移动更快，可执行足球运动员动作 |
| `kProneBody` | 俯卧控制 - 躺下、俯卧撑 |
| `kWholeBodyDance` | 全身舞蹈 |
| `kShoot` | 射门动作 |
| `kGoalie` | 守门员动作 |

---

## 控制频率与速度关系

### 控制频率说明

```python
CONFIG = {
    "CONTROL_HZ": 20,  # 控制频率 (Hz)
}
```

| 问题 | 答案 |
|------|------|
| **控制频率是否影响实际速度？** | **否**，控制频率不会改变机器人的实际移动速度 |
| **控制频率的作用** | 决定发送运动指令的频率，影响控制的平滑度和响应性 |
| **推荐频率** | 10-50 Hz，通常 20 Hz 足够 |

### 详细解释

- `CONTROL_HZ = 20` 表示每秒发送 20 次 `Move()` 指令
- 机器人接收到指令后，会按照指令中的速度值持续运动
- **更高的频率** → 更平滑的控制、更快的响应
- **更低的频率** → 可能导致运动不连续或响应延迟

> 💡 **建议**：保持 20 Hz 的控制频率，这是大多数应用的最佳平衡点。

---

## move.py 参数详解

### 完整配置参数表

```python
CONFIG = {
    # Rerun 可视化配置
    "RERUN_IP": "192.168.30.99", 
    "RERUN_PORT": 9876,

    # 正方形行走参数（秒）
    "FORWARD_TIME": 3.0,      # 前进持续时间
    "RIGHT_TIME": 3.0,        # 向右走持续时间
    "BACKWARD_TIME": 3.0,     # 后退持续时间
    "LEFT_TIME": 3.0,         # 向左走持续时间
    "PAUSE_BETWEEN": 0,       # 动作间暂停时间

    # 正方形模式速度参数 (m/s)
    "SPEED_FORWARD": 0.3,     # 前进速度
    "SPEED_SIDEWAYS": 0.3,    # 侧向速度
    "SPEED_BACKWARD": -0.3,   # 后退速度

    # 圆弧行走参数
    "CIRCLE_FORWARD_SPEED": 10,   # ⚠️ 走圈前进速度 (被固件限速)
    "CIRCLE_TURN_SPEED": 2.0,     # 走圈转向角速度 (rad/s)
    "CIRCLE_DURATION": 5.0,       # 每个圆弧持续时间（秒）
    "TURN_180_SPEED": 2.0,        # 180度转身角速度 (rad/s)
    "TURN_180_DURATION": 5,       # 180度转身时间

    # 控制频率
    "CONTROL_HZ": 20,             # 控制频率 (Hz)
}
```

### 参数限制说明

| 参数类型 | SDK限制 | 固件限制 | 说明 |
|---------|---------|---------|------|
| 前进速度 vx | **无限制** | **有限制** | SDK 不限速，固件会限制在安全范围 |
| 侧向速度 vy | **无限制** | **有限制** | 同上 |
| 转向速度 vyaw | **无限制** | **有限制** | 同上 |
| 控制频率 | 无限制 | N/A | 建议 10-50 Hz |

### 行走模式选择

```python
# 在 move.py 中直接修改这里选择模式：
WALK_MODE = "left_turn_right"  # ← 修改此值

# 可选模式：
# "square"          - 正方形行走
# "left_circle"     - 走左圈
# "right_circle"    - 走右圈  
# "left_turn_right" - 左圈 → 180°转身 → 右圈
```

---

## 手柄控制说明

### 手柄状态读取

SDK 提供 `RemoteControllerState` 结构体读取手柄状态：

| 字段 | 类型 | 说明 |
|------|------|------|
| `lx`, `ly` | float | 左摇杆 X/Y |
| `rx`, `ry` | float | 右摇杆 X/Y |
| `a`, `b`, `x`, `y` | bool | ABXY 按钮 |
| `lb`, `rb` | bool | 左/右肩键 |
| `lt`, `rt` | bool | 左/右扳机 |
| `start`, `back` | bool | 开始/返回键 |
| `hat_u/d/l/r` | bool | 十字键方向 |

### ✅ 手柄组合键切换自动模式

SDK 支持通过组合键切换自动/手动模式：

**组合键**: `LT` + `LB` + `十字上(hat_u)`

```python
# 使用 auto_control.py 程序
python auto_control.py

# 自定义速度参数
python auto_control.py --vx 0.5 --vyaw 0.5

# 指定网络接口
python auto_control.py --network eth0
```

| 功能 | 操作 |
|------|------|
| 切换自动/手动模式 | 同时按下 LT + LB + 十字上 |
| 自动模式 | 机器人按设定速度持续运动 |
| 手动模式 | 机器人停止，等待下次切换 |

### ❌ 手柄超级控制

> **SDK 不支持"超级控制"功能**

- SDK 只能读取手柄状态，不能解除速度限制
- 速度限制在**机器人固件端**实现
- 如需解除限制，需联系 Booster Robotics 官方

---

## API 参考

### 高级 API (B1LocoClient)

| 方法 | 参数 | 说明 |
|------|------|------|
| `Move(vx, vy, vyaw)` | 速度值 | 控制机器人移动 |
| `ChangeMode(mode)` | RobotMode | 切换机器人模式 |
| `GetMode(response)` | GetModeResponse | 获取当前模式 |
| `GetStatus(response)` | GetStatusResponse | 获取当前状态 |
| `RotateHead(pitch, yaw)` | 角度 (rad) | 旋转头部 |
| `RotateHeadWithDirection(pitch_dir, yaw_dir)` | 方向 (-1/0/1) | 按方向旋转头部 |
| `MoveHandEndEffectorV2(posture, time_ms, hand)` | 位姿、时间、手部索引 | 移动手臂末端 |
| `ControlGripper(param, mode, hand)` | 运动参数、模式、手部索引 | 控制夹爪 |
| `ControlDexterousHand(fingers, hand, type)` | 手指参数、手部索引、手类型 | 控制灵巧手 |
| `WaveHand(action)` | HandAction | 挥手 |
| `Handshake(action)` | HandAction | 握手 |
| `Dance(dance_id)` | DanceId | 执行舞蹈 |
| `WholeBodyDance(dance_id)` | WholeBodyDanceId | 全身舞蹈 |
| `LieDown()` | - | 躺下 |
| `GetUp()` | - | 起身 |
| `GetUpWithMode(mode)` | RobotMode | 起身到指定模式 |
| `Shoot()` | - | 踢球 |
| `PlaySound(path)` | 文件路径 | 播放声音 |
| `StopSound()` | - | 停止声音 |
| `ZeroTorqueDrag(enable)` | bool | 零力矩拖拽 |
| `RecordTrajectory(enable)` | bool | 录制轨迹 |
| `ReplayTrajectory(path)` | 文件路径 | 回放轨迹 |

### 低级 API (底层订阅)

| 类 | 说明 |
|-----|------|
| `B1LowStateSubscriber` | 订阅低级状态（IMU、电机状态）|
| `B1OdometerStateSubscriber` | 订阅里程计数据 |
| `B1LowCmdPublisher` | 发布低级控制指令 |
| `B1LowHandDataScriber` | 订阅手部数据 |
| `B1LowHandTouchDataScriber` | 订阅手部触觉数据 |

---

## 代码示例

### 完整运动控制示例

```python
from booster_robotics_sdk_python import (
    B1LocoClient, ChannelFactory, RobotMode,
    B1LowStateSubscriber, B1OdometerStateSubscriber
)
import time

# 初始化
ChannelFactory.Instance().Init(0, "127.0.0.1")
client = B1LocoClient()
client.Init()

# 切换到行走模式
client.ChangeMode(RobotMode.kWalking)
time.sleep(1)

# 前进 3 秒
for _ in range(60):  # 20Hz * 3秒
    client.Move(0.3, 0.0, 0.0)  # vx=0.3 m/s
    time.sleep(0.05)

# 原地转向 2 秒
for _ in range(40):  # 20Hz * 2秒
    client.Move(0.0, 0.0, 0.5)  # vyaw=0.5 rad/s
    time.sleep(0.05)

# 停止
client.Move(0.0, 0.0, 0.0)
```

### 传感器数据采集示例

```python
from booster_robotics_sdk_python import (
    ChannelFactory, B1LowStateSubscriber, B1OdometerStateSubscriber
)
import time

def imu_handler(msg):
    imu = msg.imu_state
    print(f"IMU - RPY: [{imu.rpy[0]:.3f}, {imu.rpy[1]:.3f}, {imu.rpy[2]:.3f}]")
    print(f"    Gyro: [{imu.gyro[0]:.3f}, {imu.gyro[1]:.3f}, {imu.gyro[2]:.3f}]")
    print(f"    Acc:  [{imu.acc[0]:.3f}, {imu.acc[1]:.3f}, {imu.acc[2]:.3f}]")

def odom_handler(msg):
    print(f"Odometer - x: {msg.x:.3f}, y: {msg.y:.3f}, theta: {msg.theta:.3f}")

ChannelFactory.Instance().Init(0)

imu_sub = B1LowStateSubscriber(imu_handler)
imu_sub.InitChannel()

odom_sub = B1OdometerStateSubscriber(odom_handler)
odom_sub.InitChannel()

while True:
    time.sleep(1)
```

---

## 总结

### SDK 功能支持表

| 功能 | 支持状态 | 说明 |
|------|---------|------|
| 陀螺仪/IMU 数据读取 | ✅ 支持 | rpy, gyro, acc |
| 里程计/速度读取 | ✅ 支持 | x, y, theta |
| 电机状态读取 | ✅ 支持 | q, dq, ddq, tau_est |
| 模式切换 | ✅ 支持 | 5种模式 |
| 运动控制 | ✅ 支持 | vx, vy, vyaw |
| 手臂/夹爪控制 | ✅ 支持 | 末端位姿控制 |
| 灵巧手控制 | ✅ 支持 | 6自由度手指控制 |
| 舞蹈/动作 | ✅ 支持 | 多种预设动作 |
| 轨迹录制/回放 | ✅ 支持 | 记录和重放运动 |
| 手柄超级控制 | ❌ 不支持 | 限速在固件端 |
| SDK 端速度限制 | ❌ 无 | 固件端限速 |

---

## License

本项目采用 Apache License 2.0 许可证。

使用的第三方库：
- fastDDS (Apache License 2.0)
- pybind11 (BSD 3-Clause License)
- pybind11-stubgen (MIT License)
