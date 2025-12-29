# Booster Robotics SDK

<p align="center">
  <b>🌐 Language / 语言切换</b><br>
  <a href="#english">English</a> | <a href="#中文">中文</a>
</p>

---

<a name="english"></a>
# 🇺🇸 English

This repository is a community-driven supplement to the original [Booster Robotics SDK](https://github.com/BoosterRobotics/booster_robotics_sdk). It aims to provide more detailed API explanations and practical examples for developers.

### ✨ Additional Features (Not in Official SDK)

| Feature | Description |
|---------|-------------|
| Gamepad State Subscription | `B1RemoteControllerStateSubscriber` for reading controller input |
| RC Simulation Publisher | `B1RemoteControllerStatePublisher` for simulating remote controller |
| Max-Speed Circle Walking | `move_rc_sim.py` - achieves max speed during arc walking |
| Sensor Monitoring | `sensor_monitor.py` - real-time IMU/odometer/motor display |
| Speed Comparison | `speed_monitor.py` - compare RC speed vs program speed |
| RC State Monitor | `rc_monitor.py` - view raw controller messages |
| Closed-Loop Circle Walk | `circle_walk.py` - gyro + odometer feedback control |
| Gamepad Combo Controls | LT+RT+D-pad for mode switching |

> 🔗 **Official SDK**: [https://github.com/BoosterRobotics/booster_robotics_sdk](https://github.com/BoosterRobotics/booster_robotics_sdk)

## Table of Contents

- [Requirements](#requirements)
- [Installation](#installation)
- [Core Programs](#core-programs)
- [SDK API Reference](#sdk-api-reference)
- [Quick Start](#quick-start)
- [License](#license-en)

---

## Requirements

| Item | Requirement |
|------|-------------|
| OS | Ubuntu 22.04 LTS |
| CPU | aarch64 / x86_64 |
| Compiler | gcc 11.4.0 |
| Python | 3.x |

---

## Installation

### 1. Install SDK
```bash
sudo ./install.sh
```

### 2. Install Python Dependencies
```bash
pip3 install pybind11
pip3 install pybind11-stubgen
```

### 3. Build Python Binding
```bash
mkdir build && cd build
cmake .. -DBUILD_PYTHON_BINDING=on
make
sudo make install
```

> If pybind11-stubgen cannot be found:
> ```bash
> export PATH=/home/[user name]/.local/bin:$PATH
> ```

### 4. Build C++ Examples
```bash
mkdir build && cd build
cmake ..
make
```

---

## Core Programs

| Program | Function | Control | Command |
|---------|----------|---------|---------|
| `move_rc_sim.py` | **Recommended** RC simulation circle walk | Gamepad (LT+RT+D-pad) | `sudo bash -c "source /opt/ros/humble/setup.bash && python3 move_rc_sim.py"` |
| `move.py` | Basic motion control (square/arc) | Gamepad/Keyboard | `python3 move.py` |
| `circle_walk.py` | Closed-loop circle walk + return | Gyro + Odometer | `python3 circle_walk.py` |
| `sensor_monitor.py` | Sensor data monitor (IMU/Odom/Motor) | Monitor only | `python3 sensor_monitor.py` |
| `speed_monitor.py` | Speed comparison (RC vs Program) | Monitor only | `python3 speed_monitor.py` |
| `rc_monitor.py` | Remote controller state monitor | Monitor only | `python3 rc_monitor.py` |

### Difference Between move.py and move_rc_sim.py

| Comparison | `move.py` | `move_rc_sim.py` |
|------------|-----------|------------------|
| **Control Method** | Uses `Move(vx, vy, vyaw)` API | Uses `RemoteControllerStatePublisher` to simulate RC |
| **Speed Performance** | Cannot reach max speed during circle walk | Can reach the same max speed as real RC |
| **Principle** | Like using only the right joystick (diagonal) | Simulates left stick forward + right stick turn (separate control) |
| **Circle Effect** | Spins in place when setting vx and vyaw together | Normal circle walk, no spinning |
| **Recommended For** | Simple tests, square walk | **Circle walk, max speed scenarios** |

> ⚠️ **Key Difference**:
> 
> The `Move(vx, vy, vyaw)` command is equivalent to the **right joystick diagonal push** on the remote controller, which can control forward and turning. However, when walking in a circle, due to internal control logic differences, **it cannot achieve maximum speed**.
>
> If you want to **turn and move forward at maximum speed simultaneously**, you need to use the **virtual gamepad method** in the SDK, fully simulating the "left stick forward + right stick turn only" control method. See `move_rc_sim.py` for implementation.

### Monitoring Programs

| Program | Function | Command |
|---------|----------|---------|
| `sensor_monitor.py` | Real-time IMU, odometer, motor status | `python3 sensor_monitor.py` |
| `speed_monitor.py` | Compare RC speed vs program speed | `python3 speed_monitor.py` |
| `rc_monitor.py` | View raw RC messages (event, joysticks, buttons) | `python3 rc_monitor.py` |

---

## SDK API Reference

### Import SDK

```python
from booster_robotics_sdk_python import (
    B1LocoClient,          # High-level motion control client
    ChannelFactory,        # Communication channel factory
    RobotMode,             # Robot mode enum
)
```

### High-Level API (`B1LocoClient`)

#### Initialization

```python
from booster_robotics_sdk_python import B1LocoClient, ChannelFactory

ChannelFactory.Instance().Init(0, "127.0.0.1")  # Domain ID, IP
client = B1LocoClient()
client.Init()
```

#### Motion Control

| Method | Parameters | Description |
|--------|------------|-------------|
| `Move(vx, vy, vyaw)` | vx (m/s), vy (m/s), vyaw (rad/s) | Control robot movement |
| `ChangeMode(mode)` | RobotMode enum | Switch robot mode |
| `GetMode(response)` | GetModeResponse | Get current mode |
| `GetStatus(response)` | GetStatusResponse | Get current status |

> ⚠️ **Important**:
> - SDK has **no speed limits**, actual limits are enforced by robot **firmware**
> - `Move()` is like the **right joystick**, can control forward and turning
> - When **walking in a circle**, setting vx and vyaw together **cannot achieve max speed**
> - For max speed circle walk, use virtual gamepad method in `move_rc_sim.py`

### Robot Modes (`RobotMode`)

| Mode | Value | Description |
|------|-------|-------------|
| `kDamping` | 0 | Damping mode - motors in damping state |
| `kPrepare` | 1 | Prepare mode - standing ready |
| `kWalking` | 2 | Walking mode - can move and rotate |
| `kCustom` | 3 | Custom mode |
| `kSoccer` | 4 | Soccer mode - may have higher speed limits |

### Sensor Subscribers

```python
from booster_robotics_sdk_python import (
    B1LowStateSubscriber,           # Low-level state (IMU, motors)
    B1OdometerStateSubscriber,      # Odometer
    B1RemoteControllerStateSubscriber,  # Gamepad state
)
```

### Remote Controller Simulation

```python
from booster_robotics_sdk_python import (
    B1RemoteControllerStatePublisher,
    RemoteControllerState
)

publisher = B1RemoteControllerStatePublisher()
publisher.InitChannel()

rc_msg = RemoteControllerState()
rc_msg.event = 1536    # ★ Critical: must be 1536
rc_msg.ly = -1.0       # Left stick Y: negative = forward
rc_msg.rx = -0.5       # Right stick X: negative = turn left

publisher.Write(rc_msg)
```

### Gamepad Combo Keys

| Combo | Function |
|-------|----------|
| **LT + RT + D-pad Up** | Pause/Resume (toggle manual/auto mode) |
| **LT + RT + D-pad Left** | Switch to left circle mode (pause first) |
| **LT + RT + D-pad Right** | Switch to right circle mode (pause first) |
| **LT + RT + D-pad Down** | Switch to straight walk mode (pause first) |

---

## Quick Start

### RC Simulation Circle Walk (Recommended)

```bash
cd /Downloads/booster_robotics_sdk
sudo bash -c "source /opt/ros/humble/setup.bash && python3 move_rc_sim.py"
```

### Force Stop Program

```bash
sudo pkill -9 -f move_rc_sim.py
```

---

## Feature Support

| Feature | Status | Notes |
|---------|--------|-------|
| Motion Control (vx, vy, vyaw) | ✅ | Move() API |
| IMU/Gyroscope Data | ✅ | rpy, gyro, acc |
| Odometer Data | ✅ | x, y, theta |
| Motor Status | ✅ | q, dq, ddq, tau_est |
| Mode Switching | ✅ | 5 robot modes |
| Gamepad State Reading | ✅ | Full buttons and sticks |
| RC Simulation | ✅ | Circle/straight walk |
| Arm/Gripper Control | ✅ | End-effector pose control |
| Dexterous Hand | ✅ | 6-DOF finger control |
| Dance/Actions | ✅ | Multiple preset actions |
| Trajectory Recording | ✅ | Record and playback |
| SDK Speed Limits | ❌ | Limits in firmware |

---

<a name="license-en"></a>
## License

This project is licensed under the Apache License, Version 2.0. See the [LICENSE](LICENSE) file for details.

Third-party libraries:
- fastDDS (Apache License 2.0)
- pybind11 (BSD 3-Clause License)
- pybind11-stubgen (MIT License)

---

<p align="center">
  <a href="#english">⬆️ Back to Top (English)</a>
</p>

---
---

<a name="中文"></a>
# 🇨🇳 中文

本项目是针对 [Booster Robotics SDK](https://github.com/BoosterRobotics/booster_robotics_sdk) 的二次开发与内容补充。旨在原版基础上提供更详尽的 API 说明及程序示例，方便开发者学习参考。

### ✨ 新增功能（官方 SDK 未提供）

| 功能 | 说明 |
|------|------|
| 手柄状态订阅 | `B1RemoteControllerStateSubscriber` 读取手柄输入 |
| 遥控器模拟发布 | `B1RemoteControllerStatePublisher` 模拟遥控器输入 |
| 最大速度走圆 | `move_rc_sim.py` - 实现弧线行走时的最大速度 |
| 传感器监控 | `sensor_monitor.py` - 实时显示 IMU/里程计/电机状态 |
| 速度对比 | `speed_monitor.py` - 对比遥控器与程序速度 |
| 遥控器监控 | `rc_monitor.py` - 查看原始遥控器消息 |
| 闭环圆形行走 | `circle_walk.py` - 陀螺仪+里程计反馈控制 |
| 手柄组合键控制 | LT+RT+方向键 切换模式 |

> 🔗 **官方 SDK**: [https://github.com/BoosterRobotics/booster_robotics_sdk](https://github.com/BoosterRobotics/booster_robotics_sdk)

## 目录

- [环境要求](#环境要求)
- [安装](#安装)
- [核心程序](#核心程序)
- [SDK API 接口](#sdk-api-接口)
- [快速开始](#快速开始)
- [许可证](#许可证)

---

## 环境要求

| 项目 | 要求 |
|------|------|
| OS | Ubuntu 22.04 LTS |
| CPU | aarch64 / x86_64 |
| Compiler | gcc 11.4.0 |
| Python | 3.x |

---

## 安装

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

> 如果 pybind11-stubgen 找不到，请执行：
> ```bash
> export PATH=/home/[user name]/.local/bin:$PATH
> ```

### 4. 编译 C++ 示例
```bash
mkdir build && cd build
cmake ..
make
```

---

## 核心程序

| 程序 | 功能 | 控制方式 | 运行命令 |
|------|------|----------|----------|
| `move_rc_sim.py` | **推荐** 遥控器模拟走圆 | 手柄(LT+RT+方向键) | `sudo bash -c "source /opt/ros/humble/setup.bash && python3 move_rc_sim.py"` |
| `move.py` | 基础运动控制 (正方形/圆弧) | 手柄/键盘 | `python3 move.py` |
| `circle_walk.py` | 闭环圆形行走 + 回原点 | 陀螺仪+里程计 | `python3 circle_walk.py` |
| `sensor_monitor.py` | 传感器数据监控 (IMU/里程计/电机) | 纯监控 | `python3 sensor_monitor.py` |
| `speed_monitor.py` | 速度对比监控 (遥控器 vs 程序) | 纯监控 | `python3 speed_monitor.py` |
| `rc_monitor.py` | 遥控器状态监控 (摇杆/按键) | 纯监控 | `python3 rc_monitor.py` |

### move.py 与 move_rc_sim.py 的区别

| 对比项 | `move.py` | `move_rc_sim.py` |
|--------|-----------|------------------|
| **控制方式** | 使用 `Move(vx, vy, vyaw)` API | 使用 `RemoteControllerStatePublisher` 模拟遥控器 |
| **速度表现** | 走圆时无法发挥最大速度 | 可达到与真实遥控器相同的最大速度 |
| **原理** | 相当于只用右摇杆（斜打）| 模拟左摇杆前进 + 右摇杆转向（分开控制）|
| **走圆效果** | 同时设置 vx 和 vyaw 时原地转圈 | 正常走圆，不会原地转圈 |
| **推荐场景** | 简单测试、正方形行走 | **走圆、需要最大速度的场景** |

> ⚠️ **关键差异说明**：
> 
> `Move(vx, vy, vyaw)` 指令相当于遥控器的**右摇杆斜打**，可以控制前进和转弯。但在走圆的时候，由于内部控制逻辑的差异，**无法发挥出最大速度**。
>
> 如果想要以**最大速度同时转弯和前进**，需要使用 SDK 中的**虚拟手柄方式**实现，完全模拟真实遥控器的"左摇杆推满 + 右摇杆只打转向"的操控方式。具体实现参考 `move_rc_sim.py`。

### 监控程序说明

| 程序 | 功能 | 运行命令 |
|------|------|----------|
| `sensor_monitor.py` | 实时显示 IMU、里程计、电机状态 | `python3 sensor_monitor.py` |
| `speed_monitor.py` | 对比遥控器速度与程序速度 | `python3 speed_monitor.py` |
| `rc_monitor.py` | 查看遥控器原始消息 (event, 摇杆, 按键) | `python3 rc_monitor.py` |

---

## SDK API 接口

### 导入 SDK

```python
from booster_robotics_sdk_python import (
    B1LocoClient,          # 高级运动控制客户端
    ChannelFactory,        # 通信通道工厂
    RobotMode,             # 机器人模式枚举
)
```

### 高级 API (`B1LocoClient`)

#### 初始化

```python
from booster_robotics_sdk_python import B1LocoClient, ChannelFactory

ChannelFactory.Instance().Init(0, "127.0.0.1")  # Domain ID, IP
client = B1LocoClient()
client.Init()
```

#### 运动控制

| 方法 | 参数 | 说明 |
|------|------|------|
| `Move(vx, vy, vyaw)` | vx (m/s), vy (m/s), vyaw (rad/s) | 控制机器人移动 |
| `ChangeMode(mode)` | RobotMode 枚举 | 切换机器人模式 |
| `GetMode(response)` | GetModeResponse | 获取当前模式 |
| `GetStatus(response)` | GetStatusResponse | 获取当前状态 |

> ⚠️ **重要说明**:
> - SDK 本身**无速度限制**，实际限速由机器人**固件**执行
> - `Move()` 指令相当于遥控器的**右摇杆**，可以控制前进和转弯
> - 但在**走圆**时，同时设置 vx 和 vyaw 会导致**无法发挥最大速度**
> - 如需最大速度走圆，请使用 `move_rc_sim.py` 的虚拟手柄方式

### 机器人模式 (`RobotMode`)

| 模式 | 枚举值 | 说明 |
|------|--------|------|
| `kDamping` | 0 | 阻尼模式 - 电机阻尼状态 |
| `kPrepare` | 1 | 准备模式 - 站立待命 |
| `kWalking` | 2 | 行走模式 - 可移动、旋转 |
| `kCustom` | 3 | 自定义模式 |
| `kSoccer` | 4 | 足球模式 - 可能有更高速度限制 |

### 传感器订阅器

```python
from booster_robotics_sdk_python import (
    B1LowStateSubscriber,           # 低级状态（IMU、电机）
    B1OdometerStateSubscriber,      # 里程计
    B1RemoteControllerStateSubscriber,  # 手柄状态
)
```

### 遥控器模拟接口

```python
from booster_robotics_sdk_python import (
    B1RemoteControllerStatePublisher,
    RemoteControllerState
)

publisher = B1RemoteControllerStatePublisher()
publisher.InitChannel()

rc_msg = RemoteControllerState()
rc_msg.event = 1536    # ★ 关键参数，必须设为1536
rc_msg.ly = -1.0       # 左摇杆Y轴：负值=前进
rc_msg.rx = -0.5       # 右摇杆X轴：负值=左转

publisher.Write(rc_msg)
```

### 手柄组合键操作

| 组合键 | 功能 |
|--------|------|
| **LT + RT + 十字键上** | 暂停/继续（切换手动/自动模式） |
| **LT + RT + 十字键左** | 切换到左圈模式（需先暂停） |
| **LT + RT + 十字键右** | 切换到右圈模式（需先暂停） |
| **LT + RT + 十字键下** | 切换到直走模式（需先暂停） |

---

## 快速开始

### 遥控器模拟走圆（推荐）

```bash
cd /Downloads/booster_robotics_sdk
sudo bash -c "source /opt/ros/humble/setup.bash && python3 move_rc_sim.py"
```

### 强制关闭程序

```bash
sudo pkill -9 -f move_rc_sim.py
```

---

## 功能支持表

| 功能 | 状态 | 说明 |
|------|------|------|
| 运动控制 (vx, vy, vyaw) | ✅ | Move() API |
| IMU/陀螺仪数据 | ✅ | rpy, gyro, acc |
| 里程计数据 | ✅ | x, y, theta |
| 电机状态 | ✅ | q, dq, ddq, tau_est |
| 模式切换 | ✅ | 5种机器人模式 |
| 手柄状态读取 | ✅ | 完整按键和摇杆 |
| 遥控器模拟 | ✅ | 可实现走圆/直走 |
| 手臂/夹爪控制 | ✅ | 末端位姿控制 |
| 灵巧手控制 | ✅ | 6自由度手指 |
| 舞蹈/动作 | ✅ | 多种预设动作 |
| 轨迹录制/回放 | ✅ | 记录和重放运动 |
| SDK端速度限制 | ❌ | 限速在固件端实现 |

---

## 详细文档

| 文档 | 说明 |
|------|------|
| [SDK_使用指南.md](SDK_使用指南.md) | SDK 完整功能和 API 详解 |
| [robot_walk_guide.md](robot_walk_guide.md) | 所有程序的详细设计和配置 |
| [遥控器模拟说明.md](遥控器模拟说明.md) | 遥控器模拟原理和使用方法 |
| [circle_walk_guide.md](circle_walk_guide.md) | 闭环圆形行走详细说明 |

---

## 许可证

本项目采用 Apache License 2.0 许可证。详见 [LICENSE](LICENSE) 文件。

使用的第三方库：
- fastDDS (Apache License 2.0)
- pybind11 (BSD 3-Clause License)
- pybind11-stubgen (MIT License)

---

<p align="center">
  <a href="#中文">⬆️ 返回顶部 (中文)</a> | <a href="#english">🇺🇸 Switch to English</a>
</p>