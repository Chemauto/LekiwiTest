# LekiwiTest - 移动机器人仿真与控制系统 / Mobile Robot Simulation & Control System

## 📖 项目概述 / Project Overview

本项目是一个完整的移动机器人控制框架，基于 MuJoCo 物理仿真引擎，提供从仿真到实际硬件控制的全方位解决方案。项目支持多种移动平台，包括双轮差速小车和三麦克纳姆轮全向移动平台，并实现了基于 PID 的全局导航系统。

This project is a comprehensive mobile robot control framework built on MuJoCo physics simulation engine, providing solutions from simulation to actual hardware control. It supports multiple mobile platforms including 2-wheel differential drive and 3-wheel omnidirectional platforms, with PID-based global navigation system.

### 核心特性 / Key Features
- 🔧 **多种平台支持** - 双轮差速 & 三麦克纳姆轮全向移动平台
- 🎯 **双层控制架构** - 底层速度控制 + 高层 PID 导航
- 🌍 **全局坐标系导航** - 基于航位推算的点对点导航
- 🎮 **多种控制方式** - 键盘控制、轨迹跟踪、路径规划
- 🔌 **硬件接口** - 支持实际机器人控制（Feetech STS3215 舵机）
- 📚 **完整文档** - 详细的使用说明和故障排除指南

---

## 📂 项目结构 / Project Structure

```
LekiwiTest/
├── model/                          # 仿真模型和场景
│   ├── car.xml                     # 双轮差速小车模型
│   └── assets/                     # 三麦克纳姆轮平台模型
│       ├── scene.xml               # 全向机器人场景
│       └── lekiwi/                 # LeKiwi 机器人资源文件
│
├── Mujoco4CarTest/                 # 双轮差速小车 (2WD)
│   ├── control.py                  # 底层控制器
│   ├── keyboard.py                 # 键盘控制接口
│   └── demo.py                     # 自动演示脚本
│
├── Mujoco4Nano/                    # 三麦克纳姆轮全向平台 (3WD Omni) ⭐
│   ├── omni_controller.py          # 全向轮运动学控制器
│   ├── global_navigator.py         # 全局导航控制器（PID）
│   ├── test_omni_viewer.py         # 可视化测试界面
│   ├── test_global_navigation.py   # 导航功能测试
│   └── README_OMNI.md              # 详细文档（555行）
│
├── RealBase/                       # 实际硬件控制接口
│   ├── motor_controller.py         # 舵机控制器
│   ├── calibration_gui.py          # 电机标定工具
│   ├── motors/                     # Feetech 舵机驱动
│   └── requirements.txt            # 硬件依赖
│
├── requirements.txt                # 仿真环境依赖
└── README.md                       # 本文件
```

---

## 🚀 快速开始 / Quick Start

### 环境配置 / Environment Setup

#### 仿真环境 / Simulation Environment
```bash
# 安装基础依赖
pip install -r requirements.txt

# 主要依赖
# - mujoco >= 3.0.0  # 物理仿真引擎
# - numpy >= 1.21.0  # 数值计算
# - scipy >= 1.7.0   # 空间变换
```

#### 硬件控制环境 / Hardware Control Environment
```bash
cd RealBase
pip install -r requirements.txt

# 额外依赖
# - pyserial           # 串口通信
# - SCServo-SDK        # Feetech 舵机 SDK
```

---

## 🎮 使用示例 / Usage Examples

### 1️⃣ 双轮差速小车 (2WD Differential Drive)

简单的前进+转向控制，适合基础学习。

```bash
cd Mujoco4CarTest

# 键盘控制
python keyboard.py

# 自动演示（圆形轨迹）
python demo.py
```

**控制键 / Controls:**
- `w/i`: 前进 / Forward
- `s/k`: 后退 / Backward
- `a/j`: 左转 / Turn left
- `d/l`: 右转 / Turn right
- `Space`: 停止 / Stop
- `q`: 退出 / Quit

**代码示例 / Code Example:**
```python
from control import CarController
import mujoco

# 加载模型
model = mujoco.MjModel.from_xml_path('model/car.xml')
data = mujoco.MjData(model)

# 创建控制器
controller = CarController(model, data)

# 控制：前进速度 0.5，转向 0.3
controller.set_control(forward=0.5, turn=0.3)
controller.apply_control()

# 仿真步进
mujoco.mj_step(model, data)
```

---

### 2️⃣ 三麦克纳姆轮全向平台 (3WD Omnidirectional Platform) ⭐

**推荐使用** - 支持全向移动和全局导航。

```bash
cd Mujoco4Nano

# 基础运动测试
python test_kinematics.py

# 选择演示模式:
# 1 - 全向运动自动演示（9种运动模式）
# 2 - 圆形轨迹跟踪
# 3 - 方形轨迹跟踪
# 4 - 简单演示
# 5 - 手动控制（自定义参数）

# 全局导航测试
python test_global_navigation.py

# 选择导航模式:
# 1 - 单目标点导航
# 2 - 多路径点导航
# 3 - 带朝向控制的导航
```

#### 双层控制架构 / Two-Layer Architecture

**底层：运动学控制器 / Low-Level Controller**
```python
from omni_controller import OmniWheelController

controller = OmniWheelController(model, data)

# 全向移动：前后左右 + 旋转
# vx, vy: 全局坐标系下的速度分量
# omega: 旋转角速度
controller.set_velocity(
    linear_speed=0.5,
    vx=1.0,  # 前进
    vy=0.0,  # 无侧向
    omega=0  # 无旋转
)
```

**高层：导航控制器 / High-Level Navigator**
```python
from global_navigator import GlobalNavigator

navigator = GlobalNavigator(model, data)

# 设置目标点（全局坐标系）
navigator.set_target(
    x=2.0,      # 目标 x 坐标（米）
    y=1.5,      # 目标 y 坐标（米）
    theta=0.0   # 目标朝向（弧度），可选
)

# 更新导航（自动计算速度输出）
navigator.update()
```

**主要特性 / Key Features:**
- ✅ **3 DOF 全向移动** - 前后左右平移 + 原地旋转
- ✅ **全局坐标系导航** - 基于航位推算的点对点移动
- ✅ **PID 位置控制** - 精确的位置和朝向控制
- ✅ **多路径点规划** - 支持路径点序列导航
- ✅ **实时可视化** - MuJoCo 3D 仿真界面

**详细文档:** 查看 [`Mujoco4Nano/README_OMNI.md`](Mujoco4Nano/README_OMNI.md) 获取完整 API 文档、参数调优指南和故障排除。

---

### 3️⃣ 硬件控制 (Real Hardware Control)

连接实际的 LeKiwi 机器人（配备 Feetech STS3215 舵机）。

```bash
cd RealBase

# 电机标定工具
python calibration_gui.py

# 运行硬件控制器（需要根据实际配置修改）
python motor_controller.py
```

**硬件配置 / Hardware Specs:**
- **平台**: 三麦克纳姆轮全向移动底盘
- **执行器**: 3 × Feetech STS3215 舵机
- **通信**: 串口通信（SCServo 协议）
- **控制精度**: 0-4095 位置分辨率

---

## 📊 平台对比 / Platform Comparison

| 特性 / Feature | 2WD 差速 / Differential | 3WD 全向 / Omnidirectional |
|----------------|------------------------|---------------------------|
| **移动方式** | 前进 + 转向 | 前后左右 + 原地旋转 |
| **运动自由度** | 2 DOF | 3 DOF (全向) |
| **执行器数量** | 2 | 3 |
| **控制复杂度** | 简单 | 中等 |
| **适用场景** | 基础学习、简单任务 | 精确定位、狭窄空间、全向移动 |
| **导航支持** | ❌ 仅手动控制 | ✅ 全局坐标系导航 |
| **代码位置** | `Mujoco4CarTest/` | `Mujoco4Nano/` ⭐ |

---

## 🔧 技术细节 / Technical Details

### 运动学模型 / Kinematics

#### 双轮差速 / Differential Drive
```
v_left  = v_forward - v_turn * wheel_base / 2
v_right = v_forward + v_turn * wheel_base / 2
```

#### 三麦克纳姆轮 / 3-Wheel Omni
基于 LeKiwi 官方运动学实现，三个全向轮呈 120° 均匀分布：

```
[v1]   [ -sin(θ1)  cos(θ1)  R ] [vx]
[v2] = [ -sin(θ2)  cos(θ2)  R ] [vy]
[v3]   [ -sin(θ3)  cos(θ3)  R ] [ω]

其中 θ1=0°, θ2=120°, θ3=240°
R 为机器人半径
```

### PID 导航控制器 / PID Navigator

**位置控制 / Position Control:**
```python
# 计算位置误差
error_x = target_x - current_x
error_y = target_y - current_y

# PID 控制输出
output_vx = Kp_x * error_x + Kd_x * error_vx
output_vy = Kp_y * error_y + Kd_y * error_vy
```

**朝向控制 / Orientation Control:**
```python
# 计算角度误差（考虑角度环绕）
error_theta = target_theta - current_theta
error_theta = (error_theta + np.pi) % (2 * np.pi) - np.pi

# PID 控制输出
output_omega = Kp_theta * error_theta
```

**系统参数 / System Parameters:**
- 机器人半径: 0.15 m
- 轮子半径: 0.03 m
- 最大速度: 0.5 m/s
- PID 参数可在 `global_navigator.py` 中调整

---

## 📚 详细文档 / Documentation

- **[Mujoco4Nano/README_OMNI.md](Mujoco4Nano/README_OMNI.md)** - 三麦克纳姆轮平台完整文档（555行）
  - 双层控制架构详解
  - 完整 API 参考和代码示例
  - 系统参数调优指南
  - 故障排除和常见问题
  - 运动学推导和实现细节

- **[RealBase/README.md](RealBase/README.md)** - 硬件控制接口文档

---

## 🛠️ 依赖要求 / Requirements

### 仿真环境 / Simulation
```
mujoco >= 3.0.0
numpy >= 1.21.0
scipy >= 1.7.0
```

### 硬件控制 / Hardware Control
```
pyserial
SCServo-SDK (Feetech)
```

### 可选依赖 / Optional
```
draccus     # 配置文件解析
deepdiff    # 数据对比
tqdm        # 进度条
```

---

## 📖 参考资料 / References

- **MuJoCo 文档**: https://mujoco.readthedocs.io/
- **LeKiwi 项目**: `/home/dora/RoboOs/New/doralekiwi/lekiwi`
- **Feetech 舵机**: https://www.feetechrc.com/

---

## 🎯 应用场景 / Applications

- 🎓 **机器人教育** - 学习控制理论和机器人学
- 🔬 **算法开发** - 在仿真中测试控制算法
- 📈 **导航研究** - PID 控制调优和路径规划
- 🤖 **竞赛机器人** - 全向移动机器人控制
- 🏭 **实际应用** - 仓储物流、服务机器人

---

## 📝 更新日志 / Changelog

### 主要功能更新
- **2026-01-16**: 增加全局坐标系导航系统（基于航位推算的 PID 控制）
- **2026-01-15**: 完善三麦克纳姆轮平台仿真和文档
- **2026-01-14**: 添加双轮差速小车基础控制
- **2026-01-13**: 项目初始化，添加 MuJoCo 模型

---

**最后更新 / Last Updated**: 2026-01-16

**维护者 / Maintainer**: Lekiwi Development Team

**许可证 / License**: 详见项目根目录 LICENSE 文件
