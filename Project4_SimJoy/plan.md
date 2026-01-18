
/home/robot/work/unitree_mujoco/simulate_python/ 
  config.py参考这个文件,你看它的输入是xbox手柄,我  
  需要你模拟一个手柄出来,要求有一个漂亮的ui界面,和 
  类似xbox的形状,鼠标按下则代表模拟真实的按键按下, 
  项目文件放在/home/robot/work/LekiwiTest/Project4 
  SimJoy这个文件夹下面

# Project4_SimJoy - Xbox手柄模拟器开发计划

## 项目概述

开发一个具有图形用户界面(GUI)的虚拟Xbox手柄模拟器,用于替代真实手柄输入到Unitree机器人控制系统中。该模拟器需要提供与真实Xbox手柄相同的接口和数据格式。

## 参考文件分析

基于 `/home/robot/work/unitree_mujoco/simulate_python/config.py` 和 `unitree_sdk2py_bridge.py` 的分析:

### Xbox手柄按键映射

**按键(button_id):**
- A (button_id=0)
- B (button_id=1)
- X (button_id=2)
- Y (button_id=3)
- LB (button_id=4) - 左肩键
- RB (button_id=5) - 右肩键
- SELECT (button_id=6) - 选择键
- START (button_id=7) - 开始键

**摇杆和扳机轴(axis_id):**
- LX (axis_id=0) - 左摇杆X轴 [-1, 1]
- LY (axis_id=1) - 左摇杆Y轴 [-1, 1]
- LT (axis_id=2) - 左扳机 [0, 1]
- RX (axis_id=3) - 右摇杆X轴 [-1, 1]
- RY (axis_id=4) - 右摇杆Y轴 [-1, 1]
- RT (axis_id=5) - 右扳机 [0, 1]
- 方向键(hat) - 上下左右四个方向

### 输出数据格式

模拟器需要提供以下接口,兼容pygame的joystick API:
- `get_button(button_id)` - 获取按键状态(0/1)
- `get_axis(axis_id)` - 获取轴值(浮点数)
- `get_hat(0)` - 获取方向键状态(元组)

## 技术方案

### GUI框架选择
**推荐: PyQt5 / PyQt6**
- 优点: 强大的绘图能力,支持复杂的UI设计,跨平台
- 可以绘制高质量的2D图形来模拟手柄外观
- 事件处理机制完善,支持鼠标拖拽实现摇杆控制
- 可以实时更新界面显示按键状态

### UI设计方案

#### 手柄布局
```
┌─────────────────────────────────────────┐
│  [LB]                    [RB]           │
│                                         │
│      [LT]              [RT]             │
│                                         │
│    ┌─────┐                  ┌─────┐     │
│    │  Y  │                  │  Y  │     │
│    ┌─────┐                  ┌─────┐     │
│  [X] [Y] [B]              [X] [Y] [B]   │
│    ┌─────┐                  ┌─────┐     │
│    │  X  │                  │  A  │     │
│    └─────┘                  └─────┘     │
│      [A]              [D-PAD]           │
│                                         │
│    [Left Stick]      [Right Stick]      │
│       (  )              (  )           │
│                                         │
│           [SELECT] [START]              │
└─────────────────────────────────────────┘
```

#### 组件详细设计

1. **左上方区域**
   - LB按钮: 圆形按钮,点击触发
   - LT扳机: 半圆形或椭圆,点击触发(模拟压力)

2. **右上方区域**
   - RB按钮: 圆形按钮,点击触发
   - RT扳机: 半圆形或椭圆,点击触发(模拟压力)

3. **左方ABXY按键区**
   - 4个圆形按钮呈菱形排列
   - A(下/绿)、B(右/红)、X(左/蓝)、Y(上/黄)

4. **右方ABXY按键区**
   - 同样的布局(可选双模式)

5. **方向键(D-PAD)**
   - 十字型,支持上/下/左/右四个方向

6. **左摇杆**
   - 圆形底座 + 可移动的摇杆头
   - 鼠标拖拽实现双轴控制
   - 回弹到中心
   - 显示当前位置

7. **右摇杆**
   - 同左摇杆

8. **中间功能键**
   - SELECT(左)和START(右)小圆形按钮

9. **状态显示区**
   - 实时显示所有按键和轴的当前值
   - 用于调试和验证

### 交互设计

1. **按键交互**
   - 鼠标按下: 按键变为激活状态(颜色变化/下压动画)
   - 鼠标释放: 按键恢复原状
   - 视觉反馈: 激活时高亮显示

2. **摇杆交互**
   - 鼠标在摇杆区域按下: 捕获摇杆控制
   - 拖拽: 摇杆头跟随鼠标移动
   - 释放: 摇杆回弹到中心
   - 计算X/Y轴值: 根据偏离中心的距离归一化到[-1, 1]

3. **扳机交互**
   - 点击触发,可以按住保持激活
   - 可选: 实现压力模拟(根据点击位置)

4. **键盘快捷键(可选)**
   - WASD - 左摇杆
   - 方向键 - 右摇杆或D-PAD
   - J/K/L/I - ABXY按键

## 实现步骤

### Phase 1: 基础框架搭建
1. 创建项目目录结构
2. 设置PyQt5项目基础
3. 定义手柄数据结构和状态管理类
4. 实现与pygame兼容的接口类

### Phase 2: UI界面实现
1. 设计并绘制手柄轮廓
2. 实现各个按钮组件
3. 实现摇杆组件(包括拖拽逻辑)
4. 实现扳机和方向键组件
5. 添加状态显示面板

### Phase 3: 交互逻辑实现
1. 实现鼠标事件处理(按下/释放/移动)
2. 实现摇杆的拖拽计算和回弹逻辑
3. 实现按键状态切换
4. 实现键盘快捷键支持(可选)

### Phase 4: 数据接口实现
1. 实现get_button()接口
2. 实现get_axis()接口
3. 实现get_hat()接口
4. 确保数据格式与pygame一致

### Phase 5: 集成测试
1. 创建测试脚本,模拟使用场景
2. 测试所有按键和轴的功能
3. 验证与unitree_sdk2py_bridge的兼容性
4. 性能优化(延迟、响应速度)

### Phase 6: 文档和发布
1. 编写使用说明
2. 添加示例代码
3. 创建requirements.txt
4. 编写README.md

## 项目结构

```
Project4_SimJoy/
├── plan.md                    # 本文件
├── README.md                  # 项目说明
├── requirements.txt           # 依赖列表
├── src/
│   ├── __init__.py
│   ├── main.py               # 程序入口
│   ├── virtual_joystick.py   # 核心手柄类(提供pygame兼容接口)
│   ├── joystick_ui.py        # GUI界面类
│   ├── joystick_state.py     # 手柄状态管理
│   └── components/           # UI组件
│       ├── button.py         # 按钮组件
│       ├── stick.py          # 摇杆组件
│       ├── trigger.py        # 扳机组件
│       └── dpad.py           # 方向键组件
├── test/
│   ├── test_joystick.py      # 单元测试
│   └── integration_test.py   # 集成测试(与unitree系统)
└── assets/                   # 资源文件(可选)
    └── icons/                # 图标资源
```

## 技术要点

### 摇杆算法
```python
# 计算摇杆轴值
dx = mouse_x - center_x
dy = mouse_y - center_y
distance = min(sqrt(dx*dx + dy*dy), max_radius)
angle = atan2(dy, dx)

x_value = (distance / max_radius) * cos(angle)
y_value = (distance / max_radius) * sin(angle)
```

### 兼容性设计
- 提供与pygame.joystick.Joystick相同的API
- 可以作为drop-in replacement使用
- 支持动态切换虚拟/真实手柄

### 性能考虑
- 使用定时器定期更新界面(60FPS)
- 事件驱动更新按键状态
- 避免阻塞主线程

## 可选增强功能

1. **配置保存**: 加载/保存按键映射配置
2. **手柄样式切换**: 支持多种手柄外观(Xbox/Switch/PS5)
3. **震动反馈**: 如果支持,提供触觉反馈
4. **宏功能**: 录制和回放按键序列
5. **网络控制**: 支持远程网络控制手柄
6. **触摸支持**: 支持触摸屏设备

## 依赖库

```
PyQt5>=5.15.0
PyQt6>=6.0.0  # 可选,使用PyQt6
pygame>=2.0.0  # 用于兼容性测试
numpy>=1.19.0  # 用于数学计算
```

## 开发时间估算

- Phase 1: 1-2天
- Phase 2: 2-3天
- Phase 3: 2-3天
- Phase 4: 1-2天
- Phase 5: 1-2天
- Phase 6: 1天

总计: 约8-13天(根据功能完整性)

## 成功标准

1. ✅ UI界面美观,符合Xbox手柄布局
2. ✅ 所有按键功能正常
3. ✅ 摇杆拖拽流畅,数据准确
4. ✅ 与unitree_sdk2py_bridge无缝集成
5. ✅ 响应延迟小于50ms
6. ✅ 代码结构清晰,易于维护

## 风险和挑战

1. **摇杆灵敏度**: 需要调整参数以达到最佳手感
2. **性能优化**: 确保GUI更新不影响控制延迟
3. **兼容性**: 确保与现有系统完全兼容
4. **跨平台**: 测试不同操作系统下的表现

## 参考资料

- PyQt5官方文档: https://www.riverbankcomputing.com/static/Docs/PyQt5/
- Xbox手柄布局图
- pygame.joystick文档
- Unitree机器人SDK文档
