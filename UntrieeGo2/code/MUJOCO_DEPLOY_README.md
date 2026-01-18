# MuJoCo策略部署指南

## 概述

本项目用于将IsaacLab中为MuJoCo模型训练的Unitree Go2行走策略部署到MuJoCo仿真环境。

## 策略信息

- **策略文件**: `Rough_Walk_policy4Mujoco.pt`
- **训练环境**: IsaacLab (基于MuJoCo XML配置)
- **训练迭代**: 499
- **网络结构**: 4层 MLP [512, 256, 128, 12]
- **输入维度**: 235
- **输出维度**: 12

## MuJoCo模型配置

### 关节顺序

**Joint定义顺序** (Joint 1-12，跳过freejoint):
```
FL_hip, FL_thigh, FL_calf,    # 1-3  左前腿
FR_hip, FR_thigh, FR_calf,    # 4-6  右前腿
RL_hip, RL_thigh, RL_calf,    # 7-9  左后腿
RR_hip, RR_thigh, RR_calf     # 10-12 右后腿
```

**Actuator顺序** (0-11):
```
FR_hip, FR_thigh, FR_calf,    # 0-2  右前腿
FL_hip, FL_thigh, FL_calf,    # 3-5  左前腿
RR_hip, RR_thigh, RR_calf,    # 6-8  右后腿
RL_hip, RL_thigh, RL_calf     # 9-11 左后腿
```

**注意**: Actuator和Joint的顺序不同！需要映射。

### 默认姿态

从MuJoCo XML keyframe:
```python
基座高度: 0.27m
关节位置: [0, 0.9, -1.8] (hip, thigh, calf) 所有腿相同
四元数: [1, 0, 0, 0] (w, x, y, z)
```

### Actuator到Joint的映射

```python
actuator_to_joint = [
    4, 5, 6,   # FR actuators 0-2 -> joints 4-6
    1, 2, 3,   # FL actuators 3-5 -> joints 1-3
    10, 11, 12,# RR actuators 6-8 -> joints 10-12
    7, 8, 9    # RL actuators 9-11 -> joints 7-9
]
```

## 观测空间 (235维)

```python
观测组成:
- base_lin_vel (3): 基座线速度 [vx, vy, vz]
- base_ang_vel (3): 基座角速度 [wx, wy, wz]
- projected_gravity (3): 投影重力向量 [gx, gy, gz]
- velocity_command (3): 速度命令 [cmd_vx, cmd_vy, cmd_wz]
- joint_pos_rel (12): 关节位置(相对于默认位置)
- joint_vel_rel (12): 关节速度
- last_action (12): 上一个动作
- height_scan (187): 高度扫描
```

## 动作空间 (12维)

- **类型**: 关节位置偏移
- **缩放**: 0.25 (action_scale)
- **目标位置**: `default_pos + action * 0.25`
- **力矩限制**: ±33.5 Nm

## PD控制器

```python
kp = 25.0  # 比例增益
kd = 0.5   # 微分增益

# 力矩计算
torque = kp * (target_pos - current_pos) - kd * current_vel
torque = clip(torque, -33.5, 33.5)
```

## 使用方法

### 1. 基础测试

```bash
cd /home/robot/work/LekiwiTest/UntrieeGo2/code
conda activate ros2_env

# 测试策略加载
python test_policy.py
```

### 2. 交互式仿真

```bash
python mujoco_deploy.py
```

**控制命令**:
- `0.5 0 0` - 前进 0.5 m/s
- `0 0.3 0` - 向左平移 0.3 m/s
- `0 0 0.5` - 原地旋转 0.5 rad/s
- `0 0 0` - 停止
- `q` - 退出

## 文件结构

```
code/
├── mujoco_deploy.py       # 主部署脚本
├── test_policy.py          # 策略测试脚本
├── policy_inference.py     # 通用策略推理（旧版，IsaacLab用）
├── policy_config.py        # 配置文件（旧版）
├── demo.py                 # 演示脚本（旧版）
└── MUJOCO_DEPLOY_README.md # 本文档
```

## 关键代码示例

### 策略加载

```python
from mujoco_deploy import MuJoCoPolicy

policy = MuJoCoPolicy(
    "/home/robot/work/LekiwiTest/UntrieeGo2/policy/Rough_Walk_policy4Mujoco.pt",
    device='cpu'
)

# 获取动作
obs = build_observation(...)  # 235维
action = policy.get_action(obs)  # 12维
```

### MuJoCo控制循环

```python
import mujoco

model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

while running:
    # 1. 获取观测
    obs = get_observation(data)

    # 2. 策略推理
    action = policy.get_action(obs)

    # 3. 应用控制
    for i in range(12):
        jid = actuator_to_joint[i]
        current_pos = data.qpos[jid + 6]
        current_vel = data.qvel[jid + 5]

        target_pos = default_pos[i] + action[i]
        torque = kp * (target_pos - current_pos) - kd * current_vel

        data.ctrl[i] = np.clip(torque, -33.5, 33.5)

    # 4. 仿真步进
    mujoco.mj_step(model, data)
```

## 投影重力计算

```python
from scipy.spatial.transform import Rotation as R

# 从MuJoCo获取四元数 (w, x, y, z)
quat = data.qpos[3:7].copy()
# 转换为scipy格式 (x, y, z, w)
quat_xyzw = [quat[1], quat[2], quat[3], quat[0]]
# 创建旋转对象
rotation = R.from_quat(quat_xyzw)
# 将重力向量旋转到基座坐标系
gravity_world = np.array([0.0, 0.0, -1.0])
projected_gravity = rotation.inv().apply(gravity_world)
```

## 调试输出

前100步会打印调试信息：
```
[调试] 步骤20:
  动作(FL): [0.123 -0.456 0.789]
  关节(FL): [0.05 0.92 -1.75]
  基座高度: 0.270
  投影重力: [0.01 0.02 -0.999]
```

## 性能指标

- **仿真频率**: ~60 FPS (time.sleep控制)
- **控制频率**: 每步更新（与仿真同步）
- **物理时间步**: 由MuJoCo模型决定

## 常见问题

### Q1: 机器人倒塌
- 检查默认位置是否正确
- 验证actuator到joint的映射
- 确认PD增益与训练时一致

### Q2: 机器人不动
- 检查速度命令是否为0
- 验证观测是否正确构建
- 查看动作输出是否合理

### Q3: 关节顺序混乱
- 确认使用正确的映射
- 检查MuJoCo XML中的actuator和joint定义

## 参考资源

- **MuJoCo模型**: `/home/robot/work/LekiwiTest/UntrieeGo2/model/go2/scene.xml`
- **策略文件**: `/home/robot/work/LekiwiTest/UntrieeGo2/policy/Rough_Walk_policy4Mujoco.pt`
- **IsaacLab配置**: `/home/robot/work/BiShe/IsaacLabBisShe/source/MyProject/MyProject/tasks/manager_based/WalkTest/`

## 下一步

1. 在MuJoCo中验证策略效果
2. 调整参数优化性能
3. 准备Sim2Real部署到真实机器人

---

**注意**: 此策略专门为MuJoCo模型训练，与IsaacLab原生模型不兼容。请确保使用正确的MuJoCo XML配置。
