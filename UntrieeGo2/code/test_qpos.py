#!/usr/bin/env python3
"""
测试MuJoCo qpos的实际数据
"""
import mujoco
import numpy as np

model_path = "/home/robot/work/LekiwiTest/UntrieeGo2/model/go2/scene.xml"
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

print("=" * 60)
print("MuJoCo qpos 测试")
print("=" * 60)

print(f"\nqpos总维度: {data.qpos.shape}")
print(f"  qpos[0:3] (base位置): 预期 [0, 0, 0.27]")
print(f"  qpos[3:7] (base四元数): 预期 [1, 0, 0, 0]")
print(f"  qpos[7:19] (12个关节): 预期 [0, 0.9, -1.8, 0, 0.9, -1.8, ...]")

# 设置keyframe值
data.qpos[:] = 0
data.qpos[0:3] = [0, 0, 0.27]
data.qpos[3:7] = [1, 0, 0, 0]
# 从keyframe: "0 0 0.27 1 0 0 0 0 0.9 -1.8 0 0.9 -1.8 0 0.9 -1.8 0 0.9 -1.8"
data.qpos[7:19] = [0, 0.9, -1.8, 0, 0.9, -1.8, 0, 0.9, -1.8, 0, 0.9, -1.8]

print(f"\n设置后的qpos[7:19]:")
print(f"  完整: {data.qpos[7:19]}")

print(f"\n按腿分组 (Joint定义顺序: FL, FR, RL, RR):")
print(f"  FL (Joint 1-3): qpos[8:11] = {data.qpos[8:11]}")  # 1+7=8, 3+7=11
print(f"  FR (Joint 4-6): qpos[11:14] = {data.qpos[11:14]}") # 4+7=11, 6+7=14
print(f"  RL (Joint 7-9): qpos[14:17] = {data.qpos[14:17]}") # 7+7=14, 9+7=17
print(f"  RR (Joint 10-12): qpos[17:20] = {data.qpos[17:20]}") # 10+7=17, 12+7=20

print(f"\nActuator顺序 (FR, FL, RR, RL):")
print(f"  FR (Actuator 0-2) 控制 Joint 4-6")
print(f"  FL (Actuator 3-5) 控制 Joint 1-3")
print(f"  RR (Actuator 6-8) 控制 Joint 10-12")
print(f"  RL (Actuator 9-11) 控制 Joint 7-9")

print(f"\n所以，actuator对应的qpos索引:")
actuator_to_qpos_idx = [
    11, 12, 13,  # FR: Joint 4,5,6 -> qpos 11,12,13
    8, 9, 10,    # FL: Joint 1,2,3 -> qpos 8,9,10
    17, 18, 19,  # RR: Joint 10,11,12 -> qpos 17,18,19
    14, 15, 16   # RL: Joint 7,8,9 -> qpos 14,15,16
]
print(f"  FR (0-2): qpos[{actuator_to_qpos_idx[0]}:{actuator_to_qpos_idx[2]+1}]")
print(f"  FL (3-5): qpos[{actuator_to_qpos_idx[3]}:{actuator_to_qpos_idx[5]+1}]")
print(f"  RR (6-8): qpos[{actuator_to_qpos_idx[6]}:{actuator_to_qpos_idx[8]+1}]")
print(f"  RL (9-11): qpos[{actuator_to_qpos_idx[9]}:{actuator_to_qpos_idx[11]+1}]")

print(f"\n验证actuator控制的关节:")
for i in range(12):
    qpos_idx = actuator_to_qpos_idx[i]
    leg_name = ['FR', 'FR', 'FR', 'FL', 'FL', 'FL', 'RR', 'RR', 'RR', 'RL', 'RL', 'RL'][i]
    joint_name = ['hip', 'thigh', 'calf'][i % 3]
    print(f"  Actuator {i} ({leg_name}_{joint_name}) -> qpos[{qpos_idx}] = {data.qpos[qpos_idx]:.2f}")
