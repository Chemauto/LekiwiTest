#!/usr/bin/env python3
"""
简单测试脚本 - 验证策略加载和推理
"""
import sys
sys.path.insert(0, '/home/robot/work/LekiwiTest/UntrieeGo2/code')

import numpy as np
from mujoco_deploy import MuJoCoPolicy

print("=" * 60)
print("测试策略加载")
print("=" * 60)

# 加载策略
policy_path = "/home/robot/work/LekiwiTest/UntrieeGo2/policy/Rough_Walk_policy4Mujoco.pt"
policy = MuJoCoPolicy(policy_path, device='cpu')

# 创建测试观测 (235维)
print("\n创建测试观测...")
obs = np.random.randn(235).astype(np.float32)

# 获取动作
print("执行策略推理...")
action = policy.get_action(obs)

print(f"\n结果:")
print(f"  输入维度: {obs.shape}")
print(f"  输出维度: {action.shape}")
print(f"  动作范围: [{action.min():.3f}, {action.max():.3f}]")
print(f"  动作均值: {action.mean():.3f}")
print(f"  动作标准差: {action.std():.3f}")

print("\n✓ 策略测试通过!")
