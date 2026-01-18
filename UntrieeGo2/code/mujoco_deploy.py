#!/usr/bin/env python3
"""
MuJoCo部署脚本 - 专门为MuJoCo训练的策略
基于MuJoCo XML配置:
- 关节顺序: FL, FR, RL, RR
- Actuator顺序: FR, FL, RR, RL
- 默认位置: [0, 0.9, -1.8]
"""
import os
os.environ['MUJOCO_GL'] = 'egl'

import numpy as np
import time
import threading
import mujoco
import mujoco.viewer
from scipy.spatial.transform import Rotation as R
import torch


class MuJoCoPolicy:
    """MuJoCo专用策略加载器"""

    def __init__(self, policy_path: str, device='cpu'):
        print("=" * 60)
        print("加载MuJoCo训练策略")
        print("=" * 60)

        self.device = device

        # 加载checkpoint
        print(f"\n[1] 加载策略文件: {policy_path}")
        checkpoint = torch.load(policy_path, map_location=self.device)

        # 提取actor权重
        self.layers = []
        state_dict = checkpoint['model_state_dict']

        # 查找所有actor层（按层号排序）
        actor_keys = [k for k in state_dict.keys() if k.startswith('actor') and 'weight' in k]
        actor_keys.sort()

        for weight_key in actor_keys:
            bias_key = weight_key.replace('weight', 'bias')

            weight = state_dict[weight_key].to(self.device)
            bias = state_dict[bias_key].to(self.device)

            in_dim = weight.shape[1]
            out_dim = weight.shape[0]

            self.layers.append({
                'weight': weight,
                'bias': bias,
                'in_dim': in_dim,
                'out_dim': out_dim,
                'activation': 'elu'  # 所有隐藏层用ELU
            })

        # 最后一层（输出层）不需要激活函数
        if self.layers:
            self.layers[-1]['activation'] = None

        self.obs_dim = self.layers[0]['in_dim']
        self.action_dim = self.layers[-1]['out_dim']

        print(f"[2] 策略结构: {len(self.layers)}层")
        print(f"    输入维度: {self.obs_dim}")
        print(f"    输出维度: {self.action_dim}")
        print(f"    设备: {self.device}")
        print(f"    训练迭代: {checkpoint['iter']}")

    def forward(self, obs: torch.Tensor) -> torch.Tensor:
        """前向传播"""
        if obs.dim() == 1:
            obs = obs.unsqueeze(0)

        x = obs.to(self.device)

        for layer in self.layers:
            x = torch.mm(x, layer['weight'].t()) + layer['bias']
            if layer['activation'] == 'elu':
                x = torch.nn.functional.elu(x)

        return x.squeeze(0) if x.shape[0] == 1 else x

    def get_action(self, obs: np.ndarray) -> np.ndarray:
        """获取动作（已经是缩放后的关节位置偏移）"""
        obs_tensor = torch.from_numpy(obs).float()

        with torch.no_grad():
            action_tensor = self.forward(obs_tensor)

        action = action_tensor.cpu().numpy()

        # IsaacLab的action_scale = 0.25
        action = action * 0.25

        return action


class MuJoCoDemo:
    """MuJoCo交互式演示"""

    def __init__(self, model_path: str, policy_path: str):
        print("\n" + "=" * 60)
        print("MuJoCo策略部署")
        print("=" * 60)

        # 加载模型
        print(f"\n[1] 加载MuJoCo模型...")
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)

        # 加载策略
        print(f"[2] 加载策略...")
        self.policy = MuJoCoPolicy(policy_path, device='cpu')

        # 速度命令
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0

        # MuJoCo配置（从XML）
        # 关节定义顺序 (Joint 1-12): FL_hip, FL_thigh, FL_calf, FR_hip, FR_thigh, FR_calf, RL_hip, RL_thigh, RL_calf, RR_hip, RR_thigh, RR_calf
        # Actuator顺序: FR_hip, FR_thigh, FR_calf, FL_hip, FL_thigh, FL_calf, RR_hip, RR_thigh, RR_calf, RL_hip, RL_thigh, RL_calf
        # 默认位置: [0, 0.9, -1.8] (hip, thigh, calf)

        self.joint_indices = list(range(1, 13))  # Joint 1-12
        self.actuator_indices = list(range(12))

        # 默认关节位置（所有腿相同）
        self.default_pos = np.array([0.0, 0.9, -1.8] * 4)

        # Actuator到Joint的映射 (actuator -> joint index)
        # Actuator: FR(0-2), FL(3-5), RR(6-8), RL(9-11)
        # Joint: FL(0-2), FR(3-5), RL(6-8), RR(9-12)
        # 所以:
        #   FR actuators 0-2 -> joints 4-6 (offset by +1 for freejoint)
        #   FL actuators 3-5 -> joints 1-3
        #   RR actuators 6-8 -> joints 10-12
        #   RL actuators 9-11 -> joints 7-9
        self.actuator_to_joint = np.array([
            4, 5, 6,   # FR actuators -> FR joints (4,5,6)
            1, 2, 3,   # FL actuators -> FL joints (1,2,3)
            10, 11, 12, # RR actuators -> RR joints (10,11,12)
            7, 8, 9    # RL actuators -> RL joints (7,8,9)
        ])

        # PD增益（从IsaacLab DCMotorCfg）
        self.kp = 25.0
        self.kd = 0.5

        self.last_action = np.zeros(12)

        print(f"[3] 配置完成")
        print(f"    关节数: {len(self.joint_indices)}")
        print(f"    Actuator数: {len(self.actuator_indices)}")

        self.running = True

    def reset_to_initial_state(self):
        """重置到初始状态"""
        # 设置基座位置（从keyframe: 0 0 0.27）
        self.data.qpos[0:3] = [0, 0, 0.27]
        self.data.qpos[3:7] = [1, 0, 0, 0]  # 四元数

        # 设置关节位置（从keyframe: 0 0.9 -1.8 重复4次）
        self.data.qpos[7:19] = self.default_pos

        # 设置速度为0
        self.data.qvel[:] = 0

        # 设置控制为0
        self.data.ctrl[:] = 0

        # 前向运动学
        mujoco.mj_forward(self.model, self.data)

        print(f"\n[重置] 初始状态:")
        print(f"  基座高度: {self.data.qpos[2]:.3f}")
        print(f"  关节位置(FL): {self.data.qpos[8:11]}")

    def get_observation(self):
        """构建观测（235维）"""
        # 读取关节状态（FL, FR, RL, RR顺序）
        joint_pos = np.array([self.data.qpos[j + 6] for j in self.joint_indices])
        joint_vel = np.array([self.data.qvel[j + 5] for j in self.joint_indices])

        # 相对于默认位置
        joint_pos_rel = joint_pos - self.default_pos
        joint_vel_rel = joint_vel

        # 基座状态
        base_lin_vel = self.data.qvel[0:3].copy()
        base_ang_vel = self.data.qvel[3:6].copy()

        # 投影重力
        quat = self.data.qpos[3:7].copy()  # (w,x,y,z)
        quat_xyzw = np.array([quat[1], quat[2], quat[3], quat[0]])
        rotation = R.from_quat(quat_xyzw)
        gravity_world = np.array([0.0, 0.0, -1.0])
        projected_gravity = rotation.inv().apply(gravity_world)

        # 速度命令
        velocity_command = np.array([self.vx, self.vy, self.wz])

        # 高度扫描
        height_scan = np.zeros(187)

        # 构建完整观测 (235维)
        obs = np.concatenate([
            base_lin_vel,           # 3
            base_ang_vel,           # 3
            projected_gravity,      # 3
            velocity_command,       # 3
            joint_pos_rel,          # 12
            joint_vel_rel,          # 12
            self.last_action,       # 12
            height_scan,            # 187
        ])

        return obs

    def step(self):
        """执行一步控制"""
        # 获取观测
        obs = self.get_observation()

        # 获取动作（FL, FR, RL, RR顺序）
        action = self.policy.get_action(obs)
        self.last_action = action.copy()

        # 调试输出（前100步）
        if not hasattr(self, '_step_count'):
            self._step_count = 0
        self._step_count += 1

        if self._step_count <= 100 and self._step_count % 20 == 0:
            joint_pos = np.array([self.data.qpos[j + 6] for j in self.joint_indices])
            quat = self.data.qpos[3:7].copy()
            quat_xyzw = np.array([quat[1], quat[2], quat[3], quat[0]])
            rotation = R.from_quat(quat_xyzw)
            proj_grav = rotation.inv().apply(np.array([0.0, 0.0, -1.0]))

            print(f"[调试] 步骤{self._step_count}:")
            print(f"  动作(FL): {action[:3]}")
            print(f"  关节(FL): {joint_pos[:3]}")
            print(f"  基座高度: {self.data.qpos[2]:.3f}")
            print(f"  投影重力: {proj_grav}")

        # 应用控制到actuators
        for i in range(12):
            aid = self.actuator_indices[i]
            jid = self.actuator_to_joint[i]

            # 当前状态
            current_pos = self.data.qpos[jid + 6]
            current_vel = self.data.qvel[jid + 5]

            # 目标位置 = 默认 + 动作偏移
            target_pos = self.default_pos[i] + action[i]

            # PD控制
            torque = self.kp * (target_pos - current_pos) - self.kd * current_vel

            # 限制力矩（IsaacLab effort_limit = 33.5）
            torque = np.clip(torque, -33.5, 33.5)

            self.data.ctrl[aid] = torque

        # 仿真步进
        mujoco.mj_step(self.model, self.data)

    def run(self):
        """运行仿真"""
        self.print_help()

        # 输入线程
        input_thread = threading.Thread(target=self.input_loop, daemon=True)
        input_thread.start()

        # 主循环
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            step_count = 0
            last_time = time.time()

            # 重置
            print("\n重置到初始状态...")
            self.reset_to_initial_state()
            time.sleep(0.5)

            print("\n开始仿真...")

            while viewer.is_running() and self.running:
                self.step()

                step_count += 1
                if step_count % 100 == 0:
                    base_pos = self.data.qpos[0:3]
                    base_vel = self.data.qvel[0:3]
                    elapsed = time.time() - last_time
                    fps = 100 / (elapsed + 1e-6)

                    print(f"[状态] 步骤:{step_count} | "
                          f"位置:[{base_pos[0]:.2f}, {base_pos[1]:.2f}, {base_pos[2]:.2f}] | "
                          f"速度:[{base_vel[0]:.2f}, {base_vel[1]:.2f}] | "
                          f"命令:[{self.vx:.2f}, {self.vy:.2f}, {self.wz:.2f}] | "
                          f"FPS:{fps:.1f}")

                    last_time = time.time()

                viewer.sync()
                time.sleep(1.0/60.0)

        print("\n仿真结束")

    def input_loop(self):
        """输入循环"""
        while self.running:
            try:
                cmd = input("\n请输入速度命令 (vx vy wz) 或 'q' 退出: ").strip()

                if cmd.lower() == 'q':
                    self.running = False
                    break

                parts = cmd.split()
                if len(parts) == 3:
                    self.vx = float(parts[0])
                    self.vy = float(parts[1])
                    self.wz = float(parts[2])
                    print(f"✓ 速度命令已更新: vx={self.vx:.2f}, vy={self.vy:.2f}, wz={self.wz:.2f}")
                else:
                    print("✗ 格式错误")
            except:
                break

    def print_help(self):
        """打印帮助"""
        print("\n" + "=" * 60)
        print("控制说明:")
        print("=" * 60)
        print("  命令格式: <vx> <vy> <wz>")
        print("  示例:")
        print("    0.5 0.0 0.0   - 前进 0.5 m/s")
        print("    0.0 0.3 0.0   - 向左平移 0.3 m/s")
        print("    0.0 0.0 0.5   - 原地旋转 0.5 rad/s")
        print("    0.0 0.0 0.0   - 停止")
        print("  输入 'q' 退出")
        print("=" * 60)


def main():
    model_path = "/home/robot/work/LekiwiTest/UntrieeGo2/model/go2/scene.xml"
    policy_path = "/home/robot/work/LekiwiTest/UntrieeGo2/policy/Rough_Walk_policy4Mujoco.pt"

    demo = MuJoCoDemo(model_path, policy_path)
    demo.run()


if __name__ == "__main__":
    main()
