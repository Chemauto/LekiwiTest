#!/usr/bin/env python3
"""
MuJoCo部署脚本 - 专门为MuJoCo训练的策略
基于MuJoCo XML配置:
- 关节顺序: FL, FR, RL, RR
- Actuator顺序: FR, FL, RR, RL
- 默认位置: [0, 0.9, -1.8]

交互模式:
- 机器人持续执行当前命令
- 随时输入新命令改变速度
- Ctrl+C 或输入 'q' 退出
- 后台运行时自动进入演示模式
"""
import os
os.environ['MUJOCO_GL'] = 'egl'

import numpy as np
import time
import mujoco
import mujoco.viewer
from scipy.spatial.transform import Rotation as R
import torch
import threading
import sys

# 检测是否在交互模式
def is_interactive():
    """检测是否在交互终端中运行"""
    return sys.stdin.isatty()


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
        """获取动作"""
        obs_tensor = torch.from_numpy(obs).float()

        with torch.no_grad():
            action_tensor = self.forward(obs_tensor)

        action = action_tensor.cpu().numpy()

        # IsaacLab训练时使用了action_scale=0.25
        action = action * 0.25

        return action


class MuJoCoDemo:
    """MuJoCo交互式演示"""

    def __init__(self, model_path: str, policy_path: str, decimation: int = 1):
        """
        Args:
            model_path: MuJoCo XML路径
            policy_path: 策略文件路径
            decimation: 控制频率降低倍数（默认1=每步都控制）
                       IsaacLab常用值: 2, 4
        """
        print("\n" + "=" * 60)
        print("MuJoCo策略部署")
        print("=" * 60)

        self.decimation = decimation
        self.step_count = 0

        # 加载模型
        print(f"\n[1] 加载MuJoCo模型...")
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)

        # 显示时间步长信息
        print(f"    时间步长: {self.model.opt.timestep}")
        print(f"    物理频率: {1/self.model.opt.timestep:.1f} Hz")
        print(f"    Decimation: {self.decimation}")
        print(f"    控制频率: {1/self.model.opt.timestep/self.decimation:.1f} Hz")

        # 加载策略
        print(f"[2] 加载策略...")
        self.policy = MuJoCoPolicy(policy_path, device='cpu')

        # 速度命令
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0

        # 速度命令缩放（IsaacLab训练时可能使用了缩放）
        self.vel_command_scale = 1.0

        # MuJoCo配置（从XML）
        # 关节定义顺序 (Joint 1-12): FL_hip, FL_thigh, FL_calf, FR_hip, FR_thigh, FR_calf, RL_hip, RL_thigh, RL_calf, RR_hip, RR_thigh, RR_calf
        # Actuator顺序: FR_hip, FR_thigh, FR_calf, FL_hip, FL_thigh, FL_calf, RR_hip, RR_thigh, RR_calf, RL_hip, RL_thigh, RL_calf

        self.joint_indices = list(range(1, 13))  # Joint 1-12
        self.actuator_indices = list(range(12))

        # IsaacLab默认位置（FL, FR, RL, RR顺序）
        # FL: [0.1, 0.8, -1.5]
        # FR: [-0.1, 0.8, -1.5]
        # RL: [0.1, 1.0, -1.5]
        # RR: [-0.1, 1.0, -1.5]
        self.default_pos = np.array([
            0.1, 0.8, -1.5,   # FL
            -0.1, 0.8, -1.5,  # FR
            0.1, 1.0, -1.5,   # RL
            -0.1, 1.0, -1.5,  # RR
        ])

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
        # 恢复到原始值
        self.kp = 25.0
        self.kd = 0.5

        self.last_action = np.zeros(12)

        print(f"[3] 配置完成")
        print(f"    关节数: {len(self.joint_indices)}")
        print(f"    Actuator数: {len(self.actuator_indices)}")
        print(f"    PD增益: kp={self.kp}, kd={self.kd}")

        self.running = True

    def reset_to_initial_state(self):
        """重置到初始状态"""
        # 设置基座位置（IsaacLab: 0 0 0.4）
        self.data.qpos[0:3] = [0, 0, 0.4]
        self.data.qpos[3:7] = [1, 0, 0, 0]  # 四元数

        # 设置关节位置（IsaacLab默认位置，FL,FR,RL,RR顺序）
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
        print(f"  期望位置(FL): {self.default_pos[:3]}")

    def get_observation(self):
        """构建观测（235维）"""
        # 按joint定义顺序构建（FL, FR, RL, RR）
        # 因为IsaacLab训练时用的是这个顺序（preserve_order=True）
        joint_pos = np.array([self.data.qpos[jid + 6] for jid in self.joint_indices])
        joint_vel = np.array([self.data.qvel[jid + 5] for jid in self.joint_indices])

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

        # 速度命令（应用缩放）
        velocity_command = np.array([self.vx, self.vy, self.wz]) * self.vel_command_scale

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
        self.step_count += 1

        # 根据decimation决定是否更新动作
        if self.step_count % self.decimation == 0:
            # 获取观测
            obs = self.get_observation()

            # 获取动作（FL, FR, RL, RR顺序，IsaacLab训练顺序）
            isaaclab_action = self.policy.get_action(obs)
            self.last_action = isaaclab_action.copy()

            # 将IsaacLab顺序(FL,FR,RL,RR)映射到MuJoCo actuator顺序(FR,FL,RR,RL)
            mujoco_action = np.zeros(12)
            mujoco_action[0:3] = isaaclab_action[3:6]    # FR
            mujoco_action[3:6] = isaaclab_action[0:3]    # FL
            mujoco_action[6:9] = isaaclab_action[9:12]   # RR
            mujoco_action[9:12] = isaaclab_action[6:9]   # RL

            # 调试输出（前100步）
            if self.step_count <= 100 and self.step_count % 20 == 0:
                fl_pos = self.data.qpos[8:11]  # FL joints 1-3 -> qpos 8-10
                fr_pos = self.data.qpos[11:14] # FR joints 4-6 -> qpos 11-13

                quat = self.data.qpos[3:7].copy()
                quat_xyzw = np.array([quat[1], quat[2], quat[3], quat[0]])
                rotation = R.from_quat(quat_xyzw)
                proj_grav = rotation.inv().apply(np.array([0.0, 0.0, -1.0]))

                print(f"[调试] 步骤{self.step_count}:")
                print(f"  IsaacLab动作(FL): {isaaclab_action[:3]}")
                print(f"  IsaacLab动作(FR): {isaaclab_action[3:6]}")
                print(f"  关节(FL): {fl_pos}")
                print(f"  关节(FR): {fr_pos}")
                print(f"  基座高度: {self.data.qpos[2]:.3f}")
                print(f"  投影重力: {proj_grav}")
        else:
            # 使用上一个动作
            isaaclab_action = self.last_action
            mujoco_action = np.zeros(12)
            mujoco_action[0:3] = isaaclab_action[3:6]
            mujoco_action[3:6] = isaaclab_action[0:3]
            mujoco_action[6:9] = isaaclab_action[9:12]
            mujoco_action[9:12] = isaaclab_action[6:9]

        # 应用控制到actuators（FR, FL, RR, RL顺序）
        for i in range(12):
            aid = self.actuator_indices[i]
            jid = self.actuator_to_joint[i]

            # 当前状态
            current_pos = self.data.qpos[jid + 6]
            current_vel = self.data.qvel[jid + 5]

            # 目标位置 = 默认 + MuJoCo动作
            target_pos = self.default_pos[i] + mujoco_action[i]

            # PD控制
            torque = self.kp * (target_pos - current_pos) - self.kd * current_vel
            torque = np.clip(torque, -33.5, 33.5)

            self.data.ctrl[aid] = torque

        # 仿真步进
        mujoco.mj_step(self.model, self.data)

    def _input_loop(self):
        """输入线程 - 在后台等待用户输入"""
        while self.running:
            try:
                cmd = input("> ").strip()
                if cmd.lower() == 'q':
                    self.running = False
                    break
                elif cmd.lower() == 's':
                    # 停止命令
                    self.vx = 0.0
                    self.vy = 0.0
                    self.wz = 0.0
                    print(f"✓ 已停止: vx=0.00, vy=0.00, wz=0.00")
                else:
                    parts = cmd.split()
                    if len(parts) == 3:
                        try:
                            self.vx = float(parts[0])
                            self.vy = float(parts[1])
                            self.wz = float(parts[2])
                            print(f"✓ 命令已更新: vx={self.vx:.2f}, vy={self.vy:.2f}, wz={self.wz:.2f}")
                        except ValueError:
                            print("✗ 格式错误，请输入三个数字")
                    else:
                        print("✗ 格式错误，请输入: vx vy wz，或输入 's' 停止，'q' 退出")
            except (EOFError, KeyboardInterrupt):
                self.running = False
                break

    def run(self):
        """运行仿真 - 机器人持续执行当前命令"""

        # 检测是否交互模式
        interactive = is_interactive()

        if interactive:
            self.print_help()
            print("\n模式: 交互控制")
        else:
            print("\n模式: 自动演示 (检测到非交互环境)")
            print("演示序列: 前进 -> 停止 -> 旋转 -> 停止 -> 循环")

        # 重置
        print("\n重置到初始状态...")
        self.reset_to_initial_state()
        time.sleep(0.5)

        # 主控制循环
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            step_count = 0
            last_print_time = time.time()

            # 演示模式: 定义命令序列
            demo_sequence = [
                (0.0, 0.0, 0.0, 3.0),  # 站立 3秒
                (0.3, 0.0, 0.0, 5.0),  # 前进 5秒
                (0.0, 0.0, 0.0, 2.0),  # 停止 2秒
                (0.0, 0.0, 0.5, 4.0),  # 旋转 4秒
                (0.0, 0.0, 0.0, 2.0),  # 停止 2秒
            ]
            demo_idx = 0
            demo_start_time = time.time()

            # 交互模式: 先获取第一个命令
            if interactive:
                print("\n开始仿真...")
                print("\n" + "=" * 60)
                print(f"当前命令: vx={self.vx:.2f}, vy={self.vy:.2f}, wz={self.wz:.2f}")
                print("=" * 60)
                print("\n使用说明:")
                print("  0.5 0 0  - 前进 0.5 m/s (机器人持续执行)")
                print("  s        - 停止 (保持站立)")
                print("  q        - 退出")
                print("=" * 60)
                print("\n注意: 建议速度范围 0.3-0.8 m/s")
                print("\n请输入命令:")

                try:
                    cmd = input("> ").strip()
                    if cmd.lower() == 'q':
                        print("退出仿真...")
                        return
                    elif cmd.lower() == 's':
                        self.vx = 0.0
                        self.vy = 0.0
                        self.wz = 0.0
                        print(f"✓ 已停止: vx=0.00, vy=0.00, wz=0.00")
                    else:
                        parts = cmd.split()
                        if len(parts) == 3:
                            self.vx = float(parts[0])
                            self.vy = float(parts[1])
                            self.wz = float(parts[2])
                            print(f"✓ 命令已更新: vx={self.vx:.2f}, vy={self.vy:.2f}, wz={self.wz:.2f}")
                        else:
                            print("格式错误，使用默认命令 (0 0 0)")
                except (EOFError, KeyboardInterrupt, ValueError):
                    print("输入错误，使用默认命令 (0 0 0)")

                print("\n机器人正在执行命令... (随时输入新命令改变运动)")
                print("提示: 在下方输入新命令，按 Enter 立即生效\n")

            # 启动输入线程（仅在交互模式）
            if interactive:
                self.running = True
                input_thread = threading.Thread(target=self._input_loop, daemon=True)
                input_thread.start()

            try:
                while viewer.is_running() and self.running:
                    # 演示模式: 自动切换命令
                    if not interactive:
                        vx, vy, wz, duration = demo_sequence[demo_idx]

                        # 检查是否需要切换
                        elapsed = time.time() - demo_start_time
                        if elapsed >= duration:
                            demo_idx = (demo_idx + 1) % len(demo_sequence)
                            demo_start_time = time.time()
                            vx, vy, wz, duration = demo_sequence[demo_idx]
                            print(f"\n[自动演示] 切换命令: vx={vx:.1f}, vy={vy:.1f}, wz={wz:.1f}")

                        self.vx, self.vy, self.wz = vx, vy, wz

                    # 执行控制
                    self.step()

                    step_count += 1

                    # 每秒打印一次状态
                    current_time = time.time()
                    if current_time - last_print_time >= 1.0:
                        base_pos = self.data.qpos[0:3]
                        base_vel = self.data.qvel[0:3]

                        mode_str = "交互" if interactive else "演示"
                        print(f"\r[{mode_str}] 步骤:{step_count} | 命令:[{self.vx:.2f}, {self.vy:.2f}, {self.wz:.2f}] | "
                              f"位置:[{base_pos[0]:.2f}, {base_pos[1]:.2f}, {base_pos[2]:.2f}] | "
                              f"速度:[{base_vel[0]:.2f}, {base_vel[1]:.2f}]", end="", flush=True)
                        last_print_time = current_time

                    viewer.sync()
                    time.sleep(1.0/60.0)

            except KeyboardInterrupt:
                print("\n\n用户中断")
            finally:
                self.running = False

        print("\n仿真结束")

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
        print("    s            - 停止 (保持站立)")
        print("    q            - 退出")
        print("  注意: 建议速度范围 0.3-0.8 m/s")
        print("=" * 60)


def main():
    model_path = "/home/robot/work/LekiwiTest/UntrieeGo2/model/go2/scene.xml"
    policy_path = "/home/robot/work/LekiwiTest/UntrieeGo2/policy/Rough_Walk_policy4Mujoco.pt"

    # 使用decimation=2来匹配IsaacLab的控制频率
    # 如果MuJoCo timestep=0.002 (500Hz)，decimation=2 → 250Hz控制
    # IsaacLab通常使用60-120Hz控制频率
    demo = MuJoCoDemo(model_path, policy_path, decimation=4)
    demo.run()


if __name__ == "__main__":
    main()
