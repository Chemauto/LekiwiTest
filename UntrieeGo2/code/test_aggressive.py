#!/usr/bin/env python3
"""
激进测试 - 放大策略输出看是否能运动
"""
import os
os.environ['MUJOCO_GL'] = 'egl'

import numpy as np
import time
import mujoco
import mujoco.viewer
from scipy.spatial.transform import Rotation as R
from mujoco_deploy import MuJoCoDemo

class AggressiveDemo(MuJoCoDemo):
    """激进版本 - 放大策略输出"""

    def step(self):
        """执行一步控制"""
        obs = self.get_observation()

        # 获取动作
        isaaclab_action = self.policy.get_action(obs)
        self.last_action = isaaclab_action.copy()

        # 放大策略输出
        amplified_action = isaaclab_action * 3.0  # 放大3倍
        print(f"[调试] 原始动作: {isaaclab_action[:3]}, 放大后: {amplified_action[:3]}")

        # 映射到MuJoCo actuator顺序
        mujoco_action = np.zeros(12)
        mujoco_action[0:3] = amplified_action[3:6]    # FR
        mujoco_action[3:6] = amplified_action[0:3]    # FL
        mujoco_action[6:9] = amplified_action[9:12]   # RR
        mujoco_action[9:12] = amplified_action[6:9]   # RL

        # 应用控制
        for i in range(12):
            aid = self.actuator_indices[i]
            jid = self.actuator_to_joint[i]

            current_pos = self.data.qpos[jid + 6]
            current_vel = self.data.qvel[jid + 5]

            target_pos = self.default_pos[i] + mujoco_action[i]

            # PD控制
            torque = self.kp * (target_pos - current_pos) - self.kd * current_vel
            torque = np.clip(torque, -33.5, 33.5)

            self.data.ctrl[aid] = torque

        mujoco.mj_step(self.model, self.data)

def main():
    model_path = "/home/robot/work/LekiwiTest/UntrieeGo2/model/go2/scene.xml"
    policy_path = "/home/robot/work/LekiwiTest/UntrieeGo2/policy/Rough_Walk_policy4Mujoco.pt"

    demo = AggressiveDemo(model_path, policy_path)

    print("\n" + "="*60)
    print("激进测试 - 放大策略输出3倍")
    print("="*60)

    demo.reset_to_initial_state()

    # 设置速度命令
    demo.vx = 0.5
    demo.vy = 0.0
    demo.wz = 0.0

    print(f"速度命令: vx={demo.vx}, vy={demo.vy}, wz={demo.wz}")
    print(f"策略输出放大: 3.0x")
    print("\n开始测试...")

    with mujoco.viewer.launch_passive(demo.model, demo.data) as viewer:
        step_count = 0

        try:
            while viewer.is_running() and step_count < 300:  # 运行5秒
                demo.step()

                step_count += 1

                if step_count % 60 == 0:
                    base_pos = demo.data.qpos[0:3]
                    base_vel = demo.data.qvel[0:3]
                    print(f"步骤:{step_count} | 位置:[{base_pos[0]:.2f}, {base_pos[1]:.2f}, {base_pos[2]:.2f}] | "
                          f"速度:[{base_vel[0]:.2f}, {base_vel[1]:.2f}]")

                viewer.sync()
                time.sleep(1.0/60.0)

        except KeyboardInterrupt:
            print("\n\n用户中断")

    print(f"\n测试完成!")
    print(f"总步数: {step_count}")
    print(f"最终位置: {demo.data.qpos[0:3]}")
    print(f"最终速度: {demo.data.qvel[0:3]}")

if __name__ == "__main__":
    main()
