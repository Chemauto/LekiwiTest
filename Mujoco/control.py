"""
MuJoCo 仿真小车控制器
控制小车的前进和转向电机
"""

import mujoco
import numpy as np


class CarController:
    def __init__(self, model, data):
        """
        初始化小车控制器

        参数:
            model: MuJoCo 模型实例
            data: MuJoCo 数据实例
        """
        self.model = model
        self.data = data

        # 获取执行器ID
        self.forward_actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'forward')
        self.turn_actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'turn')

        # 控制信号 (-1 到 1)
        self.forward_control = 0.0  # 前进控制
        self.turn_control = 0.0      # 转向控制

    def set_control(self, forward, turn):
        """
        设置小车的控制信号

        参数:
            forward: 前进电机控制值 (-1 到 1)
                    -1 = 全速后退, 0 = 停止, 1 = 全速前进
            turn: 转向电机控制值 (-1 到 1)
                  -1 = 全速左转, 0 = 直行, 1 = 全速右转
        """
        # 将值限制在有效范围内
        self.forward_control = np.clip(forward, -1.0, 1.0)
        self.turn_control = np.clip(turn, -1.0, 1.0)

    def get_forward_control(self):
        """获取当前前进控制值"""
        return self.forward_control

    def get_turn_control(self):
        """获取当前转向控制值"""
        return self.turn_control

    def stop(self):
        """停止小车"""
        self.forward_control = 0.0
        self.turn_control = 0.0

    def apply_control(self):
        """
        将控制信号应用到 MuJoCo 执行器

        这个方法应该在每次 mj_step 之前调用
        """
        self.data.ctrl[self.forward_actuator_id] = self.forward_control
        self.data.ctrl[self.turn_actuator_id] = self.turn_control

    def get_car_position(self):
        """
        获取小车当前位置

        返回:
            numpy数组，包含 [x, y, z] 位置坐标
        """
        car_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, 'car')
        return self.data.xpos[car_body_id].copy()

    def get_car_orientation(self):
        """
        获取小车当前姿态

        返回:
            numpy数组，包含 [w, x, y, z] 四元数
        """
        car_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, 'car')
        return self.data.xquat[car_body_id].copy()
