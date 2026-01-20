#!/usr/bin/env python3
"""
使用虚拟手柄控制3全向轮底盘
支持两种模式：
1. pygame 读取真实手柄或虚拟手柄设备（需要虚拟手柄创建 uinput 设备）
2. 读取虚拟手柄状态文件（与 Project4_SimJoy GUI 配合使用）
"""

import sys
import os
import time
import argparse
import pickle

from motor_controller import OmniWheelController

# --- 配置参数 ---
LINEAR_SPEED = 0.3  # 最大线速度 (m/s)
OMEGA_SPEED = 0.8  # 最大角速度 (rad/s)
JOYSTICK_DEADZONE = 0.1  # 手柄死区

# 虚拟手柄状态文件路径
STATE_FILE = "/tmp/virtual_joystick_state.pkl"


class VirtualJoystickReader:
    """从状态文件读取虚拟手柄状态"""

    def __init__(self, state_file=STATE_FILE):
        self.state_file = state_file
        self.last_state = None

    def get_axis(self, axis_id: int) -> float:
        """获取轴值"""
        state = self._read_state()
        if state and 'axes' in state:
            return state['axes'].get(axis_id, 0.0)
        return 0.0

    def get_button(self, button_id: int) -> bool:
        """获取按键状态"""
        state = self._read_state()
        if state and 'buttons' in state:
            return state['buttons'].get(button_id, False)
        return False

    def get_hat(self, hat_id: int = 0) -> tuple:
        """获取方向键状态"""
        state = self._read_state()
        if state and 'hats' in state:
            return state['hats'].get(hat_id, (0, 0))
        return (0, 0)

    def _read_state(self):
        """读取状态文件"""
        try:
            if os.path.exists(self.state_file):
                # 检查文件修改时间，避免读取过旧的文件
                mtime = os.path.getmtime(self.state_file)
                current_time = time.time()
                # 如果文件超过1秒未更新，认为虚拟手柄未运行
                if current_time - mtime > 1.0:
                    return None

                with open(self.state_file, 'rb') as f:
                    state = pickle.load(f)
                    self.last_state = state
                    return state
        except Exception as e:
            pass
        return None

    def is_running(self) -> bool:
        """检查虚拟手柄是否正在运行"""
        try:
            if os.path.exists(self.state_file):
                mtime = os.path.getmtime(self.state_file)
                current_time = time.time()
                return (current_time - mtime) < 1.0
        except Exception:
            pass
        return False

def main(port):
    """主函数"""
    print("="*60)
    print("手柄控制程序")
    print("="*60)
    print(f"将使用串口: {port}")

    # 创建虚拟手柄读取器
    joystick = VirtualJoystickReader()

    # 检查虚拟手柄是否正在运行
    print("\n正在检查虚拟手柄状态...")
    if not joystick.is_running():
        print("\n未检测到虚拟手柄！")
        print("请确保虚拟手柄模拟器正在运行:")
        print("  cd /home/dora/RoboOs/LekiwiTest/Project4_SimJoy && ./run.sh")
        return

    print("虚拟手柄已连接 (通过状态文件)")
    print("\n控制映射:")
    print("  - 左摇杆 (上下): 前进/后退 (vx)")
    print("  - 左摇杆 (左右): 左移/右移 (vy)")
    print("  - 右摇杆 (左右): 原地旋转 (omega)")
    print("\n按 Ctrl+C 退出程序")
    print("="*60)

    # 初始化底盘控制器
    controller = OmniWheelController(port=port)
    print("\n正在连接底盘...")
    if not controller.connect():
        print("\n连接失败,请检查:")
        print(f"  - 串口端口 '{port}' 是否正确")
        print("  - 电机是否已上电并连接")
        print(f"  - 电机ID是否正确({controller.WHEEL_IDS})")
        return
    print("底盘连接成功!")

    try:
        last_warn_time = 0
        while True:
            # 检查虚拟手柄是否仍在运行
            if not joystick.is_running():
                current_time = time.time()
                if current_time - last_warn_time > 2.0:  # 每2秒警告一次
                    print("\n警告: 虚拟手柄连接丢失，停止机器人...")
                    last_warn_time = current_time
                controller.stop()
                time.sleep(0.1)
                continue

            # 读取摇杆轴值
            # 左摇杆 Y 轴 (前进/后退), 默认上为-1, 下为1, 所以取反
            vx_raw = -joystick.get_axis(1)
            # 左摇杆 X 轴 (左移/右移)
            vy_raw = joystick.get_axis(0)
            # 右摇杆 X 轴 (旋转)
            omega_raw = -joystick.get_axis(3)

            # 应用死区
            vx = vx_raw if abs(vx_raw) > JOYSTICK_DEADZONE else 0.0
            vy = vy_raw if abs(vy_raw) > JOYSTICK_DEADZONE else 0.0
            omega = omega_raw if abs(omega_raw) > JOYSTICK_DEADZONE else 0.0

            # 如果所有输入都在死区内，则停止
            if vx == 0.0 and vy == 0.0 and omega == 0.0:
                controller.stop()
                print(f"\r停止中                                      ", end="")
            else:
                # 将 [-1, 1] 的摇杆输入映射到实际速度
                final_vx = vx * LINEAR_SPEED
                final_vy = vy * LINEAR_SPEED
                final_omega = omega * OMEGA_SPEED

                # 发送速度指令
                controller.set_velocity_raw(vx=final_vx, vy=final_vy, omega=final_omega)

                # 打印状态
                print(f"\rJoystick -> vx: {vx:+.2f}, vy: {vy:+.2f}, omega: {omega:+.2f} | "
                      f"Sending -> vx: {final_vx:+.2f}m/s, vy: {final_vy:+.2f}m/s, omega: {final_omega:+.2f}rad/s", end="")

            # 循环延时
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n\n程序被中断, 正在停止机器人...")

    except Exception as e:
        print(f"\n发生错误: {e}")
        import traceback
        traceback.print_exc()

    finally:
        # 断开连接
        print("\n正在断开连接...")
        controller.stop()
        controller.disconnect()
        print("程序结束")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="使用虚拟手柄控制3全向轮底盘")
    parser.add_argument(
        '--port',
        type=str,
        default='/dev/ttyACM0',
        help='电机控制器连接的串口端口'
    )
    args = parser.parse_args()
    main(args.port)
