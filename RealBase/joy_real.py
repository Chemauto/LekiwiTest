#!/usr/bin/env python3
"""
使用真实Xbox手柄控制3全向轮底盘
使用pygame库读取真实手柄输入
"""

import sys
import os
import time
import argparse

try:
    import pygame
    from pygame import joystick
except ImportError:
    print("错误: 未安装pygame库")
    print("请运行: pip install pygame")
    sys.exit(1)

from motor_controller import OmniWheelController

# --- 配置参数 ---
LINEAR_SPEED = 0.3  # 最大线速度 (m/s)
OMEGA_SPEED = 0.8  # 最大角速度 (rad/s)
JOYSTICK_DEADZONE = 0.1  # 手柄死区


class XboxJoystickReader:
    """读取真实Xbox手柄状态"""

    def __init__(self, joystick_id=0):
        """初始化手柄

        Args:
            joystick_id: 手柄ID，默认为0（第一个连接的手柄）
        """
        # 初始化pygame
        pygame.init()
        pygame.joystick.init()

        # 检查是否有手柄连接
        if pygame.joystick.get_count() == 0:
            raise RuntimeError("未检测到任何手柄设备！请确保手柄已连接并重新插拔。")

        # 打开手柄
        self.js = pygame.joystick.Joystick(joystick_id)
        self.js.init()

        print(f"已连接手柄: {self.js.get_name()}")
        print(f"轴数量: {self.js.get_numaxes()}")
        print(f"按键数量: {self.js.get_numbuttons()}")
        print(f"方向键数量: {self.js.get_numhats()}")

        # 校准摇杆零点偏移
        self.axis_offset = []
        self._calibrate()

    def _calibrate(self):
        """校准摇杆零点，记录初始偏移"""
        print("\n正在校准手柄，请松开所有摇杆和按键...")
        time.sleep(0.5)  # 等待手柄稳定

        # 清空事件队列
        for _ in pygame.event.get():
            pass

        # 读取所有轴的初始值作为偏移
        num_axes = self.js.get_numaxes()
        self.axis_offset = []
        for i in range(num_axes):
            offset = self.js.get_axis(i)
            self.axis_offset.append(offset)

        print("校准完成!")
        print("轴偏移量:", [f"{x:.3f}" for x in self.axis_offset])

    def get_axis(self, axis_id: int) -> float:
        """获取轴值（已减去零点偏移）

        Args:
            axis_id: 轴ID (0-左摇杆X, 1-左摇杆Y, 2-右摇杆X, 3-右摇杆Y)

        Returns:
            轴值，范围 [-1.0, 1.0]
        """
        if axis_id < self.js.get_numaxes():
            # 轴值需要处理pygame事件队列才能更新
            for _ in pygame.event.get():
                pass
            raw_value = self.js.get_axis(axis_id)
            # 减去校准时的零点偏移
            offset = self.axis_offset[axis_id] if axis_id < len(self.axis_offset) else 0.0
            corrected_value = raw_value - offset
            # 限制在[-1, 1]范围内
            return max(-1.0, min(1.0, corrected_value))
        return 0.0

    def get_button(self, button_id: int) -> bool:
        """获取按键状态

        Args:
            button_id: 按键ID

        Returns:
            True表示按下，False表示未按下
        """
        if button_id < self.js.get_numbuttons():
            # 按键状态需要处理pygame事件队列才能更新
            for _ in pygame.event.get():
                pass
            return self.js.get_button(button_id)
        return False

    def get_hat(self, hat_id: int = 0) -> tuple:
        """获取方向键状态

        Args:
            hat_id: 方向键ID

        Returns:
            (x, y) 元组，x和y取值为 -1, 0, 1
        """
        if hat_id < self.js.get_numhats():
            # 方向键状态需要处理pygame事件队列才能更新
            for _ in pygame.event.get():
                pass
            return self.js.get_hat(hat_id)
        return (0, 0)

    def is_connected(self) -> bool:
        """检查手柄是否仍然连接"""
        try:
            pygame.joystick.init()
            return pygame.joystick.get_count() > 0
        except Exception:
            return False


def main(port, debug=False):
    """主函数"""
    print("=" * 60)
    print("Xbox手柄控制程序")
    print("=" * 60)
    print(f"将使用串口: {port}")

    # 创建Xbox手柄读取器
    try:
        joystick_reader = XboxJoystickReader()
    except RuntimeError as e:
        print(f"\n错误: {e}")
        print("\n请确保:")
        print("  - Xbox手柄已通过USB或无线接收器连接到电脑")
        print("  - 手柄驱动已正确安装")
        print("  - 已安装pygame库: pip install pygame")
        print("\n连接手柄后，请重新运行程序")
        return

    # 如果是诊断模式，只显示轴值
    if debug:
        print("\n" + "=" * 60)
        print("诊断模式 - 显示所有轴的实时值")
        print("=" * 60)
        print("请操作各个摇杆和按键，观察数值变化")
        print("按 Ctrl+C 退出")
        print("=" * 60)

        try:
            while True:
                for _ in pygame.event.get():
                    pass

                # 显示所有轴的值
                num_axes = joystick_reader.js.get_numaxes()
                axis_values = []
                for i in range(num_axes):
                    value = joystick_reader.get_axis(i)
                    axis_values.append(f"{value:+.3f}")

                # 显示所有按键状态
                num_buttons = joystick_reader.js.get_numbuttons()
                button_values = []
                for i in range(num_buttons):
                    pressed = joystick_reader.get_button(i)
                    button_values.append(f"B{i}:{int(pressed)}")

                print(f"\r轴: [{' | '.join(axis_values)}]  按键: {' '.join(button_values)}    ", end="")
                time.sleep(0.05)

        except KeyboardInterrupt:
            print("\n\n退出诊断模式")
            pygame.quit()
            return

    print("\n控制映射:")
    print("  - 左摇杆 (上下): 前进/后退 (vx)")
    print("  - 左摇杆 (左右): 左移/右移 (vy)")
    print("  - 右摇杆 (左右): 原地旋转 (omega)")
    print("  - Xbox键(LS): 切换速度档位 (低/中/高)")
    print("\n速度档位:")
    print("  - 低速模式: 0.15 m/s, 0.4 rad/s")
    print("  - 中速模式: 0.3 m/s, 0.8 rad/s (默认)")
    print("  - 高速模式: 0.5 m/s, 1.2 rad/s")
    print("\n按 Ctrl+C 退出程序")
    print("=" * 60)

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

    # 速度档位设置
    speed_modes = [
        {"linear": 0.15, "omega": 0.4, "name": "低速"},
        {"linear": 0.30, "omega": 0.8, "name": "中速"},
        {"linear": 0.50, "omega": 1.2, "name": "高速"}
    ]
    current_mode = 1  # 默认中速模式

    try:
        last_warn_time = 0
        last_button_state = False  # 用于检测按钮按下事件

        while True:
            # 处理pygame事件
            for event in pygame.event.get():
                if event.type == pygame.JOYDEVICEREMOVED:
                    print("\n警告: 手柄已断开连接！")
                    joystick_reader = None

            # 检查手柄连接状态
            if joystick_reader is None or not joystick_reader.is_connected():
                current_time = time.time()
                if current_time - last_warn_time > 2.0:  # 每2秒警告一次
                    print("\n警告: 手柄连接丢失，停止机器人...")
                    print("请重新连接手柄后重启程序")
                    last_warn_time = current_time
                controller.stop()
                time.sleep(0.1)
                continue

            # 读取摇杆轴值
            # 左摇杆 Y 轴 (axis 1): 前进/后退, 上为-1, 下为1, 所以取反
            vx_raw = -joystick_reader.get_axis(1)
            # 左摇杆 X 轴 (axis 0): 左移/右移
            vy_raw = joystick_reader.get_axis(0)
            # 右摇杆 X 轴 (axis 3): 旋转（注意：Xbox手柄右摇杆X轴通常是axis 3）
            omega_raw = joystick_reader.get_axis(3)

            # 应用死区
            vx = vx_raw if abs(vx_raw) > JOYSTICK_DEADZONE else 0.0
            vy = vy_raw if abs(vy_raw) > JOYSTICK_DEADZONE else 0.0
            omega = omega_raw if abs(omega_raw) > JOYSTICK_DEADZONE else 0.0

            # 检测Xbox左摇杆按键(LS/按钮8)用于切换速度档位
            ls_button = joystick_reader.get_button(8)  # Xbox左摇杆按下
            if ls_button and not last_button_state:
                current_mode = (current_mode + 1) % len(speed_modes)
                mode = speed_modes[current_mode]
                print(f"\n切换到{mode['name']}模式: {mode['linear']} m/s, {mode['omega']} rad/s")
            last_button_state = ls_button

            # 获取当前速度档位
            mode = speed_modes[current_mode]

            # 如果所有输入都在死区内，则停止
            if vx == 0.0 and vy == 0.0 and omega == 0.0:
                controller.stop()
                print(f"\r[{mode['name']}] 停止中                                      ", end="")
            else:
                # 将 [-1, 1] 的摇杆输入映射到实际速度
                final_vx = vx * mode['linear']
                final_vy = vy * mode['linear']
                final_omega = omega * mode['omega']

                # 发送速度指令
                controller.set_velocity_raw(vx=final_vx, vy=final_vy, omega=final_omega)

                # 打印状态
                print(f"\r[{mode['name']}] Joystick -> vx: {vx:+.2f}, vy: {vy:+.2f}, omega: {omega:+.2f} | "
                      f"Sending -> vx: {final_vx:+.2f}m/s, vy: {final_vy:+.2f}m/s, omega: {final_omega:+.2f}rad/s",
                      end="")

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
        pygame.quit()
        print("程序结束")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="使用真实Xbox手柄控制3全向轮底盘")
    parser.add_argument(
        '--port',
        type=str,
        default='/dev/ttyACM0',
        help='电机控制器连接的串口端口'
    )
    parser.add_argument(
        '--debug',
        action='store_true',
        help='诊断模式：显示所有轴和按键的实时值（不连接底盘）'
    )
    args = parser.parse_args()
    main(args.port, args.debug)
