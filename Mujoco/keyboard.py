"""
MuJoCo 小车仿真的键盘控制界面
提供 UI 输入控制来驾驶小车 (类似 ROS 小海龟 teleop)
"""

import time
import threading
import sys
import select
import termios
import tty
import mujoco
import mujoco.viewer
from control import CarController


class KeyboardController:
    def __init__(self, model_path):
        """
        初始化键盘控制器

        参数:
            model_path: MuJoCo XML 模型文件的路径
        """
        # 加载 MuJoCo 模型
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)

        # 创建小车控制器
        self.controller = CarController(self.model, self.data)

        # 控制状态 (使用类似 ROS teleop 的直接控制方式)
        self.forward_speed = 0.0  # 前进速度 (-1 到 1)
        self.turn_speed = 0.0     # 转向速度 (-1 到 1)

        # 控制参数 (类似 ROS teleop)
        self.forward_step = 0.1    # 每次按键的增量
        self.turn_step = 0.1       # 每次按键的增量
        self.max_speed = 1.0       # 最大速度

        # 按键状态
        self.current_key = None
        self.running = True

        # 终端输入设置
        self.settings = None

    def get_key(self):
        """从终端获取单个按键 (非阻塞)"""
        if select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.read(1)
        return None

    def handle_key(self, key):
        """
        处理单个按键 (类似 ROS teleop)

        控制键:
        - w / i: 增加前进速度
        - s / k: 减少前进速度 (后退)
        - a / j: 左转
        - d / l: 右转
        - q: 退出
        - 空格: 停止
        """
        if key is None:
            return

        key = key.lower()

        if key == 'w' or key == 'i':
            # 增加前进速度
            self.forward_speed = min(self.forward_speed + self.forward_step, self.max_speed)
        elif key == 's' or key == 'k':
            # 减少前进速度
            self.forward_speed = max(self.forward_speed - self.forward_step, -self.max_speed)
        elif key == 'a' or key == 'j':
            # 左转 (turn > 0 时左转，根据 car.xml 的定义)
            self.turn_speed = min(self.turn_speed + self.turn_step, 1.0)
        elif key == 'd' or key == 'l':
            # 右转 (turn < 0 时右转，根据 car.xml 的定义)
            self.turn_speed = max(self.turn_speed - self.turn_step, -1.0)
        elif key == ' ':
            # 立即停止
            self.forward_speed = 0.0
            self.turn_speed = 0.0
        elif key == 'q':
            # 退出仿真
            self.running = False

    def print_controls(self):
        """打印控制说明 (类似 ROS teleop)"""
        print("\n" + "="*60)
        print("小车控制系统 (ROS 风格 teleop)")
        print("="*60)
        print("\n控制命令:")
        print("  w / i    : 增加前进速度")
        print("  s / k    : 减少前进速度 (后退)")
        print("  a / j    : 左转")
        print("  d / l    : 右转")
        print("  空格     : 立即停止")
        print("  q        : 退出仿真")
        print("\n" + "="*60)
        print("按任意控制键开始驾驶小车...")
        print("="*60 + "\n")

    def print_status(self):
        """打印当前状态 (ROS teleop 风格)"""
        # 清除行并打印状态
        status_line = (f"\r运动状态: 前进={self.forward_speed:5.2f} "
                      f"转向={self.turn_speed:5.2f} | "
                      f"位置: [{self.controller.get_car_position()[0]:5.2f}, "
                      f"{self.controller.get_car_position()[1]:5.2f}]   ")
        print(status_line, end="", flush=True)

    def run(self, duration=None):
        """
        运行带键盘控制的仿真

        参数:
            duration: 运行时长（秒），None 表示无限运行
        """
        self.print_controls()

        # 保存终端设置并设置为非阻塞模式
        self.settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())

            with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
                start_time = time.time()
                last_print_time = 0

                while viewer.is_running() and self.running:
                    # 检查运行时长
                    if duration and (time.time() - start_time > duration):
                        break

                    step_start = time.time()

                    # 检查键盘输入
                    key = self.get_key()
                    if key:
                        self.handle_key(key)

                    # 应用控制信号
                    self.controller.set_control(self.forward_speed, self.turn_speed)
                    self.controller.apply_control()

                    # 物理步进
                    mujoco.mj_step(self.model, self.data)

                    # 定期打印状态
                    if time.time() - last_print_time > 0.1:  # 每 0.1 秒打印一次
                        self.print_status()
                        last_print_time = time.time()

                    # 同步查看器
                    viewer.sync()

                    # 时间控制
                    time_until_next_step = self.model.opt.timestep - (time.time() - step_start)
                    if time_until_next_step > 0:
                        time.sleep(time_until_next_step)

        finally:
            # 恢复终端设置
            if self.settings:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            print("\n\n仿真结束。")


def main():
    """主入口函数"""
    model_path = '../model/car.xml'

    try:
        controller = KeyboardController(model_path)
        controller.run()
    except KeyboardInterrupt:
        print("\n\n仿真被用户停止")
    except Exception as e:
        print(f"\n错误: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
