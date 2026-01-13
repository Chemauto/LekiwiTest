"""
演示脚本 - 展示如何使用小车控制器
这个演示展示了无需键盘输入的自动控制模式
"""

import time
import mujoco
import mujoco.viewer
from control import CarController


def run_circle_demo(model_path, duration=30):
    """
    运行圆形轨迹演示 - 小车将沿圆形路径行驶

    参数:
        model_path: 小车 XML 模型路径
        duration: 运行时长（秒）
    """
    # 加载模型
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)

    # 创建控制器
    controller = CarController(model, data)

    print("\n" + "="*50)
    print("圆形演示 - 小车将沿圆形路径行驶")
    print("="*50 + "\n")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        start_time = time.time()

        while viewer.is_running() and (time.time() - start_time) < duration:
            step_start = time.time()

            # 圆形运动控制逻辑: 前进 + 持续转向
            controller.set_control(forward=0.5, turn=0.3)
            controller.apply_control()

            # 物理步进
            mujoco.mj_step(model, data)

            # 打印状态
            if int(data.time * 100) % 50 == 0:
                pos = controller.get_car_position()
                print(f"\r时间: {data.time:.2f}秒 | 位置: [{pos[0]:.2f}, {pos[1]:.2f}]", end="")

            # 同步查看器
            viewer.sync()

            # 时间控制
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)


def run_figure_eight_demo(model_path, duration=40):
    """
    运行 8 字形轨迹演示 - 小车将沿 8 字形路径行驶

    参数:
        model_path: 小车 XML 模型路径
        duration: 运行时长（秒）
    """
    # 加载模型
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)

    # 创建控制器
    controller = CarController(model, data)

    print("\n" + "="*50)
    print("8 字形演示 - 小车将沿 8 字形路径行驶")
    print("="*50 + "\n")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        start_time = time.time()

        while viewer.is_running() and (time.time() - start_time) < duration:
            step_start = time.time()

            # 8 字形运动控制逻辑: 变化的转向方向
            # 使用正弦波来交替转向
            turn = 0.5 * (time.time() - start_time)
            controller.set_control(forward=0.6, turn=turn)
            controller.apply_control()

            # 物理步进
            mujoco.mj_step(model, data)

            # 打印状态
            if int(data.time * 100) % 50 == 0:
                pos = controller.get_car_position()
                print(f"\r时间: {data.time:.2f}秒 | 位置: [{pos[0]:.2f}, {pos[1]:.2f}]", end="")

            # 同步查看器
            viewer.sync()

            # 时间控制
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)


def run_square_demo(model_path, duration=30):
    """
    运行方形轨迹演示 - 小车将沿方形路径行驶

    参数:
        model_path: 小车 XML 模型路径
        duration: 运行时长（秒）
    """
    # 加载模型
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)

    # 创建控制器
    controller = CarController(model, data)

    print("\n" + "="*50)
    print("方形演示 - 小车将沿方形路径行驶")
    print("="*50 + "\n")

    # 方形运动的状态机
    # 状态: 0=前进, 1=左转, 2=前进, 3=左转, 等等
    state = 0
    state_timer = 0
    forward_duration = 3.0  # 前进持续时间（秒）
    turn_duration = 1.0     # 转向持续时间（秒）

    with mujoco.viewer.launch_passive(model, data) as viewer:
        start_time = time.time()

        while viewer.is_running() and (time.time() - start_time) < duration:
            step_start = time.time()

            # 状态机逻辑
            if state == 0:  # 前进状态
                controller.set_control(forward=0.7, turn=0.0)
                state_timer += model.opt.timestep
                if state_timer >= forward_duration:
                    state = 1
                    state_timer = 0
            elif state == 1:  # 转向状态
                controller.set_control(forward=0.3, turn=-0.8)
                state_timer += model.opt.timestep
                if state_timer >= turn_duration:
                    state = 0
                    state_timer = 0

            controller.apply_control()

            # 物理步进
            mujoco.mj_step(model, data)

            # 打印状态
            if int(data.time * 100) % 50 == 0:
                pos = controller.get_car_position()
                print(f"\r时间: {data.time:.2f}秒 | 状态: {state} | 位置: [{pos[0]:.2f}, {pos[1]:.2f}]", end="")

            # 同步查看器
            viewer.sync()

            # 时间控制
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)


def main():
    """主入口函数 - 运行不同的演示"""
    model_path = '../model/car.xml'

    print("\n" + "="*50)
    print("小车控制器演示")
    print("="*50)
    print("\n选择演示:")
    print("  1 - 圆形 (小车沿圆形路径行驶)")
    print("  2 - 8 字形 (小车沿 8 字形路径行驶)")
    print("  3 - 方形 (小车沿方形路径行驶)")

    try:
        choice = input("\n请输入选择 (1-3): ").strip()

        if choice == '1':
            run_circle_demo(model_path)
        elif choice == '2':
            run_figure_eight_demo(model_path)
        elif choice == '3':
            run_square_demo(model_path)
        else:
            print("无效选择，运行圆形演示...")
            run_circle_demo(model_path)

    except KeyboardInterrupt:
        print("\n\n演示被用户停止")
    except Exception as e:
        print(f"\n错误: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
