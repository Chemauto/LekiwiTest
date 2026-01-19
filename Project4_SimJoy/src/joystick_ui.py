"""
Xbox手柄模拟器主界面
组合所有UI组件，提供完整的手柄外观和交互
"""
import pickle
import os
import time
import atexit
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout,
                             QLabel, QFrame, QSplitter)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QPainter, QColor, QPen, QBrush, QLinearGradient, QRadialGradient
from .virtual_joystick import VirtualJoystick
from .components.button import JoystickButton, ShoulderButton, SmallButton
from .components.stick import AnalogStick
from .components.trigger import Trigger
from .components.dpad import DPad

# 状态文件路径
STATE_FILE = "/tmp/virtual_joystick_state.pkl"


def cleanup_state_file():
    """退出时清理状态文件"""
    try:
        if os.path.exists(STATE_FILE):
            os.remove(STATE_FILE)
            print(f"已清理状态文件: {STATE_FILE}")
    except Exception as e:
        pass


# 注册退出清理函数
atexit.register(cleanup_state_file)


class JoystickUI(QWidget):
    """
    Xbox手柄模拟器主界面
    提供完整的手柄外观和控制功能
    """

    # 信号：状态改变时发射
    state_updated = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)

        # 创建虚拟手柄对象
        self.joystick = VirtualJoystick(0)

        # 初始化UI
        self.init_ui()

        # 设置定时器用于状态显示更新
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self._update_status_display)
        self.update_timer.start(100)  # 每100ms更新一次状态显示

        # 设置定时器定期写入状态文件（即使没有用户操作）
        self.state_file_timer = QTimer()
        self.state_file_timer.timeout.connect(self._write_state_file)
        self.state_file_timer.start(100)  # 每100ms写入一次状态文件

    def init_ui(self):
        """初始化UI"""
        # 设置窗口属性
        self.setWindowTitle("Virtual Xbox Controller")
        self.setMinimumSize(900, 600)

        # 主布局
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(20, 20, 20, 20)
        main_layout.setSpacing(20)

        # 创建手柄绘制区域
        self.joystick_widget = JoystickWidget(self.joystick, self)
        main_layout.addWidget(self.joystick_widget, stretch=3)

        # 创建状态显示区域
        self.status_label = QLabel("Controller Status")
        self.status_label.setStyleSheet("""
            QLabel {
                background-color: #2b2b2b;
                color: #00ff00;
                font-family: 'Courier New', monospace;
                font-size: 12px;
                padding: 10px;
                border: 2px solid #444;
                border-radius: 5px;
            }
        """)
        self.status_label.setWordWrap(True)
        main_layout.addWidget(self.status_label, stretch=1)

        # 连接信号
        self.joystick_widget.state_updated.connect(self._on_state_updated)

    def _on_state_updated(self):
        """状态更新时调用"""
        self.state_updated.emit()
        self._update_status_display()
        self._write_state_file()

    def _write_state_file(self):
        """将手柄状态写入文件，供其他进程读取"""
        try:
            state = self.joystick.get_state()
            # 提取需要的数据
            state_data = {
                'buttons': state.buttons.copy(),
                'axes': state.axes.copy(),
                'hats': state.hats.copy(),
                'timestamp': time.time()
            }
            with open(STATE_FILE, 'wb') as f:
                pickle.dump(state_data, f)
        except Exception as e:
            pass  # 静默失败，避免频繁的错误打印

    def _update_status_display(self):
        """更新状态显示"""
        state = self.joystick.get_state()
        self.status_label.setText(str(state))

    def get_joystick(self) -> VirtualJoystick:
        """获取虚拟手柄对象"""
        return self.joystick


class JoystickWidget(QWidget):
    """
    手柄绘制组件
    负责绘制手柄外观和布局所有控制组件
    """

    state_updated = pyqtSignal()

    def __init__(self, joystick: VirtualJoystick, parent=None):
        super().__init__(parent)
        self.joystick = joystick

        # 启用鼠标追踪
        self.setMouseTracking(True)

        # 初始化所有组件
        self.init_components()

    def init_components(self):
        """初始化所有手柄组件"""

        # ========== 上方区域：肩键和扳机 ==========
        # 左肩键 LB
        self.btn_lb = ShoulderButton(4, "LB", 80, 30, self)
        self.btn_lb.move(120, 10)
        self.btn_lb.state_changed.connect(self._on_button_changed)

        # 右肩键 RB
        self.btn_rb = ShoulderButton(5, "RB", 80, 30, self)
        self.btn_rb.move(620, 10)
        self.btn_rb.state_changed.connect(self._on_button_changed)

        # 左扳机 LT
        self.trigger_lt = Trigger(2, "LT", 100, 50, self)
        self.trigger_lt.move(110, 45)
        self.trigger_lt.axis_changed.connect(self._on_axis_changed)

        # 右扳机 RT
        self.trigger_rt = Trigger(5, "RT", 100, 50, self)
        self.trigger_rt.move(610, 45)
        self.trigger_rt.axis_changed.connect(self._on_axis_changed)

        # ========== 中间区域：ABXY按键和方向键 ==========
        # 左侧ABXY按键
        btn_radius = 28

        # Y键（上，黄色）
        self.btn_y = JoystickButton(3, "Y", QColor(255, 215, 0), btn_radius, self)
        self.btn_y.move(320, 100)
        self.btn_y.state_changed.connect(self._on_button_changed)

        # X键（左，蓝色）
        self.btn_x = JoystickButton(2, "X", QColor(0, 100, 255), btn_radius, self)
        self.btn_x.move(270, 150)
        self.btn_x.state_changed.connect(self._on_button_changed)

        # B键（右，红色）
        self.btn_b = JoystickButton(1, "B", QColor(220, 20, 60), btn_radius, self)
        self.btn_b.move(370, 150)
        self.btn_b.state_changed.connect(self._on_button_changed)

        # A键（下，绿色）
        self.btn_a = JoystickButton(0, "A", QColor(0, 200, 0), btn_radius, self)
        self.btn_a.move(320, 200)
        self.btn_a.state_changed.connect(self._on_button_changed)

        # 右侧方向键
        self.dpad = DPad(0, 120, self)
        self.dpad.move(500, 120)
        self.dpad.hat_changed.connect(self._on_hat_changed)

        # ========== 下方区域：摇杆 ==========
        stick_radius = 50

        # 左摇杆
        self.stick_left = AnalogStick(0, 1, stick_radius, self)
        self.stick_left.move(150, 280)
        self.stick_left.axis_changed.connect(self._on_axis_changed)

        # 右摇杆
        self.stick_right = AnalogStick(3, 4, stick_radius, self)
        self.stick_right.move(570, 280)
        self.stick_right.axis_changed.connect(self._on_axis_changed)

        # ========== 中间功能键 ==========
        # SELECT键
        self.btn_select = SmallButton(6, "SEL", 20, self)
        self.btn_select.move(380, 350)
        self.btn_select.state_changed.connect(self._on_button_changed)

        # START键
        self.btn_start = SmallButton(7, "STA", 20, self)
        self.btn_start.move(440, 350)
        self.btn_start.state_changed.connect(self._on_button_changed)

    def _on_button_changed(self, button_id: int, state: bool):
        """按钮状态改变回调"""
        self.joystick.set_button(button_id, state)
        self.state_updated.emit()

    def _on_axis_changed(self, axis_id: int, value: float):
        """轴值改变回调"""
        self.joystick.set_axis(axis_id, value)
        self.state_updated.emit()

    def _on_hat_changed(self, hat_id: int, x: int, y: int):
        """方向键状态改变回调"""
        self.joystick.set_hat(hat_id, (x, y))
        self.state_updated.emit()

    def paintEvent(self, event):
        """绘制手柄外观"""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        # 绘制手柄主体
        self._draw_controller_body(painter)

    def _draw_controller_body(self, painter: QPainter):
        """
        绘制手柄主体
        使用渐变色和圆角，模拟真实手柄外观
        """
        # 手柄主体参数
        body_width = 800
        body_height = 450
        body_x = (self.width() - body_width) // 2
        body_y = 70

        # 绘制阴影
        shadow_offset = 8
        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor(0, 0, 0, 100))
        painter.drawRoundedRect(
            body_x + shadow_offset,
            body_y + shadow_offset,
            body_width,
            body_height,
            40, 40
        )

        # 创建主体渐变
        gradient = QLinearGradient(body_x, body_y, body_x, body_y + body_height)
        gradient.setColorAt(0, QColor(80, 80, 80))
        gradient.setColorAt(0.5, QColor(60, 60, 60))
        gradient.setColorAt(1, QColor(50, 50, 50))

        # 绘制主体
        painter.setPen(QPen(QColor(30, 30, 30), 3))
        painter.setBrush(gradient)
        painter.drawRoundedRect(
            body_x, body_y, body_width, body_height,
            40, 40
        )

        # 绘制内圈边框（装饰）
        inner_margin = 15
        painter.setPen(QPen(QColor(45, 45, 45), 2))
        painter.setBrush(Qt.NoBrush)
        painter.drawRoundedRect(
            body_x + inner_margin,
            body_y + inner_margin,
            body_width - inner_margin * 2,
            body_height - inner_margin * 2,
            30, 30
        )

        # 绘制中间装饰条
        decoration_y = body_y + body_height // 2
        painter.setPen(QPen(QColor(40, 40, 40), 1))
        painter.drawLine(
            body_x + 50, decoration_y,
            body_x + body_width - 50, decoration_y
        )

        # 绘制左右握手纹理
        self._draw_grip_texture(painter, body_x + 30, body_y + body_height - 150, 60, 130)
        self._draw_grip_texture(painter, body_x + body_width - 90, body_y + body_height - 150, 60, 130)

    def _draw_grip_texture(self, painter: QPainter, x: int, y: int, width: int, height: int):
        """
        绘制握手纹理

        Args:
            painter: QPainter对象
            x, y, width, height: 纹理区域
        """
        # 绘制点状纹理
        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor(35, 35, 35, 150))

        dot_radius = 2
        spacing = 10

        for dy in range(0, height, spacing):
            for dx in range(0, width, spacing):
                # 添加一些随机偏移
                offset_x = (dy % 2) * (spacing // 2)
                painter.drawEllipse(
                    x + dx + offset_x,
                    y + dy,
                    dot_radius * 2,
                    dot_radius * 2
                )
