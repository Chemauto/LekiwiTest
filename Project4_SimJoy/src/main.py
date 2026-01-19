"""
Xbox手柄模拟器主程序入口
启动虚拟手柄GUI界面
"""
import sys
import os

# 添加父目录到路径，以便导入模块
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtCore import Qt
from src.joystick_ui import JoystickUI, STATE_FILE


class MainWindow(QMainWindow):
    """主窗口类"""

    def __init__(self):
        super().__init__()
        self.init_ui()

    def init_ui(self):
        """初始化UI"""
        # 创建手柄UI
        self.joystick_ui = JoystickUI(self)
        self.setCentralWidget(self.joystick_ui)

        # 设置窗口属性
        self.setWindowTitle("Virtual Xbox Controller Simulator")
        self.resize(1000, 700)

        # 设置窗口样式
        self.setStyleSheet("""
            QMainWindow {
                background-color: #1e1e1e;
            }
        """)

        # 居中显示
        self.center()

    def center(self):
        """将窗口居中显示"""
        screen = self.joystick_ui.screen()
        if screen:
            screen_geometry = screen.availableGeometry()
            window_geometry = self.frameGeometry()
            center_point = screen_geometry.center()
            window_geometry.moveCenter(center_point)
            self.move(window_geometry.topLeft())

    def get_joystick(self):
        """获取虚拟手柄对象"""
        return self.joystick_ui.get_joystick()

    def closeEvent(self, event):
        """窗口关闭事件"""
        # 清理状态文件
        try:
            if os.path.exists(STATE_FILE):
                os.remove(STATE_FILE)
                print(f"已清理状态文件: {STATE_FILE}")
        except Exception as e:
            pass
        # 接受关闭事件
        event.accept()


def main():
    """主函数"""
    # 启用高DPI支持
    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)

    # 创建应用
    app = QApplication(sys.argv)
    app.setApplicationName("Virtual Xbox Controller")
    app.setOrganizationName("SimJoy")

    # 创建并显示主窗口
    window = MainWindow()
    window.show()

    # 运行应用
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
