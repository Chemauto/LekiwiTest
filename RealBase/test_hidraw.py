#!/usr/bin/env python3
import sys

print("测试读取hidraw设备...\n")

devices = [
    ('/dev/hidraw0', 'Interface 0 - Vendor Specific (可能包含摇杆)'),
    ('/dev/hidraw1', 'Interface 2 - HID')
]

for dev_path, desc in devices:
    print(f"=== {dev_path} - {desc} ===")
    try:
        with open(dev_path, 'rb') as f:
            print("打开成功！请摇动摇杆或按键，观察数据变化...")
            print("前10个数据包:\n")
            for i in range(10):
                data = f.read(32)
                hex_str = ' '.join(f'{b:02x}' for b in data)
                print(f"  {hex_str}")
    except PermissionError:
        print("权限不足，需要sudo")
    except Exception as e:
        print(f"错误: {e}")
    print()
