#!/usr/bin/env python3
import sys
import select
import os

print("测试所有hidraw设备...")
print("请按手柄上的按键或摇动摇杆！\n")

devices = ['/dev/hidraw0', '/dev/hidraw1']

for dev in devices:
    print(f"\n{'='*60}")
    print(f"测试 {dev}")
    print('='*60)

    try:
        fd = os.open(dev, os.O_RDWR | os.O_NONBLOCK)
        print(f"✓ 打开成功")

        print("等待数据... (按Ctrl+C跳过)")
        print("请按手柄按键或摇动摇杆！")

        data_count = 0
        last_data = None

        import time
        start_time = time.time()

        while data_count < 5 and time.time() - start_time < 10:
            # 检查是否有数据可读
            r, _, _ = select.select([fd], [], [], 0.1)
            if r:
                try:
                    data = os.read(fd, 64)
                    if data and data != last_data:
                        data_count += 1
                        hex_str = ' '.join(f'{b:02x}' for b in data[:32])
                        dec_str = ' '.join(f'{b:3d}' for b in data[:32])
                        print(f"\n数据包 #{data_count}:")
                        print(f"  HEX: {hex_str}")
                        print(f"  DEC: {dec_str}")
                        last_data = data
                except BlockingIOError:
                    pass

        os.close(fd)

        if data_count == 0:
            print("✗ 未读取到数据（可能手柄休眠或此接口无数据）")
        else:
            print(f"\n✓ 成功读取到 {data_count} 个数据包")

    except FileNotFoundError:
        print(f"✗ 设备不存在")
    except PermissionError:
        print(f"✗ 权限不足")
    except Exception as e:
        print(f"✗ 错误: {e}")

print(f"\n{'='*60}")
print("诊断完成")
print("如果所有设备都无数据，可能需要:")
print("1. 按手柄Home键唤醒")
print("2. 重新插拔USB接收器")
print("3. 或更换其他品牌的手柄（原装Xbox手柄支持最好）")
