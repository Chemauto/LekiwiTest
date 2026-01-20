#!/usr/bin/env python3
import time

print("Xbox手柄原始数据读取")
print("="*80)
print("请操作手柄并观察数据变化")
print("- 左摇杆左右: 应该看到某字节变化")
print("- 左摇杆上下: 应该看到某字节变化")
print("- 右摇杆左右: 应该看到某字节变化")
print("- 按键A/B/X/Y: 应该看到某字节变化")
print("="*80)
print("\n正在读取... (按Ctrl+C退出)\n")

with open('/dev/hidraw0', 'rb') as f:
    last_data = None
    packet_count = 0

    try:
        while True:
            data = f.read(32)

            # 只在有数据变化时显示
            if last_data is None or data != last_data:
                print(f"\n包 #{packet_count}:")
                print("字节:  0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19")
                print("值:  ", end="")

                for i in range(min(20, len(data))):
                    # 高亮变化的字节
                    if last_data is not None and data[i] != last_data[i]:
                        print(f'\033[1;32m{data[i]:02x}\033[0m ', end='')  # 绿色
                    else:
                        print(f'{data[i]:02x} ', end='')
                print()

                # 十进制显示（更容易理解摇杆值）
                print("十进制:", end=" ")
                for i in range(min(20, len(data))):
                    if last_data is not None and data[i] != last_data[i]:
                        print(f'\033[1;32m{data[i]:3d}\033[0m ', end='')
                    else:
                        print(f'{data[i]:3d} ', end='')
                print()

                last_data = data
                packet_count += 1

            time.sleep(0.02)  # 50Hz采样率

    except KeyboardInterrupt:
        print(f"\n\n总共读取了 {packet_count} 个数据包")
