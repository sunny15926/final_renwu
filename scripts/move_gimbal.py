
#!/usr/bin/env python3
"""让云台动一下的最小测试脚本（不依赖 ROS / pyserial，纯标准库）。

按电控协议直接向串口发送 AimbotFrame（0x55 头 / 0xFF 尾，ID=0x02）：
    uint8  _SOF                 = 0x55
    uint8  ID                   = 0x02
    float  PitchRelativeAngle   (rad，发送时取负)
    float  YawRelativeAngle     (rad，发送时取负)
    uint8  _EOF                 = 0xFF
共 11 字节，小端（与 C++ __attribute__((packed)) + memcpy 一致）。

跟随规则：持续发包 = 跟随；停止发包 = 不跟随。
所以本脚本按固定频率连发 `duration` 秒，随后停发。

用法：
    python3 scripts/move_gimbal.py                        # 默认 yaw=0.1 rad 动一下
    python3 scripts/move_gimbal.py --yaw 0.2 --pitch -0.1 --hz 100 --duration 2
    python3 scripts/move_gimbal.py --port /dev/ttyUSB0
"""

import argparse
import fcntl
import os
import struct
import termios
import time

FRAME = struct.Struct("<BBffB")
SOF = 0x55
ID_AIMBOT = 0x02
EOF = 0xFF


def open_serial(port: str, baud: int) -> int:
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NDELAY)
    flags = fcntl.fcntl(fd, fcntl.F_GETFL)
    fcntl.fcntl(fd, fcntl.F_SETFL, flags | os.O_NDELAY)

    attrs = termios.tcgetattr(fd)
    speed = getattr(termios, f"B{baud}", termios.B460800)
    attrs[4] = attrs[5] = speed  # ispeed / ospeed

    # 8N1
    attrs[2] &= ~(termios.PARENB | termios.PARODD | termios.CSTOPB | termios.CSIZE)
    attrs[2] |= termios.CS8 | termios.CLOCAL | termios.CREAD
    # raw，无流控
    attrs[0] &= ~(termios.IXON | termios.IXOFF | termios.IXANY)
    attrs[0] &= ~(termios.INLCR | termios.IGNCR | termios.ICRNL)
    attrs[0] &= ~(termios.BRKINT | termios.INPCK | termios.ISTRIP)
    attrs[1] &= ~termios.OPOST
    attrs[3] &= ~(termios.ICANON | termios.ECHO | termios.ECHOE | termios.ISIG)
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 0

    termios.tcsetattr(fd, termios.TCSANOW, attrs)

    # 显式拉高 DTR/RTS（部分 STM32 CDC 固件据此判断“上位机已连接”才开始收数据）
    try:
        fcntl.ioctl(fd, 0x5416, 0x006)  # TIOCMBIS: DTR(0x2) | RTS(0x4)
    except OSError:
        pass
    return fd


def main() -> None:
    ap = argparse.ArgumentParser(description="按电控协议发 AimbotFrame 让云台动一下")
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=460800)
    ap.add_argument("--yaw", type=float, default=0.1, help="yaw 期望角 (rad)，默认 0.1")
    ap.add_argument("--pitch", type=float, default=0.0, help="pitch 期望角 (rad)")
    ap.add_argument("--duration", type=float, default=2.0, help="持续发包秒数")
    ap.add_argument("--hz", type=float, default=100.0, help="发包频率")
    args = ap.parse_args()

    fd = open_serial(args.port, args.baud)
    # 发送帧里角度取负（与 serial_node.cpp 的 -期望角 一致）
    frame = FRAME.pack(SOF, ID_AIMBOT, -args.pitch, -args.yaw, EOF)
    period = 1.0 / args.hz

    print(f"向 {args.port}@{args.baud} 下发 AimbotFrame："
          f"pitch={args.pitch:.3f}rad yaw={args.yaw:.3f}rad，持续 {args.duration}s")
    end = time.monotonic() + args.duration
    n = 0
    try:
        while time.monotonic() < end:
            nw = os.write(fd, frame)
            if nw != len(frame):
                print(f"警告: write 返回 {nw}/{len(frame)}")
            n += 1
            time.sleep(period)
    except KeyboardInterrupt:
        print("\n手动中断")
    finally:
        os.close(fd)

    print(f"完成：共下发 {n} 帧（已停发 = 不跟随）")


if __name__ == "__main__":
    main()
