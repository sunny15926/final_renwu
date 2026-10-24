#!/usr/bin/env python3
"""读取云台 IMU 四元数，解算 yaw/pitch/roll 并打印（只读，不发任何数据）。

四元数约定 q0 q1 q2 q3 = w x y z。yaw 为绕 z 轴（水平转向）的角。
"""

import argparse
import math
import os
import struct
import termios
import time

SOF = 0x55
IMU_ID = 0x01
EOF = 0xFF
FRAME = struct.Struct("<BBffffB")  # 19 字节


def open_serial(port: str, baud: int) -> int:
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NDELAY)
    attrs = termios.tcgetattr(fd)
    speed = getattr(termios, f"B{baud}", termios.B460800)
    attrs[4] = attrs[5] = speed
    attrs[2] &= ~(termios.PARENB | termios.PARODD | termios.CSTOPB | termios.CSIZE)
    attrs[2] |= termios.CS8 | termios.CLOCAL | termios.CREAD
    attrs[0] &= ~(termios.IXON | termios.IXOFF | termios.IXANY)
    attrs[0] &= ~(termios.INLCR | termios.IGNCR | termios.ICRNL)
    attrs[0] &= ~(termios.BRKINT | termios.INPCK | termios.ISTRIP)
    attrs[1] &= ~termios.OPOST
    attrs[3] &= ~(termios.ICANON | termios.ECHO | termios.ECHOE | termios.ISIG)
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    return fd


def quat_to_ypr(w, x, y, z):
    # 标准 z-y-x (yaw-pitch-roll)，单位 rad
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return yaw, pitch, roll


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=460800)
    ap.add_argument("--frames", type=int, default=10)
    args = ap.parse_args()

    fd = open_serial(args.port, args.baud)
    buf = b""
    got = 0
    end = time.monotonic() + 2.0
    while got < args.frames and time.monotonic() < end:
        try:
            buf += os.read(fd, 256)
        except BlockingIOError:
            time.sleep(0.005)
        # 解析出完整 19 字节 IMU 帧
        while len(buf) >= 19:
            if buf[0] == SOF and buf[1] == IMU_ID and buf[18] == EOF:
                _sof, _id, q0, q1, q2, q3, _eof = FRAME.unpack(buf[:19])
                yaw, pitch, roll = quat_to_ypr(q0, q1, q2, q3)
                print(f"帧{got}: yaw={math.degrees(yaw):8.2f}° "
                      f"pitch={math.degrees(pitch):8.2f}° "
                      f"roll={math.degrees(roll):8.2f}° "
                      f"(q0={q0:.4f} q1={q1:.4f} q2={q2:.4f} q3={q3:.4f})")
                got += 1
                buf = buf[19:]
            else:
                buf = buf[1:]  # 重新对齐帧头
    os.close(fd)
    print(f"共解析 {got} 帧")


if __name__ == "__main__":
    main()
