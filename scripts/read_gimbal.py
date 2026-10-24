#!/usr/bin/env python3
"""只读串口原始字节，诊断电控是否在发数据、波特率是否匹配、帧长多少。

不发送任何数据，纯读取。默认读 1.5s，打印十六进制预览 + 0x55 帧头位置与间距。
"""

import argparse
import os
import termios
import time


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


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=460800)
    ap.add_argument("--duration", type=float, default=1.5)
    args = ap.parse_args()

    fd = open_serial(args.port, args.baud)
    data = b""
    end = time.monotonic() + args.duration
    while time.monotonic() < end:
        try:
            chunk = os.read(fd, 256)
        except BlockingIOError:
            chunk = b""
        if chunk:
            data += chunk
    os.close(fd)

    print(f"@{args.baud} 读到 {len(data)} 字节")
    if data:
        print("hex:", data[:256].hex(" "))
        idx = [i for i, b in enumerate(data) if b == 0x55]
        print(f"0x55 帧头出现 {len(idx)} 次，位置: {idx[:20]}")
        if len(idx) > 1:
            gaps = [idx[i + 1] - idx[i] for i in range(min(len(idx) - 1, 20))]
            print(f"相邻 0x55 间距: {gaps}")
    else:
        print("（没读到任何字节 —— 电控没往外发，或波特率不对）")


if __name__ == "__main__":
    main()
