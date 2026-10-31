#!/usr/bin/env python3
"""扫描候选帧格式，客观判断云台是否响应（四元数角距离）。"""
import argparse
import array
import fcntl
import math
import os
import struct
import termios
import threading
import time


def open_serial(port, baud):
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NDELAY)
    try:
        fcntl.ioctl(fd, 0x5416, array.array("i", [0x006]))
    except OSError:
        pass
    a = termios.tcgetattr(fd)
    a[4] = a[5] = getattr(termios, f"B{baud}", termios.B460800)
    a[2] &= ~(termios.PARENB | termios.PARODD | termios.CSTOPB | termios.CSIZE)
    a[2] |= termios.CS8 | termios.CLOCAL | termios.CREAD
    a[0] &= ~(termios.IXON | termios.IXOFF | termios.IXANY)
    a[0] &= ~(termios.INLCR | termios.IGNCR | termios.ICRNL)
    a[1] &= ~termios.OPOST
    a[3] &= ~(termios.ICANON | termios.ECHO | termios.ECHOE | termios.ISIG)
    a[6][termios.VMIN] = 0
    a[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, a)
    return fd


def candidates(pitch, yaw):
    p, y = -pitch, -yaw
    SOF, ID, EOF = 0x55, 0x02, 0xFF
    out = {}
    out["11B 约定"] = struct.pack("<BBffB", SOF, ID, p, y, EOF)
    out["12B +state"] = struct.pack("<BBBffB", SOF, ID, 0x01, p, y, EOF)
    out["13B +state+target"] = struct.pack("<BBBBffB", SOF, ID, 0x01, 0x01, p, y, EOF)
    out["13B yaw先"] = struct.pack("<BBBBffB", SOF, ID, 0x01, 0x01, y, p, EOF)
    out["15B +dist"] = struct.pack("<BBBBfffB", SOF, ID, 0x01, 0x01, p, y, 0.0, EOF)
    out["19B 4float"] = struct.pack("<BBffffB", SOF, ID, p, y, 0.0, 0.0, EOF)
    out["25B +ps+ys"] = struct.pack("<BBBBfffffB", SOF, ID, 0x01, 0x01, p, y, 0.0, 0.0, 0.0, EOF)
    out["29B 旧版全字段"] = struct.pack("<BBBBfffffIB", SOF, ID, 0x01, 0x01, p, y, 0.0, 0.0, 0.0, 0, EOF)
    return out


def mean_quat(qs):
    m = [0.0, 0.0, 0.0, 0.0]
    for q in qs:
        if sum(a * b for a, b in zip(m, q)) < 0:
            q = tuple(-x for x in q)
        for k in range(4):
            m[k] += q[k]
    n = math.sqrt(sum(x * x for x in m))
    return tuple(x / n for x in m) if n > 0 else None


def angle_between(qa, qb):
    d = min(1.0, abs(sum(a * b for a, b in zip(qa, qb))))
    return 2.0 * math.acos(d)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=460800)
    ap.add_argument("--yaw", type=float, default=0.6)
    ap.add_argument("--pitch", type=float, default=0.0)
    ap.add_argument("--send_s", type=float, default=1.5)
    ap.add_argument("--hz", type=float, default=100.0)
    ap.add_argument("--only", default=None, help="只测某个 label 前缀")
    args = ap.parse_args()

    fd = open_serial(args.port, args.baud)
    cand = candidates(args.pitch, args.yaw)

    samples = []  # (t, q)
    stop = threading.Event()
    t0 = time.monotonic()

    def reader():
        buf = b""
        while not stop.is_set():
            try:
                d = os.read(fd, 4096)
                if d:
                    buf += d
                    while True:
                        i = buf.find(b"\x55")
                        if i < 0 or len(buf) < i + 19:
                            break
                        if buf[i + 1] == 0x01 and buf[i + 18] == 0xFF:
                            samples.append((time.monotonic() - t0,
                                            struct.unpack_from("<ffff", buf, i + 2)))
                            buf = buf[i + 19:]
                        else:
                            buf = buf[i + 1:]
            except BlockingIOError:
                pass
            time.sleep(0.001)

    th = threading.Thread(target=reader, daemon=True)
    th.start()

    results = []
    period = 1.0 / args.hz
    for label, frame in cand.items():
        if args.only and args.only not in label:
            continue
        start = time.monotonic() - t0
        # 基线 0.8s
        time.sleep(0.8)
        base_t = start + 0.8
        # 发送
        end = time.monotonic() + args.send_s
        n = 0
        while time.monotonic() < end:
            try:
                os.write(fd, frame)
                n += 1
            except OSError:
                pass
            time.sleep(period)
        send_end_t = time.monotonic() - t0
        time.sleep(0.4)  # 停发观察

        base = [q for t, q in samples if start <= t < base_t]
        send = [q for t, q in samples if base_t <= t < send_end_t]
        mb, ms = mean_quat(base), mean_quat(send)
        deg = math.degrees(angle_between(mb, ms)) if (mb and ms) else float("nan")
        results.append((label, deg, len(base), len(send), n))
        print(f"  {label:18s} 转动={deg:+6.2f}°  (基线{len(base)}帧/指令{len(send)}帧/发{n})")

    stop.set()
    th.join(timeout=1.0)
    os.close(fd)

    print("\n=== 汇总（按转动量排序）===")
    for label, deg, _, _, _ in sorted(results, key=lambda r: -r[1]):
        flag = "  <-- 有响应!" if deg > 2.0 else ""
        print(f"  {label:18s} {deg:+6.2f}°{flag}")


if __name__ == "__main__":
    main()
