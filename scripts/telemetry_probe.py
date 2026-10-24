#!/usr/bin/env python3
"""客观探测云台是否响应：一边读电控回传的四元数算 yaw，一边下发指令。

用法：
    python3 scripts/telemetry_probe.py                 # 11 字节帧，yaw=0.5rad
    python3 scripts/telemetry_probe.py --size 29       # 29 字节帧
    python3 scripts/telemetry_probe.py --yaw 0.5 --duration 3

输出：时间戳(ms) + 云台 yaw(°) 的时间线，若指令期 yaw 明显变化说明云台在动。
"""
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
        fcntl.ioctl(fd, 0x5416, array.array("i", [0x006]))  # DTR|RTS
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


def yaw_from_quat(q0, q1, q2, q3):
    """w,x,y,z -> yaw(rad)，绕 Z 轴。"""
    return math.atan2(2.0 * (q0 * q3 + q1 * q2), 1.0 - 2.0 * (q2 * q2 + q3 * q3))


def make_frame(size, yaw, pitch):
    if size == 11:
        return struct.pack("<BBffB", 0x55, 0x02, -pitch, -yaw, 0xFF)
    if size == 29:
        # SOF ID AimbotState AimbotTarget pitch yaw dist ps ys timer EOF
        return struct.pack("<BBBBfffffIB", 0x55, 0x02, 0x01, 0x00,
                           -pitch, -yaw, 0.0, 0.0, 0.0, 0, 0xFF)
    if size == 19:
        # 对称猜测：SOF ID + 4 float + EOF（pitch,yaw,pitch_speed,yaw_speed）
        return struct.pack("<BBffffB", 0x55, 0x02, -pitch, -yaw, 0.0, 0.0, 0xFF)
    raise ValueError(f"未知帧大小 {size}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=460800)
    ap.add_argument("--size", type=int, default=11, choices=[11, 19, 29])
    ap.add_argument("--yaw", type=float, default=0.5)
    ap.add_argument("--pitch", type=float, default=0.0)
    ap.add_argument("--baseline", type=float, default=1.5)
    ap.add_argument("--duration", type=float, default=3.0)
    ap.add_argument("--hz", type=float, default=100.0)
    args = ap.parse_args()

    fd = open_serial(args.port, args.baud)
    frame = make_frame(args.size, args.yaw, args.pitch)

    samples = []  # (t, q0,q1,q2,q3)
    stop = threading.Event()

    def reader():
        buf = b""
        t0 = time.monotonic()
        while not stop.is_set():
            try:
                d = os.read(fd, 4096)
                if d:
                    buf += d
                    # 解析 19 字节帧：55 01 [4f] ff
                    while True:
                        i = buf.find(b"\x55")
                        if i < 0 or len(buf) < i + 19:
                            break
                        if buf[i + 1] == 0x01 and buf[i + 18] == 0xFF:
                            q = struct.unpack_from("<ffff", buf, i + 2)
                            samples.append((time.monotonic() - t0, q))
                            buf = buf[i + 19:]
                        else:
                            buf = buf[i + 1:]
            except BlockingIOError:
                pass
            time.sleep(0.001)

    th = threading.Thread(target=reader, daemon=True)
    th.start()

    print(f"[{args.size}字节帧] 基线 {args.baseline}s ...")
    time.sleep(args.baseline)

    print(f"下发 yaw={args.yaw}rad pitch={args.pitch}rad，{args.hz}Hz 持续 {args.duration}s ...")
    send_t0 = time.monotonic()
    end = send_t0 + args.duration
    period = 1.0 / args.hz
    n = 0
    while time.monotonic() < end:
        try:
            os.write(fd, frame)
            n += 1
        except OSError:
            pass
        time.sleep(period)

    time.sleep(0.5)  # 停发后再观察 0.5s
    stop.set()
    th.join(timeout=1.0)
    os.close(fd)

    if not samples:
        print("未收到任何电控四元数帧！")
        return

    def mean_quat(qs):
        m = [0.0, 0.0, 0.0, 0.0]
        for q in qs:
            if sum(a * b for a, b in zip(m, q)) < 0:
                q = tuple(-x for x in q)
            for k in range(4):
                m[k] += q[k]
        norm = math.sqrt(sum(x * x for x in m))
        return tuple(x / norm for x in m) if norm > 0 else None

    def angle_between(qa, qb):
        d = min(1.0, abs(sum(a * b for a, b in zip(qa, qb))))
        return 2.0 * math.acos(d)

    base = [q for t, q in samples if t < args.baseline - 0.2]
    send = [q for t, q in samples if args.baseline - 0.2 <= t < args.baseline + args.duration + 0.3]
    after = [q for t, q in samples if t >= args.baseline + args.duration + 0.3]

    mb = mean_quat(base)
    ms = mean_quat(send)
    ma = mean_quat(after)

    print(f"共解析 {len(samples)} 帧；已下发 {n} 帧")
    print(f"  基线期 n={len(base)}  指令期 n={len(send)}  停发后 n={len(after)}")
    if mb and ms:
        print(f"  >>> 指令期相对基线 云台转动 = {math.degrees(angle_between(mb, ms)):+.2f}°")
    if ms and ma:
        print(f"  >>> 停发后相对指令期 云台转动 = {math.degrees(angle_between(ms, ma)):+.2f}°")
    if mb and ma:
        print(f"  >>> 停发后相对基线 云台转动 = {math.degrees(angle_between(mb, ma)):+.2f}°")


if __name__ == "__main__":
    main()
