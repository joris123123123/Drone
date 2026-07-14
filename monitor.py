#!/usr/bin/env python3
"""Drone serial monitor – Live-Dashboard, pure stdlib (keine Abhängigkeiten).

Usage:
  ./monitor.py                   # Default: /dev/ttyUSB0
  ./monitor.py /dev/ttyAMA0      # eigener Port
  ./monitor.py -b 57600          # eigene Baudrate
  ./monitor.py --help
"""

import argparse
import fcntl
import os
import re
import select
import struct
import subprocess
import sys
import termios
import time
from collections import deque
from dataclasses import dataclass
from typing import Optional

# ── Regex für das Debug-Format (aus main.c) ───────────────────────────────
# ax=   +42 ay=    +10 az=  -100 gx=   +42 gy=    +10 gz=   -100 ARM
DEBUG_RE = re.compile(
    r"p=\s*([+-]?\d+)\s+"
    r"r=\s*([+-]?\d+)\s+"
    r"gx=\s*([+-]?\d+)\s+"
    r"gy=\s*([+-]?\d+)\s+"
    r"gz=\s*([+-]?\d+)\s+"
    r"(ARM|DIS)\s+"
    r"(\d+)"
)



@dataclass
class DroneData:
    pitch: int = 0
    roll: int = 0
    gx: int = 0
    gy: int = 0
    gz: int = 0
    armed: bool = False
    avg_ticks: int = 0
    updated: float = 0.0
    log: deque = None

    def __post_init__(self):
        if self.log is None:
            self.log = deque(maxlen=8)


def parse_line(line: str, data: DroneData) -> Optional[DroneData]:
    m = DEBUG_RE.search(line)
    if m:
        data.pitch = int(m.group(1))
        data.roll = int(m.group(2))
        data.gx = int(m.group(3))
        data.gy = int(m.group(4))
        data.gz = int(m.group(5))
        data.armed = m.group(6) == "ARM"
        data.avg_ticks = int(m.group(7))
        data.updated = time.time()
        return data
    if line.strip():
        data.log.append(line.strip())
    return None


# ── Serial (nur stdlib – termios) ─────────────────────────────────────────

def set_baud_termios2(fd: int, baud: int) -> bool:
    try:
        TCGETS2 = 0x802C542A
        TCSETS2 = 0x402C542B
        BOTHER = 0x10000000
        buf = bytearray(44)
        fcntl.ioctl(fd, TCGETS2, buf, True)
        data = list(struct.unpack_from('<IIIIb19I', buf))
        data[6] = (data[6] & ~0x000F0000) | BOTHER
        data[7] = baud
        data[8] = baud
        struct.pack_into('<IIIIb19I', buf, 0, *data)
        fcntl.ioctl(fd, TCSETS2, buf, False)
        return True
    except Exception:
        return False


def open_serial(port: str, baud: int) -> int:
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY)

    std_baud = {9600, 19200, 38400, 57600, 115200, 230400}
    if baud not in std_baud and not set_baud_termios2(fd, baud):
        subprocess.run(['stty', '-F', port, str(baud)],
                       capture_output=True)

    attrs = termios.tcgetattr(fd)
    attrs[2] &= ~(termios.CSTOPB | termios.PARENB | termios.CSIZE)
    attrs[2] |= termios.CS8 | termios.CREAD
    attrs[0] &= ~(termios.ICANON | termios.ECHO | termios.ECHOE | termios.ISIG)
    attrs[1] &= ~(termios.IXON | termios.IXOFF | termios.IXANY)
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 1
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    return fd


def close_serial(fd: int):
    try:
        os.close(fd)
    except OSError:
        pass


# ── ANSI ──────────────────────────────────────────────────────────────────

CSI = "\033["
HIDE = CSI + "?25l"
SHOW = CSI + "?25h"
CLS = CSI + "2J" + CSI + "H"
CLR_END = CSI + "J"
RESET = CSI + "0m"
BOLD = CSI + "1m"
DIM = CSI + "2m"
GREEN = CSI + "32m"
RED = CSI + "31m"
YELLOW = CSI + "33m"
LGRAY = CSI + "90m"


def pos(y: int, x: int = 0):
    return CSI + f"{y + 1};{x + 1}H"


# ── Dashboard ─────────────────────────────────────────────────────────────

def render(data: DroneData, port: str, fps: float):
    w = os.get_terminal_size().columns

    armed_sym = f"{GREEN}ARMED{RESET}" if data.armed else f"{RED}DISARMED{RESET}"

    lines = []

    # ── Header ────────────────────────────────────────────────────────────
    avg_us = data.avg_ticks // 2
    loop_hz = 2000000 // data.avg_ticks if data.avg_ticks else 0
    lines.append(
        f"{BOLD}IMU MONITOR{RESET}  {LGRAY}{port}{RESET}  "
        f"|  {armed_sym}  |  {fps:4.0f} Hz ({loop_hz:4.0f} loop)  "
        f"|  {avg_us:5d} us/loop"
    )
    lines.append("")

    # ── Convert to physical units ────────────────────────────────────────
    GYRO_SENS = 131.0     # ±250°/s → LSB/°/s

    gx_dps = data.gx / GYRO_SENS
    gy_dps = data.gy / GYRO_SENS
    gz_dps = data.gz / GYRO_SENS

    pitch_deg = data.pitch / 10.0
    roll_deg = data.roll / 10.0

    # ── Color helpers ────────────────────────────────────────────────────
    def angle_color(deg: float) -> str:
        return RED if abs(deg) >= 30.0 else (YELLOW if abs(deg) >= 10.0 else GREEN)

    def gyro_color(dps: float) -> str:
        return RED if abs(dps) >= 90 else (YELLOW if abs(dps) >= 10 else GREEN)

    # ── Attitude ────────────────────────────────────────────────────────
    att_str = (f"Pitch {angle_color(pitch_deg)}{pitch_deg:+6.1f}{RESET}\u00b0  "
               f"Roll {angle_color(roll_deg)}{roll_deg:+6.1f}{RESET}\u00b0")

    # ── Gyroscope ────────────────────────────────────────────────────────
    gyr_str = (f"{gyro_color(gx_dps)}{gx_dps:+7.1f}{RESET}\u00b0/s  "
               f"{gyro_color(gy_dps)}{gy_dps:+7.1f}{RESET}\u00b0/s  "
               f"{gyro_color(gz_dps)}{gz_dps:+7.1f}{RESET}\u00b0/s")

    # ── Layout ──────────────────────────────────────────────────────────
    sep = f"  {BOLD}{LGRAY}{'─' * (w // 2)}{RESET}"
    h, _ = os.get_terminal_size()
    if h >= 10:
        lines.append("")
        lines.append(f"  {BOLD}ATTITUDE{RESET}")
        lines.append(sep)
        lines.append(f"    {att_str}")
        lines.append(sep)
        lines.append("")
        lines.append(f"  {BOLD}GYRO{RESET}")
        lines.append(sep)
        lines.append(f"    {gyr_str}")
        lines.append(sep)
    else:
        lines.append(f"  {BOLD}A{RESET} {att_str}")
        lines.append(f"  {BOLD}G{RESET} {gyr_str}")

    lines.append("")
    if data.log:
        lines.append(f"  {BOLD}LOG{RESET}")
        lines.append(sep)
        for entry in data.log:
            lines.append(f"    {LGRAY}{entry}{RESET}")
        lines.append(sep)
        lines.append("")
    lines.append(f"{DIM}[q] quit{RESET}")

    out = "\n".join(lines)
    max_line_w = max((len(l) for l in lines), default=0)
    if max_line_w > w:
        out = "\n".join(l[:w] for l in lines)
    print(pos(0) + out + CLR_END, end="")
    sys.stdout.flush()


# ── Init-Screen (vor connect) ─────────────────────────────────────────────

def init_screen():
    print(HIDE + CLS, end="")
    sys.stdout.flush()
    import atexit
    atexit.register(lambda: print(SHOW, end="") or sys.stdout.flush())


# ── Main ──────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description="Drone Gyro Monitor (no deps)")
    ap.add_argument("port", nargs="?", default="/dev/ttyUSB0", help="Serial port")
    ap.add_argument("-b", "--baud", type=int, default=100000, help="Baud rate")
    args = ap.parse_args()

    init_screen()
    data = DroneData()
    fd: Optional[int] = None
    buf = b""
    last_render = 0.0
    frame_count = 0
    fps = 0.0
    fps_timer = time.time()

    def disconnect():
        nonlocal fd, buf
        if fd is not None:
            close_serial(fd)
            fd = None
        buf = b""

    try:
        while True:
            # ── Tastendruck ───────────────────────────────────────────────
            if select.select([sys.stdin], [], [], 0)[0]:
                ch = sys.stdin.read(1)
                if ch == "q":
                    break

            # ── Verbinden ─────────────────────────────────────────────────
            if fd is None:
                try:
                    fd = open_serial(args.port, args.baud)
                except OSError as e:
                    msg = f"Waiting for {args.port}... ({e})"
                    print(pos(0) + f"  {YELLOW}{msg}{RESET}", end="")
                    sys.stdout.flush()
                    time.sleep(0.5)
                    continue

            # ── Lesen ─────────────────────────────────────────────────────
            try:
                r, _, _ = select.select([fd], [], [], 0.05)
                if r:
                    chunk = os.read(fd, 1024)
                    if not chunk:
                        disconnect()
                        continue
                    buf += chunk
                    while b"\n" in buf:
                        line_bytes, buf = buf.split(b"\n", 1)
                        line = line_bytes.decode("utf-8", errors="replace").rstrip("\r")
                        parse_line(line, data)
            except OSError:
                disconnect()
                continue

            # ── FPS ───────────────────────────────────────────────────────
            now = time.time()
            frame_count += 1
            if now - fps_timer >= 1.0:
                fps = frame_count / (now - fps_timer)
                frame_count = 0
                fps_timer = now

            # ── Rendern (max 20 Hz) ───────────────────────────────────────
            if now - last_render >= 0.05:
                last_render = now
                try:
                    render(data, args.port, fps)
                except OSError:
                    break
    except KeyboardInterrupt:
        pass
    finally:
        disconnect()
        print(SHOW + "\n  bye.\n")


if __name__ == "__main__":
    main()
