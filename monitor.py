#!/usr/bin/env python3
"""Drone serial monitor – Live-Dashboard, pure stdlib (keine Abhängigkeiten).

Usage:
  ./monitor.py                   # Default: /dev/ttyUSB0
  ./monitor.py /dev/ttyAMA0      # eigener Port
  ./monitor.py -b 57600          # eigene Baudrate
  ./monitor.py --help
"""

import argparse
import math
import os
import re
import select
import sys
import termios
import time
from dataclasses import dataclass
from typing import Optional

# ── Regex für das Debug-Format (aus main.c) ───────────────────────────────
# ax=   +42 ay=    +10 az=  -100 gx=   +42 gy=    +10 gz=   -100 ARM
DEBUG_RE = re.compile(
    r"ax=\s*([+-]?\d+)\s+"
    r"ay=\s*([+-]?\d+)\s+"
    r"az=\s*([+-]?\d+)\s+"
    r"gx=\s*([+-]?\d+)\s+"
    r"gy=\s*([+-]?\d+)\s+"
    r"gz=\s*([+-]?\d+)\s+"
    r"(ARM|DIS)\s+"
    r"(\d+)"
)



@dataclass
class DroneData:
    ax: int = 0
    ay: int = 0
    az: int = 0
    gx: int = 0
    gy: int = 0
    gz: int = 0
    armed: bool = False
    avg_ticks: int = 0
    updated: float = 0.0


def parse_line(line: str, data: DroneData) -> Optional[DroneData]:
    m = DEBUG_RE.search(line)
    if not m:
        return None
    data.ax = int(m.group(1))
    data.ay = int(m.group(2))
    data.az = int(m.group(3))
    data.gx = int(m.group(4))
    data.gy = int(m.group(5))
    data.gz = int(m.group(6))
    data.armed = m.group(7) == "ARM"
    data.avg_ticks = int(m.group(8))
    data.updated = time.time()
    return data


# ── Serial (nur stdlib – termios) ─────────────────────────────────────────

def open_serial(port: str, baud: int) -> int:
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY)
    attrs = termios.tcgetattr(fd)

    baud_map = {
        9600: termios.B9600,
        19200: termios.B19200,
        38400: termios.B38400,
        57600: termios.B57600,
        115200: termios.B115200,
        230400: termios.B230400,
    }
    b = baud_map.get(baud, termios.B115200)

    attrs[4] = b
    attrs[5] = b
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

    dt = time.time() - data.updated if data.updated else 99
    stale = dt > 0.5

    armed_sym = f"{GREEN}ARMED{RESET}" if data.armed else f"{RED}DISARMED{RESET}"
    stale_warn = f"  {YELLOW}\u26a0 no signal for {dt:.0f}s{RESET}" if stale else ""

    lines = []

    # ── Header ────────────────────────────────────────────────────────────
    avg_us = data.avg_ticks // 2
    loop_hz = 2000000 // data.avg_ticks if data.avg_ticks else 0
    lines.append(
        f"{BOLD}IMU MONITOR{RESET}  {LGRAY}{port}{RESET}  "
        f"|  {armed_sym}  |  {fps:4.0f} Hz ({loop_hz:4.0f} loop)  "
        f"|  {avg_us:5d} us/loop"
        f"{stale_warn}"
    )
    lines.append("")

    # ── Convert to physical units ────────────────────────────────────────
    ACCEL_SENS = 16384.0  # ±2g → LSB/g
    GYRO_SENS = 131.0     # ±250°/s → LSB/°/s

    ax_g = data.ax / ACCEL_SENS
    ay_g = data.ay / ACCEL_SENS
    az_g = data.az / ACCEL_SENS

    gx_dps = data.gx / GYRO_SENS
    gy_dps = data.gy / GYRO_SENS
    gz_dps = data.gz / GYRO_SENS

    # attitude from accelerometer
    pitch_deg = math.atan2(-ax_g, math.sqrt(ay_g * ay_g + az_g * az_g)) * 180.0 / math.pi
    roll_deg = math.atan2(ay_g, az_g) * 180.0 / math.pi

    # ── Color helpers ────────────────────────────────────────────────────
    def acc_color(g: float) -> str:
        return RED if abs(g) >= 1.5 else (YELLOW if abs(g) >= 0.3 else GREEN)

    def gyro_color(dps: float) -> str:
        return RED if abs(dps) >= 90 else (YELLOW if abs(dps) >= 10 else GREEN)

    # ── Accelerometer ────────────────────────────────────────────────────
    acc_str = (f"{acc_color(ax_g)}{ax_g:+6.2f}{RESET}g  "
               f"{acc_color(ay_g)}{ay_g:+6.2f}{RESET}g  "
               f"{acc_color(az_g)}{az_g:+6.2f}{RESET}g")

    att_str = f"Pitch {GREEN}{pitch_deg:+6.1f}{RESET}\u00b0  Roll {GREEN}{roll_deg:+6.1f}{RESET}\u00b0"

    # ── Gyroscope ────────────────────────────────────────────────────────
    gyr_str = (f"{gyro_color(gx_dps)}{gx_dps:+7.1f}{RESET}\u00b0/s  "
               f"{gyro_color(gy_dps)}{gy_dps:+7.1f}{RESET}\u00b0/s  "
               f"{gyro_color(gz_dps)}{gz_dps:+7.1f}{RESET}\u00b0/s")

    # ── Layout ──────────────────────────────────────────────────────────
    sep = f"  {BOLD}{LGRAY}{'─' * (w // 2)}{RESET}"
    h, _ = os.get_terminal_size()
    if h >= 12:
        lines.append("")
        lines.append(f"  {BOLD}ACCEL{RESET}")
        lines.append(sep)
        lines.append(f"    {acc_str}")
        lines.append(f"    {att_str}")
        lines.append(sep)
        lines.append("")
        lines.append(f"  {BOLD}GYRO{RESET}")
        lines.append(sep)
        lines.append(f"    {gyr_str}")
        lines.append(sep)
    else:
        lines.append(f"  {BOLD}A{RESET} {acc_str}")
        lines.append(f"  {BOLD}G{RESET} {gyr_str}")

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
    ap.add_argument("-b", "--baud", type=int, default=230400, help="Baud rate")
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
