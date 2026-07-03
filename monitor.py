#!/usr/bin/env python3
"""Drone serial monitor – Live-Dashboard, pure stdlib (keine Abhängigkeiten).

Usage:
  ./monitor.py                   # Default: /dev/ttyUSB0
  ./monitor.py /dev/ttyAMA0      # eigener Port
  ./monitor.py -b 57600          # eigene Baudrate
  ./monitor.py --help
"""

import argparse
import os
import re
import select
import sys
import termios
import time
from dataclasses import dataclass, field
from typing import Optional

# ── Regex für das Debug-Format (aus main.c) ───────────────────────────────
# thr= 1500 rp=  +0 er=    +0 p=  +0 i=     0 d=  +0 out=  +0 m 1000 ...
DEBUG_RE = re.compile(
    r"thr=\s*(\d+)\s+"
    r"rp=\s*([+-]?\d+)\s+"
    r"er=\s*([+-]?\d+)\s+"
    r"p=\s*([+-]?\d+)\s+"
    r"i=\s*([+-]?\d+)\s+"
    r"d=\s*([+-]?\d+)\s+"
    r"out=\s*([+-]?\d+)\s+"
    r"m\s*([+-]?\d+)\s*([+-]?\d+)\s*([+-]?\d+)\s*([+-]?\d+)\s+"
    r"e=(\d{3})(\d{3})(\d{3})(\d{3})\s+"
    r"(ARM|DIS)"
)

# Motor-Reihenfolge aus control_get_motors: 0=BR, 1=FL, 2=FR, 3=BL
MOTOR_NAMES = ["BR", "FL", "FR", "BL"]


@dataclass
class DroneData:
    thr: int = 0
    rp: int = 0
    er: int = 0
    p: int = 0
    i: int = 0
    d: int = 0
    out: int = 0
    motors: list = field(default_factory=lambda: [0, 0, 0, 0])
    esc_raw: list = field(default_factory=lambda: [0, 0, 0, 0])
    armed: bool = False
    updated: float = 0.0


def parse_line(line: str, data: DroneData) -> Optional[DroneData]:
    m = DEBUG_RE.search(line)
    if not m:
        return None
    data.thr = int(m.group(1))
    data.rp = int(m.group(2))
    data.er = int(m.group(3))
    data.p = int(m.group(4))
    data.i = int(m.group(5))
    data.d = int(m.group(6))
    data.out = int(m.group(7))
    data.motors = [int(m.group(8 + i)) for i in range(4)]
    data.esc_raw = [int(m.group(12 + i)) for i in range(4)]
    data.armed = m.group(16) == "ARM"
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
RESET = CSI + "0m"
BOLD = CSI + "1m"
DIM = CSI + "2m"
GREEN = CSI + "32m"
RED = CSI + "31m"
YELLOW = CSI + "33m"
CYAN = CSI + "36m"
LGRAY = CSI + "90m"


def pos(y: int, x: int = 0):
    return CSI + f"{y + 1};{x + 1}H"


# ── Dashboard ─────────────────────────────────────────────────────────────

def render(data: DroneData, raw_lines: list[str], port: str, fps: float):
    w = os.get_terminal_size().columns
    lines = []

    dt = time.time() - data.updated if data.updated else 99
    stale = dt > 0.5

    armed_sym = f"{GREEN}ARMED{RESET}" if data.armed else f"{RED}DISARMED{RESET}"
    stale_warn = f"  {YELLOW}⚠ no signal for {dt:.0f}s{RESET}" if stale else ""

    # ── Header ────────────────────────────────────────────────────────────
    lines.append(
        f"{BOLD}DRONE MONITOR{RESET}  {LGRAY}{port}{RESET}  "
        f"|  {armed_sym}  |  {fps:4.0f} Hz  |  thr {data.thr:4d}"
        f"{stale_warn}"
    )
    lines.append("")

    # ── PID + Motors 2-Spalten-Layout ────────────────────────────────────
    col_w = max(20, w // 2 - 2)

    # Linke Spalte: Pitch PID
    lines.append(f"{BOLD}PID Pitch{RESET}               {BOLD}Motors{RESET}")
    lines.append(f"  {CYAN}P:{RESET} {data.p:+6d}            "
                 f"  {MOTOR_NAMES[2]}  {data.motors[2]:+5d}")
    lines.append(f"  {CYAN}I:{RESET} {data.i:+6d}            "
                 f"  {MOTOR_NAMES[1]}  {data.motors[1]:+5d}")
    lines.append(f"  {CYAN}D:{RESET} {data.d:+6d}            "
                 f"  {MOTOR_NAMES[0]}  {data.motors[0]:+5d}")
    lines.append(f"  {CYAN}Err:{RESET} {data.er:+6d}           "
                 f"  {MOTOR_NAMES[3]}  {data.motors[3]:+5d}")
    lines.append(f"  {CYAN}Out:{RESET} {data.out:+6d}")
    lines.append("")

    # ── ESC Raw ─────────────────────────────────────────────────────────
    lines.append(f"{BOLD}ESC Raw{RESET}")
    for i in range(4):
        lines.append(f"  {MOTOR_NAMES[i]}  {data.esc_raw[i]:3d}")
    lines.append("")

    # ── Steuerungshinweise ────────────────────────────────────────────────
    lines.append(f"{DIM}[q] quit  [c] clear raw buffer{RESET}")

    # ── Raw Ausgabe ──────────────────────────────────────────────────────
    lines.append(f"{DIM}── Raw ──────────────────────────────────────────────{RESET}")

    h, _ = os.get_terminal_size()
    max_raw = h - len(lines) - 2
    for rl in raw_lines[-max_raw:]:
        rl = rl.rstrip("\r\n")
        lines.append(f"  {rl}")

    out = "\n".join(lines)
    max_line_w = max((len(l) for l in lines), default=0)
    if max_line_w > w:
        out = "\n".join(l[:w] for l in lines)
    print(pos(0) + out, end="")
    sys.stdout.flush()


# ── Init-Screen (vor connect) ─────────────────────────────────────────────

def init_screen():
    print(HIDE + CLS, end="")
    sys.stdout.flush()
    import atexit
    atexit.register(lambda: print(SHOW, end="") or sys.stdout.flush())


# ── Main ──────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description="Drone Serial Monitor (no deps)")
    ap.add_argument("port", nargs="?", default="/dev/ttyUSB0", help="Serial port")
    ap.add_argument("-b", "--baud", type=int, default=115200, help="Baud rate")
    args = ap.parse_args()

    init_screen()
    data = DroneData()
    raw_lines: list[str] = []
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
                elif ch == "c":
                    raw_lines.clear()

            # ── Verbinden ─────────────────────────────────────────────────
            if fd is None:
                try:
                    fd = open_serial(args.port, args.baud)
                    raw_lines.append(f"{GREEN}Connected to {args.port}{RESET}")
                except OSError as e:
                    ts = time.strftime("%H:%M:%S")
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
                        raw_lines.append(f"{YELLOW}Port closed{RESET}")
                        disconnect()
                        continue
                    buf += chunk
                    while b"\n" in buf:
                        line_bytes, buf = buf.split(b"\n", 1)
                        line = line_bytes.decode("utf-8", errors="replace").rstrip("\r")
                        raw_lines.append(line)
                        parse_line(line, data)
            except OSError:
                raw_lines.append(f"{RED}Connection lost{RESET}")
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
                    render(data, raw_lines, args.port, fps)
                except OSError:
                    break
    except KeyboardInterrupt:
        pass
    finally:
        disconnect()
        print(SHOW + "\n  bye.\n")


if __name__ == "__main__":
    main()
