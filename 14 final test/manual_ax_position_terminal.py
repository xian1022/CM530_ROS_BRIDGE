#!/usr/bin/env python3
"""Interactive manual AX-position terminal for CM-530 final-test firmware.

This tool is for bench testing before ROS sends trajectory points.
It lets the operator type AX position integers from PowerShell:

    512
    512 520 504 512
    AX,512,520,504,512
    listen
    reset-go
    quiet
    center
    probe-tx
    HOME
    STOP

The firmware still owns all motor safety clamping. This script only formats
commands and prints TX/RX clearly.
"""

from __future__ import annotations

import argparse
import sys
import time
from typing import Iterable

try:
    import serial
except ImportError as exc:
    print("FAIL: pyserial is not installed. Install it with: python -m pip install pyserial")
    raise SystemExit(2) from exc


DEFAULT_PORT = "COM4"
DEFAULT_BAUD = 57600
JOINT_ORDER = "j1->ID17, j2->ID3, j3->ID2, j4->ID7"
TX_MODES = ("normal", "rts-high", "rts-low", "pulse-high", "pulse-low")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Manual AX-position terminal for CM-530 final-test firmware."
    )
    parser.add_argument("--port", default=DEFAULT_PORT, help=f"Serial port, default: {DEFAULT_PORT}")
    parser.add_argument("--baud", default=DEFAULT_BAUD, type=int, help=f"Baud rate, default: {DEFAULT_BAUD}")
    parser.add_argument("--timeout", default=2.0, type=float, help="Read window after each command, default: 2.0")
    parser.add_argument(
        "--startup-listen",
        default=4.0,
        type=float,
        help="Seconds to listen after opening COM port, default: 4.0",
    )
    parser.add_argument(
        "--line-end",
        choices=("lf", "cr", "crlf"),
        default="crlf",
        help="Line ending sent after each command, default: crlf.",
    )
    parser.add_argument(
        "--char-delay",
        default=0.15,
        type=float,
        help="Seconds to wait between transmitted bytes, default: 0.15.",
    )
    parser.add_argument(
        "--dtr",
        choices=("off", "on", "unchanged"),
        default="unchanged",
        help="DTR state after opening the serial port, default: unchanged.",
    )
    parser.add_argument(
        "--rts",
        choices=("off", "on", "unchanged"),
        default="unchanged",
        help="RTS state after opening the serial port, default: unchanged.",
    )
    parser.add_argument(
        "--tx-mode",
        choices=TX_MODES,
        default="normal",
        help="How to drive RTS while sending commands, default: normal.",
    )
    parser.add_argument(
        "--rts-settle",
        default=0.03,
        type=float,
        help="Seconds to wait after changing RTS, default: 0.03.",
    )
    return parser.parse_args()


def line_ending(name: str) -> bytes:
    if name == "cr":
        return b"\r"
    if name == "crlf":
        return b"\r\n"
    return b"\n"


def print_header(args: argparse.Namespace) -> None:
    print("CM-530 Manual AX Position Terminal")
    print(f"Port       : {args.port}")
    print(f"Baud       : {args.baud}")
    print(f"DTR/RTS    : {args.dtr}/{args.rts}")
    print(f"Line end   : {args.line_end}")
    print(f"Char delay : {args.char_delay:.2f}s")
    print(f"TX mode    : {args.tx_mode}")
    print(f"Joint order: {JOINT_ORDER}")
    print("Examples   : 512 | AX,512 | AX,512,520,504,512 | HOME | STOP")
    print("Utility    : listen | go | reset-go | quiet | center | probe-tx")
    print("Exit       : q, quit, exit")
    print()


def open_serial(args: argparse.Namespace) -> serial.Serial:
    ser = serial.Serial()
    ser.port = args.port
    ser.baudrate = args.baud
    ser.timeout = 0.1
    ser.bytesize = serial.EIGHTBITS
    ser.parity = serial.PARITY_NONE
    ser.stopbits = serial.STOPBITS_ONE
    ser.rtscts = False
    ser.dsrdtr = False
    if args.dtr != "unchanged":
        ser.dtr = args.dtr == "on"
    if args.rts != "unchanged":
        ser.rts = args.rts == "on"
    ser.open()
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    return ser


def read_available(ser: serial.Serial, duration_s: float, quiet_s: float = 0.25) -> list[str]:
    lines: list[str] = []
    deadline = time.monotonic() + max(0.0, duration_s)
    last_rx = time.monotonic()
    raw_tail = b""

    while time.monotonic() < deadline:
        if ser.in_waiting:
            raw = ser.readline()
            if not raw:
                continue
            raw_tail += raw
            while b"\n" in raw_tail or b"\r" in raw_tail:
                split_at = min(
                    [idx for idx in (raw_tail.find(b"\n"), raw_tail.find(b"\r")) if idx >= 0]
                )
                chunk = raw_tail[:split_at]
                raw_tail = raw_tail[split_at + 1 :]
                text = chunk.decode("utf-8", errors="replace").strip()
                if text:
                    print(f"RX <- {text}")
                    lines.append(text)
                    last_rx = time.monotonic()
        else:
            if lines and (time.monotonic() - last_rx) >= quiet_s:
                break
            time.sleep(0.02)

    if raw_tail:
        text = raw_tail.decode("utf-8", errors="replace").strip()
        if text:
            print(f"RX <- {text}")
            lines.append(text)

    if not lines:
        print("RX <- (none)")
    return lines


def set_rts_for_mode(ser: serial.Serial, mode: str, active: bool, settle_s: float) -> None:
    if mode in ("normal",):
        return
    if mode == "rts-high":
        ser.rts = True
    elif mode == "rts-low":
        ser.rts = False
    elif mode == "pulse-high":
        ser.rts = active
    elif mode == "pulse-low":
        ser.rts = not active
    time.sleep(max(0.0, settle_s))


def send_raw(
    ser: serial.Serial,
    payload: bytes,
    per_byte_delay_s: float = 0.0,
    tx_mode: str = "normal",
    rts_settle_s: float = 0.03,
) -> None:
    print(f"TX raw -> {payload!r}")
    for value in payload:
        set_rts_for_mode(ser, tx_mode, True, rts_settle_s)
        ser.write(bytes([value]))
        ser.flush()
        set_rts_for_mode(ser, tx_mode, False, rts_settle_s)
        if per_byte_delay_s > 0:
            time.sleep(per_byte_delay_s)


def has_app_output(lines: list[str]) -> bool:
    markers = ("Final Test Firmware", "DXL UART initialized", "ALIVE", "Go:")
    return any(marker in line for line in lines for marker in markers)


def send_go_sequence(ser: serial.Serial) -> bool:
    """Try to start CM-530 user app through the bootloader GO command."""
    print("Trying bootloader GO variants...")
    variants = [
        (b"-GO\r", 0.15),
        (b"-GO\n", 0.15),
        (b"-GO\r\n", 0.15),
        (b"GO\r", 0.15),
    ]

    for payload, delay_s in variants:
        send_raw(ser, payload, per_byte_delay_s=delay_s)
        lines = read_available(ser, 3.0)
        if has_app_output(lines):
            print("PASS: app/bootloader output detected after GO.")
            return True

    print("GO probe finished. If RX is still none, start the app from RoboPlus Terminal.")
    return False


def probe_tx_modes(ser: serial.Serial, args: argparse.Namespace) -> bool:
    """Try TX modes using NOP, a safe no-motion multi-byte command."""
    print("Probing multi-byte TX modes with safe command: NOP")
    print("First sending single-key Q to stop ALIVE during the probe.")
    send_raw(ser, b"Q", tx_mode=args.tx_mode, rts_settle_s=args.rts_settle)
    read_available(ser, 1.0)

    for mode in TX_MODES:
        print(f"\n--- TX mode probe: {mode} ---")
        send_command(ser, "NOP", line_ending(args.line_end), args.char_delay, mode, args.rts_settle)
        lines = read_available(ser, max(args.timeout, 2.0))
        if any("OK,NOP" in line for line in lines):
            args.tx_mode = mode
            print(f"PASS: TX mode selected: {mode}")
            return True

        # Finish any partial line before trying the next mode.
        send_raw(ser, b"\r\n", per_byte_delay_s=args.char_delay, tx_mode=mode, rts_settle_s=args.rts_settle)
        read_available(ser, 0.5)

    print("FAIL: no TX mode received OK,STOP.")
    print("Try RoboPlus Terminal manual input, or check the USB serial adapter direction control.")
    return False


def print_local_help() -> None:
    print("Local commands:")
    print("  listen    Read COM4 for 8 seconds without transmitting.")
    print("  go        Try bootloader -GO variants, then listen.")
    print("  reset-go  Prompt you to press RESET, listen, try -GO, then listen.")
    print("  quiet     Send single-key Q to stop ALIVE output.")
    print("  verbose   Send single-key V to enable ALIVE output.")
    print("  center    Send single-key C to move all joints to AX 512.")
    print("  stop1     Send single-key X to stop trajectory.")
    print("  probe-tx  Auto-select TX mode using NOP.")
    print("  mode NAME Set TX mode: normal/rts-high/rts-low/pulse-high/pulse-low.")
    print("  NOP       Safe no-motion multi-byte firmware probe.")
    print("  512       Send same AX position to all joints.")
    print("  AX,512    Send same AX position to all joints.")
    print("  AX,512,520,504,512")
    print("  HOME | STOP | HELP | PING")


def is_uint(text: str) -> bool:
    return text.isdigit()


def all_uint(parts: Iterable[str]) -> bool:
    return all(is_uint(part) for part in parts)


def normalize_command(text: str) -> str | None:
    stripped = text.strip()
    if not stripped:
        return None

    lower = stripped.lower()
    if lower in ("q", "quit", "exit"):
        return "EXIT"
    if lower in ("listen", "go", "reset-go", "probe-tx", "quiet", "verbose", "center", "stop1", "?"):
        return "LOCAL:" + lower
    if lower.startswith("mode "):
        return "LOCAL:" + lower

    upper = stripped.upper()
    if upper in ("PING", "P", "HELP", "HOME", "STOP", "NOP"):
        return upper

    if "," in stripped:
        parts = [part.strip() for part in stripped.split(",")]
        if parts and parts[0].upper() == "AX":
            return "AX," + ",".join(parts[1:])
        return stripped

    parts = stripped.split()
    if len(parts) == 1 and is_uint(parts[0]):
        return parts[0]
    if len(parts) == 4 and all_uint(parts):
        return "AX," + ",".join(parts)

    return stripped


def send_command(
    ser: serial.Serial,
    command: str,
    ending: bytes,
    char_delay_s: float,
    tx_mode: str,
    rts_settle_s: float,
) -> None:
    print(f"TX -> {command}  [mode={tx_mode}]")
    payload = command.encode("ascii") + ending
    for value in payload:
        set_rts_for_mode(ser, tx_mode, True, rts_settle_s)
        ser.write(bytes([value]))
        ser.flush()
        set_rts_for_mode(ser, tx_mode, False, rts_settle_s)
        if char_delay_s > 0:
            time.sleep(char_delay_s)


def interactive_loop(ser: serial.Serial, args: argparse.Namespace) -> int:
    ending = line_ending(args.line_end)

    print("Startup listen:")
    read_available(ser, args.startup_listen)
    print()

    print("Type an AX position command. Start safe with 512.")
    while True:
        try:
            typed = input("AX> ")
        except EOFError:
            print()
            return 0

        command = normalize_command(typed)
        if command is None:
            continue
        if command == "EXIT":
            return 0

        try:
            if command == "LOCAL:listen":
                print("Listening 8 seconds. You may press CM-530 RESET now.")
                read_available(ser, 8.0)
            elif command == "LOCAL:go":
                send_go_sequence(ser)
                print("Follow-up listen:")
                read_available(ser, 5.0)
            elif command == "LOCAL:reset-go":
                print("Press CM-530 RESET or power-cycle now. Listening 8 seconds...")
                lines = read_available(ser, 8.0)
                if has_app_output(lines):
                    print("PASS: app output detected; skipping bootloader GO.")
                else:
                    send_go_sequence(ser)
                print("Follow-up listen:")
                read_available(ser, 5.0)
            elif command == "LOCAL:probe-tx":
                probe_tx_modes(ser, args)
            elif command == "LOCAL:quiet":
                send_raw(ser, b"Q", tx_mode=args.tx_mode, rts_settle_s=args.rts_settle)
                read_available(ser, args.timeout)
            elif command == "LOCAL:verbose":
                send_raw(ser, b"V", tx_mode=args.tx_mode, rts_settle_s=args.rts_settle)
                read_available(ser, args.timeout)
            elif command == "LOCAL:center":
                send_raw(ser, b"C", tx_mode=args.tx_mode, rts_settle_s=args.rts_settle)
                read_available(ser, max(args.timeout, 4.0))
            elif command == "LOCAL:stop1":
                send_raw(ser, b"X", tx_mode=args.tx_mode, rts_settle_s=args.rts_settle)
                read_available(ser, args.timeout)
            elif command.startswith("LOCAL:mode "):
                mode = command.split(" ", maxsplit=1)[1]
                if mode in TX_MODES:
                    args.tx_mode = mode
                    print(f"TX mode set to: {mode}")
                else:
                    print("Invalid mode. Use: normal/rts-high/rts-low/pulse-high/pulse-low")
            elif command == "LOCAL:?":
                print_local_help()
            else:
                send_command(ser, command, ending, args.char_delay, args.tx_mode, args.rts_settle)
                read_available(ser, args.timeout)
        except serial.SerialException as exc:
            print(f"FAIL: serial error: {exc}")
            return 1


def main() -> int:
    args = parse_args()
    print_header(args)

    try:
        ser = open_serial(args)
    except serial.SerialException as exc:
        print(f"FAIL: open serial - {exc}")
        print("Close RoboPlus Terminal, other serial monitors, and old test windows using COM4.")
        return 1

    print(f"PASS: open serial - {args.port} @ {args.baud}")
    print()
    try:
        return interactive_loop(ser, args)
    finally:
        ser.close()


if __name__ == "__main__":
    sys.exit(main())
