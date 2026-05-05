#!/usr/bin/env python3
"""Formal ROS-protocol smoke test for the CM-530 AX-position firmware.

This script intentionally uses only the handoff protocol:
PING, BEGIN, PT, END, STOP, and HOME.
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass

try:
    import serial
except ImportError as exc:
    print("FAIL: pyserial is not installed. Install it with: python -m pip install pyserial")
    raise SystemExit(2) from exc


DEFAULT_PORT = "COM4"
DEFAULT_BAUD = 57600
JOINT_ORDER = "j1->ID17, j2->ID3, j3->ID2, j4->ID7"


@dataclass
class StepResult:
    name: str
    passed: bool
    detail: str = ""


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Test CM-530 formal ROS ASCII protocol over serial."
    )
    parser.add_argument("--port", default=DEFAULT_PORT, help=f"Serial port, default: {DEFAULT_PORT}")
    parser.add_argument("--baud", default=DEFAULT_BAUD, type=int, help=f"Baud rate, default: {DEFAULT_BAUD}")
    parser.add_argument("--timeout", default=5.0, type=float, help="Base response timeout in seconds")
    parser.add_argument("--char-delay", default=0.05, type=float, help="Delay between TX bytes, default: 0.05s")
    parser.add_argument(
        "--line-end",
        choices=("lf", "cr", "crlf"),
        default="lf",
        help="Command line ending for app commands, default: lf.",
    )
    parser.add_argument("--no-motion", action="store_true", help="Only test PING/STOP; do not move the arm")
    parser.add_argument("--no-go", action="store_true", help="Do not try bootloader -GO recovery if PING fails")
    parser.add_argument("--no-pause", action="store_true", help="Exit immediately at the end")
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
    return parser.parse_args()


def print_header(args: argparse.Namespace) -> None:
    print("CM-530 Formal ROS Protocol Smoke Test")
    print(f"Port       : {args.port}")
    print(f"Baud       : {args.baud}")
    print(f"Timeout    : {args.timeout:.1f}s base")
    print(f"DTR/RTS    : {args.dtr}/{args.rts}")
    print(f"Line end   : {args.line_end}")
    print(f"Char delay : {args.char_delay:.2f}s")
    print(f"Joint order: {JOINT_ORDER}")
    print("Protocol   : PING / BEGIN / PT / END / STOP / HOME only")
    print(f"Mode       : {'communication only' if args.no_motion else 'small safe motion'}")
    print()


def print_result(result: StepResult) -> None:
    status = "PASS" if result.passed else "FAIL"
    suffix = f" - {result.detail}" if result.detail else ""
    print(f"{status}: {result.name}{suffix}")
    print()


def open_serial(args: argparse.Namespace) -> tuple[serial.Serial | None, StepResult]:
    try:
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
        time.sleep(0.5)
    except serial.SerialException as exc:
        detail = str(exc)
        if "PermissionError" in detail or "Access is denied" in detail:
            detail += " | Close RoboPlus Terminal and old test windows using COM4."
        return None, StepResult("open serial", False, detail)

    return ser, StepResult("open serial", True, f"{args.port} @ {args.baud}")


def send_bytes(ser: serial.Serial, payload: bytes, char_delay_s: float) -> None:
    for value in payload:
        ser.write(bytes([value]))
        ser.flush()
        if char_delay_s > 0:
            time.sleep(char_delay_s)


def line_ending(name: str) -> bytes:
    if name == "cr":
        return b"\r"
    if name == "crlf":
        return b"\r\n"
    return b"\n"


def send_command(ser: serial.Serial, command: str, char_delay_s: float, ending: bytes) -> None:
    print(f"TX -> {command}")
    send_bytes(ser, command.encode("ascii") + ending, char_delay_s)


def send_raw(ser: serial.Serial, payload: bytes, char_delay_s: float) -> None:
    print(f"TX raw -> {payload!r}")
    send_bytes(ser, payload, char_delay_s)


def read_available_lines(ser: serial.Serial, duration_s: float, quiet_s: float = 0.25) -> list[str]:
    lines: list[str] = []
    deadline = time.monotonic() + max(0.0, duration_s)
    last_rx = time.monotonic()

    while time.monotonic() < deadline:
        if ser.in_waiting:
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode("utf-8", errors="ignore").strip()
            if line:
                print(f"RX <- {line}")
                lines.append(line)
                last_rx = time.monotonic()
        else:
            if lines and (time.monotonic() - last_rx) >= quiet_s:
                break
            time.sleep(0.02)

    if not lines:
        print("RX <- (none)")
    return lines


def is_error_line(line: str) -> bool:
    return line.startswith("ERR,")


def send_and_expect(
    ser: serial.Serial,
    command: str,
    expected: str,
    timeout_s: float,
    char_delay_s: float,
    ending: bytes,
) -> StepResult:
    send_command(ser, command, char_delay_s, ending)
    deadline = time.monotonic() + max(0.1, timeout_s)
    lines: list[str] = []

    while time.monotonic() < deadline:
        if ser.in_waiting:
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode("utf-8", errors="ignore").strip()
            if not line:
                continue
            print(f"RX <- {line}")
            lines.append(line)

            if line == expected:
                return StepResult(command, True)
            if is_error_line(line):
                return StepResult(command, False, f"firmware returned {line}")
        else:
            time.sleep(0.02)

    if lines:
        return StepResult(command, False, f"expected {expected}, got: " + " | ".join(lines))
    return StepResult(command, False, "no response")


def ensure_app_running(ser: serial.Serial, args: argparse.Namespace) -> StepResult:
    ending = line_ending(args.line_end)
    result = send_and_expect(ser, "PING", "PONG", args.timeout, args.char_delay, ending)
    if result.passed or args.no_go:
        return result

    print("PING did not respond. If CM-530 is in bootloader, start the app now.")
    try:
        input("Press CM-530 RESET / power-cycle, then press Enter here to send -GO...")
    except EOFError:
        pass

    print("Listening before -GO:")
    read_available_lines(ser, 3.0)
    go_variants = (b"-GO\r", b"-GO\n", b"-GO\r\n", b"GO\r", b"GO\n", b"GO\r\n")
    for payload in go_variants:
        send_raw(ser, payload, max(args.char_delay, 0.15))
        print("Listening after GO variant:")
        read_available_lines(ser, 1.5)
        result = send_and_expect(ser, "PING", "PONG", args.timeout, args.char_delay, ending)
        if result.passed:
            return result
    return result


def run_motion_sequence(ser: serial.Serial, args: argparse.Namespace) -> list[StepResult]:
    ending = line_ending(args.line_end)
    tests = [
        ("HOME", "OK,HOME", max(args.timeout, 6.0)),
        ("BEGIN,1,4,3", "OK,BEGIN,1", args.timeout),
        ("PT,0,300,512,512,512,512", "OK,PT,0", max(args.timeout, 2.0)),
        ("PT,1,400,520,512,512,512", "OK,PT,1", max(args.timeout, 2.0)),
        ("PT,2,400,512,512,512,512", "OK,PT,2", max(args.timeout, 2.0)),
        ("END,1", "OK,END,1", max(args.timeout, 6.0)),
    ]

    results: list[StepResult] = []
    for command, expected, timeout_s in tests:
        result = send_and_expect(ser, command, expected, timeout_s, args.char_delay, ending)
        results.append(result)
        print_result(result)
        if not result.passed:
            break
    return results


def print_summary(results: list[StepResult]) -> int:
    passed = sum(1 for result in results if result.passed)
    total = len(results)

    print("Summary")
    print(f"Passed: {passed}/{total}")
    for result in results:
        status = "PASS" if result.passed else "FAIL"
        suffix = f" - {result.detail}" if result.detail else ""
        print(f"{status}: {result.name}{suffix}")
    return 0 if passed == total else 1


def run(args: argparse.Namespace) -> int:
    print_header(args)

    results: list[StepResult] = []
    ser, open_result = open_serial(args)
    results.append(open_result)
    print_result(open_result)
    if ser is None:
        return print_summary(results)

    try:
        print("Startup listen, formal firmware may stay silent:")
        read_available_lines(ser, 1.0)
        print()

        ping_result = ensure_app_running(ser, args)
        results.append(ping_result)
        print_result(ping_result)
        if not ping_result.passed:
            return print_summary(results)

        ending = line_ending(args.line_end)

        stop_result = send_and_expect(ser, "STOP", "OK,STOP", args.timeout, args.char_delay, ending)
        results.append(stop_result)
        print_result(stop_result)
        if not stop_result.passed or args.no_motion:
            return print_summary(results)

        results.extend(run_motion_sequence(ser, args))

        final_stop = send_and_expect(ser, "STOP", "OK,STOP", args.timeout, args.char_delay, ending)
        results.append(final_stop)
        print_result(final_stop)

        return print_summary(results)
    finally:
        ser.close()


def pause_if_needed(args: argparse.Namespace) -> None:
    if args.no_pause:
        return
    try:
        input("Press Enter to exit...")
    except EOFError:
        pass


if __name__ == "__main__":
    parsed = parse_args()
    code = 1
    try:
        code = run(parsed)
    finally:
        pause_if_needed(parsed)
    sys.exit(code)
