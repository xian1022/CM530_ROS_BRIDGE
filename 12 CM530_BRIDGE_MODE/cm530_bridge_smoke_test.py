#!/usr/bin/env python3
"""Smoke test for the CM-530 bridge-mode firmware.

The firmware uses an ASCII command protocol on the PC serial port:
PING, HELP, HOME, BEGIN, PT, END, and STOP.
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass
from typing import Callable, Iterable

try:
    import serial
except ImportError as exc:
    print("FAIL: pyserial is not installed. Install it with: python -m pip install pyserial")
    raise SystemExit(2) from exc


DEFAULT_PORT = "COM4"
DEFAULT_BAUD = 57600
JOINT_ORDER = "j1->ID17, j2->ID3, j3->ID2, j4->ID7"


Predicate = Callable[[str], bool]


@dataclass(frozen=True)
class ExpectedLine:
    label: str
    predicate: Predicate


@dataclass
class StepResult:
    name: str
    passed: bool
    detail: str = ""


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run a serial smoke test against CM-530 bridge-mode firmware."
    )
    parser.add_argument("--port", default=DEFAULT_PORT, help=f"Serial port, default: {DEFAULT_PORT}")
    parser.add_argument("--baud", default=DEFAULT_BAUD, type=int, help=f"Baud rate, default: {DEFAULT_BAUD}")
    parser.add_argument("--timeout", default=2.0, type=float, help="Base response timeout in seconds")
    parser.add_argument(
        "--no-motion",
        action="store_true",
        help="Only test communication commands; skip HOME and trajectory motion.",
    )
    parser.add_argument(
        "--no-pause",
        action="store_true",
        help="Exit immediately instead of waiting for Enter at the end.",
    )
    parser.add_argument(
        "--dtr",
        choices=("off", "on", "unchanged"),
        default="off",
        help="DTR state after opening the serial port, default: off.",
    )
    parser.add_argument(
        "--rts",
        choices=("off", "on", "unchanged"),
        default="off",
        help="RTS state after opening the serial port, default: off.",
    )
    return parser.parse_args()


def line_equals(expected: str) -> ExpectedLine:
    return ExpectedLine(expected, lambda line: line == expected)


def line_contains(label: str, text: str) -> ExpectedLine:
    return ExpectedLine(label, lambda line: text in line)


def is_error_line(line: str) -> bool:
    lower = line.lower()
    return (
        line.startswith("ERR,")
        or line.startswith("COMM_")
        or lower.endswith("error!")
        or "communication error" in lower
    )


def print_header(args: argparse.Namespace) -> None:
    print("CM-530 Bridge Mode Smoke Test")
    print(f"Port       : {args.port}")
    print(f"Baud       : {args.baud}")
    print(f"Timeout    : {args.timeout:.1f}s base")
    print(f"DTR/RTS    : {args.dtr}/{args.rts}")
    print(f"Joint order: {JOINT_ORDER}")
    print("Positions  : integer AX units, expected range 0..1023")
    print(f"Mode       : {'communication only' if args.no_motion else 'small safe motion'}")
    print()


def print_result(result: StepResult) -> None:
    status = "PASS" if result.passed else "FAIL"
    if result.detail:
        print(f"{status}: {result.name} - {result.detail}")
    else:
        print(f"{status}: {result.name}")
    print()


def read_available_lines(ser: serial.Serial, duration_s: float, quiet_s: float = 0.2) -> list[str]:
    """Read lines until duration expires, stopping early after quiet output."""
    lines: list[str] = []
    deadline = time.monotonic() + duration_s
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
        print("RX <- (no response)")

    return lines


def send_and_expect(
    ser: serial.Serial,
    command: str,
    expected: Iterable[ExpectedLine],
    timeout_s: float,
) -> StepResult:
    expected_list = list(expected)
    seen = {item.label: False for item in expected_list}

    print(f"TX -> {command}")
    ser.write((command + "\n").encode("ascii"))
    ser.flush()

    deadline = time.monotonic() + timeout_s
    last_rx = time.monotonic()
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
            last_rx = time.monotonic()

            if is_error_line(line):
                return StepResult(command, False, f"firmware returned error: {line}")

            for item in expected_list:
                if not seen[item.label] and item.predicate(line):
                    seen[item.label] = True

            if all(seen.values()) and (time.monotonic() - last_rx) >= 0.15:
                break
        else:
            if all(seen.values()) and (time.monotonic() - last_rx) >= 0.15:
                break
            time.sleep(0.02)

    missing = [label for label, found in seen.items() if not found]
    if missing:
        if lines:
            return StepResult(command, False, "missing expected response(s): " + ", ".join(missing))
        return StepResult(command, False, "no response")

    return StepResult(command, True)


def open_serial(args: argparse.Namespace) -> tuple[serial.Serial | None, StepResult]:
    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
        if args.dtr != "unchanged":
            ser.dtr = args.dtr == "on"
        if args.rts != "unchanged":
            ser.rts = args.rts == "on"
        time.sleep(0.2)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
    except serial.SerialException as exc:
        detail = str(exc)
        if "PermissionError" in detail or "Access is denied" in detail or "存取被拒" in detail:
            detail += " | Close RoboPlus Terminal, serial monitors, or any app using this COM port."
        return None, StepResult("open serial", False, detail)

    return ser, StepResult("open serial", True, f"{args.port} @ {args.baud}")


def run_startup_read(ser: serial.Serial, timeout_s: float) -> StepResult:
    print("STEP: startup read")
    read_available_lines(ser, duration_s=max(0.5, timeout_s))
    return StepResult("startup read", True, "startup output is optional")


def run_stop(ser: serial.Serial, timeout_s: float) -> StepResult:
    return send_and_expect(
        ser,
        "STOP",
        [line_equals("OK,STOP")],
        timeout_s=max(timeout_s, 1.0),
    )


def run_communication_tests(ser: serial.Serial, base_timeout_s: float) -> list[StepResult]:
    results: list[StepResult] = []

    results.append(
        send_and_expect(
            ser,
            "PING",
            [line_equals("PONG")],
            timeout_s=base_timeout_s,
        )
    )
    if not results[-1].passed:
        return results

    results.append(
        send_and_expect(
            ser,
            "HELP",
            [
                line_equals("PING"),
                line_contains("BEGIN help", "BEGIN,<traj_id>,<joint_count>,<point_count>"),
                line_contains("PT help", "PT,<seq>,<dt_ms>,<j1_pos>,<j2_pos>,<j3_pos>,<j4_pos>"),
                line_equals("HOME"),
            ],
            timeout_s=base_timeout_s,
        )
    )

    return results


def run_motion_tests(ser: serial.Serial, base_timeout_s: float) -> list[StepResult]:
    results: list[StepResult] = []

    results.append(
        send_and_expect(
            ser,
            "HOME",
            [line_equals("OK,HOME")],
            timeout_s=max(base_timeout_s, 4.0),
        )
    )
    if not results[-1].passed:
        return results

    results.append(
        send_and_expect(
            ser,
            "BEGIN,1,4,3",
            [line_equals("OK,BEGIN,1")],
            timeout_s=base_timeout_s,
        )
    )
    if not results[-1].passed:
        return results

    points = [
        ("PT,0,200,512,512,512,512", 200),
        ("PT,1,250,520,520,504,520", 250),
        ("PT,2,250,512,512,512,512", 250),
    ]
    for command, dt_ms in points:
        seq = command.split(",", maxsplit=2)[1]
        results.append(
            send_and_expect(
                ser,
                command,
                [line_equals(f"OK,PT,{seq}")],
                timeout_s=max(base_timeout_s, 2.5 + (dt_ms / 1000.0)),
            )
        )
        if not results[-1].passed:
            return results

    results.append(
        send_and_expect(
            ser,
            "END,1",
            [line_equals("END"), line_equals("OK,END,1")],
            timeout_s=max(base_timeout_s, 6.0),
        )
    )

    return results


def print_summary(results: list[StepResult]) -> int:
    passed = sum(1 for result in results if result.passed)
    total = len(results)

    print("Summary")
    print(f"Passed: {passed}/{total}")
    for result in results:
        status = "PASS" if result.passed else "FAIL"
        detail = f" - {result.detail}" if result.detail else ""
        print(f"{status}: {result.name}{detail}")

    return 0 if passed == total else 1


def pause_if_needed(args: argparse.Namespace) -> None:
    if args.no_pause:
        return
    try:
        input("Press Enter to exit...")
    except EOFError:
        pass


def run(args: argparse.Namespace) -> int:
    print_header(args)

    results: list[StepResult] = []
    ser, open_result = open_serial(args)
    results.append(open_result)
    print_result(open_result)

    if ser is None:
        return print_summary(results)

    motion_started = False
    stop_sent = False
    try:
        startup_result = run_startup_read(ser, args.timeout)
        results.append(startup_result)
        print_result(startup_result)

        for result in run_communication_tests(ser, args.timeout):
            results.append(result)
            print_result(result)
            if not result.passed:
                return print_summary(results)

        if not args.no_motion:
            motion_started = True
            for result in run_motion_tests(ser, args.timeout):
                results.append(result)
                print_result(result)
                if not result.passed:
                    break

        stop_result = run_stop(ser, args.timeout)
        stop_sent = True
        results.append(stop_result)
        print_result(stop_result)

        return print_summary(results)
    finally:
        if motion_started and not stop_sent and any(not result.passed for result in results):
            try:
                print("Cleanup: sending STOP after failed motion test")
                send_and_expect(ser, "STOP", [line_equals("OK,STOP")], timeout_s=1.0)
            except serial.SerialException:
                pass
        ser.close()


if __name__ == "__main__":
    args = parse_args()
    exit_code = 1
    try:
        exit_code = run(args)
    finally:
        pause_if_needed(args)
    sys.exit(exit_code)
