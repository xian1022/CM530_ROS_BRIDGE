#!/usr/bin/env python3
"""Interactive ROS-protocol serial terminal for CM-530 formal firmware.

Use this from PowerShell to manually send the same ASCII lines that ROS will
send to the CM-530:

    PING
    BEGIN,1,4,3
    PT,0,300,512,512,512,512
    PT,1,400,520,512,512,512
    PT,2,400,512,512,512,512
    END,1
    STOP
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass

try:
    import serial
except ImportError as exc:
    print("FAIL: pyserial is not installed. Install with: python -m pip install pyserial")
    raise SystemExit(2) from exc


DEFAULT_PORT = "COM4"
DEFAULT_BAUD = 57600
JOINT_ORDER = "j1->ID17, j2->ID3, j3->ID2, j4->ID7"


@dataclass(frozen=True)
class DemoStep:
    command: str
    expected: str
    timeout_s: float


DEMO_STEPS = [
    DemoStep("PING", "PONG", 3.0),
    DemoStep("STOP", "OK,STOP", 3.0),
    DemoStep("HOME", "OK,HOME", 8.0),
    DemoStep("BEGIN,1,4,3", "OK,BEGIN,1", 3.0),
    DemoStep("PT,0,300,512,512,512,512", "OK,PT,0", 4.0),
    DemoStep("PT,1,400,520,512,512,512", "OK,PT,1", 4.0),
    DemoStep("PT,2,400,512,512,512,512", "OK,PT,2", 4.0),
    DemoStep("END,1", "OK,END,1", 8.0),
    DemoStep("STOP", "OK,STOP", 3.0),
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Manual ROS protocol terminal for CM-530.")
    parser.add_argument("--port", default=DEFAULT_PORT, help=f"Serial port, default: {DEFAULT_PORT}")
    parser.add_argument("--baud", default=DEFAULT_BAUD, type=int, help=f"Baud rate, default: {DEFAULT_BAUD}")
    parser.add_argument("--timeout", default=5.0, type=float, help="Read timeout after manual commands")
    parser.add_argument("--char-delay", default=0.05, type=float, help="Delay between TX bytes")
    parser.add_argument(
        "--line-end",
        choices=("lf", "cr", "crlf"),
        default="lf",
        help="Line ending sent after commands, default: lf",
    )
    parser.add_argument(
        "--dtr",
        choices=("off", "on", "unchanged"),
        default="off",
        help="DTR state before opening serial, default: off",
    )
    parser.add_argument(
        "--rts",
        choices=("off", "on", "unchanged"),
        default="off",
        help="RTS state before opening serial, default: off",
    )
    return parser.parse_args()


def line_ending(name: str) -> bytes:
    if name == "cr":
        return b"\r"
    if name == "crlf":
        return b"\r\n"
    return b"\n"


def open_serial(args: argparse.Namespace) -> serial.Serial:
    ser = serial.Serial()
    ser.port = args.port
    ser.baudrate = args.baud
    ser.timeout = 0.1
    ser.write_timeout = 2.0
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
    return ser


def print_header(args: argparse.Namespace) -> None:
    print("CM-530 ROS Protocol Manual Terminal")
    print(f"Port       : {args.port}")
    print(f"Baud       : {args.baud}")
    print(f"Line end   : {args.line_end}")
    print(f"Char delay : {args.char_delay:.2f}s")
    print(f"DTR/RTS    : {args.dtr}/{args.rts}")
    print(f"Joint order: {JOINT_ORDER}")
    print("Commands   : PING | STOP | HOME | BEGIN/PT/END")
    print("Local      : demo | listen | go | start | formal-live | reset-start | reset-listen | reopen | raw-p | tx-diag | rx-check | rx-live | sdk-check | sdk-live | probe-lines | reset-go | ? | q")
    print("Rule       : start CM-530 app first, then open COM; do not RESET while COM is open")
    print()


def send_bytes(ser: serial.Serial, payload: bytes, char_delay_s: float) -> None:
    for value in payload:
        written = ser.write(bytes([value]))
        if written != 1:
            print(f"WARN: serial.write returned {written} for byte 0x{value:02X}")
        ser.flush()
        if char_delay_s > 0:
            time.sleep(char_delay_s)


def send_line(ser: serial.Serial, command: str, ending: bytes, char_delay_s: float) -> None:
    print(f"TX -> {command}")
    send_bytes(ser, command.encode("ascii") + ending, char_delay_s)


def send_raw(ser: serial.Serial, payload: bytes, char_delay_s: float) -> None:
    print(f"TX raw -> {payload!r}")
    send_bytes(ser, payload, char_delay_s)


def read_lines(ser: serial.Serial, duration_s: float, quiet_s: float = 0.25) -> list[str]:
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


def looks_like_bootloader(lines: list[str]) -> bool:
    markers = (
        "Bad Command",
        "SYSTEM O.K.",
        "CM530 Boot loader",
        "Read Protection",
        "Write Protection",
    )
    return any(any(marker in line for marker in markers) or line == "-" for line in lines)


def send_and_wait(
    ser: serial.Serial,
    command: str,
    expected: str | None,
    timeout_s: float,
    ending: bytes,
    char_delay_s: float,
) -> bool:
    send_line(ser, command, ending, char_delay_s)
    lines = read_lines(ser, timeout_s)
    if looks_like_bootloader(lines):
        print("INFO: bootloader response detected; app is not running yet.")
    if expected is None:
        return True

    ok = any(line == expected for line in lines)
    if ok:
        print(f"PASS: {expected}")
    else:
        print(f"FAIL: expected {expected}")
    return ok


def ping_once(ser: serial.Serial, args: argparse.Namespace) -> bool:
    return send_and_wait(
        ser,
        "PING",
        "PONG",
        max(args.timeout, 3.0),
        line_ending(args.line_end),
        args.char_delay,
    )


def reopen_existing_serial(ser: serial.Serial, args: argparse.Namespace) -> None:
    print("Closing and reopening COM port. Do not reset CM-530.")
    ser.close()
    time.sleep(0.8)
    if args.dtr != "unchanged":
        ser.dtr = args.dtr == "on"
    if args.rts != "unchanged":
        ser.rts = args.rts == "on"
    ser.open()
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    time.sleep(0.5)
    print(f"PASS: reopened serial - {args.port} @ {args.baud}")


def ensure_app_running(ser: serial.Serial, args: argparse.Namespace) -> bool:
    print("Checking app with PING...")
    if ping_once(ser, args):
        return True

    print("Trying bootloader -GO variants, then PING again.")
    go_seen = try_go(ser, args)
    if go_seen:
        reopen_existing_serial(ser, args)
    else:
        print("INFO: no app PONG and no bootloader Go response was seen.")
        print("INFO: use RoboPlus to flash/start CM530.hex, close RoboPlus, reopen this terminal.")
    if ping_once(ser, args):
        return True

    return probe_ping(ser, args)


def reset_and_start(ser: serial.Serial, args: argparse.Namespace) -> bool:
    try:
        input("Press CM-530 RESET / power-cycle now, then press Enter...")
    except EOFError:
        pass

    print("After reset listen:")
    read_lines(ser, 5.0)

    print("Checking app with PING after reset...")
    if ping_once(ser, args):
        return True

    print("Trying bootloader GO after reset, then PING again.")
    if try_go(ser, args):
        reopen_existing_serial(ser, args)
    if ping_once(ser, args):
        return True

    return probe_ping(ser, args)


def run_demo(ser: serial.Serial, args: argparse.Namespace) -> None:
    print("Running formal ROS demo trajectory.")
    print("Small motion: j1 512 -> 520 -> 512.")
    ending = line_ending(args.line_end)
    if not ensure_app_running(ser, args):
        print("Demo stopped because app did not respond to PING.")
        return

    for step in DEMO_STEPS[1:]:
        ok = send_and_wait(ser, step.command, step.expected, step.timeout_s, ending, args.char_delay)
        print()
        if not ok:
            print("Demo stopped because expected ACK was not received.")
            return
    print("Demo completed.")


def print_help() -> None:
    print("Local commands:")
    print("  demo      Send PING, STOP, HOME, BEGIN/PT/END small trajectory, STOP.")
    print("  listen    Read serial for 8 seconds without TX.")
    print("  go        Send bootloader -GO variants.")
    print("  start     PING; if bootloader is detected, send GO, reopen COM, PING again.")
    print("  formal-live Test formal CM530.hex app without RESET/GO.")
    print("  reset-start Diagnostic only; reset while COM is open can break PC->CM530 TX.")
    print("  reset-listen Diagnostic only; reset while COM is open can break PC->CM530 TX.")
    print("  reopen    Close and reopen COM port without resetting CM-530.")
    print("  raw-p     Send one raw byte P without newline.")
    print("  tx-diag   Print pyserial modem lines and write result for PING.")
    print("  rx-check  Confirm READY first, then test PING/raw P/DTR/RTS.")
    print("  rx-live   Test CM530_rx_probe.hex without resetting CM-530 after opening COM.")
    print("  sdk-check Confirm READY first, then test the official-SDK-style serial probe.")
    print("  sdk-live  Test SDK serial probe without resetting CM-530 after opening COM.")
    print("  probe-ping Try PING with LF, CR, and CRLF.")
    print("  probe-lines Try PING with several DTR/RTS line states.")
    print("  reset-go  Ask you to reset/power-cycle, then send -GO variants.")
    print("  q         Quit.")
    print("Formal command examples:")
    print("  PING")
    print("  STOP")
    print("  HOME")
    print("  BEGIN,1,4,3")
    print("  PT,0,300,512,512,512,512")
    print("  PT,1,400,520,512,512,512")
    print("  PT,2,400,512,512,512,512")
    print("  END,1")


def try_go(ser: serial.Serial, args: argparse.Namespace) -> bool:
    variants = (
        b"-go\r",
        b"-go\n",
        b"-go\r\n",
        b"go\r",
        b"go\n",
        b"go\r\n",
        b"-GO\r",
        b"-GO\n",
        b"-GO\r\n",
        b"GO\r",
        b"GO\n",
        b"GO\r\n",
    )
    time.sleep(0.5)
    for payload in variants:
        send_raw(ser, payload, max(args.char_delay, 0.15))
        lines = read_lines(ser, 1.5)
        if any("Go:" in line for line in lines):
            print("INFO: bootloader reported Go; waiting for app startup.")
            time.sleep(2.0)
            if ser.in_waiting:
                read_lines(ser, 0.5)
            return True
    return False


def probe_ping(ser: serial.Serial, args: argparse.Namespace) -> bool:
    print("Probing PING with LF / CR / CRLF.")
    variants = (("lf", b"\n"), ("cr", b"\r"), ("crlf", b"\r\n"))
    for name, ending in variants:
        print(f"--- PING line ending: {name} ---")
        ok = send_and_wait(ser, "PING", "PONG", max(args.timeout, 3.0), ending, args.char_delay)
        if ok:
            print(f"PASS: PING works with {name}")
            return True
        print()
    return False


def set_line_state(ser: serial.Serial, dtr: bool | None, rts: bool | None) -> None:
    if dtr is not None:
        ser.dtr = dtr
    if rts is not None:
        ser.rts = rts
    time.sleep(0.2)


def probe_line_states(ser: serial.Serial, args: argparse.Namespace) -> None:
    print("Probing DTR/RTS states with PING.")
    print("Use this with CM530_rx_probe.hex; success should show [RXDBG] bytes or PONG.")
    ending = line_ending(args.line_end)
    states = (
        ("unchanged", None, None),
        ("dtr-off rts-off", False, False),
        ("dtr-on rts-off", True, False),
        ("dtr-off rts-on", False, True),
        ("dtr-on rts-on", True, True),
    )
    for name, dtr, rts in states:
        print(f"--- line state: {name} ---")
        set_line_state(ser, dtr, rts)
        send_line(ser, "PING", ending, args.char_delay)
        read_lines(ser, max(args.timeout, 3.0))
        print()


def tx_diag(ser: serial.Serial, args: argparse.Namespace) -> None:
    print("TX diagnostic from Python/pyserial side.")
    print(f"Port open    : {ser.is_open}")
    print(f"DTR/RTS      : {ser.dtr}/{ser.rts}")
    print(f"CTS/DSR/CD/RI: {ser.cts}/{ser.dsr}/{ser.cd}/{ser.ri}")
    print(f"out_waiting before: {getattr(ser, 'out_waiting', 'n/a')}")

    payload = b"PING\n"
    print(f"TX diag raw -> {payload!r}")
    try:
        written = ser.write(payload)
        ser.flush()
    except serial.SerialTimeoutException as exc:
        print(f"FAIL: write timeout: {exc}")
        return

    print(f"serial.write returned: {written}")
    print(f"out_waiting after : {getattr(ser, 'out_waiting', 'n/a')}")
    read_lines(ser, max(args.timeout, 3.0))


def has_ready_marker(lines: list[str]) -> bool:
    return any(("READY" in line) or ("EADY" in line) for line in lines)


def has_rx_probe_marker(lines: list[str]) -> bool:
    return any("RX_PROBE" in line for line in lines)


def has_sdk_serial_probe_marker(lines: list[str]) -> bool:
    return any("SDK_SERIAL_PROBE" in line for line in lines)


def rx_check(ser: serial.Serial, args: argparse.Namespace) -> None:
    print("RX path check for CM530_rx_probe.hex.")
    print("Step 1: reset/power-cycle CM-530 so the probe app prints READY,RX_PROBE.")
    try:
        input("Press CM-530 RESET / power-cycle now, then press Enter...")
    except EOFError:
        pass

    print("Listening for READY:")
    ready_lines = read_lines(ser, 8.0)
    if has_rx_probe_marker(ready_lines):
        print("PASS: READY,RX_PROBE seen; correct RX probe firmware is running.")
    elif has_ready_marker(ready_lines):
        print("FAIL: READY/EADY seen, but not READY,RX_PROBE.")
        print("You may have flashed CM530_ready_probe.hex instead of CM530_rx_probe.hex.")
        return
    else:
        print("FAIL: READY,RX_PROBE not seen; app state is not confirmed. Reflash CM530_rx_probe.hex or reset again.")
        return

    print()
    print("Step 2: send full PING. RX probe should print each byte as [RXDBG].")
    send_line(ser, "PING", line_ending(args.line_end), args.char_delay)
    read_lines(ser, max(args.timeout, 3.0))

    print()
    print("Step 3: send raw single byte P without newline.")
    send_raw(ser, b"P", args.char_delay)
    read_lines(ser, max(args.timeout, 3.0))

    print()
    print("Step 4: try DTR/RTS line states.")
    probe_line_states(ser, args)


def rx_live_check(ser: serial.Serial, args: argparse.Namespace) -> None:
    print("RX live check for CM530_rx_probe.hex.")
    print("Do not press RESET. This assumes RoboPlus already booted the probe app with Go.")
    print("Step 1: short listen.")
    read_lines(ser, 1.0)

    print()
    print("Step 2: send full PING. Expected: [RXDBG] bytes and PONG.")
    send_line(ser, "PING", line_ending(args.line_end), args.char_delay)
    read_lines(ser, max(args.timeout, 5.0), quiet_s=1.0)

    print()
    print("Step 3: send raw single byte P without newline. Expected: [RXDBG] only.")
    send_raw(ser, b"P", args.char_delay)
    read_lines(ser, max(args.timeout, 5.0), quiet_s=1.0)


def sdk_check(ser: serial.Serial, args: argparse.Namespace) -> None:
    print("SDK serial check for CM530_sdk_serial_probe.hex.")
    print("This probe uses only the official SDK USART3 RX interrupt path.")
    print("Step 1: reset/power-cycle CM-530 so the probe app prints READY,SDK_SERIAL_PROBE.")
    try:
        input("Press CM-530 RESET / power-cycle now, then press Enter...")
    except EOFError:
        pass

    print("Listening for READY:")
    ready_lines = read_lines(ser, 8.0)
    if has_sdk_serial_probe_marker(ready_lines):
        print("PASS: READY,SDK_SERIAL_PROBE seen; correct SDK serial probe is running.")
    elif has_ready_marker(ready_lines):
        print("FAIL: READY/EADY seen, but not READY,SDK_SERIAL_PROBE.")
        print("You may have flashed a different probe hex.")
        return
    else:
        print("FAIL: READY,SDK_SERIAL_PROBE not seen; app state is not confirmed.")
        return

    print()
    print("Step 2: send full PING. Expected: [RXDBG] bytes and PONG after P.")
    send_line(ser, "PING", line_ending(args.line_end), args.char_delay)
    read_lines(ser, max(args.timeout, 3.0))

    print()
    print("Step 3: send raw single byte P without newline. Expected: [RXDBG] and PONG.")
    send_raw(ser, b"P", args.char_delay)
    read_lines(ser, max(args.timeout, 3.0))

    print()
    print("Step 4: show Python-side TX diagnostics.")
    tx_diag(ser, args)

    print()
    print("Step 5: try DTR/RTS line states against the SDK serial probe.")
    probe_line_states(ser, args)


def sdk_live_check(ser: serial.Serial, args: argparse.Namespace) -> None:
    print("SDK live check for CM530_sdk_serial_probe.hex.")
    print("Do not press RESET during this check. The app should already be running.")
    print("Step 1: short listen.")
    read_lines(ser, 1.0)

    print()
    print("Step 2: send raw single byte P without newline.")
    send_raw(ser, b"P", args.char_delay)
    read_lines(ser, max(args.timeout, 3.0))

    print()
    print("Step 3: send full PING.")
    send_line(ser, "PING", line_ending(args.line_end), args.char_delay)
    read_lines(ser, max(args.timeout, 3.0))


def formal_live_check(ser: serial.Serial, args: argparse.Namespace) -> bool:
    print("Formal live check for CM530.hex.")
    print("Do not press RESET. This assumes RoboPlus already booted the app with Go.")
    variants = (("lf", b"\n"), ("cr", b"\r"), ("crlf", b"\r\n"))
    for name, ending in variants:
        print(f"--- formal PING line ending: {name} ---")
        if send_and_wait(ser, "PING", "PONG", max(args.timeout, 3.0), ending, args.char_delay):
            print("PASS: formal firmware is responding.")
            return True
        print()
    print("FAIL: formal firmware did not respond.")
    print("Close this terminal, start CM530.hex from RoboPlus until you see Go:, close RoboPlus, then reopen Python.")
    return False


def interactive_loop(ser: serial.Serial, args: argparse.Namespace) -> int:
    ending = line_ending(args.line_end)
    print("Startup listen; formal firmware may stay silent:")
    read_lines(ser, 1.0)
    print()
    print("Type formal ROS protocol lines. Start with PING or type demo.")

    while True:
        try:
            typed = input("ROS> ").strip()
        except EOFError:
            print()
            return 0

        if not typed:
            continue

        lower = typed.lower()
        if lower in ("q", "quit", "exit"):
            return 0
        if lower == "?":
            print_help()
            continue
        if lower == "listen":
            read_lines(ser, 8.0)
            continue
        if lower == "go":
            try_go(ser, args)
            continue
        if lower == "start":
            ensure_app_running(ser, args)
            continue
        if lower == "formal-live":
            formal_live_check(ser, args)
            continue
        if lower == "reset-start":
            reset_and_start(ser, args)
            continue
        if lower == "reset-listen":
            try:
                input("Press CM-530 RESET / power-cycle now, then press Enter...")
            except EOFError:
                pass
            print("After reset listen-only:")
            read_lines(ser, 12.0)
            continue
        if lower == "reopen":
            reopen_existing_serial(ser, args)
            continue
        if lower == "raw-p":
            send_raw(ser, b"P", args.char_delay)
            read_lines(ser, max(args.timeout, 3.0))
            continue
        if lower == "probe-ping":
            probe_ping(ser, args)
            continue
        if lower == "probe-lines":
            probe_line_states(ser, args)
            continue
        if lower == "tx-diag":
            tx_diag(ser, args)
            continue
        if lower == "rx-check":
            rx_check(ser, args)
            continue
        if lower == "rx-live":
            rx_live_check(ser, args)
            continue
        if lower == "sdk-check":
            sdk_check(ser, args)
            continue
        if lower == "sdk-live":
            sdk_live_check(ser, args)
            continue
        if lower == "reset-go":
            try:
                input("Press CM-530 RESET / power-cycle now, then press Enter...")
            except EOFError:
                pass
            print("Before GO listen:")
            read_lines(ser, 3.0)
            try_go(ser, args)
            continue
        if lower == "demo":
            run_demo(ser, args)
            continue

        command = typed.upper() if "," not in typed else typed.upper()
        send_and_wait(ser, command, None, args.timeout, ending, args.char_delay)

    return 0


def main() -> int:
    args = parse_args()
    print_header(args)
    try:
        ser = open_serial(args)
    except serial.SerialException as exc:
        print(f"FAIL: open serial - {exc}")
        print("Close RoboPlus Terminal and other windows using COM4.")
        return 1

    try:
        print(f"PASS: open serial - {args.port} @ {args.baud}")
        print()
        return interactive_loop(ser, args)
    finally:
        ser.close()


if __name__ == "__main__":
    sys.exit(main())
