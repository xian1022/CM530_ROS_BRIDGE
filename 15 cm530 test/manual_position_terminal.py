#!/usr/bin/env python3
"""
CM-530 manual AX position terminal for the formal 15 cm530 test firmware.

This tool talks to the CM-530 over the formal ROS/CM530 ASCII protocol.
It is meant for PowerShell manual testing before the real ROS node is ready.
"""

from __future__ import annotations

import argparse
import re
import sys
import time
from typing import Iterable, List, Optional, Tuple

try:
    import serial
except ImportError as exc:  # pragma: no cover - user-facing startup error
    print("ERROR: pyserial is required. Install with: python -m pip install pyserial")
    raise SystemExit(2) from exc


JOINT_ORDER = "j1->ID17, j2->ID3, j3->ID2, j4->ID15"
LINE_ENDINGS = {
    "lf": b"\n",
    "cr": b"\r",
    "crlf": b"\r\n",
}
FORMAL_COMMANDS = ("PING", "AX", "HOME", "STOP", "BEGIN", "PT", "END")
LOCAL_COMMANDS = {"?", "HELP", "DEMO", "LISTEN", "RAW-P", "Q", "QUIT", "EXIT"}


def configure_console() -> None:
    """Keep Chinese prompts readable in modern PowerShell when possible."""
    for stream_name in ("stdout", "stderr"):
        stream = getattr(sys, stream_name)
        if hasattr(stream, "reconfigure"):
            try:
                stream.reconfigure(encoding="utf-8")
            except Exception:
                pass


def is_int_text(text: str) -> bool:
    return re.fullmatch(r"[+-]?\d+", text.strip()) is not None


def parse_int(text: str) -> Optional[int]:
    if not is_int_text(text):
        return None
    return int(text.strip(), 10)


def validate_positions(values: Iterable[int]) -> Optional[str]:
    for value in values:
        if value < 0 or value > 1023:
            return f"position {value} out of range 0..1023"
    return None


def normalize_user_input(text: str) -> Tuple[str, Optional[str], Optional[str]]:
    """
    Convert friendly manual input into a formal CM530 command.

    Returns (kind, command, error):
      kind == "local" means command is handled by this Python tool.
      kind == "send" means command should be sent to CM530.
    """
    raw = text.strip()
    if not raw:
        return "local", None, None

    upper = raw.upper()
    if upper in LOCAL_COMMANDS:
        return "local", upper, None

    if is_int_text(raw):
        value = int(raw, 10)
        err = validate_positions([value])
        if err:
            return "local", None, err
        return "send", f"AX,{value}", None

    friendly_four = re.fullmatch(
        r"\s*([+-]?\d+)[,\s]+([+-]?\d+)[,\s]+([+-]?\d+)[,\s]+([+-]?\d+)\s*",
        raw,
    )
    if friendly_four and not upper.startswith("PT"):
        values = [int(part, 10) for part in friendly_four.groups()]
        err = validate_positions(values)
        if err:
            return "local", None, err
        return "send", "AX," + ",".join(str(value) for value in values), None

    parts = [part.strip() for part in raw.replace("，", ",").split(",")]
    if not parts:
        return "local", None, None

    command = parts[0].upper()
    args = parts[1:]

    if command not in FORMAL_COMMANDS:
        return "send", raw.upper(), None

    if command in ("PING", "HOME", "STOP"):
        if args:
            return "local", None, f"{command} does not take arguments"
        return "send", command, None

    if command == "AX":
        if len(args) not in (1, 4):
            return "local", None, "AX format must be AX,<pos> or AX,<j1>,<j2>,<j3>,<j4>"
        values = [parse_int(arg) for arg in args]
        if any(value is None for value in values):
            return "local", None, "AX positions must be integers"
        err = validate_positions(value for value in values if value is not None)
        if err:
            return "local", None, err
        return "send", "AX," + ",".join(str(value) for value in values), None

    if command == "BEGIN":
        if len(args) != 3:
            return "local", None, "BEGIN format must be BEGIN,<traj_id>,4,<point_count>"
        values = [parse_int(arg) for arg in args]
        if any(value is None for value in values):
            return "local", None, "BEGIN arguments must be integers"
        if values[1] != 4 or values[2] <= 0:
            return "local", None, "BEGIN requires joint_count=4 and point_count>0"
        return "send", "BEGIN," + ",".join(str(value) for value in values), None

    if command == "PT":
        if len(args) != 6:
            return "local", None, "PT format must be PT,<seq>,<dt_ms>,<j1>,<j2>,<j3>,<j4>"
        values = [parse_int(arg) for arg in args]
        if any(value is None for value in values):
            return "local", None, "PT arguments must be integers"
        if values[1] < 0:
            return "local", None, "PT dt_ms must be >= 0"
        err = validate_positions(values[2:])
        if err:
            return "local", None, err
        return "send", "PT," + ",".join(str(value) for value in values), None

    if command == "END":
        if len(args) != 1 or parse_int(args[0]) is None:
            return "local", None, "END format must be END,<traj_id>"
        return "send", f"END,{int(args[0], 10)}", None

    return "send", raw, None


def open_serial(args: argparse.Namespace) -> serial.Serial:
    ser = serial.Serial()
    ser.port = args.port
    ser.baudrate = args.baud
    ser.bytesize = serial.EIGHTBITS
    ser.parity = serial.PARITY_NONE
    ser.stopbits = serial.STOPBITS_ONE
    ser.timeout = 0.03
    ser.write_timeout = args.timeout
    ser.dtr = False
    ser.rts = False
    ser.open()
    ser.dtr = False
    ser.rts = False
    return ser


def format_rx_line(data: bytes) -> str:
    return data.decode("utf-8", errors="replace").rstrip("\r\n")


def read_rx(ser: serial.Serial, duration: float) -> List[str]:
    deadline = time.monotonic() + duration
    buffer = bytearray()
    lines: List[str] = []

    while time.monotonic() < deadline:
        waiting = ser.in_waiting
        chunk = ser.read(waiting or 1)
        if not chunk:
            continue

        for byte in chunk:
            if byte in (10, 13):
                if buffer:
                    line = format_rx_line(bytes(buffer))
                    print(f"RX <- {line}")
                    lines.append(line)
                    buffer.clear()
            else:
                buffer.append(byte)

    if buffer:
        line = format_rx_line(bytes(buffer))
        print(f"RX <- {line}")
        lines.append(line)

    if not lines:
        print("RX <- (none)")
    return lines


def send_line(ser: serial.Serial, command: str, line_end: bytes, char_delay: float, timeout: float) -> List[str]:
    payload = command.encode("ascii") + line_end
    print(f"TX -> {command}")
    for byte in payload:
        ser.write(bytes([byte]))
        if char_delay > 0:
            time.sleep(char_delay)
    ser.flush()
    return read_rx(ser, timeout)


def send_raw_p(ser: serial.Serial, timeout: float) -> None:
    print("TX raw -> b'P'")
    ser.write(b"P")
    ser.flush()
    read_rx(ser, timeout)


def print_help() -> None:
    print("")
    print("可輸入的格式:")
    print("  512                       -> 送 AX,512")
    print("  520,512,512,512           -> 送 AX,520,512,512,512")
    print("  520 512 512 512           -> 送 AX,520,512,512,512")
    print("  AX,512")
    print("  AX,520,512,512,512")
    print("  PING | HOME | STOP")
    print("  BEGIN,1,4,3")
    print("  PT,0,300,512,512,512,512")
    print("  END,1")
    print("")
    print("本機指令:")
    print("  demo      自動送一段小幅正式軌跡")
    print("  listen    只監聽幾秒")
    print("  raw-p     送單一 byte P")
    print("  ?         顯示說明")
    print("  q         離開")
    print("")


def demo_commands() -> List[str]:
    return [
        "PING",
        "HOME",
        "BEGIN,1,4,3",
        "PT,0,300,512,512,512,512",
        "PT,1,300,520,512,512,512",
        "PT,2,300,512,512,512,512",
        "END,1",
        "STOP",
    ]


def run_self_test() -> int:
    cases = {
        "512": "AX,512",
        "520,512,512,512": "AX,520,512,512,512",
        "520 512 512 512": "AX,520,512,512,512",
        "ax,512": "AX,512",
        "AX，520，512，512，512": "AX,520,512,512,512",
        "PING": "PING",
        "home": "HOME",
        "BEGIN,1,4,3": "BEGIN,1,4,3",
        "PT,0,300,512,512,512,512": "PT,0,300,512,512,512,512",
        "END,1": "END,1",
    }
    for text, expected in cases.items():
        kind, command, error = normalize_user_input(text)
        if error or kind != "send" or command != expected:
            print(f"FAIL: {text!r} -> kind={kind!r}, command={command!r}, error={error!r}")
            return 1

    error_cases = ("AX,2000", "PT,0,300,512", "BEGIN,1,3,3")
    for text in error_cases:
        _kind, _command, error = normalize_user_input(text)
        if not error:
            print(f"FAIL: {text!r} should be rejected locally")
            return 1

    print("PASS: self-test")
    return 0


def run_terminal(args: argparse.Namespace) -> int:
    line_end = LINE_ENDINGS[args.line_end]

    print("CM-530 Manual AX Position Terminal")
    print(f"Port       : {args.port}")
    print(f"Baud       : {args.baud}")
    print(f"Line end   : {args.line_end}")
    print(f"Char delay : {args.char_delay}s")
    print("DTR/RTS    : off/off")
    print(f"Joint order: {JOINT_ORDER}")
    print("Examples   : 512 | 520,512,512,512 | AX,512 | HOME | STOP | demo")
    print("Exit       : q, quit, exit")
    print("")
    print("請先關閉 RoboPlus Terminal / 舊 Python 視窗，避免占用 COM4。")
    print("正式版韌體開機只會輸出 READY，手動輸入會由這個視窗顯示 TX/RX。")
    print("")

    try:
        ser = open_serial(args)
    except Exception as exc:
        print(f"FAIL: open serial - {exc}")
        print("請先關閉 RoboPlus Terminal 或其他占用 COM4 的程式。")
        return 1

    with ser:
        print(f"PASS: open serial - {args.port} @ {args.baud}")
        print("")
        print("Startup listen:")
        read_rx(ser, args.startup_listen)
        print("")
        print("先建議輸入 PING，再輸入 512 或四軸座標。輸入 ? 可看說明。")

        while True:
            try:
                text = input("AX> ")
            except (EOFError, KeyboardInterrupt):
                print("")
                return 0

            kind, command, error = normalize_user_input(text)
            if error:
                print(f"LOCAL ERR: {error}")
                continue
            if command is None:
                continue

            if kind == "local":
                if command in ("Q", "QUIT", "EXIT"):
                    return 0
                if command in ("?", "HELP"):
                    print_help()
                    continue
                if command == "LISTEN":
                    read_rx(ser, args.listen_seconds)
                    continue
                if command == "RAW-P":
                    send_raw_p(ser, args.timeout)
                    continue
                if command == "DEMO":
                    print("Running small formal demo: j1 512 -> 520 -> 512")
                    for demo_command in demo_commands():
                        lines = send_line(ser, demo_command, line_end, args.char_delay, args.timeout)
                        if not any(line.startswith(("PONG", "OK,")) for line in lines):
                            print("Demo stopped: expected PONG or OK response was not received.")
                            break
                    continue

            send_line(ser, command, line_end, args.char_delay, args.timeout)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Manual CM-530 AX position terminal")
    parser.add_argument("--port", default="COM4")
    parser.add_argument("--baud", type=int, default=57600)
    parser.add_argument("--timeout", type=float, default=1.5)
    parser.add_argument("--startup-listen", type=float, default=1.0)
    parser.add_argument("--listen-seconds", type=float, default=5.0)
    parser.add_argument("--line-end", choices=sorted(LINE_ENDINGS), default="lf")
    parser.add_argument("--char-delay", type=float, default=0.0)
    parser.add_argument("--self-test", action="store_true")
    return parser


def main() -> int:
    configure_console()
    args = build_arg_parser().parse_args()
    if args.self_test:
        return run_self_test()
    return run_terminal(args)


if __name__ == "__main__":
    raise SystemExit(main())
