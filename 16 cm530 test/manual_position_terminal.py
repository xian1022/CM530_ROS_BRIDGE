#!/usr/bin/env python3
"""CM530 v16 manual terminal. A/B is mandatory on every motion command."""
from __future__ import annotations

import argparse
import re
import sys
import time
from pathlib import Path
from typing import Optional, Tuple

JOINT_ORDER = {"A": (17, 3, 2, 15), "B": (12, 1, 8, 16)}
LINE_ENDINGS = {"lf": b"\n", "cr": b"\r", "crlf": b"\r\n"}
LOCAL_COMMANDS = {"?", "HELP", "DEMO", "Q", "QUIT", "EXIT"}


def parse_int(text: str) -> int:
    if not re.fullmatch(r"[+-]?[0-9]+", text):
        raise ValueError("arguments must be decimal integers")
    value = int(text, 10)
    if not -(2**31) <= value <= 2**31 - 1:
        raise ValueError("integer exceeds signed 32-bit range")
    return value


def normalize_user_input(text: str, arm: Optional[str] = None) -> Tuple[str, Optional[str], Optional[str]]:
    raw = text.strip().lstrip("\ufeff").replace("，", ",")
    if not raw:
        return "local", None, None
    if raw.upper() in LOCAL_COMMANDS:
        return "local", raw.upper(), None
    try:
        if re.fullmatch(r"[+\-0-9,\s]+", raw):
            values = re.split(r"[,\s]+", raw)
            if len(values) not in (1, 4):
                raise ValueError("numeric shortcut requires one or four positions")
            if arm not in JOINT_ORDER:
                raise ValueError("numeric shortcuts require --arm A or --arm B")
            raw = "AX," + arm + "," + ",".join(values)
        parts = [part.strip().upper() for part in raw.split(",")]
        command = parts[0]
        if command == "PING":
            if len(parts) != 1:
                raise ValueError("PING takes no arguments")
            return "send", "PING", None
        if command not in {"AX", "HOME", "STOP", "BEGIN", "PT", "END"}:
            raise ValueError("unknown command; enter ? for help")
        if len(parts) < 2 or parts[1] not in JOINT_ORDER:
            raise ValueError("command must explicitly specify A or B")
        counts = {"AX": (3, 6), "HOME": (2,), "STOP": (2,), "BEGIN": (5,), "PT": (8,), "END": (3,)}
        if len(parts) not in counts[command]:
            raise ValueError("wrong argument count; enter ? for help")
        values = [parse_int(part) for part in parts[2:]]
        if command == "BEGIN" and (values[1] != 4 or values[2] <= 0):
            raise ValueError("BEGIN requires joint_count=4 and point_count>0")
        if command == "PT" and values[1] < 0:
            raise ValueError("PT dt_ms must be nonnegative")
        positions = values if command == "AX" else values[2:] if command == "PT" else []
        if any(value < 0 or value > 1023 for value in positions):
            raise ValueError("position must be in 0..1023")
        normalized = ",".join(parts[:2] + [str(value) for value in values])
        if len(normalized.encode("ascii")) >= 96:
            raise ValueError("command exceeds firmware line buffer")
        return "send", normalized, None
    except ValueError as exc:
        return "local", None, str(exc)


def expected_reply(command: str) -> str:
    parts = command.split(",")
    if parts[0] == "PING":
        return "PONG"
    reply = "OK," + ",".join(parts[:2])
    if parts[0] in {"BEGIN", "PT", "END"}:
        reply += "," + parts[2]
    return reply


class ProtocolError(RuntimeError):
    pass


class Connection:
    """Keep partial RX lines across reads; accept only the exact expected ACK."""
    def __init__(self, serial_port, timeout=2.0, line_end=b"\n", char_delay=0.0):
        self.serial = serial_port
        self.timeout = timeout
        self.line_end = line_end
        self.char_delay = char_delay
        self.buffer = bytearray()

    def read_line(self, deadline):
        while time.monotonic() < deadline:
            data = self.serial.read(1)
            if not data:
                continue
            if data in (b"\r", b"\n"):
                if self.buffer:
                    line = self.buffer.decode("ascii", errors="replace")
                    self.buffer.clear()
                    print("RX <- " + line)
                    return line
            else:
                self.buffer.extend(data)
                if len(self.buffer) > 256:
                    raise ProtocolError("response line is too long")
        return None

    def startup(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            line = self.read_line(deadline)
            if line is None or line == "READY":
                return
            raise ProtocolError("unexpected startup response: " + line)

    def send(self, command):
        expected = expected_reply(command)
        print("TX -> " + command)
        payload = command.encode("ascii") + self.line_end
        if self.char_delay:
            for byte in payload:
                self.serial.write(bytes([byte]))
                time.sleep(self.char_delay)
        else:
            self.serial.write(payload)
        self.serial.flush()
        deadline = time.monotonic() + self.timeout
        while True:
            line = self.read_line(deadline)
            if line is None:
                raise ProtocolError("timeout waiting for " + expected)
            if line == "READY":
                # During a request READY means the controller restarted.
                raise ProtocolError("controller restarted during request")
            if line != expected:
                raise ProtocolError("expected " + expected + "; received " + line)
            return line


def demo_commands(arm=None):
    selected = (arm,) if arm else ("A", "B")
    commands = ["PING"]
    for target in selected:
        commands.extend(["HOME," + target, "BEGIN," + target + ",1,4,3"])
    for seq, j1 in enumerate((512, 520, 512)):
        for target in selected:
            commands.append("PT,{},{},300,{},512,512,512".format(target, seq, j1))
    for target in selected:
        commands.extend(["END," + target + ",1", "STOP," + target])
    return commands


def run_demo(connection, arm=None):
    for command in demo_commands(arm):
        connection.send(command)  # Any mismatch/error/timeout aborts the sequence.
        if command.startswith("PT,"):
            time.sleep(0.3)  # Host pacing only; not evidence of physical arrival.
        elif command.startswith("HOME,"):
            time.sleep(1.0)


def print_help():
    print("PING | AX,A,512 | AX,B,520,512,512,512 | HOME,A | STOP,B")
    print("BEGIN,B,1,4,3 | PT,B,0,300,512,512,512,512 | END,B,1")
    print("Numeric shortcuts require --arm A/B. Explicit commands always name the arm.")
    print("demo: small trajectory on --arm, or alternating A/B when no --arm; q: quit")
    print("ACK means target sent, NOT arrival. STOP re-sends the last target.")


def run_self_test():
    import unittest
    suite = unittest.defaultTestLoader.discover(str(Path(__file__).parent / "tests"), pattern="test_terminal.py")
    return 0 if unittest.TextTestRunner(verbosity=2).run(suite).wasSuccessful() else 1


def main():
    for name in ("stdout", "stderr"):
        stream = getattr(sys, name)
        if hasattr(stream, "reconfigure"):
            stream.reconfigure(encoding="utf-8")
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="COM4")
    parser.add_argument("--baud", type=int, default=57600)
    parser.add_argument("--arm", choices=("A", "B"))
    parser.add_argument("--timeout", type=float, default=2.0)
    parser.add_argument("--startup-listen", type=float, default=6.0)
    parser.add_argument("--line-end", choices=sorted(LINE_ENDINGS), default="lf")
    parser.add_argument("--char-delay", type=float, default=0.0)
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        return run_self_test()
    if args.timeout <= 0 or args.startup_listen < 0 or args.char_delay < 0:
        parser.error("timeout must be positive; startup-listen/char-delay must be nonnegative")
    try:
        import serial
    except ImportError:
        print("Install serial support: python -m pip install pyserial")
        return 2
    print("CM530 v16 | A: {} | B: {}".format(JOINT_ORDER["A"], JOINT_ORDER["B"]))
    print("Close RoboPlus and other programs using {}.".format(args.port))
    print_help()
    try:
        ser = serial.Serial()
        ser.port, ser.baudrate = args.port, args.baud
        ser.bytesize, ser.parity, ser.stopbits = serial.EIGHTBITS, serial.PARITY_NONE, serial.STOPBITS_ONE
        ser.timeout, ser.write_timeout = 0.03, args.timeout
        ser.dtr = ser.rts = False
        ser.open()
        with ser:
            connection = Connection(ser, args.timeout, LINE_ENDINGS[args.line_end], args.char_delay)
            connection.startup(args.startup_listen)
            connection.send("PING")
            while True:
                kind, command, error = normalize_user_input(input("A/B> "), args.arm)
                if error:
                    print("LOCAL ERR: " + error)
                    continue
                if command is None:
                    continue
                if kind == "local":
                    if command in {"Q", "QUIT", "EXIT"}:
                        return 0
                    if command in {"?", "HELP"}:
                        print_help()
                    elif command == "DEMO":
                        run_demo(connection, args.arm)
                else:
                    connection.send(command)
    except (EOFError, KeyboardInterrupt):
        return 0
    except (ProtocolError, serial.SerialException, OSError) as exc:
        print("Session stopped: " + str(exc))
        print("No further motion commands sent. Inspect the controller before reconnecting.")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
