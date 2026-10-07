#!/usr/bin/env python3
"""CM530 v17 manual terminal. arm1/arm2 is mandatory on every motion command."""
from __future__ import annotations

import argparse
import re
import sys
import time
from pathlib import Path
from typing import Optional, Tuple

JOINT_ORDER = {"arm1": (17, 3, 2, 15), "arm2": (12, 1, 8, 16)}
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
            if len(values) != 4:
                raise ValueError("numeric shortcut requires four positions")
            if arm not in JOINT_ORDER:
                raise ValueError("numeric shortcuts require --arm arm1 or --arm arm2")
            raw = "AX," + arm + "," + ",".join(values)
        parts = [part.strip() for part in raw.split(",")]
        parts[0] = parts[0].upper()
        if len(parts) > 1:
            parts[1] = parts[1].lower()
        command = parts[0]
        if command in {"PING", "VERSION"}:
            if len(parts) != 1:
                raise ValueError(command + " takes no arguments")
            return "send", command, None
        if command not in {"AX", "TORQUE", "LED", "HOME", "GET_HOME", "READ", "HOLD"}:
            raise ValueError("unknown command; enter ? for help")
        if len(parts) < 2 or parts[1] not in JOINT_ORDER:
            raise ValueError("command must explicitly specify arm1 or arm2")
        counts = {"AX": (6,), "TORQUE": (3,), "LED": (3,), "HOME": (2,), "GET_HOME": (2,), "READ": (2,), "HOLD": (2,)}
        if len(parts) not in counts[command]:
            raise ValueError("wrong argument count; enter ? for help")
        if command == "LED":
            parts[2] = parts[2].upper()
            if parts[2] not in {"MOVING", "STOPPED"}:
                raise ValueError("LED requires MOVING or STOPPED")
            return "send", ",".join(parts), None
        values = [parse_int(part) for part in parts[2:]]
        if command == "TORQUE" and values[0] not in (0, 1):
            raise ValueError("TORQUE requires 0 or 1")
        positions = values if command == "AX" else []
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
    if parts[0] == "VERSION":
        return "VERSION,5"
    if parts[0] in {"READ", "GET_HOME"}:
        return ("POS" if parts[0] == "READ" else "HOME") + "," + parts[1] + ",<j1>,<j2>,<j3>,<j4>"
    reply = "OK," + ",".join(parts[:2])
    if parts[0] in {"TORQUE", "LED"}:
        reply += "," + parts[2]
    return reply


def reply_matches(command: str, line: str) -> bool:
    parts = command.split(",")
    if parts[0] not in {"READ", "GET_HOME"}:
        return line == expected_reply(command)
    fields = line.split(",")
    tag = "POS" if parts[0] == "READ" else "HOME"
    if len(fields) != 6 or fields[:2] != [tag, parts[1]]:
        return False
    # The board emits canonical unsigned decimal integers, never stale ACKs.
    return all(re.fullmatch(r"[0-9]{1,4}", value) and 0 <= int(value) <= 1023
               for value in fields[2:])


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
            if line is not None:
                print("RX <- " + line)
            if line is None or line == "READY,5":
                break
            raise ProtocolError("unexpected startup response: " + line)
        # READY may have been emitted before the serial port was opened.
        self.send("VERSION")

    def reject_pending_input(self):
        # A previous ACK can leave its LF behind, but any other queued byte
        # belongs to an unsolicited event or stale response, not a new request.
        if self.buffer:
            raise ProtocolError("partial unsolicited response before request")
        drained = 0
        while self.serial.in_waiting:
            data = self.serial.read(min(self.serial.in_waiting, 257))
            drained += len(data)
            if not data or drained > 256:
                raise ProtocolError("unexpected pending input before request")
            if data.strip(b"\r\n"):
                raise ProtocolError("unsolicited response before request: " +
                                    data.decode("ascii", errors="replace").strip())

    def send(self, command):
        expected = expected_reply(command)
        self.reject_pending_input()
        quiet = command.startswith("LED,")
        if not quiet:
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
            if not quiet or line != expected:
                print("RX <- " + line)
            if line == "READY" or line.startswith("READY,"):
                # During a request READY means the controller restarted.
                raise ProtocolError("controller restarted during request")
            if not reply_matches(command, line):
                raise ProtocolError("expected " + expected + "; received " + line)
            return line


def demo_commands(arm=None):
    selected = (arm,) if arm else ("arm1", "arm2")
    commands = ["PING"]
    for target in selected:
        commands.extend(["AX," + target + ",512,512,512,512", "TORQUE," + target + ",1"])
    for j1 in (512, 520, 512):
        for target in selected:
            commands.append("AX,{},{},512,512,512".format(target, j1))
    return commands


def run_demo(connection, arm=None):
    for command in demo_commands(arm):
        connection.send(command)  # Any mismatch/error/timeout aborts the sequence.
        if command.startswith("AX,"):
            time.sleep(0.3)  # Host pacing only; not evidence of physical arrival.
        elif command.startswith("TORQUE,") and command.endswith(",1"):
            time.sleep(1.0)


def print_help():
    print("PING | VERSION | AX,arm1,512,512,512,512 | AX,arm2,520,512,512,512")
    print("TORQUE,arm1,0 | TORQUE,arm1,1 (requires a successful AX/HOME/HOLD target)")
    print("LED,arm1,MOVING | LED,arm1,STOPPED (normal LED TX/ACK output is hidden)")
    print("LED displays host-reported state only; STOPPED does not stop motors; demo does not change LEDs.")
    print("Four-number shortcuts require --arm arm1/arm2. Single-number shortcuts are unsupported.")
    print("demo: direct goals on --arm, or alternating arm1/arm2 when no --arm; q: quit")
    print("Protocol 5: motor ACK means packet sent; LED ACK means GPIO updated. Neither proves motion state.")
    print("HOME,arm1 | GET_HOME,arm1 | READ,arm1 | HOLD,arm1 (also arm2)")
    print("HOME writes the board preset; HOLD reads current positions then writes goals; neither enables torque.")
    print("READ returns measured positions; ROS decides arrival. BEGIN/PT/END/STOP remain unsupported.")
    print("demo uses example positions 512/520, explicitly enables torque and leaves it enabled.")
    print("Check that these goals suit the mechanism before running demo; 512 is not a calibrated home.")


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
    parser.add_argument("--arm", type=str.lower, choices=("arm1", "arm2"))
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
    print("CM530 v17 / protocol 5 | arm1: {} | arm2: {}".format(JOINT_ORDER["arm1"], JOINT_ORDER["arm2"]))
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
                kind, command, error = normalize_user_input(input("arm1/arm2> "), args.arm)
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
