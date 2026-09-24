import contextlib
import io
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import manual_position_terminal as terminal


class FakeSerial:
    def __init__(self, response=b"", scripted=None):
        self.rx = bytearray(response)
        self.tx = []
        self.scripted = scripted

    def write(self, data):
        self.tx.append(data)
        if self.scripted:
            self.rx.extend(self.scripted(data))
        return len(data)

    def flush(self):
        pass

    def read(self, count):
        data = bytes(self.rx[:count])
        del self.rx[:count]
        return data


class TerminalTests(unittest.TestCase):
    def setUp(self):
        self.redirect = contextlib.redirect_stdout(io.StringIO())
        self.redirect.__enter__()

    def tearDown(self):
        self.redirect.__exit__(None, None, None)

    def test_explicit_commands(self):
        for raw, expected in (
            ("ax,b,520,512,512,512", "AX,B,520,512,512,512"),
            ("AX，A，512", "AX,A,512"),
            ("home,a", "HOME,A"), ("stop,b", "STOP,B"),
            ("BEGIN,B,7,4,2", "BEGIN,B,7,4,2"),
            ("PT,B,0,300,0,511,512,1023", "PT,B,0,300,0,511,512,1023"),
            ("END,B,7", "END,B,7"), ("ping", "PING"),
        ):
            self.assertEqual(terminal.normalize_user_input(raw), ("send", expected, None))

    def test_shortcuts_require_arm(self):
        for text in ("512", "520,512,512,512", "520 512 512 512"):
            self.assertIsNotNone(terminal.normalize_user_input(text)[2])
            self.assertTrue(terminal.normalize_user_input(text, "B")[1].startswith("AX,B,"))
        self.assertEqual(terminal.normalize_user_input("AX,A,512", "B")[1], "AX,A,512")

    def test_invalid_input(self):
        for text in ("AX,512", "HOME", "STOP", "BEGIN,1,4,2", "PT,0,300,512,512,512,512",
                     "END,1", "AX,C,512", "AX,A,1024", "AX,B,-1", "AX,A,51.2", "AX,A,",
                     "AX,A,2147483648", "AX,A,-2147483649", "AX,A,4294967808", "PING,A",
                     "BEGIN,A,1,8,1", "BEGIN,A,1,4,0", "PT,B,0,-1,512,512,512,512",
                     "PT,B,0,0,512,512,512,512,extra", "AX,A,512\x00", "HELLO"):
            with self.subTest(text=text):
                self.assertIsNotNone(terminal.normalize_user_input(text)[2])

    def test_expected_ack(self):
        self.assertEqual(terminal.expected_reply("PT,B,2,300,512,512,512,512"), "OK,PT,B,2")
        self.assertEqual(terminal.expected_reply("BEGIN,A,9,4,2"), "OK,BEGIN,A,9")
        self.assertEqual(terminal.expected_reply("END,B,9"), "OK,END,B,9")

    def test_fragmented_response_and_crlf(self):
        serial = FakeSerial(b"\r\nOK,AX,B\r\n")
        connection = terminal.Connection(serial)
        self.assertEqual(connection.send("AX,B,512"), "OK,AX,B")
        self.assertEqual(serial.tx, [b"AX,B,512\n"])

    def test_wrong_arm_sequence_error_and_restart(self):
        for response in (b"OK,PT,A,0\n", b"OK,PT,B,1\n", b"ERR,RANGE,B\n", b"READY\n",
                         b"OK,PT,B,0junk\n"):
            with self.subTest(response=response):
                connection = terminal.Connection(FakeSerial(response))
                with self.assertRaises(terminal.ProtocolError):
                    connection.send("PT,B,0,300,512,512,512,512")

    def test_partial_line_timeout_is_not_ack(self):
        connection = terminal.Connection(FakeSerial(b"OK,AX,B"), timeout=0.01)
        with self.assertRaises(terminal.ProtocolError):
            connection.send("AX,B,512")

    def test_demo_stops_at_failure(self):
        sent = []
        def response(data):
            command = data.decode().strip()
            sent.append(command)
            return ("ERR,DXL_TX,A\n" if command == "HOME,A" else terminal.expected_reply(command) + "\n").encode()
        connection = terminal.Connection(FakeSerial(scripted=response))
        with self.assertRaises(terminal.ProtocolError):
            terminal.run_demo(connection)
        self.assertEqual(sent, ["PING", "HOME,A"])

    def test_demo_alternates_and_checks_all_responses(self):
        serial = FakeSerial(scripted=lambda data: (terminal.expected_reply(data.decode().strip()) + "\n").encode())
        with patch.object(terminal.time, "sleep"):
            terminal.run_demo(terminal.Connection(serial))
        sent = [data.decode().strip() for data in serial.tx]
        points = [cmd.split(",")[1] for cmd in sent if cmd.startswith("PT,")]
        self.assertEqual(points, ["A", "B", "A", "B", "A", "B"])
        self.assertEqual(sent, terminal.demo_commands())


if __name__ == "__main__":
    unittest.main()
