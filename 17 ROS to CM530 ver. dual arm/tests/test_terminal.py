import contextlib
import io
import sys
import unittest
from types import SimpleNamespace
from pathlib import Path
from unittest.mock import patch
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import manual_position_terminal as terminal

class FakeSerial:
    def __init__(self, response=b"", scripted=None, queued=False):
        self.rx = bytearray(response if queued else b"")
        self.response = b"" if queued else response
        self.tx, self.scripted = [], scripted
    @property
    def in_waiting(self):
        return len(self.rx)
    def write(self, data):
        self.tx.append(data)
        self.rx.extend(self.response)
        self.response = b""
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

    def test_v17_commands_and_data(self):
        for cmd in ("HOME", "HOLD", "READ", "GET_HOME"):
            self.assertEqual(terminal.normalize_user_input(cmd.lower()+",ARM2"), ("send",cmd+",arm2",None))
            for invalid in (cmd,cmd+",ALL",cmd+",arm1,0"):
                self.assertIsNotNone(terminal.normalize_user_input(invalid)[2])
        for cmd,tag in (("READ","POS"),("GET_HOME","HOME")):
            response=tag+",arm2,0,511,512,1023"
            self.assertEqual(terminal.Connection(FakeSerial((response+"\r\n").encode())).send(cmd+",arm2"),response)
            for bad in (tag+",arm1,0,511,512,1023", tag+",arm2,0,511,512",tag+",arm2,0,511,512,1024",
                        tag+",arm2,0,511,512,-1",tag+",arm2,0,511,512,51.2",tag+",arm2,0,511,512,512,0",
                        "OK,"+cmd+",arm2","READY,5","ERR,DXL_TIMEOUT,arm2,12",tag+",arm2,0,511,512,512junk"):
                serial=FakeSerial((bad+"\n").encode())
                with self.assertRaises(terminal.ProtocolError): terminal.Connection(serial).send(cmd+",arm2")
                self.assertEqual(len(serial.tx),1)
        for cmd in ("HOME", "HOLD"):
            response="OK,"+cmd+",arm1"
            self.assertEqual(terminal.Connection(FakeSerial((response+"\n").encode())).send(cmd+",arm1"),response)
        with self.assertRaises(terminal.ProtocolError):
            terminal.Connection(FakeSerial(b"READY,4\n",queued=True)).startup(.01)

    def test_explicit_commands(self):
        for raw, expected in (
            ("ax,ArM2,520,512,512,512", "AX,arm2,520,512,512,512"),
            ("AX，arm1，0，511，512，1023", "AX,arm1,0,511,512,1023"),
            ("torque,ARM2,1", "TORQUE,arm2,1"), ("version", "VERSION"), ("ping", "PING")):
            self.assertEqual(terminal.normalize_user_input(raw), ("send", expected, None))
    def test_four_value_shortcuts_require_arm(self):
        for text in ("520,512,512,512", "520 512 512 512"):
            self.assertIsNotNone(terminal.normalize_user_input(text)[2])
            self.assertEqual(terminal.normalize_user_input(text, "arm2")[1], "AX,arm2,520,512,512,512")
        self.assertIsNotNone(terminal.normalize_user_input("512", "arm2")[2])
        self.assertEqual(terminal.normalize_user_input("AX,arm1,512,512,512,512", "arm2")[1], "AX,arm1,512,512,512,512")
    def test_removed_commands(self):
        for text in ("BEGIN,arm1,1,4,1",
                     "PT,arm1,0,300,512,512,512,512", "END,arm1,1", "STOP,arm1", "AX,arm1,512"):
            with self.subTest(text=text):
                self.assertIsNotNone(terminal.normalize_user_input(text)[2])
    def test_invalid_input(self):
        for text in ("AX,512", "AX", "TORQUE", "PING,arm1", "TORQUE,arm1,2",
                     "TORQUE,arm2,-1", "VERSION,arm1", "HELLO", "AX,arm1,512,512,512,512,extra"):
            self.assertIsNotNone(terminal.normalize_user_input(text)[2])
        for value in ("1024", "-1", "51.2", "", "2147483648", "-2147483649", "4294967808", "512\x00"):
            self.assertIsNotNone(terminal.normalize_user_input("AX,arm1," + value + ",512,512,512")[2])
        for arm in ("A", "B", "arm", "arm0", "arm3", "arm1extra"):
            self.assertIsNotNone(terminal.normalize_user_input("AX," + arm + ",512,512,512,512")[2])
    def test_expected_ack(self):
        for command, reply in (("AX,arm2,0,511,512,1023", "OK,AX,arm2"),
                               ("TORQUE,arm1,0", "OK,TORQUE,arm1,0"),
                               ("VERSION", "VERSION,5"), ("PING", "PONG")):
            self.assertEqual(terminal.expected_reply(command), reply)
    def test_fragmented_response_and_crlf(self):
        serial = FakeSerial(b"\r\nOK,AX,arm2\r\n")
        self.assertEqual(terminal.Connection(serial).send("AX,arm2,512,512,512,512"), "OK,AX,arm2")
        self.assertEqual(serial.tx, [b"AX,arm2,512,512,512,512\n"])
    def test_mismatched_error_and_restart(self):
        for response in (b"OK,AX,arm1\n", b"OK,TORQUE,arm2,1\n", b"ERR,RANGE,arm2\n",
                         b"READY\n", b"OK,AX,arm2junk\n", b"READY,5\n"):
            with self.assertRaises(terminal.ProtocolError):
                terminal.Connection(FakeSerial(response)).send("AX,arm2,512,512,512,512")
    def test_startup_requires_version_even_if_ready_was_missed(self):
        for initial in (b"", b"READY,5\r\n"):
            serial = FakeSerial(initial, scripted=lambda _: b"VERSION,5\n", queued=True)
            terminal.Connection(serial).startup(0.001)
            self.assertEqual(serial.tx, [b"VERSION\n"])
    def test_startup_rejects_old_or_failed_firmware(self):
        for initial in (b"READY\n", b"READY,2\n", b"READY,3\n", b"ERR,INIT_FAILED\n"):
            serial = FakeSerial(initial, queued=True)
            with self.assertRaises(terminal.ProtocolError):
                terminal.Connection(serial).startup(0.01)
            self.assertEqual(serial.tx, [])
        for reply in (b"VERSION,2\n", b"VERSION,3\n", b"ERR,BAD_CMD\n"):
            serial = FakeSerial(scripted=lambda _: reply)
            with self.assertRaises(terminal.ProtocolError):
                terminal.Connection(serial).startup(0)
            self.assertEqual(serial.tx, [b"VERSION\n"])
    def test_torque_ack_checks_value(self):
        with self.assertRaises(terminal.ProtocolError):
            terminal.Connection(FakeSerial(b"OK,TORQUE,arm1,0\n")).send("TORQUE,arm1,1")
    def test_demo_explicit_enable_and_direct_goals(self):
        commands = terminal.demo_commands("arm1")
        self.assertEqual(commands[:3], ["PING", "AX,arm1,512,512,512,512", "TORQUE,arm1,1"])
        self.assertEqual(commands[-1], "AX,arm1,512,512,512,512")
        for command in commands:
            self.assertIn(command.split(",")[0], ("PING", "AX", "TORQUE"))
            self.assertEqual(terminal.normalize_user_input(command)[0], "send")
    def test_partial_line_timeout_is_not_ack(self):
        with self.assertRaises(terminal.ProtocolError):
            terminal.Connection(FakeSerial(b"OK,AX,arm2"), timeout=0.01).send("AX,arm2,512,512,512,512")
    def test_demo_stops_at_failure(self):
        for failure in (b"ERR,DXL_TX,arm1\n", b"OK,AX,arm2\n", b"READY,5\n", b""):
            def response(data):
                return b"PONG\n" if data == b"PING\n" else failure
            serial = FakeSerial(scripted=response)
            with self.assertRaises(terminal.ProtocolError):
                terminal.run_demo(terminal.Connection(serial, timeout=0.01))
            self.assertEqual(serial.tx, [b"PING\n", b"AX,arm1,512,512,512,512\n"])
    def test_demo_restart_or_error_during_pacing_sends_nothing_more(self):
        for event in (b"READY,5\n", b"ERR,INIT_FAILED\n", b"OK,AX,arm1\n", b"READ"):
            def respond(data):
                return b"PONG\n" if data == b"PING\n" else b"OK,AX,arm1\n"
            serial = FakeSerial(scripted=respond)
            with patch.object(terminal.time, "sleep", side_effect=lambda _: serial.rx.extend(event)):
                with self.assertRaises(terminal.ProtocolError):
                    terminal.run_demo(terminal.Connection(serial), "arm1")
            self.assertEqual(serial.tx, [b"PING\n", b"AX,arm1,512,512,512,512\n"])
    def test_duplicate_ack_cannot_satisfy_next_request(self):
        serial = FakeSerial(b"OK,AX,arm1\r\nOK,AX,arm1\r\n")
        connection = terminal.Connection(serial)
        connection.send("AX,arm1,512,512,512,512")
        with self.assertRaises(terminal.ProtocolError):
            connection.send("AX,arm1,520,512,512,512")
        self.assertEqual(serial.tx, [b"AX,arm1,512,512,512,512\n"])
    def test_idle_restart_rejects_first_motion(self):
        serial = FakeSerial(b"READY,5\r\n", queued=True)
        with self.assertRaises(terminal.ProtocolError):
            terminal.Connection(serial).send("AX,arm1,512,512,512,512")
        self.assertEqual(serial.tx, [])
    def test_crlf_tail_does_not_block_next_request(self):
        serial = FakeSerial(scripted=lambda _: b"OK,AX,arm1\r\n")
        connection = terminal.Connection(serial)
        connection.send("AX,arm1,512,512,512,512")
        connection.send("AX,arm1,520,512,512,512")
        self.assertEqual(len(serial.tx), 2)

    def test_led_input_and_ack(self):
        self.assertEqual(terminal.normalize_user_input(" led , ARM2 , moving "), ("send", "LED,arm2,MOVING", None))
        self.assertEqual(terminal.normalize_user_input("LED,arm1,stopped"), ("send", "LED,arm1,STOPPED", None))
        self.assertEqual(terminal.expected_reply("LED,arm2,MOVING"), "OK,LED,arm2,MOVING")
        for text in ("LED", "LED,arm3,MOVING", "LED,arm1,0", "LED,arm1,STOP", "LED,arm2,", "LED,arm2,MOVING,extra"):
            self.assertIsNotNone(terminal.normalize_user_input(text)[2])
    def test_led_success_is_silent_but_sent_and_checked(self):
        for arm in ("arm1", "arm2"):
            for state in ("MOVING", "STOPPED"):
                reply = "OK,LED," + arm + "," + state
                serial = FakeSerial((reply + "\r\n").encode())
                captured = io.StringIO()
                with contextlib.redirect_stdout(captured):
                    self.assertEqual(terminal.Connection(serial).send("LED," + arm + "," + state), reply)
                self.assertEqual(captured.getvalue(), "")
                self.assertEqual(serial.tx, [("LED," + arm + "," + state + "\n").encode()])
    def test_led_error_reply_is_visible(self):
        for reply in (b"ERR,BAD_ARG,arm1\n", b"OK,LED,arm2,MOVING\n", b"OK,LED,arm1,STOPPED\n", b"READY,5\n"):
            serial = FakeSerial(reply); captured = io.StringIO()
            with contextlib.redirect_stdout(captured):
                with self.assertRaises(terminal.ProtocolError):
                    terminal.Connection(serial).send("LED,arm1,MOVING")
            self.assertIn(reply.decode().strip(), captured.getvalue())
            self.assertNotIn("TX -> LED", captured.getvalue())
            self.assertEqual(serial.tx, [b"LED,arm1,MOVING\n"])
    def test_normal_commands_and_startup_remain_visible(self):
        captured = io.StringIO()
        serial = FakeSerial(b"READY,5\r\n", scripted=lambda _: b"VERSION,5\r\n", queued=True)
        with contextlib.redirect_stdout(captured):
            terminal.Connection(serial).startup(0.01)
            terminal.Connection(FakeSerial(b"PONG\n")).send("PING")
        self.assertIn("RX <- READY,5", captured.getvalue())
        self.assertIn("TX -> VERSION", captured.getvalue())
        self.assertIn("RX <- VERSION,5", captured.getvalue())
        self.assertIn("TX -> PING", captured.getvalue())
        self.assertIn("RX <- PONG", captured.getvalue())

    def test_led_failures_stop_cli_and_show_error_without_later_commands(self):
        class CLISerial(FakeSerial):
            def open(self):
                pass
            def __enter__(self):
                return self
            def __exit__(self, *args):
                pass
        for failure in (b"ERR,BAD_ARG,arm1\n", b"OK,LED,arm2,MOVING\n", b"READY,5\n", b"", b"OK,LED,arm1,MOV"):
            def respond(data):
                if data==b"VERSION\n": return b"VERSION,5\n"
                if data==b"PING\n": return b"PONG\n"
                return failure
            serial = CLISerial(scripted=respond)
            module = SimpleNamespace(Serial=lambda: serial, EIGHTBITS=8, PARITY_NONE='N', STOPBITS_ONE=1, SerialException=OSError)
            captured = io.StringIO()
            with patch.dict(sys.modules, serial=module), patch.object(sys, 'argv', ['terminal', '--startup-listen', '0', '--timeout', '0.001']), patch('builtins.input', side_effect=['LED,arm1,MOVING', 'AX,arm1,512,512,512,512']) as prompt, contextlib.redirect_stdout(captured):
                self.assertEqual(terminal.main(), 1)
            self.assertEqual(prompt.call_count, 1)
            self.assertEqual(serial.tx, [b"VERSION\n", b"PING\n", b"LED,arm1,MOVING\n"])
            self.assertIn("Session stopped:", captured.getvalue())
            self.assertNotIn("TX -> LED", captured.getvalue())
            self.assertNotIn("RX <- OK,LED,arm1,MOVING", captured.getvalue())
    def test_led_pending_rx_is_not_consumed_as_new_ack(self):
        for pending in (b"READY,5\n", b"ERR,INIT_FAILED\n", b"OK,LED,arm1,MOVING\n", b"REA"):
            serial = FakeSerial(pending, queued=True)
            with self.assertRaises(terminal.ProtocolError) as error:
                terminal.Connection(serial).send("LED,arm1,MOVING")
            self.assertIn("unsolicited", str(error.exception))
            self.assertEqual(serial.tx, [])

    def test_demo_alternates_and_checks_all_responses(self):
        def respond(data):
            command = data.decode().strip().split(",")
            if command[0] == "PING":
                return b"PONG\n"
            if command[0] == "AX":
                return ("OK,AX," + command[1] + "\n").encode()
            return ("OK,TORQUE," + command[1] + ",1\n").encode()
        serial = FakeSerial(scripted=respond)
        with patch.object(terminal.time, "sleep") as sleep:
            terminal.run_demo(terminal.Connection(serial))
        points = [data.decode().split(",")[1] for data in serial.tx if data.startswith(b"AX,")]
        self.assertEqual(points, ["arm1", "arm2"] * 4)
        self.assertEqual(sleep.call_count, 10)

if __name__ == "__main__":
    unittest.main()
