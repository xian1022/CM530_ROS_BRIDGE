# CM-530 ROS Handoff Notes

## Required Startup Order

Use this order for ROS / Python control:

1. Flash `CM530.hex` with RoboPlus.
2. Let RoboPlus boot the app. The bootloader line should show `Go: 0800....`.
3. Close RoboPlus / RoboPlus Terminal so COM4 is released.
4. Open ROS / Python serial on `COM4 @ 57600`.
5. Do not press RESET or power-cycle CM-530 while ROS / Python still has COM4 open.

Important finding:

- CM-530 -> PC works after reset while COM4 is open.
- PC -> CM-530 may stop working if RESET / power-cycle happens while COM4 is already open.
- If CM-530 was reset, close and reopen the ROS / Python serial port before sending commands again.
- The formal firmware initializes the DXL UART lazily on the first motion command. `PING` does not depend on AX-12A bus state.

## Formal Serial Settings

```text
Port      : COM4
Baud      : 57600
Data bits : 8
Parity    : none
Stop bits : 1
Flow ctrl : none
Line end  : LF
```

Joint order:

```text
j1 -> ID17
j2 -> ID3
j3 -> ID2
j4 -> ID7
```

Positions are integer AX units only, expected range `0..1023`.

## Formal Protocol

ROS sends ASCII lines:

```text
PING
BEGIN,<traj_id>,4,<point_count>
PT,<seq>,<dt_ms>,<j1_pos>,<j2_pos>,<j3_pos>,<j4_pos>
END,<traj_id>
STOP
HOME
```

Expected firmware responses:

```text
PONG
OK,BEGIN,<traj_id>
OK,PT,<seq>
OK,END,<traj_id>
OK,STOP
OK,HOME
ERR,<code>
```

## Python Manual Test

After flashing `CM530.hex`, close RoboPlus and run:

```powershell
cd "C:\Users\39165\Downloads\CM530_ROS_BRIDGE\14 final test"
python .\ros_protocol_terminal.py --port COM4 --baud 57600
```

Then use:

```text
start
demo
```

Do not use `reset-start` for the normal ROS handoff test. It is diagnostic only.

## Debug Result Summary

The `CM530_sdk_serial_probe.hex` test confirmed:

- RoboPlus can send `P` and CM-530 replies `[RXDBG] 0x50 'P'` / `PONG`.
- Python can also send `P` successfully when CM-530 app is already running and COM4 is freshly opened.
- Python / .NET may fail to send after CM-530 is reset while COM4 remains open.

Therefore the ROS-side rule is: close/reopen the serial port after any CM-530 reset.

Optional validation image:

- `CM530_formal_ready_probe.hex` is the same strict protocol path as formal firmware, but prints `READY` after boot.
- Use it only to confirm the app reaches the command loop.
- The deliverable firmware remains `CM530.hex`.
