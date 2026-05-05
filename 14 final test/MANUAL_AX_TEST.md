# CM-530 Final Test - Formal ROS Protocol Test

中文操作說明請看：`中文操作說明.md`

This folder now builds the ROS handoff firmware. It uses only the formal AX
position protocol:

```text
PING
BEGIN,<traj_id>,4,<point_count>
PT,<seq>,<dt_ms>,<j1_pos>,<j2_pos>,<j3_pos>,<j4_pos>
END,<traj_id>
STOP
HOME
```

The firmware is silent on startup and does not print echo, ALIVE, target dumps,
or actual-position dumps. ACK lines are kept compatible with the spec:

```text
PONG
OK,BEGIN,<id>
OK,PT,<seq>
OK,END,<id>
OK,STOP
OK,HOME
ERR,<code>
```

## Firmware To Flash

```text
C:\Users\39165\Downloads\CM530_ROS_BRIDGE\14 final test\CM530.hex
```

## PowerShell Smoke Test

Close RoboPlus Terminal first, then run:

```powershell
cd "C:\Users\39165\Downloads\CM530_ROS_BRIDGE\14 final test"
python .\cm530_bridge_smoke_test.py --port COM4 --baud 57600
```

The motion test sends a small formal trajectory:

```text
HOME
BEGIN,1,4,3
PT,0,300,512,512,512,512
PT,1,400,520,512,512,512
PT,2,400,512,512,512,512
END,1
STOP
```

Joint order:

```text
j1 -> ID17
j2 -> ID3
j3 -> ID2
j4 -> ID7
```
