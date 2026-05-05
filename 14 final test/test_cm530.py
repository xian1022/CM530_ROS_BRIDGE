print("STEP 1: script started")

import serial
import time

PORT = "COM4"
BAUD = 57600

print(f"STEP 2: opening {PORT} @ {BAUD}...")
ser = serial.Serial(PORT, BAUD, timeout=1)
print("STEP 3: COM opened successfully")

def read_all(wait_s=0.6):
    time.sleep(wait_s)
    got = False
    while ser.in_waiting:
        got = True
        line = ser.readline().decode(errors="ignore").strip()
        print("RX <-", line)
    if not got:
        print("RX <- (no response)")

def send(cmd, wait_s=0.6):
    print("TX ->", cmd)
    ser.write((cmd + "\n").encode())
    read_all(wait_s)

try:
    # 基本通訊
    send("PING")
    send("HELP", 1.0)

    # HOME 測試
    send("HOME", 1.5)

    # 單點軌跡測試
    send("BEGIN,1,4,1")
    send("PT,0,1000,0.0000,0.0500,0.1000,0.1500", 1.5)
    send("END,1", 2.5)

    # 三點小軌跡測試
    send("BEGIN,2,4,3")
    send("PT,0,300,0.0000,0.0000,0.0000,0.0000", 0.8)
    send("PT,1,300,0.0000,0.1000,0.1500,0.2000", 0.8)
    send("PT,2,300,0.0000,0.1500,0.2500,0.3000", 0.8)
    send("END,2", 2.5)

finally:
    print("STEP 4: closing serial")
    ser.close()
    print("STEP 5: done")
    input("Press Enter to exit...")