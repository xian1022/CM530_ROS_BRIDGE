# CM530_ROS_BRIDGE

CM-530 與 ROS 對接用韌體專案。  
目前主要正式韌體是：

```text
15 cm530 test
```

這版以 ROBOTIS 官方 Embedded C SDK 架構為基礎，CM530 只負責接收 ROS 端送來的 AX-12A position 整數、控制 AX-12A 馬達，並回傳 ACK / ERR。

---

## 目前主版本

```text
15 cm530 test/
```

重要檔案：

```text
15 cm530 test/APP/src/main.c
15 cm530 test/CM530.hex
15 cm530 test/CM530.bin
15 cm530 test/ROS_CM530_INTERFACE_SPEC.txt
```

用途：

- `main.c`：正式版 CM530 ROS bridge 韌體原始碼，已有中文註解，英文輔助。
- `CM530.hex`：燒錄到 CM-530 的正式版韌體。
- `ROS_CM530_INTERFACE_SPEC.txt`：給 ROS 端負責人看的 ROS-CM530 對接規格書。

舊資料夾說明：

- `12 CM530_BRIDGE_MODE`：早期 bridge mode 版本，保留作歷史參考。
- `14 final test`：前期除錯與 RX/TX 驗證版本，保留作 debug 參考。
- `15 cm530 test`：目前正式主版本。

---

## 系統角色

CM530 節點只負責 ROS-CM530 這段：

```text
ROS 主控端
    |
    | USB Serial, COM4 @ 57600
    v
CM-530
    |
    | Dynamixel TTL bus, 1 Mbps
    v
AX-12A motors
```

CM530 不負責：

- 不計算 IK。
- 不接收 rad / degree。
- 不做座標轉換。
- 不回讀 AX-12A present position。

ROS 端必須先把 IK 結果轉成 AX-12A position 整數，再送給 CM530。

---

## 通訊設定

PC / ROS 到 CM530：

```text
Port      : COM4
Baudrate  : 57600
Format    : 8N1
Encoding  : ASCII
Line end  : \n
```

CM530 也容忍 `\r\n`。

AX-12A：

```text
Baudrate  : 1 Mbps
Protocol  : Dynamixel Protocol 1.0
Write     : SYNC_WRITE Goal Position
```

Joint order：

```text
j1 -> ID17
j2 -> ID3
j3 -> ID2
j4 -> ID7
```

Position：

```text
AX position integer: 0..1023
HOME position      : 512
```

---

## 支援命令

```text
PING
AX,<pos>
AX,<j1_pos>,<j2_pos>,<j3_pos>,<j4_pos>
HOME
STOP
BEGIN,<traj_id>,4,<point_count>
PT,<seq>,<dt_ms>,<j1_pos>,<j2_pos>,<j3_pos>,<j4_pos>
END,<traj_id>
```

成功回覆：

```text
PONG
OK,AX
OK,HOME
OK,STOP
OK,BEGIN,<traj_id>
OK,PT,<seq>
OK,END,<traj_id>
```

錯誤回覆：

```text
ERR,BAD_CMD
ERR,BAD_ARG
ERR,RANGE
ERR,BAD_TRAJ
ERR,DXL_TX
ERR,DXL_TORQUE,<id>
ERR,OVERFLOW
```

更完整的協定請看：

```text
15 cm530 test/ROS_CM530_INTERFACE_SPEC.txt
```

---

## Build

在 PowerShell 執行：

```powershell
cd "C:\Users\39165\Downloads\CM530_ROS_BRIDGE\15 cm530 test"
make clean
make CM530.hex CM530.bin
```

輸出：

```text
CM530.hex
CM530.bin
```

備註：

- 官方 SDK 舊式 C code 會出現一些 warning，通常不影響 `CM530.hex` / `CM530.bin` 產生。
- 若 `make all` 在產生 `.lss` 時出問題，正式燒錄只需要 `CM530.hex` 或 `CM530.bin`。

---

## 燒錄

使用 RoboPlus Terminal：

1. 開啟 RoboPlus Terminal。
2. 連接 CM-530 的 `COM4 @ 57600`。
3. 燒錄：

```text
15 cm530 test/CM530.hex
```

4. 燒錄後 app 啟動時會輸出：

```text
READY
```

正式版韌體已關閉 command echo，所以在 RoboPlus Terminal 手動打字時，可能看不到自己輸入的內容。這是正常的，目的是避免 ROS 端收到自己送出的 echo。

---

## RoboPlus 手動測試

此測試只用來確認韌體與手臂基本可動。

輸入：

```text
PING
AX,512
AX,520,512,512,512
HOME
STOP
```

預期：

```text
PONG
OK,AX
OK,AX
OK,HOME
OK,STOP
```

正式軌跡測試：

```text
BEGIN,1,4,3
PT,0,300,512,512,512,512
PT,1,300,520,512,512,512
PT,2,300,512,512,512,512
END,1
```

預期：

```text
OK,BEGIN,1
OK,PT,0
OK,PT,1
OK,PT,2
OK,END,1
```

---

## ROS 串接操作

正式 ROS 測試時，不要開 RoboPlus Terminal。

COM4 使用規則：

```text
同一時間只能有一個程式開啟 COM4。
```

建議流程：

1. 先用 RoboPlus Terminal 燒錄 `15 cm530 test/CM530.hex`。
2. 關閉 RoboPlus Terminal。
3. 開 ROS serial node 或 Python 模擬器。
4. ROS 開啟 `COM4 @ 57600`。
5. ROS 可忽略或清掉開機的 `READY`。
6. ROS 每送一行 command，都等待一行 ACK / ERR。
7. 收到 `OK,...` 或 `PONG` 才送下一行。
8. 收到 `ERR,...` 就停止本次 motion 流程並印出錯誤。

ROS / Python terminal 建議印出：

```text
TX -> PT,1,300,520,512,512,512
RX <- OK,PT,1
```

目前 CM530 第一版不回傳馬達實際位置，ROS 顯示的是 target AX position。

---

## ROS 端最小實作要求

ROS serial node 至少需要：

- 開啟 `COM4 @ 57600, 8N1`。
- 每行 command 使用 `\n` 結尾。
- 以 `\n` 讀取 CM530 response。
- 每送一行都等待 response。
- 收到 `PONG` / `OK,...` 才繼續。
- 收到 `ERR,...` 要停止或進入錯誤處理。
- 不依賴 CM530 echo。
- 不要求 CM530 回傳 present position。

---

## 第一版限制

目前正式版不包含：

- AX-12A present position readback。
- 馬達實際角度回傳。
- `GETPOS` 指令。
- IK 計算。
- rad / degree 轉換。

若未來需要馬達實際位置，可新增第二版指令，例如：

```text
GETPOS
POS,<j1_pos>,<j2_pos>,<j3_pos>,<j4_pos>
```

但這需要另外處理 AX-12A status packet timeout / corrupt 的穩定性問題。

---

## Author

```text
xian1022
```
