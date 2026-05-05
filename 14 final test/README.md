# CM-530 ROS 對接目前狀態與問題整理

更新日期：2026-05-05  
資料夾：`14 final test`

這份 README 是給老師討論用的整理版。內容會比較口語，重點是說清楚目前做到哪裡、已經排除什麼、還卡在哪裡，以及下一步怎麼查。

## 1. 目前目標

這個資料夾的目標是做一版可以交給 ROS 端使用的 CM-530 韌體。

ROS 端負責算 IK，然後只把 AX-12A position 整數丟給 CM-530。CM-530 不算 IK，只照收到的 AX position 控制手臂。

正式協定固定如下：

```text
Port      : COM4
Baud      : 57600
Data bits : 8
Parity    : none
Stop bits : 1
Flow ctrl : none
Line end  : LF
```

關節順序固定：

```text
j1 -> ID17
j2 -> ID3
j3 -> ID2
j4 -> ID7
```

正式命令只保留這些：

```text
PING
BEGIN,<traj_id>,4,<point_count>
PT,<seq>,<dt_ms>,<j1_pos>,<j2_pos>,<j3_pos>,<j4_pos>
END,<traj_id>
STOP
HOME
```

正式回覆格式：

```text
PONG
OK,BEGIN,<traj_id>
OK,PT,<seq>
OK,END,<traj_id>
OK,STOP
OK,HOME
ERR,<code>
```

## 2. 目前已經驗證的事情

### CM-530 傳資料回 PC 是正常的

已經看過 CM-530 可以透過 COM4 傳資料回 RoboPlus / PowerShell。

例如燒入 `CM530_rx_probe.hex` 或 `CM530_sdk_serial_probe.hex` 後，可以看到：

```text
READY,RX_PROBE
READY,SDK_SERIAL_PROBE
[RXDBG] ...
PONG
ALIVE
```

所以 CM-530 -> PC 這條 TX 路徑是通的。

### Python 不是完全不能送資料

一開始看起來像是 Python 送不進 CM-530，但後來用 `sdk-live` 和 `.NET SerialPort reopen matrix` 測試後，已經證明 Python / .NET 在某些條件下可以成功把 `P` 或 `PING` 送到 CM-530。

成功例子：

```text
TX raw -> b'P'
RX <- [RXDBG] 0x50 'P'
RX <- PONG
```

以及：

```text
TX -> PING
RX <- [RXDBG] 0x50 'P'
RX <- [RXDBG] 0x49 'I'
RX <- [RXDBG] 0x4E 'N'
RX <- [RXDBG] 0x47 'G'
RX <- [RXDBG] 0x0A <LF>
RX <- [LINE]PING
```

所以目前不能說「Python 透過 COM4 不可行」。比較準確的說法是：**開機順序、reset 時機、COM port 重新開啟狀態，會影響 PC -> CM-530 方向能不能成功。**

### RoboPlus Terminal 可以送單字元進 CM-530

RoboPlus Terminal 測試 `CM530_sdk_serial_probe.hex` 時，手動輸入 `P` 可以看到：

```text
[RXDBG] 0x50 'P'
PONG
```

代表韌體的 RX 中斷路徑本身可以收到資料。

### AX-12A 手臂有實際動過

手動測試時，曾經用單鍵或 AX 指令讓手臂動了一下，表示 DXL bus 和馬達控制不是完全失效。

另外分別測過：

```text
ID17 + ID3
ID17 + ID2
ID17 + ID7
```

兩顆一組時都有 OK。ID3、ID2、ID7 單測也 OK。ID17 曾經出現過 status packet 錯誤，但搭配其他馬達時有成功過，因此還需要再確認 ID17 的回包穩定性、接線、電源、ID 設定和 bus 負載。

## 3. 目前主要問題

### 問題 A：正式版 `CM530.hex` 對 Python `PING` 還沒有穩定回 `PONG`

目前正式測試：

```text
python .\ros_protocol_terminal.py --port COM4 --baud 57600
ROS> formal-live
```

結果仍是：

```text
TX -> PING
RX <- (none)
FAIL: expected PONG
```

這代表正式版還不能直接宣告交付給 ROS 端使用。至少要先讓正式版穩定通過：

```text
PING -> PONG
```

才可以繼續跑小幅動作 `demo`。

### 問題 B：reset / power-cycle 時如果 COM4 已經被 Python 開著，PC -> CM-530 可能失效

目前觀察到：

- Python 開著 COM4 時按 RESET，CM-530 有時會傳回 `READY`，但 Python 再送 `PING` 進去時，韌體不一定收到。
- 如果先讓 CM-530 app 跑起來，再重新開 Python COM4，反而比較容易成功。

所以目前暫定操作規則是：

```text
1. 用 RoboPlus 燒錄並啟動 app，看到 Go: 0800....
2. 關閉 RoboPlus / RoboPlus Terminal，釋放 COM4。
3. 再開 Python / ROS。
4. Python / ROS 開著時不要按 RESET 或重新上電。
5. 如果按了 RESET，就先關掉 Python / ROS，再重新開 COM4。
```

這一點未來 ROS 端也要知道，不然 ROS node 可能以為 serial port 還活著，但實際 PC -> CM-530 方向已經不收資料。

### 問題 C：bootloader `GO` 由 Python 送出不穩

RoboPlus 可以正常燒錄並送 `-GO` / `GO` 讓 app 起來。

Python 有時可以看到 bootloader 回：

```text
Go: 08006xxx
```

但接著再送 `PING` 不一定會收到 `PONG`。所以目前不建議把「Python 自動叫 bootloader GO」當正式流程。

比較安全的流程還是：

```text
RoboPlus 啟動 app -> 關 RoboPlus -> Python/ROS 開 COM4
```

### 問題 D：正式版和 RX probe 的行為還沒完全對齊

`CM530_rx_probe.hex` 已經能看到 Python 送出的完整 `PING\n`：

```text
[RXDBG] 0x50 'P'
[RXDBG] 0x49 'I'
[RXDBG] 0x4E 'N'
[RXDBG] 0x47 'G'
[RXDBG] 0x0A <LF>
[LINE]PING
```

但正式版 `CM530.hex` 目前 `formal-live` 還沒有穩定回 `PONG`。

這表示下一步要確認：

- 正式版 `CM530.hex` 是否真的有燒到最新建置。
- 正式版和 `CM530_rx_probe.hex` 是否走同一條 PC UART RX / parser 路徑。
- 正式版是否在進入主迴圈前卡住。
- `PING` handler 是否真的有被呼叫。

## 4. 目前比較可能的原因

### 比較不像的原因

目前比較不像是這些：

- 不是單純 pyserial 壞掉，因為 Python 已經成功讓 RX probe 收到完整 `PING\n`。
- 不是 CM-530 完全不能 TX，因為 READY / RXDBG / ALIVE 都收得到。
- 不是 DXL 馬達完全不能動，因為手臂曾經動過，部分 ID 測試 OK。

### 比較像的原因

比較可能是下面幾類：

1. **COM4 開啟時機問題**  
   RESET / power-cycle 和 Windows serial port 狀態互相影響，導致 PC -> CM-530 的方向失效。

2. **正式版 image / debug image 沒有完全一致**  
   RX probe 能收到完整命令，但正式版沒有回 PONG，需要確認正式 `CM530.hex` 是不是最新 build，並確認 parser 沒有被條件編譯關掉。

3. **正式版進 main loop 前或 parser 前卡住**  
   之前 BOOT0 / BOOT1 / BOOT2 / BOOT3 測試顯示 app 可以進到後面，但正式靜默版沒有 marker，所以需要再用 probe 版定位。

4. **DXL bus 會干擾 motion 命令，但理論上不該影響 PING**  
   已經把 DXL 初始化改成 lazy init，`PING` 不應該依賴 AX-12A 狀態。若 `PING` 都失敗，優先查 PC UART / parser，不先查 IK 或軌跡。

## 5. 下一步建議

### 第一步：先用 RX probe 確認 parser 完整跑完

燒錄：

```text
C:\Users\39165\Downloads\CM530_ROS_BRIDGE\14 final test\CM530_rx_probe.hex
```

RoboPlus 看到 app 啟動後，應該要看到新版 marker：

```text
READY,RX_PROBE,V3_DISPATCH
```

關閉 RoboPlus，再開 PowerShell：

```powershell
cd "C:\Users\39165\Downloads\CM530_ROS_BRIDGE\14 final test"
python .\ros_protocol_terminal.py --port COM4 --baud 57600
```

輸入：

```text
rx-live
```

理想輸出應該包含：

```text
[RXDBG] 0x50 'P'
[RXDBG] 0x49 'I'
[RXDBG] 0x4E 'N'
[RXDBG] 0x47 'G'
[RXDBG] 0x0A <LF>
[LINE]PING
[LOOP]
[DISPATCH]PING
[PING_MATCH]
PONG
```

如果只到 `[LINE]PING`，代表 RX 收到了，但主迴圈或 command dispatch 還沒走完。

### 第二步：RX probe 通過後，再測正式版

燒錄：

```text
C:\Users\39165\Downloads\CM530_ROS_BRIDGE\14 final test\CM530.hex
```

RoboPlus 看到：

```text
Go: 0800....
```

之後關閉 RoboPlus，開 Python：

```powershell
python .\ros_protocol_terminal.py --port COM4 --baud 57600
```

輸入：

```text
formal-live
```

正式版至少要通過：

```text
PING -> PONG
```

### 第三步：正式版 PONG 通過後，再跑小幅動作

輸入：

```text
demo
```

demo 會送小幅軌跡：

```text
PING
STOP
HOME
BEGIN,1,4,3
PT,0,300,512,512,512,512
PT,1,400,520,512,512,512
PT,2,400,512,512,512,512
END,1
STOP
```

這個動作只讓 j1 從 512 到 520，再回 512，是目前比較安全的小幅測試。

## 6. 目前資料夾裡重要檔案

```text
APP/src/main.c
```

韌體主程式。正式版、RX probe、SDK serial probe 都由這份 source 透過不同 build flag 產生。

```text
CM530.hex
```

正式 ROS 交付版。理論上開機靜默，只回正式 ACK / ERR。

```text
CM530_rx_probe.hex
```

RX / parser 除錯版。用來確認 Python 送出的每個 byte 是否真的進到 CM-530，並確認 parser 是否有 dispatch 到 `PING`。

```text
CM530_sdk_serial_probe.hex
```

更底層的 SDK serial probe。只測官方 SDK USART3 RX interrupt 路徑，排除上層 parser 干擾。

```text
ros_protocol_terminal.py
```

PowerShell / Python 手動測試工具。可以輸入 `formal-live`、`rx-live`、`sdk-live`、`demo` 等本地測試命令。

```text
ROS_HANDOFF.md
```

給 ROS 端的人看的交接規格，比這份 README 更偏正式操作規範。

```text
CM530_rx_probe_main.txt
CM530_rx_probe_ros_protocol_terminal.py
```

提供給老師或其他人對照用的 RX probe 版本 source / Python 工具備份。

## 7. 跟老師討論時可以直接講的重點

目前不是卡在 IK，也不是 ROS command 格式本身。真正卡點比較像是：

```text
Windows COM4 / RoboPlus / Python 開啟順序
+ CM-530 bootloader / app 啟動狀態
+ 正式版和 debug probe 的 parser 行為是否完全一致
```

目前最有價值的證據是：

```text
RX probe 已經證明 Python 可以把 PING\n 完整送到 CM-530。
```

所以接下來要把問題縮小到：

```text
為什麼 RX probe 收得到 PING，但正式版 CM530.hex 還沒有穩定 PONG？
```

只要正式版穩定做到：

```text
PING -> PONG
```

後面 ROS 端送 `BEGIN / PT / END` 的架構就比較明確了。

