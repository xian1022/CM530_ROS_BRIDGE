# CM530 第 16 版：A／B 雙手臂

本版以第 15 版為基礎，兩隻手臂共用一塊 CM-530。ROS 每筆命令指定 A 或 B，CM530 只寫入指定手臂的四軸目標。

| 由下到上 | A 手臂 | B 手臂 |
|---|---:|---:|
| j1 | 17 | 12 |
| j2 | 3 | 1 |
| j3 | 2 | 8 |
| j4 | 15 | 16 |

Position 範圍 0..1023；HOME 四軸皆 512。PC：57600 / 8N1；DXL：1 Mbps / Protocol 1.0。八顆 ID 應已在硬體設定完成，本韌體不改寫馬達 ID。

## 使用方式

1. 依下方步驟建置並燒錄本資料夾新產生的 `CM530.hex`。
2. 關閉 RoboPlus 與其他占用序列埠的程式。
3. 安裝 Python serial 套件後啟動終端：

```powershell
python -m pip install -r requirements.txt
python manual_position_terminal.py --port COM4
```

每筆命令明確指定 A／B：

```text
PING
AX,A,520,512,512,512
AX,B,520,512,512,512
HOME,A
HOME,B
STOP,A
STOP,B
```

`--arm B` 僅供數字快捷輸入，例如 `512` 或 `520 512 512 512`；完整命令仍須指定代號，且以命令內代號為準。未提供 `--arm` 時，數字快捷輸入會被拒絕。

終端的 `demo` 會執行 HOME 及 j1 的 512→520→512 小幅軌跡；有 `--arm` 時測單臂，沒有時交替測 A／B。請在 HOME=512 姿態及活動空間確認後使用。收到 ERR、逾時、錯誤手臂／序號 ACK 時，程式終止該次連線，不再送後續動作。`q` 離開不會自動送 HOME 或 STOP。

## ROS 對接

```text
TX -> BEGIN,A,10,4,1
RX <- OK,BEGIN,A,10
TX -> BEGIN,B,20,4,1
RX <- OK,BEGIN,B,20
TX -> PT,A,0,300,520,512,512,512
RX <- OK,PT,A,0
TX -> PT,B,0,300,520,512,512,512
RX <- OK,PT,B,0
TX -> END,A,10
RX <- OK,END,A,10
TX -> END,B,20
RX <- OK,END,B,20
```

ROS 必須一問一答，核對命令名稱、手臂、軌跡 ID／序號。ACK 只表示命令處理完成、目標封包已送出；不表示馬達已接收或到位。`dt_ms` 不會在 CM530 內延時，ROS 自行安排發送時間與協作順序。

`STOP,A/B` 取消該臂軌跡狀態，重送該臂最後成功傳送的目標；不讀取當前位置、不關 torque，因此不是立即停止移動的急停。開機尚無目標時，最後目標預設為四軸 512。

不包含八軸同時命令、位置回讀、IK、吸盤控制、碰撞規劃或自主交接。完整協定見 [ROS_CM530_INTERFACE_SPEC.txt](ROS_CM530_INTERFACE_SPEC.txt)。

## 程式結構

- `APP/src/bridge.c`：A／B 狀態、輸入解析、四軸 SYNC_WRITE。
- `APP/src/main.c`：原第 15 版時鐘、UART、中斷與硬體適配；接收資料損壞時丟棄該行。
- `manual_position_terminal.py`：手動操作、精確 ACK 核對及示範流程。
- `tests/test_bridge.c`：正式 bridge + Dynamixel SDK，模擬實體 HAL 檢查封包。

開機先初始化兩臂 torque，再輸出唯一一行 `READY`；不自動發送 HOME。沿用第 15 版的 best-effort torque 政策：每顆嘗試一次，沒有 status packet 也不阻擋後續位置封包。八顆都未回覆時，初始化可能約 4.8 秒，終端預設啟動監聽 6 秒。READY 不是馬達連線成功證明。

## 建置

需要 GNU make、ARM Embedded GCC 與 objcopy；電腦端 GCC 僅能跑離線測試。

在本資料夾執行，沿用舊工具鏈名稱：

```powershell
make CM530.hex CM530.bin
```

若工具鏈名稱為 `arm-none-eabi-*`，可覆寫前綴：

```powershell
make TCHAIN_PREFIX=arm-none-eabi- CM530.hex CM530.bin
```

SDK 是舊版 STM32／ROBOTIS 程式；更换工具鏈時須檢查相容性及完整建置結果。燒錄前須確認輸出來自第 16 版成功建置，不能使用第 15 版檔案代替。已使用 GNU Arm Embedded 10.3-2021.10 完成建置並驗證 HEX／BIN 一致。詳細結果見 [VALIDATION.md](VALIDATION.md)。

## 離線驗證

不需要連接控制板，Python 自測也不需要 pyserial：

```powershell
python manual_position_terminal.py --self-test
gcc -std=gnu89 -Wall -Wextra -IAPP/inc APP/src/bridge.c APP/src/dynamixel.c tests/test_bridge.c -o tests/test_bridge.exe
./tests/test_bridge.exe
```

亦可執行 `tests/run_tests.ps1 -Python <python路徑> -Compiler <gcc路徑>`。詳細已完成／未完成項目見 [VALIDATION.md](VALIDATION.md)。

本次驗證通過的完整重建命令為 `make -B TCHAIN_PREFIX=arm-none-eabi- CM530.hex CM530.bin`。建置後可執行 `python tests/verify_firmware.py` 檢查 ELF、HEX 與 BIN。
