# 第 16 版 CM530 DUAL ARM：直接轉接韌體

本版接收 ROS 已計算完成的四軸目標、torque 與 LED 命令，執行參數驗證、指定手臂的封包傳送或燈號設定，並回 ACK／ERR。**不執行運動計算或任務流程。**

- 專案版本：16；主機序列協定：**4**。
- 啟動識別：`READY,4`；版本查詢：`VERSION,4`。
- 馬達端：Dynamixel Protocol **1.0**，與主機序列協定版本不同。
- 更新：2026-09-30。
- [完整 ROS 對接規格](ROS_CM530_INTERFACE_SPEC.txt)／[系統架構](../README.md)／[驗證紀錄](VALIDATION.md)。

## 1. 實作範圍

| 韌體保留 | 交由 ROS／外部模組 |
|---|---|
| ASCII 命令解析與範圍檢查 | 視覺、3D 定位、座標轉換、IK、關節校正 |
| arm1／arm2 固定 ID 路由 | 軌跡產生、插值、取樣、發送時間與完成判斷 |
| 四軸 Goal Position／Torque Enable SYNC_WRITE | A → B → C 流程、Idle/Pick/Place/Return 四狀態 |
| 開機 torque-off、每臂目標資格 | B 站互斥、C 站 2×4 格位的辨識、預留與選擇 |
| LED 顯示命令、ACK／ERR、UART 丟行恢復 | 吸盤／ESP32、監控、異常恢復及記錄 |

不提供 HOME、HOLD、軌跡封裝、位置查詢、速度控制、到位事件、micro-ROS client 或 ROS 節點。所有主機動作只發送寫入封包；不發送 READ。

## 2. 硬體與接線責任

| 關節順序（沿用由下到上） | arm1 ID | arm2 ID |
|---|---:|---:|
| j1 | 17 | 12 |
| j2 | 3 | 1 |
| j3 | 2 | 8 |
| j4 | 15 | 16 |

| 路徑 | 設定 |
|---|---|
| PC／Jetson → CM530 主機通訊介面 | 單埠、57600 baud、8 data bits、no parity、1 stop bit、無流量控制 |
| 主機硬體適配 | USART3 |
| 馬達硬體適配 | USART1 → CM530 Dynamixel TTL bus |
| 八顆 AX-12A | 共用 bus，1 Mbps，Protocol 1.0 |
| 四軸位置寫入 | Goal Position 位址 30，2 bytes／軸 |
| 四軸 torque 寫入 | Torque Enable 位址 24，1 byte／軸 |

使用 CM530 既有主機通訊連接及 AX-12A TTL 接口；不將電腦的 RS-232 訊號直接接到馬達 bus。兩臂馬達接在同一條 TTL 通訊網路，韌體依 ID 選擇手臂，不以接頭位置辨識手臂。吸盤接外部控制器，沒有本版 CM530 吸盤指令。

供電、纜線、接頭方向、八顆 ID 唯一性與 1 Mbps 設定需在實機事先確認。韌體不改 ID、EEPROM、速度、限位或零位。位置 0～1023 僅為資料範圍，不能代替機械關節限制；512 也不是已校正的原點或 Return 姿態。

## 3. 開機與目標資格

開機依序對 arm1、arm2 各送一筆四軸 torque-off；第一筆失敗仍嘗試第二筆。

- 兩筆本地傳送成功：`READY,4`。
- 任一筆失敗：`ERR,INIT_FAILED`；動作命令鎖定到重啟，PING／VERSION／LED 仍可用。
- 不發位置、不啟用 torque，不設定預設 512 目標。

每臂只記錄「可啟用目標」資格，不記錄已確認的馬達 torque 或到位狀態：

| 事件 | 該臂目標資格 |
|---|---|
| 開機 | 清除 |
| AX 參數正確且位置封包傳送成功 | 建立 |
| AX 傳送失敗 | 清除；不能使用舊目標資格 |
| 有效 TORQUE,0 命令 | 傳送前即清除，即使傳送失敗也不恢復 |
| TORQUE,1 | 需要有效資格；不改變資格 |
| 格式／範圍錯誤、未知命令 | 不改變 |
| 另一臂的命令 | 不改變 |

啟用順序為 **完整 AX 目標 → 等待 OK → TORQUE,1 → 等待 OK**。torque 關閉時可以寫目標。若 torque 已啟用，AX 可能立刻產生動作；重新開啟終端不會重設控制板或馬達。

READY 與馬達 OK 只表示本地傳送完成，LED OK 只表示 GPIO 已設定，都不確認馬達實際卸力、施力或到位。初始化失敗後重新送 VERSION 不會解鎖。

## 4. 命令與錯誤

每行以 LF、CR 或 CRLF 結尾；回覆固定 CRLF。命令與手臂大小寫不拘，回覆手臂固定小寫。正式 ROS 請送大寫命令、小寫手臂、ASCII 半形逗號與十進位整數。

| 命令範例 | 成功回覆 |
|---|---|
| `PING` | `PONG` |
| `VERSION` | `VERSION,4` |
| `AX,arm1,512,512,512,512` | `OK,AX,arm1` |
| `AX,arm2,520,512,512,512` | `OK,AX,arm2` |
| `TORQUE,arm1,0` | `OK,TORQUE,arm1,0` |
| `TORQUE,arm1,1` | `OK,TORQUE,arm1,1` |
| `LED,arm1,MOVING` | `OK,LED,arm1,MOVING`（正常終端不印出） |
| `LED,arm2,STOPPED` | `OK,LED,arm2,STOPPED`（正常終端不印出） |

AX 每軸為 0～1023，固定四個值；TORQUE 只接受 0／1。拒絕省略手臂、A/B 代號、ALL/BOTH、單值 AX 及八軸一次命令。已移除的 HOME/HOLD/BEGIN/PT/END/STOP 回 BAD_CMD，不做相容轉譯。

正常非空命令一行一回覆；空白行不回覆。行緩衝區 96 bytes，最多 95 bytes 內容（不含行尾）。超限或 UART 資料丟失會回 OVERFLOW，捨棄損壞行到下一個行尾。

| 錯誤碼 | 意義與處理方向 |
|---|---|
| BAD_CMD | 不支援的命令；更新客戶端，不改用舊命令 |
| BAD_ARG | 手臂、欄位數、整數格式／32-bit 溢位、torque 值或 LED 狀態錯誤 |
| RANGE | AX 整數可解析，但超出 0～1023 |
| NO_TARGET | 先成功傳送該臂完整 AX，才允許 TORQUE,1 |
| DXL_TX | 本地馬達封包傳送失敗，不可視為「一定沒動」 |
| INIT_FAILED | 啟動傳送失敗，動作鎖定到控制板重啟 |
| OVERFLOW | 行太長或 UART 丟資料，整行丟棄 |

一般錯誤為 `ERR,<code>[,<arm>]`，有辨識到合法手臂才附手臂；啟動 INIT_FAILED 及 OVERFLOW 不附手臂。不存在馬達 ID、序號或遙測欄位。詳細錯誤優先順序與副作用見對接規格。

### LED 規則

| 手臂 | 綠燈 MOVING | 紅燈 STOPPED |
|---|---|---|
| arm1 左欄 | MANAGE，PB13 | PROGRAM，PB14 |
| arm2 右欄 | TX，PC14 | RX，PC15 |

顏色依使用者確認的板子配置；腳位參考原始 `01 LED`。應用韌體初始化輸出後，BridgeInit 在馬達初始化傳送前設定雙紅待命，不承諾 bootloader 階段的燈號。紅色開機值是待命預設，不代表量測到馬達停止。

`LED,<arm>,MOVING|STOPPED` 每次只更新指定手臂。低電位亮、高電位滅，先滅舊燈再亮新燈，兩臂互不影響。PLAY／AUX 固定熄滅，POWER 不變；TX／RX 改為 arm2 指示，不另作通訊閃燈。

LED 不發馬達封包、不改 torque／有效目標，即使馬達初始化失敗仍可設定。錯誤參數不改燈；重複設定仍正常 ACK。AX／TORQUE 不改燈；demo、斷線、退出與異常也不自動追加燈號命令。重啟回雙紅，其他情況保留最後設定。

ROS 準備動作時明確送 MOVING，依外部完成判斷確認停止後送 STOPPED。STOPPED 只切燈，不是停止命令；燈號不是馬達感測回報，斷線時可能保留舊顯示。

## 5. 建置、映像與燒錄

以下命令在本資料夾執行。需要 GNU make、ARM GCC／objcopy、Python；主機 C 測試另需 GCC 或 Zig。

已驗證 Arm GNU Toolchain 14.2 rel1 的建置方式：

```powershell
make -B TCHAIN_PREFIX=arm-none-eabi- "COMPILE_OPTS=-mcpu=cortex-m3 -mthumb -Wall -g -Os -fno-common -fno-strict-aliasing -Wno-error=implicit-function-declaration -Wno-error=incompatible-pointer-types -Wno-error=int-conversion" CM530.hex CM530.bin
python tests/verify_firmware.py
```

相容旗標用於既有 STM32／ROBOTIS SDK；仍存在 HAL 函式宣告及 vector 警告，詳見 VALIDATION。修改 header／旗標後使用 -B 完整重建，避免使用舊 object。Makefile 預設是 arm-eabi-*；改用其他工具鏈時須自行確認完整建置結果。

| 產物 | 用途 |
|---|---|
| CM530.elf | 連結結果、區段與符號 |
| CM530.hex | 帶地址的 Intel HEX |
| CM530.bin | 原始二進位映像 |
| CM530.elf.map | 連結配置 |
| SHA256SUMS | 交付 ELF／HEX／BIN 的 SHA-256；make 不自動更新 |

映像檢查涵蓋 ELF 配置、HEX checksum、三種映像的可載入資料一致性、向量、ID 表、協定 4 識別，以及舊命令標記不存在。ELF 含除錯資訊，工具鏈或建置路徑改變可能改變 ELF 雜湊；以同次建置的三個檔案驗證。

程式 Flash 起點為 `0x08003000`，初始 stack 為 `0x20010000`。使用既有 CM530 載入流程燒錄本次映像；若載入工具要求 BIN 起點，必須與 linker 配置一致。不可將原始 BIN 任意寫入 Flash 0 起點。本專案不提供自動燒錄腳本，也未代為接板燒錄。

```powershell
Get-FileHash .\CM530.hex -Algorithm SHA256
```

將結果與 SHA256SUMS 比較。ELF 及中間編譯檔依既有 ignore 規則留在本機；移交完整驗證產物時一起提供同次 ELF／HEX／BIN。

## 6. 手動終端

```powershell
python -m pip install -r requirements.txt
python manual_position_terminal.py --port COM4 --arm arm1
```

requirements 使用 pyserial>=3.5,<4。COM4 改成實際埠；Linux／Jetson 使用實際裝置路徑。先關閉 RoboPlus、其他終端及 ROS 序列程式。

| 參數 | 預設 | 說明 |
|---|---|---|
| --port | COM4 | 裝置路徑 |
| --baud | 57600 | 須匹配韌體；不會改寫韌體 baudrate |
| --arm | 無 | 四數字快捷輸入與 demo 的目標手臂 |
| --timeout | 2.0 秒 | 命令送完後的回覆期限 |
| --startup-listen | 6.0 秒 | READY 監聽，可設 0；仍執行 VERSION |
| --line-end | lf | 可選 lf/cr/crlf |
| --char-delay | 0 秒 | 人工除錯逐字延遲，正式對接保持 0 |
| --self-test | 關閉 | 離線 Python 測試，不開埠 |

終端先監聽 READY，再強制核對 VERSION,4，最後 PING。READY 若在開埠前已送出，仍可透過 VERSION 握手，但不能據此假定 torque 已關閉。舊 READY／VERSION 不相容時直接停止。

`?`／`help` 顯示說明；`demo` 執行示範；`q`／`quit`／`exit` 離開。指定 --arm 後可輸入四個數字，例如 `520 512 512 512`；單值 `512` 不接受。完整命令中的手臂不被 --arm 改寫。

LED 正常操作不印 `TX -> LED,...` 或 `RX <- OK,LED,...`，也不增加狀態提示；ACK 仍在線上傳輸且在內部完整核對。LED 的錯誤、逾時、重啟與錯配仍顯示錯誤並停止操作，其他命令和啟動識別照常顯示。

LOCAL ERR 只表示本地輸入被拒絕、未送出，可繼續輸入。每次發送前也檢查已收到的非空殘留資料，防止把舊 ACK 或等待期間的 READY 當成新回覆。板子 ERR、逾時、錯配 ACK、重啟或序列錯誤則結束本次操作；不自動重送、不追加 torque-off。退出及 Ctrl+C 也沒有自動動作。

## 7. 操作範例與 demo

下列數值僅示範通訊，不是 A/B/C 座標或已校正姿態。每筆命令等待對應回覆，位置更新時機由主機判斷。

```text
TX -> AX,arm1,512,512,512,512
RX <- OK,AX,arm1
TX -> TORQUE,arm1,1
RX <- OK,TORQUE,arm1,1
TX -> AX,arm1,520,512,512,512
RX <- OK,AX,arm1
```

兩臂任務可並行，但同一序列埠只允許一筆待回覆命令；兩臂 AX 逐筆交錯發送，不保證同步起動。韌體不儲存點列或執行定時器排程。

demo 確切流程：PING；依選定手臂送四軸 512 的 AX、TORQUE,1；再依 j1=512→520→512，其他三軸=512，交替發送 AX。每筆 AX 後等 0.3 秒，每筆 TORQUE,1 後等 1 秒；這些是主機等待，不是到位證據。有 --arm 只測該臂，沒有則測兩臂。

demo 使用前需確認數值與現有 torque 狀態適合機構。demo 不先卸力，不在結束時卸力，不回傳物理完成事件。Return 應由 ROS 發送它所設定的姿態，不由韌體提供 HOME 替代。

本版無當前位置回讀，沒有等效 HOLD。重送最後目標仍是前往該目標，不能宣稱停在當前位置；TORQUE,0 是卸力，不是保持位置或硬體急停。

## 8. 程式結構與離線測試

| 檔案 | 職責 |
|---|---|
| APP/src/main.c、stm32f10x_it.c | 時鐘、GPIO、UART、計時器及中斷適配 |
| APP/src/bridge.c、APP/inc/bridge.h | 解析、ACK／ERR、初始化鎖定及每臂目標資格 |
| APP/src/arm_led.c、APP/inc/arm_led.h | LED GPIO 初始化、映射與紅綠切換 |
| APP/src/ax12.c、APP/inc/ax12.h | 固定 ID 表、目標／torque SYNC_WRITE |
| APP/src/dynamixel.c、dxl_hal.c | 既有 Protocol 1.0 SDK、實體傳輸及有界接收 |
| manual_position_terminal.py | 協定 4 終端、版本及 ACK 核對 |
| tests/test_bridge.c | 正式 bridge＋AX12＋SDK，僅模擬實體 HAL |
| tests/test_arm_led.c、tests/fakes | 正式 LED 適配層與模擬 GPIO／RCC 測試 |
| tests/test_terminal.py | 終端輸入、握手、回覆及錯誤中止 |
| tests/verify_firmware.py | ELF／HEX／BIN 檢查 |

通用 SDK 的接收器與 50 ms 接收常數仍保留，供 SDK 回歸測試；協定 4 沒有呼叫位置回讀的命令，不將此期限當成 ROS 讀位置功能。

```powershell
python manual_position_terminal.py --self-test
./tests/run_tests.ps1 -Python python -Compiler gcc
```

使用 Zig 時以實際路徑取代範例：

```powershell
./tests/run_tests.ps1 -Python python -Compiler "C:\tools\zig\zig.exe" -CompilerArgs cc
```

不需 pyserial 或控制板即可離線測試；主機測試編譯使用 -Wall -Wextra -Werror。完整建置警告、雜湊與尚未執行的實機項目列於 [VALIDATION.md](VALIDATION.md)。

## 9. 舊版 → 協定 4 遷移

| 舊功能 | 協定 4 處理 |
|---|---|
| 協定 2／3 的 READY／VERSION | 更新為 READY,4／VERSION,4，拒絕版本不符 |
| 協定 3 無 LED 命令 | 增加 LED 指令；正常 LED TX／ACK 不印出，內部核對保留 |
| 單值 AX | ROS／終端必須明確提供四軸 |
| HOME | ROS 管理校正後的姿態，送完整 AX |
| BEGIN／PT／END | ROS 管理點列、序號、時間和完成判斷，逐筆送 AX |
| HOLD | 已移除；沒有位置回讀，不能以最後目標冒充當前位置保持 |
| STOP | 仍不支援，不能靜默映射為 TORQUE,0 |
| 開機 torque-off、明確 torque-on | 保留；資格只由成功 AX 建立 |
| 遙測／到位回報 | 本版仍不提供 |

先升級韌體與終端並完成實機驗收，再切換 ROS 客戶端。其他歷史資料夾保留原樣；主機協定不相容不能靠重送舊命令解決。
