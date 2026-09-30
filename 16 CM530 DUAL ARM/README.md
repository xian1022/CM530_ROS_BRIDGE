# 第 16 版 CM530 DUAL ARM：直接轉接韌體

這份文件是第 16 版 CM-530 控制板韌體的操作與開發說明，涵蓋接線、四軸馬達控制、燈號、通訊指令、建置、燒錄及測試。若第一次接觸專案，可先閱讀[專案首頁](../README.md)的控制板與基本操作介紹。

本版接收 ROS 已計算完成的四軸目標、torque（馬達施力開關）與 LED（指示燈）命令，執行參數驗證、指定手臂的封包傳送或燈號設定，並回 ACK（確認回覆）／ERR（錯誤回覆）。**運動計算與任務流程由 ROS 主機負責。** ROS 是機器人軟體框架，本資料夾沒有 ROS 節點程式。

- 專案版本：16；主機序列協定：**4**。
- 啟動識別：`READY,4`；版本查詢：`VERSION,4`。
- 馬達端：Dynamixel Protocol **1.0**，與主機序列協定版本不同。
- 文件更新：2026-10-01；本次為閱讀說明與圖片更新，韌體仍使用協定 4。
- [完整 ROS 對接規格](ROS_CM530_INTERFACE_SPEC.txt)／[系統架構](../README.md)／[驗證紀錄](VALIDATION.md)。

## 閱讀導覽

| 你的需求 | 建議閱讀 |
|---|---|
| 先了解哪些工作由控制板負責 | 第 1 節：實作範圍 |
| 看板子、接線與馬達編號 | 第 2 節：硬體與接線 |
| 理解為何不能直接啟用馬達 | 第 3 節：開機與目標資格 |
| 查指令中文解釋、燈號與錯誤碼 | 第 4 節：命令與錯誤 |
| 把程式編譯並寫入控制板 | 第 5 節：建置、映像與燒錄 |
| 在電腦上手動發指令 | 第 6、7 節：終端與操作範例 |
| 修改程式、驗證或從舊版升級 | 第 8、9 節 |
| 遇到連線或操作疑問 | 第 10 節：常見問題 |

## 1. 實作範圍

韌體（Firmware）是寫入控制板、開機後執行的程式。本版工作順序為「接收命令 → 檢查 → 選擇手臂 → 寫入馬達或設定燈號 → 回覆」，其中四軸是指一支手臂的四個馬達關節。

| 韌體功能 | 處理內容 |
|---|---|
| 文字命令解析 | 檢查指令、手臂名稱、欄位數及數值範圍 |
| 固定馬達映射 | 依 arm1／arm2 選擇該臂四顆馬達 ID |
| 四軸位置寫入 | 用 SYNC_WRITE（同步寫入封包）一次送出四顆馬達的目標位置 |
| 施力開關與目標資格 | 開機送出卸力封包；只有成功設定目標後才允許啟用施力 |
| 紅綠燈控制 | 依 LED 命令更新指定手臂的板上燈號 |
| 通訊回覆與錯誤處理 | 提供版本、ACK／ERR；UART（序列收發介面）丟資料時捨棄損壞行 |

主機負責先算好四軸目標與發送時間；視覺、任務協調與吸盤由外部模組處理。整體應用圖見[首頁的系統圖](../README.md#7-應用背景與系統圖)。

本版不提供 HOME、HOLD、軌跡封裝、位置查詢、速度控制或到位事件；也不實作 ROS 節點或 micro-ROS（微控制器與 ROS 2 整合方案）。SYNC_WRITE 只表示一筆封包寫入多顆馬達，不是到位判斷；所有支援命令都不發送 READ（馬達資料讀取封包）。

## 2. 硬體與接線責任

### 2.1 認識控制板與面板

![CM-530 正面外觀：PC LINK、DXL 接口、電源及左右兩欄指示燈](../docs/images/cm530-controller.png)

*使用者提供的控制板外觀參考圖。左右方向以此正面朝向為準，照片亮燈不代表目前韌體狀態。*

CM-530 使用 STM32F103RE 微控制器，提供 PC LINK 主機連線、DYNAMIXEL 馬達接口與周邊接口；板子接頭與硬體資料以 [ROBOTIS 官方手冊](https://emanual.robotis.com/docs/en/parts/controller/cm-530/)為準。本專案使用自訂 C 語言韌體，不能直接套用原廠 RoboPlus 的模式燈含義與操作流程。

| 面板標示 | 中文說明與本版用途 |
|---|---|
| PC LINK | 電腦通訊端；以相容通訊線連接主機，使用下列 57600 設定 |
| DXL | DYNAMIXEL 馬達接口；八顆 AX-12A 串接在共用通訊網路 |
| BAT、12V DC、POWER | 電池端、直流電源端、電源標示；供電須同時符合控制板與馬達規格 |
| MANAGE、PROGRAM、PLAY | 左欄管理／程式／執行標示；本版用前兩顆顯示 arm1，PLAY 熄滅 |
| TX（TxD）、RX（RxD）、AUX | 右欄傳送／接收／輔助標示；本版用前兩顆顯示 arm2，AUX 熄滅 |
| MODE、START、U/L/D/R | 模式、開始及方向按鍵；本版應用程式未實作按鍵動作或停止功能 |
| WIRELESS、PORT | 無線模組／周邊接口；本版不透過它們控制吸盤或接收 ROS 命令 |

照片不能代替接腳圖。電源、接頭方向及線材請依[官方腳位說明](https://emanual.robotis.com/docs/en/parts/controller/cm-530/#pinout)核對；下列設定描述本版應用程式，不是開機載入程式（bootloader）的操作規則。

### 2.2 四個關節與馬達 ID

`arm1`／`arm2` 是通訊中的手臂名稱；`j1`～`j4` 是四軸的位置順序。ID 是馬達的唯一裝置編號，不是控制板接頭編號。

| 關節順序（沿用由下到上） | arm1 ID | arm2 ID |
|---|---:|---:|
| j1 | 17 | 12 |
| j2 | 3 | 1 |
| j3 | 2 | 8 |
| j4 | 15 | 16 |

### 2.3 主機與馬達的通訊設定

| 路徑 | 設定 |
|---|---|
| PC／Jetson → CM530 主機通訊介面 | 單埠、57600 baud、8 data bits、no parity、1 stop bit、無流量控制 |
| 主機硬體適配 | USART3 |
| 馬達硬體適配 | USART1 → CM530 Dynamixel TTL bus |
| 八顆 AX-12A | 共用 bus，1 Mbps，Protocol 1.0 |
| 四軸位置寫入 | Goal Position 位址 30，2 bytes／軸 |
| 四軸 torque 寫入 | Torque Enable 位址 24，1 byte／軸 |

57600 baud 表示主機端序列傳輸速率；8N1 是「8 個資料位元、無同位檢查、1 個停止位元」的簡寫，1 Mbps 則是每秒一百萬位元。USART1／USART3 是板內序列通訊硬體名稱。TTL bus 是馬達共用的通訊線；Goal Position 是目標位置，Torque Enable 是施力啟用開關，位址 30／24 是馬達內部控制表欄位，主機不需要自行輸入這些位址。

使用 CM530 既有主機通訊連接及 AX-12A TTL 接口；不將電腦的 RS-232 訊號直接接到馬達 bus。兩臂馬達接在同一條 TTL 通訊網路，韌體依 ID 選擇手臂，不以接頭位置辨識手臂。吸盤接外部控制器，沒有本版 CM530 吸盤指令。

供電、纜線、接頭方向、八顆 ID 唯一性與 1 Mbps 設定需在實機事先確認。韌體不改 ID、EEPROM、速度、限位或零位。位置 0～1023 僅為資料範圍，不能代替機械關節限制；512 也不是已校正的原點或 Return 姿態。

## 3. 開機與目標資格

### 3.1 開機訊息

開機依序對 arm1、arm2 各送一筆四軸 torque-off；第一筆失敗仍嘗試第二筆。

- 兩筆本地傳送成功：`READY,4`。
- 任一筆失敗：`ERR,INIT_FAILED`；動作命令鎖定到重啟，PING／VERSION／LED 仍可用。
- 不發位置、不啟用 torque，不設定預設 512 目標。

READY 是「初始化傳送流程完成」的啟動訊息，數字 4 是主機協定版本。啟用 torque 是讓馬達施力追蹤目標；關閉 torque 則是卸力，不是要求馬達固定在某個位置。

### 3.2 為何啟用前必須先設定目標

「目標資格」是韌體內部記錄：這支手臂是否已成功傳送過可供啟用的完整位置目標。它避免主機未設定新目標就開啟施力，但不代表韌體知道馬達的實際位置。

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

### 4.1 文字格式與欄位

每行以 LF、CR 或 CRLF 結尾；回覆固定 CRLF。命令與手臂大小寫不拘，回覆手臂固定小寫。正式 ROS 請送大寫命令、小寫手臂、ASCII 半形逗號與十進位整數。

ASCII 是此處使用的英文字母、數字與標點文字編碼；LF 是換行、CR 是歸位、CRLF 是兩者組合。手動終端會替每筆輸入加上行尾，通常保持預設 LF 即可。

以 `AX,arm1,520,512,500,512` 為例，各欄依序是：`AX`（設定位置）、`arm1`（手臂 1）、`520`（j1）、`512`（j2）、`500`（j3）、`512`（j4）。數值是馬達原始位置，不是角度或公釐；範例不代表已校正或適合實際機構。

| 命令範例 | 中文解釋 | 成功回覆 |
|---|---|---|
| `PING` | 確認通訊是否可回應；不測試馬達到位 | `PONG` |
| `VERSION` | 查詢主機通訊協定版本 | `VERSION,4` |
| `AX,arm1,512,512,512,512` | 寫入手臂 1 的四軸目標；不自動啟用施力 | `OK,AX,arm1` |
| `AX,arm2,520,512,512,512` | 寫入手臂 2 的四軸目標 | `OK,AX,arm2` |
| `TORQUE,arm1,0` | 對手臂 1 發出卸力封包，並清除目標資格 | `OK,TORQUE,arm1,0` |
| `TORQUE,arm1,1` | 對手臂 1 發出啟用施力封包，需先有目標資格 | `OK,TORQUE,arm1,1` |
| `LED,arm1,MOVING` | 將手臂 1 燈號設成動作中，綠亮紅滅 | `OK,LED,arm1,MOVING`（正常終端不印出） |
| `LED,arm2,STOPPED` | 將手臂 2 燈號設成停止／待命，紅亮綠滅；不停止馬達 | `OK,LED,arm2,STOPPED`（正常終端不印出） |

AX 每軸為 0～1023，固定四個值；TORQUE 只接受 0／1。拒絕省略手臂、A/B 代號、ALL/BOTH、單值 AX 及八軸一次命令。已移除的 HOME/HOLD/BEGIN/PT/END/STOP 回 BAD_CMD，不做相容轉譯。

正常非空命令一行一回覆；空白行不回覆。行緩衝區 96 bytes，最多 95 bytes 內容（不含行尾）。超限或 UART 資料丟失會回 OVERFLOW，捨棄損壞行到下一個行尾。

### 4.2 錯誤碼與處理

ERR 表示錯誤；例如 `ERR,NO_TARGET,arm1` 是「手臂 1 沒有可啟用的目標」。下表是回覆中的代碼，不能當作命令輸入。

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

### 4.3 LED（指示燈）規則

| 手臂 | 綠燈 MOVING | 紅燈 STOPPED |
|---|---|---|
| arm1 左欄 | MANAGE，PB13 | PROGRAM，PB14 |
| arm2 右欄 | TX，PC14 | RX，PC15 |

MOVING 是動作中，STOPPED 是停止／待命。PB13 表示 GPIOB 的第 13 號腳位，PC14 表示 GPIOC 的第 14 號腳位；這些是程式對照用的硬體名稱，不是需要額外外接 LED 的指示。

顏色依使用者確認的板子配置；腳位參考原始 `01 LED`。應用韌體初始化輸出後，BridgeInit 在馬達初始化傳送前設定雙紅待命，不承諾 bootloader 階段的燈號。紅色開機值是待命預設，不代表量測到馬達停止。

`LED,<arm>,MOVING|STOPPED` 每次只更新指定手臂。低電位亮、高電位滅，先滅舊燈再亮新燈，兩臂互不影響。PLAY／AUX 固定熄滅，POWER 不變；TX／RX 改為 arm2 指示，不另作通訊閃燈。

LED 不發馬達封包、不改 torque／有效目標，即使馬達初始化失敗仍可設定。錯誤參數不改燈；重複設定仍正常 ACK。AX／TORQUE 不改燈；demo、斷線、退出與異常也不自動追加燈號命令。重啟回雙紅，其他情況保留最後設定。

ROS 準備動作時明確送 MOVING，依外部完成判斷確認停止後送 STOPPED。STOPPED 只切燈，不是停止命令；燈號不是馬達感測回報，斷線時可能保留舊顯示。

## 5. 建置、映像與燒錄

### 5.1 建置前準備

建置（Build）是把 C 原始碼編譯成控制板可執行的程式映像；燒錄（Flash）則是將映像寫入控制板。Python 終端只負責通訊，不會自動完成這兩個步驟。

以下命令在本資料夾執行。需要 GNU make、ARM GCC／objcopy、Python；主機 C 測試另需 GCC 或 Zig。

GNU make 依 Makefile 的規則建置；ARM GCC 是把 C 程式編成 ARM 處理器指令的編譯器；objcopy 用於轉換映像格式。請先將工具加入 PATH（系統尋找可執行程式的路徑）。若從專案根目錄開始，先執行 `cd "16 CM530 DUAL ARM"`。建置與測試工具屬於電腦端，不安裝在控制板上。

### 5.2 完整建置與映像檢查

已驗證 Arm GNU Toolchain 14.2 rel1 的建置方式：

```powershell
make -B TCHAIN_PREFIX=arm-none-eabi- "COMPILE_OPTS=-mcpu=cortex-m3 -mthumb -Wall -g -Os -fno-common -fno-strict-aliasing -Wno-error=implicit-function-declaration -Wno-error=incompatible-pointer-types -Wno-error=int-conversion" CM530.hex CM530.bin
python tests/verify_firmware.py
```

| 命令片段 | 用途 |
|---|---|
| `make -B` | 強制完整重建，不沿用既有中間檔 |
| `TCHAIN_PREFIX=arm-none-eabi-` | 指定 ARM 工具名稱的前綴；須與已安裝工具一致 |
| `COMPILE_OPTS=...` | 指定處理器、編譯選項與舊 SDK 相容設定；一般使用者可保留整段 |
| `-mcpu=cortex-m3 -mthumb` | 指定本板使用的處理器與指令集 |
| `-Wall -g -Os` | 開啟常用警告、保留除錯資訊、以縮小程式為最佳化目標 |
| `-fno-common -fno-strict-aliasing` | 控制全域符號與指標相關編譯行為，沿用本專案設定 |
| `-Wno-error=...` | 指定舊 SDK 的某類警告不視為編譯錯誤；警告仍會列出 |
| `CM530.hex CM530.bin` | 指定要產生的 HEX 與 BIN 產物 |
| `python tests/verify_firmware.py` | 在電腦檢查映像內容；不會燒錄或驅動馬達 |

相容旗標用於既有 STM32／ROBOTIS SDK（軟體開發套件）；仍存在 HAL（硬體適配層）函式宣告及 vector（中斷向量）警告，詳見 VALIDATION。修改 header（標頭檔）／旗標後使用 -B 完整重建，避免使用舊 object（中間目的檔）。Makefile 預設是 arm-eabi-*；改用其他工具鏈時須自行確認完整建置結果。

### 5.3 產物與燒錄注意事項

| 產物 | 用途 |
|---|---|
| CM530.elf | 連結結果、區段與符號 |
| CM530.hex | 帶地址的 Intel HEX |
| CM530.bin | 原始二進位映像 |
| CM530.elf.map | 連結配置 |
| SHA256SUMS | 交付 ELF／HEX／BIN 的 SHA-256；make 不自動更新 |

ELF 是含程式區段與符號資訊的執行檔格式；HEX 是附記憶體位址的文字映像；BIN 是原始二進位資料。SHA-256 是檔案內容摘要，用於核對檔案是否相同，不是韌體版本號。

映像檢查涵蓋 ELF 配置、HEX checksum、三種映像的可載入資料一致性、向量、ID 表、協定 4 識別，以及舊命令標記不存在。ELF 含除錯資訊，工具鏈或建置路徑改變可能改變 ELF 雜湊；以同次建置的三個檔案驗證。

程式 Flash 起點為 `0x08003000`，初始 stack 為 `0x20010000`。使用既有 CM530 載入流程燒錄本次映像；若載入工具要求 BIN 起點，必須與 linker 配置一致。不可將原始 BIN 任意寫入 Flash 0 起點。本專案不提供自動燒錄腳本，也未代為接板燒錄。

```powershell
Get-FileHash .\CM530.hex -Algorithm SHA256
```

將結果與 SHA256SUMS 比較。ELF 及中間編譯檔依既有 ignore 規則留在本機；移交完整驗證產物時一起提供同次 ELF／HEX／BIN。

## 6. 手動終端

### 6.1 安裝與連線

終端（Terminal）是在電腦上輸入指令、查看回覆的工具。以下兩行在 Windows PowerShell 中執行，工作目錄為本資料夾：

```powershell
python -m pip install -r requirements.txt
python manual_position_terminal.py --port COM4 --arm arm1
```

第一行使用 pip（Python 套件安裝工具）讀取 requirements.txt 並安裝相依套件；第二行啟動本專案的通訊程式。`--port COM4` 指定序列埠，`--arm arm1` 指定快捷輸入與示範操作的預設手臂。可在 Windows 裝置管理員的連接埠項目核對實際 COM 編號。

requirements 使用 pyserial>=3.5,<4。COM4 改成實際埠；Linux／Jetson 使用實際裝置路徑。先關閉 RoboPlus、其他終端及 ROS 序列程式。

pyserial 是 Python 的序列通訊套件；`>=3.5,<4` 表示允許版本 3.5 以上但低於 4。Linux 的連接埠名稱可能是 `/dev/ttyUSB0`，仍須依實際裝置確認。

### 6.2 啟動參數

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

表中的參數加在程式檔名後，不是送給控制板的指令。例如 `--timeout 2` 表示等回覆最多 2 秒；`--baud` 是電腦端通訊速率，無法藉此修改板上的固定速率。

### 6.3 連線核對與互動輸入

終端先監聽 READY，再強制核對 VERSION,4，最後 PING。READY 若在開埠前已送出，仍可透過 VERSION 握手，但不能據此假定 torque 已關閉。舊 READY／VERSION 不相容時直接停止。

`?`／`help` 顯示說明；`demo` 執行示範；`q`／`quit`／`exit` 離開。指定 --arm 後可輸入四個數字，例如 `520 512 512 512`；單值 `512` 不接受。完整命令中的手臂不被 --arm 改寫。

這些 help／demo／quit 是終端自己的操作，不是韌體命令。終端畫面中的 `TX ->` 是電腦送出，`RX <-` 是電腦收到；只輸入後面的命令內容，不要輸入 TX／RX 標記，也不要輸入板子回覆的 OK。

### 6.4 靜默顯示與錯誤中止

LED 正常操作不印 `TX -> LED,...` 或 `RX <- OK,LED,...`，也不增加狀態提示；ACK 仍在線上傳輸且在內部完整核對。LED 的錯誤、逾時、重啟與錯配仍顯示錯誤並停止操作，其他命令和啟動識別照常顯示。

LOCAL ERR 只表示本地輸入被拒絕、未送出，可繼續輸入。每次發送前也檢查已收到的非空殘留資料，防止把舊 ACK 或等待期間的 READY 當成新回覆。板子 ERR、逾時、錯配 ACK、重啟或序列錯誤則結束本次操作；不自動重送、不追加 torque-off。退出及 Ctrl+C 也沒有自動動作。

## 7. 操作範例與 demo

### 7.1 設定目標後明確啟用

下列數值僅示範通訊，不是 A/B/C 座標或已校正姿態。每筆命令等待對應回覆，位置更新時機由主機判斷。

```text
TX -> AX,arm1,512,512,512,512
RX <- OK,AX,arm1
TX -> TORQUE,arm1,1
RX <- OK,TORQUE,arm1,1
TX -> AX,arm1,520,512,512,512
RX <- OK,AX,arm1
```

第一組收發設定四軸目標；第二組開啟手臂 1 的施力；第三組只把 j1 目標改成 520，其餘仍送出 512。每組 OK 都是本地傳送成功，不是實際完成。ROS 必須依另外定義的完成判斷決定何時做下一個任務。

若要手動標示動作中，可輸入 `LED,arm1,MOVING`；自行確認停止後，輸入 `LED,arm1,STOPPED`。這兩筆成功時不會列印收發紀錄，但仍等待並核對回覆。需要卸力時明確送 `TORQUE,arm1,0`；卸力與設定紅燈是兩件獨立的操作。

兩臂任務可並行，但同一序列埠只允許一筆待回覆命令；兩臂 AX 逐筆交錯發送，不保證同步起動。韌體不儲存點列或執行定時器排程。

### 7.2 demo 示範會做什麼

demo 確切流程：PING；依選定手臂送四軸 512 的 AX、TORQUE,1；再依 j1=512→520→512，其他三軸=512，交替發送 AX。每筆 AX 後等 0.3 秒，每筆 TORQUE,1 後等 1 秒；這些是主機等待，不是到位證據。有 --arm 只測該臂，沒有則測兩臂。

demo 使用前需確認數值與現有 torque 狀態適合機構。demo 不先卸力，不在結束時卸力，不回傳物理完成事件。Return 應由 ROS 發送它所設定的姿態，不由韌體提供 HOME 替代。

本版無當前位置回讀，沒有等效 HOLD。重送最後目標仍是前往該目標，不能宣稱停在當前位置；TORQUE,0 是卸力，不是保持位置或硬體急停。

## 8. 程式結構與離線測試

### 8.1 原始碼分工

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

GPIO 是通用輸入／輸出腳位，RCC 是時鐘控制模組，UART 是序列收發介面。測試中的 fake／模擬層讓電腦代替實際硬體回應，用來檢查程式邏輯；不能證明真實馬達或燈號已正常運作。

### 8.2 執行測試

```powershell
python manual_position_terminal.py --self-test
./tests/run_tests.ps1 -Python python -Compiler gcc
```

第一行執行終端的 Python 自我測試；第二行執行 Python 與 C 測試套件。`-Python` 指定 Python 執行檔，`-Compiler` 指定電腦端 C 編譯器，不是 ARM 韌體編譯器。

使用 Zig 時以實際路徑取代範例：

```powershell
./tests/run_tests.ps1 -Python python -Compiler "C:\tools\zig\zig.exe" -CompilerArgs cc
```

Zig 也可提供 C 編譯功能；`-CompilerArgs cc` 表示使用它的 C 編譯入口。請將範例路徑換成實際安裝位置。

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

## 10. 常見問題

| 現象／疑問 | 說明與處理方向 |
|---|---|
| 不知道 COM4 是哪個裝置 | COM4 只是範例。確認控制板通訊線與驅動，再由系統的序列埠清單選擇實際編號 |
| 開啟序列埠失敗 | 確認埠名、線材與驅動；關閉 ROS、RoboPlus 或其他正在占用該埠的程式 |
| 沒有看到 READY | 板子可能在開埠前已啟動；終端仍會查 VERSION，但查到版本不代表板子剛重啟或馬達已卸力 |
| VERSION 回的不是 4 | 韌體與客戶端不相容，確認燒錄版本；不要忽略檢查或繼續重送舊命令 |
| 出現 ERR,NO_TARGET,arm1 | 該臂尚無有效目標。先處理中止原因並重新建立操作連線，再成功設定完整 AX，才送 TORQUE,1 |
| AX 回 OK，但手臂沒動 | AX 不自動啟用施力；也需檢查供電、馬達 ID、速率、接線與機構。OK 不能證明馬達收到或到位 |
| LED 成功時畫面沒有紀錄 | 正常設計。終端隱藏正常 LED 收發，仍核對 ACK；異常才顯示錯誤 |
| 紅燈亮了，為何手臂仍可能動？ | STOPPED 只改顯示；燈號不是停止控制或馬達感測結果 |
| 退出終端後會自動卸力嗎？ | 不會。q、Ctrl+C、斷線、示範結束及錯誤都不追加馬達或 LED 指令 |
| 可以拿 512 當回家位置嗎？ | 不能直接假設。512 只是原始位置值，Return／待機姿態必須依實際機構校正並由 ROS 設定 |
| 可以只靠這個專案執行整套搬運嗎？ | 還需要 ROS 視覺、運動規劃、任務協調、完成判斷與外部吸盤控制；本版只提供轉接韌體與對接契約 |

更多英文縮寫可參考[首頁的常用術語對照](../README.md#6-常用術語對照)。通訊細節以 [ROS 對接規格](ROS_CM530_INTERFACE_SPEC.txt)為準；實機燈色與馬達動作仍以 [VALIDATION](VALIDATION.md) 所列待驗收項目逐一確認。
