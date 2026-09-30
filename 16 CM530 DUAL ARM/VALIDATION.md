# 第 16 版／主機序列協定 4 驗證紀錄

更新與驗證日期：2026-09-30。此次為離線修改、測試與建置，**未連接、燒錄或操作 CM530／馬達**。

## 1. 修改範圍

在協定 3 的直接轉接韌體上，新增 ROS 明確通知的 LED 命令與終端靜默顯示。主機識別改為 READY,4／VERSION,4；馬達仍使用 Dynamixel Protocol 1.0。

- arm1：MANAGE PB13 綠、PROGRAM PB14 紅。
- arm2：TX PC14 綠、RX PC15 紅。
- 應用韌體初始化為雙紅待命，不表示量測到停止。
- LED 只控制 GPIO，不發馬達封包、不改目標資格；初始化失敗時仍可設定。
- 正常 LED TX／ACK 不印到終端，錯誤／超時／錯配／重啟仍顯示並停止。
- AX／TORQUE、demo、退出、錯誤及斷線都不自動改燈。
- 原始 01 LED 與其他歷史資料夾沒有修改。

## 2. 測試與結果

先新增測試再執行舊版：Python 22 項中出現 4 項失敗與 3 項錯誤；C LED 組出現 29 個失敗檢查，辨識出舊版本、缺少 LED 命令／初始化及缺少終端靜默核對。更新程式後通過，另補上 GPIO 適配及 CLI 異常停止測試。

| 項目 | 結果 |
|---|---|
| Python 終端／CLI | 24/24 通過 |
| C 正式 bridge＋AX12＋SDK，模擬實體 HAL／LED 回呼 | 全部通過 |
| 正式 arm_led.c，模擬 GPIO／RCC 函式 | 全部通過 |
| 兩組主機 C 編譯 -Wall -Wextra -Werror | 通過，無警告 |
| ARM 完整建置 | 通過，保留 10 個既有 SDK 警告 |
| ELF／HEX／BIN 一致性、向量、ID、協定 4 標記 | 通過 |
| 實機燈色、電氣行為、馬達動作及 ROS 流程 | 待驗證 |

### C 命令與 GPIO 覆蓋

- 開機雙紅與兩筆 torque-off；任一筆初始化 TX 失敗仍嘗試另一臂，AX／TORQUE 鎖定但 LED 可使用。
- 每臂 MOVING／STOPPED、大小寫與空白、重複設定、非法手臂／欄位／狀態；兩臂獨立。
- LED 不發馬達封包、不建立或清除目標資格；馬達命令及其失敗不改燈。
- 直接測試正式 GPIO 適配層：GPIOB/C 時鐘、輸出前預載熄滅、低電位亮、先熄滅再點亮、同臂紅綠互斥。
- 對照 PB13/PB14、PC14/PC15，確認 PLAY/AUX 熄滅、POWER 與非 LED 腳位維持原值。
- 保留每筆 runtime 馬達封包只含同一臂四個 ID、只寫位址 30／24、無 READ 的檢查。
- 保留目標資格、torque-off、初始化失敗、非法數值／溢位、UART 丟行與接收恢復測試。
- 通用 SDK 接收測試保留為 SDK-only，與主機命令不發 READ 分開；不代表新增對外回讀功能。

### Python 終端覆蓋

- LED 輸入正規化與完整狀態 ACK。
- 四種 LED 組合確實送出命令、核對 ACK，但正常收發輸出為空。
- 錯手臂／錯狀態 ACK、ERR 與 READY 仍顯示異常回覆。
- CLI 級測試確認 LED 失敗、超時或部分回覆會印 Session stopped，且不讀取／送出下一筆動作。
- 預先存在的 READY、ERR、舊 LED ACK 或部分訊息在送出前被拒絕。
- 一般命令、READY、VERSION 仍可見。
- 保留 demo 不自動送 LED、馬達命令精確 ACK、舊版拒絕、CRLF、殘留回覆與異常中止測試。

執行環境：Windows PowerShell、Python 3.14、Zig 0.13.0 的 Clang 主機編譯器。無需 pyserial 或控制板即可測試：

```powershell
./tests/run_tests.ps1 -Python <python.exe路徑> -Compiler <zig.exe路徑> -CompilerArgs cc
```

也可使用主機 GCC：`./tests/run_tests.ps1 -Python python -Compiler gcc`。GPIO fake header 僅用於主機 LED 測試；正式 ARM 建置使用原有 STM32 SDK。

## 3. ARM 建置與映像

工具鏈：Arm GNU Toolchain 14.2 rel1。

```powershell
make -B TCHAIN_PREFIX=arm-none-eabi- "COMPILE_OPTS=-mcpu=cortex-m3 -mthumb -Wall -g -Os -fno-common -fno-strict-aliasing -Wno-error=implicit-function-declaration -Wno-error=incompatible-pointer-types -Wno-error=int-conversion" CM530.hex CM530.bin
python tests/verify_firmware.py
```

| 產物／區段 | 大小 |
|---|---:|
| CM530.elf | 139,864 bytes |
| CM530.hex | 33,274 bytes |
| CM530.bin | 11,808 bytes |
| ELF text | 11,792 bytes |
| ELF data | 16 bytes |
| ELF bss（含 linker 預留） | 1,020 bytes |

Flash 起點 0x08003000；初始 SP 0x20010000；Thumb Reset_Handler 0x08005C61。

映像檢查先對舊協定 3 產物執行，確認因缺少 READY,4 失敗；重建後驗證通過：
ELF 可載入資料／HEX checksum／BIN 逐 byte 一致、記憶體範圍、向量、ID 表、
READY,4／VERSION,4／LED／MOVING／STOPPED，以及舊協定識別與已移除命令標記不存在。

[SHA256SUMS](SHA256SUMS) 涵蓋同次 ELF／HEX／BIN。make 不會自行更新雜湊；本次已依實際產物更新。ELF 與中間檔依既有 ignore 規則留在本機。

完整 ARM 建置仍有 10 個既有 SDK 警告：dxl_hal.c 的 8 個硬體函式隱含宣告，以及 STM32 vector 的 2 個型別警告。此次新 LED 程式未新增編譯警告。完整紀錄在忽略的本機 tests/build.log。

文件及交付交叉檢查通過：13 個本地連結、15 組成功 TX／ACK 範例、三份產物 SHA-256，以及 git diff --check。

## 4. 審查與驗證限制

已自行對照核准需求、正式程式、測試、三份主要文件及燈位映射。獨立審查工具因用量限制而失敗，**本次協定 4 不宣稱已完成獨立審查**；先前協定 3 的審查不作為此次 LED 改動的審查證據。

主機測試不能驗證實體 LED 色彩、亮度或 GPIO 電氣狀態。顏色配置依使用者提供資訊，最終仍需逐燈實機驗收。開機雙紅只涵蓋應用韌體初始化完成後，不保證 bootloader 的顯示。

終端可拒絕發送前已收到的異常資料；檢查後才到達的重啟仍可能與傳送競態。協定沒有 request ID，也不提供跨重啟 exactly-once 保證。SDK-only 模擬仍未完整覆蓋殘留馬達回覆後再讀取的恢復路徑，此路徑不由主機命令使用。

## 5. 實機驗收——全部待驗證

1. 燒錄本次映像，核對 READY,4／VERSION,4／PONG。
2. 應用韌體啟動後確認左右紅燈亮、綠燈滅；PLAY/AUX 滅，POWER 維持原狀。
3. 依序送四種 LED 命令，確認 MANAGE 綠、PROGRAM 紅、TX 綠、RX 紅，以及同臂互斥、雙臂獨立。
4. 確認 LED 命令不帶動馬達、不更動 torque，且不影響主機或馬達通訊。
5. 確認正常 LED 收發不印文字；異常仍顯示且停止後續發送。
6. 使用已確認適合機構的目標，驗收 AX → TORQUE,1、小幅 AX、TORQUE,0 及雙臂交替；這些操作不自動改燈。
7. 確認重啟回雙紅；退出、斷線或停止送新目標不自動改燈，也不能視為機械已停止。
8. 由 ROS 驗證動作前明確通知 MOVING，依實際完成條件確認停止後通知 STOPPED。

ROS 節點、四狀態任務、B 站互斥、C 站八格、吸盤協調與到位判斷均未在本次實作；外部整合要求見 [ROS 對接規格](ROS_CM530_INTERFACE_SPEC.txt)。
