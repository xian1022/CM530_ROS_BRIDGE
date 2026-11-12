# 第 17 版／協定 5 驗證紀錄

## 2026-11-13：HOME 設定更新

home_A、home_C 原始碼及 C 測試預期均更新為 256,512,512,512，並同步使用文件。Python 終端 25/25 測試通過；git diff --check 通過。ARM 工具無法啟動（系統 DLL 重新定位不正確），未完成韌體重建，現有 HEX/BIN 與 SHA256SUMS 仍屬前次設定。主機 gcc/clang 與原 Zig 路徑不可用，未執行 C 主機測試或 ASan；未燒錄或操作實機。以下為歷史驗證紀錄。


## 2026-10-25：HOME 暫定觀察姿態更新

home_A 與 home_C 均改為 **512,205,512,205**。暫定 j1～j4 由下到上，兩臂須分別實機校正。以下均為離線驗證，未燒錄或驅動手臂；下方 2026-10-07 紀錄中的全 512 為歷史設定。

| 項目 | 本次結果 |
|---|---|
| Python 終端與 CLI | 25/25 通過 |
| C bridge/AX12/SDK 與 LED GPIO 模擬測試 | 全通過，gnu89、Wall/Wextra/Werror |
| 兩臂 HOME/GET_HOME | 回覆與 SYNC_WRITE 均核對 512,205,512,205；不自動 torque-on |
| ARM 完整重建 | 成功，保留原有 10 個 SDK 警告 |
| ELF/HEX/BIN 一致性、HEX checksum、向量、ID 與協定標記 | 通過 |
| 映像 HOME 常數 | nm 確認 home_A=0x0800600c、home_C=0x08006014，各 8 bytes；BIN 按 little-endian 四個 unsigned short 解碼均等於新設定 |
| SHA256SUMS | 已更新 ELF/HEX/BIN 三筆雜湊；BIN 仍為 12612 bytes |
| 實機視野、限位、持物遮擋與回程避障 | 尚未驗證 |

沿用下方 ARM 建置與映像驗證命令。本次 Python 使用實際執行檔避開無法啟動的 WindowsApps 別名；舊 Zig 暫存安裝缺少支援檔案，於獨立暫存位置安裝相同 0.13.0 版本後執行：

```powershell
./tests/run_tests.ps1 -Python "C:/Users/39165/AppData/Local/Python/pythoncore-3.14-64/python.exe" -Compiler "C:/Users/39165/AppData/Local/Temp/cm530-home-20261025-tools/ziglang/zig.exe" -CompilerArgs cc
```

ARM 建置與 Zig 測試需在沙箱外執行以避免工具啟動／快取重新命名受限。本次未執行 ASan 或 fuzzing；本次差異審查見 DIFFERENTIAL_REVIEW_REPORT.md 的 2026-10-25 段落。

## 2026-10-07 原始驗證紀錄

日期：2026-10-07。以下為離線測試，**未連接、燒錄或操作控制板及馬達**。

## 測試結果

| 項目 | 結果 |
|---|---|
| Python 終端與 CLI | 25/25 通過 |
| C 正式 bridge + AX12 + SDK，模擬 HAL | 全通過 |
| 正式 LED adapter，模擬 GPIO/RCC | 全通過 |
| 主機 C -std=gnu89 -Wall -Wextra -Werror | 通過 |
| ARM -B 完整建置 | 成功，10 個既有 SDK 警告 |
| ELF/HEX/BIN 一致性、HEX checksum、向量、ID 與協定標記 | 通過 |
| 實際 UART、馬達、HOLD 行為、ROS 搬運流程 | 待實機驗收 |

新增測試先對舊實作執行：C 新功能組失敗 323 個檢查；Python 25 個測試中 3 failures、2 errors。實作後全數通過。

覆蓋 HOME/GET_HOME 的手臂映射與 512 預設、無自動 torque、每臂資格隔離、READ 順序、HOLD 先讀完四軸才寫入、HOLD 讀/寫失敗清除資格、READ 不改資格、初始化失敗鎖定及另一臂仍可操作。

故障模擬包括無回覆、錯 ID、短/長/零長度、checksum 錯誤、馬達錯誤位元、超範圍位置、截斷封包、HAL 接收失敗及傳送失敗。保留行溢位、NUL、整數溢位、SDK 分段接收、LED 隔離及 CLI 停止操作回歸測試。

映像驗證的舊 PT 字串檢查已加 NUL 前邊界，避免誤中新增 DXL_CORRUPT 字尾；不支援 PT 的實際行為由 parser 測試驗證。

## 工具與重現

ARM：Arm GNU Toolchain 14.2 rel1，arm-none-eabi-gcc；make 為既有 WinARM 工具。
主機 C：Zig 0.13.0 的 cc；Python 3.14。

```powershell
./tests/run_tests.ps1 -Python python -Compiler "C:\Users\39165\AppData\Local\Temp\cm530-v17-host-tools\ziglang\zig.exe" -CompilerArgs cc
make -B TCHAIN_PREFIX=arm-none-eabi- "COMPILE_OPTS=-mcpu=cortex-m3 -mthumb -Wall -g -Os -fno-common -fno-strict-aliasing -Wno-error=implicit-function-declaration -Wno-error=incompatible-pointer-types -Wno-error=int-conversion" CM530.hex CM530.bin
python tests/verify_firmware.py
```

Zig 路徑是本機暫存測試工具位置；換電腦請改為當地 Zig/GCC。產品只需既有 pyserial 終端依賴，不依賴 Zig。

既有警告：dxl_hal.c 的 8 個平台函式隱含宣告；原廠 vector 的 stack 項目及 RAM boot workaround 共 2 個警告。新 bridge/AX12 邏輯的主機編譯以 -Werror 通過。保留舊 SDK 相容旗標，未將這些警告當作實機驗收結果。

## 映像

Flash 起点 0x08003000、stack 0x20010000。BIN 12612 bytes；ELF/HEX/BIN 為同次完整建置，精確雜湊見 SHA256SUMS。ELF 含路徑與除錯資訊，換建置路徑可能改變雜湊。

HEX/BIN 可供既有 CM-530 bootloader 載入；本次沒有自動燒錄。ELF 與 build.log 本機保留且被 git ignore。修改任何韌體或 HOME header 後，必須完整重建、重新驗證並更新 SHA256SUMS。

## 實機驗收清單（尚未執行）

- [ ] 開機無位置寫入、不自動回 HOME；兩臂 torque-off 與雙紅確認。
- [ ] VERSION,5／PING 握手，兩臂 ID、1Mbps、關節模式及回覆等級確認。
- [ ] GET_HOME 值與 header 一致；分別 HOME、TORQUE,1、READ，確認方向及四軸位置。
- [ ] ROS 逐點 AX 與終點 READ；容差、穩定次數、採樣率及到位期限校正。
- [ ] 運動中 HOLD 與後續 READ；量測回讀延遲、保持誤差、兩臂序列處理影響。
- [ ] 單顆馬達無回覆時正確回報 ID；HOLD 不寫部分目標，另一臂仍可通訊。
- [ ] 通訊中斷／主機重連／板子重啟可被 ROS 辨識，不重送未知結果的動作。
- [ ] B 區未確認離區時不放行；HOLD 失敗保持 B 鎖定並要求重新確認。
- [ ] C 全滿停止新增搬運、保留 B 餘料，清臺後人工確認並重掃狀態。

B/C 及吸盤項目是系統整合驗收，並非本韌體自動提供的功能。

## 最終審查與額外檢查

獨立審查未確認新的執行缺陷；已更新舊驗證文件/終端 banner 並加入 arm_config.h 增量建置依賴。修改 HOME header 後 make dry-run 會重編 bridge.o。

依新個人指示另套用 Trail of Bits C API 與差異審查，報告見 DIFFERENTIAL_REVIEW_REPORT.md。AddressSanitizer 嘗試因 Windows Zig 缺少 __asan_* runtime 連結失敗，WSL 無 gcc/clang，故 ASan **未執行成功**；未進行 coverage-guided fuzzing 或量測覆蓋率。
