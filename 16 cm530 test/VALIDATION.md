# 第 16 版驗證紀錄

日期：2026-09-24。已完成 ARM 編譯及檔案驗證；未連接或操作 CM530／馬達，未燒錄。

## 建置結果

工具鏈：Arm 官方 GNU Arm Embedded 10.3-2021.10，GCC 10.3.1。
下載來源：https://armkeil.blob.core.windows.net/developer/Files/downloads/gnu-rm/10.3-2021.10/gcc-arm-none-eabi-10.3-2021.10-win32.zip
下載檔 MD5：2bc8f0c4c4659f8259c8176223eeafc1，與官方 HTTP Content-MD5 相符。
工具鏈解壓到本機暫存目錄，未放入專案或改動全域 PATH。

完整重建命令：

```text
make -B TCHAIN_PREFIX=arm-none-eabi- CM530.hex CM530.bin
python tests/verify_firmware.py
```

Makefile 為新版工具鏈補上連結階段 Cortex-M3／Thumb 參數、-nostartfiles 與 Reset_Handler 入口；使用 SDK 的啟動程式。編譯加上 -fno-strict-aliasing，以相容舊 STM32 SDK 的指標轉型存取方式。

| 輸出 | 大小 |
|---|---:|
| CM530.hex | 35,524 bytes |
| CM530.bin | 12,608 bytes |
| ELF text | 12,592 bytes |
| ELF data | 16 bytes |
| ELF bss（含 linker 預留 stack 區） | 1,056 bytes |

程式起始位址：0x08003000。
初始 SP：0x20010000。
Thumb Reset_Handler：0x08005E21。
HEX 校驗碼、ELF 配置、HEX/BIN 內容一致性、重置向量及 A/B ID 表檢查全部通過。
產物 SHA-256 見 SHA256SUMS。

編譯仍保留原 SDK 警告：dxl_hal.c 平台函式缺少宣告，以及啟動向量表的舊型別寫法。無編譯或連結錯誤；原廠檔案未因消除警告而更動。

## 離線測試

- Python 3.8.10：manual_position_terminal.py --self-test，9 項測試通過。
- GCC 3.4.4（Cygwin，gnu89）：正式 bridge.c + 正式 dynamixel.c + 模擬 HAL 測試通過。
- tests/run_tests.ps1 完整跑完 Python 與 C 測試。
- 以複製前 SHA-256 比對第 15 版 77 個來源／設定／文件檔案，變更數為 0。
- 第 16 版的物件、SDK 靜態庫與韌體全部重新建置，未使用第 15 版二進位輸出。

C 測試檢查實際 SDK 封包的 header、長度、checksum、Goal Position 位址、四顆 ID 及位置 bytes；包含 A/B 分離、交錯軌跡、HOME/STOP、TX 失敗後狀態保留及錯誤輸入處理。
Python 測試包含 A/B 指令、--arm 快捷輸入設定、精確 ACK、錯誤手臂／序號、部分回覆逾時、重新啟動及 demo 失敗後停止發送。

## 實機待驗證

- CM530 實際燒錄與啟動。
- 八顆馬達實際 ID、baudrate、torque 與接線。
- A/B 個別小幅動作、交替軌跡與 ACK。
- UART 中斷層實際溢位後的恢復。
- HOME=512 是否符合兩臂機構；ACK 不代表實際到位。

先前因本機缺少 ARM 編譯器而建置失敗；本次已透過官方免安裝工具鏈完成。
