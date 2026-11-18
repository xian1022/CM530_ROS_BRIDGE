# 第 18 版｜ROS to CM530 ver. dual arm

**主機協定：6｜文件核對：2026-11-18**

第 18 版與第 17 版的差別（2026-11-18）：

- 每次寫入目標都同時寫入 AX-12A 的 Moving Speed（位址 32）。AX 可以多帶四個速度（1～1023）；不帶速度時，以及 HOME、HOLD，一律使用預設速度 100（約 67°/s）。速度 0 會被拒絕，因為 AX-12A 把 0 當成「不控制速度、全速」。
- 主機連線改成 **1 Mbps**（第 17 版是 57600）。
- 協定版本改成 6（`READY,6`、`VERSION,6`）。
- HOME 改成板子 2026-11-13 實際回覆的 `GET_HOME` 值：兩臂都是 256、512、512、512。

第 17 版不寫 Moving Speed。速度暫存器開機後的值不明，若是 0，主機每送一小步，馬達就全速走完再停下來等下一步，手臂因此卡頓、前後擺動。

[系統分工與目前進度](../README.md) · [搬運流程圖](../docs/images/dual-arm-flowchart.png) · [ROS 對接規格](ROS_CM530_INTERFACE_SPEC.md) · [驗證與測試總整理](../驗證與測試總整理.md) · [歷史驗收紀錄](VALIDATION.md)

## 控制設定

| 手臂 | 搬運區域 | j1、j2、j3、j4 馬達 ID | 目前原始碼 HOME（原始位置值） |
|---|---|---|---|
| arm1 | A → B | 17、3、2、15 | home_A：256、512、512、512 |
| arm2 | B → C | 12、1、8、16 | home_C：256、512、512、512 |

HOME 集中於 [APP/inc/arm_config.h](APP/inc/arm_config.h)。上述數值不是角度，也不代表已完成實機校正；板上設定須逐臂以 `GET_HOME` 確認。量測流程見 [HOME 校正資料收集](HOME_CALIBRATION_TEST.md)，確認數值後修改 header、重建、驗證並燒錄。需避障的回程由 ROS 取得 GET_HOME 值後規劃，透過 AX 逐點執行。

| 通訊 | 設定 |
|---|---|
| 主機 ↔ CM-530 | USART3、1 Mbps（36 MHz ÷ (16 × 2.25)，誤差 0%）、8N1、無流量控制；USB 轉接 latency timer 建議設 1 ms |
| CM-530 ↔ AX-12A | USART1、1 Mbps、DYNAMIXEL Protocol 1.0 |

馬達使用關節模式，Status Return Level 為 1 或 2。韌體不修改馬達 ID、EEPROM 或限位；Moving Speed 是 RAM 暫存器，每次寫目標時一起寫入。

## 指令速查

以下 arm1 可替換為 arm2。每筆命令等待完整回覆後才送下一筆；位置參數範圍為 0～1023，不代表機構可安全走完整個範圍。命令可用 LF／CR／CRLF 結尾，回覆以 CRLF 結尾。

| 命令 | 成功回覆 | 用途 |
|---|---|---|
| `PING` | `PONG` | 通訊確認 |
| `VERSION` | `VERSION,6` | 協定核對 |
| `AX,arm1,512,512,512,512` | `OK,AX,arm1` | 四軸目標，預設速度 100 |
| `AX,arm1,512,512,512,512,60,60,60,60` | `OK,AX,arm1` | 四軸目標加四個 Moving Speed（1～1023） |
| `HOME,arm1` | `OK,HOME,arm1` | 寫入該臂 HOME |
| `GET_HOME,arm1` | `HOME,arm1,256,512,512,512` | 查詢 HOME 設定 |
| `READ,arm1` | `POS,arm1,j1,j2,j3,j4` | 回讀四軸位置 |
| `HOLD,arm1` | `OK,HOLD,arm1` | 全部回讀成功後寫為保持目標 |
| `TORQUE,arm1,1` | `OK,TORQUE,arm1,1` | 啟用施力，需先有有效目標 |
| `TORQUE,arm1,0` | `OK,TORQUE,arm1,0` | 卸力並清除目標資格 |
| `LED,arm1,MOVING` | `OK,LED,arm1,MOVING` | 綠亮紅滅 |
| `LED,arm1,STOPPED` | `OK,LED,arm1,STOPPED` | 紅亮綠滅 |

`GET_HOME,arm2` 對應目前原始碼的回覆為 `HOME,arm2,256,512,512,512`；板上若是舊映像，數值可能不同。表中的 AX 512 是格式範例，不是目前 HOME。

**操作要點：**

- AX／HOME／HOLD 不自動啟用 torque；若 torque 已啟用，寫入目標即可造成動作。ACK 只代表本地處理或傳送成功，到位由 ROS 判斷。
- READ 四軸依序取樣；任一讀取失敗即回報錯誤，不回傳部分資料。HOLD 讀／寫失敗會清除該臂目標資格。
- HOLD 有取樣及傳輸延遲，不是硬體急停；torque 關閉時 HOLD 不會重新啟用。
- 開機雙紅待命；LED 由 ROS 明確設定，STOPPED 不會停止馬達。
- 錯誤、退出與斷線不自動重送、卸力或回 HOME。BEGIN／PT／END／STOP 不支援。

燈號：arm1 為 MANAGE/PB13 綠、PROGRAM/PB14 紅；arm2 為 TX/PC14 綠、RX/PC15 紅。

## ROS 執行順序

1. 韌體開機嘗試送出雙臂 torque-off，兩筆傳送成功才輸出 `READY,6`；此訊息不證明馬達實際已卸力。初始化失敗輸出 `ERR,INIT_FAILED`，鎖定 AX／TORQUE／HOME／READ／HOLD 至重啟；PING／VERSION／LED／GET_HOME 仍可用。
2. 主機核對 `VERSION,6`、`PONG`，查詢 `GET_HOME`。重新開埠不代表馬達已卸力。
3. 確認路徑可直接回 HOME 時，送 `HOME,arm1`，等 OK 後送 `TORQUE,arm1,1`（另一臂改用 arm2）；需要避障則由 ROS 規劃 AX 點列。
4. ROS 依軌跡時間逐點送 AX；終點以 READ 檢查位置誤差、穩定次數及期限。
5. 配合視覺確認取放及離區，再更新 B 區狀態與釋放使用權。
6. 異常停止新軌跡，要求 HOLD 並保留 B 區鎖定；重新確認姿態、持物與各區狀態後再續行。

回讀錯誤包含 `DXL_TX`、`DXL_RX`、`DXL_TIMEOUT`、`DXL_CORRUPT`、`DXL_MOTOR`、`DXL_RANGE`，附手臂與馬達 ID；每顆接收期限 50 ms、無重試。完整格式見 [對接規格](ROS_CM530_INTERFACE_SPEC.md)。

## 手動測試

在本資料夾執行，COM4 改成實際埠；ROS 與終端不可同時占用連線。

```powershell
python -m pip install -r requirements.txt
python manual_position_terminal.py --port COM4 --arm arm1
```

輸入 `?` 查指令、`q` 離開。指定 --arm 後可輸入四數字快捷 AX。正常 LED 收發不顯示，但仍核對回覆。`demo` 先寫四軸 512 並啟用 torque，再讓 j1 執行 512→520→512，其餘三軸維持 512；這不是 HOME 流程。指定 `--arm` 時只操作該臂，未指定則交替操作兩臂；結束不卸力，使用前確認所有範例目標與路徑適合機構。

## 建置與驗證

需 ARM 工具鏈及 make；C 主機測試使用 GCC，或改用 `-Compiler <zig.exe路徑> -CompilerArgs cc`。

```powershell
make -B TCHAIN_PREFIX=arm-none-eabi- "COMPILE_OPTS=-mcpu=cortex-m3 -mthumb -Wall -g -Os -fno-common -fno-strict-aliasing -Wno-error=implicit-function-declaration -Wno-error=incompatible-pointer-types -Wno-error=int-conversion" CM530.hex CM530.bin
python tests/verify_firmware.py
python manual_position_terminal.py --self-test
./tests/run_tests.ps1 -Python python -Compiler gcc
```

燒錄檔：[CM530.hex](CM530.hex)／[CM530.bin](CM530.bin)，Flash 起點 **0x08003000**，使用既有 CM-530 bootloader 流程。修改韌體後需重建、驗證並更新 [SHA256SUMS](SHA256SUMS)。

2026-11-18 第 18 版：ARM 完整建置（arm-none-eabi-gcc 14.2）與 `verify_firmware.py` 通過，雜湊已寫入 SHA256SUMS；C harness（含新增的速度測試）在 WSL gcc 下通過，另以 ASan／UBSan 再跑一次也通過；Python 26 項通過。**第 18 版尚未燒錄，1 Mbps 連線、速度控制與 HOME 的實機驗收都還沒做**。最新檢查集中於 [驗證與測試總整理](../驗證與測試總整理.md)，歷史限制見 [VALIDATION.md](VALIDATION.md)。
