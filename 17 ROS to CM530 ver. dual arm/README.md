# 第 17 版｜ROS to CM530 ver. dual arm

**主機協定：5｜更新：2026-10-07**

[系統分工與目前進度](../README.md) · [搬運流程圖](../docs/images/dual-arm-flowchart.png) · [ROS 對接規格](ROS_CM530_INTERFACE_SPEC.md) · [驗收紀錄](VALIDATION.md)

## 控制設定

| 手臂 | 搬運區域 | j1、j2、j3、j4 馬達 ID | HOME 初值 |
|---|---|---|---|
| arm1 | A → B | 17、3、2、15 | home_A：512、512、512、512 |
| arm2 | B → C | 12、1、8、16 | home_C：512、512、512、512 |

HOME 集中於 [APP/inc/arm_config.h](APP/inc/arm_config.h)，校正後修改、重建並燒錄。需避障的回程由 ROS 取得 GET_HOME 值後規劃，透過 AX 逐點執行。

| 通訊 | 設定 |
|---|---|
| 主機 ↔ CM-530 | USART3、57600 baud、8N1、無流量控制 |
| CM-530 ↔ AX-12A | USART1、1 Mbps、DYNAMIXEL Protocol 1.0 |

馬達使用關節模式，Status Return Level 為 1 或 2。韌體不修改馬達 ID、EEPROM、速度或限位。

## 指令速查

以下 arm1 可替換為 arm2。每筆命令等待完整回覆後才送下一筆；位置範圍為 0～1023，回覆以 CRLF 結尾。

| 命令 | 成功回覆 | 用途 |
|---|---|---|
| `PING` | `PONG` | 通訊確認 |
| `VERSION` | `VERSION,5` | 協定核對 |
| `AX,arm1,512,512,512,512` | `OK,AX,arm1` | 四軸目標 |
| `HOME,arm1` | `OK,HOME,arm1` | 寫入該臂 HOME |
| `GET_HOME,arm1` | `HOME,arm1,512,512,512,512` | 查詢 HOME 設定 |
| `READ,arm1` | `POS,arm1,j1,j2,j3,j4` | 回讀四軸位置 |
| `HOLD,arm1` | `OK,HOLD,arm1` | 全部回讀成功後寫為保持目標 |
| `TORQUE,arm1,1` | `OK,TORQUE,arm1,1` | 啟用施力，需先有有效目標 |
| `TORQUE,arm1,0` | `OK,TORQUE,arm1,0` | 卸力並清除目標資格 |
| `LED,arm1,MOVING` | `OK,LED,arm1,MOVING` | 綠亮紅滅 |
| `LED,arm1,STOPPED` | `OK,LED,arm1,STOPPED` | 紅亮綠滅 |

**操作要點：**

- AX／HOME／HOLD 不自動啟用 torque；ACK 只代表本地處理或傳送成功，到位由 ROS 判斷。
- READ 四軸依序取樣；任一讀取失敗即回報錯誤，不回傳部分資料。HOLD 讀／寫失敗會清除該臂目標資格。
- HOLD 有取樣及傳輸延遲，不是硬體急停；torque 關閉時 HOLD 不會重新啟用。
- 開機雙紅待命；LED 由 ROS 明確設定，STOPPED 不會停止馬達。
- 錯誤、退出與斷線不自動重送、卸力或回 HOME。BEGIN／PT／END／STOP 不支援。

燈號：arm1 為 MANAGE/PB13 綠、PROGRAM/PB14 紅；arm2 為 TX/PC14 綠、RX/PC15 紅。

## ROS 執行順序

1. 開機送雙臂 torque-off，成功回覆 `READY,5`；初始化失敗鎖定馬達命令至重啟。
2. 主機核對 `VERSION,5`、`PONG`，查詢 `GET_HOME`。重新開埠不代表馬達已卸力。
3. 確認路徑可直接回 HOME 時，送 `HOME`，等 OK 後送 `TORQUE,1`；需要避障則由 ROS 規劃 AX 點列。
4. ROS 依軌跡時間逐點送 AX；終點以 READ 檢查位置誤差、穩定次數及期限。
5. 配合視覺確認取放及離區，再更新 B 區狀態與釋放使用權。
6. 異常停止新軌跡，要求 HOLD 並保留 B 區鎖定；重新確認姿態、持物與各區狀態後再續行。

回讀錯誤包含 `DXL_TIMEOUT`、`DXL_CORRUPT`、`DXL_MOTOR`、`DXL_RANGE`，附手臂與馬達 ID；每顆接收期限 50 ms、無重試。完整格式見 [對接規格](ROS_CM530_INTERFACE_SPEC.md)。

## 手動測試

在本資料夾執行，COM4 改成實際埠；ROS 與終端不可同時占用連線。

```powershell
python -m pip install -r requirements.txt
python manual_position_terminal.py --port COM4 --arm arm1
```

輸入 `?` 查指令、`q` 離開。指定 --arm 後可輸入四數字快捷 AX。正常 LED 收發不顯示，但仍核對回覆。`demo` 會執行 512→520→512 範例、啟用 torque 且結束不卸力，使用前確認機構允許。

## 建置與驗證

需 ARM 工具鏈及 make；C 主機測試使用 GCC，或改用 `-Compiler <zig.exe路徑> -CompilerArgs cc`。

```powershell
make -B TCHAIN_PREFIX=arm-none-eabi- "COMPILE_OPTS=-mcpu=cortex-m3 -mthumb -Wall -g -Os -fno-common -fno-strict-aliasing -Wno-error=implicit-function-declaration -Wno-error=incompatible-pointer-types -Wno-error=int-conversion" CM530.hex CM530.bin
python tests/verify_firmware.py
python manual_position_terminal.py --self-test
./tests/run_tests.ps1 -Python python -Compiler gcc
```

燒錄檔：[CM530.hex](CM530.hex)／[CM530.bin](CM530.bin)，Flash 起點 **0x08003000**，使用既有 CM-530 bootloader 流程。修改韌體後需重建、驗證並更新 [SHA256SUMS](SHA256SUMS)。

Python 25 項、C 邏輯／LED 測試及 ARM 映像檢查已離線通過；**第 17 版尚待燒錄與實機驗收**。測試限制及待驗項目見 [VALIDATION.md](VALIDATION.md)。
