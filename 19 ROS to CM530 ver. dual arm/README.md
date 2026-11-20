# 第 19 版｜ROS to CM530 ver. dual arm

**主機協定：7｜文件核對：2026-11-20**

第 19 版與第 18 版的差別（2026-11-20）：

- `READ` 改為每顆馬達一次讀 36～41（位置、速度、負載），回覆多出四個 Present Load：`POS,arm1,j1..j4,l1..l4`。負載 0～1023 是大小，加 1024（bit 10）代表 CW 方向。讀取次數和第 18 版一樣（每臂四次），每顆只多 4 bytes。
- 新增 `COMP,arm1,m1..m4,s1..s4,p1..p4`：寫入 Compliance Margin（0～254）、Compliance Slope（2、4、8、16、32、64、128）和 Punch（32～1023），CW 與 CCW 寫成同一值。用兩筆 SYNC_WRITE（位址 26 寬 4、位址 48 寬 2）。任一值超出範圍時整筆拒絕，不送任何封包。不改目標資格，也不改 torque。
- 新增 `GET_COMP,arm1`：每顆讀 26～49，回覆 `COMP,arm1,m1..m4,s1..s4,p1..p4`；CW 和 CCW 不同時回 `ERR,DXL_ASYM,arm1,<id>`。
- 新增 `DIAG,arm1`：每顆讀 42～43，回覆 `DIAG,arm1,v1..v4,t1..t4`（電壓單位 0.1 V、溫度 °C）。
- 協定版本改成 7（`READY,7`、`VERSION,7`）。HOME、1 Mbps、Moving Speed 與第 18 版相同。

用途：2026-11-19 的晃動診斷（主 repo `dual_arm_ws/docs/evidence/diag_20261119/README.md`）懷疑手臂離開 HOME 時齒輪換邊，加上出廠 compliance（slope 32）偏軟。第 18 版不寫 compliance、也讀不到負載，所以沒辦法改參數比較，也看不到換邊點。

Compliance 和 Punch 是 AX-12A 的 RAM 暫存器，馬達斷電後回到出廠值（margin 1、slope 32、punch 32）。韌體開機時**不寫入**這些暫存器，所以沒送 `COMP` 時，行為和第 18 版完全一樣；由主機在每次啟用時寫入並用 `GET_COMP` 讀回確認。

[系統分工與目前進度](../README.md) · [搬運流程圖](../docs/images/dual-arm-flowchart.png) · [ROS 對接規格](ROS_CM530_INTERFACE_SPEC.md) · [驗證與測試總整理](../驗證與測試總整理.md) · [歷史驗收紀錄](VALIDATION.md)

## 控制設定

| 手臂 | 搬運區域 | j1、j2、j3、j4 馬達 ID | 目前原始碼 HOME（原始位置值） |
|---|---|---|---|
| arm1 | A → B | 17、3、2、15 | home_A：256、512、512、512 |
| arm2 | B → C | 12、1、8、16 | home_C：256、512、512、512 |

HOME 集中於 [APP/inc/arm_config.h](APP/inc/arm_config.h)。上述數值不是角度，也不代表已完成實機校正；板上設定須逐臂以 `GET_HOME` 確認。量測流程見 [HOME 校正資料收集](HOME_CALIBRATION_TEST.md)。

| 通訊 | 設定 |
|---|---|
| 主機 ↔ CM-530 | USART3、1 Mbps（36 MHz ÷ (16 × 2.25)，誤差 0%）、8N1、無流量控制；USB 轉接 latency timer 建議設 1 ms |
| CM-530 ↔ AX-12A | USART1、1 Mbps、DYNAMIXEL Protocol 1.0 |

馬達使用關節模式，Status Return Level 為 1 或 2。韌體不修改馬達 ID、EEPROM 或限位；Moving Speed、Compliance 和 Punch 都是 RAM 暫存器。

## 指令速查

以下 arm1 可替換為 arm2。每筆命令等待完整回覆後才送下一筆；位置參數範圍為 0～1023，不代表機構可安全走完整個範圍。命令可用 LF／CR／CRLF 結尾，回覆以 CRLF 結尾。

| 命令 | 成功回覆 | 用途 |
|---|---|---|
| `PING` | `PONG` | 通訊確認 |
| `VERSION` | `VERSION,7` | 協定核對 |
| `AX,arm1,512,512,512,512` | `OK,AX,arm1` | 四軸目標，預設速度 100 |
| `AX,arm1,512,512,512,512,60,60,60,60` | `OK,AX,arm1` | 四軸目標加四個 Moving Speed（1～1023） |
| `HOME,arm1` | `OK,HOME,arm1` | 寫入該臂 HOME |
| `GET_HOME,arm1` | `HOME,arm1,256,512,512,512` | 查詢 HOME 設定 |
| `READ,arm1` | `POS,arm1,j1,j2,j3,j4,l1,l2,l3,l4` | 回讀四軸位置與負載 |
| `HOLD,arm1` | `OK,HOLD,arm1` | 全部回讀成功後寫為保持目標 |
| `TORQUE,arm1,1` | `OK,TORQUE,arm1,1` | 啟用施力，需先有有效目標 |
| `TORQUE,arm1,0` | `OK,TORQUE,arm1,0` | 卸力並清除目標資格 |
| `COMP,arm1,1,1,1,1,32,16,16,32,32,32,32,32` | `OK,COMP,arm1` | 寫入四軸 margin、slope、punch |
| `GET_COMP,arm1` | `COMP,arm1,1,1,1,1,32,16,16,32,32,32,32,32` | 從馬達讀回 margin、slope、punch |
| `DIAG,arm1` | `DIAG,arm1,120,121,120,119,34,36,35,33` | 電壓（0.1 V）與溫度（°C） |
| `LED,arm1,MOVING` | `OK,LED,arm1,MOVING` | 綠亮紅滅 |
| `LED,arm1,STOPPED` | `OK,LED,arm1,STOPPED` | 紅亮綠滅 |

**操作要點：**

- AX／HOME／HOLD 不自動啟用 torque；若 torque 已啟用，寫入目標即可造成動作。ACK 只代表本地處理或傳送成功，到位由 ROS 判斷。
- `OK,COMP` 只代表兩筆封包送出，不代表馬達收到；一定要用 `GET_COMP` 讀回比對。第一筆成功、第二筆失敗時，馬達的 margin／slope 已改、punch 沒改，也是回 `ERR,DXL_TX`。
- Slope 越小越硬（目標附近出力越大）；Punch 越大，最小出力越大，太大容易抖動、發熱。改值時一次只改一項，並用 `DIAG` 看溫度。
- READ 四軸依序取樣；任一讀取失敗即回報錯誤，不回傳部分資料。HOLD 讀／寫失敗會清除該臂目標資格。
- HOLD 有取樣及傳輸延遲，不是硬體急停；torque 關閉時 HOLD 不會重新啟用。
- 開機雙紅待命；LED 由 ROS 明確設定，STOPPED 不會停止馬達。
- 錯誤、退出與斷線不自動重送、卸力或回 HOME。BEGIN／PT／END／STOP 不支援。

燈號：arm1 為 MANAGE/PB13 綠、PROGRAM/PB14 紅；arm2 為 TX/PC14 綠、RX/PC15 紅。

## ROS 執行順序

1. 韌體開機嘗試送出雙臂 torque-off，兩筆傳送成功才輸出 `READY,7`；此訊息不證明馬達實際已卸力。初始化失敗輸出 `ERR,INIT_FAILED`，鎖定 AX／TORQUE／HOME／READ／HOLD／COMP／GET_COMP／DIAG 至重啟；PING／VERSION／LED／GET_HOME 仍可用。
2. 主機核對 `VERSION,7`、`PONG`，查詢 `GET_HOME`。重新開埠不代表馬達已卸力。
3. 主機（`cm530_hardware` 驅動）有設定檔時，在 torque 之前送 `COMP`，再用 `GET_COMP` 讀回比對，不一致就不啟用。
4. 確認路徑可直接回 HOME 時，送 `HOME,arm1`，等 OK 後送 `TORQUE,arm1,1`；需要避障則由 ROS 規劃 AX 點列。
5. ROS 依軌跡時間逐點送 AX；終點以 READ 檢查位置誤差、穩定次數及期限。
6. 異常停止新軌跡，要求 HOLD 並保留 B 區鎖定；重新確認姿態、持物與各區狀態後再續行。

回讀錯誤包含 `DXL_TX`、`DXL_RX`、`DXL_TIMEOUT`、`DXL_CORRUPT`、`DXL_MOTOR`、`DXL_RANGE`、`DXL_ASYM`，附手臂與馬達 ID；每顆接收期限 50 ms、無重試。完整格式見 [對接規格](ROS_CM530_INTERFACE_SPEC.md)。

## 手動測試

在本資料夾執行，COM4 改成實際埠；ROS 與終端不可同時占用連線。

```powershell
python -m pip install -r requirements.txt
python manual_position_terminal.py --port COM4 --arm arm1
```

輸入 `?` 查指令、`q` 離開。指定 --arm 後可輸入四數字快捷 AX。`demo` 先寫四軸 512 並啟用 torque，再讓 j1 執行 512→520→512；這不是 HOME 流程。使用前確認所有範例目標與路徑適合機構。

## 建置與驗證

需 ARM 工具鏈及 make；C 主機測試使用 GCC，或改用 `-Compiler <zig.exe路徑> -CompilerArgs cc`。

```powershell
make -B TCHAIN_PREFIX=arm-none-eabi- "COMPILE_OPTS=-mcpu=cortex-m3 -mthumb -Wall -g -Os -fno-common -fno-strict-aliasing -Wno-error=implicit-function-declaration -Wno-error=incompatible-pointer-types -Wno-error=int-conversion" CM530.hex CM530.bin
python tests/verify_firmware.py
python manual_position_terminal.py --self-test
./tests/run_tests.ps1 -Python python -Compiler gcc
```

燒錄檔：[CM530.hex](CM530.hex)／[CM530.bin](CM530.bin)，Flash 起點 **0x08003000**，使用既有 CM-530 bootloader 流程。修改韌體後需重建、驗證並更新 [SHA256SUMS](SHA256SUMS)。

2026-11-20 第 19 版：ARM 完整建置（arm-none-eabi-gcc 14.2）與 `verify_firmware.py` 通過，雜湊已寫入 SHA256SUMS；C harness（新增 COMP／GET_COMP／DIAG／負載測試）在 WSL gcc 下通過，ASan／UBSan 再跑一次也通過；Python 27 項通過。**第 19 版尚未燒錄；COMP 寫入、讀回、負載與 DIAG 的實機驗收都還沒做**。最新檢查集中於 [驗證與測試總整理](../驗證與測試總整理.md)，歷史限制見 [VALIDATION.md](VALIDATION.md)。
