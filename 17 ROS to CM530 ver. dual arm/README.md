# 第 17 版 ROS to CM530 ver. dual arm

CM-530 雙臂轉接韌體，主機序列協定 **5**。以第 16 版為基礎，參考第 15 版 HOME 與原廠 09／10 讀寫範例，增加板內 HOME、實際位置回讀及 HOLD。

## 分工

ROS 2 負責三相機視覺、IK、MoveIt2 軌跡規劃、發送時間、到位判斷、arm1/arm2 state0～3、B 區互鎖及 C 區八格排列。CM-530 只執行底層命令；吸盤由 ESP32 控制。本資料夾不包含 ROS 任務節點。

兩臂可在各自區域並行，但共用一條序列連線及 DXL bus；主機一次只送一筆命令，收到完整回覆後才送下一筆。

| 手臂 | 用途 | j1、j2、j3、j4 馬達 ID | HOME |
|---|---|---|---|
| arm1 | A → B | 17、3、2、15 | home_A |
| arm2 | B → C | 12、1、8、16 | home_C |

主機：USART3、57600 baud、8N1、無流量控制；馬達：USART1、1 Mbps、DYNAMIXEL Protocol 1.0。沿用原廠 STM32 SDK、時鐘、中斷與 bootloader 位址。

### 雙臂協同搬運流程圖

![雙臂 A→B→C 搬運流程：HOME 待機、取放、B 區互鎖、C 區八格排列及異常復歸](../docs/images/dual-arm-flowchart.png)

[開啟完整解析度流程圖](../docs/images/dual-arm-flowchart.png)

流程圖為整套系統的工作分工：ROS 管理 state0～3、視覺、軌跡及 B 區授權；CM-530 執行 AX／HOME／READ／HOLD；ESP32 控制吸盤。指令 ACK 不等於到位，需回讀與視覺確認後才可釋放 B 區使用權。C 區全滿時停止新增搬運、保留 B 區餘料，清臺並人工確認後重新辨識各區狀態。

## HOME 設定

修改 [APP/inc/arm_config.h](APP/inc/arm_config.h)，完整重新建置並燒錄：

```c
static const unsigned short home_A[4] = {512, 512, 512, 512};
static const unsigned short home_C[4] = {512, 512, 512, 512};
```

數值為各關節原始位置 0～1023，順序見上表。512 是本次指定的初值，尚未做機構校正。HOME 不具原點感測校正或避障功能，需要避障時由 ROS 取得 GET_HOME 值、規劃路徑並逐點送 AX。

## 命令速查

所有 `<arm>` 均明確指定 arm1 或 arm2；回覆以 CRLF 結尾。

| 命令 | 回覆／用途 |
|---|---|
| `PING` | `PONG`，檢查主機通訊 |
| `VERSION` | `VERSION,5` |
| `AX,arm1,512,512,512,512` | `OK,AX,arm1`；寫四軸目標 |
| `HOME,arm1` | `OK,HOME,arm1`；寫 home_A 目標 |
| `HOME,arm2` | `OK,HOME,arm2`；寫 home_C 目標 |
| `GET_HOME,arm1` | `HOME,arm1,512,512,512,512`；讀取板內預設 |
| `READ,arm1` | `POS,arm1,j1,j2,j3,j4`；四軸實際回讀 |
| `HOLD,arm1` | `OK,HOLD,arm1`；讀完四軸後寫為保持目標 |
| `TORQUE,arm1,1` | `OK,TORQUE,arm1,1`；啟用施力，需先有有效目標 |
| `TORQUE,arm1,0` | `OK,TORQUE,arm1,0`；卸力並清除目標資格 |
| `LED,arm1,MOVING` | `OK,LED,arm1,MOVING`；綠亮紅滅 |
| `LED,arm1,STOPPED` | `OK,LED,arm1,STOPPED`；紅亮綠滅 |

AX、HOME、HOLD 都不自動啟用施力。ACK 只代表本地命令處理／傳送成功；不能當成到位、停止或離開 B 區的證据。READ 四軸依序取樣，不是同一瞬間的位置。HOLD 取樣期間手臂仍可能移動，屬軟體保持目標，不是硬體急停；torque 關閉時 HOLD 不會重新啟用。

LED 沿用第 16 版：arm1 MANAGE/PB13 綠與 PROGRAM/PB14 紅，arm2 TX/PC14 綠與 RX/PC15 紅，低電位亮。PLAY/AUX 熄滅，POWER 不變。STOPPED 只改燈，AX/HOME/HOLD/TORQUE 不自動改燈。

## ROS 使用順序

1. 开機雙臂送 torque-off、雙紅待命；成功輸出 READY,5，失敗輸出 ERR,INIT_FAILED 並鎖定馬達命令至重啟。
2. 主機開埠後核對 VERSION,5 及 PING。單純開埠不代表板子重啟或已卸力。
3. 各臂先 GET_HOME，再 HOME，等待 OK 後明確 TORQUE,1。若目前路徑不能直接回 HOME，ROS 先規劃，再依 AX 的目標／啟用順序執行。
4. ROS 依發送時間逐點下 AX，每點等待對應 ACK；不等待每個中間點到位。
5. 終點以 READ 回讀四軸；ROS 比較目標、容差、連續穩定樣本與總期限，再判斷到位。這些參數由 ROS 校正設定。
6. 取放結果由視覺／吸盤狀態確認；到位及離區確認完成後，ROS 才可更新 B 區物件狀態與釋放使用權。
7. 異常時停止新軌跡指令，分別送 HOLD，保留 B 區鎖定。HOLD 失敗不保證已停止，不能據此放行；恢復後重新確認姿態、持物與 A/B/C 狀態。

滿格停止新增搬運、B 區餘料保留、人工清臺確認續行都由 ROS 任務協調器執行。斷線、退出、逾時不自動追加卸力或 HOME。

## 錯誤與馬達回讀

READ/HOLD 第一次失敗即終止，不回傳部分資料或沿用舊位置。常見回覆：

```text
ERR,DXL_TIMEOUT,arm1,17
ERR,DXL_CORRUPT,arm2,12
ERR,DXL_MOTOR,arm1,3,4
ERR,DXL_RANGE,arm2,8
```

最後的 4 是馬達錯誤位元遮罩，不是位置。每顆馬達接收期限 50 ms、無重試；每次四軸讀取的等待上限約 4 × 50 ms，加傳輸／處理時間，主機預設命令 timeout 為 2 秒。READ 不改變目標資格；HOLD 讀取或寫入失敗會清除該臂資格。

馬達須設定為關節模式，Status Return Level 須為 1 或 2 才回應 READ。韌體不改 EEPROM、馬達 ID、速度或限位。詳見 [ROBOTIS AX-12A 控制表](https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/) 與 [ROS 對接規格](ROS_CM530_INTERFACE_SPEC.txt)。

## 手動終端與建置

在本資料夾的 PowerShell 執行：

```powershell
python -m pip install -r requirements.txt
python manual_position_terminal.py --port COM4 --arm arm1
```

COM4 換成實際埠。可輸入完整命令、`?`、`q`；指定 --arm 後可用四數字快捷 AX。正常 LED 收發不顯示，但仍核對 ACK；其他命令照常顯示。demo 沿用第 16 版 512→520→512 的小幅範例，會明確啟用 torque，結束不卸力，也不代表已驗證到位。

```powershell
make -B TCHAIN_PREFIX=arm-none-eabi- "COMPILE_OPTS=-mcpu=cortex-m3 -mthumb -Wall -g -Os -fno-common -fno-strict-aliasing -Wno-error=implicit-function-declaration -Wno-error=incompatible-pointer-types -Wno-error=int-conversion" CM530.hex CM530.bin
python tests/verify_firmware.py
python manual_position_terminal.py --self-test
./tests/run_tests.ps1 -Python python -Compiler gcc
```

亦可使用 `-Compiler <zig.exe完整路徑> -CompilerArgs cc` 執行 C 主機測試。ARM 編譯器只建置韌體；主機 C 測試需 GCC 或 Zig。

建置後更新 SHA256SUMS；交付 CM530.hex／CM530.bin，CM530.elf 留供一致性檢查。映像 Flash 起點 **0x08003000**、stack **0x20010000**，使用既有 CM-530 bootloader 載入流程，不將 BIN 寫入 Flash 0 起點。

本版離線測試與實機待驗項目見 [VALIDATION.md](VALIDATION.md)。第 15、16 版保留不變；舊主機須更新為協定 5，BEGIN/PT/END/STOP 仍不支援。
