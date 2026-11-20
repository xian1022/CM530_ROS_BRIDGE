# CM530_ROS_BRIDGE｜CM-530 雙臂 AX-12A 橋接韌體

**目前版本：第 19 版｜主機序列協定：7｜主機連線：1 Mbps｜文件核對：2026-11-20**

這是 ROBOTIS CM-530 控制板的自製韌體，屬於「視覺導引雙臂協同搬運系統」。CM-530 夾在 ROS 主機（NVIDIA Jetson AGX Orin）和兩支四軸 AX-12A 手臂之間：主機用一條 USB 序列線送文字指令，CM-530 驗證指令後，透過 DYNAMIXEL 匯流排寫入馬達目標、讀回位置，再回覆結果。

韌體本身**不做**路徑規劃、插值、到位判斷或軌跡排程，這些全部由 ROS 端負責。韌體只做三件事：驗證指令、寫馬達、讀馬達。

> **狀態：** 第 19 版的建置與所有主機端測試都已通過，**但還沒燒錄到板子上，也還沒在實機驗收**。板子目前是第 18 版；第 18 版保留在原目錄，隨時可以燒回。
>
> **第 19 版（2026-11-20）相對第 18 版：** `READ` 多回四個 Present Load（`POS,arm1,j1..j4,l1..l4`）；新增 `COMP`（寫 Compliance Margin／Slope 與 Punch）、`GET_COMP`（讀回）、`DIAG`（電壓、溫度）；協定改成 7。開機不寫 compliance，沒送 `COMP` 時馬達行為和第 18 版一樣。用途是 2026-11-19 的晃動診斷：要能調 compliance 比較，並看到負載在哪裡換方向。詳見 [第 19 版說明](19%20ROS%20to%20CM530%20ver.%20dual%20arm/README.md)。下面第 1～4、6～8、10～13 節描述的第 18 版內容在第 19 版不變。

---

## 目錄

1. [第 18 版改了什麼、為什麼要改](#1-第-18-版改了什麼為什麼要改)
2. [硬體與接線](#2-硬體與接線)
3. [通訊設定](#3-通訊設定)
4. [協定規則](#4-協定規則)
5. [指令一覽](#5-指令一覽)
6. [AX 指令與 Moving Speed](#6-ax-指令與-moving-speed)
7. [扭力與「目標資格」](#7-扭力與目標資格)
8. [開機與握手流程](#8-開機與握手流程)
9. [錯誤碼](#9-錯誤碼)
10. [HOME 設定](#10-home-設定)
11. [建置、驗證與燒錄](#11-建置驗證與燒錄)
12. [測試](#12-測試)
13. [手動測試終端](#13-手動測試終端)
14. [目錄結構](#14-目錄結構)
15. [驗證狀態](#15-驗證狀態)
16. [版本歷史](#16-版本歷史)
17. [相關文件](#17-相關文件)

---

## 1. 第 18 版改了什麼、為什麼要改

### 問題

第 17 版只寫入 AX-12A 的目標位置（Goal Position，位址 30），**從來不寫移動速度（Moving Speed，位址 32）**。

ROBOTIS 官方說明，Moving Speed 設成 0 代表「不控制速度、用最高轉速移動」，而這個暫存器開機後的值沒有保證。ROS 端每個控制週期只送一小步（幾個 tick），所以當 Moving Speed 是 0 時，馬達會用全速走完這一小步、停下來，再等下一筆指令。手臂因此一頓一頓地前進，還會前後擺動。

### 第 18 版的做法

| 項目 | 第 17 版 | 第 18 版 |
|---|---|---|
| 寫入馬達的內容 | 只有 Goal Position | Goal Position **加上** Moving Speed，同一個 SYNC_WRITE 封包 |
| `AX` 指令 | 4 個位置 | 4 個位置，可再加 4 個速度（1～1023） |
| 不帶速度的 `AX`、`HOME`、`HOLD` | 沿用馬達當下的速度（可能是 0，也就是全速） | 一律用預設速度 **100**（約 67°/s） |
| 速度 0 | — | 拒絕（`ERR,RANGE`），因為 0 代表全速、不控速 |
| 主機連線鮑率 | 57600 | **1,000,000（1 Mbps）** |
| 協定版本 | 5（`READY,5`、`VERSION,5`） | **6**（`READY,6`、`VERSION,6`） |
| HOME | 兩臂都是 256,512,512,512 | 相同 |

位置和速度用同一個封包寫入，所以馬達不可能「用舊速度去追新目標」。速度由 ROS 依軌跡算出，讓馬達在下一筆指令到達前剛好走完這一步，動作就會連續。

鮑率提高是為了縮短每個控制週期：57600 下，兩支手臂各一次「AX＋READ」光傳輸就要約 28 ms；1 Mbps 下約 1.6 ms。

**協定版本改成 6，是刻意讓新舊不相容。** 新的 ROS 驅動只接受 `VERSION,6`；板子如果還是第 17 版，連線會直接失敗，手臂不會在「沒有速度控制」的情況下被驅動。

---

## 2. 硬體與接線

| 項目 | 內容 |
|---|---|
| 控制板 | ROBOTIS CM-530，MCU 為 STM32F103RE（Cortex-M3，72 MHz） |
| 主機連線 | CM-530 的 PC Link（mini USB），板上 FT232R 轉 UART，接到 STM32 的 USART3 |
| 馬達匯流排 | STM32 的 USART1，DYNAMIXEL Protocol 1.0，1 Mbps，半雙工 TTL |
| 馬達 | AX-12A × 8，關節模式（CW／CCW Angle Limit 都不是 0） |
| 供電 | CM-530 建議 11.1 V；AX-12A 規格 9～12 V |

### 馬達 ID 對照

| 手臂 | 搬運區域 | j1（底座旋轉） | j2（肩） | j3（肘） | j4（腕） |
|---|---|---:|---:|---:|---:|
| arm1 | A → B | 17 | 3 | 2 | 15 |
| arm2 | B → C | 12 | 1 | 8 | 16 |

ID 寫死在 [APP/src/ax12.c](18%20ROS%20to%20CM530%20ver.%20dual%20arm/APP/src/ax12.c)，韌體不會修改馬達的 ID、EEPROM 或角度限位。

### 指示燈

| 手臂 | 綠燈（MOVING） | 紅燈（STOPPED） |
|---|---|---|
| arm1 | MANAGE（PB13） | PROGRAM（PB14） |
| arm2 | TXD（PC14） | RXD（PC15） |

LED 都是低電位點亮。開機時兩臂都亮紅燈。燈號只顯示 ROS 用 `LED` 指令設定的狀態，**`STOPPED` 不會讓馬達停下來**。

---

## 3. 通訊設定

| 連線 | 設定 |
|---|---|
| 主機 ↔ CM-530 | 1,000,000 baud、8 data bits、no parity、1 stop bit、無流量控制 |
| CM-530 ↔ AX-12A | 1,000,000 baud、DYNAMIXEL Protocol 1.0 |

### 為什麼 1 Mbps 可行

- STM32F103 的 USART3 掛在 36 MHz 的 APB1 上。官方規格中 USART2～5 最高 2.25 Mbit/s。1 Mbps 的除數是 36 MHz ÷ (16 × 1 Mbps) = **2.25，剛好整除，誤差 0%**。
- FT232R 在 TTL 電位下最高 3 Mbaud，1 Mbps = 3 MHz ÷ 3，**誤差 0%**。
- 主機和 STM32 之間是 FT232R 直接接 TTL，沒有 RS-232 轉換晶片，不受常見 RS-232 晶片 250 kbps 的限制。

ROBOTIS 沒有文件保證 CM-530 的 PC Link 能跑 1 Mbps，所以燒錄後要先用 `PING`／`VERSION` 確認一次。如果現場通訊不穩，可以退回 115200（兩端合計誤差約 0.3%，遠低於 3% 容許值），要同時改 [main.c](18%20ROS%20to%20CM530%20ver.%20dual%20arm/APP/src/main.c) 的 `Baudrate_PC` 和主機端程式。

### USB latency timer（主機端要設定）

FT232R 預設的 latency timer 是 **16 ms**：收到的資料如果沒填滿緩衝區，最多要等 16 ms 才交給電腦。CM-530 的回覆都很短，幾乎每次都要等滿，一個控制週期最多會多出約 64 ms。ROBOTIS 建議高鮑率時改成 **1 ms**。

Linux（例如 Orin）：

```bash
echo 'ACTION=="add", SUBSYSTEM=="usb-serial", DRIVER=="ftdi_sio", ATTR{latency_timer}="1"' | sudo tee /etc/udev/rules.d/99-ftdi-latency.rules
sudo udevadm control --reload-rules
```

重新插拔後確認，應該讀到 `1`：

```bash
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
```

Windows：裝置管理員 → 連接埠 → 該 COM 埠內容 → 連接埠設定 → 進階 → Latency Timer 設成 1。

---

## 4. 協定規則

- **格式：** ASCII 文字，大寫命令、小寫 `arm1`／`arm2`、半形逗號、十進位整數。例：`AX,arm1,512,512,512,512\n`
- **行尾：** 送出時 LF、CR、CRLF 都可以，建議用 LF；回覆一律是 CRLF。
- **長度：** 一行最多 95 個字元（緩衝 96 bytes，含行尾）。最長的合法指令 `AX,arm1,1023,…`（8 個數字）約 48 字元。
- **一問一答：** 整條連線同一時間只能有一筆待回覆的指令，包括兩支手臂和 LED。一定要收完並核對上一筆回覆，才能送下一筆。
- **沒有指令編號：** 韌體沒有序號、去重、佇列、取消或完成事件。**逾時後不可自動重送**，因為無法分辨遲到的回覆屬於哪一筆。
- **ACK 不等於到位：** `OK,AX,arm1` 只代表封包已送上馬達匯流排。到位要由 ROS 用 `READ` 判斷。
- **輸入容錯：** 韌體容許大小寫混用、欄位前後空白、整數正負號、BOM、全形逗號，但正式主機程式不應依賴這些。浮點數、空欄位、多餘欄位一律拒絕。
- **兩臂共用連線：** 兩支手臂的指令可以交錯送，但同一瞬間只處理一筆，不保證兩臂同時起動。

---

## 5. 指令一覽

以下 `arm1` 都可以換成 `arm2`。位置範圍 0～1023，**不代表機構能安全走完整個範圍**。

| 指令 | 成功回覆 | 作用 |
|---|---|---|
| `PING` | `PONG` | 確認通訊 |
| `VERSION` | `VERSION,7` | 確認協定版本 |
| `AX,arm1,p1,p2,p3,p4` | `OK,AX,arm1` | 寫入四軸目標，速度用預設 100 |
| `AX,arm1,p1,p2,p3,p4,s1,s2,s3,s4` | `OK,AX,arm1` | 寫入四軸目標和四個 Moving Speed（1～1023） |
| `HOME,arm1` | `OK,HOME,arm1` | 把板內 HOME 寫成目標，速度 100 |
| `GET_HOME,arm1` | `HOME,arm1,256,512,512,512` | 查詢板內 HOME，不會動作 |
| `READ,arm1` | `POS,arm1,j1,j2,j3,j4,l1,l2,l3,l4` | 讀回四軸位置與負載（第 19 版加上負載：0～1023 大小，+1024 = CW） |
| `HOLD,arm1` | `OK,HOLD,arm1` | 讀回四軸後，把目前位置寫成目標，速度 100 |
| `TORQUE,arm1,1` | `OK,TORQUE,arm1,1` | 上扭力；必須先有目標資格（見第 7 節） |
| `TORQUE,arm1,0` | `OK,TORQUE,arm1,0` | 卸扭力，並清除目標資格 |
| `COMP,arm1,m1..m4,s1..s4,p1..p4` | `OK,COMP,arm1` | 第 19 版：寫 margin（0～254）、slope（2～128 的 2 的次方）、punch（32～1023），CW＝CCW；不改目標與扭力 |
| `GET_COMP,arm1` | `COMP,arm1,m1..m4,s1..s4,p1..p4` | 第 19 版：從馬達讀回；CW 與 CCW 不同回 `DXL_ASYM` |
| `DIAG,arm1` | `DIAG,arm1,v1..v4,t1..t4` | 第 19 版：電壓（0.1 V）與溫度（°C） |
| `LED,arm1,MOVING` | `OK,LED,arm1,MOVING` | 綠燈亮、紅燈滅 |
| `LED,arm1,STOPPED` | `OK,LED,arm1,STOPPED` | 紅燈亮、綠燈滅 |

**注意事項：**

- `AX`、`HOME`、`HOLD` 不會自動上扭力。但扭力如果已經開著，寫入目標就會立刻動作。
- `READ` 依序讀四顆馬達，任何一顆失敗就回錯誤，不會回傳部分資料。每顆的接收期限 50 ms，不重試。
- `HOLD` 要等四軸都讀成功才寫入目標，讀取失敗時不寫入部分目標。它有取樣和傳輸延遲，**不是急停**。
- 舊協定的 `BEGIN`、`PT`、`END`、`STOP` 已移除，送出會回 `ERR,BAD_CMD`。

---

## 6. AX 指令與 Moving Speed

### 單位

| 量 | 數值 |
|---|---|
| 位置 1 tick | 300° ÷ 1024 ≈ 0.29° |
| Moving Speed 1 單位 | 0.111 rpm ≈ 0.666°/s ≈ 2.27 tick/s |
| 最高轉速（12 V、無負載） | 59 rpm ≈ 354°/s，約等於 Moving Speed 531 |

超過約 531 的數值不會更快，因為馬達本身到不了；帶負載或電壓偏低時會更慢。

### 換算範例

| Moving Speed | rpm | °/s | rad/s |
|---:|---:|---:|---:|
| 20 | 2.2 | 13 | 0.23 |
| 50 | 5.6 | 33 | 0.58 |
| 100（預設） | 11.1 | 67 | 1.16 |
| 300 | 33.3 | 200 | 3.49 |

### 規則

- 速度範圍 **1～1023**。送 0 回 `ERR,RANGE`；不是整數或欄位數不對回 `ERR,BAD_ARG`。
- 只帶 4 個位置時，四軸都用預設速度 100。
- `HOME` 和 `HOLD` **一律用預設速度 100**，不會沿用上一筆 AX 的速度。這是為了避免上一筆設了很小的速度，導致 HOME 走很久；也避免開機後沿用未知的速度。
- 預設速度定義在 [arm_config.h](18%20ROS%20to%20CM530%20ver.%20dual%20arm/APP/inc/arm_config.h) 的 `ARM_DEFAULT_SPEED`。
- 每一筆 AX 都會直接送出，即使和上一筆完全一樣。

### 封包內容

韌體送出一個 SYNC_WRITE：起始位址 30，每顆馬達 4 bytes，依序是 Goal Position 低位元組、高位元組、Moving Speed 低位元組、高位元組。四顆馬達共用這一個封包。

### ROS 端怎麼算速度（參考）

主控端驅動（`cm530_hardware`）每筆 AX 會依關節計算速度：

```
速度 = max(1.2 × 指令角速度, 剩餘誤差 ÷ 0.25 秒)，換算成 Moving Speed 後限制在 20～300
```

指令角速度取自控制器連續兩筆位置指令的差。乘上 1.2 是讓馬達稍微快一點，不會越落越後；剩餘誤差那一項則讓軌跡結束後，最後幾個 tick 也能在約 0.25 秒內走完。

---

## 7. 扭力與「目標資格」

「目標資格」是韌體內部的旗標，每支手臂各一個，決定能不能上扭力。它**不代表**已經上扭力、已經到位，或馬達確實收到目標。

| 事件 | 該臂的目標資格 | 對馬達的影響 |
|---|---|---|
| 開機 | 清除 | 嘗試卸扭力 |
| `AX`／`HOME` 成功 | 建立 | 寫入目標和速度 |
| `AX`／`HOME` 傳送失敗 | 清除 | 結果未知 |
| `HOLD` 成功 | 建立 | 讀四軸後寫入目標 |
| `HOLD` 讀取或寫入失敗 | 清除 | 讀取失敗時不寫入；寫入失敗時結果未知 |
| `TORQUE,0`（不論傳送成敗） | 清除 | 嘗試卸扭力 |
| `TORQUE,1` | 不變 | 有資格才嘗試上扭力，沒有就回 `ERR,NO_TARGET` |
| `READ`、`GET_HOME`、`PING`、`VERSION`、`LED` | 不變 | 只讀取，或只更新燈號 |
| 格式或範圍錯誤 | 不變 | 不寫馬達 |

**正常的上扭力順序：** 先設定完整目標（`AX`、`HOME` 或 `HOLD`）→ 確認 `OK` → `TORQUE,arm1,1` → 確認 `OK`。任何一次卸扭力之後，都要重新設定目標才能再上扭力。

---

## 8. 開機與握手流程

### 控制板開機

1. 初始化時脈、GPIO、LED（兩臂亮紅燈）、計時器、兩個 UART。
2. 對兩支手臂各送一次卸扭力。
3. 兩筆都傳送成功 → 輸出 `READY,6`；任何一筆失敗 → 輸出 `ERR,INIT_FAILED`。

`READY,6` 只代表封包送出，不證明馬達真的卸力。初始化失敗後，`AX`、`TORQUE`、`HOME`、`READ`、`HOLD` 都會回 `ERR,INIT_FAILED,<arm>`，直到重新開機；`PING`、`VERSION`、`LED`、`GET_HOME` 仍然可以用。

### 主機開埠握手

1. 開啟序列埠（1 Mbps、8N1）。`READY,6` 只在開機那一刻送出，開埠時可能已經錯過，沒收到不算失敗。
2. 建議先送一串 DEL（0x7F）字元，清掉韌體緩衝區裡上一次留下的半行。DEL 只會刪字，不會像換行一樣把殘留內容當成指令執行。
3. `VERSION` → 必須回 `VERSION,6`。回 `VERSION,5` 表示板子還是第 17 版，應該停止。
4. `PING` → `PONG`。
5. 依需要查詢 `GET_HOME`、`READ`，確認姿勢後再設定目標、上扭力。

主機在一筆指令進行中收到 `READY` 開頭的行，代表控制板重新開機了，所有狀態都要重新確認。

---

## 9. 錯誤碼

格式：`ERR,<代碼>[,<arm>][,<馬達 ID>][,<錯誤位元>]`

| 代碼 | 意思 | 處理方式 |
|---|---|---|
| `BAD_CMD` | 不支援的指令（例如 `STOP`） | 修正指令，不要重試 |
| `BAD_ARG` | 欄位數、arm、整數格式、扭力值或 LED 狀態不合法 | 修正指令 |
| `RANGE` | 位置不在 0～1023、速度不在 1～1023，或板內 HOME 超出範圍 | 修正數值 |
| `NO_TARGET` | 還沒有目標資格就要上扭力 | 先處理之前的失敗，再設定目標 |
| `INIT_FAILED` | 開機初始化失敗，馬達指令被鎖住 | 重新開機 |
| `OVERFLOW` | 一行太長，或 UART 收資料時遺失 | 停止排程，重新同步連線 |
| `DXL_TX` | 送往馬達的封包傳送失敗；帶 ID 時是讀取請求失敗 | 不能證明馬達沒動；不自動重送 |
| `DXL_TIMEOUT` | 讀某顆馬達時 50 ms 內沒有回應 | 檢查 ID、供電、線材 |
| `DXL_CORRUPT` | 回應的 ID、長度或 checksum 不符，或只收到一部分 | 檢查線材、雜訊 |
| `DXL_RX` | 其他接收失敗 | 檢查硬體 |
| `DXL_MOTOR` | 馬達回報錯誤位元 | 依位元判斷（見下表） |
| `DXL_RANGE` | 讀回的位置不在 0～1023、負載不在 0～2047，或 punch 超過 1023 | 不要當成有效位置 |
| `DXL_ASYM` | 第 19 版 `GET_COMP`：某顆的 CW 與 CCW margin／slope 不同 | 重新送 `COMP`，不要啟用 |

`DXL_MOTOR` 的錯誤位元（十進位，可以相加）：1 電壓、2 角度限制、4 過熱、8 範圍、16 checksum、32 過載、64 指令。例如 5 = 電壓＋過熱。

---

## 10. HOME 設定

HOME 寫在 [APP/inc/arm_config.h](18%20ROS%20to%20CM530%20ver.%20dual%20arm/APP/inc/arm_config.h)：

```c
static const unsigned short home_A[4] = {256, 512, 512, 512};  /* arm1 */
static const unsigned short home_C[4] = {256, 512, 512, 512};  /* arm2 */
#define ARM_DEFAULT_SPEED 100
```

這是馬達的原始位置值，不是角度。數值是 2026-11-13 改定的，並以當天板子 `GET_HOME` 的回覆確認過。

- `HOME` 只是一組預設目標，不是感測器歸零，也不保證回程路徑不會碰撞。手臂離 HOME 很遠時，由 ROS 規劃避障路徑，用 `AX` 逐點送比較安全。
- 修改 HOME 後要重新建置、驗證、更新 SHA256SUMS 並燒錄。Makefile 已設定修改 `arm_config.h` 時會重新編譯 `bridge.o`。
- ROS 端的校正檔（`expected_home`）要和這裡一致，否則驅動會拒絕連線。

---

## 11. 建置、驗證與燒錄

在 `18 ROS to CM530 ver. dual arm/` 目錄下執行。

### 需要的工具

| 工具 | 用途 | 已驗證的版本 |
|---|---|---|
| Arm GNU Toolchain（`arm-none-eabi-gcc`） | 編譯韌體 | 14.2 rel1 |
| make | 建置 | WinARM 內附 |
| gcc（主機端） | C 單元測試 | WSL Ubuntu 24.04 的 gcc 13 |
| Python 3 ＋ pyserial | 手動終端、驗證腳本 | — |

### 建置

```powershell
make -B TCHAIN_PREFIX=arm-none-eabi- "COMPILE_OPTS=-mcpu=cortex-m3 -mthumb -Wall -g -Os -fno-common -fno-strict-aliasing -Wno-error=implicit-function-declaration -Wno-error=incompatible-pointer-types -Wno-error=int-conversion" CM530.hex CM530.bin
```

建置會出現約 10 個來自原廠 SDK（`dxl_hal.c` 的隱式宣告）的警告，第 17 版就有，不影響功能。

### 驗證產物

```powershell
python tests/verify_firmware.py
```

這會檢查 ELF 是 ARM 格式、HEX 的 checksum、HEX 和 BIN 內容一致、向量表、馬達 ID 對照，以及協定 6 的字串都在、舊協定的字串都不在。通過後更新 [SHA256SUMS](18%20ROS%20to%20CM530%20ver.%20dual%20arm/SHA256SUMS)。

目前第 18 版的產物：

| 檔案 | 大小 | SHA-256 |
|---|---:|---|
| CM530.hex | 36,109 bytes | `e3c1b4137203ccc8eff809366557aad13497b75be2b270890671a212b7d830cf` |
| CM530.bin | 12,816 bytes | `a6223babbc1d335b2c419bf96bdcaf1556b5529619aa75d622a3b2b208a0f33b` |

### 燒錄

- 燒錄檔：[CM530.hex](18%20ROS%20to%20CM530%20ver.%20dual%20arm/CM530.hex) 或 [CM530.bin](18%20ROS%20to%20CM530%20ver.%20dual%20arm/CM530.bin)
- Flash 起點：**0x08003000**（前面是 CM-530 原廠 bootloader，不可覆蓋）
- 使用既有的 CM-530 bootloader 流程燒錄

燒錄後確認：

1. 開機兩臂亮紅燈。
2. 以 1 Mbps 開埠，`VERSION` → `VERSION,6`，`PING` → `PONG`。
3. `GET_HOME,arm1`、`GET_HOME,arm2` 都回 `256,512,512,512`。
4. 連續送約 1000 次 `PING`，確認 1 Mbps 沒有通訊錯誤。

要回到第 17 版時，燒錄 `17 ROS to CM530 ver. dual arm/CM530.hex`，主機端也要換回 57600、協定 5 的程式。

---

## 12. 測試

主機端測試不需要板子：用模擬的 HAL 和 GPIO，直接編譯正式的 `bridge.c`、`ax12.c`、`dynamixel.c` 來測。

### Windows（PowerShell）

```powershell
python -m pip install -r requirements.txt
python manual_position_terminal.py --self-test
./tests/run_tests.ps1 -Python python -Compiler gcc
```

### Linux 或 WSL

```bash
gcc -std=gnu89 -Wall -Wextra -Werror -IAPP/inc APP/src/bridge.c APP/src/ax12.c APP/src/dynamixel.c tests/test_bridge.c -o /tmp/test_bridge && /tmp/test_bridge
gcc -std=gnu89 -Wall -Wextra -Werror -Itests/fakes -IAPP/inc APP/src/arm_led.c tests/test_arm_led.c -o /tmp/test_arm_led && /tmp/test_arm_led
```

加上 `-fsanitize=address,undefined -fno-omit-frame-pointer -g` 可以同時檢查記憶體和未定義行為。

### 測試涵蓋範圍

| 測試 | 內容 |
|---|---|
| `test_bridge.c` boot | 開機卸扭力、`READY,6`、目標資格、初始化失敗鎖定 |
| `test_bridge.c` direct | AX 直接寫入、已移除的舊指令、傳送失敗時清除資格 |
| `test_bridge.c` sdk | DYNAMIXEL SDK 接收：逾時、ID 錯、長度錯、checksum 錯、截斷 |
| `test_bridge.c` led | LED 指令、兩臂隔離、初始化失敗時 LED 仍可用 |
| `test_bridge.c` input | 格式錯誤、溢位、BOM、全形逗號、NUL、過長的行 |
| `test_bridge.c` v17 | HOME、GET_HOME、READ、HOLD 和各種讀取錯誤 |
| `test_bridge.c` speed | **第 18 版新增：** 四個速度的封包內容、速度 0／1024／負數、欄位數錯誤、HOME／HOLD 一律用預設速度 |
| `test_arm_led.c` | LED 腳位、低電位點亮、紅綠互斥、不碰電源和 UART 腳位 |
| `tests/test_terminal.py` | 手動終端的指令解析、回覆核對、握手（26 項） |
| `tests/verify_firmware.py` | 燒錄產物的完整性 |

---

## 13. 手動測試終端

[manual_position_terminal.py](18%20ROS%20to%20CM530%20ver.%20dual%20arm/manual_position_terminal.py) 是現場用的序列終端，預設 1 Mbps、協定 6。ROS 驅動和終端不能同時開同一個序列埠。

```powershell
python manual_position_terminal.py --port COM4 --arm arm1
```

```bash
python3 manual_position_terminal.py --port /dev/serial/by-id/<控制板> --arm arm1
```

- 輸入 `?` 看指令，`q` 離開。
- 有指定 `--arm` 時，可以直接輸入四個數字當作 AX。
- 終端會檢查位置 0～1023、速度 1～1023、行長度，並核對每一筆回覆。
- `demo` 會寫入 512 並上扭力，再讓 j1 在 512 → 520 → 512 之間移動。這不是 HOME 流程，使用前要確認這些位置對機構是安全的。結束時不會卸扭力。

---

## 14. 目錄結構

```
CM530_ROS_BRIDGE/
├── 19 ROS to CM530 ver. dual arm/      ← 目前版本（協定 7：負載、compliance）
│   ├── APP/
│   │   ├── inc/arm_config.h            HOME 與預設速度
│   │   ├── src/main.c                  時脈、UART（含主機鮑率）、主迴圈
│   │   ├── src/bridge.c                指令解析、目標資格、回覆
│   │   ├── src/ax12.c                  SYNC_WRITE（位置＋速度、compliance、punch）、扭力、讀位置／負載／compliance／診斷
│   │   ├── src/arm_led.c               兩臂指示燈
│   │   ├── src/dynamixel.c, dxl_hal.c  ROBOTIS DYNAMIXEL SDK
│   │   └── src/stm32f10x_it.c          中斷
│   ├── stm32f10x_lib/                  ST 原廠函式庫
│   ├── tests/                          主機端測試
│   ├── manual_position_terminal.py     手動終端
│   ├── ROS_CM530_INTERFACE_SPEC.md     完整對接規格
│   ├── Makefile, stm32.ld              建置設定（Flash 起點 0x08003000）
│   └── CM530.hex, CM530.bin, SHA256SUMS
├── 18 ROS to CM530 ver. dual arm/      ← 上一版（協定 6），板子目前燒的版本，保留不動
├── 17 ROS to CM530 ver. dual arm/      ← 協定 5、57600，保留不動
├── 12～16 …                            更早的版本
├── 01～11 …                            ROBOTIS 原廠範例
└── embeddec_c(cm530_v1_02) of/         ROBOTIS Embedded C 原始範例
```

韌體架構：USART3 的接收中斷把字元放進 128 bytes 的環形緩衝區；主迴圈逐字取出交給 `BridgeFeed()`，遇到行尾就解析並執行指令。執行 DYNAMIXEL 傳輸期間，主機必須等回覆才送下一筆，所以緩衝區不會被灌爆。

---

## 15. 驗證狀態

| 項目 | 狀態 | 日期 |
|---|---|---|
| 第 19 版 ARM 完整建置、`verify_firmware.py` | 通過 | 2026-11-20 |
| 第 19 版 C 測試（含 COMP／GET_COMP／DIAG／負載），gcc、`-Werror`，另加 ASan＋UBSan | 通過 | 2026-11-20 |
| 第 19 版 Python 終端測試 27 項 | 通過 | 2026-11-20 |
| ROS 驅動透過模擬序列埠跑第 19 版解析器（端對端，含 COMP 寫入與讀回） | 通過 | 2026-11-20 |
| **燒錄第 19 版、COMP／負載／DIAG 實機驗收** | 尚未執行 | — |
| 第 18 版 ARM 完整建置、`verify_firmware.py` | 通過 | 2026-11-18 |
| 第 18 版 C 測試（含速度測試），gcc 13、`-Werror` | 通過 | 2026-11-18 |
| 同上，加 AddressSanitizer ＋ UBSan | 通過 | 2026-11-18 |
| 第 18 版 Python 終端測試 26 項 | 通過 | 2026-11-18 |
| ROS 驅動透過模擬序列埠跑第 18 版解析器（端對端） | 通過 | 2026-11-18 |
| **燒錄第 18 版** | 尚未執行 | — |
| **1 Mbps 實機連線** | 尚未執行 | — |
| **速度控制實機效果（卡頓、擺動是否改善）** | 尚未執行 | — |
| HOME 實機姿態 | 尚未驗收 | — |

所有驗證紀錄集中在 [驗證與測試總整理](驗證與測試總整理.md)。

---

## 16. 版本歷史

| 版本 | 協定 | 主機鮑率 | 重點 |
|---|---|---|---|
| **19**（2026-11-20） | 7 | 1 Mbps | READ 帶 Present Load；COMP／GET_COMP／DIAG；開機不寫 compliance |
| 18（2026-11-18） | 6 | 1 Mbps | AX 帶 Moving Speed；HOME／HOLD 用預設速度 100；拒絕速度 0 |
| 17（2026-10-07，HOME 於 2026-11-13 改為 256,512,512,512） | 5 | 57600 | 新增 HOME、GET_HOME、READ、HOLD；移除 BEGIN／PT／END／STOP；只寫位置 |
| 16（2026-09-30） | 4 | — | 雙臂 AX、TORQUE、LED |
| 12～15 | — | — | 早期橋接版本，指令語法（例如 BEGIN／PT／END）與現行不相容 |
| 01～11 | — | — | ROBOTIS 原廠功能範例（LED、按鈕、序列埠、DYNAMIXEL 讀寫等） |

---

## 17. 相關文件

| 文件 | 內容 |
|---|---|
| [第 19 版操作說明](19%20ROS%20to%20CM530%20ver.%20dual%20arm/README.md) | 版本目錄內的簡要說明 |
| [ROS ↔ CM-530 對接規格](19%20ROS%20to%20CM530%20ver.%20dual%20arm/ROS_CM530_INTERFACE_SPEC.md) | 完整協定、收發範例、錯誤處理建議 |
| [第 18 版操作說明](18%20ROS%20to%20CM530%20ver.%20dual%20arm/README.md) | 上一版 |
| [驗證與測試總整理](驗證與測試總整理.md) | 所有版本的測試與驗收紀錄 |
| [HOME 校正資料收集](18%20ROS%20to%20CM530%20ver.%20dual%20arm/HOME_CALIBRATION_TEST.md) | HOME 量測流程 |
| [AGENTS.md](AGENTS.md) | 開發規範 |
| [ROBOTIS CM-530 e-Manual](https://emanual.robotis.com/docs/en/parts/controller/cm-530/) | 控制板官方說明 |
| [ROBOTIS AX-12A e-Manual](https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/) | 馬達控制表（Goal Position、Moving Speed 等） |
