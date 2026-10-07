# ROS ↔ CM-530 對接規格

**第 17 版韌體／主機序列協定 5**
更新：2026-10-07

本文件供 ROS 通訊與運動控制程式開發使用，描述目前韌體的實際介面。標示「ROS 端建議」的內容屬主機實作方式，不代表本儲存庫已提供 ROS 節點。

## 目錄

- [1. 對接範圍與固定設定](#section-1)
- [2. 序列格式與收發規則](#section-2)
- [3. 開機、握手與重新連線](#section-3)
- [4. 指令與回覆詳細定義](#section-4)
- [5. 每臂目標資格與副作用](#section-5)
- [6. ROS 軌跡發送與位置回饋](#section-6)
- [7. 完整收發範例](#section-7)
- [8. 錯誤碼與處理](#section-8)
- [9. ROS 通訊層實作建議](#section-9)
- [10. 對接驗收與檔案索引](#section-10)

<a id="section-1"></a>

## 1. 對接範圍與固定設定

### 1.1 分工

ROS：關節順序與單位換算、目標與路徑計算、發送時間、回覆核對、到位判斷及異常處理。
CM-530：解析命令、選擇手臂、寫入目標／torque、讀取馬達位置、設定 LED、回覆處理結果。
外部任務、視覺及吸盤流程不在本協定內；它們由 ROS 使用本介面的結果銜接。

### 1.2 連線設定

主機 ↔ CM-530：USART3，57600 baud，8 data bits，no parity，1 stop bit，無流量控制。
CM-530 ↔ AX-12A：USART1，1 Mbps，DYNAMIXEL Protocol 1.0。
「協定 5」指主機文字協定；不代表馬達端改用其他 DYNAMIXEL 協定。

一塊 CM-530 管理兩臂，共用主機序列埠與馬達 bus。
各臂關節欄位順序固定：

| 手臂 | j1 | j2 | j3 | j4 |
|---|---:|---:|---:|---:|
| arm1 | 17 | 3 | 2 | 15 |
| arm2 | 12 | 1 | 8 | 16 |

表內為馬達 ID；主機傳 arm1/arm2，不在 AX 命令中傳 ID。

馬達須為關節模式，Status Return Level=1 或 2 才可回應 READ。
韌體不自動修改 ID、EEPROM、速度、限位或校正資料。

### 1.3 位置單位

AX／HOME／POS 的 j1..j4 均為 AX-12A 原始位置整數，範圍 0..1023。
不接受 rad、degree、XYZ、速度、加速度或 time_from_start 欄位。
ROS 必須依每個關節的零點、方向與實際機構限位完成換算；不可直接把 ROS 浮點關節角度送入 AX。
READ 回傳亦依同一關節順序，由 ROS 反向換算後供模型或 joint state 使用。
0..1023 只是協定數值範圍，不等於機構允許範圍。

<a id="section-2"></a>

## 2. 序列格式與收發規則

### 2.1 正式格式

使用 ASCII、大寫命令、小寫 arm1/arm2、半形逗號、十進位整數。

- **例如實際發送 bytes：** `AX,arm1,512,512,512,512\n`

本文件顯示的 TX／RX 是紀錄標記，不是封包的一部分。
`<arm>`、`<j1>` 等尖括號內容是欄位名稱，發送時必須替換成實際值。

輸入行尾：LF、CR、CRLF 皆可；建議 ROS 統一 LF。
輸出行尾：固定 CRLF。
一筆 read() 可能只收到部分回覆，也可能帶有多筆資料；ROS 必須累積 bytes 並按行尾拆行。
空行與只含空白的行不回覆；不可用空行進行握手或計算 ACK 數量。

### 2.2 輸入限制

- 行緩衝為 96 bytes，最多 95 bytes 內容，不含行尾。
- AX 固定四軸，不接受單值 AX、八軸一次輸入或省略 arm。
- arm 只接受 arm1、arm2；ALL、BOTH、A、B 皆不支援。
- 整數先檢查 signed 32-bit 範圍，再檢查命令允許值。
- 浮點、空欄位、額外欄位、數字夾雜其他字元會被拒絕。
- 韌體容許命令／arm／LED 狀態大小寫、欄位前後空白與 tab、整數正負號；正式主機不需依賴這些容錯。
- 相容人工輸入的 BOM、全形逗號、backspace 處理仍保留；正式主機請只送標準 ASCII。
- 內嵌 NUL 不會截斷並接受前半段指令，而會導致該內容無法按正常命令解析。

### 2.3 一問一答

整條主機連線任何時刻最多一筆待回覆命令，包括不同手臂及 LED 命令。
必須收完並核對前一筆回覆後才發下一筆，不可批次寫入整段 AX 點列。
韌體沒有 command ID、sequence number、重複命令去重、軌跡佇列、取消封包或 DONE 事件。
READ／GET_HOME 成功時回一筆資料行，不會先後回 OK 再回資料。
開機 READY 與 UART 溢位是需另外處理的事件，不能直接視為當前命令成功回覆。

### 2.4 回覆核對

PING：只接受 PONG。
VERSION：只接受 VERSION,5。
AX／HOME／HOLD：精確比對 OK、命令名稱、arm，共 3 欄。
TORQUE／LED：精確比對 OK、命令名稱、arm、要求值，共 4 欄。
READ：只接受 POS、指定 arm、四個 0..1023 整數，共 6 欄。
GET_HOME：只接受 HOME、指定 arm、四個 0..1023 整數，共 6 欄。
ERR：本筆操作失敗，按第 8 節分類；不得作為到位或未動作證據。

錯 arm、錯命令、錯欄位數、越界值、未預期 READY、其他非空資料均屬錯配。
發送前若有上一筆殘留的非空資料或未完成片段，不要拿它當作新命令回覆。
僅行尾殘留（例如上一筆 CRLF 中的 LF）可忽略。
本協定無法靠命令編號辨別遲到的同型 ACK，因此逾時後不能直接續送同型命令或自動重送。

<a id="section-3"></a>

## 3. 開機、握手與重新連線

### 3.1 控制板開機

1. 設定兩臂 LED 為 STOPPED（雙紅）。
2. 清除兩臂目標資格。
3. 依序送兩臂各四軸 torque-off；第一筆失敗仍嘗試第二筆。
4. 兩筆本地傳送成功：READY,5。
5. 任一失敗：ERR,INIT_FAILED，馬達命令鎖定至控制板重啟。

開機不發位置、不執行 HOME、不自動 torque-on。
READY 表示本地初始化傳送完成，不表示已回讀確認八顆馬達的 torque 狀態。
初始化失敗仍可用 PING、VERSION、LED、GET_HOME；AX／HOME／READ／HOLD／TORQUE 被鎖定。
查詢 VERSION 不會解除鎖定。

### 3.2 ROS 開埠握手

- 序列埠由單一 ROS 通訊管理者獨占，測試終端先關閉。
- 可監聽 READY，但它可能在開埠前已發出；沒有 READY 不能單獨判為連線失敗。
- 每次開埠都必須 VERSION -> VERSION,5，再 PING -> PONG。
- 協定不符停止動作，不自動降級或嘗試舊命令。
- GET_HOME 可用來同步韌體預設值；READ 可用來取得當前姿態。
- VERSION/PING 成功只確認主機介面，不證明馬達可讀、初始化成功或已卸力。

手動終端預設啟動監聽 6 秒、單筆回覆期限 2 秒；這是主機預設，不是韌體自動執行的等待流程。

### 3.3 重新連線／執行中重啟

重新開啟序列埠不等於控制板重啟；馬達可能仍有 torque 且追蹤最後目標。
若待回覆期間收到 READY，將本次操作標為失敗，停止後續軌跡，重新握手並重新取得狀態。
連線中斷後不要沿用舊 READ 資料、舊「已到位」判斷或未完成命令佇列。
重新連線應先確認姿態與現場狀態，再由 ROS 明確選擇後續目標／torque 操作。
單純重新握手不能判斷斷線前的命令是否已被執行。

<a id="section-4"></a>

## 4. 指令與回覆詳細定義

### 4.1 PING／VERSION

```text
PING -> PONG
VERSION -> VERSION,5
```

均不帶 arm、不碰馬達、不改目標資格；多餘參數回 BAD_ARG。

### 4.2 AX：四軸位置目標

- **格式：** `AX,<arm>,<j1>,<j2>,<j3>,<j4>`
- **成功：** `OK,AX,<arm>`
- **例：** `AX,arm2,520,512,500,512 -> OK,AX,arm2`

四個值檢查通過後，對指定 arm 送一筆四軸 Goal Position(30) SYNC_WRITE。
只寫位置，不設定時間、速度或 torque；torque 已開啟時可能立即開始移動。
成功建立該臂目標資格；傳送失敗清除資格，不允許用舊資格啟用 torque。
每筆 AX 都直接送出，即使與上一筆位置完全相同。
OK,AX 不包含目標值回顯，主機須保存本筆四軸目標並嚴格採一問一答。

### 4.3 HOME：使用板內預設目標

- **格式：** `HOME,<arm>`
- **成功：** `OK,HOME,<arm>`
arm1 使用 home_A；arm2 使用 home_C。
目前兩組都是 512,512,512,512，定義於 APP/inc/arm_config.h。

HOME 檢查板內四軸範圍後，使用與 AX 相同的寫入路徑及資格規則。
不先 READ、不插補、不規劃避障、不自動啟用 torque，也不等待到位才 ACK。
需要沿規劃路徑返回時，ROS 使用 GET_HOME 的結果作為規劃終點，再逐點發 AX。

### 4.4 GET_HOME：取得板內預設

- **格式：** `GET_HOME,<arm>`
- **成功：** `HOME,<arm>,<j1>,<j2>,<j3>,<j4>`
- **例：** `GET_HOME,arm1 -> HOME,arm1,512,512,512,512`

只回傳韌體常數，不讀馬達、不移動、不建立目標資格。
GET_HOME 的 HOME 資料行與 HOME 命令的 OK,HOME 回覆不可混用。
修改設定並重新燒錄後，主機應重新查詢，避免沿用舊快取。

### 4.5 READ：取得實際位置

- **格式：** `READ,<arm>`
- **成功：** `POS,<arm>,<j1>,<j2>,<j3>,<j4>`
- **例：** `READ,arm1 -> POS,arm1,510,514,512,511`

依 j1..j4 的 ID 順序，逐顆單播讀 Present Position(36)，每顆讀 2 bytes。
每顆接收期限 50 ms，不重試；檢查 transport、馬達 error bits、位置範圍。
任一顆失敗立即終止，回 ERR，不回部分 POS、不補舊值。
四顆成功才回 POS；四值來自不同取樣時間，無板端時間戳。
READ 不寫目標、不改 torque、不改目標資格，亦不主動宣告到位。

### 4.6 HOLD：讀取目前位置後設為目標

- **格式：** `HOLD,<arm>`
- **成功：** `OK,HOLD,<arm>`

有效命令先清除該臂目標資格，接著執行一次全新的四軸 READ。
全部成功才把這四個值用 SYNC_WRITE 寫為目標；成功重建目標資格。
讀取失敗：不發任何保持位置寫入，資格保持無效。
寫入失敗：回 DXL_TX，資格保持無效；不能推論馬達一定沒收到封包。
HOLD 不修改 torque；本來關閉的 torque 不會被重新開啟。
OK,HOLD 不回傳寫入值，也不表示手臂已靜止；主機需要另發 READ 觀察。
取樣至寫入之間手臂仍可能移動，因此 HOLD 不承諾瞬間停止或固定停止距離。

### 4.7 TORQUE：施力開關

- **格式：** `TORQUE,<arm>,<enabled>`
enabled 只接受 0 或 1。

- **成功：** `OK,TORQUE,<arm>,<enabled>`

0：傳送前清除該臂資格，再發四軸 torque-off；即使傳送失敗也不恢復資格。
1：該臂先前須有成功 AX／HOME／HOLD，否則 ERR,NO_TARGET,`<arm>`；不會自動補目標。
TORQUE,1 成功或本地 TX 失敗都不改原本資格，沿用第 16 版語意。
韌體資格不是已確認的 torque 狀態；本命令沒有 readback 驗證。
TORQUE,0 為卸力，不等同 HOLD。

### 4.8 LED：ROS 通知顯示狀態

- **格式：** `LED,<arm>,<MOVING|STOPPED>`
- **成功：** `OK,LED,<arm>,<MOVING|STOPPED>`

MOVING：綠亮紅滅；STOPPED：紅亮綠滅。
arm1：MANAGE/PB13 綠、PROGRAM/PB14 紅；arm2：TX/PC14 綠、RX/PC15 紅。
低電位亮；PLAY/AUX 熄滅，POWER 不變。
只操作 GPIO，不發馬達封包、不改目標資格或 torque；初始化失敗仍可設定。
AX／HOME／HOLD／TORQUE 不自動切 LED，錯誤、退出與斷線也不自動切換。
ROS 準備動作時可送 MOVING，確認停止後再送 STOPPED；STOPPED 本身不是停止命令。
終端隱藏正常 LED TX/RX 顯示，但線上 ACK 仍存在且必須核對。

<a id="section-5"></a>

## 5. 每臂目標資格與副作用

「目標資格」是允許 TORQUE,1 的韌體內部紀錄，每臂獨立。
它不代表已施力、已到位或已確認馬達實際收到目標。

| 事件 | 該臂資格 | 馬達副作用 |
|---|---|---|
| 開機 | 清除 | 嘗試 torque-off |
| 成功 AX／HOME | 建立 | 寫目標 |
| AX／HOME TX 失敗 | 清除 | 結果未知 |
| 成功 HOLD | 建立 | 讀四軸後寫目標 |
| HOLD 讀取失敗 | 清除 | 無保持目標寫入 |
| HOLD 寫入失敗 | 清除 | 目標寫入結果未知 |
| 有效 TORQUE,0，不論 TX 成敗 | 清除 | 嘗試 torque-off |
| TORQUE,1，不論 TX 成敗 | 不變 | 有資格才嘗試 torque-on |
| READ，不論讀取成敗 | 不變 | 只讀取 |
| GET_HOME／PING／VERSION／LED | 不變 | LED 僅更新 GPIO |
| 格式／範圍錯誤、未知命令 | 不變 | 無馬達寫入 |
| 另一臂的命令 | 不變 | 操作另一臂 |

正常啟用順序：設定完整目標 -> 核對 OK -> TORQUE,1 -> 核對 OK。
任何卸力之後，必須重新成功設定目標才能再啟用。

<a id="section-6"></a>

## 6. ROS 軌跡發送與位置回饋

### 6.1 逐點執行

ROS 保存整段軌跡，將各點轉成四軸原始位置，依主機排程時間發 AX。
CM-530 不接收 dt_ms，不存點列，不計算插補，不執行 BEGIN／PT／END。
中間點核對 ACK 後，由 ROS 決定下一點發送時機；不需要逐點等待機械到位。
ACK 表示傳送完成，不保證實際運動符合原始規劃速度／加速度。

兩臂 AX 可交錯發送，但同一瞬間僅處理一筆命令，不保證兩臂同步起動。
主機排程必須計入 ACK、READ、LED 與兩臂共用連線的時間。
若落後排程，ROS 應依自身軌跡策略中止或重新規劃；不可為追趕進度一次灌入所有未送點。

### 6.2 到位判斷（ROS 端建議）

- 保存該臂目前最終目標 target[4]。
- 完成最後一筆 AX／HOME 的 ACK 核對後，開始新的到位監測。
- READ 四軸成功才更新樣本；錯誤或逾時不得沿用上次樣本充當本次成功。
- 比較每軸 abs(measured[j]-target[j]) <= tolerance[j]。
- 要求連續 stable_count 次符合，任何一次超差即重新計數。
- 使用主機 monotonic clock 計算總期限；arrival_timeout 到期標為失敗。
- 新目標、HOLD、卸力、重連或重啟後，舊到位判斷失效。

tolerance[4]、stable_count、採樣間隔與 arrival_timeout 必須由 ROS 設定並實機校正。
此處未指定通用閾值，韌體也沒有對應設定命令。
位置穩定只能支持關節到位判斷，不直接證明取放任務或其他模組已完成。

### 6.3 通訊與時間限制

單顆 DXL READ 的接收期限為 50 ms；四軸等候預算至多約 4×50 ms，加上傳輸與處理。
這是接收等待預算，不是包含所有 HAL／作業系統延遲的硬即時保證。
手動終端每筆命令 timeout 預設 2 秒，ROS 可將其設為可配置參數。
通訊命令期限與機械到位總期限應分開，不可用「等到 ACK」取代到位監測。
韌體只在 READ/HOLD 指令時讀取；不主動持續送 joint state。
ROS 若發布 joint state，應由成功 READ 換算，附主機時間與有效性／新鮮度管理。

<a id="section-7"></a>

## 7. 完整收發範例

以下位置僅示範協定，需先確認校正值及運動路徑適合實機。
每一行 TX 必須在上一筆 RX 完整核對後執行；範例不含 ROS 的時間排程。

### 7.1 握手與查詢（不發位置、不啟用 torque）

```text
RX  READY,5                            （僅板子開機時發送，可能已錯過）
TX  VERSION
RX  VERSION,5
TX  PING
RX  PONG
TX  GET_HOME,arm1
RX  HOME,arm1,512,512,512,512
TX  READ,arm1
RX  POS,arm1,510,514,512,511
```

### 7.2 明確要求回 HOME 並啟用

```text
TX  HOME,arm1
RX  OK,HOME,arm1
TX  TORQUE,arm1,1
RX  OK,TORQUE,arm1,1
TX  READ,arm1
RX  POS,arm1,511,513,512,511
TX  READ,arm1
RX  POS,arm1,512,512,512,512
```

最後一筆數值不會讓韌體額外發 DONE；是否滿足到位條件由 ROS 決定。
arm2 另送 HOME,arm2 與 TORQUE,arm2,1，不會被 arm1 命令自動帶動。

### 7.3 雙臂交錯更新（假設兩臂已明確啟用 torque）

```text
TX  AX,arm1,520,512,512,512
RX  OK,AX,arm1
TX  AX,arm2,508,512,512,512
RX  OK,AX,arm2
TX  READ,arm1
RX  POS,arm1,518,512,512,512
TX  READ,arm2
RX  POS,arm2,509,512,512,512
```

兩次 READ 不代表同一時刻的雙臂快照。

### 7.4 HOLD 成功及失敗

```text
TX  HOLD,arm1
RX  OK,HOLD,arm1
TX  READ,arm1
RX  POS,arm1,517,512,512,512
```

另一種結果：

```text
TX  HOLD,arm1
RX  ERR,DXL_TIMEOUT,arm1,3
TX  TORQUE,arm1,1
RX  ERR,NO_TARGET,arm1
```

第二段說明失敗後資格已清除；不表示馬達已卸力或已停止。

<a id="section-8"></a>

## 8. 錯誤碼與處理

### 8.1 格式

```text
ERR,<code>[,<arm>][,<id>][,<mask>]
```
括號表示依錯誤種類出現的欄位，不是所有錯誤都可任意帶齊。
mask 僅出現在 DXL_MOTOR；ID 僅用於單顆讀取的錯誤定位。

#### BAD_CMD

回覆格式：`ERR,BAD_CMD[,<arm>]`

不支援的命令，例如 BEGIN/PT/END/STOP；有辨識到合法 arm 才附 arm。
ROS 修正命令，不用舊指令重試。

#### BAD_ARG

回覆格式：`ERR,BAD_ARG[,<arm>]`

欄位數、arm、整數格式／32-bit 溢位、torque 值或 LED 狀態不合法。

- **例：** `AX,arm1,512 -> ERR,BAD_ARG,arm1。`

#### RANGE

回覆格式：`ERR,RANGE,<arm>`

AX 整數可解析但超出 0..1023，或板內 HOME 值超範圍。

- **例：** `AX,arm2,1024,512,512,512 -> ERR,RANGE,arm2。`

#### NO_TARGET

回覆格式：`ERR,NO_TARGET,<arm>`

未建立可啟用目標資格；先處理先前失敗原因，再明確設定目標，不應只反覆 TORQUE,1。

#### INIT_FAILED

回覆格式：`ERR,INIT_FAILED 或 ERR,INIT_FAILED,<arm>`

前者為開機事件，後者為馬達命令被鎖定。控制板重啟前不解除。

#### OVERFLOW

回覆格式：`ERR,OVERFLOW`

主機行過長或 UART 丟資料；損壞內容捨棄至下一行尾。
可能在整行收完前即發出，且丟棄中的內容不會各自回 ACK；ROS 停止排程並重新同步連線。

#### DXL_TX

回覆格式：`ERR,DXL_TX,<arm> 或 ERR,DXL_TX,<arm>,<id>`

無 ID：位置／torque SYNC_WRITE 本地傳送失敗。
有 ID：READ/HOLD 的單顆讀取請求傳送失敗。
傳送失敗不能證明馬達沒有動作；不自動重送。

#### DXL_TIMEOUT

回覆格式：`ERR,DXL_TIMEOUT,<arm>,<id>`

單顆讀取在接收期限內沒有資料。檢查馬達 ID、供電、線路、速率及回覆等級。

#### DXL_CORRUPT

回覆格式：`ERR,DXL_CORRUPT,<arm>,<id>`

回覆 ID、長度、checksum 不符，或收到部分資料後到期。

#### DXL_RX

回覆格式：`ERR,DXL_RX,<arm>,<id>`

HAL 接收失敗或其他非成功接收狀態。

#### DXL_MOTOR

回覆格式：`ERR,DXL_MOTOR,<arm>,<id>,<mask>`

有效 status packet 的 error bits 非零。mask 為十進位位元遮罩，可同時包含多項：
1 電壓、2 角度限制、4 過熱、8 範圍、16 checksum、32 過載、64 指令。
128 為保留位；實作亦拒絕非零值。例 mask=5 代表位元 1 與 4。

#### DXL_RANGE

回覆格式：`ERR,DXL_RANGE,<arm>,<id>`

讀回位置不在 0..1023；不要發布為有效位置或視為到位。

### 8.2 判斷順序

整行拆分／過多欄位 -> PING/VERSION/LED/GET_HOME 分支 -> 未知命令 -> arm -> 初始化鎖定 -> 各命令參數 -> DXL 操作。
AX 各位置依 j1..j4 順序解析並檢查範圍；先遇到的錯誤先回報，不保證全欄位格式錯誤優先於前一欄越界。
單顆回讀：transport -> 馬達 error bits -> 位置範圍。

### 8.3 主機異常策略（ROS 端建議）

- 完整 ERR：結束本筆命令並停止本次動作流程；記錄 arm、命令、code、ID、mask。
- 逾時／錯配／重啟／斷線：將待處理結果視為未知，停止發送後續動作並使舊監測失效。
- 軌跡取消：先停止主機佇列新增點；若連線同步且上一筆已結束，可逐臂明確送 HOLD。
- 若連線已逾時或不同步，不要把 HOLD 當成可繞過收發同步的急停封包；先處理通訊與現場狀態。
- 沒有韌體 STOP、watchdog、自動 torque-off 或斷線 HOLD，主機不可假設控制板會自動補做。
- 故障解除後重新取得位置並明確建立目標；不要自動接續舊軌跡。

<a id="section-9"></a>

## 9. ROS 通訊層實作建議

此節是 ROS 端設計建議，名稱與 topics/actions 可依小組現有架構決定；不是新增的序列命令。

### 9.1 單一連線管理者

由一個通訊管理者持有 serial，兩臂請求共用一個有界佇列與一筆 pending request。
收發可用專用 worker 或非阻塞事件處理，避免 READ 等候阻塞其他 ROS callback。
即使上層同時提出兩臂請求，線上仍須逐筆完成，不可讓各 callback 各自 write/read。
連線故障後清除或取消舊佇列，避免恢復時突然執行過期位置。

### 9.2 每筆請求保存資料

命令種類、arm、目標四軸／torque／LED 值、預期回覆形式、發送時間、截止時間及所屬動作。
解析到資料行時按第 2.4 節核對，不能只測試字串是否以 OK 開頭。
成功 READ 更新 measured_position；成功 AX/HOME 更新 requested_target，兩者分開儲存。
HOLD ACK 沒有回傳其目標快照，勿把上一筆 AX 當成 HOLD 的新目標。

### 9.3 建議可配置項目

serial port、baud（固定匹配 57600）、command_timeout、startup_listen、每臂關節對應及校正參數、READ 採樣間隔、每軸到位容差、stable_count、arrival_timeout。
韌體未承諾固定控制頻率，需以實機測量 round-trip 與雙臂混合負載決定。
ROS 記錄至少包含送出命令、完整回覆、主機時間、結果與失敗原因。

<a id="section-10"></a>

## 10. 對接驗收與檔案索引

建議按以下順序驗收 ROS ↔ CM-530：

1. VERSION/PING 正常，舊版本與錯配回覆被拒絕。
2. GET_HOME 兩臂值與韌體設定一致，沒有寫入馬達。
3. AX 正確映射兩臂四軸；未設定目標時 TORQUE,1 被拒絕。
4. HOME 不自行開 torque；卸力後重新啟用要求新目標。
5. READ 四軸順序正確；單顆故障回報對應 ID，不回部分 POS。
6. HOLD 全讀成功才寫目標；部分失敗資格無效，另一臂不受影響。
7. CRLF 分段接收、長行、錯欄位、延遲回覆、重啟與斷線不導致舊 ACK 被當成新成功。
8. 雙臂交錯 AX/READ 使用同一收發管理者，沒有兩筆 pending request。
9. 終點到位使用實際 READ、多次穩定與總期限，不依 AX/HOME ACK 直接完成。
10. 取消或故障不自動恢復舊軌跡；HOLD/卸力行為符合上層明確操作。

本次韌體與終端離線測試已通過，ROS 節點與實機整合仍須另行驗收。

原始碼與查閱位置（相對於第 17 版資料夾）：

| 檔案 | 用途 |
|---|---|
| [APP/src/bridge.c](APP/src/bridge.c) | 命令解析、回覆、資格與初始化鎖定 |
| [APP/src/ax12.c](APP/src/ax12.c) | ID 映射、SYNC_WRITE、位置讀取與錯誤分類 |
| [APP/inc/arm_config.h](APP/inc/arm_config.h) | home_A/home_C 初值與後續校正入口 |
| [manual_position_terminal.py](manual_position_terminal.py) | Python 收發與回覆核對參考；不是 ROS 節點 |
| [README.md](README.md) | 操作與建置入口 |
| [VALIDATION.md](VALIDATION.md) | 已執行測試、工具限制、待做實機項目 |

韌體與主機需匹配協定 5；第 16 版資料夾保留歷史規格。
