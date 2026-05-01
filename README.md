# CM530_ROS_BRIDGE

## 專案簡介（Project Overview）

本專案基於 [ROBOTIS 官方 Embedded C SDK](https://emanual.robotis.com/docs/en/software/embedded_sdk/embedded_c_cm530/)，對 CM-530 控制器韌體進行修改與重構，
實作一個 **ROS 與 DYNAMIXEL AX-12A 之間的橋接控制系統（Bridge Firmware）**。

系統由 ROS2 / MoveIt 產生關節軌跡，透過 Serial 傳送至 CM-530，
再由 CM-530 轉換為 DYNAMIXEL Protocol 1.0 指令，控制機械手臂動作。

---

## 系統架構（System Architecture）

```text
OpenCV (optional)
        ↓
ROS2 / MoveIt
        ↓
Serial (USB)
        ↓
CM-530 (Bridge Firmware)
        ↓
DYNAMIXEL TTL Bus
        ↓
AX-12A
        ↓
PhantomX Pincher
```

說明：

* ROS 負責軌跡規劃與控制邏輯
* CM-530 負責通訊解析與馬達控制
* AX-12A 負責實際關節運動

---

## 專案架構（Repository Structure）

```text
CM530_ROS_BRIDGE/
└── 12 CM530_BRIDGE_MODE/       ← 本專案主要韌體
    ├── APP/
    │   ├── inc/
    │   │   ├── dxl_hal.h
    │   │   ├── dynamixel.h
    │   │   └── stm32f10x_it.h
    │   └── src/
    │       ├── main.c
    │       ├── dxl_hal.c
    │       ├── dynamixel.c
    │       ├── stm32f10x_it.c
    │       └── syscalls.c
    │
    ├── stm32f10x_lib/          ← 官方 STM32 Library
    ├── Makefile
    ├── stm32.ld
    ├── stm32f10x_conf.h
    ├── CM530.bin / .elf / .hex
    └── test_cm530.py
```

說明：

* `12 CM530_BRIDGE_MODE`：本專案核心（基於官方 SDK 修改）
* 其餘結構為 ROBOTIS 官方 Embedded SDK

---

## 核心功能（Core Functionality）

### 1. Serial 指令解析

CM-530 透過 UART 接收 ROS 傳來的 ASCII 指令，並逐行解析。

### 2. 軌跡播放（Trajectory Playback）

支援多點軌跡（Trajectory Points）逐點執行，包含：

* 序列檢查（seq）
* 時間間隔控制（dt_ms）
* 多關節同步更新

### 3. 整數位置控制（Integer AX Position）

本版本採用 **AX-12A 整數位置控制（0~1023）**，避免浮點運算問題。
ROS 需先完成 rad → position 轉換，再傳送至 CM-530。

### 4. DYNAMIXEL 控制

透過官方函式：

* `dxl_write_word()` → 設定 Goal Position
* `dxl_read_word()` → 回讀 Present Position

使用 Half-Duplex UART 與 AX-12A 通訊。

### 5. 回饋機制（Feedback）

每個軌跡點會：

* 回讀實際位置
* 輸出狀態資訊
* 回傳 ACK（OK / ERR）

### 6. 最終到位確認

在 `END` 指令後，系統會等待所有馬達進入容差範圍內才結束。

---

## 通訊格式（Communication Protocol）

### Serial 設定

```text
Baud Rate : 57600
Data Bits : 8
Parity    : None
Stop Bits : 1
Format    : ASCII
Line End  : \n
```

---

### 指令格式

```text
PING
BEGIN,<traj_id>,4,<point_count>
PT,<seq>,<dt_ms>,<j1_pos>,<j2_pos>,<j3_pos>,<j4_pos>
END,<traj_id>
STOP
HOME
```

---

### 範例

```text
BEGIN,1,4,3
PT,0,50,512,430,760,580
PT,1,45,520,440,748,590
PT,2,60,530,455,730,602
END,1
```

---

### 馬達對應（Joint Mapping）

```text
j1 → ID17
j2 → ID3
j3 → ID2
j4 → ID7
```

---

## 核心程式說明（Key Modules）

### `main.c`

主要控制流程：

* 系統初始化（GPIO / UART / Timer）
* 初始化 DYNAMIXEL bus
* 初始化 Joint 與 Trajectory 狀態
* 持續接收並解析指令
* 控制 AX-12A 執行動作

程式內實作：

* 指令解析（PING / BEGIN / PT / END / STOP / HOME）
* 軌跡狀態機（Trajectory State Machine）
* 多馬達控制與回饋
* 最終到位檢查機制

---

### `dxl_hal.c`

負責 DYNAMIXEL 半雙工 UART 底層控制。

---

### `dynamixel.c`

封裝 DYNAMIXEL Protocol 1.0 指令操作。

---

## 硬體需求（Hardware）

* ROBOTIS CM-530
* DYNAMIXEL AX-12A（4 顆）
* PhantomX Pincher（或等效機械手臂）

---

## 編譯方式（Build）

```bash
make all
```

輸出：

```text
CM530.bin
```

---

## 燒錄方式（Flash）

透過 RoboPlus：

1. 進入 Boot Loader 模式
2. 上傳 `.bin`
3. 重啟控制器

---

## 作者資訊（Author）

* 姓名：xian1022

---

## 說明（Notes）

* 本專案基於 ROBOTIS 官方 SDK 修改
* CM-530 僅負責執行，不進行浮點計算
* ROS 需負責角度轉換（rad → AX position）
