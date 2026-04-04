# CM530_ROS_BRIDGE

## 專案簡介

這個專案是把 **ROBOTIS CM-530** 改寫成一個「橋接控制器」，
讓電腦端（ROS / MoveIt）可以透過 Serial 控制 **AX-12A 馬達**，進而操作機械手臂。

---

## 系統在做什麼

整體流程如下：

```text
ROS / MoveIt
     ↓
Serial (USB)
     ↓
CM-530（本專案韌體）
     ↓
DYNAMIXEL Bus
     ↓
AX-12A 馬達
```

簡單說：

👉 ROS 算好角度
👉 CM-530 幫忙轉成馬達指令
👉 AX-12A 執行動作

---

## 核心功能

* 接收 ROS 傳來的關節角度（rad）
* 將角度轉換成 AX-12A 的位置值
* 產生 DYNAMIXEL Protocol 1.0 封包
* 控制多顆馬達同步運動
* 回傳簡單 ACK（OK / ERR）

---

## 專案重點資料夾

```text
12 CM530_BRIDGE_MODE   ← ⭐ 主要韌體（你要看的重點）
其他資料夾            ← 官方 SDK（基本沒改）
```

---

## ROS 傳送格式（簡化版）

```text
PT,seq,dt_ms,j1,j2,j3,...
```

範例：

```text
PT,0,50,0.0,-0.3,1.1,0.2
```

說明：

* `seq`：第幾個點
* `dt_ms`：時間間隔（毫秒）
* `j1~jN`：各關節角度（rad）

---

## 編譯方式

```bash
make all
```

會產生：

```text
CM530.bin
```

---

## 燒錄方式

使用 RoboPlus Terminal：

1. 進入 Boot Loader
2. 上傳 `.bin`
3. 重開機

---

## 測試方式

```text
PING
→ PONG
```

或直接送：

```text
PT,0,50,0,0,0,0
```

---

## 總結

👉 這個韌體讓 CM-530 變成
**「ROS → AX-12A 的橋接器」**

