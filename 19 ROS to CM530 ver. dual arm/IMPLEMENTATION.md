# 第 17 版實作結構

- `APP/inc/arm_config.h`：集中 HOME 常數；由 bridge 唯一包含。
- `APP/src/bridge.c`：協定 5 解析、回覆、每臂目標資格與 HOME/READ/HOLD 流程。
- `APP/src/ax12.c`：固定 ID 映射、四軸 SYNC_WRITE、逐顆 READ 及錯誤分類。回讀全成功才提交輸出陣列。
- `APP/src/dynamixel.c`：沿用第 16 版有界接收器，檢查 ID/長度/checksum/期限。
- `APP/src/main.c`、`dxl_hal.c`、`stm32f10x_it.c`：沿用原廠初始化/UART/計時器適配。
- `APP/src/arm_led.c`：沿用第 16 版燈號。
- `manual_position_terminal.py`：協定 5 客戶端；資料回覆按種類/手臂/四軸範圍核對。
- `tests/test_bridge.c`：正式 bridge/AX12/SDK，只模擬 HAL；測試開機、解析、隔離、回讀故障與 HOLD。
- `tests/test_arm_led.c`：正式 LED adapter 的 GPIO 測試。
- `tests/test_terminal.py`：25 個主機終端與 CLI 測試。
- `tests/verify_firmware.py`：ELF/HEX/BIN 一致性、位址與協定識別檢查。

底層仍使用舊 STM32 SDK 的 Makefile 與 linker 配置；修改 header 後必須 `make -B` 完整重建。
ROS 任務狀態、到位閾值、B 使用權與吸盤狀態均不存於韌體。
