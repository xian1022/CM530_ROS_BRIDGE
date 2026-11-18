# 第 17 版實作紀錄
依使用者已核准的聊天計畫執行。
- 工作分支：codex/cm530-v17；在要求的獨立 17 資料夾內實作，保留 16 版。
- 工作項目：韌體與測試、終端與測試、文件與映像、最終審查。
- 介面：協定 5；HOME/GET_HOME/READ/HOLD。ROS 負責到位與任務判斷。
- READ/GET_HOME 不改目標資格；初始化失敗時 GET_HOME 可查，READ 與動作命令鎖定。
- 測試與建置紀錄於完成各階段後補入。

- 韌體 RED：未修改的 v16 對新 v17 C 測試失敗 323 個檢查；加入實作後全套通過。
- 終端 RED：25 個測試中 3 failures、2 errors；實作後 25/25 通過。
- 主機工具原有 Zig 不完整；另在系統暫存區安裝 ziglang 0.13.0，未更改產品依賴。
- 韌體與终端階段完成：Python 25/25、C bridge/AX12/SDK、C LED 全通過。
- 初次 ARM -B 建置成功；既有 HAL 宣告與 vector 警告保留，詳見 VALIDATION。
- 映像檢查修正：舊版搜尋 PT\0 誤中 DXL_CORRUPT 的字尾；改查完整 NUL 邊界，命令不支援另由正式 parser 測試驗證。

- 獨立審查：無新增 runtime 缺陷；版本 banner/驗證文件已更新，HOME Makefile 依賴已修正。
- 依新使用者規則完成 Trail of Bits sharp-edges/differential-review/harness review；ASan 無 runtime，已記錄限制。
- HOME 依賴修正 RED→GREEN：make -n -W 從 up-to-date 改為 bridge.o 重編。
- 未改變規劃分工；初始化失敗允許 GET_HOME，其餘馬達命令繼續鎖定，符合既有啟動失敗策略。
