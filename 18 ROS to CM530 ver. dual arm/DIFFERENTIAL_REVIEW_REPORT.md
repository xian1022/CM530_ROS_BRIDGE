# 第 17 版差異與 C API 審查

日期：2026-10-07；基線：9e94ba4，16 CM530 DUAL ARM；目標：codex/cm530-v17 的新增 17 資料夾。

## 結論

未確認新的高／中嚴重度執行邏輯缺陷。審查結论限於本次變更與離線測試，實機部署仍須完成 VALIDATION 清單。涉及馬達寫入的新命令列為高關注邊界，不將主機 ACK 當實際停止證據。

| 審查項目 | 結果 |
|---|---|
| 新增記憶體安全／權限跨臂缺陷 | 未確認 |
| 文件版本與終端標示 | 已改為協定 5 |
| HOME header 增量建置依賴 | 已補齊並以 make -n -W 驗證 |
| 實機、ASan、覆蓋率導向 fuzzing | 尚未驗證／未執行，見限制 |

## 變更及基線

git log／blame 確認 bridge 的有界輸入與資格管理源自 3ebf13b，LED/協定4源自 bc75aaf。本次保留那些驗證與復原機制；未移除已知安全修補。

詳細閱讀 bridge.c、ax12.c、ax12.h、arm_config.h、bridge.h 及主機終端差異；檢查 main.c、dxl_hal.c、dynamixel.c 一層依賴與測試。原廠 STM32 library 未全面重新稽核。

新增主要路徑：BridgeFeed → process → readPositions → Ax12ReadPositions → dxl_read_word；readPositions 有 1 個正式呼叫位置供 READ/HOLD 共用，Ax12ReadPositions 有 1 個正式呼叫者。HOME/AX/HOLD 共用 apply；無新增中斷處理器、動態記憶體或執行緒。

## 威脅邊界與誤用檢查

- 序列輸入不可信：固定 96-byte 行缓衝、欄位數、手臂、32-bit 十進位及 0..1023 範圍驗證；溢位捨棄至行尾，NUL 不能隱藏尾端指令。
- 馬達回覆不可信：接收器驗證 ID、長度、checksum 與有限期限；讀回值在 status 成功前不得使用。馬達 error bits 非零及位置越界拒絕。
- 部分讀取：使用區域 sample[4]，全成功才複製到呼叫者輸出；HOLD 全成功才送 SYNC_WRITE。
- 狀態誤用：開機無可啟用目標；HOLD 先撤銷舊資格，任何讀/寫失敗不恢復；另一臂不受影響。READ 不改資格。
- C 內部 API：Ax12ReadPositions 的 arm 由 parser 驗證為 0/1，兩個輸出指標均由內部有效堆疊物件提供；此介面不是對外的未驗證入口。
- 默認行為：HOME 常數為使用者明確指定的 512；HOME/HOLD 不自動 torque-on；板內 HOME 範圍在寫入前檢查。主機必須按回覆分類，資料回覆不能用 ACK 代替。
- HOLD 讀取與寫入有時間差，不能保證瞬間停止；ACK、LED STOPPED、TORQUE=0 與實際到位/保持的意義不同，文件與範例明確分開。
- ROS 是唯一任務與 B 區授權來源，沒有新增另一個互鎖管理者。主機一次一筆命令，避免多個寫入者互相搶回覆。

## 測試證據

Python 25/25、正式 C bridge+AX12+SDK 模擬 HAL、正式 LED adapter 測試均通過。新增測試先對舊實作失敗，後通過。HOLD 包含各軸故障、傳送失敗、無部分寫入與另一臂隔離。回讀後錯誤不把上次資料當成功。

使用 harness-writing 指引檢查現有 HAL harness：每案例 reset 狀態、接收期限由可重現輪詢計數模擬，無實體馬達 I/O；測試中的封包與輸出收集設邊界檢查。此次未新增或宣稱 coverage-guided fuzzing，也沒有量測覆蓋率百分比。

HOME 建置依賴修正前 make -n -W APP/inc/arm_config.h 顯示 up to date；修正後會重編 bridge.o、重連 ELF 並產生 BIN。

## 限制

套用 sharp-edges、address-sanitizer、differential-review、harness-writing 指引。Windows Zig 0.13.0 加 -fsanitize=address 可編譯但因 __asan_* 缺少 runtime 而連結失敗；PATH 無 clang/clang-cl，WSL Ubuntu 無 gcc/clang。**ASan 未成功執行**，普通測試不能取代它。未安裝新的系統分析工具。

未跑 comprehensive c-review：目標是 bare-metal 韌體，該工作流程排除此類專案。本次不是安全漏洞修補，不宣稱執行 post-patch-validation 的完整 exploit 工作流程。

原廠 HAL/vector 10 個既有 ARM 警告、UART 實體噪聲／電氣問題、馬達接收與動作、ISR 時序、機構碰撞與停止距離均仍需實機驗收。沒有已確認但未修正的新增執行缺陷。
