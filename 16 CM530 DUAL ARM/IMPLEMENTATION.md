# Protocol 4 LED implementation ledger

Authority: user-approved dual-arm LED and silent-terminal plan, 2026-09-30.
Scope: existing v16 firmware/terminal/docs/tests/images; preserve 01 LED and unrelated workspace changes.
Mapping confirmed by user: arm1 MANAGE PB13 green / PROGRAM PB14 red; arm2 TX PC14 green / RX PC15 red.
Boot red means standby, not measured stopped; only explicit ROS LED changes indicators thereafter.
Pre-flight: LED parser -> platform callback, active-low GPIO; terminal must quietly validate exactLEDACK; docs and images must identify version4.
Execution: reuse authorized current checkout, no commit or reset.
Task 1 complete: new LED protocol tests RED against protocol3 (C29 checks, Python4fail/3error), then GREEN after parser/callback and silent-terminal implementation.
Task 2 complete: GPIO module integrated in main/Makefile; real adapter tested with fake GPIO/RCC for pin masks, polarity, off-before-on, independent arms and POWER/UART preservation.
Task 3 complete: root README, v16 README and ROS contract updated for protocol4, LED mapping, silent normal terminal output and unchanged motor responsibilities.
Task 4 complete: 24 Python + C bridge/SDK + GPIO suites pass; ARM full build + ELF/HEX/BIN verifier pass; hashes and validation record updated. Physical tests remain pending.
Review limitation: fresh independent reviewer failed due tool usage cap; no independent protocol4 review claimed. Primary performed code/spec cross-check.
Ruling: LED handled before motor init lock so failed motor startup still permits GPIO display. No extra motor behavior or auto LED actions.

---
Historical records (superseded):

# Protocol 3 implementation ledger

Authority: user-approved ROS -> AX-12A direct bridge plan, 2026-09-29.
Scope: v16 firmware, terminal, tests, root/v16 READMEs and ROS contract; no ROS nodes.
Ruling: reuse explicitly authorized current checkout; preserve other folders and existing changes; no commit/reset.
Pre-flight: firmware and terminal must agree on READY/VERSION 3, explicit four-value AX, TORQUE replies; image checker and docs must match.
Task 1: new C tests observed failing protocol 2 version/legacy command behavior and runtime READ assertion.
Task 2: new terminal tests observed failing legacy commands, version and demo behavior.
Test fixture corrections: AX ACK has no echoed positions; generic SDK HAL receive error is COMM_RXFAIL.
Task 1/2: complete; full production C suite and 14 Python tests passed after protocol 3 implementation.
Task 3: complete; root README, v16 README and full ROS contract updated to four-state FSM, B lock, C 2x4 minimum empty slot, write-only protocol and migration.
Task 4: ARM full build and ELF/HEX/BIN verifier passed; SHA256SUMS updated, physical validation explicitly pending.
Final review resolved: independent reviewer rechecked fix and 18 Python tests; no remaining Critical/Important issues. Documentation cross-check: 13 local links, 11 success TX/ACK pairs and three SHA-256 hashes passed.
Final review: found queued RX could allow next command before detecting idle-time restart/stale ACK. Reproduced with 3 failing new tests; added pre-send bounded pending-input check; full 18 Python + C tests now pass.
Ruling: preserve existing SDK receiver hardening and known ARM SDK warnings; no hardware operations, ROS code, commits or changes to historical folders.
Ruling: SDK receive regression tests remain SDK-only; protocol 3 runtime must never issue READ.

---
Historical protocol 2 implementation record (superseded by protocol 3 above):

# Protocol 2 implementation ledger

Authority: user-approved 第 16 版 CM530 雙臂轉接韌體修改計畫 in this task.
Scope: v16 serial-to-AX12A firmware, terminal, tests, documentation. User expanded documentation scope on 2026-09-29 to include root README.

## Tasks

1. Extend physical-HAL tests for explicit torque, HOLD, boot failures and trajectories; implement bridge and motor layer.
2. Exercise malformed/fragmented SDK replies; bound receive parsing and 50 ms read timeout.
3. Test and update terminal commands, startup identification and abort behavior.
4. Update interface documentation, build/verify images and hashes, independent final review.

## Decisions and evidence

- User authorized editing the current main checkout with 「開始修改」 after the workspace question. Existing untracked v16 content and other working-tree changes are preserved; no automatic commit, reset or worktree transfer.
- Original directory copied to OS temporary directory `cm530-v16-before-protocol2` for review against the actual uncommitted baseline.
- Baseline Python terminal: 10 tests passed with Python 3.14.
- Ruling: retain PING/PONG and add VERSION -> VERSION,2. A terminal attaching after READY has already been emitted needs a non-motion protocol-version handshake; otherwise it could operate old firmware. No ROS node is added.
- Ruling: HOLD invalidates enable eligibility before reading. If HOLD fails, an older goal must not authorize a later TORQUE,1. Cost: caller must send a fresh goal after failure.
- Ruling: firmware checks initialization in the bridge as well as startup adapter; PING/VERSION remain diagnostic while all motion commands fail INIT_FAILED.
- Pre-flight: motor-layer TX results feed target eligibility and trajectory counters; terminal must match the same ACK/error/version grammar; firmware image checks must match READY,2 and the new motor ID table.

## Progress

- Task 1 complete: baseline C passed; new tests failed old behavior; boot/torque and trajectory suites passed after bridge + AX12 layer implementation.
- Task 2 complete: SDK length=255 reproduced process crash; bounded receiver made full HOLD suite pass. Each transaction deadline now uses DXL_RX_TIMEOUT_MS=50.
- Task 3 complete: Python tests RED (6/14 failed) -> GREEN (14/14 passed), including version handshake and torque ACK validation.
- Task 4 complete: firmware built/verified, hashes updated, root README and v16 README/spec expanded and checked against implementation.
- Ruling: any failed goal write also clears eligibility because a partial write cannot safely authorize torque against a previous target. Cost: send a fresh target before enabling.
- Verification: full tests/run_tests.ps1 with Python 3.14 + Zig 0.13 cc: Python 14/14 and C suite PASS; ARM 14.2 full build PASS with existing HAL/vector warnings.
- Independent final review (protocol2_review): first attempt failed due to usage limit; resumed 2026-09-29 and completed. No Critical/Important findings. Reviewer independently ran C executable, Python tests and artifact checks; all passed.
- Final: minor (deferred): C HAL fake clears RX on every TX; stale/unread-byte recovery across consecutive HOLD commands without reset is not directly protected by a regression test. Current receiver clears before unicast, but simulated stale-buffer behavior could be improved.
- Final: Ruling: physical timing, turnaround, torque and mechanics remain unverified because hardware was not connected; keep them explicitly pending, not infer success from simulation.
- Final: Ruling: inherited HAL prototype/vector warnings and unchanged UART adapter limitations stay outside this protocol update; disclose build warnings and require hardware validation.
- Final: Ruling: ROS orchestration/collision/suction/telemetry are intentionally external, per approved scope; documentation defines responsibility without implementing nodes.
- Reviewer excluded documents under active editing; root and v16 documentation are checked separately against source and examples by the implementer.
- Integration: user authorized in-place edits on main; preserve uncommitted changes as requested. No automatic commit/merge/push, no worktree cleanup.

- Final verification 2026-09-29: run_tests.ps1 rebuilt host C with -Werror and ran full C suite plus Python 14/14; all passed. verify_firmware.py and SHA256SUMS checks passed. Documentation QA checked UTF-8, balanced fences, 13 local links and 22 TX examples/ACK pairs; git diff --check on root README passed.
