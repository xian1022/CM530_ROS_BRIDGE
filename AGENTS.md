# Repository Guidelines

## Project Structure & Module Organization

The active firmware is in `17 ROS to CM530 ver. dual arm/` (serial protocol 5). Numbered directories `01`–`16` preserve examples and earlier implementations; avoid changing them unless explicitly required.

- `APP/src/` and `APP/inc/`: bridge parser, AX12 control, LED adapter, and configuration headers.
- `stm32f10x_lib/`: vendor STM32 SDK; keep vendor changes narrowly scoped.
- `tests/`: C harnesses, fake GPIO headers, Python terminal tests, and firmware verification.
- `manual_position_terminal.py`: host serial CLI; `requirements.txt` declares pyserial.
- `Makefile`, `stm32.ld`, and `CM530.*`: build configuration and firmware artifacts.

## Build, Test, and Development Commands

Run commands from the active version directory in PowerShell:

```powershell
python -m pip install -r requirements.txt
python manual_position_terminal.py --self-test
./tests/run_tests.ps1 -Python python -Compiler gcc
python tests/verify_firmware.py
```

These install CLI dependencies, run Python unit tests, run Python plus both C harnesses, and verify existing ELF/HEX/BIN consistency, respectively. Host tests require no board. Zig is supported through `-Compiler <zig.exe> -CompilerArgs cc`.

For a complete ARM build, use:

```powershell
make -B TCHAIN_PREFIX=arm-none-eabi- "COMPILE_OPTS=-mcpu=cortex-m3 -mthumb -Wall -g -Os -fno-common -fno-strict-aliasing -Wno-error=implicit-function-declaration -Wno-error=incompatible-pointer-types -Wno-error=int-conversion" CM530.hex CM530.bin
```

Reverify artifacts and update `SHA256SUMS` after firmware or HOME configuration changes.

## Coding Style & Naming Conventions

Follow neighboring code: four-space indentation, GNU89-compatible C, and opening function braces on separate lines. Preserve C API prefixes such as `Bridge` and `Ax12`; use uppercase configuration macros. Python uses snake_case functions and `test_*` methods. No shared formatter configuration was found; host C compilation enforces `-Wall -Wextra -Werror`.

## Testing Guidelines

Python uses `unittest`; C harnesses exercise production code with simulated HAL/GPIO. Extend `tests/test_terminal.py`, `test_bridge.c`, or `test_arm_led.c` for behavioral changes. Cover malformed input, failure recovery, and arm isolation. No numerical coverage threshold is defined.

Record all verification commands, results, failures, and pending checks in `驗證與測試總整理.md`. Distinguish historical evidence from fresh runs and hardware observations from simulations. Flashing and motor movement require explicit authorization; ACK does not prove physical arrival.

## Commit & Pull Request Guidelines

History uses concise imperative subjects, sometimes prefixed `docs:`. Follow examples such as `Set v17 HOME presets and rebuild firmware`. PRs should identify the affected version, behavior changes, validation results, hardware limitations, and related issues when available. Explain protocol or configuration changes and include artifact hashes when rebuilding firmware.
