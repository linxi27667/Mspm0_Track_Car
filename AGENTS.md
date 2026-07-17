# Track_Car Project Guide

## Target

- MCU: TI MSPM0G3507
- Runtime: bare metal / noRTOS
- Toolchains: Keil MDK-ARM (primary), TI Clang and GCC project files also exist
- Vehicle: eight-channel grayscale line sensor, dual encoder motors, ICM42688-P, ESP-01S using AT firmware
- Host: Python Flask + Flask-SocketIO web application

## Architecture

- `BSP/`: reusable device logic and low-level drivers
- `APP/`: hardware binding, algorithms, ESP-01S and tuning protocol
- `TASK/track.c`: line-following state machine and control orchestration
- `main.c`: initialization, interrupts and non-blocking time-slice scheduler
- `host/`: TCP bridge, tuning engine, simulator and browser UI
- `tests/`: host-executed logic and protocol tests

Keep the 5 ms tracking loop non-blocking. ISR code may only service hardware and publish bounded state; parsing, logging and business logic belong in the main loop. On invalid sensor data, motor output must fail safe.

## Host Role

- The host is debug and logging only. It must never be a motion dependency.
- Host disconnect, TCP/Wi-Fi loss, ESP link drop, or missing heartbeat must not stop, pause, or alter any ongoing motion.
- Vehicle motion is command-first: accepted onboard commands and the local state machine own motion; host UI is optional observability and tuning.
- Every command that changes vehicle motion still needs validation and an observable acknowledgement when a host is connected; lack of host does not block execution.

## Build And Test

Keil project:

```powershell
& 'C:\Keil_v5\UV4\UV4.exe' -b '.\keil\gpio_toggle_output_LP_MSPM0G3507_nortos_keil.uvprojx' -j0 -o build.log
```

Host and portable tests:

```powershell
python -m unittest discover -s tests -v
python -m compileall -q host tests
```

Run the host and simulator from the repository root:

```powershell
python host\app.py
python host\simulator.py
```

## Change Discipline

- Preserve generated `ti_msp_dl_config.c/.h` manual fixes unless SysConfig regeneration is explicitly intended.
- Keep tunable parameter ranges identical in firmware, host protocol and UI.
- Do not flash or erase a connected board until the exact debug probe/port and safe motor state are confirmed.
- Do not overwrite user-modified build logs or unrelated working-tree changes.
