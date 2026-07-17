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
- Always emit startup and calibration diagnostics for vehicle peripherals, except manually disabled ESP-01S details. Gate periodic vehicle and sensor tuning logs by the relevant motion state, and keep idle/finished states quiet.

## Remote Gyro Rotation

- TC1 `cmd:"rotate"` takes a signed nonzero `deg` in the inclusive range `-180..180`: positive is left/counter-clockwise and negative is right/clockwise.
- Rotation is accepted only from Safety READY with tracking idle and a calibrated, fresh ICM42688-P frame. The 5 ms rotation state closes on relative yaw, slows near target, corrects overshoot, and returns to READY after three settled frames.
- A failed ICM42688-P read invalidates the current sample and stops rotation. Encoder fault, safety revocation, stop command, and angle timeout also cut motion. Host disconnect or missing heartbeat must not cut motion.
- Host UI exposes ±90 degree buttons and a custom signed angle. Telemetry rotation fields are `tg`, `ta`, `te`, `rr`; the shared TC1 JSON ceiling is 384 bytes.

## First Track Test Profile

- The initial real-car tracking run uses conservative direct-PWM values from `APP/Inc/user_tuning.h`: base 28, straight 30, curve 26, corner approach 30, capture 26, and pivot maximum 40.
- Keep ESP tuning enabled for the initial run. Vehicle, track, grayscale digital, IMU, and 1 Hz grayscale ADC logs must remain enabled until the first straight, single-corner, and one-lap evidence has been reviewed.
- A real-car trace proved that mask `0x0E` is ordinary left drift, not a square corner. Corner entry must retain the S3/S4 center pair, extend through S2 to S0/S1, and use a four-black-channel default threshold. ESP overflow must discard complete log lines so diagnostic frames are never emitted as fragments.

## Square Track Control

- Eight-channel masks are classified as normal contiguous line, counter-clockwise left-corner candidate, lost line, or invalid pattern. Normal 1-3 channel runs use the `-3.5..+3.5` centroid; invalid patterns hold one frame and stop on the third consecutive frame.
- A corner needs two preceding centered frames and three consecutive `0x1E`/`0x1F`-style left-chain frames. `0x0E` and `0x0F` must never trigger a corner.
- Corner motion is non-blocking: encoder-distance approach with entry-heading hold, 100 ms stop, IMU +90 degree pivot, then three consecutive normal centered capture frames. Only a successful capture increments the corner count.
- `USER_ENCODER_LEFT_COUNTS_PER_MM`, `USER_ENCODER_RIGHT_COUNTS_PER_MM`, and `USER_SENSOR_TO_AXLE_MM` intentionally default to zero. Straight tracking remains available, but corner entry fails safe with `approach_cal_missing` until all physical values are measured and compiled in.
- READY requires every grayscale channel to have at least 200 ADC counts between wide-white and wide-black calibration. Line tracking defaults to direct PWM; `USER_TRACK_SPEED_PID_ENABLE` is enabled only after independent left/right wheel calibration.
