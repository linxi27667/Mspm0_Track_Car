# Track_Car Project Guide

## Target

- MCU: TI MSPM0G3507
- Runtime: bare metal / noRTOS
- Toolchains: Keil MDK-ARM (primary), TI Clang and GCC project files also exist
- Vehicle: eight-channel grayscale line sensor, dual encoder motors, SeekFree IMU660RB (LSM6DSRTR), ESP-01S using AT firmware
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
- Rotation is accepted only from Safety READY with tracking idle and a calibrated, fresh IMU660RB frame. The 5 ms rotation state closes on relative yaw, slows near target, corrects overshoot, and returns to READY after 20 continuous settled frames (100 ms).
- A failed IMU660RB read invalidates the current sample and stops rotation. Encoder fault, safety revocation, stop command, and angle timeout also cut motion. Host disconnect or missing heartbeat must not cut motion.
- Host UI exposes ±90 degree buttons and a custom signed angle. Telemetry rotation fields are `tg`, `ta`, `te`, `rr`; the shared TC1 JSON ceiling is 384 bytes.

## IMU660RB

- The SeekFree IMU660RB is based on ST LSM6DSRTR and remains on SPI1: PB6 CS, PB7 MISO, PB8 MOSI, PB9 SCLK.
- SPI uses Mode 0 at 8 MHz. Firmware verifies `WHO_AM_I=0x6B`, enables register auto-increment and block-data-update, and reads temperature, gyro and accelerometer as one 14-byte little-endian burst from `0x20`.
- Accelerometer and gyroscope run at 416 Hz with ±4 g and ±500 dps ranges. This keeps fresh samples available to the non-blocking 5 ms control loop.
- Startup calibration accepts only samples whose acceleration magnitude is near 1 g and whose Z rate is below 3 dps. It derives both the zero-rate offset and a noise-based dead zone; repeated motion or excessive standard deviation fails calibration in bounded time.
- After calibration, motor-disabled 5 ms ticks confirm 100 stationary samples before slowly adapting the zero-rate offset. Bias learning is frozen whenever motion is authorized. The heading path applies a one-pole rate filter, dynamic dead zone and trapezoidal integration, while qualified stationary samples contribute zero heading drift.
- Flat `0x00`/`0xFF` frames and transport failures invalidate the sample immediately. Before motor-enabled rotation, bench-check that positive Z yaw still corresponds to the project's positive left/counter-clockwise convention; adjust `IMU660RB_GYRO_Z_SIGN` only if the mounted module reports the opposite sign.

## First Track Test Profile

- The initial real-car tracking run uses conservative direct-PWM values from `APP/Inc/user_tuning.h`: base 28, straight 30, curve 26, corner approach 30, capture 26, and pivot maximum 40.
- Keep ESP tuning enabled for the initial run. Vehicle, track, grayscale digital, IMU, and 1 Hz grayscale ADC logs must remain enabled until the first straight, single-corner, and one-lap evidence has been reviewed.
- A real-car trace proved that mask `0x0E` is ordinary left drift, not a square corner. Corner entry must retain the S3/S4 center pair, extend through S2 to S0/S1, and use a four-black-channel default threshold. ESP overflow must discard complete log lines so diagnostic frames are never emitted as fragments.

## Square Track Control

- Eight-channel masks are classified as normal contiguous line, counter-clockwise left-corner candidate, lost line, or invalid pattern. Normal 1-3 channel runs use the `-3.5..+3.5` centroid; invalid patterns hold one frame and stop on the third consecutive frame.
- A corner needs two preceding centered frames and three consecutive `0x1E`/`0x1F`-style left-chain frames. `0x0E` and `0x0F` must never trigger a corner.
- Corner motion is non-blocking: encoder-distance approach with entry-heading hold, 100 ms stop, IMU +90 degree pivot, then three consecutive normal centered capture frames. Only a successful capture increments the corner count.
- On a short lost/invalid line frame, the line PID is frozen to avoid integrating stale error. The car immediately drops to recovery speed and combines a bounded last-line proportional term with IMU heading P + yaw-rate D control toward the last valid line heading; failure to reacquire within 600 ms stops the car.
- The right-angle sequence has no stop-producing duration or retry watchdog. A 3.5 s phase timer only alternates capture and heading correction until a normal line is found or the operator stops the run. Path Fish transport faults and unhealthy IMU data still stop immediately.
- `USER_ENCODER_LEFT_COUNTS_PER_MM`, `USER_ENCODER_RIGHT_COUNTS_PER_MM`, and `USER_SENSOR_TO_AXLE_MM` intentionally default to zero. Straight tracking remains available, but corner entry fails safe with `approach_cal_missing` until all physical values are measured and compiled in.
- READY requires every grayscale channel to have at least 200 ADC counts between wide-white and wide-black calibration. Line tracking defaults to direct PWM; `USER_TRACK_SPEED_PID_ENABLE` is enabled only after independent left/right wheel calibration.
- K1 tracking start no longer requires a centered line. READY is established by two fresh complete Path Fish frames; K1 still rejects stale/faulted Path Fish data or an unhealthy IMU, and the running controller still stops on invalid or expired sensor data.
- Periodic Path Fish and track telemetry is emitted only while line tracking is active. `FISH_PROBE` is edge-triggered on transport state or bus-line changes so a persistent fault cannot flood RTT or ESP logs.
- OLED is a fixed operational page: tracking active, right-angle sequence active, and completed laps. The non-blocking buzzer emits one pulse on tracking start, one pulse per completed lap, and three pulses when an active tracking run ends.
- Every debounced key event requests audible feedback, including rejected K1/K2 actions. Sensor faults temporarily replace the first two OLED rows with `SENSOR:ERROR` and a short Fish/IMU reason; laps and the 12-channel pattern remain visible.
- The 2026-07-25 full-lap trace measured about 1407-1422 encoder counts over the 700 ms fallback with accurate 88-89 degree gyro pivots and 20-25 ms capture handoff. The requested fallback is now 750 ms, the pre-pivot stop is 60 ms, and capture accepts any contiguous normal line up to error 3.5 after three frames before handing control back to the full line PID. Tracking defaults are base 26, straight 30, curve 24, and capture 18.
- OLED line 4 displays the complete `A1..A12` Path Fish digital pattern as `A:xxxxxxxxxxxx`.

## Board 1.0 I/O Map

- The 2026-07-18 board netlist keeps both motors, both quadrature encoders, the eight-channel grayscale multiplexer, IMU660RB SPI1, ESP-01S UART1, debug UART0, and four active-low keys on their existing pins.
- PB4 is the only MCU-controlled status LED and is active high. PB5 is no longer LED2; it drives the buzzer transistor and is active high. K1/K2/K3/K4 remain PB15/PB16/PB2/PA30 with pull-ups.
- Key feedback is non-blocking: ordinary debounced key events request one 60 ms beep; K3/K4 calibration actions and K3 long motor-scale calibration request two beeps separated by 60 ms.
- OLED support is an unbound SSD1306-compatible 128x64 transport abstraction. It must not use the legacy PB8/PB9 software-I2C binding because those pins are IMU660RB MOSI/SCLK. Do not call `App_Oled_Init()` until a new bus is attached.
- See `docs/board-1.0-pinmap.md` for the full functional map, netlist caveats, and the unresolved active-versus-passive buzzer check.
