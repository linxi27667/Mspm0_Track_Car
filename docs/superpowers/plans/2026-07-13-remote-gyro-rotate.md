# Remote Gyro Rotation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement safe remote signed-angle in-place rotation and web controls.

**Architecture:** The host sends a validated `cmd:"rotate"` TC1 request with `deg`; firmware independently validates it and runs a non-blocking `TRACK_STATE_ROTATE`. MPU6050 relative yaw is the angle feedback, with existing encoder/safety monitoring as the motor-output authority.

**Tech Stack:** C99 bare-metal MSPM0, Keil MDK-ARM, TC1 JSON framing, Python Flask-SocketIO, vanilla JS, `unittest`, GCC host logic tests.

## Global Constraints

- Keep the 5 ms control path non-blocking and leave ISR code hardware-only.
- Accept only signed degrees `[-180.0, 180.0]` excluding zero: positive means left/CCW, negative means right/CW.
- Accept rotation only from Safety READY, with tracking idle and MPU6050 healthy and calibrated.
- Stop both motors within the control tick on invalid IMU, safety revocation, encoder fault, stop command, heartbeat loss, or timeout.
- Keep host and firmware field ranges identical; every motion command receives an observable ACK.
- Do not flash before exact J-Link/probe identification and raised-wheel confirmation.

---

### Task 1: Define and test pure rotation decisions

**Files:**
- Modify: `TASK/track_logic.h`
- Modify: `tests/test_track_logic.py`

**Interfaces:** `TRACK_STATE_ROTATE`; `Track_Rotate_TargetValid(float)`; `Track_Rotate_Error(float baseline, float heading, float target)`; `Track_Rotate_Complete(float error, float abs_rate)`; `Track_Rotate_Command(float target, int32_t speed, int32_t *left, int32_t *right)`; `Track_Rotate_Timeout(uint32_t now, uint32_t started, uint32_t duration)`.

- [ ] **Step 1: Add the failing host-GCC test**

```python
def test_rotate_logic_validates_direction_and_completion(self):
    result = compile_and_run(r'''
        #include "TASK/track_logic.h"
        int main(void) {
            int32_t left = 0, right = 0;
            if (!Track_Rotate_TargetValid(90.0f)) return 1;
            if (Track_Rotate_TargetValid(0.0f)) return 2;
            if (Track_Rotate_TargetValid(180.1f)) return 3;
            if (Track_Rotate_Error(100.0f, 130.0f, 90.0f) != 60.0f) return 4;
            if (!Track_Rotate_Complete(1.5f, 7.0f)) return 5;
            Track_Rotate_Command(90.0f, 30, &left, &right);
            if (left >= 0 || right <= 0) return 6;
            Track_Rotate_Command(-90.0f, 30, &left, &right);
            return (left > 0 && right < 0) ? 0 : 7;
        }
    ''')
    self.assertEqual(result.returncode, 0, result.stderr + result.stdout)
```

- [ ] **Step 2: Run the test and verify RED**

Run: `python -m unittest tests.test_track_logic.TrackLogicTests.test_rotate_logic_validates_direction_and_completion -v`

Expected: GCC compile failure because `Track_Rotate_*` does not exist.

- [ ] **Step 3: Implement only deterministic helpers**

Add `TRACK_STATE_ROTATE` before `TRACK_STATE_FINISHED`; make `Track_State_Allows_Motor()` allow it. Add range constants `1.0f`, `180.0f`, `2.0f` completion error, and `8.0f deg/s` settling rate. `Track_Rotate_Error` must calculate `target - (heading - baseline)`. `Track_Rotate_Command` must output `(-speed,+speed)` for positive targets and `(+speed,-speed)` for negative targets. Use the existing unsigned elapsed-time pattern for timeout.

- [ ] **Step 4: Verify GREEN**

Run: `python -m unittest tests.test_track_logic -v`

Expected: all logic tests pass.

### Task 2: Add safety-gated non-blocking firmware rotation

**Files:**
- Modify: `APP/Inc/user_tuning.h`
- Modify: `TASK/track.h`
- Modify: `TASK/track.c`
- Modify: `TASK/track_logic.h`
- Modify: `tests/test_track_logic.py`

**Interfaces:** `bool Track_Start_Rotation(float target_deg)`; `bool Track_Is_Rotating(void)`; `float Track_Get_Rotation_Target(void)`; `float Track_Get_Rotation_Angle(void)`; `float Track_Get_Rotation_Error(void)`; `const char *Track_Get_Rotation_Result(void)`.

- [ ] **Step 1: Add a failing boundary test for wrap-safe timeout**

```python
def test_rotate_timeout_is_wrap_safe(self):
    result = compile_and_run(r'''
        #include <stdint.h>
        #include "TASK/track_logic.h"
        int main(void) {
            return Track_Rotate_Timeout(20u, UINT32_MAX - 9u, 30u) ? 0 : 1;
        }
    ''')
    self.assertEqual(result.returncode, 0, result.stderr + result.stdout)
```

- [ ] **Step 2: Run the new test and verify RED**

Run: `python -m unittest tests.test_track_logic.TrackLogicTests.test_rotate_timeout_is_wrap_safe -v`

Expected: GCC compile failure for the absent timeout helper.

- [ ] **Step 3: Implement the minimal state machine**

Add centralized tuning constants: speed `30`, slow speed `16`, slow zone `20.0f`, base timeout `1000u`, and `30u` ms per requested degree. Add rotation baseline, target, actual angle, error, start time, deadline, settle-frame count, and a fixed-size result string to `track_t`.

`Track_Start_Rotation` must reject invalid target, state other than `TRACK_STATE_IDLE`, safety other than `SAFETY_STATE_READY`, `!Mpu_Sensor.gyro_ok`, or `!Mpu_Sensor.calibrated`. It captures the yaw baseline, resets the gyro PID, calls `Safety_TransitionToRunning()`, verifies `Safety_AllowsMotorOutput()`, and enters `TRACK_STATE_ROTATE`.

In `Track_Update`, update IMU and run rotation even when no grayscale frame is available. For rotate, derive actual angle from `Mpu_Sensor.heading - baseline`; slow down inside the slow zone; command exactly opposite wheel targets using the helper; require three consecutive complete frames before both targets are zero and `Safety_TransitionToReady()` returns the car to repeatable READY. On any invalid IMU, timeout, or safety revocation invoke the existing immediate motor stop path and record a stable result string.

Do not route this state through `Track_Motor_Output`, because it intentionally prevents wheel reversal. Enable speed PID before setting targets and write zero PWM when terminating. Existing `Safety_ControlTick()` remains the final authorization and already turns encoder/heartbeat faults into a motor cut.

- [ ] **Step 4: Verify firmware without flashing**

Run: `python -m unittest discover -s tests -v`

Expected: PASS.

Run: `& 'C:\Keil_v5\UV4\UV4.exe' -b '.\keil\gpio_toggle_output_LP_MSPM0G3507_nortos_keil.uvprojx' -j0 -o rotate-build.log`

Expected: `0 Error(s)` in the new `rotate-build.log`; do not alter `keil/build_log.txt`.

### Task 3: Carry rotation safely across TC1

**Files:**
- Modify: `host/protocol.py`
- Modify: `host/tcp_server.py`
- Modify: `APP/Src/app_tuning.c`
- Modify: `tests/test_host_protocol.py`
- Modify: `tests/test_tcp_server.py`

**Interfaces:** exact request shape `{"v":1,"id":N,"cmd":"rotate","deg":signed_number}`; host `build_rotate(degrees: float, request_id: int | None = None) -> str`; `RequestManager.send_rotate(degrees: float) -> bool`; telemetry rotation group `rt`, `ra`, `re`, `rs`.

- [ ] **Step 1: Add failing host tests**

```python
def test_build_rotate_validates_signed_range(self):
    self.assertEqual(json.loads(build_rotate(90, request_id=19)),
                     {"v": 1, "cmd": "rotate", "deg": 90.0, "id": 19})
    for value in (0, 180.1, -180.1, math.nan, math.inf, "90"):
        with self.subTest(value=value):
            with self.assertRaises((ValueError, ProtocolError)):
                build_rotate(value, request_id=19)
```

- [ ] **Step 2: Run test and verify RED**

Run: `python -m unittest tests.test_host_protocol.RequestTests.test_build_rotate_validates_signed_range -v`

Expected: import failure for absent `build_rotate`.

- [ ] **Step 3: Implement matching host and firmware validation**

In `protocol.py`, accept `deg` only for `cmd == "rotate"`, require a finite nonzero float in the inclusive range, and reject `deg` for all other commands. Implement `build_rotate` without changing legacy `build_cmd` output. In `RequestManager`, add `send_rotate`, retain a display label such as `rotate:-90.0` for retries/status, and resend the original JSON with the original request ID.

In `app_tuning.c`, parse `deg` only for rotate, reject malformed/out-of-range/unready requests with `send_ack(... false, "cmd", Track_Get_Rotation_Result())`, and call `Track_Start_Rotation` only after authentication. Extend state abbreviation with `RT`; include all four rotation telemetry fields as a complete group when active or terminal. If a larger static telemetry buffer is needed, size it from the longest `snprintf` format and preserve zero dynamic allocation.

- [ ] **Step 4: Verify host and Keil builds**

Run: `python -m unittest tests.test_host_protocol tests.test_tcp_server -v`

Expected: PASS.

Run: `& 'C:\Keil_v5\UV4\UV4.exe' -b '.\keil\gpio_toggle_output_LP_MSPM0G3507_nortos_keil.uvprojx' -j0 -o rotate-build.log`

Expected: `0 Error(s)`.

### Task 4: Add web controls with terminal-state visibility

**Files:**
- Modify: `host/app.py`
- Modify: `host/templates/index.html`
- Modify: `host/static/app.js`
- Modify: `host/static/style.css`
- Modify: `tests/test_request_manager.py`

**Interfaces:** Socket.IO `send_rotate` input `{ "deg": number }`; `rotate_status` output `{ "state", "target_deg", "actual_deg", "error_deg", "result" }`.

- [ ] **Step 1: Add a failing request-manager release test**

```python
def test_rotate_ack_releases_pending_request(self):
    self.assertTrue(self.manager.send_rotate(90.0))
    request_id = self.manager.get_pending_id()
    self.manager.on_ack({"v": 1, "t": "ack", "id": request_id,
                         "ok": True, "kind": "cmd"})
    self.assertFalse(self.manager.is_busy())
```

- [ ] **Step 2: Run the regression after Task 3 and verify the request is released**

Run: `python -m unittest tests.test_request_manager -v`

Expected: PASS; the ACK must clear the rotate request so Stop is never blocked by stale state.

- [ ] **Step 3: Implement the rotate panel**

Add `左转 90°`, `右转 90°`, a `-180..180` number input, and `按设定角度旋转` near the existing command stack. In JS validate finite nonzero range, display a direction/magnitude confirmation, disable all rotate start controls after send, leave Stop enabled, and update status with `textContent` only. Re-enable only upon reject, timeout, or terminal telemetry result.

Add `ws_send_rotate` to `host/app.py`, performing the same range/busy/connection checks before `req_mgr.send_rotate`. Emit pending/rejected/timeout status and translate rotation telemetry to `rotate_status`. Scope CSS under `.rotate-*` to avoid changing existing layout.

- [ ] **Step 4: Verify all host code**

Run: `python -m unittest discover -s tests -v`

Expected: PASS.

Run: `python -m compileall -q host tests`

Expected: exit code 0.

### Task 5: Hardware-ready evidence and no-flash gate

**Files:**
- Modify: `docs/superpowers/specs/2026-07-13-remote-gyro-rotate-design.md`
- Modify: `.claude/CLAUDE.md` only if its project summary lacks the final interface.

- [ ] **Step 1: Run full non-hardware verification**

Run: `python -m unittest discover -s tests -v; python -m compileall -q host tests; & 'C:\Keil_v5\UV4\UV4.exe' -b '.\keil\gpio_toggle_output_LP_MSPM0G3507_nortos_keil.uvprojx' -j0 -o rotate-build.log`

Expected: all Python tests pass, compileall exits zero, and Keil reports `0 Error(s)`.

- [ ] **Step 2: Identify J-Link without programming**

Run: `Get-PnpDevice -PresentOnly | Where-Object { $_.InstanceId -match 'VID_1366' } | Format-List Status,Class,FriendlyName,InstanceId`

Expected: an identified SEGGER device or an explicit no-device result that blocks flash.

- [ ] **Step 3: Pause for physical authorization**

Report the build result, candidate probe identity, and request confirmation of exact SWD interface/probe plus raised wheels. Do not execute Keil `-f`, erase, or J-Link programming commands.
