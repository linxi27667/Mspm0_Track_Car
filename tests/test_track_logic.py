import subprocess
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def compile_and_run(source: str) -> subprocess.CompletedProcess:
    with tempfile.TemporaryDirectory(prefix="track_logic_test_") as tmpdir:
        tmp = Path(tmpdir) / "_track_logic_test.c"
        exe = Path(tmpdir) / "_track_logic_test.exe"
        tmp.write_text(source, encoding="utf-8")
        compile_result = subprocess.run(
            ["gcc", "-std=c99", "-Wall", "-Wextra", "-I", str(ROOT), str(tmp), "-o", str(exe)],
            check=False,
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            cwd=ROOT,
        )
        if compile_result.returncode != 0:
            return compile_result
        return subprocess.run(
            [str(exe)],
            check=False,
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            cwd=ROOT,
        )


class TrackLogicTests(unittest.TestCase):
    def test_line_error_uses_only_digital_channel_weights(self):
        result = compile_and_run(
            r'''
            #include "TASK/track_logic.h"

            int main(void) {
                float centered = Track_ComputeDigitalLineError8(0x18u, 2.0f);
                if (centered < -0.01f || centered > 0.01f) return 1;
                if (Track_ComputeDigitalLineError8(0x00u, -1.25f) != -1.25f) return 2;
                if (Track_ComputeDigitalLineError8(0x06u, 0.0f) != -2.0f) return 3;
                if (Track_ComputeDigitalLineError8(0x60u, 0.0f) != 2.0f) return 4;
                if (Track_ComputeDigitalLineError8(0x08u, 0.0f) != -0.5f) return 5;
                if (Track_ComputeDigitalLineError8(0x10u, 0.0f) != 0.5f) return 6;
                return 0;
            }
            '''
        )
        self.assertEqual(result.returncode, 0, result.stderr + result.stdout)

    def test_line_classifier_accepts_measured_split_left_corners(self):
        result = compile_and_run(
            r'''
            #include "TASK/track_logic.h"

            int main(void) {
                track_line_observation_t center = Track_ClassifyLine8(0x18u, 3.0f, 4u);
                track_line_observation_t left = Track_ClassifyLine8(0x0Cu, 0.0f, 4u);
                track_line_observation_t right = Track_ClassifyLine8(0x30u, 0.0f, 4u);
                if (center.type != TRACK_LINE_NORMAL || center.error != 0.0f) return 1;
                if (left.type != TRACK_LINE_NORMAL || right.type != TRACK_LINE_NORMAL) return 2;
                if (left.error != -right.error || left.error != -1.0f) return 3;
                if (Track_ClassifyLine8(0x00u, 1.25f, 4u).type != TRACK_LINE_LOST) return 4;
                if (Track_ClassifyLine8(0x81u, 1.25f, 4u).type != TRACK_LINE_INVALID) return 5;
                if (Track_ClassifyLine8(0xFFu, 1.25f, 4u).type != TRACK_LINE_INVALID) return 6;
                if (Track_ClassifyLine8(0x0Eu, 0.0f, 4u).type != TRACK_LINE_NORMAL) return 7;
                if (Track_ClassifyLine8(0x0Fu, 0.0f, 4u).type != TRACK_LINE_LEFT_CORNER) return 8;
                if (Track_ClassifyLine8(0x1Eu, 0.0f, 4u).type != TRACK_LINE_LEFT_CORNER) return 9;
                if (Track_ClassifyLine8(0x09u, 0.0f, 4u).type != TRACK_LINE_LEFT_CORNER) return 10;
                if (Track_ClassifyLine8(0x0Du, 0.0f, 4u).type != TRACK_LINE_LEFT_CORNER) return 11;
                if (Track_ClassifyLine8(0x0Fu, 0.0f, 8u).type != TRACK_LINE_LEFT_CORNER) return 12;
                if (Track_ClassifyLine8(0x1Fu, 0.0f, 8u).type != TRACK_LINE_LEFT_CORNER) return 13;
                return 0;
            }
            '''
        )
        self.assertEqual(result.returncode, 0, result.stderr + result.stdout)

    def test_corner_requires_centered_history_and_capture_requires_frames(self):
        result = compile_and_run(
            r'''
            #include "TASK/track_logic.h"

            int main(void) {
                uint8_t corner = 0u;
                uint8_t capture = 0u;
                track_line_observation_t good = Track_ClassifyLine8(0x18u, 0.0f, 4u);
                track_line_observation_t bad = Track_ClassifyLine8(0x81u, 0.0f, 4u);
                corner = Track_UpdateCornerConfirm(corner, 1u, true);
                if (corner != 0u) return 1;
                corner = Track_UpdateCornerConfirm(corner, 2u, true);
                if (corner != 1u) return 2;
                corner = Track_UpdateCornerConfirm(corner, 2u, false);
                if (corner != 0u) return 3;
                if (Track_CornerArmed(199u, 0u, 0u, 0u, 200u, 300u)) return 6;
                if (!Track_CornerArmed(200u, 0u, 0u, 0u, 200u, 300u)) return 7;
                if (Track_CornerArmed(499u, 0u, 300u, 1u, 200u, 300u)) return 8;
                if (!Track_CornerArmed(600u, 0u, 300u, 1u, 200u, 300u)) return 9;
                if (Track_WhiteCornerFallbackReady(1u, 2u, 100u, 90u, 100u)) return 10;
                if (Track_WhiteCornerFallbackReady(2u, 1u, 100u, 90u, 100u)) return 11;
                if (!Track_WhiteCornerFallbackReady(2u, 2u, 100u, 90u, 100u)) return 12;
                if (Track_WhiteCornerFallbackReady(2u, 2u, 201u, 100u, 100u)) return 13;
                if (!Track_WhiteCornerFallbackReady(2u, 2u, 9u, UINT32_MAX - 10u, 100u)) return 14;
                for (int i = 0; i < 2; ++i) {
                    capture = Track_UpdateConfirmFrames(
                        capture, Track_CaptureLineValid(good, 1.0f));
                }
                if (capture != 2u) return 4;
                capture = Track_UpdateConfirmFrames(
                    capture, Track_CaptureLineValid(bad, 1.0f));
                if (capture != 0u) return 5;
                return 0;
            }
            '''
        )
        self.assertEqual(result.returncode, 0, result.stderr + result.stdout)

    def test_encoder_distance_is_independent_and_wrap_safe(self):
        result = compile_and_run(
            r'''
            #include <stdint.h>
            #include "TASK/track_logic.h"

            int main(void) {
                int32_t start = INT32_MAX - 10;
                int32_t current = (int32_t)((uint32_t)start + 40u);
                if (Track_EncoderDelta(current, start) != 40) return 1;
                if (!Track_EncoderDistanceReached(200, 100, 0, 0,
                                                   2.0f, 1.0f, 100.0f)) return 2;
                if (Track_EncoderDistanceReached(200, 50, 0, 0,
                                                  2.0f, 1.0f, 100.0f)) return 3;
                if (Track_EncoderDistanceReached(200, 100, 0, 0,
                                                  0.0f, 1.0f, 10.0f)) return 4;
                return 0;
            }
            '''
        )
        self.assertEqual(result.returncode, 0, result.stderr + result.stdout)

    def test_all_eight_calibration_channels_must_have_contrast(self):
        result = compile_and_run(
            r'''
            #include "BSP/Inc/bsp_grayscale.h"

            int main(void) {
                uint16_t white[8] = {1200,1200,1200,1200,1200,1200,1200,1200};
                uint16_t black[8] = {400,400,400,400,400,400,400,400};
                if (!Bsp_Grayscale_ContrastValid8(white, black, 200u)) return 1;
                black[6] = 1050u;
                if (Bsp_Grayscale_ContrastValid8(white, black, 200u)) return 2;
                black[6] = 1400u;
                if (!Bsp_Grayscale_ContrastValid8(white, black, 200u)) return 3;
                return 0;
            }
            '''
        )
        self.assertEqual(result.returncode, 0, result.stderr + result.stdout)

    def test_periodic_logs_are_gated_by_motion_state(self):
        source = (ROOT / "main.c").read_text(encoding="utf-8")
        vehicle_log = source.split("static void Main_LogVehicleStatus", 1)[1].split(
            "static void Main_LogGrayscale", 1
        )[0]
        grayscale_log = source.split("static void Main_LogGrayscale", 1)[1].split(
            "static void Main_HandleKeyPush", 1
        )[0]

        self.assertIn("if (!Track_State_Allows_Motor(Track.state)) return;", vehicle_log)
        self.assertIn("if (!Track_State_Is_LineTracking(Track.state)) return;", grayscale_log)

    def test_safety_health_ignores_heartbeat_and_keeps_sensor_faults(self):
        result = compile_and_run(
            r'''
            #include <stdint.h>
            #include "APP/Inc/safety_logic.h"

            int main(void) {
                safety_health_t h = {0};
                if (Safety_Logic_Check(200u, &h, 50u) != SAFETY_LOGIC_OK) return 1;
                h.grayscale_required = true;
                h.grayscale_seen = true;
                h.last_grayscale_tick = UINT32_MAX - 10u;
                if (Safety_Logic_Check(9u, &h, 50u) != SAFETY_LOGIC_OK) return 7;
                if (Safety_Logic_Check(50u, &h, 50u) != SAFETY_LOGIC_GRAYSCALE_EXPIRED) return 8;
                h.grayscale_required = false;
                h.sensor_abnormal = true;
                if (Safety_Logic_Check(50u, &h, 50u) != SAFETY_LOGIC_SENSOR_ABNORMAL) return 2;
                if (!Safety_Logic_CanEnterRotation(0u) ||
                    !Safety_Logic_CanEnterRotation(2u)) return 5;
                if (Safety_Logic_CanEnterRotation(1u) ||
                    Safety_Logic_CanEnterRotation(3u) ||
                    Safety_Logic_CanEnterRotation(4u)) return 6;
                if (Safety_Logic_FinishedHoldElapsed(1999u, 1000u, 1000u)) return 9;
                if (!Safety_Logic_FinishedHoldElapsed(2000u, 1000u, 1000u)) return 10;
                if (!Safety_Logic_FinishedHoldElapsed(3u, UINT32_MAX - 996u, 1000u)) return 11;
                return 0;
            }
            '''
        )
        self.assertEqual(result.returncode, 0, result.stderr + result.stdout)

    def test_encoder_zero_speed_is_allowed_and_impossible_delta_is_fault(self):
        result = compile_and_run(
            r'''
            #include "APP/Inc/encoder_health_logic.h"

            int main(void) {
                for (int i = 0; i < 1000; ++i) {
                    if (!Encoder_Health_DeltaValid(0, 20000)) return 1;
                }
                if (!Encoder_Health_DeltaValid(3, 20000)) return 2;
                if (!Encoder_Health_DeltaValid(-20000, 20000)) return 3;
                if (Encoder_Health_DeltaValid(20001, 20000)) return 4;
                if (Encoder_Health_DeltaValid(-20001, 20000)) return 5;
                return 0;
            }
            '''
        )
        self.assertEqual(result.returncode, 0, result.stderr + result.stdout)

    def test_gpio_quadrature_decoder_counts_all_four_edges(self):
        result = compile_and_run(
            r'''
            #include "APP/Inc/encoder_quadrature_logic.h"

            int main(void) {
                const unsigned char forward[] = {0u, 2u, 3u, 1u, 0u};
                const unsigned char reverse[] = {0u, 1u, 3u, 2u, 0u};
                int forward_sum = 0;
                int reverse_sum = 0;
                for (int i = 1; i < 5; ++i) {
                    forward_sum += Encoder_Quadrature_Delta(forward[i - 1], forward[i]);
                    reverse_sum += Encoder_Quadrature_Delta(reverse[i - 1], reverse[i]);
                }
                if (forward_sum != 4) return 1;
                if (reverse_sum != -4) return 2;
                if (Encoder_Quadrature_Delta(0u, 3u) != 0) return 3;
                if (Encoder_Quadrature_Delta(2u, 2u) != 0) return 4;
                return 0;
            }
            '''
        )
        self.assertEqual(result.returncode, 0, result.stderr + result.stdout)

    def test_sysconfig_uses_symmetric_gpio_quadrature_inputs(self):
        syscfg = (ROOT / "TI_moban.syscfg").read_text(encoding="utf-8")
        generated = (ROOT / "ti_msp_dl_config.c").read_text(encoding="utf-8")
        motor_source = (ROOT / "APP" / "Src" / "app_encoder_motor.c").read_text(
            encoding="utf-8"
        )

        self.assertNotIn('addModule("/ti/driverlib/QEI"', syscfg)
        for group in ("GPIO9", "GPIO11"):
            for channel in (0, 1):
                prefix = f"{group}.associatedPins[{channel}]"
                self.assertIn(f"{prefix}.interruptEn", syscfg)
                self.assertIn(f'{prefix}.polarity         = "RISE_FALL"', syscfg)
        self.assertIn("DL_GPIO_setUpperPinsPolarity", generated)
        self.assertIn("LEFT_ENC_PHA_PIN | LEFT_ENC_PHB_PIN", motor_source)
        self.assertIn("RIGHT_ENC_PHA_PIN | RIGHT_ENC_PHB_PIN", motor_source)

    def test_motor_bench_test_is_nonblocking_bounded_and_stoppable(self):
        track_source = (ROOT / "TASK" / "track.c").read_text(encoding="utf-8")
        tuning_source = (ROOT / "APP" / "Src" / "app_tuning.c").read_text(
            encoding="utf-8"
        )
        start = track_source.split("bool Track_Start_Motor_Test", 1)[1].split(
            "bool Track_Is_Motor_Test", 1
        )[0]
        handler = track_source.split("static void Track_Handle_Motor_Test", 1)[1].split(
            "static void Track_Handle_Heading_Hold", 1
        )[0]

        self.assertIn("Safety_TransitionToBenchTest", start)
        self.assertIn("App_Motor_Enable_Speed_PID(false);", start)
        self.assertIn("TRACK_MOTOR_TEST_TIMEOUT_MS", handler)
        self.assertIn("encoder_sign_or_stall", handler)
        self.assertIn("App_Motor_Enable_Speed_PID(true);", handler)
        self.assertIn("TRACK_MOTOR_TEST_OPEN_PHASES", handler)
        self.assertIn("TRACK_MOTOR_TEST_STOP_PHASE", handler)
        self.assertIn("-TRACK_MOTOR_TEST_TARGET_LOW", handler)
        self.assertIn("TRACK_MOTOR_TEST_WHEEL_MISMATCH", track_source)
        self.assertIn("TRACK_MOTOR_TEST_STOP_MAX_AVG", track_source)
        self.assertIn('"qualification_failed"', handler)
        self.assertNotIn("Bsp_Delay_ms", handler)
        self.assertIn('strcmp(cmd, "motor_test") == 0', tuning_source)
        self.assertIn('find_bool(json, "lifted", &lifted)', tuning_source)
        self.assertIn('Track_Motor_Test_End("stopped")', track_source)

    def test_motion_rejections_are_specific_and_faults_auto_recover(self):
        tuning_source = (ROOT / "APP" / "Src" / "app_tuning.c").read_text(
            encoding="utf-8"
        )
        safety_source = (ROOT / "APP" / "Src" / "app_safety.c").read_text(
            encoding="utf-8"
        )
        main_source = (ROOT / "main.c").read_text(encoding="utf-8")
        motor_start = (ROOT / "TASK" / "track.c").read_text(
            encoding="utf-8"
        ).split("bool Track_Start_Motor_Test", 1)[1].split(
            "bool Track_Is_Motor_Test", 1
        )[0]

        self.assertNotIn("safety_not_ready", tuning_source)
        self.assertIn('"grayscale_not_ready"', tuning_source)
        self.assertNotIn('"link_not_ready"', tuning_source)
        self.assertNotIn("Safety_ArmRemoteMotion", tuning_source)
        self.assertNotIn("Safety_DisarmRemoteMotion", tuning_source)
        self.assertIn('"motion_busy"', tuning_source)
        self.assertIn("SAFETY_STATE_BOOT", motor_start)
        self.assertIn('elog_w("MOTION", "motion stopped: %s (auto recovery)"',
                      safety_source)
        self.assertIn("Safety_ClearFault();", main_source)

        common_header = (ROOT / "APP" / "Inc" / "app_common.h").read_text(
            encoding="utf-8"
        )
        safety_header = (ROOT / "APP" / "Inc" / "safety_logic.h").read_text(
            encoding="utf-8"
        )
        host_config = (ROOT / "host" / "config.py").read_text(encoding="utf-8")
        self.assertNotIn("HEARTBEAT_TIMEOUT", common_header)
        self.assertNotIn("HEARTBEAT", safety_header)
        self.assertIn("HEARTBEAT_INTERVAL_MS = 250", host_config)

    def test_right_angle_heading_lock_requires_angle_and_rate(self):
        result = compile_and_run(
            r'''
            #include <stdbool.h>
            #include "TASK/track_logic.h"

            int main(void) {
                if (!Track_RightAngle_HeadingSettled(3.0f, 12.0f)) return 1;
                if (Track_RightAngle_HeadingSettled(3.1f, 0.0f)) return 2;
                if (Track_RightAngle_HeadingSettled(0.0f, 12.1f)) return 3;
                return 0;
            }
            '''
        )
        self.assertEqual(result.returncode, 0, result.stderr + result.stdout)

    def test_square_corner_accepts_only_counterclockwise_left_chain(self):
        result = compile_and_run(
            r'''
            #include <stdbool.h>
            #include "TASK/track_logic.h"

            int main(void) {
                if (Track_SquareLeftCorner(0x03u, 3u)) return 1;
                if (Track_SquareLeftCorner(0x18u, 3u)) return 2;
                /* 0x0E=S1/S2/S3: captured real-car straight-line drift. */
                if (Track_SquareLeftCorner(0x0Eu, 3u)) return 3;
                if (Track_SquareLeftCorner(0x70u, 3u)) return 4;
                if (Track_SquareLeftCorner(0xFFu, 3u)) return 5;
                if (!Track_SquareLeftCorner(0x1Eu, 4u)) return 6;
                if (!Track_SquareLeftCorner(0x1Fu, 4u)) return 7;
                if (Track_SquareLeftCorner(0x1Eu, 5u)) return 8;
                /* Real-car L-corner masks: split branch + incoming line. */
                if (!Track_SquareLeftCorner(0x09u, 4u)) return 9;
                if (!Track_SquareLeftCorner(0x0Du, 4u)) return 10;
                if (!Track_SquareLeftCorner(0x0Fu, 4u)) return 11;
                /* S0 without the incoming-line anchor, and 0x0E without S0,
                 * must remain rejected. */
                if (Track_SquareLeftCorner(0x01u, 4u)) return 12;
                if (Track_SquareLeftCorner(0x0Eu, 4u)) return 13;
                return 0;
            }
            '''
        )
        self.assertEqual(result.returncode, 0, result.stderr + result.stdout)

    def test_corner_phases_and_lap_count(self):
        result = compile_and_run(
            r'''
            #include "TASK/track_logic.h"

            int main(void) {
                uint8_t frames = 0;
                frames = Track_UpdateConfirmFrames(frames, true);
                frames = Track_UpdateConfirmFrames(frames, true);
                if (frames != 2u) return 1;
                frames = Track_UpdateConfirmFrames(frames, false);
                if (frames != 0u) return 2;
                for (uint8_t laps = 1u; laps <= 5u; laps++) {
                    uint16_t corners = (uint16_t)laps * 4u;
                    if (Track_CornersComplete((uint16_t)(corners - 1u), laps)) return 6;
                    if (!Track_CornersComplete(corners, laps)) return 7;
                    if (Track_CirclesFromCorners(corners, laps) != laps) return 8;
                }
                return 0;
            }
            '''
        )
        self.assertEqual(result.returncode, 0, result.stderr + result.stdout)

    def test_square_runtime_does_not_dispatch_legacy_track_states(self):
        source = (ROOT / "TASK" / "track.c").read_text(encoding="utf-8")
        update_switch = source.split("switch (Track.state)", 1)[1].split(
            "if (!Track_State_Allows_Motor", 1
        )[0]
        self.assertNotIn("case TRACK_STATE_START_LINE", update_switch)
        self.assertNotIn("case TRACK_STATE_CROSS", update_switch)
        timeout = source.split('APP_TRACK_LOG_W("TRACK", "CORNER_FAIL reason=timeout', 1)[0].rsplit(
            "if (Track_Timeout_Reached(now, Track.corner_started_at", 1
        )[1]
        self.assertNotIn("right_angle_count++", timeout)
        self.assertIn("Track.state = TRACK_STATE_FINISHED;", timeout)
        self.assertIn("Track_Motor_Stop_Immediate();", timeout)
        self.assertIn("Track.corner_phase == TRACK_CORNER_PIVOT", source)
        self.assertIn('CORNER_RECOVER pivot_timeout -> capture', source)
        self.assertIn("Track_EncoderDistanceReached", source)
        self.assertNotIn("CORNER_FAIL reason=approach_cal_missing", source)
        self.assertIn("Track.corner_phase = TRACK_CORNER_APPROACH;", source)
        self.assertIn("TRACK_RIGHT_ANGLE_APPROACH_FALLBACK_MS", source)
        self.assertIn('Track_BeginRightAngle(now, digital_mask, "mask");', source)
        self.assertIn('Track_BeginRightAngle(now, digital_mask, "white");', source)
        self.assertIn("Track.line_mask_history = (Track.line_mask_history << 8)", source)
        self.assertIn("Track_Apply_Rotate_Envelope(angle_error, Imu_Sensor.gyro_z);", source)
        self.assertNotIn("Track_RightAngle_PivotPwm", source)
        self.assertIn("if (!pivot_turn) {", source)
        self.assertNotIn("USER_RIGHT_ANGLE_APPROACH_MS", source)
        self.assertIn("App_Motor_Enable_Speed_PID(false);", source)
        self.assertIn("TRACK_MOTOR_INNER_MIN", source)
        completion = source.split("void Track_Check_RightAngle_Completion(void)", 1)[1].split(
            "static void Track_Motor_Output", 1
        )[0]
        self.assertNotIn("Track.state = TRACK_STATE_FINISHED", completion)
        self.assertNotIn("Track_Motor_Stop_Immediate", completion)
        self.assertNotIn("Safety_TransitionToFinished", completion)
        self.assertIn("LAP_TARGET_REACHED", completion)

    def test_line_lost_does_not_stop_and_manual_stop_returns_ready(self):
        track_source = (ROOT / "TASK" / "track.c").read_text(encoding="utf-8")
        detect = track_source.split("static void Track_Detect_Pattern", 1)[1].split(
            "static void Track_Motor_Stop_Immediate", 1
        )[0]
        stop = track_source.split("void Track_Stop(void)", 1)[1].split(
            "bool Track_Start_Rotation", 1
        )[0]
        self.assertNotIn("Track.state = TRACK_STATE_LINE_LOST", detect)
        self.assertNotIn("LINE_FAIL reason=lost", detect)
        self.assertNotIn("LINE_FAIL reason=invalid", detect)
        self.assertNotIn("Track_Handle_LineLost", track_source)
        self.assertLess(
            stop.index("Safety_TransitionToFinished();"),
            stop.index("Safety_TransitionToReady();"),
        )
        self.assertIn("Safety_ClearFault();", stop)

        tuning_source = (ROOT / "APP" / "Src" / "app_tuning.c").read_text(
            encoding="utf-8"
        )
        start = tuning_source.split('strcmp(cmd, "start") == 0', 1)[1].split(
            'strcmp(cmd, "stop") == 0', 1
        )[0]
        self.assertNotIn("SAFETY_STATE_FINISHED", start)

    def test_start_does_not_reject_current_line_pattern_and_result_is_acked(self):
        track_source = (ROOT / "TASK" / "track.c").read_text(encoding="utf-8")
        start = track_source.split("bool Track_Start(void)", 1)[1].split(
            "void Track_Stop(void)", 1
        )[0]
        self.assertNotIn("Track_StartPatternValid", start)
        self.assertNotIn("invalid_start_pattern", start)
        self.assertLess(
            start.index("Track_Imu_Healthy"),
            start.index("Safety_TransitionToRunning"),
        )
        self.assertLess(
            start.index("Grayscale_Calibration_Valid"),
            start.index("Safety_TransitionToRunning"),
        )

        tuning_source = (ROOT / "APP" / "Src" / "app_tuning.c").read_text(
            encoding="utf-8"
        )
        self.assertIn("*error = Track_Get_Start_Result();", tuning_source)

    def test_timing_state_and_direction_helpers(self):
        result = compile_and_run(
            r'''
            #include <stdint.h>
            #include "TASK/track_logic.h"

            int main(void) {
                if (Track_State_Allows_Motor(TRACK_STATE_IDLE)) return 1;
                if (!Track_State_Allows_Motor(TRACK_STATE_LINE_FOLLOW)) return 2;
                if (Track_State_Allows_Motor(TRACK_STATE_FINISHED)) return 3;
                if (Track_State_Is_LineTracking(TRACK_STATE_IDLE)) return 4;
                if (!Track_State_Is_LineTracking(TRACK_STATE_LINE_FOLLOW)) return 5;
                if (!Track_State_Is_LineTracking(TRACK_STATE_RIGHT_ANGLE)) return 6;
                if (!Track_State_Is_LineTracking(TRACK_STATE_LINE_LOST)) return 7;
                if (!Track_State_Is_LineTracking(TRACK_STATE_START_LINE)) return 8;
                if (!Track_State_Is_LineTracking(TRACK_STATE_CROSS)) return 9;
                if (Track_State_Is_LineTracking(TRACK_STATE_ROTATE)) return 10;
                if (Track_State_Is_LineTracking(TRACK_STATE_HEADING_HOLD)) return 11;
                if (Track_State_Is_LineTracking(TRACK_STATE_FINISHED)) return 12;
                if (Track_Timeout_Reached(1049u, 1000u, 50u)) return 13;
                if (!Track_Timeout_Reached(1050u, 1000u, 50u)) return 14;
                if (!Track_Timeout_Reached(3u, UINT32_MAX - 5u, 8u)) return 15;
                if (Track_LineTurn_WithGyroDamping(-12.0f, 20.0f, 0.15f) != -9.0f) return 19;
                if (Track_LineTurn_WithGyroDamping(12.0f, -20.0f, 0.15f) != 9.0f) return 20;
                return 0;
            }
            '''
        )
        self.assertEqual(result.returncode, 0, result.stderr + result.stdout)

    def test_rotate_logic_validates_direction_and_completion(self):
        result = compile_and_run(
            r'''
            #include "TASK/track_logic.h"

            int main(void) {
                int32_t left = 0, right = 0;
                if (!Track_Rotate_TargetValid(90.0f)) return 1;
                if (Track_Rotate_TargetValid(0.0f)) return 2;
                if (Track_Rotate_TargetValid(180.1f)) return 3;
                if (Track_Rotate_Error(100.0f, 130.0f, 90.0f) != 60.0f) return 4;
                if (!Track_Rotate_Complete(3.0f, 6.0f)) return 5;
                if (Track_Rotate_Complete(3.1f, 0.0f)) return 6;
                if (!Track_Rotate_AngleInside(-3.0f)) return 9;
                if (Track_Rotate_AngleInside(3.1f)) return 10;
                if (Track_Rotate_SpeedLimit(20.0f, 20.0f, 40.0f, 5.0f, 16.0f) != 5.0f) return 11;
                if (Track_Rotate_SpeedLimit(40.0f, 20.0f, 40.0f, 5.0f, 16.0f) != 16.0f) return 12;
                if (Track_Rotate_SpeedLimit(30.0f, 20.0f, 40.0f, 5.0f, 16.0f) != 10.5f) return 13;
                if (!Track_Rotate_ShouldCoast(5.0f, 8.0f, 3.5f, 4.0f)) return 14;
                if (Track_Rotate_ShouldCoast(5.0f, -1.0f, 3.5f, 4.0f)) return 15;
                if (Track_Rotate_ShouldCoast(5.0f, 0.0f, 3.5f, 4.0f)) return 16;
                if (Track_Rotate_ShouldCoast(5.0f, 8.0f, 4.0f, 4.0f)) return 17;
                if (Track_CaptureBlendTurn(0.0f, -1.5f, true, 3.5f, 8.0f) != -5.25f) return 18;
                if (Track_CaptureBlendTurn(2.0f, -3.5f, true, 3.5f, 8.0f) != -8.0f) return 19;
                if (Track_CaptureBlendTurn(2.0f, -3.5f, false, 3.5f, 8.0f) != 2.0f) return 20;
                if (Track_RightAngle_HybridHandoff(69.0f, 0.0f, 2u, 70.0f, 82.0f, 25.0f, 2u)) return 21;
                if (Track_RightAngle_HybridHandoff(75.0f, 0.0f, 1u, 70.0f, 82.0f, 25.0f, 2u)) return 22;
                if (!Track_RightAngle_HybridHandoff(75.0f, 20.0f, 2u, 70.0f, 82.0f, 25.0f, 2u)) return 23;
                if (!Track_RightAngle_HybridHandoff(82.0f, 20.0f, 0u, 70.0f, 82.0f, 25.0f, 2u)) return 24;
                if (Track_RightAngle_HybridHandoff(90.0f, 30.0f, 2u, 70.0f, 82.0f, 25.0f, 2u)) return 25;
                Track_Rotate_Command(90.0f, 30, &left, &right);
                if (left >= 0 || right <= 0) return 7;
                Track_Rotate_Command(-90.0f, 30, &left, &right);
                return (left > 0 && right < 0) ? 0 : 8;
            }
            '''
        )
        self.assertEqual(result.returncode, 0, result.stderr + result.stdout)

    def test_rotate_slew_crosses_zero_before_reversing(self):
        result = compile_and_run(
            r'''
            #include "TASK/track_logic.h"

            int main(void) {
                int32_t value = -2;
                value = Track_Slew_Command(value, 8, 1);
                if (value != -1) return 1;
                value = Track_Slew_Command(value, 8, 1);
                if (value != 0) return 2;
                value = Track_Slew_Command(value, 8, 1);
                if (value != 1) return 3;
                if (Track_Slew_Command(7, 8, 2) != 8) return 4;
                return Track_Slew_Command(5, 0, 1) == 0 ? 0 : 5;
            }
            '''
        )
        self.assertEqual(result.returncode, 0, result.stderr + result.stdout)

    def test_manual_rotation_uses_fast_near_target_speed_envelope(self):
        tuning = (ROOT / "APP" / "Inc" / "user_tuning.h").read_text(
            encoding="utf-8"
        )
        self.assertIn("USER_ROTATE_PID_MAX_SPEED       16.0f", tuning)
        self.assertIn("USER_ROTATE_PID_MIN_SPEED       8", tuning)
        self.assertIn("USER_ROTATE_PID_FINE_ZONE_DEG    10.0f", tuning)
        self.assertIn("USER_ROTATE_PID_BLEND_ZONE_DEG   25.0f", tuning)
        self.assertIn("USER_ROTATE_PID_FINE_MAX_SPEED    7.0f", tuning)
        self.assertIn("USER_ROTATE_PID_COAST_OUTPUT    4.0f", tuning)
        self.assertIn("USER_RIGHT_ANGLE_APPROACH_SPEED  28", tuning)
        self.assertIn("USER_RIGHT_ANGLE_CONFIRM_FRAMES 1", tuning)
        self.assertIn("USER_RIGHT_ANGLE_WHITE_FALLBACK_FRAMES 2u", tuning)
        self.assertIn("USER_RIGHT_ANGLE_WHITE_RECENT_MS 100u", tuning)
        self.assertIn("USER_RIGHT_ANGLE_HYBRID_MIN_TURN_DEG 70.0f", tuning)
        self.assertIn("USER_RIGHT_ANGLE_HYBRID_HANDOFF_DEG 82.0f", tuning)
        self.assertIn("USER_RIGHT_ANGLE_HYBRID_LINE_FRAMES 2u", tuning)
        self.assertIn("USER_RIGHT_ANGLE_CAPTURE_SPEED    14", tuning)
        self.assertIn("USER_RIGHT_ANGLE_APPROACH_FALLBACK_MS 1300u", tuning)
        self.assertIn("USER_RIGHT_ANGLE_TIMEOUT   7000", tuning)
        self.assertNotIn("USER_RIGHT_ANGLE_PIVOT_MAX_PWM", tuning)

    def test_rotation_requires_twenty_continuous_settle_frames(self):
        result = compile_and_run(
            r'''
            #include "TASK/track_logic.h"

            int main(void) {
                uint8_t frames = 0;
                for (int i = 0; i < 19; ++i) {
                    frames = Track_Rotate_UpdateSettleFrames(frames, true);
                }
                if (Track_Rotate_Settled(frames, 20u)) return 1;
                frames = Track_Rotate_UpdateSettleFrames(frames, false);
                if (frames != 0u) return 2;
                for (int i = 0; i < 20; ++i) {
                    frames = Track_Rotate_UpdateSettleFrames(frames, true);
                }
                return Track_Rotate_Settled(frames, 20u) ? 0 : 3;
            }
            '''
        )
        self.assertEqual(result.returncode, 0, result.stderr + result.stdout)

    def test_speed_pwm_never_reverses_requested_direction(self):
        result = compile_and_run(
            r'''
            #include "APP/Inc/motor_speed_logic.h"

            int main(void) {
                if (Motor_Speed_ConstrainPwm(-12, 28, 24) != 0) return 1;
                if (Motor_Speed_ConstrainPwm(12, -28, 24) != 0) return 2;
                if (Motor_Speed_ConstrainPwm(-12, -8, 24) != -24) return 3;
                if (Motor_Speed_ConstrainPwm(12, 8, 24) != 24) return 4;
                if (Motor_Speed_ConstrainPwm(0, 90, 24) != 0) return 5;
                return 0;
            }
            '''
        )
        self.assertEqual(result.returncode, 0, result.stderr + result.stdout)

    def test_pwm_duty_maps_to_down_count_compare(self):
        result = compile_and_run(
            r'''
            #include "APP/Inc/motor_speed_logic.h"

            int main(void) {
                if (Motor_Pwm_DutyToCompare(0u, 100u) != 100u) return 1;
                if (Motor_Pwm_DutyToCompare(20u, 100u) != 80u) return 2;
                if (Motor_Pwm_DutyToCompare(100u, 100u) != 0u) return 3;
                if (Motor_Pwm_DutyToCompare(120u, 100u) != 0u) return 4;
                return 0;
            }
            '''
        )
        self.assertEqual(result.returncode, 0, result.stderr + result.stdout)

    def test_esp_transparent_tx_waits_for_whole_frame_capacity(self):
        source = (ROOT / "APP" / "Src" / "app_esp01s.c").read_text(
            encoding="utf-8"
        )
        transparent = source.split("if (s_transparent_mode)", 1)[1].split(
            "switch (s_tx_state)", 1
        )[0]
        self.assertIn("Bsp_UART_TrySendFrame(&ESP_Uart_Dev", transparent)
        self.assertIn("ESP01S_TX_CHUNK_BYTES", transparent)
        self.assertIn("ESP01S_TX_CHUNK_GAP_MS", transparent)
        self.assertIn("s_tx_active_offset", transparent)
        uart_source = (ROOT / "BSP" / "Src" / "bsp_uart.c").read_text(
            encoding="utf-8"
        )
        frame_send = uart_source.split("bool Bsp_UART_TrySendFrame", 1)[1].split(
            "void Bsp_UART_Send_Block", 1
        )[0]
        self.assertIn("uart->tx_busy || !tx_fifo_empty(uart)", frame_send)
        self.assertIn("uart->tx_fifo_head = 0u", frame_send)
        self.assertIn("memcpy(uart->tx_fifo_buf, data, len)", frame_send)

    def test_tracking_speed_pi_is_default_and_encoder_monitoring_stays_active(self):
        track_source = (ROOT / "TASK" / "track.c").read_text(encoding="utf-8")
        start = track_source.split("bool Track_Start(void)", 1)[1].split(
            "void Track_Stop(void)", 1
        )[0]
        output = track_source.split("static void Track_Motor_Output", 1)[1].split(
            "void Track_Init", 1
        )[0]
        self.assertIn("App_Motor_Enable_Speed_PID(Track.use_speed_pid);", start)
        self.assertIn("USER_TRACK_SPEED_PID_ENABLE 1", (ROOT / "APP/Inc/user_tuning.h").read_text(encoding="utf-8"))
        self.assertIn("App_Motor_Enable_Speed_PID(true);", track_source)
        self.assertIn("Motor_Set_Speed(Track.left_speed);", output)
        self.assertIn("Motor_Right_Set_Speed(Track.right_speed);", output)
        self.assertIn("!pivot_turn && !capture_turn", output)

        motor_source = (ROOT / "APP" / "Src" / "app_encoder_motor.c").read_text(
            encoding="utf-8"
        )
        update = motor_source.split("static void Speed_PID_Update_Single", 1)[1].split(
            "void App_Motor_Speed_PID_Update", 1
        )[0]
        self.assertLess(
            update.index("Bsp_Motor_Update_Status(motor);"),
            update.index("if (!motor->use_speed_pid) return;"),
        )

    def test_white_frames_keep_following_without_counting_a_corner(self):
        source = (ROOT / "TASK" / "track.c").read_text(encoding="utf-8")
        detect = source.split("static void Track_Detect_Pattern", 1)[1].split(
            "static void Track_Motor_Stop_Immediate", 1
        )[0]
        self.assertIn("TRACK_RIGHT_ANGLE_WHITE_FALLBACK_FRAMES", detect)
        self.assertNotIn("TRACK_LINE_LOST_CONFIRM_FRAMES", detect)
        self.assertNotIn("Track.state = TRACK_STATE_LINE_LOST;", detect)
        self.assertNotIn("Track.state = TRACK_STATE_FINISHED;", detect)
        self.assertNotIn("Track_Handle_LineLost", source)

    def test_rotate_timeout_is_wrap_safe(self):
        result = compile_and_run(
            r'''
            #include <stdint.h>
            #include "TASK/track_logic.h"

            int main(void) {
                return Track_Rotate_Timeout(20u, UINT32_MAX - 9u, 30u) ? 0 : 1;
            }
            '''
        )
        self.assertEqual(result.returncode, 0, result.stderr + result.stdout)

    def test_imu_health_gate_rejects_uncalibrated_or_invalid_data(self):
        result = compile_and_run(
            r'''
            #include <stdbool.h>
            #include "TASK/imu_health_logic.h"

            int main(void) {
                if (Imu_Health_AllowsMotion(false, true, IMU_CAL_DONE)) return 1;
                if (Imu_Health_AllowsMotion(true, false, IMU_CAL_DONE)) return 2;
                if (Imu_Health_AllowsMotion(true, true, IMU_CAL_RUNNING)) return 3;
                if (!Imu_Health_AllowsMotion(true, true, IMU_CAL_DONE)) return 4;
                return 0;
            }
            '''
        )
        self.assertEqual(result.returncode, 0, result.stderr + result.stdout)


if __name__ == "__main__":
    unittest.main()
