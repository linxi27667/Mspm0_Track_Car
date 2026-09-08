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
            ["gcc", "-std=c99", "-Wall", "-Wextra", "-I", str(ROOT),
             "-I", str(ROOT / "BSP" / "Inc"), str(tmp), "-o", str(exe)],
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
    def test_line_error_uses_all_12_digital_channel_weights(self):
        result = compile_and_run(
            r'''
            #include "TASK/track_logic.h"

            int main(void) {
                float centered = Track_ComputeDigitalLineError12(0x060u, 2.0f);
                if (centered < -0.01f || centered > 0.01f) return 1;
                if (Track_ComputeDigitalLineError12(0x000u, -1.25f) != -1.25f) return 2;
                if (Track_ComputeDigitalLineError12(0x001u, 0.0f) != -3.5f) return 3;
                if (Track_ComputeDigitalLineError12(0x800u, 0.0f) < 3.49f) return 4;
                if (Track_ComputeDigitalLineError12(0x020u, 0.0f) >= 0.0f) return 5;
                if (Track_ComputeDigitalLineError12(0x040u, 0.0f) <= 0.0f) return 6;
                if (Track_LimitRecoveryError(-3.5f, 1.5f) != -1.5f) return 7;
                if (Track_LimitRecoveryError(3.5f, 1.5f) != 1.5f) return 8;
                if (Track_LimitRecoveryError(0.5f, 1.5f) != 0.5f) return 9;
                return 0;
            }
            '''
        )
        self.assertEqual(result.returncode, 0, result.stderr + result.stdout)

    def test_12_channel_classifier_handles_line_corner_white_black_and_noise(self):
        result = compile_and_run(
            r'''
            #include "TASK/track_logic.h"

            int main(void) {
                track_line_observation_t center = Track_ClassifyLine12(0x060u, 3.0f, 6u);
                track_line_observation_t left = Track_ClassifyLine12(0x00Cu, 0.0f, 6u);
                track_line_observation_t right = Track_ClassifyLine12(0x300u, 0.0f, 6u);
                if (center.type != TRACK_LINE_NORMAL ||
                    Track_Abs_Float(center.error) > 0.001f) return 1;
                if (left.type != TRACK_LINE_NORMAL || right.type != TRACK_LINE_NORMAL) return 2;
                if (Track_Abs_Float(left.error + right.error) > 0.001f) return 3;
                if (Track_ClassifyLine12(0x000u, 1.25f, 6u).type != TRACK_LINE_LOST) return 4;
                if (Track_ClassifyLine12(0x801u, 1.25f, 6u).type != TRACK_LINE_INVALID) return 5;
                if (Track_ClassifyLine12(0xFFFu, 1.25f, 6u).type != TRACK_LINE_INVALID) return 6;
                if (Track_ClassifyLine12(0x01Eu, 0.0f, 6u).type != TRACK_LINE_NORMAL) return 7;
                if (Track_ClassifyLine12(0x03Eu, 0.0f, 6u).type != TRACK_LINE_INVALID) return 8;
                if (Track_ClassifyLine12(0x03Fu, 0.0f, 6u).type != TRACK_LINE_LEFT_CORNER) return 9;
                if (Track_ClassifyLine12(0xFC0u, 0.0f, 6u).type != TRACK_LINE_LEFT_CORNER) return 10;
                if (Track_ClassifyLine12(0x3CFu, 0.0f, 10u).type != TRACK_LINE_INVALID) return 11;
                if (Track_ClassifyLine12(0x3FFu, 0.0f, 10u).type != TRACK_LINE_LEFT_CORNER) return 12;
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
                track_line_observation_t edge = Track_ClassifyLine12(0x200u, 0.0f, 6u);
                corner = Track_UpdateCornerConfirm(corner, 1u, true);
                if (corner != 0u) return 1;
                corner = Track_UpdateCornerConfirm(corner, 2u, true);
                if (corner != 1u) return 2;
                corner = Track_UpdateCornerConfirm(corner, 2u, false);
                if (corner != 0u) return 3;
                corner = Track_UpdateCornerGate(0u, true, true, false, true);
                if (corner != 0u) return 4;
                corner = Track_UpdateCornerGate(1u, true, true, true, false);
                if (corner != 0u) return 5;
                corner = Track_UpdateCornerGate(0u, true, true, true, true);
                if (corner != 1u) return 30;
                if (!Track_CornerCandidateReady(true, true, true, corner, 1u)) return 31;
                if (Track_CornerCandidateReady(true, false, true, corner, 1u)) return 32;
                if (Track_CornerArmed(199u, 0u, 0u, 0u, 200u, 300u)) return 6;
                if (!Track_CornerArmed(200u, 0u, 0u, 0u, 200u, 300u)) return 7;
                if (Track_CornerArmed(499u, 0u, 300u, 1u, 200u, 300u)) return 8;
                if (!Track_CornerArmed(600u, 0u, 300u, 1u, 200u, 300u)) return 9;
                if (Track_WhiteCornerFallbackReady(1u, 2u, 2u, 100u, 90u, 100u)) return 10;
                if (Track_WhiteCornerFallbackReady(2u, 1u, 2u, 100u, 90u, 100u)) return 11;
                if (!Track_WhiteCornerFallbackReady(2u, 2u, 2u, 100u, 90u, 100u)) return 12;
                if (Track_WhiteCornerFallbackReady(2u, 2u, 2u, 201u, 100u, 100u)) return 13;
                if (!Track_WhiteCornerFallbackReady(2u, 2u, 2u, 9u, UINT32_MAX - 10u, 100u)) return 14;
                if (Track_CornerTurnSign(TRACK_LINE_LEFT_CORNER) != 1) return 15;
                if (Track_CornerTurnSign(TRACK_LINE_RIGHT_CORNER) != -1) return 16;
                if (Track_CornerTurnSign(TRACK_LINE_NORMAL) != 0) return 17;
                if (Track_CornerDirectionFromMask(0x1Fu) != 1) return 18;
                if (Track_CornerDirectionFromMask(0x78u) != -1) return 19;
                if (Track_CornerDirectionFromMask(0x1Cu) != 0) return 20;
                if (Track_CornerDirectionFromMask(0x38u) != 0) return 21;
                if (Track_CornerDirectionFromMask(0xFFu) != 0) return 22;
                if (Track_CornerDirectionFromMask(0x8Du) != 0) return 23;
                if (Track_CornerDirectionFromMask(0x09u) != 0) return 28;
                if (Track_CornerDirectionFromMask(0x90u) != 0) return 29;
                if (Track_CornerDirectionFromMask(0x0Du) != 0) return 30;
                if (Track_CornerDirectionFromMask(0xB0u) != 0) return 31;
                if (Track_CornerDirectionFromMask(0x37u) != 1) return 32;
                if (Track_CornerDirectionFromMask(0xECu) != -1) return 33;
                if (!Track_CornerEvidenceRecent(-1, 20u, UINT32_MAX - 10u, 32u)) return 24;
                if (Track_CornerEvidenceRecent(0, 20u, 10u, 32u)) return 25;
                if (Track_CornerEvidenceRecent(1, 50u, 10u, 32u)) return 26;
                if (Track_CenteredEvidenceRecent(false, 20u, 10u, 32u)) return 34;
                if (!Track_CenteredEvidenceRecent(true, 20u, 10u, 32u)) return 35;
                if (Track_CenteredEvidenceRecent(true, 50u, 10u, 32u)) return 36;
                if (!Track_CenteredEvidenceRecent(true, 9u, UINT32_MAX - 10u, 32u)) return 37;
                if (!Track_CornerArmed(600u, 0u, 300u, 256u, 200u, 300u)) return 27;
                for (int i = 0; i < 2; ++i) {
                    capture = Track_UpdateConfirmFrames(
                        capture, Track_CaptureLineValid(good, 1.0f));
                }
                if (capture != 2u) return 4;
                capture = Track_UpdateConfirmFrames(
                    capture, Track_CaptureLineValid(bad, 1.0f));
                if (capture != 0u) return 5;
                if (!Track_CaptureLineValid(edge, 3.5f)) return 38;
                if (Track_CaptureLineValid(edge, 2.0f)) return 39;
                return 0;
            }
            '''
        )
        self.assertEqual(result.returncode, 0, result.stderr + result.stdout)

    def test_ccw_corner_gate_sequences_do_not_latch_stale_candidates(self):
        result = compile_and_run(
            r'''
            #include <stdbool.h>
            #include <stddef.h>
            #include <stdint.h>
            #include "TASK/track_logic.h"

            static int run_sequence(const uint8_t *masks, size_t count,
                                    bool corner_armed) {
                uint8_t centered = 0u;
                uint8_t confirm = 0u;
                bool centered_recent = false;
                bool line_follow = true;
                int triggers = 0;

                for (size_t i = 0u; i < count; ++i) {
                    track_line_observation_t observation =
                        Track_ClassifyLine8(masks[i], 0.0f, 4u);
                    if (!line_follow) continue;
                    if (observation.type == TRACK_LINE_NORMAL) {
                        centered = Track_UpdateConfirmFrames(
                            centered, Track_Abs_Float(observation.error) <= 0.75f);
                        if (centered >= 2u) centered_recent = true;
                        confirm = 0u;
                        continue;
                    }
                    centered = 0u;
                    confirm = Track_UpdateCornerGate(
                        confirm, Track_SquareCornerEvidence(masks[i], 4u),
                        true, corner_armed, centered_recent);
                    if (Track_CornerCandidateReady(
                            true, corner_armed, centered_recent, confirm, 1u)) {
                        ++triggers;
                        line_follow = false;
                    }
                }
                return triggers;
            }

            int main(void) {
                const uint8_t with_white[] = {0x18u, 0x18u, 0x04u, 0x00u,
                                              0x00u, 0x87u, 0x87u};
                const uint8_t direct[] = {0x18u, 0x18u, 0x04u, 0x87u};
                uint8_t confirm = 0u;
                if (run_sequence(with_white, sizeof(with_white), true) != 1) return 1;
                if (run_sequence(direct, sizeof(direct), true) != 1) return 2;
                if (run_sequence(direct, sizeof(direct), false) != 0) return 3;

                /* A candidate seen while disarmed cannot be carried across the
                 * retrigger guard, and expired centered evidence cannot reopen it. */
                confirm = Track_UpdateCornerGate(0u, true, true, false, true);
                if (confirm != 0u) return 4;
                confirm = Track_UpdateCornerGate(confirm, true, true, true, false);
                if (confirm != 0u) return 5;
                if (Track_CornerCandidateReady(true, true, false, 1u, 1u)) return 6;
                if (Track_SquareCornerEvidence(0x00u, 4u)) return 7;
                if (Track_SquareCornerEvidence(0xFFu, 4u)) return 8;
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

    def test_periodic_logs_are_gated_by_motion_state(self):
        source = (ROOT / "main.c").read_text(encoding="utf-8")
        vehicle_log = source.split("static void Main_LogVehicleStatus", 1)[1].split(
            "static void Main_LogPathFish", 1
        )[0]
        sensor_log = source.split("static void Main_LogPathFish", 1)[1].split(
            "static void Main_UpdateOled", 1
        )[0]

        self.assertIn("if (!Track_State_Allows_Motor(Track.state)) return;", vehicle_log)
        self.assertIn("if (!Track_State_Is_LineTracking(Track.state)) return;", sensor_log)
        self.assertNotIn('"FISH_DATA"', sensor_log)

    def test_safety_health_ignores_heartbeat_and_keeps_sensor_faults(self):
        result = compile_and_run(
            r'''
            #include <stdint.h>
            #include "APP/Inc/safety_logic.h"

            int main(void) {
                safety_health_t h = {0};
                if (Safety_Logic_Check(200u, &h, 50u) != SAFETY_LOGIC_OK) return 1;
                h.path_fish_required = true;
                h.path_fish_seen = true;
                h.last_path_fish_tick = UINT32_MAX - 10u;
                if (Safety_Logic_Check(9u, &h, 50u) != SAFETY_LOGIC_OK) return 7;
                if (Safety_Logic_Check(50u, &h, 50u) != SAFETY_LOGIC_PATH_FISH_EXPIRED) return 8;
                h.path_fish_required = false;
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

        motor_header = (ROOT / "APP" / "Inc" / "app_encoder_motor.h").read_text(
            encoding="utf-8"
        )
        self.assertIn("#define LEFT_ENC_FORWARD_SIGN       (1)", motor_header)
        self.assertIn("#define RIGHT_ENC_FORWARD_SIGN      (-1)", motor_header)
        self.assertIn(
            ".input1_pwm = {MOTOR_LEFT_AIN2_PWM_TIMER, MOTOR_LEFT_AIN2_PWM_CHANNEL}",
            motor_source,
        )
        self.assertIn(
            ".input2_pwm = {MOTOR_LEFT_AIN1_PWM_TIMER, MOTOR_LEFT_AIN1_PWM_CHANNEL}",
            motor_source,
        )

    def test_path_fish_bus_fault_is_recovered_and_backed_off(self):
        binding = (ROOT / "APP" / "Src" / "app_path_fish.c").read_text(
            encoding="utf-8"
        )
        header = (ROOT / "APP" / "Inc" / "app_path_fish.h").read_text(
            encoding="utf-8"
        )
        syscfg = (ROOT / "TI_moban.syscfg").read_text(encoding="utf-8")
        main = (ROOT / "main.c").read_text(encoding="utf-8")
        common = (ROOT / "APP" / "Inc" / "app_common.h").read_text(
            encoding="utf-8"
        )

        self.assertIn("PATH_FISH_FAULT_RETRY_PERIOD_MS 250u", header)
        self.assertIn("PATH_FISH_BUS_RECOVERY_PERIOD_MS 1000u", header)
        self.assertIn("PATH_FISH_TRANSFER_TIMEOUT_MS    10u", header)
        self.assertIn("HW_PathFish_RecoverBus()", binding)
        self.assertIn("pulse < 9u", binding)
        self.assertIn("DL_GPIO_RESISTOR_PULL_UP", binding)
        self.assertIn("DL_I2C_CONTROLLER_STATUS_BUSY_BUS) == 0u", binding)
        self.assertIn("bus_monitor == PATH_FISH_BUS_LEVEL_MASK", binding)
        idle_recovery = binding.split(
            "if (s_transfer_state == PATH_FISH_TRANSFER_IDLE &&", 1
        )[1].split("if (s_transfer_state == PATH_FISH_TRANSFER_IDLE &&", 1)[0]
        self.assertIn("DL_I2C_CONTROLLER_STATUS_IDLE) != 0u", idle_recovery)
        self.assertIn("bus_monitor != PATH_FISH_BUS_LEVEL_MASK", idle_recovery)
        self.assertNotIn("DL_I2C_CONTROLLER_STATUS_BUSY_BUS) != 0u", idle_recovery)
        self.assertIn("s_last_request_at = now;", binding)
        self.assertIn('I2C1.sdaPinConfig.internalResistor     = "PULL_UP"', syscfg)
        self.assertIn('I2C1.sclPinConfig.internalResistor     = "PULL_UP"', syscfg)
        self.assertIn("#define APP_PATH_FISH_I2C_DIAG_ENABLE     0", common)
        self.assertIn("#if APP_PATH_FISH_I2C_DIAG_ENABLE", main)
        self.assertIn("flt=%lu,%lu,%lu stuck=%lu rec=%lu/%lu", main)
        self.assertNotIn('"AGE:%03lu"', main)
        self.assertIn('"TRACK:%s"', main)
        self.assertIn('"CORNER:%s"', main)
        self.assertIn('"LAPS:%u"', main)
        self.assertIn("line[i + 2u] = Path_Fish_Sensor.digital[i] ? '1' : '0';", main)

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

    def test_motion_rejections_are_specific_and_sensor_faults_latch(self):
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
        self.assertIn('"path_fish_not_ready"', tuning_source)
        self.assertNotIn('"link_not_ready"', tuning_source)
        self.assertNotIn("Safety_ArmRemoteMotion", tuning_source)
        self.assertNotIn("Safety_DisarmRemoteMotion", tuning_source)
        self.assertIn('"motion_busy"', tuning_source)
        self.assertIn("SAFETY_STATE_BOOT", motor_start)
        self.assertIn("SAFETY_FAULT_PATH_FISH_EXPIRED", safety_source)
        fault_branch = main_source.split(
            "} else if (Safety_GetState() == SAFETY_STATE_FAULT)", 1
        )[1].split("static void Main_LogVehicleStatus", 1)[0]
        self.assertNotIn("Safety_ClearFault();", fault_branch)

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
                if (!Track_RightAngle_HeadingSettled(1.5f, 6.0f)) return 1;
                if (Track_RightAngle_HeadingSettled(1.6f, 0.0f)) return 2;
                if (Track_RightAngle_HeadingSettled(0.0f, 6.1f)) return 3;
                return 0;
            }
            '''
        )
        self.assertEqual(result.returncode, 0, result.stderr + result.stdout)

    def test_square_corner_detection_uses_12_bit_count_and_fixed_left_turn(self):
        result = compile_and_run(
            r'''
            #include <stdbool.h>
            #include "TASK/track_logic.h"

            int main(void) {
                if (Track_SquareCornerEvidence12(0x01Fu, 6u)) return 1;
                if (!Track_SquareCornerEvidence12(0x03Fu, 6u)) return 2;
                if (!Track_SquareCornerEvidence12(0xFC0u, 6u)) return 3;
                if (Track_SquareCornerEvidence12(0xFFFu, 6u)) return 4;
                if (Track_SquareCornerEvidence12(0x000u, 6u)) return 5;
                if (!Track_SquareCornerEvidence12(0x3FFu, 10u)) return 6;
                if (Track_SquareCornerEvidence12(0x1FFu, 10u)) return 7;
                if (Track_ClassifyLine12(0x03Fu, 0.0f, 6u).type !=
                    TRACK_LINE_LEFT_CORNER) return 8;
                if (Track_ClassifyLine12(0xFC0u, 0.0f, 6u).type !=
                    TRACK_LINE_LEFT_CORNER) return 9;
                if (Track_CornerTurnSign(TRACK_LINE_LEFT_CORNER) != 1) return 10;
                return 0;
            }
            '''
        )
        self.assertEqual(result.returncode, 0, result.stderr + result.stdout)

    def test_all_masks_preserve_left_right_mirror_invariants(self):
        result = compile_and_run(
            r'''
            #include "TASK/track_logic.h"

            static uint16_t mirror12(uint16_t mask) {
                uint16_t mirror = 0u;
                for (uint8_t bit = 0u; bit < 12u; bit++) {
                    if ((mask & (uint16_t)(1u << bit)) != 0u) {
                        mirror |= (uint16_t)(1u << (11u - bit));
                    }
                }
                return mirror;
            }

            int main(void) {
                for (uint16_t mask = 0u; mask <= 0x0FFFu; mask++) {
                    uint16_t mirror = mirror12(mask);
                    track_line_observation_t a =
                        Track_ClassifyLine12(mask, 0.0f, 6u);
                    track_line_observation_t b =
                        Track_ClassifyLine12(mirror, 0.0f, 6u);
                    if (mirror12(mirror) != mask) return 1;
                    if (Track_CountBits12(mask) != Track_CountBits12(mirror)) return 2;
                    if (a.type != b.type) return 3;
                    if (a.type == TRACK_LINE_NORMAL && b.type == TRACK_LINE_NORMAL &&
                        Track_Abs_Float(a.error + b.error) > 0.001f) return 4;
                }
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
        timeout = source.rsplit(
            "if (Track_Timeout_Reached(now, Track.state_timer", 1
        )[1].split("/* 正方形计圈", 1)[0]
        self.assertNotIn("right_angle_count++", timeout)
        self.assertNotIn("Track.state = TRACK_STATE_FINISHED;", timeout)
        self.assertNotIn("Track_Motor_Stop_Immediate();", timeout)
        self.assertIn("Track.corner_phase == TRACK_CORNER_PIVOT", source)
        self.assertIn('CORNER_RETRY phase=pivot', source)
        self.assertIn('CORNER_REPIVOT reason=capture_timeout', source)
        self.assertIn('CORNER_REPIVOT reason=pose', source)
        pose_repivot = source.split(
            'CORNER_REPIVOT reason=pose', 1
        )[0].rsplit(
            'if (Track.corner_phase == TRACK_CORNER_CAPTURE', 1
        )[1]
        self.assertIn("observation.type != TRACK_LINE_NORMAL", pose_repivot)
        self.assertIn("turn_progress >= TRACK_RIGHT_ANGLE_SEEK_MIN_TURN_DEG", source)
        self.assertIn('"seek_progress" : "gyro_settled"', source)
        self.assertNotIn("Track_RightAngle_HybridHandoff", source)
        self.assertIn("Track_EncoderDistanceReached", source)
        self.assertNotIn("CORNER_FAIL reason=approach_cal_missing", source)
        self.assertIn("Track.corner_phase = TRACK_CORNER_APPROACH;", source)
        self.assertIn("TRACK_RIGHT_ANGLE_APPROACH_FALLBACK_MS", source)
        self.assertIn("Track_SquareCornerEvidence12", source)
        self.assertIn("Track_BeginRightAngle(now, digital_mask, trigger, 1);", source)
        self.assertIn("((Track.line_mask_history << 12) | Track.line_mask)", source)
        self.assertIn("Track_Apply_Rotate_Envelope(angle_error, Imu_Sensor.gyro_z);", source)
        self.assertIn("Track_Handle_RightAngle(observation, line_frame_fresh);", source)
        self.assertIn("Track_CornerCapturePoseValid", source)
        self.assertNotIn("Track_CornerSequenceExpired", source)
        self.assertNotIn("Track_Corner_Abort", source)
        self.assertIn("encoder_distance_reached || approach_time_reached", source)
        self.assertIn("Path_Fish_TakeFault()", source)
        self.assertIn("SENSOR_BAD reason=path_fish_transport", source)
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

    def test_line_lost_recovers_for_600ms_then_faults_and_manual_stop_clears(self):
        track_source = (ROOT / "TASK" / "track.c").read_text(encoding="utf-8")
        detect = track_source.split("static void Track_Detect_Pattern", 1)[1].split(
            "static void Track_Motor_Stop_Immediate", 1
        )[0]
        stop = track_source.split("void Track_Stop(void)", 1)[1].split(
            "bool Track_Start_Rotation", 1
        )[0]
        self.assertNotIn("Track.state = TRACK_STATE_LINE_LOST", detect)
        self.assertIn("USER_LINE_LOST_RECOVERY_MS", detect)
        self.assertIn("Safety_NotifyLineLostTimeout();", detect)
        self.assertIn('"white_gap"', detect)
        line_follow = track_source.split("case TRACK_STATE_LINE_FOLLOW:", 1)[1].split(
            "case TRACK_STATE_RIGHT_ANGLE:", 1
        )[0]
        self.assertNotIn("invalid_line_confirm", line_follow)
        self.assertNotIn("Track.target_speed = 0", line_follow)
        self.assertNotIn("Track.base_speed = 0", line_follow)
        self.assertIn("Track_LimitRecoveryError", line_follow)
        self.assertIn("Track_LineRecoveryTurn", line_follow)
        self.assertIn("Track.line_heading_reference - Imu_Sensor.heading", line_follow)
        self.assertIn("USER_LINE_RECOVERY_SPEED", line_follow)
        self.assertIn("if (Track.line_type == TRACK_LINE_NORMAL)", line_follow)
        self.assertIn("previous_line_type != TRACK_LINE_NORMAL", track_source)
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

    def test_start_requires_fresh_path_fish_but_not_centered_line(self):
        track_source = (ROOT / "TASK" / "track.c").read_text(encoding="utf-8")
        start = track_source.split("bool Track_Start(void)", 1)[1].split(
            "void Track_Stop(void)", 1
        )[0]
        self.assertIn("Path_Fish_IsFresh(now)", start)
        self.assertIn("Safety_PathFishReady()", start)
        self.assertNotIn("start_line.type", start)
        self.assertNotIn("path_fish_not_centered", start)
        self.assertIn('s_track_start_result = "path_fish_not_ready"', start)
        self.assertLess(
            start.index("Track_Imu_Healthy"),
            start.index("Safety_TransitionToRunning"),
        )
        self.assertLess(
            start.index("Path_Fish_IsFresh"),
            start.index("Safety_TransitionToRunning"),
        )

        tuning_source = (ROOT / "APP" / "Src" / "app_tuning.c").read_text(
            encoding="utf-8"
        )
        self.assertIn("*error = Track_Get_Start_Result();", tuning_source)

    def test_oled_and_buzzer_follow_tracking_events(self):
        source = (ROOT / "main.c").read_text(encoding="utf-8")
        feedback = source.split("static void Main_UpdateTrackFeedback", 1)[1].split(
            "static bool Main_PolarityAllowed", 1
        )[0]
        self.assertIn("Track_State_Is_LineTracking", feedback)
        self.assertIn("Buzzer_Request(1u, now);", feedback)
        self.assertIn("Buzzer_Request(completed, now);", feedback)
        self.assertIn("Buzzer_Request(3u, now);", feedback)
        self.assertIn("Track.state == TRACK_STATE_FINISHED", feedback)
        keys = source.split("static void Main_ProcessKeys", 1)[1].split(
            "static void Main_ProcessDebugUart", 1
        )[0]
        self.assertIn("Buzzer_Request(action_feedback ? 2u : 1u", keys)
        self.assertNotIn("key == 0u || key == 1u", keys)
        oled = source.split("static const char *Main_SensorIssue", 1)[1].split(
            "static void Main_UpdateTrackFeedback", 1
        )[0]
        self.assertIn('App_Oled_SetLine(1u, "SENSOR:ERROR")', oled)
        self.assertIn('return "FISH:NO DATA"', oled)
        self.assertIn('return "IMU:ERROR"', oled)

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
                if (Track_LineRecoveryTurn(-1.0f, 5.0f, 0.0f,
                        1.5f, 0.3f, 0.06f, 6.0f, 10.0f) != -3.0f) return 21;
                return 0;
            }
            '''
        )
        self.assertEqual(result.returncode, 0, result.stderr + result.stdout)

    def test_rotate_logic_validates_direction_and_completion(self):
        result = compile_and_run(
            r'''
            #define TRACK_ROTATE_DONE_ERR_DEG 1.5f
            #include "TASK/track_logic.h"

            int main(void) {
                int32_t left = 0, right = 0;
                if (!Track_Rotate_TargetValid(90.0f)) return 1;
                if (Track_Rotate_TargetValid(0.0f)) return 2;
                if (Track_Rotate_TargetValid(180.1f)) return 3;
                if (Track_Rotate_Error(100.0f, 130.0f, 90.0f) != 60.0f) return 4;
                if (!Track_Rotate_Complete(1.5f, 6.0f)) return 5;
                if (Track_Rotate_Complete(1.6f, 0.0f)) return 6;
                if (!Track_Rotate_WrongWay(90.0f, -15.0f, 15.0f)) return 26;
                if (Track_Rotate_WrongWay(90.0f, 15.0f, 15.0f)) return 27;
                if (Track_Rotate_WrongWay(-90.0f, -15.0f, 15.0f)) return 28;
                if (!Track_Rotate_AngleInside(-1.5f)) return 9;
                if (Track_Rotate_AngleInside(1.6f)) return 10;
                if (Track_Rotate_SpeedLimit(20.0f, 20.0f, 40.0f, 5.0f, 16.0f) != 5.0f) return 11;
                if (Track_Rotate_SpeedLimit(40.0f, 20.0f, 40.0f, 5.0f, 16.0f) != 16.0f) return 12;
                if (Track_Rotate_SpeedLimit(30.0f, 20.0f, 40.0f, 5.0f, 16.0f) != 10.5f) return 13;
                if (!Track_Rotate_ShouldCoast(5.0f, 8.0f, 3.5f, 4.0f)) return 14;
                if (Track_Rotate_ShouldCoast(5.0f, -1.0f, 3.5f, 4.0f)) return 15;
                if (Track_Rotate_ShouldCoast(5.0f, 0.0f, 3.5f, 4.0f)) return 16;
                if (Track_Rotate_ShouldCoast(5.0f, 8.0f, 4.0f, 4.0f)) return 17;
                if (Track_CaptureBlendTurn(0.0f, -1.5f, true, 3.5f, 3.0f, 8.0f) != -3.0f) return 18;
                if (Track_CaptureBlendTurn(2.0f, -3.5f, true, 3.5f, 3.0f, 8.0f) != -1.0f) return 19;
                if (Track_CaptureBlendTurn(2.0f, -3.5f, false, 3.5f, 3.0f, 8.0f) != 2.0f) return 20;
                if (Track_CornerTargetHeading(10.0f, 1, 90.0f) != 100.0f) return 27;
                if (Track_CornerTargetHeading(10.0f, -1, 90.0f) != -80.0f) return 28;
                if (Track_CornerTargetHeading(10.0f, -1, -90.0f) != -80.0f) return 29;
                if (Track_CornerTurnProgress(10.0f, 95.0f, 1) != 85.0f) return 30;
                if (Track_CornerTurnProgress(10.0f, -75.0f, -1) != 85.0f) return 31;
                if (Track_CornerTurnProgress(10.0f, 95.0f, -1) >= 0.0f) return 32;
                if (Track_CornerCapturePoseValid(79.0f, 2.0f, 80.0f, 12.0f)) return 33;
                if (!Track_CornerCapturePoseValid(80.0f, 10.0f, 80.0f, 12.0f)) return 34;
                if (Track_CornerCapturePoseValid(450.0f, -360.0f, 80.0f, 12.0f)) return 35;
                if (!Track_CornerCapturePoseValid(70.0f, 20.0f, 70.0f, 20.0f)) return 36;
                if (Track_CornerCapturePoseValid(450.0f, -360.0f, 70.0f, 20.0f)) return 37;
                if (Track_CaptureHeadingPriority(8.0f, -1.0f, 2.0f, 3.0f) != -1.0f) return 38;
                if (Track_CaptureHeadingPriority(-8.0f, 1.0f, -2.0f, 3.0f) != 1.0f) return 39;
                if (Track_CaptureHeadingPriority(2.0f, -1.0f, 2.0f, 3.0f) != 2.0f) return 40;
                if (Track_CaptureHeadingPriority(8.0f, -1.0f, 0.0f, 3.0f) != -1.0f) return 41;
                if (Track_CaptureHeadingPriority(-8.0f, 1.0f, 0.0f, 3.0f) != 1.0f) return 42;
                if (!Track_Rotate_StallCandidate(10.0f, 0.0f, 0, 0, 6.0f, 15.0f, 2.0f, 1)) return 46;
                if (!Track_Rotate_StallCandidate(-10.0f, 0.0f, -1, 1, 6.0f, 15.0f, 2.0f, 1)) return 47;
                if (Track_Rotate_StallCandidate(4.0f, 0.0f, 0, 0, 6.0f, 15.0f, 2.0f, 1)) return 48;
                if (Track_Rotate_StallCandidate(16.0f, 0.0f, 0, 0, 6.0f, 15.0f, 2.0f, 1)) return 49;
                if (Track_Rotate_StallCandidate(10.0f, 2.1f, 0, 0, 6.0f, 15.0f, 2.0f, 1)) return 50;
                if (Track_Rotate_StallCandidate(10.0f, 0.0f, 2, 0, 6.0f, 15.0f, 2.0f, 1)) return 51;
                if (Track_Rotate_RecoveryMoving(4.0f, 1, -1, 4.0f, 1)) return 52;
                if (!Track_Rotate_RecoveryMoving(4.1f, 0, 0, 4.0f, 1)) return 53;
                if (!Track_Rotate_RecoveryMoving(0.0f, 2, 0, 4.0f, 1)) return 54;
                if (Track_Rotate_FeedbackFault(10, -10, 4) !=
                    TRACK_ROTATE_FEEDBACK_OK) return 55;
                if (Track_Rotate_FeedbackFault(2, -10, 4) !=
                    TRACK_ROTATE_FEEDBACK_LEFT_MISSING) return 56;
                if (Track_Rotate_FeedbackFault(10, -2, 4) !=
                    TRACK_ROTATE_FEEDBACK_RIGHT_MISSING) return 57;
                if (Track_Rotate_FeedbackFault(2, -2, 4) !=
                    TRACK_ROTATE_FEEDBACK_BOTH_MISSING) return 58;
                if (Track_Rotate_FeedbackFault(-4, 4, 4) !=
                    TRACK_ROTATE_FEEDBACK_OK) return 59;
                Track_Rotate_Command(90.0f, 30, &left, &right);
                if (left >= 0 || right <= 0) return 7;
                Track_Rotate_Command(-90.0f, 30, &left, &right);
                return (left > 0 && right < 0) ? 0 : 8;
            }
            '''
        )
        self.assertEqual(result.returncode, 0, result.stderr + result.stdout)

    def test_square_course_corner_direction_is_always_counter_clockwise(self):
        result = compile_and_run(
            r'''
            #include "TASK/track_logic.h"

            int main(void) {
                const float entry = 10.0f;
                int8_t left_mask_sign = Track_CornerTurnSign(
                    Track_ClassifyLine12(0x03Fu, 0.0f, 6u).type);
                int8_t right_mask_sign = Track_CornerTurnSign(
                    Track_ClassifyLine12(0xFC0u, 0.0f, 6u).type);
                float left_target =
                    Track_CornerTargetHeading(entry, left_mask_sign, 90.0f);
                float right_target =
                    Track_CornerTargetHeading(entry, right_mask_sign, 90.0f);
                if (left_mask_sign != 1 || right_mask_sign != 1) return 1;
                if (left_target != 100.0f || right_target != 100.0f) return 2;
                return 0;
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
        self.assertIn("USER_ROTATE_PID_MAX_SPEED       24.0f", tuning)
        self.assertIn("USER_ROTATE_PID_MIN_SPEED       8", tuning)
        self.assertIn("USER_ROTATE_PID_FINE_ZONE_DEG    10.0f", tuning)
        self.assertIn("USER_ROTATE_PID_BLEND_ZONE_DEG   25.0f", tuning)
        self.assertIn("USER_ROTATE_PID_FINE_MAX_SPEED    9.0f", tuning)
        self.assertIn("USER_ROTATE_PID_COAST_OUTPUT    4.0f", tuning)
        self.assertIn("USER_ROTATE_DONE_ERR_DEG          1.5f", tuning)
        self.assertIn("USER_ROTATE_STALL_KICK_SPEED    32", tuning)
        self.assertIn("USER_ROTATE_STALL_CONFIRM_FRAMES 20u", tuning)
        self.assertIn("USER_ROTATE_STALL_MIN_ERROR_DEG   6.0f", tuning)
        self.assertIn("USER_ROTATE_STALL_MAX_ERROR_DEG 180.0f", tuning)
        self.assertIn("USER_ROTATE_STALL_KICK_HOLD_FRAMES 40u", tuning)
        self.assertIn("USER_ROTATE_WRONG_WAY_FRAMES       20u", tuning)
        self.assertIn("USER_ROTATE_FEEDBACK_GRACE_MS      150u", tuning)
        self.assertIn("USER_ROTATE_FEEDBACK_MIN_COUNTS      4", tuning)
        self.assertIn("USER_RIGHT_ANGLE_APPROACH_SPEED  28", tuning)
        self.assertIn("USER_RIGHT_ANGLE_CONFIRM_FRAMES 1", tuning)
        self.assertIn("USER_RIGHT_ANGLE_WHITE_FALLBACK_FRAMES 2u", tuning)
        self.assertIn("USER_RIGHT_ANGLE_WHITE_RECENT_MS 100u", tuning)
        self.assertIn("USER_RIGHT_ANGLE_DONE_ERROR       1.5f", tuning)
        self.assertIn("USER_RIGHT_ANGLE_DONE_RATE        6.0f", tuning)
        self.assertIn("USER_RIGHT_ANGLE_DONE_FRAMES     20u", tuning)
        self.assertIn("USER_RIGHT_ANGLE_SEEK_MIN_TURN_DEG 80.0f", tuning)
        self.assertIn("USER_RIGHT_ANGLE_CAPTURE_SPEED    18", tuning)
        self.assertIn("USER_RIGHT_ANGLE_APPROACH_FALLBACK_MS 750u", tuning)
        self.assertIn("USER_RIGHT_ANGLE_TIMEOUT   3500", tuning)
        self.assertNotIn("USER_RIGHT_ANGLE_TOTAL_TIMEOUT_MS", tuning)
        self.assertNotIn("USER_RIGHT_ANGLE_MAX_RETRIES", tuning)
        self.assertIn("USER_RIGHT_ANGLE_CAPTURE_LINE_ERROR 3.5f", tuning)
        self.assertIn("USER_LINE_RECOVERY_HEADING_KP 0.30f", tuning)
        self.assertIn("USER_LINE_RECOVERY_SPEED     12", tuning)
        self.assertIn("USER_LINE_LOST_RECOVERY_MS   600u", tuning)
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
                if (Motor_Pwm_DutyToCompare(0u, 100u) != 0u) return 1;
                if (Motor_Pwm_DutyToCompare(20u, 100u) != 20u) return 2;
                if (Motor_Pwm_DutyToCompare(100u, 100u) != 100u) return 3;
                if (Motor_Pwm_DutyToCompare(120u, 100u) != 100u) return 4;
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

    def test_esp_connection_state_machine_never_blocks_for_at_response(self):
        source = (ROOT / "APP" / "Src" / "app_esp01s.c").read_text(
            encoding="utf-8"
        )
        connection = source.split("6. 连接状态机", 1)[1].split(
            "7. 业务切入点", 1
        )[0]
        self.assertIn("conn_at_start", connection)
        self.assertIn("conn_at_poll", connection)
        self.assertNotIn("Bsp_ESP_AT_SendCmd(", connection)
        self.assertNotIn("Bsp_Delay_ms(", connection)

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
        self.assertIn("!Track.use_speed_pid && !pivot_turn && !capture_turn", output)

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

    def test_white_frames_never_stop_or_start_corner(self):
        source = (ROOT / "TASK" / "track.c").read_text(encoding="utf-8")
        detect = source.split("static void Track_Detect_Pattern", 1)[1].split(
            "static void Track_Motor_Stop_Immediate", 1
        )[0]
        self.assertNotIn("TRACK_LINE_LOST_CONFIRM_FRAMES", detect)
        self.assertNotIn("Track.state = TRACK_STATE_LINE_LOST;", detect)
        self.assertNotIn("Track.state = TRACK_STATE_FINISHED;", detect)
        self.assertNotIn("Track_Handle_LineLost", source)
        self.assertIn("Track_CenteredEvidenceRecent", detect)
        self.assertNotIn('Track_BeginRightAngle(now, digital_mask, "white", 1);', detect)
        self.assertIn("Track_UpdateCornerGate", detect)
        self.assertNotIn("TRACK_LINE_RIGHT_CORNER", detect)

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
