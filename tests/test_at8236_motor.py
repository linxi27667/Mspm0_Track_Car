import subprocess
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def compile_and_run(source: str) -> subprocess.CompletedProcess:
    with tempfile.TemporaryDirectory(prefix="at8236_motor_test_") as tmpdir:
        source_path = Path(tmpdir) / "at8236_motor_test.c"
        exe_path = Path(tmpdir) / "at8236_motor_test.exe"
        source_path.write_text(source, encoding="utf-8")
        compile_result = subprocess.run(
            [
                "gcc",
                "-std=c99",
                "-Wall",
                "-Wextra",
                "-Werror",
                "-I",
                str(ROOT / "BSP" / "Inc"),
                str(source_path),
                str(ROOT / "BSP" / "Src" / "bsp_encoder_motor.c"),
                "-o",
                str(exe_path),
            ],
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
            [str(exe_path)],
            check=False,
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            cwd=ROOT,
        )


class At8236MotorTests(unittest.TestCase):
    def test_signed_command_routes_pwm_to_only_one_input(self):
        result = compile_and_run(
            r'''
            #include <stdint.h>
            #include "bsp_encoder_motor.h"

            typedef struct {
                void *timer;
                uint32_t channel;
                uint32_t duty;
            } pwm_call_t;

            static pwm_call_t calls[4];
            static uint32_t call_count;
            static uint32_t timer_in1;
            static uint32_t timer_in2;

            static void pwm_write(void *timer, uint32_t channel, uint32_t duty)
            {
                calls[call_count].timer = timer;
                calls[call_count].channel = channel;
                calls[call_count].duty = duty;
                call_count++;
            }

            static int expect_call(uint32_t index, void *timer,
                                   uint32_t channel, uint32_t duty)
            {
                return calls[index].timer == timer &&
                       calls[index].channel == channel &&
                       calls[index].duty == duty;
            }

            int main(void)
            {
                bsp_motor_t motor = {
                    .input1_pwm = {&timer_in1, 1u},
                    .input2_pwm = {&timer_in2, 2u},
                    .Pwm_Write = pwm_write
                };

                Bsp_Motor_Write_Pwm(&motor, 40);
                if (call_count != 2u) return 1;
                if (!expect_call(0u, &timer_in2, 2u, 0u)) return 2;
                if (!expect_call(1u, &timer_in1, 1u, 40u)) return 3;
                if (motor.current_pwm != 40) return 4;

                call_count = 0u;
                Bsp_Motor_Write_Pwm(&motor, -35);
                if (call_count != 2u) return 5;
                if (!expect_call(0u, &timer_in1, 1u, 0u)) return 6;
                if (!expect_call(1u, &timer_in2, 2u, 35u)) return 7;
                if (motor.current_pwm != -35) return 8;

                call_count = 0u;
                Bsp_Motor_Write_Pwm(&motor, 0);
                if (call_count != 2u) return 9;
                if (!expect_call(0u, &timer_in1, 1u, 0u)) return 10;
                if (!expect_call(1u, &timer_in2, 2u, 0u)) return 11;

                call_count = 0u;
                Bsp_Motor_Write_Pwm(&motor, 140);
                if (!expect_call(1u, &timer_in1, 1u, 100u)) return 12;
                if (motor.current_pwm != 100) return 13;

                call_count = 0u;
                Bsp_Motor_Write_Pwm(&motor, -140);
                if (!expect_call(1u, &timer_in2, 2u, 100u)) return 14;
                if (motor.current_pwm != -100) return 15;

                return 0;
            }
            '''
        )
        self.assertEqual(result.returncode, 0, result.stderr + result.stdout)

    def test_sysconfig_maps_original_in_pins_to_four_pwm_outputs(self):
        syscfg = (ROOT / "TI_moban.syscfg").read_text(encoding="utf-8")
        generated = (ROOT / "ti_msp_dl_config.h").read_text(encoding="utf-8")

        self.assertIn('PWM1.peripheral.ccp1Pin.$assign         = "PA22";', syscfg)
        self.assertIn('PWM1.peripheral.ccp2Pin.$assign         = "PA15";', syscfg)
        self.assertIn('PWM2.peripheral.ccp0Pin.$assign         = "PB0";', syscfg)
        self.assertIn('PWM3.peripheral.ccp1Pin.$assign         = "PB11";', syscfg)
        self.assertEqual(syscfg.count("dutyCycle            = 0;"), 4)

        self.assertIn("IOMUX_PINCM47_PF_TIMA0_CCP1", generated)
        self.assertIn("IOMUX_PINCM37_PF_TIMA0_CCP2", generated)
        self.assertIn("IOMUX_PINCM12_PF_TIMA1_CCP0", generated)
        self.assertIn("IOMUX_PINCM28_PF_TIMG6_CCP1", generated)


if __name__ == "__main__":
    unittest.main()
