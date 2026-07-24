"""Path Fish 12 路半帧协议与故障语义测试。"""

from __future__ import annotations

import subprocess
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def compile_and_run(source: str) -> subprocess.CompletedProcess[str]:
    with tempfile.TemporaryDirectory(prefix="path_fish_test_") as tmpdir:
        source_path = Path(tmpdir) / "path_fish_test.c"
        executable = Path(tmpdir) / "path_fish_test.exe"
        source_path.write_text(source, encoding="utf-8")
        compiled = subprocess.run(
            [
                "gcc", "-std=c99", "-Wall", "-Wextra",
                "-I", str(ROOT / "BSP" / "Inc"),
                str(source_path),
                str(ROOT / "BSP" / "Src" / "bsp_path_fish.c"),
                "-o", str(executable),
            ],
            check=False,
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            cwd=ROOT,
        )
        if compiled.returncode != 0:
            return compiled
        return subprocess.run(
            [str(executable)],
            check=False,
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            cwd=ROOT,
        )


class PathFishProtocolTests(unittest.TestCase):
    def assert_c_program(self, body: str):
        result = compile_and_run(
            '#include "bsp_path_fish.h"\n'
            "#include <stdint.h>\n"
            "int main(void) {\n"
            f"{body}\n"
            "}\n"
        )
        self.assertEqual(result.returncode, 0, result.stderr + result.stdout)

    def test_ascii_and_raw_halves_publish_atomically(self):
        self.assert_c_program(
            r'''
            bsp_path_fish_t sensor;
            const uint8_t first[7] = {'#', '1', '0', '1', '0', '1', '0'};
            const uint8_t second[7] = {'!', 0, 1, 0, 1, 0, 1};
            Bsp_PathFish_Init(&sensor, true);
            if (Bsp_PathFish_AcceptPacket(&sensor, first, 7u, 100u)) return 1;
            if (sensor.frame_seq != 0u || sensor.data_ready) return 2;
            if (!Bsp_PathFish_AcceptPacket(&sensor, second, 7u, 105u)) return 3;
            if (sensor.frame_seq != 1u || !sensor.data_ready) return 4;
            if (sensor.mask != 0x0A95u || sensor.black_count != 6u) return 5;
            if (sensor.raw[0] != 1u || sensor.raw[11] != 1u) return 6;
            return 0;
            '''
        )

    def test_reverse_half_order_is_accepted(self):
        self.assert_c_program(
            r'''
            bsp_path_fish_t sensor;
            const uint8_t second[7] = {'!', 1, 1, 0, 0, 1, 0};
            const uint8_t first[7] = {'#', 0, 1, 0, 1, 0, 1};
            Bsp_PathFish_Init(&sensor, true);
            if (Bsp_PathFish_AcceptPacket(&sensor, second, 7u, 10u)) return 1;
            if (!Bsp_PathFish_AcceptPacket(&sensor, first, 7u, 20u)) return 2;
            if (sensor.mask != 0x04EAu || sensor.black_count != 6u) return 3;
            return 0;
            '''
        )

    def test_duplicate_and_timeout_do_not_mix_frames(self):
        self.assert_c_program(
            r'''
            bsp_path_fish_t sensor;
            const uint8_t first_a[7] = {'#', 1, 0, 0, 0, 0, 0};
            const uint8_t first_b[7] = {'#', 0, 1, 0, 0, 0, 0};
            const uint8_t second[7] = {'!', 0, 0, 1, 0, 0, 0};
            Bsp_PathFish_Init(&sensor, true);
            (void)Bsp_PathFish_AcceptPacket(&sensor, first_a, 7u, 0u);
            if (Bsp_PathFish_AcceptPacket(&sensor, first_b, 7u, 1u)) return 1;
            if (sensor.error_count != 1u || sensor.frame_seq != 0u) return 2;
            if (!Bsp_PathFish_AcceptPacket(&sensor, second, 7u, 2u)) return 3;
            if (sensor.mask != 0x0102u) return 4;

            (void)Bsp_PathFish_AcceptPacket(&sensor, first_a, 7u, 100u);
            if (Bsp_PathFish_AcceptPacket(&sensor, second, 7u, 116u)) return 5;
            if (sensor.frame_seq != 1u || sensor.error_count != 2u) return 6;
            return 0;
            '''
        )

    def test_invalid_header_payload_and_length_fault(self):
        self.assert_c_program(
            r'''
            bsp_path_fish_t sensor;
            const uint8_t bad_header[7] = {'?', 0, 0, 0, 0, 0, 0};
            const uint8_t bad_value[7] = {'#', 0, 0, 2, 0, 0, 0};
            Bsp_PathFish_Init(&sensor, true);
            if (Bsp_PathFish_AcceptPacket(&sensor, bad_header, 7u, 0u)) return 1;
            if (!sensor.frame_fault || sensor.error_count != 1u) return 2;
            sensor.frame_fault = false;
            if (Bsp_PathFish_AcceptPacket(&sensor, bad_value, 7u, 1u)) return 3;
            if (!sensor.frame_fault || sensor.error_count != 2u) return 4;
            sensor.frame_fault = false;
            if (Bsp_PathFish_AcceptPacket(&sensor, bad_header, 6u, 2u)) return 5;
            if (!sensor.frame_fault || sensor.error_count != 3u) return 6;
            return 0;
            '''
        )

    def test_snapshot_stays_unchanged_until_opposite_half_arrives(self):
        self.assert_c_program(
            r'''
            bsp_path_fish_t sensor;
            const uint8_t first[7] = {'#', 1, 0, 0, 0, 0, 0};
            const uint8_t second[7] = {'!', 0, 0, 0, 0, 0, 1};
            const uint8_t next_first[7] = {'#', 0, 1, 1, 0, 0, 0};
            Bsp_PathFish_Init(&sensor, true);
            (void)Bsp_PathFish_AcceptPacket(&sensor, first, 7u, 0u);
            if (!Bsp_PathFish_AcceptPacket(&sensor, second, 7u, 5u)) return 1;
            if (sensor.mask != 0x0801u || sensor.frame_seq != 1u) return 2;
            sensor.data_ready = false;
            if (Bsp_PathFish_AcceptPacket(&sensor, next_first, 7u, 10u)) return 3;
            if (sensor.mask != 0x0801u || sensor.frame_seq != 1u) return 4;
            if (sensor.data_ready) return 5;
            return 0;
            '''
        )

    def test_polarity_freshness_wraparound_and_transport_fault(self):
        self.assert_c_program(
            r'''
            bsp_path_fish_t sensor;
            const uint8_t first[7] = {'#', 1, 0, 1, 0, 1, 0};
            const uint8_t second[7] = {'!', 0, 1, 0, 1, 0, 1};
            Bsp_PathFish_Init(&sensor, true);
            if (Bsp_PathFish_FrameAge(&sensor, 0u) != UINT32_MAX) return 1;
            (void)Bsp_PathFish_AcceptPacket(&sensor, first, 7u, UINT32_MAX - 5u);
            (void)Bsp_PathFish_AcceptPacket(&sensor, second, 7u, UINT32_MAX - 1u);
            if (Bsp_PathFish_FrameAge(&sensor, 3u) != 5u) return 2;
            if (!Bsp_PathFish_IsFresh(&sensor, 3u, 50u)) return 3;
            Bsp_PathFish_SetBlackPolarity(&sensor, false);
            if (sensor.mask != 0x056Au || sensor.black_count != 6u) return 4;
            Bsp_PathFish_NotifyTransportFault(&sensor);
            if (!sensor.frame_fault || sensor.error_count != 1u) return 5;
            if (Bsp_PathFish_IsFresh(&sensor, 3u, 50u)) return 6;
            return 0;
            '''
        )

    def test_hardware_binding_recovers_transfer_outside_isr(self):
        source = (ROOT / "APP" / "Src" / "app_path_fish.c").read_text(
            encoding="utf-8"
        )
        poll = source[source.index("void Path_Fish_Poll"):
                      source.index("bool Path_Fish_TakeFrame")]
        isr = source[source.index("void PATH_FISH_I2C_INST_IRQHandler"):]
        self.assertIn("HW_PathFish_ResetTransfer();", poll)
        self.assertIn("DL_I2C_disableController(PATH_FISH_I2C_INST)", source)
        self.assertIn("DL_I2C_flushControllerRXFIFO(PATH_FISH_I2C_INST)", source)
        self.assertIn("DL_I2C_enableController(PATH_FISH_I2C_INST)", source)
        self.assertNotIn("Bsp_PathFish_AcceptPacket", isr)
        self.assertNotIn("elog_", isr)
        self.assertLessEqual(isr.count("DL_I2C_receiveControllerData"), 0)


if __name__ == "__main__":
    unittest.main()
