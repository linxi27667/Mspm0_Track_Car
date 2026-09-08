import importlib.util
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "motor_qualification_analyzer",
    ROOT / "scripts" / "analyze_motor_qualification.py",
)
ANALYZER = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(ANALYZER)


def complete_run(offset: int = 0) -> str:
    lines = ["I/MOTORTEST start phase=0 open_loop pwm=20 timeout=12600"]
    for phase, target, left, right in (
        (0, 20, 2970, 3040), (1, 30, 4800, 4900), (2, 40, 6600, 6710)
    ):
        lines.append(
            f"I/MOTORTEST open phase={phase} pwm={target} "
            f"Lavg={left + offset} Ravg={right + offset} (x1000) n=161"
        )
    for phase, target, left, right, exp_l, exp_r in (
        (3, 20, 3000, 3070, 3220, 3280),
        (4, 30, 4480, 4570, 4830, 4920),
        (5, 40, 6150, 6260, 6440, 6560),
        (6, -20, -2950, -3010, -3220, -3280),
        (7, -30, -4440, -4530, -4830, -4920),
        (8, -40, -6110, -6230, -6440, -6560),
    ):
        lines.append(
            f"I/MOTORTEST closed phase={phase} target={target} "
            f"Lavg={left + offset} Ravg={right + offset} expL={exp_l} expR={exp_r} "
            "mismatch=100 (x1000) pass=1"
        )
    lines.extend(
        [
            "I/MOTORTEST stop phase=9 Labs=0 Rabs=0 (x1000) pass=1",
            "I/MOTORTEST end result=complete phase=9 target=0 L=1 R=1",
        ]
    )
    return "\n".join(lines)


class MotorQualificationAnalyzerTests(unittest.TestCase):
    def test_two_complete_repeatable_runs_pass(self):
        runs = ANALYZER.parse_runs(complete_run() + "\n" + complete_run(5))
        report = ANALYZER.analyze(runs, expected_runs=2, max_cv=0.05)
        self.assertTrue(report["bench_qualified"])
        self.assertEqual(report["complete_runs"], 2)

    def test_missing_phase_fails_closed(self):
        runs = ANALYZER.parse_runs(complete_run().replace("closed phase=8", "ignored phase=8"))
        report = ANALYZER.analyze(runs, expected_runs=1, max_cv=0.05)
        self.assertFalse(report["bench_qualified"])
        self.assertTrue(any("missing closed phases" in item for item in report["failures"]))

    def test_excessive_cross_run_variation_fails(self):
        runs = ANALYZER.parse_runs(complete_run() + "\n" + complete_run(1000))
        report = ANALYZER.analyze(runs, expected_runs=2, max_cv=0.05)
        self.assertFalse(report["bench_qualified"])
        self.assertTrue(any("repeatability CV" in item for item in report["failures"]))


if __name__ == "__main__":
    unittest.main()

