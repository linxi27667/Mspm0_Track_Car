"""Parse repeated MOTORTEST RTT evidence and produce strict JSON/Markdown verdicts."""

from __future__ import annotations

import argparse
import json
import math
import re
import statistics
from pathlib import Path

OPEN_RE = re.compile(r"open phase=(\d+) pwm=(-?\d+) Lavg=(-?\d+) Ravg=(-?\d+)")
CLOSED_RE = re.compile(
    r"closed phase=(\d+) target=(-?\d+) Lavg=(-?\d+) Ravg=(-?\d+) "
    r"expL=(-?\d+) expR=(-?\d+) mismatch=(-?\d+).*pass=(\d+)"
)
STOP_RE = re.compile(r"stop phase=(\d+) Labs=(\d+) Rabs=(\d+).*pass=(\d+)")
END_RE = re.compile(r"end result=([^ ]+) phase=(\d+) target=(-?\d+)")


def coefficient_of_variation(values: list[float]) -> float:
    mean = statistics.fmean(values)
    return statistics.pstdev(values) / mean if mean else math.inf


def parse_runs(text: str) -> list[dict[str, object]]:
    runs: list[dict[str, object]] = []
    current: dict[str, object] | None = None
    for line in text.splitlines():
        if "MOTORTEST" not in line:
            continue
        if "start phase=0" in line:
            current = {"open": {}, "closed": {}, "stop": None, "end": None}
            runs.append(current)
            continue
        if current is None:
            continue
        if match := OPEN_RE.search(line):
            phase, target, left, right = map(int, match.groups())
            current["open"][phase] = {"target": target, "left": left, "right": right}
        elif match := CLOSED_RE.search(line):
            phase, target, left, right, exp_l, exp_r, mismatch, passed = map(int, match.groups())
            current["closed"][phase] = {
                "target": target, "left": left, "right": right,
                "expected_left": exp_l, "expected_right": exp_r,
                "mismatch_milli_speed": mismatch, "pass": bool(passed),
            }
        elif match := STOP_RE.search(line):
            phase, left, right, passed = map(int, match.groups())
            current["stop"] = {"phase": phase, "left_abs": left, "right_abs": right, "pass": bool(passed)}
        elif match := END_RE.search(line):
            result, phase, target = match.groups()
            current["end"] = {"result": result, "phase": int(phase), "target": int(target)}
    return runs


def analyze(runs: list[dict[str, object]], expected_runs: int, max_cv: float) -> dict[str, object]:
    failures: list[str] = []
    if len(runs) != expected_runs:
        failures.append(f"expected {expected_runs} runs, found {len(runs)}")
    complete_runs = 0
    for index, run in enumerate(runs, 1):
        if set(run["open"]) != {0, 1, 2}:
            failures.append(f"run {index}: missing open phases")
        if set(run["closed"]) != {3, 4, 5, 6, 7, 8}:
            failures.append(f"run {index}: missing closed phases")
        if any(not phase["pass"] for phase in run["closed"].values()):
            failures.append(f"run {index}: a closed-loop phase failed")
        if not run["stop"] or not run["stop"]["pass"]:
            failures.append(f"run {index}: stop residual failed")
        if run["end"] and run["end"]["result"] == "complete":
            complete_runs += 1
        else:
            failures.append(f"run {index}: terminal result is not complete")

    cv_by_phase: dict[str, float] = {}
    if len(runs) == expected_runs and expected_runs >= 2:
        for phase in range(3, 9):
            for side in ("left", "right"):
                values = [abs(run["closed"][phase][side]) for run in runs]
                cv = coefficient_of_variation(values)
                cv_by_phase[f"phase_{phase}_{side}"] = cv
                if cv > max_cv:
                    failures.append(f"phase {phase} {side}: repeatability CV {cv:.3%} > {max_cv:.1%}")

    return {
        "bench_qualified": not failures,
        "expected_runs": expected_runs,
        "observed_runs": len(runs),
        "complete_runs": complete_runs,
        "max_allowed_cv": max_cv,
        "max_observed_cv": max(cv_by_phase.values(), default=None),
        "cv_by_phase": cv_by_phase,
        "failures": failures,
        "runs": runs,
        "scope": "unloaded raised-wheel bench only; ground-load tracking remains unproven",
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("log", type=Path)
    parser.add_argument("--runs", type=int, default=5)
    parser.add_argument("--max-cv", type=float, default=0.05)
    parser.add_argument("--json", type=Path, required=True)
    parser.add_argument("--markdown", type=Path, required=True)
    args = parser.parse_args()

    runs = parse_runs(args.log.read_text(encoding="utf-8", errors="replace"))
    report = analyze(runs, args.runs, args.max_cv)
    args.json.parent.mkdir(parents=True, exist_ok=True)
    args.json.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    verdict = "PASS" if report["bench_qualified"] else "FAIL"
    lines = [
        f"# Motor qualification: {verdict}", "",
        f"- Complete runs: {report['complete_runs']}/{report['expected_runs']}",
        f"- Maximum steady-state CV: {report['max_observed_cv']}",
        "- Scope: raised-wheel, unloaded bench verification only.",
        "- Ground decision: PASS only permits a supervised low-speed straight-line test; it does not prove track readiness.",
    ]
    if report["failures"]:
        lines.extend(["", "## Failures", ""] + [f"- {item}" for item in report["failures"]])
    args.markdown.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"[ANALYZE] {verdict}; report={args.json}")
    return 0 if report["bench_qualified"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

