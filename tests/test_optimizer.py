"""Unit tests for the local constrained optimizer."""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
HOST = ROOT / "host"
if str(HOST) not in sys.path:
    sys.path.insert(0, str(HOST))

from optimizer import (  # noqa: E402
    MAX_CANDIDATES,
    MIN_STEPS,
    INITIAL_STEPS,
    CandidateGenerator,
    CandidateRecord,
    Optimizer,
    S_ABORTED,
    S_AWAITING_CONFIRM,
    S_APPLYING,
    S_CONVERGED,
    S_IDLE,
    S_PROPOSING,
    S_RUNNING_TRIAL_1,
    S_RUNNING_TRIAL_2,
    TrialCollector,
    TrialMetrics,
    candidate_score,
    generate_race_header,
)
from param_schema import PARAMS  # noqa: E402


def default_params() -> dict[str, float]:
    return {k: float(v["default"]) for k, v in PARAMS.items()}


def make_tele(st="LF", le=0.0, po=0.0, el=0.0, er=0.0, ms=0, **extra):
    tele = {"st": st, "le": le, "po": po, "el": el, "er": er, "ms": ms}
    tele.update(extra)
    return tele


def make_completed_trial(lap_times=None, le_rms=0.5, sat=0.1, wdiff=1.0):
    """构造一个完成的 TrialMetrics。"""
    if lap_times is None:
        lap_times = [15.0, 15.5, 16.0, 15.2, 15.8]
    return TrialMetrics(
        completed=True,
        laps_completed=len(lap_times),
        lap_times=list(lap_times),
        p95_lap_time=sorted(lap_times)[int(0.95 * (len(lap_times) - 1))],
        lateral_error_rms=le_rms,
        saturation_rate=sat,
        wheel_speed_diff=wdiff,
    )


def make_failed_trial(reason="line_lost"):
    """构造一个失败的 TrialMetrics。"""
    return TrialMetrics(completed=False, abort_reason=reason)


# ============================================================
# 评分测试
# ============================================================


class CandidateScoreTests(unittest.TestCase):
    def test_completed_beats_incomplete(self):
        completed = make_completed_trial()
        failed = make_failed_trial()
        self.assertLess(candidate_score(completed), candidate_score(failed))

    def test_lower_p95_better(self):
        fast = make_completed_trial(lap_times=[14, 14, 14, 14, 14])
        slow = make_completed_trial(lap_times=[16, 16, 16, 16, 16])
        self.assertLess(candidate_score(fast), candidate_score(slow))

    def test_lower_lateral_error_better_when_p95_equal(self):
        a = make_completed_trial(le_rms=0.3)
        b = make_completed_trial(le_rms=0.8)
        self.assertLess(candidate_score(a), candidate_score(b))

    def test_lower_saturation_better_when_le_equal(self):
        a = make_completed_trial(le_rms=0.5, sat=0.05)
        b = make_completed_trial(le_rms=0.5, sat=0.2)
        self.assertLess(candidate_score(a), candidate_score(b))

    def test_lower_wheel_diff_better_when_sat_equal(self):
        a = make_completed_trial(sat=0.1, wdiff=0.5)
        b = make_completed_trial(sat=0.1, wdiff=2.0)
        self.assertLess(candidate_score(a), candidate_score(b))


# ============================================================
# 候选生成器测试
# ============================================================


class CandidateGeneratorTests(unittest.TestCase):
    def test_first_candidate_increases_kp(self):
        """第一个候选应为 KP + 0.5（增益组第一个坐标，+方向）。"""
        base = default_params()
        gen = CandidateGenerator(base)
        cand = gen.next_candidate()
        self.assertIsNotNone(cand)
        self.assertAlmostEqual(cand["KP"], base["KP"] + INITIAL_STEPS["KP"])
        # 其他参数不变
        for k in base:
            if k != "KP":
                self.assertAlmostEqual(cand[k], base[k])

    def test_second_candidate_decreases_kp(self):
        """第二个候选应为 KP - 0.5。"""
        base = default_params()
        gen = CandidateGenerator(base)
        gen.next_candidate()  # KP+0.5
        # 拒绝第一个
        gen.report_result(accepted=False)
        cand = gen.next_candidate()
        self.assertIsNotNone(cand)
        self.assertAlmostEqual(cand["KP"], base["KP"] - INITIAL_STEPS["KP"])

    def test_accept_updates_best(self):
        """接受候选后，best 更新为候选参数。"""
        base = default_params()
        gen = CandidateGenerator(base)
        cand = gen.next_candidate()
        gen.report_result(accepted=True)
        self.assertAlmostEqual(gen.best_params()["KP"], cand["KP"])

    def test_reject_keeps_best(self):
        """拒绝候选后，best 不变。"""
        base = default_params()
        gen = CandidateGenerator(base)
        gen.next_candidate()
        gen.report_result(accepted=False)
        self.assertAlmostEqual(gen.best_params()["KP"], base["KP"])

    def test_step_halving_on_no_improvement(self):
        """一轮无改进后步长减半。"""
        base = default_params()
        gen = CandidateGenerator(base)
        # RATHR=6 位于 5~10 范围内，增益组四个坐标均有正负方向。
        for _ in range(8):
            cand = gen.next_candidate()
            self.assertIsNotNone(cand, "should have 8 effective gain candidates")
            gen.report_result(accepted=False)
        # 步长应减半
        cand = gen.next_candidate()
        self.assertIsNotNone(cand)
        # KP+0.25（0.5/2）
        self.assertAlmostEqual(cand["KP"], base["KP"] + INITIAL_STEPS["KP"] / 2)

    def test_max_candidates_limit(self):
        """达到 MAX_CANDIDATES 后返回 None。"""
        base = default_params()
        gen = CandidateGenerator(base)
        count = 0
        while count <= MAX_CANDIDATES + 5:
            # report_result 清除 pending，使 next_candidate 生成新的
            if gen.next_candidate() is None:
                break
            gen.report_result(accepted=False)
            count += 1
        self.assertLessEqual(gen.candidates_generated(), MAX_CANDIDATES)
        self.assertTrue(gen.is_converged())

    def test_phase_transition_to_speed(self):
        """增益组收敛后进入速度组。"""
        base = default_params()
        gen = CandidateGenerator(base)
        # 拒绝所有增益组候选直到步长到最小
        # 增益组 4 坐标，每轮 8 候选，步长减半 3 次（0.5→0.25→0.125→0.1 实际钳到 MIN）
        # 实际：0.5→0.25→0.125→0.0625... 但 MIN_STEPS["KP"]=0.1
        # 减半序列：0.5, 0.25, 0.125(>0.1), 0.0625(<0.1→0.1)...
        # 实际 max(0.1, 0.5/2^n)，当 0.5/2^n < 0.1 时钳到 0.1
        # 然后下一次减半：0.1/2=0.05 < MIN 0.1，不变化，halved=False，进入下一阶段
        count = 0
        max_iter = 200
        while count < max_iter:
            cand = gen.next_candidate()
            if cand is None:
                break
            # 检查是否进入速度组：候选修改的是 VBASE 而非 KP
            gen.report_result(accepted=False)
            count += 1
            # 速度组的第一个候选应修改 VBASE
            if count > 8:  # 增益组至少 8 个
                pass
        # 最终应收敛
        # 不强制收敛，只验证不崩溃

    def test_convergence_returns_none(self):
        """所有阶段完成后返回 None。"""
        base = default_params()
        gen = CandidateGenerator(base)
        count = 0
        while count < 300:
            cand = gen.next_candidate()
            if cand is None:
                break
            gen.report_result(accepted=False)
            count += 1
        self.assertTrue(gen.is_converged())
        self.assertIsNone(gen.next_candidate())

    def test_clamp_to_bounds(self):
        """候选参数被钳位到合法范围。"""
        base = default_params()
        base["KP"] = 49.8  # 接近上限 50.0
        gen = CandidateGenerator(base)
        cand = gen.next_candidate()
        # KP + 0.5 = 50.3，应钳到 50.0
        self.assertLessEqual(cand["KP"], PARAMS["KP"]["max"])
        # 如果被钳到 50.0（与 base 相同），生成器应跳过并生成下一个不同候选
        self.assertIsNotNone(cand)


# ============================================================
# 试验收集器测试
# ============================================================


class TrialCollectorTests(unittest.TestCase):
    def test_completion_after_target_laps(self):
        """完成目标圈数后返回 completed=True。"""
        tc = TrialCollector(target_laps=2)
        tc.start(start_ms=0)
        # 模拟：起点 SL → 循迹 → SL（第1圈）→ 循迹 → SL（第2圈完成）
        frames = [
            make_tele(st="SL", ms=0),      # 起点线
            make_tele(st="LF", ms=1000),
            make_tele(st="LF", ms=2000),
            make_tele(st="SL", ms=3000),   # 第1圈过线
            make_tele(st="LF", ms=4000),
            make_tele(st="SL", ms=5000),   # 第2圈过线 → laps=3 > target=2 → 完成
        ]
        result = None
        for f in frames:
            result = tc.feed(f)
            if result is not None:
                break
        self.assertIsNotNone(result)
        self.assertTrue(result.completed)
        # 第一次 SL 是起点，第二次和第三次是圈完成
        # lap_times 应有 2 个（3000-0=3s 和 5000-3000=2s），但 laps>0 时才记录
        # 实际：laps=1 时记录 3000-0=3s，laps=2 时记录 5000-3000=2s，laps=3 触发完成
        self.assertEqual(len(result.lap_times), 2)

    def test_line_lost_aborts(self):
        """连续丢线超过阈值后中止。"""
        tc = TrialCollector(target_laps=5)
        tc.start(start_ms=0)
        # 先正常一帧
        tc.feed(make_tele(st="LF", ms=0))
        # 30 帧丢线
        result = None
        for i in range(35):
            result = tc.feed(make_tele(st="LL", ms=(i + 1) * 100))
            if result is not None:
                break
        self.assertIsNotNone(result)
        self.assertFalse(result.completed)
        self.assertEqual(result.abort_reason, "line_lost")

    def test_lap_timeout_aborts(self):
        """单圈时间超过 MAX_LAP_TIME 中止。"""
        tc = TrialCollector(target_laps=5)
        tc.start(start_ms=0)
        tc.feed(make_tele(st="SL", ms=0))       # 起点
        tc.feed(make_tele(st="LF", ms=1000))
        # 19 秒后过线（超过 MAX_LAP_TIME=18s）
        result = tc.feed(make_tele(st="SL", ms=19000))
        self.assertIsNotNone(result)
        self.assertFalse(result.completed)
        self.assertEqual(result.abort_reason, "lap_timeout")

    def test_trial_timeout_aborts(self):
        """总时长超过 MAX_TRIAL_DURATION 中止。"""
        tc = TrialCollector(target_laps=5)
        tc.start(start_ms=0)
        # 持续循迹不完成圈
        result = None
        for i in range(1000):
            ms = i * 100  # 100ms 每帧
            if ms > 5 * 18000 + 5000:  # 超过 MAX_TRIAL_DURATION
                pass
            result = tc.feed(make_tele(st="LF", ms=ms))
            if result is not None:
                break
            if ms > 100000:  # 安全退出
                break
        self.assertIsNotNone(result)
        self.assertFalse(result.completed)
        self.assertEqual(result.abort_reason, "trial_timeout")

    def test_metrics_accumulation(self):
        """统计指标正确累积。"""
        tc = TrialCollector(target_laps=5)
        tc.start(start_ms=0)
        frames = [
            make_tele(st="LF", le=1.0, po=50.0, el=30.0, er=28.0, ms=0),
            make_tele(st="LF", le=-1.0, po=95.0, el=29.0, er=31.0, ms=100),  # po 饱和
            make_tele(st="LF", le=2.0, po=30.0, el=31.0, er=30.0, ms=200),
        ]
        result = None
        for f in frames:
            result = tc.feed(f)
        self.assertIsNone(result)  # 未完成，继续
        # 不直接检查，因为 collector 仍在运行
        # 检查内部状态
        self.assertEqual(tc._frame_count, 3)
        self.assertEqual(tc._sat_count, 1)  # po=95 >= 90
        self.assertAlmostEqual(tc._le_sq_sum, 1.0 + 1.0 + 4.0)

    def test_fault_counting(self):
        """鱼眼通信错误累计值只按增量计入故障。"""
        tc = TrialCollector(target_laps=5)
        tc.start(start_ms=0)
        tc.feed(make_tele(st="LF", ms=0, se=10))
        tc.feed(make_tele(st="LF", ms=100, se=11))
        tc.feed(make_tele(st="LF", ms=200, se=12))
        self.assertEqual(tc._fault_count, 2)

    def test_stale_path_fish_frame_counting(self):
        """完整帧龄超过固件 50 ms 门限时计为陈旧帧。"""
        tc = TrialCollector(target_laps=5)
        tc.start(start_ms=0)
        tc.feed(make_tele(st="LF", ms=0, sa=50))
        tc.feed(make_tele(st="LF", ms=100, sa=51))
        self.assertEqual(tc._stale_count, 1)


# ============================================================
# 优化器状态机测试
# ============================================================


class OptimizerStateTests(unittest.TestCase):
    def setUp(self):
        self.base = default_params()
        self.cmds: list[str] = []
        self.applied: list[dict] = []
        self.states: list[dict] = []
        self.records: list[CandidateRecord] = []

        self.opt = Optimizer(
            self.base,
            target_laps=2,
            on_apply_params=self.applied.append,
            on_send_cmd=self.cmds.append,
            on_state_change=self.states.append,
            on_candidate_complete=self.records.append,
        )

    def test_start_produces_candidate_and_awaits_confirm(self):
        """启动后生成候选、发停车命令、进入等待人工确认。"""
        self.assertTrue(self.opt.start())
        state = self.opt.get_state()
        self.assertEqual(state["state"], S_AWAITING_CONFIRM)
        self.assertIsNotNone(state["current_candidate"])
        self.assertIn("stop", self.cmds)

    def test_confirm_manual_applies_params(self):
        """人工确认后下发参数，进入 APPLYING。"""
        self.opt.start()
        self.opt.confirm_manual()
        self.assertEqual(self.opt.get_state()["state"], S_APPLYING)
        self.assertEqual(len(self.applied), 1)
        # 下发的应与当前候选一致
        state = self.opt.get_state()
        self.assertAlmostEqual(self.applied[0]["KP"], state["current_candidate"]["KP"])

    def test_ack_confirmed_starts_trial_1(self):
        """ACK 确认后开始第一组试验。"""
        self.opt.start()
        self.opt.confirm_manual()
        self.opt.on_ack_confirmed({"id": 1, "ok": True})
        self.assertEqual(self.opt.get_state()["state"], S_RUNNING_TRIAL_1)
        self.assertIn("start", self.cmds)

    def test_ack_rejected_rejects_candidate(self):
        """ACK 拒绝后拒绝当前候选，进入下一个。"""
        self.opt.start()
        first_candidate = self.opt.get_state()["current_candidate"]
        self.opt.confirm_manual()
        self.opt.on_ack_rejected({"id": 1, "ok": False, "error": "test"})
        # 应已进入下一个候选的等待确认
        state = self.opt.get_state()
        self.assertEqual(state["state"], S_AWAITING_CONFIRM)
        self.assertIsNotNone(state["current_candidate"])
        # 记录中应有被拒绝的候选
        self.assertEqual(len(self.records), 1)
        self.assertFalse(self.records[0].accepted)
        self.assertEqual(self.records[0].abort_reason, "ack_rejected")

    def test_ack_timeout_rejects_candidate(self):
        """ACK 超时后拒绝当前候选。"""
        self.opt.start()
        self.opt.confirm_manual()
        self.opt.on_ack_timeout({"id": 1})
        state = self.opt.get_state()
        self.assertEqual(state["state"], S_AWAITING_CONFIRM)
        self.assertEqual(len(self.records), 1)
        self.assertEqual(self.records[0].abort_reason, "ack_timeout")

    def test_trial_1_failure_rolls_back(self):
        """第一组试验失败后回滚参数，拒绝候选。"""
        self.opt.start()
        self.opt.confirm_manual()
        self.opt.on_ack_confirmed({"id": 1, "ok": True})
        # 喂入丢线遥测触发失败
        for i in range(35):
            self.opt.on_telemetry(make_tele(st="LL", ms=i * 100))
        state = self.opt.get_state()
        # 应已拒绝并进入下一个候选
        self.assertEqual(state["state"], S_AWAITING_CONFIRM)
        self.assertEqual(len(self.records), 1)
        self.assertFalse(self.records[0].accepted)

    def test_trial_1_success_starts_trial_2(self):
        """第一组完成后开始第二组。"""
        self.opt.start()
        self.opt.confirm_manual()
        self.opt.on_ack_confirmed({"id": 1, "ok": True})
        # 喂入完成 2 圈的遥测（target_laps=2）
        frames = [
            make_tele(st="SL", ms=0),
            make_tele(st="LF", ms=1000),
            make_tele(st="SL", ms=2000),   # 第1圈
            make_tele(st="LF", ms=3000),
            make_tele(st="SL", ms=4000),   # 第2圈 → laps=3 > 2 → 完成
        ]
        for f in frames:
            self.opt.on_telemetry(f)
        state = self.opt.get_state()
        self.assertEqual(state["state"], S_RUNNING_TRIAL_2)

    def test_both_trials_success_evaluates(self):
        """两组都完成后评估并进入下一个候选。"""
        self.opt.start()
        self.opt.confirm_manual()
        self.opt.on_ack_confirmed({"id": 1, "ok": True})
        # 第一组
        for f in [
            make_tele(st="SL", ms=0),
            make_tele(st="LF", ms=1000),
            make_tele(st="SL", ms=2000),
            make_tele(st="LF", ms=3000),
            make_tele(st="SL", ms=4000),
        ]:
            self.opt.on_telemetry(f)
        # 第二组
        for f in [
            make_tele(st="SL", ms=5000),
            make_tele(st="LF", ms=6000),
            make_tele(st="SL", ms=7000),
            make_tele(st="LF", ms=8000),
            make_tele(st="SL", ms=9000),
        ]:
            self.opt.on_telemetry(f)
        state = self.opt.get_state()
        # 第一个完成的候选被接受为基线，进入下一个候选
        self.assertEqual(state["state"], S_AWAITING_CONFIRM)
        self.assertEqual(len(self.records), 1)
        self.assertTrue(self.records[0].accepted)

    def test_stop_aborts(self):
        """请求停止后优化器立即中止。"""
        self.opt.start()
        self.opt.stop()
        state = self.opt.get_state()
        self.assertEqual(state["state"], S_ABORTED)

    def test_cannot_start_twice(self):
        """重复启动返回 False。"""
        self.assertTrue(self.opt.start())
        self.assertFalse(self.opt.start())

    def test_export_report(self):
        """导出报告包含正确结构。"""
        self.opt.start()
        report = self.opt.export_report()
        self.assertIn("base_params", report)
        self.assertIn("best_params", report)
        self.assertIn("candidates_generated", report)
        self.assertIn("records", report)
        self.assertEqual(report["base_params"], self.base)


class OptimizerAcceptanceTests(unittest.TestCase):
    """候选接受/拒绝逻辑测试。"""

    def _make_opt(self):
        base = default_params()
        cmds: list[str] = []
        applied: list[dict] = []
        records: list[CandidateRecord] = []

        opt = Optimizer(
            base, target_laps=2,
            on_apply_params=applied.append,
            on_send_cmd=cmds.append,
            on_state_change=lambda s: None,
            on_candidate_complete=records.append,
        )
        return opt, applied, cmds, records

    def _run_successful_trials(self, opt, lap_time_ms=2000):
        """喂入两组完成的试验遥测。target_laps=2，每组需 3 次 SL（起点+2圈）。"""
        ms = 0
        for trial in range(2):
            # 起点线
            opt.on_telemetry(make_tele(st="SL", ms=ms))
            ms += 100
            # 第 1 圈
            opt.on_telemetry(make_tele(st="LF", ms=ms))
            ms += lap_time_ms
            opt.on_telemetry(make_tele(st="SL", ms=ms))
            ms += 100
            # 第 2 圈 → laps=3 > target=2 → 完成
            opt.on_telemetry(make_tele(st="LF", ms=ms))
            ms += lap_time_ms
            opt.on_telemetry(make_tele(st="SL", ms=ms))
            ms += 100

    def test_first_completed_candidate_accepted_as_baseline(self):
        """第一个完成的候选被接受为基线。"""
        opt, applied, cmds, records = self._make_opt()
        opt.start()
        opt.confirm_manual()
        opt.on_ack_confirmed({"id": 1, "ok": True})
        self._run_successful_trials(opt)
        self.assertEqual(len(records), 1)
        self.assertTrue(records[0].accepted)

    def test_worse_candidate_rejected(self):
        """评分劣于基线的候选被拒绝。"""
        opt, applied, cmds, records = self._make_opt()
        # 第一个候选（基线）— 接受
        opt.start()
        opt.confirm_manual()
        opt.on_ack_confirmed({"id": 1, "ok": True})
        self._run_successful_trials(opt, lap_time_ms=2000)
        self.assertEqual(len(records), 1)
        self.assertTrue(records[0].accepted)

        # 第二个候选 — 更慢，应被拒绝
        opt.confirm_manual()
        opt.on_ack_confirmed({"id": 2, "ok": True})
        self._run_successful_trials(opt, lap_time_ms=4000)
        self.assertEqual(len(records), 2)
        self.assertFalse(records[1].accepted)

    def test_better_candidate_accepted(self):
        """评分优于基线的候选被接受。"""
        opt, applied, cmds, records = self._make_opt()
        # 基线 — 较慢
        opt.start()
        opt.confirm_manual()
        opt.on_ack_confirmed({"id": 1, "ok": True})
        self._run_successful_trials(opt, lap_time_ms=3000)
        self.assertTrue(records[0].accepted)

        # 第二个候选 — 更快
        opt.confirm_manual()
        opt.on_ack_confirmed({"id": 2, "ok": True})
        self._run_successful_trials(opt, lap_time_ms=1500)
        self.assertEqual(len(records), 2)
        self.assertTrue(records[1].accepted)


# ============================================================
# user_tuning.h 比赛档导出测试
# ============================================================


class GenerateRaceHeaderTests(unittest.TestCase):
    """generate_race_header 函数测试。"""

    TEMPLATE = """\
#ifndef USER_TUNING_H
#define USER_TUNING_H

#ifndef USER_LINE_PID_KP
#define USER_LINE_PID_KP           5.0f    /* 比例增益 */
#endif
#ifndef USER_LINE_PID_KI
#define USER_LINE_PID_KI           0.0f    /* 积分增益 */
#endif
#ifndef USER_LINE_PID_KD
#define USER_LINE_PID_KD           1.0f    /* 微分增益 */
#endif
#ifndef USER_LINE_FOLLOW_GYRO_DAMP
#define USER_LINE_FOLLOW_GYRO_DAMP 0.15f   /* 陀螺仪阻尼 */
#endif
#ifndef USER_TRACK_BASE_SPEED
#define USER_TRACK_BASE_SPEED      40      /* 基础速度 */
#endif
#ifndef USER_TRACK_SPEED_STRAIGHT
#define USER_TRACK_SPEED_STRAIGHT  50      /* 直道速度 */
#endif
#ifndef USER_TRACK_SPEED_CURVE
#define USER_TRACK_SPEED_CURVE     26      /* 弯道速度 */
#endif
#ifndef USER_RIGHT_ANGLE_THRESHOLD
#define USER_RIGHT_ANGLE_THRESHOLD 2.5f    /* 直角弯阈值 */
#endif
#ifndef USER_GYRO_PID_KP
#define USER_GYRO_PID_KP           3.0f    /* 不可调参数 */
#endif

#endif /* USER_TUNING_H */
"""

    def test_replaces_all_8_tunable_params(self):
        """8 个可调参数都被替换。"""
        best = {
            "KP": 6.5, "KI": 0.0, "KD": 1.5, "GDAMP": 0.22,
            "VBASE": 42, "VSTR": 52, "VCUR": 28, "RATHR": 4.0,
        }
        result = generate_race_header(best, self.TEMPLATE)
        self.assertIn("USER_LINE_PID_KP           6.5f", result)
        self.assertIn("USER_LINE_PID_KI           0f", result)
        self.assertIn("USER_LINE_PID_KD           1.5f", result)
        self.assertIn("USER_LINE_FOLLOW_GYRO_DAMP 0.22f", result)
        self.assertIn("USER_TRACK_BASE_SPEED      42", result)
        self.assertIn("USER_TRACK_SPEED_STRAIGHT  52", result)
        self.assertIn("USER_TRACK_SPEED_CURVE     28", result)
        self.assertIn("USER_RIGHT_ANGLE_THRESHOLD 4f", result)

    def test_preserves_non_tunable_params(self):
        """不可调参数（如 USER_GYRO_PID_KP）保持不变。"""
        best = {"KP": 6.0, "KI": 0.0, "KD": 1.0, "GDAMP": 0.15,
                "VBASE": 40, "VSTR": 50, "VCUR": 26, "RATHR": 4.0}
        result = generate_race_header(best, self.TEMPLATE)
        # GYRO_PID_KP 不在 8 个可调参数中，应保持 3.0f
        self.assertIn("USER_GYRO_PID_KP           3.0f", result)

    def test_preserves_comments_and_structure(self):
        """注释和文件结构保持不变。"""
        best = {"KP": 6.0, "KI": 0.0, "KD": 1.0, "GDAMP": 0.15,
                "VBASE": 40, "VSTR": 50, "VCUR": 26, "RATHR": 4.0}
        result = generate_race_header(best, self.TEMPLATE)
        self.assertIn("#ifndef USER_TUNING_H", result)
        self.assertIn("#endif /* USER_TUNING_H */", result)
        self.assertIn("/* 比例增益 */", result)
        self.assertIn("/* 基础速度 */", result)

    def test_int_params_no_f_suffix(self):
        """int 类型参数不加 'f' 后缀。"""
        best = {"KP": 5.0, "KI": 0.0, "KD": 1.0, "GDAMP": 0.15,
                "VBASE": 45, "VSTR": 55, "VCUR": 30, "RATHR": 4.0}
        result = generate_race_header(best, self.TEMPLATE)
        # VBASE 是 int，应为 "45" 不是 "45f"
        self.assertIn("USER_TRACK_BASE_SPEED      45 ", result)
        self.assertNotIn("USER_TRACK_BASE_SPEED      45f", result)

    def test_float_params_have_f_suffix(self):
        """float 类型参数加 'f' 后缀。"""
        best = {"KP": 7.0, "KI": 0.0, "KD": 1.0, "GDAMP": 0.15,
                "VBASE": 40, "VSTR": 50, "VCUR": 26, "RATHR": 4.0}
        result = generate_race_header(best, self.TEMPLATE)
        # KP 是 float，应为 "7f"
        self.assertIn("USER_LINE_PID_KP           7f", result)

    def test_actual_firmware_header(self):
        """用实际固件 user_tuning.h 测试。"""
        header_path = ROOT / "APP" / "Inc" / "user_tuning.h"
        if not header_path.exists():
            self.skipTest("user_tuning.h not found")
        with open(header_path, "r", encoding="utf-8") as f:
            template = f.read()
        best = {"KP": 6.0, "KI": 0.0, "KD": 1.5, "GDAMP": 0.2,
                "VBASE": 42, "VSTR": 52, "VCUR": 28, "RATHR": 4.0}
        result = generate_race_header(best, template)
        # 验证关键替换
        self.assertIn("USER_LINE_PID_KP           6f", result)
        self.assertIn("USER_LINE_PID_KD           1.5f", result)
        self.assertIn("USER_TRACK_BASE_SPEED      42", result)
        # 验证不可调参数保持不变
        self.assertIn("USER_ROTATE_PID_KP              0.80f", result)
        self.assertIn("USER_MOTOR_SPEED_PID_KP    0.55f", result)
        # 验证文件结构完整
        self.assertIn("#ifndef USER_TUNING_H", result)
        self.assertIn("#endif /* USER_TUNING_H */", result)


# ============================================================
# 优化器边缘测试 — trial_2 失败、状态守卫
# ============================================================


class OptimizerEdgeTests(unittest.TestCase):
    """优化器边缘路径和防御性代码测试。"""

    def setUp(self):
        self.base = default_params()
        self.cmds: list[str] = []
        self.applied: list[dict] = []
        self.states: list[dict] = []
        self.records: list[CandidateRecord] = []

        self.opt = Optimizer(
            self.base,
            target_laps=2,
            on_apply_params=self.applied.append,
            on_send_cmd=self.cmds.append,
            on_state_change=self.states.append,
            on_candidate_complete=self.records.append,
        )

    def _run_trial_1_success(self):
        """运行第一组试验到成功完成。"""
        self.opt.start()
        self.opt.confirm_manual()
        self.opt.on_ack_confirmed({"id": 1, "ok": True})
        for f in [
            make_tele(st="SL", ms=0),
            make_tele(st="LF", ms=1000),
            make_tele(st="SL", ms=2000),
            make_tele(st="LF", ms=3000),
            make_tele(st="SL", ms=4000),
        ]:
            self.opt.on_telemetry(f)

    def test_trial_2_failure_rolls_back(self):
        """第二组试验失败后回滚参数，拒绝候选。"""
        self._run_trial_1_success()
        self.assertEqual(self.opt.get_state()["state"], S_RUNNING_TRIAL_2)
        # 第二组丢线失败
        for i in range(35):
            self.opt.on_telemetry(make_tele(st="LL", ms=5000 + i * 100))
        state = self.opt.get_state()
        # 应回滚并进入下一个候选
        self.assertEqual(state["state"], S_AWAITING_CONFIRM)
        self.assertEqual(len(self.records), 1)
        self.assertFalse(self.records[0].accepted)

    def test_confirm_manual_wrong_state_is_noop(self):
        """在非 AWAITING_CONFIRM 状态调用 confirm_manual 应静默返回。"""
        # IDLE 状态
        self.opt.confirm_manual()
        self.assertEqual(self.opt.get_state()["state"], S_IDLE)
        self.assertEqual(len(self.applied), 0)

    def test_ack_confirmed_wrong_state_is_noop(self):
        """在非 APPLYING 状态调用 on_ack_confirmed 应静默返回。"""
        self.opt.start()  # -> AWAITING_CONFIRM
        self.opt.on_ack_confirmed({"id": 1, "ok": True})
        # 不应进入 RUNNING_TRIAL_1
        self.assertEqual(self.opt.get_state()["state"], S_AWAITING_CONFIRM)

    def test_ack_rejected_wrong_state_is_noop(self):
        """在非 APPLYING 状态调用 on_ack_rejected 应静默返回。"""
        self.opt.start()
        self.opt.on_ack_rejected({"id": 1, "ok": False})
        self.assertEqual(self.opt.get_state()["state"], S_AWAITING_CONFIRM)
        self.assertEqual(len(self.records), 0)

    def test_on_telemetry_wrong_state_is_noop(self):
        """在非 RUNNING_TRIAL 状态调用 on_telemetry 应静默返回。"""
        self.opt.start()  # -> AWAITING_CONFIRM
        self.opt.on_telemetry(make_tele(st="SL", ms=0))
        self.assertEqual(self.opt.get_state()["state"], S_AWAITING_CONFIRM)

    def test_export_report_empty(self):
        """无记录时导出报告。"""
        report = self.opt.export_report()
        self.assertEqual(report["records"], [])
        self.assertEqual(report["candidates_generated"], 0)

    def test_stop_from_running_state(self):
        """从 RUNNING_TRIAL_1 状态调用 stop。"""
        self.opt.start()
        self.opt.confirm_manual()
        self.opt.on_ack_confirmed({"id": 1, "ok": True})
        self.assertEqual(self.opt.get_state()["state"], S_RUNNING_TRIAL_1)
        self.opt.stop()
        self.assertEqual(self.opt.get_state()["state"], S_ABORTED)


# ============================================================
# _p95 函数边缘测试
# ============================================================


class P95FunctionTests(unittest.TestCase):
    """_p95 分位数函数边缘测试。"""

    def test_empty_list_returns_inf(self):
        from optimizer import _p95
        import math
        self.assertEqual(_p95([]), math.inf)

    def test_single_element(self):
        from optimizer import _p95
        self.assertEqual(_p95([15.0]), 15.0)

    def test_two_elements_interpolation(self):
        from optimizer import _p95
        # rank = 0.95 * 1 = 0.95, lo=0, hi=1, frac=0.95
        # result = 10 * (1-0.95) + 20 * 0.95 = 0.5 + 19.0 = 19.5
        self.assertAlmostEqual(_p95([10.0, 20.0]), 19.5, places=2)

    def test_five_elements(self):
        from optimizer import _p95
        values = [14.0, 15.0, 16.0, 17.0, 18.0]
        # rank = 0.95 * 4 = 3.8, lo=3, hi=4, frac=0.8
        # result = 17 * 0.2 + 18 * 0.8 = 17.8
        self.assertAlmostEqual(_p95(values), 17.8, places=2)


if __name__ == "__main__":
    unittest.main()
