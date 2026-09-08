"""Local constrained coordinate-search optimizer for track-car tuning.

Replaces "GLM suggests and host pushes" with a deterministic local search:
  Phase A: gain group KP/KD/GDAMP/RATHR (KI locked at 0)
  Phase B: speed group VBASE/VSTR/VCUR

Per candidate: stop -> manual confirm -> MCU ACK -> two independent 5-lap trials.
Any line-loss/fault/timeout/stale-frame immediately rolls back.
Candidates ranked by: zero-fault, P95 lap time, lateral error, saturation, wheel diff.

Initial steps: KP 0.5, KD 0.2, GDAMP 0.02, RATHR 1, speed 2.
Halve on no improvement down to: 0.1/0.05/0.01/1/1.
Max 24 candidates per session.
"""

from __future__ import annotations

import logging
import math
import threading
from dataclasses import dataclass, field
from typing import Any, Callable

from param_schema import (
    OPTIMIZER_GROUP_GAINS,
    OPTIMIZER_GROUP_SPEED,
)
from protocol import clamp

logger = logging.getLogger("optimizer")


# ============================================================
# 试验指标与评分
# ============================================================

# 转向输出饱和阈值（|po| 超过此值视为饱和）
SATURATION_THRESHOLD = 90.0
# 丢线连续帧数阈值（@10Hz 遥测，30 帧 = 3 秒）
LINE_LOST_FRAMES = 30
# 单圈最大时间（秒），超过判超时
MAX_LAP_TIME = 18.0
# 试验最大总时长（秒），5 圈上限
MAX_TRIAL_DURATION = 5 * MAX_LAP_TIME + 5.0


@dataclass
class TrialMetrics:
    """单组 N 圈试验的指标。"""

    completed: bool = False              # 是否完成目标圈数（未丢线/故障/超时）
    laps_completed: int = 0              # 实际完成圈数
    lap_times: list[float] = field(default_factory=list)  # 每圈时间（秒）
    p95_lap_time: float = math.inf       # P95 圈时（秒），未完成时为 inf
    lateral_error_rms: float = 0.0       # 横向误差 RMS（le 的均方根）
    saturation_rate: float = 0.0         # 转向饱和率（|po|>=阈值的帧比例）
    wheel_speed_diff: float = 0.0        # 左右速度误差均值（|el-er| 的均值）
    fault_count: int = 0                 # 故障帧数
    line_lost_count: int = 0             # 丢线帧数
    stale_frame_count: int = 0           # 陈旧帧数
    duration: float = 0.0                # 试验持续时间（秒）
    abort_reason: str = ""               # 未完成时的中止原因


def candidate_score(metrics: TrialMetrics) -> tuple:
    """返回排序键，越小越好。

    按计划依次排序：零故障 > P95 圈时 > 横向误差 > 饱和率 > 左右速度误差。
    未完成的候选（completed=False）排在所有完成候选之后。
    """
    return (
        0 if metrics.completed else 1,
        metrics.p95_lap_time,
        metrics.lateral_error_rms,
        metrics.saturation_rate,
        metrics.wheel_speed_diff,
    )


def _p95(values: list[float]) -> float:
    """计算 P95 分位数（线性插值）。空列表返回 inf。"""
    if not values:
        return math.inf
    s = sorted(values)
    n = len(s)
    if n == 1:
        return s[0]
    rank = 0.95 * (n - 1)
    lo = int(rank)
    hi = min(lo + 1, n - 1)
    frac = rank - lo
    return s[lo] * (1 - frac) + s[hi] * frac


# ============================================================
# 试验收集器 — 从遥测流提取指标
# ============================================================


class TrialCollector:
    """从遥测流收集单组试验指标。

    用法：
        tc = TrialCollector(target_laps=5)
        tc.start(start_ms=0)
        for tele in telemetry_stream:
            result = tc.feed(tele)
            if result is not None:
                break  # 试验结束（完成或中止）
    """

    def __init__(self, target_laps: int = 5):
        self.target_laps = target_laps
        self._reset()

    def _reset(self):
        self._active = False
        self._start_ms: int | None = None
        self._prev_st: str = ""
        self._lap_start_ms: int | None = None
        self._laps: int = 0
        self._lap_times: list[float] = []
        self._le_sq_sum = 0.0
        self._sat_count = 0
        self._wheel_diff_sum = 0.0
        self._frame_count = 0
        self._fault_count = 0
        self._line_lost_count = 0
        self._stale_count = 0
        self._consecutive_ll = 0
        self._abort_reason = ""
        self._last_sensor_errors: int | None = None

    def start(self, start_ms: int = 0):
        """开始收集。start_ms 为试验开始的 MCU 时间戳（毫秒）。"""
        self._reset()
        self._active = True
        self._start_ms = start_ms
        self._lap_start_ms = start_ms

    def feed(self, tele: dict[str, Any]) -> TrialMetrics | None:
        """喂入一帧遥测。返回 None 表示试验继续，返回 TrialMetrics 表示结束。

        遥测字段：st, le, po, el, er, ms, seq, sa, se 等。
        """
        if not self._active:
            return None

        ms = tele.get("ms", 0)
        st = tele.get("st", "")
        le = float(tele.get("le", 0.0))
        po = float(tele.get("po", 0.0))
        el = float(tele.get("el", 0.0))
        er = float(tele.get("er", 0.0))

        self._frame_count += 1

        # 累积统计
        self._le_sq_sum += le * le
        if abs(po) >= SATURATION_THRESHOLD:
            self._sat_count += 1
        self._wheel_diff_sum += abs(el - er)

        # se 是鱼眼通信错误累计值；只统计本次试验期间的新增错误。
        sensor_errors = int(tele.get("se", 0))
        if self._last_sensor_errors is not None and sensor_errors > self._last_sensor_errors:
            self._fault_count += sensor_errors - self._last_sensor_errors
        self._last_sensor_errors = sensor_errors

        # 12路完整帧正常约10ms；超过50ms与固件Safety门限一致。
        if tele.get("sa", 0) > 50:
            self._stale_count += 1

        # 丢线检测
        if st == "LL":
            self._line_lost_count += 1
            self._consecutive_ll += 1
            if self._consecutive_ll >= LINE_LOST_FRAMES:
                return self._finish(abort="line_lost")
        else:
            self._consecutive_ll = 0

        # 圈计数：检测非 SL -> SL 转换
        if st == "SL" and self._prev_st != "SL":
            if self._lap_start_ms is not None and self._laps > 0:
                # 完成一圈
                lap_time = (ms - self._lap_start_ms) / 1000.0
                if lap_time > MAX_LAP_TIME:
                    return self._finish(abort="lap_timeout")
                self._lap_times.append(lap_time)
            self._lap_start_ms = ms
            self._laps += 1
            if self._laps > self.target_laps:
                # 超过目标圈数（第 target_laps+1 次过线 = 完成 target_laps 圈）
                return self._finish(completed=True)
        self._prev_st = st

        # 总时长超限
        if self._start_ms is not None and ms > self._start_ms:
            duration = (ms - self._start_ms) / 1000.0
            if duration > MAX_TRIAL_DURATION:
                return self._finish(abort="trial_timeout")

        return None

    def _finish(self, completed: bool = False, abort: str = "") -> TrialMetrics:
        self._active = False
        n = self._frame_count
        return TrialMetrics(
            completed=completed,
            laps_completed=max(0, self._laps - 1),  # 第一次 SL 是起点
            lap_times=list(self._lap_times),
            p95_lap_time=_p95(self._lap_times),
            lateral_error_rms=math.sqrt(self._le_sq_sum / n) if n > 0 else 0.0,
            saturation_rate=self._sat_count / n if n > 0 else 0.0,
            wheel_speed_diff=self._wheel_diff_sum / n if n > 0 else 0.0,
            fault_count=self._fault_count,
            line_lost_count=self._line_lost_count,
            stale_frame_count=self._stale_count,
            duration=0.0,  # 由 Optimizer 从 ms 计算
            abort_reason=abort,
        )


# ============================================================
# 候选生成器 — 坐标搜索
# ============================================================

# 初始步长
INITIAL_STEPS: dict[str, float] = {
    "KP": 0.5, "KD": 0.2, "GDAMP": 0.02, "RATHR": 1.0,
    "VBASE": 2, "VSTR": 2, "VCUR": 2,
}
# 最小步长（低于此值该坐标收敛）
MIN_STEPS: dict[str, float] = {
    "KP": 0.1, "KD": 0.05, "GDAMP": 0.01, "RATHR": 1.0,
    "VBASE": 1, "VSTR": 1, "VCUR": 1,
}
MAX_CANDIDATES = 24

_SEARCH_GROUPS = [OPTIMIZER_GROUP_GAINS, OPTIMIZER_GROUP_SPEED]


class CandidateGenerator:
    """坐标搜索候选生成器。

    两阶段：先增益组（KP/KD/GDAMP/RATHR），后速度组（VBASE/VSTR/VCUR）。
    每个坐标方向 +-step 生成候选，钳位到合法范围。
    一轮（当前组所有方向）无改进 -> 步长减半。
    步长低于最小值 -> 当前组结束，进入下一组。
    所有组结束 -> 收敛。
    单次会话最多 MAX_CANDIDATES 个候选。
    """

    def __init__(self, base_params: dict[str, float]):
        self._base = {k: float(v) for k, v in base_params.items()}
        self._best = dict(self._base)
        self._phase = 0  # 0=gains, 1=speed, 2=converged
        self._steps: dict[str, float] = {}
        self._coord_index = 0
        self._direction = 0  # 0=+, 1=-
        self._round_had_improvement = False
        self._candidates_generated = 0
        self._pending: dict[str, float] | None = None  # 待评估的候选
        self._init_phase()

    def _init_phase(self):
        group = _SEARCH_GROUPS[self._phase]
        self._steps = {k: INITIAL_STEPS[k] for k in group}
        self._coord_index = 0
        self._direction = 0
        self._round_had_improvement = False

    def _current_group(self) -> list[str]:
        if self._phase >= len(_SEARCH_GROUPS):
            return []
        return _SEARCH_GROUPS[self._phase]

    def best_params(self) -> dict[str, float]:
        return dict(self._best)

    def candidates_generated(self) -> int:
        return self._candidates_generated

    def is_converged(self) -> bool:
        return self._phase >= len(_SEARCH_GROUPS)

    def next_candidate(self) -> dict[str, float] | None:
        """返回下一个候选参数（完整 8 参数快照），或 None 表示收敛。"""
        if self._pending is not None:
            return dict(self._pending)

        if self.is_converged():
            return None
        if self._candidates_generated >= MAX_CANDIDATES:
            self._phase = len(_SEARCH_GROUPS)
            return None

        group = self._current_group()
        while self._phase < len(_SEARCH_GROUPS):
            if self._coord_index >= len(group):
                # 本轮结束
                if self._round_had_improvement:
                    # 有改进，重置坐标索引，保持步长继续搜索
                    self._coord_index = 0
                    self._round_had_improvement = False
                    continue
                else:
                    # 无改进，步长减半
                    halved = False
                    for k in group:
                        if self._steps[k] > MIN_STEPS[k]:
                            self._steps[k] = max(MIN_STEPS[k], self._steps[k] / 2)
                            halved = True
                    if not halved:
                        # 所有步长已到最小，进入下一阶段
                        self._phase += 1
                        if self._phase < len(_SEARCH_GROUPS):
                            self._init_phase()
                            group = self._current_group()
                            continue
                        return None
                    else:
                        self._coord_index = 0
                        self._round_had_improvement = False
                        continue

            coord = group[self._coord_index]
            step = self._steps[coord]
            if step < MIN_STEPS[coord]:
                # 该坐标已收敛，跳过
                self._coord_index += 1
                continue

            # 生成候选
            if self._direction == 0:
                delta = +step
                self._direction = 1
            else:
                delta = -step
                self._direction = 0
                self._coord_index += 1

            candidate = dict(self._best)
            try:
                candidate[coord] = clamp(coord, self._best[coord] + delta)
            except (ValueError, KeyError):
                # 钳位失败，跳过该方向
                continue

            # 候选与当前最佳相同（已在边界），跳过
            if abs(candidate[coord] - self._best[coord]) < 1e-9:
                continue

            self._pending = candidate
            self._candidates_generated += 1
            return dict(candidate)

        return None

    def report_result(self, accepted: bool, metrics: TrialMetrics | None = None):
        """报告上一个 next_candidate() 返回的候选的试验结果。

        accepted=True 表示候选被接受（有改进），更新 best。
        accepted=False 表示候选被拒绝（无改进或失败），保持 best。
        """
        if self._pending is None:
            return
        if accepted:
            self._best = dict(self._pending)
            self._round_had_improvement = True
        self._pending = None


# ============================================================
# 优化器状态机
# ============================================================

# 状态枚举
S_IDLE = "idle"
S_PROPOSING = "proposing"                 # 生成候选
S_AWAITING_CONFIRM = "awaiting_confirm"   # 停车后等待人工确认
S_APPLYING = "applying"                   # 下发参数，等 ACK
S_RUNNING_TRIAL_1 = "running_trial_1"     # 第一组 5 圈
S_RUNNING_TRIAL_2 = "running_trial_2"     # 第二组 5 圈
S_ROLLING_BACK = "rolling_back"           # 回滚参数
S_CONVERGED = "converged"                 # 收敛
S_ABORTED = "aborted"                     # 中止


@dataclass
class CandidateRecord:
    """单个候选的完整记录（用于导出报告）。"""

    index: int                                  # 候选序号（1-based）
    params: dict[str, float]                    # 候选参数
    accepted: bool = False                      # 是否被接受
    trial_1: TrialMetrics | None = None         # 第一组试验
    trial_2: TrialMetrics | None = None         # 第二组试验
    score: tuple | None = None                  # 综合评分（取两组最佳）
    abort_reason: str = ""                      # 中止原因


class Optimizer:
    """本地约束优化器状态机。

    通过回调与外部交互（下发参数/命令、状态推送），本身不依赖 socketio/tcp。
    试验指标通过 on_telemetry 流式收集，自动检测完成/失败。

    回调：
      on_apply_params(params: dict) -> bool   下发参数（返回是否成功入队）
      on_send_cmd(cmd: str) -> bool           下发命令（返回是否成功入队）
      on_state_change(state: dict)            状态变更通知
      on_candidate_complete(record: CandidateRecord)  候选完成通知
    """

    def __init__(
        self,
        base_params: dict[str, float],
        target_laps: int = 5,
        on_apply_params: Callable[[dict[str, float]], bool] | None = None,
        on_send_cmd: Callable[[str], bool] | None = None,
        on_state_change: Callable[[dict[str, Any]], None] | None = None,
        on_candidate_complete: Callable[[CandidateRecord], None] | None = None,
    ):
        self._base = {k: float(v) for k, v in base_params.items()}
        self._gen = CandidateGenerator(self._base)
        self._target_laps = target_laps
        self._on_apply = on_apply_params or (lambda p: True)
        self._on_cmd = on_send_cmd or (lambda c: True)
        self._on_state = on_state_change or (lambda s: None)
        self._on_candidate = on_candidate_complete or (lambda r: None)

        self._lock = threading.Lock()
        self._state = S_IDLE
        self._current_candidate: dict[str, float] | None = None
        self._current_record: CandidateRecord | None = None
        self._trial_collector: TrialCollector | None = None
        self._trial_1: TrialMetrics | None = None
        self._trial_2: TrialMetrics | None = None
        self._trial_start_ms: int = 0
        self._records: list[CandidateRecord] = []
        self._stop_requested = False
        self._best_score: tuple | None = None  # 当前最佳候选的评分

    # ---------- 公开 API ----------

    def start(self) -> bool:
        """启动优化器。返回是否成功启动。"""
        with self._lock:
            if self._state != S_IDLE:
                return False
            self._stop_requested = False
            self._records.clear()
            self._best_score = None
            self._gen = CandidateGenerator(self._base)
            self._set_state(S_PROPOSING)
            self._propose_next()
            return True

    def stop(self):
        """请求停止优化器 — 立即中止并停车。"""
        with self._lock:
            if self._state in (S_IDLE, S_CONVERGED, S_ABORTED):
                return
            self._stop_requested = True
            self._on_cmd("stop")
            self._set_state(S_ABORTED)
            self._emit_state(msg="优化器已中止")

    def on_ack_confirmed(self, ack: dict[str, Any]):
        """参数下发 ACK 确认 — 开始第一组试验。"""
        with self._lock:
            if self._state != S_APPLYING:
                return
            self._trial_1 = None
            self._trial_2 = None
            self._trial_collector = TrialCollector(self._target_laps)
            self._trial_start_ms = 0
            self._set_state(S_RUNNING_TRIAL_1)
            self._on_cmd("start")
            self._emit_state(msg="参数已确认，开始第一组试验")

    def on_ack_rejected(self, ack: dict[str, Any]):
        """参数下发 ACK 拒绝。"""
        with self._lock:
            if self._state != S_APPLYING:
                return
            self._emit_state(msg=f"参数被拒绝: {ack.get('error', '?')}")
            self._reject_current("ack_rejected")

    def on_ack_timeout(self, snapshot: dict[str, Any]):
        """参数下发 ACK 超时。"""
        with self._lock:
            if self._state != S_APPLYING:
                return
            self._emit_state(msg="参数下发超时")
            self._reject_current("ack_timeout")

    def confirm_manual(self):
        """人工物理确认通过 — 下发候选参数，等待 MCU ACK。"""
        with self._lock:
            if self._state != S_AWAITING_CONFIRM or self._current_candidate is None:
                return
            self._set_state(S_APPLYING)
            self._on_apply(self._current_candidate)
            self._emit_state(msg="人工确认通过，下发参数等待 ACK")

    def on_telemetry(self, tele: dict[str, Any]):
        """喂入遥测帧。"""
        with self._lock:
            if self._state not in (S_RUNNING_TRIAL_1, S_RUNNING_TRIAL_2):
                return
            if self._trial_collector is None or not self._trial_collector._active:
                # 记录试验起始 ms
                self._trial_start_ms = int(tele.get("ms", 0))
                self._trial_collector.start(self._trial_start_ms)

            result = self._trial_collector.feed(tele)
            if result is not None:
                self._on_trial_finished(result)

    def get_state(self) -> dict[str, Any]:
        with self._lock:
            return {
                "state": self._state,
                "candidates_generated": self._gen.candidates_generated(),
                "max_candidates": MAX_CANDIDATES,
                "best_params": self._gen.best_params(),
                "current_candidate": self._current_candidate,
                "records": len(self._records),
                "target_laps": self._target_laps,
                "stop_requested": self._stop_requested,
            }

    def export_report(self) -> dict[str, Any]:
        """导出候选报告。"""
        with self._lock:
            return {
                "base_params": dict(self._base),
                "best_params": self._gen.best_params(),
                "candidates_generated": self._gen.candidates_generated(),
                "max_candidates": MAX_CANDIDATES,
                "converged": self._gen.is_converged(),
                "records": [
                    {
                        "index": r.index,
                        "params": r.params,
                        "accepted": r.accepted,
                        "trial_1": _metrics_to_dict(r.trial_1),
                        "trial_2": _metrics_to_dict(r.trial_2),
                        "abort_reason": r.abort_reason,
                    }
                    for r in self._records
                ],
            }

    # ---------- 内部状态转移 ----------

    def _propose_next(self):
        """生成下一个候选：发停车命令，进入人工确认等待。"""
        if self._stop_requested:
            self._set_state(S_ABORTED)
            return
        candidate = self._gen.next_candidate()
        if candidate is None:
            self._set_state(S_CONVERGED)
            self._emit_state(msg="搜索收敛")
            return
        self._current_candidate = candidate
        idx = self._gen.candidates_generated()
        self._current_record = CandidateRecord(index=idx, params=dict(candidate))
        self._on_cmd("stop")
        self._set_state(S_AWAITING_CONFIRM)
        self._emit_state(msg=f"候选 #{idx}: {candidate}，已停车，等待人工确认")

    def _reject_current(self, reason: str):
        """拒绝当前候选，记录并进入下一个。"""
        if self._current_record is not None:
            self._current_record.abort_reason = reason
            self._records.append(self._current_record)
            self._on_candidate(self._current_record)
        self._gen.report_result(accepted=False)
        self._current_candidate = None
        self._current_record = None
        self._set_state(S_PROPOSING)
        self._propose_next()

    def _on_trial_finished(self, metrics: TrialMetrics):
        """单组试验结束。"""
        # 计算 duration
        metrics.duration = 0.0  # TrialCollector 不计算，保留
        if self._state == S_RUNNING_TRIAL_1:
            self._trial_1 = metrics
            if not metrics.completed:
                # 第一组失败，立即回滚
                self._set_state(S_ROLLING_BACK)
                self._on_cmd("stop")
                self._apply_params(self._gen.best_params())
                self._emit_state(msg=f"第一组失败({metrics.abort_reason})，回滚")
                self._reject_current(metrics.abort_reason or "trial_1_failed")
                return
            # 第一组成功，停车后开始第二组
            self._on_cmd("stop")
            self._trial_collector = TrialCollector(self._target_laps)
            self._trial_start_ms = 0
            self._set_state(S_RUNNING_TRIAL_2)
            self._on_cmd("start")
            self._emit_state(msg="第一组通过，开始第二组试验")
            return
        if self._state == S_RUNNING_TRIAL_2:
            self._trial_2 = metrics
            if not metrics.completed:
                self._set_state(S_ROLLING_BACK)
                self._on_cmd("stop")
                self._apply_params(self._gen.best_params())
                self._emit_state(msg=f"第二组失败({metrics.abort_reason})，回滚")
                self._reject_current(metrics.abort_reason or "trial_2_failed")
                return
            # 两组都成功，评估
            self._evaluate()
            return

    def _evaluate(self):
        """评估候选：取两组中较差评分与当前最佳比较，决定接受或拒绝。"""
        assert self._trial_1 is not None and self._trial_2 is not None
        # 取两组中较差的作为候选评分（保守策略，确保两组都达标）
        score = max(candidate_score(self._trial_1), candidate_score(self._trial_2))
        # 停车
        self._on_cmd("stop")

        # 接受策略：第一个完成的候选接受为基线；后续仅当评分严格优于 best_score 时接受
        if self._best_score is None:
            accepted = True
        else:
            accepted = score < self._best_score

        if self._current_record is not None:
            self._current_record.trial_1 = self._trial_1
            self._current_record.trial_2 = self._trial_2
            self._current_record.score = score
            self._current_record.accepted = accepted
            self._records.append(self._current_record)
            self._on_candidate(self._current_record)

        if accepted:
            self._best_score = score

        self._gen.report_result(accepted=accepted)
        self._current_candidate = None
        self._current_record = None
        self._trial_1 = None
        self._trial_2 = None
        self._set_state(S_PROPOSING)
        self._propose_next()

    def _apply_params(self, params: dict[str, float]) -> bool:
        """下发参数。"""
        return self._on_apply(params)

    def _set_state(self, new_state: str):
        self._state = new_state
        self._emit_state()

    def _emit_state(self, msg: str = ""):
        self._on_state({
            "state": self._state,
            "candidate": self._current_candidate,
            "candidates_generated": self._gen.candidates_generated(),
            "max_candidates": MAX_CANDIDATES,
            "best_params": self._gen.best_params(),
            "msg": msg,
        })


def _metrics_to_dict(m: TrialMetrics | None) -> dict[str, Any] | None:
    if m is None:
        return None
    return {
        "completed": m.completed,
        "laps_completed": m.laps_completed,
        "lap_times": m.lap_times,
        "p95_lap_time": m.p95_lap_time if m.p95_lap_time != math.inf else None,
        "lateral_error_rms": m.lateral_error_rms,
        "saturation_rate": m.saturation_rate,
        "wheel_speed_diff": m.wheel_speed_diff,
        "fault_count": m.fault_count,
        "line_lost_count": m.line_lost_count,
        "stale_frame_count": m.stale_frame_count,
        "abort_reason": m.abort_reason,
    }


def generate_race_header(best_params: dict[str, float], template_text: str) -> str:
    """用优化器最佳参数生成 user_tuning.h 比赛档内容。

    只替换 8 个远程可调参数的 #define 值，保留所有其他行和注释。
    float 参数加 'f' 后缀，int 参数不加。

    Args:
        best_params: 优化器导出的最佳参数，如 {"KP": 6.0, ...}
        template_text: 现有 user_tuning.h 的完整文本
    Returns:
        更新后的 user_tuning.h 文本
    """
    import re
    from param_schema import PARAM_SCHEMA

    # 构建 macro -> (value, kind) 映射
    macro_map: dict[str, tuple[float, str]] = {}
    for p in PARAM_SCHEMA:
        name = p["name"]
        if name in best_params:
            macro_map[p["macro"]] = (float(best_params[name]), p["kind"])

    lines = template_text.splitlines(keepends=True)
    pattern = re.compile(
        r'^(\s*#define\s+)(USER_[A-Z0-9_]+)(\s+)([0-9]+\.?[0-9]*)f?(\s*(?:/\*.*)?)$'
    )
    out: list[str] = []
    replaced: set[str] = set()
    for line in lines:
        match = pattern.match(line)
        if match:
            macro = match.group(2)
            if macro in macro_map:
                value, kind = macro_map[macro]
                prefix = match.group(1)
                spacing = match.group(3)
                comment = match.group(5)
                if kind == "int":
                    new_val = str(int(value))
                else:
                    new_val = f"{value:g}f"
                out.append(f"{prefix}{macro}{spacing}{new_val}{comment}\n")
                replaced.add(macro)
                continue
        out.append(line)

    missing = set(macro_map) - replaced
    if missing:
        logger.warning("generate_race_header: macros not found in template: %s", missing)

    return "".join(out)
