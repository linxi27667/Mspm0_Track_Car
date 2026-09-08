"""Run repeatable left/right IMU rotation trials through the host Socket.IO API."""

from __future__ import annotations

import argparse
import json
import math
import queue
import threading
import time
from pathlib import Path

import socketio


def _drain(events: queue.Queue) -> None:
    while True:
        try:
            events.get_nowait()
        except queue.Empty:
            return


def _wait_for(events: queue.Queue, predicate, timeout: float, label: str):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        remaining = deadline - time.monotonic()
        try:
            item = events.get(timeout=max(0.01, remaining))
        except queue.Empty:
            break
        if predicate(item):
            return item
    raise TimeoutError(f"timeout waiting for {label}")


def _same_target(tele: dict, target: float) -> bool:
    value = tele.get("tg")
    return isinstance(value, (int, float)) and math.isclose(
        float(value), target, abs_tol=0.1
    )


def run_trials(
    url: str,
    angle: float,
    repeats: int,
    settle_seconds: float,
    direction: str = "both",
) -> dict:
    sio = socketio.Client()
    request_events: queue.Queue = queue.Queue()
    telemetry_events: queue.Queue = queue.Queue()
    client_events: queue.Queue = queue.Queue()
    latest_lock = threading.Lock()
    latest_telemetry: dict = {}

    @sio.on("request_status")
    def on_request_status(data):
        request_events.put(dict(data or {}))

    @sio.on("telemetry")
    def on_telemetry(data):
        nonlocal latest_telemetry
        snapshot = dict(data or {})
        with latest_lock:
            latest_telemetry = snapshot
        telemetry_events.put(snapshot)

    @sio.on("client_status")
    def on_client_status(data):
        client_events.put(dict(data or {}))

    sio.connect(url, transports=["polling"])
    try:
        _wait_for(
            client_events,
            lambda item: item.get("connected") is True,
            8.0,
            "vehicle connection",
        )
        _wait_for(
            telemetry_events,
            lambda item: item.get("st") == "ID",
            8.0,
            "idle telemetry",
        )

        results = []
        targets = []
        if direction in {"left", "both"}:
            targets.extend([angle] * repeats)
        if direction in {"right", "both"}:
            targets.extend([-angle] * repeats)
        for index, target in enumerate(targets, start=1):
            _drain(request_events)
            _drain(telemetry_events)
            with latest_lock:
                before = dict(latest_telemetry)

            sio.emit("send_rotate", {"deg": target})
            ack = _wait_for(
                request_events,
                lambda item: item.get("state") in {
                    "confirmed", "rejected", "timeout"
                },
                5.0,
                f"trial {index} acknowledgement",
            )
            if ack.get("state") != "confirmed":
                raise RuntimeError(f"trial {index} rejected: {ack}")

            started = _wait_for(
                telemetry_events,
                lambda item: item.get("st") == "RT"
                and item.get("rr") == "running"
                and _same_target(item, target),
                5.0,
                f"trial {index} start",
            )
            samples = [started]
            terminal = None
            trial_timeout = max(9.0, 3.0 + abs(target) * 0.15)
            deadline = time.monotonic() + trial_timeout
            while time.monotonic() < deadline:
                try:
                    tele = telemetry_events.get(timeout=0.5)
                except queue.Empty:
                    continue
                if not _same_target(tele, target):
                    continue
                samples.append(tele)
                if tele.get("st") != "RT" and tele.get("rr") != "running":
                    terminal = tele
                    break
            if terminal is None:
                raise TimeoutError(f"trial {index} did not reach a terminal state")

            stopped = _wait_for(
                telemetry_events,
                lambda item: item.get("st") == "ID"
                and item.get("el") == 0
                and item.get("er") == 0,
                3.0,
                f"trial {index} wheel stop",
            )
            samples.append(stopped)

            start_ms = int(started.get("ms", 0))
            end_ms = int(terminal.get("ms", start_ms))
            err_before = int(before.get("se", started.get("se", 0)))
            err_after = int(stopped.get("se", err_before))
            result = {
                "trial": index,
                "direction": "left" if target > 0 else "right",
                "target_deg": target,
                "actual_deg": float(terminal.get("ta", 0.0)),
                "error_deg": float(terminal.get("te", 0.0)),
                "result": terminal.get("rr", "missing"),
                "duration_ms": (end_ms - start_ms) & 0xFFFFFFFF,
                "peak_rate_deg_s": max(abs(float(item.get("gz", 0.0))) for item in samples),
                "peak_left_speed": max(abs(int(item.get("el", 0))) for item in samples),
                "peak_right_speed": max(abs(int(item.get("er", 0))) for item in samples),
                "peak_left_pwm": max(abs(int(item.get("lp", 0))) for item in samples),
                "peak_right_pwm": max(abs(int(item.get("rp", 0))) for item in samples),
                "path_error_delta": max(0, err_after - err_before),
                "stop_left_speed": int(stopped.get("el", 0)),
                "stop_right_speed": int(stopped.get("er", 0)),
            }
            results.append(result)
            print(json.dumps(result, ensure_ascii=False), flush=True)
            if result["result"] != "complete":
                raise RuntimeError(f"trial {index} failed: {result}")
            time.sleep(settle_seconds)

        return {
            "angle_deg": angle,
            "direction": direction,
            "repeats_per_direction": repeats,
            "trials": results,
        }
    except Exception:
        with latest_lock:
            motion_active = latest_telemetry.get("st") == "RT"
        if motion_active:
            sio.emit("send_cmd", {"cmd": "stop"})
            time.sleep(0.5)
        raise
    finally:
        sio.disconnect()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--url", default="http://127.0.0.1:5000")
    parser.add_argument("--angle", type=float, default=90.0)
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--settle-seconds", type=float, default=0.5)
    parser.add_argument(
        "--direction",
        choices=("left", "right", "both"),
        default="both",
    )
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if not 1.0 <= abs(args.angle) <= 180.0 or args.repeats < 1:
        parser.error("angle must be within 1..180 degrees and repeats must be positive")

    report = run_trials(
        args.url,
        abs(args.angle),
        args.repeats,
        args.settle_seconds,
        args.direction,
    )
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(report, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
