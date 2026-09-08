"""TC1 protocol simulator for the track-car host."""

from __future__ import annotations

import math
import random
import socket
import sys
import threading
import time

from protocol import (
    MAX_FRAME_BYTES,
    PARAMS,
    ProtocolError,
    build_ack,
    decode_frame,
    encode_frame,
    validate_ping,
    validate_request,
)


DEFAULT_PARAMS = {name: info["default"] for name, info in PARAMS.items()}
SCENE_CYCLE = 20.0
TELEMETRY_INTERVAL = 0.01


def simulate(host: str = "127.0.0.1", port: int = 8888):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(1.0)
    try:
        sock.connect((host, port))
    except OSError as exc:
        print(f"[SIM] 无法连接 {host}:{port}: {exc}")
        sock.close()
        return

    print(f"[SIM] 已连接上位机 {host}:{port}")
    print("[SIM] 开始发送 100Hz TC1 遥测，按 Ctrl+C 退出")

    params = dict(DEFAULT_PARAMS)
    state = {"current": "LF"}
    state_lock = threading.Lock()
    send_lock = threading.Lock()
    stop_event = threading.Event()

    def send_message(message) -> bool:
        try:
            frame = encode_frame(message)
            with send_lock:
                sock.sendall(frame)
            return True
        except (OSError, ProtocolError) as exc:
            if not stop_event.is_set():
                print(f"[SIM] 发送失败: {exc}")
            stop_event.set()
            return False

    def send_nack(message: dict, error: str):
        request_id = message.get("id")
        kind = "set" if "set" in message else "cmd" if "cmd" in message else None
        if isinstance(request_id, int) and not isinstance(request_id, bool) and kind:
            send_message(build_ack(request_id, False, kind, error=error[:256]))

    def handle_request(message: dict):
        try:
            request = validate_request(message)
        except ProtocolError as exc:
            print(f"[SIM] 拒绝请求: {exc}")
            send_nack(message, str(exc))
            return

        request_id = request["id"]
        if "set" in request:
            with state_lock:
                motor_names = {"MKP", "MKI", "MKD", "MSL", "MSR"}
                if motor_names & set(request["set"]) and state["current"] not in {"ID", "MT"}:
                    send_message(build_ack(
                        request_id, False, "set",
                        error="motor_tuning_requires_idle",
                    ))
                    return
                params.update(request["set"])
                applied = dict(params)
            print(f"[SIM] 收到参数 id={request_id}: {request['set']}")
            send_message(build_ack(request_id, True, "set", applied=applied))
            return

        cmd = request["cmd"]
        with state_lock:
            if cmd == "start":
                state["current"] = "LF"
            elif cmd == "stop":
                state["current"] = "ID"
        print(f"[SIM] 收到指令 id={request_id}: {cmd}")
        send_message(build_ack(request_id, True, "cmd"))

    def recv_loop():
        buf = bytearray()
        while not stop_event.is_set():
            try:
                data = sock.recv(4096)
            except socket.timeout:
                continue
            except OSError:
                break
            if not data:
                break

            buf.extend(data)
            while True:
                newline = buf.find(b"\n")
                if newline < 0:
                    break
                line = bytes(buf[: newline + 1])
                del buf[: newline + 1]
                if len(line) > MAX_FRAME_BYTES:
                    print(f"[SIM] 丢弃超长帧: {len(line)} bytes")
                    continue
                try:
                    message = decode_frame(line)
                except ProtocolError as exc:
                    print(f"[SIM] 丢弃坏帧: {exc}")
                    continue

                if message.get("t") == "ping":
                    try:
                        send_message(validate_ping(message))
                    except ProtocolError as exc:
                        print(f"[SIM] 丢弃无效 ping: {exc}")
                else:
                    handle_request(message)

            if len(buf) >= MAX_FRAME_BYTES:
                print("[SIM] 未终止帧超过上限，断开连接")
                break
        stop_event.set()

    threading.Thread(target=recv_loop, daemon=True, name="sim-recv").start()

    start_time = time.monotonic()
    next_send = start_time
    prev_error = 0.0
    frame_count = 0

    try:
        while not stop_event.is_set():
            now = time.monotonic()
            if now < next_send:
                time.sleep(min(next_send - now, TELEMETRY_INTERVAL))
                continue
            next_send += TELEMETRY_INTERVAL
            if now - next_send > 0.5:
                next_send = now + TELEMETRY_INTERVAL

            elapsed = now - start_time
            with state_lock:
                p = dict(params)
                requested_state = state["current"]

            error, gyro, track_state = _scene_values(elapsed)
            error = max(-3.5, min(3.5, error))
            raw_turn = p["KP"] * error + p["KD"] * (error - prev_error) * 10
            prev_error = error
            turn = max(-100.0, min(100.0, raw_turn))

            if requested_state == "ID":
                error = 0.0
                gyro = 0.0
                turn = 0.0
                track_state = "ID"
                left_speed = 0
                right_speed = 0
            else:
                base = p["VBASE"]
                left_speed = int(base - turn * 0.5)
                right_speed = int(base + turn * 0.5)

            line = _simulate_line(error, track_state)
            line_mask = sum(bit << index for index, bit in enumerate(line))
            frame_count += 1
            telemetry = {
                "v": 1,
                "t": "tele",
                "seq": frame_count & 0xFFFFFFFF,
                "ms": int(elapsed * 1000) & 0xFFFFFFFF,
                "line": line,
                "lm": line_mask,
                "bc": sum(line),
                "sa": 5,
                "se": 0,
                "bp": 1,
                "gz": round(gyro, 2),
                "el": left_speed,
                "er": right_speed,
                "le": round(error, 2),
                "po": round(turn, 2),
                "st": track_state,
                "kp": p["KP"],
                "ki": p["KI"],
                "kd": p["KD"],
                "gd": p["GDAMP"],
                "vb": p["VBASE"],
                "vs": p["VSTR"],
                "vc": p["VCUR"],
                "rt": p["RATHR"],
                "drop": 0,
            }
            if not send_message(telemetry):
                break

            if frame_count % 100 == 0:
                print(f"[SIM] 已发送 {frame_count} 帧, t={elapsed:.1f}s, error={error:.2f}, st={track_state}")
            if frame_count % 50 == 0:
                send_message({
                    "v": 1,
                    "t": "log",
                    "msg": f"I/track [{int(elapsed * 1000)}] State:{track_state} Error:{error:.2f} KP:{p['KP']:.1f} KD:{p['KD']:.1f}",
                })
    except KeyboardInterrupt:
        print("\n[SIM] 退出")
    finally:
        stop_event.set()
        try:
            sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        sock.close()


def _scene_values(elapsed: float) -> tuple[float, float, str]:
    scene_t = elapsed % SCENE_CYCLE
    if scene_t < 5.0:
        return (
            0.3 * math.sin(elapsed * 2.0) + random.gauss(0, 0.15),
            1.5 * math.sin(elapsed * 5.0) + random.gauss(0, 0.3),
            "LF",
        )
    if scene_t < 10.0:
        error = 2.8 * math.sin((scene_t - 5.0) * 1.2) + random.gauss(0, 0.2)
        state = "LL" if abs(error) > 3.0 and random.random() < 0.08 else "LF"
        return error, 0.5 * math.cos((scene_t - 5.0) * 1.2) + random.gauss(0, 0.2), state
    if scene_t < 11.0:
        return 3.2 + random.gauss(0, 0.1), 5.0, "RA"
    if scene_t < 13.0:
        return -0.5 + random.gauss(0, 0.1), random.gauss(0, 0.2), "LF"
    if scene_t < 15.0:
        return 0.0, 0.3 + random.gauss(0, 0.1), "LL"
    return (
        0.5 * math.sin(elapsed * 1.5) + random.gauss(0, 0.15),
        0.2 * math.sin(elapsed * 1.5) + random.gauss(0, 0.2),
        "LF",
    )


def _simulate_line(error: float, state: str) -> list[int]:
    if state == "LL":
        return [0] * 12
    if state == "RA":
        return [1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0]

    line = [0] * 12
    position = max(0.0, min(11.0, (error + 3.5) * 11.0 / 7.0))
    left = int(math.floor(position))
    right = min(11, left + 1)
    line[left] = 1
    if right != left and position - left > 0.25:
        line[right] = 1
    return line


if __name__ == "__main__":
    target_host = sys.argv[1] if len(sys.argv) > 1 else "127.0.0.1"
    target_port = int(sys.argv[2]) if len(sys.argv) > 2 else 8888
    simulate(target_host, target_port)
