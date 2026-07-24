"""ESP-01S 台架注入测试 (t-inject).

通过真实 loopback socket 向 TcpServer 注入 TC1 帧，模拟 ESP-01S 经 TCP 传输
的各类场景：正常遥测流、分片帧、粘包帧、CRC 错误、垃圾重同步、ACK 往返、
日志投递、ping 心跳、超大帧丢弃、连接生命周期。

与 test_tcp_server.py 的区别：后者只覆盖少量分发场景；本文件聚焦 ESP 台架
注入路径的端到端验证，包括 RequestManager 下行→对端 ACK→确认的完整往返。
"""

from __future__ import annotations

import socket
import sys
import threading
import time
import unittest
import zlib
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
HOST = ROOT / "host"
if str(HOST) not in sys.path:
    sys.path.insert(0, str(HOST))

from protocol import (  # noqa: E402
    MAX_FRAME_BYTES,
    MAX_JSON_BYTES,
    PARAMS,
    PROTOCOL_VERSION,
    build_ack,
    build_cmd,
    build_set,
    decode_frame,
    encode_frame,
)
from tcp_server import TcpServer  # noqa: E402
from tcp_server import RequestManager  # noqa: E402


# ============================================================
# 辅助构造函数
# ============================================================

def make_telemetry(seq=1, **updates):
    """构造一帧合法遥测对象。"""
    obj = {
        "v": PROTOCOL_VERSION,
        "t": "tele",
        "seq": seq,
        "ms": seq * 100,
        "line": [0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0],
        "lm": 0x060,
        "bc": 2,
        "sa": 5,
        "se": 0,
        "bp": 1,
        "gz": 0.3,
        "el": 40,
        "er": 41,
        "le": 0.2,
        "po": 1.5,
        "st": "LF",
        "kp": 5.0,
        "kd": 1.0,
    }
    obj.update(updates)
    return obj


def full_applied(**updates):
    values = {name: info["default"] for name, info in PARAMS.items()}
    values.update(updates)
    return values


def make_ack_frame(request_id, ok=True, kind="set", applied=None, error=None):
    """构造一帧 ACK 的 wire bytes。"""
    s = build_ack(request_id, ok, kind, applied=applied, error=error)
    return encode_frame(s)


def make_raw_frame(json_str):
    """直接用 JSON 字符串构造 wire bytes（绕过 validate_params）。"""
    json_bytes = json_str.encode("utf-8") if isinstance(json_str, str) else json_str
    crc = zlib.crc32(json_bytes) & 0xFFFFFFFF
    return (
        b"TC1 " + str(len(json_bytes)).encode("ascii")
        + b" " + f"{crc:08X}".encode("ascii")
        + b" " + json_bytes + b"\n"
    )


def make_bad_crc_frame(json_str):
    """构造 CRC 错误的帧。"""
    json_bytes = json_str.encode("utf-8") if isinstance(json_str, str) else json_str
    return (
        b"TC1 " + str(len(json_bytes)).encode("ascii")
        + b" DEADBEEF " + json_bytes + b"\n"
    )


# ============================================================
# 测试基类：管理 TcpServer + loopback socket
# ============================================================

class EspInjectBase(unittest.TestCase):
    """ESP 注入测试基类，提供 socket 生命周期管理。"""

    def setUp(self):
        self.telemetry = []
        self.acks = []
        self.logs = []
        self.connected = threading.Event()
        self.disconnected = threading.Event()
        self.server = TcpServer(
            port=0,
            on_telemetry=self._on_telemetry,
            on_ack=self._on_ack,
            on_log=self._on_log,
            on_connect=lambda _addr: self.connected.set(),
            on_disconnect=self.disconnected.set,
        )
        self.server.start()
        self.client = socket.create_connection(
            ("127.0.0.1", self.server.port), timeout=2
        )
        self.client.settimeout(2)
        self.assertTrue(self.connected.wait(2), "server did not accept client")
        self._recv_buf = bytearray()
        self._lock = threading.Lock()

    def tearDown(self):
        try:
            self.client.close()
        except OSError:
            pass
        self.server.stop()

    def _on_telemetry(self, tele):
        with self._lock:
            self.telemetry.append(tele)

    def _on_ack(self, ack):
        with self._lock:
            self.acks.append(ack)

    def _on_log(self, msg):
        with self._lock:
            self.logs.append(msg)

    def wait_for(self, predicate, timeout=2):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return True
            time.sleep(0.01)
        return predicate()

    def send_bytes(self, data: bytes):
        """模拟 ESP-01S 向 host 发送原始字节。"""
        self.client.sendall(data)

    def send_frame(self, obj):
        """编码并发送一帧合法 TC1 对象。"""
        self.send_bytes(encode_frame(obj))

    def recv_frame(self, timeout=2):
        """接收一帧 host 下发的 TC1 帧。"""
        deadline = time.monotonic() + timeout
        while b"\n" not in self._recv_buf:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None
            self.client.settimeout(remaining)
            try:
                data = self.client.recv(4096)
            except socket.timeout:
                return None
            if not data:
                return None
            self._recv_buf.extend(data)
        newline = self._recv_buf.index(b"\n")
        line = bytes(self._recv_buf[: newline + 1])
        del self._recv_buf[: newline + 1]
        return decode_frame(line)


# ============================================================
# 正常遥测流注入
# ============================================================

class TelemetryStreamTests(EspInjectBase):
    """模拟 ESP-01S 以 10Hz 持续发送遥测帧。"""

    def test_single_telemetry_dispatched(self):
        self.send_frame(make_telemetry(seq=1))
        self.assertTrue(self.wait_for(lambda: len(self.telemetry) >= 1))
        self.assertEqual(self.telemetry[0]["seq"], 1)
        self.assertEqual(self.telemetry[0]["st"], "LF")

    def test_ten_hz_burst_ten_frames(self):
        for i in range(1, 11):
            self.send_frame(make_telemetry(seq=i))
        self.assertTrue(self.wait_for(lambda: len(self.telemetry) >= 10))
        seqs = [t["seq"] for t in self.telemetry[:10]]
        self.assertEqual(seqs, list(range(1, 11)))

    def test_telemetry_seq_wraparound_16bit(self):
        """seq 超过 0xFFFF 后回绕（MCU 侧 uint16_t）。"""
        self.send_frame(make_telemetry(seq=0xFFFF))
        self.send_frame(make_telemetry(seq=0))
        self.assertTrue(self.wait_for(lambda: len(self.telemetry) >= 2))
        self.assertEqual(self.telemetry[0]["seq"], 0xFFFF)
        self.assertEqual(self.telemetry[1]["seq"], 0)

    def test_telemetry_all_states_delivered(self):
        """所有状态缩写都能正确投递。"""
        states = ["ID", "LF", "RA", "LL", "SL", "CR", "FN"]
        for i, st in enumerate(states):
            self.send_frame(make_telemetry(seq=i, st=st))
        self.assertTrue(self.wait_for(lambda: len(self.telemetry) >= len(states)))
        got = [t["st"] for t in self.telemetry[: len(states)]]
        self.assertEqual(got, states)


# ============================================================
# 分片帧注入（模拟 ESP-01S 缓慢发送 / TCP 分段）
# ============================================================

class FragmentationTests(EspInjectBase):
    """模拟 ESP-01S 逐字节或分批发送帧。"""

    def test_byte_by_byte_injection(self):
        """逐字节注入一帧遥测，host 应完整重组。"""
        frame = encode_frame(make_telemetry(seq=42))
        for b in frame:
            self.client.sendall(bytes([b]))
            time.sleep(0.001)
        self.assertTrue(self.wait_for(lambda: len(self.telemetry) >= 1))
        self.assertEqual(self.telemetry[0]["seq"], 42)

    def test_split_at_arbitrary_boundary(self):
        """在帧中间任意位置断开，host 应正确重组。"""
        frame = encode_frame(make_telemetry(seq=7, le=-1.5))
        mid = len(frame) // 2
        self.client.sendall(frame[:mid])
        time.sleep(0.05)
        self.client.sendall(frame[mid:])
        self.assertTrue(self.wait_for(lambda: len(self.telemetry) >= 1))
        self.assertAlmostEqual(self.telemetry[0]["le"], -1.5)

    def test_split_into_three_parts(self):
        """分成三段发送。"""
        frame = encode_frame(make_telemetry(seq=99))
        n = len(frame)
        p1, p2, p3 = frame[: n // 3], frame[n // 3: 2 * n // 3], frame[2 * n // 3:]
        for part in (p1, p2, p3):
            self.client.sendall(part)
            time.sleep(0.02)
        self.assertTrue(self.wait_for(lambda: len(self.telemetry) >= 1))
        self.assertEqual(self.telemetry[0]["seq"], 99)


# ============================================================
# 粘包帧注入（模拟 TCP 合并多个帧到一个包）
# ============================================================

class CoalescedFrameTests(EspInjectBase):
    """模拟多个 TC1 帧合并在一个 TCP 包中发送。"""

    def test_two_telemetry_coalesced(self):
        f1 = encode_frame(make_telemetry(seq=1))
        f2 = encode_frame(make_telemetry(seq=2))
        self.client.sendall(f1 + f2)
        self.assertTrue(self.wait_for(lambda: len(self.telemetry) >= 2))
        self.assertEqual(self.telemetry[0]["seq"], 1)
        self.assertEqual(self.telemetry[1]["seq"], 2)

    def test_mixed_type_coalesced(self):
        """遥测 + 日志 + 遥测 合并在一个包。"""
        t1 = encode_frame(make_telemetry(seq=1))
        log_frame = encode_frame({"v": PROTOCOL_VERSION, "t": "log", "msg": "hello esp"})
        t2 = encode_frame(make_telemetry(seq=2))
        self.client.sendall(t1 + log_frame + t2)
        self.assertTrue(self.wait_for(lambda: len(self.telemetry) >= 2))
        self.assertTrue(self.wait_for(lambda: len(self.logs) >= 1))
        self.assertEqual(self.logs[0], "hello esp")

    def test_five_frames_coalesced(self):
        frames = b"".join(
            encode_frame(make_telemetry(seq=i)) for i in range(5)
        )
        self.client.sendall(frames)
        self.assertTrue(self.wait_for(lambda: len(self.telemetry) >= 5))


# ============================================================
# CRC 错误与垃圾重同步
# ============================================================

class ErrorAndResyncTests(EspInjectBase):
    """模拟链路噪声导致的 CRC 错误和垃圾数据。"""

    def test_bad_crc_frame_ignored(self):
        """CRC 错误的帧被丢弃，不触发回调。"""
        bad = make_bad_crc_frame('{"v":1,"t":"tele","seq":1,"ms":0,"line":[0,0,0,0,0,1,1,0,0,0,0,0],"lm":96,"bc":2,"sa":5,"se":0,"bp":1,"gz":0,"el":0,"er":0,"le":0,"po":0,"st":"LF"}')
        good = encode_frame(make_telemetry(seq=2))
        self.client.sendall(bad + good)
        self.assertTrue(self.wait_for(lambda: len(self.telemetry) >= 1))
        # 只有 good 帧被投递
        self.assertEqual(len(self.telemetry), 1)
        self.assertEqual(self.telemetry[0]["seq"], 2)

    def test_resync_after_garbage_line(self):
        """垃圾行后 host 应重同步并解析后续合法帧。"""
        good = encode_frame(make_telemetry(seq=3))
        self.client.sendall(b"GARBAGE DATA WITHOUT TC1 PREFIX\n" + good)
        self.assertTrue(self.wait_for(lambda: len(self.telemetry) >= 1))
        self.assertEqual(self.telemetry[0]["seq"], 3)

    def test_resync_after_partial_magic(self):
        """部分 magic 行后重同步（host 按行分隔，partial magic 需以换行结尾）。"""
        good = encode_frame(make_telemetry(seq=4))
        # "TC\n" 作为独立行被丢弃，后续 good 帧正常解析
        self.client.sendall(b"TC\n" + good)
        self.assertTrue(self.wait_for(lambda: len(self.telemetry) >= 1))
        self.assertEqual(self.telemetry[0]["seq"], 4)

    def test_bad_then_good_then_bad_sequence(self):
        """坏帧-好帧-坏帧交替，只有好帧投递。"""
        bad = make_bad_crc_frame('{"v":1,"t":"tele","seq":0,"ms":0,"line":[0,0,0,0,0,1,1,0,0,0,0,0],"lm":96,"bc":2,"sa":5,"se":0,"bp":1,"gz":0,"el":0,"er":0,"le":0,"po":0,"st":"ID"}')
        good1 = encode_frame(make_telemetry(seq=10))
        good2 = encode_frame(make_telemetry(seq=11))
        self.client.sendall(bad + good1 + bad + good2)
        self.assertTrue(self.wait_for(lambda: len(self.telemetry) >= 2))
        seqs = [t["seq"] for t in self.telemetry]
        self.assertIn(10, seqs)
        self.assertIn(11, seqs)
        self.assertEqual(len(self.telemetry), 2)


# ============================================================
# 超大帧保护
# ============================================================

class OversizedFrameTests(EspInjectBase):
    """模拟超过 MAX_FRAME_BYTES 的异常帧。"""

    def test_oversized_terminated_line_discarded(self):
        """超大的已终止行被丢弃，后续帧正常。"""
        # 构造一个 JSON 超过 MAX_JSON_BYTES 的帧
        big_json = '{"v":1,"t":"log","msg":"' + "A" * (MAX_JSON_BYTES + 50) + '"}'
        big_frame = make_raw_frame(big_json)
        self.assertGreater(len(big_frame), MAX_FRAME_BYTES)
        good = encode_frame(make_telemetry(seq=5))
        self.client.sendall(big_frame + good)
        self.assertTrue(self.wait_for(lambda: len(self.telemetry) >= 1))
        self.assertEqual(self.telemetry[0]["seq"], 5)


# ============================================================
# 日志帧注入
# ============================================================

class LogFrameTests(EspInjectBase):
    """模拟 ESP-01S 发送日志帧。"""

    def test_single_log_delivered(self):
        self.send_frame({"v": PROTOCOL_VERSION, "t": "log", "msg": "boot ok"})
        self.assertTrue(self.wait_for(lambda: len(self.logs) >= 1))
        self.assertEqual(self.logs[0], "boot ok")

    def test_multiple_logs_in_order(self):
        msgs = ["line1", "line2", "line3"]
        for m in msgs:
            self.send_frame({"v": PROTOCOL_VERSION, "t": "log", "msg": m})
        self.assertTrue(self.wait_for(lambda: len(self.logs) >= 3))
        self.assertEqual(self.logs[:3], msgs)

    def test_long_log_message_delivered(self):
        """接近 JSON 上限的日志消息正常投递（JSON 总长 ≤ 256 字节）。"""
        # {"v":1,"t":"log","msg":""} 开销约 26 字节，msg 最多约 230 字节
        msg = "X" * 200
        self.send_frame({"v": PROTOCOL_VERSION, "t": "log", "msg": msg})
        self.assertTrue(self.wait_for(lambda: len(self.logs) >= 1))
        self.assertEqual(self.logs[0], msg)

    def test_log_interleaved_with_telemetry(self):
        """日志和遥测交错发送，各自正确投递。"""
        self.send_frame(make_telemetry(seq=1))
        self.send_frame({"v": PROTOCOL_VERSION, "t": "log", "msg": "mid"})
        self.send_frame(make_telemetry(seq=2))
        self.assertTrue(self.wait_for(lambda: len(self.telemetry) >= 2 and len(self.logs) >= 1))
        self.assertEqual(self.logs[0], "mid")


# ============================================================
# Ping 心跳注入
# ============================================================

class PingFrameTests(EspInjectBase):
    """模拟 ESP-01S / MCU 的 ping 心跳帧。"""

    def test_ping_does_not_trigger_callbacks(self):
        """ping 帧被验证但不触发任何回调。"""
        self.send_frame({"v": PROTOCOL_VERSION, "t": "ping", "ms": 1000})
        time.sleep(0.3)
        self.assertEqual(len(self.telemetry), 0)
        self.assertEqual(len(self.acks), 0)
        self.assertEqual(len(self.logs), 0)

    def test_ping_followed_by_telemetry(self):
        """ping 后紧跟遥测，遥测正常投递。"""
        self.send_frame({"v": PROTOCOL_VERSION, "t": "ping", "ms": 2000})
        self.send_frame(make_telemetry(seq=1))
        self.assertTrue(self.wait_for(lambda: len(self.telemetry) >= 1))
        self.assertEqual(len(self.acks), 0)

    def test_rapid_pings_no_interference(self):
        """连续 10 个 ping 不影响后续遥测。"""
        for i in range(10):
            self.send_frame({"v": PROTOCOL_VERSION, "t": "ping", "ms": i})
        self.send_frame(make_telemetry(seq=100))
        self.assertTrue(self.wait_for(lambda: len(self.telemetry) >= 1))
        self.assertEqual(self.telemetry[0]["seq"], 100)


# ============================================================
# ACK 往返：RequestManager 下行 → 对端 ACK → 确认
# ============================================================

class AckRoundTripTests(EspInjectBase):
    """模拟 host 下发 set/cmd，ESP-01S 回复 ACK 的完整往返。"""

    def setUp(self):
        super().setUp()
        self.confirmed = []
        self.rejected = []
        self.timed_out = []
        self.rm = RequestManager(
            self.server,
            on_confirmed=self.confirmed.append,
            on_rejected=self.rejected.append,
            on_timeout=self.timed_out.append,
        )
        self.server.on_ack = self.rm.on_ack

    def test_set_then_ack_confirmed(self):
        """host 下发 set → ESP 回复 ack(ok=True) → on_confirmed 触发。"""
        self.assertTrue(self.rm.send_set({"KP": 6.0}))
        # ESP 侧接收 set 帧
        req = self.recv_frame()
        self.assertIsNotNone(req)
        self.assertIn("set", req)
        self.assertAlmostEqual(req["set"]["KP"], 6.0)
        # ESP 回复 ACK
        ack_frame = make_ack_frame(
            req["id"], ok=True, kind="set",
            applied=full_applied(KP=6.0),
        )
        self.send_bytes(ack_frame)
        self.assertTrue(self.wait_for(lambda: len(self.confirmed) >= 1))
        self.assertTrue(self.confirmed[0]["ok"])
        self.assertFalse(self.rm.is_busy())

    def test_cmd_then_ack_confirmed(self):
        """host 下发 cmd=start → ESP 回复 ack(ok=True) → on_confirmed 触发。"""
        self.assertTrue(self.rm.send_cmd("start"))
        req = self.recv_frame()
        self.assertIsNotNone(req)
        self.assertEqual(req["cmd"], "start")
        self.send_bytes(make_ack_frame(req["id"], ok=True, kind="cmd"))
        self.assertTrue(self.wait_for(lambda: len(self.confirmed) >= 1))
        self.assertFalse(self.rm.is_busy())

    def test_set_then_nack_rejected(self):
        """host 下发 set → ESP 回复 ack(ok=False) → on_rejected 触发。"""
        self.assertTrue(self.rm.send_set({"KP": 99.0}))
        req = self.recv_frame()
        self.assertIsNotNone(req)
        self.send_bytes(make_ack_frame(
            req["id"], ok=False, kind="set", error="out of range"
        ))
        self.assertTrue(self.wait_for(lambda: len(self.rejected) >= 1))
        self.assertFalse(self.rejected[0]["ok"])
        self.assertEqual(self.rejected[0]["error"], "out of range")
        self.assertFalse(self.rm.is_busy())

    def test_stale_ack_ignored(self):
        """对端回复不匹配的 id 时，RequestManager 忽略。"""
        self.assertTrue(self.rm.send_set({"KD": 2.0}))
        req = self.recv_frame()
        self.assertIsNotNone(req)
        # 发送一个错误 id 的 ACK
        wrong_id = req["id"] + 999
        self.send_bytes(make_ack_frame(
            wrong_id, ok=True, kind="set", applied=full_applied(KD=2.0)))
        time.sleep(0.5)
        # 不应确认
        self.assertEqual(len(self.confirmed), 0)
        self.assertTrue(self.rm.is_busy())

    def test_set_with_telemetry_interleaved(self):
        """下行 set 期间，ESP 同时发送遥测，两者互不干扰。"""
        # 先发几帧遥测
        for i in range(3):
            self.send_frame(make_telemetry(seq=i))
        # 下行 set
        self.assertTrue(self.rm.send_set({"VBASE": 45}))
        req = self.recv_frame()
        self.assertIsNotNone(req)
        # 继续发遥测
        for i in range(3, 6):
            self.send_frame(make_telemetry(seq=i))
        # 回复 ACK
        self.send_bytes(make_ack_frame(
            req["id"], ok=True, kind="set",
            applied=full_applied(VBASE=45),
        ))
        self.assertTrue(self.wait_for(lambda: len(self.confirmed) >= 1))
        self.assertTrue(self.wait_for(lambda: len(self.telemetry) >= 6))


# ============================================================
# 连接生命周期
# ============================================================

class ConnectionLifecycleTests(EspInjectBase):
    """模拟 ESP-01S 断连和重连。"""

    def test_disconnect_detected(self):
        """客户端关闭后 on_disconnect 触发。"""
        self.client.close()
        self.assertTrue(self.wait_for(lambda: self.disconnected.is_set(), timeout=3))

    def test_reconnect_replaces_previous(self):
        """新连接替换旧连接，on_disconnect + on_connect 依次触发。"""
        old_disconnected = threading.Event()
        new_connected = threading.Event()
        self.server.on_disconnect = lambda: old_disconnected.set()
        self.server.on_connect = lambda _addr: new_connected.set()

        # 关闭旧连接
        self.client.close()
        self.assertTrue(self.wait_for(lambda: old_disconnected.is_set(), timeout=3))

        # 新连接
        new_client = socket.create_connection(
            ("127.0.0.1", self.server.port), timeout=2
        )
        new_client.settimeout(2)
        self.assertTrue(new_connected.wait(2))
        # 新连接发送遥测正常
        frame = encode_frame(make_telemetry(seq=1))
        new_client.sendall(frame)
        self.assertTrue(self.wait_for(lambda: len(self.telemetry) >= 1))
        new_client.close()


# ============================================================
# 高频压力测试
# ============================================================

class StressTests(EspInjectBase):
    """模拟 ESP-01S 高频发送场景。"""

    def test_100_hz_50_frames(self):
        """模拟 100Hz 遥测持续 0.5s（50 帧）。"""
        frames = b"".join(
            encode_frame(make_telemetry(seq=i)) for i in range(50)
        )
        self.client.sendall(frames)
        self.assertTrue(self.wait_for(lambda: len(self.telemetry) >= 50, timeout=3))
        self.assertEqual(len(self.telemetry), 50)

    def test_continuous_stream_with_interleaved_logs(self):
        """连续遥测流中穿插日志帧。"""
        parts = []
        for i in range(20):
            parts.append(encode_frame(make_telemetry(seq=i)))
            if i % 5 == 4:
                parts.append(encode_frame({"v": PROTOCOL_VERSION, "t": "log",
                                           "msg": f"checkpoint {i}"}))
        self.client.sendall(b"".join(parts))
        self.assertTrue(self.wait_for(lambda: len(self.telemetry) >= 20, timeout=3))
        self.assertTrue(self.wait_for(lambda: len(self.logs) >= 4, timeout=3))
        self.assertIn("checkpoint 4", self.logs)

    def test_param_mirror_fields_delivered(self):
        """遥测中包含参数镜像字段（kp/ki/kd/gd/vb/vs/vc/rt），全部投递。"""
        self.send_frame(make_telemetry(
            seq=1, kp=7.5, ki=0.1, kd=2.0, gd=0.3,
            vb=45, vs=55, vc=28, rt=6.0,
        ))
        self.assertTrue(self.wait_for(lambda: len(self.telemetry) >= 1))
        t = self.telemetry[0]
        self.assertAlmostEqual(t["kp"], 7.5)
        self.assertAlmostEqual(t["ki"], 0.1)
        self.assertAlmostEqual(t["kd"], 2.0)
        self.assertAlmostEqual(t["gd"], 0.3)
        self.assertEqual(t["vb"], 45)
        self.assertEqual(t["vs"], 55)
        self.assertEqual(t["vc"], 28)
        self.assertAlmostEqual(t["rt"], 6.0)

    def test_extended_telemetry_fields_delivered(self):
        """鱼眼健康与链路扩展字段正常投递。"""
        self.send_frame(make_telemetry(seq=1, drop=1, sa=9, se=3, bp=0))
        self.assertTrue(self.wait_for(lambda: len(self.telemetry) >= 1))
        t = self.telemetry[0]
        self.assertEqual(t["bc"], 2)
        self.assertEqual(t["drop"], 1)
        self.assertEqual(t["sa"], 9)
        self.assertEqual(t["se"], 3)
        self.assertEqual(t["bp"], 0)


if __name__ == "__main__":
    unittest.main()
