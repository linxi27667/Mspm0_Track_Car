import json
import socket
import sys
import threading
import time
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
HOST = ROOT / "host"
if str(HOST) not in sys.path:
    sys.path.insert(0, str(HOST))

from protocol import (  # noqa: E402
    MAX_FRAME_BYTES,
    PARAMS,
    build_ack,
    build_auth,
    build_cmd,
    decode_frame,
    encode_frame,
)
from tcp_server import CLIENT_IO_TIMEOUT, TcpServer  # noqa: E402


def telemetry(seq=1, **updates):
    obj = {
        "v": 1,
        "t": "tele",
        "seq": seq,
        "ms": seq * 10,
        "line": [0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0],
        "lm": 0x060,
        "bc": 2,
        "sa": 5,
        "se": 0,
        "bp": 1,
        "gz": 0.2,
        "el": 40,
        "er": 41,
        "le": 0.1,
        "po": 2.0,
        "st": "LF",
        "kp": 5.0,
        "kd": 1.0,
    }
    obj.update(updates)
    return obj


class TcpServerTests(unittest.TestCase):
    def setUp(self):
        self.telemetry = []
        self.acks = []
        self.logs = []
        self.connected = threading.Event()
        self.disconnected = threading.Event()
        self.server = TcpServer(
            port=0,
            on_telemetry=self.telemetry.append,
            on_ack=self.acks.append,
            on_log=self.logs.append,
            on_connect=lambda _addr: self.connected.set(),
            on_disconnect=self.disconnected.set,
        )
        self.server.start()
        self.client = socket.create_connection(("127.0.0.1", self.server.port), timeout=2)
        self.client.settimeout(2)
        self.assertTrue(self.connected.wait(2), "server did not accept loopback client")
        self.recv_buffer = bytearray()

    def tearDown(self):
        try:
            self.client.close()
        except OSError:
            pass
        self.server.stop()

    def wait_for(self, predicate, timeout=2):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return True
            time.sleep(0.01)
        return predicate()

    def recv_frame(self):
        while b"\n" not in self.recv_buffer:
            self.recv_buffer.extend(self.client.recv(4096))
        newline = self.recv_buffer.index(b"\n")
        line = bytes(self.recv_buffer[: newline + 1])
        del self.recv_buffer[: newline + 1]
        return decode_frame(line)

    def test_fragmented_and_coalesced_frames_dispatch_by_type(self):
        first = encode_frame(telemetry(1))
        for offset in range(0, len(first), 3):
            self.client.sendall(first[offset : offset + 3])

        applied = {name: info["default"] for name, info in PARAMS.items()}
        ack = encode_frame(build_ack(7, True, "set", applied=applied))
        second = encode_frame(telemetry(2))
        self.client.sendall(ack + second)

        self.assertTrue(self.wait_for(lambda: len(self.telemetry) == 2 and len(self.acks) == 1))
        self.assertEqual([item["seq"] for item in self.telemetry], [1, 2])
        self.assertEqual(self.acks[0]["id"], 7)
        self.assertEqual(self.server.get_frame_count(), 2)

    def test_bad_crc_raw_json_and_invalid_schema_never_reach_callbacks(self):
        valid = encode_frame(telemetry(3))
        parts = valid.rstrip(b"\n").split(b" ", 3)
        bad_crc = parts[0] + b" " + parts[1] + b" 00000000 " + parts[3] + b"\n"
        invalid_schema = encode_frame(telemetry(4, line=[]))
        raw_json = json.dumps(telemetry(5)).encode("utf-8") + b"\n"
        self.client.sendall(bad_crc + invalid_schema + raw_json + valid)

        self.assertTrue(self.wait_for(lambda: len(self.telemetry) == 1))
        time.sleep(0.05)
        self.assertEqual([item["seq"] for item in self.telemetry], [3])
        self.assertEqual(self.server.get_frame_count(), 1)

    def test_send_frames_explicit_and_automatic_request_ids(self):
        self.assertTrue(self.server.send(build_cmd("stop", request_id=42)))
        self.assertEqual(self.recv_frame(), {"v": 1, "t": "cmd", "cmd": "stop", "id": 42})
        self.assertEqual(self.server.get_last_request_id(), 42)

        self.assertTrue(self.server.send('{"cmd":"start"}'))
        automatic = self.recv_frame()
        self.assertEqual(automatic["v"], 1)
        self.assertEqual(automatic["t"], "cmd")
        self.assertEqual(automatic["cmd"], "start")
        self.assertIsInstance(automatic["id"], int)
        self.assertGreater(automatic["id"], 0)

    def test_send_auth_frame(self):
        self.assertTrue(self.server.send(build_auth("track_car_2025", session="session-123")))
        self.assertEqual(
            self.recv_frame(),
            {"v": 1, "t": "auth", "token": "track_car_2025", "session": "session-123"},
        )

    def test_log_callback_and_keepalive_are_enabled(self):
        self.client.sendall(encode_frame({"v": 1, "t": "log", "msg": "I/test [1] ok"}))
        self.assertTrue(self.wait_for(lambda: self.logs == ["I/test [1] ok"]))
        with self.server._lock:
            accepted = self.server._client
            self.assertIsNotNone(accepted)
            self.assertEqual(accepted.getsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE), 1)
            self.assertEqual(accepted.gettimeout(), CLIENT_IO_TIMEOUT)

    def test_oversized_unterminated_frame_disconnects_client(self):
        self.client.sendall(b"X" * MAX_FRAME_BYTES)
        self.assertTrue(self.disconnected.wait(2))
        self.assertTrue(self.wait_for(lambda: not self.server.is_connected()))


if __name__ == "__main__":
    unittest.main()
