"""Unit tests for the RequestManager (ACK retry, idempotency, timeout)."""

from __future__ import annotations

import sys
import threading
import time
import unittest
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
HOST = ROOT / "host"
if str(HOST) not in sys.path:
    sys.path.insert(0, str(HOST))

from protocol import REQUEST_MAX_RETRIES, REQUEST_TIMEOUT_MS, build_ack  # noqa: E402
from tcp_server import RequestManager  # noqa: E402


class MockTcpServer:
    """Minimal TcpServer mock for RequestManager testing."""

    def __init__(self):
        self.sent_frames: list[str] = []
        self.connected = True
        self._lock = threading.Lock()

    def send(self, json_str: str) -> bool:
        if not self.connected:
            return False
        with self._lock:
            self.sent_frames.append(json_str)
        return True

    def get_sent_count(self) -> int:
        with self._lock:
            return len(self.sent_frames)


class ImmediateAckTcpServer(MockTcpServer):
    """Deliver an ACK synchronously from send(), before send() returns."""

    def __init__(self):
        super().__init__()
        self.manager: RequestManager | None = None

    def send(self, json_str: str) -> bool:
        sent = super().send(json_str)
        if sent and self.manager is not None:
            request_id = int(json_str.rsplit('"id":', 1)[1].rstrip("}"))
            self.manager.on_ack(make_ack_dict(request_id, kind="cmd"))
        return sent


class BlockingTcpServer(MockTcpServer):
    """Hold send() open so another caller can probe the reservation window."""

    def __init__(self):
        super().__init__()
        self.entered = threading.Event()
        self.release = threading.Event()

    def send(self, json_str: str) -> bool:
        self.entered.set()
        self.release.wait(timeout=1.0)
        return super().send(json_str)


def make_ack_dict(request_id: int, ok: bool = True, kind: str = "set",
                  applied: dict[str, Any] | None = None) -> dict:
    """Build a validated ACK dict for testing."""
    ack: dict[str, Any] = {
        "v": 1,
        "t": "ack",
        "id": request_id,
        "ok": ok,
        "kind": kind,
    }
    if applied is not None:
        ack["applied"] = applied
    return ack


class RequestManagerConfirmedTests(unittest.TestCase):
    """ACK confirmed scenarios."""

    def test_send_set_and_ack_confirmed(self):
        """send_set → ACK ok=True → confirmed callback with applied params."""
        tcp = MockTcpServer()
        confirmed_acks: list[dict] = []
        rm = RequestManager(tcp, on_confirmed=confirmed_acks.append)

        self.assertTrue(rm.send_set({"KP": 6.0}))
        self.assertTrue(rm.is_busy())
        self.assertEqual(tcp.get_sent_count(), 1)

        applied = {
            "KP": 6.0, "KI": 0.0, "KD": 1.0, "GDAMP": 0.15,
            "VBASE": 18, "VSTR": 20, "VCUR": 16, "RATHR": 6.0,
            "MKP": 1.2, "MKI": 0.1, "MKD": 0.0,
            "MSL": 0.161, "MSR": 0.164,
        }
        ack = make_ack_dict(1, ok=True, applied=applied)
        rm.on_ack(ack)

        self.assertFalse(rm.is_busy())
        self.assertEqual(len(confirmed_acks), 1)
        self.assertEqual(confirmed_acks[0]["applied"], applied)

    def test_send_cmd_and_ack_confirmed(self):
        """send_cmd → ACK ok=True → confirmed callback."""
        tcp = MockTcpServer()
        confirmed_acks: list[dict] = []
        rm = RequestManager(tcp, on_confirmed=confirmed_acks.append)

        self.assertTrue(rm.send_cmd("start"))
        self.assertTrue(rm.is_busy())

        ack = make_ack_dict(1, ok=True, kind="cmd")
        rm.on_ack(ack)

        self.assertFalse(rm.is_busy())
        self.assertEqual(len(confirmed_acks), 1)

    def test_send_rotate_and_ack_confirmed(self):
        """send_rotate preserves signed degrees and releases the pending request."""
        tcp = MockTcpServer()
        confirmed_acks: list[dict] = []
        rm = RequestManager(tcp, on_confirmed=confirmed_acks.append)

        self.assertTrue(rm.send_rotate(-90.0))
        self.assertEqual(tcp.sent_frames[0], '{"v":1,"t":"cmd","cmd":"rotate","deg":-90.0,"id":1}')

        rm.on_ack(make_ack_dict(1, ok=True, kind="cmd"))
        self.assertFalse(rm.is_busy())
        self.assertEqual(len(confirmed_acks), 1)

    def test_send_motor_test_includes_lifted_confirmation(self):
        tcp = MockTcpServer()
        rm = RequestManager(tcp)

        self.assertTrue(rm.send_motor_test())
        self.assertEqual(
            tcp.sent_frames[0],
            '{"v":1,"t":"cmd","cmd":"motor_test","lifted":true,"id":1}',
        )

    def test_stop_replaces_pending_rotate_request(self):
        """Stop must not be blocked while a rotate ACK is still pending."""
        tcp = MockTcpServer()
        rm = RequestManager(tcp)

        self.assertTrue(rm.send_rotate(90.0))
        self.assertTrue(rm.send_stop_override())
        self.assertEqual(tcp.sent_frames[-1], '{"v":1,"t":"cmd","cmd":"stop","id":2}')
        self.assertEqual(rm.get_pending_id(), 2)

    def test_stale_ack_ignored(self):
        """ACK with non-matching ID is silently ignored."""
        tcp = MockTcpServer()
        confirmed: list[dict] = []
        rm = RequestManager(tcp, on_confirmed=confirmed.append)

        self.assertTrue(rm.send_set({"KP": 6.0}))
        # ACK with wrong ID (stale from previous request)
        stale_ack = make_ack_dict(999, ok=True)
        rm.on_ack(stale_ack)

        self.assertTrue(rm.is_busy())  # still pending
        self.assertEqual(len(confirmed), 0)

        # Correct ACK resolves it
        correct_ack = make_ack_dict(1, ok=True)
        rm.on_ack(correct_ack)
        self.assertFalse(rm.is_busy())
        self.assertEqual(len(confirmed), 1)

    def test_ack_when_not_busy_ignored(self):
        """ACK received when no pending request is ignored."""
        tcp = MockTcpServer()
        confirmed: list[dict] = []
        rm = RequestManager(tcp, on_confirmed=confirmed.append)

        ack = make_ack_dict(1, ok=True)
        rm.on_ack(ack)  # no pending request
        self.assertEqual(len(confirmed), 0)

    def test_immediate_ack_during_send_is_not_lost(self):
        tcp = ImmediateAckTcpServer()
        confirmed: list[dict] = []
        rm = RequestManager(tcp, on_confirmed=confirmed.append)
        tcp.manager = rm

        self.assertTrue(rm.send_cmd("start"))
        self.assertFalse(rm.is_busy())
        self.assertEqual(len(confirmed), 1)


class RequestManagerRejectedTests(unittest.TestCase):
    """ACK rejected scenarios."""

    def test_ack_rejected(self):
        """ACK ok=False → rejected callback."""
        tcp = MockTcpServer()
        rejected: list[dict] = []
        rm = RequestManager(tcp, on_rejected=rejected.append)

        self.assertTrue(rm.send_cmd("start"))
        ack = make_ack_dict(1, ok=False, kind="cmd")
        ack["error"] = "calibration incomplete"
        rm.on_ack(ack)

        self.assertFalse(rm.is_busy())
        self.assertEqual(len(rejected), 1)
        self.assertEqual(rejected[0]["error"], "calibration incomplete")


class RequestManagerBusyTests(unittest.TestCase):
    """Busy rejection scenarios."""

    def test_send_set_while_busy_rejected(self):
        """send_set while pending → returns False."""
        tcp = MockTcpServer()
        rm = RequestManager(tcp)

        self.assertTrue(rm.send_set({"KP": 6.0}))
        self.assertFalse(rm.send_set({"KD": 2.0}))  # busy
        self.assertEqual(tcp.get_sent_count(), 1)  # only first sent

    def test_send_cmd_while_busy_rejected(self):
        """send_cmd while pending → returns False."""
        tcp = MockTcpServer()
        rm = RequestManager(tcp)

        self.assertTrue(rm.send_cmd("start"))
        self.assertFalse(rm.send_cmd("stop"))  # busy
        self.assertEqual(tcp.get_sent_count(), 1)

    def test_send_when_not_connected(self):
        """send when TCP not connected → returns False, not tracked."""
        tcp = MockTcpServer()
        tcp.connected = False
        rm = RequestManager(tcp)

        self.assertFalse(rm.send_set({"KP": 6.0}))
        self.assertFalse(rm.is_busy())

    def test_request_is_reserved_before_transport_send_returns(self):
        tcp = BlockingTcpServer()
        rm = RequestManager(tcp)
        first_result: list[bool] = []
        sender = threading.Thread(target=lambda: first_result.append(rm.send_cmd("start")))
        sender.start()
        self.assertTrue(tcp.entered.wait(timeout=1.0))

        self.assertTrue(rm.is_busy())
        self.assertFalse(rm.send_cmd("stop"))

        tcp.release.set()
        sender.join(timeout=1.0)
        self.assertEqual(first_result, [True])


class RequestManagerTimeoutTests(unittest.TestCase):
    """Timeout and retry scenarios."""

    def test_timeout_after_max_retries(self):
        """No ACK → 3 attempts (original + 2 retries) → timeout callback."""
        tcp = MockTcpServer()
        timeouts: list[dict] = []
        rm = RequestManager(tcp, on_timeout=timeouts.append)

        self.assertTrue(rm.send_set({"KP": 6.0}))
        # Wait for all retries to exhaust: (1 + REQUEST_MAX_RETRIES) * timeout
        max_wait = (1 + REQUEST_MAX_RETRIES) * REQUEST_TIMEOUT_MS / 1000.0 + 0.5
        deadline = time.time() + max_wait
        while rm.is_busy() and time.time() < deadline:
            time.sleep(0.05)

        self.assertFalse(rm.is_busy())
        self.assertEqual(len(timeouts), 1)
        # Original + 2 retries = 3 sends
        self.assertEqual(tcp.get_sent_count(), 1 + REQUEST_MAX_RETRIES)
        self.assertEqual(timeouts[0]["attempts"], 1 + REQUEST_MAX_RETRIES)

    def test_retry_uses_same_id(self):
        """Retry resends the exact same JSON (same request ID)."""
        tcp = MockTcpServer()
        rm = RequestManager(tcp)

        self.assertTrue(rm.send_set({"KP": 6.0}))
        # Wait for first retry
        time.sleep(REQUEST_TIMEOUT_MS / 1000.0 + 0.15)

        # Should have sent at least 2 frames (original + 1 retry)
        self.assertGreaterEqual(tcp.get_sent_count(), 2)
        # All sent frames should be identical (same ID, same params)
        first = tcp.sent_frames[0]
        for frame in tcp.sent_frames[1:]:
            self.assertEqual(frame, first, "retry should resend identical JSON")

    def test_retry_succeeds_on_second_attempt(self):
        """ACK arrives during retry → confirmed, no further retries."""
        tcp = MockTcpServer()
        confirmed: list[dict] = []
        rm = RequestManager(tcp, on_confirmed=confirmed.append)

        self.assertTrue(rm.send_set({"KP": 6.0}))
        # Wait for first retry to fire
        time.sleep(REQUEST_TIMEOUT_MS / 1000.0 + 0.15)
        self.assertGreaterEqual(tcp.get_sent_count(), 2)  # original + retry

        # Now send ACK
        ack = make_ack_dict(1, ok=True)
        rm.on_ack(ack)

        self.assertFalse(rm.is_busy())
        self.assertEqual(len(confirmed), 1)

        # Wait a bit more to ensure no further retries
        time.sleep(REQUEST_TIMEOUT_MS / 1000.0 + 0.15)
        # sent count should not have increased after ACK
        sent_after_ack = tcp.get_sent_count()
        time.sleep(REQUEST_TIMEOUT_MS / 1000.0 + 0.15)
        self.assertEqual(tcp.get_sent_count(), sent_after_ack)

    def test_send_fails_on_retry_triggers_timeout(self):
        """If tcp.send fails during retry, treat as immediate timeout."""
        tcp = MockTcpServer()
        timeouts: list[dict] = []
        rm = RequestManager(tcp, on_timeout=timeouts.append)

        self.assertTrue(rm.send_set({"KP": 6.0}))
        # Disconnect after first send so retry fails
        tcp.connected = False

        # Wait for timeout
        time.sleep(REQUEST_TIMEOUT_MS / 1000.0 + 0.3)

        self.assertFalse(rm.is_busy())
        self.assertEqual(len(timeouts), 1)


class RequestManagerIdAllocationTests(unittest.TestCase):
    """ID allocation and monotonic behavior."""

    def test_ids_are_monotonically_increasing(self):
        """Sequential requests get increasing IDs."""
        tcp = MockTcpServer()
        rm = RequestManager(tcp)

        ids = []
        for _ in range(5):
            self.assertTrue(rm.send_set({"KP": 6.0}))
            ids.append(rm.get_pending_id())
            # Immediately ACK to free the manager
            ack = make_ack_dict(ids[-1], ok=True)
            rm.on_ack(ack)

        self.assertEqual(ids, [1, 2, 3, 4, 5])

    def test_get_pending_id_when_idle(self):
        """get_pending_id returns None when idle."""
        tcp = MockTcpServer()
        rm = RequestManager(tcp)
        self.assertIsNone(rm.get_pending_id())


if __name__ == "__main__":
    unittest.main()
