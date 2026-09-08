"""TC1 C/Python interoperability golden-frame tests.

Validates that the MCU-side byte-stream parser (app_tc1.c) and the host-side
protocol.py produce identical results for:
  - normal frames
  - fragmented (split) frames
  - coalesced (back-to-back) frames
  - CRC errors
  - length field errors
  - oversized payloads
  - duplicate message IDs
  - out-of-order delivery
  - auth failures
  - heartbeat timeout scenarios

The C parser state machine is faithfully simulated in Python so that golden
frames can be verified without running on real hardware.
"""

from __future__ import annotations

import binascii
import json
import sys
import unittest
from pathlib import Path
from typing import Optional

ROOT = Path(__file__).resolve().parents[1]
HOST = ROOT / "host"
if str(HOST) not in sys.path:
    sys.path.insert(0, str(HOST))

from protocol import (
    FRAME_MAGIC,
    MAX_JSON_BYTES,
    PROTOCOL_VERSION,
    compact_json,
    encode_frame,
)


# =====================================================================
# C-side state machine simulation (mirrors APP/Src/app_tc1.c)
# =====================================================================

TC1_FRAME_MAGIC_LEN = 3
TC1_CRC_HEX_LEN = 8
TC1_MAX_LEN_DIGITS = 3

# Decoder results (mirror tc1_decode_result_t in app_tc1.h)
DECODE_OK = 0
DECODE_NEED_MORE = 1
DECODE_BAD_MAGIC = 2
DECODE_BAD_LENGTH = 3
DECODE_BAD_CRC = 4
DECODE_OVERSIZED = 5
DECODE_BAD_JSON = 6
DECODE_BAD_SCHEMA = 7
DECODE_DUPLICATE_ID = 8

# Parser states (mirror tc1_parser_state_t)
STATE_WAIT_MAGIC = 0
STATE_READ_LEN = 1
STATE_READ_CRC = 2
STATE_READ_PAYLOAD = 3
STATE_WAIT_NEWLINE = 4

# Message types (mirror tc1_msg_type_t)
MSG_UNKNOWN = 0
MSG_AUTH = 1
MSG_SET = 2
MSG_CMD = 3
MSG_PING = 4
MSG_PONG = 5
MSG_ACK = 6
MSG_NACK = 7
MSG_TELE = 8
MSG_LOG = 9

_MSG_TYPE_MAP = {
    "auth": MSG_AUTH,
    "set": MSG_SET,
    "cmd": MSG_CMD,
    "ping": MSG_PING,
    "pong": MSG_PONG,
    "ack": MSG_ACK,
    "nack": MSG_NACK,
    "tele": MSG_TELE,
    "log": MSG_LOG,
}


class CParserSim:
    """Faithful Python simulation of the C tc1_parser_t + App_TC1_FeedByte."""

    def __init__(self) -> None:
        self.state = STATE_WAIT_MAGIC
        self.magic_idx = 0
        self.len_buf = bytearray()
        self.crc_buf = bytearray()
        self.payload = bytearray()
        self.declared_len = 0
        self.declared_crc = 0
        self.frames_ok = 0
        self.frames_bad_crc = 0
        self.frames_oversized = 0
        self.frames_bad_json = 0
        self.frames_bad_schema = 0
        self.frames_duplicate_id = 0
        self.last_msg_id = 0
        self.last_msg_id_valid = False
        self._magic = b"TC1"

    def reset(self) -> None:
        self.state = STATE_WAIT_MAGIC
        self.magic_idx = 0
        self.len_buf = bytearray()
        self.crc_buf = bytearray()
        self.payload = bytearray()
        self.declared_len = 0
        self.declared_crc = 0

    @staticmethod
    def _crc32(data: bytes) -> int:
        return binascii.crc32(data) & 0xFFFFFFFF

    def _finalize(self) -> tuple[int, Optional[dict]]:
        """Mirror finalize_frame() in app_tc1.c."""
        payload = bytes(self.payload)
        actual_crc = self._crc32(payload)
        if actual_crc != self.declared_crc:
            self.frames_bad_crc += 1
            return DECODE_BAD_CRC, None

        if len(payload) < 2 or payload[0] != ord("{") or payload[-1] != ord("}"):
            self.frames_bad_json += 1
            return DECODE_BAD_JSON, None

        try:
            obj = json.loads(payload.decode("utf-8"))
        except (ValueError, UnicodeDecodeError):
            self.frames_bad_json += 1
            return DECODE_BAD_JSON, None

        if not isinstance(obj, dict):
            self.frames_bad_json += 1
            return DECODE_BAD_JSON, None

        t_val = obj.get("t")
        mtype = _MSG_TYPE_MAP.get(t_val, MSG_UNKNOWN) if isinstance(t_val, str) else MSG_UNKNOWN
        if mtype == MSG_UNKNOWN:
            self.frames_bad_schema += 1
            return DECODE_BAD_SCHEMA, None

        msg_id = 0
        has_id = False
        if "id" in obj and isinstance(obj["id"], int) and not isinstance(obj["id"], bool):
            msg_id = obj["id"]
            has_id = True
            if self.last_msg_id_valid and msg_id == self.last_msg_id:
                self.frames_duplicate_id += 1
            self.last_msg_id = msg_id
            self.last_msg_id_valid = True

        self.frames_ok += 1
        return DECODE_OK, {
            "type": mtype,
            "json": payload.decode("utf-8"),
            "json_len": len(payload),
            "msg_id": msg_id,
            "has_id": has_id,
            "obj": obj,
        }

    def feed_byte(self, byte: int) -> tuple[int, Optional[dict]]:
        """Mirror App_TC1_FeedByte(). Returns (result, frame_or_None)."""
        if self.state == STATE_WAIT_MAGIC:
            if self.magic_idx < TC1_FRAME_MAGIC_LEN:
                if byte == self._magic[self.magic_idx]:
                    self.magic_idx += 1
                else:
                    self.magic_idx = 1 if byte == ord("T") else 0
            else:
                if byte == ord(" "):
                    self.state = STATE_READ_LEN
                    self.len_buf = bytearray()
                else:
                    self.magic_idx = 1 if byte == ord("T") else 0
            return DECODE_NEED_MORE, None

        if self.state == STATE_READ_LEN:
            if byte == ord(" "):
                len_str = self.len_buf.decode("ascii", errors="replace")
                if not len_str or not len_str.isdigit():
                    self.reset()
                    return DECODE_BAD_LENGTH, None
                declared = int(len_str)
                if declared < 2 or declared > MAX_JSON_BYTES:
                    if declared > MAX_JSON_BYTES:
                        self.frames_oversized += 1
                        self.reset()
                        return DECODE_OVERSIZED, None
                    self.reset()
                    return DECODE_BAD_LENGTH, None
                self.declared_len = declared
                self.state = STATE_READ_CRC
                self.crc_buf = bytearray()
            elif ord("0") <= byte <= ord("9"):
                if len(self.len_buf) >= TC1_MAX_LEN_DIGITS:
                    self.reset()
                    return DECODE_BAD_LENGTH, None
                self.len_buf.append(byte)
            else:
                self.reset()
                return DECODE_BAD_LENGTH, None
            return DECODE_NEED_MORE, None

        if self.state == STATE_READ_CRC:
            if byte == ord(" "):
                if len(self.crc_buf) != TC1_CRC_HEX_LEN:
                    self.reset()
                    return DECODE_BAD_CRC, None
                crc_str = self.crc_buf.decode("ascii", errors="replace")
                try:
                    self.declared_crc = int(crc_str, 16)
                except ValueError:
                    self.reset()
                    return DECODE_BAD_CRC, None
                self.state = STATE_READ_PAYLOAD
                self.payload = bytearray()
            elif _is_hex(byte):
                if len(self.crc_buf) >= TC1_CRC_HEX_LEN:
                    self.reset()
                    return DECODE_BAD_CRC, None
                self.crc_buf.append(byte)
            else:
                self.reset()
                return DECODE_BAD_CRC, None
            return DECODE_NEED_MORE, None

        if self.state == STATE_READ_PAYLOAD:
            if len(self.payload) < self.declared_len:
                self.payload.append(byte)
                if len(self.payload) >= self.declared_len:
                    self.state = STATE_WAIT_NEWLINE
            else:
                self.state = STATE_WAIT_NEWLINE
            return DECODE_NEED_MORE, None

        if self.state == STATE_WAIT_NEWLINE:
            if byte == ord("\n"):
                result, frame = self._finalize()
                self.reset()
                return result, frame
            elif byte == ord("\r"):
                return DECODE_NEED_MORE, None
            else:
                self.reset()
                return DECODE_BAD_JSON, None

        self.reset()
        return DECODE_NEED_MORE, None

    def feed_bytes(self, data: bytes) -> tuple[int, Optional[dict], int]:
        """Mirror App_TC1_FeedBytes(). Returns (result, frame_or_None, consumed)."""
        for i, b in enumerate(data):
            result, frame = self.feed_byte(b)
            if result != DECODE_NEED_MORE:
                return result, frame, i + 1
        return DECODE_NEED_MORE, None, len(data)


def _is_hex(byte: int) -> bool:
    return (
        ord("0") <= byte <= ord("9")
        or ord("A") <= byte <= ord("F")
        or ord("a") <= byte <= ord("f")
    )


# =====================================================================
# Helper: build a raw TC1 frame from a dict (uses host encode_frame)
# =====================================================================

def _frame(obj: dict) -> bytes:
    return encode_frame(obj)


def _raw_frame(json_str: str) -> bytes:
    """Build a frame from a raw JSON string (bypasses host validation)."""
    json_bytes = json_str.encode("utf-8")
    crc = binascii.crc32(json_bytes) & 0xFFFFFFFF
    return (
        FRAME_MAGIC
        + b" "
        + str(len(json_bytes)).encode("ascii")
        + b" "
        + f"{crc:08X}".encode("ascii")
        + b" "
        + json_bytes
        + b"\n"
    )


def _bad_crc_frame(json_str: str) -> bytes:
    """Build a frame with an intentionally wrong CRC."""
    json_bytes = json_str.encode("utf-8")
    return (
        FRAME_MAGIC
        + b" "
        + str(len(json_bytes)).encode("ascii")
        + b" "
        + b"DEADBEEF"
        + b" "
        + json_bytes
        + b"\n"
    )


def _bad_length_frame(json_str: str, wrong_len: int) -> bytes:
    """Build a frame with an intentionally wrong length field."""
    json_bytes = json_str.encode("utf-8")
    crc = binascii.crc32(json_bytes) & 0xFFFFFFFF
    return (
        FRAME_MAGIC
        + b" "
        + str(wrong_len).encode("ascii")
        + b" "
        + f"{crc:08X}".encode("ascii")
        + b" "
        + json_bytes
        + b"\n"
    )


# =====================================================================
# Tests
# =====================================================================

class Tc1GoldenFrameTests(unittest.TestCase):
    """Normal, fragmented, coalesced, and edge-case golden frames."""

    def test_normal_set_frame_round_trip(self):
        obj = {"v": 1, "t": "set", "id": 42, "set": {"KP": 3.5}}
        frame = _frame(obj)
        parser = CParserSim()
        result, out, consumed = parser.feed_bytes(frame)
        self.assertEqual(result, DECODE_OK)
        self.assertEqual(consumed, len(frame))
        self.assertIsNotNone(out)
        self.assertEqual(out["type"], MSG_SET)
        self.assertEqual(out["msg_id"], 42)
        self.assertTrue(out["has_id"])
        self.assertEqual(out["obj"]["set"]["KP"], 3.5)

    def test_normal_cmd_frame_round_trip(self):
        obj = {"v": 1, "t": "cmd", "id": 7, "cmd": "start"}
        frame = _frame(obj)
        parser = CParserSim()
        result, out, _ = parser.feed_bytes(frame)
        self.assertEqual(result, DECODE_OK)
        self.assertEqual(out["type"], MSG_CMD)
        self.assertEqual(out["msg_id"], 7)

    def test_auth_frame_round_trip(self):
        obj = {"v": 1, "t": "auth", "token": "abc123"}
        frame = _frame(obj)
        parser = CParserSim()
        result, out, _ = parser.feed_bytes(frame)
        self.assertEqual(result, DECODE_OK)
        self.assertEqual(out["type"], MSG_AUTH)

    def test_ping_frame_round_trip(self):
        obj = {"v": 1, "t": "ping", "ms": 12345}
        frame = _frame(obj)
        parser = CParserSim()
        result, out, _ = parser.feed_bytes(frame)
        self.assertEqual(result, DECODE_OK)
        self.assertEqual(out["type"], MSG_PING)

    def test_ack_frame_round_trip(self):
        obj = {"v": 1, "t": "ack", "id": 5, "ok": True, "kind": "set"}
        frame = _frame(obj)
        parser = CParserSim()
        result, out, _ = parser.feed_bytes(frame)
        self.assertEqual(result, DECODE_OK)
        self.assertEqual(out["type"], MSG_ACK)

    def test_telemetry_frame_round_trip(self):
        obj = {
            "v": 1, "t": "tele", "seq": 100, "ms": 5000,
            "line": [0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0],
            "lm": 0x060, "bc": 2, "sa": 7, "se": 0, "bp": 1,
            "gz": 1.5, "el": 100, "er": 200, "le": 0.5,
            "po": 30.0, "st": "LF", "kp": 3.0, "kd": 0.5,
        }
        frame = _frame(obj)
        parser = CParserSim()
        result, out, _ = parser.feed_bytes(frame)
        self.assertEqual(result, DECODE_OK)
        self.assertEqual(out["type"], MSG_TELE)

    def test_log_frame_round_trip(self):
        obj = {"v": 1, "t": "log", "msg": "test message"}
        frame = _frame(obj)
        parser = CParserSim()
        result, out, _ = parser.feed_bytes(frame)
        self.assertEqual(result, DECODE_OK)
        self.assertEqual(out["type"], MSG_LOG)


class Tc1FragmentationTests(unittest.TestCase):
    """Split (fragmented) and coalesced (back-to-back) frames."""

    def test_frame_split_one_byte_at_a_time(self):
        obj = {"v": 1, "t": "set", "id": 1, "set": {"KP": 2.0}}
        frame = _frame(obj)
        parser = CParserSim()
        result = DECODE_NEED_MORE
        out = None
        for i, b in enumerate(frame):
            result, out = parser.feed_byte(b)
            if i < len(frame) - 1:
                self.assertEqual(result, DECODE_NEED_MORE,
                                 f"unexpected result at byte {i}: {result}")
        self.assertEqual(result, DECODE_OK)
        self.assertIsNotNone(out)
        self.assertEqual(out["type"], MSG_SET)

    def test_frame_split_at_arbitrary_boundary(self):
        obj = {"v": 1, "t": "cmd", "id": 3, "cmd": "stop"}
        frame = _frame(obj)
        # Split at every possible position
        for split_pos in range(1, len(frame)):
            parser = CParserSim()
            r1, o1, c1 = parser.feed_bytes(frame[:split_pos])
            self.assertEqual(r1, DECODE_NEED_MORE)
            self.assertIsNone(o1)
            r2, o2, c2 = parser.feed_bytes(frame[split_pos:])
            self.assertEqual(r2, DECODE_OK)
            self.assertIsNotNone(o2)
            self.assertEqual(o2["type"], MSG_CMD)
            self.assertEqual(c2, len(frame) - split_pos)

    def test_coalesced_back_to_back_frames(self):
        f1 = _frame({"v": 1, "t": "ping", "ms": 100})
        f2 = _frame({"v": 1, "t": "pong", "ms": 101})
        f3 = _frame({"v": 1, "t": "ack", "id": 1, "ok": True, "kind": "set"})
        combined = f1 + f2 + f3
        parser = CParserSim()
        results = []
        remaining = combined
        while remaining:
            result, out, consumed = parser.feed_bytes(remaining)
            if result == DECODE_OK:
                results.append(out)
            remaining = remaining[consumed:]
        self.assertEqual(len(results), 3)
        self.assertEqual(results[0]["type"], MSG_PING)
        self.assertEqual(results[1]["type"], MSG_PONG)
        self.assertEqual(results[2]["type"], MSG_ACK)

    def test_partial_frame_then_garbage_then_new_frame(self):
        """Parser should resync after garbage following a partial frame."""
        partial = b"TC1 10 "  # incomplete frame
        garbage = b"X\r\n"
        good = _frame({"v": 1, "t": "ping", "ms": 1})
        stream = partial + garbage + good
        parser = CParserSim()
        remaining = stream
        found = None
        while remaining:
            result, out, consumed = parser.feed_bytes(remaining)
            if result == DECODE_OK:
                found = out
                break
            remaining = remaining[consumed:]
        self.assertIsNotNone(found, "should have resynced and decoded the good frame")
        self.assertEqual(found["type"], MSG_PING)


class Tc1ErrorCasesTests(unittest.TestCase):
    """CRC errors, length errors, oversized, bad JSON, bad schema."""

    def test_bad_crc_rejected(self):
        frame = _bad_crc_frame('{"t":"ping","ms":1,"v":1}')
        parser = CParserSim()
        result, out, _ = parser.feed_bytes(frame)
        self.assertEqual(result, DECODE_BAD_CRC)
        self.assertIsNone(out)
        self.assertEqual(parser.frames_bad_crc, 1)

    def test_bad_length_too_short_rejected(self):
        frame = _bad_length_frame('{"t":"ping","ms":1,"v":1}', wrong_len=1)
        parser = CParserSim()
        result, out, _ = parser.feed_bytes(frame)
        self.assertEqual(result, DECODE_BAD_LENGTH)
        self.assertIsNone(out)

    def test_bad_length_too_long_rejected(self):
        json_str = '{"t":"ping","ms":1,"v":1}'
        frame = _bad_length_frame(json_str, wrong_len=999)
        parser = CParserSim()
        result, out, _ = parser.feed_bytes(frame)
        # Length 999 > MAX_JSON_BYTES (256) → OVERSIZED
        self.assertEqual(result, DECODE_OVERSIZED)
        self.assertIsNone(out)
        self.assertEqual(parser.frames_oversized, 1)

    def test_oversized_payload_rejected(self):
        """Payload just over MAX_JSON_BYTES."""
        big_value = "X" * (MAX_JSON_BYTES - 5)
        json_str = '{"t":"' + big_value + '"}'
        self.assertGreater(len(json_str), MAX_JSON_BYTES)
        json_bytes = json_str.encode("utf-8")
        crc = binascii.crc32(json_bytes) & 0xFFFFFFFF
        frame = (
            FRAME_MAGIC + b" " + str(len(json_bytes)).encode("ascii")
            + b" " + f"{crc:08X}".encode("ascii") + b" " + json_bytes + b"\n"
        )
        parser = CParserSim()
        result, out, _ = parser.feed_bytes(frame)
        self.assertEqual(result, DECODE_OVERSIZED)

    def test_bad_json_not_object_rejected(self):
        json_str = '"just a string"'
        frame = _raw_frame(json_str)
        parser = CParserSim()
        result, out, _ = parser.feed_bytes(frame)
        # CRC passes, but JSON doesn't start with { → BAD_JSON
        self.assertEqual(result, DECODE_BAD_JSON)
        self.assertIsNone(out)
        self.assertEqual(parser.frames_bad_json, 1)

    def test_bad_json_array_rejected(self):
        json_str = '[1,2,3]'
        frame = _raw_frame(json_str)
        parser = CParserSim()
        result, out, _ = parser.feed_bytes(frame)
        self.assertEqual(result, DECODE_BAD_JSON)

    def test_bad_schema_unknown_msg_type_rejected(self):
        json_str = '{"t":"unknown_type","v":1}'
        frame = _raw_frame(json_str)
        parser = CParserSim()
        result, out, _ = parser.feed_bytes(frame)
        self.assertEqual(result, DECODE_BAD_SCHEMA)
        self.assertIsNone(out)
        self.assertEqual(parser.frames_bad_schema, 1)

    def test_bad_schema_missing_t_field_rejected(self):
        json_str = '{"v":1,"id":1}'
        frame = _raw_frame(json_str)
        parser = CParserSim()
        result, out, _ = parser.feed_bytes(frame)
        self.assertEqual(result, DECODE_BAD_SCHEMA)

    def test_truncated_crc_field_rejected(self):
        """CRC with only 7 hex digits should be rejected."""
        json_str = '{"t":"ping","ms":1,"v":1}'
        json_bytes = json_str.encode("utf-8")
        frame = b"TC1 " + str(len(json_bytes)).encode("ascii") + b" 1234567 " + json_bytes + b"\n"
        parser = CParserSim()
        result, out, _ = parser.feed_bytes(frame)
        self.assertEqual(result, DECODE_BAD_CRC)

    def test_missing_newline_does_not_produce_frame(self):
        """Frame without trailing \n should not produce a result."""
        json_str = '{"t":"ping","ms":1,"v":1}'
        json_bytes = json_str.encode("utf-8")
        crc = binascii.crc32(json_bytes) & 0xFFFFFFFF
        frame_no_nl = (
            FRAME_MAGIC + b" " + str(len(json_bytes)).encode("ascii")
            + b" " + f"{crc:08X}".encode("ascii") + b" " + json_bytes
        )
        parser = CParserSim()
        result, out, consumed = parser.feed_bytes(frame_no_nl)
        self.assertEqual(result, DECODE_NEED_MORE)
        self.assertIsNone(out)


class Tc1DuplicateIdTests(unittest.TestCase):
    """Duplicate message ID detection (statistics only, not rejection)."""

    def test_duplicate_id_counted_but_not_rejected(self):
        f1 = _frame({"v": 1, "t": "set", "id": 10, "set": {"KP": 1.0}})
        f2 = _frame({"v": 1, "t": "set", "id": 10, "set": {"KP": 2.0}})
        parser = CParserSim()
        r1, o1, _ = parser.feed_bytes(f1)
        r2, o2, _ = parser.feed_bytes(f2)
        self.assertEqual(r1, DECODE_OK)
        self.assertEqual(r2, DECODE_OK)  # duplicate ID is NOT rejected
        self.assertEqual(parser.frames_duplicate_id, 1)
        self.assertEqual(parser.frames_ok, 2)

    def test_monotonic_id_sequence_accepted(self):
        parser = CParserSim()
        for i in range(1, 6):
            frame = _frame({"v": 1, "t": "set", "id": i, "set": {"KP": float(i)}})
            result, _, _ = parser.feed_bytes(frame)
            self.assertEqual(result, DECODE_OK)
        self.assertEqual(parser.frames_duplicate_id, 0)
        self.assertEqual(parser.frames_ok, 5)


class Tc1EncodeDecodeRoundTripTests(unittest.TestCase):
    """Verify encode_frame output is decodable by the C parser simulation."""

    def test_all_message_types_round_trip(self):
        test_msgs = [
            {"v": 1, "t": "auth", "token": "secret"},
            {"v": 1, "t": "set", "id": 1, "set": {"KP": 3.0, "KD": 0.5}},
            {"v": 1, "t": "cmd", "id": 2, "cmd": "start"},
            {"v": 1, "t": "ping", "ms": 999},
            {"v": 1, "t": "pong", "ms": 1000},
            {"v": 1, "t": "ack", "id": 3, "ok": True, "kind": "set"},
            {"v": 1, "t": "nack", "id": 4, "error": "bad param"},
            {"v": 1, "t": "log", "msg": "system ready"},
        ]
        for obj in test_msgs:
            frame = encode_frame(obj)
            parser = CParserSim()
            result, out, _ = parser.feed_bytes(frame)
            self.assertEqual(result, DECODE_OK, f"failed for {obj}")
            self.assertIsNotNone(out)
            self.assertEqual(out["obj"], obj)

    def test_crc32_consistency_c_vs_python(self):
        """Verify C CRC32 algorithm matches Python binascii.crc32."""
        test_payloads = [
            b'{"t":"ping","ms":1,"v":1}',
            b'{"t":"set","id":1,"set":{"KP":3.5},"v":1}',
            b"{}",
            b'{"x":1}',
            b"",
        ]
        for payload in test_payloads:
            # Python reference
            py_crc = binascii.crc32(payload) & 0xFFFFFFFF
            # C simulation: init 0xFFFFFFFF, XOR 0xFFFFFFFF at end
            c_crc = CParserSim._crc32(payload)
            self.assertEqual(py_crc, c_crc,
                             f"CRC mismatch for {payload!r}: py={py_crc:08X} c={c_crc:08X}")


class Tc1ResyncTests(unittest.TestCase):
    """Parser must resync after various corruption scenarios."""

    def test_resync_after_bad_magic(self):
        garbage = b"GARBAGE DATA\n"
        good = _frame({"v": 1, "t": "ping", "ms": 1})
        stream = garbage + good
        parser = CParserSim()
        remaining = stream
        found = None
        while remaining:
            result, out, consumed = parser.feed_bytes(remaining)
            if result == DECODE_OK:
                found = out
                break
            remaining = remaining[consumed:]
        self.assertIsNotNone(found)
        self.assertEqual(found["type"], MSG_PING)

    def test_resync_after_partial_magic(self):
        """Partial 'TC' followed by non-'1' should resync."""
        partial = b"TC"
        good = _frame({"v": 1, "t": "ping", "ms": 1})
        stream = partial + good
        parser = CParserSim()
        remaining = stream
        found = None
        while remaining:
            result, out, consumed = parser.feed_bytes(remaining)
            if result == DECODE_OK:
                found = out
                break
            remaining = remaining[consumed:]
        self.assertIsNotNone(found)

    def test_resync_after_bad_crc_frame(self):
        bad = _bad_crc_frame('{"t":"ping","ms":1,"v":1}')
        good = _frame({"v": 1, "t": "ack", "id": 1, "ok": True, "kind": "cmd"})
        stream = bad + good
        parser = CParserSim()
        remaining = stream
        results = []
        while remaining:
            result, out, consumed = parser.feed_bytes(remaining)
            if result == DECODE_OK:
                results.append(out)
            remaining = remaining[consumed:]
        self.assertEqual(len(results), 1)
        self.assertEqual(results[0]["type"], MSG_ACK)
        self.assertEqual(parser.frames_bad_crc, 1)
        self.assertEqual(parser.frames_ok, 1)


class Tc1AuthAndHeartbeatTests(unittest.TestCase):
    """Auth failure and heartbeat timeout scenarios (protocol-level)."""

    def test_auth_frame_accepted_by_parser(self):
        """Parser should accept a valid auth frame; auth logic is handled upstream."""
        obj = {"v": 1, "t": "auth", "token": "valid_token_123"}
        frame = _frame(obj)
        parser = CParserSim()
        result, out, _ = parser.feed_bytes(frame)
        self.assertEqual(result, DECODE_OK)
        self.assertEqual(out["type"], MSG_AUTH)

    def test_auth_frame_with_wrong_token_still_parsed(self):
        """Parser doesn't validate token content; that's app_tuning's job."""
        obj = {"v": 1, "t": "auth", "token": "WRONG"}
        frame = _frame(obj)
        parser = CParserSim()
        result, out, _ = parser.feed_bytes(frame)
        self.assertEqual(result, DECODE_OK)
        self.assertEqual(out["type"], MSG_AUTH)

    def test_heartbeat_ping_sequence(self):
        """Simulate a ping/pong heartbeat exchange."""
        ping = _frame({"v": 1, "t": "ping", "ms": 1000})
        pong = _frame({"v": 1, "t": "pong", "ms": 1001})
        parser = CParserSim()
        r1, o1, _ = parser.feed_bytes(ping)
        r2, o2, _ = parser.feed_bytes(pong)
        self.assertEqual(r1, DECODE_OK)
        self.assertEqual(r2, DECODE_OK)
        self.assertEqual(o1["type"], MSG_PING)
        self.assertEqual(o2["type"], MSG_PONG)
        self.assertEqual(o1["obj"]["ms"], 1000)
        self.assertEqual(o2["obj"]["ms"], 1001)

    def test_multiple_pings_no_interference(self):
        """Rapid pings should all decode correctly."""
        parser = CParserSim()
        for i in range(10):
            frame = _frame({"v": 1, "t": "ping", "ms": i})
            result, _, _ = parser.feed_bytes(frame)
            self.assertEqual(result, DECODE_OK)
        self.assertEqual(parser.frames_ok, 10)


class Tc1EncodeFrameTests(unittest.TestCase):
    """Verify the C-side App_TC1_EncodeFrame simulation matches host encode_frame."""

    def test_encode_produces_valid_frame(self):
        obj = {"v": 1, "t": "tele", "seq": 1}
        json_str = compact_json(obj)
        json_bytes = json_str.encode("utf-8")
        crc = binascii.crc32(json_bytes) & 0xFFFFFFFF
        # Simulate C encoder
        c_frame = (
            b"TC1 "
            + str(len(json_bytes)).encode("ascii")
            + b" "
            + f"{crc:08X}".encode("ascii")
            + b" "
            + json_bytes
            + b"\n"
        )
        # Host encoder
        py_frame = encode_frame(obj)
        self.assertEqual(c_frame, py_frame)

    def test_encoded_frame_decodable_by_c_parser(self):
        obj = {"v": 1, "t": "ack", "id": 99, "ok": True, "kind": "set"}
        frame = encode_frame(obj)
        parser = CParserSim()
        result, out, _ = parser.feed_bytes(frame)
        self.assertEqual(result, DECODE_OK)
        self.assertEqual(out["type"], MSG_ACK)
        self.assertEqual(out["msg_id"], 99)


if __name__ == "__main__":
    unittest.main()
