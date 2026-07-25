import json
import math
import sys
import unittest
import zlib
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
HOST = ROOT / "host"
if str(HOST) not in sys.path:
    sys.path.insert(0, str(HOST))

from protocol import (  # noqa: E402
    PARAMS,
    PROTOCOL_VERSION,
    ProtocolError,
    build_ack,
    build_auth,
    build_cmd,
    build_motor_test,
    build_rotate,
    build_ping,
    build_set,
    clamp,
    decode_frame,
    encode_frame,
    parse_ack,
    parse_telemetry,
    validate_ping,
    validate_request,
)


def telemetry(**updates):
    obj = {
        "t": "tele",
        "line": [0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0],
        "lm": 0x060,
        "bc": 2,
        "sa": 5,
        "se": 0,
        "bp": 1,
        "gz": 0.25,
        "el": 40,
        "er": 41,
        "le": -0.3,
        "po": 2.5,
        "st": "LF",
        "kp": 5.0,
        "kd": 1.0,
    }
    obj.update(updates)
    return obj


class Tc1FrameTests(unittest.TestCase):
    def test_encode_frame_uses_utf8_byte_length_and_uppercase_crc(self):
        obj = {"v": 1, "t": "log", "msg": "循迹 ok"}
        json_bytes = json.dumps(
            obj, ensure_ascii=False, separators=(",", ":"), allow_nan=False
        ).encode("utf-8")
        expected_crc = zlib.crc32(json_bytes) & 0xFFFFFFFF

        frame = encode_frame(obj)

        self.assertEqual(
            frame,
            b"TC1 "
            + str(len(json_bytes)).encode("ascii")
            + b" "
            + f"{expected_crc:08X}".encode("ascii")
            + b" "
            + json_bytes
            + b"\n",
        )
        self.assertEqual(decode_frame(frame), obj)

    def test_decode_rejects_bad_length_crc_header_and_non_object(self):
        frame = encode_frame({"v": 1, "t": "ping", "ms": 5})
        parts = frame.rstrip(b"\n").split(b" ", 3)

        with self.assertRaises(ProtocolError):
            decode_frame(b"BAD " + b" ".join(parts[1:]) + b"\n")
        with self.assertRaises(ProtocolError):
            decode_frame(parts[0] + b" 999 " + parts[2] + b" " + parts[3] + b"\n")
        with self.assertRaises(ProtocolError):
            decode_frame(parts[0] + b" " + parts[1] + b" 00000000 " + parts[3] + b"\n")
        with self.assertRaises(ProtocolError):
            encode_frame([1, 2, 3])


class RequestTests(unittest.TestCase):
    def test_build_set_and_cmd_accept_request_id(self):
        set_obj = json.loads(build_set({"KP": 100, "KD": 1.5}, request_id=17))
        self.assertEqual(set_obj, {"v": 1, "t": "set", "set": {"KP": 50.0, "KD": 1.5}, "id": 17})
        self.assertEqual(
            json.loads(build_cmd("stop", request_id=18)),
            {"v": 1, "t": "cmd", "cmd": "stop", "id": 18},
        )
        self.assertEqual(validate_request(set_obj), set_obj)

    def test_build_without_id_remains_compatible_for_tcp_server(self):
        self.assertEqual(json.loads(build_cmd("start")), {"v": 1, "t": "cmd", "cmd": "start"})
        self.assertEqual(json.loads(build_set({"KP": 6})), {"v": 1, "t": "set", "set": {"KP": 6.0}})

    def test_build_rotate_requires_finite_nonzero_angle_in_range(self):
        self.assertEqual(
            json.loads(build_rotate(90, request_id=19)),
            {"v": 1, "t": "cmd", "cmd": "rotate", "deg": 90.0, "id": 19},
        )
        for value in (0, 180.1, -180.1, math.nan, math.inf, "90"):
            with self.subTest(value=value):
                with self.assertRaises((ValueError, ProtocolError)):
                    build_rotate(value, request_id=19)

    def test_motor_test_requires_explicit_lifted_confirmation(self):
        self.assertEqual(
            json.loads(build_motor_test(request_id=20)),
            {"v": 1, "t": "cmd", "cmd": "motor_test", "lifted": True, "id": 20},
        )
        for request in (
            {"v": 1, "t": "cmd", "cmd": "motor_test"},
            {"v": 1, "t": "cmd", "cmd": "motor_test", "lifted": False},
        ):
            with self.assertRaises(ProtocolError):
                validate_request(request)

    def test_motor_test_telemetry_is_a_complete_group(self):
        sample = telemetry(
            st="MT", mt=True, mph=2, mtg=30, mlp=31, mrp=29, mrs="running"
        )
        self.assertEqual(parse_telemetry(sample), sample)
        sample.pop("mrp")
        self.assertIsNone(parse_telemetry(sample))

    def test_motor_test_accepts_reverse_and_stop_qualification_phases(self):
        reverse = telemetry(
            st="MT", mt=True, mph=8, mtg=-40,
            mlp=-42, mrp=-39, mrs="running",
        )
        self.assertEqual(parse_telemetry(reverse), reverse)
        stopped = telemetry(
            st="MT", mt=True, mph=9, mtg=0,
            mlp=0, mrp=0, mrs="running",
        )
        self.assertEqual(parse_telemetry(stopped), stopped)

    def test_invalid_parameter_types_and_non_finite_values_are_rejected(self):
        for value in (True, "5", None, math.nan, math.inf, -math.inf):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    clamp("KP", value)
                with self.assertRaises((ValueError, ProtocolError)):
                    build_set({"KP": value})
        with self.assertRaises(ProtocolError):
            build_set({"UNKNOWN": 1})
        with self.assertRaises(ProtocolError):
            validate_request({"v": 1, "id": 1, "set": {"KP": 51}})

    def test_ping_schema(self):
        ping = json.loads(build_ping(1234))
        self.assertEqual(validate_ping(ping), {"v": PROTOCOL_VERSION, "t": "ping", "ms": 1234})
        with self.assertRaises(ProtocolError):
            validate_ping({"v": 1, "t": "ping", "ms": -1})

    def test_build_auth_has_bounded_token(self):
        self.assertEqual(
            json.loads(build_auth("track_car_2025")),
            {"v": 1, "t": "auth", "token": "track_car_2025"},
        )
        self.assertEqual(
            json.loads(build_auth("track_car_2025", session="abc123")),
            {"v": 1, "t": "auth", "token": "track_car_2025", "session": "abc123"},
        )
        with self.assertRaises(ProtocolError):
            build_auth("")
        with self.assertRaises(ProtocolError):
            build_auth("track_car_2025", session="")


class TelemetryTests(unittest.TestCase):
    def test_path_fish_and_extended_telemetry_are_accepted(self):
        base = telemetry()
        self.assertEqual(parse_telemetry(json.dumps(base)), base)

        extended = telemetry(
            v=1,
            seq=42,
            ms=12345,
            ki=0.0,
            gd=0.15,
            vb=40,
            vs=50,
            vc=30,
            rt=6.0,
            drop=2,
            sa=9,
            se=3,
            lp=24,
            rp=-31,
        )
        self.assertEqual(parse_telemetry(extended), extended)

    def test_bad_telemetry_schema_is_rejected(self):
        invalid_objects = [
            telemetry(line=[]),
            telemetry(line=[0] * 11 + [2]),
            telemetry(lm=0x061),
            telemetry(bc=3),
            telemetry(le="0.1"),
            telemetry(le=math.nan),
            telemetry(st="UNKNOWN"),
            telemetry(seq=-1),
            telemetry(bp=2),
            telemetry(extra=1),
        ]
        for obj in invalid_objects:
            with self.subTest(obj=obj):
                self.assertIsNone(parse_telemetry(obj))
        raw_nan = json.dumps(telemetry()).replace('"le": -0.3', '"le": NaN')
        self.assertIsNone(parse_telemetry(raw_nan))


class AckTests(unittest.TestCase):
    def test_ack_with_full_applied_snapshot(self):
        applied = {name: info["default"] for name, info in PARAMS.items()}
        ack = json.loads(build_ack(9, True, "set", applied=applied))
        parsed = parse_ack(ack)
        self.assertIsNotNone(parsed)
        self.assertEqual(parsed["id"], 9)
        self.assertEqual(set(parsed["applied"]), set(PARAMS))

    def test_ack_rejects_wrong_types_and_partial_snapshot(self):
        self.assertIsNone(parse_ack({"v": 1, "t": "ack", "id": 1, "ok": 1, "kind": "cmd"}))
        self.assertIsNone(
            parse_ack({
                "v": 1,
                "t": "ack",
                "id": 1,
                "ok": True,
                "kind": "set",
                "applied": {"KP": 5.0},
            })
        )


if __name__ == "__main__":
    unittest.main()
