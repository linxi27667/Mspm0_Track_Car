"""Track-car tuning protocol and TC1 line framing."""

from __future__ import annotations

import json
import math
import zlib
from typing import Any

from param_schema import PARAM_SCHEMA, TELEMETRY_PARAM_FIELDS as _SCHEMA_TELEMETRY_FIELDS


PROTOCOL_VERSION = 1
FRAME_MAGIC = b"TC1"
MAX_JSON_BYTES = 384
MAX_FRAME_BYTES = len(FRAME_MAGIC) + 1 + 3 + 1 + 8 + 1 + MAX_JSON_BYTES + 1
MAX_REQUEST_ID = 0xFFFFFFFF

# Request manager: only one pending request at a time, 300ms timeout, 2 retries
REQUEST_TIMEOUT_MS = 300
REQUEST_MAX_RETRIES = 2  # 2 retries = 3 total attempts (original + 2 retries)


class ProtocolError(ValueError):
    """Raised when a frame or protocol object is malformed."""


# PARAMS is derived from the shared param_schema (single source of truth).
# Only the fields needed by the protocol/UI are exposed here.
PARAMS: dict[str, dict[str, Any]] = {
    p["name"]: {
        "min": p["min"],
        "max": p["max"],
        "default": p["default"],
        "desc": p["desc"],
        "help": p.get("help", ""),
        "step": p["step"],
    }
    for p in PARAM_SCHEMA
}

STATE_MAP = {
    "ID": "IDLE",
    "LF": "LINE_FOLLOW",
    "RA": "RIGHT_ANGLE",
    "LL": "LINE_LOST",
    "SL": "START_LINE",
    "CR": "CROSS",
    "RT": "ROTATE",
    "HH": "HEADING_HOLD",
    "MT": "MOTOR_TEST",
    "FN": "FINISHED",
}

VALID_CMDS = {
    "start", "stop", "rotate", "motor_test",
    "heading_hold_on", "heading_hold_off",
}

_TELEMETRY_REQUIRED = {
    "t", "line", "lm", "bc", "sa", "se", "bp",
    "gz", "el", "er", "le", "po", "st", "kp", "kd",
}
_TELEMETRY_OPTIONAL = {
    "v", "seq", "ms", "ki", "gd", "vb", "vs", "vc", "rt", "drop",
    "lp", "rp",
    "tg", "ta", "te", "rr", "hh", "ht", "ha", "he", "hr",
    "mt", "mph", "mtg", "mlp", "mrp", "mrs",
}
_TELEMETRY_PARAM_FIELDS = _SCHEMA_TELEMETRY_FIELDS


def _reject_json_constant(value: str):
    raise ProtocolError(f"non-finite JSON number: {value}")


def _load_object(value: str | bytes | bytearray | dict[str, Any]) -> dict[str, Any]:
    if isinstance(value, dict):
        return dict(value)
    if isinstance(value, (bytes, bytearray)):
        try:
            value = bytes(value).decode("utf-8")
        except UnicodeDecodeError as exc:
            raise ProtocolError("JSON is not valid UTF-8") from exc
    if not isinstance(value, str):
        raise ProtocolError("JSON payload must be an object, string, or bytes")
    try:
        obj = json.loads(value, parse_constant=_reject_json_constant)
    except (json.JSONDecodeError, TypeError, ValueError) as exc:
        if isinstance(exc, ProtocolError):
            raise
        raise ProtocolError(f"invalid JSON: {exc}") from exc
    if not isinstance(obj, dict):
        raise ProtocolError("JSON payload must be an object")
    return obj


def compact_json(obj: dict[str, Any]) -> str:
    """Serialize one JSON object deterministically without insignificant spaces."""
    if not isinstance(obj, dict):
        raise ProtocolError("JSON payload must be an object")
    try:
        return json.dumps(
            obj,
            ensure_ascii=False,
            separators=(",", ":"),
            allow_nan=False,
        )
    except (TypeError, ValueError) as exc:
        raise ProtocolError(f"JSON serialization failed: {exc}") from exc


def load_json_object(value: str | bytes | bytearray | dict[str, Any]) -> dict[str, Any]:
    """Parse a JSON object with non-finite constants disabled."""
    return _load_object(value)


def encode_frame(payload: dict[str, Any] | str | bytes | bytearray) -> bytes:
    """Encode a JSON object as ``TC1 <len> <CRC32> <json>\n``."""
    obj = _load_object(payload)
    json_bytes = compact_json(obj).encode("utf-8")
    if not json_bytes or len(json_bytes) > MAX_JSON_BYTES:
        raise ProtocolError(f"JSON payload length out of range: {len(json_bytes)}")
    crc = zlib.crc32(json_bytes) & 0xFFFFFFFF
    return FRAME_MAGIC + b" " + str(len(json_bytes)).encode("ascii") + b" " + f"{crc:08X}".encode("ascii") + b" " + json_bytes + b"\n"


def decode_frame(frame: bytes | bytearray | str) -> dict[str, Any]:
    """Validate and decode one complete TC1 line frame."""
    if isinstance(frame, str):
        frame = frame.encode("utf-8")
    if not isinstance(frame, (bytes, bytearray)):
        raise ProtocolError("frame must be bytes or str")
    raw = bytes(frame)
    if raw.endswith(b"\n"):
        raw = raw[:-1]
    if raw.endswith(b"\r"):
        raw = raw[:-1]
    if len(raw) + 1 > MAX_FRAME_BYTES:
        raise ProtocolError("frame exceeds maximum length")

    parts = raw.split(b" ", 3)
    if len(parts) != 4 or parts[0] != FRAME_MAGIC:
        raise ProtocolError("invalid TC1 frame header")
    length_text, crc_text, json_bytes = parts[1], parts[2], parts[3]
    if not length_text or not length_text.isdigit():
        raise ProtocolError("invalid JSON byte length")
    json_length = int(length_text)
    if json_length <= 0 or json_length > MAX_JSON_BYTES:
        raise ProtocolError("JSON byte length out of range")
    if len(json_bytes) != json_length:
        raise ProtocolError(f"JSON byte length mismatch: expected {json_length}, got {len(json_bytes)}")
    if len(crc_text) != 8:
        raise ProtocolError("CRC32 must contain 8 hex digits")
    try:
        expected_crc = int(crc_text, 16)
    except ValueError as exc:
        raise ProtocolError("CRC32 is not hexadecimal") from exc
    actual_crc = zlib.crc32(json_bytes) & 0xFFFFFFFF
    if actual_crc != expected_crc:
        raise ProtocolError(f"CRC32 mismatch: expected {expected_crc:08X}, got {actual_crc:08X}")
    return _load_object(json_bytes)


def _require_int(name: str, value: Any, minimum: int = 0, maximum: int = MAX_REQUEST_ID) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ProtocolError(f"{name} must be an integer")
    if value < minimum or value > maximum:
        raise ProtocolError(f"{name} out of range")
    return value


def _require_number(name: str, value: Any, minimum: float | None = None, maximum: float | None = None) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ProtocolError(f"{name} must be a number")
    number = float(value)
    if not math.isfinite(number):
        raise ProtocolError(f"{name} must be finite")
    if minimum is not None and number < minimum:
        raise ProtocolError(f"{name} below minimum {minimum}")
    if maximum is not None and number > maximum:
        raise ProtocolError(f"{name} above maximum {maximum}")
    return number


def clamp(name: str, value: Any) -> float:
    """Clamp a finite numeric parameter; invalid types are rejected."""
    if name not in PARAMS:
        raise ValueError(f"unknown param: {name}")
    p = PARAMS[name]
    try:
        number = _require_number(name, value)
    except ProtocolError as exc:
        raise ValueError(str(exc)) from exc
    return max(float(p["min"]), min(float(p["max"]), number))


def validate_params(
    params: dict[str, Any],
    *,
    require_all: bool = False,
    clamp_values: bool = False,
) -> dict[str, float]:
    if not isinstance(params, dict) or not params:
        raise ProtocolError("parameter set must be a non-empty object")
    unknown = set(params) - set(PARAMS)
    if unknown:
        raise ProtocolError(f"unknown parameters: {', '.join(sorted(unknown))}")
    if require_all and set(params) != set(PARAMS):
        missing = set(PARAMS) - set(params)
        raise ProtocolError(f"applied parameter snapshot missing: {', '.join(sorted(missing))}")

    clean: dict[str, float] = {}
    for name, value in params.items():
        info = PARAMS[name]
        if clamp_values:
            clean[name] = clamp(name, value)
        else:
            clean[name] = _require_number(name, value, float(info["min"]), float(info["max"]))
    return clean


def _validate_version(obj: dict[str, Any], *, optional: bool = False) -> None:
    if "v" not in obj:
        if optional:
            return
        raise ProtocolError("missing protocol version")
    if _require_int("v", obj["v"], PROTOCOL_VERSION, PROTOCOL_VERSION) != PROTOCOL_VERSION:
        raise ProtocolError("unsupported protocol version")


def validate_request(obj: dict[str, Any], *, require_id: bool = True) -> dict[str, Any]:
    """Validate a downlink set/cmd request and normalize parameter numbers."""
    if not isinstance(obj, dict):
        raise ProtocolError("request must be an object")
    _validate_version(obj)
    has_set = "set" in obj
    has_cmd = "cmd" in obj
    if has_set == has_cmd:
        raise ProtocolError("request must contain exactly one of set or cmd")
    expected_type = "set" if has_set else "cmd"
    if obj.get("t") != expected_type:
        raise ProtocolError(f"request t must be {expected_type!r}")
    if require_id or "id" in obj:
        _require_int("id", obj.get("id"))

    clean = dict(obj)
    if has_set:
        if set(obj) - {"v", "t", "id", "set"}:
            raise ProtocolError("request contains unknown fields")
        clean["set"] = validate_params(obj["set"])
    else:
        cmd = obj["cmd"]
        if not isinstance(cmd, str) or cmd not in VALID_CMDS:
            raise ProtocolError(f"invalid cmd: {cmd}")
        allowed = {"v", "t", "id", "cmd"}
        if cmd == "rotate":
            allowed.add("deg")
            if "deg" not in obj:
                raise ProtocolError("rotate requires deg")
            degrees = _require_number("deg", obj["deg"], -180.0, 180.0)
            if degrees == 0.0:
                raise ProtocolError("rotate deg must be nonzero")
            clean["deg"] = degrees
        elif cmd == "motor_test":
            allowed.add("lifted")
            if obj.get("lifted") is not True:
                raise ProtocolError("motor_test requires lifted=true")
            clean["lifted"] = True
        if set(obj) - allowed:
            raise ProtocolError("request contains unknown fields")
    return clean


def validate_telemetry(obj: dict[str, Any]) -> dict[str, Any]:
    """Validate one 12-channel Path Fish telemetry object."""
    if not isinstance(obj, dict):
        raise ProtocolError("telemetry must be an object")
    missing = _TELEMETRY_REQUIRED - set(obj)
    unknown = set(obj) - _TELEMETRY_REQUIRED - _TELEMETRY_OPTIONAL
    if missing:
        raise ProtocolError(f"telemetry missing fields: {', '.join(sorted(missing))}")
    if unknown:
        raise ProtocolError(f"telemetry contains unknown fields: {', '.join(sorted(unknown))}")
    if obj.get("t") != "tele":
        raise ProtocolError("not a telemetry object")
    _validate_version(obj, optional=True)

    line = obj["line"]
    if not isinstance(line, list) or len(line) != 12:
        raise ProtocolError("line must contain exactly 12 bits")
    for index, sample in enumerate(line):
        _require_int(f"line[{index}]", sample, 0, 1)
    line_mask = _require_int("lm", obj["lm"], 0, 0x0FFF)
    black_count = _require_int("bc", obj["bc"], 0, 12)
    expected_mask = sum(bit << index for index, bit in enumerate(line))
    if line_mask != expected_mask:
        raise ProtocolError("lm does not match line bits")
    if black_count != sum(line):
        raise ProtocolError("bc does not match line bits")
    _require_int("sa", obj["sa"])
    _require_int("se", obj["se"])
    _require_int("bp", obj["bp"], 0, 1)
    _require_number("gz", obj["gz"])
    _require_int("el", obj["el"], -0x80000000, 0x7FFFFFFF)
    _require_int("er", obj["er"], -0x80000000, 0x7FFFFFFF)
    for field in ("lp", "rp"):
        if field in obj:
            _require_int(field, obj[field], -100, 100)
    _require_number("le", obj["le"], -3.5, 3.5)
    _require_number("po", obj["po"], -100.0, 100.0)
    if not isinstance(obj["st"], str) or obj["st"] not in STATE_MAP:
        raise ProtocolError("invalid telemetry state")

    for field, param_name in _TELEMETRY_PARAM_FIELDS.items():
        if field in obj:
            info = PARAMS[param_name]
            _require_number(field, obj[field], float(info["min"]), float(info["max"]))
    for field in ("seq", "ms", "drop"):
        if field in obj:
            _require_int(field, obj[field])
    rotation_fields = {"tg", "ta", "te", "rr"}
    present_rotation_fields = rotation_fields & set(obj)
    if present_rotation_fields and present_rotation_fields != rotation_fields:
        raise ProtocolError("rotation telemetry group is incomplete")
    if present_rotation_fields:
        target = _require_number("tg", obj["tg"], -180.0, 180.0)
        if target == 0.0:
            raise ProtocolError("rotation target must be nonzero")
        _require_number("ta", obj["ta"], -360.0, 360.0)
        _require_number("te", obj["te"], -360.0, 360.0)
        if not isinstance(obj["rr"], str) or not obj["rr"] or len(obj["rr"]) > 32:
            raise ProtocolError("invalid rotation result")
    hold_fields = {"hh", "ht", "ha", "he", "hr"}
    present_hold_fields = hold_fields & set(obj)
    if present_hold_fields and present_hold_fields != hold_fields:
        raise ProtocolError("heading-hold telemetry group is incomplete")
    if present_hold_fields:
        if obj["hh"] is not True:
            raise ProtocolError("heading hold must be active")
        _require_number("ht", obj["ht"])
        _require_number("ha", obj["ha"])
        _require_number("he", obj["he"])
        if not isinstance(obj["hr"], str) or not obj["hr"] or len(obj["hr"]) > 32:
            raise ProtocolError("invalid heading-hold result")
    motor_test_fields = {"mt", "mph", "mtg", "mlp", "mrp", "mrs"}
    present_motor_test_fields = motor_test_fields & set(obj)
    if present_motor_test_fields and present_motor_test_fields != motor_test_fields:
        raise ProtocolError("motor-test telemetry group is incomplete")
    if present_motor_test_fields:
        if obj["mt"] is not True or obj["st"] != "MT":
            raise ProtocolError("motor test must be active in MT state")
        _require_int("mph", obj["mph"], 0, 9)
        _require_int("mtg", obj["mtg"], -40, 40)
        _require_int("mlp", obj["mlp"], -100, 100)
        _require_int("mrp", obj["mrp"], -100, 100)
        if not isinstance(obj["mrs"], str) or not obj["mrs"] or len(obj["mrs"]) > 32:
            raise ProtocolError("invalid motor-test result")
    return dict(obj)


def parse_telemetry(value: str | bytes | bytearray | dict[str, Any]):
    """Return validated telemetry or ``None`` for an invalid object."""
    try:
        return validate_telemetry(_load_object(value))
    except (ProtocolError, TypeError, ValueError):
        return None


def validate_ack(obj: dict[str, Any]) -> dict[str, Any]:
    if not isinstance(obj, dict):
        raise ProtocolError("ack must be an object")
    allowed = {"v", "t", "id", "ok", "kind", "applied", "error"}
    if set(obj) - allowed:
        raise ProtocolError("ack contains unknown fields")
    _validate_version(obj)
    if obj.get("t") != "ack":
        raise ProtocolError("not an ack object")
    _require_int("id", obj.get("id"))
    if not isinstance(obj.get("ok"), bool):
        raise ProtocolError("ack ok must be boolean")
    if obj.get("kind") not in {"set", "cmd"}:
        raise ProtocolError("ack kind must be set or cmd")
    if "applied" in obj:
        validate_params(obj["applied"], require_all=True)
    if "error" in obj:
        error = obj["error"]
        if not isinstance(error, str) or not error or len(error) > 256:
            raise ProtocolError("ack error must be a non-empty string up to 256 characters")
    return dict(obj)


def parse_ack(value: str | bytes | bytearray | dict[str, Any]):
    try:
        return validate_ack(_load_object(value))
    except (ProtocolError, TypeError, ValueError):
        return None


def validate_ping(obj: dict[str, Any]) -> dict[str, Any]:
    if set(obj) != {"v", "t", "ms"} or obj.get("t") != "ping":
        raise ProtocolError("invalid ping object")
    _validate_version(obj)
    _require_int("ms", obj["ms"])
    return dict(obj)


def build_set(params: dict[str, Any], request_id: int | None = None) -> str:
    obj: dict[str, Any] = {
        "v": PROTOCOL_VERSION,
        "t": "set",
        "set": validate_params(params, clamp_values=True),
    }
    if request_id is not None:
        obj["id"] = _require_int("id", request_id)
    return compact_json(obj)


def build_cmd(cmd: str, request_id: int | None = None) -> str:
    obj: dict[str, Any] = {"v": PROTOCOL_VERSION, "t": "cmd", "cmd": cmd}
    if request_id is not None:
        obj["id"] = _require_int("id", request_id)
    validate_request(obj, require_id=request_id is not None)
    return compact_json(obj)


def build_rotate(degrees: float, request_id: int | None = None) -> str:
    obj: dict[str, Any] = {
        "v": PROTOCOL_VERSION,
        "t": "cmd",
        "cmd": "rotate",
        "deg": degrees,
    }
    if request_id is not None:
        obj["id"] = _require_int("id", request_id)
    return compact_json(validate_request(obj, require_id=request_id is not None))


def build_motor_test(request_id: int | None = None) -> str:
    obj: dict[str, Any] = {
        "v": PROTOCOL_VERSION,
        "t": "cmd",
        "cmd": "motor_test",
        "lifted": True,
    }
    if request_id is not None:
        obj["id"] = _require_int("id", request_id)
    return compact_json(validate_request(obj, require_id=request_id is not None))


def build_ping(ms: int) -> str:
    obj = {"v": PROTOCOL_VERSION, "t": "ping", "ms": _require_int("ms", ms)}
    return compact_json(obj)


def build_auth(token: str, session: str | None = None) -> str:
    """Build the one-way connection authentication message."""
    if not isinstance(token, str) or not token or len(token) >= 64:
        raise ProtocolError("auth token must contain 1 to 63 characters")
    obj = {"v": PROTOCOL_VERSION, "t": "auth", "token": token}
    if session is not None:
        if not isinstance(session, str) or not session or len(session) > 32:
            raise ProtocolError("auth session must contain 1 to 32 characters")
        obj["session"] = session
    return compact_json(obj)


def build_ack(
    request_id: int,
    ok: bool,
    kind: str,
    *,
    applied: dict[str, Any] | None = None,
    error: str | None = None,
) -> str:
    obj: dict[str, Any] = {
        "v": PROTOCOL_VERSION,
        "t": "ack",
        "id": _require_int("id", request_id),
        "ok": ok,
        "kind": kind,
    }
    if applied is not None:
        obj["applied"] = validate_params(applied, require_all=True)
    if error is not None:
        obj["error"] = error
    validate_ack(obj)
    return compact_json(obj)


def state_full_name(abbr: str) -> str:
    return STATE_MAP.get(abbr, "UNKNOWN")
