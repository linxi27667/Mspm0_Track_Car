"""Verify parameter schema consistency across firmware, host protocol, and UI.

This test ensures that:
  * host/param_schema.py defaults match APP/Inc/user_tuning.h #define values
  * host/param_schema.py bounds match user_tuning.h USER_PARAM_*_MIN/MAX
  * host/protocol.py PARAMS is derived from param_schema.py
  * firmware macro names in param_schema.py exist in user_tuning.h
"""

import re
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
HOST = ROOT / "host"
if str(HOST) not in sys.path:
    sys.path.insert(0, str(HOST))

from param_schema import PARAM_SCHEMA, PARAMS as SCHEMA_PARAMS, parse_user_tuning_defaults  # noqa: E402
from protocol import PARAMS as PROTOCOL_PARAMS  # noqa: E402


USER_TUNING_PATH = ROOT / "APP" / "Inc" / "user_tuning.h"


def _load_header() -> str:
    return USER_TUNING_PATH.read_text(encoding="utf-8")


def _parse_bounds(header_text: str) -> dict[str, tuple[float, float]]:
    """Parse USER_PARAM_<NAME>_MIN/MAX pairs from user_tuning.h."""
    bounds: dict[str, tuple[float, float]] = {}
    pattern = re.compile(
        r'#define\s+USER_PARAM_([A-Z0-9]+)_(MIN|MAX)\s+([0-9]+\.?[0-9]*)f?',
    )
    raw: dict[str, dict[str, float]] = {}
    for match in pattern.finditer(header_text):
        name = match.group(1)
        kind = match.group(2)
        value = float(match.group(3))
        raw.setdefault(name, {})[kind] = value
    for name, vals in raw.items():
        if "MIN" in vals and "MAX" in vals:
            bounds[name] = (vals["MIN"], vals["MAX"])
    return bounds


class ParamSchemaDefaultsTests(unittest.TestCase):
    """Verify param_schema.py defaults match user_tuning.h."""

    @classmethod
    def setUpClass(cls):
        cls.header = _load_header()
        cls.firmware_defaults = parse_user_tuning_defaults(cls.header)

    def test_all_schema_macros_exist_in_firmware(self):
        for entry in PARAM_SCHEMA:
            with self.subTest(macro=entry["macro"]):
                self.assertIn(
                    entry["macro"],
                    self.firmware_defaults,
                    f"{entry['macro']} not found in user_tuning.h",
                )

    def test_defaults_match_firmware(self):
        for entry in PARAM_SCHEMA:
            macro = entry["macro"]
            if macro not in self.firmware_defaults:
                continue
            fw_value = self.firmware_defaults[macro]
            schema_default = float(entry["default"])
            with self.subTest(param=entry["name"], macro=macro):
                self.assertAlmostEqual(
                    fw_value,
                    schema_default,
                    places=6,
                    msg=(
                        f"{entry['name']} ({macro}): schema default={schema_default} "
                        f"!= firmware default={fw_value}"
                    ),
                )


class ParamSchemaBoundsTests(unittest.TestCase):
    """Verify param_schema.py bounds match user_tuning.h USER_PARAM_*_MIN/MAX."""

    @classmethod
    def setUpClass(cls):
        cls.header = _load_header()
        cls.firmware_bounds = _parse_bounds(cls.header)

    def test_all_schema_params_have_firmware_bounds(self):
        for entry in PARAM_SCHEMA:
            with self.subTest(param=entry["name"]):
                self.assertIn(
                    entry["name"],
                    self.firmware_bounds,
                    f"USER_PARAM_{entry['name']}_MIN/MAX not found in user_tuning.h",
                )

    def test_bounds_match_firmware(self):
        for entry in PARAM_SCHEMA:
            name = entry["name"]
            if name not in self.firmware_bounds:
                continue
            fw_min, fw_max = self.firmware_bounds[name]
            with self.subTest(param=name):
                self.assertAlmostEqual(
                    fw_min,
                    float(entry["min"]),
                    places=6,
                    msg=f"{name}: firmware MIN={fw_min} != schema min={entry['min']}",
                )
                self.assertAlmostEqual(
                    fw_max,
                    float(entry["max"]),
                    places=6,
                    msg=f"{name}: firmware MAX={fw_max} != schema max={entry['max']}",
                )


class ProtocolParamsTests(unittest.TestCase):
    """Verify protocol.py PARAMS is derived from param_schema.py."""

    def test_protocol_params_match_schema(self):
        self.assertEqual(set(PROTOCOL_PARAMS), set(SCHEMA_PARAMS))
        for name, schema_entry in SCHEMA_PARAMS.items():
            proto_entry = PROTOCOL_PARAMS[name]
            with self.subTest(param=name):
                self.assertAlmostEqual(
                    float(proto_entry["min"]),
                    float(schema_entry["min"]),
                    places=6,
                )
                self.assertAlmostEqual(
                    float(proto_entry["max"]),
                    float(schema_entry["max"]),
                    places=6,
                )
                self.assertAlmostEqual(
                    float(proto_entry["default"]),
                    float(schema_entry["default"]),
                    places=6,
                )
                self.assertAlmostEqual(
                    float(proto_entry["step"]),
                    float(schema_entry["step"]),
                    places=6,
                )
                self.assertEqual(proto_entry["desc"], schema_entry["desc"])
                self.assertEqual(
                    proto_entry.get("help", ""),
                    schema_entry.get("help", ""),
                )
                self.assertTrue(
                    schema_entry.get("help"),
                    f"{name}: help text is required for the host UI",
                )


if __name__ == "__main__":
    unittest.main()
