"""tuning_engine.py 单元测试 — LocalRuleEngine 和 GlmEngine。"""

import json
import sys
import unittest
from pathlib import Path
from unittest.mock import MagicMock

ROOT = Path(__file__).resolve().parents[1]
HOST = ROOT / "host"
if str(HOST) not in sys.path:
    sys.path.insert(0, str(HOST))

from param_schema import PARAMS  # noqa: E402
from tuning_engine import LocalRuleEngine, GlmEngine, get_engine  # noqa: E402


# ============================================================
# 辅助函数
# ============================================================

def make_frame(le=0.0, po=0.0, gz=0.0, st="LF"):
    """构造一帧遥测。"""
    return {"le": le, "po": po, "gz": gz, "st": st}


def make_history(n=30, le_fn=None, gz_fn=None, st_fn=None):
    """构造 n 帧遥测历史。"""
    history = []
    for i in range(n):
        le = le_fn(i) if le_fn else 0.0
        gz = gz_fn(i) if gz_fn else 0.0
        st = st_fn(i) if st_fn else "LF"
        history.append(make_frame(le=le, gz=gz, st=st))
    return history


def make_current():
    """构造当前参数字典（全部使用默认值）。"""
    return {name: info["default"] for name, info in PARAMS.items()}


# ============================================================
# LocalRuleEngine 基础测试
# ============================================================

class LocalRuleEngineBasicTests(unittest.TestCase):
    """LocalRuleEngine 基础行为测试。"""

    def test_insufficient_data_returns_low_confidence(self):
        """history < 20 帧时返回低置信度和空 params。"""
        engine = LocalRuleEngine()
        result = engine.analyze(make_history(10), make_current())
        self.assertEqual(result["engine"], "local")
        self.assertEqual(result["params"], {})
        self.assertEqual(result["confidence"], 0.2)
        self.assertIn("数据不足", result["reason"])

    def test_boundary_exactly_20_frames(self):
        """恰好 20 帧时应进行分析（非数据不足）。"""
        engine = LocalRuleEngine()
        result = engine.analyze(make_history(20), make_current())
        self.assertEqual(result["engine"], "local")
        self.assertIn("遥测正常", result["reason"])
        self.assertEqual(result["params"], {})

    def test_normal_telemetry_no_adjustment(self):
        """正常遥测（无规则触发）返回空 params 和低置信度。"""
        engine = LocalRuleEngine()
        result = engine.analyze(make_history(30), make_current())
        self.assertEqual(result["params"], {})
        self.assertEqual(result["confidence"], 0.3)
        self.assertIn("遥测正常", result["reason"])
        self.assertEqual(result["engine"], "local")


# ============================================================
# LocalRuleEngine 规则测试
# ============================================================

class LocalRuleEngineRuleTests(unittest.TestCase):
    """LocalRuleEngine 5 条调参规则独立测试。"""

    def test_rule1_oscillation_increases_kd(self):
        """规则1: 误差频繁变号 (>30% 帧) -> KD*1.5。"""
        engine = LocalRuleEngine()
        history = make_history(30, le_fn=lambda i: 1.5 if i % 2 == 0 else -1.5)
        current = make_current()
        result = engine.analyze(history, current)
        self.assertIn("KD", result["params"])
        expected_kd = PARAMS["KD"]["default"] * 1.5
        self.assertAlmostEqual(result["params"]["KD"], expected_kd, places=2)
        self.assertIn("震荡", result["reason"])

    def test_rule2_line_lost_decreases_vcur(self):
        """规则2: 弯道频繁丢线 (>15% 帧 LL) -> VCUR-5。"""
        engine = LocalRuleEngine()
        history = make_history(30, st_fn=lambda i: "LL" if i % 5 == 0 else "LF")
        current = make_current()
        result = engine.analyze(history, current)
        self.assertIn("VCUR", result["params"])
        expected_vcur = PARAMS["VCUR"]["default"] - 5
        self.assertEqual(result["params"]["VCUR"], expected_vcur)
        self.assertIn("丢线", result["reason"])

    def test_rule3_big_error_increases_kp(self):
        """规则3: 误差持续偏大 (>50% 帧 |le|>2) -> KP*1.2。"""
        engine = LocalRuleEngine()
        history = make_history(30, le_fn=lambda i: 3.0)
        current = make_current()
        result = engine.analyze(history, current)
        self.assertIn("KP", result["params"])
        expected_kp = PARAMS["KP"]["default"] * 1.2
        self.assertAlmostEqual(result["params"]["KP"], expected_kp, places=2)
        self.assertIn("误差持续偏大", result["reason"])

    def test_rule4_gyro_jitter_increases_gdamp(self):
        """规则4: 直道抖动 (avg|gz|>2.0) -> GDAMP+0.05。"""
        engine = LocalRuleEngine()
        history = make_history(30, gz_fn=lambda i: 3.0)
        current = make_current()
        result = engine.analyze(history, current)
        self.assertIn("GDAMP", result["params"])
        expected_gdamp = PARAMS["GDAMP"]["default"] + 0.05
        self.assertAlmostEqual(result["params"]["GDAMP"], expected_gdamp, places=3)
        self.assertIn("陀螺仪抖动", result["reason"])

    def test_rule5_right_angle_frequent_increases_rathr(self):
        """规则5: 直角弯频繁 (>15% 帧 RA) -> RATHR+1。"""
        engine = LocalRuleEngine()
        history = make_history(30, st_fn=lambda i: "RA" if i % 5 == 0 else "LF")
        current = make_current()
        result = engine.analyze(history, current)
        self.assertIn("RATHR", result["params"])
        expected_rathr = PARAMS["RATHR"]["default"] + 1.0
        self.assertAlmostEqual(result["params"]["RATHR"], expected_rathr, places=2)
        self.assertIn("直角弯", result["reason"])


# ============================================================
# LocalRuleEngine 多规则组合测试
# ============================================================

class LocalRuleEngineMultiRuleTests(unittest.TestCase):
    """LocalRuleEngine 多规则同时触发和合并行为。"""

    def test_multiple_rules_triggered(self):
        """多规则同时触发时合并 params 和 reason。"""
        engine = LocalRuleEngine()
        history = make_history(
            30,
            le_fn=lambda i: 3.0 if i % 2 == 0 else -3.0,
            gz_fn=lambda i: 3.0,
        )
        result = engine.analyze(history, make_current())
        self.assertIn("KD", result["params"])
        self.assertIn("KP", result["params"])
        self.assertIn("GDAMP", result["params"])
        self.assertGreaterEqual(result.get("rules_triggered", 0), 2)

    def test_multi_rule_confidence_boost(self):
        """多规则触发时置信度应比单规则更高。"""
        engine = LocalRuleEngine()
        single = engine.analyze(
            make_history(30, gz_fn=lambda i: 3.0),
            make_current(),
        )
        multi = engine.analyze(
            make_history(
                30,
                le_fn=lambda i: 3.0 if i % 2 == 0 else -3.0,
                gz_fn=lambda i: 3.0,
            ),
            make_current(),
        )
        self.assertGreater(multi["confidence"], single["confidence"])

    def test_clamp_applied_to_all_suggestions(self):
        """所有建议参数都应被钳位到合法范围。"""
        engine = LocalRuleEngine()
        history = make_history(30, le_fn=lambda i: 3.0)
        current = make_current()
        current["KP"] = 45.0  # 45*1.2=54 > max=50
        result = engine.analyze(history, current)
        self.assertLessEqual(result["params"]["KP"], PARAMS["KP"]["max"])
        self.assertGreaterEqual(result["params"]["KP"], PARAMS["KP"]["min"])


# ============================================================
# LocalRuleEngine 边缘路径测试
# ============================================================

class LocalRuleEngineEdgeTests(unittest.TestCase):
    """LocalRuleEngine 边缘路径和防御性代码。"""

    def test_empty_history(self):
        """空 history 返回数据不足。"""
        engine = LocalRuleEngine()
        result = engine.analyze([], make_current())
        self.assertEqual(result["confidence"], 0.2)
        self.assertIn("数据不足", result["reason"])

    def test_current_missing_params_uses_defaults(self):
        """current 缺少某些参数时使用 PARAMS 默认值。"""
        engine = LocalRuleEngine()
        history = make_history(30, le_fn=lambda i: 1.5 if i % 2 == 0 else -1.5)
        result = engine.analyze(history, {"KP": 5.0})
        self.assertIn("KD", result["params"])
        expected = PARAMS["KD"]["default"] * 1.5
        self.assertAlmostEqual(result["params"]["KD"], expected, places=2)

    def test_no_rules_triggered_below_threshold(self):
        """规则条件不满足时不触发。"""
        engine = LocalRuleEngine()
        history = make_history(30, le_fn=lambda i: 0.1, gz_fn=lambda i: 0.1)
        result = engine.analyze(history, make_current())
        self.assertEqual(result["params"], {})
        self.assertIn("遥测正常", result["reason"])


# ============================================================
# GlmEngine 测试（mock requests）
# ============================================================

class GlmEngineTests(unittest.TestCase):
    """GlmEngine API 调用和响应解析测试。"""

    def test_requests_not_available(self):
        """requests 未安装时返回不可用。"""
        engine = GlmEngine("fake-key")
        engine._requests = None
        result = engine.analyze(make_history(30), make_current())
        self.assertEqual(result["engine"], "glm")
        self.assertEqual(result["confidence"], 0.0)
        self.assertIn("不可用", result["reason"])

    def test_parse_response_direct_json(self):
        """直接 JSON 解析成功。"""
        engine = GlmEngine("fake-key")
        content = json.dumps({
            "params": {"KP": 6.0},
            "reason": "误差偏大",
            "confidence": 0.8,
        })
        result = engine._parse_response(content, make_current())
        self.assertEqual(result["params"]["KP"], 6.0)
        self.assertEqual(result["reason"], "误差偏大")

    def test_parse_response_clamps_params(self):
        """返回参数被钳位到合法范围。"""
        engine = GlmEngine("fake-key")
        content = json.dumps({
            "params": {"KP": 999.0, "KD": -10.0},
            "reason": "test",
            "confidence": 0.5,
        })
        result = engine._parse_response(content, make_current())
        self.assertEqual(result["params"]["KP"], PARAMS["KP"]["max"])
        self.assertEqual(result["params"]["KD"], PARAMS["KD"]["min"])

    def test_parse_response_filters_unknown_keys(self):
        """未知参数 key 被过滤。"""
        engine = GlmEngine("fake-key")
        content = json.dumps({
            "params": {"KP": 6.0, "UNKNOWN": 123},
            "reason": "test",
            "confidence": 0.5,
        })
        result = engine._parse_response(content, make_current())
        self.assertIn("KP", result["params"])
        self.assertNotIn("UNKNOWN", result["params"])

    def test_parse_response_json_in_codeblock(self):
        """JSON 包裹在 ```json``` 中。"""
        engine = GlmEngine("fake-key")
        content = '```json\n{"params": {"KP": 7.0}, "reason": "test"}\n```'
        result = engine._parse_response(content, make_current())
        self.assertEqual(result["params"]["KP"], 7.0)

    def test_parse_response_garbage_returns_low_confidence(self):
        """完全无法解析时返回低置信度。"""
        engine = GlmEngine("fake-key")
        content = "这不是 JSON，只是普通文本"
        result = engine._parse_response(content, make_current())
        self.assertEqual(result["params"], {})
        self.assertEqual(result["confidence"], 0.2)
        self.assertIn("无法解析", result["reason"])

    def test_build_param_desc_contains_all_params(self):
        """_build_param_desc 包含所有 8 个参数。"""
        desc = GlmEngine._build_param_desc()
        for name in PARAMS:
            self.assertIn(name, desc)

    def test_system_prompt_contains_param_desc(self):
        """SYSTEM_PROMPT 包含参数说明。"""
        engine = GlmEngine("fake-key")
        prompt = engine.SYSTEM_PROMPT
        self.assertIn("KP", prompt)
        self.assertIn("VCUR", prompt)
        self.assertIn("调参经验", prompt)

    def test_build_prompt_contains_stats(self):
        """_build_prompt 包含统计指标和采样数据。"""
        engine = GlmEngine("fake-key")
        history = make_history(30, le_fn=lambda i: 1.0, gz_fn=lambda i: 0.5)
        prompt = engine._build_prompt(history, make_current())
        self.assertIn("当前参数", prompt)
        self.assertIn("误差 le", prompt)
        self.assertIn("陀螺仪 gz", prompt)
        self.assertIn("状态分布", prompt)
        self.assertIn("采样数据", prompt)

    def test_analyze_api_success(self):
        """API 调用成功时返回解析后的建议。"""
        engine = GlmEngine("fake-key")
        mock_resp = MagicMock()
        mock_resp.json.return_value = {
            "choices": [{"message": {"content": json.dumps({
                "params": {"KP": 7.0},
                "reason": "test success",
                "confidence": 0.9,
            })}}]
        }
        mock_resp.raise_for_status = MagicMock()
        engine._requests = MagicMock()
        engine._requests.post.return_value = mock_resp

        result = engine.analyze(make_history(30), make_current())
        self.assertEqual(result["engine"], "glm")
        self.assertEqual(result["params"]["KP"], 7.0)
        self.assertEqual(result["reason"], "test success")

    def test_analyze_api_exception(self):
        """API 异常时返回错误信息。"""
        engine = GlmEngine("fake-key")
        engine._requests = MagicMock()
        engine._requests.post.side_effect = ConnectionError("network error")

        result = engine.analyze(make_history(30), make_current())
        self.assertEqual(result["engine"], "glm")
        self.assertEqual(result["confidence"], 0.0)
        self.assertIn("API 调用失败", result["reason"])


# ============================================================
# get_engine 工厂函数测试
# ============================================================

class GetEngineTests(unittest.TestCase):
    """get_engine 工厂函数测试。"""

    def test_returns_local_when_no_key(self):
        """无 GLM_API_KEY 时返回 LocalRuleEngine。"""
        config = MagicMock()
        config.GLM_API_KEY = ""
        engine = get_engine(config)
        self.assertIsInstance(engine, LocalRuleEngine)

    def test_returns_glm_when_key_present(self):
        """有 GLM_API_KEY 时返回 GlmEngine。"""
        config = MagicMock()
        config.GLM_API_KEY = "test-key"
        engine = get_engine(config)
        self.assertIsInstance(engine, GlmEngine)


if __name__ == "__main__":
    unittest.main()
