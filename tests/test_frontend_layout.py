import re
import unittest
from pathlib import Path


CSS_PATH = Path(__file__).resolve().parents[1] / "host" / "static" / "style.css"
HOST_ROOT = CSS_PATH.parents[1]
HTML_PATH = HOST_ROOT / "templates" / "index.html"
JS_PATH = HOST_ROOT / "static" / "app.js"


class FrontendLayoutCssTests(unittest.TestCase):
    def test_motor_bench_test_requires_lifted_confirmation(self):
        html = HTML_PATH.read_text(encoding="utf-8")
        js = JS_PATH.read_text(encoding="utf-8")
        self.assertIn('id="btn-motor-test"', html)
        self.assertIn('id="motor-test-status"', html)
        self.assertIn('socket.emit("send_motor_test", { lifted: true })', js)
        self.assertIn("两个驱动轮完全架空", js)

    @classmethod
    def setUpClass(cls):
        cls.css = CSS_PATH.read_text(encoding="utf-8")

    def _media_block(self, max_width):
        marker = f"@media (max-width: {max_width}px)"
        start = self.css.index(marker)
        opening = self.css.index("{", start)
        depth = 1
        for index in range(opening + 1, len(self.css)):
            if self.css[index] == "{":
                depth += 1
            elif self.css[index] == "}":
                depth -= 1
                if depth == 0:
                    return self.css[opening + 1:index]
        self.fail(f"unterminated media block: {marker}")

    def _rule(self, css, selector):
        match = re.search(
            rf"(?ms)^\s*{re.escape(selector)}\s*\{{([^}}]*)\}}",
            css,
        )
        self.assertIsNotNone(match, f"missing {selector} rule")
        return match.group(1)

    def test_responsive_telemetry_panel_has_stable_minimum_height(self):
        desktop = self._rule(self._media_block(1180), ".telemetry-panel")
        mobile = self._rule(self._media_block(720), ".telemetry-panel")

        self.assertRegex(desktop, r"(?m)^\s*min-height:\s*560px;")
        self.assertRegex(mobile, r"(?m)^\s*min-height:\s*760px;")

    def test_mobile_line_grid_uses_two_columns(self):
        line_grid = self._rule(self._media_block(720), ".line-data-grid")

        self.assertRegex(line_grid, r"grid-template-columns:\s*repeat\(2,\s*minmax\(0,\s*1fr\)\);")

    def test_frontend_displays_raw_telemetry_without_charts(self):
        html = HTML_PATH.read_text(encoding="utf-8")
        javascript = JS_PATH.read_text(encoding="utf-8")

        self.assertNotIn("chart.umd.min.js", html)
        self.assertNotIn("<canvas", html)
        self.assertNotIn("new Chart", javascript)
        self.assertIn('id="telemetry-raw"', html)
        self.assertIn('id="tele-gz"', html)
        self.assertIn("JSON.stringify(tele, null, 2)", javascript)

    def test_web_control_defaults_to_loopback_and_same_origin(self):
        app_source = (HOST_ROOT / "app.py").read_text(encoding="utf-8")
        config_source = (HOST_ROOT / "config.py").read_text(encoding="utf-8")

        self.assertIn('WEB_HOST = "127.0.0.1"', config_source)
        self.assertIn("host=config.WEB_HOST", app_source)
        self.assertNotIn('cors_allowed_origins="*"', app_source)
        self.assertNotIn('SECRET_KEY"] = "tuning-secret"', app_source)


if __name__ == "__main__":
    unittest.main()
