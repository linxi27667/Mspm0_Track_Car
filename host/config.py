"""
config.py — 上位机配置加载

默认值可被 host_config.json 覆盖（现场修改不重启代码）。
host_config.json 示例：
{
  "TCP_PORT": 8888,
  "WEB_HOST": "127.0.0.1",
  "WEB_PORT": 5000,
  "GLM_API_KEY": "your_zhipuai_api_key_here"
}
"""
import os
import json
import logging

logger = logging.getLogger("config")


class Config:
    TUNING_AUTH_TOKEN = "track_car_2025"
    # Diagnostic ping cadence; missed heartbeats never alter vehicle motion.
    HEARTBEAT_INTERVAL_MS = 250

    # TCP server 监听端口（与 app_esp01s_cfg.h ESP01S_SERVER_PORT 一致）
    TCP_PORT = 8888

    # Web 界面端口
    WEB_HOST = "127.0.0.1"
    WEB_PORT = 5000

    # GLM API key（空则用本地规则引擎，配置后启用 GLM）
    GLM_API_KEY = ""

    # GLM 模型名（glm-4-flash 免费且快，glm-4 更准但收费）
    GLM_MODEL = "glm-4-flash"

    # 遥测历史窗口（500 帧 = 5 秒 @100Hz）
    TELEMETRY_WINDOW = 500

    # AI 分析取最近 N 帧（100 帧 = 1 秒）
    AI_ANALYZE_FRAMES = 100

    @classmethod
    def load(cls):
        """从 host_config.json 加载配置（覆盖默认值）

        Returns:
            Config 实例
        """
        cfg_path = os.path.join(os.path.dirname(__file__), "host_config.json")
        if os.path.exists(cfg_path):
            try:
                with open(cfg_path, "r", encoding="utf-8") as f:
                    data = json.load(f)
                for k, v in data.items():
                    if hasattr(cls, k):
                        setattr(cls, k, v)
                logger.info("config loaded from %s", cfg_path)
            except (json.JSONDecodeError, OSError) as e:
                logger.warning("config load failed: %s, using defaults", e)
        else:
            logger.info("no host_config.json, using defaults")
        return cls()

    def as_dict(self):
        """返回配置字典（前端展示用）"""
        return {
            "TCP_PORT": self.TCP_PORT,
            "WEB_HOST": self.WEB_HOST,
            "WEB_PORT": self.WEB_PORT,
            "GLM_API_KEY": "***" if self.GLM_API_KEY else "",
            "GLM_MODEL": self.GLM_MODEL,
            "TELEMETRY_WINDOW": self.TELEMETRY_WINDOW,
            "AI_ANALYZE_FRAMES": self.AI_ANALYZE_FRAMES,
        }
