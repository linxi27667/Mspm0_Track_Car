"""
tuning_engine.py — 调参引擎（本地规则 + GLM API 双后端）

接口：
  engine.analyze(history, current_params) -> suggestion dict
  suggestion = {
      "params": {"KP": 6.0, ...},   # 建议参数（空字典=无需调整）
      "reason": "...",               # 自然语言解释
      "confidence": 0.7,             # 0~1
      "engine": "local" | "glm"      # 引擎标识
  }

本地规则引擎基于 PID 调参经验：
  - 误差震荡（频繁变号）→ KD 偏小，加大 50%
  - 误差持续偏大（|le|>2 持续 >50%）→ KP 偏小，加大 20%
  - 直道陀螺仪抖动 → GDAMP 偏小，+0.05
  - 弯道频繁丢线 → VCUR 偏大，-5
  - 状态机频繁进 RIGHT_ANGLE 但转不过 → RATHR 偏小或 KP 偏小

GLM 后端调用智谱 API，给出自然语言解释 + 参数建议。
"""
import json
import logging
import statistics
from protocol import PARAMS, clamp

logger = logging.getLogger("tuning")


# ============================================================
# 本地规则引擎
# ============================================================

class LocalRuleEngine:
    """基于 PID 调参经验的本地规则引擎

    优点：离线可用、响应快（<1ms）、可解释
    缺点：规则有限，复杂场景不如 LLM

    v2 优化：多规则组合模式
      - 所有规则独立评估，收集所有触发的建议
      - 合并 params（同一参数被多规则触发时取最激进的调整）
      - confidence 取所有触发规则的加权平均（按规则置信度加权）
      - reason 拼接所有触发规则的说明
    """

    def _check_rules(self, history, current):
        """评估所有规则，返回触发的规则列表

        每个规则返回 (params_dict, reason_str, confidence_float) 或 None
        """
        errors = [h["le"] for h in history]
        gyro_rates = [h["gz"] for h in history]
        states = [h["st"] for h in history]
        n = len(history)
        triggered = []

        # 规则1: 误差震荡（左右频繁变号）→ KD 太小
        sign_changes = sum(
            1 for i in range(1, n) if errors[i] * errors[i - 1] < 0
        )
        if sign_changes > n * 0.3:
            new_kd = clamp("KD", current.get("KD", PARAMS["KD"]["default"]) * 1.5)
            triggered.append((
                {"KD": new_kd},
                f"误差震荡明显({sign_changes}/{n}帧变号),KD偏小,建议加大50%",
                0.7,
            ))

        # 规则2: 弯道冲出（状态频繁进 LINE_LOST）→ VCUR 太大
        lost_count = sum(1 for s in states if s == "LL")
        if lost_count > n * 0.15:
            new_vcur = clamp("VCUR", current.get("VCUR", PARAMS["VCUR"]["default"]) - 5)
            triggered.append((
                {"VCUR": new_vcur},
                f"弯道频繁丢线({lost_count}/{n}帧LL状态),VCUR偏大,建议-5",
                0.6,
            ))

        # 规则3: 误差持续偏大（|le| > 2 持续 >50% 帧）→ KP 太小
        big_errors = sum(1 for e in errors if abs(e) > 2.0)
        if big_errors > n * 0.5:
            new_kp = clamp("KP", current.get("KP", PARAMS["KP"]["default"]) * 1.2)
            triggered.append((
                {"KP": new_kp},
                f"误差持续偏大({big_errors}/{n}帧|le|>2),KP偏小,建议加大20%",
                0.6,
            ))

        # 规则4: 直道抖动（gyro_rate 平均幅度过大）→ GDAMP 太小
        avg_gyro = statistics.mean(abs(g) for g in gyro_rates)
        if avg_gyro > 2.0:
            new_damp = clamp("GDAMP", current.get("GDAMP", PARAMS["GDAMP"]["default"]) + 0.05)
            triggered.append((
                {"GDAMP": new_damp},
                f"直道陀螺仪抖动明显(平均|gz|={avg_gyro:.1f}),GDAMP偏小,建议+0.05",
                0.5,
            ))

        # 规则5: 直角弯频繁触发但未完成 → RATHR 偏小
        ra_count = sum(1 for s in states if s == "RA")
        if ra_count > n * 0.15:
            new_rathr = clamp("RATHR", current.get("RATHR", PARAMS["RATHR"]["default"]) + 1.0)
            triggered.append((
                {"RATHR": new_rathr},
                f"直角弯频繁触发({ra_count}/{n}帧RA),可能误判,建议RATHR+1",
                0.4,
            ))

        return triggered, errors, gyro_rates, states, n

    def analyze(self, history: list, current: dict) -> dict:
        """分析最近 N 帧遥测，返回参数建议

        Args:
            history: 遥测历史列表，每帧含 le/po/gz/st 等字段
            current: 当前参数字典，如 {"KP": 5.0, ...}
        Returns:
            suggestion dict，含 params/reason/confidence/engine
        """
        if len(history) < 20:
            return {
                "params": {},
                "reason": f"数据不足({len(history)}/20帧),无法分析",
                "confidence": 0.2,
                "engine": "local",
            }

        triggered, errors, gyro_rates, states, n = self._check_rules(history, current)

        if not triggered:
            # 一切正常
            return {
                "params": {},
                "reason": f"遥测正常(误差均值={statistics.mean(errors):.2f},"
                          f"陀螺仪均值={statistics.mean(gyro_rates):.2f}),无需调整",
                "confidence": 0.3,
                "engine": "local",
            }

        # 合并多规则建议
        # 同一参数被多规则触发时：数值型取变化幅度最大的，置信度取最高
        merged_params = {}
        param_confidences = {}  # 记录每个参数的最高置信度
        reasons = []
        total_weight = 0
        weighted_conf = 0

        for params, reason, conf in triggered:
            reasons.append(reason)
            total_weight += conf
            weighted_conf += conf * conf  # 置信度平方加权（高置信度规则权重更大）

            for k, v in params.items():
                if k not in merged_params:
                    merged_params[k] = v
                    param_confidences[k] = conf
                else:
                    # 同一参数被多规则触发：保留变化幅度最大的
                    old_v = merged_params[k]
                    old_delta = abs(old_v - current.get(k, 0))
                    new_delta = abs(v - current.get(k, 0))
                    if new_delta > old_delta:
                        merged_params[k] = v
                        param_confidences[k] = conf

        # 合并置信度：所有触发规则的加权平均
        final_confidence = weighted_conf / total_weight if total_weight > 0 else 0.3
        # 多规则同时触发时置信度提升（最多 +0.2）
        if len(triggered) > 1:
            final_confidence = min(1.0, final_confidence + 0.1 * (len(triggered) - 1))

        return {
            "params": merged_params,
            "reason": " | ".join(reasons),
            "confidence": round(final_confidence, 2),
            "engine": "local",
            "rules_triggered": len(triggered),  # 触发规则数（调试用）
        }


# ============================================================
# GLM API 后端
# ============================================================

class GlmEngine:
    """智谱 GLM API 调参后端

    调用 glm-4-flash 模型，给出自然语言解释 + 参数建议。
    需配置 GLM_API_KEY。
    """

    @staticmethod
    def _build_param_desc() -> str:
        """从 PARAMS 动态生成参数范围说明，避免硬编码不一致。"""
        lines = []
        for name, info in PARAMS.items():
            lines.append(f"  {name}: {info['desc']}（{info['min']}~{info['max']}）")
        return "\n".join(lines)

    @property
    def SYSTEM_PROMPT(self) -> str:
        return f"""你是循迹小车 PID 调参专家。根据遥测数据给出参数调整建议。

遥测字段说明：
  line: 12路鱼眼数字黑线位（A1左到A12右，0/1）
  lm/bc/sa/se/bp: 12位mask、黑点数、完整帧龄、通信错误数、黑线极性
  gz: 陀螺仪Z轴角速度（deg/s）
  el/er: 左/右编码器速度
  le: 循迹误差（正=偏左,负=偏右,0=居中）
  po: PID转向输出（正=右转,负=左转）
  st: 状态缩写（ID=空闲,LF=循迹,RA=直角弯,LL=丢线,SL=起点,CR=十字,FN=完成）

参数说明：
{self._build_param_desc()}

调参经验：
  1. 误差震荡（le 频繁变号）→ KD 偏小，加大 50%
  2. 误差持续偏大（|le|>2）→ KP 偏小，加大 20%
  3. 直道抖动（gz 波动大）→ GDAMP 偏小，+0.05
  4. 弯道冲出（频繁 LL）→ VCUR 偏大，-5
  5. 直角弯误判 → RATHR +1

只输出 JSON，不要多余文字。格式：
{{"params":{{"KP":6.0}},"reason":"...","confidence":0.7}}
"""

    def __init__(self, api_key: str, model: str = "glm-4-flash"):
        self.api_key = api_key
        self.model = model
        self.url = "https://open.bigmodel.cn/api/paas/v4/chat/completions"
        # 延迟导入 requests，无网络环境也能加载模块
        try:
            import requests
            self._requests = requests
        except ImportError:
            logger.error("requests not installed, GLM engine unavailable")
            self._requests = None

    def analyze(self, history: list, current: dict) -> dict:
        if not self._requests:
            return {
                "params": {},
                "reason": "GLM 引擎不可用（requests 未安装），回退本地规则",
                "confidence": 0.0,
                "engine": "glm",
            }

        # 构造用户 prompt：当前参数 + 最近 50 帧遥测摘要
        prompt = self._build_prompt(history, current)

        try:
            resp = self._requests.post(
                self.url,
                headers={
                    "Authorization": f"Bearer {self.api_key}",
                    "Content-Type": "application/json",
                },
                json={
                    "model": self.model,
                    "messages": [
                        {"role": "system", "content": self.SYSTEM_PROMPT},
                        {"role": "user", "content": prompt},
                    ],
                    "temperature": 0.3,
                    "max_tokens": 300,
                },
                timeout=10,
            )
            resp.raise_for_status()
            data = resp.json()
            content = data["choices"][0]["message"]["content"]
            # 解析 GLM 返回的 JSON
            suggestion = self._parse_response(content, current)
            suggestion["engine"] = "glm"
            return suggestion
        except Exception as e:
            logger.error("GLM API error: %s", e)
            return {
                "params": {},
                "reason": f"GLM API 调用失败: {e}",
                "confidence": 0.0,
                "engine": "glm",
            }

    def _build_prompt(self, history: list, current: dict) -> str:
        """构造 GLM 用户 prompt"""
        # 取最近 50 帧摘要，避免 token 过多
        recent = history[-50:]
        # 降采样：每 5 帧取 1 帧
        sampled = recent[::5][-10:]

        # 计算统计指标
        errors = [h["le"] for h in recent]
        gyro_rates = [h["gz"] for h in recent]
        states = [h["st"] for h in recent]
        sign_changes = sum(
            1 for i in range(1, len(errors)) if errors[i] * errors[i - 1] < 0
        )

        prompt = f"""当前参数: {json.dumps(current, ensure_ascii=False)}

最近 {len(recent)} 帧遥测统计:
  误差 le: 均值={statistics.mean(errors):.2f}, 最大={max(abs(e) for e in errors):.2f}
  陀螺仪 gz: 均值={statistics.mean(gyro_rates):.2f}, 最大|gz|={max(abs(g) for g in gyro_rates):.2f}
  误差变号次数: {sign_changes}/{len(errors)}
  状态分布: {dict((s, states.count(s)) for s in set(states))}

采样数据（最近 10 帧）:
"""
        for h in sampled:
            prompt += f"  le={h['le']:.2f} po={h['po']:.2f} gz={h['gz']:.2f} st={h['st']}\n"

        prompt += "\n请分析并给出参数调整建议（只输出 JSON）。"
        return prompt

    def _parse_response(self, content: str, current: dict) -> dict:
        """解析 GLM 返回的 JSON（容错）"""
        # 尝试直接解析
        try:
            obj = json.loads(content)
            if "params" in obj:
                # 钳位建议参数
                obj["params"] = {
                    k: clamp(k, v) for k, v in obj["params"].items() if k in PARAMS
                }
                obj.setdefault("reason", "")
                obj.setdefault("confidence", 0.7)
                return obj
        except json.JSONDecodeError:
            pass

        # 尝试提取 JSON 块（GLM 可能包裹在 ```json ... ``` 中）
        # 使用平衡花括号匹配以支持嵌套 JSON 对象（正则 [^{}]* 无法匹配嵌套）
        extracted = self._extract_balanced_json(content)
        if extracted is not None:
            try:
                obj = json.loads(extracted)
                if "params" in obj:
                    obj["params"] = {
                        k: clamp(k, v) for k, v in obj["params"].items() if k in PARAMS
                    }
                    obj.setdefault("reason", content[:100])
                    obj.setdefault("confidence", 0.6)
                    return obj
            except json.JSONDecodeError:
                pass

        # 解析失败，返回原文作为 reason
        return {
            "params": {},
            "reason": f"GLM 返回无法解析: {content[:200]}",
            "confidence": 0.2,
        }

    @staticmethod
    def _extract_balanced_json(content: str):
        """从文本中提取第一个平衡的 JSON 对象（支持嵌套花括号）。

        跳过字符串字面量内的花括号。返回提取的子串或 None。
        """
        start = content.find("{")
        while start != -1:
            depth = 0
            in_str = False
            escape = False
            for i in range(start, len(content)):
                ch = content[i]
                if in_str:
                    if escape:
                        escape = False
                    elif ch == "\\":
                        escape = True
                    elif ch == '"':
                        in_str = False
                else:
                    if ch == '"':
                        in_str = True
                    elif ch == "{":
                        depth += 1
                    elif ch == "}":
                        depth -= 1
                        if depth == 0:
                            return content[start:i + 1]
            # 未找到匹配的右括号，尝试下一个 {
            start = content.find("{", start + 1)
        return None


# ============================================================
# 工厂函数
# ============================================================

def get_engine(config):
    """根据配置创建引擎

    有 GLM_API_KEY 用 GLM，否则用本地规则
    """
    if config.GLM_API_KEY:
        logger.info("using GLM engine (model: glm-4-flash)")
        return GlmEngine(config.GLM_API_KEY)
    logger.info("using local rule engine (no GLM_API_KEY)")
    return LocalRuleEngine()
