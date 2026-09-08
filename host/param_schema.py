"""Shared parameter schema — single source of truth for tunable parameters.

This module drives:
  * firmware range clamping (via generated constants in user_tuning.h)
  * Python protocol (host/protocol.py PARAMS is derived from here)
  * browser UI controls (min/max/step/help rendered by templates/index.html)

The default values MUST match the values in APP/Inc/user_tuning.h.
A unit test (tests/test_param_schema.py) verifies this consistency.

Motor-loop parameters are exposed for stationary TUNING builds; the firmware
rejects their modification while normal vehicle motion is active.
"""

from __future__ import annotations

from typing import Any


# Each entry maps a protocol parameter name to its firmware macro and bounds.
# Order matters: the optimizer searches only its explicit groups below.
#
# UI fields:
#   desc — short label under the parameter name
#   help — plain-Chinese explanation shown in the host panel
PARAM_SCHEMA: list[dict[str, Any]] = [
    {
        "name": "KP",
        "macro": "USER_LINE_PID_KP",
        "min": 0.0, "max": 50.0, "default": 3.5, "step": 0.1,
        "desc": "循迹比例P",
        "kind": "float",
        "help": (
            "输入：鱼眼12路中心偏差 le（正=偏右，负=偏左）。"
            "输出：转向差速量。KP 越大，偏线时纠正越猛。"
            "偏小：跟线慢、弯道切内侧；偏大：左右甩、冲出黑线。"
            "首轮建议 2.5~4.0，先只动 KP。"
        ),
    },
    {
        "name": "KI",
        "macro": "USER_LINE_PID_KI",
        "min": 0.0, "max": 5.0, "default": 0.0, "step": 0.05,
        "desc": "循迹积分I",
        "kind": "float",
        "help": (
            "把历史偏差累加，用来消除“一直偏一点”的静差。"
            "循迹一般保持 0：弯道误差变化快，积分容易饱和导致转不过弯。"
            "只有直道长期单侧偏、且 KP/KD 已稳时，才从 0.05 试加。"
        ),
    },
    {
        "name": "KD",
        "macro": "USER_LINE_PID_KD",
        "min": 0.0, "max": 20.0, "default": 1.5, "step": 0.1,
        "desc": "循迹微分D",
        "kind": "float",
        "help": (
            "看偏差变化速度：误差突然变大就提前刹车式回正，抑制左右摆。"
            "偏小：S 形摇摆；偏大：反应发黏、进弯迟钝，噪声也会被放大。"
            "经验：先定 KP，再把 KD 调到大约 KP 的 1/5~1/3。"
        ),
    },
    {
        "name": "GDAMP",
        "macro": "USER_LINE_FOLLOW_GYRO_DAMP",
        "min": 0.0, "max": 1.0, "default": 0.25, "step": 0.01,
        "desc": "陀螺仪阻尼",
        "kind": "float",
        "help": (
            "在线 PID 转向量上再减 GDAMP×陀螺仪Z轴角速度，压直道抖动。"
            "公式：最终转向 ≈ 线PID − GDAMP × gyro_z。"
            "偏小：直道左右抖；偏大：弯道发钝、过弯慢。"
            "先把 KP/KD 调顺，再微调 0.10~0.30。"
        ),
    },
    {
        "name": "VBASE",
        "macro": "USER_TRACK_BASE_SPEED",
        "min": 0, "max": 100, "default": 26, "step": 1,
        "desc": "启动基础PWM",
        "kind": "int",
        "help": (
            "启动循迹时的初始基础 PWM（百分比）。"
            "跑起来后会向 VSTR/VCUR 插值目标渐变，不是全程固定车速。"
            "太小：起步无力；太大：一启动就冲，校准/进弯都危险。"
            "当前实车默认 26，确认能压住线再继续上调。"
        ),
    },
    {
        "name": "VSTR",
        "macro": "USER_TRACK_SPEED_STRAIGHT",
        "min": 0, "max": 100, "default": 30, "step": 1,
        "desc": "直道PWM",
        "kind": "int",
        "help": (
            "鱼眼线型几乎居中（|le|≈0）时的目标基础 PWM，直道速度上限。"
            "左右轮最终 ≈ 基础PWM ± 转向量。"
            "想提速先加 VSTR；若出弯后猛冲再甩，先降 VSTR 或加 KD。"
            "一般保持 VSTR ≥ VCUR。"
        ),
    },
    {
        "name": "VCUR",
        "macro": "USER_TRACK_SPEED_CURVE",
        "min": 0, "max": 100, "default": 24, "step": 1,
        "desc": "偏线/弯道PWM",
        "kind": "int",
        "help": (
            "偏差较大时的目标基础 PWM。"
            "固件按 |le| 在 VSTR↔VCUR 之间线性插值，弯越大越慢。"
            "冲出弯道 → 降 VCUR；弯道太磨蹭 → 略加 VCUR，并确认 KP 够。"
            "首轮可先比 VSTR 低 2~6。"
        ),
    },
    {
        "name": "RATHR",
        "macro": "USER_RIGHT_ANGLE_THRESHOLD",
        "min": 5.0, "max": 10.0, "default": 6.0, "step": 1.0,
        "desc": "直角最少黑通道",
        "kind": "float",
        "help": (
            "方形拐角识别门槛：中心入口仍压线，且左侧或右侧远端形成支路，"
            "12路鱼眼总黑点数达到此整数值，且不能全黑。"
            "固件与上位机统一限制为 5~10。偏小易误判宽线，偏大易漏角。"
            "只影响拐角检测，不改变直线 PID。首轮保持 6.0。"
        ),
    },
    {
        "name": "MKP",
        "macro": "USER_MOTOR_SPEED_PID_KP",
        "min": 0.0, "max": 5.0, "default": 0.55, "step": 0.05,
        "desc": "电机速度环P",
        "kind": "float",
        "help": "仅TUNING且车辆静止/台架测试可改；应用后左右速度PID同时复位。",
    },
    {
        "name": "MKI",
        "macro": "USER_MOTOR_SPEED_PID_KI",
        "min": 0.0, "max": 1.0, "default": 0.0, "step": 0.01,
        "desc": "电机速度环I",
        "kind": "float",
        "help": "速度环积分，通常保持0，避免起停积分饱和；只允许静止调节。",
    },
    {
        "name": "MKD",
        "macro": "USER_MOTOR_SPEED_PID_KD",
        "min": 0.0, "max": 5.0, "default": 0.0, "step": 0.05,
        "desc": "电机速度环D",
        "kind": "float",
        "help": "速度变化阻尼；编码器量化噪声较大时不要过高，只允许静止调节。",
    },
    {
        "name": "MSL",
        "macro": "USER_MOTOR_SPEED_SCALE_LEFT",
        "min": 0.01, "max": 2.0, "default": 0.367, "step": 0.001,
        "desc": "左电机前馈比例",
        "kind": "float",
        "help": "左轮编码器计数/周期对应1%PWM的比例尺；架空车轮标定后填写。",
    },
    {
        "name": "MSR",
        "macro": "USER_MOTOR_SPEED_SCALE_RIGHT",
        "min": 0.01, "max": 2.0, "default": 0.375, "step": 0.001,
        "desc": "右电机前馈比例",
        "kind": "float",
        "help": "右轮独立比例尺；架空车轮标定后填写，不与左轮共用。",
    },
]

# Dict view for consumers that need O(1) lookup by name.
PARAMS: dict[str, dict[str, Any]] = {p["name"]: p for p in PARAM_SCHEMA}

# Telemetry field abbreviation -> protocol parameter name.
TELEMETRY_PARAM_FIELDS: dict[str, str] = {
    "kp": "KP",
    "ki": "KI",
    "kd": "KD",
    "gd": "GDAMP",
    "vb": "VBASE",
    "vs": "VSTR",
    "vc": "VCUR",
    "rt": "RATHR",
}

# Parameter groups for the local optimizer (search order).
OPTIMIZER_GROUP_GAINS = ["KP", "KD", "GDAMP", "RATHR"]
OPTIMIZER_GROUP_SPEED = ["VBASE", "VSTR", "VCUR"]
# Parameters the optimizer must never touch automatically.
OPTIMIZER_LOCKED = ["KI", "MKP", "MKI", "MKD", "MSL", "MSR"]


def parse_user_tuning_defaults(header_text: str) -> dict[str, float]:
    """Parse #define values from user_tuning.h text.

    Returns a {macro: value} dict for all USER_* macros found.
    """
    import re
    defaults: dict[str, float] = {}
    pattern = re.compile(
        r'^\s*#define\s+(USER_[A-Z0-9_]+)\s+([0-9]+\.?[0-9]*)f?\s*(?:/\*.*)?$',
        re.MULTILINE,
    )
    for match in pattern.finditer(header_text):
        macro = match.group(1)
        value = float(match.group(2))
        defaults[macro] = value
    return defaults
