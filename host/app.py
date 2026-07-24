"""
app.py — Flask + SocketIO 上位机主程序

启动流程：
  1. 加载配置
  2. 创建调参引擎（本地规则或 GLM）
  3. 启动 TCP server 线程（监听 8888，接收小车遥测）
  4. 启动 Flask + SocketIO Web 服务（端口 5000）

Web 路由：
  GET  /           主界面
  SocketIO 事件:
    connect          浏览器连接
    telemetry        推送遥测帧到所有浏览器
    client_status    推送小车连接状态
    ai_result        推送 AI 分析结果
    send_cmd         前端发指令（start/stop/...）
    send_params      前端发参数
    ai_analyze       前端请求 AI 分析
"""
import argparse
import logging
import math
import secrets
import threading
import time
from collections import deque
from pathlib import Path
import sys
import webbrowser

from flask import Flask, render_template, request
from flask_socketio import SocketIO

from config import Config
from optimizer import Optimizer as LocalOptimizer
from protocol import PARAMS, build_auth, build_ping, clamp
from tcp_server import TcpServer, RequestManager
from tuning_engine import get_engine


def _resource_root() -> Path:
    """返回资源根目录（开发态=当前脚本目录，冻结打包态=_MEIPASS）。"""
    if getattr(sys, "frozen", False):
        return Path(getattr(sys, "_MEIPASS", Path(sys.executable).parent))
    return Path(__file__).resolve().parent


def _resource_path(*parts: str) -> str:
    """兼容源码/打包态返回资源路径。"""
    candidates = [
        _resource_root() / Path(*parts),
        _resource_root() / "host" / Path(*parts),
    ]
    for p in candidates:
        if p.exists():
            return str(p)
    return str(candidates[0])

# ============================================================
# 日志配置
# ============================================================
logging.basicConfig(
    level=logging.INFO,
    format="[%(asctime)s] %(levelname)s %(name)s: %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger("host")

# ============================================================
# Flask 应用
# ============================================================
app = Flask(
    __name__,
    template_folder=_resource_path("templates"),
    static_folder=_resource_path("static"),
    static_url_path="/static",
)
app.config["SECRET_KEY"] = secrets.token_hex(32)


socketio = SocketIO(app, async_mode=None)  # 默认仅允许同源浏览器连接

# ============================================================
# 全局状态
# ============================================================
config = Config.load()
engine = get_engine(config)

# 遥测历史（线程安全的双端队列，固定长度）
telemetry_history = deque(maxlen=config.TELEMETRY_WINDOW)
telemetry_frame_count = 0

# 当前参数（运行时可改，前端展示+下发给小车）
current_params = {k: v["default"] for k, v in PARAMS.items()}

# 小车连接状态
client_connected = False
client_addr = None

# 日志缓冲（前端展示最近 200 条）
log_buffer = deque(maxlen=200)

# 小车/ESP-01S 日志缓冲（新打开的浏览器也能看到最近日志）
car_log_buffer = deque(maxlen=500)

REQUEST_ERROR_MESSAGES = {
    "not_ready": "当前已有循迹或其他控制任务在运行，请先停止后再旋转",
    "imu_unavailable": "IMU660RB 未连接、未识别或尚未完成静止校准",
    "safety_rejected": "安全状态机拒绝执行，请检查故障状态和心跳",
    "invalid_angle": "角度无效，范围为 -180 到 180 且不能为 0",
    "lifted_confirmation_required": "电机测试必须明确确认驱动轮已经架空",
    "encoder_sign_or_stall": "编码器正转符号错误或车轮未转动",
    "path_fish_not_ready": "鱼眼循迹模块未获得连续居中的健康完整帧",
    "path_fish_stale": "鱼眼循迹模块完整帧已超时",
    "path_fish_not_centered": "鱼眼线型未居中，或极性/板载标定不正确",
    "motor_tuning_requires_idle": "电机环参数仅允许在静止或架空台架测试时修改",
    "motion_busy": "当前运动尚未结束，请停止或等待自动恢复后重试",
    "invalid cmd": "固件未识别该命令",
    "not authenticated": "认证尚未完成，请等待小车重新连接",
}

# 线程锁
state_lock = threading.Lock()


def add_log(level: str, msg: str):
    """添加日志到缓冲（前端通过 SocketIO 推送）"""
    entry = {"time": time.strftime("%H:%M:%S"), "level": level, "msg": msg}
    log_buffer.append(entry)
    socketio.emit("log", entry, namespace="/")
    logger.log(getattr(logging, level.upper(), logging.INFO), msg)


def parse_car_log_line(line: str) -> dict:
    """解析 EasyLogger 行，给前端提供结构化字段用于上色。"""
    import re
    clean = re.sub(r"(?:\x1b)?\[[0-9;]*m", "", line).strip()
    match = re.match(r"^(?P<lvl>[VDIWEA])/(?P<tag>\S+)\s+\[(?P<mcu>\d+)\]\s*(?P<msg>.*)$", clean)
    if not match:
        return {
            "line": clean,
            "level": "INFO",
            "tag": "",
            "mcu_time": "",
            "msg": clean,
        }

    level_map = {
        "V": "TRACE",
        "D": "DEBUG",
        "I": "INFO",
        "W": "WARNING",
        "E": "ERROR",
        "A": "ASSERT",
    }
    return {
        "line": clean,
        "level": level_map.get(match.group("lvl"), "INFO"),
        "tag": match.group("tag"),
        "mcu_time": match.group("mcu"),
        "msg": match.group("msg"),
    }


# ============================================================
# TCP server 回调
# ============================================================

def on_telemetry(tele: dict):
    """收到遥测帧"""
    global telemetry_frame_count
    with state_lock:
        telemetry_history.append(tele)
        telemetry_frame_count += 1
        frame_no = telemetry_frame_count
        # 同步当前参数（小车端 kp/kd 可能因其他方式改变）
        if "kp" in tele:
            current_params["KP"] = tele["kp"]
        if "kd" in tele:
            current_params["KD"] = tele["kd"]
    if frame_no <= 10 or frame_no % 20 == 0:
        logger.info("telemetry #%d: st=%s le=%s kp=%s kd=%s",
                    frame_no, tele.get("st"), tele.get("le"),
                    tele.get("kp"), tele.get("kd"))
    # 推送到所有浏览器客户端
    socketio.emit("telemetry", tele, namespace="/")
    if {"tg", "ta", "te", "rr"}.issubset(tele):
        socketio.emit("rotate_status", {
            "state": "running" if tele.get("st") == "RT" else "terminal",
            "target_deg": tele["tg"],
            "actual_deg": tele["ta"],
            "error_deg": tele["te"],
            "result": tele["rr"],
        })
    if {"hh", "ht", "ha", "he", "hr"}.issubset(tele):
        socketio.emit("heading_hold_status", {
            "active": tele["hh"],
            "target_deg": tele["ht"],
            "actual_deg": tele["ha"],
            "error_deg": tele["he"],
            "result": tele["hr"],
        })
    # 转发给优化器（若活跃）
    if optimizer is not None:
        optimizer.on_telemetry(tele)


def on_connect(addr):
    """小车连接"""
    global client_connected, client_addr
    with state_lock:
        client_connected = True
        client_addr = addr
    add_log("INFO", f"小车连接: {addr[0]}:{addr[1]}")
    socketio.emit("client_status", {"connected": True, "addr": f"{addr[0]}:{addr[1]}"})
    auth_session = secrets.token_hex(8)
    auth_frame = build_auth(config.TUNING_AUTH_TOKEN, session=auth_session)
    threading.Thread(
        target=_send_auth_after_online,
        args=(addr, auth_frame),
        daemon=True,
    ).start()


def _send_auth_after_online(addr, auth_frame: str):
    """Let the ESP finish its first uplink before starting downlink traffic."""
    time.sleep(3.0)
    with state_lock:
        same_client = client_connected and client_addr == addr
    if not same_client:
        return
    if tcp.send(auth_frame):
        add_log("INFO", "小车调参认证已发送")
    else:
        add_log("ERROR", "小车调参认证发送失败")

    time.sleep(2.0)
    with state_lock:
        same_client = client_connected and client_addr == addr
    if same_client and tcp.send(auth_frame):
        add_log("INFO", "小车调参认证重发")


def on_disconnect():
    """小车断开"""
    global client_connected, client_addr
    with state_lock:
        client_connected = False
        client_addr = None
    add_log("WARNING", "小车断开连接")
    socketio.emit("client_status", {"connected": False, "addr": ""})


def on_car_log(line: str):
    """收到小车 elog 透传日志行（裸文本，非遥测 JSON）

    固件 elog_port_output 同时写 RTT 和 ESP 环形缓冲，
    主循环 elog_port_esp_flush 通过 TCP 发到这里。
    """
    parsed = parse_car_log_line(line)
    entry = {"time": time.strftime("%H:%M:%S"), **parsed}
    car_log_buffer.append(entry)
    logger.info("car: %s", parsed["line"])
    socketio.emit("car_log", entry)


def heartbeat_worker():
    """Send diagnostic pings without participating in vehicle motion control."""
    interval_s = config.HEARTBEAT_INTERVAL_MS / 1000.0
    was_connected = False
    heartbeat_ready_at = 0.0
    while True:
        connected = tcp.is_connected()
        now = time.monotonic()
        if connected and not was_connected:
            # Start immediately so link diagnostics become available quickly.
            heartbeat_ready_at = now
        if connected and now >= heartbeat_ready_at:
            tcp.send(build_ping(int(time.monotonic() * 1000)))
        was_connected = connected
        time.sleep(interval_s)


# ============================================================
# 创建 TCP server + 请求管理器
# ============================================================
tcp = TcpServer(
    port=config.TCP_PORT,
    on_telemetry=on_telemetry,
    on_connect=on_connect,
    on_disconnect=on_disconnect,
    on_log=on_car_log,
)


def on_request_confirmed(ack: dict):
    """ACK 确认成功 — 从 ACK 的 applied 快照更新当前参数。"""
    applied = ack.get("applied")
    if applied:
        with state_lock:
            for k, v in applied.items():
                if k in PARAMS:
                    current_params[k] = v
        socketio.emit("params_update", current_params)
        add_log("INFO", f"参数已确认: {applied}")
    else:
        add_log("INFO", f"指令已确认 (id={ack.get('id')})")
    socketio.emit("request_status", {
        "state": "confirmed",
        "id": ack.get("id"),
        "kind": ack.get("kind"),
    })
    # 转发给优化器
    if optimizer is not None:
        optimizer.on_ack_confirmed(ack)


def on_request_rejected(ack: dict):
    """ACK 拒绝 - 参数未应用，通知前端。"""
    error = ack.get("error", "unknown")
    message = REQUEST_ERROR_MESSAGES.get(error, f"小车拒绝执行（{error}）")
    add_log("ERROR", f"请求被拒绝: {message}")
    socketio.emit("request_status", {
        "state": "rejected",
        "id": ack.get("id"),
        "error": error,
        "message": message,
    })
    # 转发给优化器
    if optimizer is not None:
        optimizer.on_ack_rejected(ack)


def on_request_timeout(snapshot: dict):
    """请求超时 — 3 次尝试均未收到 ACK。"""
    kind = snapshot.get("kind", "?")
    add_log("ERROR", f"请求超时 (id={snapshot.get('id')}, kind={kind}, "
                    f"attempts={snapshot.get('attempts')})")
    socketio.emit("request_status", {
        "state": "timeout",
        "id": snapshot.get("id"),
        "kind": kind,
        "message": "小车未在重试期限内确认，请检查 ESP-01S 链路",
    })
    # 转发给优化器
    if optimizer is not None:
        optimizer.on_ack_timeout(snapshot)


# 请求管理器：单待确认请求 + 300ms 超时 + 2 次重试
req_mgr = RequestManager(
    tcp,
    on_confirmed=on_request_confirmed,
    on_rejected=on_request_rejected,
    on_timeout=on_request_timeout,
)
tcp.on_ack = req_mgr.on_ack  # 将 ACK 路由到请求管理器


# ============================================================
# 本地约束优化器
# ============================================================

def _opt_on_apply(params: dict) -> bool:
    """优化器下发候选参数 — 通过请求管理器，等待 MCU ACK。"""
    return req_mgr.send_set(params)


def _opt_on_cmd(cmd: str) -> bool:
    """优化器下发命令（start/stop）。"""
    return req_mgr.send_cmd(cmd)


def _opt_on_state(state: dict):
    """优化器状态变更 — 推送到前端。"""
    socketio.emit("optimizer_state", state)
    msg = state.get("msg", "")
    if msg:
        add_log("INFO", f"[OPT] {state.get('state')}: {msg}")


def _opt_on_candidate(record):
    """候选完成 — 推送到前端。"""
    socketio.emit("optimizer_candidate", {
        "index": record.index,
        "params": record.params,
        "accepted": record.accepted,
        "abort_reason": record.abort_reason,
    })


optimizer = LocalOptimizer(
    base_params=current_params,
    target_laps=5,
    on_apply_params=_opt_on_apply,
    on_send_cmd=_opt_on_cmd,
    on_state_change=_opt_on_state,
    on_candidate_complete=_opt_on_candidate,
)


# ============================================================
# Web 路由
# ============================================================

@app.route("/")
def index():
    return render_template(
        "index.html",
        params=PARAMS,
        current=current_params,
        config=config.as_dict(),
    )


# ============================================================
# SocketIO 事件
# ============================================================

@socketio.on("connect")
def ws_connect():
    """浏览器连接，推送当前状态"""
    sid = request.sid
    socketio.emit("client_status", {
        "connected": client_connected,
        "addr": f"{client_addr[0]}:{client_addr[1]}" if client_addr else "",
    }, room=sid)
    socketio.emit("params_update", current_params, room=sid)
    # 推送历史日志
    for entry in list(log_buffer)[-5:]:
        socketio.emit("log", entry, room=sid)
    # Keep namespace setup bounded.  Replaying hundreds of individual
    # Socket.IO events can exceed the client's connect timeout after the car
    # has been online for a while; the remaining history is still available
    # in the server-side ring and new logs continue streaming live.
    for entry in list(car_log_buffer)[-5:]:
        socketio.emit("car_log", entry, room=sid)


@socketio.on("send_cmd")
def ws_send_cmd(data):
    """前端发指令：data = {"cmd": "start"}"""
    cmd = data.get("cmd", "")
    try:
        if req_mgr.is_busy():
            if cmd == "stop" and req_mgr.send_stop_override():
                add_log("WARNING", "停止指令已替换待确认请求")
                socketio.emit("request_status", {"state": "pending", "cmd": "stop"})
                return
            add_log("WARNING", "上一个请求待确认，请稍后")
            return
        if req_mgr.send_cmd(cmd):
            add_log("INFO", f"指令已发送(待确认): {cmd}")
            socketio.emit("request_status", {"state": "pending", "cmd": cmd})
        else:
            add_log("ERROR", f"指令发送失败（小车未连接）: {cmd}")
    except ValueError as e:
        add_log("ERROR", f"非法指令: {e}")


@socketio.on("send_rotate")
def ws_send_rotate(data):
    """前端发相对角度原地旋转：正数左转，负数右转。"""
    degrees = (data or {}).get("deg")
    if isinstance(degrees, bool) or not isinstance(degrees, (int, float)):
        add_log("ERROR", "旋转角度必须是数字")
        socketio.emit("rotate_status", {"state": "rejected", "result": "invalid_angle"})
        return
    degrees = float(degrees)
    if not math.isfinite(degrees) or degrees == 0.0 or abs(degrees) > 180.0:
        add_log("ERROR", "旋转角度必须在 -180 到 180 之间且不能为 0")
        socketio.emit("rotate_status", {"state": "rejected", "result": "invalid_angle"})
        return
    if req_mgr.is_busy():
        add_log("WARNING", "上一个请求待确认，请稍后")
        socketio.emit("rotate_status", {"state": "rejected", "result": "request_busy"})
        return
    if req_mgr.send_rotate(degrees):
        add_log("INFO", f"旋转指令已发送(待确认): {degrees:.1f}°")
        socketio.emit("request_status", {"state": "pending", "cmd": "rotate", "deg": degrees})
    else:
        add_log("ERROR", "旋转指令发送失败（小车未连接）")
        socketio.emit("rotate_status", {"state": "rejected", "result": "not_connected"})


@socketio.on("send_motor_test")
def ws_send_motor_test(data):
    """启动固定分档的架空电机测试；浏览器必须明确确认 lifted。"""
    if not isinstance(data, dict) or data.get("lifted") is not True:
        add_log("ERROR", "电机测试已拒绝：未确认驱动轮架空")
        socketio.emit("request_status", {
            "state": "rejected",
            "error": "lifted_confirmation_required",
        })
        return
    if req_mgr.is_busy():
        add_log("WARNING", "上一个请求待确认，请稍后")
        socketio.emit("request_status", {
            "state": "rejected",
            "error": "request_busy",
        })
        return
    if req_mgr.send_motor_test():
        add_log("WARNING", "架空电机测试已发送：开环符号检查后运行闭环20/30/40档")
        socketio.emit("request_status", {"state": "pending", "cmd": "motor_test"})
    else:
        add_log("ERROR", "电机测试发送失败（小车未连接）")
        socketio.emit("request_status", {
            "state": "rejected",
            "error": "not_connected",
        })


@socketio.on("send_params")
def ws_send_params(data):
    """前端发参数：data = {"KP": 6.0, "KD": 1.5}

    参数在 MCU ACK 确认后才更新到 current_params（on_request_confirmed）。
    发送期间不预先更新本地参数，确保 ACK 前不得更新已应用参数。
    """
    if not data:
        return
    try:
        if req_mgr.is_busy():
            add_log("WARNING", "上一个请求待确认，请稍后")
            return
        if req_mgr.send_set(data):
            add_log("INFO", f"参数已发送(待确认): {data}")
            socketio.emit("request_status", {"state": "pending", "params": data})
        else:
            add_log("ERROR", f"参数发送失败（小车未连接）: {data}")
    except Exception as e:
        add_log("ERROR", f"参数发送异常: {e}")


@socketio.on("ai_analyze")
def ws_ai_analyze():
    """前端请求 AI 分析"""
    with state_lock:
        if not telemetry_history:
            socketio.emit("ai_result", {"error": "无遥测数据"})
            return
        history = list(telemetry_history)[-config.AI_ANALYZE_FRAMES:]
        params_copy = current_params.copy()

    add_log("INFO", f"AI 分析中（{len(history)} 帧，引擎: {type(engine).__name__}）...")
    try:
        suggestion = engine.analyze(history, params_copy)
        socketio.emit("ai_result", suggestion)
        add_log("INFO", f"AI 建议: {suggestion.get('params', {})} "
                        f"置信度: {suggestion.get('confidence', 0):.2f}")
    except Exception as e:
        add_log("ERROR", f"AI 分析失败: {e}")
        socketio.emit("ai_result", {"error": str(e)})


@socketio.on("get_status")
def ws_get_status():
    """前端查询状态"""
    opt_state = optimizer.get_state()
    socketio.emit("status", {
        "client_connected": client_connected,
        "client_addr": f"{client_addr[0]}:{client_addr[1]}" if client_addr else "",
        "frame_count": tcp.get_frame_count(),
        "history_len": len(telemetry_history),
        "engine": type(engine).__name__,
        "auto_tuning": auto_tuner.active,
        "auto_tuning_rounds": auto_tuner.rounds,
        "request_pending": req_mgr.is_busy(),
        "request_pending_id": req_mgr.get_pending_id(),
        "optimizer_state": opt_state["state"],
        "optimizer_candidates": opt_state["candidates_generated"],
        "optimizer_max_candidates": opt_state["max_candidates"],
        "optimizer_best": opt_state["best_params"],
    })


# ============================================================
# 自动调参闭环模式
# ============================================================

class AutoTuner:
    """自动调参闭环：每 interval 秒自动分析+下发，达到停止条件自动停止

    停止条件：
      1. AI 置信度 < min_confidence（说明已无需调整）
      2. 连续 max_rounds 轮参数变化 < convergence_delta（参数收敛）
      3. 达到 max_rounds 上限
      4. 用户手动停止
    """

    def __init__(self):
        self.active = False
        self.thread = None
        self.rounds = 0
        self.interval = 5.0            # 每轮间隔（秒）
        self.min_confidence = 0.4      # 低于此置信度自动停止
        self.max_rounds = 10           # 最多迭代轮数
        self.convergence_delta = 0.05  # 参数变化幅度小于此值视为收敛
        self.history_snapshots = []    # 每轮参数快照（前端可视化）

    def start(self, interval=None, max_rounds=None):
        if self.active:
            return False, "自动调参已在运行"
        self.active = True
        self.rounds = 0
        self.history_snapshots = []
        if interval: self.interval = interval
        if max_rounds: self.max_rounds = max_rounds
        self.thread = threading.Thread(target=self._loop, daemon=True, name="auto-tuner")
        self.thread.start()
        return True, f"自动调参启动（间隔{self.interval}s，最多{self.max_rounds}轮）"

    def stop(self):
        if not self.active:
            return False, "自动调参未运行"
        self.active = False
        return True, "自动调参已停止"

    def _loop(self):
        add_log("INFO", f"[AUTO] 自动调参启动（间隔{self.interval}s）")
        socketio.emit("auto_tuning_status", {"active": True, "rounds": 0})

        while self.active and self.rounds < self.max_rounds:
            self.rounds += 1
            # 1. 分析
            with state_lock:
                if len(telemetry_history) < 20:
                    add_log("WARNING", f"[AUTO] 第{self.rounds}轮: 数据不足，等待...")
                    socketio.emit("auto_tuning_status", {
                        "active": True, "rounds": self.rounds, "status": "waiting_data"
                    })
                    time.sleep(self.interval)
                    continue
                history = list(telemetry_history)[-config.AI_ANALYZE_FRAMES:]
                params_before = current_params.copy()

            try:
                suggestion = engine.analyze(history, params_before)
            except Exception as e:
                add_log("ERROR", f"[AUTO] 第{self.rounds}轮分析失败: {e}")
                time.sleep(self.interval)
                continue

            params_suggested = suggestion.get("params", {})
            confidence = suggestion.get("confidence", 0)

            # 记录快照
            snapshot = {
                "round": self.rounds,
                "params_before": params_before.copy(),
                "params_suggested": params_suggested.copy(),
                "confidence": confidence,
                "reason": suggestion.get("reason", ""),
            }
            self.history_snapshots.append(snapshot)
            socketio.emit("auto_tuning_round", snapshot)

            # 2. 停止条件检查
            if not params_suggested:
                add_log("INFO", f"[AUTO] 第{self.rounds}轮: 无需调整，自动停止")
                add_log("INFO", f"[AUTO] 原因: {suggestion.get('reason','')}")
                break

            if confidence < self.min_confidence:
                add_log("INFO", f"[AUTO] 第{self.rounds}轮: 置信度{confidence:.2f}<{self.min_confidence}，自动停止")
                break

            # 3. 下发参数（通过请求管理器，等待 ACK 确认后才更新 current_params）
            if req_mgr.is_busy():
                add_log("WARNING", f"[AUTO] 第{self.rounds}轮: 请求管理器忙，跳过本轮")
                time.sleep(self.interval)
                continue
            if not req_mgr.send_set(params_suggested):
                add_log("ERROR", f"[AUTO] 第{self.rounds}轮: 下发失败（小车未连接），停止")
                break
            # 等待 ACK 确认（最多 1.2s = 3×300ms + 余量）
            ack_waited = 0.0
            while self.active and req_mgr.is_busy() and ack_waited < 1.2:
                time.sleep(0.05)
                ack_waited += 0.05
            if req_mgr.is_busy():
                add_log("ERROR", f"[AUTO] 第{self.rounds}轮: ACK 超时，停止")
                break
            add_log("INFO", f"[AUTO] 第{self.rounds}轮: 下发 {params_suggested} "
                            f"(置信度{confidence:.2f})")

            # 4. 收敛检测
            if self.rounds > 1 and self.history_snapshots:
                prev = self.history_snapshots[-2]["params_suggested"] if len(self.history_snapshots) >= 2 else {}
                # 计算参数变化幅度
                all_keys = set(prev.keys()) | set(params_suggested.keys())
                if all_keys:
                    max_delta = max(
                        abs(params_suggested.get(k, 0) - prev.get(k, 0))
                        for k in all_keys
                    )
                    if max_delta < self.convergence_delta:
                        add_log("INFO", f"[AUTO] 第{self.rounds}轮: 参数变化{max_delta:.3f}<{self.convergence_delta}，收敛停止")
                        break

            # 5. 等待下一轮（让新参数生效并收集遥测）
            socketio.emit("auto_tuning_status", {
                "active": True, "rounds": self.rounds, "status": "waiting_next"
            })
            # 分段 sleep 以便快速响应 stop
            slept = 0
            while self.active and slept < self.interval:
                time.sleep(0.5)
                slept += 0.5

        self.active = False
        add_log("INFO", f"[AUTO] 自动调参结束（共{self.rounds}轮）")
        socketio.emit("auto_tuning_status", {
            "active": False, "rounds": self.rounds,
            "snapshots": self.history_snapshots,
        })


auto_tuner = AutoTuner()


@socketio.on("auto_tuning_start")
def ws_auto_tuning_start(data):
    """启动自动调参：data = {"interval": 5.0, "max_rounds": 10}"""
    if not client_connected:
        add_log("ERROR", "[AUTO] 小车未连接，无法启动自动调参")
        return
    interval = data.get("interval", 5.0) if data else 5.0
    max_rounds = data.get("max_rounds", 10) if data else 10
    ok, msg = auto_tuner.start(interval=interval, max_rounds=max_rounds)
    add_log("INFO" if ok else "WARNING", msg)


@socketio.on("auto_tuning_stop")
def ws_auto_tuning_stop():
    """停止自动调参"""
    ok, msg = auto_tuner.stop()
    add_log("INFO" if ok else "WARNING", msg)


# ============================================================
# 本地约束优化器事件
# ============================================================

@socketio.on("optimizer_start")
def ws_optimizer_start(data):
    """启动优化器：data = {"target_laps": 5}"""
    if not client_connected:
        add_log("ERROR", "[OPT] 小车未连接，无法启动优化器")
        return
    target_laps = data.get("target_laps", 5) if data else 5
    optimizer._target_laps = int(target_laps)
    if optimizer.start():
        add_log("INFO", f"[OPT] 优化器启动（目标 {target_laps} 圈）")
    else:
        add_log("WARNING", "[OPT] 优化器已在运行")


@socketio.on("optimizer_stop")
def ws_optimizer_stop():
    """停止优化器"""
    optimizer.stop()
    add_log("INFO", "[OPT] 优化器已停止")


@socketio.on("optimizer_confirm")
def ws_optimizer_confirm():
    """人工物理确认通过 — 下发候选参数"""
    optimizer.confirm_manual()
    add_log("INFO", "[OPT] 人工确认已通过")


@socketio.on("optimizer_status")
def ws_optimizer_status():
    """查询优化器状态"""
    socketio.emit("optimizer_state", optimizer.get_state())


@socketio.on("optimizer_export")
def ws_optimizer_export():
    """导出候选报告"""
    report = optimizer.export_report()
    socketio.emit("optimizer_report", report)
    add_log("INFO", f"[OPT] 报告导出: {report['candidates_generated']} 个候选")


@socketio.on("optimizer_export_race")
def ws_optimizer_export_race():
    """导出最终 user_tuning.h 比赛档内容（人工审核后烧录 RACE）。"""
    from optimizer import generate_race_header
    template_path = _resource_path("..", "APP", "Inc", "user_tuning.h")
    try:
        with open(template_path, "r", encoding="utf-8") as f:
            template_text = f.read()
    except OSError as e:
        add_log("ERROR", f"[OPT] 读取 user_tuning.h 失败: {e}")
        return
    best = optimizer.export_report()["best_params"]
    race_header = generate_race_header(best, template_text)
    socketio.emit("optimizer_race_header", {
        "best_params": best,
        "content": race_header,
    })
    add_log("INFO", f"[OPT] 比赛档已生成（最佳参数: {best}），请人工审核后烧录 RACE")


# ============================================================
# 主入口
# ============================================================


def run_host(no_browser: bool = False, web_port: int | None = None,
             web_host: str | None = None):
    """启动上位机主循环（给 exe 入口复用）。"""
    if web_port is not None:
        config.WEB_PORT = web_port
    if web_host is not None:
        config.WEB_HOST = web_host

    tcp.start()
    threading.Thread(target=heartbeat_worker, daemon=True, name="tuning-heartbeat").start()
    add_log("INFO", f"TCP server 监听 :{config.TCP_PORT}")
    add_log("INFO", f"调参引擎: {type(engine).__name__}")

    url = f"http://localhost:{config.WEB_PORT}"
    print(f"\n[HOST] 上位机已启动")
    print(f"[HOST] TCP server 监听端口: {config.TCP_PORT}")
    print(f"[HOST] Web 界面: {url}")
    print(f"[HOST] 调参引擎: {type(engine).__name__}")
    print(f"[HOST] 按 Ctrl+C 退出\n")

    if not no_browser:
        threading.Thread(
            target=lambda: (time.sleep(1.0), webbrowser.open(url)),
            daemon=True,
        ).start()

    try:
        socketio.run(
            app, host=config.WEB_HOST, port=config.WEB_PORT,
            debug=False, allow_unsafe_werkzeug=True,
        )
    except KeyboardInterrupt:
        print("\n[HOST] 正在停止...")
    finally:
        tcp.stop()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Track Car 上位机")
    parser.add_argument("--no-browser", action="store_true", help="启动后不自动打开浏览器")
    parser.add_argument("--web-port", type=int, default=config.WEB_PORT, help="Web 端口")
    parser.add_argument("--web-host", default=config.WEB_HOST,
                        help="Web 监听地址；默认 127.0.0.1，仅显式设置后才开放局域网")
    args = parser.parse_args()
    run_host(no_browser=args.no_browser, web_port=args.web_port, web_host=args.web_host)
