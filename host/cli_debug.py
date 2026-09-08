"""CLI 调参客户端 — 通过上位机 SocketIO 下发指令/参数并监听遥测。

用法:
  python host/cli_debug.py status                 # 查询当前状态
  python host/cli_debug.py params KP=6.0 KD=1.5   # 下发参数
  python host/cli_debug.py cmd start               # 下发指令
  python host/cli_debug.py cmd stop
  python host/cli_debug.py rotate 90               # 旋转 90 度
  python host/cli_debug.py motor_test               # 架空轮子电机测试
  python host/cli_debug.py tele 5                  # 监听 5 秒遥测
"""
import sys
import time
import socketio

URL = "http://127.0.0.1:5000"


def _parse_params(args):
    params = {}
    for a in args:
        if "=" not in a:
            raise ValueError(f"参数格式错误，应为 NAME=VALUE: {a}")
        k, v = a.split("=", 1)
        k = k.strip().upper()
        try:
            val = float(v) if "." in v or "e" in v.lower() else int(v)
        except ValueError:
            val = float(v)
        params[k] = val
    return params


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return
    action = sys.argv[1]
    sio = socketio.Client()

    tele_buf = []
    log_buf = []

    @sio.event
    def connect():
        pass

    @sio.on("status")
    def on_status(data):
        print("[STATUS]", _fmt(data))

    @sio.on("telemetry")
    def on_tele(data):
        line = (f"tele: t={data.get('t')} st={data.get('st')} le={data.get('le')} "
                f"po={data.get('po')} gz={data.get('gz')} el={data.get('el')} er={data.get('er')} "
                f"lp={data.get('lp')} rp={data.get('rp')} "
                f"tg={data.get('tg')} ta={data.get('ta')} te={data.get('te')} rr={data.get('rr')} "
                f"kp={data.get('kp')} kd={data.get('kd')} vb={data.get('vb')} vs={data.get('vs')} vc={data.get('vc')} "
                f"mt={data.get('mt')} mph={data.get('mph')} mtg={data.get('mtg')} "
                f"mlp={data.get('mlp')} mrp={data.get('mrp')} mrs={data.get('mrs')}")
        tele_buf.append(line)

    @sio.on("log")
    def on_log(data):
        log_buf.append(f"[{data.get('time')}] {data.get('level')}: {data.get('msg')}")
        print(f"[LOG {data.get('level')}]", data.get("msg"))

    @sio.on("request_status")
    def on_req(data):
        print("[REQ]", _fmt(data))

    @sio.on("car_log")
    def on_car_log(data):
        print("[CAR]", data.get("line", data))

    @sio.on("client_status")
    def on_cs(data):
        print("[CLIENT]", _fmt(data))

    @sio.on("ai_result")
    def on_ai(data):
        print("[AI]", _fmt(data))

    sio.connect(URL, transports=["polling"])
    print(f"connected to {URL}")

    if action == "status":
        sio.emit("get_status")
        time.sleep(1.0)
    elif action == "params":
        params = _parse_params(sys.argv[2:])
        print(f"send_params: {params}")
        sio.emit("send_params", params)
        time.sleep(1.5)
    elif action == "cmd":
        cmd = sys.argv[2]
        print(f"send_cmd: {cmd}")
        sio.emit("send_cmd", {"cmd": cmd})
        time.sleep(1.5)
    elif action == "rotate":
        deg = float(sys.argv[2])
        print(f"send_rotate: {deg}")
        sio.emit("send_rotate", {"deg": deg})
        time.sleep(2.0)
    elif action == "motor_test":
        print("send_motor_test: lifted=true")
        sio.emit("send_motor_test", {"lifted": True})
        time.sleep(15.0)
    elif action == "tele":
        secs = int(sys.argv[2]) if len(sys.argv) > 2 else 5
        print(f"监听遥测 {secs} 秒...")
        sio.emit("get_status")
        time.sleep(secs)
        print(f"\n=== 收到 {len(tele_buf)} 帧遥测 ===")
        for line in tele_buf[-20:]:
            print(line)
        if log_buf:
            print("\n=== 日志 ===")
            for line in log_buf:
                print(line)
    else:
        print(f"未知 action: {action}")
        print(__doc__)

    sio.disconnect()


def _fmt(data):
    if isinstance(data, dict):
        items = []
        for k, v in data.items():
            if isinstance(v, float):
                items.append(f"{k}={v:.3f}")
            else:
                items.append(f"{k}={v}")
        return "{" + ", ".join(items) + "}"
    return str(data)


if __name__ == "__main__":
    main()
