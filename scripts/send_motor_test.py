"""Send one explicitly lifted motor qualification request to the local host."""

from __future__ import annotations

import argparse
import sys
import threading

import socketio


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--url", default="http://127.0.0.1:5000")
    parser.add_argument("--timeout", type=float, default=5.0)
    args = parser.parse_args()

    client = socketio.Client(reconnection=False, logger=False, engineio_logger=False)
    done = threading.Event()
    result: dict[str, object] = {}

    @client.on("request_status")
    def on_status(data: dict[str, object]) -> None:
        state = data.get("state")
        if state in {"pending", "confirmed", "rejected", "timeout"}:
            result.update(data)
            if state != "pending":
                done.set()

    try:
        client.connect(args.url, transports=["polling"], wait_timeout=args.timeout)
        client.emit("send_motor_test", {"lifted": True})
        if not done.wait(args.timeout):
            print("request accepted locally; firmware ACK not observed before timeout")
            return 0
        state = result.get("state")
        print(f"request_status={result}")
        return 0 if state == "confirmed" else 2
    except Exception as exc:  # noqa: BLE001 - CLI must report transport failures
        print(f"motor test request failed: {exc}", file=sys.stderr)
        return 3
    finally:
        if client.connected:
            client.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())

