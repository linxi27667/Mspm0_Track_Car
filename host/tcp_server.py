"""Threaded single-client TCP server for the TC1 tuning protocol."""

from __future__ import annotations

import logging
import socket
import threading
from typing import Any, Callable

from protocol import (
    MAX_FRAME_BYTES,
    MAX_REQUEST_ID,
    PROTOCOL_VERSION,
    REQUEST_MAX_RETRIES,
    REQUEST_TIMEOUT_MS,
    ProtocolError,
    build_auth,
    build_cmd,
    build_motor_test,
    build_rotate,
    build_set,
    decode_frame,
    encode_frame,
    load_json_object,
    parse_ack,
    parse_telemetry,
    validate_ping,
    validate_request,
)


logger = logging.getLogger("tcp_server")

ACCEPT_TIMEOUT = 0.5
CLIENT_IO_TIMEOUT = 1.0
RECV_CHUNK_SIZE = 4096


class TcpServer:
    """TCP server with TC1 framing and one active vehicle connection."""

    def __init__(
        self,
        port: int = 8888,
        on_telemetry: Callable[[dict], None] | None = None,
        on_connect: Callable[[tuple], None] | None = None,
        on_disconnect: Callable[[], None] | None = None,
        on_log: Callable[[str], None] | None = None,
        on_ack: Callable[[dict], None] | None = None,
    ):
        self.port = port
        self.on_telemetry = on_telemetry
        self.on_connect = on_connect
        self.on_disconnect = on_disconnect
        self.on_log = on_log
        self.on_ack = on_ack

        self._sock: socket.socket | None = None
        self._client: socket.socket | None = None
        self._client_addr: tuple | None = None
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._running = False
        self._accept_thread: threading.Thread | None = None
        self._frame_count = 0
        self._next_request_id = 1
        self._last_request_id: int | None = None

    def start(self):
        """Start listening in a daemon thread."""
        with self._lock:
            if self._running:
                return

        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind(("0.0.0.0", self.port))
            server.listen(1)
            server.settimeout(ACCEPT_TIMEOUT)
        except Exception:
            server.close()
            raise

        with self._lock:
            self._sock = server
            self.port = server.getsockname()[1]
            self._running = True
            self._stop_event.clear()
            self._accept_thread = threading.Thread(
                target=self._accept_loop,
                args=(server,),
                daemon=True,
                name="tcp-accept",
            )
            self._accept_thread.start()
        logger.info("TCP server listening on :%d", self.port)

    def stop(self):
        """Stop listening and close the active client without lock recursion."""
        self._stop_event.set()
        with self._lock:
            self._running = False
            server = self._sock
            client = self._client
            accept_thread = self._accept_thread
            self._sock = None
            self._client = None
            self._client_addr = None
            self._accept_thread = None

        self._safe_close(server)
        self._safe_close(client)
        if accept_thread and accept_thread is not threading.current_thread():
            accept_thread.join(timeout=ACCEPT_TIMEOUT + 0.5)

    def send(self, json_str: str | bytes | bytearray | dict[str, Any]) -> bool:
        """Validate, frame, and send one downlink request, auth, or ping.

        Existing callers may omit ``v`` and ``id`` from set/cmd JSON. The server
        adds protocol version 1 and a monotonically increasing request id before
        transmitting the frame.
        """
        try:
            obj = load_json_object(json_str)
        except ProtocolError as exc:
            logger.error("send rejected: %s", exc)
            return False

        disconnected = False
        send_error: OSError | None = None
        with self._lock:
            if not self._client:
                logger.warning("send failed: no client connected")
                return False
            try:
                obj = self._prepare_outgoing_locked(obj)
                frame = encode_frame(obj)
            except ProtocolError as exc:
                logger.error("send rejected: %s", exc)
                return False

            client = self._client
            try:
                client.sendall(frame)
                logger.debug("send request id=%s: %s", obj.get("id"), obj)
                return True
            except (OSError, socket.timeout) as exc:
                send_error = exc
                disconnected = self._close_client_locked(client)

        logger.error("send failed: %s", send_error)
        if disconnected:
            self._invoke(self.on_disconnect)
        return False

    def is_connected(self) -> bool:
        with self._lock:
            return self._client is not None

    def get_client_addr(self):
        with self._lock:
            return self._client_addr

    def get_frame_count(self) -> int:
        with self._lock:
            return self._frame_count

    def get_last_request_id(self) -> int | None:
        with self._lock:
            return self._last_request_id

    def _prepare_outgoing_locked(self, obj: dict[str, Any]) -> dict[str, Any]:
        if "set" in obj or "cmd" in obj:
            obj = dict(obj)
            obj.setdefault("v", PROTOCOL_VERSION)
            obj.setdefault("t", "set" if "set" in obj else "cmd")
            if "id" not in obj:
                obj["id"] = self._allocate_request_id_locked()
            clean = validate_request(obj)
            self._last_request_id = clean["id"]
            return clean
        if obj.get("t") == "auth":
            return load_json_object(build_auth(
                obj.get("token", ""), session=obj.get("session"),
            ))
        if obj.get("t") == "ping":
            return validate_ping(obj)
        raise ProtocolError("outgoing message must be set, cmd, or ping")

    def _allocate_request_id_locked(self) -> int:
        request_id = self._next_request_id
        self._next_request_id = 1 if request_id >= MAX_REQUEST_ID else request_id + 1
        return request_id

    def _accept_loop(self, server: socket.socket):
        while not self._stop_event.is_set():
            try:
                conn, addr = server.accept()
            except socket.timeout:
                continue
            except OSError:
                break

            self._configure_client(conn)
            with self._lock:
                if not self._running:
                    self._safe_close(conn)
                    break
                old_client = self._client
                had_old_client = old_client is not None
                self._client = conn
                self._client_addr = addr

            self._safe_close(old_client)
            if had_old_client:
                self._invoke(self.on_disconnect)
            logger.info("client connected: %s:%d", addr[0], addr[1])
            self._invoke(self.on_connect, addr)
            threading.Thread(
                target=self._recv_loop,
                args=(conn,),
                daemon=True,
                name=f"tcp-recv-{addr[1]}",
            ).start()

    def _recv_loop(self, conn: socket.socket):
        buf = bytearray()
        while not self._stop_event.is_set():
            try:
                data = conn.recv(RECV_CHUNK_SIZE)
            except socket.timeout:
                continue
            except OSError:
                break
            if not data:
                break

            buf.extend(data)
            while True:
                newline = buf.find(b"\n")
                if newline < 0:
                    break
                line = bytes(buf[: newline + 1])
                del buf[: newline + 1]
                if len(line) > MAX_FRAME_BYTES:
                    logger.warning("discard oversized TC1 line: %d bytes", len(line))
                    continue
                self._handle_frame(line)

            if len(buf) >= MAX_FRAME_BYTES:
                logger.warning("closing client with oversized unterminated frame")
                break

        if self._disconnect_client(conn):
            logger.info("client disconnected")
            self._invoke(self.on_disconnect)

    def _handle_frame(self, line: bytes):
        try:
            obj = decode_frame(line)
        except ProtocolError as exc:
            logger.warning("discard invalid TC1 frame: %s", exc)
            return

        frame_type = obj.get("t")
        if frame_type == "tele":
            telemetry = parse_telemetry(obj)
            if telemetry is None:
                logger.warning("discard invalid telemetry schema")
                return
            with self._lock:
                self._frame_count += 1
            self._invoke(self.on_telemetry, telemetry)
            return

        if frame_type == "ack":
            ack = parse_ack(obj)
            if ack is None:
                logger.warning("discard invalid ack schema")
                return
            self._invoke(self.on_ack, ack)
            return

        if frame_type == "log":
            if not self._valid_log(obj):
                logger.warning("discard invalid log schema")
                return
            self._invoke(self.on_log, obj["msg"])
            return

        if frame_type == "ping":
            try:
                validate_ping(obj)
            except ProtocolError as exc:
                logger.warning("discard invalid ping: %s", exc)
            return

        if frame_type == "pong":
            try:
                validate_ping({"v": obj.get("v"), "t": "ping", "ms": obj.get("ms")})
            except ProtocolError as exc:
                logger.warning("discard invalid pong: %s", exc)
            return

        logger.warning("discard unknown TC1 message type: %r", frame_type)

    @staticmethod
    def _valid_log(obj: dict[str, Any]) -> bool:
        if set(obj) - {"v", "t", "msg"}:
            return False
        if "v" in obj and obj["v"] != PROTOCOL_VERSION:
            return False
        return obj.get("t") == "log" and isinstance(obj.get("msg"), str) and len(obj["msg"]) <= 1024

    @staticmethod
    def _configure_client(conn: socket.socket):
        conn.settimeout(CLIENT_IO_TIMEOUT)
        try:
            conn.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
            if hasattr(socket, "TCP_KEEPIDLE"):
                conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPIDLE, 10)
            if hasattr(socket, "TCP_KEEPINTVL"):
                conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, 3)
            if hasattr(socket, "TCP_KEEPCNT"):
                conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT, 3)
            if hasattr(socket, "SIO_KEEPALIVE_VALS"):
                conn.ioctl(socket.SIO_KEEPALIVE_VALS, (1, 10_000, 3_000))
        except OSError:
            logger.debug("platform keepalive tuning unavailable", exc_info=True)

    def _disconnect_client(self, conn: socket.socket) -> bool:
        with self._lock:
            return self._close_client_locked(conn)

    def _close_client_locked(self, expected: socket.socket | None = None) -> bool:
        if self._client is None or (expected is not None and expected is not self._client):
            return False
        client = self._client
        self._client = None
        self._client_addr = None
        self._safe_close(client)
        return True

    @staticmethod
    def _safe_close(sock: socket.socket | None):
        if sock is None:
            return
        try:
            sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        try:
            sock.close()
        except OSError:
            pass

    @staticmethod
    def _invoke(callback, *args):
        if callback is None:
            return
        try:
            callback(*args)
        except Exception:
            logger.exception("TCP callback failed")


# ============================================================
# RequestManager — 单待确认请求 + 300ms 超时 + 2 次重试
# ============================================================


class RequestManager:
    """Manages a single pending set/cmd request with timeout and retry.

    - Only one pending request at a time (send returns False if busy)
    - 300ms timeout per attempt, retries twice with the same ID (3 total attempts)
    - ACK must be received before parameters are considered applied
    - Stale ACKs (non-matching ID) are silently ignored

    Usage::

        rm = RequestManager(tcp,
                           on_confirmed=handle_confirmed,
                           on_rejected=handle_rejected,
                           on_timeout=handle_timeout)
        tcp.on_ack = rm.on_ack  # route ACKs to manager
        rm.send_set({"KP": 6.0})
    """

    def __init__(
        self,
        tcp_server: TcpServer,
        on_confirmed: Callable[[dict], None] | None = None,
        on_rejected: Callable[[dict], None] | None = None,
        on_timeout: Callable[[dict], None] | None = None,
    ):
        self._tcp = tcp_server
        self._on_confirmed = on_confirmed
        self._on_rejected = on_rejected
        self._on_timeout = on_timeout

        self._lock = threading.Lock()
        self._next_id = 1
        self._pending_json: str | None = None
        self._pending_id: int | None = None
        self._pending_kind: str | None = None
        self._pending_params: dict[str, Any] | None = None
        self._pending_cmd: str | None = None
        self._attempt = 0
        self._timer: threading.Timer | None = None

    # ---------- public API ----------

    def is_busy(self) -> bool:
        with self._lock:
            return self._pending_json is not None

    def get_pending_id(self) -> int | None:
        with self._lock:
            return self._pending_id

    def send_set(self, params: dict[str, Any]) -> bool:
        """Send a set request. Returns False if busy or send failed."""
        with self._lock:
            if self._pending_json is not None:
                logger.warning("request manager: busy, rejecting send_set")
                return False
            request_id = self._allocate_id_locked()
            try:
                json_str = build_set(params, request_id=request_id)
            except (ProtocolError, ValueError) as exc:
                logger.error("request manager: build_set failed: %s", exc)
                return False
            self._pending_json = json_str
            self._pending_id = request_id
            self._pending_kind = "set"
            self._pending_params = dict(params)
            self._pending_cmd = None
            self._attempt = 0
        return self._send_reserved(request_id, "set")

    def send_cmd(self, cmd: str) -> bool:
        """Send a cmd request. Returns False if busy or send failed."""
        with self._lock:
            if self._pending_json is not None:
                logger.warning("request manager: busy, rejecting send_cmd")
                return False
            request_id = self._allocate_id_locked()
            try:
                json_str = build_cmd(cmd, request_id=request_id)
            except (ProtocolError, ValueError) as exc:
                logger.error("request manager: build_cmd failed: %s", exc)
                return False
            self._pending_json = json_str
            self._pending_id = request_id
            self._pending_kind = "cmd"
            self._pending_params = None
            self._pending_cmd = cmd
            self._attempt = 0
        return self._send_reserved(request_id, "cmd")

    def send_rotate(self, degrees: float) -> bool:
        """Send one signed-angle rotate command with normal ACK/retry handling."""
        with self._lock:
            if self._pending_json is not None:
                logger.warning("request manager: busy, rejecting send_rotate")
                return False
            request_id = self._allocate_id_locked()
            try:
                json_str = build_rotate(degrees, request_id=request_id)
            except (ProtocolError, ValueError) as exc:
                logger.error("request manager: build_rotate failed: %s", exc)
                return False
            self._pending_json = json_str
            self._pending_id = request_id
            self._pending_kind = "cmd"
            self._pending_params = None
            self._pending_cmd = f"rotate:{degrees:.1f}"
            self._attempt = 0
        return self._send_reserved(request_id, "rotate")

    def send_motor_test(self) -> bool:
        """Send the fixed lifted-wheel bench test with ACK/retry handling."""
        with self._lock:
            if self._pending_json is not None:
                logger.warning("request manager: busy, rejecting send_motor_test")
                return False
            request_id = self._allocate_id_locked()
            try:
                json_str = build_motor_test(request_id=request_id)
            except (ProtocolError, ValueError) as exc:
                logger.error("request manager: build_motor_test failed: %s", exc)
                return False
            self._pending_json = json_str
            self._pending_id = request_id
            self._pending_kind = "cmd"
            self._pending_params = None
            self._pending_cmd = "motor_test"
            self._attempt = 0
        return self._send_reserved(request_id, "motor_test")

    def send_stop_override(self) -> bool:
        """Cancel the pending request locally and send a tracked stop immediately."""
        with self._lock:
            if self._pending_json is not None:
                logger.warning("request manager: replacing pending id=%d with stop", self._pending_id)
                self._clear_pending_locked()
        return self.send_cmd("stop")

    def on_ack(self, ack: dict):
        """Handle an incoming ACK. Resolves the pending request if ID matches."""
        with self._lock:
            if self._pending_id is None:
                return
            if ack.get("id") != self._pending_id:
                return  # stale ACK from a previous request
            self._cancel_timer_locked()
            kind = self._pending_kind
            request_id = self._pending_id
            self._clear_pending_locked()

        if ack.get("ok"):
            logger.info("request manager: confirmed id=%d kind=%s", request_id, kind)
            if self._on_confirmed:
                self._on_confirmed(ack)
        else:
            logger.warning(
                "request manager: rejected id=%d kind=%s error=%s",
                request_id, kind, ack.get("error", ""),
            )
            if self._on_rejected:
                self._on_rejected(ack)

    # ---------- internal ----------

    def _send_reserved(self, request_id: int, label: str) -> bool:
        """Send after publishing pending state, allowing a synchronous ACK safely."""
        with self._lock:
            json_str = self._pending_json
        if json_str is None or not self._tcp.send(json_str):
            with self._lock:
                if self._pending_id == request_id:
                    self._clear_pending_locked()
            return False

        with self._lock:
            # A fast ACK may already have resolved the request.
            if self._pending_id == request_id:
                self._start_timer_locked()
        logger.info("request manager: sent %s id=%d (attempt 0)", label, request_id)
        return True

    def _handle_timeout(self):
        """Timer callback: retry with same ID or declare timeout."""
        snapshot = None
        with self._lock:
            if self._pending_json is None:
                return  # already resolved by ACK

            self._attempt += 1
            request_id = self._pending_id
            attempt = self._attempt

            if attempt > REQUEST_MAX_RETRIES:
                snapshot = {
                    "id": request_id,
                    "kind": self._pending_kind,
                    "params": self._pending_params,
                    "cmd": self._pending_cmd,
                    "attempts": attempt,
                }
                self._clear_pending_locked()
                logger.warning(
                    "request manager: timeout id=%d after %d attempts",
                    request_id, attempt,
                )
            else:
                # retry: resend exact same JSON (same ID)
                json_str = self._pending_json
                snapshot = None

        if snapshot is not None:
            if self._on_timeout:
                self._on_timeout(snapshot)
            return

        sent_ok = self._tcp.send(json_str)
        with self._lock:
            # A synchronous ACK may have cleared this request during send().
            if self._pending_id != request_id:
                return
            if sent_ok:
                self._start_timer_locked()
                logger.info("request manager: retry id=%d (attempt %d)", request_id, attempt)
                return
            snapshot = {
                "id": request_id,
                "kind": self._pending_kind,
                "params": self._pending_params,
                "cmd": self._pending_cmd,
                "attempts": attempt,
            }
            self._clear_pending_locked()
            logger.warning("request manager: send failed on retry id=%d", request_id)

        # invoke callback outside lock
        if self._on_timeout:
            self._on_timeout(snapshot)

    def _allocate_id_locked(self) -> int:
        request_id = self._next_id
        self._next_id = 1 if request_id >= MAX_REQUEST_ID else request_id + 1
        return request_id

    def _start_timer_locked(self):
        self._cancel_timer_locked()
        self._timer = threading.Timer(
            REQUEST_TIMEOUT_MS / 1000.0,
            self._handle_timeout,
        )
        self._timer.daemon = True
        self._timer.start()

    def _cancel_timer_locked(self):
        if self._timer is not None:
            self._timer.cancel()
            self._timer = None

    def _clear_pending_locked(self):
        self._pending_json = None
        self._pending_id = None
        self._pending_kind = None
        self._pending_params = None
        self._pending_cmd = None
        self._cancel_timer_locked()
