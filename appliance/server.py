"""HTTP appliance v1 (stdlib) — token optional. VT-ARM-002."""

from __future__ import annotations

import json
import os
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Dict, Optional
from urllib.parse import parse_qs, urlparse

from .interpolator import TrajectoryInterpolator, TrajectorySample
from .recording import RecordingStore

HELLO = {
    "schema_version": 1,
    "module_id": "armpi_mini",
    "fw": "appliance-1.0.0",
    "caps": ["trajectory", "cartesian", "camera", "record", "gripper"],
}


class Appliance:
    def __init__(
        self,
        token: str = "",
        motion: Optional[TrajectoryInterpolator] = None,
        background: bool = True,
        ak: Any = None,
    ) -> None:
        self.token = token or os.environ.get("ARMPI_TOKEN", "")
        self.motion = motion or TrajectoryInterpolator()
        self.record = RecordingStore()
        self.started = time.time()
        self.ak = ak
        self.battery_fn = None
        if background:
            self.motion.start_background()

    def hello(self) -> Dict[str, Any]:
        return dict(HELLO)

    def health(self) -> Dict[str, Any]:
        root = self.record.root
        free = 0
        try:
            st = os.statvfs(root)
            free = st.f_bavail * st.f_frsize
        except OSError:
            pass
        return {
            "ok": not self.motion.state.estop,
            "estop": self.motion.state.estop,
            "loop_hz": self.motion.state.loop_hz,
            "queue_depth": 1 if self.motion.state.busy else 0,
            "uptime_s": time.time() - self.started,
            "ssd_free_bytes": free,
            "recording": self.record.active_id,
            "battery_v": self._battery(),
        }

    def _battery(self):
        if self.battery_fn is None:
            return None
        try:
            return float(self.battery_fn())
        except Exception:
            return None

    def state(self) -> Dict[str, Any]:
        s = self.motion.state
        return {
            "joints_deg": list(s.joints_deg),
            "busy": s.busy,
            "following": s.following,
            "estop": s.estop,
            "packets": s.packets,
        }

    def load_trajectory(self, body: Dict[str, Any]) -> Dict[str, Any]:
        samples = [
            TrajectorySample(float(p["t_s"]), [float(x) for x in p["joints_deg"]])
            for p in body.get("samples", [])
        ]
        self.motion.load(samples, replace=bool(body.get("replace_queue")))
        return {"ok": True, "n": len(samples)}


def _json(handler: BaseHTTPRequestHandler, code: int, payload: Any) -> None:
    data = json.dumps(payload).encode("utf-8")
    handler.send_response(code)
    handler.send_header("Content-Type", "application/json")
    handler.send_header("Content-Length", str(len(data)))
    handler.send_header("Access-Control-Allow-Origin", "*")
    handler.end_headers()
    handler.wfile.write(data)


def make_handler(app: Appliance):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt: str, *args: Any) -> None:
            return

        def _auth_ok(self) -> bool:
            if not app.token:
                return True
            hdr = self.headers.get("Authorization", "")
            return hdr == f"Bearer {app.token}"

        def do_OPTIONS(self) -> None:  # noqa: N802
            self.send_response(204)
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Access-Control-Allow-Headers", "Authorization, Content-Type")
            self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
            self.end_headers()

        def do_GET(self) -> None:  # noqa: N802
            if not self._auth_ok():
                return _json(self, 401, {"error": "unauthorized"})
            path = urlparse(self.path).path
            if path == "/v1/hello":
                return _json(self, 200, app.hello())
            if path == "/v1/health":
                return _json(self, 200, app.health())
            if path == "/v1/state":
                return _json(self, 200, app.state())
            if path in ("/v1/state/stream", "/v1/state/ws"):
                self.send_response(200)
                self.send_header("Content-Type", "text/event-stream")
                self.send_header("Cache-Control", "no-cache")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers()
                try:
                    once = "once=1" in (urlparse(self.path).query or "")
                    while True:
                        payload = json.dumps(app.state()).encode("utf-8")
                        self.wfile.write(b"data: " + payload + b"\n\n")
                        self.wfile.flush()
                        if once:
                            return
                        time.sleep(0.2)
                except (BrokenPipeError, ConnectionResetError, OSError):
                    return
            if path == "/video_feed":
                host = self.headers.get("Host", "127.0.0.1:8000").split(":")[0]
                self.send_response(302)
                self.send_header("Location", f"http://{host}:8080/?action=stream")
                self.end_headers()
                return
            if path == "/v1/recordings":
                return _json(self, 200, {"sessions": app.record.list_sessions()})
            if path.startswith("/v1/recordings/"):
                parts = path.strip("/").split("/")
                if len(parts) == 3:
                    try:
                        return _json(self, 200, app.record.session_meta(parts[2]))
                    except FileNotFoundError:
                        return _json(self, 404, {"error": "not_found"})
                if len(parts) >= 4 and parts[3] == "blob":
                    qs = parse_qs(urlparse(self.path).query)
                    rel = qs.get("path", ["joints.jsonl"])[0]
                    try:
                        blob = app.record.blob_path(parts[2], rel)
                    except (FileNotFoundError, ValueError):
                        return _json(self, 404, {"error": "not_found"})
                    data = blob.read_bytes()
                    rng = self.headers.get("Range", "")
                    start, end = 0, len(data)
                    code = 200
                    if rng.startswith("bytes="):
                        spec = rng.split("=", 1)[1]
                        a, _, b = spec.partition("-")
                        start = int(a or 0)
                        end = int(b) + 1 if b else len(data)
                        end = min(end, len(data))
                        code = 206
                    chunk = data[start:end]
                    self.send_response(code)
                    self.send_header("Content-Type", "application/octet-stream")
                    self.send_header("Content-Length", str(len(chunk)))
                    self.send_header("Accept-Ranges", "bytes")
                    if code == 206:
                        self.send_header(
                            "Content-Range", f"bytes {start}-{end - 1}/{len(data)}"
                        )
                    self.end_headers()
                    self.wfile.write(chunk)
                    return
            return _json(self, 404, {"error": "not_found"})

        def do_POST(self) -> None:  # noqa: N802
            if not self._auth_ok():
                return _json(self, 401, {"error": "unauthorized"})
            length = int(self.headers.get("Content-Length") or 0)
            raw = self.rfile.read(length) if length else b"{}"
            try:
                body = json.loads(raw.decode("utf-8") or "{}")
            except json.JSONDecodeError:
                return _json(self, 400, {"error": "bad_json"})
            path = urlparse(self.path).path
            try:
                if path == "/v1/trajectory":
                    return _json(self, 200, app.load_trajectory(body))
                if path == "/v1/named_pose":
                    pose = {
                        "home": [0.0, 0.0, 0.0, 0.0],
                        "park": [0.0, 20.0, -20.0, 10.0],
                    }.get(body.get("name", "home"), [0.0, 0.0, 0.0, 0.0])
                    app.motion.replace_goal(pose, float(body.get("duration_s", 1.0)))
                    return _json(self, 200, {"ok": True})
                if path == "/v1/cartesian":
                    from .board_io import cartesian_to_joints

                    if app.ak is None:
                        return _json(self, 501, {"error": "cartesian_requires_board_ik"})
                    joints = cartesian_to_joints(
                        app.ak,
                        float(body.get("x_cm", 0)),
                        float(body.get("y_cm", 10)),
                        float(body.get("z_cm", 8)),
                        float(body.get("pitch_deg", 0)),
                    )
                    app.motion.replace_goal(joints, float(body.get("duration_s", 1.0)))
                    return _json(self, 200, {"ok": True, "joints_deg": joints})
                if path == "/v1/replace_goal":
                    app.motion.replace_goal(body.get("joints_deg", [0, 0, 0, 0]))
                    return _json(self, 200, {"ok": True})
                if path == "/v1/estop":
                    app.motion.estop()
                    return _json(self, 200, {"ok": True})
                if path == "/v1/gripper":
                    return _json(
                        self,
                        200,
                        {"ok": True, "position": float(body.get("position", 0))},
                    )
                if path == "/v1/record/start":
                    sid = app.record.start(body.get("experiment_id") or "anon")
                    return _json(self, 200, {"ok": True, "session_id": sid})
                if path == "/v1/record/stop":
                    return _json(self, 200, app.record.stop())
            except Exception as exc:  # noqa: BLE001 — appliance boundary
                return _json(self, 400, {"error": str(exc)})
            return _json(self, 404, {"error": "not_found"})

    return Handler


def serve(
    host: str = "0.0.0.0",
    port: int = 8000,
    token: str = "",
    app: Optional[Appliance] = None,
) -> ThreadingHTTPServer:
    appliance = app or Appliance(token=token)
    httpd = ThreadingHTTPServer((host, port), make_handler(appliance))
    httpd.appliance = appliance  # type: ignore[attr-defined]
    return httpd


def main() -> None:
    host = os.environ.get("ARMPI_HOST", "0.0.0.0")
    port = int(os.environ.get("ARMPI_PORT", "8000"))
    token = os.environ.get("ARMPI_TOKEN", "")
    httpd = serve(host, port, token)
    print(f"ArmPi appliance v1 on {host}:{port}")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        httpd.shutdown()


if __name__ == "__main__":
    main()
