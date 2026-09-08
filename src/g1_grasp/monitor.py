"""Token-protected, read-only camera and process monitor for Galbot G1.

The module intentionally imports no motion, navigation, joint-command or
gripper-command class. Its HTTP surface contains GET endpoints only.
"""

from __future__ import annotations

import argparse
import hmac
import json
import os
import threading
import time
import urllib.error
import urllib.request
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any


CAMERAS = ("head_left", "head_right", "left_wrist", "right_wrist")


class CameraHub:
    def __init__(self) -> None:
        self.robot = None
        self.sensors: dict[str, Any] = {}
        self.ready = False
        self.error = "initializing"
        self.started_at = time.time()
        self.last_frame_at: dict[str, float] = {}
        self._lock = threading.Lock()
        threading.Thread(target=self._initialize, name="g1-camera-init", daemon=True).start()

    def _initialize(self) -> None:
        try:
            from galbot_sdk.g1 import GalbotRobot, SensorType

            get_instance = getattr(GalbotRobot, "get_instance", None)
            robot = get_instance() if get_instance is not None else GalbotRobot()
            sensors = {
                "head_left": SensorType.HEAD_LEFT_CAMERA,
                "head_right": SensorType.HEAD_RIGHT_CAMERA,
                "left_wrist": SensorType.LEFT_ARM_CAMERA,
                "right_wrist": SensorType.RIGHT_ARM_CAMERA,
            }
            if not robot.init(set(sensors.values())):
                raise RuntimeError("GalbotRobot camera-only initialization returned false")
            self.robot, self.sensors = robot, sensors
            self.ready, self.error = True, ""
        except Exception as exc:  # fail closed and keep the status page alive
            self.ready, self.error = False, f"{type(exc).__name__}: {exc}"

    def frame(self, name: str) -> tuple[bytes, str]:
        if name not in CAMERAS:
            raise KeyError(name)
        if not self.ready or self.robot is None:
            raise RuntimeError(self.error or "camera hub is not ready")
        with self._lock:
            message = self.robot.get_rgb_data(self.sensors[name])
        if not message or not message.get("data"):
            raise RuntimeError("camera returned no frame")
        payload = bytes(message["data"])
        image_type = _image_type(payload)
        self.last_frame_at[name] = time.time()
        return payload, image_type


def _image_type(payload: bytes) -> str:
    if payload.startswith(b"\xff\xd8\xff"):
        return "image/jpeg"
    if payload.startswith(b"\x89PNG\r\n\x1a\n"):
        return "image/png"
    raise RuntimeError("camera payload is not JPEG or PNG")


def _process_flags() -> dict[str, bool]:
    flags = {"teleop_execute": False, "motion_planner": False, "camera_viewer": False}
    proc = Path("/proc")
    if not proc.is_dir():
        return flags
    for child in proc.iterdir():
        if not child.name.isdigit():
            continue
        try:
            command = (child / "cmdline").read_bytes().replace(b"\0", b" ").decode(errors="replace")
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
        flags["teleop_execute"] |= "g1_teleop" in command and "--mode execute" in command
        flags["motion_planner"] |= "service_motion_plan" in command
        flags["camera_viewer"] |= "camera_viewer.py" in command
    return flags


def _cloud_health(url: str) -> dict[str, Any]:
    try:
        with urllib.request.urlopen(url.rstrip("/") + "/healthz", timeout=1.0) as response:
            return json.load(response)
    except (OSError, ValueError, urllib.error.URLError) as exc:
        return {"status": "unavailable", "error": str(exc), "execution_permitted": False}


def make_handler(token: str, cloud_url: str, hub: CameraHub):
    if len(token) < 32:
        raise RuntimeError("G1_MONITOR_TOKEN must contain at least 32 characters")

    class Handler(BaseHTTPRequestHandler):
        server_version = "G1Monitor/0.1"

        def log_message(self, format: str, *args: Any) -> None:
            print(f"monitor_http peer={self.client_address[0]} {format % args}", flush=True)

        def do_GET(self) -> None:  # noqa: N802
            path = self.path.split("?", 1)[0]
            if path == "/":
                self._send(HTTPStatus.OK, MONITOR_HTML.encode(), "text/html; charset=utf-8")
                return
            if not hmac.compare_digest(
                self.headers.get("Authorization", ""), f"Bearer {token}"
            ):
                self._json(HTTPStatus.UNAUTHORIZED, {"error": "unauthorized"})
                return
            if path == "/api/status":
                processes = _process_flags()
                cloud = _cloud_health(cloud_url)
                self._json(
                    HTTPStatus.OK,
                    {
                        "mode": "MONITOR_ONLY",
                        "activity": (
                            "CAMERA_MONITORING"
                            if hub.ready
                            else "CAMERA_INITIALIZING"
                            if hub.error == "initializing"
                            else "CAMERA_ERROR"
                        ),
                        "camera_ready": hub.ready,
                        "camera_error": hub.error,
                        "last_frame_age_ms": {
                            name: round((time.time() - timestamp) * 1000, 1)
                            for name, timestamp in hub.last_frame_at.items()
                        },
                        "processes": processes,
                        "cloud_shadow": cloud,
                        "execution_permitted": False,
                        "motion_routes": 0,
                        "uptime_s": round(time.time() - hub.started_at, 1),
                    },
                )
                return
            if path.startswith("/camera/") and path.endswith(".jpg"):
                name = path[len("/camera/") : -len(".jpg")]
                try:
                    frame, image_type = hub.frame(name)
                except KeyError:
                    self._json(HTTPStatus.NOT_FOUND, {"error": "unknown camera"})
                    return
                except RuntimeError as exc:
                    self._json(HTTPStatus.SERVICE_UNAVAILABLE, {"error": str(exc)})
                    return
                self._send(HTTPStatus.OK, frame, image_type)
                return
            self._json(HTTPStatus.NOT_FOUND, {"error": "not found"})

        def _json(self, status: HTTPStatus, payload: dict[str, Any]) -> None:
            self._send(
                status,
                json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode(),
                "application/json; charset=utf-8",
            )

        def _send(self, status: HTTPStatus, body: bytes, content_type: str) -> None:
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.end_headers()
            self.wfile.write(body)

    return Handler


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=7862)
    args = parser.parse_args()
    token = os.environ.get("G1_MONITOR_TOKEN", "")
    cloud_url = os.environ.get("GRASP_CLOUD_URL", "http://192.168.31.170:8088")
    hub = CameraHub()
    server = ThreadingHTTPServer((args.host, args.port), make_handler(token, cloud_url, hub))
    print(
        f"G1 monitor-only dashboard ready host={args.host} port={args.port} motion_routes=0",
        flush=True,
    )
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
    return 0


MONITOR_HTML = r"""<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8"><meta name="referrer" content="no-referrer"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>G1 Feeding Monitor</title><style>
:root{color-scheme:dark;--bg:#071018;--panel:#0e1b27;--line:#20394c;--cyan:#36d7e8;--green:#4ade80;--red:#fb7185;--muted:#91a4b5}
*{box-sizing:border-box}body{margin:0;background:radial-gradient(circle at top,#123047,#071018 52%);font:14px system-ui;color:#eaf5fb}
header{display:flex;align-items:center;justify-content:space-between;padding:18px 24px;border-bottom:1px solid var(--line);position:sticky;top:0;background:#071018e8;backdrop-filter:blur(14px);z-index:2}
h1{font-size:19px;margin:0;letter-spacing:.04em}.lock{color:var(--green);font-weight:800}.wrap{max-width:1500px;margin:auto;padding:18px;display:grid;grid-template-columns:1fr 330px;gap:16px}
.cams{display:grid;grid-template-columns:1fr 1fr;gap:12px}.card{background:linear-gradient(145deg,#102433,#0a151e);border:1px solid var(--line);border-radius:14px;overflow:hidden;box-shadow:0 14px 40px #0006}
.camhead{padding:10px 13px;display:flex;justify-content:space-between}.dot{width:8px;height:8px;background:var(--cyan);border-radius:50%;box-shadow:0 0 12px var(--cyan)}
img{display:block;width:100%;aspect-ratio:16/9;object-fit:cover;background:#02070b}.side{display:flex;flex-direction:column;gap:12px}.body{padding:15px}.label{color:var(--muted);font-size:12px}.value{font:700 16px ui-monospace;margin:5px 0 13px}.ok{color:var(--green)}.bad{color:var(--red)}
.banner{border-color:#245b43;background:#0d2a20;padding:13px 16px;color:#b9f6d0;font-weight:700}.rows{display:grid;grid-template-columns:1fr auto;gap:9px;border-top:1px solid var(--line);padding-top:12px}.rows span:nth-child(odd){color:var(--muted)}
a{color:var(--cyan)}@media(max-width:900px){.wrap{grid-template-columns:1fr}.cams{grid-template-columns:1fr}}
</style></head><body><header><h1>GALBOT G1 · FEEDING SHADOW MONITOR</h1><div class="lock">● MONITOR ONLY · MOTION LOCKED</div></header>
<main class="wrap"><section class="cams" id="cams"></section><aside class="side">
<div class="card banner">此页面没有运动、关节、夹爪或导航接口。</div>
<div class="card body"><div class="label">当前活动</div><div id="activity" class="value">连接中…</div><div class="label">相机状态</div><div id="camera" class="value">—</div><div class="label">云端规划</div><div id="cloud" class="value">—</div><div class="rows"><span>执行许可</span><b id="exec">FALSE</b><span>运动路由</span><b id="routes">0</b><span>执行遥操作</span><b id="teleop">—</b><span>运动规划进程</span><b id="planner">—</b><span>运行时间</span><b id="uptime">—</b></div></div>
<div class="card body"><div class="label">三维空间</div><p><a href="http://192.168.31.170:8090/viewer.html" target="_blank" rel="noreferrer">打开双臂可达点云查看器 ↗</a></p><div class="label">刷新时间</div><div id="clock" class="value">—</div></div>
</aside></main><script>
const token=new URLSearchParams(location.hash.slice(1)).get('token')||'';
const auth={Authorization:`Bearer ${token}`};
const names=[['head_left','头部左相机'],['head_right','头部右相机'],['left_wrist','左腕相机'],['right_wrist','右腕相机']];
const cams=document.getElementById('cams');
const el=id=>document.getElementById(id);
for(const [id,label] of names){const c=document.createElement('div');c.className='card';c.innerHTML=`<div class="camhead"><b>${label}</b><span class="dot"></span></div><img id="${id}" alt="${label}">`;cams.appendChild(c)}
async function refreshImages(){for(const [id] of names){try{const r=await fetch(`/camera/${id}.jpg`,{headers:auth,cache:'no-store'});if(!r.ok)continue;const img=document.getElementById(id),old=img.dataset.url,url=URL.createObjectURL(await r.blob());img.src=url;img.dataset.url=url;if(old)URL.revokeObjectURL(old)}catch(e){}}}
const cls=(v)=>v?'ok':'bad';const yn=(v)=>v?'ACTIVE':'CLEAR';
async function refreshStatus(){try{const r=await fetch('/api/status',{headers:auth,cache:'no-store'});const s=await r.json();
el('activity').textContent=s.activity;el('activity').className='value '+cls(s.activity==='CAMERA_MONITORING');el('camera').textContent=s.camera_ready?'4 CAMERA READY':(s.camera_error||'INITIALIZING');el('camera').className='value '+cls(s.camera_ready);
el('cloud').textContent=s.cloud_shadow.status==='ok'?'SHADOW ONLINE':'UNAVAILABLE';el('cloud').className='value '+cls(s.cloud_shadow.status==='ok');el('exec').textContent=String(s.execution_permitted).toUpperCase();el('routes').textContent=s.motion_routes;
el('teleop').textContent=yn(s.processes.teleop_execute);el('teleop').className=cls(!s.processes.teleop_execute);el('planner').textContent=yn(s.processes.motion_planner);el('planner').className=cls(!s.processes.motion_planner);el('uptime').textContent=s.uptime_s+' s';el('clock').textContent=new Date().toLocaleTimeString();}catch(e){el('activity').textContent='MONITOR OFFLINE';el('activity').className='value bad'}}
setInterval(refreshImages,700);setInterval(refreshStatus,1000);refreshImages();refreshStatus();
</script></body></html>"""


if __name__ == "__main__":
    raise SystemExit(main())
