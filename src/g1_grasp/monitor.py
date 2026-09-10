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
        self.sdk_ready = False
        self.frames: dict[str, bytes] = {}
        self.errors: dict[str, str] = {name: "initializing" for name in CAMERAS}
        self.backends: dict[str, str] = {name: "initializing" for name in CAMERAS}
        self.started_at = time.time()
        self.last_frame_at: dict[str, float] = {}
        self._lock = threading.Lock()
        self.mode = os.environ.get("G1_CAMERA_MODE", "auto").strip().lower()
        if self.mode not in {"auto", "sdk", "v4l2"}:
            raise RuntimeError("G1_CAMERA_MODE must be auto, sdk, or v4l2")
        target = self._initialize_v4l2 if self.mode == "v4l2" else self._initialize_sdk
        threading.Thread(target=target, name="g1-camera-init", daemon=True).start()

    @property
    def ready(self) -> bool:
        if self.sdk_ready:
            return True
        now = time.time()
        with self._lock:
            return any(now - stamp < 3.0 for stamp in self.last_frame_at.values())

    @property
    def error(self) -> str:
        status = self.camera_status()
        ready_count = sum(1 for item in status.values() if item["ready"])
        if ready_count:
            return f"{ready_count}/{len(CAMERAS)} cameras ready"
        messages = list(dict.fromkeys(str(item["error"]) for item in status.values()))
        return "; ".join(messages) or "initializing"

    def _initialize_sdk(self) -> None:
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
            self.sdk_ready = True
            with self._lock:
                for name in CAMERAS:
                    self.errors[name] = ""
                    self.backends[name] = "galbot_sdk"
        except Exception as exc:  # fail closed and keep the status page alive
            message = f"{type(exc).__name__}: {exc}"
            with self._lock:
                for name in CAMERAS:
                    self.errors[name] = message
                    self.backends[name] = "galbot_sdk"
            if self.mode == "auto":
                self._initialize_v4l2()

    def _initialize_v4l2(self) -> None:
        head = os.environ.get("G1_HEAD_DEVICE", "/dev/video6")
        left = os.environ.get("G1_LEFT_WRIST_DEVICE", "/dev/video4")
        right = os.environ.get("G1_RIGHT_WRIST_DEVICE", "/dev/video12")
        jobs = (
            (("head_left", "head_right"), head, 1280, 480, "MJPG"),
            (("left_wrist",), left, 640, 480, "YUYV"),
            (("right_wrist",), right, 640, 480, "YUYV"),
        )
        for names, device, width, height, fourcc in jobs:
            threading.Thread(
                target=self._v4l2_loop,
                args=(names, device, width, height, fourcc),
                name=f"g1-v4l2-{'-'.join(names)}",
                daemon=True,
            ).start()

    def _v4l2_loop(
        self,
        names: tuple[str, ...],
        device: str,
        width: int,
        height: int,
        fourcc: str,
    ) -> None:
        backend = f"v4l2:{device}"
        while True:
            capture = None
            try:
                import cv2

                source: Any = device
                if device.startswith("/dev/video") and device[len("/dev/video") :].isdigit():
                    source = int(device[len("/dev/video") :])
                capture = cv2.VideoCapture(source, cv2.CAP_V4L2)
                capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
                capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
                capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
                capture.set(cv2.CAP_PROP_FPS, 10)
                if not capture.isOpened():
                    raise RuntimeError(f"{device} unavailable or busy")
                while True:
                    ok, image = capture.read()
                    if not ok or image is None:
                        raise RuntimeError(f"{device} stopped returning frames")
                    if len(names) == 2:
                        midpoint = image.shape[1] // 2
                        images = (image[:, :midpoint], image[:, midpoint:])
                    else:
                        images = (image,)
                    for name, camera_image in zip(names, images):
                        encoded, payload = cv2.imencode(
                            ".jpg", camera_image, [cv2.IMWRITE_JPEG_QUALITY, 82]
                        )
                        if encoded:
                            self._publish(name, payload.tobytes(), backend)
            except Exception as exc:  # a busy device degrades only its own tile
                message = f"{type(exc).__name__}: {exc}"
                with self._lock:
                    for name in names:
                        self.errors[name] = message
                        self.backends[name] = backend
            finally:
                if capture is not None:
                    capture.release()
            time.sleep(3.0)

    def _publish(self, name: str, payload: bytes, backend: str) -> None:
        with self._lock:
            self.frames[name] = payload
            self.last_frame_at[name] = time.time()
            self.errors[name] = ""
            self.backends[name] = backend

    def camera_status(self) -> dict[str, dict[str, Any]]:
        now = time.time()
        with self._lock:
            return {
                name: {
                    "ready": self.sdk_ready
                    or (name in self.frames and now - self.last_frame_at.get(name, 0) < 3.0),
                    "backend": self.backends[name],
                    "error": self.errors[name],
                    "age_ms": (
                        round((now - self.last_frame_at[name]) * 1000, 1)
                        if name in self.last_frame_at
                        else None
                    ),
                }
                for name in CAMERAS
            }

    def frame(self, name: str) -> tuple[bytes, str]:
        if name not in CAMERAS:
            raise KeyError(name)
        if self.sdk_ready and self.robot is not None:
            with self._lock:
                message = self.robot.get_rgb_data(self.sensors[name])
            if not message or not message.get("data"):
                raise RuntimeError("camera returned no frame")
            payload = bytes(message["data"])
            image_type = _image_type(payload)
            self.last_frame_at[name] = time.time()
            return payload, image_type
        with self._lock:
            payload = self.frames.get(name)
            stamp = self.last_frame_at.get(name, 0)
            error = self.errors[name]
        if payload is None or time.time() - stamp >= 3.0:
            raise RuntimeError(error or f"{name} has no fresh frame")
        return payload, "image/jpeg"


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
                cameras = hub.camera_status()
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
                        "cameras": cameras,
                        "last_frame_age_ms": {
                            name: item["age_ms"]
                            for name, item in cameras.items()
                            if item["age_ms"] is not None
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
.camhead{padding:10px 13px;display:flex;justify-content:space-between;align-items:center}.camstate{display:flex;gap:7px;align-items:center;color:var(--muted);font-size:12px}.dot{width:8px;height:8px;background:var(--red);border-radius:50%}.dot.live{background:var(--cyan);box-shadow:0 0 12px var(--cyan)}
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
const names=[['head_left','头部双目 · 左半'],['head_right','头部双目 · 右半'],['left_wrist','左腕相机'],['right_wrist','右腕相机']];
const cams=document.getElementById('cams');
const el=id=>document.getElementById(id);
for(const [id,label] of names){const c=document.createElement('div');c.className='card';c.innerHTML=`<div class="camhead"><b>${label}</b><span class="camstate"><span id="${id}-text">等待</span><span id="${id}-dot" class="dot"></span></span></div><img id="${id}" alt="${label}">`;cams.appendChild(c)}
async function refreshImages(){for(const [id] of names){try{const r=await fetch(`/camera/${id}.jpg`,{headers:auth,cache:'no-store'});if(!r.ok)continue;const img=document.getElementById(id),old=img.dataset.url,url=URL.createObjectURL(await r.blob());img.src=url;img.dataset.url=url;if(old)URL.revokeObjectURL(old)}catch(e){}}}
const cls=(v)=>v?'ok':'bad';const yn=(v)=>v?'ACTIVE':'CLEAR';
async function refreshStatus(){try{const r=await fetch('/api/status',{headers:auth,cache:'no-store'});const s=await r.json();
el('activity').textContent=s.activity;el('activity').className='value '+cls(s.activity==='CAMERA_MONITORING');const live=Object.values(s.cameras||{}).filter(v=>v.ready).length;el('camera').textContent=`${live}/4 CAMERA READY`;el('camera').className='value '+cls(live>0);
for(const [id] of names){const c=(s.cameras||{})[id]||{};el(id+'-text').textContent=c.ready?'实时':(c.error&&c.error.includes('busy')?'占用':'离线');el(id+'-dot').className='dot '+(c.ready?'live':'');}
el('cloud').textContent=s.cloud_shadow.status==='ok'?'SHADOW ONLINE':'UNAVAILABLE';el('cloud').className='value '+cls(s.cloud_shadow.status==='ok');el('exec').textContent=String(s.execution_permitted).toUpperCase();el('routes').textContent=s.motion_routes;
el('teleop').textContent=yn(s.processes.teleop_execute);el('teleop').className=cls(!s.processes.teleop_execute);el('planner').textContent=yn(s.processes.motion_planner);el('planner').className=cls(!s.processes.motion_planner);el('uptime').textContent=s.uptime_s+' s';el('clock').textContent=new Date().toLocaleTimeString();}catch(e){el('activity').textContent='MONITOR OFFLINE';el('activity').className='value bad'}}
setInterval(refreshImages,700);setInterval(refreshStatus,1000);refreshImages();refreshStatus();
</script></body></html>"""


if __name__ == "__main__":
    raise SystemExit(main())
