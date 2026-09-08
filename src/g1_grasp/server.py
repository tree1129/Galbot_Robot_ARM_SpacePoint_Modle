"""Authenticated, non-actuating HTTP service for G1 shadow grasp plans."""

from __future__ import annotations

import argparse
import hmac
import json
import os
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any

from .pipeline import GraspCoordinator
from .reachability import DualArmReachability
from .serialization import parse_scene

MAX_REQUEST_BYTES = 2 * 1024 * 1024
SERVICE_VERSION = "0.2.0-shadow"


class ShadowApplication:
    def __init__(self, data_dir: Path, token: str):
        if len(token) < 32:
            raise RuntimeError("GRASP_SERVICE_TOKEN must contain at least 32 characters")
        started = time.perf_counter()
        self.coordinator = GraspCoordinator(DualArmReachability(data_dir))
        self.map_load_ms = round((time.perf_counter() - started) * 1000, 2)
        self.token = token
        self.started_at = time.time()
        self.request_count = 0
        self.rejected_count = 0

    def health(self) -> dict[str, Any]:
        return {
            "status": "ok",
            "version": SERVICE_VERSION,
            "mode": "shadow",
            "maps": "loaded",
            "map_load_ms": self.map_load_ms,
            "execution_permitted": False,
            "uptime_s": round(time.time() - self.started_at, 1),
        }

    def plan(self, command: str, scene: dict[str, Any]) -> dict[str, Any]:
        if not isinstance(command, str) or not 1 <= len(command.strip()) <= 256:
            raise ValueError("command must contain 1..256 characters")
        candidates, obstacles, safety, current_tcp = parse_scene(scene)
        self.request_count += 1
        plan = self.coordinator.plan(command, candidates, obstacles, safety, current_tcp)
        if plan.status != "READY_FOR_SHADOW_REVIEW":
            self.rejected_count += 1
        result = plan.to_dict()
        # Defense in depth: the network boundary cannot emit an execution permit.
        result["execution_permitted"] = False
        return result


def make_handler(application: ShadowApplication):
    class Handler(BaseHTTPRequestHandler):
        server_version = "G1Shadow/0.2"

        def log_message(self, format: str, *args: Any) -> None:
            # Do not log request bodies, commands, tokens, or image data.
            print(f"shadow_http peer={self.client_address[0]} {format % args}", flush=True)

        def do_GET(self) -> None:  # noqa: N802
            if self.path == "/healthz":
                self._send_json(HTTPStatus.OK, application.health())
                return
            if self.path == "/metrics":
                self._send_json(
                    HTTPStatus.OK,
                    {
                        "requests": application.request_count,
                        "rejected": application.rejected_count,
                        "execution_permitted": False,
                    },
                )
                return
            self._send_json(HTTPStatus.NOT_FOUND, {"error": "not found"})

        def do_POST(self) -> None:  # noqa: N802
            if self.path != "/v1/plan":
                self._send_json(HTTPStatus.NOT_FOUND, {"error": "not found"})
                return
            if not self._authorized():
                self._send_json(HTTPStatus.UNAUTHORIZED, {"error": "unauthorized"})
                return
            try:
                size = int(self.headers.get("Content-Length", "0"))
                if size <= 0 or size > MAX_REQUEST_BYTES:
                    raise ValueError("request body size is invalid")
                payload = json.loads(self.rfile.read(size))
                result = application.plan(payload["command"], payload["scene"])
            except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
                self._send_json(HTTPStatus.BAD_REQUEST, {"error": str(exc)})
                return
            self._send_json(HTTPStatus.OK, result)

        def _authorized(self) -> bool:
            supplied = self.headers.get("Authorization", "")
            expected = f"Bearer {application.token}"
            return hmac.compare_digest(supplied, expected)

        def _send_json(self, status: HTTPStatus, payload: dict[str, Any]) -> None:
            body = json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode()
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.end_headers()
            self.wfile.write(body)

    return Handler


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8088)
    parser.add_argument("--data-dir", type=Path, default=Path("data"))
    args = parser.parse_args()
    token = os.environ.get("GRASP_SERVICE_TOKEN", "")
    application = ShadowApplication(args.data_dir, token)
    server = ThreadingHTTPServer((args.host, args.port), make_handler(application))
    print(
        f"G1 shadow service ready host={args.host} port={args.port} "
        f"map_load_ms={application.map_load_ms} execution_permitted=false",
        flush=True,
    )
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

