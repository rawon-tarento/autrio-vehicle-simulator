"""Operator console: a local web page with the map and the car's controls.

The page polls GET /state and posts operator commands to POST /command. A command
is queued for the car's own loop, exactly like a line on stdin, and the answer is
only "queued". What the car did shows up in the next state. GET /geocode looks up
places on OpenStreetMap Nominatim so the Drive panel can search without Google.
"""

from __future__ import annotations

import json
import os
import queue
import ssl
import threading
import urllib.error
import urllib.parse
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any

_PAGE = Path(__file__).resolve().parent / "ui" / "index.html"
_MAX_BODY = 64 * 1024
_GEOCODE_URL = "https://nominatim.openstreetmap.org/search"
_GEOCODE_UA = "vehicle-simulator-lab/1.0 (local operator console)"
# Prefer Bengaluru results; other cities still return.
_VIEWBOX = "77.45,13.15,77.75,12.82"


class Console:
    def __init__(self, host: str, port: int) -> None:
        self.commands: queue.Queue[dict[str, Any]] = queue.Queue()
        self._lock = threading.Lock()
        self._state = b"{}"
        self._route = b"null"
        self._server = ThreadingHTTPServer((host, port), _handler(self))
        self._server.daemon_threads = True
        self._thread = threading.Thread(target=self._server.serve_forever, name="ui", daemon=True)

    @property
    def url(self) -> str:
        host, port = self._server.server_address[:2]
        return f"http://{host}:{port}/"

    def start(self) -> None:
        self._thread.start()

    def publish(self, state: dict[str, Any], route: dict[str, Any] | None = None, *, route_changed: bool = False) -> None:
        state_bytes = json.dumps(state, separators=(",", ":")).encode()
        route_bytes = json.dumps(route, separators=(",", ":")).encode() if route_changed else None
        with self._lock:
            self._state = state_bytes
            if route_bytes is not None:
                self._route = route_bytes

    def snapshot(self) -> tuple[bytes, bytes]:
        with self._lock:
            return self._state, self._route

    def close(self) -> None:
        self._server.shutdown()
        self._server.server_close()


def _handler(console: Console) -> type[BaseHTTPRequestHandler]:
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self) -> None:
            path = self.path.split("?", 1)[0]
            if path in ("/", "/index.html"):
                self._send(200, _PAGE.read_bytes(), "text/html; charset=utf-8")
            elif path == "/state":
                self._send(200, console.snapshot()[0])
            elif path == "/route":
                self._send(200, console.snapshot()[1])
            elif path == "/geocode":
                self._geocode()
            else:
                self._send(404, b'{"error":"not found"}')

        def _geocode(self) -> None:
            query = urllib.parse.parse_qs(urllib.parse.urlparse(self.path).query)
            q = (query.get("q") or [""])[0].strip()
            if len(q) < 2:
                self._send(400, b'{"error":"q required"}')
                return
            try:
                hits = lookup_place(q)
            except (urllib.error.URLError, TimeoutError, json.JSONDecodeError, OSError):
                self._send(502, b'{"error":"lookup failed"}')
                return
            self._send(200, json.dumps({"results": hits}, separators=(",", ":")).encode())

        def do_POST(self) -> None:
            if self.path.split("?", 1)[0] != "/command":
                self._send(404, b'{"error":"not found"}')
                return
            length = int(self.headers.get("Content-Length") or 0)
            if length <= 0 or length > _MAX_BODY:
                self._send(400, b'{"error":"body required"}')
                return
            try:
                body = json.loads(self.rfile.read(length))
            except json.JSONDecodeError:
                self._send(400, b'{"error":"not json"}')
                return
            if not isinstance(body, dict) or not isinstance(body.get("command"), str):
                self._send(400, b'{"error":"command required"}')
                return
            console.commands.put(body)
            self._send(202, b'{"queued":true}')

        def _send(self, status: int, body: bytes, kind: str = "application/json") -> None:
            self.send_response(status)
            self.send_header("Content-Type", kind)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, *_args: Any) -> None:
            return

    return Handler


def lookup_place(q: str, *, limit: int = 6) -> list[dict[str, Any]]:
    """Nominatim search. Used by the console so the browser does not talk to OSM directly."""
    params = urllib.parse.urlencode(
        {
            "q": q,
            "format": "json",
            "limit": str(limit),
            "addressdetails": "0",
            "viewbox": _VIEWBOX,
            "bounded": "0",
        }
    )
    request = urllib.request.Request(
        f"{_GEOCODE_URL}?{params}",
        headers={"User-Agent": _GEOCODE_UA, "Accept": "application/json"},
    )
    with urllib.request.urlopen(request, context=_ssl_context(), timeout=8) as response:
        rows = json.loads(response.read().decode())
    hits: list[dict[str, Any]] = []
    for row in rows:
        try:
            hits.append(
                {
                    "label": str(row["display_name"]),
                    "lat": float(row["lat"]),
                    "lon": float(row["lon"]),
                }
            )
        except (KeyError, TypeError, ValueError):
            continue
    return hits


def _ssl_context() -> ssl.SSLContext:
    try:
        import certifi
    except ImportError:
        certifi = None
    if certifi is not None:
        return ssl.create_default_context(cafile=certifi.where())
    if os.path.isfile("/etc/ssl/cert.pem"):
        return ssl.create_default_context(cafile="/etc/ssl/cert.pem")
    return ssl.create_default_context()
