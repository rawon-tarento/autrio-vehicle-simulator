"""The lab CAN bus: one localhost multicast port per interface, one datagram per frame.

Every reader joins the group and sees every frame on that interface, like a CAN
socket. A datagram is {"id": 599, "ext": false, "data": "00a1...", "src": "car"}.
Readers timestamp a frame when it arrives; the wire carries no time.
"""

from __future__ import annotations

import argparse
import json
import socket
import struct
import uuid
from dataclasses import dataclass
from datetime import datetime
from typing import Iterable

from vehicle_simulator.encode import Frame

GROUP = "239.255.70.1"
PORT = 47000
HOST = "127.0.0.1"


@dataclass(frozen=True)
class Received:
    interface: str
    frame_id: int
    extended: bool
    data: bytes
    src: str


def is_multicast(host: str) -> bool:
    try:
        first = int(host.split(".", 1)[0])
    except ValueError:
        return False
    return 224 <= first <= 239


class CanBus:
    """Send frames to their interface and read frames other nodes sent."""

    def __init__(self, interfaces: dict[str, int], *, group: str = GROUP, src: str | None = None, listen: bool = True) -> None:
        if not interfaces:
            raise ValueError("at least one interface is needed")
        self.group = group
        self.interfaces = dict(interfaces)
        self.src = src or f"car-{uuid.uuid4().hex[:8]}"
        self._out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        if is_multicast(group):
            self._out.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 1)
            self._out.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 1)
        self._in: dict[str, socket.socket] = {}
        if listen:
            for name, port in self.interfaces.items():
                sock = join(group, port) if is_multicast(group) else _bind(group, port)
                sock.setblocking(False)
                self._in[name] = sock

    def send(self, frames: Iterable[Frame]) -> None:
        for frame in frames:
            port = self.interfaces.get(frame.interface)
            if port is None:
                raise ValueError(f"no interface {frame.interface}")
            body = {"id": frame.frame_id, "ext": frame.extended, "data": frame.data.hex(), "src": self.src}
            self._out.sendto(json.dumps(body, separators=(",", ":")).encode(), (self.group, port))

    def receive(self, limit: int = 1000) -> list[Received]:
        """Frames waiting on every interface, without blocking. Our own frames are skipped."""
        received: list[Received] = []
        for name, sock in self._in.items():
            while len(received) < limit:
                try:
                    payload, _addr = sock.recvfrom(65535)
                except BlockingIOError:
                    break
                frame = _parse(name, payload)
                if frame is not None and frame.src != self.src:
                    received.append(frame)
        return received

    def close(self) -> None:
        self._out.close()
        for sock in self._in.values():
            sock.close()


def _parse(interface: str, payload: bytes) -> Received | None:
    try:
        body = json.loads(payload.decode())
        return Received(interface, int(body["id"]), bool(body.get("ext", False)), bytes.fromhex(body["data"]), str(body.get("src", "")))
    except (ValueError, KeyError, TypeError, UnicodeDecodeError):
        return None


def _bind(host: str, port: int) -> socket.socket:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((host, port))
    return sock


def join(group: str = GROUP, port: int = PORT) -> socket.socket:
    """Bind the interface port and join the group. More than one process can join."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    if hasattr(socket, "SO_REUSEPORT"):
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
    sock.bind(("", port))
    membership = struct.pack("4s4s", socket.inet_aton(group), socket.inet_aton("0.0.0.0"))
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, membership)
    return sock


def listen(group: str = GROUP, port: int = PORT) -> None:
    """Print each frame on one interface with its arrival time."""
    sock = join(group, port)
    print(f"listening on {group}:{port}", flush=True)
    while True:
        payload, _addr = sock.recvfrom(65535)
        stamp = datetime.now().astimezone().isoformat(timespec="milliseconds")
        print(f"{stamp} {payload.decode()}", flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(prog="python -m vehicle_simulator.udp")
    parser.add_argument("--group", default=GROUP)
    parser.add_argument("--port", type=int, default=PORT)
    args = parser.parse_args()
    listen(args.group, args.port)
