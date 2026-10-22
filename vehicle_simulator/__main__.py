"""Wall-clock car on the lab CAN bus.

Operator commands are JSON lines on stdin, or in command.json. They are what a
person in or at the car does, so they print nothing: the frames change.

{"command": "unlock"}
{"command": "door", "door": "front_left", "open": true}
{"command": "drive", "lat": 18.53, "lon": 73.84, "style": "smooth"}
{"command": "plug", "type": "ac"}
{"command": "climate", "on": true}

Requests from other nodes arrive as frames on the bus and are applied the same way.
"""

from __future__ import annotations

import argparse
import json
import os
import select
import signal
import sys
import time
from concurrent.futures import Future, ThreadPoolExecutor
from dataclasses import replace
from datetime import datetime
from pathlib import Path
from typing import Any

from vehicle_simulator.car import DrivePlan, Vehicle
from vehicle_simulator.card import WEATHER_SOURCES, load_card
from vehicle_simulator.environment import Environment, LiveWeather
from vehicle_simulator.encode import Encoder, Frame, load_encoder
from vehicle_simulator.udp import GROUP, CanBus
from vehicle_simulator.ui import Console

_TICK_S = 0.01
_PHYSICS_S = 0.1
_COMMANDS_S = 0.1
_SAVE_S = 10.0
_PRINT_S = 1.0


class Runner:
    def __init__(self, car: Vehicle, encoder: Encoder, bus: CanBus, *, verbose: bool) -> None:
        self.car = car
        self.encoder = encoder
        self.bus = bus
        self.verbose = verbose
        self._planner = ThreadPoolExecutor(max_workers=1)
        self._plan: Future[DrivePlan] | None = None
        self.last: dict[tuple[str, int], Frame] = {}
        self.destination: tuple[float, float] | None = None
        self.nav_error = ""
        self._shown_route: object = None
        self.route_id = 0

    def apply(self, body: dict[str, Any]) -> None:
        name = body["command"]
        if name not in _COMMANDS:
            raise KeyError(f"unknown command {name}")
        if name not in ("sleep", "asleep"):
            self.car.wake()
        if name == "drive":
            self._start_planning(body)
        elif name == "stop":
            self._plan = None
            self.destination = None
            self.car.stop()
        else:
            _COMMANDS[name](self.car, body)

    def bus_in(self) -> None:
        for rx in self.bus.receive():
            self.car.wake()
            changes = self.encoder.requests(rx.interface, rx.frame_id, rx.extended, rx.data, self.car.request_values())
            for name, value in changes:
                try:
                    self.car.apply_request(name, value)
                except (RuntimeError, ValueError, KeyError) as exc:
                    self._note(f"{name}={value}: {exc}")

    def planned(self) -> None:
        plan = self._plan
        if plan is None or not plan.done():
            return
        self._plan = None
        try:
            route = plan.result()
        except Exception as exc:
            self.nav_error = f"no route: {exc}"
            self.destination = None
            self._note(f"drive: {exc}")
            return
        try:
            self.car.start_drive(route)
        except Exception as exc:
            self.destination = None
            self._note(f"drive: {exc}")

    def ui_state(self) -> tuple[dict[str, Any], dict[str, Any] | None, bool]:
        """What the console shows, the route when it changed, and whether it changed."""
        car = self.car
        st = car.sample
        changed = car.route is not self._shown_route
        if changed:
            self._shown_route = car.route
            self.route_id += 1
        if not car.driving and car.route is None and self._plan is None:
            self.destination = None
        blocked = car.blocked_actions()
        if self._plan is not None:
            blocked.setdefault("drive", "finding a route")
        state = {
            "signals": car.signals(),
            "mode": st.mode,
            "position": [st.latitude_deg, st.longitude_deg],
            "power_kw": st.pack_power_kw,
            "planning": self._plan is not None,
            "destination": self.destination,
            "nav_error": self.nav_error,
            "route_id": self.route_id,
            "travelled_m": car.travelled_m,
            "home": [car.card.home_lat, car.card.home_lon],
            "blocked": blocked,
        }
        return state, car.route_view() if changed else None, changed

    def bus_out(self, now_s: float) -> None:
        if not self.car.awake:
            return
        frames = self.encoder.due(now_s, self.car.signals())
        if frames:
            self.bus.send(frames)
            for frame in frames:
                self.last[(frame.interface, frame.frame_id)] = frame

    def refused(self, exc: Exception) -> None:
        self._note(str(exc))

    def close(self) -> None:
        self._planner.shutdown(wait=False, cancel_futures=True)

    def _start_planning(self, body: dict[str, Any]) -> None:
        if self._plan is not None:
            raise RuntimeError("the car is already driving")
        reason = self.car.why_not("drive")
        if reason:
            raise RuntimeError(reason)
        lat = float(body["lat"])
        lon = float(body["lon"])
        self.destination = (lat, lon)
        self.nav_error = ""
        kwargs = {
            "style": body.get("style", "smooth"),
            "traffic": body.get("traffic", "light"),
            "speed_kph": body.get("speed_kph"),
            "origin": (self.car.sample.latitude_deg, self.car.sample.longitude_deg),
        }
        self._plan = self._planner.submit(self.car.plan_drive, lat, lon, **kwargs)

    def _note(self, text: str) -> None:
        if self.verbose:
            print(json.dumps({"ignored": text}), file=sys.stderr, flush=True)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="python -m vehicle_simulator")
    parser.add_argument("card", type=Path)
    parser.add_argument("--soc", type=float, default=None, help="start at this SOC (overrides saved state)")
    parser.add_argument("--odometer-km", type=float, default=None, help="start at this odometer (overrides saved state)")
    parser.add_argument("--state", type=Path, default=Path("state.json"))
    parser.add_argument("--fresh", action="store_true", help="ignore the saved state")
    parser.add_argument("--command-file", type=Path, default=Path("command.json"))
    parser.add_argument("--udp-host", default=GROUP)
    parser.add_argument("--quiet", action="store_true", help="do not print the frames each second")
    parser.add_argument("--verbose", action="store_true", help="say on stderr when the car could not do something")
    parser.add_argument("--ui-host", default="127.0.0.1")
    parser.add_argument("--ui-port", type=int, default=8090, help="operator console port, 0 to turn it off")
    parser.add_argument("--weather", choices=WEATHER_SOURCES, default="live", help="live Open-Meteo, a daily curve, or the card's fixed ambient")
    args = parser.parse_args(argv)

    card = replace(load_card(args.card), weather=args.weather, sensor_noise=True)
    live = LiveWeather(card) if args.weather == "live" else None
    environment = Environment(card, live)
    car = Vehicle(card, soc_pct=80.0 if args.soc is None else args.soc, odometer_km=args.odometer_km or 0.0, environment=environment)
    if args.state.is_file() and not args.fresh:
        car.restore(json.loads(args.state.read_text()))
        if args.soc is not None:
            car.restore({**car.snapshot(), "soc_pct": args.soc})
        if args.odometer_km is not None:
            car.sample.odometer_km = args.odometer_km
    encoder = _load_alias()
    encoder.check(car.signals())
    bus = CanBus(encoder.interfaces, group=args.udp_host)
    runner = Runner(car, encoder, bus, verbose=args.verbose)
    signal.signal(signal.SIGTERM, _exit)
    stdin = _Stdin()
    console = None
    if args.ui_port:
        console = Console(args.ui_host, args.ui_port)
        console.start()
        print(json.dumps({"console": console.url}), file=sys.stderr, flush=True)

    start = time.monotonic()
    next_physics = next_commands = next_save = next_print = start
    try:
        while True:
            now = time.monotonic()
            runner.bus_in()
            if now >= next_commands:
                for line in stdin.lines():
                    _handle(runner, line)
                _read_command_file(runner, args.command_file)
                while console is not None and not console.commands.empty():
                    _handle_body(runner, console.commands.get_nowait())
                runner.planned()
                if console is not None:
                    state, route, changed = runner.ui_state()
                    console.publish(state, route, route_changed=changed)
                next_commands += _COMMANDS_S
            if now - next_physics > 1.0:
                next_physics = now
            while now >= next_physics:
                car.step(_PHYSICS_S)
                next_physics += _PHYSICS_S
            runner.bus_out(now - start)
            if not args.quiet and now >= next_print:
                _print(runner)
                next_print = max(next_print + _PRINT_S, now)
            if now >= next_save:
                _save(car, args.state)
                next_save = now + _SAVE_S
            time.sleep(max(0.0, _TICK_S - (time.monotonic() - now)))
    except KeyboardInterrupt:
        pass
    finally:
        _save(car, args.state)
        runner.close()
        bus.close()
        if console is not None:
            console.close()
    return 0


def _exit(_signum: int, _frame: Any) -> None:
    raise SystemExit(0)


def _print(runner: Runner) -> None:
    car = runner.car
    line = {
        "ts": datetime.now().astimezone().isoformat(timespec="seconds"),
        "mode": car.sample.mode,
        "frames": [
            {"bus": frame.interface, "id": f"0x{frame.frame_id:03X}", "name": frame.name, "data": frame.data.hex()}
            for _key, frame in sorted(runner.last.items(), key=lambda item: (item[0][0], item[0][1]))
        ]
        if car.awake
        else [],
    }
    print(json.dumps(line), flush=True)


def _save(car: Vehicle, path: Path) -> None:
    tmp = path.with_name(path.name + ".tmp")
    tmp.write_text(json.dumps(car.snapshot(), indent=2))
    os.replace(tmp, path)


def _load_alias() -> Encoder:
    path = Path(__file__).resolve().parents[1] / "alias.json"
    if not path.is_file():
        raise SystemExit(f"alias file not found: {path}")
    return load_encoder(path)


def _read_command_file(runner: Runner, path: Path) -> None:
    if not path.is_file():
        return
    text = path.read_text()
    path.unlink()
    for line in text.splitlines():
        _handle(runner, line)


class _Stdin:
    """Whole lines from stdin without blocking. Reads the descriptor directly so no line waits in a buffer."""

    def __init__(self) -> None:
        self._fd = sys.stdin.fileno()
        self._pending = b""
        self._closed = False

    def lines(self) -> list[str]:
        while not self._closed:
            ready, _, _ = select.select([self._fd], [], [], 0)
            if not ready:
                break
            chunk = os.read(self._fd, 65536)
            if not chunk:
                self._closed = True
                break
            self._pending += chunk
        *complete, self._pending = self._pending.split(b"\n")
        return [line.decode(errors="replace") for line in complete]


def _handle(runner: Runner, line: str) -> None:
    text = line.strip()
    if not text:
        return
    try:
        body = json.loads(text)
    except json.JSONDecodeError as exc:
        print(json.dumps({"bad_command": text, "why": str(exc)}), file=sys.stderr, flush=True)
        return
    _handle_body(runner, body)


def _handle_body(runner: Runner, body: Any) -> None:
    try:
        runner.apply(body)
    except (KeyError, TypeError, AttributeError) as exc:
        print(json.dumps({"bad_command": body, "why": str(exc)}), file=sys.stderr, flush=True)
    except (RuntimeError, ValueError) as exc:
        runner.refused(exc)


def _flag(body: dict[str, Any], key: str = "open") -> bool:
    value = body[key]
    if not isinstance(value, bool):
        raise TypeError(f"{key} must be true or false")
    return value


_COMMANDS: dict[str, Any] = {
    "drive": None,
    "stop": None,
    "plug": lambda car, body: car.plug(body.get("type", body.get("profile", "ac"))),
    "unplug": lambda car, body: car.unplug(),
    "charging": lambda car, body: car.set_charging(_flag(body, "on")),
    "charge_limit": lambda car, body: car.set_charge_limit(float(body["pct"])),
    "charge": lambda car, body: car.charge(body.get("profile", "ac")),
    "lock": lambda car, body: car.lock(),
    "unlock": lambda car, body: car.unlock(),
    "set_lock": lambda car, body: car.set_lock(_flag(body, "locked")),
    "door": lambda car, body: car.door(str(body.get("door", "all")), _flag(body)),
    "trunk": lambda car, body: car.trunk(_flag(body)),
    "frunk": lambda car, body: car.frunk(_flag(body)),
    "charge_port": lambda car, body: car.charge_port(_flag(body)),
    "climate": lambda car, body: car.climate(_flag(body, "on")),
    "climate_mode": lambda car, body: car.climate_mode_set(str(body["mode"])),
    "battery_precondition": lambda car, body: car.battery_precondition(_flag(body, "on")),
    "overheat_protection": lambda car, body: car.set_overheat_protection(str(body["mode"])),
    "charge_current": lambda car, body: car.charge_current(float(body["amps"])),
    "charge_schedule": lambda car, body: car.charge_schedule(body.get("start")),
    "enter": lambda car, body: car.enter(),
    "leave": lambda car, body: car.leave(),
    "temperature": lambda car, body: car.set_temperature(float(body["c"])),
    "seat_heater": lambda car, body: car.seat_heater(str(body.get("seat", "all")), int(body["level"])),
    "honk": lambda car, body: car.honk(),
    "lights": lambda car, body: car.lights(str(body["mode"])),
    "windows": lambda car, body: car.windows(str(body["position"])),
    "sleep": lambda car, body: car.asleep(),
    "asleep": lambda car, body: car.asleep(),
    "wake": lambda car, body: car.wake(),
}


if __name__ == "__main__":
    raise SystemExit(main())
