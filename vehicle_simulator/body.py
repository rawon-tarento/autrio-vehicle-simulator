"""Doors, closures, lock, horn, lights, windows, seats, climate, and charge port. Plant names only."""

from __future__ import annotations

from dataclasses import dataclass, field

CLOSED = "closed"
OPENING = "opening"
OPEN = "open"
CLOSING = "closing"

DOORS = ("front_left", "front_right", "rear_left", "rear_right")
SEATS = ("front_left", "front_right", "rear_left", "rear_center", "rear_right")
LIGHT_MODES = ("auto", "on", "off", "parking")
WINDOW_POSITIONS = ("closed", "vent", "open")
PLUGS = ("ac", "fast")
CLIMATE_MODES = ("off", "on", "precondition", "keep", "dog", "camp")
KEEP_MODES = ("keep", "dog", "camp")


@dataclass
class Closure:
    """Something that takes time to open or close: a door latch, the trunk, the charge port door."""

    travel_s: float
    state: str = CLOSED
    left_s: float = field(default=0.0, repr=False)

    @property
    def moving(self) -> bool:
        return self.state in (OPENING, CLOSING)

    def open(self) -> None:
        if self.state in (OPEN, OPENING):
            return
        self.state = OPENING
        self.left_s = self.travel_s

    def close(self) -> None:
        if self.state in (CLOSED, CLOSING):
            return
        self.state = CLOSING
        self.left_s = self.travel_s

    def step(self, dt_s: float) -> None:
        if not self.moving:
            return
        self.left_s -= dt_s
        if self.left_s <= 0.0:
            self.state = OPEN if self.state == OPENING else CLOSED
            self.left_s = 0.0

    def settled(self) -> str:
        """Where it ends up. A restart does not resume half a movement."""
        if self.state == OPENING:
            return OPEN
        if self.state == CLOSING:
            return CLOSED
        return self.state

    def set(self, state: str) -> None:
        if state not in (OPEN, CLOSED):
            raise ValueError(f"closure state must be open or closed, got {state}")
        self.state = state
        self.left_s = 0.0


@dataclass
class Body:
    doors: dict[str, Closure]
    trunk: Closure
    frunk: Closure
    charge_port: Closure
    setpoint_c: float
    cabin_c: float
    charge_limit_pct: float
    locked: bool = True
    horn_until_s: float = -1.0
    lights: str = "auto"
    windows: str = "closed"
    seat_heat: dict[str, int] = field(default_factory=lambda: {seat: 0 for seat in SEATS})
    climate_mode: str = "off"
    plug: str = ""
    charge_enabled: bool = True
    ac_limit_a: float = 32.0
    charge_schedule: str = ""
    battery_precondition: bool = False

    @property
    def climate_on(self) -> bool:
        return self.climate_mode != "off"

    def closures(self) -> list[Closure]:
        return [*self.doors.values(), self.trunk, self.frunk, self.charge_port]

    def step(self, dt_s: float) -> None:
        for closure in self.closures():
            closure.step(dt_s)

    def any_door_open(self) -> bool:
        return any(door.state != CLOSED for door in self.doors.values())

    def doors_open(self) -> int:
        return sum(door.state != CLOSED for door in self.doors.values())

    def any_closure_open(self) -> bool:
        return self.any_door_open() or self.trunk.state != CLOSED or self.frunk.state != CLOSED

    def any_moving(self) -> bool:
        return any(closure.moving for closure in self.closures())
