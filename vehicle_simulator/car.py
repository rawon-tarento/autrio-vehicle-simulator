"""One live car. It idles on the clock until someone in or near the car does something, or a request arrives on the bus.

The car is a set of orthogonal states: power, motion, charging, climate, battery thermal, access.
PHYSICS.md lists every equation and every rule that ties them together.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from datetime import datetime
from typing import Any, Callable

from vehicle_simulator import charging as cs
from vehicle_simulator.body import (
    CLIMATE_MODES,
    CLOSED,
    DOORS,
    KEEP_MODES,
    LIGHT_MODES,
    OPEN,
    PLUGS,
    SEATS,
    WINDOW_POSITIONS,
    Body,
    Closure,
)
from vehicle_simulator.card import OVERHEAT_SETTINGS, VehicleCard
from vehicle_simulator.charging import ChargeSession
from vehicle_simulator.environment import Environment
from vehicle_simulator.geo import Route, haversine_m, heading_at, sample_route
from vehicle_simulator.lv import LowVoltage
from vehicle_simulator.pack import Limits, Pack
from vehicle_simulator.powertrain import DriveOutput, drive_step
from vehicle_simulator.roads import fetch_route
from vehicle_simulator.sensors import Sensors
from vehicle_simulator.state import Sample
from vehicle_simulator.styles import MotionLimits, limits_for, traffic_scale
from vehicle_simulator.thermal import OFF, ThermalDemand, ThermalResult, battery_demand, cabin_load, run_hvac, step_cabin

ASLEEP = "asleep"
WAKING = "waking"
AWAKE = "awake"

_NEAR_M = 30.0
_HONK_S = 0.5
_HORN_HOLD_S = 0.3
_SETPOINT_RANGE_C = (15.0, 28.0)
_CHARGE_LIMIT_RANGE_PCT = (50.0, 100.0)
_AC_LIMIT_RANGE_A = (5.0, 48.0)
_SHIFT_S = 0.5
_DEPART_S = 1.0
_PARK_S = 1.0
_LEAVE_DOOR_S = 2.0
_SCHEDULE_WINDOW_MIN = 6 * 60
_OVERHEAT_HYST_C = 3.0
_PARKING_LIGHTS_W = 10.0
_PLANNED_BRAKE_SHARE = 0.55
_NO_LIMITS = Limits(0.0, 0.0, 0.0, 0.0)

RouteFetch = Callable[..., Route]
Clock = Callable[[], datetime]


@dataclass(frozen=True)
class DrivePlan:
    route: Route
    limits: MotionLimits
    style: str
    pace: float


class Vehicle:
    """Parked at the home pin, awake, locked, nobody inside."""

    def __init__(
        self,
        card: VehicleCard,
        *,
        soc_pct: float = 80.0,
        odometer_km: float = 0.0,
        clock: Clock | None = None,
        environment: Environment | None = None,
    ) -> None:
        if not 0.0 <= soc_pct <= 100.0:
            raise ValueError("soc_pct must be between 0 and 100")
        self.card = card
        self._clock = clock or datetime.now
        self.env = environment or Environment(card)
        self.env.update(self._clock(), card.home_lat, card.home_lon, 0.0)
        ambient = self.env.ambient_c
        self.pack = Pack(card, soc_pct=soc_pct, temp_c=ambient)
        self.lv = LowVoltage(card)
        self.charge_session = ChargeSession(card)
        self.sensors = Sensors(card)
        self.sample = Sample(
            soc_pct=soc_pct,
            odometer_km=odometer_km,
            latitude_deg=card.home_lat,
            longitude_deg=card.home_lon,
            nominal_full_pack_kwh=card.nominal_full_pack_kwh,
            beginning_of_life_kwh=card.beginning_of_life_kwh,
        )
        self.body = Body(
            doors={door: Closure(card.door_travel_s) for door in DOORS},
            trunk=Closure(card.trunk_travel_s),
            frunk=Closure(card.frunk_travel_s),
            charge_port=Closure(card.charge_port_travel_s),
            setpoint_c=card.cabin_set_c,
            cabin_c=ambient,
            charge_limit_pct=card.charge_limit_pct,
            ac_limit_a=card.evse_ac_max_a,
        )
        self.overheat_setting = card.overheat_protection
        self.occupied = False
        self._power = AWAKE
        self._woke_for = "user"
        self._precharge_left_s = 0.0
        self._mode = "idle"
        self._phase = ""
        self._phase_s = 0.0
        self._route: Route | None = None
        self._s_m = 0.0
        self._end_m = 0.0
        self._heading = 0.0
        self._limits = limits_for(card, "smooth")
        self._traffic = 1.0
        self._accel = 0.0
        self._yaw = 0.0
        self._lat_accel = 0.0
        self._drive: DriveOutput | None = None
        self._thermal: ThermalResult = OFF
        self._other_hv_w = 0.0
        self._quiet_s = 0.0
        self._parked_s = 0.0
        self._cabin_precondition_s = 0.0
        self._battery_precondition_s = 0.0
        self._overheat_active = False
        self._maintenance = False
        self._port_idle_s = 0.0
        self._port_after_unplug = False
        self._leave_door_s: float | None = None
        self._walkaway_s: float | None = None
        self._gps = (card.home_lat, card.home_lon, 30.0)
        self._sync()

    # ----- state the outside world can read -----

    @property
    def awake(self) -> bool:
        return self._power != ASLEEP

    @property
    def power_state(self) -> str:
        return self._power

    @property
    def contactors_closed(self) -> bool:
        return self._power == AWAKE

    @property
    def driving(self) -> bool:
        return self._mode == "drive"

    @property
    def climate_mode(self) -> str:
        return self.body.climate_mode

    @property
    def overheat_active(self) -> bool:
        return self._overheat_active

    @property
    def route(self) -> Route | None:
        return self._route

    @property
    def travelled_m(self) -> float:
        """Distance along the current route. 0 when parked."""
        return self._s_m if self._route is not None else 0.0

    def signals(self) -> dict[str, Any]:
        """Everything the car knows, under plant names. The alias picks what goes on the bus."""
        values = self.sample.canonicals()
        values.update(self._status())
        values.update(self.request_values())
        return values

    def route_view(self) -> dict[str, Any] | None:
        """The road being driven: points, and the distance along it at each point."""
        route = self._route
        if route is None:
            return None
        return {
            "points": [[round(p.lat, 6), round(p.lon, 6)] for p in route.points],
            "s_m": [round(s, 1) for s in route.s_m],
            "length_m": route.length_m,
        }

    def request_values(self) -> dict[str, Any]:
        """What the car's own controls currently ask for. A request on the bus that differs from this is applied."""
        body = self.body
        mode = body.climate_mode
        values: dict[str, Any] = {
            "request.lock": "idle",
            "request.horn": 0,
            "request.closure": "idle",
            "request.charge_enable": int(body.charge_enabled),
            "request.charge_port_open": 0,
            "request.charge_port_close": 0,
            "request.charge_limit_pct": body.charge_limit_pct,
            "request.climate_power": "precondition" if mode == "precondition" else ("off" if mode == "off" else "on"),
            "request.climate_setpoint_c": body.setpoint_c,
            "request.climate_keep": mode if mode in KEEP_MODES else "off",
            "request.battery_precondition": int(body.battery_precondition),
            "request.ac_current_limit_a": body.ac_limit_a,
            "request.lights": body.lights,
            "request.windows": "idle",
        }
        for seat in SEATS:
            values[f"request.seat_heat_{seat}"] = body.seat_heat[seat]
        return values

    def apply_request(self, name: str, value: Any) -> None:
        """A request decoded from the bus. Raises when the car cannot do it. The caller decides whether to say so."""
        handler = self._request_handlers().get(name)
        if handler is None:
            raise KeyError(f"unknown request {name}")
        handler(value)

    def why_not(self, action: str) -> str | None:
        """Why this operator action will not run right now. None means it would. The console uses the same answers."""
        driving = self._mode == "drive"
        moving = driving and self.sample.speed_kph > 1.0
        plugged = bool(self.body.plug)
        locked = self.body.locked
        floor = self.card.keep_climate_min_soc_pct
        if action == "drive":
            if driving:
                return "the car is already driving"
            if plugged:
                return "unplug before driving"
            return None
        if action == "stop":
            return None if driving else "the car is not driving"
        if action == "sleep":
            if self._power == ASLEEP:
                return "the car is already asleep"
            if driving:
                return "the car is driving"
            if self.charge_session.active or self._charge_wants_power(self._clock()):
                return "the car stays awake while it charges"
            if self.body.climate_mode in KEEP_MODES:
                return f"{self.body.climate_mode} mode keeps the car awake"
            return None
        if action == "enter":
            if driving:
                return "the car is driving"
            if self.occupied:
                return "someone is already in the car"
            return None
        if action == "leave":
            if moving:
                return "the car is moving"
            if not self.occupied:
                return "nobody is in the car"
            return None
        if action == "plug_ac":
            return self._why_not_plug("ac")
        if action == "plug_fast":
            return self._why_not_plug("fast")
        if action == "unplug":
            return None if plugged else "no cable"
        if action == "charge_port_open":
            return "the charge port does not open while driving" if driving else None
        if action == "charge_port_close":
            return "the cable is in the port" if plugged else None
        if action == "lock":
            return "already locked" if locked else None
        if action == "unlock":
            return "already unlocked" if not locked else None
        if action.endswith("_open") and action.startswith("door_"):
            if locked:
                return "the car is locked"
            if moving:
                return "the car is moving"
            return None
        if action in ("trunk_open", "frunk_open"):
            if driving:
                return "the car is driving"
            if locked:
                return "the car is locked"
            return None
        if action in ("climate_keep", "climate_dog", "climate_camp"):
            mode = action.split("_", 1)[1]
            if driving:
                return f"{mode} mode is for a parked car"
            if self.pack.soc_pct < floor:
                return f"{mode} mode needs at least {floor:.0f} % charge"
            return None
        return None

    def blocked_actions(self) -> dict[str, str]:
        """Every operator action the console must disable, with the reason the car would refuse it."""
        keys = [
            "drive",
            "stop",
            "sleep",
            "enter",
            "leave",
            "plug_ac",
            "plug_fast",
            "unplug",
            "charge_port_open",
            "charge_port_close",
            "lock",
            "unlock",
            "trunk_open",
            "frunk_open",
            "climate_keep",
            "climate_dog",
            "climate_camp",
        ]
        keys.extend(f"door_{name}_open" for name in DOORS)
        return {key: reason for key in keys if (reason := self.why_not(key))}

    def _why_not_plug(self, kind: str) -> str | None:
        if self._mode == "drive":
            return "park before plugging in"
        if self.body.plug == kind:
            return "the cable is already in"
        if self.body.plug:
            return "unplug before plugging in a different charger"
        return None

    def _refuse(self, action: str) -> None:
        reason = self.why_not(action)
        if reason:
            raise RuntimeError(reason)

    # ----- the clock -----

    def step(self, dt_s: float = 1.0) -> Sample:
        if dt_s <= 0:
            raise ValueError("dt_s must be positive")
        now = self._clock()
        st = self.sample
        st.t_s += dt_s
        self.env.update(now, st.latitude_deg, st.longitude_deg, dt_s)
        if self._power == ASLEEP:
            self._wake_if_needed(now)
        self._power_step(dt_s)
        self.body.step(dt_s)
        self._rules(dt_s)
        contactors = self.contactors_closed
        limits = self.pack.limits() if contactors else _NO_LIMITS

        traction_w = 0.0
        if self._mode == "drive":
            traction_w = self._drive_step(dt_s, limits)
        else:
            self._parked_s += dt_s

        if contactors:
            self._thermal = self._run_thermal(dt_s)
        else:
            self._thermal = OFF
            self.body.cabin_c = step_cabin(
                self.card, self.body.cabin_c, dt_s, ambient_c=self.env.ambient_c, load=self._cabin_load(), hvac_w=0.0
            )
        lv_loads = self._lv_loads_w() if self.awake else self.card.sleep_w
        dcdc_in = self.lv.step(dt_s, loads_w=lv_loads, dcdc_on=contactors)
        other_hv = self._thermal.hv_el_w + dcdc_in
        self._other_hv_w = other_hv
        hv_loads = traction_w + other_hv

        self._charge_update(dt_s, now)
        session = self.charge_session
        shore_w = 0.0
        if contactors and session.flowing:
            into_pack = limits.charge_w if session.stage in (cs.RAMP, cs.CHARGING) else 0.0
            shore_w = min(session.available_w(self.body.ac_limit_a, self.pack.voltage_v), hv_loads + into_pack)
        pack_w = self.pack.step(
            hv_loads - shore_w,
            dt_s,
            contactors_closed=contactors,
            ambient_c=self.env.ambient_c,
            cooling_w=self._thermal.battery_cool_w,
            heating_w=self._thermal.battery_heat_w,
            limit_pct=self.body.charge_limit_pct if session.plugged else None,
        )
        delivered = hv_loads - pack_w if shore_w > 0 else 0.0
        session.deliver(delivered, self.pack.voltage_v, -pack_w, dt_s)

        if self.awake:
            self.sensors.tick(dt_s)
        self._gps = self.sensors.gps(st.latitude_deg, st.longitude_deg, st.speed_kph, dt_s)
        self._sync()
        self._auto_sleep(dt_s)
        return self.sample

    # ----- someone in or at the car -----

    def wake(self, reason: str = "user") -> None:
        """Opening a door, touching the screen, a phone key nearby, or traffic on the bus."""
        self._quiet_s = 0.0
        if self._power == ASLEEP:
            self._power = WAKING
            self._precharge_left_s = self.card.precharge_s
            self._woke_for = reason
            self.sensors.asleep()
        elif reason == "user":
            self._woke_for = "user"
        self._sync()

    def asleep(self) -> None:
        """The driver walked away and the car went to sleep straight away."""
        self._refuse("sleep")
        self.occupied = False
        self.body.locked = True
        for door in self.body.doors.values():
            door.set(CLOSED)
        self._cabin_off()
        self._sleep_now()

    sleep = asleep

    def enter(self) -> None:
        """Someone with a key gets in: unlocks, opens and shuts the driver door, sits down."""
        self._refuse("enter")
        self.body.locked = False
        self.occupied = True
        self._walkaway_s = None
        self._leave_door_s = None
        self.body.doors["front_left"].set(CLOSED)

    def leave(self) -> None:
        """The driver gets out: Park, door opens, door shuts, the phone key walks away."""
        self._refuse("leave")
        if self._mode == "drive":
            self._park()
        self.occupied = False
        self.body.locked = False
        self.body.doors["front_left"].open()
        self._leave_door_s = _LEAVE_DOOR_S
        self._walkaway_s = None

    def plan_drive(
        self,
        lat: float,
        lon: float,
        *,
        style: str = "smooth",
        traffic: str = "light",
        speed_kph: float | None = None,
        fetch: RouteFetch = fetch_route,
        cache_dir=None,
        origin: tuple[float, float] | None = None,
    ) -> DrivePlan:
        """Look up the road. This does not touch the car, so it can run off the clock."""
        limits = limits_for(self.card, style)
        pace = traffic_scale(traffic)
        cruise = self.card.cruise_speed_kph * limits.speed_scale * pace
        if speed_kph is not None:
            cruise = min(float(speed_kph), limits.max_speed_kph) * pace
        start = origin or (self.sample.latitude_deg, self.sample.longitude_deg)
        route = fetch(start, (lat, lon), speed_kph=cruise, lat_accel_limit=limits.max_lat_accel_mps2, cache_dir=cache_dir)
        return DrivePlan(route, limits, style, pace)

    def start_drive(self, plan: DrivePlan) -> None:
        self._check_can_drive()
        self._begin(plan.route, plan.limits, plan.style, plan.pace)

    def drive_to(self, lat: float, lon: float, **kwargs: Any) -> None:
        """Follow the road from the current pin to lat, lon."""
        self._check_can_drive()
        self.start_drive(self.plan_drive(lat, lon, **kwargs))

    def follow(self, route: Route, *, style: str = "smooth", traffic: str = "light") -> None:
        """Follow a route that already starts at the current pin. Tests use this."""
        self._check_can_drive()
        start = route.points[0]
        gap = haversine_m(self.sample.latitude_deg, self.sample.longitude_deg, start.lat, start.lon)
        if gap > _NEAR_M:
            raise RuntimeError(f"route starts {gap:.0f} m from the car")
        self._begin(route, limits_for(self.card, style), style, traffic_scale(traffic))

    def stop(self) -> None:
        """Pull over: brake to a stop where you are, then park."""
        if self.why_not("stop"):
            return
        speed = self.sample.speed_kph / 3.6
        stopping_m = speed * speed / (2.0 * self._limits.max_brake_mps2)
        self._end_m = min(self._end_m, self._s_m + stopping_m + 0.5)

    def plug(self, kind: str) -> None:
        """Plug the cable in. Pressing the cable button opens the port door."""
        if kind not in PLUGS:
            raise ValueError(f"plug must be one of {', '.join(PLUGS)}")
        self._refuse("plug_fast" if kind == "fast" else "plug_ac")
        self.body.charge_port.set(OPEN)
        self.body.plug = kind
        self.charge_session.plug(kind)
        self._port_idle_s = 0.0
        self._port_after_unplug = False

    def unplug(self) -> None:
        """Press the cable button: charging stops, the latch lets go, the cable comes out."""
        self._refuse("unplug")
        self.body.plug = ""
        self.charge_session.unplug()
        self._port_idle_s = 0.0
        self._port_after_unplug = True

    def set_charging(self, on: bool) -> None:
        """The charge on/off control. With no cable it is only the setting for next time."""
        self.body.charge_enabled = bool(on)

    def charge(self, profile: str) -> None:
        """Plug in and charge."""
        if self.body.plug != profile:
            self.plug(profile)
        self.set_charging(True)

    def set_charge_limit(self, pct: float) -> None:
        low, high = _CHARGE_LIMIT_RANGE_PCT
        pct = float(pct)
        if not low <= pct <= high:
            raise ValueError(f"charge limit must be {low:.0f} to {high:.0f} %")
        self.body.charge_limit_pct = pct

    def charge_current(self, amps: float) -> None:
        low, high = _AC_LIMIT_RANGE_A
        amps = float(amps)
        if not low <= amps <= high:
            raise ValueError(f"charge current must be {low:.0f} to {high:.0f} A")
        self.body.ac_limit_a = amps

    def charge_schedule(self, start: str | None) -> None:
        """Start charging at HH:MM local time. None or empty charges as soon as plugged in."""
        if not start:
            self.body.charge_schedule = ""
            return
        _parse_hhmm(start)
        self.body.charge_schedule = start

    def lock(self) -> None:
        """Locking from outside means the driver left. Keep, Dog, and Camp hold the cabin; plain climate stops."""
        self.body.locked = True
        if self._mode == "drive":
            return
        if self.body.climate_mode != "camp":
            self.occupied = False
        if self.body.climate_mode == "on":
            self.body.climate_mode = "off"
        if self.body.climate_mode not in KEEP_MODES:
            for seat in self.body.seat_heat:
                self.body.seat_heat[seat] = 0

    def unlock(self) -> None:
        self.body.locked = False

    def set_lock(self, locked: bool) -> None:
        if locked:
            self.lock()
        else:
            self.unlock()

    def door(self, door: str, open_: bool) -> None:
        names = DOORS if door == "all" else (door,)
        for name in names:
            if name not in self.body.doors:
                raise ValueError(f"door must be one of {', '.join(DOORS)}, or all")
        if open_:
            self._refuse("door_front_left_open")
            if self._mode == "drive" and "front_left" in names:
                self._park()
        for name in names:
            closure = self.body.doors[name]
            if open_:
                closure.open()
            else:
                closure.close()

    def trunk(self, open_: bool) -> None:
        if open_:
            self._refuse("trunk_open")
            self.body.trunk.open()
        else:
            self.body.trunk.close()

    def frunk(self, open_: bool) -> None:
        if open_:
            self._refuse("frunk_open")
            self.body.frunk.open()
        else:
            self.body.frunk.close()

    def charge_port(self, open_: bool) -> None:
        if open_:
            self._refuse("charge_port_open")
            self.body.charge_port.open()
            self._port_idle_s = 0.0
            self._port_after_unplug = False
        else:
            self._refuse("charge_port_close")
            self.body.charge_port.close()

    def climate(self, on: bool) -> None:
        self.climate_mode_set("on" if on else "off")

    def climate_mode_set(self, mode: str) -> None:
        if mode not in CLIMATE_MODES:
            raise ValueError(f"climate mode must be one of {', '.join(CLIMATE_MODES)}")
        if mode in KEEP_MODES:
            self._refuse(f"climate_{mode}")
        if mode == "precondition" and (self.occupied or self._mode == "drive"):
            mode = "on"
        self.body.climate_mode = mode
        self._cabin_precondition_s = 0.0
        if mode != "off":
            self._overheat_active = False

    def battery_precondition(self, on: bool) -> None:
        self.body.battery_precondition = bool(on)
        self._battery_precondition_s = 0.0

    def set_overheat_protection(self, setting: str) -> None:
        if setting not in OVERHEAT_SETTINGS:
            raise ValueError(f"overheat protection must be one of {', '.join(OVERHEAT_SETTINGS)}")
        self.overheat_setting = setting

    def set_temperature(self, celsius: float) -> None:
        low, high = _SETPOINT_RANGE_C
        celsius = float(celsius)
        if not low <= celsius <= high:
            raise ValueError(f"temperature must be {low:.0f} to {high:.0f} C")
        self.body.setpoint_c = celsius

    def seat_heater(self, seat: str, level: int) -> None:
        level = int(level)
        if not 0 <= level <= 3:
            raise ValueError("seat heater level must be 0 to 3")
        names = SEATS if seat == "all" else (seat,)
        for name in names:
            if name not in self.body.seat_heat:
                raise ValueError(f"seat must be one of {', '.join(SEATS)}, or all")
        for name in names:
            self.body.seat_heat[name] = level

    def honk(self) -> None:
        self.body.horn_until_s = max(self.body.horn_until_s, self.sample.t_s + _HONK_S)

    def lights(self, mode: str) -> None:
        if mode not in LIGHT_MODES:
            raise ValueError(f"lights must be one of {', '.join(LIGHT_MODES)}")
        self.body.lights = mode

    def windows(self, position: str) -> None:
        if position not in WINDOW_POSITIONS:
            raise ValueError(f"windows must be one of {', '.join(WINDOW_POSITIONS)}")
        self.body.windows = position

    # ----- restart -----

    def snapshot(self) -> dict[str, Any]:
        body = self.body
        st = self.sample
        pack = self.pack
        return {
            "version": 2,
            "soc_pct": pack.soc_pct,
            "odometer_km": st.odometer_km,
            "latitude_deg": st.latitude_deg,
            "longitude_deg": st.longitude_deg,
            "elevation_m": st.elevation_m,
            "heading_deg": st.heading_deg,
            "pack_temp_c": pack.temp_c,
            "total_charge_kwh": pack.total_charge_kwh,
            "total_discharge_kwh": pack.total_discharge_kwh,
            "lv_soc_pct": self.lv.soc_pct,
            "asleep": self._power == ASLEEP,
            "occupied": self.occupied,
            "locked": body.locked,
            "doors": {name: door.settled() for name, door in body.doors.items()},
            "trunk": body.trunk.settled(),
            "frunk": body.frunk.settled(),
            "charge_port": body.charge_port.settled(),
            "plug": body.plug,
            "charge_enabled": body.charge_enabled,
            "charge_limit_pct": body.charge_limit_pct,
            "ac_limit_a": body.ac_limit_a,
            "charge_schedule": body.charge_schedule,
            "climate_mode": body.climate_mode,
            "battery_precondition": body.battery_precondition,
            "overheat_protection": self.overheat_setting,
            "setpoint_c": body.setpoint_c,
            "cabin_c": body.cabin_c,
            "seat_heat": dict(body.seat_heat),
            "lights": body.lights,
            "windows": body.windows,
        }

    def restore(self, data: dict[str, Any]) -> None:
        """Load a snapshot. A drive in progress does not survive a restart: the car is parked where it stopped."""
        body = self.body
        st = self.sample
        pack = self.pack
        card = self.card
        soc = float(data["soc_pct"])
        if not 0.0 <= soc <= 100.0:
            raise ValueError("saved soc_pct is out of range")
        pack.soc_pct = soc
        pack.v_rc = 0.0
        pack.temp_c = float(data.get("pack_temp_c", self.env.ambient_c))
        pack.total_charge_kwh = float(data.get("total_charge_kwh", 0.0))
        pack.total_discharge_kwh = float(data.get("total_discharge_kwh", 0.0))
        self.lv.soc_pct = float(data.get("lv_soc_pct", self.lv.soc_pct))
        st.odometer_km = float(data["odometer_km"])
        st.latitude_deg = float(data["latitude_deg"])
        st.longitude_deg = float(data["longitude_deg"])
        st.elevation_m = float(data.get("elevation_m", 0.0))
        st.heading_deg = float(data.get("heading_deg", 0.0))
        self._heading = st.heading_deg
        self.occupied = bool(data.get("occupied", False))
        body.locked = bool(data.get("locked", True))
        for name, state in (data.get("doors") or {}).items():
            if name in body.doors:
                body.doors[name].set(state)
        body.trunk.set(data.get("trunk", CLOSED))
        body.frunk.set(data.get("frunk", CLOSED))
        body.charge_port.set(data.get("charge_port", CLOSED))
        plug = data.get("plug", "")
        body.plug = plug if plug in PLUGS else ""
        if body.plug:
            self.charge_session.plug(body.plug)
        else:
            self.charge_session.unplug()
        body.charge_enabled = bool(data.get("charge_enabled", True))
        body.charge_limit_pct = float(data.get("charge_limit_pct", card.charge_limit_pct))
        body.ac_limit_a = float(data.get("ac_limit_a", card.evse_ac_max_a))
        body.charge_schedule = str(data.get("charge_schedule", "") or "")
        mode = data.get("climate_mode")
        if mode is None:
            mode = "on" if data.get("climate_on") else "off"
        body.climate_mode = mode if mode in CLIMATE_MODES else "off"
        body.battery_precondition = bool(data.get("battery_precondition", False))
        setting = data.get("overheat_protection", card.overheat_protection)
        self.overheat_setting = setting if setting in OVERHEAT_SETTINGS else card.overheat_protection
        body.setpoint_c = float(data.get("setpoint_c", card.cabin_set_c))
        body.cabin_c = float(data.get("cabin_c", self.env.ambient_c))
        for seat, level in (data.get("seat_heat") or {}).items():
            if seat in body.seat_heat:
                body.seat_heat[seat] = int(level)
        body.lights = data.get("lights") if data.get("lights") in LIGHT_MODES else "auto"
        body.windows = data.get("windows") if data.get("windows") in WINDOW_POSITIONS else "closed"
        self._route = None
        self._mode = "idle"
        self._phase = ""
        self._parked_s = 0.0
        st.speed_kph = 0.0
        st.gear = "park"
        if data.get("asleep"):
            self._sleep_now()
        else:
            self._power = AWAKE
        self._sync()

    # ----- power -----

    def _power_step(self, dt_s: float) -> None:
        if self._power != WAKING:
            return
        self._precharge_left_s -= dt_s
        if self._precharge_left_s <= 0.0:
            self._precharge_left_s = 0.0
            self._power = AWAKE

    def _wake_if_needed(self, now: datetime) -> None:
        if self.lv.needs_topup:
            self._maintenance = True
            self.wake("lv")
        elif self._charge_wants_power(now):
            self.wake("charge")
        elif self._overheat_wanted() and self.body.cabin_c > self.card.overheat_on_c:
            self.wake("overheat")

    def _sleep_now(self) -> None:
        self._power = ASLEEP
        self._precharge_left_s = 0.0
        self._maintenance = False
        self._overheat_active = False
        self._quiet_s = 0.0
        self.body.horn_until_s = -1.0
        self.body.battery_precondition = False
        for seat in self.body.seat_heat:
            self.body.seat_heat[seat] = 0
        if self.body.climate_mode in ("on", "precondition"):
            self.body.climate_mode = "off"
        self.sensors.asleep()
        self._sync()

    def _busy(self) -> bool:
        body = self.body
        return (
            self._power == WAKING
            or self._mode == "drive"
            or self.occupied
            or body.climate_mode != "off"
            or self._overheat_active
            or body.battery_precondition
            or self._thermal.battery_cool_w > 0
            or self._thermal.battery_heat_w > 0
            or self.charge_session.active
            or self._maintenance
            or self._horn_on()
            or body.any_door_open()
            or body.any_moving()
            or self._leave_door_s is not None
            or self._walkaway_s is not None
        )

    def _auto_sleep(self, dt_s: float) -> None:
        if self._power == ASLEEP:
            return
        if self._maintenance and self.lv.topped_up:
            self._maintenance = False
        limit = self.card.auto_sleep_after_s
        if self._busy():
            self._quiet_s = 0.0
            return
        if self._woke_for != "user":
            self._sleep_now()
            return
        if limit is None:
            return
        self._quiet_s += dt_s
        if self._quiet_s >= limit:
            self._sleep_now()

    # ----- the car's own rules -----

    def _rules(self, dt_s: float) -> None:
        body = self.body
        card = self.card
        if self._leave_door_s is not None:
            self._leave_door_s -= dt_s
            if self._leave_door_s <= 0:
                self._leave_door_s = None
                body.doors["front_left"].close()
                self._walkaway_s = card.walk_away_lock_s
        if self._walkaway_s is not None and not body.any_door_open():
            self._walkaway_s -= dt_s
            if self._walkaway_s <= 0:
                self._walkaway_s = None
                if not self.occupied:
                    self.lock()
        if self.awake and body.charge_port.state == OPEN and not body.plug:
            self._port_idle_s += dt_s
            wait = card.port_close_after_unplug_s if self._port_after_unplug else card.port_close_idle_s
            if self._port_idle_s >= wait:
                body.charge_port.close()
                self._port_after_unplug = False
        mode = body.climate_mode
        if mode == "precondition":
            self._cabin_precondition_s += dt_s
            if self.occupied or self._mode == "drive":
                body.climate_mode = "on"
            elif self._cabin_precondition_s >= card.cabin_precondition_timeout_s:
                body.climate_mode = "off"
        if mode in KEEP_MODES and self.pack.soc_pct < card.keep_climate_min_soc_pct:
            body.climate_mode = "off"
        if body.battery_precondition:
            self._battery_precondition_s += dt_s
            if self._battery_precondition_s >= card.precondition_timeout_s:
                body.battery_precondition = False
        if self._overheat_active:
            if not self._overheat_wanted() or body.cabin_c < card.overheat_on_c - _OVERHEAT_HYST_C:
                self._overheat_active = False
        elif self.awake and self._overheat_wanted() and body.cabin_c > card.overheat_on_c:
            self._overheat_active = True

    def _overheat_wanted(self) -> bool:
        card = self.card
        return (
            self.overheat_setting != "off"
            and self._mode != "drive"
            and not self.occupied
            and self.body.locked
            and self.body.climate_mode == "off"
            and self._parked_s < card.overheat_hours * 3600.0
            and self.pack.soc_pct >= card.overheat_min_soc_pct
        )

    # ----- charging -----

    def _schedule_allows(self, now: datetime) -> bool:
        start = self.body.charge_schedule
        if not start:
            return True
        hour, minute = _parse_hhmm(start)
        since = (now.hour * 60 + now.minute - (hour * 60 + minute)) % (24 * 60)
        return since < _SCHEDULE_WINDOW_MIN

    def _at_limit(self) -> bool:
        return self.pack.soc_pct >= self.body.charge_limit_pct - 1e-6

    def _below_topup(self) -> bool:
        gap = self.card.charge_limit_pct - self.card.topup_below_pct
        return self.pack.soc_pct < self.body.charge_limit_pct - gap

    def _charge_wants_power(self, now: datetime) -> bool:
        session = self.charge_session
        if not session.plugged or not self.body.charge_enabled or not self._schedule_allows(now):
            return False
        if session.stage == cs.CONNECTED:
            return not self._at_limit()
        if session.stage == cs.COMPLETE:
            return session.kind == "ac" and self._below_topup()
        return False

    def _charge_update(self, dt_s: float, now: datetime) -> None:
        session = self.charge_session
        if not session.plugged:
            return
        want = self.body.charge_enabled and self._schedule_allows(now) and self.contactors_closed and self._mode != "drive"
        session.update(dt_s, want=want, at_limit=self._at_limit(), below_topup=self._below_topup())

    def _latch(self) -> str:
        if not self.body.plug:
            return "released"
        if self.charge_session.active or self.charge_session.flowing or self.body.locked:
            return "engaged"
        return "released"

    # ----- climate and thermal -----

    def _cabin_target(self) -> tuple[float | None, bool]:
        mode = self.body.climate_mode
        if mode != "off":
            return self.body.setpoint_c, False
        if self._overheat_active:
            if self.overheat_setting == "ac":
                return self.card.overheat_on_c - _OVERHEAT_HYST_C, False
            return None, True
        return None, False

    def _occupants(self) -> int:
        if self.occupied or self.body.climate_mode in ("dog", "camp"):
            return 1
        return 0

    def _run_thermal(self, dt_s: float) -> ThermalResult:
        body = self.body
        env = self.env
        session = self.charge_session
        target, fan_only = self._cabin_target()
        charging_now = session.stage in (cs.CABLE_CHECK, cs.PRECHARGE, cs.HANDSHAKE, cs.RAMP, cs.CHARGING)
        cool, heat = battery_demand(
            self.card,
            self.pack.temp_c,
            awake=True,
            dc_charging=charging_now and session.kind == "fast",
            ac_charging=charging_now and session.kind == "ac",
            preconditioning=body.battery_precondition,
        )
        demand = ThermalDemand(
            cabin_target_c=target,
            fan_only=fan_only,
            battery_cool_w=cool,
            battery_heat_w=heat,
            battery_first=(charging_now and session.kind == "fast") or self.pack.temp_c > 45.0,
        )
        load = self._cabin_load()
        result = run_hvac(self.card, demand, cabin_c=body.cabin_c, ambient_c=env.ambient_c, load=load)
        body.cabin_c = step_cabin(self.card, body.cabin_c, dt_s, ambient_c=env.ambient_c, load=load, hvac_w=result.cabin_heat_w)
        return result

    def _cabin_load(self):
        body = self.body
        return cabin_load(
            self.card,
            speed_kph=self.sample.speed_kph,
            windows=body.windows,
            doors_open=body.doors_open(),
            trunk_open=body.trunk.state != CLOSED,
            solar_w_m2=self.env.solar_w_m2,
            occupants=self._occupants(),
        )

    def _lv_loads_w(self) -> float:
        card = self.card
        watts = card.elec_kw * 1000.0 + self._thermal.lv_el_w
        watts += card.seat_heat_w_per_level * sum(self.body.seat_heat.values())
        if self._low_beam():
            watts += card.lights_w
        elif self.body.lights == "parking":
            watts += _PARKING_LIGHTS_W
        return watts

    def _cabin_off(self) -> None:
        self.body.climate_mode = "off"
        for seat in self.body.seat_heat:
            self.body.seat_heat[seat] = 0

    # ----- driving -----

    def _begin(self, route: Route, limits: MotionLimits, style: str, traffic: float) -> None:
        if route.length_m <= 0:
            raise ValueError("route has no length")
        self.wake()
        self._route = route
        self._s_m = 0.0
        self._end_m = route.length_m
        self._limits = limits
        self._traffic = traffic
        self._heading = heading_at(route, 0.0)
        self._mode = "drive"
        self._phase = "depart"
        self._phase_s = 0.0
        self._accel = 0.0
        self._overheat_active = False
        self._leave_door_s = None
        self._walkaway_s = None
        self.occupied = True
        body = self.body
        body.locked = False
        for door in body.doors.values():
            door.set(CLOSED)
        if body.charge_port.state != CLOSED:
            body.charge_port.close()
        if body.climate_mode in ("off", "precondition", *KEEP_MODES):
            body.climate_mode = "on"
        self.sample.style = style
        lat, lon, ele, *_ = sample_route(route, 0.0)
        self.sample.latitude_deg = lat
        self.sample.longitude_deg = lon
        self.sample.elevation_m = ele
        self._sync()

    def _park(self) -> None:
        self._route = None
        self._mode = "idle"
        self._phase = ""
        self._parked_s = 0.0
        self._accel = 0.0
        self._yaw = 0.0
        self._lat_accel = 0.0
        self._drive = None
        st = self.sample
        st.style = ""
        st.gear = "park"
        st.speed_kph = 0.0
        st.pedal_pct = 0.0
        st.brake = False
        if self.card.unlock_on_park:
            self.body.locked = False
        self._sync()

    def _drive_step(self, dt_s: float, limits: Limits) -> float:
        route = self._route
        if route is None:
            raise RuntimeError("drive has no route")
        st = self.sample
        self._phase_s += dt_s
        if self._phase == "depart":
            st.brake = True
            if self._phase_s >= _SHIFT_S:
                st.gear = "drive"
            if self._phase_s >= _DEPART_S and self.contactors_closed:
                self._phase = "run"
                self._phase_s = 0.0
            return self._hold(dt_s, limits)
        if self._phase == "arrive":
            if self._phase_s >= _PARK_S:
                self._park()
                return 0.0
            return self._hold(dt_s, limits)
        remaining = max(0.0, self._end_m - self._s_m)
        if remaining <= 0.5 and st.speed_kph < 0.4:
            self._phase = "arrive"
            self._phase_s = 0.0
            return self._hold(dt_s, limits)
        want = self._driver_accel(route, remaining, dt_s)
        out = drive_step(
            self.card,
            speed_mps=st.speed_kph / 3.6,
            want_accel_mps2=want,
            grade=st.grade,
            max_discharge_w=limits.discharge_w,
            max_charge_w=limits.charge_w,
            other_loads_w=self._other_hv_w,
        )
        self._drive = out
        speed = st.speed_kph / 3.6
        new_speed = max(0.0, speed + out.accel_mps2 * dt_s)
        ds = min(remaining, 0.5 * (speed + new_speed) * dt_s)
        if remaining <= 0.5 * (speed + new_speed) * dt_s:
            new_speed = 0.0
        self._accel = (new_speed - speed) / dt_s
        self._s_m += ds
        lat, lon, ele, grade, _target, _curve = sample_route(route, self._s_m)
        heading = heading_at(route, self._s_m)
        yaw = math.radians(wrap(heading - self._heading)) / dt_s
        self._heading = heading
        st.latitude_deg = lat
        st.longitude_deg = lon
        st.elevation_m = ele
        st.grade = grade
        st.heading_deg = heading
        st.speed_kph = new_speed * 3.6
        self._yaw = yaw
        self._lat_accel = new_speed * yaw
        st.odometer_km += ds / 1000.0
        st.brake = out.brake_pedal
        st.pedal_pct = out.pedal_pct
        if not self.body.locked and st.speed_kph > self.card.auto_lock_kph:
            self.body.locked = True
        return out.traction_el_w

    def _hold(self, dt_s: float, limits: Limits) -> float:
        """Standing on the brake, in gear or about to be."""
        st = self.sample
        out = drive_step(
            self.card,
            speed_mps=0.0,
            want_accel_mps2=0.0,
            grade=st.grade,
            max_discharge_w=limits.discharge_w,
            max_charge_w=limits.charge_w,
            other_loads_w=self._other_hv_w,
        )
        self._drive = out
        self._accel = 0.0
        self._yaw = 0.0
        self._lat_accel = 0.0
        st.speed_kph = 0.0
        st.pedal_pct = 0.0
        st.brake = True
        return 0.0

    def _driver_accel(self, route: Route, remaining: float, dt_s: float) -> float:
        """What the driver asks for: road speed, curves ahead, the stop at the end, smoothed by jerk."""
        st = self.sample
        limits = self._limits
        _lat, _lon, _ele, _grade, target_kph, curve_kph = sample_route(route, self._s_m)
        speed = st.speed_kph / 3.6
        planned = limits.max_brake_mps2 * _PLANNED_BRAKE_SHARE
        look_m = max(25.0, speed**2 / (2.0 * planned))
        curve_kph = min(curve_kph, _slowest(route, self._s_m, look_m))
        stop_mps = math.sqrt(max(0.0, 2.0 * planned * max(0.0, remaining - 0.3)))
        wander = (1.0, 1.08, 0.92)[int(st.t_s // 20) % 3]
        desired_kph = min(target_kph * limits.speed_scale * self._traffic * wander, curve_kph, limits.max_speed_kph)
        desired = min(desired_kph / 3.6, stop_mps)
        if remaining > 0.5 and desired < 0.3:
            desired = 0.3
        accel = _clamp((desired - speed) / max(dt_s, 1.0), -limits.max_brake_mps2, limits.max_accel_mps2)
        jerk = limits.jerk_mps3 * dt_s
        accel = _clamp(accel, self._accel - jerk, self._accel + jerk)
        stopping = speed * speed / (2.0 * max(0.1, remaining - 0.3))
        if stopping > 0.8 * planned:
            accel = min(accel, -min(stopping, limits.max_brake_mps2))
        return accel

    # ----- status -----

    def _horn_on(self) -> bool:
        return self.awake and self.sample.t_s < self.body.horn_until_s

    def _low_beam(self) -> bool:
        if not self.awake:
            return False
        mode = self.body.lights
        if mode == "on":
            return True
        if mode == "auto":
            return self._mode == "drive" and self.env.dark
        return False

    def _drive_state(self) -> str:
        if self._power != AWAKE:
            return "unavailable"
        if self._mode == "drive" and self.sample.gear == "drive":
            return "enable"
        return "standby"

    def _hv_bus_v(self) -> float:
        if self._power == ASLEEP:
            return 0.0
        if self._power == WAKING:
            done = 1.0 - self._precharge_left_s / self.card.precharge_s
            return self.pack.ocv * (1.0 - math.exp(-5.0 * done))
        return self.pack.voltage_v

    def _time_to_full_h(self) -> float:
        power_kw = -self.pack.power_w / 1000.0
        if power_kw <= 0.05 or self._at_limit():
            return 0.0
        need_kwh = (self.body.charge_limit_pct - self.pack.soc_pct) / 100.0 * self.card.nominal_full_pack_kwh * self.pack.soh
        return need_kwh / power_kw

    def _status(self) -> dict[str, Any]:
        body = self.body
        pack = self.pack
        session = self.charge_session
        thermal = self._thermal
        drive = self._drive if self._mode == "drive" else None
        limits = pack.limits() if self.contactors_closed else _NO_LIMITS
        lat, lon, accuracy = self._gps
        st = self.sample
        values: dict[str, Any] = {
            "vehicle.awake": int(self.awake),
            "vehicle.power_state": self._power,
            "vehicle.drive_state": self._drive_state(),
            "vehicle.occupied": int(self.occupied),
            "vehicle.brake_pedal": int(st.brake),
            "vehicle.display_speed_kph": self.sensors.display_speed(st.speed_kph),
            "vehicle.range_km": pack.energy_kwh * 1000.0 / self.card.rated_wh_per_km,
            "vehicle.latitude_deg": lat,
            "vehicle.longitude_deg": lon,
            "vehicle.gps_accuracy_m": accuracy,
            "drive.motor_rpm": drive.motor_rpm if drive else 0.0,
            "drive.motor_torque_nm": drive.motor_torque_nm if drive else 0.0,
            "drive.motor_power_kw": (drive.motor_power_w if drive else 0.0) / 1000.0,
            "drive.traction_power_kw": (drive.traction_el_w if drive else 0.0) / 1000.0,
            "drive.friction_brake_n": -(drive.friction_force_n if drive else 0.0),
            "battery.temp_c": pack.temp_c,
            "battery.ocv_v": pack.ocv,
            "battery.hv_bus_voltage_v": self._hv_bus_v(),
            "battery.contactors_closed": int(self.contactors_closed),
            "battery.max_discharge_kw": limits.discharge_w / 1000.0,
            "battery.max_charge_kw": limits.charge_w / 1000.0,
            "battery.max_discharge_a": limits.discharge_a,
            "battery.max_charge_a": limits.charge_a,
            "battery.total_charge_kwh": pack.total_charge_kwh,
            "battery.total_discharge_kwh": pack.total_discharge_kwh,
            "battery.energy_to_limit_kwh": max(0.0, (body.charge_limit_pct - pack.soc_pct) / 100.0 * self.card.nominal_full_pack_kwh * pack.soh),
            "thermal.battery_heater_kw": thermal.battery_heat_w / 1000.0,
            "thermal.battery_cooling_kw": thermal.battery_cool_w / 1000.0,
            "thermal.battery_precondition": int(body.battery_precondition),
            "lv.voltage_v": self.lv.voltage_v,
            "lv.current_a": self.lv.current_a,
            "lv.soc_pct": self.lv.soc_pct,
            "dcdc.active": int(self.lv.dcdc_on),
            "dcdc.output_a": self.lv.dcdc_out_a,
            "dcdc.input_kw": self.lv.dcdc_in_w / 1000.0,
            "environment.ambient_c": self.env.ambient_c,
            "environment.ambient_filtered_c": self.env.sensor_c,
            "environment.solar_w_m2": self.env.solar_w_m2,
            "environment.sun_elevation_deg": self.env.sun_elevation_deg,
            "environment.cloud_pct": 100.0 * self.env.cloud,
            "environment.weather_source": self.env.source,
            "body.locked": int(body.locked),
            "body.trunk": body.trunk.state,
            "body.frunk": body.frunk.state,
            "body.any_door_open": int(body.any_door_open()),
            "body.any_closure_open": int(body.any_closure_open()),
            "body.horn_on": int(self._horn_on()),
            "body.windows": body.windows,
            "lights.mode": body.lights,
            "lights.low_beam": "on" if self._low_beam() else "off",
            "lights.brake": "on" if self.awake and (st.brake or (drive is not None and drive.brake_light)) else "off",
            "climate.mode": body.climate_mode,
            "climate.power_state": self._climate_power_state(),
            "climate.cabin_c": body.cabin_c,
            "climate.setpoint_c": body.setpoint_c,
            "climate.ac_running": int(thermal.ac_running),
            "climate.blower_pct": thermal.blower_pct,
            "climate.blower_segment": round(thermal.blower_pct / 100.0 * 11),
            "climate.hvac_power_kw": thermal.hvac_el_w / 1000.0,
            "climate.compressor_kw": thermal.compressor_el_w / 1000.0,
            "climate.ptc_kw": thermal.ptc_el_w / 1000.0,
            "climate.overheat_protection": self.overheat_setting,
            "climate.overheat_active": int(self._overheat_active),
            "charge.status": session.status(),
            "charge.stage": session.stage,
            "charge.port_door": body.charge_port.state,
            "charge.plug": body.plug or "none",
            "charge.cable_present": int(bool(body.plug)),
            "charge.latch": self._latch(),
            "charge.enabled": int(body.charge_enabled),
            "charge.limit_pct": body.charge_limit_pct,
            "charge.ac_limit_a": body.ac_limit_a,
            "charge.pilot_a": session.pilot_a(),
            "charge.schedule": body.charge_schedule or "none",
            "charge.line_voltage_v": session.line_voltage_v,
            "charge.line_current_a": session.line_current_a,
            "charge.line_power_kw": session.line_power_w / 1000.0,
            "charge.evse_dc_voltage_v": session.dc_voltage_v,
            "charge.evse_dc_current_a": session.dc_current_a,
            "charge.power_kw": max(0.0, -pack.power_w) / 1000.0 if session.plugged else 0.0,
            "charge.session_kwh": session.session_kwh,
            "charge.hours_to_full": self._time_to_full_h() if session.plugged else 0.0,
        }
        for name, door in body.doors.items():
            values[f"body.door_{name}"] = door.state
        for seat, level in body.seat_heat.items():
            values[f"climate.seat_heat_{seat}"] = level
        return values

    def _climate_power_state(self) -> str:
        if not self.awake:
            return "off"
        mode = self.body.climate_mode
        if mode == "precondition":
            return "precondition"
        if mode != "off":
            return "on"
        if self._overheat_active:
            return "overheat_fan" if self.overheat_setting == "fan" else "overheat"
        return "off"

    # ----- requests from the bus -----

    def _request_handlers(self) -> dict[str, Callable[[Any], None]]:
        handlers: dict[str, Callable[[Any], None]] = {
            "request.lock": self._rq_lock,
            "request.horn": self._rq_horn,
            "request.closure": self._rq_closure,
            "request.charge_enable": lambda value: self.set_charging(bool(_number(value))),
            "request.charge_port_open": lambda value: self.charge_port(True) if _number(value) else None,
            "request.charge_port_close": lambda value: self.charge_port(False) if _number(value) else None,
            "request.charge_limit_pct": lambda value: self.set_charge_limit(_number(value)),
            "request.climate_power": self._rq_climate_power,
            "request.climate_setpoint_c": lambda value: self.set_temperature(_number(value)),
            "request.climate_keep": self._rq_climate_keep,
            "request.battery_precondition": lambda value: self.battery_precondition(bool(_number(value))),
            "request.ac_current_limit_a": lambda value: self.charge_current(_number(value)),
            "request.lights": self._rq_lights,
            "request.windows": self._rq_windows,
        }
        for seat in SEATS:
            handlers[f"request.seat_heat_{seat}"] = lambda value, seat=seat: self.seat_heater(seat, round(_number(value)))
        return handlers

    def _rq_lock(self, value: Any) -> None:
        if value in ("lock", "remote_lock"):
            self.lock()
        elif value in ("unlock", "remote_unlock"):
            self.unlock()

    def _rq_horn(self, value: Any) -> None:
        if _number(value):
            self.body.horn_until_s = max(self.body.horn_until_s, self.sample.t_s + _HORN_HOLD_S)

    def _rq_climate_power(self, value: Any) -> None:
        if value == "off":
            self.climate_mode_set("off")
        elif value == "precondition":
            self.climate_mode_set("precondition")
        elif value == "on":
            self.climate_mode_set("on")

    def _rq_climate_keep(self, value: Any) -> None:
        if value in KEEP_MODES:
            self.climate_mode_set(value)
        elif value == "off" and self.body.climate_mode in KEEP_MODES:
            self.climate_mode_set("on" if self.occupied else "off")

    def _rq_closure(self, value: Any) -> None:
        if value == "front_trunk_move":
            self._refuse("frunk_open")
            self.body.frunk.open()
        elif value == "rear_trunk_move":
            if self._mode == "drive":
                raise RuntimeError("the trunk does not move while driving")
            trunk = self.body.trunk
            if trunk.moving:
                return
            if trunk.state == OPEN:
                trunk.close()
            else:
                trunk.open()

    def _rq_lights(self, value: Any) -> None:
        if value in LIGHT_MODES:
            self.lights(value)

    def _rq_windows(self, value: Any) -> None:
        if value in WINDOW_POSITIONS:
            self.windows(value)

    # ----- helpers -----

    def _check_can_drive(self) -> None:
        self._refuse("drive")

    def _sync(self) -> None:
        st = self.sample
        pack = self.pack
        if self._power == ASLEEP:
            st.mode = "asleep"
        elif self._mode == "drive":
            st.mode = "drive"
        else:
            st.mode = "charge" if self.body.plug else "idle"
        st.awake = self.awake
        st.charging = self.charge_session.stage in (cs.RAMP, cs.CHARGING)
        st.locked = self.body.locked
        st.charge_profile = self.body.plug
        st.soc_pct = pack.soc_pct
        st.energy_remaining_kwh = pack.energy_kwh
        st.pack_voltage_v = self.sensors.pack_voltage(pack.voltage_v)
        st.pack_current_a = self.sensors.pack_current(pack.current_a)
        st.pack_power_kw = pack.power_w / 1000.0
        st.temp_max_c = pack.temp_max_c
        st.temp_min_c = pack.temp_min_c
        moving = self._mode == "drive"
        st.long_accel_mps2 = self.sensors.long_accel(self._accel, st.grade) if self.awake else 0.0
        st.lat_accel_mps2 = self.sensors.lat_accel(self._lat_accel if moving else 0.0) if self.awake else 0.0
        st.yaw_rate_rads = self.sensors.yaw_rate(self._yaw if moving else 0.0) if self.awake else 0.0
        if not moving:
            st.gear = "park"
            st.speed_kph = 0.0
            st.pedal_pct = 0.0
            st.brake = False


def wrap(delta: float) -> float:
    return (delta + 180.0) % 360.0 - 180.0


def _parse_hhmm(text: str) -> tuple[int, int]:
    try:
        hour_text, minute_text = text.split(":")
        hour, minute = int(hour_text), int(minute_text)
    except (ValueError, AttributeError) as exc:
        raise ValueError(f"schedule must be HH:MM, got {text!r}") from exc
    if not (0 <= hour < 24 and 0 <= minute < 60):
        raise ValueError(f"schedule must be HH:MM, got {text!r}")
    return hour, minute


def _number(value: Any) -> float:
    if isinstance(value, bool):
        return float(value)
    if isinstance(value, (int, float)):
        return float(value)
    raise ValueError(f"expected a number, got {value!r}")


def _slowest(route: Route, s_m: float, look_m: float) -> float:
    slowest = 200.0
    cursor = s_m
    end = min(route.length_m, s_m + look_m)
    while cursor <= end:
        *_, curve = sample_route(route, cursor)
        slowest = min(slowest, curve)
        cursor += 8.0
    return slowest


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))
