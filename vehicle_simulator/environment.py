"""Air temperature and sunlight at the car. Real weather when it can be fetched, a daily curve when not."""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Callable

from vehicle_simulator.card import VehicleCard
from vehicle_simulator.geo import haversine_m

_FORECAST = "https://api.open-meteo.com/v1/forecast"
_SOLAR_CONSTANT_CLEAR_W_M2 = 1000.0
_WARMEST_HOUR = 15.0


@dataclass(frozen=True)
class Weather:
    ambient_c: float
    solar_w_m2: float
    cloud: float
    lat: float
    lon: float
    fetched_at: float


WeatherFetch = Callable[[float, float], Weather]


def sun_elevation_deg(when: datetime, lat: float, lon: float) -> float:
    """Height of the sun above the horizon. Declination plus equation of time, good to about a degree."""
    utc = _utc(when)
    day = utc.timetuple().tm_yday
    hours = utc.hour + utc.minute / 60.0 + utc.second / 3600.0
    decl = math.radians(23.44) * math.sin(2.0 * math.pi * (284 + day) / 365.0)
    b = 2.0 * math.pi * (day - 81) / 364.0
    eot_min = 9.87 * math.sin(2.0 * b) - 7.53 * math.cos(b) - 1.5 * math.sin(b)
    solar_hours = hours + lon / 15.0 + eot_min / 60.0
    hour_angle = math.radians(15.0 * (solar_hours - 12.0))
    phi = math.radians(lat)
    sin_el = math.sin(phi) * math.sin(decl) + math.cos(phi) * math.cos(decl) * math.cos(hour_angle)
    return math.degrees(math.asin(max(-1.0, min(1.0, sin_el))))


def clear_sky_w_m2(elevation_deg: float) -> float:
    if elevation_deg <= 0.0:
        return 0.0
    return _SOLAR_CONSTANT_CLEAR_W_M2 * math.sin(math.radians(elevation_deg)) ** 1.15


def cloud_factor(cloud: float) -> float:
    """Kasten and Czeplak: share of clear-sky sunlight that gets through a cloud fraction."""
    return 1.0 - 0.75 * max(0.0, min(1.0, cloud)) ** 3.4


def curve_ambient_c(card: VehicleCard, when: datetime, lon: float) -> float:
    """Daily swing around the card mean, warmest mid-afternoon solar time."""
    utc = _utc(when)
    solar_hours = utc.hour + utc.minute / 60.0 + lon / 15.0
    return card.ambient_c + card.ambient_swing_c * math.cos(2.0 * math.pi * (solar_hours - _WARMEST_HOUR) / 24.0)


class Environment:
    """What the air and sun are doing where the car is. ``live`` is optional and runs off the clock."""

    def __init__(self, card: VehicleCard, live: LiveWeather | None = None) -> None:
        self.card = card
        self.live = live
        self.ambient_c = card.ambient_c
        self.sensor_c = card.ambient_c
        self.solar_w_m2 = 0.0
        self.sun_elevation_deg = 0.0
        self.cloud = card.cloud_cover
        self.source = card.weather
        self._primed = False

    @property
    def dark(self) -> bool:
        return self.sun_elevation_deg < 0.0 or self.solar_w_m2 < 50.0

    def update(self, when: datetime, lat: float, lon: float, dt_s: float) -> None:
        card = self.card
        self.sun_elevation_deg = sun_elevation_deg(when, lat, lon)
        observed = self.live.latest(lat, lon) if self.live is not None else None
        if observed is not None:
            self.source = "live"
            self.ambient_c = observed.ambient_c
            self.cloud = observed.cloud
            self.solar_w_m2 = observed.solar_w_m2
        else:
            self.source = "fixed" if card.weather == "fixed" else "curve"
            self.cloud = card.cloud_cover
            self.ambient_c = card.ambient_c if card.weather == "fixed" else curve_ambient_c(card, when, lon)
            self.solar_w_m2 = clear_sky_w_m2(self.sun_elevation_deg) * cloud_factor(self.cloud)
        if not self._primed:
            self.sensor_c = self.ambient_c
            self._primed = True
        else:
            self.sensor_c += (self.ambient_c - self.sensor_c) * (1.0 - math.exp(-dt_s / card.ambient_sensor_tau_s))


class LiveWeather:
    """Open-Meteo current conditions, fetched in the background when stale or after the car moved far."""

    def __init__(self, card: VehicleCard, fetch: WeatherFetch | None = None) -> None:
        self.card = card
        self._fetch = fetch or fetch_open_meteo
        self._lock = threading.Lock()
        self._latest: Weather | None = None
        self._busy = False
        self._last_try = -math.inf
        self.error = ""

    def latest(self, lat: float, lon: float) -> Weather | None:
        with self._lock:
            latest = self._latest
            busy = self._busy
        now = time.monotonic()
        stale = latest is None or now - latest.fetched_at > self.card.weather_refresh_s
        far = latest is not None and haversine_m(lat, lon, latest.lat, latest.lon) > self.card.weather_refresh_km * 1000.0
        if (stale or far) and not busy and now - self._last_try > 60.0:
            self._last_try = now
            with self._lock:
                self._busy = True
            threading.Thread(target=self._refresh, args=(lat, lon), daemon=True).start()
        return latest

    def _refresh(self, lat: float, lon: float) -> None:
        try:
            weather = self._fetch(lat, lon)
            with self._lock:
                self._latest = weather
            self.error = ""
        except Exception as exc:
            self.error = str(exc)
        finally:
            with self._lock:
                self._busy = False


def fetch_open_meteo(lat: float, lon: float) -> Weather:
    from vehicle_simulator.roads import _get_json

    url = f"{_FORECAST}?latitude={lat:.4f}&longitude={lon:.4f}&current=temperature_2m,shortwave_radiation,cloud_cover"
    current = _get_json(url)["current"]
    return Weather(
        ambient_c=float(current["temperature_2m"]),
        solar_w_m2=float(current["shortwave_radiation"]),
        cloud=float(current["cloud_cover"]) / 100.0,
        lat=lat,
        lon=lon,
        fetched_at=time.monotonic(),
    )


def _utc(when: datetime) -> datetime:
    if when.tzinfo is None:
        when = when.astimezone()
    return when.astimezone(timezone.utc)
