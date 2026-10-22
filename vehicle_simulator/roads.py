"""A driving path from the current pin to a destination. OpenStreetMap plus height."""

from __future__ import annotations

import json
import os
import ssl
import urllib.error
import urllib.request
from pathlib import Path

from vehicle_simulator.geo import Route, route_from_track

_OSRM = "https://router.project-osrm.org/route/v1/driving"
_TERRAIN = "https://api.open-meteo.com/v1/elevation"


class RoadError(RuntimeError):
    pass


def fetch_route(
    origin: tuple[float, float],
    destination: tuple[float, float],
    *,
    speed_kph: float,
    lat_accel_limit: float,
    cache_dir: Path | None = None,
) -> Route:
    cache = _cache_path(cache_dir, origin, destination, speed_kph)
    if cache is not None and cache.exists():
        stored = json.loads(cache.read_text())
        return route_from_track([tuple(row) for row in stored["points"]], lat_accel_limit=lat_accel_limit)
    points = _osrm(origin, destination)
    heights = _elevation([(lat, lon) for lat, lon in points])
    track = [(lat, lon, height, speed_kph) for (lat, lon), height in zip(points, heights, strict=True)]
    if cache is not None:
        cache.parent.mkdir(parents=True, exist_ok=True)
        cache.write_text(json.dumps({"points": track}))
    return route_from_track(track, lat_accel_limit=lat_accel_limit)


def _osrm(origin: tuple[float, float], destination: tuple[float, float]) -> list[tuple[float, float]]:
    url = f"{_OSRM}/{origin[1]},{origin[0]};{destination[1]},{destination[0]}?overview=full&geometries=geojson"
    body = _get_json(url)
    if body.get("code") != "Ok":
        raise RoadError(f"road lookup failed: {body.get('code')}")
    coords = body["routes"][0]["geometry"]["coordinates"]
    points = [(lat, lon) for lon, lat in coords]
    if len(points) < 2:
        raise RoadError("the road has fewer than two points")
    return points


def _elevation(points: list[tuple[float, float]]) -> list[float]:
    heights: list[float] = []
    for start in range(0, len(points), 100):
        chunk = points[start : start + 100]
        lats = ",".join(str(lat) for lat, _lon in chunk)
        lons = ",".join(str(lon) for _lat, lon in chunk)
        body = _get_json(f"{_TERRAIN}?latitude={lats}&longitude={lons}")
        heights.extend(float(value) for value in body["elevation"])
    if len(heights) != len(points):
        raise RoadError("elevation count does not match the road")
    return heights


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


def _get_json(url: str) -> dict:
    request = urllib.request.Request(url, headers={"User-Agent": "vehicle-simulator"})
    try:
        with urllib.request.urlopen(request, context=_ssl_context(), timeout=20) as response:
            return json.loads(response.read().decode())
    except urllib.error.URLError as exc:
        raise RoadError(str(exc)) from exc


def _cache_path(cache_dir: Path | None, origin: tuple[float, float], destination: tuple[float, float], speed_kph: float) -> Path | None:
    if cache_dir is None:
        return None
    key = f"{origin[0]:.5f},{origin[1]:.5f},{destination[0]:.5f},{destination[1]:.5f},{speed_kph:.1f}"
    return cache_dir / f"{key.replace(',', '_')}.json"
