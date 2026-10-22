"""Path geometry. A route is a polyline the car can follow."""

from __future__ import annotations

import math
from dataclasses import dataclass

_EARTH_M = 6_371_000.0


@dataclass(frozen=True)
class GeoPoint:
    lat: float
    lon: float
    ele_m: float = 0.0
    target_speed_kph: float = 60.0
    curve_speed_kph: float = 200.0


@dataclass(frozen=True)
class Route:
    points: tuple[GeoPoint, ...]
    s_m: tuple[float, ...]

    @property
    def length_m(self) -> float:
        return self.s_m[-1] if self.s_m else 0.0


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlmb = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlmb / 2) ** 2
    return 2 * _EARTH_M * math.asin(min(1.0, math.sqrt(a)))


def bearing_deg(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dlmb = math.radians(lon2 - lon1)
    east = math.sin(dlmb) * math.cos(phi2)
    north = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlmb)
    return math.degrees(math.atan2(east, north)) % 360.0


def destination(lat: float, lon: float, heading: float, distance_m: float) -> tuple[float, float]:
    ang = distance_m / _EARTH_M
    theta = math.radians(heading)
    phi1 = math.radians(lat)
    lam1 = math.radians(lon)
    phi2 = math.asin(math.sin(phi1) * math.cos(ang) + math.cos(phi1) * math.sin(ang) * math.cos(theta))
    lam2 = lam1 + math.atan2(
        math.sin(theta) * math.sin(ang) * math.cos(phi1),
        math.cos(ang) - math.sin(phi1) * math.sin(phi2),
    )
    return math.degrees(phi2), math.degrees(lam2)


def wrap_deg(delta: float) -> float:
    return (delta + 180.0) % 360.0 - 180.0


def straight_route(
    lat: float,
    lon: float,
    *,
    distance_m: float,
    heading_deg: float,
    speed_kph: float,
    step_m: float = 25.0,
    grade: float = 0.0,
) -> Route:
    if distance_m <= 0:
        raise ValueError("distance_m must be positive")
    n = max(1, math.ceil(distance_m / step_m))
    points: list[GeoPoint] = []
    s_values: list[float] = []
    for i in range(n + 1):
        s = min(distance_m, i * step_m)
        plat, plon = destination(lat, lon, heading_deg, s)
        points.append(GeoPoint(plat, plon, ele_m=s * grade, target_speed_kph=speed_kph, curve_speed_kph=200.0))
        s_values.append(s)
    return Route(tuple(points), tuple(s_values))


def route_from_track(samples: list[tuple[float, float, float, float]], *, lat_accel_limit: float) -> Route:
    """Samples are lat, lon, elevation, target speed."""
    if len(samples) < 2:
        raise ValueError("a road needs at least two points")
    kept: list[tuple[float, float, float, float]] = [samples[0]]
    for row in samples[1:]:
        prev = kept[-1]
        if haversine_m(prev[0], prev[1], row[0], row[1]) >= 8.0:
            kept.append(row)
    if len(kept) < 2:
        kept.append(samples[-1])
    s_values = [0.0]
    for i in range(1, len(kept)):
        s_values.append(s_values[-1] + haversine_m(kept[i - 1][0], kept[i - 1][1], kept[i][0], kept[i][1]))
    curve = [_curve_speed(kept, s_values, i, lat_accel_limit) for i in range(len(kept))]
    points = tuple(
        GeoPoint(lat, lon, ele, target_speed_kph=speed, curve_speed_kph=curve[i])
        for i, (lat, lon, ele, speed) in enumerate(kept)
    )
    return Route(points, tuple(s_values))


def sample_route(route: Route, s_m: float) -> tuple[float, float, float, float, float, float]:
    s_m = min(max(0.0, s_m), route.length_m)
    hi = 1
    while hi < len(route.s_m) and route.s_m[hi] < s_m:
        hi += 1
    lo = hi - 1
    span = route.s_m[hi] - route.s_m[lo]
    blend = 0.0 if span <= 1e-6 else (s_m - route.s_m[lo]) / span
    a, b = route.points[lo], route.points[hi]
    lat = a.lat + (b.lat - a.lat) * blend
    lon = a.lon + (b.lon - a.lon) * blend
    ele = a.ele_m + (b.ele_m - a.ele_m) * blend
    grade = 0.0 if span <= 1.0 else max(-0.12, min(0.12, (b.ele_m - a.ele_m) / span))
    target = a.target_speed_kph + (b.target_speed_kph - a.target_speed_kph) * blend
    curve = min(a.curve_speed_kph, b.curve_speed_kph)
    return lat, lon, ele, grade, target, curve


def heading_at(route: Route, s_m: float) -> float:
    ahead = min(route.length_m, s_m + 15.0)
    behind = max(0.0, s_m - 15.0)
    if ahead - behind < 1.0:
        return 0.0
    lat1, lon1, *_ = sample_route(route, behind)
    lat2, lon2, *_ = sample_route(route, ahead)
    return bearing_deg(lat1, lon1, lat2, lon2)


def _curve_speed(points: list[tuple[float, float, float, float]], s_m: list[float], index: int, a_lat: float) -> float:
    if len(points) < 3:
        return 200.0
    index = min(max(index, 1), len(points) - 2)
    span = ((s_m[index] - s_m[index - 1]) + (s_m[index + 1] - s_m[index])) / 2.0
    if span < 5.0:
        return 200.0
    incoming = bearing_deg(points[index - 1][0], points[index - 1][1], points[index][0], points[index][1])
    outgoing = bearing_deg(points[index][0], points[index][1], points[index + 1][0], points[index + 1][1])
    turn = abs(wrap_deg(outgoing - incoming))
    if turn < 1.0:
        return 200.0
    kappa = math.radians(turn) / span
    return math.sqrt(a_lat / kappa) * 3.6
