"""Driver style and traffic scale the same limits. They do not change the equations."""

from __future__ import annotations

from dataclasses import dataclass

from vehicle_simulator.card import VehicleCard

STYLES = ("smooth", "aggressive")
TRAFFIC = {"light": 1.0, "moderate": 0.85, "heavy": 0.60}
# accel, brake, jerk, speed. Smooth uses the calm driver. Aggressive uses the aggressive driver.
_DRIVER = {
    "smooth": (0.80, 0.85, 0.80, 0.95),
    "aggressive": (1.35, 1.25, 1.50, 1.08),
}


@dataclass(frozen=True)
class MotionLimits:
    max_accel_mps2: float
    max_brake_mps2: float
    max_lat_accel_mps2: float
    jerk_mps3: float
    speed_scale: float
    max_speed_kph: float


def limits_for(card: VehicleCard, style: str) -> MotionLimits:
    if style not in STYLES:
        raise ValueError(f"unknown style {style}")
    accel, brake, jerk, speed = _DRIVER[style]
    return MotionLimits(
        card.max_accel_mps2 * accel,
        card.max_brake_mps2 * brake,
        card.max_lat_accel_mps2,
        card.jerk_mps3 * jerk,
        speed,
        card.max_speed_kph,
    )


def traffic_scale(name: str) -> float:
    try:
        return TRAFFIC[name]
    except KeyError:
        raise ValueError(f"unknown traffic {name}") from None
