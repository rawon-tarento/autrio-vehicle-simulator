"""One second of the car. Names are canonicals, not DBC signals."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any


@dataclass
class Sample:
    t_s: float = 0.0
    mode: str = "idle"
    awake: bool = True
    gear: str = "park"
    speed_kph: float = 0.0
    long_accel_mps2: float = 0.0
    lat_accel_mps2: float = 0.0
    long_accel_valid: int = 1
    lat_accel_valid: int = 1
    yaw_rate_rads: float = 0.0
    heading_deg: float = 0.0
    grade: float = 0.0
    elevation_m: float = 0.0
    pack_voltage_v: float = 0.0
    pack_current_a: float = 0.0
    pack_power_kw: float = 0.0
    soc_pct: float = 80.0
    temp_max_c: float = 25.0
    temp_min_c: float = 23.0
    energy_remaining_kwh: float = 0.0
    nominal_full_pack_kwh: float = 0.0
    beginning_of_life_kwh: float = 0.0
    odometer_km: float = 0.0
    latitude_deg: float = 0.0
    longitude_deg: float = 0.0
    gps_accuracy_m: float = 4.0
    charging: bool = False
    locked: bool = True
    pedal_pct: float = 0.0
    brake: bool = True
    style: str = ""
    charge_profile: str = ""

    def canonicals(self) -> dict[str, Any]:
        full = self.nominal_full_pack_kwh
        bol = self.beginning_of_life_kwh
        return {
            "vehicle.speed_kph": self.speed_kph,
            "vehicle.long_accel_mps2": self.long_accel_mps2,
            "vehicle.lat_accel_mps2": self.lat_accel_mps2,
            "vehicle.long_accel_valid": self.long_accel_valid,
            "vehicle.lat_accel_valid": self.lat_accel_valid,
            "vehicle.gear": self.gear,
            "vehicle.odometer_km": self.odometer_km,
            "vehicle.latitude_deg": self.latitude_deg,
            "vehicle.longitude_deg": self.longitude_deg,
            "vehicle.gps_accuracy_m": self.gps_accuracy_m,
            "vehicle.yaw_rate_rads": self.yaw_rate_rads,
            "vehicle.heading_deg": self.heading_deg,
            "vehicle.grade": self.grade,
            "vehicle.elevation_m": self.elevation_m,
            "vehicle.pedal_pct": self.pedal_pct,
            "battery.pack_voltage_v": self.pack_voltage_v,
            "battery.pack_current_a": self.pack_current_a,
            "battery.pack_power_kw": self.pack_power_kw,
            "battery.soc_pct": self.soc_pct,
            "battery.temp_max_c": self.temp_max_c,
            "battery.temp_min_c": self.temp_min_c,
            "battery.nominal_full_pack_kwh": full,
            "battery.beginning_of_life_kwh": bol,
            "battery.nominal_energy_remaining_kwh": self.energy_remaining_kwh,
            "battery.soh_pct": None if bol == 0 else 100.0 * full / bol,
        }

    def row(self) -> dict[str, Any]:
        return asdict(self)
