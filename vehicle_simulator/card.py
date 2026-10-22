"""Numbers that change the car. The equations do not change per vehicle. PHYSICS.md says where each one is used."""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

WEATHER_SOURCES = ("live", "curve", "fixed")
OVERHEAT_SETTINGS = ("ac", "fan", "off")


@dataclass(frozen=True)
class VehicleCard:
    pack_kwh: float
    mass_kg: float
    charge_power_kw: float
    nominal_voltage_v: float
    home_lat: float
    home_lon: float

    # road load and motion
    cruise_speed_kph: float = 60.0
    max_speed_kph: float = 130.0
    cd: float = 0.23
    frontal_area_m2: float = 2.22
    rot_mass_factor: float = 1.05
    rolling_coeff: float = 0.012
    max_accel_mps2: float = 2.5
    max_brake_mps2: float = 3.5
    max_lat_accel_mps2: float = 3.0
    jerk_mps3: float = 1.5

    # motor and inverter
    max_drive_power_kw: float = 300.0
    max_motor_torque_nm: float = 420.0
    gear_ratio: float = 9.0
    wheel_radius_m: float = 0.35
    motor_fixed_loss_w: float = 250.0
    motor_copper_loss_w_per_nm2: float = 0.012
    motor_iron_loss_w_per_rads: float = 1.1
    max_regen_kw: float = 75.0
    regen_fade_kph: float = 8.0
    liftoff_decel_mps2: float = 1.2
    pedal_coast_pct: float = 15.0
    brake_light_decel_mps2: float = 1.3

    # high-voltage pack
    internal_resistance_ohm: float = 0.07
    r0_fraction: float = 0.7
    rc_tau_s: float = 30.0
    r_activation_k: float = 2500.0
    capacity_ah: float | None = None
    cell_v_max: float = 4.2
    cell_v_min: float = 3.0
    max_discharge_a: float = 900.0
    coulombic_eff_charge: float = 0.99
    nominal_full_pack_kwh: float | None = None
    beginning_of_life_kwh: float | None = None
    rated_wh_per_km: float = 150.0
    precharge_s: float = 1.5

    # pack thermal
    heat_capacity_j_per_k: float = 400_000.0
    heat_loss_w_per_k: float = 15.0
    cool_on_c: float = 35.0
    dc_cool_on_c: float = 30.0
    chiller_w_per_k: float = 1000.0
    chiller_max_w: float = 6000.0
    pack_heater_w: float = 6000.0
    dc_heat_target_c: float = 25.0
    ac_heat_below_c: float = 5.0
    precondition_target_c: float = 35.0
    precondition_timeout_s: float = 2700.0
    pump_w: float = 60.0
    spread_base_c: float = 1.5
    spread_k_c_per_a: float = 0.01
    spread_tau_s: float = 300.0

    # charging
    fast_charge_power_kw: float = 150.0
    dc_max_kw: float = 170.0
    evse_dc_max_a: float = 500.0
    ac_line_voltage_v: float = 230.0
    ac_phases: int = 1
    evse_ac_max_a: float = 32.0
    obc_efficiency: float = 0.92
    charge_limit_pct: float = 90.0
    topup_below_pct: float = 89.5
    ac_handshake_s: float = 3.0
    dc_cable_check_s: float = 15.0
    dc_precharge_s: float = 10.0
    ac_ramp_s: float = 8.0
    dc_ramp_s: float = 20.0
    port_close_after_unplug_s: float = 5.0
    port_close_idle_s: float = 120.0

    # 12 V system
    elec_kw: float = 0.30
    sleep_w: float = 15.0
    lv_capacity_ah: float = 40.0
    lv_bus_v: float = 14.0
    lv_charge_a: float = 20.0
    lv_topup_below_pct: float = 90.0
    lv_topup_to_pct: float = 98.0
    dcdc_efficiency: float = 0.94
    seat_heat_w_per_level: float = 30.0
    lights_w: float = 60.0

    # cabin and HVAC
    cabin_set_c: float = 22.0
    cabin_heat_capacity_j_per_k: float = 100_000.0
    cabin_ua_w_per_k: float = 30.0
    cabin_ua_per_kph: float = 0.5
    window_vent_ua_w_per_k: float = 40.0
    window_open_ua_w_per_k: float = 200.0
    door_open_ua_w_per_k: float = 150.0
    trunk_open_ua_w_per_k: float = 80.0
    solar_gain_m2: float = 1.2
    occupant_w: float = 100.0
    hvac_cool_max_w: float = 6000.0
    hvac_heat_max_w: float = 7000.0
    hvac_pull_tau_s: float = 120.0
    compressor_max_w: float = 8000.0
    heat_pump: bool = True
    blower_max_w: float = 300.0

    # parked climate modes
    keep_climate_min_soc_pct: float = 20.0
    cabin_precondition_timeout_s: float = 1800.0
    overheat_protection: str = "ac"
    overheat_on_c: float = 40.0
    overheat_hours: float = 12.0
    overheat_min_soc_pct: float = 20.0

    # weather
    weather: str = "curve"
    ambient_c: float = 25.0
    ambient_swing_c: float = 5.0
    cloud_cover: float = 0.3
    weather_refresh_s: float = 900.0
    weather_refresh_km: float = 20.0
    ambient_sensor_tau_s: float = 300.0

    # what the car does on its own
    auto_sleep_after_s: float | None = 900.0
    auto_lock_kph: float = 8.0
    unlock_on_park: bool = True
    walk_away_lock_s: float | None = 10.0

    # closures
    door_travel_s: float = 0.4
    trunk_travel_s: float = 5.0
    frunk_travel_s: float = 1.0
    charge_port_travel_s: float = 2.0

    # sensors
    sensor_noise: bool = False
    sensor_seed: int = 7
    speedo_overread: float = 0.02
    gps_fix_s: float = 10.0

    def __post_init__(self) -> None:
        if self.nominal_full_pack_kwh is None:
            object.__setattr__(self, "nominal_full_pack_kwh", self.pack_kwh)
        if self.beginning_of_life_kwh is None:
            object.__setattr__(self, "beginning_of_life_kwh", self.pack_kwh)
        if self.pack_kwh <= 0 or self.mass_kg <= 0:
            raise ValueError("pack_kwh and mass_kg must be positive")
        if self.charge_power_kw <= 0 or self.fast_charge_power_kw <= 0:
            raise ValueError("charge powers must be positive")
        if self.nominal_voltage_v <= 0 or self.cruise_speed_kph <= 0:
            raise ValueError("voltage and cruise speed must be positive")
        if not 0.0 < self.obc_efficiency <= 1.0 or not 0.0 < self.dcdc_efficiency <= 1.0:
            raise ValueError("charger and DC-DC efficiency must be in (0, 1]")
        if not 0.0 < self.r0_fraction < 1.0:
            raise ValueError("r0_fraction must be between 0 and 1")
        if not 0.0 < self.charge_limit_pct <= 100.0:
            raise ValueError("charge_limit_pct must be in (0, 100]")
        if not -90.0 <= self.home_lat <= 90.0 or not -180.0 <= self.home_lon <= 180.0:
            raise ValueError("home pin is out of range")
        if not self.topup_below_pct < self.charge_limit_pct:
            raise ValueError("topup_below_pct must be below charge_limit_pct")
        if not self.lv_topup_below_pct < self.lv_topup_to_pct <= 100.0:
            raise ValueError("lv_topup_below_pct must be below lv_topup_to_pct")
        if self.auto_sleep_after_s is not None and self.auto_sleep_after_s <= 0:
            raise ValueError("auto_sleep_after_s must be positive, or null to stay awake")
        if self.weather not in WEATHER_SOURCES:
            raise ValueError(f"weather must be one of {', '.join(WEATHER_SOURCES)}")
        if self.overheat_protection not in OVERHEAT_SETTINGS:
            raise ValueError(f"overheat_protection must be one of {', '.join(OVERHEAT_SETTINGS)}")
        if self.ac_phases not in (1, 3):
            raise ValueError("ac_phases must be 1 or 3")

    @property
    def cells_in_series(self) -> int:
        return max(1, round(self.nominal_voltage_v / 3.7))


def load_card(path: Path) -> VehicleCard:
    raw = json.loads(path.read_text())
    known = {field for field in VehicleCard.__dataclass_fields__}
    extra = set(raw) - known
    if extra:
        raise ValueError(f"unknown card fields: {', '.join(sorted(extra))}")
    return VehicleCard(**raw)
