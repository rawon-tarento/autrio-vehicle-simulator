"""Cabin heat balance, the HVAC loop, and battery heating and cooling.

One compressor serves the cabin evaporator, the heat pump, and the battery chiller.
Positive heat flows into the cabin or the pack. Electrical figures are what the HV bus pays.
"""

from __future__ import annotations

from dataclasses import dataclass

from vehicle_simulator.card import VehicleCard

_BLOWER_IDLE_W = 30.0
_FAN_ONLY_UA_W_PER_K = 100.0
_HEAT_PUMP_MIN_C = -15.0


@dataclass(frozen=True)
class CabinLoad:
    ua_w_per_k: float
    solar_w: float
    occupant_w: float


@dataclass(frozen=True)
class ThermalDemand:
    """What each consumer wants this step, before the compressor is shared."""

    cabin_target_c: float | None
    fan_only: bool
    battery_cool_w: float
    battery_heat_w: float
    battery_first: bool


@dataclass(frozen=True)
class ThermalResult:
    cabin_heat_w: float
    battery_cool_w: float
    battery_heat_w: float
    compressor_el_w: float
    ptc_el_w: float
    battery_heater_el_w: float
    pump_el_w: float
    blower_el_w: float
    blower_pct: float

    @property
    def hvac_el_w(self) -> float:
        return self.compressor_el_w + self.ptc_el_w

    @property
    def hv_el_w(self) -> float:
        """Paid straight from the HV bus. The blower and pump run off 12 V."""
        return self.compressor_el_w + self.ptc_el_w + self.battery_heater_el_w

    @property
    def lv_el_w(self) -> float:
        return self.blower_el_w + self.pump_el_w

    @property
    def ac_running(self) -> bool:
        return self.compressor_el_w > 0.0


OFF = ThermalResult(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


def cabin_load(
    card: VehicleCard,
    *,
    speed_kph: float,
    windows: str,
    doors_open: int,
    trunk_open: bool,
    solar_w_m2: float,
    occupants: int,
) -> CabinLoad:
    ua = card.cabin_ua_w_per_k + card.cabin_ua_per_kph * speed_kph
    if windows == "vent":
        ua += card.window_vent_ua_w_per_k
    elif windows == "open":
        ua += card.window_open_ua_w_per_k
    ua += card.door_open_ua_w_per_k * doors_open
    if trunk_open:
        ua += card.trunk_open_ua_w_per_k
    return CabinLoad(ua, card.solar_gain_m2 * solar_w_m2, card.occupant_w * occupants)


def cooling_cop(ambient_c: float) -> float:
    return _clamp(4.0 - 0.07 * (ambient_c - 20.0), 1.8, 4.0)


def heating_cop(card: VehicleCard, ambient_c: float) -> float:
    """Heat pump COP. 0 when there is no heat pump or it is too cold to run one: the PTC heater takes over."""
    if not card.heat_pump or ambient_c < _HEAT_PUMP_MIN_C:
        return 0.0
    return 1.2 + 1.6 * _clamp((ambient_c - _HEAT_PUMP_MIN_C) / 22.0, 0.0, 1.0)


def cabin_request_w(card: VehicleCard, cabin_c: float, target_c: float, ambient_c: float, load: CabinLoad) -> float:
    """Heat to put into the cabin: hold against losses and gains, then close the gap over the pull time."""
    hold = load.ua_w_per_k * (cabin_c - ambient_c) - load.solar_w - load.occupant_w
    pull = card.cabin_heat_capacity_j_per_k * (target_c - cabin_c) / card.hvac_pull_tau_s
    return _clamp(hold + pull, -card.hvac_cool_max_w, card.hvac_heat_max_w)


def run_hvac(card: VehicleCard, demand: ThermalDemand, *, cabin_c: float, ambient_c: float, load: CabinLoad) -> ThermalResult:
    cabin_w = 0.0
    if demand.cabin_target_c is not None:
        cabin_w = cabin_request_w(card, cabin_c, demand.cabin_target_c, ambient_c, load)
    cabin_cool = max(0.0, -cabin_w)
    cabin_heat = max(0.0, cabin_w)
    hp_cop = heating_cop(card, ambient_c)
    hp_wanted = cabin_heat if hp_cop > 0 else 0.0
    capacity = card.compressor_max_w
    bat_cool = min(demand.battery_cool_w, card.chiller_max_w)
    if demand.battery_first:
        bat_cool = min(bat_cool, capacity)
        capacity -= bat_cool
        cabin_cool = min(cabin_cool, capacity)
        capacity -= cabin_cool
    else:
        cabin_cool = min(cabin_cool, capacity)
        capacity -= cabin_cool
        bat_cool = min(bat_cool, capacity)
        capacity -= bat_cool
    hp_heat = min(hp_wanted, capacity)
    ptc_heat = cabin_heat - hp_heat
    cop_c = cooling_cop(ambient_c)
    compressor_el = (cabin_cool + bat_cool) / cop_c + (hp_heat / hp_cop if hp_heat > 0 else 0.0)
    bat_heat = min(demand.battery_heat_w, card.pack_heater_w)
    pump = card.pump_w if (bat_cool > 0 or bat_heat > 0) else 0.0
    if demand.cabin_target_c is not None:
        pct = _clamp(abs(cabin_w) / max(card.hvac_cool_max_w, card.hvac_heat_max_w), 0.2, 1.0)
    elif demand.fan_only:
        pct = 0.6
        cabin_heat = 0.0
        cabin_cool = 0.0
    else:
        pct = 0.0
    blower = _BLOWER_IDLE_W + (card.blower_max_w - _BLOWER_IDLE_W) * pct * pct if pct > 0 else 0.0
    net_cabin = cabin_heat - cabin_cool
    if demand.fan_only and demand.cabin_target_c is None:
        net_cabin = -_FAN_ONLY_UA_W_PER_K * (cabin_c - ambient_c)
    return ThermalResult(
        cabin_heat_w=net_cabin,
        battery_cool_w=bat_cool,
        battery_heat_w=bat_heat,
        compressor_el_w=compressor_el,
        ptc_el_w=ptc_heat,
        battery_heater_el_w=bat_heat,
        pump_el_w=pump,
        blower_el_w=blower,
        blower_pct=100.0 * pct,
    )


def step_cabin(card: VehicleCard, cabin_c: float, dt_s: float, *, ambient_c: float, load: CabinLoad, hvac_w: float) -> float:
    """C dT/dt = -UA (T - Tamb) + Q_solar + Q_occupants + Q_hvac."""
    net = -load.ua_w_per_k * (cabin_c - ambient_c) + load.solar_w + load.occupant_w + hvac_w
    return cabin_c + net * dt_s / card.cabin_heat_capacity_j_per_k


def battery_demand(
    card: VehicleCard,
    pack_c: float,
    *,
    awake: bool,
    dc_charging: bool,
    ac_charging: bool,
    preconditioning: bool,
) -> tuple[float, float]:
    """Chiller heat to remove and heater power to add, in watts thermal."""
    if not awake:
        return 0.0, 0.0
    cool_on = card.dc_cool_on_c if dc_charging else card.cool_on_c
    cool = max(0.0, card.chiller_w_per_k * (pack_c - cool_on))
    target = None
    if preconditioning:
        target = card.precondition_target_c
    elif dc_charging:
        target = card.dc_heat_target_c
    elif ac_charging and pack_c < card.ac_heat_below_c:
        target = card.ac_heat_below_c + 5.0
    heat = 0.0
    if target is not None and pack_c < target:
        heat = card.pack_heater_w
    return cool, heat


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))
