"""High-voltage pack: open-circuit voltage, R0 plus one RC branch, BMS limits, heat, and energy counters.

Positive current discharges. No DBC names.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from vehicle_simulator.card import VehicleCard

_OCV_SOC = (0.0, 10.0, 20.0, 50.0, 80.0, 90.0, 100.0)
_OCV_CELL = (3.0, 3.45, 3.55, 3.70, 3.95, 4.10, 4.18)
_CELL_AT_NOMINAL = 3.70

# Supercharger-like: hold peak to ~20–30%, roll off through mid SOC, CV tail after 80%.
_ACCEPT_SOC = (0.0, 20.0, 30.0, 40.0, 50.0, 70.0, 80.0, 90.0, 95.0, 100.0)
_ACCEPT_SOC_FACTOR = (1.0, 1.0, 0.92, 0.70, 0.52, 0.38, 0.28, 0.18, 0.08, 0.0)
_ACCEPT_TEMP = (-10.0, 0.0, 15.0, 25.0, 45.0, 55.0)
_ACCEPT_TEMP_FACTOR = (0.02, 0.05, 0.5, 1.0, 1.0, 0.3)
_DISCHARGE_TEMP = (-20.0, 0.0, 15.0, 50.0, 60.0)
_DISCHARGE_TEMP_FACTOR = (0.4, 0.7, 1.0, 1.0, 0.5)


@dataclass(frozen=True)
class Limits:
    """What the BMS allows right now. Charge values are positive numbers."""

    discharge_a: float
    charge_a: float
    discharge_w: float
    charge_w: float


class Pack:
    def __init__(self, card: VehicleCard, *, soc_pct: float, temp_c: float) -> None:
        self.card = card
        self.soc_pct = soc_pct
        self.temp_c = temp_c
        self.spread_c = card.spread_base_c
        self.v_rc = 0.0
        self.current_a = 0.0
        self.voltage_v = ocv_v(card, soc_pct)
        self.power_w = 0.0
        self.heat_w = 0.0
        self.total_charge_kwh = 0.0
        self.total_discharge_kwh = 0.0

    @property
    def soh(self) -> float:
        bol = self.card.beginning_of_life_kwh
        return 1.0 if not bol else self.card.nominal_full_pack_kwh / bol

    @property
    def energy_kwh(self) -> float:
        return self.card.nominal_full_pack_kwh * self.soh * self.soc_pct / 100.0

    @property
    def ocv(self) -> float:
        return ocv_v(self.card, self.soc_pct)

    @property
    def r0(self) -> float:
        return pack_resistance_ohm(self.card, self.temp_c, self.soh) * self.card.r0_fraction

    @property
    def r1(self) -> float:
        return pack_resistance_ohm(self.card, self.temp_c, self.soh) * (1.0 - self.card.r0_fraction)

    def limits(self) -> Limits:
        card = self.card
        cells = card.cells_in_series
        v_min = card.cell_v_min * cells
        v_max = card.cell_v_max * cells
        ocv = self.ocv
        r0 = self.r0
        dis_a = min(max(0.0, (ocv - self.v_rc - v_min) / r0), card.max_discharge_a)
        dis_a *= _interp(self.temp_c, _DISCHARGE_TEMP, _DISCHARGE_TEMP_FACTOR)
        peak_a = card.dc_max_kw * 1000.0 / card.nominal_voltage_v
        accept_a = peak_a * _interp(self.soc_pct, _ACCEPT_SOC, _ACCEPT_SOC_FACTOR) * _interp(self.temp_c, _ACCEPT_TEMP, _ACCEPT_TEMP_FACTOR)
        cv_a = max(0.0, (v_max - ocv + self.v_rc) / r0)
        chg_a = min(accept_a, cv_a)
        dis_w = min(dis_a * (ocv - self.v_rc - dis_a * r0), (ocv - self.v_rc) ** 2 / (4.0 * r0))
        chg_w = chg_a * (ocv - self.v_rc + chg_a * r0)
        return Limits(dis_a, chg_a, max(0.0, dis_w), max(0.0, chg_w))

    def solve_current(self, power_w: float) -> float:
        """Current that delivers ``power_w`` at the terminals: P = (OCV - Vrc) I - R0 I^2."""
        return current_a(self.ocv - self.v_rc, self.r0, power_w)

    def step(
        self,
        power_w: float,
        dt_s: float,
        *,
        contactors_closed: bool,
        ambient_c: float,
        cooling_w: float = 0.0,
        heating_w: float = 0.0,
        limit_pct: float | None = None,
    ) -> float:
        """Draw ``power_w`` (negative charges) for ``dt_s``. Returns the power the pack actually gave."""
        card = self.card
        current = 0.0
        if contactors_closed:
            limits = self.limits()
            current = _clamp(self.solve_current(power_w), -limits.charge_a, limits.discharge_a)
            if limit_pct is not None and current < 0 and self.soc_pct >= limit_pct:
                current = max(current, 0.0)
        r0 = self.r0
        r1 = self.r1
        decay = math.exp(-dt_s / card.rc_tau_s)
        self.v_rc = self.v_rc * decay + current * r1 * (1.0 - decay)
        voltage = self.ocv - current * r0 - self.v_rc
        moved_ah = current * dt_s / 3600.0
        if current < 0:
            moved_ah *= card.coulombic_eff_charge
        self.soc_pct = _clamp(self.soc_pct - moved_ah / (capacity_ah(card) * self.soh) * 100.0, 0.0, 100.0)
        energy_kwh = voltage * current * dt_s / 3.6e6
        if energy_kwh > 0:
            self.total_discharge_kwh += energy_kwh
        else:
            self.total_charge_kwh -= energy_kwh
        self.heat_w = current * current * r0 + (self.v_rc * self.v_rc / r1 if r1 > 0 else 0.0)
        net_w = self.heat_w - card.heat_loss_w_per_k * (self.temp_c - ambient_c) - cooling_w + heating_w
        self.temp_c += net_w * dt_s / card.heat_capacity_j_per_k
        spread_target = card.spread_base_c + card.spread_k_c_per_a * abs(current)
        self.spread_c += (spread_target - self.spread_c) * (1.0 - math.exp(-dt_s / card.spread_tau_s))
        self.current_a = current
        self.voltage_v = voltage
        self.power_w = voltage * current
        return self.power_w

    @property
    def temp_max_c(self) -> float:
        return self.temp_c + self.spread_c / 2.0

    @property
    def temp_min_c(self) -> float:
        return self.temp_c - self.spread_c / 2.0


def ocv_v(card: VehicleCard, soc_pct: float) -> float:
    """Cell curve scaled so 50% SOC sits on the card's nominal voltage."""
    return card.nominal_voltage_v * (_interp(soc_pct, _OCV_SOC, _OCV_CELL) / _CELL_AT_NOMINAL)


def pack_resistance_ohm(card: VehicleCard, temp_c: float, soh: float) -> float:
    """Total DC resistance: grows as the pack ages, and by Arrhenius as it gets cold."""
    base = card.internal_resistance_ohm * (1.0 + 0.5 * (1.0 - soh))
    return base * math.exp(card.r_activation_k * (1.0 / (temp_c + 273.15) - 1.0 / 298.15))


def current_a(source_v: float, resistance: float, power_w: float) -> float:
    if resistance <= 0:
        return 0.0 if abs(source_v) < 1e-6 else power_w / source_v
    capped = source_v * source_v / (4.0 * resistance)
    power_w = min(power_w, capped)
    disc = max(0.0, source_v * source_v - 4.0 * resistance * power_w)
    return (source_v - math.sqrt(disc)) / (2.0 * resistance)


def capacity_ah(card: VehicleCard) -> float:
    if card.capacity_ah is not None:
        return card.capacity_ah
    return card.pack_kwh * 1000.0 / card.nominal_voltage_v


def _interp(x: float, xs: tuple[float, ...], ys: tuple[float, ...]) -> float:
    if x <= xs[0]:
        return ys[0]
    if x >= xs[-1]:
        return ys[-1]
    for index in range(1, len(xs)):
        if x <= xs[index]:
            span = xs[index] - xs[index - 1]
            blend = 0.0 if span == 0 else (x - xs[index - 1]) / span
            return ys[index - 1] + (ys[index] - ys[index - 1]) * blend
    return ys[-1]


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))
