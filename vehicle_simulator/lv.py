"""The 12 V battery and the DC-DC converter that feeds it from the HV pack.

Asleep, the contactors are open and the 12 V battery alone carries the sleep load. Awake, the DC-DC holds
the 12 V bus, runs every low-voltage load, and recharges the battery. A low 12 V battery wakes the car.
"""

from __future__ import annotations

from vehicle_simulator.card import VehicleCard

_REST_EMPTY_V = 11.9
_REST_FULL_V = 12.8
_BATTERY_OHM = 0.02


class LowVoltage:
    def __init__(self, card: VehicleCard, soc_pct: float = 95.0) -> None:
        self.card = card
        self.soc_pct = soc_pct
        self.voltage_v = rest_v(soc_pct)
        self.current_a = 0.0
        self.dcdc_on = False
        self.dcdc_out_a = 0.0
        self.dcdc_in_w = 0.0

    def step(self, dt_s: float, *, loads_w: float, dcdc_on: bool) -> float:
        """Run ``loads_w`` for ``dt_s``. Returns what the DC-DC draws from the HV bus."""
        card = self.card
        self.dcdc_on = dcdc_on
        if dcdc_on:
            bus_v = card.lv_bus_v
            charge_a = card.lv_charge_a * _clamp((100.0 - self.soc_pct) / 10.0, 0.05, 1.0)
            if self.soc_pct >= 100.0:
                charge_a = 0.0
            self.current_a = -charge_a
            self.dcdc_out_a = loads_w / bus_v + charge_a
            self.voltage_v = bus_v
            self.dcdc_in_w = self.dcdc_out_a * bus_v / card.dcdc_efficiency
        else:
            v = rest_v(self.soc_pct)
            self.current_a = loads_w / v
            self.voltage_v = v - self.current_a * _BATTERY_OHM
            self.dcdc_out_a = 0.0
            self.dcdc_in_w = 0.0
        moved_ah = self.current_a * dt_s / 3600.0
        self.soc_pct = _clamp(self.soc_pct - moved_ah / card.lv_capacity_ah * 100.0, 0.0, 100.0)
        return self.dcdc_in_w

    @property
    def needs_topup(self) -> bool:
        return self.soc_pct < self.card.lv_topup_below_pct

    @property
    def topped_up(self) -> bool:
        return self.soc_pct >= self.card.lv_topup_to_pct


def rest_v(soc_pct: float) -> float:
    return _REST_EMPTY_V + (_REST_FULL_V - _REST_EMPTY_V) * soc_pct / 100.0


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))
