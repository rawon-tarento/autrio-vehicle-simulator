"""One charge session: plug, handshake with the station, ramp, constant current, taper, complete.

The station delivers power onto the HV bus. Whatever the car's own loads do not use goes into the pack,
up to what the BMS accepts. That is why a pack near full or cold charges slowly whatever the station offers.
"""

from __future__ import annotations

from vehicle_simulator.card import VehicleCard

NONE = "none"
CONNECTED = "connected"
CABLE_CHECK = "cable_check"
PRECHARGE = "precharge"
HANDSHAKE = "handshake"
RAMP = "ramp"
CHARGING = "charging"
COMPLETE = "complete"

_LINE_OHM = 0.08


class ChargeSession:
    def __init__(self, card: VehicleCard) -> None:
        self.card = card
        self.kind = ""
        self.stage = NONE
        self.stage_s = 0.0
        self.ramp = 0.0
        self.session_kwh = 0.0
        self.line_voltage_v = 0.0
        self.line_current_a = 0.0
        self.line_power_w = 0.0
        self.dc_voltage_v = 0.0
        self.dc_current_a = 0.0
        self.bus_w = 0.0

    @property
    def plugged(self) -> bool:
        return self.stage != NONE

    @property
    def flowing(self) -> bool:
        """The station is handing over power: the pack takes some, or the car runs off it."""
        return self.stage in (RAMP, CHARGING) or (self.stage == COMPLETE and self.kind == "ac")

    @property
    def active(self) -> bool:
        """The session is busy getting energy into the pack. Keeps the car awake."""
        return self.stage in (HANDSHAKE, CABLE_CHECK, PRECHARGE, RAMP, CHARGING)

    def plug(self, kind: str) -> None:
        self.kind = kind
        self._enter(CONNECTED)
        self.session_kwh = 0.0

    def unplug(self) -> None:
        self.kind = ""
        self._enter(NONE)
        self._idle_line()

    def update(self, dt_s: float, *, want: bool, at_limit: bool, below_topup: bool) -> None:
        """Move between stages. ``want`` is: enabled, schedule allows, car awake and parked."""
        stage = self.stage
        if stage == NONE:
            return
        self.stage_s += dt_s
        if not want:
            if stage != CONNECTED and not (stage == COMPLETE and at_limit):
                self._enter(CONNECTED)
            return
        if stage == CONNECTED and not at_limit:
            self._enter(CABLE_CHECK if self.kind == "fast" else HANDSHAKE)
        elif stage == CONNECTED and at_limit:
            self._enter(COMPLETE)
        elif stage == HANDSHAKE and self.stage_s >= self.card.ac_handshake_s:
            self._enter(RAMP)
        elif stage == CABLE_CHECK and self.stage_s >= self.card.dc_cable_check_s:
            self._enter(PRECHARGE)
        elif stage == PRECHARGE and self.stage_s >= self.card.dc_precharge_s:
            self._enter(RAMP)
        elif stage == RAMP:
            ramp_s = self.card.dc_ramp_s if self.kind == "fast" else self.card.ac_ramp_s
            self.ramp = min(1.0, self.stage_s / ramp_s)
            if self.ramp >= 1.0:
                self._enter(CHARGING)
                self.ramp = 1.0
        elif stage == COMPLETE and below_topup and self.kind == "ac":
            self._enter(HANDSHAKE)
        if self.stage in (RAMP, CHARGING) and at_limit:
            self._enter(COMPLETE)

    def available_w(self, ac_limit_a: float, pack_v: float) -> float:
        """Most the station can put on the HV bus now."""
        if not self.flowing:
            return 0.0
        card = self.card
        ramp = self.ramp if self.stage == RAMP else 1.0
        if self.kind == "fast":
            return min(card.fast_charge_power_kw * 1000.0, card.evse_dc_max_a * pack_v) * ramp
        amps = min(card.evse_ac_max_a, ac_limit_a)
        line_w = min(amps * card.ac_line_voltage_v * card.ac_phases, card.charge_power_kw * 1000.0 / card.obc_efficiency)
        return line_w * card.obc_efficiency * ramp

    def deliver(self, bus_w: float, pack_v: float, into_pack_w: float, dt_s: float) -> None:
        """Record what the station actually supplied this step."""
        card = self.card
        self.bus_w = max(0.0, bus_w)
        self.session_kwh += max(0.0, into_pack_w) * dt_s / 3.6e6
        if self.kind == "fast":
            self.line_voltage_v = self.line_current_a = self.line_power_w = 0.0
            self.dc_voltage_v = pack_v if self.stage in (PRECHARGE, RAMP, CHARGING) else 0.0
            self.dc_current_a = self.bus_w / pack_v if pack_v > 0 else 0.0
            return
        self.dc_voltage_v = self.dc_current_a = 0.0
        if self.kind != "ac" or not self.flowing:
            self._idle_line()
            return
        self.line_power_w = self.bus_w / card.obc_efficiency
        nominal = card.ac_line_voltage_v
        amps = self.line_power_w / (nominal * card.ac_phases)
        self.line_voltage_v = nominal - _LINE_OHM * amps
        self.line_current_a = amps

    def pilot_a(self) -> float:
        return self.card.evse_ac_max_a if self.kind == "ac" else 0.0

    def status(self) -> str:
        """The charge-port controller's view, close to what a car reports on the bus."""
        return {
            NONE: "inactive",
            CONNECTED: "connected",
            HANDSHAKE: "connected",
            CABLE_CHECK: "ext_evse_test",
            PRECHARGE: "evse_test_passed",
            RAMP: "enabled",
            CHARGING: "enabled",
            COMPLETE: "standby",
        }[self.stage]

    def _enter(self, stage: str) -> None:
        self.stage = stage
        self.stage_s = 0.0
        if stage != RAMP:
            self.ramp = 1.0 if stage == CHARGING else 0.0

    def _idle_line(self) -> None:
        self.line_voltage_v = self.line_current_a = self.line_power_w = 0.0
        self.dc_voltage_v = self.dc_current_a = 0.0
        self.bus_w = 0.0
