"""SOH falls from calendar SEI, cycle throughput, and cold/fast charge plating."""

from __future__ import annotations

import unittest

from vehicle_simulator.card import VehicleCard
from vehicle_simulator.pack import Pack, pack_resistance_ohm
from vehicle_simulator.car import Vehicle


def _card(**extra) -> VehicleCard:
    return VehicleCard(
        pack_kwh=75,
        mass_kg=1800,
        charge_power_kw=7,
        fast_charge_power_kw=150,
        nominal_voltage_v=350,
        home_lat=18.52,
        home_lon=73.85,
        weather="fixed",
        sensor_noise=False,
        **extra,
    )


def _idle(pack: Pack, seconds: float, dt: float = 1.0) -> None:
    for _ in range(round(seconds / dt)):
        pack.step(0.0, dt, contactors_closed=False, ambient_c=pack.temp_c)


class CalendarTests(unittest.TestCase):
    def test_one_equivalent_year_at_25c_50pct_costs_two_percent(self) -> None:
        pack = Pack(_card(aging_accel=1_000_000.0), soc_pct=50.0, temp_c=25.0)
        _idle(pack, 31.5576)
        self.assertAlmostEqual(pack.loss_sei, 0.02, delta=0.001)
        self.assertAlmostEqual(pack.soh, 0.98, delta=0.001)

    def test_four_years_is_twice_one_year(self) -> None:
        one = Pack(_card(aging_accel=1_000_000.0), soc_pct=50.0, temp_c=25.0)
        four = Pack(_card(aging_accel=1_000_000.0), soc_pct=50.0, temp_c=25.0)
        _idle(one, 31.5576)
        _idle(four, 126.2304)
        self.assertAlmostEqual(four.loss_sei / one.loss_sei, 2.0, delta=0.05)

    def test_high_soc_hot_calendar_beats_cool_mid_soc(self) -> None:
        easy = Pack(_card(aging_accel=1_000_000.0), soc_pct=50.0, temp_c=15.0)
        hard = Pack(_card(aging_accel=1_000_000.0), soc_pct=90.0, temp_c=40.0)
        _idle(easy, 20.0)
        _idle(hard, 20.0)
        self.assertGreater(hard.loss_sei, 2.0 * easy.loss_sei)

    def test_accel_zero_freezes_soh(self) -> None:
        pack = Pack(_card(aging_accel=0.0), soc_pct=90.0, temp_c=40.0)
        _idle(pack, 60.0)
        pack.step(-20_000.0, 10.0, contactors_closed=True, ambient_c=40.0)
        self.assertEqual(pack.soh, 1.0)
        self.assertEqual(pack.loss_sei, 0.0)
        self.assertEqual(pack.loss_cycle, 0.0)
        self.assertEqual(pack.loss_plate, 0.0)


class CycleTests(unittest.TestCase):
    def test_discharge_throughput_cuts_capacity(self) -> None:
        pack = Pack(_card(aging_accel=100_000.0), soc_pct=80.0, temp_c=25.0)
        for _ in range(50):
            pack.step(40_000.0, 1.0, contactors_closed=True, ambient_c=25.0)
        self.assertGreater(pack.eq_full_cycles, 0.5)
        self.assertGreater(pack.loss_cycle, pack.loss_sei)
        self.assertLess(pack.soh, 0.995)

    def test_resistance_grows_as_soh_falls(self) -> None:
        card = _card(aging_accel=1_000_000.0)
        pack = Pack(card, soc_pct=50.0, temp_c=25.0)
        r0 = pack_resistance_ohm(card, 25.0, pack.soh)
        _idle(pack, 80.0)
        r1 = pack_resistance_ohm(card, 25.0, pack.soh)
        self.assertLess(pack.soh, 0.97)
        self.assertGreater(r1, r0)


class PlatingTests(unittest.TestCase):
    def test_cold_dc_plates_more_than_warm_charge(self) -> None:
        warm = Pack(_card(aging_accel=2_000.0), soc_pct=90.0, temp_c=25.0)
        cold = Pack(_card(aging_accel=2_000.0), soc_pct=90.0, temp_c=5.0)
        for _ in range(80):
            warm.step(-60_000.0, 0.1, contactors_closed=True, ambient_c=25.0)
            cold.step(-60_000.0, 0.1, contactors_closed=True, ambient_c=5.0)
        self.assertGreater(cold.loss_plate, 5.0 * warm.loss_plate)
        self.assertGreater(cold.loss_plate, 1e-6)


class VehicleAgingTests(unittest.TestCase):
    def test_bus_soh_and_full_pack_follow_the_pack(self) -> None:
        car = Vehicle(_card(aging_accel=1_000_000.0), soc_pct=50.0)
        car.asleep()
        for _ in range(40):
            car.step(1.0)
        signals = car.signals()
        self.assertAlmostEqual(signals["battery.soh_pct"], 100.0 * car.pack.soh, places=4)
        self.assertAlmostEqual(signals["battery.nominal_full_pack_kwh"], car.pack.full_kwh, places=4)
        self.assertLess(signals["battery.soh_pct"], 99.0)
        self.assertGreater(signals["battery.loss_sei_pct"], 0.5)

    def test_snapshot_keeps_aging(self) -> None:
        car = Vehicle(_card(aging_accel=1_000_000.0), soc_pct=80.0)
        for _ in range(30):
            car.step(1.0)
        saved = car.snapshot()
        again = Vehicle(_card(aging_accel=1_000_000.0), soc_pct=80.0)
        again.restore(saved)
        self.assertAlmostEqual(again.pack.soh, car.pack.soh, places=8)
        self.assertAlmostEqual(again.pack.sei_time_eq_s, car.pack.sei_time_eq_s, places=6)
        self.assertAlmostEqual(again.signals()["battery.soh_pct"], car.signals()["battery.soh_pct"], places=6)


if __name__ == "__main__":
    unittest.main()
