"""The live car idles, then follows a command. No DBC is loaded."""

from __future__ import annotations

import unittest

from vehicle_simulator.car import Vehicle
from vehicle_simulator.card import VehicleCard
from vehicle_simulator.geo import straight_route


def _car(**extra) -> Vehicle:
    return Vehicle(
        VehicleCard(
            pack_kwh=75,
            mass_kg=1800,
            charge_power_kw=7,
            fast_charge_power_kw=120,
            nominal_voltage_v=350,
            home_lat=18.52,
            home_lon=73.85,
            cruise_speed_kph=60,
            max_accel_mps2=2.5,
            weather="fixed",
            sensor_noise=False,
            **extra,
        )
    )


class LiveCarTests(unittest.TestCase):
    def test_boot_is_idle_at_the_home_pin(self) -> None:
        car = _car()
        sample = car.step()
        self.assertEqual(sample.mode, "idle")
        self.assertEqual(sample.gear, "park")
        self.assertEqual(sample.speed_kph, 0.0)
        self.assertAlmostEqual(sample.latitude_deg, 18.52)
        self.assertGreater(sample.pack_current_a, 0.0)
        self.assertNotIn("DI_", str(sample.canonicals()))
        self.assertEqual(car.signals()["vehicle.drive_state"], "standby")

    def test_idle_uses_a_little_energy(self) -> None:
        car = _car()
        start = car.sample.soc_pct
        for _ in range(30):
            car.step()
        self.assertLess(car.sample.soc_pct, start)
        self.assertEqual(car.sample.speed_kph, 0.0)

    def test_asleep_draws_less_than_idle(self) -> None:
        idle = _car()
        idle.step()
        asleep = _car()
        asleep.asleep()
        sample = asleep.step()
        self.assertAlmostEqual(sample.pack_current_a, 0.0, delta=0.2)
        self.assertGreater(abs(asleep.lv.current_a), 0.0)
        self.assertGreater(idle.sample.pack_current_a, 0.0)
        self.assertLess(abs(asleep.lv.current_a) * asleep.lv.voltage_v, idle.sample.pack_power_kw * 1000.0)
        self.assertFalse(sample.awake)
        self.assertTrue(sample.locked)
        self.assertEqual(asleep.signals()["battery.contactors_closed"], 0)

    def test_straight_drive_moves_and_then_idles(self) -> None:
        car = _car()
        car.follow(straight_route(18.52, 73.85, distance_m=200, heading_deg=90, speed_kph=40))
        seen_moving = False
        for _ in range(80):
            car.step()
            seen_moving = seen_moving or car.sample.speed_kph > 5
        self.assertTrue(seen_moving)
        self.assertEqual(car.sample.mode, "idle")
        self.assertGreater(car.sample.odometer_km, 0.15)

    def test_aggressive_is_quicker_than_smooth_on_the_same_road(self) -> None:
        def run(style: str) -> float:
            car = _car()
            car.follow(straight_route(18.52, 73.85, distance_m=2000, heading_deg=90, speed_kph=80), style=style)
            for _ in range(8):
                car.step(0.5)
            return car.sample.speed_kph

        self.assertGreater(run("aggressive"), run("smooth"))

    def test_fast_charge_adds_more_energy_than_ac(self) -> None:
        def gained(profile: str) -> float:
            car = _car()
            start = car.pack.energy_kwh
            car.charge(profile)
            for _ in range(80):
                car.step()
            return car.pack.energy_kwh - start

        self.assertGreater(gained("fast"), gained("ac"))

    def test_full_pack_sits_in_standby(self) -> None:
        car = Vehicle(
            VehicleCard(1, 1800, 7, 350, 18.52, 73.85, fast_charge_power_kw=50, weather="fixed"),
            soc_pct=100,
        )
        car.charge("fast")
        sample = car.step()
        self.assertFalse(sample.charging)
        self.assertEqual(sample.mode, "charge")
        self.assertEqual(car.signals()["charge.status"], "standby")
        self.assertAlmostEqual(sample.pack_current_a, 0.0, delta=2.0)

    def test_drive_while_moving_is_refused(self) -> None:
        car = _car()
        road = straight_route(18.52, 73.85, distance_m=500, heading_deg=0, speed_kph=40)
        car.follow(road)
        with self.assertRaises(RuntimeError):
            car.follow(road)

    def test_charge_stops_at_the_limit(self) -> None:
        car = Vehicle(
            VehicleCard(1, 1800, 7, 350, 18.52, 73.85, fast_charge_power_kw=150, weather="fixed"),
            soc_pct=89.0,
        )
        car.charge("fast")
        for _ in range(80):
            car.step()
        self.assertAlmostEqual(car.sample.soc_pct, 90.0, delta=0.2)
        self.assertEqual(car.signals()["charge.status"], "standby")
        self.assertEqual(car.sample.mode, "charge")

    def test_heavy_traffic_is_slower_than_light(self) -> None:
        def speed(traffic: str) -> float:
            car = _car()
            car.follow(straight_route(18.52, 73.85, distance_m=2000, heading_deg=90, speed_kph=80), traffic=traffic)
            for _ in range(40):
                car.step()
            return car.sample.speed_kph

        self.assertGreater(speed("light"), speed("heavy"))

    def test_full_pack_regen_is_cut_back(self) -> None:
        full = _car()
        full.pack.soc_pct = 99.0
        mid = _car()
        mid.pack.soc_pct = 80.0
        self.assertLess(full.pack.limits().charge_w, mid.pack.limits().charge_w)

    def test_route_that_does_not_start_here_is_refused(self) -> None:
        car = _car()
        road = straight_route(18.60, 73.85, distance_m=200, heading_deg=0, speed_kph=40)
        with self.assertRaises(RuntimeError):
            car.follow(road)


if __name__ == "__main__":
    unittest.main()
