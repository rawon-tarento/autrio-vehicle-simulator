"""The plant equations: handshake, shore power, IMU gravity, sag, regen fade, keep-mode, auto-lock."""

from __future__ import annotations

import math
import unittest

from vehicle_simulator.car import Vehicle
from vehicle_simulator.card import VehicleCard
from vehicle_simulator.geo import straight_route
from vehicle_simulator.pack import Pack
from vehicle_simulator.powertrain import G, drive_step


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
            weather="fixed",
            sensor_noise=False,
            **extra,
        )
    )


def _run(car: Vehicle, seconds: float, dt: float = 0.1) -> None:
    for _ in range(round(seconds / dt)):
        car.step(dt)


class HandshakeTests(unittest.TestCase):
    def test_ac_handshake_then_ramp(self) -> None:
        car = _car(ac_handshake_s=3.0, ac_ramp_s=8.0)
        car.charge("ac")
        car.step(0.1)
        self.assertEqual(car.signals()["charge.stage"], "handshake")
        self.assertEqual(car.signals()["charge.status"], "connected")
        self.assertEqual(car.signals()["charge.cable_present"], 1)
        _run(car, 3.0)
        self.assertEqual(car.signals()["charge.stage"], "ramp")
        self.assertEqual(car.signals()["charge.status"], "enabled")
        _run(car, 9.0)
        self.assertEqual(car.signals()["charge.stage"], "charging")
        self.assertGreater(car.signals()["charge.line_current_a"], 1.0)
        self.assertLess(car.pack.current_a, 0.0)

    def test_dc_cable_check_then_precharge(self) -> None:
        car = _car(dc_cable_check_s=2.0, dc_precharge_s=2.0, dc_ramp_s=2.0)
        car.charge("fast")
        car.step(0.1)
        self.assertEqual(car.signals()["charge.stage"], "cable_check")
        self.assertEqual(car.signals()["charge.status"], "ext_evse_test")
        _run(car, 2.0)
        self.assertEqual(car.signals()["charge.stage"], "precharge")
        self.assertEqual(car.signals()["charge.status"], "evse_test_passed")
        _run(car, 2.0)
        self.assertEqual(car.signals()["charge.status"], "enabled")
        self.assertGreater(car.signals()["charge.evse_dc_voltage_v"], 300.0)


class ShorePowerTests(unittest.TestCase):
    def test_station_covers_hvac_before_the_pack(self) -> None:
        climate = _car(ambient_c=35.0)
        climate.body.cabin_c = 35.0
        climate.climate(True)
        _run(climate, 5.0)
        climate_only = climate.pack.current_a

        plugged = _car(ambient_c=35.0, ac_handshake_s=0.1, ac_ramp_s=0.1)
        plugged.body.cabin_c = 35.0
        plugged.climate(True)
        plugged.charge("ac")
        _run(plugged, 5.0)
        self.assertGreater(climate_only, 0.0)
        self.assertLess(plugged.pack.current_a, climate_only)
        self.assertGreater(plugged.signals()["charge.line_power_kw"], 0.5)


class SensorPhysicsTests(unittest.TestCase):
    def test_accelerometer_reports_gravity_on_a_slope(self) -> None:
        car = _car()
        car.sample.grade = 0.2
        car._sync()
        expected = G * math.sin(math.atan(0.2))
        self.assertAlmostEqual(car.signals()["vehicle.long_accel_mps2"], expected, places=5)

    def test_cluster_speed_reads_high(self) -> None:
        car = _car(speedo_overread=0.02)
        car.sample.speed_kph = 100.0
        self.assertAlmostEqual(car.signals()["vehicle.display_speed_kph"], 102.0, places=5)

    def test_gps_is_coarse_just_after_waking(self) -> None:
        car = _car(gps_fix_s=10.0)
        car.asleep()
        car.wake()
        car.step(0.1)
        cold = car.signals()["vehicle.gps_accuracy_m"]
        _run(car, 30.0)
        self.assertGreater(cold, car.signals()["vehicle.gps_accuracy_m"])


class PackPhysicsTests(unittest.TestCase):
    def test_voltage_sags_under_load_then_recovers(self) -> None:
        pack = Pack(_car().card, soc_pct=80.0, temp_c=25.0)
        ocv = pack.ocv
        pack.step(40_000.0, 1.0, contactors_closed=True, ambient_c=25.0)
        self.assertLess(pack.voltage_v, ocv)
        loaded = pack.voltage_v
        pack.step(0.0, 60.0, contactors_closed=True, ambient_c=25.0)
        self.assertGreater(pack.voltage_v, loaded)
        self.assertLess(abs(pack.voltage_v - pack.ocv), 1.0)

    def test_regen_fades_at_walking_speed(self) -> None:
        card = _car().card
        limits = _car().pack.limits()
        kwargs = dict(
            card=card,
            want_accel_mps2=-2.0,
            grade=0.0,
            max_discharge_w=limits.discharge_w,
            max_charge_w=limits.charge_w,
            other_loads_w=0.0,
        )
        walking = drive_step(speed_mps=2.0 / 3.6, **kwargs)
        rolling = drive_step(speed_mps=40.0 / 3.6, **kwargs)
        self.assertLess(abs(walking.motor_force_n), abs(rolling.motor_force_n))
        self.assertLess(walking.friction_force_n, -1.0)


class InterlockTests(unittest.TestCase):
    def test_keep_mode_refuses_sleep(self) -> None:
        car = _car()
        car.climate_mode_set("keep")
        with self.assertRaises(RuntimeError):
            car.asleep()
        self.assertEqual(car.signals()["climate.mode"], "keep")
        self.assertEqual(car.signals()["climate.power_state"], "on")

    def test_keep_turns_off_when_the_pack_is_low(self) -> None:
        car = _car(keep_climate_min_soc_pct=20.0)
        car.climate_mode_set("dog")
        car.pack.soc_pct = 19.0
        car.step(0.1)
        self.assertEqual(car.signals()["climate.mode"], "off")

    def test_pull_away_locks_the_doors(self) -> None:
        car = _car(auto_lock_kph=8.0)
        car.unlock()
        car.follow(straight_route(18.52, 73.85, distance_m=2000, heading_deg=90, speed_kph=60))
        self.assertEqual(car.signals()["body.locked"], 0)
        _run(car, 8.0)
        self.assertGreater(car.sample.speed_kph, 8.0)
        self.assertEqual(car.signals()["body.locked"], 1)

    def test_plugged_car_holds_park(self) -> None:
        car = _car()
        car.charge("ac")
        with self.assertRaises(RuntimeError):
            car.follow(straight_route(18.52, 73.85, distance_m=200, heading_deg=0, speed_kph=40))
        self.assertEqual(car.signals()["vehicle.gear"], "park")


class WeatherTests(unittest.TestCase):
    def test_fixed_ambient_does_not_follow_the_sun(self) -> None:
        car = _car(ambient_c=12.0)
        _run(car, 5.0)
        self.assertAlmostEqual(car.signals()["environment.ambient_c"], 12.0, places=5)
        self.assertAlmostEqual(car.signals()["environment.ambient_filtered_c"], 12.0, places=3)


if __name__ == "__main__":
    unittest.main()
