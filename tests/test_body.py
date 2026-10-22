"""What a person does at the car, what a request on the bus does, sleep, and restart."""

from __future__ import annotations

import unittest
from datetime import datetime

from vehicle_simulator.car import Vehicle
from vehicle_simulator.card import VehicleCard
from vehicle_simulator.geo import straight_route


def _car(**card) -> Vehicle:
    card.setdefault("weather", "fixed")
    card.setdefault("sensor_noise", False)
    return Vehicle(VehicleCard(75, 1800, 7.2, 350, 18.5204, 73.8567, **card), clock=lambda: datetime(2026, 1, 1, 21, 0))


def _run(car: Vehicle, seconds: float, dt: float = 0.1) -> None:
    for _ in range(round(seconds / dt)):
        car.step(dt)


class OperatorTests(unittest.TestCase):
    def test_doors_need_the_car_unlocked(self) -> None:
        car = _car()
        with self.assertRaises(RuntimeError):
            car.door("front_left", True)
        car.unlock()
        car.door("front_left", True)
        self.assertEqual(car.signals()["body.door_front_left"], "opening")
        _run(car, 1)
        self.assertEqual(car.signals()["body.door_front_left"], "open")
        self.assertEqual(car.signals()["body.any_door_open"], 1)

    def test_trunk_takes_its_travel_time(self) -> None:
        car = _car()
        car.unlock()
        car.trunk(True)
        _run(car, 2)
        self.assertEqual(car.signals()["body.trunk"], "opening")
        _run(car, 4)
        self.assertEqual(car.signals()["body.trunk"], "open")

    def test_plug_opens_the_port_and_charging_is_separate(self) -> None:
        car = _car()
        car.set_charging(False)
        car.plug("ac")
        self.assertEqual(car.signals()["charge.port_door"], "open")
        self.assertEqual(car.signals()["charge.status"], "connected")
        soc = car.sample.soc_pct
        _run(car, 10)
        self.assertLessEqual(car.sample.soc_pct, soc)
        car.set_charging(True)
        _run(car, 10)
        self.assertEqual(car.signals()["charge.status"], "enabled")
        self.assertGreater(car.sample.soc_pct, soc)
        with self.assertRaises(RuntimeError):
            car.charge_port(False)
        car.unplug()
        car.charge_port(False)
        _run(car, 3)
        self.assertEqual(car.signals()["charge.port_door"], "closed")
        self.assertEqual(car.signals()["charge.status"], "inactive")

    def test_charge_limit_stops_charging_early(self) -> None:
        car = _car()
        car.pack.soc_pct = 59.5
        car.set_charge_limit(60)
        car.plug("fast")
        _run(car, 90, dt=1.0)
        self.assertEqual(car.signals()["charge.status"], "standby")
        self.assertAlmostEqual(car.sample.soc_pct, 60.0, delta=0.2)

    def test_cannot_drive_away_plugged_in(self) -> None:
        car = _car()
        car.plug("ac")
        route = straight_route(18.5204, 73.8567, distance_m=500, heading_deg=90, speed_kph=40)
        with self.assertRaises(RuntimeError):
            car.follow(route)

    def test_climate_moves_the_cabin_and_costs_energy(self) -> None:
        hot = _car(ambient_c=35.0)
        cold = _car(ambient_c=35.0)
        hot.climate(True)
        _run(hot, 600, dt=1.0)
        _run(cold, 600, dt=1.0)
        self.assertLess(hot.body.cabin_c, 25.0)
        self.assertAlmostEqual(cold.body.cabin_c, 35.0, delta=0.3)
        self.assertLess(hot.sample.soc_pct, cold.sample.soc_pct)
        self.assertEqual(hot.signals()["climate.power_state"], "on")

    def test_lights_auto_come_on_when_driving_at_night(self) -> None:
        car = _car()
        self.assertEqual(car.signals()["lights.low_beam"], "off")
        car.follow(straight_route(18.5204, 73.8567, distance_m=2000, heading_deg=90, speed_kph=40))
        car.step(0.1)
        self.assertEqual(car.signals()["lights.low_beam"], "on")
        car.lights("off")
        self.assertEqual(car.signals()["lights.low_beam"], "off")

    def test_honk_is_short(self) -> None:
        car = _car()
        car.honk()
        self.assertEqual(car.signals()["body.horn_on"], 1)
        _run(car, 1)
        self.assertEqual(car.signals()["body.horn_on"], 0)

    def test_drive_closes_doors_and_stop_brakes_to_park(self) -> None:
        car = _car()
        car.unlock()
        car.door("front_left", True)
        car.follow(straight_route(18.5204, 73.8567, distance_m=3000, heading_deg=90, speed_kph=60))
        self.assertEqual(car.signals()["body.door_front_left"], "closed")
        _run(car, 30)
        self.assertGreater(car.sample.speed_kph, 20)
        car.stop()
        _run(car, 3)
        self.assertTrue(car.driving)
        self.assertGreater(car.sample.speed_kph, 0)
        _run(car, 30)
        self.assertFalse(car.driving)
        self.assertEqual(car.sample.gear, "park")


class RequestTests(unittest.TestCase):
    def test_remote_unlock_then_lock(self) -> None:
        car = _car()
        car.apply_request("request.lock", "remote_unlock")
        self.assertEqual(car.signals()["body.locked"], 0)
        car.apply_request("request.lock", "remote_lock")
        self.assertEqual(car.signals()["body.locked"], 1)

    def test_charge_enable_needs_a_cable_to_do_anything(self) -> None:
        car = _car()
        car.set_charging(False)
        car.apply_request("request.charge_enable", 1)
        self.assertEqual(car.signals()["charge.status"], "inactive")
        car.plug("ac")
        _run(car, 12)
        self.assertEqual(car.signals()["charge.status"], "enabled")

    def test_rear_trunk_toggles_once_per_move(self) -> None:
        car = _car()
        for _ in range(5):
            car.apply_request("request.closure", "rear_trunk_move")
            car.step(0.1)
        _run(car, 6)
        self.assertEqual(car.signals()["body.trunk"], "open")
        car.apply_request("request.closure", "rear_trunk_move")
        self.assertEqual(car.signals()["body.trunk"], "closing")

    def test_frunk_does_not_open_while_driving(self) -> None:
        car = _car()
        car.follow(straight_route(18.5204, 73.8567, distance_m=2000, heading_deg=90, speed_kph=40))
        with self.assertRaises(RuntimeError):
            car.apply_request("request.closure", "front_trunk_move")

    def test_horn_is_held_while_requested(self) -> None:
        car = _car()
        car.apply_request("request.horn", 1)
        car.step(0.1)
        self.assertEqual(car.signals()["body.horn_on"], 1)
        _run(car, 1)
        self.assertEqual(car.signals()["body.horn_on"], 0)

    def test_climate_setpoint_seats_and_limit(self) -> None:
        car = _car()
        car.apply_request("request.climate_power", "precondition")
        car.apply_request("request.climate_setpoint_c", 20.5)
        car.apply_request("request.seat_heat_front_left", 3)
        car.apply_request("request.charge_limit_pct", 80)
        values = car.signals()
        self.assertEqual(values["climate.power_state"], "precondition")
        self.assertEqual(values["climate.mode"], "precondition")
        self.assertEqual(values["climate.setpoint_c"], 20.5)
        self.assertEqual(values["climate.seat_heat_front_left"], 3)
        self.assertEqual(values["request.charge_limit_pct"], 80)
        with self.assertRaises(ValueError):
            car.apply_request("request.charge_limit_pct", 30)

    def test_unknown_request_is_an_error(self) -> None:
        with self.assertRaises(KeyError):
            _car().apply_request("request.nothing", 1)

    def test_keep_climate_request(self) -> None:
        car = _car()
        car.apply_request("request.climate_keep", "camp")
        self.assertEqual(car.signals()["climate.mode"], "camp")
        car.apply_request("request.battery_precondition", 1)
        self.assertEqual(car.signals()["thermal.battery_precondition"], 1)
        car.apply_request("request.ac_current_limit_a", 16)
        self.assertEqual(car.signals()["charge.ac_limit_a"], 16)


class SleepTests(unittest.TestCase):
    def test_parked_car_falls_asleep_when_nothing_happens(self) -> None:
        car = _car(auto_sleep_after_s=60.0)
        _run(car, 59, dt=1.0)
        self.assertTrue(car.awake)
        _run(car, 2, dt=1.0)
        self.assertFalse(car.awake)
        self.assertEqual(car.signals()["body.locked"], 1)

    def test_climate_or_open_door_keeps_it_awake(self) -> None:
        car = _car(auto_sleep_after_s=60.0)
        car.climate(True)
        _run(car, 120, dt=1.0)
        self.assertTrue(car.awake)
        car.climate(False)
        car.unlock()
        car.door("rear_left", True)
        _run(car, 120, dt=1.0)
        self.assertTrue(car.awake)

    def test_cannot_sleep_while_charging(self) -> None:
        car = _car()
        car.plug("ac")
        with self.assertRaises(RuntimeError):
            car.asleep()

    def test_sleeping_plugged_car_wakes_to_top_up(self) -> None:
        car = _car()
        car.pack.soc_pct = 90.0
        car.plug("ac")
        car.set_charging(True)
        car.asleep()
        car.pack.soc_pct = 89.0
        _run(car, 15)
        self.assertTrue(car.awake)
        self.assertEqual(car.signals()["charge.status"], "enabled")

    def test_sleep_turns_the_cabin_off(self) -> None:
        car = _car()
        car.climate(True)
        car.seat_heater("all", 2)
        car.asleep()
        values = car.signals()
        self.assertEqual(values["climate.power_state"], "off")
        self.assertEqual(values["climate.seat_heat_rear_center"], 0)


class BlockedActionTests(unittest.TestCase):
    """The console disables the same actions the car would refuse, with the same words."""

    def test_parked_locked_car_blocks_the_obvious_moves(self) -> None:
        car = _car()
        blocked = car.blocked_actions()
        self.assertEqual(blocked["lock"], "already locked")
        self.assertEqual(blocked["door_front_left_open"], "the car is locked")
        self.assertEqual(blocked["trunk_open"], "the car is locked")
        self.assertEqual(blocked["frunk_open"], "the car is locked")
        self.assertEqual(blocked["stop"], "the car is not driving")
        self.assertEqual(blocked["unplug"], "no cable")
        self.assertEqual(blocked["leave"], "nobody is in the car")
        self.assertNotIn("drive", blocked)
        self.assertNotIn("unlock", blocked)
        self.assertNotIn("sleep", blocked)
        with self.assertRaisesRegex(RuntimeError, "the car is locked"):
            car.door("front_left", True)
        with self.assertRaisesRegex(RuntimeError, "nobody is in the car"):
            car.leave()

    def test_plug_blocks_drive_and_the_other_plug(self) -> None:
        car = _car()
        car.plug("ac")
        blocked = car.blocked_actions()
        self.assertEqual(blocked["drive"], "unplug before driving")
        self.assertEqual(blocked["plug_ac"], "the cable is already in")
        self.assertEqual(blocked["plug_fast"], "unplug before plugging in a different charger")
        self.assertEqual(blocked["charge_port_close"], "the cable is in the port")
        self.assertNotIn("unplug", blocked)
        with self.assertRaisesRegex(RuntimeError, "unplug before driving"):
            car.follow(straight_route(18.5204, 73.8567, distance_m=200, heading_deg=90, speed_kph=40))
        with self.assertRaisesRegex(RuntimeError, "unplug before plugging"):
            car.plug("fast")
        with self.assertRaisesRegex(RuntimeError, "the cable is in the port"):
            car.charge_port(False)

    def test_keep_mode_and_charge_block_sleep(self) -> None:
        car = _car()
        car.climate_mode_set("keep")
        self.assertEqual(car.why_not("sleep"), "keep mode keeps the car awake")
        with self.assertRaisesRegex(RuntimeError, "keep mode keeps the car awake"):
            car.asleep()
        car.climate_mode_set("off")
        car.plug("ac")
        self.assertEqual(car.why_not("sleep"), "the car stays awake while it charges")

    def test_driving_blocks_plug_frunk_and_parked_climate(self) -> None:
        car = _car()
        car.unlock()
        car.follow(straight_route(18.5204, 73.8567, distance_m=2000, heading_deg=90, speed_kph=40))
        _run(car, 5)
        blocked = car.blocked_actions()
        self.assertEqual(blocked["drive"], "the car is already driving")
        self.assertEqual(blocked["plug_ac"], "park before plugging in")
        self.assertEqual(blocked["frunk_open"], "the car is driving")
        self.assertEqual(blocked["climate_camp"], "camp mode is for a parked car")
        self.assertNotIn("stop", blocked)
        with self.assertRaisesRegex(RuntimeError, "park before plugging in"):
            car.plug("ac")
        with self.assertRaisesRegex(RuntimeError, "camp mode is for a parked car"):
            car.climate_mode_set("camp")

    def test_low_soc_blocks_dog_mode(self) -> None:
        car = _car(keep_climate_min_soc_pct=20.0)
        car.pack.soc_pct = 10.0
        self.assertIn("at least 20 % charge", car.why_not("climate_dog"))
        with self.assertRaisesRegex(RuntimeError, "at least 20 % charge"):
            car.climate_mode_set("dog")


class RestartTests(unittest.TestCase):
    def test_snapshot_round_trip(self) -> None:
        car = _car()
        car.unlock()
        car.trunk(True)
        car.plug("fast")
        car.set_charge_limit(80)
        car.climate(True)
        car.set_temperature(19.5)
        car.seat_heater("front_right", 2)
        car.lights("on")
        _run(car, 10)
        saved = car.snapshot()
        again = _car()
        again.restore(saved)
        for key in ("body.locked", "body.trunk", "charge.plug", "charge.limit_pct", "climate.setpoint_c", "climate.seat_heat_front_right", "lights.mode", "charge.port_door"):
            self.assertEqual(again.signals()[key], car.signals()[key], key)
        self.assertAlmostEqual(again.sample.soc_pct, car.sample.soc_pct, places=6)

    def test_a_drive_does_not_survive_restart(self) -> None:
        car = _car()
        car.follow(straight_route(18.5204, 73.8567, distance_m=2000, heading_deg=90, speed_kph=40))
        _run(car, 20)
        again = _car()
        again.restore(car.snapshot())
        self.assertFalse(again.driving)
        self.assertEqual(again.sample.gear, "park")
        self.assertAlmostEqual(again.sample.latitude_deg, car.sample.latitude_deg, places=6)


if __name__ == "__main__":
    unittest.main()
