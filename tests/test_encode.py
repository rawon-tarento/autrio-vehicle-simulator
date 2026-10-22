"""The encoder works from the DBC and alias.json alone. Nothing about one vehicle is built in."""

from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from vehicle_simulator.car import Vehicle
from vehicle_simulator.card import VehicleCard
from vehicle_simulator.encode import Frame, load_encoder

_ALIAS = Path(__file__).resolve().parents[1] / "alias.json"
_OFF_BUS = {
    "vehicle.yaw_rate_rads",
    "vehicle.heading_deg",
    "vehicle.grade",
    "vehicle.elevation_m",
    "vehicle.pedal_pct",
    "battery.pack_power_kw",
    "battery.soh_pct",
}

_TOY_DBC = """VERSION ""

NS_ :

BS_:

BU_: Body Panel

BO_ 256 Status: 4 Body
 SG_ Index M : 0|2@1+ (1,0) [0|3] "" Vector__XXX
 SG_ Speed : 8|8@1+ (1,0) [0|250] "kph" Vector__XXX
 SG_ Temp m0 : 16|8@1+ (0.5,-20) [-20|100] "C" Vector__XXX
 SG_ Level m1 : 16|8@1+ (1,0) [0|100] "%" Vector__XXX
 SG_ Spare m1 : 24|8@1+ (1,0) [0|255] "" Vector__XXX

BO_ 512 Request: 2 Panel
 SG_ Mode : 0|2@1+ (1,0) [0|3] "" Vector__XXX
 SG_ Target : 8|8@1+ (0.5,0) [0|100] "" Vector__XXX

VAL_ 512 Mode 0 "IDLE" 1 "GO" 2 "HALT" 3 "SNA" ;
"""


def _car() -> Vehicle:
    return Vehicle(VehicleCard(75, 1800, 7.2, 350, 18.5204, 73.8567, weather="fixed", sensor_noise=False))


def _toy(tmp: Path, **extra) -> Path:
    (tmp / "toy.dbc").write_text(_TOY_DBC)
    alias = {
        "dbc": "toy.dbc",
        "interfaces": {"a": 47100, "b": 47101},
        "interface_by_sender": {"Body": "a", "Panel": "b"},
        "periods_ms": {"Status": 100},
        "fixed": {"Status.Spare": 7},
        "signals": [
            {"signal": "Status.Speed", "plant": "x.speed"},
            {"signal": "Status.Temp", "plant": "x.temp"},
            {"signal": "Status.Level", "plant": "x.level"},
            {"signal": "Request.Mode", "request": "rq.mode", "values": {"idle": "IDLE", "go": "GO", "halt": 2}},
            {"signal": "Request.Target", "request": "rq.target"},
        ],
        **extra,
    }
    path = tmp / "alias.json"
    path.write_text(json.dumps(alias))
    return path


class ModelThreeAliasTests(unittest.TestCase):
    def test_every_alias_name_is_something_the_car_has(self) -> None:
        load_encoder(_ALIAS).check(_car().signals())

    def test_idle_round_trip_keeps_use_case_signals_only(self) -> None:
        encoder = load_encoder(_ALIAS)
        car = _car()
        car.step()
        frames = encoder.encode_all(car.signals())
        decoded = encoder.decode(frames)
        self.assertTrue(_OFF_BUS.isdisjoint(decoded))
        self.assertEqual(decoded["vehicle.gear"], "park")
        self.assertAlmostEqual(decoded["vehicle.speed_kph"], 0.0, delta=0.08)
        self.assertAlmostEqual(decoded["battery.soc_pct"], car.sample.soc_pct, delta=0.1)
        self.assertAlmostEqual(decoded["vehicle.latitude_deg"], 18.5204, delta=0.0001)
        self.assertEqual(decoded["charge.status"], "inactive")
        self.assertEqual(decoded["body.door_front_left"], "closed")
        self.assertEqual(decoded["climate.power_state"], "off")

    def test_status_follows_what_happens_at_the_car(self) -> None:
        encoder = load_encoder(_ALIAS)
        car = _car()
        car.unlock()
        car.door("front_left", True)
        car.plug("ac")
        car.climate(True)
        for _ in range(120):
            car.step(0.1)
        car.honk()
        car.step(0.1)
        decoded = encoder.decode(encoder.encode_all(car.signals()))
        self.assertEqual(decoded["body.door_front_left"], "open")
        self.assertEqual(decoded["body.any_door_open"], 1.0)
        self.assertEqual(decoded["charge.status"], "enabled")
        self.assertEqual(decoded["climate.power_state"], "on")
        self.assertEqual(decoded["body.horn_on"], 1.0)

    def test_fixed_values_are_on_the_frame(self) -> None:
        encoder = load_encoder(_ALIAS)
        frames = encoder.encode_all(_car().signals())
        frame = next(f for f in frames if f.name == "ID118DriveSystemStatus")
        message = next(item.message for item in encoder.tx if item.message.name == frame.name)
        decoded = message.decode(frame.data)
        self.assertEqual(str(decoded["DI_systemState"]), "DI_SYS_STANDBY")
        self.assertEqual(str(decoded["DI_immobilizerState"]), "DI_IMM_STATE_DISARMED")

    def test_senders_land_on_their_interface(self) -> None:
        encoder = load_encoder(_ALIAS)
        by_name = {item.message.name: item.interface for item in encoder.tx}
        self.assertEqual(by_name["ID257DIspeed"], "can0")
        self.assertEqual(by_name["ID04FGPSLatLong"], "can1")

    def test_remote_lock_frame_becomes_a_request(self) -> None:
        encoder = load_encoder(_ALIAS)
        car = _car()
        item = next(item for item in encoder.tx if item.message.name == "ID273UI_vehicleControl")
        own = encoder.encode_all(car.signals())
        frame = next(f for f in own if f.frame_id == item.message.frame_id)
        self.assertEqual(encoder.requests("can0", frame.frame_id, False, frame.data, car.request_values()), [])
        raw = item.message.decode(frame.data, decode_choices=False, scaling=False)
        raw["UI_lockRequest"] = 3
        data = item.message.encode(raw, scaling=False, strict=False)
        changes = encoder.requests("can0", frame.frame_id, False, data, car.request_values())
        self.assertEqual(changes, [("request.lock", "remote_unlock")])


class AnyDbcTests(unittest.TestCase):
    def setUp(self) -> None:
        self._tmp = tempfile.TemporaryDirectory()
        self.tmp = Path(self._tmp.name)

    def tearDown(self) -> None:
        self._tmp.cleanup()

    def _values(self) -> dict:
        return {"x.speed": 42, "x.temp": 21.5, "x.level": 64, "rq.mode": "idle", "rq.target": 30.0}

    def test_signals_on_two_mux_pages_make_two_frames(self) -> None:
        encoder = load_encoder(_toy(self.tmp))
        frames = [f for f in encoder.encode_all(self._values()) if f.frame_id == 256]
        self.assertEqual(len(frames), 2)
        decoded = encoder.decode(frames)
        self.assertEqual(decoded["x.speed"], 42.0)
        self.assertEqual(decoded["x.temp"], 21.5)
        self.assertEqual(decoded["x.level"], 64.0)
        message = encoder.tx[0].message
        page1 = next(f for f in frames if message.decode(f.data)["Index"] == 1)
        self.assertEqual(message.decode(page1.data)["Spare"], 7)

    def test_each_sender_has_its_interface(self) -> None:
        encoder = load_encoder(_toy(self.tmp))
        self.assertEqual({item.message.name: item.interface for item in encoder.tx}, {"Status": "a", "Request": "b"})

    def test_periods_come_from_the_alias_then_the_default(self) -> None:
        encoder = load_encoder(_toy(self.tmp))
        counts: dict[int, int] = {}
        t = 0.0
        while t < 1.0 - 1e-9:
            for frame in encoder.due(t, self._values()):
                counts[frame.frame_id] = counts.get(frame.frame_id, 0) + 1
            t += 0.01
        self.assertEqual(counts[256], 20)
        self.assertEqual(counts[512], 1)

    def test_requests_use_dbc_value_names_and_scale(self) -> None:
        encoder = load_encoder(_toy(self.tmp))
        message = next(item.message for item in encoder.tx if item.message.name == "Request")
        data = message.encode({"Mode": 1, "Target": 100}, scaling=False)
        changes = encoder.requests("b", 512, False, data, self._values())
        self.assertEqual(changes, [("rq.mode", "go"), ("rq.target", 50.0)])
        self.assertEqual(encoder.requests("a", 512, False, data, self._values()), [])
        unknown = message.encode({"Mode": 3, "Target": 60}, scaling=False)
        self.assertEqual(encoder.requests("b", 512, False, unknown, self._values()), [])

    def test_check_names_the_missing_signals(self) -> None:
        encoder = load_encoder(_toy(self.tmp))
        with self.assertRaisesRegex(ValueError, "x.level"):
            encoder.check({"x.speed", "x.temp", "rq.mode", "rq.target"})

    def test_bad_alias_is_refused(self) -> None:
        with self.assertRaisesRegex(ValueError, "unknown keys"):
            load_encoder(_toy(self.tmp, extra_thing=1))
        with self.assertRaisesRegex(ValueError, "no value named"):
            alias = json.loads(_toy(self.tmp).read_text())
            alias["signals"][3]["values"]["go"] = "GOING"
            (self.tmp / "alias.json").write_text(json.dumps(alias))
            load_encoder(self.tmp / "alias.json")

    def test_frame_wire_has_no_time_or_sender(self) -> None:
        self.assertEqual(Frame(0x257, b"\x01\x02", "can0").wire(), {"id": 0x257, "ext": False, "data": "0102"})


if __name__ == "__main__":
    unittest.main()
