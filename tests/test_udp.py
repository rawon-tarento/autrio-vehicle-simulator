"""One datagram per frame, on the port of its interface. A node does not hear itself."""

from __future__ import annotations

import json
import socket
import time
import unittest

from vehicle_simulator.encode import Frame
from vehicle_simulator.udp import GROUP, CanBus, join


def _free_port() -> int:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", 0))
    port = sock.getsockname()[1]
    sock.close()
    return port


class CanBusTests(unittest.TestCase):
    def test_each_frame_is_one_datagram_on_its_interface(self) -> None:
        can0, can1 = _free_port(), _free_port()
        reader0 = join(GROUP, can0)
        reader1 = join(GROUP, can1)
        reader0.settimeout(1)
        reader1.settimeout(1)
        bus = CanBus({"can0": can0, "can1": can1}, src="car", listen=False)
        bus.send([Frame(0x257, b"\x01", "can0"), Frame(0x04F, b"\x02\x03", "can1"), Frame(0x118, b"\x04", "can0")])
        bus.close()
        first = json.loads(reader0.recvfrom(65535)[0])
        second = json.loads(reader0.recvfrom(65535)[0])
        other = json.loads(reader1.recvfrom(65535)[0])
        reader0.close()
        reader1.close()
        self.assertEqual(first, {"id": 0x257, "ext": False, "data": "01", "src": "car"})
        self.assertEqual(second["id"], 0x118)
        self.assertEqual(other, {"id": 0x04F, "ext": False, "data": "0203", "src": "car"})

    def test_a_node_hears_others_but_not_itself(self) -> None:
        port = _free_port()
        car = CanBus({"can0": port}, src="car")
        app = CanBus({"can0": port}, src="remote-ops", listen=False)
        car.send([Frame(0x273, b"\x00", "can0")])
        app.send([Frame(0x273, b"\x03", "can0")])
        received = []
        deadline = time.monotonic() + 1.0
        while not received and time.monotonic() < deadline:
            received = car.receive()
            time.sleep(0.01)
        car.close()
        app.close()
        self.assertEqual(len(received), 1)
        self.assertEqual((received[0].src, received[0].frame_id, received[0].data), ("remote-ops", 0x273, b"\x03"))


if __name__ == "__main__":
    unittest.main()
