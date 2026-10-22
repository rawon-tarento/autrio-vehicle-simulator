"""The console serves the page and the latest state, and queues commands for the car's loop."""

from __future__ import annotations

import json
import unittest
import urllib.error
import urllib.request

from vehicle_simulator.ui import Console


class ConsoleTests(unittest.TestCase):
    def setUp(self) -> None:
        self.console = Console("127.0.0.1", 0)
        self.console.start()
        self.base = self.console.url

    def tearDown(self) -> None:
        self.console.close()

    def _get(self, path: str) -> bytes:
        with urllib.request.urlopen(self.base + path, timeout=2) as response:
            return response.read()

    def _post(self, body: bytes) -> int:
        request = urllib.request.Request(self.base + "command", data=body, method="POST", headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(request, timeout=2) as response:
                return response.status
        except urllib.error.HTTPError as exc:
            return exc.code

    def test_page_state_and_route(self) -> None:
        page = self._get("")
        self.assertIn(b"leaflet", page)
        self.assertIn(b"applyBlocked", page)
        self.assertIn(b"blockReason", page)
        self.assertIn(b"charge-enable-hint", page)
        self.assertIn(b"On next plug", page)
        self.assertIn(b'data-panel="drive"', page)
        self.assertIn(b"drawer-close", page)
        self.assertIn(b"setPanel", page)
        self.assertIn(b'id="dest-q"', page)
        self.assertIn(b"/geocode", page)
        self.assertIn(b"12.90873", page)
        self.console.publish({"route_id": 1}, {"points": [[1, 2]], "s_m": [0], "length_m": 0}, route_changed=True)
        self.console.publish({"route_id": 1, "travelled_m": 5})
        self.assertEqual(json.loads(self._get("state")), {"route_id": 1, "travelled_m": 5})
        self.assertEqual(json.loads(self._get("route"))["points"], [[1, 2]])

    def test_geocode_requires_query(self) -> None:
        request = urllib.request.Request(self.base + "geocode", method="GET")
        with self.assertRaises(urllib.error.HTTPError) as caught:
            urllib.request.urlopen(request, timeout=2)
        self.assertEqual(caught.exception.code, 400)

    def test_commands_are_queued_not_answered(self) -> None:
        self.assertEqual(self._post(b'{"command": "honk"}'), 202)
        self.assertEqual(self.console.commands.get_nowait(), {"command": "honk"})
        self.assertEqual(self._post(b"not json"), 400)
        self.assertEqual(self._post(b'{"lat": 1}'), 400)
        self.assertTrue(self.console.commands.empty())


if __name__ == "__main__":
    unittest.main()
