# autrio-vehicle-simulator

Live electric-vehicle plant for the DriveStream lab CAN bus. Physics lives on plant names. `alias.json` maps whichever of those names a DBC actually has. Sensors the DBC omits still exist and still drive the rest of the car.

Lab clone: `ds-flow-sandbox/lab/vehicle-simulator`. Work here, commit, and push so others can pull the same history.

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
.venv/bin/python -m vehicle_simulator examples/ev_card.json
```

The car joins UDP multicast `239.255.70.1` (`can0` 47000, `can1` 47001). It serves an operator console at <http://127.0.0.1:8090/>. `--ui-port 0` turns the console off.

```bash
echo '{"command": "unlock"}' > command.json
```

## Docs

| File | What it is |
|---|---|
| `PHYSICS.md` | How every plant signal is calculated |
| `signals.md` | Plant names the car produces |
| `commands.md` | Operator commands, console, run options |
| `dbc.md` | How to point the car at another DBC |
| `bench/BENCHMARK.md` | Physics scorecard vs EPA / FASTSim |
| `bench/COMPARISON.md` | Head-to-head with autrio-car-sim |

## Tests and bench

```bash
python3 -m unittest discover -s tests
python3 bench/run_benchmark.py
```

`state.json` is the live snapshot. It is not in git. `--fresh` starts at the card's home pin.
