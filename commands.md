# Commands

There are two ways to make the car do something.

1. **Operator commands** in this file are what a person in or at the car does: open a door, plug in, turn on climate. They print nothing. The car does what the real car would do, and the frames change. If the real car would not do it (open a door while locked, drive away while plugged in), nothing happens. Run with `--verbose` to see those on stderr.
2. **Requests on the bus.** Any other node (for example the remote-ops app) can put a request frame such as `UI_lockRequest` on the bus. The car reads it and acts on it the same way. Which signals are requests is set in `alias.json`.

```bash
python3 -m vehicle_simulator examples/ev_card.json
```

## Console

The car also serves an operator console at <http://127.0.0.1:8090/>. It shows the car on an OpenStreetMap map, pointing the way it is heading. Click the map to pick a destination, then **Drive**. While it drives, the route is drawn: blue ahead, grey behind. The panel has every command below. Each control shows what the car is doing now. A door button reads `open`, `closed`, or `opening`. A button that the car does not act on simply leaves the state as it was, the same as on stdin.

The map tiles come from `tile.openstreetmap.org`, so the page needs the internet. Use `--ui-port 0` to turn the console off.

## Writing commands

Commands are JSON, one per line, on stdin or in `command.json` in the folder you started from. The file is read every 100 ms and then deleted. It can hold several lines.

```bash
echo '{"command": "unlock"}' > command.json
```

Every operator command except `sleep` wakes the car first, the way opening a door or touching the screen does. A frame on the bus from another node also wakes it.

Home pin for `examples/ev_card.json` is Colive Riga, BTM Layout, Bengaluru (12.90873, 77.609752). Click the map or search a place in the Drive panel to pick a destination.

## Run options

| Option | Default | What it does |
|---|---|---|
| `--state` | `state.json` | Where the car is saved every 10 s and on exit. Next start continues from it. |
| `--fresh` | off | Ignore the saved state and start at the home pin. |
| `--soc` | saved, or 80 | Start at this SOC. |
| `--odometer-km` | saved, or 0 | Start at this odometer. |
| `--command-file` | `command.json` | The command file. |
| `--udp-host` | `239.255.70.1` | The bus group. |
| `--quiet` | off | Do not print the latest frames once a second. |
| `--verbose` | off | Say on stderr when the car could not do something. |
| `--ui-port` | 8090 | Console port. 0 turns it off. |
| `--ui-host` | `127.0.0.1` | Console address. |
| `--weather` | `live` | `live` Open-Meteo at the pin, `curve` daily swing, or `fixed` card ambient. |

How every plant signal is calculated is in `PHYSICS.md`. Plant names are in `signals.md`. The DBC mapping is `alias.json` (see `dbc.md`).

A drive in progress does not survive a restart. The car is parked where it was.

## Watch the bus

Each interface in `alias.json` is its own port. The Model 3 alias has `can0` on 47000 (VehicleBus) and `can1` on 47001 (ChassisBus). Each frame is one datagram:

```json
{"id": 599, "ext": false, "data": "0080...", "src": "car-1a2b3c4d"}
```

There is no time on the wire. A reader stamps a frame when it arrives, as a CAN socket would.

```bash
python3 -m vehicle_simulator.udp --port 47000
```

## Driving

| Command | Fields | What happens |
|---|---|---|
| `drive` | `lat`, `lon`, `style` (`smooth`, `aggressive`), `traffic` (`light`, `moderate`, `heavy`), `speed_kph` | Looks up the road, unlocks, closes the doors, turns climate on, and drives there. Not while plugged in or already driving. |
| `stop` | | Brakes to a stop where it is, then parks. |

```bash
echo '{"command": "drive", "lat": 18.5310, "lon": 73.8446, "style": "smooth", "traffic": "light"}' > command.json
```

## Charging

| Command | Fields | What happens |
|---|---|---|
| `plug` | `type`: `ac` or `fast` | The cable goes in. The port door opens. Charging starts if charging is on. Not while driving. |
| `unplug` | | The cable comes out. The port door stays open until closed. |
| `charging` | `on`: true or false | The charge on/off setting. With no cable it is kept for next time. |
| `charge_limit` | `pct`: 50 to 100 | Charging stops here. |
| `charge_current` | `amps`: 5 to 48 | User AC current cap. |
| `charge_schedule` | `start`: `HH:MM` or null | Six-hour window from that local time. Empty charges now. |
| `charge_port` | `open`: true or false | Port door. It cannot close with the cable in. |
| `charge` | `profile`: `ac` or `fast` | Same as `plug` then `charging on`. |

The session goes handshake (AC) or cable-check / precharge (DC), then ramp, then charging. Status on the bus is `inactive`, `connected`, `ext_evse_test`, `evse_test_passed`, `enabled`, `standby`. A sleeping car that is plugged in wakes to top up when SOC drops below the top-up line. The cable latch follows lock while plugged.

## Doors and locks

| Command | Fields | What happens |
|---|---|---|
| `lock` | | Locks. When parked this also turns climate and seat heaters off, as walking away does. |
| `unlock` | | Unlocks. |
| `set_lock` | `locked`: true or false | Same as `lock` or `unlock`. |
| `door` | `door`: `front_left`, `front_right`, `rear_left`, `rear_right`, or `all`; `open` | Opening needs the car unlocked and parked. |
| `trunk` | `open` | Takes about 5 s. Opening needs unlocked and parked. |
| `frunk` | `open` | Opening needs unlocked and parked. |
| `windows` | `position`: `closed`, `vent`, `open` | |
| `enter` | | Unlock, sit in the driver seat. |
| `leave` | | Park, open then shut the driver door, walk away. Locks after `walk_away_lock_s`. |

## Cabin

| Command | Fields | What happens |
|---|---|---|
| `climate` | `on` | Cabin `on` or `off`. |
| `climate_mode` | `mode`: `off`, `on`, `precondition`, `keep`, `dog`, `camp` | Parked climate. Keep / dog / camp need SOC above the floor and block sleep. |
| `temperature` | `c`: 15 to 28 | Set temperature. |
| `battery_precondition` | `on` | Heat the pack toward the DC-charge window. |
| `overheat_protection` | `mode`: `ac`, `fan`, `off` | Cabin overheat while locked and empty. |
| `seat_heater` | `seat`: `front_left`, `front_right`, `rear_left`, `rear_center`, `rear_right`, or `all`; `level`: 0 to 3 | |
| `honk` | | Half a second of horn. |
| `lights` | `mode`: `auto`, `on`, `off`, `parking` | In `auto` the low beam is on while driving at night. |

## Sleep

| Command | What happens |
|---|---|
| `sleep` (or `asleep`) | Locks, turns the cabin off, and sleeps. No frames are sent. Not while driving or charging. |
| `wake` | Wakes the car. |

A parked car also falls asleep by itself after `auto_sleep_after_s` (card, default 900) with nothing going on: no charging, no climate, no horn, doors closed, nothing moving.
