# Using another DBC

The car does not change. Physics lives on the plant (`PHYSICS.md`, `signals.md`). A different vehicle is a DBC file plus `alias.json`. The file name stays `alias.json`. Sensors the DBC omits still exist and still drive the rest of the car.

1. Put the `.dbc` file in `dbc/` and set `dbc` to `dbc/your-file.dbc`.
2. Map only the signals this vehicle actually has. Leave out the rest.
3. Start the car. It refuses to start if a row names a DBC signal that is not there, or a car name (left of a row) the car does not have.

```bash
python3 -m vehicle_simulator examples/ev_card.json
```

## alias.json

| Key | What it is |
|---|---|
| `dbc` | DBC path, relative to the simulator folder. |
| `interfaces` | Interface name to UDP port, e.g. `{"can0": 47000, "can1": 47001}`. |
| `interface_by_sender` | DBC transmitter to interface. A message goes out on its sender's interface. |
| `default_interface` | For a message whose sender is not listed. |
| `periods_ms` | Message name to how often it is sent. |
| `default_period_ms` | For every other message. Default 1000. The DBC `GenMsgCycleTime` is used when the alias does not name the message. |
| `fixed` | `Message.Signal` to a constant: a DBC value name such as `"DI_SYS_ENABLE"`, or a physical number. |
| `signals` | The rows. |

## Rows

A row is either a status the car sends or a request the car listens for.

```json
{"signal": "ID102VCLEFT_doorStatus.VCLEFT_frontLatchStatus", "plant": "body.door_front_left",
 "values": {"closed": "LATCH_CLOSED", "opening": "LATCH_OPENING", "open": "LATCH_OPENED", "closing": "LATCH_CLOSING"}}

{"signal": "ID273UI_vehicleControl.UI_lockRequest", "request": "request.lock",
 "values": {"idle": "UI_LOCK_REQUEST_IDLE", "remote_unlock": "UI_LOCK_REQUEST_REMOTE_UNLOCK"}}
```

- `plant` rows are sent from what the car is doing. Names are in `signals.md`.
- `request` rows are sent too, with what the car's own controls ask for (a lock request is `idle`, climate is the current setting). When a frame from another node carries a different value, the car does it. A value with no name in `values`, such as `SNA`, is ignored.
- `values` maps the car's word to a DBC value name or a raw number. Text values need it. Numbers use the DBC scale and offset, and are clamped to the DBC range.
- One car name can feed several rows (left and right low beam). A DBC signal can be on only one row.

## What goes on a frame

A message is sent if it has a row or a `fixed` value. Signals in it with no row are set to a neutral filler (0, or the nearest DBC limit). A muxed message is sent once per page that has a row on it. The selector is set for you, including nested selectors.

Car names with no row, such as yaw or grade, are still calculated. They are just not on the bus.

If cantools cannot load the DBC, the simulator will not start.
