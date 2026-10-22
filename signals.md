# Signals

Plant names the car produces. `alias.json` maps a subset onto a DBC. Names the DBC does not have still exist and still drive the rest of the car. How each one is calculated is in `PHYSICS.md`.

Physics runs at 10 steps a second. Each CAN message is sent at its own period. Asleep sends nothing.

Positive `battery.pack_current_a` is discharge. Negative is charge or regen.

## Vehicle

| Signal | Unit | What it is |
|---|---|---|
| `vehicle.speed_kph` | km/h | True speed along the path |
| `vehicle.display_speed_kph` | km/h | Cluster speed, a few percent high |
| `vehicle.long_accel_mps2` | m/s² | Accelerometer: \(a + g\sin\theta\), plus noise |
| `vehicle.lat_accel_mps2` | m/s² | \(v\dot\psi\), plus noise |
| `vehicle.long_accel_valid` | 0 or 1 | |
| `vehicle.lat_accel_valid` | 0 or 1 | |
| `vehicle.gear` | text | `park` or `drive` |
| `vehicle.odometer_km` | km | |
| `vehicle.latitude_deg` | deg | GPS report |
| `vehicle.longitude_deg` | deg | GPS report |
| `vehicle.gps_accuracy_m` | m | 3 m plus speed and a cold start after waking |
| `vehicle.yaw_rate_rads` | rad/s | |
| `vehicle.heading_deg` | deg | 0 is north |
| `vehicle.grade` | rise/run | ±0.12 |
| `vehicle.elevation_m` | m | |
| `vehicle.pedal_pct` | % | Accelerator demand |
| `vehicle.brake_pedal` | 0 or 1 | Friction brake or standstill hold |
| `vehicle.awake` | 0 or 1 | |
| `vehicle.power_state` | text | `asleep`, `waking`, `awake` |
| `vehicle.drive_state` | text | `unavailable`, `standby`, `enable` |
| `vehicle.occupied` | 0 or 1 | Someone in the car |
| `vehicle.range_km` | km | Energy left / rated Wh/km |

## Drive

| Signal | Unit | What it is |
|---|---|---|
| `drive.motor_rpm` | rpm | 0 in Park |
| `drive.motor_torque_nm` | N·m | Signed. Negative is regen |
| `drive.motor_power_kw` | kW | Mechanical |
| `drive.traction_power_kw` | kW | Electrical, including motor loss |
| `drive.friction_brake_n` | N | Extra braking beyond regen |

## Battery

| Signal | Unit | What it is |
|---|---|---|
| `battery.pack_voltage_v` | V | Terminals, after R0 and the RC sag |
| `battery.pack_current_a` | A | Positive discharges |
| `battery.pack_power_kw` | kW | Same sign as current |
| `battery.soc_pct` | % | |
| `battery.temp_c` | °C | Pack mean |
| `battery.temp_max_c` | °C | Mean plus half the spread |
| `battery.temp_min_c` | °C | Mean minus half the spread |
| `battery.ocv_v` | V | Open-circuit voltage |
| `battery.hv_bus_voltage_v` | V | 0 asleep, ramps while waking, pack voltage awake |
| `battery.contactors_closed` | 0 or 1 | |
| `battery.nominal_full_pack_kwh` | kWh | Usable full pack now, falls with SOH |
| `battery.beginning_of_life_kwh` | kWh | Card constant |
| `battery.nominal_energy_remaining_kwh` | kWh | \(E_{full}\times SOC/100\) |
| `battery.soh_pct` | % | \(E_{full}/E_{BOL}\times 100\) |
| `battery.loss_sei_pct` | % | Calendar SEI film |
| `battery.loss_cycle_pct` | % | Throughput / C-rate / SOC |
| `battery.loss_plate_pct` | % | Cold or fast charge plating |
| `battery.eq_full_cycles` | — | Discharge Ah / BOL Ah |
| `battery.max_discharge_kw` | kW | BMS, this step |
| `battery.max_charge_kw` | kW | BMS charge acceptance |
| `battery.max_discharge_a` | A | |
| `battery.max_charge_a` | A | |
| `battery.total_charge_kwh` | kWh | Lifetime counter |
| `battery.total_discharge_kwh` | kWh | |
| `battery.energy_to_limit_kwh` | kWh | From here to the charge limit |

## 12 V

| Signal | Unit | What it is |
|---|---|---|
| `lv.voltage_v` | V | ~12.6 asleep, ~14 with the DC-DC |
| `lv.current_a` | A | Positive leaves the 12 V battery |
| `lv.soc_pct` | % | |
| `dcdc.active` | 0 or 1 | |
| `dcdc.output_a` | A | |
| `dcdc.input_kw` | kW | Drawn from the HV pack |

## Charging

| Signal | Values | What it is |
|---|---|---|
| `charge.status` | `inactive`, `connected`, `ext_evse_test`, `evse_test_passed`, `enabled`, `standby` | Charge-port view |
| `charge.stage` | `none`, `connected`, `handshake`, `cable_check`, `precharge`, `ramp`, `charging`, `complete` | Session |
| `charge.plug` | `none`, `ac`, `fast` | |
| `charge.port_door` | `closed`, `opening`, `open`, `closing` | |
| `charge.cable_present` | 0 or 1 | |
| `charge.latch` | `released`, `engaged` | Engaged while charging or locked on a cable |
| `charge.enabled` | 0 or 1 | Setting |
| `charge.limit_pct` | % | 50–100 |
| `charge.ac_limit_a` | A | User AC cap |
| `charge.pilot_a` | A | EVSE AC max if AC is plugged |
| `charge.schedule` | `HH:MM` or `none` | |
| `charge.line_voltage_v` | V | AC |
| `charge.line_current_a` | A | AC |
| `charge.line_power_kw` | kW | AC |
| `charge.evse_dc_voltage_v` | V | DC station |
| `charge.evse_dc_current_a` | A | DC station |
| `charge.power_kw` | kW | Into the pack |
| `charge.session_kwh` | kWh | This plug-in |
| `charge.hours_to_full` | h | At the present rate |

## Climate

| Signal | Values | What it is |
|---|---|---|
| `climate.mode` | `off`, `on`, `precondition`, `keep`, `dog`, `camp` | Setting |
| `climate.power_state` | `off`, `on`, `precondition`, `overheat`, `overheat_fan` | What HVAC is doing |
| `climate.cabin_c` | °C | |
| `climate.setpoint_c` | °C | 15–28 |
| `climate.ac_running` | 0 or 1 | Compressor |
| `climate.blower_pct` | % | |
| `climate.blower_segment` | 0–11 | |
| `climate.hvac_power_kw` | kW | Compressor + PTC on HV |
| `climate.compressor_kw` | kW | |
| `climate.ptc_kw` | kW | Resistive cabin heat |
| `climate.overheat_protection` | `ac`, `fan`, `off` | Setting |
| `climate.overheat_active` | 0 or 1 | Running now |
| `climate.seat_heat_*` | 0–3 | Five seats |

## Thermal (pack)

| Signal | Unit | What it is |
|---|---|---|
| `thermal.battery_heater_kw` | kW | Pack heater |
| `thermal.battery_cooling_kw` | kW | Chiller |
| `thermal.battery_precondition` | 0 or 1 | Setting |

## Body and lights

| Signal | Values | What it is |
|---|---|---|
| `body.locked` | 0 or 1 | |
| `body.door_*` | `closed`, `opening`, `open`, `closing` | Four doors |
| `body.trunk`, `body.frunk` | same | |
| `body.any_door_open` | 0 or 1 | |
| `body.any_closure_open` | 0 or 1 | |
| `body.horn_on` | 0 or 1 | |
| `body.windows` | `closed`, `vent`, `open` | |
| `lights.mode` | `auto`, `on`, `off`, `parking` | |
| `lights.low_beam` | `on`, `off` | |
| `lights.brake` | `on`, `off` | Pedal or strong regen |

## Environment

| Signal | Unit | What it is |
|---|---|---|
| `environment.ambient_c` | °C | True air |
| `environment.ambient_filtered_c` | °C | Sensor, lagged |
| `environment.solar_w_m2` | W/m² | On the car |
| `environment.sun_elevation_deg` | deg | |
| `environment.cloud_pct` | % | |
| `environment.weather_source` | text | `live`, `curve`, `fixed` |

## Requests

Momentary requests sit at idle unless a remote node writes a different value. Settings mirror the car.

| Request | Values | What another node's value does |
|---|---|---|
| `request.lock` | `idle`, `lock`, `unlock`, `remote_lock`, `remote_unlock` | Lock or unlock |
| `request.horn` | 0 or 1 | Horn while 1 |
| `request.closure` | `idle`, `rear_trunk_move`, `front_trunk_move` | Trunk toggles; frunk opens. Not while driving |
| `request.charge_enable` | 0 or 1 | |
| `request.charge_port_open` | 0 or 1 | |
| `request.charge_port_close` | 0 or 1 | Refused if the cable is in |
| `request.charge_limit_pct` | 50–100 | |
| `request.ac_current_limit_a` | A | AC cap |
| `request.climate_power` | `off`, `on`, `precondition` | |
| `request.climate_setpoint_c` | 15–28 | |
| `request.climate_keep` | `off`, `keep`, `dog`, `camp` | Parked climate modes |
| `request.battery_precondition` | 0 or 1 | |
| `request.seat_heat_*` | 0–3 | |
| `request.lights` | `auto`, `on`, `off`, `parking` | |
| `request.windows` | `idle`, `closed`, `vent`, `open` | |
