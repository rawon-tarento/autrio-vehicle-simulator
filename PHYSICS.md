# Vehicle physics

How the simulator turns time into every plant signal. The car is DBC-agnostic: these names live on the plant. `alias.json` maps whichever of them a given DBC actually has. A sensor that a DBC omits still exists here and still drives the rest of the car.

The clock is 10 Hz (`dt = 0.1 s`) in the live process. Tests may use a larger `dt`. Equations below are one step.

Card numbers are the only per-vehicle inputs. Defaults match a ~75 kWh, ~1800 kg car. See `vehicle_simulator/card.py`.

---

## 1. Orthogonal states

The car is not one mode. Five dimensions run in parallel. `PHYSICS.md` §8 lists which combinations are legal.

| Dimension | Values | Held in |
|---|---|---|
| Power | `asleep`, `waking`, `awake` | `Vehicle._power` |
| Motion | parked (`P`) or driving (`D`) | `_mode`, `sample.gear` |
| Charging | `none` → `connected` → handshake → `ramp` → `charging` → `complete` | `ChargeSession` |
| Climate | `off`, `on`, `precondition`, `keep`, `dog`, `camp` + overheat protection | `Body.climate_mode`, `_overheat_active` |
| Access | locked/unlocked, each closure, occupancy | `Body`, `occupied` |

Battery thermal (passive / heating / cooling) is a consequence of pack temperature and charging, not a driver-selected mode.

---

## 2. Environment

`vehicle_simulator/environment.py`

### Sun elevation (degrees)

UTC hour `H`, day-of-year `n`, latitude `φ`, longitude `λ`:

\[
\delta = 23.44^\circ \sin\bigl(2\pi (284+n)/365\bigr)
\]
\[
EOT = 9.87\sin 2B - 7.53\cos B - 1.5\sin B,\quad B = 2\pi(n-81)/364
\]
\[
t_{solar} = H + \lambda/15 + EOT/60
\]
\[
\sin e = \sin\phi\sin\delta + \cos\phi\cos\delta\cos\bigl(15^\circ(t_{solar}-12)\bigr)
\]

`environment.sun_elevation_deg` is \(e\). Night for auto lights: \(e < 0\) or irradiance \(< 50\ \mathrm{W/m^2}\).

### Clear-sky irradiance and clouds

\[
G_{clear} = 1000 \cdot (\sin e)^{1.15}\ \mathrm{W/m^2}\quad(e>0),\quad 0\ \mathrm{otherwise}
\]
\[
G = G_{clear}\,(1 - 0.75\,c^{3.4})
\]

`c` is cloud fraction 0–1 (`card.cloud_cover` on the curve, or Open-Meteo).

### Air temperature

- **`live`:** Open-Meteo `temperature_2m` and `shortwave_radiation` at the car's pin, refreshed every `weather_refresh_s` or after `weather_refresh_km`. Falls back to the curve while waiting or if the fetch fails.
- **`curve`:** \(T_{amb} = T_{mean} + A\cos\bigl(2\pi(t_{solar}-15)/24\bigr)\) (`ambient_c`, `ambient_swing_c`). Warmest mid-afternoon solar time.
- **`fixed`:** `ambient_c` from the card.

The ambient *sensor* lags the true air:

\[
T_{sens} \leftarrow T_{sens} + (T_{amb}-T_{sens})\,(1-e^{-dt/\tau_{sens}})
\]

`environment.ambient_c` is true air. `environment.ambient_filtered_c` is the sensor.

---

## 3. High-voltage pack

`vehicle_simulator/pack.py`

### Open-circuit voltage

A seven-point cell curve, scaled so 50 % SOC sits on `nominal_voltage_v`:

| SOC % | 0 | 10 | 20 | 50 | 80 | 90 | 100 |
|---|---|---|---|---|---|---|---|
| Cell V | 3.00 | 3.45 | 3.55 | 3.70 | 3.95 | 4.10 | 4.18 |

\[
OCV(SOC) = V_{nom}\cdot V_{cell}(SOC)/3.70
\]

`battery.ocv_v`. Series count `cells_in_series = round(V_nom / 3.7)`.

### Resistance

Total DC resistance grows with age and with cold (Arrhenius):

\[
R(T,SOH) = R_{int}(1 + 0.5(1-SOH))\,\exp\Bigl[k_R\bigl(\tfrac{1}{T+273.15}-\tfrac{1}{298.15}\bigr)\Bigr]
\]

Split into a fast ohmic part and one RC branch:

\[
R_0 = r_0 R,\quad R_1 = (1-r_0)R,\quad \tau = R_1 C_1 = \texttt{rc\_tau\_s}
\]

### Terminals

Positive current discharges.

\[
V = OCV - I R_0 - V_{rc}
\]
\[
V_{rc} \leftarrow V_{rc}\,e^{-dt/\tau} + I R_1 (1-e^{-dt/\tau})
\]

Power at the terminals \(P = V I\). Given a requested power, current is the physical root of \(P = (OCV-V_{rc})I - R_0 I^2\):

\[
I = \frac{V_s - \sqrt{V_s^2 - 4 R_0 P}}{2 R_0},\quad V_s = OCV - V_{rc}
\]

\(P\) is capped at the maximum power \(V_s^2/(4R_0)\) so the discriminant stays non-negative. \(I\) is then clamped to the BMS limits.

SOC (coulomb counting; charge current is reduced by `coulombic_eff_charge`):

\[
SOC \leftarrow SOC - \frac{I\,dt}{3600\cdot Ah\cdot SOH}\cdot 100
\]

\[
SOH = E_{full}/E_{BOL},\quad E_{rem} = E_{full}\cdot SOH\cdot SOC/100
\]

### BMS limits

Discharge current: voltage headroom down to `cell_v_min`, times a temperature factor (0.4 at −20 °C, 1.0 from 15–50 °C, 0.5 at 60 °C), capped at `max_discharge_a`.

Charge acceptance: a SOC taper (full to ~20–30 %, ~0.5 at 50 %, ~0.3 at 80 %, ~0.2 at 90 %, 0 at 100 %) times a temperature taper (almost none below 0 °C, full at 25 °C, derated above 45 °C), then a constant-voltage cap \((V_{max} - OCV + V_{rc})/R_0\).

`battery.max_discharge_kw`, `battery.max_charge_kw`, `battery.max_discharge_a`, `battery.max_charge_a`.

### Heat and cell spread

\[
Q_{gen} = I^2 R_0 + V_{rc}^2 / R_1
\]
\[
C_{th}\frac{dT}{dt} = Q_{gen} - k_{loss}(T-T_{amb}) - Q_{cool} + Q_{heat}
\]

Spread relaxes toward `spread_base + spread_k |I|` with time constant `spread_tau_s`. `battery.temp_max_c` / `temp_min_c` sit ±spread/2 around `battery.temp_c`.

Contactors open (`asleep`): \(I=0\), \(V_{rc}\) decays, temperature drifts to ambient.

---

## 4. 12 V system

`vehicle_simulator/lv.py`

Asleep, the 12 V battery alone feeds `sleep_w`. Rest voltage 11.9–12.8 V by SOC, minus \(I R_{12}\) with \(R_{12}=0.02\ \Omega\).

Awake, the DC-DC holds `lv_bus_v` (14 V), supplies every 12 V load, and tops the battery at up to `lv_charge_a`:

\[
P_{DCDC,in} = (P_{loads}/V_{bus} + I_{charge})\,V_{bus}/\eta_{DCDC}
\]

That input is an HV load. A 12 V SOC below `lv_topup_below_pct` wakes the car; it sleeps again once `lv_topup_to_pct` is reached if nothing else is busy.

`lv.voltage_v`, `lv.current_a`, `lv.soc_pct`, `dcdc.*`.

12 V loads while awake: `elec_kw` + HVAC blower and pump + seat heaters (`seat_heat_w_per_level` × sum of levels) + `lights_w` if the low beam is on (or 10 W for parking lights).

---

## 5. Cabin and HVAC

`vehicle_simulator/thermal.py`

### Cabin heat balance

\[
C_{cab}\frac{dT_{cab}}{dt} = -UA(T_{cab}-T_{amb}) + Q_{solar} + Q_{occ} + Q_{HVAC}
\]

- \(C_{cab} =\) `cabin_heat_capacity_j_per_k`
- \(UA = UA_0 + k_v v_{kph} + UA_{window} + n_{doors}UA_{door} + UA_{trunk}\)
- \(Q_{solar} = A_{gain}\,G\)
- \(Q_{occ} = 100\ \mathrm{W}\) if occupied, dog, or camp; else 0
- \(Q_{HVAC}\) is signed: positive heats the cabin

`climate.cabin_c`.

### What HVAC asks for

Hold against losses, then close the gap over `hvac_pull_tau_s`, clamped to `hvac_cool_max_w` / `hvac_heat_max_w`.

Cooling COP falls with ambient (about 4 at 20 °C, 1.8 in extreme heat). Heat-pump COP rises from 1.2 at −15 °C to 2.8 in mild weather; below −15 °C or with no heat pump, the PTC heater (COP 1) takes over.

One compressor of `compressor_max_w` is shared. During DC charging, or if the pack is above 45 °C, the battery chiller is served first; cabin cooling takes what is left.

Pack heater `pack_heater_w` is independent (resistive). The coolant pump runs whenever the pack is being heated or chilled.

Electrical:

\[
P_{comp} = Q_{cool}/COP_{cool} + Q_{hp}/COP_{hp},\quad P_{PTC} = Q_{PTC},\quad P_{heater} = Q_{pack\ heat}
\]

Blower 30–300 W from how hard the cabin is pulling. `climate.hvac_power_kw` is compressor + PTC (HV). Blower and pump are 12 V.

`climate.ac_running` is 1 when the compressor is on. `climate.blower_pct` / `blower_segment` (0–11) follow blower demand.

### Climate modes

| Mode | Cabin target | Sleep | Lock | Notes |
|---|---|---|---|---|
| `off` | none | allowed | — | Cabin drifts |
| `on` | setpoint | blocked | walking away turns it off | |
| `precondition` | setpoint | blocked | stays on | Becomes `on` if someone sits or a drive starts. Stops after `cabin_precondition_timeout_s` (30 min) |
| `keep` | setpoint | blocked | stays on | Needs SOC ≥ `keep_climate_min_soc_pct` |
| `dog` | setpoint | blocked | stays on | Same SOC floor. Occupant load stays 1 |
| `camp` | setpoint | blocked | driver may stay | Same SOC floor |
| Overheat, A/C | `overheat_on_c − 3` | wakes, then sleeps when done | locked, empty, climate off, first `overheat_hours` | Hysteresis 3 °C, SOC floor |
| Overheat, fan | none, extra UA | same | same | Fan-only mixing with outside air |

`climate.mode` is the setting. `climate.power_state` is what the HVAC reports: `off`, `on`, `precondition`, `overheat`, `overheat_fan`. Keep/dog/camp report `on`.

Pack heating/cooling request (`battery_demand`):

- DC charging: heat toward `dc_heat_target_c` if cold; chill above `dc_cool_on_c`
- AC charging: heat if below `ac_heat_below_c`
- Battery precondition: heat toward `precondition_target_c` for up to `precondition_timeout_s`
- Otherwise: chill above `cool_on_c`

`thermal.battery_heater_kw`, `thermal.battery_cooling_kw`, `thermal.battery_precondition`.

---

## 6. Charging

`vehicle_simulator/charging.py`

### Stages and `charge.status`

| Stage | Status on the bus | Current |
|---|---|---|
| `none` | `inactive` | 0 |
| `connected` | `connected` | 0 |
| `handshake` (AC, `ac_handshake_s`) | `connected` | 0 |
| `cable_check` (DC, `dc_cable_check_s`) | `ext_evse_test` | 0 |
| `precharge` (DC, `dc_precharge_s`) | `evse_test_passed` | 0 |
| `ramp` | `enabled` | rising 0 → 1 over `ac_ramp_s` / `dc_ramp_s` |
| `charging` | `enabled` | BMS-limited |
| `complete` | `standby` | 0 into the pack; AC still covers hotel loads |

A session starts when the operator plugs in (`ac` or `fast`). It advances only while: charge enable is on, the schedule allows, contactors are closed, and the car is in Park. Disable, unplug, or hitting the SOC limit sends it to `connected` or `complete`. AC resumes from `complete` when SOC falls below the top-up line (`charge_limit − (card.charge_limit − topup_below)`).

Schedule `HH:MM` allows charging for six hours after that local time. Empty means “now”.

### Power split

The station puts at most `available_w` on the HV bus.

AC:

\[
I_{line} = \min(I_{EVSE},\, I_{user}),\quad P_{line} = \min(I_{line} V_{line} n_{ph},\, P_{OBC}/\eta_{OBC})
\]
\[
P_{bus} = \eta_{OBC}\,P_{line}\cdot ramp
\]

DC: \(\min(P_{fast}, I_{EVSE,dc} V_{pack})\cdot ramp\).

The car's HV loads (traction is zero in Park, plus HVAC, pack heater, DC-DC) are served *from the station first*. What remains, up to `battery.max_charge_kw`, goes into the pack. A weak charger with climate on can leave the pack current near zero or even slightly discharging.

`charge.line_*`, `charge.evse_dc_*`, `charge.power_kw`, `charge.session_kwh`, `charge.hours_to_full` (\(E_{to\ limit}/P_{into\ pack}\)).

### Latch and port

`charge.latch` is `engaged` while charging or while locked with a cable in; otherwise `released`. The port door cannot close on a cable. It closes on its own `port_close_after_unplug_s` after unplug, or `port_close_idle_s` if opened and never used.

---

## 7. Driving

`vehicle_simulator/powertrain.py`, `Vehicle._drive_step`

A drive is `depart` (1 s: brake, shift to D, contactors already closed) → `run` → `arrive` (1 s in Park).

### Road load

\[
F_{resist} = mg C_{rr}/\sqrt{1+\gamma^2} + mg\gamma/\sqrt{1+\gamma^2} + \tfrac12\rho C_d A v^2
\]

\(\gamma\) is grade (rise/run). Effective mass \(m_{eff} = m\cdot\) `rot_mass_factor`.

### Driver request → motor and friction brake

The driver asks for an acceleration (road speed, curve look-ahead, stop distance, jerk-limited). The motor gives what torque, inverter power, and the BMS allow. Lift-off regen is capped at `liftoff_decel_mps2` and fades to zero by `regen_fade_kph` (8 km/h). Anything beyond regen is the friction brake.

\[
P_{el} = F_{motor} v + P_{iron}(|\omega|) + P_{copper}(\tau^2) + P_{fixed}
\]

`vehicle.pedal_pct` is the cause: 0 on the brake, `pedal_coast_pct` at lift-off, 100 % at max drive force.

`vehicle.brake_pedal` is 1 when held at standstill or the friction brake is doing real work. The brake light also comes on for regen stronger than `brake_light_decel_mps2`.

Auto-lock at `auto_lock_kph`. Unlock on Park if `unlock_on_park`. Frunk/trunk/port refuse to open while moving. Opening the driver door at a standstill in D shifts to Park.

Displayed speed: \(v_{ui} = v(1+\varepsilon)\) with \(\varepsilon=\) `speedo_overread` (2 %). `vehicle.speed_kph` is true; `vehicle.display_speed_kph` is the cluster.

`drive.motor_rpm`, `drive.motor_torque_nm`, `drive.motor_power_kw`, `drive.traction_power_kw`, `drive.friction_brake_n`.

### Inertial sensors

An accelerometer measures specific force, not coordinate acceleration:

\[
a_{long,report} = a + g\sin(\arctan\gamma) + \mathcal{N}(0,0.02)
\]
\[
a_{lat,report} = v\dot\psi + \mathcal{N}(0,0.02)
\]

Yaw rate is \(\dot\psi\) plus 0.002 rad/s noise. Noise is off in tests (`sensor_noise`); the live process turns it on.

GPS: true pin plus a slow random walk the size of the accuracy figure. Accuracy is 3 m plus 0.04 km/h, plus a 27 m cold-start term that decays over `gps_fix_s` after waking. `vehicle.latitude_deg` / `longitude_deg` in `signals()` are the *reported* fix. The plant's true pin stays on `sample` and is what the next drive starts from.

---

## 8. Power state and sleep

Waking takes `precharge_s` (HV bus voltage \(V_{ocv}(1-e^{-5t/T})\)). `battery.contactors_closed` is 1 only in `awake`. `battery.hv_bus_voltage_v` is 0 asleep, ramping while waking, pack voltage once closed.

`vehicle.drive_state`: `unavailable` unless awake; `enable` in D; otherwise `standby`.

The car stays awake while any of: waking, driving, occupied, climate not off, overheat protection, battery precondition, pack heater/chiller, an active charge session, 12 V top-up, horn, a door open or moving, walk-away lock in progress.

A wake that was not for the user (charge, 12 V, overheat) sleeps again as soon as that job finishes. A user wake waits `auto_sleep_after_s` (15 min) of quiet.

Walk-away: `leave` opens the driver door, shuts it after 2 s, then locks after `walk_away_lock_s`. Locking turns climate `on` off (keep/dog/camp stay) and zeros seat heaters unless camp.

---

## 9. How every plant signal is produced

Signals the DBC does not map still exist. They still affect the signals it does map.

### Vehicle / motion

| Signal | Source |
|---|---|
| `vehicle.speed_kph` | \(\int a\,dt\) along the route; 0 in Park |
| `vehicle.display_speed_kph` | speed × (1 + overread) |
| `vehicle.long_accel_mps2` | specific force + noise |
| `vehicle.lat_accel_mps2` | \(v\dot\psi\) + noise |
| `vehicle.long_accel_valid`, `lat_accel_valid` | 1 |
| `vehicle.yaw_rate_rads` | heading rate + noise |
| `vehicle.heading_deg` | path bearing |
| `vehicle.grade` | road lookup, ±0.12 |
| `vehicle.elevation_m` | Open-Meteo elevation on the route |
| `vehicle.gear` | `park` or `drive` |
| `vehicle.odometer_km` | \(\int v\,dt\) |
| `vehicle.latitude_deg`, `longitude_deg` | GPS report (true pin is internal) |
| `vehicle.gps_accuracy_m` | 3 m + speed term + cold start |
| `vehicle.pedal_pct` | driver demand from powertrain |
| `vehicle.brake_pedal` | standstill hold or friction brake |
| `vehicle.awake` | power ≠ asleep |
| `vehicle.power_state` | `asleep` / `waking` / `awake` |
| `vehicle.drive_state` | `unavailable` / `standby` / `enable` |
| `vehicle.occupied` | 1 after `enter` or a drive, 0 after `leave` / walk-away lock |
| `vehicle.range_km` | \(E_{rem}/(rated\_wh\_per\_km/1000)\) |

### Battery / HV / LV

| Signal | Source |
|---|---|
| `battery.pack_voltage_v` | \(OCV - IR_0 - V_{rc}\) + noise |
| `battery.pack_current_a` | solved from power, + noise. Discharge > 0 |
| `battery.pack_power_kw` | \(VI/1000\) |
| `battery.soc_pct` | coulomb count |
| `battery.temp_c`, `temp_max_c`, `temp_min_c` | heat balance + spread |
| `battery.ocv_v` | OCV curve |
| `battery.hv_bus_voltage_v` | 0 / precharge / pack |
| `battery.contactors_closed` | awake |
| `battery.nominal_*`, `soh_pct`, energy remaining | card and SOC |
| `battery.max_*` | BMS limits this step |
| `battery.total_charge_kwh`, `total_discharge_kwh` | counters |
| `battery.energy_to_limit_kwh` | energy from here to the charge limit |
| `lv.*`, `dcdc.*` | 12 V model |
| `thermal.battery_*` | pack heater / chiller / precondition flag |

### Charging

| Signal | Source |
|---|---|
| `charge.status` | stage table in §6 |
| `charge.stage` | session stage |
| `charge.plug` | `none` / `ac` / `fast` |
| `charge.port_door` | closure |
| `charge.cable_present` | plug ≠ none |
| `charge.latch` | engaged while charging or locked-on-cable |
| `charge.enabled` | setting |
| `charge.limit_pct` | setting 50–100 |
| `charge.ac_limit_a` | user AC cap |
| `charge.pilot_a` | EVSE AC max if AC plugged |
| `charge.schedule` | `HH:MM` or `none` |
| `charge.line_*` | AC side |
| `charge.evse_dc_*` | DC side |
| `charge.power_kw` | power into the pack while plugged |
| `charge.session_kwh`, `hours_to_full` | this session |

### Climate, body, lights, environment

| Signal | Source |
|---|---|
| `climate.mode` | setting |
| `climate.power_state` | reported HVAC state |
| `climate.cabin_c`, `setpoint_c` | heat balance / setting |
| `climate.ac_running`, `blower_*`, `hvac_power_kw`, `compressor_kw`, `ptc_kw` | HVAC this step |
| `climate.overheat_protection`, `overheat_active` | setting / running |
| `climate.seat_heat_*` | 0–3 |
| `body.*` | closures, lock, horn, windows |
| `lights.mode` | switch |
| `lights.low_beam` | on, or auto while driving in the dark |
| `lights.brake` | brake pedal or strong regen |
| `environment.*` | §2 |
| `drive.*` | motor this step; 0 when parked |

### Requests (what the car itself is sending)

Momentary (`lock`, `horn`, `closure`, port open/close, windows) sit at idle unless a remote node writes a different value. Settings (`charge_enable`, limit, AC amps, climate, keep, seats, lights, battery precondition, setpoint) mirror the current setting. Applying a remote value runs the same operator path.

---

## 10. Dependencies (what affects what)

A dash means no effect. That is part of the definition.

| ↓ affects → | Power | Motion | Charging | Climate | Pack E | Pack T | Cabin T | Access | Lights |
|---|---|---|---|---|---|---|---|---|---|
| **Power** | | Must be awake to move | Handshake needs contactors | HVAC needs contactors | Asleep: sleep drain only | Drifts to ambient | Drifts | Requests wake it | Off asleep |
| **Motion** | Driving blocks sleep | | Plug blocked; plug blocks D | Speed raises UA | Traction / regen | \(I^2R\) | Speed raises leak | Auto-lock; closures blocked | Auto beam only in D |
| **Charging** | Active session blocks sleep; top-up wakes | Gear held in P | | — | Station first, then pack | DC heating / chiller | — | Latch follows lock | — |
| **Climate** | Blocks sleep; remote wakes | — | Takes station power | | HVAC + PTC + heater | Shares compressor | Drives it | Lock turns `on` off | — |
| **Pack T** | Heater/chiller keep awake | Discharge/regen limits | Charge acceptance | Cabin cooling yields to DC | Limits | — | — | — | — |
| **Access** | Unlock/door wakes; open door blocks sleep | Driver door at rest → Park | — | Open door/window raises UA | — | — | Leak | | — |
| **Ambient/sun** | — | — | Cold limits DC | Sets HVAC load | Indirect | Drift | Drift + solar | — | Night → auto beam |
| **SOC** | — | Power limit | Limit → complete | Keep/dog/camp/overheat floors | OCV | — | — | — | — |
| **12 V** | Low SOC wakes | — | — | — | DC-DC HV load | — | — | — | — |

Confirmed independences:

- Climate never changes gear, GPS, IMU, odometer, or charge *state*.
- Charging never changes climate *mode*, doors, lights, horn, or position.
- Seat heaters are independent of climate mode (except lock zeros them).
- Horn and lights (except their 12 V watts) affect nothing else.

---

## 11. Legal combinations

Invalid requests are refused or resolved with no message. The frames just don't change.

**Always illegal**

- Asleep while driving
- Asleep while handshake/ramp/charging, or while climate is not off, or with a door open
- Driving while plugged in
- AC and DC at once
- Two climate modes at once
- Port door closed on a cable
- Keep/dog/camp below the SOC floor (the mode turns off)
- Frunk/trunk/port opening while moving

**Typical legal rows**

| Power | Motion | Charge | Climate | Situation |
|---|---|---|---|---|
| asleep | P | none | off | Overnight |
| asleep | P | complete | off | Finished, sleeping on the cable |
| asleep | P | connected, scheduled | off | Waiting for the timer |
| waking | P | any parked | off | 1.5 s precharge |
| awake | P | none | off / on / precondition / keep / dog / camp / overheat | Parked |
| awake | P | connected | off or on | Plugged, not charging |
| awake | P | ramp / charging AC | off, on, keep, precondition | Home charging |
| awake | P | ramp / charging DC | off or on | DC charger (pack cooling first) |
| awake | P | complete | precondition / keep | Pre-departure on shore power |
| awake | D | none | off or on | Driving |

---

## 12. Files

| File | What it integrates |
|---|---|
| `environment.py` | Air, sun, live weather |
| `pack.py` | OCV, R0+RC, SOC, BMS limits, pack heat |
| `lv.py` | 12 V battery and DC-DC |
| `thermal.py` | Cabin UA, HVAC, shared compressor, pack heater/chiller |
| `charging.py` | Session stages and station power |
| `powertrain.py` | Road load, motor, regen, friction brake, pedal |
| `sensors.py` | IMU gravity, noise, GPS fix, speedometer overread |
| `body.py` | Closures, lock, climate mode, plug |
| `car.py` | Interlocks, sleep, the step that calls everything |
| `card.py` | The numbers |
| `signals.md` | Plant names for the alias |
| `alias.json` | This DBC's subset |

---

## 13. One clock tick

`Vehicle.step(dt)` always runs in this order. A signal listed later in the table cannot affect an earlier one in the same tick.

1. Environment (air, sun, live weather fetch if due).
2. If asleep, wake for 12 V top-up, charge top-up, or overheat.
3. Finish HV precharge (`waking` → `awake`).
4. Move closures (doors, trunk, frunk, port).
5. Body rules: walk-away lock, auto-close the port, climate timeouts, overheat hysteresis, keep-mode SOC floor.
6. Drive along the route, or sit in Park.
7. HVAC and pack heater/chiller (needs contactors). Cabin still leaks while asleep.
8. 12 V battery / DC-DC.
9. Charge session stage machine.
10. Pack: draw `P_HV − P_shore`. Station power is applied here, not as a separate “charge current”.
11. Session records what the station actually delivered (line or DC EVSE figures).
12. IMU, pack noise, GPS fix.
13. Copy plant state onto `sample` / `signals()`.
14. Auto-sleep if nothing is busy.

Operator commands and bus requests run *between* ticks. They change settings; the next tick computes the signals.

---

## 14. Card numbers used in the equations

Every symbol above is a field on `VehicleCard` (`vehicle_simulator/card.py`). Defaults match a ~75 kWh, ~1800 kg car.

| Card field | Used for |
|---|---|
| `pack_kwh`, `nominal_voltage_v`, `capacity_ah` | Pack energy and Ah |
| `nominal_full_pack_kwh`, `beginning_of_life_kwh` | SOH |
| `internal_resistance_ohm`, `r0_fraction`, `rc_tau_s`, `r_activation_k` | R0 + RC |
| `cell_v_min`, `cell_v_max`, `max_discharge_a`, `dc_max_kw` | BMS limits |
| `coulombic_eff_charge` | Charge Ah |
| `heat_capacity_j_per_k`, `heat_loss_w_per_k`, `spread_*` | Pack temperature |
| `chiller_*`, `pack_heater_w`, `dc_heat_target_c`, `ac_heat_below_c`, `cool_on_c`, `dc_cool_on_c` | Pack thermal demand |
| `precondition_target_c`, `precondition_timeout_s` | Battery precondition |
| `mass_kg`, `cd`, `frontal_area_m2`, `rolling_coeff`, `rot_mass_factor` | Road load |
| `max_drive_power_kw`, `max_motor_torque_nm`, `gear_ratio`, `wheel_radius_m`, `motor_*_loss_*` | Motor |
| `max_regen_kw`, `regen_fade_kph`, `liftoff_decel_mps2`, `pedal_coast_pct`, `brake_light_decel_mps2` | Regen and pedals |
| `charge_power_kw`, `fast_charge_power_kw`, `evse_*`, `ac_line_voltage_v`, `ac_phases`, `obc_efficiency` | Station power |
| `ac_handshake_s`, `dc_cable_check_s`, `dc_precharge_s`, `ac_ramp_s`, `dc_ramp_s` | Charge stages |
| `charge_limit_pct`, `topup_below_pct` | Charge complete / top-up |
| `sleep_w`, `elec_kw`, `lv_*`, `dcdc_efficiency` | 12 V |
| `cabin_*`, `hvac_*`, `compressor_max_w`, `heat_pump`, `blower_max_w`, `solar_gain_m2`, `occupant_w` | Cabin |
| `keep_climate_min_soc_pct`, `cabin_precondition_timeout_s`, `overheat_*` | Parked climate |
| `weather`, `ambient_c`, `ambient_swing_c`, `cloud_cover`, `ambient_sensor_tau_s` | Air and sun |
| `speedo_overread`, `sensor_noise`, `gps_fix_s` | Sensors |
| `auto_sleep_after_s`, `auto_lock_kph`, `unlock_on_park`, `walk_away_lock_s`, `precharge_s` | Power and access |
| `rated_wh_per_km` | Displayed range |
