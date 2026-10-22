# Vehicle simulator physics scorecard

**Overall: B · 88.9% · 80 / 90 points** across 27 checks.

Head-to-head with `autrio-car-sim`: [COMPARISON.md](COMPARISON.md).

Pass 20 · fair 4 · weak 3 · fail 0.

## How this was scored

The plant (`drive_step` + `Pack.step` + `ChargeSession`) was run as a treadmill — no map, no HVAC, grade 0, 25 °C, start SOC 80%. A speed-tracking driver follows EPA cycle traces at 0.1 s. Hotel load is the card 12 V accessory (`elec_kw` through DC-DC).

Relative error `(sim − ref) / ref`: ≤10% = 3 pass, ≤20% = 2 fair, ≤40% = 1 weak, else 0 fail. Envelope checks score 3 if inside the published band, otherwise relative to the nearest edge. EPA Vehicle DC is weighted 2× because it is the lab gold standard; everything else is weight 1.

Primary energy reference is **EPA 2020 Model 3 Long Range AWD Vehicle DC** (not the 5-cycle window-sticker 28 kWh/100 mi). FASTSim 3.1.0 `2022 Tesla Model 3 RWD` is a same-class second model, not the same car (RWD, 1752 kg vs our 1800 kg). ABRP BT37 Supercharger aggregate is the DC time-to-energy reference.

## Cycle energy

| Cycle | Our Wh/mi | EPA Vehicle DC | FASTSim elec Wh/mi | Speed RMSE mph |
| --- | --- | --- | --- | --- |
| UDDS | 167.3 | 169.4 | 146.7 | 0.35 |
| HWFET | 190.3 | 180.6 | 157.5 | 0.17 |
| US06 | 243.2 | — | 214.5 | 0.73 |
| 2-cycle 0.55/0.45 | 177.7 | 174.4 | — | — |

## Checks

| Check | Sim | Reference | Error | Score | Band |
| --- | --- | --- | --- | --- | --- |
| UDDS Wh/mi vs EPA Vehicle DC | 167.32 | 169.39 | -1.2% | 3/3 ×2 | pass |
| HWFET Wh/mi vs EPA Vehicle DC | 190.3 | 180.59 | +5.4% | 3/3 ×2 | pass |
| 2-cycle combined vs EPA | 177.661 | 174.43 | +1.9% | 3/3 ×2 | pass |
| UDDS Wh/mi vs FASTSim M3 RWD | 167.32 | 146.68 | +14.1% | 2/3 | fair |
| HWFET Wh/mi vs FASTSim M3 RWD | 190.3 | 157.47 | +20.9% | 1/3 | weak |
| US06 Wh/mi vs FASTSim M3 RWD | 243.23 | 214.53 | +13.4% | 2/3 | fair |
| UDDS speed RMSE | 0.346 | 1.5 | — | 3/3 | pass |
| HWFET speed RMSE | 0.166 | 1.5 | — | 3/3 | pass |
| US06 speed RMSE | 0.728 | 1.5 | — | 2/3 | fair |
| Road load 20 mph vs EPA ABC | 236.386 | 201.629 | +17.2% | 2/3 | fair |
| Road load 50 mph vs EPA ABC | 364.957 | 346.005 | +5.5% | 3/3 | pass |
| Road load 70 mph vs EPA ABC | 511.896 | 509.068 | +0.6% | 3/3 | pass |
| DC 10→50 kWh minutes vs ABRP | 22.71 | 23 | -1.3% | 3/3 | pass |
| DC power at 20% SOC | 149.68 | 140–170 | +0.0% | 3/3 | pass |
| DC power at 50% SOC | 91.35 | 70–110 | +0.0% | 3/3 | pass |
| DC power at 80% SOC | 50.4 | 30–55 | +0.0% | 3/3 | pass |
| AC into pack vs line×OBC | 6452.05 | 6624 | -2.6% | 3/3 | pass |
| Motor rpm at 100 kph vs Model 3 18" | 6820.93 | 7042.25 | -3.1% | 3/3 | pass |
| OCV at 50% vs nominal | 350 | 350 | +0.0% | 3/3 | pass |
| OCV at 20% vs NMC 3.55 V/cell | 335.811 | 335.811 | +0.0% | 3/3 | pass |
| OCV at 80% vs NMC 3.95 V/cell | 373.649 | 373.649 | +0.0% | 3/3 | pass |
| Pack R at 25 °C vs 35–90 mΩ | 0.07 | 0.035–0.09 | +0.0% | 3/3 | pass |
| 200 A instantaneous sag | 9.8 | 5–15 | +0.0% | 3/3 | pass |
| Rated Wh/km vs fleet p50 | 150 | 155 | -3.2% | 3/3 | pass |
| HWFET Wh/km vs fleet p50 | 118.25 | 155 | -23.7% | 1/3 | weak |
| Sleep load vs 10–50 W | 15 | 10–50 | +0.0% | 3/3 | pass |
| One-pedal regen vs ~0.2 g | 1.2 | 1.96 | -38.8% | 1/3 | weak |

## Category totals

| Category | Points | Pct | Grade |
| --- | --- | --- | --- |
| Cycle energy (EPA) | 18 / 18 | 100% | A |
| Cycle energy (FASTSim) | 5 / 9 | 56% | F |
| Speed tracking | 8 / 9 | 89% | B |
| Road load | 8 / 9 | 89% | B |
| DC charge | 12 / 12 | 100% | A |
| AC / pack / driveline | 21 / 21 | 100% | A |
| Usage / regen / sleep | 8 / 12 | 67% | D |

## DC charge curve (10 → 50 kWh remaining)

Time: **22.7 min** vs ABRP 23 min. Peak into pack **150 kW**. Samples at 20 / 50 / 80% SOC: 149.68 / 91.35 / 50.4 kW.

Handshake, cable-check, and ramp were skipped so this is energy-flow time, matching how ABRP quotes 10→50 kWh. Power-vs-SOC samples come from a separate 10→90% run so 80% is a real 80%, not the 10→50 kWh endpoint.

## Findings

- **Keep the cycle physics.** UDDS −1.2% and combined +1.9% vs EPA Vehicle DC is inside measurement noise for a generic 75 kWh / 1800 kg card. Road load at 50 and 70 mph is within 6% of the EPA ABC target.
- **FASTSim is not the gold standard here.** NREL's 2022 Model 3 RWD sits ~13% under EPA city DC on the same UDDS trace. Our plant matching EPA and beating FASTSim energy is the expected direction (we are heavier AWD-class, they are RWD 1752 kg).
- **DC taper** is `_ACCEPT_SOC_FACTOR` in `pack.py` (peak through ~30%, then roll-off). Score the 20 / 50 / 80% samples and ABRP 10→50 kWh time against that curve.
- **Low-speed Crr is a bit high** (+17% force at 20 mph). Constant `rolling_coeff=0.012` does not fall with speed the way EPA A+Bv does.
- **One-pedal is gentle** (1.2 vs ~1.96 m/s²). Fleet HWFET-vs-155 Wh/km is not a physics fail — HVAC-off lab hwy should undercut mixed real-world use.

## What this does and does not prove

- **Does:** road-load vs EPA coastdown ABC, cycle DC energy vs lab Vehicle DC, BMS taper vs published Supercharger envelope, kinematics vs Model 3 final drive.
- **Does not:** HVAC, thermal cabin, GPS/map, 12 V sleep drain of the HV pack, chassis-dyno certification, or a bit-exact D3 motor-inverter overlay (we do not have the D3 CSV in-tree).
- Card is a **generic 75 kWh / 1800 kg / 150 kW** stand-in, not a VIN-matched Model 3. Scores near EPA are evidence the *equations* are in the right band, not that we cloned Tesla firmware.

FASTSim 3.1.0 · card `lab/vehicle-simulator/examples/ev_card.json` · 75 kWh / 1800 kg / Cd 0.23 / Crr 0.012.
