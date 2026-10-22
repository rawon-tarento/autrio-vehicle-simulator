# Physics comparison: this plant vs autrio-car-sim

Shareable report for the colleague sim (sources + links): `lab/comparisons/autrio-car-sim-physics-report.md`.

Same treadmill, same references, same scoring rule as `BENCHMARK.md`.
Colleague plant is `timeline_gen/battery.py` on a prescribed EPA speed trace at 1 Hz.
Clone: `lab/comparisons/autrio-car-sim` from https://github.com/kuriandiliptarento/autrio-car-sim (`main`).

## Head-to-head

| Simulator | Card | Grade | Points | Pct | Pass / fair / weak / fail |
|---|---|---|---|---|---|
| **This plant** (`vehicle-simulator`) | 75 kWh / 1800 kg | **B** | 80/90 | 88.9% | 20/4/3/0 |
| autrio-car-sim as shipped | 82 kWh / 1900 kg | **F** | 50/90 | 55.6% | 11/1/9/6 |
| autrio-car-sim, our card numbers | 75 kWh / 1800 kg | **D** | 54/90 | 60.0% | 11/4/6/6 |

On the **21 checks both can produce**: this plant 90.3% (A) vs theirs as-shipped 69.4% (D). Wins: us 8 · them 0 · tie 13.

## Cycle energy (Wh/mi)

| Cycle | This plant | autrio-car-sim shipped | autrio-car-sim our card | EPA Vehicle DC | FASTSim M3 RWD |
|---|---|---|---|---|---|
| UDDS | 167.3 | 205.2 | 197.2 | 169.4 | 146.7 |
| HWFET | 190.3 | 193.2 | 187.6 | 180.6 | 157.5 |
| US06 | 243.2 | 284.3 | 275.6 | — | 214.5 |
| 2-cycle | 177.66 | 199.8 | 192.9 | 174.4 | — |

## Checks (as shipped vs this plant)

| Check | This plant | autrio-car-sim | Winner |
|---|---|---|---|
| UDDS Wh/mi vs EPA Vehicle DC | 167.32 (pass, 3/3, -1.2%) | 205.19 (weak, 1/3, +21.1%) | us |
| HWFET Wh/mi vs EPA Vehicle DC | 190.3 (pass, 3/3, +5.4%) | 193.23 (pass, 3/3, +7.0%) | tie |
| 2-cycle combined vs EPA | 177.661 (pass, 3/3, +1.9%) | 199.808 (fair, 2/3, +14.6%) | us |
| UDDS Wh/mi vs FASTSim M3 RWD | 167.32 (fair, 2/3, +14.1%) | 205.19 (weak, 1/3, +39.9%) | us |
| HWFET Wh/mi vs FASTSim M3 RWD | 190.3 (weak, 1/3, +20.9%) | 193.23 (weak, 1/3, +22.7%) | tie |
| US06 Wh/mi vs FASTSim M3 RWD | 243.23 (fair, 2/3, +13.4%) | 284.33 (weak, 1/3, +32.5%) | us |
| UDDS speed RMSE | 0.346 (pass, 3/3, —) | n/a (open-loop prescribed speed; no closed-loop driver) | us (they cannot produce this) |
| HWFET speed RMSE | 0.166 (pass, 3/3, —) | n/a (open-loop prescribed speed; no closed-loop driver) | us (they cannot produce this) |
| US06 speed RMSE | 0.728 (fair, 2/3, —) | n/a (open-loop prescribed speed; no closed-loop driver) | us (they cannot produce this) |
| Road load 20 mph vs EPA ABC | 236.3858 (fair, 2/3, +17.2%) | 248.1578 (weak, 1/3, +23.1%) | us |
| Road load 50 mph vs EPA ABC | 364.9571 (pass, 3/3, +5.5%) | 376.7291 (pass, 3/3, +8.9%) | tie |
| Road load 70 mph vs EPA ABC | 511.8958 (pass, 3/3, +0.6%) | 523.6678 (pass, 3/3, +2.9%) | tie |
| DC 10→50 kWh minutes vs ABRP | 22.71 (pass, 3/3, -1.3%) | 17.58 (weak, 1/3, -23.6%) | us |
| DC power at 20% SOC | 149.68 (pass, 3/3, +0.0%) | 149.7 (pass, 3/3, +0.0%) | tie |
| DC power at 50% SOC | 91.35 (pass, 3/3, +0.0%) | 149.7 (weak, 1/3, +36.1%) | us |
| DC power at 80% SOC | 50.4 (pass, 3/3, +0.0%) | 72.8 (weak, 1/3, +32.4%) | us |
| AC into pack vs line×OBC | 6452.0511 (pass, 3/3, -2.6%) | 6324.0 (pass, 3/3, -4.5%) | tie |
| Motor rpm at 100 kph vs Model 3 18" | 6820.9261 (pass, 3/3, -3.1%) | n/a (no gear_ratio / wheel_radius on their card) | us (they cannot produce this) |
| OCV at 50% vs nominal | 350.0 (pass, 3/3, +0.0%) | 355.2 (pass, 3/3, +0.0%) | tie |
| OCV at 20% vs NMC 3.55 V/cell | 335.8108 (pass, 3/3, +0.0%) | 340.8 (pass, 3/3, -0.0%) | tie |
| OCV at 80% vs NMC 3.95 V/cell | 373.6486 (pass, 3/3, +0.0%) | 379.2 (pass, 3/3, +0.0%) | tie |
| Pack R at 25 °C vs 35–90 mΩ | 0.07 (pass, 3/3, +0.0%) | 0.07 (pass, 3/3, +0.0%) | tie |
| 200 A instantaneous sag | 9.8 (pass, 3/3, +0.0%) | 14.0 (pass, 3/3, +0.0%) | tie |
| Rated Wh/km vs fleet p50 | 150.0 (pass, 3/3, -3.2%) | n/a (no rated_wh_per_km on their card) | us (they cannot produce this) |
| HWFET Wh/km vs fleet p50 | 118.25 (weak, 1/3, -23.7%) | 120.07 (weak, 1/3, -22.5%) | tie |
| Sleep load vs 10–50 W | 15.0 (pass, 3/3, +0.0%) | 15 (pass, 3/3, +0.0%) | tie |
| One-pedal regen vs ~0.2 g | 1.2 (weak, 1/3, -38.8%) | n/a (no lift-off deceleration; regen is a power cap) | us (they cannot produce this) |

## What this comparison is

- **This plant** is a closed-loop 10 Hz vehicle: speed-tracking driver, motor losses, pack RC, BMS taper.
- **autrio-car-sim** is a 1 Hz open-loop energy step on a prescribed speed. Motion comes from a week script, not from tracking EPA.
- Missing checks (speed RMSE, motor rpm, rated Wh/km, one-pedal) score 0 on the full card because the colleague sim cannot answer them. The overlap row is the fair physics-only comparison.
- HVAC off on both. Their as-shipped card is heavier (1900 kg) and a larger pack (82 kWh) than ours (1800 kg / 75 kWh).
