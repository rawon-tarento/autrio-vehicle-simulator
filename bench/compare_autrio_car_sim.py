#!/usr/bin/env python3
"""Score kuriandiliptarento/autrio-car-sim on the same physics checks as this plant.

Their live loop is a week-timeline player. The physics that can be scored is
`timeline_gen/battery.py` (B1–B20) driven by a prescribed EPA speed trace, HVAC
off, 25 °C, start SOC 80% — the same treadmill as `run_benchmark.py`.

They have no closed-loop driver, gear ratio, or one-pedal lift-off, so those
checks are marked missing (0 points) rather than guessed.
"""

from __future__ import annotations

import copy
import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
LAB = ROOT.parent
THEIR_ROOT = LAB / "comparisons" / "autrio-car-sim"
sys.path.insert(0, str(THEIR_ROOT))
sys.path.insert(0, str(HERE))

from run_benchmark import (  # noqa: E402
    ABRP_10_TO_50_MIN,
    AMBIENT_C,
    CYCLES,
    DC_BAND_KW,
    EPA_A_LBF,
    EPA_B_LBF_PER_MPH,
    EPA_C_LBF_PER_MPH2,
    EPA_HWFET_WH_PER_MI,
    EPA_UDDS_WH_PER_MI,
    FLEET_WH_PER_KM,
    LBF_N,
    MI_M,
    MPH_MPS,
    PACK_R_OHM_BAND,
    SLEEP_W_BAND,
    SOC_START,
    load_cycle,
    resample,
    run_fastsim,
    score_band,
    score_rel,
    summarize,
    trap_distance_m,
)
from timeline_gen.battery import MODES, load_vehicle, simulate  # noqa: E402

DT_S = 1.0  # their battery step is 1 Hz
G = 9.81
RHO = 1.2


def bench_vehicle() -> dict:
    veh = copy.deepcopy(load_vehicle(THEIR_ROOT / "vehicles" / "tesla_model3"))
    veh["aux"]["hvac_k_kw_per_k"] = 0.0
    veh["aux"]["hvac_modes"] = []
    veh["start"]["soc_pct"] = SOC_START
    veh["ambient"]["min_c"] = AMBIENT_C
    veh["ambient"]["max_c"] = AMBIENT_C
    veh["thermal"]["start_temp_c"] = AMBIENT_C
    return veh


def same_card_vehicle(base: dict) -> dict:
    """Map our 75 kWh / 1800 kg card onto their equations."""
    veh = copy.deepcopy(base)
    veh["body"]["mass_kg"] = 1800.0
    veh["pack"]["energy_bol_kwh"] = 75.0
    veh["pack"]["capacity_ah"] = 75_000.0 / 350.0
    veh["charging"]["dc_station_kw"] = 150.0
    veh["charging"]["dc_max_kw"] = 170.0
    return veh


def replay_cycle(veh: dict, times: list[float], speeds: list[float]) -> dict:
    t_s, v_des = resample(times, speeds, DT_S)
    v = np.asarray(v_des, dtype=float)
    accel = np.diff(v, prepend=v[0]) / DT_S
    n = len(v)
    mode = np.full(n, MODES.index("driving"), dtype=int)
    plugged = np.zeros(n, dtype=int)
    charger = np.zeros(n, dtype=int)
    asleep = np.zeros(n, dtype=bool)
    res = simulate(mode, v * 3.6, accel, plugged, charger, asleep, veh, start_temp_c=AMBIENT_C)
    e_full = veh["pack"]["energy_bol_kwh"] * veh["pack"]["soh"]
    e_start = e_full * veh["start"]["soc_pct"] / 100.0
    e_end = float(res["energy_remaining_kwh"][-1])
    net_kwh = e_start - e_end
    dist_m = trap_distance_m(t_s, v.tolist())
    dist_mi = dist_m / MI_M
    dist_km = dist_m / 1000.0
    return {
        "duration_s": round(t_s[-1], 1),
        "distance_km": round(dist_km, 4),
        "distance_mi": round(dist_mi, 4),
        "net_kwh": round(net_kwh, 5),
        "wh_per_mi": round(net_kwh * 1000.0 / dist_mi, 2) if dist_mi else None,
        "wh_per_km": round(net_kwh * 1000.0 / dist_km, 2) if dist_km else None,
        "soc_end_pct": round(float(res["soc_pct"][-1]), 3),
        "speed_rmse_mph": 0.0,
        "dt_s": DT_S,
        "motion": "prescribed",
    }


def resist_force_n(veh: dict, speed_mps: float) -> float:
    b = veh["body"]
    v = speed_mps
    aero = 0.5 * b["air_density"] * b["cd"] * b["frontal_area_m2"] * v * v
    roll = b["crr"] * b["mass_kg"] * G if v > 0 else 0.0
    return aero + roll


def epa_force_n(mph: float) -> float:
    return (EPA_A_LBF + EPA_B_LBF_PER_MPH * mph + EPA_C_LBF_PER_MPH2 * mph * mph) * LBF_N


def ocv_v(veh: dict, soc_pct: float) -> float:
    p = veh["pack"]
    return float(np.interp(soc_pct, p["ocv_soc_pct"], np.asarray(p["ocv_cell_v"], float) * p["series_cells"]))


def _run_dc(veh: dict, *, start_soc: float, end_soc: float, max_s: int = 7200) -> tuple[float, list[dict]]:
    n = max_s
    mode = np.full(n, MODES.index("charging"), dtype=int)
    speed = np.zeros(n)
    accel = np.zeros(n)
    plugged = np.ones(n, dtype=int)
    charger = np.full(n, 2, dtype=int)
    asleep = np.zeros(n, dtype=bool)
    v = copy.deepcopy(veh)
    v["start"]["soc_pct"] = start_soc
    res = simulate(mode, speed, accel, plugged, charger, asleep, v, start_temp_c=AMBIENT_C)
    soc = res["soc_pct"]
    power = -res["pack_power_kw"]  # into pack
    curve: list[dict] = []
    last_bin = -1
    t_end = float(n)
    for i in range(n):
        s = float(soc[i])
        bin_soc = int(s)
        if bin_soc != last_bin:
            curve.append({"t_s": float(i + 1), "soc_pct": round(s, 2), "power_kw": round(float(power[i]), 2)})
            last_bin = bin_soc
        if s >= end_soc - 1e-6:
            t_end = float(i + 1)
            break
    return t_end, curve


def _sample_curve(curve: list[dict], mark: float, *, max_delta: float = 1.5) -> float | None:
    if not curve:
        return None
    hit = min(curve, key=lambda row: abs(row["soc_pct"] - mark))
    if abs(hit["soc_pct"] - mark) > max_delta:
        return None
    return hit["power_kw"]


def dc_session(veh: dict) -> dict:
    e_full = veh["pack"]["energy_bol_kwh"] * veh["pack"]["soh"]
    t_1050, _ = _run_dc(veh, start_soc=10.0 / e_full * 100.0, end_soc=50.0 / e_full * 100.0)
    _, curve = _run_dc(veh, start_soc=10.0, end_soc=90.0)
    samples = {str(mark): _sample_curve(curve, float(mark)) for mark in sorted(DC_BAND_KW)}
    return {
        "minutes": round(t_1050 / 60.0, 2),
        "samples_kw": samples,
        "peak_kw": round(max((row["power_kw"] for row in curve), default=0.0), 2),
        "curve": curve,
        "pack_kwh": e_full,
    }


def ac_into_pack_w(veh: dict) -> float:
    n = 5
    mode = np.full(n, MODES.index("charging"), dtype=int)
    v = copy.deepcopy(veh)
    v["start"]["soc_pct"] = 50.0
    res = simulate(
        mode,
        np.zeros(n),
        np.zeros(n),
        np.ones(n, dtype=int),
        np.ones(n, dtype=int),
        np.zeros(n, dtype=bool),
        v,
        start_temp_c=AMBIENT_C,
    )
    return float(-res["pack_power_kw"][2] * 1000.0)


def missing(reason: str) -> dict:
    return {
        "sim": None,
        "ref": None,
        "error_pct": None,
        "points": 0,
        "max": 3,
        "weight": 1,
        "band": "fail",
        "missing": reason,
    }


def score_plant(veh: dict, *, label: str, fastsim: dict[str, dict | None]) -> dict:
    cycles: dict = {}
    for name in CYCLES:
        times, speeds = load_cycle(name)
        plant = replay_cycle(veh, times, speeds)
        epa = {"udds": EPA_UDDS_WH_PER_MI, "hwfet": EPA_HWFET_WH_PER_MI}.get(name)
        cycles[name] = {"plant": plant, "fastsim": fastsim.get(name), "epa_wh_per_mi": epa}

    combined = 0.55 * cycles["udds"]["plant"]["wh_per_mi"] + 0.45 * cycles["hwfet"]["plant"]["wh_per_mi"]
    epa_combined = 0.55 * EPA_UDDS_WH_PER_MI + 0.45 * EPA_HWFET_WH_PER_MI

    road = {}
    for mph in (20, 50, 70):
        v = mph * MPH_MPS
        ours = resist_force_n(veh, v)
        road[str(mph)] = {"ours_n": ours, "epa_n": epa_force_n(mph)}

    dc = dc_session(veh)
    ac_w = ac_into_pack_w(veh)
    ac_ref = veh["charging"]["ac_kw"] * 1000.0 * veh["charging"]["eta_obc"]
    pack_r = veh["pack"]["r0_ohm"]
    sag = 200.0 * pack_r
    ocv20, ocv50, ocv80 = ocv_v(veh, 20), ocv_v(veh, 50), ocv_v(veh, 80)
    nmc = {
        20: 3.55 / 3.70 * (veh["pack"]["series_cells"] * 3.70),
        50: veh["pack"]["series_cells"] * 3.70,
        80: 3.95 / 3.70 * (veh["pack"]["series_cells"] * 3.70),
    }

    checks: dict[str, dict] = {}
    checks["UDDS Wh/mi vs EPA Vehicle DC"] = score_rel(cycles["udds"]["plant"]["wh_per_mi"], EPA_UDDS_WH_PER_MI, weight=2)
    checks["HWFET Wh/mi vs EPA Vehicle DC"] = score_rel(cycles["hwfet"]["plant"]["wh_per_mi"], EPA_HWFET_WH_PER_MI, weight=2)
    checks["2-cycle combined vs EPA"] = score_rel(combined, epa_combined, weight=2)
    if cycles["udds"]["fastsim"]:
        checks["UDDS Wh/mi vs FASTSim M3 RWD"] = score_rel(cycles["udds"]["plant"]["wh_per_mi"], cycles["udds"]["fastsim"]["wh_per_mi"])
        checks["HWFET Wh/mi vs FASTSim M3 RWD"] = score_rel(cycles["hwfet"]["plant"]["wh_per_mi"], cycles["hwfet"]["fastsim"]["wh_per_mi"])
        checks["US06 Wh/mi vs FASTSim M3 RWD"] = score_rel(cycles["us06"]["plant"]["wh_per_mi"], cycles["us06"]["fastsim"]["wh_per_mi"])
    for name in CYCLES:
        row = missing("open-loop prescribed speed; no closed-loop driver")
        row["weight"] = 1
        row["max"] = 3
        checks[f"{name.upper()} speed RMSE"] = row
    for mph in (20, 50, 70):
        checks[f"Road load {mph} mph vs EPA ABC"] = score_rel(road[str(mph)]["ours_n"], road[str(mph)]["epa_n"])
    checks["DC 10→50 kWh minutes vs ABRP"] = score_rel(dc["minutes"], ABRP_10_TO_50_MIN)
    for mark, (lo, hi) in DC_BAND_KW.items():
        kw = dc["samples_kw"].get(str(mark))
        if kw is not None:
            checks[f"DC power at {mark}% SOC"] = score_band(kw, lo, hi)
        else:
            checks[f"DC power at {mark}% SOC"] = missing("curve did not pass this SOC")
    checks["AC into pack vs line×OBC"] = score_rel(ac_w, ac_ref)
    checks["Motor rpm at 100 kph vs Model 3 18\""] = missing("no gear_ratio / wheel_radius on their card")
    checks["OCV at 50% vs nominal"] = score_rel(ocv50, nmc[50])
    checks["OCV at 20% vs NMC 3.55 V/cell"] = score_rel(ocv20, nmc[20])
    checks["OCV at 80% vs NMC 3.95 V/cell"] = score_rel(ocv80, nmc[80])
    checks["Pack R at 25 °C vs 35–90 mΩ"] = score_band(pack_r, *PACK_R_OHM_BAND)
    checks["200 A instantaneous sag"] = score_band(sag, 5.0, 15.0)
    checks["Rated Wh/km vs fleet p50"] = missing("no rated_wh_per_km on their card")
    checks["HWFET Wh/km vs fleet p50"] = score_rel(cycles["hwfet"]["plant"]["wh_per_km"], FLEET_WH_PER_KM)
    checks["Sleep load vs 10–50 W"] = score_band(veh["aux"]["sleep_w"], *SLEEP_W_BAND)
    checks["One-pedal regen vs ~0.2 g"] = missing("no lift-off deceleration; regen is a power cap")

    return {
        "label": label,
        "score": summarize(checks),
        "checks": checks,
        "cycles": cycles,
        "combined_wh_per_mi": round(combined, 2),
        "epa_combined_wh_per_mi": round(epa_combined, 2),
        "road_load": {k: {ik: round(iv, 3) for ik, iv in row.items()} for k, row in road.items()},
        "dc": {k: v for k, v in dc.items() if k != "curve"} | {"curve_n": len(dc["curve"])},
        "ac_into_pack_w": round(ac_w, 1),
        "ac_ref_w": round(ac_ref, 1),
        "ocv": {"20": round(ocv20, 2), "50": round(ocv50, 2), "80": round(ocv80, 2)},
        "card": {
            "mass_kg": veh["body"]["mass_kg"],
            "pack_kwh": veh["pack"]["energy_bol_kwh"],
            "cd": veh["body"]["cd"],
            "crr": veh["body"]["crr"],
            "frontal_area_m2": veh["body"]["frontal_area_m2"],
        },
    }


def overlap_score(ours_checks: dict, theirs_checks: dict) -> dict:
    keys = [k for k, row in theirs_checks.items() if not row.get("missing")]
    ours = summarize({k: ours_checks[k] for k in keys if k in ours_checks})
    theirs = summarize({k: theirs_checks[k] for k in keys})
    wins = {"ours": 0, "theirs": 0, "tie": 0}
    deltas = []
    for k in keys:
        if k not in ours_checks:
            continue
        op, tp = ours_checks[k]["points"], theirs_checks[k]["points"]
        if op > tp:
            wins["ours"] += 1
            who = "ours"
        elif tp > op:
            wins["theirs"] += 1
            who = "theirs"
        else:
            wins["tie"] += 1
            who = "tie"
        deltas.append({"check": k, "ours": op, "theirs": tp, "winner": who})
    return {"n": len(keys), "ours": ours, "theirs": theirs, "wins": wins, "deltas": deltas}


def write_markdown(path: Path, ours: dict, shipped: dict, same: dict, overlap: dict) -> None:
    os_, ss, sm = ours["score"], shipped["score"], same["score"]
    lines = [
        "# Physics comparison: this plant vs autrio-car-sim",
        "",
        "Same treadmill, same references, same scoring rule as `BENCHMARK.md`.",
        "Colleague plant is `timeline_gen/battery.py` on a prescribed EPA speed trace at 1 Hz.",
        "Clone: `lab/comparisons/autrio-car-sim` from https://github.com/kuriandiliptarento/autrio-car-sim (`main`).",
        "",
        "## Head-to-head",
        "",
        "| Simulator | Card | Grade | Points | Pct | Pass / fair / weak / fail |",
        "|---|---|---|---|---|---|",
        f"| **This plant** (`vehicle-simulator`) | 75 kWh / 1800 kg | **{os_['grade']}** | {os_['earned']:.0f}/{os_['possible']:.0f} | {os_['pct']}% | {os_['counts']['pass']}/{os_['counts']['fair']}/{os_['counts']['weak']}/{os_['counts']['fail']} |",
        f"| autrio-car-sim as shipped | {shipped['card']['pack_kwh']:g} kWh / {shipped['card']['mass_kg']:g} kg | **{ss['grade']}** | {ss['earned']:.0f}/{ss['possible']:.0f} | {ss['pct']}% | {ss['counts']['pass']}/{ss['counts']['fair']}/{ss['counts']['weak']}/{ss['counts']['fail']} |",
        f"| autrio-car-sim, our card numbers | 75 kWh / 1800 kg | **{sm['grade']}** | {sm['earned']:.0f}/{sm['possible']:.0f} | {sm['pct']}% | {sm['counts']['pass']}/{sm['counts']['fair']}/{sm['counts']['weak']}/{sm['counts']['fail']} |",
        "",
        f"On the **{overlap['n']} checks both can produce**: this plant {overlap['ours']['pct']}% ({overlap['ours']['grade']}) vs theirs as-shipped {overlap['theirs']['pct']}% ({overlap['theirs']['grade']}). Wins: us {overlap['wins']['ours']} · them {overlap['wins']['theirs']} · tie {overlap['wins']['tie']}.",
        "",
        "## Cycle energy (Wh/mi)",
        "",
        "| Cycle | This plant | autrio-car-sim shipped | autrio-car-sim our card | EPA Vehicle DC | FASTSim M3 RWD |",
        "|---|---|---|---|---|---|",
    ]
    for name in CYCLES:
        epa = shipped["cycles"][name]["epa_wh_per_mi"]
        fs = shipped["cycles"][name]["fastsim"]
        ours_w = ours["cycles"][name]["plant"]["wh_per_mi"]
        lines.append(
            "| "
            + " | ".join(
                [
                    name.upper(),
                    f"{ours_w:.1f}",
                    f"{shipped['cycles'][name]['plant']['wh_per_mi']:.1f}",
                    f"{same['cycles'][name]['plant']['wh_per_mi']:.1f}",
                    "—" if epa is None else f"{epa:.1f}",
                    "—" if not fs else f"{fs['wh_per_mi']:.1f}",
                ]
            )
            + " |"
        )
    lines += [
        f"| 2-cycle | {ours.get('combined_wh_per_mi', '')} | {shipped['combined_wh_per_mi']:.1f} | {same['combined_wh_per_mi']:.1f} | {shipped['epa_combined_wh_per_mi']:.1f} | — |",
        "",
        "## Checks (as shipped vs this plant)",
        "",
        "| Check | This plant | autrio-car-sim | Winner |",
        "|---|---|---|---|",
    ]
    for key, theirs in shipped["checks"].items():
        ours_row = ours["checks"].get(key)
        o_cell = _cell(ours_row) if ours_row else "—"
        t_cell = _cell(theirs)
        if theirs.get("missing"):
            win = "us (they cannot produce this)"
        elif ours_row is None:
            win = "—"
        elif ours_row["points"] > theirs["points"]:
            win = "us"
        elif theirs["points"] > ours_row["points"]:
            win = "them"
        else:
            win = "tie"
        lines.append(f"| {key} | {o_cell} | {t_cell} | {win} |")
    lines += [
        "",
        "## What this comparison is",
        "",
        "- **This plant** is a closed-loop 10 Hz vehicle: speed-tracking driver, motor losses, pack RC, BMS taper.",
        "- **autrio-car-sim** is a 1 Hz open-loop energy step on a prescribed speed. Motion comes from a week script, not from tracking EPA.",
        "- Missing checks (speed RMSE, motor rpm, rated Wh/km, one-pedal) score 0 on the full card because the colleague sim cannot answer them. The overlap row is the fair physics-only comparison.",
        "- HVAC off on both. Their as-shipped card is heavier (1900 kg) and a larger pack (82 kWh) than ours (1800 kg / 75 kWh).",
        "",
    ]
    path.write_text("\n".join(lines))


def _cell(row: dict | None) -> str:
    if not row:
        return "—"
    if row.get("missing"):
        return f"n/a ({row['missing']})"
    err = row.get("error_pct")
    err_s = "—" if err is None else f"{err:+.1f}%"
    return f"{row['sim']} ({row['band']}, {row['points']}/{row['max']}, {err_s})"


def main() -> None:
    ours_path = HERE / "results.json"
    if not ours_path.is_file():
        raise SystemExit("run bench/run_benchmark.py first")
    ours = json.loads(ours_path.read_text())

    sys.path.insert(0, str(LAB / ".pydeps"))
    fs_by_cycle: dict[str, dict | None] = {}
    try:
        import fastsim as _fastsim  # noqa: F401

        for name in CYCLES:
            times, speeds = load_cycle(name)
            fs_by_cycle[name] = run_fastsim(times, speeds)
    except ImportError:
        for name in CYCLES:
            fs_by_cycle[name] = ours["cycles"][name].get("fastsim")

    veh = bench_vehicle()
    shipped = score_plant(veh, label="autrio-car-sim shipped", fastsim=fs_by_cycle)
    same = score_plant(same_card_vehicle(veh), label="autrio-car-sim our card", fastsim=fs_by_cycle)
    overlap = overlap_score(ours["checks"], shipped["checks"])

    report = {"ours": {"score": ours["score"], "cycles": ours["cycles"]}, "shipped": shipped, "same_card": same, "overlap": overlap}
    # drop nested cycle fastsim duplication in json if huge — fine
    out_json = HERE / "comparison.json"
    slim = copy.deepcopy(report)
    for block in (slim["shipped"], slim["same_card"]):
        block.pop("cycles", None)
        block.get("dc", {}).pop("curve", None)
    out_json.write_text(json.dumps({"ours_score": ours["score"], "shipped": shipped, "same_card": same, "overlap": overlap}, indent=2, default=str))
    write_markdown(HERE / "COMPARISON.md", ours, shipped, same, overlap)
    print(
        json.dumps(
            {
                "ours": ours["score"],
                "autrio_car_sim_shipped": shipped["score"],
                "autrio_car_sim_our_card": same["score"],
                "overlap": {"n": overlap["n"], "ours_pct": overlap["ours"]["pct"], "theirs_pct": overlap["theirs"]["pct"], "wins": overlap["wins"]},
                "out": str(HERE / "COMPARISON.md"),
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
