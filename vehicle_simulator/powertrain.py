"""From the acceleration the driver wants to the wheel force, motor torque, and electrical power.

The driver asks for an acceleration. The motor gives what torque, power, and the battery allow.
Lifting off gives one-pedal regen. Anything beyond regen is the friction brake, and the brake pedal goes down.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from vehicle_simulator.card import VehicleCard

G = 9.81
RHO_AIR = 1.2
_REGEN_MIN_KPH = 1.0
_HOLD_MPS = 0.05


@dataclass(frozen=True)
class DriveOutput:
    accel_mps2: float
    motor_force_n: float
    friction_force_n: float
    motor_torque_nm: float
    motor_rpm: float
    motor_power_w: float
    traction_el_w: float
    pedal_pct: float
    brake_pedal: bool
    brake_light: bool


def resist_force_n(card: VehicleCard, speed_mps: float, grade: float) -> float:
    """Rolling, climbing, and air drag along the road. Climbing is negative downhill."""
    slope = math.sqrt(1.0 + grade * grade)
    roll = card.rolling_coeff * card.mass_kg * G / slope if speed_mps > _HOLD_MPS else 0.0
    climb = card.mass_kg * G * grade / slope
    drag = 0.5 * RHO_AIR * card.cd * card.frontal_area_m2 * speed_mps * speed_mps
    return roll + climb + drag


def effective_mass_kg(card: VehicleCard) -> float:
    return card.mass_kg * card.rot_mass_factor


def motor_loss_w(card: VehicleCard, torque_nm: float, omega_rads: float) -> float:
    return card.motor_fixed_loss_w + card.motor_copper_loss_w_per_nm2 * torque_nm * torque_nm + card.motor_iron_loss_w_per_rads * abs(omega_rads)


def drive_step(
    card: VehicleCard,
    *,
    speed_mps: float,
    want_accel_mps2: float,
    grade: float,
    max_discharge_w: float,
    max_charge_w: float,
    other_loads_w: float,
) -> DriveOutput:
    m_eff = effective_mass_kg(card)
    resist = resist_force_n(card, speed_mps, grade)
    need = m_eff * want_accel_mps2 + resist
    v = max(speed_mps, 0.5)
    ratio = card.gear_ratio / card.wheel_radius_m
    torque_force = card.max_motor_torque_nm * ratio
    drive_w = min(card.max_drive_power_kw * 1000.0, max(0.0, max_discharge_w - other_loads_w))
    max_drive_force = min(torque_force, drive_w * 0.92 / v)
    regen_w = min(card.max_regen_kw * 1000.0, max(0.0, max_charge_w + other_loads_w))
    fade = _clamp((speed_mps * 3.6 - _REGEN_MIN_KPH) / (card.regen_fade_kph - _REGEN_MIN_KPH), 0.0, 1.0)
    max_regen_force = min(torque_force, regen_w / 0.92 / v) * fade
    liftoff_force = min(max_regen_force, m_eff * card.liftoff_decel_mps2)

    holding = speed_mps <= _HOLD_MPS and want_accel_mps2 <= 0.0
    friction = 0.0
    if holding:
        motor = 0.0
    elif need >= 0.0:
        motor = min(need, max_drive_force)
    else:
        motor = max(need, -liftoff_force)
        friction = need - motor

    accel = (motor + friction - resist) / m_eff
    if holding:
        friction = -resist
        accel = 0.0
    elif speed_mps <= _HOLD_MPS and accel < 0.0:
        accel = 0.0
    torque = motor / ratio
    omega = speed_mps / card.wheel_radius_m * card.gear_ratio
    mech_w = motor * speed_mps
    traction_el = 0.0 if holding and speed_mps <= _HOLD_MPS else mech_w + motor_loss_w(card, torque, omega)
    brake_pedal = holding or friction < -1.0
    if motor >= 0.0:
        pedal = card.pedal_coast_pct + (100.0 - card.pedal_coast_pct) * (motor / max_drive_force if max_drive_force > 0 else 0.0)
    else:
        pedal = card.pedal_coast_pct * (1.0 - (-motor / liftoff_force if liftoff_force > 0 else 1.0))
    if brake_pedal:
        pedal = 0.0
    decel_regen = -motor / m_eff
    return DriveOutput(
        accel_mps2=accel,
        motor_force_n=motor,
        friction_force_n=friction,
        motor_torque_nm=torque,
        motor_rpm=omega * 60.0 / (2.0 * math.pi),
        motor_power_w=mech_w,
        traction_el_w=traction_el,
        pedal_pct=_clamp(pedal, 0.0, 100.0),
        brake_pedal=brake_pedal,
        brake_light=brake_pedal or decel_regen > card.brake_light_decel_mps2,
    )


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))
