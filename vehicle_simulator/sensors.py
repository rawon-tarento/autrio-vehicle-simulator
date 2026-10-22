"""What the car's sensors report, as opposed to the true state: noise, bias, filtering, and GPS fix time."""

from __future__ import annotations

import math
import random

from vehicle_simulator.card import VehicleCard
from vehicle_simulator.powertrain import G

_IMU_SIGMA_MPS2 = 0.02
_YAW_SIGMA_RADS = 0.002
_PACK_V_SIGMA = 0.05
_PACK_A_SIGMA = 0.1
_GPS_TAU_S = 20.0
_GPS_BASE_M = 3.0
_GPS_COLD_M = 27.0
_M_PER_DEG = 111_320.0


class Sensors:
    def __init__(self, card: VehicleCard) -> None:
        self.card = card
        self._rng = random.Random(card.sensor_seed)
        self._gps_n = 0.0
        self._gps_e = 0.0
        self.awake_s = 0.0

    def noise(self, sigma: float) -> float:
        if not self.card.sensor_noise:
            return 0.0
        return self._rng.gauss(0.0, sigma)

    def asleep(self) -> None:
        self.awake_s = 0.0

    def tick(self, dt_s: float) -> None:
        self.awake_s += dt_s

    def long_accel(self, accel_mps2: float, grade: float) -> float:
        """An accelerometer feels the slope too: a + g sin(theta)."""
        return accel_mps2 + G * math.sin(math.atan(grade)) + self.noise(_IMU_SIGMA_MPS2)

    def lat_accel(self, lat_mps2: float) -> float:
        return lat_mps2 + self.noise(_IMU_SIGMA_MPS2)

    def yaw_rate(self, yaw_rads: float) -> float:
        return yaw_rads + self.noise(_YAW_SIGMA_RADS)

    def pack_voltage(self, volts: float) -> float:
        return volts + self.noise(_PACK_V_SIGMA)

    def pack_current(self, amps: float) -> float:
        return amps + self.noise(_PACK_A_SIGMA)

    def display_speed(self, speed_kph: float) -> float:
        return speed_kph * (1.0 + self.card.speedo_overread)

    def gps_accuracy(self, speed_kph: float) -> float:
        cold = _GPS_COLD_M * math.exp(-self.awake_s / self.card.gps_fix_s)
        return min(30.0, _GPS_BASE_M + min(9.0, 0.04 * speed_kph) + cold)

    def gps(self, lat: float, lon: float, speed_kph: float, dt_s: float) -> tuple[float, float, float]:
        """Reported fix: the true point plus a slowly wandering error the size of the accuracy figure."""
        accuracy = self.gps_accuracy(speed_kph)
        if self.card.sensor_noise:
            sigma = accuracy * 0.5
            k = dt_s / _GPS_TAU_S
            scale = sigma * math.sqrt(2.0 * k)
            self._gps_n += -self._gps_n * k + self._rng.gauss(0.0, scale)
            self._gps_e += -self._gps_e * k + self._rng.gauss(0.0, scale)
        lat_out = lat + self._gps_n / _M_PER_DEG
        lon_out = lon + self._gps_e / (_M_PER_DEG * max(0.01, math.cos(math.radians(lat))))
        return lat_out, lon_out, accuracy
