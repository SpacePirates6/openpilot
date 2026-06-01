import time

import numpy as np

from openpilot.common.params import Params, UnknownKeyName
from openpilot.common.realtime import DT_CTRL

# Hold learning and ramp commands after the driver releases the gas pedal.
RECOVERY_DURATION = 1.2
# Max increase in commanded accel during recovery (m/s² per second).
ACCEL_RAMP_UP = 1.2
# Allow braking to catch up faster than acceleration.
ACCEL_RAMP_DOWN = 2.5
# Bosch gas command units per control frame during recovery.
RECOVERY_GAS_RAMP = 25
# Pull an inflated gasfactor back toward 1.0 during recovery.
GASFACTOR_DECAY_RATE = 0.004

_params: Params | None = None


def is_gas_override_smooth_enabled() -> bool:
  global _params
  if _params is None:
    _params = Params()
  try:
    return _params.get_bool("GasOverrideSmoothEnabled")
  except UnknownKeyName:
    return True


class GasOverrideSmooth:
  def __init__(self):
    self.gas_pressed_prev = False
    self.recovery_until = 0.0
    self._smooth_accel: float | None = None

  def update(self, gas_pressed: bool) -> None:
    if self.gas_pressed_prev and not gas_pressed:
      self.recovery_until = time.monotonic() + RECOVERY_DURATION
      self._smooth_accel = None
    if gas_pressed:
      self.recovery_until = 0.0
    self.gas_pressed_prev = gas_pressed

  def in_recovery(self) -> bool:
    return time.monotonic() < self.recovery_until

  def allow_gasfactor_learning(self) -> bool:
    return not self.in_recovery()

  def recovery_gas_ramp(self, default_ramp: float) -> float:
    return RECOVERY_GAS_RAMP if self.in_recovery() else default_ramp

  def decay_gasfactor(self, gasfactor: float) -> float:
    if not self.in_recovery():
      return gasfactor
    return float(np.clip(gasfactor + np.clip(1.0 - gasfactor, -GASFACTOR_DECAY_RATE, GASFACTOR_DECAY_RATE), 0.1, 3.0))

  def smooth_accel(self, target_accel: float, a_ego: float) -> float:
    if not self.in_recovery():
      self._smooth_accel = target_accel
      return target_accel

    if self._smooth_accel is None:
      self._smooth_accel = a_ego

    max_up = ACCEL_RAMP_UP * DT_CTRL
    max_down = ACCEL_RAMP_DOWN * DT_CTRL
    delta = target_accel - self._smooth_accel
    if delta >= 0.0:
      self._smooth_accel += min(delta, max_up)
    else:
      self._smooth_accel += max(delta, -max_down)
    return float(self._smooth_accel)
