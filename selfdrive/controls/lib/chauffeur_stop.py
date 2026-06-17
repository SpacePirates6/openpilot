import numpy as np

from openpilot.common.params import Params, UnknownKeyName
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.chauffeur_helpers import (
  CHAUFFEUR_MAX_SPEED,
  detect_rollback,
  hill_hold_offset,
  in_chauffeur_zone,
)

# Re-exported for opendbc Honda carcontroller imports.
__all__ = (
  "CHAUFFEUR_MAX_SPEED",
  "apply_chauffeur_stop",
  "compute_chauffeur_decel_target",
  "in_chauffeur_zone",
  "is_chauffeur_stop_enabled",
  "rollback_brake_accel",
)

DECEL_AT_STOP = -0.12
DECEL_AT_THRESHOLD = -1.0
CHAUFFEUR_JERK = 2.0

# Match Honda/carcontroller hill compensation: sin(pitch) * g added to accel command.
MAX_UPHILL_HOLD_ACCEL = 0.35
MAX_DOWNHILL_DECEL = -2.5
STEEP_DOWNHILL_HILL = -0.25

ROLLBACK_BRAKE_ACCEL = -2.5

_params: Params | None = None


def is_chauffeur_stop_enabled() -> bool:
  global _params
  if _params is None:
    _params = Params()
  try:
    return _params.get_bool("ChauffeurStopEnabled")
  except UnknownKeyName:
    return False


def rollback_brake_accel(stop_accel: float, accel_min: float) -> float:
  return float(min(stop_accel, accel_min, ROLLBACK_BRAKE_ACCEL))


def compute_chauffeur_decel_target(v_ego: float, a_ego: float, pitch: float = 0.0, roll: float = 0.0) -> float:
  del roll  # reserved for future cross-slope tuning
  hill = hill_hold_offset(pitch)

  decel_at_stop = float(np.clip(DECEL_AT_STOP + hill, MAX_DOWNHILL_DECEL, MAX_UPHILL_HOLD_ACCEL))
  decel_at_threshold = float(np.clip(DECEL_AT_THRESHOLD + hill, MAX_DOWNHILL_DECEL, 0.0))

  if v_ego <= 0.0:
    return decel_at_stop

  speed_ratio = min(v_ego / CHAUFFEUR_MAX_SPEED, 1.0)
  taper = speed_ratio ** 2.0
  target = decel_at_stop + taper * (decel_at_threshold - decel_at_stop)

  # Ease off early on flat/mild grades; keep more brake loaded on downhill.
  if a_ego < target and hill > STEEP_DOWNHILL_HILL:
    target = min(target + min((target - a_ego) * 0.25, 0.35), -0.05)

  return float(np.clip(target, decel_at_stop, 0.0))


def apply_chauffeur_stop(output_accel: float,
                         v_ego: float,
                         a_ego: float,
                         prev_output_accel: float,
                         dt: float,
                         *,
                         pitch: float = 0.0,
                         roll: float = 0.0,
                         v_forward: float = 0.0,
                         stop_accel: float = -2.0,
                         accel_min: float = -3.5,
                         stopping: bool = False) -> float:
  chauffeur_active = is_chauffeur_stop_enabled() and in_chauffeur_zone(v_ego) and output_accel < 0.0

  if detect_rollback(v_ego, v_forward, chauffeur_active or stopping):
    return rollback_brake_accel(stop_accel, accel_min)

  if not is_chauffeur_stop_enabled() or not in_chauffeur_zone(v_ego) or output_accel >= 0.0:
    return output_accel

  target = compute_chauffeur_decel_target(v_ego, a_ego, pitch, roll)
  capped = max(output_accel, target)

  hill = hill_hold_offset(pitch)
  jerk = CHAUFFEUR_JERK * (1.5 if hill < STEEP_DOWNHILL_HILL else 1.0)
  max_delta = max(jerk * dt, DT_CTRL)
  return float(np.clip(capped, prev_output_accel - max_delta, prev_output_accel + max_delta))
