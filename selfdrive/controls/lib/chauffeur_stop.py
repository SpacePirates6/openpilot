import math
import time

import numpy as np

from openpilot.common.constants import ACCELERATION_DUE_TO_GRAVITY, CV
from openpilot.common.params import Params, UnknownKeyName
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.chauffeur_learned import (
  detect_rollback,
  learned_offsets_for_grade,
  load_store,
)

# Below 2 mph the car has little kinetic energy left but suspension is still loaded.
# Taper brake commands here to avoid the final jolt when inertia and spring rebound meet.
CHAUFFEUR_MAX_SPEED = 2.0 * CV.MPH_TO_MS

DECEL_AT_STOP = -0.12
DECEL_AT_THRESHOLD = -1.0
CHAUFFEUR_JERK = 2.0

# Match Honda/carcontroller hill compensation: sin(pitch) * g added to accel command.
MAX_UPHILL_HOLD_ACCEL = 0.35
MAX_DOWNHILL_DECEL = -2.5
STEEP_DOWNHILL_HILL = -0.25

ROLLBACK_BRAKE_ACCEL = -2.5

_params: Params | None = None
_learned_store = None
_learned_store_ts = 0.0
LEARNED_STORE_TTL = 2.0


def is_chauffeur_stop_enabled() -> bool:
  global _params
  if _params is None:
    _params = Params()
  try:
    return _params.get_bool("ChauffeurStopEnabled")
  except UnknownKeyName:
    return False


def _get_learned_store():
  global _learned_store, _learned_store_ts
  now = time.monotonic()
  if _learned_store is None or (now - _learned_store_ts) > LEARNED_STORE_TTL:
    _learned_store = load_store(_params)
    _learned_store_ts = now
  return _learned_store


def invalidate_learned_cache() -> None:
  global _learned_store_ts
  _learned_store_ts = 0.0


def in_chauffeur_zone(v_ego: float) -> bool:
  return 0.0 <= v_ego < CHAUFFEUR_MAX_SPEED


def hill_hold_offset(pitch: float) -> float:
  return math.sin(pitch) * ACCELERATION_DUE_TO_GRAVITY


def rollback_brake_accel(stop_accel: float, accel_min: float) -> float:
  return float(min(stop_accel, accel_min, ROLLBACK_BRAKE_ACCEL))


def compute_chauffeur_decel_target(v_ego: float, a_ego: float, pitch: float = 0.0, roll: float = 0.0) -> float:
  hill = hill_hold_offset(pitch)
  decel_offset, _, taper_offset = learned_offsets_for_grade(_get_learned_store(), pitch, roll)

  decel_at_stop = float(np.clip(DECEL_AT_STOP + hill + decel_offset, MAX_DOWNHILL_DECEL, MAX_UPHILL_HOLD_ACCEL))
  decel_at_threshold = float(np.clip(DECEL_AT_THRESHOLD + hill + decel_offset, MAX_DOWNHILL_DECEL, 0.0))

  if v_ego <= 0.0:
    return decel_at_stop

  speed_ratio = min(v_ego / CHAUFFEUR_MAX_SPEED, 1.0)
  taper_power = float(np.clip(2.0 + taper_offset, 1.2, 3.0))
  taper = speed_ratio ** taper_power
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

  _ = stopping
  if not is_chauffeur_stop_enabled() or not in_chauffeur_zone(v_ego) or output_accel >= 0.0:
    return output_accel

  target = compute_chauffeur_decel_target(v_ego, a_ego, pitch, roll)
  capped = max(output_accel, target)

  _, jerk_offset, _ = learned_offsets_for_grade(_get_learned_store(), pitch, roll)
  hill = hill_hold_offset(pitch)
  jerk = (CHAUFFEUR_JERK + jerk_offset) * (1.5 if hill < STEEP_DOWNHILL_HILL else 1.0)
  max_delta = max(jerk * dt, DT_CTRL)
  return float(np.clip(capped, prev_output_accel - max_delta, prev_output_accel + max_delta))
