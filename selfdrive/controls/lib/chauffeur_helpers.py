import math

from openpilot.common.constants import ACCELERATION_DUE_TO_GRAVITY, CV

CHAUFFEUR_MAX_SPEED = 2.0 * CV.MPH_TO_MS
ROLLBACK_V_EGO = -0.03
ROLLBACK_V_FORWARD = -0.05


def hill_hold_offset(pitch: float) -> float:
  return math.sin(pitch) * ACCELERATION_DUE_TO_GRAVITY


def in_chauffeur_zone(v_ego: float) -> bool:
  return 0.0 <= v_ego < CHAUFFEUR_MAX_SPEED


def detect_rollback(v_ego: float, v_forward: float, chauffeur_active: bool) -> bool:
  if not chauffeur_active:
    return False
  return v_ego < ROLLBACK_V_EGO or v_forward < ROLLBACK_V_FORWARD
