import pytest

from openpilot.common.constants import CV
from openpilot.selfdrive.controls.lib.chauffeur_helpers import detect_rollback
from openpilot.selfdrive.controls.lib.chauffeur_stop import (
  CHAUFFEUR_MAX_SPEED,
  apply_chauffeur_stop,
  compute_chauffeur_decel_target,
  hill_hold_offset,
  in_chauffeur_zone,
  rollback_brake_accel,
)


class TestChauffeurStop:

  def test_in_chauffeur_zone(self):
    assert in_chauffeur_zone(0.0)
    assert in_chauffeur_zone(CHAUFFEUR_MAX_SPEED - 0.01)
    assert not in_chauffeur_zone(CHAUFFEUR_MAX_SPEED)
    assert not in_chauffeur_zone(5.0)

  def test_decel_target_softens_near_stop(self):
    at_threshold = compute_chauffeur_decel_target(CHAUFFEUR_MAX_SPEED, 0.0)
    near_stop = compute_chauffeur_decel_target(0.1, 0.0)
    assert at_threshold < near_stop < 0.0
    assert near_stop > -0.2

  def test_downhill_needs_more_brake(self):
    flat = compute_chauffeur_decel_target(0.1, 0.0, pitch=0.0)
    downhill = compute_chauffeur_decel_target(0.1, 0.0, pitch=-0.08)
    assert downhill < flat

  def test_uphill_needs_less_brake(self):
    flat = compute_chauffeur_decel_target(0.1, 0.0, pitch=0.0)
    uphill = compute_chauffeur_decel_target(0.1, 0.0, pitch=0.08)
    assert uphill > flat

  def test_hill_offset_matches_carcontroller(self):
    pitch = 0.05
    assert hill_hold_offset(pitch) > 0.0

  def test_apply_caps_aggressive_brake(self):
    prev = -2.0
    softened = apply_chauffeur_stop(prev, 0.5, -1.5, prev, 0.01)
    assert softened > prev

  def test_apply_jerk_limits_step(self):
    prev = -0.5
    softened = apply_chauffeur_stop(-2.0, 0.5, -1.5, prev, 0.01)
    assert softened >= prev - 0.02
    assert softened <= prev + 0.02

  def test_no_change_outside_zone(self):
    prev = -2.0
    assert apply_chauffeur_stop(prev, 5.0, -1.0, prev, 0.01) == prev

  def test_no_change_when_not_braking(self):
    prev = 0.5
    assert apply_chauffeur_stop(prev, 0.5, 0.0, prev, 0.01) == prev

  def test_unload_ease_when_decelerating_hard(self):
    gentle = compute_chauffeur_decel_target(0.4, -0.2)
    hard = compute_chauffeur_decel_target(0.4, -1.5)
    assert hard > gentle

  def test_disabled_param_passthrough(self, monkeypatch):
    monkeypatch.setattr(
      "openpilot.selfdrive.controls.lib.chauffeur_stop.is_chauffeur_stop_enabled",
      lambda: False,
    )
    prev = -2.0
    assert apply_chauffeur_stop(prev, 0.5, -1.5, prev, 0.01) == prev

  def test_rollback_locks_brakes(self):
    locked = apply_chauffeur_stop(-0.2, 0.3, -0.5, -0.2, 0.01, v_forward=-0.1, stop_accel=-3.5, accel_min=-3.5)
    assert locked <= rollback_brake_accel(-3.5, -3.5)

  def test_detect_rollback_forward_velocity(self):
    assert detect_rollback(0.2, -0.08, True)
