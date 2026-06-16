import numpy as np
from cereal import car
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.chauffeur_helpers import detect_rollback
from openpilot.selfdrive.controls.lib.chauffeur_stop import (
  apply_chauffeur_stop,
  in_chauffeur_zone,
  is_chauffeur_stop_enabled,
  rollback_brake_accel,
)
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N
from openpilot.common.pid import PIDController
from openpilot.selfdrive.modeld.constants import ModelConstants

CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]

LongCtrlState = car.CarControl.Actuators.LongControlState


def long_control_state_trans(CP, CP_SP, active, long_control_state, v_ego,
                             should_stop, brake_pressed, cruise_standstill):
  # Gas Interceptor
  cruise_standstill = cruise_standstill and not CP_SP.enableGasInterceptor

  stopping_condition = should_stop
  starting_condition = (not should_stop and
                        not cruise_standstill and
                        not brake_pressed)
  started_condition = v_ego > CP.vEgoStarting

  if not active:
    long_control_state = LongCtrlState.off

  else:
    if long_control_state == LongCtrlState.off:
      if not starting_condition:
        long_control_state = LongCtrlState.stopping
      else:
        if starting_condition and CP.startingState:
          long_control_state = LongCtrlState.starting
        else:
          long_control_state = LongCtrlState.pid

    elif long_control_state == LongCtrlState.stopping:
      if starting_condition and CP.startingState:
        long_control_state = LongCtrlState.starting
      elif starting_condition:
        long_control_state = LongCtrlState.pid

    elif long_control_state in [LongCtrlState.starting, LongCtrlState.pid]:
      if stopping_condition:
        long_control_state = LongCtrlState.stopping
      elif started_condition:
        long_control_state = LongCtrlState.pid
  return long_control_state

class LongControl:
  def __init__(self, CP, CP_SP):
    self.CP = CP
    self.CP_SP = CP_SP
    self.long_control_state = LongCtrlState.off
    self.pid = PIDController((CP.longitudinalTuning.kpBP, CP.longitudinalTuning.kpV),
                             (CP.longitudinalTuning.kiBP, CP.longitudinalTuning.kiV),
                             rate=1 / DT_CTRL)
    self.last_output_accel = 0.0

  def reset(self):
    self.pid.reset()

  def _apply_chauffeur(self, output_accel, CS, pitch, roll, v_forward, accel_limits, stopping=False):
    chauffeur_active = is_chauffeur_stop_enabled() and in_chauffeur_zone(CS.vEgo)
    if detect_rollback(CS.vEgo, v_forward, chauffeur_active or stopping):
      return rollback_brake_accel(self.CP.stopAccel, accel_limits[0])

    if not chauffeur_active:
      return output_accel

    if not stopping and output_accel >= 0.0:
      return output_accel

    return apply_chauffeur_stop(output_accel, CS.vEgo, CS.aEgo, self.last_output_accel, DT_CTRL,
                                pitch=pitch, roll=roll, v_forward=v_forward,
                                stop_accel=self.CP.stopAccel, accel_min=accel_limits[0],
                                stopping=stopping)

  def update(self, active, CS, a_target, should_stop, accel_limits, pitch=0.0, roll=0.0, v_forward=0.0):
    """Update longitudinal control. This updates the state machine and runs a PID loop"""
    self.pid.neg_limit = accel_limits[0]
    self.pid.pos_limit = accel_limits[1]

    self.long_control_state = long_control_state_trans(self.CP, self.CP_SP, active, self.long_control_state, CS.vEgo,
                                                       should_stop, CS.brakePressed,
                                                       CS.cruiseState.standstill)
    if self.long_control_state == LongCtrlState.off:
      self.reset()
      output_accel = 0.

    elif self.long_control_state == LongCtrlState.stopping:
      output_accel = self.last_output_accel
      if is_chauffeur_stop_enabled() and in_chauffeur_zone(CS.vEgo):
        output_accel = self._apply_chauffeur(output_accel, CS, pitch, roll, v_forward, accel_limits, stopping=True)
      elif output_accel > self.CP.stopAccel:
        output_accel = min(output_accel, 0.0)
        output_accel -= self.CP.stoppingDecelRate * DT_CTRL
      self.reset()

    elif self.long_control_state == LongCtrlState.starting:
      output_accel = self.CP.startAccel
      self.reset()

    else:  # LongCtrlState.pid
      if CS.gasPressed:
        # Driver is on the gas — track their actual accel instead of integrating
        # speed error that builds while the car ignores openpilot commands.
        self.pid.reset()
        output_accel = float(CS.aEgo)
      else:
        error = a_target - CS.aEgo
        output_accel = self.pid.update(error, speed=CS.vEgo,
                                       feedforward=a_target)
        if is_chauffeur_stop_enabled() and in_chauffeur_zone(CS.vEgo) and output_accel < 0.0:
          output_accel = self._apply_chauffeur(output_accel, CS, pitch, roll, v_forward, accel_limits)

    # Hard safety: zero positive accel while driver is braking, no exceptions.
    if CS.brakePressed:
      output_accel = min(output_accel, 0.0)

    self.last_output_accel = np.clip(output_accel, accel_limits[0], accel_limits[1])
    return self.last_output_accel
