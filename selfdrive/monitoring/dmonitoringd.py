#!/usr/bin/env python3
"""Stub driver monitoring: publishes safe defaults without camera or model."""
import cereal.messaging as messaging
from cereal import log
from openpilot.common.params import Params
from openpilot.common.realtime import DT_DMON, Ratekeeper, config_realtime_process

AlertLevel = log.DriverMonitoringState.AlertLevel
MonitoringPolicy = log.DriverMonitoringState.MonitoringPolicy


def _fill_driver_data(data, is_rhd: bool):
  data.faceOrientation = [0., 0., 0.]
  data.faceOrientationStd = [0.01, 0.01, 0.01]
  data.facePosition = [0., 0.]
  data.facePositionStd = [0.01, 0.01]
  data.faceProb = 1.0
  data.eyesVisibleProb = 1.0
  data.eyesClosedProb = 0.0
  data.phoneProb = 0.0


def _driver_state_packet(frame_id: int, is_rhd: bool):
  msg = messaging.new_message('driverStateV2', valid=True)
  ds = msg.driverStateV2
  ds.frameId = frame_id
  ds.modelExecutionTime = 0.0
  ds.gpuExecutionTime = 0.0
  ds.wheelOnRightProb = 1.0 if is_rhd else 0.0
  _fill_driver_data(ds.leftDriverData, is_rhd=False)
  _fill_driver_data(ds.rightDriverData, is_rhd=True)
  return msg


def _monitoring_state_packet(is_rhd: bool):
  msg = messaging.new_message('driverMonitoringState', valid=True)
  dm = msg.driverMonitoringState

  dm.lockout = False
  dm.alwaysOnLockout = False
  dm.alertLevel = AlertLevel.none
  dm.activePolicy = MonitoringPolicy.vision
  dm.isRHD = is_rhd

  dm.visionPolicyState.faceDetected = True
  dm.visionPolicyState.isDistracted = False
  dm.visionPolicyState.awarenessPercent = 100
  dm.visionPolicyState.uncertainOffroadAlertPercent = 0

  dm.wheeltouchPolicyState.awarenessPercent = 100
  dm.wheeltouchPolicyState.driverInteracting = False
  return msg


def dmonitoringd_thread():
  config_realtime_process(0, 5)

  params = Params()
  if params.get_bool("DriverTooDistracted"):
    params.put_bool("DriverTooDistracted", False)

  pm = messaging.PubMaster(['driverMonitoringState', 'driverStateV2'])
  rk = Ratekeeper(1. / DT_DMON, print_delay_threshold=None)
  frame_id = 0

  while True:
    is_rhd = params.get_bool("IsRhdDetected")
    pm.send('driverMonitoringState', _monitoring_state_packet(is_rhd))
    pm.send('driverStateV2', _driver_state_packet(frame_id, is_rhd))
    frame_id += 1
    rk.keep_time()


def main():
  dmonitoringd_thread()


if __name__ == '__main__':
  main()
