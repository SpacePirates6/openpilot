#!/usr/bin/env python3
import cereal.messaging as messaging
from cereal import car
from openpilot.common.params import Params
from openpilot.common.realtime import Priority, config_realtime_process
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib.chauffeur_learned import ChauffeurStopLearner
from openpilot.selfdrive.locationd.helpers import Pose, PoseCalibrator


def main():
  config_realtime_process(20, Priority.CTRL_LOW)

  params = Params()
  cloudlog.info("chauffeur_learnedd is waiting for CarParams")
  messaging.log_from_bytes(params.get("CarParams", block=True), car.CarParams)

  learner = ChauffeurStopLearner()
  calibrator = PoseCalibrator()
  sm = messaging.SubMaster(['carState', 'carControl', 'livePose', 'liveCalibration', 'longitudinalPlan'],
                           poll='carState')

  while True:
    sm.update()
    if not sm.all_checks():
      continue

    if sm.updated['liveCalibration']:
      calibrator.feed_live_calib(sm['liveCalibration'])

    pitch = 0.0
    roll = 0.0
    v_forward = 0.0
    if sm.valid['livePose']:
      pose = Pose.from_live_pose(sm['livePose'])
      calibrated = calibrator.build_calibrated_pose(pose)
      pitch = float(calibrated.orientation.pitch)
      roll = float(calibrated.orientation.roll)
      v_forward = float(calibrated.velocity.x)

    cs = sm['carState']
    cc = sm['carControl']
    lp = sm['longitudinalPlan']
    t = sm.logMonoTime['carState'] * 1e-9

    learner.update(
      t=t,
      v_ego=float(cs.vEgo),
      a_ego=float(cs.aEgo),
      pitch=pitch,
      roll=roll,
      v_forward=v_forward,
      long_active=bool(cc.longActive),
      brake_pressed=bool(cs.brakePressed),
      gas_pressed=bool(cs.gasPressed),
      standstill=bool(cs.standstill),
      should_stop=bool(lp.shouldStop),
      fcw=bool(lp.fcw),
    )


if __name__ == "__main__":
  main()
