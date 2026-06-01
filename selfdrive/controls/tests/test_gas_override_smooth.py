import unittest
from unittest.mock import patch

from openpilot.selfdrive.controls.lib.gas_override_smooth import (
  GasOverrideSmooth,
  RECOVERY_DURATION,
)


class TestGasOverrideSmooth(unittest.TestCase):
  def test_recovery_starts_on_gas_release(self):
    smooth = GasOverrideSmooth()
    with patch("openpilot.selfdrive.controls.lib.gas_override_smooth.time.monotonic", side_effect=[0.0, 0.0, 0.1]):
      smooth.update(False)
      self.assertFalse(smooth.in_recovery())
      smooth.update(True)
      self.assertFalse(smooth.in_recovery())
      smooth.update(False)
      self.assertTrue(smooth.in_recovery())
      self.assertFalse(smooth.allow_gasfactor_learning())

  def test_smooth_accel_ramps_up_after_release(self):
    smooth = GasOverrideSmooth()
    with patch("openpilot.selfdrive.controls.lib.gas_override_smooth.time.monotonic", return_value=0.0):
      smooth.update(True)
      smooth.update(False)
      out = smooth.smooth_accel(2.0, 0.0)
    self.assertGreater(out, 0.0)
    self.assertLess(out, 2.0)

  def test_gasfactor_decays_toward_one(self):
    smooth = GasOverrideSmooth()
    with patch("openpilot.selfdrive.controls.lib.gas_override_smooth.time.monotonic", return_value=0.0):
      smooth.update(True)
      smooth.update(False)
      decayed = smooth.decay_gasfactor(2.5)
    self.assertLess(decayed, 2.5)
    self.assertGreater(decayed, 1.0)

  def test_recovery_expires(self):
    smooth = GasOverrideSmooth()
    with patch("openpilot.selfdrive.controls.lib.gas_override_smooth.time.monotonic", side_effect=[0.0, 0.0, RECOVERY_DURATION + 0.1]):
      smooth.update(True)
      smooth.update(False)
      self.assertTrue(smooth.in_recovery())
      self.assertFalse(smooth.in_recovery())


if __name__ == "__main__":
  unittest.main()
