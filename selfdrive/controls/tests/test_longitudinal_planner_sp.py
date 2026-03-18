import numpy as np

from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner


def test_icbm_op_long_target_cruise():
  target = LongitudinalPlanner.get_icbm_op_long_target(np.array([27.0, 27.0, 27.0]), 26.5, False)
  assert target == 26.5


def test_icbm_op_long_target_stop_intent():
  target = LongitudinalPlanner.get_icbm_op_long_target(np.array([12.0, 8.0, 4.0]), 3.5, True)
  assert target == 0.0
