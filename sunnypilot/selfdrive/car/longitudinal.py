"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from cereal import custom, car
from openpilot.common.params import Params

ICBM_OPENPILOT_LONGITUDINAL = "ICBMOpenpilotLongitudinal"


def is_icbm_openpilot_longitudinal_enabled(CP: car.CarParams, CP_SP: custom.CarParamsSP,
                                           params: Params | None = None) -> bool:
  if params is None:
    params = Params()

  return bool(
    not CP.openpilotLongitudinalControl and
    CP_SP.intelligentCruiseButtonManagementAvailable and
    params.get_bool("IntelligentCruiseButtonManagement") and
    params.get_bool(ICBM_OPENPILOT_LONGITUDINAL)
  )


def has_longitudinal_planner_ownership(CP: car.CarParams, CP_SP: custom.CarParamsSP,
                                       params: Params | None = None) -> bool:
  return CP.openpilotLongitudinalControl or is_icbm_openpilot_longitudinal_enabled(CP, CP_SP, params)
