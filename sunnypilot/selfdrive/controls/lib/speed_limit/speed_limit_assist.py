"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import time

from cereal import custom, car
from openpilot.common.params import Params
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit import PCM_LONG_REQUIRED_MAX_SET_SPEED, CONFIRM_SPEED_THRESHOLD, SLA_DYNAMIC_OFFSET_LIMIT
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.common import Mode
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.helpers import compare_cluster_target, set_speed_limit_assist_availability

ButtonType = car.CarState.ButtonEvent.Type
EventNameSP = custom.OnroadEventSP.EventName
SpeedLimitAssistState = custom.LongitudinalPlanSP.SpeedLimit.AssistState
SpeedLimitSource = custom.LongitudinalPlanSP.SpeedLimit.Source

ACTIVE_STATES = (SpeedLimitAssistState.active, SpeedLimitAssistState.adapting)
ENABLED_STATES = (SpeedLimitAssistState.preActive, SpeedLimitAssistState.pending, *ACTIVE_STATES)

DISABLED_GUARD_PERIOD = 0.5  # secs.
# secs. Time to wait after activation before considering temp deactivation signal.
PRE_ACTIVE_GUARD_PERIOD = {
  True: 15,
  False: 5,
}
SPEED_LIMIT_CHANGED_HOLD_PERIOD = 1  # secs. Time to wait after speed limit change before switching to preActive.

LIMIT_MIN_ACC = -1.5  # m/s^2 Maximum deceleration allowed for limit controllers to provide.
LIMIT_MAX_ACC = 1.0   # m/s^2 Maximum acceleration allowed for limit controllers to provide while active.
LIMIT_MIN_SPEED = 8.33  # m/s, Minimum speed limit to provide as solution on limit controllers.
LIMIT_SPEED_OFFSET_TH = -1.  # m/s Maximum offset between speed limit and current speed for adapting state.
V_CRUISE_UNSET = 255.

CRUISE_BUTTONS_PLUS = (ButtonType.accelCruise, ButtonType.resumeCruise)
CRUISE_BUTTONS_MINUS = (ButtonType.decelCruise, ButtonType.setCruise)
CRUISE_BUTTON_CONFIRM_HOLD = 0.5  # secs.
COMBO_SEQUENCE = ("plus", "plus", "minus", "minus")
COMBO_MAX_WINDOW = 2.0  # secs.
DRIVER_SET_SPEED_CHANGE_WINDOW = 1.0  # secs.


class SpeedLimitAssist:
  _speed_limit_final_last: float
  _distance: float
  v_ego: float
  a_ego: float
  v_offset: float

  def __init__(self, CP: car.CarParams, CP_SP: custom.CarParamsSP):
    self.params = Params()
    self.CP = CP
    self.CP_SP = CP_SP
    self.frame = -1
    self.long_engaged_timer = 0
    self.pre_active_timer = 0
    self.is_metric = self.params.get_bool("IsMetric")
    set_speed_limit_assist_availability(self.CP, self.CP_SP, self.params)
    self.enabled = self.params.get("SpeedLimitMode", return_default=True) == Mode.assist
    self.long_enabled = False
    self.long_enabled_prev = False
    self.is_enabled = False
    self.is_active = False
    self.output_v_target = V_CRUISE_UNSET
    self.output_a_target = 0.
    self.v_ego = 0.
    self.a_ego = 0.
    self.v_offset = 0.
    self.target_set_speed_conv = 0
    self.prev_target_set_speed_conv = 0
    self.v_cruise_cluster = 0.
    self.v_cruise_cluster_prev = 0.
    self.v_cruise_cluster_conv = 0
    self.prev_v_cruise_cluster_conv = 0
    self._has_speed_limit = False
    self._speed_limit = 0.
    self._speed_limit_final_last = 0.
    self.speed_limit_prev = 0.
    self.speed_limit_final_last_conv = 0
    self.prev_speed_limit_final_last_conv = 0
    self._distance = 0.
    self.state = SpeedLimitAssistState.disabled
    self._state_prev = SpeedLimitAssistState.disabled
    self.pcm_op_long = CP.openpilotLongitudinalControl and CP.pcmCruise

    self._plus_hold = 0.
    self._minus_hold = 0.
    self._last_carstate_ts = 0.
    self._combo_start_ts = 0.
    self._combo_progress = 0
    self._combo_completed = False
    self._combo_failed_no_speed_limit = False
    self._last_set_speed_button_ts = 0.
    self.dynamic_offset = 0.

    # TODO-SP: SLA's own output_a_target for planner
    # Solution functions mapped to respective states
    self.acceleration_solutions = {
      SpeedLimitAssistState.disabled: self.get_current_acceleration_as_target,
      SpeedLimitAssistState.inactive: self.get_current_acceleration_as_target,
      SpeedLimitAssistState.preActive: self.get_current_acceleration_as_target,
      SpeedLimitAssistState.pending: self.get_current_acceleration_as_target,
      SpeedLimitAssistState.adapting: self.get_adapting_state_target_acceleration,
      SpeedLimitAssistState.active: self.get_active_state_target_acceleration,
    }

  @property
  def speed_limit_changed(self) -> bool:
    return self._has_speed_limit and bool(self._speed_limit != self.speed_limit_prev)

  @property
  def v_cruise_cluster_changed(self) -> bool:
    return bool(self.v_cruise_cluster_conv != self.prev_v_cruise_cluster_conv)

  @property
  def target_set_speed_confirmed(self) -> bool:
    return bool(self.v_cruise_cluster_conv == self.target_set_speed_conv)

  @property
  def v_cruise_cluster_below_confirm_speed_threshold(self) -> bool:
    return bool(self.v_cruise_cluster_conv < CONFIRM_SPEED_THRESHOLD[self.is_metric])

  def update_active_event(self, events_sp: EventsSP) -> None:
    events_sp.add(EventNameSP.speedLimitActive)

  def get_v_target_from_control(self) -> float:
    if self._has_speed_limit and self.is_active:
      return self._get_non_pcm_target_speed()

    # Fallback
    return V_CRUISE_UNSET

  # TODO-SP: SLA's own output_a_target for planner
  def get_a_target_from_control(self) -> float:
    return self.a_ego

  def update_params(self) -> None:
    if self.frame % int(PARAMS_UPDATE_PERIOD / DT_MDL) == 0:
      self.is_metric = self.params.get_bool("IsMetric")
      set_speed_limit_assist_availability(self.CP, self.CP_SP, self.params)
      self.enabled = self.params.get("SpeedLimitMode", return_default=True) == Mode.assist

  def update_car_state(self, CS: car.CarState) -> None:
    now = time.monotonic()
    self._last_carstate_ts = now

    for b in CS.buttonEvents:
      if not b.pressed:
        self._update_combo_progress(b.type, now)
        if b.type in CRUISE_BUTTONS_PLUS:
          self._plus_hold = max(self._plus_hold, now + CRUISE_BUTTON_CONFIRM_HOLD)
        elif b.type in CRUISE_BUTTONS_MINUS:
          self._minus_hold = max(self._minus_hold, now + CRUISE_BUTTON_CONFIRM_HOLD)

        if self.long_enabled and self.enabled and (b.type in CRUISE_BUTTONS_PLUS or b.type in CRUISE_BUTTONS_MINUS):
          self._last_set_speed_button_ts = now

  def _reset_combo_progress(self) -> None:
    self._combo_start_ts = 0.
    self._combo_progress = 0

  def _button_direction(self, button_type: car.CarState.ButtonEvent.Type) -> str | None:
    if button_type in CRUISE_BUTTONS_PLUS:
      return "plus"
    if button_type in CRUISE_BUTTONS_MINUS:
      return "minus"
    return None

  def _update_combo_progress(self, button_type: car.CarState.ButtonEvent.Type, now: float) -> None:
    if not self.long_enabled or not self.enabled:
      return

    direction = self._button_direction(button_type)
    if direction is None:
      return

    expected = COMBO_SEQUENCE[self._combo_progress] if self._combo_progress < len(COMBO_SEQUENCE) else None
    timed_out = self._combo_progress > 0 and (now - self._combo_start_ts > COMBO_MAX_WINDOW)

    if timed_out:
      self._reset_combo_progress()
      expected = COMBO_SEQUENCE[0]

    if self._combo_progress == 0:
      if direction == COMBO_SEQUENCE[0]:
        self._combo_start_ts = now
        self._combo_progress = 1
      return

    if direction == expected:
      self._combo_progress += 1
    else:
      self._reset_combo_progress()
      if direction == COMBO_SEQUENCE[0]:
        self._combo_start_ts = now
        self._combo_progress = 1
      return

    if self._combo_progress == len(COMBO_SEQUENCE):
      if now - self._combo_start_ts <= COMBO_MAX_WINDOW:
        self._combo_completed = True
      self._reset_combo_progress()

  def _get_button_release(self, req_plus: bool, req_minus: bool) -> bool:
    now = time.monotonic()
    if req_plus and now <= self._plus_hold:
      self._plus_hold = 0.
      return True
    elif req_minus and now <= self._minus_hold:
      self._minus_hold = 0.
      return True

    # expired
    if now > self._plus_hold:
      self._plus_hold = 0.
    if now > self._minus_hold:
      self._minus_hold = 0.
    return False

  def update_calculations(self, v_cruise_cluster: float) -> None:
    speed_conv = CV.MS_TO_KPH if self.is_metric else CV.MS_TO_MPH
    self.v_cruise_cluster = v_cruise_cluster

    # Update current velocity offset (error)
    self.v_offset = self.get_v_target_from_control() - self.v_ego if self.is_active else self._speed_limit_final_last - self.v_ego

    self.speed_limit_final_last_conv = round(self._speed_limit_final_last * speed_conv)
    self.v_cruise_cluster_conv = round(self.v_cruise_cluster * speed_conv)

    cst_low, cst_high = PCM_LONG_REQUIRED_MAX_SET_SPEED[self.is_metric]
    pcm_long_required_max = cst_low if self._has_speed_limit and self.speed_limit_final_last_conv < CONFIRM_SPEED_THRESHOLD[self.is_metric] else \
                            cst_high
    pcm_long_required_max_set_speed_conv = round(pcm_long_required_max * speed_conv)

    self.target_set_speed_conv = pcm_long_required_max_set_speed_conv if self.pcm_op_long else self.speed_limit_final_last_conv

  @property
  def apply_confirm_speed_threshold(self) -> bool:
    # below CST: always require user confirmation
    if self.v_cruise_cluster_below_confirm_speed_threshold:
      return True

    # at/above CST:
    # - new speed limit >= CST: auto change
    # - new speed limit < CST: user confirmation required
    return bool(self.speed_limit_final_last_conv < CONFIRM_SPEED_THRESHOLD[self.is_metric])

  def get_current_acceleration_as_target(self) -> float:
    return self.a_ego

  def get_adapting_state_target_acceleration(self) -> float:
    if self._distance > 0:
      return (self._speed_limit_final_last ** 2 - self.v_ego ** 2) / (2. * self._distance)

    return self.v_offset / float(ModelConstants.T_IDXS[CONTROL_N])

  def get_active_state_target_acceleration(self) -> float:
    return self.v_offset / float(ModelConstants.T_IDXS[CONTROL_N])

  def _update_confirmed_state(self):
    if self._has_speed_limit:
      v_target = self._get_non_pcm_target_speed()
      self.v_offset = v_target - self.v_ego
      if self.v_offset < LIMIT_SPEED_OFFSET_TH:
        self.state = SpeedLimitAssistState.adapting
      else:
        self.state = SpeedLimitAssistState.active
    else:
      self.state = SpeedLimitAssistState.pending

  def _get_dynamic_offset_limit(self) -> float:
    return SLA_DYNAMIC_OFFSET_LIMIT[self.is_metric]

  def _get_non_pcm_target_speed(self) -> float:
    if not self._has_speed_limit:
      return 0.
    return max(0., self._speed_limit_final_last + self.dynamic_offset)

  def _set_dynamic_offset_from_set_speed(self, base_speed_limit: float) -> bool:
    if base_speed_limit <= 0.:
      return False
    requested_offset = self.v_cruise_cluster - base_speed_limit
    if abs(requested_offset) > self._get_dynamic_offset_limit():
      self.dynamic_offset = 0.
      return False
    self.dynamic_offset = requested_offset
    return True

  def _consume_recent_driver_set_speed_change(self) -> bool:
    now = time.monotonic()
    is_recent = self._last_set_speed_button_ts > 0. and (now - self._last_set_speed_button_ts) <= DRIVER_SET_SPEED_CHANGE_WINDOW
    if is_recent:
      self._last_set_speed_button_ts = 0.
      return True
    return False

  def _update_non_pcm_long_confirmed_state(self) -> bool:
    if self.target_set_speed_confirmed:
      return True

    if self.state != SpeedLimitAssistState.preActive:
      return False

    req_plus, req_minus = compare_cluster_target(self.v_cruise_cluster, self._speed_limit_final_last, self.is_metric)

    return self._get_button_release(req_plus, req_minus)

  def update_state_machine_pcm_op_long(self):
    # PCM now follows the same SLA engagement/disengagement and dynamic-offset behavior as non-PCM.
    return self.update_state_machine_non_pcm_long()

  def update_state_machine_non_pcm_long(self):
    self._combo_failed_no_speed_limit = False

    if not self.long_enabled or not self.enabled:
      self.state = SpeedLimitAssistState.disabled
      self.dynamic_offset = 0.
      self._reset_combo_progress()
      self._combo_completed = False
    else:
      if not self.long_enabled_prev:
        # Always disengage SLA on each cruise-control engagement.
        self.state = SpeedLimitAssistState.inactive
        self.dynamic_offset = 0.
        self._reset_combo_progress()
        self._combo_completed = False

      if self._combo_completed:
        self._combo_completed = False
        if self._speed_limit > 0. and self._set_dynamic_offset_from_set_speed(self._speed_limit):
          self._update_confirmed_state()
        else:
          self.state = SpeedLimitAssistState.inactive
          self._combo_failed_no_speed_limit = self._speed_limit <= 0.

      elif self.state in ACTIVE_STATES:
        # Recompute dynamic offset whenever driver set speed changes while SLA is engaged.
        if self.v_cruise_cluster_changed and self._consume_recent_driver_set_speed_change():
          if not self._set_dynamic_offset_from_set_speed(self._speed_limit_final_last):
            self.state = SpeedLimitAssistState.inactive
          else:
            self._update_confirmed_state()
        else:
          self._update_confirmed_state()
      else:
        self.state = SpeedLimitAssistState.inactive

    enabled = self.state in ENABLED_STATES
    active = self.state in ACTIVE_STATES

    return enabled, active

  def update_events(self, events_sp: EventsSP) -> None:
    if self._combo_failed_no_speed_limit:
      events_sp.add(EventNameSP.speedLimitPreActive)

    if self.state == SpeedLimitAssistState.preActive:
      events_sp.add(EventNameSP.speedLimitPreActive)

    if self.state == SpeedLimitAssistState.pending and self._state_prev != SpeedLimitAssistState.pending:
      events_sp.add(EventNameSP.speedLimitPending)

    if self.is_active:
      if self._state_prev not in ACTIVE_STATES:
        events_sp.add(EventNameSP.speedLimitActive)
      elif self._speed_limit != self.speed_limit_prev and self._speed_limit > 0:
        # Always notify (and chime) when an updated speed limit is detected while SLA is active.
        events_sp.add(EventNameSP.speedLimitChanged)

  def update(self, long_enabled: bool, long_override: bool, v_ego: float, a_ego: float, v_cruise_cluster: float, speed_limit: float,
             speed_limit_final_last: float, has_speed_limit: bool, distance: float, events_sp: EventsSP) -> None:
    self.long_enabled = long_enabled
    self.v_ego = v_ego
    self.a_ego = a_ego

    self._has_speed_limit = has_speed_limit
    self._speed_limit = speed_limit
    self._speed_limit_final_last = speed_limit_final_last
    self._distance = distance

    self.update_params()
    self.update_calculations(v_cruise_cluster)

    self._state_prev = self.state
    if self.pcm_op_long:
      self.is_enabled, self.is_active = self.update_state_machine_pcm_op_long()
    else:
      self.is_enabled, self.is_active = self.update_state_machine_non_pcm_long()

    self.update_events(events_sp)

    # Update change tracking variables
    self.speed_limit_prev = self._speed_limit
    self.v_cruise_cluster_prev = self.v_cruise_cluster
    self.long_enabled_prev = self.long_enabled
    self.prev_target_set_speed_conv = self.target_set_speed_conv
    self.prev_v_cruise_cluster_conv = self.v_cruise_cluster_conv
    self.prev_speed_limit_final_last_conv = self.speed_limit_final_last_conv

    self.output_v_target = self.get_v_target_from_control()
    self.output_a_target = self.get_a_target_from_control()

    self.frame += 1
