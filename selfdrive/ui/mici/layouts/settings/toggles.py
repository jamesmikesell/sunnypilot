from cereal import log

from openpilot.system.ui.widgets.scroller import NavScroller
from openpilot.selfdrive.ui.mici.widgets.button import BigParamControl, BigMultiParamToggle
from openpilot.selfdrive.ui.sunnypilot.mici.widgets.button import BigPolygonMultiParamToggle
from openpilot.sunnypilot.selfdrive.car.longitudinal import ICBM_OPENPILOT_LONGITUDINAL
from openpilot.system.ui.lib.application import gui_app
from openpilot.selfdrive.ui.layouts.settings.common import restart_needed_callback
from openpilot.selfdrive.ui.ui_state import ui_state

PERSONALITY_TO_INT = log.LongitudinalPersonality.schema.enumerants


class TogglesLayoutMici(NavScroller):
  def __init__(self):
    super().__init__()

    self._personality_toggle = BigMultiParamToggle("driving personality", "LongitudinalPersonality", ["aggressive", "standard", "relaxed"])
    self._experimental_btn = BigParamControl("experimental mode", "ExperimentalMode")
    self._speed_limit_mode_toggle = BigMultiParamToggle("sla mode", "SpeedLimitMode", ["off", "info", "warning", "assist"],
                                                       select_callback=self._on_speed_limit_mode_changed)
    self._speed_limit_source_toggle = BigPolygonMultiParamToggle("sla source", "SpeedLimitPolicy",
                                                                 ["car only", "map only", "car first", "map first", "combined"])
    icbm_toggle = BigParamControl("icbm", "IntelligentCruiseButtonManagement")
    self._icbm_op_long_toggle = BigParamControl("op long target for icbm", ICBM_OPENPILOT_LONGITUDINAL)
    scc_vision_toggle = BigParamControl("smart cruise - vision", "SmartCruiseControlVision")
    scc_map_toggle = BigParamControl("smart cruise - map", "SmartCruiseControlMap")
    is_metric_toggle = BigParamControl("use metric units", "IsMetric")
    ldw_toggle = BigParamControl("lane departure warnings", "IsLdwEnabled")
    always_on_dm_toggle = BigParamControl("always-on driver monitor", "AlwaysOnDM")
    record_front = BigParamControl("record & upload driver camera", "RecordFront", toggle_callback=restart_needed_callback)
    record_mic = BigParamControl("record & upload mic audio", "RecordAudio", toggle_callback=restart_needed_callback)
    enable_openpilot = BigParamControl("enable sunnypilot", "OpenpilotEnabledToggle", toggle_callback=restart_needed_callback)

    self._scroller.add_widgets([
      self._personality_toggle,
      self._experimental_btn,
      icbm_toggle,
      self._icbm_op_long_toggle,
      self._speed_limit_mode_toggle,
      self._speed_limit_source_toggle,
      scc_vision_toggle,
      scc_map_toggle,
      is_metric_toggle,
      ldw_toggle,
      always_on_dm_toggle,
      record_front,
      record_mic,
      enable_openpilot,
    ])

    # Toggle lists
    self._refresh_toggles = (
      ("ExperimentalMode", self._experimental_btn),
      ("IsMetric", is_metric_toggle),
      ("IsLdwEnabled", ldw_toggle),
      ("AlwaysOnDM", always_on_dm_toggle),
      ("RecordFront", record_front),
      ("RecordAudio", record_mic),
      ("SmartCruiseControlMap", scc_map_toggle),
      ("SmartCruiseControlVision", scc_vision_toggle),
      ("IntelligentCruiseButtonManagement", icbm_toggle),
      (ICBM_OPENPILOT_LONGITUDINAL, self._icbm_op_long_toggle),
      ("OpenpilotEnabledToggle", enable_openpilot),
    )

    enable_openpilot.set_enabled(lambda: not ui_state.engaged)
    record_front.set_enabled(False if ui_state.params.get_bool("RecordFrontLock") else (lambda: not ui_state.engaged))
    record_mic.set_enabled(lambda: not ui_state.engaged)

    if ui_state.params.get_bool("ShowDebugInfo"):
      gui_app.set_show_touches(True)
      gui_app.set_show_fps(True)

    ui_state.add_engaged_transition_callback(self._update_toggles)

  def _update_state(self):
    super()._update_state()

    if ui_state.sm.updated["selfdriveState"]:
      personality = PERSONALITY_TO_INT[ui_state.sm["selfdriveState"].personality]
      if personality != ui_state.personality and ui_state.started:
        self._personality_toggle.set_value(self._personality_toggle._options[personality])
      ui_state.personality = personality

  def show_event(self):
    super().show_event()
    self._update_toggles()

  def _update_speed_limit_source_enabled(self, mode_idx: int | None = None):
    if mode_idx is None:
      mode_idx = ui_state.params.get("SpeedLimitMode", return_default=True)
      mode_idx = max(0, min(mode_idx, len(self._speed_limit_mode_toggle._options) - 1))
    self._speed_limit_source_toggle.set_enabled(mode_idx != 0)

  def _on_speed_limit_mode_changed(self, value: str):
    mode_idx = self._speed_limit_mode_toggle._options.index(value)
    self._update_speed_limit_source_enabled(mode_idx)

  def _update_toggles(self):
    ui_state.update_params()

    # CP gating for experimental mode
    if ui_state.CP is not None:
      if ui_state.has_longitudinal_control or ui_state.has_icbm_openpilot_long:
        self._experimental_btn.set_visible(True)
        self._personality_toggle.set_visible(True)
      else:
        # no long for now
        self._experimental_btn.set_visible(False)
        self._experimental_btn.set_checked(False)
        self._personality_toggle.set_visible(False)
        ui_state.params.remove("ExperimentalMode")

    icbm_op_long_available = ui_state.CP is not None and ui_state.CP_SP is not None and \
                             ui_state.CP_SP.intelligentCruiseButtonManagementAvailable and not ui_state.has_longitudinal_control
    self._icbm_op_long_toggle.set_visible(icbm_op_long_available)
    if icbm_op_long_available:
      self._icbm_op_long_toggle.set_enabled(ui_state.params.get_bool("IntelligentCruiseButtonManagement"))
    else:
      ui_state.params.remove(ICBM_OPENPILOT_LONGITUDINAL)

    # Refresh toggles from params to mirror external changes
    for key, item in self._refresh_toggles:
      item.set_checked(ui_state.params.get_bool(key))

    mode_idx = ui_state.params.get("SpeedLimitMode", return_default=True)
    mode_idx = max(0, min(mode_idx, len(self._speed_limit_mode_toggle._options) - 1))
    self._speed_limit_mode_toggle.set_value(self._speed_limit_mode_toggle._options[mode_idx])
    self._update_speed_limit_source_enabled(mode_idx)

    source_idx = ui_state.params.get("SpeedLimitPolicy", return_default=True)
    source_idx = max(0, min(source_idx, len(self._speed_limit_source_toggle._options) - 1))
    self._speed_limit_source_toggle.set_value(self._speed_limit_source_toggle._options[source_idx])
