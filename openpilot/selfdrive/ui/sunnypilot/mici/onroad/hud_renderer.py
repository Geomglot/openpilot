"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import pyray as rl

from opendbc.car.rivian.values import RivianFlags
from openpilot.common.params import Params
from openpilot.selfdrive.ui.mici.onroad.hud_renderer import HudRenderer
from openpilot.selfdrive.ui.sunnypilot.onroad.blind_spot_indicators import BlindSpotIndicators
from openpilot.selfdrive.ui.sunnypilot.onroad.lane_centering_indicator import LaneCenteringIndicator
from openpilot.selfdrive.ui.sunnypilot.onroad.lateral_mode import lateral_mode
from openpilot.selfdrive.ui.sunnypilot.onroad.model_speed_button import ModelSpeedButton, MODEL_OFF_COLOR
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import MousePos

# tap target over the MAX box in the top-left corner (the box is about 162 px wide and ends near 145 px down)
MAX_TAP_WIDTH = 162
MAX_TAP_HEIGHT = 150
# a tap switches Model Controls Speed only once the MAX box has been on screen this long, so a quick
# double tap on a hidden box (or a tap just after the switch) only shows it
SWITCH_AFTER_SHOWN_S = 0.5
MAX_SHOWING_ALPHA = 0.5


class HudRendererSP(HudRenderer):
  def __init__(self):
    super().__init__()
    self.blind_spot_indicators = BlindSpotIndicators()
    self.lane_centering_indicator = LaneCenteringIndicator(compact=True)
    # comma 4 angle/torque tap toggle on the wheel icon (backend is device-agnostic; this is the trigger)
    self._params = Params()
    self._angle_tap_armed = False
    self._sp_engaged: bool = False
    # comma 4 Model Controls Speed switch on the MAX box: first tap shows the box, a second tap switches
    self.model_speed_button = ModelSpeedButton()
    self._max_tap_armed = False
    self._max_was_showing = False
    self._hud_tap_consumed = False

  def _update_state(self) -> None:
    super()._update_state()
    self.blind_spot_indicators.update()
    self.lane_centering_indicator.update()
    lateral_mode.update()
    self.wheel_tint = lateral_mode.wheel_tint
    self.model_speed_button.refresh()
    self.set_speed_tint = MODEL_OFF_COLOR if self.model_speed_button.visible() and not self.model_speed_button.model_on else None

    sp_engaged = ui_state.engaged
    if sp_engaged != self._sp_engaged:
      self._chestnut_fade_time = rl.get_time() if sp_engaged else 0
    self._sp_engaged = sp_engaged

  def _render(self, rect: rl.Rectangle) -> None:
    super()._render(rect)
    self.blind_spot_indicators.render(rect)
    self.lane_centering_indicator.render(rect)

  def _has_blind_spot_detected(self) -> bool:

    return self.blind_spot_indicators.detected

  def _torque_toggle_ctx(self) -> bool:
    # C4 wheel is display-only for Experimental (no shared ExpButton), so unlike the big-UI path
    # the tap is NOT gated on Experimental mode. Allow whenever: angle-harness Rivian, master
    # switch on, and MADS actively steering.
    cp = ui_state.CP
    if cp is None or cp.brand != "rivian" or not (cp.flags & RivianFlags.ANGLE_HARNESS):
      return False
    if not self._params.get_bool("RivianEnableAngleSteering"):
      return False
    sm = ui_state.sm
    return sm.recv_frame["carControl"] >= ui_state.started_frame and sm["carControl"].latActive

  def _max_tap_rect(self) -> rl.Rectangle:
    return rl.Rectangle(self._rect.x, self._rect.y, MAX_TAP_WIDTH, MAX_TAP_HEIGHT)

  def _max_tap_ctx(self) -> bool:
    # only where the MAX box can be drawn: openpilot speed control, cruise set, engaged and no alert covering it
    return self.model_speed_button.visible() and self.is_cruise_set and self._engaged and self._can_draw_top_icons

  def _max_showing(self) -> bool:
    return (self._set_speed_alpha_filter.x > MAX_SHOWING_ALPHA and
            rl.get_time() - self._set_speed_changed_time >= SWITCH_AFTER_SHOWN_S)

  def _handle_mouse_press(self, mouse_pos: MousePos) -> None:
    # arm a wheel tap only if the press lands on the wheel and the toggle is currently allowed
    self._angle_tap_armed = (self._wheel_hit_rect is not None and
                             rl.check_collision_point_rec(mouse_pos, self._wheel_hit_rect) and
                             self._torque_toggle_ctx())
    # the wheel wins where the two targets touch
    self._max_tap_armed = (not self._angle_tap_armed and rl.check_collision_point_rec(mouse_pos, self._max_tap_rect()) and
                           self._max_tap_ctx())
    # judged at the press, so the switch needs the box to have been on screen when the driver touched it
    self._max_was_showing = self._max_tap_armed and self._max_showing()

  def _handle_mouse_release(self, mouse_pos: MousePos) -> None:
    # a genuine tap = press + release both on the target; act and consume the tap so the road view
    # does not also navigate to the home screen (checked via hud_tap_consumed()).
    if self._angle_tap_armed and self._wheel_hit_rect is not None and rl.check_collision_point_rec(mouse_pos, self._wheel_hit_rect):
      self._params.put_bool("RivianForceTorqueSteerReq", not self._params.get_bool("RivianForceTorqueSteerReq"))
      self._hud_tap_consumed = True
    elif self._max_tap_armed and rl.check_collision_point_rec(mouse_pos, self._max_tap_rect()):
      # first tap shows the box; a tap while it is showing switches the model (if fast enough etc).
      # Either way the box shows for its usual 2.5 s, in the new colour after a switch.
      if self._max_was_showing:
        self.model_speed_button.toggle()
        self.set_speed_tint = MODEL_OFF_COLOR if not self.model_speed_button.model_on else None
      self._set_speed_changed_time = rl.get_time()
      self._hud_tap_consumed = True
    self._angle_tap_armed = False
    self._max_tap_armed = False
    self._max_was_showing = False

  def hud_tap_consumed(self) -> bool:
    consumed, self._hud_tap_consumed = self._hud_tap_consumed, False
    return consumed
