"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import time

import pyray as rl

from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.widgets import Widget

# set-speed number while "Model Controls Speed" is off
MODEL_OFF_COLOR = rl.Color(0x7d, 0xc8, 0xff, 0xff)
MODEL_OFF_DIM_COLOR = rl.Color(0x7d, 0xc8, 0xff, 0x80)

# a tap is ignored below these, so switching the model out can never set the car off from a stop
TAP_MIN_SPEED_MPH = 20
TAP_MIN_SPEED_KPH = 30
PARAM_REFRESH_S = 1.0


def tap_min_speed_ms() -> float:
  return TAP_MIN_SPEED_KPH * CV.KPH_TO_MS if ui_state.is_metric else TAP_MIN_SPEED_MPH * CV.MPH_TO_MS


class ModelSpeedButton(Widget):
  """Invisible tap target over the MAX box: a tap switches "Model Controls Speed" while driving.

  It draws nothing itself; the HUD colours the set speed from model_on. The planner picks the change up within
  half a second and eases it in.
  """

  def __init__(self):
    super().__init__()
    self._params = Params()
    self.model_on = self._read()
    self._last_read = time.monotonic()

  def _read(self) -> bool:
    # an unwritten key reads as on, the registered default
    return bool(self._params.get("ModelControlsSpeed", return_default=True))

  @staticmethod
  def visible() -> bool:
    return ui_state.has_longitudinal_control

  @staticmethod
  def fast_enough() -> bool:
    sm = ui_state.sm
    return sm.recv_frame["carState"] >= ui_state.started_frame and sm["carState"].vEgo >= tap_min_speed_ms()

  def tap_allowed(self) -> bool:
    return self.visible() and self.fast_enough()

  def _update_state(self) -> None:
    # pick up a change made in the settings while parked, or on another drive
    now = time.monotonic()
    if now - self._last_read >= PARAM_REFRESH_S:
      self._last_read = now
      self.model_on = self._read()

  def _handle_mouse_release(self, mouse_pos) -> None:
    super()._handle_mouse_release(mouse_pos)
    if self.tap_allowed():
      self.model_on = not self.model_on
      self._params.put_bool("ModelControlsSpeed", self.model_on)
      self._last_read = time.monotonic()

  def _render(self, rect: rl.Rectangle) -> None:
    pass
