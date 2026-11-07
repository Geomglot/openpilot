"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import pyray as rl

from opendbc.car.rivian.values import RivianFlags
from openpilot.common.params import Params
from openpilot.selfdrive.ui.sunnypilot.onroad.lateral_mode import lateral_mode
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.widgets import Widget

# same size and spot as the wheel button that the Experimental Mode retirement removed
BUTTON_SIZE = 192
ICON_SIZE = 144
BORDER_SIZE = 30


def is_angle_capable_rivian() -> bool:
  cp = ui_state.CP
  return cp is not None and cp.brand == "rivian" and bool(cp.flags & RivianFlags.ANGLE_HARNESS)


class SteeringModeButton(Widget):
  """Top-right wheel on a Rivian with the angle harness: its colour shows angle (green) or torque (blue)
  steering, and a tap asks to switch between torque-only and torque/angle.

  On dev-a this lived inside the Experimental Mode button, which upstream deleted when it retired
  Experimental Mode. The tap only flips the request; CarController owns the switch itself.
  """

  def __init__(self):
    super().__init__()
    self._params = Params()
    self._black_bg: rl.Color = rl.Color(0, 0, 0, 166)
    self._txt_wheel: rl.Texture = gui_app.texture('icons/chffr_wheel.png', ICON_SIZE, ICON_SIZE)
    self._rect = rl.Rectangle(0, 0, BUTTON_SIZE, BUTTON_SIZE)

  @staticmethod
  def visible() -> bool:
    return is_angle_capable_rivian()

  @staticmethod
  def steering_active() -> bool:
    # MADS is actively steering, judged from a carControl of this drive
    sm = ui_state.sm
    return sm.recv_frame["carControl"] >= ui_state.started_frame and sm["carControl"].latActive

  def toggle_allowed(self) -> bool:
    # master switch off means torque-only, so the tap does nothing. Read on tap only, not every frame.
    return self.visible() and self.steering_active() and self._params.get_bool("RivianEnableAngleSteering")

  def _handle_mouse_release(self, mouse_pos) -> None:
    super()._handle_mouse_release(mouse_pos)
    if self.toggle_allowed():
      self._params.put_bool("RivianForceTorqueSteerReq", not self._params.get_bool("RivianForceTorqueSteerReq"))

  def place(self, hud_rect: rl.Rectangle) -> rl.Rectangle:
    return rl.Rectangle(hud_rect.x + hud_rect.width - BORDER_SIZE - BUTTON_SIZE, hud_rect.y + BORDER_SIZE, BUTTON_SIZE, BUTTON_SIZE)

  def _render(self, rect: rl.Rectangle) -> None:
    center_x = int(rect.x + rect.width // 2)
    center_y = int(rect.y + rect.height // 2)
    alpha = 180 if self.is_pressed or not self.steering_active() else 255

    tint = lateral_mode.wheel_tint
    color = rl.Color(tint.r, tint.g, tint.b, alpha) if tint is not None else rl.Color(255, 255, 255, alpha)

    rl.draw_circle(center_x, center_y, rect.width / 2, self._black_bg)
    tex = self._txt_wheel
    rl.draw_texture_ex(tex, rl.Vector2(center_x - tex.width / 2, center_y - tex.height / 2), 0.0, 1.0, color)
