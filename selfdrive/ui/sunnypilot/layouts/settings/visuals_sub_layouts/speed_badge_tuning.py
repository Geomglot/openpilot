"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from collections.abc import Callable

import pyray as rl
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.sunnypilot.widgets.list_view import option_item_sp
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.network import NavButton
from openpilot.system.ui.widgets.scroller_tici import Scroller


def _gps_acc_gate_label(v: int) -> str:
  # v is tenths of m/s (the GPS speedAccuracy gate); show it in the driver's speed unit.
  ms = v / 10.0
  return f"≤ {ms * 3.6:.1f} km/h" if ui_state.is_metric else f"≤ {ms * 2.23694:.1f} mph"


def _gps_match_tol_label(v: int) -> str:
  # v is tenths of km/h (the match-enter tolerance); show it in the driver's speed unit.
  kph = v / 10.0
  return f"± {kph:.1f} km/h" if ui_state.is_metric else f"± {kph * 0.621371:.1f} mph"


def _seconds_label(v: int) -> str:
  # v is tenths of a second (badge debounce / GPS hold). Unit-agnostic (time).
  s = v / 10.0
  return tr("Off") if s <= 0 else f"{s:.1f} s"


class SpeedBadgeTuningLayout(Widget):
  """Submenu holding the GPS/WHEEL speed-badge tuning controls (reached from Visuals)."""

  def __init__(self, back_btn_callback: Callable):
    super().__init__()
    self._back_button = NavButton(tr("Back"))
    self._back_button.set_click_callback(back_btn_callback)

    items = self._initialize_items()
    self._scroller = Scroller(items, line_separator=True, spacing=0)

  def _initialize_items(self):
    self._gps_acc_gate = option_item_sp(
      title=lambda: tr("GPS Accuracy Gate"),
      param="GpsBadgeSpeedAccMax",
      min_value=3, max_value=30, value_change_step=1,
      description=lambda: tr("Trust GPS for the speed-source badge only when its reported speed accuracy "
                             "is within this value. Raise it if the GPS badge rarely appears."),
      label_callback=_gps_acc_gate_label,
      inline=True,
    )
    self._gps_match_tol = option_item_sp(
      title=lambda: tr("GPS Match Tolerance"),
      param="GpsBadgeMatchTol",
      min_value=0, max_value=50, value_change_step=1,
      description=lambda: tr("Show the green GPS badge when the displayed speed is within this much of GPS. "
                             "The badge releases a little beyond this (hysteresis) to avoid flicker."),
      label_callback=_gps_match_tol_label,
      inline=True,
    )
    self._gps_debounce = option_item_sp(
      title=lambda: tr("Stability (Debounce)"),
      param="GpsBadgeDebounce",
      min_value=5, max_value=40, value_change_step=5,
      description=lambda: tr("How long a change must persist before the badge switches. Higher = calmer "
                             "badge (less flicker on hilly/bendy roads) but slower to react."),
      label_callback=_seconds_label,
      inline=True,
    )
    self._gps_hold = option_item_sp(
      title=lambda: tr("GPS Hold"),
      param="GpsBadgeGpsHold",
      min_value=0, max_value=100, value_change_step=10,
      description=lambda: tr("Keep using the last good GPS fix for this long through brief accuracy "
                             "dropouts, so the badge doesn't flicker to WHEEL where GPS is poor. 0 = off."),
      label_callback=_seconds_label,
      inline=True,
    )

    return [
      self._gps_acc_gate,
      self._gps_match_tol,
      self._gps_debounce,
      self._gps_hold,
    ]

  def _render(self, rect):
    self._back_button.set_position(self._rect.x, self._rect.y + 20)
    self._back_button.render()

    content_rect = rl.Rectangle(rect.x, rect.y + self._back_button.rect.height + 40,
                                rect.width, rect.height - self._back_button.rect.height - 40)
    self._scroller.render(content_rect)

  def show_event(self):
    self._scroller.show_event()

  def hide_event(self):
    self._scroller.hide_event()
