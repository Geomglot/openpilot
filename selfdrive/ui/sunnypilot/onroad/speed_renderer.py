"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from enum import IntEnum

import pyray as rl

from openpilot.common.constants import CV
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.selfdrive.ui.onroad.hud_renderer import FONT_SIZES, COLORS

# GPS/WHEEL trust badge tuning (compared in m/s)
MATCH_TOL_MS = 1.0 * CV.KPH_TO_MS       # <=1 kph (~0.6 mph): displayed speed counts as matching GPS
GPS_MIN_SPEED_MS = 10.0 * CV.KPH_TO_MS  # GPS speed unreliable below ~10 km/h. Deliberately far below the
                                        # learner's 8.0 m/s sample floor: that floor keeps the slow EWA's
                                        # samples pristine; the badge only needs GPS to be non-noisy.
GPS_ACC_MAX_MS = 0.5                    # max speedAccuracy (m/s) to treat GPS as healthy (matches the learner)


class SpeedSource(IntEnum):
  NONE = 0         # badge hidden (TrueVEgoUI off or speedo hidden)
  GPS_MATCH = 1    # green: displayed speed agrees with GPS
  WHEEL_DIFF = 2   # grey: GPS healthy but displayed speed differs
  WHEEL_NOGPS = 3  # outline: GPS unavailable / unreliable


class SpeedRenderer:
  def __init__(self):
    self.speed: float = 0.0
    self.v_ego_ms: float = 0.0
    self.v_ego_cluster_seen: bool = False

    # GPS/WHEEL trust badge state, with debounce to avoid GPS-noise flicker
    self.source: SpeedSource = SpeedSource.NONE
    self._source_candidate: SpeedSource = SpeedSource.NONE
    self._source_frames: int = 0
    self._debounce_frames: int = max(1, gui_app.target_fps // 2)

    self._font_bold: rl.Font = gui_app.font(FontWeight.BOLD)
    self._font_medium: rl.Font = gui_app.font(FontWeight.MEDIUM)

  def update(self) -> None:
    car_state = ui_state.sm['carState']
    v_ego_cluster = car_state.vEgoCluster
    self.v_ego_cluster_seen = self.v_ego_cluster_seen or v_ego_cluster != 0.0
    if self.v_ego_cluster_seen and not ui_state.true_v_ego_ui:
      v_ego = v_ego_cluster
    elif ui_state.live_speed_correction:
      offset_ms = ui_state.cruise_speed_offset_kph * CV.KPH_TO_MS
      v_ego = max(0.0, car_state.vEgo - offset_ms)
    else:
      v_ego = car_state.vEgo
    self.v_ego_ms = v_ego
    speed_conversion = CV.MS_TO_KPH if ui_state.is_metric else CV.MS_TO_MPH
    self.speed = max(0.0, v_ego * speed_conversion)

    self._update_source(v_ego)

  def _gps_speed(self) -> float | None:
    """Speed (m/s) from the first healthy GPS service, preferring external (ublox) like the learner."""
    sm = ui_state.sm
    for svc in ("gpsLocationExternal", "gpsLocation"):
      if not sm.valid[svc]:
        continue
      gps = sm[svc]
      if gps.speed > 0 and gps.speedAccuracy <= GPS_ACC_MAX_MS:
        return gps.speed
    return None

  def _update_source(self, disp_ms: float) -> None:
    # The badge only makes sense when the speedometer is showing wheel-derived ("true") speed.
    if not ui_state.true_v_ego_ui or ui_state.hide_v_ego_ui:
      raw = SpeedSource.NONE
    else:
      gps_speed = self._gps_speed()
      if gps_speed is None or disp_ms < GPS_MIN_SPEED_MS:
        raw = SpeedSource.WHEEL_NOGPS
      elif abs(disp_ms - gps_speed) <= MATCH_TOL_MS:
        raw = SpeedSource.GPS_MATCH
      else:
        raw = SpeedSource.WHEEL_DIFF

    self._commit_source(raw)

  def _commit_source(self, raw: SpeedSource) -> None:
    if raw == self.source:
      self._source_candidate = raw
      self._source_frames = 0
      return
    # Show/hide of the whole badge is immediate; only inter-state flips are debounced.
    if raw == SpeedSource.NONE or self.source == SpeedSource.NONE:
      self.source = raw
      self._source_candidate = raw
      self._source_frames = 0
      return
    if raw != self._source_candidate:
      self._source_candidate = raw
      self._source_frames = 0
    self._source_frames += 1
    if self._source_frames >= self._debounce_frames:
      self.source = raw
      self._source_frames = 0

  def render(self, rect: rl.Rectangle) -> None:
    if ui_state.hide_v_ego_ui:
      return

    # Draw current speed and unit
    speed_text = str(round(self.speed))
    speed_text_size = measure_text_cached(self._font_bold, speed_text, FONT_SIZES.current_speed)
    speed_pos = rl.Vector2(rect.x + rect.width / 2 - speed_text_size.x / 2, 180 - speed_text_size.y / 2)
    rl.draw_text_ex(self._font_bold, speed_text, speed_pos, FONT_SIZES.current_speed, 0, COLORS.WHITE)

    unit_text = tr("km/h") if ui_state.is_metric else tr("mph")
    unit_text_size = measure_text_cached(self._font_medium, unit_text, FONT_SIZES.speed_unit)
    unit_pos = rl.Vector2(rect.x + rect.width / 2 - unit_text_size.x / 2, 290 - unit_text_size.y / 2)
    rl.draw_text_ex(self._font_medium, unit_text, unit_pos, FONT_SIZES.speed_unit, 0, COLORS.WHITE_TRANSLUCENT)

    # GPS/WHEEL trust badge, to the right of the speed number
    self._render_source_badge(speed_pos, speed_text_size)

  def _render_source_badge(self, speed_pos: rl.Vector2, speed_text_size: rl.Vector2) -> None:
    if self.source == SpeedSource.NONE:
      return

    label = "GPS" if self.source == SpeedSource.GPS_MATCH else "WHEEL"
    font_size = 30
    pad_h, pad_v = 14, 6

    text_size = measure_text_cached(self._font_bold, label, font_size)
    box_w = text_size.x + pad_h * 2
    box_h = text_size.y + pad_v * 2
    box_x = speed_pos.x + speed_text_size.x + 16
    box_y = 180 - box_h / 2  # vertically centred on the speed number (centre y = 180)
    box = rl.Rectangle(box_x, box_y, box_w, box_h)

    if self.source == SpeedSource.GPS_MATCH:
      rl.draw_rectangle_rounded(box, 0.4, 10, rl.Color(0, 255, 0, 255))
      text_color = rl.Color(0, 0, 0, 255)
    elif self.source == SpeedSource.WHEEL_DIFF:
      rl.draw_rectangle_rounded(box, 0.4, 10, COLORS.GREY)
      text_color = rl.Color(0, 0, 0, 255)
    else:  # WHEEL_NOGPS: dimmed outline only, to distinguish "no GPS" from "GPS differs"
      rl.draw_rectangle_rounded_lines_ex(box, 0.4, 10, 2.0, COLORS.GREY)
      text_color = COLORS.GREY

    text_pos = rl.Vector2(box_x + pad_h, box_y + pad_v)
    rl.draw_text_ex(self._font_bold, label, text_pos, font_size, 0, text_color)
