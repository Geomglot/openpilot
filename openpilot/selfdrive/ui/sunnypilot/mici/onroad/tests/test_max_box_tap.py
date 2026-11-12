"""The comma 4 MAX box as the "Model Controls Speed" switch: first tap shows the box, a tap while it shows switches.

The HUD is built without a window, so nothing is drawn. The clock is a stand-in for rl.get_time, the box's fade is
set directly on its filter, and Params is a dict with the same get/put_bool calls.
"""
from types import SimpleNamespace

import pytest

pytest.importorskip("pyray")

import pyray as rl

from openpilot.cereal import messaging
from openpilot.common.constants import CV
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.selfdrive.ui.sunnypilot.mici.onroad import hud_renderer as hud_mod
from openpilot.selfdrive.ui.sunnypilot.mici.onroad.hud_renderer import HudRendererSP
from openpilot.selfdrive.ui.sunnypilot.onroad.model_speed_button import MODEL_OFF_COLOR, ModelSpeedButton
from openpilot.selfdrive.ui.ui_state import ui_state

ON_BOX = rl.Vector2(60, 60)
OFF_BOX = rl.Vector2(300, 120)


class DictParams:
  def __init__(self, **values):
    self.values = dict(values)

  def get(self, key, return_default=False):
    if key not in self.values:
      return True if return_default else None
    return self.values[key]

  def get_bool(self, key):
    return bool(self.values.get(key, False))

  def put_bool(self, key, value):
    self.values[key] = bool(value)


class FakeSubMaster:
  def __init__(self, v_ego):
    msg = messaging.new_message('carState')
    msg.carState.vEgo = v_ego
    self._cs = msg.as_reader().carState
    self.recv_frame = {"carState": 10}

  def __getitem__(self, s):
    assert s == "carState"
    return self._cs


@pytest.fixture
def clock(monkeypatch):
  now = [100.0]
  monkeypatch.setattr(hud_mod.rl, "get_time", lambda: now[0])
  return now


@pytest.fixture
def drive(monkeypatch):
  def _set(mph=40.0, has_long=True):
    monkeypatch.setattr(ui_state, "has_longitudinal_control", has_long)
    monkeypatch.setattr(ui_state, "is_metric", False)
    monkeypatch.setattr(ui_state, "sm", FakeSubMaster(mph * CV.MPH_TO_MS))
    monkeypatch.setattr(ui_state, "started_frame", 5)
  _set()
  return _set


def _hud(model_on=True, showing_for=None, clock_now=100.0):
  """showing_for: seconds the MAX box has been on screen, or None for hidden."""
  hud = HudRendererSP.__new__(HudRendererSP)
  hud._rect = rl.Rectangle(0, 0, 536, 240)
  hud.is_cruise_set = True
  hud._engaged = True
  hud._can_draw_top_icons = True
  hud._wheel_hit_rect = None
  hud._angle_tap_armed = False
  hud._max_tap_armed = False
  hud._max_was_showing = False
  hud._hud_tap_consumed = False
  hud.set_speed_tint = None if model_on else MODEL_OFF_COLOR
  hud._set_speed_alpha_filter = FirstOrderFilter(0.0, 0.1, 0.05)
  if showing_for is None:
    hud._set_speed_changed_time = 0.0
  else:
    hud._set_speed_alpha_filter.x = 1.0
    hud._set_speed_changed_time = clock_now - showing_for

  btn = ModelSpeedButton.__new__(ModelSpeedButton)
  btn._params = DictParams(ModelControlsSpeed=model_on)
  btn.model_on = model_on
  btn._last_read = 0.0
  hud.model_speed_button = btn
  return hud


def _tap(hud, press=ON_BOX, release=None):
  hud._handle_mouse_press(press)
  hud._handle_mouse_release(release if release is not None else press)


def _model_on(hud):
  return hud.model_speed_button._params.values["ModelControlsSpeed"]


def test_first_tap_on_a_hidden_box_only_shows_it(clock, drive):
  hud = _hud()
  _tap(hud)
  assert _model_on(hud)
  assert hud._set_speed_changed_time == clock[0]
  assert hud.hud_tap_consumed()
  assert not hud.hud_tap_consumed()  # read once


def test_tap_while_the_box_shows_switches_and_turns_it_light_blue(clock, drive):
  hud = _hud(showing_for=1.0)
  _tap(hud)
  assert not _model_on(hud)
  assert hud.set_speed_tint == MODEL_OFF_COLOR
  assert hud._set_speed_changed_time == clock[0]  # stays up to show the new colour
  assert hud.hud_tap_consumed()


def test_tap_switches_back_on_and_clears_the_colour(clock, drive):
  hud = _hud(model_on=False, showing_for=1.0)
  _tap(hud)
  assert _model_on(hud)
  assert hud.set_speed_tint is None


def test_double_tap_on_a_hidden_box_does_not_switch(clock, drive):
  hud = _hud()
  _tap(hud)
  hud._set_speed_alpha_filter.x = 1.0  # fully faded in already
  clock[0] += 0.3
  _tap(hud)
  assert _model_on(hud)


def test_second_tap_after_half_a_second_switches(clock, drive):
  hud = _hud()
  _tap(hud)
  hud._set_speed_alpha_filter.x = 1.0
  clock[0] += 0.6
  _tap(hud)
  assert not _model_on(hud)


def test_too_slow_still_shows_the_box_but_does_not_switch(clock, drive):
  drive(mph=19.5)
  hud = _hud(showing_for=1.0)
  _tap(hud)
  assert _model_on(hud)
  assert hud.set_speed_tint is None
  assert hud.hud_tap_consumed()


@pytest.mark.parametrize("why, attr, value", [
  ("not engaged", "_engaged", False),
  ("cruise not set", "is_cruise_set", False),
  ("an alert covers the box", "_can_draw_top_icons", False),
])
def test_tap_passes_through_where_no_box_can_show(clock, drive, why, attr, value):
  hud = _hud(showing_for=1.0)
  setattr(hud, attr, value)
  _tap(hud)
  assert _model_on(hud), why
  assert not hud.hud_tap_consumed(), why


def test_tap_passes_through_without_openpilot_speed_control(clock, drive):
  drive(has_long=False)
  hud = _hud(showing_for=1.0)
  _tap(hud)
  assert _model_on(hud)
  assert not hud.hud_tap_consumed()


def test_tap_elsewhere_passes_through(clock, drive):
  hud = _hud(showing_for=1.0)
  _tap(hud, press=OFF_BOX)
  assert _model_on(hud)
  assert not hud.hud_tap_consumed()


def test_press_on_the_box_released_elsewhere_does_nothing(clock, drive):
  hud = _hud(showing_for=1.0)
  _tap(hud, press=ON_BOX, release=OFF_BOX)
  assert _model_on(hud)
  assert not hud.hud_tap_consumed()


def test_wheel_wins_where_the_targets_touch(clock, drive, monkeypatch):
  hud = _hud(showing_for=1.0)
  hud._params = DictParams(RivianForceTorqueSteerReq=False)
  hud._wheel_hit_rect = rl.Rectangle(0, 140, 100, 100)
  monkeypatch.setattr(HudRendererSP, "_torque_toggle_ctx", lambda self: True)
  _tap(hud, press=rl.Vector2(50, 145))
  assert _model_on(hud)
  assert hud._params.values["RivianForceTorqueSteerReq"] is True
  assert hud.hud_tap_consumed()



class BarsSubMaster:
  def __init__(self):
    msg = messaging.new_message('selfdriveState')
    msg.selfdriveState.personality = 'standard'
    self._ss = msg.as_reader().selfdriveState
    self._events = messaging.new_message('onroadEvents', 0).as_reader().onroadEvents

  def __getitem__(self, s):
    return {"selfdriveState": self._ss, "onroadEvents": self._events}[s]


@pytest.mark.parametrize("tint", [None, MODEL_OFF_COLOR])
def test_distance_bars_carry_the_set_speed_colour(clock, monkeypatch, tint):
  from openpilot.selfdrive.ui.mici.onroad import long_indicator as li_mod
  drawn = []
  monkeypatch.setattr(li_mod.rl, "draw_texture_ex", lambda tex, pos, rot, scale, color: drawn.append((tex.kind, (color.r, color.g, color.b))))
  monkeypatch.setattr(ui_state, "sm", BarsSubMaster())

  ind = li_mod.LongIndicator.__new__(li_mod.LongIndicator)
  ind._txt_distance = [(SimpleNamespace(kind="white", width=40, height=9), SimpleNamespace(kind="green", width=60, height=35)) for _ in range(3)]
  ind._distance_filters = [(FirstOrderFilter(1.0, 0.1, 0.05), FirstOrderFilter(0.0, 0.1, 0.05)) for _ in range(3)]
  ind._personality = 1
  ind._personality_changed_time = -10.0
  ind.set_bar_tint(tint)
  ind._draw_distance_bars(rl.Rectangle(0, 0, 536, 240), 1.0, True)

  want = (255, 255, 255) if tint is None else (tint.r, tint.g, tint.b)
  assert [rgb for kind, rgb in drawn if kind == "white"] == [want] * 3
  # the green personality highlight keeps its own colour
  assert all(rgb == (255, 255, 255) for kind, rgb in drawn if kind == "green")


@pytest.mark.parametrize("model_on", [True, False])
def test_max_box_draws_in_the_right_colour(clock, drive, monkeypatch, model_on):
  # draws for real down to the raylib calls, which are captured, so a bad colour value fails here
  texts = []
  monkeypatch.setattr(hud_mod.rl, "draw_circle_gradient", lambda *a: None)
  monkeypatch.setattr(hud_mod.rl, "draw_text_ex", lambda font, text, pos, size, spacing, color: texts.append((text, (color.r, color.g, color.b))))
  monkeypatch.setattr(ui_state, "is_metric", False)
  hud = _hud(model_on=model_on, showing_for=1.0)
  hud.set_speed = 64.0
  hud._font_display = hud._font_semi_bold = None
  hud._draw_set_speed(rl.Rectangle(0, 0, 536, 240))

  number = (MODEL_OFF_COLOR.r, MODEL_OFF_COLOR.g, MODEL_OFF_COLOR.b) if not model_on else (255, 255, 255)
  assert texts[0] == ("40", number)
  assert texts[1][1] == (255, 255, 255)  # MAX keeps its colour
