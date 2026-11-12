"""The comma 4 MAX box as the "Model Controls Speed" switch: first tap shows the box, a tap while it shows switches.

The HUD is built without a window, so nothing is drawn. The clock is a stand-in for rl.get_time, the box's fade is
set directly on its filter, and Params is a dict with the same get/put_bool calls.
"""
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
