"""The MAX box as the "Model Controls Speed" switch on the big UI: when a tap is taken, and what it writes.

The button is built without a window, so nothing is drawn. carState is a real capnp message; only Params is
replaced, by a dict with the same get/put_bool calls.
"""
import pytest

pytest.importorskip("pyray")

from openpilot.cereal import messaging
from openpilot.common.constants import CV
from openpilot.selfdrive.ui.sunnypilot.onroad.model_speed_button import ModelSpeedButton
from openpilot.selfdrive.ui.ui_state import ui_state


class DictParams:
  def __init__(self, **values):
    self.values = dict(values)

  def get(self, key, return_default=False):
    if key not in self.values:
      return True if return_default else None  # registered default "1"
    return self.values[key]

  def put_bool(self, key, value):
    self.values[key] = bool(value)


class FakeSubMaster:
  def __init__(self, v_ego, recv_frame=10):
    msg = messaging.new_message('carState')
    msg.carState.vEgo = v_ego
    self._cs = msg.as_reader().carState
    self.recv_frame = {"carState": recv_frame}
    self.frame = recv_frame

  def __getitem__(self, s):
    assert s == "carState"
    return self._cs


def _button(**params):
  btn = ModelSpeedButton.__new__(ModelSpeedButton)
  btn._params = DictParams(**params)
  btn.model_on = btn._read()
  btn._last_read = 0.0
  btn._click_delay = None
  btn._click_callback = None
  return btn


@pytest.fixture
def drive(monkeypatch):
  def _set(mph=40.0, has_long=True, metric=False, recv_frame=10, started_frame=5, kph=None):
    v_ego = kph * CV.KPH_TO_MS if kph is not None else mph * CV.MPH_TO_MS
    monkeypatch.setattr(ui_state, "has_longitudinal_control", has_long)
    monkeypatch.setattr(ui_state, "is_metric", metric)
    monkeypatch.setattr(ui_state, "sm", FakeSubMaster(v_ego, recv_frame))
    monkeypatch.setattr(ui_state, "started_frame", started_frame)
  return _set


def test_unwritten_setting_reads_as_on():
  assert _button().model_on


def test_tap_switches_the_setting_both_ways(drive):
  drive()
  btn = _button(ModelControlsSpeed=True)
  btn._handle_mouse_release(None)
  assert not btn.model_on
  assert btn._params.values["ModelControlsSpeed"] is False
  btn._handle_mouse_release(None)
  assert btn.model_on
  assert btn._params.values["ModelControlsSpeed"] is True


@pytest.mark.parametrize("why, setup", [
  ("below 20 mph", {"mph": 19.5}),
  ("below 30 km/h on a metric car", {"metric": True, "kph": 29.5}),
  ("no openpilot longitudinal", {"has_long": False}),
  ("carState from before this drive", {"recv_frame": 3, "started_frame": 5}),
])
def test_tap_is_ignored(drive, why, setup):
  drive(**setup)
  btn = _button(ModelControlsSpeed=True)
  btn._handle_mouse_release(None)
  assert btn.model_on, why
  assert btn._params.values["ModelControlsSpeed"] is True, why


@pytest.mark.parametrize("setup", [{"mph": 20.5}, {"metric": True, "kph": 30.5}])
def test_tap_is_taken_just_above_the_cut_off(drive, setup):
  drive(**setup)
  btn = _button(ModelControlsSpeed=True)
  btn._handle_mouse_release(None)
  assert not btn.model_on


def test_follows_a_change_made_elsewhere(drive):
  drive()
  btn = _button(ModelControlsSpeed=True)
  btn._params.values["ModelControlsSpeed"] = False
  btn._update_state()
  assert not btn.model_on
