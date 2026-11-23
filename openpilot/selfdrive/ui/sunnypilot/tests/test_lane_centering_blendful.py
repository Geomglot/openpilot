"""Lane Centering and Blendful are mutually exclusive: switching one on switches the other off.

Runs UIStateSP._enforce_lane_centering_blendful against an in-memory params stand-in, so it covers
every way the params can change (device toggle, small screen, Sunnylink) without a window.
"""
import pytest


from openpilot.selfdrive.ui.sunnypilot.ui_state import UIStateSP, lane_centering_blendful_loser


class FakeParams:
  def __init__(self, **values):
    self.values = dict(values)

  def get_bool(self, key):
    return bool(self.values.get(key, False))

  def put_bool(self, key, value, block=False):
    self.values[key] = value

  def get(self, key):
    return self.values.get(key)

  def put(self, key, value, block=False):
    self.values[key] = value


def _ui(**values):
  ui = UIStateSP.__new__(UIStateSP)
  ui.params = FakeParams(**values)
  ui._prev_lane_centering = False
  ui._prev_blendful = False
  return ui


def _set(ui, lane_centering, blendful):
  ui.params.values["LaneCenteringEnabled"] = lane_centering
  ui.params.values["Blendful"] = blendful
  ui._enforce_lane_centering_blendful()
  return ui.params.get_bool("LaneCenteringEnabled"), ui.params.get_bool("Blendful")


@pytest.mark.parametrize("lc,bf", [(False, False), (True, False), (False, True)])
def test_one_or_none_on_is_left_alone(lc, bf):
  ui = _ui()
  assert _set(ui, lc, bf) == (lc, bf)
  assert ui.params.get("ParamsVersion") is None


def test_turning_on_lane_centering_turns_off_blendful():
  ui = _ui()
  _set(ui, False, True)
  assert _set(ui, True, True) == (True, False)


def test_turning_on_blendful_turns_off_lane_centering():
  ui = _ui()
  _set(ui, True, False)
  assert _set(ui, True, True) == (False, True)


def test_both_on_at_startup_keeps_blendful():
  ui = _ui()
  assert _set(ui, True, True) == (False, True)


def test_switching_back_and_forth():
  ui = _ui()
  assert _set(ui, True, False) == (True, False)
  assert _set(ui, True, True) == (False, True)   # Blendful switched on
  assert _set(ui, True, True) == (True, False)   # Lane Centering switched back on
  assert _set(ui, True, True) == (False, True)   # and Blendful again


def test_flip_bumps_params_version_for_sunnylink():
  ui = _ui(ParamsVersion="7")
  _set(ui, True, True)
  assert ui.params.get("ParamsVersion") == "8"


def test_loser_helper():
  assert lane_centering_blendful_loser(True, True, False, True) == "Blendful"
  assert lane_centering_blendful_loser(True, True, True, False) == "LaneCenteringEnabled"
  assert lane_centering_blendful_loser(True, True, False, False) == "LaneCenteringEnabled"
  assert lane_centering_blendful_loser(True, False, False, True) is None
