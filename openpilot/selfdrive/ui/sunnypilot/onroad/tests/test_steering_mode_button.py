"""The Rivian steering-mode wheel on the big UI: who sees it, and when a tap asks to switch modes.

The button is built without a window, so nothing is drawn. CarParams and carControl are real
capnp messages; only Params is replaced, by a dict with the same get_bool/put_bool calls.
"""
import pytest

pytest.importorskip("pyray")

from opendbc.car.rivian.values import RivianFlags
from opendbc.car.structs import car
from openpilot.cereal import messaging
from openpilot.selfdrive.ui.sunnypilot.onroad.steering_mode_button import SteeringModeButton
from openpilot.selfdrive.ui.ui_state import ui_state


class DictParams:
  def __init__(self, **values):
    self.values = dict(values)

  def get_bool(self, key):
    return bool(self.values.get(key, False))

  def put_bool(self, key, value):
    self.values[key] = bool(value)


class FakeSubMaster:
  def __init__(self, lat_active, recv_frame=10):
    msg = messaging.new_message('carControl')
    msg.carControl.latActive = lat_active
    self._cc = msg.as_reader().carControl
    self.recv_frame = {"carControl": recv_frame}
    self.frame = recv_frame

  def __getitem__(self, s):
    assert s == "carControl"
    return self._cc


def _cp(brand="rivian", angle_harness=True):
  cp = car.CarParams.new_message()
  cp.brand = brand
  cp.flags = int(RivianFlags.ANGLE_HARNESS) if angle_harness else 0
  return cp.as_reader()


def _button(**params):
  btn = SteeringModeButton.__new__(SteeringModeButton)
  btn._params = DictParams(**params)
  btn._click_delay = None
  btn._click_callback = None
  return btn


@pytest.fixture
def drive(monkeypatch):
  def _set(cp=None, lat_active=True, recv_frame=10, started_frame=5):
    monkeypatch.setattr(ui_state, "CP", cp if cp is not None else _cp())
    monkeypatch.setattr(ui_state, "sm", FakeSubMaster(lat_active, recv_frame))
    monkeypatch.setattr(ui_state, "started_frame", started_frame)
  return _set


@pytest.mark.parametrize("cp, shown", [
  (_cp(), True),
  (_cp(angle_harness=False), False),
  (_cp(brand="toyota"), False),
])
def test_only_angle_harness_rivians_see_the_wheel(drive, cp, shown):
  drive(cp=cp)
  assert SteeringModeButton.visible() is shown


def test_no_car_params_hides_the_wheel(monkeypatch):
  monkeypatch.setattr(ui_state, "CP", None)
  assert not SteeringModeButton.visible()


def test_tap_flips_the_torque_only_request(drive):
  drive()
  btn = _button(RivianEnableAngleSteering=True)
  btn._handle_mouse_release(None)
  assert btn._params.get_bool("RivianForceTorqueSteerReq")
  btn._handle_mouse_release(None)
  assert not btn._params.get_bool("RivianForceTorqueSteerReq")


@pytest.mark.parametrize("why, setup, params", [
  ("angle steering master switch off", {}, {"RivianEnableAngleSteering": False}),
  ("MADS not steering", {"lat_active": False}, {"RivianEnableAngleSteering": True}),
  ("carControl from before this drive", {"recv_frame": 3, "started_frame": 5}, {"RivianEnableAngleSteering": True}),
  ("torque-only harness", {"cp": _cp(angle_harness=False)}, {"RivianEnableAngleSteering": True}),
])
def test_tap_does_nothing_when_switching_is_not_allowed(drive, why, setup, params):
  drive(**setup)
  btn = _button(**params)
  btn._handle_mouse_release(None)
  assert not btn._params.get_bool("RivianForceTorqueSteerReq"), why
