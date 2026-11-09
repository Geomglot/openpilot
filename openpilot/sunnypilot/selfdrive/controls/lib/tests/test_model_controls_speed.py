"""The "Model controls speed" setting, through the real planner update.

On (the default) the model's request competes in the planner exactly as before. Off, it is left out, and the
cruise ramp gets back the acceleration and turn limits of the old non-Experimental planner. The MPC and the
unrelated sunnypilot controllers are the same seams as in test_acceleration_control_planner.
"""
import pytest

from openpilot.cereal import log
from openpilot.selfdrive.controls.lib.longitudinal_planner import get_max_accel, limit_accel_in_turns
from openpilot.sunnypilot.selfdrive.controls.lib.tests.test_acceleration_control_planner import Source, _planner

V_EGO = 25.0  # m/s, what _planner drives at
STEER_RATIO = 15.0
WHEELBASE = 3.0


def _setup(monkeypatch, model_controls_speed, raw=-2.0, cruise=0.5, lead=1.0, angle=0.0, model_stop=False):
  planner, sm = _planner(monkeypatch, raw, cruise, lead)
  planner.model_controls_speed = model_controls_speed
  planner.CP.steerRatio = STEER_RATIO
  planner.CP.wheelbase = WHEELBASE
  sm['carState'].steeringAngleDeg = angle
  sm['vehicleParameters'] = log.VehicleParameters.new_message(angleOffsetDeg=0.0)
  sm['modelV2'].action.shouldStop = model_stop
  return planner, sm


def test_on_keeps_the_model_in_charge(monkeypatch):
  # unchanged behaviour: a model asking to slow wins over cruise and lead
  planner, sm = _setup(monkeypatch, True)
  planner.update(sm)
  assert planner.mpc.source == Source.e2e
  assert planner.output_a_target == pytest.approx(-2.0)


@pytest.mark.parametrize('raw', [-2.0, -0.3, 0.0, 0.2, 3.0])
def test_off_never_lets_the_model_win(monkeypatch, raw):
  planner, sm = _setup(monkeypatch, False, raw=raw)
  planner.update(sm)
  assert planner.mpc.source == Source.cruise
  assert planner.output_a_target == pytest.approx(0.5, abs=1e-6)


def test_off_still_follows_the_lead(monkeypatch):
  planner, sm = _setup(monkeypatch, False, lead=-1.0)
  planner.update(sm)
  assert planner.mpc.source == Source.lead0
  assert planner.output_a_target == pytest.approx(-1.0, abs=1e-6)


def test_off_ignores_a_model_stop(monkeypatch):
  planner, sm = _setup(monkeypatch, False, model_stop=True)
  planner.update(sm)
  assert not planner.output_should_stop
  planner, sm = _setup(monkeypatch, True, model_stop=True)
  planner.update(sm)
  assert planner.output_should_stop


def test_off_does_not_boost(monkeypatch):
  planner, sm = _setup(monkeypatch, False, raw=0.1)
  planner.update(sm)
  assert planner.acceleration_control.boost == 0.0
  assert planner.accel_boost.total_boost == 0.0


def test_off_caps_cruise_acceleration_at_the_old_limit(monkeypatch):
  # far below set speed: on, the cruise ramp may ask for more than the old 0.8 m/s^2 at 25 m/s; off, it may not
  planner, sm = _setup(monkeypatch, False, raw=3.0, cruise=0.8, lead=3.0)
  sm['carState'].vCruise = (V_EGO + 5.0) * 3.6
  planner.update(sm)
  assert get_max_accel(V_EGO) == pytest.approx(0.8)
  assert planner.output_a_target == pytest.approx(0.8, abs=1e-6)


def test_off_holds_acceleration_in_a_tight_turn(monkeypatch):
  # 10 degrees at 25 m/s is about 2.4 m/s^2 sideways, more than the 2.075 m/s^2 grip budget at this speed
  assert limit_accel_in_turns(V_EGO, 10.0, _setup(monkeypatch, False)[0].CP) == 0.0
  planner, sm = _setup(monkeypatch, False, raw=3.0, cruise=0.0, lead=3.0, angle=10.0)
  sm['carState'].vCruise = (V_EGO + 5.0) * 3.6
  planner.update(sm)
  assert planner.output_a_target == pytest.approx(0.0, abs=1e-6)


def test_old_limit_values():
  # the pre-#39097 tables, so a later edit cannot quietly change what "off" means
  assert [get_max_accel(v) for v in (0., 10., 25., 40.)] == pytest.approx([1.6, 1.2, 0.8, 0.6])
  cp = type('CP', (), {'steerRatio': STEER_RATIO, 'wheelbase': WHEELBASE})
  assert limit_accel_in_turns(20.0, 0.0, cp) == pytest.approx(1.7)
  assert limit_accel_in_turns(40.0, 0.0, cp) == pytest.approx(3.2)
