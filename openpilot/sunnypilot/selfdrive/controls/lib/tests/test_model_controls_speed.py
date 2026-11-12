"""The "Model controls speed" setting, through the real planner update.

On (the default) the model's request competes in the planner exactly as before. Off, it is left out, and the
cruise ramp gets back the acceleration and turn limits of the old non-Experimental planner. It can be switched
while driving: the planner picks the change up within half a second, not at a crawl, and eases it in. The MPC
and the unrelated sunnypilot controllers are the same seams as in test_acceleration_control_planner.
"""
import pytest

from openpilot.cereal import log
from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlannerSP
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.longitudinal_planner import (get_max_accel, limit_accel_in_turns, MODEL_SWITCH_JERK,
                                                                   MODEL_SWITCH_MIN_SPEED, MODEL_SWITCH_REFRESH_FRAMES)
from openpilot.sunnypilot.selfdrive.controls.lib.tests.test_acceleration_control_planner import Source, _planner

V_EGO = 25.0  # m/s, what _planner drives at
STEER_RATIO = 15.0
WHEELBASE = 3.0


def _setup(monkeypatch, model_controls_speed, raw=-2.0, cruise=0.5, lead=1.0, angle=0.0, model_stop=False):
  planner, sm = _planner(monkeypatch, raw, cruise, lead)
  planner.model_controls_speed = model_controls_speed
  planner.params.model_controls_speed = model_controls_speed
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


STEP = MODEL_SWITCH_JERK * DT_MDL


def _run(planner, sm, frames):
  out = []
  for _ in range(frames):
    planner.update(sm)
    out.append((float(planner.output_a_target), planner.mpc.source))
  return out


def _switch(planner, sm, on):
  # settle on the current setting, then flip it as a tap would, at the start of a refresh period
  _run(planner, sm, MODEL_SWITCH_REFRESH_FRAMES - planner.model_switch_frame % MODEL_SWITCH_REFRESH_FRAMES)
  before = float(planner.output_a_target)
  planner.params.model_controls_speed = on
  return before


def test_switch_waits_for_the_next_refresh(monkeypatch):
  planner, sm = _setup(monkeypatch, True, raw=-1.0)
  planner.update(sm)
  planner.params.model_controls_speed = False
  _run(planner, sm, MODEL_SWITCH_REFRESH_FRAMES - 1)
  assert planner.model_controls_speed
  _run(planner, sm, 1)
  assert not planner.model_controls_speed


def test_switching_off_eases_out_of_the_model_braking(monkeypatch):
  planner, sm = _setup(monkeypatch, True, raw=-1.0, cruise=0.5, lead=1.0)
  assert _switch(planner, sm, False) == pytest.approx(-1.0)
  out = _run(planner, sm, 60)
  accels = [-1.0] + [a for a, _ in out]
  steps = [b - a for a, b in zip(accels, accels[1:], strict=False)]
  assert max(steps) == pytest.approx(STEP)  # no jump: rises at the easing rate
  assert all(s <= STEP + 1e-9 for s in steps)
  assert accels[-1] == pytest.approx(0.5, abs=1e-6)  # and does get to cruise
  assert sum(s > 1e-9 for s in steps) == pytest.approx(1.5 / STEP, abs=1)
  assert not planner.model_switch_blending
  assert out[-1][1] == Source.cruise


def test_switching_on_eases_into_the_model_braking(monkeypatch):
  planner, sm = _setup(monkeypatch, False, raw=-1.0, cruise=0.5, lead=1.0)
  assert _switch(planner, sm, True) == pytest.approx(0.5, abs=1e-6)
  out = _run(planner, sm, 60)
  accels = [0.5] + [a for a, _ in out]
  steps = [b - a for a, b in zip(accels, accels[1:], strict=False)]
  assert min(steps) == pytest.approx(-STEP)
  assert all(s >= -STEP - 1e-9 for s in steps)
  assert accels[-1] == pytest.approx(-1.0)
  assert not planner.model_switch_blending
  assert out[-1][1] == Source.e2e


@pytest.mark.parametrize('on', [True, False])
def test_lead_braking_is_never_eased(monkeypatch, on):
  planner, sm = _setup(monkeypatch, not on, raw=-1.0, cruise=0.5, lead=1.0)
  _switch(planner, sm, on)
  _run(planner, sm, 3)
  assert planner.model_switch_blending
  planner.mpc = type(planner.mpc)(-2.5)
  _run(planner, sm, 1)
  assert planner.output_a_target == pytest.approx(-2.5, abs=1e-6)
  assert planner.mpc.source == Source.lead0


def test_switching_on_restarts_the_override_boost(monkeypatch):
  planner, sm = _setup(monkeypatch, False, raw=0.2)
  planner.accel_boost.total_boost = 0.15
  _switch(planner, sm, True)
  _run(planner, sm, 1)
  assert planner.model_controls_speed
  assert planner.accel_boost.total_boost == 0.0


def test_no_switch_at_a_crawl_while_openpilot_controls_the_speed(monkeypatch):
  # switching off at a red light with no lead would set off towards the set speed
  planner, sm = _setup(monkeypatch, True, raw=-1.0)
  sm['carState'].vEgo = MODEL_SWITCH_MIN_SPEED - 0.5
  planner.params.model_controls_speed = False
  _run(planner, sm, 3 * MODEL_SWITCH_REFRESH_FRAMES)
  assert planner.model_controls_speed
  sm['carState'].vEgo = MODEL_SWITCH_MIN_SPEED + 0.5
  _run(planner, sm, MODEL_SWITCH_REFRESH_FRAMES)
  assert not planner.model_controls_speed


def test_switch_while_not_engaged_applies_at_any_speed_without_easing(monkeypatch):
  planner, sm = _setup(monkeypatch, True, raw=-1.0)
  sm['carState'].vEgo = 0.0
  sm['controlsState'].longControlState = 'off'
  planner.params.model_controls_speed = False
  _run(planner, sm, 1)
  assert not planner.model_controls_speed
  assert not planner.model_switch_blending


@pytest.mark.parametrize('on', [True, False])
def test_easing_ignores_the_curve_controllers_decel(monkeypatch, on):
  # Smart Cruise Control writes its own decel into output_a_target before the planner takes a_prev. The easing must
  # still start from what was actually sent, or switching mid-curve would jump to that decel.
  planner, sm = _setup(monkeypatch, not on, raw=-1.0, cruise=0.5, lead=1.0)
  before = _switch(planner, sm, on)
  monkeypatch.setattr(LongitudinalPlannerSP, 'update_targets', lambda self, sm, v_ego, a_ego, v_cruise: (v_cruise, -2.0))
  _run(planner, sm, 1)
  direction = -1.0 if on else 1.0
  assert planner.output_a_target == pytest.approx(before + direction * STEP)
