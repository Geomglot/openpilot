"""Acceleration Control: the lift toward the cruise ramp, its fades and drops, and the chain with Gas Override Boost.

Messages use the real schemas, including the model's speed plan and path. These synthetic tests establish the
shape and state of the lift; they do not establish road comfort.
"""
import numpy as np
import pytest

from opendbc.car import structs
from openpilot.cereal import log
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.accel_boost import AccelBoost
from openpilot.sunnypilot.selfdrive.controls.lib.acceleration_control import (
  AccelerationControl, ACCELERATION_CONTROL_BOOST, LIFT_FALL, LIFT_RISE, PARAM_REFRESH_FRAMES,
)

Personality = log.LongitudinalPersonality
PLAN_T = [0.0, 0.5, 1.0, 1.5, 2.0, 3.0, 4.0, 6.0, 8.0, 10.0]
SETTLE = 60  # frames, more than enough for the largest lift to build up at LIFT_RISE
F32 = 1e-6  # the model's plan and path are float32 in the message, so a faded-out lift is ~1e-8, not 0


class FakeParams:
  def __init__(self, enabled=True):
    self.values = {"AccelerationControl": enabled}

  def get_bool(self, key):
    return bool(self.values.get(key, False))


def _sm(v_ego=25.0, personality=Personality.standard, engaged=True, should_stop=False,
        desired_accel=0.1, gas_pressed=False, plan_drop_mph=0.0, lat_accel=0.0):
  """plan_drop_mph: how far the model's speed plan falls by 6 s. lat_accel: sideways m/s^2 on the path ahead."""
  vx = [max(v_ego - plan_drop_mph * CV.MPH_TO_MS * min(t, 6.0) / 6.0, 0.0) for t in PLAN_T]
  yaw = [lat_accel / max(v, 1e-3) for v in vx]
  model = log.ModelDataV2.new_message(action={'desiredAcceleration': desired_accel, 'shouldStop': should_stop},
                                      velocity={'t': PLAN_T, 'x': vx}, orientationRate={'t': PLAN_T, 'z': yaw})
  return {
    'carState': structs.CarState(vEgo=v_ego, gasPressed=gas_pressed),
    'selfdriveState': log.SelfdriveState.new_message(enabled=engaged, personality=personality),
    'modelV2': model,
  }


def _assist(enabled=True, op_long=True):
  return AccelerationControl(structs.CarParams(openpilotLongitudinalControl=op_long), params=FakeParams(enabled))


def _settle(assist, sm, raw, a_cruise=1.0, frames=SETTLE, throttle_scale=1.0):
  out = None
  for _ in range(frames):
    out = assist.update(sm, raw, a_cruise, throttle_scale)
  return out


@pytest.mark.parametrize('personality', list(ACCELERATION_CONTROL_BOOST))
def test_lift_reaches_the_personality_cap_well_below_the_set_speed(personality):
  assist = _assist()
  out = _settle(assist, _sm(personality=personality, desired_accel=0.1), 0.1)
  assert assist.boost == pytest.approx(ACCELERATION_CONTROL_BOOST[personality])
  assert out == pytest.approx(0.1 + ACCELERATION_CONTROL_BOOST[personality])


@pytest.mark.parametrize('a_cruise, expected', [(0.15, 0.05), (0.1, 0.0), (0.0, 0.0)])
def test_never_lifts_past_the_cruise_ramp(a_cruise, expected):
  assist = _assist()
  _settle(assist, _sm(desired_accel=0.1), 0.1, a_cruise=a_cruise)
  assert assist.boost == pytest.approx(expected)


def test_near_the_set_speed_a_wobbling_model_is_not_passed_on():
  # cruise ramp at +0.1 near the set speed, model wobbling between -0.05 and +0.05 every 4 s
  assist = _assist()
  _settle(assist, _sm(desired_accel=0.0), 0.0, a_cruise=0.1)
  outs = []
  for i in range(200):
    raw = float(0.05 * np.sin(2 * np.pi * i * DT_MDL / 4.0))
    outs.append(assist.update(_sm(desired_accel=raw), raw, 0.1))
  assert max(outs) - min(outs) < 0.01  # the cruise ramp sets the speed; the model moved 0.1


@pytest.mark.parametrize('raw, fade', [(0.0, 1.0), (-0.05, 0.75), (-0.1, 0.5), (-0.2, 0.0), (-0.4, 0.0)])
def test_lift_fades_out_as_the_model_asks_to_slow(raw, fade):
  assist = _assist()
  _settle(assist, _sm(desired_accel=raw), raw)
  assert assist.boost == pytest.approx(0.2 * fade)


@pytest.mark.parametrize('drop_mph, guard', [(0.0, 1.0), (1.0, 1.0), (2.0, 0.5), (3.0, 0.0), (8.0, 0.0)])
def test_model_planning_to_slow_keeps_its_early_slowing(drop_mph, guard):
  assist = _assist()
  _settle(assist, _sm(desired_accel=-0.05, plan_drop_mph=drop_mph), -0.05)
  assert assist.boost == pytest.approx(0.2 * 0.75 * guard, abs=F32)


def test_planned_slowdown_does_not_guard_a_model_asking_to_speed_up():
  assist = _assist()
  _settle(assist, _sm(desired_accel=0.05, plan_drop_mph=8.0), 0.05)
  assert assist.boost == pytest.approx(0.2)


@pytest.mark.parametrize('lat, scale', [(0.0, 1.0), (0.5, 1.0), (0.75, 0.5), (1.0, 0.0), (2.0, 0.0)])
def test_lift_fades_in_a_bend_ahead(lat, scale):
  assist = _assist()
  _settle(assist, _sm(desired_accel=0.1, lat_accel=lat), 0.1)
  assert assist.boost == pytest.approx(0.2 * scale, abs=F32)


def test_lift_builds_up_and_falls_away_gradually():
  assist = _assist()
  assist.update(_sm(desired_accel=0.1), 0.1, 1.0)
  assert assist.boost == pytest.approx(LIFT_RISE * DT_MDL)
  _settle(assist, _sm(desired_accel=0.1), 0.1)
  # the model eases to -0.3: no lift wanted, but -0.3 is not real braking, so it falls at LIFT_FALL
  assist.update(_sm(desired_accel=-0.3), -0.3, 1.0)
  assert assist.boost == pytest.approx(0.2 - LIFT_FALL * DT_MDL)


@pytest.mark.parametrize('case', ['braking', 'stop', 'disengaged', 'disabled', 'unavailable'])
def test_drops_at_once(case):
  assist = _assist()
  _settle(assist, _sm(), 0.1)
  assert assist.boost == pytest.approx(0.2)
  raw = -0.6 if case == 'braking' else 0.1
  sm = _sm(desired_accel=raw, engaged=case != 'disengaged', should_stop=case == 'stop')
  if case in ('disabled', 'unavailable'):
    assist.params.values['AccelerationControl'] = case != 'disabled'
    assist.available = case != 'unavailable'
    assist._frame = PARAM_REFRESH_FRAMES
  assert assist.update(sm, raw, 1.0) == raw
  assert assist.boost == 0
  assert assist.lift == 0
  # and it builds up from zero again afterwards
  assist.available = True
  assist.params.values['AccelerationControl'] = True
  assist._frame = PARAM_REFRESH_FRAMES
  assist.update(_sm(), 0.1, 1.0)
  assert assist.boost == pytest.approx(LIFT_RISE * DT_MDL)


@pytest.mark.parametrize('mph, scale', [(5, 0.0), (10, 0.0), (15, 0.5), (20, 1.0), (50, 1.0)])
def test_speed_ramp(mph, scale):
  assist = _assist()
  _settle(assist, _sm(v_ego=mph * CV.MPH_TO_MS), 0.1)
  assert assist.boost == pytest.approx(0.2 * scale)


@pytest.mark.parametrize('scale, expected', [(1.0, 0.2), (0.5, 0.1), (0.0, 0.0), (-1.0, 0.0), (2.0, 0.2)])
def test_lateral_throttle_interlock_cuts_at_once_and_keeps_the_lift(scale, expected):
  assist = _assist()
  _settle(assist, _sm(), 0.1)
  assert assist.update(_sm(), 0.1, 1.0, throttle_scale=scale) == pytest.approx(0.1 + expected)
  assert assist.lift == pytest.approx(0.2)  # back in full the moment the interlock releases


def test_reset_starts_over_from_zero():
  assist = _assist()
  _settle(assist, _sm(), 0.1)
  assist.reset()
  assert (assist.lift, assist.boost) == (0.0, 0.0)
  assist.update(_sm(), 0.1, 1.0)
  assert assist.boost == pytest.approx(LIFT_RISE * DT_MDL)


def test_missing_model_plan_is_treated_as_straight_and_steady():
  assist = _assist()
  sm = _sm(desired_accel=-0.05)
  sm['modelV2'] = log.ModelDataV2.new_message(action={'desiredAcceleration': -0.05})
  _settle(assist, sm, -0.05)
  assert assist.boost == pytest.approx(0.2 * 0.75)


def test_setting_refresh_and_availability():
  for enabled, available in ((False, True), (True, False)):
    assist = _assist(enabled, available)
    assert _settle(assist, _sm(), 0.1) == 0.1
  assist = _assist(enabled=False)
  assist.update(_sm(), 0.1, 1.0)
  assist.params.values['AccelerationControl'] = True
  _settle(assist, _sm(), 0.1)
  assert assist.boost == pytest.approx(0.2)
  assist.params.values['AccelerationControl'] = False
  _settle(assist, _sm(), 0.1, frames=PARAM_REFRESH_FRAMES)
  assert assist.boost == 0


def test_unknown_real_personality_uses_standard_fallback():
  assist = _assist()
  _settle(assist, _sm(personality=99), 0.1)
  assert assist.boost == pytest.approx(0.2)


def _override_cycles(gas, assist, raw, lead, cruise=1.0, presses=4):
  # Real chain in production order: 2 s hands-off, then 3 s of gas, repeated.
  for _ in range(presses):
    for pressed, frames in ((False, 40), (True, 60)):
      for _ in range(frames):
        sm = _sm(desired_accel=raw, gas_pressed=pressed)
        gas.update(sm, assist.update(sm, raw, cruise), lead, cruise)


def test_gas_override_does_not_learn_when_assist_already_cleared_the_model():
  # Model mildly timid (0.12), lead allows 0.30. With the assist, the e2e candidate (0.12 + 0.2)
  # is above the lead, so the lead is the limit and the driver's gas presses say nothing about
  # the model. The override must not latch eligibility from the un-lifted request.
  gas, assist = AccelBoost(), _assist()
  _override_cycles(gas, assist, raw=0.12, lead=0.30)
  assert gas.total_boost == 0
  # Without the assist the model really is the limit, so the override learns as upstream intends.
  gas, assist = AccelBoost(), _assist(enabled=False)
  _override_cycles(gas, assist, raw=0.12, lead=0.30)
  # two overrides of 0.05 each; then 0.12 + 0.10 is no longer 0.1 under the lead, so it stops
  assert gas.total_boost == pytest.approx(0.1)


def test_gas_override_boost_survives_the_lift_coming_and_going():
  gas, assist = AccelBoost(), _assist()
  idle = _sm(desired_accel=0.0)
  gas.update(idle, 0, 1, 1)
  for _ in range(40):
    gas.update(_sm(desired_accel=0.0, gas_pressed=True), 0, 1, 1)
  gas.update(idle, 0, 1, 1)
  latched = gas.total_boost
  assert latched > 0
  for raw in [0.1] * 40 + [-0.3] * 40 + [0.05] * 40:
    sm = _sm(desired_accel=raw)
    out = gas.update(sm, assist.update(sm, raw, 1.0), 1, 1)
    assert gas.total_boost == pytest.approx(latched)
    assert out == pytest.approx(raw + assist.boost + latched)
