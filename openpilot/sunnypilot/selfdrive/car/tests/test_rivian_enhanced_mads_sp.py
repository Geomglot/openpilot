"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

"Use enhanced Rivian MADS": the Rivian MADS handling in CarSpecificEventsSP only runs with the
setting on. With it off, MADS is stock and this code emits nothing.
"""
from unittest.mock import MagicMock

from openpilot.cereal import custom, log
from opendbc.car import structs
from openpilot.common.constants import CV
from opendbc.sunnypilot.car.rivian.values import RivianFlagsSP
from openpilot.selfdrive.selfdrived.events import Events
from openpilot.sunnypilot.mads.helpers import MadsSteeringModeOnBrake
from openpilot.sunnypilot.mads.mads import ModularAssistiveDrivingSystem
from openpilot.sunnypilot.selfdrive.car import car_specific
from openpilot.sunnypilot.selfdrive.car.car_specific import CarSpecificEventsSP
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP

EventName = log.OnroadEvent.EventName
EventNameSP = custom.OnroadEventSP.EventName
State = custom.ModularAssistiveDrivingSystem.ModularAssistiveDrivingSystemState
ButtonType = structs.CarState.ButtonEvent.Type
GearShifter = structs.CarState.GearShifter


class FakeParams:
  # The constructor reads the brake mode and the minimum engage speed once.
  def __init__(self, steering_mode, min_engage_mph):
    self.values = {"MadsSteeringMode": steering_mode, "MadsMinEngageSpeed": min_engage_mph}

  def get(self, key, return_default=False):
    return self.values[key]


def _make(monkeypatch, enhanced=True, steering_mode=MadsSteeringModeOnBrake.DISENGAGE, min_engage_mph=0):
  monkeypatch.setattr(car_specific, "Params", lambda: FakeParams(steering_mode, min_engage_mph))
  CP = structs.CarParams.new_message()
  CP.brand = 'rivian'
  CP_SP = structs.CarParamsSP()
  if enhanced:
    CP_SP.flags |= RivianFlagsSP.ENHANCED_MADS.value
  return CarSpecificEventsSP(CP, CP_SP)


def _step(ev, gear=GearShifter.drive, up2=None, brake=False, pcm_enable=False, v_ego_mph=30.):
  CS = structs.CarState.new_message()
  CS.gearShifter = gear
  CS.brakePressed = brake
  CS.vEgo = v_ego_mph * CV.MPH_TO_MS
  if up2 is not None:
    CS.buttonEvents = [structs.CarState.ButtonEvent(pressed=up2, type=ButtonType.altButton2)]
  events = Events()
  if pcm_enable:
    events.add(EventName.pcmEnable)
  events_sp = ev.update(CS, events)
  return events_sp, events


class TestRivianEnhancedMads:
  def test_up2_disengages_and_blocks_cruise_engage_while_held(self, monkeypatch):
    ev = _make(monkeypatch)
    events_sp, events = _step(ev, up2=True, pcm_enable=True)
    assert events_sp.has(EventNameSP.lkasDisable)
    assert not events.has(EventName.pcmEnable)
    events_sp, events = _step(ev, pcm_enable=True)
    assert not events.has(EventName.pcmEnable)
    _step(ev, up2=False)
    events_sp, events = _step(ev, pcm_enable=True)
    assert events.has(EventName.pcmEnable)

  def test_park_entry_disengages_on_two_frames(self, monkeypatch):
    ev = _make(monkeypatch)
    out = [_step(ev, gear=g)[0].has(EventNameSP.lkasDisable) for g in
           (GearShifter.drive, GearShifter.park, GearShifter.park, GearShifter.park)]
    assert out == [False, True, True, False]

  def test_reverse_entry_disengages_on_two_frames(self, monkeypatch):
    # frame N loses to silentLkasDisable (paused), frame N+1 lands State.disabled
    ev = _make(monkeypatch)
    R, D = GearShifter.reverse, GearShifter.drive
    out = [_step(ev, gear=g)[0].has(EventNameSP.lkasDisable) for g in (D, R, R, R, D, R, R)]
    assert out == [False, True, True, False, False, True, True]

  def test_single_frame_reverse_blip(self, monkeypatch):
    ev = _make(monkeypatch)
    R, D = GearShifter.reverse, GearShifter.drive
    assert [_step(ev, gear=g)[0].has(EventNameSP.lkasDisable) for g in (D, R, D)] == [False, True, False]

  def test_pause_mode_holds_pause_for_the_whole_brake_press(self, monkeypatch):
    ev = _make(monkeypatch, steering_mode=MadsSteeringModeOnBrake.PAUSE)
    for _ in range(3):
      assert _step(ev, brake=True)[0].has(EventNameSP.silentLkasDisable)
    assert not _step(ev, brake=False)[0].has(EventNameSP.silentLkasDisable)

  def test_disengage_mode_adds_nothing_on_brake(self, monkeypatch):
    ev = _make(monkeypatch, steering_mode=MadsSteeringModeOnBrake.DISENGAGE)
    assert not _step(ev, brake=True)[0].has(EventNameSP.silentLkasDisable)

  def test_min_engage_speed_blocks_stalk_engage_below_it(self, monkeypatch):
    ev = _make(monkeypatch, min_engage_mph=5)
    assert _step(ev, v_ego_mph=0)[0].has(EventNameSP.belowMadsMinEngageSpeed)
    assert _step(ev, v_ego_mph=4.9)[0].has(EventNameSP.belowMadsMinEngageSpeed)
    assert not _step(ev, v_ego_mph=5.1)[0].has(EventNameSP.belowMadsMinEngageSpeed)

  def test_min_engage_speed_does_not_block_cruise_engage(self, monkeypatch):
    ev = _make(monkeypatch, min_engage_mph=5)
    assert not _step(ev, v_ego_mph=0, pcm_enable=True)[0].has(EventNameSP.belowMadsMinEngageSpeed)

  def test_min_engage_speed_zero_disables_the_gate(self, monkeypatch):
    ev = _make(monkeypatch, min_engage_mph=0)
    assert not _step(ev, v_ego_mph=0)[0].has(EventNameSP.belowMadsMinEngageSpeed)

  def test_off_is_stock(self, monkeypatch):
    ev = _make(monkeypatch, enhanced=False, min_engage_mph=5)
    frames = [{"up2": True, "pcm_enable": True}, {"gear": GearShifter.park, "pcm_enable": True},
              {"gear": GearShifter.park, "brake": True}, {"brake": True}, {"gear": GearShifter.reverse},
              {"gear": GearShifter.reverse}, {"v_ego_mph": 0}]
    for kw in frames:
      events_sp, events = _step(ev, **kw)
      assert not events_sp.has(EventNameSP.lkasDisable)
      assert not events_sp.has(EventNameSP.silentLkasDisable)
      assert not events_sp.has(EventNameSP.belowMadsMinEngageSpeed)
      assert events.has(EventName.pcmEnable) == kw.get("pcm_enable", False)


class TestRivianEnhancedMadsWithStateMachine:
  """Runs the real MADS state machine on the events this code emits, plus the gear and pedal
  events selfdrived would add, to check the resulting MADS state rather than single events."""

  def _setup(self, monkeypatch, steering_mode):
    ev = _make(monkeypatch, steering_mode=steering_mode)
    sd = MagicMock()
    sd.CP = structs.CarParams()
    sd.CP.brand = 'rivian'
    sd.CP_SP = ev.CP_SP
    sd.params = MagicMock()
    sd.params.get_bool = MagicMock(side_effect=lambda k: {"Mads": True, "MadsUnifiedEngagementMode": True}.get(k, False))
    sd.params.get = MagicMock(return_value=steering_mode)
    sd.events = Events()
    sd.events_sp = EventsSP()
    sd.enabled = sd.enabled_prev = False
    sd.initialized = True
    sd.CS_prev = structs.CarState.new_message()
    sd.sm = {'pandaStates': []}
    sd.state_machine = MagicMock()
    mads = ModularAssistiveDrivingSystem(sd)
    mads.enabled_toggle = True
    mads.steering_mode_on_brake = steering_mode
    return ev, mads, sd

  def _frame(self, ev, mads, sd, gear=GearShifter.drive, brake=False, v_ego_mph=0.):
    CS = structs.CarState.new_message()
    CS.gearShifter = gear
    CS.brakePressed = brake
    CS.vEgo = v_ego_mph * CV.MPH_TO_MS
    CS.standstill = v_ego_mph == 0.
    CS.cruiseState.available = True
    sd.events.clear()
    sd.events_sp.clear()
    # what selfdrived adds before MADS runs
    if gear == GearShifter.park:
      sd.events.add(EventName.wrongGear)
    elif gear == GearShifter.reverse:
      sd.events.add(EventName.reverseGear)
    if brake and (not sd.CS_prev.brakePressed or not CS.standstill):
      sd.events.add(EventName.pedalPressed)
    for name in ev.update(CS, sd.events).names:
      sd.events_sp.add(name)
    mads.update(CS)
    sd.CS_prev = CS
    return mads.state_machine.state

  def _brake_to_stop_then_shift(self, monkeypatch, steering_mode, gear):
    ev, mads, sd = self._setup(monkeypatch, steering_mode)
    mads.state_machine.state = State.enabled
    mads.enabled = mads.active = True
    self._frame(ev, mads, sd, brake=True, v_ego_mph=10.)
    for _ in range(5):
      self._frame(ev, mads, sd, brake=True)
    return [self._frame(ev, mads, sd, gear=gear, brake=True) for _ in range(3)], ev, mads, sd

  def test_pause_mode_brake_held_pauses(self, monkeypatch):
    ev, mads, sd = self._setup(monkeypatch, MadsSteeringModeOnBrake.PAUSE)
    mads.state_machine.state = State.enabled
    mads.enabled = mads.active = True
    self._frame(ev, mads, sd, brake=True, v_ego_mph=10.)
    assert [self._frame(ev, mads, sd, brake=True) for _ in range(5)] == [State.paused] * 5
    assert self._frame(ev, mads, sd) == State.enabled

  def test_pause_mode_park_with_brake_held_switches_off(self, monkeypatch):
    states, ev, mads, sd = self._brake_to_stop_then_shift(monkeypatch, MadsSteeringModeOnBrake.PAUSE, GearShifter.park)
    assert states[-1] == State.disabled
    # stays off after Drive is selected and the brake is released
    self._frame(ev, mads, sd, brake=True)
    assert self._frame(ev, mads, sd) == State.disabled

  def test_pause_mode_reverse_with_brake_held_switches_off(self, monkeypatch):
    states, ev, mads, sd = self._brake_to_stop_then_shift(monkeypatch, MadsSteeringModeOnBrake.PAUSE, GearShifter.reverse)
    assert states[-1] == State.disabled
    self._frame(ev, mads, sd, brake=True)
    assert self._frame(ev, mads, sd) == State.disabled

  def test_remain_active_park_with_brake_held_switches_off(self, monkeypatch):
    states, _, _, _ = self._brake_to_stop_then_shift(monkeypatch, MadsSteeringModeOnBrake.REMAIN_ACTIVE, GearShifter.park)
    assert states[-1] == State.disabled

  def test_pause_mode_park_without_brake_switches_off(self, monkeypatch):
    ev, mads, sd = self._setup(monkeypatch, MadsSteeringModeOnBrake.PAUSE)
    mads.state_machine.state = State.enabled
    mads.enabled = mads.active = True
    self._frame(ev, mads, sd)
    assert [self._frame(ev, mads, sd, gear=GearShifter.park) for _ in range(3)][-1] == State.disabled
