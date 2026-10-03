"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

"Use enhanced Rivian MADS": the Rivian MADS handling in CarSpecificEventsSP only runs with the
setting on. With it off, MADS is stock and this code emits nothing.
"""
from openpilot.cereal import custom, log
from opendbc.car import structs
from openpilot.common.constants import CV
from opendbc.sunnypilot.car.rivian.values import RivianFlagsSP
from openpilot.selfdrive.selfdrived.events import Events
from openpilot.sunnypilot.mads.helpers import MadsSteeringModeOnBrake
from openpilot.sunnypilot.selfdrive.car import car_specific
from openpilot.sunnypilot.selfdrive.car.car_specific import CarSpecificEventsSP

EventName = log.OnroadEvent.EventName
EventNameSP = custom.OnroadEventSP.EventName
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
