"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from types import SimpleNamespace

from opendbc.car import structs
from opendbc.sunnypilot.car.rivian.mads import MadsCarController

GearShifter = structs.CarState.GearShifter


def _step(mc, lat_active, gear=GearShifter.drive, mads_available=True, enabled=False):
  CC = structs.CarControl(latActive=lat_active, enabled=enabled)
  CC_SP = structs.CarControlSP()
  CC_SP.mads.available = mads_available
  CS = SimpleNamespace(out=structs.CarState(gearShifter=gear))
  mc.update(CC, CC_SP, CS)
  return mc.mads


class TestRivianMadsCarController:
  def test_icon_follows_lateral_from_the_first_active_frame(self):
    # The EPAS faults (ToiFlt) if torque is requested on a frame where the icon state is still off.
    mc = MadsCarController()
    assert _step(mc, False) == (False, False)
    assert _step(mc, True) == (True, True)
    assert _step(mc, False) == (False, False)
