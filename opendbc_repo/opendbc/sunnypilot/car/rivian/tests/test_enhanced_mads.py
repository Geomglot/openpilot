"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from opendbc.car import structs
from opendbc.sunnypilot.car.interfaces import _initialize_rivian_enhanced_mads
from opendbc.sunnypilot.car.rivian.values import RivianFlagsSP, RivianSafetyFlagsSP


class TestEnhancedMadsSetting:
  def _init(self, brand, value):
    CP = structs.CarParams(brand=brand)
    CP_SP = structs.CarParamsSP()
    params_dict = {} if value is None else {"RivianEnhancedMads": value}
    _initialize_rivian_enhanced_mads(CP, CP_SP, params_dict)
    return CP_SP

  def test_on_sets_flag_and_safety_bit(self):
    for value in (True, "1"):
      CP_SP = self._init('rivian', value)
      assert CP_SP.flags & RivianFlagsSP.ENHANCED_MADS
      assert CP_SP.safetyParam & RivianSafetyFlagsSP.ENHANCED_MADS

  def test_off_or_unset_leaves_both_clear(self):
    for value in (False, "0", None):
      CP_SP = self._init('rivian', value)
      assert not CP_SP.flags & RivianFlagsSP.ENHANCED_MADS
      assert not CP_SP.safetyParam & RivianSafetyFlagsSP.ENHANCED_MADS

  def test_other_brands_ignored(self):
    CP_SP = self._init('tesla', True)
    assert CP_SP.flags == 0
    assert CP_SP.safetyParam == 0
