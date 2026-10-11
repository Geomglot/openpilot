import json

from openpilot.cereal import log
from openpilot.system.webrtc import opview_slim


def _reader(service, build):
  ev = log.Event.new_message()
  build(ev.init(service))
  return getattr(ev.as_reader(), service)


class TestOpviewSlim:
  def test_keeps_only_listed_fields_and_rounds(self):
    def build(cs):
      cs.vEgo = 17.123456789
      cs.leftBlinker = True
      cs.gearShifter = "drive"
      cs.cruiseState.enabled = True
      cs.cruiseState.speed = 30.
      cs.steeringPressed = True  # not read by opview
    out = opview_slim.slim("carState", _reader("carState", build))
    assert out["vEgo"] == 17.123
    assert out["leftBlinker"] is True
    assert out["gearShifter"] == "drive"
    assert out["cruiseState"] == {"enabled": True}
    assert "steeringPressed" not in out
    json.dumps(out)

  def test_union_keeps_the_active_member(self):
    def build(cs):
      cs.curvature = 0.001234567
      cs.lateralControlState.init("torqueState").saturated = True
    out = opview_slim.slim("controlsState", _reader("controlsState", build))
    assert out["lateralControlState"] == {"torqueState": {"saturated": True}}
    assert out["curvature"] == 0.001

  def test_leads(self):
    def build(rs):
      rs.leadOne.present = True
      rs.leadOne.dRel = 30.4567
      rs.leadOne.radar = True
    out = opview_slim.slim("radarState", _reader("radarState", build))
    assert out["leadOne"]["present"] is True
    assert out["leadOne"]["dRel"] == 30.457
    assert out["leadOne"]["radar"] is True
    assert "aLeadTau" not in out["leadOne"]

  def test_list_of_structs(self):
    ev = log.Event.new_message()
    events = ev.init("onroadEvents", 1)
    events[0].overrideLongitudinal = True
    out = opview_slim.slim("onroadEvents", ev.as_reader().onroadEvents)
    assert out[0]["overrideLongitudinal"] is True

  def test_unlisted_service_is_sent_whole_with_rounded_floats(self):
    def build(gps):
      gps.latitude = 45.123456789
      gps.hasFix = True
    out = opview_slim.slim("gpsLocationExternal", _reader("gpsLocationExternal", build))
    assert out["latitude"] == 45.123
    assert out["hasFix"] is True

  def test_rates(self):
    assert opview_slim.min_gap_s("modelV2") is None
    assert opview_slim.min_gap_s("longitudinalPlan") > opview_slim.min_gap_s("carState")
    assert opview_slim.min_gap_s("someNewService") == 0.9 / opview_slim.OPVIEW_DEFAULT_RATE_HZ
