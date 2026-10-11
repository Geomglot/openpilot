from openpilot.system.webrtc.webrtcd import cors_origin_allowed


def test_cors_origins():
  assert cors_origin_allowed("https://inertiaimpact.github.io")
  assert cors_origin_allowed("http://192.168.1.20:8080")
  assert not cors_origin_allowed(None)  # the opview app sends no Origin
  assert not cors_origin_allowed("")
