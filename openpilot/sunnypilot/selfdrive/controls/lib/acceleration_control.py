"""Personality-linked lift of the model's acceleration request toward the cruise ramp, before planner min-selection.

The driving model does not know the set speed and can settle the car below it, asking for a little less than zero.
This lifts the model's request, by at most the personality cap, toward the cruise ramp and never past it. The cruise
ramp tapers to zero at the set speed, so close to it the ramp sets the speed and small changes in the model's
request have no effect.

The lift is full while the model asks for 0 or more and fades out by -0.2 m/s^2. It also fades out when the model's
own speed plan slows the car by 1 to 3 mph over the next 6 s, so it does not cut back the model's early slowing for
a light, sign or slower traffic, and through bends 0.5 to 1.0 m/s^2 sideways on the path of the next 2 s. It drops at
once when the model brakes harder than -0.5 m/s^2 or predicts a stop. Otherwise it rises at 0.5 and falls at
0.3 m/s^3, so a wobble in the model's request is not passed on to the car faster than it came.

Curve control's throttle interlock scales the lift as before, without smoothing, so it still cuts at once.

It runs BEFORE Gas Override Boost, which keeps its own state and output. That one latches "the model is limiting"
from the candidate it is handed; it must see this lift, or it learns a boost from overrides where the lead, not the
model, was the limit. Competing cruise and lead candidates and final acceleration limits still constrain the result.

Tuned from an open-loop replay of logged drives (all Aggressive); road acceptance still needed.
"""
import numpy as np

from openpilot.cereal import log
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.accel_boost import ACCEL_BOOST_MIN_SPEED

Personality = log.LongitudinalPersonality

# m/s^2, the most the model's request is lifted, by personality
ACCELERATION_CONTROL_BOOST = {
  Personality.relaxed: 0.1,
  Personality.standard: 0.2,
  Personality.aggressive: 0.3,
}

# model's raw request, m/s^2: full lift at 0 or more, none at -0.2; real braking below -0.5 drops the lift at once
MODEL_FADE_BP = [-0.2, 0.0]
MODEL_BRAKING = -0.5
# the model's own speed plan: slowing by 1 to 3 mph within PLANNED_DROP_T fades the lift out (only while it asks to slow)
PLANNED_DROP_BP = [1 * CV.MPH_TO_MS, 3 * CV.MPH_TO_MS]
PLANNED_DROP_T = 6.0  # s
# sideways acceleration on the model's path within LAT_ACCEL_T: full lift at 0.5 m/s^2, none at 1.0
LAT_ACCEL_BP = [0.5, 1.0]
LAT_ACCEL_T = 2.0  # s
# m/s^3, how fast the lift may change outside the immediate drops
LIFT_RISE = 0.5
LIFT_FALL = 0.3

PARAM_REFRESH_FRAMES = 20  # 1 s at the 20 Hz planner rate


def planned_drop(model, v_ego: float) -> float:
  """m/s the model's speed plan falls below the current speed within PLANNED_DROP_T, 0 if it does not slow."""
  t, vx = model.velocity.t, model.velocity.x
  if len(t) == 0 or len(t) != len(vx):
    return 0.0
  ahead = [vx[i] for i in range(len(t)) if t[i] <= PLANNED_DROP_T]
  return max(0.0, v_ego - min(ahead)) if ahead else 0.0


def lat_accel_ahead(model) -> float:
  """Largest sideways acceleration, m/s^2, on the model's path within LAT_ACCEL_T."""
  t, yaw_rate, vx = model.orientationRate.t, model.orientationRate.z, model.velocity.x
  if len(t) == 0 or len(yaw_rate) != len(t) or len(vx) != len(t):
    return 0.0
  return max((abs(yaw_rate[i] * vx[i]) for i in range(len(t)) if t[i] <= LAT_ACCEL_T), default=0.0)


class AccelerationControl:
  def __init__(self, CP, params=None):
    self.available = CP.openpilotLongitudinalControl
    if params is None:
      from openpilot.common.params import Params
      params = Params()
    self.params = params
    self.enabled = False
    self.lift = 0.0  # smoothed lift, before curve control's interlock
    self.boost = 0.0  # lift applied this frame, published as accelerationControlBoostRequested
    self._frame = 0
    self._refresh()

  def _refresh(self) -> None:
    self.enabled = self.available and self.params.get_bool("AccelerationControl")

  def reset(self) -> None:
    self.lift = 0.0
    self.boost = 0.0

  def update(self, sm, output_a_target_e2e: float, a_cruise: float, throttle_scale: float = 1.0) -> float:
    if self._frame % PARAM_REFRESH_FRAMES == 0:
      self._refresh()
    self._frame += 1

    model = sm['modelV2']
    raw = model.action.desiredAcceleration
    if not self.enabled or not sm['selfdriveState'].enabled or model.action.shouldStop or raw < MODEL_BRAKING:
      self.reset()
      return output_a_target_e2e

    v_ego = sm['carState'].vEgo
    # same low-speed ramp as Gas Override Boost: none below 10 mph, full by 20 mph
    speed_scale = np.interp(v_ego, [ACCEL_BOOST_MIN_SPEED, 2 * ACCEL_BOOST_MIN_SPEED], [0.0, 1.0])
    fade = float(np.interp(raw, MODEL_FADE_BP, [0.0, 1.0]))
    fade *= float(np.interp(lat_accel_ahead(model), LAT_ACCEL_BP, [1.0, 0.0]))
    if raw < 0.0:
      # the model plans to slow down soon: leave its early slowing alone
      fade *= float(np.interp(planned_drop(model, v_ego), PLANNED_DROP_BP, [1.0, 0.0]))

    cap = ACCELERATION_CONTROL_BOOST.get(sm['selfdriveState'].personality.raw, ACCELERATION_CONTROL_BOOST[Personality.standard])
    target = float(np.clip(a_cruise - output_a_target_e2e, 0.0, cap * speed_scale * fade))
    self.lift = float(np.clip(target, self.lift - LIFT_FALL * DT_MDL, self.lift + LIFT_RISE * DT_MDL))

    # curve control's "don't accelerate into a saturated steer" interlock: 1.0 when inactive
    self.boost = self.lift * float(np.clip(throttle_scale, 0.0, 1.0))
    return output_a_target_e2e + self.boost
