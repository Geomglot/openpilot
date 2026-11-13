#!/usr/bin/env python3
import math
import numpy as np

import openpilot.cereal.messaging as messaging
from opendbc.car.interfaces import ACCEL_MIN, ACCEL_MAX
from openpilot.common.constants import CV
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.controls.lib.accel_boost import AccelBoost
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc, LongitudinalPlanSource
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import T_IDXS as T_IDXS_MPC
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N, get_accel_from_plan, should_stop
from openpilot.selfdrive.car.cruise import V_CRUISE_MAX, V_CRUISE_UNSET
from openpilot.common.swaglog import cloudlog

from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlannerSP, LongitudinalPlanSource as TargetSource
from openpilot.sunnypilot.selfdrive.controls.lib.acceleration_control import AccelerationControl

J_CRUISE_VALS = [1.6, 1.2, 0.8, 0.6]
J_CRUISE_BP = [0., 10.0, 25., 40.]
A_CRUISE_MIN = -0.5
CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]

# sunnypilot "Model controls speed" off: the cruise limits of the non-Experimental planner that upstream
# removed in #39097, restored unchanged.
A_CRUISE_MAX_VALS = [1.6, 1.2, 0.8, 0.6]
A_CRUISE_MAX_BP = [0., 10.0, 25., 40.]
_A_TOTAL_MAX_V = [1.7, 3.2]
_A_TOTAL_MAX_BP = [20., 40.]
# With the model out, cruise is the only way Curve Speed Control, Smart Cruise Control (map and vision) and Speed Limit
# Assist can slow the car, and they plan for more than A_CRUISE_MIN (map 1.2, vision 1.0, curve 1.8 m/s^2). While one
# of them sets the target, allow the pre-#39060 limit. A lower set speed on the stalk keeps the gentle A_CRUISE_MIN.
A_CRUISE_MIN_SPEED_CAP = -1.2


# "Model controls speed" can be switched while driving (a tap on the MAX box). The planner picks it up within half
# a second, never below MODEL_SWITCH_MIN_SPEED while openpilot is controlling the speed (switching off at a red light
# with no lead would set off towards the set speed), and eases the change in at MODEL_SWITCH_JERK instead of jumping.
MODEL_SWITCH_REFRESH_FRAMES = 10  # 0.5 s at the 20 Hz planner rate
MODEL_SWITCH_MIN_SPEED = 5.0  # m/s, about 11 mph; the on-screen tap needs 20 mph / 30 km/h, this is the backstop
MODEL_SWITCH_JERK = 1.0  # m/s^3


def get_max_accel(v_ego):
  return np.interp(v_ego, A_CRUISE_MAX_BP, A_CRUISE_MAX_VALS)


def limit_accel_in_turns(v_ego, angle_steers, CP):
  # leave room in the total grip budget for the lateral acceleration of the current turn
  a_total_max = np.interp(v_ego, _A_TOTAL_MAX_BP, _A_TOTAL_MAX_V)
  a_y = v_ego ** 2 * angle_steers * CV.DEG_TO_RAD / (CP.steerRatio * CP.wheelbase)
  return math.sqrt(max(a_total_max ** 2 - a_y ** 2, 0.))


def get_cruise_accel(v_cruise, v_ego, a_cruise_prev, dt, max_accel=ACCEL_MAX, min_accel=A_CRUISE_MIN):
  target_accel = np.clip(v_cruise - v_ego, min_accel, max_accel)
  j_cruise = np.interp(v_ego, J_CRUISE_BP, J_CRUISE_VALS)
  target_accel = float(np.clip(target_accel, a_cruise_prev - j_cruise * dt, a_cruise_prev + j_cruise * dt))

  return target_accel


class LongitudinalPlanner(LongitudinalPlannerSP):
  def __init__(self, CP, CP_SP, init_v=0.0, init_a=0.0, dt=DT_MDL):
    self.CP = CP
    self.mpc = LongitudinalMpc(dt=dt)
    LongitudinalPlannerSP.__init__(self, self.CP, CP_SP, self.mpc)
    self.fcw = False
    self.dt = dt
    self.accel_boost = AccelBoost()
    self.acceleration_control = AccelerationControl(CP)
    # Off: the model's acceleration request is left out of the planner, as in the old non-Experimental mode.
    # A key that was never written (fresh params, simulations, replay) must read as on, the upstream behaviour,
    # so use the registered default.
    self.params = Params()
    self.model_controls_speed = self._read_model_controls_speed()
    self.model_switch_frame = 0
    self.model_switch_blending = False
    # what the planner actually sent last frame. The easing works from this, not from a_prev: Smart Cruise Control
    # and Speed Limit Assist overwrite output_a_target with their own decel before a_prev is taken.
    self.model_switch_prev_a = init_a

    self.v_desired_filter = FirstOrderFilter(init_v, 2.0, self.dt)
    self.a_cruise = init_a
    self.output_a_target = init_a
    self.output_should_stop = False

    self.v_desired_trajectory = np.zeros(CONTROL_N)
    self.a_desired_trajectory = np.zeros(CONTROL_N)
    self.j_desired_trajectory = np.zeros(CONTROL_N)

  def _read_model_controls_speed(self) -> bool:
    return bool(self.params.get("ModelControlsSpeed", return_default=True))

  def _update_model_controls_speed(self, v_ego: float, reset_state: bool) -> None:
    if self.model_switch_frame % MODEL_SWITCH_REFRESH_FRAMES == 0:
      wanted = self._read_model_controls_speed()
      if wanted != self.model_controls_speed and (reset_state or v_ego >= MODEL_SWITCH_MIN_SPEED):
        self.model_controls_speed = wanted
        # ease in only while openpilot is controlling the speed; otherwise the state reset below covers it
        self.model_switch_blending = not reset_state
        if wanted:
          # the gas-override boost stopped learning while the model was out; start it again from zero,
          # and let Acceleration Control's lift build up again from zero
          self.accel_boost = AccelBoost()
          self.acceleration_control.reset()
    self.model_switch_frame += 1

  def update(self, sm):
    LongitudinalPlannerSP.update(self, sm)

    v_ego = sm['carState'].vEgo
    v_cruise_kph = min(sm['carState'].vCruise, V_CRUISE_MAX)
    v_cruise = v_cruise_kph * CV.KPH_TO_MS
    if sm['controlsState'].forceDecel:
      v_cruise = 0.0

    long_control_off = sm['controlsState'].longControlState == LongCtrlState.off

    # Reset current state when not engaged, or user is controlling the speed
    reset_state = long_control_off if self.CP.openpilotLongitudinalControl else not sm['selfdriveState'].enabled
    # PCM cruise speed may be updated a few cycles later, check if initialized
    v_cruise_initialized = sm['carState'].vCruise != V_CRUISE_UNSET
    reset_state = reset_state or not v_cruise_initialized

    self._update_model_controls_speed(v_ego, reset_state)

    if reset_state:
      self.model_switch_blending = False
      self.v_desired_filter.x = v_ego
      self.output_a_target = np.clip(sm['carState'].aEgo, ACCEL_MIN, ACCEL_MAX)
      self.a_cruise = self.output_a_target

    # Prevent divergence, smooth in current v_ego
    self.v_desired_filter.x = max(0.0, self.v_desired_filter.update(v_ego))

    # No change cost when user is controlling the speed, or when standstill
    prev_accel_constraint = not (reset_state or sm['carState'].standstill)

    # Get new v_cruise and a_target from Smart Cruise Control and Speed Limit Assist
    v_cruise, self.output_a_target = LongitudinalPlannerSP.update_targets(self, sm, self.v_desired_filter.x, self.output_a_target, v_cruise)

    self.mpc.set_weights(prev_accel_constraint, personality=sm['selfdriveState'].personality)
    if self.CP.openpilotLongitudinalControl:
      self.mpc.set_cur_state(self.v_desired_filter.x, self.output_a_target)
    else:
      self.mpc.set_cur_state(v_ego, sm['carState'].aEgo)
    self.mpc.update(sm['radarState'], personality=sm['selfdriveState'].personality)

    self.v_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.v_solution)
    self.a_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.a_solution)
    self.j_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC[:-1], self.mpc.j_solution)

    # TODO counter is only needed because radar is glitchy, remove once radar is gone
    self.fcw = self.mpc.crash_cnt > 2 and not sm['carState'].standstill
    if self.fcw:
      cloudlog.info("FCW triggered")

    # Save starting point for next iteration
    a_prev = self.output_a_target

    action_t =  self.CP.longitudinalActuatorDelay + DT_MDL
    output_a_target_mpc = get_accel_from_plan(self.v_desired_trajectory, self.a_desired_trajectory, CONTROL_N_T_IDX,
                                              action_t=action_t)
    output_should_stop_mpc = should_stop(v_ego, output_a_target_mpc)
    output_a_target_e2e = sm['modelV2'].action.desiredAcceleration
    output_should_stop_e2e = sm['modelV2'].action.shouldStop

    min_accel = A_CRUISE_MIN
    if self.model_controls_speed:
      max_accel = ACCEL_MAX
    else:
      steer_angle_without_offset = sm['carState'].steeringAngleDeg - sm['vehicleParameters'].angleOffsetDeg
      max_accel = min(get_max_accel(v_ego), limit_accel_in_turns(v_ego, steer_angle_without_offset, self.CP))
      if self.source != TargetSource.cruise:
        min_accel = A_CRUISE_MIN_SPEED_CAP
    self.a_cruise = get_cruise_accel(v_cruise, v_ego, self.a_cruise, self.dt, max_accel, min_accel)
    cruise_should_stop = should_stop(v_ego, self.a_cruise)

    candidates = [(output_a_target_mpc, self.mpc.source, output_should_stop_mpc),
                  (self.a_cruise, LongitudinalPlanSource.cruise, cruise_should_stop)]

    if self.model_controls_speed:
      # Acceleration Control first: Gas Override Boost latches "the model is limiting" from the candidate it is
      # given, so it has to see the lifted one or it learns a boost from overrides where the lead was the limit.
      output_a_target_e2e = self.acceleration_control.update(sm, output_a_target_e2e, self.a_cruise, self.scc.governor.throttle_scale())
      output_a_target_e2e = self.accel_boost.update(sm, output_a_target_e2e, output_a_target_mpc, self.a_cruise)
      if self.model_switch_blending:
        # just switched on: bring the model's braking in gradually. Only the model's request is held back;
        # braking for a lead car is never delayed.
        eased_e2e = max(output_a_target_e2e, self.model_switch_prev_a - MODEL_SWITCH_JERK * self.dt)
        self.model_switch_blending = eased_e2e != output_a_target_e2e
        output_a_target_e2e = eased_e2e
      candidates.append((output_a_target_e2e, LongitudinalPlanSource.e2e, output_should_stop_e2e))

    output_a_target, self.mpc.source, _ = min(candidates, key=lambda c: c[0])
    if self.model_switch_blending and not self.model_controls_speed:
      # just switched off: the model's braking went away, so let the acceleration rise gradually. This only ever
      # asks for less acceleration than the plan, never less braking.
      eased = min(output_a_target, self.model_switch_prev_a + MODEL_SWITCH_JERK * self.dt)
      self.model_switch_blending = eased != output_a_target
      output_a_target = eased
    self.output_should_stop = any(should_stop for _, _, should_stop in candidates)
    self.output_a_target = np.clip(output_a_target, ACCEL_MIN, ACCEL_MAX)
    self.model_switch_prev_a = float(self.output_a_target)

    self.v_desired_filter.x = self.v_desired_filter.x + self.dt * (self.output_a_target + a_prev) / 2.0

  def publish(self, sm, pm):
    plan_send = messaging.new_message('longitudinalPlan')

    plan_send.valid = sm.all_checks()

    longitudinalPlan = plan_send.longitudinalPlan
    longitudinalPlan.modelMonoTime = sm.logMonoTime['modelV2']
    longitudinalPlan.processingDelay = (plan_send.logMonoTime / 1e9) - sm.logMonoTime['modelV2']
    longitudinalPlan.solverExecutionTime = self.mpc.solve_time

    longitudinalPlan.speeds = self.v_desired_trajectory.tolist()
    longitudinalPlan.accels = self.a_desired_trajectory.tolist()
    longitudinalPlan.jerks = self.j_desired_trajectory.tolist()

    longitudinalPlan.hasLead = sm['radarState'].leadOne.present
    longitudinalPlan.longitudinalPlanSource = self.mpc.source
    longitudinalPlan.fcw = self.fcw

    longitudinalPlan.aTarget = float(self.output_a_target)
    longitudinalPlan.accelBoost = float(self.accel_boost.total_boost)
    longitudinalPlan.shouldStop = bool(self.output_should_stop)
    longitudinalPlan.allowBrake = True

    pm.send('longitudinalPlan', plan_send)

    self.publish_longitudinal_plan_sp(sm, pm, self.acceleration_control.boost)
