"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from cereal import log, custom

from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL

LaneChangeDirection = log.LaneChangeDirection
LaneChangeState = log.LaneChangeState
AutoPassPhase = custom.AutoPassStateSP.Phase

MIN_CLOSING_SPEED = 1.0  # m/s; ignore near-zero closing rate so long-range noise doesn't trigger
TTC_TRIGGER_S = 8.0      # placeholder -- needs shadow-mode log tuning before live use
COOLDOWN_S = 20.0        # minimum time between successive attempts, prevents re-trigger flapping

# Sustained-closing requirement before the outbound trigger arms: a single frame
# of ttc < TTC_TRIGGER_S is not enough to arm on its own. Matters more than it
# would on a radar-equipped car -- this platform (2020-21 Silverado/Sierra 1500,
# GM's CAMERA_ACC_CAR set, radarUnavailable=True) has no physical radar, so
# radarState.leadOne is always vision-derived (see radard.py's
# get_RadarState_from_vision fallback), and vision-based vRel is noisier than a
# direct Doppler measurement -- a momentary estimation spike must not be able to
# arm a lane change on its own.
TTC_DWELL_S = 1.0  # placeholder -- needs shadow-mode tuning against this platform's actual vision-lead noise

# Gaze-confirm: mirrors GM Super Cruise / BMW Active Lane Change Assist -- driver
# glances toward the mirror on the side of the maneuver to confirm it, rather than
# the maneuver auto-executing off a bare timer. Left confirms the outbound pass;
# right confirms returning to the original lane afterward.
GAZE_YAW_THRESHOLD_RAD = 0.26   # ~15 deg; placeholder -- needs bench tuning against DRIVER_MONITOR_SETTINGS yaw thresholds
GAZE_MAX_UNCERTAINTY = 0.3      # placeholder -- align with monitoring policy's model_std_max gating once tuned
GAZE_CONFIRM_DWELL_S = 0.5      # sustained glance required so a flick can't confirm
CONFIRM_WINDOW_S = 4.0          # max time to wait for a confirming glance before the opportunity lapses (abort, not auto-execute)
# NOTE: sign of driverMonitoringState.visionPolicyState.pose.yaw for "looking left"
# vs "looking right" is UNVERIFIED against real hardware -- confirm on the bench
# (watch_dm.py-style live readout while glancing at each mirror) before relying on
# this for a live confirm gesture. Flip if left/right come out swapped.
GAZE_LEFT_SIGN = -1

# Return-to-lane: after a completed left auto-pass, how long the return side must
# stay clear before we arm a return countdown at all.
RETURN_MIN_CLEAR_S = 3.0  # placeholder -- needs shadow-mode tuning


class AutoPassController:
  """
  Autonomously arms a lane change to pass a closing lead vehicle -- no driver
  blinker/torque input required to arm it. Hard-gated on:
    - time-to-collision to the lead vehicle (radar)
    - a same-direction, oneway-tagged multi-lane road (OSM, fail-closed --
      see sunnypilot/mapd/live_map_data/osm_lane_data.py)
    - blind spot clear on the chosen side
  and only ever executes once the driver confirms with a sustained glance
  toward the mirror on the maneuver's side (left to pass, right to return --
  see GAZE_* constants above), within a bounded confirm window. Aborts on
  braking, opposing steering pressure, blind spot occupancy, the road losing
  its multi-lane validity mid-countdown, or the confirm window lapsing
  without a qualifying glance.

  After a left pass completes, this also monitors for a safe opportunity to
  return to the original lane (right blind spot clear for RETURN_MIN_CLEAR_S)
  and, once found, arms a right-direction maneuver through the same
  preLaneChange/gaze-confirm path. US-only: passing only ever goes left;
  right-direction maneuvers here are exclusively the return leg, never a
  fresh pass.

  This never drives openpilot.selfdrive.controls.lib.desire_helper.DesireHelper's
  LaneChangeState machine directly -- it only exposes `trigger_ready` (read once,
  from LaneChangeState.off) and `execute_allowed`/`should_abort` (read while
  DesireHelper is in preLaneChange because of us, tracked via `was_armed_by_us`).
  DesireHelper.lane_change_state remains the single source of truth for whether
  a maneuver is in progress.
  """

  def __init__(self, desire_helper):
    self.DH = desire_helper
    self.params = Params()

    self.enabled = False
    self.shadow_mode = True
    self.param_read_counter = 0
    self.read_params()

    self.trigger_ready = False
    self.was_armed_by_us = False
    self.candidate_direction = LaneChangeDirection.none
    self.countdown_timer = 0.0
    self.cooldown_timer = 0.0
    self.execute_allowed = False
    self.should_abort = False

    self.gaze_confirmed = False
    self.gaze_dwell_timer = 0.0

    self.ttc_dwell_timer = 0.0

    self.return_pending = False
    self.return_clear_timer = 0.0
    self.return_trigger_ready = False

    self.ttc = float('inf')
    self.v_rel = 0.0
    self.multi_lane_valid = False
    self.multi_lane_same_direction = False
    self.blindspot_clear = False

  def read_params(self) -> None:
    self.enabled = self.params.get_bool("AutoPassEnabled")
    self.shadow_mode = self.params.get_bool("AutoPassShadowMode")

  def update_params(self) -> None:
    if self.param_read_counter % 50 == 0:
      self.read_params()
    self.param_read_counter += 1

  @staticmethod
  def _select_side(blindspot_left: bool) -> int:
    # US-only: a fresh pass only ever goes left. Right is exclusively the
    # return leg, armed separately once a left pass has completed.
    return LaneChangeDirection.left if not blindspot_left else LaneChangeDirection.none

  @staticmethod
  def _blindspot_clear_for(direction: int, blindspot_left: bool, blindspot_right: bool) -> bool:
    if direction == LaneChangeDirection.left:
      return not blindspot_left
    if direction == LaneChangeDirection.right:
      return not blindspot_right
    return False

  @staticmethod
  def _gaze_toward(direction: int, driver_yaw: float) -> bool:
    """True if driver_yaw is past GAZE_YAW_THRESHOLD_RAD toward the given side."""
    signed_yaw = driver_yaw * GAZE_LEFT_SIGN
    if direction == LaneChangeDirection.left:
      return signed_yaw > GAZE_YAW_THRESHOLD_RAD
    if direction == LaneChangeDirection.right:
      return signed_yaw < -GAZE_YAW_THRESHOLD_RAD
    return False

  @staticmethod
  def _compute_ttc(lead_one) -> float:
    if lead_one is None or not (lead_one.status or lead_one.radar):
      return float('inf')
    if lead_one.vRel >= -MIN_CLOSING_SPEED:
      return float('inf')
    return -lead_one.dRel / lead_one.vRel

  def mark_triggered(self) -> None:
    """Called by DesireHelper exactly when it arms preLaneChange because of us."""
    self.was_armed_by_us = True
    self.countdown_timer = 0.0
    self.execute_allowed = False
    self.should_abort = False
    self.trigger_ready = False
    self.return_trigger_ready = False
    self.gaze_confirmed = False
    self.gaze_dwell_timer = 0.0
    self.ttc_dwell_timer = 0.0

  def _idle(self) -> None:
    self.trigger_ready = False
    self.was_armed_by_us = False
    self.candidate_direction = LaneChangeDirection.none
    self.countdown_timer = 0.0
    self.execute_allowed = False
    self.should_abort = False
    self.gaze_confirmed = False
    self.gaze_dwell_timer = 0.0
    self.ttc_dwell_timer = 0.0
    self.return_pending = False
    self.return_clear_timer = 0.0
    self.return_trigger_ready = False

  def update(self, carstate, lead_one, multi_lane_valid: bool, multi_lane_same_direction: bool,
             below_lane_change_speed: bool, driver_yaw: float, driver_yaw_uncertainty: float,
             driver_pose_calibrated: bool) -> None:
    self.multi_lane_valid = multi_lane_valid
    self.multi_lane_same_direction = bool(multi_lane_valid and multi_lane_same_direction)
    self.ttc = self._compute_ttc(lead_one)
    self.v_rel = lead_one.vRel if lead_one is not None else 0.0

    if self.cooldown_timer > 0:
      self.cooldown_timer = max(0.0, self.cooldown_timer - DT_MDL)

    if not self.enabled:
      self._idle()
      return

    lane_change_state = self.DH.lane_change_state

    if lane_change_state == LaneChangeState.off:
      self.was_armed_by_us = False
      self.countdown_timer = 0.0
      self.execute_allowed = False
      self.should_abort = False
      self.gaze_confirmed = False
      self.gaze_dwell_timer = 0.0

      if self.return_pending:
        # Watching for a safe gap to return to the original lane after a
        # completed left pass -- don't also evaluate a fresh outbound trigger
        # while this is outstanding.
        return_clear = (self.cooldown_timer <= 0 and not below_lane_change_speed and
                         not carstate.rightBlindspot and self.multi_lane_same_direction)
        self.return_clear_timer = self.return_clear_timer + DT_MDL if return_clear else 0.0
        self.return_trigger_ready = self.return_clear_timer >= RETURN_MIN_CLEAR_S
        self.trigger_ready = False
        self.candidate_direction = LaneChangeDirection.right if self.return_trigger_ready else LaneChangeDirection.none
        return

      self.return_trigger_ready = False
      ttc_condition = (self.cooldown_timer <= 0 and not below_lane_change_speed and
                        self.ttc < TTC_TRIGGER_S and self.multi_lane_same_direction)
      self.ttc_dwell_timer = self.ttc_dwell_timer + DT_MDL if ttc_condition else 0.0
      ready = self.ttc_dwell_timer >= TTC_DWELL_S
      direction = self._select_side(carstate.leftBlindspot) if ready else LaneChangeDirection.none
      self.trigger_ready = ready and direction != LaneChangeDirection.none
      self.candidate_direction = direction if self.trigger_ready else LaneChangeDirection.none
      return

    self.trigger_ready = False
    self.return_trigger_ready = False

    if not self.was_armed_by_us:
      # A driver-initiated (blinker) lane change is in progress -- not ours to touch.
      return

    if lane_change_state == LaneChangeState.preLaneChange:
      blindspot_clear = self._blindspot_clear_for(self.candidate_direction, carstate.leftBlindspot, carstate.rightBlindspot)
      self.blindspot_clear = blindspot_clear

      opposing_torque = carstate.steeringPressed and (
        (carstate.steeringTorque > 0 and self.candidate_direction == LaneChangeDirection.right) or
        (carstate.steeringTorque < 0 and self.candidate_direction == LaneChangeDirection.left))

      self.should_abort = bool(carstate.brakePressed or opposing_torque or not blindspot_clear or
                                not self.multi_lane_same_direction or below_lane_change_speed)

      if self.should_abort:
        self.execute_allowed = False
        self.countdown_timer = 0.0
        self.gaze_confirmed = False
        self.gaze_dwell_timer = 0.0
        self.cooldown_timer = COOLDOWN_S
        # return_pending, if set, is deliberately left alone here -- an aborted
        # return attempt still owes a return to the original lane; cooldown
        # paces the retry rather than abandoning it (which would let a second
        # outbound pass trigger while we're still out in the passing lane).
        return

      self.countdown_timer += DT_MDL

      gaze_ok = (driver_pose_calibrated and driver_yaw_uncertainty < GAZE_MAX_UNCERTAINTY and
                 self._gaze_toward(self.candidate_direction, driver_yaw))
      self.gaze_dwell_timer = self.gaze_dwell_timer + DT_MDL if gaze_ok else 0.0
      self.gaze_confirmed = self.gaze_dwell_timer >= GAZE_CONFIRM_DWELL_S

      if self.gaze_confirmed:
        self.execute_allowed = not self.shadow_mode
      elif self.countdown_timer >= CONFIRM_WINDOW_S:
        # Confirm window lapsed without a qualifying glance -- the opportunity
        # lapses, it does not silently auto-execute.
        self.should_abort = True
        self.execute_allowed = False
        self.countdown_timer = 0.0
        self.cooldown_timer = COOLDOWN_S
        # Same as above: return_pending stays set so a lapsed return-confirm
        # window retries after cooldown instead of being abandoned.
      else:
        self.execute_allowed = False
    else:
      # laneChangeStarting / laneChangeFinishing -- maneuver is underway, nothing left to gate.
      self.execute_allowed = False
      self.should_abort = False
      if lane_change_state == LaneChangeState.laneChangeFinishing:
        was_return = self.return_pending and self.candidate_direction == LaneChangeDirection.right
        self.was_armed_by_us = False
        self.cooldown_timer = COOLDOWN_S
        if was_return:
          self.return_pending = False
          self.return_clear_timer = 0.0
        elif self.candidate_direction == LaneChangeDirection.left:
          self.return_pending = True  # completed a left pass -- now watch for a safe return gap

  @property
  def phase(self) -> int:
    """Telemetry-only derived phase for AutoPassStateSP; never used to drive control."""
    if not self.enabled:
      return AutoPassPhase.idle
    lane_change_state = self.DH.lane_change_state
    if lane_change_state == LaneChangeState.off:
      return AutoPassPhase.monitoring if (self.trigger_ready or self.return_trigger_ready) else AutoPassPhase.idle
    if not self.was_armed_by_us:
      return AutoPassPhase.idle
    if lane_change_state == LaneChangeState.preLaneChange:
      return AutoPassPhase.countdown
    return AutoPassPhase.executing
