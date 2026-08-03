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
COUNTDOWN_S = 1.5        # placeholder -- warn-then-execute window, confirm before enabling live execution
COOLDOWN_S = 20.0        # minimum time between successive attempts, prevents re-trigger flapping


class AutoPassController:
  """
  Autonomously initiates a lane change to pass a closing lead vehicle -- no
  driver blinker/torque input required to arm it. Hard-gated on:
    - time-to-collision to the lead vehicle (radar)
    - a same-direction, oneway-tagged multi-lane road (OSM, fail-closed --
      see sunnypilot/mapd/live_map_data/osm_lane_data.py)
    - blind spot clear on the chosen side
  and only ever executes after a warn-then-execute countdown that aborts on
  braking, opposing steering pressure, blind spot occupancy, or the road
  losing its multi-lane validity mid-countdown.

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
  def _select_side(blindspot_left: bool, blindspot_right: bool) -> int:
    # US passing convention: prefer left when both sides are clear.
    if not blindspot_left:
      return LaneChangeDirection.left
    if not blindspot_right:
      return LaneChangeDirection.right
    return LaneChangeDirection.none

  @staticmethod
  def _blindspot_clear_for(direction: int, blindspot_left: bool, blindspot_right: bool) -> bool:
    if direction == LaneChangeDirection.left:
      return not blindspot_left
    if direction == LaneChangeDirection.right:
      return not blindspot_right
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

  def _idle(self) -> None:
    self.trigger_ready = False
    self.was_armed_by_us = False
    self.candidate_direction = LaneChangeDirection.none
    self.countdown_timer = 0.0
    self.execute_allowed = False
    self.should_abort = False

  def update(self, carstate, lead_one, multi_lane_valid: bool, multi_lane_same_direction: bool,
             below_lane_change_speed: bool) -> None:
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

      ready = (self.cooldown_timer <= 0 and not below_lane_change_speed and
               self.ttc < TTC_TRIGGER_S and self.multi_lane_same_direction)
      direction = self._select_side(carstate.leftBlindspot, carstate.rightBlindspot) if ready else LaneChangeDirection.none
      self.trigger_ready = ready and direction != LaneChangeDirection.none
      self.candidate_direction = direction if self.trigger_ready else LaneChangeDirection.none
      return

    self.trigger_ready = False

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
        self.cooldown_timer = COOLDOWN_S
        return

      self.countdown_timer += DT_MDL
      self.execute_allowed = (self.countdown_timer >= COUNTDOWN_S) and not self.shadow_mode
    else:
      # laneChangeStarting / laneChangeFinishing -- maneuver is underway, nothing left to gate.
      self.execute_allowed = False
      self.should_abort = False
      if lane_change_state == LaneChangeState.laneChangeFinishing:
        self.was_armed_by_us = False
        self.cooldown_timer = COOLDOWN_S

  @property
  def phase(self) -> int:
    """Telemetry-only derived phase for AutoPassStateSP; never used to drive control."""
    if not self.enabled:
      return AutoPassPhase.idle
    lane_change_state = self.DH.lane_change_state
    if lane_change_state == LaneChangeState.off:
      return AutoPassPhase.monitoring if self.trigger_ready else AutoPassPhase.idle
    if not self.was_armed_by_us:
      return AutoPassPhase.idle
    if lane_change_state == LaneChangeState.preLaneChange:
      return AutoPassPhase.countdown
    return AutoPassPhase.executing
