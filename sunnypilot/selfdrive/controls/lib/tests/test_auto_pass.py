"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from cereal import car, log

from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.desire_helper import DesireHelper, LaneChangeState, LaneChangeDirection
from openpilot.sunnypilot.selfdrive.controls.lib.auto_pass import COOLDOWN_S, COUNTDOWN_S, MIN_CLOSING_SPEED, TTC_TRIGGER_S


def make_carstate(left_blindspot=False, right_blindspot=False, brake_pressed=False,
                   steering_pressed=False, steering_torque=0.0):
  cs = car.CarState.new_message()
  cs.leftBlindspot = left_blindspot
  cs.rightBlindspot = right_blindspot
  cs.brakePressed = brake_pressed
  cs.steeringPressed = steering_pressed
  cs.steeringTorque = steering_torque
  return cs


def make_lead(dRel=30.0, vRel=-5.0, status=True):
  lead = log.RadarState.LeadData.new_message()
  lead.dRel = dRel
  lead.vRel = vRel
  lead.status = status
  return lead


CLOSING_LEAD = make_lead(dRel=10.0, vRel=-5.0, status=True)  # ttc = 2s, well under TTC_TRIGGER_S
FAR_LEAD = make_lead(dRel=200.0, vRel=-1.0, status=True)  # ttc = 200s, well over TTC_TRIGGER_S
NO_LEAD = make_lead(dRel=0.0, vRel=0.0, status=False)


class TestAutoPassController:
  def setup_method(self):
    self.DH = DesireHelper()
    self.ap = self.DH.auto_pass
    self._reset_states()

  def _reset_states(self):
    self.ap.enabled = True
    self.ap.shadow_mode = False
    self.ap.cooldown_timer = 0.0
    self.DH.lane_change_state = LaneChangeState.off
    self.DH.lane_change_direction = LaneChangeDirection.none

  def _clear_side_carstate(self):
    return make_carstate(left_blindspot=False, right_blindspot=False)

  def test_no_trigger_when_multi_lane_invalid_even_at_zero_ttc(self):
    """The single most important guarantee: unknown/invalid map data must never be
    treated as permissive, no matter how urgent the closing speed looks."""
    very_close_lead = make_lead(dRel=1.0, vRel=-50.0, status=True)
    self.ap.update(self._clear_side_carstate(), very_close_lead,
                   multi_lane_valid=False, multi_lane_same_direction=True, below_lane_change_speed=False)
    assert not self.ap.trigger_ready

    # Even a "same_direction=True" claim from a caller must be ignored if valid=False.
    self.ap.update(self._clear_side_carstate(), very_close_lead,
                   multi_lane_valid=False, multi_lane_same_direction=False, below_lane_change_speed=False)
    assert not self.ap.trigger_ready

  def test_no_trigger_when_multi_lane_valid_but_not_same_direction(self):
    self.ap.update(self._clear_side_carstate(), CLOSING_LEAD,
                   multi_lane_valid=True, multi_lane_same_direction=False, below_lane_change_speed=False)
    assert not self.ap.trigger_ready

  def test_no_trigger_when_ttc_above_threshold(self):
    self.ap.update(self._clear_side_carstate(), FAR_LEAD,
                   multi_lane_valid=True, multi_lane_same_direction=True, below_lane_change_speed=False)
    assert not self.ap.trigger_ready
    assert self.ap.ttc > TTC_TRIGGER_S

  def test_no_trigger_when_no_lead(self):
    self.ap.update(self._clear_side_carstate(), NO_LEAD,
                   multi_lane_valid=True, multi_lane_same_direction=True, below_lane_change_speed=False)
    assert not self.ap.trigger_ready
    assert self.ap.ttc == float('inf')

  def test_no_trigger_when_below_lane_change_speed(self):
    self.ap.update(self._clear_side_carstate(), CLOSING_LEAD,
                   multi_lane_valid=True, multi_lane_same_direction=True, below_lane_change_speed=True)
    assert not self.ap.trigger_ready

  def test_no_trigger_when_disabled(self):
    self.ap.enabled = False
    self.ap.update(self._clear_side_carstate(), CLOSING_LEAD,
                   multi_lane_valid=True, multi_lane_same_direction=True, below_lane_change_speed=False)
    assert not self.ap.trigger_ready
    assert not self.ap.was_armed_by_us

  def test_trigger_ready_prefers_left_when_both_clear(self):
    self.ap.update(self._clear_side_carstate(), CLOSING_LEAD,
                   multi_lane_valid=True, multi_lane_same_direction=True, below_lane_change_speed=False)
    assert self.ap.trigger_ready
    assert self.ap.candidate_direction == LaneChangeDirection.left

  def test_trigger_ready_falls_back_to_right_when_left_occupied(self):
    cs = make_carstate(left_blindspot=True, right_blindspot=False)
    self.ap.update(cs, CLOSING_LEAD, multi_lane_valid=True, multi_lane_same_direction=True, below_lane_change_speed=False)
    assert self.ap.trigger_ready
    assert self.ap.candidate_direction == LaneChangeDirection.right

  def test_no_trigger_when_both_sides_occupied(self):
    cs = make_carstate(left_blindspot=True, right_blindspot=True)
    self.ap.update(cs, CLOSING_LEAD, multi_lane_valid=True, multi_lane_same_direction=True, below_lane_change_speed=False)
    assert not self.ap.trigger_ready
    assert self.ap.candidate_direction == LaneChangeDirection.none

  def _arm(self):
    """Drive the off -> preLaneChange transition the way DesireHelper does."""
    self.ap.update(self._clear_side_carstate(), CLOSING_LEAD,
                   multi_lane_valid=True, multi_lane_same_direction=True, below_lane_change_speed=False)
    assert self.ap.trigger_ready
    self.DH.lane_change_direction = self.ap.candidate_direction
    self.DH.lane_change_state = LaneChangeState.preLaneChange
    self.ap.mark_triggered()

  def test_execute_allowed_only_after_full_clean_countdown(self):
    self._arm()

    # Safely below the threshold, with margin for float accumulation drift.
    num_updates_below = int(COUNTDOWN_S / DT_MDL) - 2
    for _ in range(num_updates_below):
      self.ap.update(self._clear_side_carstate(), CLOSING_LEAD,
                     multi_lane_valid=True, multi_lane_same_direction=True, below_lane_change_speed=False)
      assert not self.ap.execute_allowed

    # Enough additional updates to guarantee crossing the threshold.
    for _ in range(5):
      self.ap.update(self._clear_side_carstate(), CLOSING_LEAD,
                     multi_lane_valid=True, multi_lane_same_direction=True, below_lane_change_speed=False)
    assert self.ap.execute_allowed

  def test_shadow_mode_clamps_execute_allowed(self):
    self.ap.shadow_mode = True
    self._arm()

    num_updates = int(COUNTDOWN_S / DT_MDL) + 2
    for _ in range(num_updates):
      self.ap.update(self._clear_side_carstate(), CLOSING_LEAD,
                     multi_lane_valid=True, multi_lane_same_direction=True, below_lane_change_speed=False)
      assert not self.ap.execute_allowed
      assert not self.ap.should_abort

  def test_abort_on_brake(self):
    self._arm()
    cs = make_carstate(brake_pressed=True)
    self.ap.update(cs, CLOSING_LEAD, multi_lane_valid=True, multi_lane_same_direction=True, below_lane_change_speed=False)
    assert self.ap.should_abort
    assert not self.ap.execute_allowed
    assert self.ap.cooldown_timer > 0

  def test_abort_on_opposing_steering(self):
    self._arm()  # candidate_direction is left (both sides clear)
    cs = make_carstate(steering_pressed=True, steering_torque=-1.0)  # torque opposing left
    self.ap.update(cs, CLOSING_LEAD, multi_lane_valid=True, multi_lane_same_direction=True, below_lane_change_speed=False)
    assert self.ap.should_abort

  def test_no_abort_on_agreeing_steering(self):
    self._arm()
    cs = make_carstate(steering_pressed=True, steering_torque=1.0)  # torque agreeing with left
    self.ap.update(cs, CLOSING_LEAD, multi_lane_valid=True, multi_lane_same_direction=True, below_lane_change_speed=False)
    assert not self.ap.should_abort

  def test_abort_on_blindspot_occupied(self):
    self._arm()  # candidate_direction is left
    cs = make_carstate(left_blindspot=True)
    self.ap.update(cs, CLOSING_LEAD, multi_lane_valid=True, multi_lane_same_direction=True, below_lane_change_speed=False)
    assert self.ap.should_abort
    assert not self.ap.blindspot_clear

  def test_abort_on_multi_lane_invalidated_mid_countdown(self):
    self._arm()
    self.ap.update(self._clear_side_carstate(), CLOSING_LEAD,
                   multi_lane_valid=False, multi_lane_same_direction=True, below_lane_change_speed=False)
    assert self.ap.should_abort
    assert self.ap.cooldown_timer > 0

  def test_abort_just_before_threshold_still_aborts(self):
    self._arm()
    num_updates_below = int(COUNTDOWN_S / DT_MDL) - 2
    for _ in range(num_updates_below):
      self.ap.update(self._clear_side_carstate(), CLOSING_LEAD,
                     multi_lane_valid=True, multi_lane_same_direction=True, below_lane_change_speed=False)
      assert not self.ap.execute_allowed

    # Brake right before the countdown would otherwise complete -- must still abort.
    cs = make_carstate(brake_pressed=True)
    self.ap.update(cs, CLOSING_LEAD, multi_lane_valid=True, multi_lane_same_direction=True, below_lane_change_speed=False)
    assert self.ap.should_abort
    assert not self.ap.execute_allowed

  def test_disabled_forces_idle_from_any_phase(self):
    self._arm()
    self.ap.update(self._clear_side_carstate(), CLOSING_LEAD,
                   multi_lane_valid=True, multi_lane_same_direction=True, below_lane_change_speed=False)
    assert self.ap.was_armed_by_us

    self.ap.enabled = False
    self.ap.update(self._clear_side_carstate(), CLOSING_LEAD,
                   multi_lane_valid=True, multi_lane_same_direction=True, below_lane_change_speed=False)
    assert not self.ap.was_armed_by_us
    assert not self.ap.execute_allowed
    assert not self.ap.trigger_ready

  def test_cooldown_prevents_immediate_retrigger(self):
    self._arm()
    cs = make_carstate(brake_pressed=True)
    self.ap.update(cs, CLOSING_LEAD, multi_lane_valid=True, multi_lane_same_direction=True, below_lane_change_speed=False)
    assert self.ap.should_abort
    assert self.ap.cooldown_timer == COOLDOWN_S

    # DesireHelper would drop back to off after should_abort; simulate that.
    self.DH.lane_change_state = LaneChangeState.off
    self.DH.lane_change_direction = LaneChangeDirection.none

    self.ap.update(self._clear_side_carstate(), CLOSING_LEAD,
                   multi_lane_valid=True, multi_lane_same_direction=True, below_lane_change_speed=False)
    assert not self.ap.trigger_ready

    # After the cooldown elapses, it should be eligible again.
    num_updates = int(COOLDOWN_S / DT_MDL) + 1
    for _ in range(num_updates):
      self.ap.update(self._clear_side_carstate(), CLOSING_LEAD,
                     multi_lane_valid=True, multi_lane_same_direction=True, below_lane_change_speed=False)
    assert self.ap.trigger_ready

  def test_execute_allowed_false_once_maneuver_underway(self):
    self._arm()
    num_updates = int(COUNTDOWN_S / DT_MDL) + 1
    for _ in range(num_updates):
      self.ap.update(self._clear_side_carstate(), CLOSING_LEAD,
                     multi_lane_valid=True, multi_lane_same_direction=True, below_lane_change_speed=False)
    assert self.ap.execute_allowed

    self.DH.lane_change_state = LaneChangeState.laneChangeStarting
    self.ap.update(self._clear_side_carstate(), CLOSING_LEAD,
                   multi_lane_valid=True, multi_lane_same_direction=True, below_lane_change_speed=False)
    assert not self.ap.execute_allowed

  def test_cooldown_starts_and_ownership_clears_on_finish(self):
    self._arm()
    self.DH.lane_change_state = LaneChangeState.laneChangeFinishing
    self.ap.update(self._clear_side_carstate(), CLOSING_LEAD,
                   multi_lane_valid=True, multi_lane_same_direction=True, below_lane_change_speed=False)
    assert not self.ap.was_armed_by_us
    assert self.ap.cooldown_timer > 0

  def test_ttc_ignores_lead_below_min_closing_speed(self):
    slow_lead = make_lead(dRel=5.0, vRel=-(MIN_CLOSING_SPEED - 0.01), status=True)
    self.ap.update(self._clear_side_carstate(), slow_lead,
                   multi_lane_valid=True, multi_lane_same_direction=True, below_lane_change_speed=False)
    assert self.ap.ttc == float('inf')
    assert not self.ap.trigger_ready

  def test_driver_initiated_lane_change_is_untouched(self):
    """A blinker-driven change (was_armed_by_us stays False) must never be gated
    or aborted by Auto Pass logic."""
    self.DH.lane_change_state = LaneChangeState.preLaneChange
    cs = make_carstate(brake_pressed=True)  # would abort an auto-pass-owned change
    self.ap.update(cs, CLOSING_LEAD, multi_lane_valid=True, multi_lane_same_direction=True, below_lane_change_speed=False)
    assert not self.ap.was_armed_by_us
    assert not self.ap.execute_allowed
