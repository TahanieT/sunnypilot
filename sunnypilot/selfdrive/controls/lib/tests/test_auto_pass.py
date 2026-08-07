"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from cereal import car, log

from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.desire_helper import DesireHelper, LaneChangeState, LaneChangeDirection
from openpilot.sunnypilot.selfdrive.controls.lib.auto_pass import (
  COOLDOWN_S, CONFIRM_WINDOW_S, GAZE_CONFIRM_DWELL_S, GAZE_LEFT_SIGN, GAZE_MAX_UNCERTAINTY,
  GAZE_YAW_THRESHOLD_RAD, MIN_CLOSING_SPEED, RETURN_MIN_CLEAR_S, TTC_DWELL_S, TTC_TRIGGER_S,
)
# _stub_auto_pass_params in conftest.py (autouse) bypasses the stale compiled
# params key-validation table for every test in this directory -- see its
# docstring for why.


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

# Sign-convention-agnostic: derived from GAZE_LEFT_SIGN (bench-confirmed, see
# auto_pass.py) rather than hardcoded, so these stay correct if it's ever
# revisited. These raw driver_yaw values confirm the stated direction, and
# CONFIRMED_UNCERTAINTY always passes the confidence gate.
_MARGIN = 0.1
YAW_LEFT = GAZE_LEFT_SIGN * (GAZE_YAW_THRESHOLD_RAD + _MARGIN)
YAW_RIGHT = -GAZE_LEFT_SIGN * (GAZE_YAW_THRESHOLD_RAD + _MARGIN)
YAW_NEUTRAL = 0.0
CONFIRMED_UNCERTAINTY = GAZE_MAX_UNCERTAINTY / 2
UNCONFIRMED_UNCERTAINTY = GAZE_MAX_UNCERTAINTY * 2

_GAZE_YAW = {LaneChangeDirection.left: YAW_LEFT, LaneChangeDirection.right: YAW_RIGHT, None: YAW_NEUTRAL}


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

  def _update(self, cs, lead=CLOSING_LEAD, multi_lane_valid=True, multi_lane_same_direction=True,
              below_lane_change_speed=False, gaze_direction=None, uncertainty=CONFIRMED_UNCERTAINTY,
              calibrated=True):
    self.ap.update(cs, lead, multi_lane_valid, multi_lane_same_direction, below_lane_change_speed,
                   _GAZE_YAW[gaze_direction], uncertainty, calibrated)

  def test_no_trigger_when_multi_lane_invalid_even_at_zero_ttc(self):
    """The single most important guarantee: unknown/invalid map data must never be
    treated as permissive, no matter how urgent the closing speed looks."""
    very_close_lead = make_lead(dRel=1.0, vRel=-50.0, status=True)
    self._update(self._clear_side_carstate(), very_close_lead, multi_lane_valid=False, multi_lane_same_direction=True)
    assert not self.ap.trigger_ready

    # Even a "same_direction=True" claim from a caller must be ignored if valid=False.
    self._update(self._clear_side_carstate(), very_close_lead, multi_lane_valid=False, multi_lane_same_direction=False)
    assert not self.ap.trigger_ready

  def test_no_trigger_when_multi_lane_valid_but_not_same_direction(self):
    self._update(self._clear_side_carstate(), CLOSING_LEAD, multi_lane_valid=True, multi_lane_same_direction=False)
    assert not self.ap.trigger_ready

  def test_no_trigger_when_ttc_above_threshold(self):
    self._update(self._clear_side_carstate(), FAR_LEAD)
    assert not self.ap.trigger_ready
    assert self.ap.ttc > TTC_TRIGGER_S

  def test_no_trigger_when_no_lead(self):
    self._update(self._clear_side_carstate(), NO_LEAD)
    assert not self.ap.trigger_ready
    assert self.ap.ttc == float('inf')

  def test_no_trigger_when_below_lane_change_speed(self):
    self._update(self._clear_side_carstate(), CLOSING_LEAD, below_lane_change_speed=True)
    assert not self.ap.trigger_ready

  def test_no_trigger_when_disabled(self):
    self.ap.enabled = False
    self._update(self._clear_side_carstate(), CLOSING_LEAD)
    assert not self.ap.trigger_ready
    assert not self.ap.was_armed_by_us

  def _dwell_ttc(self, cs, lead=CLOSING_LEAD, margin_updates=2):
    """Sustained closing for just over TTC_DWELL_S -- a single frame under
    threshold must not be enough to trigger, guarding against noisy
    vision-derived vRel on this radar-unavailable (CAMERA_ACC_CAR) platform."""
    num_updates = int(TTC_DWELL_S / DT_MDL) + margin_updates
    for _ in range(num_updates):
      self._update(cs, lead)

  def test_single_frame_of_closing_ttc_does_not_trigger(self):
    self._update(self._clear_side_carstate(), CLOSING_LEAD)
    assert not self.ap.trigger_ready

  def test_trigger_ready_left_after_sustained_closing(self):
    self._dwell_ttc(self._clear_side_carstate())
    assert self.ap.trigger_ready
    assert self.ap.candidate_direction == LaneChangeDirection.left

  def test_no_trigger_when_left_occupied(self):
    """US-only: a fresh pass never goes right, even if the left is blocked and
    the right is clear -- right is exclusively the return leg."""
    cs = make_carstate(left_blindspot=True, right_blindspot=False)
    self._dwell_ttc(cs)
    assert not self.ap.trigger_ready
    assert self.ap.candidate_direction == LaneChangeDirection.none

  def test_no_trigger_when_both_sides_occupied(self):
    cs = make_carstate(left_blindspot=True, right_blindspot=True)
    self._dwell_ttc(cs)
    assert not self.ap.trigger_ready
    assert self.ap.candidate_direction == LaneChangeDirection.none

  def test_ttc_dwell_resets_if_closing_condition_drops(self):
    """A brief closing spike followed by the condition dropping out (e.g. a
    noisy vision vRel blip) must not leave partial credit toward the dwell."""
    num_updates_below = max(int(TTC_DWELL_S / DT_MDL) - 2, 1)
    for _ in range(num_updates_below):
      self._update(self._clear_side_carstate(), CLOSING_LEAD)
    assert not self.ap.trigger_ready

    self._update(self._clear_side_carstate(), FAR_LEAD)  # condition drops out
    assert self.ap.ttc_dwell_timer == 0.0

  def _arm(self):
    """Drive the off -> preLaneChange transition the way DesireHelper does."""
    self._dwell_ttc(self._clear_side_carstate())
    assert self.ap.trigger_ready
    assert self.ap.candidate_direction == LaneChangeDirection.left
    self.DH.lane_change_direction = self.ap.candidate_direction
    self.DH.lane_change_state = LaneChangeState.preLaneChange
    self.ap.mark_triggered()

  def _confirm(self, direction, margin_updates=2):
    """Feed a qualifying, sustained glance toward `direction` for just over the
    dwell requirement."""
    num_updates = int(GAZE_CONFIRM_DWELL_S / DT_MDL) + margin_updates
    for _ in range(num_updates):
      self._update(self._clear_side_carstate(), CLOSING_LEAD, gaze_direction=direction)

  def test_execute_allowed_only_after_gaze_confirmed(self):
    self._arm()
    assert not self.ap.execute_allowed

    # Not looking anywhere -- countdown running, but never confirms.
    for _ in range(5):
      self._update(self._clear_side_carstate(), CLOSING_LEAD)
      assert not self.ap.execute_allowed
      assert not self.ap.gaze_confirmed

    self._confirm(LaneChangeDirection.left)
    assert self.ap.gaze_confirmed
    assert self.ap.execute_allowed

  def test_brief_glance_under_dwell_does_not_confirm(self):
    self._arm()
    num_updates_below = max(int(GAZE_CONFIRM_DWELL_S / DT_MDL) - 2, 1)
    for _ in range(num_updates_below):
      self._update(self._clear_side_carstate(), CLOSING_LEAD, gaze_direction=LaneChangeDirection.left)
    assert not self.ap.gaze_confirmed
    assert not self.ap.execute_allowed

    # Look away before dwell completes -- timer must reset, not just pause.
    self._update(self._clear_side_carstate(), CLOSING_LEAD, gaze_direction=None)
    assert self.ap.gaze_dwell_timer == 0.0

  def _run_until_abort(self, gaze_direction=None, uncertainty=CONFIRMED_UNCERTAINTY, calibrated=True, max_updates=None):
    """Loop _update calls, stopping the instant should_abort fires -- mirrors
    how DesireHelper reacts within the same cycle in the real system
    (auto_pass.update() would never be called again in preLaneChange after
    that). should_abort is recomputed fresh from the hard conditions at the
    top of every preLaneChange frame, so a bare loop that keeps calling
    update() past the abort frame would see it reset back to False, unlike
    the real system which exits preLaneChange immediately."""
    if max_updates is None:
      max_updates = int(CONFIRM_WINDOW_S / DT_MDL) + 2
    for _ in range(max_updates):
      self._update(self._clear_side_carstate(), CLOSING_LEAD, gaze_direction=gaze_direction,
                   uncertainty=uncertainty, calibrated=calibrated)
      if self.ap.should_abort:
        return True
    return False

  def test_right_gaze_does_not_confirm_left_pass(self):
    self._arm()  # candidate_direction is left
    aborted = self._run_until_abort(gaze_direction=LaneChangeDirection.right)
    assert not self.ap.gaze_confirmed
    assert not self.ap.execute_allowed
    assert aborted  # confirm window lapsed without a qualifying glance

  def test_gaze_confirm_requires_calibrated_pose(self):
    self._arm()
    aborted = self._run_until_abort(gaze_direction=LaneChangeDirection.left, calibrated=False)
    assert not self.ap.gaze_confirmed
    assert aborted

  def test_gaze_confirm_requires_low_uncertainty(self):
    self._arm()
    aborted = self._run_until_abort(gaze_direction=LaneChangeDirection.left, uncertainty=UNCONFIRMED_UNCERTAINTY)
    assert not self.ap.gaze_confirmed
    assert aborted

  def test_confirm_window_lapses_without_gaze_aborts(self):
    self._arm()
    aborted = self._run_until_abort()
    assert aborted
    assert not self.ap.execute_allowed
    assert self.ap.cooldown_timer > 0

  def test_shadow_mode_clamps_execute_allowed(self):
    self.ap.shadow_mode = True
    self._arm()
    self._confirm(LaneChangeDirection.left)
    assert self.ap.gaze_confirmed
    assert not self.ap.execute_allowed
    assert not self.ap.should_abort

  def test_abort_on_brake(self):
    self._arm()
    cs = make_carstate(brake_pressed=True)
    self._update(cs, CLOSING_LEAD)
    assert self.ap.should_abort
    assert not self.ap.execute_allowed
    assert self.ap.cooldown_timer > 0

  def test_abort_on_opposing_steering(self):
    self._arm()  # candidate_direction is left
    cs = make_carstate(steering_pressed=True, steering_torque=-1.0)  # torque opposing left
    self._update(cs, CLOSING_LEAD)
    assert self.ap.should_abort

  def test_no_abort_on_agreeing_steering(self):
    self._arm()
    cs = make_carstate(steering_pressed=True, steering_torque=1.0)  # torque agreeing with left
    self._update(cs, CLOSING_LEAD)
    assert not self.ap.should_abort

  def test_abort_on_blindspot_occupied(self):
    self._arm()  # candidate_direction is left
    cs = make_carstate(left_blindspot=True)
    self._update(cs, CLOSING_LEAD)
    assert self.ap.should_abort
    assert not self.ap.blindspot_clear

  def test_abort_on_multi_lane_invalidated_mid_countdown(self):
    self._arm()
    self._update(self._clear_side_carstate(), CLOSING_LEAD, multi_lane_valid=False, multi_lane_same_direction=True)
    assert self.ap.should_abort
    assert self.ap.cooldown_timer > 0

  def test_abort_just_before_gaze_confirm_still_aborts(self):
    self._arm()
    num_updates_below = max(int(GAZE_CONFIRM_DWELL_S / DT_MDL) - 2, 1)
    for _ in range(num_updates_below):
      self._update(self._clear_side_carstate(), CLOSING_LEAD, gaze_direction=LaneChangeDirection.left)
      assert not self.ap.execute_allowed

    # Brake right before the glance would otherwise complete -- must still abort.
    cs = make_carstate(brake_pressed=True)
    self._update(cs, CLOSING_LEAD, gaze_direction=LaneChangeDirection.left)
    assert self.ap.should_abort
    assert not self.ap.execute_allowed

  def test_disabled_forces_idle_from_any_phase(self):
    self._arm()
    self._update(self._clear_side_carstate(), CLOSING_LEAD)
    assert self.ap.was_armed_by_us

    self.ap.enabled = False
    self._update(self._clear_side_carstate(), CLOSING_LEAD)
    assert not self.ap.was_armed_by_us
    assert not self.ap.execute_allowed
    assert not self.ap.trigger_ready
    assert not self.ap.return_pending

  def test_cooldown_prevents_immediate_retrigger(self):
    self._arm()
    cs = make_carstate(brake_pressed=True)
    self._update(cs, CLOSING_LEAD)
    assert self.ap.should_abort
    assert self.ap.cooldown_timer == COOLDOWN_S

    # DesireHelper would drop back to off after should_abort; simulate that.
    self.DH.lane_change_state = LaneChangeState.off
    self.DH.lane_change_direction = LaneChangeDirection.none

    self._update(self._clear_side_carstate(), CLOSING_LEAD)
    assert not self.ap.trigger_ready

    # After the cooldown elapses (plus the sustained-closing dwell on top of
    # that), it should be eligible again.
    num_updates = int((COOLDOWN_S + TTC_DWELL_S) / DT_MDL) + 2
    for _ in range(num_updates):
      self._update(self._clear_side_carstate(), CLOSING_LEAD)
    assert self.ap.trigger_ready

  def test_execute_allowed_false_once_maneuver_underway(self):
    self._arm()
    self._confirm(LaneChangeDirection.left)
    assert self.ap.execute_allowed

    self.DH.lane_change_state = LaneChangeState.laneChangeStarting
    self._update(self._clear_side_carstate(), CLOSING_LEAD)
    assert not self.ap.execute_allowed

  def test_cooldown_starts_and_ownership_clears_on_finish(self):
    self._arm()
    self.DH.lane_change_state = LaneChangeState.laneChangeFinishing
    self._update(self._clear_side_carstate(), CLOSING_LEAD)
    assert not self.ap.was_armed_by_us
    assert self.ap.cooldown_timer > 0

  def test_ttc_ignores_lead_below_min_closing_speed(self):
    slow_lead = make_lead(dRel=5.0, vRel=-(MIN_CLOSING_SPEED - 0.01), status=True)
    self._update(self._clear_side_carstate(), slow_lead)
    assert self.ap.ttc == float('inf')
    assert not self.ap.trigger_ready

  def test_driver_initiated_lane_change_is_untouched(self):
    """A blinker-driven change (was_armed_by_us stays False) must never be gated
    or aborted by Auto Pass logic."""
    self.DH.lane_change_state = LaneChangeState.preLaneChange
    cs = make_carstate(brake_pressed=True)  # would abort an auto-pass-owned change
    self._update(cs, CLOSING_LEAD)
    assert not self.ap.was_armed_by_us
    assert not self.ap.execute_allowed


class TestAutoPassReturnToLane:
  """Covers the post-pass return-to-original-lane leg: right blind spot clear
  for RETURN_MIN_CLEAR_S arms a candidate, then a sustained right-directed
  glance confirms it -- mirrors the outbound left-pass path exactly, just
  flipped, per BMW's bidirectional gaze-confirm model."""

  def setup_method(self):
    self.DH = DesireHelper()
    self.ap = self.DH.auto_pass
    self.ap.enabled = True
    self.ap.shadow_mode = False
    self.ap.cooldown_timer = 0.0
    self.DH.lane_change_state = LaneChangeState.off
    self.DH.lane_change_direction = LaneChangeDirection.none

  def _update(self, cs, lead=CLOSING_LEAD, multi_lane_valid=True, multi_lane_same_direction=True,
              below_lane_change_speed=False, gaze_direction=None, uncertainty=CONFIRMED_UNCERTAINTY,
              calibrated=True):
    self.ap.update(cs, lead, multi_lane_valid, multi_lane_same_direction, below_lane_change_speed,
                   _GAZE_YAW[gaze_direction], uncertainty, calibrated)

  def _clear_side_carstate(self, **kwargs):
    kwargs.setdefault('left_blindspot', False)
    kwargs.setdefault('right_blindspot', False)
    return make_carstate(**kwargs)

  def _complete_left_pass(self):
    """Fast-forward straight to 'just finished an auto-pass-left', the way a
    full drive through off -> preLaneChange -> starting -> finishing would."""
    num_updates = int(TTC_DWELL_S / DT_MDL) + 2
    for _ in range(num_updates):
      self._update(self._clear_side_carstate(), CLOSING_LEAD)
    assert self.ap.trigger_ready
    self.DH.lane_change_direction = LaneChangeDirection.left
    self.DH.lane_change_state = LaneChangeState.preLaneChange
    self.ap.mark_triggered()

    num_updates = int(GAZE_CONFIRM_DWELL_S / DT_MDL) + 2
    for _ in range(num_updates):
      self._update(self._clear_side_carstate(), CLOSING_LEAD, gaze_direction=LaneChangeDirection.left)
    assert self.ap.execute_allowed

    self.DH.lane_change_state = LaneChangeState.laneChangeFinishing
    self._update(self._clear_side_carstate(), CLOSING_LEAD)
    assert self.ap.return_pending
    assert not self.ap.was_armed_by_us
    self.DH.lane_change_state = LaneChangeState.off
    self.DH.lane_change_direction = LaneChangeDirection.none

  def _wait_out_cooldown(self):
    # +TTC_DWELL_S margin so callers that immediately check the outbound
    # trigger afterward (once return_pending is no longer set) have enough
    # trailing settled time to also clear the sustained-closing dwell.
    num_updates = int((COOLDOWN_S + TTC_DWELL_S) / DT_MDL) + 2
    for _ in range(num_updates):
      self._update(self._clear_side_carstate(), CLOSING_LEAD)

  def test_return_not_ready_while_right_blindspot_occupied(self):
    self._complete_left_pass()
    self._wait_out_cooldown()
    cs = self._clear_side_carstate(right_blindspot=True)
    num_updates = int(RETURN_MIN_CLEAR_S / DT_MDL) + 2
    for _ in range(num_updates):
      self._update(cs, CLOSING_LEAD)
    assert not self.ap.return_trigger_ready

  def test_return_trigger_ready_after_clear_dwell(self):
    self._complete_left_pass()
    self._wait_out_cooldown()
    num_updates = int(RETURN_MIN_CLEAR_S / DT_MDL) + 2
    for _ in range(num_updates):
      self._update(self._clear_side_carstate(), CLOSING_LEAD)
    assert self.ap.return_trigger_ready
    assert self.ap.candidate_direction == LaneChangeDirection.right

  def test_no_fresh_outbound_trigger_while_return_pending(self):
    """Must not stack a second left pass while still owing a return from the first."""
    self._complete_left_pass()
    self._update(self._clear_side_carstate(), CLOSING_LEAD)
    assert not self.ap.trigger_ready

  def _arm_return(self):
    self._complete_left_pass()
    self._wait_out_cooldown()
    num_updates = int(RETURN_MIN_CLEAR_S / DT_MDL) + 2
    for _ in range(num_updates):
      self._update(self._clear_side_carstate(), CLOSING_LEAD)
    assert self.ap.return_trigger_ready
    self.DH.lane_change_direction = LaneChangeDirection.right
    self.DH.lane_change_state = LaneChangeState.preLaneChange
    self.ap.mark_triggered()

  def test_left_gaze_does_not_confirm_return(self):
    """should_abort is recomputed fresh from the hard conditions at the top of
    every preLaneChange frame, so we must check right as it fires -- the real
    DesireHelper exits preLaneChange the same cycle, it never loops past it."""
    self._arm_return()
    max_updates = int(CONFIRM_WINDOW_S / DT_MDL) + 2
    aborted = False
    for _ in range(max_updates):
      self._update(self._clear_side_carstate(), CLOSING_LEAD, gaze_direction=LaneChangeDirection.left)
      if self.ap.should_abort:
        aborted = True
        break
    assert not self.ap.execute_allowed
    assert aborted

  def test_right_gaze_confirms_return(self):
    self._arm_return()
    num_updates = int(GAZE_CONFIRM_DWELL_S / DT_MDL) + 2
    for _ in range(num_updates):
      self._update(self._clear_side_carstate(), CLOSING_LEAD, gaze_direction=LaneChangeDirection.right)
    assert self.ap.gaze_confirmed
    assert self.ap.execute_allowed

  def test_return_pending_clears_once_return_completes(self):
    self._arm_return()
    num_updates = int(GAZE_CONFIRM_DWELL_S / DT_MDL) + 2
    for _ in range(num_updates):
      self._update(self._clear_side_carstate(), CLOSING_LEAD, gaze_direction=LaneChangeDirection.right)
    assert self.ap.execute_allowed

    self.DH.lane_change_state = LaneChangeState.laneChangeFinishing
    self._update(self._clear_side_carstate(), CLOSING_LEAD)
    assert not self.ap.return_pending

    # No return owed anymore -- a fresh left-pass trigger must be reachable again.
    self.DH.lane_change_state = LaneChangeState.off
    self.DH.lane_change_direction = LaneChangeDirection.none
    self._wait_out_cooldown()
    self._update(self._clear_side_carstate(), CLOSING_LEAD)
    assert self.ap.trigger_ready
    assert self.ap.candidate_direction == LaneChangeDirection.left

  def test_aborted_return_retries_after_cooldown_instead_of_abandoning(self):
    """A return attempt that aborts (e.g. blind spot re-occupied) must still be
    owed -- it should retry after cooldown, not silently give up and let a
    second outbound pass trigger instead."""
    self._arm_return()
    cs = self._clear_side_carstate(right_blindspot=True)  # re-occupied mid-countdown
    self._update(cs, CLOSING_LEAD, gaze_direction=LaneChangeDirection.right)
    assert self.ap.should_abort
    assert self.ap.return_pending  # still owed, not abandoned

    self.DH.lane_change_state = LaneChangeState.off
    self.DH.lane_change_direction = LaneChangeDirection.none
    self._wait_out_cooldown()

    num_updates = int(RETURN_MIN_CLEAR_S / DT_MDL) + 2
    for _ in range(num_updates):
      self._update(self._clear_side_carstate(), CLOSING_LEAD)
    assert self.ap.return_trigger_ready
    assert self.ap.candidate_direction == LaneChangeDirection.right
