"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from cereal import car, custom, log

from openpilot.selfdrive.controls.lib.desire_helper import DesireHelper, LaneChangeState, LaneChangeDirection


def make_carstate(v_ego=25., left_blinker=False, right_blinker=False, left_blindspot=False, right_blindspot=False,
                   brake_pressed=False, steering_pressed=False, steering_torque=0.0):
  cs = car.CarState.new_message()
  cs.vEgo = v_ego
  cs.leftBlinker = left_blinker
  cs.rightBlinker = right_blinker
  cs.leftBlindspot = left_blindspot
  cs.rightBlindspot = right_blindspot
  cs.brakePressed = brake_pressed
  cs.steeringPressed = steering_pressed
  cs.steeringTorque = steering_torque
  return cs


def make_lead(dRel=10.0, vRel=-5.0, status=True):
  lead = log.RadarState.LeadData.new_message()
  lead.dRel = dRel
  lead.vRel = vRel
  lead.status = status
  return lead


def make_live_map_data(multi_lane_valid=True, multi_lane_same_direction=True):
  data = custom.LiveMapDataSP.new_message()
  data.multiLaneValid = multi_lane_valid
  data.multiLaneSameDirection = multi_lane_same_direction
  return data


class TestDesireHelperBlinkerRegression:
  """Auto Pass (auto_pass.py) is wired into DesireHelper.update() via ORs added
  onto the existing blinker/torque conditions. These tests pin down that a
  driver-initiated (blinker + torque nudge) lane change behaves exactly as it
  did before that change, with Auto Pass disabled (the default)."""

  def setup_method(self):
    self.DH = DesireHelper()
    assert not self.DH.auto_pass.enabled, "test assumes AutoPassEnabled defaults to False"

  def test_blinker_alone_does_not_start_lane_change(self):
    cs = make_carstate(left_blinker=True)
    self.DH.update(cs, lateral_active=True, lane_change_prob=0.0)
    assert self.DH.lane_change_state == LaneChangeState.preLaneChange
    assert self.DH.lane_change_direction == LaneChangeDirection.left

    # Without a nudge, it should stay in preLaneChange, not advance.
    for _ in range(5):
      self.DH.update(cs, lateral_active=True, lane_change_prob=0.0)
    assert self.DH.lane_change_state == LaneChangeState.preLaneChange

  def test_blinker_plus_nudge_starts_lane_change(self):
    cs = make_carstate(left_blinker=True)
    self.DH.update(cs, lateral_active=True, lane_change_prob=0.0)
    assert self.DH.lane_change_state == LaneChangeState.preLaneChange

    cs = make_carstate(left_blinker=True, steering_pressed=True, steering_torque=1.0)
    self.DH.update(cs, lateral_active=True, lane_change_prob=0.0)
    assert self.DH.lane_change_state == LaneChangeState.laneChangeStarting

  def test_blindspot_blocks_lane_change_start(self):
    cs = make_carstate(left_blinker=True)
    self.DH.update(cs, lateral_active=True, lane_change_prob=0.0)

    cs = make_carstate(left_blinker=True, left_blindspot=True, steering_pressed=True, steering_torque=1.0)
    self.DH.update(cs, lateral_active=True, lane_change_prob=0.0)
    assert self.DH.lane_change_state == LaneChangeState.preLaneChange

  def test_releasing_blinker_cancels_pre_lane_change(self):
    cs = make_carstate(left_blinker=True)
    self.DH.update(cs, lateral_active=True, lane_change_prob=0.0)
    assert self.DH.lane_change_state == LaneChangeState.preLaneChange

    cs = make_carstate(left_blinker=False)
    self.DH.update(cs, lateral_active=True, lane_change_prob=0.0)
    assert self.DH.lane_change_state == LaneChangeState.off

  def test_below_lane_change_speed_blocks_arming(self):
    cs = make_carstate(v_ego=2., left_blinker=True)  # well under 20mph
    self.DH.update(cs, lateral_active=True, lane_change_prob=0.0)
    assert self.DH.lane_change_state == LaneChangeState.off

  def test_auto_pass_never_arms_when_disabled_even_under_favorable_conditions(self):
    """No blinker at all, closing fast on a lead, on a road that would otherwise
    satisfy every Auto Pass gate: AutoPassEnabled=False must still keep it off."""
    cs = make_carstate()  # no blinker, no torque
    lead = make_lead()
    live_map_data = make_live_map_data()
    for _ in range(50):
      self.DH.update(cs, lateral_active=True, lane_change_prob=0.0, lead_one=lead, live_map_data=live_map_data)
    assert self.DH.lane_change_state == LaneChangeState.off
    assert self.DH.lane_change_direction == LaneChangeDirection.none
