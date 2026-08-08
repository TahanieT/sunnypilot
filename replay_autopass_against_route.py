#!/usr/bin/env python3
"""
Feed a real recorded drive's messages through a fresh DesireHelper/
AutoPassController, exactly mirroring modeld.py's own update call, to see
what AutoPass would have decided -- without needing the vision/camera
pipeline (no model inference, just the downstream message processing
where AutoPass logic actually lives).

Also useful for debugging: if DH.update() throws here against real data
that the live device never surfaced an exception for, that's a strong lead.

Usage:
  /usr/local/venv/bin/python3 replay_autopass_against_route.py /data/media/0/realdata/<route>--0 ...
"""
import sys
import zstandard
from cereal import log as capnp_log, custom
from openpilot.selfdrive.controls.lib.desire_helper import DesireHelper, LANE_CHANGE_SPEED_MIN
from openpilot.sunnypilot.selfdrive.controls.lib.auto_pass import TTC_TRIGGER_S

Phase = custom.AutoPassStateSP.Phase
PHASE_NAMES = {v: k for k, v in Phase.schema.enumerants.items()}

WATCHED = {'carState', 'carControl', 'radarState', 'liveMapDataSP', 'driverMonitoringState', 'modelV2'}


def iter_segment_messages(segment_dir):
  rlog_path = f"{segment_dir}/rlog.zst"
  try:
    with open(rlog_path, 'rb') as f:
      data = zstandard.ZstdDecompressor().decompress(f.read(), max_output_size=500 * 1024 * 1024)
  except FileNotFoundError:
    print(f"  (skipping {segment_dir}: no rlog.zst)")
    return
  yield from capnp_log.Event.read_multiple_bytes(data)


def main(segment_dirs):
  DH = DesireHelper()
  latest = dict.fromkeys(WATCHED)

  last_phase = None
  update_count = 0
  exceptions = 0
  t0 = None

  min_ttc = float('inf')
  gate_stats = {
    'below_speed': 0, 'ttc_under_threshold': 0, 'multi_lane_valid': 0,
    'multi_lane_same_direction_given_valid': 0, 'left_blindspot_clear': 0,
  }

  for segment_dir in segment_dirs:
    for m in iter_segment_messages(segment_dir):
      w = m.which()
      if w in WATCHED:
        latest[w] = getattr(m, w)

      if w != 'carState':
        continue  # drive the update cadence off carState arrivals, ~matches modeld's own rate
      if latest['carState'] is None or latest['carControl'] is None:
        continue

      t = m.logMonoTime / 1e9
      if t0 is None:
        t0 = t

      lead_one = latest['radarState'].leadOne if latest['radarState'] is not None else None
      lmd = latest['liveMapDataSP']

      if latest['modelV2'] is not None:
        desire_state = latest['modelV2'].meta.desireState
        lane_change_prob = desire_state[capnp_log.Desire.laneChangeLeft] + desire_state[capnp_log.Desire.laneChangeRight]
      else:
        lane_change_prob = 0.0

      if latest['driverMonitoringState'] is not None:
        pose = latest['driverMonitoringState'].visionPolicyState.pose
        yaw, uncertainty, calibrated = pose.yaw, pose.uncertainty, pose.calibrated
      else:
        yaw, uncertainty, calibrated = 0.0, float('inf'), False

      try:
        DH.update(latest['carState'], latest['carControl'].latActive, lane_change_prob,
                  lead_one, lmd, yaw, uncertainty, calibrated)
        update_count += 1
      except Exception as e:
        exceptions += 1
        print(f"[{t - t0:>8.2f}s] EXCEPTION in DH.update(): {type(e).__name__}: {e}")
        continue

      ap = DH.auto_pass
      below_speed = latest['carState'].vEgo < LANE_CHANGE_SPEED_MIN
      if below_speed:
        gate_stats['below_speed'] += 1
      if ap.ttc < min_ttc:
        min_ttc = ap.ttc
      if ap.ttc < TTC_TRIGGER_S:
        gate_stats['ttc_under_threshold'] += 1
      if ap.multi_lane_valid:
        gate_stats['multi_lane_valid'] += 1
        if ap.multi_lane_same_direction:
          gate_stats['multi_lane_same_direction_given_valid'] += 1
      if not latest['carState'].leftBlindspot:
        gate_stats['left_blindspot_clear'] += 1

      phase = DH.auto_pass.phase
      if phase != last_phase:
        print(f"[{t - t0:>8.2f}s] phase -> {PHASE_NAMES.get(phase, phase):<12} "
              f"(enabled={DH.auto_pass.enabled}, shadow={DH.auto_pass.shadow_mode}, "
              f"ttc={DH.auto_pass.ttc:.1f}s, dir={DH.auto_pass.candidate_direction})")
        last_phase = phase

  print(f"\n{update_count} DH.update() call(s) completed, {exceptions} exception(s) hit.")
  if update_count == 0:
    print("No carState messages found -- wrong segment path?")
    return

  n = update_count
  print("\n" + "=" * 70)
  print("TRIGGER GATE BREAKDOWN (which condition(s) were actually blocking it)")
  print("=" * 70)
  print(f"min TTC reached: {min_ttc:.1f}s  (needs < {TTC_TRIGGER_S}s to pass the gate)")
  print(f"below LANE_CHANGE_SPEED_MIN ({LANE_CHANGE_SPEED_MIN:.1f} m/s): "
        f"{100 * gate_stats['below_speed'] / n:.1f}% of samples")
  print(f"ttc < TTC_TRIGGER_S ({TTC_TRIGGER_S}s): {100 * gate_stats['ttc_under_threshold'] / n:.1f}% of samples")
  print(f"multi_lane_valid=True: {100 * gate_stats['multi_lane_valid'] / n:.1f}% of samples")
  if gate_stats['multi_lane_valid'] > 0:
    print(f"  of those, multi_lane_same_direction=True: "
          f"{100 * gate_stats['multi_lane_same_direction_given_valid'] / gate_stats['multi_lane_valid']:.1f}%")
  print(f"left blindspot clear: {100 * gate_stats['left_blindspot_clear'] / n:.1f}% of samples")


if __name__ == '__main__':
  if len(sys.argv) < 2:
    print(__doc__)
    sys.exit(1)
  main(sys.argv[1:])
