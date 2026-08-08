#!/usr/bin/env python3
"""
Walk one or more recorded segment directories and print a timeline of
AutoPassStateSP phase transitions -- for reviewing shadow-mode drives.

Usage (run on the device, from /data/openpilot):
  /usr/local/venv/bin/python3 review_autopass_log.py /data/media/0/realdata/<route>--0 /data/media/0/realdata/<route>--1 ...

Or all segments of a route at once:
  /usr/local/venv/bin/python3 review_autopass_log.py /data/media/0/realdata/<route>--*
"""
import sys
import zstandard
from cereal import log as capnp_log
from cereal import custom

Phase = custom.AutoPassStateSP.Phase
Direction = custom.AutoPassStateSP.Direction

PHASE_NAMES = {v: k for k, v in Phase.schema.enumerants.items()}
DIRECTION_NAMES = {v: k for k, v in Direction.schema.enumerants.items()}


def iter_segment_messages(segment_dir):
  rlog_path = f"{segment_dir}/rlog.zst"
  try:
    with open(rlog_path, 'rb') as f:
      data = zstandard.ZstdDecompressor().decompress(f.read(), max_output_size=500 * 1024 * 1024)
  except FileNotFoundError:
    print(f"  (skipping {segment_dir}: no rlog.zst)")
    return
  for m in capnp_log.Event.read_multiple_bytes(data):
    yield m


def main(segment_dirs):
  last_phase = None
  last_direction = None
  phase_entered_at = None
  event_count = 0

  for segment_dir in segment_dirs:
    for m in iter_segment_messages(segment_dir):
      if m.which() != 'autoPassStateSP':
        continue
      ap = m.autoPassStateSP
      t = m.logMonoTime / 1e9

      if ap.phase != last_phase:
        if last_phase is not None and phase_entered_at is not None:
          dwell = t - phase_entered_at
          print(f"[{t:>12.2f}s] {PHASE_NAMES.get(last_phase, last_phase):<12} -> "
                f"{PHASE_NAMES.get(ap.phase, ap.phase):<12} "
                f"(held {dwell:5.1f}s, dir={DIRECTION_NAMES.get(last_direction, last_direction)}, "
                f"ttc={ap.ttc:.1f}s, blindspotClear={ap.blindspotClear})")
        else:
          print(f"[{t:>12.2f}s] start -> {PHASE_NAMES.get(ap.phase, ap.phase):<12} "
                f"(dir={DIRECTION_NAMES.get(ap.candidateDirection, ap.candidateDirection)})")
        last_phase = ap.phase
        phase_entered_at = t
        event_count += 1
      last_direction = ap.candidateDirection

  print(f"\n{event_count} phase transitions found across {len(segment_dirs)} segment(s).")
  if event_count == 0:
    print("No autoPassStateSP activity -- check AutoPassEnabled was actually on during this drive.")


if __name__ == '__main__':
  if len(sys.argv) < 2:
    print(__doc__)
    sys.exit(1)
  main(sys.argv[1:])
