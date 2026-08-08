#!/usr/bin/env python3
"""
Walk one or more recorded segment directories and print a timeline of
AutoPassStateSP phase transitions, plus a stability summary -- for
reviewing shadow-mode drives.

Usage (run on the device, from /data/openpilot):
  /usr/local/venv/bin/python3 review_autopass_log.py /data/media/0/realdata/<route>--0 /data/media/0/realdata/<route>--1 ...

Or all segments of a route at once:
  /usr/local/venv/bin/python3 review_autopass_log.py /data/media/0/realdata/<route>--*
"""
import sys
import zstandard
from cereal import log as capnp_log
from cereal import custom
from openpilot.sunnypilot.selfdrive.controls.lib.auto_pass import COOLDOWN_S

Phase = custom.AutoPassStateSP.Phase
Direction = custom.AutoPassStateSP.Direction

PHASE_NAMES = {v: k for k, v in Phase.schema.enumerants.items()}
DIRECTION_NAMES = {v: k for k, v in Direction.schema.enumerants.items()}

FLICKER_THRESHOLD_S = 1.0  # idle/monitoring dwell shorter than this is flagged as chatter


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


def collect_transitions(segment_dirs):
  """Returns a list of dicts: t, from_phase, to_phase, dwell, direction, ttc, blindspot_clear."""
  transitions = []
  last_phase = None
  last_direction = None
  phase_entered_at = None

  for segment_dir in segment_dirs:
    for m in iter_segment_messages(segment_dir):
      if m.which() != 'autoPassStateSP':
        continue
      ap = m.autoPassStateSP
      t = m.logMonoTime / 1e9

      if ap.phase != last_phase:
        if last_phase is not None and phase_entered_at is not None:
          transitions.append({
            't': t, 'from_phase': last_phase, 'to_phase': ap.phase,
            'dwell': t - phase_entered_at, 'direction': last_direction,
            'ttc': ap.ttc, 'blindspot_clear': ap.blindspotClear,
          })
        last_phase = ap.phase
        phase_entered_at = t
      last_direction = ap.candidateDirection

  return transitions


def print_timeline(transitions):
  for tr in transitions:
    print(f"[{tr['t']:>12.2f}s] {PHASE_NAMES.get(tr['from_phase'], tr['from_phase']):<12} -> "
          f"{PHASE_NAMES.get(tr['to_phase'], tr['to_phase']):<12} "
          f"(held {tr['dwell']:5.1f}s, dir={DIRECTION_NAMES.get(tr['direction'], tr['direction'])}, "
          f"ttc={tr['ttc']:.1f}s, blindspotClear={tr['blindspot_clear']})")


def print_stability_summary(transitions):
  print("\n" + "=" * 70)
  print("STABILITY SUMMARY")
  print("=" * 70)

  if not transitions:
    print("No autoPassStateSP activity -- check AutoPassEnabled was actually on during this drive.")
    return

  # A "countdown attempt" is any transition INTO countdown. Its outcome is
  # whatever the NEXT transition goes to: executing = confirmed, anything
  # else = aborted (the phase property never emits a distinct `aborted`
  # value, so this is inferred from what countdown exits to).
  countdown_entries = [(i, tr) for i, tr in enumerate(transitions) if tr['to_phase'] == Phase.countdown]

  for direction, label in ((Direction.left, "LEFT (outbound pass)"), (Direction.right, "RIGHT (return leg)")):
    attempts = [(i, tr) for i, tr in countdown_entries if tr['direction'] == direction]
    if not attempts:
      print(f"\n{label}: no countdown attempts")
      continue

    confirmed = []
    aborted = []
    for i, tr in attempts:
      outcome = transitions[i + 1] if i + 1 < len(transitions) else None
      if outcome and outcome['from_phase'] == Phase.countdown and outcome['to_phase'] == Phase.executing:
        confirmed.append(outcome)
      else:
        aborted.append(tr)

    n = len(attempts)
    confirm_rate = 100 * len(confirmed) / n if n else 0
    print(f"\n{label}: {n} countdown attempt(s), {len(confirmed)} confirmed, {len(aborted)} aborted "
          f"({confirm_rate:.0f}% confirm rate)")
    if confirmed:
      avg_confirm_dwell = sum(o['dwell'] for o in confirmed) / len(confirmed)
      print(f"  avg time-to-confirm: {avg_confirm_dwell:.1f}s")
    if aborted:
      abort_dwells = []
      for i, tr in attempts:
        outcome = transitions[i + 1] if i + 1 < len(transitions) else None
        if outcome and not (outcome['from_phase'] == Phase.countdown and outcome['to_phase'] == Phase.executing):
          abort_dwells.append(outcome['dwell'])
      if abort_dwells:
        print(f"  avg time-to-abort: {sum(abort_dwells) / len(abort_dwells):.1f}s")

    # Cooldown gap check: time between consecutive countdown entries.
    if len(attempts) > 1:
      gaps = [attempts[j][1]['t'] - attempts[j - 1][1]['t'] for j in range(1, len(attempts))]
      short_gaps = [g for g in gaps if g < COOLDOWN_S]
      if short_gaps:
        print(f"  ⚠ {len(short_gaps)} retrigger(s) closer together than COOLDOWN_S ({COOLDOWN_S}s) "
              f"-- shortest gap {min(short_gaps):.1f}s")
      else:
        print(f"  cooldown respected: all {len(gaps)} retrigger gap(s) >= {COOLDOWN_S}s "
              f"(min {min(gaps):.1f}s)")

  # Flicker: idle/monitoring dwells shorter than FLICKER_THRESHOLD_S.
  flicker = [tr for tr in transitions
             if tr['from_phase'] in (Phase.idle, Phase.monitoring) and tr['dwell'] < FLICKER_THRESHOLD_S]
  print(f"\nFlicker check: {len(flicker)} idle/monitoring transition(s) held under {FLICKER_THRESHOLD_S}s")
  if flicker:
    print("  ⚠ possible chatter -- trigger condition flapping without committing to a countdown")

  print(f"\nTotal phase transitions: {len(transitions)}")


def main(segment_dirs):
  transitions = collect_transitions(segment_dirs)
  print_timeline(transitions)
  print_stability_summary(transitions)


if __name__ == '__main__':
  if len(sys.argv) < 2:
    print(__doc__)
    sys.exit(1)
  main(sys.argv[1:])
