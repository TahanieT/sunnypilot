# Phone-in-frame distraction gate (private patch)

## What this changes

`selfdrive/monitoring/policy.py`, `_get_distracted_types()`: a phone merely
visible in the driver-facing camera no longer trips a distraction alert on
its own. It now requires gaze *also* off-road (the existing `pose` check) at
the same time. Looking directly at the phone still alerts; holding it up
with eyes on the road does not; eyes-off-road with no phone is completely
unaffected.

```python
phone_visible = bool(self.phone_prob > self.settings._PHONE_THRESH)
# Phone-in-frame alone must not trip distraction; also require gaze off-road (pose).
# If pose confidence is low (self.pose.low_std False), fall back to phone-alone --
# conservative per spec: never let uncertain gaze data suppress a real phone alert.
self.distracted_types['phone'] = phone_visible and (self.distracted_types['pose'] or not self.pose.low_std)
```

Nothing else changed: pose/eye thresholds, timing, lockout, steering, and
longitudinal are all untouched. This is a private fork for personal testing
only — never published, no PR to sunnypilot.

## Repo layout gotcha

`master` and `release-tizi` are structurally different generations of this
file (different lockout scheme, `standstill` vs `lowspeed` naming, different
default timeouts, nested `openpilot/` dir on `master` vs flat layout on
`release-tizi`). The device runs `release-tizi`. Always check
`git rev-parse HEAD` / `git branch --show-current` on the device before
assuming which branch/layout to edit. Local worktrees:

- `c:\dev\ADS` — `master`, reference only
- `c:\dev\ADS-release-tizi` — `release-tizi`, matches the device, edit here

## Fork / remotes

- Fork: `github.com/TahanieT/sunnypilot` (remote `origin` in both worktrees
  and on the device)
- Upstream: `github.com/sunnypilot/sunnypilot` (remote `upstream` in the
  worktrees, `origin` — confusingly — was the device's original remote name
  before we added `myfork`/rewired it)
- On the device (`/data/openpilot`): `origin` = upstream sunnypilot,
  `myfork` = this fork. The branch `release-tizi` tracks `origin`
  (upstream), not `myfork` — see updater gotcha below.

## Deploying a change to the device

1. Edit in `c:\dev\ADS-release-tizi`, commit, `git push origin release-tizi`.
2. SSH to the device (`ssh comma@<device-ip>`, key must be an
   **account-level Authentication Key** at github.com/settings/keys, not a
   per-repo deploy key — deploy keys don't show up in
   `github.com/<user>.keys`, which is what the device's SSH trust reads).
3. On device: `cd /data/openpilot && git fetch myfork release-tizi &&
   git merge --ff-only myfork/release-tizi`
4. Reboot to relaunch daemons with the new code (`sudo reboot`).

## Critical gotcha: `DisableUpdates` doesn't stop an already-running updater

`system/updated/updated.py` only checks the `DisableUpdates` param **once,
at process startup** (line ~405). It does not poll it. If you set
`DisableUpdates=1` via a raw param write while `updated` is already running,
the daemon keeps executing its current cycle: `git fetch origin` →
`git checkout --force ... FETCH_HEAD` → `git reset --hard` →
**`git clean -xdff`** → resets the branch straight back to upstream
`release-tizi` and wipes any untracked files (this is what deleted our
`watch_dm.py` test script the first time). This exactly reverted our patch
commit despite the param already being set to `1`.

**Fix:** after setting `DisableUpdates=1`, you must reboot (or otherwise
restart the `updated` process) for it to actually take effect. Verify with
`pgrep -af updated` — it should find nothing after reboot, since the daemon
reads the flag at startup and `exit(0)`s immediately if it's set.

Current device state: `DisableUpdates=1`, confirmed `updated` not running,
HEAD at the patch commit.

## Verifying behavior — `watch_dm.py`

A small subscriber script (deployed at `/data/openpilot/watch_dm.py` on the
device) prints live `driverMonitoringState` fields:
`faceDet / pose / eye / phone / isDistr / alertLvl / policy`. Run with:

```
cd /data/openpilot && /usr/local/venv/bin/python3 -u watch_dm.py
```

(Needs the venv python specifically — plain `python3` lacks `zmq`/`cereal`;
also must run from `/data/openpilot` as cwd, since the script's own
directory needs to be on `sys.path` for the `cereal`/`openpilot` imports to
resolve.)

Doesn't require driving — `_get_distracted_types()` and `isDistracted`
update live regardless of engagement; only the alert-escalation timer needs
the car actually moving/engaged.

## Bench test status (as of 2026-07-26)

- ✅ Phone visible **and** looking at it (`pose=True, phone=True`) →
  `isDistr=True`. Confirmed live on device.
- ✅ No instance observed of `phone=True` alone flipping `isDistr` to
  `True` — every `phone=True` row also had `pose=True` in all runs so far.
- ⏳ **Not yet confirmed**: phone visible **with eyes kept on the road**
  (`phone=True, pose=False` → should give `isDistr=False`). Two attempts at
  night both showed `phone` never triggering at all (likely a lighting
  issue — the phone detector is a vision classifier and struggled in the
  dark). Retry in daylight, holding the phone up near face level so it's
  clearly in the driver-facing camera's view, while keeping gaze forward.
