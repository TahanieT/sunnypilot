# Auto Pass (private feature)

Gaze-confirmed autonomous lane change to pass a slower lead, plus a mirrored
return to the original lane. Private fork for personal testing only — never
published, no PR to sunnypilot.

Companion doc: `PHONE_GATE_NOTES.md` (unrelated driver-monitoring patch on the
same fork).

## What it does

`AutoPassController` (`sunnypilot/selfdrive/controls/lib/auto_pass.py`) arms a
lane change when it detects a lead worth passing, then **requires the driver to
confirm with a sustained glance toward the mirror on the maneuver's side**
before anything executes. This mirrors GM Super Cruise / BMW Active Lane Change
Assist rather than auto-executing off a bare timer.

- US-only convention: a fresh pass only ever goes **left**. No right-side
  passing.
- After a completed pass, it monitors for a safe gap and arms a mirrored
  **return** leg, confirmed by a **right** glance.
- A lapsed confirm window aborts the opportunity — it never auto-executes.

Two independent ways to arm the outbound trigger:

1. **TTC** — closing on the lead fast enough (`TTC_TRIGGER_S`), sustained for
   `TTC_DWELL_S`.
2. **Speed deficit** — lead's absolute speed sustained below our cruise set
   speed by `SPEED_DEFICIT_THRESHOLD_MS` for `SPEED_DEFICIT_DWELL_S`. This
   exists because once GM's ACC settles into steady-state following, `vRel`
   goes to ~0 and TTC to infinity, so a pure TTC trigger would never fire again
   even while stuck behind that car for miles.

## Two-layer safety gating

Both params gate the feature, independently:

| Param | Default | Effect |
| --- | --- | --- |
| `AutoPassEnabled` | off | Feature master switch |
| `AutoPassShadowMode` | on | Computes and logs everything, including what a real confirm *would* trigger, but hard-clamps `execute_allowed` false |

Shadow mode must be turned off separately in Settings → Auto Pass before
anything steers. **Do not turn shadow mode off until the verification steps
below have actually been completed on real drives.**

Note: `params_keys.h` default values are *not* applied by `Params.get()` on a
never-set key — that's a direct `util::read_file()` with no default fallback.
Defaults are written to disk by `system/manager/manager.py`'s startup loop, once
per boot. A standalone verification script therefore reads the raw unset state;
this resolves itself on any normal boot.

## Platform facts that shape the design

Vehicle: 2021 GMC Sierra 1500 AT4, `HARDWARE.get_device_type() == "tizi"`
(hence the `release-tizi` branch). GM's ACC is factory/stock and untouched
(`pcmCruise=True`).

- **No radar.** This is a `CAMERA_ACC_CAR` with `radarUnavailable=True`, so
  `radarState.leadOne` is always vision-derived from the comma 3X's own camera,
  never GM's. Every closing-rate signal is noisier than on a radar-equipped car.
  This is why the triggers have dwell timers at all — a single-frame estimation
  spike must never be able to arm a lane change.
- **Gaze sign is empirically verified, not guessed.** Raw
  `driverMonitoringState.visionPolicyState.pose.yaw` is **negative looking
  left, positive looking right, ~0 straight ahead** — checked live on this
  truck via a `watch_gaze.py` bench readout (2026-08-07) while physically
  glancing at each mirror. `GAZE_LEFT_SIGN = -1` is correct for that
  convention. If this constant is ever touched again, that measurement is the
  ground truth — do not re-derive it from code.
- **The turn signal is automatic.** `selfdrive/controls/controlsd.py` sets
  `CC.leftBlinker`/`rightBlinker` unconditionally whenever `lane_change_state
  != off`, with no check for whether the maneuver was driver- or
  AutoPass-initiated. Since `AutoPassController` arms the same
  `lane_change_state`/`lane_change_direction` fields a real blinker input
  would, the physical signal activates on both legs. The condition is `!= off`
  and `preLaneChange` already satisfies it, so the blinker comes on as soon as
  the countdown alert appears — before any steering, same sequencing a human
  would use.

## Tuning constants — all still placeholders

None of these have been tuned against real drives yet. Treat every value as a
starting guess pending shadow-mode logs:

`TTC_TRIGGER_S`, `TTC_DWELL_S`, `SPEED_DEFICIT_THRESHOLD_MS`,
`SPEED_DEFICIT_DWELL_S`, `GAZE_YAW_THRESHOLD_RAD`, `GAZE_MAX_UNCERTAINTY`,
`GAZE_CONFIRM_DWELL_S`, `CONFIRM_WINDOW_S`, `RETURN_MIN_CLEAR_S`.

Deliberately deferred: `ACCGapLevel` (GM's spacing-bar setting, 0–3) is readable
off CAN — confirmed via DBC, message `ASCMActiveCruiseControlStatus` (id 880),
already parsed for `ACCSpeedSetpoint` in `opendbc/car/gm/carstate.py`. It is
**not** yet extracted or wired through, rather than guess at GM's unpublished
level-to-distance formula. The plan was to plumb it through as telemetry only
(log it alongside `ttc`/`phase` for correlation) before ever using it to adjust
a threshold.

## The stock-vs-tinygrad model runner gotcha

**This cost a lot of debugging time. Read this before touching telemetry.**

`autoPassStateSP` was originally published only from
`sunnypilot/modeld_v2/modeld.py`. That process only runs under the **tinygrad**
model runner. This device runs the **stock** runner, so the message was never
published at all.

The chain: `ModelManager_ActiveBundle` unset → `get_active_bundle()` returns
`None` → `get_active_model_runner()` falls back to
`custom.ModelManagerSP.Runner.stock` (enum: `snpe@0, tinygrad@1, stock@2`) →
`is_stock_model` true → `system/manager/process_config.py` launches
`selfdrive.modeld.modeld` and never launches `sunnypilot/modeld_v2`.

Critically, `DesireHelper` — and therefore `AutoPassController` — is constructed
in **both** modelds. So the AutoPass logic was running the whole time under the
stock runner; it was simply **invisible**, with no telemetry and no UI.

**Safety consequence:** `selfdrive/selfdrived/selfdrived.py` gates the
gaze-confirm countdown alert on `sm['autoPassStateSP'].phase == countdown`. With
the message never published, `phase` sat at its default, so the driver-facing
confirm prompt could never appear. Had shadow mode been switched off in that
state, the maneuver logic would have been live with no countdown prompt.

**Fix:** the publish block was added to stock `selfdrive/modeld/modeld.py` as
well, mirroring modeld_v2's field set. This was chosen over switching the device
to the tinygrad runner, which would have swapped the actual driving model to fix
a telemetry gap.

**Diagnostic lesson:** the symptom was believed to be "`modelDataV2SP` *and*
`autoPassStateSP` are both never-alive," which made it look mysterious. A
message census over a real log showed `modelDataV2SP` publishing perfectly
(1021 msgs, exactly matching `modelV2`) and only `autoPassStateSP` at 0. When
diagnosing message plumbing, run the census first — don't trust a remembered
symptom:

```python
from openpilot.tools.lib.logreader import LogReader
from collections import Counter
c = Counter()
for m in LogReader("/data/media/0/realdata/<route>--<seg>/rlog.zst"):
    c[m.which()] += 1
print(c.most_common(20))
```

## The loggerd / stale `services.h` gotcha

**A second, independent reason AutoPass telemetry was missing — found after
the runner fix above was already deployed and driven.**

Fixing the runner problem made modeld publish `autoPassStateSP` correctly, and
live Python subscribers (including `selfdrived`, so the countdown alert) receive
it fine. But a full drive still recorded **zero** `autoPassStateSP` messages
across 14,176 model frames.

Cause: `cereal/services.h` is a **generated** C++ header (`services.py` has a
`build_header()` that prints it), and it is **tracked in git**. `services.py`
gained `autoPassStateSP` in the Auto Pass commit, but `services.h` was never
regenerated — it was last written by the upstream release commit. `loggerd` is
C++, `#include`s that header, and iterates the compiled-in `services` map
(`loggerd.cc:234`) to decide what to subscribe to. A service missing from the
header is never subscribed and never reaches any rlog.

This is the same class of bug as the stale `params_pyx.so`: a generated or
compiled artifact drifting out of sync with the Python source that defines it.

**Why it can't simply be rebuilt here:** this `release-tizi` branch ships
prebuilt — 579 tracked executables and **no root `SConstruct`**. Regenerating
the header alone changes nothing, because the shipped `loggerd` binary already
has the old map baked in. The `loggerd.cc` sources are present, so a hand build
is possible in principle, but loggerd also owns video encoding and all logging,
so a botched build means no logs at all.

**Workaround in place:** `ModelDataV2SP` gained an embedded `autoPass` field
holding a copy of the same state. `modelDataV2SP` *is* in the generated header
and logs reliably (confirmed at ~1200/segment, exactly matching `modelV2`), so
the copy is what actually survives into the rlog. Adding a capnp field is
backward-compatible by design, so the prebuilt C++ readers are unaffected and
nothing needed recompiling. Both modelds fill both destinations via
`fill_auto_pass_state()` in `auto_pass.py`, and `review_autopass_log.py` prefers
the standalone service when present and falls back to the embedded copy.

`cereal/services.h` has also been regenerated and committed, so a future build
from source is correct. **That does not fix the running binary** — the embedded
copy remains the working path until a rebuilt `loggerd` ships.

**Diagnostic tip:** if a message is published but absent from logs, check
whether it's in `cereal/services.h` (not just `services.py`) before suspecting
the publisher. A useful narrowing trick: this publish block sits *before*
`pm.send('modelV2', ...)`, so if it were throwing, `modelV2` would have stopped
too — `modelV2` being healthy proved the publisher was fine and moved suspicion
to the logger.

## Log review tooling

`review_autopass_log.py` (repo root) walks a route's `rlog.zst` segments and
prints:

1. A timeline of every `autoPassStateSP.phase` transition, with dwell time,
   direction, `ttc`, and `blindspotClear`.
2. A stability summary — per-direction (left pass / right return) countdown
   attempt count, confirm rate, average time-to-confirm and time-to-abort, a
   cooldown-gap check against the real `COOLDOWN_S` constant (imported directly
   from `auto_pass.py`, not hardcoded), and a flicker check for short-dwell
   idle/monitoring bouncing.

Usage, from `/data/openpilot` on the device:

```
/usr/local/venv/bin/python3 review_autopass_log.py /data/media/0/realdata/<route>--0 ...
```

**Known limitation:** `auto_pass.py`'s `phase` property never actually emits
`AutoPassPhase.aborted` or `.completed` — those enum values exist in the capnp
schema, but the Python property only returns idle/monitoring/countdown/
executing. The script infers an abort as "countdown exited to anything other
than executing," which is reliable, but the raw telemetry cannot distinguish
abort *reasons*: wrong-direction gaze, low confidence, hard abort, and
confirm-window timeout all look identical. Worth adding real telemetry fields
if root-causing a specific abort ever matters.

## Path to trusting this on the road

In order, none of it skippable:

1. Reboot test on the current `params_pyx.so` (see below) — confirm normal
   operation survives a real boot.
2. Enable `AutoPassEnabled` in Settings.
3. Drive with `AutoPassShadowMode` **still on**. It logs and shows the
   direction-aware countdown alert but never steers.
4. Verify the telemetry actually recorded. Note that `autoPassStateSP` will
   still read 0 in the census — that is expected, see the loggerd section
   below. What matters is that `modelDataV2SP` carries a populated embedded
   `autoPass` copy. Check with:
   ```python
   for m in LogReader(".../rlog.zst"):
       if m.which() == "modelDataV2SP":
           print(m.modelDataV2SP.autoPass); break
   ```
5. Run `review_autopass_log.py` against the route and review the stability
   summary. Tune the placeholder constants against what it shows.
6. Only then consider disabling shadow mode.

## Device operations

### Never blindly reboot or restart the comma service

`/usr/comma/comma.sh` checks
`/sys/class/input/input2/device/touch_count` at boot and, if it reads `>4`,
launches `/usr/comma/reset --tap-reset` — a **factory reset**. This device has
been observed reporting stray values (49, 87, 6, 125) unrelated to any physical
touch. The check is gated by `/tmp/booted`, so it runs once per real boot —
which means a plain `sudo reboot` is just as dangerous as `systemctl restart
comma` whenever `touch_count` is elevated.

**Always check `touch_count` first.** If it's `>4`, avoid both, and restart
`manager.py` directly instead (below).

### Restarting manager.py by hand

Required after changing any file manager preimports — see next section.

```
ssh comma@<ip> 'bash -c "source /etc/profile && cd /data/openpilot && \
  setsid nohup ./launch_openpilot.sh > /tmp/launch_manual.log 2>&1 < /dev/null & sleep 0.5; echo STARTED"'
```

- **`source /etc/profile` is mandatory.** Without it the launch uses system
  `python3` instead of `/usr/local/venv`, and manager dies immediately with
  `ModuleNotFoundError: No module named 'capnp'`. A non-interactive ssh shell
  does not source it automatically.
- **Never `pkill -f <pattern>` over ssh when the pattern also appears in your
  own command line** — `pkill -f "launch_chffrplus.sh"` matches the remote
  `bash -c` running the very script you typed and kills it mid-sequence. Kill
  by explicit PID, or use a bracket pattern (`"manager[.]py"`).
- The launching ssh call often won't return promptly (the detached child holds
  the connection). Let it background and verify with a separate short call.
- Verify: `pgrep -af "manager[.]py"` (expect two — parent, plus the child that
  spawns everything) and `selfdrive.ui.u[i]`, then tail the launch log.

### Code changes need a manager restart, not just an ignition cycle

`system/manager/manager.py` calls `p.prepare()`, which `importlib.import_module`s
every managed module **in the manager parent**. `PythonProcess.start()` uses
`multiprocessing.Process` with no `set_start_method`, so Linux forks — and
forked children inherit the parent's already-imported `sys.modules`. Their own
`import_module` in `launcher()` is just a cache hit on the **old** code.

Editing a file manager preimports has no effect until manager itself restarts.
Going onroad is not enough. Confirm by comparing the file's mtime against
manager's start time.

### `DisableUpdates` is not live-polled

`system/updated/updated.py` reads `DisableUpdates` **once at process startup**.
Setting it while the daemon runs does not stop the current cycle, which does
`git fetch origin` → `git checkout --force` → `git reset --hard` →
`git clean -xdff` (this has silently reverted a patch and wiped untracked files
before).

Reboot-free way to apply it: `updated` is manager-managed, so set the param then
`kill <pid>`. Manager respawns it, the fresh process reads the flag and
`exit(0)`s immediately. Verify `pgrep -af system.updated.updated` stays empty.

A factory reset wipes `DisableUpdates` back to `0` — re-set it after any
reset or reinstall. Its fetch targets `origin` (upstream sunnypilot), not
`myfork`, so on a fork-only branch the fetch fails and it cannot clobber the
checkout — but do not rely on that as the safety mechanism.

### Rebuilding `params_pyx.so`

Only needed when `params_keys.h` gains new keys. Build natively on the device
(real aarch64) rather than under Docker's qemu emulation — the venv already
bundles the needed dev libs. From `/data/openpilot`:

```
g++ -shared -fPIC -O2 -std=c++17 -D__TICI__ \
  -I. -I/usr/include/python3.12 \
  -I/usr/local/venv/lib/python3.12/site-packages/capnproto/install/include \
  -I/usr/local/venv/lib/python3.12/site-packages/json11/install/include \
  -L/usr/lib/aarch64-linux-gnu \
  -L/usr/local/venv/lib/python3.12/site-packages/capnproto/install/lib \
  -L/usr/local/venv/lib/python3.12/site-packages/json11/install/lib \
  common/params_pyx.cpp common/params.cc common/swaglog.cc common/util.cc \
  -lpython3.12 -lcapnp -lkj -ljson11 -lzmq -lpthread \
  -o common/params_pyx.so
```

**`-D__TICI__` is not optional.** `system/hardware/hw.h` does `#if __TICI__` to
select `HardwareTici` vs `HardwarePC`. Without it the build silently uses
`HardwarePC`, whose `PC()` unconditionally returns true, which routes
`Path::params()` to `$HOME/.comma/params` instead of `/data/params`. A build
made this way is not "broken" in an obvious way — it reads and writes an
entirely *different, wrong directory*, invisible to every other process
including `manager.py`. The real-world symptom was that **all** param writes
system-wide silently failed: registration and driver-training state never
survived a reboot, forcing re-registration every single boot.

**Validation checklist before ever committing or deploying a rebuilt `.so`** —
the original failure got through because only key *recognition* was tested, and
that's a pure in-memory lookup unaffected by a wrong `params_path`:

1. Write a new key; confirm the file's real on-disk path and byte content
   (`od -c /data/params/d/<key>`).
2. Read it back from a **separate fresh process**, not the one that wrote it.
3. Read **pre-existing production data** (`DongleId`,
   `CompletedTrainingVersion`, `HasAcceptedTerms`) and confirm real values come
   back. This is the check whose absence caused the original regression.
4. Ideally have the device reboot and confirm normal operation before fully
   trusting it.

## Repo hygiene

On Windows, ~20 `.sh` files show as permanently modified in
`c:\dev\ADS-release-tizi` — this is pure LF→CRLF line-ending conversion, not
real edits. **Never `git add -A` or `git commit -a` here.** Committing CRLF into
`launch_openpilot.sh` and friends risks breaking the launch scripts on the
device's Linux shell. Always stage explicit paths.

## Deploying to the device

```
cd /data/openpilot
git fetch myfork auto-pass-dev
git merge --ff-only myfork/auto-pass-dev
```

Then restart `manager.py` by hand (see above) — **not** `systemctl restart
comma`, and not a reboot, while `touch_count` is elevated.
