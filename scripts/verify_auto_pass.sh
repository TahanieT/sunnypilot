#!/usr/bin/env bash
# Build/test verification for the Auto Pass feature (auto-pass-dev branch), run
# inside the repo's own dev Docker image since this codebase needs Linux
# (capnp bindings, tinygrad models) and won't build on Windows directly.
#
# Usage: from a Linux/WSL shell with Docker running, from the repo root:
#   ./scripts/verify_auto_pass.sh
set -e

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT="$SCRIPT_DIR/.."
cd "$REPO_ROOT"

# Build the dev image for linux/arm64 (matches the comma 3X's real
# architecture). This branch is missing common/SConscript, selfdrive/SConscript
# etc. (present in later upstream syncs but never merged here), so `scons`
# can't run locally regardless of platform -- but common/params_pyx.so (and
# presumably other compiled extensions) are committed as prebuilt ARM64
# binaries, so building for arm64 lets them load as-is without a scons rebuild.
export TARGET_ARCHITECTURE=arm64
./selfdrive/test/docker_build.sh
IMAGE=openpilot-arm64:latest

# Git Bash/MSYS auto-converts leading-/ arguments into Windows paths; harmless
# here since we no longer pass any container-side path as a bare argument, but
# left disabled defensively in case that changes.
MSYS_NO_PATHCONV=1 docker run --rm --platform linux/arm64 \
  "$IMAGE" \
  bash -c '
    set -e
    echo "=== Auto Pass unit tests ==="
    pytest sunnypilot/selfdrive/controls/lib/tests/test_auto_pass.py -v

    echo "=== DesireHelper regression (AutoPassEnabled=False equivalence) ==="
    pytest sunnypilot/selfdrive/controls/lib/tests/test_desire_helper_auto_pass_regression.py -v

    echo "=== Sibling controller regressions ==="
    pytest sunnypilot/selfdrive/controls/lib/tests/test_auto_lane_change.py -v
    pytest sunnypilot/selfdrive/controls/lib/tests/test_blinker_pause_lateral.py -v

    echo "=== Regenerating settings_ui.json from steering.yaml ==="
    python sunnypilot/sunnylink/tools/compile_settings_ui.py

    echo "=== All checks passed ==="
  '
