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

# Build the dev image (same image used by selfdrive/test/docker_build.sh).
./selfdrive/test/docker_build.sh

# Run inside the container with the live checkout mounted over the image's
# COPY'd source, so it tests your actual current auto-pass-dev tree.
docker run --rm \
  -v "$REPO_ROOT:/home/batman/openpilot" \
  -w /home/batman/openpilot \
  openpilot:latest \
  bash -c '
    set -e
    echo "=== scons build ==="
    scons -j4

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
