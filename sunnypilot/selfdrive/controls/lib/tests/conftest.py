"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import pytest

from openpilot.sunnypilot.selfdrive.controls.lib import auto_pass as auto_pass_module


class _StubParams:
  """Bypasses the compiled params key-validation table, which is stale on this
  branch's Docker dev image: common/params_pyx.so predates AutoPassEnabled/
  AutoPassShadowMode being added to params_keys.h, and this branch is missing
  the SConscript needed to rebuild it locally (see verify_auto_pass.sh).
  Every DesireHelper()-constructing test in this directory hits this via
  AutoPassController.__init__ -> read_params(), regardless of whether the
  test is actually about Auto Pass. Tests that care about enabled/shadow_mode
  override those attributes directly after construction, so the values
  returned here are never read for anything meaningful -- this only needs to
  let construction succeed without touching the real, stale compiled
  extension."""
  def get_bool(self, key):
    return False


@pytest.fixture(autouse=True)
def _stub_auto_pass_params(monkeypatch):
  monkeypatch.setattr(auto_pass_module, "Params", _StubParams)
