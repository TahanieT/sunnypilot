"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import pyray as rl

from cereal import custom
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app
from openpilot.common.filter_simple import FirstOrderFilter

Phase = custom.AutoPassStateSP.Phase

INDICATOR_SIZE = 36
INDICATOR_MARGIN_X = 20
INDICATOR_MARGIN_Y = 20
READY_COLOR = rl.Color(0x00, 0xc8, 0x50, 0xff)  # green


class AutoPassIndicator:
  """Small persistent square shown whenever AutoPass's trigger permissives
  are all currently met (phase == monitoring -- armed to countdown as soon
  as DesireHelper picks it up next frame), visible in shadow mode as much as
  live, since phase computation is unaffected by shadow_mode. Not the
  countdown alert -- this is the step before that, for seeing at a glance
  that conditions are being met at all, not just after a countdown starts."""

  def __init__(self):
    self._alpha_filter = FirstOrderFilter(0, 0.15, 1 / gui_app.target_fps)

  def update(self) -> None:
    ap = ui_state.sm['autoPassStateSP']
    ready = ap.phase == Phase.monitoring
    self._alpha_filter.update(1.0 if ready else 0.0)

  def render(self, rect: rl.Rectangle) -> None:
    if self._alpha_filter.x <= 0.01:
      return

    pos_x = int(rect.x + rect.width - INDICATOR_MARGIN_X - INDICATOR_SIZE)
    pos_y = int(rect.y + INDICATOR_MARGIN_Y)
    alpha = int(255 * self._alpha_filter.x)
    color = rl.Color(READY_COLOR.r, READY_COLOR.g, READY_COLOR.b, alpha)
    rl.draw_rectangle(pos_x, pos_y, INDICATOR_SIZE, INDICATOR_SIZE, color)
