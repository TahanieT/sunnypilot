"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import time

import requests

from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot.navd.helpers import Coordinate

# The vendored mapd binary (third_party/mapd_pfeiferj) is a closed, prebuilt
# release binary with no exposed query interface for arbitrary OSM tags, so
# lane-count data is fetched independently here via a live Overpass query.
OVERPASS_URL = "https://overpass-api.de/api/interpreter"
QUERY_RADIUS_M = 30
REQUEST_TIMEOUT_S = 5.0
MIN_REQUERY_DISTANCE_M = 150.0
MIN_REQUERY_INTERVAL_S = 10.0
STALE_TIMEOUT_S = 30.0
CACHE_MAX_ENTRIES = 20

# v1 deliberately only recognizes oneway (divided/one-way) roads. Determining
# which side of a two-way road is "your direction" requires matching OSM way
# geometry against vehicle bearing -- exactly the kind of logic that, if
# subtly wrong, would produce the one failure mode this must never produce
# (steering into an oncoming lane). Undivided multi-lane-per-direction roads
# are out of scope until that's built and separately validated.
ALLOWED_HIGHWAY_CLASSES = {"motorway", "trunk", "primary", "secondary"}


def _grid_key(pos: Coordinate) -> tuple[int, int]:
  # ~100m cache cell, coarse on purpose -- this only dedupes repeated queries
  # on the same stretch of road, the staleness timeout is what bounds trust.
  return (round(pos.latitude * 1000), round(pos.longitude * 1000))


def _lane_count(tags: dict) -> int:
  for key in ("lanes:forward", "lanes"):
    value = tags.get(key)
    if value is None:
      continue
    try:
      return int(value)
    except (TypeError, ValueError):
      continue
  return 0


def _is_multi_lane_same_direction(tags: dict) -> bool:
  return (
    tags.get("oneway") == "yes"
    and tags.get("highway") in ALLOWED_HIGHWAY_CLASSES
    and _lane_count(tags) >= 2
  )


class LaneStatus:
  def __init__(self, valid: bool = False, same_direction: bool = False, resolved_at: float = 0.0):
    self.valid = valid
    self.same_direction = same_direction
    self.resolved_at = resolved_at

  def age(self, now: float) -> float:
    return now - self.resolved_at


class OsmLaneData:
  """
  Fail-closed multi-lane/same-direction road lookup for the Auto Pass feature.
  Any missing, stale, or ambiguous data resolves to "not multi-lane" -- callers
  must never treat "unknown" as permissive.
  """

  def __init__(self):
    self._cache: dict[tuple[int, int], LaneStatus] = {}
    self._last_query_pos: Coordinate | None = None
    self._last_query_time: float = 0.0
    self._status = LaneStatus()

  def _should_query(self, pos: Coordinate, now: float) -> bool:
    if self._last_query_pos is None:
      return True
    if now - self._last_query_time < MIN_REQUERY_INTERVAL_S:
      return False
    return pos.distance_to(self._last_query_pos) >= MIN_REQUERY_DISTANCE_M

  def _query_overpass(self, pos: Coordinate) -> dict | None:
    query = (
      f"[out:json][timeout:{int(REQUEST_TIMEOUT_S)}];"
      f"way(around:{QUERY_RADIUS_M},{pos.latitude},{pos.longitude})[highway];"
      f"out tags 1;"
    )
    try:
      response = requests.post(OVERPASS_URL, data={"data": query}, timeout=REQUEST_TIMEOUT_S)
      response.raise_for_status()
      elements = response.json().get("elements", [])
    except Exception as e:
      cloudlog.warning(f"OsmLaneData: overpass query failed: {e}")
      return None

    if not elements:
      return None

    return elements[0].get("tags", {})

  def update(self, pos: Coordinate | None, localizer_valid: bool) -> None:
    now = time.monotonic()

    if not localizer_valid or pos is None:
      self._status = LaneStatus()
      return

    if self._should_query(pos, now):
      self._last_query_pos = pos
      self._last_query_time = now

      key = _grid_key(pos)
      cached = self._cache.get(key)
      if cached is not None and cached.age(now) < STALE_TIMEOUT_S:
        self._status = cached
      else:
        tags = self._query_overpass(pos)
        if tags is not None:
          status = LaneStatus(valid=True, same_direction=_is_multi_lane_same_direction(tags), resolved_at=now)
          self._status = status
          if len(self._cache) >= CACHE_MAX_ENTRIES:
            self._cache.pop(next(iter(self._cache)))
          self._cache[key] = status
        # a single failed request leaves the prior status in place rather than
        # instantly blanking it -- the staleness check below still fail-closes
        # it once it's genuinely too old to trust.

    if self._status.age(now) > STALE_TIMEOUT_S:
      self._status = LaneStatus()

  def get_status(self) -> tuple[bool, bool, float]:
    now = time.monotonic()
    return self._status.valid, self._status.same_direction, self._status.age(now)
