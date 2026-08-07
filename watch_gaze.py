#!/usr/bin/env python3
import time
import cereal.messaging as messaging

sm = messaging.SubMaster(['driverMonitoringState'])

print(f"{'yaw_deg':>10} {'pitch_deg':>10} {'calibrated':>11} {'uncertainty':>12}")

last_print = 0
while True:
  sm.update(100)
  if sm.updated['driverMonitoringState']:
    dm = sm['driverMonitoringState']
    now = time.monotonic()
    if now - last_print > 0.2:
      last_print = now
      pose = dm.visionPolicyState.pose
      yaw_deg = pose.yaw * 57.29578
      pitch_deg = pose.pitch * 57.29578
      print(f'{yaw_deg:>10.1f} {pitch_deg:>10.1f} {str(pose.calibrated):>11} {pose.uncertainty:>12.3f}')
