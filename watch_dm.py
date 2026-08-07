#!/usr/bin/env python3
import time
import cereal.messaging as messaging

sm = messaging.SubMaster(['driverMonitoringState'])

print(f"{'faceDet':>8} {'pose':>6} {'eye':>6} {'phone':>6} {'isDistr':>8} {'alertLvl':>9} {'policy':>10}")

last_print = 0
while True:
  sm.update(100)
  if sm.updated['driverMonitoringState']:
    dm = sm['driverMonitoringState']
    now = time.monotonic()
    if now - last_print > 0.2:  # ~5Hz print
      last_print = now
      vp = dm.visionPolicyState
      print(f"{str(vp.faceDetected):>8} {str(vp.distractedTypes.pose):>6} {str(vp.distractedTypes.eye):>6} "
            f"{str(vp.distractedTypes.phone):>6} {str(vp.isDistracted):>8} {str(dm.alertLevel):>9} {str(dm.activePolicy):>10}")
