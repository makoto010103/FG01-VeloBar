import csv

# We will read the user's latest trace (which had 0.000 for 6 seconds)
# and try to reverse-engineer why ZUPT triggers so hard in the case.
# But wait, we don't have RAW gyro/accel data in the CSV. We only have velocity.
# 
# Wait, let's look at the sequence again:
# SQ,0,2026-02-28T09:51:50.281Z,0.000
# ...
# SQ,0,2026-02-28T09:51:56.956Z,0.023
# SQ,0,2026-02-28T09:51:57.691Z,2.062
# 
# Wait, when the device is inside the case, it doesn't rattle.
# When it DOES NOT rattle, `acc_mag` stays very close to 1.0f.
# If `acc_near_1g` (0.95 to 1.05) triggers, AND `gyro_mag < 15.0f`, AND `abs(acc_mag - 1.0) < 0.25`
# Then `is_static = true` is set.
# 
# Is it possible that a slow, smooth squat in a case NEVER exceeds 1.05g?
# Accelerating slowly from rest: 1.0g -> 1.04g.
# If it never exceeds 1.05g, `acc_near_1g` remains TRUE for the entire squat!
# Wait: `acc_near_1g` is defined as `(acc_mag > 0.95f && acc_mag < 1.05f)`
# If you squat very smoothly at heavy weights, your peak acceleration might only be 0.04g.
# Therefore, 1.04g < 1.05g.
# AND because it's in a tight case, it doesn't rotate, so gyro_mag < 15.0dps.
# If BOTH are true, the device thinks it is STATIC the ENTIRE SQUAT.

print("If the squat is perfectly smooth (no rattle) and slow, acc_mag never leaves [0.95, 1.05].")
print("Without the case, the rattling adds high frequency noise (e.g. 1.2g spikes).")
print("Those spikes break 'acc_near_1g', allowing the velocity to integrate!")
