# If we simply abolish the static clamp while moving, and ONLY clamp when velocity is already near 0 AND acc is near 1.
# Current code:
# if (is_static) { velocity *= 0.8; if (abs(velocity) < 0.01) velocity = 0; vertical_accel_mps2 = 0; }
# This means if is_static Triggers mid-rep, it kills the rep.
# But what if is_static DEPENDS on velocity being low?
# "If we are already moving, do not apply static clamp unless we have STOPPED moving."
# Wait, ZUPT is supposed to correct drift. If we drift to 0.5m/s while standing still, we need is_static to activate to bring it to 0.
# So we can't just check `velocity < 0.1` because velocity might be 0.5 due to drift.
#
# BUT! If we use a stricter AC filter: 
# Gyro is usually < 2.0dps when on a rack. During a squat, even a smooth one, gyro is at least 10-20dps because humans wobble slightly.
# Let's check the current gyro threshold: gyro_mag < 15.0f.
# 15.0dps is actually quite high! 15 degrees per second.
# When the bar is racked, it should be 0-2 dps.
# If we change the static condition to gyro_mag < 3.0f, then ONLY a truly 
# racked bar (or rock solid floor) will trigger it. A human holding the bar WILL have gyro > 3.0.

print("Idea: tighten gyro_mag < 15.0f to gyro_mag < 4.0f.")
