import math
import numpy as np

def sim(ax, ay, az):
    acc_mag = math.sqrt(ax*ax + ay*ay + az*az)
    print(f"acc_mag: {acc_mag}")
    # Simulating the static reset "q" (device just resting sideways)
    # The true gravity is down (say Z is down, but device is turned 90 deg so gravity is on Y)
    
    # We cheat the Madgwick filter gravity here by saying filter q represents gravity on Y axis
    gravity_x, gravity_y, gravity_z = 0.0, 1.0, 0.0
    
    # Now user does a squat (accelerates upwards, meaning negative acceleration on Y axis)
    lin_acc_x = ax - gravity_x
    lin_acc_y = ay - gravity_y
    lin_acc_z = az - gravity_z
    
    vertical_accel_g = (lin_acc_x * gravity_x) + (lin_acc_y * gravity_y) + (lin_acc_z * gravity_z)
    print(f"vertical_accel_mps2: {vertical_accel_g * 9.80665}")

print("--- Device Normal (Upright, Gravity on Z) ---")
# Normal gravity is 1g on Z. During squat lift, we accelerate UP. Acceleration read is > 1g.
sim(0, 0, 1.5)

print("\n--- Device Sideways in Case (Gravity on Y) ---")
# Gravity 1g on Y. Squat upwards means acc read is > 1g on Y.
sim(0, 1.5, 0)
