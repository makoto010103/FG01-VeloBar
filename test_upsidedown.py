import math
import numpy as np

def sim(gx, gy, gz, ax, ay, az):
    print(f"Inputs: Accel({ax},{ay},{az})")
    
    # Simple dot product simulation based on arbitrary initialized Q
    # Let's say device is initialized upright. Q corresponds to gravity on Z.
    # Therefore gravity_z = 1.0, gravity_x = 0, gravity_y = 0
    gravity_x, gravity_y, gravity_z = 0, 0, 1.0
    
    lin_acc_x = ax - gravity_x
    lin_acc_y = ay - gravity_y
    lin_acc_z = az - gravity_z
    
    vertical_accel_g = (lin_acc_x * gravity_x) + (lin_acc_y * gravity_y) + (lin_acc_z * gravity_z)
    print(f"Vertical Accel: {vertical_accel_g * 9.80665}")

# Now, user drops the device upside down in the case.
# The IMU now registers gravity as -1.0 on the Z axis.
print("--- Upside down, stationary ---")
sim(0,0,0, 0, 0, -1.0)

# The user squats (upwards). Since it's upside down, upward movement
# causes a positive acceleration in the Z axis (+0.5G from movement).
# Total accel = -1g (gravity) + 0.5g (movement) = -0.5g.
print("\n--- Upside down, Squatting ---")
sim(0,0,0, 0, 0, -0.5)

