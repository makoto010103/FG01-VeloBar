import math

def sim(gx, gy, gz, ax, ay, az, noise=False):
    lin_acc_x = ax - gx
    lin_acc_y = ay - gy
    lin_acc_z = az - gz
    
    vertical_accel_g = (lin_acc_x * gx) + (lin_acc_y * gy) + (lin_acc_z * gz)
    
    if gz < 0:
        vertical_accel_g = -vertical_accel_g
        
    print(f"Gravity_Z={gz} | Result={vertical_accel_g}")


print("--- Normal Orientation (Upright, noise) ---")
# gz = 1, ax = 1.05
sim(0,0,1, 0,0,1.05)
sim(0,0,1, 0,0,0.95)

print("\n--- Upside Down Orientation (Upside down, noise) ---")
# gz = -1, az = -1.05
sim(0,0,-1, 0,0,-1.05)
sim(0,0,-1, 0,0,-0.95)

