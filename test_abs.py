import math

# if device is slightly tilted or drift occurs:
lin_acc_z = 0.05 # 0.05g noise
gravity_z = 1.0

vertical_accel_g = lin_acc_z * gravity_z
print(f"Original Accel: {vertical_accel_g}")
print(f"With ABS: {abs(vertical_accel_g)}")
