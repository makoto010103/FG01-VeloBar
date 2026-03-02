import math

def sim_madgwick(ax, ay, az, gx, gy, gz):
    # Madgwick already gives us the gravity vector relative to the sensor frame.
    # For example, if the device is upside down (Z points up), gravity (which goes down)
    # will appear to point in the -Z direction. So gz = -1.0.
    
    # Linear acceleration is measured - gravity.
    # If the device is upside down, and we accelerate UPWARDS:
    # Sensor accelerates in +Z direction (since its top is pointing down).
    # Wait, if device is upside down, its +Z arrow points to the floor.
    # Accelerating towards the ceiling means accelerating in the -Z direction.
    # So ax=0, ay=0, az = -0.5G (movement) + 1.0G (gravity pushing up on the mass) = +0.5G ?
    # Let's check accelerometer physics.
    pass

def test_dot_product(ax, ay, az, gx, gy, gz):
    lin_x = ax - gx
    lin_y = ay - gy
    lin_z = az - gz
    
    # The dot product of linear acceleration and the gravity vector
    # project the linear acceleration ONTO the gravity vector.
    # Since gravity points DOWN, a positive dot product means acceleration is DOWNWARD.
    # A negative dot product means acceleration is UPWARD.
    dot = (lin_x * gx) + (lin_y * gy) + (lin_z * gz)
    
    # We want UPWARD acceleration to be POSITIVE.
    # So vertical_accel should be -dot
    vert_accel = -dot
    print(f"Accel({ax},{ay},{az}) | Grav({gx},{gy},{gz}) -> LinZ: {lin_z} | Dot: {dot} | Vert: {vert_accel}")

# 1. Normal Upright. Gravity points down -> +Z in sensor?
# Usually, resting flat on table, Z axis feels +1G normal force. So ax=0, ay=0, az=1.
# This means the gravity vector algorithm will estimate gx=0, gy=0, gz=1.
print("--- Upright ---")
test_dot_product(0, 0, 1.0,  0, 0, 1.0) # Resting
# Accelerating UPWARDS (+Z direction). Sensor feels +Z normal force + +Z acceleration force.
test_dot_product(0, 0, 1.5,  0, 0, 1.0) 

# 2. Upside Down. Gravity points down -> -Z in sensor.
# Resting flat upside down, Z axis feels -1G normal force.
print("\n--- Upside Down ---")
test_dot_product(0, 0, -1.0, 0, 0, -1.0) # Resting
# Accelerating UPWARDS (which is -Z direction for the sensor).
# Normal force is -1G, plus -0.5G from upward acceleration = -1.5G total.
test_dot_product(0, 0, -1.5, 0, 0, -1.0)

