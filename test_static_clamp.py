# Acc_mag is the absolute magnitude of the accelerometer vector.
# Gravity is 1.0g.
# If I squat smoothly with 150kg, my acceleration might only be 0.4 m/s^2.
# 0.4 m/s^2 / 9.8 = 0.04g.
# Total acc_mag = 1.04g.
# The code: bool acc_near_1g = (acc_mag > 0.95f && acc_mag < 1.05f);
# If acc_mag is 1.04g, acc_near_1g is TRUE.
# If it's true, AND gyro is low, it triggers is_static!
# If is_static is true, Velocity = 0.000 !!
print("A smooth squat will literally output 0.000 because of the 1.05g clamp!")
