# A 1RM squat takes maybe 2 seconds to complete the concentric phase.
# Distance traveled is about 0.6 meters.
# Using s = ut + 0.5at^2 for a constant acceleration simplification:
# 0.6 = 0 + 0.5 * a * (2)^2
# 0.6 = 2a
# a = 0.3 m/s^2

print(f"Average acceleration for a 2-second, 0.6m squat: {0.3} m/s^2")
# In G's:
print(f"In Gs: {0.3 / 9.80665} G")

# What about a 3-second grinder squat?
# 0.6 = 0.5 * a * (3)^2 -> 0.6 = 4.5a -> a = 0.133 m/s^2
print(f"Average acceleration for a 3-second, 0.6m squat: {0.133} m/s^2")
print(f"In Gs: {0.133 / 9.80665} G")
