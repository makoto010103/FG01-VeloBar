import math

class ZUPT_Test:
    def __init__(self):
        self.zupt_frames = 0
        self.is_static = True
        self.velocity = 0.0
        
    def update(self, ax_g, gy_dps):
        # Current logic
        acc_near_1g = (ax_g > 0.97 and ax_g < 1.03)
        if acc_near_1g and gy_dps < 15.0:
            self.zupt_frames += 1
            if self.zupt_frames > 15:
                self.is_static = True
        else:
            self.zupt_frames = 0
            # Wait, currently the code NEVER SETS is_static TO FALSE!
            # Let me check the original code!
            # ... wait.
            # ah, is_static is initialized to FALSE at the top of the loop!
            pass
            
print("Checking is_static initialization...")
