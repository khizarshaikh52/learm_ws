#!/usr/bin/env python3
import math
from typing import List, Tuple

# Example link lengths (mm) � REPLACE with your LeArm values
L1, L2, L3 = 90.0, 110.0, 100.0

# Joint order:
# [shoulder_pan, shoulder_lift, elbow1, elbow2, wrist_flex, wrist_roll, grip_left]

def ik_planar_xyz(target_xyz: Tuple[float, float, float]) -> List[float]:
    x, y, z = target_xyz
    yaw = math.degrees(math.atan2(y, x))               # shoulder_pan
    r = math.hypot(x, y)
    z_eff = z

    d = math.hypot(r, z_eff)
    d = max(min(d, L1 + L2 - 1e-3), 1e-3)

    cos_e = (L1*L1 + L2*L2 - d*d) / (2*L1*L2)
    cos_e = max(min(cos_e, 1.0), -1.0)
    elbow = math.degrees(math.acos(cos_e)) - 180.0     # elbow1 (signed)

    phi = math.degrees(math.atan2(z_eff, r))
    cos_s = (L1*L1 + d*d - L2*L2) / (2*L1*d)
    cos_s = max(min(cos_s, 1.0), -1.0)
    theta = math.degrees(math.acos(cos_s))
    shoulder_lift = phi + theta

    wrist_flex = - (shoulder_lift + elbow) * 0.5
    wrist_roll = 0.0
    grip = 0.0

    return [yaw, shoulder_lift, elbow, 0.0, wrist_flex, wrist_roll, grip]
