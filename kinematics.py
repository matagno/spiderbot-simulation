
import math
import numpy as np

##################################
#       Robot description        #
#
# Thigh length : 45.5 mm + 49 mm = 94.5             // 60 mm + 76.84 mm = 136.84 mm
# Foot length : 97.64 mm                            // 128.05 mm           
# (94.529 mm, -24.467 mm)                           // Osef
#
# Offset foot angle : 14.51 deg (0.2443461 rad)     // 9.5 deg (0,15708 rad)
#
# Result : Hip, Knee, Foot
#


def clamp(x, a=-1.0, b=1.0):
    return max(a, min(b, x))


def ik_leg(target, leg_rotation=0.0, leg_base=(0.0,0.0,0.0), coxa=60, tl=76.84, fl=128.05, offset_foot_angle=0.15708):
    local_target = np.array(target) - np.array(leg_base)
    rot = np.array([[math.cos(-leg_rotation), -math.sin(-leg_rotation), 0],
                    [math.sin(-leg_rotation),  math.cos(-leg_rotation), 0],
                    [0, 0, 1]])
    local_target = rot @ local_target
    x, y, z = local_target.tolist()

    # Projection for knee and foot
    horiz = math.hypot(x, y)
    ex = horiz - coxa
    ez = z
    l = math.hypot(ex, ez)
    if l > (tl+fl) or l < abs(tl-fl) :
        print("Impossible to go here")
        return

    # Hip
    hip = (math.pi/2) - math.atan2(y, x) 

    # Foot
    foot = math.acos(clamp((tl**2 + fl**2 - l**2) / (2.0 * tl * fl))) - math.pi + offset_foot_angle

    # Knee
    va = - math.atan2(ez, ex)
    vb = math.acos(clamp((l**2 + tl**2 - fl**2) / (2.0 * l * tl)))
    knee = va - vb

    return np.array([hip, knee, foot]) 




