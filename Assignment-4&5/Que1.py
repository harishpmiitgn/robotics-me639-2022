# HPM's Intro To Robotics (Assignment-4&5)
# written by Kush Patel (20110131)
# Question - 1

import numpy as np
import math

# def IK_spherical(x,y,z):
#     r = math.sqrt(x**2+y**2)
#     s = z - l1
#     theta1 = np.arctan2(y,x)
#     theta2 = np.arctan2(s,r)
#     d3 = math.sqrt(r**2 + s**2) - l2
#     theta1 = math.degrees(theta1)
#     theta2 = math.degrees(theta2)
#     return theta1, theta2, d3

def IK_Stanford(x,y,z):
    r = math.sqrt(x**2+y**2)
    s = z - l1
    theta1 = np.arctan2(y,x) 
    theta2 = -np.arctan2(-s,r) + np.arctan2(d3,l2)
    d3 = math.sqrt(r**2 + s**2  - l2**2)
    return theta1,theta2,d3

px = 5
py = 5
pz = 0
l1 = 5
l2 = 5
# ans = IK_spherical(px,py,pz)
ans = IK_Stanford(px,py,pz)
print(ans)

