# HPM's Intro To Robotics (Assignment-4&5)
# written by Kush Patel (20110131)
# Question - 6

import numpy as np
import math

def IK_sprical_wrist(U):
    theta = np.arctan2(math.sqrt(1-U[2][2]**2),U[2][2])
    phi = np.arctan2(U[1][2],U[0][2])
    psi = np.arctan2(U[2][1],-U[2][0])
    theta = math.degrees(theta)
    phi = math.degrees(phi)
    psi = math.degrees(psi)
    return theta,phi,psi

U = np.random.randn(3,3)
ans = IK_sprical_wrist(U)
print(ans)


### NOTE ###
# Here, user only needs to give the U matrix as an input then the above code 
# will return the values of Euler angles (theta,phi,psi)