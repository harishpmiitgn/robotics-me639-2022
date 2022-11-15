# Intro To Robotics (Assignment-2)
# written by Kush Patel (20110131)

### RRR Configuration ###

import numpy as np
import math

def RRR_jacob(l1,l2,l3,q1,q2,q3):
    q1 = math.radians(q1)
    q2 = math.radians(q2)
    q3 = math.radians(q3)
    J = np.array([[-l1*np.sin(q1) - l2*np.sin(q1+q2) - l3*np.sin(q1+q2+q3) , -l2*np.sin(q1+q2) - l3*np.sin(q1+q2+q3), - l3*np.sin(q1+q2+q3)],
                  [l1*np.cos(q1) + l2*np.cos(q1+q2) + l3*np.cos(q1+q2+q3), l2*np.cos(q1+q2) + l3*np.cos(q1+q2+q3), l3*np.cos(q1+q2+q3)],
                  [0                               ,0                  ,0],
                  [0                               ,0                  , 0],
                  [0                               ,0                  , 0],
                  [1                               ,1                  , 1],])
    return J

l1 = 2   # Length of Link-1
l2 = 2   # Length of Link-2
l3 = 2   # Length of Link-3
q1 = 10   # Joint angle 1 in degrees
q2 = 20   # Joint angle 2 in degrees
q3 = 30   # Joint angle 3 in degrees
jacobian = RRR_jacob(l1,l2,l3,q1,q2,q3)
print(jacobian)