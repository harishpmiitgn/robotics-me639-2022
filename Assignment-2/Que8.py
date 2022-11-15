# Intro To Robotics (Assignment-2)
# written by Kush Patel (20110131)

### RRP SCARA Configuration ###
import numpy as np
import math

def RRP_SCARA_jacob(l1,l2,q1,q2):
    q1 = math.radians(q1)
    q2 = math.radians(q2)
    J = np.array([[-l1*np.sin(q1) - l2*np.sin(q1+q2), -l2*np.sin(q1+q2), 0],
                  [l1*np.cos(q1) + l2*np.cos(q1+q2), l2*np.cos(q1+q2), 0],
                  [0                               ,0                  ,-1],
                  [0                               ,0                  , 0],
                  [0                               ,0                  , 0],
                  [1                               ,1                  , 0],])
    return J

l1 = 2   # Length of Link-1
l2 = 2   # Length of Link-2
q1 = 10   # Joint angle 1 in degrees
q2 = 20   # Joint angle 2 in degrees
jacobian = RRP_SCARA_jacob(l1,l2,q1,q2)
print(jacobian)