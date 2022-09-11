# Intro To Robotics (Assignment-2)
# written by Kush Patel (20110131)

### RRP Stanford Configuration
import numpy as np
import math 

def RRP_stanford_Transformation(q1,q2,p3):
    q1 = math.radians(q1)
    q2 = math.radians(q2)
    # Joint 2 and Joint 3
    H23 = np.array([[1,0,0,l2],
                    [0,1,0,0 ],
                    [0,0,1,0],
                    [0,0,0,1]])
    # Joint 1 and Joint 2
    H12 = np.array([[np.cos(q2),-np.sin(q2),0,0],                    
                    [0,0,-1,0],
                    [np.sin(q2),np.cos(q2),0,l1 ],
                    [0,0,0,1]])
    # Initial frame and Joint 1
    H01 = np.array([[np.cos(q1),-np.sin(q1),0,0],
                    [np.sin(q1),np.cos(q1),0,0 ],
                    [0,0,1,0],
                    [0,0,0,1]])
    
    p3 = np.transpose(p3)
    ans = np.matmul(np.matmul(np.matmul(H01,H12),H23),p3)
    p0 = ans[:-1]
    return p0


q1 = 10     # Joint angle 1 in degrees
q2 = 20     # Joint angle 2 in degrees
dpz = 5     # Movement of Prismatic Joint
l1 = 3      # Length of Link-1
l2 = 3      # Length of Link-2
p3 = [0,0,dpz,1]   

p0 = RRP_stanford_Transformation(q1,q2,p3)
print(p0)
