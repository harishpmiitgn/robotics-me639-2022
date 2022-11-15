# HPM's Intro To Robotics (Assignment-4&5)
# written by Kush Patel (20110131)
# Question - 2

import numpy as np
import math

def FK_SCARA(q1,q2,p3):
    q1 = math.radians(q1)
    q2 = math.radians(q2)
    # Joint 2 and Joint 3
    H23 = np.array([[1,0,0,l2],
                    [0,1,0,0 ],
                    [0,0,1,0],
                    [0,0,0,1]])
    # Joint 1 and Joint 2
    H12 = np.array([[np.cos(q2),-np.sin(q2),0,l1],
                    [np.sin(q2),np.cos(q2),0,0 ],
                    [0,0,1,0],
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

l1 = 5
l2 = 4

def IK_SCARA(x,y,z):
    theta2 = np.arccos((x**2+y**2-l1**2-l2**2)/(2*l1*l2))
    # theta2 = np.arctan2((math.sqrt(1-D**2)),D)
    theta1 = np.arctan2(y,x) - np.arctan2((l2*np.sin(theta2)),(l1 + l2*np.cos(theta2)))
    d3 = z
    theta1 = math.degrees(theta1)
    theta2 = math.degrees(theta2)
    return theta1,theta2,d3

n = 3      # Number of links
dpx = 5    # 3rd joint variable of SCARA [P joint - distance]
d = [0,0,dpx,1]
x = FK_SCARA(30,60,d)      # Forward Kinematics (q1,q2,d3)
y = IK_SCARA(x[0],x[1],x[2])  # Inverse Kinematics (px,py,pz) - End Effector position
print(x)
print(y)


### NOTE ###

# Here the inputs are joint variable of SCARA configuration and it goes to 
# Forward Kinematics fucntion and we are getting the end effector postion.

# This end effector postion again goes to Inverse Kinematics fucntion and 
# we are getting the same value that we gave initially as an input.