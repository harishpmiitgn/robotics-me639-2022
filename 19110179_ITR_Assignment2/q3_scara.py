import numpy as np
import math

def scara_posvector_groundframe(l1,l2,theta1,theta2,l):
    
    #the derivation of this is given in the pdf q2:)
    p0 = np.array([[l2*math.cos(theta1+theta2)+l1*math.cos(theta1)],[l2*math.sin(theta1+theta2)+l1*math.sin(theta1)],[l1+l]])
    return p0


l1 = int(input("enter the length of link 1:"))
l2 = int(input("enter the length of link 2:"))
theta1 = int(input("enter the angle of link 1 degrees:"))
theta2 = int(input("enter the angle of link 2 degrees:"))
#since the motion for the prismatic joint at the last joint (end effector) is along the z axis enter the just the z coordinate of choice
l = int(input("enter the z co-ordinate of the end effector:"))

print (scara_posvector_groundframe(l1,l2,theta1,theta2,l))


