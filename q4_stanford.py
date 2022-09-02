import numpy as np
import math

def stanford_posvector_groundframe(l1,l2,theta1,theta2,l):

    #the derivation of this is given in the pdf q4:)
    
    p0 = np.array([l2*math.cos(theta1)-l*math.cos(theta1)*math.sin(theta1)],[l*(math.cos(theta1)**2)+l2*math.sin(theta1)],[l1+l*math.sin(theta2)])
    return p0

l1 = int(input("enter the length of link 1:"))
l2 = int(input("enter the length of link 2:"))
theta1 = int(input("enter the angle of link 1 degrees:"))
theta2 = int(input("enter the angle of link 2 degrees:"))
#since the motion for the prismatic joint at the last joint (end effector) is along the y axis enter the just the z coordinate of choice
l = int(input("enter the z co-ordinate of the end effector:"))

print("p0:")
print (stanford_posvector_groundframe(l1,l2,theta1,theta2,l))