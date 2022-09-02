import numpy as np
import math

def RRR_jacobian(l1,l2,l3,theta1,theta2,theta3):

    #derivation given in the pdf q8v:)
    
    J = np.array([[-l3*math.sin(theta1+theta2+theta3)-l2*math.sin(theta1+theta2)-l1*math.sin(theta1),-l3*math.sin(theta1+theta2+theta3)-l2*math.sin(theta1+theta2),-l3*math.sin(theta1+theta2+theta3)],
                    [l3*math.cos(theta1+theta2+theta3)+l2*math.cos(theta1+theta2)+l1*math.cos(theta1),l3*math.cos(theta1+theta2+theta3)+l2*math.cos(theta1+theta2),l3*math.cos(theta1+theta2+theta3)],
                        [0,0,0],[0,0,0],[0,0,0],[1,1,1]])
    return J



l1 = int(input("enter the length of link 1:"))
l2 = int(input("enter the length of link 2:"))
l3 = int(input("enter the length of link 3:"))
theta1 = int(input("enter the angle of link 1 degrees:"))
theta2 = int(input("enter the angle of link 2 degrees:"))
theta3 = int(input("enter the angle of link 3 degrees:"))

print("J:")
print (RRR_jacobian(l1,l2,l3,theta1,theta2,theta3))