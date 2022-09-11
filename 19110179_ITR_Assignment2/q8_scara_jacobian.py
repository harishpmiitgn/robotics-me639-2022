import numpy as np
import math

def scara_jacobian(l1,l2,theta1,theta2):

    #derivation of jacobian given in the pdf q7 :)
    
    J = np.array([[-l2*math.sin(theta1+theta2)-l1*math.sin(theta1),-l2*math.sin(theta1+theta2),0],[l2*math.cos(theta1+theta2)+l1*math.cos(theta1),l2*math.cos(theta1+theta2),0],[0,0,1],[0,0,0],[0,0,0],[1,1,0]])

    return J



l1 = int(input("enter the length of link 1:"))
l2 = int(input("enter the length of link 2:"))
theta1 = int(input("enter the angle of link 1 degrees:"))
theta2 = int(input("enter the angle of link 2 degrees:"))


print("J:")
print (scara_jacobian(l1,l2,theta1,theta2)) 