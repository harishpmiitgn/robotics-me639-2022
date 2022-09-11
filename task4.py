
import numpy as np
from matplotlib import pyplot as plt

def fk_2R(q1,q2,l1,l2):
    x1 = l1*np.cos(q1)
    y1 = l2*np.sin(q1)
    x2 = x1+l2*np.cos(q2)
    y2 = y1+l2*np.sin(q2)
    return [x1,y1,x2,y2]

def ik_2R(x,y,elbow,l1,l2):
    
    D = (x**2+y**2-l1**2-l2**2)/(2*l1*l2)
    if D>1 or D<-1:
        q1 = 0
        q2 = 0
        ws = 0 # Outside workspace
    else:
        ws = 1 # Inside workspace
        theta2 = np.arctan2(elbow*np.sqrt(1-D**2),D)
        q1 = np.arctan2(y,x)-np.arctan2(l2*np.sin(theta2),(l1+l2*np.cos(theta2)))
        q2 = q1+theta2
        
        

    return [ws,q1,q2]

def ik_2R_array(xa,ya,elbow,l1,l2):
    
    q = np.zeros((len(xa),2))
    for i in range(len(xa)):
        x = xa[i]
        y = ya[i]
        print(x)
        D = (x**2+y**2-l1**2-l2**2)/(2*l1*l2)
        if D>1 or D<-1:
            q1 = 0
            q2 = 0
            ws = 0 # Outside workspace
        else:
            ws = 1 # Inside workspace
            theta2 = np.arctan2(elbow*np.sqrt(1-D**2),D)
            q1 = np.arctan2(y,x)-np.arctan2(l2*np.sin(theta2),(l1+l2*np.cos(theta2)))
            q2 = q1+theta2
            q[i] = np.array([q1,q2])
    return q



def plot_joint(q1,q2,l1,l2):
        [x1,y1,x2,y2]=fk_2R(q1,q2,l1,l2)
        
        plt.plot([x2],[y2],'ok')
        plt.xlim([-1.1*(l1+l2),1.1*(l1+l2)])
        plt.ylim([-1.1*(l1+l2),1.1*(l1+l2)])
        

l1 = 1
l2 = 1
q1 = np.linspace(35*np.pi/180,145*np.pi/180,101)
q2 = np.linspace(35*np.pi/180,145*np.pi/180,101)

for i in range(len(q1)):
    for j in range(len(q2)):
        plot_joint(q1[i],q2[j],l1,l2)
plt.show()