import math
import matplotlib.pyplot as plt
import numpy as np
from render import Renderer
import cv2

class object(Renderer):
    def __init__(self, recordLocation=None):
        super().__init__(recordLocation=recordLocation)
        self.x=0
        self.y=0
        self.x0=200
        self.y0=400
        self.q1=0
        self.q2=0
        self.l1=100
        self.l2=100

        #setting up iteration number
        self.its =1
        self.traj=[]


    def getInfo(self):
        info = {
            'x2' : round(self.x, 4),
            'y2' : round(self.y, 4),
        }
        return info


    def calculations(self):
        
        #let us consider our trajectory to be a circle of radius=50 and centre as (400,200)
        radius=50
        xc=400
        yc=200
        x = radius*np.cos(self.its/100) +xc
        y = radius*np.sin(self.its/100) +yc

        #making the start point of link 1 as the origin
        self.x= x-400
        self.y= y-200

        #Calculating angles
        theta= np.arccos((self.x**2+self.y**2-self.l1**2-self.l2**2)/(2*self.l1*self.l2))
        self.q1 = np.arctan(self.y/self.x) - np.arctan((self.l2*np.sin(theta))/(self.l1+self.l2*np.cos(theta)))
        self.q2 = self.q1+theta
       

        if x<400:
           self.q1= -np.pi+self.q1
           self.q2= -np.pi+self.q2

        #Joint and end-effector pointa
        self.x1=self.x0+int(self.l1*np.cos(self.q1))
        self.y1=self.y0+int(self.l1*np.sin(self.q1))
        self.x2=xc+int(self.l1*np.cos(self.q1))+int(self.l2*np.cos(self.q2))
        self.y2=yc+int(self.l1*np.sin(self.q1))+int(self.l2*np.sin(self.q2))

        self.its +=1
        self.traj.append((int(x), int(y)))
        


    def draw(self, image):
        #Drawing the trajectory
        for x, y  in self.traj:
            cv2.circle(image, (x, y), 1, (120, 0, 0), 1)

        #Drawing the links of the robot
        cv2.line(image, (self.x0, self.y0),(self.x1,self.y1), (0, 0, 255), 1)      
        cv2.line(image, (self.x1,self.y1) , (int(self.x2),int(self.y2)), (0, 255, 0), 1)

        return image

obj = object()    

for i in range(1000):
    obj.calculations() 
    if i % 1 == 0:
       obj.render(height= 600, pause = 10)