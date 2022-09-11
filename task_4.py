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


    def calculations(self,min_angle,max_angle):
        for q1 in range(min_angle,max_angle+1):
            for q2 in range(min_angle,max_angle+1):
                self.q1=-q1*math.pi/180
                self.q2=-q2*math.pi/180
                self.render()

         #Joint and end-effector points
        # self.x1=self.x0+int(self.l1*np.cos(self.q1))
        # self.y1=self.y0+int(self.l1*np.sin(self.q1))
        # self.x2=self.x1+int(self.l1*np.cos(self.q1))+int(self.l2*np.cos(self.q2))
        # self.y2=self.y1+int(self.l1*np.sin(self.q1))+int(self.l2*np.sin(self.q2))

        
        


    def draw(self, image):
        #Drawing the trajectory
        for x, y  in self.traj:
            cv2.circle(image, (x, y), 1, (120, 0, 0), 3)

        self.x1=self.x0+int(self.l1*np.cos(self.q1))
        self.y1=self.y0+int(self.l1*np.sin(self.q1))
        self.x2=self.x1+int(self.l1*np.cos(self.q1))+int(self.l2*np.cos(self.q2))
        self.y2=self.y1+int(self.l1*np.sin(self.q1))+int(self.l2*np.sin(self.q2))

        #Drawing the links of the robot
        cv2.line(image, (self.x0, self.y0),(self.x1,self.y1), (0, 0, 255), 1)      
        cv2.line(image, (self.x1,self.y1) , (int(self.x2),int(self.y2)), (0, 255, 0), 1)

    
        
        self.its +=1
        self.traj.append((int(self.x2), int(self.y2)))

        return image


obj = object()    

for i in range(1000):
    obj.calculations(35,145)
    if i % 1 == 0:
        obj.render(height= 600, pause = 10)

    

    
    
