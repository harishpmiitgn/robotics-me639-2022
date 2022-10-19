import numpy as np
import matplotlib.pyplot as plt
import math
import scipy
import cv2
import pandas as pd


from render import Renderer
# Renderer is a class(package) which helps to plot in cv2
class object(Renderer):
    def __init__(self, recordLocation = None):
        super().__init__(recordLocation=recordLocation)
        self.q1 = 0     # q1 is the angle made by the first link of manipulator with x axis
        self.q2 = 0     # q2 is the angle made by the second link of manipulator with x axis
        self.theta = 0  # theta is the angle made by the link 2 w.r.t to link 1
        self.l1 = 150   # length of the first link
        self.l2 = 200   # length of the second link
        self.workspace = []
        self.points = []
    
    def plot(self):
        data = np.array(self.workspace)
        plt.scatter(data[:,0],data[:,1])
        plt.show()
        return data
        
    def step(self):
        self.q1 = [i for i in range(35,145,1)]
        self.q2 = [i for i in range(35,145,1)]
        for i in range(len(self.q1)):
            for j in range(len(self.q2)):
                # (x1,y1) is the coordinate of the joint 1
                self.x1 = 300 + self.l1*math.cos(self.q1[i]*math.pi/180)
                self.y1 = 100 + self.l1*math.sin(self.q1[i]*math.pi/180)
        
                # (x2,y2) is the coordinate of the joint 2
                self.x2 = self.x1+self.l2*math.cos(self.q2[j]*math.pi/180)
                self.y2 = self.y1+self.l2*math.sin(self.q2[j]*math.pi/180)
                self.workspace.append((self.x2,self.y2))
                self.points.append((self.x1,self.y1,self.x2,self.y2))
        
    

anim = object(recordLocation ='anim.mp4')
anim.step()
#anim.render(height=600,pause=1)
print(anim.plot())