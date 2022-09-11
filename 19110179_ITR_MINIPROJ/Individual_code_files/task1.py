#assuming no effect of gravity

import numpy as np
import math
import matplotlib.pyplot as plt
import cv2
import scipy
from render import Renderer

class T1(Renderer):
    def __init__(self, recordLocation=None):
        super().__init__(recordLocation=recordLocation)
        self.i = 0
        self.m1=1
        self.m2=1
        self.l1=100
        self.l2=100
        self.x1 = 400
        self.y1 = 300
        self.x2 = 500
        self.y2 = 300
        self.q1 = 0
        self.q2 = 0
        self.points1 = set()
        self.points2 = set()
    
    def getInfo(self):
        info = {
            'q1' : round(self.q1*180/np.pi, 4),
            'q2' : round(self.q2*180/np.pi, 4)
        }
        return info

    def dynamics (self):

        #enter the trajectory here given here is an ellipse
        x = 50*np.sin(self.i/500)
        y = 20*np.cos(self.i/500)+10

        # print(x,y)

        if (x**2+y**2<=(self.l1+self.l2)**2) and (x**2+y**2>=(self.l1-self.l2)**2):
            theta = math.acos((x**2+y**2-self.l1**2-self.l2**2)/(2*self.l1*self.l2))

            q1 = math.atan2(y,x) - math.atan2((self.l2*math.sin(theta)),(self.l1+self.l2*math.cos(theta)))
            self.q1 = q1

            q2 = q1 + theta
            self.q2 = q2
        
            x1 =self.l1*math.cos(self.q1)
            self.x1 = x1+300

            y1 = self.l1*math.sin(self.q1)
            self.y1 = y1+300

            x2 = self.l1*math.cos(self.q1)+self.l2*math.cos(self.q2)
            self.x2 = x2+300

            y2 = self.l1*math.sin(self.q1)+self.l2*math.sin(self.q2)
            self.y2 = y2+300

            self.points2.add((self.x2,self.y2))
            # print(self.x1,self.y1,self.x2,self.y2)
            
            self.i+=1
        
        else:
            print("enter valid trajectory")

    def draw(self,image):
        cv2.line(image,(300,300),(int(self.x1),int(self.y1)),(0,255,0),1)
        cv2.line(image,(int(self.x1),int(self.y1)),(int(self.x2),int(self.y2)),(255,0,0),1)
        for x,y in self.points2:
            cv2.circle(image,(int(x),int(y)),1,(0,0,255),1)

        return image


anim= T1(recordLocation='t1.mp4v')
for i in range(8000):
    anim.dynamics()
    if i % 10==0:
        anim.render(height= 600, pause = 10) 