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
        self.points1 = []
        self.points2 = []
        self.q1_values = []
        self.q2_values =[]
    
    def getInfo(self):
        info = {
        }
        return info


    def workspace(self):
        for i  in range (35,146,5):
            self.q1 = i*math.pi/180
            self.q1_values.append(self.q1)
        
        for j in range (35,146,5):
            self.q2 = j*math.pi/180
            self.q2_values.append(self.q2)   

    def draw(self,image):
        for q1 in self.q1_values:
            x1 =self.l1*math.cos(-q1) +300

            y1 = self.l1*math.sin(-q1)+300

            cv2.line(image,(300,300),(int(x1),int(y1)),(255,255,255),1)

            for q2 in self.q2_values:

                x2 = self.l1*math.cos(-q1)+self.l2*math.cos(-q2) +300

                y2 = self.l1*math.sin(-q1)+self.l2*math.sin(-q2)+300

                self.points2.append((x2,y2))

                cv2.line(image,(int(x1),int(y1)),(int(x2),int(y2)),(255,255,255),1)

            for x,y in self.points2 :
                cv2.circle(image,(int(x),int(y)),1,(0,0,255),1)

        return image


anim= T1(recordLocation='t4.mp4v')
for i in range(1000):
    anim.workspace()
    if i % 10==0:
        anim.render(height= 600, pause = 10) 
        
