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
        self.xp1=0
        self.yp1=0
        self.xp2=0
        self.yp2=0
        self.q1 = 0
        self.q2 = 0
        self.tau1=0
        self.tau2=0
        self.points1 = []
        self.points2 = []
        self.q1_values = []
        self.q2_values =[]
    
    def getInfo(self):
        info = {
            'q1' : round(self.q1*180/np.pi, 4),
            'q2' : round(self.q2*180/np.pi, 4),
            'tau1': round (self.tau1,4),
            'tau2':round(self.tau2,4),
            'x2' : round(self.x2,4),
            'y2' : round(self.y2,4)
        }
        return info

    def dynamics (self,xp1,yp1,xp2,yp2,x3,y3,f,dt):

        x3 = x3-300
        y3 = y3-300
        theta = math.acos((x3**2+y3**2-self.l1**2-self.l2**2)/(2*self.l1*self.l2))

        q1 = math.atan2(y3,x3) - math.atan2((self.l2*math.sin(theta)),(self.l1+self.l2*math.cos(theta)))

        q2 = q1 + theta

        diff_q1 = q1 - self.q1
        diff_q2 = q2 - self.q2
        
        self.xp1=xp1
        self.yp1=yp1
        self.xp2=xp2
        self.yp2=yp2

        while int(diff_q1*180/math.pi) != 0 or int(diff_q2*180/math.pi) != 0 :
            
            self.q1 = self.q1 + diff_q1/dt
            self.q2 = self.q2 + diff_q2/dt

            diff_q1 = q1 - self.q1
            diff_q2 = q2 - self.q2
            
            # print(self.l1*np.cos(q1)+self.l2*np.cos(q2),self.l1*np.cos(self.q1)+self.l2*np.cos(self.q2))
            x1 =self.l1*math.cos(self.q1)
            self.x1 = x1+300

            y1 = self.l1*math.sin(self.q1)
            self.y1 = y1+300

            #self.points1.append((self.x1,self.y1))

            x2 = self.l1*math.cos(self.q1)+self.l2*math.cos(self.q2)
            self.x2 = x2+300

            y2 = self.l1*math.sin(self.q1)+self.l2*math.sin(self.q2)
            self.y2 = y2+300

            #self.points1.append((self.x2,self.y2))
            self.render()

        
        slope_angle = math.atan2((self.yp2-self.yp1),(self.xp2-self.xp1))
        Fx = f*math.cos(slope_angle)
        Fy = f*math.sin(slope_angle)

        self.tau1 = (-self.l1*math.sin(q1)*Fx) + (self.l1*math.cos(q1)*Fy)
        self.tau2 = (-self.l2*math.sin(q2)*Fx) + (self.l2*math.cos(q2)*Fy)
        

    def draw(self,image):
        cv2.line(image,(300,300),(int(self.x1),int(self.y1)),(255,255,0),1)
        cv2.line(image,(int(self.x1),int(self.y1)),(int(self.x2),int(self.y2)),(0,0,255),1)
        cv2.circle(image,(int(self.x2),int(self.y2)),1,(0,0,0),2)
        cv2.line(image,(self.xp1,self.yp1),(self.xp2,self.yp2),(0,255,0),1)
        # for x1,y1 in self.points1:
        #     cv2.line(image,(300,300),(int(x1),int(y1)),(255,255,0),1)
        #     for x2,y2 in self.points2:
        #         cv2.line(image,(int(x1),int(y1)),(int(x2),int(y2)),(0,0,255),1)
        #         cv2.circle(image,(int(x2),int(y2)),1,(0,0,0),2)
        # cv2.line(image,(self.xp1,self.yp1),(self.xp2,self.yp2),(0,255,0),1)
        return image


anim= T1(recordLocation='t2.mp4v')
for i in range(80000):
   
#here I have taken a line passing through 300,200 and 200,300 and the point of contact is taken as 250,250 and force to be 100
#also specify the magnitude of force xp1,yp1,xp2,yp2,x3,y3,force,time step

    anim.dynamics(300,200,200,300,250,250,100,100)
    if i % 10==0:
        anim.render(height= 600, pause = 10) 