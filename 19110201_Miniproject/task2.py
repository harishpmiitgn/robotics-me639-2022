import numpy as np
import matplotlib.pyplot as plt
import scipy
import math
from render import Renderer
import cv2

class Task2(Renderer):
    def __init__(self):
        super().__init__()
        self.l1=100
        self.l2=100
        self.x = 300
        self.y = 300
        self.q1 = 0
        self.q2 = 0
        #Wall orientation can be change below
        self.wallp1=(200,50)
        self.wallp2=(-100,100)
        #Force value can be set here
        self.force=10

        self.tau1=0
        self.tau2=0
        self.reach=self.l1+self.l2
        self.arr=[]
    
    def getInfo(self):
        info = {
            'q1' : round(self.q1*180/np.pi,2),
            'q2' : round(self.q2*180/np.pi,2),
            'tau1' : round(self.tau1,2),
            'tau2' : round(self.tau2,2)
        }
        return info

    def dynamics (self,its=100):
        if (self.wallp2[1]==self.wallp1[1]):
            flag=0
            y=self.wallp2[1]
            x=0
        else:
            sl=(self.wallp2[0]-self.wallp1[0])/(self.wallp2[1]-self.wallp1[1])
            x=((self.wallp1[0]/sl)-self.wallp1[1])/(sl+(1/sl))
            flag=1
            y=-sl*x
        theta = math.acos((x**2+y**2-self.l1**2-self.l2**2)/(2*self.l1*self.l2))
        q1 = math.atan2(y,x) - math.atan2((self.l2*math.sin(theta)),(self.l1+self.l2*math.cos(theta)))
        q2 = q1 + theta
        delta1=q1-self.q1
        delta2=q2-self.q2
        while(round((self.q1-q1)*180/np.pi,2)!=0 or round((self.q2-q2)*180/np.pi,2)!=0):
            self.q1+=(delta1)/its
            self.q2+=(delta2)/its
            self.render()
        if (flag==0):
            fy=self.force
            fx=0
        else:
            fy=self.force*np.sin(-np.arctan(sl))
            fx=self.force*np.cos(-np.arctan(sl))
        self.tau1 = fy*self.l1*np.cos(self.q1) - fx*self.l1*np.sin(self.q1)
        self.tau2 = fy*self.l2*np.cos(self.q2) - fx*self.l2*np.sin(self.q2)

    def draw(self,image):
        cv2.line(image,(self.wallp1[0]+300,300-self.wallp1[1]),(self.wallp2[0]+300,300-self.wallp2[1]),(0,0,0),1)
        cv2.line(image,(300,300),(int(300+self.l1*np.cos(-self.q1)),int(300+self.l1*np.sin(-self.q1))),(0,255,0),1)
        cv2.line(image,(int(300+self.l1*np.cos(-self.q1)),int(300+self.l1*np.sin(-self.q1))),(int(int(300+self.l1*np.cos(-self.q1))+self.l2*np.cos(-self.q2)),int(int(300+self.l1*np.sin(-self.q1))+self.l2*np.sin(-self.q2))),(0,0,255),1)
        if(len(self.arr)>200):
            self.arr=self.arr[len(self.arr)-200:]
        for p in self.arr:
            cv2.circle(image, p, 1, (0,0,0),-1)
        return image

anim=Task2()
while(True):
    anim.dynamics(400)
    anim.render()