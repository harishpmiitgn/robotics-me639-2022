import numpy as np
import matplotlib.pyplot as plt
import scipy
import math
from render import Renderer
import cv2

class Task4(Renderer):
    def __init__(self):
        super().__init__()
        self.l1=100
        self.l2=100
        self.x = 300
        self.y = 300
        self.q1 = np.pi/2
        self.q2 = np.pi/2
        self.arr=[]
        self.reach=self.l1+self.l2
        self.lim1=35*np.pi/180
        self.lim2=145*np.pi/180
        self.null=''

    
    def getInfo(self):
        info = {
            'q1' : round(self.q1*180/np.pi,2),
            'q2' : round(self.q2*180/np.pi,2),
            'Move the cursor within the figure' : self.null,
            'This region represents the workspace if the angles are limited between 35 and 145 degrees' : self.null
        }
        return info

    def dynamics (self,event,x,y,a,b):
        if event==cv2.EVENT_MOUSEMOVE:
            self.x=x-300
            self.y=300-y
            if (self.x**2+self.y**2>self.reach**2):
                print(f"Unreachable Point: ({self.x},{self.y})")
            else:
                theta = math.acos((self.x**2+self.y**2-self.l1**2-self.l2**2)/(2*self.l1*self.l2))
                q1 = math.atan2(self.y,self.x) - math.atan2((self.l2*math.sin(theta)),(self.l1+self.l2*math.cos(theta)))
                q2 = q1 + theta
                if (q1<self.lim1) or (q1>self.lim2) or (q2<self.lim1) or (q2>self.lim2):
                    print(f"Unreachable Point: ({self.x},{self.y})")
                else:   
                    self.q1 = q1
                    self.q2 = q2
                    point=(x,y)
                    self.arr.append(point)

    def step(self):
        cv2.setMouseCallback("window",self.dynamics)  

    def draw(self,image):
        cv2.line(image,(300,300),(int(300+self.l1*np.cos(-self.q1)),int(300+self.l1*np.sin(-self.q1))),(0,255,0),1)
        cv2.line(image,(int(300+self.l1*np.cos(-self.q1)),int(300+self.l1*np.sin(-self.q1))),(int(int(300+self.l1*np.cos(-self.q1))+self.l2*np.cos(-self.q2)),int(int(300+self.l1*np.sin(-self.q1))+self.l2*np.sin(-self.q2))),(0,0,255),1)
        for p in self.arr:
            cv2.circle(image, p, 1, (0,0,0),-1)
        cv2.ellipse(image,(300,300),(200,200),0,-35,-145, (0,0,0),1)
        c1=(int(300-100*math.cos(self.lim1)),int(300-100*math.sin(self.lim1)))
        cv2.ellipse(image,c1,(100,100),0,-35,-145, (0,0,0),1)
        c2=(int(300-100*math.cos(self.lim2)),int(300-100*math.sin(self.lim2)))
        cv2.ellipse(image,c2,(100,100),0,-35,-145, (0,0,0),1)
        return image

anim=Task4()
while(True):
    cv2.namedWindow("window")
    anim.step()
    anim.render()