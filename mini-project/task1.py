from render import Renderer
import cv2
import numpy as np
from scipy.optimize import fsolve
import time
import random

class RR(Renderer):
    def __init__(self, include_dynamics):
        super().__init__()
        self.l1=310
        self.l2=310
        self.m1=1
        self.m2=1
        self.q1=np.pi/2
        self.q2=np.pi/2
        self.x=250
        self.y=300
        self.g=9.81
        self.done = False
        if include_dynamics == 'n':
            self.include_dynamics = False
        else:
            self.include_dynamics = True
        self._q1=self.q1
        self._q2=self.q2
        self.path=[]
        self.qpath=[]
        self.info={}

    def points(self, radius):
        alpha = 2 * np.pi * random.random()
        radius = 10 * radius #* random.random()
        x = radius * np.cos(alpha)
        y = radius * np.sin(alpha)
        return x+200, y+200

    def get_path(self):
        for i in range(50):
            x,y=self.points(10)
            t=i*0.1
            theta = np.arccos((x**2 + y**2 - self.l1**2 - self.l2**2) / (2 * self.l1 * self.l2))
            self.q1 = np.arctan(y/x) - np.arctan((self.l1*np.sin(theta))/(self.l2*np.cos(theta)+self.l1))
            self.q2 = theta + self.q1
            self.path.append((x,y,t))
            self.qpath.append((self.q1,self.q2))

    def update(self, steps=10):
        self.get_path()
        for i in range(len(self.path)):
            x=self.path[i][0]
            y=self.path[i][1]
            t=self.path[i][2]
            self.x=x
            self.y=y

            if i>2 and (self.x**2+self.y**2)<=(self.l1+self.l2)**2:
                q1_0=self.qpath[i-2][0]
                q1_1=self.qpath[i-1][0]
                q1_2=self.qpath[i][0]
                q2_0=self.qpath[i-2][1]
                q2_1=self.qpath[i-1][1]
                q2_2=self.qpath[i][1]
                t0=self.path[i-2][2]
                t2=self.path[i][2]

                dt=(t2-t0)/2

                q1_dot=(q1_2-q1_1)/dt
                q2_dot=(q2_2-q2_1)/dt
                q1_dotdot=(q1_0+q1_2-2*q1_1)/(2*t*t)
                q2_dotdot=(q2_0+q2_2-2*q2_1)/(2*t*t)

                q1=q1_2
                q2=q2_2
                q1_1=q1_dot
                q1_2=q1_dotdot
                q2_1=q2_dot
                q2_2=q2_dotdot

                m1, m2, l1, l2, g = self.m1, self.m2, self.l1, self.l2, self.g
                coeff1 = -0.5*m2*l1*l2*q2_1*(q2_1-q1_1)*np.sin(q2-q1) - 0.5*m2*l1*l2*q1_1*q2_1*np.sin(q2-q1) + 0.5*m1*g*l1*np.cos(q1) + m2*g*l1*np.cos(q1)
                coeff2 = -0.5*m2*l1*l2*q1_1*(q2_1-q1_1)*np.sin(q2-q1) + 0.5*m2*l1*l2*q1_1*q2_1*np.sin(q2-q1) + 0.5*m2*g*l2*np.cos(q2)
                coeff3 = 1/3*m1*l1**2 + m2*l1**2
                coeff4 = 0.5*m2*l1*l2*np.cos(q2-q1)
                coeff6 = 1/3*m2*l1**2 + 1/4*m2*l2**2
                coeff5 = 0.5*m2*l1*l2*np.cos(q2-q1)

                tau1=coeff3*q1_2 + coeff4*q2_2 + coeff1
                tau2=coeff5*q1_2 + coeff6*q2_2 + coeff2

                if self.include_dynamics:
                    self.info = {"tau1":tau1,"tau2":tau2, "q1":q1, "q2": q2}
                else:
                    self.info = {"q1":q1, "q2": q2}

                self.q1=q1
                self.q2=q2
            time.sleep(1/steps)
            self.render()
        self.done = True

    def step(self,steps=10):
        self.update(steps)

    def getInfo(self):
        return self.info

    def draw(self, image):
        line1=(int(250+self.l1*np.cos(-1*self.q1)),int(300+self.l1*np.sin(-1*self.q1)))
        cv2.line(image, (250,300), line1, (255,0,0), 1)
        line2=(int(line1[0]+self.l2*np.cos(-self.q2)),int(line1[1]+self.l2*np.sin(-self.q2)))
        cv2.line(image, line1, line2, (0,255,0), 1)
        for x,y,t in self.path:
            cv2.circle(image, (int(x)+250,-int(y)+300), 1, (0,0,0),-1)
        return image
    

dynamics = input('Include dynamics? (Default is yes) y/n: ')
mybot=RR(dynamics)
while(True):
    cv2.namedWindow("window")
    mybot.step(10)
    mybot.render()
    if (mybot.done):
        cv2.waitKey(0)
        break