import numpy as np
import matplotlib.pyplot as plt
import scipy
import math
from render import Renderer
import cv2

class Task1_dynamics(Renderer):
    def __init__(self):
        super().__init__()
        self.i = 0
        self.m1=1
        self.m2=1
        self.g=9.81
        self.l1=100
        self.l2=100
        self.q1 = 0
        self.q2 = 0
        self.reach=self.l1+self.l2
        self.tau1 =0
        self.tau2 = 0
        self.q1_dot=0
        self.q1_dot_dot =0
        self.q2_dot=0
        self.q2_dot_dot=0
        self.q1_values =[0]
        self.q2_values =[0]
        self.p = []
    
    def getInfo(self):
        info = {
            'q1 (in degrees)' : round(self.q1*180/np.pi, 2),
            'q2 (in degrees)' : round(self.q2*180/np.pi, 2),
            'tau_1' : round(self.tau1,2),
            'tau_2' : round(self.tau2,2),
        }
        return info

    def dynamics (self):
        t=self.i/100
        #Change trajectory Here
        x = 50*np.cos(t) 
        y = 50*np.sin(t)+150
        if (x**2+y**2>self.reach**2):
            print("Trajectory goes out of reach")
            return
        self.p.append((x,y))
        theta = math.acos((x**2+y**2-self.l1**2-self.l2**2)/(2*self.l1*self.l2))
        q1 = math.atan2(y,x) - math.atan2((self.l2*math.sin(theta)),(self.l1+self.l2*math.cos(theta)))
        self.q1 = q1
        self.q1_values.append(self.q1)
        q2 = q1 + theta
        self.q2 = q2
        self.q2_values.append(self.q2)
        self.x_curr = x
        self.y_curr = y
        self.i+=1
        q1_dot = (self.q1_values[self.i]-self.q1_values[self.i-1])/(1/100)
        self.q1_dot=q1_dot
        q2_dot = (self.q2_values[self.i]-self.q2_values[self.i-1])/(1/100)
        self.q2_dot=q2_dot
        if self.i>1 :
            q1_dot_dot= ((self.q1_values[self.i]-(2*self.q1_values[self.i-1])+self.q1_values[self.i-1]))/((1/100)*(1/100))
            self.q1_dot_dot=q1_dot_dot
            q2_dot_dot= ((self.q2_values[self.i]-(2*self.q2_values[self.i-1])+self.q2_values[self.i-1]))/((1/100)*(1/100))
            self.q2_dot_dot=q2_dot_dot
        tau1 = (0.5*self.m2*self.l1*self.l2*self.q2_dot_dot*math.cos(theta))+(1/3*self.m1*self.l1**2*self.q1_dot_dot)-(self.m2*self.l1**2*self.q2_dot_dot)-(0.5*self.m2*self.l1*self.l2*self.q2_dot*(self.q2_dot-self.q1_dot)*math.sin(self.q2-self.q1))+(self.m1*self.g*0.5*self.l1*math.cos(self.q1))+(self.m2*self.g*self.l1*math.cos(self.q1))
        self.tau1 = tau1
        tau2 = (0.5*self.m2*self.l1*self.l2*self.q1_dot_dot*math.cos(theta))+(1/3*self.m2*self.l2**2*self.q2_dot_dot)-(0.25*self.m1*self.l2**2*self.q2_dot_dot)-(0.5*self.m2*self.l1*self.l2*self.q1_dot*(self.q2_dot-self.q1_dot)*math.sin(self.q2-self.q1))+(self.m2*self.g*0.5*self.l2*math.sin(self.q2))
        self.tau2 = tau2


    def draw(self,image):
        cv2.line(image,(300,300),(int(300+self.l1*np.cos(-self.q1)),int(300+self.l1*np.sin(-self.q1))),(0,255,0),1)
        cv2.line(image,(int(300+self.l1*np.cos(-self.q1)),int(300+self.l1*np.sin(-self.q1))),(int(int(300+self.l1*np.cos(-self.q1))+self.l2*np.cos(-self.q2)),int(int(300+self.l1*np.sin(-self.q1))+self.l2*np.sin(-self.q2))),(0,0,255),1)
        cv2.circle(image, (int(int(300+self.l1*np.cos(-self.q1))+self.l2*np.cos(-self.q2)),int(int(300+self.l1*np.sin(-self.q1))+self.l2*np.sin(-self.q2))), 1, (0,0,0),-1)
        for (x,y) in self.p:
            cv2.circle(image, (int(x)+300,300-int(y)),1,(0,0,0),-1)
        return image

anim=Task1_dynamics()
while(True):
    cv2.namedWindow("window")
    anim.dynamics()
    anim.render()