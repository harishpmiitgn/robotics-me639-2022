import numpy as np
import math
import matplotlib.pyplot as plt
import cv2
import scipy
from render import Renderer

class Task3(Renderer):
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
        self.m=1
        self.vx=0
        self.vy=0
        self.reach=self.l1+self.l2

        #Change following parameters only
        #Mean position of the Spring:
        self.x0 = 50
        self.y0 = 50
        #Initial position of the end affector
        self.xi=60
        self.yi=100
        #Stiffness of the spring
        self.k=10
        #Damping factor
        self.dampingfactor=0.45

        #Do not change the following variables
        self.x_curr=self.x0
        self.y_curr=self.y0
        self.x_prev=self.x0
        self.y_prev=self.y0
        self.tau1 =0
        self.tau2 = 0
        self.q1_dot=0
        self.q1_dot_dot =0
        self.q2_dot=0
        self.q2_dot_dot=0
        self.q1_values =[0]
        self.q2_values =[0]
        self.points1 = []
        self.points2 = []

    def getInfo(self):
        info = {
            'q1 (in degrees)' : round(self.q1*180/np.pi, 2),
            'q2 (in degrees)' : round(self.q2*180/np.pi, 2),
            'vx' : round(self.vx,2),
            'vy' : round(self.vy,2),
            'k'  : self.k,
            'damping factor' : self.dampingfactor,
            'tau_1 (will become constant if damped for enough time)' : round(self.tau1,2),
            'tau_2 (will become constant if damped for enough time)' : round(self.tau2,2),
            # 'q1.' : round(self.q1_dot,2),
            # 'q2.' : round(self.q2_dot,2),
            # 'q1..': round(self.q1_dot_dot,2),
            # 'q2..': round(self.q2_dot_dot,2)
        }
        return info

    def dynamics (self):
        if (self.xi**2+self.yi**2>self.reach**2) or ((2*self.x0-self.xi)**2+(2*self.x0-self.xi)**2>self.reach**2):
            print("There is a point out of reach on the trajectory")
            return
        self.x_prev=self.x_curr
        self.y_prev=self.y_curr
        x0=self.x0
        y0=self.y0
        xi=self.xi
        yi=self.yi
        x = x0+(xi-x0)*np.cos(math.sqrt(self.k/self.m)*self.i/100)*math.e**(-self.dampingfactor*self.i/100)
        y = y0+(yi-y0)*np.cos(math.sqrt(self.k/self.m)*self.i/100)*math.e**(-self.dampingfactor*self.i/100)
        theta = math.acos((x**2+y**2-self.l1**2-self.l2**2)/(2*self.l1*self.l2))
        q1 = math.atan2(y,x) - math.atan2((self.l2*math.sin(theta)),(self.l1+self.l2*math.cos(theta)))
        self.q1 = q1
        self.q1_values.append(self.q1)
        q2 = q1 + theta
        self.q2 = q2
        self.q2_values.append(self.q2)
        self.x_curr = x
        self.y_curr = y
        if (self.i==0):
            self.vx=0
            self.vy=0
        else:
            self.vx=(self.x_curr-self.x_prev)/(1/100)
            self.vy=(self.y_curr-self.y_prev)/(1/100)
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
        cv2.circle(image, (self.x0+300,-self.y0+300), 1, (0,0,0),-1)
        cv2.circle(image, (int(int(300+self.l1*np.cos(-self.q1))+self.l2*np.cos(-self.q2)),int(int(300+self.l1*np.sin(-self.q1))+self.l2*np.sin(-self.q2))), 1, (0,0,0),-1)
        return image


anim= Task3()
while(True):
    anim.dynamics()
    anim.render(height= 600, pause = 10) 