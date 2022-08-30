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
        #enter spring constant by end mass ratio
        self.kbym = 1

        # self.x1 = 400
        # self.y1 = 300

        #enter the end effector coordinates the length of the bar will change

        self.x2 = 400
        self.y2 = 200

        #mean position about which spring motion will occur

        self.xm = 350
        self.ym = 300

        #for velocity display
        self.vx = 0
        self.vy = 0
        self.xlast = 0
        self.ylast = 0 

        self.q1 = 0
        self.q2 = 0
        self.tau1=0
        self.tau2=0
        self.q1_dot=0
        self.q1_dot_dot =0
        self.q2_dot=0
        self.q2_dot_dot=0
        self.points1 = []
        self.points2 = []
        self.q1_values =[0]
        self.q2_values =[0]
    
    def getInfo(self):
        info = {
            'q1' : round(self.q1*180/np.pi, 4),
            'q2' : round(self.q2*180/np.pi, 4),
            'vx' : round(self.vx,4),
            'vy' :round(self.vy,4),
            'tau1':round(self.tau1,4),
            'tau2':round(self.tau2,4)
        }
        return info

    def dynamics (self,dt):
        g=9.81
        # if (x**2+y**2<=(self.l1+self.l2)**2) and (x**2+y**2>=(self.l1-self.l2)**2):
        self.points2.append((self.x2,self.y2))
        self.xlast = self.x2
        self.ylast = self.y2
        x_mean = self.xm
        y_mean = self.ym
        x_endeff = self.points2[0][0]
        y_endeff = self.points2[0][1]

        # harmonics trajectory in x and y
        x = x_mean + (x_endeff-x_mean)*math.cos((self.kbym**0.5)*(self.i/500)) - 300
        y = y_mean + (y_endeff-y_mean)*math.cos((self.kbym**0.5)*(self.i/500)) - 300


        if (x**2+y**2<=(self.l1+self.l2)**2) and (x**2+y**2>=(self.l1-self.l2)**2):
            theta = math.acos((x**2+y**2-self.l1**2-self.l2**2)/(2*self.l1*self.l2))

            q1 = math.atan2(y,x) - math.atan2((self.l2*math.sin(theta)),(self.l1+self.l2*math.cos(theta)))
            self.q1 = q1
            self.q1_values.append(self.q1)

            q2 = q1 + theta
            self.q2 = q2
            self.q2_values.append(self.q2)
        
            x1 =self.l1*math.cos(self.q1)
            self.x1 = x1+300

            y1 = self.l1*math.sin(self.q1)
            self.y1 = y1+300

            x2 = self.l1*math.cos(self.q1)+self.l2*math.cos(self.q2)
            self.x2 = x2+300

            y2 = self.l1*math.sin(self.q1)+self.l2*math.sin(self.q2)
            self.y2 = y2+300

            q1_dot = (self.q1_values[self.i]-self.q1_values[self.i-1])/dt
            self.q1_dot=q1_dot
            q2_dot = (self.q2_values[self.i]-self.q2_values[self.i-1])/dt 
            self.q2_dot=q2_dot

            if self.i>1 :

                q1_dot_dot= ((self.q1_values[self.i]-(2*self.q1_values[self.i-1])+self.q1_values[self.i-1]))/(dt*dt)
                self.q1_dot_dot=q1_dot_dot
                q2_dot_dot= ((self.q2_values[self.i]-(2*self.q2_values[self.i-1])+self.q2_values[self.i-1]))/(dt*dt)
                self.q2_dot_dot=q2_dot_dot


            tau1 = (1/3*self.m1*self.l1**2*self.q1_dot_dot)-(self.m2*self.l1**2*self.q2_dot_dot)+(0.5*self.m2*self.l1*self.l2*self.q2_dot_dot*math.cos(theta))-(0.5*self.m2*self.l1*self.l2*self.q2_dot*(self.q2_dot-self.q1_dot)*math.sin(self.q2-self.q1))+(self.m1*g*0.5*self.l1*math.cos(self.q1))+(self.m2*g*self.l1*math.cos(self.q1))
            self.tau1 = tau1

            tau2 = (1/3*self.m2*self.l2**2*self.q2_dot_dot)-(0.25*self.m1*self.l2**2*self.q2_dot_dot)+(0.5*self.m2*self.l1*self.l2*self.q1_dot_dot*math.cos(theta))-(0.5*self.m2*self.l1*self.l2*self.q1_dot*(self.q2_dot-self.q1_dot)*math.sin(self.q2-self.q1))+(self.m2*g*0.5*self.l2*math.sin(self.q2))
            self.tau2 = tau2

            self.vx = (self.x2 - self.xlast)/dt
            self.vy = (self.y2 - self.ylast)/dt
            self.i+=1
        
        else:
            print("enter valid trajectory")

    def draw(self,image):
        cv2.line(image,(300,300),(int(self.x1),int(self.y1)),(0,255,0),1)
        cv2.line(image,(int(self.x1),int(self.y1)),(int(self.x2),int(self.y2)),(255,0,0),1)
        cv2.circle(image,(int(self.xm),int(self.ym)),1,(0,0,0),2)

        return image


anim= T1(recordLocation='t3.mp4v')
for i in range(8000):
    anim.dynamics(0.01)
    if i % 10==0:
        anim.render(height= 600, pause = 10) 