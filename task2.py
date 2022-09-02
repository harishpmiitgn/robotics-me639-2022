import math
import numpy as np
import matplotlib.pyplot as plt
import cv2
import scipy
from render import Renderer
import pandas as pd
import xlsxwriter


class object(Renderer):
    # Initializing the variables.
    def __init__(self, recordLocation=None):
        super().__init__(recordLocation=recordLocation)
        self.its=0
        self.l1=225 # Length of link 1
        self.l2=226  # Length of link 2
        self.theta=0
        self.q1=0    # Angle made by link 1 with horizontal
    
        self.q2=0     # Angle made by link 2 with horizontal
        self.fx=5    # Force applied on the wall in x-direction
        self.fy=5   # Force applied on the wall in y-direction
        self.tau1=0  # Iniliazing the torques
        self.tau2=0

        
       
        self.points=set()

    def getInfo(self):
        # Printing the required variables during simulation.
        info = {
            'q1' : round(self.q1, 4),
            'q2' : round(self.q2, 4),
            'x'  : round(self.x,  4),
            'y'  : round(self.y,  4),
            'tau1': round(self.tau1, 4),
            'tau2' : round(self.tau2, 4),
            'fx'  : round(self.fx, 4),
            'fy'  : round(self.fy, 4)
         }
        return info

    def plot(self):
        plt.plot(1,1)
        plt.show()
        # Just using the plot function because due to some internal issue animation window was turning off after sucessful 
        # exhibition of the code.
        # Although this plot is of no use.


    def step(self,u,v):
        # Incrementing the variables according to provided excel file.
        self.j=0
        while self.j<len(u):
            self.x = u[self.j]
            self.y= v[self.j]
            self.theta=math.acos(((self.x-50)**2+(self.y-500)**2-(self.l1)**2-(self.l2)**2)/(2*self.l1*self.l2))
            
            self.q1=math.atan((self.y-500)/(self.x-50))-math.atan((self.l2*math.sin(self.theta))/(self.l1+(self.l2*math.cos(self.theta))))
            self.q2=(self.q1+self.theta)


            self.a=50 # Starting point of the link 1, x-coordinate
            self.b=500 # Starting point of the link 1, y-coordinate
            self.c=(self.a)+(self.l1*math.cos(self.q1))   # Starting point of the link 2, x-coordinate
            self.d=(self.b)+(self.l1*math.sin(self.q1))    # Starting point of the link 2, y-coordinate
            self.x1=(self.c)+(self.l2*math.cos(self.q2))    # ending point of the link 2, x-coordinate
            self.y1=(self.d)+(self.l2*math.sin(self.q2))      # ending point of the link 2, y-coordinate
            self.tau1=((self.l1)*(math.cos(self.q1))*(self.fy))/1000-((self.l1)*(math.sin(self.q1))*(self.fx))/1000
            self.tau2=((self.l2)*(math.cos(self.q2))*(self.fy))/1000-((self.l2)*(math.sin(self.q2))*(self.fx))/1000

            
            #self.its += 1
            
            self.points.add((self.c,self.d,int(self.x),int(self.y),int(self.x1),(self.y1),(self.tau1),(self.tau2)))
            
            self.j=self.j+1
            anim.render(height=600,pause=100)


    def draw(self,image):
        # Showing the animation.
        cv2.line(image,(50,500),(int(self.c),int(self.d)),(0,255,0),1)
        cv2.line(image,(int(self.c),int(self.d)),(int(self.x1),int(self.y1)),(0,0,255),1)
        cv2.circle(image,(int(self.x1),int(self.y1)),3,(255,0,),-1)
            
    
        

        return image


anim=object(recordLocation='anim.mp4')
# Reaching the location of wall with initial co-ordinates as (200,200)
u=[]
v=[]
n=100
wall=list(map(int,input('Enter the x and y co-ordinate of wall where you want to apply force: ').split(' ')))
dtx=(wall[0]-200)/n
dty=(wall[1]-200)/n
for i in range(n+1):
    u.append(200+(i*dtx))
    v.append(200+(i*dty))

anim.step(u,v)
anim.plot()




