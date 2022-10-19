import math
import numpy as np
import matplotlib.pyplot as plt
import cv2
import scipy
from render import Renderer
import pandas as pd
import xlsxwriter

# The co-ordinate system provided in the render system is slightly different.
# The window size is approx 600*600
# The bootom co-ordiante is considered as (50,500)



user=input('Enter the excel file name: ') # Taking the excel file as a input which has x,y co-ordinates of end effector as a columns.
pf=pd.read_excel(user)
data=pf.to_numpy()  # Converting the excel file in list of arrays.



class object(Renderer):
    def __init__(self, traject, recordLocation=None):
        super().__init__(recordLocation=recordLocation) # Accesing the parent class Renderer
        # Initializing all the required variables.
        self.its=0
        self.l1=225 # Length of link 1
        self.l2=226  # Length of link 2
        self.theta=0
        self.q1=0    # Angle made by link 1 with horizontal
    
        self.q2=0     # Angle made by link 2 with horizontal
        self.traject=traject
        self.points=[]

        
       
    

    def getInfo(self):
        # Printing the required variables during simulation.
        info = {
            'q1' : round(self.q1, 4),
            'q2' : round(self.q2, 4),
            'x'  : round(self.x,  4),
            'y'  : round(self.y,  4),
         }
        return info

    def plot(self,data):
        # Plotting the desired end-effector points.
        plt.scatter(data[:,0],data[:,1]) 
            
        plt.show()
        





    def step(self):
        # Incrementing the variables according to provided excel file.
        
        self.x = self.traject[self.its,0]
        self.y = self.traject[self.its,1]
        self.theta=math.acos(((self.x-50)**2+(self.y-500)**2-(self.l1)**2-(self.l2)**2)/(2*self.l1*self.l2))
        
        self.q1=math.atan((self.y-500)/(self.x-50))-math.atan((self.l2*math.sin(self.theta))/(self.l1+(self.l2*math.cos(self.theta))))
        self.q2=(self.q1+self.theta)

        self.a=50 # Starting point of the link 1, x-coordinate
        self.b=500 # Starting point of the link 1, y-coordinate
        self.c=(self.a)+(self.l1*math.cos(self.q1))   # Starting point of the link 2, x-coordinate
        self.d=(self.b)+(self.l1*math.sin(self.q1))    # Starting point of the link 2, y-coordinate
        self.x1=(self.c)+(self.l2*math.cos(self.q2))    # ending point of the link 2, x-coordinate
        self.y1=(self.d)+(self.l2*math.sin(self.q2))      # ending point of the link 2, y-coordinate

        
        self.its += 1
        
        self.points.append((self.c,self.d,self.x1,self.y1))


    def draw(self,image):
        # Showing the animation.
        for c,d,x1,y1 in self.points:
            
            
            cv2.line(image,(50,500),(int(self.c),int(self.d)),(0,255,0),1)
            cv2.line(image,(int(self.c),int(self.d)),(int(self.x1),int(self.y1)),(0,0,255),1)
            cv2.circle(image,(int(x1),int(y1)),3,(255,0,),-1)
            
    
        

        return image

speed=int(input('Enter the speed of end-effector: '))
anim=object(data, recordLocation='anim.mp4')

for i in range(len(data[:,1])):
    anim.step()
    if int(100/speed)==0:
        anim.render(height=600,pause=1)
    else: 
        anim.render(height=600,pause=int(100/speed))
anim.plot(data)

