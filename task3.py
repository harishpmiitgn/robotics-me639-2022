import math
import numpy as np
import matplotlib.pyplot as plt
import cv2
import scipy
from render import Renderer
import pandas as pd
import xlsxwriter


class object(Renderer):
    def __init__(self, recordLocation=None):
        #initializing the variables
        super().__init__(recordLocation=recordLocation)
        self.its=0


        self.l1=225 # Length of link 1
        self.l2=226  # Length of link 2
        self.theta=0 # angle made by link 2 w.r.t link 1
        self.q1=0    # Angle made by link 1 with horizontal
    
        self.q2=0     # Angle made by link 2 with horizontal
        
    
        self.m1=8  # mass of link 1
        self.m2=5 # mass of link 2
    
        self.g=10 # accerlation due to gravity
        self.k=30 # spring constant
        self.x_dot=1000/n # Considering the speed of end-effector as constant.
        self.y_dot=1000/n # Consider speed in both direction is same

        
       
        self.points=set()

    def getInfo(self):
        # Printing the required variables during simulation.
        info = {
            'q1' : round(self.q1, 4),
            'q2' : round(self.q2, 4),
            'x'  : round(self.x,  4),
            'y'  : round(self.y,  4),
            'ts1': round(self.ts1, 4),
            'ts2' : round(self.ts2, 4),
            't1'  : round(self.t1,4),  
            't2' : round(self.t2,4),
            'fx' : round(self.fx, 4),
            'fy' : round(self.fy, 4)   
            }
        return info

    def plot(self):
        plt.plot([1,2,3,4,5],[4,6,9,7,3])
        plt.show()
        # Just using the plot function because due to some internal issue animation window was turning off after sucessful 
        # exhibition of the code.
        # Although this plot is of no use.
         




    def step(self,u,v):
        # Incrementing the variables according to provided excel file.
        self.j=0
        while self.j < len(u):
            self.x = u[self.j]
            self.y = v[self.j]
            self.theta=math.acos(((self.x-50)**2+(self.y-500)**2-(self.l1)**2-(self.l2)**2)/(2*self.l1*self.l2))
        
            self.q1=math.atan((self.y-500)/(self.x-50))-math.atan((self.l2*math.sin(self.theta))/(self.l1+(self.l2*math.cos(self.theta))))
            self.q2=(self.q1+self.theta)
            self.fx = self.k*(self.l1*math.cos(self.q1)+self.l2*math.cos(self.q2) - self.x)/1000 
            self.fy = self.k*(self.l1*math.sin(self.q1)+self.l2*math.sin(self.q2) - self.y)/1000
        

            self.a=50 # Starting point of the link 1, x-coordinate
            self.b=500 # Starting point of the link 1, y-coordinate
            self.c=(self.a)+(self.l1*math.cos(self.q1))   # Starting point of the link 2, x-coordinate
            self.d=(self.b)+(self.l1*math.sin(self.q1))    # Starting point of the link 2, y-coordinate
            self.x1=(self.c)+(self.l2*math.cos(self.q2))    # ending point of the link 2, x-coordinate
            self.y1=(self.d)+(self.l2*math.sin(self.q2))      # ending point of the link 2, y-coordinate

            # q1_dot and q2_dot are the rate of change of angle at joint 1 and joint 2 respectively
            self.q1_dot = (self.l2*math.cos(self.q2)*self.x_dot + self.l2*math.sin(self.q2)*self.y_dot)/(self.l1*self.l2*math.sin(self.q2-self.q1))
            self.q2_dot = (-1*self.l1*math.cos(self.q1)*self.x_dot - self.l1*math.sin(self.q1)*self.y_dot)/(self.l1*self.l2*math.sin(self.q2-self.q1))
            
            # q1_ddot and q2_ddot are the rate of change of angular velocit at joint 1 and at joint 2 respectively
            self.q2_ddot = ((self.q1_dot)**2*self.l1 + (self.q2_dot)**2*math.cos(self.q2-self.q1)*self.l2)/self.l2*math.sin(self.q1-self.q2)
            self.q1_ddot = -1*(self.l1*math.cos(self.q1)*(self.q1_dot)**2 + self.l2*math.cos(self.q2)*(self.q2_dot)**2 +self.l2*math.sin(self.q2)*self.q2_ddot)/(self.l1*math.sin(self.q1))

            # Torque at joint 1 due to fx
            self.ts1=((self.l1)*(math.cos(self.q1))*(self.fy))/1000-((self.l1)*(math.sin(self.q1))*(self.fx))/1000
             # Torque at joint 1 due to fy
            self.ts2=((self.l2)*(math.cos(self.q2))*(self.fy))/1000-((self.l2)*(math.sin(self.q2))*(self.fx))/1000
            # Torque 1 due to dynamic effect
            self.t1 = ((1/3*self.m1+self.m2)*((self.l1)**2)*self.q1_ddot+1/2*self.m2*self.l1*self.l2*self.q2_ddot*math.cos(self.q2-self.q1)-1/2*self.m2*self.l1*self.l2*self.q2_dot*(self.q2_dot-self.q1_dot)*math.sin(self.q2-self.q1))/1000000 + ((1/2*self.m1+self.m2)*self.g*self.l1*math.cos(self.q1))/1000
            #Torque 2 due to dynamic effect
            self.t2 = ((1/3*self.m2+1/4*self.m2)*((self.l2)**2)*self.q2_ddot + 1/2*self.m2*self.l1*self.l2*self.q1_ddot*math.cos(self.q2-self.q1)-1/2*self.m2*self.l1*self.l2*self.q1_dot*(self.q2_dot-self.q1_dot)*math.sin(self.q2-self.q1))/1000000 + (1/2*self.m2*self.g*self.l2*math.sin(self.q2))/1000
            
            
            
            self.total_torque1=self.ts1+self.t1 # Total torque provided at joint 1

            self.total_torque2=self.ts2+self.t1 # Total torque provided at joint 2
    
        
            self.points.add((self.c,self.d,int(self.x),int(self.y),int(self.x1),(self.y1),(self.ts1),(self.ts2)))
         
            self.j=self.j+1
            anim.render(height=600,pause=10)

    def draw(self,image):
         # Showing the animation.
        cv2.line(image,(50,500),(int(self.c),int(self.d)),(0,255,0),1)
        cv2.line(image,(int(self.c),int(self.d)),(int(self.x1),int(self.y1)),(0,0,255),1)
        cv2.circle(image,(int(self.x1),int(self.y1)),3,(255,0,),-1)
        return image


# Here, I am creating a list of arrays so that end-eefector can behave like a virtual spring.
# List u contains the x-coordinates of the path followed by the end effector.
# List v contains the y-coordinates of the path followed by the end effector.   
speed=int(input('Enter the speed of oscillation: '))
# If the speed is greater than 10. It will automatically consider 10, as it is the maximum speed at which end effector can move.

  
u=[]
u1=[]
u2=[]
v2=[]
v=[]
v1=[]
osil=list(map(int,input('Enter the mean position coordinates and extreme position co-ordinates, like 250 250 300 300 : ').split(' ')))
# Mean position means the position at which manipulator behaves as virtual spring
# Extreme position means the final position at which we are taking end-effector after applying some force. 
if int(1000/speed)>=100: # No of steps between one point to another. 
    n=int(1000/speed)
else:
    n=100

# Finally, n is the number of steps between two points. 
dtx=(osil[2]-osil[0])/n
dty=(osil[3]-osil[1])/n
for i in range(n+1):
    u1.append(osil[0]+(i*dtx))

u1.reverse()
u=u+u1

for i in range(n+1):
    u2.append(osil[0]-(i*dtx))
u=u+u2
u2.reverse()
u=u+u2
u1.reverse()
u=u+u1

for j in range(n+1):
    v1.append(osil[1]+(j*dty))

v1.reverse()
v=v+v1
for j in range(n+1):
    v2.append(osil[1]-(j*dty))
v=v+v2
v2.reverse()
v=v+v2
v1.reverse()
v=v+v1
# Finally, we had appended the full path in the lsit u and v


anim=object(recordLocation='anim.mp4')
for i in range(5): # Although this loop will run infinite time, ideally as there is no damping effect.
    # For our convention I am running this only 5 times.
    anim.step(u,v)
    


anim.plot()